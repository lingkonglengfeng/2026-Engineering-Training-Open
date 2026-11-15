#include "match_control.h"
#include <stddef.h>

#define MATCH_LEAVE_TIMEOUT_MS (15000U)
#define MATCH_NAV_TIMEOUT_MS   (120000U)
static void MatchControl_ClearMilestones(match_control_t *flow)
{
    unsigned i;flow->milestone_mask=0U;
    for(i=0U;i<9U;i++)flow->milestone_ms[i]=0U;
}

static void MatchControl_Notify(match_control_t *flow)
{
    if(flow->state==MATCH_FINISHED && flow->run_timer_started && !flow->run_timer_finished) {
        flow->run_elapsed_ms=(uint32_t)(flow->entered_tick-flow->run_started_tick);
        flow->run_timer_finished=1U;
    }
    if (flow->ports.changed != NULL) {
        flow->ports.changed(flow->state, flow->checkpoint);
    }
}

static void MatchControl_Schedule(match_control_t *flow, match_state_t state,
                                  uint32_t now)
{
    flow->state = state;
    flow->entered_tick = now;
    if(state==MATCH_LEAVING) {
        MatchControl_ClearMilestones(flow);
        flow->run_started_tick=now;flow->run_elapsed_ms=0U;
        flow->run_timer_started=1U;flow->run_timer_finished=0U;
    }
    flow->next_request++;
    if (flow->next_request == 0U) { flow->next_request++; }
    flow->active_request = flow->next_request;
    flow->pending = 1U;
    MatchControl_Notify(flow);
}

void MatchControl_Init(match_control_t *flow, const match_ports_t *ports)
{
    flow->ports = *ports;
    flow->state = MATCH_IDLE;
    flow->checkpoint = MATCH_QR;
    flow->fault_reason = MATCH_FAULT_NONE;
    flow->zone = 0U;
    flow->pending = 0U;
    flow->next_request = 0U;
    flow->first_round_only = 0U;
    flow->automatic = 0U;
    flow->qr_required = 0U;
    flow->active_request = 0U;
    flow->entered_tick = 0U;
    flow->run_started_tick=flow->run_elapsed_ms=0U;
    flow->run_timer_started=flow->run_timer_finished=0U;
    MatchControl_ClearMilestones(flow);
}

uint8_t MatchControl_IsActive(const match_control_t *flow)
{
    return (uint8_t)((flow->state == MATCH_LEAVING) ||
                     (flow->state == MATCH_MOVING) ||
                     (flow->state == MATCH_WAIT_OK) || (flow->state==MATCH_PREPARING) ||
                     (flow->state==MATCH_WAIT_START_OK));
}

uint8_t MatchControl_Prepare(match_control_t *flow,uint8_t zone,uint32_t now)
{
    if(flow->state!=MATCH_IDLE || zone>2U)return 0U;
    flow->zone=zone;flow->checkpoint=MATCH_QR;
    MatchControl_Schedule(flow,MATCH_PREPARING,now);return 1U;
}

uint8_t MatchControl_Start(match_control_t *flow, uint8_t zone, uint32_t now)
{
    if ((flow->state != MATCH_IDLE) || ((zone != 1U) && (zone != 2U))) {
        return 0U;
    }
    flow->zone = zone;
    flow->checkpoint = MATCH_QR;
    MatchControl_Schedule(flow, MATCH_LEAVING, now);
    return 1U;
}

uint8_t MatchControl_Ok(match_control_t *flow, uint32_t received_tick)
{
    if(flow->state==MATCH_WAIT_START_OK) {
        if((int32_t)(received_tick-flow->entered_tick)<=0)return 0U;
        MatchControl_Schedule(flow,MATCH_LEAVING,received_tick);return 1U;
    }
    /* A confirmation received while travelling must never carry into a stop. */
    if ((flow->state != MATCH_WAIT_OK) ||
        ((int32_t)(received_tick - flow->entered_tick) <= 0)) {
        return 0U;
    }
    if(flow->first_round_only && flow->checkpoint==MATCH_TEMP1) {
        flow->state=MATCH_FINISHED;flow->active_request=0U;flow->pending=0U;
        flow->entered_tick=received_tick;MatchControl_Notify(flow);return 1U;
    }
    flow->checkpoint = (match_checkpoint_t)((unsigned int)flow->checkpoint + 1U);
    MatchControl_Schedule(flow, MATCH_MOVING, received_tick);
    return 1U;
}

static void MatchControl_Fail(match_control_t *flow, match_fault_reason_t reason)
{
    flow->fault_reason = reason;
    flow->active_request = 0U;
    flow->pending = 0U;
    flow->state = MATCH_FAULT;
    if (flow->ports.cancel != NULL) { flow->ports.cancel(); }
    MatchControl_Notify(flow);
}

void MatchControl_Abort(match_control_t *flow)
{
    MatchControl_Fail(flow, MATCH_FAULT_EXTERNAL);
}

void MatchControl_PauseTimeout(match_control_t *flow, uint32_t duration, uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - flow->entered_tick);
    /* A stage can be restarted/completed earlier in this same poll. Only
     * exclude the part of the poll gap that belongs to the current stage.
     * Also caps overlapping arm/obstacle waits at the current tick. */
    if (duration > elapsed) { duration = elapsed; }
    flow->entered_tick += duration;
}

void MatchControl_Reset(match_control_t *flow)
{
    flow->active_request = 0U;
    flow->pending = 0U;
    if (flow->ports.cancel != NULL) { flow->ports.cancel(); }
    /* Preserve request numbering so late results cannot finish a new run. */
    flow->state = MATCH_IDLE;
    flow->fault_reason = MATCH_FAULT_NONE;
    flow->checkpoint = MATCH_QR;
    flow->zone = 0U;
    flow->run_started_tick=flow->run_elapsed_ms=0U;
    flow->run_timer_started=flow->run_timer_finished=0U;
    MatchControl_ClearMilestones(flow);
    MatchControl_Notify(flow);
}

void MatchControl_Result(match_control_t *flow, uint32_t request,
                         uint8_t success, uint32_t now)
{
    if ((request == 0U) || (request != flow->active_request) ||
        (flow->pending != 0U) ||
        ((flow->state != MATCH_LEAVING) && (flow->state != MATCH_MOVING) && (flow->state!=MATCH_PREPARING))) {
        return;
    }
    if (success == 0U) { MatchControl_Fail(flow, MATCH_FAULT_RESULT); return; }
    flow->active_request = 0U;
    flow->pending = 0U;
    if(flow->state==MATCH_PREPARING) {
        flow->entered_tick=now;flow->state=MATCH_WAIT_START_OK;MatchControl_Notify(flow);return;
    }
    if(flow->run_timer_started) {
        unsigned stage=flow->state==MATCH_LEAVING?0U:(unsigned)flow->checkpoint+1U;
        flow->milestone_ms[stage]=(uint32_t)(now-flow->run_started_tick);
        flow->milestone_mask|=(uint16_t)(1U<<stage);
    }
    if (flow->state == MATCH_LEAVING) {
        if(flow->automatic && !flow->qr_required)flow->checkpoint=MATCH_RAW1;
        MatchControl_Schedule(flow, MATCH_MOVING, now);
        return;
    }
    if(flow->automatic) {
        if(flow->checkpoint==MATCH_RETURN || (flow->first_round_only && flow->checkpoint==MATCH_TEMP1)) {
            flow->entered_tick=now;flow->state=MATCH_FINISHED;MatchControl_Notify(flow);return;
        }
        flow->checkpoint=(match_checkpoint_t)((unsigned int)flow->checkpoint+1U);
        MatchControl_Schedule(flow,MATCH_MOVING,now);return;
    }
    flow->entered_tick = now;
    flow->state = (flow->checkpoint == MATCH_RETURN) ?
                   MATCH_FINISHED : MATCH_WAIT_OK;
    MatchControl_Notify(flow);
}

void MatchControl_Service(match_control_t *flow, uint32_t now)
{
    static const match_motion_t motions[8] = {
        MATCH_MOTION_QR, MATCH_MOTION_RAW, MATCH_MOTION_ROUGH, MATCH_MOTION_TEMP,
        MATCH_MOTION_RAW, MATCH_MOTION_ROUGH, MATCH_MOTION_TEMP, MATCH_MOTION_RETURN
    };
    uint32_t timeout;
    match_motion_t motion;
    if ((flow->state != MATCH_LEAVING) && (flow->state != MATCH_MOVING) && (flow->state!=MATCH_PREPARING)) { return; }
    timeout = (flow->state == MATCH_LEAVING) ?
                MATCH_LEAVE_TIMEOUT_MS : MATCH_NAV_TIMEOUT_MS;
    if ((uint32_t)(now - flow->entered_tick) >= timeout) {
        MatchControl_Fail(flow, MATCH_FAULT_TIMEOUT);
        return;
    }
    if (flow->pending != 0U) {
        uint8_t result;
        uint32_t request=flow->active_request;
        motion = (flow->state==MATCH_PREPARING)?MATCH_MOTION_PREPARE:
                   ((flow->state == MATCH_LEAVING) ? MATCH_MOTION_LEAVE : motions[flow->checkpoint]);
        flow->pending = 0U;
        result=(flow->ports.start!=NULL)?flow->ports.start(motion,flow->zone,request):MATCH_START_REJECTED;
        /* A synchronous completion/reset may already have changed requests. */
        if(flow->active_request==request &&
           (flow->state==MATCH_LEAVING || flow->state==MATCH_MOVING || flow->state==MATCH_PREPARING)) {
            if(result==MATCH_START_BUSY)flow->pending=1U;
            else if(result!=MATCH_START_ACCEPTED)MatchControl_Fail(flow,MATCH_FAULT_START_REJECTED);
        }
    }
}

const char *MatchControl_CheckpointName(match_checkpoint_t checkpoint)
{
    static const char *const names[8] = {
        "QR", "RAW_BATCH1", "ROUGH_BATCH1", "TEMP_BATCH1",
        "RAW_BATCH2", "ROUGH_BATCH2", "TEMP_BATCH2", "RETURN_START"
    };
    return ((unsigned int)checkpoint < 8U) ? names[checkpoint] : "UNKNOWN";
}

const char *MatchControl_StateName(match_state_t state)
{
    static const char *const names[8] = {
        "IDLE", "LEAVING", "MOVING", "WAIT_OK", "FINISHED", "FAULT", "PREPARING", "WAIT_START_OK"
    };
    return ((unsigned int)state < 8U) ? names[state] : "UNKNOWN";
}

const char *MatchControl_FaultReasonName(match_fault_reason_t reason)
{
    static const char *const names[] = {
        "NONE", "EXTERNAL_ABORT", "ACTION_RESULT_FAILED", "STAGE_TIMEOUT", "START_REJECTED"
    };
    return ((unsigned int)reason < sizeof(names)/sizeof(names[0])) ? names[reason] : "UNKNOWN";
}
