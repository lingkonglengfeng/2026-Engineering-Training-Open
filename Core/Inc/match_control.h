#ifndef MATCH_CONTROL_H
#define MATCH_CONTROL_H

#include <stdint.h>

/* Launch-confirmed or serial-stepped flow; called only by the ZDT task. */
typedef enum {
    MATCH_IDLE, MATCH_LEAVING, MATCH_MOVING, MATCH_WAIT_OK,
    MATCH_FINISHED, MATCH_FAULT, MATCH_PREPARING, MATCH_WAIT_START_OK
} match_state_t;

typedef enum {
    MATCH_QR, MATCH_RAW1, MATCH_ROUGH1, MATCH_TEMP1,
    MATCH_RAW2, MATCH_ROUGH2, MATCH_TEMP2, MATCH_RETURN
} match_checkpoint_t;

typedef enum {
    MATCH_MOTION_LEAVE, MATCH_MOTION_QR, MATCH_MOTION_RAW,
    MATCH_MOTION_ROUGH, MATCH_MOTION_TEMP, MATCH_MOTION_RETURN, MATCH_MOTION_PREPARE
} match_motion_t;

/* start() must not initiate motion when returning BUSY. The same request
 * stays pending and is retried by Service without extending its timeout. */
typedef enum {
    MATCH_START_REJECTED = 0U, MATCH_START_ACCEPTED = 1U, MATCH_START_BUSY = 2U
} match_start_result_t;

typedef enum {
    MATCH_FAULT_NONE, MATCH_FAULT_EXTERNAL, MATCH_FAULT_RESULT,
    MATCH_FAULT_TIMEOUT, MATCH_FAULT_START_REJECTED
} match_fault_reason_t;

typedef struct {
    uint8_t (*start)(match_motion_t motion, uint8_t zone, uint32_t request);
    void (*cancel)(void);
    void (*changed)(match_state_t state, match_checkpoint_t checkpoint);
} match_ports_t;

typedef struct {
    match_ports_t ports;
    match_state_t state;
    match_checkpoint_t checkpoint;
    match_fault_reason_t fault_reason;
    uint8_t zone;
    uint8_t pending;
    uint8_t first_round_only;
    uint8_t automatic; /* Advance only on successful station results. */
    uint8_t qr_required; /* Scan before RAW; independent of automatic station progression. */
    uint8_t run_timer_started;
    uint8_t run_timer_finished;
    uint32_t next_request;
    uint32_t active_request;
    uint32_t entered_tick;
    uint32_t run_started_tick; /* LEAVING accepted; excludes preparation/human wait. */
    uint32_t run_elapsed_ms; /* Frozen at FINISHED; includes all in-run waits. */
    uint32_t milestone_ms[9]; /* LEAVE, then checkpoint+1; cumulative from launch. */
    uint16_t milestone_mask; /* Only successful completed stages are present. */
} match_control_t;

void MatchControl_Init(match_control_t *flow, const match_ports_t *ports);
uint8_t MatchControl_Start(match_control_t *flow, uint8_t zone, uint32_t now);
uint8_t MatchControl_Prepare(match_control_t *flow,uint8_t zone,uint32_t now);
uint8_t MatchControl_Ok(match_control_t *flow, uint32_t received_tick);
void MatchControl_Service(match_control_t *flow, uint32_t now);
/* Exclude only already elapsed waiting time; never move entered_tick past now. */
void MatchControl_PauseTimeout(match_control_t *flow, uint32_t duration, uint32_t now);
void MatchControl_Result(match_control_t *flow, uint32_t request,
                         uint8_t success, uint32_t now);
void MatchControl_Abort(match_control_t *flow);
void MatchControl_Reset(match_control_t *flow);
uint8_t MatchControl_IsActive(const match_control_t *flow);
const char *MatchControl_CheckpointName(match_checkpoint_t checkpoint);
const char *MatchControl_StateName(match_state_t state);
const char *MatchControl_FaultReasonName(match_fault_reason_t reason);

#endif
