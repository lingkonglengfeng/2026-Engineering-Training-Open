#include "arm_calibration.h"
#include "arm_speed.h"
#include "arm_servo.h"
#include "arm_start_pose.h"
#include "arm_material.h"
#include "arm_round1.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_task.h"
#include "fdcan.h"
#include "zdt_motor.h"

static arm_cal_config_t s_cfg;
static arm_cal_axis_status_t s_axes[2];
static arm_cal_state_t s_state;
static uint8_t s_inited;
static uint8_t s_active;
static uint8_t s_upper;
static uint8_t s_seek;
static uint8_t s_stall_samples;
static uint8_t s_settle_samples;
static uint32_t s_started;
static uint32_t s_motion_started;
static uint32_t s_last_query;
static arm_feedback_wait_t s_feedback_pause;
static volatile uint8_t s_feedback_holding;
static uint32_t s_flags_baseline;
static uint32_t s_position_baseline;
static int64_t s_settle_position;
static uint8_t s_top_mode,s_drop_record;
static uint8_t s_zero_record;
static float s_drop_cal_mm;
static double s_down_counts_per_mm;
static float s_top_drop_limit,s_top_min,s_top_max;
static double s_percent_span[2];
static float s_percent_target,s_percent_last;
static int16_t s_percent_rpm;
static uint8_t s_percent_ready;
static uint8_t s_percent_empty_lift;
static uint32_t s_percent_timeout;
static uint32_t s_percent_position_tick;
static float s_percent_speed_gain;
static float s_percent_rate_factor=1.0f;
static float s_percent_min,s_percent_max;
static uint8_t s_percent_stop_wait;
/* ID5 percentage jobs are native ABS jobs. ID6 legacy percentage control
 * remains separate from its existing native reach owners. */
static uint8_t s_lift_native_sent,s_lift_native_samples;
static int32_t s_lift_native_goal;
static uint16_t s_lift_native_speed;
static float s_lift_native_height;
static uint32_t s_lift_native_ack_fence,s_lift_native_ack_tick,s_lift_native_last_send,s_lift_native_ack_seen;
static uint8_t s_lift_native_attempts,s_lift_native_acked,s_lift_native_exhausted;
static uint32_t s_lift_progress_tick,s_lift_progress_seq;
static double s_lift_best_error;
#define LIFT_NATIVE_ACK_WAIT_MS (100U)
#define LIFT_NATIVE_ACK_ATTEMPTS (3U)
static int64_t s_lift_native_last;
static uint32_t s_lift_native_flags,s_lift_native_quiet_tick;
static arm_camera_state_t s_camera_state;
static uint32_t s_camera_started,s_camera_step_started,s_camera_dm_sequence,s_camera_dm_settle_tick;
static uint32_t s_camera_enable_position,s_camera_enable_flags;
static uint32_t s_camera_enable_sent;
static uint8_t s_camera_enable_attempts,s_camera_dm_prepare_attempts;
static uint8_t s_camera_dm_settle_samples;
static uint8_t s_camera_dm_prepare_active;
static uint32_t s_camera_dm_prepare_started,s_camera_dm_prepare_next;
static float s_camera_reach_percent,s_camera_active_reach_deg;
/* User targets are angles relative to the explicitly established zeros. */
static float s_camera_pose_deg[3]={ARM_CAMERA_LIFT_DEG,ARM_CAMERA_REACH_DEG,ARM_CAMERA_DM_DEG};
static float s_camera_lift_percent;
static int64_t s_camera_reference_positions[2];
static uint32_t s_camera_reference_pos_seq[2],s_camera_reference_flag_seq[2],s_camera_reference_tick;
static uint8_t s_camera_reference_samples;
static uint32_t s_camera_hold_sequence,s_camera_hold_since,s_percent_command_tick;
static uint8_t s_camera_hold_samples;
static uint8_t s_camera_position_mode=1U;
static uint8_t s_camera_native_pending,s_camera_native_ack,s_camera_native_ack_seen,s_camera_native_samples;
static uint8_t s_camera_native_motion_seen;
static uint8_t s_camera_native_hold_only; /* No native move was issued: verify holding, not travel direction. */
static uint8_t s_camera_dm_zero_hold_known;
static uint8_t s_camera_native_stop_verify;
static uint32_t s_camera_native_stop_tick;
static uint32_t s_camera_native_tick,s_camera_native_timeout,s_camera_native_position_seq,s_camera_native_flags_seq,s_camera_native_settle_tick;
static int64_t s_camera_native_last_counts;
static float s_camera_native_start_pct,s_camera_native_min,s_camera_native_max;
/* CAMERA-only same-absolute-target delivery/recovery; no new stop or reverse. */
#define CAMERA_REPLAY_ACK_MS 100U
#define CAMERA_REPLAY_STALL_MS 500U
#define CAMERA_REPLAY_MAX 50U
static int32_t s_camera_replay_goal;
static uint8_t s_camera_replay_count;
static uint32_t s_camera_replay_sent,s_camera_progress_tick,s_camera_progress_seq;
static float s_camera_best_error;
static uint32_t s_camera_retry_generation;
static uint8_t s_camera_top_hold,s_camera_top_samples;
static uint8_t s_camera_release_raise,s_camera_raise_overlap;
static float s_camera_raise_half;
static uint32_t s_camera_top_sequence,s_camera_top_tick;
static int64_t s_camera_top_counts;
/* V149: native retry owns moving reach stalls; this observer only bridges
 * PRECHECK/lift/DM_HOME/REACH_ENABLE before a new reach job starts. */
#define CAMERA_HANDOFF_STALL_SAMPLES (3U)
#define CAMERA_HANDOFF_STALL_CONFIRM_MS (300U)
#define CAMERA_HANDOFF_STALL_MAX_MS (1000U)
#define CAMERA_HANDOFF_PROGRESS_COUNTS (16.0)
#if !ARM_28_STALL_IGNORE_TRIAL
static uint8_t s_camera_handoff_pending,s_camera_handoff_samples;
#endif
/* Idle gaps do not age into a motion fault. A new movement/completion
 * requires real fresh packets; V179 reacquires without a stale-age FAULT. */
static uint8_t s_camera_feedback_wait;
static uint32_t s_camera_feedback_tick,s_camera_feedback_pos[2],s_camera_feedback_flags[2];
#if !ARM_28_STALL_IGNORE_TRIAL
static uint32_t s_camera_handoff_flags,s_camera_handoff_position,s_camera_handoff_first_position;
static uint32_t s_camera_handoff_tick,s_camera_handoff_progress_tick;
static int64_t s_camera_handoff_counts;
#endif
/* CAMERA native completion and position arrive in separate read replies.
 * Wait for new, settled position before judging a REACHED residual. */
#define CAMERA_RESIDUAL_MAX_PERCENT (2.0f)
#define CAMERA_RESIDUAL_RPM (6U)
static uint8_t s_camera_reached_wait,s_camera_residual;
static uint32_t s_camera_reached_tick,s_camera_residual_tick;
typedef struct {
    const char *reason;
    uint32_t tick,dm_age,pos_age[2],flags_age[2];
    float dm_deg,dm_vel;
    uint8_t valid,phase;
    int64_t reach_raw;
    float reach_actual_percent,reach_start_percent,reach_target_percent;
    uint8_t reach_actual_valid,reach_flags,ack_seen,ack,motion_seen;
    uint32_t loop_gap,previous_work,current_work,rx_wait_max,rx_drops,wire_age[2][2];
    uint8_t rx_pending;
} camera_failure_t;
static camera_failure_t s_camera_failure;
static void Cal_CameraNativePoll(uint32_t now);
static void Cal_CameraPoll(uint32_t now);
static void Cal_CameraFail(const char *reason);
static uint8_t Cal_CameraDmHealthyZero(float tolerance,float velocity)
{
    dm_feedback_t fb;float actual,target;
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    return (ArmTask_IsEnabled() && !ArmTask_IsFaulted() && fb.enabled && fb.fb_count &&
        (fb.err_code==0U || fb.err_code==1U) && HAL_GetTick()-fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT &&
        ArmTask_GetMotor()->active_mode==DM_POS_MODE && isfinite(fb.vel_rads) && fabsf(fb.vel_rads)<=velocity &&
        ArmDriver_GetRotationDeg(&actual)==HAL_OK && fabsf(actual-s_camera_pose_deg[2])<=tolerance &&
        ArmDriver_MapRawRotation(ArmTask_GetMotor()->target_pos_rad,&target)==HAL_OK && fabsf(target-s_camera_pose_deg[2])<0.01f)?1U:0U;
}
static uint8_t Cal_CameraActive(void)
{
    return ((s_camera_state>=ARM_CAMERA_PRECHECK && s_camera_state<=ARM_CAMERA_LIFT_FINAL) ||
        s_camera_state==ARM_CAMERA_REFERENCE)?1U:0U;
}
/* Owner-task read scheduler. Never burst four same-bus queries or synthesize
 * feedback timestamps. ISR continues to post to the existing RX queue. */
#define CAL_QUERY_GAP_MS 10U
#define CAL_QUERY_REPLY_MS 30U
#define CAL_QUERY_SEND_WINDOW_MS 90U
static arm_cal_query_status_t s_query_stats[2];
static uint8_t s_query_pending,s_query_cursor,s_query_slot,s_query_active,s_query_wait,s_query_attempts;
static uint32_t s_query_next_tick,s_query_claim_tick,s_query_sent_tick,s_query_baseline;
static HAL_StatusTypeDef Cal_PercentVelocity(float error);
static void Cal_PercentPoll(uint32_t now);
static void Cal_Fail(const char *reason);

#define CAL_RX_SLOTS (16U)
#define CAL_COMMAND_SLOTS (4U)
typedef struct { uint32_t identifier; uint32_t received_tick,fd_sequence; uint8_t data[8]; uint8_t length; } cal_rx_t;
/* Both indices and slot storage are shared with producers in tasks/ISRs. */
static volatile cal_rx_t s_rx[CAL_RX_SLOTS];
static volatile uint32_t s_wire_tick[2][2],s_rx_drops;
static volatile uint32_t s_fd_wire_sequence[2]; /* Written by RX ISR/task producers. */
static arm_cal_position_ack_t s_position_ack[2]; /* Drained/read by the owner task only. */
static uint32_t s_rx_wait_max,s_loop_last,s_loop_gap,s_loop_previous_work,s_loop_enter;
static uint8_t s_loop_seen,s_io_active;
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile char s_commands[CAL_COMMAND_SLOTS][96];
static volatile uint8_t s_command_head;
static volatile uint8_t s_command_tail;
static void Cal_Height(uint32_t now);
static uint8_t Cal_FeedbackWait(uint32_t now);

static uint8_t Cal_Context(void) { return ((s_inited != 0U) && (__get_IPSR() == 0U)) ? 1U : 0U; }
void ArmCalibration_DiscardCommands(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();s_command_tail=s_command_head;__set_PRIMASK(key);
}
static int Cal_Index(arm_driver_axis_t axis)
{
    if (axis == ARM_DRIVER_LIFT) { return 0; }
    if (axis == ARM_DRIVER_REACH) { return 1; }
    return -1;
}
static arm_driver_axis_t Cal_Axis(void) { return (s_active == 0U) ? ARM_DRIVER_LIFT : ARM_DRIVER_REACH; }
static uint8_t Cal_Address(uint8_t index) { return (index == 0U) ? ARM_DRIVER_LIFT_ID : ARM_DRIVER_REACH_ID; }
static uint8_t Cal_IsActive(void)
{
    return ((s_state == ARM_CAL_PRECHECK) || (s_state == ARM_CAL_SEEKING) ||
            (s_state == ARM_CAL_SETTLING) || (s_state == ARM_CAL_PERCENT_MOVE) ||
            (s_state == ARM_CAL_PERCENT_SETTLE)) ? 1U : 0U;
}

HAL_StatusTypeDef ArmCalibration_PostCanFrame(const FDCAN_RxHeaderTypeDef *header,
                                              const uint8_t *data, uint8_t length)
{
    uint32_t address;
    uint32_t mask;
    uint8_t next;
    uint8_t i;
    if ((header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_EXTENDED_ID) || (header->RxFrameType != FDCAN_DATA_FRAME) ||
        (header->FDFormat != FDCAN_CLASSIC_CAN) || ((header->Identifier & 0xFFU) != 0U) ||
        !(((length == 3U) && (data[0] == 0x3AU || data[0] == 0xFDU)) || ((length == 7U) && (data[0] == 0x36U))))
    {
        return HAL_ERROR;
    }
    address = header->Identifier >> 8U;
    if ((address != ARM_DRIVER_LIFT_ID) && (address != ARM_DRIVER_REACH_ID)) { return HAL_ERROR; }
    mask = __get_PRIMASK();
    __disable_irq();
    {uint32_t received=HAL_GetTick();
     if(data[length-1U]==ZDT_CAN_CHECKSUM && (data[0]==0x36U || data[0]==0x3AU))
        s_wire_tick[address==ARM_DRIVER_LIFT_ID?0:1][data[0]==0x36U?0:1]=received;
     next = (uint8_t)((s_rx_head + 1U) % CAL_RX_SLOTS);
     if (next == s_rx_tail) { s_rx_drops++;__set_PRIMASK(mask); return HAL_BUSY; }
     s_rx[s_rx_head].identifier = header->Identifier;
     s_rx[s_rx_head].fd_sequence=(length==3U && data[0]==0xFDU && data[2]==ZDT_CAN_CHECKSUM)?
        ++s_fd_wire_sequence[address==ARM_DRIVER_LIFT_ID?0:1]:0U;
     s_rx[s_rx_head].received_tick = received;}
    s_rx[s_rx_head].length = length;
    for (i = 0U; i < length; ++i) { s_rx[s_rx_head].data[i] = data[i]; }
    __DMB();
    s_rx_head = next;
    __set_PRIMASK(mask);
    return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_PostCommand(const char *line)
{
    uint32_t mask;
    size_t length;
    size_t i;
    uint8_t next;
    if ((line == NULL) || (__get_IPSR() != 0U)) { return HAL_ERROR; }
    /* Bound the scan as well as the copy. Caller supplies a complete line. */
    for (length = 0U; (length < 96U) && (line[length] != '\0'); ++length) { }
    if (length >= 96U) { return HAL_ERROR; }
    mask = __get_PRIMASK();
    __disable_irq();
    next = (uint8_t)((s_command_head + 1U) % CAL_COMMAND_SLOTS);
    if (next == s_command_tail) { __set_PRIMASK(mask); return HAL_BUSY; }
    for (i = 0U; i <= length; ++i) { s_commands[s_command_head][i] = line[i]; }
    __DMB();
    s_command_head = next;
    __set_PRIMASK(mask);
    return HAL_OK;
}

static void Cal_DrainFeedback(void)
{
    uint32_t mask;
    uint8_t i;
    uint8_t budget;
    for (budget = 0U; budget < CAL_RX_SLOTS; ++budget)
    {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[8];
        uint8_t length;
        uint32_t received_tick,fd_sequence;
        mask = __get_PRIMASK(); __disable_irq();
        if (s_rx_tail == s_rx_head) { __set_PRIMASK(mask); break; }
        header.Identifier = s_rx[s_rx_tail].identifier;
        received_tick = s_rx[s_rx_tail].received_tick;
        fd_sequence = s_rx[s_rx_tail].fd_sequence;
        {uint32_t age=HAL_GetTick()-received_tick;if(age>s_rx_wait_max)s_rx_wait_max=age;}
        length = s_rx[s_rx_tail].length;
        for (i = 0U; i < length; ++i) { data[i] = s_rx[s_rx_tail].data[i]; }
        s_rx_tail = (uint8_t)((s_rx_tail + 1U) % CAL_RX_SLOTS);
        __set_PRIMASK(mask);
        header.IdType = FDCAN_EXTENDED_ID;
        header.RxFrameType = FDCAN_DATA_FRAME;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        if(data[0]==0xFDU) {
            if(length==3U && data[2]==ZDT_CAN_CHECKSUM) {
                uint8_t index=(header.Identifier>>8U)==ARM_DRIVER_LIFT_ID?0U:1U;
                s_position_ack[index].sequence=fd_sequence;
                s_position_ack[index].received_tick=received_tick;
                s_position_ack[index].response=data[1];
            }
            if(length==3U && data[2]==ZDT_CAN_CHECKSUM && (header.Identifier>>8U)==ARM_DRIVER_LIFT_ID &&
               ((s_active==0U && s_lift_native_sent &&
                 (s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE)) || ArmMaterial_IsActive())) {
                char line[112];
                (void)snprintf(line,sizeof(line),"[ARM] id=5 native FD ACK=0x%02X cal=%u material=%u; ACK is not arrival\r\n",
                    data[1],(unsigned)s_state,ArmMaterial_IsActive());
                ZDT_Console_Print(line);
            }
            if(length==3U && data[2]==ZDT_CAN_CHECKSUM && (header.Identifier>>8U)==ARM_DRIVER_REACH_ID &&
               s_camera_native_pending && (int32_t)(received_tick-s_camera_native_tick)>=0) {
                if(!s_camera_native_ack_seen || s_camera_native_ack!=data[1]) {
                    char line[65];
                    (void)snprintf(line,sizeof(line),"[ARM CAMERA] native FD ACK=0x%02X (02=accepted,9F=reached)\r\n",(unsigned int)data[1]);
                    ZDT_Console_Print(line);
                }
                s_camera_native_ack=data[1];s_camera_native_ack_seen=1U;
            }
            continue;
        }
        if (ArmCalibration_OnCanFrame(&header, data, length) != 0U)
        {
            uint8_t index = ((header.Identifier >> 8U) == ARM_DRIVER_LIFT_ID) ? 0U : 1U;
            if (data[0] == 0x3AU) { s_axes[index].flags_tick = received_tick; }
            else { s_axes[index].position_tick = received_tick; }
        }
    }
}
static void Cal_DrainPorts(void)
{
    uint32_t mask;uint8_t i;
    Cal_DrainFeedback();
    Cal_Height(HAL_GetTick());
    /* One command per tick; nested wait pump drains feedback only. */
    {
        char line[96];
        mask = __get_PRIMASK(); __disable_irq();
        if (s_command_tail == s_command_head) { __set_PRIMASK(mask); return; }
        for (i = 0U; i < 96U; ++i) { line[i] = s_commands[s_command_tail][i]; if (line[i] == '\0') { break; } }
        s_command_tail = (uint8_t)((s_command_tail + 1U) % CAL_COMMAND_SLOTS);
        __set_PRIMASK(mask);
        {
            uint8_t read=0,write=0;
            while(line[read]) {
                char c=line[read++];if(c=='\t')c=' ';
                if(c==' ' && (write==0 || line[write-1]==' '))continue;
                line[write++]=c;
            }
            if(write && line[write-1]==' ')write--;
            line[write]=0;
        }
        (void)ArmCalibration_ProcessCommand(line);
    }
}

static HAL_StatusTypeDef Cal_Send(uint8_t index, const uint8_t *data, uint32_t dlc)
{
    FDCAN_TxHeaderTypeDef header = {0};
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    const zdt_motor_status_t *status = ZDT_Motor_GetStatus();
    if ((status == NULL) || (status->can_ready == 0U) ||
        (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol) != HAL_OK) || (protocol.BusOff != 0U))
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U) { return HAL_BUSY; }
    header.Identifier = (uint32_t)Cal_Address(index) << 8U;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    {
        HAL_StatusTypeDef result;uint32_t key=__get_PRIMASK();__disable_irq();
        result=HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&header,(uint8_t *)(uintptr_t)data);
        __set_PRIMASK(key);return result;
    }
}

static void Cal_Query(uint8_t index)
{
    uint8_t bits;
    if(index>1U)return;
    bits=(uint8_t)(3U<<(index*2U));
    if(s_query_active)bits&=(uint8_t)~(1U<<s_query_slot);
    s_query_pending|=bits;
}
static void Cal_QueryAdvance(uint32_t now)
{
    s_query_active=s_query_wait=0U;s_query_cursor=(uint8_t)((s_query_slot+1U)%4U);
    s_query_next_tick=now+CAL_QUERY_GAP_MS;
}
static void Cal_QueryService(uint32_t now)
{
    arm_cal_axis_status_t *a;arm_cal_query_status_t *d;uint8_t index,k,position;
    uint32_t sequence,received;HAL_StatusTypeDef result;uint8_t data[2];
    if(s_query_active && s_query_wait) {
        index=(uint8_t)(s_query_slot/2U);position=(uint8_t)(s_query_slot&1U);a=&s_axes[index];
        sequence=position?a->position_sequence:a->flags_sequence;
        received=position?a->position_tick:a->flags_tick;
        if(sequence!=s_query_baseline && (int32_t)(received-s_query_sent_tick)>=0) {
            Cal_QueryAdvance(now);return;
        }
        if(now-s_query_sent_tick<CAL_QUERY_REPLY_MS)return;
        s_query_stats[index].no_reply++;
        if(s_query_attempts>=2U){Cal_QueryAdvance(now);return;}
        s_query_wait=0U;s_query_next_tick=now+CAL_QUERY_GAP_MS;
        s_query_stats[index].retries++;return;
    }
    if((int32_t)(now-s_query_next_tick)<0)return;
    if(!s_query_active) {
        for(k=0U;k<4U;k++) {
            uint8_t slot=(uint8_t)((s_query_cursor+k)%4U);
            if(s_query_pending&(1U<<slot)) {
                s_query_slot=slot;s_query_pending&=(uint8_t)~(1U<<slot);
                s_query_active=1U;s_query_wait=0U;s_query_attempts=0U;s_query_claim_tick=now;break;
            }
        }
        if(!s_query_active)return;
    }
    if(now-s_query_claim_tick>=CAL_QUERY_SEND_WINDOW_MS){Cal_QueryAdvance(now);return;}
    index=(uint8_t)(s_query_slot/2U);position=(uint8_t)(s_query_slot&1U);a=&s_axes[index];d=&s_query_stats[index];
    s_query_baseline=position?a->position_sequence:a->flags_sequence;
    data[0]=position?0x36U:0x3AU;data[1]=ZDT_CAN_CHECKSUM;
    result=Cal_Send(index,data,FDCAN_DLC_BYTES_2);d->last_hal=(uint8_t)result;
    if(result==HAL_OK) {
        if(position)d->queued_position++;else d->queued_flags++;
        s_query_sent_tick=now;s_query_wait=1U;s_query_attempts++;
    } else {
        if(result==HAL_BUSY)d->busy++;else d->error++;
        s_query_next_tick=now+CAL_QUERY_GAP_MS;
    }
}

void ArmCalibration_ServiceIo(void)
{
    uint32_t now;
    if(!Cal_Context() || s_io_active)return;
    s_io_active=1U;Cal_DrainFeedback();now=HAL_GetTick();Cal_Height(now);
    if(now-s_last_query>=s_cfg.query_period_ms){s_last_query=now;Cal_Query(0U);Cal_Query(1U);}
    Cal_QueryService(now);s_io_active=0U;
}

void ArmCalibration_ServiceWait(void)
{
    uint32_t now;
    if(!Cal_Context())return;
    ArmCalibration_ServiceIo();now=HAL_GetTick();ArmServo_Update(now);
    if(Cal_FeedbackWait(now))return;
    /* Only maintain a target already accepted by the percent controller.
     * Never recurse into CAMERA/POSE/MATERIAL or process queued commands. */
    if(s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) {
        if(now-s_started>=s_percent_timeout)Cal_Fail("percent motion timeout during DM wait");
        else Cal_PercentPoll(now);
        if(s_state==ARM_CAL_FAILED)(void)ArmDriver_Disable(ARM_DRIVER_ROTATION); /* Abort the in-flight DM handshake. */
    }
    if(!ARM_ZDT_WAIT_ON_STALE || ArmCalibration_ZdtFeedbackFresh())(void)ArmDriver_CheckHeightInterlock();
}

uint8_t ArmCalibration_ZdtFeedbackFresh(void)
{
    uint8_t i;uint32_t now=HAL_GetTick();
    for(i=0U;i<2U;i++)if(!s_axes[i].position_sequence || !s_axes[i].flags_sequence ||
        now-s_axes[i].position_tick>s_cfg.feedback_timeout_ms ||
        now-s_axes[i].flags_tick>ArmCalibration_StatusTimeoutMs(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT))return 0U;
    return 1U;
}
uint8_t ArmCalibration_WaitForFeedback(arm_feedback_wait_t *wait,uint32_t now,uint32_t *paused_ms)
{
    uint8_t fresh_fault=0U,i;
    *paused_ms=0U;
    if(!ARM_ZDT_WAIT_ON_STALE){(void)wait;(void)now;return 0U;}
    if(wait->active)*paused_ms=now-wait->tick;
    wait->tick=now;
    /* DM faults remain independent of the two ZDT reply ages. */
    for(i=0U;i<2U;i++)if(s_axes[i].flags_sequence &&
        now-s_axes[i].flags_tick<=ArmCalibration_StatusTimeoutMs(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT) &&
        ArmCalibration_FaultFlags(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,s_axes[i].flags))fresh_fault=1U;
    if(ArmCalibration_ZdtFeedbackFresh() || ArmTask_IsFaulted() || fresh_fault) {
        if(wait->active)ZDT_Console_Print("[ARM FEEDBACK] fresh5/6 restored; resume SAME action\r\n");
        wait->active=0U;s_feedback_holding=0U;return 0U;
    }
    if(!wait->active)ZDT_Console_Print("[ARM FEEDBACK] stale5/6: QUERY/wait, no feedback-age FAULT; arrival still needs NEW packets\r\n");
    wait->active=1U;s_feedback_holding=1U;Cal_Query(0U);Cal_Query(1U);Cal_QueryService(now);return 1U;
}
uint8_t ArmCalibration_FeedbackWaiting(void){return s_feedback_holding;}
static uint8_t Cal_FeedbackWait(uint32_t now)
{
    uint32_t gap;uint8_t waiting;
    if(!Cal_CameraActive() && s_state!=ARM_CAL_PERCENT_MOVE && s_state!=ARM_CAL_PERCENT_SETTLE) {
        memset(&s_feedback_pause,0,sizeof(s_feedback_pause));return 0U;
    }
    waiting=ArmCalibration_WaitForFeedback(&s_feedback_pause,now,&gap);
    if(gap) {
        s_started+=gap;s_motion_started+=gap;s_percent_command_tick+=gap;
        s_camera_started+=gap;s_camera_step_started+=gap;s_camera_native_tick+=gap;
        s_camera_native_stop_tick+=gap;s_camera_native_settle_tick+=gap;
        s_camera_replay_sent+=gap;s_camera_progress_tick+=gap;
        s_camera_dm_prepare_started+=gap;s_camera_dm_prepare_next+=gap;
        s_camera_dm_settle_tick+=gap;s_camera_reference_tick+=gap;
        s_camera_top_tick+=gap;s_camera_reached_tick+=gap;s_camera_residual_tick+=gap;
    }
    if(waiting && s_percent_rpm) {
        if(ArmDriver_Stop(Cal_Axis())!=HAL_OK){Cal_Fail("feedback wait velocity STOP send failed");return 0U;}
        s_percent_rpm=0;s_percent_command_tick=now;
    }
    return waiting;
}

static void Cal_Report(uint8_t index, const char *event)
{
    const arm_cal_axis_status_t *a = &s_axes[index];
    char line[190];
    arm_cal_query_status_t q={0};
    uint8_t direction=255U;
    arm_driver_axis_t axis=index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT;
    (void)ArmDriver_GetLinearDirection(axis,&direction);
    (void)snprintf(line,sizeof(line),"[ARM] id=%u dir=%u speed=%urpm enable_req=%u\r\n",
        (unsigned int)Cal_Address(index),(unsigned int)direction,
        (unsigned int)ArmDriver_GetLinearSpeed(axis),(unsigned int)ArmDriver_IsEnableRequested(axis));
    ZDT_Console_Print(line);
    ArmCalibration_GetQueryStatus(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&q);
    (void)snprintf(line,sizeof(line),"[ARM] id=%u query tx36=%lu tx3A=%lu busy=%lu error=%lu no_reply=%lu retry=%lu last_hal=%u pending=%u\r\n",
        (unsigned int)Cal_Address(index),(unsigned long)q.queued_position,(unsigned long)q.queued_flags,
        (unsigned long)q.busy,(unsigned long)q.error,(unsigned long)q.no_reply,(unsigned long)q.retries,q.last_hal,q.pending);
    ZDT_Console_Print(line);
    (void)snprintf(line, sizeof(line),
        "[ARM] id=%u %s raw=%lld flags=0x%02X stall=%u protected=%u\r\n",
        (unsigned int)Cal_Address(index), event, (long long)a->position_counts,
        (unsigned int)a->flags, (unsigned int)((a->flags & ARM_CAL_FLAG_STALL) != 0U),
        (unsigned int)((a->flags & ARM_CAL_FLAG_PROTECTED) != 0U));
    ZDT_Console_Print(line);
    {
        float percent;
        uint32_t now=HAL_GetTick();
        HAL_StatusTypeDef valid=ArmCalibration_GetPercent((index==0U)?ARM_DRIVER_LIFT:ARM_DRIVER_REACH,&percent);
        (void)snprintf(line,sizeof(line),"[ARM] id=%u pos_rx=%lu pos_age=%lums flags_rx=%lu flags_age=%lums\r\n",
            (unsigned int)Cal_Address(index),(unsigned long)a->position_sequence,
            (unsigned long)(now-a->position_tick),(unsigned long)a->flags_sequence,
            (unsigned long)(now-a->flags_tick));ZDT_Console_Print(line);
        if(valid==HAL_OK) {
            (void)snprintf(line,sizeof(line),"[ARM] id=%u position=%.2f%% relative_deg=%.2f span=%.3fcounts travel=%.1fmm\r\n",
                (unsigned int)Cal_Address(index),(double)percent,
                (double)(a->position_counts-a->zero_counts)*360.0/65536.0,
                s_percent_span[index],(double)a->travel_mm);
        } else {
            (void)snprintf(line,sizeof(line),"[ARM] id=%u position=UNKNOWN (record 0%% and require fresh feedback); span=%.3fcounts travel=%.1fmm\r\n",
                (unsigned int)Cal_Address(index),s_percent_span[index],(double)a->travel_mm);
        }
        ZDT_Console_Print(line);
    }
    if ((a->lower_valid != 0U) && (a->upper_valid != 0U))
    {
        (void)snprintf(line, sizeof(line),
            "[ARM] id=%u lower=%lld upper=%lld span=%lld counts rev=%.5f travel_mm=%.3f\r\n",
            (unsigned int)Cal_Address(index), (long long)a->lower_counts,
            (long long)a->upper_counts, (long long)(a->upper_counts - a->lower_counts),
            (double)(a->upper_counts - a->lower_counts) / 65536.0, (double)a->travel_mm);
        ZDT_Console_Print(line);
    }
}

static void Cal_Fail(const char *reason)
{
    const arm_cal_axis_status_t *a=&s_axes[s_active];
    uint32_t now=HAL_GetTick();
    uint8_t direction=255U,enable_req=ArmDriver_IsEnableRequested(Cal_Axis());
    char line[210];
    uint8_t percent_move=(s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE)?1U:0U;
    (void)ArmDriver_GetLinearDirection(Cal_Axis(),&direction);
    HAL_StatusTypeDef stopped = ArmDriver_Stop(Cal_Axis());
    s_state = ARM_CAL_FAILED;
    s_drop_record=0;s_zero_record=0;
    ZDT_Console_Print("[ARM] calibration failed: ");
    ZDT_Console_Print(reason);
    ZDT_Console_Print(stopped == HAL_OK ? "; stop queued\r\n" : "; STOP SEND FAILED\r\n");
    (void)snprintf(line,sizeof(line),
        "[ARM] id=%u fault dir=%u flags=0x%02X enabled=%u enable_req=%u stall=%u protected=%u raw=%lld pos_age=%lums flags_age=%lums\r\n",
        (unsigned int)Cal_Address(s_active),(unsigned int)direction,(unsigned int)a->flags,
        (unsigned int)((a->flags&ARM_CAL_FLAG_ENABLED)!=0U),(unsigned int)enable_req,
        (unsigned int)((a->flags&ARM_CAL_FLAG_STALL)!=0U),(unsigned int)((a->flags&ARM_CAL_FLAG_PROTECTED)!=0U),
        (long long)a->position_counts,(unsigned long)(now-a->position_tick),(unsigned long)(now-a->flags_tick));
    ZDT_Console_Print(line);
    if(percent_move) {
        float current=0;HAL_StatusTypeDef valid=ArmCalibration_GetPercent(Cal_Axis(),&current);
        (void)snprintf(line,sizeof(line),"[ARM] id=%u percent fault target=%.4f actual=%.4f valid=%u last=%.4f cmd_rpm=%d zero=%lld span=%.3f\r\n",
            (unsigned int)Cal_Address(s_active),(double)s_percent_target,(double)current,(unsigned int)(valid==HAL_OK),
            (double)s_percent_last,(int)s_percent_rpm,(long long)a->zero_counts,s_percent_span[s_active]);
        ZDT_Console_Print(line);
    }
}

void ArmCalibration_DefaultConfig(arm_cal_config_t *out)
{
    if (out == NULL) { return; }
    (void)memset(out, 0, sizeof(*out));
    out->seek_speed_rpm = 20U;
    out->acceleration = 200U;
    out->query_period_ms = 100U;
    out->feedback_timeout_ms = ARM_ZDT_FEEDBACK_TIMEOUT_MS;
    out->seek_timeout_ms = 15000U;
    out->stall_grace_ms = 300U;
}

HAL_StatusTypeDef ArmCalibration_Init(const arm_cal_config_t *config)
{
    if ((config == NULL) || (__get_IPSR() != 0U)) { return HAL_ERROR; }
    if (s_inited != 0U) { return HAL_BUSY; }
    if ((config->seek_speed_rpm == 0U) || (config->seek_speed_rpm > 100U) ||
        (config->lift_reverse > 1U) || (config->reach_reverse > 1U) ||
        (config->query_period_ms < 20U) || (config->query_period_ms > 200U) ||
        (config->feedback_timeout_ms < 2U * config->query_period_ms) ||
        (config->feedback_timeout_ms > ARM_ZDT_FEEDBACK_TIMEOUT_MS) ||
        (config->seek_timeout_ms <= config->feedback_timeout_ms) ||
        (config->seek_timeout_ms > 60000U) || (config->stall_grace_ms >= config->seek_timeout_ms))
    {
        return HAL_ERROR;
    }
    s_cfg = *config;
    s_percent_span[0]=ARM_LIFT_FULL_TRAVEL_COUNTS;
    s_percent_span[1]=ARM_REACH_FULL_TRAVEL_COUNTS;
    s_axes[0].travel_mm=ARM_LIFT_FULL_TRAVEL_MM;
    s_axes[1].travel_mm=ARM_REACH_FULL_TRAVEL_MM;
    s_axes[0].upper_counts=ARM_LIFT_TOP_ZERO_COUNTS;s_axes[0].upper_valid=1U;
    s_axes[0].zero_counts=ARM_LIFT_TOP_ZERO_COUNTS;s_axes[0].zero_valid=1U;
    s_axes[1].lower_counts=ARM_REACH_RETRACT_ZERO_COUNTS;s_axes[1].lower_valid=1U;
    s_axes[1].zero_counts=ARM_REACH_RETRACT_ZERO_COUNTS;s_axes[1].zero_valid=1U;
    s_down_counts_per_mm=s_percent_span[0]/(double)s_axes[0].travel_mm;
    s_state = ARM_CAL_IDLE;
    s_inited = 1U;
    s_last_query = HAL_GetTick();
    memset(&s_camera_failure,0,sizeof(s_camera_failure));
    memset(s_query_stats,0,sizeof(s_query_stats));s_query_pending=s_query_active=s_query_wait=0U;
    s_query_cursor=2U;s_query_next_tick=s_last_query; /* Begin with previously starved ID6. */
    return HAL_OK;
}

uint8_t ArmCalibration_OnCanFrame(const FDCAN_RxHeaderTypeDef *header,
                                  const uint8_t *data, uint8_t length)
{
    uint32_t address;
    uint8_t index;
    arm_cal_axis_status_t *a;
    uint32_t magnitude;
    if ((Cal_Context() == 0U) || (header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_EXTENDED_ID) || (header->RxFrameType != FDCAN_DATA_FRAME) ||
        (header->FDFormat != FDCAN_CLASSIC_CAN) || ((header->Identifier & 0xFFU) != 0U)) { return 0U; }
    address = header->Identifier >> 8U;
    if ((address != ARM_DRIVER_LIFT_ID) && (address != ARM_DRIVER_REACH_ID)) { return 0U; }
    index = (address == ARM_DRIVER_LIFT_ID) ? 0U : 1U;
    a = &s_axes[index];
    if ((length == 3U) && (data[0] == 0x3AU) && (data[2] == ZDT_CAN_CHECKSUM))
    {
        a->flags = data[1];
        a->flags_tick = HAL_GetTick();
        ++a->flags_sequence;
        return 1U;
    }
    if ((length == 7U) && (data[0] == 0x36U) && (data[1] <= 1U) && (data[6] == ZDT_CAN_CHECKSUM))
    {
        magnitude = ((uint32_t)data[2] << 24U) | ((uint32_t)data[3] << 16U) |
                    ((uint32_t)data[4] << 8U) | (uint32_t)data[5];
        a->position_counts = (data[1] != 0U) ? -(int64_t)magnitude : (int64_t)magnitude;
        a->position_tick = HAL_GetTick();
        ++a->position_sequence;
        return 1U;
    }
    return 0U;
}

static HAL_StatusTypeDef Cal_Start(arm_driver_axis_t axis, uint8_t upper, uint8_t seek)
{
    if(ARM_28_STALL_IGNORE_TRIAL && seek && axis==ARM_DRIVER_REACH) {
        ZDT_Console_Print("[28 TRIAL] stall-based reach seek unavailable; use stationary arm zero6/reference teach\r\n");return HAL_ERROR;
    }
    int index = Cal_Index(axis);
    if ((Cal_Context() == 0U) || (index < 0) || (upper > 1U)) { return HAL_ERROR; }
    if (Cal_IsActive() != 0U) { return HAL_BUSY; }
    if (seek != 0U)
    {
        if (ArmDriver_IsEnableRequested(axis) == 0U) { return HAL_ERROR; }
        if ((axis == ARM_DRIVER_LIFT) &&
            (ArmDriver_CheckLiftSeek((upper != 0U) ? 1 : -1) != HAL_OK)) { return HAL_ERROR; }
    }
    else if (ArmDriver_Stop(axis) != HAL_OK) { return HAL_ERROR; }
    s_active = (uint8_t)index;
    s_upper = upper;
    s_seek = seek;
    s_drop_record=0;
    s_zero_record=0;
    s_percent_rpm=0;
    s_stall_samples = 0U;
    s_settle_samples = 0U;
    s_started = HAL_GetTick();
    s_flags_baseline = s_axes[index].flags_sequence;
    s_position_baseline = s_axes[index].position_sequence;
    s_state = ARM_CAL_PRECHECK;
    Cal_Query(s_active);
    s_last_query = s_started;
    return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_MarkEndpoint(arm_driver_axis_t axis, uint8_t upper)
{
    return Cal_Start(axis, upper, 0U);
}
HAL_StatusTypeDef ArmCalibration_SeekEndpoint(arm_driver_axis_t axis, uint8_t upper)
{
    return Cal_Start(axis, upper, 1U);
}

HAL_StatusTypeDef ArmCalibration_Cancel(void)
{
    memset(&s_feedback_pause,0,sizeof(s_feedback_pause));
    s_feedback_holding=0U;
    ArmRound1_AbortState();
    ArmDriver_CancelReachRetry();
    uint8_t pose_active=ArmStartPose_IsActive();
    ArmMaterial_AbortState();
    ArmStartPose_Cancel();
    HAL_StatusTypeDef result = HAL_OK;
    if (Cal_Context() == 0U) { return HAL_ERROR; }
    DM_Motor_DiagCancel(ArmTask_GetMotor());
    if(Cal_CameraActive()) {
        s_camera_native_pending=s_camera_native_hold_only=0U;
        s_camera_state=ARM_CAMERA_IDLE;
        result=ArmDriver_StopAll();
        ZDT_Console_Print("[ARM CAMERA] cancelled; stop requested\r\n");
    } else if(pose_active) {
        result=ArmDriver_StopAll(); /* The START native job is outside s_state. */
        ZDT_Console_Print("[ARM START] cancelled; native reach STOP requested\r\n");
    } else if (Cal_IsActive() != 0U) { result = ArmDriver_Stop(Cal_Axis()); }
    s_state = (result == HAL_OK) ? ARM_CAL_IDLE : ARM_CAL_FAILED;
    s_drop_record=0;
    s_zero_record=0;
    s_percent_rpm=0;
    s_lift_native_sent=s_lift_native_samples=0U;
    return result;
}

HAL_StatusTypeDef ArmCalibration_SetTravelMm(arm_driver_axis_t axis, float travel_mm)
{
    int index = Cal_Index(axis);
    if ((Cal_Context() == 0U) || (index < 0) || !isfinite(travel_mm) || (travel_mm <= 0.0f))
    {
        return HAL_ERROR;
    }
    if (Cal_IsActive() != 0U) { return HAL_BUSY; }
    s_axes[index].travel_mm = travel_mm;
    if(index==0 && s_top_mode)s_down_counts_per_mm=s_percent_span[0]/(double)travel_mm;
    return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_ClearProtection(arm_driver_axis_t axis)
{
    const uint8_t data[3] = {0x0EU, 0x52U, ZDT_CAN_CHECKSUM};
    int index = Cal_Index(axis);
    if ((Cal_Context() == 0U) || (index < 0)) { return HAL_ERROR; }
    /* Reach recovery is independent of an ongoing lift move. */
    if (Cal_IsActive() && !(index==1 && (s_active==0U || ARM_28_STALL_IGNORE_TRIAL))) { return HAL_BUSY; }
    return Cal_Send((uint8_t)index, data, FDCAN_DLC_BYTES_3);
}

void ArmCalibration_GetAxisStatus(arm_driver_axis_t axis, arm_cal_axis_status_t *out)
{
    int index = Cal_Index(axis);
    if ((Cal_Context() != 0U) && (out != NULL) && (index >= 0)) { *out = s_axes[index]; }
}
uint32_t ArmCalibration_PositionAckFence(arm_driver_axis_t axis)
{
    int index=Cal_Index(axis);
    return Cal_Context() && index>=0?s_fd_wire_sequence[index]:0U;
}
void ArmCalibration_GetPositionAck(arm_driver_axis_t axis,arm_cal_position_ack_t *out)
{
    int index=Cal_Index(axis);
    if(Cal_Context() && out && index>=0)*out=s_position_ack[index];
}
void ArmCalibration_GetQueryStatus(arm_driver_axis_t axis,arm_cal_query_status_t *out)
{
    int index=Cal_Index(axis);
    if(!Cal_Context() || !out || index<0)return;
    *out=s_query_stats[index];out->pending=(uint8_t)((s_query_pending>>(index*2U))&3U);
    if(s_query_active && s_query_slot/2U==(uint8_t)index)out->pending|=(uint8_t)(1U<<(s_query_slot&1U));
}
arm_cal_state_t ArmCalibration_GetState(void) { return s_state; }
uint8_t ArmCalibration_IsLiftRaisingToClearance(void)
{return (s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) && s_active==0U && fabsf(s_percent_target-ArmCalibration_TransferPercent(s_axes[0].travel_mm))<0.001f;}
uint8_t ArmCalibration_IsLiftRaisingToTop(void)
{return (s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) && s_active==0U && s_percent_target==0.0f;}

HAL_StatusTypeDef ArmCalibration_GetPercent(arm_driver_axis_t axis,float *percent)
{
    int index=Cal_Index(axis);
    const arm_cal_axis_status_t *a;
    if(!Cal_Context() || index<0 || percent==NULL)return HAL_ERROR;
    a=&s_axes[index];
    if(!a->zero_valid || !a->position_sequence ||
       HAL_GetTick()-a->position_tick>s_cfg.feedback_timeout_ms || s_percent_span[index]==0.0)return HAL_ERROR;
    *percent=(float)((double)(a->position_counts-a->zero_counts)/s_percent_span[index]*100.0);
    return isfinite(*percent)?HAL_OK:HAL_ERROR;
}

HAL_StatusTypeDef ArmCalibration_GetSpan(arm_driver_axis_t axis,float *span)
{
    int i=Cal_Index(axis);
    if(!Cal_Context() || i<0 || !span)return HAL_ERROR;
    *span=(float)s_percent_span[i];return isfinite(*span) && *span!=0.0f?HAL_OK:HAL_ERROR;
}
HAL_StatusTypeDef ArmCalibration_GetAbsoluteReachPulses(float percent,int32_t *pulses)
{
    double raw,value;
    if(!Cal_Context() || !pulses || !isfinite(percent) || percent< -ARM_REACH_REFERENCE_TOLERANCE_MM/ARM_REACH_FULL_TRAVEL_MM*100.0f || percent>100.0f ||
       !s_axes[1].zero_valid || !isfinite(s_percent_span[1]) || s_percent_span[1]==0.0)return HAL_ERROR;
    raw=(double)s_axes[1].zero_counts+(double)percent*s_percent_span[1]/100.0;
    value=round(raw*(s_percent_span[1]<0.0?-1.0:1.0)*ARM_CAMERA_POSITION_STEPS_REV/65536.0);
    if(!isfinite(value) || value>2147483647.0 || value< -2147483647.0)return HAL_ERROR;
    *pulses=(int32_t)value;return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_GetAbsoluteLiftPulses(float percent,int32_t *pulses)
{
    double raw,value;
    if(!Cal_Context() || !pulses || !isfinite(percent) || percent<0.0f || percent>100.0f ||
       !s_axes[0].zero_valid || !isfinite(s_percent_span[0]) || s_percent_span[0]==0.0)return HAL_ERROR;
    raw=(double)s_axes[0].zero_counts+(double)percent*s_percent_span[0]/100.0;
    value=round(raw*ARM_LIFT_POSITION_STEPS_REV/65536.0);
    if(!isfinite(value) || value>2147483647.0 || value< -2147483647.0)return HAL_ERROR;
    *pulses=(int32_t)value;return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_MovePercent(arm_driver_axis_t axis,float percent)
{
    int index=Cal_Index(axis);
    float current;
    double timeout;
    HAL_StatusTypeDef result;
    float minimum=0.0f;
    uint16_t motion_base_speed;uint8_t release_open;
    if(index==1 && ArmStartPose_IsActive()) {
        /* Composite reach targets may lie just across the recorded zero.
         * Keep the same 3mm endpoint allowance as the native position path. */
        if(!isfinite(s_axes[1].travel_mm) || s_axes[1].travel_mm<=0.0f)return HAL_ERROR;
        minimum=-ARM_REACH_REFERENCE_TOLERANCE_MM/s_axes[1].travel_mm*100.0f;
    }
    if(!Cal_Context() || index<0 || !isfinite(percent) || percent<minimum || percent>100.0f)return HAL_ERROR;
    if(Cal_IsActive()) {
        /* Repeated same-target requests are idempotent while already moving. */
        if((s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) &&
           s_active==(uint8_t)index && percent==s_percent_target)return HAL_OK;
        return HAL_BUSY;
    }
    if(ArmCalibration_GetPercent(axis,&current)!=HAL_OK || !ArmDriver_IsEnableRequested(axis))return HAL_ERROR;
    if(index==0 && (!s_top_mode || !s_axes[0].upper_valid || s_axes[0].travel_mm<=0.0f))return HAL_ERROR;
    release_open=(uint8_t)(index==0 && (ArmStartPose_IsReleaseOpening() ||
        (s_camera_release_raise && Cal_CameraActive() && ArmServo_GripOpeningOrOpen())));
    motion_base_speed=release_open?ArmDriver_GetConfiguredLinearSpeed(axis):ArmDriver_GetLinearSpeed(axis);
    if(!motion_base_speed)return HAL_ERROR;
    if(index==0 && ArmCalibration_GetAbsoluteLiftPulses(percent,&s_lift_native_goal)!=HAL_OK)return HAL_ERROR;
    timeout=fabs((double)(percent-current)*s_percent_span[index]/100.0)*60000.0/
        (65536.0*(double)motion_base_speed)+10000.0;
    if(timeout>4294967295.0)return HAL_ERROR;
    result=ArmDriver_Stop(axis);
    if(result!=HAL_OK)return result;
    s_active=(uint8_t)index;s_started=HAL_GetTick();
    /* Latch the grasp/release state at command acceptance: an empty descent
     * may overlap closing; its next loaded raise gets a new slow profile. */
    s_percent_empty_lift=(uint8_t)(index==0 && (ArmServo_GripIsOpen() || release_open));
    s_lift_native_sent=s_lift_native_samples=0U;
    s_lift_native_flags=s_axes[0].flags_sequence;s_lift_native_last=s_axes[0].position_counts;
    s_lift_native_attempts=s_lift_native_acked=s_lift_native_exhausted=0U;
    s_lift_progress_tick=s_started;s_lift_progress_seq=s_axes[0].position_sequence;
    s_lift_best_error=fabs((double)s_lift_native_goal*65536.0/ARM_LIFT_POSITION_STEPS_REV-s_axes[0].position_counts);
    s_lift_native_height=-percent*s_axes[0].travel_mm/100.0f;
    s_lift_native_speed=motion_base_speed;
    if(index==0 && (Cal_CameraActive() || ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive())) {
        if(s_percent_empty_lift)s_lift_native_speed=(uint16_t)fminf(
            s_lift_native_speed*ARM_EMPTY_LIFT_SPEED_SCALE,ARM_EMPTY_LIFT_RPM_MAX);
        else if(s_lift_native_speed>ARM_CAMERA_LIFT_RPM_MAX)s_lift_native_speed=ARM_CAMERA_LIFT_RPM_MAX;
    }
    s_percent_target=percent;s_percent_last=current;s_percent_rpm=0;s_percent_ready=0;
    s_percent_rate_factor=ArmSpeed_Factor(index?ARM_SPEED_REACH:ARM_SPEED_LIFT);
    if(index==0)s_lift_native_speed=ArmSpeed_Rpm(ARM_SPEED_LIFT,s_lift_native_speed);
    s_percent_command_tick=s_started;
    s_percent_position_tick=s_axes[index].position_tick;
    s_percent_speed_gain=1.0f;s_percent_stop_wait=0U;
    s_percent_min=s_percent_max=current;
    s_percent_timeout=ArmSpeed_Timeout((timeout>(double)s_cfg.seek_timeout_ms)?(uint32_t)timeout:s_cfg.seek_timeout_ms);
    s_flags_baseline=s_axes[index].flags_sequence;s_position_baseline=s_axes[index].position_sequence;
    s_settle_samples=0;s_state=ARM_CAL_PERCENT_MOVE;
    Cal_Query(s_active);s_last_query=s_started;
    {
        char line[100];
        (void)snprintf(line,sizeof(line),"[ARM] id=%u percent target=%.2f%% from=%.2f%%\r\n",
            (unsigned int)Cal_Address(s_active),(double)percent,(double)current);ZDT_Console_Print(line);
    }
    return HAL_OK;
}

HAL_StatusTypeDef ArmCalibration_HoldLiftForReachRecovery(void)
{
    HAL_StatusTypeDef result;
    if(!Cal_Context() || s_state==ARM_CAL_FAILED)return HAL_ERROR;
    if(s_state!=ARM_CAL_IDLE && !((s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) && s_active==0U))return HAL_BUSY;
    result=ArmDriver_Stop(ARM_DRIVER_LIFT);
    if(result!=HAL_OK)return result;
    s_percent_rpm=0;s_percent_ready=s_percent_stop_wait=s_settle_samples=0U;
    s_lift_native_sent=s_lift_native_samples=0U;
    s_state=ARM_CAL_IDLE;return HAL_OK;
}
uint8_t ArmCalibration_EmptyLiftMotionActive(void)
{
    return s_active==0U && s_percent_empty_lift &&
        (s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) &&
        (Cal_CameraActive() || ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive());
}
static HAL_StatusTypeDef Cal_PercentVelocity(float error)
{
    /* Slow the final approach instead of forcing 5RPM just outside the band.
     * Real feedback delay/coast can exceed the old 100ms ideal model. */
    double counts=fabs((double)error*s_percent_span[s_active]/100.0);
    uint16_t cap=ArmDriver_GetLinearSpeed(Cal_Axis());
    double speed=counts*60.0/65536.0;
    int16_t rpm;
    HAL_StatusTypeDef result;
    if(Cal_CameraActive() || ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive()) {
        uint16_t camera_cap=s_active?ArmDriver_ReachMotionSpeedRpm():
            (s_percent_empty_lift?ARM_EMPTY_LIFT_RPM_MAX:ARM_CAMERA_LIFT_RPM_MAX);
        if(!s_active) {
            speed*=ARM_CAMERA_LIFT_SPEED_GAIN;
            if(s_percent_empty_lift){speed*=ARM_EMPTY_LIFT_SPEED_SCALE;cap=(uint16_t)fminf(cap*ARM_EMPTY_LIFT_SPEED_SCALE,ARM_EMPTY_LIFT_RPM_MAX);}
        }
        if(s_active)cap=camera_cap;
        else if(cap>camera_cap)cap=camera_cap;
    }
    if(s_active?!ArmServo_GripIsOpen():!s_percent_empty_lift)speed*=ARM_LOADED_SPEED_SCALE*ARM_CARRY_SPEED_SCALE;
    speed*=ARM_MOTION_SPEED_SCALE;
    if(s_active)speed*=ARM_REACH_SPEED_SCALE*ARM_REACH_TRIAL_SPEED_SCALE;
    if(s_active==1U && s_camera_residual==2U && Cal_CameraActive() && cap>CAMERA_RESIDUAL_RPM)
        cap=CAMERA_RESIDUAL_RPM;
    /* V160 trial: no fixed near-TOP RPM cap; retain distance-based slowdown,
     * measured velocity compensation and arrival/STOP checks below. */
    if(speed>cap)speed=cap;
    speed=fmin((double)ZDT_CAN_RUN_MAX_RPM,speed*s_percent_rate_factor);
    speed/=(double)s_percent_speed_gain;
    if(speed<1.0)speed=1.0;
    if(!cap)return HAL_ERROR;
    rpm=(int16_t)speed;
    if((s_active==0U && error>0.0f) || (s_active==1U && error<0.0f))rpm=(int16_t)-rpm;
    if(Cal_CameraActive() || ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive()) {
        uint32_t elapsed=HAL_GetTick()-s_percent_command_tick;
        int16_t step;
        if(elapsed>200U)elapsed=200U;
        uint32_t accel=s_active?ARM_CAMERA_REACH_ACCEL_RPM_S:ARM_CAMERA_LIFT_ACCEL_RPM_S;
        step=(int16_t)(elapsed*accel/1000U);
        /* Slew only acceleration. Never preserve an outward command while
         * the feedback asks for braking or a direction reversal. */
        if((rpm>0 && s_percent_rpm<0) || (rpm<0 && s_percent_rpm>0))rpm=0;
        else if(abs(rpm)>abs(s_percent_rpm)) {
            if(step==0)return HAL_OK;
            if(rpm>s_percent_rpm+step)rpm=(int16_t)(s_percent_rpm+step);
            if(rpm<s_percent_rpm-step)rpm=(int16_t)(s_percent_rpm-step);
        }
    }
    if(rpm==s_percent_rpm && !(ARM_28_STALL_IGNORE_TRIAL && s_active==1U && (s_axes[1].flags&ARM_CAL_FLAG_STALL)))return HAL_OK;
    result=ArmDriver_RunLinear(Cal_Axis(),rpm,-s_percent_target*s_axes[0].travel_mm/100.0f);
    if(result==HAL_OK) { s_percent_rpm=rpm;s_percent_command_tick=HAL_GetTick(); }
    return result;
}

static uint8_t Cal_LiftNativeAckPoll(uint32_t now)
{
    arm_cal_position_ack_t ack={0};arm_cal_axis_status_t *a=&s_axes[0];HAL_StatusTypeDef result;
    double goal=(double)s_lift_native_goal*65536.0/ARM_LIFT_POSITION_STEPS_REV;
    double tolerance=fabs(s_percent_span[0])/(double)a->travel_mm*(double)ARM_LIFT_NATIVE_TOLERANCE_MM;
    char line[184];
    uint8_t camera=(uint8_t)(Cal_CameraActive() || ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive());
    if(!s_lift_native_sent || (s_lift_native_acked && !camera))return 0U;
    if(a->position_sequence!=s_lift_progress_seq) {
        double error=fabs((double)a->position_counts-goal);
        s_lift_progress_seq=a->position_sequence;
        if(s_lift_best_error-error>=fabs(s_percent_span[0])*0.001) {
            s_lift_best_error=error;s_lift_progress_tick=now;
        }
    }
    ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&ack);
    if((int32_t)(ack.sequence-s_lift_native_ack_fence)>0 && ack.sequence!=s_lift_native_ack_seen &&
       (int32_t)(ack.received_tick-s_lift_native_ack_tick)>=0) {
        s_lift_native_ack_seen=ack.sequence;
        if(ack.response==0x02U || ack.response==0x9FU) {
            s_lift_native_acked=1U;
            (void)snprintf(line,sizeof(line),"[ARM LIFT ACK] accepted=0x%02X attempt=%u ABS=%ld target=%.3f%%; actual arrival still required\r\n",
                ack.response,s_lift_native_attempts,(long)s_lift_native_goal,(double)s_percent_target);
            ZDT_Console_Print(line);return 0U;
        }
    }
    /* Actual near-target motion only needs the original stable confirmation.
     * Do not kick a converging/held target just because its ACK was lost. */
    if(fabs((double)a->position_counts-goal)<=tolerance || now-s_lift_native_last_send<LIFT_NATIVE_ACK_WAIT_MS)return 0U;
    if(camera && now-s_lift_progress_tick<(s_lift_native_acked?CAMERA_REPLAY_STALL_MS:LIFT_NATIVE_ACK_WAIT_MS))return 0U;
    if(s_lift_native_attempts>=(camera?CAMERA_REPLAY_MAX:LIFT_NATIVE_ACK_ATTEMPTS)) {
        if(!s_lift_native_exhausted) {
            s_lift_native_exhausted=1U;
            ZDT_Console_Print(camera?"[ARM LIFT REPLAY] 50 sends exhausted; original target/deadline retained\r\n":
                "[ARM LIFT ACK] no accepted ACK after3 sends; keep SAME target; original arrival/timeout retained\r\n");
        }
        return 0U;
    }
    s_lift_native_last_send=now;
    result=ArmDriver_LiftAbsoluteRawPulses(s_lift_native_goal,s_lift_native_speed,s_lift_native_height);
    if(result==HAL_BUSY)return 0U;
    if(result!=HAL_OK && !camera){Cal_Fail("native lift ACK resend rejected");return 1U;}
    ++s_lift_native_attempts;s_lift_native_flags=a->flags_sequence;
    if(camera){s_lift_progress_tick=now;s_lift_best_error=fabs((double)a->position_counts-goal);}
    s_lift_native_last=a->position_counts;s_lift_native_samples=0U;s_state=ARM_CAL_PERCENT_MOVE;
    (void)snprintf(line,sizeof(line),"[ARM LIFT ACK] retry=%u/%u SAME ABS=%ld target=%.3f%% speed=%uRPM HAL=%u; deadline unchanged\r\n",
        s_lift_native_attempts,camera?CAMERA_REPLAY_MAX:LIFT_NATIVE_ACK_ATTEMPTS,(long)s_lift_native_goal,(double)s_percent_target,s_lift_native_speed,(unsigned)result);
    ZDT_Console_Print(line);return 1U;
}
static void Cal_LiftNativePoll(uint32_t now,float current)
{
    arm_cal_axis_status_t *a=&s_axes[0];HAL_StatusTypeDef result;
    double per_mm=fabs(s_percent_span[0])/(double)a->travel_mm;
    double goal=(double)s_lift_native_goal*65536.0/ARM_LIFT_POSITION_STEPS_REV;
    uint8_t near=(uint8_t)(fabs((double)a->position_counts-goal)<=per_mm*ARM_LIFT_NATIVE_TOLERANCE_MM);
    if(!s_lift_native_sent && !near) {
        s_lift_native_ack_fence=ArmCalibration_PositionAckFence(ARM_DRIVER_LIFT);
        s_lift_native_ack_seen=s_lift_native_ack_fence;s_lift_native_ack_tick=now;
        result=ArmDriver_LiftAbsoluteRawPulses(s_lift_native_goal,s_lift_native_speed,s_lift_native_height);
        if(result==HAL_BUSY)return; /* Retry enqueue after the next fresh sample. */
        if(result!=HAL_OK){Cal_Fail("native lift position command rejected");return;}
        s_lift_native_sent=1U;s_lift_native_flags=a->flags_sequence;
        s_lift_native_attempts=1U;s_lift_native_last_send=HAL_GetTick();
        s_lift_native_last=a->position_counts;s_lift_native_samples=0U;
        {char line[176];
         (void)snprintf(line,sizeof(line),"[ARM] NATIVE FD id=5 mode=01 ABS goal_pulses=%ld steps/rev=%u target=%.3f%% speed=%uRPM; driver owns ramp/position\r\n",
            (long)s_lift_native_goal,ARM_LIFT_POSITION_STEPS_REV,(double)s_percent_target,s_lift_native_speed);
         ZDT_Console_Print(line);}
        return;
    }
    /* Do not accept a stale pre-command enabled/reached state. ACK alone is
     * never arrival. No STOP on entering the former8mm/0.5% band: let the
     * drive converge to its exact ABS target and keep holding that target. */
    if(a->flags_sequence==s_lift_native_flags)return;
    if(ArmStartPose_IsTrayPlaceDescent() &&
       fabs((double)a->position_counts-goal)<=per_mm*ARM_TRAY_PLACE_RELEASE_LIFT_MM) {
        /* Relinquish the software job for the existing OPEN50ms handoff.
         * Keep the drive's exact ABS target; the next lift owner replaces it. */
        char line[160];s_state=ARM_CAL_IDLE;
        (void)snprintf(line,sizeof(line),"[ARM] id=5 TRAY RELEASE within5mm actual=%.3f%% target=%.3f%%; ABS retained, no arrival STOP\r\n",
            (double)current,(double)s_percent_target);ZDT_Console_Print(line);return;
    }
    if(near && fabs((double)(a->position_counts-s_lift_native_last))<=per_mm*ARM_LIFT_NATIVE_STABLE_MM) {
        if(!s_lift_native_samples)s_lift_native_quiet_tick=now;
        if(s_lift_native_samples<ARM_LIFT_NATIVE_SAMPLES)s_lift_native_samples++;
        if(s_lift_native_samples>=ARM_LIFT_NATIVE_SAMPLES && now-s_lift_native_quiet_tick>=ARM_MOTION_SETTLE_MS) {
            char line[150];s_state=ARM_CAL_IDLE;
            (void)snprintf(line,sizeof(line),"[ARM] id=5 NATIVE ARRIVED target=%.3f%% actual=%.3f%% raw=%lld; ABS target held, no arrival STOP\r\n",
                (double)s_percent_target,(double)current,(long long)a->position_counts);
            ZDT_Console_Print(line);return;
        }
        s_state=ARM_CAL_PERCENT_SETTLE;
    } else {s_lift_native_samples=0U;s_state=ARM_CAL_PERCENT_MOVE;}
    s_lift_native_last=a->position_counts;
}

static void Cal_PercentPoll(uint32_t now)
{
    arm_cal_axis_status_t *a=&s_axes[s_active];
    float current,error;
    float tolerance=s_active?ARM_PERCENT_TOLERANCE:
        ArmCalibration_LiftTolerancePercent(s_percent_target,a->travel_mm);
    if(now-a->position_tick>s_cfg.feedback_timeout_ms || now-a->flags_tick>ArmCalibration_StatusTimeoutMs(Cal_Axis()) ||
       ArmCalibration_GetPercent(Cal_Axis(),&current)!=HAL_OK) { Cal_Fail("percent feedback missing/stale");return; }
    if(!s_percent_ready && (a->flags_sequence==s_flags_baseline || a->position_sequence==s_position_baseline))return;
    if(a->flags&ARM_CAL_FLAG_PROTECTED) { Cal_Fail("percent motor protected");return; }
    if(ArmCalibration_StallFault(Cal_Axis(),a->flags)) { Cal_Fail("percent motor stalled");return; }
    if(!(a->flags&ARM_CAL_FLAG_ENABLED)) { Cal_Fail("percent motor disabled");return; }
    if(!ArmDriver_IsEnableRequested(Cal_Axis())) { Cal_Fail("percent enable request cleared");return; }
    if(s_active==0U && Cal_LiftNativeAckPoll(now))return;
    if(s_percent_ready && a->position_sequence==s_position_baseline)return;
    s_percent_ready=1;s_position_baseline=a->position_sequence;
    error=s_percent_target-current;
    if(current<s_percent_min)s_percent_min=current;
    if(current>s_percent_max)s_percent_max=current;
    if(s_active==0U) {Cal_LiftNativePoll(now,current);return;}
    if(!Cal_CameraActive() && (ArmStartPose_IsFastTransfer() || ArmMaterial_IsActive() || ArmRound1_IsActive()) && fabsf(error)<=tolerance) {
        if(ArmDriver_Stop(Cal_Axis())!=HAL_OK){Cal_Fail("percent stop send failed");return;}
        s_percent_rpm=0;s_percent_stop_wait=0U;s_state=ARM_CAL_IDLE;
        {char line[160];(void)snprintf(line,sizeof(line),"[ARM] id=%u TARGET OBSERVED target=%.2f%% actual=%.2f%%; STOP, no extra stable dwell\r\n",
          (unsigned int)Cal_Address(s_active),(double)s_percent_target,(double)current);ZDT_Console_Print(line);}
        return;
    }
    if(s_percent_stop_wait) {
        if(fabsf(current-s_percent_last)<=ARM_PERCENT_TOLERANCE*0.3f)s_settle_samples++;
        else {s_settle_samples=0U;s_percent_command_tick=now;}
        s_percent_last=current;s_percent_position_tick=a->position_tick;
        if(s_settle_samples<ARM_LINEAR_POST_STOP_SAMPLES || now-s_percent_command_tick<ARM_MOTION_SETTLE_MS)return;
        s_percent_stop_wait=0U;s_percent_command_tick=now;
    }
    if(s_percent_rpm && ((s_active==0U)?-s_percent_rpm:s_percent_rpm)*(current-s_percent_last)<
        -fabsf((float)s_percent_rpm)*ARM_PERCENT_TOLERANCE) {
        Cal_Fail("percent direction mismatch; check arm dir");return;
    }
    if(s_percent_rpm && a->position_tick-s_percent_position_tick>=20U &&
       a->position_tick-s_percent_position_tick<=s_cfg.feedback_timeout_ms) {
        double delta=fabs((double)(current-s_percent_last)*s_percent_span[s_active]/100.0);
        double gain=delta*60000.0/(65536.0*(double)(a->position_tick-s_percent_position_tick)*abs(s_percent_rpm));
        if(delta>=64.0 && isfinite(gain)) {
            if(gain>12.0)gain=12.0;
            if(gain>(double)s_percent_speed_gain)s_percent_speed_gain=(float)gain;
        }
    }
    if(s_percent_rpm && ((s_percent_target-s_percent_last)*error<0.0f) && fabsf(error)>tolerance) {
        char line[160];
        if(ArmDriver_Stop(Cal_Axis())!=HAL_OK) { Cal_Fail("target crossing stop send failed");return; }
        s_percent_rpm=0;s_settle_samples=0;s_percent_stop_wait=1U;s_percent_command_tick=now;s_state=ARM_CAL_PERCENT_SETTLE;
        (void)snprintf(line,sizeof(line),"[ARM] id=%u target crossed: target=%.3f actual=%.3f; STOP and settle; speed_gain=%.2f\r\n",
            (unsigned int)Cal_Address(s_active),(double)s_percent_target,(double)current,(double)s_percent_speed_gain);
        ZDT_Console_Print(line);
        s_percent_last=current;s_percent_position_tick=a->position_tick;
        if(s_camera_state==ARM_CAMERA_REACH_MOVE && fabsf(error)>5.0f)Cal_Fail("camera target overshoot >5%; check speed/feedback");
        return;
    }
    if(s_state==ARM_CAL_PERCENT_SETTLE && fabsf(error)<=tolerance &&
       fabsf(current-s_percent_last)<=ARM_PERCENT_TOLERANCE*0.3f) {
        if(++s_settle_samples>=ARM_LINEAR_POST_STOP_SAMPLES) {
            char line[180];s_state=ARM_CAL_IDLE;
            (void)snprintf(line,sizeof(line),"[ARM] id=%u PERCENT ARRIVED target=%.2f%% actual=%.2f%% range=[%.2f,%.2f]%% speed_gain=%.2f\r\n",
                (unsigned int)Cal_Address(s_active),(double)s_percent_target,(double)current,
                (double)s_percent_min,(double)s_percent_max,(double)s_percent_speed_gain);ZDT_Console_Print(line);
        }
    } else if(fabsf(error)<=tolerance) {
        if(ArmDriver_Stop(Cal_Axis())!=HAL_OK) { Cal_Fail("percent stop send failed");return; }
        s_percent_rpm=0;s_settle_samples=0;s_percent_stop_wait=1U;s_percent_command_tick=now;s_state=ARM_CAL_PERCENT_SETTLE;
    } else {
        s_state=ARM_CAL_PERCENT_MOVE;s_settle_samples=0;
        if(Cal_PercentVelocity(error)!=HAL_OK) { Cal_Fail("percent velocity/height guard rejected");return; }
    }
    s_percent_last=current;
    s_percent_position_tick=a->position_tick;
}

static void Cal_Height(uint32_t now)
{
    const arm_cal_axis_status_t *a = &s_axes[0];
    if(s_top_mode) {
        uint8_t fresh=(uint8_t)(a->upper_valid && a->position_sequence &&
            now-a->position_tick<=s_cfg.feedback_timeout_ms);
        float h=0;
        if(fresh && s_down_counts_per_mm!=0.0)
            h=-(float)((double)(a->position_counts-a->upper_counts)/s_down_counts_per_mm);
        else if(fresh && a->position_counts!=a->upper_counts)fresh=0;
        if(!isfinite(h) || (ARM_LIFT_RANGE_STOP_ENABLED && h>ARM_TOP_REFERENCE_TOLERANCE_MM))fresh=0;
        else if(h>0.0f)h=0.0f; /* Near TOP: tolerate reference/encoder variation. */
        ArmDriver_UpdateLiftHeight(h,a->position_tick,fresh);return;
    }
    uint8_t valid = ((a->lower_valid != 0U) && (a->upper_valid != 0U) &&
                     (a->lower_counts != a->upper_counts) && (a->travel_mm > 0.0f) &&
                     (a->position_sequence != 0U) && (a->flags_sequence != 0U) &&
                     ((now - a->position_tick) <= s_cfg.feedback_timeout_ms) &&
                     ((now - a->flags_tick) <= ARM_ZDT_LIFT_STATUS_TIMEOUT_MS) &&
                     ((a->flags & 0x80U) == 0U)) ? 1U : 0U;
    float height = 0.0f;
    if (valid != 0U)
    {
        height = (float)((double)(a->position_counts - a->lower_counts) * (double)a->travel_mm /
                        (double)(a->upper_counts - a->lower_counts));
        if (!isfinite(height) || (ARM_LIFT_RANGE_STOP_ENABLED &&
            ((height < -ARM_TOP_REFERENCE_TOLERANCE_MM) ||
             (height > a->travel_mm + ARM_TOP_REFERENCE_TOLERANCE_MM)))) { valid = 0U; }
    }
    ArmDriver_UpdateLiftHeight(height, a->position_tick, valid);
}
HAL_StatusTypeDef ArmCalibration_UseTopGuard(float drop_mm,float min_deg,float max_deg)
{
    HAL_StatusTypeDef result;
    if(!Cal_Context() || Cal_IsActive())return HAL_BUSY;
    ArmDriver_UpdateLiftHeight(0,0,0);
    result=ArmDriver_SetTopRotationLimit(drop_mm,min_deg,max_deg);
    if(result==HAL_OK) { s_top_mode=1;s_top_drop_limit=drop_mm;s_top_min=min_deg;s_top_max=max_deg; }
    return result;
}

arm_camera_state_t ArmCalibration_GetCameraState(void) { return s_camera_state; }
const char *ArmCalibration_GetCameraFailureReason(void)
{return s_camera_failure.valid?s_camera_failure.reason:"none";}
HAL_StatusTypeDef ArmCalibration_SetCameraPositionMode(uint8_t enabled)
{
    if(!Cal_Context() || enabled>1U)return HAL_ERROR;
    if(Cal_CameraActive() || Cal_IsActive())return HAL_BUSY;
    s_camera_position_mode=enabled;return HAL_OK;
}
static HAL_StatusTypeDef Cal_CameraConfigure(float lift,float reach,float dm)
{
    double p5=(double)lift*65536.0/360.0/s_percent_span[0]*100.0;
    double p6=(double)reach*65536.0/360.0/s_percent_span[1]*100.0;
    if(Cal_IsActive() || Cal_CameraActive() || ArmStartPose_IsActive())return HAL_BUSY;
    if(!isfinite(p5) || !isfinite(p6) || !isfinite(dm) ||
       p5<0.0 || p5>100.0 || p6<0.0 || p6>100.0 || fabsf(dm)>180.0f)return HAL_ERROR;
    if(s_top_mode && p5*s_axes[0].travel_mm/100.0>s_top_drop_limit &&
       (dm<s_top_min || dm>s_top_max))return HAL_ERROR;
    s_camera_pose_deg[0]=lift;s_camera_pose_deg[1]=reach;s_camera_pose_deg[2]=dm;
    return HAL_OK;
}

static HAL_StatusTypeDef Cal_CameraStart(uint8_t record_reference,uint8_t reach_zero)
{
    double target;
    if(!Cal_Context())return HAL_ERROR;
    if(Cal_CameraActive())return HAL_OK; /* Repeated command cannot restart a stage. */
    if(Cal_IsActive() || ArmStartPose_IsActive())return HAL_BUSY;
    if((!record_reference && (!s_axes[0].zero_valid || !s_axes[1].zero_valid)) || !s_top_mode ||
       !ArmDriver_IsRotationReferenceValid() || !ArmTask_IsReady() ||
       !isfinite(s_percent_span[0]) || s_percent_span[0]==0.0 || s_axes[0].travel_mm<=0.0f ||
       !isfinite(s_percent_span[1]) || s_percent_span[1]==0.0)return HAL_ERROR;
    s_camera_active_reach_deg=reach_zero?0.0f:s_camera_pose_deg[1];
    target=(double)s_camera_active_reach_deg*65536.0/360.0/s_percent_span[1]*100.0;
    if(!isfinite(target) || target<0.0 || target>100.0)return HAL_ERROR;
    s_camera_release_raise=(uint8_t)(!record_reference && ArmStartPose_IsReleasedPlace());
    s_camera_raise_overlap=0U;
    ArmStartPose_ReleaseForCamera();
    s_camera_reach_percent=(float)target;
    s_camera_lift_percent=(float)((double)s_camera_pose_deg[0]*65536.0/360.0/s_percent_span[0]*100.0);
    if(!isfinite(s_camera_lift_percent) || s_camera_lift_percent<0.0f || s_camera_lift_percent>100.0f)return HAL_ERROR;
    s_state=ARM_CAL_IDLE;
    s_camera_native_pending=s_camera_native_ack_seen=s_camera_native_ack=s_camera_native_motion_seen=0U;
    s_camera_native_hold_only=0U;
    s_camera_native_samples=s_camera_native_stop_verify=s_camera_reached_wait=s_camera_residual=0U;
    s_camera_native_start_pct=0.0f;
    (void)ArmCalibration_GetPercent(ARM_DRIVER_REACH,&s_camera_native_start_pct);
    s_camera_native_min=s_camera_native_max=s_camera_native_start_pct;
#if !ARM_28_STALL_IGNORE_TRIAL
    s_camera_handoff_pending=s_camera_handoff_samples=0U;
    s_camera_handoff_flags=s_axes[1].flags_sequence;s_camera_handoff_position=s_axes[1].position_sequence;
    s_camera_handoff_counts=s_axes[1].position_counts;
#endif
    s_camera_dm_zero_hold_known=Cal_CameraDmHealthyZero(1.0f,0.05f);
    memset(&s_camera_failure,0,sizeof(s_camera_failure));
    s_camera_feedback_wait=0U;
    s_camera_started=s_camera_step_started=HAL_GetTick();
    s_camera_top_hold=s_camera_top_samples=0U;
    s_camera_dm_prepare_active=0U;
    s_camera_state=record_reference?ARM_CAMERA_REFERENCE:ARM_CAMERA_PRECHECK;
    if(!record_reference && ArmTask_IsEnabled() && !s_camera_dm_zero_hold_known &&
       ArmDriver_Disable(ARM_DRIVER_ROTATION)!=HAL_OK) {
        Cal_CameraFail("DM release before lift failed");return HAL_ERROR;
    }
    if(record_reference) {
        uint8_t i;
        s_camera_reference_samples=0U;s_camera_reference_tick=s_camera_started;
        for(i=0U;i<2U;i++) {
            s_camera_reference_positions[i]=s_axes[i].position_counts;
            s_camera_reference_pos_seq[i]=s_axes[i].position_sequence;
            s_camera_reference_flag_seq[i]=s_axes[i].flags_sequence;
        }
        /* Stop linear axes before teaching. Preserve an already healthy zero
         * hold; otherwise release DM until the asserted height is established. */
        if(ArmDriver_Stop(ARM_DRIVER_LIFT)!=HAL_OK || ArmDriver_Stop(ARM_DRIVER_REACH)!=HAL_OK) {
            Cal_CameraFail("reference preparation send failed");return HAL_ERROR;
        }
        if(Cal_CameraDmHealthyZero(1.0f,0.05f)) {
            ZDT_Console_Print("[ARM CAMERA] INIT: DM already holds existing zero; keep enabled\r\n");
        } else if(ArmDriver_Disable(ARM_DRIVER_ROTATION)!=HAL_OK) {
            Cal_CameraFail("reference DM disable send failed");return HAL_ERROR;
        }
        ZDT_Console_Print("[ARM CAMERA] INIT: asserted TOP/RETRACTED; waiting stationary RAM reference; no flash write\r\n");
    }
    Cal_Query(0U);Cal_Query(1U);
    {
        char line[180];(void)snprintf(line,sizeof(line),
            "[ARM CAMERA] start: TOP first -> DM=%+.2fdeg -> reach=%+.2fdeg -> lift=%+.2fdeg; waiting fresh feedback\r\n",
            (double)s_camera_pose_deg[2],(double)s_camera_active_reach_deg,(double)s_camera_pose_deg[0]);ZDT_Console_Print(line);
    }
    return HAL_OK;
}
HAL_StatusTypeDef ArmCalibration_DeployCamera(void) { return Cal_CameraStart(0U,0U); }
HAL_StatusTypeDef ArmCalibration_DeployCameraInit(void) { return Cal_CameraStart(1U,0U); }
HAL_StatusTypeDef ArmCalibration_DeployParkingCamera(void) { if(Cal_CameraActive())return HAL_BUSY;return Cal_CameraStart(0U,1U); }
void ArmCalibration_ClearCameraReady(void)
{
    if(!Cal_CameraActive())s_camera_state=ARM_CAMERA_IDLE;
}

static void Cal_CameraFail(const char *reason)
{
    HAL_StatusTypeDef stopped;
    dm_feedback_t dm;float dm_angle=0;uint8_t dm_valid;char dm_line[190];
    uint8_t keep_dm=(uint8_t)(s_camera_dm_zero_hold_known && Cal_CameraDmHealthyZero(2.0f,0.20f));
    HAL_StatusTypeDef next;
    DM_GetFeedback(ArmTask_GetMotor(),&dm);
    dm_valid=(uint8_t)(ArmDriver_GetRotationDeg(&dm_angle)==HAL_OK);
    {
        uint8_t i;uint32_t now=HAL_GetTick();
        s_camera_failure.reason=reason;s_camera_failure.tick=now;
        s_camera_failure.phase=(uint8_t)s_camera_state;s_camera_failure.dm_age=now-dm.last_tick;
        s_camera_failure.dm_deg=dm_angle;s_camera_failure.dm_vel=dm.vel_rads;
        s_camera_failure.reach_raw=s_axes[1].position_counts;s_camera_failure.reach_flags=s_axes[1].flags;
        s_camera_failure.reach_actual_valid=(uint8_t)(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&s_camera_failure.reach_actual_percent)==HAL_OK);
        s_camera_failure.reach_start_percent=s_camera_native_start_pct;s_camera_failure.reach_target_percent=s_camera_reach_percent;
        s_camera_failure.ack_seen=s_camera_native_ack_seen;s_camera_failure.ack=s_camera_native_ack;
        s_camera_failure.motion_seen=s_camera_native_motion_seen;
        for(i=0U;i<2U;i++) {
            s_camera_failure.pos_age[i]=now-s_axes[i].position_tick;
            s_camera_failure.flags_age[i]=now-s_axes[i].flags_tick;
        }
        s_camera_failure.loop_gap=s_loop_gap;s_camera_failure.previous_work=s_loop_previous_work;
        s_camera_failure.current_work=now-s_loop_enter;s_camera_failure.rx_wait_max=s_rx_wait_max;
        {uint32_t key=__get_PRIMASK();__disable_irq();
         s_camera_failure.rx_pending=(uint8_t)((s_rx_head+CAL_RX_SLOTS-s_rx_tail)%CAL_RX_SLOTS);
         s_camera_failure.rx_drops=s_rx_drops;
         for(i=0;i<2;i++){s_camera_failure.wire_age[i][0]=now-s_wire_tick[i][0];s_camera_failure.wire_age[i][1]=now-s_wire_tick[i][1];}
         __set_PRIMASK(key);}
        s_camera_failure.valid=1U;
    }
    s_camera_state=ARM_CAMERA_FAILED; /* Never advance after a failure. */
    s_camera_native_pending=0U;
    stopped=ArmCalibration_Cancel();
    next=ArmDriver_Stop(ARM_DRIVER_LIFT);if(next!=HAL_OK)stopped=next;
    next=ArmDriver_Stop(ARM_DRIVER_REACH);if(next!=HAL_OK)stopped=next;
    next=ArmServo_StopAll();if(next!=HAL_OK)stopped=next;
    if(keep_dm && ArmDriver_RotateToDeg(s_camera_pose_deg[2])==HAL_OK) {
        ZDT_Console_Print("[ARM CAMERA] ZDT stop requested; DM stays ENABLED holding configured camera angle\r\n");
    } else {
        next=ArmDriver_StopAll();if(next!=HAL_OK)stopped=next;
        ZDT_Console_Print("[ARM CAMERA] DM hold unsafe/unconfirmed; release requested\r\n");
    }
    ZDT_Console_Print("[ARM CAMERA] FAILED: ");ZDT_Console_Print(reason);
    ZDT_Console_Print(stopped==HAL_OK?"; stop requested\r\n":"; STOP SEND FAILED\r\n");
    (void)snprintf(dm_line,sizeof(dm_line),"[ARM CAMERA] DM fault angle=%.4fdeg valid=%u raw=%.6frad vel=%.5frad/s age=%lums enabled=%u code=%u hold_bad=%u\r\n",
        (double)dm_angle,(unsigned int)dm_valid,(double)dm.pos_rad,(double)dm.vel_rads,
        (unsigned long)(HAL_GetTick()-dm.last_tick),(unsigned int)dm.enabled,(unsigned int)dm.err_code,(unsigned int)s_camera_hold_samples);
    ZDT_Console_Print(dm_line);
    {
        uint8_t i;char line[195];
        for(i=0U;i<2U;i++) {
            float percent=0;uint8_t valid=(uint8_t)(ArmCalibration_GetPercent(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&percent)==HAL_OK);
            (void)snprintf(line,sizeof(line),"[ARM CAMERA] id=%u reference raw=%lld zero=%lld pct=%.4f valid=%u flags=0x%02X span=%.3f\r\n",
                (unsigned int)Cal_Address(i),(long long)s_axes[i].position_counts,(long long)s_axes[i].zero_counts,
                (double)percent,(unsigned int)valid,(unsigned int)s_axes[i].flags,s_percent_span[i]);
            ZDT_Console_Print(line);
        }
    }
}

static uint8_t Cal_CameraAxisFresh(uint8_t index,uint32_t now)
{
    const arm_cal_axis_status_t *a=&s_axes[index];
    return (a->position_sequence && a->flags_sequence &&
        now-a->position_tick<=s_cfg.feedback_timeout_ms && now-a->flags_tick<=ArmCalibration_StatusTimeoutMs(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT))?1U:0U;
}

/* Idle prerequisite gate: no velocity target is issued while waiting. Do
 * not rewrite RX timestamps or claim cached positions as a new observation. */
static uint8_t Cal_CameraFreshBeforeMotion(uint32_t now)
{
    uint8_t i,ready=1U;
    for(i=0U;i<2U;i++) {
        if(!Cal_CameraAxisFresh(i,now))ready=0U;
        if(s_camera_feedback_wait &&
           (s_axes[i].position_sequence==s_camera_feedback_pos[i] ||
            s_axes[i].flags_sequence==s_camera_feedback_flags[i] ||
            (int32_t)(s_axes[i].position_tick-s_camera_feedback_tick)<0 ||
            (int32_t)(s_axes[i].flags_tick-s_camera_feedback_tick)<0))ready=0U;
    }
    if(ready) {
        if(s_camera_feedback_wait)ZDT_Console_Print("[ARM CAMERA] fresh axes restored; continue SAME action\r\n");
        s_camera_feedback_wait=0U;return 1U;
    }
    if(!s_camera_feedback_wait) {
        s_camera_feedback_wait=1U;s_camera_feedback_tick=now;
        for(i=0U;i<2U;i++) {
            s_camera_feedback_pos[i]=s_axes[i].position_sequence;
            s_camera_feedback_flags[i]=s_axes[i].flags_sequence;
        }
        ZDT_Console_Print("[ARM CAMERA] idle ZDT feedback gap; QUERY before next motion, hold axes\r\n");
    }
    Cal_Query(0U);Cal_Query(1U);
    if(!ARM_ZDT_WAIT_ON_STALE && now-s_camera_feedback_tick>=2000U)Cal_CameraFail("fresh ZDT feedback unavailable before next motion");
    return 0U;
}

/* Called only after fresh positions/status and immediate protection guards.
 * Return1 while blocked or failed. Cache reads never increment confirmation. */
static uint8_t Cal_CameraReachHandoffBlocked(uint32_t now)
{
#if ARM_28_STALL_IGNORE_TRIAL
    (void)now;return 0U; /* No STALL-based gate in any camera stage. */
#else
    const arm_cal_axis_status_t *a=&s_axes[1];
    if(!(a->flags&ARM_CAL_FLAG_STALL)) {
        if(s_camera_handoff_pending)ZDT_Console_Print("[CAMERA HANDOFF] reach stall cleared in NEW status; resume same stage, no retry consumed\r\n");
        s_camera_handoff_pending=s_camera_handoff_samples=0U;
        s_camera_handoff_flags=a->flags_sequence;s_camera_handoff_position=a->position_sequence;
        s_camera_handoff_counts=a->position_counts;return 0U;
    }
    if(!s_camera_handoff_pending) {
        s_camera_handoff_pending=1U;s_camera_handoff_samples=0U;
        s_camera_handoff_tick=s_camera_handoff_progress_tick=now;
        s_camera_handoff_first_position=s_camera_handoff_position=a->position_sequence;
        s_camera_handoff_counts=a->position_counts;
        ZDT_Console_Print("[CAMERA HANDOFF] reach stall flag: observe NEW status/position,3 samples/300ms; no new reach move\r\n");
    }
    if(a->position_sequence!=s_camera_handoff_position) {
        s_camera_handoff_position=a->position_sequence;
        if(fabs((double)a->position_counts-(double)s_camera_handoff_counts)>=CAMERA_HANDOFF_PROGRESS_COUNTS)
            s_camera_handoff_progress_tick=now;
        s_camera_handoff_counts=a->position_counts;
    }
    if(a->flags_sequence!=s_camera_handoff_flags && (int32_t)(a->flags_tick-s_camera_started)>=0) {
        s_camera_handoff_flags=a->flags_sequence;
        if(s_camera_handoff_samples<255U)++s_camera_handoff_samples;
    }
    if(s_camera_handoff_samples>=CAMERA_HANDOFF_STALL_SAMPLES &&
       a->position_sequence!=s_camera_handoff_first_position && (int32_t)(a->position_tick-s_camera_handoff_tick)>0) {
        if(now-s_camera_handoff_tick>=CAMERA_HANDOFF_STALL_CONFIRM_MS &&
           now-s_camera_handoff_progress_tick>=CAMERA_HANDOFF_STALL_CONFIRM_MS) {
            Cal_CameraFail("reach sustained stall during CAMERA handoff");return 1U;
        }
        /* Real progress may coexist with a latched flag, but cannot mask it
         * indefinitely while the unowned reach should be holding. */
        if(now-s_camera_handoff_tick>=CAMERA_HANDOFF_STALL_MAX_MS) {
            Cal_CameraFail("reach stall flag persists during CAMERA handoff");return 1U;
        }
    }
    return 1U;
#endif
}

static void Cal_CameraEnable(uint8_t index,uint32_t now)
{
    s_camera_enable_position=s_axes[index].position_sequence;
    s_camera_enable_flags=s_axes[index].flags_sequence;
    (void)ArmDriver_Enable(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT);
    s_camera_enable_sent=HAL_GetTick();s_camera_enable_attempts=1U;
    s_camera_step_started=now;
    s_camera_state=index?ARM_CAMERA_REACH_ENABLE:ARM_CAMERA_LIFT_ENABLE;
    Cal_Query(index);
}
static void Cal_CameraEnableRetry(uint8_t index,uint32_t now)
{
    char line[144];HAL_StatusTypeDef result;
    if(((s_axes[index].flags&ARM_CAL_FLAG_ENABLED) && ArmDriver_IsEnableRequested(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT)) ||
        now-s_camera_enable_sent<100U || s_camera_enable_attempts>=10U)return;
    s_camera_enable_sent=now;result=ArmDriver_Enable(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT);
    if(result==HAL_BUSY)return;
    ++s_camera_enable_attempts;Cal_Query(index);
    snprintf(line,sizeof(line),"[ARM CAMERA ENABLE RETRY] id=%u send=%u/10 HAL=%u; original feedback deadline retained\r\n",
        index?6U:5U,s_camera_enable_attempts,(unsigned)result);ZDT_Console_Print(line);
}

static void Cal_CameraHome(void)
{
    dm_feedback_t fb;float zero=0;char line[145];uint32_t now=HAL_GetTick();HAL_StatusTypeDef result;
    if(!s_camera_dm_prepare_active) {
        s_camera_dm_prepare_active=1U;s_camera_dm_prepare_started=now;s_camera_dm_prepare_next=now;s_camera_dm_prepare_attempts=0U;
    }
    if((int32_t)(now-s_camera_dm_prepare_next)<0)return;
    if(s_camera_pose_deg[2]==0.0f)result=ArmDriver_ReturnRotationZero();
    else {
        result=ArmDriver_Enable(ARM_DRIVER_ROTATION);
        if(result==HAL_OK)result=ArmDriver_RotateToDeg(s_camera_pose_deg[2]);
    }
    if(result!=HAL_OK && now-s_camera_dm_prepare_started<3000U) {
        s_camera_dm_prepare_next=HAL_GetTick()+100U; /* Don't burst disable/query while the chassis moves. */
        ++s_camera_dm_prepare_attempts;
        snprintf(line,sizeof(line),"[ARM CAMERA DM ENABLE RETRY] attempt=%u HAL=%u; same reference/target, original 3s window\r\n",
            s_camera_dm_prepare_attempts,(unsigned)result);ZDT_Console_Print(line);
        return;
    }
    if(result!=HAL_OK) {
        (void)snprintf(line,sizeof(line),"[ARM CAMERA] DM home start HAL=%u (BUSY may be enable sway); capture cap %.2frad/s\r\n",
            (unsigned int)result,(double)ARM_ROTATION_ENABLE_MAX_VEL_RAD_S);ZDT_Console_Print(line);
        Cal_CameraFail(result==HAL_BUSY?"DM enable sway did not settle within 3s":"DM home rejected; check feedback/reference/height");return;
    }
    s_camera_dm_prepare_active=0U;
    if(ArmDriver_GetRotationZeroRaw(&zero)==HAL_OK) {
        (void)snprintf(line,sizeof(line),"[ARM CAMERA] DM HOME raw_ref=%.6frad target=%.6frad offset=%.6frad; existing zero unchanged\r\n",
            (double)zero,(double)ArmTask_GetMotor()->target_pos_rad,(double)ArmTask_GetMotor()->pos_offset_rad);ZDT_Console_Print(line);
    }
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    s_camera_dm_sequence=fb.fb_count;s_camera_dm_settle_samples=0U;
    s_camera_step_started=HAL_GetTick();s_camera_state=ARM_CAMERA_DM_HOME;
    Cal_Query(0U);Cal_Query(1U);
    ZDT_Console_Print("[ARM CAMERA] 2/3 DM camera target queued after lift TOP; waiting actual angle\r\n");
}

static HAL_StatusTypeDef Cal_CameraNativeStart(uint32_t now)
{
    float current;double distance_pulses,timeout;int32_t goal_pulses;
    uint8_t no_move;uint16_t speed=ArmDriver_ReachMotionSpeedRpm();
    if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK || !speed)return HAL_ERROR;
    {uint16_t cap=ArmServo_GripIsOpen()?ARM_CAMERA_REACH_RPM_MAX:ARM_LOADED_REACH_RPM_MAX;
     if(speed>cap)speed=cap;}
    speed=ArmSpeed_Rpm(ARM_SPEED_REACH,speed);
    distance_pulses=round(fabs((double)(s_camera_reach_percent-current)*s_percent_span[1]/100.0)*
        ARM_CAMERA_POSITION_STEPS_REV/65536.0);
    if(!isfinite(distance_pulses) || distance_pulses>2147483647.0 ||
       ArmCalibration_GetAbsoluteReachPulses(s_camera_reach_percent,&goal_pulses)!=HAL_OK)return HAL_ERROR;
    no_move=(uint8_t)(fabsf(s_camera_reach_percent-current)<=ARM_PERCENT_TOLERANCE);
    /* Effective pulse calibration changed by 2x; do not assume the driver's
     * RPM conversion changed with it. Feedback still determines arrival. */
    timeout=distance_pulses*120000.0/(ARM_CAMERA_POSITION_STEPS_REV*(double)speed)+5000.0;
    if(timeout<10000.0)timeout=10000.0;
    if(timeout>120000.0)return HAL_ERROR;
    s_camera_native_tick=now;s_camera_native_timeout=(uint32_t)timeout;
    s_camera_native_stop_verify=0U;
    s_camera_reached_wait=s_camera_residual=0U;
    s_camera_native_position_seq=s_axes[1].position_sequence;s_camera_native_flags_seq=s_axes[1].flags_sequence;
    s_camera_native_last_counts=s_axes[1].position_counts;
    s_camera_native_start_pct=s_camera_native_min=s_camera_native_max=current;
    s_camera_native_ack_seen=0U;s_camera_native_ack=0U;s_camera_native_samples=0U;
    s_camera_native_motion_seen=0U;
    s_camera_replay_goal=goal_pulses;s_camera_replay_count=0U;s_camera_replay_sent=s_camera_progress_tick=now;
    s_camera_progress_seq=s_axes[1].position_sequence;s_camera_best_error=fabsf(s_camera_reach_percent-current);
    s_camera_native_pending=no_move?0U:1U;
    s_camera_native_hold_only=no_move;
    s_camera_retry_generation=ArmDriver_ReachRetryGeneration();
    if(no_move) {
        if(ArmDriver_Stop(ARM_DRIVER_REACH)!=HAL_OK)return HAL_ERROR;
        ZDT_Console_Print("[ARM CAMERA] reach already near target; no native move issued; STOP/verify holding, bounded drift correction if needed\r\n");
    } else {
        if(ArmDriver_ReachAbsolutePulses(goal_pulses,speed,ARM_CAMERA_POSITION_ACCEL)!=HAL_OK)return HAL_ERROR;
        s_camera_retry_generation=ArmDriver_ReachRetryGeneration();
        {
            char line[190];
            (void)snprintf(line,sizeof(line),"[ARM CAMERA] NATIVE FD id=6 mode=01 ABS goal_pulses=%ld from=%.3f%% target=%.3f%% speed=%uRPM acc=%u; verify actual target\r\n",
                (long)goal_pulses,(double)current,(double)s_camera_reach_percent,(unsigned int)speed,(unsigned int)ARM_CAMERA_POSITION_ACCEL);
            ZDT_Console_Print(line);
        }
    }
    return HAL_OK;
}

static uint8_t Cal_CameraReplayPoll(uint32_t now,float current)
{
    float error=fabsf(s_camera_reach_percent-current);uint8_t missing,stalled;HAL_StatusTypeDef result;
    if(!s_camera_native_pending || s_camera_native_stop_verify || s_camera_residual || error<=ARM_PERCENT_TOLERANCE)return 0U;
    if(s_axes[1].position_sequence!=s_camera_progress_seq) {
        s_camera_progress_seq=s_axes[1].position_sequence;
        if(s_camera_best_error-error>=0.1f) {s_camera_best_error=error;s_camera_progress_tick=now;}
    }
    missing=(uint8_t)(!s_camera_native_ack_seen);
    stalled=(uint8_t)(error>ARM_PERCENT_TOLERANCE && now-s_camera_progress_tick>=CAMERA_REPLAY_STALL_MS);
    if(missing && s_camera_native_motion_seen && now-s_camera_progress_tick<CAMERA_REPLAY_STALL_MS)return 0U;
    if((!missing && !stalled) || s_camera_replay_count>=CAMERA_REPLAY_MAX)return 0U;
    if(now-s_camera_replay_sent>=CAMERA_REPLAY_ACK_MS) {
        s_camera_replay_sent=now;
        result=ArmDriver_ReplayReachAbsolutePulses(s_camera_replay_goal);
        if(result!=HAL_BUSY)++s_camera_replay_count;
        if(result==HAL_OK) {
            char line[180];s_camera_progress_tick=now;s_camera_best_error=error;
            s_camera_reached_wait=0U;s_camera_native_samples=0U;
            snprintf(line,sizeof(line),"[ARM CAMERA REPLAY] reason=%s send=%u/50 SAME ABS=%ld actual=%.3f%% target=%.3f%% ACK=%u/0x%02X; speed/acc unchanged\r\n",
                missing?"NO_ACK":"NO_PROGRESS",s_camera_replay_count,(long)s_camera_replay_goal,(double)current,(double)s_camera_reach_percent,s_camera_native_ack_seen,s_camera_native_ack);
            ZDT_Console_Print(line);
        }
    }
    return 1U;
}

static void Cal_CameraNativePoll(uint32_t now)
{
    arm_cal_axis_status_t *a=&s_axes[1];float current,error,travel;
    if(ArmDriver_ReachRetryFailed()){Cal_CameraFail("reach positioning retry failed");return;}
    if(ArmDriver_ReachRetryRecovering())return;
    if(s_camera_retry_generation!=ArmDriver_ReachRetryGeneration()) {
        if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Cal_CameraFail("reach retry feedback stale");return;}
        s_camera_retry_generation=ArmDriver_ReachRetryGeneration();s_camera_native_tick=now;
        s_camera_native_timeout=ArmDriver_ReachRetryTimeoutMs((double)(current-s_camera_reach_percent)*s_percent_span[1]/100.0);
        s_camera_native_stop_verify=s_camera_native_samples=s_camera_native_motion_seen=0U;
        s_camera_native_hold_only=0U;
        s_camera_reached_wait=s_camera_residual=0U;
        s_camera_replay_sent=s_camera_progress_tick=now;s_camera_progress_seq=a->position_sequence;
        s_camera_best_error=fabsf(s_camera_reach_percent-current);
        s_camera_native_start_pct=current;s_camera_native_last_counts=a->position_counts;
        s_camera_native_position_seq=a->position_sequence;s_camera_native_flags_seq=a->flags_sequence;return;
    }
    if(!(a->flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH)) {
        Cal_CameraFail("native reach disabled");return;
    }
    if(s_camera_residual==2U) {
        if(now-s_camera_residual_tick>=10000U){Cal_CameraFail("CAMERA small residual correction timeout");return;}
        if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK ||
           fabsf(current-s_camera_reach_percent)>CAMERA_RESIDUAL_MAX_PERCENT) {
            Cal_CameraFail("CAMERA residual correction moved outside small-error range");return;
        }
        if(s_state!=ARM_CAL_IDLE)return;
        /* Percentage correction already stopped; demand new held replies
         * again before publishing CAMERA READY to the station owner. */
        s_camera_residual=0U;s_camera_native_stop_verify=1U;s_camera_native_samples=0U;
        s_camera_native_stop_tick=now;s_camera_native_position_seq=a->position_sequence;
        s_camera_native_flags_seq=a->flags_sequence;s_camera_native_last_counts=a->position_counts;
        ZDT_Console_Print("[ARM CAMERA] residual corrected; verify NEW stopped feedback before READY\r\n");return;
    }
    if((!s_camera_native_stop_verify && now-s_camera_native_tick>=s_camera_native_timeout) ||
       (s_camera_native_stop_verify && now-s_camera_native_stop_tick>=2000U)) {
        Cal_CameraFail(s_camera_native_stop_verify?"native reach stop verification timeout":"native reach timeout");return;
    }
    if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK) { Cal_CameraFail("native reach position stale");return; }
    if(current<s_camera_native_min)s_camera_native_min=current;
    if(current>s_camera_native_max)s_camera_native_max=current;
    error=s_camera_reach_percent-current;travel=s_camera_reach_percent-s_camera_native_start_pct;
    if(current<-ARM_REACH_REFERENCE_TOLERANCE_MM/s_axes[1].travel_mm*100.0f ||
       current>100.0f+ARM_REACH_REFERENCE_TOLERANCE_MM/s_axes[1].travel_mm*100.0f ||
       (!s_camera_native_hold_only && travel*(current-s_camera_native_start_pct)<-fabsf(travel)*ARM_PERCENT_TOLERANCE)) {
        Cal_CameraFail("native reach wrong direction/range");return;
    }
    if(travel*error<0.0f && fabsf(error)>5.0f) { Cal_CameraFail("native reach target overrun; check pulse settings");return; }
    if(ArmDriver_ReachStallPending())return;
    /* A near-zero dither of0.36deg used to count as command startup. Require
     * meaningful encoder progress, independent of a latched REACHED flag. */
    if(a->position_sequence!=s_camera_native_position_seq && (int32_t)(a->position_tick-s_camera_native_tick)>=0 &&
       fabsf(current-s_camera_native_start_pct)>=fminf(ARM_PERCENT_TOLERANCE,fabsf(travel)*0.25f))s_camera_native_motion_seen=1U;
    if(s_camera_native_pending) {
        if(s_camera_native_ack_seen && s_camera_native_ack!=0x02U && s_camera_native_ack!=0x9FU) {
            Cal_CameraFail("motor rejected native FD command");return;
        }
        if(Cal_CameraReplayPoll(now,current))return;
        if(!s_camera_native_motion_seen && fabsf(error)>ARM_PERCENT_TOLERANCE)return;
    }
    if(a->position_sequence==s_camera_native_position_seq || a->flags_sequence==s_camera_native_flags_seq ||
       (int32_t)(a->position_tick-s_camera_native_tick)<0 || (int32_t)(a->flags_tick-s_camera_native_tick)<0)return;
    if(s_camera_native_stop_verify && ((int32_t)(a->position_tick-s_camera_native_stop_tick)<=0 ||
       (int32_t)(a->flags_tick-s_camera_native_stop_tick)<=0))return;
    s_camera_native_position_seq=a->position_sequence;s_camera_native_flags_seq=a->flags_sequence;
    if(s_camera_native_hold_only && !s_camera_residual && fabsf(error)>ARM_PERCENT_TOLERANCE) {
        /* No motion was requested from this near-target starting point.
         * A small release rebound is a held-position residual, not evidence
         * that a native travel command ran backwards. Reuse the existing
         * bounded STOP/hold/low-speed correction and final verification. */
        if(fabsf(error)>CAMERA_RESIDUAL_MAX_PERCENT){Cal_CameraFail("CAMERA residual exceeds bounded correction range");return;}
        if(ArmDriver_StopReachForHold()!=HAL_OK){Cal_CameraFail("CAMERA residual STOP rejected");return;}
        s_camera_residual=1U;s_camera_native_stop_verify=1U;s_camera_native_stop_tick=HAL_GetTick();
        s_camera_native_samples=0U;s_camera_native_last_counts=a->position_counts;
        {char line[176];(void)snprintf(line,sizeof(line),"[ARM CAMERA] near-target HOLD drift: start=%.3f%% actual=%.3f%% target=%.3f%%; STOP then fresh hold/correction, not native wrong-direction\r\n",
            (double)s_camera_native_start_pct,(double)current,(double)s_camera_reach_percent);ZDT_Console_Print(line);}
        return;
    }
    if(s_camera_residual==1U) {
        double stable=fabs((double)(a->position_counts-s_camera_native_last_counts));
        s_camera_native_last_counts=a->position_counts;
        if(fabsf(error)>CAMERA_RESIDUAL_MAX_PERCENT){Cal_CameraFail("CAMERA residual exceeds bounded correction range");return;}
        if(stable>fabs(s_percent_span[1])*0.0003){s_camera_native_samples=0U;return;}
        if(!s_camera_native_samples)s_camera_native_settle_tick=now;
        if(++s_camera_native_samples<ARM_MOTION_SETTLE_SAMPLES || now-s_camera_native_settle_tick<ARM_MOTION_SETTLE_MS)return;
        if(fabsf(error)<=ARM_PERCENT_TOLERANCE) {
            s_camera_residual=0U;s_camera_native_samples=0U;return;
        }
        if(ArmCalibration_MovePercent(ARM_DRIVER_REACH,s_camera_reach_percent)!=HAL_OK) {
            Cal_CameraFail("CAMERA small residual correction rejected");return;
        }
        s_camera_residual=2U;s_camera_residual_tick=HAL_GetTick();s_camera_native_pending=0U;
        {
            char line[190];(void)snprintf(line,sizeof(line),
                "[ARM CAMERA] small residual=%+.3fdeg (%.3fmm); encoder correction <=6RPM to SAME target; DM holds\r\n",
                (double)error*s_percent_span[1]/100.0*360.0/65536.0,
                (double)fabsf(error)*a->travel_mm/100.0);ZDT_Console_Print(line);
        }
        return;
    }
    if(s_camera_native_pending && (a->flags&ARM_CAL_FLAG_REACHED) && fabsf(error)>ARM_PERCENT_TOLERANCE &&
       now-s_camera_native_tick>=500U) {
        double stable=fabs((double)(a->position_counts-s_camera_native_last_counts));
        s_camera_native_last_counts=a->position_counts;
        if(!s_camera_reached_wait) {
            s_camera_reached_wait=1U;s_camera_reached_tick=now;s_camera_native_samples=0U;
            s_camera_native_settle_tick=now;
            ZDT_Console_Print("[ARM CAMERA] driver REACHED; wait NEW position/stability before residual decision\r\n");return;
        }
        if((int32_t)(a->position_tick-s_camera_reached_tick)<=0)return;
        if(stable>fabs(s_percent_span[1])*0.0003){s_camera_native_samples=0U;s_camera_native_settle_tick=now;return;}
        if(s_camera_native_samples<255U)++s_camera_native_samples;
        if(s_camera_native_samples<3U || now-s_camera_native_settle_tick<500U)return;
        if(fabsf(error)>CAMERA_RESIDUAL_MAX_PERCENT) {
            Cal_CameraFail("native stable residual >2%; check microsteps/input scale");return;
        }
        if(ArmDriver_StopReachForHold()!=HAL_OK){Cal_CameraFail("CAMERA residual STOP rejected");return;}
        s_camera_residual=1U;s_camera_native_stop_verify=1U;s_camera_native_stop_tick=HAL_GetTick();
        s_camera_native_samples=0U;
        ZDT_Console_Print("[ARM CAMERA] small stable residual; STOP then NEW hold samples before correction\r\n");return;
    }
    if(s_camera_reached_wait){s_camera_reached_wait=0U;s_camera_native_samples=0U;}
    if(fabsf(error)>ARM_PERCENT_TOLERANCE ||
       fabs((double)(a->position_counts-s_camera_native_last_counts))>fabs(s_percent_span[1])*ARM_PERCENT_TOLERANCE*0.003) {
        s_camera_native_samples=0U;
    } else {
        if(!s_camera_native_samples)s_camera_native_settle_tick=now;
        if(s_camera_native_samples<255U)s_camera_native_samples++;
    }
    s_camera_native_last_counts=a->position_counts;
    if(s_camera_native_samples>=ARM_MOTION_SETTLE_SAMPLES && now-s_camera_native_settle_tick>=ARM_MOTION_SETTLE_MS) {
        if(s_camera_native_pending && !(a->flags&ARM_CAL_FLAG_REACHED) && !s_camera_native_stop_verify) {
            if(ArmDriver_StopReachForHold()!=HAL_OK) { Cal_CameraFail("native measured arrival stop enqueue failed");return; }
            s_camera_native_stop_verify=1U;s_camera_native_stop_tick=HAL_GetTick();s_camera_native_samples=0U;
            ZDT_Console_Print("[ARM CAMERA] encoder target stable, driver REACHED=0; STOP sent, verify fresh hold feedback\r\n");return;
        }
        char line[195];
        (void)snprintf(line,sizeof(line),"[ARM CAMERA] NATIVE ARRIVED target=%.3f%% actual=%.3f%% range=[%.3f,%.3f]%% reply=%s; target/stopped feedback confirmed\r\n",
            (double)s_camera_reach_percent,(double)current,(double)s_camera_native_min,(double)s_camera_native_max,
            s_camera_native_ack_seen?"RECEIVED":"NONE");ZDT_Console_Print(line);
        s_camera_native_pending=s_camera_native_hold_only=0U;
        ArmDriver_CancelReachRetry();
        /* Mark native completion separately from the existing percentage job. */
        s_camera_native_samples=255U;
    }
}

static void Cal_CameraPoll(uint32_t now)
{
    dm_feedback_t fb;
    float angle,current;
    uint8_t index;
    arm_cal_axis_status_t *a;
    if(!Cal_CameraActive())return;
    ArmCalibration_ServiceIo();now=HAL_GetTick(); /* Apply already received packets before judging freshness. */
    if(now-s_camera_started>=ArmSpeed_Timeout(120000U)) { Cal_CameraFail("whole sequence timeout");return; }
    if(!ARM_ZDT_WAIT_ON_STALE && s_camera_feedback_wait && now-s_camera_feedback_tick>=2000U &&
       (!Cal_CameraAxisFresh(0U,now) || !Cal_CameraAxisFresh(1U,now))) {
        Cal_CameraFail("fresh ZDT feedback unavailable before next motion");return;
    }
    if(s_camera_state==ARM_CAMERA_REFERENCE) {
        uint8_t i,moved=0U;
        if(now-s_camera_step_started>=5000U) { Cal_CameraFail("stationary reference timeout");return; }
        for(i=0U;i<2U;i++) {
            a=&s_axes[i];
            if(!Cal_CameraAxisFresh(i,now))return;
            if(ArmCalibration_FaultFlags(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,a->flags)) { Cal_CameraFail("reference axis stalled/protected");return; }
            if(a->position_sequence==s_camera_reference_pos_seq[i] || a->flags_sequence==s_camera_reference_flag_seq[i] ||
               (int32_t)(a->position_tick-s_camera_step_started)<0 || (int32_t)(a->flags_tick-s_camera_step_started)<0)return;
        }
        for(i=0U;i<2U;i++) {
            a=&s_axes[i];
            if(fabs((double)a->position_counts-(double)s_camera_reference_positions[i])>32.0)moved=1U;
            s_camera_reference_positions[i]=a->position_counts;
            s_camera_reference_pos_seq[i]=a->position_sequence;s_camera_reference_flag_seq[i]=a->flags_sequence;
        }
        if(moved || !s_camera_reference_samples) { s_camera_reference_samples=1U;s_camera_reference_tick=now; }
        else if(s_camera_reference_samples<255U)s_camera_reference_samples++;
        if(s_camera_reference_samples<4U || now-s_camera_reference_tick<300U)return;
        s_axes[0].zero_counts=s_axes[0].upper_counts=s_axes[0].position_counts;
        s_axes[0].zero_valid=s_axes[0].upper_valid=1U;s_axes[0].lower_valid=0U;
        s_axes[1].zero_counts=s_axes[1].lower_counts=s_axes[1].position_counts;
        s_axes[1].zero_valid=s_axes[1].lower_valid=1U;s_axes[1].upper_valid=0U;
        Cal_Height(now);
        {
            char line[160];
            (void)snprintf(line,sizeof(line),"[ARM CAMERA] INIT confirmed: TOP5=%lld RETRACT6=%lld counts; RAM only\r\n",
                (long long)s_axes[0].zero_counts,(long long)s_axes[1].zero_counts);ZDT_Console_Print(line);
        }
        s_camera_state=ARM_CAMERA_PRECHECK;s_camera_step_started=now;
        return;
    }
    if(s_camera_state==ARM_CAMERA_PRECHECK) {
        if(!Cal_CameraAxisFresh(0U,now) || !Cal_CameraAxisFresh(1U,now)) {
            if(now-s_camera_step_started>=2000U)Cal_CameraFail("ZDT feedback missing/stale");
            return;
        }
        if((s_axes[0].flags&(ARM_CAL_FLAG_STALL|ARM_CAL_FLAG_PROTECTED)) || (s_axes[1].flags&ARM_CAL_FLAG_PROTECTED)) {
            Cal_CameraFail("ZDT stalled/protected before start");return;
        }
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK ||
           (ARM_LIFT_RANGE_STOP_ENABLED &&
            (current<-ARM_TOP_REFERENCE_TOLERANCE_MM/s_axes[0].travel_mm*100.0f || current>100.0f+ARM_PERCENT_TOLERANCE)) ||
           ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK ||
           current<-ARM_REACH_REFERENCE_TOLERANCE_MM/s_axes[1].travel_mm*100.0f ||
           current>100.0f+ARM_REACH_REFERENCE_TOLERANCE_MM/s_axes[1].travel_mm*100.0f) {
            Cal_CameraFail("zero/span/position reference inconsistent");return;
        }
        if(Cal_CameraReachHandoffBlocked(now))return;
        Cal_CameraEnable(0U,now);
        return;
    }
    /* Lift-first phase deliberately requires no enabled/zero DM: it must be
     * possible to raise out of the low-height angular restriction first. */
    if(s_camera_state==ARM_CAMERA_LIFT_ENABLE || s_camera_state==ARM_CAMERA_LIFT_ZERO) {
        a=&s_axes[0];
        if(!Cal_CameraAxisFresh(0U,now)) { Cal_CameraFail("ZDT feedback missing/stale during lift");return; }
        if(s_state==ARM_CAL_FAILED || (s_axes[0].flags&(ARM_CAL_FLAG_STALL|ARM_CAL_FLAG_PROTECTED)) ||
           (s_axes[1].flags&ARM_CAL_FLAG_PROTECTED)) {
            Cal_CameraFail("ZDT positioning fault during lift");return;
        }
        if(Cal_CameraAxisFresh(1U,now) && Cal_CameraReachHandoffBlocked(now))return;
        if(s_camera_state==ARM_CAMERA_LIFT_ENABLE) {
            Cal_CameraEnableRetry(0U,now);
            if(now-s_camera_step_started>=1000U) { Cal_CameraFail("lift enable feedback timeout");return; }
            if(a->position_sequence==s_camera_enable_position || a->flags_sequence==s_camera_enable_flags ||
               (int32_t)(a->position_tick-s_camera_step_started)<0 || (int32_t)(a->flags_tick-s_camera_step_started)<0 ||
               !(a->flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT))return;
            if(s_state==ARM_CAL_IDLE && ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)==HAL_OK &&
               isfinite(current) && fabsf(current)<=ArmCalibration_LiftTolerancePercent(0.0f,a->travel_mm)) {
                if(ArmDriver_Stop(ARM_DRIVER_LIFT)!=HAL_OK){Cal_CameraFail("TOP hold stop rejected");return;}
                s_camera_top_hold=1U;s_camera_top_samples=0U;s_camera_top_sequence=a->position_sequence;
                s_camera_top_counts=a->position_counts;s_camera_top_tick=HAL_GetTick();
                s_camera_state=ARM_CAMERA_LIFT_ZERO;
                ZDT_Console_Print("[ARM CAMERA] 1/3 TOP already arrived; STOP and verify NEW held samples; skip duplicate ascent\r\n");return;
            }
            if(s_camera_release_raise && ArmServo_GripOpeningOrOpen() &&
               ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)==HAL_OK && isfinite(current) && current>0.0f) {
                s_camera_raise_overlap=1U;s_camera_raise_half=current*0.5f;
            }
            if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)!=HAL_OK) { Cal_CameraFail("lift zero target rejected");return; }
            s_camera_state=ARM_CAMERA_LIFT_ZERO;
            ZDT_Console_Print("[ARM CAMERA] 1/3 lift moving to TOP 0%; DM/reach wait\r\n");return;
        }
        if(!(a->flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT)) { Cal_CameraFail("lift lost enable");return; }
        if(s_state!=ARM_CAL_IDLE) {
            if(s_camera_raise_overlap && ArmCalibration_IsLiftRaisingToTop() && ArmServo_GripOpenConfirmed() &&
               ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)==HAL_OK && isfinite(current) && current<=s_camera_raise_half) {
                if(!s_camera_dm_prepare_active)ZDT_Console_Print("[ARM CAMERA] released PLACE half-rise crossed; turn during remaining true TOP raise\r\n");
                Cal_CameraHome();
            }
            return;
        }
        if(s_camera_release_raise && !ArmServo_GripOpenConfirmed())return;
        s_camera_raise_overlap=0U;
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK) {Cal_CameraFail("lift zero feedback invalid");return;}
        if(fabsf(current)>ArmCalibration_LiftTolerancePercent(0.0f,a->travel_mm)) {
            if(s_camera_top_hold) {
                s_camera_top_hold=0U;
                if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)!=HAL_OK){Cal_CameraFail("TOP drift correction rejected");return;}
                ZDT_Console_Print("[ARM CAMERA] TOP hold drift; bounded near-TOP correction; DM/reach wait\r\n");return;
            }
            Cal_CameraFail("lift zero not confirmed");return;
        }
        if(s_camera_top_hold) {
            double stable;
            if(a->position_sequence==s_camera_top_sequence || (int32_t)(a->position_tick-s_camera_top_tick)<=0)return;
            s_camera_top_sequence=a->position_sequence;
            stable=fabs((double)a->position_counts-(double)s_camera_top_counts);s_camera_top_counts=a->position_counts;
            if(stable>fabs(s_percent_span[0])*ARM_PERCENT_TOLERANCE*0.3/100.0) {
                s_camera_top_samples=0U;s_camera_top_tick=now;return;
            }
            if(s_camera_top_samples<255U)++s_camera_top_samples;
            if(s_camera_top_samples<ARM_LINEAR_POST_STOP_SAMPLES || now-s_camera_top_tick<ARM_MOTION_SETTLE_MS)return;
            s_camera_top_hold=0U;
        }
        Cal_CameraHome();return;
    }
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    /* RX may have run after the caller captured now. Read the clock AFTER
     * the coherent feedback snapshot, or a fresh packet can underflow age. */
    now=HAL_GetTick();
    if(!ArmTask_IsEnabled() || ArmTask_IsFaulted() || !fb.fb_count ||
       now-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT || !fb.enabled ||
       (fb.err_code!=0U && fb.err_code!=1U) || !isfinite(fb.vel_rads) ||
       ArmDriver_GetRotationDeg(&angle)!=HAL_OK) { Cal_CameraFail("DM disabled/faulted/stale");return; }
    /* Only the axis actually moving/settling has an immediate feedback
     * deadline. DM_HOME holds both ZDT axes; they remain continuously queried. */
    if((s_camera_state==ARM_CAMERA_REACH_MOVE &&
        ((s_camera_position_mode && s_camera_native_samples!=255U) ||
         (!s_camera_position_mode && s_state!=ARM_CAL_IDLE)) &&
        !Cal_CameraAxisFresh(1U,now)) ||
       (s_camera_state==ARM_CAMERA_LIFT_FINAL && s_state!=ARM_CAL_IDLE &&
        !Cal_CameraAxisFresh(0U,now))) {
        Cal_CameraFail("ZDT feedback missing/stale");return;
    }
    if(s_state==ARM_CAL_FAILED) { Cal_CameraFail("ZDT positioning failed; see axis fault above");return; }
    if((s_axes[0].flags&(ARM_CAL_FLAG_STALL|ARM_CAL_FLAG_PROTECTED)) ||
       (s_axes[1].flags&ARM_CAL_FLAG_PROTECTED) ||
       (ArmCalibration_StallFault(ARM_DRIVER_REACH,s_axes[1].flags) && s_camera_state!=ARM_CAMERA_DM_HOME &&
        s_camera_state!=ARM_CAMERA_REACH_ENABLE && !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachStallPending())) {
        Cal_CameraFail("ZDT stalled/protected during sequence");return;
    }
    /* Post-release turn may overlap the remaining ascent. Reach/final camera
     * positioning still waits for the original true-TOP arrival confirmation. */
    if(s_camera_raise_overlap && s_state==ARM_CAL_IDLE)s_camera_raise_overlap=0U;
    if(!(s_axes[0].flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT) ||
       (s_camera_raise_overlap && (!ArmCalibration_IsLiftRaisingToTop() || !ArmServo_GripOpenConfirmed())) ||
       (Cal_CameraAxisFresh(0U,now) &&
        (ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK ||
         (s_camera_state!=ARM_CAMERA_LIFT_FINAL && fabsf(current)>(s_camera_raise_overlap?s_camera_raise_half:0.0f)+ARM_TOP_REFERENCE_TOLERANCE_MM/s_axes[0].travel_mm*100.0f)))) {
        Cal_CameraFail("lift did not hold TOP before DM/reach");return;
    }
    if(s_camera_state==ARM_CAMERA_REACH_MOVE && ArmDriver_ReachRetryRecovering())return;
    if(s_camera_state==ARM_CAMERA_DM_HOME) {
        if(s_camera_raise_overlap)return;
        if(Cal_CameraAxisFresh(1U,now) && Cal_CameraReachHandoffBlocked(now))return;
        if(!s_camera_feedback_wait && now-s_camera_step_started>=ArmSpeed_Timeout(10000U)) { Cal_CameraFail("DM home timeout");return; }
        if(fb.fb_count==s_camera_dm_sequence)return;
        s_camera_dm_sequence=fb.fb_count;
        if(fabsf(angle-s_camera_pose_deg[2])>1.0f || fabsf(fb.vel_rads)>0.05f) { s_camera_dm_settle_samples=0U;return; }
        if(!s_camera_dm_settle_samples)s_camera_dm_settle_tick=now;
        if(s_camera_dm_settle_samples<255U)s_camera_dm_settle_samples++;
        if(s_camera_dm_settle_samples>=ARM_MOTION_SETTLE_SAMPLES && now-s_camera_dm_settle_tick>=ARM_MOTION_SETTLE_MS) {
            if(!Cal_CameraFreshBeforeMotion(now))return;
            s_camera_dm_zero_hold_known=1U;
            s_camera_hold_sequence=fb.fb_count;s_camera_hold_samples=0U;
            ZDT_Console_Print("[ARM CAMERA] DM camera target confirmed; stays ENABLED; enabling reach\r\n");
            Cal_CameraEnable(1U,now);
        }
        return;
    }
    /* Homing needs tight settling. Under extension load, allow short small
     * disturbances while retaining immediate link/error and large-motion stops. */
    if(fabsf(angle-s_camera_pose_deg[2])>8.0f || fabsf(fb.vel_rads)>2.0f) { Cal_CameraFail("DM large deviation from camera target");return; }
    if(fb.fb_count!=s_camera_hold_sequence) {
        s_camera_hold_sequence=fb.fb_count;
        if(fabsf(angle-s_camera_pose_deg[2])>2.0f || fabsf(fb.vel_rads)>0.20f) {
            if(!s_camera_hold_samples)s_camera_hold_since=now;
            if(s_camera_hold_samples<255U)s_camera_hold_samples++;
            if(s_camera_hold_samples>=3U && now-s_camera_hold_since>=300U) {
                Cal_CameraFail("DM sustained deviation from camera target");return;
            }
        } else s_camera_hold_samples=0U;
    }
    if(s_camera_state==ARM_CAMERA_REACH_ENABLE) {
        uint8_t reacquiring=s_camera_feedback_wait;
        index=1U;a=&s_axes[index];
        if(!Cal_CameraFreshBeforeMotion(now))return;
        if(reacquiring)s_camera_step_started=now; /* Enable deadline excludes a bounded idle refresh. */
        if(a->flags&ARM_CAL_FLAG_PROTECTED) { Cal_CameraFail("ZDT stalled/protected after enable");return; }
        if(Cal_CameraReachHandoffBlocked(now))return;
        Cal_CameraEnableRetry(1U,now);
        if(now-s_camera_step_started>=1000U) { Cal_CameraFail("ZDT enable feedback timeout");return; }
        if(a->position_sequence==s_camera_enable_position || a->flags_sequence==s_camera_enable_flags ||
           (int32_t)(a->position_tick-s_camera_step_started)<0 || (int32_t)(a->flags_tick-s_camera_step_started)<0 ||
           !(a->flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH))return;
        if((index && s_camera_position_mode?Cal_CameraNativeStart(now):
            ArmCalibration_MovePercent(index?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,index?s_camera_reach_percent:0.0f))!=HAL_OK) {
            Cal_CameraFail("ZDT target rejected");return;
        }
        s_camera_state=ARM_CAMERA_REACH_MOVE;
        if(index) { s_camera_dm_settle_samples=0U;s_camera_dm_sequence=fb.fb_count; }
        ZDT_Console_Print("[ARM CAMERA] 3/3 reach moving to configured camera angle\r\n");
        return;
    }
    if(s_camera_state==ARM_CAMERA_REACH_MOVE && s_camera_position_mode && s_camera_native_samples!=255U) {
        Cal_CameraNativePoll(now);
        if(s_camera_state==ARM_CAMERA_FAILED || s_camera_native_samples!=255U)return;
    }
    if((s_camera_state==ARM_CAMERA_REACH_MOVE || s_camera_state==ARM_CAMERA_LIFT_FINAL) &&
       (!(s_axes[1].flags&ARM_CAL_FLAG_ENABLED) || !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH))) {
        Cal_CameraFail("reach did not hold enabled");return;
    }
    if(s_state!=ARM_CAL_IDLE)return; /* ARRIVED requires fresh stopped feedback. */
    if(s_camera_state==ARM_CAMERA_REACH_MOVE) {
        if(!Cal_CameraFreshBeforeMotion(now))return;
        /* Native/percent movement has completed. Final target residual is
         * diagnostic only; keep the fresh-feedback requirement. */
        if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK) {
            Cal_CameraFail("reach final feedback unavailable");return;
        }
        /* READY must include a settled, enabled DM at zero, even if extension
         * finished during a brief allowed load disturbance. */
        if(fabsf(angle-s_camera_pose_deg[2])>1.0f || fabsf(fb.vel_rads)>0.05f) { s_camera_dm_settle_samples=0U;return; }
        if(fb.fb_count!=s_camera_dm_sequence) {
            s_camera_dm_sequence=fb.fb_count;
            if(!s_camera_dm_settle_samples)s_camera_dm_settle_tick=now;
            if(s_camera_dm_settle_samples<255U)s_camera_dm_settle_samples++;
        }
        if(s_camera_dm_settle_samples<ARM_MOTION_SETTLE_SAMPLES || now-s_camera_dm_settle_tick<ARM_MOTION_SETTLE_MS)return;
        if(fabsf(s_camera_lift_percent)>ARM_PERCENT_TOLERANCE) {
            if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,s_camera_lift_percent)!=HAL_OK) {
                Cal_CameraFail("final camera lift target rejected");return;
            }
            s_camera_state=ARM_CAMERA_LIFT_FINAL;return;
        }
        s_camera_state=ARM_CAMERA_DONE;
    }
    if(s_camera_state==ARM_CAMERA_LIFT_FINAL && s_state==ARM_CAL_IDLE) {
        if(!Cal_CameraFreshBeforeMotion(now))return;
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK ||
           fabsf(current-s_camera_lift_percent)>ArmCalibration_LiftTolerancePercent(s_camera_lift_percent,s_axes[0].travel_mm)) {
            Cal_CameraFail("final camera lift not confirmed");return;
        }
        if(fabsf(angle-s_camera_pose_deg[2])>1.0f || fabsf(fb.vel_rads)>0.20f)return;
        s_camera_state=ARM_CAMERA_DONE;
    }
    if(s_camera_state==ARM_CAMERA_DONE) {
        char line[150];(void)snprintf(line,sizeof(line),
            "[ARM CAMERA] READY: lift=%+.2fdeg reach=%+.2fdeg DM=%+.2fdeg; motors hold\r\n",
            (double)s_camera_pose_deg[0],(double)s_camera_active_reach_deg,(double)s_camera_pose_deg[2]);ZDT_Console_Print(line);
    }
}

void ArmCalibration_Poll(void)
{
    uint32_t now;
    arm_cal_axis_status_t *a;
    if (Cal_Context() == 0U) { return; }
    s_loop_enter=HAL_GetTick();s_loop_gap=s_loop_seen?s_loop_enter-s_loop_last:0U;
    s_loop_seen=1U;s_loop_last=s_loop_enter;
    ArmServo_Update(HAL_GetTick());
    Cal_DrainPorts();
    ArmDriver_PollReachRetry();
    now=HAL_GetTick();
    if(Cal_FeedbackWait(now)) {
        ArmCalibration_ServiceIo();s_loop_previous_work=HAL_GetTick()-s_loop_enter;return;
    }
    if(ArmDriver_ReachRetryFailed()) {
        if(Cal_CameraActive())Cal_CameraFail("reach native positioning failed after retry recovery");
        else {
            (void)ArmDriver_StopAll();s_percent_rpm=0;s_state=ARM_CAL_FAILED;
        }
        return;
    }
    now = HAL_GetTick();
    Cal_Height(now);
    if ((!ARM_ZDT_WAIT_ON_STALE || ArmCalibration_ZdtFeedbackFresh()) && ArmDriver_CheckHeightInterlock() != HAL_OK)
    {
        if (Cal_IsActive() != 0U) { Cal_Fail("height interlock"); }
        if(Cal_CameraActive())Cal_CameraFail("height interlock");
        ZDT_Console_Print("[ARM] height interlock: DM disable and lift stop requested\r\n");
    }
    if ((Cal_IsActive() != 0U) && ((now - s_started) >=
        ((s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE)?s_percent_timeout:s_cfg.seek_timeout_ms)))
    {
        Cal_Fail((s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE)?"percent motion timeout":"endpoint timeout");
    }
    a = &s_axes[s_active];
    if(s_state==ARM_CAL_PERCENT_MOVE || s_state==ARM_CAL_PERCENT_SETTLE) { Cal_PercentPoll(now); }
    else if (s_state == ARM_CAL_PRECHECK)
    {
        if ((now - s_started) > s_cfg.feedback_timeout_ms) { Cal_Fail("no fresh baseline feedback"); }
        else if ((a->flags_sequence != s_flags_baseline) && (a->position_sequence != s_position_baseline))
        {
            if ((s_seek != 0U) && ((a->flags & (ARM_CAL_FLAG_STALL | ARM_CAL_FLAG_PROTECTED)) != 0U))
            {
                Cal_Fail("old stall/protection active; clear explicitly before seek");
            }
            else if ((s_seek != 0U) && ((a->flags & ARM_CAL_FLAG_ENABLED) == 0U))
            {
                Cal_Fail("motor did not confirm enable");
            }
            else if (s_seek != 0U)
            {
                uint8_t reverse = (s_active == 0U) ? s_cfg.lift_reverse : s_cfg.reach_reverse;
                uint8_t direction = (uint8_t)(((s_upper == 0U) ? 1U : 0U) ^ reverse);
                uint8_t data[7] = {0xF6U, direction,
                    (uint8_t)(s_cfg.seek_speed_rpm >> 8U), (uint8_t)s_cfg.seek_speed_rpm,
                    s_cfg.acceleration, 0U, ZDT_CAN_CHECKSUM};
                if (Cal_Send(s_active, data, FDCAN_DLC_BYTES_7) != HAL_OK) { Cal_Fail("seek send failed"); }
                else
                {
                    s_state = ARM_CAL_SEEKING;
                    s_motion_started = now;
                    s_flags_baseline = a->flags_sequence;
                    Cal_Report(s_active, "seek started");
                }
            }
            else
            {
                s_state = ARM_CAL_SETTLING;
                s_position_baseline = a->position_sequence;
                s_settle_position = a->position_counts;
                s_settle_samples = 0U;
            }
        }
    }
    else if (s_state == ARM_CAL_SEEKING)
    {
        if (((now - a->flags_tick) > ArmCalibration_StatusTimeoutMs(Cal_Axis())) ||
            ((now - a->position_tick) > s_cfg.feedback_timeout_ms)) { Cal_Fail("seek feedback lost"); }
        else if (a->flags_sequence != s_flags_baseline)
        {
            s_flags_baseline = a->flags_sequence;
            if ((a->flags & (ARM_CAL_FLAG_STALL | ARM_CAL_FLAG_PROTECTED)) == 0U)
            {
                s_stall_samples = 0U;
                if ((a->flags & ARM_CAL_FLAG_ENABLED) == 0U) { Cal_Fail("motor disabled during seek"); }
            }
            else if ((now - s_motion_started) >= s_cfg.stall_grace_ms)
            {
                ++s_stall_samples;
                if (s_stall_samples >= 2U)
                {
                    if (ArmDriver_Stop(Cal_Axis()) != HAL_OK) { Cal_Fail("endpoint stop send failed"); }
                    else
                    {
                        s_state = ARM_CAL_SETTLING;
                        s_position_baseline = a->position_sequence;
                        s_settle_position = a->position_counts;
                        s_settle_samples = 0U;
                        Cal_Report(s_active, "fresh stall; stop queued");
                    }
                }
            }
        }
    }
    else if (s_state == ARM_CAL_SETTLING)
    {
        if ((now - a->position_tick) > s_cfg.feedback_timeout_ms) { Cal_Fail("no final position feedback"); }
        else if (a->position_sequence != s_position_baseline)
        {
            s_position_baseline = a->position_sequence;
            if (a->position_counts == s_settle_position) { ++s_settle_samples; }
            else { s_settle_position = a->position_counts; s_settle_samples = 0U; }
            if (s_settle_samples >= 2U)
            {
                if(s_drop_record) {
                    int64_t delta=a->position_counts-a->upper_counts;
                    s_drop_record=0;
                    if(delta==0) { Cal_Fail("drop calibration has zero travel"); }
                    else {
                        char line[144];
                        s_down_counts_per_mm=(double)delta/s_drop_cal_mm;s_state=ARM_CAL_IDLE;
                        (void)snprintf(line,sizeof(line),"[ARM] down feedback=%.6f counts/mm; reuse: arm feedback 5 %.6f\r\n",
                            s_down_counts_per_mm,s_down_counts_per_mm);ZDT_Console_Print(line);
                    }
                } else if(s_zero_record) {
                    s_zero_record=0;a->zero_counts=a->position_counts;a->zero_valid=1U;s_state=ARM_CAL_IDLE;
                    Cal_Report(s_active,"WORK ZERO RECORDED");
                } else {
                if (s_upper != 0U) { a->upper_counts = a->position_counts; a->upper_valid = 1U; }
                else { a->lower_counts = a->position_counts; a->lower_valid = 1U; }
                if ((a->lower_valid != 0U) && (a->upper_valid != 0U) && (a->lower_counts == a->upper_counts))
                {
                    a->lower_valid = 0U; a->upper_valid = 0U;
                    Cal_Fail("zero endpoint span");
                }
                else
                {
                    if(s_active==0U && s_upper) { a->zero_counts=a->position_counts;a->zero_valid=1U; }
                    s_state = ARM_CAL_IDLE;
                    Cal_Report(s_active, (s_upper != 0U) ? "UPPER RECORDED" : "LOWER RECORDED");
                }
                }
            }
        }
    }
    Cal_Height(now); /* reference/scale may have completed in this tick */
    if ((now - s_last_query) >= s_cfg.query_period_ms)
    {
        s_last_query = now;
        Cal_Query(0U);
        Cal_Query(1U);
    }
    Cal_QueryService(now);
    Cal_CameraPoll(HAL_GetTick());
    s_loop_previous_work=HAL_GetTick()-s_loop_enter;
}

static uint8_t Cal_Float(const char *token, float *out)
{
    char *end;
    if (token == NULL) { return 0U; }
    *out = strtof(token, &end);
    return ((end != token) && (*end == '\0') && isfinite(*out)) ? 1U : 0U;
}
static uint8_t Cal_Integer(const char *token, int32_t *out)
{
    char *end;
    double value;
    if (token == NULL) { return 0U; }
    value = strtod(token, &end);
    if ((end == token) || (*end != '\0') || !isfinite(value) ||
        (value > 2147483647.0) || (value < -2147483647.0) || (floor(value) != value)) { return 0U; }
    *out = (int32_t)value;
    return 1U;
}
static uint8_t Cal_ParseAxis(const char *token, arm_driver_axis_t *axis)
{
    if (strcmp(token, "5") == 0) { *axis = ARM_DRIVER_LIFT; return 1U; }
    if (strcmp(token, "6") == 0) { *axis = ARM_DRIVER_REACH; return 1U; }
    if (strcmp(token, "dm") == 0) { *axis = ARM_DRIVER_ROTATION; return 1U; }
    return 0U;
}

/* Console-owner controls; the existing emergency disable remains immediate. */
static HAL_StatusTypeDef Cal_DmOperation(uint8_t op)
{
    static const char *const names[]={"ON","OFF","SOFT ZERO","ZERO SAVE","QUERY","HOME"};
    HAL_StatusTypeDef result;dm_feedback_t fb;char reply[192];dm_motor_t *motor=ArmTask_GetMotor();
    if(!ARM_DM_SOFTWARE_ZERO_ENABLED && op==2U) {
        ZDT_Console_Print("[DM] software zero disabled; place at mechanical zero, then dm off and dm zero save (motor internal flash)\r\n");return HAL_ERROR;
    }
    if(Cal_IsActive()) {
        if(op!=1U && op!=4U){ZDT_Console_Print("[DM] busy: finish arm calibration or arm stop first.\r\n");return HAL_BUSY;}
        if(op==1U) {
            result=ArmDriver_Disable(ARM_DRIVER_ROTATION);
            ZDT_Console_Print("[DM] OFF requested during calibration; feedback confirmation pending.\r\n");return result;
        }
    }
    if(op==0U)result=ArmDriver_Enable(ARM_DRIVER_ROTATION);
    else if(op==1U)result=ArmDriver_DisableRotationConfirmed();
    else if(op<4U)result=ArmDriver_ZeroRotation((uint8_t)(op==3U));
    else if(op==4U)result=DM_Motor_ReadPosition(motor,300U,&fb);
    else result=ArmDriver_ReturnRotationZero();
    if(result==HAL_OK) {
        if(op==0U)ZDT_Console_Print("[DM] ON confirmed; motor enabled.\r\n");
        else if(op==1U)ZDT_Console_Print("[DM] OFF confirmed by fresh motor feedback.\r\n");
        else if(op==2U)ZDT_Console_Print("[DM] SOFT ZERO OK: current arm angle=0; RAM only, stays OFF.\r\n");
        else if(op==3U)ZDT_Console_Print("[DM] ZERO SAVE OK: raw zero feedback confirmed; stays OFF. Power-cycle motor to verify persistence.\r\n");
        else if(op==5U)ZDT_Console_Print("[DM] HOME queued: target arm 0deg; dm query checks actual angle.\r\n");
        else {
            float angle;double raw_deg=(double)fb.pos_rad*57.29577951308232;
            if(ArmDriver_MapRawRotation(fb.pos_rad,&angle)==HAL_OK)
                (void)snprintf(reply,sizeof(reply),"[DM] QUERY raw_deg=%+.4f arm_deg=%+.4f raw_rad=%+.6f age=%lums enabled=%u code=%u\r\n",
                    raw_deg,(double)angle,(double)fb.pos_rad,(unsigned long)(HAL_GetTick()-fb.last_tick),fb.enabled,fb.err_code);
            else (void)snprintf(reply,sizeof(reply),"[DM] QUERY raw_deg=%+.4f arm_deg=UNKNOWN raw_rad=%+.6f age=%lums enabled=%u code=%u\r\n",
                    raw_deg,(double)fb.pos_rad,(unsigned long)(HAL_GetTick()-fb.last_tick),fb.enabled,fb.err_code);
            ZDT_Console_Print(reply);
        }
    } else {
        DM_GetFeedback(motor,&fb);
        (void)snprintf(reply,sizeof(reply),"[DM] %s FAILED HAL=%u state=%u fb=%lu age=%lums enabled=%u code=%u raw=%.3f vel=%.3f ref=%u\r\n",
            names[op],(unsigned int)result,(unsigned int)motor->state,(unsigned long)fb.fb_count,
            (unsigned long)(fb.fb_count?HAL_GetTick()-fb.last_tick:UINT32_MAX),fb.enabled,fb.err_code,
            (double)fb.pos_rad,(double)fb.vel_rads,ArmDriver_IsRotationReferenceValid());
        ZDT_Console_Print(reply);
        if(op==0U)ZDT_Console_Print("[DM] check POS mode, fresh CAN feedback, motor fault and height/angle limit; dm status for details.\r\n");
        else if(!ArmDriver_IsRotationReferenceValid())ZDT_Console_Print("[DM] zero reference unknown; motor stays OFF. Retry dm zero/save after link recovers.\r\n");
        else if(op==4U)ZDT_Console_Print("[DM] no fresh position reply; target/enable unchanged.\r\n");
        else if(op==5U)ZDT_Console_Print("[DM] HOME requires valid zero, fresh position and permitted height/angle.\r\n");
        else ZDT_Console_Print("[DM] need disabled, stationary, fresh feedback; no confirmed change.\r\n");
    }
    return result;
}
/* Individual disable cancels the composite job but releases only the requested
 * motor. Other axes stop their motion and retain their holding torque. */
static void Cal_CancelKeepingHolds(void)
{
    HAL_StatusTypeDef result,next;float angle;
    ArmRound1_AbortState();ArmMaterial_AbortState();ArmStartPose_Cancel();s_camera_state=ARM_CAMERA_IDLE;s_camera_native_pending=0U;
    result=ArmDriver_Stop(ARM_DRIVER_LIFT);next=ArmDriver_Stop(ARM_DRIVER_REACH);
    if(result==HAL_OK)result=next;
    if(ArmTask_IsEnabled() && ArmDriver_GetRotationDeg(&angle)==HAL_OK) {
        next=ArmDriver_RotateToDeg(angle);if(result==HAL_OK)result=next;
    }
    s_state=result==HAL_OK?ARM_CAL_IDLE:ARM_CAL_FAILED;
    s_drop_record=s_zero_record=s_percent_rpm=0;
    ZDT_Console_Print(result==HAL_OK?
        "[ARM] pose cancelled; other axes stopped and retain hold\r\n":
        "[ARM] pose cancellation send failed; requested axis disable still proceeds; arm status\r\n");
}
uint8_t ArmCalibration_ProcessCommand(const char *line)
{
    char copy[96];
    char *args[6];
    char *p;
    uint8_t count = 0U;
    arm_driver_axis_t axis;
    HAL_StatusTypeDef result = HAL_ERROR;
    float x, y, z;
    int32_t pulses;
    char reply[192];
    if(line==NULL)return 0U;
    if(!strncmp(line,"arm rates",9U) && (!line[9] || line[9]==' ')) {
        if(!Cal_Context())return 1U;
        return ArmSpeed_Command(line);
    }
    if(!strcmp(line,"arm ui")) {
        uint8_t i;uint32_t now=HAL_GetTick();
        ZDT_Console_Print("[ARM UI BEGIN] v=1 firmware=306 variant=" COMPETITION_VARIANT " disc_profiles=1 disc_pose_count=4\r\n");
        (void)snprintf(reply,sizeof(reply),"[ARM UI STATE] busy=%u pose=%u calibration=%u material=%u round=%u shared_ring=%u param_bank=" COMPETITION_VARIANT " boot_params=%s\r\n",
            (unsigned int)(ArmStartPose_IsActive() || Cal_CameraActive() || Cal_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive()),
            ArmStartPose_IsActive(),(unsigned int)s_state,ArmMaterial_IsActive(),ArmRound1_IsActive(),ArmStartPose_RingShared(),ArmStartPose_BootParameterSource());ZDT_Console_Print(reply);
        for(i=0U;i<2U;i++) {
            arm_cal_axis_status_t *a=&s_axes[i];
            uint8_t valid=(uint8_t)(a->zero_valid && a->position_sequence && now-a->position_tick<=s_cfg.feedback_timeout_ms);
            (void)snprintf(reply,sizeof(reply),"[ARM UI AXIS] id=%u valid=%u angle=%.4f span=%.4f travel=%.4f enabled=%u\r\n",
                Cal_Address(i),valid,(double)(a->position_counts-a->zero_counts)*360.0/65536.0,
                s_percent_span[i],(double)a->travel_mm,(unsigned int)((a->flags&ARM_CAL_FLAG_ENABLED)!=0U));ZDT_Console_Print(reply);
        }
        ArmDriver_PrintUi();ArmServo_PrintStatus();ZDT_Console_Print("[ARM UI END]\r\n");return 1U;
    }
    if(!strcmp(line,"arm points")){ArmStartPose_PrintUiPoints();return 1U;}
    if(ArmRound1_Command(line))return 1U;
    if(ArmRound1_IsActive()) {
        uint8_t release=(uint8_t)(strncmp(line,"arm disable ",12U)==0 || strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0 ||
                                  strcmp(line,"arm grip disable")==0 || strcmp(line,"arm table disable")==0);
        uint8_t allowed=(uint8_t)(release || strcmp(line,"arm stop")==0 || strcmp(line,"arm material stop")==0 ||
            strcmp(line,"arm material status")==0 || strncmp(line,"arm material window ",20U)==0 ||
            strncmp(line,"arm material fresh ",19U)==0 || strncmp(line,"arm material confirm ",21U)==0 ||
            strncmp(line,"arm material jump ",18U)==0 || strncmp(line,"arm query ",10U)==0 ||
            strcmp(line,"arm status")==0 || strcmp(line,"arm pick status")==0 || strcmp(line,"arm place status")==0 ||
            strcmp(line,"arm ring status")==0 || strcmp(line,"arm grip status")==0 || strcmp(line,"arm table status")==0 ||
            strcmp(line,"dm status")==0 || strcmp(line,"dm query")==0);
        if(!allowed){ZDT_Console_Print("[ROUND1] station task owns arm; status/query/window, disable or STOP available\r\n");return 1U;}
        if(release)Cal_CancelKeepingHolds();
    }
    if(ArmMaterial_Command(line))return 1U;
    if(ArmMaterial_IsActive() && (strcmp(line,"arm grip disable")==0 || strcmp(line,"arm table disable")==0 ||
       strncmp(line,"arm disable ",12U)==0 || strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0)) {
        /* Material now owns an independent native reach move alongside lift.
         * Cancel BOTH before releasing just the explicitly requested axis. */
        Cal_CancelKeepingHolds();
    }
    if(ArmMaterial_IsActive() && strcmp(line,"arm stop") && strncmp(line,"arm disable ",12U) &&
       strcmp(line,"dm off") && strcmp(line,"dm disable") && strncmp(line,"arm query ",10U) &&
       strncmp(line,"arm angle ",10U) && strcmp(line,"arm status") && strcmp(line,"arm camera status") && strcmp(line,"dm status") && strcmp(line,"dm query")) {
        ZDT_Console_Print("[MAT] material/color job owns arm; status/query, disable or arm material/color stop available\r\n");return 1U;
    }
    if(ArmMaterial_IsActive() && (strcmp(line,"arm stop")==0 || strncmp(line,"arm disable ",12U)==0 ||
       strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0))ArmMaterial_AbortState();

    if((ArmStartPose_IsActive() || Cal_CameraActive()) &&
       (strncmp(line,"arm disable ",12U)==0 || strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0)) {
        Cal_CancelKeepingHolds();
    }
    if((ArmStartPose_IsActive() && strcmp(line,"arm camera")==0) ||
       (Cal_CameraActive() && strcmp(line,"arm start")==0)) {
        if(ArmCalibration_Cancel()!=HAL_OK) {
            ZDT_Console_Print("[ARM] pose switch stop failed; arm status\r\n");return 1U;
        }
    }
    /* Read-only snapshots remain available while the pose sequencer is busy. */
    if(sscanf(line,"arm query %5s %c",copy,reply)==1 ||
       sscanf(line,"arm angle %5s %c",copy,reply)==1) {
        if(!Cal_ParseAxis(copy,&axis)) {
            ZDT_Console_Print("[ARM QUERY] usage: arm query 5|6|dm\r\n");return 1U;
        }
        if(axis==ARM_DRIVER_ROTATION){(void)Cal_DmOperation(4U);return 1U;}
        {
            uint8_t i=(uint8_t)Cal_Index(axis);arm_cal_axis_status_t *a=&s_axes[i];
            uint32_t age=HAL_GetTick()-a->position_tick;
            if(!a->position_sequence || !a->zero_valid || age>s_cfg.feedback_timeout_ms) {
                Cal_Query(i);ZDT_Console_Print("[ARM QUERY] waiting fresh position/zero; retry query or arm zero axis\r\n");return 1U;
            }
            (void)snprintf(reply,sizeof(reply),
                "[ARM QUERY] id=%u angle=%+.4fdeg raw=%lld zero=%lld age=%lums flags=0x%02X flags_age=%lums\r\n",
                (unsigned int)Cal_Address(i),(double)(a->position_counts-a->zero_counts)*360.0/65536.0,
                (long long)a->position_counts,(long long)a->zero_counts,(unsigned long)age,
                (unsigned int)a->flags,(unsigned long)(HAL_GetTick()-a->flags_tick));ZDT_Console_Print(reply);
            Cal_Query(i);return 1U;
        }
    }
    if(ArmStartPose_IsActive()) {
        if(strcmp(line,"arm stop")==0 || strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0 ||
           strncmp(line,"arm disable ",12U)==0) {ArmStartPose_Cancel();(void)ArmCalibration_Cancel();}
        else if(strcmp(line,"arm start status")!=0 && strcmp(line,"arm pick status")!=0 && strcmp(line,"arm place status")!=0 && strcmp(line,"arm ring status")!=0 && strcmp(line,"arm status")!=0 && strcmp(line,"dm status")!=0) {
            ZDT_Console_Print("[ARM START] busy; arm start status or arm stop\r\n");return 1U;
        }
    }
    if(ArmStartPose_Command(line))return 1U;
    if(DM_Motor_DiagIsActive(ArmTask_GetMotor())) {
        if(strcmp(line,"arm stop")==0 || strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0 ||
           strcmp(line,"dm stop")==0 || strncmp(line,"arm disable ",12U)==0)DM_Motor_DiagCancel(ArmTask_GetMotor());
        else if(strcmp(line,"dm diag")!=0 && strcmp(line,"dm diag status")!=0 && strcmp(line,"dm status")!=0 &&
                strcmp(line,"dm help")!=0 && strcmp(line,"arm status")!=0 && strcmp(line,"arm help")!=0) {
            ZDT_Console_Print("[DM DIAG] busy: wait DONE, query status, or arm stop\r\n");return 1U;
        }
    }
    if(Cal_CameraActive()) {
        if(strncmp(line,"arm disable ",12U)==0 || strcmp(line,"dm off")==0 ||
           strcmp(line,"dm disable")==0 || strcmp(line,"dm stop")==0) {
            (void)ArmCalibration_Cancel();
        } else if(strcmp(line,"arm camera")!=0 && strcmp(line,"arm camera status")!=0 &&
                  strcmp(line,"arm status")!=0 && strcmp(line,"arm stop")!=0 &&
                  strcmp(line,"arm help")!=0 && strcmp(line,"dm status")!=0 && strcmp(line,"dm help")!=0) {
            ZDT_Console_Print("[ARM CAMERA] busy; arm camera status or arm stop\r\n");return 1U;
        }
    }
    if(strncmp(line,"dm",2U)==0 && (line[2]==' ' || line[2]=='\0')) {
        if(!Cal_Context())return 1U;
        if(strncmp(line,"dm zero w25",11U)==0 && (line[11]==' ' || line[11]=='\0')) {
            ZDT_Console_Print("[DM] W25 zero support REMOVED; use dm off then dm zero save for motor internal zero\r\n");return 1U;
        }
        if(strcmp(line,"dm")==0 || strcmp(line,"dm help")==0) {
            ZDT_Console_Print("[DM] diag: read-only async firmware/Gr/scales/p_m/xout scan; diag status\r\n");
            ZDT_Console_Print("[DM] on/off: enable or confirmed disable; query/angle: fresh actual angle\r\n");
            ZDT_Console_Print("[DM] zero save/confirm: save motor INTERNAL zero, stays OFF; home: motor raw 0deg\r\n");
            ZDT_Console_Print("[DM] home: safely enable if needed, then move to current arm 0deg\r\n");
            ZDT_Console_Print("[DM] status/target/probe/recover/clear: diagnostics and explicit recovery\r\n");return 1U;
        }
        if(strcmp(line,"dm diag")==0 || strcmp(line,"dm diag status")==0) {
            if(strcmp(line,"dm diag")==0) {
                result=Cal_IsActive()?HAL_BUSY:DM_Motor_DiagStart(ArmTask_GetMotor());
                (void)snprintf(reply,sizeof(reply),"[DM DIAG] start HAL=%u; read-only, require still motor\r\n",(unsigned int)result);
                ZDT_Console_Print(reply);
            }
            {
                dm_motor_t *m=ArmTask_GetMotor();float zero=0;
                (void)ArmDriver_GetRotationZeroRaw(&zero);
                (void)snprintf(reply,sizeof(reply),"[DM DIAG] active=%u raw_ref=%.6f target=%.6f offset=%.6f compiled PMAX=%.3f VMAX=%.3f TMAX=%.3f\r\n",
                    DM_Motor_DiagIsActive(m),(double)zero,(double)m->target_pos_rad,(double)m->pos_offset_rad,
                    (double)DM_P_MAX,(double)DM_V_MAX,(double)DM_T_MAX);ZDT_Console_Print(reply);return 1U;
            }
        }
        if(strcmp(line,"dm on")==0 || strcmp(line,"dm enable")==0){(void)Cal_DmOperation(0U);return 1U;}
        if(strcmp(line,"dm off")==0 || strcmp(line,"dm disable")==0){(void)Cal_DmOperation(1U);return 1U;}
        if(strcmp(line,"dm zero")==0 || strcmp(line,"dm zerohere")==0){(void)Cal_DmOperation(2U);return 1U;}
        if(strcmp(line,"dm zero save")==0 || strcmp(line,"dm zero confirm")==0){(void)Cal_DmOperation(3U);return 1U;}
        if(strcmp(line,"dm query")==0 || strcmp(line,"dm angle")==0){(void)Cal_DmOperation(4U);return 1U;}
        if(strcmp(line,"dm home")==0){(void)Cal_DmOperation(5U);return 1U;}
        if(Cal_IsActive() && strcmp(line,"dm status")!=0 && strcmp(line,"dm target")!=0 && strcmp(line,"dm disable")!=0) {
            ZDT_Console_Print("[ARM] calibration busy\r\n");return 1U;
        }
        if(strncmp(line,"dm raw",6)==0 || strncmp(line,"dm mit",6)==0 || strncmp(line,"dm speed",8)==0 || strncmp(line,"dm zero",7)==0) {
            ZDT_Console_Print("[ARM] use arm zero/rotate; raw/MIT/speed/flash-zero bypass the height limit\r\n");return 1U;
        }
        if(strcmp(line,"dm enable")==0) {
            result=ArmDriver_Enable(ARM_DRIVER_ROTATION);
            (void)snprintf(reply,sizeof(reply),"[ARM] DM enable HAL=%u\r\n",(unsigned int)result);
            ZDT_Console_Print(reply);return 1U;
        }
        if(strncmp(line,"dm pos ",7)==0) {
            char *end;float pos=strtof(line+7,&end);
            if(end==line+7 || !isfinite(pos) || ArmDriver_CheckRawRotationTarget(pos+ArmTask_GetMotor()->pos_offset_rad)!=HAL_OK) {
                ZDT_Console_Print("[ARM] DM target rejected by height/angle limit\r\n");return 1U;
            }
        }
        (void)DM_Console_ProcessCommand(ArmTask_GetMotor(),line);
        if(strncmp(line,"dm pos ",7)==0 && ArmTask_GetMotor()->last_err!=HAL_OK)ArmTask_EmergencyStop();
        (void)ArmDriver_CheckHeightInterlock();return 1U;
    }
    if ((strncmp(line, "arm", 3U) != 0) ||
        ((line[3] != '\0') && (line[3] != ' '))) { return 0U; }
    if (Cal_Context() == 0U) { return 1U; }
    if (strlen(line) >= sizeof(copy)) { ZDT_Console_Print("[ARM] command too long\r\n"); return 1U; }
    (void)strcpy(copy, line);
    p = copy;
    while (*p != '\0')
    {
        while (*p == ' ') { ++p; }
        if (*p == '\0') { break; }
        if (count >= 6U) { ZDT_Console_Print("[ARM] too many arguments\r\n"); return 1U; }
        args[count++] = p;
        while ((*p != '\0') && (*p != ' ')) { ++p; }
        if (*p != '\0') { *p++ = '\0'; }
    }
    if ((count == 1U) || ((count == 2U) && (strcmp(args[1], "help") == 0)))
    {
        ZDT_Console_Print("arm ui/points; arm move <lift_deg> <reach_deg> <dm_deg>; arm params save|load|status\r\n");
        ZDT_Console_Print("arm trim rough|temp <1..3> <dl> <dr> <dd>; arm ring rough|temp <1..3>\r\n");
        ZDT_Console_Print("arm grip preset <open> <close>; arm table presets <slot1> <slot2> <slot3>; RAM, no motion\r\n");
        ZDT_Console_Print("arm camera [init|status|defaults]; arm camera mode position|encoder; POSITION 3200steps/rev (1.8deg/16); arm stop cancels\r\n");
        ZDT_Console_Print(ARM_28_STALL_IGNORE_TRIAL?
        "arm enable/disable 5|6|dm; arm pos 6 <deg> (native ABS, ID6 STALL_IGNORE trial); arm stop; arm status\r\n":
        "arm enable/disable 5|6|dm; arm pos 6 <deg> (native ABS, max3 stall attempts); arm stop; arm status\r\n");
        ZDT_Console_Print("dm on/off; dm zero save (motor INTERNAL flash, stays OFF); W25 zero ignored in internal mode\r\n");
        ZDT_Console_Print("dm query/angle (fresh actual angle); dm home (move to arm zero)\r\n");
        ZDT_Console_Print("arm mark/seek 5|6 min|max; arm clear 5|6\r\n");
        ZDT_Console_Print("arm pulse/mm 5|6 <signed>; arm rotate/by <deg>\r\n");
        ZDT_Console_Print("arm pct 5|6 [0..100]: absolute travel position; no auto-enable\r\n");
        ZDT_Console_Print("arm span 5|6 <signed_feedback_counts>; defaults: 5=-219505 6=-132619.378\r\n");
        ZDT_Console_Print("arm travel/scale 5|6 <value>; arm zero\r\n");
        ZDT_Console_Print("arm top 5; arm dropcal <measured_down_mm>; arm feedback 5 <signed_counts/mm>\r\n");
        ZDT_Console_Print("arm zero dm|5|6; arm dir 5|6 0|1; arm speed 5|6 <rpm>\r\n");
        ZDT_Console_Print("arm query 5|6|dm; arm zero dm writes INTERNAL zero; 5/6 record RAM references\r\n");
        ZDT_Console_Print("arm start/camera set <lift_deg> <reach_deg> <dm_deg>; RAM targets; prepare 1|2 then OK\r\n");
        ZDT_Console_Print("arm guardtop <drop_mm> <min_deg> <max_deg>; arm guardoff\r\n");
        ZDT_Console_Print("arm limit <min_deg> <max_deg>; arm guard <height_mm> <min_deg> <max_deg>\r\n");
        ZDT_Console_Print("arm grip test <0..360deg> (enable+target); angle <deg> after enable; open225/close308; status\r\n");
        ZDT_Console_Print("arm material [status|stop|loop start/stop/status|loop limit 0..1000|auto on/off|window gx gy wx wy]; arm ring 1|2|3 (after PICK); arm pick/place; arm pick/place status; place requires closed grip\r\n");
        ZDT_Console_Print("arm color [1..6] -> CAMERA READY then CAM1 touch calibration; arm color status/stop; save colors on screen\r\n");
        ZDT_Console_Print("arm table enable/disable; arm table go/save 1|2|3 (19/109/199deg); arm table angle <0..360deg>; test <0..385deg> (385=2639us trial)\r\n");
        return 1U;
    }
    if((count==6U && strcmp(args[1],"camera")==0 && strcmp(args[2],"set")==0) ||
       (count==3U && strcmp(args[1],"camera")==0 && strcmp(args[2],"defaults")==0)) {
        x=ARM_CAMERA_LIFT_DEG;y=ARM_CAMERA_REACH_DEG;z=ARM_CAMERA_DM_DEG;
        if(count==6U && (!Cal_Float(args[3],&x) || !Cal_Float(args[4],&y) || !Cal_Float(args[5],&z))) {
            ZDT_Console_Print("[ARM CAMERA] usage: arm camera set <lift_deg> <reach_deg> <dm_deg>\r\n");return 1U;
        }
        result=Cal_CameraConfigure(x,y,z);
        (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] set HAL=%u lift=%+.2f reach=%+.2f DM=%+.2f deg; RAM only\r\n",
            (unsigned int)result,(double)s_camera_pose_deg[0],(double)s_camera_pose_deg[1],(double)s_camera_pose_deg[2]);
        ZDT_Console_Print(reply);return 1U;
    }
    if(count==4U && strcmp(args[1],"camera")==0 && strcmp(args[2],"mode")==0 &&
       (strcmp(args[3],"position")==0 || strcmp(args[3],"encoder")==0)) {
        result=ArmCalibration_SetCameraPositionMode((uint8_t)(strcmp(args[3],"position")==0));
        (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] mode=%s HAL=%u\r\n",s_camera_position_mode?"POSITION":"ENCODER",(unsigned int)result);
        ZDT_Console_Print(reply);return 1U;
    }
    if(count==3U && !strcmp(args[1],"camera") && !strcmp(args[2],"park")) {
        result=ArmCalibration_DeployParkingCamera();
        snprintf(reply,sizeof(reply),"[ARM PARK] lift/DM=CAMERA reach6=0 HAL=%u; normal CAMERA targets unchanged\r\n",(unsigned)result);ZDT_Console_Print(reply);return 1U;
    }
    if((count==2U || (count==3U && (strcmp(args[2],"status")==0 || strcmp(args[2],"init")==0))) && strcmp(args[1],"camera")==0) {
        if(count==3U)result=HAL_OK;
        if(count==3U && strcmp(args[2],"init")==0)result=ArmCalibration_DeployCameraInit();
        if(count==2U) {
            result=ArmCalibration_DeployCamera();
            if(result!=HAL_OK)ZDT_Console_Print("[ARM CAMERA] refused: busy or zero/span/top reference unavailable\r\n");
        }
        (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] state=%u active=%u targets lift=%+.2f reach=%+.2f DM=%+.2f deg HAL=%u mode=%s\r\n",
            (unsigned int)s_camera_state,(unsigned int)Cal_CameraActive(),(double)s_camera_pose_deg[0],
            (double)s_camera_pose_deg[1],(double)s_camera_pose_deg[2],(unsigned int)result,s_camera_position_mode?"POSITION":"ENCODER");
        ZDT_Console_Print(reply);
        if(s_camera_failure.valid) {
            (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] last_failure phase=%u reason=%.96s elapsed=%lums\r\n",
                s_camera_failure.phase,s_camera_failure.reason,(unsigned long)(HAL_GetTick()-s_camera_failure.tick));ZDT_Console_Print(reply);
            (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] at_fault DM=%+.4fdeg vel=%.5f age=%lums ZDT5 age=%lu/%lums ZDT6 age=%lu/%lums\r\n",
                (double)s_camera_failure.dm_deg,(double)s_camera_failure.dm_vel,(unsigned long)s_camera_failure.dm_age,
                (unsigned long)s_camera_failure.pos_age[0],(unsigned long)s_camera_failure.flags_age[0],
                (unsigned long)s_camera_failure.pos_age[1],(unsigned long)s_camera_failure.flags_age[1]);ZDT_Console_Print(reply);
            (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] at_fault reach from=%.3f%% actual=%.3f%% valid=%u target=%.3f%% raw=%lld flags=0x%02X ACK=%u/0x%02X motion=%u\r\n",
                (double)s_camera_failure.reach_start_percent,(double)s_camera_failure.reach_actual_percent,s_camera_failure.reach_actual_valid,
                (double)s_camera_failure.reach_target_percent,(long long)s_camera_failure.reach_raw,s_camera_failure.reach_flags,
                s_camera_failure.ack_seen,s_camera_failure.ack,s_camera_failure.motion_seen);ZDT_Console_Print(reply);
        }
        {
            if(s_camera_failure.valid){char io[240];
                (void)snprintf(io,sizeof(io),"[ARM CAMERA] at_fault IO gap=%lums prev_work=%lums work=%lums rxq=%u drops=%lu wait_max=%lums wire5=%lu/%lums wire6=%lu/%lums\r\n",
                  (unsigned long)s_camera_failure.loop_gap,(unsigned long)s_camera_failure.previous_work,(unsigned long)s_camera_failure.current_work,
                  s_camera_failure.rx_pending,(unsigned long)s_camera_failure.rx_drops,(unsigned long)s_camera_failure.rx_wait_max,
                  (unsigned long)s_camera_failure.wire_age[0][0],(unsigned long)s_camera_failure.wire_age[0][1],
                  (unsigned long)s_camera_failure.wire_age[1][0],(unsigned long)s_camera_failure.wire_age[1][1]);ZDT_Console_Print(io);}
            dm_feedback_t fb;float angle=0;uint8_t valid;
            DM_GetFeedback(ArmTask_GetMotor(),&fb);valid=(uint8_t)(ArmDriver_GetRotationDeg(&angle)==HAL_OK);
            (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] DM enabled_req=%u feedback_enabled=%u angle=%.4fdeg valid=%u vel=%.5frad/s age=%lums\r\n",
                (unsigned int)ArmTask_IsEnabled(),(unsigned int)fb.enabled,(double)angle,(unsigned int)valid,
                (double)fb.vel_rads,(unsigned long)(HAL_GetTick()-fb.last_tick));ZDT_Console_Print(reply);
            {
                float zero=0;
                if(ArmDriver_GetRotationZeroRaw(&zero)==HAL_OK) {
                    (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] DM raw_ref=%.6frad target=%.6frad offset=%.6frad\r\n",
                        (double)zero,(double)ArmTask_GetMotor()->target_pos_rad,(double)ArmTask_GetMotor()->pos_offset_rad);ZDT_Console_Print(reply);
                }
            }
        }
        return 1U;
    }
    if ((count == 2U) && (strcmp(args[1], "status") == 0))
    {
        Cal_Report(0U, "status"); Cal_Report(1U, "status");
        (void)snprintf(reply,sizeof(reply),"[ARM] zero5=%u delta5=%lldcounts zero6=%u delta6=%lldcounts\r\n",
            s_axes[0].zero_valid,(long long)(s_axes[0].position_counts-s_axes[0].zero_counts),
            s_axes[1].zero_valid,(long long)(s_axes[1].position_counts-s_axes[1].zero_counts));ZDT_Console_Print(reply);
        ArmServo_PrintStatus();
        (void)snprintf(reply, sizeof(reply), "[ARM] calibration_state=%u\r\n", (unsigned int)s_state);
        ZDT_Console_Print(reply);
        (void)snprintf(reply,sizeof(reply),"[ARM CAMERA] state=%u active=%u\r\n",
            (unsigned int)s_camera_state,(unsigned int)Cal_CameraActive());ZDT_Console_Print(reply);
        if(s_top_mode) {
            float down=0.0f;
            uint8_t measured=(uint8_t)(s_axes[0].upper_valid &&
                HAL_GetTick()-s_axes[0].position_tick<=s_cfg.feedback_timeout_ms);
            if(s_down_counts_per_mm!=0.0)down=(float)((double)(s_axes[0].position_counts-s_axes[0].upper_counts)/s_down_counts_per_mm);
            else if(s_axes[0].position_counts!=s_axes[0].upper_counts)measured=0;
            if(!isfinite(down) || (ARM_LIFT_RANGE_STOP_ENABLED && down < -ARM_TOP_REFERENCE_TOLERANCE_MM))measured=0;
            (void)snprintf(reply,sizeof(reply),"[ARM] down_from_top=%.6gmm measured=%u (RAM reference)\r\n",(double)down,measured);
            ZDT_Console_Print(reply);
            (void)snprintf(reply,sizeof(reply),"[ARM] TOP5 valid=%u drop_counts=%lld down_counts/mm=%.6g guard=%.6gmm [%.6g,%.6g]deg\r\n",
                s_axes[0].upper_valid,(long long)(s_axes[0].position_counts-s_axes[0].upper_counts),
                s_down_counts_per_mm,(double)s_top_drop_limit,(double)s_top_min,(double)s_top_max);
            ZDT_Console_Print(reply);
        }
        return 1U;
    }
    if ((count == 2U) && (strcmp(args[1], "stop") == 0))
    {
        HAL_StatusTypeDef cancel = ArmCalibration_Cancel();
        result = ArmDriver_StopAll();
        if (cancel != HAL_OK) { result = cancel; }
    }
    else if((count==3U || count==4U) &&
            (strcmp(args[1],"pct")==0 || strcmp(args[1],"percent")==0) && Cal_ParseAxis(args[2],&axis)) {
        if(count==3U) {
            result=ArmCalibration_GetPercent(axis,&x);
            if(result==HAL_OK) {
                (void)snprintf(reply,sizeof(reply),"[ARM] id=%u position=%.2f%%\r\n",
                    (unsigned int)((axis==ARM_DRIVER_LIFT)?5U:6U),(double)x);ZDT_Console_Print(reply);
            } else ZDT_Console_Print("[ARM] percent UNKNOWN: record 0% and check fresh position feedback\r\n");
        } else if(Cal_Float(args[3],&x)) {
            result=ArmCalibration_MovePercent(axis,x);
            if(result==HAL_ERROR) {
                int index=Cal_Index(axis);
                ZDT_Console_Print("[ARM] percent refused: need 0..100, recorded zero, fresh feedback and axis enable\r\n");
                if(index>=0) {
                    (void)snprintf(reply,sizeof(reply),"[ARM] id=%u pct check enable_req=%u zero=%u pos_rx=%lu pos_age=%lums; arm status shows query counters\r\n",
                        (unsigned int)Cal_Address((uint8_t)index),ArmDriver_IsEnableRequested(axis),s_axes[index].zero_valid,
                        (unsigned long)s_axes[index].position_sequence,(unsigned long)(HAL_GetTick()-s_axes[index].position_tick));
                    ZDT_Console_Print(reply);
                }
            }
        }
    }
    else if(count==3U && strcmp(args[1],"disable")==0 && Cal_ParseAxis(args[2],&axis)) {
        if(axis==ARM_DRIVER_ROTATION){(void)Cal_DmOperation(1U);return 1U;}
        if(Cal_IsActive() && axis==Cal_Axis())(void)ArmCalibration_Cancel();
        result=ArmDriver_Disable(axis);
    }
    else if (Cal_IsActive() != 0U) { result = HAL_BUSY; }
    else if ((count >= 3U) && ((strcmp(args[1], "grip") == 0) || (strcmp(args[1], "table") == 0)))
    {
        result = ArmServo_Command(count, args);
    }
    else if ((count == 2U) && (strcmp(args[1], "zero") == 0)) { (void)Cal_DmOperation(2U);return 1U; }
    else if(count==3U && strcmp(args[1],"zero")==0 && strcmp(args[2],"dm")==0) { (void)Cal_DmOperation(3U);return 1U; }
    else if(count==4U && strcmp(args[1],"zero")==0 && strcmp(args[2],"dm")==0 &&
        (strcmp(args[3],"save")==0 || strcmp(args[3],"confirm")==0)){(void)Cal_DmOperation(3U);return 1U;}
    else if(count==3U && strcmp(args[1],"zero")==0 && strcmp(args[2],"5")==0) { result=ArmCalibration_MarkEndpoint(ARM_DRIVER_LIFT,1U); }
    else if(count==3U && strcmp(args[1],"zero")==0 && strcmp(args[2],"6")==0) {
        result=Cal_Start(ARM_DRIVER_REACH,0U,0U);if(result==HAL_OK)s_zero_record=1U;
    }
    else if(count==3U && strcmp(args[1],"top")==0 && strcmp(args[2],"5")==0) {
        result=ArmCalibration_MarkEndpoint(ARM_DRIVER_LIFT,1U);
    }
    else if(count==3U && strcmp(args[1],"dropcal")==0 && Cal_Float(args[2],&x) && x>0 && s_axes[0].upper_valid) {
        result=Cal_Start(ARM_DRIVER_LIFT,0U,0U);
        if(result==HAL_OK) { s_drop_record=1;s_drop_cal_mm=x; }
    }
    else if(count==2U && strcmp(args[1],"guardoff")==0) { ArmDriver_ClearHeightRotationLimit();result=HAL_OK; }
    else if(count==5U && strcmp(args[1],"guardtop")==0 && Cal_Float(args[2],&x) &&
        Cal_Float(args[3],&y) && Cal_Float(args[4],&z)) { result=ArmCalibration_UseTopGuard(x,y,z); }
    else if ((count == 3U) &&
             ((strcmp(args[1], "rotate") == 0) || (strcmp(args[1], "by") == 0)) && Cal_Float(args[2], &x))
    {
        result = (strcmp(args[1], "rotate") == 0) ? ArmDriver_RotateToDeg(x) : ArmDriver_RotateByDeg(x);
    }
    else if ((count == 3U) && Cal_ParseAxis(args[2], &axis))
    {
        if (strcmp(args[1], "enable") == 0) {
            if(axis==ARM_DRIVER_ROTATION){(void)Cal_DmOperation(0U);return 1U;}
            result = ArmDriver_Enable(axis);
        }
        else if (strcmp(args[1], "disable") == 0) { result = ArmDriver_Disable(axis); }
        else if (strcmp(args[1], "clear") == 0)
        {
            result = (axis == ARM_DRIVER_ROTATION) ? ArmTask_ClearFault() : ArmCalibration_ClearProtection(axis);
        }
    }
    else if ((count == 4U) && (strcmp(args[1], "limit") == 0) &&
             Cal_Float(args[2], &x) && Cal_Float(args[3], &y)) { result = ArmDriver_SetRotationLimits(x, y); }
    else if ((count == 5U) && (strcmp(args[1], "guard") == 0) &&
             Cal_Float(args[2], &x) && Cal_Float(args[3], &y) && Cal_Float(args[4], &z))
    {
        result = ArmDriver_SetHeightRotationLimit(x, y, z);
        if(result==HAL_OK)s_top_mode=0;
    }
    else if ((count == 4U) && Cal_ParseAxis(args[2], &axis))
    {
        if (((strcmp(args[1], "mark") == 0) || (strcmp(args[1], "seek") == 0)) &&
            ((strcmp(args[3], "min") == 0) || (strcmp(args[3], "max") == 0)))
        {
            uint8_t upper = (strcmp(args[3], "max") == 0) ? 1U : 0U;
            result = (strcmp(args[1], "seek") == 0) ? ArmCalibration_SeekEndpoint(axis, upper) :
                                                     ArmCalibration_MarkEndpoint(axis, upper);
        }
        else if ((strcmp(args[1], "pulse") == 0) && Cal_Integer(args[3], &pulses))
        {
            if (axis == ARM_DRIVER_LIFT) { result = ArmDriver_LiftByPulses(pulses); }
            else if (axis == ARM_DRIVER_REACH) { result = ArmDriver_ReachByPulses(pulses); }
        }
        else if (Cal_Float(args[3], &x))
        {
            if(strcmp(args[1],"pos")==0 && axis==ARM_DRIVER_REACH) {
                float span,current;int32_t target;
                if(Cal_CameraAxisFresh(1U,HAL_GetTick()) &&
                   ArmCalibration_GetPercent(axis,&current)==HAL_OK &&
                   !(s_axes[1].flags&ARM_CAL_FLAG_PROTECTED) &&
                   ArmCalibration_GetSpan(axis,&span)==HAL_OK &&
                   ArmCalibration_GetAbsoluteReachPulses(x/360.0f*65536.0f/span*100.0f,&target)==HAL_OK) {
                    result=ArmDriver_ReachAbsolutePulses(target,ArmDriver_GetLinearSpeed(axis),ARM_CAMERA_POSITION_ACCEL);
                    (void)snprintf(reply,sizeof(reply),"[ARM POS] id=6 target=%+.4fdeg from=%+.3fdeg ABS pulses=%ld HAL=%u; %s; query checks actual\r\n",
                        (double)x,(double)current*(double)span/100.0*360.0/65536.0,(long)target,(unsigned int)result,
                        ARM_28_STALL_IGNORE_TRIAL?"STALL_IGNORE trial":"stall attempts max3");ZDT_Console_Print(reply);
                }
            }
            else if (strcmp(args[1], "travel") == 0) { result = ArmCalibration_SetTravelMm(axis, x); }
            else if(strcmp(args[1],"dir")==0 && (x==0 || x==1)) {
                result=ArmDriver_SetLinearDirection(axis,(uint8_t)x);
                if(result==HAL_OK) { if(axis==ARM_DRIVER_LIFT)s_cfg.lift_reverse=(uint8_t)x;else s_cfg.reach_reverse=(uint8_t)x; }
            }
            else if(strcmp(args[1],"speed")==0 && x>=1 && x<=ZDT_CAN_RUN_MAX_RPM && floorf(x)==x) { result=ArmDriver_SetLinearSpeed(axis,(uint16_t)x); }
            else if(strcmp(args[1],"feedback")==0 && axis==ARM_DRIVER_LIFT && x!=0.0f) { s_down_counts_per_mm=x;result=HAL_OK; }
            else if(strcmp(args[1],"span")==0 && Cal_Index(axis)>=0 && fabsf(x)>=1.0f && fabsf(x)<=4294967295.0f) {
                int index=Cal_Index(axis);s_percent_span[index]=(double)x;result=HAL_OK;
                if(index==0 && s_axes[0].travel_mm>0.0f)s_down_counts_per_mm=(double)x/s_axes[0].travel_mm;
            }
            else if (strcmp(args[1], "scale") == 0) { result = ArmDriver_SetLinearScale(axis, x); }
            else if (strcmp(args[1], "mm") == 0)
            {
                if (axis == ARM_DRIVER_LIFT) { result = ArmDriver_LiftByMm(x); }
                else if (axis == ARM_DRIVER_REACH) { result = ArmDriver_ReachByMm(x); }
            }
        }
    }
    (void)snprintf(reply, sizeof(reply), "[ARM] HAL=%u (0=queued,1=error,2=busy,3=timeout)\r\n", (unsigned int)result);
    ZDT_Console_Print(reply);
    return 1U;
}
