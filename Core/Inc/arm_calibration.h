#ifndef ARM_CALIBRATION_H
#define ARM_CALIBRATION_H

#include "arm_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* V144 arrival trial: minimum new samples; no200ms repeated arrival dwell.
 * Calibration/reference capture and fault debounce keep their own budgets. */
#define ARM_MOTION_SETTLE_MS (50U)
#define ARM_MOTION_SETTLE_SAMPLES (2U)
#define ARM_LINEAR_POST_STOP_SAMPLES (1U)

/* V153: lift status may age2000ms; positions/reach status retain800ms. */
#define ARM_ZDT_FEEDBACK_TIMEOUT_MS (800U)
#define ARM_ZDT_STATUS_TIMEOUT_MS (800U)
#define ARM_ZDT_LIFT_STATUS_TIMEOUT_MS (2000U)
#ifndef ARM_ZDT_WAIT_ON_STALE
#define ARM_ZDT_WAIT_ON_STALE 1 /* V179: wait/requery instead of stale-feedback FAULT. */
#endif
typedef struct {uint32_t tick;uint8_t active;} arm_feedback_wait_t;
uint8_t ArmCalibration_ZdtFeedbackFresh(void);
uint8_t ArmCalibration_FeedbackWaiting(void); /* Published to chassis task. */
uint8_t ArmCalibration_WaitForFeedback(arm_feedback_wait_t *wait,uint32_t now,uint32_t *paused_ms);
static inline uint32_t ArmCalibration_StatusTimeoutMs(arm_driver_axis_t axis)
{
    return axis==ARM_DRIVER_LIFT?ARM_ZDT_LIFT_STATUS_TIMEOUT_MS:ARM_ZDT_STATUS_TIMEOUT_MS;
}

#define ARM_CAL_FLAG_ENABLED   (0x01U)
#define ARM_CAL_FLAG_REACHED   (0x02U)
#define ARM_CAL_FLAG_STALL     (0x04U)
#define ARM_CAL_FLAG_PROTECTED (0x08U)
/* V166 experiment: only ID6/28mm reach ignores the STALL bit. Raw flags,
 * protection, enabled state, timestamps and actual positioning stay real.
 * Tests can compile -DARM_28_STALL_IGNORE_TRIAL=0 for the original policy. */
#ifndef ARM_28_STALL_IGNORE_TRIAL
#define ARM_28_STALL_IGNORE_TRIAL 1
#endif
static inline uint8_t ArmCalibration_StallFault(arm_driver_axis_t axis,uint8_t flags)
{
    return (uint8_t)((flags&ARM_CAL_FLAG_STALL) &&
        !(ARM_28_STALL_IGNORE_TRIAL && axis==ARM_DRIVER_REACH));
}
static inline uint8_t ArmCalibration_FaultFlags(arm_driver_axis_t axis,uint8_t flags)
{
    return (uint8_t)((flags&ARM_CAL_FLAG_PROTECTED) |
        (ArmCalibration_StallFault(axis,flags)?ARM_CAL_FLAG_STALL:0U));
}


/* Measured encoder displacement from 0% to 100%; not command pulses. */
#define ARM_LIFT_FULL_TRAVEL_COUNTS  (-219505.0)
#define ARM_REACH_FULL_TRAVEL_COUNTS (-132619.377777778)
#define ARM_LIFT_FULL_TRAVEL_MM      (240.0f)
#define ARM_REACH_FULL_TRAVEL_MM     (178.0f)
#define ARM_PERCENT_TOLERANCE        (0.5f)
#define ARM_TOP_ARRIVAL_TOLERANCE_MM (8.0f) /* V165: widen near-TOP arrival from6mm to8mm. */
/* Only near-TOP lift targets get an8mm arrival band. Other lift heights and
 * every reach target retain the existing0.5% tolerance. */
static inline float ArmCalibration_LiftTolerancePercent(float target,float travel_mm)
{
    float tolerance=ARM_PERCENT_TOLERANCE;
    if(target>=-ARM_PERCENT_TOLERANCE && target<=ARM_PERCENT_TOLERANCE && travel_mm>0.0f) {
        float top=ARM_TOP_ARRIVAL_TOLERANCE_MM*100.0f/travel_mm;
        if(top>tolerance)tolerance=top;
    }
    return tolerance;
}
#define ARM_TOP_REFERENCE_TOLERANCE_MM (20.0f) /* V165: requested TOP reference range. */
#define ARM_TRANSFER_DROP_MM (40.0f) /* Automatic rotation plane below the true TOP reference. */
static inline float ArmCalibration_TransferPercent(float travel_mm)
{return ARM_TRANSFER_DROP_MM*100.0f/travel_mm;}
#define ARM_REACH_REFERENCE_TOLERANCE_MM (3.0f)
#define ARM_CAMERA_LIFT_DEG          (-0.6647f)
#define ARM_CAMERA_REACH_DEG         (-300.0f) /* Extend to -300deg relative to retract zero. */
#define ARM_CAMERA_DM_DEG            (-0.0162f)
#define ARM_LIFT_BASE_RPM            (238U) /* Configured base; loaded/carry gains apply at runtime. */
#define ARM_CAMERA_LIFT_RPM_MAX      (410U) /* V227: round(315 x1.3), loaded both directions. */
#define ARM_CAMERA_LIFT_SPEED_GAIN   (11.88f) /* Before the common V223 motion gain. */
#define ARM_EMPTY_LIFT_SPEED_SCALE   (1.656f) /* Empty profile relative to configured base; unchanged. */
#define ARM_EMPTY_LIFT_RPM_MAX       (431U) /* V223: round(392 x1.1), both directions. */
/* Measured travel excursions do not invalidate feedback. Command bounds,
 * target arrival and the DM height/angle interlock remain enforced. */
#ifndef ARM_LIFT_RANGE_STOP_ENABLED
#define ARM_LIFT_RANGE_STOP_ENABLED  (0)
#endif
/* Read-only automatic empty-motion profile; does not loosen manual limits. */
uint8_t ArmCalibration_EmptyLiftMotionActive(void);
#define ARM_REACH_LOADED_RPM         (79U) /* Configured base; runtime load and V223 motion gains apply. */
#define ARM_LOADED_REACH_RPM_MAX     (246U) /* V233:164 x1.5. */
#define ARM_CAMERA_REACH_RPM_MAX     (182U) /* V233: round(121 x1.5). */
#define ARM_CAMERA_LIFT_ACCEL_RPM_S  (96U)
#define ARM_CAMERA_REACH_ACCEL_RPM_S (426U) /* FD209 slope:1RPM per2350us; residual speed caps remain. */
#define ARM_CAMERA_POSITION_STEPS_REV (3200U) /* Replacement Emm default1.8deg/16microsteps; old6400 correction removed. */
#define ARM_CAMERA_POSITION_ACCEL     (209U) /* Shared reach ramp for normal positioning and plate grab. */
/* ID5 has its own command-pulse calibration; ID6's former6400 correction
 * must not silently change the lift. Default1.8deg/16microsteps, not yet a
 * field measurement of ID5. Encoder feedback remains65536 counts/rev. */
#define ARM_LIFT_POSITION_STEPS_REV   (3200U)
#define ARM_LIFT_POSITION_ACCEL       (234U) /* 26/22=1.182x previous230 slope; closest integer to1.2x. */
#define ARM_LIFT_NATIVE_TOLERANCE_MM  (0.5f)
#define ARM_LIFT_NATIVE_STABLE_MM     (0.1f)
#define ARM_LIFT_NATIVE_SAMPLES       (3U)
typedef enum {
    ARM_CAMERA_IDLE=0, ARM_CAMERA_PRECHECK, ARM_CAMERA_DM_HOME,
    ARM_CAMERA_LIFT_ENABLE, ARM_CAMERA_LIFT_ZERO,
    ARM_CAMERA_REACH_ENABLE, ARM_CAMERA_REACH_MOVE, ARM_CAMERA_LIFT_FINAL,
    ARM_CAMERA_DONE, ARM_CAMERA_FAILED, ARM_CAMERA_REFERENCE
} arm_camera_state_t;
/* Owner-task sequence, explicit start only. Returns to existing zero references;
 * never captures the current pose as a new zero or writes driver flash. */
HAL_StatusTypeDef ArmCalibration_DeployCamera(void);
HAL_StatusTypeDef ArmCalibration_DeployParkingCamera(void);
/* Explicit assertion that the robot is at mechanical top/retracted. Records
 * stationary RAM references, then deploys; never writes driver flash. */
HAL_StatusTypeDef ArmCalibration_DeployCameraInit(void);
arm_camera_state_t ArmCalibration_GetCameraState(void);
/* Retained until the next accepted CAMERA command; owner-task status/debug. */
const char *ArmCalibration_GetCameraFailureReason(void);
void ArmCalibration_ClearCameraReady(void);
HAL_StatusTypeDef ArmCalibration_SetCameraPositionMode(uint8_t enabled);
/* User-defined driver coordinates: encoder0deg/0counts is lift TOP and
 * reach fully RETRACTED. References are fixed; current position still needs RX. */
#define ARM_LIFT_TOP_ZERO_COUNTS INT64_C(0)
#define ARM_REACH_RETRACT_ZERO_COUNTS INT64_C(0)

typedef enum
{
    ARM_CAL_IDLE = 0,
    ARM_CAL_PRECHECK,
    ARM_CAL_SEEKING,
    ARM_CAL_SETTLING,
    ARM_CAL_FAILED,
    ARM_CAL_PERCENT_MOVE,
    ARM_CAL_PERCENT_SETTLE
} arm_cal_state_t;

typedef struct
{
    int64_t position_counts; /* Emm encoder: 65536 counts/rev, NOT command pulses */
    int64_t lower_counts;
    int64_t upper_counts;
    int64_t zero_counts;
    uint32_t position_tick;
    uint32_t flags_tick;
    uint32_t position_sequence;
    uint32_t flags_sequence;
    float travel_mm;         /* Physical upper-minus-lower span; 0 = unknown */
    uint8_t flags;
    uint8_t lower_valid;
    uint8_t upper_valid;
    uint8_t zero_valid;
} arm_cal_axis_status_t;

typedef struct
{
    uint16_t seek_speed_rpm; /* Emm RPM, default 20; max 100 for calibration */
    uint8_t lift_reverse;
    uint8_t reach_reverse;  /* Must match ArmDriver direction config */
    uint8_t acceleration;
    uint32_t query_period_ms; /* default 100 */
    uint32_t feedback_timeout_ms; /* Position budget800; lift status2000/reach status800ms. */
    uint32_t seek_timeout_ms; /* default 15000, whole transaction */
    uint32_t stall_grace_ms; /* default 300, ignore startup stall transient */
} arm_cal_config_t;

void ArmCalibration_DefaultConfig(arm_cal_config_t *out);
HAL_StatusTypeDef ArmCalibration_UseTopGuard(float drop_mm,float min_deg,float max_deg);
HAL_StatusTypeDef ArmCalibration_Init(const arm_cal_config_t *config);
/* Poll at <=10 ms from the SAME owner task as arm commands/DM service.
 * ISR uses PostCanFrame; only the owner task calls OnCanFrame to parse frames.
 * No HAL callback is defined/overridden; FIFO0 remains owned by zdt_motor.
 */
void ArmCalibration_Poll(void);
void ArmCalibration_ServiceIo(void); /* Owner only; no commands/action FSM advancement. */
void ArmCalibration_ServiceWait(void); /* Maintain existing percent motion/servo targets during DM waits. */
uint8_t ArmCalibration_OnCanFrame(const FDCAN_RxHeaderTypeDef *header,
                                  const uint8_t *data, uint8_t length);
/* Integration ports: existing UART/CAN task can post here without touching
 * owner state. PostCanFrame is also ISR-safe (bounded copy, no print).
 * Poll drains queues in the arm owner task. HAL_BUSY means queue full.
 * Only ID5/6 state/position/native FD replies are accepted. PostCommand task-only.
 */
HAL_StatusTypeDef ArmCalibration_PostCanFrame(const FDCAN_RxHeaderTypeDef *header,
                                              const uint8_t *data, uint8_t length);
HAL_StatusTypeDef ArmCalibration_PostCommand(const char *line);
void ArmCalibration_DiscardCommands(void);

/* Mark manually placed stationary endpoint, or seek until fresh stall flag.
 * upper=1 seeks up/forward, upper=0 down/backward. Caller enables axis first.
 * One transaction at a time. Stops leave stepper enabled; no auto-clear of
 * protection and no driver-flash writes. Stall is a candidate endpoint: an
 * obstacle can cause the same flag; operator must verify calibration.
 */
HAL_StatusTypeDef ArmCalibration_MarkEndpoint(arm_driver_axis_t axis, uint8_t upper);
HAL_StatusTypeDef ArmCalibration_SeekEndpoint(arm_driver_axis_t axis, uint8_t upper);
HAL_StatusTypeDef ArmCalibration_Cancel(void);
HAL_StatusTypeDef ArmCalibration_SetTravelMm(arm_driver_axis_t axis, float travel_mm);
HAL_StatusTypeDef ArmCalibration_ClearProtection(arm_driver_axis_t axis);
void ArmCalibration_GetAxisStatus(arm_driver_axis_t axis, arm_cal_axis_status_t *out);
typedef struct {
    uint32_t sequence,received_tick;
    uint8_t response;
} arm_cal_position_ack_t;
/* Owner-task native FD reply snapshot. Fence includes already queued ISR
 * replies so a previous command's pending ACK cannot confirm a new job. */
uint32_t ArmCalibration_PositionAckFence(arm_driver_axis_t axis);
void ArmCalibration_GetPositionAck(arm_driver_axis_t axis,arm_cal_position_ack_t *out);
typedef struct {
    uint32_t queued_position,queued_flags,busy,error,no_reply,retries;
    uint8_t pending,last_hal;
} arm_cal_query_status_t;
/* Same owner task as Poll; counters mean HAL enqueue, not bus delivery. */
void ArmCalibration_GetQueryStatus(arm_driver_axis_t axis,arm_cal_query_status_t *out);
arm_cal_state_t ArmCalibration_GetState(void);
/* Owner task: an existing lift percent action targets TOP-40mm, including settle. */
uint8_t ArmCalibration_IsLiftRaisingToClearance(void);
uint8_t ArmCalibration_IsLiftRaisingToTop(void); /* True0% reference, retained for calibration callers. */
/* Absolute travel percentage, fresh feedback + recorded 0% required.
 * Lift 0%=top (arm top 5); reach 0%=retracted (arm zero 6).
 * Poll owns a nonblocking velocity/position-feedback transaction.
 */
HAL_StatusTypeDef ArmCalibration_GetPercent(arm_driver_axis_t axis, float *percent);
HAL_StatusTypeDef ArmCalibration_GetSpan(arm_driver_axis_t axis,float *span);
/* Absolute FD input coordinate, including the software zero and signed span. */
HAL_StatusTypeDef ArmCalibration_GetAbsoluteReachPulses(float percent,int32_t *pulses);
/* Raw driver absolute position, including the recorded nonzero TOP reference. */
HAL_StatusTypeDef ArmCalibration_GetAbsoluteLiftPulses(float percent,int32_t *pulses);
HAL_StatusTypeDef ArmCalibration_MovePercent(arm_driver_axis_t axis, float percent);
/* Owner-only: stop/freeze lift percent motion without cancelling the pose
 * or the independent reach retry. Caller may resume with MovePercent. */
HAL_StatusTypeDef ArmCalibration_HoldLiftForReachRecovery(void);

/* Feed complete ASCII line from existing UART4 command parser (owner task).
 * Returns 1 for any "arm" command, including malformed ones, else 0.
 * Replies through ZDT_Console_Print; does not read UART hardware or steal RX.
 */
uint8_t ArmCalibration_ProcessCommand(const char *line);

#ifdef __cplusplus
}
#endif
#endif
