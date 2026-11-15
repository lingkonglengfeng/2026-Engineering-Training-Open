/**
 * Three-axis arm facade. Reuses arm_task and zdt_motor; never starts a bus.
 * Call from ONE task which also owns DM_Motor_Service and all arm commands.
 * Task context only. No heap, timer, ISR callback, or automatic motion.
 */
#ifndef ARM_DRIVER_H
#define ARM_DRIVER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_DRIVER_LIFT_ID   (5U) /* ZDT 42: positive = up */
#define ARM_DRIVER_REACH_ID  (6U) /* ZDT 28: positive = camera forward */
#define ARM_REACH_DEFAULT_REVERSE (1U) /* Replacement ID6 direction trial; raw feedback stays signed. */
#define ARM_MOTION_SPEED_SCALE (1.1f) /* V223: normal arm motion, applied once after load selection. */
#define ARM_REACH_SPEED_SCALE (1.2f) /* V226: additional gain for all automatic reach profiles. */
#define ARM_REACH_TRIAL_SPEED_SCALE (1.5f) /* V233: additional reach-only gain after existing rounded profile. */
#define ARM_EMPTY_ROTATION_SPEED_SCALE (1.44f)
#define ARM_LOADED_SPEED_SCALE (1.2f) /* V219: all three loaded/unknown motion profiles. */
#define ARM_CARRY_SPEED_SCALE (1.3f) /* V227: multiply the previous loaded profile once. */
#define ARM_PLATE_GRAB_REACH_SPEED_SCALE (1.44f) /* V223: previous1.2 x another1.2, grab extension only. */

typedef enum
{
    ARM_DRIVER_ROTATION = 0,
    ARM_DRIVER_LIFT,
    ARM_DRIVER_REACH
} arm_driver_axis_t;

typedef struct
{
    float pulses_per_mm;       /* 0: uncalibrated, pulse interface only */
    uint32_t max_move_pulses;   /* Per-command cap, NOT a travel limit; 0: INT32_MAX */
    uint16_t speed_rpm;        /* Configured base; motion profile selects loaded/OPEN RPM. */
    uint8_t acceleration;      /* Existing Emm 0xFD acceleration field */
    uint8_t reverse;           /* 0: positive -> CW; 1: positive -> CCW */
} arm_driver_linear_config_t;

typedef struct
{
    arm_driver_linear_config_t lift;
    arm_driver_linear_config_t reach;
    float rotation_zero_rad;   /* Arm 0 degrees in RAW DM output-shaft coordinates */
    float rotation_ratio;      /* DM output-shaft turns / arm turn; default 1 */
    float rotation_min_deg;    /* Default 0 */
    float rotation_max_deg;    /* Default 360; never folds 360 back to 0 */
    float rotation_speed_dps;  /* Configured base; load selection and common motion gain apply. */
    uint8_t rotation_reverse;
} arm_driver_config_t;

/* Configured bases remain lift238/reach79rpm and rotation135deg/s.
 * V227 profiles: loaded410/164rpm and231.66deg/s;
 * OPEN lift431/reach121rpm and213.84deg/s. Grab reach gets its own gain.
 * Acceleration: lift234 unchanged, reach209 (native reach168, plate grab195).
 * Configure directions, gear ratio and a per-command pulse cap before using.
 * Init validates/copies config only; sends no command and initializes no bus.
 * Init is one-shot (repeat returns HAL_BUSY). Existing ArmTask_Init and
 * ZDT_Motor_Begin must still be run by the original startup flow.
 */
void ArmDriver_DefaultConfig(arm_driver_config_t *out);
/* Owner-task, read-only UI diagnostics; does not enable or move an axis. */
void ArmDriver_PrintUi(void);
HAL_StatusTypeDef ArmDriver_CheckUiPose(float drop_mm,float rotation_deg);
HAL_StatusTypeDef ArmDriver_Init(const arm_driver_config_t *config);

/* Explicit enable. DM first refreshes stale/disabled feedback, then performs
 * the bounded safe-hold handshake (up to about900ms plus logging).
 * Stepper HAL_OK is only CAN enqueue success.
 * Stepper enable state below is a local request state, not motor feedback.
 */
HAL_StatusTypeDef ArmDriver_Enable(arm_driver_axis_t axis);
/* A disabled DM may sway during concurrent chassis motion. Enable captures
 * and holds current position; this is NOT the stricter teach/arrival limit. */
#define ARM_ROTATION_ENABLE_MAX_VEL_RAD_S (0.5f)
#ifndef ARM_DM_SOFTWARE_ZERO_ENABLED
#define ARM_DM_SOFTWARE_ZERO_ENABLED 0U /* Use DM's own persistent zero; ignore old W25 calibration. */
#endif
HAL_StatusTypeDef ArmDriver_Disable(arm_driver_axis_t axis);
uint8_t ArmDriver_IsEnableRequested(arm_driver_axis_t axis);

/* Rotation absolute/relative angles use degrees. Relative uses fresh actual
 * DM feedback, not the previous target. Out-of-range commands are rejected.
 */
HAL_StatusTypeDef ArmDriver_RotateToDeg(float angle_deg);
/* Selected default/limit in arm deg/s;0 before init. Based on grip PWM state. */
float ArmDriver_RotationSpeedDps(void);
/* Automatic reach profile includes load selection and V223 x1.1 once. */
uint16_t ArmDriver_ReachMotionSpeedRpm(void);
/* Explicit speed up to the current loaded/OPEN limit; same motion guards. */
HAL_StatusTypeDef ArmDriver_RotateToDegAtSpeed(float angle_deg,float speed_dps);
HAL_StatusTypeDef ArmDriver_RotateByDeg(float delta_deg);
HAL_StatusTypeDef ArmDriver_GetRotationDeg(float *angle_deg);
HAL_StatusTypeDef ArmDriver_GetRotationZeroRaw(float *zero_rad);

/* Runtime calibration in RAM only, task-owner context. ZeroHere uses fresh
 * RAW feedback; it does not write DM flash. Limit changes must fit DM range.
 * A height guard requires fresh calibrated lift height in millimeters.
 * Unknown/stale height denies rotation and CheckHeightInterlock disables DM.
 */
HAL_StatusTypeDef ArmDriver_SetRotationZeroHere(void);
/* Owner-task console helpers automatically disable/query. persistent=1 writes
 * one FE command, verifies near-zero raw feedback and leaves DM disabled. */
HAL_StatusTypeDef ArmDriver_ZeroRotation(uint8_t persistent);
HAL_StatusTypeDef ArmDriver_DisableRotationConfirmed(void);
uint8_t ArmDriver_IsRotationReferenceValid(void);
HAL_StatusTypeDef ArmDriver_MapRawRotation(float raw_rad,float *angle_deg);
/* Explicit HOME validates the zero target/height, safely enables if needed,
 * then requests the current arm0deg. HAL_OK means queued, not yet arrived. */
HAL_StatusTypeDef ArmDriver_ReturnRotationZero(void);
HAL_StatusTypeDef ArmDriver_SetRotationLimits(float min_deg, float max_deg);
HAL_StatusTypeDef ArmDriver_SetLinearScale(arm_driver_axis_t axis, float pulses_per_mm);
HAL_StatusTypeDef ArmDriver_SetLinearDirection(arm_driver_axis_t axis,uint8_t reverse);
HAL_StatusTypeDef ArmDriver_GetLinearDirection(arm_driver_axis_t axis,uint8_t *reverse);
HAL_StatusTypeDef ArmDriver_SetLinearSpeed(arm_driver_axis_t axis,uint16_t rpm);
uint16_t ArmDriver_GetConfiguredLinearSpeed(arm_driver_axis_t axis);
/* Effective loaded RPM is configured base x1.2. OPEN or a latched empty lift
 * returns its base so the established empty profile can be applied once. */
uint16_t ArmDriver_GetLinearSpeed(arm_driver_axis_t axis);
uint8_t ArmDriver_GetLinearAcceleration(arm_driver_axis_t axis);
/* Owner-only velocity command for the feedback-driven percentage controller.
 * Lift target height uses the active guard coordinate (top is 0, down negative).
 */
HAL_StatusTypeDef ArmDriver_RunLinear(arm_driver_axis_t axis, int16_t rpm,
                                     float lift_target_height_mm);
HAL_StatusTypeDef ArmDriver_SetHeightRotationLimit(float below_mm, float min_deg, float max_deg);
HAL_StatusTypeDef ArmDriver_SetTopRotationLimit(float drop_mm, float min_deg, float max_deg);
HAL_StatusTypeDef ArmDriver_CheckRawRotationTarget(float raw_rad);
void ArmDriver_ClearHeightRotationLimit(void);
void ArmDriver_UpdateLiftHeight(float height_mm, uint32_t feedback_tick, uint8_t valid);
HAL_StatusTypeDef ArmDriver_CheckHeightInterlock(void);
HAL_StatusTypeDef ArmDriver_CheckLiftSeek(int8_t direction);

/* Signed RELATIVE movement: +up/+forward; -down/-backward.
 * HAL_OK means queued, never "arrived". No automatic enable, homing, retry,
 * accumulated position estimate, or broadcast synchronization.
 * The application must wait for actual arrival and enforce physical travel
 * limits before submitting the next relative command.
 */
HAL_StatusTypeDef ArmDriver_LiftByPulses(int32_t pulses);
/* Native ID5 ABS in RAW driver pulse coordinates (not a relative +up move).
 * Does not apply the relative-axis reverse flag to the absolute coordinate.
 * Uses the configured FD acceleration curve; retains current enable/target.
 * Target height is the calibrated top0/down-negative coordinate. */
HAL_StatusTypeDef ArmDriver_LiftAbsoluteRawPulses(int32_t raw_pulses,uint16_t speed_rpm,
                                                float target_height_mm);
HAL_StatusTypeDef ArmDriver_ReachByPulses(int32_t pulses);
HAL_StatusTypeDef ArmDriver_ReachPositionPulses(int32_t pulses,uint16_t speed_rpm,uint8_t acceleration);
HAL_StatusTypeDef ArmDriver_ReachAbsolutePulses(int32_t pulses,uint16_t speed_rpm,uint8_t acceleration);
/* Precise held-position correction to the already cached SAME ABS goal.
 * Keeps the existing total3 retry budget; cannot replace a recovery job. */
HAL_StatusTypeDef ArmDriver_ReassertReachAbsolutePulses(int32_t pulses,uint16_t speed_rpm);
/* Replay the same cached native goal/profile without restarting deadlines or recovery generation. */
HAL_StatusTypeDef ArmDriver_ReplayReachAbsolutePulses(int32_t pulses);
/* Native absolute-position stall recovery: initial command +2 retries.
 * Owner Poll after RX drain. STOP/disable cancels; other faults fail immediately. */
void ArmDriver_PollReachRetry(void);
uint8_t ArmDriver_ReachRetryRecovering(void);
uint8_t ArmDriver_ReachRetryActive(void);
/* Internal position-owner STOP: keep the original target monitored while
 * verifying hold/turning DM. Public STOP/disable still cancel recovery. */
HAL_StatusTypeDef ArmDriver_StopReachForHold(void);
/* A non-protected stall flag is being checked using NEW flags/position data.
 * It has not yet stopped the axis or consumed a retry. */
uint8_t ArmDriver_ReachStallPending(void);
uint8_t ArmDriver_ReachRetryFailed(void);
uint32_t ArmDriver_ReachRetryGeneration(void);
uint16_t ArmDriver_ReachRetrySpeedRpm(void);
uint32_t ArmDriver_ReachRetryTimeoutMs(double remaining_counts);
/* CAMERA owner: stationary near the start after a command failed to start.
 * Reuse cached SAME ABS goal and total3-attempt budget; no reverse retreat. */
HAL_StatusTypeDef ArmDriver_RetryUnstartedReach(void);
#define ARM_REACH_NATIVE_START_WAIT_MS (1500U)
void ArmDriver_CancelReachRetry(void);
HAL_StatusTypeDef ArmDriver_LiftByMm(float distance_mm);
HAL_StatusTypeDef ArmDriver_ReachByMm(float distance_mm);

/* Stop keeps steppers energized (important on the vertical axis); DM stop
 * uses Disable, releases torque and clears its periodic target.
 * StopAll attempts all three motors and holds both initialized position
 * servos at their current PWM outputs even if one motor stop fails.
 * Never disables the lift or releases a held gripper as an ordinary stop.
 * A failed motion enqueue cancels that axis best-effort and inhibits further
 * movement until explicit Enable. A failed DM enqueue also latches its fault.
 */
HAL_StatusTypeDef ArmDriver_Stop(arm_driver_axis_t axis);
HAL_StatusTypeDef ArmDriver_StopAll(void);

#ifdef __cplusplus
}
#endif
#endif
