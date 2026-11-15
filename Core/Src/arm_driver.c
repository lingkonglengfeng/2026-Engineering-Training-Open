#include "arm_driver.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "arm_task.h"
#include "arm_servo.h"
#include "arm_calibration.h"
#include "arm_start_pose.h"
#include "arm_material.h"
#include "fdcan.h"
#include "zdt_motor.h"
#include <stdlib.h>

#define ARM_DEG_TO_RAD (0.017453292519943295f)
#define ARM_FEEDBACK_AGE_MS (DM_LINK_TIMEOUT_MS_DEFAULT)

static arm_driver_config_t s_config;
static uint8_t s_initialized;
static uint8_t s_rotation_reference_valid;
static uint8_t s_stepper_enable_mask;
enum {RR_IDLE,RR_MONITOR,RR_STOP,RR_CLEAR,RR_WAIT_CLEAR,RR_ENABLE,RR_WAIT_ENABLE,
      RR_RETREAT_SEND,RR_RETREAT_MOVE,RR_RETREAT_HOLD,RR_SEND,RR_FAILED};
static uint8_t s_rr_state,s_rr_attempts,s_rr_acc,s_rr_reverse;
static uint16_t s_rr_speed;
static int32_t s_rr_target;
static uint32_t s_rr_tick,s_rr_flags,s_rr_generation;
static arm_feedback_wait_t s_feedback_pause;
#define REACH_STALL_CONFIRM_MS (300U)
#define REACH_STALL_FRESH_SAMPLES (3U)
/* One default Emm microstep is20.48 encoder counts. Count even one such
 * step as progress so a legitimate low-speed move is not called stuck. */
#define REACH_STALL_PROGRESS_COUNTS (16.0)
static uint8_t s_rr_stall_pending,s_rr_stall_samples;
static uint32_t s_rr_stall_tick,s_rr_progress_tick,s_rr_pos_seq,s_rr_stall_pos_seq;
static double s_rr_goal_counts,s_rr_best_error;
static int64_t s_rr_start_counts;
static uint8_t s_rr_unstarted_replay;
#define REACH_RETREAT_MM (2.0)
#define REACH_RETREAT_RPM (6U)
static int8_t s_rr_move_sign;
static uint8_t s_rr_retreat_done,s_rr_retreat_samples;
static int32_t s_rr_retreat_pulses;
static double s_rr_retreat_start,s_rr_retreat_goal,s_rr_counts_mm,s_rr_retreat_last;
static uint32_t s_rr_retreat_seq,s_rr_retreat_flags,s_rr_retreat_timeout;
static uint8_t s_height_guard;
static uint8_t s_top_guard;
static uint8_t s_height_valid;
static float s_height_mm;
static uint32_t s_height_tick;
static float s_low_height_mm;
static float s_low_min_deg;
static float s_low_max_deg;

static uint8_t ArmDriver_ContextValid(void)
{
    return ((s_initialized != 0U) && (__get_IPSR() == 0U)) ? 1U : 0U;
}

static uint8_t ArmDriver_LinearValid(const arm_driver_linear_config_t *cfg)
{
    return (isfinite(cfg->pulses_per_mm) && (cfg->pulses_per_mm >= 0.0f) &&
            (cfg->max_move_pulses <= (uint32_t)INT32_MAX) &&
            (cfg->speed_rpm > 0U) &&
            (cfg->speed_rpm <= (uint16_t)ZDT_CAN_RUN_MAX_RPM) &&
            (cfg->reverse <= 1U)) ? 1U : 0U;
}

static float ArmDriver_RawAngle(float angle_deg)
{
    float sign = (s_config.rotation_reverse != 0U) ? -1.0f : 1.0f;
    return s_config.rotation_zero_rad +
           sign * angle_deg * ARM_DEG_TO_RAD * s_config.rotation_ratio;
}

static uint8_t ArmDriver_RotationRangeValid(float zero, float min_deg, float max_deg)
{
    float sign = (s_config.rotation_reverse != 0U) ? -1.0f : 1.0f;
    float scale = sign * ARM_DEG_TO_RAD * s_config.rotation_ratio;
    float a = zero + scale * min_deg;
    float b = zero + scale * max_deg;
    dm_motor_t *motor = ArmTask_GetMotor();
    return (isfinite(min_deg) && isfinite(max_deg) && (min_deg < max_deg) &&
            isfinite(a) && isfinite(b) && (fabsf(a) <= DM_P_MAX) && (fabsf(b) <= DM_P_MAX) &&
            ((ArmTask_IsReady() == 0U) ||
             ((fabsf(a) <= motor->cfg.pos_limit_rad) && (fabsf(b) <= motor->cfg.pos_limit_rad)))) ? 1U : 0U;
}

static uint8_t ArmDriver_HeightAllows(float angle)
{
    if (s_height_guard == 0U) { return 1U; }
    if ((s_height_valid == 0U) || ((HAL_GetTick() - s_height_tick) > ARM_ZDT_FEEDBACK_TIMEOUT_MS)) {
        return (s_top_guard && angle>=s_low_min_deg && angle<=s_low_max_deg)?1U:0U;
    }
    return ((s_height_mm >= s_low_height_mm) ||
            ((angle >= s_low_min_deg) && (angle <= s_low_max_deg))) ? 1U : 0U;
}
void ArmDriver_PrintUi(void)
{
    float angle=0.0f;char line[192];uint8_t valid=(uint8_t)(ArmDriver_GetRotationDeg(&angle)==HAL_OK);
    (void)snprintf(line,sizeof(line),"[ARM UI ROT] valid=%u angle=%.4f min=%.4f max=%.4f enabled=%u\r\n",
        valid,(double)angle,(double)s_config.rotation_min_deg,(double)s_config.rotation_max_deg,ArmTask_IsEnabled());ZDT_Console_Print(line);
    (void)snprintf(line,sizeof(line),"[ARM UI GUARD] enabled=%u top=%u threshold=%.4f min=%.4f max=%.4f height=%.4f valid=%u\r\n",
        s_height_guard,s_top_guard,(double)s_low_height_mm,(double)s_low_min_deg,(double)s_low_max_deg,
        (double)s_height_mm,(unsigned int)(s_height_valid && HAL_GetTick()-s_height_tick<=ARM_ZDT_FEEDBACK_TIMEOUT_MS));ZDT_Console_Print(line);
}
HAL_StatusTypeDef ArmDriver_CheckUiPose(float drop_mm,float rotation_deg)
{
    arm_cal_axis_status_t lift;float height;
    if(!s_initialized || !isfinite(drop_mm) || !isfinite(rotation_deg) ||
       rotation_deg<s_config.rotation_min_deg || rotation_deg>s_config.rotation_max_deg)return HAL_ERROR;
    if(!s_height_guard)return HAL_OK;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    height=s_top_guard?-drop_mm:lift.travel_mm-drop_mm;
    return (height>=s_low_height_mm || (rotation_deg>=s_low_min_deg && rotation_deg<=s_low_max_deg))?HAL_OK:HAL_ERROR;
}
static uint8_t ArmDriver_CurrentHeightAllows(float angle)
{
    /* Only measured-position checks get tolerance; targets remain +/-20.
     * DM feedback quantization/settling must not cause a boundary chatter. */
    if(s_top_guard && angle>=s_low_min_deg-0.5f && angle<=s_low_max_deg+0.5f)return 1U;
    return ArmDriver_HeightAllows(angle);
}

static const arm_driver_linear_config_t *ArmDriver_LinearConfig(arm_driver_axis_t axis)
{
    if (axis == ARM_DRIVER_LIFT) { return &s_config.lift; }
    if (axis == ARM_DRIVER_REACH) { return &s_config.reach; }
    return NULL;
}

static uint8_t ArmDriver_Address(arm_driver_axis_t axis)
{
    return (axis == ARM_DRIVER_LIFT) ? ARM_DRIVER_LIFT_ID : ARM_DRIVER_REACH_ID;
}

static uint8_t ArmDriver_Mask(arm_driver_axis_t axis)
{
    return (axis == ARM_DRIVER_LIFT) ? 1U : 2U;
}

static HAL_StatusTypeDef ArmDriver_StepperReady(uint32_t required_slots)
{
    const zdt_motor_status_t *status = ZDT_Motor_GetStatus();
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    if ((status == NULL) || (status->can_ready == 0U) ||
        (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol) != HAL_OK) ||
        (protocol.BusOff != 0U))
    {
        return HAL_ERROR;
    }
    return (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) >= required_slots) ? HAL_OK : HAL_BUSY;
}

void ArmDriver_DefaultConfig(arm_driver_config_t *out)
{
    if (out == NULL) { return; }
    (void)memset(out, 0, sizeof(*out));
    out->lift.speed_rpm = ARM_LIFT_BASE_RPM;
    out->lift.acceleration = ARM_LIFT_POSITION_ACCEL;
    out->reach.speed_rpm = ARM_REACH_LOADED_RPM;
    out->reach.reverse = ARM_REACH_DEFAULT_REVERSE;
    out->reach.acceleration = ARM_CAMERA_POSITION_ACCEL;
    out->rotation_ratio = 1.0f;
    out->rotation_max_deg = 360.0f;
    out->rotation_speed_dps = 135.0f;
}

HAL_StatusTypeDef ArmDriver_Init(const arm_driver_config_t *config)
{
    float end0;
    float end1;
    float sign;
    if ((config == NULL) || (__get_IPSR() != 0U)) { return HAL_ERROR; }
    if (s_initialized != 0U) { return HAL_BUSY; }
    if ((ArmDriver_LinearValid(&config->lift) == 0U) ||
        (ArmDriver_LinearValid(&config->reach) == 0U) ||
        !isfinite(config->rotation_zero_rad) ||
        !isfinite(config->rotation_ratio) || (config->rotation_ratio <= 0.0f) ||
        !isfinite(config->rotation_min_deg) || !isfinite(config->rotation_max_deg) ||
        (config->rotation_min_deg >= config->rotation_max_deg) ||
        !isfinite(config->rotation_speed_dps) || (config->rotation_speed_dps <= 0.0f) ||
        (config->rotation_reverse > 1U))
    {
        return HAL_ERROR;
    }
    sign = (config->rotation_reverse != 0U) ? -1.0f : 1.0f;
    end0 = config->rotation_zero_rad + sign * config->rotation_min_deg *
           ARM_DEG_TO_RAD * config->rotation_ratio;
    end1 = config->rotation_zero_rad + sign * config->rotation_max_deg *
           ARM_DEG_TO_RAD * config->rotation_ratio;
    if (!isfinite(end0) || !isfinite(end1) || (fabsf(end0) > DM_P_MAX) ||
        (fabsf(end1) > DM_P_MAX) ||
        !isfinite(config->rotation_speed_dps * ARM_DEG_TO_RAD * config->rotation_ratio) ||
        (config->rotation_speed_dps * ARM_DEG_TO_RAD * config->rotation_ratio <= 0.0f) ||
        (config->rotation_speed_dps * ARM_DEG_TO_RAD * config->rotation_ratio > DM_V_MAX))
    {
        return HAL_ERROR;
    }
    s_config = *config;
    s_rotation_reference_valid=1U;
    s_stepper_enable_mask = 0U;
    s_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_Enable(arm_driver_axis_t axis)
{
    HAL_StatusTypeDef result;
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    if (axis == ARM_DRIVER_ROTATION)
    {
        float current=NAN;
        dm_feedback_t fb;
        if ((ArmTask_IsReady() == 0U) ||
            (ArmTask_GetMotor()->active_mode != DM_POS_MODE) ||
            !s_rotation_reference_valid) { return HAL_ERROR; }
        /* An explicit new enable/home can recover a software timeout latch.
         * QueryDisabled still rejects any fresh hardware fault; the enable
         * handshake clears the old latch only after a fresh enabled reply. */
        DM_GetFeedback(ArmTask_GetMotor(),&fb);
        if(!ArmTask_IsEnabled() || !fb.fb_count || HAL_GetTick()-fb.last_tick>ARM_FEEDBACK_AGE_MS) {
            result=DM_Motor_QueryDisabled(ArmTask_GetMotor(),300U,&fb);
            if(result!=HAL_OK)return result;
        }
        if(!ArmTask_IsEnabled()) {
            if(!isfinite(fb.pos_rad) || !isfinite(fb.vel_rads))return HAL_ERROR;
            if(fabsf(fb.vel_rads)>ARM_ROTATION_ENABLE_MAX_VEL_RAD_S)return HAL_BUSY;
        }
        if ((s_height_guard != 0U) &&
            ((ArmDriver_GetRotationDeg(&current) != HAL_OK) ||
             (ArmDriver_CurrentHeightAllows(current) == 0U))) {
            char line[180];
            (void)snprintf(line,sizeof(line),
                "[DM GUARD] enable/home: angle=%+.2fdeg height_valid=%u age=%lums down_from_TOP=%.2fmm drop_limit=%.2fmm sector=[%.1f,%.1f]deg\r\n",
                (double)current,s_height_valid,(unsigned long)(HAL_GetTick()-s_height_tick),
                (double)(s_top_guard?-s_height_mm:s_height_mm),(double)(s_top_guard?-s_low_height_mm:s_low_height_mm),
                (double)s_low_min_deg,(double)s_low_max_deg);ZDT_Console_Print(line);
            return HAL_ERROR;
        }
        return ArmTask_IsEnabled()?HAL_OK:ArmTask_Enable();
    }
    if (ArmDriver_LinearConfig(axis) == NULL) { return HAL_ERROR; }
    result = ArmDriver_StepperReady(1U);
    if (result != HAL_OK) { return result; }
    result = ZDT_Motor_EnableAddr(ArmDriver_Address(axis), 0U);
    if (result == HAL_OK) { s_stepper_enable_mask |= ArmDriver_Mask(axis); }
    else { s_stepper_enable_mask &= (uint8_t)~ArmDriver_Mask(axis); }
    return result;
}

HAL_StatusTypeDef ArmDriver_Disable(arm_driver_axis_t axis)
{
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    if (axis == ARM_DRIVER_ROTATION) { return ArmTask_Disable(); }
    if (ArmDriver_LinearConfig(axis) == NULL) { return HAL_ERROR; }
    if(axis==ARM_DRIVER_REACH)s_rr_state=RR_IDLE;
    s_stepper_enable_mask &= (uint8_t)~ArmDriver_Mask(axis);
    return ZDT_Motor_DisableAddr(ArmDriver_Address(axis), 0U);
}

uint8_t ArmDriver_IsEnableRequested(arm_driver_axis_t axis)
{
    if (ArmDriver_ContextValid() == 0U) { return 0U; }
    if (axis == ARM_DRIVER_ROTATION) { return ArmTask_IsEnabled(); }
    if (ArmDriver_LinearConfig(axis) == NULL) { return 0U; }
    return ((s_stepper_enable_mask & ArmDriver_Mask(axis)) != 0U) ? 1U : 0U;
}

HAL_StatusTypeDef ArmDriver_MapRawRotation(float raw_rad,float *angle_deg)
{
    float sign=(s_config.rotation_reverse!=0U)?-1.0f:1.0f;
    if(!ArmDriver_ContextValid() || !s_rotation_reference_valid || !angle_deg || !isfinite(raw_rad))return HAL_ERROR;
    *angle_deg=sign*(raw_rad-s_config.rotation_zero_rad)/(ARM_DEG_TO_RAD*s_config.rotation_ratio);
    return isfinite(*angle_deg)?HAL_OK:HAL_ERROR;
}
HAL_StatusTypeDef ArmDriver_GetRotationDeg(float *angle_deg)
{
    dm_feedback_t feedback;
    dm_motor_t *motor;
    if ((ArmDriver_ContextValid() == 0U) || (angle_deg == NULL) ||
        (ArmTask_IsReady() == 0U) || !s_rotation_reference_valid) { return HAL_ERROR; }
    motor = ArmTask_GetMotor();
    DM_GetFeedback(motor, &feedback); /* Coherent ISR snapshot from existing driver */
    if ((feedback.fb_count == 0U) || !isfinite(feedback.pos_rad) ||
        ((HAL_GetTick() - feedback.last_tick) > ARM_FEEDBACK_AGE_MS)) { return HAL_ERROR; }
    return ArmDriver_MapRawRotation(feedback.pos_rad,angle_deg);
}
HAL_StatusTypeDef ArmDriver_GetRotationZeroRaw(float *zero_rad)
{
    if(!ArmDriver_ContextValid() || !s_rotation_reference_valid || zero_rad==NULL)return HAL_ERROR;
    *zero_rad=s_config.rotation_zero_rad;return HAL_OK;
}

float ArmDriver_RotationSpeedDps(void)
{
    if(!s_initialized)return 0.0f;
    return s_config.rotation_speed_dps*(ArmServo_GripIsOpen()?ARM_EMPTY_ROTATION_SPEED_SCALE:ARM_LOADED_SPEED_SCALE*ARM_CARRY_SPEED_SCALE)*ARM_MOTION_SPEED_SCALE;
}
uint16_t ArmDriver_ReachMotionSpeedRpm(void)
{
    uint16_t cap=ArmDriver_GetLinearSpeed(ARM_DRIVER_REACH);
    return ArmServo_GripIsOpen()?(uint16_t)fminf((float)cap*7.0f/6.0f,ARM_CAMERA_REACH_RPM_MAX):
        (uint16_t)fminf(cap,ARM_LOADED_REACH_RPM_MAX);
}
HAL_StatusTypeDef ArmDriver_RotateToDegAtSpeed(float angle_deg,float speed_dps)
{
    dm_motor_t *motor;
    float raw;
    float user_target;
    float speed;
    float current_deg;
    HAL_StatusTypeDef result;
    if ((ArmDriver_ContextValid() == 0U) || !isfinite(angle_deg) || !isfinite(speed_dps) ||
        speed_dps<=0.0f || speed_dps>ArmDriver_RotationSpeedDps() ||
        (angle_deg < s_config.rotation_min_deg) || (angle_deg > s_config.rotation_max_deg) ||
        (ArmTask_IsEnabled() == 0U) || (ArmTask_IsFaulted() != 0U) ||
        (ArmDriver_GetRotationDeg(&current_deg) != HAL_OK) ||
        (ArmDriver_HeightAllows(angle_deg) == 0U) ||
        (ArmDriver_CurrentHeightAllows(current_deg) == 0U)) { return HAL_ERROR; }
    motor = ArmTask_GetMotor();
    if ((motor->active_mode != DM_POS_MODE) || (motor->enable_pending != 0U) ||
        (motor->resend_hold != 0U)) { return HAL_BUSY; }
    raw = ArmDriver_RawAngle(angle_deg);
    /* ArmTask ultimately adds pos_offset_rad; remove it once here. Its own
     * range check applies to the supplied user coordinate as well. */
    user_target = raw - motor->pos_offset_rad;
    speed = speed_dps * ARM_DEG_TO_RAD * s_config.rotation_ratio;
    if (!isfinite(raw) || !isfinite(user_target) || !isfinite(speed) ||
        (fabsf(raw) > motor->cfg.pos_limit_rad) ||
        (fabsf(user_target) > ARM_TURNTABLE_POS_LIMIT_RAD) ||
        (speed <= 0.0f) || (speed > motor->cfg.vel_limit_rad)) { return HAL_ERROR; }
    /* Owner-task contract prevents another sender from overwriting last_err. */
    result = ArmTask_MoveToRad(user_target, speed);
    if (result == HAL_OK) { result = (HAL_StatusTypeDef)motor->last_err; }
    if (result != HAL_OK)
    {
        /* The old SetPos leaves a failed target armed for periodic resend.
         * Cancel it and latch a fault rather than executing it unexpectedly. */
        ArmTask_EmergencyStop();
    }
    return result;
}
HAL_StatusTypeDef ArmDriver_RotateToDeg(float angle_deg)
{return ArmDriver_RotateToDegAtSpeed(angle_deg,ArmDriver_RotationSpeedDps());}

HAL_StatusTypeDef ArmDriver_RotateByDeg(float delta_deg)
{
    float current;
    if (!isfinite(delta_deg) || (ArmDriver_GetRotationDeg(&current) != HAL_OK))
    {
        return HAL_ERROR;
    }
    return ArmDriver_RotateToDeg(current + delta_deg);
}
HAL_StatusTypeDef ArmDriver_ReturnRotationZero(void)
{
    HAL_StatusTypeDef result;
    if(!ArmDriver_ContextValid() || !s_rotation_reference_valid || s_config.rotation_min_deg>0.0f ||
        s_config.rotation_max_deg<0.0f || !ArmDriver_HeightAllows(0.0f))return HAL_ERROR;
    result=ArmDriver_Enable(ARM_DRIVER_ROTATION);if(result!=HAL_OK)return result;
    return ArmDriver_RotateToDeg(0.0f);
}

HAL_StatusTypeDef ArmDriver_SetRotationZeroHere(void)
{
    dm_feedback_t fb;
    if ((ArmDriver_ContextValid() == 0U) || (ArmTask_IsReady() == 0U) ||
        (ArmTask_IsEnabled() != 0U)) { return HAL_ERROR; }
    DM_GetFeedback(ArmTask_GetMotor(), &fb);
    if ((fb.fb_count == 0U) || ((HAL_GetTick() - fb.last_tick) > ARM_FEEDBACK_AGE_MS) ||
        (fb.enabled != 0U) || !isfinite(fb.pos_rad) || !isfinite(fb.vel_rads) ||
        (fabsf(fb.vel_rads) > 0.05f) ||
        (ArmDriver_RotationRangeValid(fb.pos_rad, s_config.rotation_min_deg,
                                      s_config.rotation_max_deg) == 0U)) { return HAL_ERROR; }
    s_config.rotation_zero_rad = fb.pos_rad;
    s_rotation_reference_valid=1U;
    return HAL_OK;
}
uint8_t ArmDriver_IsRotationReferenceValid(void){return s_rotation_reference_valid;}
HAL_StatusTypeDef ArmDriver_DisableRotationConfirmed(void)
{
    if(!ArmDriver_ContextValid() || !ArmTask_IsReady())return HAL_ERROR;
    return DM_Motor_QueryDisabled(ArmTask_GetMotor(),300U,NULL);
}
HAL_StatusTypeDef ArmDriver_ZeroRotation(uint8_t persistent)
{
    dm_motor_t *motor;dm_feedback_t fb;HAL_StatusTypeDef result;
    if(!ArmDriver_ContextValid() || !ArmTask_IsReady() || persistent>1U)return HAL_ERROR;
    motor=ArmTask_GetMotor();result=DM_Motor_QueryDisabled(motor,300U,&fb);
    if(result!=HAL_OK)return result;
    if(fabsf(fb.vel_rads)>0.05f)return HAL_BUSY;
    if(!persistent) {
        result=ArmDriver_SetRotationZeroHere();
        if(result==HAL_OK){motor->pos_offset_rad=fb.pos_rad;motor->target_pos_rad=fb.pos_rad;}
        return result;
    }
    if(!ArmDriver_RotationRangeValid(0.0f,s_config.rotation_min_deg,s_config.rotation_max_deg))return HAL_ERROR;
    {
        char line[150];(void)snprintf(line,sizeof(line),
            "[DM ZERO] INTERNAL FE once: ESC=0x%02X POS id=0x%03X before_raw=%+.4fdeg; stationary/OFF confirmed\r\n",
            motor->cfg.esc_id,motor->cfg.esc_id+DM_MODE_POS,(double)fb.pos_rad/ARM_DEG_TO_RAD);
        ZDT_Console_Print(line);
    }
    result=DM_Motor_SavePosZero(motor);if(result!=HAL_OK)return result;
    /* FE may have changed the encoder frame: refuse old-coordinate movement
     * if verification fails or an emergency stop cancels the query. */
    s_rotation_reference_valid=0U;motor->pos_offset_rad=0.0f;
    motor->target_pos_rad=0.0f;motor->target_vel_rads=0.0f;
    result=DM_Motor_VerifyPosZero(motor,1000U);if(result!=HAL_OK)return result;
    s_config.rotation_zero_rad=0.0f;s_rotation_reference_valid=1U;return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_SetRotationLimits(float min_deg, float max_deg)
{
    if ((ArmDriver_ContextValid() == 0U) || (ArmTask_IsEnabled() != 0U) ||
        (ArmDriver_RotationRangeValid(s_config.rotation_zero_rad, min_deg, max_deg) == 0U) ||
        ((s_height_guard != 0U) && ((s_low_min_deg < min_deg) || (s_low_max_deg > max_deg))))
    {
        return HAL_ERROR;
    }
    s_config.rotation_min_deg = min_deg;
    s_config.rotation_max_deg = max_deg;
    return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_SetLinearScale(arm_driver_axis_t axis, float pulses_per_mm)
{
    if ((ArmDriver_ContextValid() == 0U) || !isfinite(pulses_per_mm) || (pulses_per_mm <= 0.0f))
    {
        return HAL_ERROR;
    }
    if (axis == ARM_DRIVER_LIFT) { s_config.lift.pulses_per_mm = pulses_per_mm; }
    else if (axis == ARM_DRIVER_REACH) { s_config.reach.pulses_per_mm = pulses_per_mm; }
    else { return HAL_ERROR; }
    return HAL_OK;
}
HAL_StatusTypeDef ArmDriver_SetLinearDirection(arm_driver_axis_t axis,uint8_t reverse)
{
    if(!ArmDriver_ContextValid() || reverse>1U || ArmDriver_LinearConfig(axis)==NULL)return HAL_ERROR;
    if(axis==ARM_DRIVER_LIFT)s_config.lift.reverse=reverse;else s_config.reach.reverse=reverse;
    return HAL_OK;
}
HAL_StatusTypeDef ArmDriver_SetLinearSpeed(arm_driver_axis_t axis,uint16_t rpm)
{
    if(!ArmDriver_ContextValid() || !rpm || rpm>ZDT_CAN_RUN_MAX_RPM || ArmDriver_LinearConfig(axis)==NULL)return HAL_ERROR;
    if(axis==ARM_DRIVER_LIFT)s_config.lift.speed_rpm=rpm;else s_config.reach.speed_rpm=rpm;
    return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_GetLinearDirection(arm_driver_axis_t axis,uint8_t *reverse)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    if(!ArmDriver_ContextValid() || cfg==NULL || reverse==NULL)return HAL_ERROR;
    *reverse=cfg->reverse;
    return HAL_OK;
}

uint16_t ArmDriver_GetConfiguredLinearSpeed(arm_driver_axis_t axis)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    float speed;
    if(!ArmDriver_ContextValid() || cfg==NULL)return 0U;
    speed=roundf(cfg->speed_rpm*ARM_MOTION_SPEED_SCALE);
    if(axis==ARM_DRIVER_REACH)speed=roundf(roundf(speed*ARM_REACH_SPEED_SCALE)*ARM_REACH_TRIAL_SPEED_SCALE);
    return (uint16_t)fminf(speed,ZDT_CAN_RUN_MAX_RPM);
}
uint16_t ArmDriver_GetLinearSpeed(arm_driver_axis_t axis)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    if(!ArmDriver_ContextValid() || cfg==NULL)return 0U;
    /* Keep an accepted empty lift profile latched through an overlapping
     * close; the following loaded raise obtains its own loaded/carry profile. */
    if(ArmServo_GripIsOpen() || (axis==ARM_DRIVER_LIFT && ArmCalibration_EmptyLiftMotionActive()))return ArmDriver_GetConfiguredLinearSpeed(axis);
    {float speed=roundf(roundf(cfg->speed_rpm*ARM_LOADED_SPEED_SCALE)*ARM_MOTION_SPEED_SCALE);
     if(axis==ARM_DRIVER_REACH)speed=roundf(speed*ARM_REACH_SPEED_SCALE);
     speed=roundf(speed*ARM_CARRY_SPEED_SCALE);
     if(axis==ARM_DRIVER_REACH)speed=roundf(speed*ARM_REACH_TRIAL_SPEED_SCALE);
     return (uint16_t)fminf(speed,ZDT_CAN_RUN_MAX_RPM);}
}
uint8_t ArmDriver_GetLinearAcceleration(arm_driver_axis_t axis)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    return (ArmDriver_ContextValid() && cfg!=NULL)?cfg->acceleration:0U;
}

HAL_StatusTypeDef ArmDriver_RunLinear(arm_driver_axis_t axis,int16_t rpm,float target_height)
{
    if(axis==ARM_DRIVER_REACH)s_rr_state=RR_IDLE;
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    HAL_StatusTypeDef result;
    if(!ArmDriver_ContextValid() || cfg==NULL ||
       abs(rpm)>(axis==ARM_DRIVER_LIFT && ArmCalibration_EmptyLiftMotionActive()?
           (int)fminf(ArmDriver_GetConfiguredLinearSpeed(axis)*ARM_EMPTY_LIFT_SPEED_SCALE,ARM_EMPTY_LIFT_RPM_MAX):(axis==ARM_DRIVER_REACH?(int)ArmDriver_ReachMotionSpeedRpm():(int)ArmDriver_GetLinearSpeed(axis))) ||
       !ArmDriver_IsEnableRequested(axis))return HAL_ERROR;
    if(rpm==0)return ArmDriver_Stop(axis);
    if(axis==ARM_DRIVER_LIFT && s_height_guard) {
        float current,target,sign=(s_config.rotation_reverse!=0U)?-1.0f:1.0f;
        if(!isfinite(target_height) || !s_height_valid ||
           HAL_GetTick()-s_height_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS)return HAL_ERROR;
        if(s_top_guard && (target_height>0.0f || (rpm>0 && s_height_mm>=0.0f)))return HAL_ERROR;
        if(rpm<0 && target_height<s_low_height_mm) {
            target=sign*(ArmTask_GetMotor()->target_pos_rad-s_config.rotation_zero_rad)/
                (ARM_DEG_TO_RAD*s_config.rotation_ratio);
            if(ArmDriver_GetRotationDeg(&current)!=HAL_OK ||
               current<s_low_min_deg-0.5f || current>s_low_max_deg+0.5f ||
               (ArmTask_IsEnabled() && (!isfinite(target) || target<s_low_min_deg || target>s_low_max_deg)))return HAL_ERROR;
        }
    }
    result=ArmDriver_StepperReady(1U);
    if(result!=HAL_OK)return result;
    if(cfg->reverse)rpm=(int16_t)-rpm;
    result=ZDT_Motor_SetSpeedRpmAddr(ArmDriver_Address(axis),rpm,0U);
    if(result!=HAL_OK) {
        s_stepper_enable_mask&=(uint8_t)~ArmDriver_Mask(axis);
        (void)ZDT_Motor_StopAddr(ArmDriver_Address(axis));
    }
    return result;
}

HAL_StatusTypeDef ArmDriver_SetHeightRotationLimit(float below_mm, float min_deg, float max_deg)
{
    if ((ArmDriver_ContextValid() == 0U) || !isfinite(below_mm) || (below_mm < 0.0f) ||
        !isfinite(min_deg) || !isfinite(max_deg) || (min_deg >= max_deg) ||
        (min_deg < s_config.rotation_min_deg) || (max_deg > s_config.rotation_max_deg))
    {
        return HAL_ERROR;
    }
    s_low_height_mm = below_mm;
    s_low_min_deg = min_deg;
    s_low_max_deg = max_deg;
    s_height_guard = 1U;
    s_top_guard = 0U;
    (void)ArmDriver_CheckHeightInterlock();
    return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_SetTopRotationLimit(float drop_mm,float min_deg,float max_deg)
{
    if(!ArmDriver_ContextValid() || !isfinite(drop_mm) || drop_mm<=0 ||
       !isfinite(min_deg) || !isfinite(max_deg) || min_deg>=max_deg ||
       min_deg<s_config.rotation_min_deg || max_deg>s_config.rotation_max_deg)return HAL_ERROR;
    s_low_height_mm=-drop_mm;s_low_min_deg=min_deg;s_low_max_deg=max_deg;
    s_height_guard=1U;s_top_guard=1U;
    (void)ArmDriver_CheckHeightInterlock();return HAL_OK;
}
HAL_StatusTypeDef ArmDriver_CheckRawRotationTarget(float raw_rad)
{
    float sign=(s_config.rotation_reverse!=0U)?-1.0f:1.0f;
    float angle=sign*(raw_rad-s_config.rotation_zero_rad)/(ARM_DEG_TO_RAD*s_config.rotation_ratio);
    return (ArmDriver_ContextValid() && s_rotation_reference_valid && isfinite(angle) &&
        angle>=s_config.rotation_min_deg && angle<=s_config.rotation_max_deg &&
        ArmDriver_HeightAllows(angle))?HAL_OK:HAL_ERROR;
}

void ArmDriver_ClearHeightRotationLimit(void)
{
    if (ArmDriver_ContextValid() != 0U) { s_height_guard = 0U; }
}

void ArmDriver_UpdateLiftHeight(float height_mm, uint32_t feedback_tick, uint8_t valid)
{
    if (ArmDriver_ContextValid() == 0U) { return; }
    s_height_valid = ((valid != 0U) && isfinite(height_mm)) ? 1U : 0U;
    s_height_mm = height_mm;
    s_height_tick = feedback_tick;
}

HAL_StatusTypeDef ArmDriver_CheckHeightInterlock(void)
{
    float current;
    float target;
    float sign;
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    if ((s_height_guard == 0U) || (ArmTask_IsEnabled() == 0U)) { return HAL_OK; }
    sign = (s_config.rotation_reverse != 0U) ? -1.0f : 1.0f;
    target = sign * (ArmTask_GetMotor()->target_pos_rad - s_config.rotation_zero_rad) /
             (ARM_DEG_TO_RAD * s_config.rotation_ratio);
    if ((ArmDriver_GetRotationDeg(&current) != HAL_OK) || !isfinite(target) ||
        (ArmDriver_CurrentHeightAllows(current) == 0U) || (ArmDriver_HeightAllows(target) == 0U))
    {
        (void)ArmTask_Disable();
        /* Avoid continuing downward while the arm is already in a forbidden
         * sector. The lift retains enable/holding torque. */
        (void)ZDT_Motor_StopAddr(ARM_DRIVER_LIFT_ID);
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef ArmDriver_CheckLiftSeek(int8_t direction)
{
    float current;
    if ((ArmDriver_ContextValid() == 0U) || ((direction != 1) && (direction != -1))) { return HAL_ERROR; }
    if (s_height_guard == 0U) { return HAL_OK; }
    /* A seek has no known displacement. Keep DM disabled throughout seek;
     * a downward seek must start within the low-height angular sector. */
    if (ArmTask_IsEnabled() != 0U) { return HAL_ERROR; }
    if ((direction < 0) && ((ArmDriver_GetRotationDeg(&current) != HAL_OK) ||
        (current < s_low_min_deg) || (current > s_low_max_deg))) { return HAL_ERROR; }
    return HAL_OK;
}

static HAL_StatusTypeDef ArmDriver_MovePulsesProfileMode(arm_driver_axis_t axis, int32_t pulses,uint16_t speed,uint8_t acceleration,uint8_t absolute)
{
    if(axis==ARM_DRIVER_REACH && !absolute)s_rr_state=RR_IDLE;
    const arm_driver_linear_config_t *cfg;
    uint32_t magnitude;
    uint16_t motion_limit;
    HAL_StatusTypeDef result;
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    cfg = ArmDriver_LinearConfig(axis);
    motion_limit=axis==ARM_DRIVER_REACH?ArmDriver_ReachMotionSpeedRpm():ArmDriver_GetLinearSpeed(axis);
    if(axis==ARM_DRIVER_REACH && ArmMaterial_GrabReachActive())
        motion_limit=(uint16_t)fminf(roundf(motion_limit*ARM_PLATE_GRAB_REACH_SPEED_SCALE),ZDT_CAN_RUN_MAX_RPM);
    /* INT32_MIN would overflow both direction reversal and the old ZDT API. */
    if ((cfg == NULL) || (pulses == INT32_MIN) || speed==0U || speed>motion_limit ||
        (ArmDriver_IsEnableRequested(axis) == 0U)) { return HAL_ERROR; }
    magnitude = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;
    if ((cfg->max_move_pulses != 0U) && (magnitude > cfg->max_move_pulses)) { return HAL_ERROR; }
    if (pulses == 0 && !absolute) { return HAL_OK; } /* Absolute zero is a real target. */
    if(axis==ARM_DRIVER_LIFT && pulses>0 && s_top_guard && s_height_valid &&
        HAL_GetTick()-s_height_tick<=ARM_ZDT_FEEDBACK_TIMEOUT_MS) {
        if(s_height_mm>=0.0f || (cfg->pulses_per_mm>0.0f &&
            s_height_mm+(float)pulses/cfg->pulses_per_mm>0.0f))return HAL_ERROR;
    }
    if ((axis == ARM_DRIVER_LIFT) && (pulses < 0) && (s_height_guard != 0U))
    {
        float current;
        float sign = (s_config.rotation_reverse != 0U) ? -1.0f : 1.0f;
        float target = sign * (ArmTask_GetMotor()->target_pos_rad - s_config.rotation_zero_rad) /
                       (ARM_DEG_TO_RAD * s_config.rotation_ratio);
        uint8_t within;
        within=0U;
        if(s_top_guard) {
            if(ArmDriver_GetRotationDeg(&current)!=HAL_OK)return HAL_ERROR;
            within=(uint8_t)(current>=s_low_min_deg && current<=s_low_max_deg &&
                (!ArmTask_IsEnabled() || (target>=s_low_min_deg && target<=s_low_max_deg)));
        }
        if(!within && ((s_height_valid == 0U) || ((HAL_GetTick() - s_height_tick) > ARM_ZDT_FEEDBACK_TIMEOUT_MS) ||
            (cfg->pulses_per_mm <= 0.0f))) { return HAL_ERROR; }
        if (!within && (s_height_mm + (float)pulses / cfg->pulses_per_mm < s_low_height_mm) &&
            ((ArmDriver_GetRotationDeg(&current) != HAL_OK) ||
             (current < s_low_min_deg) || (current > s_low_max_deg) ||
             ((ArmTask_IsEnabled() != 0U) &&
              (!isfinite(target) || (target < s_low_min_deg) || (target > s_low_max_deg)))))
        {
            return HAL_ERROR;
        }
    }
    result = ArmDriver_StepperReady(2U);
    if (result != HAL_OK) { return result; }
    if (cfg->reverse != 0U) { pulses = -pulses; }
    result = absolute?ZDT_Motor_MovePositionPulsesAbsAddr(ArmDriver_Address(axis),pulses,speed,acceleration,0U):
        ZDT_Motor_MovePositionPulsesAddr(ArmDriver_Address(axis),pulses,speed,acceleration,0U);
    if (result != HAL_OK)
    {
        s_stepper_enable_mask &= (uint8_t)~ArmDriver_Mask(axis);
        (void)ZDT_Motor_StopAddr(ArmDriver_Address(axis)); /* Partial split-frame failure */
    }
    return result;
}
static HAL_StatusTypeDef ArmDriver_MovePulsesProfile(arm_driver_axis_t axis,int32_t pulses,uint16_t speed,uint8_t acceleration)
{return ArmDriver_MovePulsesProfileMode(axis,pulses,speed,acceleration,0U);}
HAL_StatusTypeDef ArmDriver_ReachAbsolutePulses(int32_t pulses,uint16_t speed,uint8_t acceleration)
{
    arm_cal_axis_status_t a={0};
    HAL_StatusTypeDef result=ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,pulses,speed,acceleration,1U);
    s_rr_state=RR_IDLE;
    if(result==HAL_OK){
        ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);s_rr_flags=a.flags_sequence;
        s_rr_start_counts=a.position_counts;s_rr_unstarted_replay=0U;
        float span=(float)ARM_REACH_FULL_TRAVEL_COUNTS;
        (void)ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span);
        s_rr_target=pulses;s_rr_speed=speed;s_rr_acc=acceleration;s_rr_reverse=s_config.reach.reverse;s_rr_attempts=1U;s_rr_tick=HAL_GetTick();s_rr_state=RR_MONITOR;
        s_rr_goal_counts=(double)pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV*(span<0.0f?-1.0:1.0);
        s_rr_move_sign=s_rr_goal_counts>(double)a.position_counts?1:s_rr_goal_counts<(double)a.position_counts?-1:0;
        s_rr_retreat_done=0U;
        s_rr_stall_pending=s_rr_stall_samples=0U;s_rr_pos_seq=a.position_sequence;
        s_rr_best_error=fabs((double)a.position_counts-s_rr_goal_counts);s_rr_progress_tick=s_rr_tick;
    }
    return result;
}
uint8_t ArmDriver_ReachRetryRecovering(void){return s_rr_state>=RR_STOP && s_rr_state<=RR_SEND;}
HAL_StatusTypeDef ArmDriver_ReplayReachAbsolutePulses(int32_t pulses)
{
    if(s_rr_state!=RR_MONITOR || s_rr_stall_pending)return HAL_BUSY;
    if(pulses!=s_rr_target)return HAL_ERROR;
    return ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,s_rr_target,s_rr_speed,s_rr_acc,1U);
}
HAL_StatusTypeDef ArmDriver_ReassertReachAbsolutePulses(int32_t pulses,uint16_t speed)
{
    arm_cal_axis_status_t a={0};HAL_StatusTypeDef result;
    if(!ArmDriver_ContextValid() || !speed || pulses!=s_rr_target)return HAL_ERROR;
    if(s_rr_state!=RR_MONITOR)return HAL_BUSY;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);
    if(!a.position_sequence || !a.flags_sequence || HAL_GetTick()-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS ||
       HAL_GetTick()-a.flags_tick>ARM_ZDT_STATUS_TIMEOUT_MS || ArmCalibration_FaultFlags(ARM_DRIVER_REACH,a.flags))return HAL_BUSY;
    if(speed>s_rr_speed)speed=s_rr_speed;
    result=ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,pulses,speed,s_rr_acc,1U);
    if(result!=HAL_OK && !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH))return HAL_ERROR;
    if(result==HAL_OK) {
        s_rr_speed=speed;s_rr_start_counts=a.position_counts;s_rr_tick=HAL_GetTick();++s_rr_generation;
        s_rr_move_sign=s_rr_goal_counts>(double)a.position_counts?1:s_rr_goal_counts<(double)a.position_counts?-1:0;
        s_rr_flags=a.flags_sequence;s_rr_pos_seq=a.position_sequence;s_rr_stall_pending=s_rr_stall_samples=0U;
        s_rr_best_error=fabs((double)a.position_counts-s_rr_goal_counts);s_rr_progress_tick=s_rr_tick;
    }
    return result;
}
uint8_t ArmDriver_ReachRetryActive(void){return s_rr_state!=RR_IDLE && s_rr_state!=RR_FAILED;}
HAL_StatusTypeDef ArmDriver_StopReachForHold(void)
{return ArmDriver_ContextValid()?ZDT_Motor_StopAddr(ARM_DRIVER_REACH_ID):HAL_ERROR;}
uint8_t ArmDriver_ReachStallPending(void){return s_rr_state==RR_MONITOR && s_rr_stall_pending;}
uint8_t ArmDriver_ReachRetryFailed(void){return s_rr_state==RR_FAILED;}
uint32_t ArmDriver_ReachRetryGeneration(void){return s_rr_generation;}
uint16_t ArmDriver_ReachRetrySpeedRpm(void)
{
    uint16_t cap=s_rr_attempts>=3U?18U:s_rr_attempts==2U?36U:s_rr_speed;
    uint16_t speed=s_rr_speed<cap?s_rr_speed:cap;
    return speed?speed:1U;
}
uint32_t ArmDriver_ReachRetryTimeoutMs(double remaining_counts)
{
    double timeout=fabs(remaining_counts)*120000.0/(65536.0*ArmDriver_ReachRetrySpeedRpm())+5000.0;
    if(!isfinite(timeout) || timeout>120000.0)timeout=120000.0;
    if(timeout<10000.0)timeout=10000.0;
    return (uint32_t)timeout;
}
void ArmDriver_CancelReachRetry(void){s_rr_state=RR_IDLE;s_rr_stall_pending=s_rr_stall_samples=0U;memset(&s_feedback_pause,0,sizeof(s_feedback_pause));}
static void ArmDriver_ReachRetryReport(void)
{
    static const char *const phases[]={"IDLE","MONITOR","STOP","CLEAR","WAIT_CLEAR","ENABLE","WAIT_ENABLE","RETREAT_SEND","RETREAT_MOVE","RETREAT_HOLD","SEND","FAILED"};
    arm_cal_axis_status_t a={0};char line[240];uint32_t now;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);now=HAL_GetTick();
    (void)snprintf(line,sizeof(line),"[REACH RETRY] phase=%s attempt=%u/3 ABS=%ld speed=%uRPM actual=%+.3fdeg raw=%lld zero=%lld flags=0x%02X age=%lu/%lums\r\n",
        phases[s_rr_state],s_rr_attempts,(long)s_rr_target,ArmDriver_ReachRetrySpeedRpm(),
        (double)(a.position_counts-a.zero_counts)*360.0/65536.0,(long long)a.position_counts,(long long)a.zero_counts,a.flags,
        (unsigned long)(now-a.position_tick),(unsigned long)(now-a.flags_tick));ZDT_Console_Print(line);
}
static void ArmDriver_ReachRetryFail(const char *reason)
{
    char line[150];uint8_t attempts=s_rr_attempts;
    ArmDriver_ReachRetryReport();
    (void)ArmDriver_Stop(ARM_DRIVER_REACH);s_rr_state=RR_FAILED;
    (void)snprintf(line,sizeof(line),"[REACH RETRY] FAILED attempts=%u/3: %s; reach STOP requested; owner stops flow\r\n",attempts,reason);
    ZDT_Console_Print(line);
}
static void ArmDriver_ReachRetryNext(uint32_t now)
{
    char line[140];
    if(s_rr_attempts>=3U){ArmDriver_ReachRetryFail(ARM_28_STALL_IGNORE_TRIAL?"native start retry limit; actual motion absent":"stall/clear/enable failed three attempts");return;}
    ArmDriver_ReachRetryReport();
    ++s_rr_attempts;s_rr_state=RR_STOP;s_rr_tick=now;s_rr_retreat_done=0U;s_rr_unstarted_replay=0U;
    (void)snprintf(line,sizeof(line),"[REACH RETRY] stall -> attempt=%u/3 STOP/clear/enable; SAME ABS target=%ld speed=%uRPM\r\n",
        s_rr_attempts,(long)s_rr_target,ArmDriver_ReachRetrySpeedRpm());ZDT_Console_Print(line);
}
HAL_StatusTypeDef ArmDriver_RetryUnstartedReach(void)
{
    arm_cal_axis_status_t a={0};float span;double initial,threshold;uint32_t now;char line[160];
    if(!ArmDriver_ContextValid() || s_rr_state!=RR_MONITOR || !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH))return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);now=HAL_GetTick();
    if(now-s_rr_tick<ARM_REACH_NATIVE_START_WAIT_MS)return HAL_BUSY;
    if(!a.position_sequence || !a.flags_sequence || now-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || now-a.flags_tick>ARM_ZDT_STATUS_TIMEOUT_MS ||
       !(a.flags&ARM_CAL_FLAG_ENABLED) || ArmCalibration_FaultFlags(ARM_DRIVER_REACH,a.flags) ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span)!=HAL_OK)return HAL_ERROR;
    initial=fabs(s_rr_goal_counts-(double)s_rr_start_counts);
    threshold=fmin(fabs((double)span)*ARM_PERCENT_TOLERANCE/100.0,initial*0.25);
    if(initial<=fabs((double)span)*ARM_PERCENT_TOLERANCE/100.0 ||
       fabs((double)a.position_counts-(double)s_rr_start_counts)>=threshold)return HAL_BUSY;
    if(s_rr_attempts>=3U){ArmDriver_ReachRetryFail("native position never started after three attempts");return HAL_ERROR;}
    ++s_rr_attempts;s_rr_state=RR_STOP;s_rr_tick=now;s_rr_retreat_done=1U;s_rr_unstarted_replay=1U;
    (void)snprintf(line,sizeof(line),"[REACH START RETRY] attempt=%u/3 SAME ABS=%ld actual=%+.3fdeg; STOP/enable/replay, no reverse retreat\r\n",
        s_rr_attempts,(long)s_rr_target,(double)(a.position_counts-a.zero_counts)*360.0/65536.0);ZDT_Console_Print(line);
    return HAL_OK;
}
static HAL_StatusTypeDef ArmDriver_ReachPrepareRetreat(const arm_cal_axis_status_t *a)
{
    float span,pct;double low,high,target;int32_t pulses;
    if(!a->zero_valid || !s_rr_move_sign || !isfinite(a->travel_mm) || a->travel_mm<=0.0f ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span)!=HAL_OK)return HAL_ERROR;
    s_rr_counts_mm=fabs((double)span)/a->travel_mm;
    low=fmin((double)a->zero_counts,(double)a->zero_counts+span);
    high=fmax((double)a->zero_counts,(double)a->zero_counts+span);
    target=(double)a->position_counts-s_rr_move_sign*s_rr_counts_mm*REACH_RETREAT_MM;
    if(target<low)target=low;
    if(target>high)target=high;
    pct=(float)((target-(double)a->zero_counts)/(double)span*100.0);
    if(ArmCalibration_GetAbsoluteReachPulses(pct,&pulses)!=HAL_OK)return HAL_ERROR;
    s_rr_retreat_goal=(double)pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV*(span<0?-1.0:1.0);
    s_rr_retreat_start=(double)a->position_counts;
    if((s_rr_retreat_goal-s_rr_retreat_start)*s_rr_move_sign>=-16.0)return HAL_BUSY;
    s_rr_retreat_pulses=pulses;return HAL_OK;
}
#if ARM_28_STALL_IGNORE_TRIAL
static uint8_t s_28_trial_paced,s_28_trial_pending;
static uint32_t s_28_trial_tick,s_28_trial_clears;
static uint8_t ArmDriver_Trial28StallService(const arm_cal_axis_status_t *a,uint32_t now)
{
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    arm_cal_state_t calibration=ArmCalibration_GetState();
    uint8_t camera_active=(uint8_t)(camera!=ARM_CAMERA_IDLE && camera!=ARM_CAMERA_DONE && camera!=ARM_CAMERA_FAILED);
    uint8_t active=(uint8_t)(s_rr_state!=RR_IDLE && s_rr_state!=RR_FAILED);
    HAL_StatusTypeDef result;arm_cal_axis_status_t lift={0};
    active|=(uint8_t)(camera_active || ArmStartPose_IsActive() ||
        (calibration!=ARM_CAL_IDLE && calibration!=ARM_CAL_FAILED));
    /* MAT's WAIT/DESCEND/RAISE already has paced clear/enable and replays its
     *25mm cached target. Do not duplicate that owner or restart after STOP. */
    if(!active || (ArmMaterial_IsActive() && !ArmStartPose_IsActive() && !camera_active && s_rr_state==RR_IDLE)) {
        s_28_trial_pending=s_28_trial_paced=0U;return 1U;
    }
    if(!a->flags_sequence || !a->position_sequence || now-a->flags_tick>ARM_ZDT_STATUS_TIMEOUT_MS ||
       now-a->position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || (a->flags&ARM_CAL_FLAG_PROTECTED))return 1U;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(ArmCalibration_FaultFlags(ARM_DRIVER_LIFT,lift.flags) || ArmTask_IsFaulted())return 1U;
    if(!(a->flags&ARM_CAL_FLAG_STALL) && !s_28_trial_pending){s_28_trial_paced=0U;return 1U;}
    if(s_28_trial_paced && now-s_28_trial_tick<150U)return 1U;
    s_28_trial_paced=1U;s_28_trial_tick=now;
    if(!s_28_trial_pending) {
        result=ArmCalibration_ClearProtection(ARM_DRIVER_REACH);
        if(result==HAL_BUSY)return 1U;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("28 trial stall-clear CAN failed");return 0U;}
        s_28_trial_pending=1U;s_28_trial_clears++;
    }
    if(ArmDriver_IsEnableRequested(ARM_DRIVER_REACH)) {
        result=ArmDriver_Enable(ARM_DRIVER_REACH);
        if(result==HAL_BUSY)return 1U;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("28 trial re-enable CAN failed");return 0U;}
        if(s_rr_state==RR_MONITOR) {
            result=ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,s_rr_target,s_rr_speed,s_rr_acc,1U);
            if(result==HAL_BUSY)return 1U;
            if(result!=HAL_OK){ArmDriver_ReachRetryFail("28 trial SAME ABS replay CAN failed");return 0U;}
            /* Keep the target, generation, attempt count and owner timeout:
             * clearing a flag must not fabricate progress or reset deadlines. */
        }
    }
    s_28_trial_pending=0U;s_rr_stall_pending=s_rr_stall_samples=0U;
    if(s_28_trial_clears==1U || s_28_trial_clears%10U==0U)
        ZDT_Console_Print("[28 TRIAL] id6 raw STALL ignored; clear/enable SAME target, no stall retry limit\r\n");
    return 1U;
}
#endif
void ArmDriver_PollReachRetry(void)
{
    arm_cal_axis_status_t a;uint32_t now=HAL_GetTick();HAL_StatusTypeDef result;char line[130];
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);
#if ARM_28_STALL_IGNORE_TRIAL
    if(!ArmDriver_Trial28StallService(&a,now))return;
#endif
    if(s_rr_state==RR_IDLE || s_rr_state==RR_FAILED)return;
    {uint32_t gap;uint8_t waiting=ArmCalibration_WaitForFeedback(&s_feedback_pause,now,&gap);
     s_rr_tick+=gap;s_rr_stall_tick+=gap;s_rr_progress_tick+=gap;
     if(waiting)return;}
    if(!ArmDriver_IsEnableRequested(ARM_DRIVER_REACH)){s_rr_state=RR_IDLE;return;}
    if(s_rr_reverse!=s_config.reach.reverse){ArmDriver_ReachRetryFail("direction changed during native job");return;}
    if(!a.position_sequence || !a.flags_sequence || now-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || now-a.flags_tick>ARM_ZDT_STATUS_TIMEOUT_MS) {
        ArmDriver_ReachRetryFail("reach feedback unavailable/stale");return;
    }
    if(a.flags&ARM_CAL_FLAG_PROTECTED){ArmDriver_ReachRetryFail("driver protection (not a stall retry)");return;}
    {
        arm_cal_axis_status_t lift;dm_feedback_t dm;
        ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
        DM_GetFeedback(ArmTask_GetMotor(),&dm);
        if(ArmTask_IsFaulted() || (dm.fb_count && dm.err_code>1U) ||
           (lift.flags_sequence && (lift.flags&(ARM_CAL_FLAG_STALL|ARM_CAL_FLAG_PROTECTED)))) {
            ArmDriver_ReachRetryFail("other arm fault cancels reach retry");return;
        }
    }
    if(s_rr_state==RR_MONITOR) {
        if(s_rr_stall_pending && a.position_sequence!=s_rr_pos_seq) {
            double error=fabs((double)a.position_counts-s_rr_goal_counts);
            s_rr_pos_seq=a.position_sequence;
            if(error+REACH_STALL_PROGRESS_COUNTS<=s_rr_best_error) {
                s_rr_best_error=error;s_rr_progress_tick=now;
            }
        }
        if(a.flags_sequence==s_rr_flags || (int32_t)(a.flags_tick-s_rr_tick)<0)return;
        s_rr_flags=a.flags_sequence;
        if(!(a.flags&ARM_CAL_FLAG_ENABLED)){ArmDriver_ReachRetryFail("reach disabled outside recovery");return;}
        if(!ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags)) {
            if(s_rr_stall_pending) {
                (void)snprintf(line,sizeof(line),"[REACH CHECK] stall flag cleared after %lums; no STOP or retry consumed\r\n",(unsigned long)(now-s_rr_stall_tick));ZDT_Console_Print(line);
            }
            s_rr_stall_pending=s_rr_stall_samples=0U;return;
        }
        if(!s_rr_stall_pending) {
            s_rr_stall_pending=1U;s_rr_stall_samples=1U;s_rr_stall_tick=s_rr_progress_tick=now;
            s_rr_pos_seq=s_rr_stall_pos_seq=a.position_sequence;s_rr_best_error=fabs((double)a.position_counts-s_rr_goal_counts);
            ZDT_Console_Print("[REACH CHECK] stall flag: observe NEW feedback/progress; no STOP or DM pause yet\r\n");return;
        }
        if(s_rr_stall_samples<255U)++s_rr_stall_samples;
        if(s_rr_stall_samples>=REACH_STALL_FRESH_SAMPLES && now-s_rr_stall_tick>=REACH_STALL_CONFIRM_MS &&
           now-s_rr_progress_tick>=REACH_STALL_CONFIRM_MS && a.position_sequence!=s_rr_stall_pos_seq &&
           (int32_t)(a.position_tick-s_rr_stall_tick)>0) {
            ZDT_Console_Print("[REACH CHECK] sustained stall + NEW samples + no target progress300ms; begin recovery\r\n");
            s_rr_stall_pending=0U;ArmDriver_ReachRetryNext(now);
        }
        return;
    }
    if(s_rr_state!=RR_RETREAT_MOVE && s_rr_state!=RR_RETREAT_HOLD && now-s_rr_tick>1000U){ArmDriver_ReachRetryNext(now);return;}
    switch(s_rr_state) {
    case RR_STOP:
        /* Internal STOP retains this cached job; public STOP cancels it. */
        result=ZDT_Motor_StopAddr(ARM_DRIVER_REACH_ID);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("STOP send failed");return;}
        s_rr_state=s_rr_unstarted_replay?RR_ENABLE:RR_CLEAR;s_rr_tick=HAL_GetTick();return;
    case RR_CLEAR:
        if(now-s_rr_tick<150U)return;
        result=ArmCalibration_ClearProtection(ARM_DRIVER_REACH);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("clear send failed");return;}
        s_rr_flags=a.flags_sequence;s_rr_state=RR_WAIT_CLEAR;s_rr_tick=HAL_GetTick();return;
    case RR_WAIT_CLEAR:
        if(a.flags_sequence==s_rr_flags || (int32_t)(a.flags_tick-s_rr_tick)<=0 || ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags))return;
        s_rr_state=RR_ENABLE;s_rr_tick=now;return;
    case RR_ENABLE:
        result=ArmDriver_Enable(ARM_DRIVER_REACH);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("enable send failed");return;}
        s_rr_flags=a.flags_sequence;s_rr_state=RR_WAIT_ENABLE;s_rr_tick=HAL_GetTick();return;
    case RR_WAIT_ENABLE:
        if(a.flags_sequence==s_rr_flags || (int32_t)(a.flags_tick-s_rr_tick)<=0 ||
           (int32_t)(a.position_tick-s_rr_tick)<=0 ||
           ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags) || !(a.flags&ARM_CAL_FLAG_ENABLED))return;
        s_rr_state=s_rr_retreat_done?RR_SEND:RR_RETREAT_SEND;s_rr_tick=now;return;
    case RR_RETREAT_SEND:
        {
            uint16_t speed=s_rr_speed<REACH_RETREAT_RPM?s_rr_speed:REACH_RETREAT_RPM;
            double timeout;
            result=ArmDriver_ReachPrepareRetreat(&a);
            if(result==HAL_BUSY) {
                s_rr_retreat_done=1U;s_rr_state=RR_SEND;s_rr_tick=now;
                ZDT_Console_Print("[REACH RETREAT] reverse blocked by recorded travel boundary; skip retreat, keep original goal\r\n");return;
            }
            if(result!=HAL_OK){ArmDriver_ReachRetryFail("reverse retreat reference/direction invalid");return;}
            result=ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,s_rr_retreat_pulses,speed,s_rr_acc,1U);
            if(result==HAL_BUSY)return;
            if(result!=HAL_OK){ArmDriver_ReachRetryFail("reverse retreat command failed");return;}
            timeout=fabs(s_rr_retreat_goal-s_rr_retreat_start)*600000.0/(65536.0*speed)+3000.0;
            if(timeout<10000.0)timeout=10000.0;
            if(timeout>30000.0)timeout=30000.0;
            s_rr_retreat_timeout=(uint32_t)timeout;s_rr_tick=HAL_GetTick();s_rr_state=RR_RETREAT_MOVE;
            s_rr_retreat_seq=a.position_sequence;s_rr_retreat_flags=a.flags_sequence;s_rr_retreat_samples=0U;
            s_rr_stall_pending=s_rr_stall_samples=0U;s_rr_progress_tick=s_rr_tick;
            s_rr_best_error=fabs(s_rr_retreat_start-s_rr_retreat_goal);
            (void)snprintf(line,sizeof(line),"[REACH RETREAT] attempt=%u/3 reverse=%.3fmm ABS=%ld speed=%uRPM; original ABS=%ld kept\r\n",
                s_rr_attempts,fabs(s_rr_retreat_goal-s_rr_retreat_start)/s_rr_counts_mm,(long)s_rr_retreat_pulses,speed,(long)s_rr_target);ZDT_Console_Print(line);return;
        }
    case RR_RETREAT_MOVE:
    case RR_RETREAT_HOLD:
        {
            double error=fabs((double)a.position_counts-s_rr_retreat_goal),movement;
            if(now-s_rr_tick>(s_rr_state==RR_RETREAT_MOVE?s_rr_retreat_timeout:2000U)) {
                ZDT_Console_Print("[REACH RETREAT] movement/hold timeout; recovery attempt failed\r\n");ArmDriver_ReachRetryNext(now);return;
            }
            if(!(a.flags&ARM_CAL_FLAG_ENABLED)){ArmDriver_ReachRetryFail("reach disabled during reverse retreat");return;}
            if(a.position_sequence==s_rr_retreat_seq || a.flags_sequence==s_rr_retreat_flags ||
               (int32_t)(a.position_tick-s_rr_tick)<=0 || (int32_t)(a.flags_tick-s_rr_tick)<=0)return;
            s_rr_retreat_seq=a.position_sequence;s_rr_retreat_flags=a.flags_sequence;
            if(s_rr_state==RR_RETREAT_MOVE) {
                if(((double)a.position_counts-s_rr_retreat_start)*(s_rr_retreat_goal-s_rr_retreat_start)<
                   -s_rr_counts_mm*0.2*fabs(s_rr_retreat_goal-s_rr_retreat_start)) {
                    ArmDriver_ReachRetryFail("reverse retreat moved in wrong direction");return;
                }
                if(error+REACH_STALL_PROGRESS_COUNTS<=s_rr_best_error){s_rr_best_error=error;s_rr_progress_tick=now;}
                if(error<=s_rr_counts_mm*0.2) {
                    result=ArmDriver_StopReachForHold();if(result==HAL_BUSY)return;
                    if(result!=HAL_OK){ArmDriver_ReachRetryFail("reverse retreat STOP failed");return;}
                    s_rr_state=RR_RETREAT_HOLD;s_rr_tick=HAL_GetTick();s_rr_retreat_last=(double)a.position_counts;s_rr_retreat_samples=0U;
                    ZDT_Console_Print("[REACH RETREAT] position reached; STOP, verify NEW held samples before original goal\r\n");return;
                }
                if(ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags)) {
                    if(!s_rr_stall_pending){s_rr_stall_pending=1U;s_rr_stall_tick=now;s_rr_stall_samples=1U;}
                    else if(s_rr_stall_samples<255U)++s_rr_stall_samples;
                    if(s_rr_stall_samples>=REACH_STALL_FRESH_SAMPLES && now-s_rr_stall_tick>=REACH_STALL_CONFIRM_MS &&
                       now-s_rr_progress_tick>=REACH_STALL_CONFIRM_MS){ArmDriver_ReachRetryNext(now);return;}
                } else s_rr_stall_pending=s_rr_stall_samples=0U;
                return;
            }
            movement=fabs((double)a.position_counts-s_rr_retreat_last);s_rr_retreat_last=(double)a.position_counts;
            if(error>s_rr_counts_mm*0.2 || movement>s_rr_counts_mm*0.1){s_rr_retreat_samples=0U;return;}
            if(s_rr_retreat_samples<255U)++s_rr_retreat_samples;
            if(s_rr_retreat_samples<ARM_MOTION_SETTLE_SAMPLES || now-s_rr_tick<ARM_MOTION_SETTLE_MS)return;
            s_rr_retreat_done=1U;s_rr_state=ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags)?RR_CLEAR:RR_SEND;s_rr_tick=now;
            ZDT_Console_Print("[REACH RETREAT] actual reverse STOP/hold confirmed; NOW retry original ABS target\r\n");return;
        }
    case RR_SEND:
        if(ArmCalibration_StallFault(ARM_DRIVER_REACH,a.flags)){ArmDriver_ReachRetryNext(now);return;}
        result=ArmDriver_MovePulsesProfileMode(ARM_DRIVER_REACH,s_rr_target,ArmDriver_ReachRetrySpeedRpm(),s_rr_acc,1U);
        if(result==HAL_BUSY)return;
        if(result!=HAL_OK){ArmDriver_ReachRetryFail("absolute target resend failed");return;}
        ++s_rr_generation;s_rr_flags=a.flags_sequence;s_rr_state=RR_MONITOR;s_rr_tick=HAL_GetTick();
        s_rr_start_counts=a.position_counts;s_rr_unstarted_replay=0U;
        s_rr_stall_pending=s_rr_stall_samples=0U;s_rr_pos_seq=a.position_sequence;
        s_rr_best_error=fabs((double)a.position_counts-s_rr_goal_counts);s_rr_progress_tick=s_rr_tick;
        (void)snprintf(line,sizeof(line),"[REACH RETRY] attempt=%u/3 SAME ABS target=%ld speed=%uRPM resent; wait actual position\r\n",s_rr_attempts,(long)s_rr_target,ArmDriver_ReachRetrySpeedRpm());
        ZDT_Console_Print(line);return;
    default:return;
    }
}
static HAL_StatusTypeDef ArmDriver_MovePulses(arm_driver_axis_t axis,int32_t pulses)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(axis);
    if(cfg==NULL)return HAL_ERROR;
    return ArmDriver_MovePulsesProfile(axis,pulses,ArmDriver_GetLinearSpeed(axis),cfg->acceleration);
}
HAL_StatusTypeDef ArmDriver_ReachPositionPulses(int32_t pulses,uint16_t speed,uint8_t acceleration)
{
    return ArmDriver_MovePulsesProfile(ARM_DRIVER_REACH,pulses,speed,acceleration);
}

static HAL_StatusTypeDef ArmDriver_MoveMm(arm_driver_axis_t axis, float distance_mm)
{
    const arm_driver_linear_config_t *cfg;
    double rounded;
    if ((ArmDriver_ContextValid() == 0U) || !isfinite(distance_mm)) { return HAL_ERROR; }
    cfg = ArmDriver_LinearConfig(axis);
    if ((cfg == NULL) || (cfg->pulses_per_mm <= 0.0f)) { return HAL_ERROR; }
    rounded = round((double)distance_mm * (double)cfg->pulses_per_mm);
    /* Double prevents float rounding INT32_MAX up to 2147483648 before cast. */
    if (!isfinite(rounded) || (rounded > (double)INT32_MAX) ||
        (rounded < -(double)INT32_MAX) || ((distance_mm != 0.0f) && (rounded == 0.0)))
    {
        return HAL_ERROR;
    }
    return ArmDriver_MovePulses(axis, (int32_t)rounded);
}

HAL_StatusTypeDef ArmDriver_LiftByPulses(int32_t pulses)
{
    return ArmDriver_MovePulses(ARM_DRIVER_LIFT, pulses);
}

HAL_StatusTypeDef ArmDriver_LiftAbsoluteRawPulses(int32_t pulses,uint16_t speed,float target_height)
{
    const arm_driver_linear_config_t *cfg=ArmDriver_LinearConfig(ARM_DRIVER_LIFT);
    arm_cal_axis_status_t a={0};HAL_StatusTypeDef result;double delta;
    uint16_t cap;
    if(!ArmDriver_ContextValid() || !cfg || pulses==INT32_MIN || !isfinite(target_height) ||
       !ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT))return HAL_ERROR;
    cap=ArmDriver_GetLinearSpeed(ARM_DRIVER_LIFT);
    if(ArmCalibration_EmptyLiftMotionActive() || (ArmMaterial_IsActive() && ArmServo_GripIsOpen()))
        cap=(uint16_t)fminf(cap*ARM_EMPTY_LIFT_SPEED_SCALE,ARM_EMPTY_LIFT_RPM_MAX);
    if(!speed || speed>cap)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a);
    if(!a.position_sequence || HAL_GetTick()-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS)return HAL_BUSY;
    /* max_move_pulses is a displacement cap, not an absolute-coordinate cap. */
    delta=(double)pulses-(double)a.position_counts*ARM_LIFT_POSITION_STEPS_REV/65536.0;
    if(cfg->max_move_pulses && fabs(delta)>cfg->max_move_pulses)return HAL_ERROR;
    if(s_height_guard) {
        float current,target,sign=s_config.rotation_reverse?-1.0f:1.0f;
        if(!s_height_valid || HAL_GetTick()-s_height_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS)return HAL_BUSY;
        if(s_top_guard && target_height>0.0f)return HAL_ERROR;
        if(target_height<s_height_mm && target_height<s_low_height_mm) {
            target=sign*(ArmTask_GetMotor()->target_pos_rad-s_config.rotation_zero_rad)/
                (ARM_DEG_TO_RAD*s_config.rotation_ratio);
            if(ArmDriver_GetRotationDeg(&current)!=HAL_OK || current<s_low_min_deg || current>s_low_max_deg ||
               (ArmTask_IsEnabled() && (!isfinite(target) || target<s_low_min_deg || target>s_low_max_deg)))return HAL_ERROR;
        }
    }
    result=ArmDriver_StepperReady(2U);
    if(result!=HAL_OK)return result;
    result=ZDT_Motor_MovePositionPulsesAbsAddr(ARM_DRIVER_LIFT_ID,pulses,speed,cfg->acceleration,0U);
    if(result!=HAL_OK) {
        /* Existing partial-CAN-write behavior, same as the relative FD path. */
        s_stepper_enable_mask&=(uint8_t)~ArmDriver_Mask(ARM_DRIVER_LIFT);
        (void)ZDT_Motor_StopAddr(ARM_DRIVER_LIFT_ID);
    }
    return result;
}

HAL_StatusTypeDef ArmDriver_ReachByPulses(int32_t pulses)
{
    return ArmDriver_MovePulses(ARM_DRIVER_REACH, pulses);
}

HAL_StatusTypeDef ArmDriver_LiftByMm(float distance_mm)
{
    return ArmDriver_MoveMm(ARM_DRIVER_LIFT, distance_mm);
}

HAL_StatusTypeDef ArmDriver_ReachByMm(float distance_mm)
{
    return ArmDriver_MoveMm(ARM_DRIVER_REACH, distance_mm);
}

HAL_StatusTypeDef ArmDriver_Stop(arm_driver_axis_t axis)
{
    HAL_StatusTypeDef result;
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    if (axis == ARM_DRIVER_ROTATION) { return ArmTask_Disable(); }
    if (ArmDriver_LinearConfig(axis) == NULL) { return HAL_ERROR; }
    if(axis==ARM_DRIVER_REACH)s_rr_state=RR_IDLE;
    result = ZDT_Motor_StopAddr(ArmDriver_Address(axis));
    if (result != HAL_OK) { s_stepper_enable_mask &= (uint8_t)~ArmDriver_Mask(axis); }
    return result;
}

HAL_StatusTypeDef ArmDriver_StopAll(void)
{
    HAL_StatusTypeDef result;
    HAL_StatusTypeDef next;
    if (ArmDriver_ContextValid() == 0U) { return HAL_ERROR; }
    result = ArmDriver_Stop(ARM_DRIVER_LIFT);
    next = ArmDriver_Stop(ARM_DRIVER_REACH);
    if (result == HAL_OK) { result = next; }
    next = ArmDriver_Stop(ARM_DRIVER_ROTATION);
    if (result == HAL_OK) { result = next; }
    next = ArmServo_StopAll();
    if (result == HAL_OK) { result = next; }
    return result;
}
