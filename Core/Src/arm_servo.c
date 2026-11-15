#include "arm_servo.h"
#include "arm_speed.h"
#include "tim.h"
#include "zdt_motor.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static arm_servo_config_t s_cfg;
static arm_servo_status_t s_state[2];
static float s_rate_factor[2]={1.0f,1.0f};
static uint8_t s_inited;
static volatile uint8_t s_stop_request;
static uint8_t s_stop_latched;
static uint32_t s_last_tick;
static uint16_t s_grip_open_us;
static uint16_t s_grip_close_us;
static uint8_t s_grip_valid;
static uint8_t s_grip_open_tracking;
static uint32_t s_grip_open_since;
static float s_grip_move_scale=1.0f; /* Owner-only, per command; not persistent configuration. */
static float s_ui_degrees[5]={ARM_SERVO_GRIP_OPEN_DEG,ARM_SERVO_GRIP_CLOSE_DEG,
    ARM_SERVO_TABLE_SLOT1_DEG,ARM_SERVO_TABLE_SLOT2_DEG,ARM_SERVO_TABLE_SLOT3_DEG};

static uint8_t Servo_Context(void) { return ((s_inited != 0U) && (__get_IPSR() == 0U)) ? 1U : 0U; }
static uint8_t Servo_IdValid(arm_servo_id_t id) { return ((id == ARM_SERVO_GRIP) || (id == ARM_SERVO_TABLE)) ? 1U : 0U; }
static uint32_t Servo_Channel(arm_servo_id_t id) { return (id == ARM_SERVO_GRIP) ? TIM_CHANNEL_1 : TIM_CHANNEL_2; }
static arm_servo_channel_config_t *Servo_Config(arm_servo_id_t id) { return (id == ARM_SERVO_GRIP) ? &s_cfg.grip : &s_cfg.table; }
static uint8_t Servo_PulseValid(const arm_servo_channel_config_t *c, uint16_t pulse)
{
    return ((pulse >= c->min_us) && (pulse <= c->max_us)) ? 1U : 0U;
}
static uint8_t Servo_ConfigValid(const arm_servo_channel_config_t *c)
{
    return ((c->min_us >= 500U) && (c->max_us <= 2500U) && (c->min_us < c->max_us) &&
            Servo_PulseValid(c, c->enable_us) && isfinite(c->angle_span_deg) &&
            (c->angle_span_deg >= 0.0f) && (c->angle_span_deg <= 360.0f) &&
            isfinite(c->slew_us_per_second) && (c->slew_us_per_second > 0.0f) &&
            (c->slew_us_per_second <= 20000.0f) && (c->reverse_angle <= 1U)) ? 1U : 0U;
}
static uint16_t Servo_Pulse(float angle) { return (uint16_t)(500.0f + angle * (2000.0f / 360.0f) + 0.5f); }
float ArmServo_MaterialSlotDeg(uint8_t slot)
{return slot>=1U && slot<=3U?s_ui_degrees[slot+1U]:NAN;}
float ArmServo_GripOpenDeg(void){return s_ui_degrees[0];}
float ArmServo_GripCloseDeg(void){return s_ui_degrees[1];}
HAL_StatusTypeDef ArmServo_GetUiParameters(float out[5])
{if(!Servo_Context() || !out)return HAL_ERROR;memcpy(out,s_ui_degrees,sizeof(s_ui_degrees));return HAL_OK;}
HAL_StatusTypeDef ArmServo_SelectMaterialSlot(uint8_t slot)
{return ArmServo_TestAngleDeg(ARM_SERVO_TABLE,ArmServo_MaterialSlotDeg(slot));}
static void Servo_Write(arm_servo_id_t id)
{
    __HAL_TIM_SET_COMPARE(&htim4, Servo_Channel(id), (uint32_t)(s_state[id].output_us + 0.5f));
}

void ArmServo_DefaultConfig(arm_servo_config_t *out)
{
    if (out == NULL) { return; }
    (void)memset(out, 0, sizeof(*out));
    out->timer_clock_hz = 275000000U;
    out->grip.min_us = 500U; out->grip.max_us = 2500U;
    out->grip.enable_us = 1500U;
    out->grip.slew_us_per_second = 500.0f;
    out->table = out->grip;
    out->table.slew_us_per_second = 1072.5f; /* V223: 975 x1.1. */
    out->grip.slew_us_per_second = 2145.0f; /* V223: 1950 x1.1. */
    out->grip.angle_span_deg = ARM_SERVO_GRIP_SPAN_DEG;
    out->grip.enable_us = Servo_Pulse(ARM_SERVO_GRIP_OPEN_DEG);
    out->table.angle_span_deg = 360.0f;
    out->table.enable_us = Servo_Pulse(ArmServo_MaterialSlotDeg(ARM_SERVO_TABLE_BOOT_SLOT));
    out->table_slot_us[0] = Servo_Pulse(ARM_SERVO_TABLE_SLOT1_DEG);
    out->table_slot_us[1] = Servo_Pulse(ARM_SERVO_TABLE_SLOT2_DEG);
    out->table_slot_us[2] = Servo_Pulse(ARM_SERVO_TABLE_SLOT3_DEG);
    out->table_slots_valid = 0x07U;
}

HAL_StatusTypeDef ArmServo_Init(const arm_servo_config_t *config)
{
    TIM_OC_InitTypeDef oc = {0};
    HAL_StatusTypeDef result;
    uint8_t i;
    if ((config == NULL) || (__get_IPSR() != 0U)) { return HAL_ERROR; }
    if (s_inited != 0U) { return HAL_BUSY; }
    if ((htim4.Instance != TIM4) || (htim4.State == HAL_TIM_STATE_RESET) ||
        (config->timer_clock_hz < 1000000U) || (config->timer_clock_hz > 655360000U) ||
        ((config->timer_clock_hz % 1000000U) != 0U) ||
        !Servo_ConfigValid(&config->grip) || !Servo_ConfigValid(&config->table) ||
        (config->table_slots_valid > 7U)) { return HAL_ERROR; }
    /* Refuse to reconfigure an already running timer owned by another driver. */
    if ((htim4.Instance->CR1 & TIM_CR1_CEN) != 0U) { return HAL_BUSY; }
    for (i = 0U; i < ARM_SERVO_TABLE_SLOTS; ++i)
    {
        if (((config->table_slots_valid & (1U << i)) != 0U) &&
            !Servo_PulseValid(&config->table, config->table_slot_us[i])) { return HAL_ERROR; }
    }
    htim4.Init.Prescaler = config->timer_clock_hz / 1000000U - 1U;
    htim4.Init.Period = 19999U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    result = HAL_TIM_PWM_Init(&htim4);
    if (result != HAL_OK) { return result; }
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse = 0U; /* No startup motion/PWM. */
    for (i = 0U; i < 2U; ++i)
    {
        result = HAL_TIM_PWM_ConfigChannel(&htim4, &oc, Servo_Channel((arm_servo_id_t)i));
        if (result != HAL_OK) { return result; }
    }
    s_cfg = *config;
    s_grip_move_scale=1.0f;
    s_grip_open_tracking=0U;
    s_grip_open_us=Servo_Pulse(ARM_SERVO_GRIP_OPEN_DEG);
    s_grip_close_us=Servo_Pulse(ARM_SERVO_GRIP_CLOSE_DEG);
    s_grip_valid=(uint8_t)((Servo_PulseValid(&config->grip,s_grip_open_us)?1U:0U) |
                         (Servo_PulseValid(&config->grip,s_grip_close_us)?2U:0U));
    s_state[0].output_us = (float)config->grip.enable_us;
    s_state[1].output_us = (float)config->table.enable_us;
    s_state[0].target_us = config->grip.enable_us;
    s_state[1].target_us = config->table.enable_us;
    s_last_tick = HAL_GetTick();
    s_inited = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef ArmServo_Enable(arm_servo_id_t id)
{
    HAL_StatusTypeDef result;
    if (!Servo_Context() || !Servo_IdValid(id) || s_stop_request || s_stop_latched || s_state[id].fault)
    {
        return HAL_ERROR;
    }
    if (s_state[id].enabled) { return HAL_OK; }
    if(id==ARM_SERVO_GRIP){s_grip_open_tracking=0U;s_grip_move_scale=1.0f;}
    Servo_Write(id); /* Establish first pulse before enabling channel. */
    result = HAL_TIM_PWM_Start(&htim4, Servo_Channel(id));
    if (result == HAL_OK) {
        s_state[id].enabled = 1U; s_last_tick = HAL_GetTick();
        if(id==ARM_SERVO_GRIP && (s_grip_valid&1U) && s_state[id].target_us==s_grip_open_us) {
            s_grip_open_tracking=1U;s_grip_open_since=s_last_tick;
        }
    }
    else { s_state[id].fault = 1U; }
    return result;
}

HAL_StatusTypeDef ArmServo_Disable(arm_servo_id_t id)
{
    HAL_StatusTypeDef result;
    if (!Servo_Context() || !Servo_IdValid(id)) { return HAL_ERROR; }
    if(id==ARM_SERVO_GRIP){s_grip_open_tracking=0U;s_grip_move_scale=1.0f;}
    result = HAL_TIM_PWM_Stop(&htim4, Servo_Channel(id));
    if (result == HAL_OK) { s_state[id].enabled = 0U; s_state[id].fault = 0U; }
    else { s_state[id].fault = 1U; }
    s_state[id].target_us = (uint16_t)(s_state[id].output_us + 0.5f);
    s_state[id].ramping = 0U;
    return result;
}

HAL_StatusTypeDef ArmServo_SetPulseUs(arm_servo_id_t id, uint16_t pulse_us)
{
    if (!Servo_Context() || !Servo_IdValid(id) || s_stop_request || s_stop_latched ||
        !s_state[id].enabled || s_state[id].fault || !Servo_PulseValid(Servo_Config(id), pulse_us))
    {
        return HAL_ERROR;
    }
    if(id==ARM_SERVO_GRIP && s_state[id].target_us!=pulse_us)s_grip_open_tracking=0U;
    if(id==ARM_SERVO_GRIP)s_grip_move_scale=1.0f;
    if(s_state[id].target_us!=pulse_us || !s_state[id].ramping)
        s_rate_factor[id]=ArmSpeed_Factor(id==ARM_SERVO_GRIP?ARM_SPEED_GRIP:ARM_SPEED_TABLE);
    s_state[id].target_us = pulse_us;
    s_state[id].ramping = (fabsf(s_state[id].output_us - (float)pulse_us) >= 0.5f) ? 1U : 0U;
    if(id==ARM_SERVO_GRIP && (s_grip_valid&1U) && pulse_us==s_grip_open_us && !s_grip_open_tracking) {
        s_grip_open_tracking=1U;s_grip_open_since=HAL_GetTick();
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Servo_AnglePulse(arm_servo_id_t id,float angle_deg,uint16_t *pulse)
{
    arm_servo_channel_config_t *c;
    float fraction;
    if (!Servo_Context() || !Servo_IdValid(id) || !isfinite(angle_deg)) { return HAL_ERROR; }
    c = Servo_Config(id);
    if ((c->angle_span_deg <= 0.0f) || (angle_deg < 0.0f) || (angle_deg > c->angle_span_deg)) { return HAL_ERROR; }
    fraction = angle_deg / c->angle_span_deg;
    if (c->reverse_angle) { fraction = 1.0f - fraction; }
    *pulse=(uint16_t)((float)c->min_us + fraction * (float)(c->max_us - c->min_us) + 0.5f);
    return HAL_OK;
}
HAL_StatusTypeDef ArmServo_SetAngleDeg(arm_servo_id_t id,float angle_deg)
{
    uint16_t pulse;if(Servo_AnglePulse(id,angle_deg,&pulse)!=HAL_OK)return HAL_ERROR;
    return ArmServo_SetPulseUs(id,pulse);
}
HAL_StatusTypeDef ArmServo_SetUiParameters(const float values[5])
{
    uint16_t pulses[5];uint8_t i;
    if(!Servo_Context() || !values)return HAL_ERROR;
    for(i=0U;i<5U;i++)if(Servo_AnglePulse(i<2U?ARM_SERVO_GRIP:ARM_SERVO_TABLE,values[i],&pulses[i])!=HAL_OK)return HAL_ERROR;
    memcpy(s_ui_degrees,values,sizeof(s_ui_degrees));s_grip_open_us=pulses[0];s_grip_close_us=pulses[1];
    s_grip_valid=3U;s_grip_open_tracking=0U;return HAL_OK;
}
HAL_StatusTypeDef ArmServo_TestAngleDeg(arm_servo_id_t id,float angle_deg)
{
    uint16_t pulse;
    if(!Servo_Context() || !Servo_IdValid(id) || !isfinite(angle_deg) ||
       s_stop_request || s_stop_latched || s_state[id].fault)return HAL_ERROR;
    if(id==ARM_SERVO_TABLE && angle_deg>360.0f && angle_deg<=385.0f) {
        /* Explicit tray experiment only: extrapolate the existing 360deg
         * slope, never rescale the normal presets or silently wrap to 25deg. */
        if(s_cfg.table.min_us!=500U || s_cfg.table.max_us!=2500U ||
           s_cfg.table.angle_span_deg!=360.0f || s_cfg.table.reverse_angle)return HAL_ERROR;
        pulse=Servo_Pulse(angle_deg); /* 385deg -> 2639us */
        s_state[id].target_us=pulse;
        if(s_state[id].enabled) {
            s_state[id].ramping=(fabsf(s_state[id].output_us-(float)pulse)>=0.5f)?1U:0U;
            return HAL_OK;
        }
        s_state[id].output_us=(float)pulse;s_state[id].ramping=0U;
        return ArmServo_Enable(id);
    }
    if(Servo_AnglePulse(id,angle_deg,&pulse)!=HAL_OK || s_stop_request || s_stop_latched || s_state[id].fault)return HAL_ERROR;
    if(s_state[id].enabled)return ArmServo_SetPulseUs(id,pulse);
    s_state[id].output_us=(float)pulse;s_state[id].target_us=pulse;s_state[id].ramping=0U;
    return ArmServo_Enable(id);
}
HAL_StatusTypeDef ArmServo_GripCloseScaled(float scale)
{
    HAL_StatusTypeDef result;
    if(!Servo_Context() || !s_state[ARM_SERVO_GRIP].enabled || !isfinite(scale) || scale<=0.0f || scale>1.0f)return HAL_ERROR;
    result=ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ArmServo_GripCloseDeg());
    if(result==HAL_OK)s_grip_move_scale=scale;
    return result;
}

HAL_StatusTypeDef ArmServo_SetAngleSpan(arm_servo_id_t id, float span_deg)
{
    if (!Servo_Context() || !Servo_IdValid(id) || s_state[id].enabled || !isfinite(span_deg) ||
        (span_deg <= 0.0f) || (span_deg > 360.0f)) { return HAL_ERROR; }
    Servo_Config(id)->angle_span_deg = span_deg;
    if(id==ARM_SERVO_GRIP) {
        uint8_t i;uint16_t pulses[2];pulses[0]=s_grip_open_us;pulses[1]=s_grip_close_us;
        for(i=0U;i<2U;i++) {
            float fraction=(float)(pulses[i]-s_cfg.grip.min_us)/(s_cfg.grip.max_us-s_cfg.grip.min_us);
            s_ui_degrees[i]=(s_cfg.grip.reverse_angle?1.0f-fraction:fraction)*span_deg;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef ArmServo_SelectMaterialSlotForPoint(uint8_t slot,uint8_t speed_point)
{
    uint16_t previous=s_state[ARM_SERVO_TABLE].target_us;
    uint8_t moving=s_state[ARM_SERVO_TABLE].ramping;
    HAL_StatusTypeDef result=ArmServo_SelectMaterialSlot(slot);
    if(result==HAL_OK && (!moving || s_state[ARM_SERVO_TABLE].target_us!=previous))
        s_rate_factor[ARM_SERVO_TABLE]=ArmSpeed_PointFactor(speed_point,ARM_SPEED_TABLE);
    return result;
}
HAL_StatusTypeDef ArmServo_SetSlew(arm_servo_id_t id, float us_per_second)
{
    if (!Servo_Context() || !Servo_IdValid(id) || !isfinite(us_per_second) ||
        (us_per_second <= 0.0f) || (us_per_second > 20000.0f)) { return HAL_ERROR; }
    Servo_Config(id)->slew_us_per_second = us_per_second;
    if(id==ARM_SERVO_GRIP)s_grip_move_scale=1.0f;
    return HAL_OK;
}

HAL_StatusTypeDef ArmServo_GripCapture(uint8_t closed)
{
    if (!Servo_Context() || (closed > 1U) || !s_state[0].enabled || s_state[0].ramping ||
        s_state[0].fault || s_stop_request || s_stop_latched) { return HAL_ERROR; }
    if (closed) { s_grip_close_us = s_state[0].target_us; s_grip_valid |= 2U; }
    else { s_grip_open_us = s_state[0].target_us; s_grip_valid |= 1U;s_grip_open_tracking=1U;s_grip_open_since=HAL_GetTick(); }
    {arm_servo_channel_config_t *c=&s_cfg.grip;float fraction=(float)(s_state[0].target_us-c->min_us)/(c->max_us-c->min_us);
     s_ui_degrees[closed?1U:0U]=(c->reverse_angle?1.0f-fraction:fraction)*c->angle_span_deg;}
    return HAL_OK;
}
HAL_StatusTypeDef ArmServo_GripOpen(void)
{
    return ((s_grip_valid & 1U) != 0U) ? ArmServo_SetPulseUs(ARM_SERVO_GRIP, s_grip_open_us) : HAL_ERROR;
}
HAL_StatusTypeDef ArmServo_GripClose(void)
{
    return ((s_grip_valid & 2U) != 0U) ? ArmServo_SetPulseUs(ARM_SERVO_GRIP, s_grip_close_us) : HAL_ERROR;
}
uint8_t ArmServo_GripIsClosed(void)
{
    return Servo_Context() && (s_grip_valid & 2U) && s_state[0].enabled &&
        !s_state[0].ramping && !s_state[0].fault && !s_stop_request && !s_stop_latched &&
        s_state[0].target_us==s_grip_close_us && fabsf(s_state[0].output_us-s_grip_close_us)<=1.0f;
}
uint8_t ArmServo_GripIsOpen(void)
{
    return Servo_Context() && (s_grip_valid & 1U) && s_state[0].enabled &&
        !s_state[0].ramping && !s_state[0].fault && !s_stop_request && !s_stop_latched &&
        s_state[0].target_us==s_grip_open_us && fabsf(s_state[0].output_us-s_grip_open_us)<=1.0f;
}
static uint8_t Servo_GripOpenTarget(void)
{
    uint16_t target;
    if(!Servo_Context() || !(s_grip_valid&1U))return 0U;
    target=s_grip_open_us;
    return s_state[0].enabled && !s_state[0].fault &&
        !s_stop_request && !s_stop_latched && s_state[0].target_us==target;
}
uint8_t ArmServo_GripOpenConfirmed(void)
{return Servo_GripOpenTarget() && s_grip_open_tracking && HAL_GetTick()-s_grip_open_since>=ARM_SERVO_GRIP_OPEN_MIN_MS;}
uint8_t ArmServo_GripOpeningOrOpen(void){return Servo_GripOpenTarget();}
HAL_StatusTypeDef ArmServo_TableCapture(uint8_t slot)
{
    if (!Servo_Context() || (slot < 1U) || (slot > 3U) || !s_state[1].enabled || s_state[1].ramping ||
        s_state[1].fault || s_stop_request || s_stop_latched ||
        !Servo_PulseValid(&s_cfg.table,s_state[1].target_us)) { return HAL_ERROR; }
    s_cfg.table_slot_us[slot - 1U] = s_state[1].target_us;
    s_cfg.table_slots_valid |= (uint8_t)(1U << (slot - 1U));
    return HAL_OK;
}
HAL_StatusTypeDef ArmServo_TableGo(uint8_t slot)
{
    if (!Servo_Context() || (slot < 1U) || (slot > 3U) ||
        ((s_cfg.table_slots_valid & (1U << (slot - 1U))) == 0U)) { return HAL_ERROR; }
    return ArmServo_SetPulseUs(ARM_SERVO_TABLE, s_cfg.table_slot_us[slot - 1U]);
}

HAL_StatusTypeDef ArmServo_Stop(arm_servo_id_t id)
{
    if (!Servo_Context() || !Servo_IdValid(id)) { return HAL_ERROR; }
    if (s_state[id].fault) { return HAL_ERROR; }
    if(id==ARM_SERVO_GRIP)s_grip_move_scale=1.0f;
    s_state[id].target_us = (uint16_t)(s_state[id].output_us + 0.5f);
    s_state[id].output_us = (float)s_state[id].target_us;
    s_state[id].ramping = 0U;
    if (s_state[id].enabled) { Servo_Write(id); }
    return HAL_OK;
}
HAL_StatusTypeDef ArmServo_StopAll(void)
{
    HAL_StatusTypeDef a, b;
    if (__get_IPSR() != 0U) { return HAL_ERROR; }
    if (!s_inited) { return HAL_OK; }
    a = ArmServo_Stop(ARM_SERVO_GRIP);
    b = ArmServo_Stop(ARM_SERVO_TABLE);
    return (a != HAL_OK) ? a : b;
}
void ArmServo_RequestSafeStop(void) { s_stop_request = 1U; }
HAL_StatusTypeDef ArmServo_ClearSafeStop(void)
{
    HAL_StatusTypeDef a, b;
    if (!Servo_Context() || s_stop_request) { return HAL_ERROR; }
    if (!s_stop_latched) { return HAL_OK; }
    a = ArmServo_Disable(ARM_SERVO_GRIP);
    b = ArmServo_Disable(ARM_SERVO_TABLE);
    if ((a == HAL_OK) && (b == HAL_OK)) { s_stop_latched = 0U; }
    return (a != HAL_OK) ? a : b;
}
void ArmServo_Update(uint32_t now_ms)
{
    uint32_t elapsed;
    uint8_t i;
    if (!Servo_Context()) { return; }
    if (s_stop_request)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq(); s_stop_request = 0U; s_stop_latched = 1U; __set_PRIMASK(mask);
        (void)ArmServo_Disable(ARM_SERVO_GRIP);
        (void)ArmServo_Disable(ARM_SERVO_TABLE);
    }
    elapsed = now_ms - s_last_tick;
    s_last_tick = now_ms;
    if (elapsed > 50U) { elapsed = 50U; }
    if (s_stop_latched) { return; }
    for (i = 0U; i < 2U; ++i)
    {
        float delta;
        float step;
        if (!s_state[i].enabled || s_state[i].fault) { continue; }
        delta = (float)s_state[i].target_us - s_state[i].output_us;
        step = fminf(20000.0f,Servo_Config((arm_servo_id_t)i)->slew_us_per_second*s_rate_factor[i]) * (float)elapsed / 1000.0f;
        if(i==ARM_SERVO_GRIP)step*=s_grip_move_scale;
        if (delta > step) { delta = step; }
        else if (delta < -step) { delta = -step; }
        s_state[i].output_us += delta;
        s_state[i].ramping = (fabsf(s_state[i].output_us - (float)s_state[i].target_us) >= 0.5f) ? 1U : 0U;
        if (!s_state[i].ramping) { s_state[i].output_us = (float)s_state[i].target_us; }
        Servo_Write((arm_servo_id_t)i);
    }
    if(Servo_GripOpenTarget()) {
        if(!s_grip_open_tracking){s_grip_open_tracking=1U;s_grip_open_since=now_ms;}
    } else s_grip_open_tracking=0U;
}
void ArmServo_GetStatus(arm_servo_id_t id, arm_servo_status_t *out)
{
    if ((out != NULL) && Servo_Context() && Servo_IdValid(id)) { *out = s_state[id]; }
}
void ArmServo_PrintStatus(void)
{
    char line[190];
    uint8_t i;
    if (!Servo_Context()) { ZDT_Console_Print("[ARM] servos not initialized\r\n"); return; }
    for (i = 0U; i < 2U; ++i)
    {
        arm_servo_channel_config_t *c=Servo_Config((arm_servo_id_t)i);
        float scale=c->angle_span_deg/(float)(c->max_us-c->min_us);
        (void)snprintf(line, sizeof(line), "[ARM] %s enabled=%u output_us=%.1f target_us=%u pwm_angle=%.2f target_angle=%.2f span=%.0f ramping=%u fault=%u (PWM estimate)\r\n",
            (i == 0U) ? "grip" : "table", (unsigned int)s_state[i].enabled,
            (double)s_state[i].output_us, (unsigned int)s_state[i].target_us,
            (double)((s_state[i].output_us-c->min_us)*scale),(double)((s_state[i].target_us-c->min_us)*scale),(double)c->angle_span_deg,
            (unsigned int)s_state[i].ramping, (unsigned int)s_state[i].fault);
        ZDT_Console_Print(line);
    }
    (void)snprintf(line,sizeof(line),"[ARM PRESETS] open=%.4f close=%.4f table1=%.4f table2=%.4f table3=%.4f\r\n",
        (double)s_ui_degrees[0],(double)s_ui_degrees[1],(double)s_ui_degrees[2],(double)s_ui_degrees[3],(double)s_ui_degrees[4]);ZDT_Console_Print(line);
}

HAL_StatusTypeDef ArmServo_Command(uint8_t count, char *const *args)
{
    arm_servo_id_t id;
    char *end;
    float value;
    if ((args == NULL) || (count < 3U) || !Servo_Context()) { return HAL_ERROR; }
    if (strcmp(args[1], "grip") == 0) { id = ARM_SERVO_GRIP; }
    else if (strcmp(args[1], "table") == 0) { id = ARM_SERVO_TABLE; }
    else { return HAL_ERROR; }
    if((id==ARM_SERVO_GRIP && count==5U && !strcmp(args[2],"preset")) ||
       (id==ARM_SERVO_TABLE && count==6U && !strcmp(args[2],"presets"))) {
        float values[5];uint8_t i,start=id==ARM_SERVO_GRIP?0U:2U,n=id==ARM_SERVO_GRIP?2U:3U;
        HAL_StatusTypeDef result=ArmServo_GetUiParameters(values);
        for(i=0U;result==HAL_OK && i<n;i++) {
            values[start+i]=strtof(args[3U+i],&end);
            if(end==args[3U+i] || *end || !isfinite(values[start+i]))result=HAL_ERROR;
        }
        if(result==HAL_OK)result=ArmServo_SetUiParameters(values);
        {char reply[96];(void)snprintf(reply,sizeof(reply),"[ARM PRESET] HAL=%u; RAM parameters only, no PWM move\r\n",(unsigned int)result);ZDT_Console_Print(reply);}
        return result;
    }
    if (count == 3U)
    {
        if (strcmp(args[2], "enable") == 0) { return ArmServo_Enable(id); }
        if (strcmp(args[2], "disable") == 0) { return ArmServo_Disable(id); }
        if (strcmp(args[2], "stop") == 0) { return ArmServo_Stop(id); }
        if (strcmp(args[2], "clear") == 0) { return ArmServo_ClearSafeStop(); }
        if (strcmp(args[2], "status") == 0) { ArmServo_PrintStatus();return HAL_OK; }
        if ((id == ARM_SERVO_GRIP) && (strcmp(args[2], "open") == 0)) { return ArmServo_GripOpen(); }
        if ((id == ARM_SERVO_GRIP) && (strcmp(args[2], "close") == 0)) { return ArmServo_GripClose(); }
    }
    if (count != 4U) { return HAL_ERROR; }
    if ((id == ARM_SERVO_GRIP) && (strcmp(args[2], "save") == 0))
    {
        if (strcmp(args[3], "open") == 0) { return ArmServo_GripCapture(0U); }
        if (strcmp(args[3], "close") == 0) { return ArmServo_GripCapture(1U); }
        return HAL_ERROR;
    }
    value = strtof(args[3], &end);
    if ((end == args[3]) || (*end != '\0') || !isfinite(value)) { return HAL_ERROR; }
    if (strcmp(args[2], "angle") == 0 || strcmp(args[2],"test")==0) {
        HAL_StatusTypeDef result=strcmp(args[2],"test")==0?ArmServo_TestAngleDeg(id,value):ArmServo_SetAngleDeg(id,value);
        char line[155];
        (void)snprintf(line,sizeof(line),"[SERVO] %s %s requested=%.2fdeg target=%uus enabled=%u ramping=%u HAL=%u\r\n",
            id==ARM_SERVO_GRIP?"grip":"table",args[2],(double)value,s_state[id].target_us,s_state[id].enabled,s_state[id].ramping,(unsigned int)result);
        ZDT_Console_Print(line);return result;
    }
    if (strcmp(args[2], "span") == 0) { return ArmServo_SetAngleSpan(id, value); }
    if (strcmp(args[2], "speed") == 0) { return ArmServo_SetSlew(id, value); }
    if ((strcmp(args[2], "pulse") == 0) && (value >= 500.0f) && (value <= 2500.0f) && (floorf(value) == value))
    {
        return ArmServo_SetPulseUs(id, (uint16_t)value);
    }
    if ((id == ARM_SERVO_TABLE) && (value >= 1.0f) && (value <= 3.0f) && (floorf(value) == value))
    {
        if (strcmp(args[2], "go") == 0) {
            HAL_StatusTypeDef result=ArmServo_TableGo((uint8_t)value);
            char line[140];
            (void)snprintf(line,sizeof(line),"[SERVO] table slot=%u target=%uus enabled=%u ramping=%u HAL=%u\r\n",
                (unsigned int)value,s_state[1].target_us,s_state[1].enabled,s_state[1].ramping,(unsigned int)result);
            ZDT_Console_Print(line);return result;
        }
        if (strcmp(args[2], "save") == 0) { return ArmServo_TableCapture((uint8_t)value); }
    }
    return HAL_ERROR;
}
