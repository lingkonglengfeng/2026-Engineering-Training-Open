#ifndef ARM_SERVO_H
#define ARM_SERVO_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ARM_SERVO_GRIP = 0, ARM_SERVO_TABLE = 1 } arm_servo_id_t;
#define ARM_SERVO_TABLE_SLOTS (3U)
#define ARM_SERVO_TABLE_BOOT_SLOT (2U) /* Slot2=109deg. */
#define ARM_SERVO_TABLE_SLOT1_DEG (19.0f)
#define ARM_SERVO_TABLE_SLOT2_DEG (109.0f)
#define ARM_SERVO_TABLE_SLOT3_DEG (199.0f)
#define ARM_SERVO_GRIP_SPAN_DEG (360.0f)
#define ARM_SERVO_GRIP_OPEN_DEG (225.0f)
#define ARM_SERVO_GRIP_CLOSE_DEG (308.0f)
#define ARM_SERVO_GRIP_CLOSE_MIN_MS (231U) /* ceil(300/1.3). */
#define ARM_SERVO_GRIP_OPEN_MIN_MS (50U) /* OPEN command age before next action; PWM may still ramp. */
#define ARM_SERVO_TABLE_MIN_WAIT_MS (411U) /* ceil(534/1.3). */
typedef struct
{
    uint16_t min_us;
    uint16_t max_us;
    uint16_t enable_us; /* First explicitly enabled pulse; may move the servo */
    float angle_span_deg; /* 0 = angle unknown: pulse interface only */
    float slew_us_per_second;
    uint8_t reverse_angle;
} arm_servo_channel_config_t;
typedef struct
{
    uint32_t timer_clock_hz; /* Current board TIM4 clock: 275 MHz */
    arm_servo_channel_config_t grip; /* PD12/TIM4_CH1 */
    arm_servo_channel_config_t table; /* PD13/TIM4_CH2 */
    uint16_t table_slot_us[ARM_SERVO_TABLE_SLOTS];
    uint8_t table_slots_valid;
} arm_servo_config_t;
typedef struct
{
    float output_us;
    uint16_t target_us;
    uint8_t enabled;
    uint8_t ramping; /* Output ramp only, not physical feedback/arrival */
    uint8_t fault;
} arm_servo_status_t;

/* Exclusive TIM4 owner. Do not call the old Servo_* driver at the same time.
 * Call Init after MX_TIM4_Init. Configures 1us ticks/20ms period but starts
 * neither channel. Table defaults use the 360-degree mapping,
 * slots 19/109/199 degrees and slot2 first-enable position. Grip reproduces the
 * V181 competition reference: 360-degree scale, open225/close308.
 * All APIs except RequestSafeStop require one owner task; no allocation/delay.
 */
void ArmServo_DefaultConfig(arm_servo_config_t *out);
HAL_StatusTypeDef ArmServo_Init(const arm_servo_config_t *config);
HAL_StatusTypeDef ArmServo_Enable(arm_servo_id_t id);
HAL_StatusTypeDef ArmServo_Disable(arm_servo_id_t id);
HAL_StatusTypeDef ArmServo_SetPulseUs(arm_servo_id_t id, uint16_t pulse_us);
HAL_StatusTypeDef ArmServo_SetAngleDeg(arm_servo_id_t id, float angle_deg);
/* Explicit manual test: validate first, then enable at the requested pulse
 * when OFF; when ON use the normal nonblocking ramp. No initial centre jump. */
HAL_StatusTypeDef ArmServo_TestAngleDeg(arm_servo_id_t id,float angle_deg);
/* TABLE test alone permits 360..385deg extrapolation to 2500..2639us;
 * requires the default non-reversed 360deg mapping. Normal APIs stay bounded. */
HAL_StatusTypeDef ArmServo_SetAngleSpan(arm_servo_id_t id, float span_deg);
HAL_StatusTypeDef ArmServo_SetSlew(arm_servo_id_t id, float us_per_second);
HAL_StatusTypeDef ArmServo_GripCapture(uint8_t closed); /* Save stationary OUTPUT, not sensor feedback */
HAL_StatusTypeDef ArmServo_GripOpen(void);
HAL_StatusTypeDef ArmServo_GripClose(void);
/* One close at a fraction of configured slew; following normal commands reset it. */
HAL_StatusTypeDef ArmServo_GripCloseScaled(float scale);
HAL_StatusTypeDef ArmServo_TableCapture(uint8_t slot); /* slots 1..3 */
HAL_StatusTypeDef ArmServo_TableGo(uint8_t slot);
/* Measured material slots used by automatic batches; old manual presets remain. */
float ArmServo_MaterialSlotDeg(uint8_t slot);
float ArmServo_GripOpenDeg(void);
float ArmServo_GripCloseDeg(void);
/* open, close, material slot1/2/3. Owner only; setters never send PWM. */
HAL_StatusTypeDef ArmServo_GetUiParameters(float out[5]);
HAL_StatusTypeDef ArmServo_SetUiParameters(const float values[5]);
HAL_StatusTypeDef ArmServo_SelectMaterialSlot(uint8_t slot);
HAL_StatusTypeDef ArmServo_SelectMaterialSlotForPoint(uint8_t slot,uint8_t speed_point);
uint8_t ArmServo_GripIsClosed(void); /* Stable PWM at recorded closed preset, not physical feedback */
uint8_t ArmServo_GripIsOpen(void); /* Stable open PWM; no extra500ms dwell/object sensor. */
/* Configured OPEN command active for>=ARM_SERVO_GRIP_OPEN_MIN_MS; the PWM
 * ramp may continue alongside the next action. No physical servo feedback. */
uint8_t ArmServo_GripOpenConfirmed(void);
/* OPEN command is active, including an unfinished PWM ramp. */
uint8_t ArmServo_GripOpeningOrOpen(void);
HAL_StatusTypeDef ArmServo_Stop(arm_servo_id_t id); /* Freeze PWM at current output, retain holding */
HAL_StatusTypeDef ArmServo_StopAll(void); /* OK also when never initialized */
void ArmServo_Update(uint32_t now_ms); /* 10ms recommended; dt capped at 50ms */
void ArmServo_GetStatus(arm_servo_id_t id, arm_servo_status_t *out);
void ArmServo_RequestSafeStop(void); /* ISR-safe; next Update disables both PWM */
HAL_StatusTypeDef ArmServo_ClearSafeStop(void); /* Leaves PWM disabled; explicit Enable required */
/* Structured tokens from ArmCalibration_ProcessCommand, e.g. arm table go 1. */
HAL_StatusTypeDef ArmServo_Command(uint8_t count, char *const *args);
void ArmServo_PrintStatus(void);
#ifdef __cplusplus
}
#endif
#endif
