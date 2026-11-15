#ifndef ARM_MATERIAL_H
#define ARM_MATERIAL_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#define ARM_MATERIAL_RED (1U)
#define ARM_MATERIAL_BLUE (3U)
#define ARM_MATERIAL_GRAB_LIFT_DEG (-612.7679f)
#define ARM_MATERIAL_GRAB_RAISE_MM (10.0f)
#define ARM_MATERIAL_WAIT_REACH_DEG (-185.1581f)
#define ARM_MATERIAL_GRAB_EXTEND_MM (25.0f)
#define ARM_MATERIAL_DM_DEG (-2.0217f)
/* ArmTask owner only, except the read-only automatic-enable snapshot. */
void ArmMaterial_Init(void);
void ArmMaterial_Poll(void);
uint8_t ArmMaterial_Command(const char *line);
HAL_StatusTypeDef ArmMaterial_Start(void);
/* One RED into slot1..3; station owner restores CAMERA later. */
HAL_StatusTypeDef ArmMaterial_StartToSlot(uint8_t slot);
HAL_StatusTypeDef ArmMaterial_StartToSlotColor(uint8_t slot,uint8_t color);
uint8_t ArmMaterial_IsActive(void);
/* Owner-only read: enables the plate-grab reach profile, not other jobs. */
uint8_t ArmMaterial_GrabReachActive(void);
uint8_t ArmMaterial_ReadResult(uint8_t *success);
uint8_t ArmMaterial_AutoEnabled(void);
/* State-only cancellation hook from the existing arm stop owner. */
void ArmMaterial_AbortState(void);
#endif
