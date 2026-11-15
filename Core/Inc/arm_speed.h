#ifndef ARM_SPEED_H
#define ARM_SPEED_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
/* Per-target rates share the existing ArmStartPose task/storage owner. */
enum {ARM_SPEED_NONE,ARM_SPEED_WAIT,ARM_SPEED_GRAB,ARM_SPEED_TRAY,ARM_SPEED_TRAY_PICK,
      ARM_SPEED_RING1,ARM_SPEED_RING2,ARM_SPEED_RING3,ARM_SPEED_POINT_COUNT};
enum {ARM_SPEED_LIFT,ARM_SPEED_REACH,ARM_SPEED_DM,ARM_SPEED_GRIP,ARM_SPEED_TABLE,ARM_SPEED_AXIS_COUNT};
void ArmSpeed_Init(void);
uint8_t ArmSpeed_ActivePoint(void);
float ArmSpeed_Factor(uint8_t axis);
float ArmSpeed_PointFactor(uint8_t point,uint8_t axis);
float ArmSpeed_MaxFactor(uint8_t axis);
uint16_t ArmSpeed_Rpm(uint8_t axis,uint16_t base);
uint16_t ArmSpeed_AllowedRpm(uint8_t axis,uint16_t base);
uint32_t ArmSpeed_Timeout(uint32_t base_ms);
HAL_StatusTypeDef ArmSpeed_Load(void);
uint8_t ArmSpeed_Command(const char *line);
#endif
