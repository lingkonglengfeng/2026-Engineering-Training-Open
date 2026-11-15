#ifndef TEST_ARM_SPEED_DEFAULTS_H
#define TEST_ARM_SPEED_DEFAULTS_H
/* Default100% ports for isolated legacy driver/FSM fixtures. Storage and
 * per-point selection are exercised against production code in pose tests. */
#include "arm_speed.h"
static float test_arm_rate_factor[ARM_SPEED_AXIS_COUNT]={1,1,1,1,1};
void ArmSpeed_Init(void){}
uint8_t ArmSpeed_ActivePoint(void){return 0;}
float ArmSpeed_Factor(uint8_t axis){return axis<ARM_SPEED_AXIS_COUNT?test_arm_rate_factor[axis]:1.0f;}
float ArmSpeed_PointFactor(uint8_t p,uint8_t a){(void)p;return ArmSpeed_Factor(a);}
float ArmSpeed_MaxFactor(uint8_t axis){float f=ArmSpeed_Factor(axis);return f>1.0f?f:1.0f;}
uint16_t ArmSpeed_Rpm(uint8_t axis,uint16_t value){return (uint16_t)(value*ArmSpeed_Factor(axis)+.5f);}
uint16_t ArmSpeed_AllowedRpm(uint8_t axis,uint16_t value){return (uint16_t)(value*ArmSpeed_MaxFactor(axis)+.5f);}
uint32_t ArmSpeed_Timeout(uint32_t value){return value;}
HAL_StatusTypeDef ArmSpeed_Load(void){return HAL_ERROR;}
uint8_t ArmSpeed_Command(const char *s){(void)s;return 0;}
#endif
