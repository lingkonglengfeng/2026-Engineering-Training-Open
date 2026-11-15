#ifndef ARM_ROUND1_H
#define ARM_ROUND1_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#define ARM_ROUND1_RAW   (1U)
#define ARM_ROUND1_ROUGH (2U)
#define ARM_ROUND1_TEMP  (3U)
#define ARM_ROUND2_RAW   (4U)
#define ARM_ROUND2_ROUGH (5U)
#define ARM_ROUND2_TEMP  (6U)
/* Single ArmTask owner; IsActive is the chassis task's read-only snapshot.
 * RAW loads slots1/2/3. ROUGH/TEMP unload PRELIM1/2/3 or FINAL3/2/1 into assigned rings.
 * ROUGH retrieval returns items to their original slots in slot1/2/3 order.
 * TEMP only unloads. Success confirms lift TOP; remaining CAMERA restoration
 * is adopted by ArmTask under the SAME token, concurrent with chassis travel. */
void ArmRound1_Init(void);
void ArmRound1_Reset(void);
HAL_StatusTypeDef ArmRound1_Start(uint8_t task); /* 1..3 first round,4..6 second round. */
void ArmRound1_Poll(void);
uint8_t ArmRound1_IsActive(void);
uint8_t ArmRound1_ReadResult(uint8_t *success);
void ArmRound1_AbortState(void);
uint8_t ArmRound1_Command(const char *line);
#endif
