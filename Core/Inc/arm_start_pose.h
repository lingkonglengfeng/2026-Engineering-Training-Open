#ifndef ARM_START_POSE_H
#define ARM_START_POSE_H
#include <stdint.h>
#include "competition_profile.h"
#include "stm32h7xx_hal.h"
#define ARM_START_DEFAULT_LIFT_DEG  (-0.7526f)
#define ARM_START_DEFAULT_REACH_DEG (-186.8170f)
#define ARM_START_DEFAULT_DM_DEG    (134.6173f)
enum { ARM_DISC_WAIT, ARM_DISC_GRAB, ARM_DISC_TRAY, ARM_DISC_TRAY_PICK, ARM_DISC_POSE_COUNT };
/* Owner-task final-gripper profiles: calibrated degrees, independent of START. */
HAL_StatusTypeDef ArmStartPose_DiscTargets(uint8_t kind,float base[3],float target[3]);
void ArmStartPose_Init(void);
void ArmStartPose_Poll(void);
uint8_t ArmStartPose_Command(const char *line);
void ArmStartPose_PrintUiPoints(void); /* Current calibration, baseline + RAM trims. */
HAL_StatusTypeDef ArmStartPose_LoadParameters(void); /* Read only; BUSY means retry boot load. */
const char *ArmStartPose_BootParameterSource(void);
uint8_t ArmStartPose_IsActive(void);
/* Automatic transfer actions skip redundant arrival dwell, not live feedback. */
uint8_t ArmStartPose_IsFastTransfer(void);
/* PLACE deliberately releases DM while raising to rotation clearance. */
uint8_t ArmStartPose_IsRotationReleaseExpected(void);
/* Owner-task handoff: PLACE open; ring release permits rise after200ms
 * while its OPEN PWM ramp continues. */
uint8_t ArmStartPose_IsReleasedPlace(void);
uint8_t ArmStartPose_IsReleaseOpening(void);
void ArmStartPose_ReleaseForCamera(void);
void ArmStartPose_Cancel(void);
HAL_StatusTypeDef ArmStartPose_BeginRestore(void);
uint8_t ArmStartPose_IsReady(void);
uint8_t ArmStartPose_ReadResult(uint8_t *success);
/* Owner-task transfer without replacing START or automatic grip action. */
HAL_StatusTypeDef ArmStartPose_BeginMove(float lift_deg,float reach_deg,float dm_deg);
/* In-window material approach: direct DM/reach/lift, no clearance waypoint. */
HAL_StatusTypeDef ArmStartPose_BeginMaterialApproach(float lift_deg,float reach_deg,float dm_deg);
/* Owner task adopts an existing raise to true TOP after confirmed grasp.
 * Ring return: reach starts alongside the adopted raise; DM keeps half-rise.
 * Other sources retain half-rise. No new lift or grip command. */
HAL_StatusTypeDef ArmStartPose_BeginTrayPlaceDuringRaise(uint8_t slot,float raise_start_percent);
/* Retrieve using the shared ground PLACE target, including saved three-axis trim. */
HAL_StatusTypeDef ArmStartPose_BeginRingPick(uint8_t station);
/* Ground ring placement plus a one-action vertical rise (second TEMP layer). */
HAL_StatusTypeDef ArmStartPose_BeginRingDrop(uint8_t station,float rise_mm);
/* TEMP/ROUGH share placement and trim; second-layer rise stays separate. */
HAL_StatusTypeDef ArmStartPose_BeginTempRingDrop(uint8_t station,float rise_mm);
uint8_t ArmStartPose_ReadActionResult(uint8_t *success);
/* PICK/PLACE result is frozen after validated grip/hold completion. START
 * readiness remains a live pose check via IsReady/ReadResult. */
/* Owner-only: true after TOP and actual DM arrival in a transfer direction
 * away from the tray. Used to overlap tray selection without crossing it. */
uint8_t ArmStartPose_IsAwayFromTray(void);
/* Allow approach in parallel; defer final tray descent until this PWM target
 * and its original settling interval are complete. */
HAL_StatusTypeDef ArmStartPose_WaitForTray(uint16_t target_us,uint32_t selected_tick);
#endif
