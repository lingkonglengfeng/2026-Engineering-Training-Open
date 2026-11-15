#include "arm_round1.h"
#include "arm_speed.h"
#include "arm_material.h"
#include "arm_start_pose.h"
#include "arm_calibration.h"
#include "arm_driver.h"
#include "arm_servo.h"
#include "chassis_motor.h"
#include "vision.h"
#include "zdt_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "screen_metrics.h"

enum {R1_IDLE,R1_RAW,R1_RAISE,R1_TABLE,R1_TRAY_PICK,R1_RING_DROP,R1_RING_PICK,R1_TRAY_PLACE,R1_CAMERA,R1_DONE,R1_FAILED};
static volatile uint8_t s_phase;
static uint8_t s_station,s_slot,s_returning,s_tray_mask,s_rough_mask,s_temp_mask,s_held,s_failed_phase;
static uint32_t s_tick;
static arm_feedback_wait_t s_feedback_pause;
static float s_raise_start_percent;
static uint8_t s_round,s_color=ARM_MATERIAL_RED,s_temp2_mask;
static vision_qr_task_t s_task;
static uint8_t s_task_loaded;
static uint8_t s_preload_slot;
static uint8_t s_preload_wait_top;
static uint16_t s_preload_target;
static uint32_t s_preload_tick;
static uint8_t s_tray_home_pending;
static uint16_t s_tray_home_from;
static const char *s_reason="idle";
uint8_t ArmRound1_IsActive(void){return s_phase>=R1_RAW && s_phase<=R1_CAMERA;}
void ArmRound1_Reset(void)
{memset(&s_feedback_pause,0,sizeof(s_feedback_pause));ScreenMetrics_ResetCounts();s_phase=R1_IDLE;s_station=s_slot=s_returning=s_tray_mask=s_rough_mask=s_temp_mask=s_held=s_failed_phase=0U;s_preload_slot=s_preload_wait_top=s_round=s_temp2_mask=s_tray_home_pending=0U;s_color=ARM_MATERIAL_RED;s_task_loaded=0U;memset(&s_task,0,sizeof(s_task));s_reason="empty two-round inventory";}
void ArmRound1_Init(void){ArmRound1_Reset();}
void ArmRound1_AbortState(void)
{
    s_tray_home_pending=0U;
    if(ArmRound1_IsActive()) {
        s_failed_phase=s_phase;
        s_reason=(s_phase==R1_CAMERA && ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED)?
            ArmCalibration_GetCameraFailureReason():"inner arm action cancelled; see prior fault";
        s_preload_wait_top=0U;s_phase=R1_FAILED;(void)Vision_RequestStop(1U);
    }
}
uint8_t ArmRound1_ReadResult(uint8_t *success)
{if(!success || (s_phase!=R1_DONE && s_phase!=R1_FAILED))return 0U;*success=(uint8_t)(s_phase==R1_DONE);return 1U;}
static void Round_Fail(const char *reason)
{
    char out[160];uint8_t phase=s_phase;
    s_failed_phase=phase;s_phase=R1_FAILED;s_preload_wait_top=0U;s_reason=reason;
    (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();(void)Vision_RequestStop(1U);
    (void)snprintf(out,sizeof(out),"[ROUND] round=%u FAILED station=%u phase=%u slot=%u: %s\r\n",s_round,s_station,phase,s_slot,reason);ZDT_Console_Print(out);
}
static void Round_Log(const char *action)
{
    char out[224];(void)snprintf(out,sizeof(out),"[ROUND] round=%u color=%u station=%u slot=%u %s; tray=0x%X rough=0x%X temp1=0x%X temp2=0x%X held=%u\r\n",
        s_round,s_color,s_station,s_slot,action,s_tray_mask,s_rough_mask,s_temp_mask,s_temp2_mask,s_held);ZDT_Console_Print(out);
}
static uint8_t Round_Color(uint8_t slot)
{return s_task_loaded?s_task.colors[s_round-1U][slot-1U]:(s_round==2U?ARM_MATERIAL_BLUE:ARM_MATERIAL_RED);}
static uint8_t Round_Ring(uint8_t slot)
{
    uint8_t i;
    if(!s_task_loaded)return slot;
    if(s_station==ARM_ROUND1_TEMP && s_round==2U) {
        for(i=0U;i<3U;i++)if(s_task.colors[0][i]==s_task.colors[1][slot-1U])return s_task.positions[0][i];
        return 0U;
    }
    return s_task.positions[s_round-1U][slot-1U];
}
static void Round_Select(void)
{
    s_color=Round_Color(s_slot);
    arm_servo_status_t table;ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
    if(s_preload_slot==s_slot && table.enabled && !table.fault && table.target_us==s_preload_target) {
        s_phase=R1_TABLE;s_tick=s_preload_tick;s_preload_slot=0U;
        Round_Log("next tray slot preselected during arm motion; reuse original settle time");return;
    }
    s_preload_slot=0U;
    if(ArmServo_SelectMaterialSlotForPoint(s_slot,s_returning?ARM_SPEED_TRAY:ARM_SPEED_TRAY_PICK)!=HAL_OK){Round_Fail("tray slot servo rejected");return;}
    s_phase=R1_TABLE;s_tick=HAL_GetTick();Round_Log("select measured tray slot");
}
static void Round_PreloadSlot(uint8_t target)
{
    arm_servo_status_t table;char out[140];
    if(target<1U || target>3U)return;
    if(ArmServo_SelectMaterialSlotForPoint(target,s_returning?ARM_SPEED_TRAY:ARM_SPEED_TRAY_PICK)!=HAL_OK){Round_Fail("tray slot preload rejected");return;}
    ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
    s_preload_slot=target;s_preload_target=table.target_us;s_preload_tick=HAL_GetTick();
    (void)snprintf(out,sizeof(out),"[ROUND] TOP and DM away confirmed; preselect tray slot%u during transfer from slot%u\r\n",
        s_preload_slot,s_slot);ZDT_Console_Print(out);
}
static void Round_Camera(void)
{
    if(ArmCalibration_DeployCamera()!=HAL_OK){Round_Fail("travel CAMERA pose rejected");return;}
    {arm_servo_status_t table;ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
     s_tray_home_from=table.target_us;s_tray_home_pending=1U;}
    if(s_station==ARM_ROUND1_RAW) {
        s_phase=R1_DONE;s_tick=HAL_GetTick();s_reason="RAW tray3 released; CAMERA restores during drive to ROUGH";
        Round_Log("STATION_DONE RAW; tray3 release confirmed, drive NOW; CAMERA lift/restore concurrent, no TOP wait");
        return;
    }
    s_phase=R1_CAMERA;s_tick=HAL_GetTick();Round_Log("all items released/stored; CAMERA raise first, then restore during next transport");
}
static void Round_TrayHomePoll(void)
{
    arm_servo_status_t table;arm_cal_axis_status_t lift;float percent;
    arm_camera_state_t camera;HAL_StatusTypeDef result;char line[144];
    if(!s_tray_home_pending)return;
    camera=ArmCalibration_GetCameraState();ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
    /* A cancelled CAMERA or a newer tray command owns the output now. */
    if(camera==ARM_CAMERA_IDLE || camera==ARM_CAMERA_FAILED || table.target_us!=s_tray_home_from) {
        s_tray_home_pending=0U;return;
    }
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)!=HAL_OK || !isfinite(percent) ||
       percent>ArmCalibration_LiftTolerancePercent(0.0f,lift.travel_mm))return;
    result=ArmServo_SelectMaterialSlotForPoint(2U,ARM_SPEED_TRAY);
    s_tray_home_pending=0U;
    (void)snprintf(line,sizeof(line),"[ROUND TRAY HOME] station=%u slot2 HAL=%u; TOP clear, PWM concurrent; no route wait\r\n",
        s_station,(unsigned)result);ZDT_Console_Print(line);
}
static void Round_RingPick(void)
{
    s_color=Round_Color(s_slot);
    uint8_t ring=Round_Ring(s_slot);
    if(!ring || !(s_rough_mask&(1U<<(ring-1U))) || ArmStartPose_BeginRingPick(ring)!=HAL_OK) {
        Round_Fail("ring pickup rejected/missing placed item");return;
    }
    s_phase=R1_RING_PICK;s_tick=HAL_GetTick();s_preload_wait_top=s_slot;
    Round_Log("pickup ring; preselect SAME tray slot after arm clears tray");
}
HAL_StatusTypeDef ArmRound1_Start(uint8_t task)
{
    uint8_t station,round;
    if(task<1U || task>6U)return HAL_ERROR;
    station=(uint8_t)((task-1U)%3U+1U);round=task>3U?2U:1U;
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(ArmRound1_IsActive() || ArmMaterial_IsActive() || ArmStartPose_IsActive() ||
       ArmCalibration_GetState()!=ARM_CAL_IDLE || ChassisMotor_HasMotionDemand() ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    if(s_held || (round==1U && (s_temp_mask || s_temp2_mask)) ||
       (round==2U && (s_temp_mask!=7U || s_temp2_mask)) ||
       (station==ARM_ROUND1_RAW && (s_tray_mask || s_rough_mask)) ||
       (station!=ARM_ROUND1_RAW && (s_tray_mask!=7U || s_rough_mask))) {
        ZDT_Console_Print("[ROUND] inventory mismatch; arm round1 status; reset only after physically reloading/emptying\r\n");return HAL_ERROR;
    }
    if(round==1U && station==ARM_ROUND1_RAW) {
        s_task_loaded=ZDT_Match_GetQrTask(&s_task);
        if(s_task_loaded && !VisionQr_CompetitionTaskValid(&s_task)) {
            ZDT_Console_Print("[ROUND] invalid QR task: require SAME three colors and ring permutations\r\n");return HAL_ERROR;
        }
    }
    s_round=round;
    s_tray_home_pending=0U; /* A new station's normal slot selection takes priority. */
    s_station=station;s_slot=(station==ARM_ROUND1_RAW || COMPETITION_PRELIMINARY)?1U:3U;s_color=Round_Color(s_slot);
    s_returning=s_failed_phase=0U;s_preload_slot=s_preload_wait_top=0U;s_reason="station automatic task";s_tick=HAL_GetTick();
    if(station==ARM_ROUND1_RAW) {
        if(ArmMaterial_StartToSlotColor(1U,s_color)!=HAL_OK){Round_Fail("color pickup start rejected");return HAL_ERROR;}
        s_phase=R1_RAW;Round_Log("collect three NEW color observations into tray1/2/3");
    } else {
        (void)Vision_RequestStop(1U);Round_Select();
    }
    return s_phase==R1_FAILED?HAL_ERROR:HAL_OK;
}
void ArmRound1_Poll(void)
{
    uint8_t success=0U,bit,ring,ring_bit;float percent;uint32_t now=HAL_GetTick();arm_servo_status_t servo;
    Round_TrayHomePoll(); /* Runs after RAW reports DONE, alongside chassis/CAMERA. */
    if(!ArmRound1_IsActive()){memset(&s_feedback_pause,0,sizeof(s_feedback_pause));return;}
    if(ChassisMotor_HasMotionDemand()){Round_Fail("chassis moved during station task");return;}
    {uint32_t gap;uint8_t waiting=ArmCalibration_WaitForFeedback(&s_feedback_pause,now,&gap);
     s_tick+=gap;if(waiting)return;}
    /* Inner material waiting deliberately has no blind visual timeout. */
    if(s_phase!=R1_RAW && now-s_tick>ArmSpeed_Timeout(120000U)){Round_Fail("station mechanical action timeout");return;}
    bit=(uint8_t)(1U<<(s_slot-1U));
    ring=Round_Ring(s_slot);ring_bit=(uint8_t)(1U<<(ring-1U));
    if(s_phase==R1_RAW) {
        if(!ArmMaterial_ReadResult(&success))return;
        if(!success){Round_Fail("color material pickup/place failed");return;}
        s_tray_mask|=bit;Round_Log("requested color stored in selected tray slot");
        if(s_slot<3U) {
            ++s_slot;
            s_color=Round_Color(s_slot);
            if(ArmMaterial_StartToSlotColor(s_slot,s_color)!=HAL_OK){Round_Fail("next color pickup start rejected");return;}
            s_tick=now;return;
        }
        Round_Camera();return;
    }
    if(s_phase==R1_RAISE) {
        arm_cal_axis_status_t lift;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
        if(ArmCalibration_GetState()==ARM_CAL_FAILED){Round_Fail("post-ring-pick raise failed");return;}
        if(ArmCalibration_IsLiftRaisingToTop()) {
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)!=HAL_OK || !isfinite(percent)){Round_Fail("ring rise feedback unavailable");return;}
            Round_Select();return;
        }
        if(ArmCalibration_GetState()!=ARM_CAL_IDLE)return;
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)!=HAL_OK || !isfinite(percent) || percent>ArmCalibration_LiftTolerancePercent(0.0f,lift.travel_mm)) {
            Round_Fail("post-ring-pick clearance not confirmed");return;
        }
        Round_Select();return;
    }
    if(s_phase==R1_TABLE) {
        ArmServo_GetStatus(ARM_SERVO_TABLE,&servo);
        if(!servo.enabled || servo.fault){Round_Fail("tray servo unavailable");return;}
        if(servo.ramping && now-s_tick>ArmSpeed_Timeout(5000U)){Round_Fail("tray servo timeout");return;}
        if(s_returning) {
            if(ArmCalibration_IsLiftRaisingToTop()) {
                if(ArmStartPose_BeginTrayPlaceDuringRaise(s_slot,s_raise_start_percent)!=HAL_OK){Round_Fail("tray return raise adoption rejected");return;}
            } else {
                char command[24];(void)snprintf(command,sizeof(command),"arm place %u",s_slot);
                (void)ArmStartPose_Command(command);
            }
        } else (void)ArmStartPose_Command("arm pick");
        if(!ArmStartPose_IsActive()){Round_Fail("tray PICK/PLACE rejected");return;}
        if(ArmStartPose_WaitForTray(servo.target_us,s_tick)!=HAL_OK){Round_Fail("tray descent gate rejected");return;}
        s_phase=s_returning?R1_TRAY_PLACE:R1_TRAY_PICK;s_tick=now;
        Round_Log(s_returning?"return ring item to SAME tray slot":"pick selected tray slot for task-assigned ring");return;
    }
    if(s_phase==R1_TRAY_PICK || s_phase==R1_RING_DROP || s_phase==R1_RING_PICK || s_phase==R1_TRAY_PLACE) {
        uint8_t ready=ArmStartPose_ReadActionResult(&success);
        if(ready && !success){Round_Fail("three-axis pick/place failed");return;}
        if((s_phase==R1_RING_DROP || s_phase==R1_RING_PICK) && s_preload_wait_top && ArmStartPose_IsAwayFromTray()) {
            uint8_t target=s_preload_wait_top;s_preload_wait_top=0U;Round_PreloadSlot(target);
            if(s_phase==R1_FAILED)return;
        }
        if(!ready)return;
        if(s_phase==R1_TRAY_PICK) {
            if(!ArmServo_GripIsClosed()){Round_Fail("tray PICK closed PWM not confirmed");return;}
            s_tray_mask&=(uint8_t)~bit;s_held=s_slot;
            float rise=(s_round==2U && s_station==ARM_ROUND1_TEMP)?72.0f:0.0f;
            HAL_StatusTypeDef drop=(s_station==ARM_ROUND1_TEMP)?
                ArmStartPose_BeginTempRingDrop(ring,rise):ArmStartPose_BeginRingDrop(ring,rise);
            if(drop!=HAL_OK){Round_Fail("ring placement rejected");return;}
            {char line[128];(void)snprintf(line,sizeof(line),"[ROUND PLAN] round=%u station=%u tray=%u color=%u -> ring=%u rise=%.1fmm\r\n",s_round,s_station,s_slot,s_color,ring,(double)rise);ZDT_Console_Print(line);}
            s_phase=R1_RING_DROP;s_tick=now;
            s_preload_wait_top=COMPETITION_PRELIMINARY?(s_slot<3U?(uint8_t)(s_slot+1U):0U):(s_slot>1U?(uint8_t)(s_slot-1U):0U);
            Round_Log("place into task ring; next tray waits TOP and DM away");return;
        }
        if(s_phase==R1_RING_DROP) {
            if(ArmServo_GripIsClosed()){Round_Fail("ring release not confirmed");return;}
            s_held=0U;
            if(s_station==ARM_ROUND1_ROUGH)s_rough_mask|=ring_bit;else if(s_round==2U)s_temp2_mask|=ring_bit;else s_temp_mask|=ring_bit;
            Round_Log("ring release complete");
            if(COMPETITION_PRELIMINARY?s_slot<3U:s_slot>1U){s_slot=(uint8_t)(COMPETITION_PRELIMINARY?s_slot+1U:s_slot-1U);Round_Select();return;}
            if(s_station==ARM_ROUND1_TEMP){Round_Camera();return;}
            s_returning=1U;s_slot=1U;Round_RingPick();return;
        }
        if(s_phase==R1_RING_PICK) {
            if(!ArmServo_GripIsClosed()){Round_Fail("ring PICK closed PWM not confirmed");return;}
            s_rough_mask&=(uint8_t)~ring_bit;s_held=s_slot;
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&s_raise_start_percent)!=HAL_OK){Round_Fail("ring raise start unavailable");return;}
            {
                arm_cal_axis_status_t lift;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
                if(ArmCalibration_GetState()==ARM_CAL_IDLE && ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK &&
                   isfinite(percent) && percent<=ArmCalibration_LiftTolerancePercent(0.0f,lift.travel_mm)) {
                    Round_Log("ring item closed/TOP ready; skip duplicate lift and select tray");Round_Select();return;
                }
                if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)!=HAL_OK){Round_Fail("raise ring item before tray rejected");return;}
            }
            s_phase=R1_RAISE;s_tick=now;Round_Log("ring item fully closed; raise true TOP and start reach together, DM keeps actual half-rise");return;
        }
        if(ArmServo_GripIsClosed()){Round_Fail("tray return release not confirmed");return;}
        s_tray_mask|=bit;s_held=0U;Round_Log("item returned to SAME tray slot");
        if(s_slot<3U){++s_slot;Round_RingPick();return;}
        Round_Camera();return;
    }
    if(s_phase==R1_CAMERA) {
        arm_camera_state_t state=ArmCalibration_GetCameraState();
        if(state==ARM_CAMERA_FAILED || state==ARM_CAMERA_IDLE){Round_Fail("travel CAMERA restore failed");return;}
        if(state==ARM_CAMERA_DM_HOME || state==ARM_CAMERA_REACH_ENABLE || state==ARM_CAMERA_REACH_MOVE ||
           state==ARM_CAMERA_LIFT_FINAL || state==ARM_CAMERA_DONE) {
            arm_cal_axis_status_t lift;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)!=HAL_OK || !isfinite(percent) ||
               percent>ArmCalibration_LiftTolerancePercent(0.0f,lift.travel_mm))return;
            s_phase=R1_DONE;s_reason="station complete; result ready for route owner";
            Round_Log(state==ARM_CAMERA_DONE?"STATION_DONE; CAMERA ready":"STATION_DONE; lift TOP confirmed; CAMERA continues alongside chassis");
        }
    }
}
uint8_t ArmRound1_Command(const char *line)
{
    if(strcmp(line,"arm round1 status")==0) {
        char out[224];(void)snprintf(out,sizeof(out),"[ROUND] round=%u color=%u station=%u phase=%u active=%u slot=%u return=%u tray=0x%X rough=0x%X temp1=0x%X temp2=0x%X held=%u failed_phase=%u %.64s\r\n",
            s_round,s_color,s_station,s_phase,ArmRound1_IsActive(),s_slot,s_returning,s_tray_mask,s_rough_mask,s_temp_mask,s_temp2_mask,s_held,s_failed_phase,s_reason);ZDT_Console_Print(out);return 1U;
    }
    return 0U;
}
