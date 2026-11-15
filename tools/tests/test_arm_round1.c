/* Production station FSM with asynchronous inner jobs and actuator ports. */
#include "arm_round1.h"
#include "arm_speed_defaults.h"
#include "arm_material.h"
#include "arm_start_pose.h"
#include "arm_calibration.h"
#include "arm_driver.h"
#include "arm_servo.h"
#include "vision_qr.h"
#include "qr_mission_vectors.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static uint32_t tick=1000,material_calls,table_calls,pick_calls,drop_calls,ring_pick_calls,place_calls,camera_calls,stops;
static uint32_t tray_home_calls;
static uint8_t test_home_phase(void);
static uint8_t material_active,material_success,pose_active,pose_success,closed,chassis,reject;
static uint8_t materials[6],material_colors[6],tables[24],drops[12],retrieved[6],placed_slots[6];
static float drop_rise[12];
static uint8_t drop_temp[12];
static arm_cal_state_t cal;
static vision_qr_task_t test_task;
static uint8_t task_seen;
uint8_t ZDT_Match_GetQrTask(vision_qr_task_t *task){if(!task || !task_seen)return 0;*task=test_task;return 1;}
#include "vision_qr.c"
static arm_camera_state_t camera;
static arm_servo_status_t table;
static float lift_pct;
static uint8_t pose_clear;
static uint32_t tray_gates;
uint8_t ArmStartPose_IsAwayFromTray(void){return pose_active && pose_clear;}
HAL_StatusTypeDef ArmStartPose_WaitForTray(uint16_t target,uint32_t started){CHECK(pose_active && target==table.target_us && started<=tick);tray_gates++;return HAL_OK;}
uint32_t HAL_GetTick(void){return tick;}
uint8_t ArmCalibration_WaitForFeedback(arm_feedback_wait_t *w,uint32_t now,uint32_t *gap){(void)w;(void)now;*gap=0;return 0;}
void ZDT_Console_Print(const char *s){CHECK(s!=NULL);}
uint8_t ChassisMotor_HasMotionDemand(void){return chassis;}
uint8_t ArmMaterial_IsActive(void){return material_active;}
HAL_StatusTypeDef ArmMaterial_StartToSlot(uint8_t slot){CHECK(slot>=1 && slot<=3);if(reject)return HAL_ERROR;materials[material_calls++]=slot;material_active=1;material_success=0;return HAL_OK;}
HAL_StatusTypeDef ArmMaterial_StartToSlotColor(uint8_t slot,uint8_t color){HAL_StatusTypeDef r;CHECK(color>=1 && color<=6);r=ArmMaterial_StartToSlot(slot);if(r==HAL_OK)material_colors[material_calls-1]=color;return r;}
uint8_t ArmMaterial_ReadResult(uint8_t *ok){if(material_active)return 0;*ok=material_success;return 1;}
uint8_t ArmStartPose_IsActive(void){return pose_active;}
uint8_t ArmStartPose_ReadActionResult(uint8_t *ok){if(pose_active)return 0;*ok=pose_success;return 1;}
uint8_t ArmStartPose_Command(const char *cmd){
 if(reject)return 1;
 camera=ARM_CAMERA_IDLE;
 if(!strcmp(cmd,"arm pick")){pick_calls++;CHECK(!closed);}
 else if(!strncmp(cmd,"arm place ",10)){unsigned slot=0;CHECK(sscanf(cmd+10,"%u",&slot)==1 && slot>=1 && slot<=3 && closed);placed_slots[place_calls++]=(uint8_t)slot;}
 else {unsigned slot=0;CHECK(sscanf(cmd,"arm ring %u",&slot)==1);CHECK(closed && slot>=1 && slot<=3);drops[drop_calls++]=(uint8_t)slot;}
 pose_active=1;pose_success=0;pose_clear=0;return 1;
}
HAL_StatusTypeDef ArmStartPose_BeginRingPick(uint8_t slot){CHECK(!closed && slot>=1 && slot<=3);if(reject)return HAL_ERROR;camera=ARM_CAMERA_IDLE;retrieved[ring_pick_calls++]=slot;pose_active=1;pose_success=0;pose_clear=0;return HAL_OK;}
HAL_StatusTypeDef ArmStartPose_BeginRingDrop(uint8_t slot,float rise){char cmd[24];CHECK(closed && slot>=1 && slot<=3 && (rise==0 || rise==72));if(reject)return HAL_ERROR;snprintf(cmd,sizeof(cmd),"arm ring %u",slot);ArmStartPose_Command(cmd);drop_rise[drop_calls-1]=rise;drop_temp[drop_calls-1]=0;return HAL_OK;}
HAL_StatusTypeDef ArmStartPose_BeginTempRingDrop(uint8_t slot,float rise){HAL_StatusTypeDef result=ArmStartPose_BeginRingDrop(slot,rise);if(result==HAL_OK)drop_temp[drop_calls-1]=1;return result;}
arm_cal_state_t ArmCalibration_GetState(void){return cal;}
uint8_t ArmCalibration_IsLiftRaisingToTop(void){return cal==ARM_CAL_PERCENT_MOVE || cal==ARM_CAL_PERCENT_SETTLE;}
HAL_StatusTypeDef ArmStartPose_BeginTrayPlaceDuringRaise(uint8_t slot,float from){char cmd[24];CHECK(from>0 && slot>=1 && slot<=3 && closed && ArmCalibration_IsLiftRaisingToTop());snprintf(cmd,sizeof(cmd),"arm place %u",slot);ArmStartPose_Command(cmd);return reject?HAL_ERROR:HAL_OK;}
arm_camera_state_t ArmCalibration_GetCameraState(void){return camera;}
const char *ArmCalibration_GetCameraFailureReason(void){return "mock CAMERA feedback failure";}
HAL_StatusTypeDef ArmCalibration_DeployCamera(void){if(reject)return HAL_ERROR;camera_calls++;camera=ARM_CAMERA_PRECHECK;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_MovePercent(arm_driver_axis_t axis,float pct){CHECK(axis==ARM_DRIVER_LIFT && pct==0 && closed);if(reject)return HAL_ERROR;cal=ARM_CAL_PERCENT_MOVE;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_GetPercent(arm_driver_axis_t axis,float *pct){CHECK(axis==ARM_DRIVER_LIFT);*pct=lift_pct;return HAL_OK;}
void ArmCalibration_GetAxisStatus(arm_driver_axis_t axis,arm_cal_axis_status_t *s){CHECK(axis==ARM_DRIVER_LIFT);memset(s,0,sizeof(*s));s->travel_mm=ARM_LIFT_FULL_TRAVEL_MM;}
HAL_StatusTypeDef ArmCalibration_Cancel(void){ArmRound1_AbortState();material_active=pose_active=0;cal=ARM_CAL_IDLE;return HAL_OK;}
HAL_StatusTypeDef ArmDriver_StopAll(void){stops++;return HAL_OK;}
HAL_StatusTypeDef Vision_RequestStop(uint8_t camera_id){CHECK(camera_id==1);return HAL_OK;}
HAL_StatusTypeDef ArmServo_SelectMaterialSlotForPoint(uint8_t slot,uint8_t point){CHECK(point==ARM_SPEED_TRAY || point==ARM_SPEED_TRAY_PICK);return ArmServo_SelectMaterialSlot(slot);}
HAL_StatusTypeDef ArmServo_SelectMaterialSlot(uint8_t slot){CHECK(slot>=1 && slot<=3);if(reject)return HAL_ERROR;if(test_home_phase()){CHECK(slot==2);tray_home_calls++;}else tables[table_calls++]=slot;table.enabled=1;table.ramping=1;table.target_us=(uint16_t)(1000U+slot);return HAL_OK;}
void ArmServo_GetStatus(arm_servo_id_t id,arm_servo_status_t *out){CHECK(id==ARM_SERVO_TABLE);*out=table;}
uint8_t ArmServo_GripIsClosed(void){return closed;}
#include "arm_round1.c"
static uint8_t test_home_phase(void){return s_tray_home_pending && (s_phase==R1_CAMERA || s_phase==R1_DONE);}
static void step(uint32_t ms){tick+=ms;if(camera==ARM_CAMERA_DONE)lift_pct=0;ArmRound1_Poll();}
static void setup(void){
 material_calls=table_calls=pick_calls=drop_calls=ring_pick_calls=place_calls=camera_calls=stops=tray_home_calls=0;
 material_active=pose_active=closed=chassis=reject=pose_clear=0;material_success=pose_success=1;tray_gates=0;
 cal=ARM_CAL_IDLE;camera=ARM_CAMERA_IDLE;lift_pct=0;task_seen=0;memset(&test_task,0,sizeof(test_task));memset(&table,0,sizeof(table));ArmRound1_Init();
}
static void action_done(uint8_t close){
 if(s_phase==R1_RING_DROP || s_phase==R1_RING_PICK){lift_pct=0;cal=ARM_CAL_IDLE;pose_clear=1;step(10);} /* TOP + DM away */
  if(s_phase==R1_TRAY_PICK && close)lift_pct=38.61f;
  if(s_phase==R1_RING_PICK && close)lift_pct=80.0f;
  cal=ARM_CAL_IDLE;
 pose_active=0;pose_success=1;closed=close;step(10);
}
static void table_done(void){table.ramping=0;step(810);}
static void raw_done(void){
 unsigned slot;uint8_t ok;
 CHECK(ArmRound1_Start(ARM_ROUND1_RAW)==HAL_OK);
 for(slot=1;slot<=3;slot++){
   CHECK(s_phase==R1_RAW && s_slot==slot && material_calls==slot && materials[slot-1]==slot);
   step(180000);CHECK(ArmRound1_IsActive() && !ArmRound1_ReadResult(&ok)); /* never blind timed grab */
   material_active=0;material_success=1;step(10);
 }
 CHECK(s_phase==R1_DONE && s_tray_mask==7 && camera_calls==1);
 /* RAW hands off immediately after third release, even below TOP.
  * ArmTask adopts the already running CAMERA for concurrent supervision. */
 camera=ARM_CAMERA_LIFT_ZERO;lift_pct=50;
 CHECK(ArmRound1_ReadResult(&ok) && ok && !ArmRound1_IsActive());
 CHECK(lift_pct==50 && camera==ARM_CAMERA_LIFT_ZERO);
 chassis=1;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok);chassis=0;
 camera=ARM_CAMERA_DONE;
}
#define TEST_TRAY_SLOT(n) (COMPETITION_PRELIMINARY?(n):4U-(n))
int main(void){
 unsigned slot;uint8_t ok;
 /* Every completed station schedules exactly one slot2 command. RAW can
  * drive while waiting for arm clearance; no result waits for TABLE PWM. */
 for(slot=1;slot<=3;slot++) {
  setup();s_station=slot;s_round=1;s_phase=R1_TRAY_PLACE;
  table.enabled=1;table.target_us=1003;lift_pct=50;Round_Camera();
  CHECK(s_tray_home_pending && !tray_home_calls);
  chassis=slot==ARM_ROUND1_RAW;step(10);CHECK(s_tray_home_pending && !tray_home_calls && !stops);
  if(chassis)CHECK(ArmRound1_ReadResult(&ok) && ok);
  camera=ARM_CAMERA_DM_HOME;lift_pct=0;step(10);
  CHECK(!s_tray_home_pending && tray_home_calls==1 && table.target_us==1002 && table.ramping);
  CHECK(ArmRound1_ReadResult(&ok) && ok && !stops);
  chassis=1;step(5000);CHECK(tray_home_calls==1 && ArmRound1_ReadResult(&ok) && ok && !stops);
 }
 /* Cancel, newer TABLE selection and PWM rejection never block the route
  * or resurrect a late slot2 request. */
 for(slot=0;slot<3;slot++) {
  setup();s_station=1;s_phase=R1_RAW;table.enabled=1;table.target_us=1003;lift_pct=50;Round_Camera();
  if(slot==0)ArmRound1_AbortState();
  else if(slot==1)table.target_us=1001;
  else reject=1;
  camera=ARM_CAMERA_DM_HOME;lift_pct=0;chassis=1;step(10);
  CHECK(!s_tray_home_pending && !tray_home_calls && !stops && ArmRound1_ReadResult(&ok) && ok);
 }
 setup();CHECK(ArmRound1_Start(0)==HAL_ERROR && ArmRound1_Start(7)==HAL_ERROR);
 CHECK(ArmRound1_Start(4)==HAL_ERROR); /* BLUE round cannot begin before red TEMP layer */
 CHECK(ArmRound1_Start(ARM_ROUND1_ROUGH)==HAL_ERROR && !table_calls);
 raw_done();CHECK(ArmRound1_Start(ARM_ROUND1_RAW)==HAL_ERROR); /* cannot overwrite loaded slots */
 CHECK(ArmRound1_Start(ARM_ROUND1_ROUGH)==HAL_OK);
 for(slot=1;slot<=3;slot++){
   CHECK(s_phase==R1_TABLE && tables[slot-1]==TEST_TRAY_SLOT(slot) && !closed);
   step(10);CHECK(s_phase==R1_TRAY_PICK && pick_calls==slot && tray_gates>=slot); /*approach parallels table*/
   table.ramping=0;
   step(100);CHECK(s_phase==R1_TRAY_PICK && !ArmRound1_ReadResult(&ok));
   action_done(1);CHECK(s_phase==R1_RING_DROP && drops[slot-1]==TEST_TRAY_SLOT(slot) && s_held==TEST_TRAY_SLOT(slot));
   action_done(0);CHECK(s_held==0 && (s_rough_mask&(1U<<(TEST_TRAY_SLOT(slot)-1U))));
 }
 CHECK(s_phase==R1_RING_PICK && s_rough_mask==7 && !s_tray_mask && drop_calls==3);
 for(slot=1;slot<=3;slot++){
   CHECK(s_phase==R1_RING_PICK && retrieved[slot-1]==slot);
   action_done(1);CHECK(s_phase==R1_RAISE && cal==ARM_CAL_PERCENT_MOVE && s_held==slot);
   step(100);CHECK(s_phase==R1_TABLE);lift_pct=ArmCalibration_TransferPercent(240);
   CHECK(cal==ARM_CAL_PERCENT_MOVE); /* Approach adopts the unfinished TOP raise immediately. */
   CHECK(s_phase==R1_TABLE && tables[slot+2]==slot);table_done();CHECK(s_phase==R1_TRAY_PLACE && placed_slots[slot-1]==slot);
   action_done(0);CHECK(s_held==0 && (s_tray_mask&(1U<<(slot-1))));
 }
 CHECK(s_phase==R1_CAMERA && s_tray_mask==7 && !s_rough_mask && camera_calls==2 && ring_pick_calls==3 && place_calls==3);
 camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok);
 CHECK(ArmRound1_Start(ARM_ROUND1_TEMP)==HAL_OK);
 for(slot=1;slot<=3;slot++){
   CHECK(s_phase==R1_TABLE && tables[slot+5]==TEST_TRAY_SLOT(slot));table_done();action_done(1);
   CHECK(s_phase==R1_RING_DROP && drops[slot+2]==TEST_TRAY_SLOT(slot) && drop_temp[slot+2]);action_done(0);
 }
 CHECK(s_phase==R1_CAMERA && !s_tray_mask && !s_rough_mask && s_temp_mask==7 && ring_pick_calls==3 && place_calls==3);
 camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok && camera_calls==3);
 /* Full BLUE second round preserves the red TEMP layer. ROUGH remains
  * ground-level; only second TEMP drops gain72mm. */
 CHECK(ArmRound1_Start(ARM_ROUND2_RAW)==HAL_OK && s_round==2 && s_temp_mask==7);
 for(slot=1;slot<=3;slot++){
   CHECK(materials[slot+2]==slot && material_colors[slot+2]==ARM_MATERIAL_BLUE);
   material_active=0;material_success=1;step(10);
 }
 CHECK(s_phase==R1_DONE && s_tray_mask==7 && s_temp_mask==7 && !s_temp2_mask);
 camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok);
 CHECK(ArmRound1_Start(ARM_ROUND2_ROUGH)==HAL_OK);
 for(slot=1;slot<=3;slot++){
   table_done();action_done(1);CHECK(drops[slot+5]==TEST_TRAY_SLOT(slot) && drop_rise[slot+5]==0 && !drop_temp[slot+5]);
   action_done(0);
 }
 for(slot=1;slot<=3;slot++){
   action_done(1);cal=ARM_CAL_IDLE;lift_pct=0;step(10);table_done();action_done(0);
 }
 CHECK(s_phase==R1_CAMERA && s_tray_mask==7 && !s_rough_mask && s_temp_mask==7 && !s_temp2_mask);
 camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_Start(ARM_ROUND2_TEMP)==HAL_OK);
 for(slot=1;slot<=3;slot++){
   table_done();action_done(1);CHECK(drops[slot+8]==TEST_TRAY_SLOT(slot) && drop_rise[slot+8]==72 && drop_temp[slot+8]);
   action_done(0);
 }
 CHECK(s_phase==R1_CAMERA && !s_tray_mask && s_temp_mask==7 && s_temp2_mask==7 && ring_pick_calls==6 && drop_calls==12);
 camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok && camera_calls==6);
 CHECK(ArmRound1_Start(ARM_ROUND2_RAW)==HAL_ERROR); /* cannot overwrite completed stack */
 CHECK(ArmRound1_Command("arm round1 status") && !ArmRound1_Command("arm round1 blah"));
 /* Preposition slot2 while item3 is still transferring to ring3. Eight
  * seconds later an already settled servo must neither re-command nor
  * restart800ms settling, nor hit an old5s timeout before tray pickup. */
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);
 CHECK(s_phase==R1_RING_DROP && pose_active && s_held==TEST_TRAY_SLOT(1) && table_calls==1 && s_preload_wait_top);
 cal=ARM_CAL_PERCENT_MOVE;lift_pct=0.5991f;step(1000);CHECK(table_calls==1 && s_preload_wait_top); /* still rising */
 cal=ARM_CAL_IDLE;lift_pct=10;step(10);CHECK(table_calls==1); /* stopped below TOP is insufficient */
 lift_pct=0.5991f;step(10);CHECK(table_calls==1 && s_preload_wait_top); /*TOP alone still above tray*/
 pose_clear=1;step(10);CHECK(table_calls==2 && tables[1]==2 && s_preload_slot==2 && !s_preload_wait_top);
 table.ramping=0;step(8000);CHECK(s_phase==R1_RING_DROP && pick_calls==1 && closed);
 action_done(0);CHECK(s_phase==R1_TABLE && s_slot==2 && table_calls==2);
 step(1);CHECK(s_phase==R1_TRAY_PICK && pick_calls==2 && table_calls==2);
 action_done(1);CHECK(s_phase==R1_RING_DROP && s_preload_wait_top && table_calls==2);
 lift_pct=0;cal=ARM_CAL_IDLE;pose_clear=1;step(10);CHECK(s_preload_slot==TEST_TRAY_SLOT(3) && table_calls==3 && tables[2]==TEST_TRAY_SLOT(3));
 step(1000);action_done(0);step(1);CHECK(s_phase==R1_TRAY_PICK && pick_calls==3 && tray_gates); /*approach can overlap ramp*/
 table.ramping=0;step(1);CHECK(s_phase==R1_TRAY_PICK && table_calls==3);
 action_done(1);CHECK(s_phase==R1_RING_DROP && table_calls==3); /* final slot has no next preselection */
 /* A changed table target cannot reuse a stale preselection token. */
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);
 lift_pct=0;pose_clear=1;step(10);CHECK(s_preload_slot==2 && table_calls==2);
 table.target_us=1999;table.ramping=0;step(1000);action_done(0);
 CHECK(s_phase==R1_TABLE && table_calls==3 && tables[2]==2 && table.ramping && !s_preload_slot);
 /* Failed PICK never turns the table toward a following slot. */
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();pose_active=0;pose_success=0;step(10);
 CHECK(s_phase==R1_FAILED && table_calls==1 && !s_preload_slot);
 /* Cancel or a failed inner transfer before TOP cannot rotate the tray
  * when a late/high position sample later arrives. */
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);
 CHECK(s_preload_wait_top && table_calls==1);ArmCalibration_Cancel();lift_pct=0;step(10);
 CHECK(s_phase==R1_FAILED && !s_preload_wait_top && table_calls==1);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);
 pose_active=0;pose_success=0;lift_pct=0;step(10);CHECK(s_phase==R1_FAILED && table_calls==1);
 /* Fail/cancel never advances inventory or starts the following stage. */
 setup();CHECK(ArmRound1_Start(1)==HAL_OK);material_active=0;material_success=0;step(10);
 CHECK(ArmRound1_ReadResult(&ok) && !ok && !s_tray_mask && stops==1 && material_calls==1);
 setup();CHECK(ArmRound1_Start(1)==HAL_OK);chassis=1;step(10);CHECK(s_phase==R1_FAILED && stops==1);
 setup();CHECK(ArmRound1_Start(1)==HAL_OK);ArmCalibration_Cancel();step(10);CHECK(s_phase==R1_FAILED && !s_tray_mask);
 setup();s_tray_mask=7;reject=1;CHECK(ArmRound1_Start(2)==HAL_ERROR && s_phase==R1_FAILED);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table.fault=1;step(10);CHECK(s_phase==R1_FAILED);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);step(5001);CHECK(s_phase==R1_FAILED && !pick_calls);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();pose_active=0;pose_success=0;step(10);CHECK(s_phase==R1_FAILED && s_tray_mask==7 && !drop_calls);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);action_done(0);table_done();step(120001);CHECK(s_phase==R1_FAILED);
 setup();camera=ARM_CAMERA_PRECHECK;CHECK(ArmRound1_Start(1)==HAL_BUSY && !material_calls);
 setup();pose_active=1;CHECK(ArmRound1_Start(1)==HAL_BUSY);pose_active=0;material_active=1;CHECK(ArmRound1_Start(1)==HAL_BUSY);
 setup();reject=1;CHECK(ArmRound1_Start(1)==HAL_ERROR && !material_calls);
 setup();s_rough_mask=1;CHECK(ArmRound1_Start(1)==HAL_ERROR);s_rough_mask=0;s_held=1;CHECK(ArmRound1_Start(1)==HAL_ERROR);
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(2)==HAL_OK);table_done();action_done(1);action_done(1);
 CHECK(s_phase==R1_FAILED && !s_rough_mask && s_held==TEST_TRAY_SLOT(1)); /* release failed: do not mark ring loaded */
 setup();s_tray_mask=7;CHECK(ArmRound1_Start(3)==HAL_OK);
 for(slot=0;slot<3;slot++){table_done();action_done(1);action_done(0);}
 CHECK(s_phase==R1_CAMERA);camera=ARM_CAMERA_FAILED;step(10);CHECK(ArmRound1_ReadResult(&ok) && !ok && s_temp_mask==7);
 setup();s_phase=R1_CAMERA;camera=ARM_CAMERA_FAILED;ArmRound1_AbortState();
 CHECK(s_phase==R1_FAILED && s_failed_phase==R1_CAMERA && !strcmp(s_reason,"mock CAMERA feedback failure"));
 setup();s_station=2;s_slot=1;s_phase=R1_RAISE;s_tick=tick;closed=1;cal=ARM_CAL_IDLE;lift_pct=4;
 step(10);CHECK(s_phase==R1_FAILED && !table_calls); /* no tray rotation while lift target unconfirmed */
 /* Real QR plans: all ring permutations, both batches, matching-color TEMP
  * stacking, correct retrieval order back into the original tray slots. */
 {
   static const uint8_t permutations[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
   static const uint8_t colors[2][3]={{1,6,5},{5,1,6}};
   unsigned p,job,k;
   for(p=0;p<14;p++) {
     uint8_t expected_colors[2][3],expected_positions[2][3];
     setup();task_seen=1;
     if(p<6) {
       memcpy(expected_colors,colors,sizeof(colors));
       memcpy(expected_positions[0],permutations[p],3);memcpy(expected_positions[1],permutations[(p+1)%6],3);
       memcpy(test_task.colors,expected_colors,sizeof(expected_colors));memcpy(test_task.positions,expected_positions,sizeof(expected_positions));
     } else {
       vision_qr_assembly_t assembly;unsigned g,i;const char *raw=qr_mission_raw[p-6];
       VisionQr_Reset(&assembly);
       for(g=0;g<4;g++)CHECK(VisionQr_Feed(&assembly,qr_mission_frames[p-6]+g*8,(uint8_t)(p-6),2000+g)==(g==3));
       test_task=assembly.task;CHECK(!strcmp(test_task.raw,raw));
       for(g=0;g<2;g++)for(i=0;i<3;i++) {
         uint8_t c=(uint8_t)(raw[g*8+i]-'0');
         expected_colors[g][i]=c==5?6:c==6?5:c;
         expected_positions[g][i]=(uint8_t)(raw[g*8+4+i]-'0');
       }
     }
     CHECK(VisionQr_CompetitionTaskValid(&test_task));raw_done();CHECK(s_task_loaded);
     for(k=0;k<3;k++)CHECK(material_colors[k]==expected_colors[0][k]);
     task_seen=0; /* snapshot is immutable even after CAM2 STOP/reset */
     for(job=2;job<=6;job++) {
       unsigned offset=drop_calls,round=job>3?1:0;
       CHECK(ArmRound1_Start((uint8_t)job)==HAL_OK);
       if(job==4) {
         for(k=0;k<3;k++){CHECK(material_colors[k+3]==expected_colors[1][k]);material_active=0;material_success=1;step(10);}
       } else {
         for(k=0;k<3;k++) {
           unsigned index=COMPETITION_PRELIMINARY?k:2U-k,ring=expected_positions[round][index],j;
           CHECK(s_slot==TEST_TRAY_SLOT(k+1U));
           if(job==6)for(j=0;j<3;j++)if(expected_colors[0][j]==expected_colors[1][index])ring=expected_positions[0][j];
           table_done();action_done(1);CHECK(drops[offset+k]==ring && drop_rise[offset+k]==(job==6?72:0));
           CHECK(drop_temp[offset+k]==(job==3 || job==6));
           action_done(0);
         }
         if(job==2 || job==5)for(k=0;k<3;k++) {
           CHECK(retrieved[round*3+k]==expected_positions[round][k]);
           action_done(1);cal=ARM_CAL_IDLE;lift_pct=0;step(10);table_done();action_done(0);
           CHECK(placed_slots[round*3+k]==k+1);
         }
       }
       CHECK(s_phase==(job==4?R1_DONE:R1_CAMERA));camera=ARM_CAMERA_DONE;step(10);CHECK(ArmRound1_ReadResult(&ok) && ok);
     }
     CHECK(!s_tray_mask && !s_rough_mask && s_temp_mask==7 && s_temp2_mask==7 && !s_held);
   }
   setup();task_seen=1;memcpy(test_task.colors,colors,sizeof(colors));
   memcpy(test_task.positions,permutations[0],3);memcpy(test_task.positions[1],permutations[1],3);
   test_task.colors[1][0]=2;CHECK(!VisionQr_CompetitionTaskValid(&test_task));
   CHECK(ArmRound1_Start(1)==HAL_ERROR && !material_calls);
   test_task.colors[1][0]=5;test_task.positions[1][1]=test_task.positions[1][0];CHECK(!VisionQr_CompetitionTaskValid(&test_task));
   CHECK(!VisionQr_CompetitionTaskValid(NULL));
 }
 /* Adoption begins with the raise, without any measured-rise gate. */
 for(slot=0;slot<2;slot++) {
  setup();s_station=2;s_round=1;s_slot=1;s_phase=R1_RAISE;s_returning=1;s_tick=tick;
  s_raise_start_percent=slot?40.0f/2.4f:80.0f;closed=1;cal=ARM_CAL_PERCENT_MOVE;
  lift_pct=s_raise_start_percent;step(10);CHECK(s_phase==R1_TABLE && !place_calls);
  step(10);CHECK(s_phase==R1_TRAY_PLACE && place_calls==1 && cal==ARM_CAL_PERCENT_MOVE);
 }
 ArmRound1_Reset();CHECK(!ArmRound1_IsActive() && !s_tray_mask && !s_rough_mask && !s_temp_mask && !s_held && !s_task_loaded);
 return failures;
}
