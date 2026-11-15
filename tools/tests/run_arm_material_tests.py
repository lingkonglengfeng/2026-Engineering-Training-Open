"""Run the production material FSM and camera gate with deterministic actuator/UART ports."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
syscalls = (root / "tools/tests/test_arm_driver.c").read_text(encoding="utf-8")
syscalls = syscalls[syscalls.index("int _close("):syscalls.index("volatile unsigned int test_checks")]
prefix = r'''
#include "arm_material.h"
#include "arm_speed_defaults.h"
#include "arm_start_pose.h"
#include "arm_calibration.h"
#include "arm_servo.h"
#include "arm_task.h"
#include "vision.h"
#include "vision_mode_gate.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure,failed_lines[32];
#define CHECK(x) do{checks++;if(!(x)){if(failures<32)failed_lines[failures]=__LINE__;failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static uint32_t tick=1000,obs,frame_tick,mode_calls,clear_calls,stop_calls,all_stops;
static uint32_t move_calls,close_calls,open_calls,tray_calls,place_calls,camera_calls,reach_calls,reach_stops,reach_sequence;
static uint32_t wait_pose_calls,dm_snapshot_delay;
static uint8_t selected_slot;
static uint8_t pose_clear;
static uint32_t tray_gates;
static uint32_t lift_sent_tick,reach_sent_tick,grip_sent_tick;
static uint32_t lift_calls_at_reach;
static uint32_t reach_send_calls;
static uint8_t busy_reach;
static float grip_close_scale;
static uint32_t reach_position_age,status_age[2];
static uint16_t reach_speed;
static uint8_t reject_lift,reject_reach,reject_close,reject_stop,reject_pulses,reject_camera,busy_stop;
static uint8_t pose_release_expected,place_releases_dm;
static uint8_t retry_recovering,retry_failed,stall_pending;
static uint8_t reach_only_stall,reach_only_protected,reach_only_disabled,clear_busy,enable_busy,clear_error,enable_error;
static uint32_t clear_reach_calls,enable_reach_calls;
HAL_StatusTypeDef ArmCalibration_ClearProtection(arm_driver_axis_t axis){CHECK(axis==ARM_DRIVER_REACH);clear_reach_calls++;return clear_busy?HAL_BUSY:clear_error?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef ArmDriver_Enable(arm_driver_axis_t axis){CHECK(axis==ARM_DRIVER_REACH);enable_reach_calls++;return enable_busy?HAL_BUSY:enable_error?HAL_ERROR:HAL_OK;}
static uint32_t retry_generation,start_retry_calls;
static uint8_t start_retry_attempts;
static int32_t captured_grasp_pulses;
static uint16_t captured_grasp_speed;
HAL_StatusTypeDef ArmDriver_RetryUnstartedReach(void){
 start_retry_calls++;CHECK(start_retry_attempts>=1 && start_retry_attempts<=3);
 if(start_retry_attempts>=3){retry_failed=1;return HAL_ERROR;}
 start_retry_attempts++;retry_recovering=1;return HAL_OK;
}
uint8_t ArmDriver_ReachRetryRecovering(void){return retry_recovering;}
uint8_t ArmDriver_ReachStallPending(void){return stall_pending;}
uint8_t ArmDriver_ReachRetryFailed(void){return retry_failed;}
uint32_t ArmDriver_ReachRetryGeneration(void){return retry_generation;}
uint32_t ArmDriver_ReachRetryTimeoutMs(double counts){(void)counts;return 20000U;}
void ArmDriver_CancelReachRetry(void){retry_recovering=retry_failed=stall_pending=0;}
static uint8_t pose_active,pose_success,chassis,visible,axis_bad,clear_bad,rotation_on,dm_bad,dm_disabled,dm_stale,axis_disabled,axis_stale,pose_reject;
uint8_t ArmStartPose_IsAwayFromTray(void){return pose_active && pose_clear;}
HAL_StatusTypeDef ArmStartPose_WaitForTray(uint16_t target,uint32_t started){CHECK(pose_active && target==606 && started<=tick);tray_gates++;return HAL_OK;}
static arm_cal_state_t cal;
static arm_camera_state_t camera_arm;
static vision_camera_state_t peer;
static vision_pixel_t frame;
static arm_servo_status_t grip,tray;
static float lift_pct,lift_target,reach_pct,reach_target,dm_angle,grip_angle,move_lift,move_reach,move_dm;
static char console[256];
static arm_cal_position_ack_t ack_snapshot[2],ack_pending[2];
static uint32_t ack_wire[2];static uint8_t ack_queued[2],auto_ack=1U,drop_acks,busy_lift;
static uint32_t native_send_calls,native_queued;static int32_t first_native_goal;
static uint16_t first_native_speed;static float first_native_height;
uint32_t ArmCalibration_PositionAckFence(arm_driver_axis_t axis){return ack_wire[axis==ARM_DRIVER_REACH];}
void ArmCalibration_GetPositionAck(arm_driver_axis_t axis,arm_cal_position_ack_t *out){*out=ack_snapshot[axis==ARM_DRIVER_REACH];}
static void queue_ack(arm_driver_axis_t axis,uint8_t value,uint32_t received){
 unsigned i=axis==ARM_DRIVER_REACH;ack_pending[i].sequence=++ack_wire[i];
 ack_pending[i].received_tick=received;ack_pending[i].response=value;ack_queued[i]=1;
}
static uint8_t drain_acks(void){unsigned i;uint8_t found=0;for(i=0;i<2;i++)if(ack_queued[i]){ack_snapshot[i]=ack_pending[i];ack_queued[i]=0;found=1;}return found;}
uint32_t HAL_GetTick(void){return tick;}
uint8_t ArmCalibration_WaitForFeedback(arm_feedback_wait_t *w,uint32_t now,uint32_t *gap){(void)w;(void)now;*gap=0;return 0;}
void ZDT_Console_Print(const char *s){CHECK(s!=NULL);strncpy(console,s,sizeof(console)-1);}
uint8_t ChassisMotor_HasMotionDemand(void){return chassis;}
uint8_t ArmStartPose_IsActive(void){return pose_active;}
uint8_t ArmStartPose_IsRotationReleaseExpected(void){return pose_active && pose_release_expected;}
static float disc_test_trim[3][3];
HAL_StatusTypeDef ArmStartPose_DiscTargets(uint8_t kind,float base[3],float target[3]){
 unsigned i;CHECK(kind<3);
 base[0]=ARM_MATERIAL_GRAB_LIFT_DEG-(float)ARM_LIFT_FULL_TRAVEL_COUNTS/240.0f*ARM_MATERIAL_GRAB_RAISE_MM*360.0f/65536.0f;
 base[1]=ARM_MATERIAL_WAIT_REACH_DEG;base[2]=ARM_MATERIAL_DM_DEG;
 if(kind==ARM_DISC_GRAB)base[1]+=(float)ARM_REACH_FULL_TRAVEL_COUNTS/178.0f*ARM_MATERIAL_GRAB_EXTEND_MM*360.0f/65536.0f;
 for(i=0;i<3;i++){target[i]=base[i]+disc_test_trim[kind][i];}return HAL_OK;
}
HAL_StatusTypeDef ArmStartPose_BeginMove(float lift,float reach,float dm){wait_pose_calls++;if(pose_reject)return HAL_ERROR;pose_active=1;move_lift=lift;move_reach=reach;move_dm=dm;return HAL_OK;}
HAL_StatusTypeDef ArmStartPose_BeginMaterialApproach(float lift,float reach,float dm){return ArmStartPose_BeginMove(lift,reach,dm);}
uint8_t ArmStartPose_ReadActionResult(uint8_t *success){if(pose_active)return 0;*success=pose_success;return 1;}
uint8_t ArmStartPose_Command(const char *s){unsigned slot=0;CHECK(sscanf(s,"arm place %u",&slot)==1 && slot==selected_slot);place_calls++;pose_active=1;
 if(place_releases_dm){pose_release_expected=1;rotation_on=0;dm_disabled=1;}return 1;}
arm_cal_state_t ArmCalibration_GetState(void){return cal;}
HAL_StatusTypeDef ArmStartPose_BeginTrayPlaceDuringRaise(uint8_t slot,float from){char s[24];CHECK(from>0 && from<=100 && ArmCalibration_IsLiftRaisingToTop());snprintf(s,sizeof(s),"arm place %u",slot);ArmStartPose_Command(s);return HAL_OK;}
uint8_t ArmCalibration_IsLiftRaisingToTop(void){return (cal==ARM_CAL_PERCENT_MOVE || cal==ARM_CAL_PERCENT_SETTLE) && lift_target==0;}
arm_camera_state_t ArmCalibration_GetCameraState(void){return camera_arm;}
HAL_StatusTypeDef ArmCalibration_DeployCamera(void){camera_calls++;if(reject_camera)return HAL_ERROR;camera_arm=ARM_CAMERA_PRECHECK;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_Cancel(void){cal=ARM_CAL_IDLE;pose_active=0;ArmMaterial_AbortState();return HAL_OK;}
HAL_StatusTypeDef ArmDriver_StopAll(void){all_stops++;return HAL_OK;}
uint8_t ArmDriver_IsEnableRequested(arm_driver_axis_t a){return a==ARM_DRIVER_ROTATION?rotation_on:1;}
uint16_t ArmDriver_GetLinearSpeed(arm_driver_axis_t a){return a==ARM_DRIVER_LIFT?262:reach_speed;}
uint16_t ArmDriver_ReachMotionSpeedRpm(void){return reach_speed;}
uint8_t ArmDriver_GetLinearAcceleration(arm_driver_axis_t a){CHECK(a==ARM_DRIVER_LIFT);return 234;}
HAL_StatusTypeDef ArmDriver_LiftAbsoluteRawPulses(int32_t pulses,uint16_t speed,float height){
 CHECK(speed==431 && height<0 && cal==ARM_CAL_IDLE);native_send_calls++;
 if(busy_lift && native_send_calls>1){busy_lift--;return HAL_BUSY;}
 if(native_queued){CHECK(pulses==first_native_goal && speed==first_native_speed && height==first_native_height);}
 else{first_native_goal=pulses;first_native_speed=speed;first_native_height=height;}
 move_calls++;lift_sent_tick=tick;
 lift_target=(float)pulses*65536.0f/ARM_LIFT_POSITION_STEPS_REV/(float)ARM_LIFT_FULL_TRAVEL_COUNTS*100;
 if(reject_lift)return HAL_ERROR;
 native_queued++;
 if(auto_ack){if(drop_acks)drop_acks--;else queue_ack(ARM_DRIVER_LIFT,0x02,tick);}
 return HAL_OK;
}
HAL_StatusTypeDef ArmCalibration_GetAbsoluteLiftPulses(float pct,int32_t *pulses){
 *pulses=(int32_t)roundf(pct/100.0f*ARM_LIFT_FULL_TRAVEL_COUNTS*ARM_LIFT_POSITION_STEPS_REV/65536.0f);return reject_pulses?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef ArmDriver_ReachAbsolutePulses(int32_t pulses,uint16_t speed,uint8_t acceleration){
 reach_send_calls++;if(busy_reach){busy_reach--;return HAL_BUSY;}
 CHECK(speed==(uint16_t)lroundf(reach_speed*1.44f) && acceleration==209U);reach_calls++;reach_sent_tick=tick;start_retry_attempts=1;captured_grasp_pulses=pulses;
 captured_grasp_speed=speed;
 lift_calls_at_reach=move_calls;
 reach_target=(float)pulses*65536.0f/ARM_CAMERA_POSITION_STEPS_REV/(-ARM_REACH_FULL_TRAVEL_COUNTS)*100.0f;
 if(reject_reach)return HAL_ERROR;
 if(auto_ack){if(drop_acks)drop_acks--;else queue_ack(ARM_DRIVER_REACH,0x02,tick);}
 return HAL_OK;
}
HAL_StatusTypeDef ArmCalibration_GetAbsoluteReachPulses(float pct,int32_t *pulses){
 *pulses=(int32_t)roundf(pct/100.0f*(-ARM_REACH_FULL_TRAVEL_COUNTS)*ARM_CAMERA_POSITION_STEPS_REV/65536.0f);return reject_pulses?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef ArmDriver_Stop(arm_driver_axis_t a){CHECK(a==ARM_DRIVER_REACH);reach_stops++;return busy_stop?HAL_BUSY:reject_stop?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef ArmDriver_StopReachForHold(void){return ArmDriver_Stop(ARM_DRIVER_REACH);}
dm_motor_t *ArmTask_GetMotor(void){return NULL;}
uint8_t ArmTask_IsFaulted(void){return dm_bad;}
void DM_GetFeedback(const dm_motor_t *motor,dm_feedback_t *out){
 tick+=dm_snapshot_delay;
 (void)motor;memset(out,0,sizeof(*out));out->fb_count=1;out->enabled=!dm_disabled;
 out->err_code=dm_bad?8:dm_disabled?0:1;out->last_tick=tick-(dm_stale?DM_LINK_TIMEOUT_MS_DEFAULT+1U:0U);
}
HAL_StatusTypeDef ArmDriver_GetRotationDeg(float *out){*out=dm_angle;return HAL_OK;}
void ArmCalibration_GetAxisStatus(arm_driver_axis_t axis,arm_cal_axis_status_t *a){
 memset(a,0,sizeof(*a));a->position_sequence=axis==ARM_DRIVER_REACH?reach_sequence:1;a->flags_sequence=1;a->position_tick=tick;a->flags_tick=tick-status_age[axis==ARM_DRIVER_REACH?1:0];
 if(axis==ARM_DRIVER_REACH)a->position_tick=tick-reach_position_age;
 a->flags=axis_bad?0x85:axis_disabled?0x80:0x81;if(axis_stale)a->flags_tick=tick-ArmCalibration_StatusTimeoutMs(axis)-1U;
 if(axis==ARM_DRIVER_REACH){if(reach_only_stall)a->flags|=ARM_CAL_FLAG_STALL;if(reach_only_protected)a->flags|=ARM_CAL_FLAG_PROTECTED;if(reach_only_disabled)a->flags&=~ARM_CAL_FLAG_ENABLED;}
 a->zero_valid=1;a->travel_mm=axis==ARM_DRIVER_LIFT?240:178;
 a->position_counts=(int64_t)roundf((axis==ARM_DRIVER_LIFT?lift_pct:reach_pct)*(float)(axis==ARM_DRIVER_LIFT?ARM_LIFT_FULL_TRAVEL_COUNTS:ARM_REACH_FULL_TRAVEL_COUNTS)/100.0f);
}
HAL_StatusTypeDef ArmCalibration_GetSpan(arm_driver_axis_t axis,float *span){*span=(float)(axis==ARM_DRIVER_LIFT?ARM_LIFT_FULL_TRAVEL_COUNTS:ARM_REACH_FULL_TRAVEL_COUNTS);return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_GetPercent(arm_driver_axis_t axis,float *pct){*pct=axis==ARM_DRIVER_LIFT?lift_pct:reach_pct;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_MovePercent(arm_driver_axis_t axis,float pct){CHECK(axis==ARM_DRIVER_LIFT);move_calls++;lift_sent_tick=tick;lift_target=pct;cal=ARM_CAL_PERCENT_MOVE;return reject_lift?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef ArmServo_TestAngleDeg(arm_servo_id_t id,float angle){
 if(id==ARM_SERVO_TABLE){CHECK(angle==19 || angle==109 || angle==199);tray_calls++;tray.enabled=1;tray.ramping=1;tray.target_us=606;}
 else{grip_angle=angle;grip.enabled=1;grip.ramping=1;grip.target_us=(uint16_t)roundf(500+angle*2000/360);
 if(angle==ARM_SERVO_GRIP_CLOSE_DEG){
 CHECK(cal==ARM_CAL_IDLE);
 CHECK((uint32_t)(tick-reach_sent_tick)>0 && !native_queued && move_calls==lift_calls_at_reach);
 CHECK(reach_calls>0);
 close_calls++;grip_sent_tick=tick;}else{CHECK(angle==ARM_SERVO_GRIP_OPEN_DEG);open_calls++;}}
 return (id==ARM_SERVO_GRIP && angle==ARM_SERVO_GRIP_CLOSE_DEG && reject_close)?HAL_ERROR:HAL_OK;
}
float ArmServo_MaterialSlotDeg(uint8_t slot){const float degrees[]={19,109,199};return slot>=1 && slot<=3?degrees[slot-1]:NAN;}
float ArmServo_GripOpenDeg(void){return ARM_SERVO_GRIP_OPEN_DEG;}
uint8_t ArmServo_GripOpenConfirmed(void){return grip.enabled && !grip.fault && grip_angle==ARM_SERVO_GRIP_OPEN_DEG;}
uint8_t ArmServo_GripIsOpen(void){return grip.enabled && !grip.ramping && !grip.fault && grip_angle==ARM_SERVO_GRIP_OPEN_DEG;}
HAL_StatusTypeDef ArmServo_GripClose(void){grip_close_scale=1.0f;return ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG);}
HAL_StatusTypeDef ArmServo_GripCloseScaled(float scale){CHECK(scale==0.75f);grip_close_scale=scale;return ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG);}
HAL_StatusTypeDef ArmServo_SelectMaterialSlotForPoint(uint8_t slot,uint8_t point){(void)point;return ArmServo_SelectMaterialSlot(slot);}
HAL_StatusTypeDef ArmServo_SelectMaterialSlot(uint8_t slot){selected_slot=slot;return ArmServo_TestAngleDeg(ARM_SERVO_TABLE,ArmServo_MaterialSlotDeg(slot));}
void ArmServo_GetStatus(arm_servo_id_t id,arm_servo_status_t *out){*out=id==ARM_SERVO_GRIP?grip:tray;}
uint8_t ArmServo_GripIsClosed(void){return grip.enabled && !grip.ramping && !grip.fault && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG;}
uint8_t Vision_GetCameraState(uint8_t id,vision_camera_state_t *out){CHECK(id==1);*out=peer;return 1;}
HAL_StatusTypeDef Vision_RequestMode(uint8_t id,uint8_t mode,uint8_t station,uint8_t target,uint8_t hz,uint8_t flags){
 CHECK(id==1 && mode==2 && station==1 && target>=1 && target<=6 && hz==20 && (flags==0 || flags==VISION_MATERIAL_CALIBRATION_FLAG));mode_calls++;
 peer.mode=mode;peer.station=station;peer.target=target;peer.hz=hz;peer.flags=flags;peer.pending=1;peer.confirmed=0;peer.result=255;peer.request_seq=(peer.request_seq+1U)&15U;
 return HAL_OK;
}
HAL_StatusTypeDef Vision_RequestStop(uint8_t id){CHECK(id==1);stop_calls++;peer.mode=0;peer.confirmed=1;peer.pending=peer.reset_pending=0;return HAL_OK;}
uint8_t Vision_ClearPixelObservations(uint8_t id){CHECK(id==1);clear_calls++;visible=0;return !clear_bad;}
uint8_t Vision_GetPixelWindow(uint8_t id,uint8_t type,uint16_t window,vision_pixel_t *out,uint32_t *age){
 CHECK(id==1 && type==2 && window==200);memset(out,0,sizeof(*out));*age=UINT32_MAX;
 if(!visible)return 0;
 *out=frame;*age=tick-frame_tick;return 1;
}
'''
block = (root / "Core/Src/arm_material.c").read_text(encoding="utf-8") + "\n" + (root / "Core/Src/vision_mode_gate.c").read_text(encoding="utf-8")
suffix = r'''
static void step(uint32_t ms){tick+=ms;reach_sequence++;drain_acks();ArmMaterial_Poll();
 /* Default port replies immediately; a second owner poll consumes that
  * queued reply. Loss/delay tests below disable this automatic fixture. */
 if(drain_acks())ArmMaterial_Poll();}
static void finish_reach_half(void){
 CHECK(s_phase==MAT_GRAB && s_grab_reach_started && !s_grip_started);
 uint32_t before=reach_calls,closed_before=close_calls;
 reach_pct=s_reach_half_pct;step(1);
 CHECK(s_grip_started && grip.ramping && close_calls==closed_before+1 && reach_calls==before && !reach_stops && !start_retry_calls);
}
static void finish_grip(void){
 CHECK(s_grip_started && (s_phase==MAT_GRAB || s_phase==MAT_RAISE));
 if(s_phase==MAT_GRAB){uint32_t elapsed=tick-s_grab_reach_tick;grip.ramping=0;step(elapsed<s_grab_reach_ms?s_grab_reach_ms-elapsed:1);}
 CHECK(s_phase==MAT_RAISE);
}
static void image_frame(uint8_t id,uint8_t valid,float x,float y,uint32_t age){
 visible=1;memset(&frame,0,sizeof(frame));frame.type=2;frame.id=id;frame.valid=valid;frame.seq=(uint8_t)(++obs%16);
 frame.observation_id=obs;frame.dx_px=x;frame.dy_px=y;frame_tick=tick-age;
}
static void setup(void){
 memset(disc_test_trim,0,sizeof(disc_test_trim));
 memset(ack_snapshot,0,sizeof(ack_snapshot));memset(ack_pending,0,sizeof(ack_pending));
 ack_wire[0]=ack_wire[1]=0;ack_queued[0]=ack_queued[1]=0;auto_ack=1;drop_acks=busy_lift=0;
 native_send_calls=native_queued=0;first_native_goal=0;first_native_speed=0;first_native_height=0;
 reach_send_calls=0;busy_reach=0;
 pose_clear=0;tray_gates=0;
 tick=1000;obs=0;mode_calls=clear_calls=stop_calls=all_stops=0;move_calls=close_calls=open_calls=tray_calls=place_calls=camera_calls=reach_calls=reach_stops=0;reach_sequence=1;
 pose_active=chassis=visible=axis_bad=clear_bad=dm_bad=dm_disabled=dm_stale=axis_disabled=axis_stale=pose_reject=0;
 rotation_on=pose_success=1;cal=ARM_CAL_IDLE;camera_arm=ARM_CAMERA_IDLE;
 reject_lift=reject_reach=reject_close=reject_stop=reject_pulses=reject_camera=busy_stop=0;reach_speed=60;lift_sent_tick=reach_sent_tick=grip_sent_tick=0;
 reach_position_age=0;status_age[0]=status_age[1]=0;
 grip_close_scale=1.0f;
 pose_release_expected=place_releases_dm=0;
 wait_pose_calls=0;
 retry_recovering=retry_failed=0;retry_generation=0;start_retry_calls=0;start_retry_attempts=0;captured_grasp_pulses=0;
 reach_only_stall=reach_only_protected=reach_only_disabled=clear_busy=enable_busy=clear_error=enable_error=0;clear_reach_calls=enable_reach_calls=0;
 memset(&peer,0,sizeof(peer));memset(&grip,0,sizeof(grip));memset(&tray,0,sizeof(tray));peer.result=255;
 ArmMaterial_Init();
}
static void wait_ready(void){
 CHECK(ArmMaterial_Start()==HAL_OK);step(10);CHECK(mode_calls==1 && s_phase==MAT_POSITION);
 peer.pending=0;peer.confirmed=1;peer.result=0;peer.peer_age_ms=0;
 step(100);CHECK(s_phase==MAT_POSITION && move_calls==0);
 pose_active=0;lift_pct=move_lift/360.0f*65536.0f/(float)ARM_LIFT_FULL_TRAVEL_COUNTS*100;
 reach_pct=move_reach/360.0f*65536.0f/(float)ARM_REACH_FULL_TRAVEL_COUNTS*100;dm_angle=move_dm;
 grip.ramping=0;step(100);CHECK(s_phase==MAT_WAIT && clear_calls==1 && !close_calls);
 CHECK(fabsf(move_lift+562.5272f)<0.01f && fabsf(move_reach+185.1581f)<0.001f && fabsf(move_dm+2.0217f)<0.001f);
}
static void loop_ready(void){
 CHECK(ArmMaterial_Command("arm material loop start")==1);CHECK(s_loop && s_phase==MAT_POSITION);
 step(10);CHECK(mode_calls==1 && s_phase==MAT_POSITION);
 CHECK(wait_pose_calls==0 && !pose_active);
 peer.pending=0;peer.confirmed=1;peer.result=0;peer.peer_age_ms=0;
 step(10);CHECK(wait_pose_calls==1 && pose_active && s_phase==MAT_POSITION);
 pose_active=0;pose_success=1;grip.ramping=0;
 lift_pct=s_wait_pct;reach_pct=s_reach_wait_pct;dm_angle=s_wait_pose[2];
 step(10);CHECK(s_phase==MAT_WAIT && clear_calls==1 && !close_calls);
}
static void placed(void){
 image_frame(s_color,1,0,-20,0);step(10);CHECK(s_phase==MAT_GRAB);
 cal=ARM_CAL_IDLE;grip.ramping=0;finish_reach_half();finish_grip();CHECK(s_phase==MAT_RAISE);
 lift_pct=lift_target+0.49f;cal=ARM_CAL_IDLE;step(10);CHECK(s_phase==MAT_TRAY); /*clearance1.2mm band*/
 tray.ramping=0;step(810);CHECK(s_phase==MAT_PLACE);
 pose_active=0;pose_success=1;grip.ramping=1;grip_angle=ARM_SERVO_GRIP_OPEN_DEG;
 /* PLACE success already includes OPEN age50ms; ramp must not delay handoff. */
 /* Actual PLACE moves the axes away from the wait pose. */
 reach_pct=1;dm_angle=162.825f;step(10);
}
static void loop_rearm(void){
 uint32_t clear_before=clear_calls,mode_before=mode_calls;
 CHECK(s_phase==MAT_POSITION && s_loop && !ArmMaterial_ReadResult(&(uint8_t){0}));
 step(10);CHECK(mode_calls==mode_before+1 && peer.pending && s_phase==MAT_POSITION);
 pose_active=0;pose_success=1;grip.ramping=0;
 lift_pct=s_wait_pct;reach_pct=s_reach_wait_pct;dm_angle=s_wait_pose[2];
 image_frame(1,1,0,0,0);step(10);CHECK(s_phase==MAT_POSITION); /* New ACK is mandatory. */
 peer.pending=0;peer.confirmed=1;peer.result=0;step(10);
 CHECK(s_phase==MAT_POSITION && pose_active);
 pose_active=0;grip.ramping=0;step(10);
 CHECK(s_phase==MAT_WAIT && clear_calls==clear_before+1 && !visible);
}
static void reach_phase(uint8_t phase){
 setup();loop_ready();
 if(phase==MAT_WAIT)return;
 image_frame(1,1,0,0,0);step(10);
 if(phase==MAT_GRAB)return;
 cal=ARM_CAL_IDLE;grip.ramping=0;finish_reach_half();finish_grip();
 if(phase==MAT_RAISE)return;
 lift_pct=lift_target;cal=ARM_CAL_IDLE;step(10);
 if(phase==MAT_TRAY)return;
 tray.ramping=0;step(810);CHECK(phase==MAT_PLACE && s_phase==MAT_PLACE);
}
static void low_wait_regressions(void){
 /* The wait pose must finish at the existing grab endpoint before looking
  * for a NEW frame. In-window triggers issue reach in that very same poll. */
 setup();CHECK(ArmMaterial_Start()==HAL_OK);step(10);
 CHECK(!wait_pose_calls && !reach_calls && !move_calls);
 peer.pending=0;peer.confirmed=1;peer.result=0;step(10);
 CHECK(pose_active && wait_pose_calls==1 && fabsf(move_lift+562.5272f)<0.01f);
 image_frame(1,1,0,-20,0);step(200);CHECK(s_phase==MAT_POSITION && !reach_calls);
 pose_active=0;pose_success=1;grip.ramping=0;
 lift_pct=s_wait_pct;reach_pct=s_reach_wait_pct;dm_angle=ARM_MATERIAL_DM_DEG;step(10);
 CHECK(s_phase==MAT_WAIT && !visible && s_wait_pct==s_grab_pct);
 CHECK(fabsf(move_lift-s_grab_pct*(float)ARM_LIFT_FULL_TRAVEL_COUNTS/100*360/65536)<0.01f);
 step(1000);CHECK(!reach_calls && !move_calls && !close_calls);
 auto_ack=0;image_frame(1,1,0,-20,0);step(1);
 CHECK(s_phase==MAT_GRAB && reach_calls==1 && reach_sent_tick==tick);
 CHECK(!native_queued && !native_send_calls && !move_calls && !close_calls);
 CHECK(lift_pct==s_grab_pct && s_gy==-20 && s_wx==105 && s_wy==100);
 step(200);CHECK(reach_calls==2 && !move_calls && !close_calls);
 /* Tick wrap must not break the half-stroke gate or final timed completion. */
 setup();loop_ready();tick=UINT32_MAX-100U;s_wait_tick=tick;
 image_frame(1,1,0,-20,0);step(1);finish_reach_half();finish_grip();
 CHECK(s_phase==MAT_RAISE && !all_stops && !native_queued && move_calls==1);
}
static void reach_ack_and_window_regressions(void){
 uint32_t begin;int32_t goal;uint16_t speed;unsigned lost;uint8_t success;
 static const unsigned lost_cases[]={0,2,3,49};unsigned case_index;
 for(case_index=0;case_index<sizeof(lost_cases)/sizeof(lost_cases[0]);case_index++) {
  lost=lost_cases[case_index];
  setup();loop_ready();drop_acks=(uint8_t)lost;image_frame(1,1,0,-20,0);step(1);
  goal=captured_grasp_pulses;speed=captured_grasp_speed;begin=tick;
  while(!s_reach_acked){step(10);CHECK(!close_calls && !all_stops);}
  CHECK(reach_calls==lost+1 && s_reach_attempts==lost+1 && tick-begin==lost*10);
  CHECK(captured_grasp_pulses==goal && captured_grasp_speed==speed && !reach_stops && !native_queued);
  finish_reach_half();finish_grip();CHECK(s_phase==MAT_RAISE);
 }
 /* No ACK: exactly50 successful sends, then remain OPEN under the old action
  * timeout. A fresh half-crossing can still close without a late ACK. */
 setup();loop_ready();auto_ack=0;image_frame(1,1,0,-20,0);step(1);begin=tick;goal=captured_grasp_pulses;
 step(9);CHECK(reach_calls==1);step(1);CHECK(reach_calls==2);
 step(10);CHECK(reach_calls==3 && !close_calls);
 while(reach_calls<49)step(10);
 CHECK(!s_reach_exhausted && s_reach_attempts==49);
 step(9);CHECK(reach_calls==49);step(1);CHECK(reach_calls==50 && s_reach_attempts==50);
 step(10);
 CHECK(s_reach_exhausted && !all_stops && !ArmMaterial_ReadResult(&success));
 step(500);CHECK(reach_calls==50 && captured_grasp_pulses==goal && !close_calls);
 finish_reach_half();CHECK(close_calls==1 && !s_reach_acked && reach_calls==50);
 /* ACK-without-motion uses the SAME10ms cadence; same ABS,
  * original speed, no STOP/retreat, no new timeout or retry forever. */
 setup();loop_ready();image_frame(1,1,0,-20,0);step(1);goal=captured_grasp_pulses;speed=captured_grasp_speed;
 CHECK(s_reach_acked);step(9);CHECK(reach_calls==1);step(1);CHECK(reach_calls==2);
 CHECK(captured_grasp_pulses==goal && captured_grasp_speed==speed);
 step(10);CHECK(reach_calls==3 && !close_calls && !reach_stops && !all_stops);
 while(reach_calls<49)step(10);
 CHECK(!s_reach_exhausted);step(9);CHECK(reach_calls==49);step(1);CHECK(reach_calls==50);
 step(10);CHECK(s_reach_exhausted && reach_calls==50 && !all_stops);
 CHECK(captured_grasp_pulses==goal && captured_grasp_speed==speed && !close_calls && !reach_stops);
 step(120000-(tick-s_tick));CHECK(s_phase==MAT_GRAB && reach_calls==50 && !all_stops);
 step(1);CHECK(s_phase==MAT_FAILED && all_stops && reach_calls==50 && !close_calls);
 /* Measured motion suppresses replays even if ACK is lost; half still gates. */
 setup();loop_ready();auto_ack=0;image_frame(1,1,0,-20,0);step(1);
 reach_pct=s_reach_start_pct+ARM_PERCENT_TOLERANCE+0.01f;step(10);
 step(5000);CHECK(s_reach_moved && reach_calls==1 && !close_calls);
 finish_reach_half();CHECK(close_calls==1);
 /* Old/other-axis/rejected ACKs cannot accept this command. */
 setup();loop_ready();auto_ack=0;image_frame(1,1,0,-20,0);tick++;
 queue_ack(ARM_DRIVER_REACH,0x02,tick);ArmMaterial_Poll();drain_acks();ArmMaterial_Poll();
 CHECK(!s_reach_acked && s_reach_ack_fence==ack_snapshot[1].sequence);
 queue_ack(ARM_DRIVER_LIFT,0x02,tick);step(1);CHECK(!s_reach_acked);
 queue_ack(ARM_DRIVER_REACH,0xEE,tick);step(1);CHECK(!s_reach_acked);
 queue_ack(ARM_DRIVER_REACH,0x02,s_reach_ack_tick-1);step(1);CHECK(!s_reach_acked);
 queue_ack(ARM_DRIVER_REACH,0x9F,tick);step(1);CHECK(s_reach_acked && !close_calls);
 /* BUSY is not a send; retransmission doesn't restart the total stage timer. */
 setup();loop_ready();auto_ack=0;image_frame(1,1,0,-20,0);step(1);begin=s_tick;
 busy_reach=1;step(10);CHECK(reach_calls==1 && s_reach_attempts==1);
 step(10);CHECK(reach_calls==2 && s_reach_attempts==2 && s_tick==begin);
 reject_reach=1;step(10);CHECK(s_phase==MAT_FAILED && all_stops && !close_calls);
 setup();loop_ready();auto_ack=0;image_frame(1,1,0,-20,0);step(1);
 step(10);ArmMaterial_Command("arm material stop");step(1000);CHECK(reach_calls==2 && !close_calls && !place_calls);
 /* Tick and ACK sequence wrap remain supported. */
 setup();loop_ready();tick=UINT32_MAX-50U;s_wait_tick=tick;auto_ack=0;
 ack_wire[1]=ack_snapshot[1].sequence=UINT32_MAX;
 image_frame(1,1,0,-20,0);step(1);step(10);queue_ack(ARM_DRIVER_REACH,0x02,tick);step(0);
 CHECK(s_reach_acked && ack_snapshot[1].sequence==0 && reach_calls==2);finish_reach_half();
 /* 23 seconds starts at pose/open READY, not CAM1 ACK or the move start. */
 setup();loop_ready();begin=s_wait_tick;step(22999);CHECK(s_phase==MAT_WAIT && !all_stops);
 step(1);CHECK(tick-begin==23000 && s_phase==MAT_FAILED && all_stops && !reach_calls && !close_calls);
 CHECK(ArmMaterial_ReadResult(&success) && !success && !s_loop);
 setup();loop_ready();step(22998);image_frame(1,1,0,-20,0);step(1);
 CHECK(s_phase==MAT_GRAB && reach_calls==1);step(2);CHECK(s_phase==MAT_GRAB && !all_stops);
 setup();loop_ready();step(22999);image_frame(1,1,0,-20,0);step(1);
 CHECK(s_phase==MAT_FAILED && !reach_calls && !close_calls);
 setup();loop_ready();tick=UINT32_MAX-100U;s_wait_tick=tick;step(23000);
 CHECK(s_phase==MAT_FAILED && all_stops && !reach_calls);
}
static void final_gripper_regressions(void){
 setup();disc_test_trim[ARM_DISC_WAIT][1]=-10;disc_test_trim[ARM_DISC_GRAB][0]=10;disc_test_trim[ARM_DISC_GRAB][2]=3;disc_test_trim[ARM_DISC_GRAB][1]=-20;
 loop_ready();CHECK(s_phase==MAT_WAIT && fabsf(move_reach-(ARM_MATERIAL_WAIT_REACH_DEG-10))<.001f);
 image_frame(1,1,0,-20,0);step(10);
 CHECK(s_phase==MAT_GRAB && s_grab_pose_pending && pose_active && !reach_calls && !close_calls);
 CHECK(fabsf(move_lift-s_grab_pose[0])<.001f && fabsf(move_reach-s_wait_pose[1])<.001f && fabsf(move_dm-s_grab_pose[2])<.001f);
 step(20);CHECK(!reach_calls && !close_calls);
 pose_active=0;lift_pct=s_grab_pct;reach_pct=s_reach_wait_pct;dm_angle=s_grab_pose[2];step(1);
 CHECK(!s_grab_pose_pending && reach_calls==1 && !close_calls && s_grab_reach_started && !all_stops);
 finish_reach_half();CHECK(close_calls==1 && !all_stops);
 setup();disc_test_trim[ARM_DISC_GRAB][2]=3;loop_ready();image_frame(1,1,0,-20,0);step(10);
 pose_active=0;pose_success=0;step(1);CHECK(s_phase==MAT_FAILED && !reach_calls && !close_calls);
 setup();CHECK(ArmMaterial_Command("arm material once 5 3")==1 && s_color==5 && s_slot==3 && s_batch && !s_loop);
}
int main(void){
 uint8_t success,phase;uint32_t before;unsigned j;
 final_gripper_regressions();
 low_wait_regressions();
 reach_ack_and_window_regressions();
 /* Same feedback-health policy, independent of the timed close trigger. */
 {unsigned axis,k;static const uint32_t ages[]={800U,801U,807U,2000U,2001U};
  for(axis=0;axis<2;axis++)for(k=0;k<sizeof(ages)/sizeof(ages[0]);k++) {
   setup();wait_ready();status_age[axis]=ages[k];
   CHECK(Mat_HoldHealthy(tick)==(ages[k]<=(axis?800U:2000U)));
   CHECK(Mat_LoopLinksHealthy(tick)==(ages[k]<=(axis?800U:2000U)));
  }
 }
 /* Independent Emm time examples: trapezoid, triangle, acc0 and signed travel. */
 CHECK(Mat_PositionTimeMs(65536.0,60,200)==1168);
 CHECK(Mat_PositionTimeMs(-65536.0,60,200)==1168);
 CHECK(Mat_PositionTimeMs(655.36,60,200)==82);
 CHECK(Mat_PositionTimeMs(65536.0,60,0)==1000);
 CHECK(Mat_PositionTimeMs(0,60,230)==0);
 /* Default V223 OPEN101RPM gets the independent grab1.44 multiplier. */
 setup();loop_ready();reach_speed=182;image_frame(1,1,0,-20,0);step(10);
 CHECK(reach_calls==1 && s_reach_speed==262 && !move_calls && !close_calls);
 /* Respect the original configured reach speed; no initial lift command. */
 setup();loop_ready();reach_speed=5;image_frame(1,1,0,-20,0);step(10);
 CHECK(s_grab_reach_ms>1000 && reach_calls==1 && !close_calls && !move_calls);
 finish_reach_half();CHECK(close_calls==1 && reach_speed==5);
 setup();CHECK(!ArmMaterial_AutoEnabled());ArmMaterial_Command("arm material auto on");CHECK(ArmMaterial_AutoEnabled());
 ArmMaterial_Command("arm material auto off");CHECK(!ArmMaterial_AutoEnabled());
 image_frame(1,1,0,0,0);wait_ready();
 visible=1;frame_tick=s_wait_tick-1;step(10);CHECK(s_phase==MAT_WAIT && !reach_calls);
 image_frame(2,1,0,0,0);step(10);CHECK(!reach_calls);
 image_frame(1,0,0,0,0);step(10);CHECK(!reach_calls);
 image_frame(1,1,121,-20,0);step(10);CHECK(!reach_calls);
 image_frame(1,1,0,-20,151);step(10);CHECK(!reach_calls);
 image_frame(1,1,0,-20,0);step(10);
 CHECK(s_phase==MAT_GRAB && !move_calls && reach_calls==1 && !close_calls && !s_grip_started);
 CHECK(reach_sent_tick==tick && !native_send_calls && cal==ARM_CAL_IDLE);
 CHECK(s_grab_pct==s_wait_pct && fabsf(move_lift+562.5272f)<0.01f);
 CHECK(fabsf((s_reach_grab_pct-s_reach_wait_pct)*178/100-25)<0.03f);
 CHECK(grip_close_scale==1 && stop_calls==1 && s_grab_reach_ms>0);
 reach_pct=s_reach_half_pct-0.01f;step(1);CHECK(!close_calls && !all_stops);
 CHECK(fabsf((s_reach_half_pct-s_reach_start_pct)*178/100-12.5f)<0.03f);
 reach_pct=s_reach_half_pct;
 reach_sequence=s_reach_start_sequence;tick++;ArmMaterial_Poll();CHECK(!close_calls);
 reach_position_age=tick-s_grab_reach_tick+2;step(1);CHECK(!close_calls);
 reach_position_age=0;step(1);CHECK(close_calls==1 && s_grip_started && grip.ramping);
 CHECK(grip_sent_tick>reach_sent_tick && reach_pct<s_reach_grab_pct);
 step(s_grab_reach_ms/2);CHECK(close_calls==1 && reach_calls==1 && s_phase==MAT_GRAB && !move_calls);
 grip.ramping=0;step(s_grab_reach_ms-(tick-s_grab_reach_tick)-1);CHECK(s_phase==MAT_GRAB && !move_calls);
 step(1);CHECK(s_phase==MAT_RAISE && move_calls==1 && lift_sent_tick==tick);
 CHECK(lift_target==0 && fabsf(s_raise_start_percent-s_grab_pct)<0.001f);
 lift_pct=s_raise_start_percent*0.5f+1;step(10);CHECK(s_phase==MAT_RAISE);
 lift_pct=s_raise_start_percent*0.5f-0.01f;step(10);CHECK(s_phase==MAT_TRAY && cal==ARM_CAL_PERCENT_MOVE);
 step(10);CHECK(s_phase==MAT_PLACE && place_calls==1 && tray_gates==1);
 pose_active=0;pose_success=1;grip.ramping=0;grip_angle=ARM_SERVO_GRIP_OPEN_DEG;step(10);
 CHECK(s_phase==MAT_CAMERA && camera_calls==1);camera_arm=ARM_CAMERA_DONE;step(10);
 CHECK(ArmMaterial_ReadResult(&success) && success && !ArmMaterial_IsActive());
 /* Even if reach arrives early, retain V207's full timed extension finish. */
 setup();loop_ready();image_frame(1,1,0,-20,0);step(10);
 reach_pct=reach_target;step(1);CHECK(close_calls==1);grip.ramping=0;
 step(s_grab_reach_ms-(tick-s_grab_reach_tick)-1);CHECK(s_phase==MAT_GRAB);step(1);CHECK(s_phase==MAT_RAISE);
 setup();loop_ready();image_frame(1,1,0,-20,0);step(10);finish_reach_half();
 step(s_grab_reach_ms+10);CHECK(s_phase==MAT_GRAB && !move_calls && grip.ramping);
 grip.ramping=0;step(1);CHECK(s_phase==MAT_RAISE && move_calls==1);
 /* Time alone never closes; first new half-stroke sample does, without dwell. */
 setup();loop_ready();image_frame(1,1,0,-20,0);step(10);
 reach_pct=s_reach_half_pct-0.01f;step(s_grab_reach_ms+10);
 CHECK(!close_calls && !all_stops && reach_calls==1 && !move_calls);
 reach_pct=s_reach_half_pct;step(1);CHECK(close_calls==1 && grip.ramping && !move_calls);
 grip.ramping=0;step(1);CHECK(s_phase==MAT_RAISE && move_calls==1 && close_calls==1);
 /* Reach corridor is allowed even before half-stroke CLOSE. */
 setup();loop_ready();image_frame(1,1,0,-20,0);step(10);
 for(j=0;j<=10;j++){reach_pct=s_reach_wait_pct+(s_reach_grab_pct-s_reach_wait_pct)*j/10;step(1);CHECK(s_phase==MAT_GRAB && !all_stops);}
 CHECK(close_calls==1 && reach_calls==1);
 for(j=0;j<3;j++){
  setup();loop_ready();image_frame(1,1,0,-20,0);step(10);
  if(j==0)reach_pct=s_reach_grab_pct+1;else if(j==1)reach_position_age=801;else reach_pct=NAN;
  step(1);CHECK(s_phase==MAT_FAILED && all_stops && !close_calls);
 }
 /* Command rejection and STOP cannot leave a deferred close/raise. */
 for(j=0;j<3;j++){
  setup();loop_ready();if(j==0)reject_pulses=1;else if(j==1)reject_reach=1;else reject_close=1;
  image_frame(1,1,0,-20,0);step(10);if(j==2){reach_pct=s_reach_half_pct;step(1);}
  CHECK(s_phase==MAT_FAILED && all_stops && !s_loop);before=close_calls;step(10000);CHECK(close_calls==before);
 }
 setup();loop_ready();reach_speed=0;image_frame(1,1,0,-20,0);step(10);CHECK(s_phase==MAT_FAILED && !move_calls && !close_calls);
 for(j=0;j<2;j++){
  setup();loop_ready();image_frame(1,1,0,-20,0);step(10);if(j)finish_reach_half();
  before=close_calls;ArmMaterial_Command("arm material stop");grip.ramping=0;step(10000);
  CHECK(s_phase==MAT_FAILED && close_calls==before && !move_calls && !place_calls);
 }
 /* Color confirmation and pixel-window boundaries still gate motion itself. */
 setup();wait_ready();ArmMaterial_Command("arm material confirm 2");image_frame(1,1,0,0,0);step(10);CHECK(!reach_calls);
 step(10);CHECK(!reach_calls);image_frame(1,1,0,0,0);step(10);CHECK(reach_calls==1 && !close_calls);
 setup();wait_ready();ArmMaterial_Command("arm material jump 5");image_frame(1,1,125,0,0);step(10);
 image_frame(1,1,0,0,0);step(10);CHECK(!reach_calls);image_frame(1,1,1,0,0);step(10);CHECK(reach_calls==1);
 setup();loop_ready();image_frame(1,1,0,-120.1f,0);step(10);CHECK(!reach_calls);
 image_frame(1,1,0,80.1f,0);step(10);CHECK(!reach_calls);
 image_frame(1,1,-105.1f,-30,0);step(10);CHECK(!reach_calls);
 image_frame(1,1,105,-120,0);step(10);CHECK(reach_calls==1);
 setup();loop_ready();image_frame(1,1,-105,80,0);step(10);CHECK(reach_calls==1);
 setup();wait_ready();ArmMaterial_Command("arm material window 20 -10 4 3");
 image_frame(1,1,0,0,0);step(10);CHECK(!reach_calls);image_frame(1,1,20,-10,0);step(10);CHECK(reach_calls==1);
 setup();ArmMaterial_Command("arm material window 0 0 -1 12");CHECK(s_wx==105);
 ArmMaterial_Command("arm material fresh 201");CHECK(s_fresh==150);ArmMaterial_Command("arm material fresh 100");CHECK(s_fresh==100);
 /* Multiple rounds rearm with a new camera SET/ACK and no old-frame trigger. */
 setup();loop_ready();
 for(before=0;before<3;before++){
  CHECK(close_calls==before && !camera_calls);placed();CHECK(s_completed==before+1 && s_phase==MAT_POSITION);
  loop_rearm();visible=1;frame_tick=s_wait_tick-1;step(10);CHECK(s_phase==MAT_WAIT && close_calls==before+1);
 }
 ArmMaterial_Command("arm material loop status");CHECK(strstr(console,"loop=1 color=1 rounds=3 limit=0")!=NULL);
 ArmMaterial_Command("arm material loop start");CHECK(s_completed==3 && s_phase==MAT_WAIT);
 CHECK(ArmMaterial_Start()==HAL_BUSY);ArmMaterial_Command("arm material loop limit 1");CHECK(!s_loop_limit);
 ArmMaterial_Command("arm material loop stop");CHECK(!s_loop && s_phase==MAT_FAILED);
 setup();ArmMaterial_Command("arm material loop limit 2");loop_ready();placed();loop_rearm();placed();
 CHECK(!s_loop && s_phase==MAT_DONE && s_completed==2 && !camera_calls && ArmMaterial_ReadResult(&success) && success);
 setup();loop_ready();placed();peer.request_seq=15;loop_rearm();CHECK(!peer.request_seq);
 /* All batch slots and all six colors follow the same serialised grasp. */
 for(j=1;j<=6;j++){
  uint8_t slot=(uint8_t)((j-1)%3+1);setup();CHECK(ArmMaterial_StartToSlotColor(slot,(uint8_t)j)==HAL_OK);
  step(10);CHECK(peer.pending && peer.target==j);peer.pending=0;peer.confirmed=1;peer.result=0;step(10);
  pose_active=0;pose_success=1;grip.ramping=0;lift_pct=s_wait_pct;reach_pct=s_reach_wait_pct;dm_angle=ARM_MATERIAL_DM_DEG;step(10);
  image_frame((uint8_t)(j%6+1),1,0,-20,0);step(10);CHECK(s_phase==MAT_WAIT && !move_calls);
  placed();CHECK(s_phase==MAT_DONE && !camera_calls && close_calls==1 && reach_calls==1 && selected_slot==slot);
 }
 setup();CHECK(ArmMaterial_StartToSlot(0)==HAL_ERROR && ArmMaterial_StartToSlot(4)==HAL_ERROR);
 CHECK(ArmMaterial_StartToSlotColor(1,7)==HAL_ERROR && ArmMaterial_StartToSlotColor(1,0)==HAL_ERROR);
 /* STOP, peer faults, actual drive faults retain their existing ownership. */
 for(phase=MAT_WAIT;phase<=MAT_PLACE;phase++){
  reach_phase(phase);ArmMaterial_Command("arm material loop stop");CHECK(s_phase==MAT_FAILED && all_stops);
  reach_phase(phase);peer.peer_age_ms=VISION_CAMERA_PEER_TIMEOUT_MS+1U;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
  reach_phase(phase);axis_bad=1;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
  reach_phase(phase);dm_bad=1;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
  reach_phase(phase);chassis=1;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
 }
 setup();reach_phase(MAT_PLACE);rotation_on=0;dm_disabled=1;dm_stale=axis_stale=1;step(2000);
 CHECK(s_phase==MAT_PLACE && !all_stops);pose_active=0;pose_success=0;step(10);CHECK(s_phase==MAT_FAILED);
 /* Existing ID6 trial policy clears recurring stall without new moves. */
 for(phase=MAT_WAIT;phase<=MAT_RAISE;phase++){
  reach_phase(phase);if(phase==MAT_RAISE)lift_pct=s_raise_start_percent*0.75f;
  if(phase==MAT_GRAB)reach_pct=s_reach_wait_pct+ARM_PERCENT_TOLERANCE+0.1f;
  reach_only_stall=1;before=reach_calls;
  for(j=0;j<11;j++)step(150);
  CHECK(clear_reach_calls>=10 && !all_stops && reach_calls==before && !start_retry_calls);
  reach_only_stall=0;before=clear_reach_calls;step(150);CHECK(clear_reach_calls==before);
 }
 setup();loop_ready();reach_only_stall=reach_only_disabled=1;step(10);CHECK(!all_stops && enable_reach_calls==1);
 reach_only_stall=reach_only_disabled=0;step(150);CHECK(!all_stops && !s_reach_clear_pending);
 setup();loop_ready();reach_only_stall=1;clear_busy=1;step(10);CHECK(!all_stops && !enable_reach_calls);
 setup();loop_ready();reach_only_stall=1;clear_error=1;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
 setup();loop_ready();reach_only_protected=1;step(10);CHECK(s_phase==MAT_FAILED);
 /* No wait-pose motion until the newly requested CAM1 ACK is accepted. */
 setup();ArmMaterial_Command("arm material loop start");step(10);CHECK(!wait_pose_calls && !move_calls);
 peer.pending=0;peer.result=VISION_RESULT_TIMEOUT;step(10);CHECK(s_phase==MAT_FAILED && !wait_pose_calls);
 setup();ArmMaterial_Command("arm material loop start");step(10);step(2600);CHECK(s_phase==MAT_FAILED && !wait_pose_calls);
 setup();loop_ready();peer.request_seq++;step(10);
 CHECK(s_phase==MAT_WAIT && s_camera_recovering && peer.pending && !all_stops);
 setup();CHECK(ArmMaterial_Start()==HAL_OK);step(10);
 peer.pending=0;peer.confirmed=1;peer.result=0;step(10);
 CHECK(s_phase==MAT_POSITION && s_wait_pose_started && wait_pose_calls==1);
 peer.confirmed=0;step(10);CHECK(s_camera_recovering && !all_stops);
 pose_active=0;pose_success=1;grip.ramping=0;
 lift_pct=s_wait_pct;reach_pct=s_reach_wait_pct;dm_angle=s_wait_pose[2];
 peer.pending=0;peer.confirmed=1;peer.result=0;peer.peer_age_ms=0;step(10);step(10);
 CHECK(s_phase==MAT_WAIT && wait_pose_calls==1 && !close_calls);
 /* Real reconnect's initial STOP heartbeat must cause re-SET, never grab
  * on the old pixel or replay the taught wait pose. Preserve remaining23s. */
 setup();loop_ready();step(20000);
 {uint32_t wait_elapsed=tick-s_wait_tick,poses=wait_pose_calls,modes=mode_calls;
  peer.confirmed=0;peer.mode=0;peer.peer_age_ms=0;step(10);
  CHECK(s_camera_recovering && peer.pending && mode_calls==modes+1 && !all_stops);
  image_frame(1,1,0,-20,0);step(2000);
  CHECK(s_phase==MAT_WAIT && !close_calls && !reach_calls && wait_pose_calls==poses);
  peer.pending=0;peer.confirmed=1;peer.result=0;peer.peer_age_ms=0;step(10);
  CHECK(!s_camera_recovering && s_phase==MAT_WAIT && !visible && !close_calls);
  CHECK(tick-s_wait_tick==wait_elapsed+10U && wait_pose_calls==poses);
  step(10);CHECK(s_phase==MAT_WAIT && !reach_calls);
  image_frame(1,1,0,-20,0);step(10);CHECK(s_phase==MAT_GRAB && reach_calls==1);
 }
 /* A failed/rejected ACK during reconnection is retried within the same8s,
  * without extending the recovery deadline or the actuator target. */
 setup();loop_ready();peer.confirmed=0;step(10);
 {uint32_t recovery=s_camera_recover_tick,modes=mode_calls;
  peer.pending=0;peer.result=7;step(10);step(10);
  CHECK(s_camera_recovering && s_camera_recover_tick==recovery && mode_calls==modes+1 && !all_stops);
  peer.pending=0;peer.result=0;peer.confirmed=1;peer.peer_age_ms=0;step(10);
  CHECK(!s_camera_recovering && s_phase==MAT_WAIT && !close_calls);
 }
 setup();loop_ready();peer.confirmed=0;step(10);step(7999);
 CHECK(s_camera_recovering && s_phase==MAT_WAIT && !all_stops);
 step(1);CHECK(s_phase==MAT_FAILED && all_stops);
 setup();loop_ready();peer.peer_age_ms=7999;step(10);CHECK(s_phase==MAT_WAIT && !all_stops);
 peer.peer_age_ms=8000;step(10);CHECK(s_phase==MAT_FAILED && all_stops);
 setup();loop_ready();peer.confirmed=0;step(10);ArmMaterial_AbortState();step(10);
 CHECK(s_phase==MAT_FAILED && !close_calls); /* Manual STOP never resumes. */
 setup();loop_ready();dm_snapshot_delay=3;step(10);CHECK(s_phase==MAT_WAIT && !all_stops);dm_snapshot_delay=0;
 /* Color screen starts only AFTER actual CAMERA READY and a NEW matching
  * calibration SET/ACK. No grab or other material job during UI. */
 {unsigned target;for(target=1;target<=6;target++) {
   char cmd[24];uint32_t before;setup();snprintf(cmd,sizeof(cmd),"arm color %u",target);
   CHECK(ArmMaterial_Command(cmd) && s_color_phase==COLOR_CAMERA && ArmMaterial_IsActive() && camera_calls==1 && !mode_calls);
   step(1000);CHECK(s_color_phase==COLOR_CAMERA && !mode_calls);
   camera_arm=ARM_CAMERA_DONE;step(10);CHECK(s_color_phase==COLOR_SET && !mode_calls);
   step(10);CHECK(mode_calls==1 && peer.target==target && peer.flags==VISION_MATERIAL_CALIBRATION_FLAG);
   step(10);CHECK(s_color_phase==COLOR_SET); /* pending cannot be mistaken for READY */
   peer.pending=0;peer.confirmed=1;peer.result=0;step(10);
   CHECK(s_color_phase==COLOR_UI && ArmMaterial_IsActive() && clear_calls==1);
   ArmMaterial_Command("arm color");CHECK(camera_calls==1 && s_color_phase==COLOR_UI); /* repeated start never replaces the session */
   before=mode_calls;CHECK(ArmMaterial_Start()==HAL_BUSY && mode_calls==before);
   image_frame((uint8_t)target,1,0,-30,0);step(1000);CHECK(!close_calls && !move_calls && s_color_phase==COLOR_UI);
   ArmMaterial_Command("arm color status");CHECK(strstr(console,"phase=3 active=1")!=NULL);
   ArmMaterial_Command("arm color stop");CHECK(!ArmMaterial_IsActive() && s_color_phase==COLOR_OFF && stop_calls && all_stops);
   step(1000);CHECK(!mode_calls || mode_calls==before);
 }}
 setup();ArmMaterial_Command("arm color");CHECK(s_learn_target==1 && s_color_phase==COLOR_CAMERA);
 camera_arm=ARM_CAMERA_FAILED;step(10);CHECK(s_color_phase==COLOR_FAILED && !ArmMaterial_IsActive() && all_stops && !mode_calls);
 setup();ArmMaterial_Command("arm color");ArmMaterial_AbortState();step(1000);CHECK(s_color_phase==COLOR_FAILED && !mode_calls);
 setup();ArmMaterial_Command("arm color");camera_arm=ARM_CAMERA_DONE;step(10);step(10);
 peer.pending=0;peer.confirmed=1;peer.result=7;step(10);CHECK(s_color_phase==COLOR_FAILED && all_stops);
 setup();ArmMaterial_Command("arm color");camera_arm=ARM_CAMERA_DONE;step(10);step(10);
 step(2600);CHECK(s_color_phase==COLOR_FAILED && !ArmMaterial_IsActive());
 setup();ArmMaterial_Command("arm color");camera_arm=ARM_CAMERA_DONE;step(10);step(10);
 peer.pending=0;peer.confirmed=1;peer.result=0;peer.flags=0;step(10);CHECK(s_color_phase==COLOR_FAILED);
 setup();chassis=1;ArmMaterial_Command("arm color");CHECK(!camera_calls && !ArmMaterial_IsActive());
 setup();reject_camera=1;ArmMaterial_Command("arm color");CHECK(!ArmMaterial_IsActive() && !mode_calls);
 setup();ArmMaterial_Command("arm color 0");ArmMaterial_Command("arm color 7");ArmMaterial_Command("arm color -1");ArmMaterial_Command("arm color 2 junk");CHECK(!camera_calls);
 setup();loop_ready();ArmMaterial_Command("arm color");CHECK(s_phase==MAT_WAIT && !camera_calls && !Mat_ColorActive());
 {unsigned mode;for(mode=0;mode<4;mode++) {
   setup();ArmMaterial_Command("arm color");camera_arm=ARM_CAMERA_DONE;step(10);step(10);
   peer.pending=0;peer.confirmed=1;peer.result=0;step(10);CHECK(s_color_phase==COLOR_UI);
   if(mode==0)peer.peer_age_ms=VISION_CAMERA_PEER_TIMEOUT_MS+1U;else if(mode==1)peer.request_seq++;
   else if(mode==2)chassis=1;else ArmMaterial_Command("arm material stop");
   step(10);CHECK(s_color_phase==COLOR_FAILED && !ArmMaterial_IsActive() && stop_calls && all_stops && !close_calls);
 }}
 return failures;
}
'''
with tempfile.TemporaryDirectory(prefix="arm-material-tests-") as temp:
    work = Path(temp)
    c = work / "test.c"
    c.write_text(prefix + syscalls + block + suffix, encoding="utf-8")
    linker = work / "test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary = work / "test.elf"
    gcc = shutil.which("arm-none-eabi-gcc")
    includes = [root / "tools/tests", root / "Core/Inc", root / "Drivers/STM32H7xx_HAL_Driver/Inc", root / "Drivers/CMSIS/Device/ST/STM32H7xx/Include", root / "Drivers/CMSIS/Include"]
    subprocess.run([gcc,"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror","-DSTM32H723xx","-DUSE_HAL_DRIVER",*["-I"+str(x) for x in includes],"-ffunction-sections","-fdata-sections",str(c), str(root/"Core/Src/screen_metrics.c"),"-nostartfiles","--specs=nosys.specs","-Wl,--gc-sections","-Wl,-T,"+str(linker),"-lm","-o",str(binary)],check=True)
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open("rb") as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD" and segment["p_filesz"]:emu.mem_write(segment["p_vaddr"],segment.data())
        symbols = {s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()};entry=elf["e_entry"]
    sentinel=0x080FF000;completed=[]
    def returned(machine,address,size,user_data):completed.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=20_000_000)
    def value(name):return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"Material FSM: {value('checks')} checks, {value('failures')} failures; first line={value('first_failure')}")
    if value('failures'):
        lines=emu.mem_read(symbols['failed_lines'],min(value('failures'),32)*4)
        print('Failed C lines:',[int.from_bytes(lines[i:i+4],'little') for i in range(0,len(lines),4)])
    if not completed or not value("checks") or value("failures"):
        (root/"tmp/material_failed.c").write_text(c.read_text(encoding="utf-8"),encoding="utf-8")
        raise SystemExit("Material FSM failed")
