"""Read-only diagnostic FSM and reply demultiplexing, using production C."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
block = (root / "Core/Src/arm_start_pose.c").read_text(encoding="utf-8")
rx = ""
syscalls = (root / "tools/tests/test_arm_driver.c").read_text(encoding="utf-8")
syscalls = syscalls[syscalls.index("int _close("):syscalls.index("volatile unsigned int test_checks")]
prefix = r'''
#include "arm_start_pose.h"
#include "arm_servo.h"
#include "arm_calibration.h"
#include "arm_task.h"
#include "w25q64.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure;
volatile uint32_t failure_lines[32];
#define CHECK(x) do{checks++;if(!(x)){if(failures<32)failure_lines[failures]=__LINE__;failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static uint32_t tick=1000,dm_moves,linear_moves,flash_writes,stops,flash_reads;
static uint32_t test_pose_feedback_gap;
static int64_t expected_reach=-50871;
/* Expected PICK/PLACE targets are independent fixture values. Generic,
 * START and ring fixtures keep their explicitly taught expected_reach. */
static uint8_t s_pick,s_place_slot;
static char last_console[192];
static char grip_finish_failure[128];
static uint8_t enabled,read_fail,flash_fail,chassis,ref_valid=1;
static uint8_t raw_guard_reject;
HAL_StatusTypeDef ArmDriver_CheckRawRotationTarget(float raw){(void)raw;return raw_guard_reject?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef ArmDriver_CheckUiPose(float drop,float angle){return drop>150 && (angle< -44 || angle>44)?HAL_ERROR:HAL_OK;}
static float dm_target,zero_ref,grip_angle;
static float close_dm_expected=162.8250f;
static int64_t close_lift_expected=-87486; /* Tray PICK raised12mm from V84 (V142 lower3mm); ring retrieval overrides below. */
static arm_servo_status_t grip_status,table_status;
static uint8_t ring_overlap_test;
static uint8_t grip_open_confirmed;
uint8_t ArmServo_GripOpeningOrOpen(void){return grip_status.enabled && !grip_status.fault && grip_angle==225;}
uint8_t ArmServo_GripOpenConfirmed(void){return grip_open_confirmed && grip_angle==225 && grip_status.enabled && !grip_status.ramping && !grip_status.fault;}
static uint32_t grip_open_calls,grip_close_calls;
static uint8_t native_inflight,parallel_seen,place_test,follow_dm=1U,lag_on_native,drift_on_stop;
static uint32_t slow_corrections,lift_commands,lift_commands_before_pick,encoder_reach_moves;
static uint8_t pick_clearance_seen;
static float dm_speed_dps;
static uint32_t soft_calls,dm_snapshot_delay;
static uint8_t rotation_plant;
static uint8_t retry_recovering,retry_failed,stall_pending,cache_active;
static uint8_t reach_stall;
static uint32_t retry_generation;
static int32_t cached_abs_goal;
static uint32_t track_corrections,reach_stop_calls;
static uint8_t tray_start_test,tray_replay_busy,tray_replay_error;
static uint32_t tray_replay_calls,tray_ack_fence;
static uint16_t cached_reach_speed;
static uint8_t cached_reach_acc;
static arm_cal_position_ack_t tray_ack;
static HAL_StatusTypeDef track_result=HAL_OK;
static uint8_t track_follows=1U;
uint8_t ArmDriver_ReachRetryRecovering(void){return retry_recovering;}
uint8_t ArmDriver_ReachRetryActive(void){return cache_active;}
uint8_t ArmDriver_ReachStallPending(void){return stall_pending;}
uint8_t ArmDriver_ReachRetryFailed(void){return retry_failed;}
uint32_t ArmDriver_ReachRetryGeneration(void){return retry_generation;}
uint32_t ArmDriver_ReachRetryTimeoutMs(double counts){(void)counts;return 20000U;}
void ArmDriver_CancelReachRetry(void){retry_recovering=retry_failed=stall_pending=cache_active=0;}
static arm_cal_state_t cal_state=ARM_CAL_IDLE;
static uint32_t lift_holds;
static uint8_t reject_lift_hold;
HAL_StatusTypeDef ArmCalibration_HoldLiftForReachRecovery(void){lift_holds++;if(reject_lift_hold)return HAL_ERROR;cal_state=ARM_CAL_IDLE;return HAL_OK;}
static float cal_lift_target;
static arm_driver_axis_t cal_axis;
uint8_t ArmCalibration_IsLiftRaisingToClearance(void){return (cal_state==ARM_CAL_PERCENT_MOVE || cal_state==ARM_CAL_PERCENT_SETTLE) && cal_axis==ARM_DRIVER_LIFT && fabsf(cal_lift_target-ArmCalibration_TransferPercent(240))<0.001f;}
uint8_t ArmCalibration_IsLiftRaisingToTop(void){return (cal_state==ARM_CAL_PERCENT_MOVE || cal_state==ARM_CAL_PERCENT_SETTLE) && cal_axis==ARM_DRIVER_LIFT && cal_lift_target==0;}
static arm_cal_axis_status_t axes[2];
static dm_motor_t motor;
HAL_StatusTypeDef ArmServo_TestAngleDeg(arm_servo_id_t id,float angle){
 CHECK(id==ARM_SERVO_GRIP);grip_angle=angle;grip_status.enabled=1;grip_status.ramping=0;
 if(angle==225)grip_open_calls++;
 if(angle==ARM_SERVO_GRIP_CLOSE_DEG){grip_close_calls++;CHECK(fabsf(dm_target-close_dm_expected)<0.001f);
  if(ring_overlap_test){float actual;CHECK(ArmDriver_GetRotationDeg(&actual)==HAL_OK && fabsf(actual-close_dm_expected)<=1.001f);
   CHECK(fabs((double)(axes[0].position_counts-axes[0].zero_counts-close_lift_expected))*axes[0].travel_mm/fabs(ARM_LIFT_FULL_TRAVEL_COUNTS)<=3.001);grip_status.ramping=1;}
  else CHECK(llabs(axes[0].position_counts-axes[0].zero_counts-close_lift_expected)<=2);}
 return HAL_OK;
}
void ArmServo_GetStatus(arm_servo_id_t id,arm_servo_status_t *out){*out=id==ARM_SERVO_TABLE?table_status:grip_status;}
uint8_t ArmServo_GripIsClosed(void){return grip_status.enabled && !grip_status.ramping && !grip_status.fault && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG;}
uint8_t ArmServo_GripIsOpen(void){return grip_status.enabled && !grip_status.ramping && !grip_status.fault && grip_angle==ArmServo_GripOpenDeg();}
HAL_StatusTypeDef ArmServo_Stop(arm_servo_id_t id){(void)id;grip_status.ramping=0;return HAL_OK;}

static uint8_t flash[65536];
static uint8_t flash_busy;
uint8_t W25Q64_StorageTryLock(void){if(flash_busy)return 0;flash_busy=1;return 1;}
void W25Q64_StorageUnlock(void){flash_busy=0;}
static float servo_parameters[5]={225,308,205,295,25};
float ArmServo_GripOpenDeg(void){return servo_parameters[0];}
float ArmServo_GripCloseDeg(void){return servo_parameters[1];}
void ArmServo_PrintStatus(void){}
HAL_StatusTypeDef ArmServo_GetUiParameters(float out[5]){memcpy(out,servo_parameters,sizeof(servo_parameters));return HAL_OK;}
HAL_StatusTypeDef ArmServo_SetUiParameters(const float values[5]){memcpy(servo_parameters,values,sizeof(servo_parameters));return HAL_OK;}
static HAL_StatusTypeDef rotate_result=HAL_OK;
uint32_t HAL_GetTick(void){return tick;}
uint8_t ArmCalibration_WaitForFeedback(arm_feedback_wait_t *w,uint32_t now,uint32_t *gap){(void)w;(void)now;*gap=test_pose_feedback_gap;test_pose_feedback_gap=0;return 0;}
void ZDT_Console_Print(const char *line){CHECK(line!=NULL);strncpy(last_console,line,sizeof(last_console)-1);if(strncmp(line,"grip finish:",12)==0 || strncmp(line,"placement reach",15)==0){strncpy(grip_finish_failure,line,sizeof(grip_finish_failure)-1);}}
uint8_t ChassisMotor_IsEnabled(void){return chassis;}
uint8_t ChassisMotor_HasMotionDemand(void){return chassis;}
uint8_t ArmTask_IsEnabled(void){return enabled;}
uint8_t ArmTask_IsFaulted(void){return 0;}
dm_motor_t *ArmTask_GetMotor(void){return &motor;}
arm_cal_state_t ArmCalibration_GetState(void){return cal_state;}
arm_camera_state_t ArmCalibration_GetCameraState(void){return ARM_CAMERA_FAILED;}
void ArmCalibration_ClearCameraReady(void){}
HAL_StatusTypeDef ArmCalibration_Cancel(void){cal_state=ARM_CAL_IDLE;ArmStartPose_Cancel();return HAL_OK;}
void ArmCalibration_GetAxisStatus(arm_driver_axis_t axis,arm_cal_axis_status_t *out){*out=axes[axis==ARM_DRIVER_REACH];if(axis==ARM_DRIVER_REACH && reach_stall)out->flags|=ARM_CAL_FLAG_STALL;}
HAL_StatusTypeDef ArmCalibration_GetPercent(arm_driver_axis_t axis,float *p){unsigned i=axis==ARM_DRIVER_REACH;*p=(float)(axes[i].position_counts-axes[i].zero_counts)/(float)(i?ARM_REACH_FULL_TRAVEL_COUNTS:ARM_LIFT_FULL_TRAVEL_COUNTS)*100;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_GetSpan(arm_driver_axis_t axis,float *span){*span=(float)(axis==ARM_DRIVER_REACH?ARM_REACH_FULL_TRAVEL_COUNTS:ARM_LIFT_FULL_TRAVEL_COUNTS);return HAL_OK;}
HAL_StatusTypeDef ArmDriver_GetLinearDirection(arm_driver_axis_t axis,uint8_t *reverse){(void)axis;*reverse=0;return HAL_OK;}
HAL_StatusTypeDef ArmCalibration_MovePercent(arm_driver_axis_t axis,float p){
 unsigned i=axis==ARM_DRIVER_REACH;cal_axis=axis;if(!i)cal_lift_target=p;linear_moves++;if(i)encoder_reach_moves++;
 if(!i){lift_commands++;if(fabs(p-(-51.0140704f/360.0f*65536.0f/ARM_LIFT_FULL_TRAVEL_COUNTS*100.0f))<0.01f)pick_clearance_seen=1;}
 if(place_test && !i && p>1){CHECK(fabsf(motor.fb.pos_rad-dm_target/57.2957795f-zero_ref)<0.01f);}
 if(i || !ring_overlap_test)axes[i].position_counts=axes[i].zero_counts+(int64_t)round((double)p*(i?ARM_REACH_FULL_TRAVEL_COUNTS:ARM_LIFT_FULL_TRAVEL_COUNTS)/100);
 cal_state=ARM_CAL_PERCENT_MOVE;return HAL_OK;
}
uint16_t ArmDriver_GetLinearSpeed(arm_driver_axis_t axis){return axis==ARM_DRIVER_REACH?60U:120U;}
uint16_t ArmDriver_ReachMotionSpeedRpm(void){return ArmServo_GripIsClosed()?246U:182U;}
HAL_StatusTypeDef ArmCalibration_GetAbsoluteReachPulses(float pct,int32_t *p){
 *p=(int32_t)round(-(axes[1].zero_counts+(double)pct*ARM_REACH_FULL_TRAVEL_COUNTS/100.0)*ARM_CAMERA_POSITION_STEPS_REV/65536.0);return HAL_OK;
}
HAL_StatusTypeDef ArmDriver_ReachAbsolutePulses(int32_t pulses,uint16_t speed,uint8_t acc){
 CHECK((speed==60U || speed==246U || speed==182U || speed==18U) && acc==ARM_CAMERA_POSITION_ACCEL);linear_moves++;native_inflight=cache_active=1;cached_abs_goal=pulses;
 cached_reach_speed=speed;cached_reach_acc=acc;
 if(s_pick==1U)expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
 if(s_pick==2U)expected_reach=axes[1].zero_counts+(int64_t)round(1.0*65536.0/360.0);
 if(place_test && fabsf(motor.fb.pos_rad-dm_target/57.2957795f-zero_ref)>0.01f)parallel_seen=1;
 if(tray_start_test)return HAL_OK;
 axes[1].position_counts=-(int64_t)round((double)pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV);
 CHECK(llabs(axes[1].position_counts-expected_reach)<=12); /*3200p/rev quantization is20.48encoder counts/pulse.*/
 if(speed==18U)slow_corrections++;
 if(lag_on_native){axes[1].position_counts-=1800;lag_on_native=0;}return HAL_OK;
}
uint32_t ArmCalibration_PositionAckFence(arm_driver_axis_t axis){CHECK(axis==ARM_DRIVER_REACH);return tray_ack_fence;}
void ArmCalibration_GetPositionAck(arm_driver_axis_t axis,arm_cal_position_ack_t *out){CHECK(axis==ARM_DRIVER_REACH);*out=tray_ack;}
HAL_StatusTypeDef ArmDriver_ReplayReachAbsolutePulses(int32_t pulses){
 CHECK(cache_active && pulses==cached_abs_goal && cached_reach_acc==ARM_CAMERA_POSITION_ACCEL);
 if(tray_replay_busy){tray_replay_busy--;return HAL_BUSY;}if(tray_replay_error)return HAL_ERROR;
 tray_replay_calls++;if(!tray_start_test)axes[1].position_counts=-(int64_t)round((double)pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV);
 return HAL_OK;
}
HAL_StatusTypeDef ArmDriver_ReassertReachAbsolutePulses(int32_t pulses,uint16_t speed){CHECK(pulses==cached_abs_goal && speed==18 && cache_active);if(track_result!=HAL_OK)return track_result;track_corrections++;retry_generation++;if(track_follows)axes[1].position_counts=-(int64_t)round((double)pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV);return HAL_OK;}
static HAL_StatusTypeDef reach_stop_result=HAL_OK;
static HAL_StatusTypeDef MockReachStop(arm_driver_axis_t axis){CHECK(axis==ARM_DRIVER_REACH);reach_stop_calls++;if(reach_stop_result!=HAL_OK)return reach_stop_result;native_inflight=0;if(drift_on_stop){axes[1].position_counts-=1800;drift_on_stop=0;}return HAL_OK;}
HAL_StatusTypeDef ArmDriver_Stop(arm_driver_axis_t axis){cache_active=0;return MockReachStop(axis);}
HAL_StatusTypeDef ArmDriver_StopReachForHold(void){return MockReachStop(ARM_DRIVER_REACH);}
HAL_StatusTypeDef ArmDriver_Enable(arm_driver_axis_t axis){if(axis==ARM_DRIVER_ROTATION){if(!enabled)dm_target=(motor.fb.pos_rad-zero_ref)*57.2957795f;enabled=1;motor.fb.enabled=1;motor.fb.err_code=1;}else axes[axis==ARM_DRIVER_REACH].flags|=ARM_CAL_FLAG_ENABLED;return HAL_OK;}
HAL_StatusTypeDef ArmDriver_Disable(arm_driver_axis_t axis){CHECK(axis==ARM_DRIVER_ROTATION);enabled=0;motor.fb.enabled=0;return HAL_OK;}
HAL_StatusTypeDef ArmDriver_StopAll(void){stops++;enabled=0;return HAL_OK;}
HAL_StatusTypeDef ArmDriver_RotateToDeg(float d){
 dm_moves++;dm_target=d;dm_speed_dps=112.5f;return rotate_result;
}
float ArmDriver_RotationSpeedDps(void){return dm_speed_dps;}
HAL_StatusTypeDef ArmDriver_RotateToDegAtSpeed(float d,float speed){
 dm_moves++;dm_target=d;dm_speed_dps=speed;soft_calls++;return rotate_result;
}
uint8_t ArmDriver_IsRotationReferenceValid(void){return ref_valid;}
HAL_StatusTypeDef ArmDriver_GetRotationZeroRaw(float *z){*z=zero_ref;return ref_valid?HAL_OK:HAL_ERROR;}
HAL_StatusTypeDef ArmDriver_MapRawRotation(float raw,float *angle){*angle=(raw-zero_ref)*57.2957795f;return ref_valid?HAL_OK:HAL_ERROR;}
uint8_t ArmMaterial_IsActive(void){return 0U;}
HAL_StatusTypeDef ArmDriver_GetRotationDeg(float *angle){return ArmDriver_MapRawRotation(motor.fb.pos_rad,angle);}
void DM_GetFeedback(const dm_motor_t *m,dm_feedback_t *out){if(dm_snapshot_delay){tick+=dm_snapshot_delay;motor.fb.last_tick=tick;motor.fb.fb_count++;}*out=m->fb;}
HAL_StatusTypeDef DM_Motor_ReadPosition(dm_motor_t *m,uint32_t timeout,dm_feedback_t *out){CHECK(timeout==300);*out=m->fb;return read_fail?HAL_TIMEOUT:HAL_OK;}
w25q64_status_t W25Q64_Init(w25q64_id_t *id){(void)id;return W25Q64_OK;}
uint32_t W25Q64_GetCapacity(void){return 8U*1024U*1024U;}
static uint32_t at(uint32_t address,uint32_t length){uint32_t base=W25Q64_GetCapacity()-16U*4096U;CHECK(address>=base && address+length<=base+sizeof(flash));return address-base;}
w25q64_status_t W25Q64_Read(uint32_t addr,uint8_t *data,uint32_t len){flash_reads++;memcpy(data,flash+at(addr,len),len);return W25Q64_OK;}
w25q64_status_t W25Q64_EraseSector(uint32_t addr){memset(flash+at(addr,4096),255,4096);return W25Q64_OK;}
w25q64_status_t W25Q64_WriteVerified(uint32_t addr,const uint8_t *data,uint32_t len,uint8_t *verify,uint32_t cap){flash_writes++;CHECK(cap>=len);memcpy(flash+at(addr,len),data,flash_fail?len/2:len);memcpy(verify,flash+at(addr,len),len);return flash_fail?W25Q64_ERROR_VERIFY:W25Q64_OK;}
'''
suffix = r'''
static void step(void){unsigned i;tick+=100;for(i=0;i<2;i++){axes[i].position_tick=axes[i].flags_tick=tick;axes[i].position_sequence++;axes[i].flags_sequence++;}
 motor.fb.last_tick=tick;motor.fb.fb_count++;
 if(enabled && follow_dm){
   float target=dm_target/57.2957795f+zero_ref;
   if(rotation_plant){float delta=target-motor.fb.pos_rad,limit=dm_speed_dps/57.2957795f*0.1f;
     if(fabsf(delta)<=limit){motor.fb.pos_rad=target;motor.fb.vel_rads=0;}
     else{motor.fb.pos_rad+=delta>0?limit:-limit;motor.fb.vel_rads=delta>0?dm_speed_dps/57.2957795f:-dm_speed_dps/57.2957795f;}
   } else motor.fb.pos_rad=target;
 }
 ArmStartPose_Poll();}
static void finish(void){unsigned i;for(i=0;i<25 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}CHECK(!ArmStartPose_IsActive() && ArmStartPose_IsReady());}
static int test_tray_gate(void)
{
 unsigned mode;uint32_t lifts;
 for(mode=0;mode<8;mode++) {
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;s_phase=POSE_REACH;s_pick=1;
   s_run.percent[0]=40;s_run.span[0]=ARM_LIFT_FULL_TRAVEL_COUNTS;
   s_tray_gate=1;s_tray_gate_target=2139;s_tray_gate_tick=mode==7?UINT32_MAX-100U:1000U;
   memset(&table_status,0,sizeof(table_status));table_status.enabled=1;table_status.target_us=2139;
   lifts=lift_commands;
   Pose_BeginFinalDescent(s_tray_gate_tick+ARM_SERVO_TABLE_MIN_WAIT_MS-1U);
   CHECK(s_phase==POSE_REACH && s_tray_gate && lift_commands==lifts);
   if(mode==1)table_status.ramping=1;
   if(mode==2)table_status.target_us=1639;
   if(mode==3)table_status.fault=1;
   if(mode==4)table_status.enabled=0;
   if(mode==5)table_status.ramping=1;
   Pose_BeginFinalDescent(s_tray_gate_tick+(mode==5?5001U:ARM_SERVO_TABLE_MIN_WAIT_MS));
   if(mode==1) {
     CHECK(s_phase==POSE_REACH && lift_commands==lifts);
     table_status.ramping=0;Pose_BeginFinalDescent(s_tray_gate_tick+700U);
   }
   if(mode>=2 && mode<=5)CHECK(s_phase==POSE_FAILED && lift_commands==lifts);
   else CHECK(s_phase==POSE_LIFT && !s_tray_gate && lift_commands==lifts+1U);
 }
 return failures;
}
static int test_photo_targets(void)
{
 unsigned i,area,layer;float reach,dm,lift0,lift72;
 const float want_reach[2][3]={{-545.2084f,-212.0919f,-290.4869f},{-545.2084f,-212.0919f,-290.4869f}};
 const float want_dm[2][3]={{41.3044f,4.6040f,-39.5218f},{41.3044f,4.6040f,-39.5218f}};
 memset(axes,0,sizeof(axes));
 for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
 motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;
 for(area=0;area<2;area++)for(i=0;i<3;i++) {
   CHECK(Pose_RingDropTargets(i+1,area,&reach,&dm)==HAL_OK);
   CHECK(fabsf(reach-want_reach[area][i])<0.02f && fabsf(dm-want_dm[area][i])<0.002f);
   /* Public entry points must use the same profile, including TEMP72mm. */
   lift0=0;
   for(layer=0;layer<(area?2U:1U);layer++) {
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;ArmStartPose_Init();s_pick_completed=1;
     grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.fault=grip_status.ramping=0;
     CHECK((area?ArmStartPose_BeginTempRingDrop(i+1,layer?72.0f:0.0f):ArmStartPose_BeginRingDrop(i+1,0.0f))==HAL_OK);
     CHECK(s_pick==3U && s_ring_temp==area && s_phase==POSE_TOP_ENABLE);
     CHECK(fabsf(s_run.dm_deg-want_dm[area][i])<0.002f);
     reach=s_run.percent[1]*s_run.span[1]/100.0f*360.0f/65536.0f;
     CHECK(fabsf(reach-want_reach[area][i])<0.02f);
     lift72=s_run.percent[0]*240.0f/100.0f;
     if(!layer)lift0=lift72;else CHECK(fabsf((lift0-lift72)-72.0f)<0.02f);
   }
   /* Retrieval must use the exact shared ground release profile and saved
    * trim, rather than the old independent lower15mm coordinates. */
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
   s_ring_trim[i][0]=1.25f;s_ring_trim[i][1]=-2.0f;s_ring_trim[i][2]=0.3f;
   s_pick_completed=1;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
   CHECK(ArmStartPose_BeginRingDrop(i+1,0)==HAL_OK);
   {start_profile_t placed=s_run;
    ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;
    CHECK(ArmStartPose_BeginRingPick(i+1)==HAL_OK);
    CHECK(fabsf(s_run.percent[0]-placed.percent[0])<0.0001f);
    CHECK(fabsf(s_run.percent[1]-placed.percent[1])<0.0001f && fabsf(s_run.dm_deg-placed.dm_deg)<0.0001f);
   }
   ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_IDLE;
 }
 CHECK(Pose_RingDropTargets(0,0,&reach,&dm)==HAL_ERROR);
 CHECK(Pose_RingDropTargets(4,1,&reach,&dm)==HAL_ERROR);
 return failures;
}
static int test_fast_transfer(void)
{
 unsigned i,ring,kind,slot;uint32_t before;
 memset(axes,0,sizeof(axes));
 for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
 motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;motor.fb.enabled=1;motor.fb.err_code=1;enabled=1;
 for(kind=0;kind<2;kind++) {
  ArmStartPose_Init();cal_state=ARM_CAL_IDLE;enabled=1;motor.fb.enabled=1;motor.fb.pos_rad=zero_ref;
  s_run=s_saved;s_run.dm_deg=0;s_pick=kind?1:0;s_phase=POSE_DM;s_tick=s_started=tick;
  s_dm_seq=motor.fb.fb_count;motor.fb.fb_count++;before=tick;
  ArmStartPose_Poll();
  CHECK(kind?s_phase==POSE_REACH_ENABLE:s_phase==POSE_DM);CHECK(tick==before);
 }
 /* Every source ring returns at the common DM speed to every tray slot. */
 for(ring=1;ring<=3;ring++)for(slot=1;slot<=3;slot++) {
  ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_PERCENT_MOVE;cal_axis=ARM_DRIVER_LIFT;cal_lift_target=0;
  axes[0].position_counts=(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*.4);axes[1].position_counts=0;
  axes[0].position_tick=axes[1].position_tick=axes[0].flags_tick=axes[1].flags_tick=tick;
  s_pick=4;s_ring_station=ring;s_phase=POSE_READY;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=grip_status.fault=0;dm_speed_dps=162;
  CHECK(ArmStartPose_BeginTrayPlaceDuringRaise((uint8_t)slot,80)==HAL_OK);
  CHECK(fabsf(Pose_DmSpeed()-162.0f)<.01f);
 }
 /* A single in-target native sample stops a transfer and advances without
  * waiting for another sequence or50ms. START still waits its normal path. */
 for(kind=0;kind<2;kind++) {
  ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_IDLE;enabled=1;motor.fb.enabled=1;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;motor.fb.last_tick=tick;
  s_run=s_saved;s_run.dm_deg=0;s_run.percent[0]=0;s_run.percent[1]=20;s_pick=kind?1:0;s_phase=POSE_REACH;
  axes[0].position_counts=0;axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.2);
  s_native=1;s_native_started=s_started=tick;s_native_timeout=10000;s_native_initial=0;
  s_native_sequence=axes[1].position_sequence-1;s_native_last=axes[1].position_counts-1000;
  s_native_retry_generation=retry_generation;cache_active=native_inflight=1;before=tick;
  ArmStartPose_Poll();
  CHECK(kind?s_phase==POSE_LIFT:s_phase==POSE_REACH);CHECK(tick==before);
  if(kind)CHECK(!s_native && !cache_active && !native_inflight);
 }
 ArmStartPose_Cancel();ArmStartPose_Init();s_phase=POSE_LIFT;s_pick=3;s_run.span[1]=ARM_REACH_FULL_TRAVEL_COUNTS;s_run.percent[1]=20;
 axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.2+4.9*65536.0/360.0);axes[1].position_tick=tick;
 CHECK(ArmStartPose_IsFastTransfer());CHECK(Pose_PlaceReachAtRelease());
 axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.2+5.2*65536.0/360.0);CHECK(!Pose_PlaceReachAtRelease());
 s_pick=0;s_generic=0;CHECK(!ArmStartPose_IsFastTransfer());
 s_phase=POSE_READY;s_pick_result_valid=1;s_pick=2;grip_angle=ArmServo_GripOpenDeg();grip_status.enabled=1;grip_status.ramping=grip_status.fault=0;
 CHECK(ArmStartPose_IsReleasedPlace());s_pick=3;CHECK(!ArmStartPose_IsReleasedPlace());
 s_ring_release_handoff=1;CHECK(ArmStartPose_IsReleasedPlace());
 s_pick=1;CHECK(!ArmStartPose_IsReleasedPlace());s_pick=2;s_pick_result_valid=0;CHECK(!ArmStartPose_IsReleasedPlace());
 s_pick_result_valid=1;grip_status.ramping=1;CHECK(!ArmStartPose_IsReleasedPlace());grip_status.ramping=0;
 /* Released/open claw: next tray pick, ring pick and generic wait return
  * start the actual lift job, then rotate only beyond half of that ascent. */
 for(kind=0;kind<=4;kind++) {
  ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_IDLE;
  s_run=s_saved;s_run.dm_deg=0;s_pick=kind;s_generic=kind==0;s_phase=POSE_TOP_ENABLE;
  s_started=s_tick=tick;s_pos_seq[0]=axes[0].position_sequence;s_flag_seq[0]=axes[0].flags_sequence;
  grip_angle=(kind==2 || kind==3)?ARM_SERVO_GRIP_CLOSE_DEG:ArmServo_GripOpenDeg();
  grip_status.enabled=1;grip_status.ramping=grip_status.fault=0;
  enabled=1;motor.fb.enabled=1;motor.fb.err_code=1;motor.fb.vel_rads=0;motor.fb.pos_rad=zero_ref;
  axes[0].flags=ARM_CAL_FLAG_ENABLED;axes[0].position_counts=(int64_t)round(.75*ARM_LIFT_FULL_TRAVEL_COUNTS);
  step();CHECK(s_phase==POSE_TOP && s_raise_overlap && cal_state==ARM_CAL_PERCENT_MOVE);
  CHECK(s_raise_grip_closed==(kind==2 || kind==3));
  CHECK(fabsf(s_raise_half_percent-(75+Pose_ClearancePercent())*.5f)<.001f);
  axes[0].position_counts=(int64_t)round((s_raise_half_percent+.5f)/100*ARM_LIFT_FULL_TRAVEL_COUNTS);
  before=dm_moves;step();CHECK(s_phase==POSE_TOP && dm_moves==before);
  axes[0].position_counts=(int64_t)round((s_raise_half_percent-.05f)/100*ARM_LIFT_FULL_TRAVEL_COUNTS);
  step();CHECK(s_phase==POSE_REACH_ENABLE && dm_moves==before+1 && cal_state==ARM_CAL_PERCENT_MOVE);
 }
 /* Loaded transfer retains half-rise overlap on the shortened raise.
  * From180mm down to40mm, rotation begins at110mm, not at TOP. */
 for(kind=2;kind<=3;kind++) {
  ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_PERCENT_MOVE;cal_axis=ARM_DRIVER_LIFT;
  cal_lift_target=kind==2?0:ArmCalibration_TransferPercent(240);s_run=s_saved;s_run.dm_deg=0;
  s_pick=kind;s_phase=POSE_TOP;s_raise_overlap=1;s_raise_grip_closed=1;s_raise_half_percent=(75+cal_lift_target)*0.5f;
  s_tick=s_started=tick;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=grip_status.fault=0;
  enabled=1;motor.fb.enabled=1;motor.fb.err_code=1;motor.fb.vel_rads=0;motor.fb.pos_rad=zero_ref;
  axes[0].position_counts=(int64_t)round((kind==2?91.0:111.0)/240*ARM_LIFT_FULL_TRAVEL_COUNTS);
  before=dm_moves;step();CHECK(s_phase==POSE_TOP && dm_moves==before);
  axes[0].position_counts=(int64_t)round((kind==2?89.9:109.9)/240*ARM_LIFT_FULL_TRAVEL_COUNTS);
  step();CHECK(s_phase==POSE_REACH_ENABLE && dm_moves==before+1 && cal_state==ARM_CAL_PERCENT_MOVE);
  CHECK(fabsf(Pose_ClearancePercent()*240/100-(kind==2?0:40))<0.001f);
 }
 return failures;
}
static void ring_overlap_poll(float angle_error,float lift_error_mm,float velocity,uint32_t ms)
{
 unsigned i;tick+=ms;
 for(i=0;i<2;i++){axes[i].position_tick=axes[i].flags_tick=tick;axes[i].position_sequence++;axes[i].flags_sequence++;}
 axes[0].position_counts=axes[0].zero_counts+(int64_t)llround((s_run.percent[0]+lift_error_mm/axes[0].travel_mm*100.0)*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
 motor.fb.last_tick=tick;motor.fb.fb_count++;motor.fb.pos_rad=zero_ref+(s_run.dm_deg+angle_error)/57.2957795f;motor.fb.vel_rads=velocity;
 ArmStartPose_Poll();
}
static void ring_overlap_setup(uint8_t pick,float target_dm)
{
 unsigned i;ArmStartPose_Cancel();ArmStartPose_Init();cal_state=ARM_CAL_IDLE;ring_overlap_test=1;
 memset(axes,0,sizeof(axes));memset(&s_run,0,sizeof(s_run));
 for(i=0;i<2;i++){axes[i].travel_mm=i?178:240;axes[i].zero_valid=1;axes[i].flags=0x81;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;}
 s_run.percent[0]=pick==1?40:80;s_run.percent[1]=20;s_run.span[0]=ARM_LIFT_FULL_TRAVEL_COUNTS;s_run.span[1]=ARM_REACH_FULL_TRAVEL_COUNTS;s_run.dm_zero=zero_ref;s_run.dm_deg=target_dm;
 axes[1].position_counts=(int64_t)llround(.2*ARM_REACH_FULL_TRAVEL_COUNTS);
 axes[0].position_counts=(int64_t)llround(ARM_LIFT_FULL_TRAVEL_COUNTS*40/240);
 close_lift_expected=(int64_t)llround(s_run.percent[0]/100.0*ARM_LIFT_FULL_TRAVEL_COUNTS);close_dm_expected=target_dm;dm_target=target_dm;
 grip_status.enabled=1;grip_status.ramping=grip_status.fault=0;grip_angle=225;
 enabled=motor.fb.enabled=1;motor.fb.err_code=1;motor.fb.fb_count=1;motor.fb.last_tick=tick;
 motor.fb.pos_rad=zero_ref+(target_dm+16)/57.2957795f;motor.fb.vel_rads=.3f;
 s_phase=POSE_REACH;s_pick=pick;s_turn_pending=1;s_dm_wait_reach=0;s_native=s_reach_encoder=s_raise_overlap=0;
 s_started=s_tick=s_dm_started=tick;s_dm_seq=motor.fb.fb_count;
 chassis=read_fail=raw_guard_reject=retry_recovering=retry_failed=stall_pending=reach_stall=0;
}
static float test_return_start_pct;
static void return_parallel_setup(uint8_t ring,uint8_t slot,float height_mm,uint8_t unstarted,uint8_t source_kind)
{
 ring_overlap_setup(source_kind,ring==1?39.6044f:ring==2?3.404f:-40.2448f);
 tray_start_test=unstarted;tray_replay_calls=0;tray_replay_busy=tray_replay_error=0;
 memset(&tray_ack,0,sizeof(tray_ack));tray_ack_fence=7;
 test_return_start_pct=height_mm/2.4f;
 axes[0].position_counts=(int64_t)llround(test_return_start_pct*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
 axes[1].position_counts=(int64_t)llround(.2*ARM_REACH_FULL_TRAVEL_COUNTS);
 motor.fb.pos_rad=zero_ref+dm_target/57.2957795f;motor.fb.vel_rads=0;
 s_phase=POSE_READY;s_pick=source_kind;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
 cal_state=ARM_CAL_PERCENT_MOVE;cal_axis=ARM_DRIVER_LIFT;cal_lift_target=0;
 CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(slot,test_return_start_pct)==HAL_OK);
 CHECK(s_ring_return_reach==(source_kind==4?1:0));
}
static void return_parallel_step(float rise_mm,uint32_t ms)
{
 unsigned i;tick+=ms;
 for(i=0;i<2;i++){axes[i].position_tick=axes[i].flags_tick=tick;axes[i].position_sequence++;axes[i].flags_sequence++;}
 axes[0].position_counts=(int64_t)llround((test_return_start_pct-rise_mm/2.4f)*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
 motor.fb.last_tick=tick;motor.fb.fb_count++;motor.fb.pos_rad=zero_ref+dm_target/57.2957795f;motor.fb.vel_rads=0;
 ArmStartPose_Poll();
}
static int test_ring_return_parallel(void)
{
 unsigned ring,slot;uint32_t turns,native,lifts,started,timeout,grips;
 for(ring=1;ring<=3;ring++)for(slot=1;slot<=3;slot++) {
  return_parallel_setup((uint8_t)ring,(uint8_t)slot,192,0,4);turns=dm_moves;native=linear_moves;lifts=lift_commands;grips=grip_open_calls+grip_close_calls;
  return_parallel_step(0,10);CHECK(s_phase==POSE_TOP && s_ring_return_reach==2 && linear_moves==native && dm_moves==turns);
  return_parallel_step(0,10);CHECK(s_phase==POSE_TOP && s_ring_return_reach==3 && linear_moves==native+1 && dm_moves==turns);
  started=s_native_started;timeout=s_native_timeout;
  CHECK(lift_commands==lifts && grip_open_calls+grip_close_calls==grips && cal_state==ARM_CAL_PERCENT_MOVE);
  return_parallel_step(95.9f,10);CHECK(s_phase==POSE_TOP && dm_moves==turns && linear_moves==native+1);
  return_parallel_step(96.1f,10);CHECK(s_phase==POSE_REACH && dm_moves==turns+1 && linear_moves==native+1);
  CHECK(s_native_started==started && s_native_timeout==timeout);
  return_parallel_step(180,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts);
  cal_state=ARM_CAL_IDLE;return_parallel_step(192,10);CHECK(s_phase==POSE_LIFT && lift_commands==lifts+1);
  ArmStartPose_Cancel();
 }
 /* The V229 ACK/start replay is already live during the early rise. */
 return_parallel_setup(1,1,192,1,4);turns=dm_moves;lifts=lift_commands;
 return_parallel_step(0,10);return_parallel_step(0,10);started=s_native_started;timeout=s_native_timeout;
 return_parallel_step(0,10);CHECK(s_phase==POSE_TOP && tray_replay_calls==1 && dm_moves==turns);
 tray_ack.sequence=8;tray_ack.received_tick=tick;tray_ack.response=2;
 return_parallel_step(32,10);CHECK(tray_replay_calls==2 && s_reach_start.acked && s_reach_start.active);
 axes[1].position_counts=(int64_t)(.19*ARM_REACH_FULL_TRAVEL_COUNTS);return_parallel_step(33,10);
 CHECK(!s_reach_start.active && tray_replay_calls==2 && lift_commands==lifts);
 return_parallel_step(96.1f,10);CHECK(dm_moves==turns+1 && s_native_started==started && s_native_timeout==timeout);
 cal_state=ARM_CAL_IDLE;return_parallel_step(192,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts);
 axes[1].position_counts=(int64_t)llround(s_run.percent[1]*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);
 return_parallel_step(192,10);CHECK(s_phase==POSE_LIFT && lift_commands==lifts+1);
 /* Recovery retains ownership before DM starts; STOP cancels early replay. */
 return_parallel_setup(1,1,192,1,4);turns=dm_moves;
 return_parallel_step(0,10);return_parallel_step(0,10);retry_recovering=1;
 return_parallel_step(96.1f,10);CHECK(s_phase==POSE_TOP && dm_moves==turns && !tray_replay_calls);
 retry_recovering=0;return_parallel_step(96.1f,10);CHECK(dm_moves==turns+1);
 ArmStartPose_Cancel();native=linear_moves;return_parallel_step(100,100);CHECK(s_phase==POSE_IDLE && linear_moves==native && !s_ring_return_reach);
 /* The early job retains wrong-direction checks before DM half-rise,
  * and the existing stale-feedback pause applies to enable timing too. */
 return_parallel_setup(1,1,192,1,4);turns=dm_moves;
 return_parallel_step(0,10);return_parallel_step(0,10);
 axes[1].position_counts=(int64_t)llround(.22*ARM_REACH_FULL_TRAVEL_COUNTS);return_parallel_step(32,10);
 CHECK(s_phase==POSE_FAILED && dm_moves==turns);
 return_parallel_setup(1,1,192,0,4);return_parallel_step(0,10);test_pose_feedback_gap=2000;
 return_parallel_step(31,2010);CHECK(s_phase==POSE_TOP && s_ring_return_reach==3U);
 /* Non-ring adoption keeps its old shared half-rise start. */
 return_parallel_setup(1,1,192,0,0);native=linear_moves;turns=dm_moves;
 return_parallel_step(30.1f,10);CHECK(s_phase==POSE_TOP && linear_moves==native && dm_moves==turns);
 return_parallel_step(96.1f,10);CHECK(s_phase==POSE_REACH_ENABLE && linear_moves==native && dm_moves==turns+1);
 /* Short taught lifts also start reach before movement while keeping DM half-rise. */
 for(ring=0;ring<2;ring++) {
  float height=ring?20.0f:40.0f;
  return_parallel_setup(1,1,height,0,4);turns=dm_moves;native=linear_moves;
  return_parallel_step(0,10);return_parallel_step(0,10);
  CHECK(s_phase==POSE_TOP && dm_moves==turns && linear_moves==native+1);
  return_parallel_step(height*.5f-.1f,10);CHECK(s_phase==POSE_TOP && dm_moves==turns);
  return_parallel_step(height*.5f+.1f,10);CHECK(s_phase==POSE_REACH && dm_moves==turns+1 && linear_moves==native+1);
 }
 tray_start_test=0;ArmStartPose_Cancel();return failures;
}
static void tray_start_setup(void)
{
 ring_overlap_setup(2,162.825f);tray_start_test=1;tray_replay_calls=0;tray_replay_busy=tray_replay_error=0;
 memset(&tray_ack,0,sizeof(tray_ack));tray_ack_fence=7;
 s_run.percent[0]=30;s_run.percent[1]=1.0f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
 axes[0].position_counts=0;axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.395);
 motor.fb.pos_rad=zero_ref+162.825f/57.2957795f;motor.fb.vel_rads=0;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
 CHECK(Pose_ReachNative(&axes[1],tick)==HAL_OK && s_reach_start.active && s_reach_start.attempts==1);
}
static void tray_start_fresh(uint32_t ms)
{
 unsigned i;tick+=ms;for(i=0;i<2;i++){axes[i].position_tick=axes[i].flags_tick=tick;axes[i].position_sequence++;axes[i].flags_sequence++;}
 motor.fb.last_tick=tick;motor.fb.fb_count++;
}
static int test_tray_start_retry(void)
{
 uint32_t started,timeout,before,generation;unsigned i;
 tray_start_setup();started=s_native_started;timeout=s_native_timeout;generation=retry_generation;before=stops;
 tray_start_fresh(9);CHECK(Pose_ReachStartupService() && !tray_replay_calls);
 tray_start_fresh(1);CHECK(Pose_ReachStartupService() && tray_replay_calls==1 && s_reach_start.attempts==2);
 tray_ack.sequence=7;tray_ack.received_tick=tick;tray_ack.response=2;tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && !s_reach_start.acked && tray_replay_calls==2);
 tray_ack.sequence=8;tray_ack.received_tick=tick;tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && s_reach_start.acked && tray_replay_calls==3 && s_reach_start.active);
 axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.388);tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && !s_reach_start.active && tray_replay_calls==3);
 CHECK(s_native_started==started && s_native_timeout==timeout && retry_generation==generation && stops==before);
 tray_start_fresh(100);CHECK(Pose_ReachStartupService() && tray_replay_calls==3);
 /* Actual start suffices even if every ACK is missing. */
 tray_start_setup();axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.388);tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && !s_reach_start.active && !s_reach_start.acked && !tray_replay_calls);
 /* A stale queued ACK and old position sequence cannot confirm this job. */
 tray_start_setup();tray_ack.sequence=8;tray_ack.received_tick=s_reach_start.started-1;tray_ack.response=2;
 axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.388);tick+=10;
 CHECK(Pose_ReachStartupService() && s_reach_start.active && !s_reach_start.acked && tray_replay_calls==1);
 tray_start_fresh(10);CHECK(Pose_ReachStartupService() && !s_reach_start.active);
 /* Reverse drift never counts as a start; the existing wrong-direction
  * detector remains active rather than being hidden by retries. */
 tray_start_setup();axes[1].position_counts=(int64_t)(ARM_REACH_FULL_TRAVEL_COUNTS*.415);tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && s_reach_start.active && tray_replay_calls==1);
 ArmStartPose_Poll();CHECK(s_phase==POSE_FAILED && !s_reach_start.active);
 /* 50 sends include the original. Neither replay nor exhaustion renews
  * the original10s minimum native deadline or causes a new short fault. */
 tray_start_setup();started=s_native_started;timeout=s_native_timeout;
 for(i=0;i<49;i++){tray_start_fresh(10);CHECK(Pose_ReachStartupService());}
 CHECK(s_reach_start.attempts==50 && tray_replay_calls==49 && s_phase==POSE_REACH);
 tray_start_fresh(1000);CHECK(Pose_ReachStartupService() && s_reach_start.exhausted && tray_replay_calls==49);
 CHECK(s_native_started==started && s_native_timeout==timeout);
 tick=started+timeout+1;tray_start_fresh(0);ArmStartPose_Poll();CHECK(s_phase==POSE_FAILED);
 /* Integration: startup replay remains active during the concurrent lift
  * raise; it must not wait for TOP or send a replacement lift command. */
 tray_start_setup();cal_state=ARM_CAL_PERCENT_MOVE;cal_axis=ARM_DRIVER_LIFT;cal_lift_target=0;
 s_raise_overlap=s_raise_grip_closed=1;s_raise_half_percent=50;
 axes[0].position_counts=(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*.2);before=lift_commands;
 tray_start_fresh(10);ArmStartPose_Poll();
 CHECK(s_phase==POSE_REACH && tray_replay_calls==1 && lift_commands==before && cal_state==ARM_CAL_PERCENT_MOVE);
 /* An already-near endpoint does not start a native job or replay. */
 tray_start_setup();axes[1].position_counts=(int64_t)(s_run.percent[1]*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);
 CHECK(Pose_ReachNative(&axes[1],tick)==HAL_OK && !s_reach_start.active && !tray_replay_calls);
 /* BUSY does not consume a send; cancellation/recovery must retain ownership. */
 tray_start_setup();tray_replay_busy=1;tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && s_reach_start.attempts==1 && !tray_replay_calls);
 tray_start_fresh(10);CHECK(Pose_ReachStartupService() && s_reach_start.attempts==2 && tray_replay_calls==1);
 ArmStartPose_Cancel();tray_start_fresh(100);CHECK(Pose_ReachStartupService() && !s_reach_start.active && tray_replay_calls==1);
 tray_start_setup();retry_recovering=1;tray_start_fresh(10);
 CHECK(Pose_ReachStartupService() && s_reach_start.active && !tray_replay_calls);
 retry_recovering=0;retry_generation++;CHECK(Pose_ReachStartupService() && !s_reach_start.active && !tray_replay_calls);
 tray_start_setup();tray_replay_error=1;tray_start_fresh(10);CHECK(!Pose_ReachStartupService() && s_phase==POSE_FAILED);
 /* Other pose kinds never activate this return-only replay. */
 for(i=1;i<=4;i++)if(i!=2){tray_start_setup();s_pick=(uint8_t)i;tray_start_fresh(10);CHECK(Pose_ReachStartupService() && !s_reach_start.active && !tray_replay_calls);}
 tray_start_test=0;ArmStartPose_Cancel();return failures;
}
static int test_ring_pick_overlap(void)
{
 uint32_t lifts,closed;uint8_t ok;float side;unsigned k,kind;float start_error;
 for(kind=0;kind<2;kind++)for(k=0;k<2;k++) {
  side=k?-1:1;ring_overlap_setup(kind?1:4,kind?162.825f:0);start_error=kind?-56:-152;lifts=lift_commands;closed=grip_close_calls;
  ring_overlap_poll(side*15.01f,start_error,.3f,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts && grip_close_calls==closed);
  ring_overlap_poll(side*14.99f,start_error,.3f,10);CHECK(s_phase==POSE_LIFT && s_turn_pending && lift_commands==lifts+1 && cal_state==ARM_CAL_PERCENT_MOVE);
  CHECK(grip_close_calls==closed && fabsf(cal_lift_target-s_run.percent[0])<.001f);
  /* Height reached first: the 15deg descent tolerance never closes the claw. */
  ring_overlap_poll(side*8, -2, .2f,10);CHECK(s_phase==POSE_LIFT && grip_close_calls==closed);
  ring_overlap_poll(side*.8f,-3.01f,0,10);CHECK(!s_turn_pending && s_phase==POSE_LIFT && grip_close_calls==closed);
  ring_overlap_poll(side*.8f,k?2.99f:-2.99f,0,10);CHECK(s_phase==POSE_GRIP_CLOSE && grip_close_calls==closed+1 && cal_state==ARM_CAL_PERCENT_MOVE);
  CHECK(!ArmStartPose_ReadActionResult(&ok));
  ring_overlap_poll(0,0,0,200);CHECK(s_phase==POSE_GRIP_CLOSE && grip_close_calls==closed+1);
  /* PWM completion alone does not overwrite an accepted descent with raise. */
  grip_status.ramping=0;ring_overlap_poll(0,0,0,100);CHECK(s_phase==POSE_GRIP_CLOSE && lift_commands==lifts+1);
  ring_overlap_poll(0,0,0,5100);CHECK(s_phase==POSE_GRIP_CLOSE && grip_close_calls==closed+1);
  cal_state=ARM_CAL_IDLE;ring_overlap_poll(0,0,0,10);CHECK(s_phase==POSE_READY && ArmStartPose_ReadActionResult(&ok) && ok);
 }
 /* Near an edge ring, preserve the real low-height angular sector. */
 ring_overlap_setup(4,41);lifts=lift_commands;
 ring_overlap_poll(14.9f,-152,.3f,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts);
 ring_overlap_poll(2.9f,-152,.1f,10);CHECK(s_phase==POSE_LIFT && lift_commands==lifts+1 && s_turn_pending);
 /* Tray placement starts lowering at5deg on either side, including live
  * reach tracking; never opens until final DM/height arrival. */
 for(kind=0;kind<2;kind++)for(k=0;k<2;k++) {
  side=k?-1:1;ring_overlap_setup(2,134.6173f);s_run.percent[0]=30;
  grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_place_track=(uint8_t)kind;
  s_place_track_started=s_place_track_command_tick=tick;s_native_initial=20;
  s_native_retry_generation=retry_generation;s_place_track_goal_pulses=100;
  lifts=lift_commands;closed=grip_open_calls;
  ring_overlap_poll(side*5.01f,-72,.3f,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts);
  ring_overlap_poll(side*4.99f,-72,.3f,10);CHECK(s_phase==POSE_LIFT && s_turn_pending && lift_commands==lifts+1);
  ring_overlap_poll(side*4.9f,0,.2f,10);CHECK(s_phase==POSE_LIFT && grip_open_calls==closed);
  cal_state=ARM_CAL_IDLE;ring_overlap_poll(side*4.9f,0,.2f,10);
  CHECK(s_phase==POSE_LIFT && grip_open_calls==closed); /* Height first cannot release at5deg. */
  ring_overlap_poll(side*.5f,0,.1f,10);CHECK(s_phase==POSE_LIFT && grip_open_calls==closed);
  ring_overlap_poll(side*.5f,0,0,10);CHECK(s_phase==POSE_GRIP_CLOSE && !s_turn_pending && grip_open_calls==closed+1);
  ring_overlap_poll(0,0,0,200);CHECK(s_phase==POSE_GRIP_CLOSE); /* Tray OPEN timing is unchanged. */
  ring_overlap_poll(0,0,0,ARM_SERVO_GRIP_OPEN_MIN_MS-200);CHECK(s_phase==POSE_READY && ArmStartPose_ReadActionResult(&ok) && ok);
 }
 /* The post-grip hold check accepts the new +/-0.5rad/s boundary only. */
 motor.fb.vel_rads=.5f;CHECK(Pose_GripHoldFailure()==NULL);
 motor.fb.vel_rads=-.5f;CHECK(Pose_GripHoldFailure()==NULL);
 motor.fb.vel_rads=.501f;CHECK(Pose_GripHoldFailure()!=NULL);
 motor.fb.vel_rads=-.501f;CHECK(Pose_GripHoldFailure()!=NULL);
 motor.fb.vel_rads=NAN;CHECK(Pose_GripHoldFailure()!=NULL);motor.fb.vel_rads=0;
 /* Ring placement uses15deg descent too, but never releases before final
  * DM and the original final lift arrival. Exercise live reach tracking. */
 for(k=0;k<2;k++) {
  ring_overlap_setup(3,0);grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_place_track=(uint8_t)k;
  s_place_track_started=s_place_track_command_tick=tick;s_native_initial=20;
  s_native_retry_generation=retry_generation;s_place_track_goal_pulses=100;
  lifts=lift_commands;closed=grip_open_calls;
  ring_overlap_poll(15.01f,-152,.3f,10);CHECK(s_phase==POSE_REACH && lift_commands==lifts);
  ring_overlap_poll(14.99f,-152,.3f,10);CHECK(s_phase==POSE_LIFT && s_turn_pending && lift_commands==lifts+1);
  cal_state=ARM_CAL_IDLE;ring_overlap_poll(8,0,.2f,10);CHECK(s_phase==POSE_LIFT && grip_open_calls==closed);
  ring_overlap_poll(.5f,-2.99f,0,10);CHECK(s_phase==POSE_FAILED); /* Place still requires its original final height. */
  ring_overlap_setup(3,0);grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_place_track=(uint8_t)k;
  s_place_track_started=s_place_track_command_tick=tick;s_native_initial=20;s_native_retry_generation=retry_generation;
  ring_overlap_poll(10,-152,.3f,10);cal_state=ARM_CAL_IDLE;ring_overlap_poll(0,0,0,10);
  CHECK(s_phase==POSE_GRIP_CLOSE);grip_status.ramping=1;closed=grip_open_calls;
  ring_overlap_poll(0,0,0,199);CHECK(s_phase==POSE_GRIP_CLOSE);
  ring_overlap_poll(0,0,0,1);CHECK(s_phase==POSE_READY && s_ring_release_handoff && grip_status.ramping && ArmStartPose_IsReleasedPlace());
  grip_open_confirmed=0;
  if(k==0) {
   ArmStartPose_Command("arm pick");
   CHECK(s_pick==1 && s_phase==POSE_TOP_ENABLE && grip_open_calls==closed && !s_ring_release_handoff && ArmStartPose_IsReleaseOpening());
   lifts=lift_commands;ring_overlap_poll(0,(80-s_run.percent[0])*2.4f,0,10);
   CHECK(s_phase==POSE_TOP && lift_commands==lifts+1 && s_raise_overlap && grip_status.ramping);
   ring_overlap_poll(0,s_raise_half_percent*2.4f-s_run.percent[0]*2.4f-.1f,0,10);
   CHECK(s_phase==POSE_TOP); /* OPEN can overlap rise, not the subsequent rotation. */
   grip_status.ramping=0;ring_overlap_poll(0,s_raise_half_percent*2.4f-s_run.percent[0]*2.4f-.1f,0,10);
   CHECK(s_phase==POSE_REACH_ENABLE);
  } else {
   CHECK(ArmStartPose_IsReleaseOpening());ArmStartPose_ReleaseForCamera();
   CHECK(s_phase==POSE_IDLE && grip_status.ramping && ArmServo_GripOpeningOrOpen());
  }
 }
 ring_overlap_setup(4,0);closed=grip_close_calls;
 ring_overlap_poll(10,-152,.3f,10);ring_overlap_poll(.5f,-2,.1f,10);
 CHECK(s_phase==POSE_LIFT && grip_close_calls==closed); /* Not yet rotationally settled. */
 ring_overlap_poll(.5f,-2,0,10);CHECK(s_phase==POSE_GRIP_CLOSE);
 cal_state=ARM_CAL_FAILED;grip_status.ramping=0;ring_overlap_poll(0,0,0,1000);CHECK(s_phase==POSE_FAILED);
 /* STOP/cancel during the overlap cannot produce a delayed close. */
 ring_overlap_setup(4,0);closed=grip_close_calls;ring_overlap_poll(10,-152,.3f,10);ArmStartPose_Cancel();ring_overlap_poll(0,0,0,100);
 CHECK(s_phase==POSE_IDLE && grip_close_calls==closed);
 return failures;
}
int main(void){
#ifdef TEST_PROFILE_ISOLATION_ONLY
 return test_profile_isolation();
#else
 (void)test_profile_isolation;
#endif
#ifdef TEST_DISC_PROFILE_ONLY
 return test_disc_profiles();
#else
 (void)test_disc_profiles;
#endif
#ifdef TEST_RING_RETURN_PARALLEL_ONLY
 return test_ring_return_parallel();
#else
 (void)test_ring_return_parallel;
#endif
#ifdef TEST_TRAY_START_RETRY_ONLY
 return test_tray_start_retry();
#else
 (void)test_tray_start_retry;
#endif
#ifdef TEST_RING_PICK_OVERLAP_ONLY
 return test_ring_pick_overlap();
#else
 (void)test_ring_pick_overlap;
#endif
#ifdef TEST_UI_ONLY
 return test_ui_controls();
#else
 (void)test_ui_controls;
#endif
#ifdef TEST_FAST_TRANSFER_ONLY
 return test_fast_transfer();
#else
 (void)test_fast_transfer;
#endif
#ifdef TEST_PHOTO_TARGETS_ONLY
 return test_photo_targets();
#else
 (void)test_photo_targets;
#endif
#ifdef TEST_TRAY_GATE_ONLY
 return test_tray_gate();
#else
 CHECK(test_tray_gate()==0);
#endif
 unsigned i;int64_t target5=(int64_t)round(ARM_START_DEFAULT_LIFT_DEG*65536.0/360.0);
 expected_reach=(int64_t)round(ARM_START_DEFAULT_REACH_DEG*65536.0/360.0);
 memset(flash,0xA5,sizeof(flash)); /* Old/wrong W25 contents must never be read. */
 memset(axes,0,sizeof(axes));
 for(i=0;i<2;i++){axes[i].travel_mm=i?ARM_REACH_FULL_TRAVEL_MM:ARM_LIFT_FULL_TRAVEL_MM;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
 axes[0].position_counts=-7895;axes[1].position_counts=0;motor.fb.pos_rad=0.1f;motor.fb.vel_rads=0;motor.fb.fb_count=1;motor.fb.last_tick=tick;
 ArmStartPose_Init();CHECK(flash_reads==0 && flash_writes==0);
 CHECK(ArmStartPose_Command("arm ring")==1 && !ArmStartPose_IsActive() && !ArmStartPose_IsActive());
 CHECK(ArmStartPose_Command("arm ring confirm")==1 && !ArmStartPose_IsActive());
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 CHECK(ArmStartPose_BeginRestore()==HAL_OK);step();CHECK(dm_moves==0 && axes[0].position_counts==0);
 cal_state=ARM_CAL_IDLE;finish();CHECK(fabsf(dm_target-ARM_START_DEFAULT_DM_DEG)<0.001f && llabs(axes[0].position_counts-target5)<=1 && llabs(axes[1].position_counts-expected_reach)<=12);
 enabled=0;motor.fb.enabled=0;motor.fb.err_code=0;motor.fb.pos_rad=0.1f;
 CHECK(ArmStartPose_Command("arm start save")==1);
 for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
 CHECK(!ArmStartPose_IsActive() && flash_writes==0 && strstr(last_console,"RAM CAPTURED"));
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"source=2"));
 /* Reinitialization restores fixed defaults; the temporary capture is gone. */
 ArmStartPose_Init();CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 CHECK(ArmStartPose_Command("arm start set -30 -200 10")==1);
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+10.0000deg"));
 expected_reach=(int64_t)round(-200.0*65536.0/360.0);
 CHECK(ArmStartPose_BeginRestore()==HAL_OK);finish();
 CHECK(fabsf(dm_target-10)<0.001 && llabs(axes[1].position_counts-expected_reach)<=12);
 enabled=0;motor.fb.enabled=0;motor.fb.err_code=0;
 CHECK(ArmStartPose_Command("arm start set 30 -200 10")==1);
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+10.0000deg")); /* invalid lift leaves targets */
 CHECK(ArmStartPose_Command("arm start defaults")==1);
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 axes[0].zero_counts=12000;axes[0].position_counts=12000;
 axes[1].zero_counts=500;axes[1].position_counts=500;
 expected_reach=500+(int64_t)round(ARM_START_DEFAULT_REACH_DEG*65536.0/360.0);
 CHECK(ArmStartPose_BeginRestore()==HAL_OK);finish();
 CHECK(llabs(axes[0].position_counts-(12000+target5))<=1 && llabs(axes[1].position_counts-expected_reach)<=12);
 CHECK(flash_reads==0 && flash_writes==0);
 expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
 lift_commands_before_pick=lift_commands;lag_on_native=1;
 CHECK(ArmStartPose_Command("arm pick")==1 && grip_open_calls==1 && grip_close_calls==0);
 {uint32_t before=linear_moves;grip_status.ramping=1;step();step();CHECK(linear_moves==before && grip_close_calls==0);grip_status.ramping=0;}
 for(i=0;i<45 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(lift_commands==lift_commands_before_pick+1);
 CHECK(!ArmStartPose_IsActive() && grip_close_calls==1 && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG && !ArmStartPose_IsReady());
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 CHECK(ArmStartPose_Command("arm pick")==1);ArmStartPose_Cancel();
 for(i=0;i<8;i++){step();}
 CHECK(grip_close_calls==1);
 rotate_result=HAL_ERROR;CHECK(ArmStartPose_Command("arm pick")==1);
 for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(!ArmStartPose_IsActive() && grip_close_calls==1 && grip_angle==225);
 rotate_result=HAL_OK;cal_state=ARM_CAL_IDLE;
 {uint32_t moves_before=linear_moves,opens_before=grip_open_calls;
 CHECK(ArmStartPose_Command("arm place")==1 && !ArmStartPose_IsActive());
 CHECK(linear_moves==moves_before && grip_open_calls==opens_before);
 grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=1;
 CHECK(ArmStartPose_Command("arm place")==1 && !ArmStartPose_IsActive());
 grip_status.ramping=0;axes[1].position_counts=axes[1].zero_counts-54000;
 place_test=1;lag_on_native=0;drift_on_stop=0;follow_dm=0;motor.fb.pos_rad=zero_ref;CHECK(ArmStartPose_Command("arm place")==1 && ArmStartPose_IsActive());
 {uint32_t dm_before=dm_moves;
 CHECK(!enabled && ArmStartPose_IsRotationReleaseExpected() && s_phase==POSE_TOP_ENABLE);
 cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_TOP && ArmStartPose_IsRotationReleaseExpected() && !enabled);
 CHECK(grip_open_calls==opens_before);
 for(i=0;i<25 && !native_inflight && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(parallel_seen && native_inflight && dm_moves==dm_before+1 && grip_open_calls==opens_before);
 CHECK(enabled && !ArmStartPose_IsRotationReleaseExpected());
 CHECK(soft_calls==0 && s_turn_pending && fabsf(dm_target-162.825f)<0.01f); /* turn already overlaps reach */
 {int64_t near_target=axes[1].position_counts;
 axes[1].position_counts=axes[1].zero_counts-49000;
 for(i=0;i<9;i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(ArmStartPose_IsActive() && grip_open_calls==opens_before);
 CHECK(dm_moves==dm_before+1 && s_turn_pending);
 axes[1].position_counts=near_target;}
 for(i=0;i<25;i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(s_phase==POSE_REACH && dm_moves==dm_before+1 && native_inflight && s_turn_pending);
 CHECK(axes[0].position_counts==axes[0].zero_counts && grip_open_calls==opens_before);
 follow_dm=1;
 for(i=0;i<35 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(s_phase==POSE_LIFT && grip_open_calls==opens_before);
 for(i=0;i<5;i++){cal_state=ARM_CAL_PERCENT_MOVE;step();}CHECK(grip_open_calls==opens_before);
 for(i=0;i<12 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(llabs(axes[0].position_counts-axes[0].zero_counts+73767)<=2);
 CHECK(slow_corrections==0); /*PLACE uses native tracking, not percent6 controller.*/
 CHECK(!ArmStartPose_IsActive() && grip_angle==225 && grip_open_calls==opens_before+1);
 CHECK(!ArmStartPose_IsRotationReleaseExpected());
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 }
 place_test=0;pick_clearance_seen=0;axes[0].position_counts=axes[0].zero_counts-91845;
 CHECK(ArmStartPose_Command("arm pick")==1);
 for(i=0;i<50 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(!ArmStartPose_IsActive() && pick_clearance_seen && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG);
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 {uint32_t corrections_before=slow_corrections;
 grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;axes[1].position_counts=axes[1].zero_counts+1400;
 CHECK(ArmStartPose_Command("arm place")==1);
 CHECK(Pose_ReachTolerance()>=ARM_PERCENT_TOLERANCE); /* +1deg common endpoint uses existing near-zero band. */
 for(i=0;i<50 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(!ArmStartPose_IsActive() && grip_angle==225 && slow_corrections==corrections_before);
 }
 CHECK(ArmStartPose_Command("arm ring")==1 && !ArmStartPose_IsActive()); /* Last PLACE opened grip. */
 expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
 CHECK(ArmStartPose_Command("arm pick")==1);
 for(i=0;i<55 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(s_pick_completed && ArmServo_GripIsClosed());
 {
 uint32_t corrected_before=track_corrections,percent_before=encoder_reach_moves;
 expected_reach=axes[1].zero_counts-33395;lag_on_native=1;
 CHECK(ArmStartPose_Command("arm ring")==1 && ArmStartPose_IsActive());
 CHECK(s_pick==3U);
 CHECK(ArmStartPose_Command("arm ring confirm")==1 && ArmStartPose_IsActive());
 for(i=0;i<60 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(!ArmStartPose_IsActive() && grip_angle==225 && !s_pick_completed);
 CHECK(track_corrections>corrected_before && encoder_reach_moves==percent_before);
 CHECK(fabsf(dm_target-2.6040f)<0.001f);
 CHECK(llabs(axes[0].position_counts-axes[0].zero_counts+187983)<=2);
 CHECK(llabs(axes[1].position_counts-expected_reach)<=12);
 CHECK(ArmStartPose_Command("arm ring")==1 && !ArmStartPose_IsActive());
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 }
 {
 unsigned station;const int64_t reach_counts[3]={-95527,-33395,-51391};
 const float ring_dm[3]={40.9044f,2.6040f,-39.5218f};
 char command[32],station_text[20];
 CHECK(ArmStartPose_Command("arm ring 4")==1 && !ArmStartPose_IsActive());
 for(station=1;station<=3;station++) {
  CHECK(ArmStartPose_Command("arm ring 1")==1 && !ArmStartPose_IsActive()); /* No carried material. */
  expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
  CHECK(ArmStartPose_Command("arm pick")==1);
  for(i=0;i<65 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
  CHECK(s_pick_completed && ArmServo_GripIsClosed());
  expected_reach=axes[1].zero_counts+reach_counts[station-1];
  snprintf(command,sizeof(command),"arm ring %u",station);
  CHECK(ArmStartPose_Command(command)==1 && ArmStartPose_IsActive() && s_ring_station==station);
  snprintf(station_text,sizeof(station_text),"station=%u",station);
  CHECK(ArmStartPose_Command("arm ring status")==1 && strstr(last_console,station_text));
  {uint8_t saw_away=0;
   CHECK(!ArmStartPose_IsAwayFromTray());
   for(i=0;i<75 && ArmStartPose_IsActive();i++){
    cal_state=ARM_CAL_IDLE;step();
    if(s_phase==POSE_TOP_ENABLE || s_phase==POSE_TOP || s_phase==POSE_DM)CHECK(!ArmStartPose_IsAwayFromTray());
    if(ArmStartPose_IsAwayFromTray())saw_away=1;
   }
   CHECK(saw_away);
  }
  CHECK(!ArmStartPose_IsActive() && grip_angle==225 && !s_pick_completed);
  CHECK(fabsf(dm_target-ring_dm[station-1])<0.001f);
  CHECK(llabs(axes[0].position_counts-axes[0].zero_counts+187983)<=2); /* All three release endpoints UP4mm from V108. */
  CHECK(llabs(axes[1].position_counts-expected_reach)<=12);
 }
 }
 {
 uint8_t result;
 expected_reach=axes[1].zero_counts+(int64_t)round(-185.1581*65536.0/360.0);
 {uint32_t corrections_before=encoder_reach_moves,slow_before=slow_corrections;
 lag_on_native=1; /* Quiet residual on return: native ABS retry cannot be trusted to remove it. */
 CHECK(ArmStartPose_BeginMove(-361.56436f,-185.1581f,-2.0217f)==HAL_OK);
 CHECK(!ArmStartPose_IsReady());
 for(i=0;i<65 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 CHECK(ArmStartPose_ReadActionResult(&result) && result && !ArmStartPose_IsReady());
 CHECK(encoder_reach_moves==corrections_before+1 && slow_corrections==slow_before);
 CHECK(fabs((double)(axes[0].position_counts-axes[0].zero_counts)*360.0/65536.0+361.56436)<0.02);
 CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
 }
 }
 expected_reach=axes[1].zero_counts+(int64_t)round(ARM_START_DEFAULT_REACH_DEG*65536.0/360.0);
 CHECK(ArmStartPose_BeginRestore()==HAL_OK);finish();
 }
 /* Loaded165deg PLACE rotates90deg/s concurrently with reach retraction. */
 {
   uint32_t began;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
   motor.fb.pos_rad=zero_ref-2.0217f/57.2957795f;motor.fb.vel_rads=0;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   follow_dm=1;rotation_plant=1;parallel_seen=0;place_test=1;
   CHECK(ArmStartPose_Command("arm place")==1 && ArmStartPose_IsActive());began=tick;
   for(i=0;i<200 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && grip_angle==225 && parallel_seen);
   CHECK(soft_calls==0 && dm_speed_dps==112.5f);
   CHECK(tick-began<5000);
   rotation_plant=0;motor.fb.vel_rads=0;place_test=0;
 }
 /* Reverse loaded transfer also uses the original speed. */
 {
   uint8_t ok;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
   motor.fb.pos_rad=zero_ref+162.8250f/57.2957795f;motor.fb.vel_rads=0;
   expected_reach=axes[1].zero_counts+(int64_t)round(-185.1581*65536.0/360.0);
   follow_dm=1;rotation_plant=1;
   CHECK(ArmStartPose_BeginMove(-361.56436f,-185.1581f,-2.0217f)==HAL_OK);
   for(i=0;i<200 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(ArmStartPose_ReadActionResult(&ok) && ok && soft_calls==0 && dm_speed_dps==112.5f);
   rotation_plant=0;motor.fb.vel_rads=0;
 }
 /* Failure of the original rotation request still stops the job. */
 {
   uint32_t before_stops=stops;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;motor.fb.pos_rad=zero_ref;follow_dm=0;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm place")==1);
   rotate_result=HAL_ERROR;
   for(i=0;i<20 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   rotate_result=HAL_OK;
   CHECK(s_phase==POSE_FAILED && stops>before_stops && soft_calls==0);
 }
 /* Already retracted: verify fresh hold while DM moves, without lowering early. */
 {
   uint32_t dm_before=dm_moves;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;motor.fb.pos_rad=zero_ref;follow_dm=0;
   axes[1].position_counts=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_REACH && s_place_track && dm_moves==dm_before+1);
   step();step();CHECK(dm_moves==dm_before+1 && s_turn_pending);
   step();CHECK(s_phase==POSE_REACH && s_place_track && s_turn_pending);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 /* Blocked/faulted retract stops the parallel job without lowering/releasing. */
 {
   uint32_t dm_before=dm_moves,opens_before=grip_open_calls;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;motor.fb.pos_rad=zero_ref;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   axes[1].position_counts=axes[1].zero_counts-50000;
   for(i=0;i<120 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_FAILED && dm_moves==dm_before+1 && grip_open_calls==opens_before);
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   axes[1].flags=0x85;
   for(i=0;i<25 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   if(ARM_28_STALL_IGNORE_TRIAL) {
     CHECK(ArmStartPose_IsActive() && s_phase!=POSE_FAILED && dm_moves==dm_before+2 && grip_open_calls==opens_before);
     ArmStartPose_Cancel(); /* Still waiting for the real end position, not a STALL failure. */
   } else CHECK(s_phase==POSE_FAILED && dm_moves==dm_before+2 && grip_open_calls==opens_before);
   axes[1].flags=0x81;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   ArmStartPose_Cancel();for(i=0;i<10;i++)step();
   CHECK(!ArmStartPose_IsActive() && !s_turn_pending && dm_moves==dm_before+3 && grip_open_calls==opens_before);
 }
 /* Logged duplicate ascent: material already at TOP0.41%, PLACE must hold it
  * without a fresh0% motion request before retracting. Test both signed sides. */
 {
   const float top_pct[]={0.41f,0.46f,-0.27f,0.0f,0.5991f,1.0460f,3.3f,3.4f};
   unsigned k;
   for(k=0;k<8;k++){
     uint32_t lifts_before=lift_commands;
     cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
     axes[0].position_counts=axes[0].zero_counts+(int64_t)round(top_pct[k]*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
     axes[1].position_counts=axes[1].zero_counts-70000;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;
     CHECK(ArmStartPose_Command("arm place")==1 && s_phase==POSE_TOP_ENABLE);
     cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_TOP);
     CHECK(lift_commands==lifts_before+(k==7?1U:0U)); /* TOP8mm:7.92mm holds,8.16mm raises. */
     CHECK(grip_angle==ARM_SERVO_GRIP_CLOSE_DEG);
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
   }
 }
 /* Retry owns the reach stall temporarily: no lowering/release until its
  * SAME target is resent and fresh STOP/hold verification completes. */
 {
   uint32_t lifts_before=lift_commands,opens_before=grip_open_calls;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-40000;
   motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=1;place_test=0;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   retry_recovering=reach_stall=1;
   for(i=0;i<5;i++){step();CHECK(s_phase==POSE_REACH && grip_open_calls==opens_before && lift_commands==lifts_before);}
   retry_recovering=reach_stall=0;retry_generation++;
   step();CHECK(s_phase==POSE_REACH && lift_commands==lifts_before);
   for(i=0;i<30 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT && grip_open_calls==opens_before && lift_commands==lifts_before+1);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
   CHECK(ArmStartPose_Command("arm place")==1);retry_failed=1;step();
   CHECK(s_phase==POSE_FAILED && grip_open_calls==opens_before);
 }
 /* Logged mid-stroke stall: preserve normal parallel PICK, but hold DM at
  * its actual in-flight angle during recovery. Resume only after reach STOP
  * AND new held samples, then wait DM before lowering/releasing. */
 {
   unsigned mode;
   for(mode=0;mode<3;mode++){
     uint32_t lifts_before=lift_commands,opens_before=grip_open_calls,dm_before;
     float held;
     ArmStartPose_Cancel();retry_recovering=retry_failed=reach_stall=0;
     cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;
     motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=rotation_plant=1;place_test=0;lag_on_native=0;
     axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-52000;
     expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
     CHECK(ArmStartPose_Command("arm pick")==1);opens_before=grip_open_calls;
     for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_REACH && fabsf(dm_target-162.825f)<0.001f && s_turn_pending);
     axes[1].position_counts=axes[1].zero_counts-47593;retry_recovering=reach_stall=1;step();
     CHECK(s_dm_wait_reach && s_turn_pending && s_phase==POSE_REACH);
     held=dm_target;CHECK(fabsf(held-162.825f)>1);
     for(i=0;i<5;i++){
       step();CHECK(fabsf(dm_target-held)<0.001f && fabsf((motor.fb.pos_rad-zero_ref)*57.2957795f-held)<0.01f);
       CHECK(s_phase==POSE_REACH && lift_commands==lifts_before && grip_open_calls==opens_before && grip_angle==225);
     }
     if(mode){
       dm_before=dm_moves;
       if(mode==1)ArmStartPose_Cancel();else{retry_failed=1;step();}
       for(i=0;i<5;i++)step();
       CHECK(!ArmStartPose_IsActive() && !s_dm_wait_reach && dm_moves==dm_before && grip_open_calls==opens_before);
       continue;
     }
     retry_recovering=reach_stall=0;retry_generation++;step();
     CHECK(s_dm_wait_reach && s_native_timeout==20000U);
     axes[1].position_counts=expected_reach;
     for(i=0;i<20 && s_dm_wait_reach;i++){
       cal_state=ARM_CAL_IDLE;step();CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
     }
     CHECK(!s_dm_wait_reach && !native_inflight && s_turn_pending && fabsf(dm_target-162.825f)<0.001f);
     CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
     for(i=0;i<50 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1 && grip_open_calls==opens_before);
     for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_READY && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG && grip_open_calls==opens_before);
   }
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* Parallel PLACE from all three source orientations; recovery pauses DM
  * at its actual in-flight angle until reach STOP/new-held verification. */
 {
   const float source_deg[]={40.9044f,-2.0217f,-42.5218f};unsigned k;
   for(k=0;k<3;k++){
     uint32_t lifts_before=lift_commands,opens_before=grip_open_calls;float held;
     ArmStartPose_Cancel();retry_recovering=retry_failed=reach_stall=0;
     cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
     motor.fb.pos_rad=zero_ref+source_deg[k]/57.2957795f;motor.fb.vel_rads=0;follow_dm=rotation_plant=1;place_test=0;
     axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
     expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
     CHECK(ArmStartPose_Command("arm place")==1);
     for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_REACH && !s_dm_wait_reach && s_turn_pending && fabsf(dm_target-162.825f)<0.01f);
     /* Exercise confirmed recovery while DM is already turning. */
     axes[1].position_counts=axes[1].zero_counts-91953;retry_recovering=reach_stall=1;
     step();held=dm_target;CHECK(s_dm_wait_reach);
     for(i=0;i<4;i++){step();CHECK(fabsf(dm_target-held)<0.01f && lift_commands==lifts_before && grip_open_calls==opens_before);}
     retry_recovering=reach_stall=0;retry_generation++;step();axes[1].position_counts=expected_reach;
     for(i=0;i<20 && s_dm_wait_reach;i++){
       CHECK(fabsf((motor.fb.pos_rad-zero_ref)*57.2957795f-held)<0.01f);
       cal_state=ARM_CAL_IDLE;step();CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
     }
     CHECK(!s_dm_wait_reach && native_inflight && s_turn_pending && fabsf(dm_target-162.825f)<0.001f);
     for(i=0;i<100 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1 && grip_open_calls==opens_before);
     CHECK(llabs(axes[0].position_counts-axes[0].zero_counts+73767)<=2); /*V127 tray release up5mm; PICK unchanged*/
     for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_READY && grip_angle==225);
   }
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* Pending Cgi observation must not issue the recovery DM HOLD command.
  * Normal PICK target continues; arrival/release waits for observation clear. */
 {
   uint32_t dm_before,lifts_before=lift_commands;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;
   motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;rotation_plant=follow_dm=1;place_test=0;lag_on_native=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm pick")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   dm_before=dm_moves;axes[1].position_counts=axes[1].zero_counts-92973;stall_pending=reach_stall=1;
   step();step();CHECK(s_phase==POSE_REACH && !s_dm_wait_reach && dm_moves==dm_before && fabsf(dm_target-162.825f)<0.001f);
   CHECK(lift_commands==lifts_before && grip_angle==225);
   axes[1].position_counts=expected_reach;step();CHECK(s_native && !s_dm_wait_reach && grip_angle==225);
   stall_pending=reach_stall=0;
   for(i=0;i<80 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG && dm_moves==dm_before);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* V87 field regression: native STOP/hold already passed and PLACE DM is
  * turning when reach drifts and asserts Cgi. Keep managed recovery alive
  * even though s_native was cleared; do not lower/open during reverse. */
 {
   uint32_t lifts_before=lift_commands,opens_before=grip_open_calls,dm_before;
   float paused;
   ArmStartPose_Cancel();retry_failed=retry_recovering=reach_stall=stall_pending=0;
   cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
   motor.fb.pos_rad=zero_ref-2.0217f/57.2957795f;motor.fb.vel_rads=0;
   follow_dm=rotation_plant=1;place_test=0;lag_on_native=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-52000;
   expected_reach=axes[1].zero_counts+(int64_t)round(-4.0*65536.0/360.0);
   CHECK(ArmStartPose_Command("arm place")==1);
   for(i=0;i<30 && !(s_phase==POSE_REACH && s_place_track && !s_dm_wait_reach && s_turn_pending);i++){
     cal_state=ARM_CAL_IDLE;step();
   }
   CHECK(s_phase==POSE_REACH && s_place_track && s_turn_pending && cache_active);
   CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
   axes[1].position_counts=axes[1].zero_counts-2109; /* actual -11.585deg */
   stall_pending=reach_stall=1;dm_before=dm_moves;
   step();step();
   CHECK(s_phase==POSE_REACH && !s_dm_wait_reach && dm_moves==dm_before);
   CHECK(lift_commands==lifts_before && grip_open_calls==opens_before && cache_active);
   stall_pending=0;retry_recovering=1;step();paused=dm_target;
   CHECK(s_phase==POSE_REACH && s_native && s_dm_wait_reach && paused<162.0f);
   /* Simulate actual reverse away from original goal. The retry owner may
    * move opposite normal direction without the pose owner aborting it. */
   axes[1].position_counts-=1490;
   for(i=0;i<4;i++){
     step();CHECK(s_phase==POSE_REACH && s_dm_wait_reach && fabsf(dm_target-paused)<0.001f);
     CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
   }
   retry_recovering=reach_stall=0;retry_generation++;step();
   axes[1].position_counts=expected_reach;
   for(i=0;i<25 && s_dm_wait_reach;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(!s_dm_wait_reach && s_turn_pending && fabsf(dm_target-162.825f)<0.001f);
   CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
   for(i=0;i<50 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT && cache_active && lift_commands==lifts_before+1);
   for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && grip_angle==225 && grip_open_calls==opens_before+1);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* Ring retrieval lowered8mm from V89, now8mm BELOW release.
  * It opens before repositioning and closes only at actual three-axis target. */
 {
   const float reach[]={-532.9303f,-162.9795f,-249.5599f},dm[]={40.9044f,2.6040f,-42.5218f};
   unsigned slot;uint8_t success;
   retry_failed=retry_recovering=reach_stall=0;follow_dm=1;place_test=0;lag_on_native=0;
   for(slot=1;slot<=3;slot++){
     start_profile_t saved=s_saved;uint32_t closes=grip_close_calls;
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;
     motor.fb.vel_rads=0;axes[0].position_counts=axes[0].zero_counts;
     expected_reach=axes[1].zero_counts+(int64_t)round(reach[slot-1]*65536.0/360.0);
     close_dm_expected=dm[slot-1];close_lift_expected=-198958;
     CHECK(ArmStartPose_BeginRingPick(slot)==HAL_OK && s_phase==POSE_GRIP_OPEN && s_pick==4 && grip_angle==225);
     CHECK(!memcmp(&saved,&s_saved,sizeof(saved)));
     for(i=0;i<100 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_READY && !ArmStartPose_IsReady() && s_pick_completed);
     CHECK(ArmStartPose_ReadActionResult(&success) && success && grip_close_calls==closes+1 && ArmServo_GripIsClosed());
     CHECK(llabs(axes[0].position_counts-axes[0].zero_counts+198958)<=2);
     CHECK(ArmStartPose_BeginRingPick(slot)==HAL_ERROR); /* cannot overwrite held item */
   }
   ArmStartPose_Cancel();grip_angle=225;cal_state=ARM_CAL_IDLE;
   CHECK(ArmStartPose_BeginRingPick(0)==HAL_ERROR && ArmStartPose_BeginRingPick(4)==HAL_ERROR);
 }
 /* Numbered tray PLACE uses independent reach/DM offsets, same final
  * height, and leaves START/PICK/rings untouched. It must not open before
  * both axes arrive and the unchanged final lift endpoint is confirmed. */
 {
   const float reach[3]={1.0f,1.0f,1.0f};
   const float dm[3]={162.8250f,162.8250f,162.8250f};unsigned slot;char cmd[24];
   for(slot=1;slot<=3;slot++) {
     start_profile_t saved=s_saved;uint32_t opens=grip_open_calls;
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
     motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=rotation_plant=1;place_test=0;lag_on_native=0;
     axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
     expected_reach=axes[1].zero_counts+(int64_t)round(reach[slot-1]*65536.0/360.0);
     snprintf(cmd,sizeof(cmd),"arm place %u",slot);CHECK(ArmStartPose_Command(cmd)==1 && s_place_slot==slot && s_pick==2);
     CHECK(!memcmp(&saved,&s_saved,sizeof(saved)));
     for(i=0;i<100 && s_phase!=POSE_LIFT;i++){
       cal_state=ARM_CAL_IDLE;step();CHECK(grip_open_calls==opens);
     }
     CHECK(s_phase==POSE_LIFT && llabs(axes[0].position_counts-axes[0].zero_counts+73767)<=2);
     CHECK(fabsf(dm_target-dm[slot-1])<0.001f && llabs(axes[1].position_counts-expected_reach)<=12);
     for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_READY && grip_open_calls==opens+1 && grip_angle==225);
   }
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
   CHECK(ArmStartPose_Command("arm place 0")==1 && !ArmStartPose_IsActive());
   CHECK(ArmStartPose_Command("arm place 4")==1 && !ArmStartPose_IsActive());
   CHECK(ArmStartPose_Command("arm place 1 extra")==1 && !ArmStartPose_IsActive());
 }
 /* Tray PWM may run during approach, but the actual final lift descent
  * and release cannot start until it has settled at the requested slot. */
 {
   unsigned mode;
   for(mode=0;mode<4;mode++){
     uint32_t lifts_before,opens_before;
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.ramping=0;
     motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=rotation_plant=1;place_test=0;lag_on_native=0;
     axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
     memset(&table_status,0,sizeof(table_status));table_status.enabled=1;table_status.target_us=1639;table_status.output_us=1500;
     table_status.ramping=mode==3?0:1;lifts_before=lift_commands;opens_before=grip_open_calls;
     CHECK(ArmStartPose_Command("arm place 1")==1);
     CHECK(ArmStartPose_WaitForTray(1639,mode==3?tick-8000:tick)==HAL_OK);
     for(i=0;i<70 && s_phase!=POSE_LIFT && (s_phase!=POSE_REACH || s_turn_pending || s_dm_wait_reach);i++){
       cal_state=ARM_CAL_IDLE;step();CHECK(grip_open_calls==opens_before);
     }
     if(mode==3){CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1);ArmStartPose_Cancel();continue;}
     CHECK(s_phase==POSE_REACH && lift_commands==lifts_before && s_tray_gate);
     step();step();CHECK(lift_commands==lifts_before && grip_open_calls==opens_before);
     if(mode==0){
       table_status.ramping=0;table_status.output_us=1639;step();
       CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1 && !s_tray_gate);
       for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
       CHECK(s_phase==POSE_READY && grip_angle==225);
     }else if(mode==1){
       table_status.target_us=2139;step();CHECK(s_phase==POSE_FAILED && lift_commands==lifts_before && grip_open_calls==opens_before);
     }else{
       ArmStartPose_Cancel();table_status.ramping=0;step();CHECK(!s_tray_gate && lift_commands==lifts_before && grip_open_calls==opens_before);
     }
   }
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* The same descent gate applies to automatic tray PICK, before closing. */
 {
   uint32_t lifts_before,closes_before;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;
   close_dm_expected=162.8250f;close_lift_expected=-87486;
   motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=rotation_plant=1;place_test=0;lag_on_native=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   table_status.enabled=1;table_status.fault=0;table_status.target_us=2139;table_status.ramping=1;
   lifts_before=lift_commands;closes_before=grip_close_calls;
   CHECK(ArmStartPose_Command("arm pick")==1 && ArmStartPose_WaitForTray(2139,tick)==HAL_OK);
   for(i=0;i<50 && (s_phase!=POSE_REACH || s_native || s_turn_pending);i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_REACH && lift_commands==lifts_before && grip_close_calls==closes_before);
   table_status.ramping=0;step();CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1);
   for(i=0;i<30 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && grip_close_calls==closes_before+1 && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* Ring drop, ring retrieval and supply waiting poses now start reach
  * before DM arrives; recovery pauses DM and BOTH must settle before lift. */
 {
   unsigned mode;
   for(mode=0;mode<3;mode++) {
     uint32_t lifts_before;float target=mode==0?90.0f:40.9044f;
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;retry_recovering=retry_failed=reach_stall=0;
     grip_status.enabled=1;grip_status.ramping=0;grip_angle=mode==1?ARM_SERVO_GRIP_CLOSE_DEG:225;s_pick_completed=(uint8_t)(mode==1);
     motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=rotation_plant=0;place_test=0;lag_on_native=0;
     axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-40000;
     expected_reach=axes[1].zero_counts+(mode==0?-32768:mode==1?-95527:-97017);
     if(mode==0)CHECK(ArmStartPose_BeginMove(-200,-180,90)==HAL_OK);
     else if(mode==1)CHECK(ArmStartPose_BeginRingDrop(1,0)==HAL_OK);
     else CHECK(ArmStartPose_BeginRingPick(1)==HAL_OK);
     for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_REACH && native_inflight && s_turn_pending && fabsf(dm_target-target)<0.001f);
     lifts_before=lift_commands;retry_recovering=reach_stall=1;step();
     CHECK(s_dm_wait_reach && fabsf(dm_target)<0.001f && lift_commands==lifts_before);
     retry_recovering=reach_stall=0;retry_generation++;step();axes[1].position_counts=expected_reach;
     for(i=0;i<20 && s_dm_wait_reach;i++){cal_state=ARM_CAL_IDLE;step();CHECK(lift_commands==lifts_before);}
     CHECK(!s_dm_wait_reach && s_turn_pending && fabsf(dm_target-target)<0.001f);
     for(i=0;i<4;i++){cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_REACH && lift_commands==lifts_before);}
     follow_dm=1;
     for(i=0;i<20 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
     CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1);
   }
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;rotation_plant=0;
 }
 /* A known settled OPEN skips the repeat command/delay for both tray and
  * ring pickup; an unconfirmed OPEN still waits, even if PWM equals target. */
 {
   unsigned mode;
   for(mode=0;mode<2;mode++) {
     uint32_t opens=grip_open_calls;
     ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;grip_open_confirmed=1;
     motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;follow_dm=1;axes[0].position_counts=axes[0].zero_counts;
     if(mode==0)CHECK(ArmStartPose_Command("arm pick")==1);
     else CHECK(ArmStartPose_BeginRingPick(2)==HAL_OK);
     CHECK(s_phase==POSE_TOP_ENABLE && grip_open_calls==opens);ArmStartPose_Cancel();
   }
   grip_open_confirmed=0;grip_angle=225;CHECK(ArmStartPose_Command("arm pick")==1 && s_phase==POSE_GRIP_OPEN);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 /* Second TEMP placement raises the current ground endpoint72mm; other
  * ring targets and later ground requests cannot inherit that layer offset. */
 {
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.enabled=1;grip_status.ramping=0;
   close_dm_expected=162.8250f;close_lift_expected=-87486;motor.fb.vel_rads=0;follow_dm=1;rotation_plant=0;
   CHECK(ArmStartPose_Command("arm pick")==1);
   for(i=0;i<80 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_pick_completed && ArmServo_GripIsClosed());expected_reach=axes[1].zero_counts-51391;
   CHECK(ArmStartPose_BeginRingDrop(3,72.0f)==HAL_OK);
   for(i=0;i<100 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && grip_angle==225 && llabs(axes[0].position_counts-axes[0].zero_counts+122131)<=2);
   CHECK(fabsf(dm_target+39.5218f)<0.001f && llabs(axes[1].position_counts-expected_reach)<=12);
   CHECK(ArmStartPose_Command("arm ring status")==1 && strstr(last_console,"5=-670.")!=NULL);
   CHECK(ArmStartPose_BeginRingDrop(3,65.0f)==HAL_ERROR); /* cannot stack empty handed */
   CHECK(ArmStartPose_Command("arm pick")==1);
   for(i=0;i<80 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   expected_reach=axes[1].zero_counts-51391;CHECK(ArmStartPose_BeginRingDrop(3,0.0f)==HAL_OK);
   for(i=0;i<100 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && llabs(axes[0].position_counts-axes[0].zero_counts+187983)<=2);
   CHECK(ArmStartPose_BeginRingDrop(4,65.0f)==HAL_ERROR && ArmStartPose_BeginRingDrop(3,-1.0f)==HAL_ERROR);
 }
 /* V129: both TEMP layers ring1/3 extend2mm more than ROUGH. Targets are
  * absolute per move; middle ring/retrieval and layer heights unchanged. */
 {unsigned slot,layer;for(slot=1;slot<=3;slot++)for(layer=0;layer<2;layer++) {
   const float base_reach[3]={-512.4668f,-183.442985f,-282.3015f};
   const float base_dm[3]={39.9044f,2.6040f,-41.5218f};
   float reach=base_reach[slot-1]-(slot==1?3.0f*4.09269438f:0.0f);
   float dm=base_dm[slot-1]+(slot==1?1.0f:slot==3?2.0f:0.0f); /* V129 another -0.5deg */
   float temp_reach=reach-(slot==2?0.0f:2.0f*4.09269438f);
   float actual,lift;uint8_t success=0;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_pick_completed=1;
   grip_status.enabled=1;grip_status.fault=grip_status.ramping=0;
   axes[0].flags=axes[1].flags=0x81;axes[0].position_counts=axes[0].zero_counts;
   follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;lag_on_native=0;place_test=0;
   expected_reach=axes[1].zero_counts+(int64_t)round(temp_reach*65536.0/360.0);
   CHECK(ArmStartPose_BeginTempRingDrop(slot,layer?72.0f:0.0f)==HAL_OK);
   actual=s_run.percent[1]*s_run.span[1]/100.0f*360.0f/65536.0f;
   lift=s_run.percent[0]*s_run.span[0]/100.0f*360.0f/65536.0f;
   CHECK(fabsf(actual-temp_reach)<0.001f && fabsf(s_run.dm_deg-dm)<0.001f);
   CHECK(fabsf(lift-(layer?-670.8873f:-1032.6204f))<0.002f);
   for(i=0;i<100 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(ArmStartPose_ReadActionResult(&success) && success && grip_angle==225);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_pick_completed=1;
   expected_reach=axes[1].zero_counts+(int64_t)round(reach*65536.0/360.0);
   CHECK(ArmStartPose_BeginRingDrop(slot,0)==HAL_OK);
   actual=s_run.percent[1]*s_run.span[1]/100.0f*360.0f/65536.0f;
   CHECK(fabsf(actual-reach)<0.001f && fabsf(s_run.dm_deg-dm)<0.001f);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }}
 /* 300ms close minimum; ramp completion still required. Open/release
  * keeps500ms and PICK cannot be marked complete at200ms. */
 {unsigned mode;for(mode=0;mode<2;mode++) {
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_status.enabled=1;
   grip_status.ramping=grip_status.fault=0;grip_angle=225;grip_open_confirmed=1;
   follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;place_test=0;lag_on_native=0;
   if(mode==0){close_dm_expected=162.8250f;close_lift_expected=-87486;CHECK(ArmStartPose_Command("arm pick")==1);}
   else {
     close_dm_expected=40.9044f;close_lift_expected=-198958;
     expected_reach=axes[1].zero_counts+(int64_t)round(-532.9303*65536.0/360.0);
     CHECK(ArmStartPose_BeginRingPick(1)==HAL_OK);
   }
   for(i=0;i<100 && s_phase!=POSE_GRIP_CLOSE && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_GRIP_CLOSE && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG);
   step();step();CHECK(s_phase==POSE_GRIP_CLOSE && !s_pick_result_valid);
   if(mode==0){step();CHECK(s_phase==POSE_READY && s_pick_result_valid);} /* exactly300ms */
   else {grip_status.ramping=1;step();CHECK(s_phase==POSE_GRIP_CLOSE);
     grip_status.ramping=0;step();CHECK(s_phase==POSE_READY && s_pick_result_valid);}
 }}
 /* Closed grasp -> rise; DM/reach cannot start above half remaining
  * height. Once half is crossed, lift is STILL moving while native reach
  * and DM start, and final descent waits TOP plus their held feedback. */
 {unsigned mode;for(mode=0;mode<4;mode++) {
   uint32_t dm_before,lifts_before,native_before;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;place_test=0;
   grip_status.enabled=1;grip_status.fault=0;grip_status.ramping=0;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.75*ARM_LIFT_FULL_TRAVEL_COUNTS);
   axes[1].position_counts=axes[1].zero_counts-30000;
   expected_reach=axes[1].zero_counts+(int64_t)round(65536.0/360.0);
   follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;motor.fb.pos_rad=zero_ref;
   dm_before=dm_moves;lifts_before=lift_commands;native_before=linear_moves;
   if(mode==3) {
     s_pick_completed=1;expected_reach=axes[1].zero_counts+(int64_t)round(-524.7449*65536.0/360.0);
     CHECK(ArmStartPose_BeginRingDrop(1,0)==HAL_OK);
   } else CHECK(ArmStartPose_Command("arm place")==1);
   step();
   CHECK(s_phase==POSE_TOP && s_raise_overlap && cal_state==ARM_CAL_PERCENT_MOVE && lift_commands==lifts_before+1);
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.60*ARM_LIFT_FULL_TRAVEL_COUNTS);step();
   CHECK(s_phase==POSE_TOP && dm_moves==dm_before);
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.38*ARM_LIFT_FULL_TRAVEL_COUNTS);step();
   CHECK(s_phase==POSE_TOP && dm_moves==dm_before);
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.37*ARM_LIFT_FULL_TRAVEL_COUNTS);step();step();
   CHECK(s_phase==POSE_REACH && cal_state==ARM_CAL_PERCENT_MOVE && dm_moves==dm_before+1 && linear_moves>native_before+1);
   if(mode==1){ArmStartPose_Cancel();CHECK(!ArmStartPose_IsActive() && !s_raise_overlap);continue;}
   if(mode==2){cal_state=ARM_CAL_FAILED;step();CHECK(s_phase==POSE_FAILED);continue;}
   for(i=0;i<8;i++){step();}
   CHECK(lift_commands==lifts_before+1 && ArmStartPose_IsActive());
   axes[0].position_counts=axes[0].zero_counts;cal_state=ARM_CAL_IDLE;
   for(i=0;i<80 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   {uint8_t success=0;CHECK(ArmStartPose_ReadActionResult(&success) && success && !s_raise_overlap && grip_angle==225);}
   CHECK(lift_commands==lifts_before+2);
 }}
 /* The owner can adopt an already-issued TOP raise without another lift
  * command or replacing START, but not another axis/target or an open claw. */
 {
   uint32_t before;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.8*ARM_LIFT_FULL_TRAVEL_COUNTS);
   CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(0.3*ARM_LIFT_FULL_TRAVEL_COUNTS);
   before=lift_commands;CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(2,80)==HAL_OK);
   CHECK(s_phase==POSE_TOP && s_raise_overlap && s_raise_half_percent==40 && lift_commands==before);
   CHECK(ArmStartPose_Command("arm start status")==1 && strstr(last_console,"DM=+134.6173deg"));
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
   CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(2,80)==HAL_BUSY);
   cal_state=ARM_CAL_PERCENT_MOVE;cal_axis=ARM_DRIVER_REACH;
   CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(2,80)==HAL_BUSY);
   cal_axis=ARM_DRIVER_LIFT;cal_lift_target=20;
   CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(2,80)==HAL_BUSY);
   cal_lift_target=0;grip_angle=225;
   CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(2,80)==HAL_ERROR);
   CHECK(ArmStartPose_BeginTrayPlaceDuringRaise(0,80)==HAL_ERROR && ArmStartPose_BeginTrayPlaceDuringRaise(2,NAN)==HAL_ERROR);
 }
 /* Placement tracking runs while DM turns AND while lift lowers. No STOP until release,
  * no percent6 job, no repeated relative travel; current precision gates
  * release rather than an extra stable-sample stage before descent. */
 {
   uint32_t before_corrections,before_stop,before_percent,before_opens;int64_t lag;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;track_result=HAL_OK;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;
   grip_status.enabled=1;grip_status.fault=grip_status.ramping=0;axes[0].flags=axes[1].flags=0x81;
   follow_dm=0;rotation_plant=0;place_test=0;lag_on_native=0;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_place_track && s_turn_pending);
   before_corrections=track_corrections;before_stop=reach_stop_calls;before_percent=encoder_reach_moves;
   lag=(int64_t)round(1.2f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);axes[1].position_counts+=lag;
   for(i=0;i<5;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_REACH && s_turn_pending && track_corrections>before_corrections);
   CHECK(reach_stop_calls==before_stop && encoder_reach_moves==before_percent);
   CHECK(llabs(axes[1].position_counts-expected_reach)<=12);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;follow_dm=1;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;
   grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<20 && s_phase!=POSE_REACH;i++){cal_state=ARM_CAL_IDLE;step();}
   before_opens=grip_open_calls;before_stop=reach_stop_calls;
   axes[1].position_counts=axes[1].zero_counts-70000; /* Still traveling inward, far from the goal. */
   for(i=0;i<10 && s_phase!=POSE_LIFT;i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT && s_place_track && cache_active && grip_open_calls==before_opens);
   cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_LIFT && grip_open_calls==before_opens);
   axes[1].position_counts=expected_reach+(int64_t)round(0.4f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
   step();CHECK(s_phase==POSE_GRIP_CLOSE && grip_open_calls==before_opens+1); /* ONE current sample, no dwell. */
   before_corrections=track_corrections;
   for(i=0;i<12 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && s_pick_result_valid && track_corrections==before_corrections);
   CHECK(reach_stop_calls==before_stop+1 && !s_place_track && !cache_active && !native_inflight);
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 /* Field04:09:58:6 stalls DURING final descent. Hold5/DM and preserve
  * CLOSED grip; do not cancel native retry. Recover actual6 first and
  * resume only the recorded final lift target, then open once. */
 {unsigned mode;for(mode=0;mode<6;mode++) {
   uint32_t holds_before,lifts_before,opens_before;float held_dm;int32_t goal;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;reject_lift_hold=0;retry_failed=retry_recovering=reach_stall=0;
   grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;grip_status.fault=grip_status.ramping=0;
   axes[0].flags=axes[1].flags=0x81;axes[0].position_counts=axes[0].zero_counts;
   axes[1].position_counts=axes[1].zero_counts-97017;follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;place_test=0;lag_on_native=0;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<30 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT);lifts_before=lift_commands;holds_before=lift_holds;opens_before=grip_open_calls;goal=cached_abs_goal;
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(s_run.percent[0]*(mode==1?1.0:0.7)*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
   if(mode==1)cal_state=ARM_CAL_IDLE; /* already arrived case from field log */
   retry_recovering=reach_stall=1;follow_dm=0;
   motor.fb.pos_rad=zero_ref+162.425f/57.2957795f;
   if(mode==3)reject_lift_hold=1;
   step();
   if(mode==3){CHECK(s_phase==POSE_FAILED && grip_open_calls==opens_before);reject_lift_hold=0;continue;}
   CHECK(s_phase==POSE_REACH && s_dm_wait_reach && cal_state==ARM_CAL_IDLE && lift_holds==holds_before+1);
   CHECK(s_lower_recovery && s_lower_hold_pending); /* Must anchor to fresh STOP feedback, not the pre-STOP position. */
   held_dm=dm_target;CHECK(fabsf(held_dm-162.425f)<0.001f && cache_active && cached_abs_goal==goal);
   for(i=0;i<4;i++){axes[1].position_counts-=200;step();CHECK(s_phase==POSE_REACH && dm_target==held_dm && grip_angle==ARM_SERVO_GRIP_CLOSE_DEG && grip_open_calls==opens_before && lift_commands==lifts_before);}
   CHECK(!s_lower_hold_pending);
   if(mode==2){ArmStartPose_Cancel();CHECK(!ArmStartPose_IsActive() && !cache_active);continue;}
   if(mode==4){retry_failed=1;step();CHECK(s_phase==POSE_FAILED && !s_pick_result_valid && grip_open_calls==opens_before);retry_failed=0;continue;}
   if(mode==5){axes[0].flags=0x89;for(i=0;i<21;i++)step();CHECK(s_phase==POSE_FAILED && grip_open_calls==opens_before);axes[0].flags=0x81;continue;}
   retry_recovering=reach_stall=0;retry_generation++;
   axes[1].position_counts=expected_reach+(int64_t)round(1.0f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
   step();step();CHECK(s_phase==POSE_REACH && s_dm_wait_reach && lift_commands==lifts_before && grip_open_calls==opens_before);
   axes[1].position_counts=expected_reach;step();CHECK(!s_dm_wait_reach && s_turn_pending && fabsf(dm_target-162.825f)<0.001f);
   follow_dm=1;
   for(i=0;i<15 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++)step();
   CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+1 && cal_lift_target==s_run.percent[0] && grip_open_calls==opens_before && cached_abs_goal==goal);
   if(mode==0) {
     /* A second stall still uses the SAME cached reach job rather than
      * resetting its attempts. The driver production tests enforce3 total. */
     retry_recovering=reach_stall=1;step();CHECK(s_phase==POSE_REACH && s_dm_wait_reach && lift_holds==holds_before+2 && cached_abs_goal==goal);
     step();step();retry_recovering=reach_stall=0;retry_generation++;axes[1].position_counts=expected_reach;
     step();CHECK(!s_dm_wait_reach && s_turn_pending);for(i=0;i<10 && s_phase!=POSE_LIFT;i++)step();
     CHECK(s_phase==POSE_LIFT && lift_commands==lifts_before+2 && grip_open_calls==opens_before && cached_abs_goal==goal);
   }
   for(i=0;i<15 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_READY && s_pick_result_valid && grip_open_calls==opens_before+1);
 }}
 reject_lift_hold=retry_failed=retry_recovering=reach_stall=0;axes[0].flags=axes[1].flags=0x81;follow_dm=1;
 /* Field01:46:38 OPEN at0.460mm; thereafter0.505..0.753mm residual never
  * gets corrected. It must complete after settled open, not false PWM timeout.
  * Before OPEN, residual beyond5deg must block release. Within5deg no
  * redundant native reassertions are needed before or after OPEN. */
 {
   uint8_t success=0;uint32_t opened,before_corrections;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;
   grip_status.fault=grip_status.ramping=0;axes[0].flags=axes[1].flags=0x81;
   follow_dm=1;rotation_plant=0;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;lag_on_native=0;place_test=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<30 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT);opened=grip_open_calls;track_follows=0U;
   axes[1].position_counts=expected_reach+(int64_t)round(1.40f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
   cal_state=ARM_CAL_IDLE;step();step();CHECK(s_phase==POSE_LIFT && grip_open_calls==opened);
   axes[1].position_counts=expected_reach+(int64_t)round(0.46f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
   step();CHECK(s_phase==POSE_GRIP_CLOSE && grip_open_calls==opened+1);
   before_corrections=track_corrections;
   axes[1].position_counts=expected_reach+(int64_t)round(0.70f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
   for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
   CHECK(s_phase==POSE_READY && s_pick_result_valid && track_corrections==before_corrections);
   CHECK(ArmStartPose_ReadActionResult(&success) && success && grip_open_calls==opened+1);
   CHECK(fabs((double)(axes[1].position_counts-expected_reach)/ARM_REACH_FULL_TRAVEL_COUNTS*178.0)>0.5);
   track_follows=1U;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 /* V186 placement-only5deg dead zone: both signs, tray/ring, exact field
  * residual-1.785 vs+1deg. Never reassert inside; outside still blocks OPEN. */
 {unsigned kind,side;for(kind=0;kind<2;kind++)for(side=0;side<3;side++) {
   uint32_t opened,before;uint8_t success=0;float residual=side==0?4.9f:side==1?-4.9f:5.1f;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_pick_completed=1;
   grip_status.enabled=1;grip_status.fault=grip_status.ramping=0;axes[0].flags=axes[1].flags=0x83;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-50000;
   follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;motor.fb.err_code=1;place_test=0;lag_on_native=0;
   track_follows=1;track_result=HAL_OK;retry_failed=retry_recovering=reach_stall=0;
   if(kind==0)CHECK(ArmStartPose_Command("arm place 1")==1);
   else {
     expected_reach=axes[1].zero_counts+(int64_t)round(-524.744906741573*65536.0/360.0);
     CHECK(ArmStartPose_BeginRingDrop(1,0)==HAL_OK);
   }
   for(i=0;i<40 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT && s_place_track);track_follows=0;opened=grip_open_calls;before=track_corrections;
   axes[1].position_counts=expected_reach+(int64_t)round(residual*65536.0/360.0);
   cal_state=ARM_CAL_IDLE;step();
   if(side<2) {
     CHECK(s_phase==POSE_GRIP_CLOSE && grip_open_calls==opened+1 && track_corrections==before);
     CHECK(!native_inflight && !cache_active && !s_place_track); /* Accepted residual ends ABS before release. */
     for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
     CHECK(ArmStartPose_ReadActionResult(&success) && success && s_phase==POSE_READY && track_corrections==before);
   } else {
     CHECK(s_phase==POSE_LIFT && grip_open_calls==opened);
     for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
     CHECK(track_corrections>before && grip_open_calls==opened);
     tick=s_place_track_started+s_native_timeout+1U;step();
     CHECK(s_phase==POSE_FAILED && grip_open_calls==opened);
   }
   track_follows=1;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }}
 {
   uint32_t before;uint8_t success=0;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;
   axes[0].flags=axes[1].flags=0x83;axes[0].position_counts=axes[0].zero_counts;
   axes[1].position_counts=axes[1].zero_counts-97017;follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<40 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT);track_follows=0;before=track_corrections;
   axes[1].position_counts=axes[1].zero_counts-325;cal_state=ARM_CAL_IDLE;step();
   CHECK(s_phase==POSE_GRIP_CLOSE && track_corrections==before);
   for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
   CHECK(ArmStartPose_ReadActionResult(&success) && success && track_corrections==before);
   track_follows=1;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 /* PICK and generic moves retain the original0.5percent reach requirement. */
 /* STOP rejection must not release a held item. The driver command has
  * to succeed before canceling its retry owner or opening the grip. */
 {
   uint32_t opened;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.ramping=0;
   axes[0].flags=axes[1].flags=0x83;axes[0].position_counts=axes[0].zero_counts;
   axes[1].position_counts=axes[1].zero_counts-97017;follow_dm=1;motor.fb.vel_rads=0;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<40 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT);opened=grip_open_calls;reach_stop_result=HAL_ERROR;
   cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_FAILED && grip_open_calls==opened);
   reach_stop_result=HAL_OK;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }
 {unsigned mode;for(mode=0;mode<2;mode++) {
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;grip_status.ramping=0;
   axes[0].flags=axes[1].flags=0x81;follow_dm=1;motor.fb.vel_rads=0;
   if(mode==0)CHECK(ArmStartPose_Command("arm pick")==1);
   else CHECK(ArmStartPose_BeginMove(-30,-200,10)==HAL_OK);
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(s_run.percent[0]*s_run.span[0]/100.0);
   axes[1].position_counts=axes[1].zero_counts+(int64_t)round(s_run.percent[1]*s_run.span[1]/100.0+3.9*65536.0/360.0);
   motor.fb.pos_rad=zero_ref+s_run.dm_deg/57.2957795f;motor.fb.enabled=1;motor.fb.err_code=1;
   CHECK(!Pose_TargetReady());ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }}
 /* The field+25.219deg excursion still trips range protection before
  * OPEN; removing the post-release precision gate cannot waive that guard.
  * More than2mm held drift after OPEN must also fail, not force DONE. */
 {unsigned mode;for(mode=0;mode<2;mode++) {
   uint8_t success=1;uint32_t opened;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;grip_status.enabled=1;
   grip_status.fault=grip_status.ramping=0;axes[0].flags=axes[1].flags=0x81;
   follow_dm=1;rotation_plant=0;motor.fb.pos_rad=zero_ref;motor.fb.vel_rads=0;place_test=0;lag_on_native=0;
   axes[0].position_counts=axes[0].zero_counts;axes[1].position_counts=axes[1].zero_counts-97017;
   CHECK(ArmStartPose_Command("arm place 1")==1);
   for(i=0;i<30 && s_phase!=POSE_LIFT && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_LIFT);track_follows=0U;opened=grip_open_calls;
   if(mode==0) {
     axes[1].position_counts=axes[1].zero_counts+(int64_t)round(25.219*65536.0/360.0);
     cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_FAILED && grip_open_calls==opened);
   } else {
     cal_state=ARM_CAL_IDLE;step();CHECK(s_phase==POSE_GRIP_CLOSE && grip_open_calls==opened+1);
     axes[1].position_counts=expected_reach+(int64_t)round(2.30f/178.0f*ARM_REACH_FULL_TRAVEL_COUNTS);
     for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
     CHECK(s_phase==POSE_FAILED && !s_pick_result_valid);
   }
   CHECK(ArmStartPose_ReadActionResult(&success) && !success);
   track_follows=1U;ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
 }}
 /* Field TEMP ring3: ARRIVED85.64/86.13%, then opens and drifts just beyond
  * the old0.5% recheck. Validate bounded held state BEFORE DONE and freeze
  * the action result; never silently print DONE then report failure. */
 {unsigned mode;for(mode=0;mode<9;mode++) {
   uint8_t success=0;float p;uint32_t before_stops;
   ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;raw_guard_reject=0;
   grip_status.enabled=1;grip_status.fault=0;grip_status.ramping=0;grip_angle=ARM_SERVO_GRIP_CLOSE_DEG;s_pick_completed=1;
   axes[0].flags=axes[1].flags=0x81;follow_dm=1;rotation_plant=0;motor.fb.vel_rads=0;motor.fb.err_code=1;
   expected_reach=axes[1].zero_counts+(int64_t)round(-282.3015*65536.0/360.0);
   CHECK(ArmStartPose_BeginRingDrop(3,0)==HAL_OK && !s_pick_result_valid);
   for(i=0;i<80 && s_phase!=POSE_GRIP_CLOSE && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
   CHECK(s_phase==POSE_GRIP_CLOSE && grip_angle==225);
   grip_status.ramping=1;step();CHECK(!ArmStartPose_ReadActionResult(&success) && !s_pick_result_valid);
   p=s_run.percent[0]+0.60f; /*1.44mm: exceeds arrival band, within held2mm band.*/
   if(mode==1)p=s_run.percent[0]+3.0f/axes[0].travel_mm*100.0f;
   axes[0].position_counts=axes[0].zero_counts+(int64_t)round(p*ARM_LIFT_FULL_TRAVEL_COUNTS/100.0);
   if(mode==2)axes[1].position_counts+=(int64_t)round(3.0f/axes[1].travel_mm*ARM_REACH_FULL_TRAVEL_COUNTS);
   if(mode==3)axes[1].flags=0x89;
   if(mode==4)axes[1].flags=0x80;
   if(mode==5)raw_guard_reject=1;
   if(mode==6 || mode==7){follow_dm=0;motor.fb.pos_rad=zero_ref+(dm_target+(mode==6?1.5f:2.5f))/57.2957795f;}
   grip_finish_failure[0]=0;before_stops=stops;grip_status.ramping=0;
   if(mode==8){tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;axes[0].position_tick=axes[0].flags_tick=motor.fb.last_tick=tick;ArmStartPose_Poll();}
   else for(i=0;i<6 && ArmStartPose_IsActive();i++)step();
   if(mode==0 || mode==6) {
     CHECK(s_phase==POSE_READY && s_pick_result_valid && stops==before_stops && !Pose_TargetReady());
     CHECK(ArmStartPose_ReadActionResult(&success) && success);
     axes[0].position_counts-=10;CHECK(ArmStartPose_ReadActionResult(&success) && success);
     ArmStartPose_Cancel();CHECK(ArmStartPose_ReadActionResult(&success) && !success && !s_pick_result_valid);
   } else {
     CHECK(s_phase==POSE_FAILED && !s_pick_result_valid && stops>before_stops);
     CHECK(ArmStartPose_ReadActionResult(&success) && !success && grip_finish_failure[0]);
   }
 }}
 raw_guard_reject=0;follow_dm=1;axes[0].flags=axes[1].flags=0x81;motor.fb.err_code=1;
 /* Fresh ISR feedback newer than the start-of-poll clock is not stale. */
 ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;grip_angle=225;motor.fb.vel_rads=0;step();
 expected_reach=axes[1].zero_counts+(int64_t)round(-200.0*65536.0/360.0);
 CHECK(ArmStartPose_BeginMove(-30,-200,10)==HAL_OK);dm_snapshot_delay=3;
 for(i=0;i<100 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
 {uint8_t success=0;CHECK(ArmStartPose_ReadActionResult(&success) && success);}
 dm_snapshot_delay=0;
 /* V144: DM can advance at50ms, never on cached feedback or at49ms. */
 ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;enabled=motor.fb.enabled=1U;
 motor.fb.err_code=1U;motor.fb.vel_rads=0;motor.fb.pos_rad=zero_ref+s_run.dm_deg/57.2957795f;
 axes[0].position_tick=axes[0].flags_tick=axes[1].position_tick=axes[1].flags_tick=motor.fb.last_tick=tick;
 s_phase=POSE_DM;s_generic=s_pick=s_turn_pending=0U;s_samples=0U;s_tick=tick;s_dm_seq=motor.fb.fb_count;
 for(i=0;i<5U;i++)ArmStartPose_Poll();
 CHECK(s_phase==POSE_DM && !s_samples);
 motor.fb.fb_count++;ArmStartPose_Poll();CHECK(s_phase==POSE_DM && s_samples==1U);
 tick+=49U;motor.fb.last_tick=tick;motor.fb.fb_count++;ArmStartPose_Poll();CHECK(s_phase==POSE_DM);
 tick+=1U;ArmStartPose_Poll();CHECK(s_phase==POSE_DM); /* no new frame */
 motor.fb.last_tick=tick;motor.fb.fb_count++;ArmStartPose_Poll();CHECK(s_phase==POSE_REACH_ENABLE);
 ArmStartPose_Cancel();
 /* V166: raw6 STALL is ignored by every pose view;5 STALL and6 protection
  * remain fatal. Actual raw flags are not rewritten by this decision. */
 {arm_cal_axis_status_t observed[2];
  axes[0].flags=0x81;axes[1].flags=0x85;
  axes[0].position_tick=axes[0].flags_tick=axes[1].position_tick=axes[1].flags_tick=tick;
  CHECK(Pose_Axes(observed,tick)==(ARM_28_STALL_IGNORE_TRIAL?1U:0U));
  CHECK(observed[1].flags==0x85);
  axes[1].flags=0x89;CHECK(!Pose_Axes(observed,tick));
  axes[1].flags=0x81;axes[0].flags=0x85;CHECK(!Pose_Axes(observed,tick));axes[0].flags=0x81;
 }
 /* V153: lift status2000ms, reach status800ms; both positions800ms. */
 {unsigned axis,j;arm_cal_axis_status_t snapshot[2];static const uint32_t ages[]={607U,800U,801U,807U,1999U,2000U,2001U};
  for(axis=0;axis<2;axis++){
   for(j=0;j<sizeof(ages)/sizeof(ages[0]);j++){
    axes[0].position_tick=axes[0].flags_tick=axes[1].position_tick=axes[1].flags_tick=tick;
    axes[axis].flags_tick=tick-ages[j];
    CHECK(Pose_Axes(snapshot,tick)==(ages[j]<=(axis?800U:2000U)));
   }
   axes[0].position_tick=axes[0].flags_tick=axes[1].position_tick=axes[1].flags_tick=tick;
   axes[axis].position_tick=tick-800U;CHECK(Pose_Axes(snapshot,tick));
   axes[axis].position_tick=tick-801U;CHECK(!Pose_Axes(snapshot,tick));
  }
 }
 return failures;
}
'''
with tempfile.TemporaryDirectory(prefix="arm-start-tests-") as temp:
    work=Path(temp)
    c=work/"test.c"
    c.write_text(prefix+syscalls+block+rx+(root/"tools/tests/test_arm_ui_pose.inc").read_text(encoding="utf-8")+suffix,encoding="utf-8")
    linker=work/"test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary=work/"test.elf"
    gcc=shutil.which("arm-none-eabi-gcc")
    if not gcc: raise SystemExit("arm-none-eabi-gcc required")
    includes=[root/"Core/Inc",root/"Drivers/STM32H7xx_HAL_Driver/Inc",root/"Drivers/CMSIS/Device/ST/STM32H7xx/Include",root/"Drivers/CMSIS/Include"]
    subprocess.run([gcc,*(["-DCOMPETITION_PRELIMINARY=1"] if "--prelim" in sys.argv else []),*( ["-DTEST_PROFILE_ISOLATION_ONLY"] if "--profile-isolation-only" in sys.argv else [] ),*( ["-DTEST_DISC_PROFILE_ONLY"] if "--disc-profile-only" in sys.argv else [] ),*( ["-DTEST_RING_RETURN_PARALLEL_ONLY"] if ("--ring-return-parallel-only" in sys.argv or "--ring-return30-only" in sys.argv) else [] ),*( ["-DTEST_TRAY_START_RETRY_ONLY"] if "--tray-start-retry-only" in sys.argv else [] ),*( ["-DTEST_RING_PICK_OVERLAP_ONLY"] if ("--ring-pick-overlap-only" in sys.argv or "--pickup-overlap-only" in sys.argv) else [] ),*( ["-DTEST_UI_ONLY"] if "--ui-only" in sys.argv else [] ),*( ["-DTEST_FAST_TRANSFER_ONLY"] if "--fast-transfer-only" in sys.argv else [] ),*( ["-DTEST_PHOTO_TARGETS_ONLY"] if "--photo-targets-only" in sys.argv else [] ),*( ["-DTEST_TRAY_GATE_ONLY"] if "--tray-gate-only" in sys.argv else [] ),"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror",
        "-DSTM32H723xx","-DUSE_HAL_DRIVER",*["-I"+str(x) for x in includes],"-ffunction-sections","-fdata-sections",
        str(c), str(root/"Core/Src/screen_metrics.c"),"-nostartfiles","--specs=nosys.specs","-Wl,--gc-sections","-Wl,-T,"+str(linker),"-lm","-o",str(binary)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open("rb") as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD" and segment["p_filesz"]:emu.mem_write(segment["p_vaddr"],segment.data())
        symbols={s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()};entry=elf["e_entry"]
    sentinel=0x080FF000;completed=[]
    def returned(machine,address,size,user_data):completed.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=20_000_000,count=10_000_000)
    def result(name):return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"Arm start pose: {result('checks')} checks, {result('failures')} failures; first line={result('first_failure')}")
    if not completed or not result("checks") or result("failures"):
        print('Failed C lines:',[int.from_bytes(emu.mem_read(symbols['failure_lines']+i*4,4),'little') for i in range(min(result('failures'),32))])
        (root/"tmp/arm_pose_failed.c").write_text(c.read_text(encoding="utf-8"),encoding="utf-8")
        raise SystemExit("Arm start pose test failed")
