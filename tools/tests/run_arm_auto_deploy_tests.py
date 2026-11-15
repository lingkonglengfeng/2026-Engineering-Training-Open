"""Read-only diagnostic FSM and reply demultiplexing, using production C."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
source=(root/"Core/Src/arm_task.c").read_text(encoding="utf-8")
block=source[source.index("HAL_StatusTypeDef ArmTask_RequestCameraDeploy"):source.index("HAL_StatusTypeDef ArmTask_PostCommand")]
block=block[:block.index("static osThreadId_t")]
def owner_function(signature):
 start=source.index(signature);end=source.index("{",start)+1;depth=1
 while depth:
  depth+=(source[end]=="{")-(source[end]=="}");end+=1
 return source[start:end]
block+=owner_function("HAL_StatusTypeDef ArmTask_RequestRound1(")+owner_function("uint8_t ArmTask_GetRound1Result(")+owner_function("void ArmTask_RequestRound1Reset(")+owner_function("HAL_StatusTypeDef ArmTask_RequestStartPose(")+owner_function("uint8_t ArmTask_GetStartPoseResult(")+owner_function("uint8_t ArmTask_IsStartPoseReady(")
start=source.index("        if(s_start_request &&",source.index("ArmStartPose_Poll();"))
owner=source[start:source.index("        ArmServo_Update",start)]
mat_start=source.index("        if(s_material_request) {",source.index("ArmStartPose_Poll();"))
material_owner=source[mat_start:start]
stop_start=source.index("        if(s_stop_request) {",source.index("for (;;)",source.index("void ArmTask_Run")))
stop_owner=source[stop_start:source.index("        ArmCalibration_Poll();",stop_start)]
zdt=(root/"Core/Src/zdt_motor.c").read_text(encoding="utf-8")
start=zdt.index("static void ZDT_Match_Complete(uint8_t success, const char *reason)\n{")
start=zdt.index("{",start)+1
complete=zdt[start:zdt.index("  if(zdt_match_io==MATCH_IO_ALIGN",start)]
start=zdt.index("  if(zdt_arm_camera_token) {\n    uint8_t arm_success")
poll=zdt[start:zdt.index("  MatchControl_Service",start)]
def extract(signature):
 start=zdt.index(signature);end=zdt.index("{",start)+1;depth=1
 while depth:
  depth+=(zdt[end]=="{")-(zdt[end]=="}");end+=1
 return zdt[start:end]
block+=extract("static uint8_t ZDT_Camera_ResetBusy(void)\n{")+extract("static void ZDT_Camera_Reset(void)\n{")+extract("static uint8_t ZDT_Qr_Start(uint8_t for_match)\n{")+extract("static uint8_t ZDT_Qr_Command(const char *command)\n{")+extract("static uint8_t ZDT_DepartureParallelStart(void)\n{")
start=zdt.index("  if(motion==MATCH_MOTION_PREPARE) {",zdt.index("static uint8_t ZDT_Match_StartMotion(match_motion_t motion, uint8_t zone,\n"))
end=zdt.index("{",start)+1;depth=1
while depth:
 depth+=(zdt[end]=="{")-(zdt[end]=="}");end+=1
chassis_guard=extract("static void ZDT_Match_ChassisService(void)\n{")
match_source=(root/"Core/Src/match_control.c").read_text(encoding="utf-8")
def match_function(signature):
 start=match_source.index(signature);end=match_source.index("{",start)+1;depth=1
 while depth:
  depth+=(match_source[end]=="{")-(match_source[end]=="}");end+=1
 return match_source[start:end]

prepare="static uint8_t PrepareDispatch(uint32_t request){char line[112];match_motion_t motion=MATCH_MOTION_PREPARE;"+zdt[start:end]+"return MATCH_START_REJECTED;}\n"
start=zdt.index('  if(ZDT_Match_CommandWord(command,"prepare")) {')
end=zdt.index('{',start)+1;depth=1
while depth:
 depth+=(zdt[end]=='{')-(zdt[end]=='}');end+=1
prepare_command='static uint8_t PrepareCommand(const char *command){'+zdt[start:end]+'return 0U;}\n'
match_source=(root/'Core/Src/match_control.c').read_text(encoding='utf-8')
cal_source=(root/'Core/Src/arm_calibration.c').read_text(encoding='utf-8')
a=cal_source.index('HAL_StatusTypeDef ArmCalibration_Cancel(void)');b=cal_source.index('{',a)+1;depth=1
while depth:
 depth+=(cal_source[b]=='{')-(cal_source[b]=='}');b+=1
cancel_source=cal_source[a:b]
rx="static void StopOwnerService(void){"+stop_owner+"}\nstatic void MaterialOwnerService(void){"+material_owner+"}\nstatic void OwnerService(void){"+owner+"}\nstatic void ZDT_Match_Complete(uint8_t success,const char *reason){(void)reason;"+complete+"if(success)finished++;else{failed++;zdt_arm_camera_token=0;}}\nstatic void AutoPoll(void){"+poll+"}\n"
syscalls=(root/"tools/tests/test_arm_driver.c").read_text(encoding="utf-8")
syscalls=syscalls[syscalls.index("int _close("):syscalls.index("volatile unsigned int test_checks")]
prefix = r'''
/* Keep the optional no-CAM2 build variant here; production-enabled CAM2
 * transport/task ownership is exercised in test_qr_application.c. */
#define QR_CAMERA_RUNTIME_ENABLED 0U
#include "arm_task.h"
#include "arm_material.h"
#include "arm_round1.h"
#include "arm_calibration.h"
#include "match_control.h"
#include "qr_scan.h"
#include "screen_metrics.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static volatile uint32_t s_camera_request,s_camera_done,s_camera_active;
static volatile uint8_t s_camera_result,s_start_result,s_start_ready;
static volatile uint32_t s_start_request,s_start_active,s_start_done,s_start_attempt_tick;
static volatile uint32_t s_material_request,s_material_active,s_material_done;
static volatile uint8_t s_material_result;
static volatile uint32_t s_round_request,s_round_active,s_round_done;
static volatile uint8_t s_round_station,s_round_result,s_round_reset;
static volatile uint8_t s_stop_request;
static uint32_t round_reset_calls,discard_calls;
void ArmRound1_Reset(void){round_reset_calls++;}
void ArmCalibration_DiscardCommands(void){discard_calls++;}
static uint8_t round_busy,round_success;
HAL_StatusTypeDef ArmRound1_Start(uint8_t station){CHECK(station>=1 && station<=6);round_busy=1;round_success=0;return HAL_OK;}
void ArmRound1_Poll(void){}
void ArmRound1_AbortState(void){round_busy=0;round_success=0;}
uint8_t ArmRound1_ReadResult(uint8_t *success){if(round_busy)return 0;*success=round_success;return 1;}

static uint8_t material_busy,material_success;
HAL_StatusTypeDef ArmMaterial_Start(void){material_busy=1;material_success=0;return HAL_OK;}
void ArmMaterial_Poll(void){}
uint8_t ArmMaterial_ReadResult(uint8_t *success){if(material_busy)return 0;*success=material_success;return 1;}
void ArmMaterial_AbortState(void){material_busy=0;material_success=0;}
static uint32_t tick=1000,finished,failed,stops,zdt_arm_camera_token,zdt_arm_camera_started,zdt_match_request;
static uint8_t pose_busy,enabled=1,start_ready_fixture;
static uint32_t arm_stop_calls;
static arm_cal_state_t s_state=ARM_CAL_IDLE;
static arm_camera_state_t s_camera_state=ARM_CAMERA_IDLE;
static uint8_t s_camera_native_pending,s_drop_record,s_zero_record;
static uint8_t s_lift_native_sent,s_lift_native_samples,s_feedback_holding;
static arm_feedback_wait_t s_feedback_pause;
uint8_t ArmCalibration_FeedbackWaiting(void){return s_feedback_holding;}
static int16_t s_percent_rpm;
static uint8_t Cal_Context(void){return 1;}
static uint8_t Cal_CameraActive(void){return s_camera_state>=ARM_CAMERA_PRECHECK && s_camera_state<=ARM_CAMERA_LIFT_FINAL;}
static uint8_t Cal_IsActive(void){return s_state==ARM_CAL_PERCENT_MOVE;}
static arm_driver_axis_t Cal_Axis(void){return ARM_DRIVER_LIFT;}
void ArmStartPose_Cancel(void){pose_busy=0;}
void DM_Motor_DiagCancel(dm_motor_t *m){(void)m;}
dm_motor_t *ArmTask_GetMotor(void){return NULL;}
HAL_StatusTypeDef ArmDriver_StopAll(void){arm_stop_calls++;return HAL_OK;}
void ArmDriver_CancelReachRetry(void){}
HAL_StatusTypeDef ArmDriver_Stop(arm_driver_axis_t a){(void)a;arm_stop_calls++;return HAL_OK;}

static HAL_StatusTypeDef start_begin_result=HAL_OK;
static arm_camera_state_t camera_state=ARM_CAMERA_DONE;
static HAL_StatusTypeDef deploy_result=HAL_OK;
static match_control_t zdt_match;
enum{MATCH_IO_NONE,MATCH_IO_ARM_DEPLOY,MATCH_IO_CAMERA_RESET,MATCH_IO_ARM_START,MATCH_IO_CAMERA,MATCH_IO_ALIGN,MATCH_IO_ROUND1,MATCH_IO_QRSCAN};
static qr_scan_t zdt_qr_scan;
static uint8_t zdt_match_io,zdt_align_enabled=1,zdt_qr_enabled,zdt_qr_used,zdt_camera_reset_mask,zdt_qr_active,zdt_qr_for_match,zdt_qr_locked;
static uint32_t zdt_camera_reset_wait_tick,cam2_stops;
static uint32_t zdt_arm_start_token;
static uint32_t zdt_arm_material_token,zdt_arm_round_token;
static uint32_t zdt_prepare_request,zdt_prepare_tick,zdt_prepare_next,disable_calls;
static uint8_t zdt_prepare_stopped,zdt_prepare_retry_printed,zdt_script_running,nav_busy;
static uint8_t imu_ready=1,zdt_start_zone,zdt_poscal_phase,zdt_mileage_query_active;
#define ZDT_POSCAL_IDLE 0U
static const char *ZDT_SkipSpaces(const char *p){while(*p==' ')p++;return p;}
static uint8_t ZDT_Match_CommandWord(const char *p,const char *w){size_t n=strlen(w);return !strncmp(p,w,n) && (p[n]==0 || p[n]==' ');}
static void ZDT_Script_SetStartZone(uint8_t zone){zdt_start_zone=zone;imu_ready=0;}
static void ZDT_Qr_Cancel(void){}
static void ZDT_Screen_Service(void){} /* Screen/navigation owner exercised separately. */
static HAL_StatusTypeDef disable_hal=HAL_OK;
FDCAN_HandleTypeDef hfdcan2;
static uint8_t ZDT_Nav_IsActive(void){return nav_busy;}
HAL_StatusTypeDef ChassisMotor_DisableForArmPreparation(void){disable_calls++;return disable_hal;}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *can){(void)can;return 32;}

static struct {uint8_t camera;} zdt_align_config[3]={{1},{1},{1}};
static vision_camera_state_t peer[2];
static vision_mode_gate_t zdt_camera_gate;
uint8_t Vision_GetCameraState(uint8_t camera,vision_camera_state_t *state){*state=peer[camera-1];return 1;}
HAL_StatusTypeDef Vision_RequestStop(uint8_t camera){if(camera==2)cam2_stops++;peer[camera-1].reset_pending=1;return HAL_OK;}

uint32_t HAL_GetTick(void){return tick;}
uint8_t HWT101_IsZeroDriftReady(void){return imu_ready;}
void ZDT_Console_Print(const char *s){CHECK(s!=NULL);}
uint8_t ArmStartPose_IsActive(void){return pose_busy;}
HAL_StatusTypeDef ArmStartPose_BeginRestore(void){if(start_begin_result==HAL_OK)pose_busy=1;return start_begin_result;}
uint8_t ArmStartPose_IsReady(void){return !pose_busy && start_ready_fixture;}
uint8_t ArmStartPose_ReadResult(uint8_t *success){if(pose_busy)return 0;*success=start_ready_fixture;return 1;}
HAL_StatusTypeDef ArmCalibration_DeployCamera(void){if(deploy_result==HAL_OK)camera_state=ARM_CAMERA_PRECHECK;return deploy_result;}
arm_camera_state_t ArmCalibration_GetCameraState(void){return camera_state;}
uint8_t ArmTask_IsEnabled(void){return enabled;}
uint8_t ArmTask_IsFaulted(void){return 0;}
HAL_StatusTypeDef ChassisMotor_Stop(void){stops++;return HAL_OK;}
static uint32_t chassis_updates;
static HAL_StatusTypeDef chassis_update_result=HAL_ERROR;
HAL_StatusTypeDef ChassisMotor_Update(void){chassis_updates++;return chassis_update_result;}
uint8_t ChassisMotor_IsEnabled(void){return 0U;}
uint8_t ChassisMotor_HasMotionDemand(void){return 0U;}
'''
suffix = r'''
int main(void){uint8_t ok=0;
 pose_busy=1;CHECK(ArmCalibration_Cancel()==HAL_OK && !pose_busy && arm_stop_calls==1);
 CHECK(ArmCalibration_Cancel()==HAL_OK && arm_stop_calls==1); /* Idle cancellation has no motor side effects. */
 CHECK(PrepareCommand("prepare 1")==1 && zdt_start_zone==1 && zdt_match.state==MATCH_PREPARING);
 CHECK(!s_start_request && PrepareDispatch(501)==MATCH_START_BUSY && !s_start_request);
 imu_ready=1;CHECK(PrepareDispatch(501)==MATCH_START_ACCEPTED && s_start_request==501);
 s_start_request=0;zdt_match.state=MATCH_IDLE;
 CHECK(PrepareCommand("prepare 2")==1 && zdt_start_zone==2 && zdt_match.state==MATCH_PREPARING);
 zdt_match.state=MATCH_IDLE;CHECK(PrepareCommand("prepare 3")==1 && zdt_match.state==MATCH_IDLE);
 nav_busy=1;CHECK(PrepareCommand("prepare 1")==1 && zdt_match.state==MATCH_IDLE);nav_busy=0;
 imu_ready=1;zdt_prepare_request=0;

 CHECK(ArmTask_RequestCameraDeploy(0)==HAL_ERROR);
 zdt_match.state=MATCH_LEAVING;zdt_match.checkpoint=MATCH_QR;zdt_match_request=7;
 CHECK(ZDT_DepartureParallelStart());CHECK(s_camera_request==7 && zdt_arm_camera_token==7 && finished==0);
 ZDT_Match_Complete(1,NULL);CHECK(finished==1 && s_camera_request==7 && zdt_arm_camera_token==7);
 CHECK(ArmTask_RequestCameraDeploy(8)==HAL_BUSY);
 CHECK(!ArmTask_GetCameraDeployResult(7,&ok));
 zdt_match.state=MATCH_MOVING;ZDT_Match_Complete(1,NULL);CHECK(finished==1 && stops==1 && zdt_match_io==MATCH_IO_ARM_DEPLOY);
 OwnerService();CHECK(camera_state==ARM_CAMERA_PRECHECK && s_camera_active==7 && !ArmTask_GetCameraDeployResult(7,&ok));
 /* Old READY was never accepted for the new token. */
 AutoPoll();CHECK(finished==1);
 camera_state=ARM_CAMERA_DONE;OwnerService();CHECK(ArmTask_GetCameraDeployResult(7,&ok) && ok);
 AutoPoll();CHECK(finished==2 && zdt_arm_camera_token==0);
 CHECK(!ArmTask_GetCameraDeployResult(8,&ok));
 zdt_match.state=MATCH_LEAVING;zdt_match_io=MATCH_IO_NONE;zdt_match_request=8;
 CHECK(ZDT_DepartureParallelStart());ZDT_Match_Complete(1,NULL);OwnerService();zdt_match.state=MATCH_MOVING;
 camera_state=ARM_CAMERA_FAILED;OwnerService();AutoPoll();CHECK(failed==1 && zdt_arm_camera_token==0);
 /* Restore job busy rejects automatic unfold and reports that token's failure. */
 zdt_match.state=MATCH_LEAVING;zdt_match_request=9;pose_busy=1;
 CHECK(ZDT_DepartureParallelStart());ZDT_Match_Complete(1,NULL);OwnerService();CHECK(ArmTask_GetCameraDeployResult(9,&ok) && !ok);
 zdt_match.state=MATCH_MOVING;AutoPoll();CHECK(failed==2);pose_busy=0;
 /* No acknowledgement times out; the navigation flow cannot hang forever. */
 zdt_match.state=MATCH_LEAVING;zdt_match_request=10;CHECK(ZDT_DepartureParallelStart());ZDT_Match_Complete(1,NULL);
 tick+=120001;AutoPoll();CHECK(failed==3);
 s_camera_request=s_camera_active=0;enabled=0;zdt_match.state=MATCH_MOVING;
 ZDT_Match_Complete(1,NULL);CHECK(failed==4); /* no recognition after DM hold is lost */
 /* Pending CAM1 reset does not block departure/QR transport, only recognition. */
 enabled=1;zdt_match.state=MATCH_LEAVING;zdt_match_request=11;zdt_match_io=MATCH_IO_NONE;
 peer[0].reset_pending=1;CHECK(ZDT_DepartureParallelStart());ZDT_Match_Complete(1,NULL);CHECK(s_camera_request==11);
 OwnerService();zdt_match.state=MATCH_MOVING;ZDT_Match_Complete(1,NULL);
 camera_state=ARM_CAMERA_DONE;OwnerService();AutoPoll();CHECK(zdt_match_io==MATCH_IO_CAMERA_RESET && zdt_arm_camera_token==0);
 {uint32_t before=finished;AutoPoll();CHECK(finished==before);peer[0].reset_pending=0;AutoPoll();CHECK(finished==before+1);}
 /* Disabled CAM2 never blocks on its stale reset and receives no STOP. */
 peer[1].reset_pending=1;CHECK(!ZDT_Camera_ResetBusy());ZDT_Camera_Reset();CHECK(cam2_stops==0);
 peer[0].reset_pending=0;zdt_align_enabled=0;zdt_qr_enabled=1;CHECK(!ZDT_Camera_ResetBusy());
 /* Still fail safely at QR when active CAM1 reset never gets an ACK. */
 zdt_align_enabled=1;zdt_qr_enabled=0;peer[0].reset_pending=1;zdt_match_io=MATCH_IO_NONE;
 ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_CAMERA_RESET);
 tick+=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS+1001;AutoPoll();CHECK(failed==5);
 CHECK(QR_CAMERA_RUNTIME_ENABLED==0U && !ZDT_Qr_Start(0));
 zdt_qr_enabled=1;CHECK(ZDT_Qr_Command("qrscan on")==1 && !zdt_qr_enabled && cam2_stops==0);
 CHECK(ZDT_Qr_Command("qrscan start")==1 && !zdt_qr_active && cam2_stops==0);
 s_camera_request=s_camera_active=0;pose_busy=0;
 CHECK(ArmTask_RequestStartPose(32)==HAL_OK && !ArmTask_IsStartPoseReady());
 CHECK(ArmTask_RequestCameraDeploy(99)==HAL_BUSY);
 OwnerService();CHECK(!ArmTask_GetStartPoseResult(32,&ok));
 pose_busy=0;start_ready_fixture=1;OwnerService();
 CHECK(ArmTask_GetStartPoseResult(32,&ok) && ok && ArmTask_IsStartPoseReady());
 CHECK(!ArmTask_GetStartPoseResult(31,&ok));
 CHECK(ArmTask_RequestStartPose(33)==HAL_OK);OwnerService();
 CHECK(!ArmTask_IsStartPoseReady() && !ArmTask_GetStartPoseResult(33,&ok));
 pose_busy=0;start_ready_fixture=0;OwnerService();
 CHECK(ArmTask_GetStartPoseResult(33,&ok) && !ok);
 start_begin_result=HAL_BUSY;pose_busy=0;start_ready_fixture=1;
 CHECK(ArmTask_RequestStartPose(34)==HAL_OK);OwnerService();
 CHECK(s_start_request==34 && !ArmTask_GetStartPoseResult(34,&ok));
 CHECK(ArmTask_RequestCameraDeploy(35)==HAL_BUSY);
 start_begin_result=HAL_OK;tick+=100;OwnerService();
 CHECK(s_start_active==34 && !ArmTask_GetStartPoseResult(34,&ok));
 pose_busy=0;start_ready_fixture=1;OwnerService();
 CHECK(ArmTask_GetStartPoseResult(34,&ok) && ok);
 s_start_request=s_start_active=s_camera_request=s_camera_active=0;pose_busy=0;
 disable_hal=HAL_BUSY;CHECK(PrepareDispatch(42)==MATCH_START_BUSY && !s_start_request);
 tick+=100;disable_hal=HAL_OK;CHECK(PrepareDispatch(42)==MATCH_START_ACCEPTED && s_start_request==42);
 {uint32_t count=disable_calls;CHECK(PrepareDispatch(42)==MATCH_START_BUSY && disable_calls==count);}
 s_start_request=0;disable_hal=HAL_ERROR;CHECK(PrepareDispatch(43)==MATCH_START_BUSY);
 tick+=2001;CHECK(PrepareDispatch(43)==MATCH_START_REJECTED);
 disable_hal=HAL_OK;nav_busy=1;CHECK(PrepareDispatch(44)==MATCH_START_REJECTED);nav_busy=0;
 {uint32_t old_faults=failed;
  zdt_match.state=MATCH_PREPARING;ZDT_Match_ChassisService();
  CHECK(failed==old_faults && chassis_updates==0 && zdt_match.state==MATCH_PREPARING);
  s_start_request=s_start_active=0;
  CHECK(PrepareDispatch(45)==MATCH_START_ACCEPTED && s_start_request==45);
  zdt_match.state=MATCH_WAIT_START_OK;ZDT_Match_ChassisService();
  CHECK(failed==old_faults && chassis_updates==0 && zdt_match.state==MATCH_WAIT_START_OK);
  zdt_match.state=MATCH_MOVING;ZDT_Match_ChassisService();
  CHECK(chassis_updates==1 && failed==old_faults+1 && zdt_match.state==MATCH_FAULT);
 }
 s_camera_request=s_camera_active=s_start_request=s_start_active=0;
 CHECK(ArmTask_RequestMaterial(0)==HAL_ERROR);
 CHECK(ArmTask_RequestMaterial(71)==HAL_OK);
 CHECK(ArmTask_RequestStartPose(72)==HAL_BUSY && ArmTask_RequestCameraDeploy(72)==HAL_BUSY);
 MaterialOwnerService();CHECK(s_material_active==71 && !ArmTask_GetMaterialResult(71,&ok));
 material_busy=0;material_success=1;MaterialOwnerService();
 CHECK(ArmTask_GetMaterialResult(71,&ok) && ok && !s_material_active);
 CHECK(!ArmTask_GetMaterialResult(70,&ok));
 CHECK(ArmTask_RequestMaterial(72)==HAL_OK);MaterialOwnerService();
 CHECK(ArmCalibration_Cancel()==HAL_OK);MaterialOwnerService();
 CHECK(ArmTask_GetMaterialResult(72,&ok) && !ok);
 CHECK(ArmTask_RequestRound1(0,1)==HAL_ERROR && ArmTask_RequestRound1(90,0)==HAL_ERROR && ArmTask_RequestRound1(90,7)==HAL_ERROR);
 s_material_request=s_material_active=s_start_request=s_start_active=s_camera_request=s_camera_active=0;
 s_round_reset=0; /* Owner consumes the reset queued by PREPARE above. */
 CHECK(ArmTask_RequestRound1(90,2)==HAL_OK && s_round_station==2);
 CHECK(ArmTask_RequestMaterial(91)==HAL_BUSY && ArmTask_RequestStartPose(91)==HAL_BUSY && ArmTask_RequestCameraDeploy(91)==HAL_BUSY);
 MaterialOwnerService();CHECK(s_round_active==90 && round_busy && !ArmTask_GetRound1Result(90,&ok));
 camera_state=ARM_CAMERA_DM_HOME;round_busy=0;round_success=1;MaterialOwnerService();CHECK(!s_round_active && ArmTask_GetRound1Result(90,&ok) && ok);
 CHECK(s_camera_active==90 && !s_camera_request && !ArmTask_GetCameraDeployResult(90,&ok));
 CHECK(ArmTask_RequestRound1(91,3)==HAL_BUSY); /* no second arm owner during restoration */
 camera_state=ARM_CAMERA_DONE;OwnerService();CHECK(ArmTask_GetCameraDeployResult(90,&ok) && ok && !s_camera_active);
 CHECK(!ArmTask_GetRound1Result(89,&ok));
 ArmTask_RequestRound1Reset();CHECK(ArmTask_RequestRound1(92,1)==HAL_BUSY);s_round_reset=0;
 CHECK(ArmTask_RequestRound1(92,1)==HAL_OK);MaterialOwnerService();
 CHECK(ArmCalibration_Cancel()==HAL_OK);MaterialOwnerService();CHECK(ArmTask_GetRound1Result(92,&ok) && !ok);
 CHECK(ArmTask_RequestRound1(93,1)==HAL_OK);s_stop_request=1;StopOwnerService();
 CHECK(ArmTask_GetRound1Result(93,&ok) && !ok && !s_round_request && !s_round_active && discard_calls==1);
 CHECK(ArmTask_RequestRound1(94,1)==HAL_OK);MaterialOwnerService();CHECK(round_busy);
 s_stop_request=1;ArmTask_RequestRound1Reset();StopOwnerService();MaterialOwnerService();
 CHECK(ArmTask_GetRound1Result(94,&ok) && !ok && !round_busy && round_reset_calls==1 && !s_round_reset);
 CHECK(ArmTask_RequestRound1(95,3)==HAL_OK);MaterialOwnerService();CHECK(!ArmTask_GetRound1Result(94,&ok));
 camera_state=ARM_CAMERA_DONE;round_busy=0;round_success=1;MaterialOwnerService();CHECK(ArmTask_GetRound1Result(95,&ok) && ok);
 {unsigned job;for(job=4;job<=6;job++){
   CHECK(ArmTask_RequestRound1(100U+job,(uint8_t)job)==HAL_OK && s_round_station==job);
   MaterialOwnerService();CHECK(round_busy && !ArmTask_GetRound1Result(95,&ok));
   camera_state=ARM_CAMERA_DONE;round_busy=0;round_success=1;MaterialOwnerService();CHECK(ArmTask_GetRound1Result(100U+job,&ok) && ok);
   CHECK(ArmTask_GetCameraDeployResult(100U+job,&ok) && ok);
 }}
 s_camera_active=199;camera_state=ARM_CAMERA_FAILED;OwnerService();zdt_arm_camera_token=199;
 zdt_match.state=MATCH_MOVING;zdt_match.pending=1;zdt_match_request=0;AutoPoll();
 CHECK(zdt_match.state==MATCH_FAULT && !zdt_match.pending); /* failed CAMERA before new NAV dispatch */
 return failures;}
'''
with tempfile.TemporaryDirectory(prefix="arm-auto-tests-") as temp:
    work=Path(temp)
    c=work/"test.c"
    c.write_text(prefix+syscalls+block+rx+prepare+chassis_guard+"\n"+match_source+prepare_command+cancel_source+suffix,encoding="utf-8")
    linker=work/"test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary=work/"test.elf"
    gcc=shutil.which("arm-none-eabi-gcc")
    if not gcc: raise SystemExit("arm-none-eabi-gcc required")
    includes=[root/"Core/Inc",root/"Drivers/STM32H7xx_HAL_Driver/Inc",root/"Drivers/CMSIS/Device/ST/STM32H7xx/Include",root/"Drivers/CMSIS/Include"]
    subprocess.run([gcc,"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror",
        "-DSTM32H723xx","-DUSE_HAL_DRIVER",*["-I"+str(x) for x in includes],"-ffunction-sections","-fdata-sections",
        str(c),str(root/"Core/Src/screen_metrics.c"),"-nostartfiles","--specs=nosys.specs","-Wl,--gc-sections","-Wl,-T,"+str(linker),"-lm","-o",str(binary)],check=True)
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
    emu.emu_start(entry|1,sentinel+2,timeout=10_000_000,count=5_000_000)
    def result(name):return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"Arm automatic deployment: {result('checks')} checks, {result('failures')} failures; first line={result('first_failure')}")
    if not completed or not result("checks") or result("failures"):
        (root/"tmp/arm_auto_failed_v80.c").write_text(c.read_text(encoding="utf-8"),encoding="utf-8")
        raise SystemExit("Arm automatic deployment test failed")
