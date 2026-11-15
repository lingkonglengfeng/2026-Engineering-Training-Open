#include "vision.h"
#include "vision_align.h"
#include "arm_task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
enum{MATCH_MOVING=1,MATCH_RETURN=7,MATCH_IO_NONE,MATCH_IO_ZONE2,MATCH_IO_PARK};
static struct{uint8_t state,checkpoint,zone;uint32_t entered_tick;}zdt_match;
static uint32_t zdt_match_request,zdt_arm_camera_token,zdt_park_observation,zdt_park_stable_tick,zdt_park_control_tick,zdt_park_log_tick;
static uint8_t zdt_match_io,zdt_park_phase,zdt_park_stable;
static uint32_t tick,poses,sets,velocities,completions;static uint8_t busy,pose_ready,last_success;
static float last_f,last_r,last_o;
static vision_camera_state_t cam;
static vision_pixel_t px;static vision_ring_count_t count;static vision_ring_axis_t axis;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef ChassisMotor_Stop(void){return HAL_OK;}
void ChassisMotor_GetWheelRpm(int16_t rpm[4]){memset(rpm,0,8);}
HAL_StatusTypeDef ChassisMotor_SetVelocity(float f,float r,float o){velocities++;last_f=f;last_r=r;last_o=o;return HAL_OK;}
HAL_StatusTypeDef ArmTask_RequestParkingDeploy(uint32_t token){CHECK(token==123);poses++;return busy?HAL_BUSY:HAL_OK;}
uint8_t ArmTask_GetCameraDeployResult(uint32_t token,uint8_t *success){CHECK(token==123);*success=1;return pose_ready;}
HAL_StatusTypeDef Vision_RequestMode(uint8_t camera,uint8_t mode,uint8_t station,uint8_t target,uint8_t hz,uint8_t flags){CHECK(camera==1 && mode==9 && station==5 && target==2 && hz==20 && flags==1);sets++;cam.mode=mode;cam.station=station;cam.pending=1;cam.confirmed=0;return HAL_OK;}
uint8_t Vision_GetCameraState(uint8_t camera,vision_camera_state_t *out){CHECK(camera==1);*out=cam;return 1;}
uint8_t Vision_GetRingSceneWindow(uint8_t camera,uint16_t window,vision_pixel_t *p,vision_ring_count_t *c,uint32_t *age){CHECK(camera==1 && window==400);*p=px;*c=count;*age=0;return 1;}
uint8_t Vision_GetRingAxisWindow(uint8_t camera,uint16_t window,vision_ring_axis_t *a,uint32_t *age){CHECK(camera==1 && window==400);*a=axis;*age=0;return 1;}
HAL_StatusTypeDef Vision_RequestStop(uint8_t camera){CHECK(camera==1);cam.mode=0;cam.pending=1;cam.confirmed=0;cam.reset_pending=1;return HAL_OK;}
void ZDT_Console_Print(const char *line){(void)line;}
static void ZDT_Match_Complete(uint8_t ok,const char *reason){(void)reason;completions++;last_success=ok;zdt_match_io=MATCH_IO_NONE;}
#include "parking_under_test.inc"
static void reset(void){memset(&zdt_match,0,sizeof(zdt_match));memset(&cam,0,sizeof(cam));memset(&px,0,sizeof(px));memset(&count,0,sizeof(count));memset(&axis,0,sizeof(axis));zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_RETURN;zdt_match.zone=1;zdt_match_request=123;zdt_match_io=MATCH_IO_ZONE2;tick=1000;poses=sets=velocities=completions=0;busy=pose_ready=0;zdt_park_phase=zdt_park_stable=0;zdt_park_observation=0;zdt_park_control_tick=0;}
int main(void){uint32_t before;reset();zdt_match.zone=2;ReturnHook(1);CHECK(zdt_match_io==MATCH_IO_ZONE2);
 zdt_match.zone=1;ReturnHook(1);CHECK(zdt_match_io==MATCH_IO_PARK && zdt_park_phase==1);
 busy=1;ZDT_ParkingService();CHECK(zdt_park_phase==1 && !sets && !velocities);
 busy=0;ZDT_ParkingService();CHECK(zdt_park_phase==2 && zdt_arm_camera_token==123);
 ZDT_ParkingService();CHECK(!sets && !velocities);
 pose_ready=1;ZDT_ParkingService();CHECK(zdt_park_phase==4 && sets==1 && !velocities);
 tick+=20;ZDT_ParkingService();CHECK(!velocities);
 cam.pending=0;cam.confirmed=1;px.valid=count.valid=axis.valid=1;count.station=5;count.count=3;
 px.observation_id=100;count.observation_id=axis.observation_id=7;px.dx_px=8;px.dy_px=-4;axis.axis_deg=2;
 tick+=20;ZDT_ParkingService();CHECK(velocities==1 && last_f>0 && last_r<0 && last_o>0 && !completions);
 before=velocities;axis.observation_id=8;tick+=20;ZDT_ParkingService();CHECK(velocities==before && !zdt_park_stable && !completions);
 axis.observation_id=7;px.valid=0;tick+=20;ZDT_ParkingService();CHECK(velocities==before && !completions);
 px.valid=1;px.dx_px=px.dy_px=axis.axis_deg=0;px.observation_id++;
 for(unsigned i=0;i<20;i++){tick+=20;ZDT_ParkingService();}CHECK(zdt_park_stable==1 && zdt_park_phase==4); /* Duplicate frame is never stable5. */
 for(unsigned i=0;i<20 && zdt_park_phase!=5;i++){px.observation_id++;tick+=20;ZDT_ParkingService();}
 CHECK(zdt_park_phase==5 && !completions);
 tick+=20;ZDT_ParkingService();CHECK(!completions);cam.confirmed=1;cam.reset_pending=cam.pending=0;
 ZDT_ParkingService();CHECK(completions==1 && last_success && zdt_match_io==MATCH_IO_NONE);
 before=velocities;tick+=20;ZDT_ParkingService();CHECK(velocities==before);
 return test_failures;}
