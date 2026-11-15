#include "screen_metrics.h"
#include "qr_scan.h"
#include "vision.h"
#include "vision_align_diag.h"
#include "vision_yaw.h"
#include "match_control.h"
#include "zone2_route.h"
#include "arm_round1.h"
#include <stdio.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
typedef enum {MATCH_IO_NONE,MATCH_IO_LEAVE_START_QUERY,MATCH_IO_LEAVE_SCRIPT,MATCH_IO_LEAVE_END_QUERY,
    MATCH_IO_NAV,MATCH_IO_ZONE2,MATCH_IO_ALIGN,MATCH_IO_CAMERA,MATCH_IO_QRSCAN,MATCH_IO_ARM_DEPLOY,MATCH_IO_CAMERA_RESET,MATCH_IO_MATERIAL,MATCH_IO_ROUND1} zdt_match_io_t;
static match_control_t zdt_match;static zdt_match_io_t zdt_match_io;static uint32_t zdt_match_request;
static uint32_t zdt_arm_camera_token,zdt_arm_camera_started,zdt_arm_material_token;
static uint32_t zdt_arm_round_token,round_calls;
static uint8_t round_ready,round_success,round_station;
static HAL_StatusTypeDef round_hal=HAL_OK;
static uint8_t App_RoundActive(void){return !round_ready;}
static HAL_StatusTypeDef App_RoundRequest(uint32_t token,uint8_t station){CHECK(token!=0);round_calls++;round_station=station;return round_hal;}
static uint8_t App_RoundResult(uint32_t token,uint8_t *success){CHECK(token!=0);if(!round_ready)return 0;*success=round_success;return 1;}
static uint8_t test_material_auto,test_material_ready,test_material_success;
static uint32_t test_material_calls;
static HAL_StatusTypeDef test_material_hal;
static uint8_t App_MaterialAuto(void){return test_material_auto;}
static uint8_t App_MaterialActive(void){return test_material_auto && !test_material_ready;}
static HAL_StatusTypeDef App_MaterialRequest(uint32_t token){CHECK(token!=0);test_material_calls++;return test_material_hal;}
static uint8_t App_MaterialResult(uint32_t token,uint8_t *success){CHECK(token!=0);if(!test_material_ready)return 0;*success=test_material_success;return 1;}
static uint32_t zdt_camera_reset_wait_tick;
static uint8_t ZDT_Camera_ResetBusy(void){return 0U;}
static void ZDT_Camera_Reset(void){}
static uint32_t deploy_calls;static HAL_StatusTypeDef deploy_hal=HAL_OK;
static HAL_StatusTypeDef App_ArmDeploy(uint32_t token){CHECK(token!=0U);deploy_calls++;return deploy_hal;}
static uint8_t App_ArmEnabled(void){return 1U;}
static uint8_t App_ArmFaulted(void){return 0U;}
static qr_scan_t zdt_qr_scan;static vision_qr_task_t zdt_qr_task;
static uint8_t zdt_qr_enabled,zdt_qr_active,zdt_qr_for_match,zdt_qr_used,zdt_qr_locked,zdt_qr_printed;
static vision_align_diag_t zdt_align_diag;static vision_align_t zdt_align;static uint8_t zdt_align_phase,zdt_align_enabled=1U;
static vision_yaw_t zdt_vyaw;
static zone2_route_t zdt_zone2;static const char *zdt_zone2_reason;static uint32_t zdt_can_enqueue_failures,zdt_diag_can_start;
static vision_camera_state_t test_camera;static uint32_t test_tick,test_stops,test_plate,test_protocol_changes;
static uint8_t test_protocol,test_task_seen,test_busy;static vision_qr_task_t test_task;
static uint32_t App_Tick(void){return test_tick;}
static uint8_t ZDT_Align_DiagBusy(void){return test_busy;}
static uint8_t App_Port(uint8_t camera,vision_port_stats_t *p)
{CHECK(camera==2U);memset(p,0,sizeof(*p));p->protocol=test_protocol;return 1U;}
static uint8_t App_Protocol(uint8_t camera,uint8_t protocol)
{CHECK(camera==2U && protocol==1U);test_protocol=protocol;test_protocol_changes++;return 1U;}
static uint8_t App_Camera(uint8_t camera,vision_camera_state_t *c){CHECK(camera==2U);*c=test_camera;return 1U;}
static uint8_t App_Task(vision_qr_task_t *t,uint32_t *age)
{(void)age;if(!test_task_seen)return 0U;*t=test_task;return 1U;}
static HAL_StatusTypeDef App_Mode(uint8_t camera,uint8_t mode,uint8_t station,uint8_t target,uint8_t hz,uint8_t flags)
{CHECK(camera==2U && mode==6U && station==4U && !target && hz==20U && !flags);
 test_camera.mode=mode;test_camera.station=station;test_camera.hz=hz;test_camera.pending=1U;test_camera.confirmed=0U;
 test_camera.request_seq=(uint8_t)((test_camera.request_seq+1U)%16U);return HAL_OK;}
static HAL_StatusTypeDef App_Stop(uint8_t camera)
{CHECK(camera==2U);test_stops++;test_camera.pending=test_camera.reset_pending=1U;test_camera.confirmed=0U;
 test_camera.request_seq=(uint8_t)((test_camera.request_seq+1U)%16U);return HAL_OK;}
static void App_Print(const char *line){CHECK(line!=NULL);}
static uint8_t App_Pixel(uint8_t camera,uint8_t type,vision_pixel_t *p,uint32_t *age)
{(void)camera;(void)type;memset(p,0,sizeof(*p));*age=0U;return 0U;}
static void ZDT_Align_DiagObserve(uint32_t now,const vision_pixel_t *p,uint32_t age,float f,float r,float yaw)
{(void)now;(void)p;(void)age;(void)f;(void)r;(void)yaw;}
static HAL_StatusTypeDef App_ChassisStop(void){return HAL_OK;}
static uint8_t test_real_station,test_align_calls;
static uint8_t ZDT_Align_Station(void){return test_real_station && zdt_match.checkpoint>=MATCH_RAW1 && zdt_match.checkpoint<=MATCH_TEMP2?(uint8_t)((zdt_match.checkpoint-1U)%3U+1U):0U;}
static uint8_t ZDT_Align_Start(uint8_t station){CHECK(test_real_station && station>=1 && station<=3);test_align_calls++;zdt_match_io=MATCH_IO_ALIGN;return 1U;}
static void ZDT_Align_RequestCameraMode(uint8_t station){CHECK(station==1U);test_plate++;}
static void ZDT_Match_Complete(uint8_t success,const char *reason);
#define ArmMaterial_AutoEnabled App_MaterialAuto
#define ArmRound1_IsActive App_RoundActive
#define ArmTask_RequestRound1 App_RoundRequest
#define ArmTask_GetRound1Result App_RoundResult
#define ArmMaterial_IsActive App_MaterialActive
#define ArmTask_RequestMaterial App_MaterialRequest
#define ArmTask_GetMaterialResult App_MaterialResult
#define HAL_GetTick App_Tick
#define ArmTask_RequestCameraDeploy App_ArmDeploy
#define ArmTask_IsEnabled App_ArmEnabled
#define ArmTask_IsFaulted App_ArmFaulted
#define Vision_GetPortStats App_Port
#define Vision_SetCameraProtocol App_Protocol
#define Vision_GetCameraState App_Camera
#define Vision_GetQrTask App_Task
#define Vision_RequestMode App_Mode
#define Vision_RequestStop App_Stop
#define Vision_GetPixel App_Pixel
#define ChassisMotor_Stop App_ChassisStop
#define ChassisMotor_StopImmediate App_ChassisStop
#define ZDT_Console_Print App_Print
#include "qr_application_under_test.inc"
#undef HAL_GetTick
#undef Vision_GetPortStats
#undef Vision_SetCameraProtocol
#undef Vision_GetCameraState
#undef Vision_GetQrTask
#undef Vision_RequestMode
#undef Vision_RequestStop
#undef Vision_GetPixel
#undef ChassisMotor_Stop
#undef ChassisMotor_StopImmediate
#undef ZDT_Console_Print
static uint8_t start_motion(match_motion_t motion,uint8_t zone,uint32_t id)
{(void)zone;zdt_match_request=id;zdt_match_io=MATCH_IO_ZONE2;
 if(motion==MATCH_MOTION_LEAVE){CHECK(ZDT_DepartureParallelStart());}
 return 1U;}
static void cancel_motion(void){ZDT_Qr_Cancel();zdt_match_io=MATCH_IO_NONE;zdt_match_request=0U;zdt_arm_camera_token=0U;}
static void changed(match_state_t state,match_checkpoint_t checkpoint){(void)state;(void)checkpoint;}
static void reset(uint8_t enabled)
{
    const match_ports_t ports={start_motion,cancel_motion,changed};
    memset(&zdt_match,0,sizeof(zdt_match));memset(&zdt_qr_scan,0,sizeof(zdt_qr_scan));memset(&test_camera,0,sizeof(test_camera));
    memset(&zdt_align_diag,0,sizeof(zdt_align_diag));memset(&zdt_zone2,0,sizeof(zdt_zone2));
    zdt_qr_enabled=enabled;zdt_qr_active=zdt_qr_for_match=zdt_qr_used=zdt_qr_locked=zdt_qr_printed=0U;
    test_material_auto=test_material_ready=test_material_success=0U;test_material_calls=zdt_arm_material_token=0U;test_material_hal=HAL_OK;
    round_ready=round_success=round_station=0;zdt_arm_round_token=round_calls=0;round_hal=HAL_OK;
    test_tick=100U;test_stops=test_plate=test_protocol_changes=deploy_calls=0U;deploy_hal=HAL_OK;
    test_task_seen=test_busy=test_real_station=test_align_calls=0U;test_protocol=1U;
    test_camera.confirmed=1U;MatchControl_Init(&zdt_match,&ports);
    CHECK(MatchControl_Start(&zdt_match,1U,test_tick));MatchControl_Service(&zdt_match,test_tick);
    ZDT_Match_Complete(1U,NULL);MatchControl_Service(&zdt_match,++test_tick);
    /* QR-only fixture: the separate arm worker has completed successfully.
     * Pending/failure/concurrent gates are exercised in run_arm_auto_deploy_tests.py. */
    zdt_arm_camera_token=0U;
    CHECK(zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==MATCH_QR);
}
void Test_QrApplication(void)
{
    CHECK(QR_CAMERA_RUNTIME_ENABLED==1U);
    /* Early departure scan cannot complete chassis motion. Cover capture
     * before arrival, pending STOP, late capture and cancel/failure. */
    {unsigned mode;for(mode=0;mode<3;mode++) {
      uint8_t seq;uint32_t deploy_before;
      reset(1U);zdt_match.state=MATCH_LEAVING;zdt_match.qr_required=1U;
      zdt_match.active_request=zdt_match_request=501U;zdt_match_io=MATCH_IO_LEAVE_SCRIPT;
      deploy_before=deploy_calls;CHECK(ZDT_DepartureParallelStart());
      CHECK(deploy_calls==deploy_before+1 && zdt_arm_camera_token==501U && zdt_qr_for_match==2U);
      CHECK(zdt_match_io==MATCH_IO_LEAVE_SCRIPT && zdt_match.state==MATCH_LEAVING);
      ZDT_Qr_Service();test_camera.pending=0;test_camera.confirmed=1;test_camera.result=0;
      test_tick+=10;ZDT_Qr_Service();seq=test_camera.request_seq;
      CHECK(zdt_qr_scan.phase==QR_SCAN_READ && zdt_match_io==MATCH_IO_LEAVE_SCRIPT);
      test_tick+=20000;ZDT_Qr_Service();CHECK(zdt_qr_active && zdt_qr_scan.phase==QR_SCAN_READ);
      if(mode<2) {
        memset(&test_task,0,sizeof(test_task));test_task_seen=1;test_task.observation_id=10;
        test_task.request_seq=seq;
        {const uint8_t cs[2][3]={{1,6,5},{6,1,5}},ps[2][3]={{1,2,3},{2,3,1}};
         memcpy(test_task.colors,cs,sizeof(cs));memcpy(test_task.positions,ps,sizeof(ps));}
        strcpy(test_task.raw,"156+123+516+231");ZDT_Qr_Service();ZDT_Qr_Service();
        CHECK(zdt_qr_locked && zdt_qr_scan.phase==QR_SCAN_STOP && !test_plate);
        if(!mode) {
          test_camera.pending=test_camera.reset_pending=0;test_camera.confirmed=1;
          test_camera.mode=test_camera.station=test_camera.hz=0;ZDT_Qr_Service();
          CHECK(!zdt_qr_active && zdt_match.state==MATCH_LEAVING && zdt_match_io==MATCH_IO_LEAVE_SCRIPT);
        }
      }
      zdt_arm_camera_token=mode?0U:501U; /* mode0: arm still deploying at physical QR arrival */
      ZDT_Match_Complete(1,NULL);MatchControl_Service(&zdt_match,++test_tick);
      CHECK(zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==MATCH_QR && deploy_calls==deploy_before+1);
      if(!mode)zdt_match.automatic=1U;
      ZDT_Match_Complete(1,NULL); /* physical QR pose and facing actually arrived */
      if(!mode)CHECK(!test_plate && !zdt_qr_active && zdt_match.checkpoint==MATCH_RAW1 && zdt_match.pending && zdt_arm_camera_token==501U);
      else {
        CHECK(zdt_qr_active && zdt_qr_for_match==1 && zdt_match_io==MATCH_IO_QRSCAN && !test_plate);
        CHECK(zdt_qr_scan.gate.request_seq==seq); /* no new SET/session */
        if(mode==1) {
          test_camera.pending=test_camera.reset_pending=0;test_camera.confirmed=1;
          test_camera.mode=test_camera.station=test_camera.hz=0;ZDT_Qr_Service();
          CHECK(!zdt_qr_active && test_plate==1 && zdt_match_io==MATCH_IO_CAMERA);
        } else {
          test_tick+=15000;ZDT_Qr_Service();CHECK(zdt_match.state==MATCH_FAULT && !zdt_qr_locked);
        }
      }
    }
    reset(1);deploy_hal=HAL_ERROR;zdt_match.qr_required=1;
    CHECK(!ZDT_DepartureParallelStart() && !zdt_qr_active);deploy_hal=HAL_OK;
    reset(1);zdt_match.qr_required=1;CHECK(ZDT_Qr_Start(2));ZDT_Qr_Service();
    MatchControl_Reset(&zdt_match);ZDT_Qr_Service();CHECK(!zdt_qr_active && !zdt_qr_locked);
    }
    reset(0U);ZDT_Match_Complete(1U,NULL);CHECK(test_plate==1U && !zdt_qr_active);
    ZDT_Match_Complete(1U,NULL);CHECK(zdt_match.state==MATCH_WAIT_OK); /* unconnected CAM2 old flow */
    reset(1U);ZDT_Match_Complete(1U,NULL);CHECK(zdt_qr_active && !test_plate && zdt_match_io==MATCH_IO_QRSCAN);
    ZDT_Qr_Service();CHECK(test_camera.pending && zdt_match.state==MATCH_MOVING);
    test_camera.pending=0U;test_camera.confirmed=1U;test_camera.result=0U;test_tick+=10U;
    ZDT_Qr_Service();CHECK(zdt_qr_scan.phase==QR_SCAN_READ && !test_plate && !zdt_qr_locked);
    memset(&test_task,0,sizeof(test_task));test_task.observation_id=1U;test_task.request_seq=test_camera.request_seq;
    {const uint8_t colors[2][3]={{1,6,5},{6,1,5}},positions[2][3]={{1,2,3},{2,3,1}};
      memcpy(test_task.colors,colors,sizeof(colors));memcpy(test_task.positions,positions,sizeof(positions));}
    strcpy(test_task.raw,"156+123+516+231");test_task_seen=1U;test_tick+=10U;ZDT_Qr_Service();
    CHECK(zdt_qr_locked && test_stops==1U && !test_plate && zdt_match.state==MATCH_MOVING);
    test_task_seen=0U;test_camera.pending=test_camera.reset_pending=0U;test_camera.confirmed=1U;
    test_camera.mode=test_camera.station=test_camera.hz=0U;test_tick+=10U;ZDT_Qr_Service();
    CHECK(!zdt_qr_active && zdt_qr_locked && test_plate==1U && zdt_match_io==MATCH_IO_CAMERA);
    ZDT_Match_Complete(1U,NULL);CHECK(zdt_match.state==MATCH_WAIT_OK && strcmp(zdt_qr_task.raw,test_task.raw)==0);
    {vision_qr_task_t copy;CHECK(ZDT_Match_GetQrTask(&copy) && !memcmp(&copy,&zdt_qr_task,sizeof(copy)));CHECK(!ZDT_Match_GetQrTask(NULL));}
    MatchControl_Reset(&zdt_match);CHECK(!zdt_qr_locked && !zdt_qr_active);
    {vision_qr_task_t copy;CHECK(!ZDT_Match_GetQrTask(&copy));}
    reset(1U);ZDT_Match_Complete(1U,NULL);test_tick+=2600U;ZDT_Qr_Service();
    CHECK(zdt_match.state==MATCH_FAULT && !zdt_qr_locked && !test_plate);
    reset(0U);MatchControl_Reset(&zdt_match);test_protocol=2U;CHECK(ZDT_Qr_Command("qrscan start"));
    CHECK(test_protocol_changes==1U && zdt_qr_active);
    CHECK(ZDT_Qr_Command("qrscan off") && !zdt_qr_active && !zdt_qr_locked);
    /* Automatic QR station cannot advance on mode ACK or on a latched task
     * while STOP is pending; success schedules RAW without another OK. */
    {
      unsigned invalid;
      for(invalid=0;invalid<2;invalid++) {
        reset(1U);zdt_match.automatic=zdt_match.qr_required=1U;
        ZDT_Match_Complete(1U,NULL);ZDT_Qr_Service();
        test_camera.pending=0;test_camera.confirmed=1;test_camera.result=0;
        ZDT_Qr_Service();CHECK(zdt_match.checkpoint==MATCH_QR && zdt_qr_active);
        test_task_seen=1;test_task.observation_id=1;test_task.request_seq=test_camera.request_seq;
        {const uint8_t colors[2][3]={{1,6,5},{6,1,5}},positions[2][3]={{1,2,3},{2,3,1}};
         memcpy(test_task.colors,colors,sizeof(colors));memcpy(test_task.positions,positions,sizeof(positions));}
        if(invalid)test_task.colors[1][0]=2;
        ZDT_Qr_Service();CHECK(zdt_qr_locked && zdt_qr_scan.phase==QR_SCAN_STOP);
        ZDT_Qr_Service();CHECK(zdt_match.checkpoint==MATCH_QR && !test_plate);
        test_camera.pending=test_camera.reset_pending=0;test_camera.confirmed=1;
        test_camera.mode=test_camera.station=test_camera.hz=0;ZDT_Qr_Service();
        if(invalid)CHECK(zdt_match.state==MATCH_FAULT && !zdt_qr_locked && !test_plate);
        else {
          CHECK(test_plate==0);
          CHECK(zdt_match.state==MATCH_MOVING && zdt_match.pending && zdt_match.checkpoint==MATCH_RAW1);
        }
      }
    }
    reset(0U);test_material_auto=1U;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_RAW1;
    zdt_match.active_request=zdt_match_request=64U;zdt_match_io=MATCH_IO_ALIGN;zdt_align.config.camera=1U;zdt_align.config.type=VISION_TYPE_PLATE_PX;
    ZDT_Match_Complete(1U,NULL);
    CHECK(zdt_match_io==MATCH_IO_MATERIAL && zdt_match_request==64U && zdt_match.state==MATCH_MOVING);
    test_material_hal=HAL_BUSY;ZDT_MaterialService();CHECK(!zdt_arm_material_token && zdt_match.state==MATCH_MOVING);
    test_material_hal=HAL_OK;ZDT_MaterialService();CHECK(zdt_arm_material_token==64U);
    ZDT_MaterialService();CHECK(zdt_match.state==MATCH_MOVING);
    test_tick+=180000U; /* Exceeds the normal 120s navigation deadline. */ZDT_MaterialService();MatchControl_Service(&zdt_match,test_tick);
    CHECK(zdt_match.state==MATCH_MOVING);
    test_material_ready=test_material_success=1U;ZDT_MaterialService();
    CHECK(zdt_match.state==MATCH_WAIT_OK && !zdt_match_request && !zdt_arm_material_token && zdt_match_io==MATCH_IO_NONE);
    reset(0U);test_material_auto=1U;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_RAW2;
    zdt_match.active_request=zdt_match_request=65U;zdt_match_io=MATCH_IO_ALIGN;zdt_align.config.camera=1U;zdt_align.config.type=VISION_TYPE_PLATE_PX;
    ZDT_Match_Complete(1U,NULL);ZDT_MaterialService();CHECK(zdt_arm_material_token==65U);
    test_material_ready=1U;test_material_success=0U;ZDT_MaterialService();CHECK(zdt_match.state==MATCH_FAULT);
    /* Normal first round: alignment completion starts whole station ownership;
     * no WAIT_OK/route advance until all inner jobs and CAMERA restore finish. */
    {
      const match_checkpoint_t checkpoints[]={MATCH_RAW1,MATCH_ROUGH1,MATCH_TEMP1};
      unsigned i;
      for(i=0;i<3;i++) {
        reset(0);zdt_match.first_round_only=1;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=checkpoints[i];
        zdt_match.active_request=zdt_match_request=90U+i;zdt_match_io=MATCH_IO_ALIGN;
        ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ROUND1 && zdt_match.state==MATCH_MOVING && !test_material_calls);
        CHECK(!MatchControl_Ok(&zdt_match,test_tick+1));
        round_hal=HAL_BUSY;ZDT_MaterialService();CHECK(!zdt_arm_round_token && zdt_match.state==MATCH_MOVING);
        round_hal=HAL_OK;ZDT_MaterialService();CHECK(zdt_arm_round_token==90U+i && round_station==i+1);
        test_tick+=180000;ZDT_MaterialService();MatchControl_Service(&zdt_match,test_tick);CHECK(zdt_match.state==MATCH_MOVING);
        round_ready=round_success=1;ZDT_MaterialService();
        CHECK(zdt_match.state==MATCH_WAIT_OK && !zdt_match_request && !zdt_arm_round_token && !test_plate);
        CHECK(!MatchControl_Ok(&zdt_match,test_tick)); /* received at completion is old */
        CHECK(MatchControl_Ok(&zdt_match,++test_tick));
        if(i==2)CHECK(zdt_match.state==MATCH_FINISHED && zdt_match.checkpoint==MATCH_TEMP1 && !zdt_match.pending);
        else CHECK(zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==checkpoints[i+1]);
      }
      reset(0);zdt_match.first_round_only=1;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_ROUGH1;
      zdt_match.active_request=zdt_match_request=100;zdt_match_io=MATCH_IO_ALIGN;ZDT_Match_Complete(1,NULL);ZDT_MaterialService();
      round_ready=1;round_success=0;ZDT_MaterialService();CHECK(zdt_match.state==MATCH_FAULT);
      reset(0);zdt_match.first_round_only=1;zdt_match.checkpoint=MATCH_RAW1;zdt_align_enabled=0;
      zdt_match.active_request=zdt_match_request=110;zdt_match_io=MATCH_IO_NAV;
      ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ROUND1);zdt_align_enabled=1;
      round_hal=HAL_ERROR;ZDT_MaterialService();CHECK(zdt_match.state==MATCH_FAULT);
    }
    /* One-OK automatic mode: at RAW wait for concurrent arm deployment,
     * then real alignment/ROUND ownership gates the station result. No QR
     * camera setup, WAIT_OK or subsequent human confirmation is involved. */
    {
      unsigned station;
      reset(0);zdt_match.automatic=zdt_match.first_round_only=1;
      zdt_match.checkpoint=MATCH_RAW1;zdt_arm_camera_token=77;
      ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ARM_DEPLOY && !round_calls && !test_plate);
      zdt_arm_camera_token=0;
      for(station=1;station<=3;station++) {
        zdt_match_io=MATCH_IO_ALIGN;round_ready=round_success=0;
        ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ROUND1 && zdt_match.state==MATCH_MOVING);
        ZDT_MaterialService();CHECK(round_station==station && zdt_arm_round_token);
        CHECK(!MatchControl_Ok(&zdt_match,++test_tick));
        test_tick+=1000;ZDT_MaterialService();CHECK(zdt_match.state==MATCH_MOVING && !test_plate);
        round_ready=round_success=1;ZDT_MaterialService();
        if(station<3){CHECK(zdt_match.state==MATCH_MOVING && zdt_match.pending && zdt_match.checkpoint==(match_checkpoint_t)(station+1));MatchControl_Service(&zdt_match,++test_tick);}
        else CHECK(zdt_match.state==MATCH_FINISHED && zdt_match.checkpoint==MATCH_TEMP1 && !zdt_match.pending);
      }
      CHECK(round_calls==3 && !zdt_qr_active && !test_plate);
    }
    /* Full automatic station adapter passes jobs1..6 and defers the return
     * until the second TEMP job has actually completed its raised drops. */
    {
      unsigned job;reset(0);zdt_match.automatic=1;zdt_match.first_round_only=0;
      zdt_match.checkpoint=MATCH_RAW1;
      for(job=1;job<=6;job++){
        zdt_match_io=MATCH_IO_ALIGN;round_ready=round_success=0;
        ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ROUND1 && zdt_match.state==MATCH_MOVING);
        ZDT_MaterialService();CHECK(round_station==job && zdt_arm_round_token);
        CHECK(!MatchControl_Ok(&zdt_match,++test_tick));
        round_ready=round_success=1;ZDT_MaterialService();
        CHECK(zdt_match.state==MATCH_MOVING && zdt_match.pending && zdt_match.checkpoint==(match_checkpoint_t)(job+1));
        MatchControl_Service(&zdt_match,++test_tick);
      }
      CHECK(round_calls==6 && zdt_match.checkpoint==MATCH_RETURN);ZDT_Match_Complete(1,NULL);
      CHECK(zdt_match_io==MATCH_IO_ARM_DEPLOY && zdt_match.state==MATCH_MOVING);
      zdt_arm_camera_token=0;ZDT_Match_Complete(1,NULL);
      CHECK(zdt_match.state==MATCH_FINISHED && !test_plate && !zdt_qr_active);
    }
    /* All six next-station arrivals gate on the prior station's CAMERA
     * token. Chassis travel is allowed before it; vision/arm jobs are not. */
    {
      unsigned cp;
      for(cp=MATCH_RAW1;cp<=MATCH_RETURN;cp++) {
        reset(0);zdt_match.automatic=1;zdt_match.checkpoint=(match_checkpoint_t)cp;
        test_real_station=1;
        zdt_match_io=MATCH_IO_ZONE2;zdt_arm_camera_token=123;
        ZDT_Match_Complete(1,NULL);
        CHECK(zdt_match_io==MATCH_IO_ARM_DEPLOY && !round_calls && !test_plate && zdt_match.state==MATCH_MOVING);
        zdt_arm_camera_token=0;ZDT_Match_Complete(1,NULL);
        if(cp<=MATCH_TEMP2) {
          CHECK(zdt_match_io==MATCH_IO_ALIGN && test_align_calls==1 && !round_calls);
          ZDT_Match_Complete(1,NULL);CHECK(zdt_match_io==MATCH_IO_ROUND1);
        }
        else CHECK(zdt_match.state==MATCH_FINISHED); /* final return also waits for CAMERA */
      }
    }
}
