#include "vision_yaw.h"
#include "vision.h"
#include "vision_align.h"
#include "chassis_motor.h"
#include "match_control.h"
#include "nav_frame.h"
#include "nav_blend.h"
#include "zdt_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static uint32_t now=1000,observation=1,console_calls,zdt_align_status_tick;
static vision_ring_axis_t axis;
static vision_ring_axis_t make_axis(uint8_t station,float angle)
{
    vision_ring_axis_t a={0};a.valid=1;a.id=2;a.station=station;a.request_seq=7;
    a.confidence=90;a.landmarks=3;a.axis_deg=angle;a.observation_id=observation++;return a;
}
static void observe3(vision_yaw_t *v,uint8_t station,float angle)
{unsigned i;for(i=0;i<3;i++){now+=220;axis=make_axis(station,angle);VisionYaw_Observe(v,&axis,0,now,1);}}
static void step3(vision_yaw_t *v,float angle,float h,uint8_t done)
{unsigned i;for(i=0;i<3;i++){now+=220;axis=make_axis(v->station,angle);VisionYaw_Step(v,now,&axis,0,h,0,1,done);}}
static void calibrated(vision_yaw_t *v)
{
    VisionYaw_Init(v);CHECK(!VisionYaw_Enable(v,1));
    observe3(v,2,0);CHECK(VisionYaw_Zero(v,2,now));CHECK(VisionYaw_Sample(v,2,now,0));
    observe3(v,2,-3);CHECK(VisionYaw_Solve(v,2,now,3));CHECK(fabsf(v->cal[0].gain-1)<0.001f);
    observe3(v,3,0);CHECK(VisionYaw_Zero(v,3,now));CHECK(VisionYaw_Sample(v,3,now,0));
    observe3(v,3,3);CHECK(VisionYaw_Solve(v,3,now,3));CHECK(fabsf(v->cal[1].gain+1)<0.001f);
    CHECK(VisionYaw_Enable(v,1) && v->phase==V_YAW_IDLE && !v->turn_allowed);
}
static void begin_yaw(vision_yaw_t *v,float h,float angle)
{
    CHECK(VisionYaw_Begin(v,2,7,now,h));axis=make_axis(2,angle);
    VisionYaw_Step(v,now,&axis,0,h,0,1,1);CHECK(v->phase==V_YAW_ACQUIRE && !v->turn_allowed);
    step3(v,angle,h,1);
}
/* Compile the actual ZDT command and scene-reader adapters with inert ports. */
static vision_yaw_t zdt_vyaw;
static match_control_t zdt_match;
static uint8_t busy,imu_ready=1,zdt_match_io,zdt_align_status_pending;
#define MATCH_IO_ALIGN 1U
static uint32_t camera_requests,scene_reads,pixel_reads,zdt_align_imu_count,zdt_align_imu_tick;
static uint8_t align_clock_step,align_hold_samples,align_enabled=1,zdt_align_phase=4;
static uint32_t align_held_sample=99,align_health_passes,align_failures,align_stops;
static const char *align_failure_reason;
uint32_t HWT101_GetSampleCount(void){return align_hold_samples?align_held_sample:now;}
static vision_ring_axis_t current_axis;
static uint8_t coupled_inputs,coupled_moving;
static vision_pixel_t coupled_pixel;
static vision_ring_count_t coupled_scene;
static uint32_t coupled_age;
static chassis_telemetry_t telemetry;
static char console[320];
uint32_t HAL_GetTick(void){now+=align_clock_step;return now;}
static uint8_t ZDT_Align_DiagBusy(void){return busy;}
uint8_t HWT101_IsZeroDriftReady(void){return imu_ready;}
uint8_t MatchControl_IsActive(const match_control_t *m){return m->state==MATCH_MOVING;}
void ChassisMotor_GetTelemetry(chassis_telemetry_t *t){*t=telemetry;}
void ChassisMotor_GetWheelRpm(int16_t rpm[4]){memset(rpm,0,4*sizeof(*rpm));if(coupled_moving)rpm[0]=10;}
uint8_t ChassisMotor_HasMotionDemand(void){return busy;}
void ZDT_Console_Print(const char *s){console_calls++;strncpy(console,s,sizeof(console)-1);}
uint8_t Vision_GetRingAxisWindow(uint8_t camera,uint16_t window,vision_ring_axis_t *a,uint32_t *age)
{CHECK(camera==1 && window==500);*a=current_axis;*age=coupled_age;return 1;}
HAL_StatusTypeDef Vision_RequestMode(uint8_t camera,uint8_t mode,uint8_t station,uint8_t target,uint8_t hz,uint8_t flags)
{CHECK(camera==1 && mode==3 && (station==2 || station==3) && target==2 && hz==20 && flags==1);camera_requests++;return HAL_OK;}
uint8_t Vision_GetRingSceneWindow(uint8_t camera,uint16_t window,vision_pixel_t *p,vision_ring_count_t *c,uint32_t *age)
{(void)camera;(void)window;memset(p,0,sizeof(*p));memset(c,0,sizeof(*c));*age=0;scene_reads++;if(coupled_inputs){*p=coupled_pixel;*c=coupled_scene;*age=coupled_age;}return 1;}
uint8_t Vision_GetPlateSceneWindow(uint8_t camera,uint16_t window,vision_pixel_t *p,vision_plate_scene_t *c,uint32_t *age)
{(void)camera;(void)window;(void)p;(void)c;(void)age;return 0;}
uint8_t Vision_GetPixelWindow(uint8_t camera,uint8_t type,uint16_t window,vision_pixel_t *p,uint32_t *age)
{(void)camera;(void)type;(void)window;memset(p,0,sizeof(*p));*age=0;pixel_reads++;return 1;}
uint8_t ChassisMotor_IsEnabled(void){return align_enabled;}
HAL_StatusTypeDef ChassisMotor_StopImmediate(void){align_stops++;return HAL_OK;}
static vision_align_t zdt_align;
static vision_ring_count_t zdt_align_scene;
static vision_plate_scene_t zdt_align_plate_scene;
static float zdt_align_heading;
static int8_t zdt_nav_nose_x=1,zdt_nav_nose_y=0;
static uint32_t coupled_calls,coupled_finished;
static float sent_forward,sent_right,sent_omega;
#define ZDT_NAV_HOLD_KP (0.030f)
#define ZDT_NAV_HOLD_DEADBAND_DEG (0.4f)
HAL_StatusTypeDef ChassisMotor_Stop(void){sent_forward=sent_right=sent_omega=0;return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_SetVelocity(float f,float r,float o){sent_forward=f;sent_right=r;sent_omega=o;coupled_calls++;return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_SetFieldVelocity(float nx,float ny,float x,float y,float o){(void)nx;(void)ny;return ChassisMotor_SetVelocity(x,y,o);}
static void ZDT_Mileage_StartQuery(uint8_t ignored){(void)ignored;coupled_finished++;}
static void ZDT_Match_Complete(uint8_t success,const char *reason){CHECK(!success);align_failures++;align_failure_reason=reason;}
static void ZDT_Align_DiagObserve(uint32_t time,const vision_pixel_t *p,uint32_t age,float f,float r,float o)
{(void)time;(void)p;(void)age;(void)f;(void)r;(void)o;}
#include "vyaw_adapter.inc"
static void simultaneous_dispatch(void)
{
    vision_align_config_t config;unsigned station;uint32_t before;
    for(station=2;station<=3;station++) {
        coupled_inputs=1;coupled_moving=1;coupled_age=0;coupled_calls=coupled_finished=0;
        align_failures=0;telemetry.heading_deg=telemetry.gyro_dps=0;zdt_align_phase=2;
        CHECK(VisionAlign_StationConfig(&config,(uint8_t)station));
        CHECK(VisionAlign_Start(&zdt_align,&config,now,0));
        zdt_align.state=V_ALIGN_TRACK;zdt_align.last_valid=now;
        VisionYaw_UseInstallationDefaults(&zdt_vyaw);CHECK(VisionYaw_Begin(&zdt_vyaw,(uint8_t)station,7,now,0));
        current_axis=make_axis((uint8_t)station,3);
        memset(&coupled_pixel,0,sizeof(coupled_pixel));coupled_pixel.valid=1;coupled_pixel.type=VISION_TYPE_RING_PX;coupled_pixel.id=2;
        coupled_pixel.dx_px=40;coupled_pixel.dy_px=20;coupled_pixel.observation_id=current_axis.observation_id;
        memset(&coupled_scene,0,sizeof(coupled_scene));coupled_scene.valid=1;coupled_scene.count=3;coupled_scene.id=2;
        coupled_scene.station=(uint8_t)station;coupled_scene.request_seq=7;coupled_scene.observation_id=current_axis.observation_id;
        ZDT_Align_CoupledUnderTest();
        CHECK(!align_failures && coupled_calls==1 && !coupled_finished);
        CHECK((sent_forward!=0 || sent_right!=0) && sent_omega>0 && zdt_vyaw.phase==V_YAW_COMBINED);
        /* Yaw-induced XY displacement after DONE or in-band SETTLE must
         * reopen tracking immediately; neither loop can finish the other. */
        zdt_align.state=V_ALIGN_DONE;zdt_align.stable_count=5;before=coupled_calls;
        now+=20;current_axis=make_axis((uint8_t)station,2);
        coupled_pixel.observation_id=coupled_scene.observation_id=current_axis.observation_id;
        ZDT_Align_CoupledUnderTest();CHECK(coupled_calls==before+1 && sent_omega>0 && sent_forward!=0 && !coupled_finished);
        zdt_align.state=V_ALIGN_SETTLE;now+=20;current_axis=make_axis((uint8_t)station,1);
        coupled_pixel.observation_id=coupled_scene.observation_id=current_axis.observation_id;
        ZDT_Align_CoupledUnderTest();CHECK(zdt_align.state==V_ALIGN_TRACK && sent_forward!=0 && sent_omega>0);
        /* A mismatched image can keep valid XY, but must never turn using
         * an angle belonging to the previous image. */
        now+=20;coupled_pixel.observation_id=coupled_scene.observation_id=++observation;
        ZDT_Align_CoupledUnderTest();CHECK(sent_omega==0 && !coupled_finished);
        current_axis=make_axis((uint8_t)station,0);coupled_pixel.observation_id=coupled_scene.observation_id=current_axis.observation_id;
        coupled_pixel.dx_px=coupled_pixel.dy_px=0;coupled_moving=0;
        zdt_align.state=V_ALIGN_DONE;coupled_pixel.valid=0;
        now+=20;ZDT_Align_CoupledUnderTest();CHECK(!coupled_finished && !zdt_vyaw.stable);
        coupled_pixel.valid=1;now+=20;ZDT_Align_CoupledUnderTest();CHECK(!coupled_finished);
        now+=220;current_axis=make_axis((uint8_t)station,0);coupled_pixel.observation_id=coupled_scene.observation_id=current_axis.observation_id;ZDT_Align_CoupledUnderTest();
        now+=220;current_axis=make_axis((uint8_t)station,0);coupled_pixel.observation_id=coupled_scene.observation_id=current_axis.observation_id;ZDT_Align_CoupledUnderTest();
        CHECK(coupled_finished==1 && zdt_vyaw.phase==V_YAW_DONE && !align_failures);
    }
    coupled_inputs=coupled_moving=0;coupled_age=0;
}
static void missing_scene_yaw_search(void)
{
    vision_align_t a={0};vision_yaw_t v={0};
    a.state=V_ALIGN_SEARCH;v.phase=V_YAW_ACQUIRE;
    CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
    v.turn_allowed=1;CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
    v.turn_allowed=0;v.phase=V_YAW_TURN;CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
    v.phase=V_YAW_ACQUIRE;a.state=V_ALIGN_TRACK;CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
    a.state=V_ALIGN_WAIT;CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
    a.state=V_ALIGN_SEARCH;CHECK(!ZDT_Align_SearchWithYaw(&a,&v));
}
static void align_health_reset(uint32_t time)
{
    now=time;align_clock_step=0;align_hold_samples=1;align_enabled=imu_ready=1;busy=0;
    zdt_align_phase=4;zdt_match_io=MATCH_IO_ALIGN;align_held_sample=99;
    zdt_align_imu_count=0;zdt_align_imu_tick=time;
    align_health_passes=align_failures=align_stops=0;align_failure_reason=NULL;
    memset(&telemetry,0,sizeof(telemetry));memset(&current_axis,0,sizeof(current_axis));VisionYaw_Init(&zdt_vyaw);
}
static void alignment_clock_regression(void)
{
    unsigned i;static const uint32_t starts[]={1000U,UINT32_MAX-2U};
    /* IMU sample count stays fixed after the observer takes a NEW sample.
     * Clock crosses milliseconds inside that observer, including wraparound. */
    for(i=0;i<2;i++){
        align_health_reset(starts[i]);align_clock_step=1;
        ZDT_Align_HealthUnderTest();
        CHECK(align_health_passes==1 && !align_failures && !align_stops);
        CHECK(now-zdt_align_imu_tick<=3U);
    }
    /* Real age boundaries and genuine disable/not-ready/non-finite faults
     * must still stop; do not turn clock correction into relaxed freshness. */
    align_health_reset(1000);zdt_align_imu_count=99;zdt_align_imu_tick=850;
    ZDT_Align_HealthUnderTest();CHECK(align_health_passes==1 && !align_failures);
    now=1001;ZDT_Align_HealthUnderTest();CHECK(align_failures==1 && align_stops==1 && !strcmp(align_failure_reason,"alignment IMU/motor unavailable"));
    for(i=0;i<4;i++){
        align_health_reset(1000);
        if(i==0)align_enabled=0;
        if(i==1)imu_ready=0;
        if(i==2)telemetry.heading_deg=NAN;
        if(i==3)telemetry.gyro_dps=NAN;
        ZDT_Align_HealthUnderTest();CHECK(align_failures==1 && align_stops==1 && !align_health_passes);
    }
    align_health_reset(1000);zdt_match_io=0;ZDT_Align_HealthUnderTest();CHECK(!align_health_passes && !align_failures);
    align_clock_step=align_hold_samples=0;imu_ready=align_enabled=1;zdt_match_io=0;
}
static void adapters(void)
{
    vision_align_config_t cfg;vision_pixel_t p;vision_ring_count_t c;vision_plate_scene_t plate;uint32_t age;
    scene_reads=pixel_reads=0;
    VisionYaw_Init(&zdt_vyaw);CHECK(ZDT_VyawCommand("vyaw on") && !zdt_vyaw.enabled && !camera_requests);
    CHECK(ZDT_VyawCommand("vyaw view 2") && camera_requests==1 && !zdt_vyaw.enabled);
    {unsigned i;for(i=0;i<3;i++){now+=220;current_axis=make_axis(2,0);ZDT_VyawObserve();}}
    CHECK(ZDT_VyawCommand("vyaw zero 3") && !zdt_vyaw.cal[1].zero_valid);
    CHECK(ZDT_VyawCommand("vyaw zero 2") && zdt_vyaw.cal[0].zero_valid);
    CHECK(ZDT_VyawCommand("vyaw gain 2 -1") && zdt_vyaw.cal[0].gain_valid);
    CHECK(!camera_requests || camera_requests==1); /* Only VIEW requests the camera. */
    busy=1;CHECK(ZDT_VyawCommand("vyaw view 3") && camera_requests==1);
    zdt_match_io=MATCH_IO_ALIGN;CHECK(ZDT_VyawCommand("vyaw status") && zdt_align_status_pending);
    busy=0;zdt_match_io=zdt_align_status_pending=0;
    CHECK(VisionAlign_StationConfig(&cfg,2));cfg.search_axis=0;
    zdt_vyaw.phase=V_YAW_COARSE;ZDT_Align_ReadPixels(&cfg,&p,&c,&plate,&age);
    CHECK(scene_reads==1 && pixel_reads==0); /* Search OFF + yaw ON uses a same-image scene. */
    zdt_vyaw.phase=V_YAW_IDLE;ZDT_Align_ReadPixels(&cfg,&p,&c,&plate,&age);
    CHECK(pixel_reads==1); /* Disabled path remains the original pixel reader. */
    zdt_align.config=cfg;zdt_align.state=V_ALIGN_DONE;zdt_align.stable_count=5;
    zdt_vyaw.phase=V_YAW_COMBINED;zdt_vyaw.stable=1;current_axis=make_axis(2,0.6f);
    current_axis.confidence=79;ZDT_Align_TimeoutDetails();
    CHECK(strstr(console,"[ALIGN TIMEOUT]") && strstr(console,"xy=4 stable=5") && strstr(console,"q=79"));
}
static void simultaneous(void)
{
    vision_yaw_t v;unsigned station;
    for(station=2U;station<=3U;station++) {
        VisionYaw_UseInstallationDefaults(&v);CHECK(v.simultaneous);
        CHECK(VisionYaw_Begin(&v,(uint8_t)station,7,now,3));axis=make_axis((uint8_t)station,-3);
        axis.landmarks=2;VisionYaw_Step(&v,now,&axis,0,3,0,0,0);
        CHECK(v.phase==V_YAW_COARSE && !v.turn_allowed);
        axis=make_axis((uint8_t)station,-3);now+=20;
        VisionYaw_Step(&v,now,&axis,0,3,-2,0,0);
        CHECK(v.phase==V_YAW_COMBINED && v.turn_allowed && fabsf(v.goal_heading)<0.001f);
        now+=10;VisionYaw_Step(&v,now,&axis,10,1,-2,0,0);
        CHECK(fabsf(v.goal_heading)<0.001f); /* Same image cannot walk the goal. */
        axis=make_axis((uint8_t)station,-2);now+=20;
        VisionYaw_Step(&v,now,&axis,0,2,-2,0,0);CHECK(v.turn_allowed && v.phase==V_YAW_COMBINED);
        axis.valid=0;now+=20;VisionYaw_Step(&v,now,&axis,0,2,0,0,0);
        CHECK(!v.turn_allowed && v.phase==V_YAW_COMBINED && v.goal_heading==2);
        axis=make_axis((uint8_t)station,0);now+=20;VisionYaw_Step(&v,now,&axis,0,0,0,1,0);
        CHECK(v.phase==V_YAW_COMBINED && !v.turn_allowed); /* XY still needs precision. */
        step3(&v,0,0,1);CHECK(v.phase==V_YAW_DONE);
    }
    VisionYaw_UseInstallationDefaults(&v);CHECK(VisionYaw_Begin(&v,2,7,now,0));
    axis=make_axis(2,9);VisionYaw_Step(&v,now,&axis,0,0,0,0,0);CHECK(v.phase==V_YAW_FAILED);
    VisionYaw_UseInstallationDefaults(&v);CHECK(VisionYaw_Begin(&v,2,7,now,0));
    axis=make_axis(2,2);VisionYaw_Step(&v,now,&axis,0,0,0,0,0);
    now+=8000;axis=make_axis(2,2);VisionYaw_Step(&v,now,&axis,0,0,0,0,0);CHECK(v.phase==V_YAW_FAILED);
    {const float accepted[]={0.8f,-0.8f};unsigned i;
     for(i=0;i<2;i++) {
        VisionYaw_UseInstallationDefaults(&v);CHECK(VisionYaw_Begin(&v,2,7,now,0));
        step3(&v,accepted[i],0,1);CHECK(v.phase==V_YAW_DONE && !v.turn_allowed);
     }
     VisionYaw_UseInstallationDefaults(&v);CHECK(VisionYaw_Begin(&v,2,7,now,0));
     axis=make_axis(2,0.81f);VisionYaw_Step(&v,now,&axis,0,0,0,1,1);
     CHECK(v.phase==V_YAW_COMBINED && v.turn_allowed);
    }
}
static void automatic_alignment_snapshot(void)
{
    uint32_t before;
    align_health_reset(10000U);zdt_align_phase=2;VisionYaw_UseInstallationDefaults(&zdt_vyaw);
    CHECK(VisionYaw_Begin(&zdt_vyaw,3,7,now,0));zdt_vyaw.phase=V_YAW_COMBINED;
    CHECK(VisionAlign_StationConfig(&zdt_align.config,3));zdt_align.state=V_ALIGN_DONE;
    current_axis=make_axis(3,0.35f);current_axis.confidence=79;zdt_align_status_tick=now;
    zdt_align_status_pending=0;before=console_calls;
    now+=4999;ZDT_Align_ProgressService();CHECK(console_calls==before);
    now++;busy=1;ZDT_Align_ProgressService();CHECK(console_calls==before && zdt_align_status_pending);
    busy=0;ZDT_Align_ProgressService();
    CHECK(console_calls==before+2 && !zdt_align_status_pending && zdt_align_status_tick==now);
    CHECK(strstr(console,"[VYAW] LIVE") && strstr(console,"q=79"));
    before=console_calls;now+=1000;ZDT_Align_ProgressService();CHECK(console_calls==before);
    zdt_vyaw.phase=V_YAW_IDLE;now+=5000;ZDT_Align_ProgressService();CHECK(console_calls==before);
    zdt_align_status_pending=1;ZDT_Align_ProgressService();CHECK(console_calls==before+1); /* Manual status retained. */
}
int main(void)
{
    simultaneous();simultaneous_dispatch();
    /* Both public ring stations run coarse XY -> yaw -> fresh XY -> yaw check. */
    {
        vision_yaw_t common;unsigned station;
        for(station=2U;station<=3U;station++) {
            VisionYaw_UseInstallationDefaults(&common);
            common.simultaneous=0U; /* Legacy sequential path retained. */
            CHECK(VisionYaw_Begin(&common,(uint8_t)station,7,now,0));
            axis=make_axis((uint8_t)station,0);
            VisionYaw_Step(&common,now,&axis,0,0,0,1,0);
            CHECK(common.phase==V_YAW_COARSE);
            VisionYaw_Step(&common,now,&axis,0,0,0,1,1);
            CHECK(common.phase==V_YAW_ACQUIRE);
            step3(&common,0,0,1);CHECK(common.phase==V_YAW_FINE && common.restart_xy);
            axis=make_axis((uint8_t)station,0);now+=220;
            VisionYaw_Step(&common,now,&axis,0,0,0,1,0);
            CHECK(common.phase==V_YAW_FINE);
            step3(&common,0,0,1);CHECK(common.phase==V_YAW_DONE);
        }
    }
    vision_yaw_t v;float gain;
    VisionYaw_UseInstallationDefaults(&v);
    v.simultaneous=0U;
    CHECK(v.enabled && v.cal[0].zero_valid && v.cal[1].zero_valid && v.cal[0].gain_valid && v.cal[1].gain_valid);
    CHECK(v.cal[0].zero==0 && v.cal[1].zero==0 && v.cal[0].gain==1 && v.cal[1].gain==1);
    CHECK(VisionYaw_Begin(&v,2,7,now,3));axis=make_axis(2,-3);
    VisionYaw_Step(&v,now,&axis,0,3,0,1,1);step3(&v,-3,3,1);
    CHECK(v.phase==V_YAW_TURN && fabsf(v.error+3)<0.001f && fabsf(v.goal_heading)<0.001f);
    CHECK(VisionYaw_Begin(&v,3,7,now,-3));axis=make_axis(3,3);
    VisionYaw_Step(&v,now,&axis,0,-3,0,1,1);step3(&v,3,-3,1);
    CHECK(v.phase==V_YAW_TURN && fabsf(v.error-3)<0.001f && fabsf(v.goal_heading)<0.001f);
    CHECK(VisionYaw_Enable(&v,0) && !v.enabled);VisionYaw_Cancel(&v);CHECK(!v.enabled);
    CHECK(fabsf(VisionYaw_LineError(179.8f)+0.2f)<0.001f);
    CHECK(fabsf(VisionYaw_LineError(-179.8f)-0.2f)<0.001f);
    calibrated(&v);gain=v.cal[0].gain;
    CHECK(VisionYaw_Begin(&v,3,7,now,3));axis=make_axis(3,3);
    VisionYaw_Step(&v,now,&axis,0,3,0,1,1);step3(&v,3,3,1);
    CHECK(v.phase==V_YAW_TURN && fabsf(v.error+3.0f)<0.001f && fabsf(v.goal_heading)<0.001f); /* Mirrored gain=-1. */

    begin_yaw(&v,3,-3);CHECK(v.phase==V_YAW_TURN && v.turn_allowed && fabsf(v.goal_heading)<0.001f);
    axis=make_axis(2,-2);now+=20;VisionYaw_Step(&v,now,&axis,0,2,-2,0,0);CHECK(v.turn_allowed);
    axis.valid=0;now+=20;VisionYaw_Step(&v,now,&axis,0,2,0,1,0);
    CHECK(!v.turn_allowed && v.phase==V_YAW_ACQUIRE && v.goal_heading==2);
    now+=500;step3(&v,-2,2,1);CHECK(v.phase==V_YAW_TURN && v.turn_allowed && fabsf(v.goal_heading)<0.001f);
    axis=make_axis(2,0);now+=20;VisionYaw_Step(&v,now,&axis,0,0,0,1,1);CHECK(v.phase==V_YAW_ACQUIRE);
    now+=500;step3(&v,0,0,1);CHECK(v.phase==V_YAW_FINE && v.restart_xy);
    step3(&v,0,0,1);CHECK(v.phase==V_YAW_DONE && !v.turn_allowed);
    VisionYaw_Cancel(&v);CHECK(v.enabled && v.cal[0].gain==gain && v.cal[0].zero_valid);
    CHECK(VisionYaw_Enable(&v,0) && !v.enabled && v.cal[1].gain_valid);
    CHECK(VisionYaw_Enable(&v,1) && !v.turn_allowed && v.phase==V_YAW_IDLE);
    CHECK(!VisionYaw_Begin(&v,1,7,now,0));
    /* Timeout, oversized image error, actual excursion and bad-gain budget. */
    begin_yaw(&v,0,9);CHECK(v.phase==V_YAW_FAILED && !v.turn_allowed);
    CHECK(VisionYaw_Begin(&v,2,7,now,0));axis=make_axis(2,0);VisionYaw_Step(&v,now,&axis,0,0,0,1,1);
    now+=8000;VisionYaw_Step(&v,now,&axis,500,0,0,1,1);CHECK(v.phase==V_YAW_FAILED && !v.turn_allowed);
    begin_yaw(&v,0,7);CHECK(v.phase==V_YAW_TURN);
    axis=make_axis(2,4);now+=20;VisionYaw_Step(&v,now,&axis,0,7,0,1,1);
    now+=500;step3(&v,4,7,1);CHECK(v.phase==V_YAW_FAILED && !v.turn_allowed);
    begin_yaw(&v,0,2);axis=make_axis(2,0);now+=20;VisionYaw_Step(&v,now,&axis,0,11,0,1,0);
    CHECK(v.phase==V_YAW_FAILED && !v.turn_allowed);
    /* A lost session or 500ms sample cannot keep the old turn alive. */
    begin_yaw(&v,0,2);axis=make_axis(2,2);axis.request_seq=8;now+=20;
    VisionYaw_Step(&v,now,&axis,0,0,0,1,1);CHECK(!v.turn_allowed);
    begin_yaw(&v,0,2);axis=make_axis(2,2);now+=20;
    VisionYaw_Step(&v,now,&axis,500,0,0,1,1);CHECK(!v.turn_allowed);
    /* Final yaw must be measured again after XY, with three stable frames. */
    begin_yaw(&v,0,0);CHECK(v.phase==V_YAW_FINE);step3(&v,1,0,1);
    CHECK(v.phase==V_YAW_FAILED && !v.turn_allowed);
    CHECK(!VisionYaw_Gain(&v,2,NAN) && !VisionYaw_Gain(&v,2,0) && !VisionYaw_Gain(&v,2,5));
    /* Actual0.62deg field residual and both inclusive0.8deg boundaries.
     * Larger stable residuals still fail; no turn uses stale observations. */
    {
      const float accepted[]={0.62f,-0.62f,0.79f,-0.79f,0.8f,-0.8f};unsigned i;
      for(i=0;i<sizeof(accepted)/sizeof(accepted[0]);i++) {
        begin_yaw(&v,0,0);CHECK(v.phase==V_YAW_FINE);step3(&v,accepted[i],0,1);
        CHECK(v.phase==V_YAW_DONE && !v.turn_allowed);
      }
      begin_yaw(&v,0,0);step3(&v,0.81f,0,1);CHECK(v.phase==V_YAW_FAILED);
      begin_yaw(&v,0,0);step3(&v,-0.81f,0,1);CHECK(v.phase==V_YAW_FAILED);
    }
    VisionYaw_Init(&v);axis=make_axis(2,0);axis.confidence=79;
    VisionYaw_Observe(&v,&axis,0,now,1);CHECK(!VisionYaw_Zero(&v,2,now));
    missing_scene_yaw_search();adapters();alignment_clock_regression();automatic_alignment_snapshot();return test_failures?1:0;
}
