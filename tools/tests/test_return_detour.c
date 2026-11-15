#include "chassis_motor.h"
#include "grid_nav.h"
#include "zone2_route.h"
#include "nav_blend.h"
#include "nav_frame.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);} }while(0)
typedef enum {Z2_NONE,Z2_NAV,Z2_TURN} zdt_zone2_phase_t;
static zone2_route_t zdt_zone2;static zone2_action_t zdt_zone2_action;
static match_control_t zdt_match;static zdt_zone2_phase_t zdt_zone2_phase;
static uint32_t zdt_zone2_action_id,zdt_zone2_imu_count,zdt_zone2_imu_tick,zdt_zone2_tick;
static uint8_t zdt_nav_post_align,zdt_nav_queued_recenter;
static int8_t zdt_nav_nose_x=0,zdt_nav_nose_y=-1;
static const char *zdt_zone2_reason;
static float test_yaw;static uint32_t test_tick,test_nav_count,test_detour_count;
static uint8_t test_finished,test_success;
static uint32_t ReturnTick(void){return test_tick;}
static uint32_t ReturnSamples(void){return test_tick/10U;}
static float ZDT_Nav_GetHeading(void){return test_yaw;}
static void ZDT_Nav_RefreshCostModel(void){}
static float ZDT_Zone2_RelYaw(float world){return NavBlend_AngleError(world,90.0f);}
static uint8_t ZDT_Zone2_WheelsStopped(void){return 1U;}
static void ZDT_Zone2_Done(void){zdt_zone2_phase=Z2_NONE;Zone2Route_Result(&zdt_zone2,zdt_zone2_action_id,1U);}
static HAL_StatusTypeDef ReturnTurn(int16_t degrees){test_yaw+=degrees;return HAL_OK;}
static void ReturnPrint(const char *line){if(strstr(line,"checked A* detour"))test_detour_count++;}
static void ZDT_Nav_Start(void)
{
    grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];uint16_t count,j,x,y;int32_t px,py;
    CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&count));test_nav_count++;
    CHECK(count>0U);GridNav_GetPoseMm(&x,&y);px=x;py=y;
    for(j=0U;j<count;j++) {
        uint16_t d=GridNav_GetSegmentDistanceMm(j);float target=atan2f(-segments[j].dy,segments[j].dx)*57.29578f;
        float gap=fabsf(NavBlend_AngleError(target,test_yaw+90.0f));
        if(!(d<=150U && fabsf(gap-90.0f)<=3.0f))test_yaw=target-90.0f;
        px+=segments[j].dx*d;py-=segments[j].dy*d;
    }
    GridNav_SetPoseMm((uint16_t)px,(uint16_t)py);zdt_nav_post_align=1U;
}
#define HAL_GetTick ReturnTick
#define HWT101_GetSampleCount ReturnSamples
#define ChassisMotor_TurnRelative ReturnTurn
#define ChassisMotor_TurnRelativeFast ReturnTurn
#define ZDT_Console_Print ReturnPrint
/* Actual application action-start dispatch, with real field/route planners.
 * Hardware execution is stubbed and advanced by the planned signed lengths. */
#include "return_start_action_under_test.inc"
#undef HAL_GetTick
#undef HWT101_GetSampleCount
#undef ChassisMotor_TurnRelative
#undef ChassisMotor_TurnRelativeFast
#undef ZDT_Console_Print
static void cancel(void){zdt_zone2_phase=Z2_NONE;}
static void finished(uint8_t ok){test_finished=1U;test_success=ok;}
static void reset(uint8_t obstacle)
{
    const zone2_ports_t ports={ZDT_Zone2_StartAction,cancel,finished};
    GridNav_Init();GridNav_SetPoseMm(380U,1280U);
    if(obstacle)GridNav_SetObstacle(GridNav_MmToCellX(1200U),GridNav_MmToCellY(1200U),1U);
    Zone2Route_Init(&zdt_zone2,&ports);memset(&zdt_match,0,sizeof(zdt_match));
    zdt_match.zone=1U;zdt_match.checkpoint=MATCH_RETURN;zdt_nav_post_align=1U;
    zdt_nav_queued_recenter=0U;zdt_zone2_phase=Z2_NONE;zdt_zone2_reason=NULL;
    test_yaw=0.0f;test_tick=1000U;test_nav_count=test_detour_count=0U;test_finished=test_success=0U;
}
void Test_ReturnDetour(void)
{
    uint8_t obstacle,step;uint16_t x,y;zone2_action_t a={0};
    /* Direct departure-to-RAW transport is plannable from both exits. QR
     * action/facing are absent; normal corners still turn as required. */
    for(obstacle=1U;obstacle<=2U;obstacle++) {
        reset(0U);zdt_match.zone=obstacle;zdt_match.checkpoint=MATCH_RAW1;
        GridNav_SetPoseMm(2125U,obstacle==1U?450U:1950U);
        test_yaw=obstacle==1U?0.0f:-180.0f;zdt_nav_post_align=0U;
        CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RAW,MATCH_RAW1,obstacle));
        CHECK(zdt_zone2.count==2U && zdt_zone2.actions[0].x==1250U && zdt_zone2.actions[0].y==2100U);
        for(step=0;step<8 && zdt_zone2.running;step++) {
            test_tick+=100;Zone2Route_Service(&zdt_zone2,test_tick);
            if(zdt_zone2_phase!=Z2_NONE)ZDT_Zone2_Done();
        }
        CHECK(test_finished && test_success && test_nav_count==1U && !zdt_zone2_reason);
    }
    for(obstacle=0U;obstacle<=1U;obstacle++) {
        reset(obstacle);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RETURN,MATCH_RETURN,1U));
        for(step=0U;step<16U && zdt_zone2.running;step++) {
            test_tick+=100U;Zone2Route_Service(&zdt_zone2,test_tick);
            if(zdt_zone2_phase!=Z2_NONE)ZDT_Zone2_Done();
        }
        CHECK(test_finished && test_success && test_nav_count==2U && !zdt_zone2_reason);
        GridNav_GetPoseMm(&x,&y);CHECK(abs((int)x-2125)<=8 && abs((int)y-350)<=8);
        CHECK(fabsf(NavBlend_AngleError(-180.0f,test_yaw))<1.0f);
        CHECK(obstacle?test_detour_count>=1U:test_detour_count==0U);
    }
    /* A blocked endpoint must not be silently snapped to a different goal. */
    reset(1U);test_yaw=-90.0f;
    GridNav_SetObstacle(GridNav_MmToCellX(2125U),GridNav_MmToCellY(1200U),1U);
    a.kind=ZONE2_NAV;a.x=2125U;a.y=1200U;a.name="qr_return";
    CHECK(!ZDT_Zone2_StartAction(&a,1U) && !test_nav_count && zdt_zone2_reason);
    /* After the30mm approach, withdraw sideways before the lane-join sweep. */
    for(obstacle=1U;obstacle<=2U;obstacle++) {
        reset(0U);zdt_match.zone=obstacle;zdt_match.checkpoint=MATCH_ROUGH1;
        GridNav_SetPoseMm(1250U,2120U);test_yaw=-90.0f;
        CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_ROUGH,MATCH_ROUGH1,obstacle));
        CHECK(zdt_zone2.count==3U);
        for(step=0;step<8 && zdt_zone2.running;step++) {
            test_tick+=100;Zone2Route_Service(&zdt_zone2,test_tick);
            if(zdt_zone2_phase!=Z2_NONE)ZDT_Zone2_Done();
        }
        GridNav_GetPoseMm(&x,&y);CHECK(x==1200U && y==400U);
        CHECK(test_finished && test_success && test_nav_count==2U && !zdt_zone2_reason);
    }
}
