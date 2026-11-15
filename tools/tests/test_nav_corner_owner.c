#include "nav_corner.h"
#include "nav_blend.h"
#include "grid_nav.h"
#include "zone2_route.h"
#include "chassis_motor.h"
#include "nav_params.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);}}while(0)
enum{ZDT_NAV_IDLE,ZDT_NAV_WAIT_START_QUERY,ZDT_NAV_OUTER_CORNER};
enum{MATCH_IO_ZONE2};enum{Z2_NAV};
static uint8_t zdt_nav_state,zdt_nav_odometry_enabled=1,zdt_nav_start_mask=15,zdt_odo_owner;
static uint8_t zdt_nav_drive_lateral,zdt_match_io,zdt_zone2_phase,zdt_nav_align_done,zdt_nav_post_align;
static int8_t zdt_nav_move_sign=1,zdt_nav_nose_x,zdt_nav_nose_y=-1;
static uint16_t zdt_nav_segment_index,zdt_nav_segment_count;
static grid_nav_segment_t zdt_nav_segments[4];
static float distances[4],zdt_nav_segment_target_rev;
static uint8_t zdt_nav_wait_start_query,zdt_nav_wait_end_query,zdt_nav_start_ready;
static uint8_t zdt_outer_enabled=1,zdt_outer_cross_action,zdt_outer_sample_ready;
static float zdt_outer_radius=NAV_CORNER_DEFAULT_RADIUS_MM,zdt_outer_speed=NAV_CORNER_DEFAULT_SPEED_MM_S;
static nav_corner_t zdt_outer;static int32_t zdt_outer_position[4],zdt_nav_start_position[4],zdt_mileage_query_position[4];
static uint32_t zdt_outer_sample_tick,zdt_outer_control_tick,zdt_outer_query_tick,zdt_outer_deadline;
static uint32_t zdt_outer_imu_count,zdt_outer_imu_tick,zdt_outer_progress_tick,zdt_outer_settle_tick;
static uint32_t zdt_outer_started_count,zdt_outer_fallback_count;
static float zdt_outer_progress_mark;static uint8_t zdt_outer_stage;
static zone2_route_t zdt_zone2;static zone2_action_t zdt_zone2_action;
static uint8_t zdt_mileage_query_active,test_stopped=1,test_demand,test_imu=1,test_enabled=1,test_busy;
static uint32_t zdt_mileage_query_sample_tick;
static uint32_t now=1000,samples=1,queries,field_calls,failures,arrivals;
static float test_yaw=90,test_gyro,last_x,last_y,last_o;
static const char *last_failure;static HAL_StatusTypeDef field_hal=HAL_OK;
#define ZDT_MILEAGE_MM_PER_REV (255.694226f)
#define ZDT_NAV_MM_S_PER_RPM (255.694226f/60.0f)
#define ZDT_NAV_DECEL_MM_S2 (292.0f*255.694226f/60.0f)
static const double zdt_mileage_forward_signs[4]={1,-1,1,-1};
static uint16_t goal_x,goal_y;
static uint32_t Tick(void){return now;}
static float ZDT_Nav_GetHeading(void){return test_yaw;}
static float ZDT_Nav_MatchExtraMm(void){return 0;}
static float Distance(uint16_t index){CHECK(index<4);return distances[index];}
static uint16_t PlannedDistance(uint16_t index){return GridNav_GetSegmentDistanceMm(index);}
static void Goal(uint16_t *x,uint16_t *y){*x=goal_x;*y=goal_y;}
static uint8_t Enabled(void){return test_enabled;}
static uint8_t Ready(void){return test_imu;}
static uint32_t Samples(void){return samples;}
static uint8_t Demand(void){return test_demand;}
static void Telemetry(chassis_telemetry_t *t){memset(t,0,sizeof(*t));t->heading_deg=test_yaw;t->gyro_dps=test_gyro;}
static uint8_t ZDT_Zone2_WheelsStopped(void){return test_stopped;}
static uint8_t ZDT_Align_DiagBusy(void){return test_busy;}
static void ResetRamp(void){}
static HAL_StatusTypeDef Stop(void){last_x=last_y=last_o=0;return HAL_OK;}
static HAL_StatusTypeDef Field(float fx,float fy,float vx,float vy,float o)
{CHECK(fx==0 && fy==1);field_calls++;last_x=vx;last_y=vy;last_o=o;return field_hal;}
static void Print(const char *s){CHECK(s && *s);}
static void ZDT_Nav_Fail(const char *s){failures++;last_failure=s;zdt_nav_state=ZDT_NAV_IDLE;zdt_outer_cross_action=0;}
static void ZDT_Mileage_StartQuery(uint8_t report){CHECK(!report);queries++;zdt_mileage_query_active=1;}
static void ZDT_Nav_NotifyArrived(void){arrivals++;Zone2Route_Result(&zdt_zone2,7,1);}
#define HAL_GetTick Tick
#define ChassisMotor_IsEnabled Enabled
#define ChassisMotor_HasMotionDemand Demand
#define ChassisMotor_GetTelemetry Telemetry
#define HWT101_IsZeroDriftReady Ready
#define HWT101_GetSampleCount Samples
#define ChassisMotor_ResetIdleRamp ResetRamp
#define ChassisMotor_Stop Stop
#define ChassisMotor_SetFieldVelocity Field
#define ZDT_Console_Print Print
#define GridNav_GetSegmentDistanceMm Distance
#define GridNav_GetTargetMm Goal
#include "outer_corner_owner_under_test.inc"
static void reset(uint8_t internal)
{
    GridNav_Init();GridNav_SetPoseMm(1150,400);Nav_Params_Reset();now=1000;samples=1;
    memset(&zdt_outer,0,sizeof(zdt_outer));memset(&zdt_zone2,0,sizeof(zdt_zone2));
    memset(zdt_nav_segments,0,sizeof(zdt_nav_segments));memset(zdt_nav_start_position,0,sizeof(zdt_nav_start_position));
    memset(zdt_mileage_query_position,0,sizeof(zdt_mileage_query_position));
    zdt_zone2.running=1;zdt_zone2.count=internal?2:3;zdt_zone2.index=0;zdt_zone2.active_id=7;
    zdt_zone2_action.kind=ZONE2_NAV;zdt_zone2_action.x=350;zdt_zone2_action.y=400;zdt_zone2_action.extra_mm=0;
    zdt_zone2.actions[0]=zdt_zone2_action;zdt_zone2.actions[1].kind=ZONE2_NAV;
    zdt_zone2.actions[1].x=350;zdt_zone2.actions[1].y=1200;zdt_zone2.actions[2].kind=ZONE2_TURN;
    zdt_nav_state=ZDT_NAV_WAIT_START_QUERY;zdt_nav_segment_index=0;zdt_nav_segment_count=internal?2:1;
    zdt_nav_segments[0].dx=-1;zdt_nav_segments[1].dy=-1;distances[0]=distances[1]=800;
    goal_x=350;goal_y=1200;test_yaw=90;test_gyro=0;zdt_nav_move_sign=1;zdt_nav_drive_lateral=zdt_odo_owner=0;
    zdt_nav_start_mask=15;test_stopped=test_imu=test_enabled=1;test_busy=test_demand=0;field_hal=HAL_OK;
    zdt_outer_enabled=1;zdt_outer_radius=NAV_CORNER_DEFAULT_RADIUS_MM;zdt_outer_speed=NAV_CORNER_DEFAULT_SPEED_MM_S;
    zdt_outer_started_count=zdt_outer_fallback_count=queries=field_calls=failures=arrivals=0;
    zdt_mileage_query_active=0;zdt_nav_wait_start_query=zdt_nav_wait_end_query=0;last_failure=NULL;
}
static void endpoint(void)
{
    zdt_outer.x=zdt_outer.end_x;zdt_outer.y=zdt_outer.end_y;
    zdt_outer.progress=zdt_outer.total_mm;zdt_outer.rate_x=zdt_outer.rate_y=0;
    test_yaw=atan2f(zdt_outer.wy,zdt_outer.wx)*57.29578f-90.0f;test_gyro=0;zdt_outer_sample_ready=1;
}
void Test_NavCornerOwner(void)
{
    unsigned internal;uint16_t x,y;
    for(internal=0;internal<2;internal++) {
      reset((uint8_t)internal);CHECK(ZDT_Outer_TryStart());
      CHECK(zdt_nav_state==ZDT_NAV_OUTER_CORNER && zdt_outer_cross_action==!internal && zdt_outer_started_count==1);
      CHECK(fabsf(zdt_outer.in_mm-654.05f)<0.01f && fabsf(zdt_outer.out_mm-654.05f)<0.01f);
      CHECK(zdt_outer.radius==165.0f && fabsf(zdt_outer.arc_mm-259.1814f)<0.01f);
      CHECK(fabsf(zdt_outer.end_x-330.95f)<0.01f && fabsf(zdt_outer.end_y-1219.05f)<0.01f);
      CHECK(!arrivals && !field_calls && zdt_zone2.index==0);
      now+=20;samples++;ZDT_Outer_Service();CHECK(field_calls==1 && last_x<0 && !arrivals && queries==1);
      /* New wheel feedback updates actual map pose, never jumps to endpoint. */
      zdt_mileage_query_active=0;zdt_mileage_query_position[0]=zdt_mileage_query_position[2]=512;
      zdt_mileage_query_position[1]=zdt_mileage_query_position[3]=-512;
      now+=40;zdt_mileage_query_sample_tick=now;ZDT_Outer_OnQuery(15);GridNav_GetPoseMm(&x,&y);CHECK(x==1148 && y==400 && !arrivals);
      endpoint();now+=20;samples++;zdt_outer_sample_tick=now;ZDT_Outer_Service();CHECK(!arrivals);
      endpoint();now+=180;samples++;zdt_outer_sample_tick=now;ZDT_Outer_Service();
      CHECK(arrivals==1 && zdt_nav_state==ZDT_NAV_IDLE && zdt_zone2.pending);
      CHECK(zdt_zone2.index==(internal?1U:2U));
      {uint32_t before=field_calls;ZDT_Outer_Service();ZDT_Outer_OnQuery(15);CHECK(field_calls==before && arrivals==1);}
    }
    reset(0);zdt_outer_enabled=0;CHECK(!ZDT_Outer_TryStart() && !field_calls && !arrivals);
    reset(0);zdt_odo_owner=1;CHECK(!ZDT_Outer_TryStart());
    reset(0);zdt_zone2.actions[1].kind=ZONE2_TURN;CHECK(!ZDT_Outer_TryStart());
    reset(0);zdt_nav_start_mask=7;CHECK(!ZDT_Outer_TryStart());
    /* Uploaded first NE bend: true pose2123,1148, A* legs1000/850,
     * precise goal1250,2100. Old8mm lane gate silently rejected48mm bias. */
    reset(1);GridNav_SetPoseMm(2123,1148);zdt_nav_segments[0].dx=0;zdt_nav_segments[0].dy=-1;
    zdt_nav_segments[1].dx=-1;zdt_nav_segments[1].dy=0;
    distances[0]=1000;distances[1]=850;goal_x=1250;goal_y=2100;test_yaw=0.32f;
    CHECK(ZDT_Outer_TryStart() && zdt_outer.corner_x==2123 && fabsf(zdt_outer.corner_y-2123.61f)<0.01f);
    CHECK(fabsf(zdt_outer.end_x-1228.76f)<0.01f && fabsf(zdt_outer.end_y-2123.61f)<0.01f && zdt_outer_cross_action==0);
    CHECK(fabsf(zdt_outer.in_mm-810.61f)<0.01f && fabsf(zdt_outer.out_mm-729.24f)<0.01f);
    CHECK(!arrivals && zdt_zone2.index==0 && zdt_nav_state==ZDT_NAV_OUTER_CORNER);
    reset(1);GridNav_SetPoseMm(2123,1148);zdt_nav_segments[0].dx=0;zdt_nav_segments[0].dy=-1;
    zdt_nav_segments[1].dx=-1;zdt_nav_segments[1].dy=0;
    distances[0]=1000;distances[1]=850;goal_x=1250;goal_y=2050;test_yaw=0.32f;
    CHECK(!ZDT_Outer_TryStart() && !arrivals); /*98mm is not cell-centre rounding.*/
    /* RAW2 lane join: the existing NW waypoint is350,2000, next station
     * lane is2100. Curve there instead of rounding into a diagonal shortcut. */
    reset(0);GridNav_SetPoseMm(350,1280);zdt_nav_segments[0].dx=0;zdt_nav_segments[0].dy=-1;
    distances[0]=720;test_yaw=0;zdt_zone2_action.x=350;zdt_zone2_action.y=2000;
    zdt_zone2.actions[1].x=1250;zdt_zone2.actions[1].y=2100;
    CHECK(ZDT_Outer_TryStart() && zdt_outer.corner_x==350 && fabsf(zdt_outer.corner_y-2119.65f)<0.01f &&
      fabsf(zdt_outer.end_x-1272.05f)<0.01f && fabsf(zdt_outer.end_y-2119.65f)<0.01f && zdt_zone2.index==0);
    /* Uploaded V132 run: completed20/30mm strafes leave the previous
     * drive_lateral flag set. The NEXT straight has a fresh stopped wheel
     * baseline. Exercise the real recenter planner and cross-action merge. */
    reset(0);GridNav_SetPoseMm(1190,380);
    CHECK(GridNav_PlanRecenterTargetMmHeading(350,400,180));
    CHECK(GridNav_GetSegments(zdt_nav_segments,4,&zdt_nav_segment_count));
    distances[0]=PlannedDistance(0);distances[1]=PlannedDistance(1);
    CHECK(distances[0]==20 && distances[1]==840 && zdt_nav_segment_count==2);
    zdt_nav_segment_index=1;GridNav_SetPoseMm(1190,398);zdt_nav_drive_lateral=1;
    CHECK(ZDT_Outer_TryStart() && !zdt_nav_drive_lateral && zdt_outer_cross_action==1);
    CHECK(fabsf(zdt_outer.corner_x-329.75f)<0.01f && zdt_outer.corner_y==400 && !arrivals);
    reset(0);GridNav_SetPoseMm(380,1280);test_yaw=0;
    CHECK(GridNav_PlanRecenterTargetMmHeading(350,2000,90));
    CHECK(GridNav_GetSegments(zdt_nav_segments,4,&zdt_nav_segment_count));
    distances[0]=PlannedDistance(0);distances[1]=PlannedDistance(1);
    CHECK(distances[0]==30 && distances[1]==720);
    zdt_zone2_action.x=350;zdt_zone2_action.y=2000;
    zdt_zone2.actions[1].x=1250;zdt_zone2.actions[1].y=2100;
    zdt_nav_segment_index=1;GridNav_SetPoseMm(352,1280);zdt_nav_drive_lateral=-1;
    CHECK(ZDT_Outer_TryStart() && !zdt_nav_drive_lateral && zdt_outer_cross_action==1);
    CHECK(zdt_outer.corner_x==350 && fabsf(zdt_outer.corner_y-2119.65f)<0.01f && !arrivals);
    reset(0);zdt_nav_drive_lateral=1;test_demand=1;CHECK(!ZDT_Outer_TryStart());
    reset(0);zdt_nav_drive_lateral=-1;test_stopped=0;CHECK(!ZDT_Outer_TryStart());
    /* Return encodes its east-lane corner as NAV/TURN/NAV. The intermediate
     * facing is consumed only if its world heading matches the exit line. */
    reset(0);GridNav_SetPoseMm(380,1200);test_yaw=-90;
    zdt_nav_segments[0].dx=1;zdt_nav_segments[0].dy=0;distances[0]=1745;
    zdt_zone2.count=4;zdt_zone2_action.x=2125;zdt_zone2_action.y=1200;
    zdt_zone2.actions[1].kind=ZONE2_TURN;zdt_zone2.actions[1].yaw=-90;
    zdt_zone2.actions[2].kind=ZONE2_NAV;zdt_zone2.actions[2].x=2125;zdt_zone2.actions[2].y=350;
    zdt_zone2.actions[3].kind=ZONE2_TURN;zdt_zone2.actions[3].yaw=-90;
    CHECK(ZDT_Outer_TryStart() && zdt_outer_cross_action==2 && !arrivals && zdt_zone2.index==0);
    CHECK(fabsf(zdt_outer.corner_x-2172.4f)<0.01f && zdt_outer.corner_y==1200);
    endpoint();now+=20;samples++;zdt_outer_sample_tick=now;ZDT_Outer_Service();CHECK(!arrivals);
    endpoint();now+=180;samples++;zdt_outer_sample_tick=now;ZDT_Outer_Service();
    CHECK(arrivals==1 && zdt_zone2.index==3 && zdt_zone2.pending && zdt_zone2.running);
    CHECK(zdt_zone2.actions[zdt_zone2.index].kind==ZONE2_TURN);
    /* A different facing is an intentional task action, not a transport
     * turn to skip; nor may a terminal NAV omit its final facing. */
    reset(0);zdt_zone2.count=3;zdt_zone2.actions[1].kind=ZONE2_TURN;
    zdt_zone2.actions[1].yaw=180;zdt_zone2.actions[2].kind=ZONE2_NAV;
    zdt_zone2.actions[2].x=350;zdt_zone2.actions[2].y=1200;
    CHECK(!ZDT_Outer_TryStart() && !arrivals && zdt_zone2.index==0);
    reset(0);zdt_zone2.count=2;zdt_zone2.actions[1].kind=ZONE2_TURN;
    CHECK(!ZDT_Outer_TryStart() && !arrivals);
    reset(0);test_demand=1;CHECK(!ZDT_Outer_TryStart());
    reset(0);GridNav_SetObstacle(GridNav_MmToCellX(500),GridNav_MmToCellY(400),1);
    CHECK(!ZDT_Outer_TryStart() && zdt_outer_fallback_count==1 && zdt_nav_state==ZDT_NAV_WAIT_START_QUERY);
    reset(0);CHECK(ZDT_Outer_TryStart());now+=500;samples++;ZDT_Outer_Service();
    CHECK(failures==1 && !arrivals && zdt_zone2.index==0 && strstr(last_failure,"feedback"));
    reset(0);CHECK(ZDT_Outer_TryStart());ZDT_Outer_OnQuery(7);CHECK(failures==1 && !arrivals);
    reset(0);CHECK(ZDT_Outer_TryStart());now+=200;zdt_mileage_query_sample_tick=1000;
    ZDT_Outer_OnQuery(15);CHECK(failures==1 && strstr(last_failure,"sample stale") && !arrivals);
    reset(0);CHECK(ZDT_Outer_TryStart());now+=20;samples++;field_hal=HAL_ERROR;ZDT_Outer_Service();
    CHECK(failures==1 && !arrivals && strstr(last_failure,"velocity"));
    reset(0);CHECK(ZDT_Outer_TryStart());now+=200;samples++;ZDT_Outer_Service();
    CHECK(!failures && last_x==0 && last_y==0 && last_o==0); /* stale150ms brakes,400ms faults */
    reset(0);CHECK(ZDT_Outer_Command("outerarc off") && !zdt_outer_enabled);
    CHECK(ZDT_Outer_Command("outerarc on") && zdt_outer_enabled);
    CHECK(ZDT_Outer_Command("outerarc radius 120") && zdt_outer_radius==120);
    CHECK(ZDT_Outer_Command("outerarc speed 110") && zdt_outer_speed==110);
    CHECK(ZDT_Outer_Command("outerarc speed 306.8352") && fabsf(zdt_outer_speed-NAV_CORNER_DEFAULT_SPEED_MM_S)<0.001f);
    CHECK(ZDT_Outer_Command("outerarc speed 361") && fabsf(zdt_outer_speed-NAV_CORNER_DEFAULT_SPEED_MM_S)<0.001f);
    CHECK(ZDT_Outer_Command("outerarc speed 110") && zdt_outer_speed==110);
    CHECK(ZDT_Outer_Command("outerarc radius nan") && zdt_outer_radius==120);
    CHECK(ZDT_Outer_Command("outerarc speed 999") && zdt_outer_speed==110);
    test_busy=1;CHECK(ZDT_Outer_Command("outerarc off") && zdt_outer_enabled);
    CHECK(ZDT_Outer_Command("outerarc status"));CHECK(!ZDT_Outer_Command("outerarcx"));
}
