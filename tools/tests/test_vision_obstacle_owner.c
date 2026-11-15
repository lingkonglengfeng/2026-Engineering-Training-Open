#include "vision_obstacle.h"
#include "vision.h"
#include "nav_odometry.h"
#include "grid_nav.h"
#include "zone2_route.h"
#include "nav_feedback.h"
#include "nav_frame.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t checks,failures,first_failure;
volatile uint32_t debug_phase,debug_target,debug_recenter,debug_started,debug_goal_x,debug_goal_y;
volatile uint32_t debug_map_a,debug_map_b,debug_map_c;
#define CHECK(c) do{checks++;if(!(c)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<0.05f)
#define ZDT_MILEAGE_MM_PER_REV 100.0f
#define ZDT_NAV_ALIGN_TOLERANCE_DEG 3.0f
#define ZDT_DEPARTURE_LANE_TOLERANCE_MM 5
#define NAV_SHORT_MOVE_MAX_MM 150.0f
typedef struct {float heading_deg,gyro_dps;} chassis_telemetry_t;
typedef struct {float speed_scale,counts_per_rev;uint8_t position_mode;} test_params_t;
enum {ZDT_NAV_IDLE,ZDT_NAV_DRIVE,ZDT_NAV_WAIT_START_QUERY,ZDT_NAV_VOBS_STOP,ZDT_NAV_VOBS_HOLD};
enum {ZDT_SCRIPT_DIR_FWD,ZDT_SCRIPT_DIR_BACK,ZDT_SCRIPT_DIR_LEFT,ZDT_SCRIPT_DIR_RIGHT,ZDT_SCRIPT_DIR_TURN180};
typedef struct {uint8_t dir;uint16_t mm;int16_t map_dx,map_dy;} zdt_script_step_t;
static uint32_t now,stops,nav_starts,queries,nav_failures;
static uint8_t busy,arm_active,round_active,turn_active;
static float heading;
static test_params_t parameters;
static vision_obstacle_t zdt_vobs;
static uint8_t zdt_vobs_phase,zdt_vobs_wait,zdt_vobs_state_dirty;
static uint8_t zdt_vobs_auto_started,zdt_vobs_preparing,zdt_vobs_last_eligible=255;
static uint8_t zdt_vobs_resume_script,zdt_vobs_rebase_script,zdt_vobs_leave_target,zdt_vobs_depart_detoured,zdt_vobs_nav_active;
static uint8_t zdt_vobs_recenter_active;
static uint32_t zdt_vobs_arm_tick,zdt_vobs_wireless_observation;
static vision_pc_obstacle_t fake_wireless;
static uint32_t fake_wireless_age;
static uint8_t fake_wireless_seen;
static uint16_t zdt_vobs_leave_step_x,zdt_vobs_leave_step_y,zdt_vobs_leave_exit_x,zdt_vobs_leave_exit_y;
static uint32_t zdt_vobs_state_tick,zdt_vobs_poll_tick,zdt_vobs_next_query;
static uint32_t zdt_vobs_stable_since,zdt_vobs_sample_tick;
static uint16_t zdt_vobs_goal_x,zdt_vobs_goal_y,zdt_vobs_object_x,zdt_vobs_object_y;
static float zdt_vobs_start_x,zdt_vobs_start_y,zdt_vobs_world_yaw;
static float zdt_vobs_counts,zdt_vobs_mm_per_rev;
static int32_t zdt_vobs_baseline[4],zdt_vobs_previous[4];
static double zdt_vobs_signs[4];
static uint8_t zdt_vobs_history_count;
static uint32_t zdt_vobs_history_tick[8];
static float zdt_vobs_history_x[8],zdt_vobs_history_y[8];
static uint8_t zdt_nav_state,zdt_odo_owner,zdt_odo_base_pending,zdt_nav_odometry_enabled;
static uint8_t zdt_script_running,zdt_arm_camera_token,zdt_qr_active;
static uint8_t zdt_qr_enabled,zdt_script_index,zdt_script_count,zdt_script_settling;
static uint32_t zdt_script_deadline;
static int16_t zdt_script_dx_total,zdt_script_dy_total;
static zdt_script_step_t zdt_script_queue[8];
static uint8_t zdt_mileage_query_active,zdt_mileage_query_report,zdt_mileage_query_attempts;
static uint8_t zdt_nav_wait_start_query,zdt_nav_wait_end_query,zdt_nav_queued_recenter,zdt_nav_post_align;
static grid_nav_segment_t zdt_nav_segments[GRID_NAV_MAX_SEGMENTS];
static uint16_t zdt_nav_segment_count,zdt_nav_segment_index;
static uint8_t zdt_nav_recenter_route,zdt_nav_correction_count,zdt_nav_settle_valid,zdt_nav_settle_announced;
static uint8_t zdt_nav_start_ready,zdt_nav_end_ready,zdt_nav_query_failed;
static uint8_t zdt_nav_start_mask,zdt_nav_end_mask;
static int8_t zdt_nav_move_sign,zdt_nav_drive_lateral;
static uint8_t zdt_nav_align_done,test_reverse_started;
static float zdt_nav_lateral_hold_heading,test_reverse_scale;
static float zdt_nav_segment_target_rev,zdt_nav_measured_rev;
static int8_t zdt_nav_nose_x,zdt_nav_nose_y,zdt_odo_forward,zdt_odo_right;
static float zdt_odo_heading;
static uint32_t zdt_mileage_query_sample_tick;
static int32_t zdt_mileage_query_position[4];
static double zdt_mileage_forward_signs[4];
static nav_odometry_t zdt_odo;
static zone2_route_t zdt_zone2;
static match_control_t zdt_match;
enum {Z2_NONE,Z2_NAV,Z2_TURN};
enum {MATCH_IO_NONE,MATCH_IO_ZONE2,MATCH_IO_LEAVE_SCRIPT};
static uint8_t zdt_zone2_phase,zdt_match_io;
static uint32_t zdt_zone2_action_id,match_completions;
static zone2_action_t zdt_zone2_action;
static const char *zdt_zone2_reason;
static uint32_t zdt_zone2_imu_count,zdt_zone2_imu_tick,zdt_zone2_tick;
static char last_log[200];
uint32_t HAL_GetTick(void){return now;}
uint8_t Vision_GetPcObstacle(vision_pc_obstacle_t *o,uint32_t *age)
{if(!fake_wireless_seen)return 0;*o=fake_wireless;*age=fake_wireless_age;return 1;}
void Vision_ClearPcObstacle(void){fake_wireless_seen=0;}
static test_params_t *Nav_Params(void){return &parameters;}
static float Nav_Params_StraightDistance(float mm){return mm*1.03f;}
static void ChassisMotor_GetTelemetry(chassis_telemetry_t *t){t->heading_deg=heading;t->gyro_dps=0;}
static uint8_t ChassisMotor_IsTurnActive(void){return turn_active;}
static uint8_t ArmMaterial_IsActive(void){return arm_active;}
static uint8_t ArmRound1_IsActive(void){return round_active;}
static uint8_t HWT101_IsZeroDriftReady(void){return 1U;}
static uint32_t HWT101_GetSampleCount(void){return now;}
static float NavBlend_AngleError(float a,float b){return remainderf(a-b,360.0f);}
static float ZDT_Nav_GetHeading(void){return heading;}
static uint8_t ZDT_Align_DiagBusy(void){return busy;}
static void ZDT_Nav_RefreshCostModel(void){}
static void ZDT_Nav_Start(void);
static void ZDT_Nav_NotifyArrived(void);
static void ZDT_Mileage_StartQuery(uint8_t report){(void)report;queries++;zdt_mileage_query_active=1U;}
static void ZDT_Odo_Cancel(void){zdt_odo_owner=0;zdt_odo.state=NAV_ODO_IDLE;}
static void ZDT_Nav_Cancel(void){ZDT_Odo_Cancel();zdt_nav_state=ZDT_NAV_IDLE;zdt_nav_wait_start_query=zdt_nav_wait_end_query=0;}
static uint8_t ChassisMotor_IsEnabled(void){return 1;}
static int ChassisMotor_TurnRelative(int16_t deg){(void)deg;return HAL_OK;}
static int ChassisMotor_TurnRelativeFast(int16_t deg){(void)deg;return HAL_OK;}
static uint8_t ZDT_Zone2_WheelsStopped(void){return 1;}
static float ZDT_Zone2_RelYaw(float yaw){return yaw-atan2f(-zdt_nav_nose_y,zdt_nav_nose_x)*57.29578f;}
static int ChassisMotor_StopImmediate(void){stops++;return 0;}
uint8_t MatchControl_IsActive(const match_control_t *m){return m->state==MATCH_MOVING || m->state==MATCH_LEAVING;}
void ZDT_Console_Print(const char *s){strncpy(last_log,s,sizeof(last_log)-1);last_log[sizeof(last_log)-1]=0;}
static void ZDT_Match_Complete(uint8_t ok,const char *why){(void)ok;(void)why;match_completions++;}
static void ZDT_Vobs_State(void);
static uint8_t ZDT_Vobs_Command(const char *command,uint32_t received_tick);
#include "vobs_owner_under_test.inc"
#include "vobs_arrival_under_test.inc"
#include "vobs_routes_under_test.inc"
static void ZDT_Nav_Start(void){nav_starts++;ZDT_Nav_StartBody();}
static float ZDT_Nav_MatchExtraMm(void){return 0.0f;}
static void ZDT_Nav_BeginStraightDrive(float distance_rev,int8_t sign,float scale)
{
  if(scale<=0.0f)scale=parameters.speed_scale;
#include "vobs_recenter_cap_under_test.inc"
  test_reverse_started=(uint8_t)(sign<0);test_reverse_scale=scale;
}
static uint8_t dispatch_reverse(void)
{
  const grid_nav_segment_t *segment=&zdt_nav_segments[zdt_nav_segment_index];
  test_reverse_started=0;
  switch(0) {case 0:
#include "vobs_reverse_under_test.inc"
    break;
  }
  return test_reverse_started;
}
static void ZDT_Nav_Fail(const char *reason){(void)reason;nav_failures++;}
static void final_check(float nominal,float actual)
{
  zdt_nav_segment_target_rev=nominal/ZDT_MILEAGE_MM_PER_REV;
  zdt_nav_measured_rev=actual/ZDT_MILEAGE_MM_PER_REV;
  zdt_nav_start_mask=zdt_nav_end_mask=15;
#include "vobs_final_check_under_test.inc"
}
static nav_motion_feedback_t match_check(float planned_mm,float measured_mm)
{
#include "vobs_match_check_under_test.inc"
  return check;
}

static void drive(uint16_t base_y,int32_t ticks)
{
  uint8_t i;
  GridNav_SetPoseMm(2125,base_y);
  zdt_nav_state=ZDT_NAV_DRIVE;zdt_odo_owner=1;zdt_odo_base_pending=0;
  zdt_odo.state=NAV_ODO_RUN;zdt_odo.last_sample=now;zdt_odo_forward=1;zdt_odo_right=0;
  for(i=0;i<4;i++){zdt_odo.baseline[i]=0;zdt_odo.previous[i]=ticks;}
  zdt_vobs_history_count=2;
  zdt_vobs_history_tick[0]=now-200;zdt_vobs_history_tick[1]=now;
  zdt_vobs_history_x[0]=zdt_vobs_history_x[1]=2125;
  zdt_vobs_history_y[1]=base_y+ticks*.1f;zdt_vobs_history_y[0]=zdt_vobs_history_y[1]-100;
}
static void setup(void)
{
  GridNav_Init();memset(&zdt_odo,0,sizeof(zdt_odo));memset(&zdt_vobs,0,sizeof(zdt_vobs));
  memset(&zdt_zone2,0,sizeof(zdt_zone2));memset(&zdt_match,0,sizeof(zdt_match));
  now=1000;stops=nav_starts=queries=nav_failures=0;busy=arm_active=round_active=turn_active=0;fake_wireless_seen=0;
  zdt_vobs_phase=zdt_vobs_wait=zdt_mileage_query_active=0;
  zdt_vobs_auto_started=zdt_vobs_preparing=zdt_vobs_resume_script=zdt_vobs_rebase_script=0;
  zdt_vobs_leave_target=zdt_vobs_depart_detoured=zdt_vobs_nav_active=0;
  zdt_vobs_recenter_active=0;
  zdt_nav_queued_recenter=zdt_nav_post_align=0;
  zdt_script_index=zdt_script_count=zdt_script_settling=0;zdt_script_deadline=0;
  zdt_script_dx_total=zdt_script_dy_total=0;zdt_vobs_last_eligible=255;
  zdt_qr_enabled=1;
  zdt_vobs_poll_tick=now;heading=zdt_odo_heading=0;
  parameters.speed_scale=.65f;parameters.counts_per_rev=1000;
  zdt_nav_odometry_enabled=1;zdt_script_running=zdt_arm_camera_token=zdt_qr_active=0;
  zdt_nav_nose_x=0;zdt_nav_nose_y=-1;
  for(uint8_t i=0;i<4;i++)zdt_mileage_forward_signs[i]=1;
  zdt_match.state=MATCH_MOVING;zdt_match.entered_tick=now;zdt_match.run_started_tick=50;
  zdt_zone2.running=1;zdt_zone2.active_id=77;zdt_zone2.count=2;zdt_zone2.started=now;
  zdt_zone2_phase=Z2_NAV;zdt_zone2_action_id=77;zdt_match_io=MATCH_IO_ZONE2;match_completions=0;
  GridNav_SetPoseMm(2125,500);CHECK(GridNav_PlanStraightTargetMm(2125,1200));
  CHECK(ZDT_Vobs_Command("vobs on",now));CHECK(zdt_vobs.armed); /* Current .65 speed retained. */
  NEAR(parameters.speed_scale,.65f);
  drive(500,1000);
}
static void report(uint32_t seq,int gap,int right,uint32_t age,uint32_t received)
{
  char line[100];snprintf(line,sizeof(line),"vobs %lu %lu %d %d %lu",
    (unsigned long)zdt_vobs.session,(unsigned long)seq,gap,right,(unsigned long)age);
  CHECK(ZDT_Vobs_Command(line,received));
}
static void sample(uint32_t time,int32_t ticks,uint8_t mask)
{
  now=time;ZDT_Vobs_Service();CHECK(zdt_vobs_wait);
  for(uint8_t i=0;i<4;i++)zdt_mileage_query_position[i]=ticks;
  zdt_mileage_query_sample_tick=time;CHECK(ZDT_Vobs_OnMileage(mask));
}
static void setup_leave(void)
{
  setup();GridNav_Init();GridNav_SetPoseMm(2250,150);
  zdt_match.state=MATCH_LEAVING;zdt_match_io=MATCH_IO_LEAVE_SCRIPT;
  zdt_script_running=1;zdt_script_count=2;zdt_script_index=0;
  zdt_script_queue[0]=(zdt_script_step_t){ZDT_SCRIPT_DIR_FWD,300,0,300};
  zdt_script_queue[1]=(zdt_script_step_t){ZDT_SCRIPT_DIR_LEFT,125,-125,0};
  zdt_odo_owner=2;zdt_odo_forward=1;zdt_odo_right=0;zdt_odo.target_mm=309;
  zdt_odo.state=NAV_ODO_RUN;zdt_odo.last_sample=now;
  for(uint8_t i=0;i<4;i++){zdt_odo.baseline[i]=0;zdt_odo.previous[i]=1000;}
  zdt_vobs_history_count=2;zdt_vobs_history_tick[0]=now-100;zdt_vobs_history_tick[1]=now;
  zdt_vobs_history_x[0]=zdt_vobs_history_x[1]=2250;
  zdt_vobs_history_y[0]=200;zdt_vobs_history_y[1]=250;
  zdt_qr_active=zdt_arm_camera_token=1;
}
static void arrive_checked_plan(void)
{
  grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];uint16_t count=0,i,x,y,gx,gy;
  float px,py;
  GridNav_GetPoseMm(&x,&y);GridNav_GetTargetMm(&gx,&gy);px=x;py=y;
  CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&count));
  for(i=0;i<count;i++) {
    float distance=ZDT_Vobs_OdoTarget(1U,GridNav_GetSegmentDistanceMm(i));
    px+=segments[i].dx*distance;py-=segments[i].dy*distance;
  }
  NEAR(px,gx);NEAR(py,gy);
  GridNav_SetPoseMm((uint16_t)lroundf(px),(uint16_t)lroundf(py));
  zdt_nav_state=ZDT_NAV_IDLE;zdt_odo_owner=0;ZDT_Nav_NotifyArrived();
}
int main(void)
{
  uint16_t x,y;float px,py;uint32_t session;
  {
    uint32_t disabled,armed;
    now=100;memset(&zdt_vobs,0,sizeof(zdt_vobs));
    zdt_vobs_preparing=zdt_vobs_auto_started=0;
    ZDT_Vobs_MatchState(MATCH_PREPARING);disabled=zdt_vobs.session;
    CHECK(!zdt_vobs.armed && disabled && strstr(last_log,"armed=0"));
    ZDT_Vobs_MatchState(MATCH_PREPARING);CHECK(zdt_vobs.session==disabled);
    now=200;ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);armed=zdt_vobs.session;
    CHECK(zdt_vobs.armed && armed!=disabled && strstr(last_log,"armed=1"));
    ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);ZDT_Vobs_MatchState(MATCH_LEAVING);
    ZDT_Vobs_MatchState(MATCH_MOVING);CHECK(zdt_vobs.session==armed && zdt_vobs.armed);
    CHECK(ZDT_Vobs_Command("vobs off",now));disabled=zdt_vobs.session;
    ZDT_Vobs_MatchState(MATCH_LEAVING);CHECK(!zdt_vobs.armed && zdt_vobs.session==disabled);
    ZDT_Vobs_MatchState(MATCH_FINISHED);CHECK(!zdt_vobs.armed && !zdt_vobs_auto_started);
    ZDT_Vobs_MatchState(MATCH_LEAVING);CHECK(zdt_vobs.armed && zdt_vobs.session!=disabled);
    ZDT_Vobs_MatchState(MATCH_FAULT);CHECK(!zdt_vobs.armed && strstr(last_log,"armed=0"));
  }
  setup();CHECK(ZDT_Vobs_Eligible());
  zdt_arm_camera_token=zdt_qr_active=1;CHECK(ZDT_Vobs_Eligible());
  zdt_arm_camera_token=zdt_qr_active=0;
  zdt_vobs_last_eligible=0;zdt_vobs_state_dirty=0;now=1001;ZDT_Vobs_Service();
  CHECK(zdt_vobs_last_eligible==1 && strstr(last_log,"eligible=1"));
  zdt_odo_right=1;now=1002;ZDT_Vobs_Service();CHECK(zdt_vobs_last_eligible==0 && strstr(last_log,"eligible=0"));
  zdt_odo_right=0;now=1000;
  CHECK(ZDT_Vobs_LivePose(&px,&py));NEAR(px,2125);NEAR(py,600);
  zdt_odo_right=1;CHECK(!ZDT_Vobs_Eligible());zdt_odo_right=0;
  zdt_odo_forward=-1;CHECK(!ZDT_Vobs_Eligible());zdt_odo_forward=1;
  turn_active=1;CHECK(!ZDT_Vobs_Eligible());turn_active=0;
  arm_active=1;CHECK(!ZDT_Vobs_Eligible());arm_active=0;
  report(1,200,-106,20,now-190);CHECK(stops==0);CHECK(strstr(last_log,"STALE")!=0);
  report(1,200,-106,0,now);CHECK(stops==1 && zdt_vobs_phase==1);
  CHECK(zdt_zone2.active_id==77 && zdt_zone2.index==0 && zdt_zone2.running);
  CHECK(zdt_match.state==MATCH_MOVING && nav_starts==0);
  CHECK(zdt_vobs_object_x==2019 && zdt_vobs_object_y==965);
  report(1,200,-106,0,now);CHECK(stops==1);CHECK(strstr(last_log,"SEQUENCE")!=0);
  sample(1040,1200,15);CHECK(nav_starts==0);
  sample(1080,1200,15);CHECK(nav_starts==0);
  sample(1120,1200,15);CHECK(nav_starts==0);
  sample(1160,1200,15); /* New stable encoder samples, not synthetic applied_rpm. */
  GridNav_GetPoseMm(&x,&y);CHECK(x==2125 && y==620);
  CHECK(zdt_vobs_phase==0 && nav_starts==1);
  CHECK(zdt_zone2.active_id==77 && zdt_zone2.index==0 && zdt_zone2.running);
  CHECK(zdt_match.state==MATCH_MOVING && zdt_match.run_started_tick==50);
  drive(620,0);report(2,180,-106,0,now);CHECK(stops==1);CHECK(strstr(last_log,"KNOWN")!=0);
  {
    static grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];uint16_t n=0,j;int32_t mx=2125,my=620;
    CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&n));
    for(j=0;j<n;j++) {uint16_t d=GridNav_GetSegmentDistanceMm(j);mx+=segments[j].dx*d;my-=segments[j].dy*d;}
    CHECK(mx==2125 && my==1200);CHECK(zdt_zone2.active_id==77 && zdt_zone2.index==0);
    GridNav_SetPoseMm((uint16_t)mx,(uint16_t)my);zdt_nav_state=ZDT_NAV_IDLE;
    ZDT_Nav_NotifyArrived(); /* Real production callback only after the planned endpoint. */
    CHECK(zdt_zone2.index==1 && zdt_zone2.pending && !zdt_zone2.active_id);
    CHECK(zdt_zone2_phase==Z2_NONE && !zdt_zone2_action_id && !match_completions);
    Zone2Route_Result(&zdt_zone2,77,1);CHECK(zdt_zone2.index==1); /* Late duplicate id ignored. */
  }
  session=zdt_vobs.session;ZDT_Vobs_Reset();CHECK(!zdt_vobs.armed && !zdt_vobs_phase && session!=zdt_vobs.session);
  CHECK(ZDT_Vobs_Command("vobs retry",now));CHECK(nav_starts==1);

  setup();GridNav_AddObstacleDiscMm(2125,1200,50);report(1,200,-106,0,now);
  sample(1040,1200,15);sample(1080,1200,15);sample(1120,1200,15);sample(1160,1200,15);
  CHECK(zdt_vobs_phase==2 && nav_starts==0 && zdt_nav_state==ZDT_NAV_VOBS_HOLD);
  CHECK(zdt_zone2.active_id==77 && zdt_zone2.index==0 && zdt_match.state==MATCH_MOVING);
  now=1660;ZDT_Vobs_Service();CHECK(zdt_match.entered_tick==1660 && zdt_zone2.started==1660);
  CHECK(GridNav_ProcessCommand("clear"));CHECK(ZDT_Vobs_Command("vobs retry",now));
  CHECK(!zdt_vobs_phase && nav_starts==1);
  CHECK(zdt_zone2.active_id==77 && zdt_zone2.index==0);
  setup();report(1,200,-106,100,now);
  CHECK(zdt_vobs_phase==1 && zdt_vobs_object_y==915); /* 50mm earlier at measured 500mm/s. */
  setup();zdt_vobs_history_count=0;
  zdt_odo.last_sample=900;ZDT_Vobs_RecordOdo();
  zdt_odo.last_sample=950;ZDT_Vobs_RecordOdo();ZDT_Vobs_RecordOdo();
  CHECK(zdt_vobs_history_count==2); /* Duplicate wheel timestamps cannot forge velocity. */
  setup();zdt_nav_nose_x=1;zdt_nav_nose_y=0;GridNav_SetPoseMm(1935,1200);
  zdt_odo.previous[0]=zdt_odo.previous[1]=zdt_odo.previous[2]=zdt_odo.previous[3]=998;
  zdt_vobs_history_x[0]=1935;zdt_vobs_history_x[1]=2034.8f;
  zdt_vobs_history_y[0]=zdt_vobs_history_y[1]=1200;
  report(1,200,0,0,now);CHECK(stops==0);CHECK(strstr(last_log,"OUTSIDE_MAP")!=0);
  {
    int8_t f,r;
    ZDT_Vobs_MapAxes(90,-125,0,&f,&r);CHECK(f==0 && r==-1);
    ZDT_Vobs_MapAxes(0,-125,0,&f,&r);CHECK(f==-1 && r==0);
    ZDT_Vobs_MapAxes(180,-125,0,&f,&r);CHECK(f==1 && r==0);
    ZDT_Vobs_MapAxes(-90,-125,0,&f,&r);CHECK(f==0 && r==1);
    zdt_vobs_nav_active=0;NEAR(ZDT_Vobs_OdoTarget(1,100),103);
    zdt_vobs_nav_active=1;NEAR(ZDT_Vobs_OdoTarget(1,100),100);
    NEAR(ZDT_Vobs_OdoTarget(2,100),103); /* Normal script still x1.03. */
  }
  /* The normal departure starts on a cell boundary (x=2250): the physical
   * straight step is free even though that cell's representative centre is not. */
  setup_leave();CHECK(ZDT_Vobs_Eligible());
  report(1,300,-100,0,now);CHECK(zdt_vobs_resume_script && zdt_vobs_depart_detoured && stops==1);
  CHECK(zdt_vobs_leave_step_x==2250 && zdt_vobs_leave_step_y==459);
  CHECK(zdt_vobs_leave_exit_x==2121 && zdt_vobs_leave_exit_y==459);
  sample(1040,1200,15);sample(1080,1200,15);sample(1120,1200,15);sample(1160,1200,15);
  GridNav_GetPoseMm(&x,&y);CHECK(x==2250 && y==270);
  CHECK(!zdt_vobs_phase && zdt_vobs_resume_script && zdt_vobs_leave_target==0 && nav_starts==1);
  CHECK(zdt_script_index==0 && zdt_match.state==MATCH_LEAVING && !match_completions);
  GridNav_SetPoseMm(2250,459);ZDT_Nav_NotifyArrived();
  CHECK(!zdt_vobs_resume_script && zdt_script_index==1 && zdt_script_running && zdt_vobs_rebase_script);
  CHECK(!match_completions && zdt_script_dx_total==0 && zdt_script_dy_total==0);
  /* A blocked first waypoint can be bypassed to the actual exit endpoint. */
  setup_leave();CHECK(GridNav_AddObstacleDiscMm(2380,550,50)>0);
  report(1,300,-100,0,now);
  sample(1040,1200,15);sample(1080,1200,15);sample(1120,1200,15);sample(1160,1200,15);
  GridNav_GetTargetMm(&x,&y);debug_phase=zdt_vobs_phase;debug_target=zdt_vobs_leave_target;
  debug_recenter=zdt_vobs_recenter_active;debug_started=nav_starts;debug_goal_x=x;debug_goal_y=y;
  if(debug_phase==2) {
    GridNav_SetPoseMm(2225,270);debug_map_a=GridNav_RotatedBodyFreeMm(2121,459,90);
    debug_map_b=GridNav_PlanTargetMm(2121,459);
    debug_map_c=GridNav_PlanCheckedTargetMmHeading(2121,459,90);
    GridNav_SetPoseMm(2250,270);
  }
  CHECK(!zdt_vobs_phase && zdt_vobs_resume_script && zdt_vobs_leave_target==1 && !zdt_vobs_recenter_active);
  CHECK(nav_starts==1);
  GridNav_GetTargetMm(&x,&y);CHECK(x==2121 && y==459);
  {
    static grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];uint16_t n=0,j;float mx=2250,my=270;
    CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&n));
    for(j=0;j<n;j++) {float d=ZDT_Vobs_OdoTarget(1,GridNav_GetSegmentDistanceMm(j));
      mx+=segments[j].dx*d;my-=segments[j].dy*d;}
    NEAR(mx,2121);NEAR(my,459); /* Actual encoder targets, no duplicate 1.03. */
  }
  GridNav_SetPoseMm(2121,459);ZDT_Nav_NotifyArrived();
  CHECK(zdt_script_index==zdt_script_count && !zdt_vobs_resume_script && !match_completions);
  setup_leave();GridNav_Init();GridNav_SetPoseMm(2250,270);zdt_vobs_resume_script=1;
  CHECK(GridNav_AddObstacleDiscMm(2330,800,50)>0);
  CHECK(!ZDT_Vobs_PlanTarget(1200,1150));
  CHECK(ZDT_Vobs_PlanDepartureEntry(1200,1150));
  GridNav_GetTargetMm(&x,&y);CHECK(x==2175 && y==270 && zdt_vobs_recenter_active);
  CHECK(!zdt_nav_queued_recenter); /* Preflight recenter flag does not leak to the entry. */
  /* A turn at this boundary clips the field, but the recenter planner checks
   * west 125mm while facing east, followed by the north leg. Execute that
   * actual first BACK without silently substituting a 180-degree turn. */
  setup();GridNav_Init();GridNav_SetPoseMm(2250,400);
  zdt_nav_nose_x=1;zdt_nav_nose_y=0;heading=0;
  zdt_vobs_goal_x=2125;zdt_vobs_goal_y=1150;
  CHECK(!GridNav_RotatedBodyFreeMm(2250,400,45));
  ZDT_Vobs_Replan();CHECK(nav_starts==1 && zdt_nav_recenter_route && zdt_vobs_nav_active);
  CHECK(zdt_nav_segment_count==2 && zdt_nav_segments[0].dx==-1 && !zdt_nav_segments[0].dy);
  CHECK(GridNav_GetSegmentDistanceMm(0)==125);
  zdt_nav_align_done=0;CHECK(dispatch_reverse());CHECK(zdt_nav_move_sign==-1);
  NEAR(zdt_nav_lateral_hold_heading,0);NEAR(test_reverse_scale,.65f);
  NEAR(ZDT_Vobs_OdoTarget(1,125),125);
  zdt_nav_segment_index=1;zdt_nav_align_done=0;
  zdt_nav_segments[1].dx=-1;zdt_nav_segments[1].dy=0;zdt_nav_segment_target_rev=1;
  CHECK(!dispatch_reverse()); /* Only the planner's first leg permits BACK. */
  zdt_vobs_nav_active=0;CHECK(dispatch_reverse());NEAR(test_reverse_scale,.10f);
  CHECK(ZDT_Vobs_PlanTarget(2250,450));CHECK(!zdt_nav_queued_recenter);
  /* New wireless frame: camera age + MCU queue age, one observation consumed
   * only once; an observation captured before arm never starts a detour. */
  setup();zdt_vobs_arm_tick=950;fake_wireless=(vision_pc_obstacle_t){7,990,300,0,-100};
  fake_wireless_age=60;fake_wireless_seen=1;ZDT_Vobs_Service();CHECK(!stops);
  fake_wireless.observation_id=8;fake_wireless_age=20;ZDT_Vobs_Service();
  CHECK(stops==1 && zdt_vobs.last_sequence==1 && zdt_vobs_phase==1);
  ZDT_Vobs_Service();CHECK(stops==1);
  /* Production final-segment checks use the same exact/compensated target
   * as ODO_Start, including a long leg that would fail at an extra 3%. */
  setup();zdt_vobs_nav_active=1;
  final_check(2000,2000);CHECK(!nav_failures);
  CHECK(match_check(2000,2000)==NAV_MOTION_OK);
  zdt_vobs_nav_active=0;final_check(2000,2060);CHECK(!nav_failures);
  CHECK(match_check(2000,2060)==NAV_MOTION_OK);
  final_check(2000,2000);CHECK(nav_failures==1);
  CHECK(match_check(2000,2000)!=NAV_MOTION_OK);

  /* Use the real NAV_Start zero-leg branch, not a manually invoked callback:
   * old STOP/HOLD flags cannot leak into the next normal action. */
  setup();GridNav_SetPoseMm(2125,1200);
  CHECK(GridNav_PlanCheckedTargetMmHeading(2125,1200,90));
  zdt_nav_state=ZDT_NAV_VOBS_HOLD;zdt_nav_wait_start_query=zdt_nav_wait_end_query=1;
  ZDT_Nav_Start();CHECK(zdt_nav_state==ZDT_NAV_IDLE && !zdt_odo_owner);
  CHECK(!zdt_nav_wait_start_query && !zdt_nav_wait_end_query);
  CHECK(zdt_zone2.index==1 && !zdt_zone2_action_id && !match_completions);
  setup_leave();GridNav_SetPoseMm(2125,500);zdt_vobs_resume_script=1;zdt_vobs_leave_target=1;
  CHECK(GridNav_PlanCheckedTargetMmHeading(2125,500,90));
  zdt_nav_state=ZDT_NAV_VOBS_STOP;ZDT_Nav_Start();
  CHECK(zdt_nav_state==ZDT_NAV_IDLE && !zdt_vobs_resume_script);
  CHECK(zdt_script_index==zdt_script_count && !match_completions);

  /* Complete the actual departure fallback, then start the subsequent QR
   * action through the real Zone2 callback with the obstacle still mapped. */
  setup_leave();CHECK(GridNav_AddObstacleDiscMm(2380,550,50)>0);
  report(1,300,-100,0,now);
  sample(1040,1200,15);sample(1080,1200,15);sample(1120,1200,15);sample(1160,1200,15);
  arrive_checked_plan();CHECK(!zdt_vobs_resume_script && zdt_script_index==2);
  GridNav_GetPoseMm(&x,&y);CHECK(x==2121 && y==459);
  zdt_script_running=0;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_QR;zdt_match.zone=1;
  zdt_match_io=MATCH_IO_ZONE2;zdt_zone2.index=0;zdt_zone2.active_id=99;zdt_zone2_action_id=0;
  zdt_nav_post_align=1;
  {
    zone2_action_t qr_action={0};qr_action.kind=ZONE2_NAV;qr_action.x=2125;qr_action.y=1150;
    qr_action.name="qr";
    CHECK(ZDT_Zone2_StartAction(&qr_action,99));
    CHECK(zdt_vobs_nav_active && !zdt_vobs_phase && zdt_zone2_action_id==99);
    CHECK(zdt_vobs_recenter_active && zdt_zone2.index==0);
    arrive_checked_plan(); /* Short lateral entry is not QR arrival. */
    CHECK(!zdt_vobs_recenter_active && zdt_vobs_nav_active && zdt_zone2_action_id==99);
    CHECK(zdt_zone2.index==0 && !match_completions);
    arrive_checked_plan();CHECK(zdt_zone2.index==1 && !zdt_zone2_action_id);
    CHECK(!match_completions && !nav_failures);
    /* The next goal itself is physically blocked: retain its action as the
     * already-defined NO_PATH wait rather than returning a route failure. */
    CHECK(GridNav_AddObstacleDiscMm(2125,1150,50)>0);
    zdt_zone2.index=0;zdt_zone2.active_id=100;
    CHECK(ZDT_Zone2_StartAction(&qr_action,100));
    CHECK(zdt_vobs_phase==2 && zdt_nav_state==ZDT_NAV_VOBS_HOLD);
    CHECK(zdt_zone2_action_id==100 && !match_completions && !nav_failures);
  }
  /* A changed chassis heading can turn the remaining world-left script leg
   * into a physical forward leg. It has the same front-camera coverage. */
  setup_leave();GridNav_SetPoseMm(2250,459);heading=zdt_odo_heading=90;
  zdt_script_index=1;zdt_vobs_rebase_script=1;zdt_odo.target_mm=128.75f;
  for(uint8_t i=0;i<4;i++)zdt_odo.previous[i]=300;
  zdt_vobs_history_x[0]=2245;zdt_vobs_history_x[1]=2220;
  zdt_vobs_history_y[0]=zdt_vobs_history_y[1]=459;
  CHECK(ZDT_Vobs_IsLeaveOdo() && ZDT_Vobs_Eligible());
  zdt_odo_right=-1;CHECK(!ZDT_Vobs_IsLeaveOdo());zdt_odo_right=0;
  zdt_odo_forward=-1;CHECK(!ZDT_Vobs_IsLeaveOdo());zdt_odo_forward=1;
  report(1,300,-100,0,now);CHECK(stops==1 && zdt_vobs_resume_script);
  CHECK(zdt_vobs_leave_step_x==2121 && zdt_vobs_leave_step_y==459);
  sample(1040,400,15);sample(1080,400,15);sample(1120,400,15);sample(1160,400,15);
  CHECK(!zdt_vobs_phase && zdt_vobs_resume_script);
  arrive_checked_plan();CHECK(zdt_script_index==2 && !zdt_vobs_resume_script && !match_completions);
  return failures;
}
