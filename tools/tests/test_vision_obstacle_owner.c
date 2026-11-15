#include "vision_obstacle.h"
#include "station_map.h"
#include "vision.h"
#include "nav_odometry.h"
#include "grid_nav.h"
#include "zone2_route.h"
#include "nav_feedback.h"
#include "nav_frame.h"
#include "nav_corner.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t checks,failures,first_failure;
volatile uint32_t failed_lines[16];
volatile uint32_t failed_state[16][8];
static void failure_snapshot(void);
volatile uint32_t debug_phase,debug_target,debug_recenter,debug_started,debug_goal_x,debug_goal_y;
volatile uint32_t debug_map_a,debug_map_b,debug_map_c;
#define CHECK(c) do{checks++;if(!(c)){if(failures<16){failed_lines[failures]=__LINE__;failure_snapshot();}failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<0.05f)
#define COMPETITION_VARIANT "PRELIM"
#define ZDT_MILEAGE_MM_PER_REV 100.0f
#define CHASSIS_MAX_WHEEL_RPM 250.0f
#define ZDT_NAV_ALIGN_TOLERANCE_DEG 3.0f
#define ZDT_NAV_HEADING_MAX_DEG 12.0f
#define ZDT_DEPARTURE_LANE_TOLERANCE_MM 5
#define NAV_SHORT_MOVE_MAX_MM 150.0f
typedef struct {float heading_deg,gyro_dps;} chassis_telemetry_t;
typedef struct {float speed_scale,counts_per_rev;uint8_t position_mode;} test_params_t;
enum {ZDT_NAV_IDLE,ZDT_NAV_DRIVE,ZDT_NAV_WAIT_START_QUERY,ZDT_NAV_VOBS_STOP,ZDT_NAV_VOBS_HOLD,ZDT_NAV_OUTER_CORNER};
enum {ZDT_SCRIPT_DIR_FWD,ZDT_SCRIPT_DIR_BACK,ZDT_SCRIPT_DIR_LEFT,ZDT_SCRIPT_DIR_RIGHT,ZDT_SCRIPT_DIR_TURN180};
typedef struct {uint8_t dir;uint16_t mm;int16_t map_dx,map_dy;} zdt_script_step_t;
static uint32_t now,stops,nav_starts,queries,nav_failures;
static int16_t brake_rpm;
static uint8_t busy,arm_active,round_active,turn_active;
static float heading,gyro;
static test_params_t parameters;
static vision_obstacle_t zdt_vobs;
static uint8_t zdt_vobs_enabled=1,zdt_outer_enabled=1;
static uint8_t zdt_vobs_phase,zdt_vobs_wait,zdt_vobs_state_dirty;
static uint8_t zdt_vobs_auto_started,zdt_vobs_preparing,zdt_vobs_last_eligible=255;
static uint8_t zdt_vobs_resume_script,zdt_vobs_rebase_script,zdt_vobs_leave_target,zdt_vobs_depart_detoured,zdt_vobs_nav_active;
static uint8_t zdt_vobs_recenter_active;
static uint8_t zdt_vobs_retreat_active;
static float zdt_vobs_retreat_heading;
static uint8_t zdt_vobs_watch_valid,zdt_nav_rolling_handoff;
static uint8_t zdt_vobs_watch_site,zdt_vobs_object_site,test_snap;
static vision_obstacle_object_t zdt_vobs_objects[VOBS_MAX_OBJECTS];
static uint8_t zdt_vobs_watch_object,zdt_vobs_object_id,zdt_vobs_merged;
static uint8_t zdt_mileage_query_mask;
static uint8_t has_site(uint8_t site){for(uint8_t i=0;i<VOBS_MAX_OBJECTS;i++)if(zdt_vobs_objects[i].valid && zdt_vobs_objects[i].site==site)return 1;return 0;}
static uint8_t zdt_vobs_outer_pause,zdt_vobs_outer_cross_action,zdt_outer_cross_action;
static uint8_t zdt_outer_chain_valid;
static nav_corner_t zdt_outer_chain_next;
static nav_corner_t zdt_outer;
static uint32_t zdt_outer_sample_tick;
static int32_t zdt_outer_position[4];
static struct{uint32_t since;uint8_t active;}zdt_turn_band[2];
static int32_t zdt_nav_start_position[4];
static float zdt_drive_handoff_heading;
static uint32_t zdt_vobs_watch_tick;
static float zdt_vobs_watch_x,zdt_vobs_watch_y;
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
static uint32_t zdt_vobs_history_tick[VOBS_POSE_HISTORY_COUNT];
static float zdt_vobs_history_x[VOBS_POSE_HISTORY_COUNT],zdt_vobs_history_y[VOBS_POSE_HISTORY_COUNT];
static float zdt_vobs_history_yaw[VOBS_POSE_HISTORY_COUNT];
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
static uint8_t zdt_nav_align_done,test_reverse_started,zdt_nav_keep_heading;
static uint8_t zdt_departure_transition;
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
static void failure_snapshot(void)
{
  uint16_t x,y;GridNav_GetTargetMm(&x,&y);
  failed_state[failures][0]=zdt_vobs_phase;failed_state[failures][1]=zdt_vobs_recenter_active;
  failed_state[failures][2]=nav_starts;failed_state[failures][3]=zdt_zone2.index;
  failed_state[failures][4]=zdt_zone2_action.final_point;failed_state[failures][5]=x;
  failed_state[failures][6]=y;failed_state[failures][7]=zdt_zone2.active_id;
}
uint32_t HAL_GetTick(void){return now;}
uint8_t Vision_GetPcObstacle(vision_pc_obstacle_t *o,uint32_t *age)
{if(!fake_wireless_seen)return 0;*o=fake_wireless;*age=fake_wireless_age;return 1;}
void Vision_ClearPcObstacle(void){fake_wireless_seen=0;}
static test_params_t *Nav_Params(void){return &parameters;}
typedef struct {float decel_rpm_s;} test_motion_t;
static const test_motion_t test_motion={360.0f};
static const test_motion_t *Nav_Motion(void){return &test_motion;}
static float Nav_Params_StraightDistance(float mm){return mm*1.03f;}
static void ChassisMotor_GetTelemetry(chassis_telemetry_t *t){t->heading_deg=heading;t->gyro_dps=gyro;}
static uint8_t ChassisMotor_IsTurnActive(void){return turn_active;}
static uint8_t ArmMaterial_IsActive(void){return arm_active;}
static uint8_t ArmRound1_IsActive(void){return round_active;}
static uint8_t HWT101_IsZeroDriftReady(void){return 1U;}
static uint32_t HWT101_GetSampleCount(void){return now;}
static float NavBlend_AngleError(float a,float b){return remainderf(a-b,360.0f);}
static float ZDT_Nav_GetHeading(void){return heading;}
static uint8_t ZDT_Align_DiagBusy(void){return busy;}
static void ZDT_Nav_RefreshCostModel(void){GridNav_SetInitialHeading(heading+atan2f(-zdt_nav_nose_y,zdt_nav_nose_x)*57.29578f);}
static void ZDT_Nav_Start(void);
static void ZDT_Nav_NotifyArrived(void);
static void ZDT_Mileage_StartQuery(uint8_t report){(void)report;queries++;zdt_mileage_query_active=1U;}
static void ZDT_Odo_Cancel(void){zdt_odo_owner=0;zdt_odo.state=NAV_ODO_IDLE;}
static void ZDT_Nav_Cancel(void){ZDT_Odo_Cancel();zdt_nav_state=ZDT_NAV_IDLE;zdt_nav_wait_start_query=zdt_nav_wait_end_query=0;}
static uint8_t ChassisMotor_IsEnabled(void){return 1;}
static int ChassisMotor_TurnToHeading(float target,uint8_t fast){(void)target;(void)fast;return HAL_OK;}
static uint8_t ZDT_TurnBaselineReady(void){return 0;}
static void ZDT_Nav_ResumeRolling(void){}
static uint8_t ZDT_Zone2_WheelsStopped(void){return 1;}
static float ZDT_Zone2_RelYaw(float yaw){return yaw-atan2f(-zdt_nav_nose_y,zdt_nav_nose_x)*57.29578f;}
static int ChassisMotor_Stop(void){stops++;return 0;}
static void ChassisMotor_GetWheelRpm(int16_t rpm[4]){for(unsigned i=0;i<4;i++)rpm[i]=brake_rpm;}
uint8_t MatchControl_IsActive(const match_control_t *m){return m->state==MATCH_MOVING || m->state==MATCH_LEAVING;}
void ZDT_Console_Print(const char *s){strncpy(last_log,s,sizeof(last_log)-1);last_log[sizeof(last_log)-1]=0;}
static void ZDT_Match_Complete(uint8_t ok,const char *why){(void)ok;(void)why;match_completions++;}
static void ZDT_Vobs_State(void);
static float ZDT_Vobs_SpeedLimit(float scale);
static uint8_t ZDT_Vobs_Command(const char *command,uint32_t received_tick);
/* Legacy fixtures exercise the retained raw-observation fallback; dedicated
 * site fixtures below run the real matching policy on the logged failure. */
static uint8_t MatchSite(float ox,float oy,float cx,float cy,float yaw)
{return test_snap?VisionObstacle_MatchSite(ox,oy,cx,cy,yaw):0U;}
#define VisionObstacle_MatchSite MatchSite
#include "vobs_owner_under_test.inc"
#undef VisionObstacle_MatchSite
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
#define ZDT_NAV_STRAFE_MAX_CELLS 3U
#include "vobs_short_translation_under_test.inc"
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
  zdt_odo.target_mm=700;zdt_odo.progress_mm=ticks*.1f;zdt_odo.max_scale=.65f;
  zdt_nav_segment_index=0;CHECK(GridNav_GetSegments(zdt_nav_segments,GRID_NAV_MAX_SEGMENTS,&zdt_nav_segment_count));
  for(i=0;i<4;i++){zdt_odo.baseline[i]=0;zdt_odo.previous[i]=ticks;}
  zdt_vobs_history_count=2;
  zdt_vobs_history_tick[0]=now-200;zdt_vobs_history_tick[1]=now;
  zdt_vobs_history_x[0]=zdt_vobs_history_x[1]=2125;
  zdt_vobs_history_y[1]=base_y+ticks*.1f;zdt_vobs_history_y[0]=zdt_vobs_history_y[1]-100;
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=ZDT_Vobs_WorldYaw();
}
static void setup(void)
{
  GridNav_Init();memset(&zdt_odo,0,sizeof(zdt_odo));memset(&zdt_vobs,0,sizeof(zdt_vobs));
  memset(&zdt_zone2,0,sizeof(zdt_zone2));memset(&zdt_match,0,sizeof(zdt_match));
  zdt_outer_chain_valid=0;zdt_vobs_enabled=zdt_outer_enabled=1;
  now=1000;stops=nav_starts=queries=nav_failures=0;brake_rpm=0;busy=arm_active=round_active=turn_active=0;fake_wireless_seen=0;
  zdt_vobs_phase=zdt_vobs_wait=zdt_mileage_query_active=0;
  zdt_vobs_auto_started=zdt_vobs_preparing=zdt_vobs_resume_script=zdt_vobs_rebase_script=0;
  zdt_vobs_leave_target=zdt_vobs_depart_detoured=zdt_vobs_nav_active=0;
  zdt_vobs_recenter_active=zdt_vobs_watch_valid=zdt_nav_rolling_handoff=0;
  zdt_vobs_retreat_active=0;
  zdt_nav_queued_recenter=zdt_nav_post_align=0;
  zdt_script_index=zdt_script_count=zdt_script_settling=0;zdt_script_deadline=0;
  zdt_script_dx_total=zdt_script_dy_total=0;zdt_vobs_last_eligible=255;
  zdt_qr_enabled=1;
  zdt_vobs_poll_tick=now;heading=zdt_odo_heading=gyro=0;
  zdt_vobs_outer_pause=zdt_vobs_outer_cross_action=zdt_outer_cross_action=0;
  test_snap=zdt_vobs_watch_site=zdt_vobs_object_site=0;memset(zdt_vobs_objects,0,sizeof(zdt_vobs_objects));zdt_vobs_watch_object=zdt_vobs_object_id=zdt_vobs_merged=0;
  parameters.speed_scale=.65f;parameters.counts_per_rev=1000;
  zdt_nav_odometry_enabled=1;zdt_script_running=zdt_arm_camera_token=zdt_qr_active=0;
  zdt_nav_nose_x=0;zdt_nav_nose_y=-1;
  for(uint8_t i=0;i<4;i++)zdt_mileage_forward_signs[i]=1;
  zdt_match.qr_required=1;zdt_match.state=MATCH_MOVING;zdt_match.entered_tick=now;zdt_match.run_started_tick=50;
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
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=90;
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
/* Keep a synthetic optional-waypoint itinerary to test V290 fallback, even
 * though production RAW now targets the station directly in both batches. */
static void legacy_optional_waypoint(void)
{
  zdt_zone2.actions[2]=zdt_zone2.actions[1];zdt_zone2.actions[1]=zdt_zone2.actions[0];
  zdt_zone2.actions[0]=(zone2_action_t){ZONE2_NAV,350,2000,0,0,0,0,"test_optional_nw",0};zdt_zone2.count=3;
}
int main(void)
{
  uint16_t x,y;float px,py;uint32_t session;
  {
    uint32_t disabled,armed,revision;
    GridNav_Init();GridNav_SetPoseMm(2124,455);CHECK(GridNav_AddObstacleDiscMm(2090,998,50)>0);
    CHECK(!GridNav_RotatedBodyFreeMm(2125,1150,90));
    now=100;memset(&zdt_vobs,0,sizeof(zdt_vobs));
    zdt_vobs_preparing=zdt_vobs_auto_started=0;
    ZDT_Vobs_MatchState(MATCH_PREPARING);disabled=zdt_vobs.session;
    CHECK(!zdt_vobs.armed && disabled && strstr(last_log,"armed=0"));
    CHECK(!GridNav_HasObstacles() && GridNav_RotatedBodyFreeMm(2125,1150,90));
    GridNav_GetPoseMm(&x,&y);CHECK(x==2124 && y==455);
    CHECK(GridNav_CellType(GridNav_MmToCellX(750),GridNav_MmToCellY(750))==GRID_CELL_BLOCKED);
    CHECK(GridNav_CellType(GridNav_MmToCellX(1200),GridNav_MmToCellY(2350))==GRID_CELL_PAD);
    CHECK(GridNav_AddObstacleDiscMm(300,400,50)>0);revision=GridNav_GetRevision();
    ZDT_Vobs_MatchState(MATCH_PREPARING);CHECK(zdt_vobs.session==disabled);
    CHECK(GridNav_HasObstacles() && GridNav_GetRevision()==revision);
    now=200;ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);armed=zdt_vobs.session;
    CHECK(zdt_vobs.armed && armed!=disabled && strstr(last_log,"armed=1"));
    ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);ZDT_Vobs_MatchState(MATCH_LEAVING);
    ZDT_Vobs_MatchState(MATCH_MOVING);CHECK(zdt_vobs.session==armed && zdt_vobs.armed);
    CHECK(GridNav_HasObstacles());
    CHECK(ZDT_Vobs_Command("vobs off",now));disabled=zdt_vobs.session;
    ZDT_Vobs_MatchState(MATCH_LEAVING);CHECK(!zdt_vobs.armed && zdt_vobs.session==disabled);
    ZDT_Vobs_MatchState(MATCH_FINISHED);CHECK(!zdt_vobs.armed && !zdt_vobs_auto_started);
    ZDT_Vobs_MatchState(MATCH_LEAVING);CHECK(zdt_vobs.armed && zdt_vobs.session!=disabled);
    ZDT_Vobs_MatchState(MATCH_FAULT);CHECK(!zdt_vobs.armed && strstr(last_log,"armed=0"));
    CHECK(GridNav_HasObstacles());ZDT_Vobs_MatchState(MATCH_PREPARING);CHECK(!GridNav_HasObstacles());
  }
  /* Manual obstacles are installed before departure, survive prepare/reset,
   * and are mutually exclusive with camera-owned obstacle observations. */
  setup();zdt_match.state=MATCH_IDLE;zdt_nav_state=ZDT_NAV_IDLE;
  CHECK(ZDT_ManualObstacle_Command("manualobs set 2 2125 1550 1200 762"));
  CHECK(GridNav_ManualObstacleMode() && GridNav_ManualObstacleCount()==2 && !zdt_vobs_enabled && !zdt_vobs.armed && !stops);
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(2125),GridNav_MmToCellY(1550)));
  ZDT_Vobs_MatchState(MATCH_PREPARING);ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);ZDT_Vobs_MatchState(MATCH_LEAVING);
  CHECK(GridNav_ManualObstacleCount()==2 && GridNav_HasObstacles() && !zdt_vobs.armed && !zdt_vobs_enabled);
  GridNav_SetPoseMm(2125,1130);CHECK(GridNav_PlanCheckedTargetMmHeading(1250,2100,0));
  ZDT_Vobs_MatchState(MATCH_FAULT);CHECK(GridNav_ManualObstacleCount()==2);
  CHECK(ZDT_NavSwitch_Command("navswitch set 0 0") && GridNav_ManualObstacleCount()==2);
  CHECK(ZDT_ManualObstacle_Command("manualobs set 3 100 100 200 200 300 300") && GridNav_ManualObstacleCount()==2);
  CHECK(ZDT_ManualObstacle_Command("manualobs set 0 junk") && GridNav_ManualObstacleCount()==2);
  CHECK(ZDT_ManualObstacle_Command("manualobs set 1 2400 100") && GridNav_ManualObstacleCount()==2);
  zdt_match.state=MATCH_MOVING;
  CHECK(ZDT_ManualObstacle_Command("manualobs set 0") && GridNav_ManualObstacleCount()==2 && !stops);
  zdt_match.state=MATCH_IDLE;
  CHECK(ZDT_ManualObstacle_Command("manualobs set 0") && GridNav_ManualObstacleMode() && !GridNav_HasObstacles());
  CHECK(ZDT_ManualObstacle_Command("manualobs set 1 2125 1550") && GridNav_ManualObstacleCount()==1);
  CHECK(ZDT_ManualObstacle_Command("manualobs vision") && !GridNav_ManualObstacleMode() && !GridNav_HasObstacles() && zdt_vobs_enabled);
  CHECK(ZDT_ManualObstacle_Command("manualobs set 1 2125 1550"));
  CHECK(ZDT_NavSwitch_Command("navswitch set 1 1") && !GridNav_ManualObstacleMode() && !GridNav_HasObstacles());
  /* All four independent feature choices survive prepare and a new run. */
  for(uint8_t arc=0;arc<2;arc++)for(uint8_t avoid=0;avoid<2;avoid++) {
    char command[40];setup();zdt_match.state=MATCH_IDLE;zdt_nav_state=ZDT_NAV_IDLE;
    CHECK(GridNav_AddObstacleDiscMm(300,400,50)>0);
    snprintf(command,sizeof(command),"navswitch set %u %u",arc,avoid);
    CHECK(ZDT_NavSwitch_Command(command));CHECK(zdt_outer_enabled==arc && zdt_vobs_enabled==avoid && !stops);
    if(!avoid)CHECK(!GridNav_HasObstacles());
    ZDT_Vobs_MatchState(MATCH_PREPARING);ZDT_Vobs_MatchState(MATCH_WAIT_START_OK);
    CHECK(zdt_vobs.armed==avoid && zdt_outer_enabled==arc && zdt_vobs_enabled==avoid);
    ZDT_Vobs_MatchState(MATCH_FINISHED);CHECK(zdt_vobs_enabled==avoid && zdt_outer_enabled==arc);
    CHECK(ZDT_NavSwitch_Command("navswitch status"));CHECK(strstr(last_log,"[NAV SWITCH]"));
  }
  setup();CHECK(ZDT_NavSwitch_Command("navswitch set 0 0"));
  CHECK(strstr(last_log,"REJECTED") && zdt_outer_enabled && zdt_vobs_enabled && !stops);
  zdt_match.state=MATCH_IDLE;zdt_nav_state=ZDT_NAV_IDLE;
  CHECK(ZDT_NavSwitch_Command("navswitch set 0 2"));CHECK(zdt_outer_enabled && zdt_vobs_enabled && !stops);
  CHECK(ZDT_NavSwitch_Command("navswitch set 0 0 extra"));CHECK(zdt_outer_enabled && zdt_vobs_enabled);
  zdt_vobs_phase=2;CHECK(ZDT_NavSwitch_Command("navswitch set 0 0"));CHECK(zdt_outer_enabled && zdt_vobs_enabled);
  zdt_vobs_phase=0;CHECK(ZDT_NavSwitch_Command("navswitch set 1 0"));
  CHECK(!zdt_vobs.armed && !zdt_vobs_enabled && !stops);
  report(1,200,0,0,now);CHECK(strstr(last_log,"DISARMED") && !stops);
  CHECK(ZDT_Vobs_Command("vobs on",now));CHECK(!zdt_vobs.armed && strstr(last_log,"REJECTED"));
  /* Near the threshold, no1.5RPM crawl: floor15RPM,420mm fast pause.
   * Final map still waits for stationary encoder samples. */
  setup();CHECK(GridNav_PlanStraightTargetMm(2125,2000));drive(500,1000);zdt_odo.target_mm=1500;
  report(1,423,0,0,now);CHECK(!stops && !GridNav_HasObstacles());
  NEAR(ZDT_Vobs_SpeedLimit(.65f)*CHASSIS_MAX_WHEEL_RPM,15.0f);
  NEAR(ZDT_Vobs_SpeedLimit(.01f),.01f); /* Do not accelerate a slower owner. */
  report(2,419,0,0,now);CHECK(stops==1 && zdt_vobs_phase==1 && !GridNav_HasObstacles());
  brake_rpm=15;sample(1020,1000,15);sample(1140,1000,15);
  CHECK(zdt_vobs_phase==1 && !GridNav_HasObstacles());
  brake_rpm=0;
  sample(1180,1000,15);sample(1220,1000,15);sample(1260,1000,15);
  CHECK(GridNav_HasObstacles() && !zdt_vobs_phase && !nav_failures);
  /* Staged observations do not touch the map until~420mm and a stopped
   * sample. Off-path targets never alter the nominal speed or map. */
  setup();CHECK(GridNav_PlanStraightTargetMm(2125,2000));drive(500,1000);zdt_odo.target_mm=1500;
  report(1,900,-100,0,now);CHECK(!stops && !GridNav_HasObstacles() && strstr(last_log,"WATCH"));NEAR(ZDT_Vobs_SpeedLimit(.65f),.65f);
  report(2,700,-100,0,now);CHECK(!stops && !GridNav_HasObstacles());NEAR(ZDT_Vobs_SpeedLimit(.65f),.4875f);
  report(3,600,-100,0,now);NEAR(ZDT_Vobs_SpeedLimit(.65f),.325f);
  report(4,500,-100,0,now);NEAR(ZDT_Vobs_SpeedLimit(.65f),.1625f);
  now+=501;zdt_odo.last_sample=now;NEAR(ZDT_Vobs_SpeedLimit(.65f),.1625f);CHECK(!stops && !nav_failures);
  /* The observed failure: yaw corrections no longer reject484mm updates;
   * image loss and a temporarily stale pose do not cancel an accepted cap. */
  gyro=12;heading=4;CHECK(ZDT_Vobs_Eligible());
  CHECK(ZDT_Vobs_SpeedLimit(.65f)<.20f);
  zdt_odo.last_sample=now-201;CHECK(!ZDT_Vobs_Eligible());CHECK(ZDT_Vobs_SpeedLimit(.65f)<.20f);
  zdt_odo.last_sample=now;heading=gyro=0;
  report(5,1000,0,0,now);CHECK(strstr(last_log,"KEEP_NEARER"));NEAR(ZDT_Vobs_SpeedLimit(.65f),.1625f);
  zdt_odo.target_mm=700; /* Off-path observations cannot erase the closer watch. */
  report(6,1100,0,0,now);CHECK(strstr(last_log,"OFF_PATH"));CHECK(zdt_vobs_watch_valid);
  zdt_odo.target_mm=1500;now+=20;zdt_odo.last_sample=now;
  for(uint8_t wheel=0;wheel<4;wheel++)zdt_odo.previous[wheel]+=1100;
  ZDT_Vobs_Service();CHECK(stops==1 && zdt_vobs_phase==1);
  setup();CHECK(GridNav_PlanStraightTargetMm(2125,2000));drive(500,1000);zdt_odo.target_mm=1500;
  report(1,700,0,0,now);gyro=12;
  report(2,484,0,0,now);CHECK(!stops && strstr(last_log,"SLOW"));
  NEAR(ZDT_Vobs_SpeedLimit(.65f),.1365f);gyro=0;
  setup();zdt_odo.target_mm=200;report(1,400,-100,0,now);CHECK(!stops && !GridNav_HasObstacles() && strstr(last_log,"OFF_PATH"));
  setup();CHECK(GridNav_PlanStraightTargetMm(2125,2000));drive(500,1000);zdt_odo.target_mm=1500;
  report(1,400,-100,0,now);CHECK(stops==1 && zdt_vobs_phase==1 && !GridNav_HasObstacles());
  sample(1040,1200,15);sample(1080,1200,15);sample(1120,1200,15);sample(1160,1200,15);
  CHECK(GridNav_HasObstacles() && zdt_match.state==MATCH_MOVING && !match_completions);
  setup();CHECK(GridNav_PlanStraightTargetMm(2125,2000));drive(500,1000);zdt_odo.target_mm=1500;
  report(1,430,-100,0,now);CHECK(!stops && !GridNav_HasObstacles());
  now+=20;zdt_odo.last_sample=now;
  for(uint8_t wheel=0;wheel<4;wheel++)zdt_odo.previous[wheel]+=200;
  ZDT_Vobs_Service();CHECK(stops==1 && zdt_vobs_phase==1 && !GridNav_HasObstacles());
  /* Terminal station-facing turns include the swept body, not just a centre line. */
  setup();zdt_odo.target_mm=200;zdt_zone2.actions[1].kind=ZONE2_TURN;zdt_zone2.actions[1].yaw=0;
  CHECK(ZDT_Vobs_PathHit(2125,920));
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
  drive(620,0);report(2,180,-106,0,now);CHECK(stops==1);CHECK(strstr(last_log,"MERGED")!=0);
  CHECK(zdt_vobs_objects[0].tick==now);
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
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=0;
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
  setup_leave();zdt_script_count=1;zdt_script_queue[0]=(zdt_script_step_t){5U,325,-125,300};
  zdt_odo.map_mode=1;zdt_odo.map_dx=-25;zdt_odo.map_dy=60;
  zdt_odo.target_mm=334.75f;zdt_odo.progress_mm=65;
  CHECK(ZDT_Vobs_LivePose(&px,&py));NEAR(px,2225);NEAR(py,210);
  CHECK(ZDT_Vobs_Eligible() && ZDT_Vobs_PathHit(2175,330));
  CHECK(!ZDT_Vobs_PathHit(1700,330) && !stops && !match_completions);
  (void)ZDT_Vobs_BeginPause(2175,330);
  CHECK(zdt_vobs_phase==1 && stops==1);NEAR(zdt_vobs_start_x,2225);NEAR(zdt_vobs_start_y,210);
  for(unsigned i=0;i<4;i++)CHECK(zdt_vobs_baseline[i]==1000);
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
  CHECK(dispatch_reverse()); /* Later short checked legs now permit BACK too. */
  zdt_vobs_nav_active=0;zdt_nav_align_done=0;CHECK(dispatch_reverse());NEAR(test_reverse_scale,.10f);
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
  final_check(2000,2000);CHECK(!nav_failures);
  CHECK(match_check(2000,2000)==NAV_MOTION_OK); /* V256 accepts final distance residual. */

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
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=180;
  CHECK(ZDT_Vobs_IsLeaveOdo() && ZDT_Vobs_Eligible());
  zdt_odo_right=-1;CHECK(!ZDT_Vobs_IsLeaveOdo());zdt_odo_right=0;
  zdt_odo_forward=-1;CHECK(!ZDT_Vobs_IsLeaveOdo());zdt_odo_forward=1;
  report(1,300,-100,0,now);
  CHECK(!stops && !zdt_vobs_phase && !zdt_vobs_resume_script && strstr(last_log,"OFF_PATH"));
  CHECK(!GridNav_HasObstacles() && zdt_script_index==1 && !match_completions);
  /* Frame-time yaw, including +/-180 wrap, rather than reception-time yaw. */
  setup();
  zdt_vobs_history_yaw[0]=170;zdt_vobs_history_yaw[1]=-170;
  {float fx,fy,fyaw;CHECK(ZDT_Vobs_FramePose(100,&fx,&fy,&fyaw));NEAR(fyaw,180);}
  zdt_vobs_history_count=0;
  for(uint8_t i=0;i<20;i++) {now=1000+i*20;ZDT_Vobs_RecordPose(now,2125,500+i);}
  {float fx,fy,fyaw;CHECK(zdt_vobs_history_count==16);
   CHECK(ZDT_Vobs_FramePose(200,&fx,&fy,&fyaw));NEAR(fy,509);}
  /* A full straight/arc/straight owner is observable through its changing yaw.
   * Braking integrates its incremental wheel pose, not an old straight baseline. */
  setup();CHECK(NavCorner_Init(&zdt_outer,0,1,2125,500,2125,2125,1236,2125,0,1,165,0));
  zdt_nav_state=ZDT_NAV_OUTER_CORNER;zdt_outer_sample_tick=now;
  memset(zdt_outer_position,0,sizeof(zdt_outer_position));
  zdt_odo_owner=0;zdt_odo.state=NAV_ODO_IDLE;
  zdt_vobs_history_count=0;ZDT_Vobs_RecordPose(now-100,2125,500);ZDT_Vobs_RecordPose(now,2125,500);
  gyro=80;CHECK(ZDT_Vobs_Eligible());gyro=0;
  CHECK(ZDT_Vobs_PathHit(2125,1300));CHECK(ZDT_Vobs_PathHit(1500,2125));
  CHECK(!ZDT_Vobs_PathHit(1000,1000));
  report(1,700,0,0,now);CHECK(!stops && zdt_vobs_watch_valid);
  NEAR(ZDT_Vobs_SpeedLimit(.65f),.4875f);
  zdt_outer_cross_action=2;zdt_zone2.count=3;
  report(2,400,0,0,now);CHECK(stops==1 && zdt_vobs_outer_pause);
  CHECK(zdt_vobs_outer_cross_action==2 && zdt_outer_cross_action==0);
  CHECK(zdt_vobs_goal_x==1236 && zdt_vobs_goal_y==2125);
  sample(1040,100,15);CHECK(zdt_vobs_outer_pause);NEAR(zdt_outer.y,510);
  sample(1080,100,15);sample(1120,100,15);sample(1160,100,15);
  CHECK(!zdt_vobs_outer_pause && GridNav_HasObstacles() && !nav_failures);
  GridNav_GetPoseMm(&x,&y);CHECK(x==2125 && y==510);
  /* A lateral re-entry is not completion of the original compound action. */
  zdt_vobs_recenter_active=1;ZDT_Nav_NotifyArrived();CHECK(zdt_vobs_outer_cross_action==2 && zdt_zone2.index==0);
  zdt_vobs_recenter_active=0;ZDT_Nav_NotifyArrived();CHECK(zdt_zone2.index==3 && !zdt_vobs_outer_cross_action);
  /* QR approach sees the later road: preserve it through QR's facing turn,
   * then stop/replan before the RAW curve starts to drive into that road. */
  setup();test_snap=1;report(1,1016,-42,0,now);
  CHECK(strstr(last_log,"DEFERRED") && has_site(6) && !stops);
  GridNav_SetPoseMm(2125,1162);zdt_nav_state=ZDT_NAV_IDLE;turn_active=1;now=2000;ZDT_Vobs_Service();
  CHECK(has_site(6) && !stops && !GridNav_HasObstacles());turn_active=0;
  CHECK(GridNav_PlanCheckedTargetMmHeading(1250,2100,90));
  CHECK(NavCorner_Init(&zdt_outer,0,1,2125,1162,2125,2115,1236,2115,0,1,165,0));
  memset(zdt_outer_position,0,sizeof(zdt_outer_position));zdt_outer_sample_tick=now;
  zdt_nav_state=ZDT_NAV_OUTER_CORNER;zdt_odo_owner=0;ZDT_Vobs_Service();
  CHECK(stops==1 && zdt_vobs_phase==1 && zdt_vobs_object_site==6);
  CHECK(zdt_vobs_goal_x==1250 && zdt_vobs_goal_y==2100);
  sample(2040,0,15);sample(2080,0,15);sample(2120,0,15);sample(2160,0,15);
  CHECK(GridNav_HasObstacles() && !zdt_vobs_phase && nav_starts==1 && !nav_failures);
  CHECK(has_site(6) && zdt_vobs_objects[0].mapped && zdt_zone2.active_id==77 && !match_completions);
  /* Latest log's lower obstacle: raw Y~580 must plan at P11 Y762, leaving
   * ROUGH's service pose free, while the entire central lane is blocked. */
  setup();test_snap=1;heading=180;
  GridNav_SetPoseMm(1200,2098);CHECK(GridNav_PlanStraightTargetMm(1200,400));
  CHECK(GridNav_GetSegments(zdt_nav_segments,GRID_NAV_MAX_SEGMENTS,&zdt_nav_segment_count));
  zdt_odo.target_mm=1731.96f;zdt_odo.progress_mm=800;
  for(uint8_t wheel=0;wheel<4;wheel++)zdt_odo.previous[wheel]=8000;
  zdt_vobs_history_x[0]=zdt_vobs_history_x[1]=1200;
  zdt_vobs_history_y[0]=1398;zdt_vobs_history_y[1]=1298;
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=270;
  report(1,539,19,0,now);CHECK(stops==1 && zdt_vobs_object_site==11);
  CHECK(zdt_vobs_object_x==1200 && zdt_vobs_object_y==762);
  sample(1040,8000,15);sample(1080,8000,15);sample(1120,8000,15);sample(1160,8000,15);
  CHECK(!zdt_vobs_phase && nav_starts==1 && !nav_failures && zdt_zone2.active_id==77);
  CHECK(GridNav_RotatedBodyFreeMm(1200,400,-90));
  CHECK(!GridNav_RotatedBodyFreeMm(1100,762,-90) && !GridNav_RotatedBodyFreeMm(1300,762,-90));
  CHECK(!GridNav_IsObstacle(GridNav_MmToCellX(1183),GridNav_MmToCellY(580)));
  CHECK(zdt_vobs_object_site==11 && !match_completions);
  ZDT_Vobs_MatchState(MATCH_PREPARING);CHECK(!GridNav_HasObstacles() && !zdt_vobs_watch_valid);
  for(uint8_t i=0;i<VOBS_MAX_OBJECTS;i++)CHECK(!zdt_vobs_objects[i].valid);
  /* The two run004 observations are the same cylinder106mm apart, not two
   * independent barriers. Reassign P08->P11; keep the distant QR-side object. */
  setup();test_snap=1;
  ZDT_Vobs_UpdateSite(6,2125,1650,900);
  ZDT_Vobs_UpdateSite(8,1128,1025,1000);
  ZDT_Vobs_UpdateSite(11,1224,979,1164);
  CHECK(!has_site(8) && has_site(11) && has_site(6));
  CHECK(strstr(last_log,"P08->P11"));
  /* One moving estimate is one record even after an adjacent candidate update.
   * A third distinct observation never creates a third road closure. */
  CHECK(ZDT_Vobs_UpdateSite(8,1128,1025,2000)==2);
  CHECK(has_site(8) && has_site(6) && !has_site(11));
  CHECK(ZDT_Vobs_UpdateSite(13,275,275,4000)==1);
  CHECK(has_site(8) && has_site(13) && !has_site(6));
  /* Adjacent 50mm grid cells merge even after a long capture gap. */
  setup();CHECK(ZDT_Vobs_UpdateSite(8,1199,1199,1000)==1);
  CHECK(ZDT_Vobs_UpdateSite(7,1201,1201,9000)==1 && zdt_vobs_merged);
  CHECK(has_site(7) && !has_site(8));
  /* Once mapped, a new neighbour cannot move the existing planning anchor. */
  zdt_vobs_objects[0].mapped=1;
  CHECK(ZDT_Vobs_UpdateSite(8,1190,1190,10000)==1);
  CHECK(zdt_vobs_objects[0].site==7 && zdt_vobs_objects[0].mapped);
  NEAR(zdt_vobs_objects[0].plan_x,800);NEAR(zdt_vobs_objects[0].plan_y,1200);
  CHECK(ZDT_Vobs_UpdateSite(2,750,2125,11000)==2);
  CHECK(ZDT_Vobs_UpdateSite(13,275,275,12000)==1 && has_site(13) && !has_site(7));
  /* Latest run: 205mm/0.8s P01->P02 must not consume two slots. */
  setup();CHECK(ZDT_Vobs_UpdateSite(1,483,2217,1000)==1);
  CHECK(ZDT_Vobs_UpdateSite(2,683,2173,1800)==1);
  CHECK(has_site(2) && !has_site(1) && !zdt_vobs_objects[1].valid);
  setup();CHECK(ZDT_Vobs_UpdateSite(8,1128,1025,1000)==1);
  CHECK(ZDT_Vobs_UpdateSite(11,1200,750,1100)==2);
  CHECK(has_site(8) && has_site(11));
  /* Command-level third-object replacement includes mapped records, never stops. */
  setup();test_snap=1;ZDT_Vobs_UpdateSite(2,750,2125,0);
  ZDT_Vobs_UpdateSite(8,1200,1200,0);zdt_vobs_objects[0].mapped=1;
  report(1,700,0,0,now);CHECK(!strstr(last_log,"LIMIT2") && !stops);
  CHECK(!has_site(2) && has_site(8) && zdt_vobs_objects[0].valid);
  /* Use the real UART command owner across the boundary, not just the
   * association helper: the second frame must update the active limit. */
  setup();test_snap=1;heading=180;GridNav_SetPoseMm(1200,2098);
  CHECK(GridNav_PlanStraightTargetMm(1200,400));
  CHECK(GridNav_GetSegments(zdt_nav_segments,GRID_NAV_MAX_SEGMENTS,&zdt_nav_segment_count));
  zdt_odo.target_mm=1731.96f;
  zdt_vobs_history_x[0]=zdt_vobs_history_x[1]=1200;
  zdt_vobs_history_y[0]=2098;zdt_vobs_history_y[1]=1998;
  zdt_vobs_history_yaw[0]=zdt_vobs_history_yaw[1]=270;
  report(1,785,0,0,now);CHECK(zdt_vobs_watch_site==8 && !stops);
  now=1200;zdt_odo.last_sample=now;zdt_vobs_history_tick[0]=1000;zdt_vobs_history_tick[1]=now;
  zdt_vobs_history_y[0]=1998;zdt_vobs_history_y[1]=1840;
  for(uint8_t wheel=0;wheel<4;wheel++)zdt_odo.previous[wheel]=2580;
  report(2,695,-65,0,now);CHECK(zdt_vobs_watch_site==11 && !stops);
  CHECK(!has_site(8) && has_site(11) && !strstr(last_log,"KEEP_NEARER"));
  now=1400;zdt_odo.last_sample=now;
  for(uint8_t wheel=0;wheel<4;wheel++)zdt_odo.previous[wheel]=7580;
  ZDT_Vobs_Service();CHECK(stops==1 && zdt_vobs_object_site==11);
  /* Latest run006: a32mm first BACK segment after avoidance must not turn180.
   * Also cover a later short segment and a non-recenter ordinary route. */
  setup();GridNav_ClearRuntimeObstacles();GridNav_SetPoseMm(2123,1255);heading=0;
  zdt_nav_recenter_route=zdt_vobs_nav_active=1;zdt_nav_segment_index=0;
  zdt_nav_segments[0]=(grid_nav_segment_t){.dx=0,.dy=1,.length=20};zdt_nav_segment_target_rev=32/ZDT_MILEAGE_MM_PER_REV;
  zdt_nav_align_done=0;CHECK(dispatch_reverse() && zdt_nav_keep_heading && !stops && !nav_failures);
  zdt_nav_segment_index=1;zdt_nav_segments[1]=zdt_nav_segments[0];zdt_nav_align_done=0;
  CHECK(dispatch_reverse());
  zdt_vobs_nav_active=zdt_nav_recenter_route=0;zdt_nav_align_done=0;CHECK(dispatch_reverse());
  zdt_nav_segment_target_rev=151/ZDT_MILEAGE_MM_PER_REV;zdt_nav_align_done=0;CHECK(!dispatch_reverse());
  zdt_nav_segment_target_rev=25/ZDT_MILEAGE_MM_PER_REV;zdt_nav_segments[1].dx=-1;zdt_nav_segments[1].dy=0;
  CHECK(ZDT_Nav_SegmentStrafeSign(&zdt_nav_segments[1])==-1); /* Actual25mm beats legacy20-cell length. */
  zdt_nav_segment_target_rev=151/ZDT_MILEAGE_MM_PER_REV;CHECK(!ZDT_Nav_SegmentStrafeSign(&zdt_nav_segments[1]));
  zdt_nav_segments[1].dx=0;zdt_nav_segments[1].dy=1;zdt_nav_segment_target_rev=32/ZDT_MILEAGE_MM_PER_REV;
  CHECK(GridNav_AddObstacleDiscMm(2123,1230,50)>0);zdt_nav_align_done=0;CHECK(!dispatch_reverse());
  /* Reproduce the actual narrow-aisle NO_PATH pose/heading with P08 closed.
   * The real reverse dispatcher must run BACK, not attempt an180deg turn. */
  setup();heading=-176;GridNav_SetPoseMm(1219,1762);
  CHECK(GridNav_AddObstacleRectMm(1000,1150,1400,1250)>0);
  CHECK(!GridNav_PlanCheckedTargetMmHeading(1200,400,-86));
  zdt_vobs_goal_x=1200;zdt_vobs_goal_y=400;zdt_vobs_phase=2;zdt_nav_state=ZDT_NAV_VOBS_HOLD;
  ZDT_Odo_Cancel();ZDT_Vobs_Replan();
  CHECK(!zdt_vobs_phase && zdt_vobs_retreat_active && zdt_vobs_recenter_active && nav_starts==1);
  CHECK(zdt_nav_segment_count==1 && zdt_nav_segments[0].dy==-1 && !zdt_nav_segments[0].dx);
  GridNav_GetTargetMm(&x,&y);CHECK(x==1219 && y>1762);
  zdt_nav_align_done=0;CHECK(dispatch_reverse());NEAR(zdt_nav_lateral_hold_heading,-180);
  NEAR(test_reverse_scale,.65f);CHECK(!nav_failures && !match_completions && zdt_zone2.active_id==77);
  heading=-180;arrive_checked_plan(); /* Intermediate escape does not consume the station. */
  CHECK(!zdt_vobs_retreat_active && !zdt_vobs_recenter_active && zdt_zone2.active_id==77 && nav_starts==2);
  GridNav_GetTargetMm(&x,&y);CHECK(x==1200 && y==400);
  arrive_checked_plan();CHECK(!zdt_zone2_action_id && zdt_zone2.index==1 && !match_completions);
  /* A longer escape uses the same executor; existing short BACK limits still
   * apply to normal navigation when the retreat owner is inactive. */
  setup();heading=-180;GridNav_SetPoseMm(1219,1500);
  CHECK(GridNav_AddObstacleRectMm(1000,1150,1400,1250)>0);
  zdt_vobs_goal_x=1200;zdt_vobs_goal_y=400;zdt_vobs_phase=2;ZDT_Odo_Cancel();ZDT_Vobs_Replan();
  CHECK(zdt_vobs_retreat_active && GridNav_GetSegmentDistanceMm(0)>150);
  zdt_nav_align_done=0;CHECK(dispatch_reverse());CHECK(!nav_failures && zdt_zone2.active_id==77);
  ZDT_Vobs_Reset();CHECK(!zdt_vobs_retreat_active);
  /* A known rear barrier prevents retreat through it, preserving NO_PATH. */
  setup();heading=-180;GridNav_SetPoseMm(1200,1600);
  CHECK(GridNav_AddObstacleRectMm(1000,1150,1400,1250)>0);
  CHECK(GridNav_AddObstacleRectMm(1000,1800,1400,1900)>0);
  zdt_vobs_goal_x=1200;zdt_vobs_goal_y=400;zdt_vobs_phase=2;ZDT_Odo_Cancel();ZDT_Vobs_Replan();
  CHECK(zdt_vobs_phase==2 && !zdt_vobs_retreat_active && !nav_starts && !nav_failures && !match_completions);
  /* A queued continuous corner remains visible to future-path association. */
  setup();
  CHECK(NavCorner_Init(&zdt_outer,0,1,2125,500,2125,2125,1400,2125,0,1,148.5,0));
  CHECK(NavCorner_Init(&zdt_outer_chain_next,0,1,1400,2125,350,2125,350,1200,-1,0,148.5,90));
  zdt_nav_state=ZDT_NAV_OUTER_CORNER;zdt_nav_segment_count=3;zdt_outer_cross_action=0;zdt_outer_chain_valid=1;
  CHECK(ZDT_Vobs_PathHit(350,1600));zdt_outer_chain_valid=0;CHECK(!ZDT_Vobs_PathHit(350,1600));
  /* Return directly to zone1 staging, preserving the terminal south facing.
   * Replay the logged P06/P08 closures from TEMP and already-near staging. */
  for(uint8_t near=0;near<2;near++) {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};uint16_t gx,gy,total=0;
    setup();ZDT_Nav_Cancel();GridNav_Init();
    GridNav_SetPoseMm(near?2123:380,near?325:1280);heading=near?-90:90;
    CHECK(GridNav_AddObstacleRectMm(1850,1600,2400,1700)>0);
    CHECK(GridNav_AddObstacleRectMm(1150,1000,1250,1400)>0);
    Zone2Route_Init(&zdt_zone2,&ports);zdt_match.zone=1;zdt_match.checkpoint=MATCH_RETURN;
    CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RETURN,MATCH_RETURN,1));
    CHECK(zdt_zone2.count==2 && zdt_zone2.actions[0].kind==ZONE2_NAV);
    CHECK(zdt_zone2.actions[0].x==2125 && zdt_zone2.actions[0].y==350);
    CHECK(zdt_zone2.actions[1].kind==ZONE2_TURN && zdt_zone2.actions[1].yaw==-90);
    Zone2Route_Service(&zdt_zone2,now);
    /* The new yellow band can require the existing checked lateral entry
     * before the original station route, rather than turning at this pose. */
    if(zdt_vobs_recenter_active)arrive_checked_plan();
    GridNav_GetTargetMm(&gx,&gy);CHECK(gx==2125 && gy==350 && !zdt_vobs_phase && !nav_failures);
    for(uint16_t j=0;j<zdt_nav_segment_count;j++)total+=GridNav_GetSegmentDistanceMm(j);
    if(near)CHECK(total<=50); /* Never go north875mm just to touch QR. */
    else CHECK(total>0);
  }
  {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};
    setup();Zone2Route_Init(&zdt_zone2,&ports);
    CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RETURN,MATCH_RETURN,2));
    CHECK(zdt_zone2.count==2 && zdt_zone2.actions[0].x==2125 && zdt_zone2.actions[0].y==2000);
    CHECK(zdt_zone2.actions[1].kind==ZONE2_TURN && zdt_zone2.actions[1].yaw==90);
  }
  /* QR finished facing east: known north-road P06 must enter the very first
   * RAW plan, without another observation or a turn toward the blocked lane. */
  {
    zone2_action_t raw={ZONE2_NAV,1250,2100,0,0,0,1,"raw_align",0};
    setup();GridNav_SetPoseMm(2125,1130);heading=0;
    zdt_nav_state=ZDT_NAV_IDLE;zdt_odo_owner=0;
    CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,now-5000)==1);
    zdt_vobs_objects[0].observation_yaw=90;
    CHECK(!GridNav_HasObstacles() && !zdt_vobs_objects[0].mapped);
    CHECK(ZDT_Zone2_StartAction(&raw,111));
    CHECK(zdt_vobs_objects[0].mapped && GridNav_IsObstacle(GridNav_MmToCellX(2125),GridNav_MmToCellY(1650)));
    CHECK(nav_starts==1 && !zdt_vobs_phase && !match_completions);
    uint16_t gx,gy;GridNav_GetTargetMm(&gx,&gy);CHECK(gx==1250 && gy==2100);
  }
  /* Commit both records before planning; preserve the original observation
   * yaw at junction sites even after the chassis has faced a different way. */
  setup();zdt_nav_state=ZDT_NAV_IDLE;heading=90;zdt_odo_owner=0;
  CHECK(ZDT_Vobs_UpdateSite(8,1200,1200,now)==1);
  zdt_vobs_objects[0].observation_yaw=0;
  CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,now)==2);
  zdt_vobs_objects[1].observation_yaw=90;
  {uint32_t old_stops=stops,old_queries=queries;
   ZDT_Vobs_CommitBeforePlan(1250,2100);
   CHECK(zdt_vobs_objects[0].mapped && zdt_vobs_objects[1].mapped);
   CHECK(stops==old_stops && queries==old_queries && !match_completions);
   CHECK(GridNav_IsObstacle(GridNav_MmToCellX(1200),GridNav_MmToCellY(1375)));
   CHECK(!GridNav_IsObstacle(GridNav_MmToCellX(1375),GridNav_MmToCellY(1200)));
   ZDT_Vobs_CommitBeforePlan(350,1200);CHECK(stops==old_stops && queries==old_queries);
  }
  setup();zdt_vobs_enabled=0;CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,now)==1);
  ZDT_Vobs_CommitBeforePlan(1250,2100);CHECK(!zdt_vobs_objects[0].mapped && !GridNav_HasObstacles());
  zdt_vobs_enabled=1;zdt_vobs.armed=0;ZDT_Vobs_CommitBeforePlan(1250,2100);
  CHECK(!zdt_vobs_objects[0].mapped && !GridNav_HasObstacles());
  setup();CHECK(ZDT_Vobs_UpdateSite(0,800,1200,now)==1);
  ZDT_Vobs_CommitBeforePlan(350,1200);CHECK(zdt_vobs_objects[0].mapped && GridNav_HasObstacles());
  /* New third object evicts least-recently-observed record AND its road
   * layer. Reobserving an old mapped object makes it the newest record. */
  setup();CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,100)==1);
  CHECK(ZDT_Vobs_UpdateSite(11,1200,762,200)==2);
  zdt_vobs_objects[0].observation_yaw=90;zdt_vobs_objects[1].observation_yaw=90;
  ZDT_Vobs_CommitBeforePlan(1250,2100);
  CHECK(ZDT_Vobs_SiteKnown(6) && ZDT_Vobs_SiteKnown(11));
  CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,300)==1);
  CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,250)==1 && zdt_vobs_objects[0].tick==300);
  CHECK(ZDT_Vobs_UpdateSite(13,275,275,400)==2);
  CHECK(has_site(6) && has_site(13) && !has_site(11));
  CHECK(ZDT_Vobs_SiteKnown(6) && !ZDT_Vobs_SiteKnown(11) && !zdt_vobs_objects[1].mapped);
  ZDT_Vobs_CommitBeforePlan(350,1200);CHECK(ZDT_Vobs_SiteKnown(13));
  setup();CHECK(ZDT_Vobs_UpdateSite(6,2125,1650,UINT32_MAX-10U)==1);
  CHECK(ZDT_Vobs_UpdateSite(11,1200,762,20)==2);
  CHECK(ZDT_Vobs_UpdateSite(13,275,275,40)==1 && has_site(11) && has_site(13));
  /* Replay logged ROUGH exitY335: no northward65mm strafe before westward
   * travel. Same policy for both start zones, both batches and anchor shifts. */
  for(uint8_t zone=1;zone<=2;zone++)for(uint8_t round=0;round<2;round++)for(uint8_t variant=0;variant<3;variant++) {
    const uint16_t exit_y[3]={335,360,390};uint16_t y=exit_y[variant],n=0;
    grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};
    setup();GridNav_SetPoseMm(1190,y);heading=90;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    zdt_nav_post_align=1;zdt_match.checkpoint=round?MATCH_TEMP2:MATCH_TEMP1;
    Zone2Route_Init(&zdt_zone2,&ports);
    CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_TEMP,zdt_match.checkpoint,zone));
    CHECK(zdt_zone2.count==2 && zdt_zone2.actions[0].x==350 && zdt_zone2.actions[0].y==1200 && zdt_zone2.actions[0].final_point);
    CHECK(zdt_zone2.actions[1].kind==ZONE2_TURN && zdt_zone2.actions[1].yaw==90);
    Zone2Route_Service(&zdt_zone2,now);CHECK(nav_starts==1 && !zdt_vobs_phase);
    CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&n) && n>=2);
    /* Centre preference may first join south toward the bottom lane. Never
     * restore the old northward Y400 strafe or change the station target. */
    if(segments[0].dx==0 && segments[0].dy==1) {
      uint16_t d=GridNav_GetSegmentDistanceMm(0);
      CHECK(d<=150U && d<y && GridNav_RotatedBodyFreeMm(1190,(uint16_t)(y-d),180));
    } else CHECK(segments[0].dx==-1 && segments[0].dy==0);
  }
  /* Logged TEMP1 -> RAW2 failure: P02/P04 isolate the optional NW waypoint,
   * while the real RAW station remains reachable through the centre roads. */
  for(uint8_t zone=1;zone<=2;zone++) {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};uint16_t gx,gy;
    setup();GridNav_SetPoseMm(380,1280);heading=-.2f;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    zdt_match.checkpoint=MATCH_RAW2;zdt_nav_post_align=1;
    CHECK(ZDT_Vobs_UpdateSite(2,750,2125,100)==1);CHECK(ZDT_Vobs_UpdateSite(4,275,1700,200)==2);
    zdt_vobs_objects[0].observation_yaw=0;zdt_vobs_objects[1].observation_yaw=90;
    Zone2Route_Init(&zdt_zone2,&ports);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RAW,MATCH_RAW2,zone));legacy_optional_waypoint();
    zdt_zone2.actions[1].x=1247;zdt_zone2.actions[1].y=2078; /* Logged W25-adjusted target. */
    Zone2Route_Service(&zdt_zone2,now);
    {uint8_t entry=zdt_vobs_recenter_active;
     if(entry)arrive_checked_plan();
     CHECK(nav_starts==(uint32_t)(1U+entry) && !zdt_vobs_phase && zdt_zone2.index==1 && zdt_zone2.active_id==1);}
    CHECK(zdt_zone2_action_id==1 && zdt_zone2_action.final_point && !match_completions);
    GridNav_GetTargetMm(&gx,&gy);CHECK(gx==1247 && gy==2078);
    arrive_checked_plan();CHECK(zdt_zone2.index==2 && zdt_zone2.pending && !match_completions);
    Zone2Route_Service(&zdt_zone2,now);CHECK(zdt_zone2_phase==Z2_TURN && zdt_zone2_action.yaw==0);
  }
  for(uint8_t mode=0;mode<3;mode++) {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};
    setup();GridNav_SetPoseMm(380,1280);heading=-.2f;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    CHECK(ZDT_Vobs_UpdateSite(2,750,2125,100)==1);CHECK(ZDT_Vobs_UpdateSite(4,275,1700,200)==2);
    zdt_vobs_objects[1].observation_yaw=90;
    Zone2Route_Init(&zdt_zone2,&ports);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RAW,MATCH_RAW2,1));legacy_optional_waypoint();
    if(mode==0)(void)GridNav_AddObstacleRectMm(1000,1850,1400,2400); /* final also blocked */
    else if(mode==1)zdt_zone2.actions[0].final_point=1; /* mandatory exit may not be skipped */
    else zdt_zone2.actions[1].kind=ZONE2_TURN; /* may not jump past a facing action */
    Zone2Route_Service(&zdt_zone2,now);
    CHECK(!nav_starts && zdt_vobs_phase==2 && zdt_zone2.index==0 && !match_completions);
    CHECK(zdt_vobs_goal_x==350 && zdt_vobs_goal_y==2000 && zdt_zone2.active_id==1);
  }
  {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};
    setup();GridNav_SetPoseMm(380,1280);heading=-.2f;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    CHECK(ZDT_Vobs_UpdateSite(2,750,2125,100)==1); /* NW remains reachable with only P02. */
    Zone2Route_Init(&zdt_zone2,&ports);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RAW,MATCH_RAW2,1));legacy_optional_waypoint();
    Zone2Route_Service(&zdt_zone2,now);CHECK(nav_starts==1 && !zdt_vobs_phase && zdt_zone2.index==0);
  }
  /* All transport-only corners are gone from active itineraries. Preserve
   * RAW clearance, real targets and terminal facing in either start zone. */
  for(uint8_t zone=1;zone<=2;zone++)for(uint8_t round=0;round<2;round++) {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};uint16_t gx,gy;
    setup();GridNav_SetPoseMm(380,1280);heading=-.2f;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    zdt_match.checkpoint=round?MATCH_RAW2:MATCH_RAW1;
    CHECK(ZDT_Vobs_UpdateSite(4,275,1700,now)==1);zdt_vobs_objects[0].observation_yaw=90;
    Zone2Route_Init(&zdt_zone2,&ports);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_RAW,zdt_match.checkpoint,zone));
    CHECK(zdt_zone2.count==2 && zdt_zone2.actions[0].final_point && zdt_zone2.actions[0].x==1250 && zdt_zone2.actions[0].y==2100);
    CHECK(zdt_zone2.actions[1].kind==ZONE2_TURN && zdt_zone2.actions[1].yaw==0);
    Zone2Route_Service(&zdt_zone2,now);CHECK(nav_starts==1 && !zdt_vobs_phase && zdt_zone2.index==0);
    if(zdt_vobs_recenter_active)arrive_checked_plan();
    CHECK(!zdt_vobs_phase && zdt_zone2.index==0 && zdt_zone2_action.final_point);
    GridNav_GetTargetMm(&gx,&gy);CHECK(gx==1250 && gy==2100);
  }
  for(uint8_t zone=1;zone<=2;zone++) {
    const zone2_ports_t ports={ZDT_Zone2_StartAction,0,0};uint16_t gx,gy;
    setup();GridNav_SetPoseMm(1247,2068);heading=-90;zdt_odo_owner=0;zdt_nav_state=ZDT_NAV_IDLE;
    zdt_match.checkpoint=MATCH_ROUGH1;zdt_nav_post_align=1;
    CHECK(ZDT_Vobs_UpdateSite(2,750,2125,now)==1);
    Zone2Route_Init(&zdt_zone2,&ports);CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_ROUGH,MATCH_ROUGH1,zone));
    CHECK(zdt_zone2.count==2 && zdt_zone2.actions[0].x==1200 && zdt_zone2.actions[0].y==380);
    zdt_zone2.actions[0].y=355;Zone2Route_Service(&zdt_zone2,now);
    CHECK(nav_starts==1 && !zdt_vobs_phase && zdt_zone2.index==0);
    GridNav_GetTargetMm(&gx,&gy);CHECK(gx==1200 && gy==355);
    CHECK(zdt_zone2.actions[1].kind==ZONE2_TURN && zdt_zone2.actions[1].yaw==180);
  }
  return failures;
}
