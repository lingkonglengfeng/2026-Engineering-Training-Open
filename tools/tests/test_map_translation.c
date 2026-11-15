/* Production map-translation owner and navigation decision, with IO ports. */
#include "chassis_motor.h"
#include "grid_nav.h"
#include "nav_params.h"
#include "nav_odometry.h"
#include "nav_frame.h"
#include "nav_blend.h"
#include "nav_short_move.h"
#include "station_map.h"
#include "match_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
volatile unsigned test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static nav_odometry_t zdt_odo;
static uint8_t zdt_odo_owner,zdt_odo_wait,zdt_odo_base_pending,zdt_odo_map_mode;
static uint32_t zdt_odo_next_query,zdt_odo_tick,zdt_odo_imu_tick,zdt_odo_imu_count;
static float zdt_odo_mm,zdt_odo_max_scale,zdt_odo_heading,zdt_odo_approach_gain,zdt_odo_map_x,zdt_odo_map_y;
static uint8_t zdt_odo_map_hold_line,zdt_departure_roll,zdt_departure_transition;
static float zdt_departure_tail_mm,zdt_odo_map_line_offset;
static int8_t zdt_odo_forward,zdt_odo_right,zdt_nav_nose_x,zdt_nav_nose_y=-1;
static uint8_t zdt_mileage_query_active,zdt_mileage_query_report,zdt_mileage_query_attempts;
static int32_t zdt_mileage_query_position[4],zdt_nav_start_position[4];
static uint32_t zdt_mileage_query_sample_tick;
static uint8_t zdt_mileage_query_mask;
static const double zdt_mileage_forward_signs[4]={1,-1,1,-1};
static uint8_t zdt_vobs_nav_active,zdt_script_fast_exit,zdt_script_running;
static uint8_t zdt_start_zone,zdt_start_pose_set=1,zdt_script_count;
static uint8_t zdt_script_index,zdt_script_settling,zdt_nav_post_align,zdt_vobs_watch_valid,zdt_vobs_resume_script,zdt_vobs_phase,zdt_qr_enabled=1;
static uint32_t zdt_script_deadline;
static int16_t zdt_script_dx_total,zdt_script_dy_total;
static match_control_t zdt_match;
static grid_nav_segment_t zdt_nav_segments[1];static uint16_t zdt_nav_segment_index;
static uint16_t pose_x=2100,pose_y=512,qr_x=2125,qr_y=1078;
static uint32_t completes,stop_calls;
typedef struct{uint8_t dir;uint16_t mm;int16_t map_dx,map_dy;}zdt_script_step_t;
static zdt_script_step_t queued[2];
#define ZDT_SCRIPT_DIR_MAP 5U
#define ZDT_SCRIPT_DIR_FWD 0U
#define ZDT_SCRIPT_DIR_LEFT 2U
#define ZDT_SCRIPT_DIR_RIGHT 3U
static float zdt_nav_drive_scale,zdt_script_drive_scale,zdt_nav_segment_target_rev;
static float zdt_nav_lateral_hold_heading,zdt_nav_hold_peak_error,zdt_drive_handoff_heading,zdt_vobs_retreat_heading;
static uint8_t zdt_nav_odometry_enabled=1,zdt_nav_recenter_route=1,zdt_nav_state,zdt_nav_align_done;
static uint8_t zdt_nav_keep_heading,zdt_nav_rolling_handoff,zdt_nav_start_mask=15,zdt_nav_query_failed;
static uint8_t zdt_nav_start_ready=1,zdt_nav_correction_count,zdt_nav_arc_armed,zdt_vobs_retreat_active;
static int8_t zdt_nav_move_sign=1,zdt_nav_drive_sign,zdt_nav_drive_lateral;
static uint32_t now=1000,turns,field_calls,failures;
static float yaw=6.5f,field_x,field_y,field_o,extra;
static uint8_t blocked;
enum{ZDT_NAV_WAIT_START_QUERY,ZDT_NAV_DRIVE,ZDT_NAV_ALIGN_TURN};
#define ZDT_NAV_ALIGN_TOLERANCE_DEG 3.0f
#define ZDT_DEPARTURE_LANE_TOLERANCE_MM 5
#define ZDT_TURN_HANDOFF_TOLERANCE_DEG (Nav_Motion()->turn_band_deg)
#define ZDT_NAV_POSITION_SPEED_RPM 50
#define ZDT_NAV_STRAFE_MAX_CELLS 3U
#define ZDT_ODO_HOLD_KP (Nav_Motion()->heading_kp)
#define ZDT_ODO_HOLD_OMEGA_MAX (Nav_Motion()->heading_cap)
static struct{uint8_t active;}zdt_turn_band[1];
uint32_t HAL_GetTick(void){return now;}
static float ZDT_Nav_GetHeading(void){return yaw;}
static uint8_t Ready(void){return 1;}
#define HWT101_IsZeroDriftReady Ready
#define HWT101_GetSampleCount HAL_GetTick
uint8_t ChassisMotor_IsEnabled(void){return 1;}
void ChassisMotor_ResetIdleRamp(void){}
void ChassisMotor_GetWheelRpm(int16_t w[4]){memset(w,0,8);}
void ChassisMotor_GetTelemetry(chassis_telemetry_t *t){memset(t,0,sizeof(*t));t->heading_deg=yaw;}
HAL_StatusTypeDef ChassisMotor_SetVelocity(float f,float r,float o){(void)f;(void)r;(void)o;return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_SetFieldVelocity(float ix,float iy,float x,float y,float o)
{CHECK(ix==0 && iy==-zdt_nav_nose_y);field_calls++;field_x=x;field_y=y;field_o=o;return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_ContinueFieldVelocity(float ix,float iy,float x,float y,float o)
{return ChassisMotor_SetFieldVelocity(ix,iy,x,y,o);}
HAL_StatusTypeDef ChassisMotor_Stop(void){stop_calls++;return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_StopImmediate(void){return HAL_OK;}
HAL_StatusTypeDef ChassisMotor_TurnToHeading(float h,uint8_t fast){(void)h;(void)fast;turns++;return HAL_OK;}
void ZDT_Console_Print(const char *s){CHECK(s && *s);}
static void ZDT_Vobs_RecordOdo(void){}
static uint8_t ZDT_Vobs_IsLeaveOdo(void){return 0;}
static uint8_t zdt_vobs_history_count;
static float ZDT_Vobs_SpeedLimit(float scale){return scale;}
static float ZDT_Vobs_WorldYaw(void){return yaw+atan2f(-zdt_nav_nose_y,zdt_nav_nose_x)*57.29578f;}
static void ZDT_Mileage_StartQuery(uint8_t report){(void)report;zdt_mileage_query_active=1;}
void GridNav_GetPoseMm(uint16_t *x,uint16_t *y){*x=pose_x;*y=pose_y;}
void GridNav_SetPoseMm(uint16_t x,uint16_t y){pose_x=x;pose_y=y;}
uint8_t GridNav_HasObstacles(void){return blocked;}
uint8_t StationMap_TargetMm(uint8_t station,uint16_t *x,uint16_t *y)
{CHECK(station==STATION_MAP_QR);*x=qr_x;*y=qr_y;return 1;}
static uint8_t ZDT_TurnBaselineReady(void)
{return !zdt_mileage_query_active && zdt_mileage_query_mask==15 && now-zdt_mileage_query_sample_tick<=150;}
static float ZDT_Zone2_RelYaw(float angle){return angle-(zdt_nav_nose_y<0?90:-90);}
static void ZDT_Match_Complete(uint8_t success,const char *reason)
{CHECK(success && !reason && !zdt_script_running && !zdt_odo_owner);completes++;zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_QR;}
void MatchControl_Service(match_control_t *flow,uint32_t stamp){CHECK(flow==&zdt_match && stamp==now && zdt_nav_rolling_handoff);}
static uint8_t ZDT_Nav_BeginDepartureRolling(float mm,float scale);
static uint8_t ZDT_Odo_Service(uint8_t owner);
static void ZDT_Zone2_Service(void)
{
  GridNav_GetPoseMm(&pose_x,&pose_y);memcpy(zdt_nav_start_position,zdt_mileage_query_position,sizeof(zdt_nav_start_position));
  zdt_nav_segments[0].dx=0;zdt_nav_segments[0].dy=zdt_start_zone==1?-1:1;
  CHECK(ZDT_Nav_BeginDepartureRolling(fabsf((float)qr_y-pose_y),.75f));
  CHECK(!ZDT_Odo_Service(1));
}
uint8_t GridNav_RotatedBodyFreeMm(uint16_t x,uint16_t y,int16_t h)
{(void)x;(void)y;(void)h;return !blocked;}
static float ZDT_Nav_MatchExtraMm(void){return extra;}
static void ZDT_Nav_Fail(const char *s){(void)s;failures++;}
static uint8_t ZDT_Blend_TryStart(void){return 0;}
static uint8_t ZDT_Outer_TryStart(void){return 0;}
static uint8_t ZDT_Nav_TryArmArc(void){return 0;}
static float ZDT_Nav_SegmentHeadingDeg(const grid_nav_segment_t *s)
{return atan2f(-s->dy,s->dx)*57.29578f-90;}
static void ZDT_Nav_BeginStraightDrive(float mm,int8_t sign,float scale){(void)mm;(void)sign;(void)scale;}
static void ZDT_Nav_BeginLateralDrive(float mm,int8_t sign,float scale){(void)mm;(void)sign;(void)scale;}
static void ZDT_Nav_StartMove(float mm,int8_t sign,uint16_t rpm,float scale){(void)mm;(void)sign;(void)rpm;(void)scale;}
static uint8_t ZDT_Script_RequirePoseNear(uint16_t x,uint16_t y,uint16_t tol,const char *label)
{(void)x;(void)y;(void)tol;(void)label;return 1;}
static void ZDT_Script_Queue(const zdt_script_step_t *s,uint8_t n)
{CHECK(n<=2);memcpy(queued,s,n*sizeof(*s));zdt_script_count=n;zdt_script_running=1;}
#include "map_owner_under_test.inc"
int main(void)
{
  int32_t base[4]={0};grid_nav_segment_t segment={0};Nav_Params_Reset();
  for(unsigned side=0;side<2;side++) {
    zdt_odo_owner=0;zdt_nav_nose_y=side?1:-1;yaw=6.5f;
    zdt_nav_start_ready=1;zdt_nav_align_done=0;zdt_nav_state=ZDT_NAV_WAIT_START_QUERY;
    segment.dx=1;segment.dy=0;zdt_nav_segment_target_rev=25.0f/ZDT_MILEAGE_MM_PER_REV;
    Test_NavStartCase(&segment);
    CHECK(zdt_nav_state==ZDT_NAV_DRIVE && !turns && zdt_odo.map_mode && zdt_nav_keep_heading);
    CHECK(fabsf(zdt_odo.target_mm-25.75f)<.01f && zdt_nav_drive_scale==.1f);
    CHECK(ZDT_Odo_Service(1)==0 && field_x>0 && fabsf(field_y)<.0001f && fabsf(field_o)<.0001f);
    ZDT_Odo_Cancel();blocked=1;CHECK(!ZDT_Nav_TryMapTranslation(&segment));blocked=0;
    extra=1;CHECK(!ZDT_Nav_TryMapTranslation(&segment));extra=0;
    zdt_nav_segment_target_rev=151.0f/ZDT_MILEAGE_MM_PER_REV;
    CHECK(!ZDT_Nav_TryMapTranslation(&segment));
    zdt_start_zone=side?2:1;ZDT_Script_StartLeave();
    CHECK(zdt_script_count==1 && queued[0].dir==ZDT_SCRIPT_DIR_MAP && queued[0].mm==325);
    CHECK(queued[0].map_dx==-125 && queued[0].map_dy==(side?-300:300));
    CHECK(ZDT_Odo_StartMap(2,queued[0].mm,.75f,queued[0].map_dx,queued[0].map_dy,yaw,NULL,0));
    CHECK(zdt_odo_base_pending && zdt_odo_map_mode);
    zdt_odo_wait=1;zdt_mileage_query_sample_tick=now;CHECK(ZDT_Odo_OnMileage(15));
    CHECK(zdt_odo.map_mode && !zdt_odo_base_pending && fabsf(zdt_odo.target_mm-334.75f)<.01f);
    CHECK(!ZDT_Odo_Service(2) && field_x<0 && (side?field_y<0:field_y>0));
    CHECK(fabsf(field_x/field_y-(side?125.0f/300:-125.0f/300))<.001f && !turns);
    ZDT_Odo_Cancel();CHECK(ZDT_Odo_Start(1,100,.1f,1,0,yaw,base,15));
    CHECK(!zdt_odo.map_mode && !zdt_odo_map_mode);ZDT_Odo_Cancel();
    pose_x=2250;pose_y=side?2250:150;zdt_script_count=1;zdt_script_index=0;zdt_script_running=1;
    zdt_match.state=MATCH_LEAVING;zdt_match.automatic=zdt_match.qr_required=1;yaw=0;
    CHECK(ZDT_Odo_StartMap(2,325,.75f,-125,side?-300:300,0,NULL,0));
    ZDT_Departure_PrepareRoll(&queued[0]);CHECK(zdt_departure_roll);
    zdt_odo_wait=1;zdt_mileage_query_sample_tick=now;CHECK(ZDT_Odo_OnMileage(15));
    CHECK(zdt_odo.rolling_tail_mm>150);
    zdt_mileage_query_mask=15;
    zdt_odo.map_dx=-128.75f;zdt_odo.map_dy=side?-309:309;
    zdt_odo.progress_mm=zdt_odo.target_mm;zdt_odo.last_sample=now;
    CHECK(ZDT_Departure_RollStep());
    CHECK(completes==side+1 && !stop_calls && zdt_nav_state==ZDT_NAV_DRIVE && zdt_odo_owner==1);
    CHECK(zdt_odo.map_mode && zdt_odo_map_hold_line && zdt_odo.rolling_tail_mm==0 && !zdt_departure_transition);
    CHECK(field_x>0 && (side?field_y<0:field_y>0)); /* Correct residual side drift while driving. */
    ZDT_Odo_Cancel();
    zdt_match.state=MATCH_LEAVING;zdt_script_running=1;pose_x=2250;pose_y=side?2250:150;
    CHECK(ZDT_Odo_StartMap(2,325,.75f,-125,side?-300:300,0,NULL,0));
    blocked=1;ZDT_Departure_PrepareRoll(&queued[0]);CHECK(!zdt_departure_roll && !ZDT_Departure_RollStep());
    blocked=0;qr_x=2225;ZDT_Departure_PrepareRoll(&queued[0]);CHECK(!zdt_departure_roll);
    qr_x=2125;ZDT_Odo_Cancel();zdt_match.state=MATCH_IDLE;
    pose_x=2100;pose_y=512;zdt_nav_rolling_handoff=0;
  }
  CHECK(!failures);return test_failures;
}
