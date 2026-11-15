/* Production turn->drive decision blocks; hardware and next motion are ports. */
#include "chassis_motor.h"
#include "zone2_route.h"
#include "grid_nav.h"
#include "nav_blend.h"
#include "nav_params.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);}}while(0)
#define ZDT_NAV_ALIGN_TOLERANCE_DEG 3.0f
#define ZDT_TURN_HANDOFF_STABLE_MS ((uint32_t)Nav_Motion()->turn_stable_ms)
#define ZDT_TURN_HANDOFF_TOLERANCE_DEG (Nav_Motion()->turn_band_deg)
typedef struct{uint32_t since;uint8_t active;}zdt_turn_band_t;
static zdt_turn_band_t zdt_turn_band[2];
enum{ZDT_NAV_IDLE,ZDT_NAV_WAIT_START_QUERY,ZDT_NAV_ALIGN_TURN,ZDT_NAV_DRIVE,ZDT_NAV_OUTER_CORNER};
enum{Z2_NONE,Z2_NAV,Z2_TURN};
static uint8_t zdt_nav_state,zdt_nav_odometry_enabled=1,zdt_nav_start_mask,zdt_nav_wait_start_query;
static uint8_t zdt_nav_start_ready,zdt_nav_query_failed,zdt_nav_align_done,zdt_nav_rolling_handoff,zdt_odo_owner;
static uint8_t zdt_mileage_query_active,zdt_mileage_query_mask,zdt_zone2_phase;
static uint32_t zdt_mileage_query_sample_tick,zdt_zone2_imu_count,zdt_zone2_imu_tick,zdt_zone2_tick,zdt_zone2_action_id;
static uint32_t zdt_outer_control_tick,now,queries,releases,drives,arc_commands,stop_calls,failures,nav_starts;
static uint8_t turn_active,stopped,demand,odo_result,choose_arc;
static float yaw,gyro,zdt_drive_handoff_heading;
static uint32_t turns;
static HAL_StatusTypeDef TurnToHeading(float target,uint8_t fast){CHECK(target==90 && fast<=1);turns++;turn_active=1;return HAL_OK;}
static int32_t zdt_nav_start_position[4],zdt_mileage_query_position[4];
static grid_nav_segment_t test_segment;
static zone2_route_t zdt_zone2;
static zone2_action_t zdt_zone2_action;
static match_control_t zdt_match;
static struct{const char *reason;}zdt_odo;
static uint32_t Tick(void){return now;}
static float ZDT_Nav_GetHeading(void){return yaw;}
static float ZDT_Nav_SegmentHeadingDeg(const grid_nav_segment_t *s){CHECK(s==&test_segment);return 90.0f;}
static float ZDT_Zone2_RelYaw(float world){return world;}
static void Telemetry(chassis_telemetry_t *t){memset(t,0,sizeof(*t));t->heading_deg=yaw;t->gyro_dps=gyro;}
static void Release(void){turn_active=0;releases++;}
static uint8_t TurnActive(void){return turn_active;}
static uint8_t Ready(void){return 1;}
static uint32_t Samples(void){return now;}
static uint8_t ZDT_Zone2_WheelsStopped(void){return stopped;}
static uint8_t Demand(void){return demand;}
static HAL_StatusTypeDef Stop(void){stop_calls++;return HAL_OK;}
static void Print(const char *s){CHECK(s && *s);}
static void ZDT_Mileage_StartQuery(uint8_t report){CHECK(!report);queries++;zdt_mileage_query_active=1;}
static void ZDT_Nav_Fail(const char *reason){CHECK(reason!=NULL);failures++;}
static void ZDT_Zone2_Fail(const char *reason){ZDT_Nav_Fail(reason);}
static void ZDT_Nav_Service(void)
{
  nav_starts++;CHECK(zdt_nav_rolling_handoff);
  CHECK(zdt_nav_start_ready && zdt_nav_start_mask==15 && !zdt_nav_wait_start_query);
  CHECK(!memcmp(zdt_nav_start_position,zdt_mileage_query_position,sizeof(zdt_nav_start_position)));
  zdt_nav_state=choose_arc?ZDT_NAV_OUTER_CORNER:ZDT_NAV_DRIVE;zdt_odo_owner=1;
}
static uint8_t ZDT_Odo_Service(uint8_t owner){CHECK(owner==1 && !zdt_nav_rolling_handoff);drives++;return odo_result;}
static void ZDT_Outer_Service(void){CHECK(now-zdt_outer_control_tick==20);arc_commands++;}
#define HAL_GetTick Tick
#define ChassisMotor_GetTelemetry Telemetry
#define ChassisMotor_ReleaseTurnForDrive Release
#define ChassisMotor_TurnToHeading TurnToHeading
#define ChassisMotor_IsTurnActive TurnActive
#define ChassisMotor_HasMotionDemand Demand
#define ChassisMotor_Stop Stop
#define HWT101_IsZeroDriftReady Ready
#define HWT101_GetSampleCount Samples
#define ZDT_Console_Print Print
#include "turn_handoff_under_test.inc"
static uint8_t next_nav(const zone2_action_t *a,uint32_t id)
{
  CHECK(a->kind==ZONE2_NAV && id && zdt_nav_rolling_handoff && !turn_active);
  /* Production Nav_Start consumes the prefetched, actual four-wheel baseline. */
  memcpy(zdt_nav_start_position,zdt_mileage_query_position,sizeof(zdt_nav_start_position));
  zdt_nav_start_mask=15;zdt_nav_start_ready=1;zdt_nav_wait_start_query=0;
  zdt_zone2_phase=Z2_NAV;ZDT_Nav_ResumeRolling();return 1;
}
static void reset(float heading)
{
  zone2_ports_t ports={0};ports.start=next_nav;Zone2Route_Init(&zdt_zone2,&ports);
  memset(zdt_turn_band,0,sizeof(zdt_turn_band));
  now=1000;yaw=heading;gyro=90;turn_active=demand=1;stopped=choose_arc=odo_result=0;
  queries=releases=drives=arc_commands=stop_calls=failures=nav_starts=turns=0;
  zdt_nav_state=ZDT_NAV_ALIGN_TURN;zdt_nav_odometry_enabled=1;zdt_nav_rolling_handoff=0;
  zdt_nav_wait_start_query=1;zdt_nav_start_ready=zdt_nav_start_mask=zdt_nav_align_done=0;
  zdt_mileage_query_active=0;zdt_mileage_query_mask=15;zdt_mileage_query_sample_tick=995;
  zdt_mileage_query_position[0]=12345;zdt_mileage_query_position[1]=-7654;
  zdt_mileage_query_position[2]=2356;zdt_mileage_query_position[3]=-4567;
  zdt_zone2.running=1;zdt_zone2.count=2;zdt_zone2.active_id=zdt_zone2_action_id=7;
  zdt_zone2.actions[0].kind=ZONE2_TURN;zdt_zone2.actions[1].kind=ZONE2_NAV;
  zdt_zone2_action.yaw=90;zdt_zone2_phase=Z2_TURN;zdt_zone2_tick=10000;
  zdt_zone2_imu_count=3;zdt_zone2_imu_tick=1000;zdt_match.checkpoint=MATCH_RETURN;
}
static void qualify(uint8_t route)
{
  if(route)ZDT_Zone2_Service();else Test_NavTurnCase();
  CHECK(!releases && !drives);now+=99;zdt_mileage_query_active=0;zdt_mileage_query_mask=15;zdt_mileage_query_sample_tick=now;
  if(route)ZDT_Zone2_Service();else Test_NavTurnCase();
  CHECK(!releases && !drives);now++;
  if(route)ZDT_Zone2_Service();else Test_NavTurnCase();
}
void Test_TurnHandoff(void)
{
  Nav_Motion()->turn_band_deg=.5f;Nav_Motion()->turn_stable_ms=200;
  memset(zdt_turn_band,0,sizeof(zdt_turn_band));
  CHECK(!ZDT_TurnBandStable(0,.4f,1000));CHECK(!ZDT_TurnBandStable(0,.4f,1199));
  CHECK(ZDT_TurnBandStable(0,.4f,1200));CHECK(!ZDT_TurnBandStable(0,.6f,1201));
  CHECK(!ZDT_TurnBandStable(0,.4f,1210));CHECK(!ZDT_TurnBandStable(0,.4f,1409));
  CHECK(ZDT_TurnBandStable(0,.4f,1410));
  Nav_Motion()->turn_band_deg=1;Nav_Motion()->turn_stable_ms=100;

  unsigned i;const float inside[]={89.0f,89.001f,90.999f,91.0f};
  for(i=0;i<4;i++){
    reset(inside[i]);qualify(0);
    CHECK(releases==1 && nav_starts==1 && drives==1 && !turn_active);
    CHECK(!stop_calls && !failures && !queries && zdt_nav_state==ZDT_NAV_DRIVE);
    CHECK(zdt_drive_handoff_heading==90 && zdt_nav_align_done && !zdt_nav_rolling_handoff);
    reset(inside[i]);qualify(1);
    CHECK(releases==1 && nav_starts==1 && drives==1 && zdt_zone2.index==1);
    CHECK(!stop_calls && !failures && !queries && zdt_zone2_phase==Z2_NAV);
  }
  reset(88.999f);Test_NavTurnCase();CHECK(!releases && !drives && zdt_nav_state==ZDT_NAV_ALIGN_TURN);
  reset(91.001f);ZDT_Zone2_Service();CHECK(!releases && !drives && zdt_zone2.index==0);
  reset(89.8f);zdt_mileage_query_active=1;Test_NavTurnCase();CHECK(!releases && !queries);
  reset(89.8f);zdt_mileage_query_mask=7;zdt_mileage_query_sample_tick=970;Test_NavTurnCase();CHECK(queries==1 && !releases);
  reset(89.8f);zdt_mileage_query_sample_tick=849;Test_NavTurnCase();CHECK(queries==1 && !releases);
  reset(89.8f);qualify(0);CHECK(releases==1 && drives==1);
  reset(89.8f);choose_arc=1;qualify(0);CHECK(releases==1 && arc_commands==1 && !drives && !stop_calls);
  /* Leaving the band resets, rather than accumulating two short visits. */
  for(i=0;i<2;i++) {
    reset(89.8f);if(i)ZDT_Zone2_Service();else Test_NavTurnCase();
    now+=90;yaw=88.99f;if(i)ZDT_Zone2_Service();else Test_NavTurnCase();CHECK(!zdt_turn_band[i].active);
    yaw=89.8f;qualify((uint8_t)i);CHECK(releases==1 && drives==1);
  }
  /* Tick zero and uint32 rollover are valid timing origins. */
  reset(89.8f);now=0;CHECK(!ZDT_TurnBandStable(0,0.2f,now));CHECK(!ZDT_TurnBandStable(0,0.2f,99));CHECK(ZDT_TurnBandStable(0,0.2f,100));
  reset(89.8f);CHECK(!ZDT_TurnBandStable(0,0.2f,UINT32_MAX-49));CHECK(!ZDT_TurnBandStable(0,0.2f,49));CHECK(ZDT_TurnBandStable(0,0.2f,50));
  /* PID becoming inactive cannot skip the100ms gate. */
  reset(89.8f);turn_active=0;Test_NavTurnCase();CHECK(!releases && !drives && zdt_nav_state==ZDT_NAV_ALIGN_TURN);
  reset(86);turn_active=0;Test_NavTurnCase();CHECK(zdt_nav_state==ZDT_NAV_WAIT_START_QUERY && !zdt_nav_align_done && !drives);
  reset(86);turn_active=0;ZDT_Zone2_Service();CHECK(turns==1 && turn_active && !drives);
  /* A station-only/final facing never takes the rolling branch. */
  reset(89.8f);zdt_zone2.count=1;ZDT_Zone2_Service();CHECK(!releases && !drives && !failures);
  reset(88);zdt_zone2.count=1;turn_active=0;stopped=1;gyro=0;ZDT_Zone2_Service();CHECK(failures==1 && !releases);
  reset(89.6f);zdt_zone2.count=1;turn_active=0;stopped=1;gyro=0;ZDT_Zone2_Service();CHECK(!failures && !zdt_zone2.running && !releases);
  /* Legacy non-odometry navigation retains its original completed-turn query. */
  reset(89.8f);zdt_nav_odometry_enabled=0;turn_active=0;Test_NavTurnCase();CHECK(queries==1 && !releases && zdt_nav_state==ZDT_NAV_WAIT_START_QUERY);
}
