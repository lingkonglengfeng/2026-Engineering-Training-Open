#include "chassis_motor.h"
#include "zone2_route.h"
#include "nav_blend.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
typedef enum { Z2_NONE,Z2_NAV,Z2_TURN } zdt_zone2_phase_t;
static zone2_route_t zdt_zone2;
static zone2_action_t zdt_zone2_action;
static zdt_zone2_phase_t zdt_zone2_phase;
static match_control_t zdt_match;
static uint32_t zdt_zone2_tick,zdt_zone2_imu_tick,zdt_zone2_imu_count;
static uint32_t test_route_tick,test_route_samples,test_start_delay;
static uint8_t test_route_ready,test_sample_during_start;
static const char *test_route_reason;
static char test_route_log[128];
static uint32_t RouteTest_GetTick(void){return test_route_tick;}
static uint32_t RouteTest_GetSampleCount(void){return test_route_samples;}
static uint8_t RouteTest_IsReady(void){return test_route_ready;}
static void RouteTest_Telemetry(chassis_telemetry_t *t)
{memset(t,0,sizeof(*t));}
static uint8_t RouteTest_IsTurnActive(void){return 1U;}
static uint8_t ZDT_Zone2_WheelsStopped(void){return 1U;}
static float ZDT_Zone2_RelYaw(float world){return world;}
static void RouteTest_Print(const char *line)
{(void)snprintf(test_route_log,sizeof(test_route_log),"%s",line);}
static void ZDT_Zone2_Fail(const char *reason)
{test_route_reason=reason;Zone2Route_Result(&zdt_zone2,zdt_zone2.active_id,0U);}
static void ZDT_Zone2_Done(void)
{Zone2Route_Result(&zdt_zone2,zdt_zone2.active_id,1U);}
static void RouteTest_Cancel(void){zdt_zone2_phase=Z2_NONE;}
static void RouteTest_Finished(uint8_t success)
{(void)success;zdt_zone2_phase=Z2_NONE;}
static uint8_t RouteTest_Start(const zone2_action_t *a,uint32_t id)
{
    (void)id;test_route_tick+=2U; /* callback reads the clock after poll entry */
    zdt_zone2_action=*a;zdt_zone2_imu_count=test_route_samples;
    zdt_zone2_imu_tick=test_route_tick;
    zdt_zone2_phase=a->kind==ZONE2_NAV?Z2_NAV:Z2_TURN;
    test_route_tick+=test_start_delay; /* synchronous logs / planning */
    if(test_sample_during_start)test_route_samples++;
    zdt_zone2_tick=test_route_tick+10000U;return 1U;
}
/* Compile the production service body, extracted verbatim by the runner.
 * Peripheral bindings are local mocks; route callbacks are the real state
 * machine. This catches an old poll time being compared to a new action time. */
#define HAL_GetTick RouteTest_GetTick
#define HWT101_GetSampleCount RouteTest_GetSampleCount
#define HWT101_IsZeroDriftReady RouteTest_IsReady
#define ChassisMotor_GetTelemetry RouteTest_Telemetry
#define ChassisMotor_IsTurnActive RouteTest_IsTurnActive
#define ZDT_Console_Print RouteTest_Print
#include "zone2_service_under_test.inc"
#undef HAL_GetTick
#undef HWT101_GetSampleCount
#undef HWT101_IsZeroDriftReady
#undef ChassisMotor_GetTelemetry
#undef ChassisMotor_IsTurnActive
#undef ZDT_Console_Print

static void RouteTest_Reset(uint32_t tick,uint32_t delay)
{
    const zone2_ports_t ports={RouteTest_Start,RouteTest_Cancel,RouteTest_Finished};
    Zone2Route_Init(&zdt_zone2,&ports);zdt_zone2_phase=Z2_NONE;
    test_route_tick=tick;test_route_samples=100U;test_start_delay=delay;
    test_route_ready=1U;test_sample_during_start=0U;test_route_reason=NULL;
    test_route_log[0]='\0';zdt_match.checkpoint=MATCH_ROUGH2;
    CHECK(Zone2Route_StartForZone(&zdt_zone2,MATCH_MOTION_ROUGH,MATCH_ROUGH2,1U));
}
void Test_Zone2ActionClock(void)
{
    uint32_t i,started;
    /* NAV and following terminal TURN both initialize a newer timestamp.
     * Delays below150 must pass even when there is no new IMU sample. */
    for(i=0U;i<3U;i++) {
        RouteTest_Reset(i==2U?UINT32_MAX-10U:1000U+i*1000U,i*10U);
        ZDT_Zone2_Service();
        CHECK(zdt_zone2.running && zdt_zone2_phase==Z2_NAV && !test_route_reason);
        CHECK((test_route_tick-zdt_zone2_imu_tick)==test_start_delay);
        Zone2Route_Result(&zdt_zone2,zdt_zone2.active_id,1U);
        test_route_tick+=40U;ZDT_Zone2_Service();
        CHECK(zdt_zone2.running && zdt_zone2_phase==Z2_NAV && !test_route_reason);
        Zone2Route_Result(&zdt_zone2,zdt_zone2.active_id,1U);
        test_route_tick+=40U;ZDT_Zone2_Service();
        CHECK(zdt_zone2.running && zdt_zone2_phase==Z2_TURN && !test_route_reason);
    }
    /* Exact150ms is allowed;151ms without samples remains a genuine fault. */
    RouteTest_Reset(10000U,0U);ZDT_Zone2_Service();started=zdt_zone2_imu_tick;
    test_route_tick=started+150U;ZDT_Zone2_Service();CHECK(zdt_zone2.running);
    test_route_tick=started+151U;ZDT_Zone2_Service();
    CHECK(!zdt_zone2.running && test_route_reason && strstr(test_route_reason,"IMU stale"));
    CHECK(strstr(test_route_log,"age=151ms")!=NULL);
    /* Re-reading now must not erase real callback stalls. */
    RouteTest_Reset(20000U,200U);ZDT_Zone2_Service();
    CHECK(!zdt_zone2.running && test_route_reason && strstr(test_route_log,"age=200ms"));
    RouteTest_Reset(30000U,200U);test_sample_during_start=1U;ZDT_Zone2_Service();
    CHECK(zdt_zone2.running && !test_route_reason && zdt_zone2_imu_count==101U);
    CHECK(zdt_zone2_imu_tick==test_route_tick);
    /* HAL tick rollover must still reject a real151ms sample gap. */
    RouteTest_Reset(UINT32_MAX-20U,0U);ZDT_Zone2_Service();
    test_route_tick=zdt_zone2_imu_tick+151U;ZDT_Zone2_Service();
    CHECK(!zdt_zone2.running && test_route_reason && strstr(test_route_log,"age=151ms"));
    RouteTest_Reset(40000U,0U);test_route_ready=0U;ZDT_Zone2_Service();
    CHECK(!zdt_zone2.running && test_route_reason && strstr(test_route_reason,"not ready"));
}
