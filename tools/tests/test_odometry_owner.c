#include "chassis_motor.h"
#include "nav_params.h"
#include "nav_odometry.h"
#include "nav_feedback.h"
#include "nav_frame.h"
#include "nav_blend.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
#define ZDT_NAV_HOLD_KP 0.035f
#define ZDT_ODO_HOLD_KP 0.030f
#define ZDT_ODO_HOLD_OMEGA_MAX 0.050f
#define ZDT_NAV_HOLD_DEADBAND_DEG 0.15f
#define ZDT_MILEAGE_MOTOR_COUNT 4U
static nav_odometry_t zdt_odo;
static uint8_t zdt_odo_owner,zdt_odo_wait,zdt_odo_base_pending;
static uint32_t zdt_odo_next_query,zdt_odo_tick,zdt_odo_imu_tick,zdt_odo_imu_count;
static float zdt_odo_mm,zdt_odo_max_scale,zdt_odo_heading;
static float zdt_odo_approach_gain;
static uint8_t zdt_script_fast_exit;
typedef struct {uint8_t dir;uint16_t mm;int16_t map_dx,map_dy;} zdt_script_step_t;
#define ZDT_SCRIPT_DIR_TURN180 4U
#define ZDT_SCRIPT_SETTLE_MS 350U
static int8_t zdt_odo_forward,zdt_odo_right;
static uint8_t zdt_mileage_query_active,zdt_mileage_query_report,zdt_mileage_query_attempts;
static uint8_t zdt_mileage_query_mask;
static int32_t zdt_mileage_query_position[4];
static uint32_t zdt_mileage_query_sample_tick,zdt_mileage_query_baseline_seq,zdt_mileage_query_tick;
static const double zdt_mileage_forward_signs[4]={-CHASSIS_WHEEL_SIGN_FL,-CHASSIS_WHEEL_SIGN_FR,
    -CHASSIS_WHEEL_SIGN_RL,-CHASSIS_WHEEL_SIGN_RR};
static const uint8_t zdt_mileage_addresses[4]={4U,2U,3U,1U};
static const double zdt_nav_command_signs[4]={1,-1,1,-1};
#define ZDT_POSITION_PULSES_PER_REV 3730.0f
static float zdt_nav_drive_scale,zdt_script_drive_scale;
static uint32_t test_tick,test_samples,test_queries,test_finished,test_stop;
enum { ZDT_NAV_DRIVE,ZDT_NAV_WAIT_END_QUERY };
static uint8_t zdt_nav_state,zdt_nav_odometry_enabled=1U,zdt_nav_end_ready,zdt_nav_query_failed;
static uint8_t zdt_nav_settle_valid,zdt_nav_wait_end_query,test_nav_failed;
static uint32_t zdt_nav_settle_deadline,zdt_nav_motion_deadline,zdt_nav_rpm_report_tick,zdt_nav_hold_tick;
static uint8_t zdt_nav_start_mask,zdt_nav_end_mask;
static float zdt_nav_segment_target_rev,zdt_nav_measured_rev;
#define ZDT_NAV_SETTLE_TIMEOUT_MS 2000U
#define ZDT_NAV_HOLD_PERIOD_MS 80U
static int16_t test_rpm[4];static float test_f,test_r,test_yaw;static uint8_t test_enabled,test_ready;
static uint32_t Owner_GetTick(void){return test_tick;}
static uint32_t Owner_Samples(void){return test_samples;}
static uint8_t Owner_Enabled(void){return test_enabled;}
static uint8_t Owner_Ready(void){return test_ready;}
static void Owner_Telemetry(chassis_telemetry_t *t){memset(t,0,sizeof(*t));t->heading_deg=test_yaw;}
static void Owner_Rpm(int16_t *p){memcpy(p,test_rpm,sizeof(test_rpm));}
static HAL_StatusTypeDef Owner_Velocity(float f,float r,float yaw)
{(void)yaw;test_f=f;test_r=r;return HAL_OK;}
static HAL_StatusTypeDef Owner_Stop(void){test_stop++;test_f=test_r=0.0f;return HAL_OK;}
static void Owner_ResetRamp(void){}
static char test_odo_fault[256];
static void Owner_Print(const char *line){if(strncmp(line,"[ODO FAULT]",11)==0)strncpy(test_odo_fault,line,sizeof(test_odo_fault)-1);}
static HAL_StatusTypeDef Owner_Command(chassis_command_t c){(void)c;return Owner_Stop();}
static void ZDT_Nav_HeadingHold(void){}
static float ZDT_Vobs_SpeedLimit(float scale){return scale;} /* No obstacle in odometry-only fixture. */
static void ZDT_Nav_Fail(const char *why){(void)why;test_nav_failed=1U;}
static void ZDT_Mileage_StartQuery(uint8_t report)
{(void)report;test_queries++;zdt_mileage_query_active=1U;zdt_mileage_query_mask=0U;zdt_mileage_query_tick=test_tick;}
static uint8_t ZDT_Odo_OnMileage(uint8_t mask);
static void ZDT_Mileage_FinishQuery(void){test_finished++;(void)ZDT_Odo_OnMileage(zdt_mileage_query_mask);}
#define HAL_GetTick Owner_GetTick
#define HWT101_GetSampleCount Owner_Samples
#define HWT101_IsZeroDriftReady Owner_Ready
#define ChassisMotor_IsEnabled Owner_Enabled
#define ChassisMotor_GetTelemetry Owner_Telemetry
#define ChassisMotor_GetWheelRpm Owner_Rpm
#define ChassisMotor_SetVelocity Owner_Velocity
#define ChassisMotor_Stop Owner_Stop
#define ChassisMotor_StopImmediate Owner_Stop
#define ChassisMotor_ResetIdleRamp Owner_ResetRamp
#define ZDT_Console_Print Owner_Print
#define ChassisMotor_SetCommand Owner_Command
/* Real owner/query functions, extracted verbatim rather than copied. */
#include "odometry_owner_under_test.inc"
static void Owner_ActualDriveCase(void)
{switch(zdt_nav_state){
#include "odometry_drive_case_under_test.inc"
default:break;}}
static nav_motion_feedback_t Owner_ActualMatchCheck(float planned_mm,float measured_mm)
{
#include "odometry_match_check_under_test.inc"
    return check;
}
static void Owner_ActualFinalGuard(void)
{
#include "odometry_final_guard_under_test.inc"
}
#undef HAL_GetTick
#undef HWT101_GetSampleCount
#undef HWT101_IsZeroDriftReady
#undef ChassisMotor_IsEnabled
#undef ChassisMotor_GetTelemetry
#undef ChassisMotor_GetWheelRpm
#undef ChassisMotor_SetVelocity
#undef ChassisMotor_Stop
#undef ChassisMotor_StopImmediate
#undef ChassisMotor_ResetIdleRamp
#undef ZDT_Console_Print
#undef ChassisMotor_SetCommand
static void reset(void)
{
    memset(&zdt_odo,0,sizeof(zdt_odo));zdt_odo_owner=zdt_odo_wait=zdt_odo_base_pending=0U;
    zdt_script_fast_exit=0U;
    zdt_mileage_query_active=0U;zdt_mileage_query_baseline_seq=100U;
    test_tick=test_queries=test_finished=test_stop=0U;test_samples=1U;test_enabled=test_ready=1U;
    test_yaw=test_f=test_r=0.0f;memset(test_rpm,0,sizeof(test_rpm));
    test_odo_fault[0]=0;
    memset(zdt_mileage_query_position,0,sizeof(zdt_mileage_query_position));Nav_Params_Reset();
    zdt_nav_end_ready=zdt_nav_query_failed=zdt_nav_settle_valid=zdt_nav_wait_end_query=test_nav_failed=0U;
    zdt_nav_state=ZDT_NAV_DRIVE;zdt_nav_motion_deadline=1U;zdt_nav_rpm_report_tick=zdt_nav_hold_tick=0U;
    zdt_nav_odometry_enabled=1U;zdt_nav_start_mask=zdt_nav_end_mask=15U;
}
static void reply(float distance,uint8_t mask)
{
    uint8_t i;for(i=0U;i<4U;i++)zdt_mileage_query_position[i]=(int32_t)lroundf(distance*65536.0f/
        ZDT_MILEAGE_MM_PER_REV*zdt_odo.expected[i]/(float)zdt_mileage_forward_signs[i]);
    zdt_mileage_query_sample_tick=test_tick;CHECK(ZDT_Odo_OnMileage(mask));
}
void Test_OdometryOwner(void)
{
    int32_t base[4]={0};uint8_t result=0U;float actual=0.0f;int16_t rpm=0;uint32_t t;
    FDCAN_RxHeaderTypeDef h={0};uint8_t bytes[7]={0x36U,0U,0U,0U,0U,123U,0x6BU};
    /* V174 targets are enlarged once for all four directions and owners,
     * including a deferred baseline. Raw encoder units remain unchanged. */
    {
        const int8_t dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
        const float nominal[3]={1000.0f,300.0f,30.0f};
        const float target[3]={1030.0f,309.0f,30.9f};
        unsigned owner,dir;
        for(owner=1;owner<=3;owner++)for(dir=0;dir<4;dir++) {
            reset();CHECK(ZDT_Odo_Start((uint8_t)owner,nominal[owner-1],0.1f,dirs[dir][0],dirs[dir][1],0.0f,base,15U));
            CHECK(fabsf(zdt_odo.target_mm-target[owner-1])<0.001f);
            CHECK(zdt_odo.counts_per_rev==65536.0f);
        }
        reset();CHECK(ZDT_Odo_Start(2U,125.0f,0.1f,0,-1,0.0f,NULL,0U));
        CHECK(fabsf(zdt_odo_mm-128.75f)<0.001f);
        CHECK(ZDT_Odo_InitBase(base,15U,test_tick));
        CHECK(fabsf(zdt_odo.target_mm-128.75f)<0.001f);
        {const float wheels[4]={1,1,-1,-1};unsigned i;
         for(i=0;i<4;i++) {
            int32_t expected=(int32_t)(i==0 || i==3?3842:-3842);
            CHECK(ZDT_Nav_StepPulses((uint8_t)i,wheels,1.0f,1)==expected);
            CHECK(ZDT_Nav_StepPulses((uint8_t)i,wheels,1.0f,-1)==-expected);
         }}
        reset();zdt_nav_segment_target_rev=1000.0f/ZDT_MILEAGE_MM_PER_REV;
        zdt_nav_measured_rev=1030.0f/ZDT_MILEAGE_MM_PER_REV;
        Owner_ActualFinalGuard();CHECK(!test_nav_failed);
        CHECK(Owner_ActualMatchCheck(1000.0f,1030.0f)==NAV_MOTION_OK);
        zdt_nav_measured_rev=1051.0f/ZDT_MILEAGE_MM_PER_REV;
        Owner_ActualFinalGuard();CHECK(test_nav_failed);
    }
    /* The2mm lane target becomes2.06mm; minimal actual progress reaches
     * its existing2mm arrival band and hands off to the main leg. */
    reset();CHECK(ZDT_Odo_Start(1U,2.0f,0.1f,0,1,0.0f,base,15U));
    for(t=20U;t<=220U && zdt_nav_state==ZDT_NAV_DRIVE;t+=20U) {
        test_tick=t;test_samples++;if(zdt_odo_wait)reply(0.1f,15U);Owner_ActualDriveCase();
    }
    CHECK(zdt_odo.state==NAV_ODO_DONE && zdt_nav_state==ZDT_NAV_WAIT_END_QUERY && !test_nav_failed);
    zdt_nav_segment_target_rev=2.0f/ZDT_MILEAGE_MM_PER_REV;zdt_nav_measured_rev=0.0f;
    Owner_ActualFinalGuard();CHECK(!test_nav_failed);
    CHECK(Owner_ActualMatchCheck(2.0f,0.0f)==NAV_MOTION_OK);
    zdt_nav_end_mask=7U;Owner_ActualFinalGuard();CHECK(test_nav_failed);
    zdt_nav_end_mask=15U;test_nav_failed=0U;zdt_nav_segment_target_rev=20.0f/ZDT_MILEAGE_MM_PER_REV;
    Owner_ActualFinalGuard();CHECK(!test_nav_failed);
    zdt_nav_odometry_enabled=0U;CHECK(Owner_ActualMatchCheck(2.0f,0.0f)==NAV_MOTION_NO_MOVEMENT);
    zdt_nav_odometry_enabled=1U;
    CHECK(Owner_ActualMatchCheck(100.0f,0.0f)==NAV_MOTION_OK);
    CHECK(Owner_ActualMatchCheck(100.0f,-10.0f)==NAV_MOTION_REVERSED);
    CHECK(Owner_ActualMatchCheck(100.0f,NAN)==NAV_MOTION_INVALID);
    reset();CHECK(ZDT_Odo_Start(1U,100.0f,0.3f,1,0,0.0f,base,15U));
    test_tick=20U;test_samples++;Owner_ActualDriveCase();
    CHECK(zdt_nav_state==ZDT_NAV_DRIVE && !zdt_nav_wait_end_query && !test_nav_failed && test_f>0.0f);
    test_tick=220U;test_samples++;Owner_ActualDriveCase();
    CHECK(test_nav_failed && test_stop && !zdt_odo_owner && !test_f);
    reset();CHECK(ZDT_Odo_Start(1U,1650.0f,0.3f,1,0,0.0f,base,15U));
    for(t=20U;t<30000U && !result;t+=20U) {
        test_tick=t;test_samples++;
        if(zdt_odo_wait)reply(actual,15U);
        result=ZDT_Odo_Service(1U);
        rpm=ChassisMotor_RampStepRpm(rpm,(int16_t)lroundf(test_f*250.0f),0.02f,292.0f);
        actual+=rpm*ZDT_MILEAGE_MM_PER_REV/60.0f*0.02f*0.70f;
        test_rpm[0]=test_rpm[1]=test_rpm[2]=test_rpm[3]=rpm;
    }
    CHECK(result==1U && fabsf(actual-1699.5f)<3.0f && test_queries>100U);
    CHECK(!zdt_odo_owner && !zdt_odo_wait && !zdt_mileage_query_active && !test_f && !test_r);
    /* PLATE approach owner uses real LEFT wheel projection and actual20mm
     * encoder arrival. It must stop before the station arm job begins. */
    reset();actual=0.0f;rpm=0;result=0U;
    CHECK(ZDT_Odo_Start(3U,20.0f,0.080f,0,-1,0.0f,base,15U));
    for(t=20U;t<5000U && !result;t+=20U) {
        test_tick=t;test_samples++;if(zdt_odo_wait)reply(actual,15U);
        result=ZDT_Odo_Service(3U);CHECK(test_f==0.0f && test_r<=0.0f);
        rpm=ChassisMotor_RampStepRpm(rpm,(int16_t)lroundf(-test_r*250.0f),0.02f,292.0f);
        actual+=rpm*ZDT_MILEAGE_MM_PER_REV/60.0f*0.02f*0.70f;
        test_rpm[0]=test_rpm[3]=-rpm;test_rpm[1]=test_rpm[2]=rpm;
    }
    CHECK(result==1U && fabsf(zdt_odo.progress_mm-20.0f)<3.0f && actual>17.0f);
    CHECK(!zdt_odo_owner && !test_r);
    reset();CHECK(ZDT_Odo_Start(3U,20.0f,0.080f,0,-1,0.0f,base,15U));
    CHECK(ZDT_Odo_Service(3U)==0U);ZDT_Odo_Cancel();CHECK(!zdt_odo_owner && !zdt_odo_wait);
    reset();CHECK(ZDT_Odo_Start(3U,20.0f,0.080f,0,-1,0.0f,base,15U));
    (void)ZDT_Odo_Service(3U);test_tick=60U;test_samples++;reply(0.0f,7U);
    CHECK(ZDT_Odo_Service(3U)==2U && !zdt_odo_owner && test_stop && !test_r);
    /* Script must wait for a fresh baseline, with no premature movement. */
    reset();CHECK(ZDT_Odo_Start(2U,125.0f,0.3f,0,-1,0.0f,NULL,0U));
    CHECK(ZDT_Odo_Service(2U)==0U && zdt_odo_wait && !test_f && !test_r);
    test_tick=20U;test_samples++;zdt_mileage_query_sample_tick=test_tick;
    CHECK(ZDT_Odo_OnMileage(15U) && !zdt_odo_base_pending);
    CHECK(zdt_odo.approach_gain==NAV_ODOMETRY_APPROACH_GAIN);
    CHECK(ZDT_Odo_Service(2U)==0U && test_f==0.0f && test_r<0.0f);
    test_tick=60U;test_samples++;CHECK(ZDT_Odo_Service(2U)==0U && zdt_odo_wait);
    reply(0.0f,7U);CHECK(ZDT_Odo_Service(2U)==2U && test_stop && !zdt_odo_owner && !test_r);
    reset();CHECK(ZDT_Odo_Start(2U,50.0f,0.1f,0,1,0.0f,NULL,0U));
    (void)ZDT_Odo_Service(2U);test_tick=200U;test_samples++;
    CHECK(ZDT_Odo_Service(2U)==2U && test_stop && !zdt_mileage_query_active);
    reset();CHECK(ZDT_Odo_Start(1U,50.0f,0.1f,-1,0,0.0f,base,15U));
    CHECK(ZDT_Odo_Service(1U)==0U && test_f<0.0f);
    ZDT_Odo_Cancel();CHECK(!ZDT_Odo_OnMileage(15U) && !zdt_odo_owner && !zdt_mileage_query_active);
    reset();CHECK(ZDT_Odo_Start(1U,50.0f,0.1f,1,0,0.0f,base,15U));
    test_tick=151U;CHECK(ZDT_Odo_Service(1U)==2U && test_stop);
    reset();zdt_script_fast_exit=1U;
    CHECK(ZDT_Odo_Start(2U,300.0f,0.35f,1,0,0.0f,NULL,0U));
    (void)ZDT_Odo_Service(2U);test_tick=20U;test_samples++;zdt_mileage_query_sample_tick=test_tick;
    CHECK(ZDT_Odo_OnMileage(15U) && zdt_odo.approach_gain==NAV_ODOMETRY_EXIT_APPROACH_GAIN);
    {zdt_script_step_t step={0};CHECK(ZDT_Script_SettleDelay(&step)==0U);
      step.dir=ZDT_SCRIPT_DIR_TURN180;CHECK(ZDT_Script_SettleDelay(&step)==350U);
      step.dir=0U;zdt_script_fast_exit=0U;CHECK(ZDT_Script_SettleDelay(&step)==350U);}
    ZDT_Odo_Cancel();CHECK(ZDT_Odo_Start(1U,300.0f,0.35f,1,0,0.0f,base,15U));
    CHECK(zdt_odo.approach_gain==NAV_ODOMETRY_APPROACH_GAIN);
    reset();CHECK(ZDT_Odo_Start(1U,50.0f,0.1f,1,0,0.0f,base,15U));
    test_tick=20U;test_yaw=3.1f;test_samples++;CHECK(ZDT_Odo_Service(1U)==2U && test_stop);
    /* Settled distance residual is accepted without altering the measurement. */
    reset();CHECK(ZDT_Odo_Start(2U,300.0f,0.4f,1,0,0.0f,base,15U));
    test_tick=200U;test_samples++;zdt_odo.state=NAV_ODO_BRAKE;
    zdt_odo.progress_mm=330.0f;zdt_odo.last_sample=200U;zdt_odo.stable_since=0U;
    CHECK(ZDT_Odo_Service(2U)==1U && test_stop && !zdt_odo_owner);
    CHECK(fabsf(zdt_odo.progress_mm-330.0f)<0.01f && !test_odo_fault[0]);
    /* Production receive parser rejects queued-old packets and duplicates;
     * normal read queries still get all four addresses. Sequence wraps. */
    reset();zdt_mileage_query_active=1U;zdt_mileage_query_mask=0U;zdt_mileage_query_tick=100U;
    h.IdType=FDCAN_EXTENDED_ID;h.RxFrameType=FDCAN_DATA_FRAME;h.Identifier=4U<<8U;
    ZDT_Mileage_HandleFrame(&h,bytes,7U,99U,101U);CHECK(!zdt_mileage_query_mask);
    ZDT_Mileage_HandleFrame(&h,bytes,7U,100U,100U);CHECK(!zdt_mileage_query_mask);
    ZDT_Mileage_HandleFrame(&h,bytes,7U,101U,101U);CHECK(zdt_mileage_query_mask==1U && zdt_mileage_query_position[0]==123);
    bytes[5]=124U;ZDT_Mileage_HandleFrame(&h,bytes,7U,102U,102U);CHECK(zdt_mileage_query_position[0]==123);
    h.Identifier=5U<<8U;ZDT_Mileage_HandleFrame(&h,bytes,7U,103U,103U);CHECK(zdt_mileage_query_mask==1U);
    h.Identifier=2U<<8U;bytes[1]=2U;ZDT_Mileage_HandleFrame(&h,bytes,7U,103U,103U);CHECK(zdt_mileage_query_mask==1U);
    bytes[1]=0U;ZDT_Mileage_HandleFrame(&h,bytes,7U,104U,104U);CHECK(zdt_mileage_query_mask==3U);
    h.Identifier=3U<<8U;ZDT_Mileage_HandleFrame(&h,bytes,7U,105U,105U);
    h.Identifier=1U<<8U;ZDT_Mileage_HandleFrame(&h,bytes,7U,106U,106U);CHECK(test_finished==1U);
    zdt_mileage_query_mask=0U;zdt_mileage_query_baseline_seq=UINT32_MAX;
    ZDT_Mileage_HandleFrame(&h,bytes,7U,107U,0U);CHECK(zdt_mileage_query_mask==8U);
}
