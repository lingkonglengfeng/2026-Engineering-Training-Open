/* Exercise the actual chassis API with only transport/IMU mocks. */
#include "chassis_motor.h"
#include "nav_params.h"
#include "zdt_motor.h"
#include "nav_short_move.h"
#include "nav_frame.h"
#include "nav_feedback.h"
#include <math.h>
#include <stdlib.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_SetTick(uint32_t tick);
#define CHECK(x) do { ++test_checks; if(!(x)) { ++test_failures; if(!test_first_failure) test_first_failure=__LINE__; } } while(0)
static float test_yaw;
void Test_SetChassisYaw(float yaw){test_yaw=yaw;}
static unsigned int stops,arm_stops;
static HAL_StatusTypeDef disable_result=HAL_OK;
static unsigned int wheel_disables;
/* Index by physical motor ID; verify actual wire commands and wheel mixing. */
static int16_t test_motor_rpm[5],test_pending_rpm[5];
static uint8_t test_pending_mask;
static unsigned int test_syncs;
static uint32_t test_ack[5],test_last_buffered_tick;
static uint8_t test_drop_ack,test_buffered_seen,test_speed_busy_id;
static uint8_t test_stop_busy_mask,test_stop_drop_mask,test_zero_drop_mask;
static uint32_t test_stop_attempts[5],test_zero_attempts[5];
float HWT101_GetZeroDriftDeg(void) { return -test_yaw; }
float HWT101_GetGyroZDps(void) { return 0.0f; }
uint8_t HWT101_IsZeroDriftReady(void) { return 1U; }
void ZDT_Motor_SetVerbose(uint8_t enabled) { (void)enabled; }
HAL_StatusTypeDef ZDT_Motor_SetSpeedRpmAddr(uint8_t addr,int16_t rpm,uint8_t sync)
{CHECK(addr>=1U && addr<=4U);if(addr==test_speed_busy_id)return HAL_BUSY;if(sync){
 if(test_buffered_seen)CHECK(HAL_GetTick()-test_last_buffered_tick>=3U);
 test_last_buffered_tick=HAL_GetTick();test_buffered_seen=1;
 test_pending_rpm[addr]=rpm;test_pending_mask|=1U<<(addr-1U);
 if(!test_drop_ack)test_ack[addr]=((test_ack[addr]+0x100U)&0xFFFFFF00UL)|2U;
 }else{CHECK(rpm==0);test_zero_attempts[addr]++;
 if(test_zero_drop_mask&(1U<<(addr-1U)))test_zero_drop_mask&=(uint8_t)~(1U<<(addr-1U));else test_motor_rpm[addr]=0;}
 return HAL_OK;}
uint32_t ZDT_Motor_SpeedAck(uint8_t addr){return test_ack[addr];}
HAL_StatusTypeDef ZDT_Motor_EnableAddr(uint8_t addr,uint8_t sync)
{CHECK(addr>=1U && addr<=4U && sync==0);return HAL_OK;}
HAL_StatusTypeDef ZDT_Motor_DisableAddr(uint8_t addr,uint8_t sync)
{CHECK(addr>=1U && addr<=4U && sync==0);wheel_disables++;return disable_result;}
HAL_StatusTypeDef ZDT_Motor_SyncTrigger(void)
{unsigned i;CHECK(test_pending_mask==15U && HAL_GetTick()-test_last_buffered_tick>=3U);
 test_syncs++;for(i=1;i<=4;i++)test_motor_rpm[i]=test_pending_rpm[i];test_pending_mask=0;return HAL_OK;}
HAL_StatusTypeDef ZDT_Motor_StopAddr(uint8_t addr)
{ CHECK(addr>=1U && addr<=4U);++stops;test_stop_attempts[addr]++;
 if(test_stop_busy_mask&(1U<<(addr-1U)))return HAL_BUSY;
 if(test_stop_drop_mask&(1U<<(addr-1U)))test_stop_drop_mask&=(uint8_t)~(1U<<(addr-1U));else test_motor_rpm[addr]=0;
 return HAL_OK; }
void ArmTask_EmergencyStop(void) { ++arm_stops; }

void Test_ChassisFieldLifecycle(void)
{
    const float signs[4]={CHASSIS_WHEEL_SIGN_FL,CHASSIS_WHEEL_SIGN_FR,CHASSIS_WHEEL_SIGN_RL,CHASSIS_WHEEL_SIGN_RR};
    uint32_t now=100U;
    unsigned int i,j;
    int16_t rpm[4];
    Test_SetTick(now); ChassisMotor_Init();
    CHECK(ChassisMotor_Enable()==HAL_OK);CHECK(test_syncs==0 && !test_pending_mask);
    CHECK(ChassisMotor_SetFieldVelocity(0.0f,1.0f,0.2f,0.0f,0.3f)==HAL_OK);
    for(i=0U;i<60U;++i) {
        float w[4],f,r,mx,my;
        now+=4U; Test_SetTick(now); test_yaw+=0.2f;
        CHECK(ChassisMotor_Update()==HAL_OK);
        ChassisMotor_GetWheelRpm(rpm);
        for(j=0U;j<4U;++j) { w[j]=rpm[j]*signs[j]/CHASSIS_MAX_WHEEL_RPM; }
        f=(w[0]+w[1]+w[2]+w[3])*0.25f;
        r=(w[0]-w[1]-w[2]+w[3])*0.25f;
        mx=-f*sinf(test_yaw*0.0174532925f)+r*cosf(test_yaw*0.0174532925f);
        my=f*cosf(test_yaw*0.0174532925f)+r*sinf(test_yaw*0.0174532925f);
        CHECK(mx>=-0.005f && fabsf(my)<0.005f); /* Field east stays east while body turns. */
    }
    {unsigned int stop_count=stops;int16_t before[4];
    ChassisMotor_GetWheelRpm(before);
    CHECK(ChassisMotor_Stop()==HAL_OK);
    CHECK(!ChassisMotor_HasMotionDemand()); /* No new target; the output ramp remains live. */
    ChassisMotor_GetWheelRpm(rpm);
    for(j=0U;j<4U;j++)CHECK(rpm[j]==before[j]); /* No immediate software RPM reset. */
    now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);
    ChassisMotor_GetWheelRpm(rpm);
    {uint8_t moving=0U;for(j=0U;j<4U;j++)if(rpm[j])moving=1U;CHECK(moving);}
    CHECK(stops==stop_count); /* Speed ramp uses sync frames, no driver FE stop. */
    }
    for(i=0U;i<500U;++i) { now+=4U; Test_SetTick(now); CHECK(ChassisMotor_Update()==HAL_OK); }
    ChassisMotor_GetWheelRpm(rpm);
    for(i=0U;i<4U;++i) { CHECK(rpm[i]==0);CHECK(test_motor_rpm[i+1]==0); }
    now+=5000U; Test_SetTick(now); /* Long idle must not produce a 100ms acceleration jump. */
    CHECK(ChassisMotor_SetFieldVelocity(0.0f,1.0f,0.2f,0.0f,0.0f)==HAL_OK);
    CHECK(ChassisMotor_HasMotionDemand()); /* field demand before applied RPM */
    CHECK(ChassisMotor_Update()==HAL_OK);
    ChassisMotor_GetWheelRpm(rpm);
    for(i=0U;i<4U;++i) { CHECK(abs(rpm[i])<=3); }
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
    CHECK(stops==4U && arm_stops==0U);
    ChassisMotor_GetWheelRpm(rpm);
    for(i=0U;i<4U;++i) { CHECK(rpm[i]==0); }
    CHECK(ChassisMotor_EmergencyStop()==HAL_OK);
    CHECK(stops==8U && arm_stops==1U); /* Whole-car emergency semantics preserved. */
    CHECK(ChassisMotor_SetVelocity(0.3f,0.0f,0.0f)==HAL_OK);
    CHECK(ChassisMotor_HasMotionDemand()); /* requested before applied ramp */
    now+=4U; Test_SetTick(now); CHECK(ChassisMotor_Update()==HAL_OK);
    ChassisMotor_GetWheelRpm(rpm);
    for(i=0U;i<4U;++i) { CHECK(abs(rpm[i])==2); } /* Original single-wheel ramp still used. */
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
    CHECK(!ChassisMotor_HasMotionDemand());
    CHECK(ChassisMotor_SetFieldVelocity(0.0f,1.0f,0.2f,0.0f,0.2f)==HAL_OK);
    now+=4U; Test_SetTick(now); CHECK(ChassisMotor_Update()==HAL_OK);
    test_yaw+=90.0f; now+=4U; Test_SetTick(now);
    CHECK(ChassisMotor_Update()==HAL_ERROR); /* IMU jumps must stop, not rotate a live field vector. */
    ChassisMotor_GetWheelRpm(rpm);
    for(i=0U;i<4U;++i) { CHECK(rpm[i]==0); }
    /* Relative LEFT +90 must be physically CCW with the original front.
     * Exercise the production turn controller/mixer, not the test field model. */
    test_yaw=0.0f;
    CHECK(ChassisMotor_TurnRelativeFast(90)==HAL_OK);
    for(i=0U;i<50U;++i) { now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK); }
    ChassisMotor_GetWheelRpm(rpm);
    {
        float w[4],f,r,o;
        for(i=0U;i<4U;++i)w[i]=rpm[i]*signs[i];
        f=(w[0]+w[1]+w[2]+w[3])*0.25f;
        r=(w[0]-w[1]-w[2]+w[3])*0.25f;
        o=(-w[0]+w[1]-w[2]+w[3])*0.25f;
        CHECK(o>0.0f && fabsf(f)<0.01f && fabsf(r)<0.01f);
        CHECK(w[0]<0 && w[1]>0 && w[2]<0 && w[3]>0);
        for(i=1U;i<=4U;++i)CHECK(test_motor_rpm[i]>0);
    }
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
    {
        /* Field-trial reversal changes forward/right at physical IDs 1,2,3,4,
         * while clockwise/counter-clockwise commands keep the original signs. */
        static const float commands[6][3]={
            {0.2f,0.0f,0.0f},{-0.2f,0.0f,0.0f},
            {0.0f,0.2f,0.0f},{0.0f,-0.2f,0.0f},
            {0.0f,0.0f,0.2f},{0.0f,0.0f,-0.2f}
        };
        static const int16_t physical_rpm[6][4]={
#if CHASSIS_FRONT_REVERSED_TRIAL
            {50,50,-50,-50},{-50,-50,50,50},
            {50,-50,50,-50},{-50,50,-50,50},
#else
            {-50,-50,50,50},{50,50,-50,-50},
            {-50,50,-50,50},{50,-50,50,-50},
#endif
            {50,50,50,50},{-50,-50,-50,-50}
        };
        unsigned int case_index;
        for(case_index=0U;case_index<6U;++case_index) {
            CHECK(ChassisMotor_SetVelocity(commands[case_index][0],
                  commands[case_index][1],commands[case_index][2])==HAL_OK);
            for(i=0U;i<40U;++i) {now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
            for(i=0U;i<4U;++i)CHECK(test_motor_rpm[i+1U]==physical_rpm[case_index][i]);
            CHECK(ChassisMotor_StopImmediate()==HAL_OK);
        }
    }
}

void Test_CardinalTurnTransfer(void)
{
    uint32_t now=80000;unsigned i,j;int16_t before[4],after[4];chassis_telemetry_t t;
    Test_SetTick(now);test_yaw=13.25f;ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);
    CHECK(ChassisMotor_AlignHeadingReference(90)==HAL_OK);ChassisMotor_GetTelemetry(&t);
    CHECK(fabsf(t.heading_deg-90)<.001f && test_yaw==13.25f); /* No sensor zero/bias change. */
    CHECK(ChassisMotor_TurnToHeading(180.25f,1)==HAL_OK && fabsf(ChassisMotor_GetTurnTargetDeg()-180.25f)<.001f);
    for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
    ChassisMotor_GetWheelRpm(before);CHECK(abs(before[0])>20);
    CHECK(ChassisMotor_SetFieldVelocity(0,1,0,.65f,0)==HAL_ERROR); /* Ordinary entry remains stopped-only. */
    ChassisMotor_ReleaseTurnForDrive();CHECK(!ChassisMotor_IsTurnActive());
    CHECK(ChassisMotor_ContinueFieldVelocity(0,1,0,.65f,0)==HAL_OK);
    ChassisMotor_GetWheelRpm(after);for(j=0;j<4;j++)CHECK(after[j]==before[j]);
    now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(after);
    for(j=0;j<4;j++)CHECK(abs(after[j]-before[j])<=8); /* Seed the real wheel vector; no zero jump. */
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
    CHECK(ChassisMotor_TurnRelativeFast(90)==HAL_OK && fabsf(ChassisMotor_GetTurnTargetDeg()-180)<.001f);
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
}

void Test_FieldWheelLimits(void)
{
    uint32_t now=50000U;unsigned int i,j;int16_t rpm[4];chassis_telemetry_t telemetry;
    const float signs[4]={CHASSIS_WHEEL_SIGN_FL,CHASSIS_WHEEL_SIGN_FR,CHASSIS_WHEEL_SIGN_RL,CHASSIS_WHEEL_SIGN_RR};
    Test_SetTick(now);test_yaw=0;ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);
    CHECK(ChassisMotor_SetFieldVelocity(0,1,0,0.65f,0)==HAL_OK);
    for(i=0;i<500U;i++){now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
    ChassisMotor_GetWheelRpm(rpm);ChassisMotor_GetTelemetry(&telemetry);
    for(j=0;j<4U;j++)CHECK(rpm[j]*signs[j]>=162 && rpm[j]*signs[j]<=163);
    CHECK(fabsf(telemetry.vx-0.65f)<0.005f); /* Cruise and telemetry units stay unchanged. */
    CHECK(ChassisMotor_SetFieldVelocity(0,1,0,0.65f,1.15f)==HAL_OK);
    for(i=0;i<500U;i++) {
        now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(rpm);
        for(j=0;j<4U;j++)CHECK(abs(rpm[j])<=500);
    }
    CHECK(fabsf(rpm[0]*signs[0]+125.0f)<=1 && fabsf(rpm[1]*signs[1]-450.0f)<=1);
    CHECK(fabsf(rpm[2]*signs[2]+125.0f)<=1 && fabsf(rpm[3]*signs[3]-450.0f)<=1);
    CHECK(ChassisMotor_SetFieldVelocity(0,1,0,1,2)==HAL_OK);
    for(i=0;i<750U;i++) {
        now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(rpm);
        for(j=0;j<4U;j++)CHECK(abs(rpm[j])<=500);
    }
    CHECK(abs(rpm[1])>=499 && abs(rpm[3])>=499); /* Real wire outputs can reach500, not250. */
    CHECK(ChassisMotor_StopImmediate()==HAL_OK);
    /* Runtime parameters reach the actual controller, not just its report. */
    {nav_motion_parameters_t saved=*Nav_Motion();
     CHECK(Nav_Motion_ProcessCommand("motion set accel 300") && Nav_Motion_ProcessCommand("motion set wheel 400"));
     ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);now+=100;Test_SetTick(now);
     CHECK(ChassisMotor_SetFieldVelocity(0,1,0,.2f,0)==HAL_OK && ChassisMotor_Update()==HAL_OK);
     ChassisMotor_GetWheelRpm(rpm);CHECK(abs(rpm[0])==1);
     CHECK(ChassisMotor_SetFieldVelocity(0,1,0,1,2)==HAL_OK);
     for(i=0;i<1000U;i++){now+=4U;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(rpm);for(j=0;j<4U;j++)CHECK(abs(rpm[j])<=400);}
     CHECK(abs(rpm[1])>=399 && abs(rpm[3])>=399);CHECK(ChassisMotor_StopImmediate()==HAL_OK);*Nav_Motion()=saved;
    }
}

void Test_PostAlignShortMoves(void)
{
    {
        unsigned int before=arm_stops,disables=wheel_disables;
        CHECK(ChassisMotor_Enable()==HAL_OK);
        CHECK(ChassisMotor_SetVelocity(0.2f,0.0f,0.0f)==HAL_OK && ChassisMotor_HasMotionDemand());
        CHECK(ChassisMotor_DisableForArmPreparation()==HAL_OK);
        CHECK(!ChassisMotor_IsEnabled() && !ChassisMotor_HasMotionDemand() && arm_stops==before && wheel_disables==disables+4U);
        CHECK(ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP)==HAL_OK);
        Test_SetTick(45000U);
        CHECK(ChassisMotor_Update()==HAL_OK); /* Stop hold can retry while wheels are intentionally disabled. */
        CHECK(ChassisMotor_Enable()==HAL_OK);disable_result=HAL_BUSY;
        CHECK(ChassisMotor_DisableForArmPreparation()==HAL_BUSY && arm_stops==before);
        disable_result=HAL_OK;CHECK(ChassisMotor_DisableForArmPreparation()==HAL_OK && arm_stops==before);
        CHECK(ChassisMotor_Enable()==HAL_OK);
    }
    static const float distances[]={1.0f,5.0f,17.0f,50.0f,68.0f,100.0f,150.0f};
    const float signs[4]={CHASSIS_WHEEL_SIGN_FL,CHASSIS_WHEEL_SIGN_FR,
        CHASSIS_WHEEL_SIGN_RL,CHASSIS_WHEEL_SIGN_RR};
    unsigned int j,axis,i;float scale;uint32_t hold,elapsed,now;
    for(j=0U;j<sizeof(distances)/sizeof(distances[0]);j++)for(axis=0U;axis<4U;axis++) {
        float sum=0.0f;uint8_t stopped=0U;int16_t rpm[4];
        double encoder_rev[4]={0.0};float expected[4],spread,progress;
        float direction=(axis&1U)?-1.0f:1.0f;
        CHECK(NavShortMove_Plan(distances[j],axis&1U?0.35f:0.30f,CHASSIS_MAX_WHEEL_RPM,
            ZDT_MILEAGE_MM_PER_REV,CHASSIS_ACCEL_RPM_PER_S,CHASSIS_DECEL_RPM_PER_S,
            CHASSIS_TX_PERIOD_MS,&scale,&hold));
        CHECK(scale>0.0f && scale<=NAV_SHORT_MOVE_MAX_SCALE && hold>0U);
        CHECK(ChassisMotor_StopImmediate()==HAL_OK);
        now=50000U+j*16000U+axis*4000U;Test_SetTick(now);
        ChassisMotor_ResetIdleRamp();
        CHECK(ChassisMotor_SetVelocity(axis<2U?direction*scale:0.0f,
            axis<2U?0.0f:direction*scale,0.0f)==HAL_OK);
        for(elapsed=4U;elapsed<3000U;elapsed+=4U) {
            float w[4],v;uint8_t zero=1U;
            if(!stopped && elapsed>=hold){CHECK(ChassisMotor_Stop()==HAL_OK);stopped=1U;}
            Test_SetTick(now+elapsed);CHECK(ChassisMotor_Update()==HAL_OK);
            ChassisMotor_GetWheelRpm(rpm);
            for(i=0U;i<4U;i++) {
                w[i]=rpm[i]*signs[i];if(rpm[i])zero=0U;
                encoder_rev[i]+=(double)w[i]*0.004/60.0;
            }
            if(elapsed==4U)for(i=0U;i<4U;i++)CHECK(abs(rpm[i])<=2); /* idle restart */
            v=axis<2U?(w[0]+w[1]+w[2]+w[3])*0.25f:
                (w[0]-w[1]-w[2]+w[3])*0.25f;
            sum+=v*(ZDT_MILEAGE_MM_PER_REV/60.0f)*0.004f;
            if(stopped && zero)break;
        }
        CHECK(elapsed<3000U);
        CHECK(fabsf(sum-direction*distances[j])<2.0f); /* actual production ramp */
        NavFrame_VelocityWheelValues(axis<2U?(int8_t)direction:1,
            axis<2U?0:(int8_t)direction,expected);
        progress=NavFeedback_ProjectWheels(encoder_rev,expected,15U,&spread)*ZDT_MILEAGE_MM_PER_REV;
        CHECK(progress>0.0f && fabsf(progress-distances[j])<2.0f);
        CHECK(NavFeedback_Check(distances[j],progress)==NAV_MOTION_OK);
    }
    scale=0.3f;hold=12345U;
    CHECK(!NavShortMove_Plan(151.0f,0.3f,250.0f,255.0f,600.0f,292.0f,4U,&scale,&hold));
    CHECK(scale==0.3f && hold==12345U); /* long transport is unaffected */
    CHECK(!NavShortMove_Plan(NAN,0.3f,250.0f,255.0f,600.0f,292.0f,4U,&scale,&hold));
    CHECK(!NavShortMove_Plan(50.0f,0.0f,250.0f,255.0f,600.0f,292.0f,4U,&scale,&hold));
    CHECK(!NavShortMove_Plan(50.0f,0.3f,250.0f,255.0f,600.0f,292.0f,0U,&scale,&hold));
}

void Test_LegacySyncAckPacing(void)
{
 uint32_t now=1000000,before;unsigned i;
 Test_SetTick(now);test_yaw=0;ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);
 before=test_syncs;test_drop_ack=1;
 CHECK(ChassisMotor_SetVelocity(.2f,0,0)==HAL_OK);
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs>before); /* Missing optional ACK must not block the timed batch. */
 test_drop_ack=0;
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs>before);
 /* Transport BUSY is different: no broadcast until all targets enqueue. */
 CHECK(ChassisMotor_StopImmediate()==HAL_OK);before=test_syncs;test_speed_busy_id=CHASSIS_MOTOR_FL_ADDRESS;
 CHECK(ChassisMotor_SetVelocity(.2f,0,0)==HAL_OK);
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs==before);test_speed_busy_id=0;
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs>before);
 /* STOP cancels the partly staged batch and cannot trigger it later. */
 CHECK(ChassisMotor_StopImmediate()==HAL_OK);before=test_syncs;
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs==before);
 /* A normal ramped stop also flushes zero once and becomes completely idle. */
 CHECK(ChassisMotor_SetVelocity(.2f,0,0)==HAL_OK);test_drop_ack=1;
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(ChassisMotor_Stop()==HAL_OK);
 for(i=0;i<500;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 before=test_syncs;
 for(i=1;i<=4;i++)CHECK(test_motor_rpm[i]==0);
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_syncs==before);test_drop_ack=0;
}

void Test_StopDeliveryRetry(void)
{
 uint32_t now=2000000,sync_before,old_stop;unsigned wheel,i;int16_t rpm[4];
 test_drop_ack=test_speed_busy_id=0;
 for(wheel=1;wheel<=4;wheel++) {
  Test_SetTick(now);ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);
  CHECK(ChassisMotor_SetVelocity(.2f,0,0)==HAL_OK);
  for(i=0;i<50;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
  CHECK(test_motor_rpm[wheel]!=0);sync_before=test_syncs;old_stop=test_stop_attempts[wheel];
  test_stop_busy_mask=(uint8_t)(1U<<(wheel-1U));test_speed_busy_id=(uint8_t)wheel;
  CHECK(ChassisMotor_StopImmediate()==HAL_BUSY);ChassisMotor_GetWheelRpm(rpm);
  for(i=0;i<4;i++)CHECK(rpm[i]==0); /* Software zeros are NOT physical arrival. */
  for(i=0;i<80;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
  CHECK(test_motor_rpm[wheel]!=0 && test_stop_attempts[wheel]>old_stop+1 && test_syncs==sync_before);
  for(i=1;i<=4;i++)if(i!=wheel)CHECK(test_motor_rpm[i]==0);
  test_stop_busy_mask=test_speed_busy_id=0;
  for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
  for(i=1;i<=4;i++)CHECK(test_motor_rpm[i]==0);
  CHECK(test_syncs==sync_before);
  CHECK(ChassisMotor_SetVelocity(.2f,0,0)==HAL_OK);old_stop=stops;
  now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(rpm);
  for(i=0;i<4;i++)CHECK(abs(rpm[i])<=Nav_Motion()->accel_rpm_s*.004f+1.0f);
  for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
  CHECK(stops==old_stop && test_syncs>sync_before && test_motor_rpm[wheel]!=0);
 }
 /* HAL_OK can still mean a frame was dropped on the actual bus. Both FE and
  * the first zero disappear here; later hold cycles must physically stop it. */
 test_stop_drop_mask=test_zero_drop_mask=1U;
 CHECK(ChassisMotor_StopImmediate()==HAL_OK);CHECK(test_motor_rpm[1]!=0);
 sync_before=test_syncs;
 for(i=0;i<70;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_motor_rpm[1]==0 && test_syncs==sync_before && test_zero_attempts[1]>1);
 test_motor_rpm[1]=60;CHECK(ChassisMotor_SetVelocity(0,0,0)==HAL_OK);
 for(i=0;i<40;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_motor_rpm[1]==0); /* A late old target is cleared during wait. */
 now=UINT32_MAX-5U;Test_SetTick(now);ChassisMotor_Init();CHECK(ChassisMotor_Enable()==HAL_OK);test_motor_rpm[1]=60;
 test_stop_drop_mask=test_zero_drop_mask=1U;CHECK(ChassisMotor_StopImmediate()==HAL_OK);
 for(i=0;i<50;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
 CHECK(test_motor_rpm[1]==0);
 test_stop_busy_mask=test_stop_drop_mask=test_zero_drop_mask=test_speed_busy_id=0;
}
