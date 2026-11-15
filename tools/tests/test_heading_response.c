/* Ideal yaw response using production correction and actual wheel ramps. */
#include "chassis_motor.h"
#include "nav_params.h"
#include "zdt_motor.h"
#include <math.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_SetTick(uint32_t tick);
extern void Test_SetChassisYaw(float yaw);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
#include "odo_heading_under_test.inc"
uint32_t heading_response_ms[4];
void Test_HeadingResponse(void)
{
    const float signs[4]={CHASSIS_WHEEL_SIGN_FL,CHASSIS_WHEEL_SIGN_FR,CHASSIS_WHEEL_SIGN_RL,CHASSIS_WHEEL_SIGN_RR};
    unsigned scenario,i,j;uint32_t now=100000;
    for(scenario=0;scenario<4;scenario++) {
        float yaw=0,scale=scenario<2?0.65f:0.10f;uint32_t elapsed,stable=0;
        int16_t rpm[4];
        Nav_Params_Reset();Test_SetTick(now);Test_SetChassisYaw(0);ChassisMotor_Init();
        CHECK(ChassisMotor_Enable()==HAL_OK && ChassisMotor_SetVelocity(scale,0,0)==HAL_OK);
        for(i=0;i<250;i++){now+=4;Test_SetTick(now);CHECK(ChassisMotor_Update()==HAL_OK);}
        yaw=(scenario&1)?-10.0f:10.0f;
        for(elapsed=4;elapsed<=2000;elapsed+=4) {
            float w[4],spin;now+=4;Test_SetTick(now);Test_SetChassisYaw(yaw);
            CHECK(ChassisMotor_SetVelocity(scale,0,HeadingCorrection(-yaw))==HAL_OK);
            CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetWheelRpm(rpm);
            for(j=0;j<4;j++)w[j]=rpm[j]*signs[j];
            spin=(-w[0]+w[1]-w[2]+w[3])*0.25f;
            yaw+=spin*ZDT_MILEAGE_MM_PER_REV/60.0f/CHASSIS_SPIN_ARM_MM*57.2957795f*.004f;
            if(fabsf(yaw)<=ZDT_NAV_HOLD_DEADBAND_DEG)stable+=4;else stable=0;
            if(stable>=100)break;
        }
        heading_response_ms[scenario]=elapsed;
        /* V265 deliberately lowers the correction cap to0.05. Allow1.5s
         * to enter deadband, then independently observe100ms stability. */
        CHECK(elapsed>=100 && elapsed-100<=1500 && stable>=100);
        CHECK(ChassisMotor_StopImmediate()==HAL_OK);now+=3000;
    }
}
