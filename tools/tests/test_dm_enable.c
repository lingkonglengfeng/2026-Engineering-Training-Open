#include "dm_motor.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do {++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
#define DM_FB_STATE_DISABLED 0U
#define DM_FB_STATE_ENABLED 1U
#define DM_ENABLE_QUERY_TIMEOUT_MS 300U
#define DM_ENABLE_CONFIRM_TIMEOUT_MS 1000U
#define DM_ENABLE_HOLD_QUERY_MS 20U
#define DM_ENABLE_HOLD_VEL_RAD 0.10f
static dm_motor_t motor;
static uint32_t tick,fc_tick,reply_at,fc_count,fd_count,pos_count,last_pos_tick;
static uint32_t activation_delay,reply_delay,stop_at;
static uint8_t pending,reply_code,no_confirmation,fault_reply,tx_busy,bad_position,fc_immediate;
static float held_position;
uint32_t HAL_GetTick(void){return tick;}
osStatus_t osDelay(uint32_t ms)
{
    tick+=ms;
    if(stop_at && (uint32_t)(tick-fc_tick)>=stop_at && fc_count){
        motor.enable_pending=0U;motor.stop_generation++;motor.state=DM_STATE_FAULT;
        stop_at=0U;
    }
    if(pending && (int32_t)(tick-reply_at)>=0){
        pending=0U;motor.fb.fb_count++;motor.fb.last_tick=tick;
        motor.fb.err_code=reply_code;motor.fb.enabled=(uint8_t)(reply_code==1U);
        motor.fb.pos_rad=(bad_position && fc_count)?NAN:held_position;
        motor.fb.vel_rads=0.007f;
    }
    return osOK;
}
static void DM_Print(const char *line){CHECK(line!=NULL);}
static void DM_DrainRxBeforeEnable(dm_motor_t *m){CHECK(m==&motor);}
static HAL_StatusTypeDef DM_SendFrame(dm_motor_t *m,uint16_t id,const uint8_t *data,uint8_t len)
{
    uint8_t special=1U;unsigned int i;
    CHECK(m==&motor && id==0x101U && len==8U);
    for(i=0U;i<7U;i++)if(data[i]!=0xFFU)special=0U;
    if(special){
        CHECK(data[7]==0xFDU || data[7]==0xFCU); /* no FE zero writes */
        if(data[7]==0xFCU){
            fc_count++;fc_tick=tick;
            if(fc_immediate){motor.fb.fb_count++;motor.fb.err_code=1U;motor.fb.enabled=1U;motor.fb.last_tick=tick;}
            return HAL_OK;
        }
        fd_count++;pending=1U;reply_code=0U;reply_at=tick+2U;return HAL_OK;
    }
    {
        float p,v;memcpy(&p,data,4U);memcpy(&v,data+4,4U);
        CHECK(p==held_position && p==motor.target_pos_rad && v==0.10f);
        if(pos_count)CHECK((uint32_t)(tick-last_pos_tick)>=20U);
        pos_count++;last_pos_tick=tick;
        if(tx_busy)return HAL_BUSY;
        if(!no_confirmation && !pending){
            reply_code=fault_reply?9U:((uint32_t)(tick-fc_tick)>=activation_delay?1U:0U);
            pending=1U;reply_at=tick+reply_delay;
        }
    }
    return HAL_OK;
}
static uint32_t wait_service_calls;
void DM_WaitService(dm_motor_t *m){CHECK(m==&motor);wait_service_calls++;}
#include "dm_zero_under_test.inc"
static void reset(void)
{
    memset(&motor,0,sizeof(motor));motor.active_mode=DM_POS_MODE;
    motor.cfg.esc_id=1U;motor.cfg.pos_limit_rad=18.5f;motor.state=DM_STATE_READY;
    motor.fb.fb_count=7U;motor.fb.err_code=1U;motor.fb.enabled=1U; /* stale enabled cache */
    motor.pos_offset_rad=0.123f;tick=10000U;held_position=2.187f;
    fc_tick=reply_at=fc_count=fd_count=pos_count=last_pos_tick=0U;
    activation_delay=40U;reply_delay=2U;stop_at=0U;
    pending=reply_code=no_confirmation=fault_reply=tx_busy=bad_position=fc_immediate=0U;
}
int main(void)
{
    dm_feedback_t fb;
    reset();CHECK(DM_Motor_Enable(&motor)==HAL_OK);
    CHECK(fc_count==1U && fd_count==1U && pos_count==3U);
    CHECK(motor.state==DM_STATE_ENABLED && motor.enable_sent && !motor.enable_pending);
    CHECK(motor.target_pos_rad==held_position && motor.pos_offset_rad==0.123f && motor.resend_enable);
    reset();fc_immediate=1U;CHECK(DM_Motor_Enable(&motor)==HAL_OK && pos_count==1U);
    reset();activation_delay=600U;CHECK(DM_Motor_Enable(&motor)==HAL_OK && pos_count>=31U);
    reset();activation_delay=0U;reply_delay=1000U;
    CHECK(DM_Motor_Enable(&motor)==HAL_OK); /* reply at deadline must be examined */
    reset();no_confirmation=1U;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR);
    CHECK(fc_count==1U && fd_count==2U && motor.state==DM_STATE_FAULT && motor.fault_latched);
    CHECK(!motor.enable_sent && !motor.enable_pending); /* stale enabled cache never confirms */
    reset();fault_reply=1U;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR && pos_count==1U);
    CHECK(motor.fault_latched && fc_count==1U);
    reset();stop_at=10U;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR);
    CHECK(fd_count==1U && fc_count==1U && pos_count==1U && motor.state==DM_STATE_FAULT);
    reset();tx_busy=1U;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR && fd_count==2U);
    reset();bad_position=1U;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR && motor.fault_latched);
    reset();motor.enable_pending=1U;CHECK(DM_Motor_Enable(&motor)==HAL_BUSY && !fc_count && !fd_count);
    reset();motor.active_mode=DM_MIT_MODE;CHECK(DM_Motor_Enable(&motor)==HAL_ERROR && !fc_count);
    reset();tick=UINT32_MAX-20U;CHECK(DM_Motor_Enable(&motor)==HAL_OK);
    /* Generic disabled-query waiter has the same exact-deadline rule. */
    reset();motor.enable_pending=1U;pending=1U;reply_code=0U;reply_at=tick+30U;
    CHECK(DM_EnableWaitFeedback(&motor,motor.fb.fb_count,30U,0U,&fb)==1U && fb.last_tick==tick);
    CHECK(wait_service_calls>500U);
    return test_failures?1:0;
}
