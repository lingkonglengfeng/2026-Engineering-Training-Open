#include "dm_motor.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do {++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
#define DM_FB_STATE_DISABLED 0U
#define DM_FB_STATE_ENABLED 1U
static dm_motor_t motor;
static uint32_t tick,reply_at,fd_count,fe_count,pos_count,fb_count;
static uint8_t reply_pending,respond,reply_code,cancel_query,zero_applied,stop_after_reply;
static float raw_position,raw_velocity;
static HAL_StatusTypeDef tx_result;
uint32_t HAL_GetTick(void){return tick;}
osStatus_t osDelay(uint32_t ms)
{
    tick+=ms;
    if(cancel_query){motor.enable_pending=0U;cancel_query=0U;}
    if(reply_pending && (int32_t)(tick-reply_at)>=0) {
        reply_pending=0U;motor.fb.fb_count++;motor.fb.last_tick=tick;
        motor.fb.err_code=reply_code;motor.fb.enabled=(uint8_t)(reply_code==1U);
        motor.fb.pos_rad=raw_position;motor.fb.vel_rads=raw_velocity;
        if(stop_after_reply){motor.stop_generation++;stop_after_reply=0U;}
    }
    return osOK;
}
static void DM_DrainRxBeforeEnable(dm_motor_t *m){CHECK(m==&motor);}
static HAL_StatusTypeDef DM_SendFrame(dm_motor_t *m,uint16_t id,const uint8_t *data,uint8_t len)
{
    unsigned int i;CHECK(m==&motor && id==0x101U && len==8U);
    uint8_t special=1U;for(i=0U;i<7U;i++)if(data[i]!=0xFFU)special=0U;
    if(!special) {
        float p,v;memcpy(&p,data,4U);memcpy(&v,data+4,4U);
        CHECK(p==motor.target_pos_rad && v==motor.target_vel_rads);pos_count++;
        if(respond && tx_result==HAL_OK){reply_pending=1U;reply_at=tick+2U;}
        return tx_result;
    }
    for(i=0U;i<7U;i++)CHECK(data[i]==0xFFU);
    CHECK(data[7]==0xFDU || data[7]==0xFEU || data[7]==0xFBU); /* never FC / movement */
    if(data[7]==0xFDU){fd_count++;if(respond && tx_result==HAL_OK){reply_pending=1U;reply_at=tick+2U;}}
    if(data[7]==0xFBU)fb_count++;
    if(data[7]==0xFEU){fe_count++;if(zero_applied && tx_result==HAL_OK)raw_position=0.001f;}
    return tx_result;
}
/* Production functions extracted verbatim; mocks replace only task clock,
 * raw CAN TX and the FIFO drain. Feedback still passes the actual waiter. */
void DM_WaitService(dm_motor_t *m){(void)m;}
#include "dm_zero_under_test.inc"
static void reset(void)
{
    memset(&motor,0,sizeof(motor));motor.active_mode=DM_POS_MODE;motor.cfg.esc_id=1U;
    motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.link_timeout_ms=500U;motor.state=DM_STATE_READY;
    motor.cfg.vel_limit_rad=DM_V_MAX;
    tick=10000U;motor.fb.fb_count=1U;motor.fb.last_tick=0U;motor.fb.pos_rad=1.0f;
    raw_position=0.5f;raw_velocity=0.007f;respond=1U;reply_code=0U;tx_result=HAL_OK;
    reply_pending=cancel_query=zero_applied=stop_after_reply=0U;fd_count=fe_count=pos_count=fb_count=0U;
}
int main(void)
{
    dm_feedback_t fb;uint32_t before;
    reset();CHECK(DM_Motor_QueryDisabled(&motor,30U,&fb)==HAL_OK);
    CHECK(fb.fb_count==2U && fb.last_tick==tick && fb.pos_rad==0.5f);
    CHECK(!motor.enable_pending && !motor.enable_sent && !fe_count && fd_count==1U);
    CHECK(DM_Motor_SavePosZero(&motor)==HAL_OK && fe_count==1U);
    zero_applied=1U;CHECK(DM_Motor_SavePosZero(&motor)==HAL_OK && fe_count==2U);
    before=fe_count;CHECK(DM_Motor_VerifyPosZero(&motor,100U)==HAL_OK);
    CHECK(fe_count==before && motor.fb.pos_rad==0.001f); /* verification only FD */
    reset();respond=0U;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_TIMEOUT);
    CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count); /* old cached feedback unusable */
    reset();reply_code=4U;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_ERROR);
    CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count);
    reset();reply_code=1U;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_TIMEOUT);
    CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count);
    reset();cancel_query=1U;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_ERROR);
    CHECK(!motor.enable_pending && !fe_count);
    reset();tx_result=HAL_BUSY;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_BUSY);
    CHECK(!motor.enable_pending && !motor.resend_hold && !fe_count);
    reset();motor.enable_pending=1U;CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_BUSY && !fd_count);
    reset();CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_OK);
    motor.fb.vel_rads=0.1f;CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count);
    motor.fb.vel_rads=0.0f;motor.enable_sent=1U;CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count);
    motor.enable_sent=0U;motor.fb.pos_rad=NAN;CHECK(DM_Motor_SavePosZero(&motor)==HAL_ERROR && !fe_count);
    reset();CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_OK);
    CHECK(DM_Motor_SavePosZero(&motor)==HAL_OK);before=fe_count;
    CHECK(DM_Motor_VerifyPosZero(&motor,80U)==HAL_TIMEOUT && fe_count==before);
    CHECK(!motor.enable_pending && !motor.enable_sent);
    reset();tick=UINT32_MAX-1U;CHECK(DM_Motor_QueryDisabled(&motor,30U,&fb)==HAL_OK && tick==0U);
    CHECK(DM_Motor_QueryDisabled(&motor,0U,NULL)==HAL_ERROR);
    CHECK(DM_Motor_QueryDisabled(NULL,30U,NULL)==HAL_ERROR);
    motor.active_mode=DM_MIT_MODE;before=fd_count;
    CHECK(DM_Motor_QueryDisabled(&motor,30U,NULL)==HAL_ERROR && fd_count==before);
    /* Fresh query must not stop/retarget an enabled motor. */
    reset();motor.state=DM_STATE_ENABLED;motor.enable_sent=1U;reply_code=1U;
    motor.target_pos_rad=1.25f;motor.target_vel_rads=0.3f;
    CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_OK && fb.pos_rad==0.5f);
    CHECK(pos_count==1U && fd_count==0U && fe_count==0U && motor.enable_sent==1U);
    CHECK(motor.state==DM_STATE_ENABLED && motor.target_pos_rad==1.25f && motor.target_vel_rads==0.3f);
    reset();CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_OK && fd_count==1U && pos_count==0U && fe_count==0U);
    reset();reply_code=4U;CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_OK && fb.err_code==4U); /* readable fault feedback */
    reset();respond=0U;CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_TIMEOUT && !motor.enable_pending);
    reset();cancel_query=1U;CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_ERROR && !motor.enable_pending);
    reset();motor.state=DM_STATE_ENABLED;motor.enable_sent=1U;motor.target_pos_rad=NAN;motor.target_vel_rads=0.1f;
    CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_ERROR && !pos_count && !fd_count && !motor.enable_pending);
    reset();motor.fb.enabled=1U;
    CHECK(DM_Motor_ReadPosition(&motor,30U,&fb)==HAL_BUSY && !fd_count && !pos_count);
    /* Explicit clear must leave a healthy axis OFF/READY, not stuck FAULT. */
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;
    CHECK(DM_Motor_ClearError(&motor)==HAL_OK);
    CHECK(motor.state==DM_STATE_READY && !motor.fault_latched && !motor.enable_sent && !motor.fb.enabled);
    CHECK(fb_count==1U && fd_count==1U && fe_count==0U && pos_count==0U);
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;respond=0U;
    CHECK(DM_Motor_ClearError(&motor)==HAL_TIMEOUT && motor.state==DM_STATE_FAULT && motor.fault_latched);
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;tx_result=HAL_BUSY;
    CHECK(DM_Motor_ClearError(&motor)==HAL_BUSY && motor.fault_latched && motor.state==DM_STATE_FAULT && !fd_count);
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;reply_code=9U;
    CHECK(DM_Motor_ClearError(&motor)==HAL_ERROR && motor.fault_latched && motor.state==DM_STATE_FAULT);
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;cancel_query=1U;
    CHECK(DM_Motor_ClearError(&motor)==HAL_ERROR && motor.fault_latched);
    reset();motor.state=DM_STATE_FAULT;motor.fault_latched=1U;stop_after_reply=1U;
    CHECK(DM_Motor_ClearError(&motor)==HAL_ERROR && motor.fault_latched && motor.state==DM_STATE_FAULT);
    CHECK(DM_Motor_ClearError(NULL)==HAL_ERROR);
    return test_failures?1:0;
}
