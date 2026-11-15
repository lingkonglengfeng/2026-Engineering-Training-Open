#include "vision_yaw.h"
#include <math.h>
#include <string.h>
static float yaw_error(float v)
{v=fmodf(v+180.0f,360.0f);if(v<0.0f)v+=360.0f;return v-180.0f;}
float VisionYaw_LineError(float v)
{v=fmodf(v+90.0f,180.0f);if(v<0.0f)v+=180.0f;return v-90.0f;}
static uint8_t valid(const vision_ring_axis_t *a,uint32_t age)
{return a && a->valid && a->id==2U && a->station>=2U && a->station<=3U && a->landmarks==3U &&
    a->confidence>=80U && a->confidence<=100U && a->observation_id && age<V_YAW_FRESH_MS &&
    isfinite(a->axis_deg) && a->axis_deg>=-90.0f && a->axis_deg<90.0f;}
static vision_yaw_cal_t *cal(vision_yaw_t *v,uint8_t station)
{return station>=2U && station<=3U?&v->cal[station-2U]:0;}
void VisionYaw_Init(vision_yaw_t *v){memset(v,0,sizeof(*v));}
void VisionYaw_UseInstallationDefaults(vision_yaw_t *v)
{
    uint8_t i;VisionYaw_Init(v);
    for(i=0U;i<2U;i++){v->cal[i].zero=0.0f;v->cal[i].gain=1.0f;v->cal[i].zero_valid=v->cal[i].gain_valid=1U;}
    v->enabled=v->simultaneous=1U;
}
void VisionYaw_Cancel(vision_yaw_t *v)
{v->phase=V_YAW_IDLE;v->turn_allowed=v->restart_xy=0U;v->reason=0;}
void VisionYaw_Observe(vision_yaw_t *v,const vision_ring_axis_t *a,uint32_t age,uint32_t now,uint8_t stopped)
{
    vision_yaw_cal_t *c;uint32_t received=now-age;
    if(!valid(a,age)){v->cal[0].stable=v->cal[1].stable=0U;return;}
    c=cal(v,a->station);
    if(!stopped){c->stable=0U;return;}
    if(c->observation==a->observation_id)return;
    if(c->request_seq!=a->request_seq || received-c->received>=V_YAW_FRESH_MS ||
       fabsf(VisionYaw_LineError(a->axis_deg-c->mean))>0.3f)c->stable=0U;
    if(c->request_seq!=a->request_seq)c->sample_valid=0U;
    c->request_seq=a->request_seq;c->observation=a->observation_id;c->received=received;
    if(!c->stable){c->mean=a->axis_deg;c->stable_tick=received;}
    if(c->stable<200U)c->stable++;
    c->mean=VisionYaw_LineError(c->mean+VisionYaw_LineError(a->axis_deg-c->mean)/c->stable);
}
static uint8_t teachable(const vision_yaw_cal_t *c,uint32_t now)
{return c && c->stable>=3U && now-c->received<V_YAW_FRESH_MS && c->received-c->stable_tick>=300U;}
uint8_t VisionYaw_Zero(vision_yaw_t *v,uint8_t s,uint32_t now)
{
    vision_yaw_cal_t *c=cal(v,s);if(!teachable(c,now))return 0U;
    c->zero=c->mean;c->zero_valid=1U;c->sample_valid=0U;return 1U;
}
uint8_t VisionYaw_Gain(vision_yaw_t *v,uint8_t s,float gain)
{
    vision_yaw_cal_t *c=cal(v,s);if(!c || !isfinite(gain) || fabsf(gain)<0.25f || fabsf(gain)>4.0f)return 0U;
    c->gain=gain;c->gain_valid=1U;return 1U;
}
uint8_t VisionYaw_Sample(vision_yaw_t *v,uint8_t s,uint32_t now,float h)
{
    vision_yaw_cal_t *c=cal(v,s);if(!teachable(c,now) || !c->zero_valid || !isfinite(h))return 0U;
    c->sample_axis=c->mean;c->sample_heading=h;c->sample_request=c->request_seq;c->sample_valid=1U;return 1U;
}
uint8_t VisionYaw_Solve(vision_yaw_t *v,uint8_t s,uint32_t now,float h)
{
    vision_yaw_cal_t *c=cal(v,s);float da,dh;
    if(!teachable(c,now) || !c->sample_valid || c->sample_request!=c->request_seq || !isfinite(h))return 0U;
    da=VisionYaw_LineError(c->mean-c->sample_axis);dh=yaw_error(h-c->sample_heading);
    if(fabsf(da)<0.5f || fabsf(dh)<1.0f || fabsf(dh)>8.0f || !VisionYaw_Gain(v,s,-dh/da))return 0U;
    c->sample_valid=0U;return 1U;
}
uint8_t VisionYaw_Enable(vision_yaw_t *v,uint8_t on)
{
    if(on && (!v->cal[0].zero_valid || !v->cal[0].gain_valid || !v->cal[1].zero_valid || !v->cal[1].gain_valid))return 0U;
    v->enabled=(uint8_t)(on!=0U);VisionYaw_Cancel(v);return 1U;
}
uint8_t VisionYaw_Begin(vision_yaw_t *v,uint8_t s,uint8_t request,uint32_t now,float h)
{
    vision_yaw_cal_t *c=cal(v,s);VisionYaw_Cancel(v);
    if(!v->enabled || !c)return 0U;
    if(!c->zero_valid || !c->gain_valid || !isfinite(h))return 0U;
    v->station=s;v->request_seq=request;v->phase=V_YAW_COARSE;v->started=now;
    v->base_heading=v->goal_heading=v->last_heading=h;v->total=0.0f;v->stable=0U;v->last_observation=0U;v->final_wait=0U;
    v->step_tick=now;v->turn_ms=0U;return 1U;
}
static void fail(vision_yaw_t *v,const char *why)
{v->phase=V_YAW_FAILED;v->turn_allowed=0U;v->reason=why;}
void VisionYaw_Step(vision_yaw_t *v,uint32_t now,const vision_ring_axis_t *a,uint32_t age,
                    float h,float gyro,uint8_t stopped,uint8_t xy_done)
{
    vision_yaw_cal_t *c=cal(v,v->station);uint8_t good;
    v->turn_allowed=v->restart_xy=0U;
    if(v->phase==V_YAW_IDLE || v->phase==V_YAW_DONE || v->phase==V_YAW_FAILED)return;
    if(!isfinite(h) || !isfinite(gyro) || !c){fail(v,"yaw IMU/calibration invalid");return;}
    good=(uint8_t)(valid(a,age) && a->station==v->station && a->request_seq==v->request_seq);
    if(good){v->axis=a->axis_deg;v->error=c->gain*VisionYaw_LineError(a->axis_deg-c->zero);}
    if(v->phase==V_YAW_ACQUIRE && v->simultaneous && good) {
        v->phase=V_YAW_COMBINED;v->step_tick=now;v->stable=0U;
    }
    if(v->phase==V_YAW_COARSE) {
        if(v->simultaneous && good) {
            v->phase=V_YAW_COMBINED;v->step_tick=now;v->stable=0U;
        } else {
            if(!xy_done)return;
            v->phase=V_YAW_ACQUIRE;v->started=now;v->minimum_tick=now;
            v->base_heading=v->goal_heading=v->last_heading=h;v->total=0.0f;v->stable=0U;return;
        }
    }
    if(fabsf(yaw_error(h-v->base_heading))>V_YAW_TOTAL_DEG){fail(v,"yaw actual angle budget");return;}
    if(v->phase==V_YAW_COMBINED) {
        uint32_t dt=now-v->step_tick;float moved=fabsf(yaw_error(h-v->last_heading));
        v->step_tick=now;
        if(moved>=0.03f){v->total+=moved;v->last_heading=h;}
        if(v->total>V_YAW_TOTAL_DEG){fail(v,"yaw cumulative actual angle budget");return;}
        if(!good){v->goal_heading=h;v->stable=0U;return;}
        if(fabsf(v->error)>V_YAW_MAX_ERROR_DEG){fail(v,"yaw visual error exceeds 8deg");return;}
        /* Live image/body mapping is unchanged by chassis yaw. The chassis
         * mixer composes XY and this heading goal in the same velocity call. */
        if(fabsf(v->error)>(xy_done?V_YAW_FINAL_ERROR_DEG+0.0001f:0.4f)) {
            if(a->observation_id!=v->last_observation) {
                v->last_observation=a->observation_id;
                if(v->total+fabsf(v->error)>V_YAW_TOTAL_DEG){fail(v,"yaw travel plus remaining target budget; check sign/gain");return;}
                v->goal_heading=h+v->error;
            }
            v->turn_allowed=(uint8_t)(fabsf(yaw_error(v->goal_heading-h))>0.25f);
            if(v->turn_allowed)v->turn_ms+=dt;
            v->stable=0U;
            if(v->turn_ms>=V_YAW_TIMEOUT_MS){fail(v,"yaw 8s correction timeout");return;}
            return;
        }
        v->goal_heading=h;
        if(!xy_done || !stopped || fabsf(gyro)>0.8f){v->stable=0U;return;}
        if(a->observation_id==v->last_observation)return;
        v->last_observation=a->observation_id;
        if(v->stable && fabsf(VisionYaw_LineError(a->axis_deg-v->mean))>0.3f)v->stable=0U;
        if(!v->stable){v->mean=a->axis_deg;v->stable_tick=now-age;}
        if(v->stable<200U)v->stable++;
        v->mean=VisionYaw_LineError(v->mean+VisionYaw_LineError(a->axis_deg-v->mean)/v->stable);
        if(v->stable>=3U && now-age-v->stable_tick>=300U)v->phase=V_YAW_DONE;
        return;
    }
    if(v->phase==V_YAW_FINE) {
        if(!xy_done)return;
        if(!v->final_wait)v->final_wait=now;
        if(now-v->final_wait>3000U){fail(v,"yaw final stable same-frame data missing");return;}
        if(!good || !stopped || fabsf(gyro)>0.8f){v->stable=0U;return;}
        if(a->observation_id==v->last_observation)return;
        v->last_observation=a->observation_id;
        if(v->stable && fabsf(VisionYaw_LineError(a->axis_deg-v->mean))>0.3f)v->stable=0U;
        if(!v->stable){v->mean=a->axis_deg;v->stable_tick=now-age;}
        if(v->stable<200U)v->stable++;
        v->mean=VisionYaw_LineError(v->mean+VisionYaw_LineError(a->axis_deg-v->mean)/v->stable);
        if(v->stable<3U || now-age-v->stable_tick<300U)return;
        v->error=c->gain*VisionYaw_LineError(v->mean-c->zero);
        /* LineError wraps through90deg; tolerate floating rounding exactly
         * at the0.8deg boundary, far below the0.01deg wire resolution. */
        if(fabsf(v->error)>V_YAW_FINAL_ERROR_DEG+0.0001f){fail(v,"yaw changed during XY fine; recheck calibration");return;}
        v->phase=V_YAW_DONE;
        return;
    }
    {
        float moved=fabsf(yaw_error(h-v->last_heading));
        if(moved>=0.03f){v->total+=moved;v->last_heading=h;}
        if(v->total>V_YAW_TOTAL_DEG){fail(v,"yaw cumulative actual angle budget");return;}
    }
    if(now-v->started>=V_YAW_TIMEOUT_MS){fail(v,"yaw 8s correction timeout");return;}
    if(good && fabsf(v->error)>V_YAW_MAX_ERROR_DEG){fail(v,"yaw visual error exceeds 8deg");return;}
    if(!good) {
        if(v->phase==V_YAW_TURN){v->phase=V_YAW_ACQUIRE;v->minimum_tick=now+500U;v->goal_heading=h;}
        v->stable=0U;return; /* Never turn using a lost/invalid old measurement. */
    }
    if(v->phase==V_YAW_TURN) {
        if(fabsf(yaw_error(v->goal_heading-h))<=0.25f && fabsf(gyro)<=0.8f && stopped) {
            v->phase=V_YAW_ACQUIRE;v->minimum_tick=now+500U;v->goal_heading=h;v->stable=0U;
        } else v->turn_allowed=1U;
        return;
    }
    if(!stopped || fabsf(gyro)>0.8f || (int32_t)((now-age)-v->minimum_tick)<0 ||
       a->observation_id==v->last_observation)return;
    v->last_observation=a->observation_id;
    if(v->stable && fabsf(VisionYaw_LineError(a->axis_deg-v->mean))>0.3f)v->stable=0U;
    if(!v->stable){v->mean=a->axis_deg;v->stable_tick=now-age;}
    if(v->stable<200U)v->stable++;
    v->mean=VisionYaw_LineError(v->mean+VisionYaw_LineError(a->axis_deg-v->mean)/v->stable);
    if(v->stable<3U || now-age-v->stable_tick<300U)return;
    v->error=c->gain*VisionYaw_LineError(v->mean-c->zero);
    if(fabsf(v->error)<=0.4f){v->phase=V_YAW_FINE;v->goal_heading=h;v->restart_xy=1U;v->stable=0U;return;}
    if(v->total+fabsf(v->error)>V_YAW_TOTAL_DEG){fail(v,"yaw travel plus remaining target budget; check sign/gain");return;}
    v->goal_heading=h+v->error;v->phase=V_YAW_TURN;v->turn_allowed=1U;v->stable=0U;
}
