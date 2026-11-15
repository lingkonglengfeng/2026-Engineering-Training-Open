#include "nav_odometry.h"
#include "nav_feedback.h"
#include "nav_frame.h"
#include <math.h>
#include <string.h>
static void fail(nav_odometry_t *o,const char *why)
{o->state=NAV_ODO_FAILED;o->scale=0.0f;o->reason=why;}
uint8_t NavOdometry_Start(nav_odometry_t *o,float distance,float scale,
    float counts,float mm,float rpm,float decel,const int32_t base[4],uint8_t mask,
    const double signs[4],const float expected[4],uint32_t now)
{
    uint8_t i;
    if(!o || !base || !signs || !expected || mask!=15U || !isfinite(distance) ||
       distance<=0.0f || distance>10000.0f || !isfinite(scale) || scale<0.01f || scale>1.0f ||
       !isfinite(counts) || counts<=0.0f || !isfinite(mm) || mm<=0.0f ||
       !isfinite(rpm) || rpm<=0.0f || !isfinite(decel) || decel<=0.0f)return 0U;
    for(i=0U;i<4U;i++)if(!isfinite(signs[i]) || fabs(signs[i])!=1.0 ||
        !isfinite(expected[i]) || fabsf(expected[i])!=1.0f)return 0U;
    memset(o,0,sizeof(*o));o->state=NAV_ODO_RUN;o->target_mm=distance;o->max_scale=scale;
    o->counts_per_rev=counts;o->mm_per_rev=mm;o->max_mm_s=rpm*mm/60.0f;
    o->decel_mm_s2=decel*mm/60.0f;o->scale=scale;
    o->approach_gain=NAV_ODOMETRY_APPROACH_GAIN;
    memcpy(o->baseline,base,sizeof(o->baseline));memcpy(o->previous,base,sizeof(o->previous));
    memcpy(o->signs,signs,sizeof(o->signs));memcpy(o->expected,expected,sizeof(o->expected));
    o->started=o->last_sample=o->last_progress=o->stable_since=now;
    /* Time is an abnormal-motion watchdog, never an arrival criterion. */
    o->timeout_ms=(uint32_t)fminf(120000.0f,fmaxf(5000.0f,3000.0f+3000.0f*distance/(scale*o->max_mm_s)));
    return 1U;
}
uint8_t NavOdometry_SetMapDirection(nav_odometry_t *o,float ix,float iy,float mx,float my,float yaw)
{
    if(!o || !isfinite(ix) || !isfinite(iy) || !isfinite(mx) || !isfinite(my) || !isfinite(yaw) ||
       fabsf(ix*ix+iy*iy-1.0f)>0.001f || fabsf(mx*mx+my*my-1.0f)>0.001f)return 0U;
    o->map_initial_x=ix;o->map_initial_y=iy;o->map_x=mx;o->map_y=my;o->map_yaw=yaw;
    o->map_dx=o->map_dy=0.0f;o->map_mode=1U;return 1U;
}
void NavOdometry_ObserveHeading(nav_odometry_t *o,const int32_t pos[4],uint8_t mask,float yaw,uint32_t now)
{
    double rev[4];float delta[4],progress,spread,step=0.0f,dx=0.0f,dy=0.0f;uint8_t i;uint32_t dt;
    if(o->state!=NAV_ODO_RUN && o->state!=NAV_ODO_BRAKE)return;
    if(mask!=15U || !pos){fail(o,"odometry needs four fresh wheel replies");return;}
    dt=now-o->last_sample;
    if(dt>=NAV_ODOMETRY_STALE_MS){fail(o,"odometry wheel feedback stale");return;}
    for(i=0U;i<4U;i++) {
        rev[i]=(double)((int64_t)pos[i]-o->baseline[i])*o->signs[i]/o->counts_per_rev;
        delta[i]=(float)((double)((int64_t)pos[i]-o->previous[i])*o->signs[i]/o->counts_per_rev)*o->mm_per_rev;
        step=fmaxf(step,(float)(fabs((double)((int64_t)pos[i]-o->previous[i]))*o->mm_per_rev/o->counts_per_rev));
    }
    progress=NavFeedback_ProjectWheels(rev,o->expected,mask,&spread)*o->mm_per_rev;
    spread*=o->mm_per_rev;
    if(o->map_mode) {
        float mid=o->map_yaw+0.5f*remainderf(yaw-o->map_yaw,360.0f);
        float f=(delta[0]+delta[1]+delta[2]+delta[3])*0.25f;
        float r=(delta[0]-delta[1]-delta[2]+delta[3])*0.25f;
        float ef,er,expected[4];double wheel_step[4];
        if(!isfinite(yaw)){fail(o,"odometry heading invalid");return;}
        NavFrame_BodyToMap(o->map_initial_x,o->map_initial_y,mid,f,r,&dx,&dy);
        progress=(o->map_dx+dx)*o->map_x+(o->map_dy+dy)*o->map_y;
        NavFrame_MapToBody(o->map_initial_x,o->map_initial_y,mid,o->map_x,o->map_y,&ef,&er);
        NavFrame_WheelValues(ef,er,expected);
        for(i=0U;i<4U;i++)wheel_step[i]=delta[i];
        (void)NavFeedback_ProjectWheels(wheel_step,expected,mask,&spread);
        spread=fmaxf(spread,o->spread_mm); /* Peak sample spread, diagnostic only. */
    }
    if(!isfinite(progress) || !isfinite(spread) ||
       fabsf(progress-o->progress_mm)>o->max_mm_s*1.5f*dt*0.001f+12.0f) {
        fail(o,"odometry position jump");return;
    }
    if(progress< -5.0f || progress<o->progress_mm-5.0f){fail(o,"odometry reverse movement");return;}
    /* Wheel spread is diagnostic only. Keep the measured spread and median
     * projected progress; do not fail solely for unequal wheel distances. */
    o->progress_mm=progress;o->spread_mm=spread;o->last_sample=now;
    if(o->map_mode){o->map_dx+=dx;o->map_dy+=dy;o->map_yaw=yaw;}
    memcpy(o->previous,pos,sizeof(o->previous));
    if(step>0.5f)o->stable_since=now;
    if(progress-o->progress_mark>=1.0f){o->progress_mark=progress;o->last_progress=now;}
    if(o->state==NAV_ODO_RUN && o->rolling_tail_mm<=0.0f && o->target_mm-progress<=NAV_ODOMETRY_ARRIVAL_MM) {
        o->state=NAV_ODO_BRAKE;o->brake_tick=o->stable_since=now;o->scale=0.0f;
    }
}
void NavOdometry_Observe(nav_odometry_t *o,const int32_t pos[4],uint8_t mask,uint32_t now)
{NavOdometry_ObserveHeading(o,pos,mask,o->map_yaw,now);}
float NavOdometry_Step(nav_odometry_t *o,uint32_t now,uint8_t stopped)
{
    float remaining,speed,a,latency;
    if(o->state!=NAV_ODO_RUN && o->state!=NAV_ODO_BRAKE)return 0.0f;
    if(now-o->last_sample>=NAV_ODOMETRY_STALE_MS){fail(o,"odometry wheel feedback stale");return 0.0f;}
    if(now-o->started>=o->timeout_ms){fail(o,"odometry motion watchdog");return 0.0f;}
    if(o->state==NAV_ODO_BRAKE) {
        if(stopped && o->last_sample-o->stable_since>=100U) {
            /* V256: settled distance residual is diagnostic only. Keep the
             * actual measurement; never fault solely for final distance error. */
            o->state=NAV_ODO_DONE;
        } else if(now-o->brake_tick>=2000U)fail(o,"odometry wheels did not settle");
        return 0.0f;
    }
    if(now-o->last_progress>=1500U){fail(o,"odometry no progress");return 0.0f;}
    /* Distance-limited continuous approach. Reserve one query and command
     * period for latency, then let the existing wheel ramp reduce RPM. */
    remaining=fmaxf(0.0f,o->target_mm+o->rolling_tail_mm-o->progress_mm-NAV_ODOMETRY_ARRIVAL_MM);
    a=o->decel_mm_s2;latency=(NAV_ODOMETRY_QUERY_MS+30U)*0.001f;
    speed=sqrtf(a*a*latency*latency+2.0f*a*remaining)-a*latency;
    speed=fminf(speed,remaining*o->approach_gain);
    /* Integer RPM must remain nonzero outside the arrival band. */
    speed=fmaxf(speed,o->mm_per_rev*1.5f/60.0f);
    o->scale=fminf(o->max_scale,speed/o->max_mm_s);return o->scale;
}
void NavOdometry_Cancel(nav_odometry_t *o)
{o->state=NAV_ODO_IDLE;o->scale=0.0f;o->rolling_tail_mm=0.0f;}
