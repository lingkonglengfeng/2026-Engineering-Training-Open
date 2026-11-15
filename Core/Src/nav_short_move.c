#include "nav_short_move.h"
#include <math.h>
uint8_t NavShortMove_Plan(float mm,float requested,float max_rpm,float mm_per_rev,
                          float accel,float decel,uint32_t period_ms,
                          float *scale,uint32_t *hold_ms)
{
    float dt,a,d,unit,peak,rpm,v,hold;
    if(!scale || !hold_ms || !isfinite(mm) || mm<=0.0f || mm>NAV_SHORT_MOVE_MAX_MM ||
       !isfinite(requested) || requested<=0.0f || requested>1.0f ||
       !isfinite(max_rpm) || max_rpm<=0.0f || !isfinite(mm_per_rev) || mm_per_rev<=0.0f ||
       !isfinite(accel) || accel<=0.0f || !isfinite(decel) || decel<=0.0f ||
       period_ms==0U || period_ms>100U)return 0U;
    dt=period_ms*0.001f;unit=mm_per_rev/60.0f;
    /* Match the integer-RPM software ramp at its normal update period. */
    a=roundf(fmaxf(1.0f,accel*dt))/dt*unit;
    d=roundf(fmaxf(1.0f,decel*dt))/dt*unit;
    peak=sqrtf(2.0f*mm/(1.0f/a+1.0f/d))/unit;
    rpm=floorf(fminf(fminf(requested,NAV_SHORT_MOVE_MAX_SCALE)*max_rpm,peak));
    if(rpm<1.0f)return 0U;
    v=rpm*unit;
    /* D = v*T_on - v^2/(2a) + v^2/(2d). The old planner used
     * accel+cruise+brake time as T_on, then added braking a second time. */
    hold=mm/v+v/(2.0f*a)-v/(2.0f*d);
    if(!isfinite(hold) || hold<0.0f)return 0U;
    *scale=rpm/max_rpm;
    *hold_ms=(uint32_t)ceilf(fmaxf(period_ms,hold*1000.0f));
    return 1U;
}
