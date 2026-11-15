#include "nav_blend.h"
#include "nav_frame.h"
#include "grid_nav.h"
#include <math.h>

#define DEG_RAD (0.017453292519943295f)
float NavBlend_AngleError(float target, float actual)
{
    float e = fmodf(target - actual, 360.0f);
    if (e > 180.0f) { e -= 360.0f; }
    if (e < -180.0f) { e += 360.0f; }
    return e;
}
uint8_t NavBlend_Init(nav_blend_t *b, float fx, float fy, float ux, float uy,
                     float span, float start_yaw, float target_yaw)
{
    if (!isfinite(span) || !isfinite(start_yaw) || !isfinite(target_yaw) ||
        !isfinite(fx) || !isfinite(fy) || !isfinite(ux) || !isfinite(uy) ||
        span < 75.0f || span > 300.0f ||
        fabsf(fx*fx+fy*fy-1.0f) > 0.01f || fabsf(ux*ux+uy*uy-1.0f) > 0.01f) { return 0U; }
    b->initial_x=fx; b->initial_y=fy; b->ux=ux; b->uy=uy;
    b->span_mm=span; b->start_yaw=start_yaw;
    b->sweep_deg=NavBlend_AngleError(target_yaw,start_yaw);
    b->progress_mm=0.0f; b->cross_mm=0.0f;
    b->turn_start_mm=0.0f; b->integral=0.0f; b->cross_rate_mm_s=0.0f;
    b->along_rate_mm_s=0.0f;
    return 1U;
}
uint8_t NavBlend_Clear(const nav_blend_t *b, float x, float y, nav_blend_clear_t clear)
{
    unsigned int pos;
    unsigned int steps=(unsigned int)ceilf(b->span_mm/5.0f);
    float base=atan2f(b->initial_y,b->initial_x)/DEG_RAD;
    if (clear == 0 || fabsf(b->sweep_deg)<3.0f) { return 0U; }
    /* Yaw and distance now finish independently. Validate every possible yaw
     * after rotation starts, including rotation in place at the window end. */
    for (pos=0U; pos<=steps; ++pos) {
        float d=b->span_mm*(float)pos/(float)steps;
        float px=x+b->ux*d, py=y+b->uy*d;
        if (px<0.0f || py<0.0f || px>GRID_NAV_FIELD_MM || py>GRID_NAV_FIELD_MM) { return 0U; }
        unsigned int a;
        unsigned int turns=(d<b->turn_start_mm) ? 10U :
            (unsigned int)ceilf((fabsf(b->sweep_deg)+6.0f)/3.0f);
        for (a=0U; a<=turns; ++a) {
            float offset=(d<b->turn_start_mm) ? -15.0f+30.0f*(float)a/(float)turns :
                ((b->sweep_deg>0.0f) ? -3.0f : 3.0f) +
                (b->sweep_deg+((b->sweep_deg>0.0f) ? 6.0f : -6.0f))*(float)a/(float)turns;
            float yaw=base+b->start_yaw+offset;
            yaw=NavBlend_AngleError(yaw,0.0f);
            if (!clear((uint16_t)lroundf(px),(uint16_t)lroundf(py),(int16_t)lroundf(yaw))) { return 0U; }
        }
    }
    return 1U;
}
void NavBlend_Integrate(nav_blend_t *b, const float w[4], float yaw)
{
    float f=(w[0]+w[1]+w[2]+w[3])*0.25f;
    float r=(w[0]-w[1]-w[2]+w[3])*0.25f;
    float c=cosf(yaw*DEG_RAD), s=sinf(yaw*DEG_RAD);
    float fx=b->initial_x*c-b->initial_y*s;
    float fy=b->initial_x*s+b->initial_y*c;
    float dx=f*fx+r*fy, dy=f*fy-r*fx;
    b->progress_mm += dx*b->ux+dy*b->uy;
    b->cross_mm += -dx*b->uy+dy*b->ux;
}
void NavBlend_Observe(nav_blend_t *b, const float w[4], float yaw, float dt)
{
    float before=b->cross_mm;
    float along_before=b->progress_mm;
    NavBlend_Integrate(b,w,yaw);
    if(dt>0.001f && dt<0.5f) {
        float alpha=dt/(0.06f+dt);
        b->cross_rate_mm_s+=alpha*((b->cross_mm-before)/dt-b->cross_rate_mm_s);
        b->along_rate_mm_s+=alpha*((b->progress_mm-along_before)/dt-b->along_rate_mm_s);
    }
}

uint8_t NavBlend_Ready(const nav_blend_t *b, float yaw, float gyro)
{
    return (b->progress_mm>=fmaxf(20.0f,b->turn_start_mm) &&
        fabsf(NavBlend_AngleError(b->start_yaw+b->sweep_deg,yaw))<=3.0f &&
        fabsf(gyro)<=5.0f && fabsf(b->cross_mm)<=8.0f &&
        fabsf(b->along_rate_mm_s)<=15.0f && fabsf(b->cross_rate_mm_s)<=15.0f) ? 1U : 0U;
}

void NavBlend_Command(nav_blend_t *b, float yaw, float gyro,
                      const nav_blend_control_t *control,
                      float *f, float *r, float *omega)
{
    float desired=b->start_yaw+((b->progress_mm>=b->turn_start_mm) ? b->sweep_deg : 0.0f);
    float error=NavBlend_AngleError(desired,yaw);
    float remaining=fmaxf(0.0f,b->span_mm-b->progress_mm-3.0f);
    float at=control->decel_mm_s2*control->reaction_s;
    float brake_speed=sqrtf(at*at+2.0f*control->decel_mm_s2*remaining)-at;
    float v=fminf(control->max_mm_s,brake_speed);
    /* Real wheel ramps can leave translation after a zero request while yaw
     * changes. Velocity feedback actively brakes that residual chassis motion. */
    v=fminf(control->max_mm_s,fmaxf(-0.5f*control->max_mm_s,
        v+control->along_kv*(v-b->along_rate_mm_s)));
    float output;
    float lateral;
    float lead=fminf(15.0f,fmaxf(-15.0f,gyro*control->response_s));
    if(fabsf(error)<=5.0f) {
        b->integral=fminf(5.0f,fmaxf(-5.0f,b->integral+error*control->dt_s));
    } else { b->integral=0.0f; }
    /* Same gains, wheel-speed cap and motion-gated friction compensation as
     * normal FAST turns. No intermediate yaw target can trap the controller. */
    output=fminf(1.0f,fmaxf(-1.0f,control->kp*error+control->ki*b->integral-control->kd*gyro));
    if(fabsf(error)>3.0f && fabsf(output)<control->min_push &&
       error*gyro/fabsf(error)<=0.30f) {
        output=copysignf(control->min_push,error);
    }
    lateral=fminf(control->cross_cap_mm_s,fmaxf(-control->cross_cap_mm_s,
        b->cross_mm*control->cross_kp+b->cross_rate_mm_s*control->cross_kd));
    /* Commands arrive after wheel ramps: anticipate orientation at response,
     * while encoder integration always uses measured (not predicted) yaw. */
    NavFrame_MapToBody(b->initial_x,b->initial_y,yaw+lead,
                      (b->ux*v+b->uy*lateral)/control->wheel_mm_s,
                      (b->uy*v-b->ux*lateral)/control->wheel_mm_s,f,r);
    *omega=output*control->max_omega;
}
