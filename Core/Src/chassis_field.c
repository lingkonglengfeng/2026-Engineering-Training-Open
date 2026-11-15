#include "chassis_field.h"
#include <math.h>

static void wheel(float x,float y,float o,float fx,float fy,float out[4])
{
    float f=x*fx+y*fy,r=x*fy-y*fx;
    out[0]=f+r-o; out[1]=f-r+o; out[2]=f-r-o; out[3]=f+r+o;
}
void ChassisField_Step(chassis_field_profile_t *p,float tx,float ty,float to,
                       float fx,float fy,float yaw,float dt,float rpm_max,
                       float accel,float decel,float turn_decel,float rpm_limit)
{
    float dx=tx-p->x,dy=ty-p->y,dist=hypotf(dx,dy);
    float rate=(tx*tx+ty*ty>p->x*p->x+p->y*p->y && tx*p->x+ty*p->y>=0.0f) ? accel : decel;
    float k=(dist>0.0f) ? fminf(1.0f,rate*dt/rpm_max/dist) : 1.0f;
    float nx=p->x+dx*k,ny=p->y+dy*k,no;
    float angular_rate=(fabsf(to)>fabsf(p->omega) && to*p->omega>=0.0f) ? accel : turn_decel;
    float angular_step=angular_rate*dt/rpm_max;
    float base[4],candidate[4],lo=0.0f,hi=1.0f,lambda,maximum=1.0f;
    unsigned int i;
    float c=cosf(yaw*0.017453292519943295f),s=sinf(yaw*0.017453292519943295f);
    float body_fx=fx*c-fy*s,body_fy=fx*s+fy*c;
    no=p->omega+fminf(angular_step,fmaxf(-angular_step,to-p->omega));
    wheel(p->x,p->y,p->omega,body_fx,body_fy,base);
    wheel(nx,ny,no,body_fx,body_fy,candidate);
    /* Continuous wheel bounds plus one RPM for output quantization. Braking
     * authority follows normal turn braking; linear braking remains decel. */
    for(i=0U;i<4U;++i) {
        float budget=((fabsf(candidate[i])>fabsf(p->wheel[i]) && candidate[i]*p->wheel[i]>=0.0f) ?
                       accel : turn_decel)*dt/rpm_max+1.0f/rpm_max;
        float offset=base[i]-p->wheel[i],slope=candidate[i]-base[i];
        if(fabsf(slope)>1.0e-7f) {
            float a=(-budget-offset)/slope,b=(budget-offset)/slope;
            lo=fmaxf(lo,fminf(a,b)); hi=fminf(hi,fmaxf(a,b));
        } else if(fabsf(offset)>budget) { hi=0.0f; }
    }
    /* A measured yaw step may exceed a wheel bound even with no acceleration.
     * Prioritize reprojecting the existing field vector over holding a stale body vector. */
    lambda=(hi>=lo) ? fminf(1.0f,fmaxf(0.0f,hi)) : 0.0f;
    p->x+=(nx-p->x)*lambda; p->y+=(ny-p->y)*lambda; p->omega+=(no-p->omega)*lambda;
    wheel(p->x,p->y,p->omega,body_fx,body_fy,p->wheel);
    for(i=0U;i<4U;++i) { maximum=fmaxf(maximum,fabsf(p->wheel[i])*rpm_max/rpm_limit); }
    p->x/=maximum; p->y/=maximum; p->omega/=maximum;
    for(i=0U;i<4U;++i) { p->wheel[i]/=maximum; }
}
