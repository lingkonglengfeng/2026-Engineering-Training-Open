#include "nav_corner.h"
#include "nav_frame.h"
#include "nav_blend.h"
#include <math.h>
#include <string.h>
#define CORNER_RAD (0.017453292519943295f)
#define CORNER_PI (3.141592653589793f)
static float clip(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}
static float world_yaw(const nav_corner_t *p,float yaw)
{return NavBlend_AngleError(atan2f(p->initial_y,p->initial_x)/CORNER_RAD+yaw,0.0f);}
uint8_t NavCorner_Init(nav_corner_t *p,float fx,float fy,float x,float y,
    float cx,float cy,float ex,float ey,float ux,float uy,float radius,float yaw)
{
    float dx=ex-cx,dy=ey-cy,out=hypotf(dx,dy),wx,wy,in,cross;
    if(!p || !isfinite(x) || !isfinite(y) || !isfinite(cx) || !isfinite(cy) ||
       !isfinite(ex) || !isfinite(ey) || !isfinite(fx) || !isfinite(fy) ||
       !isfinite(ux) || !isfinite(uy) || !isfinite(yaw) || !isfinite(radius) ||
       radius<80.0f || radius>200.0f || fabsf(fx*fx+fy*fy-1.0f)>0.001f ||
       fabsf(ux*ux+uy*uy-1.0f)>0.001f || out<radius+100.0f)return 0U;
    /* Geometry can also describe a replanned interior corner. Execution
     * still requires the owner's current-map swept-footprint check. */
    wx=dx/out;wy=dy/out;in=(cx-x)*ux+(cy-y)*uy;cross=-(cx-x)*uy+(cy-y)*ux;
    if(fabsf(ux*wx+uy*wy)>0.001f || fabsf(ux*uy)>0.001f ||
       in<radius+100.0f || fabsf(cross)>8.0f ||
       fabsf(NavBlend_AngleError(atan2f(uy,ux)/CORNER_RAD,
          atan2f(fy,fx)/CORNER_RAD+yaw))>3.0f)return 0U;
    memset(p,0,sizeof(*p));p->initial_x=fx;p->initial_y=fy;p->x=x;p->y=y;
    p->corner_x=cx;p->corner_y=cy;p->end_x=ex;p->end_y=ey;
    p->ux=ux;p->uy=uy;p->wx=wx;p->wy=wy;p->radius=radius;
    p->start_x=cx-ux*in;p->start_y=cy-uy*in;p->in_mm=in-radius;p->out_mm=out-radius;
    p->arc_mm=CORNER_PI*0.5f*radius;p->total_mm=p->in_mm+p->arc_mm+p->out_mm;
    p->turn=ux*wy-uy*wx;p->last_yaw=yaw;return 1U;
}
uint8_t NavCorner_Pose(const nav_corner_t *p,float s,float *x,float *y,float *yaw,float *k)
{
    float a,c,sn,qx,qy;
    if(!p || !x || !y || !yaw || !k || !isfinite(s) || p->radius<=0.0f)return 0U;
    s=clip(s,0.0f,p->total_mm);*k=0.0f;
    if(s<p->in_mm) {*x=p->start_x+p->ux*s;*y=p->start_y+p->uy*s;*yaw=atan2f(p->uy,p->ux)/CORNER_RAD;}
    else if(s<p->in_mm+p->arc_mm) {
        a=p->turn*(s-p->in_mm)/p->radius;c=cosf(a);sn=sinf(a);
        qx=p->corner_x+p->radius*(p->wx-p->ux);qy=p->corner_y+p->radius*(p->wy-p->uy);
        *x=qx-p->radius*(p->wx*c-p->wy*sn);*y=qy-p->radius*(p->wx*sn+p->wy*c);
        *yaw=atan2f(p->uy,p->ux)/CORNER_RAD+a/CORNER_RAD;*k=p->turn/p->radius;
    } else {
        a=s-p->in_mm-p->arc_mm;
        *x=p->corner_x+p->radius*p->wx+p->wx*a;
        *y=p->corner_y+p->radius*p->wy+p->wy*a;*yaw=atan2f(p->wy,p->wx)/CORNER_RAD;
    }
    *yaw=NavBlend_AngleError(*yaw,0.0f);return 1U;
}
uint8_t NavCorner_Clear(const nav_corner_t *p,nav_corner_clear_t clear)
{
    float s,x,y,yaw,k,margin_yaw;int i;
    if(!p || !clear)return 0U;
    for(s=0.0f;s<p->total_mm;s+=s>=p->in_mm-20.0f && s<=p->in_mm+p->arc_mm+20.0f?5.0f:20.0f) {
        if(!NavCorner_Pose(p,s,&x,&y,&yaw,&k))return 0U;
        /* Start/end tangent is pre-aligned; tracking lag can grow in the
         * middle. Validate a smooth3..5deg envelope rather than introducing
         * an impossible instantaneous5deg yaw jump at the tangent join. */
        margin_yaw=fabsf(k)>0.0f?3.0f+2.0f*sinf((s-p->in_mm)/p->arc_mm*CORNER_PI):3.0f;
        for(i=-1;i<=1;i++)if(!clear(x,y,yaw+i*margin_yaw))return 0U;
    }
    return clear(p->end_x,p->end_y,atan2f(p->wy,p->wx)/CORNER_RAD);
}
static void nearest(nav_corner_t *p)
{
    float qx=p->corner_x+p->radius*(p->wx-p->ux),qy=p->corner_y+p->radius*(p->wy-p->uy);
    float rx=p->x-qx,ry=p->y-qy,a0=atan2f(-p->wy,-p->wx);
    float candidates[3],best=1e30f,s=0.0f,x,y,yaw,k,d;unsigned i;
    candidates[0]=clip((p->x-p->start_x)*p->ux+(p->y-p->start_y)*p->uy,0.0f,p->in_mm);
    candidates[1]=p->in_mm+p->radius*clip(p->turn*NavBlend_AngleError(atan2f(ry,rx)/CORNER_RAD,a0/CORNER_RAD)*CORNER_RAD,0.0f,CORNER_PI*0.5f);
    candidates[2]=p->in_mm+p->arc_mm+clip((p->x-p->corner_x-p->radius*p->wx)*p->wx+
       (p->y-p->corner_y-p->radius*p->wy)*p->wy,0.0f,p->out_mm);
    for(i=0;i<3;i++) {
        (void)NavCorner_Pose(p,candidates[i],&x,&y,&yaw,&k);d=(p->x-x)*(p->x-x)+(p->y-y)*(p->y-y);
        if(d<best){best=d;s=candidates[i];}
    }
    p->progress=fmaxf(p->progress,s);p->cross_mm=sqrtf(best);
}
void NavCorner_Observe(nav_corner_t *p,const float w[4],float yaw,float dt)
{
    float f=(w[0]+w[1]+w[2]+w[3])*0.25f,r=(w[0]-w[1]-w[2]+w[3])*0.25f,dx,dy;
    float mid=p->last_yaw+0.5f*NavBlend_AngleError(yaw,p->last_yaw);
    NavFrame_BodyToMap(p->initial_x,p->initial_y,mid,f,r,&dx,&dy);
    p->x+=dx;p->y+=dy;p->last_yaw=yaw;
    if(dt>0.001f && dt<0.5f) {
        float alpha=dt/(0.06f+dt);
        p->rate_x+=alpha*(dx/dt-p->rate_x);p->rate_y+=alpha*(dy/dt-p->rate_y);
    }
    nearest(p);
}
void NavCorner_Command(nav_corner_t *p,const nav_corner_control_t *c,
    float yaw,float gyro,float *vx,float *vy,float *omega)
{
    float x,y,target,k,tx,ty,v,vc,to_arc,remain,brake,err,norm,at;
    float exit_yaw=atan2f(p->wy,p->wx)/CORNER_RAD;
    float exit_error=NavBlend_AngleError(exit_yaw,world_yaw(p,yaw));
    *vx=*vy=*omega=0.0f;
    if(!NavCorner_Pose(p,p->progress,&x,&y,&target,&k))return;
    if(p->progress>=p->in_mm && p->turn*exit_error<=NAV_CORNER_EXIT_HEADING_DEG)
        p->exit_heading_locked=1U;
    if(p->exit_heading_locked) {
        /* Heading can arrive before the encoder projection reaches the arc
         * endpoint. Follow the exit tangent, but NEVER invent path progress. */
        float along=clip((p->x-p->corner_x-p->radius*p->wx)*p->wx+
            (p->y-p->corner_y-p->radius*p->wy)*p->wy,0.0f,p->out_mm);
        x=p->corner_x+(p->radius+along)*p->wx;
        y=p->corner_y+(p->radius+along)*p->wy;target=exit_yaw;k=0.0f;
    }
    err=NavBlend_AngleError(target,world_yaw(p,yaw));remain=fmaxf(0.0f,p->total_mm-p->progress-2.0f);
    vc=fminf(c->corner_mm_s,fminf(sqrtf(c->lateral_mm_s2*p->radius),c->max_yaw_dps*CORNER_RAD*p->radius));
    v=c->straight_mm_s;at=c->decel_mm_s2*0.12f;
    if(!p->exit_heading_locked && p->progress<p->in_mm+p->arc_mm) {
        to_arc=fmaxf(0.0f,p->in_mm-p->progress-v*0.12f);
        v=fminf(v,sqrtf(vc*vc+2.0f*c->decel_mm_s2*to_arc));
    }
    if(p->rolling_exit) {
        /* The midpoint is only a controller handoff. Match the next
         * controller's real arc-entry and endpoint braking envelopes. */
        to_arc=fmaxf(0.0f,p->total_mm-p->progress+p->chain_to_arc_mm-c->straight_mm_s*0.12f);
        v=fminf(v,sqrtf(vc*vc+2.0f*c->decel_mm_s2*to_arc));
        brake=sqrtf(at*at+2.0f*c->decel_mm_s2*fmaxf(0.0f,
            p->total_mm-p->progress+p->chain_to_end_mm-2.0f))-at;
    } else brake=sqrtf(at*at+2.0f*c->decel_mm_s2*remain)-at;
    v=fminf(v,brake);
    if(fabsf(err)>3.0f)v*=fmaxf(0.35f,1.0f-(fabsf(err)-3.0f)/20.0f);
    tx=cosf(target*CORNER_RAD);ty=sinf(target*CORNER_RAD);
    *vx=tx*v+clip((x-p->x)*3.0f,-60.0f,60.0f);
    *vy=ty*v+clip((y-p->y)*3.0f,-60.0f,60.0f);
    if(!p->rolling_exit && remain<25.0f) {
        *vx=(p->end_x-p->x)*3.0f;*vy=(p->end_y-p->y)*3.0f;
        norm=hypotf(*vx,*vy);if(norm>50.0f){*vx*=50.0f/norm;*vy*=50.0f/norm;}
    }
    *omega=clip(v*k+2.5f*err*CORNER_RAD-0.12f*gyro*CORNER_RAD,
       -c->max_yaw_dps*CORNER_RAD,c->max_yaw_dps*CORNER_RAD);
    if(p->progress>=p->in_mm && !p->exit_heading_locked) {
        /* Bound continued rotation by the angle left AFTER the measured
         * gyro's ramp/transport travel. Linear speed and real endpoint stay
         * unchanged; gyro damping and opposite-direction correction remain. */
        float rate=fmaxf(0.0f,p->turn*gyro*CORNER_RAD);
        float angle=fmaxf(0.0f,p->turn*exit_error*CORNER_RAD-rate*NAV_CORNER_YAW_BRAKE_LEAD_S);
        float limit=sqrtf(2.0f*c->yaw_decel_dps2*CORNER_RAD*angle);
        if(p->turn*(*omega)>limit)*omega=p->turn*limit;
    }
    if(!p->rolling_exit && remain<5.0f && hypotf(p->end_x-p->x,p->end_y-p->y)<=3.0f && fabsf(err)<=1.5f)*vx=*vy=*omega=0.0f;
}
uint8_t NavCorner_Ready(const nav_corner_t *p,float yaw,float gyro)
{
    float err=NavBlend_AngleError(atan2f(p->wy,p->wx)/CORNER_RAD,world_yaw(p,yaw));
    return (uint8_t)(!p->rolling_exit && p->progress>=p->total_mm-5.0f && hypotf(p->end_x-p->x,p->end_y-p->y)<=3.0f &&
       fabsf(err)<=1.5f && fabsf(gyro)<=2.5f && hypotf(p->rate_x,p->rate_y)<=12.0f);
}
