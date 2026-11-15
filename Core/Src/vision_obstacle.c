#include "vision_obstacle.h"
#include <math.h>

/* V263: user sketch, P11/P12 lower reference Y762. The point selects a
 * road section; the committed obstacle blocks its full width, not a50mm disc. */
static const vision_obstacle_site_t sites[VOBS_SITE_COUNT]={
    {275,2125,0,550,0},{750,2125,1850,2400,2},{1750,2125,1850,2400,2},
    {275,1700,0,550,1},{1200,1650,1000,1400,1},{2125,1650,1850,2400,1},
    {800,1200,1000,1400,2},{1200,1200,1000,1400,0},{1650,1200,1000,1400,2},
    {275,700,0,550,1},{1200,762,1000,1400,1},{2125,762,1850,2400,1},
    {275,275,0,550,0},{675,275,0,550,2},{1700,275,0,550,2}
};
const vision_obstacle_site_t *VisionObstacle_Site(uint8_t id)
{return id && id<=VOBS_SITE_COUNT?&sites[id-1U]:0;}
uint8_t VisionObstacle_MatchSite(float ox,float oy,float cx,float cy,float yaw)
{
    uint8_t i,best=0U;float angle=yaw*.01745329252f,c=cosf(angle),s=sinf(angle),score=450.0f*450.0f;
    if(!isfinite(ox)||!isfinite(oy)||!isfinite(cx)||!isfinite(cy)||!isfinite(yaw))return 0U;
    for(i=0U;i<VOBS_SITE_COUNT;i++) {
        const vision_obstacle_site_t *p=&sites[i];float dx=ox-p->x,dy=oy-p->y,d=dx*dx+dy*dy;
        if((p->x-cx)*c+(p->y-cy)*s<=0.0f || d>=score)continue;
        /* Compare within the same road band. Never snap across a yellow block
         * just because a point in a parallel road happens to be nearer. */
        if(p->axis==1U && (ox<p->cross_min || ox>p->cross_max))continue;
        if(p->axis==2U && (oy<p->cross_min || oy>p->cross_max))continue;
        if(!p->axis && (fabsf(dx)>(p->cross_max-p->cross_min)*.5f ||
                       fabsf(dy)>(p->cross_max-p->cross_min)*.5f))continue;
        score=d;best=(uint8_t)(i+1U);
    }
    return best;
}
uint8_t VisionObstacle_SiteBarrier(uint8_t id,float yaw,uint16_t *x0,uint16_t *y0,uint16_t *x1,uint16_t *y1)
{
    const vision_obstacle_site_t *p=VisionObstacle_Site(id);uint8_t axis;
    if(!p || !isfinite(yaw) || !x0 || !y0 || !x1 || !y1)return 0U;
    axis=p->axis?p->axis:(fabsf(sinf(yaw*.01745329252f))>=fabsf(cosf(yaw*.01745329252f))?1U:2U);
    if(axis==1U) {*x0=p->cross_min;*x1=p->cross_max;*y0=p->y-50U;*y1=p->y+50U;}
    else {*y0=p->cross_min;*y1=p->cross_max;*x0=p->x-50U;*x1=p->x+50U;}
    /* Corner junctions span two perpendicular outer roads. */
    if(!p->axis && id==1U && axis==2U){*y0=1850U;*y1=2400U;}
    return 1U;
}

void VisionObstacle_SetArmed(vision_obstacle_t *v,uint8_t armed,uint32_t now)
{
    if(!v->session)v->session=now;
    if(++v->session==0U)++v->session;
    v->armed=armed?1U:0U;v->sequence_valid=0U;v->last_sequence=0U;
}
const char *VisionObstacle_Accept(vision_obstacle_t *v,uint32_t session,uint32_t sequence,
                                int32_t gap,int32_t right,uint32_t age,uint8_t eligible)
{
    if(!v->armed)return "DISARMED";
    if(session!=v->session)return "SESSION";
    if(v->sequence_valid && (int32_t)(sequence-v->last_sequence)<=0)return "SEQUENCE";
    if(!eligible)return "INELIGIBLE";
    if(age>VOBS_MAX_AGE_MS)return "STALE";
    if(gap<50 || gap>VOBS_OBSERVE_MAX_MM || right< -190 || right>190)return "RANGE";
    v->last_sequence=sequence;v->sequence_valid=1U;return 0;
}
uint8_t VisionObstacle_PathSegment(float ax,float ay,float bx,float by,
                                  float ox,float oy,float radius)
{
    float dx=bx-ax,dy=by-ay,d=dx*dx+dy*dy,t;
    if(!isfinite(ax)||!isfinite(ay)||!isfinite(bx)||!isfinite(by)||!isfinite(ox)||!isfinite(oy))return 0U;
    t=d>0.001f?fminf(1.0f,fmaxf(0.0f,((ox-ax)*dx+(oy-ay)*dy)/d)):0.0f;
    dx=ox-(ax+t*dx);dy=oy-(ay+t*dy);
    return (uint8_t)(dx*dx+dy*dy<=radius*radius);
}
float VisionObstacle_SpeedRatio(float gap)
{
    return fminf(1.0f,fmaxf(0.0f,(gap-VOBS_COMMIT_MM)/(VOBS_SLOW_START_MM-VOBS_COMMIT_MM)));
}
uint8_t VisionObstacle_Project(float car_x,float car_y,float yaw,
                               float gap,float right,float *x,float *y)
{
    float angle=yaw*0.017453292519943295f,c=cosf(angle),s=sinf(angle);
    float forward=gap+VOBS_CENTRE_TO_FRONT_MM+VOBS_CYLINDER_RADIUS_MM;
    if(!isfinite(car_x) || !isfinite(car_y) || !isfinite(yaw) ||
       !isfinite(gap) || !isfinite(right) || !x || !y)return 0U;
    *x=car_x+forward*c+right*s;*y=car_y+forward*s-right*c;
    return (uint8_t)(*x>=0.0f && *x<2400.0f && *y>=0.0f && *y<2400.0f);
}
uint8_t VisionObstacle_WheelPose(float start_x,float start_y,float yaw,
    const int32_t baseline[4],const int32_t current[4],const double signs[4],
    float counts,float mm_per_rev,float *x,float *y)
{
    float w[4],f,r,c,s;uint8_t i;
    if(!isfinite(start_x) || !isfinite(start_y) || !isfinite(yaw) ||
       !isfinite(counts) || counts<=0.0f || !isfinite(mm_per_rev) || mm_per_rev<=0.0f ||
       !baseline || !current || !signs || !x || !y)return 0U;
    for(i=0U;i<4U;i++) {
        w[i]=(float)(((double)((int64_t)current[i]-baseline[i]))*signs[i]*mm_per_rev/counts);
        if(!isfinite(w[i]))return 0U;
    }
    f=(w[0]+w[1]+w[2]+w[3])*0.25f;r=(w[0]-w[1]-w[2]+w[3])*0.25f;
    c=cosf(yaw*0.017453292519943295f);s=sinf(yaw*0.017453292519943295f);
    *x=start_x+f*c+r*s;*y=start_y+f*s-r*c;
    return (uint8_t)(isfinite(*x) && isfinite(*y) && *x>=0 && *x<=2400 && *y>=0 && *y<=2400);
}
uint8_t VisionObstacle_DiscCell(uint16_t x,uint16_t y,uint16_t radius,uint8_t cx,uint8_t cy)
{
    int32_t left=(int32_t)cx*50,bottom=(47-(int32_t)cy)*50;
    int32_t dx=x<left?left-x:(x>left+50?x-left-50:0);
    int32_t dy=y<bottom?bottom-y:(y>bottom+50?y-bottom-50:0);
    return (uint8_t)(dx*dx+dy*dy<=(int32_t)radius*radius);
}
