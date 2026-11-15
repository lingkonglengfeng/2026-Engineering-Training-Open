#include "vision_obstacle.h"
#include <math.h>

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
    if(gap<50 || gap>500 || right< -190 || right>190)return "RANGE";
    v->last_sequence=sequence;v->sequence_valid=1U;return 0;
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
