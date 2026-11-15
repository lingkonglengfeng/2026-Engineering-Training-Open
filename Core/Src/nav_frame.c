#include "nav_frame.h"
#include <math.h>

#define NAV_FRAME_DEG_TO_RAD (0.017453292519943295f)

void NavFrame_MapToBody(float initial_x, float initial_y, float heading_deg,
                        float map_dx, float map_dy, float *forward, float *right)
{
    float angle = heading_deg * NAV_FRAME_DEG_TO_RAD;
    float c = cosf(angle);
    float s = sinf(angle);
    float fx = initial_x * c - initial_y * s;
    float fy = initial_x * s + initial_y * c;
    *forward = map_dx * fx + map_dy * fy;
    *right = map_dx * fy - map_dy * fx;
}

void NavFrame_BodyToMap(float initial_x,float initial_y,float heading_deg,
                        float forward,float right,float *map_x,float *map_y)
{
    float c=cosf(heading_deg*NAV_FRAME_DEG_TO_RAD),s=sinf(heading_deg*NAV_FRAME_DEG_TO_RAD);
    float fx=initial_x*c-initial_y*s,fy=initial_x*s+initial_y*c;
    *map_x=forward*fx+right*fy; *map_y=forward*fy-right*fx;
}

int8_t NavFrame_StrafeSide(float initial_x, float initial_y, float heading_deg,
                           float map_dx, float map_dy, float tolerance_deg)
{
    float forward;
    float right;
    float length = sqrtf(map_dx * map_dx + map_dy * map_dy);
    if (length == 0.0f) { return 0; }
    NavFrame_MapToBody(initial_x, initial_y, heading_deg,
                        map_dx, map_dy, &forward, &right);
    if ((fabsf(forward) > length * sinf(tolerance_deg * NAV_FRAME_DEG_TO_RAD)) ||
        (fabsf(right) < length * 0.5f)) { return 0; }
    return (right > 0.0f) ? (int8_t)1 : (int8_t)-1;
}

uint8_t NavFrame_ReverseAligned(float initial_x,float initial_y,float heading_deg,
                               float map_dx,float map_dy,float tolerance_deg)
{
    float forward,right,length=sqrtf(map_dx*map_dx+map_dy*map_dy);
    if(!isfinite(length) || length<=0.0f)return 0U;
    NavFrame_MapToBody(initial_x,initial_y,heading_deg,map_dx,map_dy,&forward,&right);
    return (uint8_t)(forward< -length*0.5f &&
        fabsf(right)<=length*sinf(tolerance_deg*NAV_FRAME_DEG_TO_RAD));
}

void NavFrame_WheelValues(float forward, float right, float values[4])
{
    values[0] = forward + right;
    values[1] = forward - right;
    values[2] = forward - right;
    values[3] = forward + right;
}

void NavFrame_VelocityWheelValues(int8_t forward_sign,int8_t strafe_sign,float values[4])
{
    NavFrame_WheelValues(strafe_sign?0.0f:(forward_sign<0?-1.0f:1.0f),
                        (float)strafe_sign,values);
}
