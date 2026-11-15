#ifndef NAV_CORNER_H
#define NAV_CORNER_H
#include <stdint.h>
/* V219: entry deceleration target/arc speed x1.2; same radius and braking. */
#define NAV_CORNER_DEFAULT_RADIUS_MM 165.0f /* V142: radius +10%, speed cap unchanged. */
#define NAV_CORNER_SPEED_RATIO 0.72f
#define NAV_CORNER_DEFAULT_SPEED_MM_S 306.8352f
#define NAV_CORNER_LATERAL_MM_S2 663.552f /* 460.8 x1.2^2 for the same-radius faster arc. */
#define NAV_CORNER_MAX_YAW_DPS 120.0f
/* One orthogonal outer-lane fillet. Distances mm, velocities mm/s, CCW yaw
 * degrees; wheel integration uses the same initial-nose frame as nav_frame. */
typedef struct {
    float initial_x,initial_y,start_x,start_y,corner_x,corner_y,end_x,end_y;
    float ux,uy,wx,wy,radius,in_mm,out_mm,arc_mm,total_mm,turn;
    float x,y,progress,cross_mm,rate_x,rate_y,last_yaw;
} nav_corner_t;
typedef struct {
    float straight_mm_s,corner_mm_s,decel_mm_s2,lateral_mm_s2,max_yaw_dps;
} nav_corner_control_t;
typedef uint8_t (*nav_corner_clear_t)(float x,float y,float world_yaw);
uint8_t NavCorner_Init(nav_corner_t *p,float initial_x,float initial_y,
    float x,float y,float corner_x,float corner_y,float end_x,float end_y,
    float ux,float uy,float radius,float actual_relative_yaw);
uint8_t NavCorner_Pose(const nav_corner_t *p,float s,float *x,float *y,float *world_yaw,float *curvature);
uint8_t NavCorner_Clear(const nav_corner_t *p,nav_corner_clear_t clear);
void NavCorner_Observe(nav_corner_t *p,const float wheel_mm[4],float relative_yaw,float dt_s);
void NavCorner_Command(const nav_corner_t *p,const nav_corner_control_t *c,
    float relative_yaw,float gyro_dps,float *map_vx,float *map_vy,float *yaw_rad_s);
uint8_t NavCorner_Ready(const nav_corner_t *p,float relative_yaw,float gyro_dps);
#endif
