#ifndef VISION_OBSTACLE_H
#define VISION_OBSTACLE_H
#include <stdint.h>

/* Experimental front-camera ranging: dimensions are in millimetres. */
#define VOBS_MAX_AGE_MS 200U
#define VOBS_MAP_RADIUS_MM 50U /* 25mm cylinder radius + 25mm range/pose allowance. */
#define VOBS_CENTRE_TO_FRONT_MM 140.0f
#define VOBS_CYLINDER_RADIUS_MM 25.0f

typedef struct {
    uint32_t session,last_sequence;
    uint8_t armed,sequence_valid;
} vision_obstacle_t;

void VisionObstacle_SetArmed(vision_obstacle_t *v,uint8_t armed,uint32_t now);
/* Rejected observations never request a chassis stop. */
const char *VisionObstacle_Accept(vision_obstacle_t *v,uint32_t session,uint32_t sequence,
                                int32_t gap,int32_t right,uint32_t age,uint8_t eligible);
uint8_t VisionObstacle_Project(float car_x,float car_y,float world_yaw,
                               float gap,float right,float *x,float *y);
/* Wheel displacement uses the same FL/FR/RL/RR signed feedback as navigation. */
uint8_t VisionObstacle_WheelPose(float start_x,float start_y,float world_yaw,
    const int32_t baseline[4],const int32_t current[4],const double signs[4],
    float counts_per_rev,float mm_per_rev,float *x,float *y);
/* cx/cy are GridNav internal indices: row 0 is the north/top map row. */
uint8_t VisionObstacle_DiscCell(uint16_t x,uint16_t y,uint16_t radius,uint8_t cx,uint8_t cy);
#endif
