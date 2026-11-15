#ifndef VISION_OBSTACLE_H
#define VISION_OBSTACLE_H
#include <stdint.h>

/* Experimental front-camera ranging: dimensions are in millimetres. */
#define VOBS_MAX_AGE_MS 200U
/* Covers the200ms frame budget at the outer owner's20ms wheel cadence. */
#define VOBS_POSE_HISTORY_COUNT 16U
#define VOBS_MAP_RADIUS_MM 50U /* 25mm cylinder radius + 25mm range/pose allowance. */
#define VOBS_CENTRE_TO_FRONT_MM 140.0f
#define VOBS_CYLINDER_RADIUS_MM 25.0f
#define VOBS_OBSERVE_MAX_MM 1200.0f
#define VOBS_SLOW_START_MM 800.0f
#define VOBS_COMMIT_MM 400.0f
#define VOBS_BRAKE_TRIGGER_MM (VOBS_COMMIT_MM+20.0f) /* Fast handoff near400mm, without asymptotic crawl. */
#define VOBS_APPROACH_MIN_RPM 15.0f
#define VOBS_SITE_COUNT 15U
#define VOBS_ASSOC_MAX_GAP_MS 1000U
#define VOBS_ASSOC_MAX_SHIFT_MM 250.0f
#define VOBS_MAX_OBJECTS 2U
typedef struct {
    float x,y,plan_x,plan_y,observation_yaw;
    uint32_t tick;
    uint8_t valid,mapped,site;
} vision_obstacle_object_t;
typedef struct {
    uint16_t x,y; /* planning reference, not a surveyed object centre */
    uint16_t cross_min,cross_max; /* complete road width */
    uint8_t axis; /* 1: north/south road, 2: east/west road, 0: junction */
} vision_obstacle_site_t;
const vision_obstacle_site_t *VisionObstacle_Site(uint8_t id);
uint8_t VisionObstacle_MatchSite(float object_x,float object_y,float car_x,float car_y,float yaw);
uint8_t VisionObstacle_SiteBarrier(uint8_t id,float approach_yaw,
                                 uint16_t *x0,uint16_t *y0,uint16_t *x1,uint16_t *y1);
/* Confirmed cached objects may be committed at the next navigation boundary;
 * otherwise moving observations retain the normal near-obstacle commit. */
uint8_t VisionObstacle_PathSegment(float ax,float ay,float bx,float by,
                                  float obstacle_x,float obstacle_y,float radius_mm);
float VisionObstacle_SpeedRatio(float front_gap_mm);

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
