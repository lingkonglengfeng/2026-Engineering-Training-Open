#ifndef NAV_ODOMETRY_H
#define NAV_ODOMETRY_H
#include <stdint.h>
#define NAV_ODOMETRY_ENABLED 1U
#define NAV_ODOMETRY_QUERY_MS 40U
#define NAV_ODOMETRY_STALE_MS 200U
#define NAV_ODOMETRY_ARRIVAL_MM 2.0f
#define NAV_ODOMETRY_APPROACH_GAIN 8.0f
#define NAV_ODOMETRY_EXIT_APPROACH_GAIN 8.0f /* Exit uses normal terminal approach; final distance error is diagnostic. */
typedef enum { NAV_ODO_IDLE,NAV_ODO_RUN,NAV_ODO_BRAKE,NAV_ODO_DONE,NAV_ODO_FAILED } nav_odo_state_t;
typedef struct {
    nav_odo_state_t state;
    int32_t baseline[4],previous[4];
    double signs[4];float expected[4];
    float counts_per_rev,mm_per_rev,max_scale,max_mm_s,decel_mm_s2;
    float target_mm,progress_mm,spread_mm,scale,approach_gain;
    float rolling_tail_mm; /* Checked continuation beyond a virtual handoff. */
    uint32_t started,last_sample,last_progress,stable_since,brake_tick,timeout_ms;
    float progress_mark;
    /* Map translation integrates encoder increments at the measured mid-yaw. */
    float map_initial_x,map_initial_y,map_x,map_y,map_dx,map_dy,map_yaw;
    uint8_t map_mode;
    const char *reason;
} nav_odometry_t;
uint8_t NavOdometry_Start(nav_odometry_t *o,float distance_mm,float max_scale,
    float counts_per_rev,float mm_per_rev,float max_rpm,float decel_rpm_s,
    const int32_t baseline[4],uint8_t mask,const double signs[4],const float expected[4],uint32_t now);
void NavOdometry_Observe(nav_odometry_t *o,const int32_t positions[4],uint8_t mask,uint32_t now);
uint8_t NavOdometry_SetMapDirection(nav_odometry_t *o,float initial_x,float initial_y,
    float map_x,float map_y,float relative_yaw);
void NavOdometry_ObserveHeading(nav_odometry_t *o,const int32_t positions[4],uint8_t mask,
    float relative_yaw,uint32_t now);
float NavOdometry_Step(nav_odometry_t *o,uint32_t now,uint8_t wheels_stopped);
void NavOdometry_Cancel(nav_odometry_t *o);
#endif
