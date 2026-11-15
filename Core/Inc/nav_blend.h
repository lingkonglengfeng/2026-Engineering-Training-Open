#ifndef NAV_BLEND_H
#define NAV_BLEND_H
#include <stdint.h>
/* Pure geometry/controller helpers. All speeds are body command fractions;
 * map x right, y up; body forward/right, yaw CCW from initial forward. */
typedef struct {
    float initial_x, initial_y, ux, uy;
    float span_mm, start_yaw, sweep_deg, progress_mm, cross_mm;
    float turn_start_mm, integral, cross_rate_mm_s, along_rate_mm_s;
} nav_blend_t;
typedef struct {
    float max_mm_s, wheel_mm_s, decel_mm_s2, reaction_s;
    float max_omega, kp, ki, kd, min_push, dt_s;
    float response_s, cross_kp, cross_kd, cross_cap_mm_s, along_kv;
} nav_blend_control_t;
typedef uint8_t (*nav_blend_clear_t)(uint16_t x, uint16_t y, int16_t map_yaw);
float NavBlend_AngleError(float target, float actual);
uint8_t NavBlend_Init(nav_blend_t *b, float initial_x, float initial_y,
                     float ux, float uy, float span, float start_yaw, float target_yaw);
uint8_t NavBlend_Clear(const nav_blend_t *b, float x, float y, nav_blend_clear_t clear);
void NavBlend_Integrate(nav_blend_t *b, const float wheel_mm[4], float midpoint_yaw);
void NavBlend_Observe(nav_blend_t *b, const float wheel_mm[4], float midpoint_yaw, float dt_s);
uint8_t NavBlend_Ready(const nav_blend_t *b, float actual_yaw, float gyro_dps);
void NavBlend_Command(nav_blend_t *b, float actual_yaw, float gyro_dps,
                      const nav_blend_control_t *control,
                      float *forward, float *right, float *omega);
#endif
