#ifndef VISION_YAW_H
#define VISION_YAW_H
#include "vision_pixel.h"
#define V_YAW_FRESH_MS 500U
#define V_YAW_TIMEOUT_MS 8000U
#define V_YAW_MAX_ERROR_DEG 8.0f
#define V_YAW_TOTAL_DEG 10.0f
#define V_YAW_RATE_DPS 2.0f
#define V_YAW_FINAL_ERROR_DEG 0.8f /* Stable recheck after XY fine alignment. */
typedef enum {V_YAW_IDLE,V_YAW_COARSE,V_YAW_ACQUIRE,V_YAW_TURN,V_YAW_FINE,V_YAW_DONE,V_YAW_FAILED,V_YAW_COMBINED} vision_yaw_phase_t;
typedef struct {
    float zero,gain,mean,sample_axis,sample_heading;
    uint32_t observation,received,stable_tick;
    uint8_t zero_valid,gain_valid,sample_valid,stable,request_seq,sample_request;
} vision_yaw_cal_t;
typedef struct {
    vision_yaw_cal_t cal[2];
    uint8_t enabled,station,request_seq,stable,turn_allowed,restart_xy;
    uint8_t simultaneous;
    vision_yaw_phase_t phase;
    uint32_t started,minimum_tick,last_observation,stable_tick,final_wait;
    uint32_t step_tick,turn_ms;
    float base_heading,goal_heading,axis,error,mean,total,last_heading;
    const char *reason;
} vision_yaw_t;
void VisionYaw_Init(vision_yaw_t *v);
/* Nominal upright/down-looking camera, unmirrored, horizontal ring row. */
void VisionYaw_UseInstallationDefaults(vision_yaw_t *v);
float VisionYaw_LineError(float delta);
void VisionYaw_Observe(vision_yaw_t *v,const vision_ring_axis_t *axis,uint32_t age,uint32_t now,uint8_t stopped);
uint8_t VisionYaw_Zero(vision_yaw_t *v,uint8_t station,uint32_t now);
uint8_t VisionYaw_Gain(vision_yaw_t *v,uint8_t station,float gain);
uint8_t VisionYaw_Sample(vision_yaw_t *v,uint8_t station,uint32_t now,float heading);
uint8_t VisionYaw_Solve(vision_yaw_t *v,uint8_t station,uint32_t now,float heading);
uint8_t VisionYaw_Enable(vision_yaw_t *v,uint8_t enabled);
void VisionYaw_Cancel(vision_yaw_t *v);
uint8_t VisionYaw_Begin(vision_yaw_t *v,uint8_t station,uint8_t request,uint32_t now,float heading);
void VisionYaw_Step(vision_yaw_t *v,uint32_t now,const vision_ring_axis_t *axis,uint32_t age,
                    float heading,float gyro,uint8_t stopped,uint8_t xy_done);
#endif
