#ifndef VISION_ALIGN_H
#define VISION_ALIGN_H
#include <stdint.h>
#include "vision_pixel.h"
#define V_ALIGN_MODE_PULSE 0U
#define V_ALIGN_MODE_SMOOTH 1U
#define V_ALIGN_SEARCH_OFF 0U
#define V_ALIGN_SEARCH_BODY_RIGHT 1U
#define V_ALIGN_SEARCH_IMAGE_RIGHT 2U
#define V_ALIGN_FADE_START_MS 120U
#define V_ALIGN_FADE_ZERO_MS 160U
/* Only while stopped: consecutive fresh verification frames may arrive at
 * 4Hz. This does not extend the 200ms motion-observation lifetime. */
#define V_ALIGN_STABLE_FRAME_GAP_MS 500U
#define V_ALIGN_PLATE_SEARCH_FRAMES 3U
#define V_ALIGN_EMPTY_LEFT_SPEED_MULTIPLIER 2.0f
#define V_ALIGN_STATION_DRIVE_LIMIT_MS 12000U
#define V_ALIGN_PLATE_TOLERANCE_PX (21.0f)
#define V_ALIGN_TEMP2_STILL_DROP_MS (2000U)
#define V_ALIGN_RING_TOLERANCE_PX (2.0f)
#define V_ALIGN_STACKED_TOLERANCE_PX (10.0f)

/* Pixel error -> normalized new-body forward/right velocity. A signed 2x2
 * calibration supports a rotated camera; zero coefficients disable an axis. */
typedef struct {
    uint8_t camera,type,id,axes; /* axes bit0=X, bit1=Y; id=0 locks first valid target */
    float forward_x,forward_y,right_x,right_y;
    float tolerance_px,max_speed,min_speed;
    uint16_t pulse_ms,settle_ms,stable_ms,loss_ms;
    uint32_t timeout_ms,max_drive_ms;
    uint8_t stable_frames;
    uint8_t mode,station,search_axis;
    uint8_t plate_search_right; /* Plate nominal approach:0=image-left,1=image-right. */
    uint8_t zero_ring_left; /* station2: trusted empty image -> body R- */
    float pid_p,pid_i,pid_d,integral_limit_px;
    uint16_t derivative_filter_ms;
    uint32_t search_limit_ms;
    uint16_t observation_ms,fade_start_ms,fade_zero_ms;
    float search_speed;
} vision_align_config_t;
typedef enum { V_ALIGN_IDLE,V_ALIGN_SETTLE,V_ALIGN_WAIT,V_ALIGN_PULSE,
               V_ALIGN_DONE,V_ALIGN_FAILED,V_ALIGN_TRACK,V_ALIGN_SEARCH } vision_align_state_t;
typedef struct {
    vision_align_config_t config;
    vision_align_state_t state;
    uint32_t started,settled,phase_tick,minimum_frame_tick,last_observation,last_valid;
    uint32_t stable_tick,stable_receive_tick,drive_ms;
    uint8_t target_id,target_locked,stable_count;
    float forward,right,error_x,error_y;
    const char *reason;
    uint32_t control_tick,pid_tick,last_scene,search_ms;
    float integral_x,integral_y,derivative_x,derivative_y,previous_x,previous_y;
    uint8_t driving,searching,pid_ready;
    uint8_t plate_seen,plate_missing_frames;
} vision_align_t;
void VisionAlign_DefaultConfig(vision_align_config_t *config);
uint8_t VisionAlign_StationConfig(vision_align_config_t *config,uint8_t station);
uint8_t VisionAlign_ConfigValid(const vision_align_config_t *config);
uint8_t VisionAlign_Start(vision_align_t *align,const vision_align_config_t *config,
                         uint32_t now,uint32_t previous_observation);
/* Single motion-owner task. Outputs always set, including stopped/failure states.
 * age is receive age; camera exposure/processing latency must be bounded on device. */
void VisionAlign_Step(vision_align_t *align,uint32_t now,const vision_pixel_t *pixel,
                      uint32_t age,uint8_t wheels_stopped,float *forward,float *right);
void VisionAlign_StepScene(vision_align_t *align,uint32_t now,const vision_pixel_t *pixel,
                           uint32_t age,const vision_ring_count_t *scene,uint32_t scene_age,
                           uint8_t wheels_stopped,float *forward,float *right);
void VisionAlign_StepTargets(vision_align_t *align,uint32_t now,const vision_pixel_t *pixel,
    uint32_t age,const vision_ring_count_t *ring,uint32_t ring_age,
    const vision_plate_scene_t *plate,uint32_t plate_age,uint8_t wheels_stopped,float *forward,float *right);
void VisionAlign_Cancel(vision_align_t *align);
/* Explicit surveyed car-centre anchors, applied only after successful align. */
uint8_t VisionAlign_StationPoseMm(uint8_t station,uint16_t *x_mm,uint16_t *y_mm);
#endif
