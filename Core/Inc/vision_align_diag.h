#ifndef VISION_ALIGN_DIAG_H
#define VISION_ALIGN_DIAG_H
#include "vision_align.h"
#include <stddef.h>

#define V_ALIGN_DIAG_CAPACITY 512U
typedef enum { V_DIAG_ACK,V_DIAG_BASE,V_DIAG_PULSE,V_DIAG_BRAKE,
    V_DIAG_SETTLE,V_DIAG_WAIT,V_DIAG_END,V_DIAG_PHASE_COUNT } vision_diag_phase_t;
/* Flags: usable pixel, fresh receive age, camera context, software stopped,
 * new observation, phase/state change, changed requested velocity. */
#define V_DIAG_VALID 1U
#define V_DIAG_FRESH 2U
#define V_DIAG_CONTEXT 4U
#define V_DIAG_STOPPED 8U
#define V_DIAG_NEW 16U
#define V_DIAG_TRANSITION 32U
#define V_DIAG_COMMAND 64U
typedef struct {
    uint32_t ms,observation,age;
    float dx,dy;
    int16_t forward,right,omega,yaw,gyro,rpm[4];
    uint8_t phase,flags,type,id,state,stable;
    uint8_t scene_count,scene_valid;
} vision_diag_record_t;
typedef struct {
    uint32_t now,age;
    const vision_pixel_t *pixel;
    float forward,right,omega,yaw,gyro;
    int16_t rpm[4];
    uint8_t phase,context,stopped,state,stable;
    uint8_t scene_count,scene_valid;
} vision_diag_input_t;
typedef struct {
    vision_diag_record_t rows[V_ALIGN_DIAG_CAPACITY],last;
    vision_align_config_t config;
    uint32_t started,last_tick,last_observation,last_receive,elapsed;
    uint32_t phase_ms[V_DIAG_PHASE_COUNT],overflow,new_frames,invalid_frames;
    uint32_t same_observation_polls,gap_max,gaps[5],tick_max,pulses,can_failures;
    int32_t wheel_start[4],wheel_end[4];
    const char *reason;
    uint16_t count;
    uint8_t enabled,active,has_data,station,last_phase,have_sample,have_receive;
    uint8_t success,start_mask,end_mask;
} vision_align_diag_t;
/* Single motion-owner task only. No ISR, HAL, printing or allocation here. */
void VisionDiag_Init(vision_align_diag_t *d);
void VisionDiag_Begin(vision_align_diag_t *d,uint8_t station,
                      const vision_align_config_t *config,uint32_t now);
void VisionDiag_Observe(vision_align_diag_t *d,const vision_diag_input_t *input);
void VisionDiag_Wheels(vision_align_diag_t *d,const int32_t position[4],
                       uint8_t mask,uint8_t end);
void VisionDiag_Finish(vision_align_diag_t *d,uint32_t now,uint8_t success,
                       const char *reason);
/* One bounded text line; caller must only format/output while chassis idle. */
uint8_t VisionDiag_Line(const vision_align_diag_t *d,uint16_t index,
                        char *line,size_t capacity);
#endif
