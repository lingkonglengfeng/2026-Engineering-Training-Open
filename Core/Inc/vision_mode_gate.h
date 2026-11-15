#ifndef VISION_MODE_GATE_H
#define VISION_MODE_GATE_H
#include "vision_camera.h"
typedef enum { V_MODE_WAIT,V_MODE_SEND,V_MODE_READY,V_MODE_FAILED } vision_mode_action_t;
typedef struct {
    uint8_t camera,mode,station,target,hz,flags,sent,request_seq;
    uint32_t started;
    vision_mode_action_t action;
    const char *reason;
} vision_mode_gate_t;
void VisionModeGate_Start(vision_mode_gate_t *gate,uint8_t camera,uint8_t mode,
                         uint8_t station,uint8_t target,uint32_t now);
vision_mode_action_t VisionModeGate_Step(vision_mode_gate_t *gate,uint32_t now,
                                        const vision_camera_state_t *state);
/* Call only after the SET was queued successfully. Busy leaves SEND retryable. */
void VisionModeGate_Sent(vision_mode_gate_t *gate,uint8_t request_seq);
uint8_t VisionModeGate_Matches(const vision_mode_gate_t *gate,const vision_camera_state_t *state);
#endif
