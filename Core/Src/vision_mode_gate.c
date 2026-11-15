#include "vision_mode_gate.h"
#include <stddef.h>
#include <string.h>

void VisionModeGate_Start(vision_mode_gate_t *g,uint8_t camera,uint8_t mode,
                         uint8_t station,uint8_t target,uint32_t now)
{
    memset(g,0,sizeof(*g));g->camera=camera;g->mode=mode;g->station=station;
    g->target=target;g->hz=mode==VISION_MODE_STOP?0U:20U;g->started=now;
    if(camera<1U || camera>2U || station>6U || target>6U ||
       (mode!=VISION_MODE_STOP && mode!=VISION_MODE_MATERIAL && mode!=VISION_MODE_PLATE && mode!=VISION_MODE_RING && mode!=VISION_MODE_QR) ||
       (mode==VISION_MODE_QR && (camera!=2U || station!=4U || target!=0U)) ||
       (mode==VISION_MODE_MATERIAL && (camera!=1U || station!=1U || target<1U || target>6U)) ||
       (mode==VISION_MODE_PLATE && target!=0U) ||
       (mode==VISION_MODE_RING && target!=0U && target!=2U)) {
        g->action=V_MODE_FAILED;g->reason="camera mode configuration invalid";
    }
}
uint8_t VisionModeGate_Matches(const vision_mode_gate_t *g,const vision_camera_state_t *s)
{
    return (uint8_t)(s!=NULL && !s->pending && !s->reset_pending && s->confirmed && s->result==0U &&
        s->mode==g->mode && s->station==g->station && s->target==g->target &&
        s->hz==g->hz && s->flags==g->flags && (!g->sent || s->request_seq==g->request_seq));
}
vision_mode_action_t VisionModeGate_Step(vision_mode_gate_t *g,uint32_t now,
                                        const vision_camera_state_t *s)
{
    if(g->action==V_MODE_FAILED || g->action==V_MODE_READY)return g->action;
    if(now-g->started>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS || s==NULL) {
        g->action=V_MODE_FAILED;g->reason="camera SET/ACK stage timeout";return g->action;
    }
    if(VisionModeGate_Matches(g,s)){g->action=V_MODE_READY;return g->action;}
    if(g->sent) {
        if(s->request_seq!=g->request_seq || !s->pending) {
            g->action=V_MODE_FAILED;
            g->reason=s->result==VISION_RESULT_TIMEOUT?"camera ACK timeout":
                      (s->result!=0U?"camera SET rejected":"camera mode/heartbeat confirmation lost");
        } else g->action=V_MODE_WAIT;
    } else g->action=s->pending?V_MODE_WAIT:V_MODE_SEND;
    return g->action;
}
void VisionModeGate_Sent(vision_mode_gate_t *g,uint8_t seq)
{ g->sent=1U;g->request_seq=seq;g->action=V_MODE_WAIT; }
