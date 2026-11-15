#include "qr_scan.h"
#include <string.h>
static qr_scan_action_t failed(qr_scan_t *s,const char *reason)
{s->phase=QR_SCAN_FAILED;s->reason=reason;return QR_SCAN_ERROR;}
void QrScan_Start(qr_scan_t *s,uint32_t now)
{
    memset(s,0,sizeof(*s));s->phase=QR_SCAN_MODE;
    VisionModeGate_Start(&s->gate,2U,VISION_MODE_QR,4U,0U,now);
}
void QrScan_ModeSent(qr_scan_t *s,uint8_t seq){VisionModeGate_Sent(&s->gate,seq);}
void QrScan_StopSent(qr_scan_t *s){s->stop_sent=1U;}
void QrScan_Cancel(qr_scan_t *s){memset(s,0,sizeof(*s));}
qr_scan_action_t QrScan_Step(qr_scan_t *s,uint32_t now,const vision_camera_state_t *c,const vision_qr_task_t *task)
{
    if(s->phase==QR_SCAN_DONE)return QR_SCAN_READY;
    if(s->phase==QR_SCAN_FAILED)return QR_SCAN_ERROR;
    if(s->phase==QR_SCAN_MODE) {
        vision_mode_action_t a;
        if(!c || now-s->gate.started>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS)return failed(s,"CAM2 QR SET/ACK timeout");
        /* Always issue a new SET, even if camera is already scanning. */
        if(!s->gate.sent)return (!c->pending && !c->reset_pending)?QR_SCAN_SEND_MODE:QR_SCAN_WAIT;
        a=VisionModeGate_Step(&s->gate,now,c);
        if(a==V_MODE_FAILED)return failed(s,s->gate.reason);
        if(a!=V_MODE_READY)return QR_SCAN_WAIT;
        s->phase=QR_SCAN_READ;s->read_started=now;
    }
    if(s->phase==QR_SCAN_READ) {
        if(!VisionModeGate_Matches(&s->gate,c))return failed(s,"CAM2 QR context/heartbeat lost");
        if(now-s->read_started>=15000U)return failed(s,"CAM2 QR complete task timeout");
        if(!task || !task->observation_id || task->request_seq!=s->gate.request_seq)return QR_SCAN_WAIT;
        s->task=*task;s->phase=QR_SCAN_STOP;s->stop_started=now;return QR_SCAN_SEND_STOP;
    }
    if(s->phase==QR_SCAN_STOP) {
        if(now-s->stop_started>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS)return failed(s,"CAM2 QR STOP ACK timeout");
        if(!s->stop_sent)return QR_SCAN_SEND_STOP;
        if(c && !c->pending && !c->reset_pending && c->confirmed && c->result==0U &&
            c->mode==VISION_MODE_STOP && c->station==0U && c->request_seq!=s->gate.request_seq) {
            s->phase=QR_SCAN_DONE;return QR_SCAN_READY;
        }
    }
    return QR_SCAN_WAIT;
}
