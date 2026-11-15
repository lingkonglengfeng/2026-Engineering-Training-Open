#ifndef QR_SCAN_H
#define QR_SCAN_H
#ifndef QR_CAMERA_RUNTIME_ENABLED
#define QR_CAMERA_RUNTIME_ENABLED 1U /* Dedicated CAM2 QR camera connected. */
#endif
#include "vision_mode_gate.h"
#include "vision_qr.h"
typedef enum { QR_SCAN_IDLE,QR_SCAN_MODE,QR_SCAN_READ,QR_SCAN_STOP,QR_SCAN_DONE,QR_SCAN_FAILED } qr_scan_phase_t;
typedef enum { QR_SCAN_WAIT,QR_SCAN_SEND_MODE,QR_SCAN_SEND_STOP,QR_SCAN_READY,QR_SCAN_ERROR } qr_scan_action_t;
typedef struct {
    qr_scan_phase_t phase;vision_mode_gate_t gate;vision_qr_task_t task;
    uint32_t read_started,stop_started;uint8_t stop_sent;
    const char *reason;
} qr_scan_t;
void QrScan_Start(qr_scan_t *s,uint32_t now);
void QrScan_ModeSent(qr_scan_t *s,uint8_t seq);
void QrScan_StopSent(qr_scan_t *s);
void QrScan_Cancel(qr_scan_t *s);
qr_scan_action_t QrScan_Step(qr_scan_t *s,uint32_t now,const vision_camera_state_t *camera,const vision_qr_task_t *task);
#endif
