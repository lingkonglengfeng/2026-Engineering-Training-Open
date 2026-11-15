#ifndef VISION_QR_H
#define VISION_QR_H
#include <stdint.h>
#define VISION_TYPE_QR_GROUP 0x07U
#define VISION_QR_GROUP_WINDOW_MS 50U
typedef struct {
    uint8_t request_seq,colors[2][3],positions[2][3];
    char raw[16]; /* Rule barcode IDs: 5 black / 6 lightblue. */
    uint32_t observation_id;
} vision_qr_task_t;
typedef struct {
    uint8_t mask,groups[4][3],complete;
    uint32_t started;
    vision_qr_task_t task;
} vision_qr_assembly_t;
void VisionQr_Reset(vision_qr_assembly_t *q);
/* Full preliminary mission: two permutations of the SAME three colors,
 * and two permutations of ring1..3, so TEMP2 can stack by color. */
uint8_t VisionQr_CompetitionTaskValid(const vision_qr_task_t *task);
/* 0=accepted partial/duplicate, 1=new complete result, 255=invalid.
 * Wire color groups1/3 use engineering IDs; positions2/4 use1..3. */
uint8_t VisionQr_Feed(vision_qr_assembly_t *q,const uint8_t packet[8],uint8_t request_seq,uint32_t now);
#endif
