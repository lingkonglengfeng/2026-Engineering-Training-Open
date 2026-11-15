#ifndef VISION_PIXEL_H
#define VISION_PIXEL_H
#include <stdint.h>
#define VISION_TYPE_MAT_PX   0x02U
#define VISION_TYPE_RING_PX  0x03U
#define VISION_TYPE_PLATE_PX 0x09U
#define VISION_TYPE_RING_COUNT 0x83U
#define VISION_TYPE_RING_AXIS 0x0AU
#define VISION_RING_AXIS_FLAG 0x01U
#define VISION_RING_OCCUPIED_FLAG 0x02U /* Enable material-occluded ring recovery only on request. */
#define VISION_RING_GREEN_SEARCH_FLAG 0x08U /* TEMP2: publish actual green visibility for search stop. */
#define VISION_MATERIAL_CALIBRATION_FLAG 0x04U /* CAM1 mode2/station1: interactive color learning, no grab pixels. */
#define VISION_TYPE_PLATE_SCENE 0x85U
#define VISION_PLATE_SEARCH 1U
#define VISION_PLATE_ACQUIRE 2U
#define VISION_PLATE_FOUND 3U
#define VISION_SCENE_PAIR_WINDOW_MS 50U
#define VISION_PIXEL_TIMEOUT_MS 200U
typedef struct {
    uint8_t seq,id,type,valid;
    float dx_px,dy_px; /* Image pixels: right/down positive, never millimetres. */
    uint32_t observation_id; /* Local publication counter; wire SEQ wraps at 16. */
} vision_pixel_t;
typedef struct {
    uint8_t seq,id,valid,count,station,request_seq;
    uint32_t observation_id;
    uint8_t green_visible; /* RING_COUNT data[6] bit0, independent of count/geometry validity. */
} vision_ring_count_t;
typedef struct {
    uint8_t seq,id,valid,confidence,landmarks,station,request_seq;
    float axis_deg; /* Raw undirected image line, x-right/y-up, [-90,90). */
    uint32_t observation_id;
} vision_ring_axis_t;
typedef struct {
    uint8_t seq,id,valid,status,station,request_seq;
    uint32_t observation_id;
} vision_plate_scene_t;
#endif
