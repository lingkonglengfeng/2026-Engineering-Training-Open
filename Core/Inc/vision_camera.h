#ifndef VISION_CAMERA_H
#define VISION_CAMERA_H
#include <stdint.h>
#define VISION_MODE_STOP     0U
#define VISION_MODE_MATERIAL 2U
#define VISION_MODE_RING     3U
#define VISION_MODE_PLATE    8U
#define VISION_MODE_QR       6U
#define VISION_RESULT_TIMEOUT 254U
#define VISION_CAMERA_ACK_WAIT_MS 500U
#define VISION_CAMERA_MAX_ATTEMPTS 3U
#define VISION_CAMERA_MODE_STAGE_TIMEOUT_MS (VISION_CAMERA_ACK_WAIT_MS*VISION_CAMERA_MAX_ATTEMPTS+1000U)
/* Complex occluded scenes may briefly delay heartbeat dispatch. Pixels retain
 * their separate <=500ms freshness bound; this is only session liveness. */
#define VISION_CAMERA_PEER_TIMEOUT_MS 3500U
typedef struct {
    uint8_t mode,station,target,hz,flags;
    uint8_t pending,confirmed,result,request_seq,attempts;
    uint8_t heartbeat_seen,protocol_version;
    uint8_t reset_pending; /* STOP queued/in flight/failed: block new modes until ACK. */
    uint32_t heartbeat_age_ms;
    uint8_t heartbeat_mode,heartbeat_station; /* Last received heartbeat identity, diagnostics only. */
    uint32_t peer_age_ms;
} vision_camera_state_t;
#endif
