#ifndef ZONE2_ROUTE_H
#define ZONE2_ROUTE_H
#include "match_control.h"
/* Fixed additions after dcomp. Station additions apply only to final legs;
 * Corner additions: 2->3 southwest 20mm, 3->1 northwest 50mm. */
/* User station numbering: 1=raw, 2=rough/intermediate, 3=temp/final. */
/* Distance-correction baseline: all empirical +/-mm additions are zero. */
#define ZONE2_RAW_EXTRA_MM   (0U)
#define ZONE2_QR_RAW_EXTRA_MM (0U)
#define ZONE1_EXIT_QR_SHORTEN_MM (0)
#define ZONE1_QR_RAW_SHORTEN_MM (0)
#define ZONE1_RAW_ROUGH_EXTRA_MM (0)
#define ZONE2_ROUGH_EXTRA_MM (0U)
#define ZONE2_TEMP_EXTRA_MM  (0U)
#define ZONE2_TEMP_CORNER_EXTRA_MM (0U)
#define ZONE2_RAW_CORNER_EXTRA_MM  (0U)
typedef enum { ZONE2_NAV,ZONE2_TURN } zone2_action_kind_t;
typedef struct {
    zone2_action_kind_t kind;
    uint16_t x,y;
    int16_t yaw;
    int8_t dx,dy;
    uint8_t final_point;
    const char *name;
    int16_t extra_mm; /* Signed transport adjustment before this waypoint. */
} zone2_action_t;
typedef struct {
    uint8_t (*start)(const zone2_action_t *,uint32_t id);
    void (*cancel)(void);
    void (*finished)(uint8_t success);
} zone2_ports_t;
typedef struct {
    zone2_ports_t ports;
    zone2_action_t actions[5];
    uint8_t count,index,running,pending;
    uint32_t next_id,active_id,started;
} zone2_route_t;
void Zone2Route_Init(zone2_route_t *r,const zone2_ports_t *ports);
uint8_t Zone2Route_Start(zone2_route_t *r,match_motion_t motion,match_checkpoint_t checkpoint);
/* Shared station itinerary with start-zone compensation and return staging. */
uint8_t Zone2Route_StartForZone(zone2_route_t *r,match_motion_t motion,
                               match_checkpoint_t checkpoint,uint8_t start_zone);
/* Terminal facing only, used after zone1 transport without changing its path. */
uint8_t Zone2Route_StartFacing(zone2_route_t *r,match_checkpoint_t checkpoint);
void Zone2Route_Service(zone2_route_t *r,uint32_t now);
void Zone2Route_Result(zone2_route_t *r,uint32_t id,uint8_t success);
void Zone2Route_Cancel(zone2_route_t *r);
uint16_t Zone2Route_FixedExtraMm(match_checkpoint_t checkpoint);
#endif
