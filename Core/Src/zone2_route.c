#include "zone2_route.h"
#include "grid_nav.h"
#include "station_map.h"
#include <string.h>

uint16_t Zone2Route_FixedExtraMm(match_checkpoint_t checkpoint)
{
    switch(checkpoint) {
      case MATCH_RAW1:return ZONE2_RAW_EXTRA_MM+ZONE2_QR_RAW_EXTRA_MM;
      case MATCH_RAW2:return ZONE2_RAW_EXTRA_MM;
      case MATCH_ROUGH1:case MATCH_ROUGH2:return ZONE2_ROUGH_EXTRA_MM;
      case MATCH_TEMP1:case MATCH_TEMP2:return ZONE2_TEMP_EXTRA_MM;
      default:return 0U;
    }
}

static void add(zone2_route_t *r,zone2_action_kind_t kind,uint16_t x,uint16_t y,
                int16_t yaw,int8_t dx,int8_t dy,uint8_t final,const char *name)
{
    zone2_action_t a={kind,x,y,yaw,dx,dy,final,name,0U};
    r->actions[r->count++]=a;
}
void Zone2Route_Init(zone2_route_t *r,const zone2_ports_t *ports)
{ memset(r,0,sizeof(*r)); r->ports=*ports; }
void Zone2Route_Cancel(zone2_route_t *r)
{
    r->running=0U;r->pending=0U;r->active_id=0U;
    if(r->ports.cancel)r->ports.cancel();
}
static void finish(zone2_route_t *r,uint8_t success)
{
    if(!success)Zone2Route_Cancel(r);
    r->running=0U;r->pending=0U;r->active_id=0U;
    if(r->ports.finished)r->ports.finished(success);
}
uint8_t Zone2Route_Start(zone2_route_t *r,match_motion_t motion,match_checkpoint_t checkpoint)
{
    return Zone2Route_StartForZone(r,motion,checkpoint,2U);
}
uint8_t Zone2Route_StartForZone(zone2_route_t *r,match_motion_t motion,
                               match_checkpoint_t checkpoint,uint8_t start_zone)
{
    uint16_t target_x=0U,target_y=0U;
    if(r->running || !r->ports.start || (start_zone!=1U && start_zone!=2U))return 0U;
    r->count=0U;r->index=0U;r->active_id=0U;
    switch(motion) {
      case MATCH_MOTION_QR:
        /* Face the east-side QR board; either start zone uses this world yaw. */
        (void)StationMap_TargetMm(STATION_MAP_QR,&target_x,&target_y);
        add(r,ZONE2_NAV,target_x,target_y,0,0,0,1,"qr");
        if(start_zone==1U)r->actions[0].extra_mm=-ZONE1_EXIT_QR_SHORTEN_MM;
        add(r,ZONE2_TURN,0,0,0,0,0,0,"QR front east");break;
      case MATCH_MOTION_RAW:
        /* Both batches plan straight to the real station target. The old
         * RAW2 NW waypoint caused a corner visit and a return/U-turn. */
        (void)StationMap_TargetMm(STATION_MAP_RAW,&target_x,&target_y);
        add(r,ZONE2_NAV,target_x,target_y,0,0,0,1,"raw_align");
        if(checkpoint==MATCH_RAW1 && start_zone==1U)r->actions[0].extra_mm=-ZONE1_QR_RAW_SHORTEN_MM;
        add(r,ZONE2_TURN,0,0,0,0,0,0,"RAW left arm north");break;
      case MATCH_MOTION_ROUGH:
        /* Plan from the actual aligned station pose; local escape/rotation
         * clearance remains the checked planner's responsibility. */
        (void)StationMap_TargetMm(STATION_MAP_ROUGH,&target_x,&target_y);
        add(r,ZONE2_NAV,target_x,target_y,0,0,0,1,"rough_align");
        if(start_zone==1U)r->actions[0].extra_mm=ZONE1_RAW_ROUGH_EXTRA_MM;
        add(r,ZONE2_TURN,0,0,180,0,0,0,"ROUGH left arm south");
        break;
      case MATCH_MOTION_TEMP:
        (void)StationMap_TargetMm(STATION_MAP_TEMP,&target_x,&target_y);
        add(r,ZONE2_NAV,target_x,target_y,0,0,0,1,"temp");
        add(r,ZONE2_TURN,0,0,90,0,0,0,"TEMP left arm west");break;
      case MATCH_MOTION_RETURN:
        /* Stop at a rotation-clear staging point. Final parking is reserved
         * for vision; do not claim the entire chassis is inside h2 yet. */
        /* Each start zone uses its own outer staging lane. Neither route
         * claims final vision parking has put the whole chassis inside. */
        if(start_zone==1U) {
          /* Plan directly to staging. A detour may already pass the southern
           * staging lane; a forced QR waypoint would send it north and back. */
          add(r,ZONE2_NAV,2125,350,0,0,0,1,"park_align");
        } else {
          add(r,ZONE2_NAV,2125,2000,0,0,0,1,"park_align");
        }
        add(r,ZONE2_TURN,0,0,(start_zone==1U)?-90:90,0,0,0,
            (start_zone==1U)?"park south":"park north");break;
      default:return 0U;
    }
    r->running=1U;r->pending=1U;return 1U;
}
void Zone2Route_Result(zone2_route_t *r,uint32_t id,uint8_t success)
{
    if(!r->running || r->pending || !id || id!=r->active_id)return;
    r->active_id=0U;
    if(!success){finish(r,0U);return;}
    if(++r->index>=r->count){finish(r,1U);return;}
    r->pending=1U;
}
uint8_t Zone2Route_StartFacing(zone2_route_t *r,match_checkpoint_t checkpoint)
{
    static const int16_t yaw[7]={0,0,180,90,0,180,90};
    static const char *const names[7]={"QR front east","RAW left arm north",
        "ROUGH left arm south","TEMP left arm west","RAW left arm north",
        "ROUGH left arm south","TEMP left arm west"};
    if(r->running || !r->ports.start || (unsigned int)checkpoint>=7U)return 0U;
    r->count=0U;r->index=0U;r->active_id=0U;
    add(r,ZONE2_TURN,0,0,yaw[checkpoint],0,0,0,names[checkpoint]);
    r->running=1U;r->pending=1U;return 1U;
}
void Zone2Route_Service(zone2_route_t *r,uint32_t now)
{
    uint32_t timeout;
    if(!r->running)return;
    if(r->pending) {
      r->pending=0U;r->active_id=++r->next_id;
      if(!r->active_id)r->active_id=++r->next_id;
      r->started=now;
      {
        uint32_t id=r->active_id;
        if(!r->ports.start(&r->actions[r->index],id) && r->running && r->active_id==id)finish(r,0U);
      }
      return;
    }
    timeout=(r->actions[r->index].kind==ZONE2_NAV)?120000U:10000U;
    if(now-r->started>=timeout)finish(r,0U);
}
