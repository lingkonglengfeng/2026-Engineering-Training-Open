#include "grid_nav.h"
#include "nav_frame.h"
#include "zone2_route.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
static match_control_t zdt_match;
static zone2_route_t zdt_zone2;
static uint8_t zdt_nav_queued_recenter;
static int8_t zdt_nav_nose_x,zdt_nav_nose_y;
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks; if(!(x)) { ++test_failures; if(!test_first_failure)test_first_failure=__LINE__; } } while(0)
void ZDT_Console_Print(const char *line) { (void)line; }
#include "departure_qr_under_test.inc"
int main(void)
{
    grid_nav_segment_t seg[GRID_NAV_MAX_SEGMENTS];zone2_action_t a={0};
    uint16_t x,y,goal_x,goal_y,count;int offset,zone;
    a.kind=ZONE2_NAV;a.x=2125U;a.y=1150U;a.name="qr";
    for(zone=1;zone<=2;zone++)for(offset=-6;offset<=6;offset++) {
        GridNav_Init();memset(&zdt_match,0,sizeof(zdt_match));memset(&zdt_zone2,0,sizeof(zdt_zone2));
        zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_QR;zdt_match.zone=(uint8_t)zone;
        zdt_nav_queued_recenter=1U;zdt_nav_nose_x=0;zdt_nav_nose_y=zone==1?1:-1;
        GridNav_SetPoseMm((uint16_t)(2125-offset),zone==1?478U:1950U);
        if(offset && abs(offset)<=5) {
            CHECK(ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));
            CHECK(GridNav_GetSegments(seg,GRID_NAV_MAX_SEGMENTS,&count) && count==1U);
            CHECK(seg[0].dx==0 && seg[0].dy==(zone==1?-1:1));
            GridNav_GetTargetMm(&goal_x,&goal_y);GridNav_GetPoseMm(&x,&y);
            CHECK(goal_x==(uint16_t)(2125-offset) && goal_y==1150U);
            CHECK(x==(uint16_t)(2125-offset) && y==(zone==1?478U:1950U));
        } else CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));
    }
    GridNav_Init();memset(&zdt_zone2,0,sizeof(zdt_zone2));zdt_match.state=MATCH_MOVING;zdt_match.checkpoint=MATCH_QR;
    zdt_nav_nose_x=0;zdt_nav_nose_y=1;zdt_nav_queued_recenter=1U;GridNav_SetPoseMm(2123U,478U);
    CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,10.0f));
    zdt_match.checkpoint=MATCH_RAW1;CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));
    zdt_match.checkpoint=MATCH_QR;zdt_nav_queued_recenter=0U;CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));
    zdt_nav_queued_recenter=1U;a.extra_mm=1;CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));a.extra_mm=0;
    zdt_zone2.index=1;CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));zdt_zone2.index=0;
    GridNav_SetObstacle(GridNav_MmToCellX(2123U),GridNav_MmToCellY(800U),1U);
    CHECK(!ZDT_Zone2_TryDepartureQrStraight(&a,0.0f));
    GridNav_GetPoseMm(&x,&y);CHECK(x==2123U && y==478U);
    GridNav_Init();GridNav_SetPoseMm(2119U,478U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,1150U,90.0f));
    CHECK(GridNav_GetSegments(seg,GRID_NAV_MAX_SEGMENTS,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==6U);
    return (int)test_failures;
}
