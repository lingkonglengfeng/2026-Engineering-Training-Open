#include "vision.h"
#include "vision_yaw.h"
#include "match_control.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
volatile unsigned checks,failures,first_failure;
#define CHECK(x) do {checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static vision_yaw_t zdt_vyaw;
static unsigned char zdt_ring_profile;
static match_control_t zdt_match;
static unsigned char zdt_manual_align_station;
/* INSERT_POLICY */
/* INSERT_MANUAL */
int main(void){unsigned on,profile,round;unsigned char f;
 for(on=0;on<2;on++)for(profile=0;profile<3;profile++)for(round=0;round<2;round++){
  zdt_vyaw.enabled=on;zdt_ring_profile=profile;zdt_match.checkpoint=round?MATCH_TEMP2:MATCH_TEMP1;
  CHECK(ZDT_Align_UsesYaw(2)==on && ZDT_Align_UsesYaw(3)==on && !ZDT_Align_UsesYaw(1));
  f=ZDT_Align_Flags(VISION_MODE_RING,2);CHECK(!!(f&VISION_RING_AXIS_FLAG)==on);
  f=ZDT_Align_Flags(VISION_MODE_RING,3);CHECK(!!(f&VISION_RING_AXIS_FLAG)==on);
  CHECK(!!(f&VISION_RING_OCCUPIED_FLAG)==(profile==2 || (profile==0 && round)));
  CHECK(!ZDT_Align_Flags(VISION_MODE_PLATE,1));
 }
 zdt_manual_align_station=3;zdt_vyaw.enabled=1;
 CHECK(ZDT_Align_UsesYaw(3) && ZDT_Align_UsesYaw(2));
 CHECK(ZDT_Align_Flags(VISION_MODE_RING,3)&VISION_RING_AXIS_FLAG);
 zdt_match.checkpoint=MATCH_TEMP2;zdt_ring_profile=0;
 CHECK(!(ZDT_Align_Flags(VISION_MODE_RING,3)&VISION_RING_OCCUPIED_FLAG));
 zdt_manual_align_station=0;CHECK(ZDT_Align_UsesYaw(3));
 test_manual_alignment();return failures;}
