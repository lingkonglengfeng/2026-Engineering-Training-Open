#include "w25q64.h"
#include "station_map.h"
#include "zone2_route.h"
#include "vision_align.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static uint32_t StationMap_Address(unsigned bank);
static uint8_t flash[18U*4096U],locked,write_fail;static unsigned writes;
static char output[4096];
void ZDT_Console_Print(const char *s){if(strlen(output)+strlen(s)<sizeof(output))strcat(output,s);}
uint32_t W25Q64_GetCapacity(void){return 8U*1024U*1024U;}
static unsigned at(uint32_t a,uint32_t n){uint32_t base=W25Q64_GetCapacity()-sizeof(flash);CHECK(a>=base && a+n<=base+sizeof(flash));return a-base;}
uint8_t W25Q64_StorageTryLock(void){if(locked)return 0;locked=1;return 1;}
void W25Q64_StorageUnlock(void){locked=0;}
w25q64_status_t W25Q64_Init(w25q64_id_t *id){(void)id;return W25Q64_OK;}
w25q64_status_t W25Q64_Read(uint32_t a,uint8_t *p,uint32_t n){memcpy(p,flash+at(a,n),n);return W25Q64_OK;}
w25q64_status_t W25Q64_EraseSector(uint32_t a){if(write_fail)return W25Q64_ERROR_PARAM;CHECK(a==StationMap_Address(0)||a==StationMap_Address(1));memset(flash+at(a,4096),255,4096);return W25Q64_OK;}
w25q64_status_t W25Q64_WriteVerified(uint32_t a,const uint8_t *p,uint32_t n,uint8_t *verify,uint32_t size){CHECK(n<=size);if(write_fail)return W25Q64_ERROR_PARAM;memcpy(flash+at(a,n),p,n);memcpy(verify,p,n);writes++;return W25Q64_OK;}
/* Storage internals are included to corrupt exactly the current bank. */
#include "grid_nav.c"
static uint8_t route_start(const zone2_action_t *a,uint32_t id){(void)a;(void)id;return 1;}
static void check_unified(void)
{
  uint8_t i;uint16_t x,y,ax,ay;
  for(i=1U;i<=3U;i++) {
    CHECK(StationMap_TargetMm(i,&x,&y));CHECK(VisionAlign_StationPoseMm(i,&ax,&ay));
    CHECK(x==ax && y==ay);
  }
}
int main(void)
{
    station_map_record_t records[2];zone2_route_t route;zone2_ports_t ports={0};
    uint16_t x,y;unsigned zone,before,i;int bank;int16_t applied[4][2];uint32_t untouched,revision;
    memset(flash,255,sizeof(flash));GridNav_Init();GridNav_SetPoseMm(1000,1200);
    CHECK(!StationMap_LoadFromFlash() && !writes && StationMap_X(0,2125)==2125);
    check_unified();
    CHECK(StationMap_ProcessCommand("fieldmap apply 10 -10 -10 10 4 -6 -3 7"));
    memcpy(applied,station_map_offsets,sizeof(applied));
    CHECK(StationMap_X(0,2125)==2135 && StationMap_Y(0,1150)==1140);
    CHECK(GridNav_FindWaypoint("qr",&x,&y) && x==2135 && y==1120);
    CHECK(GridNav_FindWaypoint("rough1",&x,&y) && x==1054 && y==324);
    CHECK(VisionAlign_StationPoseMm(1,&x,&y) && x==1240 && y==2110);
    CHECK(VisionAlign_StationPoseMm(2,&x,&y) && x==1204 && y==374);
    CHECK(VisionAlign_StationPoseMm(3,&x,&y) && x==347 && y==1207);
    check_unified();
    GridNav_GetPoseMm(&x,&y);CHECK(x==1000 && y==1200); /* Editing never changes actual pose. */
    CHECK(GridNav_AddObstacleDiscMm(2090,998,50)>0);before=writes;
    GridNav_ClearRuntimeObstacles();CHECK(!GridNav_HasObstacles() && writes==before && !memcmp(applied,station_map_offsets,sizeof(applied)));
    ports.start=route_start;
    for(zone=1;zone<=2;zone++) {
      Zone2Route_Init(&route,&ports);CHECK(Zone2Route_StartForZone(&route,MATCH_MOTION_QR,MATCH_QR,(uint8_t)zone));
      CHECK(route.actions[0].x==2135 && route.actions[0].y==1120);
      Zone2Route_Init(&route,&ports);CHECK(Zone2Route_StartForZone(&route,MATCH_MOTION_RAW,MATCH_RAW2,(uint8_t)zone));
      CHECK(route.actions[0].x==1240 && route.actions[0].y==2110);
      CHECK(VisionAlign_StationPoseMm(1,&x,&y) && x==route.actions[0].x && y==route.actions[0].y);
      Zone2Route_Init(&route,&ports);CHECK(Zone2Route_StartForZone(&route,MATCH_MOTION_ROUGH,MATCH_ROUGH1,(uint8_t)zone));
      CHECK(route.actions[0].x==1204 && route.actions[0].y==374);
      CHECK(VisionAlign_StationPoseMm(2,&x,&y) && x==route.actions[0].x && y==route.actions[0].y);
      Zone2Route_Init(&route,&ports);CHECK(Zone2Route_StartForZone(&route,MATCH_MOTION_TEMP,MATCH_TEMP2,(uint8_t)zone));
      CHECK(route.actions[0].x==347 && route.actions[0].y==1207);
    }
    {const char *bad[]={"fieldmap apply 0 0 0 0 0 0 0 301","fieldmap apply 1 0 0 0 0 0 0 -301","fieldmap apply 1.5 0 0 0 0 0 0 0","fieldmap apply 0 0 0 0 0 0 0","fieldmap apply 0 0 0 0 0 0 0 0 0","fieldmap apply 300 0 0 0 0 0 0 0"};
     revision=GridNav_GetRevision();
     for(i=0;i<6;i++){output[0]=0;CHECK(StationMap_ProcessCommand(bad[i]) && strstr(output,"REJECTED"));CHECK(!memcmp(applied,station_map_offsets,sizeof(applied)) && GridNav_GetRevision()==revision);}}
    untouched=StationMap_Crc(flash+8192,sizeof(flash)-8192);
    output[0]=0;CHECK(StationMap_ProcessCommand("fieldmap save") && strstr(output,"[FIELD MAP SAVE] OK") && writes==1);
    CHECK(StationMap_Crc(flash+8192,sizeof(flash)-8192)==untouched);
    memset(station_map_offsets,0,sizeof(station_map_offsets));CHECK(StationMap_LoadFromFlash()==1 && !memcmp(applied,station_map_offsets,sizeof(applied)));
    output[0]=0;StationMap_ProcessCommand("fieldmap status");CHECK(strstr(output,"source=W25Q64") && strstr(output,"key=qr") && strstr(output,"[FIELD MAP END]"));
    StationMap_ProcessCommand("fieldmap apply 0 0 0 0 0 0 0 0");CHECK(StationMap_Save());bank=StationMap_Newest(records);
    CHECK(bank>=0 && records[bank].sequence==2);
    flash[at(StationMap_Address((unsigned)bank),sizeof(records[0]))+offsetof(station_map_record_t,crc)]^=1;
    CHECK(StationMap_LoadFromFlash()==1 && !memcmp(applied,station_map_offsets,sizeof(applied)));
    locked=1;CHECK(StationMap_LoadFromFlash()==2 && !StationMap_Save());locked=0;
    write_fail=1;before=writes;CHECK(!StationMap_Save() && !locked && writes==before);write_fail=0;
    CHECK(StationMap_Crc(flash+8192,sizeof(flash)-8192)==untouched);
    memset(flash,255,8192);CHECK(!StationMap_LoadFromFlash() && !memcmp(applied,station_map_offsets,sizeof(applied)));
    /* V279: per-axis limit raised to +/-300mm; every resulting car-centre and
       anchor point must still be on the 2400mm field. */
    output[0]=0;CHECK(StationMap_ProcessCommand("fieldmap status") && strstr(output,"limit=300"));
    output[0]=0;CHECK(StationMap_ProcessCommand("fieldmap apply -300 300 300 -300 -300 300 300 -300") && strstr(output,"[FIELD MAP APPLY] OK"));
    CHECK(StationMap_X(0,2125)==1825 && StationMap_Y(0,1150)==1450);
    CHECK(StationMap_X(1,1250)==1550 && StationMap_Y(1,2100)==1800);
    CHECK(StationMap_X(2,1200)==900 && StationMap_Y(2,380)==680);
    CHECK(StationMap_X(3,350)==650 && StationMap_Y(3,1200)==900);
    CHECK(VisionAlign_StationPoseMm(1,&x,&y) && x==1550 && y==1800);
    CHECK(VisionAlign_StationPoseMm(2,&x,&y) && x==900 && y==680);
    CHECK(VisionAlign_StationPoseMm(3,&x,&y) && x==650 && y==900);
    check_unified();
    {const char *bad[]={"fieldmap apply 301 0 0 0 0 0 0 0","fieldmap apply 0 0 0 0 0 0 0 -301","fieldmap apply 276 0 0 0 0 0 0 0"};
     memcpy(applied,station_map_offsets,sizeof(applied));
     for(i=0;i<3;i++){output[0]=0;CHECK(StationMap_ProcessCommand(bad[i]) && strstr(output,"REJECTED"));CHECK(!memcmp(applied,station_map_offsets,sizeof(applied)));}}
    CHECK(StationMap_ProcessCommand("fieldmap apply 0 0 0 0 0 0 0 0"));
    CHECK(!StationMap_ProcessCommand("motion status"));
    return test_failures;
}
