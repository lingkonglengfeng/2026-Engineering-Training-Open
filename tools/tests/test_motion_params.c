#include "w25q64.h"
#include "nav_params.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static uint8_t flash[16U*4096U],locked,write_fail;static unsigned writes;
static char output[8192];
void ZDT_Console_Print(const char *s){if(strlen(output)+strlen(s)<sizeof(output))strcat(output,s);}
uint32_t W25Q64_GetCapacity(void){return 8U*1024U*1024U;}
static unsigned at(uint32_t a,uint32_t n){uint32_t base=W25Q64_GetCapacity()-sizeof(flash);CHECK(a>=base && a+n<=base+sizeof(flash));return a-base;}
uint8_t W25Q64_StorageTryLock(void){if(locked)return 0;locked=1;return 1;}
void W25Q64_StorageUnlock(void){locked=0;}
w25q64_status_t W25Q64_Init(w25q64_id_t *id){(void)id;return W25Q64_OK;}
w25q64_status_t W25Q64_Read(uint32_t a,uint8_t *p,uint32_t n){memcpy(p,flash+at(a,n),n);return W25Q64_OK;}
w25q64_status_t W25Q64_EraseSector(uint32_t a){if(write_fail)return W25Q64_ERROR_PARAM;memset(flash+at(a,4096),255,4096);return W25Q64_OK;}
w25q64_status_t W25Q64_WriteVerified(uint32_t a,const uint8_t *p,uint32_t n,uint8_t *verify,uint32_t size){CHECK(n<=size);if(write_fail)return W25Q64_ERROR_PARAM;memcpy(flash+at(a,n),p,n);memcpy(verify,p,n);writes++;return W25Q64_OK;}
#include "nav_params.c"
int main(void)
{
    motion_record_t records[2];unsigned i,before;int bank;uint32_t untouched;float previous[9];
    memset(flash,255,sizeof(flash));Nav_Params_Reset();
    CHECK(!Nav_Motion_LoadFromFlash() && writes==0 && Nav_Params()->speed_scale==.65f);
    Nav_Params()->speed_scale=.7f;Nav_Params()->time_1000mm_ms=1900;CHECK(Nav_Params_SaveToFlash());
    untouched=Nav_Crc32(flash+8192,sizeof(flash)-8192);
    CHECK(Nav_Motion_ProcessCommand("motion apply 0.8 350 1200 400 1000 1.1 600 250 1800"));
    CHECK(Nav_Params()->speed_scale==.8f && Nav_Motion()->accel_rpm_s==1200 && Nav_Motion()->field_wheel_rpm==600);
    CHECK(fabsf(Nav_Params_StraightDistance(1000)-1100)<.01f && Nav_Motion()->arc_mm_s==350);
    for(i=0;i<9;i++)previous[i]=*Motion_Target(i);
    CHECK(Nav_Motion_ProcessCommand("motion apply 0.9 400 1300 500 1500 1.2 650 nan 1800"));
    for(i=0;i<9;i++)CHECK(*Motion_Target(i)==previous[i]);
    CHECK(Nav_Motion_ProcessCommand("motion save") && strstr(output,"[MOTION SAVE] OK"));
    CHECK(Nav_Crc32(flash+8192,sizeof(flash)-8192)==untouched);
    before=writes;Nav_Params_Reset();s_motion.accel_rpm_s=600;
    CHECK(Nav_Params_LoadFromFlash() && Nav_Params()->speed_scale==.7f && Nav_Params()->time_1000mm_ms==1900);
    CHECK(Nav_Motion_LoadFromFlash() && Nav_Params()->speed_scale==.8f && s_motion.accel_rpm_s==1200 && writes==before);
    output[0]=0;Nav_Motion_ProcessCommand("motion status");CHECK(strstr(output,"source=W25Q64") && strstr(output,"[MOTION END]"));
    CHECK(Nav_Motion_ProcessCommand("motion set spd 0.9"));CHECK(Nav_Params()->speed_scale==.9f);
    CHECK(Nav_Motion_ProcessCommand("motion save"));bank=Motion_Newest(records);CHECK(bank>=0 && records[bank].sequence==2);
    flash[at(Motion_Address((unsigned)bank),sizeof(records[0]))+offsetof(motion_record_t,crc)]^=1;
    CHECK(Nav_Motion_LoadFromFlash() && Nav_Params()->speed_scale==.8f); /* Fall back to verified bank1. */
    write_fail=1;before=writes;CHECK(!Motion_Save() && !locked && writes==before);write_fail=0;
    locked=1;CHECK(Nav_Motion_LoadFromFlash()==2 && !Motion_Save());locked=0;
    output[0]=0;CHECK(Nav_Motion_ProcessCommand("motion set wheel 99"));CHECK(strstr(output,"REJECTED") && s_motion.field_wheel_rpm==600);
    output[0]=0;CHECK(Nav_Motion_ProcessCommand("motion set yaw inf"));CHECK(strstr(output,"REJECTED") && s_motion.arc_yaw_dps==250);
    CHECK(!Nav_Motion_ProcessCommand("nav"));
    memset(flash,255,8192);CHECK(!Nav_Motion_LoadFromFlash() && Nav_Params()->speed_scale==.8f);
    CHECK(Nav_Crc32(flash+8192,sizeof(flash)-8192)==untouched);
    /* Migrate a real v1 record without writing or losing the original nine values. */
    {motion_record_v1_t old;memset(&old,0,sizeof(old));old.magic=0x4D4F5431UL;old.version=1;old.size=sizeof(old);old.sequence=12;
     for(i=0;i<9;i++){old.values[i]=motion_refs[i];}old.values[0]=.77f;old.values[2]=1400;
     old.crc=Nav_Crc32((const uint8_t *)&old,offsetof(motion_record_v1_t,crc));
     memcpy(flash,&old,sizeof(old));s_motion.heading_kp=.12f;before=writes;
     CHECK(Nav_Motion_LoadFromFlash() && writes==before && motion_sequence==12);
     CHECK(Nav_Params()->speed_scale==.77f && s_motion.accel_rpm_s==1400);
     CHECK(s_motion.heading_kp==.03f && s_motion.heading_cap==.05f && s_motion.turn_band_deg==1 && s_motion.turn_stable_ms==100);
     CHECK(Nav_Motion_ProcessCommand("motion heading 0.04 0.08 0.3 2 150"));
     CHECK(s_motion.heading_kp==.04f && s_motion.heading_cap==.08f && s_motion.heading_deadband==.3f && s_motion.turn_band_deg==2 && s_motion.turn_stable_ms==150);
     CHECK(Motion_Save());bank=Motion_Newest(records);CHECK(bank==1 && records[bank].sequence==13 && records[bank].version==2);
     s_motion.heading_kp=0;s_motion.turn_stable_ms=0;CHECK(Nav_Motion_LoadFromFlash());
     CHECK(s_motion.heading_kp==.04f && s_motion.turn_stable_ms==150 && s_motion.turn_band_deg==2);
     output[0]=0;Nav_Motion_ProcessCommand("motion heading 0.04 0.08 0.3 0 150");CHECK(strstr(output,"REJECTED") && s_motion.turn_band_deg==2);
     CHECK(Nav_Crc32(flash+8192,sizeof(flash)-8192)==untouched);
    }
    return test_failures;
}
