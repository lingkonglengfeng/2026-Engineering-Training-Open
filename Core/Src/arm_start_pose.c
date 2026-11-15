#include "arm_start_pose.h"
#include "arm_calibration.h"
#include "arm_driver.h"
#include "arm_task.h"
#include "arm_material.h"
#include "competition_profile.h"
#include "arm_servo.h"
#include "chassis_motor.h"
#include "zdt_motor.h"
#include <math.h>
#include <stddef.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "screen_metrics.h"
#include "w25q64.h"

typedef struct {
    int64_t zero[2];
    float percent[2],dm_deg,dm_zero;
    float span[2];
    uint32_t direction[2],esc_id,mst_id;
} start_profile_t;
enum {POSE_IDLE,POSE_TEACH,POSE_GRIP_OPEN,POSE_TOP_ENABLE,POSE_TOP,POSE_DM,POSE_REACH_ENABLE,POSE_REACH,POSE_LIFT,POSE_GRIP_CLOSE,POSE_READY,POSE_FAILED};
static start_profile_t s_saved,s_run;
static uint8_t s_turn_pending,s_dm_wait_reach;
static uint32_t s_dm_started;
static arm_feedback_wait_t s_feedback_pause;
static uint8_t s_pick; /*0=start,1=tray pick,2=tray place,3=ring place,4=ring pick*/
static uint8_t s_ring_release_handoff;
static uint8_t s_raise_opening_handoff;
static uint8_t s_pick_completed,s_ring_station=2U,s_ring_temp;
static uint8_t s_pick_result_valid;
/* UI trims are absolute deltas from the shared ROUGH-based targets, never
 * cumulative. TEMP uses the same three calibrated release endpoints. */
static float s_ring_trim[3][3]; /* shared ring, lift/reach/DM motor degrees */
static float s_disc_trim[ARM_DISC_POSE_COUNT][3];
#define POSE_GRIP_HOLD_MM (2.0f)
#define POSE_GRIP_LIFT_HOLD_MM (5.0f)
#define POSE_GRIP_HOLD_DM_DEG (2.0f)
#define POSE_GRIP_HOLD_DM_RAD_S (0.5f)
#define POSE_EARLY_DESCEND_DM_DEG (15.0f)
#define POSE_TRAY_PLACE_DESCEND_DM_DEG (5.0f)
#define POSE_PICK_CLOSE_LIFT_MM (3.0f)
#define POSE_RING_RELEASE_WAIT_MS (200U)
static uint8_t Pose_UsesEarlyPickDescent(void){return s_pick==1U || s_pick==4U;}
static uint8_t Pose_UsesEarlyLower(void){return s_pick>=1U && s_pick<=4U;}
static uint8_t s_generic,s_quiet_restore,s_material_direct;
static uint8_t s_raise_overlap,s_raise_grip_closed;
static uint8_t s_lower_recovery;
static float s_lower_hold_percent;
static uint8_t s_lower_hold_pending,s_lower_hold_samples;
static uint32_t s_lower_hold_tick,s_lower_hold_sequence;
static int64_t s_lower_hold_last;
static float s_raise_half_percent;
static uint8_t s_ring_return_reach; /*0=ordinary,1=armed,2=enable,3=native issued*/
static float s_ring_return_raise_start;
static uint32_t s_ring_return_reach_tick;
static uint8_t s_place_slot;
static uint8_t s_tray_gate;
static uint16_t s_tray_gate_target;
static uint32_t s_tray_gate_tick;
static uint8_t s_valid,s_samples,s_profile_source;
static uint8_t s_phase; /* owner task only */
static uint32_t s_started,s_tick,s_pos_seq[2],s_flag_seq[2],s_dm_seq;
static int64_t s_last_counts[2];
static float s_last_dm;
static uint8_t s_native,s_native_samples,s_native_stopped,s_native_retries,s_reach_encoder;
static uint32_t s_native_quiet_since;
static uint32_t s_native_started,s_native_timeout,s_native_settle,s_native_sequence;
static int64_t s_native_last;
static float s_native_initial;
static uint32_t s_native_retry_generation;
#define POSE_REACH_START_RESEND_MS (10U)
#define POSE_REACH_START_SENDS (50U)
typedef struct {
    int32_t goal;
    uint16_t speed;
    uint8_t active,attempts,acked,exhausted;
    uint32_t started,last_send,position_sequence,ack_fence,ack_seen,generation;
    float initial,target;
} pose_reach_start_t;
static pose_reach_start_t s_reach_start;
#define POSE_PLACE_REACH_DEADBAND_DEG (5.0f) /* Placement only: tray and rings. */
#define POSE_PLACE_CORRECTION_RPM (18U)
#define POSE_PLACE_CORRECTION_GAP_MS (200U)
static uint8_t s_place_track,s_place_track_near,s_place_track_outside;
static uint32_t s_place_track_sequence,s_place_track_command_tick,s_place_track_started;
static int64_t s_place_track_last_counts;
static int32_t s_place_track_goal_pulses;
#define POSE_DM_TIMEOUT_MS (10000U)
#define POSE_RELEASE_RAISE_MM (20.0f) /* Final release lowered10mm from V83; PICK/wait endpoints unchanged. */
#define POSE_TRAY_PICK_EXTRA_LOWER_MM (3.0f) /* V142: tray retrieval only. */
#define POSE_TRAY_EXTRA_RAISE_MM (5.0f) /* Tray release only; ring release uses its independent offsets. */
#define POSE_RING_RELEASE_LOWER_MM (27.0f) /* V89 lower7mm + another20mm. */
#define POSE_RING_EXTRA_RAISE_MM (4.0f) /* V109: all ring release endpoints UP4mm fromV108. */
static float Pose_Wrap(float v)
{v=fmodf(v+3.141592653589793f,6.283185307179586f);if(v<0)v+=6.283185307179586f;return v-3.141592653589793f;}
static float Pose_ClearancePercent(void)
{
    if(s_pick==2U)return 0.0f; /* Tray placement must finish its raise at true TOP. */
    arm_cal_axis_status_t lift;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    return ArmCalibration_TransferPercent(lift.travel_mm);
}
static float Pose_ReachTolerance(void)
{
    /* Only the user-taught near-retracted PICK/PLACE endpoint gets a 6deg
     * band (~1.47mm). Other poses retain the original percent tolerance. */
    float degrees=fabsf(s_run.percent[1]*s_run.span[1]/100.0f*360.0f/65536.0f);
    if(s_pick==2U || (s_pick && degrees<=1.0f)) {
        float tolerance=6.0f/360.0f*65536.0f/fabsf(s_run.span[1])*100.0f;
        if(tolerance>ARM_PERCENT_TOLERANCE)return tolerance;
    }
    return ARM_PERCENT_TOLERANCE;
}
/* Keep the original tray grasp reference separate from the raised PICK so
 * adjusting PICK cannot silently move the user-taught PLACE endpoint. */
static HAL_StatusTypeDef Pose_TrayBaseLiftDeg(float *out)
{
    arm_cal_axis_status_t lift;float span;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(!out || !isfinite(lift.travel_mm) || lift.travel_mm<=0.0f ||
       ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK)return HAL_ERROR;
    *out=-339.9005f+span/lift.travel_mm*40.0f*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_PickLiftBaseDeg(float *out)
{
    arm_cal_axis_status_t lift;float span;
    if(Pose_TrayBaseLiftDeg(out)!=HAL_OK)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK)return HAL_ERROR;
    *out-=span/lift.travel_mm*(15.0f-POSE_TRAY_PICK_EXTRA_LOWER_MM)*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_PickLiftDeg(float *out)
{
    if(Pose_PickLiftBaseDeg(out)!=HAL_OK)return HAL_ERROR;
    *out+=s_disc_trim[ARM_DISC_TRAY_PICK][0];return HAL_OK;
}
static HAL_StatusTypeDef Pose_PlaceLiftBaseDeg(float *out)
{
    arm_cal_axis_status_t lift;float span;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(!out || !isfinite(lift.travel_mm) || lift.travel_mm<=0.0f ||
       ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK)return HAL_ERROR;
    /* Tray release is27mm above the original tray grasp reference,
     * or15mm above current PICK. Ring endpoints remain unchanged. */
    if(Pose_TrayBaseLiftDeg(out)!=HAL_OK)return HAL_ERROR;
    *out-=span/lift.travel_mm*(2.0f+POSE_RELEASE_RAISE_MM+POSE_TRAY_EXTRA_RAISE_MM)*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_PlaceLiftDeg(float *out)
{
    if(Pose_PlaceLiftBaseDeg(out)!=HAL_OK)return HAL_ERROR;
    *out+=s_disc_trim[ARM_DISC_TRAY][0];return HAL_OK;
}
static HAL_StatusTypeDef Pose_TrayPlaceBaseTargets(uint8_t slot,float *reach,float *dm)
{
    if(slot>3U || !reach || !dm)return HAL_ERROR;
    *reach=1.0f;*dm=162.8250f;
    return HAL_OK; /* Final reach target+1deg for every tray slot; no mm offsets. */
}
static HAL_StatusTypeDef Pose_TrayPlaceTargets(uint8_t slot,float *reach,float *dm)
{
    if(Pose_TrayPlaceBaseTargets(slot,reach,dm)!=HAL_OK)return HAL_ERROR;
    *reach+=s_disc_trim[ARM_DISC_TRAY][1];*dm+=s_disc_trim[ARM_DISC_TRAY][2];return HAL_OK;
}
static HAL_StatusTypeDef Pose_RingOffsetLiftDeg(float *out,float lower_mm)
{
    arm_cal_axis_status_t lift;float span;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(!out || !isfinite(lift.travel_mm) || lift.travel_mm<=0.0f ||
       ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK)return HAL_ERROR;
    *out=-1017.5482f+span/lift.travel_mm*lower_mm*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_RingLiftDeg(float *out)
{
    arm_cal_axis_status_t lift;float span;
    if(Pose_RingOffsetLiftDeg(out,POSE_RING_RELEASE_LOWER_MM)!=HAL_OK)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK)return HAL_ERROR;
    /* Shared ground release height; retrieval reuses this target plus saved trim. */
    *out-=span/lift.travel_mm*(POSE_RELEASE_RAISE_MM+POSE_RING_EXTRA_RAISE_MM)*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_RingReachDeg(float *out)
{
    arm_cal_axis_status_t reach;float span;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
    if(!out || !isfinite(reach.travel_mm) || reach.travel_mm<=0.0f ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span)!=HAL_OK)return HAL_ERROR;
    /* Retract the taught center-ring endpoint by 9mm in its signed frame. */
    *out=-199.8138f-span/reach.travel_mm*9.0f*360.0f/65536.0f;
    return isfinite(*out)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_RingTargets(uint8_t station,float *reach,float *dm)
{
    if(!reach || !dm)return HAL_ERROR;
    if(station==1U){*reach=-532.9303f;*dm=40.9044f;return HAL_OK;}
    if(station==3U){*reach=-249.5599f;*dm=-42.5218f;return HAL_OK;}
    if(station==2U){*dm=2.6040f;return Pose_RingReachDeg(reach);}
    return HAL_ERROR;
}
static HAL_StatusTypeDef Pose_RingDropBaseTargets(uint8_t station,uint8_t temp,float *reach,float *dm)
{
    /* Shared ROUGH/TEMP placement baseline uses the taught ROUGH targets.
     * Positive reach offsets extend outward; release height/retrieval unchanged.
     * Absolute offsets from taught targets; never cumulative per move. */
    static const float extend_mm[3]={0.0f,7.0f,8.0f}; /* Previous offsets +3/+2/+0mm outward. */
    /* V172: small DM leftward corrections for ring placement only.
     * Positive values move the absolute target toward the left direction;
     * reach/height and tray placement targets remain unchanged. */
    static const float dm_offset_deg[3]={2.0f,3.2f,3.3f};
    /* Positive DM is LEFT; positive reach deltas extend outward.
     * Release height and retrieval remain unchanged. */
    static const float rough_extend_delta_mm[3]={3.0f,5.0f,2.0f};
    static const float rough_dm_delta_deg[3]={-1.6f,-1.2f,-0.3f};
    arm_cal_axis_status_t a;float span;
    (void)temp; /* Area remains a command/vision identity, not a second calibration. */
    if(Pose_RingTargets(station,reach,dm)!=HAL_OK)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);
    if(!isfinite(a.travel_mm) || a.travel_mm<=0.0f || ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span)!=HAL_OK)return HAL_ERROR;
    *reach+=span/a.travel_mm*extend_mm[station-1U]*360.0f/65536.0f;
    *dm+=dm_offset_deg[station-1U];
    *reach+=span/a.travel_mm*rough_extend_delta_mm[station-1U]*360.0f/65536.0f;
    *dm+=rough_dm_delta_deg[station-1U];
    return isfinite(*reach)?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_RingDropTargets(uint8_t station,uint8_t temp,float *reach,float *dm)
{
    if(Pose_RingDropBaseTargets(station,temp,reach,dm)!=HAL_OK)return HAL_ERROR;
    *reach+=s_ring_trim[station-1U][1];
    *dm+=s_ring_trim[station-1U][2];return HAL_OK;
}
HAL_StatusTypeDef ArmStartPose_DiscTargets(uint8_t kind,float base[3],float target[3])
{
    arm_cal_axis_status_t lift,reach;float span5,span6;uint8_t i;
    if(kind>=ARM_DISC_POSE_COUNT || !base || !target)return HAL_ERROR;
    if(kind==ARM_DISC_TRAY_PICK) {
        if(Pose_PickLiftBaseDeg(&base[0])!=HAL_OK)return HAL_ERROR;
        base[1]=-4.0f;base[2]=162.8250f;
    } else if(kind==ARM_DISC_TRAY) {
        if(Pose_PlaceLiftBaseDeg(&base[0])!=HAL_OK ||
           Pose_TrayPlaceBaseTargets(1U,&base[1],&base[2])!=HAL_OK)return HAL_ERROR;
    } else {
        ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
        ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
        if(ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span5)!=HAL_OK ||
           ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span6)!=HAL_OK ||
           !isfinite(lift.travel_mm) || lift.travel_mm<=0.0f ||
           !isfinite(reach.travel_mm) || reach.travel_mm<=0.0f)return HAL_ERROR;
        base[0]=ARM_MATERIAL_GRAB_LIFT_DEG-span5/lift.travel_mm*ARM_MATERIAL_GRAB_RAISE_MM*360.0f/65536.0f;
        base[1]=ARM_MATERIAL_WAIT_REACH_DEG;
        if(kind==ARM_DISC_GRAB)base[1]+=span6/reach.travel_mm*ARM_MATERIAL_GRAB_EXTEND_MM*360.0f/65536.0f;
        base[2]=ARM_MATERIAL_DM_DEG;
    }
    for(i=0U;i<3U;i++){target[i]=base[i]+s_disc_trim[kind][i];if(!isfinite(target[i]))return HAL_ERROR;}
    return HAL_OK;
}
typedef struct {
    uint32_t magic;uint16_t version,size;uint32_t sequence;
    float trim[ARM_DISC_POSE_COUNT][3];uint32_t crc;
} disc_record_t;
/* V238 records contain only wait/grab/tray; upgrade in RAM without writing. */
typedef struct {
    uint32_t magic;uint16_t version,size;uint32_t sequence;
    float trim[3][3];uint32_t crc;
} disc_record_v1_t;
#define DISC_MAGIC (0x44495343UL)
static uint32_t s_disc_sequence;
static uint8_t s_disc_flash_source;
static uint32_t Disc_Address(uint8_t bank)
{return W25Q64_GetCapacity()-(COMPETITION_DISC_BANK_SECTORS-bank)*W25Q64_SECTOR_SIZE;}
static uint32_t Disc_CrcBytes(const void *record,uint32_t length)
{
    const uint8_t *bytes=(const uint8_t *)record;uint32_t crc=0xFFFFFFFFUL,i;uint8_t bit;
    for(i=0U;i<length;i++){crc^=bytes[i];for(bit=0U;bit<8U;bit++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);}
    return ~crc;
}
static uint32_t Disc_Crc(const disc_record_t *record)
{return Disc_CrcBytes(record,offsetof(disc_record_t,crc));}
static void Disc_UpgradeV1(disc_record_t *record)
{
    disc_record_v1_t old;
    if(record->magic!=DISC_MAGIC || record->version!=1U || record->size!=sizeof(old))return;
    memcpy(&old,record,sizeof(old));
    if(old.crc!=Disc_CrcBytes(&old,offsetof(disc_record_v1_t,crc)))return;
    memset(record,0,sizeof(*record));record->magic=DISC_MAGIC;record->version=2U;
    record->size=sizeof(*record);record->sequence=old.sequence;
    memcpy(record->trim,old.trim,sizeof(old.trim));record->crc=Disc_Crc(record);
}
static uint8_t Disc_TrimValid(const float trim[ARM_DISC_POSE_COUNT][3])
{
    float base[3],target[3],span5,span6,p5,p6;arm_cal_axis_status_t lift;uint8_t k,i;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    if(ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span5)!=HAL_OK ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span6)!=HAL_OK || !span5 || !span6)return 0U;
    for(k=0U;k<ARM_DISC_POSE_COUNT;k++) {
        if(ArmStartPose_DiscTargets(k,base,target)!=HAL_OK)return 0U;
        for(i=0U;i<3U;i++){target[i]=base[i]+trim[k][i];if(!isfinite(target[i]))return 0U;}
        p5=target[0]/360.0f*65536.0f/span5;p6=target[1]/360.0f*65536.0f/span6;
        if(p5<0.0f || p5>1.0f || p6<((k==ARM_DISC_TRAY || k==ARM_DISC_TRAY_PICK)?-ARM_REACH_REFERENCE_TOLERANCE_MM/ARM_REACH_FULL_TRAVEL_MM:0.0f) || p6>1.0f ||
           ArmDriver_CheckUiPose(p5*lift.travel_mm,target[2])!=HAL_OK)return 0U;
    }
    return 1U;
}
static uint8_t Disc_RecordValid(const disc_record_t *r)
{return r->magic==DISC_MAGIC && r->version==2U && r->size==sizeof(*r) && r->crc==Disc_Crc(r) && Disc_TrimValid(r->trim);}
static int Disc_Newest(disc_record_t records[2])
{
    uint8_t a,b;
    if(W25Q64_GetCapacity()<COMPETITION_DISC_BANK_SECTORS*W25Q64_SECTOR_SIZE ||
       W25Q64_Read(Disc_Address(0U),(uint8_t *)&records[0],sizeof(records[0]))!=W25Q64_OK ||
       W25Q64_Read(Disc_Address(1U),(uint8_t *)&records[1],sizeof(records[1]))!=W25Q64_OK)return -2;
    Disc_UpgradeV1(&records[0]);Disc_UpgradeV1(&records[1]);
    a=Disc_RecordValid(&records[0]);b=Disc_RecordValid(&records[1]);
    if(a && b)return (int32_t)(records[1].sequence-records[0].sequence)>0?1:0;
    return b?1:(a?0:-1);
}
static HAL_StatusTypeDef Disc_Load(void)
{
    disc_record_t records[2];int bank=-1;HAL_StatusTypeDef result=HAL_ERROR;
    if(!W25Q64_StorageTryLock())return HAL_BUSY;
    if(W25Q64_Init(NULL)==W25Q64_OK)bank=Disc_Newest(records);
    if(bank>=0){memcpy(s_disc_trim,records[bank].trim,sizeof(s_disc_trim));s_disc_sequence=records[bank].sequence;s_disc_flash_source=1U;result=HAL_OK;}
    W25Q64_StorageUnlock();return result;
}
static uint8_t Disc_Save(void)
{
    disc_record_t records[2],record;uint8_t verify[sizeof(record)],target,ok=0U;int bank;
    if(!W25Q64_StorageTryLock())return 0U;
    if(W25Q64_Init(NULL)==W25Q64_OK && W25Q64_GetCapacity()>=COMPETITION_DISC_BANK_SECTORS*W25Q64_SECTOR_SIZE) {
        bank=Disc_Newest(records);
        if(bank!=-2) {
            target=(uint8_t)(bank==0?1:0);memset(&record,0,sizeof(record));
            record.magic=DISC_MAGIC;record.version=2U;record.size=sizeof(record);record.sequence=bank<0?1U:records[bank].sequence+1U;
            memcpy(record.trim,s_disc_trim,sizeof(record.trim));record.crc=Disc_Crc(&record);
            if(Disc_RecordValid(&record) && W25Q64_EraseSector(Disc_Address(target))==W25Q64_OK &&
               W25Q64_WriteVerified(Disc_Address(target),(const uint8_t *)&record,sizeof(record),verify,sizeof(verify))==W25Q64_OK) {
                s_disc_sequence=record.sequence;s_disc_flash_source=1U;ok=1U;
            }
        }
    }
    W25Q64_StorageUnlock();return ok;
}
static const char *Disc_Name(uint8_t kind)
{static const char *names[ARM_DISC_POSE_COUNT]={"wait","grab","tray","traypick"};return names[kind];}
static int Disc_Kind(const char *name)
{uint8_t i;for(i=0U;i<ARM_DISC_POSE_COUNT;i++)if(!strcmp(name,Disc_Name(i)))return i;return -1;}
static void Disc_PrintPoints(void)
{
    uint8_t k;float base[3],target[3];char out[272];
    ZDT_Console_Print("[ARM DISC POINTS] BEGIN\r\n");
    for(k=0U;k<ARM_DISC_POSE_COUNT;k++) {
        if(ArmStartPose_DiscTargets(k,base,target)!=HAL_OK){ZDT_Console_Print("[ARM DISC] FAILED: invalid calibration\r\n");return;}
        (void)snprintf(out,sizeof(out),"[ARM DISC POINT] pose=%s base_l=%.4f base_r=%.4f base_d=%.4f dl=%.4f dr=%.4f dd=%.4f\r\n",
            Disc_Name(k),(double)base[0],(double)base[1],(double)base[2],(double)s_disc_trim[k][0],(double)s_disc_trim[k][1],(double)s_disc_trim[k][2]);ZDT_Console_Print(out);
    }
    (void)snprintf(out,sizeof(out),"[ARM DISC PARAMS] source=%s sequence=%lu variant=" COMPETITION_VARIANT " banks=capacity-%u/-%usectors; automatic uses these poses\r\n",
        s_disc_flash_source?"W25Q64":"RAM_DEFAULT",(unsigned long)s_disc_sequence,
        (unsigned)COMPETITION_DISC_BANK_SECTORS,(unsigned)(COMPETITION_DISC_BANK_SECTORS-1U));ZDT_Console_Print(out);
    ZDT_Console_Print("[ARM DISC POINTS] END\r\n");
}
static uint8_t Disc_Command(const char *line)
{
    char name[9],tail,out[128];float trim[ARM_DISC_POSE_COUNT][3],base[3],target[3],l,r,d;int kind;uint8_t ok=0U;
    if(strncmp(line,"arm disc ",9U))return 0U;
    if(!strcmp(line,"arm disc points") || !strcmp(line,"arm disc params status")){Disc_PrintPoints();return 1U;}
    if(ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       ChassisMotor_HasMotionDemand() || (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL)) {
        ZDT_Console_Print("[ARM DISC] REJECTED: wait for idle arm/chassis\r\n");return 1U;
    }
    if(sscanf(line,"arm disc trim %8s %f %f %f %c",name,&l,&r,&d,&tail)==4 && (kind=Disc_Kind(name))>=0) {
        memcpy(trim,s_disc_trim,sizeof(trim));trim[kind][0]=l;trim[kind][1]=r;trim[kind][2]=d;
        if(Disc_TrimValid((const float (*)[3])trim)){memcpy(s_disc_trim,trim,sizeof(trim));s_disc_flash_source=0U;ok=1U;}
        (void)snprintf(out,sizeof(out),"[ARM DISC TRIM] HAL=%u; absolute delta, RAM only, no motion\r\n",ok?0U:1U);ZDT_Console_Print(out);return 1U;
    }
    if(sscanf(line,"arm disc pose %8s %c",name,&tail)==1 && (kind=Disc_Kind(name))>=0) {
        HAL_StatusTypeDef result=ArmStartPose_DiscTargets((uint8_t)kind,base,target);
        if(result==HAL_OK)result=ArmStartPose_BeginMove(target[0],target[1],target[2]);
        (void)snprintf(out,sizeof(out),"[ARM DISC MOVE] HAL=%u; grip unchanged, original TOP/turn/reach/lower\r\n",(unsigned)result);ZDT_Console_Print(out);return 1U;
    }
    if(!strcmp(line,"arm disc params save"))ok=Disc_Save();
    else if(!strcmp(line,"arm disc params load"))ok=(uint8_t)(Disc_Load()==HAL_OK);
    else {ZDT_Console_Print("[ARM DISC] usage: points; trim wait|grab|tray|traypick l r d; pose wait|grab|tray|traypick; params save|load|status\r\n");return 1U;}
    (void)snprintf(out,sizeof(out),"[ARM DISC PARAMS] %s HAL=%u sequence=%lu\r\n",ok?(!strcmp(line,"arm disc params save")?"SAVED":"LOADED"):"FAILED",ok?0U:1U,(unsigned long)s_disc_sequence);ZDT_Console_Print(out);return 1U;
}
typedef struct {
    uint32_t magic;uint16_t version,size;uint32_t sequence;
    float trim[2][3][3],servos[5];uint32_t crc; /* Keep V196/197 Flash layout; save mirrored groups. */
} arm_ui_record_t;
#define ARM_UI_MAGIC (0x41554931UL)
static uint32_t s_params_sequence;
static uint8_t s_prelim_ready;
static uint8_t s_boot_parameter_source; /* 0=pending,1=Flash,2=defaults. */
const char *ArmStartPose_BootParameterSource(void)
{return s_boot_parameter_source==1U?"W25Q64":s_boot_parameter_source==2U?"DEFAULT":"PENDING";}
static uint32_t Pose_ParamsCrc(const arm_ui_record_t *record)
{
    const uint8_t *bytes=(const uint8_t *)record;uint32_t crc=0xFFFFFFFFUL,i;uint8_t bit;
    for(i=0U;i<offsetof(arm_ui_record_t,crc);i++){crc^=bytes[i];for(bit=0U;bit<8U;bit++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);}
    return ~crc;
}
static uint8_t Pose_TrimValid(const float trim[3][3])
{
    uint8_t r;float lift,reach,dm,span5,span6,p5,p6;arm_cal_axis_status_t axis;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&axis);
    if(Pose_RingLiftDeg(&lift)!=HAL_OK || ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span5)!=HAL_OK ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span6)!=HAL_OK)return 0U;
    for(r=0U;r<3U;r++) {
        if(Pose_RingDropBaseTargets(r+1U,0U,&reach,&dm)!=HAL_OK)return 0U;
        p5=(lift+trim[r][0])/360.0f*65536.0f/span5;
        p6=(reach+trim[r][1])/360.0f*65536.0f/span6;dm+=trim[r][2];
        if(!isfinite(p5) || !isfinite(p6) || !isfinite(dm) || p5<0.0f || p5>1.0f || p6<0.0f || p6>1.0f || dm< -180.0f || dm>185.0f)return 0U;
        if(ArmDriver_CheckUiPose(p5*axis.travel_mm,dm)!=HAL_OK)return 0U;
    }
    return 1U;
}
static uint8_t Pose_ParamsValid(const arm_ui_record_t *record)
{
    uint8_t i;
    if(record->magic!=ARM_UI_MAGIC || (record->version!=1U && record->version!=2U) || record->size!=sizeof(*record) ||
       record->crc!=Pose_ParamsCrc(record) || !Pose_TrimValid(record->trim[0]))return 0U;
    for(i=0U;i<5U;i++)if(!isfinite(record->servos[i]) || record->servos[i]<0.0f || record->servos[i]>360.0f)return 0U;
    return 1U;
}
static uint32_t Pose_ParamsAddress(uint8_t bank)
{return W25Q64_GetCapacity()-(COMPETITION_ARM_BANK_SECTORS-bank)*W25Q64_SECTOR_SIZE;}
static int Pose_ParamsNewestAt(arm_ui_record_t records[2],uint8_t sectors)
{
    uint8_t a,b;
    if(W25Q64_GetCapacity()<sectors*W25Q64_SECTOR_SIZE)return -2;
    if(W25Q64_Read(W25Q64_GetCapacity()-sectors*W25Q64_SECTOR_SIZE,(uint8_t *)&records[0],sizeof(records[0]))!=W25Q64_OK ||
       W25Q64_Read(W25Q64_GetCapacity()-(sectors-1U)*W25Q64_SECTOR_SIZE,(uint8_t *)&records[1],sizeof(records[1]))!=W25Q64_OK)return -2;
    a=Pose_ParamsValid(&records[0]) && (sectors!=8U || records[0].version==1U);
    b=Pose_ParamsValid(&records[1]) && (sectors!=8U || records[1].version==1U);
    if(a && b)return (int32_t)(records[1].sequence-records[0].sequence)>0?1:0;
    return b?1:(a?0:-1);
}
static int Pose_ParamsNewest(arm_ui_record_t records[2]){return Pose_ParamsNewestAt(records,COMPETITION_ARM_BANK_SECTORS);}
static uint8_t Pose_RecordErased(const arm_ui_record_t *r)
{const uint8_t *p=(const uint8_t *)r;uint32_t i;for(i=0;i<sizeof(*r);i++)if(p[i]!=255U)return 0U;return 1U;}
static HAL_StatusTypeDef Pose_ParamsLoad(void)
{
    arm_ui_record_t records[2];int bank=-1;HAL_StatusTypeDef result=HAL_ERROR;
    if(!W25Q64_StorageTryLock())return HAL_BUSY;
    if(W25Q64_Init(NULL)==W25Q64_OK) {
        bank=Pose_ParamsNewest(records);
        /* Only virgin final banks migrate. Corrupt final banks never load
         * the preliminary region as a fallback. Old firmware stays on8/7. */
        if(!COMPETITION_PRELIMINARY && bank==-1 && Pose_RecordErased(&records[0]) && Pose_RecordErased(&records[1])) {
            arm_ui_record_t old[2];uint8_t verify[sizeof(old[0])];int legacy=Pose_ParamsNewestAt(old,8U);
            if(legacy>=0 && W25Q64_EraseSector(Pose_ParamsAddress(0U))==W25Q64_OK &&
               W25Q64_WriteVerified(Pose_ParamsAddress(0U),(const uint8_t *)&old[legacy],sizeof(old[0]),verify,sizeof(verify))==W25Q64_OK) {
                records[0]=old[legacy];bank=0;
                ZDT_Console_Print("[ARM PROFILE] MIGRATED legacy parameters into FINAL banks12/11; legacy unchanged\r\n");
            }
        }
    }
    if(bank>=0) {
        if(ArmServo_SetUiParameters(records[bank].servos)==HAL_OK) {
            /* Existing records carry two areas: ROUGH is the canonical set.
             * Valid saved servo presets always take priority over defaults. */
            memcpy(s_ring_trim,records[bank].trim[0],sizeof(s_ring_trim));s_params_sequence=records[bank].sequence;s_prelim_ready=(uint8_t)(!COMPETITION_PRELIMINARY && records[bank].version==2U);result=HAL_OK;
        }
    }
    W25Q64_StorageUnlock();return result;
}
HAL_StatusTypeDef ArmStartPose_LoadParameters(void)
{
    HAL_StatusTypeDef result=Pose_ParamsLoad();char line[128];
    if(result==HAL_OK) {
        s_boot_parameter_source=1U;
        (void)snprintf(line,sizeof(line),"[ARM PARAMS] BOOT source=W25Q64 sequence=%lu; shared ROUGH/TEMP targets and saved servo presets active\r\n",(unsigned long)s_params_sequence);ZDT_Console_Print(line);
    } else if(result!=HAL_BUSY) {
        s_boot_parameter_source=2U;
        ZDT_Console_Print("[ARM PARAMS] BOOT source=DEFAULT; no valid usable record/read failed; no fault added\r\n");
    }
    if(result!=HAL_BUSY) {
        HAL_StatusTypeDef disc=Disc_Load();
        if(disc==HAL_BUSY)return HAL_BUSY;
        ZDT_Console_Print(disc==HAL_OK?"[ARM DISC PARAMS] BOOT source=W25Q64; " COMPETITION_VARIANT " profiles active\r\n":
            "[ARM DISC PARAMS] BOOT source=DEFAULT; " COMPETITION_VARIANT " zero trims, original targets retained\r\n");
    }
    return result;
}
static uint8_t Pose_ParamsSave(void)
{
    arm_ui_record_t records[2],record;uint8_t verify[sizeof(record)];int bank;uint8_t target,ok=0U;
    if(!W25Q64_StorageTryLock())return 0U;
    if(W25Q64_Init(NULL)==W25Q64_OK && W25Q64_GetCapacity()>=COMPETITION_ARM_BANK_SECTORS*W25Q64_SECTOR_SIZE) {
        bank=Pose_ParamsNewest(records);if(bank== -2){W25Q64_StorageUnlock();return 0U;}
        target=(uint8_t)(bank==0?1:0);memset(&record,0,sizeof(record));
        record.magic=ARM_UI_MAGIC;record.version=(!COMPETITION_PRELIMINARY && s_prelim_ready)?2U:1U;record.size=sizeof(record);record.sequence=bank<0?1U:records[bank].sequence+1U;
        memcpy(record.trim[0],s_ring_trim,sizeof(s_ring_trim));memcpy(record.trim[1],s_ring_trim,sizeof(s_ring_trim));
        if(ArmServo_GetUiParameters(record.servos)==HAL_OK) {
            record.crc=Pose_ParamsCrc(&record);
            if(Pose_ParamsValid(&record) && W25Q64_EraseSector(Pose_ParamsAddress(target))==W25Q64_OK &&
               W25Q64_WriteVerified(Pose_ParamsAddress(target),(const uint8_t *)&record,sizeof(record),verify,sizeof(verify))==W25Q64_OK) {
                s_params_sequence=record.sequence;ok=1U;
            }
        }
    }
    W25Q64_StorageUnlock();return ok;
}
static uint8_t Profile_PreparePreliminary(void)
{
#if COMPETITION_PRELIMINARY
    return (uint8_t)(Pose_ParamsLoad()==HAL_OK && Disc_Load()==HAL_OK);
#else
    arm_ui_record_t records[2],record;uint8_t verify[sizeof(record)],ok=0U,i,bank;int newest;
    /* Save current applied FINAL values before touching the old firmware's area. */
    if(!Pose_ParamsSave() || !Disc_Save())return 0U;
    if(!W25Q64_StorageTryLock())return 0U;
    newest=Pose_ParamsNewestAt(records,8U);
    if(newest!=-2) {
        if(s_prelim_ready)ok=(uint8_t)(newest>=0); /* Preserve later preliminary tuning. */
        else {
            static const float trims[3][3]={{0,5.9f,-1.7f},{0,0,-1.2f},{0,-5.59f,-.723f}};
            static const float servos[5]={225,308,19,109,199};
            memset(&record,0,sizeof(record));record.magic=ARM_UI_MAGIC;record.version=1U;record.size=sizeof(record);
            record.sequence=newest<0?0U:records[newest].sequence;
            memcpy(record.trim[0],trims,sizeof(trims));memcpy(record.trim[1],trims,sizeof(trims));memcpy(record.servos,servos,sizeof(servos));
            record.crc=Pose_ParamsCrc(&record);
            ok=Pose_ParamsValid(&record);
            /* Replace both old banks, so fallback also contains preliminary values. */
            for(i=0U;ok && i<2U;i++) {
                bank=(uint8_t)(newest==0?1U:0U);if(i)bank^=1U;
                ++record.sequence;record.crc=Pose_ParamsCrc(&record);
                {uint32_t address=W25Q64_GetCapacity()-(8U-bank)*W25Q64_SECTOR_SIZE;
                 ok=(uint8_t)(W25Q64_EraseSector(address)==W25Q64_OK &&
                     W25Q64_WriteVerified(address,(const uint8_t *)&record,sizeof(record),verify,sizeof(verify))==W25Q64_OK);}
            }
        }
    }
    W25Q64_StorageUnlock();
    if(ok && !s_prelim_ready){s_prelim_ready=1U;if(!Pose_ParamsSave()){s_prelim_ready=0U;return 0U;}}
    return ok;
#endif
}
static uint8_t Profile_Command(const char *line)
{
    uint8_t ok=0U;arm_servo_status_t grip,table;const char *target;char out[180];
    if(strncmp(line,"arm profile ",12U))return 0U;
    if(!strcmp(line,"arm profile status")) {
        (void)snprintf(out,sizeof(out),"[ARM PROFILE] active=" COMPETITION_VARIANT " prelim_ready=%u FINAL=sectors12/11+10/9 PRELIM=sectors8/7+14/13; code chooses region\r\n",s_prelim_ready);ZDT_Console_Print(out);return 1U;
    }
    target=!strcmp(line,"arm profile prelim")?"PRELIM":!strcmp(line,"arm profile final")?"FINAL":NULL;
    if(!target){ZDT_Console_Print("[ARM PROFILE] usage: prelim|final|status\r\n");return 1U;}
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
    if(ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       ChassisMotor_HasMotionDemand() || grip.ramping || table.ramping ||
       (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL)) {
        ZDT_Console_Print("[ARM PROFILE] REJECTED: wait for idle arm/chassis/servos\r\n");return 1U;
    }
    if(target[0]=='P')ok=Profile_PreparePreliminary();
    else if(COMPETITION_PRELIMINARY) {
        /* Save this code's applied data; never apply FINAL targets to PRELIM. */
        arm_ui_record_t records[2];
        if(Pose_ParamsSave() && Disc_Save() && W25Q64_StorageTryLock()) {
            ok=(uint8_t)(Pose_ParamsNewestAt(records,12U)>=0);W25Q64_StorageUnlock();
        }
    } else ok=(uint8_t)(Pose_ParamsLoad()==HAL_OK && Disc_Load()==HAL_OK);
    (void)snprintf(out,sizeof(out),"[ARM PROFILE] %s target=%s active=" COMPETITION_VARIANT " HAL=%u; no motion; flash matching code to run\r\n",
        ok?"READY":"FAILED",target,ok?0U:1U);ZDT_Console_Print(out);return 1U;
}
void ArmStartPose_PrintUiPoints(void)
{
    uint8_t area,ring;float lift,reach,dm;char line[224];
    if(Pose_RingLiftDeg(&lift)!=HAL_OK){ZDT_Console_Print("[ARM POINTS] unavailable: span/travel\r\n");return;}
    for(area=0U;area<2U;area++)for(ring=1U;ring<=3U;ring++) {
        float *trim=s_ring_trim[ring-1U];
        if(Pose_RingDropBaseTargets(ring,area,&reach,&dm)!=HAL_OK)return;
        (void)snprintf(line,sizeof(line),"[ARM POINT] area=%s ring=%u base_l=%.4f base_r=%.4f base_d=%.4f dl=%.4f dr=%.4f dd=%.4f\r\n",
            area?"temp":"rough",ring,(double)lift,(double)reach,(double)dm,(double)trim[0],(double)trim[1],(double)trim[2]);ZDT_Console_Print(line);
    }
    ZDT_Console_Print("[ARM POINTS] END\r\n");
}
static uint8_t Pose_Direction(arm_driver_axis_t axis)
{uint8_t reverse=2U;if(ArmDriver_GetLinearDirection(axis,&reverse)!=HAL_OK)return 2U;return reverse;}
static uint8_t Pose_Valid(const start_profile_t *r)
{
    return
        isfinite(r->dm_deg) && r->dm_deg>=-180.0f && r->dm_deg<=185.0f && isfinite(r->dm_zero) &&
        isfinite(r->percent[0]) && r->percent[0]>=0 && r->percent[0]<=100 &&
        isfinite(r->percent[1]) && r->percent[1]>=-ARM_REACH_REFERENCE_TOLERANCE_MM/ARM_REACH_FULL_TRAVEL_MM*100.0f && r->percent[1]<=100 &&
        isfinite(r->span[0]) && r->span[0]!=0 && isfinite(r->span[1]) && r->span[1]!=0 &&
        r->direction[0]<=1U && r->direction[1]<=1U && r->esc_id<=0x7FFU && r->mst_id<=0x7FFU;
}
uint8_t ArmStartPose_IsActive(void){return s_phase>=POSE_TEACH && s_phase<=POSE_GRIP_CLOSE;}
uint8_t ArmStartPose_IsFastTransfer(void){return ArmStartPose_IsActive() && (s_pick || s_generic);}
static float Pose_DmSpeed(void)
{return ArmDriver_RotationSpeedDps();}
static HAL_StatusTypeDef Pose_Rotate(float angle)
{return ArmDriver_RotateToDegAtSpeed(angle,Pose_DmSpeed());}

uint8_t ArmStartPose_IsAwayFromTray(void)
{
    dm_feedback_t fb;float angle;
    if(!ArmStartPose_IsActive() || !(s_pick==3U || s_pick==4U || s_generic) ||
       (s_phase!=POSE_REACH_ENABLE && s_phase!=POSE_REACH && s_phase!=POSE_LIFT && s_phase!=POSE_GRIP_CLOSE) ||
       fabsf(remainderf(s_run.dm_deg-162.8250f,360.0f))<90.0f)return 0U;
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    return ArmTask_IsEnabled() && !ArmTask_IsFaulted() && fb.enabled && fb.fb_count && fb.err_code<=1U &&
        HAL_GetTick()-fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT && ArmDriver_GetRotationDeg(&angle)==HAL_OK &&
        fabsf(angle-s_run.dm_deg)<=1.0f;
}
HAL_StatusTypeDef ArmStartPose_WaitForTray(uint16_t target_us,uint32_t selected_tick)
{
    if(!ArmStartPose_IsActive() || (s_pick!=1U && s_pick!=2U) || target_us<500U || target_us>2500U)return HAL_ERROR;
    s_tray_gate=1U;s_tray_gate_target=target_us;s_tray_gate_tick=selected_tick;return HAL_OK;
}
uint8_t ArmStartPose_IsRotationReleaseExpected(void)
{return s_pick==2U && (s_phase==POSE_TOP_ENABLE || s_phase==POSE_TOP);}
uint8_t ArmStartPose_IsReleasedPlace(void)
{return s_phase==POSE_READY && s_pick_result_valid &&
    ((s_pick==2U && ArmServo_GripIsOpen()) || (s_pick==3U && s_ring_release_handoff && ArmServo_GripOpeningOrOpen()));}
uint8_t ArmStartPose_IsReleaseOpening(void)
{return (s_ring_release_handoff || s_raise_opening_handoff) && ArmServo_GripOpeningOrOpen();}
void ArmStartPose_Cancel(void){s_ring_return_reach=0U;s_reach_start.active=0U;memset(&s_feedback_pause,0,sizeof(s_feedback_pause));ArmDriver_CancelReachRetry();if(s_pick)(void)ArmServo_Stop(ARM_SERVO_GRIP);s_phase=POSE_IDLE;s_native=0U;s_reach_encoder=0U;s_pick=0U;s_turn_pending=s_dm_wait_reach=0U;s_generic=0U;s_tray_gate=0U;s_raise_overlap=s_lower_recovery=0U;s_pick_result_valid=0U;s_place_track=0U;s_ring_release_handoff=s_raise_opening_handoff=0U;}
void ArmStartPose_ReleaseForCamera(void)
{
    if(ArmStartPose_IsReleasedPlace())s_pick=0U; /* Preserve the accepted OPEN ramp during camera handoff. */
    ArmStartPose_Cancel();
}
static HAL_StatusTypeDef Pose_Configure(float lift_deg,float reach_deg,float dm_deg,uint8_t source)
{
    start_profile_t p;arm_cal_axis_status_t axis;uint8_t i;float degrees[2];
    memset(&p,0,sizeof(p));degrees[0]=lift_deg;degrees[1]=reach_deg;
    for(i=0;i<2;i++) {
        arm_driver_axis_t which=i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT;
        ArmCalibration_GetAxisStatus(which,&axis);if(!axis.zero_valid)return HAL_ERROR;
        p.zero[i]=axis.zero_counts;p.direction[i]=Pose_Direction(which);
        if(!isfinite(degrees[i]) || ArmCalibration_GetSpan(which,&p.span[i])!=HAL_OK)return HAL_ERROR;
        p.percent[i]=degrees[i]/360.0f*65536.0f/p.span[i]*100.0f;
    }
    p.dm_deg=dm_deg;if(ArmDriver_GetRotationZeroRaw(&p.dm_zero)!=HAL_OK)return HAL_ERROR;
    p.esc_id=ArmTask_GetMotor()->cfg.esc_id;p.mst_id=ArmTask_GetMotor()->cfg.mst_id;
    if(!Pose_Valid(&p))return HAL_ERROR;
    s_material_direct=0U;s_saved=p;s_valid=1U;s_profile_source=source;s_phase=POSE_IDLE;s_pick=0U;s_generic=0U;s_raise_overlap=s_lower_recovery=0U;s_pick_result_valid=0U;s_place_track=0U;s_ring_return_reach=0U;return HAL_OK;
}
void ArmStartPose_Init(void)
{
    s_prelim_ready=0U;s_material_direct=0U;memset(s_disc_trim,0,sizeof(s_disc_trim));s_disc_sequence=0U;s_disc_flash_source=0U;
    memset(&s_reach_start,0,sizeof(s_reach_start));s_ring_return_reach=0U;
    s_ring_release_handoff=s_raise_opening_handoff=0U;
    memset(s_ring_trim,0,sizeof(s_ring_trim));
    s_place_track=0U;
    s_pick_result_valid=0U;
    s_valid=0U;s_phase=POSE_IDLE;s_pick=0U;s_turn_pending=s_dm_wait_reach=0U;s_pick_completed=0U;s_reach_encoder=0U;s_ring_station=2U;s_ring_temp=0U;s_generic=0U;s_quiet_restore=0U;s_place_slot=0U;s_tray_gate=0U;s_raise_overlap=0U;
    if(Pose_Configure(ARM_START_DEFAULT_LIFT_DEG,ARM_START_DEFAULT_REACH_DEG,ARM_START_DEFAULT_DM_DEG,0U)==HAL_OK) {
        char line[160];(void)snprintf(line,sizeof(line),
            "[ARM START] fixed defaults: lift=%+.4fdeg reach=%+.4fdeg DM=%+.4fdeg; NO W25 read/write\r\n",
            (double)ARM_START_DEFAULT_LIFT_DEG,(double)ARM_START_DEFAULT_REACH_DEG,(double)ARM_START_DEFAULT_DM_DEG);
        ZDT_Console_Print(line);
    }
    else ZDT_Console_Print("[ARM START] defaults unavailable: establish axis references first\r\n");
}
static void Pose_Fail(const char *why)
{
    s_pick_result_valid=0U;
    uint8_t teaching=(s_phase==POSE_TEACH);
    if(s_pick)ZDT_Console_Print(s_pick==3U?"[ARM RING] FAILED; detailed pose diagnostics follow\r\n":s_pick==2U?"[ARM PLACE] FAILED; detailed pose diagnostics follow\r\n":"[ARM PICK] FAILED; detailed pose diagnostics follow\r\n");
    uint8_t failed_phase=s_phase,i;arm_cal_axis_status_t a;dm_feedback_t fb;
    float angle=0.0f;char line[192];
    DM_GetFeedback(ArmTask_GetMotor(),&fb);(void)ArmDriver_GetRotationDeg(&angle);
    s_phase=POSE_FAILED;
    if(!teaching){(void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();}
    s_phase=POSE_FAILED;
    ZDT_Console_Print("[ARM START] FAILED: ");ZDT_Console_Print(why);ZDT_Console_Print("\r\n");
    (void)snprintf(line,sizeof(line),"[ARM START] phase=%u DM actual=%+.3f target=%+.3fdeg code=%u enabled=%u age=%lums\r\n",
        failed_phase,(double)angle,(double)s_run.dm_deg,fb.err_code,fb.enabled,
        (unsigned long)(HAL_GetTick()-fb.last_tick));ZDT_Console_Print(line);
    for(i=0U;i<2U;i++) {
        ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a);
        (void)snprintf(line,sizeof(line),"[ARM START] id=%u actual=%+.3f target=%+.3fdeg raw=%lld zero=%lld flags=0x%02X pos_age=%lums flags_age=%lums\r\n",
            (unsigned int)(i?6U:5U),(double)(a.position_counts-a.zero_counts)*360.0/65536.0,
            (double)s_run.percent[i]*s_run.span[i]/100.0*360.0/65536.0,(long long)a.position_counts,
            (long long)a.zero_counts,a.flags,(unsigned long)(HAL_GetTick()-a.position_tick),
            (unsigned long)(HAL_GetTick()-a.flags_tick));ZDT_Console_Print(line);
    }
}
static uint8_t Pose_Axes(arm_cal_axis_status_t a[2],uint32_t now)
{
    uint8_t i;
    for(i=0;i<2;i++) {
        ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a[i]);
        if(!a[i].zero_valid || !a[i].position_sequence || !a[i].flags_sequence ||
           now-a[i].position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || now-a[i].flags_tick>ArmCalibration_StatusTimeoutMs(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT) ||
           (a[i].flags&ARM_CAL_FLAG_PROTECTED) ||
           (ArmCalibration_StallFault(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,a[i].flags) && !(i && s_phase==POSE_REACH && ArmDriver_ReachRetryActive() &&
             (ArmDriver_ReachRetryRecovering() || ArmDriver_ReachStallPending()))))return 0;
    }
    return 1;
}
static void Pose_Enable(uint8_t i,uint32_t now)
{
    arm_cal_axis_status_t a={0};ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a);
    s_pos_seq[i]=a.position_sequence;s_flag_seq[i]=a.flags_sequence;s_tick=now;
    if(ArmDriver_Enable(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT)!=HAL_OK){Pose_Fail("axis enable rejected");return;}
    s_phase=i?POSE_REACH_ENABLE:POSE_TOP_ENABLE;
}
static HAL_StatusTypeDef Pose_OpenGrip(uint32_t now)
{
    uint8_t released_open=(uint8_t)(s_ring_release_handoff && ArmServo_GripOpeningOrOpen());
    s_ring_release_handoff=0U;
    s_raise_opening_handoff=(uint8_t)(released_open && !ArmServo_GripIsOpen());
    if(ArmServo_GripOpenConfirmed() || released_open) {
        Pose_Enable(0U,now);
        ZDT_Console_Print(released_open?"[ARM GRIP] ring OPEN >=200ms; keep PWM ramp, start rise without duplicate OPEN\r\n":
            "[ARM GRIP] OPEN PWM already settled; skip duplicate open/500ms wait\r\n");
        return s_phase==POSE_TOP_ENABLE?HAL_OK:HAL_ERROR;
    }
    if(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ArmServo_GripOpenDeg())!=HAL_OK) {
        Pose_Fail("grip open rejected");return HAL_ERROR;
    }
    s_phase=POSE_GRIP_OPEN;s_tick=now;return HAL_OK;
}
static uint8_t Pose_TargetReady(void)
{
    arm_cal_axis_status_t a[2]={{0}};dm_feedback_t fb;float p,angle,zero;uint8_t i;
    if(s_phase!=POSE_READY || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       !Pose_Axes(a,HAL_GetTick()) || ArmDriver_GetRotationZeroRaw(&zero)!=HAL_OK ||
       fabsf(Pose_Wrap(zero-s_run.dm_zero))>0.001f)return 0U;
    for(i=0;i<2;i++) {
        if(!(a[i].flags&ARM_CAL_FLAG_ENABLED) || a[i].zero_counts!=s_run.zero[i] ||
           ArmCalibration_GetPercent(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&p)!=HAL_OK ||
           fabsf(p-s_run.percent[i])>(i?ARM_PERCENT_TOLERANCE:ArmCalibration_LiftTolerancePercent(s_run.percent[0],a[0].travel_mm)))return 0U;
    }
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    return ArmTask_IsEnabled() && !ArmTask_IsFaulted() && fb.enabled && fb.fb_count &&
        HAL_GetTick()-fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT && (fb.err_code==0U || fb.err_code==1U) &&
        isfinite(fb.vel_rads) && fabsf(fb.vel_rads)<=POSE_GRIP_HOLD_DM_RAD_S && ArmDriver_GetRotationDeg(&angle)==HAL_OK &&
        fabsf(angle-s_run.dm_deg)<=1.0f;
}
uint8_t ArmStartPose_IsReady(void)
{return s_valid && !s_generic && !s_pick && Pose_TargetReady();}
static const char *Pose_GripHoldFailure(void)
{
    arm_cal_axis_status_t a[2]={{0}};dm_feedback_t fb;float p,angle,zero,tolerance,hold_mm;uint8_t i;
    if(ArmCalibration_GetState()!=ARM_CAL_IDLE)return "grip finish: percent action not idle";
    if(!Pose_Axes(a,HAL_GetTick()))return "grip finish: axis reference/stall/protection/stale";
    if(ArmDriver_GetRotationZeroRaw(&zero)!=HAL_OK || fabsf(Pose_Wrap(zero-s_run.dm_zero))>0.001f)
        return "grip finish: DM zero reference changed";
    for(i=0;i<2;i++) {
        if(!(a[i].flags&ARM_CAL_FLAG_ENABLED) || a[i].zero_counts!=s_run.zero[i])return "grip finish: axis disabled/zero changed";
        if(!isfinite(a[i].travel_mm) || a[i].travel_mm<=0.0f ||
           ArmCalibration_GetPercent(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&p)!=HAL_OK || !isfinite(p))return "grip finish: axis position invalid";
        tolerance=i?Pose_ReachTolerance():ArmCalibration_LiftTolerancePercent(s_run.percent[0],a[0].travel_mm);
        hold_mm=i?POSE_GRIP_HOLD_MM:POSE_GRIP_LIFT_HOLD_MM;
        if(tolerance<hold_mm/a[i].travel_mm*100.0f)tolerance=hold_mm/a[i].travel_mm*100.0f;
        if(fabsf(p-s_run.percent[i])>tolerance)return i?"grip finish: reach drift beyond hold tolerance":"grip finish: lift drift beyond hold tolerance";
    }
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    if(!ArmTask_IsEnabled() || ArmTask_IsFaulted() || !fb.enabled || !fb.fb_count ||
       HAL_GetTick()-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT || fb.err_code>1U ||
       !isfinite(fb.vel_rads) || fabsf(fb.vel_rads)>POSE_GRIP_HOLD_DM_RAD_S || ArmDriver_GetRotationDeg(&angle)!=HAL_OK)
        return "grip finish: DM disabled/fault/stale/unsettled";
    if(fabsf(angle-s_run.dm_deg)>POSE_GRIP_HOLD_DM_DEG)return "grip finish: DM drift beyond2deg";
    if(ArmDriver_CheckRawRotationTarget(fb.pos_rad)!=HAL_OK)return "grip finish: existing height/angle guard";
    return NULL;
}
uint8_t ArmStartPose_ReadActionResult(uint8_t *success)
{
    if(!success || ArmStartPose_IsActive())return 0U;
    /* A completed grip action is an outcome, not a second live-pose query.
     * Validate once BEFORE DONE; do not undo release for a later tiny drift. */
    *success=s_pick?(s_phase==POSE_READY && s_pick_result_valid):Pose_TargetReady();return 1U;
}
HAL_StatusTypeDef ArmStartPose_BeginMove(float lift,float reach,float dm)
{
    start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE ||
       ChassisMotor_HasMotionDemand())return HAL_BUSY;
    if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)return HAL_ERROR;
    s_quiet_restore=1U;(void)ArmStartPose_Command("arm start");s_quiet_restore=0U;
    s_saved=saved;s_valid=valid;s_profile_source=source;
    if(s_phase!=POSE_TOP_ENABLE)return HAL_ERROR;
    s_generic=1U;return HAL_OK;
}
HAL_StatusTypeDef ArmStartPose_BeginMaterialApproach(float lift,float reach,float dm)
{
    start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;dm_feedback_t fb;
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE || ChassisMotor_HasMotionDemand() ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)return HAL_ERROR;
    s_run=s_saved;s_saved=saved;s_valid=valid;s_profile_source=source;
    s_pick=0U;s_generic=1U;s_material_direct=1U;s_turn_pending=s_dm_wait_reach=s_tray_gate=0U;
    s_native=s_reach_encoder=0U;s_reach_start.active=0U;
    if(ArmDriver_Enable(ARM_DRIVER_ROTATION)!=HAL_OK || Pose_Rotate(dm)!=HAL_OK) {
        Pose_Fail("direct material DM target rejected");return HAL_ERROR;
    }
    s_phase=POSE_DM;s_samples=0U;s_tick=s_started=HAL_GetTick();
    DM_GetFeedback(ArmTask_GetMotor(),&fb);s_dm_seq=fb.fb_count;
    ZDT_Console_Print("[ARM MATERIAL APPROACH] DIRECT: DM -> hold wait reach -> taught lift; NO clearance/TOP waypoint, grip unchanged\r\n");
    return HAL_OK;
}
static HAL_StatusTypeDef Pose_BeginRingDrop(uint8_t station,float rise_mm,uint8_t temp)
{
    start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;
    arm_cal_axis_status_t a;float lift,reach,dm,span;char reply[192];
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(station<1U || station>3U || !isfinite(rise_mm) || rise_mm<0.0f)return HAL_ERROR;
    if(ArmStartPose_IsActive() || ChassisMotor_HasMotionDemand() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    if(!s_pick_completed || !ArmServo_GripIsClosed()){
        ZDT_Console_Print("[ARM RING] refused: successful PICK and closed grip required\r\n");return HAL_ERROR;
    }
    if(Pose_RingLiftDeg(&lift)!=HAL_OK || Pose_RingDropTargets(station,temp,&reach,&dm)!=HAL_OK)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a);
    if(ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span)!=HAL_OK || !isfinite(a.travel_mm) || a.travel_mm<=0.0f)return HAL_ERROR;
    lift+=s_ring_trim[station-1U][0];
    lift-=span/a.travel_mm*rise_mm*360.0f/65536.0f;
    if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)return HAL_ERROR;
    (void)ArmStartPose_Command("arm start");s_saved=saved;s_valid=valid;s_profile_source=source;
    if(s_phase!=POSE_TOP_ENABLE)return HAL_ERROR;
    s_pick=3U;s_ring_station=station;s_ring_temp=temp;
    (void)snprintf(reply,sizeof(reply),"[ARM RING] area=%s ring=%u half-rise -> DM%+.4f reach%+.4f; clearance/DM arrived -> lift%+.4fdeg (ground+%.1fmm); reach live -> open\r\n",
        temp?"TEMP":"ROUGH",station,(double)dm,(double)reach,(double)lift,(double)rise_mm);ZDT_Console_Print(reply);return HAL_OK;
}
HAL_StatusTypeDef ArmStartPose_BeginRingDrop(uint8_t station,float rise_mm)
{return Pose_BeginRingDrop(station,rise_mm,0U);}
HAL_StatusTypeDef ArmStartPose_BeginTempRingDrop(uint8_t station,float rise_mm)
{return Pose_BeginRingDrop(station,rise_mm,1U);}
HAL_StatusTypeDef ArmStartPose_BeginTrayPlaceDuringRaise(uint8_t slot,float raise_start_percent)
{
    start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;
    uint8_t ring_return=(uint8_t)(s_pick==4U);
    float lift,reach,dm,current;arm_cal_axis_status_t a[2]={{0}};
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(slot<1U || slot>3U || !isfinite(raise_start_percent) || raise_start_percent<=0.0f || raise_start_percent>100.0f)return HAL_ERROR;
    if(ArmStartPose_IsActive() || ChassisMotor_HasMotionDemand() || !ArmCalibration_IsLiftRaisingToTop() ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    if(!ArmServo_GripIsClosed() || !Pose_Axes(a,HAL_GetTick()) ||
       !(a[0].flags&ARM_CAL_FLAG_ENABLED) || !(a[1].flags&ARM_CAL_FLAG_ENABLED) ||
       ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current) ||
       current>raise_start_percent+ArmCalibration_LiftTolerancePercent(raise_start_percent,a[0].travel_mm) ||
       Pose_PlaceLiftDeg(&lift)!=HAL_OK || Pose_TrayPlaceTargets(slot,&reach,&dm)!=HAL_OK)return HAL_ERROR;
    if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)return HAL_ERROR;
    s_run=s_saved;s_saved=saved;s_valid=valid;s_profile_source=source;
    s_pick=2U;s_place_slot=slot;s_generic=s_tray_gate=s_turn_pending=s_dm_wait_reach=0U;
    s_raise_overlap=1U;s_raise_grip_closed=1U;s_raise_half_percent=(raise_start_percent+Pose_ClearancePercent())*0.5f;
    s_phase=POSE_TOP;s_tick=s_started=HAL_GetTick();
    s_ring_return_reach=ring_return?1U:0U;s_ring_return_raise_start=raise_start_percent;
    ZDT_Console_Print(ring_return?"[ARM PLACE] adopt ring TOP raise; start reach together NOW, DM keeps original half-rise\r\n":
        "[ARM PLACE] adopt existing true TOP raise; grip fully closed; DM/reach wait actual half-rise\r\n");
    return HAL_OK;
}
HAL_StatusTypeDef ArmStartPose_BeginRingPick(uint8_t station)
{
    start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;
    float reach,dm,lift;arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE || ChassisMotor_HasMotionDemand() ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    if(ArmServo_GripIsClosed() || Pose_RingDropTargets(station,0U,&reach,&dm)!=HAL_OK)return HAL_ERROR;
    if(Pose_RingLiftDeg(&lift)!=HAL_OK)return HAL_ERROR;
    lift+=s_ring_trim[station-1U][0];
    if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)return HAL_ERROR;
    s_quiet_restore=1U;(void)ArmStartPose_Command("arm start");s_quiet_restore=0U;
    s_saved=saved;s_valid=valid;s_profile_source=source;
    if(s_phase!=POSE_TOP_ENABLE)return HAL_ERROR;
    s_pick=4U;s_ring_station=station;s_ring_temp=0U;s_pick_completed=0U;
    if(Pose_OpenGrip(HAL_GetTick())!=HAL_OK)return HAL_ERROR;
    ZDT_Console_Print("[ARM RING PICK] open -> TOP-40mm -> shared PLACE reach held + DM within15deg -> lower; CLOSE at lift<=3mm AND final DM arrived -> tray\r\n");
    return HAL_OK;
}
uint8_t ArmStartPose_ReadResult(uint8_t *success)
{
    if(!success || ArmStartPose_IsActive())return 0U;
    *success=ArmStartPose_IsReady();return 1U;
}
HAL_StatusTypeDef ArmStartPose_BeginRestore(void)
{
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(ArmStartPose_IsActive() || ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE ||
       ArmCalibration_GetState()==ARM_CAL_PERCENT_SETTLE ||
       (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) || camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    (void)ArmStartPose_Command("arm start");
    return s_phase==POSE_TOP_ENABLE?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Pose_ReachNative(const arm_cal_axis_status_t *axis,uint32_t now)
{
    float current;double delta,timeout;int32_t pulses;uint16_t speed=ArmDriver_ReachMotionSpeedRpm();
    s_reach_start.active=0U;
    {uint16_t cap=ArmServo_GripIsOpen()?ARM_CAMERA_REACH_RPM_MAX:ARM_LOADED_REACH_RPM_MAX;
     if(speed>cap)speed=cap;}
    if(!speed || ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK)return HAL_ERROR;
    s_native=0U;s_reach_encoder=0U;
    if(s_pick!=3U && fabsf(current-s_run.percent[1])<=Pose_ReachTolerance()) {
        if(ArmStartPose_IsFastTransfer()) {
            if(ArmDriver_StopReachForHold()!=HAL_OK)return HAL_ERROR;
            ArmDriver_CancelReachRetry();
        }
        return HAL_OK;
    }
    delta=(double)(s_run.percent[1]-current)*s_run.span[1]/100.0;
    /* Match camera's effective pulse/encoder conversion, including signed
     * retraction. Emm pulses are not the 65536-count encoder coordinate. */
    if(ArmCalibration_GetAbsoluteReachPulses(s_run.percent[1],&pulses)!=HAL_OK)return HAL_ERROR;
    if(s_pick==2U) {
        memset(&s_reach_start,0,sizeof(s_reach_start));
        s_reach_start.ack_fence=ArmCalibration_PositionAckFence(ARM_DRIVER_REACH);
        s_reach_start.ack_seen=s_reach_start.ack_fence;s_reach_start.started=HAL_GetTick();
        s_reach_start.initial=current;s_reach_start.target=s_run.percent[1];
        s_reach_start.position_sequence=axis->position_sequence;
    }
    if(ArmDriver_ReachAbsolutePulses(pulses,speed,ARM_CAMERA_POSITION_ACCEL)!=HAL_OK)return HAL_ERROR;
    s_native_retry_generation=ArmDriver_ReachRetryGeneration();
    if(s_pick==2U) {
        s_reach_start.goal=pulses;s_reach_start.speed=speed;s_reach_start.attempts=1U;
        s_reach_start.last_send=HAL_GetTick();s_reach_start.generation=s_native_retry_generation;s_reach_start.active=1U;
    }
    timeout=fabs(delta)/65536.0*60000.0/(double)speed*2.0+5000.0;
    if(timeout<10000.0)timeout=10000.0;
    if(timeout>120000.0)timeout=120000.0;
    s_native=1U;s_native_samples=0U;s_native_stopped=0U;s_native_retries=0U;s_native_quiet_since=0U;s_native_started=now;s_native_timeout=(uint32_t)timeout;
    s_native_sequence=axis->position_sequence;s_native_last=axis->position_counts;
    s_native_initial=current;
    if(s_pick==3U) {
        s_place_track=1U;s_place_track_near=(uint8_t)(fabsf(current-s_run.percent[1])<=ARM_PERCENT_TOLERANCE);s_place_track_outside=0U;
        s_place_track_sequence=axis->position_sequence;s_place_track_last_counts=axis->position_counts;
        s_place_track_goal_pulses=pulses;s_place_track_started=s_place_track_command_tick=now;
        ZDT_Console_Print("[ARM PLACE TRACK] placement reach dead zone +/-5deg; outside read/correct during DM turn/lowering; no extra settle\r\n");
    }
    {
        char line[170];(void)snprintf(line,sizeof(line),"[ARM START] reach NATIVE mode=01 ABS goal_pulses=%ld from=%.3f%% target=%.3f%% speed=%uRPM acc=%u\r\n",
            (long)pulses,(double)current,(double)s_run.percent[1],(unsigned int)speed,ARM_CAMERA_POSITION_ACCEL);ZDT_Console_Print(line);
    }
    return HAL_OK;
}
static uint8_t Pose_ReachStartupService(void)
{
    arm_cal_position_ack_t ack={0};arm_cal_axis_status_t axis;uint32_t now;
    float actual,progress,threshold;char line[224];HAL_StatusTypeDef result;
    if(!s_reach_start.active)return 1U;
    if(s_pick!=2U || (s_phase!=POSE_REACH && !((s_phase==POSE_TOP || s_phase==POSE_REACH_ENABLE) && s_ring_return_reach==3U)) || !s_native ||
       s_reach_start.generation!=ArmDriver_ReachRetryGeneration()) {s_reach_start.active=0U;return 1U;}
    if(ArmDriver_ReachRetryRecovering() || ArmDriver_ReachStallPending())return 1U;
    ArmCalibration_GetPositionAck(ARM_DRIVER_REACH,&ack);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&axis);now=HAL_GetTick();
    if((int32_t)(ack.sequence-s_reach_start.ack_fence)>0 && ack.sequence!=s_reach_start.ack_seen &&
       (int32_t)(ack.received_tick-s_reach_start.started)>=0) {
        uint8_t report=(uint8_t)(!s_reach_start.acked);
        s_reach_start.ack_seen=ack.sequence;
        if(ack.response==0x02U || ack.response==0x9FU)s_reach_start.acked=1U;
        if(report){(void)snprintf(line,sizeof(line),"[ARM PLACE REACH ACK] id6 response=0x%02X accepted=%u send=%u/%u SAME ABS=%ld; reception is NOT start/arrival\r\n",
            ack.response,s_reach_start.acked,s_reach_start.attempts,POSE_REACH_START_SENDS,(long)s_reach_start.goal);ZDT_Console_Print(line);}
    }
    threshold=fminf(ARM_PERCENT_TOLERANCE,fabsf(s_reach_start.target-s_reach_start.initial)*0.25f);
    if(axis.position_sequence!=s_reach_start.position_sequence &&
       (int32_t)(axis.position_tick-s_reach_start.started)>=0 && now-axis.position_tick<=ARM_ZDT_FEEDBACK_TIMEOUT_MS &&
       ArmCalibration_GetPercent(ARM_DRIVER_REACH,&actual)==HAL_OK && isfinite(actual)) {
        progress=fabsf(s_reach_start.initial-s_reach_start.target)-fabsf(actual-s_reach_start.target);
        if(progress>=threshold) {
            s_reach_start.active=0U;
            (void)snprintf(line,sizeof(line),"[ARM PLACE REACH START] actual=%.3f%% toward=%.3f%% progress=%.3f%% sends=%u ACK=%u; stop replay, retain original arrival/deadline\r\n",
                (double)actual,(double)s_reach_start.target,(double)progress,s_reach_start.attempts,s_reach_start.acked);ZDT_Console_Print(line);return 1U;
        }
    }
    if(now-s_reach_start.last_send<POSE_REACH_START_RESEND_MS)return 1U;
    if(s_reach_start.attempts>=POSE_REACH_START_SENDS) {
        if(!s_reach_start.exhausted){s_reach_start.exhausted=1U;ZDT_Console_Print("[ARM PLACE REACH WAIT] id6 50 sends used; no confirmed start; original owner timeout retained\r\n");}
        return 1U;
    }
    s_reach_start.last_send=now;result=ArmDriver_ReplayReachAbsolutePulses(s_reach_start.goal);
    if(result==HAL_BUSY)return 1U;
    if(result!=HAL_OK){Pose_Fail("reach startup replay CAN rejected");return 0U;}
    ++s_reach_start.attempts;
    /* Report sparsely: printing every10ms would exceed UART4 at115200baud. */
    if(s_reach_start.attempts==2U || s_reach_start.attempts%10U==0U || s_reach_start.attempts==POSE_REACH_START_SENDS) {
        (void)snprintf(line,sizeof(line),"[ARM PLACE REACH RETRY] id6 send=%u/%u SAME ABS=%ld speed=%uRPM acc=%u ACK=%u gap=%ums; no forward progress, no extra STOP, deadline unchanged\r\n",
            s_reach_start.attempts,POSE_REACH_START_SENDS,(long)s_reach_start.goal,s_reach_start.speed,ARM_CAMERA_POSITION_ACCEL,s_reach_start.acked,POSE_REACH_START_RESEND_MS);ZDT_Console_Print(line);
    }
    return 1U;
}
static const char *Pose_NativeReachTravelFailure(float current)
{
    float error=s_run.percent[1]-current,initial=s_run.percent[1]-s_native_initial;
    if((current-s_native_initial)*initial<0.0f && fabsf(current-s_native_initial)>1.0f)return "native reach wrong direction";
    if(error*initial<0.0f && fabsf(error)>5.0f)return "native reach overshoot";
    return NULL;
}
static uint8_t Pose_RingReturnReachEarly(const arm_cal_axis_status_t a[2],uint32_t now)
{
    float current,rise;char line[176];
    if(!s_ring_return_reach)return 1U;
    if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current)) {Pose_Fail("lift feedback unavailable during ring return");return 0U;}
    rise=(s_ring_return_raise_start-current)*a[0].travel_mm/100.0f;
    if(s_ring_return_reach==1U) {
        s_pos_seq[1]=a[1].position_sequence;s_flag_seq[1]=a[1].flags_sequence;
        if(ArmDriver_Enable(ARM_DRIVER_REACH)!=HAL_OK){Pose_Fail("axis enable rejected");return 0U;}
        s_ring_return_reach_tick=HAL_GetTick();s_ring_return_reach=2U;
        (void)snprintf(line,sizeof(line),"[ARM RING RETURN] actual rise=%.2fmm; enable reach NOW, DM still waits original half-rise\r\n",(double)rise);ZDT_Console_Print(line);return 1U;
    }
    if(s_ring_return_reach==2U) {
        if(now-s_ring_return_reach_tick>1000U){Pose_Fail("enable confirmation timeout");return 0U;}
        if(a[1].position_sequence==s_pos_seq[1] || a[1].flags_sequence==s_flag_seq[1] ||
           (int32_t)(a[1].position_tick-s_ring_return_reach_tick)<0 || (int32_t)(a[1].flags_tick-s_ring_return_reach_tick)<0 ||
           !(a[1].flags&ARM_CAL_FLAG_ENABLED))return 1U;
        if(Pose_ReachNative(&a[1],now)!=HAL_OK){Pose_Fail("target rejected");return 0U;}
        s_ring_return_reach=3U;
        (void)snprintf(line,sizeof(line),"[ARM RING RETURN] reach issued: rise=%.2fmm enable_wait=%lums; keep same job/ACK/deadline; DM half-rise unchanged\r\n",
            (double)rise,(unsigned long)(HAL_GetTick()-s_ring_return_reach_tick));ZDT_Console_Print(line);
    }
    if(s_native && !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachStallPending() &&
       s_native_retry_generation==ArmDriver_ReachRetryGeneration()) {
        const char *failure;
        if(now-s_native_started>s_native_timeout){Pose_Fail("native reach timeout");return 0U;}
        if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Pose_Fail("reach feedback invalid");return 0U;}
        failure=Pose_NativeReachTravelFailure(current);if(failure){Pose_Fail(failure);return 0U;}
    }
    return Pose_ReachStartupService();
}
static float Pose_PlaceReachErrorDeg(float current)
{return fabsf((current-s_run.percent[1])*s_run.span[1]/100.0f*360.0f/65536.0f);}
static uint8_t Pose_PlaceReachAtRelease(void)
{
    arm_cal_axis_status_t reach;float current;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
    return !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachStallPending() &&
        reach.position_sequence && HAL_GetTick()-reach.position_tick<=ARM_ZDT_FEEDBACK_TIMEOUT_MS &&
        !ArmCalibration_FaultFlags(ARM_DRIVER_REACH,reach.flags) && reach.travel_mm>0.0f &&
        ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)==HAL_OK && isfinite(current) &&
        Pose_PlaceReachErrorDeg(current)<=POSE_PLACE_REACH_DEADBAND_DEG;
}
static void Pose_PlaceReachTrack(uint32_t now)
{
    arm_cal_axis_status_t reach;float current,error_mm,error_deg,error,initial;double movement;HAL_StatusTypeDef corrected;
    if(!s_place_track)return;
    if(ArmDriver_ReachRetryRecovering()) {
        if(s_phase==POSE_LIFT) {
            /* Keep the same native retry/budget. Freeze5 without cancelling
             * its parent pose or6; existing REACH recovery holds DM and
             * waits actual6 target before resuming the recorded descent. */
            if(!ArmServo_GripIsClosed()){Pose_Fail("reach recovery before release: closed grip lost");return;}
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&s_lower_hold_percent)!=HAL_OK || !isfinite(s_lower_hold_percent)){Pose_Fail("lift position unavailable for reach recovery");return;}
            {
                arm_cal_axis_status_t lift;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
                s_lower_hold_sequence=lift.position_sequence;s_lower_hold_last=lift.position_counts;
            }
            if(ArmCalibration_HoldLiftForReachRecovery()!=HAL_OK){Pose_Fail("lift hold for reach recovery rejected");return;}
            s_lower_hold_pending=1U;s_lower_hold_samples=0U;s_lower_hold_tick=HAL_GetTick();
            s_phase=POSE_REACH;s_raise_overlap=0U;s_lower_recovery=1U;s_tick=now;
            ZDT_Console_Print("[ARM PLACE RECOVERY] hold lift5 at current height, keep grip CLOSED; retry6 then resume recorded descent/open\r\n");
        } else if(s_phase==POSE_GRIP_CLOSE)Pose_Fail("reach recovery after release started; stop");
        return;
    }
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);now=HAL_GetTick();
    if(!reach.flags_sequence || now-reach.flags_tick>ARM_ZDT_STATUS_TIMEOUT_MS ||
       !(reach.flags&ARM_CAL_FLAG_ENABLED) || (reach.flags&ARM_CAL_FLAG_PROTECTED)) {
        Pose_Fail("placement reach disabled/protected/stale flags");return;
    }
    if(!reach.position_sequence || now-reach.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS ||
       !isfinite(reach.travel_mm) || reach.travel_mm<=0.0f || ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK || !isfinite(current)) {
        Pose_Fail("placement reach tracking feedback invalid");return;
    }
    error=s_run.percent[1]-current;initial=s_run.percent[1]-s_native_initial;
    if(s_native_retry_generation!=ArmDriver_ReachRetryGeneration()) {
        s_native_retry_generation=ArmDriver_ReachRetryGeneration();s_place_track_started=now;
        s_native_timeout=ArmDriver_ReachRetryTimeoutMs((double)error*s_run.span[1]/100.0);
        s_native_initial=current;s_place_track_near=s_place_track_outside=0U;initial=error;
    }
    if(current<-ARM_REACH_REFERENCE_TOLERANCE_MM/reach.travel_mm*100.0f || current>100.0f+ARM_REACH_REFERENCE_TOLERANCE_MM/reach.travel_mm*100.0f ||
       (!s_place_track_near && fabsf(initial)>ARM_PERCENT_TOLERANCE &&
        (((current-s_native_initial)*initial<0.0f && fabsf(current-s_native_initial)>1.0f) ||
         (error*initial<0.0f && fabsf(error)>5.0f)))) {Pose_Fail("placement reach tracking direction/range");return;}
    error_mm=fabsf(error)*reach.travel_mm/100.0f;
    error_deg=Pose_PlaceReachErrorDeg(current);
    if(now-s_place_track_started>s_native_timeout && error_deg>POSE_PLACE_REACH_DEADBAND_DEG) {Pose_Fail("placement reach tracking timeout");return;}
    if(reach.position_sequence==s_place_track_sequence)return;
    s_place_track_sequence=reach.position_sequence;movement=fabs((double)reach.position_counts-(double)s_place_track_last_counts);s_place_track_last_counts=reach.position_counts;
    if(fabsf(error)<=ARM_PERCENT_TOLERANCE)s_place_track_near=1U;
    if(!s_place_track_near && fabsf(error)<=2.0f)s_place_track_near=1U;
    if(s_place_track_near && fabsf(error)>2.0f){Pose_Fail("placement reach drift exceeds small correction range");return;}
    if(error_deg<=POSE_PLACE_REACH_DEADBAND_DEG || movement>fabs((double)s_run.span[1])*0.0003) {s_place_track_outside=0U;return;}
    if(!s_place_track_near || ArmDriver_ReachStallPending())return;
    if(s_place_track_outside<2U)++s_place_track_outside;
    if(s_place_track_outside<2U || now-s_place_track_command_tick<POSE_PLACE_CORRECTION_GAP_MS)return;
    corrected=ArmDriver_ReassertReachAbsolutePulses(s_place_track_goal_pulses,POSE_PLACE_CORRECTION_RPM);
    if(corrected==HAL_BUSY)return;
    if(corrected!=HAL_OK){Pose_Fail("placement reach correction send failed");return;}
    s_place_track_command_tick=HAL_GetTick();s_place_track_outside=0U;
    s_native_retry_generation=ArmDriver_ReachRetryGeneration();
    {char line[190];(void)snprintf(line,sizeof(line),"[ARM PLACE TRACK] phase=%u actual=%+.3f target=%+.3fdeg err=%.3fmm/%.3fdeg >5deg SAME ABS=%ld <=18RPM\r\n",
        s_phase,(double)current*s_run.span[1]/100.0*360.0/65536.0,(double)s_run.percent[1]*s_run.span[1]/100.0*360.0/65536.0,(double)error_mm,(double)error_deg,(long)s_place_track_goal_pulses);ZDT_Console_Print(line);}
}
static uint8_t Pose_FinalDescentAllowed(float angle,float travel_mm)
{
    float tolerance=s_pick==2U?POSE_TRAY_PLACE_DESCEND_DM_DEG:POSE_EARLY_DESCEND_DM_DEG;
    if(!s_turn_pending)return 1U;
    if(!Pose_UsesEarlyLower() || s_dm_wait_reach || fabsf(angle-s_run.dm_deg)>tolerance)return 0U;
    /* Keep the existing low-height angular sector. Wait rather than issue
     * an overlapping descent that the native height guard would reject. */
    return ArmDriver_CheckUiPose(s_run.percent[0]*travel_mm/100.0f,angle)==HAL_OK;
}
static void Pose_BeginFinalDescent(uint32_t now)
{
    if(s_tray_gate) {
        arm_servo_status_t table;ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
        if(!table.enabled || table.fault || table.target_us!=s_tray_gate_target){Pose_Fail("tray output changed/unavailable before descent");return;}
        if(table.ramping && now-s_tray_gate_tick>5000U){Pose_Fail("tray ramp timeout before descent");return;}
        if(table.ramping || now-s_tray_gate_tick<ARM_SERVO_TABLE_MIN_WAIT_MS)return;
        s_tray_gate=0U;ZDT_Console_Print("[ARM TRAY] parallel selection settled; final descent permitted\r\n");
    }
    if(!s_place_track)ArmDriver_CancelReachRetry();
    if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,s_run.percent[0])!=HAL_OK){Pose_Fail("recorded lift target rejected by height guard");return;}
    if(s_lower_recovery){s_lower_recovery=0U;ZDT_Console_Print("[ARM PLACE RECOVERY] reach/DM recovered; resume original lift endpoint, then open only at actual target\r\n");}
    s_phase=POSE_LIFT;
    if(s_pick) {
        char line[160];float lift_deg=s_run.percent[0]*s_run.span[0]/100.0f*360.0f/65536.0f;
        (void)snprintf(line,sizeof(line),s_pick==2U && s_turn_pending?
            "[ARM %s] 3/4 DM within5deg; turn CONTINUES while lowering to %+.4fdeg%s\r\n":Pose_UsesEarlyLower() && s_turn_pending?
            "[ARM %s] 3/4 DM within15deg; turn CONTINUES while lowering to %+.4fdeg%s\r\n":
            "[ARM %s] 3/4 DM arrived; lower lift to %+.4fdeg%s\r\n",
            s_pick==4U?"RING PICK":s_pick==3U?"RING":s_pick==2U?"PLACE":"PICK",(double)lift_deg,s_place_track?"; reach tracking CONTINUES, no extra pre-descent settle":"; reach held");ZDT_Console_Print(line);
    }
}
void ArmStartPose_Poll(void)
{
    arm_cal_axis_status_t a[2]={{0}};dm_feedback_t fb;float angle,current;uint32_t now=HAL_GetTick();uint8_t i;
    if(s_ring_release_handoff && !ArmServo_GripOpeningOrOpen())s_ring_release_handoff=0U;
    if(s_raise_opening_handoff && !ArmServo_GripOpeningOrOpen())s_raise_opening_handoff=0U;
    if(s_pick_completed && !ArmServo_GripIsClosed()){s_pick_completed=0U;}
    if(!ArmStartPose_IsActive()){memset(&s_feedback_pause,0,sizeof(s_feedback_pause));return;}
    {uint32_t gap;uint8_t waiting=ArmCalibration_WaitForFeedback(&s_feedback_pause,now,&gap);
     s_started+=gap;s_tick+=gap;s_dm_started+=gap;s_native_started+=gap;
     s_native_settle+=gap;s_place_track_started+=gap;s_place_track_command_tick+=gap;
     s_ring_return_reach_tick+=gap;
     if(s_native_quiet_since)s_native_quiet_since+=gap;
     s_lower_hold_tick+=gap;
     if(waiting)return;}
    if(ArmDriver_ReachRetryFailed()){Pose_Fail("reach position failed after three-attempt recovery");return;}
    Pose_PlaceReachTrack(now);if(!ArmStartPose_IsActive())return;
    if(s_phase==POSE_GRIP_OPEN || s_phase==POSE_GRIP_CLOSE) {
        arm_servo_status_t grip;ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
        if(!grip.enabled || grip.fault){Pose_Fail("grip PWM unavailable");return;}
        if(now-s_tick>5000U &&
           !(s_phase==POSE_GRIP_CLOSE && Pose_UsesEarlyPickDescent() && ArmServo_GripIsClosed())) {
            Pose_Fail("grip PWM timeout");return;
        }
        if((grip.ramping && !(s_phase==POSE_GRIP_CLOSE && s_pick==3U && now-s_tick>=POSE_RING_RELEASE_WAIT_MS)) ||
           now-s_tick<((s_phase==POSE_GRIP_CLOSE && (s_pick==1U || s_pick==4U))?ARM_SERVO_GRIP_CLOSE_MIN_MS:
            (s_phase==POSE_GRIP_CLOSE && s_pick==3U)?POSE_RING_RELEASE_WAIT_MS:ARM_SERVO_GRIP_OPEN_MIN_MS))return;
        if(s_phase==POSE_GRIP_OPEN) {
            if(s_pick==4U)ZDT_Console_Print("[ARM RING PICK] open confirmed; raise to TOP-40mm before ring positioning\r\n");
            else {
                char line[160];float clearance=Pose_ClearancePercent()*s_run.span[0]/100.0f*360.0f/65536.0f;
                (void)snprintf(line,sizeof(line),"[ARM PICK] 2/4 rotation clearance=%+.4fdeg (TOP-40mm); lift only if below, then parallel DM/reach\r\n",(double)clearance);
                ZDT_Console_Print(line);
            }
            Pose_Enable(0U,now);
        } else {
            /* Ring/tray pickup may start CLOSE in the final3mm of the accepted
             * descent. Finish that same lift job before publishing success. */
            if(Pose_UsesEarlyPickDescent() && ArmCalibration_GetState()!=ARM_CAL_IDLE) {
                if(ArmCalibration_GetState()==ARM_CAL_FAILED)Pose_Fail("ZDT positioning failed during pickup close");
                else if(now-s_started>120000U)Pose_Fail("sequence timeout");
                return;
            }
            /* Placement dead zone was checked before OPEN. Do not repeat it
             * gate after release; validate fresh held axes/DM before DONE. */
            const char *failure=Pose_GripHoldFailure();
            if(failure){Pose_Fail(failure);return;}
            if((s_pick==1U || s_pick==4U) && !ArmServo_GripIsClosed()){Pose_Fail("closed PWM not confirmed");return;}
            if(s_pick==3U && !ArmServo_GripOpeningOrOpen())return;
            s_ring_release_handoff=(uint8_t)(s_pick==3U && ArmServo_GripOpeningOrOpen());
            s_pick_result_valid=1U;s_phase=POSE_READY;
            if(s_pick==1U || s_pick==4U)ScreenMetrics_Pick();
            else if(s_pick==2U || s_pick==3U)ScreenMetrics_Place();
            if(s_place_track){ArmDriver_CancelReachRetry();s_place_track=0U;}
            ZDT_Console_Print(s_pick==3U?"[ARM RESULT] ring OPEN command >=200ms; allow rise NOW, PWM continues; fresh axes/DM held\r\n":
                "[ARM RESULT] grip settled, fresh axes/DM held; COMPLETE latched (lift5mm/reach2mm/DM2deg hold band, existing guard retained)\r\n");
            if(s_pick==1U || s_pick==4U)s_pick_completed=1U;
            if(s_pick==3U) {
                char line[120];(void)snprintf(line,sizeof(line),"[ARM RING] DONE station=%u: OPEN >=200ms; rise permitted, PWM continues\r\n",s_ring_station);
                ZDT_Console_Print(line);
            } else ZDT_Console_Print(s_pick==2U?"[ARM PLACE] DONE: grip 225deg PWM applied; motors hold (no grip feedback)\r\n":s_pick==4U?"[ARM RING PICK] DONE: grip308deg settled at taught grasp height; next lift may overlap turn\r\n":"[ARM PICK] DONE: grip308deg settled at taught grasp height; next lift may overlap turn\r\n");
        }
        return;
    }
    if(now-s_started>120000U){Pose_Fail("sequence timeout");return;}
    if(!Pose_Axes(a,now)){
        if(now-s_tick>2000U)Pose_Fail((a[0].flags|a[1].flags)&ARM_CAL_FLAG_PROTECTED?"ZDT driver protected":
            (a[0].flags|a[1].flags)&ARM_CAL_FLAG_STALL?"ZDT stall flag outside managed recovery":"fresh ZDT feedback required");
        return;
    }
    if(s_phase==POSE_TEACH) {
        if(now-s_started>5000U){Pose_Fail("stationary teach timeout; keep still and retry");return;}
        if(a[0].position_sequence==s_pos_seq[0] || a[1].position_sequence==s_pos_seq[1] ||
           a[0].flags_sequence==s_flag_seq[0] || a[1].flags_sequence==s_flag_seq[1])return;
        for(i=0;i<2;i++) {
            if(fabs((double)a[i].position_counts-(double)s_last_counts[i])>32.0)s_samples=0;
            s_last_counts[i]=a[i].position_counts;s_pos_seq[i]=a[i].position_sequence;s_flag_seq[i]=a[i].flags_sequence;
        }
        if(DM_Motor_ReadPosition(ArmTask_GetMotor(),300U,&fb)!=HAL_OK || fb.enabled || ArmTask_IsEnabled() ||
           !isfinite(fb.vel_rads) || fabsf(fb.vel_rads)>0.05f || fb.err_code ||
           ArmDriver_MapRawRotation(fb.pos_rad,&angle)!=HAL_OK){Pose_Fail("need stationary DM OFF with valid software zero");return;}
        if(s_samples && fabsf(angle-s_last_dm)>0.1f)s_samples=0;
        s_last_dm=angle;
        if(!s_samples)s_tick=now;
        s_samples++;
        if(s_samples<4 || now-s_tick<300U)return;
        {
            start_profile_t r;
            memset(&r,0,sizeof(r));
            for(i=0;i<2;i++){
                arm_driver_axis_t axis=i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT;
                r.zero[i]=a[i].zero_counts;r.direction[i]=Pose_Direction(axis);
                if(ArmCalibration_GetPercent(axis,&r.percent[i])!=HAL_OK || ArmCalibration_GetSpan(axis,&r.span[i])!=HAL_OK){Pose_Fail("axis reference invalid");return;}
            }
            r.esc_id=ArmTask_GetMotor()->cfg.esc_id;r.mst_id=ArmTask_GetMotor()->cfg.mst_id;
            r.dm_deg=angle;if(ArmDriver_GetRotationZeroRaw(&r.dm_zero)!=HAL_OK){Pose_Fail("DM reference invalid");return;}
            if(!Pose_Valid(&r) || ChassisMotor_HasMotionDemand()) {
                char reason[130];(void)snprintf(reason,sizeof(reason),"invalid RAM pose: lift %.4f%% reach %.4f%% DM %.3fdeg or chassis enabled",
                    (double)r.percent[0],(double)r.percent[1],(double)r.dm_deg);Pose_Fail(reason);return;
            }
            s_saved=r;s_run=r;s_valid=1U;s_profile_source=2U;s_phase=POSE_READY;
            {
                char line[180];
                (void)snprintf(line,sizeof(line),"[ARM START] RAM CAPTURED: lift=%.4f%% (%+.3fdeg) reach=%.4f%% (%+.3fdeg) DM=%+.4fdeg; RAM only, lost after power cycle\r\n",
                    (double)r.percent[0],(double)(a[0].position_counts-a[0].zero_counts)*360.0/65536.0,
                    (double)r.percent[1],(double)(a[1].position_counts-a[1].zero_counts)*360.0/65536.0,(double)angle);ZDT_Console_Print(line);
            }
        }
        return;
    }
    if(ArmCalibration_GetState()==ARM_CAL_FAILED){Pose_Fail("ZDT positioning failed");return;}
    if(s_raise_overlap) {
        if(s_raise_grip_closed?!ArmServo_GripIsClosed():!(ArmServo_GripIsOpen() || ArmStartPose_IsReleaseOpening())){Pose_Fail("grip PWM changed during rise/turn overlap");return;}
        if(ArmCalibration_GetState()==ARM_CAL_IDLE) {
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current) ||
               current>Pose_ClearancePercent()+ArmCalibration_LiftTolerancePercent(Pose_ClearancePercent(),a[0].travel_mm)){
                Pose_Fail("overlapped clearance raise not confirmed");return;
            }
            s_raise_overlap=0U;
            ZDT_Console_Print(s_pick==2U?"[ARM PLACE] true TOP confirmed; descent may overlap final5deg, OPEN waits final DM\r\n":Pose_UsesEarlyLower()?"[ARM TRANSFER] lift clearance confirmed; descent may overlap final15deg, grip waits final DM\r\n":
                "[ARM TRANSFER] lift clearance confirmed during turn/reach; final descent still waits both held\r\n");
        } else if(!(s_pick==2U?ArmCalibration_IsLiftRaisingToTop():ArmCalibration_IsLiftRaisingToClearance())) {Pose_Fail("overlapped lift target changed");return;}
    }
    if(s_phase==POSE_DM || s_phase==POSE_REACH_ENABLE || s_phase==POSE_REACH) {
        if(!(a[0].flags&ARM_CAL_FLAG_ENABLED) || ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK ||
           !isfinite(current)){Pose_Fail("lift feedback unavailable during DM/reach");return;}
        if(s_lower_recovery) {
            if(!ArmServo_GripIsClosed() || a[0].travel_mm<=0.0f){Pose_Fail("lift/grip hold lost during lowered reach recovery");return;}
            if(s_lower_hold_pending) {
                if(now-s_lower_hold_tick>2000U){Pose_Fail("lift STOP not confirmed during reach recovery");return;}
                if(a[0].position_sequence!=s_lower_hold_sequence && (int32_t)(a[0].position_tick-s_lower_hold_tick)>0) {
                    double moved=fabs((double)a[0].position_counts-(double)s_lower_hold_last);
                    s_lower_hold_last=a[0].position_counts;s_lower_hold_sequence=a[0].position_sequence;
                    if(s_lower_hold_samples && moved>fabs((double)s_run.span[0])*0.0003)s_lower_hold_samples=0U;
                    if(++s_lower_hold_samples>=2U){s_lower_hold_percent=current;s_lower_hold_pending=0U;}
                }
            } else if(fabsf(current-s_lower_hold_percent)*a[0].travel_mm/100.0f>POSE_GRIP_HOLD_MM){Pose_Fail("lift drift while reach recovery holds");return;}
        } else if(!s_material_direct && current>(s_raise_overlap?s_raise_half_percent:Pose_ClearancePercent())+ARM_TOP_REFERENCE_TOLERANCE_MM/ARM_LIFT_FULL_TRAVEL_MM*100.0f){Pose_Fail("lift below clearance before DM/reach finished");return;}
    }
    if(s_phase==POSE_REACH_ENABLE && s_ring_return_reach) {
        if(!Pose_RingReturnReachEarly(a,now))return;
        if(s_ring_return_reach==3U)s_phase=POSE_REACH;
        return;
    }
    if(s_phase==POSE_TOP_ENABLE || s_phase==POSE_REACH_ENABLE) {
        i=s_phase==POSE_REACH_ENABLE;
        if(now-s_tick>1000U){Pose_Fail("enable confirmation timeout");return;}
        if(a[i].position_sequence==s_pos_seq[i] || a[i].flags_sequence==s_flag_seq[i] || !(a[i].flags&ARM_CAL_FLAG_ENABLED))return;
        if(i) {
            DM_GetFeedback(ArmTask_GetMotor(),&fb);
            now=HAL_GetTick(); /* Compare age against time after ISR snapshot. */
            if(!ArmTask_IsEnabled() || !fb.enabled || now-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT ||
               ArmDriver_GetRotationDeg(&angle)!=HAL_OK || (!s_turn_pending && fabsf(angle-s_run.dm_deg)>2.0f)){Pose_Fail("DM hold lost before reach");return;}
        }
        if(!i && ArmCalibration_GetState()==ARM_CAL_IDLE && ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)==HAL_OK &&
           current<=Pose_ClearancePercent()+(s_pick==2U?ArmCalibration_LiftTolerancePercent(0.0f,a[0].travel_mm):0.0f)) {
            s_phase=POSE_TOP;
            ZDT_Console_Print("[ARM PICK] lift already above rotation clearance; hold height, skip initial lift\r\n");return;
        }
        /* Loaded transfers and open-claw returns after release share the same
         * measured half-rise gate. A plain taught START retains its full raise. */
        if(!i && (((s_pick==2U || s_pick==3U) && ArmServo_GripIsClosed()) ||
                  ((s_pick==1U || s_pick==4U || s_generic) && (ArmServo_GripIsOpen() || ArmStartPose_IsReleaseOpening()))) &&
           ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)==HAL_OK && isfinite(current) && current>Pose_ClearancePercent()) {
            s_raise_overlap=1U;s_raise_grip_closed=ArmServo_GripIsClosed();
            s_raise_half_percent=(current+Pose_ClearancePercent())*0.5f;
        }
        if((i?Pose_ReachNative(&a[1],now):ArmCalibration_MovePercent(ARM_DRIVER_LIFT,Pose_ClearancePercent()))!=HAL_OK){Pose_Fail("target rejected");return;}
        s_phase=i?POSE_REACH:POSE_TOP;
        return;
    }
    if(s_phase==POSE_TOP) {
        if(ArmCalibration_GetState()!=ARM_CAL_IDLE && !s_raise_overlap)return;
        if(!(a[0].flags&ARM_CAL_FLAG_ENABLED) || ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current)){Pose_Fail("lift clearance not confirmed");return;}
        if(!Pose_RingReturnReachEarly(a,now))return;
        if(s_ring_return_reach==3U && (ArmDriver_ReachRetryRecovering() || ArmDriver_ReachStallPending()))return;
        if(s_raise_overlap && current>s_raise_half_percent)return;
        if(s_raise_opening_handoff && !ArmServo_GripIsOpen())return; /* Rise may overlap OPEN; rotation keeps its old open-grip condition. */
        s_raise_opening_handoff=0U;
        if(!s_raise_overlap && current>Pose_ClearancePercent()+ArmCalibration_LiftTolerancePercent(Pose_ClearancePercent(),a[0].travel_mm)){Pose_Fail("lift clearance not confirmed");return;}
        if(s_raise_overlap) {
            char line[176];(void)snprintf(line,sizeof(line),"[ARM TRANSFER] actual half-rise crossed: %.3f%% <= %.3f%%; final lift=%.3f%%; %s\r\n",(double)current,(double)s_raise_half_percent,(double)Pose_ClearancePercent(),s_ring_return_reach?"start DM, keep earlier reach job":"start parallel DM/reach");ZDT_Console_Print(line);
        }
        if(ArmDriver_Enable(ARM_DRIVER_ROTATION)!=HAL_OK){Pose_Fail("DM enable rejected");return;}
        if(Pose_Rotate(s_run.dm_deg)!=HAL_OK){Pose_Fail("DM target rejected");return;}
        {char line[144];(void)snprintf(line,sizeof(line),"[ARM DM] rotation speed=%.1fdeg/s; OPEN x1.44/loaded x1.20 x1.30, common motion x1.1\r\n",
            (double)Pose_DmSpeed());ZDT_Console_Print(line);}
        s_phase=POSE_DM;s_tick=now;s_samples=0;DM_GetFeedback(ArmTask_GetMotor(),&fb);s_dm_seq=fb.fb_count;
        if(s_pick || s_generic) {
            s_turn_pending=1U;s_dm_started=HAL_GetTick();
            if(s_ring_return_reach) {
                s_phase=s_ring_return_reach==3U?POSE_REACH:POSE_REACH_ENABLE;
                ZDT_Console_Print("[ARM RING RETURN] actual half-rise crossed; start DM, keep earlier reach job without duplicate command\r\n");
            } else Pose_Enable(1U,s_dm_started);
            ZDT_Console_Print(s_pick==2U?"[ARM PLACE] rise/turn overlap retained; lower within5deg after true TOP/reach gate; OPEN waits final DM\r\n":Pose_UsesEarlyLower()?"[ARM TRANSFER] rise clearance met; DM/reach concurrent; lower within15deg after existing reach gate; grip waits final DM\r\n":
                (s_pick==2U || s_pick==3U)?"[ARM TRANSFER] rise clearance met; after clearance/DM arrival lower immediately; reach stays live\r\n":"[ARM TRANSFER] rise clearance met; DM/reach concurrent; lower only after TOP and BOTH held\r\n");
        }
        return;
    }
    DM_GetFeedback(ArmTask_GetMotor(),&fb);
    now=HAL_GetTick();
    if(!ArmTask_IsEnabled() || ArmTask_IsFaulted() || !fb.enabled || !fb.fb_count || now-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT ||
       (fb.err_code!=0 && fb.err_code!=1) || !isfinite(fb.vel_rads) || ArmDriver_GetRotationDeg(&angle)!=HAL_OK){Pose_Fail("DM hold invalid");return;}
    if(s_phase==POSE_REACH && (s_pick || s_generic) &&
       ArmDriver_ReachRetryRecovering() && !s_dm_wait_reach) {
        /* Preserve normal parallel motion. During a reach stall hold DM at
         * its ACTUAL current angle until reach has completed STOP/hold. */
        if(Pose_Rotate(angle)!=HAL_OK){Pose_Fail("DM hold during reach recovery rejected");return;}
        s_dm_wait_reach=1U;s_turn_pending=1U;s_samples=0U;
        ZDT_Console_Print("[ARM RECOVERY] reach stalled: DM holds current angle, grip unchanged; retry reach FIRST\r\n");
    }
    if(s_turn_pending && !s_dm_wait_reach) {
        if(now-s_dm_started>POSE_DM_TIMEOUT_MS){Pose_Fail(s_pick==2U?"DM arrival timeout after reach hold":"parallel DM arrival timeout");return;}
        if(fb.fb_count!=s_dm_seq) {
            s_dm_seq=fb.fb_count;
            if(fabsf(angle-s_run.dm_deg)>1.0f || fabsf(fb.vel_rads)>0.05f)s_samples=0U;
            else {
                if(!s_samples)s_tick=now;
                if(ArmStartPose_IsFastTransfer() || (++s_samples>=ARM_MOTION_SETTLE_SAMPLES && now-s_tick>=ARM_MOTION_SETTLE_MS))s_turn_pending=0U;
            }
        }
    }
    if(s_phase==POSE_DM) {
        if(now-s_tick>POSE_DM_TIMEOUT_MS){Pose_Fail("DM arrival timeout");return;}
        if(fb.fb_count==s_dm_seq)return;
        s_dm_seq=fb.fb_count;
        if(fabsf(angle-s_run.dm_deg)>1.0f || fabsf(fb.vel_rads)>0.05f){s_samples=0;return;}
        if(!s_samples)s_tick=now;
        if(ArmStartPose_IsFastTransfer() || (++s_samples>=ARM_MOTION_SETTLE_SAMPLES && now-s_tick>=ARM_MOTION_SETTLE_MS))Pose_Enable(1,now);
        return;
    }
    if(!s_turn_pending && fabsf(angle-s_run.dm_deg)>2.0f){Pose_Fail("DM moved away from recorded target");return;}
    if(!(a[0].flags&ARM_CAL_FLAG_ENABLED) || (!(a[1].flags&ARM_CAL_FLAG_ENABLED) && !ArmDriver_ReachRetryRecovering())){Pose_Fail("axis lost enable");return;}
    if(s_phase==POSE_REACH && ArmDriver_ReachRetryRecovering()) {s_native=1U;s_reach_encoder=0U;return;}
    if(!Pose_ReachStartupService())return;
    if(s_phase==POSE_REACH && s_place_track) {
        if(ArmDriver_ReachStallPending())return;
        if(s_lower_recovery && s_lower_hold_pending)return;
        if(s_dm_wait_reach) {
            if(!Pose_PlaceReachAtRelease())return;
            if(Pose_Rotate(s_run.dm_deg)!=HAL_OK){Pose_Fail("resume DM after placement reach recovery rejected");return;}
            s_dm_wait_reach=0U;s_turn_pending=1U;s_samples=0U;s_dm_started=HAL_GetTick();s_dm_seq=fb.fb_count;
            ZDT_Console_Print("[ARM PLACE TRACK] reach recovered; resume DM, keep live position tracking\r\n");return;
        }
        if(ArmCalibration_GetState()!=ARM_CAL_IDLE || !Pose_FinalDescentAllowed(angle,a[0].travel_mm))return;
        Pose_BeginFinalDescent(now);return;
    }
    if(s_phase==POSE_REACH && s_reach_encoder && now-s_native_started>s_native_timeout) {
        Pose_Fail("encoder reach correction timeout");return;
    }
    if(ArmCalibration_GetState()!=ARM_CAL_IDLE &&
       !(Pose_UsesEarlyPickDescent() && s_phase==POSE_LIFT &&
         (ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE || ArmCalibration_GetState()==ARM_CAL_PERCENT_SETTLE)))return;
    if(s_phase==POSE_REACH) {
        if(s_reach_encoder) {
            double stable;
            if(a[1].position_sequence==s_native_sequence)return;
            s_native_sequence=a[1].position_sequence;
            if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Pose_Fail("encoder reach feedback invalid");return;}
            stable=fabs((double)a[1].position_counts-(double)s_native_last);s_native_last=a[1].position_counts;
            if(fabsf(current-s_run.percent[1])>Pose_ReachTolerance() || (!ArmStartPose_IsFastTransfer() && stable>fabs((double)s_run.span[1])*0.0003)) {
                s_native_samples=0U;return;
            }
            if(!s_native_samples)s_native_settle=now;
            if(!ArmStartPose_IsFastTransfer() && (++s_native_samples<ARM_MOTION_SETTLE_SAMPLES || now-s_native_settle<ARM_MOTION_SETTLE_MS))return;
            s_reach_encoder=0U;
            ZDT_Console_Print("[ARM REACH] encoder target/hold confirmed; lift may proceed\r\n");
        }
        if(s_native) {
            float error;double stable;
            if(s_native_retry_generation!=ArmDriver_ReachRetryGeneration()) {
                if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Pose_Fail("reach retry position stale");return;}
                s_native_retry_generation=ArmDriver_ReachRetryGeneration();
                s_native_started=now;s_native_initial=current;s_native_samples=s_native_stopped=0U;s_native_quiet_since=0U;
                s_native_timeout=ArmDriver_ReachRetryTimeoutMs((double)(current-s_run.percent[1])*s_run.span[1]/100.0);
                s_native_sequence=a[1].position_sequence;s_native_last=a[1].position_counts;return;
            }
            if(now-s_native_started>s_native_timeout){Pose_Fail("native reach timeout");return;}
            if(a[1].position_sequence==s_native_sequence)return;
            if((int32_t)(a[1].position_tick-s_native_started)<0)return;
            s_native_sequence=a[1].position_sequence;
            if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Pose_Fail("reach feedback invalid");return;}
            error=s_run.percent[1]-current;
            {const char *failure=Pose_NativeReachTravelFailure(current);if(failure){Pose_Fail(failure);return;}}
            if(ArmDriver_ReachStallPending())return;
            stable=fabs((double)a[1].position_counts-(double)s_native_last);s_native_last=a[1].position_counts;
            /* Confirm encoder hold AFTER STOP; the old code accepted the
             * moving endpoint then immediately failed on the next drift sample. */
            if(fabsf(error)<=Pose_ReachTolerance() && (ArmStartPose_IsFastTransfer() || stable<=fabs((double)s_run.span[1])*0.0003)) {
                if(ArmDriver_ReachStallPending())return; /* Do not publish arrival while observation is pending. */
                s_native_quiet_since=0U;
                if(!s_native_stopped) {
                    if(ArmDriver_StopReachForHold()!=HAL_OK){Pose_Fail("reach hold stop failed");return;}
                    s_native_stopped=1U;s_native_samples=0U;
                    ZDT_Console_Print(ArmStartPose_IsFastTransfer()?"[ARM REACH] target observed; STOP queued, no extra hold dwell\r\n":"[ARM REACH] STOP queued; verify fresh held position before lowering\r\n");
                    if(!ArmStartPose_IsFastTransfer())return;
                }
                if(!s_native_samples)s_native_settle=now;
                if(!ArmStartPose_IsFastTransfer() && (++s_native_samples<ARM_MOTION_SETTLE_SAMPLES || now-s_native_settle<ARM_MOTION_SETTLE_MS))return;
                s_native=0U;
                if(ArmStartPose_IsFastTransfer())ArmDriver_CancelReachRetry();
            } else {
                s_native_samples=0U;
                if(stable>fabs((double)s_run.span[1])*0.0003) {s_native_quiet_since=0U;return;}
                if(!s_native_quiet_since)s_native_quiet_since=now;
                if(now-s_native_quiet_since<500U)return;
                /* A quiet encoder interval far from the goal does not prove
                 * the native position job finished. Keep its original deadline. */
                if(fabsf(error)>2.0f)return;
                if(s_pick || s_generic) {
                    if(ArmDriver_Stop(ARM_DRIVER_REACH)!=HAL_OK ||
                       ArmCalibration_MovePercent(ARM_DRIVER_REACH,s_run.percent[1])!=HAL_OK) {
                        Pose_Fail("encoder reach correction rejected");return;
                    }
                    s_native=0U;s_reach_encoder=1U;s_native_samples=0U;
                    s_native_sequence=a[1].position_sequence;s_native_last=a[1].position_counts;
                    {
                        char line[180];
                        (void)snprintf(line,sizeof(line),"[ARM REACH] %s encoder correction: actual=%+.3f target=%+.3fdeg; lift waits for held feedback\r\n",
                            s_pick==1U?"PICK":s_pick==2U?"PLACE":"RING",
                            (double)current*s_run.span[1]/100.0*360.0/65536.0,
                            (double)s_run.percent[1]*s_run.span[1]/100.0*360.0/65536.0);
                        ZDT_Console_Print(line);
                    }
                    return;
                }
                if(s_native_retries>=2U){Pose_Fail("reach correction exhausted");return;}
                {
                    int32_t pulses;uint16_t speed=ArmDriver_ReachMotionSpeedRpm();char line[160];
                    if(speed>POSE_PLACE_CORRECTION_RPM)speed=POSE_PLACE_CORRECTION_RPM;
                    if(ArmDriver_Stop(ARM_DRIVER_REACH)!=HAL_OK ||
                       ArmCalibration_GetAbsoluteReachPulses(s_run.percent[1],&pulses)!=HAL_OK ||
                       ArmDriver_ReachAbsolutePulses(pulses,speed,ARM_CAMERA_POSITION_ACCEL)!=HAL_OK) {
                        Pose_Fail("reach slow correction rejected");return;
                    }
                    s_native_retries++;s_native_initial=current;s_native_stopped=0U;s_native_quiet_since=0U;
                    (void)snprintf(line,sizeof(line),"[ARM REACH] auto correction %u/2 error=%+.3fdeg goal_pulses=%ld speed=%uRPM\r\n",
                        s_native_retries,(double)error*s_run.span[1]/100.0*360.0/65536.0,(long)pulses,speed);
                    ZDT_Console_Print(line);return;
                }
            }
        }
        if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)!=HAL_OK){Pose_Fail("reach feedback invalid after hold");return;}
        if(fabsf(current-s_run.percent[1])>Pose_ReachTolerance()) {
            if(fabsf(current-s_run.percent[1])<=2.0f && s_native_retries<2U) {
                s_native=1U;s_native_stopped=1U;s_native_samples=0U;s_native_quiet_since=now;
                s_native_last=a[1].position_counts;s_native_sequence=a[1].position_sequence;
                ZDT_Console_Print("[ARM REACH] hold drift while waiting DM; recheck then slow correction\r\n");return;
            }
            Pose_Fail("reach not confirmed after hold");return;
        }
        if(ArmDriver_ReachStallPending())return;
        if(s_dm_wait_reach) {
            if(Pose_Rotate(s_run.dm_deg)!=HAL_OK){Pose_Fail("resume DM after reach hold rejected");return;}
            s_dm_wait_reach=0U;s_turn_pending=1U;s_samples=0U;s_dm_started=HAL_GetTick();s_dm_seq=fb.fb_count;
            ZDT_Console_Print(s_pick==2U?"[ARM PLACE] reach actual STOP/hold confirmed; NOW rotate DM to tray; lift/grip wait\r\n":"[ARM RECOVERY] reach actual STOP/hold confirmed; resume original DM target; lift/grip wait\r\n");return;
        }
        if(!Pose_FinalDescentAllowed(angle,a[0].travel_mm))return;
        Pose_BeginFinalDescent(now);
        return;
    }
    if((s_pick==2U || s_pick==3U) && s_phase==POSE_LIFT &&
       (s_turn_pending || fabsf(angle-s_run.dm_deg)>1.0f || fabsf(fb.vel_rads)>0.05f))return;
    if(Pose_UsesEarlyPickDescent() && s_phase==POSE_LIFT) {
        if(s_turn_pending || fabsf(angle-s_run.dm_deg)>1.0f || fabsf(fb.vel_rads)>0.05f)return;
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current)) {Pose_Fail("pickup lift feedback invalid");return;}
        if(fabsf(current-s_run.percent[0])*a[0].travel_mm/100.0f>POSE_PICK_CLOSE_LIFT_MM)return;
    } else if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || fabsf(current-s_run.percent[0])>ArmCalibration_LiftTolerancePercent(s_run.percent[0],a[0].travel_mm)){Pose_Fail("lift not confirmed");return;}
    if(s_pick) {
        if(s_place_track) {
            if(!Pose_PlaceReachAtRelease())return;
            /* Accepting a dead-zone residual is not a hardware STOP. End
             * the old ABS move before OPEN, otherwise the driver can keep
             * chasing it after the retry owner is cancelled at completion.
             * Fresh held-axis checks run in the existing OPEN/PWM window. */
            if(ArmDriver_StopReachForHold()!=HAL_OK){Pose_Fail("placement release reach hold stop failed");return;}
            ArmDriver_CancelReachRetry();s_place_track=0U;s_native=0U;
            ZDT_Console_Print("[ARM PLACE HOLD] within5deg; STOP reach before OPEN; keep enabled, verify held feedback during PWM\r\n");
        }
        if(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,(s_pick==1U || s_pick==4U)?ArmServo_GripCloseDeg():ArmServo_GripOpenDeg())!=HAL_OK){Pose_Fail("grip close rejected");return;}
        if(Pose_UsesEarlyPickDescent()) {
            char line[176];(void)snprintf(line,sizeof(line),"[ARM %s PICK] CLOSE lift_err=%.3fmm <=3mm, DM_err=%.3fdeg arrived; finish descent/PWM before transfer\r\n",
                s_pick==4U?"RING":"TRAY",(double)(fabsf(current-s_run.percent[0])*a[0].travel_mm/100.0f),(double)fabsf(angle-s_run.dm_deg));ZDT_Console_Print(line);
        }
        s_phase=POSE_GRIP_CLOSE;s_tick=now;
        ZDT_Console_Print(s_pick==3U?"[ARM RING] 4/4 open grip 225deg\r\n":s_pick==2U?"[ARM PLACE] 4/4 open grip 225deg\r\n":s_pick==4U?"[ARM RING PICK] close308deg at shared PLACE height + saved trim\r\n":"[ARM PICK] 4/4 close grip 308deg\r\n");return;
    }
    s_material_direct=0U;s_phase=POSE_READY;ZDT_Console_Print(s_generic?"[ARM MOVE] READY: requested three-axis pose reached; motors hold\r\n":"[ARM START] READY: recorded launch pose reached; motors hold\r\n");
}
uint8_t ArmStartPose_Command(const char *line)
{
    if(Profile_Command(line))return 1U;
    if(Disc_Command(line))return 1U;
    uint8_t save,ring_station=0U;arm_cal_axis_status_t a[2]={{0}};float zero=0,span0=0,span1=0;uint32_t now=HAL_GetTick();char reply[192];
    if(strncmp(line,"arm params ",11U)==0) {
        uint8_t ok=0U;arm_servo_status_t grip,table;
        if(!strcmp(line,"arm params status")) {
            (void)snprintf(reply,sizeof(reply),"[ARM PARAMS] sequence=%lu banks=capacity-%u/-%usectors " COMPETITION_VARIANT "; trims + servo presets; no motor zeros\r\n",(unsigned long)s_params_sequence,
                (unsigned)COMPETITION_ARM_BANK_SECTORS,(unsigned)(COMPETITION_ARM_BANK_SECTORS-1U));
            ZDT_Console_Print(reply);ArmServo_PrintStatus();return 1U;
        }
        ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);
        if(ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE || ChassisMotor_HasMotionDemand() || grip.ramping || table.ramping) {
            ZDT_Console_Print("[ARM PARAMS] REJECTED: wait for idle arm/chassis\r\n");return 1U;
        }
        if(!strcmp(line,"arm params save"))ok=Pose_ParamsSave();
        else if(!strcmp(line,"arm params load"))ok=(uint8_t)(Pose_ParamsLoad()==HAL_OK);
        else {ZDT_Console_Print("[ARM PARAMS] usage: arm params save|load|status\r\n");return 1U;}
        (void)snprintf(reply,sizeof(reply),"[ARM PARAMS] %s HAL=%u sequence=%lu\r\n",
            ok?(!strcmp(line,"arm params save")?"SAVED":"LOADED"):"FAILED",ok?0U:1U,(unsigned long)s_params_sequence);ZDT_Console_Print(reply);return 1U;
    }
    if(strcmp(line,"arm points")==0){ArmStartPose_PrintUiPoints();return 1U;}
    if(strncmp(line,"arm move ",9U)==0) {
        float lift,reach,dm;char tail;HAL_StatusTypeDef result=HAL_ERROR;
        if(sscanf(line+9U,"%f %f %f %c",&lift,&reach,&dm,&tail)==3)
            result=ArmStartPose_BeginMove(lift,reach,dm);
        (void)snprintf(reply,sizeof(reply),"[ARM MOVE] HAL=%u; TOP/turn/reach/lower sequence; START profile unchanged\r\n",(unsigned int)result);
        ZDT_Console_Print(reply);return 1U;
    }
    if(strncmp(line,"arm trim ",9U)==0) {
        char area[6],tail;unsigned ring;float dl,dr,dd,base_l,base_r,base_d,p5,p6,span5,span6;
        uint8_t ok=0U,temp=0U;arm_camera_state_t camera=ArmCalibration_GetCameraState();
        if(!ArmStartPose_IsActive() && !ChassisMotor_HasMotionDemand() && ArmCalibration_GetState()==ARM_CAL_IDLE &&
           !(camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) && camera!=ARM_CAMERA_REFERENCE &&
           sscanf(line+9U,"%5s %u %f %f %f %c",area,&ring,&dl,&dr,&dd,&tail)==5 &&
           (!strcmp(area,"rough") || !strcmp(area,"temp")) && ring>=1U && ring<=3U &&
           isfinite(dl) && isfinite(dr) && isfinite(dd)) {
            temp=(uint8_t)(!strcmp(area,"temp"));
            if(Pose_RingLiftDeg(&base_l)==HAL_OK && Pose_RingDropBaseTargets((uint8_t)ring,temp,&base_r,&base_d)==HAL_OK &&
               ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span5)==HAL_OK && ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span6)==HAL_OK) {
                ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a[0]);
                p5=(base_l+dl)/360.0f*65536.0f/span5*100.0f;p6=(base_r+dr)/360.0f*65536.0f/span6*100.0f;
                if(isfinite(p5) && isfinite(p6) && p5>=0.0f && p5<=100.0f && p6>=0.0f && p6<=100.0f &&
                   base_d+dd>=-180.0f && base_d+dd<=185.0f &&
                   ArmDriver_CheckUiPose(p5*a[0].travel_mm/100.0f,base_d+dd)==HAL_OK) {
                    s_ring_trim[ring-1U][0]=dl;s_ring_trim[ring-1U][1]=dr;s_ring_trim[ring-1U][2]=dd;ok=1U;
                }
            }
        }
        (void)snprintf(reply,sizeof(reply),"[ARM TRIM] HAL=%u; shared ring delta in RAM; no motion or flash write\r\n",ok?0U:1U);
        ZDT_Console_Print(reply);return 1U;
    }
    if(strncmp(line,"arm ring rough ",15U)==0 || strncmp(line,"arm ring temp ",14U)==0) {
        unsigned ring;char area[6],tail;HAL_StatusTypeDef result=HAL_ERROR;
        if(sscanf(line,"arm ring %5s %u %c",area,&ring,&tail)==2 && ring>=1U && ring<=3U)
            result=Pose_BeginRingDrop((uint8_t)ring,0.0f,(uint8_t)(!strcmp(area,"temp")));
        (void)snprintf(reply,sizeof(reply),"[ARM DROP] HAL=%u\r\n",(unsigned int)result);ZDT_Console_Print(reply);return 1U;
    }
    if(strcmp(line,"arm ring status")==0) {
        float reach,lift,dm;
        if(Pose_RingLiftDeg(&lift)!=HAL_OK || Pose_RingDropTargets(s_ring_station,s_ring_temp,&reach,&dm)!=HAL_OK){ZDT_Console_Print("[ARM RING] reach span/travel invalid; arm status\r\n");return 1U;}
        if(s_pick==3U && (ArmStartPose_IsActive() || s_phase==POSE_READY)) {
            lift=s_run.percent[0]*s_run.span[0]/100.0f*360.0f/65536.0f;
            reach=s_run.percent[1]*s_run.span[1]/100.0f*360.0f/65536.0f;dm=s_run.dm_deg;
        }
        (void)snprintf(reply,sizeof(reply),"[ARM RING] area=%s ring=%u pick_completed=%u grip_closed=%u active=%u phase=%u targets 5=%+.4f 6=%+.4f DM=%+.4fdeg\r\n",
            s_ring_temp?"TEMP":"ROUGH",s_ring_station,s_pick_completed,ArmServo_GripIsClosed(),(s_pick==3U && ArmStartPose_IsActive()),s_phase,(double)lift,(double)reach,(double)dm);
        ZDT_Console_Print(reply);return 1U;
    }
    if(strcmp(line,"arm ring")==0 || strcmp(line,"arm ring confirm")==0 || strcmp(line,"arm ring 2")==0)ring_station=2U;
    else if(strcmp(line,"arm ring 1")==0)ring_station=1U;
    else if(strcmp(line,"arm ring 3")==0)ring_station=3U;
    else if(strncmp(line,"arm ring ",9U)==0) {
        ZDT_Console_Print("[ARM RING] usage: arm ring 1|2|3; arm ring status\r\n");return 1U;
    }
    if(ring_station) {
        (void)ArmStartPose_BeginRingDrop(ring_station,0.0f);return 1U;
    }
    if(strcmp(line,"arm pick status")==0 || strcmp(line,"arm place status")==0) {
        float lift=-339.9005f,reach=-4.0f,dm=162.8250f;
        if((strcmp(line,"arm place status")==0?Pose_PlaceLiftDeg(&lift):Pose_PickLiftDeg(&lift))!=HAL_OK) {
            ZDT_Console_Print("[ARM PLACE] lift span/travel invalid; arm status\r\n");return 1U;
        }
        if(!strcmp(line,"arm place status")) {
            if(Pose_TrayPlaceTargets(0U,&reach,&dm)!=HAL_OK)return 1U;
        } else {reach+=s_disc_trim[ARM_DISC_TRAY_PICK][1];dm+=s_disc_trim[ARM_DISC_TRAY_PICK][2];}
        (void)snprintf(reply,sizeof(reply),"[ARM %s] active=%u phase=%u action=%u targets lift=%+.4f reach=%+.4f DM=%+.4fdeg\r\n",
            strcmp(line,"arm place status")==0?"PLACE":"PICK",ArmStartPose_IsActive(),s_phase,s_pick,(double)lift,(double)reach,(double)dm);ZDT_Console_Print(reply);return 1U;
    }
    if(strcmp(line,"arm pick")==0 || strcmp(line,"arm place")==0 || strncmp(line,"arm place ",10U)==0) {
        uint8_t place=strcmp(line,"arm pick")!=0;
        float lift=-339.9005f,reach=-4.0f,dm=162.8250f;
        unsigned slot=0U;char trailing;
        start_profile_t saved=s_saved;uint8_t valid=s_valid,source=s_profile_source;
        if(place && strncmp(line,"arm place ",10U)==0 &&
           (sscanf(line+10U,"%u %c",&slot,&trailing)!=1 || slot<1U || slot>3U)) {
            ZDT_Console_Print("[ARM PLACE] usage: arm place 1|2|3; bare arm place keeps common pose\r\n");return 1U;
        }
        if(strcmp(line,"arm place status")==0) {
            uint8_t slot;float reach,dm;
            (void)snprintf(reply,sizeof(reply),"[ARM PLACE] active=%u phase=%u action=%u last_slot=%u; height=%+.4fdeg\r\n",
                ArmStartPose_IsActive(),s_phase,s_pick,s_place_slot,(double)lift);ZDT_Console_Print(reply);
            for(slot=1U;slot<=3U;slot++) {
                if(Pose_TrayPlaceTargets(slot,&reach,&dm)!=HAL_OK){ZDT_Console_Print("[ARM PLACE] reach span/travel invalid\r\n");return 1U;}
                (void)snprintf(reply,sizeof(reply),"[ARM PLACE] slot=%u lift=%+.4f reach=%+.4f DM=%+.4fdeg\r\n",
                    slot,(double)lift,(double)reach,(double)dm);ZDT_Console_Print(reply);
            }
            return 1U;
        }
        if(ArmStartPose_IsActive() || ChassisMotor_HasMotionDemand() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
           (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL) ||
           ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE) {
            ZDT_Console_Print("[ARM PICK] refused: stop chassis/arm first\r\n");return 1U;
        }
        if(place && !ArmServo_GripIsClosed()) {
            ZDT_Console_Print("[ARM PLACE] refused: grip must be enabled and settled at closed preset; arm grip status\r\n");return 1U;
        }
        if(!place){reach+=s_disc_trim[ARM_DISC_TRAY_PICK][1];dm+=s_disc_trim[ARM_DISC_TRAY_PICK][2];}
        if((place?Pose_PlaceLiftDeg(&lift):Pose_PickLiftDeg(&lift))!=HAL_OK ||
           (place && Pose_TrayPlaceTargets((uint8_t)slot,&reach,&dm)!=HAL_OK) || Pose_Configure(lift,reach,dm,1U)!=HAL_OK) {
            ZDT_Console_Print("[ARM PICK] invalid zero/span/reference; arm query 5/6/dm\r\n");return 1U;
        }
        if(place)s_quiet_restore=1U;
        (void)ArmStartPose_Command("arm start");
        s_quiet_restore=0U;
        s_saved=saved;s_valid=valid;s_profile_source=source;
        if(s_phase!=POSE_TOP_ENABLE)return 1U;
        s_pick=place?2U:1U;s_place_slot=(uint8_t)slot;if(!place)s_pick_completed=0U;
        {
            char line[130];
            (void)snprintf(line,sizeof(line),"[ARM REACH] arrival tolerance=%.3fdeg; target observation only; no extra stable dwell\r\n",
                (double)Pose_ReachTolerance()*fabs((double)s_run.span[1])/100.0*360.0/65536.0);
            ZDT_Console_Print(line);
        }
        if(place) {
            (void)snprintf(reply,sizeof(reply),"[ARM PLACE] slot=%u targets lift=%+.4f reach=%+.4f DM=%+.4fdeg; numbered tray placement\r\n",
                s_place_slot,(double)lift,(double)reach,(double)dm);ZDT_Console_Print(reply);
            ZDT_Console_Print("[ARM PLACE] closed PWM; half-rise -> parallel DM/reach; DM/reach observed -> lower; no extra stable dwell\r\n");return 1U;
        }
        if(Pose_OpenGrip(now)!=HAL_OK)return 1U;
        ZDT_Console_Print(s_phase==POSE_TOP_ENABLE?"[ARM PICK] 1/4 accepted OPEN handoff; start rise\r\n":
            "[ARM PICK] 1/4 open grip 225deg; wait PWM ramp before arm motion\r\n");return 1U;
    }
    if(strcmp(line,"arm start status")==0) {
        (void)snprintf(reply,sizeof(reply),"[ARM START] configured=%u source=%u phase=%u lift=%.4f%% (%+.3fdeg) reach=%.4f%% (%+.3fdeg) DM=%+.4fdeg\r\n",s_valid,s_profile_source,s_phase,
            (double)s_saved.percent[0],(double)s_saved.percent[0]*s_saved.span[0]/100.0*360.0/65536.0,
            (double)s_saved.percent[1],(double)s_saved.percent[1]*s_saved.span[1]/100.0*360.0/65536.0,(double)s_saved.dm_deg);ZDT_Console_Print(reply);return 1;
    }
    if(strcmp(line,"arm start defaults")==0 || strncmp(line,"arm start set ",14U)==0) {
        float lift=ARM_START_DEFAULT_LIFT_DEG,reach=ARM_START_DEFAULT_REACH_DEG,dm=ARM_START_DEFAULT_DM_DEG;char trailing;
        if(ArmStartPose_IsActive() || ChassisMotor_HasMotionDemand() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
           (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL)) {
            ZDT_Console_Print("[ARM START] set/defaults requires idle arm and chassis stopped\r\n");return 1U;
        }
        if(strncmp(line,"arm start set ",14U)==0 && sscanf(line+14U,"%f %f %f %c",&lift,&reach,&dm,&trailing)!=3) {
            ZDT_Console_Print("[ARM START] usage: arm start set <lift_deg> <reach_deg> <dm_deg>; RAM only\r\n");return 1U;
        }
        if(Pose_Configure(lift,reach,dm,1U)!=HAL_OK)ZDT_Console_Print("[ARM START] angles/ref invalid; previous targets kept\r\n");
        else ZDT_Console_Print("[ARM START] RAM targets configured; NO W25 save; arm start status\r\n");
        return 1U;
    }
    save=strcmp(line,"arm start save")==0;
    if(!save && strcmp(line,"arm start")!=0 && strcmp(line,"arm start go")!=0)return 0;
    /* A zero command changes the frame, not the intended relative angles.
     * Rebind those angles to the newly established references automatically. */
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a[0]);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a[1]);
    if(!save && s_valid && (a[0].zero_counts!=s_saved.zero[0] || a[1].zero_counts!=s_saved.zero[1])) {
        float lift=s_saved.percent[0]*s_saved.span[0]/100.0f*360.0f/65536.0f;
        float reach=s_saved.percent[1]*s_saved.span[1]/100.0f*360.0f/65536.0f;
        if(Pose_Configure(lift,reach,s_saved.dm_deg,s_profile_source)!=HAL_OK) {
            ZDT_Console_Print("[ARM START] reference rebind failed; arm query 5/6/dm\r\n");return 1U;
        }
        ZDT_Console_Print("[ARM START] angles rebound to newly recorded zeros\r\n");
    }
    if(ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL) ||
       ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE ||
       ChassisMotor_HasMotionDemand() || !Pose_Axes(a,now) || !ArmDriver_IsRotationReferenceValid()) {
        ZDT_Console_Print("[ARM START] refused: idle arm, chassis stopped and fresh valid references required\r\n");return 1;
    }
    s_started=s_tick=now;s_samples=0;
    if(save) {
        if(ArmTask_IsEnabled()){ZDT_Console_Print("[ARM START] save requires dm off; keep all three axes stationary\r\n");return 1;}
        uint8_t i;for(i=0;i<2;i++){s_pos_seq[i]=a[i].position_sequence;s_flag_seq[i]=a[i].flags_sequence;s_last_counts[i]=a[i].position_counts;}
        s_phase=POSE_TEACH;ZDT_Console_Print("[ARM START] capturing stationary pose in RAM; waiting fresh samples\r\n");return 1;
    }
    if(!s_valid || a[0].zero_counts!=s_saved.zero[0] || a[1].zero_counts!=s_saved.zero[1] ||
       ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span0)!=HAL_OK || ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span1)!=HAL_OK ||
       span0!=s_saved.span[0] || span1!=s_saved.span[1] ||
       Pose_Direction(ARM_DRIVER_LIFT)!=s_saved.direction[0] || Pose_Direction(ARM_DRIVER_REACH)!=s_saved.direction[1] ||
       ArmTask_GetMotor()->cfg.esc_id!=s_saved.esc_id || ArmTask_GetMotor()->cfg.mst_id!=s_saved.mst_id ||
       ArmDriver_GetRotationZeroRaw(&zero)!=HAL_OK || fabsf(Pose_Wrap(zero-s_saved.dm_zero))>0.001f) {
        (void)ArmDriver_GetRotationZeroRaw(&zero);
        (void)ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&span0);(void)ArmCalibration_GetSpan(ARM_DRIVER_REACH,&span1);
        (void)snprintf(reply,sizeof(reply),"[ARM START] mismatch saved=%u DMzero now=%+.6frad saved=%+.6frad zero5 now=%lld saved=%lld zero6 now=%lld saved=%lld\r\n",
            s_valid,(double)zero,(double)s_saved.dm_zero,(long long)a[0].zero_counts,(long long)s_saved.zero[0],
            (long long)a[1].zero_counts,(long long)s_saved.zero[1]);ZDT_Console_Print(reply);
        (void)snprintf(reply,sizeof(reply),"[ARM START] span5 now=%.3f saved=%.3f span6 now=%.3f saved=%.3f dir now=%u/%u saved=%lu/%lu\r\n",
            (double)span0,(double)s_saved.span[0],(double)span1,(double)s_saved.span[1],Pose_Direction(ARM_DRIVER_LIFT),Pose_Direction(ARM_DRIVER_REACH),
            (unsigned long)s_saved.direction[0],(unsigned long)s_saved.direction[1]);ZDT_Console_Print(reply);
        ZDT_Console_Print("[ARM START] profile/ref mismatch; arm start defaults or set after references are established\r\n");return 1;
    }
    if(ArmTask_IsEnabled() && ArmDriver_Disable(ARM_DRIVER_ROTATION)!=HAL_OK){Pose_Fail("DM release rejected");return 1;}
    ArmCalibration_ClearCameraReady();
    s_material_direct=0U;s_run=s_saved;s_pick=0U;s_turn_pending=s_dm_wait_reach=0U;s_generic=0U;s_tray_gate=0U;s_raise_overlap=0U;s_pick_result_valid=0U;
    Pose_Enable(0,now);if(!s_quiet_restore)ZDT_Console_Print("[ARM START] restore: TOP lift180RPM profile -> recorded DM -> reach native60RPM -> lift\r\n");return 1;
}
