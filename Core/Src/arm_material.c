#include "arm_material.h"
#include "arm_calibration.h"
#include "arm_driver.h"
#include "arm_start_pose.h"
#include "arm_servo.h"
#include "arm_task.h"
#include "chassis_motor.h"
#include "vision.h"
#include "vision_mode_gate.h"
#include "zdt_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "screen_metrics.h"

enum { MAT_IDLE,MAT_POSITION,MAT_WAIT,MAT_GRAB,MAT_RAISE,MAT_TRAY,MAT_PLACE,MAT_CAMERA,MAT_DONE,MAT_FAILED };
static volatile uint8_t s_phase;
static uint8_t s_frames,s_confirm=1U,s_have_previous;
static volatile uint8_t s_auto; /* Read by the chassis task. */
static uint8_t s_loop,s_mode_refresh,s_wait_pose_started;
static uint8_t s_slot=1U,s_batch;
static uint8_t s_color=ARM_MATERIAL_RED;
static uint8_t s_tray_selected;
static uint8_t s_grip_started;
static uint8_t s_grab_reach_started;
static uint8_t s_grab_pose_pending;
static float s_wait_pose[3],s_grab_pose[3];
static uint32_t s_tray_selected_tick;
static uint16_t s_loop_limit; /* 0=continuous; user removes the material from slot1 each round. */
static uint32_t s_completed;
static uint16_t s_fresh=150U;
static float s_gx,s_gy=-20.0f,s_wx=120.0f,s_wy=100.0f,s_jump;
static float s_grab_pct,s_wait_pct,s_wait_deg,s_span,s_reach_wait_pct,s_previous_x,s_previous_y;
static float s_reach_grab_pct;
static float s_reach_start_pct,s_reach_half_pct;
static uint32_t s_reach_start_sequence;
static uint32_t s_grab_reach_ms,s_grab_reach_tick;
static int32_t s_reach_goal;
static uint16_t s_reach_speed;
static uint32_t s_reach_ack_fence,s_reach_ack_seen,s_reach_ack_tick,s_reach_last_send;
static uint8_t s_reach_attempts,s_reach_acked,s_reach_moved,s_reach_exhausted;
#define MAT_WINDOW_WAIT_MS (23000U)
#define MAT_REACH_RESEND_MS (10U)
#define MAT_REACH_SEND_ATTEMPTS (50U)
#define MAT_GRAB_REACH_ACCEL ARM_CAMERA_POSITION_ACCEL /* Shared209 ramp for grab and return. */
static float s_raise_start_percent;
static uint8_t s_reach_clear_pending,s_reach_clear_paced;
static uint32_t s_reach_clear_tick,s_reach_clear_count;
static uint32_t s_tick,s_wait_tick,s_last_observation,s_log_tick;
static arm_feedback_wait_t s_feedback_pause;
static vision_mode_gate_t s_gate;
static const char *s_reason="idle";
enum {COLOR_OFF,COLOR_CAMERA,COLOR_SET,COLOR_UI,COLOR_FAILED};
static volatile uint8_t s_color_phase;
static uint8_t s_learn_target=1U,s_learn_sent;
static uint32_t s_color_tick;
static vision_mode_gate_t s_color_gate;
static const char *s_color_reason="idle";
#define MAT_TRAY1_DEG (205.0f) /* User's measured tray1 angle, independent of old manual presets. */

static uint8_t Mat_ColorActive(void){return s_color_phase>=COLOR_CAMERA && s_color_phase<=COLOR_UI;}
uint8_t ArmMaterial_IsActive(void){return (s_phase>=MAT_POSITION && s_phase<=MAT_CAMERA) || Mat_ColorActive();}
uint8_t ArmMaterial_GrabReachActive(void){return s_phase==MAT_GRAB;}
uint8_t ArmMaterial_AutoEnabled(void){return s_auto;}
void ArmMaterial_Init(void)
{s_reach_clear_pending=s_reach_clear_paced=0U;s_reach_clear_count=0U;s_phase=MAT_IDLE;s_auto=0U;s_loop=s_mode_refresh=s_batch=s_tray_selected=0U;s_slot=1U;s_color=ARM_MATERIAL_RED;s_loop_limit=0U;s_completed=0U;s_frames=0U;s_confirm=1U;s_fresh=150U;s_gx=s_jump=0.0f;s_gy=-20.0f;s_wx=120.0f;s_wy=100.0f;s_reason="idle";s_color_phase=COLOR_OFF;s_color_reason="idle";s_learn_target=1U;s_learn_sent=0U;}
void ArmMaterial_AbortState(void)
{
    memset(&s_feedback_pause,0,sizeof(s_feedback_pause));
    if(ArmMaterial_IsActive()) {
        if(Mat_ColorActive()){s_color_phase=COLOR_FAILED;s_color_reason="cancelled";}
        ArmDriver_CancelReachRetry();
        s_phase=MAT_FAILED;s_reason="cancelled";s_loop=0U;
        (void)Vision_RequestStop(1U);
    }
}
static void Mat_ColorFail(const char *why)
{
    char line[160];s_color_phase=COLOR_FAILED;s_color_reason=why;
    (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();(void)Vision_RequestStop(1U);
    (void)snprintf(line,sizeof(line),"[ARM COLOR] FAILED: %s; arm camera status / camstat\r\n",why);ZDT_Console_Print(line);
}
static void Mat_ColorPoll(void)
{
    vision_camera_state_t camera={0};uint32_t now=HAL_GetTick();
    if(ChassisMotor_HasMotionDemand()){Mat_ColorFail("chassis moved during calibration");return;}
    if(s_color_phase==COLOR_CAMERA) {
        arm_camera_state_t state=ArmCalibration_GetCameraState();
        if(state==ARM_CAMERA_FAILED || state==ARM_CAMERA_IDLE || now-s_color_tick>120000U){Mat_ColorFail("CAMERA pose failed/cancelled/timeout");return;}
        if(state!=ARM_CAMERA_DONE)return;
        VisionModeGate_Start(&s_color_gate,1U,VISION_MODE_MATERIAL,1U,s_learn_target,now);
        s_color_gate.flags=VISION_MATERIAL_CALIBRATION_FLAG;s_learn_sent=0U;
        s_color_phase=COLOR_SET;s_color_reason="CAMERA READY; requesting color screen";s_color_tick=now;
        ZDT_Console_Print("[ARM COLOR] CAMERA READY; NOW request CAM1 color calibration, flags04\r\n");return;
    }
    if(!Vision_GetCameraState(1U,&camera)){Mat_ColorFail("CAM1 state unavailable");return;}
    if(s_color_phase==COLOR_SET) {
        if(!s_learn_sent) {
            HAL_StatusTypeDef result;
            if(now-s_color_tick>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS){Mat_ColorFail("CAM1 SET busy/timeout");return;}
            result=Vision_RequestMode(1U,VISION_MODE_MATERIAL,1U,s_learn_target,20U,VISION_MATERIAL_CALIBRATION_FLAG);
            if(result==HAL_BUSY)return;
            if(result!=HAL_OK || !Vision_GetCameraState(1U,&camera)){Mat_ColorFail("CAM1 color SET rejected locally");return;}
            VisionModeGate_Sent(&s_color_gate,camera.request_seq);s_learn_sent=1U;return;
        }
        {
            vision_mode_action_t action=VisionModeGate_Step(&s_color_gate,now,&camera);
            if(action==V_MODE_FAILED){Mat_ColorFail(s_color_gate.reason);return;}
            if(action!=V_MODE_READY)return;
        }
        if(!Vision_ClearPixelObservations(1U)){Mat_ColorFail("CAM1 observation reset failed");return;}
        s_color_phase=COLOR_UI;s_color_reason="screen READY; Color/Sample/Save/Exit; arm color stop to finish";
        ZDT_Console_Print("[ARM COLOR] READY: CAM1 touch Color/Sample/Save/Exit; samples persist on CAM1; arm color stop after saving\r\n");return;
    }
    if(!VisionModeGate_Matches(&s_color_gate,&camera) || camera.peer_age_ms>VISION_CAMERA_PEER_TIMEOUT_MS){Mat_ColorFail("CAM1 color session/heartbeat lost");return;}
}
static uint8_t Mat_ColorCommand(const char *line)
{
    char extra,out[220];unsigned target=1U;HAL_StatusTypeDef result;
    if(strcmp(line,"arm color status")==0) {
        (void)snprintf(out,sizeof(out),"[ARM COLOR] phase=%u active=%u target=%u %s\r\n",s_color_phase,Mat_ColorActive(),s_learn_target,s_color_reason);ZDT_Console_Print(out);return 1U;
    }
    if(strcmp(line,"arm color stop")==0) {
        if(Mat_ColorActive()) {
            s_color_phase=COLOR_OFF;s_color_reason="stopped; saved CAM1 samples kept";
            (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();(void)Vision_RequestStop(1U);
        }
        ZDT_Console_Print("[ARM COLOR] STOP requested; save on screen BEFORE stopping; CAM1 stored samples kept\r\n");return 1U;
    }
    if(strcmp(line,"arm color")!=0 && strncmp(line,"arm color ",10U)!=0)return 0U;
    if(strcmp(line,"arm color")!=0 && (sscanf(line,"arm color %u %c",&target,&extra)!=1 || target<1U || target>6U)) {
        ZDT_Console_Print("[ARM COLOR] usage: arm color [1..6] | status | stop\r\n");return 1U;
    }
    if(ArmMaterial_IsActive() || ArmStartPose_IsActive() || ChassisMotor_HasMotionDemand() || ArmCalibration_GetState()!=ARM_CAL_IDLE) {
        ZDT_Console_Print("[ARM COLOR] busy: stop chassis/arm job before color calibration\r\n");return 1U;
    }
    result=ArmCalibration_DeployCamera();
    if(result!=HAL_OK){ZDT_Console_Print("[ARM COLOR] CAMERA start rejected; arm camera status / arm status\r\n");return 1U;}
    s_phase=MAT_IDLE;s_loop=0U;s_learn_target=(uint8_t)target;s_learn_sent=0U;
    s_color_phase=COLOR_CAMERA;s_color_tick=HAL_GetTick();s_color_reason="moving CAMERA; screen waits actual READY";
    ZDT_Console_Print("[ARM COLOR] start: CAMERA pose first -> CAM1 color screen; no automatic grasp\r\n");return 1U;
}
uint8_t ArmMaterial_ReadResult(uint8_t *success)
{
    if(!success || (s_phase!=MAT_DONE && s_phase!=MAT_FAILED))return 0U;
    *success=(uint8_t)(s_phase==MAT_DONE);return 1U;
}
static void Mat_Fail(const char *why)
{
    char line[160];uint8_t phase=s_phase;
    (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();(void)Vision_RequestStop(1U);
    s_phase=MAT_FAILED;s_reason=why;s_loop=0U;
    (void)snprintf(line,sizeof(line),"[MAT] FAILED phase=%u: %s; arm status/query for axis feedback\r\n",phase,why);
    ZDT_Console_Print(line);
}
static uint8_t Mat_LiftArrived(float target)
{
    float p;arm_cal_axis_status_t a;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a);
    return ArmCalibration_GetState()==ARM_CAL_IDLE &&
        ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&p)==HAL_OK &&
        fabsf(p-target)<=ArmCalibration_LiftTolerancePercent(target,a.travel_mm);
}
static uint8_t Mat_SelectTray(void)
{
    char line[135];
    if(s_tray_selected)return 1U;
    if(ArmServo_SelectMaterialSlot(s_slot)!=HAL_OK){Mat_Fail("early tray selection rejected");return 0U;}
    s_tray_selected=1U;s_tray_selected_tick=HAL_GetTick();
    (void)snprintf(line,sizeof(line),"[MAT] arm away from tray; preselect slot=%u angle=%.0fdeg BEFORE supply grab completes\r\n",
        s_slot,(double)ArmServo_MaterialSlotDeg(s_slot));ZDT_Console_Print(line);return 1U;
}
static uint8_t Mat_PlateReachStallContext(void)
{return s_phase==MAT_WAIT || s_phase==MAT_GRAB || s_phase==MAT_RAISE;}
static uint8_t Mat_ReachStallAllowed(void)
{return Mat_PlateReachStallContext() || ArmDriver_ReachRetryRecovering() || ArmDriver_ReachStallPending();}
static uint8_t Mat_ReachEnableRecovering(uint8_t flags)
{return ArmDriver_ReachRetryRecovering() || (Mat_PlateReachStallContext() && ((flags&ARM_CAL_FLAG_STALL) || s_reach_clear_pending));}
/* Recurring plate STALL clears do not start a new native job.
 * Link/protection faults and actual hold drift are checked before this runs. */
static uint8_t Mat_ServicePlateReachStall(uint32_t now)
{
    arm_cal_axis_status_t a;HAL_StatusTypeDef result;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);
    if(!(a.flags&ARM_CAL_FLAG_STALL) && !s_reach_clear_pending){s_reach_clear_paced=0U;return 1U;}
    if(s_reach_clear_paced && now-s_reach_clear_tick<150U)return (uint8_t)((a.flags&ARM_CAL_FLAG_ENABLED)!=0U);
    s_reach_clear_paced=1U;s_reach_clear_tick=now;
    if(!s_reach_clear_pending) {
        result=ArmCalibration_ClearProtection(ARM_DRIVER_REACH);
        if(result==HAL_BUSY)return (uint8_t)((a.flags&ARM_CAL_FLAG_ENABLED)!=0U);
        if(result!=HAL_OK){Mat_Fail("reach stall clear CAN command failed");return 0U;}
        s_reach_clear_pending=1U;++s_reach_clear_count;
        if(s_reach_clear_count==1U || s_reach_clear_count%10U==0U)
            ZDT_Console_Print("[MAT REACH] id6 STALL clear/enable; keep wait position; plate flow continues\r\n");
    }
    result=ArmDriver_Enable(ARM_DRIVER_REACH);
    if(result==HAL_BUSY)return (uint8_t)((a.flags&ARM_CAL_FLAG_ENABLED)!=0U);
    if(result!=HAL_OK){Mat_Fail("reach stall re-enable CAN command failed");return 0U;}
    if(a.flags&ARM_CAL_FLAG_ENABLED)s_reach_clear_pending=0U;
    return (uint8_t)((a.flags&ARM_CAL_FLAG_ENABLED)!=0U);
}
static uint8_t Mat_HoldHealthy(uint32_t now)
{
    arm_cal_axis_status_t a;float p,angle;
    uint8_t i;
    for(i=0U;i<2U;i++) {
        ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a);
        now=HAL_GetTick();
        if(!a.position_sequence || !a.flags_sequence || now-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || now-a.flags_tick>ArmCalibration_StatusTimeoutMs(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT) ||
           (!(a.flags&ARM_CAL_FLAG_ENABLED) && !(i && Mat_ReachEnableRecovering(a.flags))) ||
           (a.flags&ARM_CAL_FLAG_PROTECTED) || (ArmCalibration_StallFault(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,a.flags) && !(i && Mat_ReachStallAllowed())))return 0U;
    }
    if(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&p)!=HAL_OK || !isfinite(p))return 0U;
    /* Same five-degree drift allowance; during the timed grasp permit the
     * commanded extension corridor after the reach command starts. */
    {
        float low=s_reach_wait_pct,high=s_reach_wait_pct;
        if((s_phase==MAT_GRAB && s_grab_reach_started) || s_phase==MAT_RAISE) {
            low=fminf(s_reach_wait_pct,s_reach_grab_pct);high=fmaxf(s_reach_wait_pct,s_reach_grab_pct);
        }
        float outside=p<low?p-low:p>high?p-high:0.0f;
        if(fabsf(outside*s_span/100.0f*360.0f/65536.0f)>5.0f)return 0U;
    }
    return ArmDriver_IsEnableRequested(ARM_DRIVER_ROTATION) && ArmDriver_GetRotationDeg(&angle)==HAL_OK &&
        fabsf(angle-((s_phase==MAT_WAIT)?s_wait_pose[2]:s_grab_pose[2]))<=2.0f;
}
static uint8_t Mat_LoopLinksHealthy(uint32_t now)
{
    arm_cal_axis_status_t a;dm_feedback_t dm;uint8_t i;
    uint8_t pose_owned=(uint8_t)((s_phase==MAT_PLACE || (s_phase==MAT_GRAB && s_grab_pose_pending)) && ArmStartPose_IsActive());
    for(i=0U;i<2U;i++) {
        ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a);
        now=HAL_GetTick();
        if((a.flags&ARM_CAL_FLAG_PROTECTED) || (ArmCalibration_StallFault(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,a.flags) && !(i && Mat_ReachStallAllowed())))return 0U;
        /* PLACE owns enable handshakes and feedback deadlines during transfer.
         * Its real FSM checks both ZDT axes, DM and final held arrival. Do not
         * impose a second, conflicting enable/freshness gate mid-handshake. */
        if(!pose_owned && (!a.position_sequence || !a.flags_sequence || now-a.position_tick>ARM_ZDT_FEEDBACK_TIMEOUT_MS || now-a.flags_tick>ArmCalibration_StatusTimeoutMs(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT) ||
           (!(a.flags&ARM_CAL_FLAG_ENABLED) && !(i && Mat_ReachEnableRecovering(a.flags)))))return 0U;
    }
    DM_GetFeedback(ArmTask_GetMotor(),&dm);
    now=HAL_GetTick(); /* The ISR snapshot may be newer than Poll's clock. */
    if(pose_owned)return !ArmTask_IsFaulted() && dm.err_code<=1U;
    return ArmDriver_IsEnableRequested(ARM_DRIVER_ROTATION) && !ArmTask_IsFaulted() && dm.fb_count && dm.enabled &&
        dm.err_code<=1U && now-dm.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT;
}
static void Mat_ReportLinks(uint32_t now)
{
    arm_cal_axis_status_t a;dm_feedback_t dm;char out[190];uint8_t i;
    for(i=0U;i<2U;i++) {
        ArmCalibration_GetAxisStatus(i?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&a);
        (void)snprintf(out,sizeof(out),"[MAT FAULT] id=%u angle=%+.3f raw=%lld flags=0x%02X pos_age=%lums flags_age=%lums pos_rx=%lu flags_rx=%lu\r\n",
            i?6U:5U,(double)(a.position_counts-a.zero_counts)*360.0/65536.0,(long long)a.position_counts,
            a.flags,(unsigned long)(now-a.position_tick),(unsigned long)(now-a.flags_tick),
            (unsigned long)a.position_sequence,(unsigned long)a.flags_sequence);ZDT_Console_Print(out);
    }
    DM_GetFeedback(ArmTask_GetMotor(),&dm);
    (void)snprintf(out,sizeof(out),"[MAT FAULT] DM request=%u enabled=%u code=%u age=%lums task_fault=%u PLACE_owner=%u\r\n",
        ArmDriver_IsEnableRequested(ARM_DRIVER_ROTATION),dm.enabled,dm.err_code,(unsigned long)(HAL_GetTick()-dm.last_tick),
        ArmTask_IsFaulted(),(s_phase==MAT_PLACE && ArmStartPose_IsActive()));ZDT_Console_Print(out);
}
static HAL_StatusTypeDef Mat_BeginWait(void)
{
    char line[175];
    if(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ArmServo_GripOpenDeg())!=HAL_OK) {
        Mat_Fail("open grip rejected before wait pose");return HAL_ERROR;
    }
    s_wait_pose_started=s_tray_selected=s_grip_started=s_grab_reach_started=s_grab_pose_pending=0U;
    s_reach_clear_pending=s_reach_clear_paced=0U;s_reach_clear_count=0U;
    s_phase=MAT_POSITION;s_reason="waiting CAM1 ACK before motion";s_tick=HAL_GetTick();s_frames=0U;s_have_previous=0U;
    s_last_observation=0U;s_mode_refresh=s_loop;
    VisionModeGate_Start(&s_gate,1U,VISION_MODE_MATERIAL,1U,s_color,s_tick);
    (void)snprintf(line,sizeof(line),"[MAT] color=%u -> tray%u: taught WAIT lift=%+.3f reach=%+.3f DM=%+.3f; taught GRAB after in-window; CAM1 ACK FIRST\r\n",
        s_color,s_slot,(double)s_wait_pose[0],(double)s_wait_pose[1],(double)s_wait_pose[2]);
    ZDT_Console_Print(line);return HAL_OK;
}
static HAL_StatusTypeDef Mat_Start(uint8_t looping,uint8_t slot,uint8_t batch,uint8_t color)
{
    arm_cal_axis_status_t lift,reach;float lift_span,base[3];
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(slot<1U || slot>3U || color<1U || color>6U)return HAL_ERROR;
    if(ArmMaterial_IsActive() || ArmStartPose_IsActive() || ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       ChassisMotor_HasMotionDemand() || (camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL) ||
       camera==ARM_CAMERA_REFERENCE)return HAL_BUSY;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
    if(!isfinite(lift.travel_mm) || lift.travel_mm<=0 || !isfinite(reach.travel_mm) || reach.travel_mm<=0 ||
       ArmCalibration_GetSpan(ARM_DRIVER_LIFT,&lift_span)!=HAL_OK ||
       ArmCalibration_GetSpan(ARM_DRIVER_REACH,&s_span)!=HAL_OK)return HAL_ERROR;
    if(ArmStartPose_DiscTargets(ARM_DISC_WAIT,base,s_wait_pose)!=HAL_OK ||
       ArmStartPose_DiscTargets(ARM_DISC_GRAB,base,s_grab_pose)!=HAL_OK)return HAL_ERROR;
    s_wait_pct=s_wait_pose[0]/360.0f*65536.0f/lift_span*100.0f;
    s_grab_pct=s_grab_pose[0]/360.0f*65536.0f/lift_span*100.0f;
    s_reach_wait_pct=s_wait_pose[1]/360.0f*65536.0f/s_span*100.0f;
    s_reach_grab_pct=s_grab_pose[1]/360.0f*65536.0f/s_span*100.0f;
    if(!isfinite(s_wait_pct) || s_wait_pct<0 || s_wait_pct>100.0f || s_grab_pct<0 || s_grab_pct>100.0f ||
       !isfinite(s_reach_wait_pct) || s_reach_wait_pct<0.0f || s_reach_wait_pct>100.0f ||
       s_reach_grab_pct<0.0f || s_reach_grab_pct>100.0f)return HAL_ERROR;
    s_wait_deg=s_wait_pose[0];
    s_loop=looping;s_slot=slot;s_batch=batch;s_color=color;s_completed=0U;
    return Mat_BeginWait();
}
HAL_StatusTypeDef ArmMaterial_Start(void){return Mat_Start(0U,1U,0U,ARM_MATERIAL_RED);}
HAL_StatusTypeDef ArmMaterial_StartToSlot(uint8_t slot){return Mat_Start(0U,slot,1U,ARM_MATERIAL_RED);}
HAL_StatusTypeDef ArmMaterial_StartToSlotColor(uint8_t slot,uint8_t color){return Mat_Start(0U,slot,1U,color);}
/* Emm FD rest-to-rest curve, including braking. Extension completion retains
 * V207 timing; the close trigger uses measured half-reach. */
static uint32_t Mat_PositionTimeMs(double counts,uint16_t rpm,uint8_t acceleration)
{
    double turns=fabs(counts)/65536.0;
    double k=acceleration?(256.0-acceleration)*0.00005:0.0;
    double seconds=turns*60.0/rpm;
    if(k>0.0)seconds=turns*60.0<=rpm*rpm*k?
        2.0*sqrt(turns*60.0*k):seconds+rpm*k;
    return (uint32_t)ceil(seconds*1000.0);
}
static HAL_StatusTypeDef Mat_BeginGrabMotion(void)
{
    arm_cal_axis_status_t reach;int32_t rp;
    uint16_t rs=(uint16_t)lroundf(ArmDriver_ReachMotionSpeedRpm()*ARM_PLATE_GRAB_REACH_SPEED_SCALE);
    HAL_StatusTypeDef result;
    if(!rs || ArmCalibration_GetPercent(ARM_DRIVER_REACH,&s_reach_start_pct)!=HAL_OK || !isfinite(s_reach_start_pct) ||
       ArmCalibration_GetAbsoluteReachPulses(s_reach_grab_pct,&rp)!=HAL_OK)return HAL_ERROR;
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
    s_reach_half_pct=(s_reach_start_pct+s_reach_grab_pct)*0.5f;
    s_reach_start_sequence=reach.position_sequence;
    s_grab_reach_ms=Mat_PositionTimeMs((double)(s_reach_grab_pct-s_reach_start_pct)*(double)s_span/100.0,rs,MAT_GRAB_REACH_ACCEL);
    s_reach_goal=rp;s_reach_speed=rs;
    s_reach_ack_fence=ArmCalibration_PositionAckFence(ARM_DRIVER_REACH);s_reach_ack_seen=s_reach_ack_fence;
    s_reach_ack_tick=HAL_GetTick();s_reach_attempts=s_reach_acked=s_reach_moved=s_reach_exhausted=0U;
    ArmDriver_CancelReachRetry();
    result=ArmDriver_ReachAbsolutePulses(rp,rs,MAT_GRAB_REACH_ACCEL);
    if(result!=HAL_OK)return result;
    ArmDriver_CancelReachRetry(); /* Retain the plate clear/continue policy. */
    s_grab_reach_tick=HAL_GetTick();s_grab_reach_started=1U;
    s_reach_last_send=s_grab_reach_tick;s_reach_attempts=1U;s_log_tick=s_grab_reach_tick;
    s_reason="grab height held; extending OPEN until measured half-reach";
    {char line[224];(void)snprintf(line,sizeof(line),"[MAT EXTEND] HOLD grab height; NO descent; id6 ABS=%ld target=%.3f%% half=%.3f%% speed=%uRPM acc=%u HAL=0 planned=%lums; OPEN until measured half\r\n",
        (long)rp,(double)s_reach_grab_pct,(double)s_reach_half_pct,rs,MAT_GRAB_REACH_ACCEL,(unsigned long)s_grab_reach_ms);ZDT_Console_Print(line);}
    return HAL_OK;
}
static uint8_t Mat_ServiceGrabReach(void)
{
    arm_cal_position_ack_t ack={0};arm_cal_axis_status_t reach;
    float actual=NAN;uint32_t now;uint8_t fresh;char line[240];HAL_StatusTypeDef result;
    ArmCalibration_GetPositionAck(ARM_DRIVER_REACH,&ack);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);now=HAL_GetTick();
    fresh=(uint8_t)(reach.position_sequence!=s_reach_start_sequence &&
        (int32_t)(reach.position_tick-s_reach_ack_tick)>=0 &&
        now-reach.position_tick<=ARM_ZDT_FEEDBACK_TIMEOUT_MS &&
        ArmCalibration_GetPercent(ARM_DRIVER_REACH,&actual)==HAL_OK && isfinite(actual));
    /* Reuse the existing unstarted-motion threshold: at most 0.5% travel,
     * or one quarter of the commanded stroke. Do not interrupt a moving axis. */
    if(fresh && fabsf(actual-s_reach_start_pct)>=fminf(ARM_PERCENT_TOLERANCE,
       fabsf(s_reach_grab_pct-s_reach_start_pct)*0.25f))s_reach_moved=1U;
    if((int32_t)(ack.sequence-s_reach_ack_fence)>0 && ack.sequence!=s_reach_ack_seen &&
       (int32_t)(ack.received_tick-s_reach_ack_tick)>=0) {
        s_reach_ack_seen=ack.sequence;
        if(ack.response==0x02U || ack.response==0x9FU)s_reach_acked=1U;
        (void)snprintf(line,sizeof(line),"[MAT REACH ACK] id6 response=0x%02X accepted=%u attempt=%u ABS=%ld; actual half still required\r\n",
            ack.response,s_reach_acked,s_reach_attempts,(long)s_reach_goal);ZDT_Console_Print(line);
    }
    if(now-s_log_tick>=2000U) {
        s_log_tick=now;
        (void)snprintf(line,sizeof(line),"[MAT REACH WAIT] actual6=%.3f%% half=%.3f%% goal=%.3f%% pos_rx=%lu age=%lums ACK=%u moved=%u sends=%u/%u flags=0x%02X; keep OPEN\r\n",
            (double)actual,(double)s_reach_half_pct,(double)s_reach_grab_pct,(unsigned long)reach.position_sequence,
            (unsigned long)(now-reach.position_tick),s_reach_acked,s_reach_moved,s_reach_attempts,MAT_REACH_SEND_ATTEMPTS,reach.flags);ZDT_Console_Print(line);
    }
    if(s_reach_moved || now-s_reach_last_send<MAT_REACH_RESEND_MS)return 1U;
    /* ACK confirms reception, not movement. Both missing ACK and accepted
     * but unstarted targets use the requested 10ms resend cadence. */
    if(s_reach_attempts>=MAT_REACH_SEND_ATTEMPTS) {
        if(!s_reach_exhausted) {
            s_reach_exhausted=1U;
            (void)snprintf(line,sizeof(line),"[MAT REACH ACK] id6 %u sends used; keep OPEN for actual half; existing120s action timeout retained\r\n",
                MAT_REACH_SEND_ATTEMPTS);ZDT_Console_Print(line);
        }
        return 1U;
    }
    s_reach_last_send=now;
    result=ArmDriver_ReachAbsolutePulses(s_reach_goal,s_reach_speed,MAT_GRAB_REACH_ACCEL);
    if(result==HAL_BUSY)return 1U;
    if(result!=HAL_OK){Mat_Fail("reach ACK resend CAN command rejected");return 0U;}
    ArmDriver_CancelReachRetry();
    ++s_reach_attempts;s_grab_reach_tick=HAL_GetTick();
    (void)snprintf(line,sizeof(line),"[MAT REACH RETRY] id6 send=%u/%u SAME ABS=%ld speed=%uRPM acc=%u ACK=%u gap=%ums; no measured start; no extra STOP\r\n",
        s_reach_attempts,MAT_REACH_SEND_ATTEMPTS,(long)s_reach_goal,s_reach_speed,MAT_GRAB_REACH_ACCEL,s_reach_acked,MAT_REACH_RESEND_MS);ZDT_Console_Print(line);
    return 1U;
}
void ArmMaterial_Poll(void)
{
    uint32_t now=HAL_GetTick();uint8_t success=0U;arm_servo_status_t grip={0},tray={0};
    if(!ArmMaterial_IsActive()){memset(&s_feedback_pause,0,sizeof(s_feedback_pause));return;}
    if(Mat_ColorActive()){Mat_ColorPoll();return;}
    if(ChassisMotor_HasMotionDemand()){Mat_Fail("chassis moved during material job");return;}
    {uint32_t gap;uint8_t waiting=ArmCalibration_WaitForFeedback(&s_feedback_pause,now,&gap);
     s_tick+=gap;s_wait_tick+=gap;
     if(s_phase==MAT_GRAB && s_grab_reach_started)s_grab_reach_tick+=gap;
     if(waiting)return;}
    if(ArmCalibration_GetState()==ARM_CAL_FAILED){Mat_Fail("axis positioning failed");return;}
    if(s_phase!=MAT_WAIT && now-s_tick>120000U){Mat_Fail("mechanical stage timeout");return;}
    if(s_loop && s_phase!=MAT_POSITION) {
        vision_camera_state_t camera={0};
        if(!Vision_GetCameraState(1U,&camera) || camera.peer_age_ms>VISION_CAMERA_PEER_TIMEOUT_MS) {
            Mat_Fail("CAM1 offline during loop");return;
        }
        if(!Mat_LoopLinksHealthy(now)){Mat_ReportLinks(now);Mat_Fail("axis disabled/faulted/stale during loop");return;}
    }
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    if(!grip.enabled || grip.fault){Mat_Fail("grip PWM unavailable");return;}
    if(s_phase==MAT_POSITION || s_phase==MAT_WAIT) {
        vision_camera_state_t camera={0};vision_mode_action_t action;
        if(!Vision_GetCameraState(1U,&camera)){Mat_Fail("CAM1 state unavailable");return;}
        /* Each loop round needs a NEW SET/ACK even if the old mode still matches. */
        if(s_mode_refresh && !camera.pending && !camera.reset_pending &&
           now-s_gate.started<VISION_CAMERA_MODE_STAGE_TIMEOUT_MS) {
            HAL_StatusTypeDef result=Vision_RequestMode(1U,VISION_MODE_MATERIAL,1U,s_color,20U,0U);
            if(result==HAL_OK) {
                if(!Vision_GetCameraState(1U,&camera)){Mat_Fail("CAM1 SET state unavailable");return;}
                VisionModeGate_Sent(&s_gate,camera.request_seq);s_mode_refresh=0U;
            } else if(result!=HAL_BUSY){Mat_Fail("CAM1 loop SET failed");return;}
            return;
        }
        action=VisionModeGate_Step(&s_gate,now,&camera);
        if(action==V_MODE_SEND) {
            HAL_StatusTypeDef result=Vision_RequestMode(1U,VISION_MODE_MATERIAL,1U,s_color,20U,0U);
            if(result==HAL_OK) {
                if(!Vision_GetCameraState(1U,&camera)){Mat_Fail("CAM1 SET state unavailable");return;}
                VisionModeGate_Sent(&s_gate,camera.request_seq);
            } else if(result!=HAL_BUSY){Mat_Fail("CAM1 SET failed");return;}
            return;
        }
        if(action==V_MODE_FAILED){Mat_Fail(s_gate.reason);return;}
        if(action!=V_MODE_READY)return;
        if(s_mode_refresh)return;
        if(!s_gate.sent){s_gate.sent=1U;s_gate.request_seq=camera.request_seq;} /* Pin an already confirmed session too. */
        if(!VisionModeGate_Matches(&s_gate,&camera) || camera.peer_age_ms>VISION_CAMERA_PEER_TIMEOUT_MS) {
            Mat_Fail("CAM1 material session lost");return;
        }
        if(s_phase==MAT_POSITION) {
            vision_pixel_t old={0};uint32_t age;
            if(!s_wait_pose_started) {
                if(ArmStartPose_BeginMove(s_wait_deg,s_wait_pose[1],s_wait_pose[2])!=HAL_OK) {
                    Mat_Fail("wait pose start rejected after CAM1 ACK; check references/fresh feedback");return;
                }
                s_wait_pose_started=1U;s_reason="positioning after CAM1 ACK";s_tick=now;
                ZDT_Console_Print("[MAT] CAM1 ACK/session confirmed; NOW move wait pose; no grab before pose/open ready\r\n");return;
            }
            {
                uint8_t ready=ArmStartPose_ReadActionResult(&success);
                if(ready && !success){Mat_Fail("wait pose failed");return;}
                if(!s_tray_selected && (ArmStartPose_IsAwayFromTray() || (ready && success))) {
                    if(!Mat_SelectTray())return;
                }
                if(!ready)return;
            }
            if(grip.ramping)return;
            (void)Vision_GetPixelWindow(1U,VISION_TYPE_MAT_PX,200U,&old,&age);
            s_last_observation=old.observation_id;
            if(!Vision_ClearPixelObservations(1U)){Mat_Fail("material observation reset failed");return;}
            char line[128];s_phase=MAT_WAIT;s_reason="waiting requested color";s_wait_tick=s_log_tick=now;s_frames=0U;s_have_previous=0U;
            (void)snprintf(line,sizeof(line),"[MAT] WAIT_COLOR id=%u: pose/open ready; NEW matching frames; wait limit=23s, then end action\r\n",s_color);ZDT_Console_Print(line);return;
        }
    }
    if(s_phase==MAT_WAIT || (s_phase==MAT_GRAB && !s_grab_pose_pending) || s_phase==MAT_RAISE) {
        if(!Mat_HoldHealthy(now)){Mat_Fail("reach/DM hold lost at material pose");return;}
        if(!Mat_ServicePlateReachStall(now))return;
    }
    if(s_phase==MAT_WAIT) {
        vision_pixel_t p={0};uint32_t age=UINT32_MAX;uint8_t fresh;
        if(now-s_wait_tick>=MAT_WINDOW_WAIT_MS){Mat_Fail("material not in window within23s");return;}
        fresh=Vision_GetPixelWindow(1U,VISION_TYPE_MAT_PX,200U,&p,&age);
        fresh=(uint8_t)(fresh && p.valid && p.id==s_color && p.type==VISION_TYPE_MAT_PX && age<=s_fresh &&
            (int32_t)(now-age-s_wait_tick)>=0 && isfinite(p.dx_px) && isfinite(p.dy_px));
        if(!fresh)s_frames=0U;
        if(fresh && p.observation_id!=s_last_observation) {
            uint8_t in_window;
            s_last_observation=p.observation_id;
            in_window=(uint8_t)(fabsf(p.dx_px-s_gx)<=s_wx && fabsf(p.dy_px-s_gy)<=s_wy);
            if(s_have_previous && s_jump>0 && hypotf(p.dx_px-s_previous_x,p.dy_px-s_previous_y)>s_jump)in_window=0U;
            s_previous_x=p.dx_px;s_previous_y=p.dy_px;s_have_previous=1U;
            s_frames=in_window?(uint8_t)(s_frames+1U):0U;
            if(s_frames>=s_confirm) {
                char line[224];
                s_phase=MAT_GRAB;s_reason="grab height held; extend immediately";s_tick=now;s_grip_started=0U;
                if(fabsf(s_grab_pose[0]-s_wait_pose[0])>0.001f || fabsf(s_grab_pose[2]-s_wait_pose[2])>0.001f) {
                    if(ArmStartPose_BeginMaterialApproach(s_grab_pose[0],s_wait_pose[1],s_grab_pose[2])!=HAL_OK) {
                        Mat_Fail("final-gripper grasp approach rejected");return;
                    }
                    s_grab_pose_pending=1U;s_reason="moving taught grasp lift/DM before extension";
                    ZDT_Console_Print("[MAT FINAL] in-window; DIRECT taught grasp lift/DM, NO clearance raise; reach stays WAIT until ready\r\n");
                } else if(Mat_BeginGrabMotion()!=HAL_OK) {Mat_Fail("extension command rejected");return;}
                (void)Vision_RequestStop(1U);
                (void)snprintf(line,sizeof(line),"[MAT] TRIGGER color=%u seq=%u obs=%lu dx=%+.1f dy=%+.1f age=%lums; taught GRAB lift/DM if different -> taught reach -> CLOSE at measured half\r\n",
                    s_color,p.seq,(unsigned long)p.observation_id,(double)p.dx_px,(double)p.dy_px,(unsigned long)age);
                ZDT_Console_Print(line);return;
            }
        }
        if(now-s_log_tick>=2000U) {
            char line[175];s_log_tick=now;
            (void)snprintf(line,sizeof(line),"[MAT] WAIT color=%u valid=%u age=%lums dx=%+.1f dy=%+.1f aim=(%.1f,%.1f) half=(%.1f,%.1f) frames=%u/%u\r\n",
                s_color,fresh,(unsigned long)age,(double)p.dx_px,(double)p.dy_px,(double)s_gx,(double)s_gy,(double)s_wx,(double)s_wy,s_frames,s_confirm);
            ZDT_Console_Print(line);
        }
        return;
    }
    if(s_phase==MAT_GRAB) {
        if(s_grab_pose_pending) {
            if(!ArmStartPose_ReadActionResult(&success))return;
            if(!success){Mat_Fail("final-gripper grasp approach failed");return;}
            s_grab_pose_pending=0U;
            if(Mat_BeginGrabMotion()!=HAL_OK){Mat_Fail("extension command rejected");return;}
            return;
        }
        if(!s_grip_started) {
            arm_cal_axis_status_t reach;float actual;
            if(!Mat_ServiceGrabReach())return;
            ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach);
            if(reach.position_sequence==s_reach_start_sequence ||
               (int32_t)(reach.position_tick-s_grab_reach_tick)<0 ||
               ArmCalibration_GetPercent(ARM_DRIVER_REACH,&actual)!=HAL_OK || !isfinite(actual))return;
            /* First new sample crossing half is enough: no stability dwell,
             * final-position wait, extra STOP or speed change. */
            if(s_reach_grab_pct>=s_reach_start_pct ? actual<s_reach_half_pct : actual>s_reach_half_pct)return;
            if(ArmServo_GripClose()!=HAL_OK){Mat_Fail("grip close rejected");return;}
            s_grip_started=1U;s_reason="half-reach crossed; remaining extension + close in parallel";
            {char line[180];(void)snprintf(line,sizeof(line),"[MAT CLOSE] actual6=%.3f%% half6=%.3f%% target6=%.3f%%; CLOSE100%% with remaining extension; V207 timing/speeds retained\r\n",
                (double)actual,(double)s_reach_half_pct,(double)s_reach_grab_pct);ZDT_Console_Print(line);}
            return;
        }
        if(grip.ramping || now-s_grab_reach_tick<s_grab_reach_ms)return;
        if(!ArmServo_GripIsClosed()){Mat_Fail("closed PWM not confirmed");return;}
        if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&s_raise_start_percent)!=HAL_OK){Mat_Fail("raise start position unavailable");return;}
        if(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)!=HAL_OK){Mat_Fail("post-grasp clearance raise rejected");return;}
        ScreenMetrics_Pick();
        s_phase=MAT_RAISE;s_tick=now;
        ZDT_Console_Print("[MAT] extend time + close PWM complete; true TOP RAISE NOW, no arrival wait/dwell; turn/reach at half-rise\r\n");return;
    }
    if(s_phase==MAT_RAISE) {
        if(!ArmServo_GripIsClosed()){Mat_Fail("closed PWM lost during lift");return;}
        if(!Mat_LiftArrived(0.0f)) {
            float current;
            if(!ArmCalibration_IsLiftRaisingToTop())return;
            if(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&current)!=HAL_OK || !isfinite(current)){Mat_Fail("TOP half-rise feedback unavailable");return;}
            if(current>s_raise_start_percent*0.5f)return;
        }
        if(!Mat_SelectTray())return;
        s_phase=MAT_TRAY;s_tick=s_tray_selected_tick;
        {
            char out[100];(void)snprintf(out,sizeof(out),"[MAT] tray slot=%u already preselected; approach in parallel, gate final descent\r\n",s_slot);ZDT_Console_Print(out);
        }return;
    }
    if(s_phase==MAT_TRAY) {
        ArmServo_GetStatus(ARM_SERVO_TABLE,&tray);
        if(!tray.enabled || tray.fault){Mat_Fail("tray PWM unavailable");return;}
        if(tray.ramping && now-s_tick>5000U){Mat_Fail("tray PWM timeout");return;}
        if(ArmCalibration_IsLiftRaisingToTop()) {
            if(ArmStartPose_BeginTrayPlaceDuringRaise(s_slot,s_raise_start_percent)!=HAL_OK){Mat_Fail("place raise adoption rejected");return;}
        } else {
            char command[24];(void)snprintf(command,sizeof(command),"arm place %u",s_slot);
            (void)ArmStartPose_Command(command);
        }
        if(!ArmStartPose_IsActive()){Mat_Fail("tray place start rejected");return;}
        if(ArmStartPose_WaitForTray(tray.target_us,s_tick)!=HAL_OK){Mat_Fail("tray descent gate rejected");return;}
        s_phase=MAT_PLACE;s_tick=now;
        {
            char out[150];(void)snprintf(out,sizeof(out),"[MAT] tray%u: fully closed -> half-rise -> parallel DM/reach; TOP and both held -> lower/open\r\n",s_slot);ZDT_Console_Print(out);
        }return;
    }
    if(s_phase==MAT_PLACE) {
        if(!ArmStartPose_ReadActionResult(&success))return;
        if(!success){Mat_Fail("tray place failed");return;}
        if(grip.ramping)return; /* PLACE's successful result already confirms release PWM. */
        if(s_completed<UINT32_MAX)s_completed++;
        if(s_batch) {
            char out[100];s_phase=MAT_DONE;s_reason="material stored in requested tray slot (no object sensor)";
            (void)snprintf(out,sizeof(out),"[MAT] SLOT_DONE slot=%u: release complete; station batch continues\r\n",s_slot);ZDT_Console_Print(out);return;
        }
        if(s_loop) {
            char line[145];
            if(s_loop_limit && s_completed>=s_loop_limit) {
                s_loop=0U;s_phase=MAT_DONE;s_reason="loop limit reached; tray1 capacity/clearance required";
                (void)Vision_RequestStop(1U);
                (void)snprintf(line,sizeof(line),"[MAT LOOP] DONE rounds=%lu limit=%u; tray1 released; ensure slot clear before more rounds\r\n",(unsigned long)s_completed,s_loop_limit);
                ZDT_Console_Print(line);return;
            }
            (void)snprintf(line,sizeof(line),"[MAT LOOP] round=%lu placed; reopen -> wait pose -> NEW CAM1 SET/ACK; clear old observations\r\n",(unsigned long)s_completed);
            ZDT_Console_Print(line);(void)Mat_BeginWait();return;
        }
        if(ArmCalibration_DeployCamera()!=HAL_OK){Mat_Fail("restore camera pose rejected");return;}
        s_phase=MAT_CAMERA;s_tick=now;ZDT_Console_Print("[MAT] tray1 released; restore camera pose for next route\r\n");return;
    }
    if(s_phase==MAT_CAMERA) {
        arm_camera_state_t state=ArmCalibration_GetCameraState();
        if(state==ARM_CAMERA_FAILED || state==ARM_CAMERA_IDLE){Mat_Fail("restore camera pose failed");return;}
        if(state==ARM_CAMERA_DONE){s_phase=MAT_DONE;s_reason="material cycle complete (no object sensor)";ZDT_Console_Print("[MAT] DONE: material grab/place sequence -> tray1; camera arm READY; no object sensor\r\n");}
    }
}
uint8_t ArmMaterial_Command(const char *line)
{
    char extra;float gx,gy,wx,wy,value;unsigned n,color,slot;
    if(Mat_ColorCommand(line))return 1U;
    if(sscanf(line,"arm material once %u %u %c",&color,&slot,&extra)==2 && color>=1U && color<=6U && slot>=1U && slot<=3U) {
        HAL_StatusTypeDef result=Mat_Start(0U,(uint8_t)slot,1U,(uint8_t)color);char out[96];
        (void)snprintf(out,sizeof(out),"[MAT FINAL] start HAL=%u color=%u slot=%u; one item to tray\r\n",(unsigned)result,color,slot);ZDT_Console_Print(out);return 1U;
    }
    if(strcmp(line,"arm material status")==0 || strcmp(line,"arm material loop status")==0) {
        char out[260];(void)snprintf(out,sizeof(out),"[MAT] phase=%u active=%u auto=%u loop=%u color=%u rounds=%lu limit=%u %s aim=(%.1f,%.1f) half=(%.1f,%.1f) fresh=%ums confirm=%u jump=%.1fpx\r\n",
            s_phase,ArmMaterial_IsActive(),s_auto,s_loop,s_color,(unsigned long)s_completed,s_loop_limit,s_reason,(double)s_gx,(double)s_gy,(double)s_wx,(double)s_wy,s_fresh,s_confirm,(double)s_jump);
        ZDT_Console_Print(out);return 1U;
    }
    if(strcmp(line,"arm material stop")==0 || strcmp(line,"arm material loop stop")==0) {
        (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();ArmMaterial_AbortState();ZDT_Console_Print("[MAT] stop requested\r\n");return 1U;
    }
    if(strcmp(line,"arm material loop start")==0) {
        HAL_StatusTypeDef result=Mat_Start(1U,1U,0U,ARM_MATERIAL_RED);char out[145];
        (void)snprintf(out,sizeof(out),"[MAT LOOP] start HAL=%u limit=%u (0=continuous); tray1 must be cleared or have capacity for every round\r\n",(unsigned)result,s_loop_limit);
        ZDT_Console_Print(out);return 1U;
    }
    if(sscanf(line,"arm material loop limit %u %c",&n,&extra)==1 && n<=1000U) {
        if(ArmMaterial_IsActive()){ZDT_Console_Print("[MAT LOOP] set limit while stopped\r\n");return 1U;}
        s_loop_limit=(uint16_t)n;
        ZDT_Console_Print("[MAT LOOP] RAM limit updated (0=continuous); ensure tray1 clearance/capacity; loop status\r\n");return 1U;
    }
    if(strcmp(line,"arm material auto on")==0 || strcmp(line,"arm material auto off")==0) {
        s_auto=(uint8_t)(strcmp(line,"arm material auto on")==0);
        ZDT_Console_Print(s_auto?"[MAT] auto ON: after station1 PLATE alignment; one red -> tray1\r\n":"[MAT] auto OFF for subsequent arrivals; arm material stop cancels current job\r\n");return 1U;
    }
    if(strcmp(line,"arm material")==0 || strcmp(line,"arm material red")==0 || strcmp(line,"arm material blue")==0) {
        HAL_StatusTypeDef result=Mat_Start(0U,1U,0U,strcmp(line,"arm material blue")==0?ARM_MATERIAL_BLUE:ARM_MATERIAL_RED);char out[90];
        (void)snprintf(out,sizeof(out),"[MAT] start HAL=%u (0=started,1=error,2=busy)\r\n",(unsigned)result);ZDT_Console_Print(out);return 1U;
    }
    if(strncmp(line,"arm material ",13U)!=0)return 0U;
    if(ArmMaterial_IsActive() && s_phase!=MAT_WAIT){ZDT_Console_Print("[MAT] config waits for idle/WAIT_RED; status or stop available\r\n");return 1U;}
    if(sscanf(line,"arm material window %f %f %f %f %c",&gx,&gy,&wx,&wy,&extra)==4 &&
       isfinite(gx) && isfinite(gy) && isfinite(wx) && isfinite(wy) && fabsf(gx)<=320 && fabsf(gy)<=240 && wx>0 && wx<=160 && wy>0 && wy<=120) {
        s_gx=gx;s_gy=gy;s_wx=wx;s_wy=wy;s_frames=0U;
    } else if(sscanf(line,"arm material fresh %u %c",&n,&extra)==1 && n>=50 && n<=200) s_fresh=(uint16_t)n;
    else if(sscanf(line,"arm material confirm %u %c",&n,&extra)==1 && (n==1 || n==2)){s_confirm=(uint8_t)n;s_frames=0U;}
    else if(sscanf(line,"arm material jump %f %c",&value,&extra)==1 && isfinite(value) && value>=0 && value<=320){s_jump=value;s_have_previous=0U;}
    else {ZDT_Console_Print("[MAT] usage: arm material [red|blue|status|stop|auto on/off|loop start/stop/status|loop limit 0..1000|window gx gy wx wy|fresh 50..200|confirm 1/2|jump px]\r\n");return 1U;}
    ZDT_Console_Print("[MAT] RAM config updated; arm material status\r\n");return 1U;
}
