/**
  ******************************************************************************
  * @file    arm_task.c
  * @brief   机械臂三轴与舵机的命令所有者和周期服务
  *
  *          职责边界：
  *            - 本文件：机构语义（使能/松劲、绝对/相对角度、安全停机、状态快照）
  *            - dm_motor.c：达妙协议（帧格式、反馈解析、超时保护、控制台命令）
  *          本文件**不直接拼 CAN 帧**，全部通过 dm_motor 的接口。
  *
  *          调试入口：UART4 控制台上的 `arm ...` / `dm ...` 命令。
  *          本任务只负责周期服务与状态维护，不占用串口。
  ******************************************************************************
  */

#include "arm_task.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "fdcan.h"
#include "zdt_motor.h"
#include "arm_driver.h"
#include "arm_calibration.h"
#include "arm_servo.h"
#include "arm_start_pose.h"
#include "arm_material.h"
#include "arm_round1.h"

/* 本车的唯一达妙电机实例。
 * 句柄式设计：将来加第二台（例如升降轴换成达妙）就在这里再加一个实例，
 * 本文件其余逻辑不用改。 */
static dm_motor_t s_turntable;
static uint8_t s_inited;
static volatile uint8_t s_stop_request;
static volatile uint32_t s_camera_request,s_camera_done;
static volatile uint8_t s_camera_result;
static volatile uint32_t s_camera_active;
static volatile uint32_t s_material_request,s_material_active,s_material_done;
static volatile uint8_t s_material_result;
static volatile uint32_t s_round_request,s_round_active,s_round_done;
static volatile uint8_t s_round_station,s_round_result,s_round_reset;
static volatile uint32_t s_start_request,s_start_active,s_start_done;
static volatile uint8_t s_start_result,s_start_ready;
static volatile uint32_t s_start_attempt_tick;
HAL_StatusTypeDef ArmTask_RequestRound1(uint32_t token,uint8_t station)
{
    uint32_t key;HAL_StatusTypeDef result=HAL_OK;
    if(!token || station<1U || station>6U)return HAL_ERROR;
    key=__get_PRIMASK();__disable_irq();
    if(s_round_request || s_round_active || s_round_reset || s_material_request || s_material_active ||
       s_start_request || s_start_active || s_camera_request || s_camera_active)result=HAL_BUSY;
    else{s_round_done=0U;s_round_station=station;__DMB();s_round_request=token;}
    __set_PRIMASK(key);return result;
}
uint8_t ArmTask_GetRound1Result(uint32_t token,uint8_t *success)
{
    uint8_t ready;uint32_t key;
    if(!token || !success)return 0U;
    key=__get_PRIMASK();__disable_irq();
    ready=(uint8_t)(s_round_done==token);if(ready)*success=s_round_result;__set_PRIMASK(key);return ready;
}
void ArmTask_RequestRound1Reset(void){s_round_reset=1U;}
HAL_StatusTypeDef ArmTask_RequestCameraDeploy(uint32_t token)
{
    uint32_t key=__get_PRIMASK();HAL_StatusTypeDef result=HAL_OK;
    if(!token)return HAL_ERROR;
    __disable_irq();if(s_camera_request || s_camera_active || s_start_request || s_start_active || s_material_request || s_material_active || s_round_request || s_round_active)result=HAL_BUSY;
    else {s_camera_done=0U;s_camera_request=token;}
    __set_PRIMASK(key);return result;
}
uint8_t ArmTask_GetCameraDeployResult(uint32_t token,uint8_t *success)
{
    uint32_t key=__get_PRIMASK();uint8_t ready;
    if(!success || !token)return 0U;
    __disable_irq();ready=s_camera_done==token;if(ready)*success=s_camera_result;__set_PRIMASK(key);return ready;
}
HAL_StatusTypeDef ArmTask_RequestMaterial(uint32_t token)
{
    uint32_t key=__get_PRIMASK();HAL_StatusTypeDef result=HAL_OK;
    if(!token)return HAL_ERROR;
    __disable_irq();
    if(s_material_request || s_material_active || s_start_request || s_start_active || s_camera_request || s_camera_active || s_round_request || s_round_active)result=HAL_BUSY;
    else{s_material_done=0U;s_material_request=token;}
    __set_PRIMASK(key);return result;
}
uint8_t ArmTask_GetMaterialResult(uint32_t token,uint8_t *success)
{
    uint32_t key=__get_PRIMASK();uint8_t ready;
    if(!token || !success)return 0U;
    __disable_irq();ready=s_material_done==token;if(ready)*success=s_material_result;__set_PRIMASK(key);return ready;
}
static osThreadId_t volatile s_owner;
HAL_StatusTypeDef ArmTask_RequestStartPose(uint32_t token)
{
    uint32_t key=__get_PRIMASK();HAL_StatusTypeDef result=HAL_OK;
    if(!token)return HAL_ERROR;
    __disable_irq();
    if(s_start_request || s_start_active || s_camera_request || s_camera_active || s_material_request || s_material_active || s_round_request || s_round_active)result=HAL_BUSY;
    else{s_start_done=0U;s_start_ready=0U;s_start_attempt_tick=HAL_GetTick();s_start_request=token;}
    __set_PRIMASK(key);return result;
}
uint8_t ArmTask_GetStartPoseResult(uint32_t token,uint8_t *success)
{
    uint32_t key=__get_PRIMASK();uint8_t ready;
    if(!token || !success)return 0U;
    __disable_irq();ready=s_start_done==token;if(ready)*success=s_start_result;__set_PRIMASK(key);return ready;
}
uint8_t ArmTask_IsStartPoseReady(void){return s_start_ready;}
HAL_StatusTypeDef ArmTask_PostCommand(const char *line)
{
    char normalized[96];uint8_t i=0,n=0;
    if(line==NULL)return HAL_ERROR;
    while(line[i]) {
        char c=line[i++];if(i>=sizeof(normalized))return HAL_ERROR;
        if(c=='\t')c=' ';
        if(c==' ' && (n==0 || normalized[n-1]==' '))continue;
        normalized[n++]=c;
    }
    if(n && normalized[n-1]==' ')n--;
    normalized[n]=0;
    if(strcmp(normalized,"arm stop")==0) { ArmTask_RequestStop();return HAL_OK; }
    return ArmCalibration_PostCommand(normalized);
}
void ArmTask_RequestStop(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    s_stop_request=1U;s_turntable.stop_generation++;s_turntable.enable_pending=0U;__set_PRIMASK(key);
}

/* 用 uint32_t 而不是 uint8_t 存"上次看到的失联计数"：
 * 计数会一直累加，用窄类型会回绕，导致第 257 次失联开始"不再打印"。 */
static uint32_t s_last_link_lost_count;
static uint32_t s_last_bus_off_count;

/* ===================== 内部工具 ===================== */

static float Arm_Abs(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float Arm_Clamp(float v, float limit)
{
    if (v > limit)  { return limit; }
    if (v < -limit) { return -limit; }
    return v;
}

/* 位置模式下电机的物理量程是 ±DM_P_MAX，越界值电机侧会被映射成怪值，
 * 所以在命令入口就夹紧 + 明确拒绝，而不是悄悄发出去。 */
static uint8_t Arm_PosInRange(float pos_rad)
{
    return (Arm_Abs(pos_rad) <= ARM_TURNTABLE_POS_LIMIT_RAD) ? 1U : 0U;
}

static uint8_t Arm_VelInRange(float vel_rads)
{
    return ((vel_rads > 0.0f) && (vel_rads <= DM_V_MAX)) ? 1U : 0U;
}

/* ===================== 初始化 ===================== */

/* Preserve ZDT queries while a bounded DM handshake yields CPU. Never recurse
 * into arm action FSMs or process another queued command from this hook. */
void DM_WaitService(dm_motor_t *motor)
{if(motor==&s_turntable && osKernelGetState()==osKernelRunning && osThreadGetId()==s_owner)ArmCalibration_ServiceWait();}

HAL_StatusTypeDef ArmTask_Init(void)
{
    dm_config_t cfg;
    HAL_StatusTypeDef status;

    (void)memset(&s_turntable, 0, sizeof(s_turntable));
    s_last_link_lost_count = 0U;
    s_last_bus_off_count = 0U;

    /* 本车电机参数：2026-09-14 由 DMTool 导出的 `达妙电机参数.txt` 确认 ——
     * MST_ID=17(0x11)、ESC_ID=1(0x01)、cmode=2(位置-速度)、Gr=10、POLE=14、
     * TIMEOUT=0（超时保护关闭）；2026-10-06实机dm diag更正PMAX=18.5，VMAX=30、TMAX=10。
     * ⚠ 若用 DMTool 改过 ID/模式，这里必须同步，否则发了没反应。 */
    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.can = &hfdcan1;
    cfg.esc_id = DM_ESC_ID_DEFAULT;          /* 0x01 -> 控制帧 0x101 */
    cfg.mst_id = DM_MST_ID_DEFAULT;          /* 0x11 -> 反馈帧 */
    cfg.mode = DM_POS_MODE;                  /* 位置-速度模式（电机寄存器 10 = 2） */
    cfg.link_timeout_ms = DM_LINK_TIMEOUT_MS_DEFAULT;   /* 800ms 无反馈 -> 失联保护 */
    /* 电机参数里 can_br = 0xFFFFFFFF（未写入），实际波特率未知，这里只是起点；
     * 用 `dm br <code>` 可以现场扫，见 dm_motor.h 的 DM_BITRATE_*。 */
    cfg.bitrate_code = DM_BITRATE_DEFAULT;
    cfg.pos_limit_rad = ARM_TURNTABLE_POS_LIMIT_RAD;
    cfg.vel_limit_rad = DM_VEL_LIMIT_RAD;

    status = DM_Motor_Init(&s_turntable, &cfg);
    if (status != HAL_OK)
    {
        /* 总线没起来 = 后面所有控制都是空发，必须让人看见 */
        ZDT_Console_Print("[ARM] ERROR: FDCAN1 init failed; turntable joint is dead.\r\n"
                          "      (check PA11/PA12 wiring, 120R termination, motor 24V)\r\n");
        s_inited = 0U;
        return status;
    }

    s_inited = 1U;
    ZDT_Console_Print("[ARM] turntable joint ready: FDCAN1 1 Mbps classic, "
                      "esc=0x01 mst=0x11, POS mode. Not enabled yet.\r\n");
    ZDT_Console_Print("[ARM] type 'dm help'; start with 'dm probe' (read-only).\r\n");

    /* 注意：这里**不使能电机**。上电后转盘处于松劲状态，必须显式 `dm enable`
     * 或 ArmTask_Enable() 才保持力矩 —— 与安全约定一致。 */
    return HAL_OK;
}

/* ===================== 命令 ===================== */

uint8_t ArmTask_IsReady(void)
{
    return s_inited;
}

uint8_t ArmTask_IsEnabled(void)
{
    return (s_inited != 0U) ? DM_IsEnabled(&s_turntable) : 0U;
}

uint8_t ArmTask_IsFaulted(void)
{
    return (s_inited != 0U) ? DM_IsLinkLost(&s_turntable) : 1U;
}

dm_motor_t *ArmTask_GetMotor(void)
{
    return &s_turntable;
}

HAL_StatusTypeDef ArmTask_Enable(void)
{
    if (s_inited == 0U)
    {
        return HAL_ERROR;
    }
    return DM_Motor_Enable(&s_turntable);
}

HAL_StatusTypeDef ArmTask_Disable(void)
{
    if (s_inited == 0U)
    {
        return HAL_ERROR;
    }
    return DM_Motor_Disable(&s_turntable);
}

HAL_StatusTypeDef ArmTask_ClearFault(void)
{
    if (s_inited == 0U)
    {
        return HAL_ERROR;
    }
    /* 清故障后并不自动使能：让操作者显式再发一次 enable，
     * 避免"故障自恢复"把转盘带到一个意外位置。 */
    return DM_Motor_ClearError(&s_turntable);
}

HAL_StatusTypeDef ArmTask_MoveToRad(float pos_rad, float vel_rads)
{
    if(ArmDriver_CheckRawRotationTarget(pos_rad+s_turntable.pos_offset_rad)!=HAL_OK)return HAL_ERROR;
    if (s_inited == 0U)
    {
        return HAL_ERROR;
    }
    if (Arm_PosInRange(pos_rad) == 0U)
    {
        char line[112];

        (void)snprintf(line, sizeof(line),
                       "[ARM] refused: target %.3f rad is outside +/-%.2f rad\r\n",
                       (double)pos_rad, (double)ARM_TURNTABLE_POS_LIMIT_RAD);
        ZDT_Console_Print(line);
        return HAL_ERROR;
    }
    if (Arm_VelInRange(vel_rads) == 0U)
    {
        char line[112];

        (void)snprintf(line, sizeof(line),
                       "[ARM] refused: velocity must be in (0, %.1f] rad/s\r\n",
                       (double)DM_V_MAX);
        ZDT_Console_Print(line);
        return HAL_ERROR;
    }
    if (DM_IsEnabled(&s_turntable) == 0U)
    {
        ZDT_Console_Print("[ARM] refused: joint not enabled (run ArmTask_Enable "
                          "or 'dm enable' first)\r\n");
        return HAL_ERROR;
    }
    return DM_Motor_SetPos(&s_turntable, pos_rad, vel_rads);
}

HAL_StatusTypeDef ArmTask_TurnRelativeRad(float delta_rad, float vel_rads)
{
    float target;

    if (s_inited == 0U)
    {
        return HAL_ERROR;
    }
    if (DM_IsEnabled(&s_turntable) == 0U)
    {
        ZDT_Console_Print("[ARM] refused: joint not enabled (run ArmTask_Enable "
                          "or 'dm enable' first)\r\n");
        return HAL_ERROR;
    }

    /* 相对运动以**当前反馈位置**为基准。若还没有任何反馈帧，说明链路没通，
     * 这时拿 0 当基准会把转盘甩到一个未知位置，所以直接拒绝。 */
    if (DM_IsOnline(&s_turntable, DM_LINK_TIMEOUT_MS_DEFAULT) == 0U)
    {
        ZDT_Console_Print("[ARM] refused: no fresh feedback, cannot compute a "
                          "relative move safely (run dm probe)\r\n");
        return HAL_ERROR;
    }

    target = Arm_Clamp(DM_GetPosRad(&s_turntable) + delta_rad,
                       ARM_TURNTABLE_POS_LIMIT_RAD);
    return ArmTask_MoveToRad(target, vel_rads);
}

void ArmTask_EmergencyStop(void)
{
    if(osKernelGetState()==osKernelRunning && osThreadGetId()!=s_owner) {
        ArmTask_RequestStop();return;
    }
    if (s_inited == 0U)
    {
        return;
    }
    /* dm_motor 会先清目标、再发失能帧、再打印一条 EMERGENCY STOP。 */
    DM_Motor_EmergencyStop(&s_turntable);
}

/* ===================== 状态 ===================== */

void ArmTask_GetStatus(arm_status_t *out)
{
    if (out == NULL)
    {
        return;
    }
    (void)memset(out, 0, sizeof(*out));
    if (s_inited == 0U)
    {
        out->fault = 1U;
        return;
    }

    out->pos_rad = DM_GetPosRad(&s_turntable);
    out->target_rad = s_turntable.target_pos_rad;
    out->vel_rads = DM_GetVelRads(&s_turntable);
    out->tor_nm = DM_GetTorqueNm(&s_turntable);
    out->t_mos_c = DM_GetMosTempC(&s_turntable);
    out->t_coil_c = DM_GetCoilTempC(&s_turntable);
    out->temp_valid = s_turntable.fb.temp_valid;
    out->enabled = DM_IsMotorReportingEnabled(&s_turntable);
    out->fault = DM_IsLinkLost(&s_turntable);
    out->online = DM_IsOnline(&s_turntable, DM_LINK_TIMEOUT_MS_DEFAULT);
    out->state = (uint8_t)DM_GetState(&s_turntable);
    out->fb_count = DM_GetFbCount(&s_turntable);
    out->link_lost_count = DM_GetLinkLostCount(&s_turntable);
}

/* ===================== 任务体 ===================== */

void ArmTask_Run(void *argument)
{
    (void)argument;
    {
        arm_driver_config_t cfg;arm_cal_config_t cal;arm_servo_config_t servo;char line[128];
        s_owner=osThreadGetId();
        ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180.0f;cfg.rotation_max_deg=185.0f;
        cfg.reach.reverse=ARM_REACH_DEFAULT_REVERSE;
        ArmCalibration_DefaultConfig(&cal);ArmServo_DefaultConfig(&servo);
        cal.reach_reverse=cfg.reach.reverse;
        if(ArmDriver_Init(&cfg)!=HAL_OK || ArmCalibration_Init(&cal)!=HAL_OK || ArmServo_Init(&servo)!=HAL_OK ||
           ArmCalibration_UseTopGuard(ARM_TOP_DROP_LIMIT_MM,
               ARM_LOW_ROTATION_MIN_DEG,ARM_LOW_ROTATION_MAX_DEG)!=HAL_OK) {
            ZDT_Console_Print("[ARM] three-axis init failed\r\n");
            for(;;)osDelay(10U);
        }
        (void)snprintf(line,sizeof(line),"[ARM] ZDT5 lift/ZDT6 reach ready; top guard: down >%.0fmm -> [%+.0f,%+.0f]deg\r\n",
            (double)ARM_TOP_DROP_LIMIT_MM,(double)ARM_LOW_ROTATION_MIN_DEG,(double)ARM_LOW_ROTATION_MAX_DEG);
        ZDT_Console_Print(line);
        ZDT_Console_Print("[ARM] defaults: ZDT5 TOP=0deg/0counts; ZDT6 RETRACTED=0deg/0counts; wait fresh feedback.\r\n");
        ZDT_Console_Print("[DM] INTERNAL ZERO: home uses raw 0rad; START/CAMERA use their configured angles; dm zero save to calibrate\r\n");
        ArmStartPose_Init();ArmMaterial_Init();ArmRound1_Init();
        {
            HAL_StatusTypeDef load;uint8_t waiting=0U;
            do {
                load=ArmStartPose_LoadParameters();
                if(load==HAL_BUSY) {
                    if(!waiting){ZDT_Console_Print("[ARM PARAMS] BOOT waiting for active Flash transaction\r\n");waiting=1U;}
                    osDelay(10U); /* Let the other storage owner finish. No CAN-ready gate. */
                }
            } while(load==HAL_BUSY);
        }
        if(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ArmServo_GripOpenDeg())!=HAL_OK)
            ZDT_Console_Print("[ARM] grip default OPEN failed; arm grip status\r\n");
        else {
            (void)snprintf(line,sizeof(line),"[ARM] boot OPEN=%.3fdeg from active parameters; CLOSE=%.3fdeg\r\n",(double)ArmServo_GripOpenDeg(),(double)ArmServo_GripCloseDeg());ZDT_Console_Print(line);
        }
        if(ArmServo_SelectMaterialSlot(ARM_SERVO_TABLE_BOOT_SLOT)!=HAL_OK)
            ZDT_Console_Print("[ARM] tray default SLOT2 failed; arm table status\r\n");
        else {
            (void)snprintf(line,sizeof(line),"[ARM] boot tray SLOT2=%.3fdeg from active parameters\r\n",(double)ArmServo_MaterialSlotDeg(ARM_SERVO_TABLE_BOOT_SLOT));ZDT_Console_Print(line);
        }
        ZDT_Console_Print("[ARM] QUERY_PACED_V1: single read request, 10ms gap, 30ms reply, one retry; arm status for TX/RX.\r\n");
        ZDT_Console_Print("[ARM] REPLACEMENT_REACH_DIR_V2: ZDT6 dir=1 (old motor dir=0); signed feedback unchanged; test pct 6 2 first.\r\n");
    ZDT_Console_Print("[ARM] CAMERA_DEPLOY_V245_" COMPETITION_VARIANT ": PREPARE_UNBOUND_LAUNCH_BIND_SHORT1_LONG2; COMPETITION_PROFILE_BANKS_ISOLATED; ODO_HEADING_LIMIT5DEG; RING_PLACE_ORDER_BY_VARIANT; MAT_DIRECT_APPROACH_NO_CLEARANCE; TRAY_PICK_TRIM; FINAL_FOUR_POSE_W25_AUTO; MANUAL_PLATE_ALIGN; TURN_FAST_RPM_EQ_PRECISE180; PID_BOOT_W25_AUTOLOAD; TURN_MAX_RPM180_X2; TURN_PID_UI_WAVE40HZ; REACH_SPEED_X1.5_ACCEL209; RING_RETURN_LIFT_REACH_TOGETHER_DM_HALF_UNCHANGED; CAMERA_NEAR_HOLD_DRIFT_CORRECT; TRAY_PLACE_REACH_ACK_START_REPLAY10MS_MAX50; TRAY_RETURN_DM_UNIFIED; CARRY_AXES_SPEED_X1.3_EMPTY_UNCHANGED; ALL_REACH_SPEED_ACCEL_X1.2; MAT_TAUGHT_WAIT_GRAB_HALF_CLOSE; PC_MAT_CONFIRM2; PLATE_SEARCH_RAW1_LEFT_RAW2_RIGHT_NOMINAL; PD1_EXTI1_BOTH_EDGES_TASK_DEBOUNCE_LOG; ARM_MOTION_X1.1; DM_HOLD0.5RAD_S; TRAY_PLACE_DM5_DESCEND_OPEN_DM_FINAL; V207_BASE; TRAY_RING_PICK_DM15_DESCEND_LIFT3_CLOSE_DM_FINAL; RING_OPEN200MS_RISE_PWM_CONTINUES; TEMP2_STATIONARY2S_PLACE; PLATE_TOL21PX; MAT_WINDOW_CENTER160_100; MAT_WINDOW240X200; PLATE_GRAB_REACH_SPEED1.44_ACCEL209; PLATE_POST_ALIGN_APPROACH_OFF; EMPTY_RETURN_HALF_RISE; ALL_LIFT_ABS_ACK100_RETRY3; ALIGN_IDLE_SNAPSHOT5S; LOW_WAIT_DIRECT_EXTEND25_HALF_CLOSE; MAT_WAIT_GRAB_INDEPENDENT_POSES; MAT_WINDOW23S; MAT_REACH_RESEND10MS_SAME_ABS50; LOADED_AXES_X1.2; OUTER_ENTRY72RPM_TRIAL; REACH_X1.1; SERVO_X1.3; CAMERA_AND_TRAY_TRUE_TOP; GROUND_RING_MINUS40_KEEP_HALF; MAT_CLOSE_DIAG; ALIGN_TIMEOUT_DIAG; RING_PICK_EQUALS_PLACE; XY_YAW_SIMULTANEOUS; MATCH_STAGE_TIMES; MAT_LOW_WAIT_EXTEND25_HALF_CLOSE; LIFT_ACCEL234; BOOT_W25_FIRST; SHARED_ROUGH_TEMP; DEFAULT_TRAY19_109_199; XY16_2_STACK10; ARM_UI_V1; MANUAL_RING_TRIM; MANUAL_ALIGN_XY_YAW; LIFT_NATIVE_FD_ABS3200; VOBS_AUTO_WIFI86; QR_DEPARTURE_LANE_SKIP5mm; VOBS_REPLAN_CURRENT_SPEED; PHOTO3_COMMON_ROUGH_BASE; MATCH_TOTAL_MS;  LIFT_GRIP_HOLD5mm; TRANSFER_NO_EXTRA_SETTLE; RING_RELEASE_ONE_POSITION_CHECK; YAW_ROUGH_TEMP_XY_PLUS_YAW; REACH_EMPTY182_LOADED246; DM_LOADED231.66_ALL_RING_RETURNS; LIFT_FD_ACCEL234; SHARED_PHOTO_DELTA reach+3,+5,+2mm DM-1.6,-1.2,-0.3deg; PLACE_REACH_TOL5deg; PD1_START_LOW_RELEASE_ARMED_READY_ONLY; LIFT_MEASURED_RANGE_STOP_OFF; GRIP_PRESET_FROM_BOOT_PARAMS; ZDT5/6_STALE_QUERY_WAIT_NO_FAULT; ODO_FINAL_TOL40mm; CHASSIS_SPD0.650 TRAY_SLEW1072.5_GRIP2145us/s; EMPTY_DM213.84deg/s LOADED_DM231.66deg/s; CHASSIS_STRAIGHT_TARGETx1.030 outer_tangentx1.030; RING_DROP SHARED reach_mm3,12,10 DMoffset+0.4,+2.0,+3 TEMP2rise72mm; UART4 DMA_TX + arm IO pump; MAT taught wait -> in-window/taught grasp lift+DM/reach -> measured half/CLOSE100% -> reach time/PWM done RAISE; 28_TRIAL ID6 STALL_IGNORE_ALL clear150ms/SAME_ABS; lift5 STALL retained; plate6 STALL clear/continue; TJC USART3 9600; tray boot SLOT2_FROM_PARAMS; fixed QR2125,1150; ALIGN clockAFTERobserver IMU150ms; CAMERA handoff STALL3/300ms; lift5 STATUS2000ms; positions/reach6 STATUS/DM800ms; CAMERA idleZDT query/no immediate staleFAULT; preMove freshWAIT_NO_TIMEOUT; ROUGH anchor1190,380; START_CAMERA settle50ms/2samples; TRANSFER no dwell; PLATE tolerance21px/3frames searchIMAGE_LEFT0.03; tray PICK lower3mm; lift loaded410/empty431RPM base238 empty_scale1.656 BOTH_UP_DOWN TOP no fixed1RPM cap arrival8mm ref20mm; RAW post-align approachOFF; ALL_ALIGN V155 XY/SEARCH/CONFIRM; RAW1_LEFT_RAW2_RIGHT SEARCH_IMAGE0.03; TEMP2 flags3 XY_PLUS_YAW;; outer arc ratio0.72 cap306.8352 lat663.552 yaw120 radius165; TRACKING_UNCHANGED_TRIAL; TURN_PID_UI_WAVE40HZ_STATUS_QUERY.\r\n");
    }

    /* FreeRTOS 调度器已经起来了，这里才做周期服务（内部会打印，走任务上下文）。 */
    for (;;)
    {
        if(s_stop_request) {
            s_stop_request=0U;ArmCalibration_DiscardCommands();
            if(s_camera_request || s_camera_active){s_camera_result=0U;s_camera_done=s_camera_active?s_camera_active:s_camera_request;}
            if(s_start_request || s_start_active){s_start_result=0U;s_start_done=s_start_active?s_start_active:s_start_request;}
            if(s_material_request || s_material_active){s_material_result=0U;s_material_done=s_material_active?s_material_active:s_material_request;}
            if(s_round_request || s_round_active){s_round_result=0U;s_round_done=s_round_active?s_round_active:s_round_request;}
            s_round_request=s_round_active=0U;
            s_material_request=s_material_active=0U;
            s_camera_request=0U;s_camera_active=0U;s_start_request=0U;s_start_active=0U;s_start_ready=0U;
            (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();
        }
        if(s_round_reset){s_round_reset=0U;ArmRound1_Reset();}
        ArmCalibration_Poll();
        ArmStartPose_Poll();
        if(s_material_request) {
            uint32_t token=s_material_request;s_material_active=token;s_material_request=0U;
            if(ArmMaterial_Start()!=HAL_OK){s_material_result=0U;__DMB();s_material_done=token;s_material_active=0U;}
        }
        ArmMaterial_Poll();
        if(s_round_request) {
            uint32_t token=s_round_request;s_round_active=token;s_round_request=0U;
            if(ArmRound1_Start(s_round_station)!=HAL_OK){s_round_result=0U;__DMB();s_round_done=token;s_round_active=0U;}
        }
        ArmRound1_Poll();
        if(s_round_active) {
            uint8_t success=0U;
            if(ArmRound1_ReadResult(&success)) {
                uint32_t token=s_round_active;
                if(success) {
                    /* Station ownership ends only after lift TOP. Adopt its
                     * already running CAMERA; do not start/restart a command. */
                    s_camera_done=0U;
                    if(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE) {
                        s_camera_result=1U;__DMB();s_camera_done=token;
                    } else s_camera_active=token;
                }
                s_round_result=success;__DMB();s_round_done=token;s_round_active=0U;
            }
        }
        if(s_material_active) {
            uint8_t success=0U;
            if(ArmMaterial_ReadResult(&success)) {
                s_material_result=success;__DMB();s_material_done=s_material_active;s_material_active=0U;
            }
        }
        if(s_start_request && (int32_t)(HAL_GetTick()-s_start_attempt_tick)>=0) {
            uint32_t token=s_start_request;HAL_StatusTypeDef result=ArmStartPose_BeginRestore();
            if(result==HAL_BUSY)s_start_attempt_tick=HAL_GetTick()+100U;
            else {
                s_start_active=token;s_start_request=0U;
                if(result!=HAL_OK){s_start_result=0U;__DMB();s_start_done=token;s_start_active=0U;}
            }
        }
        s_start_ready=ArmStartPose_IsReady();
        if(s_start_active) {
            uint8_t success=0U;
            if(ArmStartPose_ReadResult(&success)) {
                s_start_result=success;__DMB();s_start_done=s_start_active;s_start_active=0U;
            }
        }
        if(s_camera_request) {
            uint32_t token=s_camera_request;s_camera_active=token;s_camera_request=0U;
            if(ArmStartPose_IsActive() || ArmCalibration_DeployCamera()!=HAL_OK) {
                s_camera_result=0U;__DMB();s_camera_done=token;s_camera_active=0U;
            }
        }
        if(s_camera_active && (ArmCalibration_GetCameraState()==ARM_CAMERA_DONE ||
           ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED || ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE)) {
            s_camera_result=(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE)?1U:0U;
            __DMB();s_camera_done=s_camera_active;s_camera_active=0U;
        }
        ArmServo_Update(HAL_GetTick());
        if(!ARM_ZDT_WAIT_ON_STALE || ArmCalibration_ZdtFeedbackFresh())(void)ArmDriver_CheckHeightInterlock();
        if (s_inited != 0U)
        {
            DM_Motor_Service(&s_turntable, HAL_GetTick());

            /* 失联只在**次数增加的那一刻**打印一次。
             * dm_motor 内部已经打了一条，这里补一条带计数的，方便现场判断
             * "是一次偶发还是反复断"。 */
            if (s_turntable.link_lost_count != s_last_link_lost_count)
            {
                char line[128];

                s_last_link_lost_count = s_turntable.link_lost_count;
                (void)snprintf(line, sizeof(line),
                               "[ARM] turntable link lost #%lu; disable requested, "
                               "release not confirmed without feedback. Run 'dm probe'.\r\n",
                               (unsigned long)s_turntable.link_lost_count);
                ZDT_Console_Print(line);
            }

            /* CAN 控制器层错误（bus-off / error passive / 协议错）同理只在变化时打印。
             * 这条以前完全没有：FDCAN1 总线上出问题固件是瞎的。 */
            if (s_turntable.bus_off_count != s_last_bus_off_count)
            {
                char line[128];

                s_last_bus_off_count = s_turntable.bus_off_count;
                (void)snprintf(line, sizeof(line),
                               "[ARM] FDCAN1 BUS-OFF #%lu (flags=0x%08lX): check "
                               "CANH/CANL, 120R, motor 24V.\r\n",
                               (unsigned long)s_turntable.bus_off_count,
                               (unsigned long)s_turntable.can_error_flags);
                ZDT_Console_Print(line);
            }
        }
        osDelay(ARM_TASK_PERIOD_MS);
    }
}
