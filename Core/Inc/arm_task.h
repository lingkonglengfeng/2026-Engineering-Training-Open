/**
  ******************************************************************************
  * @file    arm_task.h
  * @brief   机械臂任务（ArmTask）：管理达妙转盘（FDCAN1）、ZDT5升降、
  *          ZDT6伸缩（FDCAN2）及抓取/载台舵机（TIM4）。
  *
  *          为什么单开一个任务（而不是挂在 zdtMotorTask 里）：
  *          - 达妙的命令/反馈与底盘 4 路的 CAN 是两条独立总线（FDCAN1 / FDCAN2），
  *            混在一个 1 ms 循环里，底盘 4 ms 的节拍会被达妙的日志/超时处理拖到；
  *          - 《FreeRTOS任务安排与设计》里本来就有 ActuatorTask 这个位置；
  *          - 机械臂动作是"异步动作状态机"（start -> poll -> complete/timeout），
  *            需要一个自己的节奏，而不是跟着底盘 1 ms 跑。
  *
  *          本文件负责命令所有权与周期服务；三轴控制/最高点标定由
  *          arm_driver/arm_calibration处理，达妙帧格式由dm_motor处理。
  *
  *          硬件：达妙 DM4310，ESC/命令 ID 0x01，Master/反馈 ID 0x11，
  *                位置-速度模式（控制帧 CAN ID 0x101），FDCAN1 @ 1 Mbps 经典 CAN。
  ******************************************************************************
  */

#ifndef __ARM_TASK_H__
#define __ARM_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "dm_motor.h"

/* 任务节拍：10 ms（100 Hz）。
 * 达妙位置模式是"发一次目标、电机内部位置环自己保持"，所以不需要高频率；
 * 100 Hz 足够做失联检测（超时 200 ms = 20 拍）与将来的动作状态机轮询。
 * 参考工程是裸机大循环里"一个姿态发一帧"，本任务是它的结构化版本。 */
#define ARM_TASK_PERIOD_MS          (10U)

/* 默认最高点下降联锁；串口 arm guardtop 可在RAM中覆盖。 */
#define ARM_TOP_DROP_LIMIT_MM       (150.0f)
#define ARM_LOW_ROTATION_MIN_DEG    (-44.0f)
#define ARM_LOW_ROTATION_MAX_DEG    (44.0f)

/* 转盘软限位（rad）。位置模式下电机的物理量程就是 ±DM_P_MAX（12.5 rad，
 * 由 DMTool 参数页的 PMAX 确认）。机构角范围和高度联锁由arm_driver另行检查。 */
#define ARM_TURNTABLE_POS_LIMIT_RAD (DM_P_MAX)
/* 手动/调试默认转速（rad/s）。参考工程机械臂动作用 2~10 rad/s，
 * 转盘负载轻，先用保守的 2.0，台上确认后再放大。 */
#define ARM_TURNTABLE_JUMP_RAD      (3.14159265f)   /* 180°，手动点动步长 */

/* 转盘状态快照（给屏幕/日志用） */
typedef struct
{
    float    pos_rad;        /* 当前位置 */
    float    target_rad;     /* 当前位置目标 */
    float    vel_rads;       /* 当前速度 */
    float    tor_nm;         /* 当前扭矩 */
    float    t_mos_c;        /* MOS 温度（老固件可能为 0，见 temp_valid） */
    float    t_coil_c;       /* 线圈温度（同上） */
    uint8_t  temp_valid;
    uint8_t  enabled;        /* 电机自报使能 */
    uint8_t  fault;          /* 1 = 失去控制或电机报故障，已松劲 */
    uint8_t  online;         /* 反馈是否新鲜 */
    uint8_t  state;          /* dm_state_e */
    uint32_t fb_count;
    uint32_t link_lost_count;
} arm_status_t;

/* ===================== 任务与初始化 ===================== */

/**
  * @brief  初始化机械臂模块：绑 FDCAN1、起达妙总线。
  * @note   **必须在 FreeRTOS 调度器启动前调用**（本车在 MX_FREERTOS_Init 里），
  *         因为它内部会 HAL_FDCAN_Init/Start；放到任务里做会和底盘总线的
  *         初始化顺序纠缠。只配总线，**不使能电机**（上电不运动约定）。
  * @retval HAL_OK 总线起来了；HAL_ERROR 表示总线没起来（控制台会打印）。
  */
HAL_StatusTypeDef ArmTask_Init(void);

/* FreeRTOS 任务体（由 freertos.c 用 osThreadNew 创建） */
void ArmTask_Run(void *argument);
HAL_StatusTypeDef ArmTask_PostCommand(const char *line);
HAL_StatusTypeDef ArmTask_RequestRound1(uint32_t token,uint8_t station);
uint8_t ArmTask_GetRound1Result(uint32_t token,uint8_t *success);
/* A successful station also owns a CAMERA result under the same token.
 * Navigation may begin; recognition at the next station must await that result. */
void ArmTask_RequestRound1Reset(void);
void ArmTask_RequestStop(void);

/* ===================== 命令（供控制台/状态机调用） ===================== */

HAL_StatusTypeDef ArmTask_Enable(void);      /* 使能关节（转盘保持力矩） */
HAL_StatusTypeDef ArmTask_Disable(void);     /* 失能关节（转盘可手动转） */
HAL_StatusTypeDef ArmTask_ClearFault(void);  /* 清电机故障并解除锁存 */

/* 绝对角度移动（rad）。转盘的"零点"就是电机保存的零点（dm zero confirm 设定）。 */
HAL_StatusTypeDef ArmTask_MoveToRad(float pos_rad, float vel_rads);
/* 相对当前角度移动 delta（rad），内部会读当前反馈做累加并夹紧 */
HAL_StatusTypeDef ArmTask_TurnRelativeRad(float delta_rad, float vel_rads);

/* 立刻松劲并锁存故障。急停路径（ChassisMotor_EmergencyStop 等）会调它。
 * 幂等：重复调用安全。 */
void ArmTask_EmergencyStop(void);

/* ===================== 状态 ===================== */

void ArmTask_GetStatus(arm_status_t *out);
uint8_t ArmTask_IsReady(void);      /* 总线已起 */
uint8_t ArmTask_IsEnabled(void);    /* 已使能且未锁故障 */
uint8_t ArmTask_IsFaulted(void);    /* 失去控制/电机故障，已松劲 */

/* 直接取电机句柄（给屏幕/诊断用；不要用它绕过 ArmTask 的安全检查发控制帧） */
dm_motor_t *ArmTask_GetMotor(void);
/* ZDT task posts a token; arm owner reports completion without shared FSM edits. */
HAL_StatusTypeDef ArmTask_RequestCameraDeploy(uint32_t token);
HAL_StatusTypeDef ArmTask_RequestMaterial(uint32_t token);
uint8_t ArmTask_GetMaterialResult(uint32_t token,uint8_t *success);
uint8_t ArmTask_GetCameraDeployResult(uint32_t token,uint8_t *success);
HAL_StatusTypeDef ArmTask_RequestStartPose(uint32_t token);
uint8_t ArmTask_GetStartPoseResult(uint32_t token,uint8_t *success);
uint8_t ArmTask_IsStartPoseReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_TASK_H__ */
