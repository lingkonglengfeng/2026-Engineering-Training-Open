/**
  ******************************************************************************
  * @file    dm_motor.h
  * @brief   达妙 DM 关节电机驱动（本车：机械臂转盘水平旋转轴）
  *
  *          硬件与协议基线（2026-09-14 核对，来源见文件末尾"依据"）：
  *          - 总线：FDCAN1，PA11 = RX / PA12 = TX，AF9；**经典 CAN**，1 Mbps；
  *            消息 RAM 偏移 128 word（FDCAN2 的 ZDT 底盘占用 0~127）。
  *          - 电机：ESC/命令 ID = 0x01，Master/反馈 ID = 0x11，
  *            控制模式 = 位置-速度模式（POS_VEL，电机寄存器 10 = 2）。
  *          - 控制帧 CAN ID = ESC_ID + 模式基址：
  *              MIT 0x000 / POS 0x100 / SPD 0x200 / PSI 0x300  → 本车实际用 0x101。
  *          - POS 帧 8 字节：`float32 位置(rad) 小端 | float32 速度(rad/s) 小端`。
  *          - 反馈帧 8 字节：`状态[7:4]|ID[3:0] | pos[15:8] | pos[7:0] |
  *                            vel[11:4] | vel[3:0]<<4 | tor[11:0] | Tmos | Tcoil`。
  *          - 特殊命令帧（8 字节 `FF×7 + cmd`，发到 ESC_ID + 模式基址）：
  *              0xFC 使能 / 0xFD 失能 / 0xFE 保存零点 / 0xFB 清故障。
  *
  *          不自动写电机配置 Flash（不发寄存器写 0x55 / 保存 0xAA）。
  *            电机 ID、模式、波特率一律由上位机一次性配好、掉电保存。
  *            唯一会碰电机 Flash 的是 `DM_Motor_SavePosZero()`（0xFE，改零点），
  *            只由显式 `dm zero save` / `dm zero confirm` 触发。
  *
  *          设计约定（为接入 FreeRTOS 任务准备）：
  *          - **句柄式**：所有状态在 `dm_motor_t` 里，不再用文件级 static 全局；
  *            本车只实例化一台，将来加第二台只需再加一个结构体实例。
  *          - 常规发送/服务不等待反馈；Enable、QueryDisabled和VerifyPosZero
  *            是任务上下文限定的有界握手，等待中osDelay让出CPU。
  *          - **职责单一**：本模块只做"发帧 / 收帧 / 状态"，不做轨迹规划、
  *            不做机构语义（转盘角度、回零那类东西在 arm_task 里）。
  *
  *          依据：
  *          - 官方例程 `docs/vendor/damiao/examples/dm_ctrl_h7_fdcan_v1.1/`
  *          - 参考整车工程 `competition project - 11_25 BUDAHUA_7月18日/User/dm_motor_*.c`
  *          - 台架实测 `docs/hardware/达妙_ZDT_CAN_UART4调试总结_2026-09-09.md`
  *          - 本次移植说明 `docs/hardware/达妙转盘移植说明_2026-09-14.md`
  ******************************************************************************
  */

#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32h7xx_hal.h"

/* ===================== 控制模式 CAN ID 基址 ===================== */
#define DM_MODE_MIT                 (0x000U)
#define DM_MODE_POS                 (0x100U)
#define DM_MODE_SPD                 (0x200U)
#define DM_MODE_PSI                 (0x300U)

/* ===================== 本车电机实际参数 =====================
 * 2026-09-14 由用户提供的 `达妙电机参数.txt`（DMTool 参数页原样导出）确认：
 *     MST_ID: 17 (0x11)   ESC_ID: 1 (0x01)   cmode: 2 (位置-速度)
 *     Gr(减速比): 10      POLE: 14           TIMEOUT: 0（超时保护关闭）
 *     OT_Value: 100（过温保护 100℃）  OC_Value: 0.8A   UV: 15V   OV: 32V
 *     can_br: 0xFFFFFFFF ← **未写入**，见下方 DM_BITRATE_CODE 说明
 * UV=15V / OV=32V 说明这是一台 24V 母线的电机：供电轨必须真的在 24V 附近，
 * 12V 供电下电机不会工作。 */
#define DM_ESC_ID_DEFAULT           (0x01U)   /* 命令帧 ID 基址（寄存器 RID_ESC_ID = 8） */
#define DM_MST_ID_DEFAULT           (0x11U)   /* 反馈帧 ID（寄存器 RID_MST_ID = 7） */

/* Feedback/MIT mapping must match actual motor registers, not nominal defaults.
 * 2026-10-06 dm diag reads PMAX=18.5, VMAX=30, TMAX=10 on this motor.
 * Earlier 12.5 assumption decoded p_m=0.4734838 as about 0.320rad. */
#define DM_P_MAX                    (18.5f)
#define DM_V_MAX                    (30.0f)
#define DM_T_MAX                    (10.0f)
#define DM_KP_MAX                   (500.0f)
#define DM_KD_MAX                   (5.0f)

/* ===================== CAN 波特率代码 =====================
 * 电机寄存器 RID_CAN_BR(35) 用的是官方 SDK 的这套代码（见官方例程 bsp_fdcan.h
 * 的 CAN_BR_* 宏）。本车参数读出来是 0xFFFFFFFF = **未写入**，也就是说这台电机的
 * 波特率很可能是出厂默认、而不是 1 Mbps —— 这与台架上的 LEC=5(bit0) + bus-off 相符
 * （对端波特率不匹配就是会互相破坏波形）。所以驱动必须能改自己的波特率去试，
 * 而不是假定 1 Mbps。 */
#define DM_BITRATE_125K             (0U)
#define DM_BITRATE_200K             (1U)
#define DM_BITRATE_250K             (2U)
#define DM_BITRATE_500K             (3U)
#define DM_BITRATE_1M               (4U)
#define DM_BITRATE_2M               (5U)
#define DM_BITRATE_2M5              (6U)
#define DM_BITRATE_3M2              (7U)
#define DM_BITRATE_4M               (8U)
#define DM_BITRATE_5M               (9U)
#define DM_BITRATE_DEFAULT          (DM_BITRATE_1M)
#define DM_BITRATE_CODE_MAX         (DM_BITRATE_5M)

/* ===================== 运行参数默认值 ===================== */
/* 位置软限位与默认转速：POS 模式的位置范围就是 ±DM_P_MAX，软件先夹紧再下发，
 * 避免把越界值映射成电机侧的怪值。 */
#define DM_POS_LIMIT_RAD            (DM_P_MAX)
#define DM_VEL_DEFAULT_RAD          (2.0f)
#define DM_VEL_LIMIT_RAD            (DM_V_MAX)

/* 周期重发开关：0 = 只在命令改变时发一帧（参考工程的做法）；
 * 1 = 每 DM_RESEND_PERIOD_MS 重发当前目标。
 *
 * 2026-09-14 台架实测后**改为默认 1**，原因有两个，都是实测出来的：
 *   1. 电机**不会自发反馈** —— 收到一条位置指令后只回一帧，之后安静。所以
 *      "使能后 200 ms 收不到反馈就松劲"这个用户要求的保护，在不重发的情况下
 *      **必然误触发**（现场现象：`&dm enable` 之后立刻打印 link lost）。
 *   2. 重发让失联检测变成真的"链路检测"：断了就一定收不到反馈帧。
 * 电机的 TIMEOUT 寄存器实测为 0（超时保护关闭），所以重发不是"防电机自己松劲"，
 * 纯粹是为了感知链路。 */
#ifndef DM_PERIODIC_RESEND
#define DM_PERIODIC_RESEND          (1U)
#endif
#define DM_RESEND_PERIOD_MS         (10U)
/* 进入 bus-off 后自动恢复（0 = 关，只锁存并在 status 里告警）。
 * 为什么默认必须开：FDCAN 进 bus-off 后 **TX 路径被关闭**，
 * HAL_FDCAN_AddMessageToTxFifoQ() 因 TXFQS.TFQF 置位而返回 FIFO_FULL —— 也就是说
 * 控制器**自己不会退出** bus-off（HAL 不提供该 API），100 Hz 重发会永久失败。
 * 台架实测就是 `err=0x08000200 FIFO_FULL` 刷了 1100 多次、控制台全哑。
 * 自动恢复 = 置 CCCR.INIT（停止）→ 清空 TX 缓冲 → 清 CCCR.INIT（重启），
 * 然后电机回到"未使能"状态，必须重新 enable（安全侧：不带着力矩凭空恢复）。 */
#ifndef DM_BUSOFF_AUTO_RECOVER
#define DM_BUSOFF_AUTO_RECOVER      (1U)
#endif
/* 检测到 bus-off 后等多久再亲自恢复（给控制器的自动恢复留机会，避免互相打架） */
#define DM_BUSOFF_RECOVER_DELAY_MS  (500U)

/* 反馈超时：使能状态下超过这么久没有反馈/重发 → 判定"失去控制" → 立即失能。
 * V151: user requested all arm feedback windows800ms. Command/query
 * handshake deadlines remain separate and bounded. */
#define DM_LINK_TIMEOUT_MS_DEFAULT  (800U)

/* ===================== 类型 ===================== */

typedef enum
{
    DM_MIT_MODE = 1,    /* 力矩/位置/速度混合（kp = kd = 0 时纯力矩） */
    DM_POS_MODE = 2,    /* 位置-速度模式（本车使用，电机寄存器 10 = 2） */
    DM_SPD_MODE = 3,    /* 速度模式 */
    DM_PSI_MODE = 4     /* 位置-速度-电流混合（12 字节 FD 帧，经典 CAN 下不可用） */
} dm_mode_e;

/* 健康状态：诊断与安全联锁用 */
typedef enum
{
    DM_STATE_RESET = 0,     /* 结构体刚初始化，总线还没起 */
    DM_STATE_READY,         /* 总线已起，电机未使能 */
    DM_STATE_ENABLED,       /* 已发使能帧，等待/已有反馈 */
    DM_STATE_FAULT          /* 反馈超时或电机报故障，已失能 */
} dm_state_e;

/* 驱动配置（初始化时给一次，之后可改） */
typedef struct
{
    FDCAN_HandleTypeDef *can;           /* 绑定的 FDCAN 句柄（本车 &hfdcan1） */
    uint8_t  esc_id;                    /* 命令帧 ID 基址，默认 DM_ESC_ID_DEFAULT */
    uint8_t  mst_id;                    /* 反馈帧 ID，默认 DM_MST_ID_DEFAULT */
    dm_mode_e mode;                     /* 初始控制模式 */
    uint32_t link_timeout_ms;           /* 反馈超时（0 = 不做超时保护） */
    uint8_t  bitrate_code;              /* CAN 波特率代码（DM_BITRATE_*），见头文件说明 */
    float    pos_limit_rad;             /* 位置软限位（±），夹紧用 */
    float    vel_limit_rad;             /* 速度软限位（±），夹紧用 */
} dm_config_t;

/* 反馈快照（中断里更新，任务里读）。整个快照由 ISR 写入，任务只读取，
 * 这样使能握手不会在编译器优化下读到半更新的值。 */
typedef struct
{
    float    pos_rad;       /* 输出轴位置 rad（16 位无符号映射，中心为 0） */
    float    vel_rads;      /* 速度 rad/s */
    float    tor_nm;        /* 扭矩 N·m */
    float    t_mos_c;       /* MOS 温度 ℃（老固件可能不支持，见 temp_valid） */
    float    t_coil_c;      /* 线圈温度 ℃（同上） */
    uint8_t  err_code;      /* 反馈帧高 4 位的状态/故障码（1 = 使能） */
    uint8_t  enabled;       /* 电机自报使能状态（err_code == 1） */
    uint8_t  temp_valid;    /* 温度字段是否可信（老固件填 0） */
    uint32_t fb_count;      /* 累计反馈帧数（链路健康判据） */
    uint32_t last_tick;     /* 最近一帧的时间戳 ms */
} dm_feedback_t;

/* 电机实例：所有状态都在这里 */
typedef struct
{
    dm_config_t   cfg;
    volatile dm_feedback_t fb;

    float    target_pos_rad;    /* 当前下发的目标位置（**已含 offset**，即真实下发值） */
    float    target_vel_rads;   /* 当前下发的目标速度 */
    float    pos_offset_rad;    /* 软零点偏移（`dm zerohere`）：用户坐标 = 原始 - offset */
    dm_mode_e active_mode;      /* 实际发帧用的模式（Set 系列会改它） */
    dm_state_e state;

    uint8_t  enable_sent;       /* 使能已由反馈确认 */
    volatile uint8_t enable_pending; /* 使能握手进行中；失能/换模式可取消 */
    uint8_t  fault_latched;     /* 故障锁存，需 ClearError 或重新 Init */
    uint8_t  resend_enable;     /* 运行期周期重发开关（`dm resend 0|1` 可改） */
    volatile uint8_t resend_hold;       /* 临时抑制重发（含使能握手） */
    uint8_t  tx_log;            /* 运行期逐帧打印发送内容（`dm txlog 1`） */
    uint32_t resend_next_tick;  /* 下次重发的时间点 */
    uint8_t  last_err;          /* 最近一次 HAL 返回 */
    uint32_t last_fb_count;     /* 上次 Service 看到的反馈计数（判链路是否在动） */

    uint32_t tx_count;          /* 累计发送帧数 */
    uint32_t tx_fail_count;     /* 累计发送失败帧数 */
    uint32_t tx_fail_streak;    /* 连续失败次数（>0 表示发送通道当前不通） */
    uint32_t rx_count;          /* 匹配到本电机的反馈帧数 */
    uint32_t rx_other_count;    /* 收到但不属于本电机的帧（诊断用） */
    uint32_t link_lost_count;   /* 反馈超时失能次数 */
    volatile uint32_t last_rx_tick;      /* 最近一次合法本电机反馈的时间戳 */

    volatile uint32_t can_error_flags;  /* 中断里置位：CAN 错误事件（BUS_OFF/EP/EW/协议错） */
    volatile uint32_t bus_off_count;    /* 进入 bus-off 的次数（由 Service 累加） */
    volatile uint8_t  bus_off_latched;  /* 中断置位、Service 清零：等待恢复处理 */
    uint32_t bus_off_tick;              /* 检测到 bus-off 的时间戳 */
    uint32_t bus_off_recover_count;     /* 自动恢复成功执行的次数 */
    volatile uint32_t stop_generation; /* Emergency/explicit stop must supersede an in-flight clear. */
    /* Read-only parameter scan: owner task advances; ISR only captures replies. */
    volatile uint8_t diag_expected,diag_rid,diag_reply_ready;
    volatile uint32_t diag_request_tick,diag_reply_raw,diag_reply_tick;
    uint8_t diag_active,diag_index;
    uint32_t diag_next_tick;
} dm_motor_t;

/* ===================== 初始化与使能 ===================== */

/**
  * @brief  初始化达妙电机：按 DM_CONFIG 重配并启动 FDCAN1、清状态。
  * @note   只配置总线与解析，**不发控制帧、不使能电机**（上电不运动约定）。
  *         必须在 FreeRTOS 调度器启动前调用（本车在 main.c 的 USER CODE 2）。
  *         调用后请检查返回值：HAL_ERROR 表示总线没起来，此时一切控制都是空发。
  */
HAL_StatusTypeDef DM_Motor_Init(dm_motor_t *motor, const dm_config_t *config);

/* 便捷初始化：用本车默认参数（FDCAN1 / 0x01 / 0x11 / POS / 200 ms 超时） */
HAL_StatusTypeDef DM_Motor_InitDefault(dm_motor_t *motor, FDCAN_HandleTypeDef *can);

void DM_Motor_SetMode(dm_motor_t *motor, dm_mode_e mode);

/*
 * 仅支持当前 POS 模式的安全使能握手：先发 FF..FD 请求一帧新反馈，
 * 以新鲜原始位置建立低速保持目标，再发 FF..FC，并等待状态码=1 的确认。
 * 不写电机 Flash；HAL_OK 只在握手完成且使能反馈确认后返回。
 */
HAL_StatusTypeDef DM_Motor_Enable(dm_motor_t *motor);

/* 失能（FF..FD 帧）+ 清控制量。**安全停机路径必须调它**（失去控制要松劲）。 */
HAL_StatusTypeDef DM_Motor_Disable(dm_motor_t *motor);
HAL_StatusTypeDef DM_Motor_QueryDisabled(dm_motor_t *motor,uint32_t timeout_ms,dm_feedback_t *out);
/* FD-only verification after one FE write; never enables or repeats FE. */
HAL_StatusTypeDef DM_Motor_VerifyPosZero(dm_motor_t *motor,uint32_t timeout_ms);
/* Fresh read: enabled POS repeats its existing target, disabled sends FD.
 * No FC/FE, no retarget, and does not disable an enabled motor. */
HAL_StatusTypeDef DM_Motor_ReadPosition(dm_motor_t *motor,uint32_t timeout_ms,dm_feedback_t *out);
HAL_StatusTypeDef DM_Motor_DiagStart(dm_motor_t *motor);
void DM_Motor_DiagPoll(dm_motor_t *motor,uint32_t now_ms);
void DM_Motor_DiagCancel(dm_motor_t *motor);
uint8_t DM_Motor_DiagIsActive(const dm_motor_t *motor);

/* 清除过温/过载等故障（FF..FB 帧） */
HAL_StatusTypeDef DM_Motor_ClearError(dm_motor_t *motor);

/* 保存当前位置为零点（FF..FE 帧，改电机 Flash，只能由 dm zero confirm 触发） */
HAL_StatusTypeDef DM_Motor_SavePosZero(dm_motor_t *motor);

/* ===================== 控制目标 ===================== */

/* MIT 混合控制：kp = kd = 0 即纯力矩 */
HAL_StatusTypeDef DM_Motor_SetMIT(dm_motor_t *motor, float pos_rad, float vel_rads,
                                  float kp, float kd, float tor_nm);
/* 位置-速度控制（本车使用）；pos/vel 会先按软限位夹紧 */
HAL_StatusTypeDef DM_Motor_SetPos(dm_motor_t *motor, float pos_rad, float vel_rads);
/* 速度控制 */
HAL_StatusTypeDef DM_Motor_SetSpeed(dm_motor_t *motor, float vel_rads);
/* 位置-速度-电流混合（仅 CAN FD 下可用，经典 CAN 编译为空操作） */
HAL_StatusTypeDef DM_Motor_SetPSI(dm_motor_t *motor, float pos_rad, float vel_rads,
                                  float cur_a);
/* 清除全部控制目标（安全态用，不发帧） */
void DM_Motor_ClearCtrl(dm_motor_t *motor);

/* ===================== 周期服务（任务里调） ===================== */

/**
  * @brief  周期服务：超时失能保护 + （可选）目标重发。
  * @param  now_ms 当前毫秒时基（任务里传 HAL_GetTick()）。
  * @note   建议 1~10 ms 调一次。**必须在任务上下文调用**。
  *         失去控制时会把电机失能并在串口打印一次（不刷屏）。
  */
void DM_Motor_Service(dm_motor_t *motor, uint32_t now_ms);

/* 快速失能并锁存故障（供底盘急停等安全路径调用） */
void DM_Motor_EmergencyStop(dm_motor_t *motor);

/**
  * @brief  恢复总线：重新初始化 FDCAN1，冲掉卡死的 TX FIFO / bus-off 状态。
  * @note   存在的理由（2026-09-14 台架实测）：对端不应答时若开了自动重传，
  *         TX FIFO 会被永久占满，此后**连失能帧都发不出去**，而电机可能还带着
  *         力矩 —— 那就完全失去了让它松劲的手段。`dm recover` 是这种情况下的
  *         唯一出路，也是"卡住"时现场该做的第一件事。
  *         会短暂释放电机（重建控制器），并清零诊断计数。
  */
HAL_StatusTypeDef DM_Motor_Recover(dm_motor_t *motor);

/* 发送通道是否当前不通（连续发送失败） */
uint8_t DM_IsTxBlocked(const dm_motor_t *motor);

/* ===================== 状态读取 ===================== */

float    DM_GetPosRad(const dm_motor_t *motor);
float    DM_GetVelRads(const dm_motor_t *motor);
float    DM_GetTorqueNm(const dm_motor_t *motor);
float    DM_GetMosTempC(const dm_motor_t *motor);
float    DM_GetCoilTempC(const dm_motor_t *motor);
uint8_t  DM_GetErrCode(const dm_motor_t *motor);
uint8_t  DM_IsEnabled(const dm_motor_t *motor);          /* 期望状态：已使能且未锁故障 */
uint8_t  DM_IsMotorReportingEnabled(const dm_motor_t *motor);  /* 电机自报使能位 */
uint8_t  DM_IsOnline(const dm_motor_t *motor, uint32_t timeout_ms);
uint32_t DM_GetFbCount(const dm_motor_t *motor);
uint32_t DM_GetLinkLostCount(const dm_motor_t *motor);
uint8_t  DM_IsLinkLost(const dm_motor_t *motor);   /* 1 = 处于失去控制/故障锁存态 */
dm_state_e DM_GetState(const dm_motor_t *motor);
void     DM_GetFeedback(const dm_motor_t *motor, dm_feedback_t *out);
void     DM_GetConfig(const dm_motor_t *motor, dm_config_t *out);
void     DM_SetConfig(dm_motor_t *motor, const dm_config_t *config);

/**
  * @brief  改 CAN 波特率并**立即重配 FDCAN1**（只存在 RAM，不改电机 Flash）。
  * @param  code DM_BITRATE_125K ... DM_BITRATE_5M
  * @note   存在的理由：本车电机的 RID_CAN_BR 读出来是 0xFFFFFFFF（未写入），
  *         实际波特率未知；而台架症状（LEC=5 bit0 + bus-off）与"对端波特率不匹配"
  *         完全一致。所以驱动要先能改自己的波特率去扫，而不是假定 1 Mbps。
  *         会重建 FDCAN1 并清诊断计数。**请先失能电机**。
  *         掉电/复位后回到 DM_BITRATE_DEFAULT。
  */
HAL_StatusTypeDef DM_Motor_SetBitrate(dm_motor_t *motor, uint8_t code);

/* 取当前波特率代码 / 人读名字（"1M"、"500K"...） */
uint8_t     DM_GetBitrateCode(const dm_motor_t *motor);
const char *DM_BitrateName(uint8_t code);

/* 运行期开/关周期重发（1 = 每 DM_RESEND_PERIOD_MS 重发当前目标）。
 * 关掉它就等于"只在命令改变时发帧"，此时失联检测会因为电机不自发反馈而误判。 */
void DM_SetResend(dm_motor_t *motor, uint8_t enable);
uint8_t DM_GetResend(const dm_motor_t *motor);

/* ===================== 中断入口 ===================== */

/* FDCAN1 RX FIFO0 中断处理。由 zdt_motor.c 的 HAL_FDCAN_RxFifo0Callback 分发。
 * 注意：它只消费**绑定到本驱动所绑 FDCAN 句柄**的帧，收到别的 ID 会计数但丢弃。 */
void DM_RxFifo0Handler(dm_motor_t *motor);

/**
  * @brief  FDCAN1 错误状态中断入口（由 HAL_FDCAN_ErrorStatusCallback 分发）。
  * @param  interrupts HAL 传进来的 FDCAN_IT_* 位掩码。
  * @note   **只置标志/累加计数，不打印、不阻塞**（这是中断上下文）。
  *         bus-off 的可见化由调用方在任务里按 bus_off_count 变化补打一条。
  */
void DM_OnCanError(dm_motor_t *motor, uint32_t interrupts);

/* 控制器当前是否处于 bus-off（读 PSR.BO）。bus-off 期间 TX 必然失败，
 * 驱动会跳过发送直到恢复 —— 否则 100 Hz 重发会把 TX FIFO 撑死并刷屏。 */
uint8_t DM_IsBusOff(const dm_motor_t *motor);

/* ===================== 控制台命令 ===================== */

/* 解析 `dm ...` 命令。返回 1 = 已认领该命令（无论成功失败），0 = 不是 dm 命令。
 * 供 zdt_motor.c 的 ZDT_Process_Command 分发。 */
uint8_t DM_Console_ProcessCommand(dm_motor_t *motor, const char *command);

/* Optional owner-side IO pump during bounded handshakes; must not command motion. */
void DM_WaitService(dm_motor_t *motor);
#ifdef __cplusplus
}
#endif

#endif /* __DM_MOTOR_H__ */
