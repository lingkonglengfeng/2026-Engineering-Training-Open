#include "arm_material.h"
#include "arm_round1.h"
#include "start_button.h"
#include "gpio.h"
#include "zdt_motor.h"
#include "console_dma.h"
#include "zdt_query.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bt_button.h"
#include "arm_task.h"
#include "arm_calibration.h"
#include "chassis_motor.h"
#include "match_control.h"
#include "nav_frame.h"
#include "nav_feedback.h"
#include "nav_short_move.h"
#include "nav_odometry.h"
#include "nav_blend.h"
#include "nav_corner.h"
#include "zone2_route.h"
#include "vision.h"
#include "vision_obstacle.h"
#include "vision_align.h"
#include "vision_yaw.h"
#include "vision_align_diag.h"
#include "vision_mode_gate.h"
#include "qr_scan.h"
#include "cmsis_os2.h"
#include "dm_motor.h"
#include "fdcan.h"
#include "grid_nav.h"
#include "screen_hmi.h"
#include "screen_metrics.h"
#include "arm_start_pose.h"
#include "pid_storage.h"
#include "nav_params.h"
#include "usart.h"

#define ZDT_DIAG_HEARTBEAT_ENABLE       (0U)
#define ZDT_MILEAGE_MOTOR_COUNT          (4U)
#define ZDT_MILEAGE_QUERY_TIMEOUT_MS     (400U)
#define ZDT_MILEAGE_QUERY_RETRY_MS       (60U)
#define ZDT_MILEAGE_QUERY_MAX_ATTEMPTS   (5U)
/* 轮周长常量（255.694226 mm/圈，轮径 81.39 mm）移到了 zdt_motor.h，
   以便 nav_params.c 也能用同一份数值。 */
/* 位置模式(0xFD)的脉冲当量：下发多少脉冲 = 电机转 1 圈。
 * 本车实测（`poscal`，单地址即时下发、按"回读 65536 计数 = 1 圈"反推）：
 *   FL 3771 / FR 3771 / RL 3693 / RR 3687 脉冲/圈，取四轮平均 3730。
 * 注意：这是"回读单位取 65536 计数/圈（厂商手册 Emm 口径）"前提下的结果；
 * 若用卷尺量出实际位移只有 DIST 的 1/1.177，说明回读其实是别的尺度，
 * 需要反推修正运行参数 cnt（每圈计数）与这里的值（见文档 §8）。
 * 换驱动器或改细分后必须重测：`poscal` 一条命令即可给出四轮实际脉冲当量。 */
#define ZDT_POSITION_PULSES_PER_REV      (3730.0f)
/* 逐轮下发位置指令的间隔（毫秒）：避免 8 个 CAN 包挤在一起。 */
#define ZDT_NAV_STEP_GAP_MS              (2U)
/* 导航直线段/矫正段的运动速度（RPM）。
 * 2026-09-12 现场实测：`poscal` 在 120 RPM 下四轮脉冲当量一致（3690~3773），
 * 而导航原来用 400 RPM 时，同一次位移里各轮"每脉冲转过的角度"差异极大
 * （前两轮约 0.6 倍、后两轮约 1.6 倍），车会走斜、段末航向偏差 50°。
 * 因此先降到与 `poscal` 相同的 120 RPM，再看位置模式是否稳定。 */
#define ZDT_NAV_POSITION_SPEED_RPM       (120U)
#define ZDT_NAV_POSITION_CORRECT_RPM     (80U)
#define ZDT_NAV_POSITION_ACCELERATION    (200U)
#define ZDT_NAV_POSITION_MARGIN_MS       (1500U)
#define ZDT_NAV_HEADING_TOLERANCE_DEG    (0.5f)
/* 每段开始前"原地转到段方向"的允许偏差（度）：小于它就直接直走。 */
#define ZDT_NAV_ALIGN_TOLERANCE_DEG      (3.0f)
#define ZDT_DEPARTURE_LANE_TOLERANCE_MM   (5)
/* 段末航向偏差上限：超过它说明本段走行严重异常（轮子转错、打滑、撞到东西），
 * 此时再做一次大角度原地转只会把车带到更糟的位置，直接停车报错更安全。 */
#define ZDT_NAV_HEADING_MAX_DEG          (20.0f)
#define ZDT_NAV_REV_TOLERANCE            (0.05f)
/* 段末补走开关。0 = 只测量、只打印，**不驱动**补走（现场实测：不补走时位置刚刚好 ——
 * 时间规划已经把距离算对，残差主要来自里程回读偏差，拿它去"纠正"反而把车推偏；
 * 而且补走本身还受 DRIVE_MIN_MS 下限与斜坡影响，几毫米残差会走出十几毫米动作）。
 * 改 1 就恢复"量 → 补"的闭环，不需要动别的地方。 */
#define ZDT_NAV_END_CORRECTION_ENABLE    (0U)
#define ZDT_NAV_MAX_CORRECTIONS          (3U)
/* 单次矫正的上限：超过它说明里程不可信，宁可放弃矫正也不要一路走偏。 */
#define ZDT_NAV_CORRECT_MAX_REV          (0.50f)
/* 段末判定"轮子已停稳"的位置变化门槛（编码器计数，65536=1圈）、
   两次采样之间的最小间隔与最长等待时间。
   位置模式的实际耗时由驱动器加速度决定，只靠时间估算会在轮子还在转的时候
   就开下一段/转角矫正，现场表现为"上一段还没走完就开始转向或平移"。
   判据要成对使用：间隔 ≥20 ms 时，120 RPM 的轮子会走约 2600 计数，
   大于 1500 的门槛；而停稳后编码器抖动只有几十计数。 */
#define ZDT_NAV_SETTLE_COUNTS            (1500L)
#define ZDT_NAV_SETTLE_MIN_MS            (20U)
#define ZDT_NAV_SETTLE_TIMEOUT_MS        (1500U)
/* 位置模式不持续发速度；航向/位置只在直线段末端复核。 */
#define ZDT_CAN_QUERY_INTERVAL_MS       (1000U)
#define ZDT_CAN_QUERY_MAX_ATTEMPTS      (3U)

/* ---------- 直线段执行方式（运行时可切换：`posmode 0/1`） ----------
 * 默认：速度模式 + 定时推算
 *     用已验证可靠的速度模式（实测四轮一致到 0.03%）走够时间，时间到就发停止；
 *     距离 -> 时间 用梯形/三角形速度规划算出（见 ZDT_Nav_PlanDriveMs），
 *     `t1000` 退化为"整体时间微调系数"。
 *     段末仍用 0x36 里程复核，误差在允许范围内时补一小段直走。
 * `posmode 1`：位置模式（0xFD 脉冲）——实测本车驱动器不稳定（同一轮"每脉冲转角"
 *     会在两次测试之间变、四轮幅值差 3 倍），仅保留用于对比验证。 */
#define ZDT_NAV_USE_POSITION_MODE       (0U)
/* ==================== 只有两个可调参数 ====================
 *   spd<值>     速度：轮速(RPM) = 值 x 250；默认 0.350
 *   t1000<值>   spd=0.50参考速度下的1m时间标定；默认1930ms
 *   nav         打印当前值      navreset  恢复默认
 * 改完用 `save` 存进 W25Q64，`read` 读回来（参数本体在 nav_params.c）。 */
#define ZDT_NAV_DRIVE_MIN_MS            (40U)
/* 段末补走时的降速比例（越小越准，不动它） */
#define ZDT_NAV_CORRECT_SPEED_RATIO     (0.5f)

/* 速度字段单位系数：本车驱动器按 0.1 RPM 解释（见 SetSpeedRpmInternal 注释）。
 * 1.0f = 按 RPM 发送（厂商手册 Emm 口径）；10.0f = 按 0.1RPM 发送（实测配置）。 */
#define ZDT_SPEED_FIELD_SCALE           (10.0f)

#define ZDT_CAN_RUN_ACCELERATION        (0x00U) /* chassis profile owns ramp */
#define ZDT_UART_COMMAND_MAX_LENGTH     (63U)
#define ZDT_CAN_RX_QUEUE_SIZE           (16U)
#define ZDT_UART_RX_RING_SIZE           (128U)
#define ZDT_UART_ISR_DRAIN_LIMIT        (32U)
#define ZDT_UART_TASK_DRAIN_LIMIT       (64U)

typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t received_tick,received_seq;
} ZDT_CAN_RxFrameTypeDef;

static ZDT_CAN_RxFrameTypeDef zdt_rx_queue[ZDT_CAN_RX_QUEUE_SIZE];
static volatile uint32_t zdt_rx_head;
static volatile uint32_t zdt_rx_tail;
static volatile uint32_t zdt_rx_sequence;
static volatile uint8_t zdt_user_query_address;
static zdt_query_t zdt_user_query;
static zdt_motor_status_t zdt_status;
static match_control_t zdt_match;
typedef enum {
  MATCH_IO_NONE, MATCH_IO_LEAVE_START_QUERY, MATCH_IO_LEAVE_SCRIPT,
  MATCH_IO_LEAVE_END_QUERY, MATCH_IO_NAV, MATCH_IO_ZONE2, MATCH_IO_ALIGN, MATCH_IO_CAMERA, MATCH_IO_QRSCAN, MATCH_IO_ARM_DEPLOY, MATCH_IO_CAMERA_RESET, MATCH_IO_ARM_START, MATCH_IO_MATERIAL, MATCH_IO_ROUND1
} zdt_match_io_t;
static zdt_match_io_t zdt_match_io;
/* Explicit bench alignment owns only this correction, never advances MATCH. */
static uint8_t zdt_manual_align_station;
static uint32_t zdt_manual_align_token,zdt_manual_align_next,zdt_manual_align_started;
static uint32_t zdt_match_request;
static uint32_t zdt_arm_camera_token,zdt_arm_camera_started;
static uint32_t zdt_arm_start_token;
static uint32_t zdt_arm_material_token,zdt_arm_round_token;
static uint32_t zdt_prepare_request,zdt_prepare_tick,zdt_prepare_next;
static uint8_t zdt_prepare_stopped,zdt_prepare_retry_printed;
static uint32_t zdt_camera_reset_wait_tick;
static qr_scan_t zdt_qr_scan;
static vision_qr_task_t zdt_qr_task;
static uint8_t zdt_qr_enabled,zdt_qr_active,zdt_qr_for_match,zdt_qr_used,zdt_qr_printed;
static volatile uint8_t zdt_qr_locked;
uint8_t ZDT_Match_GetQrTask(vision_qr_task_t *task)
{
  uint32_t key;uint8_t valid;
  if(!task)return 0U;
  key=__get_PRIMASK();__disable_irq();valid=zdt_qr_locked;__DMB();
  if(valid)*task=zdt_qr_task;
  __set_PRIMASK(key);return valid;
}
static vision_align_config_t zdt_align_config[3];
static vision_align_t zdt_align;
static vision_yaw_t zdt_vyaw;
static uint8_t zdt_ring_profile; /*0=automatic,1=force fast empty,2=force occupied; RAM only. */
static vision_ring_count_t zdt_align_scene;
static vision_plate_scene_t zdt_align_plate_scene;
static vision_align_diag_t zdt_align_diag;
static uint8_t zdt_diag_dump_active,zdt_diag_dump_interrupted;
static uint16_t zdt_diag_dump_index;
static uint32_t zdt_diag_dump_tick;
static volatile uint32_t zdt_can_enqueue_failures;
static uint32_t zdt_diag_can_start;
/* Automatic station vision is enabled on boot; &valign off is a RAM override. */
static uint8_t zdt_align_enabled=1U,zdt_align_phase;
static uint8_t zdt_align_smooth_enabled=1U,zdt_align_search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
static uint8_t zdt_align_status_pending;
static uint32_t zdt_align_status_tick;
static float zdt_align_heading;
static int32_t zdt_align_wheel_start[4];
static uint32_t zdt_align_tick,zdt_align_imu_count,zdt_align_imu_tick;
static uint8_t zdt_temp2_still_tracking,zdt_temp2_skip_align;
static uint32_t zdt_temp2_still_tick;
static float zdt_align_measure_heading;
static vision_mode_gate_t zdt_camera_gate;
static uint8_t zdt_camera_reset_mask;
static zone2_route_t zdt_zone2;
typedef enum { Z2_NONE,Z2_NAV,Z2_TURN } zdt_zone2_phase_t;
static zdt_zone2_phase_t zdt_zone2_phase;
static zone2_action_t zdt_zone2_action;
static uint32_t zdt_zone2_action_id,zdt_zone2_tick,zdt_zone2_imu_tick,zdt_zone2_imu_count;
static const char *zdt_zone2_reason;
static int32_t zdt_match_leave_start[ZDT_MILEAGE_MOTOR_COUNT];
static uint32_t zdt_last_query_tick;
static uint8_t zdt_query_attempts;
static char zdt_uart_command[ZDT_UART_COMMAND_MAX_LENGTH + 1U];
static uint8_t zdt_uart_command_length;
static uint8_t zdt_uart_command_overflow;
static uint8_t zdt_uart_ignore_lf;
static volatile uint8_t zdt_uart_rx_ring[ZDT_UART_RX_RING_SIZE];
static volatile uint8_t zdt_uart_rx_head;
static volatile uint8_t zdt_uart_rx_tail;
/* RX 环形缓冲溢出计数（字节被静默丢弃的次数）。
 * 2026-09-14 加：原来丢了字节没有任何痕迹，导致"命令被腰斩/卡死"完全无法归因。
 * 现在溢出会累加，`status` 会打印，并且在溢出后主动报警一次。 */
static volatile uint32_t zdt_uart_rx_dropped;
/* 字符回显开关（默认关，`echo 0|1` 运行期改）。见 ZDT_UART_ProcessByte 的说明。 */
static uint8_t zdt_uart_echo_enabled = 0U;
static volatile uint8_t zdt_verbose_output = 0U;
/* posq 诊断：>0 时把接下来这么多帧的原始数据打印出来（与 verbose 独立）。 */
static uint8_t zdt_pos_dump_frames;
/* `&echo` 链路自检的序号：每真正收到一次命令才 +1。上位机若显示 4 条同样的行，
   看序号是否重复就能分清"命令被重发了"还是"接收侧重复渲染"（详见 `&echo` 注释）。 */
static uint32_t zdt_echo_sequence;
/* 脚本层与命令层之间的两个"请求"标志（定义在这里是因为蓝牙命令解析在文件前部，
   而脚本实现靠后）：见各自的注释与 ZDT_Script_Service() 的收尾处理。 */
static uint8_t zdt_script_then_nav;    /* 动作跑完后接着跑被推迟的导航（0 = 不用） */
/* 直走中的航向保持（陀螺闭环）参数 */
#define ZDT_NAV_HOLD_PERIOD_MS          (80U)     /* 修正周期 */
#define ZDT_NAV_HOLD_KP                 (0.030f)  /* 每 1° 误差给多少 omega */
#define ZDT_NAV_HOLD_OMEGA_MAX          (0.20f)   /* omega 限幅（1.0 = 满量程旋转） */
#define ZDT_NAV_HOLD_DEADBAND_DEG       (0.20f)   /* 死区，防抖 */

static uint32_t zdt_nav_hold_tick;
static float zdt_nav_drive_scale;
static int8_t zdt_nav_drive_sign;
/* 本次驱动是不是"横移"（≠0 时 vy 走速度、vx=0，且段末不做直走里程复核）。
   0 = 直走；+1/−1 = 向车体右/左横移。见 ZDT_Nav_BeginLateralDrive()。 */
static int8_t zdt_nav_drive_lateral;
/* 横移期间要保持的车头航向（横移开始那一刻的实测值）。 */
static float zdt_nav_lateral_hold_heading;
static float zdt_nav_hold_peak_error;

/* posref / posd 标定辅助：基准位置、有效掩码与两个请求标志。 */
static int32_t zdt_pos_ref[ZDT_MILEAGE_MOTOR_COUNT];
static uint8_t zdt_pos_ref_mask;
static uint8_t zdt_pos_ref_set;
static uint8_t zdt_pos_ref_request;
static uint8_t zdt_pos_delta_request;

/* ---------- 转向体检（每次原地转结束打一行，用于核对 ≤3 s 指标）----------
 * 输出形如：`[TURN] done 1680ms target=+90.00 err=+0.008 overshoot=0.014 peak_o=1.00`
 * 行尾标记：`FAST` 快速档（导航对准）/ `FB` 稳定带判到位 / `ST` 无进展兜底 / `OVER-3s`。 */
static uint8_t zdt_turn_was_active;
static float zdt_turn_initial_sign;
static float zdt_turn_peak_overshoot_deg;
static float zdt_turn_peak_output;

/* ---------- 逐轮脉冲当量自标定（poscal）----------
 * 一条命令测出每个驱动器的三件事：
 *   ① 脉冲当量：下发 N 个脉冲，实际转过多少圈（决定位置模式该发多少脉冲）；
 *   ② 是否真的执行 0xFD 位置指令（不动 = 该驱动器不认这条指令）；
 *   ③ 指令方向与 0x36 回读符号是否一致。
 * 做法：对每个地址依次 读位置 → 下发固定脉冲数的相对位置指令 → 等停 → 再读位置，
 *       用回读角度（厂商手册：Emm 位置 65536 计数 = 1 圈）反推脉冲当量。
 * 位置指令用 mode=0x02（相对当前位置），因此不受"驱动器一直带电、累计角度不清零"影响。 */
#define ZDT_POSCAL_MOVE_PULSES   (2000)
#define ZDT_POSCAL_SPEED_RPM     (120U)
#define ZDT_POSCAL_MOVE_WAIT_MS  (600U)   /* 下发后首次回读的等待 */
#define ZDT_POSCAL_RECHECK_MS    (250U)   /* 复检间隔 */
#define ZDT_POSCAL_STILL_COUNTS  (300)    /* 两次回读差小于它即认为停稳 */
#define ZDT_POSCAL_MAX_PROBES    (6U)
#define ZDT_POSCAL_GAP_MS        (300U)
#define ZDT_POSCAL_TIMEOUT_MS    (10000U)

#define ZDT_POSCAL_IDLE          (0U)
#define ZDT_POSCAL_WAIT_BASE     (1U)
#define ZDT_POSCAL_MOVING        (2U)
#define ZDT_POSCAL_WAIT_RESULT   (3U)

static uint8_t zdt_poscal_active;
static uint8_t zdt_poscal_phase;
static uint8_t zdt_poscal_index;
static uint32_t zdt_poscal_tick;
static uint32_t zdt_poscal_wheel_tick;
static int32_t zdt_poscal_before;
static int32_t zdt_poscal_after;
static int32_t zdt_poscal_sample;
static uint8_t zdt_poscal_probes;
static float zdt_poscal_ppr[ZDT_MILEAGE_MOTOR_COUNT];
static uint8_t zdt_poscal_moved[ZDT_MILEAGE_MOTOR_COUNT];

static const uint8_t zdt_mileage_addresses[ZDT_MILEAGE_MOTOR_COUNT] =
{
  CHASSIS_MOTOR_FL_ADDRESS,
  CHASSIS_MOTOR_FR_ADDRESS,
  CHASSIS_MOTOR_RL_ADDRESS,
  CHASSIS_MOTOR_RR_ADDRESS
};

/* 四轮"回读增量 → 物理前进量"符号表（里程/打印用）。
 *
 * 依据（现场实测）：`&U` 前进时四轮 [MILEAGE] 读数为 **同为负、幅值相等**
 * （-2.4148/-2.4148/-2.4147/-2.4152 rev），而车确实在前进
 * → 回读侧必须取 -CHASSIS_WHEEL_SIGN_*，"前进"才会读成正数。
 * 另一处交叉验证：`poscal` 对四个地址下发同一 +2000 脉冲时，回读符号为
 * FL/RL 负、FR/RR 正（左右列相反），与 CHASSIS_WHEEL_SIGN_* 的分组同源。
 *
 * 注意：位置脉冲下发用的是**另一套**符号（物理前进量 -> 驱动器命令），
 * 见 zdt_nav_command_signs[]，不要与本表混用。 */
static const double zdt_mileage_forward_signs[ZDT_MILEAGE_MOTOR_COUNT] =
{
  -(double)CHASSIS_WHEEL_SIGN_FL,
  -(double)CHASSIS_WHEEL_SIGN_FR,
  -(double)CHASSIS_WHEEL_SIGN_RL,
  -(double)CHASSIS_WHEEL_SIGN_RR
};

/* 位置模式(0xFD)下发用符号：物理前进量 -> 电机命令，与速度模式(chassis 混合
 * 公式里的 CHASSIS_WHEEL_SIGN)保持一致。 */
static const double zdt_nav_command_signs[ZDT_MILEAGE_MOTOR_COUNT] =
{
  (double)CHASSIS_WHEEL_SIGN_FL,
  (double)CHASSIS_WHEEL_SIGN_FR,
  (double)CHASSIS_WHEEL_SIGN_RL,
  (double)CHASSIS_WHEEL_SIGN_RR
};

/* 里程查询基准位置；每次查询成功后用于计算本段四轮增量。 */
static int32_t zdt_mileage_reference[ZDT_MILEAGE_MOTOR_COUNT];
static uint8_t zdt_mileage_reference_mask;
static uint8_t zdt_mileage_reference_announced;
static uint8_t zdt_mileage_query_active;
static uint8_t zdt_mileage_query_report;
static uint8_t zdt_mileage_query_mask;
static uint8_t zdt_mileage_query_attempts;
static uint32_t zdt_mileage_query_tick;
static uint32_t zdt_mileage_query_retry_tick;
static int32_t zdt_mileage_query_position[ZDT_MILEAGE_MOTOR_COUNT];
static uint32_t zdt_mileage_query_baseline_seq,zdt_mileage_query_sample_tick;
static uint8_t zdt_mileage_stop_pending;
static uint8_t zdt_mileage_stop_report;
static uint8_t zdt_mileage_forward_pending;

/* 直线段执行状态机；中间格子不停车，只在段末端测量并矫正。 */
typedef enum
{
  ZDT_NAV_IDLE = 0,
  ZDT_NAV_WAIT_START_QUERY,
  ZDT_NAV_ALIGN_TURN,
  ZDT_NAV_SEND_STEPS,
  ZDT_NAV_MOVE,
  ZDT_NAV_DRIVE,
  ZDT_NAV_WAIT_STOP,
  ZDT_NAV_WAIT_END_QUERY,
  ZDT_NAV_WAIT_HEADING,
  ZDT_NAV_ARC,
  ZDT_NAV_BLEND,
  ZDT_NAV_BLEND_STOP,
  ZDT_NAV_OUTER_CORNER,
  ZDT_NAV_VOBS_STOP,
  ZDT_NAV_VOBS_HOLD
} zdt_nav_state_t;

/* 当前执行路径的直线段、段索引和矫正状态。 */
static zdt_nav_state_t zdt_nav_state;
static grid_nav_segment_t zdt_nav_segments[GRID_NAV_MAX_SEGMENTS];
static uint16_t zdt_nav_segment_count;
static uint16_t zdt_nav_segment_index;
static uint8_t zdt_nav_correction_count;
static int8_t zdt_nav_move_sign;
/* 导航段末是否刚做过原地转角矫正：为 1 时下一段必须先重采四轮位置基准。 */
/* 逐轮位置下发的序列状态（ZDT_NAV_SEND_STEPS）。 */
static float zdt_nav_send_rev;
static int8_t zdt_nav_send_sign;
static uint16_t zdt_nav_send_rpm;
static uint8_t zdt_nav_send_index;
static uint32_t zdt_nav_send_tick;
/* 直走中途打印一次实际生效轮速的时刻（0 = 不打印） */
static uint32_t zdt_nav_rpm_report_tick;
/* 位置模式开关存在 Nav_Params()->position_mode（`posmode 0/1` 或 `save`/`read` 维护） */
/* 本段是否已经"原地转到段方向"：避免在 WAIT_START_QUERY 里反复对向。 */
static uint8_t zdt_nav_align_done;
/* 段末"等轮子停稳"用的上一次位置采样与判定状态。 */
static int32_t zdt_nav_settle_sample[ZDT_MILEAGE_MOTOR_COUNT];
static uint8_t zdt_nav_settle_valid;
static uint8_t zdt_nav_settle_announced;
static uint32_t zdt_nav_settle_sample_tick;
static uint32_t zdt_nav_settle_deadline;
static uint32_t zdt_nav_motion_deadline;
static float zdt_nav_segment_target_rev;
static uint8_t zdt_nav_post_align,zdt_nav_queued_recenter,zdt_nav_recenter_route;
static float zdt_nav_measured_rev;
static uint8_t zdt_nav_odometry_enabled=NAV_ODOMETRY_ENABLED;
/* Single owner reuses the existing four-wheel query service, never a second
 * CAN RX consumer. 1=navigation, 2=script, 0=inactive. */
static nav_odometry_t zdt_odo;
static uint8_t zdt_odo_owner,zdt_odo_wait,zdt_odo_base_pending;
static uint32_t zdt_odo_next_query,zdt_odo_tick,zdt_odo_imu_tick,zdt_odo_imu_count;
static float zdt_odo_mm,zdt_odo_max_scale,zdt_odo_heading;
static float zdt_odo_approach_gain;
static int8_t zdt_odo_forward,zdt_odo_right;
/* Normal obstacle pause keeps the original NAV/Zone2 request alive. */
static vision_obstacle_t zdt_vobs;
static uint8_t zdt_vobs_phase,zdt_vobs_wait,zdt_vobs_state_dirty=1U;
static uint8_t zdt_vobs_last_eligible=255U;
static uint8_t zdt_vobs_auto_started,zdt_vobs_preparing;
/* A departure detour owns the existing script until its interrupted step has
 * actually reached its original map target. */
static uint8_t zdt_vobs_resume_script,zdt_vobs_rebase_script;
static uint8_t zdt_vobs_leave_target,zdt_vobs_depart_detoured;
static uint8_t zdt_vobs_nav_active,zdt_vobs_recenter_active;
static uint32_t zdt_vobs_arm_tick,zdt_vobs_wireless_observation;
static uint16_t zdt_vobs_leave_step_x,zdt_vobs_leave_step_y;
static uint16_t zdt_vobs_leave_exit_x,zdt_vobs_leave_exit_y;
static uint32_t zdt_vobs_state_tick,zdt_vobs_poll_tick,zdt_vobs_next_query;
static uint32_t zdt_vobs_stable_since,zdt_vobs_sample_tick;
static uint16_t zdt_vobs_goal_x,zdt_vobs_goal_y,zdt_vobs_object_x,zdt_vobs_object_y;
static float zdt_vobs_start_x,zdt_vobs_start_y,zdt_vobs_world_yaw;
static float zdt_vobs_counts,zdt_vobs_mm_per_rev;
static int32_t zdt_vobs_baseline[4],zdt_vobs_previous[4];
static double zdt_vobs_signs[4];
static uint8_t zdt_vobs_history_count;
static uint32_t zdt_vobs_history_tick[8];
static float zdt_vobs_history_x[8],zdt_vobs_history_y[8];
/* 最近一次里程测量里四个轮的"物理前进圈数"，仅用于串口诊断打印。 */
static float zdt_nav_wheel_rev[ZDT_MILEAGE_MOTOR_COUNT];
/* 四轮投影的最大-最小差：用于判断是否有个别轮子数据异常（>0.1rev 即需查线）。 */
static float zdt_nav_wheel_spread_rev;
static uint8_t zdt_nav_wait_start_query;
static uint8_t zdt_nav_wait_end_query;
static uint8_t zdt_nav_start_ready;
static uint8_t zdt_nav_end_ready;
static uint8_t zdt_nav_query_failed;
static int32_t zdt_nav_start_position[ZDT_MILEAGE_MOTOR_COUNT];
static int32_t zdt_nav_end_position[ZDT_MILEAGE_MOTOR_COUNT];
static uint8_t zdt_nav_start_mask;
static uint8_t zdt_nav_end_mask;

/* ---------- 90° 圆弧过弯（`&tarc` / `&aspd`）----------
 *
 * 现场要求（2026-09-13）："可不可以试试 90° 圆弧过弯" + "设置个参数：90° 圆弧过弯需要
 * 的时间（时间越短半径越小），以及速度"。
 *
 * 与原地转的关系：现在是"到角点 → 停车 → 原地转 90°（≈1 s，车轮转了但车**没往前**）→
 * 再起步"。圆弧过弯把这一停一转换成一段**边走边转**的四分之一圆：
 *   · 半径 R = 2·v_arc·T/π（T = `&tarc`），所以**时间越短半径越小**；
 *   · 前一段少走 R、后一段少走 R，圆弧本身走 (π/2)R —— 净路程比原来**短** 0.43R；
 *   · 省下的主要就是那次原地转（现场约 1 s/个直角）。
 *
 * 两件会咬人的事（都写进日志，现场一眼能看到）：
 *   ① **几何**：圆弧中途车头正好斜 45°，车体扫过的是 约198 mm外接圆半径（280×280的半对角线，
 *      原地转也是）—— 转角附近必须有这么多余量。所以半径不是想给多大就给多大：
 *      用 GridNav_RotatedBodyFreeMm() 逐位姿精确判定，超了就**逐 5 mm 降半径**，
 *      降到 ZDT_NAV_ARC_MIN_RADIUS_MM 还不行的转角就**放弃圆弧、照旧原地转**。
 *   ② **轮速**：麦轮外轮要跑 v + 300·(v/R)（CHASSIS_SPIN_ARM_MM），R 越小越吃紧；
 *      超过满量程时底盘 Mix 会自动整体缩放（半径不变、车变慢），
 *      所以这里按缩放后的速度算驱动时长，否则会提前停车、把角点位置走丢。
 *
 * 参数（`&tarc<ms>` / `&aspd<比例>`，`&tarc 0` = 关闭 = 出厂默认）都在 flash 里，
 * 现场不用重新编译。
 *
 * 注意：圆弧的执行**跨两个段**（前段末尾 + 后段开头），所以状态机里：
 *   WAIT_START_QUERY 时若"下一段是 90° 拐弯"就**预先武装**（arm）并把本段目标减 R；
 *   本段走完在 WAIT_END_QUERY 里不回到 WAIT_START_QUERY，而是进 ZDT_NAV_ARC；
 *   圆弧走完把净位移（弦）记进 mm 位姿，然后让**后段**目标也减 R 并跳过它的原地对准。 */
#define ZDT_NAV_ARC_MIN_RADIUS_MM   (50.0f)    /* 小于这个半径就不值得/转不动 */
#define ZDT_NAV_ARC_RADIUS_STEP_MM  (5.0f)     /* 几何不过时逐档降半径的步长 */
#define ZDT_NAV_ARC_LEG_MARGIN_MM   (50.0f)    /* 两段各至少要留 R + 这个余量 */
#define ZDT_NAV_ARC_SAMPLES         (9U)       /* 圆弧上取几个位姿做碰撞判定 */
/* 圆弧的**收工判据用陀螺**（现场问"不能通过姿态传感器来进行圆弧转弯吗"）：
   规划时长只是前馈 + 超时兜底，真正决定"转够了没有"的是这条航向判据 ——
   麦轮"边走边转"必然打滑（四轮要同时给出 v±300ω），按时间收工会留下
   "时间到了但还差十几度"的偏航；按陀螺收工则一定落在目标航向上。
   代价：转得慢时圆弧会拖长（半径变大），所以位姿按**实测转角**折算（见收尾处）。 */
#define ZDT_NAV_ARC_DONE_DEG        (1.0f)     /* |目标航向 − 当前航向| ≤ 此值即收工 */
#define ZDT_NAV_ARC_MIN_SWEEP_DEG   (5.0f)     /* 实测转角逐小于此值时位姿按规划值兜底 */
#define ZDT_NAV_ARC_TIMEOUT_MARGIN_MS (400U)   /* 超时兜底 = 规划时长 + 这个值 */

static uint8_t zdt_nav_arc_armed;       /* 本段末尾要接一个圆弧（已把本段减了 R） */
static uint8_t zdt_nav_arc_running;     /* 正在走圆弧（WAIT_END_QUERY 里要特殊收尾） */
static float zdt_nav_arc_radius_mm;
static float zdt_nav_arc_v_mm_s;        /* 圆弧车体速度（已按轮速上限折算） */
static float zdt_nav_arc_delta_yaw;     /* ±90：地图坐标系里的转角（逆时针为正） */
static float zdt_nav_arc_start_heading; /* 圆弧起始的陀螺航向 */
static float zdt_nav_arc_ux;            /* 入射方向（地图单位向量），算实测弦用 */
static float zdt_nav_arc_uy;
static uint32_t zdt_nav_arc_tick;       /* 圆弧开始的时刻（算航向进度用） */
static uint32_t zdt_nav_arc_deadline;   /* 超时兜底时刻（= 开始 + 规划时长 + 余量） */
static uint8_t zdt_nav_arc_by_heading;  /* 1 = 靠陀螺转到位收工，0 = 超时收工 */
static float zdt_nav_arc_planned_ms;    /* 本次圆弧的规划时长（日志用） */
static float zdt_nav_arc_planned_rpm;   /* 本次圆弧的外轮转速（日志用） */

/* Experimental station-departure blend, explicitly enabled in RAM only. */
static uint8_t zdt_nav_endturn_enabled;
static uint8_t zdt_nav_blend_used;
static nav_blend_t zdt_nav_blend;
static int32_t zdt_nav_blend_position[4];
static float zdt_nav_blend_yaw, zdt_nav_blend_x, zdt_nav_blend_y;
static uint32_t zdt_nav_blend_sample_tick, zdt_nav_blend_control_tick;
static uint32_t zdt_nav_blend_query_tick, zdt_nav_blend_deadline;
static uint32_t zdt_nav_blend_imu_count, zdt_nav_blend_imu_tick;
static uint8_t zdt_nav_blend_sample_ready;
static uint8_t zdt_nav_blend_deferred_input;
static uint32_t zdt_nav_blend_progress_tick;
static float zdt_nav_blend_progress_mark, zdt_nav_blend_error_mark, zdt_nav_blend_cross_mark;
#define ZDT_BLEND_PERIOD_MS (20U)
#define ZDT_BLEND_QUERY_MS (20U)
#define ZDT_BLEND_MARGIN_MM (10U)
#define ZDT_NAV_BLEND_ENABLE (0U) /* Rollback: planned heading changes only while stopped. */

/* V130 RAM trial. Legacy tarc stays disabled in encoder navigation. */
static uint8_t zdt_outer_enabled=1U,zdt_outer_cross_action,zdt_outer_sample_ready;
static float zdt_outer_radius=NAV_CORNER_DEFAULT_RADIUS_MM,zdt_outer_speed=NAV_CORNER_DEFAULT_SPEED_MM_S;
static nav_corner_t zdt_outer;
static int32_t zdt_outer_position[4];
static uint32_t zdt_outer_sample_tick,zdt_outer_control_tick,zdt_outer_query_tick,zdt_outer_deadline;
static uint32_t zdt_outer_imu_count,zdt_outer_imu_tick,zdt_outer_progress_tick,zdt_outer_settle_tick;
static uint32_t zdt_outer_started_count,zdt_outer_fallback_count;
static float zdt_outer_progress_mark;
static uint8_t zdt_outer_stage;

/* 四轮位置查询异步状态；report=1 时查询完成后打印本段里程。 */
static void ZDT_Mileage_StartQuery(uint8_t report);
static void ZDT_CAN_SendAllVersionQueries(void);
static void ZDT_Mileage_RequestStop(uint8_t report);
static void ZDT_Mileage_RequestForward(void);
static void ZDT_Mileage_Service(void);
static void ZDT_Mileage_HandleFrame(const FDCAN_RxHeaderTypeDef *header,
                                    const uint8_t *data, uint8_t data_length,uint32_t received_tick,uint32_t received_seq);
static void ZDT_Nav_Start(void);
static void ZDT_Nav_Cancel(void);
/* 定点动作脚本（启停区出场/回场）：定义在后面，先声明 */
static void ZDT_Script_Service(void);
static void ZDT_Script_StartLeave(void);
static void ZDT_Script_StartHome(void);
static void ZDT_Script_SetStartZone(uint8_t zone);
static void ZDT_Script_StartNudge(char map_dir, uint16_t mm);
static void ZDT_Script_StartCross(uint8_t target_zone);
static void ZDT_Nav_StartPending(void);
static void ZDT_Nav_RefreshCostModel(void);
static uint8_t ZDT_Script_StartLaneSnapIfNeeded(void);
static uint8_t ZDT_Script_PoseInStartZone(void);
static uint8_t ZDT_Nav_IsActive(void);
static float ZDT_Nav_GetHeading(void);
static float ZDT_Zone2_RelYaw(float world);
static void ZDT_Nav_Service(void);
static void ZDT_Nav_OnQueryComplete(uint8_t mask);
static void ZDT_Nav_OnQueryFailed(void);
static void ZDT_Nav_Fail(const char *reason);
static float ZDT_Nav_SegmentHeadingDeg(const grid_nav_segment_t *segment);



static void ZDT_PosCal_Service(void);
static void ZDT_PosCal_Start(void);
static void ZDT_PosCal_OnQueryComplete(uint8_t mask);
static void ZDT_PosCal_Report(uint8_t index, uint8_t responded);
static uint8_t ZDT_Match_Command(const char *command, uint32_t received_tick);
static uint8_t ZDT_Match_StartMotion(match_motion_t motion, uint8_t zone,
                                     uint32_t request);
static void ZDT_Match_CancelMotion(void);
static void ZDT_Camera_Reset(void);
static void ZDT_Match_Changed(match_state_t state, match_checkpoint_t checkpoint);
static void ZDT_Match_Complete(uint8_t success, const char *reason);
static void ZDT_Match_OnMileage(uint8_t mask);
static void ZDT_Match_OnScriptDone(void);
static uint8_t ZDT_Zone2_StartAction(const zone2_action_t *action,uint32_t id);
static void ZDT_Zone2_Cancel(void);
static void ZDT_Zone2_Finished(uint8_t success);
static void ZDT_Zone2_Service(void);
static void ZDT_Zone2_OnMileage(uint8_t mask);
static void ZDT_Nav_NotifyArrived(void);
static uint8_t ZDT_Align_Start(uint8_t station);
static void ZDT_Align_Service(void);
static void ZDT_Align_OnMileage(uint8_t mask);
static uint8_t ZDT_Outer_TryStart(void);
static void ZDT_Outer_OnQuery(uint8_t mask);
static void ZDT_Outer_Service(void);
static uint8_t ZDT_Outer_Command(const char *command);
static uint8_t ZDT_Align_Command(const char *command);
static uint8_t ZDT_Align_DiagBusy(void);
static uint8_t ZDT_UserQuery_Command(const char *command);
/* 转向调参与波形命令（&tstat / &ttrace）。蓝牙 token 与裸串口两条路都要认领，
 * 所以这里先声明、实现放在 ZDT_TurnLog_Service 旁边。 */
static uint8_t ZDT_Turn_Command(const char *command);
static void ZDT_Align_RequestCameraMode(uint8_t station);
static vision_mode_action_t ZDT_Camera_ModeStep(void);
static void ZDT_Camera_Service(void);
static uint8_t ZDT_Qr_Start(uint8_t for_match);
static void ZDT_Qr_Service(void);
static void ZDT_Qr_Cancel(void);
static uint8_t ZDT_Qr_Command(const char *command);
static uint8_t ZDT_Odo_Start(uint8_t owner,float mm,float scale,int8_t forward,int8_t right,
    float heading,const int32_t *base,uint8_t mask);
static uint8_t ZDT_Odo_Service(uint8_t owner);
static uint8_t ZDT_Odo_OnMileage(uint8_t mask);
static void ZDT_Odo_Cancel(void);
static uint8_t ZDT_Vobs_Command(const char *command,uint32_t received_tick);
static uint8_t ZDT_Vobs_OnMileage(uint8_t mask);
static void ZDT_Vobs_Service(void);
static void ZDT_Vobs_Reset(void);
static void ZDT_Vobs_State(void);
static float ZDT_Vobs_WorldYaw(void);
static void ZDT_Vobs_MatchState(match_state_t state);
static uint8_t ZDT_Vobs_IsLeaveOdo(void);
static void ZDT_Vobs_MapAxes(float yaw,int16_t dx,int16_t dy,int8_t *forward,int8_t *right);
static float ZDT_Vobs_OdoTarget(uint8_t owner,float mm);
static void ZDT_Vobs_RecordOdo(void);

static float ZDT_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}
static uint8_t ZDT_CAN_DataLengthBytes(uint32_t data_length)
{
  uint8_t dlc = (uint8_t)((data_length >> 16U) & 0x0FU);
  return (dlc <= 8U) ? dlc : 8U;
}

/* V152: all UART4 output is bounded producer enqueue, never wire waits. */
static void ZDT_Console_WriteBytes(const uint8_t *data,size_t length)
{(void)ConsoleDma_Write(data,length);}
void ZDT_Console_Print(const char *text){ConsoleDma_Print(text);}

static void ZDT_LogStatus(void)
{
    console_dma_stats_t q;char line[220];ConsoleDma_GetStats(&q);
    (void)snprintf(line,sizeof(line),"[LOG DMA] queued=%lu high=%lu sent=%lu dropped=%lu calls=%lu errors=%lu timeout=%lu busy=%lu abort=%lu\r\n",
     (unsigned long)q.queued_bytes,(unsigned long)q.high_water,(unsigned long)q.sent_bytes,(unsigned long)q.dropped_bytes,
     (unsigned long)q.dropped_calls,(unsigned long)q.tx_errors,(unsigned long)q.tx_timeouts,(unsigned long)q.tx_busy,(unsigned long)q.aborts);
    ZDT_Console_Print(line);
}

static void ZDT_Print_Prompt(void)
{
  ZDT_Console_Print("ZDT> ");
}

/* 取命令字之后的第一个非空白字符。
 *
 * 蓝牙 token 会**保留内部空格**（`&startpose 2` 到达时是 "startpose 2"），所以参数
 * 不能用固定下标取。曾经写成 `event.text[9] == '2'` —— 第 9 位正是那个空格，于是
 * **`&startpose 2` 被静默当成 1 号区**（只有写成 "&startpose2" 才对）。
 * 现场症状（2026-09-13，用户实测）：车摆在右上角 2 号区、固件按 1 号区算（位姿当作
 * (2250,150)、车头当作 +y），于是
 *   · `&leave` 跟着 1 号区动作表走，打印的落点 (2125,450) "看着完全正确"，
 *     但车实际朝右多平移 250 mm，半个车身悬在场地外；
 *   · 紧接着 `&goto raw` 用错误的起点/朝向规划，车直接冲出场地。
 * 教训：**参数缺失/写错一律报 usage，不许有"默认猜一个"**。 */
static const char *ZDT_SkipSpaces(const char *text)
{
  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }
  return text;
}

static uint8_t ZDT_CommandIsGoto(const char *command)
{
  while ((*command == ' ') || (*command == '\t') || (*command == '&'))
  {
    command++;
  }
  return (uint8_t)(strncmp(command, "goto", 4U) == 0);
}

/* `&echo` 链路自检：每真正收到一次命令就打印一条带序号的行。
 * 串口/蓝牙日志里经常看到"同一行重复 4 次"（甚至字符交错），原因只有两种：
 *   ① 上位机或蓝牙模块把同一条命令重复投递了（固件真的执行了 4 次）；
 *   ② 接收侧重复渲染/重复显示，固件只收到一次。
 * 两条路在序号上完全不同：
 *   n=7 出现 4 次   -> 固件只收了一次，重复在接收/显示侧；
 *   n=7,8,9,10      -> 命令真的被重发了，要查上位机发送逻辑。
 * 注意这不只是"日志好看"的问题：命令被重发是**功能性**故障。`&goto` 重发不会
 * 被忽略 —— 每条 `&goto` 都会走 `ZDT_Nav_Start()`，而它开头就 `ZDT_Nav_Cancel()`
 * （停车 + 回 IDLE + 按当前位姿重新规划），现场表现是"走到一半顿一下又重新起步"；
 * `&startpose` 重发更是会把刚摆好的位姿与航向基准一次次冲掉。 */
static void ZDT_Print_Echo(void)
{
  char echo_line[64];

  zdt_echo_sequence++;
  (void)snprintf(echo_line, sizeof(echo_line), "[ECHO] n=%lu tick=%lums\r\n",
                 (unsigned long)zdt_echo_sequence,
                 (unsigned long)HAL_GetTick());
  ZDT_Console_Print(echo_line);
}

static uint8_t ZDT_ArmCommand(const char *line)
{
  HAL_StatusTypeDef result;
  while(*line==' ' || *line=='\t' || *line=='&')line++;
  if(!((strncmp(line,"arm",3)==0 && (line[3]==0 || line[3]==' ' || line[3]=='\t')) ||
       (strncmp(line,"dm",2)==0 && (line[2]==0 || line[2]==' ' || line[2]=='\t'))))return 0U;
  result=ArmTask_PostCommand(line);
  if(result!=HAL_OK)ZDT_Console_Print("[ARM] command queue busy; resend when idle\r\n");
  return 1U;
}
static void ZDT_ProcessBluetoothButtons(void)
{
  bt_button_event_t event;

  while (BT_Button_Pop(&event) != 0U)
  {
    if((zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) &&
        event.id!=BT_BUTTON_CUSTOM && event.id!=BT_BUTTON_STOP) {
      zdt_nav_blend_deferred_input=1U; continue;
    }
    if (((MatchControl_IsActive(&zdt_match) != 0U) || zdt_qr_active || ArmMaterial_IsActive() || ArmRound1_IsActive()) &&
        (event.id != BT_BUTTON_CUSTOM) && (event.id != BT_BUTTON_STOP))
    {
      ZDT_Console_Print("[MATCH] chassis owned by flow; use ok or STOP.\r\n");
      continue;
    }
    switch (event.id)
    {
      case BT_BUTTON_ENABLE:
      {
        HAL_StatusTypeDef result=ChassisMotor_Enable();char line[80];
        (void)snprintf(line,sizeof(line),"[CHASSIS] enable HAL=%u (0=queued,1=error,2=busy)\r\n",(unsigned int)result);
        ZDT_Console_Print(line);
        break;
      }

      case BT_BUTTON_M1:
        (void)ChassisMotor_SetMode(CHASSIS_MODE_TURN);
        break;

      case BT_BUTTON_M2:
        (void)ChassisMotor_SetMode(CHASSIS_MODE_TRANSLATION);
        break;

      case BT_BUTTON_FORWARD:
        ZDT_Mileage_RequestForward();
        break;

      case BT_BUTTON_LEFT:
        (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_LEFT);
        break;

      case BT_BUTTON_BACKWARD:
        (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_BACKWARD);
        break;

      case BT_BUTTON_RIGHT:
        (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_RIGHT);
        break;

      case BT_BUTTON_ROTATE_RIGHT:
      {
        int16_t magnitude = (event.angle_deg < 0) ? (int16_t)-event.angle_deg : event.angle_deg;
        if (ChassisMotor_TurnRelative((int16_t)-magnitude) != HAL_OK)
        {
          ZDT_Console_Print("[TURN] rejected: need &EN and IMU zero-drift ready\r\n");
        }
        break;
      }

      case BT_BUTTON_ROTATE_LEFT:
      {
        int16_t magnitude = (event.angle_deg < 0) ? (int16_t)-event.angle_deg : event.angle_deg;
        if (ChassisMotor_TurnRelative(magnitude) != HAL_OK)
        {
          ZDT_Console_Print("[TURN] rejected: need &EN and IMU zero-drift ready\r\n");
        }
        break;
      }

      case BT_BUTTON_CUSTOM:
      {
        uint32_t revision = GridNav_GetRevision();
        if(strcmp(event.text,"logstat")==0){ZDT_LogStatus();break;}
        if (ZDT_UserQuery_Command(event.text) != 0U) { break; }
        if (ZDT_Match_Command(event.text, event.tick_ms) != 0U) { break; }
        if (Vision_ProcessCommand(event.text) != 0U) { break; }
        if (ZDT_ArmCommand(event.text) != 0U) { break; }
        if (strcmp(event.text, "echo") == 0)
        {
          /* &echo：链路自检，见 ZDT_Print_Echo()。蓝牙这条路上尤其要能分清
             "命令被重发"和"接收侧重复显示"。 */
          ZDT_Print_Echo();
          break;
        }
        if (strcmp(event.text, "zero") == 0)
        {
          /* &zero：摆好车头方向后重新对航向基准（并把位姿归零）。
             调用后必须静止约 3 s，等零漂窗口（HWT101_CALIB_MS）+ 角速度零偏重新估计完。
             屏幕会显示 `IMU ZERO CAL x.xs` 倒计时，完成后变 `IMU ZERO DONE`。 */
          ChassisMotor_ClearHeadingReference();HWT101_ResetZeroDrift();
      
          ZDT_Console_Print("[BT] heading zero re-armed; keep still ~3s.\r\n");
          break;
        }
        if ((strncmp(event.text, "startpose", 9U) == 0) ||
            (strncmp(event.text, "startzone", 9U) == 0))
        {
          /* &startpose 1|2：设定启停区起始位姿 + 车头朝向绑定 + 重对航向基准。
             需要在启停区里摆好车、且之后静止约 3 s（屏幕 IMU ZERO CAL 倒计时）。
             ⚠ 参数从跳过空格后的位置取：`&startpose 2` 与 `&startpose2` 都要能用，
             且**参数不对就报 usage**，绝不默认当 1 号区（见 ZDT_SkipSpaces 注释）。 */
          const char *arg = ZDT_SkipSpaces(event.text + 9U);

          if ((*arg != '1') && (*arg != '2'))
          {
            ZDT_Console_Print("[NAV] usage: &startpose 1|2"
                           "  (1=bottom-right zone, 2=top-right zone)\r\n");
            break;
          }
          ZDT_Script_SetStartZone((uint8_t)(*arg - '0'));
          break;
        }
        if (strncmp(event.text, "nudge", 5U) == 0)
        {
          /* &nudge u|d|l|r [mm]：朝向自检 —— 沿**地图**方向走一小段，
             用来验证"摆放朝向"与"固件绑定"是否一致（转向是相对的，陀螺查不出朝反） */
          char dir = 0;
          const char *args = ZDT_SkipSpaces(event.text + 5U);
          uint16_t mm = 50U;

          if ((*args == 'u') || (*args == 'd') || (*args == 'l') || (*args == 'r'))
          {
            dir = *args;
            args = ZDT_SkipSpaces(args + 1);
            if (*args != '\0')
            {
              char *end;unsigned long distance;
              if(*args<'0' || *args>'9') {
                ZDT_Console_Print("[NAV] nudge needs an integer distance 1..1000mm\r\n");break;
              }
              distance=strtoul(args,&end,10);
              if(distance<1UL || distance>1000UL || *ZDT_SkipSpaces(end)!='\0') {
                ZDT_Console_Print("[NAV] nudge needs an integer distance 1..1000mm\r\n");break;
              }
              mm=(uint16_t)distance;
            }
            ZDT_Script_StartNudge(dir, mm);
          }
          else
          {
            ZDT_Console_Print("[NAV] usage: &nudge u|d|l|r [mm]\r\n");
          }
          break;
        }
        if (strncmp(event.text, "cross", 5U) == 0)
        {
          /* &cross [1|2]：从一个启停区直走到另一个启停区
             （出场 → 沿右边缘车道直走 1500 mm → 掉头 180° → 进区），
             到位后自动把当前区号/位姿/车头绑定切到目标区。 */
          const char *arg = ZDT_SkipSpaces(event.text + 5U);
          uint8_t current = ZDT_Script_GetStartZone();
          uint8_t target;

          if (*arg == '\0')
          {
            target = (uint8_t)((current == 1U) ? 2U : 1U);   /* 默认 = 另一个区 */
          }
          else if ((*arg == '1') || (*arg == '2'))
          {
            target = (uint8_t)(*arg - '0');
          }
          else
          {
            ZDT_Console_Print("[NAV] usage: &cross [1|2]"
                           "  (no arg = the other start zone)\r\n");
            break;
          }
          if (target == current)
          {
            ZDT_Console_Print("[NAV] cross refused: already in that start zone"
                           " (use &leave to move out, or &startpose N to correct"
                           " the zone)\r\n");
            break;
          }
          ZDT_Script_StartCross(target);
          break;
        }
        if (strncmp(event.text, "leave", 5U) == 0)
        {
          /* &leave：出场 —— 前进 300 mm 再平移 125 mm 到外环带中心线 */
          ZDT_Script_StartLeave();
          break;
        }
        if (strncmp(event.text, "home", 4U) == 0)
        {
          /* &home：回场 —— 平移 125 mm 再后退 300 mm 进启停区 */
          ZDT_Script_StartHome();
          break;
        }
        if (strcmp(event.text, "poscal") == 0)
        {
          /* 与串口控制台同名命令：现场常只接蓝牙，因此这里也挂一份。 */
          ZDT_PosCal_Start();
          break;
        }
        if ((strcmp(event.text, "dis") == 0) ||
            (strcmp(event.text, "off") == 0))
        {
          /* &dis / &off：关闭四轮使能 → 闭环步进解除锁轴，可以手转轮子做标定 */
          (void)ChassisMotor_Disable();
          ZDT_Console_Print("[BT] motors disabled; wheels turn freely by hand.\r\n");
          break;
        }
        if (strcmp(event.text, "posref") == 0)
        {
          zdt_pos_ref_request = 1U;
          ZDT_Console_Print("[ZDT] posref: sampling current positions...\r\n");
          ZDT_Console_Print("[ZDT] tip: if wheels are locked, send &dis first.\r\n");
          ZDT_Mileage_StartQuery(0U);
          break;
        }
        if (strcmp(event.text, "posd") == 0)
        {
          zdt_pos_delta_request = 1U;
          ZDT_Console_Print("[ZDT] posd: sampling and comparing with posref...\r\n");
          ZDT_Mileage_StartQuery(0U);
          break;
        }
        if (strcmp(event.text, "verall") == 0)
        {
          ZDT_CAN_SendAllVersionQueries();
          break;
        }
        if (strncmp(event.text, "posmode", 7U) == 0)
        {
          if(zdt_nav_odometry_enabled) {
            Nav_Params()->position_mode=0U;
            ZDT_Console_Print("[NAV] ODOMETRY_V2_4 active; posmode cannot select timed/0xFD transport.\r\n");break;
          }
          /* 蓝牙端只做"切换"（token 里带空格不方便），控制台可用 `posmode 0/1`。 */
          Nav_Params()->position_mode =
              (uint8_t)((Nav_Params()->position_mode != 0U) ? 0U : 1U);
          ZDT_Console_Print(Nav_Params()->position_mode != 0U ?
                         "[ZDT] nav mode = POSITION (0xFD pulses)\r\n" :
                         "[ZDT] nav mode = VELOCITY + timed drive\r\n");
          break;
        }
        if (Nav_Params_ProcessCommand(event.text) != 0U)
        {
          /* &nav / &ns10.14 / &nm80 / ... 与串口同一套参数命令 */
          break;
        }
        if (strcmp(event.text, "posq") == 0)
        {
          zdt_pos_dump_frames = 16U;
          ZDT_Console_Print("[ZDT] posq: dumping raw 0x36 replies...\r\n");
          ZDT_Mileage_StartQuery(0U);
          break;
        }
        /* 规划前刷新代价模型：把"转向耗时相当于走几格"按**当前 spd** 算好，
           这样 A* 就是在比较"转向时间 + 行驶时间"（见 ZDT_Nav_RefreshCostModel）。 */
        ZDT_Nav_RefreshCostModel();
        if (GridNav_ProcessCommand(event.text) != 0U)
        {
          if ((ZDT_CommandIsGoto(event.text) != 0U) &&
              (GridNav_GetRevision() != revision))
          {
            /* ★ 现场要求："开始的时候指令无论是什么都是先出库"。
               车还在启停区里时收到 `&goto`，**先自动出场**（前进 300 + 平移 125
               到车道中线），出场完再按新位姿重新规划出发（ZDT_Nav_StartPending）。
               这样"在库里直接发目标点"不会再让车从区中心斜着往外蹭 ——
               区三面是围栏/场边，只有"先前进再平移"这一条路出得去。 */
            if ((ZDT_Script_PoseInStartZone() != 0U) &&
                (ZDT_Script_IsStartPoseSet() != 0U))
            {
              ZDT_Console_Print("[NAV] car is in the start zone: leaving first,"
                             " then running the goto\r\n");
              zdt_script_then_nav = 1U;
              ZDT_Script_StartLeave();
            }
            else if (ZDT_Script_StartLaneSnapIfNeeded() == 0U)
            {
              ZDT_Nav_Start();
            }
          }
        }
        else if (DM_Console_ProcessCommand(ArmTask_GetMotor(), event.text) != 0U)
        {
          /* &dm ...：达妙转盘命令走蓝牙这条路的入口。
           *
           * 三个地方都必须有它，缺一个就会出现"要么被静默丢弃、要么收到也
           * 什么都不干"这种最难查的现场现象：
           *   1. bt_button.c 的 BT_Button_IsCustomPrefix()  —— token 存活
           *   2. bt_button.c 的 BT_Button_IsCustomToken()   —— flush 时派发
           *   3. bt_button.c 的 BT_Button_IsArgCommand()    —— 空格保留（带参数）
           *   4. 就是这里                                     —— 真正路由到解析器
           * 前三个由 tools/check_uart_command_paths.py 机械校验。
           *
           * 注：蓝牙事件文本只有 16 字节（bt_button_event_t.text[]），载荷上限
           * 15 字符，所以 `dm mit ...` 这种长命令请走有线控制台。 */
        }
        else if (ZDT_Turn_Command(event.text) != 0U)
        {
          /* &tstat / &ttrace：转向状态查询与波形开关（实现在 ZDT_TurnLog_Service 旁） */
        }
        else if (PID_Storage_ProcessCommand(event.text) == 0U)
        {
          /* 走到这里说明 token 通过了白名单、但没有任何处理器认领它
             （典型是命令名写错）。以前是"什么都不打印"，现场看不出是没收到
             还是没匹配；这里给一行提示，ASCII 输出。 */
          char unknown_line[80];

          (void)snprintf(unknown_line, sizeof(unknown_line),
                         "[BT] unknown command: '%s'\r\n", event.text);
          ZDT_Console_Print(unknown_line);
        }
        break;
      }

      case BT_BUTTON_STOP:
      {
        ArmTask_RequestStop();
        chassis_command_t previous_command = ChassisMotor_GetCommand();
        if (MatchControl_IsActive(&zdt_match) != 0U) {
          MatchControl_Abort(&zdt_match);
        } else ZDT_Match_CancelMotion();
        ZDT_Nav_Cancel();
        (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
        ZDT_Mileage_RequestStop(
            (previous_command == CHASSIS_COMMAND_FORWARD) ? 1U : 0U);
        break;
      }

      default:
        break;
    }
  }
}
static void ZDT_Print_Help(void)
{
  /* 2026-09-13 重写：旧文本还写着 "8x8 map (0..7)"、"map x=right y=down"，
     而且没有 &startpose/&leave/&home 等新命令 —— 照着旧 help 操作会错。
     ASCII only（ARMCC 对中文字符串字面量报 #870）。 */
  ZDT_Console_Print("\r\n=== commands (press Enter after each; & form also works over BT) ===\r\n");
  ZDT_Console_Print("-- drive / motion --\r\n");
  ZDT_Console_Print("  &EN           enable holding torque (saved PID needs &read)\r\n");
  ZDT_Console_Print("  &M1 / &M2     mode: turn-in-place / strafe\r\n");
  ZDT_Console_Print("  &U &D &L &R   manual fwd/back/left/right (1/3 speed)\r\n");
  ZDT_Console_Print("  &STOP         stop (after &U it also prints wheel mileage)\r\n");
  ZDT_Console_Print("  &L90 / &R90   relative heading turn, PID, 90 deg\r\n");
  ZDT_Console_Print("  &L180/&R180   same precise-profile turn, 180 deg (1..360 accepted)\r\n");
  ZDT_Console_Print("  &pid          print current turn PID (RAM), &kp/&ki/&kd set one\r\n");
  ZDT_Console_Print("  &tstat        turn/PID/IMU status; &ttrace 1|0 = 40Hz [TURNW] wave\r\n");
  ZDT_Console_Print("  &dis / &off   disable motors (wheels free for hand calibration)\r\n");
  ZDT_Console_Print("-- map / navigation (48x48 x 50mm, origin=left-bottom, y UP) --\r\n");
  ZDT_Console_Print("  &map / &grid        print the whole map + pose + goal\r\n");
  ZDT_Console_Print("  &waypoints / &wp    print named waypoints (mm)\r\n");
  ZDT_Console_Print("  &goto raw           plan+run to a NAME: raw rough1..3 temp1..3\r\n");
  ZDT_Console_Print("                      qr h1 h2 exit1 exit2 center\r\n");
  ZDT_Console_Print("  &goto 1200 2250     plan by mm (a value >=48 means mm)\r\n");
  ZDT_Console_Print("  &goto 24 3          plan by cell (both values < 48)\r\n");
  ZDT_Console_Print("  &obs x y / &free x y  add/remove a runtime obstacle\r\n");
  ZDT_Console_Print("                (both <48 = cell, else rule-book mm; RAM only)\r\n");
  ZDT_Console_Print("  &clear              clear runtime obstacles (blocks stay)\r\n");
  ZDT_Console_Print("  &pose x y / &posemm x y   set current pose (cell / mm)\r\n");
  ZDT_Console_Print("  &startpose 1|2      set start-zone pose + heading binding\r\n");
  ZDT_Console_Print("  &leave              zone exit: fwd 300mm then strafe 125mm\r\n");
  ZDT_Console_Print("  &camraw 1|2|all|off  raw camera RX on this debug UART\r\n");
  ZDT_Console_Print("  &camstat             camera byte/error/task counters\r\n");
  ZDT_Console_Print("  &valign on|off|status automatic station pixel alignment\r\n");
  ZDT_Console_Print("  &home               return: strafe 125mm then back 300mm\r\n");
  ZDT_Console_Print("  &cross [1|2]        drive straight to the OTHER start zone:\r\n");
  ZDT_Console_Print("                      exit -> 1500mm straight on lane x=2125 ->\r\n");
  ZDT_Console_Print("                      180 turn -> enter; switches zone/binding\r\n");
  ZDT_Console_Print("-- tuning (stored in W25Q64 by save) --\r\n");
  ZDT_Console_Print("  nav                 print spd / t1000 / cnt\r\n");
  ZDT_Console_Print("  spd0.35             wheel rpm = value x 250 (factory 0.350)\r\n");
  ZDT_Console_Print("  t10001930           ms per 1000 mm at reference spd=0.50\r\n");
  ZDT_Console_Print("  dcomp0.10           timed distance D*(1+0.10); range -0.30~0.30; save\r\n");
  ZDT_Console_Print("  endturn             disabled; stop before changing heading\r\n");
  ZDT_Console_Print("  cnt65536            encoder counts per wheel rev\r\n");
  ZDT_Console_Print("  t901700 / t1802800 ms per 90 / 180 deg turn (A* route cost)\r\n");
  ZDT_Console_Print("  tarc0 / tarc1000    90deg ARC cornering: target ms, 0=off (short=tight)\r\n");
  ZDT_Console_Print("  aspd1.00            arc body speed = spd x this ratio\r\n");
  ZDT_Console_Print("  outerarc on|off|status   RAM outer90 corner trial; radius150 / speed<=half straight (mm,mm/s)\r\n");
  ZDT_Console_Print("  navreset            restore default nav params\r\n");
  ZDT_Console_Print("  kp@1ki@0kd@0        SAVE pid gains (also &kp0.045 &ki0.0005 &kd0.0035)\r\n");
  ZDT_Console_Print("  show                load pid from flash and show on IPS200\r\n");
  ZDT_Console_Print("  save / read         store/load pid + nav params (+ &save / &read)\r\n");
  ZDT_Console_Print("  posmode 0/1         segment drive 0=velocity+timed (default) 1=0xFD\r\n");
  ZDT_Console_Print("-- diagnostics --\r\n");
  ZDT_Console_Print("  yaw           HWT101 raw/zero/gyro/bias/ready/cal/turn\r\n");
  ZDT_Console_Print("  status        CAN status;  ver = motor version;  verall = all 4\r\n");
  ZDT_Console_Print("  help / ?      this text;  verbose / quiet = CAN frame logging\r\n");
  ZDT_Console_Print("  zero / &zero  re-arm heading zero (keep still ~3s afterwards)\r\n");
  ZDT_Console_Print("  posq          dump raw 0x36 position replies of all 4 wheels\r\n");
  ZDT_Console_Print("  posref/posd   snapshot positions, then print delta (counts/rev)\r\n");
  ZDT_Console_Print("                (send &dis first: enabled motors hold the shaft)\r\n");
  ZDT_Console_Print("  poscal        per-wheel calib: pulses/rev, 0xFD support, sign\r\n");
  ZDT_Console_Print("  run <rpm> / test <id> <rpm> / zpos <id> <pulses> <rpm>  raw tests\r\n");
  ZDT_Console_Print("  &echo         link check: prints [ECHO] n=<seq>; repeated n = the\r\n");
  ZDT_Console_Print("                receiver duplicated the line, rising n = the command\r\n");
  ZDT_Console_Print("                really was sent several times (check the app).\r\n");
  ZDT_Console_Print("  dm help       Damiao turntable joint (FDCAN1) commands; start\r\n");
  ZDT_Console_Print("                with 'dm probe' -- it listens on the bus and prints\r\n");
  ZDT_Console_Print("                every frame, so a wrong ID or dead wiring is obvious.\r\n");
  ZDT_Console_Print("  delete / flash  erase pid sector / W25Q64 diagnostics\r\n");
  ZDT_Console_Print("-- notes --\r\n");
  ZDT_Console_Print("  - place the car in a start zone, then &startpose 1 (or 2): it sets the\r\n");
  ZDT_Console_Print("    pose, the nose-to-map binding and re-arms the gyro zero drift.\r\n");
  ZDT_Console_Print("  - turns are refused until the zero-drift baseline exists\r\n");
  ZDT_Console_Print("    (screen shows IMU ZERO CAL x.xs then IMU ZERO DONE).\r\n");
  ZDT_Console_Print("  - every turn prints one health line: err / overshoot / peak_o plus\r\n");
  ZDT_Console_Print("    FAST (fast profile) FB (settle band) ST (no progress) OVER-3s.\r\n");
}

static void ZDT_Print_Frame(const FDCAN_RxHeaderTypeDef *header,
                            const uint8_t *data)
{
  if ((zdt_verbose_output == 0U) && (zdt_pos_dump_frames == 0U))
  {
    return;
  }
  char line[96];
  char dump[128];
  uint8_t index;
  uint8_t data_length = ZDT_CAN_DataLengthBytes(header->DataLength);
  int length;

  length = snprintf(line, sizeof(line), "[ZDT] RX %s=0x%08lX DLC=%u DATA=",
                    (header->IdType == FDCAN_EXTENDED_ID) ? "EID" : "SID",
                    (unsigned long)header->Identifier,
                    (unsigned int)data_length);
  for (index = 0U; (index < data_length) && (length > 0) &&
       ((size_t)length < sizeof(line)); ++index)
  {
    length += snprintf(&line[length], sizeof(line) - (size_t)length,
                       "%02X ", data[index]);
  }
  if ((length > 0) && ((size_t)length < sizeof(line) - 2U))
  {
    (void)snprintf(&line[length], sizeof(line) - (size_t)length, "\r\n");
  }
  else
  {
    line[sizeof(line) - 3U] = '\r';
    line[sizeof(line) - 2U] = '\n';
    line[sizeof(line) - 1U] = '\0';
  }
  ZDT_Console_Print(line);

  /* posq 诊断时额外解释一帧 0x36（电机实时位置）：
     手册格式为 符号(1B) + 位置(4B)，Emm 固件 65536=1圈，X 固件 3600=1圈。 */
  if ((zdt_pos_dump_frames != 0U) && (data_length >= 7U) && (data[0] == 0x36U))
  {
    uint32_t magnitude = ((uint32_t)data[2] << 24U) |
                         ((uint32_t)data[3] << 16U) |
                         ((uint32_t)data[4] << 8U) |
                         (uint32_t)data[5];
    int32_t signed_position = (data[1] != 0U) ? (int32_t)(0U - magnitude)
                                              : (int32_t)magnitude;

    (void)snprintf(dump, sizeof(dump),
                   "[ZDT]   0x36 addr=%u sign=%u raw=%lu pos=%ld revEmm=%.4f degX=%.1f\r\n",
                   (unsigned int)(header->Identifier >> 8U),
                   (unsigned int)data[1], (unsigned long)magnitude,
                   (long)signed_position,
                   (double)signed_position / 65536.0,
                   (double)signed_position / 10.0);
    ZDT_Console_Print(dump);
  }
}

static HAL_StatusTypeDef ZDT_CAN_SendPacketToAddress(uint8_t address,
                                                     uint8_t packet_index,
                                                     const uint8_t *data,
                                                     uint8_t data_length,
                                                     const char *name,
                                                     uint8_t log_enabled)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};
  static const uint32_t dlc_table[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
  };
  HAL_StatusTypeDef result;
  uint32_t extended_id = (((uint32_t)address & 0xFFU) << 8U) |
                         ((uint32_t)packet_index & 0xFFU);
  char line[112];
  uint8_t index;
  int length;

  if ((zdt_status.can_ready == 0U) || (data == NULL) || (data_length > 8U))
  {
    if (log_enabled != 0U)
    {
      ZDT_Console_Print("[ZDT] ERROR: FDCAN is not ready or payload is invalid\r\n");
    }
    return HAL_ERROR;
  }

  tx_header.Identifier = extended_id;
  tx_header.IdType = FDCAN_EXTENDED_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = dlc_table[data_length];
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  {
    /* Bounded RAM-register transaction: serialize CAN2 FIFO writes against
     * the higher-priority arm task, without sleeping or holding across logs. */
    uint32_t key=__get_PRIMASK();__disable_irq();
    result = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header,(uint8_t *)(uintptr_t)data);
    if(result!=HAL_OK)zdt_can_enqueue_failures++;
    __set_PRIMASK(key);
  }
  if (result == HAL_OK)
  {
    zdt_status.tx_count++;
    if (log_enabled != 0U)
    {
      length = snprintf(line, sizeof(line),
                        "[ZDT] TX %s: EID=0x%08lX DLC=%u DATA=", name,
                        (unsigned long)extended_id,
                        (unsigned int)data_length);
      for (index = 0U; (index < data_length) && (length > 0) &&
           ((size_t)length < sizeof(line)); ++index)
      {
        length += snprintf(&line[length], sizeof(line) - (size_t)length,
                           "%02X ", data[index]);
      }
      if ((length > 0) && ((size_t)length < sizeof(line) - 2U))
      {
        (void)snprintf(&line[length], sizeof(line) - (size_t)length, "\r\n");
      }
      ZDT_Console_Print(line);
    }
  }
  else if (log_enabled != 0U)
  {
    (void)snprintf(line, sizeof(line),
                   "[ZDT] TX %s enqueue failed: HAL=%d CAN_ERR=0x%08lX\r\n",
                   name, (int)result,
                   (unsigned long)HAL_FDCAN_GetError(&hfdcan2));
    ZDT_Console_Print(line);
  }
  return result;
}

static HAL_StatusTypeDef ZDT_CAN_SendToAddress(uint8_t address,
                                                const uint8_t *data,
                                                uint8_t data_length,
                                                const char *name,
                                                uint8_t log_enabled)
{
  return ZDT_CAN_SendPacketToAddress(address, ZDT_CAN_PACKET_INDEX, data,
                                     data_length, name, log_enabled);
}

static HAL_StatusTypeDef ZDT_CAN_Send(const uint8_t *data, uint8_t data_length,
                                      const char *name)
{
  return ZDT_CAN_SendToAddress(ZDT_CAN_MOTOR_ADDRESS, data, data_length,
                               name, 1U);
}
static uint8_t ZDT_UserQuery_Send(uint8_t address,uint8_t command)
{
  const uint8_t data[2]={command,0x6BU};
  return (uint8_t)(ZDT_CAN_SendToAddress(address,data,2U,"motor query",0U)==HAL_OK);
}
static void ZDT_UserQuery_Report(const zdt_query_t *q,const char *reason)
{
  char line[220];uint32_t now=HAL_GetTick();zdt_user_query_address=0U;
  if(q->mask==3U)
    (void)snprintf(line,sizeof(line),"[ZDT] id=%u angle_deg=%+.3f circle_deg=%.3f encoder_count=%lld input_pulses=%lld pos_age=%lums pulse_age=%lums\r\n",
      q->address,ZDTQuery_AngleDeg(q),ZDTQuery_CircleDeg(q),(long long)q->position_counts,(long long)q->input_pulses,
      (unsigned long)(now-q->position_tick),(unsigned long)(now-q->pulse_tick));
  else if(q->mask&ZDT_QUERY_POSITION)
    (void)snprintf(line,sizeof(line),"[ZDT] id=%u angle_deg=%+.3f encoder_count=%lld input_pulses=UNKNOWN (%s)\r\n",
      q->address,ZDTQuery_AngleDeg(q),(long long)q->position_counts,reason?reason:"missing pulse reply");
  else if(q->mask&ZDT_QUERY_PULSES)
    (void)snprintf(line,sizeof(line),"[ZDT] id=%u angle_deg=UNKNOWN input_pulses=%lld (%s)\r\n",
      q->address,(long long)q->input_pulses,reason?reason:"missing position reply");
  else (void)snprintf(line,sizeof(line),"[ZDT] id=%u query FAILED mask=%u (%s); no current angle\r\n",
      q->address,q->mask,reason?reason:"missing position reply");
  ZDT_Console_Print(line);
}
static uint8_t ZDT_UserQuery_Command(const char *command)
{
  unsigned int address;char extra;FDCAN_ProtocolStatusTypeDef protocol={0};
  if(strncmp(command,"zdt",3U)!=0 || (command[3] && command[3]!=' '))return 0U;
  if(sscanf(command,"zdt query %u %c",&address,&extra)!=1 || address<1U || address>6U) {
    ZDT_Console_Print("[ZDT] zdt query <1..6>: actual angle(36) + input pulse count(32)\r\n");return 1U;
  }
  if(ZDT_Align_DiagBusy() || zdt_mileage_query_active || zdt_user_query.running) {
    ZDT_Console_Print("[ZDT] query requires idle chassis; finish current motion/query first.\r\n");return 1U;
  }
  if(!zdt_status.can_ready || HAL_FDCAN_GetProtocolStatus(&hfdcan2,&protocol)!=HAL_OK || protocol.BusOff) {
    ZDT_Console_Print("[ZDT] query refused: CAN2 unavailable/bus-off.\r\n");return 1U;
  }
  {const zdt_query_ports_t ports={ZDT_UserQuery_Send,ZDT_UserQuery_Report};
    zdt_user_query_address=(uint8_t)address;
    (void)ZDTQuery_Start(&zdt_user_query,&ports,(uint8_t)address,HAL_GetTick(),zdt_rx_sequence);
  }
  return 1U;
}
static void ZDT_CAN_SendVersionQuery(void)
{
  const uint8_t data[2] = {ZDT_CAN_CMD_READ_VERSION, ZDT_CAN_CHECKSUM};

  zdt_last_query_tick = HAL_GetTick();
  if (zdt_query_attempts < ZDT_CAN_QUERY_MAX_ATTEMPTS)
  {
    zdt_query_attempts++;
  }
  (void)ZDT_CAN_Send(data, 2U, "version query");
}

static void ZDT_CAN_SendAllVersionQueries(void)
{
  const uint8_t data[2] = {ZDT_CAN_CMD_READ_VERSION, ZDT_CAN_CHECKSUM};
  uint8_t i;

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    (void)ZDT_CAN_SendToAddress(zdt_mileage_addresses[i], data, 2U,
                                "version query", 0U);
  }
  ZDT_Console_Print("[ZDT] verall: version query sent to addrs 1/3/2/4; replies below\r\n");
}

static uint8_t ZDT_Mileage_AddressIndex(uint8_t address, uint8_t *index)
{
  uint8_t i;

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    if (zdt_mileage_addresses[i] == address)
    {
      *index = i;
      return 1U;
    }
  }
  return 0U;
}

static int32_t ZDT_Mileage_DecodeSignedPosition(const uint8_t *data)
{
  uint32_t magnitude = ((uint32_t)data[2] << 24U) |
                       ((uint32_t)data[3] << 16U) |
                       ((uint32_t)data[4] << 8U) |
                       (uint32_t)data[5];

  if (data[1] != 0U)
  {
    return (int32_t)(0U - magnitude);
  }
  return (int32_t)magnitude;
}

static void ZDT_Mileage_PrintResults(uint8_t report)
{
  double rev[ZDT_MILEAGE_MOTOR_COUNT] = {0.0};
  int64_t delta[ZDT_MILEAGE_MOTOR_COUNT] = {0};
  char value[ZDT_MILEAGE_MOTOR_COUNT][24];
  double average = 0.0;
  double distance_mm = 0.0;
  int64_t average_counts = 0;
  uint8_t valid_count = 0U;
  char line[192];
  uint8_t i;

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    uint8_t mask = (uint8_t)(1U << i);

    if (((zdt_mileage_query_mask & mask) != 0U) &&
        ((zdt_mileage_reference_mask & mask) != 0U))
    {
      delta[i] = (int64_t)zdt_mileage_query_position[i] -
                 (int64_t)zdt_mileage_reference[i];
      rev[i] = ((double)delta[i] * zdt_mileage_forward_signs[i]) /
               Nav_Params()->counts_per_rev;
      average += rev[i];
      average_counts += (int64_t)(((double)delta[i]) *
                                  zdt_mileage_forward_signs[i]);
      valid_count++;
      (void)snprintf(value[i], sizeof(value[i]), "%+.4frev", rev[i]);
    }
    else
    {
      (void)snprintf(value[i], sizeof(value[i]), "NA");
    }
  }

  if (valid_count != 0U)
  {
    average /= (double)valid_count;
    average_counts /= (int64_t)valid_count;
    distance_mm = average * ZDT_MILEAGE_MM_PER_REV;
  }

  if (report == 0U)
  {
    return;
  }

  (void)snprintf(line, sizeof(line),
                 "[MILEAGE] FL=%s FR=%s\r\n", value[0], value[1]);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[MILEAGE] RL=%s RR=%s\r\n", value[2], value[3]);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[MILEAGE] AVG(%u/4)=%+.4frev CNT=%ld\r\n",
                 (unsigned int)valid_count, average, (long)average_counts);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[MILEAGE] DIST=%+.1fmm (%+.3fm)\r\n",
                 distance_mm, distance_mm / 1000.0);
  ZDT_Console_Print(line);
}

static void ZDT_Mileage_StartPendingForward(void)
{
  if (zdt_mileage_forward_pending != 0U)
  {
    zdt_mileage_forward_pending = 0U;
    (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_FORWARD);
  }
}

static void ZDT_Mileage_FinishQuery(void)
{
  uint8_t received_mask = zdt_mileage_query_mask;
  uint8_t first_reference = (uint8_t)(zdt_mileage_reference_mask == 0U);
  uint8_t i;

  if(ZDT_Vobs_OnMileage(received_mask))return;
  if(ZDT_Odo_OnMileage(received_mask))return;
  ZDT_Match_OnMileage(received_mask);

  if (received_mask == 0U)
  {
    ZDT_Nav_OnQueryFailed();
    ZDT_Console_Print("[MILEAGE] no motor position response.\r\n");
    zdt_mileage_query_active = 0U;
    zdt_mileage_query_report = 0U;
    ZDT_Mileage_StartPendingForward();
    return;
  }

  /* 先通知导航回调保存起点/终点位置，再更新普通里程基准。 */
  ZDT_Nav_OnQueryComplete(received_mask);

  /* poscal 自标定：每次查询完成时取数（在里程基准刷新之前）。 */
  ZDT_PosCal_OnQueryComplete(received_mask);

  /* 标定辅助：posref 抓基准，posd 打印与基准的差值（手转一圈即可算出计数/圈）。 */
  if (zdt_pos_ref_request != 0U)
  {
    char pos_line[96];
    zdt_pos_ref_request = 0U;
    memcpy(zdt_pos_ref, zdt_mileage_query_position, sizeof(zdt_pos_ref));
    zdt_pos_ref_mask = received_mask;
    zdt_pos_ref_set = 1U;
    (void)snprintf(pos_line, sizeof(pos_line), "[ZDT] posref set (mask=0x%X).\r\n",
                   (unsigned int)received_mask);
    ZDT_Console_Print(pos_line);
  }
  else if (zdt_pos_delta_request != 0U)
  {
    zdt_pos_delta_request = 0U;
    if (zdt_pos_ref_set == 0U)
    {
      ZDT_Console_Print("[ZDT] no posref yet; send posref first.\r\n");
    }
    else
    {
      for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
      {
        uint8_t mask = (uint8_t)(1U << i);
        if (((received_mask & mask) != 0U) && ((zdt_pos_ref_mask & mask) != 0U))
        {
          char pos_line[112];
          long cnt = (long)((int64_t)zdt_mileage_query_position[i] -
                            (int64_t)zdt_pos_ref[i]);
          double scale = (double)Nav_Params()->counts_per_rev;

          /* 直接给出"按当前刻度算的圈数"，并保留几个候选刻度的换算做对照。 */
          (void)snprintf(pos_line, sizeof(pos_line),
                         "[ZDT] d%d cnt=%ld rev=%.4f (cnt=%.0f) | /3200=%.3f /3600=%.3f\r\n",
                         (int)zdt_mileage_addresses[i], cnt,
                         (scale > 0.0) ? ((double)cnt / scale) : 0.0, scale,
                         (double)cnt / 3200.0, (double)cnt / 3600.0);
          ZDT_Console_Print(pos_line);
        }
      }
    }
  }

  if (zdt_mileage_query_report != 0U)
  {
    ZDT_Mileage_PrintResults(1U);
  }




  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    uint8_t mask = (uint8_t)(1U << i);
    if ((received_mask & mask) != 0U)
    {
      zdt_mileage_reference[i] = zdt_mileage_query_position[i];
      zdt_mileage_reference_mask |= mask;
    }
  }

  if ((first_reference != 0U) &&
      (zdt_mileage_reference_announced == 0U) &&
      (zdt_mileage_reference_mask != 0U))
  {
    char line[80];
    (void)snprintf(line, sizeof(line),
                   "[MILEAGE] reference ready (%u/4).\r\n",
                   (unsigned int)__builtin_popcount((unsigned int)zdt_mileage_reference_mask));
    ZDT_Console_Print(line);
    zdt_mileage_reference_announced = 1U;
  }

  if (received_mask != ((1U << ZDT_MILEAGE_MOTOR_COUNT) - 1U))
  {
    char line[80];
    (void)snprintf(line, sizeof(line),
                   "[MILEAGE] partial position response: 0x%X/0xF.\r\n",
                   (unsigned int)received_mask);
    ZDT_Console_Print(line);
  }

  zdt_mileage_query_active = 0U;
  zdt_mileage_query_report = 0U;
  zdt_mileage_query_attempts = 0U;
  ZDT_Mileage_StartPendingForward();
}

static void ZDT_Mileage_SendQueries(uint8_t missing_only)
{
  const uint8_t data[2] = {0x36U, ZDT_CAN_CHECKSUM};
  uint8_t i;

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    uint8_t mask = (uint8_t)(1U << i);
    if ((missing_only == 0U) ||
        ((zdt_mileage_query_mask & mask) == 0U))
    {
      (void)ZDT_CAN_SendToAddress(zdt_mileage_addresses[i], data, 2U,
                                  "position query", 0U);
    }
  }
}

static void ZDT_Mileage_StartQuery(uint8_t report)
{
  uint32_t now = HAL_GetTick();

  if (zdt_mileage_query_active != 0U)
  {
    if (report != 0U)
    {
      zdt_mileage_query_report = 1U;
    }
    return;
  }

  zdt_mileage_query_mask = 0U;
  zdt_mileage_query_report = (report != 0U) ? 1U : 0U;
  zdt_mileage_query_active = 1U;
  zdt_mileage_query_attempts = 1U;
  zdt_mileage_query_tick = now;
  zdt_mileage_query_baseline_seq=zdt_rx_sequence;
  zdt_mileage_query_sample_tick=now;
  zdt_mileage_query_retry_tick = now + ZDT_MILEAGE_QUERY_RETRY_MS;
  memset(zdt_mileage_query_position, 0, sizeof(zdt_mileage_query_position));
  ZDT_Mileage_SendQueries(0U);
}

/* ---------- poscal：逐轮脉冲当量自标定 ---------- */

static void ZDT_PosCal_Report(uint8_t index, uint8_t responded)
{
  static const char *const names[ZDT_MILEAGE_MOTOR_COUNT] =
      {"FL", "FR", "RL", "RR"};
  char line[144];

  if (responded == 0U)
  {
    (void)snprintf(line, sizeof(line), "[ZDT] poscal %s d%d: no 0x36 response\r\n",
                   names[index], (int)zdt_mileage_addresses[index]);
    ZDT_Console_Print(line);
    return;
  }

  {
    int32_t delta = (int32_t)((int64_t)zdt_poscal_after -
                              (int64_t)zdt_poscal_before);

    if (delta == 0)
    {
      (void)snprintf(line, sizeof(line),
                     "[ZDT] poscal %s d%d: NO MOVEMENT (0xFD not executed)\r\n",
                     names[index], (int)zdt_mileage_addresses[index]);
    }
    else
    {
      double magnitude = (delta < 0) ? -(double)delta : (double)delta;
      double ppr = ((double)ZDT_POSCAL_MOVE_PULSES *
                    (double)Nav_Params()->counts_per_rev) / magnitude;

      /* 指令侧固定下发 +ZDT_POSCAL_MOVE_PULSES（驱动器口径为 CW）；
         回读为正说明 0x36 的符号与指令方向同号，为负则相反。 */
      (void)snprintf(line, sizeof(line),
                     "[ZDT] poscal %s d%d: cnt=%+ld rev=%+.4f ppr=%.0f p/rev read-sign=%s\r\n",
                     names[index], (int)zdt_mileage_addresses[index], (long)delta,
                     (double)delta / 65536.0, ppr,
                     (delta > 0) ? "same" : "inverted");
      zdt_poscal_ppr[index] = (float)ppr;
      zdt_poscal_moved[index] = 1U;
    }
    ZDT_Console_Print(line);
  }
}

static void ZDT_PosCal_NextWheel(void)
{
  zdt_poscal_index++;
  if (zdt_poscal_index >= ZDT_MILEAGE_MOTOR_COUNT)
  {
    char line[176];
    uint8_t moved = 0U;
    uint8_t i;

    for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
    {
      if (zdt_poscal_moved[i] != 0U)
      {
        moved++;
      }
    }

    (void)snprintf(line, sizeof(line),
                   "[ZDT] poscal SUMMARY: FL=%.0f FR=%.0f RL=%.0f RR=%.0f p/rev (%u/4 moved)\r\n",
                   (double)zdt_poscal_ppr[0], (double)zdt_poscal_ppr[1],
                   (double)zdt_poscal_ppr[2], (double)zdt_poscal_ppr[3],
                   (unsigned int)moved);
    ZDT_Console_Print(line);

    if (moved == ZDT_MILEAGE_MOTOR_COUNT)
    {
      ZDT_Console_Print("[ZDT] poscal: all four moved -> set ZDT_POSITION_PULSES_PER_REV to that value.\r\n");
    }
    else
    {
      ZDT_Console_Print("[ZDT] poscal: some wheel did not move -> that driver does not\r\n");
      ZDT_Console_Print("             execute 0xFD; check its firmware/params/wiring.\r\n");
    }

    zdt_poscal_active = 0U;
    zdt_poscal_phase = ZDT_POSCAL_IDLE;
    return;
  }

  zdt_poscal_phase = ZDT_POSCAL_WAIT_BASE;
  zdt_poscal_tick = HAL_GetTick() + ZDT_POSCAL_GAP_MS;
  zdt_poscal_wheel_tick = HAL_GetTick();
}

static void ZDT_PosCal_OnQueryComplete(uint8_t mask)
{
  uint8_t responded;
  uint8_t bit;

  if ((zdt_poscal_active == 0U) ||
      (zdt_poscal_index >= ZDT_MILEAGE_MOTOR_COUNT))
  {
    return;
  }

  bit = (uint8_t)(1U << zdt_poscal_index);
  responded = (uint8_t)((mask & bit) != 0U);

  if (zdt_poscal_phase == ZDT_POSCAL_WAIT_BASE)
  {
    if (responded == 0U)
    {
      ZDT_PosCal_Report(zdt_poscal_index, 0U);
      ZDT_PosCal_NextWheel();
      return;
    }

    zdt_poscal_before = zdt_mileage_query_position[zdt_poscal_index];
    /* mode=0x02（相对当前位置）、非缓存（立即执行）：单轮小角度正转。
       不需要同步触发，也不会与速度模式互相覆盖（本轮没有速度指令）。 */
    if (ZDT_Motor_MovePositionPulsesAddr(zdt_mileage_addresses[zdt_poscal_index],
                                         ZDT_POSCAL_MOVE_PULSES,
                                         ZDT_POSCAL_SPEED_RPM,
                                         ZDT_NAV_POSITION_ACCELERATION, 0U) != HAL_OK)
    {
      ZDT_Console_Print("[ZDT] poscal: position command rejected\r\n");
      ZDT_PosCal_NextWheel();
      return;
    }

    zdt_poscal_phase = ZDT_POSCAL_MOVING;
    zdt_poscal_probes = 0U;
    zdt_poscal_tick = HAL_GetTick() + ZDT_POSCAL_MOVE_WAIT_MS;
    return;
  }

  if (zdt_poscal_phase == ZDT_POSCAL_WAIT_RESULT)
  {
    if (responded == 0U)
    {
      ZDT_PosCal_Report(zdt_poscal_index, 0U);
      ZDT_PosCal_NextWheel();
      return;
    }

    zdt_poscal_after = zdt_mileage_query_position[zdt_poscal_index];
    if (zdt_poscal_probes == 0U)
    {
      /* 首次回读：还不能确定已经停稳，隔一小段再读一次比对。 */
      zdt_poscal_probes = 1U;
      zdt_poscal_sample = zdt_poscal_after;
      zdt_poscal_phase = ZDT_POSCAL_MOVING;
      zdt_poscal_tick = HAL_GetTick() + ZDT_POSCAL_RECHECK_MS;
      return;
    }

    {
      int32_t change = (int32_t)((int64_t)zdt_poscal_after -
                                 (int64_t)zdt_poscal_sample);

      if (change < 0)
      {
        change = -change;
      }
      if ((change > (int32_t)ZDT_POSCAL_STILL_COUNTS) &&
          (zdt_poscal_probes < ZDT_POSCAL_MAX_PROBES))
      {
        zdt_poscal_probes++;
        zdt_poscal_sample = zdt_poscal_after;
        zdt_poscal_phase = ZDT_POSCAL_MOVING;
        zdt_poscal_tick = HAL_GetTick() + ZDT_POSCAL_RECHECK_MS;
        return;
      }
    }

    ZDT_PosCal_Report(zdt_poscal_index, 1U);
    ZDT_PosCal_NextWheel();
  }
}

static void ZDT_PosCal_Service(void)
{
  uint32_t now = HAL_GetTick();

  if (zdt_poscal_active == 0U)
  {
    return;
  }

  if ((uint32_t)(now - zdt_poscal_wheel_tick) >= ZDT_POSCAL_TIMEOUT_MS)
  {
    zdt_poscal_active = 0U;
    zdt_poscal_phase = ZDT_POSCAL_IDLE;
    ZDT_Console_Print("[ZDT] poscal timeout; aborted.\r\n");
    return;
  }

  if (zdt_poscal_phase == ZDT_POSCAL_WAIT_BASE)
  {
    if (((int32_t)(now - zdt_poscal_tick) >= 0) &&
        (zdt_mileage_query_active == 0U))
    {
      ZDT_Mileage_StartQuery(0U);
    }
  }
  else if (zdt_poscal_phase == ZDT_POSCAL_MOVING)
  {
    if (((int32_t)(now - zdt_poscal_tick) >= 0) &&
        (zdt_mileage_query_active == 0U))
    {
      zdt_poscal_phase = ZDT_POSCAL_WAIT_RESULT;
      ZDT_Mileage_StartQuery(0U);
    }
  }
  else
  {
    /* WAIT_RESULT 由 ZDT_PosCal_OnQueryComplete 推进。 */
  }
}

static void ZDT_PosCal_Start(void)
{
  uint8_t i;

  if (zdt_poscal_active != 0U)
  {
    ZDT_Console_Print("[ZDT] poscal already running.\r\n");
    return;
  }
  if (ZDT_Nav_IsActive() != 0U)
  {
    ZDT_Console_Print("[ZDT] poscal refused: navigation is running.\r\n");
    return;
  }

  ZDT_Console_Print("[ZDT] poscal: LIFT the car first (each wheel turns ~1 rev).\r\n");

  /* 清掉速度模式残留指令并确保四轮使能，否则位置指令不会被执行。
     此后速度为 0 且 output_dirty=0，底盘任务不会再发速度帧覆盖位置指令。 */
  (void)ChassisMotor_EmergencyStop();
  if (ChassisMotor_Enable() != HAL_OK)
  {
    ZDT_Console_Print("[ZDT] poscal: enable failed (check CAN wiring).\r\n");
  }

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    zdt_poscal_ppr[i] = 0.0f;
    zdt_poscal_moved[i] = 0U;
  }
  zdt_poscal_index = 0U;
  zdt_poscal_phase = ZDT_POSCAL_WAIT_BASE;
  zdt_poscal_tick = HAL_GetTick() + ZDT_POSCAL_GAP_MS;
  zdt_poscal_wheel_tick = HAL_GetTick();
  zdt_poscal_active = 1U;
  ZDT_Console_Print("[ZDT] poscal started.\r\n");
}

static void ZDT_Mileage_RequestStop(uint8_t report)
{
  zdt_mileage_stop_pending = 1U;
  if (report != 0U)
  {
    zdt_mileage_stop_report = 1U;
  }
}

static void ZDT_Mileage_RequestForward(void)
{
  zdt_mileage_forward_pending = 1U;
  if ((zdt_mileage_query_active == 0U) &&
      (zdt_mileage_stop_pending == 0U))
  {
    ZDT_Mileage_StartQuery(0U);
  }
}

/* 处理“停止后等电机完全静止，再查询最终位置”的流程。 */
static void ZDT_Mileage_Service(void)
{
  uint32_t now = HAL_GetTick();

  if (zdt_mileage_stop_pending != 0U)
  {
    int16_t rpm[ZDT_MILEAGE_MOTOR_COUNT];
    uint8_t stopped = 1U;
    uint8_t i;

    ChassisMotor_GetWheelRpm(rpm);
    for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
    {
      if (rpm[i] != 0)
      {
        stopped = 0U;
        break;
      }
    }

    if (stopped != 0U)
    {
      zdt_mileage_stop_pending = 0U;
      ZDT_Mileage_StartQuery(zdt_mileage_stop_report);
      zdt_mileage_stop_report = 0U;
    }
  }

  if (zdt_mileage_query_active == 0U)
  {
    return;
  }

  if ((zdt_mileage_query_mask != ((1U << ZDT_MILEAGE_MOTOR_COUNT) - 1U)) &&
      (zdt_mileage_query_attempts < ZDT_MILEAGE_QUERY_MAX_ATTEMPTS) &&
      ((int32_t)(now - zdt_mileage_query_retry_tick) >= 0))
  {
    ZDT_Mileage_SendQueries(1U);
    zdt_mileage_query_attempts++;
    zdt_mileage_query_retry_tick = now + ZDT_MILEAGE_QUERY_RETRY_MS;
  }

  if ((now - zdt_mileage_query_tick) >= ZDT_MILEAGE_QUERY_TIMEOUT_MS)
  {
    ZDT_Mileage_FinishQuery();
  }
}

static void ZDT_Mileage_HandleFrame(const FDCAN_RxHeaderTypeDef *header,
                                    const uint8_t *data, uint8_t data_length,uint32_t received_tick,uint32_t received_seq)
{
  uint8_t address;
  uint8_t index;

  if ((zdt_mileage_query_active == 0U) ||
      (header->IdType != FDCAN_EXTENDED_ID) ||
      (header->RxFrameType != FDCAN_DATA_FRAME) ||
      (data_length != 7U) || (data[0] != 0x36U) || data[1]>1U ||
      (int32_t)(received_seq-zdt_mileage_query_baseline_seq)<=0 ||
      (int32_t)(received_tick-zdt_mileage_query_tick)<0 ||
      (data[6] != ZDT_CAN_CHECKSUM))
  {
    return;
  }

  address = (uint8_t)(header->Identifier >> 8U);
  if (ZDT_Mileage_AddressIndex(address, &index) == 0U)
  {
    return;
  }

  if(zdt_mileage_query_mask&(1U<<index))return;
  if(!zdt_mileage_query_mask)zdt_mileage_query_sample_tick=received_tick;
  zdt_mileage_query_position[index] = ZDT_Mileage_DecodeSignedPosition(data);
  zdt_mileage_query_mask |= (uint8_t)(1U << index);
  if (zdt_mileage_query_mask == ((1U << ZDT_MILEAGE_MOTOR_COUNT) - 1U))
  {
    ZDT_Mileage_FinishQuery();
  }
}


static float ZDT_Nav_GetHeading(void)
{
  chassis_telemetry_t telemetry;
  ChassisMotor_GetTelemetry(&telemetry);
  return telemetry.heading_deg;
}

static uint8_t ZDT_Nav_IsActive(void)
{
  return (uint8_t)(zdt_nav_state != ZDT_NAV_IDLE);
}

/* 估算驱动器完成该段所需时间，仅用于等待，不再用于生成速度曲线。 */
static uint32_t ZDT_Nav_PositionTimeMs(float distance_rev, uint16_t speed_rpm)
{
  float speed_rev_s = (float)speed_rpm / 60.0f;
  float time_ms;

  if ((distance_rev <= 0.0f) || (speed_rev_s <= 0.0f))
  {
    return ZDT_NAV_POSITION_MARGIN_MS;
  }
  time_ms = (distance_rev / speed_rev_s) * 1000.0f;
  return (uint32_t)time_ms + ZDT_NAV_POSITION_MARGIN_MS;
}

/* 发车零航向参考（内部坐标系单位向量），不是转弯后的实时车头方向。
 * 地图→实时车体转换由nav_frame叠加当前IMU航向完成。
 *
 * 刚性约束：车体右侧 R 必须是车头 F 顺时针转 90°（俯视）。在内部坐标系里
 * （x = 地图 x，y = 地图 y 取反），这个关系写成 R = (-F.y, F.x)，于是：
 *     vx(前向) = d · F        vy(车体向右) = d · R
 * 现场摆放时车头必须与地图轴对齐，否则走斜线 —— 用 `&nudge u|d|l|r` 自检。
 *
 * 2026-09-13 现场**最终**确认的摆法（与官方规则图一致）：
 *   两个启停区在**右边缘**的右上角与右下角，二维码板在右边缘中部（把两区夹在中间），
 *   原料区在顶边中央。
 *   下面的静态初值 (0,−1) = 地图 **+y**，对应 **1 号区（右下角）**；本队实际发车区是
 *   2 号区（右上角、车头 −y），开机由 ZDT_Script_ApplyDefaultZone() 按
 *   ZDT_START_ZONE_DEFAULT 覆盖成 (0,+1)，`&startpose 1|2` 也随时会按区设好。 */
static int8_t zdt_nav_nose_x = 0;    /* 初值：车头朝地图 +y（1 号区的绑定） */
static int8_t zdt_nav_nose_y = -1;

/* dir: 0=地图 −y(下) 1=地图 +y(上) 2=地图 +x(右) 3=地图 −x(左) */
static void ZDT_Nav_SetNoseDir(uint8_t dir)
{
  switch (dir)
  {
    case 0U: zdt_nav_nose_x = 0;  zdt_nav_nose_y = 1;  break;
    case 1U: zdt_nav_nose_x = 0;  zdt_nav_nose_y = -1; break;
    case 2U: zdt_nav_nose_x = 1;  zdt_nav_nose_y = 0;  break;
    default: zdt_nav_nose_x = -1; zdt_nav_nose_y = 0;  break;
  }
}

/* Initial nose defines gyro zero; actual body axes rotate with current yaw.
 * Grid rows point down, so convert both baseline/segment to map y-up first. */
static void ZDT_Nav_GetBodyVelocity(const grid_nav_segment_t *segment,
                                    float *vx, float *vy)
{
  NavFrame_MapToBody((float)zdt_nav_nose_x, (float)(-zdt_nav_nose_y),
                     ZDT_Nav_GetHeading(), (float)segment->dx,
                     (float)(-segment->dy), vx, vy);
}

static void ZDT_Nav_GetWheelValues(const grid_nav_segment_t *segment,
                                   float values[ZDT_MILEAGE_MOTOR_COUNT])
{
  float vx;
  float vy;

  ZDT_Nav_GetBodyVelocity(segment, &vx, &vy);
  NavFrame_WheelValues(vx, vy, values);
}

/* 把四轮位置增量投影到当前直线段方向，得到实际行驶圈数。
   单位必须是"圈"（回读是编码器计数，Emm 固件 65536 计数 = 1 圈），
   否则实测值会比目标大 65536 倍，矫正量随之放大到跑飞。

   四个投影里任何一个都可能因单轮接线/地址冲突/编码器异常而变成野值，
   取平均值会被它整体拖偏（现象：同一段长度两次测出差别极大的圈数）。
   因此这里用中位数（4 个样本取中间两值平均）：单轮野值不影响结果。
   同时把四轮投影的最大-最小差存起来供调用方打印，用于定位是哪一轮异常。 */
static float ZDT_Nav_MeasureDistance(void)
{
  const grid_nav_segment_t *segment = &zdt_nav_segments[zdt_nav_segment_index];
  float wheel_values[ZDT_MILEAGE_MOTOR_COUNT];
  double wheel_rev[ZDT_MILEAGE_MOTOR_COUNT] = {0.0};
  uint8_t valid_mask=(uint8_t)(zdt_nav_start_mask&zdt_nav_end_mask);
  uint8_t i;

  ZDT_Nav_GetWheelValues(segment, wheel_values);

  /* 速度模式直行按实际请求前进/倒退选择四轮投影符号，
     真正的横移使用交替符号并带左右方向。
     位置模式仍按该段的车体运动分量投影。 */
  if (zdt_nav_odometry_enabled || Nav_Params()->position_mode == 0U)
  {
    /* Straight commands use body-forward; strafe commands use body-right.
     * Using [1,1,1,1] for a strafe cancels its +/- wheel deltas to zero. */
    /* Recenter BACK is progress along the planned segment, so its expected
     * body-forward wheel deltas are NEGATIVE. Preserve sign for wrong-way
     * detection rather than taking the absolute encoder distance. */
    NavFrame_VelocityWheelValues(zdt_nav_recenter_route?zdt_nav_drive_sign:1,
                                 zdt_nav_drive_lateral,wheel_values);
  }
  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    zdt_nav_wheel_rev[i] = 0.0f;
  }
  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    uint8_t mask = (uint8_t)(1U << i);
    if (((zdt_nav_start_mask & mask) != 0U) &&
        ((zdt_nav_end_mask & mask) != 0U) &&
        (fabsf(wheel_values[i]) > 0.1f))
    {
      double delta_rev = ((double)((int64_t)zdt_nav_end_position[i] -
                                   (int64_t)zdt_nav_start_position[i]) *
                          zdt_mileage_forward_signs[i]) /
                         (double)Nav_Params()->counts_per_rev;
      zdt_nav_wheel_rev[i] = (float)delta_rev;
      wheel_rev[i] = delta_rev;
    }
  }

  return NavFeedback_ProjectWheels(wheel_rev,wheel_values,valid_mask,&zdt_nav_wheel_spread_rev);
}

/* 比较两次位置采样，判断是否还有轮子在转（超过门槛即认为未停稳）。 */
static uint8_t ZDT_Nav_PositionsMoving(const int32_t *previous,
                                       const int32_t *current)
{
  uint8_t i;
  double threshold = (MatchControl_IsActive(&zdt_match) != 0U) ?
        (double)NavFeedback_SettleCounts(Nav_Params()->counts_per_rev,
                                         ZDT_MILEAGE_MM_PER_REV) :
        (double)ZDT_NAV_SETTLE_COUNTS;

  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i)
  {
    int64_t delta = (int64_t)current[i] - (int64_t)previous[i];
    if (delta < 0)
    {
      delta = -delta;
    }
    if ((double)delta > threshold)
    {
      return 1U;
    }
  }
  return 0U;
}

/* 连续段速度命令：x 轴平移、y 轴平移，并叠加姿态航向修正。 */
/* 对当前直线段下发四轮“相对位置脉冲 + 速度”，最后广播同步触发。 */
/* 逐轮下发位置指令（即时执行，不用缓存+广播触发）。
 *
 * 为什么不用缓存+触发：实测 `&goto` 时地址 3、2 完全没有动作（`revs FR+0.045
 * RL-0.057`），而 `poscal` 用"单地址、即时"下发时四轮全部动作
 * （见 docs/hardware/底盘方向与麦轮排布说明.md §0.1-14/§8）。
 * 因此导航改为逐轮即时下发、每轮之间留一点间隔，避免 8 个 CAN 包挤在一起。
 * 代价是四轮启动相差几毫秒，对轮式底盘无影响。 */
/* 计算某一轮这段位移要下发的脉冲数（打印与下发共用，保证一致）。 */
static int32_t ZDT_Nav_StepPulses(uint8_t index, const float *wheel_values,
                                  float distance_rev, int8_t sign)
{
  float wheel_sign = (float)zdt_nav_command_signs[index];
  float requested = wheel_values[index] * wheel_sign * (float)sign *
                    Nav_Params_StraightDistance(distance_rev) * ZDT_POSITION_PULSES_PER_REV;

  return (int32_t)((requested >= 0.0f) ? (requested + 0.5f)
                                       : (requested - 0.5f));
}

static HAL_StatusTypeDef ZDT_Nav_SendPositionMoveStep(uint8_t index,
                                                       float distance_rev,
                                                       int8_t sign,
                                                       uint16_t speed_rpm)
{
  const grid_nav_segment_t *segment = &zdt_nav_segments[zdt_nav_segment_index];
  float wheel_values[ZDT_MILEAGE_MOTOR_COUNT];
  int32_t pulses;

  if (index >= ZDT_MILEAGE_MOTOR_COUNT)
  {
    return HAL_ERROR;
  }

  ZDT_Nav_GetWheelValues(segment, wheel_values);
  if (fabsf(wheel_values[index]) < 0.1f)
  {
    return HAL_OK; /* 该轮这一段不参与，无需下发 */
  }

  pulses = ZDT_Nav_StepPulses(index, wheel_values, distance_rev, sign);
  return ZDT_Motor_MovePositionPulsesAddr(zdt_mileage_addresses[index], pulses,
                                          speed_rpm,
                                          ZDT_NAV_POSITION_ACCELERATION, 0U);
}

/* 启动一次段内位移：逐轮下发由 ZDT_NAV_SEND_STEPS 状态推进。
 * 同时打印四轮实际脉冲数——现场据此判断"导航到底命令了什么方向"。 */
static void ZDT_Nav_BeginPositionMove(float distance_rev, int8_t sign,
                                       uint16_t speed_rpm)
{
  const grid_nav_segment_t *segment = &zdt_nav_segments[zdt_nav_segment_index];
  float wheel_values[ZDT_MILEAGE_MOTOR_COUNT];
  char line[176];

  /* 位置模式这一路仍然按段方向走轮子（不做横移），所以清掉横移标记：
     否则上一段若是横移，段末的里程复核/航向体检会被错误跳过。 */
  zdt_nav_drive_lateral = 0;
  ZDT_Nav_GetWheelValues(segment, wheel_values);
  (void)snprintf(line, sizeof(line),
                 "[NAV] move %.3frev sign=%+d pulses FL%+ld FR%+ld RL%+ld RR%+ld\r\n",
                 (double)distance_rev, (int)sign,
                 (long)ZDT_Nav_StepPulses(0U, wheel_values, distance_rev, sign),
                 (long)ZDT_Nav_StepPulses(1U, wheel_values, distance_rev, sign),
                 (long)ZDT_Nav_StepPulses(2U, wheel_values, distance_rev, sign),
                 (long)ZDT_Nav_StepPulses(3U, wheel_values, distance_rev, sign));
  ZDT_Console_Print(line);

  zdt_nav_send_rev = distance_rev;
  zdt_nav_send_sign = sign;
  zdt_nav_send_rpm = speed_rpm;
  zdt_nav_send_index = 0U;
  zdt_nav_send_tick = HAL_GetTick();
  zdt_nav_state = ZDT_NAV_SEND_STEPS;
}

/* ---------- 速度模式 + 定时推算的位移执行 ---------- */

/* ---------- 梯形/三角形速度规划（方案 B，2026-09-12）----------
 * 底盘层对"下发轮速 → 实际轮速"做了斜率限制（见 chassis_motor.c 的
 * ChassisMotor_StepToward）：加速 CHASSIS_ACCEL_RPM_PER_S、减速
 * CHASSIS_DECEL_RPM_PER_S，每 CHASSIS_TX_PERIOD_MS 更新一步。所以任何一段直走的
 * 真实速度曲线都是"加速 - 匀速 - 减速"的梯形；段太短时退化成三角形。
 *
 * 这里按运动学解出"走完 L 毫米需要驱动多少毫秒"，取代原来的"距离 × t1000"线性换算：
 *   v = scale × CHASSIS_MAX_WHEEL_RPM  → mm/s
 *   A = CHASSIS_ACCEL_RPM_PER_S → mm/s²，D = CHASSIS_DECEL_RPM_PER_S → mm/s²
 *   梯形（L ≥ v²/2A + v²/2D）：t = v/A + v/D + (L - v²/2A - v²/2D)/v
 *   三角形（L 更短，到不了 v）：v_peak = √(2L / (1/A + 1/D))，t = v_peak/A + v_peak/D
 *
 * 为什么值得换：
 *   ① 同一个 t1000 在任意段长都成立 —— 斜坡那笔"固定欠走"被显式建模，不再靠标定吸收
 *      （原来是"按 1 格标定则长段偏长约 4%"，段长一变就得重标）；
 *   ② 改 `spd` 之后距离不再跟着变（v 直接进公式，不再需要重标 t1000）；
 *   ③ 段末补走那种几十毫米的小步会走三角形解，不再被 ZDT_NAV_DRIVE_MIN_MS 硬夹。
 *
 * `t1000` 的含义保持不变：**spd = 0.50 时走 1000 mm 需要的毫秒数**。这里把它当
 * "整体时间微调系数"用（k = t1000 / plan_ms(1000mm, spd=0.50)），现场卷尺标定流程
 * 完全不用改；标定一次之后，各段长 / 各 spd 都跟着对。 */
#define ZDT_NAV_MM_S_PER_RPM        (ZDT_MILEAGE_MM_PER_REV / 60.0f)   /* 4.2616 mm/s per RPM */
#define ZDT_NAV_ACCEL_MM_S2         (CHASSIS_ACCEL_RPM_PER_S * ZDT_NAV_MM_S_PER_RPM)
#define ZDT_NAV_DECEL_MM_S2         (CHASSIS_DECEL_RPM_PER_S * ZDT_NAV_MM_S_PER_RPM)
#define ZDT_NAV_REF_SPEED_SCALE     (0.50f)   /* 固定标定基准；运行默认为0.350 */
#define ZDT_NAV_REF_DISTANCE_MM     (1000.0f) /* t1000 的标定距离 */
#define ZDT_NAV_RAW_ROUGH_EXTRA_MM  (0.0f)
#define ZDT_NAV_RAW_APPROACH_EXTRA_MM (0.0f)
#define ZDT_NAV_ZONE1_POINT_EXTRA_MM (0.0f)

/* 走完 distance_mm 需要多少秒（不含任何经验系数）。 */
static float ZDT_Nav_PlanSeconds(float distance_mm, float speed_mm_s)
{
  float accel_mm;
  float decel_mm;

  if (speed_mm_s < 1.0f)
  {
    speed_mm_s = 1.0f;    /* 防 0 除：scale 被限制为 ≥0.02，这里只是保险 */
  }
  if (distance_mm < 0.0f)
  {
    distance_mm = -distance_mm;
  }

  accel_mm = (speed_mm_s * speed_mm_s) / (2.0f * ZDT_NAV_ACCEL_MM_S2);
  decel_mm = (speed_mm_s * speed_mm_s) / (2.0f * ZDT_NAV_DECEL_MM_S2);

  if (distance_mm >= (accel_mm + decel_mm))
  {
    /* 梯形：加速段 + 减速段 + 匀速段 */
    return (speed_mm_s / ZDT_NAV_ACCEL_MM_S2) + (speed_mm_s / ZDT_NAV_DECEL_MM_S2) +
           ((distance_mm - accel_mm - decel_mm) / speed_mm_s);
  }

  {
    /* 三角形：还没到巡航速度就该开始减速 */
    float peak = sqrtf((2.0f * distance_mm) /
                       ((1.0f / ZDT_NAV_ACCEL_MM_S2) + (1.0f / ZDT_NAV_DECEL_MM_S2)));

    return (peak / ZDT_NAV_ACCEL_MM_S2) + (peak / ZDT_NAV_DECEL_MM_S2);
  }
}

/* 参考条件（spd=0.50 走 1000 mm）下的理论驱动时间，单位**毫秒**。
 *
 * ⚠ ZDT_Nav_PlanSeconds() 返回的是**秒**（参考条件下 ≈2.0508），而下游要的是毫秒
 * （日志里的 k 分母、以及没标定时的默认 t1000）。忘了这个 ×1000 的后果不是"少打一位
 * 小数"：k 会打印成 1023.964、理论值打印成 2ms，而且 `navreset` 或 flash 记录失效时
 * 会把 t1000 设成 2.05 ms ⇒ 每段驱动时间都被 ZDT_NAV_DRIVE_MIN_MS 夹住 ⇒ **车几乎
 * 不动**，且全程没有任何报错。换算只在这一个地方做，调用点不许再自己乘。 */
static float ZDT_Nav_ReferencePlanMs(void)
{
  return 1000.0f * ZDT_Nav_PlanSeconds(
      ZDT_NAV_REF_DISTANCE_MM,
      ZDT_NAV_REF_SPEED_SCALE * CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM);
}

/* 距离（圈）+ 速度比例 → 驱动时间（毫秒）。
 * scale ≤ 0 时用导航的 `spd`；段末补走会传 0.5× 的速度比例，这里如实按那个速度算时间
 * （旧版按全速的 t1000 算时间、却用半速执行 → 补走只走了一半距离，要补 2~3 次）。 */
static uint32_t ZDT_Nav_PlanCommandDistanceMs(float command_mm, float scale)
{
  float speed_mm_s;
  float ref_mm_s;
  float ms;

  if (scale <= 0.0f)
  {
    scale = Nav_Params()->speed_scale;
  }

  speed_mm_s = scale * CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM;
  ref_mm_s = ZDT_NAV_REF_SPEED_SCALE * CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM;

  /* k = t1000 / 参考条件下的理论时间（参考条件 = spd 0.50 走 1000 mm） */
  ms = (ZDT_Nav_PlanSeconds(Nav_Params_StraightDistance(command_mm), speed_mm_s) /
        ZDT_Nav_PlanSeconds(1000.0f, ref_mm_s)) * Nav_Params()->time_1000mm_ms;

  if (ms < (float)ZDT_NAV_DRIVE_MIN_MS)
  {
    ms = (float)ZDT_NAV_DRIVE_MIN_MS;
  }
  return (uint32_t)ms;
}

static uint32_t ZDT_Nav_PlanDriveMs(float distance_rev, float scale)
{
  /* Apply dcomp once; reference denominator and nominal goals stay unchanged. */
  return ZDT_Nav_PlanCommandDistanceMs(Nav_Params_CompensatedDistance(
                       distance_rev * ZDT_MILEAGE_MM_PER_REV), scale);
}

static float ZDT_Nav_Zone1PointExtraMm(void)
{
  if (zdt_match.state != MATCH_MOVING || zdt_match.zone != 1U ||
      zdt_match.checkpoint == MATCH_QR ||
      zdt_nav_segment_index >= zdt_nav_segment_count ||
      zdt_nav_segment_index + 1U != zdt_nav_segment_count ||
      Nav_Params()->position_mode != 0U || zdt_nav_arc_armed != 0U) {
    return 0.0f;
  }
  return ZDT_NAV_ZONE1_POINT_EXTRA_MM;
}

static float ZDT_Nav_MatchExtraMm(void)
{
  const grid_nav_segment_t *segment;
  float extra;
  if ((zdt_match.state != MATCH_MOVING) ||
      (zdt_nav_segment_index >= zdt_nav_segment_count) ||
      (Nav_Params()->position_mode != 0U) || (zdt_nav_arc_armed != 0U)) {
    return 0.0f;
  }
  segment = &zdt_nav_segments[zdt_nav_segment_index];
  if(zdt_match_io==MATCH_IO_ZONE2) {
    if(zdt_nav_segment_index+1U!=zdt_nav_segment_count)return 0.0f;
    return (float)zdt_zone2_action.extra_mm + (zdt_zone2_action.final_point?
        (float)Zone2Route_FixedExtraMm(zdt_match.checkpoint):0.0f);
  }
  extra = ZDT_Nav_Zone1PointExtraMm();
  /* Both batches: extend only the final northward approach to raw. */
  if (((zdt_match.checkpoint == MATCH_RAW1) || (zdt_match.checkpoint == MATCH_RAW2)) &&
      ((zdt_nav_segment_index + 1U) == zdt_nav_segment_count) &&
      (segment->dx == 0) && (segment->dy == -1)) {
    extra += ZDT_NAV_RAW_APPROACH_EXTRA_MM;
  }
  /* RAW1/RAW2 -> ROUGH1/ROUGH2: preserve the southward outbound correction. */
  if (((zdt_match.checkpoint == MATCH_ROUGH1) || (zdt_match.checkpoint == MATCH_ROUGH2)) &&
      (zdt_nav_segment_index == 0U) &&
      (segment->dx == 0) && (segment->dy == 1)) {
    extra += ZDT_NAV_RAW_ROUGH_EXTRA_MM;
  }
  return extra;
}

/* 起始车道居中（现场要求"尽可能让车辆行驶在路中间"）：见 ZDT_Nav_ComputeLaneSnap()。
   偏差小于这个值就不动 —— 车道中线落在格边界上，格心天然偏离 25 mm，
   不设死区会为了 ±25 mm 反复横移。 */
#define ZDT_NAV_LANE_SNAP_DEADBAND_MM   (40)

/* 起始车道居中：算出"把车横移到最近车道中线"需要的车体方向与位移。
 * 返回 0 = 不需要动（已经在中线上、或本段本来就是横移、或没有规划）。 */
static uint8_t ZDT_Nav_ComputeLaneSnap(const grid_nav_segment_t *segment,
                                       int8_t *body_sign, uint16_t *mm_out,
                                       int16_t *map_dx, int16_t *map_dy)
{
  uint16_t pose_x = 0U;
  uint16_t pose_y = 0U;
  float vx = 0.0f;
  float vy = 0.0f;
  int32_t delta;          /* 目标中线 − 当前坐标（带符号，对应地图轴） */
  int32_t lateral;
  int32_t target;
  uint8_t along_x;

  if (segment == NULL)
  {
    return 0U;
  }
  ZDT_Nav_GetBodyVelocity(segment, &vx, &vy);
  if (fabsf(vy) > fabsf(vx))
  {
    /* 这一段本身就是横移：它已经在把车挪到别处，不再叠加居中修正。 */
    return 0U;
  }
  along_x = (uint8_t)(segment->dx != 0);

  GridNav_GetPoseMm(&pose_x, &pose_y);
  if (along_x != 0U)
  {
    lateral = (int32_t)pose_y;                 /* 沿 x 走 → 横向是 y */
    target = (int32_t)GridNav_LaneMm(pose_y);
  }
  else
  {
    lateral = (int32_t)pose_x;                 /* 沿 y 走 → 横向是 x */
    target = (int32_t)GridNav_LaneMm(pose_x);
  }
  delta = target - lateral;
  if ((delta > -ZDT_NAV_LANE_SNAP_DEADBAND_MM) &&
      (delta < ZDT_NAV_LANE_SNAP_DEADBAND_MM))
  {
    return 0U;                                 /* 已经在中线上 */
  }

  /* 先把这次横移写进"地图位移"，再定车体方向。 */
  if (along_x != 0U)
  {
    *map_dx = 0;
    *map_dy = (int16_t)delta;      /* 沿 x 走 → 横向是 y */
  }
  else
  {
    *map_dx = (int16_t)delta;      /* 沿 y 走 → 横向是 x */
    *map_dy = 0;
  }

  *body_sign = NavFrame_StrafeSide((float)zdt_nav_nose_x,
                  (float)(-zdt_nav_nose_y), ZDT_Nav_GetHeading(),
                  (float)*map_dx, (float)*map_dy, ZDT_NAV_ALIGN_TOLERANCE_DEG);
  if (*body_sign == 0) { return 0U; } /* Let navigation align before moving. */
  *mm_out = (uint16_t)((delta >= 0) ? delta : -delta);
  return 1U;
}

/* 直线行进：车头已对准该段方向，所以只发 vx（前进/后退），不横移。
 * 速度用 `spd`（段末补走时按 ZDT_NAV_CORRECT_SPEED_RATIO 降速更准）。 */
static uint8_t ZDT_Nav_PlanRecenterStep(float distance_rev,float extra_mm,
                                       float *scale,float *command_mm,uint32_t *ms)
{
  float nominal_mm=Nav_Params_StraightDistance(distance_rev*ZDT_MILEAGE_MM_PER_REV);char line[144];
  if(!zdt_nav_recenter_route || extra_mm!=0.0f || Nav_Params()->position_mode!=0U)return 0U;
  if(!NavShortMove_Plan(nominal_mm,*scale,CHASSIS_MAX_WHEEL_RPM,ZDT_MILEAGE_MM_PER_REV,
      CHASSIS_ACCEL_RPM_PER_S,CHASSIS_DECEL_RPM_PER_S,CHASSIS_TX_PERIOD_MS,scale,ms))return 0U;
  *command_mm=nominal_mm;
  /* Encoder-derived lane offset is a short step, not calibrated transport:
   * no station addition/dcomp/t1000 here; time reserves remaining braking. */
  ChassisMotor_ResetIdleRamp();
  (void)snprintf(line,sizeof(line),"[NAV] POST_ALIGN_STEP_V2 %.1fmm spd=%.3f hold=%lums; brake included\r\n",
    (double)nominal_mm,(double)*scale,(unsigned long)*ms);
  ZDT_Console_Print(line);return 1U;
}
static void ZDT_Nav_BeginStraightDrive(float distance_rev, int8_t sign,
                                        float scale)
{
  uint32_t ms;
  char line[160];
  float command_mm = Nav_Params_CompensatedDistance(distance_rev * ZDT_MILEAGE_MM_PER_REV);
  float extra_mm = (sign > 0) ? ZDT_Nav_MatchExtraMm() : 0.0f;

  if (scale <= 0.0f)
  {
    scale = Nav_Params()->speed_scale;
  }
  if(zdt_nav_odometry_enabled) {
    if(zdt_nav_recenter_route && !zdt_vobs_nav_active && distance_rev*ZDT_MILEAGE_MM_PER_REV<=NAV_SHORT_MOVE_MAX_MM)
      scale=fminf(scale,0.10f);
    zdt_nav_drive_scale=scale;zdt_nav_drive_sign=sign;zdt_nav_drive_lateral=0;
    zdt_nav_hold_peak_error=0.0f;
    if(!ZDT_Odo_Start(1U,distance_rev*ZDT_MILEAGE_MM_PER_REV,scale,sign,0,
       (zdt_nav_recenter_route && sign<0)?zdt_nav_lateral_hold_heading:
       ZDT_Nav_SegmentHeadingDeg(&zdt_nav_segments[zdt_nav_segment_index]),
       zdt_nav_start_position,zdt_nav_start_mask)) {ZDT_Nav_Fail("odometry start rejected");return;}
    zdt_nav_state=ZDT_NAV_DRIVE;return;
  }
  if (extra_mm != 0.0f) {
    command_mm += extra_mm;
    (void)snprintf(line, sizeof(line),
                   "[NAV] %s: fixed adjustment %+.0fmm after dcomp\r\n",
                   MatchControl_CheckpointName(zdt_match.checkpoint),
                   (double)extra_mm);
    ZDT_Console_Print(line);
  }
  if(command_mm<=0.0f){ZDT_Nav_Fail("fixed adjustment removed the entire segment");return;}
  if(!ZDT_Nav_PlanRecenterStep(distance_rev,extra_mm,&scale,&command_mm,&ms))
    ms = ZDT_Nav_PlanCommandDistanceMs(command_mm, scale);
  (void)snprintf(line, sizeof(line),
                 "[NAV] drive %.3frev (%.0fmm) %s spd=%.3f v=%.0fmm/s t=%lums cmd=%.0fmm\r\n",
                 (double)distance_rev,
                 (double)(distance_rev * ZDT_MILEAGE_MM_PER_REV),
                 (sign >= 0) ? "FWD" : "BACK",
                 (double)scale,
                 (double)(scale * CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM),
                 (unsigned long)ms,
                 (double)command_mm);
  ZDT_Console_Print(line);

  if (ChassisMotor_SetVelocity((float)sign * scale, 0.0f, 0.0f) != HAL_OK)
  {
    ZDT_Nav_Fail("velocity command rejected");
    return;
  }
  zdt_nav_drive_scale = scale;
  zdt_nav_drive_sign = sign;
  zdt_nav_drive_lateral = 0;      /* 直走：段后要按航向复核 */
  zdt_nav_hold_tick = HAL_GetTick() + ZDT_NAV_HOLD_PERIOD_MS;
  zdt_nav_hold_peak_error = 0.0f;
  zdt_nav_motion_deadline = HAL_GetTick() + ms;
  /* 中途打印一次"实际生效的轮速"：用于判断距离偏差出在速度侧还是里程回读侧。 */
  zdt_nav_rpm_report_tick = HAL_GetTick() + (ms / 2U);
  zdt_nav_state = ZDT_NAV_DRIVE;
}

/* ---------- 小幅横移（strafe jog）----------
 *
 * 现场要求（2026-09-13）：**小幅度让位不要"转 90°→走一点→转回 90°"**，直接横移过去；
 * 只有真正改变方向（转角、掉头）才保留原地转。
 *
 * 典型场景：车在右边缘环带里走，A* 选的下一格比当前格偏 1 格（50 mm）——
 * 老做法是"转 90°、走 50 mm、再转 90° 回来"，两次原地转 ≈1.5~2 s，还叠加两次
 * ±0.5° 对准残差；横移则完全不转车头，一次带走完。
 *
 * 判据（用与导航同源的映射，不另算角度）：
 *   ① 段长 ≤ ZDT_NAV_STRAFE_MAX_CELLS 格（默认 3 格 = 150 mm）—— 超过就算"改变方向"；
 *   ② 该段在**当前车头**下的车体速度里 x 分量≈0、y 分量明显 → 纯横向。
 * 横移只走时间规划 + 停稳判据，**不做里程复核**：横移时四轮转向是 [1,-1,-1,1]，
 * 直走的 [1,1,1,1] 投影不适用（地图文档早就记过这条）。 */
#define ZDT_NAV_STRAFE_MAX_CELLS    (3U)

static int8_t ZDT_Nav_SegmentStrafeSign(const grid_nav_segment_t *segment)
{
  if ((segment == NULL) || (segment->length > ZDT_NAV_STRAFE_MAX_CELLS))
  {
    return 0;
  }
  return NavFrame_StrafeSide((float)zdt_nav_nose_x,
                  (float)(-zdt_nav_nose_y), ZDT_Nav_GetHeading(),
                  (float)segment->dx, (float)(-segment->dy),
                  ZDT_NAV_ALIGN_TOLERANCE_DEG);
}

/* 横移：车头**不转**，只给 vy。速度与时间规划同直走。 */
static void ZDT_Nav_BeginLateralDrive(float distance_rev, int8_t sign,
                                       float scale)
{
  uint32_t ms;
  char line[160];
  float extra_mm = ZDT_Nav_MatchExtraMm();
  float command_mm = Nav_Params_CompensatedDistance(distance_rev * ZDT_MILEAGE_MM_PER_REV) + extra_mm;

  if (scale <= 0.0f)
  {
    scale = Nav_Params()->speed_scale;
  }
  if(zdt_nav_odometry_enabled) {
    if(zdt_nav_recenter_route && !zdt_vobs_nav_active && distance_rev*ZDT_MILEAGE_MM_PER_REV<=NAV_SHORT_MOVE_MAX_MM)
      scale=fminf(scale,0.10f);
    zdt_nav_drive_scale=scale;zdt_nav_drive_sign=1;zdt_nav_drive_lateral=sign;
    zdt_nav_lateral_hold_heading=ZDT_Nav_GetHeading();zdt_nav_hold_peak_error=0.0f;
    if(!ZDT_Odo_Start(1U,distance_rev*ZDT_MILEAGE_MM_PER_REV,scale,0,sign,
       zdt_nav_lateral_hold_heading,zdt_nav_start_position,zdt_nav_start_mask)) {
      ZDT_Nav_Fail("odometry strafe start rejected");return;
    }
    zdt_nav_state=ZDT_NAV_DRIVE;return;
  }
  if (extra_mm != 0.0f) {
    (void)snprintf(line, sizeof(line),
                   "[NAV] %s: fixed adjustment %+.0fmm after dcomp (strafe)\r\n",
                   MatchControl_CheckpointName(zdt_match.checkpoint), (double)extra_mm);
    ZDT_Console_Print(line);
  }
  if(command_mm<=0.0f){ZDT_Nav_Fail("fixed adjustment removed the entire strafe");return;}
  if(!ZDT_Nav_PlanRecenterStep(distance_rev,extra_mm,&scale,&command_mm,&ms))
    ms = ZDT_Nav_PlanCommandDistanceMs(command_mm, scale);
  (void)snprintf(line, sizeof(line),
                 "[NAV] strafe %.0fmm %s (no turn) yaw=%+.1f spd=%.3f t=%lums cmd=%.0fmm\r\n",
                 (double)(distance_rev * ZDT_MILEAGE_MM_PER_REV),
                 (sign > 0) ? "RIGHT" : "LEFT",
                 (double)ZDT_Nav_GetHeading(), (double)scale, (unsigned long)ms,
                 (double)command_mm);
  ZDT_Console_Print(line);

  if (ChassisMotor_SetVelocity(0.0f, (float)sign * scale, 0.0f) != HAL_OK)
  {
    ZDT_Nav_Fail("strafe command rejected");
    return;
  }
  zdt_nav_drive_scale = scale;
  zdt_nav_drive_sign = 1;          /* 横移不用它算 vx */
  zdt_nav_drive_lateral = sign;
  /* 横移期间要保持的是**当前车头**（车头没转），不是本段的地图航向
     （后者与车头差 90°，照它纠会把车拧过去）。 */
  zdt_nav_lateral_hold_heading = ZDT_Nav_GetHeading();
  zdt_nav_hold_tick = HAL_GetTick() + ZDT_NAV_HOLD_PERIOD_MS;
  zdt_nav_hold_peak_error = 0.0f;
  zdt_nav_motion_deadline = HAL_GetTick() + ms;
  zdt_nav_rpm_report_tick = 0U;    /* 横移的"轮速"没有直走那样好读，不打 */
  zdt_nav_state = ZDT_NAV_DRIVE;
}

/* 直走中的航向保持：用陀螺 yaw 把车头钉在本段应保持的航向上。
 * 现场现象：原地转 90° 后直走，走到最后变成 90.6°，即走行中航向缓慢漂移、
 * 终点方向偏一点。这里按固定周期读取"目标航向 - 当前航向"，用一个受限的
 * omega（= 车身小幅旋转）修回去 —— 只纠航向，不改变行进速度。
 *   omega = clamp(kp_hold * 误差, ±omega_max)，误差小于死区时不修（防抖）。
 * 单位：误差 deg；omega 是 Mix 的归一化量（1.0 = 满量程 omega）。 */
static void ZDT_Nav_HeadingHoldTo(float target)
{
  float error = target - ZDT_Nav_GetHeading();
  float omega;

  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }

  if (fabsf(error) > zdt_nav_hold_peak_error)
  {
    zdt_nav_hold_peak_error = fabsf(error);
  }

  if (fabsf(error) <= ZDT_NAV_HOLD_DEADBAND_DEG)
  {
    omega = 0.0f;
  }
  else
  {
    omega = ZDT_NAV_HOLD_KP * error;
    if (omega > ZDT_NAV_HOLD_OMEGA_MAX)
    {
      omega = ZDT_NAV_HOLD_OMEGA_MAX;
    }
    else if (omega < -ZDT_NAV_HOLD_OMEGA_MAX)
    {
      omega = -ZDT_NAV_HOLD_OMEGA_MAX;
    }
  }

  if (zdt_nav_drive_lateral != 0)
  {
    /* 横移：vx = 0，速度在 vy 上，只补 omega。 */
    (void)ChassisMotor_SetVelocity(0.0f,
                                   (float)zdt_nav_drive_lateral *
                                       zdt_nav_drive_scale,
                                   omega);
  }
  else
  {
    /* 直走：只改航向（vy 仍为 0），vx 保持不变。 */
    (void)ChassisMotor_SetVelocity((float)zdt_nav_drive_sign * zdt_nav_drive_scale,
                                   0.0f, omega);
  }
}

static void ZDT_Nav_HeadingHold(void)
{
  const grid_nav_segment_t *segment = &zdt_nav_segments[zdt_nav_segment_index];

  /* 横移段要保持的是"开始横移时的车头"，不是本段的地图航向（差 90°）。 */
  if (zdt_nav_drive_lateral != 0 || (zdt_nav_recenter_route && zdt_nav_drive_sign<0))
  {
    ZDT_Nav_HeadingHoldTo(zdt_nav_lateral_hold_heading);
    return;
  }
  ZDT_Nav_HeadingHoldTo(ZDT_Nav_SegmentHeadingDeg(segment));
}

/* ---------- 转向调参与波形（V234，上位机“转向PID”页配套）----------
 * &tstat        打一行当前转向/PID/IMU 状态（PC 面板刷新用，不驱动电机）
 * &ttrace 1|0   打开/关闭 [TURNW] 波形采样；只在原地转进行中输出
 * 波形行（默认 40 Hz）：
 *   [TURNW] t=125 tgt=+90.00 act=+12.34 err=+77.66 out=+0.650
 *   t=起转后毫秒；tgt=绝对目标角（左转为正、内部连续绕组，越界不回绕）；
 *   act=当前航向；err=目标−实际；out=PID 归一化输出（1.0=吃满轮速上限）。
 * 默认关闭：比赛流程不输出，避免占 UART4 带宽；由上位机显式打开。 */
#define ZDT_TURN_TRACE_PERIOD_MS (25U)
static uint8_t zdt_turn_trace_enabled;
static uint32_t zdt_turn_trace_tick;

static uint8_t ZDT_Turn_Command(const char *command)
{
  const char *arg;
  chassis_telemetry_t t;
  float kp;
  float ki;
  float kd;
  char line[192];

  if (command == NULL) { return 0U; }
  ChassisMotor_GetTelemetry(&t);
  ChassisMotor_GetTurnGains(&kp, &ki, &kd);

  if (strcmp(command, "tstat") == 0)
  {
    (void)snprintf(line, sizeof(line),
                   "[TURN] status enabled=%u active=%u imu=%u trace=%u rpm=%.0f tgt=%+.2f act=%+.2f "
                   "err=%+.3f out=%+.3f kp=%.4f ki=%.4f kd=%.4f src=%s\r\n",
                   (unsigned int)ChassisMotor_IsEnabled(),
                   (unsigned int)ChassisMotor_IsTurnActive(),
                   (unsigned int)HWT101_IsZeroDriftReady(),
                   (unsigned int)zdt_turn_trace_enabled,
                   (double)ChassisMotor_GetTurnMaxWheelRpm(),
                   (double)ChassisMotor_GetTurnTargetDeg(), (double)t.heading_deg,
                   (double)t.pid_error_deg, (double)t.pid_output,
                   (double)kp, (double)ki, (double)kd, PID_Storage_Source());
    ZDT_Console_Print(line);
    return 1U;
  }

  if (strncmp(command, "ttrace", 6U) != 0) { return 0U; }

  arg = ZDT_SkipSpaces(command + 6U);
  if ((arg[0] == '1') && (arg[1] == '\0'))
  {
    zdt_turn_trace_enabled = 1U;
    zdt_turn_trace_tick = 0U;
    (void)snprintf(line, sizeof(line), "[TURN] trace on period=%ums\r\n",
                   (unsigned int)ZDT_TURN_TRACE_PERIOD_MS);
    ZDT_Console_Print(line);
  }
  else if ((arg[0] == '0') && (arg[1] == '\0'))
  {
    zdt_turn_trace_enabled = 0U;
    ZDT_Console_Print("[TURN] trace off\r\n");
  }
  else
  {
    ZDT_Console_Print("[TURN] usage: &ttrace 1|0 ; &tstat\r\n");
  }
  return 1U;
}

/* ---------- 转向体检：每次原地转结束打一行 ----------
 * 输出形如：`[TURN] done 1680ms target=+90.00 err=+0.008 overshoot=0.014 peak_o=1.00`
 * 行尾标记：
 *   `FAST` = 本次是快速档（导航段前对准，判据 0.5°/200 ms、上限 120 RPM）
 *   `FB`   = 靠稳定带判到位（精确档是 0.05°；带内且角速度达标只需 200 ms）
 *   `ST`   = 靠"无进展兜底"收工（900/400 ms 内误差压不下去，已到机械下限）
 *   `OVER-3s` = 耗时超过 3000 ms
 *   overshoot = 反向误差峰值（过冲），peak_o = PID 输出峰值（1.0 = 吃满速度上限）
 * 每次转向只打一行，流量可忽略。 */

static void ZDT_TurnLog_Service(void)
{
  chassis_telemetry_t t;
  uint32_t now = HAL_GetTick();

  ChassisMotor_GetTelemetry(&t);

  if ((t.turn_active != 0U) && (zdt_turn_was_active == 0U))
  {
    /* 转向开始：重置体检跟踪；波形打开时补一行 start（含绝对目标与档位），
     * 上位机据此清空并开始新一段曲线。 */
    zdt_turn_initial_sign = (t.pid_error_deg >= 0.0f) ? 1.0f : -1.0f;
    zdt_turn_peak_overshoot_deg = 0.0f;
    zdt_turn_peak_output = 0.0f;
    zdt_turn_trace_tick = 0U;
    if (zdt_turn_trace_enabled != 0U)
    {
      char line[112];

      (void)snprintf(line, sizeof(line),
                     "[TURN] start tgt=%+.2f act=%+.2f profile=%s\r\n",
                     (double)ChassisMotor_GetTurnTargetDeg(), (double)t.heading_deg,
                     (t.turn_profile != 0U) ? "FAST" : "PRECISE");
      ZDT_Console_Print(line);
    }
  }
  else if ((t.turn_active == 0U) && (zdt_turn_was_active != 0U))
  {
    /* 转向结束：打一行"耗时/过冲/峰值输出"体检（每次转向一行，量很小）。
       转向时间目标 ≤3 s（3000 ms），超了标 OVER-3s。 */
    char line[128];

    (void)snprintf(line, sizeof(line),
                   "[TURN] done %lums target=%+.2f err=%+.3f overshoot=%.3f peak_o=%.2f%s%s%s%s\r\n",
                   (unsigned long)t.turn_elapsed_ms, (double)t.target_heading_deg,
                   (double)t.turn_final_error_deg, (double)zdt_turn_peak_overshoot_deg,
                   (double)zdt_turn_peak_output,
                   (t.turn_elapsed_ms > 3000UL) ? "  OVER-3s" : "",
                   (t.turn_profile != 0U) ? "  FAST" : "",
                   (t.turn_fallback != 0U) ? "  FB" : "",
                   (t.turn_stall != 0U) ? "  ST" : "");
    ZDT_Console_Print(line);
  }
  else if (t.turn_active != 0U)
  {
    float signed_error = t.pid_error_deg * zdt_turn_initial_sign;
    float magnitude = (t.pid_output >= 0.0f) ? t.pid_output : -t.pid_output;

    if (signed_error < -zdt_turn_peak_overshoot_deg)
    {
      zdt_turn_peak_overshoot_deg = -signed_error;   /* 反向误差 = 过冲 */
    }
    if (magnitude > zdt_turn_peak_output)
    {
      zdt_turn_peak_output = magnitude;
    }
    if ((zdt_turn_trace_enabled != 0U) &&
        ((zdt_turn_trace_tick == 0U) ||
         ((now - zdt_turn_trace_tick) >= ZDT_TURN_TRACE_PERIOD_MS)))
    {
      char line[112];

      zdt_turn_trace_tick = now;
      (void)snprintf(line, sizeof(line),
                     "[TURNW] t=%lu tgt=%+.2f act=%+.2f err=%+.3f out=%+.3f\r\n",
                     (unsigned long)t.turn_elapsed_ms,
                     (double)ChassisMotor_GetTurnTargetDeg(), (double)t.heading_deg,
                     (double)t.pid_error_deg, (double)t.pid_output);
      ZDT_Console_Print(line);
    }
  }
  zdt_turn_was_active = t.turn_active;
}

/* 该段在地图上的方向 -> 车体应当时刻保持的航向（度，左转为正）。
 *
 * 2026-09-13 改为**向量形式**：原来只有"车头朝 ±y"两种绑定（二进制标志），
 * 表达不了现场这种"车头沿 ±x"的摆法（右下角启停区里车头朝 −x/朝左）。
 * 现在只维护一个"车头在**内部坐标系**里的单位向量 F"，映射由点积给出：
 *     R = 车体右侧在内部坐标系里的向量 = (-F.y, F.x)
 *     vx(前向) = d · F          vy(右向) = d · R
 *     heading   = atan2(-(d · R), d · F)    ← 左转为正
 * 这样四种朝向、以及"车头朝哪都能算"，不再是 if/else 分支。
 * 内部坐标系 = 地图坐标系把 y 取反（格子行号向下），F 的取值见 zdt_nav_nose_*。 */
static float ZDT_Nav_SegmentHeadingDeg(const grid_nav_segment_t *segment)
{
  float forward_component = ((float)segment->dx * (float)zdt_nav_nose_x) +
                            ((float)segment->dy * (float)zdt_nav_nose_y);
  float right_component = ((float)segment->dx * (float)(-zdt_nav_nose_y)) +
                          ((float)segment->dy * (float)zdt_nav_nose_x);

  return atan2f(-right_component, forward_component) * 57.29578f;
}

/* 启动一次位移：默认"直走 + 定时"；`posmode 1` 时改用位置模式（0xFD 脉冲）。
 * 注意：速度模式下不再横移——每一段都先原地转到该段方向，再直走。 */
static void ZDT_Nav_StartMove(float distance_rev, int8_t sign,
                              uint16_t speed_rpm, float scale)
{
  if (!zdt_nav_odometry_enabled && Nav_Params()->position_mode != 0U)
  {
    ZDT_Nav_BeginPositionMove(distance_rev, sign, speed_rpm);
  }
  else
  {
    ZDT_Nav_BeginStraightDrive(distance_rev, sign, scale);
  }
}

static void ZDT_Nav_Fail(const char *reason)
{
  ZDT_Odo_Cancel();
  if(zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) {
    (void)ChassisMotor_StopImmediate();
  }
  ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
  zdt_nav_state = ZDT_NAV_IDLE;
  zdt_nav_recenter_route=0U;
  zdt_nav_wait_start_query = 0U;
  zdt_nav_wait_end_query = 0U;
  zdt_nav_start_ready = 0U;
  zdt_nav_end_ready = 0U;
  ZDT_Console_Print("[NAV] FAILED: ");
  ZDT_Console_Print(reason);
  ZDT_Console_Print("\r\n");
  ZDT_Match_Complete(0U, reason);
}

static void ZDT_Nav_Cancel(void)
{
  ZDT_Odo_Cancel();
  zdt_outer_cross_action=zdt_outer_sample_ready=zdt_outer_stage=0U;zdt_outer_settle_tick=0U;
  zdt_nav_recenter_route=zdt_nav_queued_recenter=0U;
  if(zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) {
    (void)ChassisMotor_StopImmediate();
  } else if (zdt_nav_state != ZDT_NAV_IDLE)
  {
    (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
  }
  zdt_nav_state = ZDT_NAV_IDLE;
  zdt_nav_wait_start_query = 0U;
  zdt_nav_wait_end_query = 0U;
  zdt_nav_start_ready = 0U;
  zdt_nav_end_ready = 0U;
  zdt_nav_settle_valid = 0U;
  zdt_nav_settle_announced = 0U;
  zdt_nav_align_done = 0U;
  zdt_nav_drive_lateral = 0;   /* 取消/失败后别把"横移"标记留给下一段 */
  /* 圆弧的两个标记也要清：否则取消/失败后残留的"已武装"会在下一段末尾
     突然减掉一个 R 并拐一个圆弧（车会停在角点之前）。 */
  zdt_nav_arc_armed = 0U;
  zdt_nav_arc_running = 0U;
  zdt_nav_blend_used = 0U;
  zdt_nav_blend_sample_ready = 0U;
  zdt_nav_blend_deferred_input=0U;
}

/* ================= 定点动作脚本（启停区出场 / 回场）=================
 * 规则要求：从启停区**先前进、再平移到大路中央**。这段动作在"格路径之外"，
 * 所以用一个小脚本队列执行 mm 级动作，不掺进 A* 路径。
 *
 * 为什么用平移而不是"转 90°→走→转回"：平移保持车头不动，不引入转角误差；
 * 转两次要 ~2 s（快速档 0.8 s×2）并叠加两次 ±0.5° 对准残差。
 * 平移 125 mm 在 spd=0.5 下约 0.5 s。
 *
 * 几何依据（规则图 mm 坐标，原点左下、y 向上）：
 *   启停区1 中心 (2250,2250)；外环带中心线 x=2125
 *     → 前进 300（落到 y=1950，车尾让开 y=2100）→ 平移 125 → (2125,1950)
 *   启停区2 中心 (2250,150)  → 前进 300（y=450）→ 平移 125 → (2125,450)
 * 启停区1 占掉右环外侧 300 mm，剩下 2400-2100=250 mm < 车宽280 mm，
 * **绕不过去**，所以必须先前进驶出、再平移对到环带中心线 —— 这正是规则的做法。
 *
 * 朝向：启停区1 车头朝场地内 = map −y_up（`&startpose 1` 设 forward_plus_y=1）；
 *       启停区2 车头朝场地内 = map +y_up（`&startpose 2` 设 forward_plus_y=0）。 */
#define ZDT_SCRIPT_MAX_STEPS     (8U)
#define ZDT_SCRIPT_SETTLE_MS     (350U)

/* 脚本步骤的动作类型。0..3 是平移/直走，4 是原地掉头（`&cross` 用）。 */
#define ZDT_SCRIPT_DIR_FWD       (0U)
#define ZDT_SCRIPT_DIR_BACK      (1U)
#define ZDT_SCRIPT_DIR_LEFT      (2U)   /* 车体左移（vy<0） */
#define ZDT_SCRIPT_DIR_RIGHT     (3U)   /* 车体右移（vy>0） */
#define ZDT_SCRIPT_DIR_TURN180   (4U)   /* 原地掉头 180°，无位移 */

typedef struct
{
  uint8_t dir;       /* ZDT_SCRIPT_DIR_*；0..3 用 mm，4 是掉头（mm 忽略） */
  uint16_t mm;
  int16_t map_dx;    /* 该动作在规则图坐标里的位移，用于动作结束后更新位姿 */
  int16_t map_dy;
} zdt_script_step_t;

static zdt_script_step_t zdt_script_queue[ZDT_SCRIPT_MAX_STEPS];
static uint8_t zdt_script_count;
static uint8_t zdt_script_index;
static uint8_t zdt_script_running;
static uint32_t zdt_script_deadline;    /* 0 = 本步还没起 */
static uint8_t zdt_script_settling;
static uint8_t zdt_script_fast_exit;
static uint32_t zdt_script_settle_tick;
static int16_t zdt_script_dx_total;
static int16_t zdt_script_dy_total;
/* `&cross` 专用：整段动作跑完后要切到哪个启停区（0 = 不切，普通 leave/home/nudge）。
   切区包含"按新区设位姿 + 翻转车头绑定 + 重对零漂"，所以必须在这里记下来。 */
static uint8_t zdt_script_end_zone;
/* zdt_script_then_nav 定义在文件前部（蓝牙命令层要用）。 */
/* ★ 默认起点区（**本队实际发车区**）：
 *     1 = 右下角 (2250,150)，车头朝地图 **+y**
 *     2 = 右上角 (2250,2250)，车头朝地图 **−y**   ← 当前默认
 * 现场换发车区**只改这一行**；`&startpose 1|2` 仍然随时可以覆盖它（并且会重对零漂）。
 *
 * 为什么必须有这个默认值：开机时位姿和车头朝向都还没被人确认过，而屏幕上的黄点和
 * `&leave`/`&home` 的**相对动作表**都必须先假定一个区。默认假定错区的后果很严重
 * （日志看着正确、车却半悬在场外，见调试手册 §4.26），所以这个默认值必须等于实际发车区。
 * 它在开机时由 ZDT_Script_ApplyDefaultZone() 装上，但**不**把位姿标记成"已确认"，
 * 屏幕会显示黄色的 `ZONE n?` 提醒"这只是假定值"。 */
#define ZDT_START_ZONE_DEFAULT      (2U)

static uint8_t zdt_start_zone = ZDT_START_ZONE_DEFAULT;  /* 1 = 右下角，2 = 右上角 */
/* 1 = 本次上电已经敲过 &startpose（位姿+朝向绑定+零漂都设好了）。
   没设过就 leave/home 会打警告：朝向绑定还是默认值，摆位不一致时会直接冲出场外。 */
static uint8_t zdt_start_pose_set;

/* 脚本步的航向闭环（现场要求：`&cross` 的 1500 mm 直走不能再自己拐）：
   每步开始时记下当时的车头，之后每 ZDT_NAV_HOLD_PERIOD_MS 纠一次。 */
static float zdt_script_hold_heading;
static uint32_t zdt_script_hold_tick;
static float zdt_script_drive_scale,zdt_script_measured_mm;

static void ZDT_Script_Queue(const zdt_script_step_t *steps, uint8_t count)
{
  if ((steps == NULL) || (count == 0U) || (count > ZDT_SCRIPT_MAX_STEPS))
  {
    return;
  }
  memcpy(zdt_script_queue, steps, (size_t)count * sizeof(steps[0]));
  zdt_script_count = count;
  zdt_script_index = 0U;
  zdt_script_deadline = 0U;
  zdt_script_settling = 0U;
  zdt_script_fast_exit=0U;
  zdt_vobs_resume_script=zdt_vobs_rebase_script=0U;
  zdt_script_dx_total = 0;
  zdt_script_dy_total = 0;
  zdt_script_running = 1U;
  ZDT_Nav_Cancel();
}

/* 朝向自检：沿**地图**方向走一小段，用来验证"车头朝向绑定"和实际摆放是否一致。
 * 为什么需要它：转向都是**相对**的（陀螺只测转了多少），所以"车头朝反了"这件事
 * 陀螺和里程都发现不了 —— 但地图方向的映射会全反，于是第一条指令就把车开出场地。
 * 这个命令用**与导航完全相同**的映射（ZDT_Nav_GetBodyVelocity 同源），所以：
 *   敲 &nudge u 300，车应当朝**场地内侧/上方（原料区那侧）**走 300 mm；
 *   如果它朝反方向走，说明摆放朝向与绑定不一致 → 按提示重摆/重设。 */
static void ZDT_Script_StartNudge(char map_dir, uint16_t mm)
{
  zdt_script_step_t step;
  int16_t map_dx = 0;
  int16_t map_dy = 0;
  float vx;
  float vy;
  char line[160];

  if (mm == 0U)
  {
    mm = 50U;
  }
  if (mm > 1000U)
  {
    mm = 1000U;
  }

  switch (map_dir)
  {
    case 'u': map_dx = 0;  map_dy = (int16_t)mm;  break;   /* 地图 +y（上方） */
    case 'd': map_dx = 0;  map_dy = (int16_t)-mm; break;   /* 地图 -y（下方） */
    case 'r': map_dx = (int16_t)mm;  map_dy = 0;  break;   /* 地图 +x（右方） */
    case 'l': map_dx = (int16_t)-mm; map_dy = 0;  break;   /* 地图 -x（左方） */
    default:
      ZDT_Console_Print("[NAV] usage: &nudge u|d|l|r [mm]  (map frame)\r\n");
      return;
  }

  NavFrame_MapToBody((float)zdt_nav_nose_x, (float)(-zdt_nav_nose_y),
                     ZDT_Nav_GetHeading(), (float)map_dx / (float)mm,
                     (float)map_dy / (float)mm, &vx, &vy);
  if (fminf(fabsf(vx), fabsf(vy)) >
      sinf(ZDT_NAV_ALIGN_TOLERANCE_DEG * 0.017453292f)) {
    ZDT_Console_Print("[NAV] nudge refused: align heading with map axes first.\r\n");
    return;
  }

  step.mm = mm;
  if (vx > 0.5f)       { step.dir = 0U; }   /* FWD */
  else if (vx < -0.5f) { step.dir = 1U; }   /* BACK */
  else if (vy < 0.0f)  { step.dir = 2U; }   /* LEFT */
  else                 { step.dir = 3U; }   /* RIGHT */
  step.map_dx = map_dx;
  step.map_dy = map_dy;

  ZDT_Script_Queue(&step, 1U);

  (void)snprintf(line, sizeof(line),
                 "[NAV] nudge %c %umm: expect the car to move toward map %s; "
                 "body vx=%+.2f vy=%+.2f nose=%s\r\n",
                 map_dir, (unsigned int)mm,
                 (map_dir == 'u') ? "+y (top / raw-material side)" :
                 (map_dir == 'd') ? "-y (bottom / rough-machining side)" :
                 (map_dir == 'r') ? "+x (right / QR side)" :
                                    "-x (left / storage side)",
                 (double)vx, (double)vy,
                 (zdt_nav_nose_x < 0) ? "-x (left)" :
                 (zdt_nav_nose_x > 0) ? "+x (right)" :
                 (zdt_nav_nose_y > 0) ? "-y (bottom)" : "+y (top)");
  ZDT_Console_Print(line);
  ZDT_Console_Print("[NAV] if it moved the OTHER way, placement and nose binding differ:"
                 " re-place the car and run &startpose 1|2\r\n");
}

/* 当前位姿是不是还在**所选启停区**里（300×300 盒子，中心 (2250,150)/(2250,2250)）。
   与 ZDT_Script_RequirePoseNear() 用同一套盒子，只是这个不打印、只回答"在不在"。 */
static uint8_t ZDT_Script_PoseInStartZone(void)
{
  uint16_t px = 0U;
  uint16_t py = 0U;
  uint16_t zone_y = (zdt_start_zone == 1U) ? 150U : 2250U;

  GridNav_GetPoseMm(&px, &py);
  return (uint8_t)((px >= (uint16_t)(2250U - 150U)) &&
                   (px <= (uint16_t)(2250U + 150U)) &&
                   (py >= (uint16_t)(zone_y - 150U)) &&
                   (py <= (uint16_t)(zone_y + 150U)));
}

/* 跑"被推迟的导航"：车在启停区里收到 `&goto` 时，先出场、出场完再调这里。
 * 为什么在这里重新规划：出场把车从区中心挪到了出场落点（mm 位姿已更新），
 * 用旧位姿算出来的路径起点/段长都是错的（就是 §4.19 那次"冲出场地"的坑）。 */
static void ZDT_Nav_StartPending(void)
{
  uint16_t target_x = 0U;
  uint16_t target_y = 0U;

  GridNav_GetTargetMm(&target_x, &target_y);
  ZDT_Console_Print("[NAV] now planning from the exit point\r\n");
  ZDT_Nav_RefreshCostModel();
  if (GridNav_PlanTargetMm(target_x, target_y) != 0U)
  {
    ZDT_Nav_Start();
  }
  else { ZDT_Nav_Fail("no path after script"); }
}

/* 起步前把车横移到车道中线（现场要求"尽可能让车辆行驶在路中间"）。
 * 实现成一个 1 步脚本：复用脚本的横移 + 停稳 + 位姿累计，跑完自动接着跑导航。
 * 返回 1 = 已经排好这一步（调用方不要再直接起导航）。 */
static uint8_t ZDT_Script_StartLaneSnapIfNeeded(void)
{
  grid_nav_segment_t segments[1];
  uint16_t count = 0U;
  zdt_script_step_t step;
  int8_t body_sign = 0;
  uint16_t mm = 0U;
  int16_t map_dx = 0;
  int16_t map_dy = 0;
  char line[128];

  if (GridNav_GetSegments(segments, 1U, &count) == 0U)
  {
    return 0U;
  }
  if (ZDT_Nav_ComputeLaneSnap(&segments[0], &body_sign, &mm,
                              &map_dx, &map_dy) == 0U)
  {
    return 0U;
  }

  step.dir = (body_sign > 0) ? ZDT_SCRIPT_DIR_RIGHT : ZDT_SCRIPT_DIR_LEFT;
  step.mm = mm;
  step.map_dx = map_dx;
  step.map_dy = map_dy;
  (void)snprintf(line, sizeof(line),
                 "[NAV] lane centring: strafe %umm %s to the nearest lane centre\r\n",
                 (unsigned int)mm, (body_sign > 0) ? "RIGHT" : "LEFT");
  ZDT_Console_Print(line);
  zdt_script_then_nav = 1U;
  ZDT_Script_Queue(&step, 1U);
  return 1U;
}

/* 出场/回场前的**位姿合理性闸门**。
 *
 * 为什么必须要有：`&leave` / `&home` 都是**相对动作**（前进 300 + 平移 125），它们只
 * 对"车确实蹲在启停区里"这个前提成立。位姿一旦来自别处（已经出场过一次又要再出场、
 * 位姿是手输的、或区号选错了），同样的相对动作会把车顶到围栏或推出场地，**而日志上
 * 印的落点还是"正确"的** —— 现场就是这么被骗过去的：
 *   `&startpose 2` 被参数解析 bug 当成 1 号区 → 车在 2 号区却按 1 号区算 →
 *   `&leave` 打印 (2125,450) 看着没问题，车实际朝右多平移 250 mm 半悬在场外 →
 *   紧接着 `&goto raw` 直接冲出场地。
 * 所以：宁可**拒绝执行**，并且把"当前位姿"和"期望位姿"一起打出来，让人一眼看出差在哪。 */
static uint8_t ZDT_Script_RequirePoseNear(uint16_t expect_x, uint16_t expect_y,
                                          uint16_t tolerance, const char *what)
{
  uint16_t px = 0U;
  uint16_t py = 0U;
  int32_t dx;
  int32_t dy;
  char line[192];

  GridNav_GetPoseMm(&px, &py);
  dx = (int32_t)px - (int32_t)expect_x;
  dy = (int32_t)py - (int32_t)expect_y;
  if ((dx > -(int32_t)tolerance) && (dx < (int32_t)tolerance) &&
      (dy > -(int32_t)tolerance) && (dy < (int32_t)tolerance))
  {
    return 1U;
  }

  (void)snprintf(line, sizeof(line),
                 "[NAV] REFUSED %s: expected pose near (%u,%u) but it is (%u,%u)."
                 " This action is RELATIVE and would drive off the mat; fix the pose,"
                 " or re-place the car and send &startpose 1|2.\r\n",
                 what,
                 (unsigned int)expect_x, (unsigned int)expect_y,
                 (unsigned int)px, (unsigned int)py);
  ZDT_Console_Print(line);
  return 0U;
}

/* ================= `&cross`：从一个启停区直走到另一个启停区 =================
 *
 * 现场要求（2026-09-13）："从一个启停区直走到另一个启停区"。
 *
 * 几何（两个区都在右边缘、中间夹着二维码板，见地图文档 §6）：
 *   1 号区中心 (2250,150)，2 号区中心 (2250,2250)，右边缘环带车道 x=2125。
 *   所以整段路 = 出场（前进 300 + 平移 125 到车道）→ **沿车道直走 1500 mm**
 *   → 掉头 180° → 进区（平移 125 + 后退 300）。中间那 1500 mm 是**一条直线**，
 *   这也是现场要的"直走"。
 *
 * 为什么必须掉头 180°：两个区要求车头都朝**场地内**（1 号区朝 +y、2 号区朝 −y），
 * 方向正好相反。沿同一条车道直走过去，车头方向不变，于是到对面时车头朝向场外 ——
 * 那样下一次 `&leave` 的"前进"会直接撞墙。掉头放在**车道尽头（出场落点）**做，
 * 那里有空间，而且做完正好用"平移 + 后退"进区。
 *
 * 为什么到位后要调 ZDT_Script_SetStartZone()：它会把当前区号、位姿（精确到区中心）、
 * **车头绑定**一起切过去 —— 掉头后绑定必须翻，否则地图方向映射全反。
 * 它同时重对陀螺零漂，所以到位后要静止约 3 s（屏幕 `IMU ZERO CAL`）。
 *
 * 用法：`&cross`（去另一个区）或 `&cross 1|2`（明确指定目标区；已在目标区则拒绝）。 */
#define ZDT_SCRIPT_LANE_X_MM        (2125)   /* 右边缘环带车道 */
#define ZDT_SCRIPT_ZONE_X_MM        (2250)   /* 启停区中心 x */
#define ZDT_SCRIPT_ZONE1_Y_MM       (150)
#define ZDT_SCRIPT_ZONE2_Y_MM       (2250)
#define ZDT_SCRIPT_LEAVE_MM         (300U)   /* 出场前进 / 进区后退 */
#define ZDT_SCRIPT_STRAFE_MM        (125U)   /* 区中心 x ↔ 车道 x */

static void ZDT_Script_StartCross(uint8_t target_zone)
{
  zdt_script_step_t steps[6];
  uint8_t zone = (target_zone == 2U) ? 2U : 1U;
  uint16_t from_y = (zdt_start_zone == 1U) ? ZDT_SCRIPT_ZONE1_Y_MM
                                          : ZDT_SCRIPT_ZONE2_Y_MM;
  uint16_t to_y = (zone == 1U) ? ZDT_SCRIPT_ZONE1_Y_MM : ZDT_SCRIPT_ZONE2_Y_MM;
  /* ⚠ 直走量算的是**两个出场落点之间**，不是两个区中心之间：
     出场已经朝场地内走了 300，进区还要再走 300，所以车道上的净直走
     = |Δ区中心| − 2×300 = 2100 − 600 = 1500 mm。
     （这里踩过一次：按区中心差写成 2100，车会多冲 600 mm 撞到对面墙。） */
  int32_t from_exit_y = (int32_t)from_y + ((zdt_start_zone == 1U) ? 300 : -300);
  int32_t to_exit_y = (int32_t)to_y - ((zone == 2U) ? 300 : -300);
  int32_t run = to_exit_y - from_exit_y;   /* 车道上的直走量（带符号） */
  uint8_t dir_leave_fwd;
  uint8_t dir_leave_strafe;
  uint8_t dir_run;
  uint8_t dir_enter_strafe;
  uint8_t dir_enter_back;

  /* 车体动作方向由"车头绑定"决定：车头朝地图 +y（1 号区）时前进 = map +y，
     车头朝 −y（2 号区）时前进 = map −y。方向表与 &leave/&home 完全同源。 */
  if (zdt_start_zone == 1U)
  {
    dir_leave_fwd = ZDT_SCRIPT_DIR_FWD;      /* map +y */
    dir_leave_strafe = ZDT_SCRIPT_DIR_LEFT;  /* map −x → 到 x=2125 */
    dir_run = (run >= 0) ? ZDT_SCRIPT_DIR_FWD : ZDT_SCRIPT_DIR_BACK;
  }
  else
  {
    dir_leave_fwd = ZDT_SCRIPT_DIR_FWD;      /* map −y */
    dir_leave_strafe = ZDT_SCRIPT_DIR_RIGHT; /* map −x */
    dir_run = (run <= 0) ? ZDT_SCRIPT_DIR_FWD : ZDT_SCRIPT_DIR_BACK;
  }
  /* 掉头之后车头翻转：进区那两步要按**目标区**的绑定算方向。 */
  if (zone == 2U)
  {
    dir_enter_strafe = ZDT_SCRIPT_DIR_LEFT;  /* map +x → 回区中心 x=2250 */
    dir_enter_back = ZDT_SCRIPT_DIR_BACK;    /* map +y → 进 2 号区 */
  }
  else
  {
    dir_enter_strafe = ZDT_SCRIPT_DIR_RIGHT; /* map +x */
    dir_enter_back = ZDT_SCRIPT_DIR_BACK;    /* map −y → 进 1 号区 */
  }

  /* ①② 出场：前进 300 + 平移到车道 */
  steps[0].dir = dir_leave_fwd;
  steps[0].mm = ZDT_SCRIPT_LEAVE_MM;
  steps[0].map_dx = 0;
  steps[0].map_dy = (int16_t)((zdt_start_zone == 1U) ? 300 : -300);
  steps[1].dir = dir_leave_strafe;
  steps[1].mm = ZDT_SCRIPT_STRAFE_MM;
  steps[1].map_dx = -125;
  steps[1].map_dy = 0;
  /* ③ 沿车道直走（这就是"直走到另一个启停区"的那一段） */
  steps[2].dir = dir_run;
  steps[2].mm = (uint16_t)((run >= 0) ? run : -run);
  steps[2].map_dx = 0;
  steps[2].map_dy = (int16_t)run;
  /* ④ 在车道尽头掉头 180°（无位移） */
  steps[3].dir = ZDT_SCRIPT_DIR_TURN180;
  steps[3].mm = 0U;
  steps[3].map_dx = 0;
  steps[3].map_dy = 0;
  /* ⑤⑥ 进区：平移回区中心轴线 + 后退进区 */
  steps[4].dir = dir_enter_strafe;
  steps[4].mm = ZDT_SCRIPT_STRAFE_MM;
  steps[4].map_dx = 125;
  steps[4].map_dy = 0;
  steps[5].dir = dir_enter_back;
  steps[5].mm = ZDT_SCRIPT_LEAVE_MM;
  steps[5].map_dx = 0;
  steps[5].map_dy = (int16_t)((zone == 2U) ? 300 : -300);

  /* 起手位姿闸门：`&cross` 同样是**相对动作**（先出场、再进区），
     起手不在当前启停区里就会算错起点（和 &leave/&home 同一类事故）。 */
  if (ZDT_Script_RequirePoseNear(ZDT_SCRIPT_ZONE_X_MM, from_y, 150U,
                                 "&cross") == 0U)
  {
    return;
  }
  {
    char line[160];

    (void)snprintf(line, sizeof(line),
                   "[NAV] cross: zone %u -> zone %u, straight %.0fmm on lane x=%u,"
                   " then 180 turn\r\n",
                   (unsigned int)zdt_start_zone, (unsigned int)zone,
                   (double)((run >= 0) ? run : -run),
                   (unsigned int)ZDT_SCRIPT_LANE_X_MM);
    ZDT_Console_Print(line);
  }
  zdt_script_end_zone = zone;
  ZDT_Script_Queue(steps, 6U);
}

/* 出场：前进一个车长（驶出启停区）→ 平移 125 mm 到外环带中心线。
 *
 * ★ 现场排布（2026-09-13 最终确认，与官方规则图一致）：两个启停区在**右边缘**的
 *   右上角与右下角，二维码板在右边缘中部、正好把两区夹在中间（认场地最快的判据），
 *   原料区在顶边中央。
 *     · 1 号区 = 右下角 (2250,150)，车头朝 **+y**（北）→ 地图 −x（西）在车体**左**侧 ⇒ STRAFE-LEFT
 *     · 2 号区 = 右上角 (2250,2250)，车头朝 **−y**（南）→ 地图 −x（西）在车体**右**侧 ⇒ STRAFE-RIGHT
 *   出场都是"沿车头前进 300 → 朝右边缘环带中心线 x=2125 平移 125"，落点 (2125,450)/(2125,1950)。
 *   本队实际发车区 = 2 号区（`ZDT_START_ZONE_DEFAULT`）。
 * 为什么必须先前进：启停区占掉右边缘环带的外侧 300 mm，剩下的通道
 *   2400−2100 = 250 mm < 车宽280 mm，**绕不过去**。
 *
 * ⚠ 历史（别再照抄）：本动作表先后按"左右下/左上斜对角"和"两区都在底边"改过两轮，
 *   那两版都把车头当成沿 ±x。最终版只认上面这一份；改朝向时**动作表与车头绑定必须同时改**。 */
static void ZDT_Script_StartLeave(void)
{
  /* 闸门：起手位姿必须还在所选启停区里（300×300，中心 (2250,150)/(2250,2250)）。
     否则这次"前进 300 + 平移 125"会从错误起点算起，把车推出场外。 */
  if (ZDT_Script_RequirePoseNear(2250U,
                                 (uint16_t)((zdt_start_zone == 1U) ? 150U : 2250U),
                                 150U, "&leave") == 0U)
  {
    return;
  }
  /* 启停区**可通行**（2026-09-13 现场更正）：这里原来会记下"刚离开的区"以便
     出库后锁成车体不可接触，现在整套锁定已删除。 */
  if (zdt_start_zone == 1U)
  {
    /* 右下角（起点）：车头 +y（北）→ 地图 −x 在车体**左**侧 */
    static const zdt_script_step_t steps[2] =
    {
      { 0U, 300U,    0, +300 },   /* FWD 300（车头 +y）→ map y +300 到 y=450 */
      { 2U, 125U, -125,    0 }    /* LEFT 125 → map x −125 到 x=2125 */
    };
    ZDT_Script_Queue(steps, 2U);
  }
  else
  {
    /* 右上角：车头 −y（南）→ 地图 −x 在车体**右**侧 */
    static const zdt_script_step_t steps[2] =
    {
      { 0U, 300U,    0, -300 },   /* FWD 300（车头 −y）→ map y −300 到 y=1950 */
      { 3U, 125U, -125,    0 }    /* RIGHT 125 → map x −125 到 x=2125 */
    };
    ZDT_Script_Queue(steps, 2U);
  }
  zdt_script_fast_exit=1U;
  if (zdt_start_pose_set == 0U)
  {
    ZDT_Console_Print("[NAV] WARNING: &startpose was not run since power-on; the nose"
                   " binding is still the default. If the car was not placed nose-first"
                   " into the field, it will drive the wrong way (and can leave the mat).\r\n");
  }
  ZDT_Console_Print("[NAV] leaving start zone: forward 300mm then strafe 125mm\r\n");
}

/* 回场：反向执行（先平移回启停区轴线，再后退进区）。 */
static void ZDT_Script_StartHome(void)
{
  /* 闸门：回场是出场动作的逆运算，起手位姿必须在**出场落点**附近
     （exit1=(2125,450) / exit2=(2125,1950)），否则同样是相对动作错起点。 */
  if (ZDT_Script_RequirePoseNear(2125U,
                                 (uint16_t)((zdt_start_zone == 1U) ? 450U : 1950U),
                                 200U, "&home") == 0U)
  {
    return;
  }
  if (zdt_start_zone == 1U)
  {
    static const zdt_script_step_t steps[2] =
    {
      { 3U, 125U, +125,    0 },   /* RIGHT 125 → map x +125（退回启停区轴线） */
      { 1U, 300U,    0, -300 }    /* BACK 300 → map y −300（退回启停区中心） */
    };
    ZDT_Script_Queue(steps, 2U);
  }
  else
  {
    static const zdt_script_step_t steps[2] =
    {
      { 2U, 125U, +125,    0 },   /* LEFT 125 → map x +125 */
      { 1U, 300U,    0, +300 }    /* BACK 300 → map y +300 */
    };
    ZDT_Script_Queue(steps, 2U);
  }
  if (zdt_start_pose_set == 0U)
  {
    ZDT_Console_Print("[NAV] WARNING: &startpose was not run since power-on.\r\n");
  }
  ZDT_Console_Print("[NAV] homing: strafe 125mm then back 300mm into start zone\r\n");
}

/* 设定起始位姿：位姿(mm) + 车头朝向 + 重对航向基准（要静止 3 s）。
 * 2026-10-04: 正向左侧为机械臂；当前为调头180度试验版，认定的车头朝场地内。
 * 1 = **右下角**启停区 (2250,150)，车头朝地图 **+y**（朝场地内）
 * 2 = **右上角**启停区 (2250,2250)，车头朝地图 **−y**（朝场地内）← `ZDT_START_ZONE_DEFAULT`
 * 两个区都在右边缘，二维码板在中间（官方规则图布局）。 */
static void ZDT_Script_SetStartZone(uint8_t zone)
{
  char line[128];

  zdt_start_zone = (zone == 2U) ? 2U : 1U;
  if (zdt_start_zone == 1U)
  {
    ZDT_Nav_SetNoseDir(1U);            /* 车头朝地图 +y */
    GridNav_SetPoseMm(2250U, 150U);
  }
  else
  {
    ZDT_Nav_SetNoseDir(0U);            /* 车头朝地图 −y */
    GridNav_SetPoseMm(2250U, 2250U);
  }
  ChassisMotor_ClearHeadingReference();HWT101_ResetZeroDrift();
  zdt_start_pose_set = 1U;
  /* 启停区**可通行**（2026-09-13 现场更正）：这里原来会 `GridNav_SetZoneNoTouch(0)`
     解锁"出库后不可接触"，现在整套锁定都删了 —— 车可以照常压过两个区。 */
  /* 这行是现场唯一能当场发现"区号选错"的地方，所以把**角位**和**车头方向**都写清楚：
     1 号区在右下角、车头朝 +y；2 号区在右上角、车头朝 −y。 */
  (void)snprintf(line, sizeof(line),
                 "[NAV] start zone %u (%s, nose %s into the field):"
                 " pose=(2250mm,%umm); keep still ~3s\r\n",
                 (unsigned int)zdt_start_zone,
                 (zdt_start_zone == 1U) ? "BOTTOM-RIGHT" : "TOP-RIGHT",
                 (zdt_start_zone == 1U) ? "+y" : "-y",
                 (unsigned int)((zdt_start_zone == 1U) ? 150U : 2250U));
  ZDT_Console_Print(line);
#if CHASSIS_FRONT_REVERSED_TRIAL
  ZDT_Console_Print("[NAV] FRONT180 TRIAL: forward/right reversed; CCW unchanged; motors FL4 FR2 RL3 RR1.\r\n");
#else
  ZDT_Console_Print("[NAV] original front restored; arm on body LEFT.\r\n");
#endif
}

/* Bind the physical pose only at accepted launch; keep the calibrated gyro bias. */
static HAL_StatusTypeDef ZDT_ConfirmLaunchZone(uint8_t zone)
{
  char line[144];
  if(zone<1U || zone>2U || ChassisMotor_RebaseHeading(0.0f)!=HAL_OK)return HAL_ERROR;
  zdt_start_zone=zone;ZDT_Nav_SetNoseDir(zone==1U?1U:0U);
  GridNav_SetPoseMm(2250U,zone==1U?150U:2250U);zdt_start_pose_set=1U;
  (void)snprintf(line,sizeof(line),"[MATCH] LAUNCH_BIND zone=%u pose=(2250,%u) nose=%s; current heading rebound, gyro bias retained\r\n",
      zone,zone==1U?150U:2250U,zone==1U?"+y":"-y");ZDT_Console_Print(line);return HAL_OK;
}

/* 开机把"默认起点区"（ZDT_START_ZONE_DEFAULT）的位姿与车头绑定装好。
 *
 * 与 ZDT_Script_SetStartZone() 的区别：这里**不**重对陀螺零漂、也**不**把位姿标记成
 * "已确认"（zdt_start_pose_set 保持 0）。因为这时人还没在场地里确认过摆放 ——
 * 屏幕会据此显示黄色的 `ZONE n?`，提醒"位姿只是假定值，发车前请发 &startpose N"。 */
static void ZDT_Script_ApplyDefaultZone(void)
{
  zdt_start_zone = ZDT_START_ZONE_DEFAULT;
  if (zdt_start_zone == 1U)
  {
    ZDT_Nav_SetNoseDir(1U);            /* 车头朝地图 +y */
    GridNav_SetPoseMm(2250U, 150U);
  }
  else
  {
    ZDT_Nav_SetNoseDir(0U);            /* 车头朝地图 −y */
    GridNav_SetPoseMm(2250U, 2250U);
  }
}

/* 启停区状态导出（屏幕显示用，见 zdt_motor.h 的说明）。 */
uint8_t ZDT_Script_GetStartZone(void)
{
  return (zdt_start_zone == 2U) ? 2U : 1U;
}

uint8_t ZDT_Script_IsStartPoseSet(void)
{
  return (zdt_start_pose_set != 0U) ? 1U : 0U;
}

/* 脚本步的航向保持：把车头钉在"本步开始时的航向"上（`zdt_script_hold_heading`）。
 *
 * 与导航的 ZDT_Nav_HeadingHoldTo() 用**同一套参数**（kp / 限幅 / 死区 / 80 ms 周期），
 * 但速度通道要按**本步的车体方向**给：直走 = vx，平移 = vy。不能直接复用导航那个函数 ——
 * 它按 `zdt_nav_drive_lateral` / `zdt_nav_drive_sign` 选通道，那是导航段的标记，
 * 脚本用会选错（会把平移步当成直走步下发 vx）。
 *
 * 只在脚本步真正走行期间调用；掉头步不调（转向有自己的 PID）。 */
static void ZDT_Script_HeadingHold(void)
{
  const zdt_script_step_t *step = &zdt_script_queue[zdt_script_index];
  float error = zdt_script_hold_heading - ZDT_Nav_GetHeading();
  float omega;
  float v = zdt_nav_odometry_enabled?zdt_script_drive_scale:Nav_Params()->speed_scale;
  float vx = 0.0f;
  float vy = 0.0f;

  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }

  if (fabsf(error) <= ZDT_NAV_HOLD_DEADBAND_DEG)
  {
    omega = 0.0f;
  }
  else
  {
    omega = ZDT_NAV_HOLD_KP * error;
    if (omega > ZDT_NAV_HOLD_OMEGA_MAX)
    {
      omega = ZDT_NAV_HOLD_OMEGA_MAX;
    }
    else if (omega < -ZDT_NAV_HOLD_OMEGA_MAX)
    {
      omega = -ZDT_NAV_HOLD_OMEGA_MAX;
    }
  }

  switch (step->dir)
  {
    case ZDT_SCRIPT_DIR_FWD:   vx = v;  break;
    case ZDT_SCRIPT_DIR_BACK:  vx = -v; break;
    case ZDT_SCRIPT_DIR_LEFT:  vy = -v; break;
    default:                   vy = v;  break;   /* RIGHT */
  }
  (void)ChassisMotor_SetVelocity(vx, vy, omega);
}

static uint32_t ZDT_Script_SettleDelay(const zdt_script_step_t *step)
{
  /* Odo DONE has already verified stopped wheels with new position replies.
   * Avoid a second fixed 350ms wait on each departure translation. */
  return (zdt_nav_odometry_enabled && zdt_script_fast_exit && step &&
    step->dir!=ZDT_SCRIPT_DIR_TURN180)?0U:ZDT_SCRIPT_SETTLE_MS;
}
static void ZDT_Script_Service(void)
{
  const zdt_script_step_t *step;
  uint32_t now = HAL_GetTick();
  float scale;
  /* Keep the original script queued while its interrupted forward step is
   * being rerouted; NAV owns the shared odometry until actual arrival. */
  if(zdt_vobs_resume_script || zdt_vobs_phase)return;
  if (zdt_script_running == 0U)
  {
    return;
  }

  /* 全部动作做完：把累计位移写回位姿并打一行总结 */
  if (zdt_script_index >= zdt_script_count)
  {
    char line[112];
    uint16_t px = 0U;
    uint16_t py = 0U;

    /* 启停区**可通行**（2026-09-13 现场更正）：这里原来会把"刚离开的区"锁成
       车体不可接触，现在整套锁定已删除，出库/跨区之后区里照常可以走。 */

    if (zdt_script_end_zone != 0U)
    {
      /* `&cross`：到位后直接把"当前启停区"切到目标区 —— 它会按新区设位姿
         （精确到区中心，不用累计位移）、翻转车头绑定、并重对陀螺零漂。
         注意车头绑定必须跟着翻：掉头 180° 后"车头朝场地内"的方向正好相反，
         不翻的话下一次 `&leave` 的"前进"会直接撞墙。 */
      uint8_t zone = zdt_script_end_zone;

      zdt_script_end_zone = 0U;
      zdt_script_running = 0U;
      ZDT_Script_SetStartZone(zone);
      (void)snprintf(line, sizeof(line),
                     "[NAV] cross done: now in start zone %u (keep still ~3s)\r\n",
                     (unsigned int)zone);
      ZDT_Console_Print(line);
      if (zdt_script_then_nav != 0U)
      {
        zdt_script_then_nav = 0U;
        ZDT_Nav_StartPending();
      }
      return;
    }

    GridNav_GetPoseMm(&px, &py);
    GridNav_SetPoseMm((uint16_t)((int32_t)px + zdt_script_dx_total),
                      (uint16_t)((int32_t)py + zdt_script_dy_total));
    if(zdt_nav_odometry_enabled)zdt_nav_post_align=1U;
    GridNav_GetPoseMm(&px, &py);
    (void)snprintf(line, sizeof(line),
                   "[NAV] script done: pose=(%umm,%umm)\r\n",
                   (unsigned int)px, (unsigned int)py);
    ZDT_Console_Print(line);
    zdt_script_running = 0U;
    /* ② 出场之后接着跑被推迟的那次导航（`&goto` 在启停区里被推迟到这里）。 */
    ZDT_Match_OnScriptDone();
    if (zdt_script_then_nav != 0U)
    {
      zdt_script_then_nav = 0U;
      ZDT_Nav_StartPending();
    }
    return;
  }

  step = &zdt_script_queue[zdt_script_index];
  scale = Nav_Params()->speed_scale;

  if (zdt_script_settling != 0U)
  {
    if ((now - zdt_script_settle_tick) >= ZDT_Script_SettleDelay(step))
    {
      zdt_script_settling = 0U;
      float fraction=(zdt_nav_odometry_enabled && step->mm)?zdt_script_measured_mm/step->mm:1.0f;
      zdt_script_dx_total = (int16_t)(zdt_script_dx_total + (int16_t)lroundf(step->map_dx*fraction));
      zdt_script_dy_total = (int16_t)(zdt_script_dy_total + (int16_t)lroundf(step->map_dy*fraction));
      zdt_script_index++;
    }
    return;
  }

  /* 掉头步：发一次 180° 原地转，然后等它转完（转向结束由电机层给出），
     再走常规的"停稳 350 ms"再进下一步。 */
  if (step->dir == ZDT_SCRIPT_DIR_TURN180)
  {
    if (zdt_script_deadline == 0U)
    {
      char line[112];

      if (ChassisMotor_TurnRelativeFast(180) != HAL_OK)
      {
        ZDT_Console_Print("[NAV] cross aborted: 180 turn rejected"
                       " (need &EN and IMU zero-drift ready)\r\n");
        zdt_script_running = 0U;
        zdt_script_end_zone = 0U;
        return;
      }
      zdt_script_deadline = 1U;   /* 非 0 = 已启动，等转向结束 */
      (void)snprintf(line, sizeof(line),
                     "[NAV] script %u/%u: TURN 180 (face into the field)\r\n",
                     (unsigned int)(zdt_script_index + 1U),
                     (unsigned int)zdt_script_count);
      ZDT_Console_Print(line);
      return;
    }
    if (ChassisMotor_IsTurnActive() != 0U)
    {
      return;   /* 还在转 */
    }
    zdt_script_deadline = 0U;
    zdt_script_settling = 1U;
    zdt_script_settle_tick = now;
    return;
  }

  if(zdt_nav_odometry_enabled) {
    if(zdt_script_deadline==0U) {
      int8_t forward=0,right=0;
      zdt_script_hold_heading=ZDT_Nav_GetHeading();zdt_script_drive_scale=scale;
      if(step->dir==ZDT_SCRIPT_DIR_FWD)forward=1;
      else if(step->dir==ZDT_SCRIPT_DIR_BACK)forward=-1;
      else right=step->dir==ZDT_SCRIPT_DIR_LEFT?-1:1;
      if(zdt_vobs_rebase_script) {
        /* A detour can leave the chassis facing another cardinal direction.
         * Preserve the script's world-map displacement, not its old body axis. */
        ZDT_Vobs_MapAxes(ZDT_Vobs_WorldYaw(),step->map_dx,step->map_dy,&forward,&right);
      }
      if(!ZDT_Odo_Start(2U,(float)step->mm,scale,forward,right,zdt_script_hold_heading,NULL,0U)) {
        zdt_script_running=0U;ZDT_Match_Complete(0U,"script odometry start rejected");return;
      }
      zdt_script_deadline=1U;
    } else {
      uint8_t result=ZDT_Odo_Service(2U);
      if(result==2U) {zdt_script_running=0U;zdt_script_then_nav=0U;zdt_script_end_zone=0U;
        ZDT_Match_Complete(0U,zdt_odo.reason);return;}
      if(result==1U) {zdt_script_measured_mm=zdt_odo.progress_mm;zdt_script_deadline=0U;
        zdt_script_settling=1U;zdt_script_settle_tick=now;}
    }
    return;
  }
  if (zdt_script_deadline == 0U)
  {
    /* 起这一步：按车体方向给速度（平移 = vy，前进/后退 = vx） */
    float vx = 0.0f;
    float vy = 0.0f;

    switch (step->dir)
    {
      case 0U: vx = scale;  break;
      case 1U: vx = -scale; break;
      case 2U: vy = -scale; break;
      default: vy = scale;  break;
    }
    if ((ChassisMotor_IsEnabled() == 0U) ||
        (ChassisMotor_SetVelocity(vx, vy, 0.0f) != HAL_OK))
    {
      ZDT_Console_Print("[NAV] script aborted: motors disabled or command rejected\r\n");
      zdt_script_running = 0U;
      ZDT_Match_Complete(0U, "script command rejected");
      return;
    }
    zdt_script_deadline = now +
        ZDT_Nav_PlanDriveMs((float)step->mm / ZDT_MILEAGE_MM_PER_REV, scale);
    /* ★ 记下"本步开始时的车头"：后面每 80 ms 用它做航向闭环。
       脚本动作原来**完全没有航向闭环**，`&cross` 那条 1500 mm 直走（spd=0.2 下约 7.5 s）
       期间车头一旦慢慢偏就没人管，而它后面接的掉头/平移/后退都是**相对**动作
       ⇒ 段末歪多少，落点就偏多少（现场表现：车停在区里是斜的）。 */
    zdt_script_hold_heading = ZDT_Nav_GetHeading();
    zdt_script_hold_tick = now + ZDT_NAV_HOLD_PERIOD_MS;
    {
      char line[112];
      (void)snprintf(line, sizeof(line),
                     "[NAV] script %u/%u: %s %umm cmd=%.0fmm\r\n",
                     (unsigned int)(zdt_script_index + 1U),
                     (unsigned int)zdt_script_count,
                     (step->dir == ZDT_SCRIPT_DIR_FWD) ? "FWD" :
                     (step->dir == ZDT_SCRIPT_DIR_BACK) ? "BACK" :
                     (step->dir == ZDT_SCRIPT_DIR_LEFT) ? "STRAFE-L" : "STRAFE-R",
                     (unsigned int)step->mm,
                     (double)Nav_Params_CompensatedDistance((float)step->mm));
      ZDT_Console_Print(line);
    }
    return;
  }

  if ((int32_t)(now - zdt_script_deadline) >= 0)
  {
    (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
    zdt_script_deadline = 0U;
    zdt_script_settling = 1U;
    zdt_script_settle_tick = now;
    return;
  }

  /* 还在走：航向闭环（每 ZDT_NAV_HOLD_PERIOD_MS 纠一次，与导航同一套参数）。
     掉头步在更上面就 return 了，所以这里只管 FWD/BACK/STRAFE 四类步。 */
  if ((int32_t)(now - zdt_script_hold_tick) >= 0)
  {
    zdt_script_hold_tick = now + ZDT_NAV_HOLD_PERIOD_MS;
    ZDT_Script_HeadingHold();
  }
}

static void ZDT_Odo_Cancel(void)
{
  if(zdt_odo_wait)zdt_mileage_query_active=0U;
  zdt_odo_owner=zdt_odo_wait=zdt_odo_base_pending=0U;
  NavOdometry_Cancel(&zdt_odo);
}
static uint8_t ZDT_Odo_InitBase(const int32_t *base,uint8_t mask,uint32_t now)
{
  uint8_t ready;
  float expected[4];NavFrame_VelocityWheelValues(zdt_odo_forward,zdt_odo_right,expected);
  ready=NavOdometry_Start(&zdt_odo,zdt_odo_mm,zdt_odo_max_scale,Nav_Params()->counts_per_rev,
    ZDT_MILEAGE_MM_PER_REV,CHASSIS_MAX_WHEEL_RPM,CHASSIS_DECEL_RPM_PER_S,
    base,mask,zdt_mileage_forward_signs,expected,now);
  if(ready)zdt_odo.approach_gain=zdt_odo_approach_gain;
  return ready;
}
static uint8_t ZDT_Odo_Start(uint8_t owner,float mm,float scale,int8_t forward,int8_t right,
    float heading,const int32_t *base,uint8_t mask)
{
  char line[184];uint32_t now=HAL_GetTick();
  if(zdt_odo_owner || !ChassisMotor_IsEnabled() || !HWT101_IsZeroDriftReady() ||
     !isfinite(heading) || (abs(forward)+abs(right)!=1) || !isfinite(mm) || mm<=0.0f ||
     !isfinite(scale) || scale<0.01f || scale>1.0f)return 0U;
  /* One target expansion, before either immediate or deferred baseline.
   * Keep the encoder counts/rev and measured map displacement unchanged. */
  /* A checked obstacle detour is verified at exact mm coordinates; applying
   * the usual calibrated 1.03 expansion again would execute a different path.
   * Normal script and navigation retain their original compensation. */
  zdt_odo_mm=ZDT_Vobs_OdoTarget(owner,mm);
  zdt_odo_max_scale=scale;zdt_odo_forward=forward;zdt_odo_right=right;zdt_odo_heading=heading;
  zdt_odo_approach_gain=(owner==2U && zdt_script_fast_exit)?NAV_ODOMETRY_EXIT_APPROACH_GAIN:NAV_ODOMETRY_APPROACH_GAIN;
  NavOdometry_Cancel(&zdt_odo);
  if(base && !ZDT_Odo_InitBase(base,mask,now))return 0U;
  zdt_odo_owner=owner;zdt_odo_base_pending=(uint8_t)(base==NULL);zdt_odo_wait=0U;
  if(owner==1U || ZDT_Vobs_IsLeaveOdo()) {
    zdt_vobs_history_count=0U;if(!zdt_odo_base_pending)ZDT_Vobs_RecordOdo();
  }
  zdt_odo_tick=zdt_odo_imu_tick=zdt_odo_next_query=now;zdt_odo_imu_count=HWT101_GetSampleCount();
  ChassisMotor_ResetIdleRamp();
  (void)snprintf(line,sizeof(line),"[ODO] V174 owner=%s nominal=%.2f target=%.2fmm x%.3f F=%d R=%d vmax=%.3f; encoder arrival\r\n",
    owner==1U?"NAV":owner==3U?"ALIGN":"SCRIPT",(double)mm,(double)zdt_odo_mm,
    (double)((owner==1U && zdt_vobs_nav_active)?1.0f:NAV_STRAIGHT_TARGET_SCALE),forward,right,(double)scale);ZDT_Console_Print(line);
  return 1U;
}
static uint8_t ZDT_Odo_OnMileage(uint8_t mask)
{
  uint32_t now=HAL_GetTick();
  if(!zdt_odo_owner || !zdt_odo_wait)return 0U;
  zdt_odo_wait=0U;zdt_mileage_query_active=zdt_mileage_query_report=zdt_mileage_query_attempts=0U;
  if(zdt_odo_base_pending) {
    zdt_odo_base_pending=0U;
    if(!ZDT_Odo_InitBase(zdt_mileage_query_position,mask,zdt_mileage_query_sample_tick)) {
      zdt_odo.state=NAV_ODO_FAILED;zdt_odo.reason="odometry baseline needs four valid replies";
    }
  } else NavOdometry_Observe(&zdt_odo,zdt_mileage_query_position,mask,zdt_mileage_query_sample_tick);
  if(mask==0x0fU && (zdt_odo_owner==1U || ZDT_Vobs_IsLeaveOdo()) &&
      !zdt_odo_base_pending)ZDT_Vobs_RecordOdo();
  zdt_odo_next_query=now+NAV_ODOMETRY_QUERY_MS;return 1U;
}
static uint8_t ZDT_Odo_Service(uint8_t owner)
{
  uint32_t now=HAL_GetTick(),samples;uint8_t i,stopped=1U;
  chassis_telemetry_t t;int16_t rpm[4];float scale,error,omega;
  if(zdt_odo_owner!=owner){zdt_odo.reason="odometry motion owner lost";return 2U;}
  ChassisMotor_GetTelemetry(&t);ChassisMotor_GetWheelRpm(rpm);
  samples=HWT101_GetSampleCount();
  if(samples!=zdt_odo_imu_count){zdt_odo_imu_count=samples;zdt_odo_imu_tick=now;}
  error=NavBlend_AngleError(zdt_odo_heading,t.heading_deg);
  if(!ChassisMotor_IsEnabled() || !HWT101_IsZeroDriftReady() || !isfinite(t.heading_deg) ||
      !isfinite(t.gyro_dps) || now-zdt_odo_imu_tick>150U || fabsf(error)>5.0f) {
    zdt_odo.state=NAV_ODO_FAILED;zdt_odo.reason="odometry chassis/IMU/heading invalid";
  }
  if(zdt_odo_base_pending && now-zdt_odo_tick>=NAV_ODOMETRY_STALE_MS) {
    zdt_odo.state=NAV_ODO_FAILED;zdt_odo.reason="odometry baseline timeout";
  }
  for(i=0U;i<4U;i++)if(rpm[i])stopped=0U;
  scale=zdt_odo_base_pending?0.0f:NavOdometry_Step(&zdt_odo,now,stopped);
  if(zdt_odo.state==NAV_ODO_FAILED) {
    const char *reason=zdt_odo.reason;char line[220];
    (void)ChassisMotor_StopImmediate();
    /* Preserve the actual final measurement before cancellation and emit
     * diagnostics only after the chassis STOP request. Never infer whether
     * the failure was overshoot from a reason string without distance. */
    (void)snprintf(line,sizeof(line),"[ODO FAULT] owner=%u target=%.1f measured=%.1f err=%+.1fmm spread=%.1f age=%lums gain=%.1f final_tol=%.1f; %s\r\n",
      owner,(double)zdt_odo.target_mm,(double)zdt_odo.progress_mm,
      (double)(zdt_odo.target_mm-zdt_odo.progress_mm),(double)zdt_odo.spread_mm,
      (unsigned long)(now-zdt_odo.last_sample),(double)zdt_odo.approach_gain,(double)NAV_ODOMETRY_FINAL_MM,
      reason?reason:"unknown");
    ZDT_Console_Print(line);ZDT_Odo_Cancel();
    zdt_odo.reason=reason;return 2U;
  }
  if(zdt_odo.state==NAV_ODO_DONE) {
    char line[112];
    (void)snprintf(line,sizeof(line),"[ODO] done target=%.1f measured=%.1f err=%+.1fmm\r\n",
      (double)zdt_odo.target_mm,(double)zdt_odo.progress_mm,(double)(zdt_odo.target_mm-zdt_odo.progress_mm));
    ZDT_Console_Print(line);zdt_odo_owner=0U;(void)ChassisMotor_Stop();return 1U;
  }
  omega=fabsf(error)<=ZDT_NAV_HOLD_DEADBAND_DEG?0.0f:fmaxf(-0.02f,fminf(0.02f,error*ZDT_NAV_HOLD_KP));
  if(scale==0.0f)omega=0.0f;
  if(ChassisMotor_SetVelocity(zdt_odo_forward*scale,zdt_odo_right*scale,omega)!=HAL_OK) {
    (void)ChassisMotor_StopImmediate();ZDT_Odo_Cancel();zdt_odo.reason="odometry velocity command rejected";return 2U;
  }
  if(owner==1U)zdt_nav_drive_scale=scale;else zdt_script_drive_scale=scale;
  if(!zdt_odo_wait && !zdt_mileage_query_active && (int32_t)(now-zdt_odo_next_query)>=0) {
    zdt_odo_wait=1U;ZDT_Mileage_StartQuery(0U);
  }
  return 0U;
}
/* ---------- Experimental front-camera obstacle/reroute owner ---------- */
static float ZDT_Vobs_WorldYaw(void)
{
  return ZDT_Nav_GetHeading()+atan2f((float)-zdt_nav_nose_y,(float)zdt_nav_nose_x)*57.29578f;
}
static float ZDT_Vobs_OdoTarget(uint8_t owner,float mm)
{
  return (owner==1U && zdt_vobs_nav_active)?mm:Nav_Params_StraightDistance(mm);
}
static void ZDT_Vobs_MapAxes(float yaw,int16_t dx,int16_t dy,int8_t *forward,int8_t *right)
{
  float angle=yaw*0.017453292519943295f;
  float f=dx*cosf(angle)+dy*sinf(angle),r=dx*sinf(angle)-dy*cosf(angle);
  *forward=*right=0;
  if(fabsf(f)>=fabsf(r))*forward=f>=0.0f?1:-1;
  else *right=r>=0.0f?1:-1;
}
static uint8_t ZDT_Vobs_IsLeaveOdo(void)
{
  return (uint8_t)(zdt_odo_owner==2U && zdt_script_running && !zdt_vobs_resume_script &&
    zdt_match.state==MATCH_LEAVING && zdt_match_io==MATCH_IO_LEAVE_SCRIPT &&
    zdt_script_index<zdt_script_count && zdt_odo_forward==1 && zdt_odo_right==0 &&
    zdt_script_queue[zdt_script_index].dir!=ZDT_SCRIPT_DIR_TURN180);
}
static uint8_t ZDT_Vobs_LivePose(float *x,float *y)
{
  uint16_t px,py;GridNav_GetPoseMm(&px,&py);*x=px;*y=py;
  if(zdt_vobs_phase==1U)
    return VisionObstacle_WheelPose(zdt_vobs_start_x,zdt_vobs_start_y,zdt_vobs_world_yaw,
        zdt_vobs_baseline,zdt_vobs_previous,zdt_vobs_signs,zdt_vobs_counts,
        zdt_vobs_mm_per_rev,x,y);
  if((zdt_odo_owner==1U || ZDT_Vobs_IsLeaveOdo()) && !zdt_odo_base_pending)
    return VisionObstacle_WheelPose((float)px+(ZDT_Vobs_IsLeaveOdo()?zdt_script_dx_total:0),
        (float)py+(ZDT_Vobs_IsLeaveOdo()?zdt_script_dy_total:0),ZDT_Vobs_WorldYaw(),zdt_odo.baseline,
        zdt_odo.previous,zdt_mileage_forward_signs,Nav_Params()->counts_per_rev,
        ZDT_MILEAGE_MM_PER_REV,x,y);
  return (uint8_t)(isfinite(*x) && isfinite(*y));
}
static uint8_t ZDT_Vobs_Eligible(void)
{
  chassis_telemetry_t t;ChassisMotor_GetTelemetry(&t);
  return (uint8_t)(zdt_vobs.armed && !zdt_vobs_phase && zdt_nav_odometry_enabled &&
    ((zdt_nav_state==ZDT_NAV_DRIVE && zdt_odo_owner==1U &&
      (!zdt_script_running || zdt_vobs_resume_script)) || ZDT_Vobs_IsLeaveOdo()) &&
    !zdt_odo_base_pending &&
    zdt_odo.state==NAV_ODO_RUN && zdt_odo_forward==1 && zdt_odo_right==0 && zdt_vobs_history_count>=2U &&
    HAL_GetTick()-zdt_odo.last_sample<=VOBS_MAX_AGE_MS &&
    !ChassisMotor_IsTurnActive() && !ArmMaterial_IsActive() && !ArmRound1_IsActive() &&
    HWT101_IsZeroDriftReady() && isfinite(t.heading_deg) && isfinite(t.gyro_dps) &&
    fabsf(t.gyro_dps)<=3.0f && fabsf(NavBlend_AngleError(zdt_odo_heading,t.heading_deg))<=3.0f);
}
static void ZDT_Vobs_Reset(void)
{
  VisionObstacle_SetArmed(&zdt_vobs,0U,HAL_GetTick());
  zdt_vobs_phase=zdt_vobs_wait=0U;zdt_vobs_auto_started=zdt_vobs_preparing=0U;
  zdt_vobs_resume_script=zdt_vobs_rebase_script=0U;
  zdt_vobs_leave_target=zdt_vobs_depart_detoured=zdt_vobs_nav_active=zdt_vobs_recenter_active=0U;
  zdt_vobs_state_dirty=1U;
  zdt_vobs_wireless_observation=0U;zdt_vobs_arm_tick=0U;Vision_ClearPcObstacle();
  /* Explicit disarmed boundary prevents a later GUI connection from adopting
   * an old session during STOP/reset or a new prepare. */
  ZDT_Vobs_State();
}
static void ZDT_Vobs_MatchState(match_state_t state)
{
  if(state==MATCH_PREPARING) {
    if(!zdt_vobs_preparing){ZDT_Vobs_Reset();zdt_vobs_preparing=1U;}
  } else if(state==MATCH_WAIT_START_OK || state==MATCH_LEAVING) {
    if(!zdt_vobs_auto_started) {
      if(!zdt_vobs_preparing)ZDT_Vobs_Reset(); /* Legacy departure boundary. */
      Vision_ClearPcObstacle();zdt_vobs_wireless_observation=0U;zdt_vobs_arm_tick=HAL_GetTick();
      VisionObstacle_SetArmed(&zdt_vobs,1U,HAL_GetTick());
      zdt_vobs_auto_started=1U;zdt_vobs_preparing=0U;
      zdt_vobs_state_dirty=1U;ZDT_Vobs_State();
    }
  } else if((state==MATCH_IDLE || state==MATCH_FAULT || state==MATCH_FINISHED) &&
            (zdt_vobs_auto_started || zdt_vobs_preparing)) {
    ZDT_Vobs_Reset();
  }
}
static void ZDT_Vobs_RecordOdo(void)
{
  uint8_t n=zdt_vobs_history_count;float x,y;
  if(zdt_odo_forward!=1 || zdt_odo_right || !ZDT_Vobs_LivePose(&x,&y))return;
  if(n && zdt_vobs_history_tick[n-1U]==zdt_odo.last_sample)return;
  if(n==8U) {
    memmove(zdt_vobs_history_tick,zdt_vobs_history_tick+1,7U*sizeof(uint32_t));
    memmove(zdt_vobs_history_x,zdt_vobs_history_x+1,7U*sizeof(float));
    memmove(zdt_vobs_history_y,zdt_vobs_history_y+1,7U*sizeof(float));n=7U;
  }
  zdt_vobs_history_tick[n]=zdt_odo.last_sample;zdt_vobs_history_x[n]=x;zdt_vobs_history_y[n]=y;
  zdt_vobs_history_count=(uint8_t)(n+1U);
}
static uint8_t ZDT_Vobs_FramePose(uint32_t age,float *x,float *y)
{
  uint32_t tick=HAL_GetTick()-age,dt;uint8_t i,n=zdt_vobs_history_count;float fraction;
  if(n<2U || (int32_t)(tick-zdt_vobs_history_tick[0])<0)return 0U;
  for(i=1U;i<n;i++)if((int32_t)(tick-zdt_vobs_history_tick[i])<=0)break;
  if(i==n)i=(uint8_t)(n-1U); /* Latest measured velocity extrapolates at most the current sample age. */
  dt=zdt_vobs_history_tick[i]-zdt_vobs_history_tick[i-1U];if(!dt)return 0U;
  fraction=(float)(tick-zdt_vobs_history_tick[i-1U])/(float)dt;
  *x=zdt_vobs_history_x[i-1U]+fraction*(zdt_vobs_history_x[i]-zdt_vobs_history_x[i-1U]);
  *y=zdt_vobs_history_y[i-1U]+fraction*(zdt_vobs_history_y[i]-zdt_vobs_history_y[i-1U]);
  return (uint8_t)(isfinite(*x) && isfinite(*y));
}
static void ZDT_Vobs_State(void)
{
  float x,y;char line[176];uint8_t eligible=ZDT_Vobs_Eligible();
  (void)ZDT_Vobs_LivePose(&x,&y);
  (void)snprintf(line,sizeof(line),
    "[VOBS STATE] v=1 session=%lu armed=%u eligible=%u x=%.0f y=%.0f yaw=%.1f\r\n",
    (unsigned long)zdt_vobs.session,zdt_vobs.armed,eligible,
    (double)x,(double)y,(double)ZDT_Vobs_WorldYaw());
  ZDT_Console_Print(line);zdt_vobs_state_tick=HAL_GetTick();zdt_vobs_state_dirty=0U;
  zdt_vobs_last_eligible=eligible;
}
static uint8_t ZDT_Vobs_DepartTargets(uint16_t px,uint16_t py)
{
  int32_t x=(int32_t)px+zdt_script_dx_total,y=(int32_t)py+zdt_script_dy_total;
  uint8_t i;
  for(i=zdt_script_index;i<zdt_script_count;i++) {
    const zdt_script_step_t *step=&zdt_script_queue[i];
    float distance=(i==zdt_script_index)?zdt_odo.target_mm:Nav_Params_StraightDistance((float)step->mm);
    if(step->dir==ZDT_SCRIPT_DIR_TURN180 || !step->mm)continue;
    x+=(int32_t)lroundf(step->map_dx*distance/step->mm);
    y+=(int32_t)lroundf(step->map_dy*distance/step->mm);
    if(x<0 || x>=(int32_t)GRID_NAV_FIELD_MM || y<0 || y>=(int32_t)GRID_NAV_FIELD_MM)return 0U;
    if(i==zdt_script_index){zdt_vobs_leave_step_x=(uint16_t)x;zdt_vobs_leave_step_y=(uint16_t)y;}
  }
  zdt_vobs_leave_exit_x=(uint16_t)x;zdt_vobs_leave_exit_y=(uint16_t)y;
  return 1U;
}
static uint8_t ZDT_Vobs_Known(uint16_t x,uint16_t y)
{
  uint8_t cx,cy;
  for(cy=0U;cy<GRID_NAV_SIZE;cy++)for(cx=0U;cx<GRID_NAV_SIZE;cx++)
    if(VisionObstacle_DiscCell(x,y,VOBS_MAP_RADIUS_MM,cx,cy) && !GridNav_IsObstacle(cx,cy))return 0U;
  return 1U;
}
static uint8_t ZDT_Vobs_PlanTarget(uint16_t gx,uint16_t gy)
{
  uint16_t x,y,i,n;int32_t dx,dy;float yaw=ZDT_Vobs_WorldYaw(),target,gap;
  zdt_nav_queued_recenter=0U;
  GridNav_GetPoseMm(&x,&y);dx=(int32_t)gx-x;dy=(int32_t)gy-y;
  if((!dx || !dy) && (dx || dy)) {
    target=atan2f((float)dy,(float)dx)*57.29578f;
    gap=fabsf(NavBlend_AngleError(target,yaw));
    if((gap<=3.0f || ((labs((long)dx)+labs((long)dy))<=150 && fabsf(gap-90.0f)<=3.0f)) &&
       GridNav_PlanStraightTargetMm(gx,gy)) {
      n=(uint16_t)((labs((long)dx)+labs((long)dy)+4)/5);
      for(i=0U;i<=n;i++) {
        uint16_t sx=(uint16_t)((int32_t)x+dx*i/n),sy=(uint16_t)((int32_t)y+dy*i/n);
        if(!GridNav_RotatedBodyFreeMm(sx,sy,(int16_t)lroundf(yaw)))break;
      }
      if(i>n)return 1U;
    }
  }
  if(dx && dy && GridNav_PlanRecenterTargetMmHeading(gx,gy,yaw)) {
    /* This planner can retain heading with a short first BACK segment.
     * Carry that execution choice with the plan instead of turning 180. */
    zdt_nav_queued_recenter=1U;return 1U;
  }
  return GridNav_PlanCheckedTargetMmHeading(gx,gy,yaw);
}
static uint8_t ZDT_Vobs_PlanDepartureEntry(uint16_t gx,uint16_t gy)
{
  uint16_t x,y,cx,cy,ex,ey,i,j,n;int32_t dx,dy,index,base;float yaw=ZDT_Vobs_WorldYaw();
  int horizontal=(fabsf(cosf(yaw*0.017453292519943295f))<0.1f);
  GridNav_GetPoseMm(&x,&y);
  base=horizontal?GridNav_MmToCellX(x):GridNav_MmToCellY(y);
  /* Try the nearest grid-centre lines first; at the x=2250 boundary this
   * picks x=2225 before considering a larger sideways displacement. */
  for(i=0U;i<9U;i++) {
    index=base+(i?((i&1U)?-(int32_t)((i+1U)/2U):(int32_t)(i/2U)):0);
    if(index<0 || index>=(int32_t)GRID_NAV_SIZE)continue;
    cx=horizontal?(uint16_t)index:GridNav_MmToCellX(x);
    cy=horizontal?GridNav_MmToCellY(y):(uint16_t)index;
    ex=horizontal?GridNav_CellCenterMmX(cx):x;ey=horizontal?y:GridNav_CellCenterMmY(cy);
    dx=(int32_t)ex-x;dy=(int32_t)ey-y;
    if((!dx && !dy) || labs((long)dx)+labs((long)dy)>150 ||
       !GridNav_RotatedBodyFreeMm(ex,ey,(int16_t)lroundf(yaw)))continue;
    n=(uint16_t)((labs((long)dx)+labs((long)dy)+4)/5);
    for(j=0U;j<=n;j++)if(!GridNav_RotatedBodyFreeMm(
       (uint16_t)((int32_t)x+dx*j/n),(uint16_t)((int32_t)y+dy*j/n),(int16_t)lroundf(yaw)))break;
    if(j<=n)continue;
    /* Preflight the onward route from the candidate, then restore the actual
     * pose before installing the short checked lateral entry segment. */
    GridNav_SetPoseMm(ex,ey);
    {uint8_t onward=ZDT_Vobs_PlanTarget(gx,gy);
     GridNav_SetPoseMm(x,y);
     if(onward && GridNav_PlanStraightTargetMm(ex,ey)) {
       zdt_nav_queued_recenter=0U; /* The preflight plan was replaced by a lateral entry. */
       zdt_vobs_recenter_active=1U;
       ZDT_Console_Print("[VOBS] checked lateral entry before detour\r\n");return 1U;
     }}
  }
  return 0U;
}
static void ZDT_Vobs_Replan(void)
{
  char line[160];uint8_t ok=0U;
  ZDT_Nav_RefreshCostModel();
  if(zdt_vobs_resume_script) {
    /* First preserve the interrupted departure step. If its waypoint is
     * blocked, attempt the remaining exit endpoint and then the actual next
     * station; every candidate is checked against the full physical map. */
    while(zdt_vobs_leave_target<3U && !ok) {
      if(zdt_vobs_leave_target==0U) {
        zdt_vobs_goal_x=zdt_vobs_leave_step_x;zdt_vobs_goal_y=zdt_vobs_leave_step_y;
      } else if(zdt_vobs_leave_target==1U) {
        zdt_vobs_goal_x=zdt_vobs_leave_exit_x;zdt_vobs_goal_y=zdt_vobs_leave_exit_y;
      } else if(zdt_match.qr_required && zdt_qr_enabled) {
        zdt_vobs_goal_x=GRID_NAV_QR_SCAN_X_MM;zdt_vobs_goal_y=GRID_NAV_QR_SCAN_Y_MM;
      } else {
        zdt_vobs_goal_x=GRID_NAV_RAW_APPROACH_X_MM;zdt_vobs_goal_y=GRID_NAV_RAW_APPROACH_Y_MM;
      }
      ok=ZDT_Vobs_PlanTarget(zdt_vobs_goal_x,zdt_vobs_goal_y);
      if(!ok)ok=ZDT_Vobs_PlanDepartureEntry(zdt_vobs_goal_x,zdt_vobs_goal_y);
      if(!ok)zdt_vobs_leave_target++;
    }
  } else {
    ok=ZDT_Vobs_PlanTarget(zdt_vobs_goal_x,zdt_vobs_goal_y);
    /* A measured pose may be free while a turn there is obstructed. Reuse
     * the checked short lateral entry before starting the onward route. */
    if(!ok)ok=ZDT_Vobs_PlanDepartureEntry(zdt_vobs_goal_x,zdt_vobs_goal_y);
  }
  if(!ok) {
    zdt_vobs_phase=2U;zdt_nav_state=ZDT_NAV_VOBS_HOLD;zdt_vobs_state_dirty=1U;
    ZDT_Console_Print("[VOBS REPLAN] result=NO_PATH; paused at measured pose; clear obstacle then explicit vobs retry, or STOP/reset\r\n");
    return;
  }
  zdt_vobs_phase=0U;zdt_vobs_wait=0U;
  /* Do not finish/cancel Zone2: its action id still owns this same target. */
  (void)snprintf(line,sizeof(line),"[VOBS REPLAN] result=OK goal_x=%u goal_y=%u leave_target=%u; continue\r\n",
      zdt_vobs_goal_x,zdt_vobs_goal_y,zdt_vobs_resume_script?zdt_vobs_leave_target:0U);ZDT_Console_Print(line);
  zdt_nav_post_align=0U;zdt_vobs_nav_active=1U;ZDT_Nav_Start();
  zdt_vobs_state_dirty=1U;
}
static uint8_t ZDT_Vobs_OnMileage(uint8_t mask)
{
  uint32_t now=HAL_GetTick();float x,y,max_step=0.0f;uint8_t i,added;char line[144];
  if(zdt_vobs_phase!=1U || !zdt_vobs_wait)return 0U;
  zdt_vobs_wait=0U;zdt_mileage_query_active=zdt_mileage_query_report=zdt_mileage_query_attempts=0U;
  zdt_vobs_next_query=now+NAV_ODOMETRY_QUERY_MS;
  /* Keep this normal obstacle pause until fresh stop measurements arrive.
   * No new loss/timeout watchdog or automatic MATCH fault is introduced. */
  if(mask!=0x0fU)return 1U;
  for(i=0U;i<4U;i++) {
    float step=fabsf((float)((double)((int64_t)zdt_mileage_query_position[i]-zdt_vobs_previous[i])*
      zdt_vobs_signs[i]*zdt_vobs_mm_per_rev/zdt_vobs_counts));
    if(step>max_step)max_step=step;
  }
  memcpy(zdt_vobs_previous,zdt_mileage_query_position,sizeof(zdt_vobs_previous));
  /* First post-stop sample only establishes a post-stop baseline. */
  if(!zdt_vobs_sample_tick || max_step>0.5f)zdt_vobs_stable_since=zdt_mileage_query_sample_tick;
  zdt_vobs_sample_tick=zdt_mileage_query_sample_tick;
  if(zdt_vobs_sample_tick-zdt_vobs_stable_since<100U)return 1U;
  if(!VisionObstacle_WheelPose(zdt_vobs_start_x,zdt_vobs_start_y,zdt_vobs_world_yaw,
      zdt_vobs_baseline,zdt_mileage_query_position,zdt_vobs_signs,zdt_vobs_counts,
      zdt_vobs_mm_per_rev,&x,&y))return 1U;
  GridNav_SetPoseMm((uint16_t)lroundf(x),(uint16_t)lroundf(y));
  /* Departure accounting is now rebased at the measured stationary pose;
   * its previous displacement must not be added a second time at completion. */
  if(zdt_vobs_resume_script)zdt_script_dx_total=zdt_script_dy_total=0;
  added=GridNav_AddObstacleDiscMm(zdt_vobs_object_x,zdt_vobs_object_y,VOBS_MAP_RADIUS_MM);
  (void)snprintf(line,sizeof(line),"[VOBS MAP] session=%lu seq=%lu x=%u y=%u radius=%u added=%u\r\n",
      (unsigned long)zdt_vobs.session,(unsigned long)zdt_vobs.last_sequence,
      zdt_vobs_object_x,zdt_vobs_object_y,VOBS_MAP_RADIUS_MM,added);ZDT_Console_Print(line);
  ZDT_Vobs_Replan();return 1U;
}
static void ZDT_Vobs_Service(void)
{
  uint32_t now=HAL_GetTick(),gap=now-zdt_vobs_poll_tick;vision_pc_obstacle_t wireless;uint32_t age;
  zdt_vobs_poll_tick=now;
  if(zdt_vobs_phase || zdt_vobs_resume_script) {
    /* Waiting for an obstacle decision does not finish or time out its route.
     * The full-run elapsed timer still includes this wait. */
    if(zdt_zone2.running)zdt_zone2.started+=gap;
    if(MatchControl_IsActive(&zdt_match))zdt_match.entered_tick+=gap;
    if(zdt_vobs_phase==1U && !zdt_vobs_wait && !zdt_mileage_query_active &&
        (int32_t)(now-zdt_vobs_next_query)>=0) {
      zdt_vobs_wait=1U;ZDT_Mileage_StartQuery(0U);
    }
  }
  /* Announce the first usable forward sample immediately, instead of waiting
   * up to one heartbeat while the chassis keeps moving. */
  if(ZDT_Vobs_Eligible()!=zdt_vobs_last_eligible)zdt_vobs_state_dirty=1U;
  if(zdt_vobs_state_dirty || (zdt_vobs.armed && now-zdt_vobs_state_tick>=200U))ZDT_Vobs_State();
  if(Vision_GetPcObstacle(&wireless,&age) && wireless.observation_id!=zdt_vobs_wireless_observation) {
    zdt_vobs_wireless_observation=wireless.observation_id;
    /* A frame captured before this run's arm cannot drive the new run. The
     * source id is separate from COM's sequence; normalize at this owner. */
    if(zdt_vobs.armed && ZDT_Vobs_Eligible() && age<=VOBS_MAX_AGE_MS && age<=now-zdt_vobs_arm_tick) {
      uint32_t sequence=zdt_vobs.sequence_valid?zdt_vobs.last_sequence+1U:1U;
      char command[64];
      (void)snprintf(command,sizeof(command),"vobs %lu %lu %u %d %lu",
        (unsigned long)zdt_vobs.session,(unsigned long)sequence,wireless.front_gap_mm,
        (int)wireless.body_right_mm,(unsigned long)age);
      (void)ZDT_Vobs_Command(command,now);
    }
  }
}
static uint8_t ZDT_Vobs_Command(const char *command,uint32_t received_tick)
{
  unsigned long session,sequence,age;long front,right;char extra,line[144];
  const char *reason;float x,y,ox,oy;uint16_t px,py;uint8_t leave;
  if(strncmp(command,"vobs",4U) || (command[4] && command[4]!=' '))return 0U;
  if(!strcmp(command,"vobs") || !strcmp(command,"vobs status")){ZDT_Vobs_State();return 1U;}
  if(!strcmp(command,"vobs on")) {
    if(ZDT_Align_DiagBusy() || zdt_vobs_phase) {
      ZDT_Console_Print("[VOBS] ON rejected: stopped chassis required; speed unchanged\r\n");return 1U;
    }
    Vision_ClearPcObstacle();zdt_vobs_wireless_observation=0U;zdt_vobs_arm_tick=HAL_GetTick();
    VisionObstacle_SetArmed(&zdt_vobs,1U,HAL_GetTick());ZDT_Vobs_State();return 1U;
  }
  if(!strcmp(command,"vobs off")) {
    VisionObstacle_SetArmed(&zdt_vobs,0U,HAL_GetTick());ZDT_Vobs_State();return 1U;
  }
  if(!strcmp(command,"vobs retry")) {
    if(zdt_vobs_phase==2U) {
      if(zdt_vobs_resume_script && zdt_vobs_leave_target>=3U)zdt_vobs_leave_target=0U;
      ZDT_Vobs_Replan();
    }
    else ZDT_Console_Print("[VOBS] retry ignored: no held NO_PATH request\r\n");
    return 1U;
  }
  if(sscanf(command,"vobs %lu %lu %ld %ld %lu %c",&session,&sequence,&front,&right,&age,&extra)!=5) {
    ZDT_Console_Print("[VOBS] usage: vobs on|off|status|retry, or vobs session seq front_gap_mm right_mm age_ms\r\n");return 1U;
  }
  /* Include the MCU console queue time as well as the PC pre-write age. */
  {
    uint32_t queued=HAL_GetTick()-received_tick;
    if(age>VOBS_MAX_AGE_MS || queued>VOBS_MAX_AGE_MS || age+queued>VOBS_MAX_AGE_MS)
      age=VOBS_MAX_AGE_MS+1U;
    else age+=queued;
  }
  reason=VisionObstacle_Accept(&zdt_vobs,(uint32_t)session,(uint32_t)sequence,
      (int32_t)front,(int32_t)right,(uint32_t)age,ZDT_Vobs_Eligible());
  if(!reason && !ZDT_Vobs_FramePose((uint32_t)age,&x,&y))reason="POSE_HISTORY";
  if(!reason && (
      !VisionObstacle_Project(x,y,ZDT_Vobs_WorldYaw(),(float)front,(float)right,&ox,&oy)))reason="OUTSIDE_MAP";
  if(!reason && (lroundf(ox)>=(long)GRID_NAV_FIELD_MM || lroundf(oy)>=(long)GRID_NAV_FIELD_MM))reason="OUTSIDE_MAP";
  if(!reason && ZDT_Vobs_Known((uint16_t)lroundf(ox),(uint16_t)lroundf(oy)))reason="KNOWN";
  leave=ZDT_Vobs_IsLeaveOdo();GridNav_GetPoseMm(&px,&py);
  if(!reason && leave && !ZDT_Vobs_DepartTargets(px,py))reason="LEAVE_TARGET";
  if(reason) {
    (void)snprintf(line,sizeof(line),"[VOBS ACK] session=%lu seq=%lu result=%s\r\n",session,sequence,reason);
    ZDT_Console_Print(line);return 1U;
  }
  zdt_vobs_start_x=(float)px+(leave?zdt_script_dx_total:0);
  zdt_vobs_start_y=(float)py+(leave?zdt_script_dy_total:0);
  zdt_vobs_world_yaw=ZDT_Vobs_WorldYaw();zdt_vobs_counts=Nav_Params()->counts_per_rev;
  zdt_vobs_mm_per_rev=ZDT_MILEAGE_MM_PER_REV;
  memcpy(zdt_vobs_baseline,zdt_odo.baseline,sizeof(zdt_vobs_baseline));
  memcpy(zdt_vobs_previous,zdt_odo.previous,sizeof(zdt_vobs_previous));
  memcpy(zdt_vobs_signs,zdt_mileage_forward_signs,sizeof(zdt_vobs_signs));
  if(leave) {
    zdt_vobs_resume_script=1U;zdt_vobs_leave_target=0U;zdt_vobs_depart_detoured=1U;
    zdt_vobs_goal_x=zdt_vobs_leave_step_x;zdt_vobs_goal_y=zdt_vobs_leave_step_y;
  } else GridNav_GetTargetMm(&zdt_vobs_goal_x,&zdt_vobs_goal_y);
  zdt_vobs_object_x=(uint16_t)lroundf(ox);zdt_vobs_object_y=(uint16_t)lroundf(oy);
  /* Requested avoidance pause: brake first, then diagnose/plan. */
  (void)ChassisMotor_StopImmediate();ZDT_Odo_Cancel();zdt_mileage_query_active=0U;
  zdt_nav_wait_start_query=zdt_nav_wait_end_query=0U;zdt_nav_state=ZDT_NAV_VOBS_STOP;
  zdt_vobs_phase=1U;zdt_vobs_wait=0U;zdt_vobs_sample_tick=0U;
  zdt_vobs_poll_tick=zdt_vobs_next_query=HAL_GetTick();zdt_vobs_state_dirty=1U;
  (void)snprintf(line,sizeof(line),"[VOBS ACK] session=%lu seq=%lu result=ACCEPTED\r\n",session,sequence);
  ZDT_Console_Print(line);return 1U;
}

/* ---------- Serial-stepped competition flow (single ZDT task owner) ---------- */

/* Station alignment remains in the ZDT motion-owner task. Camera task only
 * publishes observations; correction never sends motors from UART interrupts. */
static uint8_t ZDT_Align_DiagBusy(void)
{
  int16_t rpm[4];uint8_t i;chassis_telemetry_t t;
  if(zdt_match_io==MATCH_IO_ALIGN || zdt_script_running || ZDT_Nav_IsActive() ||
     ChassisMotor_IsTurnActive() || zdt_match.state==MATCH_MOVING ||
     zdt_match.state==MATCH_LEAVING || zdt_poscal_phase!=ZDT_POSCAL_IDLE)return 1U;
  ChassisMotor_GetWheelRpm(rpm);
  for(i=0U;i<4U;i++)if(rpm[i])return 1U;
  ChassisMotor_GetTelemetry(&t);
  if(ChassisMotor_HasMotionDemand() || fabsf(t.gyro_dps)>1.5f)return 1U;
  return 0U;
}
static uint8_t ZDT_Camera_ResetBusy(void)
{
  uint8_t i;vision_camera_state_t state;
  for(i=0U;zdt_align_enabled && i<3U;i++)if(Vision_GetCameraState(zdt_align_config[i].camera,&state) && state.reset_pending)
    return 1U;
  if(QR_CAMERA_RUNTIME_ENABLED && zdt_qr_enabled && Vision_GetCameraState(2U,&state) && state.reset_pending)return 1U;
  return 0U;
}
static void ZDT_Camera_Reset(void)
{
  uint8_t i,mask=0U;
  for(i=0U;i<3U;i++)if(zdt_align_config[i].camera>=1U && zdt_align_config[i].camera<=2U)
    mask|=(uint8_t)(1U<<(zdt_align_config[i].camera-1U));
  if(QR_CAMERA_RUNTIME_ENABLED && (zdt_qr_enabled || zdt_qr_used))mask|=2U;
  if(!QR_CAMERA_RUNTIME_ENABLED)mask&=(uint8_t)~2U;
  for(i=1U;i<=2U;i++)if(mask&(1U<<(i-1U))) {
    if(Vision_RequestStop(i)==HAL_OK)zdt_camera_reset_mask|=(uint8_t)(1U<<(i-1U));
  }
  memset(&zdt_camera_gate,0,sizeof(zdt_camera_gate));
}
static void ZDT_Camera_ResetService(void)
{
  uint8_t i;char line[100];vision_camera_state_t state;
  if(!zdt_camera_reset_mask || ZDT_Align_DiagBusy())return;
  for(i=1U;i<=2U;i++)if(zdt_camera_reset_mask&(1U<<(i-1U))) {
    if(!Vision_GetCameraState(i,&state) || state.pending)continue;
    if(!state.reset_pending && state.mode==VISION_MODE_STOP && state.confirmed && state.result==0U) {
      (void)snprintf(line,sizeof(line),"[CAM%u] STOP reset ACK confirmed; old targets cleared.\r\n",i);
    } else if(state.reset_pending && state.result!=255U) {
      (void)snprintf(line,sizeof(line),"[CAM%u] STOP reset not confirmed result=%u; match reset to retry, camstat to inspect.\r\n",i,state.result);
    } else continue;
    zdt_camera_reset_mask&=(uint8_t)~(1U<<(i-1U));ZDT_Console_Print(line);
  }
}
static uint8_t ZDT_Align_DiagPhase(uint8_t stopped)
{
  if(zdt_align_phase==4U)return V_DIAG_ACK;
  if(zdt_align_phase==1U)return V_DIAG_BASE;
  if(zdt_align_phase==3U)return V_DIAG_END;
  if(zdt_align.state==V_ALIGN_PULSE || zdt_align.state==V_ALIGN_TRACK || zdt_align.state==V_ALIGN_SEARCH)return V_DIAG_PULSE;
  if(zdt_align.state==V_ALIGN_SETTLE)return stopped?V_DIAG_SETTLE:V_DIAG_BRAKE;
  return zdt_align.state==V_ALIGN_WAIT?V_DIAG_WAIT:V_DIAG_END;
}
static void ZDT_Align_DiagObserve(uint32_t now,const vision_pixel_t *p,uint32_t age,
                                  float forward,float right,float omega)
{
  vision_diag_input_t s={0};chassis_telemetry_t t;vision_camera_state_t camera={0};uint8_t i;
  if(!zdt_align_diag.active)return;
  zdt_align_diag.can_failures=zdt_can_enqueue_failures-zdt_diag_can_start;
  ChassisMotor_GetTelemetry(&t);ChassisMotor_GetWheelRpm(s.rpm);
  s.stopped=(uint8_t)(fabsf(t.gyro_dps)<=1.5f);
  for(i=0U;i<4U;i++)if(s.rpm[i])s.stopped=0U;
  (void)Vision_GetCameraState(zdt_camera_gate.camera,&camera);
  s.context=VisionModeGate_Matches(&zdt_camera_gate,&camera);
  s.now=now;s.age=age;s.pixel=p;s.forward=forward;s.right=right;s.omega=omega;
  s.yaw=t.heading_deg;s.gyro=t.gyro_dps;s.phase=ZDT_Align_DiagPhase(s.stopped);
  if((zdt_align.state==V_ALIGN_TRACK || zdt_align.state==V_ALIGN_SEARCH) && forward==0.0f && right==0.0f)
    s.phase=V_DIAG_WAIT;
  s.state=(uint8_t)zdt_align.state;s.stable=zdt_align.stable_count;
  s.scene_count=zdt_align.config.type==VISION_TYPE_PLATE_PX?zdt_align_plate_scene.status:zdt_align_scene.count;
  s.scene_valid=zdt_align.config.type==VISION_TYPE_PLATE_PX?zdt_align_plate_scene.valid:zdt_align_scene.valid;
  VisionDiag_Observe(&zdt_align_diag,&s);
}
static void ZDT_Align_DiagService(void)
{
  char line[384];uint32_t now=HAL_GetTick();
  if(ZDT_Align_DiagBusy()) {
    if(zdt_diag_dump_active){zdt_diag_dump_active=0U;zdt_diag_dump_interrupted=1U;}
    return;
  }
  if(zdt_diag_dump_interrupted) {
    zdt_diag_dump_interrupted=0U;
    ZDT_Console_Print("[VDIAG] dump interrupted by motion; export again while stopped.\r\n");
    return;
  }
  if(!zdt_diag_dump_active || (int32_t)(now-zdt_diag_dump_tick)<0)return;
  if(VisionDiag_Line(&zdt_align_diag,zdt_diag_dump_index,line,sizeof(line))) {
    ZDT_Console_Print(line);zdt_diag_dump_index++;
    zdt_diag_dump_tick=HAL_GetTick()+5U; /* One bounded line per poll, idle only. */
  } else zdt_diag_dump_active=0U;
}
static uint8_t ZDT_Align_DiagCommand(const char *command)
{
  const char *arg;char line[160];
  if(strncmp(command,"valign trace",12U)!=0 || (command[12] && command[12]!=' '))return 0U;
  arg=ZDT_SkipSpaces(command+12U);
  if(ZDT_Align_DiagBusy()) {
    ZDT_Console_Print("[VDIAG] trace commands require idle chassis / WAIT_OK.\r\n");return 1U;
  }
  if(strcmp(arg,"dump")==0) {
    if(zdt_align_diag.active || !zdt_align_diag.has_data) {
      ZDT_Console_Print("[VDIAG] no frozen alignment; enable trace before the next station.\r\n");return 1U;
    }
    zdt_diag_dump_index=0U;zdt_diag_dump_tick=HAL_GetTick();zdt_diag_dump_active=1U;
  } else if(strcmp(arg,"on")==0 || strcmp(arg,"off")==0) {
    zdt_align_diag.enabled=(uint8_t)(strcmp(arg,"on")==0);
  } else if(strcmp(arg,"clear")==0) {
    uint8_t enabled=zdt_align_diag.enabled;
    zdt_diag_dump_active=0U;VisionDiag_Init(&zdt_align_diag);zdt_align_diag.enabled=enabled;
  } else if(strcmp(arg,"cancel")==0)zdt_diag_dump_active=0U;
  else if(*arg && strcmp(arg,"status")!=0) {
    ZDT_Console_Print("[VDIAG] valign trace on|off|status|dump|clear|cancel\r\n");return 1U;
  }
  (void)snprintf(line,sizeof(line),"[VDIAG] DIAG_V2 enabled=%u active=%u frozen=%u station=%u rows=%u/%u dropped=%lu dump=%u RAM=%lu\r\n",
    zdt_align_diag.enabled,zdt_align_diag.active,
    (unsigned int)(zdt_align_diag.has_data && !zdt_align_diag.active),zdt_align_diag.station,
    zdt_align_diag.count,V_ALIGN_DIAG_CAPACITY,(unsigned long)zdt_align_diag.overflow,
    zdt_diag_dump_active,(unsigned long)sizeof(zdt_align_diag));
  ZDT_Console_Print(line);return 1U;
}
static uint8_t ZDT_Align_UsesYaw(uint8_t station)
{return (uint8_t)(zdt_vyaw.enabled && (station==2U || station==3U));}
static uint8_t ZDT_Align_Flags(uint8_t mode,uint8_t station)
{
  uint8_t flags=0U;
  if(mode==VISION_MODE_RING && station>=2U && station<=3U) {
    if(ZDT_Align_UsesYaw(station))flags|=VISION_RING_AXIS_FLAG;
    if(zdt_ring_profile==2U || (!zdt_manual_align_station && !zdt_ring_profile && station==3U && zdt_match.checkpoint==MATCH_TEMP2))flags|=VISION_RING_OCCUPIED_FLAG;
  }
  return flags;
}
static void ZDT_Align_RequestCameraMode(uint8_t station)
{
  vision_align_config_t *c=&zdt_align_config[station-1U];
  uint8_t mode=c->type==VISION_TYPE_PLATE_PX?VISION_MODE_PLATE:VISION_MODE_RING;
  char line[128];
  VisionModeGate_Start(&zdt_camera_gate,c->camera,mode,station,c->id,HAL_GetTick());
  zdt_camera_gate.flags=ZDT_Align_Flags(mode,station);
  (void)snprintf(line,sizeof(line),"[CAM%u] request %s station=%u target=%u flags=%u profile=%s; waiting ACK\r\n",
    c->camera,mode==VISION_MODE_PLATE?"PLATE":"RING",station,c->id,zdt_camera_gate.flags,
    (zdt_camera_gate.flags&VISION_RING_OCCUPIED_FLAG)?"occupied":"fast");
  ZDT_Console_Print(line);
}
static vision_mode_action_t ZDT_Camera_ModeStep(void)
{
  vision_camera_state_t state={0};vision_mode_action_t action;
  if(!Vision_GetCameraState(zdt_camera_gate.camera,&state))
    return VisionModeGate_Step(&zdt_camera_gate,HAL_GetTick(),NULL);
  action=VisionModeGate_Step(&zdt_camera_gate,HAL_GetTick(),&state);
  if(action==V_MODE_SEND) {
    HAL_StatusTypeDef result=Vision_RequestMode(zdt_camera_gate.camera,zdt_camera_gate.mode,
      zdt_camera_gate.station,zdt_camera_gate.target,zdt_camera_gate.hz,zdt_camera_gate.flags);
    if(result==HAL_OK) {
      if(Vision_GetCameraState(zdt_camera_gate.camera,&state))VisionModeGate_Sent(&zdt_camera_gate,state.request_seq);
      else {zdt_camera_gate.action=V_MODE_FAILED;zdt_camera_gate.reason="camera state unavailable after SET";}
    } else if(result!=HAL_BUSY) {
      zdt_camera_gate.action=V_MODE_FAILED;zdt_camera_gate.reason="camera SET enqueue rejected";
    }
    return zdt_camera_gate.action;
  }
  return action;
}
static void ZDT_Camera_FailureDetails(void)
{
  vision_camera_state_t state={0};vision_port_stats_t stats={0};char line[192];
  (void)Vision_GetCameraState(zdt_camera_gate.camera,&state);
  (void)Vision_GetPortStats(zdt_camera_gate.camera,&stats);
  (void)snprintf(line,sizeof(line),"[CAM%u] wanted mode=%u station=%u target=%u; actual mode=%u station=%u result=%u attempts=%u HBseen=%u HBage=%lums RX=%lu parse=%lu\r\n",
    zdt_camera_gate.camera,zdt_camera_gate.mode,zdt_camera_gate.station,zdt_camera_gate.target,
    state.mode,state.station,state.result,state.attempts,state.heartbeat_seen,
    (unsigned long)state.heartbeat_age_ms,(unsigned long)stats.rx_bytes,
    (unsigned long)Vision_GetParseErrorCount());
  ZDT_Console_Print(line);
  (void)snprintf(line,sizeof(line),"[CAM%u] lost-confirm detail: gate_sent=%u wanted seq=%u actual seq=%u confirmed=%u pending=%u reset=%u target=%u hz=%u flags=%u HBver=%u UARTinit=%u protocol=%u\r\n",
    zdt_camera_gate.camera,zdt_camera_gate.sent,zdt_camera_gate.request_seq,state.request_seq,state.confirmed,state.pending,state.reset_pending,
    state.target,state.hz,state.flags,state.protocol_version,stats.init_status,stats.protocol);ZDT_Console_Print(line);
  (void)snprintf(line,sizeof(line),"[CAM%u] UART last_rx_age=%lums queued=%u dropped=%lu errors=%lu recoveries=%lu HBmode=%u HBstation=%u peer_age=%lums\r\n",
    zdt_camera_gate.camera,(unsigned long)(HAL_GetTick()-stats.last_rx_tick),stats.queued_bytes,
    (unsigned long)stats.dropped_bytes,(unsigned long)stats.uart_errors,(unsigned long)stats.recoveries,
    state.heartbeat_mode,state.heartbeat_station,(unsigned long)state.peer_age_ms);ZDT_Console_Print(line);
}
static uint8_t ZDT_Qr_Start(uint8_t for_match)
{
#if !QR_CAMERA_RUNTIME_ENABLED
  (void)for_match;return 0U;
#else
  vision_port_stats_t port={0};
  if(zdt_qr_active || (!for_match && ZDT_Align_DiagBusy()))return 0U;
  if(!Vision_GetPortStats(2U,&port) || port.init_status!=HAL_OK || (port.protocol!=VISION_PROTOCOL_BINARY8 &&
      !Vision_SetCameraProtocol(2U,VISION_PROTOCOL_BINARY8)))return 0U;
  ScreenMetrics_ClearTask();ScreenMetrics_SetState(SCREEN_WAIT_QR);
  QrScan_Start(&zdt_qr_scan,HAL_GetTick());zdt_qr_active=zdt_qr_used=1U;
  zdt_qr_for_match=for_match;zdt_qr_locked=zdt_qr_printed=0U;
  if(for_match==1U)zdt_match_io=MATCH_IO_QRSCAN; /* 2=background transport; retain chassis owner. */
  ZDT_Console_Print("[QR] CAM2 QRSCAN_V1: new SET6 station4, then full task, STOP ACK; no obstacle detection.\r\n");return 1U;
#endif
}
static void ZDT_Qr_Cancel(void)
{
  if(QR_CAMERA_RUNTIME_ENABLED && (zdt_qr_active || zdt_qr_used))(void)Vision_RequestStop(2U);
  QrScan_Cancel(&zdt_qr_scan);zdt_qr_active=zdt_qr_for_match=zdt_qr_locked=zdt_qr_printed=0U;
}
static void ZDT_Qr_Service(void)
{
  vision_camera_state_t state={0};vision_qr_task_t task={0};qr_scan_action_t action;HAL_StatusTypeDef result;
  if(!zdt_qr_active)return;
  /* Starting scan early must not spend the stopped QR read budget in transit. */
  if(zdt_qr_for_match==2U && zdt_qr_scan.phase==QR_SCAN_READ)zdt_qr_scan.read_started=HAL_GetTick();
  (void)Vision_GetCameraState(2U,&state);(void)Vision_GetQrTask(&task,NULL);
  action=QrScan_Step(&zdt_qr_scan,HAL_GetTick(),&state,task.observation_id?&task:NULL);
  if(zdt_qr_scan.phase==QR_SCAN_STOP && !zdt_qr_printed) {
    char line[180];zdt_qr_task=zdt_qr_scan.task;__DMB();zdt_qr_locked=zdt_qr_printed=1U;
    (void)snprintf(line,sizeof(line),"[QR] LOCK raw=%s session=%u; wire colors=%u%u%u/%u%u%u positions=%u%u%u/%u%u%u; waiting STOP ACK\r\n",
      zdt_qr_task.raw,zdt_qr_task.request_seq,zdt_qr_task.colors[0][0],zdt_qr_task.colors[0][1],zdt_qr_task.colors[0][2],
      zdt_qr_task.colors[1][0],zdt_qr_task.colors[1][1],zdt_qr_task.colors[1][2],
      zdt_qr_task.positions[0][0],zdt_qr_task.positions[0][1],zdt_qr_task.positions[0][2],
      zdt_qr_task.positions[1][0],zdt_qr_task.positions[1][1],zdt_qr_task.positions[1][2]);ZDT_Console_Print(line);
  }
  if(action==QR_SCAN_SEND_MODE) {
    result=Vision_RequestMode(2U,VISION_MODE_QR,4U,0U,20U,0U);
    if(result==HAL_OK){(void)Vision_GetCameraState(2U,&state);QrScan_ModeSent(&zdt_qr_scan,state.request_seq);}
    else if(result!=HAL_BUSY){zdt_qr_scan.phase=QR_SCAN_FAILED;zdt_qr_scan.reason="CAM2 QR SET enqueue rejected";}
  } else if(action==QR_SCAN_SEND_STOP) {
    result=Vision_RequestStop(2U);
    if(result==HAL_OK)QrScan_StopSent(&zdt_qr_scan);
    else if(result!=HAL_BUSY){zdt_qr_scan.phase=QR_SCAN_FAILED;zdt_qr_scan.reason="CAM2 QR STOP enqueue rejected";}
  } else if(action==QR_SCAN_READY) {
    uint8_t for_match=zdt_qr_for_match;zdt_qr_active=zdt_qr_for_match=0U;
    ZDT_Console_Print("[QR] task latched; CAM2 STOP ACK confirmed.\r\n");
    if(for_match && !VisionQr_CompetitionTaskValid(&zdt_qr_task)) {
      zdt_qr_locked=0U;zdt_match_io=MATCH_IO_NONE;
      ZDT_Match_Complete(0U,"QR mission requires SAME three colors and ring permutations");
    } else {
      (void)ScreenMetrics_SetTask(zdt_qr_task.raw);
      if(for_match==1U){zdt_match_io=MATCH_IO_NONE;ZDT_Match_Complete(1U,NULL);}
    else if(for_match==2U)ZDT_Console_Print("[QR] captured during transport; cached task/STOP ACK, route continues to QR pose\r\n");
    }
  }
  if(zdt_qr_scan.phase==QR_SCAN_FAILED) {
    uint8_t for_match=zdt_qr_for_match;const char *reason=zdt_qr_scan.reason;
    zdt_qr_active=zdt_qr_for_match=zdt_qr_locked=0U;(void)Vision_RequestStop(2U);
    ZDT_Console_Print("[QR] failed: ");ZDT_Console_Print(reason);ZDT_Console_Print("\r\n");
    if(for_match)ZDT_Match_Complete(0U,reason);
  }
}
static uint8_t ZDT_Qr_Command(const char *command)
{
  if(strncmp(command,"qrscan",6U) || (command[6] && command[6]!=' '))return 0U;
#if !QR_CAMERA_RUNTIME_ENABLED
    zdt_qr_enabled=zdt_qr_active=zdt_qr_for_match=zdt_qr_used=zdt_qr_locked=0U;
    ZDT_Console_Print("[QR] CAM2 temporarily disabled in this firmware; QR checkpoint uses manual OK; CAM1 retained\r\n");return 1U;
#else
  char line[140];
  if(strcmp(command,"qrscan")==0 || strcmp(command,"qrscan status")==0) {
    (void)snprintf(line,sizeof(line),"[QR] enabled=%u active=%u phase=%u locked=%u raw=%s; CAM2 USART2; qrscan on|off|start|status\r\n",
      zdt_qr_enabled,zdt_qr_active,zdt_qr_scan.phase,zdt_qr_locked,zdt_qr_locked?zdt_qr_task.raw:"UNKNOWN");
    ZDT_Console_Print(line);return 1U;
  }
  if(strcmp(command,"qrscan off")==0) {
    uint8_t for_match=zdt_qr_for_match;zdt_qr_enabled=0U;zdt_match.qr_required=0U;ZDT_Qr_Cancel();
    if(for_match)ZDT_Match_Complete(0U,"QR scan canceled by operator");
    ZDT_Console_Print("[QR] automatic scan OFF; manual QR checkpoint remains.\r\n");return 1U;
  }
  if(ZDT_Align_DiagBusy() || zdt_qr_active){ZDT_Console_Print("[QR] configure/start only with stopped chassis.\r\n");return 1U;}
  if(strcmp(command,"qrscan on")==0) {
    vision_port_stats_t port={0};
    if(!Vision_GetPortStats(2U,&port) || port.init_status!=HAL_OK || (port.protocol!=VISION_PROTOCOL_BINARY8 &&
        !Vision_SetCameraProtocol(2U,VISION_PROTOCOL_BINARY8))) {
      ZDT_Console_Print("[QR] CAM2 protocol switch busy; retry when UART TX completes.\r\n");return 1U;
    }
    zdt_qr_enabled=zdt_qr_used=1U;zdt_match.qr_required=1U;zdt_qr_locked=0U;(void)Vision_RequestStop(2U);
    ZDT_Console_Print("[QR] automatic scan ON: connect CAM2, match reset/startpose before a new flow.\r\n");
  } else if(strcmp(command,"qrscan start")==0) {
    if(!ZDT_Qr_Start(0U))ZDT_Console_Print("[QR] scan start rejected.\r\n");
  } else ZDT_Console_Print("[QR] qrscan on|off|start|status\r\n");return 1U;
#endif
}
static void ZDT_Camera_Service(void)
{
  vision_mode_action_t action;
  if(zdt_match_io!=MATCH_IO_CAMERA)return;
  action=ZDT_Camera_ModeStep();
  if(action==V_MODE_FAILED){ZDT_Camera_FailureDetails();ZDT_Match_Complete(0U,zdt_camera_gate.reason);return;}
  if(action==V_MODE_READY) {
    ZDT_Console_Print("[CAM] mode ACK confirmed\r\n");ZDT_Match_Complete(1U,NULL);
  }
}
static uint8_t ZDT_Align_Station(void)
{
  if(zdt_manual_align_station)return zdt_manual_align_station;
  switch(zdt_match.checkpoint) {
    case MATCH_RAW1:case MATCH_RAW2:return 1U;
    case MATCH_ROUGH1:case MATCH_ROUGH2:return 2U;
    case MATCH_TEMP1:case MATCH_TEMP2:return 3U;
    default:return 0U;
  }
}
static uint8_t ZDT_Align_ConfigForStage(uint8_t station,vision_align_config_t *out)
{
  if(!out || station<1U || station>3U)return 0U;
  *out=zdt_align_config[station-1U];
  if(!zdt_manual_align_station && station==1U && out->type==VISION_TYPE_PLATE_PX &&
     (zdt_match.checkpoint==MATCH_RAW1 || zdt_match.checkpoint==MATCH_RAW2)) {
    /* Fixed mission approach sides, independent of the last avoidance move. */
    out->plate_search_right=(uint8_t)(zdt_match.checkpoint==MATCH_RAW2);
  }
  if(!zdt_manual_align_station && station==3U && zdt_match.checkpoint==MATCH_TEMP2 && out->type==VISION_TYPE_RING_PX) {
    const float stacking_tolerance_px=V_ALIGN_STACKED_TOLERANCE_PX;
    if(out->tolerance_px<stacking_tolerance_px)out->tolerance_px=stacking_tolerance_px;
    out->stable_frames=3U; /* Last occupied stacking pass only; retain stable_ms. */
  }
  return VisionAlign_ConfigValid(out);
}
static uint8_t ZDT_Align_Start(uint8_t station)
{
  char line[160];vision_align_config_t c;
  if(!ZDT_Align_ConfigForStage(station,&c) ||
     !ChassisMotor_IsEnabled() || !HWT101_IsZeroDriftReady())return 0U;
  if(ZDT_Align_UsesYaw(station) && (zdt_align_config[station-1U].camera!=1U ||
      zdt_align_config[station-1U].type!=VISION_TYPE_RING_PX || zdt_align_config[station-1U].id!=2U)) {
    ZDT_Console_Print("[VYAW] calibration requires CAM1 RING ID2 at stations2/3\r\n");return 0U;
  }
  zdt_align_heading=ZDT_Nav_GetHeading();
  if(!isfinite(zdt_align_heading))return 0U;
  zdt_temp2_still_tracking=zdt_temp2_skip_align=0U;
  memset(&zdt_align_scene,0,sizeof(zdt_align_scene));
  memset(&zdt_align_plate_scene,0,sizeof(zdt_align_plate_scene));
  zdt_align_status_pending=0U;zdt_align_status_tick=HAL_GetTick()-1000U;
  if(zdt_diag_dump_active){zdt_diag_dump_active=0U;zdt_diag_dump_interrupted=1U;}
  VisionDiag_Begin(&zdt_align_diag,station,&c,HAL_GetTick());
  zdt_diag_can_start=zdt_can_enqueue_failures;
  zdt_align_imu_count=HWT101_GetSampleCount();zdt_align_imu_tick=HAL_GetTick();
  ZDT_Align_RequestCameraMode(station);
  zdt_align_phase=4U;zdt_align_tick=HAL_GetTick();zdt_match_io=MATCH_IO_ALIGN;
  VisionAlign_Cancel(&zdt_align);
  (void)ChassisMotor_Stop();
  (void)snprintf(line,sizeof(line),"[ALIGN] station %u CAM%u %s id=%u tol=%.1fpx stable=%uframes; ACK then settle then pixels\r\n",
      station,zdt_align_config[station-1U].camera,
      zdt_align_config[station-1U].type==VISION_TYPE_PLATE_PX?"PLATE":"RING",
      zdt_align_config[station-1U].id,(double)c.tolerance_px,c.stable_frames);
  ZDT_Console_Print(line);
  if(c.type==VISION_TYPE_PLATE_PX && c.search_axis) {
    (void)snprintf(line,sizeof(line),"[ALIGN PLATE SEARCH] checkpoint=%u image=%s speed=%.3f; nominal approach, ignore detour direction\r\n",
        (unsigned)zdt_match.checkpoint,c.plate_search_right?"RIGHT":"LEFT",(double)c.search_speed);ZDT_Console_Print(line);
  }
  return 1U;
}
static uint8_t ZDT_Align_Temp2StationaryReady(uint32_t now,uint8_t stopped)
{
  if(zdt_manual_align_station || zdt_match.checkpoint!=MATCH_TEMP2 || zdt_align_phase!=2U ||
     zdt_align.config.type!=VISION_TYPE_RING_PX || !stopped) {
    zdt_temp2_still_tracking=0U;return 0U;
  }
  if(!zdt_temp2_still_tracking) {zdt_temp2_still_tracking=1U;zdt_temp2_still_tick=now;return 0U;}
  if(now-zdt_temp2_still_tick<V_ALIGN_TEMP2_STILL_DROP_MS)return 0U;
  zdt_temp2_still_tracking=0U;zdt_temp2_skip_align=1U;return 1U;
}
static void ZDT_Align_OnMileage(uint8_t mask)
{
  uint32_t now=HAL_GetTick();
  VisionDiag_Wheels(&zdt_align_diag,zdt_mileage_query_position,mask,(uint8_t)(zdt_align_phase==3U));
  if(mask!=0x0FU){ZDT_Match_Complete(0U,"alignment needs 4 fresh wheel replies");return;}
  if(zdt_align_phase==1U) {
    vision_pixel_t p={0};uint32_t age;
    uint8_t station=ZDT_Align_Station();
    vision_align_config_t config,*c;
    if(station<1U || station>3U){ZDT_Match_Complete(0U,"alignment station invalid");return;}
    if(!ZDT_Align_ConfigForStage(station,&config)){ZDT_Match_Complete(0U,"alignment config invalid");return;}
    c=&config;
    memcpy(zdt_align_wheel_start,zdt_mileage_query_position,sizeof(zdt_align_wheel_start));
    zdt_align_measure_heading=ZDT_Nav_GetHeading();
    (void)Vision_GetPixel(c->camera,c->type,&p,&age);
    if(!VisionAlign_Start(&zdt_align,c,now,p.observation_id)) {
      ZDT_Match_Complete(0U,"alignment config invalid");return;
    }
    {vision_camera_state_t camera={0};(void)Vision_GetCameraState(c->camera,&camera);
     VisionYaw_Cancel(&zdt_vyaw);
     if(ZDT_Align_UsesYaw(station))(void)VisionYaw_Begin(&zdt_vyaw,station,camera.request_seq,now,zdt_align_heading);}
    zdt_align_phase=2U;
  } else if(zdt_align_phase==3U) {
    float wheels[4],forward,right;uint16_t x,y;uint8_t i,station=ZDT_Align_Station();char line[192];
    for(i=0U;i<4U;i++)wheels[i]=(float)((double)((int64_t)zdt_mileage_query_position[i]-
        zdt_align_wheel_start[i])*zdt_mileage_forward_signs[i]*ZDT_MILEAGE_MM_PER_REV/
        Nav_Params()->counts_per_rev);
    forward=(wheels[0]+wheels[1]+wheels[2]+wheels[3])*0.25f;
    right=(wheels[0]-wheels[1]-wheels[2]+wheels[3])*0.25f;
    if(!isfinite(forward) || !isfinite(right)) {
      ZDT_Match_Complete(0U,"alignment wheel feedback invalid");return;
    }
    if(zdt_temp2_skip_align) {
      float dx,dy,heading=zdt_align_measure_heading+0.5f*NavBlend_AngleError(ZDT_Nav_GetHeading(),zdt_align_measure_heading);
      GridNav_GetPoseMm(&x,&y);
      NavFrame_BodyToMap((float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,heading,forward,right,&dx,&dy);
      GridNav_SetPoseMm((uint16_t)lroundf((float)x+dx),(uint16_t)lroundf((float)y+dy));
      GridNav_GetPoseMm(&x,&y);zdt_nav_post_align=1U;zdt_temp2_skip_align=0U;zdt_align_phase=0U;
      (void)snprintf(line,sizeof(line),"[ALIGN] TEMP2 stationary2s BYPASS complete; odometry estimate=(%u,%u), no visual anchor rebase; dispatch normal placement\r\n",x,y);
      ZDT_Console_Print(line);ZDT_Match_Complete(1U,NULL);return;
    }
    /* User-provided aligned car-centre poses supersede accumulated wheel
     * odometry after stable vision and four final wheel replies. */
    if(!VisionAlign_StationPoseMm(station,&x,&y)){ZDT_Match_Complete(0U,"alignment station has no anchor");return;}
    if(station>=2U && zdt_vyaw.phase==V_YAW_DONE && zdt_vyaw.station==station) {
      float before=ZDT_Nav_GetHeading();
      float expected=ZDT_Zone2_RelYaw(station==2U?180.0f:90.0f);
      if(ChassisMotor_RebaseHeading(expected)!=HAL_OK) {
        ZDT_Match_Complete(0U,"visual heading reference needs stopped wheels/valid IMU");return;
      }
      zdt_align_heading=ZDT_Nav_GetHeading();
      (void)snprintf(line,sizeof(line),"[VYAW] REFERENCE station=%u before=%+.2f map=%+.2f now=%+.2f delta=%+.2fdeg; following route uses this heading\r\n",
          station,(double)before,(double)expected,(double)zdt_align_heading,
          (double)NavBlend_AngleError(zdt_align_heading,before));ZDT_Console_Print(line);
    }
    GridNav_SetPoseMm(x,y);
    zdt_nav_post_align=1U;
    (void)snprintf(line,sizeof(line),"[ALIGN] done dx=%+.1f dy=%+.1f px, wheel delta F%+.1f R%+.1f mm\r\n",
        (double)zdt_align.error_x,(double)zdt_align.error_y,(double)forward,(double)right);
    ZDT_Console_Print(line);
    (void)snprintf(line,sizeof(line),"[ALIGN] ANCHOR station=%u pose=(%umm,%umm)\r\n",station,x,y);
    ZDT_Console_Print(line);
    if(station==1U) {
      ZDT_Console_Print("[ALIGN] PLATE aligned; post-align approach OFF; keep aligned anchor, loading starts NOW\r\n");
    }
    zdt_align_phase=0U;
    ZDT_Match_Complete(1U,NULL);
  }
}
static void ZDT_Align_ReadPixels(const vision_align_config_t *c,vision_pixel_t *p,
                                vision_ring_count_t *ring,vision_plate_scene_t *plate,uint32_t *age)
{
  if((c->search_axis || zdt_vyaw.phase!=V_YAW_IDLE) && c->type==VISION_TYPE_RING_PX)
    (void)Vision_GetRingSceneWindow(c->camera,c->observation_ms,p,ring,age);
  else if(c->search_axis && c->type==VISION_TYPE_PLATE_PX &&
      Vision_GetPlateSceneWindow(c->camera,c->observation_ms,p,plate,age))return;
  else (void)Vision_GetPixelWindow(c->camera,c->type,c->observation_ms,p,age);
}
static void ZDT_VyawObserve(void)
{
  vision_ring_axis_t axis={0};uint32_t age=UINT32_MAX,samples=HWT101_GetSampleCount(),now=HAL_GetTick();
  chassis_telemetry_t t;int16_t rpm[4];uint8_t i,stopped=1U;
  if(samples!=zdt_align_imu_count){zdt_align_imu_count=samples;zdt_align_imu_tick=now;}
  ChassisMotor_GetTelemetry(&t);ChassisMotor_GetWheelRpm(rpm);
  for(i=0;i<4;i++)if(rpm[i])stopped=0U;
  if(ChassisMotor_HasMotionDemand() || !HWT101_IsZeroDriftReady() || now-zdt_align_imu_tick>150U ||
     !isfinite(t.gyro_dps) || fabsf(t.gyro_dps)>0.5f)stopped=0U;
  if(!HWT101_IsZeroDriftReady() || now-zdt_align_imu_tick>150U)zdt_vyaw.cal[0].sample_valid=zdt_vyaw.cal[1].sample_valid=0U;
  (void)Vision_GetRingAxisWindow(1U,V_YAW_FRESH_MS,&axis,&age);
  VisionYaw_Observe(&zdt_vyaw,&axis,age,HAL_GetTick(),stopped);
}
static uint8_t ZDT_VyawCommand(const char *command)
{
  unsigned int station;float gain;char extra,line[250];vision_ring_axis_t axis={0};uint32_t age=UINT32_MAX;
  chassis_telemetry_t t;uint8_t ok=0U,i;float current_error=NAN;
  if(strncmp(command,"vyaw",4U) || (command[4] && command[4]!=' '))return 0U;
  if(ZDT_Align_DiagBusy() || (MatchControl_IsActive(&zdt_match) && zdt_match.state!=MATCH_WAIT_OK)) {
    if(zdt_match_io==MATCH_IO_ALIGN && (!strcmp(command,"vyaw") || !strcmp(command,"vyaw status"))) {
      zdt_align_status_pending|=1U;return 1U; /* Printed at the next genuinely stopped interval. */
    }
    ZDT_Console_Print("[VYAW] command requires stopped chassis outside an active correction; STOP/reset first\r\n");return 1U;
  }
  ZDT_VyawObserve();ChassisMotor_GetTelemetry(&t);
  (void)Vision_GetRingAxisWindow(1U,V_YAW_FRESH_MS,&axis,&age);
  if(strcmp(command,"vyaw on")==0 || strcmp(command,"vyaw off")==0) {
    ok=VisionYaw_Enable(&zdt_vyaw,(uint8_t)(strcmp(command,"vyaw on")==0));
    ZDT_Console_Print(ok?"[VYAW] switch applied; no motor enable/movement; RAM calibration kept\r\n":
      "[VYAW] ON refused: both station2/3 need zero AND signed gain calibration\r\n");
  } else if(sscanf(command,"vyaw view %u %c",&station,&extra)==1 && station>=2U && station<=3U) {
    HAL_StatusTypeDef result=Vision_RequestMode(1U,VISION_MODE_RING,(uint8_t)station,2U,20U,VISION_RING_AXIS_FLAG);
    (void)snprintf(line,sizeof(line),"[VYAW] view station=%u HAL=%u; camera flags1 only, no chassis movement; wait 3 stable frames\r\n",station,(unsigned int)result);
    ZDT_Console_Print(line);return 1U;
  } else if(sscanf(command,"vyaw zero %u %c",&station,&extra)==1 && station>=2U && station<=3U) {
    ok=(uint8_t)(axis.valid && axis.station==station && age<V_YAW_FRESH_MS && HWT101_IsZeroDriftReady() &&
        isfinite(t.heading_deg) && isfinite(t.gyro_dps) && fabsf(t.gyro_dps)<=0.5f &&
        VisionYaw_Zero(&zdt_vyaw,(uint8_t)station,HAL_GetTick()));
    ZDT_Console_Print(ok?"[VYAW] zero recorded in RAM; car must have been physically square\r\n":
      "[VYAW] zero needs view station, 3 fresh stable axes/conf>=80, and stopped car\r\n");
  } else if(sscanf(command,"vyaw gain %u %f %c",&station,&gain,&extra)==2 && station>=2U && station<=3U) {
    ok=VisionYaw_Gain(&zdt_vyaw,(uint8_t)station,gain);
    ZDT_Console_Print(ok?"[VYAW] signed gain explicitly set in RAM\r\n":"[VYAW] gain needs signed |value| in [0.25,4]\r\n");
  } else if(sscanf(command,"vyaw sample %u %c",&station,&extra)==1 && station>=2U && station<=3U) {
    ok=(uint8_t)(axis.valid && axis.station==station && age<V_YAW_FRESH_MS && HWT101_IsZeroDriftReady() &&
        isfinite(t.heading_deg) && isfinite(t.gyro_dps) && fabsf(t.gyro_dps)<=0.5f &&
        VisionYaw_Sample(&zdt_vyaw,(uint8_t)station,HAL_GetTick(),t.heading_deg));
    ZDT_Console_Print(ok?"[VYAW] baseline captured; manually rotate car 1..8deg, stop and wait 3 frames, then vyaw solve station\r\n":
      "[VYAW] sample needs taught zero, view station and fresh stable axis/IMU\r\n");
  } else if(sscanf(command,"vyaw solve %u %c",&station,&extra)==1 && station>=2U && station<=3U) {
    ok=(uint8_t)(axis.valid && axis.station==station && age<V_YAW_FRESH_MS && HWT101_IsZeroDriftReady() &&
        isfinite(t.heading_deg) && isfinite(t.gyro_dps) && fabsf(t.gyro_dps)<=0.5f &&
        VisionYaw_Solve(&zdt_vyaw,(uint8_t)station,HAL_GetTick(),t.heading_deg));
    ZDT_Console_Print(ok?"[VYAW] signed gain solved=-IMU_delta/line_axis_delta; RAM only\r\n":
      "[VYAW] solve needs same view/session, 3 fresh stable frames, IMU turn 1..8deg and axis change>=0.5deg\r\n");
  } else if(strcmp(command,"vyaw") && strcmp(command,"vyaw status")) {
    ZDT_Console_Print("[VYAW] on|off|status; view/zero/sample/solve 2|3; gain 2|3 <signed_ratio>\r\n");return 1U;
  }
  (void)Vision_GetRingAxisWindow(1U,V_YAW_FRESH_MS,&axis,&age);
  if(axis.valid && axis.station>=2U && axis.station<=3U && zdt_vyaw.cal[axis.station-2U].zero_valid &&
     zdt_vyaw.cal[axis.station-2U].gain_valid)
    current_error=zdt_vyaw.cal[axis.station-2U].gain*VisionYaw_LineError(axis.axis_deg-zdt_vyaw.cal[axis.station-2U].zero);
  (void)snprintf(line,sizeof(line),"[VYAW] enabled=%u phase=%u axis=%+.2f valid=%u conf=%u station=%u age=%lums raw_error=%+.2f last_error=%+.2f travel=%.2fdeg\r\n",
    zdt_vyaw.enabled,(unsigned int)zdt_vyaw.phase,(double)axis.axis_deg,axis.valid,axis.confidence,axis.station,
    (unsigned long)age,(double)current_error,(double)zdt_vyaw.error,(double)zdt_vyaw.total);ZDT_Console_Print(line);
  for(i=0;i<2;i++) {
    vision_yaw_cal_t *c=&zdt_vyaw.cal[i];
    (void)snprintf(line,sizeof(line),"[VYAW] station=%u zero=%+.3f/%u gain=%+.4f/%u stable=%u sample=%u mean=%+.3f baseline_axis=%+.3f baseline_IMU=%+.3f\r\n",
      i+2U,(double)c->zero,c->zero_valid,(double)c->gain,c->gain_valid,c->stable,c->sample_valid,
      (double)c->mean,(double)c->sample_axis,(double)c->sample_heading);ZDT_Console_Print(line);
  }
  return 1U;
}
/* V171 restores V155: acquire/turn yaw with translation stopped. */
static uint8_t ZDT_Align_SearchWithYaw(const vision_align_t *align,const vision_yaw_t *yaw)
{
  (void)align;(void)yaw;return 0U;
}
static void ZDT_Align_ResumeCombinedXY(vision_align_t *align,const vision_yaw_t *yaw,
    const vision_pixel_t *p,uint32_t age,uint32_t now)
{
  if(yaw->phase!=V_YAW_COMBINED || (align->state!=V_ALIGN_DONE && align->state!=V_ALIGN_SETTLE) || !p->valid ||
     !p->observation_id || p->type!=align->config.type || p->id!=align->config.id ||
     age>=align->config.observation_ms || !isfinite(p->dx_px) || !isfinite(p->dy_px))return;
  if(fabsf(p->dx_px)>align->config.tolerance_px || fabsf(p->dy_px)>align->config.tolerance_px) {
    align->state=V_ALIGN_WAIT;align->stable_count=0U;align->last_observation=0U;
    align->minimum_frame_tick=now-age;align->last_valid=now;
  }
}
static float ZDT_Align_YawOmega(const vision_yaw_t *yaw,float heading)
{
  float cap=(float)CHASSIS_SPIN_ARM_MM*V_YAW_RATE_DPS*0.01745329252f/
      (ZDT_MILEAGE_MM_PER_REV*CHASSIS_MAX_WHEEL_RPM/60.0f);
  float error=NavBlend_AngleError(yaw->goal_heading,heading);
  float omega=yaw->turn_allowed?error*ZDT_NAV_HOLD_KP:0.0f;
  if(omega>cap)omega=cap;
  if(omega< -cap)omega=-cap;
  return fabsf(error)<=0.25f?0.0f:omega;
}
static void ZDT_Align_TimeoutDetails(void)
{
  vision_ring_axis_t axis={0};uint32_t age=UINT32_MAX;char line[256];
  (void)Vision_GetRingAxisWindow(zdt_align.config.camera,V_YAW_FRESH_MS,&axis,&age);
  (void)snprintf(line,sizeof(line),"[ALIGN TIMEOUT] xy=%u stable=%u err=%+.1f,%+.1f tol=%.1f yaw=%u yaw_stable=%u axis_valid=%u q=%u n=%u angle=%+.2f age=%lums turn=%u obs=%lu/%lu\r\n",
      (unsigned)zdt_align.state,zdt_align.stable_count,(double)zdt_align.error_x,(double)zdt_align.error_y,
      (double)zdt_align.config.tolerance_px,(unsigned)zdt_vyaw.phase,zdt_vyaw.stable,axis.valid,axis.confidence,
      axis.landmarks,(double)axis.axis_deg,(unsigned long)age,zdt_vyaw.turn_allowed,
      (unsigned long)axis.observation_id,(unsigned long)zdt_align_scene.observation_id);
  ZDT_Console_Print(line);
}
static void ZDT_Align_Service(void)
{
  uint32_t now,age=UINT32_MAX,samples;
  chassis_telemetry_t t;vision_pixel_t p={0};vision_ring_count_t scene={0};vision_plate_scene_t plate={0};float forward,right,error,omega;
  int16_t rpm[4];uint8_t i,stopped=1U;
  ZDT_VyawObserve();
  /* The observer can publish a newer shared IMU tick. Read time AFTER it:
   * a pre-observer timestamp minus that tick underflows uint32_t, even1ms. */
  now=HAL_GetTick();
  if(zdt_match_io!=MATCH_IO_ALIGN)return;
  if(zdt_align_phase!=2U)ZDT_Align_DiagObserve(now,NULL,UINT32_MAX,0.0f,0.0f,0.0f);
  ChassisMotor_GetTelemetry(&t);
  samples=HWT101_GetSampleCount();
  if(samples!=zdt_align_imu_count){zdt_align_imu_count=samples;zdt_align_imu_tick=now;}
  if(!ChassisMotor_IsEnabled() || !HWT101_IsZeroDriftReady() || !isfinite(t.heading_deg) ||
      !isfinite(t.gyro_dps) || now-zdt_align_imu_tick>150U) {
    (void)ChassisMotor_StopImmediate();
    ZDT_Match_Complete(0U,"alignment IMU/motor unavailable");return;
  }
  error=NavBlend_AngleError(zdt_align_heading,t.heading_deg);
  if(fabsf(error)>3.0f && zdt_vyaw.phase!=V_YAW_ACQUIRE && zdt_vyaw.phase!=V_YAW_TURN && zdt_vyaw.phase!=V_YAW_COMBINED){
    char line[180];ChassisMotor_GetWheelRpm(rpm);
    (void)snprintf(line,sizeof(line),"[ALIGN] yaw drift target=%.2f actual=%.2f error=%+.2f gyro=%+.2f cmd F%+.4f R%+.4f O%+.4f rpm=%d,%d,%d,%d\r\n",
      (double)zdt_align_heading,(double)t.heading_deg,(double)error,(double)t.gyro_dps,
      (double)t.vx,(double)t.vy,(double)t.omega,rpm[0],rpm[1],rpm[2],rpm[3]);
    ZDT_Console_Print(line);ZDT_Match_Complete(0U,"alignment heading drift");return;
  }
  if(now-zdt_align_tick>=zdt_align_config[ZDT_Align_Station()-1U].timeout_ms+VISION_CAMERA_MODE_STAGE_TIMEOUT_MS+1000U) {
    ZDT_Align_TimeoutDetails();
    ZDT_Match_Complete(0U,"alignment stage timeout");return;
  }
  if(zdt_align_phase==4U) {
    vision_mode_action_t action=ZDT_Camera_ModeStep();
    if(action==V_MODE_FAILED){ZDT_Camera_FailureDetails();ZDT_Match_Complete(0U,zdt_camera_gate.reason);return;}
    if(action==V_MODE_READY && !zdt_mileage_query_active && !zdt_mileage_stop_pending) {
      /* After ACK clear station-boundary pixels without cancelling mode state. */
      (void)Vision_ClearPixelObservations(zdt_camera_gate.camera);
      zdt_align_phase=1U;ZDT_Mileage_StartQuery(0U);
    }
    return;
  }
  {
    vision_camera_state_t state={0};
    if(!Vision_GetCameraState(zdt_camera_gate.camera,&state) || !VisionModeGate_Matches(&zdt_camera_gate,&state)) {
      (void)ChassisMotor_StopImmediate();
      ZDT_Camera_FailureDetails(); /* Snapshot BEFORE failure sends STOP and overwrites the camera session. */
      ZDT_Match_Complete(0U,"camera confirmation lost during alignment");return;
    }
  }
  if(zdt_align_phase!=2U)return;
  ChassisMotor_GetWheelRpm(rpm);
  for(i=0U;i<4U;i++)if(rpm[i])stopped=0U;
  if(fabsf(t.gyro_dps)>1.5f)stopped=0U;
  ZDT_Align_ReadPixels(&zdt_align.config,&p,&scene,&plate,&age);
  zdt_align_scene=scene;zdt_align_plate_scene=plate;
  if(ZDT_Align_Temp2StationaryReady(now,(uint8_t)(stopped && !ChassisMotor_HasMotionDemand()))) {
    (void)ChassisMotor_Stop();VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);
    zdt_align_heading=t.heading_deg; /* Hold the stopped actual heading, not the abandoned yaw goal. */
    zdt_align_phase=3U;
    ZDT_Console_Print("[ALIGN] TEMP2 stationary2s: skip remaining XY/yaw; final wheel snapshot then normal placement\r\n");
    ZDT_Mileage_StartQuery(0U);return;
  }
  forward=right=omega=0.0f;
  ZDT_Align_ResumeCombinedXY(&zdt_align,&zdt_vyaw,&p,age,now);
  if(zdt_vyaw.phase!=V_YAW_ACQUIRE && zdt_vyaw.phase!=V_YAW_TURN)
    VisionAlign_StepTargets(&zdt_align,now,&p,age,&scene,age,&plate,age,stopped,&forward,&right);
  if(zdt_vyaw.phase!=V_YAW_IDLE) {
    vision_ring_axis_t axis={0};uint32_t axis_age=UINT32_MAX;vision_yaw_phase_t old_phase=zdt_vyaw.phase;
    (void)Vision_GetRingAxisWindow(zdt_align.config.camera,V_YAW_FRESH_MS,&axis,&axis_age);
    if(axis.observation_id!=scene.observation_id)axis.valid=0U;
    if(zdt_vyaw.simultaneous && zdt_align.state==V_ALIGN_DONE &&
       (!p.valid || p.observation_id!=scene.observation_id || age>=zdt_align.config.observation_ms))axis.valid=0U;
    /* Keep the original initial stopped-settle before launching both loops.
     * Subsequent in-band settling can reopen XY on a new out-of-band frame. */
    if(zdt_vyaw.phase==V_YAW_COARSE && zdt_align.state==V_ALIGN_SETTLE)axis.valid=0U;
    VisionYaw_Step(&zdt_vyaw,now,&axis,axis_age,t.heading_deg,t.gyro_dps,stopped,(uint8_t)(zdt_align.state==V_ALIGN_DONE));
    if(zdt_vyaw.phase!=old_phase) {
      char line[256];(void)ChassisMotor_StopImmediate();
      (void)snprintf(line,sizeof(line),"[VYAW] station=%u phase=%u->%u axis=%+.2f valid=%u q=%u n=%u age=%lums seq=%u set=%u error=%+.2f goal=%+.2f IMU=%+.2f travel=%.2f\r\n",
        zdt_vyaw.station,(unsigned int)old_phase,(unsigned int)zdt_vyaw.phase,(double)axis.axis_deg,axis.valid,
        axis.confidence,axis.landmarks,(unsigned long)axis_age,axis.seq,axis.request_seq,
        (double)zdt_vyaw.error,(double)zdt_vyaw.goal_heading,(double)t.heading_deg,(double)zdt_vyaw.total);ZDT_Console_Print(line);
    }
    if(zdt_vyaw.phase==V_YAW_FAILED){(void)ChassisMotor_StopImmediate();ZDT_Match_Complete(0U,zdt_vyaw.reason);return;}
    if(zdt_vyaw.phase==V_YAW_ACQUIRE || zdt_vyaw.phase==V_YAW_TURN || zdt_vyaw.phase==V_YAW_COMBINED || zdt_vyaw.restart_xy)
      zdt_align_heading=zdt_vyaw.goal_heading;
    if(zdt_vyaw.restart_xy) {
      vision_align_config_t config=zdt_align.config;
      uint32_t driven=zdt_align.drive_ms,searched=zdt_align.search_ms;
      if(!VisionAlign_Start(&zdt_align,&config,now,p.observation_id)){ZDT_Match_Complete(0U,"yaw XY restart rejected");return;}
      zdt_align.drive_ms=driven;zdt_align.search_ms=searched;
    }
    if((zdt_vyaw.phase==V_YAW_ACQUIRE || zdt_vyaw.phase==V_YAW_TURN) &&
       !ZDT_Align_SearchWithYaw(&zdt_align,&zdt_vyaw)) {
      omega=ZDT_Align_YawOmega(&zdt_vyaw,t.heading_deg);
      if(omega==0.0f)(void)ChassisMotor_StopImmediate();
      else if(ChassisMotor_SetVelocity(0.0f,0.0f,omega)!=HAL_OK)ZDT_Match_Complete(0U,"yaw velocity rejected");
      return;
    }
    if(zdt_vyaw.phase==V_YAW_COMBINED)omega=ZDT_Align_YawOmega(&zdt_vyaw,t.heading_deg);
    if(zdt_align.state==V_ALIGN_DONE && zdt_vyaw.phase==V_YAW_FINE){(void)ChassisMotor_StopImmediate();return;}
  }
  if(zdt_vyaw.phase!=V_YAW_COMBINED && (forward!=0.0f || right!=0.0f)) {
    omega=error*ZDT_NAV_HOLD_KP;
    if(omega>0.02f)omega=0.02f;
    if(omega< -0.02f)omega=-0.02f;
    if(fabsf(error)<=ZDT_NAV_HOLD_DEADBAND_DEG)omega=0.0f;
  }
  ZDT_Align_DiagObserve(now,&p,age,forward,right,omega);
  if(zdt_align.state==V_ALIGN_FAILED) {
    char line[160];
    (void)ChassisMotor_StopImmediate();
    (void)snprintf(line,sizeof(line),"[ALIGN] last dx=%+.1f dy=%+.1f px, drive=%lu/%lums, valid=%u age=%lums\r\n",
      (double)zdt_align.error_x,(double)zdt_align.error_y,
      (unsigned long)zdt_align.drive_ms,(unsigned long)zdt_align.config.max_drive_ms,
      (unsigned int)p.valid,(unsigned long)age);
    ZDT_Console_Print(line);
    (void)snprintf(line,sizeof(line),"[ALIGN] scene value=%u trusted=%u station=%u set_seq=%u seq=%u age=%lums search=%lu/%lums state=%u\r\n",
      p.type==VISION_TYPE_PLATE_PX?plate.status:scene.count,p.type==VISION_TYPE_PLATE_PX?plate.valid:scene.valid,
      p.type==VISION_TYPE_PLATE_PX?plate.station:scene.station,p.type==VISION_TYPE_PLATE_PX?plate.request_seq:scene.request_seq,
      p.type==VISION_TYPE_PLATE_PX?plate.seq:scene.seq,(unsigned long)age,
      (unsigned long)zdt_align.search_ms,(unsigned long)zdt_align.config.search_limit_ms,
      (unsigned int)zdt_align.state);
    ZDT_Console_Print(line);
    ZDT_Match_Complete(0U,zdt_align.reason);return;
  }
  if(zdt_align.state==V_ALIGN_DONE && (zdt_vyaw.phase==V_YAW_IDLE || zdt_vyaw.phase==V_YAW_DONE)) {
    (void)ChassisMotor_Stop();zdt_align_phase=3U;ZDT_Mileage_StartQuery(0U);return;
  }
  if(forward==0.0f && right==0.0f && omega==0.0f){(void)ChassisMotor_Stop();return;}
  if(zdt_align.config.mode==V_ALIGN_MODE_SMOOTH || zdt_align.state==V_ALIGN_SEARCH) {
    float mx,my;
    NavFrame_BodyToMap((float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,t.heading_deg,forward,right,&mx,&my);
    if(ChassisMotor_SetFieldVelocity((float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,mx,my,omega)!=HAL_OK)
      ZDT_Match_Complete(0U,"alignment field velocity rejected");
  } else if(ChassisMotor_SetVelocity(forward,right,omega)!=HAL_OK)
      ZDT_Match_Complete(0U,"alignment velocity rejected");
}
/* A bounded, requested snapshot is emitted only at zero motor demand/RPM.
 * Never dump a whole trace or print per camera frame from motion control. */
static void ZDT_Align_ProgressService(void)
{
  static const char *const names[]={"IDLE","SETTLE","WAIT_PIXELS","PULSE","DONE","FAULT","TRACK","SEARCH"};
  uint32_t now=HAL_GetTick(),age=UINT32_MAX;uint8_t i;
  int16_t rpm[4];chassis_telemetry_t telemetry;vision_pixel_t pixel={0};vision_ring_count_t scene={0};vision_plate_scene_t plate={0};
  char line[256];const char *request;
  if(zdt_match_io!=MATCH_IO_ALIGN){zdt_align_status_pending=0U;return;}
  /* A real stationary combined-alignment stall needs a bounded snapshot
   * without requiring a manual command. Existing stopped-only checks below
   * keep this diagnostic out of motion; it never changes control decisions. */
  if(!zdt_align_status_pending && zdt_vyaw.phase==V_YAW_COMBINED && now-zdt_align_status_tick>=5000U)
    zdt_align_status_pending=1U;
  if(!zdt_align_status_pending || zdt_align_phase!=2U || now-zdt_align_status_tick<1000U ||
      ChassisMotor_HasMotionDemand())return;
  ChassisMotor_GetWheelRpm(rpm);
  for(i=0U;i<4U;i++)if(rpm[i])return;
  ChassisMotor_GetTelemetry(&telemetry);
  if(!isfinite(telemetry.gyro_dps) || fabsf(telemetry.gyro_dps)>1.5f)return;
  ZDT_Align_ReadPixels(&zdt_align.config,&pixel,&scene,&plate,&age);
  request=(zdt_align_status_pending&2U)?"OK needs WAIT_OK":
    ((zdt_align_status_pending&4U)?"trace dump needs idle":"status");
  (void)snprintf(line,sizeof(line),"[ALIGN] LIVE %s state=%s stable=%u/%u px=%+.1f,%+.1f valid=%u age=%lums rings=%u/%u drive=%lums search=%lums gap_limit=%u\r\n",
    request,(unsigned int)zdt_align.state<8U?names[zdt_align.state]:"UNKNOWN",
    zdt_align.stable_count,zdt_align.config.stable_frames,(double)pixel.dx_px,(double)pixel.dy_px,
    pixel.valid,(unsigned long)age,pixel.type==VISION_TYPE_PLATE_PX?plate.status:scene.count,
    pixel.type==VISION_TYPE_PLATE_PX?plate.valid:scene.valid,(unsigned long)zdt_align.drive_ms,
    (unsigned long)zdt_align.search_ms,V_ALIGN_STABLE_FRAME_GAP_MS);
  zdt_align_status_pending=0U;zdt_align_status_tick=now;ZDT_Console_Print(line);
  if(zdt_vyaw.phase!=V_YAW_IDLE) {
    vision_ring_axis_t axis={0};uint32_t axis_age=UINT32_MAX;float e=NAN;
    (void)Vision_GetRingAxisWindow(zdt_align.config.camera,V_YAW_FRESH_MS,&axis,&axis_age);
    if(axis.valid)e=zdt_vyaw.cal[zdt_vyaw.station-2U].gain*
        VisionYaw_LineError(axis.axis_deg-zdt_vyaw.cal[zdt_vyaw.station-2U].zero);
    (void)snprintf(line,sizeof(line),"[VYAW] LIVE enabled=%u phase=%u axis=%+.2f valid=%u q=%u n=%u age=%lums error=%+.2f stable=%u/3 goal_IMU=%+.2f travel=%.2fdeg\r\n",
        zdt_vyaw.enabled,(unsigned int)zdt_vyaw.phase,(double)axis.axis_deg,axis.valid,axis.confidence,axis.landmarks,
        (unsigned long)axis_age,(double)e,zdt_vyaw.stable,(double)zdt_vyaw.goal_heading,(double)zdt_vyaw.total);
    ZDT_Console_Print(line);
  }
}
static void ZDT_Align_SetTiming(vision_align_config_t *c)
{
  c->observation_ms=c->mode==V_ALIGN_MODE_SMOOTH?400U:200U;
  c->fade_start_ms=c->mode==V_ALIGN_MODE_SMOOTH?260U:120U;
  c->fade_zero_ms=c->mode==V_ALIGN_MODE_SMOOTH?360U:160U;
}
static uint8_t ZDT_RingProfileCommand(const char *command)
{
  if(strncmp(command,"valign rings",12U)==0 && (!command[12] || command[12]==' ')) {
    const char *profile="auto";
    if(strcmp(command,"valign rings")==0 || strcmp(command,"valign rings status")==0) {
      profile=zdt_ring_profile==2U?"occupied":zdt_ring_profile==1U?"fast":"auto";
    } else {
      if(ZDT_Align_DiagBusy()){ZDT_Console_Print("[ALIGN] ring profile change requires stopped chassis\r\n");return 1U;}
      if(strcmp(command,"valign rings auto")==0)zdt_ring_profile=0U;
      else if(strcmp(command,"valign rings fast")==0)zdt_ring_profile=1U;
      else if(strcmp(command,"valign rings occupied")==0)zdt_ring_profile=2U;
      else {ZDT_Console_Print("[ALIGN] valign rings auto|fast|occupied|status\r\n");return 1U;}
      profile=zdt_ring_profile==2U?"occupied":zdt_ring_profile==1U?"fast":"auto";
    }
    ZDT_Console_Print("[ALIGN] ring profile=");ZDT_Console_Print(profile);
    ZDT_Console_Print(" (auto: occupied only TEMP2); axis correction remains independent\r\n");return 1U;
  }
  return 0U;
}
static uint8_t ZDT_ManualAlignCommand(const char *command)
{
  if(!strcmp(command,"valign run rough") || !strcmp(command,"valign run temp") || !strcmp(command,"valign run plate")) {
    uint32_t token;
    uint8_t plate=(uint8_t)(!strcmp(command,"valign run plate"));
    if(zdt_manual_align_station || ZDT_Align_DiagBusy() || MatchControl_IsActive(&zdt_match) || zdt_match_request ||
       zdt_match_io!=MATCH_IO_NONE || ArmMaterial_IsActive() || ArmRound1_IsActive() || ArmStartPose_IsActive() ||
       zdt_qr_active || zdt_arm_camera_token || (!plate && !zdt_vyaw.enabled)) {
      ZDT_Console_Print("[MANUAL ALIGN] REJECTED: idle arm/chassis/flow required; vyaw ON for rings\r\n");return 1U;
    }
    token=0x80000000U|(++zdt_manual_align_next & 0x7FFFFFFFU);
    if(ArmTask_RequestCameraDeploy(token)!=HAL_OK) {
      ZDT_Console_Print("[MANUAL ALIGN] REJECTED: camera pose request busy\r\n");return 1U;
    }
    zdt_manual_align_station=plate?1U:(uint8_t)(!strcmp(command,"valign run rough")?2U:3U);
    zdt_manual_align_token=token;zdt_manual_align_started=HAL_GetTick();
    ZDT_Console_Print(plate?"[MANUAL ALIGN] CAMERA: then competition PLATE XY alignment; no pickup or MATCH advance\r\n":
        "[MANUAL ALIGN] CAMERA: then XY+yaw; no station pickup or MATCH advance\r\n");return 1U;
  }
  return 0U;
}
static void ZDT_ManualAlignService(void)
{
  if(zdt_manual_align_token) {
    uint8_t success=0U;
    if(ArmTask_GetCameraDeployResult(zdt_manual_align_token,&success)) {
      zdt_manual_align_token=0U;
      if(!success)ZDT_Match_Complete(0U,"camera pose failed");
      else if(ChassisMotor_Enable()!=HAL_OK || !ZDT_Align_Start(zdt_manual_align_station))
        ZDT_Match_Complete(0U,"alignment start rejected");
    } else if(HAL_GetTick()-zdt_manual_align_started>=120000U)ZDT_Match_Complete(0U,"camera pose timeout");
  }
}
static uint8_t ZDT_Align_Command(const char *command)
{
  unsigned int station,camera,id,axes;char type[8],extra;float fx,fy,rx,ry;
  if(ZDT_ManualAlignCommand(command))return 1U;
  if(ZDT_Align_DiagCommand(command))return 1U;
  if(ZDT_RingProfileCommand(command))return 1U;
  if(strncmp(command,"valign",6U)==0) {
    if(zdt_match_io==MATCH_IO_ALIGN){return 1U;}
    if(strcmp(command,"valign smooth on")==0 || strcmp(command,"valign smooth off")==0 ||
       strcmp(command,"valign search off")==0 || strcmp(command,"valign search image-right")==0) {
      uint8_t i;
      if(ZDT_Align_DiagBusy()){ZDT_Console_Print("[ALIGN] mode change requires stopped chassis.\r\n");return 1U;}
      if(strncmp(command,"valign smooth",13U)==0)
        zdt_align_smooth_enabled=(uint8_t)(strcmp(command,"valign smooth on")==0);
      else zdt_align_search_axis=(uint8_t)(strcmp(command,"valign search off")==0?0U:V_ALIGN_SEARCH_IMAGE_RIGHT);
      for(i=0U;i<3U;i++) {
        zdt_align_config[i].mode=zdt_align_smooth_enabled?V_ALIGN_MODE_SMOOTH:V_ALIGN_MODE_PULSE;
        zdt_align_config[i].search_axis=zdt_align_search_axis;ZDT_Align_SetTiming(&zdt_align_config[i]);
      }
      ZDT_Console_Print("[ALIGN] smooth/search settings updated; valign status to inspect.\r\n");return 1U;
    }
    if(strcmp(command,"valign on")==0) {
      if(!zdt_align_enabled || ZDT_Camera_ResetBusy())ZDT_Camera_Reset();
      zdt_align_enabled=1U;
    }
    else if(strcmp(command,"valign off")==0){zdt_align_enabled=0U;ZDT_Camera_Reset();}
    else if(strcmp(command,"valign")==0 || strcmp(command,"valign status")==0) {
      uint8_t i;char line[220];
      if(ZDT_Align_DiagBusy()) {
        ZDT_Console_Print("[ALIGN] full status requires stopped chassis / WAIT_OK.\r\n");return 1U;
      }
      for(i=0U;i<3U;i++) {
        vision_align_config_t *c=&zdt_align_config[i];
        (void)snprintf(line,sizeof(line),"[ALIGN] %s station%u CAM%u type=%02X id=%u axes=%u K=%+.4f,%+.4f,%+.4f,%+.4f\r\n",
          zdt_align_enabled?"ON":"OFF",i+1U,c->camera,c->type,c->id,c->axes,
          (double)c->forward_x,(double)c->forward_y,(double)c->right_x,(double)c->right_y);
        ZDT_Console_Print(line);
        (void)snprintf(line,sizeof(line),"[ALIGN] station%u mode=%s tol=%.2f vmax=%.4f vmin=%.4f pulse=%u settle=%u stable_ms=%u stable_frames=%u\r\n",
          i+1U,c->mode==V_ALIGN_MODE_SMOOTH?"SMOOTH_PID":"PULSE",(double)c->tolerance_px,(double)c->max_speed,(double)c->min_speed,
          c->pulse_ms,c->settle_ms,c->stable_ms,c->stable_frames);
        ZDT_Console_Print(line);
        (void)snprintf(line,sizeof(line),"[ALIGN] station%u PID p=%.3f i=%.3f d=%.3f Ifence=%.1fpx Dfilter=%u search_axis=%u search_limit=%lums\r\n",
          i+1U,(double)c->pid_p,(double)c->pid_i,(double)c->pid_d,(double)c->integral_limit_px,
          c->derivative_filter_ms,c->search_axis,(unsigned long)c->search_limit_ms);
        ZDT_Console_Print(line);
        (void)snprintf(line,sizeof(line),"[ALIGN] station%u loss=%u timeout=%lu drive_limit=%lu rx_age_limit=%u fade=%u/%u seek=%.3f emptyL=%u emptyLspd=%.3f trace=%u capacity=%u\r\n",
          i+1U,c->loss_ms,(unsigned long)c->timeout_ms,(unsigned long)c->max_drive_ms,
          c->observation_ms,c->fade_start_ms,c->fade_zero_ms,(double)c->search_speed,c->zero_ring_left,
          (double)(c->zero_ring_left?fminf(c->search_speed*V_ALIGN_EMPTY_LEFT_SPEED_MULTIPLIER,c->max_speed):0.0f),
          zdt_align_diag.enabled,V_ALIGN_DIAG_CAPACITY);
        ZDT_Console_Print(line);
      }
      return 1U;
    } else {ZDT_Console_Print("[ALIGN] valign on|off|status\r\n");return 1U;}
    ZDT_Console_Print(zdt_align_enabled?"[ALIGN] automatic station alignment ON\r\n":
      "[ALIGN] automatic station alignment OFF\r\n");return 1U;
  }
  if(strncmp(command,"vsource",7U)==0 || strncmp(command,"vgain",5U)==0 || strncmp(command,"vtune",5U)==0 || strncmp(command,"vpid",4U)==0) {
    vision_align_config_t c;
    if(zdt_match.state==MATCH_MOVING || zdt_match.state==MATCH_LEAVING) {
      ZDT_Console_Print("[ALIGN] change calibration only while stopped\r\n");return 1U;
    }
    if(strncmp(command,"vpid",4U)==0) {
      float pg,ig,dg;
      if(sscanf(command,"vpid %u %f %f %f %c",&station,&pg,&ig,&dg,&extra)!=4 || station<1U || station>3U) {
        ZDT_Console_Print("[ALIGN] vpid <station> <P multiplier> <I 1/s> <D seconds>; defaults 1 0 0\r\n");return 1U;
      }
      c=zdt_align_config[station-1U];c.pid_p=pg;c.pid_i=ig;c.pid_d=dg;
    } else if(strncmp(command,"vsource",7U)==0) {
      if(sscanf(command,"vsource %u %u %7s %u %u %c",&station,&camera,type,&id,&axes,&extra)!=5 ||
          station<1U || station>3U || camera<1U || camera>2U || id>6U || axes<1U || axes>3U) {
        ZDT_Console_Print("[ALIGN] vsource <1..3> <cam1|2> <plate|ring> <id> <axes1..3>\r\n");return 1U;
      }
      c=zdt_align_config[station-1U];c.camera=(uint8_t)camera;c.id=(uint8_t)id;c.axes=(uint8_t)axes;
      if(strcmp(type,"plate")==0)c.type=VISION_TYPE_PLATE_PX;
      else if(strcmp(type,"ring")==0)c.type=VISION_TYPE_RING_PX;
      else {ZDT_Console_Print("[ALIGN] source must be plate or ring\r\n");return 1U;}
      c.search_axis=zdt_align_search_axis;
      c.zero_ring_left=(uint8_t)(c.type==VISION_TYPE_RING_PX && c.station==2U);
      c.search_limit_ms=c.type==VISION_TYPE_RING_PX?8000U:4000U;
    } else if(strncmp(command,"vtune",5U)==0) {
      unsigned int pulse,settle;float tolerance,speed;
      if(sscanf(command,"vtune %u %f %f %u %u %c",&station,&tolerance,&speed,&pulse,&settle,&extra)!=5 ||
          station<1U || station>3U || pulse>300U || settle>5000U) {
        ZDT_Console_Print("[ALIGN] vtune <1..3> <tol_px> <max_spd> <pulse_ms> <settle_ms>\r\n");return 1U;
      }
      c=zdt_align_config[station-1U];c.tolerance_px=tolerance;c.max_speed=speed;
      c.pulse_ms=(uint16_t)pulse;c.settle_ms=(uint16_t)settle;
    } else {
      if(sscanf(command,"vgain %u %f %f %f %f %c",&station,&fx,&fy,&rx,&ry,&extra)!=5 || station<1U || station>3U) {
        ZDT_Console_Print("[ALIGN] vgain <1..3> <Fdx> <Fdy> <Rdx> <Rdy>\r\n");return 1U;
      }
      c=zdt_align_config[station-1U];c.forward_x=fx;c.forward_y=fy;c.right_x=rx;c.right_y=ry;
    }
    if(!VisionAlign_ConfigValid(&c)){ZDT_Console_Print("[ALIGN] invalid source/gain matrix\r\n");return 1U;}
    zdt_align_config[station-1U]=c;ZDT_Console_Print("[ALIGN] calibration updated in RAM\r\n");return 1U;
  }
  return 0U;
}

static float ZDT_Zone2_RelYaw(float world)
{
  return NavBlend_AngleError(world,atan2f((float)-zdt_nav_nose_y,(float)zdt_nav_nose_x)*57.29578f);
}
static uint8_t ZDT_Zone2_WheelsStopped(void)
{
  int16_t rpm[4];uint8_t i;ChassisMotor_GetWheelRpm(rpm);
  for(i=0;i<4;i++)if(rpm[i])return 0U;
  return 1U;
}
static void ZDT_Zone2_Cancel(void)
{
  zdt_zone2_phase=Z2_NONE;zdt_zone2_action_id=0U;
  zdt_mileage_query_active=0U;ZDT_Nav_Cancel();
  (void)ChassisMotor_StopImmediate();
}
static void ZDT_Zone2_Finished(uint8_t success)
{
  zdt_zone2_phase=Z2_NONE;zdt_zone2_action_id=0U;
  ZDT_Match_Complete(success,success?NULL:(zdt_zone2_reason?zdt_zone2_reason:"zone2 action rejected/timeout"));
}
static void ZDT_Zone2_Fail(const char *reason)
{
  zdt_zone2_reason=reason;
  Zone2Route_Result(&zdt_zone2,zdt_zone2_action_id,0U);
}
static void ZDT_Zone2_Done(void)
{
  uint32_t id=zdt_zone2_action_id;
  zdt_zone2_phase=Z2_NONE;zdt_zone2_action_id=0U;
  Zone2Route_Result(&zdt_zone2,id,1U);
}
/* QR departure only: an up-to-5mm lateral error does not justify a separate
 * stop/strafe. Plan the actual parallel line from the measured pose, keeping
 * the normal straight-line clearance check and the QR forward coordinate. */
static uint8_t ZDT_Zone2_TryDepartureQrStraight(const zone2_action_t *a,float yaw)
{
  uint16_t px,py;int32_t offset;char line[112];
  if(!a || !zdt_nav_queued_recenter || zdt_match.state!=MATCH_MOVING ||
     zdt_match.checkpoint!=MATCH_QR || zdt_zone2.index!=0U || a->extra_mm!=0)return 0U;
  GridNav_GetPoseMm(&px,&py);offset=(int32_t)a->x-(int32_t)px;
  if(!offset || abs(offset)>ZDT_DEPARTURE_LANE_TOLERANCE_MM || py==a->y ||
     !NavFrame_StrafeSide((float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,yaw,
         offset>0?1.0f:-1.0f,0.0f,ZDT_NAV_ALIGN_TOLERANCE_DEG))return 0U;
  if(!GridNav_PlanStraightTargetMm(px,a->y))return 0U;
  (void)snprintf(line,sizeof(line),"[NAV] QR departure lane offset=%+ldmm <=5mm; skip strafe, actual x=%u\r\n",
      (long)offset,(unsigned int)px);
  ZDT_Console_Print(line);return 1U;
}
static uint8_t ZDT_Zone2_StartAction(const zone2_action_t *a,uint32_t id)
{
  char line[128];float target,error,yaw=ZDT_Nav_GetHeading();
  float tolerance=(zdt_match.checkpoint==MATCH_RETURN)?0.6f:1.5f;
  zdt_zone2_action=*a;zdt_zone2_action_id=id;
  if(!isfinite(yaw)){zdt_zone2_reason="zone2 heading invalid";return 0U;}
  zdt_zone2_imu_count=HWT101_GetSampleCount();zdt_zone2_imu_tick=HAL_GetTick();
  (void)snprintf(line,sizeof(line),"[ZONE%u] %u/%u %s goal=(%u,%u) yaw=%d\r\n",
      (unsigned int)zdt_match.zone,zdt_zone2.index+1U,zdt_zone2.count,a->name,a->x,a->y,a->yaw);ZDT_Console_Print(line);
  if(a->kind==ZONE2_NAV) {
    zdt_zone2_phase=Z2_NAV;ZDT_Nav_RefreshCostModel();
    zdt_nav_queued_recenter=zdt_nav_post_align;
    if(ZDT_Zone2_TryDepartureQrStraight(a,yaw)) {
      /* Actual start x and goal y are already retained by the straight plan. */
      if(GridNav_HasObstacles())zdt_vobs_nav_active=1U;
    } else if(GridNav_HasObstacles()) {
      /* A previously mapped obstacle must remain part of the real subsequent
       * station route. Keep this Zone2 action pending if no path exists. */
      zdt_vobs_goal_x=a->x;zdt_vobs_goal_y=a->y;zdt_vobs_resume_script=0U;
      zdt_vobs_poll_tick=HAL_GetTick();ZDT_Vobs_Replan();return 1U;
    } else if(zdt_nav_queued_recenter && GridNav_PlanRecenterTargetMmHeading(a->x,a->y,
        ZDT_Nav_GetHeading()+atan2f((float)-zdt_nav_nose_y,(float)zdt_nav_nose_x)*57.29578f))
      ZDT_Console_Print("[NAV] POST_ALIGN_MM_V2: heading-aware lane offset, then transport.\r\n");
    else if(GridNav_PlanStraightTargetMm(a->x,a->y))
      ZDT_Console_Print("[ZONE2] straight mm route; no grid-centre jog.\r\n");
    else if(zdt_match.checkpoint==MATCH_RETURN && zdt_match.zone==1U) {
      if(!GridNav_PlanCheckedTargetMmHeading(a->x,a->y,
          yaw+atan2f((float)-zdt_nav_nose_y,(float)zdt_nav_nose_x)*57.29578f)) {
        zdt_zone2_reason="return has no collision-free detour";return 0U;
      }
      zdt_nav_queued_recenter=0U;
      ZDT_Console_Print("[RETURN] direct corridor blocked; checked A* detour keeps QR/staging target.\r\n");
    }
    else if(!GridNav_PlanTargetMm(a->x,a->y))return 0U;
    zdt_nav_post_align=0U;ZDT_Nav_Start();return 1U;
  }
  target=ZDT_Zone2_RelYaw(a->yaw);error=NavBlend_AngleError(target,yaw);
  if(fabsf(error)<=tolerance) {
    (void)snprintf(line,sizeof(line),"[FACE] zone=%u target=%ddeg error=%+.2fdeg; already aligned\r\n",
      (unsigned int)zdt_match.zone,a->yaw,(double)error);
    ZDT_Console_Print(line);ZDT_Zone2_Done();return 1U;
  }
  if(!ZDT_Zone2_WheelsStopped()){zdt_zone2_reason="zone2 wheels still moving";return 0U;}
  /* Fixed-route turns execute the requested yaw without a map sweep veto,
   * as requested during station turn trials. */
  if(((zdt_match.checkpoint==MATCH_RETURN)?ChassisMotor_TurnRelative((int16_t)lroundf(error)):
      ChassisMotor_TurnRelativeFast((int16_t)lroundf(error)))!=HAL_OK) {
    zdt_zone2_reason="zone2 turn start rejected";return 0U;
  }
  (void)snprintf(line,sizeof(line),"[ZONE2] turn %+.1f deg (%s); target relative=%+.1f\r\n",
      (double)error,(error>0.0f)?"LEFT":"RIGHT",(double)target);ZDT_Console_Print(line);
  zdt_zone2_phase=Z2_TURN;zdt_zone2_tick=HAL_GetTick()+10000U;return 1U;
}
static void ZDT_Nav_NotifyArrived(void)
{
  zdt_vobs_nav_active=0U;
  if(zdt_vobs_recenter_active) {
    zdt_vobs_recenter_active=0U;ZDT_Vobs_Replan();return;
  }
  if(zdt_vobs_resume_script && zdt_script_running && zdt_match_io==MATCH_IO_LEAVE_SCRIPT) {
    zdt_vobs_resume_script=0U;
    if(zdt_vobs_leave_target==0U) {
      zdt_script_index++;zdt_vobs_rebase_script=1U;
      ZDT_Console_Print("[VOBS] interrupted departure step reached; continue remaining script in map coordinates\r\n");
    } else {
      zdt_script_index=zdt_script_count;zdt_vobs_rebase_script=0U;
      ZDT_Console_Print(zdt_vobs_leave_target==1U?
        "[VOBS] departure exit reached by checked detour; continue match\r\n":
        "[VOBS] next station reached by checked detour; continue match and station actions\r\n");
    }
    zdt_script_deadline=0U;zdt_script_settling=0U;
    return;
  }
  if(zdt_match_io==MATCH_IO_ZONE2 && zdt_zone2_phase==Z2_NAV) {
    /* Timed transport keeps the user's dcomp/station calibration. Wheel
     * residuals are diagnostic, never an implicit reverse/diagonal action. */
    ZDT_Zone2_Done();
  } else { ZDT_Match_Complete(1U,NULL); }
}
static void ZDT_Zone2_OnMileage(uint8_t mask)
{
  if(mask!=0x0fU)ZDT_Zone2_Fail("zone2 needs all 4 fresh wheel replies");
}
static void ZDT_Zone2_Service(void)
{
  uint32_t now=HAL_GetTick(),samples;chassis_telemetry_t t;
  if(!zdt_zone2.running)return;
  Zone2Route_Service(&zdt_zone2,now);
  if(!zdt_zone2.running || zdt_zone2_phase==Z2_NONE)return;
  /* Service may synchronously start the next action. That callback records a
   * later IMU tick and can print/plan before returning. Re-read time AFTER it:
   * an older poll timestamp minus the new action tick underflows uint32_t. */
  now=HAL_GetTick();
  ChassisMotor_GetTelemetry(&t);
  if(!HWT101_IsZeroDriftReady() || !isfinite(t.heading_deg) || !isfinite(t.gyro_dps)) {
    ZDT_Zone2_Fail("zone2 IMU not ready");return;
  }
  samples=HWT101_GetSampleCount();
  if(samples!=zdt_zone2_imu_count){zdt_zone2_imu_count=samples;zdt_zone2_imu_tick=now;}
  if(now-zdt_zone2_imu_tick>150U){
    char line[128];
    (void)snprintf(line,sizeof(line),"[IMU] route stale phase=%u age=%lums now=%lu last=%lu samples=%lu\r\n",
      (unsigned int)zdt_zone2_phase,(unsigned long)(now-zdt_zone2_imu_tick),
      (unsigned long)now,(unsigned long)zdt_zone2_imu_tick,(unsigned long)samples);
    ZDT_Console_Print(line);ZDT_Zone2_Fail("zone2 IMU stale");return;
  }
  if(zdt_zone2_phase==Z2_NAV)return;
  if((int32_t)(now-zdt_zone2_tick)>=0){ZDT_Zone2_Fail("zone2 turn timeout");return;}
  if(zdt_zone2_phase==Z2_TURN) {
    if(!ChassisMotor_IsTurnActive() && ZDT_Zone2_WheelsStopped() && fabsf(t.gyro_dps)<5.0f) {
      if(fabsf(NavBlend_AngleError(ZDT_Zone2_RelYaw(zdt_zone2_action.yaw),t.heading_deg))>
          ((zdt_match.checkpoint==MATCH_RETURN)?0.6f:1.5f))
        ZDT_Zone2_Fail("zone2 did not face target");
      else {
        char line[112];
        (void)snprintf(line,sizeof(line),"[FACE] zone=%u target=%ddeg verified; station turn complete\r\n",
          (unsigned int)zdt_match.zone,zdt_zone2_action.yaw);
        ZDT_Console_Print(line);ZDT_Zone2_Done();
      }
    }
    return;
  }
}

static void ZDT_Match_CancelMotion(void)
{
  ZDT_Vobs_Reset();
  if(zdt_manual_align_token)ArmTask_RequestStop();
  zdt_manual_align_station=0U;zdt_manual_align_token=0U;
  if(zdt_arm_camera_token || zdt_arm_start_token || zdt_arm_material_token || zdt_arm_round_token || ArmMaterial_IsActive() || ArmRound1_IsActive() || zdt_match_io==MATCH_IO_MATERIAL || zdt_match_io==MATCH_IO_ROUND1)ArmTask_RequestStop();
  zdt_arm_start_token=0U;zdt_arm_material_token=0U;zdt_arm_round_token=0U;
  zdt_prepare_request=0U;zdt_prepare_stopped=0U;
  zdt_arm_camera_token=0U;
  uint8_t i;
  ZDT_Qr_Cancel();
  zdt_nav_post_align=zdt_nav_queued_recenter=zdt_nav_recenter_route=0U;
  if(zdt_align_diag.active)zdt_align_diag.can_failures=zdt_can_enqueue_failures-zdt_diag_can_start;
  VisionDiag_Finish(&zdt_align_diag,HAL_GetTick(),0U,"cancel/reset");
  VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);zdt_align_phase=0U;
  if(zdt_zone2.running || zdt_zone2_phase!=Z2_NONE) { Zone2Route_Cancel(&zdt_zone2); }
  zdt_match_io = MATCH_IO_NONE;
  zdt_match_request = 0U;
  zdt_script_running = 0U;
  zdt_script_then_nav = 0U;
  zdt_script_end_zone = 0U;
  zdt_mileage_query_active = 0U;
  zdt_mileage_stop_pending = 0U;
  zdt_mileage_forward_pending = 0U;
  ZDT_Nav_Cancel();
  (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
  for (i = 0U; i < ZDT_MILEAGE_MOTOR_COUNT; ++i) {
    (void)ZDT_Motor_StopAddr(zdt_mileage_addresses[i]);
  }
  ZDT_Camera_Reset();
}

static void ZDT_Match_Changed(match_state_t state, match_checkpoint_t checkpoint)
{
  ZDT_Vobs_MatchState(state);
  ScreenMetrics_SetState(state==MATCH_FAULT?SCREEN_ERROR:state==MATCH_FINISHED?SCREEN_DONE:
    (state==MATCH_IDLE || state==MATCH_PREPARING || state==MATCH_WAIT_START_OK || checkpoint==MATCH_QR)?SCREEN_WAIT_QR:SCREEN_RUNNING);
  static const char *const prompts[7] = {
    "QR checked: ok -> raw batch1",
    "Load batch1 onto platform: ok -> rough",
    "Place batch1, pick it back onto platform: ok -> temp",
    "Unload batch1: ok -> raw batch2",
    "Load batch2 onto platform: ok -> rough",
    "Place batch2, pick it back onto platform: ok -> temp",
    "Stack batch2 on same colors: ok -> original start"
  };
  char line[128];
  (void)snprintf(line, sizeof(line), "[MATCH] %s %s zone=%u request=%lu\r\n",
                 MatchControl_StateName(state), MatchControl_CheckpointName(checkpoint),
                 (unsigned int)zdt_match.zone, (unsigned long)zdt_match.active_request);
  ZDT_Console_Print(line);
  if(state==MATCH_WAIT_START_OK) {
    ZDT_Console_Print(zdt_match.automatic?"[MATCH] launch pose READY; PD1 short/long release selects zone1/2, or OK 1/2; position binds at launch\r\n":
        "[MATCH] launch pose READY; send a NEW &OK to enable chassis and depart\r\n");
  }
  if(state==MATCH_MOVING && zdt_match.automatic)
    ZDT_Console_Print("[MATCH] automatic continuation; no further OK required\r\n");
  if ((state == MATCH_WAIT_OK) && ((unsigned int)checkpoint < 7U)) {
    if(zdt_match.first_round_only) {
      static const char *const trial[4]={"QR ready: NEW OK -> supply disc", "RED tray1/2/3 loaded: NEW OK -> rough rings", "rings1/2/3 placed AND returned to matching tray slots: NEW OK -> point3 TEMP", "point3 rings1/2/3 unloaded: NEW OK -> FINISHED (no round2)"};
      if(checkpoint<=MATCH_TEMP1){ZDT_Console_Print("[MATCH] ");ZDT_Console_Print(trial[checkpoint]);ZDT_Console_Print("\r\n");return;}
    }
    if(ArmMaterial_AutoEnabled() && (checkpoint==MATCH_RAW1 || checkpoint==MATCH_RAW2)) {
      ZDT_Console_Print("[MATCH] red trial cycle complete: send OK for next transport\r\n");return;
    }
    ZDT_Console_Print("[MATCH] ");
    ZDT_Console_Print((checkpoint==MATCH_TEMP2)?
        "Stack batch2: ok -> return staging (vision parking pending)":prompts[checkpoint]);
    ZDT_Console_Print("\r\n");
  } else if (state == MATCH_FAULT) {
    ZDT_Console_Print("[MATCH] flow stopped. Re-place car, match reset, startpose N.\r\n");
  } else if (state == MATCH_FINISHED) {
    ZDT_Console_Print(zdt_match.first_round_only?"[MATCH] FIRST ROUND FINISHED at point3; three items released; no round2/return motion.\r\n":"[MATCH] TWO ROUNDS FINISHED; returned to selected-zone staging; final parking pending.\r\n");
    if(zdt_match.run_timer_finished) {
      uint32_t elapsed=zdt_match.run_elapsed_ms;
      (void)snprintf(line,sizeof(line),"[MATCH TIME] total_ms=%lu elapsed=%lu:%02lu.%03lu endpoint=%s\r\n",
          (unsigned long)elapsed,(unsigned long)(elapsed/60000U),
          (unsigned long)((elapsed/1000U)%60U),(unsigned long)(elapsed%1000U),
          zdt_match.first_round_only?"TEMP1_DONE":"RETURN_STAGING_DONE");
      ZDT_Console_Print(line);
      {
        unsigned stage;uint32_t previous=0U;
        for(stage=0U;stage<9U;stage++)if(zdt_match.milestone_mask&(1U<<stage)) {
          uint32_t completed=zdt_match.milestone_ms[stage];
          (void)snprintf(line,sizeof(line),"[MATCH STAGE] %s from_start_ms=%lu stage_ms=%lu completed=%lu:%02lu.%03lu\r\n",
              stage?MatchControl_CheckpointName((match_checkpoint_t)(stage-1U)):"LEAVE",
              (unsigned long)completed,(unsigned long)(completed-previous),
              (unsigned long)(completed/60000U),(unsigned long)((completed/1000U)%60U),(unsigned long)(completed%1000U));
          ZDT_Console_Print(line);previous=completed;
        }
      }
    }
  }
}

static uint8_t ZDT_ManualAlignFinish(uint8_t success,const char *reason)
{
  if(zdt_manual_align_station) {
    char line[192];
    if(!success && zdt_manual_align_token)ArmTask_RequestStop();
    if(!success)(void)ChassisMotor_StopImmediate();else (void)ChassisMotor_Stop();
    if(zdt_align_diag.active)VisionDiag_Finish(&zdt_align_diag,HAL_GetTick(),success,reason);
    (void)snprintf(line,sizeof(line),"[MANUAL ALIGN] %s station=%u reason=%s\r\n",
        success?"DONE":"FAILED",zdt_manual_align_station,reason?reason:"complete");ZDT_Console_Print(line);
    VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);zdt_align_phase=0U;
    zdt_manual_align_station=0U;zdt_manual_align_token=0U;zdt_match_io=MATCH_IO_NONE;return 1U;
  }
  return 0U;
}
static void ZDT_Match_Complete(uint8_t success, const char *reason)
{
  if(ZDT_ManualAlignFinish(success,reason))return;
  uint8_t recognition_entry=(uint8_t)(zdt_match.checkpoint<=MATCH_RETURN &&
      zdt_match_io!=MATCH_IO_CAMERA && zdt_match_io!=MATCH_IO_ALIGN && zdt_match_io!=MATCH_IO_ROUND1);
  if(success && zdt_match.state==MATCH_LEAVING) {
    ZDT_Console_Print(zdt_match.qr_required?"[MATCH] departure complete: arm/QR already started; drive to QR (2125,1150), face east\r\n":
        "[MATCH] departure complete: arm already started; drive DIRECT to RAW concurrently\r\n");
  }
  if(success && zdt_match.state==MATCH_MOVING && recognition_entry && zdt_arm_camera_token &&
     !(zdt_match.checkpoint==MATCH_QR && zdt_qr_enabled)) {
    (void)ChassisMotor_Stop();zdt_match_io=MATCH_IO_ARM_DEPLOY;
    ZDT_Console_Print(zdt_match.checkpoint!=MATCH_QR?"[MATCH] station arrived; waiting CAMERA READY before visual alignment\r\n":
        "[MATCH] QR arrived; waiting arm READY before camera recognition\r\n");return;
  }
  if(success && zdt_match.state==MATCH_MOVING && recognition_entry && zdt_match.checkpoint<=MATCH_TEMP2 &&
     !(zdt_match.checkpoint==MATCH_QR && zdt_qr_enabled) && (!ArmTask_IsEnabled() || ArmTask_IsFaulted())) {
    ZDT_Match_Complete(0U,"arm DM hold lost before recognition");return;
  }
  if(success && zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==MATCH_QR &&
     zdt_qr_enabled && zdt_qr_active) {
    if(zdt_qr_for_match==2U && zdt_qr_scan.phase==QR_SCAN_READ)zdt_qr_scan.read_started=HAL_GetTick();
    zdt_qr_for_match=1U;zdt_match_io=MATCH_IO_QRSCAN;
    ZDT_Console_Print("[MATCH] QR pose arrived; existing scan continues, no duplicate SET; wait task/STOP ACK only\r\n");return;
  }
  if(success && zdt_match.state==MATCH_MOVING && recognition_entry && zdt_match.checkpoint<=MATCH_TEMP2 &&
     (zdt_align_enabled || zdt_qr_enabled) && ZDT_Camera_ResetBusy()) {
    if(zdt_match_io!=MATCH_IO_CAMERA_RESET) {
      (void)ChassisMotor_Stop();zdt_match_io=MATCH_IO_CAMERA_RESET;zdt_camera_reset_wait_tick=HAL_GetTick();
      ZDT_Camera_Reset();ZDT_Console_Print("[MATCH] retry camera STOP synchronization before recognition\r\n");
    }
    return;
  }
  if(zdt_match_io==MATCH_IO_ALIGN && zdt_align_diag.active) {
    if(!success) {
      vision_pixel_t p={0};uint32_t age=UINT32_MAX;
      (void)Vision_GetPixel(zdt_align_diag.config.camera,zdt_align_diag.config.type,&p,&age);
      ZDT_Align_DiagObserve(HAL_GetTick(),&p,age,0.0f,0.0f,0.0f);
    }
    zdt_align_diag.can_failures=zdt_can_enqueue_failures-zdt_diag_can_start;
    VisionDiag_Finish(&zdt_align_diag,HAL_GetTick(),success,reason);
  }
  if(!success && zdt_zone2.running) {
    zdt_zone2_reason=reason;
    if(zdt_zone2.active_id) { Zone2Route_Result(&zdt_zone2,zdt_zone2.active_id,0U); return; }
    Zone2Route_Cancel(&zdt_zone2);
  }
  uint32_t request = zdt_match_request;
  if (request == 0U) { return; }
  if(success && zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==MATCH_QR &&
      zdt_qr_enabled && !zdt_qr_locked) {
    if(!ZDT_Qr_Start(1U))ZDT_Match_Complete(0U,"QR scan start rejected");
    return;
  }
  if(success && zdt_match.automatic && zdt_match.state==MATCH_MOVING && zdt_match.checkpoint==MATCH_QR &&
     zdt_qr_enabled && zdt_qr_locked && !zdt_qr_active) {
    /* Next RAW dispatch overlaps CAM1 SET with driving. CAM2 QR needs no
     * CAM1 recognition or completed CAMERA arm pose. */
    zdt_match_io=MATCH_IO_CAMERA;
    ZDT_Console_Print("[MATCH] QR task/STOP ready; continue RAW NOW, CAMERA restoration still parallel if pending\r\n");
  }
  if(success && zdt_match.state==MATCH_MOVING && zdt_match_io!=MATCH_IO_ALIGN && zdt_match_io!=MATCH_IO_CAMERA &&
      zdt_align_enabled && ZDT_Align_Station()!=0U) {
    if(!ZDT_Align_Start(ZDT_Align_Station()))ZDT_Match_Complete(0U,"visual alignment start rejected");
    return;
  }
  if(success && (zdt_match.first_round_only || zdt_match.automatic) && zdt_match.state==MATCH_MOVING &&
     (zdt_match_io==MATCH_IO_ALIGN || (!zdt_align_enabled && zdt_match_io!=MATCH_IO_CAMERA && zdt_match_io!=MATCH_IO_ROUND1)) &&
     (zdt_match.checkpoint>=MATCH_RAW1 && zdt_match.checkpoint<=MATCH_TEMP2)) {
    VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);zdt_align_phase=0U;
    (void)ChassisMotor_Stop();zdt_match_io=MATCH_IO_ROUND1;zdt_arm_round_token=0U;
    ZDT_Console_Print(zdt_match.automatic?"[MATCH] station aligned: ROUND1 owns arm/camera; task success advances automatically\r\n":
        "[MATCH] station aligned: ROUND1 owns arm/camera; automatic three-item task before OK\r\n");return;
  }
  if(success && zdt_match.state==MATCH_MOVING && zdt_match_io==MATCH_IO_ALIGN && ArmMaterial_AutoEnabled() &&
     zdt_align.config.camera==1U && zdt_align.config.type==VISION_TYPE_PLATE_PX &&
     (zdt_match.checkpoint==MATCH_RAW1 || zdt_match.checkpoint==MATCH_RAW2)) {
    VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);zdt_align_phase=0U;
    (void)ChassisMotor_Stop();zdt_match_io=MATCH_IO_MATERIAL;zdt_arm_material_token=0U;
    ZDT_Console_Print("[MATCH] PLATE aligned: material RED job owns arm/camera; wait loading tray1 before OK\r\n");return;
  }
  if(success && zdt_match.state==MATCH_MOVING && zdt_match_io!=MATCH_IO_CAMERA && zdt_align_enabled &&
      (zdt_match.checkpoint==MATCH_QR || zdt_match.checkpoint==MATCH_RAW1 || zdt_match.checkpoint==MATCH_RAW2)) {
    uint8_t station=zdt_match.checkpoint==MATCH_QR?1U:2U;
    VisionAlign_Cancel(&zdt_align);VisionYaw_Cancel(&zdt_vyaw);zdt_align_phase=0U;
    (void)ChassisMotor_Stop();zdt_match_io=MATCH_IO_CAMERA;
    ZDT_Align_RequestCameraMode(station);return;
  }
  if(zdt_match_io==MATCH_IO_ALIGN) {
    zdt_align_phase=0U;
    if(!success)(void)ChassisMotor_StopImmediate();
    VisionAlign_Cancel(&zdt_align);
  }
  zdt_match_io = MATCH_IO_NONE;
  zdt_match_request = 0U;
  if ((success == 0U) && (reason != NULL)) {
    ZDT_Console_Print("[MATCH] motion failed: ");
    ZDT_Console_Print(reason);
    ZDT_Console_Print("\r\n");
  }
  MatchControl_Result(&zdt_match, request, success, HAL_GetTick());
}

static void ZDT_MaterialService(void)
{
  if(zdt_match_io==MATCH_IO_ROUND1 && zdt_match_request) {
    if(zdt_qr_enabled && !zdt_qr_locked){ZDT_Match_Complete(0U,"station task needs locked QR mission");return;}
    if(zdt_arm_round_token && ArmRound1_IsActive())zdt_match.entered_tick=HAL_GetTick();
    if(!zdt_arm_round_token) {
      uint8_t station=(uint8_t)zdt_match.checkpoint; /* RAW1..TEMP2 map to jobs1..6. */
      HAL_StatusTypeDef result=ArmTask_RequestRound1(zdt_match_request,station);
      if(result==HAL_OK)zdt_arm_round_token=zdt_match_request;
      else if(result!=HAL_BUSY)ZDT_Match_Complete(0U,"station task request rejected");
    } else {
      uint8_t success=0U;
      if(ArmTask_GetRound1Result(zdt_arm_round_token,&success)) {
        if(success) {
          zdt_arm_camera_token=zdt_arm_round_token;zdt_arm_camera_started=HAL_GetTick();
          ZDT_Console_Print("[MATCH] lift TOP clear; next drive and CAMERA restoration run concurrently\r\n");
        }
        zdt_arm_round_token=0U;
        if(success){zdt_match_io=MATCH_IO_CAMERA;ZDT_Match_Complete(1U,NULL);}
        else ZDT_Match_Complete(0U,"station task failed; arm round1 status");
      }
    }
    return;
  }
  if(zdt_match_io==MATCH_IO_MATERIAL && zdt_match_request) {
    /* Material waiting is a separate owner job, not unfinished navigation.
     * Its camera liveness and mechanical-stage deadlines remain enforced. */
    if(zdt_arm_material_token && ArmMaterial_IsActive())zdt_match.entered_tick=HAL_GetTick();
    if(!zdt_arm_material_token) {
      HAL_StatusTypeDef result=ArmTask_RequestMaterial(zdt_match_request);
      if(result==HAL_OK)zdt_arm_material_token=zdt_match_request;
      else if(result!=HAL_BUSY)ZDT_Match_Complete(0U,"material arm request rejected");
    } else {
      uint8_t success=0U;
      if(ArmTask_GetMaterialResult(zdt_arm_material_token,&success)) {
        zdt_arm_material_token=0U;
        if(success){zdt_match_io=MATCH_IO_CAMERA;ZDT_Match_Complete(1U,NULL);}
        else ZDT_Match_Complete(0U,"material red grab/place failed; arm material status");
      }
    }
  }
}

static uint8_t ZDT_Match_StartMotion(match_motion_t motion, uint8_t zone,
                                     uint32_t request)
{
  char line[112];
  if(ArmMaterial_IsActive() || ArmRound1_IsActive())return MATCH_START_BUSY;
  if(motion==MATCH_MOTION_PREPARE) {
    HAL_StatusTypeDef result;uint32_t now=HAL_GetTick();
    if(zdt_script_running || ZDT_Nav_IsActive()) {
      ZDT_Console_Print("[MATCH] prepare rejected: navigation/script still active\r\n");return MATCH_START_REJECTED;
    }
    if(zdt_prepare_request!=request) {
      zdt_prepare_request=request;zdt_prepare_tick=zdt_prepare_next=now;
      zdt_prepare_stopped=zdt_prepare_retry_printed=0U;
    }
    if(!zdt_prepare_stopped) {
      if((int32_t)(now-zdt_prepare_next)<0)return MATCH_START_BUSY;
      result=ChassisMotor_DisableForArmPreparation();
      if(result!=HAL_OK) {
        if(now-zdt_prepare_tick>=2000U) {
          (void)snprintf(line,sizeof(line),"[MATCH] prepare rejected: wheels-only disable HAL=%u FIFO=%lu CANerr=0x%08lX\r\n",
            (unsigned int)result,(unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2),(unsigned long)hfdcan2.ErrorCode);
          ZDT_Console_Print(line);return MATCH_START_REJECTED;
        }
        if(!zdt_prepare_retry_printed) {
          (void)snprintf(line,sizeof(line),"[MATCH] prepare: wheels-only disable HAL=%u; retry for <=2s, DM unaffected\r\n",(unsigned int)result);
          ZDT_Console_Print(line);zdt_prepare_retry_printed=1U;
        }
        zdt_prepare_next=HAL_GetTick()+100U;return MATCH_START_BUSY;
      }
      zdt_prepare_stopped=1U;
    }
    /* prepare resets only the IMU bias; keep the robot still until ready. */
    if(!HWT101_IsZeroDriftReady())return MATCH_START_BUSY;
    result=ArmTask_RequestStartPose(request);
    if(result==HAL_BUSY)return MATCH_START_BUSY;
    if(result!=HAL_OK) {
      (void)snprintf(line,sizeof(line),"[MATCH] prepare rejected: ARM request HAL=%u\r\n",(unsigned int)result);
      ZDT_Console_Print(line);return MATCH_START_REJECTED;
    }
    zdt_arm_start_token=zdt_match_request=request;zdt_match_io=MATCH_IO_ARM_START;
    ZDT_Console_Print("[MATCH] restore arm launch pose first; chassis stays OFF, OK after READY\r\n");return MATCH_START_ACCEPTED;
  }
  if(motion!=MATCH_MOTION_LEAVE && motion!=MATCH_MOTION_QR &&
     !(zdt_match.automatic && motion==MATCH_MOTION_RAW && zdt_match.checkpoint==MATCH_RAW1) &&
     (zdt_align_enabled || zdt_qr_enabled) && ZDT_Camera_ResetBusy()) {
    uint8_t i;vision_camera_state_t state;
    for(i=0U;zdt_align_enabled && i<3U;i++)if(Vision_GetCameraState(zdt_align_config[i].camera,&state) &&
        state.reset_pending && !state.pending && state.result!=255U) {
      ZDT_Console_Print("[MATCH] camera STOP reset failed; retry match reset before motion.\r\n");
      return MATCH_START_REJECTED;
    }
    if(zdt_qr_enabled && Vision_GetCameraState(2U,&state) && state.reset_pending && !state.pending && state.result!=255U)
      return MATCH_START_REJECTED;
    return MATCH_START_BUSY;
  }
  if (ChassisMotor_IsEnabled() == 0U) {
    ZDT_Console_Print("[MATCH] motors disabled; send &EN.\r\n");
    return MATCH_START_REJECTED;
  }
  /* Finishing read-only mileage work is transient, not a rejected move.
   * Wait before claiming the request or starting a fresh motion baseline. */
  if(zdt_mileage_query_active || zdt_mileage_stop_pending)return MATCH_START_BUSY;
  if(zdt_script_running || ZDT_Nav_IsActive() || ChassisMotor_IsTurnActive()) {
    ZDT_Console_Print("[MATCH] motion service already executing another move.\r\n");
    return MATCH_START_REJECTED;
  }
  if(zdt_align_enabled && (motion==MATCH_MOTION_RAW || motion==MATCH_MOTION_ROUGH || motion==MATCH_MOTION_TEMP)) {
    uint8_t station=motion==MATCH_MOTION_RAW?1U:(motion==MATCH_MOTION_ROUGH?2U:3U);
    vision_align_config_t *c=&zdt_align_config[station-1U];
    vision_mode_gate_t desired;vision_camera_state_t state={0};
    VisionModeGate_Start(&desired,c->camera,c->type==VISION_TYPE_PLATE_PX?VISION_MODE_PLATE:VISION_MODE_RING,
      station,c->id,HAL_GetTick());
    desired.flags=ZDT_Align_Flags(desired.mode,station);
    if(!Vision_GetCameraState(c->camera,&state))return MATCH_START_REJECTED;
    /* Overlap camera setup with transport, but arrival still gates on ACK. */
    if(!state.pending && !VisionModeGate_Matches(&desired,&state)) {
      HAL_StatusTypeDef result=Vision_RequestMode(c->camera,desired.mode,station,c->id,20U,desired.flags);
      if(result!=HAL_OK && result!=HAL_BUSY)return MATCH_START_REJECTED;
    }
  }
  zdt_match_request = request;
  if (motion == MATCH_MOTION_LEAVE) {
    zdt_match_io = MATCH_IO_LEAVE_START_QUERY;
    /* Fresh four-wheel baseline prevents timed leave from reporting a dead bus
     * as success. Reuse the existing bounded position-query service. */
    ZDT_Mileage_StartQuery(0U);
    return 1U;
  }
  zdt_match_io=MATCH_IO_ZONE2;zdt_zone2_reason=NULL;
  (void)snprintf(line,sizeof(line),"[MATCH] FIXED_OUTER_V3 zone=%u: waypoints + facing + signed compensation\r\n",
    (unsigned int)zone);ZDT_Console_Print(line);
  if(motion==MATCH_MOTION_RETURN && zone==1U)
    ZDT_Console_Print("[RETURN] QR90_V1: east -> QR -> south staging\r\n");
  return Zone2Route_StartForZone(&zdt_zone2,motion,zdt_match.checkpoint,zone);
}

static uint8_t ZDT_DepartureParallelStart(void)
{
  if(ArmTask_RequestCameraDeploy(zdt_match_request)!=HAL_OK)return 0U;
  zdt_arm_camera_token=zdt_match_request;zdt_arm_camera_started=HAL_GetTick();
  if(zdt_match.qr_required && zdt_qr_enabled && !ZDT_Qr_Start(2U))return 0U;
  ZDT_Console_Print("[MATCH] OK departure: CAMERA deploy and CAM2 scan start NOW alongside chassis; START preparation unchanged\r\n");
  return 1U;
}
static void ZDT_Match_OnMileage(uint8_t mask)
{
  uint8_t i;
  if(zdt_match_io==MATCH_IO_ALIGN) { ZDT_Align_OnMileage(mask);return; }
  if(zdt_match_io==MATCH_IO_ZONE2) { ZDT_Zone2_OnMileage(mask); return; }
  if ((zdt_match_io != MATCH_IO_LEAVE_START_QUERY) &&
      (zdt_match_io != MATCH_IO_LEAVE_END_QUERY)) {
    if ((zdt_match_io == MATCH_IO_NAV) && (mask != 0x0FU)) {
      ZDT_Match_Complete(0U, "navigation needs all 4 position replies");
    }
    return;
  }
  if (mask != 0x0FU) {
    ZDT_Match_Complete(0U, "leave needs fresh position replies from all 4 wheels");
    return;
  }
  if (zdt_match_io == MATCH_IO_LEAVE_START_QUERY) {
    memcpy(zdt_match_leave_start, zdt_mileage_query_position,
           sizeof(zdt_match_leave_start));
    if(!ZDT_DepartureParallelStart()){ZDT_Match_Complete(0U,"departure arm/QR parallel start rejected");return;}
    zdt_match_io = MATCH_IO_LEAVE_SCRIPT;
    ZDT_Script_StartLeave();
    if (zdt_script_running == 0U) { ZDT_Match_Complete(0U, "leave rejected"); }
  } else {
    for (i = 0U; !zdt_vobs_depart_detoured && i < ZDT_MILEAGE_MOTOR_COUNT; ++i) {
      int64_t delta = (int64_t)zdt_mileage_query_position[i] - zdt_match_leave_start[i];
      double mm = fabs((double)delta) * ZDT_MILEAGE_MM_PER_REV /
                    Nav_Params()->counts_per_rev;
      if (mm < 10.0) {
        ZDT_Match_Complete(0U, "leave finished by timer but wheel movement not confirmed");
        return;
      }
    }
    /* Checked detour segments already completed with four wheel feedback;
     * turning can cancel a wheel's net displacement from the original baseline. */
    zdt_vobs_depart_detoured=0U;
    ZDT_Match_Complete(1U, NULL);
  }
}

static void ZDT_Match_OnScriptDone(void)
{
  if (zdt_match_io == MATCH_IO_LEAVE_SCRIPT) {
    zdt_match_io = MATCH_IO_LEAVE_END_QUERY;
    ZDT_Mileage_StartQuery(0U);
  }
}

static uint8_t ZDT_Match_CommandWord(const char *command, const char *word)
{
  size_t n = strlen(word);
  return (uint8_t)((strncmp(command, word, n) == 0) &&
                   ((command[n] == '\0') || (command[n] == ' ') ||
                    (command[n] == '\t') ||
                    ((command[n] >= '0') && (command[n] <= '9')) ||
                    (command[n] == '-') || (command[n] == '+')));
}

static uint8_t ZDT_Match_Command(const char *command, uint32_t received_tick)
{
  static const char *const owned[] = {
    "goto", "path", "pose", "posemm", "gotomm", "leave", "home", "cross",
    "nudge", "zero", "run", "test", "zpos", "poscal", "posq", "posref", "posd",
    "enable"
  };
  uint8_t i;
  command = ZDT_SkipSpaces(command);
  if (*command == '&') { command = ZDT_SkipSpaces(command + 1U); }
  if(ZDT_Vobs_Command(command,received_tick))return 1U;
  if(!strcmp(command,"stop") || !strcmp(command,"disable") || !strcmp(command,"dis") ||
      !strcmp(command,"off") || !strcmp(command,"match reset"))ZDT_Vobs_Reset();
  if(zdt_vobs_phase)for(i=0U;i<(uint8_t)(sizeof(owned)/sizeof(owned[0]));i++)
    if(ZDT_Match_CommandWord(command,owned[i])) {
      ZDT_Console_Print("[VOBS] original route paused; clear/free then vobs retry, or STOP/reset before a new motion\r\n");
      return 1U;
    }
  if(ArmMaterial_IsActive() || ArmRound1_IsActive()) {
    uint8_t blocked=(uint8_t)(ZDT_Match_CommandWord(command,"ok") || ZDT_Match_CommandWord(command,"prepare"));
    for(i=0U;i<(uint8_t)(sizeof(owned)/sizeof(owned[0]));i++)if(ZDT_Match_CommandWord(command,owned[i]))blocked=1U;
    if(blocked){ZDT_Console_Print("[ARM] station/material job active; wait STATION_DONE or STOP before chassis commands\r\n");return 1U;}
  }
  if((strcmp(command,"arm material")==0 || strcmp(command,"arm material red")==0 ||
      strcmp(command,"arm material loop start")==0) &&
     ((MatchControl_IsActive(&zdt_match) && zdt_match.state!=MATCH_WAIT_OK) || zdt_script_running || ZDT_Nav_IsActive())) {
    ZDT_Console_Print("[MAT] manual start waits for stopped chassis/WAIT_OK; auto on hooks after PLATE alignment\r\n");return 1U;
  }
  /* Manual pose/zero commands cannot replace the automatic running job. */
  if(((MatchControl_IsActive(&zdt_match) && zdt_match.state!=MATCH_WAIT_START_OK) ||
      zdt_script_running || ZDT_Nav_IsActive()) &&
     (ZDT_Match_CommandWord(command,"arm camera") || ZDT_Match_CommandWord(command,"arm start") ||
      ZDT_Match_CommandWord(command,"arm zero") || ZDT_Match_CommandWord(command,"dm zero") || ZDT_Match_CommandWord(command,"arm color")) &&
     strcmp(command,"arm camera status") && strcmp(command,"arm start status") && strcmp(command,"arm color status") && strcmp(command,"arm color stop")) {
    ZDT_Console_Print("[MATCH] manual pose/zero waits until stopped; arm query/status and disable remain available\r\n");return 1U;
  }
  if(zdt_manual_align_station && strcmp(command,"stop") && strcmp(command,"disable") && strcmp(command,"dis") &&
     strcmp(command,"off") && strcmp(command,"match reset")) {
    ZDT_Console_Print("[MANUAL ALIGN] busy; STOP cancels this correction\r\n");return 1U;
  }
  if(ZDT_VyawCommand(command))return 1U;
  if(ZDT_Outer_Command(command))return 1U;
  if(ZDT_Qr_Command(command))return 1U;
  if(zdt_qr_active && strcmp(command,"stop") && strcmp(command,"disable") && strcmp(command,"dis") &&
      strcmp(command,"off") && strcmp(command,"match reset") && strcmp(command,"arm stop") &&
      strncmp(command,"arm disable ",12U) && strcmp(command,"dm off") && strcmp(command,"dm disable"))return 1U;
  if((zdt_match_io==MATCH_IO_ALIGN || zdt_match_io==MATCH_IO_CAMERA) && strcmp(command,"stop")!=0 &&
      strcmp(command,"disable")!=0 && strcmp(command,"dis")!=0 && strcmp(command,"off")!=0 &&
      strcmp(command,"match reset")!=0) {
    /* No blocking bulk diagnostics or manual motion while vision owns the
     * chassis. Requested status is deferred to a genuinely stopped interval. */
    if(zdt_match_io==MATCH_IO_ALIGN) {
      if(ZDT_Match_CommandWord(command,"ok"))zdt_align_status_pending|=3U;
      else if(strcmp(command,"match")==0 || strcmp(command,"match status")==0 ||
          strcmp(command,"valign")==0 || strcmp(command,"valign status")==0)
        zdt_align_status_pending|=1U;
      else if(strncmp(command,"valign trace",12U)==0)zdt_align_status_pending|=5U;
    }
    return 1U;
  }
  if(ZDT_Align_Command(command))return 1U;
  if(strcmp(command,"dis")==0 || strcmp(command,"off")==0 || strcmp(command,"disable")==0)
    ZDT_Camera_Reset();
  if(strcmp(command,"stop")==0)ArmTask_RequestStop();
  if((zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) &&
      strcmp(command,"stop")!=0 && strcmp(command,"disable")!=0 &&
      strcmp(command,"dis")!=0 && strcmp(command,"off")!=0 && strcmp(command,"match reset")!=0) {
    /* 9600-baud diagnostic prints would block the 4ms field conversion loop. */
    zdt_nav_blend_deferred_input=1U; return 1U;
  }
  if (strncmp(command, "endturn", 7U) == 0) {
    zdt_nav_endturn_enabled=0U;
    ZDT_Console_Print("[NAV] endturn disabled: stop before heading changes; no turn+move blend.\r\n");
    return 1U;
  }
  if ((strncmp(command, "dcomp", 5U) == 0) &&
      ((ZDT_Nav_IsActive() != 0U) || (zdt_script_running != 0U) ||
       (zdt_match.state == MATCH_MOVING) || (zdt_match.state == MATCH_LEAVING))) {
    ZDT_Console_Print("[NAV] set dcomp when stopped or at WAIT_OK.\r\n");
    return 1U;
  }
  if ((strncmp(command, "startpose", 9U) == 0) ||
      (strncmp(command, "startzone", 9U) == 0)) {
    const char *arg = ZDT_SkipSpaces(command + 9U);
    if (MatchControl_IsActive(&zdt_match) != 0U) {
      ZDT_Console_Print("[MATCH] start zone is locked until reset.\r\n");
    } else if (((*arg != '1') && (*arg != '2')) ||
               (*ZDT_SkipSpaces(arg + 1U) != '\0')) {
      ZDT_Console_Print("[MATCH] usage: startpose 1|2\r\n");
    } else { ZDT_Script_SetStartZone((uint8_t)(*arg - '0')); }
    return 1U;
  }
  if(ZDT_Match_CommandWord(command,"prepare")) {
    const char *arg=ZDT_SkipSpaces(command+7U);
    if(*arg && ((*arg!='1' && *arg!='2') || *ZDT_SkipSpaces(arg+1U)!='\0')) {
      ZDT_Console_Print("[MATCH] usage: prepare; arm START + gyro bias only; select zone at OK\r\n");return 1U;
    }
    if(MatchControl_IsActive(&zdt_match) || zdt_script_running || ZDT_Nav_IsActive() ||
       zdt_poscal_phase!=ZDT_POSCAL_IDLE || zdt_mileage_query_active) {
      ZDT_Console_Print("[MATCH] prepare busy: finish current motion or STOP/reset first\r\n");return 1U;
    }
    ZDT_Qr_Cancel();ScreenMetrics_ClearTask();MatchControl_Reset(&zdt_match);ArmTask_RequestRound1Reset();
    zdt_start_pose_set=0U;ChassisMotor_ClearHeadingReference();HWT101_ResetZeroDrift();
    (void)MatchControl_Prepare(&zdt_match,0U,HAL_GetTick());
    ZDT_Console_Print("[MATCH] prepare accepted: IMU settle -> arm START -> wait OK; zone UNBOUND, position not confirmed\r\n");return 1U;
  }
  if (strcmp(command, "depart") == 0) {
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    if(zdt_match.state==MATCH_PREPARING || zdt_match.state==MATCH_WAIT_START_OK) {
      ZDT_Console_Print("[MATCH] launch preparation already requested; wait READY then send OK\r\n");return 1U;
    }
    if((zdt_align_enabled || zdt_qr_enabled) && ZDT_Camera_ResetBusy())
      ZDT_Console_Print("[MATCH] camera STOP not confirmed; defer synchronization to QR after arm deployment\r\n");
    if ((zdt_match.state != MATCH_IDLE) || (zdt_start_pose_set == 0U) ||
        (HWT101_IsZeroDriftReady() == 0U) || (zdt_script_running != 0U) ||
        (ZDT_Nav_IsActive() != 0U) || (zdt_poscal_phase != ZDT_POSCAL_IDLE) ||
        (zdt_mileage_query_active != 0U)) {
      ZDT_Console_Print("[MATCH] depart refused: need IDLE, startpose N, IMU ready and idle chassis.\r\n");
      return 1U;
    }
    if ((zdt_status.can_ready == 0U) ||
        (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol) != HAL_OK) ||
        (protocol.BusOff != 0U) || (protocol.ErrorPassive != 0U)) {
      ZDT_Console_Print("[MATCH] depart refused: chassis CAN is not healthy.\r\n");
      return 1U;
    }
    (void)MatchControl_Prepare(&zdt_match, zdt_start_zone, HAL_GetTick());
    return 1U;
  }
  if (ZDT_Match_CommandWord(command,"ok")) {
    const char *arg=ZDT_SkipSpaces(command+2U);uint8_t zone=1U;
    if(*arg) {
      if((*arg!='1' && *arg!='2') || *ZDT_SkipSpaces(arg+1U)) {
        ZDT_Console_Print("[MATCH] usage: OK [1|2]; bare OK selects zone1 at launch\r\n");return 1U;
      }
      zone=(uint8_t)(*arg-'0');
    }
    if(zdt_match.state==MATCH_PREPARING) {
      ZDT_Console_Print("[MATCH] OK ignored: arm launch pose is still preparing\r\n");return 1U;
    }
    if(zdt_match.state==MATCH_WAIT_START_OK) {
      FDCAN_ProtocolStatusTypeDef protocol={0};
      if((int32_t)(received_tick-zdt_match.entered_tick)<=0) {
        ZDT_Console_Print("[MATCH] old OK ignored: send a new OK after launch READY\r\n");return 1U;
      }
      if(!ArmTask_IsStartPoseReady()) {
        ZDT_Console_Print("[MATCH] start OK refused: need fresh measured launch pose; arm status/arm start status\r\n");return 1U;
      }
      if(HWT101_IsZeroDriftReady()==0U || zdt_status.can_ready==0U ||
         HAL_FDCAN_GetProtocolStatus(&hfdcan2,&protocol)!=HAL_OK || protocol.BusOff || protocol.ErrorPassive ||
         (!ChassisMotor_IsEnabled() && ChassisMotor_Enable()!=HAL_OK)) {
        ZDT_Console_Print("[MATCH] start OK refused: IMU/CAN/enable unavailable; chassis does not depart\r\n");return 1U;
      }
      if(ZDT_ConfirmLaunchZone(zone)!=HAL_OK) {
        ZDT_Console_Print("[MATCH] start OK refused: stopped pose/heading rebase unavailable; zone not bound\r\n");return 1U;
      }
      zdt_match.zone=zone;
    }
    if(zdt_qr_enabled && zdt_match.checkpoint==MATCH_QR && zdt_match.state==MATCH_WAIT_OK && !zdt_qr_locked) {
      ZDT_Console_Print("[QR] no task locked; qrscan start here, or reset and run with qrscan on.\r\n");return 1U;
    }
    if (MatchControl_Ok(&zdt_match, received_tick) == 0U) {
      char line[144];
      (void)snprintf(line,sizeof(line),"[MATCH] ok ignored: state=%s checkpoint=%s; %s\r\n",
          MatchControl_StateName(zdt_match.state),MatchControl_CheckpointName(zdt_match.checkpoint),
          zdt_match.state==MATCH_FAULT?"inspect arm camera/status fault before reset":
          zdt_match.automatic?"automatic run; no further OK required":"wait for WAIT_OK, then send a new ok");
      ZDT_Console_Print(line);
    }
    return 1U;
  }
  if ((strcmp(command, "match") == 0) || (strcmp(command, "match status") == 0)) {
    ZDT_Match_Changed(zdt_match.state, zdt_match.checkpoint);
    return 1U;
  }
  if (strcmp(command, "match reset") == 0) {
    ZDT_Qr_Cancel();
    ScreenMetrics_ClearTask();MatchControl_Reset(&zdt_match);
    ArmTask_RequestRound1Reset();
    zdt_start_pose_set = 0U;
    ZDT_Console_Print("[MATCH] reset two-round inventory EMPTY; empty tray physically, then prepare.\r\n");
    return 1U;
  }
  if (ZDT_Match_CommandWord(command, "match") != 0U) {
    ZDT_Console_Print("[MATCH] commands: prepare, OK [1|2]; startpose 1|2, depart; match status/reset, STOP.\r\n");
    return 1U;
  }
  if (MatchControl_IsActive(&zdt_match) != 0U) {
    if ((strcmp(command, "stop") == 0) || (strcmp(command, "disable") == 0) ||
        (strcmp(command, "dis") == 0) ||
        (strcmp(command, "off") == 0)) {
      MatchControl_Abort(&zdt_match);
      return 0U; /* Existing handler still stops/disables the actual motors. */
    }
    for (i = 0U; i < (uint8_t)(sizeof(owned) / sizeof(owned[0])); ++i) {
      if (ZDT_Match_CommandWord(command, owned[i]) != 0U) {
        ZDT_Console_Print("[MATCH] chassis owned by flow; use ok or STOP.\r\n");
        return 1U;
      }
    }
  }
  return 0U;
}

/* ---------- 规划代价模型：让 A* 按"真实时间"比较路线 ----------
 *
 * 现场观察（2026-09-13）：`temp→rough` **两条路的曼哈顿长度完全相同**（都 34 格），
 * 区别只在"90°+90° 走外圈"还是"180°+90° 掉头走中间"，而原来的代价里 90° 与 180°
 * 同价 → A* 只能随手选一条（现场看到的正是"该走外圈却掉头走中间"）。
 *
 * 改成按时间比较：一格的时间 = 当前 `spd` 下走 50 mm 的规划时间；转向时间**取运行参数**
 * `t90` / `t180`（`&t90<ms>` / `&t180<ms>`，存 flash）—— 现场实测后填，不用重编译。
 * 单位仍取"格数当量"（走一格 = GRID_NAV_MOVE_COST），这样 A* 的 g/f 不会溢出 uint16。 */

/* 圆弧的驱动时长：与 ZDT_Nav_PlanSeconds() 同一套梯形/三角形解，只是加减速要用
   "按轮速上限缩放后"的值（圆弧时底盘 Mix 会整体缩放，斜坡也跟着按比例变慢；
   不这么算会提前停车、把车停在角点之前）。 */
static float ZDT_Nav_ArcPlanMs(float distance_mm, float v_mm_s,
                               float accel_mm_s2, float decel_mm_s2)
{
  float accel_mm;
  float decel_mm;

  if (v_mm_s < 1.0f) { v_mm_s = 1.0f; }
  if (distance_mm < 0.0f) { distance_mm = -distance_mm; }
  accel_mm = (v_mm_s * v_mm_s) / (2.0f * accel_mm_s2);
  decel_mm = (v_mm_s * v_mm_s) / (2.0f * decel_mm_s2);
  if (distance_mm >= (accel_mm + decel_mm))
  {
    return ((v_mm_s / accel_mm_s2) + (v_mm_s / decel_mm_s2) +
            ((distance_mm - accel_mm - decel_mm) / v_mm_s)) * 1000.0f;
  }
  {
    float peak = sqrtf((2.0f * distance_mm) /
                       ((1.0f / accel_mm_s2) + (1.0f / decel_mm_s2)));

    return ((peak / accel_mm_s2) + (peak / decel_mm_s2)) * 1000.0f;
  }
}

/* 圆弧上所有采样位姿是否都安全（含两端：起点 = 上一段末尾，终点 = 下一段开头）。
 *
 * 几何（两种转向都成立，转的方向由 w-u 隐含，不需要分左右）：
 *   S = corner − R·u        （角点前 R，在本段车道中心线上）
 *   E = corner + R·w        （角点后 R，在下一段车道中心线上）
 *   C = corner + R·(w − u)  （圆心，与两条车道线都相切）
 * 车头角从 u 转到 w，转角 = 路径扫过的角度（两者同号同大小）。 */
static uint8_t ZDT_Nav_ArcGeometryOk(float corner_x, float corner_y,
                                     float ux, float uy, float wx, float wy,
                                     float radius, float *delta_yaw_deg)
{
  float cx = corner_x + (radius * (wx - ux));
  float cy = corner_y + (radius * (wy - uy));
  float sx = corner_x - (radius * ux);
  float sy = corner_y - (radius * uy);
  float ex = corner_x + (radius * wx);
  float ey = corner_y + (radius * wy);
  float a0 = atan2f(sy - cy, sx - cx);
  float a1 = atan2f(ey - cy, ex - cx);
  float sweep = a1 - a0;
  float yaw0 = atan2f(uy, ux);
  uint8_t i;

  while (sweep > 3.14159265f) { sweep -= 6.28318531f; }
  while (sweep < -3.14159265f) { sweep += 6.28318531f; }
  /* 四分之一圆：扫角必须≈±90°（同向段已被 grid_nav 合并、掉头不走圆弧，
     所以这里只有这一种可能；不成立说明段方向算错了，直接拒绝）。 */
  if (fabsf(fabsf(sweep) - 1.57079633f) > 0.05f)
  {
    return 0U;
  }

  for (i = 0U; i < ZDT_NAV_ARC_SAMPLES; ++i)
  {
    float t = (float)i / (float)(ZDT_NAV_ARC_SAMPLES - 1U);
    float a = a0 + (sweep * t);
    float px = cx + (radius * cosf(a));
    float py = cy + (radius * sinf(a));
    int16_t yaw_deg = (int16_t)lroundf((yaw0 + (sweep * t)) * 57.2957795f);

    if ((px < 0.0f) || (py < 0.0f) ||
        (px > (float)GRID_NAV_FIELD_MM) || (py > (float)GRID_NAV_FIELD_MM))
    {
      return 0U;
    }
    if (GridNav_RotatedBodyFreeMm((uint16_t)lroundf(px), (uint16_t)lroundf(py),
                                  yaw_deg) == 0U)
    {
      return 0U;
    }
  }
  if (delta_yaw_deg != NULL)
  {
    *delta_yaw_deg = sweep * 57.2957795f;
  }
  return 1U;
}

/* 在"本段起点"决定：本段末尾那个直角能不能改成圆弧。能则武装（并把本段目标距离减 R）。
 * 每次进入某一段只调用一次（由 WAIT_START_QUERY 的 align_done==0 那道门保证）。 */
static uint8_t ZDT_Nav_TryArmArc(void)
{
  const grid_nav_segment_t *cur = &zdt_nav_segments[zdt_nav_segment_index];
  const grid_nav_segment_t *nxt;
  float v_max = (float)CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM;
  float scale;
  float v_arc;
  float r_req;
  float r;
  float ux;
  float uy;
  float wx;
  float wy;
  float corner_x;
  float corner_y;
  float delta_yaw = 0.0f;
  float mix;
  float v_used;
  float rpm;
  float ms;
  float leg_cur_mm;
  float leg_next_mm;
  float pose_x;
  float pose_y;
  uint16_t pose_mm_x = 0U;
  uint16_t pose_mm_y = 0U;
  char line[176];

  if(zdt_match_io==MATCH_IO_ZONE2) return 0U;
  if ((Nav_Params()->arc90_ms <= 0.0f) || (Nav_Params()->position_mode != 0U))
  {
    return 0U;
  }
  if ((zdt_nav_segment_index + 1U) >= zdt_nav_segment_count)
  {
    return 0U;
  }
  nxt = &zdt_nav_segments[zdt_nav_segment_index + 1U];
  /* 只做**直角**：同向段已合并（点积 >0），掉头（<0）仍原地转 ——
     180° 圆弧扫过的面积是 90° 的两倍多，环带里放不下。 */
  if ((((int16_t)cur->dx * (int16_t)nxt->dx) +
       ((int16_t)cur->dy * (int16_t)nxt->dy)) != 0)
  {
    return 0U;
  }
  /* 地图方向的单位向量：内部行号向下 ⇒ map_dy = −dy */
  ux = (float)cur->dx;
  uy = -(float)cur->dy;
  wx = (float)nxt->dx;
  wy = -(float)nxt->dy;

  leg_cur_mm = (float)cur->length * (float)GRID_NAV_CELL_MM;
  leg_next_mm = (float)nxt->length * (float)GRID_NAV_CELL_MM;

  scale = Nav_Params()->speed_scale * Nav_Params()->arc_spd_ratio;
  v_arc = scale * v_max;
  if (v_arc < 1.0f)
  {
    return 0U;
  }
  r_req = (2.0f * v_arc * (Nav_Params()->arc90_ms / 1000.0f)) / 3.14159265f;

  /* 角点 = 本段起点（当前 mm 位姿）+ 本段名义位移 */
  GridNav_GetPoseMm(&pose_mm_x, &pose_mm_y);
  pose_x = (float)pose_mm_x;
  pose_y = (float)pose_mm_y;
  corner_x = pose_x + (ux * leg_cur_mm);
  corner_y = pose_y + (uy * leg_cur_mm);

  /* 半径吃掉的直线长度不能超过两段本身（各留 ZDT_NAV_ARC_LEG_MARGIN_MM 余量） */
  r = r_req;
  {
    float limit = ((leg_cur_mm < leg_next_mm) ? leg_cur_mm : leg_next_mm) -
                  ZDT_NAV_ARC_LEG_MARGIN_MM;

    if (limit < r)
    {
      r = (limit > 0.0f) ? limit : 0.0f;
    }
  }
  while ((r >= ZDT_NAV_ARC_MIN_RADIUS_MM) &&
         (ZDT_Nav_ArcGeometryOk(corner_x, corner_y, ux, uy, wx, wy, r,
                                &delta_yaw) == 0U))
  {
    r -= ZDT_NAV_ARC_RADIUS_STEP_MM;
  }
  if (r < ZDT_NAV_ARC_MIN_RADIUS_MM)
  {
    (void)snprintf(line, sizeof(line),
                   "[NAV] arc refused: corner (%.0f,%.0f) wants R=%.0fmm, geometry"
                   " allows less than %.0fmm -> in-place turn as before\r\n",
                   (double)corner_x, (double)corner_y, (double)r_req,
                   (double)ZDT_NAV_ARC_MIN_RADIUS_MM);
    ZDT_Console_Print(line);
    return 0U;
  }

  /* 轮速：麦轮外轮 = v × (1 + 自转力臂/R)，归一化后若 >1 底盘会整体缩放
     （半径不变、车变慢）—— 所以这里要按缩放后的速度算驱动时长。 */
  mix = scale * (1.0f + ((float)CHASSIS_SPIN_ARM_MM / r));
  if (mix < 1.0f)
  {
    mix = 1.0f;
  }
  v_used = v_arc / mix;
  rpm = (v_used * (1.0f + ((float)CHASSIS_SPIN_ARM_MM / r))) /
        ZDT_NAV_MM_S_PER_RPM;
  ms = ZDT_Nav_ArcPlanMs(1.57079633f * r, v_used,
                         ZDT_NAV_ACCEL_MM_S2 / mix, ZDT_NAV_DECEL_MM_S2 / mix);

  zdt_nav_arc_radius_mm = r;
  zdt_nav_arc_v_mm_s = v_used;
  zdt_nav_arc_delta_yaw = delta_yaw;
  zdt_nav_arc_ux = ux;
  zdt_nav_arc_uy = uy;
  zdt_nav_arc_planned_ms = ms;
  zdt_nav_arc_planned_rpm = rpm;
  zdt_nav_arc_armed = 1U;
  /* 本段少走 R（圆弧会替它走掉） */
  zdt_nav_segment_target_rev -= r / (float)ZDT_MILEAGE_MM_PER_REV;

  (void)snprintf(line, sizeof(line),
                 "[NAV] arc armed: corner (%.0f,%.0f) R=%.0fmm %+.0fdeg v=%.0fmm/s"
                 " t=%.0fms outer=%.0fRPM (this leg -%.0fmm)%s\r\n",
                 (double)corner_x, (double)corner_y, (double)r, (double)delta_yaw,
                 (double)v_used, (double)ms, (double)rpm, (double)r,
                 (mix > (scale + 0.001f)) ? " [wheel-cap slowed]" : "");
  ZDT_Console_Print(line);
  return 1U;
}

/* 进入圆弧：发一次"边走边转"的速度指令（vx 沿车头 + omega），
 * 之后靠 ZDT_Nav_ArcHold() 按**线性推进的期望航向**纠偏；后段目标同时减 R。 */
static void ZDT_Nav_BeginArc(void)
{
  float v_max = (float)CHASSIS_MAX_WHEEL_RPM * ZDT_NAV_MM_S_PER_RPM;
  float omega_max = v_max / (float)CHASSIS_SPIN_ARM_MM;
  float v_norm = zdt_nav_arc_v_mm_s / v_max;
  float omega_norm = ((zdt_nav_arc_v_mm_s / zdt_nav_arc_radius_mm) / omega_max) *
                     ((zdt_nav_arc_delta_yaw >= 0.0f) ? 1.0f : -1.0f);
  char line[160];

  zdt_nav_arc_start_heading = ZDT_Nav_GetHeading();
  zdt_nav_arc_tick = HAL_GetTick();
  zdt_nav_arc_deadline = HAL_GetTick() +
      (uint32_t)zdt_nav_arc_planned_ms + ZDT_NAV_ARC_TIMEOUT_MARGIN_MS;
  zdt_nav_arc_by_heading = 0U;
  zdt_nav_arc_running = 1U;
  zdt_nav_drive_lateral = 0;
  zdt_nav_drive_sign = 1;
  zdt_nav_drive_scale = v_norm;
  zdt_nav_hold_peak_error = 0.0f;
  zdt_nav_hold_tick = HAL_GetTick() + ZDT_NAV_HOLD_PERIOD_MS;
  zdt_nav_rpm_report_tick = 0U;
  zdt_nav_motion_deadline = HAL_GetTick() + (uint32_t)zdt_nav_arc_planned_ms;
  /* 圆弧替后段走掉了开头 R，所以后段目标也要减 R */
  zdt_nav_segment_target_rev =
      ((float)GridNav_GetSegmentDistanceMm(zdt_nav_segment_index) - zdt_nav_arc_radius_mm) /
      (float)ZDT_MILEAGE_MM_PER_REV;
  zdt_nav_align_done = 1U;      /* 圆弧结束时车头已对准后段，不要再原地转 */
  zdt_nav_correction_count = 0U;
  zdt_nav_move_sign = 1;
  zdt_nav_settle_announced = 0U;

  if (ChassisMotor_SetVelocity(v_norm, 0.0f, omega_norm) != HAL_OK)
  {
    zdt_nav_arc_running = 0U;
    ZDT_Nav_Fail("arc command rejected");
    return;
  }

  (void)snprintf(line, sizeof(line),
                 "[NAV] ARC start: R=%.0fmm %+.0fdeg vx=%.3f omega=%+.3f t=%.0fms"
                 " outer=%.0fRPM\r\n",
                 (double)zdt_nav_arc_radius_mm, (double)zdt_nav_arc_delta_yaw,
                 (double)v_norm, (double)omega_norm,
                 (double)zdt_nav_arc_planned_ms,
                 (double)zdt_nav_arc_planned_rpm);
  ZDT_Console_Print(line);
  zdt_nav_state = ZDT_NAV_ARC;
}

/* 圆弧中的航向纠偏：期望航向按时间从"起始航向"线性推到"起始 ±90°"。
 * 用 time/planned 当进度（这段是定时驱动，没有格位姿可跟）。 */
static void ZDT_Nav_ArcHold(void)
{
  float progress;
  float target;

  if (zdt_nav_arc_planned_ms < 1.0f)
  {
    return;
  }
  progress = (float)(HAL_GetTick() - zdt_nav_arc_tick) / zdt_nav_arc_planned_ms;
  if (progress > 1.0f)
  {
    progress = 1.0f;
  }
  target = zdt_nav_arc_start_heading + (zdt_nav_arc_delta_yaw * progress);
  ZDT_Nav_HeadingHoldTo(target);
}

static void ZDT_Nav_RefreshCostModel(void)
{
  float scale = Nav_Params()->speed_scale;
  float cell_ms = (float)ZDT_Nav_PlanDriveMs(
      (float)GRID_NAV_CELL_MM / ZDT_MILEAGE_MM_PER_REV, scale);
  uint16_t t90;
  uint16_t t180;

  if (cell_ms < 1.0f)
  {
    cell_ms = 1.0f;
  }
  t90 = (uint16_t)(0.5f + ((float)GRID_NAV_MOVE_COST *
                           ((float)Nav_Params()->turn90_ms / cell_ms)));
  t180 = (uint16_t)(0.5f + ((float)GRID_NAV_MOVE_COST *
                            ((float)Nav_Params()->turn180_ms / cell_ms)));
  GridNav_SetCostModel(t90, t180);
}

static uint8_t ZDT_Blend_Clear(uint16_t x, uint16_t y, int16_t yaw)
{
  return GridNav_RotatedBodyFreeMarginMm(x,y,yaw,ZDT_BLEND_MARGIN_MM);
}

static uint8_t ZDT_Blend_TryStart(void)
{
  static const char *const stations[]={"raw","rough","temp","qr"};
  const grid_nav_segment_t *seg=&zdt_nav_segments[zdt_nav_segment_index];
  uint16_t x,y,wx,wy;
  uint8_t near=0U, i;
  float span, target, yaw=ZDT_Nav_GetHeading();
  char line[144];
  if (!ZDT_NAV_BLEND_ENABLE || !zdt_nav_endturn_enabled || zdt_nav_blend_used ||
      zdt_nav_segment_index!=0U || zdt_nav_align_done ||
      Nav_Params()->position_mode || Nav_Params()->arc90_ms>0.0f ||
      !MatchControl_IsActive(&zdt_match)) { return 0U; }
  target=ZDT_Nav_SegmentHeadingDeg(seg);
  if (fabsf(NavBlend_AngleError(target,yaw))<15.0f) { return 0U; }
  GridNav_GetPoseMm(&x,&y);
  for(i=0U;i<4U;++i) {
    if (GridNav_FindWaypoint(stations[i],&wx,&wy) &&
        abs((int)x-(int)wx)<=50 && abs((int)y-(int)wy)<=50) { near=1U; }
  }
  if (!near) { return 0U; }
  if (zdt_nav_start_mask!=0x0fU || !HWT101_IsZeroDriftReady() || !isfinite(yaw) ||
      !isfinite(Nav_Params()->counts_per_rev) || Nav_Params()->counts_per_rev<=0.0f) {
    ZDT_Nav_Fail("blend needs all wheel positions and calibrated IMU"); return 1U;
  }
  span=fminf(300.0f,zdt_nav_segment_target_rev*ZDT_MILEAGE_MM_PER_REV-100.0f);
  for(;span>=75.0f;span-=25.0f) {
    if (NavBlend_Init(&zdt_nav_blend,(float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,
                      (float)seg->dx,(float)-seg->dy,span,yaw,target)) {
      if(NavBlend_Clear(&zdt_nav_blend,(float)x,(float)y,ZDT_Blend_Clear)) { break; }
      /* Back away from the work table first, then allow any yaw at each pose. */
      zdt_nav_blend.turn_start_mm=40.0f;
      if(NavBlend_Clear(&zdt_nav_blend,(float)x,(float)y,ZDT_Blend_Clear)) { break; }
    }
  }
  zdt_nav_blend_used=1U; /* A rejected geometry stays rejected on realign/requery. */
  if(span<75.0f) {
    ZDT_Console_Print("[NAV] endturn skipped: no clear station-departure window; turn in place.\r\n");
    return 0U;
  }
  memcpy(zdt_nav_blend_position,zdt_nav_start_position,sizeof(zdt_nav_blend_position));
  zdt_nav_blend_x=(float)x; zdt_nav_blend_y=(float)y; zdt_nav_blend_yaw=yaw;
  zdt_nav_blend_sample_ready=0U;
  (void)snprintf(line,sizeof(line),
      "[NAV] BLEND field %.0fmm yaw %+.0f -> %+.0f; retreat %.0fmm; spd=%.3f turncap=%.0fRPM\r\n",
      (double)span,(double)yaw,(double)(yaw+zdt_nav_blend.sweep_deg),
      (double)zdt_nav_blend.turn_start_mm,(double)Nav_Params()->speed_scale,
      (double)CHASSIS_TURN_FAST_WHEEL_RPM);
  ZDT_Console_Print(line); /* Print while stopped, before starting the control clock. */
  zdt_nav_blend_sample_tick=HAL_GetTick();
  zdt_nav_blend_control_tick=HAL_GetTick();
  zdt_nav_blend_query_tick=HAL_GetTick()+ZDT_BLEND_QUERY_MS;
  zdt_nav_blend_deadline=HAL_GetTick()+15000U;
  zdt_nav_blend_imu_count=HWT101_GetSampleCount(); zdt_nav_blend_imu_tick=HAL_GetTick();
  zdt_nav_blend_progress_tick=HAL_GetTick(); zdt_nav_blend_progress_mark=0.0f;
  zdt_nav_blend_error_mark=fabsf(zdt_nav_blend.sweep_deg);
  zdt_nav_blend_cross_mark=0.0f;
  zdt_nav_state=ZDT_NAV_BLEND;
  return 1U;
}

static void ZDT_Blend_Fail(const char *reason)
{
  char line[144];
  (void)snprintf(line,sizeof(line),
      "[NAV] BLEND diag: progress=%.1f/%.1fmm cross=%+.1fmm yaw=%+.1f err=%+.1fdeg\r\n",
      (double)zdt_nav_blend.progress_mm,(double)zdt_nav_blend.span_mm,
      (double)zdt_nav_blend.cross_mm,(double)ZDT_Nav_GetHeading(),
      (double)NavBlend_AngleError(zdt_nav_blend.start_yaw+zdt_nav_blend.sweep_deg,ZDT_Nav_GetHeading()));
  ZDT_Nav_Fail(reason); /* Immediate CAN stop precedes all diagnostic UART writes. */
  ZDT_Console_Print(line);
}

static void ZDT_Blend_OnQuery(uint8_t mask)
{
  float wheel_mm[4], yaw=ZDT_Nav_GetHeading();
  uint8_t i;
  if(mask!=0x0fU || !isfinite(yaw)) { ZDT_Blend_Fail("blend lost wheel/IMU feedback"); return; }
  for(i=0U;i<4U;++i) {
    wheel_mm[i]=(float)((double)((int64_t)zdt_mileage_query_position[i]-zdt_nav_blend_position[i])*
                         zdt_mileage_forward_signs[i]*ZDT_MILEAGE_MM_PER_REV/Nav_Params()->counts_per_rev);
    if(!isfinite(wheel_mm[i]) || fabsf(wheel_mm[i])>200.0f) {
      ZDT_Blend_Fail("blend encoder jump"); return;
    }
  }
  NavBlend_Observe(&zdt_nav_blend,wheel_mm,
      zdt_nav_blend_yaw+0.5f*NavBlend_AngleError(yaw,zdt_nav_blend_yaw),
      (HAL_GetTick()-zdt_nav_blend_sample_tick)*0.001f);
  memcpy(zdt_nav_blend_position,zdt_mileage_query_position,sizeof(zdt_nav_blend_position));
  zdt_nav_blend_yaw=yaw; zdt_nav_blend_sample_tick=HAL_GetTick();
  zdt_nav_blend_sample_ready=1U;
  if(!isfinite(zdt_nav_blend.progress_mm) || !isfinite(zdt_nav_blend.cross_mm)) {
    ZDT_Blend_Fail("blend invalid odometry");
  } else if(zdt_nav_blend.progress_mm < -10.0f) {
    ZDT_Blend_Fail("blend reverse progress");
  } else if(zdt_nav_blend.progress_mm > zdt_nav_blend.span_mm+15.0f) {
    ZDT_Blend_Fail("blend distance overshoot");
  } else if(fabsf(zdt_nav_blend.cross_mm)>30.0f) {
    ZDT_Blend_Fail("blend cross-track >30mm");
  }
}

static void ZDT_Blend_Service(void)
{
  uint32_t now=HAL_GetTick(), samples=HWT101_GetSampleCount();
  chassis_telemetry_t t;
  float f,r,o,x,y,map_yaw;
  float error;
  if(samples!=zdt_nav_blend_imu_count) {
    zdt_nav_blend_imu_count=samples; zdt_nav_blend_imu_tick=now;
  }
  ChassisMotor_GetTelemetry(&t);
  if(!isfinite(t.heading_deg) || !isfinite(t.gyro_dps) || !HWT101_IsZeroDriftReady()) {
    ZDT_Blend_Fail("blend IMU invalid/unready"); return;
  }
  if(now-zdt_nav_blend_imu_tick>150U) {
    ZDT_Blend_Fail("blend IMU samples stale >150ms"); return;
  }
  if(now-zdt_nav_blend_sample_tick>400U) {
    ZDT_Blend_Fail("blend wheel feedback stale >400ms"); return;
  }
  if((int32_t)(now-zdt_nav_blend_deadline)>=0) {
    ZDT_Blend_Fail("blend 15s timeout"); return;
  }
  if((int32_t)(now-zdt_nav_blend_query_tick)>=0 && !zdt_mileage_query_active) {
    zdt_nav_blend_query_tick=now+ZDT_BLEND_QUERY_MS;
    ZDT_Mileage_StartQuery(0U);
  }
  error=NavBlend_AngleError(zdt_nav_blend.start_yaw+zdt_nav_blend.sweep_deg,t.heading_deg);
  if(zdt_nav_state==ZDT_NAV_BLEND) {
    if(zdt_nav_blend.progress_mm>zdt_nav_blend_progress_mark+3.0f ||
        fabsf(error)<zdt_nav_blend_error_mark-0.5f ||
        fabsf(zdt_nav_blend.cross_mm)<zdt_nav_blend_cross_mark-0.5f) {
      zdt_nav_blend_progress_tick=now; zdt_nav_blend_progress_mark=zdt_nav_blend.progress_mm;
      zdt_nav_blend_error_mark=fabsf(error);
      zdt_nav_blend_cross_mark=fabsf(zdt_nav_blend.cross_mm);
    } else if(now-zdt_nav_blend_progress_tick>2000U) {
      ZDT_Blend_Fail("blend no position/yaw progress for 2s"); return;
    }
  }
  if(zdt_nav_state==ZDT_NAV_BLEND_STOP) {
    if(zdt_nav_blend_sample_ready) {
      int16_t rpm[4]; uint8_t moving,i;
      zdt_nav_blend_sample_ready=0U;
      if(!zdt_nav_settle_valid) {
        memcpy(zdt_nav_settle_sample,zdt_nav_blend_position,sizeof(zdt_nav_settle_sample));
        zdt_nav_settle_valid=1U; return;
      }
      moving=ZDT_Nav_PositionsMoving(zdt_nav_settle_sample,zdt_nav_blend_position);
      memcpy(zdt_nav_settle_sample,zdt_nav_blend_position,sizeof(zdt_nav_settle_sample));
      ChassisMotor_GetWheelRpm(rpm);
      for(i=0U;i<4U;++i) { if(rpm[i]!=0) { moving=1U; } }
      if(!moving) {
        char line[112];
        if(fabsf(error)>3.0f || fabsf(zdt_nav_blend.cross_mm)>10.0f ||
            zdt_nav_blend.progress_mm<fmaxf(20.0f,zdt_nav_blend.turn_start_mm) ||
            zdt_nav_blend.progress_mm>zdt_nav_blend.span_mm+15.0f) {
          ZDT_Blend_Fail("blend stopped outside distance/yaw tolerance"); return;
        }
        zdt_nav_segment_target_rev-=zdt_nav_blend.progress_mm/ZDT_MILEAGE_MM_PER_REV;
        memcpy(zdt_nav_start_position,zdt_nav_blend_position,sizeof(zdt_nav_start_position));
        zdt_nav_start_mask=0x0fU; zdt_nav_start_ready=1U; zdt_nav_align_done=1U;
        zdt_nav_settle_valid=0U; zdt_nav_settle_announced=0U;
        zdt_nav_state=ZDT_NAV_WAIT_START_QUERY;
        (void)snprintf(line,sizeof(line),"[NAV] BLEND done %.0fmm cross=%+.1fmm yawerr=%+.1f; remaining %.0fmm\r\n",
            (double)zdt_nav_blend.progress_mm,(double)zdt_nav_blend.cross_mm,(double)error,
            (double)(zdt_nav_segment_target_rev*ZDT_MILEAGE_MM_PER_REV));
        ZDT_Console_Print(line);
        if(zdt_nav_blend_deferred_input) {
          zdt_nav_blend_deferred_input=0U;
          ZDT_Console_Print("[NAV] input ignored during BLEND; wait for WAIT_OK.\r\n");
        }
      }
    }
    if(zdt_nav_state==ZDT_NAV_BLEND_STOP && (int32_t)(now-zdt_nav_settle_deadline)>=0) {
      ZDT_Blend_Fail("blend wheels did not settle");
    }
    return;
  }
  if(NavBlend_Ready(&zdt_nav_blend,t.heading_deg,t.gyro_dps)) {
    if(ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP)!=HAL_OK) {
      ZDT_Blend_Fail("blend stop rejected"); return;
    }
    zdt_nav_state=ZDT_NAV_BLEND_STOP; zdt_nav_settle_valid=0U;
    zdt_nav_blend_sample_ready=0U; zdt_nav_settle_deadline=now+ZDT_NAV_SETTLE_TIMEOUT_MS;
    return;
  }
  if((int32_t)(now-zdt_nav_blend_control_tick)<0) { return; }
  zdt_nav_blend_control_tick=now+ZDT_BLEND_PERIOD_MS;
  x=zdt_nav_blend_x+zdt_nav_blend.ux*zdt_nav_blend.progress_mm-zdt_nav_blend.uy*zdt_nav_blend.cross_mm;
  y=zdt_nav_blend_y+zdt_nav_blend.uy*zdt_nav_blend.progress_mm+zdt_nav_blend.ux*zdt_nav_blend.cross_mm;
  map_yaw=NavBlend_AngleError(atan2f(zdt_nav_blend.initial_y,zdt_nav_blend.initial_x)*57.29578f+t.heading_deg,0.0f);
  if(x<0.0f || y<0.0f || x>GRID_NAV_FIELD_MM || y>GRID_NAV_FIELD_MM ||
      !ZDT_Blend_Clear((uint16_t)lroundf(x),(uint16_t)lroundf(y),(int16_t)lroundf(map_yaw))) {
    ZDT_Blend_Fail("blend actual pose outside clearance"); return;
  }
  {
    nav_blend_control_t control={0};
    control.wheel_mm_s=CHASSIS_MAX_WHEEL_RPM*ZDT_NAV_MM_S_PER_RPM;
    control.max_mm_s=Nav_Params()->speed_scale*control.wheel_mm_s;
    if(now-zdt_nav_blend_sample_tick>60U) { control.max_mm_s=0.0f; }
    /* Reserve wheel-ramp authority for changing body vx/vy plus rotation. */
    control.decel_mm_s2=0.4f*ZDT_NAV_DECEL_MM_S2; control.reaction_s=0.04f;
    control.max_omega=CHASSIS_TURN_FAST_WHEEL_RPM/CHASSIS_MAX_WHEEL_RPM;
    control.min_push=CHASSIS_TURN_FAST_MIN_PUSH; control.dt_s=ZDT_BLEND_PERIOD_MS*0.001f;
    control.response_s=0.0f; control.cross_kp=6.0f; control.cross_kd=0.5f;
    control.cross_cap_mm_s=0.5f*control.max_mm_s;
    control.along_kv=1.0f;
    ChassisMotor_GetTurnGains(&control.kp,&control.ki,&control.kd);
    NavBlend_Command(&zdt_nav_blend,t.heading_deg,t.gyro_dps,&control,&f,&r,&o);
  }
  {
    float vx,vy;
    NavFrame_BodyToMap(zdt_nav_blend.initial_x,zdt_nav_blend.initial_y,t.heading_deg,f,r,&vx,&vy);
    if(ChassisMotor_SetFieldVelocity(zdt_nav_blend.initial_x,zdt_nav_blend.initial_y,vx,vy,o)!=HAL_OK) {
      ZDT_Blend_Fail("blend field velocity rejected");
    }
  }
}

static uint8_t ZDT_Outer_Clear(float x,float y,float yaw)
{
  static const int8_t ox[5]={0,2,-2,0,0},oy[5]={0,0,0,2,-2};uint8_t i;
  if(!isfinite(x) || !isfinite(y) || !isfinite(yaw))return 0U;
  for(i=0U;i<5U;i++) {
    float px=x+ox[i],py=y+oy[i];
    if(px<0.0f || py<0.0f || px>GRID_NAV_FIELD_MM || py>GRID_NAV_FIELD_MM ||
       !GridNav_RotatedBodyFreeMm((uint16_t)lroundf(px),(uint16_t)lroundf(py),
          (int16_t)lroundf(NavBlend_AngleError(yaw,0.0f))))return 0U;
  }
  return 1U;
}
static uint8_t ZDT_Outer_Skip(const char *reason,float value)
{
  char line[128];
  (void)snprintf(line,sizeof(line),"[OUTER ARC] SKIP %s value=%.1f; stopped route retained\r\n",reason,(double)value);
  ZDT_Console_Print(line);return 0U;
}
static uint8_t ZDT_Outer_TryStart(void)
{
  const grid_nav_segment_t *cur=&zdt_nav_segments[zdt_nav_segment_index];
  float cx,cy,ex,ey,ux=cur->dx,uy=-cur->dy,length,yaw=ZDT_Nav_GetHeading();
  uint16_t x,y,gx,gy;uint8_t cross_action=0U;char line[192];
  if(!zdt_outer_enabled || !zdt_nav_odometry_enabled || zdt_nav_start_mask!=15U ||
     zdt_nav_move_sign!=1 || zdt_odo_owner ||
     !isfinite(yaw) || ZDT_Nav_MatchExtraMm()!=0.0f ||
     zdt_match_io!=MATCH_IO_ZONE2 || zdt_zone2_phase!=Z2_NAV || !zdt_zone2.running)return 0U;
  GridNav_GetPoseMm(&x,&y);length=(float)GridNav_GetSegmentDistanceMm(zdt_nav_segment_index);
  cx=x+ux*length;cy=y+uy*length;
  if(zdt_nav_segment_index+1U<zdt_nav_segment_count) {
    const grid_nav_segment_t *next=&zdt_nav_segments[zdt_nav_segment_index+1U];
    if(cur->dx*next->dx+cur->dy*next->dy!=0)return 0U;
    ex=cx+next->dx*GridNav_GetSegmentDistanceMm(zdt_nav_segment_index+1U);
    ey=cy-next->dy*GridNav_GetSegmentDistanceMm(zdt_nav_segment_index+1U);
    if(zdt_nav_segment_index+2U==zdt_nav_segment_count){GridNav_GetTargetMm(&gx,&gy);ex=gx;ey=gy;}
    /* A* stores cell-sized legs, while start/goal retain exact millimetres.
     * Their combined cell-centre bias can be one whole50mm cell. Rebuild
     * the perpendicular corner at the exact exit lane; validate THAT path. */
    if(fabsf(ux)>0.5f) {
      float delta=ex-cx;
      if(fabsf(delta)>GRID_NAV_CELL_MM)return ZDT_Outer_Skip("exit lane mismatch >one cell",delta);
      cx=ex;
    } else {
      float delta=ey-cy;
      if(fabsf(delta)>GRID_NAV_CELL_MM)return ZDT_Outer_Skip("exit lane mismatch >one cell",delta);
      cy=ey;
    }
  } else {
    const zone2_action_t *next,*turn=NULL;uint8_t next_index;
    if(zdt_zone2.index+1U>=zdt_zone2.count || zdt_zone2_action.extra_mm)return 0U;
    next_index=zdt_zone2.index+1U;next=&zdt_zone2.actions[next_index];
    /* Transport can be stored as NAV -> explicit facing -> NAV. Only
     * consume a facing that agrees with the outgoing tangent; station
     * facing/180deg turns remain separate. Advance all consumed actions
     * only after actual end-position/heading and stopped wheels. */
    if(next->kind==ZONE2_TURN) {
      turn=next;
      if(turn->extra_mm || turn->final_point || next_index+1U>=zdt_zone2.count)return 0U;
      next=&zdt_zone2.actions[++next_index];
    }
    if(next->kind!=ZONE2_NAV || next->extra_mm)return 0U;
    cx=zdt_zone2_action.x;cy=zdt_zone2_action.y;ex=next->x;ey=next->y;
    /* The next RAW action has a100mm lane join along incoming travel.
     * Extend the incoming line to that perpendicular lane before filleting. */
    if(fabsf(ux)>0.5f){if(fabsf(cx-ex)>200.0f)return 0U;cx=ex;}
    else {if(fabsf(cy-ey)>200.0f)return 0U;cy=ey;}
    if(turn && fabsf(NavBlend_AngleError((float)turn->yaw,
         atan2f(ey-cy,ex-cx)*57.29578f))>0.01f)return ZDT_Outer_Skip("facing differs from exit tangent",turn->yaw);
    cross_action=next_index-zdt_zone2.index;
  }
  if(!NavCorner_Init(&zdt_outer,zdt_nav_nose_x,-zdt_nav_nose_y,x,y,cx,cy,ex,ey,ux,uy,zdt_outer_radius,yaw)) {
    if(length>=zdt_outer_radius+100.0f && (cx<=500.0f || cx>=1900.0f || cy<=500.0f || cy>=1900.0f))
      return ZDT_Outer_Skip("geometry/leg/heading not eligible",yaw);
    return 0U;
  }
  /* The continuous corner bypasses the ordinary straight owner. Expand
   * its two tangent legs once, retaining radius and the90deg arc itself.
   * Rebuild and clear the resulting REAL path rather than falsifying odometry. */
  {
    float extra_in=Nav_Params_StraightDistance(zdt_outer.in_mm)-zdt_outer.in_mm;
    float extra_out=Nav_Params_StraightDistance(zdt_outer.out_mm)-zdt_outer.out_mm;
    cx+=ux*extra_in;cy+=uy*extra_in;
    ex+=ux*extra_in+zdt_outer.wx*extra_out;
    ey+=uy*extra_in+zdt_outer.wy*extra_out;
    if(!NavCorner_Init(&zdt_outer,zdt_nav_nose_x,-zdt_nav_nose_y,x,y,cx,cy,ex,ey,ux,uy,zdt_outer_radius,yaw))
      return ZDT_Outer_Skip("scaled tangent geometry rejected",NAV_STRAIGHT_TARGET_SCALE);
  }
  if(!NavCorner_Clear(&zdt_outer,ZDT_Outer_Clear)) {
    zdt_outer_fallback_count++;
    ZDT_Console_Print("[OUTER ARC] footprint clearance rejected; original stopped corner retained\r\n");return 0U;
  }
  if(!ZDT_Zone2_WheelsStopped() || ChassisMotor_HasMotionDemand())return ZDT_Outer_Skip("start baseline still moving",0.0f);
  /* drive_lateral describes the LAST completed segment. It can still be
   * set after a short post-align strafe. The fresh stopped baseline above
   * is authoritative for this NEW segment; do not silently veto its arc. */
  zdt_nav_drive_lateral=0;
  zdt_outer_cross_action=cross_action;zdt_outer_started_count++;zdt_outer_stage=0U;
  memcpy(zdt_outer_position,zdt_nav_start_position,sizeof(zdt_outer_position));
  (void)snprintf(line,sizeof(line),"[OUTER ARC] START corner=(%.0f,%.0f) end=(%.0f,%.0f) R=%.0fmm curve<=%.0fmm/s span=%.0fmm %s; continuous straight/arc/straight\r\n",
    (double)cx,(double)cy,(double)ex,(double)ey,(double)zdt_outer_radius,(double)zdt_outer_speed,
    (double)zdt_outer.total_mm,cross_action==2U?"NAV/TURN/NAV":cross_action?"two NAV actions":"two segments");ZDT_Console_Print(line);
  zdt_outer_sample_tick=zdt_outer_control_tick=zdt_outer_query_tick=zdt_outer_progress_tick=HAL_GetTick();
  zdt_outer_imu_count=HWT101_GetSampleCount();zdt_outer_imu_tick=HAL_GetTick();
  zdt_outer_deadline=HAL_GetTick()+30000U;
  zdt_outer_progress_mark=0.0f;zdt_outer_sample_ready=0U;zdt_outer_settle_tick=0U;
  zdt_nav_wait_start_query=zdt_nav_wait_end_query=0U;zdt_nav_state=ZDT_NAV_OUTER_CORNER;
  ChassisMotor_ResetIdleRamp();return 1U;
}
static void ZDT_Outer_OnQuery(uint8_t mask)
{
  uint32_t now=HAL_GetTick();float w[4],yaw=ZDT_Nav_GetHeading();uint8_t i;
  uint32_t received=zdt_mileage_query_sample_tick;
  if(zdt_nav_state!=ZDT_NAV_OUTER_CORNER)return;
  if(mask!=15U || !Nav_Params()->counts_per_rev || !isfinite(yaw)) {ZDT_Nav_Fail("outer arc needs four fresh wheels/IMU");return;}
  if((int32_t)(now-received)>150){ZDT_Nav_Fail("outer arc received wheel sample stale");return;}
  if((int32_t)(received-zdt_outer_sample_tick)<=0)return;
  for(i=0U;i<4U;i++) {
    w[i]=(float)((double)((int64_t)zdt_mileage_query_position[i]-zdt_outer_position[i])*
       zdt_mileage_forward_signs[i]*ZDT_MILEAGE_MM_PER_REV/Nav_Params()->counts_per_rev);
    if(!isfinite(w[i]) || fabsf(w[i])>200.0f){ZDT_Nav_Fail("outer arc encoder jump");return;}
  }
  NavCorner_Observe(&zdt_outer,w,yaw,(received-zdt_outer_sample_tick)*0.001f);
  memcpy(zdt_outer_position,zdt_mileage_query_position,sizeof(zdt_outer_position));
  zdt_outer_sample_tick=received;zdt_outer_sample_ready=1U;
  if(!isfinite(zdt_outer.x) || !isfinite(zdt_outer.y) || zdt_outer.cross_mm>30.0f ||
     !ZDT_Outer_Clear(zdt_outer.x,zdt_outer.y,
       atan2f((float)-zdt_nav_nose_y,(float)zdt_nav_nose_x)*57.29578f+yaw)) {
    ZDT_Nav_Fail("outer arc actual path/footprint outside margin");return;
  }
  GridNav_SetPoseMm((uint16_t)lroundf(zdt_outer.x),(uint16_t)lroundf(zdt_outer.y));
}
static void ZDT_Outer_Service(void)
{
  uint32_t now=HAL_GetTick(),samples=HWT101_GetSampleCount();chassis_telemetry_t t;
  nav_corner_control_t c;float vx,vy,o,wheel_speed=CHASSIS_MAX_WHEEL_RPM*ZDT_NAV_MM_S_PER_RPM;
  uint8_t stage;char line[180];
  if(zdt_nav_state!=ZDT_NAV_OUTER_CORNER)return;
  ChassisMotor_GetTelemetry(&t);
  if(samples!=zdt_outer_imu_count){zdt_outer_imu_count=samples;zdt_outer_imu_tick=now;}
  if(!ChassisMotor_IsEnabled() || !HWT101_IsZeroDriftReady() || !isfinite(t.heading_deg) || !isfinite(t.gyro_dps) ||
     now-zdt_outer_imu_tick>150U || now-zdt_outer_sample_tick>400U) {
    ZDT_Nav_Fail("outer arc chassis/IMU/wheel feedback unavailable");return;
  }
  if((int32_t)(now-zdt_outer_deadline)>=0){ZDT_Nav_Fail("outer arc 30s timeout");return;}
  if(zdt_outer.progress>zdt_outer_progress_mark+3.0f) {
    zdt_outer_progress_mark=zdt_outer.progress;zdt_outer_progress_tick=now;
  } else if(now-zdt_outer_progress_tick>3000U && !zdt_outer_settle_tick) {
    ZDT_Nav_Fail("outer arc no encoder progress for3s");return;
  }
  if(!zdt_mileage_query_active && (int32_t)(now-zdt_outer_query_tick)>=0) {
    zdt_outer_query_tick=now+20U;ZDT_Mileage_StartQuery(0U);
  }
  if(zdt_outer_sample_ready && NavCorner_Ready(&zdt_outer,t.heading_deg,t.gyro_dps) && ZDT_Zone2_WheelsStopped()) {
    zdt_outer_sample_ready=0U;
    if(!zdt_outer_settle_tick)zdt_outer_settle_tick=now;
    else if(now-zdt_outer_settle_tick>=150U) {
      uint8_t cross_action=zdt_outer_cross_action;
      (void)ChassisMotor_Stop();zdt_outer_cross_action=0U;
      (void)snprintf(line,sizeof(line),"[OUTER ARC] DONE actual=(%.1f,%.1f) cross=%.1fmm path=%.1f/%.1fmm; no corner stop, final stop verified\r\n",
        (double)zdt_outer.x,(double)zdt_outer.y,(double)zdt_outer.cross_mm,(double)zdt_outer.progress,(double)zdt_outer.total_mm);ZDT_Console_Print(line);
      if(cross_action) {
        zdt_nav_state=ZDT_NAV_IDLE;zdt_nav_post_align=1U;
        zdt_zone2.index+=cross_action; /* Consume NAV[/TURN]/NAV only after actual arrival. */
        ZDT_Nav_NotifyArrived();
      } else {
        zdt_nav_segment_index+=2U;
        if(zdt_nav_segment_index>=zdt_nav_segment_count) {
          zdt_nav_state=ZDT_NAV_IDLE;zdt_nav_post_align=1U;ZDT_Nav_NotifyArrived();
        } else {
          zdt_nav_segment_target_rev=GridNav_GetSegmentDistanceMm(zdt_nav_segment_index)/ZDT_MILEAGE_MM_PER_REV;
          zdt_nav_align_done=0U;zdt_nav_wait_start_query=1U;zdt_nav_start_ready=0U;
          zdt_nav_state=ZDT_NAV_WAIT_START_QUERY;
          if(!zdt_mileage_query_active)ZDT_Mileage_StartQuery(0U);
        }
      }
      return;
    }
  } else if(zdt_outer_sample_ready) {zdt_outer_sample_ready=0U;zdt_outer_settle_tick=0U;}
  if(now-zdt_outer_control_tick<20U)return;
  zdt_outer_control_tick=now;
  stage=zdt_outer.progress<zdt_outer.in_mm?0U:zdt_outer.progress<zdt_outer.in_mm+zdt_outer.arc_mm?1U:2U;
  c.straight_mm_s=Nav_Params()->speed_scale*wheel_speed;c.corner_mm_s=fminf(zdt_outer_speed,c.straight_mm_s*NAV_CORNER_SPEED_RATIO);
  c.decel_mm_s2=0.4f*ZDT_NAV_DECEL_MM_S2;c.lateral_mm_s2=NAV_CORNER_LATERAL_MM_S2;c.max_yaw_dps=NAV_CORNER_MAX_YAW_DPS;
  if(now-zdt_outer_sample_tick>150U)c.straight_mm_s=c.corner_mm_s=0.0f;
  NavCorner_Command(&zdt_outer,&c,t.heading_deg,t.gyro_dps,&vx,&vy,&o);
  if(now-zdt_outer_sample_tick>150U)vx=vy=o=0.0f;
  if(ChassisMotor_SetFieldVelocity(zdt_nav_nose_x,-zdt_nav_nose_y,vx/wheel_speed,vy/wheel_speed,
       o*CHASSIS_SPIN_ARM_MM/wheel_speed)!=HAL_OK){ZDT_Nav_Fail("outer arc field velocity rejected");return;}
  /* Send control BEFORE concise phase diagnostics. No periodic long prints
   * during moving curves: low-baud consoles must not stall the control task. */
  if(stage!=zdt_outer_stage) {
    zdt_outer_stage=stage;
    (void)snprintf(line,sizeof(line),"[OUTER ARC] phase=%u cross=%.1f yaw=%.1f\r\n",
       stage,(double)zdt_outer.cross_mm,(double)t.heading_deg);ZDT_Console_Print(line);
  }
}
static uint8_t ZDT_Outer_Command(const char *command)
{
  float value;char extra,line[160];
  if(strncmp(command,"outerarc",8U) || (command[8] && command[8]!=' '))return 0U;
  if(!strcmp(command,"outerarc") || !strcmp(command,"outerarc status")) {
    (void)snprintf(line,sizeof(line),"[OUTER ARC] enabled=%u active=%u R=%.0fmm cap=%.1fmm/s ratio=%.2f lat<=%.1fmm/s2 yaw<=%.0fdeg/s starts=%lu clearance_fallbacks=%lu RAM only\r\n",
       zdt_outer_enabled,zdt_nav_state==ZDT_NAV_OUTER_CORNER,(double)zdt_outer_radius,(double)zdt_outer_speed,
       (double)NAV_CORNER_SPEED_RATIO,(double)NAV_CORNER_LATERAL_MM_S2,(double)NAV_CORNER_MAX_YAW_DPS,
       (unsigned long)zdt_outer_started_count,(unsigned long)zdt_outer_fallback_count);ZDT_Console_Print(line);return 1U;
  }
  if(ZDT_Align_DiagBusy()){ZDT_Console_Print("[OUTER ARC] configure with stopped idle chassis; status/STOP remain available\r\n");return 1U;}
  if(!strcmp(command,"outerarc on"))zdt_outer_enabled=1U;
  else if(!strcmp(command,"outerarc off"))zdt_outer_enabled=0U;
  else if(sscanf(command,"outerarc radius %f %c",&value,&extra)==1 && isfinite(value) && value>=80.0f && value<=200.0f)zdt_outer_radius=value;
  else if(sscanf(command,"outerarc speed %f %c",&value,&extra)==1 && isfinite(value) && value>=80.0f && value<=360.0f)zdt_outer_speed=value;
  else {ZDT_Console_Print("[OUTER ARC] on|off|status|radius80..200mm|speed80..360mm/s; old tarc unchanged\r\n");return 1U;}
  ZDT_Console_Print("[OUTER ARC] RAM trial updated; unsupported/blocked corners keep stopped turns\r\n");return 1U;
}
/* 开始执行 GridNav 最近一次保存的直线段路径。 */
static void ZDT_Nav_Start(void)
{
  uint16_t count = 0U;
  uint8_t recenter=zdt_nav_queued_recenter;
  zdt_nav_queued_recenter=0U;
  if(zdt_nav_odometry_enabled)Nav_Params()->position_mode=0U;

  if (ChassisMotor_IsEnabled() == 0U)
  {
    ZDT_Console_Print("[NAV] FAILED: motors disabled; send &EN first.\r\n");
    ZDT_Match_Complete(0U, "motors disabled");
    return;
  }

  if (GridNav_GetSegments(zdt_nav_segments, GRID_NAV_MAX_SEGMENTS,
                          &count) == 0U)
  {
    ZDT_Console_Print("[NAV] no executable path.\r\n");
    ZDT_Match_Complete(0U, "no executable path");
    return;
  }
  if (count == 0U)
  {
    ZDT_Console_Print("[NAV] ARRIVED (already at target).\r\n");
    ZDT_Odo_Cancel();
    zdt_nav_state=ZDT_NAV_IDLE;zdt_nav_wait_start_query=zdt_nav_wait_end_query=0U;
    ZDT_Nav_NotifyArrived();
    return;
  }

  ZDT_Nav_Cancel();
  zdt_nav_segment_count = count;
  zdt_nav_recenter_route=recenter;
  zdt_nav_segment_index = 0U;
  zdt_nav_correction_count = 0U;
  zdt_nav_move_sign = 1;
  zdt_nav_drive_lateral = 0;   /* 新任务：第一段按直走处理，别继承上一轮的横移标记 */
  zdt_nav_settle_valid = 0U;
  zdt_nav_settle_announced = 0U;
  zdt_nav_segment_target_rev =
      (float)GridNav_GetSegmentDistanceMm(0U) /
      (float)ZDT_MILEAGE_MM_PER_REV;
  zdt_nav_start_ready = 0U;
  zdt_nav_end_ready = 0U;
  zdt_nav_query_failed = 0U;
  zdt_nav_wait_start_query = 1U;
  zdt_nav_wait_end_query = 0U;
  zdt_nav_state = ZDT_NAV_WAIT_START_QUERY;
  ZDT_Console_Print("[NAV] EXEC start.\r\n");
  ZDT_Mileage_StartQuery(0U);
}

static void ZDT_Nav_OnQueryComplete(uint8_t mask)
{
  if(zdt_nav_state==ZDT_NAV_OUTER_CORNER){ZDT_Outer_OnQuery(mask);return;}
  if(zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) {
    ZDT_Blend_OnQuery(mask); return;
  }
  if (zdt_nav_wait_start_query != 0U)
  {
    memcpy(zdt_nav_start_position, zdt_mileage_query_position,
           sizeof(zdt_nav_start_position));
    zdt_nav_start_mask = mask;
    zdt_nav_wait_start_query = 0U;
    zdt_nav_start_ready = 1U;
  }
  else if (zdt_nav_wait_end_query != 0U)
  {
    memcpy(zdt_nav_end_position, zdt_mileage_query_position,
           sizeof(zdt_nav_end_position));
    zdt_nav_end_mask = mask;
    zdt_nav_wait_end_query = 0U;
    zdt_nav_end_ready = 1U;
  }
}

static void ZDT_Nav_OnQueryFailed(void)
{
  if(zdt_nav_state==ZDT_NAV_OUTER_CORNER){ZDT_Nav_Fail("outer arc position query failed");return;}
  if(zdt_nav_state==ZDT_NAV_BLEND || zdt_nav_state==ZDT_NAV_BLEND_STOP) {
    ZDT_Blend_Fail("blend position query failed"); return;
  }
  if ((zdt_nav_wait_start_query != 0U) || (zdt_nav_wait_end_query != 0U))
  {
    zdt_nav_query_failed = 1U;
    zdt_nav_wait_start_query = 0U;
    zdt_nav_wait_end_query = 0U;
  }
}

/* 直线段执行器：连续运行 -> 段末停车 -> 查询位置 -> 误差矫正。 */
static void ZDT_Nav_Service(void)
{
  const grid_nav_segment_t *segment;

  if(zdt_nav_state==ZDT_NAV_VOBS_STOP || zdt_nav_state==ZDT_NAV_VOBS_HOLD)return;

  if ((zdt_nav_state == ZDT_NAV_IDLE) ||
      (zdt_nav_segment_index >= zdt_nav_segment_count))
  {
    return;
  }
  segment = &zdt_nav_segments[zdt_nav_segment_index];

  switch (zdt_nav_state)
  {
    case ZDT_NAV_WAIT_START_QUERY:
      if (zdt_nav_query_failed != 0U)
      {
        ZDT_Nav_Fail("start odometry query");
      }
      else if (zdt_nav_start_ready != 0U)
      {
        zdt_nav_start_ready = 0U;

        if(!zdt_nav_odometry_enabled && ZDT_Blend_TryStart()) { break; }

        if(zdt_nav_recenter_route && (!zdt_vobs_nav_active || zdt_nav_segment_index==0U) &&
            !zdt_nav_align_done && Nav_Params()->position_mode==0U &&
            ZDT_Nav_MatchExtraMm()==0.0f &&
            zdt_nav_segment_target_rev*ZDT_MILEAGE_MM_PER_REV<=NAV_SHORT_MOVE_MAX_MM &&
            NavFrame_ReverseAligned((float)zdt_nav_nose_x,(float)-zdt_nav_nose_y,
                ZDT_Nav_GetHeading(),segment->dx,-segment->dy,ZDT_NAV_ALIGN_TOLERANCE_DEG)) {
          zdt_nav_lateral_hold_heading=ZDT_Nav_GetHeading();
          zdt_nav_align_done=1U;zdt_nav_move_sign=-1;
          ZDT_Console_Print("[NAV] recenter BACK: hold heading, no 180 turn.\r\n");
          ZDT_Nav_BeginStraightDrive(zdt_nav_segment_target_rev,-1,0.0f);break;
        }

        /* ★ 小幅横移优先：本段相对当前车头是"纯横向"且很短（≤3 格）→ **不转向**，
           直接横移过去（见 ZDT_Nav_BeginLateralDrive 的说明）。这样 A* 路径里那些
           1 格让位不再变成"转 90°→走 50 mm→转回 90°"。方向真正改变（转角/掉头、
           或跨度超过 3 格的横向段）时，strafe_sign 为 0，走下面的原逻辑原地转。 */
        {
          int8_t strafe_sign = 0;

          if ((zdt_nav_align_done == 0U) && (Nav_Params()->position_mode == 0U))
          {
            strafe_sign = ZDT_Nav_SegmentStrafeSign(segment);
          }
          if (strafe_sign != 0)
          {
            zdt_nav_align_done = 1U;
            zdt_nav_correction_count = 0U;
            zdt_nav_move_sign = 1;
            ZDT_Nav_BeginLateralDrive(zdt_nav_segment_target_rev, strafe_sign, 0.0f);
            break;
          }
        }

        /* ★ 90° 圆弧过弯（`&tarc`）：先看"本段末尾那个直角"能不能改成圆弧。
           必须在下面的原地对准**之前**做，而且只在 align_done==0 时做一次
           （否则同一段会被反复减掉 R）。武装成功只是把本段目标距离减 R；
           真正走圆弧是在本段走完、WAIT_END_QUERY 里进入 ZDT_NAV_ARC。 */
        if (!zdt_nav_odometry_enabled && (zdt_nav_align_done == 0U) && (zdt_nav_arc_armed == 0U))
        {
          (void)ZDT_Nav_TryArmArc();
        }

        /* 先原地转到本段方向，再直走（不横移）。转向后四轮位置变了，
           所以转完必须重采一次里程基准，这里用 zdt_nav_align_done 防止反复对向。
           位置模式（`posmode 1`）保留原有的"按段方向横移"行为，故不做对准。 */
        if ((zdt_nav_align_done == 0U) && (Nav_Params()->position_mode == 0U))
        {
          float target_heading = ZDT_Nav_SegmentHeadingDeg(segment);
          float heading_error = target_heading - ZDT_Nav_GetHeading();
          int16_t turn_angle;

          while (heading_error > 180.0f)
          {
            heading_error -= 360.0f;
          }
          while (heading_error < -180.0f)
          {
            heading_error += 360.0f;
          }

          if (fabsf(heading_error) > ZDT_NAV_ALIGN_TOLERANCE_DEG)
          {
            char line[112];

            turn_angle = (int16_t)lroundf(heading_error);
            if (turn_angle == 0)
            {
              turn_angle = (heading_error > 0.0f) ? 1 : -1;
            }
            /* 段前对准走**快速档**：允许 0.5° 收工、驻留 200 ms、上限 120 RPM。
               对准之后直走有航向保持（每 80 ms 纠一次）兜住残余偏差，
               所以没必要在转弯处抠到 0.05°（那是手动 &L90 的精确档要求）。 */
            if (ChassisMotor_TurnRelativeFast(turn_angle) != HAL_OK)
            {
              ZDT_Nav_Fail("align turn rejected");
              break;
            }
            (void)snprintf(line, sizeof(line),
                           "[NAV] align turn %+.1f deg (target %+.0f, fast)\r\n",
                           (double)heading_error, (double)target_heading);
            ZDT_Console_Print(line);
            zdt_nav_align_done = 1U;
            zdt_nav_state = ZDT_NAV_ALIGN_TURN;
            break;
          }
          zdt_nav_align_done = 1U;
        }

        zdt_nav_correction_count = 0U;
        zdt_nav_move_sign = 1;
        if(!zdt_vobs_nav_active && ZDT_Outer_TryStart())break;
        ZDT_Nav_StartMove(zdt_nav_segment_target_rev, 1,
                          ZDT_NAV_POSITION_SPEED_RPM, 0.0f);
      }
      break;

    case ZDT_NAV_ALIGN_TURN:
      /* 等到原地转完成 -> 重采里程基准 -> 回到 WAIT_START_QUERY 直走。 */
      if (ChassisMotor_IsTurnActive() == 0U)
      {
        zdt_nav_start_ready = 0U;
        zdt_nav_query_failed = 0U;
        zdt_nav_wait_start_query = 1U;
        zdt_nav_state = ZDT_NAV_WAIT_START_QUERY;
        ZDT_Mileage_StartQuery(0U);
      }
      break;

    case ZDT_NAV_DRIVE:
      if(zdt_nav_odometry_enabled) {
        uint8_t result=ZDT_Odo_Service(1U);
        if(result==2U){ZDT_Nav_Fail(zdt_odo.reason);return;}
        if(result==1U) {
          zdt_nav_end_ready=zdt_nav_query_failed=zdt_nav_settle_valid=0U;
          zdt_nav_wait_end_query=1U;zdt_nav_settle_deadline=HAL_GetTick()+ZDT_NAV_SETTLE_TIMEOUT_MS;
          zdt_nav_state=ZDT_NAV_WAIT_END_QUERY;ZDT_Mileage_StartQuery(0U);
        }
        break;
      }
      /* 速度模式：时间到就发停止（时间由 ZDT_Nav_PlanDriveMs 按梯形/三角形规划算出）。 */
      if ((zdt_nav_rpm_report_tick != 0U) &&
          ((int32_t)(HAL_GetTick() - zdt_nav_rpm_report_tick) >= 0))
      {
        int16_t rpm[ZDT_MILEAGE_MOTOR_COUNT];
        char rpm_line[128];

        zdt_nav_rpm_report_tick = 0U;
        ChassisMotor_GetWheelRpm(rpm);
        (void)snprintf(rpm_line, sizeof(rpm_line),
                       "[NAV] applied rpm FL%+d FR%+d RL%+d RR%+d\r\n",
                       (int)rpm[0], (int)rpm[1], (int)rpm[2], (int)rpm[3]);
        ZDT_Console_Print(rpm_line);
      }
      if ((int32_t)(HAL_GetTick() - zdt_nav_motion_deadline) >= 0)
      {
        (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
        zdt_nav_end_ready = 0U;
        zdt_nav_query_failed = 0U;
        zdt_nav_wait_end_query = 1U;
        zdt_nav_settle_valid = 0U;
        zdt_nav_settle_deadline = HAL_GetTick() + ZDT_NAV_SETTLE_TIMEOUT_MS;
        zdt_nav_state = ZDT_NAV_WAIT_END_QUERY;
        ZDT_Mileage_StartQuery(0U);
      }
      else if ((int32_t)(HAL_GetTick() - zdt_nav_hold_tick) >= 0)
      {
        /* 直走中：用陀螺保持航向（每 ZDT_NAV_HOLD_PERIOD_MS 修一次） */
        zdt_nav_hold_tick = HAL_GetTick() + ZDT_NAV_HOLD_PERIOD_MS;
        ZDT_Nav_HeadingHold();
      }
      break;

    case ZDT_NAV_BLEND:
    case ZDT_NAV_BLEND_STOP:
      ZDT_Blend_Service();
      break;
    case ZDT_NAV_OUTER_CORNER:
      ZDT_Outer_Service();
      break;

    case ZDT_NAV_ARC:
      /* 圆弧过弯：定时**前馈** + 陀螺**收工判据**。
         "转够了没有"由姿态传感器决定（现场要求）：|目标航向 − 当前航向| ≤ 1° 就停车，
         规划时长只当超时兜底 —— 麦轮边走边转一定打滑，按时间收工会留下十几度偏航。
         期间每 ZDT_NAV_HOLD_PERIOD_MS 按"线性推进的期望航向"纠一次 omega。 */
      {
        float arc_error = (zdt_nav_arc_start_heading + zdt_nav_arc_delta_yaw) -
                          ZDT_Nav_GetHeading();

        while (arc_error > 180.0f) { arc_error -= 360.0f; }
        while (arc_error < -180.0f) { arc_error += 360.0f; }

        if (fabsf(arc_error) <= ZDT_NAV_ARC_DONE_DEG)
        {
          zdt_nav_arc_by_heading = 1U;
          (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
          zdt_nav_end_ready = 0U;
          zdt_nav_query_failed = 0U;
          zdt_nav_wait_end_query = 1U;
          zdt_nav_settle_valid = 0U;
          zdt_nav_settle_deadline = HAL_GetTick() + ZDT_NAV_SETTLE_TIMEOUT_MS;
          zdt_nav_state = ZDT_NAV_WAIT_END_QUERY;
          ZDT_Mileage_StartQuery(0U);
        }
        else if ((int32_t)(HAL_GetTick() - zdt_nav_arc_deadline) >= 0)
        {
          char arc_line[112];

          (void)snprintf(arc_line, sizeof(arc_line),
                         "[NAV] arc timeout: heading still off %+.1f deg after %lums"
                         " (wheels slipping); stopping anyway\r\n",
                         (double)arc_error,
                         (unsigned long)(HAL_GetTick() - zdt_nav_arc_tick));
          ZDT_Console_Print(arc_line);
          (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
          zdt_nav_end_ready = 0U;
          zdt_nav_query_failed = 0U;
          zdt_nav_wait_end_query = 1U;
          zdt_nav_settle_valid = 0U;
          zdt_nav_settle_deadline = HAL_GetTick() + ZDT_NAV_SETTLE_TIMEOUT_MS;
          zdt_nav_state = ZDT_NAV_WAIT_END_QUERY;
          ZDT_Mileage_StartQuery(0U);
        }
        else if ((int32_t)(HAL_GetTick() - zdt_nav_hold_tick) >= 0)
        {
          zdt_nav_hold_tick = HAL_GetTick() + ZDT_NAV_HOLD_PERIOD_MS;
          ZDT_Nav_ArcHold();
        }
      }
      break;

    case ZDT_NAV_SEND_STEPS:
      /* 逐轮下发（每轮之间留 ZDT_NAV_STEP_GAP_MS），全部发完后开始计时。 */
      if ((int32_t)(HAL_GetTick() - zdt_nav_send_tick) >= 0)
      {
        if (ZDT_Nav_SendPositionMoveStep(zdt_nav_send_index,
                                         zdt_nav_send_rev,
                                         zdt_nav_send_sign,
                                         zdt_nav_send_rpm) != HAL_OK)
        {
          ZDT_Nav_Fail("position command rejected");
          break;
        }
        zdt_nav_send_index++;
        zdt_nav_send_tick = HAL_GetTick() + ZDT_NAV_STEP_GAP_MS;
        if (zdt_nav_send_index >= ZDT_MILEAGE_MOTOR_COUNT)
        {
          zdt_nav_motion_deadline = HAL_GetTick() +
              ZDT_Nav_PositionTimeMs(zdt_nav_send_rev, zdt_nav_send_rpm);
          zdt_nav_state = ZDT_NAV_MOVE;
        }
      }
      break;

    case ZDT_NAV_MOVE:
      if ((int32_t)(HAL_GetTick() - zdt_nav_motion_deadline) >= 0)
      {
        zdt_nav_end_ready = 0U;
        zdt_nav_query_failed = 0U;
        zdt_nav_wait_end_query = 1U;
        zdt_nav_settle_valid = 0U;
        zdt_nav_settle_deadline = HAL_GetTick() + ZDT_NAV_SETTLE_TIMEOUT_MS;
        zdt_nav_state = ZDT_NAV_WAIT_END_QUERY;
        ZDT_Mileage_StartQuery(0U);
      }
      break;

    case ZDT_NAV_WAIT_HEADING:
      /* 保留状态（位置模式/将来加"段中航向回正"时会用到）；当前流程不进入。 */
      if (ChassisMotor_IsTurnActive() == 0U)
      {
        zdt_nav_end_ready = 1U;
        zdt_nav_state = ZDT_NAV_WAIT_END_QUERY;
      }
      break;

    case ZDT_NAV_WAIT_END_QUERY:
      if (zdt_nav_query_failed != 0U)
      {
        ZDT_Nav_Fail("end odometry query");
      }
      else if (zdt_nav_end_ready != 0U)
      {
        float error;
        uint8_t wheels_moving;

        /* 先确认四个轮子真的停稳。位置模式的实际耗时由驱动器加速度决定，
           只按时间估算会在轮子还在转的时候就进入下一段/转角矫正，
           现场表现为"上一段还没走完就开始转向或平移"。 */
        if (zdt_nav_settle_valid == 0U)
        {
          memcpy(zdt_nav_settle_sample, zdt_nav_end_position,
                 sizeof(zdt_nav_settle_sample));
          zdt_nav_settle_valid = 1U;
          zdt_nav_settle_sample_tick = HAL_GetTick();
          zdt_nav_end_ready = 0U;
          zdt_nav_wait_end_query = 1U;
          ZDT_Mileage_StartQuery(0U);
          break;
        }
        if ((HAL_GetTick() - zdt_nav_settle_sample_tick) <
            ZDT_NAV_SETTLE_MIN_MS)
        {
          /* 两次采样间隔太短，结论不可信：保留 end_ready，下一拍再判。 */
          break;
        }
        wheels_moving = ZDT_Nav_PositionsMoving(zdt_nav_settle_sample,
                                               zdt_nav_end_position);
        if ((MatchControl_IsActive(&zdt_match) != 0U) &&
            (Nav_Params()->position_mode == 0U)) {
          int16_t applied[CHASSIS_MOTOR_COUNT];
          uint8_t wheel;
          ChassisMotor_GetWheelRpm(applied);
          for (wheel = 0U; wheel < CHASSIS_MOTOR_COUNT; ++wheel) {
            if (applied[wheel] != 0) { wheels_moving = 1U; }
          }
        }
        if ((wheels_moving != 0U) &&
            ((int32_t)(HAL_GetTick() - zdt_nav_settle_deadline) < 0))
        {
          memcpy(zdt_nav_settle_sample, zdt_nav_end_position,
                 sizeof(zdt_nav_settle_sample));
          zdt_nav_settle_sample_tick = HAL_GetTick();
          if (zdt_nav_settle_announced == 0U)
          {
            zdt_nav_settle_announced = 1U;
            ZDT_Console_Print("[NAV] waiting for wheels to settle\r\n");
          }
          zdt_nav_end_ready = 0U;
          zdt_nav_wait_end_query = 1U;
          ZDT_Mileage_StartQuery(0U);
          break;
        }

        zdt_nav_end_ready = 0U;

        if ((MatchControl_IsActive(&zdt_match) != 0U) && (wheels_moving != 0U)) {
          ZDT_Nav_Fail("wheels did not settle before timeout");
          return;
        }

        /* ★ 90° 圆弧过弯收尾（`&tarc`）：圆弧是"定时驱动 + 陀螺纠航向"的动作，
           没有单一的段方向，所以不走下面的里程复核/航向体检 —— 只做两件事：
             ① 把圆弧的**净位移（弦）**记进 mm 位姿（走过的弧长不等于位移）；
             ② 报出结束时的航向残差，残差大的话让下一段照旧做一次原地对准兜底。 */
        if (zdt_nav_arc_running != 0U)
        {
          uint16_t arc_pose_x = 0U;
          uint16_t arc_pose_y = 0U;
          int32_t arc_x;
          int32_t arc_y;
          float arc_drift;
          float arc_chord_x;
          float arc_chord_y;
          float arc_swept_deg = 0.0f;
          float arc_radius_act = 0.0f;
          char arc_line[160];

          zdt_nav_arc_running = 0U;
          GridNav_GetPoseMm(&arc_pose_x, &arc_pose_y);

          /* 净位移用**陀螺实测的转角**算（不再直接用规划弦）：
             圆弧半径 = 走过的弧长 ÷ 转过的角度。弧长取规划值（"走多少"是时间规划
             负责的、现场用 t1000 标定过），"转了多少"只有姿态传感器知道 ——
             打滑导致转慢时圆弧会被拖长（半径变大），照规划弦记位姿就会越记越偏，
             而位姿偏了正是"第二次 &goto 从错起点规划、冲出场地"那类老 bug 的根源。
             弦 = 2R·sin(θ/2)，方向 = 入射方向旋转 θ/2。 */
          {
            float swept = ZDT_Nav_GetHeading() - zdt_nav_arc_start_heading;
            float use_yaw;
            float swept_rad;
            float half_rad;
            float r_act;
            float chord_len;

            while (swept > 180.0f) { swept -= 360.0f; }
            while (swept < -180.0f) { swept += 360.0f; }
            use_yaw = (fabsf(swept) >= ZDT_NAV_ARC_MIN_SWEEP_DEG)
                          ? swept : zdt_nav_arc_delta_yaw;   /* 兜底：按规划值 */
            swept_rad = use_yaw * 0.017453292f;
            half_rad = swept_rad * 0.5f;
            r_act = (1.57079633f * zdt_nav_arc_radius_mm) /
                    ((fabsf(swept_rad) > 1.0e-4f) ? fabsf(swept_rad) : 1.0e-4f);
            chord_len = 2.0f * r_act * fabsf(sinf(half_rad));
            arc_chord_x = chord_len * ((zdt_nav_arc_ux * cosf(half_rad)) -
                                       (zdt_nav_arc_uy * sinf(half_rad)));
            arc_chord_y = chord_len * ((zdt_nav_arc_ux * sinf(half_rad)) +
                                       (zdt_nav_arc_uy * cosf(half_rad)));
            arc_swept_deg = swept;
            arc_radius_act = r_act;
          }

          arc_x = (int32_t)lroundf((float)arc_pose_x + arc_chord_x);
          arc_y = (int32_t)lroundf((float)arc_pose_y + arc_chord_y);
          if (arc_x < 0) { arc_x = 0; }
          if (arc_y < 0) { arc_y = 0; }
          if (arc_x > (int32_t)GRID_NAV_FIELD_MM) { arc_x = (int32_t)GRID_NAV_FIELD_MM; }
          if (arc_y > (int32_t)GRID_NAV_FIELD_MM) { arc_y = (int32_t)GRID_NAV_FIELD_MM; }
          GridNav_SetPoseMm((uint16_t)arc_x, (uint16_t)arc_y);

          arc_drift = ZDT_Nav_SegmentHeadingDeg(&zdt_nav_segments[zdt_nav_segment_index]) -
                      ZDT_Nav_GetHeading();
          while (arc_drift > 180.0f) { arc_drift -= 360.0f; }
          while (arc_drift < -180.0f) { arc_drift += 360.0f; }

          (void)snprintf(arc_line, sizeof(arc_line),
                         "[NAV] ARC done: R=%.0fmm(act %.0f) %+.0fdeg(meas %+.1f) %lums"
                         " %s, heading err %+.1f deg, pose now (%d,%d)\r\n",
                         (double)zdt_nav_arc_radius_mm, (double)arc_radius_act,
                         (double)zdt_nav_arc_delta_yaw, (double)arc_swept_deg,
                         (unsigned long)(HAL_GetTick() - zdt_nav_arc_tick),
                         (zdt_nav_arc_by_heading != 0U) ? "by sensor" : "by timeout",
                         (double)arc_drift,
                         (int)arc_x, (int)arc_y);
          ZDT_Console_Print(arc_line);

          if (fabsf(arc_drift) > ZDT_NAV_HEADING_MAX_DEG)
          {
            ZDT_Console_Print("[NAV] arc heading error is large; next segment will"
                           " re-align in place\r\n");
          }
          /* 残差超过对准判据就让下一段照旧做一次原地对准（兜底，几乎不花时间：
             圆弧结束时本来就已经贴近目标航向了）。 */
          if (fabsf(arc_drift) > ZDT_NAV_ALIGN_TOLERANCE_DEG)
          {
            zdt_nav_align_done = 0U;
          }
          zdt_nav_start_ready = 0U;
          zdt_nav_query_failed = 0U;
          zdt_nav_wait_start_query = 1U;
          zdt_nav_state = ZDT_NAV_WAIT_START_QUERY;
          ZDT_Mileage_StartQuery(0U);
          break;
        }

        /* ★ 平移段现在也做段末距离复核（2026-09-13）：
           原来这里对横移段直接跳过复核，理由是"横移四轮是 [1,-1,-1,1]，用直走的
           [1,1,1,1] 投影会量出 0"。这个顾虑是**多余的** —— ZDT_Nav_MeasureDistance()
           是拿**每一轮自己的投影分量**当分母（delta_rev / wheel_values[i]），
           而横移时那组分量正是 [1,-1,-1,1]，所以量出来的就是真实横向位移。
           代价是横移段一直没有距离闭环：现场"出库横移总是多/少几毫米到十几毫米"
           正好落在这一段上。 */
        zdt_nav_measured_rev = ZDT_Nav_MeasureDistance();
        if(zdt_nav_odometry_enabled && ((zdt_nav_start_mask&zdt_nav_end_mask)!=0x0FU ||
            NavFeedback_CheckOdometry(ZDT_Vobs_OdoTarget(1U,zdt_nav_segment_target_rev*ZDT_MILEAGE_MM_PER_REV),
                zdt_nav_measured_rev*ZDT_MILEAGE_MM_PER_REV)!=NAV_MOTION_OK)) {
          ZDT_Nav_Fail("odometry final segment verification failed");return;
        }
        error = (zdt_nav_odometry_enabled?ZDT_Vobs_OdoTarget(1U,zdt_nav_segment_target_rev*ZDT_MILEAGE_MM_PER_REV)/ZDT_MILEAGE_MM_PER_REV:zdt_nav_segment_target_rev) - zdt_nav_measured_rev;
        {
          char rev_line[112];
          (void)snprintf(rev_line, sizeof(rev_line),
                         "[NAV] revs FL%+.3f FR%+.3f RL%+.3f RR%+.3f spread=%.3f%s\r\n",
                         (double)zdt_nav_wheel_rev[0],
                         (double)zdt_nav_wheel_rev[1],
                         (double)zdt_nav_wheel_rev[2],
                         (double)zdt_nav_wheel_rev[3],
                         (double)zdt_nav_wheel_spread_rev,
                         (zdt_nav_drive_lateral != 0) ? " (strafe)" : "");
          ZDT_Console_Print(rev_line);
        }
        /* Timed translation uses D*(1+dcomp) upfront. Encoder residuals
         * remain visible without spread trips or MATCH end pulses.
         * WAIT_OK is an operator checkpoint, not a precision docking claim. */
        if (MatchControl_IsActive(&zdt_match) != 0U) {
          float planned_mm = fabsf(zdt_nav_segment_target_rev) * ZDT_MILEAGE_MM_PER_REV;
          float measured_mm = zdt_nav_measured_rev * ZDT_MILEAGE_MM_PER_REV;
          nav_motion_feedback_t check = zdt_nav_odometry_enabled?
            NavFeedback_CheckOdometry(ZDT_Vobs_OdoTarget(1U,planned_mm),measured_mm):NavFeedback_Check(planned_mm, measured_mm);
          char check_line[144];
          /* Spread remains diagnostic: heading hold intentionally gives the
           * two sides different RPM. No spread trip and no slow end pulses. */
          if (check == NAV_MOTION_RESIDUAL) {
            (void)snprintf(check_line, sizeof(check_line),
                           "[MATCH] distance residual %+.0fmm: plan=%.0f measured=%.0f; tune dcomp, no end pulses\r\n",
                           (double)(planned_mm-measured_mm),
                           (double)planned_mm, (double)measured_mm);
            ZDT_Console_Print(check_line);
          } else if (check != NAV_MOTION_OK) {
            (void)snprintf(check_line, sizeof(check_line),
                           "[MATCH] segment rejected: plan=%.0fmm measured=%.0fmm error=%.0fmm\r\n",
                           (double)planned_mm, (double)measured_mm,
                           (double)fabsf(planned_mm-measured_mm));
            ZDT_Console_Print(check_line);
            ZDT_Nav_Fail(NavFeedback_Reason(check));
            return;
          }
        }
        if ((MatchControl_IsActive(&zdt_match) == 0U) &&
            (ZDT_NAV_END_CORRECTION_ENABLE != 0U) &&
            (fabsf(error) > ZDT_NAV_REV_TOLERANCE) &&
            (zdt_nav_correction_count < ZDT_NAV_MAX_CORRECTIONS) &&
            (fabsf(error) <= ZDT_NAV_CORRECT_MAX_REV))
        {
          char line[96];
          zdt_nav_correction_count++;
          zdt_nav_move_sign = (error > 0.0f) ? 1 : -1;
          (void)snprintf(line, sizeof(line),
                         "[NAV] correct %+.3frev %s (%u/%u)\r\n",
                         (double)error,
                         (zdt_nav_drive_lateral != 0) ? "STRAFE" : "FWD/BACK",
                         (unsigned int)zdt_nav_correction_count,
                         (unsigned int)ZDT_NAV_MAX_CORRECTIONS);
          ZDT_Console_Print(line);
          if (zdt_nav_drive_lateral != 0)
          {
            /* 横移段：补走也必须走**横移**通道（vy），不能走直走通道（vx）——
               否则车会朝前/后补一段，横向残差原封不动，还会把 lateral 标记清掉。 */
            ZDT_Nav_BeginLateralDrive(
                fabsf(error), zdt_nav_move_sign,
                Nav_Params()->speed_scale * ZDT_NAV_CORRECT_SPEED_RATIO);
          }
          else
          {
            ZDT_Nav_StartMove(fabsf(error), zdt_nav_move_sign,
                              ZDT_NAV_POSITION_CORRECT_RPM,
                              Nav_Params()->speed_scale * ZDT_NAV_CORRECT_SPEED_RATIO);
          }
        }
        else
        {
          char line[128];

          /* 残差大到"超过一整段长度的合理范围" → 里程不可信（符号/单位/打滑）。
             注意门槛用的是 CORRECT_MAX_REV（大），不是 REV_TOLERANCE（小）：
             后者只表示"有点残差"，在补走关闭时那是**正常现象**，不该报 suspect。 */
          if (fabsf(error) > ZDT_NAV_CORRECT_MAX_REV)
          {
            (void)snprintf(line, sizeof(line),
                           "[NAV] odometry suspect %+.3frev (residual too large)\r\n",
                           (double)error);
            ZDT_Console_Print(line);
          }
          else if ((ZDT_NAV_END_CORRECTION_ENABLE == 0U) &&
                   (fabsf(error) > ZDT_NAV_REV_TOLERANCE))
          {
            /* 补走按设计关闭：只把残差报出来。要修系统性欠走/多走请动 `t1000`/`spd`。 */
            (void)snprintf(line, sizeof(line),
                           (MatchControl_IsActive(&zdt_match) != 0U) ?
                           "[NAV] residual %+.3frev (%.0fmm), tune dcomp; no end pulses\r\n" :
                           "[NAV] residual %+.3frev (%.0fmm), no correction by design\r\n",
                           (double)error,
                           (double)(error * ZDT_MILEAGE_MM_PER_REV));
            ZDT_Console_Print(line);
          }

          /* 段末不再"回正航向"：下一段开始前会按该段方向做一次原地对准，
             在这里回正只会出现"先转回去、再转过来"的多余动作。
             这里只做体检：偏差太大说明本段走行异常，停车报错。
             横移段跳过这项：它与车头本来就差 90°，不是"跑偏"。 */
          if (zdt_nav_drive_lateral == 0)
          {
            float expected=(zdt_nav_recenter_route && zdt_nav_drive_sign<0)?
                zdt_nav_lateral_hold_heading:ZDT_Nav_SegmentHeadingDeg(segment);
            float drift = expected - ZDT_Nav_GetHeading();

            while (drift > 180.0f)
            {
              drift -= 360.0f;
            }
            while (drift < -180.0f)
            {
              drift += 360.0f;
            }
            if (fabsf(drift) > ZDT_NAV_HEADING_MAX_DEG)
            {
              (void)snprintf(line, sizeof(line),
                             "[NAV] heading drift %+.1f deg > %.0f deg limit\r\n",
                             (double)drift,
                             (double)ZDT_NAV_HEADING_MAX_DEG);
              ZDT_Console_Print(line);
              ZDT_Nav_Fail("segment went off course; check wheels/odometry");
              break;
            }
            if (fabsf(drift) > ZDT_NAV_ALIGN_TOLERANCE_DEG)
            {
              (void)snprintf(line, sizeof(line),
                             "[NAV] heading drift %+.1f deg (next segment will align)\r\n",
                             (double)drift);
              ZDT_Console_Print(line);
            }
          }

          if (zdt_nav_drive_lateral != 0)
          {
            (void)snprintf(line, sizeof(line),
                           "[NAV] SEG%u done (strafe %.0fmm, no turn)\r\n",
                           (unsigned int)(zdt_nav_segment_index + 1U),
                           (double)(zdt_nav_segment_target_rev *
                                    ZDT_MILEAGE_MM_PER_REV));
          }
          else
          {
            (void)snprintf(line, sizeof(line),
                           "[NAV] SEG%u done: %.3f/%.3frev (hold peak %.2f deg)%s\r\n",
                           (unsigned int)(zdt_nav_segment_index + 1U),
                           (double)zdt_nav_measured_rev,
                           (double)zdt_nav_segment_target_rev,
                           (double)zdt_nav_hold_peak_error,
                           (zdt_nav_arc_armed != 0U) ?
                               "  [arc: leg shortened by R]" : "");
          }
          ZDT_Console_Print(line);

          /* ★必须同时推进 mm 位姿（2026-09-13 修）：
             原来这里只调 GridNav_SetPose(to_x,to_y)（格位姿），**mm 位姿一直是计划
             时那份**，于是第二次 `&goto` 的 A* 起点是错的 —— 表现就是"路线算出来是
             从上一个点出发的"，车按错误的段长直走 → 冲出场地。
             推进方式：按本段**名义位移**（格数 × 50 mm，内部 dy 取反 = 地图 +y）累加；
             若是最后一段，直接落到**精确目标 mm**（吸收格心吸附带来的 ±25 mm）。 */
          {
            uint16_t pose_x_mm = 0U;
            uint16_t pose_y_mm = 0U;
            int32_t advanced_x;
            int32_t advanced_y;
            uint16_t goal_x_mm = 0U;
            uint16_t goal_y_mm = 0U;

            GridNav_GetPoseMm(&pose_x_mm, &pose_y_mm);
            advanced_x = (int32_t)pose_x_mm +
                         ((int32_t)segment->dx *
                          (int32_t)GridNav_GetSegmentDistanceMm(zdt_nav_segment_index));
            advanced_y = (int32_t)pose_y_mm -
                         ((int32_t)segment->dy *
                          (int32_t)GridNav_GetSegmentDistanceMm(zdt_nav_segment_index));
            /* 本段末尾要接圆弧：车实际只走到角点前 R 处（圆弧会走掉剩下的 R），
               位姿必须按**实际走的距离**推进，否则圆弧起点就错了。 */
            if (zdt_nav_arc_armed != 0U)
            {
              advanced_x -= (int32_t)lroundf((float)segment->dx *
                                             zdt_nav_arc_radius_mm);
              advanced_y += (int32_t)lroundf((float)segment->dy *
                                             zdt_nav_arc_radius_mm);
            }
            if(zdt_nav_odometry_enabled) {
              advanced_x=(int32_t)pose_x_mm+(int32_t)lroundf(segment->dx*zdt_nav_measured_rev*ZDT_MILEAGE_MM_PER_REV);
              advanced_y=(int32_t)pose_y_mm-(int32_t)lroundf(segment->dy*zdt_nav_measured_rev*ZDT_MILEAGE_MM_PER_REV);
            }
            if (!zdt_nav_odometry_enabled && (zdt_nav_segment_index + 1U) >= zdt_nav_segment_count)
            {
              GridNav_GetTargetMm(&goal_x_mm, &goal_y_mm);
              advanced_x = (int32_t)goal_x_mm;
              advanced_y = (int32_t)goal_y_mm;
            }
            if (advanced_x < 0) { advanced_x = 0; }
            if (advanced_y < 0) { advanced_y = 0; }
            if (advanced_x > (int32_t)GRID_NAV_FIELD_MM)
            {
              advanced_x = (int32_t)GRID_NAV_FIELD_MM;
            }
            if (advanced_y > (int32_t)GRID_NAV_FIELD_MM)
            {
              advanced_y = (int32_t)GRID_NAV_FIELD_MM;
            }
            GridNav_SetPoseMm((uint16_t)advanced_x, (uint16_t)advanced_y);
            (void)snprintf(line, sizeof(line),
                           "[NAV] pose now (%umm,%umm) cell=(%u,%u) after SEG%u\r\n",
                           (unsigned int)advanced_x, (unsigned int)advanced_y,
                           (unsigned int)GridNav_MmToCellX((uint16_t)advanced_x),
                           (unsigned int)GridNav_MmToCellY((uint16_t)advanced_y),
                           (unsigned int)(zdt_nav_segment_index + 1U));
            ZDT_Console_Print(line);
          }
          zdt_nav_segment_index++;
          if (zdt_nav_segment_index >= zdt_nav_segment_count)
          {
            if(zdt_nav_odometry_enabled)zdt_nav_post_align=1U;
            ZDT_Console_Print("[NAV] ARRIVED.\r\n");
            zdt_nav_state = ZDT_NAV_IDLE;
            ZDT_Nav_NotifyArrived();
          }
          else
          {
            zdt_nav_segment_target_rev =
                (float)GridNav_GetSegmentDistanceMm(zdt_nav_segment_index) /
                ZDT_MILEAGE_MM_PER_REV;
            zdt_nav_correction_count = 0U;
            zdt_nav_move_sign = 1;
            zdt_nav_settle_announced = 0U;
            zdt_nav_align_done = 0U;   /* 新的一段：需要重新对准方向 */
            /* ★ 本段末尾武装了圆弧：不走"回 WAIT_START_QUERY 再原地对准"，
               直接进 ZDT_NAV_ARC 边走边转（ZDT_Nav_BeginArc 会把新段目标减 R、
               并把 align_done 置 1，圆弧走完就不需要再对准了）。 */
            if (zdt_nav_arc_armed != 0U)
            {
              zdt_nav_arc_armed = 0U;
              ZDT_Nav_BeginArc();
              break;
            }
            /* 每一段都必须回到 WAIT_START_QUERY：那里既重采里程基准，
               也会按新段方向做"原地对准"。栅格路径里相邻两段方向一定不同
               （同方向连续格已在 grid_nav 合并），所以不能跳过对准直接开走，
               否则车会朝着上一段的方向直走（表现为"段末航向漂移 90°、停车报错"）。 */
            zdt_nav_start_ready = 0U;
            zdt_nav_query_failed = 0U;
            zdt_nav_wait_start_query = 1U;
            zdt_nav_state = ZDT_NAV_WAIT_START_QUERY;
            ZDT_Mileage_StartQuery(0U);
          }
        }
      }
      break;

    default:
      zdt_nav_state = ZDT_NAV_IDLE;
      break;
  }
}

static HAL_StatusTypeDef ZDT_Motor_SendEnable(uint8_t address, uint8_t enable,
                                              uint8_t buffered,
                                              uint8_t log_enabled)
{
  uint8_t data[5] = {0xF3U, 0xABU, enable, buffered, ZDT_CAN_CHECKSUM};
  return ZDT_CAN_SendToAddress(address, data, 5U,
                               (enable != 0U) ? "enable" : "disable",
                               log_enabled);
}

HAL_StatusTypeDef ZDT_Motor_Enable(void)
{
  return ZDT_Motor_SendEnable(ZDT_CAN_MOTOR_ADDRESS, 1U, 0U, 1U);
}

HAL_StatusTypeDef ZDT_Motor_Disable(void)
{
  return ZDT_Motor_SendEnable(ZDT_CAN_MOTOR_ADDRESS, 0U, 0U, 1U);
}

HAL_StatusTypeDef ZDT_Motor_Stop(void)
{
  return ZDT_Motor_StopAddr(ZDT_CAN_MOTOR_ADDRESS);
}

HAL_StatusTypeDef ZDT_Motor_EnableAddr(uint8_t address, uint8_t buffered)
{
  return ZDT_Motor_SendEnable(address, 1U, (buffered != 0U) ? 1U : 0U, 0U);
}

HAL_StatusTypeDef ZDT_Motor_DisableAddr(uint8_t address, uint8_t buffered)
{
  return ZDT_Motor_SendEnable(address, 0U, (buffered != 0U) ? 1U : 0U, 0U);
}

HAL_StatusTypeDef ZDT_Motor_StopAddr(uint8_t address)
{
  const uint8_t data[4] = {0xFEU, 0x98U, 0x00U, ZDT_CAN_CHECKSUM};
  return ZDT_CAN_SendToAddress(address, data, 4U, "immediate stop", 0U);
}

static HAL_StatusTypeDef ZDT_Motor_SetSpeedRpmInternal(uint8_t address,
                                                        int16_t rpm,
                                                        uint8_t buffered,
                                                        uint8_t log_enabled)
{
  uint16_t magnitude;
  uint8_t direction;
  uint8_t sync = (buffered != 0U) ? 1U : 0U;
  char label[48];

  if (((long)rpm > ZDT_CAN_RUN_MAX_RPM) ||
      ((long)rpm < -ZDT_CAN_RUN_MAX_RPM))
  {
    if (log_enabled != 0U)
    {
      ZDT_Console_Print("[ZDT] ERROR: RPM must be an integer from -3000 to +3000.\r\n");
    }
    return HAL_ERROR;
  }

  direction = (rpm < 0) ? 0x01U : 0x00U;
  /* 实测本车驱动器把速度字段按 0.1 RPM 解释（下发 126 → 实际约 12.6 RPM，
     与厂商手册 X 固件"速度 0-30000 (0.1RPM)"一致），因此发送前 ×10。
     若以后换成按 RPM 解释的固件，把 ZDT_SPEED_FIELD_SCALE 改回 1.0f 即可。 */
  magnitude = (uint16_t)((((rpm < 0) ? -rpm : rpm)) * ZDT_SPEED_FIELD_SCALE);
  if (magnitude > 30000U)
  {
    magnitude = 30000U;
  }

  if (log_enabled != 0U)
  {
    (void)snprintf(label, sizeof(label), "speed %s %d RPM",
                   (direction == 0U) ? "CW" : "CCW", (int)rpm);
  }
  else
  {
    label[0] = '\0';
  }

#if ZDT_FIRMWARE_X
  {
    uint16_t speed_0p1 = magnitude;   /* magnitude 已是 0.1RPM 单位 */
    uint8_t data[8];

    data[0] = 0xF6U;
    data[1] = direction;
    data[2] = 0x00U;
    data[3] = 0x00U;
    data[4] = (uint8_t)(speed_0p1 >> 8U);
    data[5] = (uint8_t)speed_0p1;
    data[6] = sync;
    data[7] = ZDT_CAN_CHECKSUM;
    return ZDT_CAN_SendToAddress(address, data, 8U, label, log_enabled);
  }
#else
  {
    uint8_t data[7];

    data[0] = 0xF6U;
    data[1] = direction;
    data[2] = (uint8_t)(magnitude >> 8U);
    data[3] = (uint8_t)magnitude;
    data[4] = ZDT_CAN_RUN_ACCELERATION;
    data[5] = sync;
    data[6] = ZDT_CAN_CHECKSUM;
    return ZDT_CAN_SendToAddress(address, data, 7U, label, log_enabled);
  }
#endif
}
HAL_StatusTypeDef ZDT_Motor_SetSpeedRpm(int16_t rpm)
{
  if (rpm == 0)
  {
    return ZDT_Motor_Stop();
  }
  return ZDT_Motor_SetSpeedRpmInternal(ZDT_CAN_MOTOR_ADDRESS, rpm, 0U, 1U);
}

HAL_StatusTypeDef ZDT_Motor_SetSpeedRpmAddr(uint8_t address, int16_t rpm,
                                            uint8_t buffered)
{
  if ((rpm == 0) && (buffered == 0U))
  {
    return ZDT_Motor_StopAddr(address);
  }
  return ZDT_Motor_SetSpeedRpmInternal(address, rpm,
                                        (buffered != 0U) ? 1U : 0U, 0U);
}

static HAL_StatusTypeDef ZDT_MovePositionModeAddr(uint8_t address,
                                                   int32_t pulses,
                                                   uint16_t speed_rpm,
                                                   uint8_t acceleration,
                                                   uint8_t buffered,uint8_t mode)
{
  uint32_t magnitude = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;
  uint8_t direction = (pulses < 0) ? 0x01U : 0x00U;
  uint8_t sync = (buffered != 0U) ? 1U : 0U;
  uint8_t packet0[8];
  uint8_t packet1[5];
  HAL_StatusTypeDef result;

  if ((speed_rpm == 0U) || (speed_rpm > (uint16_t)ZDT_CAN_RUN_MAX_RPM))
  {
    return HAL_ERROR;
  }

  /* Emm FD position frame is 12 bytes, so split it into two CAN packets. */
  packet0[0] = 0xFDU;
  packet0[1] = direction;
  packet0[2] = (uint8_t)(speed_rpm >> 8U);
  packet0[3] = (uint8_t)speed_rpm;
  packet0[4] = acceleration;
  packet0[5] = (uint8_t)(magnitude >> 24U);
  packet0[6] = (uint8_t)(magnitude >> 16U);
  packet0[7] = (uint8_t)(magnitude >> 8U);

  packet1[0] = 0xFDU; /* every split packet repeats the function code */
  packet1[1] = (uint8_t)magnitude;
  packet1[2] = mode; /* 01=absolute; 02=current-position relative (new firmware). */
  packet1[3] = sync;
  packet1[4] = ZDT_CAN_CHECKSUM;

  result = ZDT_CAN_SendPacketToAddress(address, 0U, packet0, 8U,
                                       "position move p0", 0U);
  if (result == HAL_OK)
  {
    result = ZDT_CAN_SendPacketToAddress(address, 1U, packet1, 5U,
                                         "position move p1", 0U);
  }
  return result;
}

HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAddr(uint8_t address,int32_t pulses,
    uint16_t speed_rpm,uint8_t acceleration,uint8_t buffered)
{return ZDT_MovePositionModeAddr(address,pulses,speed_rpm,acceleration,buffered,0x02U);}
HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAbsAddr(uint8_t address,int32_t pulses,
    uint16_t speed_rpm,uint8_t acceleration,uint8_t buffered)
{return ZDT_MovePositionModeAddr(address,pulses,speed_rpm,acceleration,buffered,0x01U);}

HAL_StatusTypeDef ZDT_Motor_SyncTrigger(void)
{
  const uint8_t data[3] = {0xFFU, 0x66U, ZDT_CAN_CHECKSUM};
  return ZDT_CAN_SendToAddress(ZDT_CAN_BROADCAST_ADDRESS, data, 3U,
                               "sync trigger", 0U);
}

void ZDT_Motor_SetVerbose(uint8_t enabled)
{
  zdt_verbose_output = (enabled != 0U) ? 1U : 0U;
}
static uint32_t ZDT_CanKernelClockHz(void)
{
  PLL1_ClocksTypeDef pll1;
  PLL2_ClocksTypeDef pll2;

  switch (__HAL_RCC_GET_FDCAN_SOURCE())
  {
    case RCC_FDCANCLKSOURCE_PLL:
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) { return 0U; }
      HAL_RCCEx_GetPLL1ClockFreq(&pll1);
      return pll1.PLL1_Q_Frequency;
    case RCC_FDCANCLKSOURCE_PLL2:
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) == RESET) { return 0U; }
      HAL_RCCEx_GetPLL2ClockFreq(&pll2);
      return pll2.PLL2_Q_Frequency;
    case RCC_FDCANCLKSOURCE_HSE:
      return (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) ? HSE_VALUE : 0U;
    default:
      return 0U;
  }
}

static const char *ZDT_CanLecName(uint32_t lec)
{
  static const char *const names[8] =
  {
    "none", "stuff", "form", "ACK", "bit1", "bit0", "CRC", "no-change"
  };
  return (lec < 8U) ? names[lec] : "unknown";
}

static void ZDT_Print_Status(void)
{
  FDCAN_ProtocolStatusTypeDef protocol = {0};
  FDCAN_ErrorCountersTypeDef counters = {0};
  uint32_t nbtp = hfdcan2.Instance->NBTP;
  uint32_t brp = ((nbtp & FDCAN_NBTP_NBRP) >> FDCAN_NBTP_NBRP_Pos) + 1U;
  uint32_t seg1 = ((nbtp & FDCAN_NBTP_NTSEG1) >> FDCAN_NBTP_NTSEG1_Pos) + 1U;
  uint32_t seg2 = ((nbtp & FDCAN_NBTP_NTSEG2) >> FDCAN_NBTP_NTSEG2_Pos) + 1U;
  uint32_t div_code = (FDCAN_CCU->CCFG & FDCANCCU_CCFG_CDIV) >>
                       FDCANCCU_CCFG_CDIV_Pos;
  uint32_t divider = (div_code == 0U) ? 1U : (div_code * 2U);
  uint32_t clock_hz = ZDT_CanKernelClockHz();
  uint32_t rate = clock_hz / (divider * brp * (1U + seg1 + seg2));
  uint32_t last_lec = zdt_status.last_protocol_lec;
  char line[192];

  (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol);
  (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters);
  (void)snprintf(line, sizeof(line),
                 "[ZDT] Status: CAN=%s, version reply=%s, address=%u, CAN error=0x%08lX\r\n",
                 (zdt_status.can_ready != 0U) ? "listening" : "not-listening",
                 (zdt_status.motor_responded != 0U) ? "received" : "not received",
                 (unsigned int)ZDT_CAN_MOTOR_ADDRESS,
                 (unsigned long)HAL_FDCAN_GetError(&hfdcan2));
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[ZDT] BusOff=%u warning=%u passive=%u TEC=%lu REC=%lu RX=%lu dropped=%lu\r\n",
                 (unsigned int)protocol.BusOff, (unsigned int)protocol.Warning,
                 (unsigned int)protocol.ErrorPassive,
                 (unsigned long)counters.TxErrorCnt,
                 (unsigned long)counters.RxErrorCnt,
                 (unsigned long)zdt_status.rx_count,
                 (unsigned long)zdt_status.rx_dropped);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[ZDT] protocol: LEC=%lu(%s) last=%lu(%s) errors=%lu\r\n",
                 (unsigned long)protocol.LastErrorCode,
                 ZDT_CanLecName(protocol.LastErrorCode),
                 (unsigned long)last_lec, ZDT_CanLecName(last_lec),
                 (unsigned long)zdt_status.protocol_error_count);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[ZDT] timing: kernel=%luHz div=%lu NBTP=0x%08lX brp=%lu seg1=%lu seg2=%lu calculated_rate=%lubit/s\r\n",
                 (unsigned long)clock_hz, (unsigned long)divider,
                 (unsigned long)nbtp, (unsigned long)brp,
                 (unsigned long)seg1, (unsigned long)seg2, (unsigned long)rate);
  ZDT_Console_Print(line);
  if (clock_hz == 0U)
  {
    ZDT_Console_Print("[ZDT] clock unavailable; calculated_rate=0 is not a measured bitrate.\r\n");
  }
  ZDT_LogStatus();
  /* UART4 输入丢字节计数：**非 0 就说明命令会缺字/被腰斩**，是"发出去没反应/
   * 卡住"的头号嫌疑。字符回显（默认关）是它的主要来源，见 `echo 0|1`。 */
  (void)snprintf(line, sizeof(line),
                 "[ZDT] uart rx: dropped=%lu bytes, char-echo=%u (uecho 0|1)\r\n",
                 (unsigned long)zdt_uart_rx_dropped,
                 (unsigned int)zdt_uart_echo_enabled);
  ZDT_Console_Print(line);
  (void)snprintf(line, sizeof(line),
                 "[ZDT] UART4 diag: irq=%lu bytes=%lu ore=%lu fe=%lu ne=%lu pe=%lu recover=%lu txfail=%lu\r\n",
                 (unsigned long)zdt_status.uart_rx_irq,
                 (unsigned long)zdt_status.uart_rx_bytes,
                 (unsigned long)zdt_status.uart_rx_ore,
                 (unsigned long)zdt_status.uart_rx_fe,
                 (unsigned long)zdt_status.uart_rx_ne,
                 (unsigned long)zdt_status.uart_rx_pe,
                 (unsigned long)zdt_status.uart_rx_recover,
                 (unsigned long)zdt_status.uart_tx_failed);
  ZDT_Console_Print(line);
  if (zdt_uart_rx_dropped != 0U)
  {
    ZDT_Console_Print("[ZDT] WARNING: bytes were dropped on UART4 -> commands can\r\n"
                      "      arrive truncated. Send commands with a pause between them,\r\n"
                      "      and keep char-echo off (`uecho 0`).\r\n");
  }
}

static void ZDT_Process_Command(char *command)
{
  char *end;
  long rpm;

  if (ZDT_UserQuery_Command(command) != 0U) { return; }
  if (ZDT_Match_Command(command, HAL_GetTick()) != 0U) { return; }
  if (Vision_ProcessCommand(command) != 0U) { return; }
  if (ZDT_ArmCommand(command) != 0U) { return; }

  {
    uint32_t revision = GridNav_GetRevision();
    ZDT_Nav_RefreshCostModel();   /* 控制台路径同样：规划前按当前 spd 折算转向代价 */
    if (GridNav_ProcessCommand(command) != 0U)
    {
      if ((ZDT_CommandIsGoto(command) != 0U) &&
          (GridNav_GetRevision() != revision))
      {
        ZDT_Nav_Start();
      }
      return;
    }
  }
  if (DM_Console_ProcessCommand(ArmTask_GetMotor(), command) != 0U)
  {
    return;
  }
  if (PID_Storage_ProcessCommand(command) != 0U)
  {
    return;
  }
  /* 无 '&' 的裸串口写法：tstat / ttrace 1 也能用（& 形式走蓝牙 tokenizer 那条路）。
   * 放在 PID 之后不影响 &kp/&ki/&kd 的解析。 */
  if (ZDT_Turn_Command(command) != 0U)
  {
    return;
  }
  if (Nav_Params_ProcessCommand(command) != 0U)
  {
    return;
  }

  if (strcmp(command, "verbose") == 0)
  {
    ZDT_Motor_SetVerbose(1U);
    ZDT_Console_Print("[ZDT] CAN frame logging enabled.\r\n");
  }
  else if (strcmp(command, "posref") == 0)
  {
    /* 记录四轮当前位置作为基准：手转一圈后再 posd，即可读出"计数/圈"。 */
    zdt_pos_ref_request = 1U;
    ZDT_Console_Print("[ZDT] posref: sampling current positions...\r\n");
    ZDT_Mileage_StartQuery(0U);
  }
  else if (strcmp(command, "posd") == 0)
  {
    zdt_pos_delta_request = 1U;
    ZDT_Console_Print("[ZDT] posd: sampling and comparing with posref...\r\n");
    ZDT_Mileage_StartQuery(0U);
  }
  else if (strcmp(command, "poscal") == 0)
  {
    /* 逐轮自标定：测出每轮的脉冲当量、是否执行 0xFD、回读符号是否与指令同号。 */
    ZDT_PosCal_Start();
  }
  else if (strcmp(command, "verall") == 0)
  {
    /* 逐地址读驱动器版本：判断四轮是否同一固件（0xFD 不执行的常见原因）。 */
    ZDT_CAN_SendAllVersionQueries();
  }
  else if (strncmp(command, "posmode", 7U) == 0)
  {
    if(zdt_nav_odometry_enabled) {
      Nav_Params()->position_mode=0U;
      ZDT_Console_Print("[NAV] ODOMETRY_V2_4 active; posmode cannot select timed/0xFD transport.\r\n");return;
    }
    /* posmode / posmode 0 / posmode 1：直线段执行方式（速度+定时 / 位置模式）。 */
    if ((command[7] == ' ') && ((command[8] == '0') || (command[8] == '1')))
    {
      Nav_Params()->position_mode = (uint8_t)(command[8] - '0');
    }
    ZDT_Console_Print(Nav_Params()->position_mode != 0U ?
                   "[ZDT] nav mode = POSITION (0xFD pulses)\r\n" :
                   "[ZDT] nav mode = VELOCITY + timed drive\r\n");
  }
  else if ((strcmp(command, "dis") == 0) || (strcmp(command, "off") == 0))
  {
    /* 关闭四轮使能：闭环步进解除锁轴，可以手转轮子（posref/posd 标定用）。
     * 2026-09-14：达妙转盘也一起失能 —— 这台车的"松劲"必须是整车语义，
     * 否则现场按 dis 之后轮子能推、转盘还带力矩，手一伸就会被夹。 */
    (void)ChassisMotor_Disable();
    (void)ArmTask_Disable();
    ZDT_Console_Print("[ZDT] motors disabled; wheels turn freely by hand.\r\n");
    ZDT_Console_Print("[ARM] turntable joint disabled too (shaft free).\r\n");
  }
  else if (strcmp(command, "zero") == 0)
  {
    /* 把当前朝向重新当作 0°：摆好车（车头贴地图轴线）后再对基准，
       否则所有"前进"都会斜着走。之后约 0.1 s 内不要动车。
       同时把位姿估计重置到原点（当前位置 = 地图原点）。 */
    ChassisMotor_ClearHeadingReference();HWT101_ResetZeroDrift();
    ZDT_Console_Print("[ZDT] heading zero re-armed; keep still for ~3s.\r\n");
  }
  else if (strcmp(command, "posq") == 0)
  {
    /* 诊断：查询四轮 0x36 实时位置，并把每一帧原始数据打印出来，
       用于核对驱动器回读的帧格式与位置单位（Emm: 65536=1圈，X: 3600=1圈）。 */
    zdt_pos_dump_frames = 16U;
    ZDT_Console_Print("[ZDT] position dump: sending 0x36 to 4 wheels.\r\n");
    ZDT_Mileage_StartQuery(1U);
  }
  else if (strcmp(command, "echo") == 0)
  {
    ZDT_Print_Echo();
  }
  else if (strcmp(command, "quiet") == 0)
  {
    ZDT_Motor_SetVerbose(0U);
    ZDT_Console_Print("[ZDT] CAN frame logging disabled.\r\n");
  }
  else if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0))
  {
    ZDT_Print_Help();
  }
  else if ((strcmp(command, "ver") == 0) ||
           (strcmp(command, "version") == 0))
  {
    ZDT_CAN_SendVersionQuery();
  }
  else if (strcmp(command, "yaw") == 0)
  {
    char yaw_line[192];
    (void)snprintf(yaw_line, sizeof(yaw_line),
                   "[YAW] raw=%d.%02d zero=%d.%02d gyro=%d.%02d bias=%d.%03d ready=%u cal=%u turn=%u\r\n",
                   (int)HWT101_GetYawDeg(),
                   (int)(ZDT_AbsFloat(HWT101_GetYawDeg() - (float)(int)HWT101_GetYawDeg()) * 100.0f),
                   (int)HWT101_GetZeroDriftDeg(),
                   (int)(ZDT_AbsFloat(HWT101_GetZeroDriftDeg() - (float)(int)HWT101_GetZeroDriftDeg()) * 100.0f),
                   (int)HWT101_GetGyroZDps(),
                   (int)(ZDT_AbsFloat(HWT101_GetGyroZDps() - (float)(int)HWT101_GetGyroZDps()) * 100.0f),
                   (int)HWT101_GetGyroBiasDps(),
                   (int)(ZDT_AbsFloat(HWT101_GetGyroBiasDps() - (float)(int)HWT101_GetGyroBiasDps()) * 1000.0f),
                   (unsigned int)HWT101_IsZeroDriftReady(),
                   (unsigned int)HWT101_IsCalibrating(),
                   (unsigned int)ChassisMotor_IsTurnActive());
    ZDT_Console_Print(yaw_line);
    {
      chassis_telemetry_t t;float raw;
      ChassisMotor_GetTelemetry(&t);raw=HWT101_GetZeroDriftDeg()*CHASSIS_YAW_RIGHT_POSITIVE_SIGN;
      (void)snprintf(yaw_line,sizeof(yaw_line),"[YAW] navigation=%+.3fdeg sensor_ccw=%+.3fdeg reference_offset=%+.3fdeg\r\n",
          (double)t.heading_deg,(double)raw,(double)(t.heading_deg-raw));ZDT_Console_Print(yaw_line);
    }
  } else if(strcmp(command,"logstat")==0){ZDT_LogStatus();}
  else if (strcmp(command, "status") == 0)
  {
    ZDT_Print_Status();
  }
  else if ((strncmp(command, "uecho", 5U) == 0) &&
           ((command[5] == '\0') || (command[5] == ' ')))
  {
    /* `uecho 0|1`：UART **字符回显**开关（默认 0 = 关）。
     * 打开对"手工敲键盘"有用，但在 9600 baud 下每字符阻塞 ~1 ms，
     * 蓝牙 App 一次推整行时会挤爆 RX 缓冲 → 命令缺字。见 ZDT_UART_ProcessByte。
     * 注意：这与 `&echo`（应用层链路自检）完全是两回事，所以名字不叫 `echo`。
     *
     * ⚠ 参数必须用 ZDT_SkipSpaces() 取，**不能**写 command[6]：蓝牙 token 里
     * 空格是保留在 token 内的（"uecho 1" 的第 5 位就是那个空格），固定偏移会错位。
     * tools/check_arg_parsing.py 专门守这条规则。 */
    const char *arg = ZDT_SkipSpaces(command + 5U);

    if (*arg == '\0')
    {
      ZDT_Console_Print(zdt_uart_echo_enabled != 0U ?
                        "[ZDT] uart char echo = 1 (on; slows bulk input)\r\n" :
                        "[ZDT] uart char echo = 0 (off; recommended)\r\n");
    }
    else if (*arg == '1')
    {
      zdt_uart_echo_enabled = 1U;
      ZDT_Console_Print("[ZDT] uart char echo = 1. WARNING: at 9600 baud this blocks\r\n"
                        "      ~1 ms per character; for hand typing only. Send `uecho 0` after.\r\n");
    }
    else if (*arg == '0')
    {
      zdt_uart_echo_enabled = 0U;
      ZDT_Console_Print("[ZDT] uart char echo = 0 (off; recommended)\r\n");
    }
    else
    {
      ZDT_Console_Print("[ZDT] usage: uecho [0|1]   (uart character echo)\r\n");
    }
  }
  else if (strcmp(command, "enable") == 0)
  {
    (void)ZDT_Motor_Enable();
  }
  else if (strcmp(command, "disable") == 0)
  {
    (void)ZDT_Motor_Disable();
  }
  else if (strcmp(command, "stop") == 0)
  {
    ArmTask_RequestStop();
    chassis_command_t previous_command = ChassisMotor_GetCommand();
    ZDT_Match_CancelMotion();
    ZDT_Nav_Cancel();
    (void)ChassisMotor_SetCommand(CHASSIS_COMMAND_STOP);
    ZDT_Mileage_RequestStop(
        (previous_command == CHASSIS_COMMAND_FORWARD) ? 1U : 0U);
  }
  else if (strncmp(command, "test ", 5U) == 0)
  {
    long address_value;
    char *rpm_text;
    end = &command[5];
    address_value = strtol(end, &rpm_text, 10);
    if(address_value==5L || address_value==6L) { ZDT_Console_Print("[ARM] use arm pulse/mm for ID5/6\r\n");return; }
    while (*rpm_text == ' ')
    {
      rpm_text++;
    }
    if ((address_value < 1L) || (address_value > 255L) || (*rpm_text == '\0'))
    {
      ZDT_Console_Print("[ZDT] Usage: test <address 1-255> <rpm -3000..3000>.\r\n");
    }
    else
    {
      rpm = strtol(rpm_text, &end, 10);
      while (*end == ' ')
      {
        end++;
      }
      if ((*end != '\0') || (rpm > ZDT_CAN_RUN_MAX_RPM) ||
          (rpm < -ZDT_CAN_RUN_MAX_RPM))
      {
        ZDT_Console_Print("[ZDT] Usage: test <address 1-255> <rpm -3000..3000>.\r\n");
      }
      else
      {
        (void)ZDT_Motor_SendEnable((uint8_t)address_value, 1U, 0U, 1U);
        (void)ZDT_Motor_SetSpeedRpmInternal((uint8_t)address_value,
                                             (int16_t)rpm, 0U, 1U);
      }
    }
  }
  else if (strncmp(command, "zpos ", 5U) == 0)
  {
    long address_value;
    long pulses;
    long speed;
    char *pulse_text;
    char *speed_text;

    end = &command[5];
    address_value = strtol(end, &pulse_text, 10);
    if(address_value==5L || address_value==6L) { ZDT_Console_Print("[ARM] use arm pulse/mm for ID5/6\r\n");return; }
    pulses = strtol(pulse_text, &speed_text, 10);
    speed = strtol(speed_text, &end, 10);
    while (*end == ' ') { end++; }
    if ((address_value < 1L) || (address_value > 255L) ||
        (speed <= 0L) || (speed > 3000L) || (*end != '\0') ||
        (pulses < -2147483647L) || (pulses > 2147483647L))
    {
      ZDT_Console_Print("[ZDT] Usage: zpos <addr> <pulses> <rpm>.\r\n");
    }
    else
    {
      (void)ZDT_Motor_SendEnable((uint8_t)address_value, 1U, 0U, 1U);
      if (ZDT_Motor_MovePositionPulsesAddr((uint8_t)address_value,
                                           (int32_t)pulses,
                                           (uint16_t)speed, 200U,
                                           0U) == HAL_OK)
      {
        ZDT_Console_Print("[ZDT] position test sent.\r\n");
      }
      else
      {
        ZDT_Console_Print("[ZDT] position test failed.\r\n");
      }
    }
  }
  else if (strncmp(command, "run", 3U) == 0)
  {
    end = &command[3];
    while (*end == ' ')
    {
      end++;
    }
    if (*end == '\0')
    {
      ZDT_Console_Print("[ZDT] Usage: run <rpm>, range -3000 to +3000.\r\n");
      return;
    }
    rpm = strtol(end, &end, 10);
    while (*end == ' ')
    {
      end++;
    }
    if ((*end != '\0') || (rpm > ZDT_CAN_RUN_MAX_RPM) ||
        (rpm < -ZDT_CAN_RUN_MAX_RPM))
    {
      ZDT_Console_Print("[ZDT] Usage: run <rpm>, range -3000 to +3000.\r\n");
    }
    else
    {
      (void)ZDT_Motor_SetSpeedRpm((int16_t)rpm);
    }
  }
  else
  {
    ZDT_Console_Print("[ZDT] Unknown command. Type help for available commands.\r\n");
  }
}

static void ZDT_UART_RingPush(uint8_t byte)
{
  uint8_t next = (uint8_t)((zdt_uart_rx_head + 1U) % ZDT_UART_RX_RING_SIZE);

  zdt_status.uart_rx_bytes++;
  if (next != zdt_uart_rx_tail)
  {
    zdt_uart_rx_ring[zdt_uart_rx_head] = byte;
    __DMB();
    zdt_uart_rx_head = next;
  }
  else
  {
    /* 环形缓冲满：这个字节只能丢。计数，否则"命令缺字/卡住"无法归因。 */
    zdt_uart_rx_dropped++;
  }
}

static void ZDT_UART_RecordErrors(uint32_t isr)
{
  uint32_t errors = isr & (USART_ISR_ORE | USART_ISR_FE |
                           USART_ISR_NE | USART_ISR_PE);

  if (errors == 0U)
  {
    return;
  }
  if ((errors & USART_ISR_ORE) != 0U) { zdt_status.uart_rx_ore++; }
  if ((errors & USART_ISR_FE) != 0U) { zdt_status.uart_rx_fe++; }
  if ((errors & USART_ISR_NE) != 0U) { zdt_status.uart_rx_ne++; }
  if ((errors & USART_ISR_PE) != 0U) { zdt_status.uart_rx_pe++; }
  UART4->ICR = USART_ICR_ORECF | USART_ICR_FECF |
               USART_ICR_NECF | USART_ICR_PECF;
}

/* The IRQ is the normal, single RDR consumer. The bounded loop avoids an ISR
 * that can run forever if bytes keep arriving while it is active. */
void ZDT_Console_Uart4Irq(void)
{
  uint32_t budget = ZDT_UART_ISR_DRAIN_LIMIT;

  zdt_status.uart_rx_irq++;
  while (((UART4->ISR & USART_ISR_RXNE_RXFNE) != 0U) && (budget-- != 0U))
  {
    ZDT_UART_RingPush((uint8_t)(UART4->RDR & 0xFFU));
  }
  ZDT_UART_RecordErrors(UART4->ISR);
}

/* If HAL or a previous error path masked RXNEIE, briefly take ownership of
 * UART4 in the task, move pending bytes into the same ring, and restore both
 * receive and error interrupts. The parser never reads RDR directly. */
static void ZDT_UART_ServiceHardware(void)
{
  uint32_t isr = UART4->ISR;
  uint32_t cr1 = UART4->CR1;
  uint32_t cr3 = UART4->CR3;
  uint32_t error_mask = USART_ISR_ORE | USART_ISR_FE |
                        USART_ISR_NE | USART_ISR_PE;
  uint8_t rx_enabled = ((cr1 & USART_CR1_RXNEIE_RXFNEIE) != 0U) ? 1U : 0U;
  uint8_t error_enabled = ((cr3 & USART_CR3_EIE) != 0U) ? 1U : 0U;

  if ((rx_enabled != 0U) && (error_enabled != 0U) &&
      ((isr & (USART_ISR_RXNE_RXFNE | error_mask)) == 0U))
  {
    return;
  }

  /* NVIC masking is limited to this UART while the task becomes the producer;
   * other interrupts and the scheduler remain enabled. */
  HAL_NVIC_DisableIRQ(UART4_IRQn);
  __HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&huart4, UART_IT_ERR);
  HAL_NVIC_ClearPendingIRQ(UART4_IRQn);

  {
    uint32_t budget = ZDT_UART_TASK_DRAIN_LIMIT;
    while (((UART4->ISR & USART_ISR_RXNE_RXFNE) != 0U) &&
           (budget-- != 0U))
    {
      ZDT_UART_RingPush((uint8_t)(UART4->RDR & 0xFFU));
    }
  }
  ZDT_UART_RecordErrors(UART4->ISR);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
  __DMB();
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  if ((rx_enabled == 0U) || (error_enabled == 0U) ||
      ((isr & error_mask) != 0U))
  {
    zdt_status.uart_rx_recover++;
  }
}

static void ZDT_UART_ProcessByte(uint8_t byte)
{
  if (BT_Button_ProcessByte(byte) != 0U)
  {
    return;
  }

  if ((byte == '\n') && (zdt_uart_ignore_lf != 0U))
  {
    zdt_uart_ignore_lf = 0U;
    return;
  }
  if ((byte == '\r') || (byte == '\n'))
  {
    zdt_uart_ignore_lf = (byte == '\r') ? 1U : 0U;
    if (zdt_uart_command_overflow != 0U)
    {
      ZDT_Console_Print("\r\n[ZDT] ERROR: command is too long.\r\n");
    }
    else if (zdt_uart_command_length != 0U)
    {
      zdt_uart_command[zdt_uart_command_length] = '\0';
      ZDT_Console_Print("\r\n");
      ZDT_Process_Command(zdt_uart_command);
    }
    zdt_uart_command_length = 0U;
    zdt_uart_command_overflow = 0U;
    ZDT_Print_Prompt();
    return;
  }
  if ((byte == 0x08U) || (byte == 0x7FU))
  {
    if (zdt_uart_command_length != 0U)
    {
      zdt_uart_command_length--;
      ZDT_Console_Print("\b \b");
    }
    return;
  }
  if ((byte >= 0x20U) && (byte <= 0x7EU))
  {
    if (zdt_uart_command_length < ZDT_UART_COMMAND_MAX_LENGTH)
    {
      byte = (uint8_t)tolower((int)byte);
      zdt_uart_command[zdt_uart_command_length++] = (char)byte;
      /* 字符回显：**默认关闭**（2026-09-14）。
       *
       * 原来是 `HAL_UART_Transmit(&huart4, &byte, 1, 100)` 每字符同步发一次。
       * UART4 已改为115200；旧9600链路约1.04ms/字符，所以蓝牙 App 一次推一整行时，
       * 任务每收一个字符就阻塞 ~1 ms 去回显：15 字符的命令 = 15 ms 任务阻塞，
       * 期间 RX 环形缓冲（128 字节）被 ISR 填满就**静默丢字节** → 命令被腰斩、
       * 尾部字符变成下一条命令（现场表现："txlog" 和 "1" 分成两条），
       * 严重时命令永远拼不完整，看起来就像"卡死要 reset"。
       *
       * 人手工敲键盘时回显是有用的（能看见自己打了什么），所以保留运行期开关：
       * `echo 1` 打开 / `echo 0` 关闭。蓝牙 App 自己会显示发出的内容，不需要它。
       * 链接自检请用 `&echo`（那是应用层应答，和这里的字符回显无关）。 */
      if (zdt_uart_echo_enabled != 0U)
      {
        ZDT_Console_WriteBytes(&byte, 1U);
      }
    }
    else
    {
      zdt_uart_command_overflow = 1U;
    }
  }
}

static void ZDT_UART_ProcessInput(void)
{
  ZDT_UART_ServiceHardware();
  while (zdt_uart_rx_tail != zdt_uart_rx_head)
  {
    uint8_t byte = zdt_uart_rx_ring[zdt_uart_rx_tail];
    zdt_uart_rx_tail = (uint8_t)((zdt_uart_rx_tail + 1U) % ZDT_UART_RX_RING_SIZE);
    ZDT_UART_ProcessByte(byte);
  }
}

static start_button_t zdt_start_button;
static uint8_t zdt_start_debounce_pending;

void ZDT_Motor_Begin(void)
{
  uint8_t station;
  uint8_t start_released;uint32_t start_changed_tick;
  uint32_t reset_flags=RCC->RSR;
  char reset_line[128];
  const match_ports_t ports = {
    ZDT_Match_StartMotion, ZDT_Match_CancelMotion, ZDT_Match_Changed
  };
  const zone2_ports_t zone2_ports={ZDT_Zone2_StartAction,ZDT_Zone2_Cancel,ZDT_Zone2_Finished};
  Zone2Route_Init(&zdt_zone2,&zone2_ports);zdt_zone2_phase=Z2_NONE;
  MatchControl_Init(&zdt_match, &ports);
  GPIO_StartButtonExti_Reset(&start_released,&start_changed_tick);
  StartButton_Init(&zdt_start_button,start_released,start_changed_tick);zdt_start_debounce_pending=1U;
  ZDT_Console_Print("[START KEY] PD1 EXTI1 both edges, pull-up active LOW; debounce30ms in task; boot-held release/press; SHORT1/LONG2 on release,1s; READY only\r\n");
  zdt_match.first_round_only=0U;
  zdt_match.automatic=1U;zdt_match.qr_required=1U;
  zdt_qr_enabled=1U; /* ONE launch OK -> QR mission -> unattended two rounds. */
  VisionDiag_Init(&zdt_align_diag);zdt_diag_dump_active=zdt_diag_dump_interrupted=0U;
  for(station=0U;station<3U;station++) {
    vision_align_config_t *c=&zdt_align_config[station];
    (void)VisionAlign_StationConfig(c,(uint8_t)(station+1U));
    c->mode=zdt_align_smooth_enabled?V_ALIGN_MODE_SMOOTH:V_ALIGN_MODE_PULSE;
    c->search_axis=zdt_align_search_axis;
    ZDT_Align_SetTiming(c);
  }
  VisionAlign_Cancel(&zdt_align);VisionYaw_UseInstallationDefaults(&zdt_vyaw);zdt_align_phase=0U;memset(&zdt_camera_gate,0,sizeof(zdt_camera_gate));
  ZDT_Console_Print("[ALIGN] V197 XY: PLATE16px/3frames RING2px/5frames TEMP2stack10px/3frames; original speed/search/PID/timeouts and yaw retained\r\n");
  zdt_nav_endturn_enabled = 0U;
  zdt_match_io = MATCH_IO_NONE;
  zdt_match_request = 0U;
  (void)memset(&zdt_status, 0, sizeof(zdt_status));
  zdt_rx_head = 0U;
  zdt_rx_tail = 0U;
  zdt_query_attempts = 0U;
  zdt_uart_command_length = 0U;
  zdt_uart_command_overflow = 0U;
  zdt_uart_ignore_lf = 0U;
  zdt_uart_rx_head = 0U;
  zdt_uart_rx_tail = 0U;
  zdt_mileage_reference_mask = 0U;
  zdt_mileage_reference_announced = 0U;
  zdt_mileage_query_active = 0U;
  zdt_mileage_query_report = 0U;
  zdt_mileage_query_mask = 0U;
  zdt_mileage_query_attempts = 0U;
  zdt_mileage_query_retry_tick = 0U;
  zdt_mileage_stop_pending = 0U;
  zdt_mileage_stop_report = 0U;
  zdt_mileage_forward_pending = 0U;
  zdt_nav_state = ZDT_NAV_IDLE;
  zdt_nav_segment_count = 0U;
  zdt_nav_segment_index = 0U;
  zdt_nav_settle_valid = 0U;
  zdt_nav_settle_announced = 0U;
  zdt_nav_wait_start_query = 0U;
  zdt_nav_wait_end_query = 0U;
  zdt_nav_start_ready = 0U;
  zdt_nav_end_ready = 0U;
  zdt_nav_query_failed = 0U;
  BT_Button_Init();
  ChassisMotor_Init();
  GridNav_Init();
  /* 装上默认起点区（ZDT_START_ZONE_DEFAULT）的位姿与车头绑定。
     必须在 GridNav_Init() 之后：它会覆盖 grid_nav 里那个最底层的默认位姿。 */
  ZDT_Script_ApplyDefaultZone();
  /* 把"参考条件（spd=0.50 走 1000 mm）下的理论驱动时间（毫秒）"交给参数模块，
     用于日志里k的分母；不会覆盖出厂标定1930ms或Flash标定值。
     单位换算在函数里做（见其注释）。 */
  Nav_Params_SetReferencePlanMs(ZDT_Nav_ReferencePlanMs());
  /* 上电自动载入上次 `save` 的导航行进参数（没有有效记录就保持默认值）。 */
  if (Nav_Params_LoadFromFlash() != 0U)
  {
    ZDT_Console_Print("[NAV] params loaded from W25Q64\r\n");
    Nav_Params_Print();
  }
  /* V236：上电同样装载 `&save` 过的转向 PID（旧版本只有 &read 才装载，
   * 所以现场调完参数、保存后一断电重启就退回宏默认 0.045/0.0035）。 */
  (void)PID_Storage_ApplyBoot();
  if(zdt_nav_odometry_enabled) {
    Nav_Params()->position_mode=0U;
    ZDT_Console_Print("[NAV] ODOMETRY_V2_4: live four-wheel distance, continuous deceleration; t1000/dcomp ignored.\r\n");
  }
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

  ZDT_Console_Print("\r\n[ZDT] FDCAN2 controller: 500 kbit/s, UART4 115200 8N1\r\n");
  ZDT_Console_Print("[ZDT] UART4 RX: ordered ring + auto-recovery\r\n");
  ZDT_Console_Print("[BT] Buttons: &EN &M1 &M2 &U &L &D &R &STOP\r\n");
  ZDT_Console_Print("[VYAW] default ON at ROUGH2/TEMP3; XY+YAW simultaneous on trusted3-ring axis; nominal zero0deg gain+1; vyaw off/status, RAM overrides\r\n");
  ZDT_Console_Print("[MATCH] Serial debug flow: prepare -> READY -> OK 1/2; legacy startpose/depart available.\r\n");
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    ZDT_Console_Print("[ZDT] ERROR: FDCAN filter configuration failed\r\n");
    return;
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    ZDT_Console_Print("[ZDT] ERROR: FDCAN start failed\r\n");
    return;
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
        FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
        FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
        0U) != HAL_OK)
  {
    (void)HAL_FDCAN_Stop(&hfdcan2);
    ZDT_Console_Print("[ZDT] ERROR: FDCAN notification enable failed\r\n");
    return;
  }

  zdt_status.can_ready = 1U;
  ZDT_Console_Print("[ZDT] No motion command is sent at startup. Type help.\r\n");
  (void)snprintf(reset_line,sizeof(reset_line),"[BOOT] RSR=0x%08lX BOR=%u POR=%u PIN=%u SOFT=%u IWDG=%u WWDG=%u\r\n",
    (unsigned long)reset_flags,(unsigned int)((reset_flags&RCC_RSR_BORRSTF)!=0U),
    (unsigned int)((reset_flags&RCC_RSR_PORRSTF)!=0U),(unsigned int)((reset_flags&RCC_RSR_PINRSTF)!=0U),
    (unsigned int)((reset_flags&RCC_RSR_SFTRSTF)!=0U),(unsigned int)((reset_flags&RCC_RSR_IWDG1RSTF)!=0U),
    (unsigned int)((reset_flags&RCC_RSR_WWDG1RSTF)!=0U));
  ZDT_Console_Print(reset_line);__HAL_RCC_CLEAR_RESET_FLAGS();
  ZDT_CAN_SendVersionQuery();
  ZDT_Mileage_StartQuery(0U);
  ZDT_Print_Prompt();
}

static void ZDT_Match_ChassisService(void)
{
  HAL_StatusTypeDef result;char line[96];
  /* These phases deliberately keep wheels disabled. STOP dirty while OFF is
   * not a failed drive update; preparation owns its explicit stop/disable. */
  if(zdt_match.state==MATCH_PREPARING || zdt_match.state==MATCH_WAIT_START_OK)return;
  result=ChassisMotor_Update();
  if(result!=HAL_OK && MatchControl_IsActive(&zdt_match)) {
    (void)snprintf(line,sizeof(line),"[MATCH] chassis update HAL=%u state=%s enabled=%u before request=%lu\r\n",
      (unsigned int)result,MatchControl_StateName(zdt_match.state),ChassisMotor_IsEnabled(),(unsigned long)zdt_match_request);
    ZDT_Console_Print(line);
    ZDT_Match_Complete(0U,"chassis CAN update failed");
    if(MatchControl_IsActive(&zdt_match))MatchControl_Abort(&zdt_match);
  }
}

static uint8_t ZDT_Screen_CanApply(void)
{
  uint8_t i;int16_t rpm[4];
  if((zdt_match.state!=MATCH_IDLE && zdt_match.state!=MATCH_WAIT_START_OK) ||
     zdt_script_running || ZDT_Nav_IsActive() || ChassisMotor_IsTurnActive() ||
     ChassisMotor_HasMotionDemand() || ArmMaterial_IsActive() || ArmRound1_IsActive() || ArmStartPose_IsActive())return 0U;
  ChassisMotor_GetWheelRpm(rpm);for(i=0;i<4U;i++)if(rpm[i])return 0U;
  return 1U;
}
static void ZDT_Screen_Service(void)
{
  screen_batch_t batch;
  if(ScreenHmi_TakeBatch(&batch)) {
    uint8_t i,ok=0U;uint16_t x[SCREEN_MAX_OBSTACLES],y[SCREEN_MAX_OBSTACLES];
    if(ZDT_Screen_CanApply() && batch.count<=SCREEN_MAX_OBSTACLES) {
      for(i=0;i<batch.count;i++){x[i]=batch.points[i].x;y[i]=batch.points[i].y;}
      ok=GridNav_ReplaceObstaclesMm(x,y,batch.count);
      ZDT_Console_Print(ok?"[SCREEN] obstacle batch applied, old plan invalidated\r\n":"[SCREEN] obstacle batch rejected\r\n");
    }
    ScreenHmi_BatchResult(ok);
  }
}

static void ZDT_StartButton_Service(void)
{
  uint32_t pressed_tick,changed_tick,now,duration=0U;uint8_t released,zone;
  if(GPIO_StartButtonExti_Take(&released,&changed_tick)) {
    char line[128];
    (void)snprintf(line,sizeof(line),"[START KEY] EXTI1 level=%s edge_tick=%lums; debounce30ms\r\n",
        released?"HIGH/released":"LOW/pressed",(unsigned long)changed_tick);ZDT_Console_Print(line);
    StartButton_OnEdge(&zdt_start_button,released,changed_tick);zdt_start_debounce_pending=1U;
  }
  if(!zdt_start_debounce_pending)return;
  now=HAL_GetTick();
  if(now-zdt_start_button.changed_tick<START_BUTTON_DEBOUNCE_MS)return;
  zdt_start_debounce_pending=0U;
  zone=StartButton_GestureUpdate(&zdt_start_button,now,&pressed_tick,&duration);
  if(!zone) {
    ZDT_Console_Print(zdt_start_button.stable_released?
        "[START KEY] stable HIGH/released; next LOW press armed\r\n":
        "[START KEY] stable LOW/held; release to classify SHORT1/LONG2; no launch yet\r\n");return;
  }
  if(zdt_match.state!=MATCH_WAIT_START_OK) {
    ZDT_Console_Print("[START KEY] PD1 press ignored: wait launch READY, release then press again\r\n");return;
  }
  {char command[8],line[144];
   (void)snprintf(command,sizeof(command),"ok %u",zone);
   (void)snprintf(line,sizeof(line),"[START KEY] %s release duration=%lums zone=%u -> OK; original press edge must follow READY\r\n",
       zone==1U?"SHORT":"LONG",(unsigned long)duration,zone);ZDT_Console_Print(line);
   (void)ZDT_Match_Command(command,pressed_tick);}
  ZDT_Console_Print(zdt_match.state==MATCH_LEAVING?
      "[START KEY] launch accepted; MATCH LEAVING\r\n":
      "[START KEY] launch rejected by original OK checks; release before retry\r\n");
}

void ZDT_Motor_Poll(void)
{
  uint32_t pending;
  FDCAN_ProtocolStatusTypeDef protocol = {0};
  ZDT_CAN_RxFrameTypeDef frame;

  {
    const bt_button_stats_t *bt_stats = BT_Button_GetStats();
    uint32_t bt_before = bt_stats->valid_count;
    ZDT_UART_ProcessInput();
    BT_Button_Poll();
    if (bt_stats->valid_count != bt_before)
    {
      ZDT_Console_Print("OK!\r\n");
    }
  }
  ZDT_ProcessBluetoothButtons();
  ZDT_Vobs_Service();
  ZDT_StartButton_Service();
  ZDT_Script_Service();
  ZDT_Match_ChassisService();
  HWT101_PollReport();
  ZDT_Mileage_Service();
  ZDT_PosCal_Service();
  ZDT_TurnLog_Service();
  ZDT_Nav_Service();
  if (ChassisMotor_TakeTurnFinished() != 0U)
  {
    ZDT_Console_Print("FINISH!\r\n");
    if (ZDT_Nav_IsActive() == 0U && MatchControl_IsActive(&zdt_match) == 0U)
    {
      /* Manual-turn mileage only. Competition turns already own their next
       * baseline; a background query must not race an immediate WAIT_OK. */
      ZDT_Mileage_RequestStop(0U);
    }
  }
  for (pending = 0U; (pending < ZDT_CAN_RX_QUEUE_SIZE) &&
       (zdt_rx_tail != zdt_rx_head); ++pending)
  {
    uint8_t data_length;

    __DMB();
    frame = zdt_rx_queue[zdt_rx_tail];
    zdt_rx_tail = (zdt_rx_tail + 1U) % ZDT_CAN_RX_QUEUE_SIZE;
    ZDT_Print_Frame(&frame.header, frame.data);
    if (zdt_pos_dump_frames != 0U)
    {
      zdt_pos_dump_frames--;
    }
    data_length = ZDT_CAN_DataLengthBytes(frame.header.DataLength);
    if(frame.header.IdType==FDCAN_EXTENDED_ID && frame.header.RxFrameType==FDCAN_DATA_FRAME &&
        (frame.header.Identifier&0xFFU)==0U && (frame.header.Identifier>>8U)>=1U &&
        (frame.header.Identifier>>8U)<=6U)
      ZDTQuery_Frame(&zdt_user_query,(uint8_t)(frame.header.Identifier>>8U),frame.data,data_length,
                     frame.received_tick,frame.received_seq);
    ZDT_Mileage_HandleFrame(&frame.header, frame.data, data_length,frame.received_tick,frame.received_seq);
    if ((frame.header.IdType == FDCAN_EXTENDED_ID) &&
        (frame.header.RxFrameType == FDCAN_DATA_FRAME) &&
        (data_length >= 1U) &&
        (frame.data[0] == ZDT_CAN_CMD_READ_VERSION))
    {
      uint8_t version_address = (uint8_t)(frame.header.Identifier >> 8U);
      uint8_t version_index;

      if (version_address == ZDT_CAN_MOTOR_ADDRESS)
      {
        zdt_status.motor_responded = 1U;
        ZDT_Console_Print("[ZDT] PASS: address 3 returned a version-response frame.\r\n");
        ZDT_Print_Prompt();
      }

      /* 逐地址打印版本回包原文：用于确认四个驱动器是否同一固件/版本
         （地址 3、2 不执行 0xFD 时，版本差异是最常见的原因）。 */
      if (ZDT_Mileage_AddressIndex(version_address, &version_index) != 0U)
      {
        char version_line[112];
        char hex[40];
        uint8_t k;
        int length = 0;

        hex[0] = '\0';
        for (k = 0U; (k < data_length) && (k < 8U) &&
             ((size_t)(length + 3) < sizeof(hex)); ++k)
        {
          (void)snprintf(&hex[length], sizeof(hex) - (size_t)length,
                         "%02X ", frame.data[k]);
          length += 3;
        }
        (void)snprintf(version_line, sizeof(version_line),
                       "[ZDT] ver addr=%u idx=%u bytes=%s\r\n",
                       (unsigned int)version_address,
                       (unsigned int)version_index, hex);
        ZDT_Console_Print(version_line);
      }
    }
    ZDT_UART_ProcessInput();
  }

  if ((zdt_status.can_ready != 0U) &&
      (zdt_status.motor_responded == 0U) &&
      (zdt_query_attempts < ZDT_CAN_QUERY_MAX_ATTEMPTS) &&
      ((HAL_GetTick() - zdt_last_query_tick) >= ZDT_CAN_QUERY_INTERVAL_MS))
  {
    ZDT_CAN_SendVersionQuery();
  }

  zdt_status.last_can_error = HAL_FDCAN_GetError(&hfdcan2);
  if ((zdt_status.can_ready != 0U) &&
      (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol) == HAL_OK))
  {
    zdt_status.bus_off = (uint8_t)protocol.BusOff;
    if ((MatchControl_IsActive(&zdt_match) != 0U) &&
        ((protocol.BusOff != 0U) || (protocol.ErrorPassive != 0U)))
    {
      MatchControl_Abort(&zdt_match);
      ZDT_Console_Print("[MATCH] stopped: chassis CAN error state.\r\n");
    }
  }
  ZDT_ManualAlignService();
  ZDT_MaterialService();
  if(zdt_arm_start_token) {
    uint8_t arm_success=0U;
    if(ArmTask_GetStartPoseResult(zdt_arm_start_token,&arm_success)) {
      if(!arm_success)ZDT_Match_Complete(0U,"arm launch pose restore failed; inspect RAM angles/references with arm start status");
      else{zdt_arm_start_token=0U;ZDT_Match_Complete(1U,NULL);}
    }
  }
  if(zdt_arm_camera_token) {
    uint8_t arm_success=0U;
    if(ArmTask_GetCameraDeployResult(zdt_arm_camera_token,&arm_success)) {
      if(!arm_success) {
        ZDT_Match_Complete(0U,"arm camera deployment failed");
        /* Also cover the short handoff interval before the next route request
         * is dispatched: a failed restoration must never start new motion. */
        if(zdt_match.state==MATCH_MOVING && zdt_match.pending)MatchControl_Abort(&zdt_match);
      }
      else {
        zdt_arm_camera_token=0U;
        if(zdt_match_io==MATCH_IO_ARM_DEPLOY)ZDT_Match_Complete(1U,NULL);
      }
    } else if(HAL_GetTick()-zdt_arm_camera_started>=120000U) {
      ZDT_Match_Complete(0U,"arm camera deployment timeout");
      if(zdt_match.state==MATCH_MOVING && zdt_match.pending)MatchControl_Abort(&zdt_match);
    }
  }
  if(zdt_match_io==MATCH_IO_CAMERA_RESET && zdt_match_request) {
    if(!ZDT_Camera_ResetBusy())ZDT_Match_Complete(1U,NULL);
    else if(HAL_GetTick()-zdt_camera_reset_wait_tick>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS+1000U)
      ZDT_Match_Complete(0U,"camera STOP reset ACK timeout before recognition; inspect camstat");
  }
  ZDT_Screen_Service();
  {
    static uint32_t feedback_poll_tick;uint32_t now=HAL_GetTick();
    uint32_t gap=feedback_poll_tick?now-feedback_poll_tick:0U;feedback_poll_tick=now;
    /* A stationary route waiting for arm replies has not failed navigation.
     * Keep its request alive while the arm re-queries; moving drive watchdogs remain live. */
    if(ARM_ZDT_WAIT_ON_STALE && ArmCalibration_FeedbackWaiting() &&
       !ChassisMotor_HasMotionDemand() &&
       (zdt_arm_camera_token || zdt_arm_start_token || zdt_arm_material_token || zdt_arm_round_token))
        zdt_match.entered_tick+=gap;
  }
  MatchControl_Service(&zdt_match, HAL_GetTick());
  ZDTQuery_Service(&zdt_user_query,HAL_GetTick());
  ZDT_Zone2_Service();
  ZDT_Camera_Service();
  ZDT_Align_Service();
  ZDT_Align_ProgressService();
  ZDT_Camera_ResetService();
  ZDT_Qr_Service();
  ZDT_Align_DiagService();
  {
    int16_t rpm[CHASSIS_MOTOR_COUNT];uint8_t wheel;
    uint8_t busy=(uint8_t)(ZDT_Nav_IsActive() || zdt_script_running || ChassisMotor_IsTurnActive() ||
        zdt_match.state==MATCH_MOVING || zdt_match.state==MATCH_LEAVING);
    ChassisMotor_GetWheelRpm(rpm);
    for(wheel=0U;wheel<CHASSIS_MOTOR_COUNT;++wheel)if(rpm[wheel]!=0)busy=1U;
    Vision_DebugPoll(busy);
  }
}

const zdt_motor_status_t *ZDT_Motor_GetStatus(void)
{
  return &zdt_status;
}

void ZDT_Motor_RxFifo0MsgPending(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t interrupts)
{
  ZDT_CAN_RxFrameTypeDef frame = {0};
  uint32_t next;

  if (hfdcan != &hfdcan2)
  {
    return;
  }
  if ((interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    zdt_status.rx_dropped++;
  }
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &frame.header,
                               frame.data) != HAL_OK)
    {
      break;
    }
    if ((frame.header.FDFormat != FDCAN_CLASSIC_CAN) ||
        (frame.header.RxFrameType != FDCAN_DATA_FRAME))
    {
      continue;
    }
    if(frame.header.IdType==FDCAN_EXTENDED_ID &&
       ((frame.header.Identifier>>8U)==5U || (frame.header.Identifier>>8U)==6U)) {
      HAL_StatusTypeDef posted=ArmCalibration_PostCanFrame(&frame.header,frame.data,
          ZDT_CAN_DataLengthBytes(frame.header.DataLength));
      uint8_t mirror=(uint8_t)(zdt_user_query_address &&
          (frame.header.Identifier>>8U)==zdt_user_query_address && frame.data[0]==0x36U);
      if(posted==HAL_OK && !mirror) { zdt_status.rx_count++;continue; }
      if(posted==HAL_BUSY) { zdt_status.rx_dropped++;if(!mirror)continue; }
    }
    next = (zdt_rx_head + 1U) % ZDT_CAN_RX_QUEUE_SIZE;
    if (next == zdt_rx_tail)
    {
      zdt_status.rx_dropped++;
      continue;
    }
    frame.received_tick=HAL_GetTick();frame.received_seq=++zdt_rx_sequence;
    zdt_rx_queue[zdt_rx_head] = frame;
    __DMB();
    zdt_rx_head = next;
    zdt_status.rx_count++;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t interrupts)
{
  if (hfdcan->Instance == FDCAN2)
  {
    ZDT_Motor_RxFifo0MsgPending(hfdcan, interrupts);
  }
  else if (hfdcan->Instance == FDCAN1)
  {
    /* 达妙转盘反馈。2026-09-14 起驱动是句柄式的，实例从 arm_task 取。 */
    DM_RxFifo0Handler(ArmTask_GetMotor());
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN2)
  {
    /* Protocol errors use ErrorCallback in this HAL, not ErrorStatusCallback.
     * PSR reads consume LEC. Preserve informative errors before task polling
     * can replace them with NO_CHANGE. No logging or bus operations in ISR. */
    uint32_t lec = hfdcan->Instance->PSR & FDCAN_PSR_LEC;
    if ((lec > FDCAN_PROTOCOL_ERROR_NONE) &&
        (lec < FDCAN_PROTOCOL_ERROR_NO_CHANGE))
    {
      zdt_status.protocol_error_count++;
      zdt_status.last_protocol_lec = (uint8_t)lec;
    }
    zdt_status.last_can_error = HAL_FDCAN_GetError(hfdcan);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t interrupts)
{
  if (hfdcan->Instance == FDCAN2)
  {
    zdt_status.last_can_error = HAL_FDCAN_GetError(hfdcan);
  }
  else if (hfdcan->Instance == FDCAN1)
  {
    /* 2026-09-14：达妙总线的错误分支以前完全没有人处理 —— DM_Motor_Init 开了
     * BUS_OFF/ERROR_* 通知，但回调里只认 FDCAN2，于是总线断了固件看不见。
     * 这里把 bus-off 记到驱动里（中断上下文只置标志，绝不打印/阻塞）。
     * 打印由 ArmTask 在任务上下文按计数变化补一条，见 arm_task.c。 */
    DM_OnCanError(ArmTask_GetMotor(), interrupts);
  }
}

void ZDT_Motor_Task(void *argument)
{
  (void)argument;
  ZDT_Motor_Begin();
  for (;;)
  {
    ZDT_Motor_Poll();
    osDelay(1U);
  }
}
