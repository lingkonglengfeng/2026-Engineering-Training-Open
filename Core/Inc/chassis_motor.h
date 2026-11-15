/**
  ******************************************************************************
  * @file    chassis_motor.h
  * @brief   Four-wheel X-configuration mecanum chassis control.
  *
  * Coordinate convention:
  *   vx    > 0: forward
  *   vy    > 0: right
  *   omega > 0: counter-clockwise
  *
  * Button modes:
  *   CHASSIS_MODE_TURN:        U/D drive, L/R rotate in place
  *   CHASSIS_MODE_TRANSLATION: U/D drive, L/R strafe left/right
  *
  * Motor addresses and wheel direction signs are compile-time settings. After
  * installation, correct CHASSIS_WHEEL_SIGN_* if one wheel runs backwards.
  ******************************************************************************
  */

#ifndef __CHASSIS_MOTOR_H__
#define __CHASSIS_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "zdt_motor.h"

#define CHASSIS_MOTOR_COUNT              (4U)

/* Field trial: rotate the software body frame 180 degrees relative to the
 * first wiring table. Forward/right reverse together; physical CCW is kept.
 * All speed, field-ramp, mileage and position-command paths share this table. */
#define CHASSIS_FRONT_REVERSED_TRIAL      (1U)
#if CHASSIS_FRONT_REVERSED_TRIAL
#define CHASSIS_MOTOR_FL_ADDRESS         (4U)
#define CHASSIS_MOTOR_FR_ADDRESS         (2U)
#define CHASSIS_MOTOR_RL_ADDRESS         (3U)
#define CHASSIS_MOTOR_RR_ADDRESS         (1U)
#else
#define CHASSIS_MOTOR_FL_ADDRESS         (1U)
#define CHASSIS_MOTOR_FR_ADDRESS         (3U)
#define CHASSIS_MOTOR_RL_ADDRESS         (2U)
#define CHASSIS_MOTOR_RR_ADDRESS         (4U)
#endif

/* 满量程轮速（速度指令 1.0 对应的轮速）。
 * 2026-09-12 实测驱动器按 0.1RPM 解释速度字段（见 zdt_motor.c 的
 * ZDT_SPEED_FIELD_SCALE），发送侧已 ×10 修正；因此这里的"满量程"从 900 降到
 * 250 RPM（1.0 ≈ 1.06 m/s）。导航用 `spd` 在这个量程里取比例，
 * 手动按钮用 CHASSIS_MANUAL_SPEED_SCALE 再打折。 */
#define CHASSIS_MAX_WHEEL_RPM            (250.0f)

/* 麦轮"自转力臂"：原地自转时每个轮子要跑的**毫米/弧度**，= 半轴距 + 半轮距。
 * 车体 300×300、四轮在四角 → 150 + 150 = 300 mm。麦轮运动学：
 *     轮子线速度(mm/s) = vx ± vy ± CHASSIS_SPIN_ARM_MM × omega
 * 两边都用"轮子跑多少毫米"，与轮径无关，所以它是**转角/圆弧耗时的唯一几何量**：
 *   · 原地自转：轮速 = 300 × omega → 120 RPM(=511 mm/s) 就是 1.70 rad/s ≈ 97.6°/s，
 *     即现场实测的"90° 约 1 s、掉头约 2 s"（含斜坡）—— 这条式子和实测对得上；
 *   · 圆弧过弯：外轮 = v + 300 × v/R，所以 R 越小外轮越吃紧（`&tarc` 的下限就在这）。
 * 单位与 ZDT_MILEAGE_MM_PER_REV 一致，换算 RPM 直接用。 */
#define CHASSIS_SPIN_ARM_MM              (300.0f)

/* 手动控制（&U/&D 前进后退、&L/&R 平移或连续转）的速度折扣。
 * 2026-09-12 按要求降到原来的 1/3（避免手动一按就冲出去）。
 * 只影响手动按钮，不影响导航（导航速度由 `spd` 决定）。 */
#define CHASSIS_MANUAL_SPEED_SCALE       (0.3333f)
#define CHASSIS_ACCEL_RPM_PER_S          (600.0f)
#define CHASSIS_DECEL_RPM_PER_S          (292.0f)
/* In-place turns brake faster; translation keeps the existing deceleration. */
#define CHASSIS_TURN_DECEL_RPM_PER_S     (900.0f)
/* 下发节拍（= 控制环采样周期，也是**执行粒度的下限**）。
 * 2026-09-12 由 20 ms 改为 4 ms，原因见 CHASSIS_TURN_MIN_EFFECTIVE_OUTPUT：
 * 转向末端靠"摩擦补偿 kick"一点点挪，kick 对应的物理角速度很小（0.08 × 档位轮速上限；
 * V235 把精确档上限 90→180 RPM 后，**同一输出对应的实际角速度也翻倍**），
 *   - 20 ms 一拍 = 0.25°，是 0.05° 判据带宽的 5 倍 → 一冲就过冲、来回蹭、拖到超时；
 *   - 4 ms 一拍 ≈ 0.05°，正好等于判据带宽 → 末端能在带内落定（落点窗约 ±0.025°）。
 * 顺带：所有到达判据都改成**按毫秒计时**（原来按"拍数"数，改节拍就会跟着变），
 * 加减速斜坡按"实际经过的毫秒数"算，标称斜率为 600/292 RPM/s，导航的梯形规划
 * 模型不受影响（4 ms 一步的浮点增量约为 2.4/1.168 RPM，应用值取整后约为 2/1 RPM；
 * 因此从约 88 RPM 减到 0 的软件停稳时间约为 88 × 4 ms = 352 ms）。
 * CAN 负载：4 轮 × 250 Hz = 1000 帧/s，FDCAN2 是 500 kbps 经典帧（约 26% 占用）。
 * 若现场出现丢帧/抖动，先退到 5 ms（粒度 0.0625°，略超带宽）。 */
#define CHASSIS_TX_PERIOD_MS             (4U)
#define CHASSIS_USE_SYNC_TRIGGER         (0U)

/* PID heading control.
   CHASSIS_YAW_RIGHT_POSITIVE_SIGN 作用在 HWT101 的"原始零漂相对角"上：
   传感器倒装，原始读数是右转（顺时针）为正，乘 -1 之后固件内部
   heading_deg 才是左转（俯视逆时针）为正，与 omega>0 同向。 */
#define CHASSIS_YAW_RIGHT_POSITIVE_SIGN  (-1.0f)
/* 转向 PID 默认增益（2026-09-12 现场实测稳定值）。
 * 注意：flash 里若已存过 PID（`&save`），`&EN` 会用 flash 的值覆盖这里的默认值；
 * 现场改增益用 `&kp/&ki/&kd` 再 `&save`。 */
#define CHASSIS_TURN_KP                  (0.045f)
#define CHASSIS_TURN_KI                  (0.0005f)
#define CHASSIS_TURN_KD                  (0.0035f)
#define CHASSIS_TURN_MAX_OUTPUT          (1.00f)
/* 原地转（TurnRelative：`&L90`/`&R90`、导航段前对准）的轮速上限。
 * 这是"转向 PID + 上限"共同决定的角度转速度：同一 PID 输出 × 上限 = 实际轮速，
 * 所以上限翻倍会让环"变热"（同样的 kp/kd 对应的实际轮速翻倍、更易过冲）。
 * V235：按现场要求把精确档上限改为 **2×（90→180 RPM）**。
 * V237：按现场要求把**快速档（导航段前对准）也改成与精确档相同（180 RPM）**，
 *       两档不再有速度差；判据（0.5°/200 ms）与 PID 增益仍各自保持不变。 */
#define CHASSIS_TURN_MAX_WHEEL_RPM       (180.0f)
#define CHASSIS_TURN_INTEGRAL_LIMIT      (5.0f)

/* ---------- 末端"摩擦补偿 kick"与死区（2026-09-12 重做）----------
 * 齿轮减速电机的静摩擦死区：PID 输出小到一定程度轮子压根不转。
 * 0.08 × 90 RPM = 7.2 RPM 就是现场标出来的"刚好能推动"的最小指令。
 *
 * 旧参数有个**无人区**：kick 只在 |误差| > 0.20° 时给，判到位却要求 |误差| ≤ 0.05°
 * → 误差落在 0.05°~0.20° 时 PID 只给 0.045×0.15 ≈ 0.007（≈0.6 RPM，推不动）
 * → 车卡在那儿不动，一直等到 8 s 超时。这是"转圈大概率进 timeout 区"的主因。
 *
 * 现在：kick 门槛压到判据带边缘（每档的 kick_deg 略高于该档的 band），
 * 并把 kick 的**角速度门限去掉** —— kick 的方向永远跟着误差符号，不会和运动对抗，
 * 门限留着反而让末端退化成"每 ~100 ms 猛推一次 0.25°"的粗 bang-bang。
 * 配合 CHASSIS_TX_PERIOD_MS = 4 ms，末端一拍只走 0.0625°（< 判据带宽的 2 倍），
 * 能在带内做细 bang-bang，一次就能落进判据带并停住。 */
#define CHASSIS_TURN_MIN_EFFECTIVE_OUTPUT (0.08f)
/* 摩擦补偿（kick）的**运动门控**门槛：只有当"朝目标的闭合角速度"≤ 它时才补推力。
 * 这条是 2026-09-12 修"末端一直抖"的关键：
 *   旧写法只要 |PID 输出| < kick 就无条件把输出顶成 ±kick，于是车正在滑向目标、
 *   D 项正在**刹车**（输出为负）时也会被顶成正向推力 → 必然冲过目标 → 反向再来一次
 *   → 极限环。ki=0 时 D 项是唯一阻尼，所以现场表现为"越调 ki 越抖、调成 0 也抖"。
 * 现在：正在朝目标动（closing 大）就完全交给 PID（含刹车），
 *       只有"该动却没动"（closing 很小）才补推力。 */
#define CHASSIS_TURN_KICK_CLOSE_DPS      (0.30f)
/* 旧的两个常量已由档位表取代（见 chassis_turn_profile_t 的 kick_deg 字段）：
   CHASSIS_TURN_MIN_OUTPUT_ERROR_DEG(0.20°) 门槛 → 每档 kick_deg；
   CHASSIS_TURN_MIN_OUTPUT_RATE_DPS(2 dps) 角速度门限 → 已删除。 */

/* ---------- 转向档位（2026-09-12：为了比赛总时间）----------
 *  精确档（`&L90`/`&R90` 等手动角度转）：现场指定"误差 ≤0.05° 稳定 0.5 s 才停"。
 *    另外加一条**等价但更快**的收工路径：带内且 |角速度| ≤ 0.3 dps 时只需 200 ms
 *    （"停住"这件事由角速度门限证明，不必再干等 0.5 s）。两条谁先满足算谁。
 *  快速档（导航每段前的原地对准）：对准之后还有**直走航向保持**每 80 ms 纠一次，
 *    所以没必要在转弯处抠到 0.05° —— 允许 0.5° 收工，驻留缩到 200 ms。
 *    V237 起轮速上限与精确档相同（180 RPM），不再单独设 120；
 *    kick 门槛仍按档位表（0.10°）放宽。
 *
 *  注意：轮速上限一提，同样的 kp 对应的实际轮速就变大（环变"热"）。快速档抖的话
 *  先降 `kd`（现 0.0035）再降 `kp`，或把 `CHASSIS_TURN_FAST_WHEEL_RPM` 单独退到 120。 */
/* V237：快速档轮速上限 = 精确档（现场要求两档一样，避免比赛流程转向仍被 120 卡住）。*/
#define CHASSIS_TURN_FAST_WHEEL_RPM      (CHASSIS_TURN_MAX_WHEEL_RPM)
#define CHASSIS_TURN_FAST_MIN_PUSH       (0.08f)

typedef enum
{
  CHASSIS_TURN_PROFILE_PRECISE = 0,
  CHASSIS_TURN_PROFILE_FAST = 1
} chassis_turn_profile_id_t;

typedef struct
{
  float max_wheel_rpm;      /* 轮速上限 */
  float kick_deg;           /* 误差大于它才考虑摩擦补偿（应**远小于** band_deg 的量级：
                               它决定"推到多近才停手"，设大了会留下明显残余误差） */
  float min_push;           /* 摩擦补偿推力（归一化输出；× max_wheel_rpm = 实际轮速） */
  float strict_deg;         /* 主判据带 */
  uint16_t strict_ms;       /* 主判据连续时长 */
  float band_deg;           /* 稳定带 */
  uint16_t band_ms;         /* 稳定带驻留（不卡角速度） */
  float band_rate_dps;      /* 带内且角速度 ≤ 它 → 只需 band_fast_ms */
  uint16_t band_fast_ms;
  float stall_band_deg;     /* 无进展兜底的误差上限 */
  uint16_t stall_ms;        /* 无进展判定时长 */
} chassis_turn_profile_t;

#define CHASSIS_TURN_TIMEOUT_MS          (8000U)
#define CHASSIS_TURN_STALL_PROGRESS_DEG  (0.01f)

/* Change a sign to -1.0f if that physical wheel runs in the wrong direction. */
#define CHASSIS_WHEEL_SIGN_FL            (-1.0f)
#define CHASSIS_WHEEL_SIGN_FR            (1.0f)
#define CHASSIS_WHEEL_SIGN_RL            (-1.0f)
#define CHASSIS_WHEEL_SIGN_RR            (1.0f)

typedef struct
{
  float heading_deg;
  float target_heading_deg;
  float gyro_dps;
  float vx;
  float vy;
  float omega;
  float pid_error_deg;
  float pid_p_term;
  float pid_i_term;
  float pid_d_term;
  float pid_output;
  uint32_t turn_elapsed_ms;
  uint8_t turn_active;
  uint8_t turn_fallback; /* 1 = 靠稳定带判到位（带内驻留够久） */
  uint8_t turn_stall;    /* 1 = 靠"无进展兜底"收工（已到机械/分辨率下限） */
  uint8_t turn_profile;  /* 本次转向用的档位（chassis_turn_profile_id_t） */
  float turn_final_error_deg; /* 判到位瞬间的残余误差（转结束后仍可读） */
} chassis_telemetry_t;
typedef enum
{
  CHASSIS_MODE_TURN = 0,
  CHASSIS_MODE_TRANSLATION = 1
} chassis_mode_t;

typedef enum
{
  CHASSIS_COMMAND_STOP = 0,
  CHASSIS_COMMAND_FORWARD,
  CHASSIS_COMMAND_BACKWARD,
  CHASSIS_COMMAND_LEFT,
  CHASSIS_COMMAND_RIGHT,
  CHASSIS_COMMAND_MANUAL
} chassis_command_t;

typedef enum
{
  CHASSIS_WHEEL_FL = 0,
  CHASSIS_WHEEL_FR,
  CHASSIS_WHEEL_RL,
  CHASSIS_WHEEL_RR
} chassis_wheel_t;

void ChassisMotor_Init(void);
/* Owner task only, stopped wheels: bind a visually verified direction to
 * its map-relative heading without restarting IMU drift calibration. */
HAL_StatusTypeDef ChassisMotor_RebaseHeading(float heading_deg);
/* Call together with a new IMU zero / start placement. RAM only. */
void ChassisMotor_ClearHeadingReference(void);
HAL_StatusTypeDef ChassisMotor_SetMode(chassis_mode_t mode);
HAL_StatusTypeDef ChassisMotor_SetCommand(chassis_command_t command);
HAL_StatusTypeDef ChassisMotor_SetVelocity(float vx, float vy, float omega);
HAL_StatusTypeDef ChassisMotor_SetFieldVelocity(float initial_x,float initial_y,
                                               float map_vx,float map_vy,float omega);
HAL_StatusTypeDef ChassisMotor_Update(void);
HAL_StatusTypeDef ChassisMotor_Enable(void);
HAL_StatusTypeDef ChassisMotor_Disable(void);
/* Owner-task preparation: disable wheels without emergency-stopping DM. */
HAL_StatusTypeDef ChassisMotor_DisableForArmPreparation(void);
HAL_StatusTypeDef ChassisMotor_Stop(void);
/* Send chassis-only stop immediately, before blocking diagnostic output. */
HAL_StatusTypeDef ChassisMotor_StopImmediate(void);
HAL_StatusTypeDef ChassisMotor_EmergencyStop(void);
HAL_StatusTypeDef ChassisMotor_TurnRelative(int16_t angle_deg);
/* 快速档原地转（导航每段前的对准用它）：判据放宽到 0.5°、驻留 200 ms、
 * 轮速上限与精确档相同（V237：180 RPM），后续直走的航向保持会把残余偏差纠回来。 */
HAL_StatusTypeDef ChassisMotor_TurnRelativeFast(int16_t angle_deg);
uint8_t ChassisMotor_IsTurnActive(void);
/* Read-only demand check, including field targets before the first ramp tick. */
uint8_t ChassisMotor_HasMotionDemand(void);
/* Restart a translation ramp only when targets and applied wheels are zero. */
void ChassisMotor_ResetIdleRamp(void);
uint8_t ChassisMotor_IsEnabled(void);
uint8_t ChassisMotor_TakeTurnFinished(void);
void ChassisMotor_GetWheelRpm(int16_t rpm[CHASSIS_MOTOR_COUNT]);
/* Pure wheel ramp prediction, shared by the update loop and offline verification. */
int16_t ChassisMotor_RampStepRpm(int16_t current_rpm, int16_t target_rpm,
                                float dt_s, float decel_rpm_s);
void ChassisMotor_GetTelemetry(chassis_telemetry_t *telemetry);
/* 本次转向的绝对目标（内部连续绕组角，左转为正）。转向进行中读数最准；
 * 未转向时保留上一次的目标，供上位机波形/状态显示。 */
float ChassisMotor_GetTurnTargetDeg(void);
/* 手动精转（`&L*`/`&R*`）的轮速上限（RPM）。上位机“转向PID”页用它确认速度限幅
 * 是否已生效（V235：精确档 180；V237 起导航快速档也同为 180）。 */
float ChassisMotor_GetTurnMaxWheelRpm(void);
void ChassisMotor_SetTurnGains(float kp, float ki, float kd);
void ChassisMotor_GetTurnGains(float *kp, float *ki, float *kd);
chassis_mode_t ChassisMotor_GetMode(void);
chassis_command_t ChassisMotor_GetCommand(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_MOTOR_H__ */
