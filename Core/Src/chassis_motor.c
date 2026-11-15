#include "chassis_motor.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

#include "arm_task.h"
#include "chassis_field.h"

static const uint8_t chassis_addresses[CHASSIS_MOTOR_COUNT] =
{
  CHASSIS_MOTOR_FL_ADDRESS,
  CHASSIS_MOTOR_FR_ADDRESS,
  CHASSIS_MOTOR_RL_ADDRESS,
  CHASSIS_MOTOR_RR_ADDRESS
};

static const float chassis_wheel_signs[CHASSIS_MOTOR_COUNT] =
{
  CHASSIS_WHEEL_SIGN_FL,
  CHASSIS_WHEEL_SIGN_FR,
  CHASSIS_WHEEL_SIGN_RL,
  CHASSIS_WHEEL_SIGN_RR
};

static chassis_mode_t chassis_mode;
static chassis_command_t chassis_command;
/* Retain the selected ramp through stopping, until the next motion command. */
static float chassis_decel_rpm_per_s;
static uint8_t chassis_field_active;
static float chassis_field_x,chassis_field_y,chassis_field_o,chassis_field_fx,chassis_field_fy;
static float chassis_field_yaw;
static float chassis_heading_offset;
static chassis_field_profile_t chassis_field_profile;
static int16_t chassis_target_rpm[CHASSIS_MOTOR_COUNT];
static int16_t chassis_applied_rpm[CHASSIS_MOTOR_COUNT];
static uint8_t chassis_output_dirty;
static uint8_t chassis_motors_enabled;
static uint8_t chassis_profile_started;
static uint32_t chassis_last_update_tick;
static uint8_t chassis_turn_active;
static float chassis_turn_target_deg;
static float chassis_turn_integral;
static float chassis_turn_last_error;
static uint32_t chassis_turn_start_tick;
/* 1 = 本次转向靠"稳定带"(0.05° 保持 0.5 s) 判到位，而不是 0.01° 主判据 */
static uint8_t chassis_turn_fallback_used;
/* 主判据（0.01° + 低角速度）的连续计时 */
static uint8_t chassis_turn_strict_active;
static uint32_t chassis_turn_strict_tick;
/* 稳定带驻留计时：进带时记 tick，掉出带就清标志 */
static uint8_t chassis_turn_band_active;
static uint32_t chassis_turn_band_tick;
/* 无进展兜底：本段见过的最小 |误差| 及它出现的时刻 */
static float chassis_turn_best_error_deg;
static uint32_t chassis_turn_best_tick;
/* 1 = 本次转向靠"无进展兜底"收工（已到机械/分辨率下限），体检行标 ST */
static uint8_t chassis_turn_stall_used;
/* 本次转向用的档位（精确档 = 手动 &L90；快速档 = 导航段前对准），
   在 chassis_turn_profile_precise/fast 定义之后初始化 */

/* ---------- 两个档位（数值都在这里，现场改这一处即可）----------
 * 精确档：现场指定"误差 ≤0.05° 连续稳定 0.5 s 才停"；另外给一条等价但更快的
 *         收工路径（带内且角速度 ≤0.3 dps 只需 200 ms，由角速度证明"停住了"）。
 * 快速档：给导航每段前的对准用。对准之后直走有航向保持（每 80 ms 纠一次），
 *         所以允许 0.5° 收工、驻留 200 ms；V237 起轮速上限与精确档相同（180 RPM）。 */
const chassis_turn_profile_t chassis_turn_profile_precise =
{
  CHASSIS_TURN_MAX_WHEEL_RPM,   /* max_wheel_rpm  */
  0.06f,                        /* kick_deg       */
  0.05f,                        /* min_push       */
  0.01f,                        /* strict_deg     */
  200U,                         /* strict_ms      */
  0.05f,                        /* band_deg       */
  500U,                         /* band_ms        */
  0.30f,                        /* band_rate_dps  */
  200U,                         /* band_fast_ms   */
  0.20f,                        /* stall_band_deg */
  900U                          /* stall_ms       */
};

const chassis_turn_profile_t chassis_turn_profile_fast =
{
  CHASSIS_TURN_FAST_WHEEL_RPM,  /* max_wheel_rpm  */
  0.10f,                        /* kick_deg：0.60 → 0.10，见下 */
  CHASSIS_TURN_FAST_MIN_PUSH,    /* min_push       */
  0.15f,                        /* strict_deg     */
  60U,                          /* strict_ms      */
  0.50f,                        /* band_deg       */
  200U,                         /* band_ms        */
  0.80f,                        /* band_rate_dps  */
  100U,                         /* band_fast_ms   */
  1.00f,                        /* stall_band_deg */
  400U                          /* stall_ms       */
};

/* 当前档位指针：默认精确档，每次起转由 ChassisMotor_StartTurn() 指定。 */
static const chassis_turn_profile_t *chassis_turn_profile =
    &chassis_turn_profile_precise;
/* 判到位那一刻的残余误差（chassis_pid_error_deg 结束时会被清零，留一份给体检打印） */
static float chassis_turn_final_error_deg;
static uint8_t chassis_turn_finished;
static float chassis_vx_cmd;
static float chassis_vy_cmd;
static float chassis_omega_cmd;
static float chassis_pid_error_deg;
static float chassis_pid_target_deg;
static float chassis_pid_p_term;
static float chassis_pid_i_term;
static float chassis_pid_d_term;
static float chassis_pid_output;
static float chassis_turn_kp = CHASSIS_TURN_KP;
static float chassis_turn_ki = CHASSIS_TURN_KI;
static float chassis_turn_kd = CHASSIS_TURN_KD;

static float ChassisMotor_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ChassisMotor_Limit(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static int16_t ChassisMotor_ToRpm(float value)
{
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t ChassisMotor_AllZero(const int16_t *values)
{
  uint32_t i;

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    if (values[i] != 0)
    {
      return 0U;
    }
  }
  return 1U;
}

static void ChassisMotor_Mix(float vx, float vy, float omega)
{
  float wheel[CHASSIS_MOTOR_COUNT];
  float maximum = 0.0f;
  uint32_t i;

  chassis_vx_cmd = vx;
  chassis_vy_cmd = vy;
  chassis_omega_cmd = omega;

  /* X 型（对角线同型号）四轮麦轮底盘的速度分解：
       FL 与 RR 同型，FR 与 RL 同型；辊子被动空转方向沿各自对角线。
     正负号由 CHASSIS_WHEEL_SIGN_* 统一补偿到电机命令。
     约定：vx>0 前进、vy>0 车体向右、omega>0 俯视逆时针。
     排布与实测判定见 docs/hardware/底盘方向与麦轮排布说明.md */
  wheel[CHASSIS_WHEEL_FL] = vx + vy - omega;
  wheel[CHASSIS_WHEEL_FR] = vx - vy + omega;
  wheel[CHASSIS_WHEEL_RL] = vx - vy - omega;
  wheel[CHASSIS_WHEEL_RR] = vx + vy + omega;

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    float magnitude = ChassisMotor_Abs(wheel[i]);
    if (magnitude > maximum)
    {
      maximum = magnitude;
    }
  }

  if (maximum > 1.0f)
  {
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
    {
      wheel[i] /= maximum;
    }
  }

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    float rpm = wheel[i] * CHASSIS_MAX_WHEEL_RPM * chassis_wheel_signs[i];
    chassis_target_rpm[i] = ChassisMotor_ToRpm(rpm);
  }
}

static void ChassisMotor_MixTurn(float omega, float max_wheel_rpm)
{
  float wheel[CHASSIS_MOTOR_COUNT];

  chassis_vx_cmd = 0.0f;
  chassis_vy_cmd = 0.0f;
  chassis_omega_cmd = omega;

  wheel[CHASSIS_WHEEL_FL] = -omega;
  wheel[CHASSIS_WHEEL_FR] = omega;
  wheel[CHASSIS_WHEEL_RL] = -omega;
  wheel[CHASSIS_WHEEL_RR] = omega;

  chassis_target_rpm[CHASSIS_WHEEL_FL] = ChassisMotor_ToRpm(
      wheel[CHASSIS_WHEEL_FL] * max_wheel_rpm * CHASSIS_WHEEL_SIGN_FL);
  chassis_target_rpm[CHASSIS_WHEEL_FR] = ChassisMotor_ToRpm(
      wheel[CHASSIS_WHEEL_FR] * max_wheel_rpm * CHASSIS_WHEEL_SIGN_FR);
  chassis_target_rpm[CHASSIS_WHEEL_RL] = ChassisMotor_ToRpm(
      wheel[CHASSIS_WHEEL_RL] * max_wheel_rpm * CHASSIS_WHEEL_SIGN_RL);
  chassis_target_rpm[CHASSIS_WHEEL_RR] = ChassisMotor_ToRpm(
      wheel[CHASSIS_WHEEL_RR] * max_wheel_rpm * CHASSIS_WHEEL_SIGN_RR);
}

int16_t ChassisMotor_RampStepRpm(int16_t current_rpm,
                                       int16_t target_rpm,
                                       float delta_time_s, float decel_rpm_s)
{
  float current = (float)current_rpm;
  float target = (float)target_rpm;
  float rate;
  float delta;
  uint8_t same_direction;

  if (current == target)
  {
    return target_rpm;
  }

  same_direction = (uint8_t)(((current >= 0.0f) && (target >= 0.0f)) ||
                             ((current <= 0.0f) && (target <= 0.0f)));

  /* Direction reversal: finish decelerating to zero first, then accelerate. */
  if (((current > 0.0f) && (target < 0.0f)) ||
      ((current < 0.0f) && (target > 0.0f)))
  {
    delta = decel_rpm_s * delta_time_s;
    if (delta < 1.0f)
    {
      delta = 1.0f;
    }
    if (current > 0.0f)
    {
      current = (current > delta) ? (current - delta) : 0.0f;
    }
    else
    {
      current = ((-current) > delta) ? (current + delta) : 0.0f;
    }
    return ChassisMotor_ToRpm(current);
  }

  if (current == 0.0f)
  {
    rate = CHASSIS_ACCEL_RPM_PER_S;
  }
  else if ((same_direction != 0U) &&
           (ChassisMotor_Abs(target) > ChassisMotor_Abs(current)))
  {
    rate = CHASSIS_ACCEL_RPM_PER_S;
  }
  else
  {
    rate = decel_rpm_s;
  }

  delta = rate * delta_time_s;
  if (delta < 1.0f)
  {
    delta = 1.0f;
  }

  if (current < target)
  {
    current += delta;
    if (current > target)
    {
      current = target;
    }
  }
  else
  {
    current -= delta;
    if (current < target)
    {
      current = target;
    }
  }

  return ChassisMotor_ToRpm(current);
}

static HAL_StatusTypeDef ChassisMotor_SendTargets(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint32_t i;

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = ZDT_Motor_SetSpeedRpmAddr(
        chassis_addresses[i], chassis_applied_rpm[i], CHASSIS_USE_SYNC_TRIGGER);
    if (status != HAL_OK)
    {
      result = status;
    }
  }

  if ((result == HAL_OK) && (CHASSIS_USE_SYNC_TRIGGER != 0U))
  {
    result = ZDT_Motor_SyncTrigger();
  }

  return result;
}

static float ChassisMotor_GetTurnYawDeg(void)
{
  return HWT101_GetZeroDriftDeg() * CHASSIS_YAW_RIGHT_POSITIVE_SIGN + chassis_heading_offset;
}

void ChassisMotor_ClearHeadingReference(void)
{
  chassis_heading_offset=0.0f;
}

HAL_StatusTypeDef ChassisMotor_RebaseHeading(float heading_deg)
{
  float current=ChassisMotor_GetTurnYawDeg();
  if(!isfinite(heading_deg) || !isfinite(current) || !HWT101_IsZeroDriftReady())return HAL_ERROR;
  if(ChassisMotor_HasMotionDemand() || !ChassisMotor_AllZero(chassis_applied_rpm))return HAL_BUSY;
  /* Keep the current winding; a -448deg reading bound to -90 becomes -450,
   * rather than introducing a full revolution into subsequent PID targets. */
  chassis_heading_offset+=remainderf(heading_deg-current,360.0f);
  chassis_field_yaw=ChassisMotor_GetTurnYawDeg();
  chassis_pid_target_deg=chassis_field_yaw;
  return HAL_OK;
}

static void ChassisMotor_CancelTurn(void)
{
  chassis_turn_active = 0U;
  chassis_turn_target_deg = 0.0f;
  chassis_turn_integral = 0.0f;
  chassis_turn_last_error = 0.0f;
  chassis_turn_band_active = 0U;
  chassis_turn_strict_active = 0U;
  chassis_turn_best_error_deg = 180.0f;
  chassis_turn_best_tick = HAL_GetTick();
}

static uint8_t ChassisMotor_UpdateTurn(float delta_time_s)
{
  float current_yaw = ChassisMotor_GetTurnYawDeg();
  float current_rate = HWT101_GetGyroZDps() * CHASSIS_YAW_RIGHT_POSITIVE_SIGN;
  float error = chassis_turn_target_deg - current_yaw;
  float derivative;
  float p_term;
  float i_term;
  float d_term;
  float output;
  uint8_t reached = 0U;
  uint8_t timed_out = 0U;

  if ((HAL_GetTick() - chassis_turn_start_tick) >= CHASSIS_TURN_TIMEOUT_MS)
  {
    timed_out = 1U;
  }

  {
    float magnitude = ChassisMotor_Abs(error);
    float rate_magnitude = ChassisMotor_Abs(current_rate);
    uint8_t strict_ok = 0U;
    uint8_t band_ok = 0U;
    uint8_t stall_ok = 0U;

    /* 主判据：|误差| ≤ 档位 strict_deg 且角速度达标，连续保持 strict_ms。
       精确档是 0.01°（≈2 个传感器 LSB）+ 2 dps + 200 ms；快速档是 0.15° + 60 ms。 */
    if ((magnitude <= chassis_turn_profile->strict_deg) &&
        (rate_magnitude <= 2.0f))
    {
      strict_ok = 1U;
    }

    /* 稳定带判据：|误差| ≤ 档位 band_deg。
       ① 带内且 |角速度| ≤ band_rate_dps → 只需 band_fast_ms 就收工
          （"停住"由角速度门限证明，不必干等）；
       ② 否则按 band_ms 驻留（精确档 0.5 s，掉出带即重新计时）——
          0.5 s 驻留本身就等价于"平均角速度 < 0.1 dps"，所以不需要再卡角速度，
          也天然防止"擦着目标飞过去"被误判到位。 */
    if (magnitude <= chassis_turn_profile->band_deg)
    {
      if (chassis_turn_band_active == 0U)
      {
        chassis_turn_band_active = 1U;
        chassis_turn_band_tick = HAL_GetTick();
      }
      else if (((HAL_GetTick() - chassis_turn_band_tick) >=
                chassis_turn_profile->band_ms) ||
               ((rate_magnitude <= chassis_turn_profile->band_rate_dps) &&
                ((HAL_GetTick() - chassis_turn_band_tick) >=
                 chassis_turn_profile->band_fast_ms)))
      {
        band_ok = 1U;
      }
    }
    else
    {
      /* 掉出稳定带：驻留计时重来 */
      chassis_turn_band_active = 0U;
    }

    /* 无进展兜底：记录本段达到过的最小 |误差|，若已经进到 stall_band_deg 以内、
       但 stall_ms 内再也压不下去（改善不足 0.01°），说明到了机械/静摩擦/分辨率的
       下限 → 收工（标 ST），不要磨到 8 s 超时。 */
    if (magnitude < (chassis_turn_best_error_deg -
                     CHASSIS_TURN_STALL_PROGRESS_DEG))
    {
      chassis_turn_best_error_deg = magnitude;
      chassis_turn_best_tick = HAL_GetTick();
    }
    if ((magnitude <= chassis_turn_profile->stall_band_deg) &&
        ((HAL_GetTick() - chassis_turn_best_tick) >=
         chassis_turn_profile->stall_ms))
    {
      stall_ok = 1U;
    }

    if (strict_ok != 0U)
    {
      if (chassis_turn_strict_active == 0U)
      {
        chassis_turn_strict_active = 1U;
        chassis_turn_strict_tick = HAL_GetTick();
      }
      else if ((HAL_GetTick() - chassis_turn_strict_tick) >=
               chassis_turn_profile->strict_ms)
      {
        reached = 1U;
      }
    }
    else if (band_ok != 0U)
    {
      /* 靠稳定带判到位：体检行会标 FB（说明已在分辨率/机械下限附近） */
      reached = 1U;
      chassis_turn_fallback_used = 1U;
    }
    else if (stall_ok != 0U)
    {
      /* 无进展兜底：压不下去了 → 收工，体检行标 ST */
      reached = 1U;
      chassis_turn_stall_used = 1U;
    }
    else
    {
      chassis_turn_strict_active = 0U;
    }
  }

  if ((reached != 0U) || (timed_out != 0U))
  {
    chassis_turn_active = 0U;
    chassis_turn_target_deg = 0.0f;
    chassis_turn_integral = 0.0f;
    chassis_turn_last_error = 0.0f;
    chassis_turn_band_active = 0U;
    chassis_turn_strict_active = 0U;
    chassis_turn_finished = reached;
    chassis_turn_final_error_deg = error;
    chassis_pid_error_deg = 0.0f;
    chassis_pid_output = 0.0f;
    chassis_command = CHASSIS_COMMAND_STOP;
    memset(chassis_target_rpm, 0, sizeof(chassis_target_rpm));
    chassis_output_dirty = 1U;
    return 1U;
  }

  if (ChassisMotor_Abs(error) <= 5.0f)
  {
    chassis_turn_integral += error * delta_time_s;
    chassis_turn_integral = ChassisMotor_Limit(chassis_turn_integral,
                                               -CHASSIS_TURN_INTEGRAL_LIMIT,
                                               CHASSIS_TURN_INTEGRAL_LIMIT);
  }
  else
  {
    chassis_turn_integral = 0.0f;
  }
  derivative = (error - chassis_turn_last_error) / delta_time_s;
  chassis_turn_last_error = error;

  p_term = chassis_turn_kp * error;
  i_term = chassis_turn_ki * chassis_turn_integral;
  d_term = chassis_turn_kd * derivative;
  output = ChassisMotor_Limit(p_term + i_term + d_term,
                              -CHASSIS_TURN_MAX_OUTPUT,
                              CHASSIS_TURN_MAX_OUTPUT);

  /* 摩擦死区补偿（"kick"）：低速齿轮电机的静摩擦死区让很小的 PID 输出推不动轮子，
     末端会卡住不动。做法是：误差还明显大于判据带、且 PID 输出低于"刚好能推动"的门限时，
     补一个推力。
     **必须带运动门控**（2026-09-12 修"末端一直抖"）：
       旧写法只要 |输出| < kick 就无条件把输出顶成 ±kick，于是车正在滑向目标、
       D 项正在刹车（输出为负）时也被顶成正向推力 → 冲过目标 → 反向再来 → 极限环。
       ki=0 时 D 项是唯一阻尼，所以现场是"越调 ki 越抖、调成 0 也抖"。
     现在先算"朝目标的闭合角速度" closing（>0 = 误差正在变小）：
       closing 明显 > 0（正在动）→ 完全交给 PID，连它的刹车一起保留；
       closing ≤ CHASSIS_TURN_KICK_CLOSE_DPS（该动却没动）→ 才补推力。 */
  if ((ChassisMotor_Abs(error) > chassis_turn_profile->kick_deg) &&
      (ChassisMotor_Abs(output) < chassis_turn_profile->min_push))
  {
    float closing_dps = 0.0f;

    if (ChassisMotor_Abs(error) > 0.0001f)
    {
      /* closing = sign(error) × 角速度（error>0 且车在正转 → 误差在变小） */
      closing_dps = (error * current_rate) / ChassisMotor_Abs(error);
    }
    if (closing_dps <= CHASSIS_TURN_KICK_CLOSE_DPS)
    {
      output = (error > 0.0f) ? chassis_turn_profile->min_push
                              : -chassis_turn_profile->min_push;
    }
  }
  chassis_pid_target_deg = chassis_turn_target_deg;
  chassis_pid_error_deg = error;
  chassis_pid_p_term = p_term;
  chassis_pid_i_term = i_term;
  chassis_pid_d_term = d_term;
  chassis_pid_output = output;

  ChassisMotor_MixTurn(output, chassis_turn_profile->max_wheel_rpm);
  chassis_output_dirty = 1U;
  ZDT_Motor_SetVerbose(0U);
  return 0U;
}
void ChassisMotor_Init(void)
{
  ChassisMotor_ClearHeadingReference();
  chassis_field_active=0U;
  memset(chassis_target_rpm, 0, sizeof(chassis_target_rpm));
  memset(chassis_applied_rpm, 0, sizeof(chassis_applied_rpm));
  chassis_mode = CHASSIS_MODE_TURN;
  chassis_command = CHASSIS_COMMAND_STOP;
  chassis_decel_rpm_per_s = CHASSIS_DECEL_RPM_PER_S;
  chassis_output_dirty = 0U;
  chassis_motors_enabled = 0U;
  chassis_profile_started = 0U;
  chassis_last_update_tick = 0U;
  ChassisMotor_CancelTurn();
}

HAL_StatusTypeDef ChassisMotor_Enable(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint32_t i;

  ChassisMotor_CancelTurn();

  /* Clear stale buffered targets before enabling the four motors. */
  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = ZDT_Motor_SetSpeedRpmAddr(
        chassis_addresses[i], 0, CHASSIS_USE_SYNC_TRIGGER);
    if (status != HAL_OK)
    {
      result = status;
    }
    chassis_applied_rpm[i] = 0;
  }

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = ZDT_Motor_EnableAddr(chassis_addresses[i], CHASSIS_USE_SYNC_TRIGGER);
    if (status != HAL_OK)
    {
      result = status;
    }
  }

  if ((result == HAL_OK) && (CHASSIS_USE_SYNC_TRIGGER != 0U))
  {
    result = ZDT_Motor_SyncTrigger();
  }

  if (result == HAL_OK)
  {
    chassis_motors_enabled = 1U;
  }

  return result;
}

HAL_StatusTypeDef ChassisMotor_Disable(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint32_t i;

  (void)ChassisMotor_EmergencyStop();

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = ZDT_Motor_DisableAddr(chassis_addresses[i], CHASSIS_USE_SYNC_TRIGGER);
    if (status != HAL_OK)
    {
      result = status;
    }
  }

  if ((result == HAL_OK) && (CHASSIS_USE_SYNC_TRIGGER != 0U))
  {
    result = ZDT_Motor_SyncTrigger();
  }

  chassis_motors_enabled = 0U;
  return result;
}

HAL_StatusTypeDef ChassisMotor_DisableForArmPreparation(void)
{
  HAL_StatusTypeDef result=ChassisMotor_StopImmediate();uint32_t i;
  /* Preparation is not a whole-robot emergency stop: preserve the ARM owner
   * and its DM fault/zero state. Existing Disable/STOP safety is unchanged. */
  for(i=0U;i<CHASSIS_MOTOR_COUNT;i++) {
    HAL_StatusTypeDef status=ZDT_Motor_DisableAddr(chassis_addresses[i],CHASSIS_USE_SYNC_TRIGGER);
    if(status!=HAL_OK)result=status;
  }
  if(result==HAL_OK && CHASSIS_USE_SYNC_TRIGGER!=0U)result=ZDT_Motor_SyncTrigger();
  chassis_motors_enabled=0U;return result;
}

HAL_StatusTypeDef ChassisMotor_Stop(void)
{
  uint32_t i;

  ChassisMotor_CancelTurn();
  chassis_turn_finished = 0U;
  chassis_command = CHASSIS_COMMAND_STOP;
  if(chassis_field_active) { chassis_field_x=0.0f; chassis_field_y=0.0f; chassis_field_o=0.0f; }
  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    chassis_target_rpm[i] = 0;
  }
  chassis_output_dirty = 1U;
  ZDT_Motor_SetVerbose(0U);
  return HAL_OK;
}

HAL_StatusTypeDef ChassisMotor_StopImmediate(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint32_t i;

  ChassisMotor_CancelTurn();
  chassis_turn_finished = 0U;
  chassis_command = CHASSIS_COMMAND_STOP;
  if(chassis_field_active) { chassis_field_x=0.0f; chassis_field_y=0.0f; chassis_field_o=0.0f; }
  chassis_field_active=0U;
  chassis_vx_cmd=0.0f; chassis_vy_cmd=0.0f; chassis_omega_cmd=0.0f;
  memset(chassis_target_rpm, 0, sizeof(chassis_target_rpm));
  memset(chassis_applied_rpm, 0, sizeof(chassis_applied_rpm));
  chassis_output_dirty = 0U;

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    HAL_StatusTypeDef status = ZDT_Motor_StopAddr(chassis_addresses[i]);
    if (status != HAL_OK)
    {
      result = status;
    }
  }

  return result;
}

HAL_StatusTypeDef ChassisMotor_EmergencyStop(void)
{
  HAL_StatusTypeDef result=ChassisMotor_StopImmediate();
  /* 2026-09-14：急停是**整车**语义 —— 底盘停了但转盘还带力矩的话，
   * 手伸进去拿物料时转盘仍在出力，属于典型危险残留。
   * ArmTask_EmergencyStop() 会清目标 + 发达妙失能帧（松劲）+ 打印一条。
   * 幂等，重复调用安全。 */
  ArmTask_EmergencyStop();

  return result;
}

static HAL_StatusTypeDef ChassisMotor_StartTurn(int16_t angle_deg,
                                                const chassis_turn_profile_t *profile)
{
  float current_yaw;
  int16_t magnitude = (angle_deg < 0) ? (int16_t)(-angle_deg) : angle_deg;

  if ((chassis_motors_enabled == 0U) || (magnitude == 0) || (magnitude > 360))
  {
    return HAL_ERROR;
  }
  if (HWT101_IsZeroDriftReady() == 0U)
  {
    return HAL_ERROR;
  }
  chassis_field_active=0U;

  chassis_turn_profile = profile;
  current_yaw = ChassisMotor_GetTurnYawDeg();
  ChassisMotor_CancelTurn();
  chassis_turn_finished = 0U;
  chassis_turn_target_deg = current_yaw + (float)angle_deg;
  chassis_decel_rpm_per_s = CHASSIS_TURN_DECEL_RPM_PER_S;
  chassis_turn_integral = 0.0f;
  chassis_turn_last_error = (float)angle_deg;
  chassis_turn_band_active = 0U;
  chassis_turn_strict_active = 0U;
  chassis_turn_best_error_deg = 180.0f;
  chassis_turn_best_tick = HAL_GetTick();
  chassis_turn_start_tick = HAL_GetTick();
  chassis_turn_active = 1U;
  chassis_turn_fallback_used = 0U;
  chassis_turn_stall_used = 0U;
  chassis_command = CHASSIS_COMMAND_MANUAL;
  chassis_output_dirty = 1U;
  ZDT_Motor_SetVerbose(0U);
  return HAL_OK;
}

/* 精确档：手动 &L90/&R90 等，按现场要求"误差 ≤0.05° 稳定 0.5 s"。 */
HAL_StatusTypeDef ChassisMotor_TurnRelative(int16_t angle_deg)
{
  return ChassisMotor_StartTurn(angle_deg, &chassis_turn_profile_precise);
}

/* 快速档：导航每段前的对准，0.5°/200 ms 收工；轮速上限与精确档相同（V237）。（直走有航向保持兜底） */
HAL_StatusTypeDef ChassisMotor_TurnRelativeFast(int16_t angle_deg)
{
  return ChassisMotor_StartTurn(angle_deg, &chassis_turn_profile_fast);
}

uint8_t ChassisMotor_IsTurnActive(void)
{
  return chassis_turn_active;
}

uint8_t ChassisMotor_IsEnabled(void)
{
  return chassis_motors_enabled;
}
uint8_t ChassisMotor_TakeTurnFinished(void)
{
  uint8_t finished = chassis_turn_finished;
  chassis_turn_finished = 0U;
  return finished;
}
static HAL_StatusTypeDef ChassisMotor_ApplyVelocity(float vx, float vy,
                                                     float omega)
{
  chassis_field_active=0U;
  if (chassis_motors_enabled == 0U)
  {
    return HAL_ERROR;
  }
  ChassisMotor_CancelTurn();
  chassis_turn_finished = 0U;

  /* Heading hold and arcs include translation and keep its softer ramp.
     A zero command preserves the previous motion's braking rate. */
  if ((vx == 0.0f) && (vy == 0.0f) && (omega != 0.0f))
  {
    chassis_decel_rpm_per_s = CHASSIS_TURN_DECEL_RPM_PER_S;
  }
  else if ((vx != 0.0f) || (vy != 0.0f))
  {
    chassis_decel_rpm_per_s = CHASSIS_DECEL_RPM_PER_S;
  }

  vx = ChassisMotor_Limit(vx, -1.0f, 1.0f);
  vy = ChassisMotor_Limit(vy, -1.0f, 1.0f);
  omega = ChassisMotor_Limit(omega, -1.0f, 1.0f);
  ChassisMotor_Mix(vx, vy, omega);
  chassis_output_dirty = 1U;
  ZDT_Motor_SetVerbose(0U);
  return HAL_OK;
}

HAL_StatusTypeDef ChassisMotor_SetVelocity(float vx, float vy, float omega)
{
  chassis_command = CHASSIS_COMMAND_MANUAL;
  return ChassisMotor_ApplyVelocity(vx, vy, omega);
}

uint8_t ChassisMotor_HasMotionDemand(void)
{
  return (uint8_t)(chassis_turn_active || !ChassisMotor_AllZero(chassis_target_rpm) ||
      (chassis_field_active && (chassis_field_x!=0.0f || chassis_field_y!=0.0f || chassis_field_o!=0.0f)));
}

void ChassisMotor_ResetIdleRamp(void)
{
  if(!ChassisMotor_HasMotionDemand() && ChassisMotor_AllZero(chassis_applied_rpm))
    chassis_profile_started=0U;
}

HAL_StatusTypeDef ChassisMotor_SetFieldVelocity(float fx,float fy,float vx,float vy,float omega)
{
  if(!chassis_motors_enabled) { return HAL_ERROR; }
  if(!isfinite(fx) || !isfinite(fy) || !isfinite(vx) || !isfinite(vy) || !isfinite(omega) ||
      fabsf(fx*fx+fy*fy-1.0f)>0.01f) { return HAL_ERROR; }
  if(!chassis_field_active || (chassis_field_x==0.0f && chassis_field_y==0.0f && chassis_field_o==0.0f &&
      ChassisMotor_AllZero(chassis_applied_rpm))) {
    uint32_t i;
    for(i=0U;i<CHASSIS_MOTOR_COUNT;++i) {
      if(chassis_applied_rpm[i]!=0) { return HAL_ERROR; } /* Enter only from a stopped baseline. */
    }
    memset(&chassis_field_profile,0,sizeof(chassis_field_profile));
    chassis_profile_started=0U; /* Idle time must not become one large acceleration step. */
    chassis_field_yaw=ChassisMotor_GetTurnYawDeg();
  }
  ChassisMotor_CancelTurn(); chassis_turn_finished=0U;
  chassis_field_active=1U; chassis_command=CHASSIS_COMMAND_MANUAL;
  chassis_field_fx=fx; chassis_field_fy=fy;
  chassis_field_x=vx; chassis_field_y=vy; chassis_field_o=omega;
  chassis_output_dirty=1U;
  ZDT_Motor_SetVerbose(0U);
  return HAL_OK;
}

HAL_StatusTypeDef ChassisMotor_SetMode(chassis_mode_t mode)
{
  if ((mode != CHASSIS_MODE_TURN) && (mode != CHASSIS_MODE_TRANSLATION))
  {
    return HAL_ERROR;
  }
  if (mode == chassis_mode)
  {
    return HAL_OK;
  }

  (void)ChassisMotor_Stop();
  chassis_mode = mode;
  return HAL_OK;
}

HAL_StatusTypeDef ChassisMotor_SetCommand(chassis_command_t command)
{
  HAL_StatusTypeDef result;

  if ((chassis_motors_enabled == 0U) &&
      (command != CHASSIS_COMMAND_STOP))
  {
    return HAL_ERROR;
  }

  switch (command)
  {
    case CHASSIS_COMMAND_STOP:
      return ChassisMotor_Stop();

    case CHASSIS_COMMAND_FORWARD:
      result = ChassisMotor_ApplyVelocity(CHASSIS_MANUAL_SPEED_SCALE, 0.0f, 0.0f);
      break;

    case CHASSIS_COMMAND_BACKWARD:
      result = ChassisMotor_ApplyVelocity(-CHASSIS_MANUAL_SPEED_SCALE, 0.0f, 0.0f);
      break;

    case CHASSIS_COMMAND_LEFT:
      if (chassis_mode == CHASSIS_MODE_TURN)
      {
        result = ChassisMotor_ApplyVelocity(0.0f, 0.0f,
                                            CHASSIS_MANUAL_SPEED_SCALE);
      }
      else
      {
        /* 平移模式：按钮 L = 车体向左平移 => vy 为负（vy>0 是车体右侧）。 */
        result = ChassisMotor_ApplyVelocity(0.0f, -CHASSIS_MANUAL_SPEED_SCALE, 0.0f);
      }
      break;

    case CHASSIS_COMMAND_RIGHT:
      if (chassis_mode == CHASSIS_MODE_TURN)
      {
        result = ChassisMotor_ApplyVelocity(0.0f, 0.0f,
                                            -CHASSIS_MANUAL_SPEED_SCALE);
      }
      else
      {
        result = ChassisMotor_ApplyVelocity(0.0f, CHASSIS_MANUAL_SPEED_SCALE, 0.0f);
      }
      break;

    default:
      return HAL_ERROR;
  }

  if (result == HAL_OK)
  {
    chassis_command = command;
  }
  return result;
}

HAL_StatusTypeDef ChassisMotor_Update(void)
{
  HAL_StatusTypeDef result;
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms;
  float delta_time_s;
  uint32_t i;

  if ((chassis_turn_active == 0U) &&
      (chassis_output_dirty == 0U) &&
      (ChassisMotor_AllZero(chassis_target_rpm) != 0U) &&
      (ChassisMotor_AllZero(chassis_applied_rpm) != 0U))
  {
    return HAL_OK;
  }

  if ((chassis_profile_started != 0U) &&
      ((now - chassis_last_update_tick) < CHASSIS_TX_PERIOD_MS))
  {
    return HAL_OK;
  }

  if (chassis_motors_enabled == 0U)
  {
    return HAL_ERROR;
  }

  elapsed_ms = (chassis_profile_started != 0U) ?
      (now - chassis_last_update_tick) : CHASSIS_TX_PERIOD_MS;
  chassis_last_update_tick = now;
  chassis_profile_started = 1U;
  if (elapsed_ms > 100U)
  {
    elapsed_ms = 100U;
  }
  delta_time_s = (float)elapsed_ms * 0.001f;

  if(chassis_field_active) {
    float yaw=ChassisMotor_GetTurnYawDeg();
    if(!isfinite(yaw) || fabsf(remainderf(yaw-chassis_field_yaw,360.0f))>12.0f) {
      (void)ChassisMotor_StopImmediate(); return HAL_ERROR;
    }
    chassis_field_yaw=yaw;
    ChassisField_Step(&chassis_field_profile,chassis_field_x,chassis_field_y,chassis_field_o,
        chassis_field_fx,chassis_field_fy,yaw,delta_time_s,
        CHASSIS_MAX_WHEEL_RPM,CHASSIS_ACCEL_RPM_PER_S,CHASSIS_DECEL_RPM_PER_S,CHASSIS_TURN_DECEL_RPM_PER_S);
    chassis_vx_cmd=(chassis_field_profile.wheel[0]+chassis_field_profile.wheel[1]+
        chassis_field_profile.wheel[2]+chassis_field_profile.wheel[3])*0.25f;
    chassis_vy_cmd=(chassis_field_profile.wheel[0]-chassis_field_profile.wheel[1]-
        chassis_field_profile.wheel[2]+chassis_field_profile.wheel[3])*0.25f;
    chassis_omega_cmd=chassis_field_profile.omega;
    for(i=0U;i<CHASSIS_MOTOR_COUNT;++i) {
      chassis_applied_rpm[i]=ChassisMotor_ToRpm(chassis_field_profile.wheel[i]*CHASSIS_MAX_WHEEL_RPM*chassis_wheel_signs[i]);
      chassis_target_rpm[i]=chassis_applied_rpm[i];
    }
    result=ChassisMotor_SendTargets();
    if(result==HAL_OK) { chassis_output_dirty=0U; }
    return result;
  }

  if (chassis_turn_active != 0U)
  {
    (void)ChassisMotor_UpdateTurn(delta_time_s);
  }
  for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
  {
    chassis_applied_rpm[i] = ChassisMotor_RampStepRpm(
        chassis_applied_rpm[i], chassis_target_rpm[i], delta_time_s, chassis_decel_rpm_per_s);
  }

  result = ChassisMotor_SendTargets();
  if (result == HAL_OK)
  {
    chassis_output_dirty = 0U;
  }
  return result;
}

void ChassisMotor_GetWheelRpm(int16_t rpm[CHASSIS_MOTOR_COUNT])
{
  if (rpm != NULL)
  {
    memcpy(rpm, chassis_applied_rpm, sizeof(chassis_applied_rpm));
  }
}
void ChassisMotor_SetTurnGains(float kp, float ki, float kd)
{
  chassis_turn_kp = kp;
  chassis_turn_ki = ki;
  chassis_turn_kd = kd;
}

void ChassisMotor_GetTurnGains(float *kp, float *ki, float *kd)
{
  if (kp != NULL) { *kp = chassis_turn_kp; }
  if (ki != NULL) { *ki = chassis_turn_ki; }
  if (kd != NULL) { *kd = chassis_turn_kd; }
}

void ChassisMotor_GetTelemetry(chassis_telemetry_t *telemetry)
{
  if (telemetry == NULL)
  {
    return;
  }

  telemetry->heading_deg = ChassisMotor_GetTurnYawDeg();
  telemetry->target_heading_deg = chassis_pid_target_deg;
  telemetry->gyro_dps = HWT101_GetGyroZDps() * CHASSIS_YAW_RIGHT_POSITIVE_SIGN;
  telemetry->vx = chassis_vx_cmd;
  telemetry->vy = chassis_vy_cmd;
  telemetry->omega = chassis_omega_cmd;
  telemetry->pid_error_deg = chassis_pid_error_deg;
  telemetry->pid_p_term = chassis_pid_p_term;
  telemetry->pid_i_term = chassis_pid_i_term;
  telemetry->pid_d_term = chassis_pid_d_term;
  telemetry->pid_output = chassis_pid_output;
  telemetry->turn_elapsed_ms = HAL_GetTick() - chassis_turn_start_tick;
  telemetry->turn_fallback = chassis_turn_fallback_used;
  telemetry->turn_stall = chassis_turn_stall_used;
  telemetry->turn_profile =
      (uint8_t)((chassis_turn_profile == &chassis_turn_profile_fast) ?
                CHASSIS_TURN_PROFILE_FAST : CHASSIS_TURN_PROFILE_PRECISE);
  telemetry->turn_final_error_deg = chassis_turn_final_error_deg;
  telemetry->turn_active = chassis_turn_active;
}

float ChassisMotor_GetTurnTargetDeg(void)
{
  /* 直接给绝对目标，而不是 telemetry 里每个控制周期刷新的 chassis_pid_target_deg：
   * 上位机在“起转沿”就要画目标阶跃，若等控制周期刷新会晚一拍、且收工后会被清零。 */
  return chassis_turn_active ? chassis_turn_target_deg : chassis_pid_target_deg;
}

float ChassisMotor_GetTurnMaxWheelRpm(void)
{
  /* 手动 &L/&R 精转固定走精确档，所以这里直接报告**精确档**上限，
   * 不受“最近一次导航快速档”影响；上位机 &tstat 用它确认 2 倍限幅是否生效。 */
  return chassis_turn_profile_precise.max_wheel_rpm;
}


chassis_mode_t ChassisMotor_GetMode(void)
{
  return chassis_mode;
}

chassis_command_t ChassisMotor_GetCommand(void)
{
  return chassis_command;
}
