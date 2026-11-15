
/**
  ******************************************************************************
  * @file    servo.c
  * @brief   270°舵机驱动实现（TIM4_CH1/CH2，50Hz，500~2500us）
  * @note    时基推导（对应 main.c 时钟树）：
  *          SYSCLK = 550MHz(HSE 25MHz × PLL) → AHB÷2 = 275MHz
  *          → APB1÷2 = 137.5MHz → 定时器时钟×2 = 275MHz。
  *          CubeMX 生成的 TIM4 参数（PSC=0/ARR=65535）只是占位配置，
  *          因此 50Hz 时基由本驱动在 Servo_Init() 内重配，
  *          重新生成 .ioc 时不会覆盖本文件的参数表。
  ******************************************************************************
  */

#include "servo.h"

#include "tim.h"

/* ================= 时基参数 ================= */

/* 计数时钟 = 275MHz / (274+1) = 1MHz，即 1 个计数对应 1us，
 * 比较值可以直接用脉宽微秒数填写，便于参数标定。 */
#define SERVO_TIM_PRESCALER          (274U)
/* 周期 = (19999+1) × 1us = 20ms，即 PWM 频率 50Hz（冻结表 6.1 节） */
#define SERVO_TIM_PERIOD             (19999U)

/* ================= 270° 舵机脉宽约定 ================= */

#define SERVO_PULSE_MIN_US           (500U)   /* 0° 对应脉宽 */
#define SERVO_PULSE_MAX_US           (2500U)  /* 270° 对应脉宽 */

/* ================= 安全与节拍参数 ================= */

/* 限速上限：超过常规舵机空载速度太多，避免误设 */
#define SERVO_SPEED_MAX_DPS          (360.0f)
/* 单次 Update 的 dt 上限（秒）：任务被长时间阻塞后恢复时，
 * 防止按真实 dt 一次性补走大角度造成机构撞击 */
#define SERVO_STEP_MAX_DT_S          (0.05f)
/* 到位判据：目标与输出之差小于该值视为已到位 */
#define SERVO_ARRIVED_EPS_DEG        (0.2f)
/* 速度下限：防止误设 0 导致舵机永远无法运动 */
#define SERVO_SPEED_MIN_DPS          (1.0f)

/* ================= 每舵机参数表 =================
 * 冻结表第 14 节要求：零位、方向、极限角、限速集中管理。
 * 当前全部为待标定默认值，整车联调时只允许修改本表，
 * 不允许在状态机分支里散写角度/速度常数。 */

typedef struct
{
    float angle_min;    /* 机械最小角度 */
    float angle_max;    /* 机械最大角度 */
    float boot_angle;   /* 上电安全角（软启动值） */
    float speed_dps;    /* 角速度上限（度/秒） */
    uint8_t invert;     /* 安装镜像标志：1 时脉宽随角度反向映射 */
} servo_param_t;

typedef struct
{
    float current_angle; /* 当前输出角（斜坡瞬时值） */
    float target_angle;  /* 目标角 */
} servo_state_t;

static servo_param_t g_servo_params[SERVO_ID_COUNT] = {
    /* 夹爪 DS3218：行程待机械标定，先给出全行程和安全中位 */
    [SERVO_ID_GRIP] = {
        .angle_min = 0.0f,
        .angle_max = 270.0f,
        .boot_angle = 135.0f,
        .speed_dps = 90.0f,
        .invert = 0U
    },
    /* 转盘舵机：转盘零位（取料位）冻结前先给中位 */
    [SERVO_ID_TURNTABLE] = {
        .angle_min = 0.0f,
        .angle_max = 270.0f,
        .boot_angle = 135.0f,
        .speed_dps = 60.0f,
        .invert = 0U
    }
};

static servo_state_t g_servo_state[SERVO_ID_COUNT];

/* 舵机编号 → HAL 定时器通道 */
static const uint32_t g_servo_channel[SERVO_ID_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2
};

static uint8_t g_initialized;

/* 急停相关标志。g_safe_stop_request 会被 ISR 置位，必须 volatile；
 * g_safe_stop_latched 只在任务上下文（Servo_Update）中修改。 */
static volatile uint8_t g_safe_stop_request;
static uint8_t g_safe_stop_latched;

/* 上一次 Servo_Update 的时基，用于计算斜坡步长 */
static uint32_t g_last_update_ms;
static uint8_t g_last_update_valid;

/* ================= 内部函数 ================= */

/**
  * @brief  把角度钳位到该舵机的机械行程内
  */
static float Servo_ClampAngle(uint8_t index, float angle_deg)
{
    const servo_param_t *param = &g_servo_params[index];

    if (angle_deg < param->angle_min)
    {
        angle_deg = param->angle_min;
    }
    if (angle_deg > param->angle_max)
    {
        angle_deg = param->angle_max;
    }
    return angle_deg;
}

/**
  * @brief  角度 → 脉宽（微秒）
  * @note   行程线性映射 0°→500us、270°→2500us；invert 时反向。
  *         返回值同时被用作比较寄存器值（计数时钟 1MHz，1 计数=1us）。
  */
static uint32_t Servo_AngleToPulseUs(uint8_t index, float angle_deg)
{
    const servo_param_t *param = &g_servo_params[index];
    float ratio;
    float pulse_us;

    angle_deg = Servo_ClampAngle(index, angle_deg);

    /* 归一化行程比例 0~1；参数表保证 angle_max > angle_min */
    ratio = (angle_deg - param->angle_min) /
            (param->angle_max - param->angle_min);
    if (param->invert != 0U)
    {
        ratio = 1.0f - ratio;
    }

    pulse_us = (float)SERVO_PULSE_MIN_US +
               ratio * ((float)SERVO_PULSE_MAX_US - (float)SERVO_PULSE_MIN_US);

    return (uint32_t)(pulse_us + 0.5f);
}

/**
  * @brief  直接把某一路的角度写进比较寄存器
  */
static void Servo_WriteCompare(uint8_t index, float angle_deg)
{
    __HAL_TIM_SET_COMPARE(&htim4, g_servo_channel[index],
                          Servo_AngleToPulseUs(index, angle_deg));
}

/* ================= 对外接口 ================= */

HAL_StatusTypeDef Servo_Init(void)
{
    TIM_OC_InitTypeDef oc_config = {0};
    uint8_t index;
    HAL_StatusTypeDef status;

    if (g_initialized != 0U)
    {
        return HAL_OK;
    }

    /* 重配 TIM4 时基为 50Hz。前提：MX_TIM4_Init() 已运行，
     * GPIO 复用与 TIM4 时钟已就绪。重复调用 HAL_TIM_PWM_Init
     * 只会更新 PSC/ARR，不会重复执行 MspInit。 */
    htim4.Init.Prescaler = SERVO_TIM_PRESCALER;
    htim4.Init.Period = SERVO_TIM_PERIOD;
    status = HAL_TIM_PWM_Init(&htim4);
    if (status != HAL_OK)
    {
        return status;
    }

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;

    /* 上电先输出安全角（软启动）：先写比较值再启动输出，
     * 避免第一帧脉冲使用残留的占位比较值 0（对应持续低电平）。 */
    for (index = 0; index < (uint8_t)SERVO_ID_COUNT; index++)
    {
        oc_config.Pulse = Servo_AngleToPulseUs(index,
                                               g_servo_params[index].boot_angle);
        status = HAL_TIM_PWM_ConfigChannel(&htim4, &oc_config,
                                           g_servo_channel[index]);
        if (status != HAL_OK)
        {
            return status;
        }

        g_servo_state[index].current_angle =
            Servo_ClampAngle(index, g_servo_params[index].boot_angle);
        g_servo_state[index].target_angle = g_servo_state[index].current_angle;
    }

    status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    if (status != HAL_OK)
    {
        return status;
    }

    g_safe_stop_request = 0U;
    g_safe_stop_latched = 0U;
    g_last_update_valid = 0U;
    g_initialized = 1U;

    return HAL_OK;
}

void Servo_SetAngle(servo_id_t id, float angle_deg)
{
    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return;
    }
    if (g_initialized == 0U)
    {
        return;
    }
    /* 安全停锁存期间禁止新增动作（冻结表 6.1 规则 2） */
    if (g_safe_stop_latched != 0U)
    {
        return;
    }

    g_servo_state[id].target_angle = Servo_ClampAngle((uint8_t)id, angle_deg);
}

void Servo_SetPulseUs(servo_id_t id, uint16_t pulse_us)
{
    const servo_param_t *param;
    float ratio;
    float angle;

    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return;
    }
    if (g_initialized == 0U)
    {
        return;
    }
    if (g_safe_stop_latched != 0U)
    {
        return;
    }

    /* 脉宽钳位到 500~2500us */
    if (pulse_us < SERVO_PULSE_MIN_US)
    {
        pulse_us = SERVO_PULSE_MIN_US;
    }
    if (pulse_us > SERVO_PULSE_MAX_US)
    {
        pulse_us = SERVO_PULSE_MAX_US;
    }

    /* 把脉宽换算成等效角度交给限速斜坡，行程未知时不影响使用：
     * 500us 对应参数表 angle_min，2500us 对应 angle_max */
    param = &g_servo_params[id];
    ratio = ((float)pulse_us - (float)SERVO_PULSE_MIN_US) /
            ((float)SERVO_PULSE_MAX_US - (float)SERVO_PULSE_MIN_US);
    if (param->invert != 0U)
    {
        ratio = 1.0f - ratio;
    }
    angle = param->angle_min + ratio * (param->angle_max - param->angle_min);

    g_servo_state[id].target_angle = angle;
}

void Servo_SetSpeedLimit(servo_id_t id, float dps)
{
    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return;
    }
    if (dps < SERVO_SPEED_MIN_DPS)
    {
        dps = SERVO_SPEED_MIN_DPS;
    }
    if (dps > SERVO_SPEED_MAX_DPS)
    {
        dps = SERVO_SPEED_MAX_DPS;
    }
    g_servo_params[id].speed_dps = dps;
}

void Servo_Update(uint32_t now_ms)
{
    float dt_s;
    uint8_t index;

    /* 急停请求可能来自 ISR，这里只在任务上下文执行真正的 PWM 停止 */
    if (g_safe_stop_request != 0U)
    {
        g_safe_stop_request = 0U;
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        g_safe_stop_latched = 1U;
    }

    if (g_safe_stop_latched != 0U)
    {
        return;
    }

    if (g_initialized == 0U)
    {
        return;
    }

    if (g_last_update_valid == 0U)
    {
        /* 首次调用只记录时基，不产生步进 */
        g_last_update_ms = now_ms;
        g_last_update_valid = 1U;
        return;
    }

    /* 无符号减法可自然处理时基回绕 */
    dt_s = (float)(now_ms - g_last_update_ms) / 1000.0f;
    g_last_update_ms = now_ms;
    if (dt_s > SERVO_STEP_MAX_DT_S)
    {
        dt_s = SERVO_STEP_MAX_DT_S;
    }
    if (dt_s <= 0.0f)
    {
        return;
    }

    for (index = 0; index < (uint8_t)SERVO_ID_COUNT; index++)
    {
        servo_state_t *state = &g_servo_state[index];
        float delta = state->target_angle - state->current_angle;
        float max_step = g_servo_params[index].speed_dps * dt_s;

        if (delta > max_step)
        {
            delta = max_step;
        }
        else if (delta < -max_step)
        {
            delta = -max_step;
        }

        if (delta != 0.0f)
        {
            state->current_angle += delta;
            Servo_WriteCompare(index, state->current_angle);
        }
    }
}

float Servo_GetCurrentAngle(servo_id_t id)
{
    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return 0.0f;
    }
    return g_servo_state[id].current_angle;
}

float Servo_GetTargetAngle(servo_id_t id)
{
    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return 0.0f;
    }
    return g_servo_state[id].target_angle;
}

uint8_t Servo_IsBusy(servo_id_t id)
{
    float delta;

    if ((uint8_t)id >= (uint8_t)SERVO_ID_COUNT)
    {
        return 0U;
    }

    delta = g_servo_state[id].target_angle - g_servo_state[id].current_angle;
    if (delta < 0.0f)
    {
        delta = -delta;
    }
    return (delta > SERVO_ARRIVED_EPS_DEG) ? 1U : 0U;
}

void Servo_RequestSafeStop(void)
{
    /* 只置位标志，可在 ISR 中安全调用（冻结表 6.2 安全约束 1） */
    g_safe_stop_request = 1U;
}

uint8_t Servo_IsSafeStopLatched(void)
{
    return g_safe_stop_latched;
}

HAL_StatusTypeDef Servo_RecoverFromSafeStop(void)
{
    uint8_t index;

    if (g_initialized == 0U)
    {
        return HAL_ERROR;
    }
    if (g_safe_stop_latched == 0U)
    {
        return HAL_OK;
    }

    /* 脱力期间物理位置未知，按上电流程重新输出安全角：
     * 先写比较值再启动 PWM，确保第一帧脉冲就是安全角。 */
    for (index = 0; index < (uint8_t)SERVO_ID_COUNT; index++)
    {
        g_servo_state[index].current_angle =
            Servo_ClampAngle(index, g_servo_params[index].boot_angle);
        g_servo_state[index].target_angle = g_servo_state[index].current_angle;
        Servo_WriteCompare(index, g_servo_state[index].current_angle);
    }

    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    g_last_update_valid = 0U;
    g_safe_stop_latched = 0U;

    return HAL_OK;
}
