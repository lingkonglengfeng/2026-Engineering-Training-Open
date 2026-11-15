
/**
  ******************************************************************************
  * @file    light.c
  * @brief   旧补光灯软件PWM（当前仅PD2，PD1已改为启动按键）
  * @note    时基推导：TIM6 时钟 = APB1 定时器时钟 275MHz，
  *          PSC=0、ARR=17187 → 更新中断频率 275MHz/17188 ≈ 16.0kHz，
  *          计数 16 级回绕 → 调光频率 ≈ 1.0kHz。
  *          TIM6 未被 HAL 时基（TIM7）或 CubeMX 占用，可安全独占；
  *          中断服务函数直接操作寄存器，单次耗时 <1µs，CPU 占用 <0.5%。
  ******************************************************************************
  */

#include "light.h"

/* ================= 软件PWM 参数 ================= */

/* 灰度级数（16 级），必须为 2 的幂以便取模优化 */
#define LIGHT_LEVELS                 (16U)
#define LIGHT_LEVELS_MASK            (LIGHT_LEVELS - 1U)

/* TIM6 分频：275MHz / (17187+1) ≈ 16.0kHz 更新中断 */
#define LIGHT_TIM_PERIOD             (17187U)

/* V185: PD1 belongs to the START switch; legacy CH1 has no GPIO output. */
#define LIGHT_CH1_PIN                (0U)
#define LIGHT_CH2_PIN                (GPIO_PIN_2)

/* 输出有效电平：1=高电平点亮（3.3V 直驱，高电平=导通）；
 * 若实际灯接法为低电平点亮，改此宏为 1U 并交换置位/复位逻辑即可。 */
#define LIGHT_ACTIVE_HIGH            (1U)

/* ================= 内部状态 ================= */

static TIM_HandleTypeDef s_tim6;

/* 占空比（0~16 计数tick）。任务上下文写、中断读，
 * 单字节写入天然原子，仍按规范加 volatile。 */
static volatile uint8_t s_duty_ticks[LIGHT_CH_COUNT];

/* 软件PWM相位计数器，仅中断上下文修改 */
static volatile uint8_t s_phase;

static uint8_t s_saved_duty[LIGHT_CH_COUNT];   /* Light_Off 前的亮度记忆 */
static uint8_t s_initialized;

/* ================= 内部函数 ================= */

/**
  * @brief  把百分比换算成灰度 tick（0~16）
  */
static uint8_t Light_PercentToTicks(uint8_t percent)
{
    uint32_t ticks;

    if (percent >= 100U)
    {
        return (uint8_t)LIGHT_LEVELS;
    }
    ticks = ((uint32_t)percent * LIGHT_LEVELS + 50U) / 100U;
    return (uint8_t)ticks;
}

/**
  * @brief  把灰度 tick 换算回百分比
  */
static uint8_t Light_TicksToPercent(uint8_t ticks)
{
    return (uint8_t)(((uint32_t)ticks * 100U + LIGHT_LEVELS / 2U) / LIGHT_LEVELS);
}

/* ================= 对外接口 ================= */

HAL_StatusTypeDef Light_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    uint8_t index;

    if (s_initialized != 0U)
    {
        return HAL_OK;
    }

    /* PD2 推挽输出，初始低电平（灯灭）；PD1保持启动按键输入。
     * PD 口时钟由 CubeMX 已开启，此处冗余使能保证独立初始化安全。 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio_init.Pin = LIGHT_CH1_PIN | LIGHT_CH2_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio_init);
    HAL_GPIO_WritePin(GPIOD, LIGHT_CH1_PIN | LIGHT_CH2_PIN, GPIO_PIN_RESET);

    for (index = 0; index < (uint8_t)LIGHT_CH_COUNT; index++)
    {
        s_duty_ticks[index] = 0U;
        s_saved_duty[index] = Light_PercentToTicks(80U);   /* 记忆亮度默认 80% */
    }
    s_phase = 0U;

    /* TIM6：275MHz、16kHz 更新中断，仅作为软件PWM节拍 */
    __HAL_RCC_TIM6_CLK_ENABLE();
    s_tim6.Instance = TIM6;
    s_tim6.Init.Prescaler = 0U;
    s_tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim6.Init.Period = LIGHT_TIM_PERIOD;
    s_tim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim6.Init.RepetitionCounter = 0U;
    s_tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_tim6) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 中断优先级 7：数值 ≥5 满足 FreeRTOS FromISR 约束；
     * 本中断内部不调用任何 RTOS API。 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    __HAL_TIM_ENABLE_IT(&s_tim6, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&s_tim6);

    s_initialized = 1U;
    return HAL_OK;
}

void Light_SetDuty(light_ch_t ch, uint8_t percent)
{
    if ((uint8_t)ch >= (uint8_t)LIGHT_CH_COUNT)
    {
        return;
    }
    if (s_initialized == 0U)
    {
        return;
    }
    s_duty_ticks[(uint8_t)ch] = Light_PercentToTicks(percent);
}

uint8_t Light_GetDuty(light_ch_t ch)
{
    if ((uint8_t)ch >= (uint8_t)LIGHT_CH_COUNT)
    {
        return 0U;
    }
    return Light_TicksToPercent(s_duty_ticks[(uint8_t)ch]);
}

void Light_On(void)
{
    uint8_t index;

    for (index = 0; index < (uint8_t)LIGHT_CH_COUNT; index++)
    {
        s_duty_ticks[index] = s_saved_duty[index];
    }
}

void Light_Off(void)
{
    uint8_t index;

    for (index = 0; index < (uint8_t)LIGHT_CH_COUNT; index++)
    {
        s_saved_duty[index] = s_duty_ticks[index];
        s_duty_ticks[index] = 0U;
    }
}

/* ================= TIM6 更新中断：软件PWM 核心 ================= */

/**
  * @brief  TIM6 更新中断（16kHz）：按相位翻转有效灯脚（当前仅PD2）
  * @note   相位 0 时按占空比决定本周期起始电平（0 灰度不亮）；
  *         相位到达占空比 tick 时拉低，形成 16 级阶梯调光。
  *         直接读写寄存器保证单次中断耗时 <1µs。
  */
void TIM6_DAC_IRQHandler(void)
{
    uint32_t bsrr_set = 0U;
    uint32_t bsrr_reset = 0U;

    if ((TIM6->SR & TIM_SR_UIF) == 0U)
    {
        return;
    }
    TIM6->SR = (uint32_t)~TIM_SR_UIF;    /* 清更新标志（写0清零） */

    s_phase = (uint8_t)((s_phase + 1U) & LIGHT_LEVELS_MASK);

#if LIGHT_ACTIVE_HIGH
    if (s_phase == 0U)
    {
        /* 新周期开始：占空比>0 的通道置高，占空比=0 的通道保持/拉低 */
        if (s_duty_ticks[(uint8_t)LIGHT_CH1] != 0U) { bsrr_set |= LIGHT_CH1_PIN; }
        else                                        { bsrr_reset |= LIGHT_CH1_PIN; }
        if (s_duty_ticks[(uint8_t)LIGHT_CH2] != 0U) { bsrr_set |= LIGHT_CH2_PIN; }
        else                                        { bsrr_reset |= LIGHT_CH2_PIN; }
    }
    if (s_phase == s_duty_ticks[(uint8_t)LIGHT_CH1]) { bsrr_reset |= LIGHT_CH1_PIN; }
    if (s_phase == s_duty_ticks[(uint8_t)LIGHT_CH2]) { bsrr_reset |= LIGHT_CH2_PIN; }
#else
    if (s_phase == 0U)
    {
        if (s_duty_ticks[(uint8_t)LIGHT_CH1] != LIGHT_LEVELS) { bsrr_set |= LIGHT_CH1_PIN; }
        else                                                  { bsrr_reset |= LIGHT_CH1_PIN; }
        if (s_duty_ticks[(uint8_t)LIGHT_CH2] != LIGHT_LEVELS) { bsrr_set |= LIGHT_CH2_PIN; }
        else                                                  { bsrr_reset |= LIGHT_CH2_PIN; }
    }
    if (s_phase == (uint8_t)(LIGHT_LEVELS - s_duty_ticks[(uint8_t)LIGHT_CH1])) { bsrr_reset |= LIGHT_CH1_PIN; }
    if (s_phase == (uint8_t)(LIGHT_LEVELS - s_duty_ticks[(uint8_t)LIGHT_CH2])) { bsrr_reset |= LIGHT_CH2_PIN; }
#endif

    if (bsrr_set != 0U)
    {
        GPIOD->BSRR = bsrr_set;
    }
    if (bsrr_reset != 0U)
    {
        GPIOD->BSRR = bsrr_reset << 16U;
    }
}
