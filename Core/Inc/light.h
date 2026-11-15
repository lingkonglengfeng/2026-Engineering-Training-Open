

/**
  ******************************************************************************
  * @file    light.h
  * @brief   旧补光灯软件PWM接口（当前仅PD2，PD1已改为启动按键）
  * @note    背景（冻结表 v1.3 §6.4）：LED指示灯曾使用PD1/PD2补光；V185将PD1保留给启动键，PD2维持灯
  *          调光输出。H723 上 PD1 无定时器复用、PD2 仅有 TIM3_ETR 输入复用，
  *          两脚均无法硬件 PWM，故采用 **TIM6 更新中断 + GPIO 翻转** 的软件 PWM：
  *          - 中断频率 16kHz，灰度 16 级 → 调光频率 1kHz（LED 无可见闪烁）；
  *          - 两通道相互独立，占空比 0~100%（内部换算 16 级）；
  *          - 灯为 3.3V 直驱，限流电阻由灯侧处理；单通道电流若超过
  *            GPIO 20mA 上限，须在灯侧加三极管/MOS，驱动极性不变。
  *
  *          使用顺序：任意任务上下文调用 Light_Init() 一次（灯灭启动），
  *          之后 Light_SetDuty()/Light_On()/Light_Off() 随时调光。
  *          输出由 TIM6 中断维持，任务无需周期喂狗式刷新。
  ******************************************************************************
  */

#ifndef __LIGHT_H__
#define __LIGHT_H__

#include <stdint.h>

#include "stm32h7xx_hal.h"

/* 补光灯通道编号 */
typedef enum
{
    LIGHT_CH1 = 0,      /* V185: legacy logical channel; no GPIO, PD1 is START. */
    LIGHT_CH2 = 1,      /* PD2：补光灯通道 2 / 备用 */
    LIGHT_CH_COUNT
} light_ch_t;

/**
  * @brief  初始化旧补光灯：仅配置PD2输出；PD1保留为启动按键输入
  * @note   初始化后两通道占空比为 0（灯灭）。重复调用直接返回成功。
  * @retval HAL_OK 成功；其他值表示 TIM6 配置失败
  */
HAL_StatusTypeDef Light_Init(void);

/**
  * @brief  设置通道占空比（亮度）
  * @param  ch      通道编号
  * @param  percent 0~100，超范围截断；0=常灭，100=常亮
  * @note   下一个 PWM 周期（≤1ms）生效；可安全在任意任务中调用。
  */
void Light_SetDuty(light_ch_t ch, uint8_t percent);

/**
  * @brief  读取通道当前占空比（0~100）
  */
uint8_t Light_GetDuty(light_ch_t ch);

/**
  * @brief  全部通道按各自记忆亮度输出
  */
void Light_On(void);

/**
  * @brief  全部通道立即熄灭（占空比记忆保留，Light_On 恢复）
  */
void Light_Off(void);

#endif /* __LIGHT_H__ */
