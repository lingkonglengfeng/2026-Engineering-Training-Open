/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile uint32_t g_boot_stage;
extern volatile uint32_t g_boot_error;
float HWT101_GetYawDeg(void);
float HWT101_GetGyroZDps(void);
uint32_t HWT101_GetSampleCount(void);
uint32_t HWT101_GetErrorCount(void);
float HWT101_GetZeroDriftDeg(void);
uint8_t HWT101_IsZeroDriftReady(void);
/* 标定完成时刻（HAL tick，0 = 还没完成过）；屏幕用它显示"去零漂完成"数秒。 */
uint32_t HWT101_GetZeroDriftReadyTick(void);
/* 当前静止窗口已累计毫秒数（被打断归零）；屏幕显示倒计时用。 */
uint32_t HWT101_GetCalibElapsedMs(void);
/* 上电自动标定得到的角速度零偏（dps）；HWT101_GetGyroZDps 已经扣掉它。 */
float HWT101_GetGyroBiasDps(void);
/* 零漂标定进行中（此期间必须静止，否则窗口重新计时）。 */
uint8_t HWT101_IsCalibrating(void);
/* 任务上下文里调用：把"标定完成"那行打出去（ISR 里只置标志）。 */
void HWT101_PollReport(void);
/* 把当前朝向重新当作 0°（摆好车以后再对基准）。调用后需静止约 3 s（HWT101_CALIB_MS）。 */
void HWT101_ResetZeroDrift(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IPS_CS_Pin GPIO_PIN_6
#define IPS_CS_GPIO_Port GPIOC
#define IPS_DC_Pin GPIO_PIN_7
#define IPS_DC_GPIO_Port GPIOC
#define IPS_RST_Pin GPIO_PIN_8
#define IPS_RST_GPIO_Port GPIOC
#define START_BUTTON_Pin GPIO_PIN_1
#define START_BUTTON_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
