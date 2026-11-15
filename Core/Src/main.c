/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "vision.h"
#include "dm_motor.h"
#include "zdt_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_boot_stage;
volatile uint32_t g_boot_error;

/* HWT101 serial data is written in the USART10 interrupt callback. */
static volatile uint8_t hwt101_frame[11];
static volatile uint8_t hwt101_frame_pos;
static volatile float hwt101_yaw_deg;
static volatile float hwt101_gyro_z_dps;
static volatile uint32_t hwt101_sample_count;
static volatile uint32_t hwt101_rx_error_count;
/* 零漂基准在收到若干帧姿态后建立，之后累计 Yaw 相对基准的变化量。 */
static volatile uint8_t hwt101_drift_ready;
static volatile float hwt101_yaw_baseline_deg;
static volatile float hwt101_yaw_previous_deg;
static volatile float hwt101_zero_drift_deg;
/* 静止标定窗口的累加器与角速度零偏 */
static volatile uint8_t hwt101_calib_active;
static volatile uint32_t hwt101_calib_start_tick;
static volatile uint32_t hwt101_calib_count;
static volatile double hwt101_calib_yaw_sum;
static volatile double hwt101_calib_gyro_sum;
static volatile float hwt101_gyro_bias_dps;
/* 标定完成后的打印交给任务上下文（ZDT_Motor_Poll）做：
   0x53 是在 UART 空闲中断回调里解析的，在 ISR 里阻塞发 60 字符会拖住接收重挂。 */
static volatile uint8_t hwt101_calib_report_pending;
static volatile uint32_t hwt101_calib_report_samples;
static volatile uint32_t hwt101_calib_report_ms;
static volatile uint8_t hwt101_calib_reported;
/* 标定完成时刻（给屏幕显示"去零漂完成"用；0 = 还没完成过） */
static volatile uint32_t hwt101_zero_ready_tick;
/* 窗口内 yaw 的极值，用于"静止度"检查（见 HWT101_CALIB_MAX_SPAN_DEG） */
static volatile float hwt101_calib_yaw_min;
static volatile float hwt101_calib_yaw_max;

/* HWT101 上电自动零漂标定：
 *   只有在"车真的静止"时才建立基准，并且用一段时间的**平均值**做基准与角速度零偏，
 *   避免上电瞬间被人手拿着/车在动导致基准错掉。
 *   判据：① 角速度 |gz| ≤ HWT101_CALIB_MAX_RATE_DPS 连续保持 ≥ HWT101_CALIB_MS 毫秒；
 *         ② 窗口内 yaw 极差 ≤ HWT101_CALIB_MAX_SPAN_DEG（长时间窗口下，光看瞬时角速度
 *            挡不住"慢慢转"或"被人轻轻挪"）；
 *         ③ 样本数 ≥ HWT101_CALIB_MIN_SAMPLES。
 *   任一条不满足就重新开始计时，所以上电后不动它 ~3 s 就会自己完成。 */
#define HWT101_CALIB_MAX_RATE_DPS       (1.5f)
#define HWT101_CALIB_MS                 (3000U)
#define HWT101_CALIB_MAX_SPAN_DEG       (0.50f)
#define HWT101_CALIB_MIN_SAMPLES        (20U)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void HWT101_OnDmaData(const uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int16_t HWT101_ReadLeInt16(const volatile uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static float HWT101_WrapAngleDeltaDeg(float delta_deg)
{
  /* 将 Yaw 跨越 +/-180 度的角度差折回到 (-180,180]，保证累计量连续。 */
  while (delta_deg > 180.0F)
  {
    delta_deg -= 360.0F;
  }

  while (delta_deg <= -180.0F)
  {
    delta_deg += 360.0F;
  }

  return delta_deg;
}

static void HWT101_ProcessByte(uint8_t data)
{
  uint8_t checksum = 0U;
  uint8_t index;

  if (hwt101_frame_pos == 0U)
  {
    if (data == 0x55U)
    {
      hwt101_frame[0] = data;
      hwt101_frame_pos = 1U;
    }
    return;
  }

  hwt101_frame[hwt101_frame_pos++] = data;
  if (hwt101_frame_pos < sizeof(hwt101_frame))
  {
    return;
  }

  for (index = 0U; index < 10U; index++)
  {
    checksum = (uint8_t)(checksum + hwt101_frame[index]);
  }

  if (checksum == hwt101_frame[10])
  {
    if (hwt101_frame[1] == 0x52U)
    {
      hwt101_gyro_z_dps = ((float)HWT101_ReadLeInt16(&hwt101_frame[6]) * 2000.0F) / 32768.0F;
    }
    else if (hwt101_frame[1] == 0x53U)
    {
      float yaw_raw_deg = ((float)HWT101_ReadLeInt16(&hwt101_frame[6]) * 180.0F) / 32768.0F;
      float rate_dps = hwt101_gyro_z_dps;   /* 最近一帧 0x52 的角速度 */

      hwt101_yaw_deg = yaw_raw_deg;
      hwt101_sample_count++;

      if (hwt101_drift_ready == 0U)
      {
        /* ---- 上电自动零漂：静止窗口内取平均做基准与角速度零偏 ---- */
        uint8_t restart = 0U;

        if ((rate_dps > HWT101_CALIB_MAX_RATE_DPS) ||
            (rate_dps < -HWT101_CALIB_MAX_RATE_DPS))
        {
          /* 瞬时角速度超标：说明还在动 */
          restart = 1U;
        }
        else if ((hwt101_calib_active != 0U) &&
                 (((yaw_raw_deg - hwt101_calib_yaw_min) >
                   HWT101_CALIB_MAX_SPAN_DEG) ||
                  ((hwt101_calib_yaw_max - yaw_raw_deg) >
                   HWT101_CALIB_MAX_SPAN_DEG)))
        {
          /* 窗口内 yaw 晃超了：3 s 的窗口很长，光看瞬时角速度挡不住"慢慢转/被人轻挪" */
          restart = 1U;
        }
        else
        {
          /* 通过检查：继续累加 */
        }

        if (restart != 0U)
        {
          /* 丢弃窗口，重新开始计时（说明还没静下来） */
          hwt101_calib_active = 0U;
          hwt101_calib_count = 0U;
          hwt101_calib_yaw_sum = 0.0;
          hwt101_calib_gyro_sum = 0.0;
        }
        else
        {
          if (hwt101_calib_active == 0U)
          {
            hwt101_calib_active = 1U;
            hwt101_calib_start_tick = HAL_GetTick();
            hwt101_calib_count = 0U;
            hwt101_calib_yaw_sum = 0.0;
            hwt101_calib_gyro_sum = 0.0;
            hwt101_calib_yaw_min = yaw_raw_deg;
            hwt101_calib_yaw_max = yaw_raw_deg;
          }
          if (yaw_raw_deg < hwt101_calib_yaw_min)
          {
            hwt101_calib_yaw_min = yaw_raw_deg;
          }
          if (yaw_raw_deg > hwt101_calib_yaw_max)
          {
            hwt101_calib_yaw_max = yaw_raw_deg;
          }
          hwt101_calib_count++;
          hwt101_calib_yaw_sum += (double)yaw_raw_deg;
          hwt101_calib_gyro_sum += (double)rate_dps;

          if ((hwt101_calib_count >= (uint32_t)HWT101_CALIB_MIN_SAMPLES) &&
              ((HAL_GetTick() - hwt101_calib_start_tick) >= HWT101_CALIB_MS))
          {
            double count = (double)hwt101_calib_count;
            float yaw_avg = (float)(hwt101_calib_yaw_sum / count);

            hwt101_gyro_bias_dps =
                (float)(hwt101_calib_gyro_sum / count);   /* 角速度零偏 */
            hwt101_yaw_baseline_deg = yaw_avg;            /* 朝向基准 */
            hwt101_yaw_previous_deg = yaw_raw_deg;
            hwt101_zero_drift_deg = 0.0F;
            hwt101_drift_ready = 1U;
            hwt101_calib_active = 0U;
            hwt101_zero_ready_tick = HAL_GetTick();

            if (hwt101_calib_reported == 0U)
            {
              /* 只置标志，实际打印在任务上下文（HWT101_PollReport）里做 */
              hwt101_calib_report_samples = hwt101_calib_count;
              hwt101_calib_report_ms = HAL_GetTick() - hwt101_calib_start_tick;
              hwt101_calib_report_pending = 1U;
              hwt101_calib_reported = 1U;
            }
          }
        }
      }
      else
      {
        float delta_deg = HWT101_WrapAngleDeltaDeg(yaw_raw_deg -
                                                   hwt101_yaw_previous_deg);

        hwt101_zero_drift_deg += delta_deg;
        hwt101_yaw_previous_deg = yaw_raw_deg;
      }
    }
  }

  hwt101_frame_pos = 0U;
}

/**
  * @brief  USART10 DMA ring buffer data entry (called by vision.c RxEventCallback)
  */
void HWT101_OnDmaData(const uint8_t *data, uint16_t len)
{
  uint16_t index;

  for (index = 0U; index < len; index++)
  {
    HWT101_ProcessByte(data[index]);
  }
}

/* 其他任务通过只读接口获取姿态数据，避免直接访问 main.c 私有变量。 */
float HWT101_GetYawDeg(void)
{
  return (float)hwt101_yaw_deg;
}

float HWT101_GetGyroZDps(void)
{
  /* Keep the sampled value raw so the calibration window above averages the
     uncorrected rate. Apply the measured bias exactly once at this public
     interface; while a new calibration is in progress, do not reuse the
     previous window's bias. */
  float raw_dps = (float)hwt101_gyro_z_dps;
  float bias_dps = (hwt101_drift_ready != 0U) ?
                   (float)hwt101_gyro_bias_dps : 0.0F;

  return raw_dps - bias_dps;
}

uint32_t HWT101_GetSampleCount(void)
{
  return (uint32_t)hwt101_sample_count;
}

uint32_t HWT101_GetErrorCount(void)
{
  return (uint32_t)hwt101_rx_error_count;
}

float HWT101_GetZeroDriftDeg(void)
{
  return (float)hwt101_zero_drift_deg;
}

uint8_t HWT101_IsZeroDriftReady(void)
{
  return (uint8_t)hwt101_drift_ready;
}

/* 标定完成时刻（ms，0 = 还没完成过）；屏幕用它显示几秒钟的"去零漂完成"。 */
uint32_t HWT101_GetZeroDriftReadyTick(void)
{
  return (uint32_t)hwt101_zero_ready_tick;
}

/* 当前静止窗口已累计的毫秒数（窗口被"车在动"打断会归零）；屏幕用它显示倒计时。 */
uint32_t HWT101_GetCalibElapsedMs(void)
{
  if (hwt101_calib_active == 0U)
  {
    return 0U;
  }
  return (uint32_t)(HAL_GetTick() - hwt101_calib_start_tick);
}

/* 上电自动标定得到的角速度零偏（dps），HWT101_GetGyroZDps 已把它减掉。 */
float HWT101_GetGyroBiasDps(void)
{
  return (float)hwt101_gyro_bias_dps;
}

/* 零漂标定是否正在进行（=1 时别动车，否则标定会被重新计时）。 */
uint8_t HWT101_IsCalibrating(void)
{
  return (uint8_t)hwt101_calib_active;
}

/* 任务上下文里调用（ZDT_Motor_Poll）：把"标定完成"那一行打出去。
 * 放这里而不是放在 UART 空闲中断回调里，避免 ISR 里阻塞发串口。 */
void HWT101_PollReport(void)
{
  char line[128];

  if (hwt101_calib_report_pending == 0U)
  {
    return;
  }
  hwt101_calib_report_pending = 0U;

  (void)snprintf(line, sizeof(line),
                 "[IMU] zero-drift ready: %lu samples/%lums, gyro bias %+.3f dps\r\n",
                 (unsigned long)hwt101_calib_report_samples,
                 (unsigned long)hwt101_calib_report_ms,
                 (double)hwt101_gyro_bias_dps);
  ZDT_Console_Print(line);
}

/**
  * @brief  重新建立零漂基准：把当前朝向当作 0°。
  * @note   用于"上电后才把车摆到起点、朝向和上电时不一致"的现场情况。
  *         调用后必须**静止不动约 3 s**（HWT101_CALIB_MS），窗口满足后才会建立
  *         新基准并打印 [IMU] zero-drift ready（同时重新估计角速度零偏）。
  *         这 3 s 内屏幕会显示 `IMU CALIBRATING`，完成后显示 `IMU CAL DONE`。
  */
void HWT101_ResetZeroDrift(void)
{
  hwt101_drift_ready = 0U;
  hwt101_sample_count = 0U;
  hwt101_zero_drift_deg = 0.0F;
  hwt101_yaw_baseline_deg = 0.0F;
  hwt101_yaw_previous_deg = 0.0F;
  hwt101_calib_active = 0U;
  hwt101_calib_count = 0U;
  hwt101_calib_yaw_sum = 0.0;
  hwt101_calib_gyro_sum = 0.0;
  hwt101_calib_yaw_min = 0.0F;
  hwt101_calib_yaw_max = 0.0F;
  hwt101_calib_reported = 0U;
  hwt101_zero_ready_tick = 0U;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART10_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  Vision_Init();
  g_boot_stage = 13U;
  /* 达妙转盘电机（FDCAN1）的总线初始化已移到 MX_FREERTOS_Init() 里的
   * ArmTask_Init()：那里紧挨着任务创建，配置来源单一（arm_task.c 里的 dm_config_t），
   * 不再出现"main.c 传一套 ID、驱动里硬编码另一套"的分叉。
   * 2026-09-14 之前这里写的是 DM_Motor_Init(0x05U, 0x00U, DM_MIT_MODE)，
   * 而那三个参数在驱动里被 (void) 丢弃、实际按 0x01/0x11/POS 发帧 —— 已修。 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 64;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c1235ClockSelection = RCC_I2C1235CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  if (g_boot_error == 0U)
  {
    g_boot_error = 1U;
  }
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
