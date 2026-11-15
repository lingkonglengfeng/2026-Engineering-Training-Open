/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "zdt_motor.h"
#include "grid_nav.h"
#include "chassis_motor.h"
#include "arm_task.h"
#include "ips200.h"
#include "vision.h"
#include "screen_hmi.h"
#include "console_dma.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IPS200_TELEMETRY_ENABLE (1U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uintptr_t g_overflow_task;
volatile char g_overflow_task_name[configMAX_TASK_NAME_LEN];

osThreadId_t zdtMotorTaskHandle;
const osThreadAttr_t zdtMotorTask_attributes = {
  .name = "zdtMotorTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};

/* 机械臂任务（2026-09-14 新增）。
 *
 * 周期 10 ms / 优先级 Normal3（比 defaultTask 高、比 zdtMotorTask 低），
 * 与《FreeRTOS任务安排与设计》里 ActuatorTask 的定位一致。
 * 栈 2048 B：任务体只做 DM_Motor_Service + 一条 snprintf，
 * 但 dm_motor 的 `dm status` / `dm probe` 会走 192 字节的局部缓冲和
 * snprintf（ARMCC 下浮点格式化栈开销较大），所以按设计文档的 1536 B 上浮一档。
 * 首版跑起来后用 uxTaskGetStackHighWaterMark() 复核。 */
osThreadId_t armTaskHandle;
osThreadId_t visionTaskHandle;
static osThreadId_t screenTaskHandle;
static osThreadId_t consoleTaskHandle;
static const osThreadAttr_t consoleTask_attributes={.name="consoleDMA",.priority=osPriorityBelowNormal2,.stack_size=1024U};
static const osThreadAttr_t screenTask_attributes={.name="screenTask",.priority=osPriorityBelowNormal,.stack_size=1536U};
const osThreadAttr_t visionTask_attributes = {
  .name = "visionTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 2048
};
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 4096
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 2048 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#if (IPS200_TELEMETRY_ENABLE != 0U)
/* 左侧 128x128 像素地图区域，右侧保留原有 PID/速度信息。 */
#define IPS_MAP_X             (6U)
#define IPS_MAP_Y             (28U)
#define IPS_MAP_CELL          (16U)
#define IPS_MAP_PIXELS        (GRID_NAV_SIZE * IPS_MAP_CELL)
#define IPS_TELEMETRY_X       (150U)
/* 顶部右侧零漂标定状态的起点（x=180 起约 23 字符可用：320-180=140px / 6px） */
#define IPS_IMU_STATE_X       (180U)
static void ChassisTelemetry_Show(void);
static void Dashboard_DrawMap(void);
#endif

void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void);

/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  ConsoleDma_Init();
  consoleTaskHandle=osThreadNew(ConsoleDma_Run,NULL,&consoleTask_attributes);
  ScreenHmi_Init();
  screenTaskHandle=osThreadNew(ScreenHmi_Run,NULL,&screenTask_attributes);
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  zdtMotorTaskHandle = osThreadNew(ZDT_Motor_Task, NULL,
                                   &zdtMotorTask_attributes);
  /* 机械臂（达妙转盘）任务。
   * ArmTask_Init() 必须在调度器启动前跑：它内部要 HAL_FDCAN_Init/Start FDCAN1，
   * 放到任务里做会和底盘 FDCAN2 的初始化顺序纠缠，且首帧可能早于总线就绪。 */
  if (ArmTask_Init() != HAL_OK)
  {
    /* 总线没起来不致命（底盘还能跑），但一定要留痕：
     * g_boot_error 会被屏幕/日志看到，控制台也已有 [ARM] ERROR 一行。 */
    g_boot_error = 5U;
  }
  armTaskHandle = osThreadNew(ArmTask_Run, NULL, &armTask_attributes);
  visionTaskHandle = osThreadNew(Vision_TaskRun, NULL, &visionTask_attributes);
  if ((defaultTaskHandle == NULL) || (zdtMotorTaskHandle == NULL) ||
      (armTaskHandle == NULL) || (visionTaskHandle == NULL) || (screenTaskHandle == NULL) || (consoleTaskHandle == NULL))
  {
    g_boot_error = 3U;
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
#if (IPS200_TELEMETRY_ENABLE != 0U)
#if (IPS200_TELEMETRY_ENABLE != 0U)
/* 网格路径都是水平/垂直线段，直接画 3 像素粗线。 */
static void Dashboard_DrawPathSegment(uint16_t x0, uint16_t y0,
                                      uint16_t x1, uint16_t y1)
{
  if (y0 == y1)
  {
    uint16_t xmin = (x0 < x1) ? x0 : x1;
    uint16_t xmax = (x0 < x1) ? x1 : x0;
    (void)ips200_fill_rect(xmin, (uint16_t)(y0 - 1U),
                           xmax, (uint16_t)(y0 + 1U),
                           IPS200_COLOR_CYAN);
  }
  else
  {
    uint16_t ymin = (y0 < y1) ? y0 : y1;
    uint16_t ymax = (y0 < y1) ? y1 : y0;
    (void)ips200_fill_rect((uint16_t)(x0 - 1U), ymin,
                           (uint16_t)(x0 + 1U), ymax,
                           IPS200_COLOR_CYAN);
  }
}

/* 根据 GridNav 当前状态重绘：障碍红、路径青、小车黄、终点绿。 */
/* 屏幕左侧 128x128 的场地示意图。
 * 2026-09-12：栅格由 8x8@300mm 改成 48x48@50mm 后，**不能按格画了**
 * （48 格 × 9 px = 432 px 超屏宽），改成按"分区几何"画示意图：
 *   底色 = 可走区（十字通道 + 外环带），深红 = 四块黄色禁区，
 *   蓝 = 两个启停区，青 = 边缘工位，黄 = 小车，绿 = 目标。
 * 坐标换算：规则图 mm（原点左下、y 向上）→ 屏幕像素（y 向下）。 */
#define IPS_ARENA_MM          ((uint16_t)GRID_NAV_FIELD_MM)
#define IPS_ARENA_PX          (128U)
/* 屏幕上最多逐个画多少个运行期障碍（再多就只留一个角落提示点，防止面板刷屏卡顿）。 */
#define IPS_MAP_MAX_OBSTACLE_MARKS  (48U)

static uint16_t Dashboard_MmToPxX(uint16_t mm_x)
{
  return (uint16_t)(IPS_MAP_X + ((uint32_t)mm_x * IPS_ARENA_PX) / IPS_ARENA_MM);
}

static uint16_t Dashboard_MmToPxY(uint16_t mm_y)
{
  /* y 向上 → 屏幕 y 向下，所以这里翻转 */
  return (uint16_t)(IPS_MAP_Y + ((uint32_t)(IPS_ARENA_MM - mm_y) *
                                 IPS_ARENA_PX) / IPS_ARENA_MM);
}

static void Dashboard_FillMm(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                             uint16_t color)
{
  (void)ips200_fill_rect(Dashboard_MmToPxX(x0), Dashboard_MmToPxY(y1),
                         Dashboard_MmToPxX(x1), Dashboard_MmToPxY(y0), color);
}

static void Dashboard_DrawMap(void)
{
  uint16_t pose_x = 0U;
  uint16_t pose_y = 0U;
  uint16_t goal_x = 0U;
  uint16_t goal_y = 0U;
  grid_nav_segment_t segments[24];
  uint16_t segment_count = 0U;
  uint8_t has_plan = 0U;
  uint16_t i;

  /* 底色：整个场地先按"可走"铺满（十字 + 外环带合起来覆盖全场地）。 */
  Dashboard_FillMm(0U, 0U, 2400U, 2400U, IPS200_COLOR_GRAY);

  /* 四块 450x450 黄色禁区（不可通行）。 */
  Dashboard_FillMm(550U, 1400U, 1000U, 1850U, IPS200_COLOR_RED);
  Dashboard_FillMm(1400U, 1400U, 1850U, 1850U, IPS200_COLOR_RED);
  Dashboard_FillMm(550U, 550U, 1000U, 1000U, IPS200_COLOR_RED);
  Dashboard_FillMm(1400U, 550U, 1850U, 1000U, IPS200_COLOR_RED);

  /* 两个启停区（300x300）。 */
  Dashboard_FillMm(2100U, 2100U, 2400U, 2400U, IPS200_COLOR_BLUE); /* 启停区2 右上角 */
  Dashboard_FillMm(2100U, 0U, 2400U, 300U, IPS200_COLOR_BLUE);   /* 启停区1 右下角 */

  /* 边缘工位：原料区(上)/粗加工区(下)/暂存区(左)/二维码板(右)。 */
  Dashboard_FillMm(1000U, 2250U, 1400U, 2400U, IPS200_COLOR_CYAN);
  Dashboard_FillMm(1000U, 0U, 1400U, 150U, IPS200_COLOR_CYAN);
  Dashboard_FillMm(0U, 1050U, 150U, 1350U, IPS200_COLOR_CYAN);
  Dashboard_FillMm(2350U, 1100U, 2400U, 1300U, IPS200_COLOR_CYAN);

  /* 运行期障碍（`&obs` / `&free` 设置的）：**按真实格位画**成品红小方块。
   *
   * 2026-09-13 修：原来这里只要"存在至少一个障碍"就在地图**左上角**点一个 5×5 的品红点，
   * 从来不画障碍到底在哪 —— 现场 `&obs 1200 1200` 之后屏幕上除了角落那个点以外什么也看不出来，
   * 会以为"指令没生效"。
   * 现在每个障碍格画 3×3 px（50 mm 格在 128 px 示意图上约 2.7 px，3 px 刚好可见），
   * 并在图例下方标出个数，一眼能看出"设了几个、在哪"。 */
  {
    uint8_t x;
    uint8_t y;
    uint16_t drawn = 0U;
    uint8_t capped = 0U;

    for (y = 0U; y < GRID_NAV_SIZE; ++y)
    {
      for (x = 0U; x < GRID_NAV_SIZE; ++x)
      {
        uint16_t cx;
        uint16_t cy;

        if (GridNav_IsObstacle(x, y) == 0U)
        {
          continue;
        }
        if (drawn >= IPS_MAP_MAX_OBSTACLE_MARKS)
        {
          /* 障碍太多就不逐个画（面板 SPI 逐像素推，画多了卡屏），
             只留左上角一个提示点，个数仍由图例那行给出（带 + 号表示"还有更多"）。 */
          (void)ips200_fill_rect(IPS_MAP_X, IPS_MAP_Y,
                                 (uint16_t)(IPS_MAP_X + 5U),
                                 (uint16_t)(IPS_MAP_Y + 5U),
                                 IPS200_COLOR_MAGENTA);
          capped = 1U;
          y = GRID_NAV_SIZE;
          break;
        }
        cx = Dashboard_MmToPxX(GridNav_CellCenterMmX(x));
        cy = Dashboard_MmToPxY(GridNav_CellCenterMmY(y));
        (void)ips200_fill_rect((uint16_t)(cx - 1U), (uint16_t)(cy - 1U),
                               (uint16_t)(cx + 1U), (uint16_t)(cy + 1U),
                               IPS200_COLOR_MAGENTA);
        drawn++;
      }
    }
    {
      char ob_text[16];
      char ob_line[16];
      int ob_len;

      if (drawn == 0U)
      {
        ob_text[0] = '\0';
      }
      else
      {
        ob_len = snprintf(ob_text, sizeof(ob_text), "OBS %u", (unsigned int)drawn);
        if ((capped != 0U) && (ob_len > 0) && (ob_len < (int)sizeof(ob_text) - 1))
        {
          /* 画到上限了：标成 "OBS 48+"，不要让人以为场上真的只有 48 个。 */
          ob_text[ob_len] = '+';
          ob_text[ob_len + 1] = '\0';
        }
      }

      /* 定宽输出（右侧空格补齐）：个数从 12 变 3、或 `&clear` 清零之后，
         残留的旧字符会被空格连同黑底一起盖掉，不会留在屏上。 */
      (void)snprintf(ob_line, sizeof(ob_line), "%-8s", ob_text);
      (void)ips200_draw_text(6U, (uint16_t)(IPS_MAP_Y + IPS_ARENA_PX + 3U),
                             ob_line, IPS200_COLOR_MAGENTA, IPS200_COLOR_BLACK);
    }
  }

  /* 规划路径：用**段**画（段数很少），不用 2304 点的路径数组（栈放不下）。
     GridNav_GetSegments() 的返回值同时表示"到底规划过没有"，下面画目标点用同一个判据。 */
  has_plan = GridNav_GetSegments(segments, 24U, &segment_count);
  if (has_plan != 0U)
  {
    for (i = 0U; i < segment_count; ++i)
    {
      uint16_t x0 = GridNav_CellCenterMmX(segments[i].from_x);
      uint16_t y0 = GridNav_CellCenterMmY(segments[i].from_y);
      uint16_t x1 = GridNav_CellCenterMmX(segments[i].to_x);
      uint16_t y1 = GridNav_CellCenterMmY(segments[i].to_y);

      Dashboard_DrawPathSegment(Dashboard_MmToPxX(x0), Dashboard_MmToPxY(y0),
                                Dashboard_MmToPxX(x1), Dashboard_MmToPxY(y1));
    }
  }

  GridNav_GetPoseMm(&pose_x, &pose_y);
  (void)ips200_fill_rect((uint16_t)(Dashboard_MmToPxX(pose_x) - 3U),
                         (uint16_t)(Dashboard_MmToPxY(pose_y) - 3U),
                         (uint16_t)(Dashboard_MmToPxX(pose_x) + 3U),
                         (uint16_t)(Dashboard_MmToPxY(pose_y) + 3U),
                         IPS200_COLOR_YELLOW);

  /* 目标点只在**真的规划过**之后才画：GridNav_Init() 会把 last_target 预置成 (2250,2250)，
     无条件画的话开机会凭空出现一个"目标"绿点（现场看着就像屏幕显示错了）。 */
  if (has_plan != 0U)
  {
    GridNav_GetTargetMm(&goal_x, &goal_y);
    (void)ips200_fill_rect((uint16_t)(Dashboard_MmToPxX(goal_x) - 2U),
                           (uint16_t)(Dashboard_MmToPxY(goal_y) - 2U),
                           (uint16_t)(Dashboard_MmToPxX(goal_x) + 2U),
                           (uint16_t)(Dashboard_MmToPxY(goal_y) + 2U),
                           IPS200_COLOR_GREEN);
  }

  /* 外框：⚠ ips200_fill_rect() 是**填充**矩形（不是描边），所以边框必须用 4 条
     1 px 细线来画。这里踩过一次坑：原来写成一次 fill_rect(整片区域, GRAY)，
     等于画完分区后又把整张地图刷成灰色（面板 SPI 逐像素推，所以先闪一下正常地图
     再全灰）。 */
  (void)ips200_fill_rect(IPS_MAP_X, IPS_MAP_Y,
                         (uint16_t)(IPS_MAP_X + IPS_ARENA_PX), IPS_MAP_Y,
                         IPS200_COLOR_WHITE);
  (void)ips200_fill_rect(IPS_MAP_X, (uint16_t)(IPS_MAP_Y + IPS_ARENA_PX),
                         (uint16_t)(IPS_MAP_X + IPS_ARENA_PX),
                         (uint16_t)(IPS_MAP_Y + IPS_ARENA_PX),
                         IPS200_COLOR_WHITE);
  (void)ips200_fill_rect(IPS_MAP_X, IPS_MAP_Y, IPS_MAP_X,
                         (uint16_t)(IPS_MAP_Y + IPS_ARENA_PX),
                         IPS200_COLOR_WHITE);
  (void)ips200_fill_rect((uint16_t)(IPS_MAP_X + IPS_ARENA_PX), IPS_MAP_Y,
                         (uint16_t)(IPS_MAP_X + IPS_ARENA_PX),
                         (uint16_t)(IPS_MAP_Y + IPS_ARENA_PX),
                         IPS200_COLOR_WHITE);
}
#endif

static void ChassisTelemetry_Show(void)
{
  chassis_telemetry_t t;
  int16_t wheel_rpm[CHASSIS_MOTOR_COUNT];
  float kp;
  float ki;
  float kd;
  static uint8_t layout_ready;
  static uint32_t cached_map_revision = 0xFFFFFFFFU;
  static char cached[8][64];
  static char cached_imu_state[24];
  static char cached_zone[28];
  static uint16_t cached_imu_color;
  char lines[8][64];
  const char *imu_state;
  uint16_t imu_color;
  uint32_t imu_ready_tick;
  const uint16_t line_y[8] = { 28U, 46U, 64U, 82U, 100U, 124U, 146U, 164U };
  const uint16_t line_color[8] =
  {
    IPS200_COLOR_YELLOW, IPS200_COLOR_WHITE, IPS200_COLOR_WHITE,
    IPS200_COLOR_GREEN, IPS200_COLOR_GREEN, IPS200_COLOR_MAGENTA,
    IPS200_COLOR_WHITE, IPS200_COLOR_WHITE
  };
  uint32_t i;

  ChassisMotor_GetTelemetry(&t);
  ChassisMotor_GetWheelRpm(wheel_rpm);
  ChassisMotor_GetTurnGains(&kp, &ki, &kd);

  /* ANG / PID 误差都显示到 0.01°：HWT101 的 yaw 分辨率是 1 LSB ≈ 0.0055°，
     所以 0.01° 已经是"看得见的最小变化"（原来只到 0.1° 会把 0.05° 级别的
     残余误差全显示成同一个数）。GYRO 保持 0.1 deg/s —— 它的 1 LSB 就是 0.061 dps。 */
  (void)snprintf(lines[0], sizeof(lines[0]), "ANG %+8.2f deg",
                 (double)t.heading_deg);
  (void)snprintf(lines[1], sizeof(lines[1]), "GYRO %+7.1f deg/s",
                 (double)t.gyro_dps);
  (void)snprintf(lines[2], sizeof(lines[2]), "VX %+4d%% VY %+4d%% W %+4d%%",
                 (int)(t.vx * 100.0f), (int)(t.vy * 100.0f),
                 (int)(t.omega * 100.0f));
  (void)snprintf(lines[3], sizeof(lines[3]), "RPM1 %4d  RPM2 %4d",
                 (int)wheel_rpm[CHASSIS_WHEEL_FL],
                 (int)wheel_rpm[CHASSIS_WHEEL_RL]);
  (void)snprintf(lines[4], sizeof(lines[4]), "RPM3 %4d  RPM4 %4d",
                 (int)wheel_rpm[CHASSIS_WHEEL_FR],
                 (int)wheel_rpm[CHASSIS_WHEEL_RR]);
  (void)snprintf(lines[5], sizeof(lines[5]), "PID e %+7.2f out %+5.2f",
                 (double)t.pid_error_deg, (double)t.pid_output);
  (void)snprintf(lines[6], sizeof(lines[6]), "KP %.4f KI %.4f",
                 (double)kp, (double)ki);
  (void)snprintf(lines[7], sizeof(lines[7]), "KD %.4f  %s",
                 (double)kd, (t.turn_active != 0U) ? "TURN" : "IDLE");

  if (layout_ready == 0U)
  {
    (void)ips200_fill_screen(IPS200_COLOR_BLACK);
    (void)ips200_draw_text(8U, 8U, "NAV MAP", IPS200_COLOR_CYAN,
                           IPS200_COLOR_BLACK);
    (void)ips200_draw_text(150U, 8U, "PID", IPS200_COLOR_CYAN,
                           IPS200_COLOR_BLACK);
    (void)ips200_draw_text(8U, 188U, "Y CAR  G GOAL  M OBS  C PATH",
                           IPS200_COLOR_WHITE, IPS200_COLOR_BLACK);
    memset(cached, 0, sizeof(cached));
    cached_zone[0] = '\0';
    Dashboard_DrawMap();
    cached_map_revision = GridNav_GetRevision();
    layout_ready = 1U;
  }

  /* ---- 地图下方一行：固件"以为"的启停区 + 位姿数字 ----
     这一行是给现场判"屏幕对不对"用的：黄点画在哪里完全由位姿决定，而位姿的来源有两种 ——
       `&startpose N`（人确认过）或**开机默认区**（ZDT_START_ZONE_DEFAULT，只是假定值）。
     所以这里把三种状态分开显示：
       ZONE 2  POSE 2250,2250   绿色 = 已发过 `&startpose`，位姿可信
       ZONE 2? POSE 2250,2250   黄色 = **开机假定值**，人还没确认，发车前请发 &startpose 2
       ZONE ?  POSE 0,0         红色 = 连默认区都没装好（不该出现，出现就是初始化出错）
     现场判据：这一行必须与车实际所在的区一致，否则发 `&leave`/`&goto` 一定出事。
     位置放在地图下沿(156)与图例行(188)之间；右半边 x>=150 是遥测文字，不能压过去。 */
  {
    char zone_line[28];
    uint16_t px = 0U;
    uint16_t py = 0U;
    uint16_t zone_color;

    GridNav_GetPoseMm(&px, &py);
    if (ZDT_Script_IsStartPoseSet() != 0U)
    {
      (void)snprintf(zone_line, sizeof(zone_line), "ZONE %u  POSE %u,%u",
                     (unsigned int)ZDT_Script_GetStartZone(),
                     (unsigned int)px, (unsigned int)py);
      zone_color = IPS200_COLOR_GREEN;
    }
    else if ((px != 0U) || (py != 0U))
    {
      (void)snprintf(zone_line, sizeof(zone_line), "ZONE %u? POSE %u,%u",
                     (unsigned int)ZDT_Script_GetStartZone(),
                     (unsigned int)px, (unsigned int)py);
      zone_color = IPS200_COLOR_YELLOW;
    }
    else
    {
      (void)snprintf(zone_line, sizeof(zone_line), "ZONE ?  POSE 0,0");
      zone_color = IPS200_COLOR_RED;
    }
    if (strcmp(cached_zone, zone_line) != 0)
    {
      (void)ips200_fill_rect(6U, 176U, 148U, 185U, IPS200_COLOR_BLACK);
      (void)ips200_draw_text(6U, 176U, zone_line, zone_color,
                             IPS200_COLOR_BLACK);
      (void)strncpy(cached_zone, zone_line, sizeof(cached_zone) - 1U);
      cached_zone[sizeof(cached_zone) - 1U] = '\0';
    }
  }

  if (cached_map_revision != GridNav_GetRevision())
  {
    Dashboard_DrawMap();
    cached_map_revision = GridNav_GetRevision();
  }

  /* ---- 顶部右侧：零漂（去零飘）标定状态 ----
     还没标定完 → `IMU ZERO CAL 2.4s`（黄色，倒计时提示"保持静止"；
                  计数被打断归零说明车还在动）
     完成后 5 s 内 → `IMU ZERO DONE`（绿色），之后清空
     3 s 的静止窗口见 main.c 的 HWT101_CALIB_MS / HWT101_CALIB_MAX_SPAN_DEG。 */
  {
    char imu_line[24];
    uint32_t imu_elapsed;

    imu_ready_tick = HWT101_GetZeroDriftReadyTick();
    if (HWT101_IsZeroDriftReady() == 0U)
    {
      imu_elapsed = HWT101_GetCalibElapsedMs();
      (void)snprintf(imu_line, sizeof(imu_line), "IMU ZERO CAL %lu.%lus",
                     (unsigned long)(imu_elapsed / 1000U),
                     (unsigned long)((imu_elapsed % 1000U) / 100U));
      imu_state = imu_line;
      imu_color = IPS200_COLOR_YELLOW;
    }
    else if ((imu_ready_tick != 0U) &&
             ((HAL_GetTick() - imu_ready_tick) < 5000U))
    {
      imu_state = "IMU ZERO DONE";
      imu_color = IPS200_COLOR_GREEN;
    }
    else
    {
      imu_state = "";
      imu_color = IPS200_COLOR_BLACK;
    }
  }
  if ((strcmp(cached_imu_state, imu_state) != 0) ||
      (cached_imu_color != imu_color))
  {
    (void)ips200_fill_rect(IPS_IMU_STATE_X, 8U,
                           (uint16_t)(ips200_get_width() - 1U), 18U,
                           IPS200_COLOR_BLACK);
    if (imu_state[0] != '\0')
    {
      (void)ips200_draw_text(IPS_IMU_STATE_X, 8U, imu_state, imu_color,
                             IPS200_COLOR_BLACK);
    }
    (void)strncpy(cached_imu_state, imu_state,
                  sizeof(cached_imu_state) - 1U);
    cached_imu_state[sizeof(cached_imu_state) - 1U] = '\0';
    cached_imu_color = imu_color;
  }

  for (i = 0U; i < 8U; ++i)
  {
    if (strcmp(cached[i], lines[i]) != 0)
    {
      (void)ips200_fill_rect(IPS_TELEMETRY_X, line_y[i],
                             (uint16_t)(ips200_get_width() - 1U),
                             (uint16_t)(line_y[i] + 10U),
                             IPS200_COLOR_BLACK);
      (void)ips200_draw_text(IPS_TELEMETRY_X, line_y[i], lines[i],
                             line_color[i], IPS200_COLOR_BLACK);
      (void)strncpy(cached[i], lines[i], sizeof(cached[i]) - 1U);
      cached[i][sizeof(cached[i]) - 1U] = '\0';
    }
  }
}
#endif

void StartDefaultTask(void *argument)
{
  (void)argument;
  g_boot_stage = 17U;
  for(;;)
  {
#if (IPS200_TELEMETRY_ENABLE != 0U)
    static uint8_t display_ready = 0U;
    if (display_ready == 0U)
    {
      display_ready = (ips200_init(IPS200_ROTATION_LANDSCAPE) == HAL_OK) ? 1U : 0U;
    }
    if (display_ready != 0U)
    {
      ChassisTelemetry_Show();
    }
    osDelay(50U);
#else
    osDelay(1000U);
#endif
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
