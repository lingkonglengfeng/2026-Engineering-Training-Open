

/**
  ******************************************************************************
  * @file    vision.h
  * @brief   视觉串口协议解析 + DMA 接收管理（USART1/USART2 视觉帧、USART10 HWT101）
  * @note    默认相机协议：6B TYPE META D0 D1 D2 D3 CRC8，固定8字节。
  *          MAT_PX(02)/RING_PX(03)/PLATE_PX(09) 使用图像像素，右/下正，
  *          小端 int16×10；按相机端口独立发布，不写入旧毫米圆环接口。
  *          原 ASCII 协议仅在显式选择 VISION_PROTOCOL_ASCII 后启用：
  *          `@TYPE,field=value,...,seq=NN*CS\r\n`
  *          - 最大 128 字节，超长整帧丢弃并计错；
  *          - CS = '@' 后到 '*' 前所有字节的 XOR-8，两位大写十六进制；
  *          - 必须四类帧：QR / RING / COLOR / OBS。
  *
  *          接收链路（CubeMX 已配 4 路 Circular RX DMA）：
  *          - USART1/2：循环 DMA + IDLE/HT/TC，ISR复制到有界接收环，
  *            VisionTask解析相机数据；ISR不解析相机协议或打印；
  *          - USART10：同样方式接收 HWT101 字节流，转交 main.c 的
  *            `HWT101_OnDmaData()` 逐字节解析；
  *          - DMA 缓冲区定位在 AXI SRAM（0x24010000起，DMA1可访问）；
  *            工程未开启 D-Cache，无需缓存维护。
  *
  *          使用：main 初始化处调用 `Vision_Init()` 一次；
  *          之后任务通过 Vision_GetQR/Ring/Color/Obs 读取最新数据与数据年龄。
  ******************************************************************************
  */

#ifndef __VISION_H__
#define __VISION_H__

#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "vision_pixel.h"
#include "vision_camera.h"
#include "vision_qr.h"

#define VISION_PROTOCOL_BINARY8 1U
#define VISION_PROTOCOL_ASCII   2U
#ifndef VISION_CAMERA1_PROTOCOL
#define VISION_CAMERA1_PROTOCOL VISION_PROTOCOL_BINARY8
#endif
#ifndef VISION_CAMERA2_PROTOCOL
#define VISION_CAMERA2_PROTOCOL VISION_PROTOCOL_BINARY8
#endif

/* Task context. Default is binary8 on both ports; never auto-detect ASCII.
 * Changing protocol flushes queued bytes and invalidates that port's pixels. */
uint8_t Vision_SetCameraProtocol(uint8_t camera, uint8_t protocol);
/* Return 1 if a snapshot exists, including VALID=0. Check out->valid.
 * Stale (>=200ms), dropped/context-changed or explicitly invalid data is zeroed.
 * Identical valid SEQ duplicates within 50ms do not refresh receive age. */
uint8_t Vision_GetPixel(uint8_t camera, uint8_t type, vision_pixel_t *out, uint32_t *age_ms_out);
/* CAM2 wireless obstacle observation, independent of STOP/QR camera modes.
 * Age includes the camera-side capture-to-UART age plus MCU RX queue age. */
typedef struct {
    uint32_t observation_id,rx_tick;
    uint16_t front_gap_mm,bridge_age_ms;
    int16_t body_right_mm;
} vision_pc_obstacle_t;
uint8_t Vision_GetPcObstacle(vision_pc_obstacle_t *out,uint32_t *age_ms_out);
void Vision_ClearPcObstacle(void);
/* Bounded200..500ms opt-in profile window; context/transport faults still
 * invalidate immediately. Legacy getters retain200ms. */
uint8_t Vision_GetPixelWindow(uint8_t camera,uint8_t type,uint16_t window_ms,
                             vision_pixel_t *out,uint32_t *age_ms_out);
uint8_t Vision_GetRingSceneWindow(uint8_t camera,uint16_t window_ms,vision_pixel_t *pixel,
                                vision_ring_count_t *count,uint32_t *age_ms_out);
uint8_t Vision_GetRingAxisWindow(uint8_t camera,uint16_t window_ms,vision_ring_axis_t *axis,uint32_t *age_ms_out);
uint8_t Vision_GetPlateSceneWindow(uint8_t camera,uint16_t window_ms,vision_pixel_t *pixel,
                                 vision_plate_scene_t *scene,uint32_t *age_ms_out);
/* Clear observations/queued RX at a station boundary, preserving mode requests
 * and ACK/heartbeat state. Does not reconfigure the camera protocol. */
uint8_t Vision_ClearPixelObservations(uint8_t camera);

/* Task context. HAL_OK=queued, not ACKed. One pending SET per camera.
 * VisionTask sends with interrupt TX, 500ms timeout and at most 2 retries.
 * target uses engineering IDs (5 lightblue/6 black), station=0..6.
 * STOP: hz/target=0; RING: target=0/2; PLATE: target=0.
 * RING target2/station2..3: bit0=axis, bit1=occupied recovery.
 * CAM1 MATERIAL/station1: flags0x04=interactive color calibration, target0..6; other modes flags0. */
HAL_StatusTypeDef Vision_RequestMode(uint8_t camera, uint8_t mode, uint8_t station,
                                    uint8_t target, uint8_t hz, uint8_t flags);
uint8_t Vision_GetCameraState(uint8_t camera, vision_camera_state_t *out);
/* CAM2 only, current confirmed QR request; copied before STOP to match latch. */
uint8_t Vision_GetQrTask(vision_qr_task_t *out,uint32_t *age_ms_out);
/* Task-only: invalidate immediately, queue STOP safely behind active UART TX. */
HAL_StatusTypeDef Vision_RequestStop(uint8_t camera);
uint8_t Vision_GetRingScene(uint8_t camera,vision_pixel_t *pixel,
                            vision_ring_count_t *count,uint32_t *age);

/* 二维码任务码（如 "156+123+516+231"，规范化字符串 ≤15 字符 + 结束符） */
typedef struct
{
    uint16_t seq;
    char digits[16];
} vision_qr_t;

/* 圆环相对坐标帧 */
typedef struct
{
    uint16_t seq;
    uint8_t id;
    float x_mm;
    float y_mm;
    float theta_rad;
    float conf;
} vision_ring_t;

/* 物料颜色帧 */
typedef struct
{
    uint16_t seq;
    uint8_t id;
    uint8_t color;
    float conf;
} vision_color_t;

/* 障碍物距离帧（9999 表示无效测距，以 valid 为准） */
typedef struct
{
    uint16_t seq;
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;
    uint8_t valid;
} vision_obs_t;

/**
  * @brief  启动视觉与姿态串口的 DMA 循环接收（USART1/2/10 各一路）
  * @note   重复调用直接返回初始状态；相机失败由VisionTask重试，仍启动IMU。
  */
HAL_StatusTypeDef Vision_Init(void);
void Vision_TaskRun(void *argument);
void Vision_Service(void); /* Task context only; also supports deterministic tests. */

typedef struct {
    uint32_t rx_bytes, dropped_bytes, uart_errors, recoveries, last_rx_tick;
    uint16_t queued_bytes;
    uint8_t init_status,protocol;
} vision_port_stats_t;
uint8_t Vision_GetPortStats(uint8_t camera, vision_port_stats_t *out);
uint8_t Vision_ProcessCommand(const char *command);
/* Call from the console owner, with motion_busy=1 while any motion is pending. */
void Vision_DebugPoll(uint8_t motion_busy);

/**
  * @brief  IDLE 断帧检查：由 it.c 各 USART 中断的 USER CODE 区调用
  * @note   相机只复制新字节并通知任务，USART10保留原IMU接收路径。
  */
void Vision_CheckIdle(UART_HandleTypeDef *huart);

/**
  * @brief  读取最新 QR 任务码
  * @param  digits_out 规范化任务码字符串缓冲（≥16 字节）
  * @param  seq_out    帧序号（可 NULL）
  * @param  age_ms_out 数据年龄 ms（自收到至今，可 NULL）
  * @retval 1 有效数据；0 从未收到过
  */
uint8_t Vision_GetQR(char digits_out[16], uint16_t *seq_out, uint32_t *age_ms_out);

/**
  * @brief  读取最新圆环帧
  */
uint8_t Vision_GetRing(vision_ring_t *out, uint32_t *age_ms_out);

/**
  * @brief  读取最新颜色帧
  */
uint8_t Vision_GetColor(vision_color_t *out, uint32_t *age_ms_out);

/**
  * @brief  读取最新障碍物帧
  */
uint8_t Vision_GetObs(vision_obs_t *out, uint32_t *age_ms_out);

/**
  * @brief  累计解析失败帧数（校验错/超长/未知类型），用于链路健康监测
  */
uint32_t Vision_GetParseErrorCount(void);

#endif /* __VISION_H__ */
