#ifndef __ZDT_MOTOR_H__
#define __ZDT_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "vision_qr.h"
/* Atomic read-only snapshot; ArmTask consumes once at RAW1. */
uint8_t ZDT_Match_GetQrTask(vision_qr_task_t *task);

#define ZDT_CAN_MOTOR_ADDRESS       (3U)
#define ZDT_CAN_BROADCAST_ADDRESS   (0U)
#define ZDT_CAN_PACKET_INDEX        (0U)
#define ZDT_CAN_EXT_ID              ((ZDT_CAN_MOTOR_ADDRESS << 8U) | ZDT_CAN_PACKET_INDEX)
#define ZDT_CAN_CHECKSUM            (0x6BU)
#define ZDT_CAN_CMD_READ_VERSION    (0x1FU)
#define ZDT_CAN_RUN_MAX_RPM         (3000L)

/* 轮径 81.39 mm -> 周长 pi x 81.39 = 255.694 mm/圈。
 * 里程换算与导航"距离 -> 时间"换算都用它（nav_params.c 打印 mm/s 也用）。 */
#define ZDT_MILEAGE_MM_PER_REV      (255.694226f)

/* 0 = Emm firmware (default), 1 = X firmware speed-command layout. */
#ifndef ZDT_FIRMWARE_X
#define ZDT_FIRMWARE_X              (0U)
#endif

typedef struct
{
  volatile uint32_t tx_count;
  volatile uint32_t rx_count;
  volatile uint32_t rx_dropped;
  volatile uint32_t last_can_error;
  volatile uint32_t protocol_error_count;
  volatile uint8_t last_protocol_lec; /* Latched in ISR before PSR reads clear LEC. */
  volatile uint32_t uart_rx_irq;
  volatile uint32_t uart_rx_bytes;
  volatile uint32_t uart_rx_ore;
  volatile uint32_t uart_rx_fe;
  volatile uint32_t uart_rx_ne;
  volatile uint32_t uart_rx_pe;
  volatile uint32_t uart_rx_recover;
  volatile uint32_t uart_tx_failed;
  volatile uint8_t can_ready;
  volatile uint8_t motor_responded;
  volatile uint8_t bus_off;
} zdt_motor_status_t;

void ZDT_Motor_Begin(void);
void ZDT_Motor_Poll(void);
HAL_StatusTypeDef ZDT_Motor_Enable(void);
HAL_StatusTypeDef ZDT_Motor_Disable(void);
HAL_StatusTypeDef ZDT_Motor_Stop(void);
HAL_StatusTypeDef ZDT_Motor_SetSpeedRpm(int16_t rpm);

HAL_StatusTypeDef ZDT_Motor_EnableAddr(uint8_t address, uint8_t buffered);
HAL_StatusTypeDef ZDT_Motor_DisableAddr(uint8_t address, uint8_t buffered);
HAL_StatusTypeDef ZDT_Motor_StopAddr(uint8_t address);
HAL_StatusTypeDef ZDT_Motor_SetSpeedRpmAddr(uint8_t address, int16_t rpm,
                                            uint8_t buffered);
HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAddr(uint8_t address,
                                                   int32_t pulses,
                                                   uint16_t speed_rpm,
                                                   uint8_t acceleration,
                                                   uint8_t buffered);
/* FD mode 01: absolute driver coordinate, supported by legacy Emm firmware. */
HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAbsAddr(uint8_t address,
                                                   int32_t pulses,
                                                   uint16_t speed_rpm,
                                                   uint8_t acceleration,
                                                   uint8_t buffered);
HAL_StatusTypeDef ZDT_Motor_SyncTrigger(void);
/* Atomic packed F6 reply token: sequence in high24 bits, response in low8. */
uint32_t ZDT_Motor_SpeedAck(uint8_t address);
void ZDT_Motor_SetVerbose(uint8_t enabled);
const zdt_motor_status_t *ZDT_Motor_GetStatus(void);

/* 启停区状态（给屏幕显示用）。
 * 现场踩过的坑：屏幕上的黄点来自位姿，而位姿在 `&startpose 1|2` 之前是**默认区**的假定值
 * （`ZDT_START_ZONE_DEFAULT` = 2 号区 = 右上角）。默认区与实际摆放不一致时，屏幕看起来
 * 就像"坏了"，实际是"位姿还只是假定值"。所以屏幕要把这三件事直接显示出来：
 *   ZDT_Script_GetStartZone()      1（右下角）/ 2（右上角）
 *   ZDT_Script_IsStartPoseSet()    本次上电是否跑过 `&startpose`（0 = 位姿只是假定值） */
uint8_t ZDT_Script_GetStartZone(void);
uint8_t ZDT_Script_IsStartPoseSet(void);

void ZDT_Motor_RxFifo0MsgPending(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t interrupts);
void ZDT_Console_Uart4Irq(void);
void ZDT_Motor_Task(void *argument);

/**
  * @brief  上下文安全的调试串口打印（UART4 @115200 8N1）。
  * @note   中断里调用 → 只入队，由 ZDT_Motor_Poll 在任务里 flush，绝不阻塞 ISR；
  *         任务里调用 → 直接阻塞式发送（按实际波特率计算超时；115200下30字符
  *         约2.6ms）。所以**任务里只在错误/状态变化时调**，不要在 1 ms 循环里
  *         对每帧都打印。dm_motor.c 的达妙日志走的就是这条通道。
  */
void ZDT_Console_Print(const char *text);



#ifdef __cplusplus
}
#endif

#endif /* __ZDT_MOTOR_H__ */
