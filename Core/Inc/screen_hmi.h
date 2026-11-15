#ifndef SCREEN_HMI_H
#define SCREEN_HMI_H
#include "stm32h7xx_hal.h"
#include "screen_protocol.h"
#define SCREEN_UART_BAUD 9600U
void ScreenHmi_Init(void);
void ScreenHmi_Run(void *argument);
void ScreenHmi_Service(uint32_t now);
void ScreenHmi_RxComplete(UART_HandleTypeDef *uart);
void ScreenHmi_TxComplete(UART_HandleTypeDef *uart);
void ScreenHmi_UartError(UART_HandleTypeDef *uart);
/* Called only by ZDT owner; packet is consumed even if motion forbids apply.
 * No screen task calls the navigation code or sends a motor command. */
uint8_t ScreenHmi_TakeBatch(screen_batch_t *out);
void ScreenHmi_BatchResult(uint8_t accepted);
typedef struct {uint32_t rx,overflow,rejected,applied,busy_rejected,uart_errors,tx_errors;uint8_t page;} screen_hmi_stats_t;
void ScreenHmi_GetStats(screen_hmi_stats_t *out);
#endif
