#ifndef CONSOLE_DMA_H
#define CONSOLE_DMA_H
#include "stm32h7xx_hal.h"
#include <stddef.h>
#define CONSOLE_DMA_QUEUE_BYTES 8192U
#define CONSOLE_DMA_CHUNK_BYTES 256U
#define CONSOLE_DMA_TEXT_LIMIT 4096U
typedef struct {
    uint32_t queued_bytes,high_water,posted_bytes,sent_bytes,dropped_bytes,dropped_calls;
    uint32_t tx_errors,tx_timeouts,tx_busy,aborts;
} console_dma_stats_t;
void ConsoleDma_Init(void);
uint8_t ConsoleDma_Write(const uint8_t *data,size_t length);
void ConsoleDma_Print(const char *text);
void ConsoleDma_Service(void); /* One logger owner only. */
void ConsoleDma_Run(void *argument);
void ConsoleDma_TxComplete(UART_HandleTypeDef *uart);
void ConsoleDma_TxError(UART_HandleTypeDef *uart);
void ConsoleDma_AbortComplete(UART_HandleTypeDef *uart);
void ConsoleDma_Uart4TxIrq(void);
void ConsoleDma_GetStats(console_dma_stats_t *out);
#endif
