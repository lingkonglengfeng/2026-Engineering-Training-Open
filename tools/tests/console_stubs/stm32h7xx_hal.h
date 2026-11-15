#ifndef TEST_CONSOLE_HAL_H
#define TEST_CONSOLE_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef enum {HAL_DMA_STATE_READY,HAL_DMA_STATE_BUSY} HAL_DMA_StateTypeDef;
typedef struct {HAL_DMA_StateTypeDef State;} DMA_HandleTypeDef;
typedef struct {uint32_t BaudRate;} UART_InitTypeDef;
typedef struct {UART_InitTypeDef Init;uint32_t gState;DMA_HandleTypeDef *hdmatx;} UART_HandleTypeDef;
typedef struct {volatile uint32_t ISR,CR1,CR3;} USART_TypeDef;
typedef struct {uint32_t CCR;} SCB_Type;
extern USART_TypeDef test_uart4;
extern SCB_Type test_scb;
#define UART4 (&test_uart4)
#define SCB (&test_scb)
#define SCB_CCR_DC_Msk 1U
#define USART_ISR_TC 1U
#define USART_CR1_TCIE 1U
#define USART_CR1_RXNEIE_RXFNEIE 2U
#define USART_CR1_PEIE 4U
#define USART_CR3_EIE 8U
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define CLEAR_BIT(a,b) ((a)&=~(b))
#define SET_BIT(a,b) ((a)|=(b))
uint32_t __get_PRIMASK(void);
uint32_t __get_IPSR(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t k);
void __DMB(void);
uint32_t HAL_GetTick(void);
void SCB_CleanDCache_by_Addr(uint32_t *p,int32_t n);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *);
void HAL_UART_IRQHandler(UART_HandleTypeDef *);
#endif
