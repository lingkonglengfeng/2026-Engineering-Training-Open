/* Producers copy bounded text only. One low-priority owner sends immutable
 * DMA chunks; no UART wait, heap allocation or task mutex in a producer. */
#include "console_dma.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#define QMASK (CONSOLE_DMA_QUEUE_BYTES-1U)
#define DONE 1U
#define ERROR 2U
#define ABORTED 4U
static volatile uint8_t s_queue[CONSOLE_DMA_QUEUE_BYTES];
static volatile uint32_t s_head,s_tail,s_events;
static volatile uint8_t s_active,s_aborting;
static volatile uint16_t s_length;
static volatile console_dma_stats_t s_stats;
static osThreadId_t volatile s_owner;
static uint8_t s_init,s_retry_wait;
static uint32_t s_started,s_deadline,s_retry;
#if defined(__CC_ARM)
static uint8_t s_stage[CONSOLE_DMA_CHUNK_BYTES] __attribute__((at(0x24010600)));
#else
static uint8_t s_stage[CONSOLE_DMA_CHUNK_BYTES] __attribute__((section(".console_dma"),aligned(32)));
#endif
static uint32_t lock(void){uint32_t k=__get_PRIMASK();__disable_irq();return k;}
static void unlock(uint32_t k){__DMB();__set_PRIMASK(k);}
static void wake(uint8_t irq_ok)
{if((irq_ok || !__get_IPSR()) && !__get_PRIMASK() && s_owner && osKernelGetState()==osKernelRunning)(void)osThreadFlagsSet(s_owner,1U);}
void ConsoleDma_Init(void){s_init=1U;} /* Keep already queued boot text. */
uint8_t ConsoleDma_Write(const uint8_t *data,size_t length)
{
    uint32_t k,used,i;
    if(!data || !length)return 0U;
    k=lock();used=s_head-s_tail;
    if(length>CONSOLE_DMA_TEXT_LIMIT || length>CONSOLE_DMA_QUEUE_BYTES-used ||
       (__get_IPSR() && length>160U)) {
        s_stats.dropped_calls++;s_stats.dropped_bytes+=(uint32_t)length;unlock(k);return 0U;
    }
    for(i=0;i<length;i++)s_queue[(s_head+i)&QMASK]=data[i];
    s_head+=(uint32_t)length;s_stats.posted_bytes+=(uint32_t)length;
    if(used+length>s_stats.high_water)s_stats.high_water=used+(uint32_t)length;
    unlock(k);wake(0U);return 1U;
}
void ConsoleDma_Print(const char *text)
{
    size_t n=0,limit=__get_IPSR()?160U:CONSOLE_DMA_TEXT_LIMIT;
    if(!text)return;
    while(n<=limit && text[n])n++;
    (void)ConsoleDma_Write((const uint8_t *)text,n);
}
void ConsoleDma_TxComplete(UART_HandleTypeDef *uart)
{if(uart==&huart4 && s_active && !s_aborting){s_events|=DONE;wake(1U);}}
void ConsoleDma_TxError(UART_HandleTypeDef *uart)
{if(uart==&huart4 && s_active){s_events|=ERROR;wake(1U);}}
void ConsoleDma_AbortComplete(UART_HandleTypeDef *uart)
{if(uart==&huart4 && s_aborting){s_events|=ABORTED;wake(1U);}}
void ConsoleDma_GetStats(console_dma_stats_t *out)
{uint32_t k;if(!out)return;k=lock();*out=s_stats;out->queued_bytes=s_head-s_tail;unlock(k);}
static uint8_t tx_idle(void)
{return huart4.gState==HAL_UART_STATE_READY && (!huart4.hdmatx || HAL_DMA_GetState(huart4.hdmatx)==HAL_DMA_STATE_READY);}
void ConsoleDma_Service(void)
{
    uint32_t k,events,tail,count,i,now=HAL_GetTick();HAL_StatusTypeDef result;
    if(!s_init)return;
    k=lock();events=s_events;unlock(k);
    if(s_aborting) {
        if((events&ABORTED) || tx_idle()) {
            k=lock();s_tail+=s_length;s_stats.dropped_bytes+=s_length;s_stats.aborts++;
            s_active=s_aborting=0U;s_length=0U;s_events=0U;unlock(k);
        }
        return; /* Never overwrite a buffer while DMA may still read it. */
    }
    if(s_active) {
        if((events&ERROR) || (!(events&DONE) && now-s_started>s_deadline)) {
            k=lock();s_aborting=1U;s_events=0U;
            if(events&ERROR)s_stats.tx_errors++;else s_stats.tx_timeouts++;
            unlock(k);(void)HAL_UART_AbortTransmit_IT(&huart4);return;
        }
        if(!(events&DONE))return;
        k=lock();s_tail+=s_length;s_stats.sent_bytes+=s_length;
        s_active=0U;s_length=0U;s_events=0U;unlock(k);
    }
    if(s_retry_wait && (int32_t)(now-s_retry)<0)return;
    s_retry_wait=0U;
    if(!huart4.hdmatx)return;
    if(!tx_idle()){s_stats.tx_busy++;return;}
    k=lock();tail=s_tail;count=s_head-tail;unlock(k);
    if(!count)return;
    if(count>CONSOLE_DMA_CHUNK_BYTES)count=CONSOLE_DMA_CHUNK_BYTES;
    for(i=0;i<count;i++)s_stage[i]=s_queue[(tail+i)&QMASK];
    /* Dedicated aligned AXI SRAM staging avoids DTCM and cache-line sharing. */
    if(SCB->CCR&SCB_CCR_DC_Msk)SCB_CleanDCache_by_Addr((uint32_t *)s_stage,CONSOLE_DMA_CHUNK_BYTES);
    k=lock();s_length=(uint16_t)count;s_active=1U;s_events=0U;unlock(k);
    s_started=now;s_deadline=huart4.Init.BaudRate?(count*10000U+huart4.Init.BaudRate-1U)/huart4.Init.BaudRate+100U:1000U;
    result=HAL_UART_Transmit_DMA(&huart4,s_stage,(uint16_t)count);
    if(result!=HAL_OK) {
        k=lock();s_active=0U;s_length=0U;s_events=0U;
        if(result==HAL_BUSY)s_stats.tx_busy++;else s_stats.tx_errors++;
        unlock(k);s_retry=now+10U;s_retry_wait=1U;
    }
}
void ConsoleDma_Run(void *argument)
{
    (void)argument;s_owner=osThreadGetId();
    for(;;){ConsoleDma_Service();(void)osThreadFlagsWait(1U,osFlagsWaitAny,5U);}
}
/* UART4 RX is owned by the existing raw-byte parser, not HAL RxISR. Mask RX
 * sources only around the HAL TX-complete handler to avoid a null RxISR call. */
void ConsoleDma_Uart4TxIrq(void)
{
    uint32_t cr1,cr3;
    if(!(UART4->ISR&USART_ISR_TC) || !(UART4->CR1&USART_CR1_TCIE))return;
    cr1=UART4->CR1&(USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE);cr3=UART4->CR3&USART_CR3_EIE;
    CLEAR_BIT(UART4->CR1,USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE);CLEAR_BIT(UART4->CR3,USART_CR3_EIE);
    HAL_UART_IRQHandler(&huart4);
    SET_BIT(UART4->CR1,cr1);SET_BIT(UART4->CR3,cr3);
}
