#include "console_dma.h"
#include "cmsis_os2.h"
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
USART_TypeDef test_uart4;
SCB_Type test_scb;
UART_HandleTypeDef huart4;
static DMA_HandleTypeDef dma;
static uint32_t tick=1000,mask,ipsr,calls,cache_calls,aborts;
static uint8_t *pointer,snapshot[256],output[20000];
static uint32_t used;
static uint16_t length;
static HAL_StatusTypeDef result=HAL_OK;
uint32_t HAL_GetTick(void){return tick;}
uint32_t __get_PRIMASK(void){return mask;}
uint32_t __get_IPSR(void){return ipsr;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t k){mask=k;}
void __DMB(void){}
osKernelState_t osKernelGetState(void){return osKernelRunning;}
uint32_t osThreadFlagsSet(osThreadId_t t,uint32_t v){(void)t;CHECK(!mask);return v;}
uint32_t osThreadFlagsWait(uint32_t f,uint32_t o,uint32_t ms){(void)o;tick+=ms;return f;}
osThreadId_t osThreadGetId(void){return (void *)1;}
void SCB_CleanDCache_by_Addr(uint32_t *p,int32_t n){CHECK(!((uintptr_t)p&31U) && n==256);cache_calls++;}
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *d){return d->State;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
    CHECK(u==&huart4 && n && n<=256 && !ipsr && !mask);calls++;
    if(result!=HAL_OK)return result;
    pointer=p;length=n;memcpy(snapshot,p,n);u->gState=HAL_UART_STATE_BUSY_TX;dma.State=HAL_DMA_STATE_BUSY;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *u){CHECK(u==&huart4);aborts++;return HAL_OK;}
void HAL_UART_IRQHandler(UART_HandleTypeDef *u)
{
    CHECK(!(UART4->CR1&(USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE)) && !(UART4->CR3&USART_CR3_EIE));
    CLEAR_BIT(UART4->CR1,USART_CR1_TCIE);u->gState=HAL_UART_STATE_READY;dma.State=HAL_DMA_STATE_READY;ConsoleDma_TxComplete(u);
}
static void complete(void)
{
    CHECK(!memcmp(pointer,snapshot,length));memcpy(output+used,pointer,length);used+=length;
    huart4.gState=HAL_UART_STATE_READY;dma.State=HAL_DMA_STATE_READY;ConsoleDma_TxComplete(&huart4);ConsoleDma_Service();
}
static void drain(void)
{
    console_dma_stats_t q;unsigned n;
    for(n=0;n<100;n++){
        ConsoleDma_GetStats(&q);if(!q.queued_bytes)return;
        if(huart4.gState==HAL_UART_STATE_READY)ConsoleDma_Service();
        if(huart4.gState==HAL_UART_STATE_BUSY_TX)complete();
    }
    CHECK(0);
}
int main(void)
{
    console_dma_stats_t q,before;char text[600];unsigned i;uint32_t n;
    huart4.hdmatx=&dma;huart4.Init.BaudRate=115200;dma.State=HAL_DMA_STATE_READY;
    /* Boot output remains queued across Init; callers never transmit. */
    ConsoleDma_Print("boot\r\n");CHECK(!calls);ConsoleDma_Init();ConsoleDma_Service();
    strcpy(text,"copied\r\n");ConsoleDma_Print(text);memset(text,'x',10);
    CHECK(!memcmp(pointer,snapshot,length));drain();CHECK(used==14 && !memcmp(output,"boot\r\ncopied\r\n",14));
    /* Multi-chunk output stays immutable through producer writes, ring wrap
     * and both task/ISR enqueue. Oversize ISR calls are bounded and counted. */
    test_scb.CCR=SCB_CCR_DC_Msk;memset(text,'A',sizeof(text));n=used;
    CHECK(ConsoleDma_Write((uint8_t *)text,sizeof(text)));ConsoleDma_Service();
    ipsr=5;ConsoleDma_Print("IRQ\r\n");CHECK(!ConsoleDma_Write((uint8_t *)text,161));ipsr=0;
    CHECK(!memcmp(pointer,snapshot,length));drain();CHECK(used-n==605);
    for(i=n;i<n+600;i++){CHECK(output[i]=='A');}
    CHECK(!memcmp(output+n+600,"IRQ\r\n",5) && cache_calls);
    ConsoleDma_GetStats(&before);
    for(i=0;i<8192/sizeof(text);i++)CHECK(ConsoleDma_Write((uint8_t *)text,sizeof(text)));
    CHECK(ConsoleDma_Write((uint8_t *)text,392));ConsoleDma_GetStats(&q);CHECK(q.queued_bytes==8192);
    CHECK(!ConsoleDma_Write((uint8_t *)"X",1));ConsoleDma_GetStats(&q);CHECK(q.dropped_calls==before.dropped_calls+1);
    ConsoleDma_Service();complete();CHECK(ConsoleDma_Write((uint8_t *)"X",1));drain();
    /* BUSY/error start retains queued data and retries without sleeping. */
    CHECK(ConsoleDma_Write((uint8_t *)"busy",4));result=HAL_BUSY;n=calls;ConsoleDma_Service();CHECK(calls==n+1);
    tick+=9;ConsoleDma_Service();CHECK(calls==n+1);tick++;result=HAL_OK;ConsoleDma_Service();drain();
    /* DMA fault and missing completion do not overwrite a live DMA buffer.
     * Abort touches TX only and late TC during abort cannot consume twice. */
    CHECK(ConsoleDma_Write((uint8_t *)"fault",5));ConsoleDma_Service();ConsoleDma_TxError(&huart4);ConsoleDma_Service();
    CHECK(aborts==1);CHECK(ConsoleDma_Write((uint8_t *)"later",5));ConsoleDma_TxComplete(&huart4);ConsoleDma_Service();
    CHECK(!memcmp(pointer,snapshot,length));huart4.gState=HAL_UART_STATE_READY;dma.State=HAL_DMA_STATE_READY;
    ConsoleDma_AbortComplete(&huart4);ConsoleDma_Service();drain();
    ConsoleDma_GetStats(&q);CHECK(q.aborts==1 && q.tx_errors==1 && !q.queued_bytes);
    tick=UINT32_MAX-10U;CHECK(ConsoleDma_Write((uint8_t *)"timeout",7));ConsoleDma_Service();tick+=150;
    ConsoleDma_Service();CHECK(aborts==2);huart4.gState=HAL_UART_STATE_READY;dma.State=HAL_DMA_STATE_READY;
    ConsoleDma_AbortComplete(&huart4);ConsoleDma_Service();ConsoleDma_GetStats(&q);CHECK(q.tx_timeouts==1 && !q.queued_bytes);
    /* Real UART4 TX IRQ masks HAL RX sources without changing raw RX setup. */
    CHECK(ConsoleDma_Write((uint8_t *)"irq",3));ConsoleDma_Service();
    UART4->ISR=USART_ISR_TC;UART4->CR1=USART_CR1_TCIE|USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE;UART4->CR3=USART_CR3_EIE;
    ConsoleDma_Uart4TxIrq();CHECK((UART4->CR1&(USART_CR1_RXNEIE_RXFNEIE|USART_CR1_PEIE))==6 && UART4->CR3==8);
    ConsoleDma_Service();ConsoleDma_GetStats(&q);CHECK(!q.queued_bytes);
    CHECK(!mask);return failures?1:0;
}
