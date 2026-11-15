#include "screen_hmi.h"
#include "screen_metrics.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#define RX_CAP 512U
static volatile uint8_t s_rx_byte,s_tx_busy,s_uart_error,s_rx_lost;
static volatile uint16_t s_head,s_tail;
static volatile uint8_t s_rx[RX_CAP];
static uint8_t s_tx[64]; /* Immutable until TX-complete; one task owns writes. */
static volatile screen_hmi_stats_t s_stats;
static volatile uint8_t s_pending;
static screen_batch_t s_batch;
static screen_parser_t s_parser;
static screen_snapshot_t s_render;
static uint32_t s_tx_tick,s_next_query,s_page_tick,s_refresh_tick,s_started;
static uint8_t s_field,s_configured,s_resync;
static uint32_t lock(void){uint32_t key=__get_PRIMASK();__disable_irq();__DMB();return key;}
static void unlock(uint32_t key){__DMB();__set_PRIMASK(key);}
void ScreenHmi_Init(void)
{
    uint32_t key=lock();s_head=s_tail=0;s_pending=s_tx_busy=s_uart_error=s_rx_lost=0;
    memset((void *)&s_stats,0,sizeof(s_stats));s_stats.page=255;
    Screen_ParserInit(&s_parser);memset(&s_render,0,sizeof(s_render));
    s_field=5;s_configured=s_resync=0;s_started=HAL_GetTick();s_next_query=s_started+1000U;
    unlock(key);
    if(HAL_UART_Receive_IT(&huart3,(uint8_t *)&s_rx_byte,1U)!=HAL_OK)s_uart_error=1U;
}
void ScreenHmi_RxComplete(UART_HandleTypeDef *uart)
{
    uint16_t next;if(uart!=&huart3)return;
    s_stats.rx++;next=(uint16_t)((s_head+1U)%RX_CAP);
    if(next==s_tail){s_stats.overflow++;s_rx_lost=1U;}
    else{s_rx[s_head]=s_rx_byte;__DMB();s_head=next;}
    if(HAL_UART_Receive_IT(uart,(uint8_t *)&s_rx_byte,1U)!=HAL_OK)s_uart_error=1U;
}
void ScreenHmi_TxComplete(UART_HandleTypeDef *uart){if(uart==&huart3)s_tx_busy=0U;}
void ScreenHmi_UartError(UART_HandleTypeDef *uart)
{if(uart==&huart3){s_stats.uart_errors++;s_uart_error=1U;}}
uint8_t ScreenHmi_TakeBatch(screen_batch_t *out)
{
    uint8_t ready;uint32_t key;if(!out)return 0U;
    key=lock();ready=s_pending;if(ready){*out=s_batch;s_pending=0;}unlock(key);return ready;
}
void ScreenHmi_BatchResult(uint8_t accepted)
{uint32_t key=lock();if(accepted)s_stats.applied++;else s_stats.busy_rejected++;unlock(key);}
void ScreenHmi_GetStats(screen_hmi_stats_t *out)
{uint32_t key;if(!out)return;key=lock();memcpy(out,(const void *)&s_stats,sizeof(*out));unlock(key);}
static uint8_t transmit(size_t length,uint32_t now)
{
    if(!length)return 0U;
    s_tx_tick=now;s_tx_busy=1U;
    if(HAL_UART_Transmit_IT(&huart3,s_tx,(uint16_t)length)!=HAL_OK) {
        s_tx_busy=0;s_stats.tx_errors++;return 0U;
    }
    return 1U;
}
static uint8_t literal(const char *text,uint32_t now)
{
    size_t n=strlen(text);if(n+3U>sizeof(s_tx))return 0;
    memcpy(s_tx,text,n);s_tx[n]=s_tx[n+1]=s_tx[n+2]=255;
    return transmit(n+3U,now);
}
void ScreenHmi_Service(uint32_t now)
{
    uint16_t budget=RX_CAP;screen_snapshot_t latest;uint32_t key;
    if(s_uart_error) {
        (void)HAL_UART_AbortReceive(&huart3);
        key=lock();s_tail=s_head;s_uart_error=0;s_rx_lost=1;unlock(key);
        if(HAL_UART_Receive_IT(&huart3,(uint8_t *)&s_rx_byte,1U)!=HAL_OK)s_uart_error=1;
    }
    if(s_rx_lost) {key=lock();s_rx_lost=0;unlock(key);Screen_ParserDrop(&s_parser);s_parser.last_byte=now;}
    while(s_tail!=s_head && budget--) {
        uint8_t byte=s_rx[s_tail];screen_batch_t batch;screen_event_t event;
        if(s_rx_lost){key=lock();s_rx_lost=0;unlock(key);Screen_ParserDrop(&s_parser);s_parser.last_byte=now;}
        __DMB();s_tail=(uint16_t)((s_tail+1U)%RX_CAP);
        event=Screen_ParserFeed(&s_parser,byte,now,&batch);
        if(event==SCREEN_EVENT_BATCH) {
            key=lock();if(!s_pending && !s_rx_lost && !s_uart_error){s_batch=batch;__DMB();s_pending=1;}else s_stats.rejected++;unlock(key);
        } else if(event==SCREEN_EVENT_REJECT)s_stats.rejected++;
        else if(event==SCREEN_EVENT_PAGE) {
            if(s_stats.page!=s_parser.page){s_stats.page=s_parser.page;s_field=5;s_refresh_tick=now-2000U;}
            s_page_tick=now;
        }
    }
    if(s_tx_busy && now-s_tx_tick>=200U) {
        (void)HAL_UART_AbortTransmit(&huart3);s_tx_busy=0;s_resync=1;s_field=5;s_stats.tx_errors++;
    }
    if(s_tx_busy || now-s_started<1000U)return;
    if(s_resync){s_tx[0]=s_tx[1]=s_tx[2]=255;if(transmit(3,now))s_resync=0;return;}
    if(!s_configured){if(literal("bkcmd=0",now))s_configured=1;return;}
    if((int32_t)(now-s_next_query)>=0){if(literal("sendme",now))s_next_query=now+1000U;return;}
    if(s_stats.page!=0U || now-s_page_tick>1500U)return;
    key=lock();memset(&latest,0,sizeof(latest));ScreenMetrics_Snapshot(&latest);unlock(key);
    if(memcmp(&latest,&s_render,sizeof(latest)) || (s_field>=5U && now-s_refresh_tick>=2000U)) {
        s_render=latest;s_field=0;s_refresh_tick=now;
    }
    if(s_field<5U && transmit(ScreenMetrics_Command(&s_render,s_field,s_tx,sizeof(s_tx)),now))s_field++;
}
void ScreenHmi_Run(void *argument)
{(void)argument;for(;;){ScreenHmi_Service(HAL_GetTick());osDelay(10U);}}
