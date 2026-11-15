#include "screen_hmi.h"
#include "screen_metrics.h"
#include "cmsis_os2.h"
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile unsigned int checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
UART_HandleTypeDef huart3,huart1;
static uint8_t *rx,*tx;static uint16_t txlen;
static uint32_t tick,transmits,aborts;static uint8_t active,fail_send;
static uint8_t history[64][64];static uint16_t lengths[64];
static uint8_t inject_loss;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{CHECK(u==&huart3 && n==1);rx=p;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{CHECK(u==&huart3 && n<=64 && transmits<64);if(fail_send || active)return HAL_BUSY;
 tx=p;txlen=n;memcpy(history[transmits],p,n);lengths[transmits++]=n;active=1;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){CHECK(u==&huart3);active=0;aborts++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u){CHECK(u==&huart3);return HAL_OK;}
osStatus_t osDelay(uint32_t n){(void)n;return osOK;}
static void complete(void){active=0;ScreenHmi_TxComplete(&huart3);}
static void bytes(const uint8_t *p,uint16_t n){while(n--){*rx=*p++;ScreenHmi_RxComplete(&huart3);}}
static void text(const char *s){bytes((const uint8_t *)s,(uint16_t)strlen(s));}
screen_event_t __real_Screen_ParserFeed(screen_parser_t *,uint8_t,uint32_t,screen_batch_t *);
screen_event_t __wrap_Screen_ParserFeed(screen_parser_t *p,uint8_t b,uint32_t now,screen_batch_t *batch)
{
    screen_event_t event=__real_Screen_ParserFeed(p,b,now,batch);
    if(event==SCREEN_EVENT_BATCH && inject_loss) {
        uint16_t i;inject_loss=0;
        /* Simulate producer IRQs while consumer has parsed LF but has not
         * published the candidate. Overflow must prevent map publication. */
        for(i=0;i<620;i++){*rx='x';ScreenHmi_RxComplete(&huart3);}
    }
    return event;
}
static void page(uint8_t id){uint8_t p[]={0x66,id,255,255,255};bytes(p,5);}
static void service(uint32_t now){tick=now;ScreenHmi_Service(now);}
static void reset(void){tick=transmits=aborts=0;active=fail_send=0;ScreenMetrics_ClearTask();ScreenMetrics_ResetCounts();ScreenMetrics_SetState(SCREEN_WAIT_QR);ScreenHmi_Init();}
static void command(uint32_t i,const char *s)
{size_t n=strlen(s);CHECK(lengths[i]==n+3);CHECK(!memcmp(history[i],s,n));CHECK(history[i][n]==255 && history[i][n+1]==255 && history[i][n+2]==255);}
int main(void)
{
    uint32_t before;uint16_t i;screen_batch_t b;screen_hmi_stats_t stats;
    reset();service(999);CHECK(!transmits);
    service(1000);command(0,"bkcmd=0");complete();service(1010);command(1,"sendme");complete();
    CHECK(ScreenMetrics_SetTask("156+123+516+231"));ScreenMetrics_Pick();ScreenMetrics_Place();page(0);
    service(1020);command(2,"tCode1.txt=\"156+123\"");
    ScreenMetrics_SetState(SCREEN_ERROR);service(1021);CHECK(transmits==3 && !memcmp(tx,history[2],txlen));
    complete();service(1030);command(3,"tCode1.txt=\"156+123\"");complete();
    service(1040);command(4,"tCode2.txt=\"516+231\"");complete();
    service(1050);command(5,"nPick.val=1");complete();service(1060);command(6,"nPlace.val=1");complete();
    service(1070);command(7,"tState.txt=\"ERROR\"");complete();
    before=transmits;page(1);ScreenMetrics_Place();service(1080);CHECK(transmits==before);
    service(1500);CHECK(transmits==before);page(0);service(1510);CHECK(transmits==before+1);
    complete();service(1520);complete();service(1530);complete();service(1540);command(transmits-1,"nPlace.val=2");complete();
    reset();text("&clear&obs 1225 1225&obs 1075 625&obs 25 25\n");service(20);
    CHECK(ScreenHmi_TakeBatch(&b) && b.count==3 && b.points[2].x==25);CHECK(!ScreenHmi_TakeBatch(&b));
    ScreenHmi_BatchResult(1);ScreenHmi_GetStats(&stats);CHECK(stats.applied==1);
    text("&clear&obs -25 25\n");service(30);CHECK(!ScreenHmi_TakeBatch(&b));
    text("&clear\n&clear\n");service(40);CHECK(ScreenHmi_TakeBatch(&b) && b.count==0);
    ScreenHmi_BatchResult(0);ScreenHmi_GetStats(&stats);CHECK(stats.rejected==2 && stats.busy_rejected==1);
    reset();for(i=0;i<620;i++){*rx='x';ScreenHmi_RxComplete(&huart3);}service(2000);
    CHECK(!ScreenHmi_TakeBatch(&b));text("\n");service(2010);text("&clear&obs 25 25\n");service(2020);
    CHECK(ScreenHmi_TakeBatch(&b) && b.count==1);ScreenHmi_GetStats(&stats);CHECK(stats.overflow>0);
    reset();inject_loss=1;text("&clear\n");service(20);CHECK(!ScreenHmi_TakeBatch(&b));
    text("\n&clear\n");service(40);CHECK(ScreenHmi_TakeBatch(&b) && b.count==0);
    reset();service(1000);CHECK(active);service(1200);CHECK(aborts==1 && lengths[1]==3);
    CHECK(history[1][0]==255 && history[1][1]==255 && history[1][2]==255);complete();service(1210);command(2,"sendme");complete();
    ScreenHmi_UartError(&huart3);service(1220);text("\n&clear\n");service(1230);
    CHECK(ScreenHmi_TakeBatch(&b));ScreenHmi_GetStats(&stats);CHECK(stats.uart_errors==1 && stats.tx_errors==1);
    reset();fail_send=1;service(1000);CHECK(!transmits);fail_send=0;service(1010);command(0,"bkcmd=0");
    return (int)failures;
}
