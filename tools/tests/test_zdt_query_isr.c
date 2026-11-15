#include "zdt_motor.h"
#include "arm_calibration.h"
#include "fdcan.h"
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_SetTick(uint32_t tick);
#define CHECK(x) do {++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
#define ZDT_CAN_RX_QUEUE_SIZE 16U
#include "zdt_query_rx_type.inc"
static ZDT_CAN_RxFrameTypeDef zdt_rx_queue[ZDT_CAN_RX_QUEUE_SIZE];
static volatile uint32_t zdt_rx_head,zdt_rx_tail,zdt_rx_sequence;
static volatile uint8_t zdt_user_query_address;
static zdt_motor_status_t zdt_status;
static volatile uint32_t zdt_speed_ack[4];
static FDCAN_HandleTypeDef query_can;
static FDCAN_RxHeaderTypeDef incoming;
static uint8_t payload[8],available;
static uint32_t arm_posts;
static HAL_StatusTypeDef post_result;
static uint32_t QueryISR_Fill(FDCAN_HandleTypeDef *can,uint32_t fifo)
{CHECK(can==&query_can && fifo==FDCAN_RX_FIFO0);return available;}
static HAL_StatusTypeDef QueryISR_Read(FDCAN_HandleTypeDef *can,uint32_t fifo,FDCAN_RxHeaderTypeDef *out,uint8_t *data)
{CHECK(can==&query_can && fifo==FDCAN_RX_FIFO0);*out=incoming;memcpy(data,payload,8U);available=0U;return HAL_OK;}
static HAL_StatusTypeDef QueryISR_Post(const FDCAN_RxHeaderTypeDef *h,const uint8_t *data,uint8_t size)
{CHECK(h->Identifier==incoming.Identifier && data[0]==payload[0] && size==7U);arm_posts++;return post_result;}
#define hfdcan2 query_can
#define HAL_FDCAN_GetRxFifoFillLevel QueryISR_Fill
#define HAL_FDCAN_GetRxMessage QueryISR_Read
#define ArmCalibration_PostCanFrame QueryISR_Post
#include "zdt_query_isr_under_test.inc"
#undef hfdcan2
#undef HAL_FDCAN_GetRxFifoFillLevel
#undef HAL_FDCAN_GetRxMessage
#undef ArmCalibration_PostCanFrame
static void receive(uint8_t address,uint8_t cmd,uint8_t requested,HAL_StatusTypeDef posted)
{
    memset((void *)&zdt_status,0,sizeof(zdt_status));zdt_rx_head=zdt_rx_tail=zdt_rx_sequence=0U;arm_posts=0U;
    memset(&incoming,0,sizeof(incoming));incoming.IdType=FDCAN_EXTENDED_ID;
    incoming.Identifier=(uint32_t)address<<8U;incoming.RxFrameType=FDCAN_DATA_FRAME;
    incoming.FDFormat=FDCAN_CLASSIC_CAN;incoming.DataLength=FDCAN_DLC_BYTES_7;
    memset(payload,0,sizeof(payload));payload[0]=cmd;payload[6]=0x6BU;
    zdt_user_query_address=requested;post_result=posted;available=1U;
    Test_SetTick(12345U);ZDT_Motor_RxFifo0MsgPending(&query_can,0U);
}
void Test_ZdtQueryRxRouting(void)
{
    receive(5U,0x36U,0U,HAL_OK);
    CHECK(arm_posts==1U && zdt_rx_head==0U && zdt_status.rx_count==1U); /* original arm-only path */
    receive(5U,0x36U,5U,HAL_OK);
    CHECK(arm_posts==1U && zdt_rx_head==1U && zdt_status.rx_count==1U);
    CHECK(zdt_rx_queue[0].received_tick==12345U && zdt_rx_queue[0].received_seq==1U && zdt_rx_queue[0].data[0]==0x36U);
    receive(6U,0x36U,6U,HAL_OK);CHECK(arm_posts==1U && zdt_rx_head==1U);
    receive(5U,0x36U,6U,HAL_OK);CHECK(arm_posts==1U && zdt_rx_head==0U); /* unrelated arm preserved */
    receive(5U,0x32U,5U,HAL_ERROR);CHECK(arm_posts==1U && zdt_rx_head==1U); /* pulse reply goes to query */
    receive(5U,0x36U,5U,HAL_BUSY);CHECK(arm_posts==1U && zdt_rx_head==1U && zdt_status.rx_dropped==1U);
    receive(4U,0x36U,4U,HAL_ERROR);CHECK(arm_posts==0U && zdt_rx_head==1U);
    incoming.Identifier=1U<<8U;incoming.DataLength=FDCAN_DLC_BYTES_3;
    payload[0]=0xF6;payload[1]=2;payload[2]=0x6B;available=1;
    ZDT_Motor_RxFifo0MsgPending(&query_can,0U);CHECK(zdt_speed_ack[0]==0x102U);
    available=1;payload[2]=0;ZDT_Motor_RxFifo0MsgPending(&query_can,0U);CHECK(zdt_speed_ack[0]==0x102U);
    payload[2]=0x6B;payload[1]=0xE2;available=1;ZDT_Motor_RxFifo0MsgPending(&query_can,0U);CHECK(zdt_speed_ack[0]==0x2E2U);
    incoming.FDFormat=FDCAN_FD_CAN;available=1U;zdt_rx_head=zdt_rx_tail=0U;
    ZDT_Motor_RxFifo0MsgPending(&query_can,0U);CHECK(zdt_rx_head==0U);
    incoming.FDFormat=FDCAN_CLASSIC_CAN;available=1U;zdt_rx_sequence=UINT32_MAX;
    ZDT_Motor_RxFifo0MsgPending(&query_can,0U);CHECK(zdt_rx_head==1U && zdt_rx_queue[0].received_seq==0U);
}
