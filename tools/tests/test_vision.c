#include "vision.h"
#include "vision_mode_gate.h"
#include "vision_align.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "vision_protocol8_vectors.h"
volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
UART_HandleTypeDef huart1,huart2,huart10,huart3,huart4;
static uint32_t log_tx,log_errors,log_aborts;
void ConsoleDma_TxComplete(UART_HandleTypeDef *u){if(u==&huart4)log_tx++;}
void ConsoleDma_TxError(UART_HandleTypeDef *u){if(u==&huart4)log_errors++;}
void ConsoleDma_AbortComplete(UART_HandleTypeDef *u){if(u==&huart4)log_aborts++;}
static uint32_t screen_rx_callbacks,screen_tx_callbacks,screen_error_callbacks;
void ScreenHmi_RxComplete(UART_HandleTypeDef *uart){if(uart==&huart3)screen_rx_callbacks++;}
void ScreenHmi_TxComplete(UART_HandleTypeDef *uart){if(uart==&huart3)screen_tx_callbacks++;}
void ScreenHmi_UartError(UART_HandleTypeDef *uart){if(uart==&huart3)screen_error_callbacks++;}
static USART_TypeDef regs[3];
static DMA_Stream_TypeDef dma_regs[3];
static DMA_HandleTypeDef dmas[3];
static uint8_t *buffers[3];
static uint16_t sizes[3],write_pos[3];
static unsigned int receives[3],aborts[3],notifications,hwt_bytes;
static uint8_t fail_cam1;
static uint8_t tx_data[2][8],tx_complete=1,tx_fail;
static uint8_t *tx_pointer[2];
static unsigned int tx_calls[2],tx_aborts[2];
static uint32_t tick;
static uint8_t protocol_switch_tick_count;
static char console[16384];static size_t console_length;
static jmp_buf task_return;
uint32_t HAL_GetTick(void){
    if(protocol_switch_tick_count && --protocol_switch_tick_count==0)
        (void)Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8);
    return tick;
}
static unsigned int index_of(UART_HandleTypeDef *uart){return uart==&huart1?0U:(uart==&huart2?1U:2U);}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,uint8_t *data,uint16_t n){
    unsigned int i=index_of(uart);receives[i]++;if(i==0U && fail_cam1)return HAL_ERROR;
    buffers[i]=data;sizes[i]=n;write_pos[i]=0;dma_regs[i].NDTR=n;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart){aborts[index_of(uart)]++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t n){
    unsigned int i=index_of(uart);CHECK(i<2 && n==8);tx_calls[i]++;memcpy(tx_data[i],data,8);tx_pointer[i]=data;
    if(tx_fail)return HAL_BUSY;
    if(tx_complete)HAL_UART_TxCpltCallback(uart);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart){tx_aborts[index_of(uart)]++;return HAL_OK;}
void HWT101_OnDmaData(const uint8_t *data,uint16_t n){(void)data;hwt_bytes+=n;}
osThreadId_t osThreadGetId(void){return (osThreadId_t)1;}
uint32_t osThreadFlagsSet(osThreadId_t id,uint32_t flags){CHECK(id==(osThreadId_t)1);notifications++;return flags;}
uint32_t osThreadFlagsWait(uint32_t flags,uint32_t options,uint32_t timeout){
    CHECK(flags==1U && options==osFlagsWaitAny && timeout==10U);longjmp(task_return,1);return flags;
}
void ZDT_Console_Print(const char *s){size_t n=strlen(s);if(console_length+n<sizeof(console)){memcpy(console+console_length,s,n+1U);console_length+=n;}}
static void clear_console(void){console_length=0;console[0]='\0';}
static void send_bytes(unsigned int i,const uint8_t *data,uint16_t n){
    UART_HandleTypeDef *uart=i==0?&huart1:(i==1?&huart2:&huart10);uint16_t b;
    for(b=0;b<n;b++){
        buffers[i][write_pos[i]++]=data[b];if(write_pos[i]==sizes[i])write_pos[i]=0;
        dma_regs[i].NDTR=write_pos[i]?sizes[i]-write_pos[i]:sizes[i];
        if(write_pos[i]==sizes[i]/2U)HAL_UART_RxHalfCpltCallback(uart);
        if(write_pos[i]==0U)HAL_UART_RxCpltCallback(uart);
    }
    SET_BIT(regs[i].ISR,USART_ISR_IDLE);Vision_CheckIdle(uart);CLEAR_BIT(regs[i].ISR,USART_ISR_IDLE);
}
static void frame(unsigned int i,const char *payload){
    char text[200];size_t n=strlen(payload),j;uint8_t cs=0;
    for(j=0;j<n;j++)cs^=(uint8_t)payload[j];
    (void)snprintf(text,sizeof(text),"@%s*%02X\r\n",payload,cs);send_bytes(i,(const uint8_t *)text,(uint16_t)strlen(text));
}
static uint8_t test_crc8(const uint8_t *data,unsigned int n){
    uint8_t crc=0;unsigned int i,j;
    for(i=0;i<n;i++) { crc^=data[i];for(j=0;j<8;j++) { uint8_t high=crc&0x80U;crc=(uint8_t)(crc<<1);if(high)crc^=7U; } }
    return crc;
}
static void pc_obstacle_tests(void)
{
    vision_pc_obstacle_t o;uint32_t age,id;uint8_t packet[8]={0x6b,0x86,6,0x2c,0x01,0x9c,0xff,0};
    static const uint8_t contract[8]={0x6b,0x86,5,0x2c,0x01,0x96,0xff,0xbe};
    tick=30000;CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_BINARY8));Vision_ClearPcObstacle();
    packet[7]=test_crc8(packet,7);
    tick=30001;send_bytes(1,packet,3);tick=30002;send_bytes(1,packet+3,5);
    tick=30082;Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age));
    CHECK(o.front_gap_mm==300 && o.body_right_mm== -100 && o.bridge_age_ms==60);
    CHECK(o.rx_tick==30001 && age==141);id=o.observation_id;
    packet[7]^=1;tick++;send_bytes(1,packet,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.observation_id==id);
    packet[7]^=1;tick++;send_bytes(0,packet,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.observation_id==id);
    /* A complete RX frame queued before a new run's clear cannot reappear. */
    tick++;send_bytes(1,packet,8);tick++;Vision_ClearPcObstacle();Vision_Service();
    CHECK(!Vision_GetPcObstacle(&o,&age));
    tick++;send_bytes(1,packet,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.observation_id!=id);
    packet[2]=21;packet[7]=test_crc8(packet,7);id=o.observation_id;
    tick++;send_bytes(1,packet,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.observation_id==id);
    CHECK(test_crc8(contract,7)==contract[7]);
    tick++;CHECK(Vision_RequestMode(2,VISION_MODE_QR,4,0,20,0)==HAL_OK);
    tick++;send_bytes(1,contract,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.body_right_mm== -106 && o.bridge_age_ms==50);
    id=o.observation_id;HAL_UART_TxCpltCallback(&huart2);
    CHECK(Vision_RequestStop(2)==HAL_OK);tick++;send_bytes(1,contract,8);Vision_Service();
    CHECK(Vision_GetPcObstacle(&o,&age) && o.observation_id!=id && o.front_gap_mm==300);
    packet[2]=0;packet[3]=0x84;packet[4]=0x03;packet[7]=test_crc8(packet,7);
    tick++;send_bytes(1,packet,8);Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age) && o.front_gap_mm==900);
    /* CAM1 changes do not clear CAM2. CAM2 protocol/reset invalidation does,
     * including complete observations that are still younger than 200 ms. */
    CHECK(Vision_ClearPixelObservations(1));CHECK(Vision_GetPcObstacle(&o,&age));
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_ASCII));
    CHECK(!Vision_GetPcObstacle(&o,&age));Vision_Service();
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_BINARY8));Vision_Service();
    send_bytes(1,contract,8);Vision_Service();CHECK(!Vision_GetPcObstacle(&o,&age));
    tick++;send_bytes(1,contract,8);Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age));
    CHECK(Vision_ClearPixelObservations(2));CHECK(!Vision_GetPcObstacle(&o,&age));Vision_Service();
    tick++;send_bytes(1,contract,8);Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age));
    HAL_UART_ErrorCallback(&huart2);tick++;Vision_Service();CHECK(!Vision_GetPcObstacle(&o,&age));
    tick++;send_bytes(1,contract,8);Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age));
    {
        uint8_t fill[200];unsigned int j;memset(fill,0x55,sizeof(fill));
        for(j=0;j<25U;j++)send_bytes(1,fill,sizeof(fill));
        tick++;Vision_Service();CHECK(!Vision_GetPcObstacle(&o,&age));
    }
    tick++;send_bytes(1,contract,8);Vision_Service();CHECK(Vision_GetPcObstacle(&o,&age));
}
static void binary_tests(void){
    vision_pixel_t value,other;uint32_t age,before,observation;uint8_t bytes[24];unsigned int i;
    CHECK(test_crc8((const uint8_t *)"123456789",9)==0xF4U);
    CHECK(!Vision_SetCameraProtocol(3,VISION_PROTOCOL_BINARY8));
    CHECK(!Vision_SetCameraProtocol(1,0));
    CHECK(!Vision_GetPixel(3,VISION_TYPE_MAT_PX,&value,NULL));
    CHECK(!Vision_GetPixel(1,0x7F,&value,NULL));
    CHECK(!Vision_GetPixel(1,VISION_TYPE_MAT_PX,NULL,NULL));
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_BINARY8));Vision_Service();
    tick=2000;send_bytes(0,vector_material,3);Vision_Service();
    CHECK(!Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL));
    tick+=5;send_bytes(0,vector_material+3,5);
    CHECK(!Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL)); /* ISR never parses */
    Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age));
    CHECK(value.valid && value.id==3 && value.seq==12 && age==0);
    observation=value.observation_id;CHECK(observation!=0U);
    CHECK(fabsf(value.dx_px-30)<.001f && fabsf(value.dy_px-20)<.001f);
    tick+=20;send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && age==20); /* duplicate age unchanged */
    CHECK(value.observation_id==observation);
    tick+=5;memcpy(bytes,vector_material,8);bytes[3]++;bytes[7]=test_crc8(bytes,7);
    before=Vision_GetParseErrorCount();send_bytes(0,bytes,8);Vision_Service();
    CHECK(Vision_GetParseErrorCount()>before);
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && age==25 && value.dx_px==30);
    tick+=31;send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && value.valid && age==0); /* SEQ after window */
    CHECK(value.observation_id!=observation);
    tick+=1;send_bytes(0,vector_material_bad,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && !value.valid && value.dx_px==0 && value.dy_px==0);
    tick+=1;send_bytes(0,vector_origin,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid && value.dx_px==0 && value.dy_px==0);
    tick+=1;send_bytes(1,vector_ring,8);send_bytes(0,vector_plate,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&value,&age) && value.valid && value.id==0);
    CHECK(value.dx_px==15 && value.dy_px==20);
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&other,NULL) && !other.valid); /* mode invalidates old type */
    CHECK(Vision_GetPixel(2,VISION_TYPE_RING_PX,&other,NULL) && other.valid && other.id==2);
    CHECK(other.dx_px==30 && other.dy_px==20); /* second port never mixes with first */
    tick+=1;send_bytes(0,vector_markers,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid);
    CHECK(fabsf(value.dx_px-257.3f)<.001f && fabsf(value.dy_px-1649.1f)<.001f); /* payload CR LF 6B @ */
    send_bytes(0,vector_meta_head,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid); /* META=6B is not a new header */
    memcpy(bytes,vector_material,8);bytes[7]^=1;
    memcpy(bytes+8,vector_material_next,8);before=Vision_GetParseErrorCount();
    send_bytes(0,bytes,16);Vision_Service();
    CHECK(Vision_GetParseErrorCount()>before);
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid && value.seq==13 && value.dx_px==31);
    /* Invalid type/ID/nonzero invalid payload cannot overwrite a good observation. */
    for(i=0;i<3;i++){
        memcpy(bytes,vector_material,8);
        if(i==0)bytes[1]=0x7F;
        else if(i==1)bytes[2]=(uint8_t)((bytes[2]&0xF8U)|7U);
        else bytes[2]&=0x7FU;
        bytes[7]=test_crc8(bytes,7);send_bytes(0,bytes,8);Vision_Service();
        CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid && value.seq==13);
    }
    tick+=1;send_bytes(0,vector_origin,4);Vision_Service();
    tick+=55;before=Vision_GetParseErrorCount();Vision_Service();CHECK(Vision_GetParseErrorCount()>before);
    send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && value.valid && value.seq==12 && age==0);
    tick+=200;CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && !value.valid && value.dx_px==0 && age==200);
    send_bytes(0,vector_limits,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid);
    CHECK(fabsf(value.dx_px+3276.8f)<.001f && fabsf(value.dy_px-3276.7f)<.001f);
    send_bytes(0,vector_black,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.id==6);
    send_bytes(0,vector_lightblue,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.id==5);
    /* ASCII must not be auto-detected inside a binary stream. */
    frame(0,"QR,321+123,seq=90");Vision_Service();
    { char digits[16];uint16_t seq;CHECK(Vision_GetQR(digits,&seq,NULL) && seq==14); }
    HAL_UART_ErrorCallback(&huart1);CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);
    tick+=100;Vision_Service();send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid);
    { uint8_t fill[200];memset(fill,0x55,sizeof(fill));
      for(i=0;i<12;i++)send_bytes(0,fill,sizeof(fill));
      CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);
      Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);
      tick+=1;send_bytes(0,vector_material_next,8);Vision_Service();
      CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid);
    }
    /* Tick wrap uses elapsed unsigned arithmetic. */
    tick=0xFFFFFFF0U;send_bytes(0,vector_material_next,8);Vision_Service();
    tick=20;CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,&age) && value.valid && age==36);
    /* Simulate another task changing context just before old-frame publish. */
    send_bytes(0,vector_material,8);protocol_switch_tick_count=10;Vision_Service();
    CHECK(protocol_switch_tick_count==0);
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);
    send_bytes(0,vector_material_next,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid && value.seq==13);
    observation=value.observation_id;
    send_bytes(0,vector_material,8); /* queued before station boundary */
    CHECK(!Vision_ClearPixelObservations(3U));
    CHECK(Vision_ClearPixelObservations(1U));Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);
    send_bytes(0,vector_material_next,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && value.valid && value.observation_id!=observation);
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_ASCII));
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&value,NULL) && !value.valid);Vision_Service();
}
static void control_tests(void){
    vision_camera_state_t state;vision_pixel_t pixel;unsigned int calls;uint8_t packet[8];
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_BINARY8));Vision_Service();tick=4000;
    CHECK(!Vision_GetCameraState(3,&state) && !Vision_GetCameraState(1,NULL));
    CHECK(Vision_RequestMode(3,2,1,3,20,0)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,8,1,1,20,0)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,3,2,1,20,0)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,0,0,0,20,0)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,2,1,3,20,1)==HAL_ERROR);
    send_bytes(0,vector_hb_stop,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.heartbeat_seen && state.protocol_version==1);
    CHECK(Vision_RequestMode(1,2,1,3,20,0)==HAL_OK);
    CHECK(Vision_RequestMode(1,3,2,2,20,0)==HAL_BUSY);
    CHECK(Vision_GetCameraState(1,&state) && state.pending && !state.confirmed && state.attempts==0);
    Vision_Service();CHECK(memcmp(tx_data[0],vector_set_material,8)==0);
    CHECK(Vision_GetCameraState(1,&state) && state.attempts==1);
    send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && !pixel.valid); /* ACK boundary */
    memcpy(packet,vector_ack_material,8);packet[2]^=8;packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state) && state.pending);
    send_bytes(0,vector_ack_material,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && !state.pending && state.confirmed && state.result==0 && state.target==3);
    send_bytes(0,vector_material,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && pixel.valid);
    send_bytes(0,vector_ring,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && pixel.valid); /* wrong type ignored */
    send_bytes(0,vector_black,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && pixel.valid && pixel.id==3);
    CHECK(Vision_RequestMode(1,3,2,2,20,0)==HAL_OK);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && !pixel.valid);
    send_bytes(0,vector_ack_ring,8);send_bytes(0,vector_ring,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.mode==3 && state.station==2);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid && pixel.id==2);
    send_bytes(0,vector_hb_ring,8);Vision_Service();
    CHECK(Vision_RequestMode(1,8,1,0,20,0)==HAL_OK);Vision_Service();
    send_bytes(0,vector_ack_plate_failure,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && !state.pending && state.mode==3 && state.result==7);
    tick+=1;send_bytes(0,vector_ring,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid);
    send_bytes(0,vector_hb_unknown,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && !state.confirmed && state.protocol_version==2);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && !pixel.valid);
    send_bytes(0,vector_hb_ring,8);Vision_Service();
    CHECK(Vision_RequestMode(1,0,0,0,0,0)==HAL_OK);Vision_Service();
    send_bytes(0,vector_ack_stop,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.mode==0);
    send_bytes(0,vector_ring,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && !pixel.valid);
    calls=tx_calls[0];CHECK(Vision_RequestMode(1,3,2,2,20,0)==HAL_OK);Vision_Service();
    memcpy(packet,tx_data[0],8);tick+=500;Vision_Service();CHECK(memcmp(packet,tx_data[0],8)==0);
    tick+=500;Vision_Service();CHECK(memcmp(packet,tx_data[0],8)==0);
    tick+=500;Vision_Service();CHECK(tx_calls[0]==calls+3);
    CHECK(Vision_GetCameraState(1,&state) && !state.pending && !state.confirmed && state.result==VISION_RESULT_TIMEOUT);
    send_bytes(0,vector_ack_retry_ring,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state) && !state.confirmed); /* late ACK */
    /* TX IRQ stall aborts in task and never blocks VisionTask. */
    tx_complete=0;calls=tx_aborts[1];CHECK(Vision_RequestMode(2,2,1,1,20,0)==HAL_OK);Vision_Service();
    CHECK(!Vision_SetCameraProtocol(2,VISION_PROTOCOL_ASCII));
    tick+=51;Vision_Service();CHECK(tx_aborts[1]==calls+1);
    tx_complete=1;tick+=500;Vision_Service();CHECK(Vision_GetCameraState(2,&state) && state.pending && state.attempts==2);
    tick+=1000;Vision_Service();tick+=500;Vision_Service();CHECK(Vision_GetCameraState(2,&state) && state.result==VISION_RESULT_TIMEOUT);
    /* New request after failure uses a new request number; TX errors bounded. */
    tx_fail=1;CHECK(Vision_RequestMode(2,0,0,0,0,0)==HAL_OK);Vision_Service();
    tx_fail=0;tick+=500;Vision_Service();CHECK(Vision_GetCameraState(2,&state) && state.attempts==2);
    CHECK(Vision_RequestMode(1,3,2,2,20,0)==HAL_OK);Vision_Service();
    memcpy(packet,vector_ack_ring,8);packet[2]=(uint8_t)(0x80U|((tx_data[0][2]>>3)<<3)|2U);packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);send_bytes(0,vector_hb_ring,8);send_bytes(0,vector_ring,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed);
    tick+=1500;Vision_Service();CHECK(Vision_GetCameraState(1,&state) && state.confirmed);
    CHECK(Vision_GetPixelWindow(1,VISION_TYPE_RING_PX,500U,&pixel,NULL) && !pixel.valid);
    /* A matching heartbeat can bridge a processing pause without a new SET;
     * stale position data must never become valid just because HB resumes. */
    send_bytes(0,vector_hb_ring,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.peer_age_ms==0U);
    CHECK(Vision_GetPixelWindow(1,VISION_TYPE_RING_PX,500U,&pixel,NULL) && !pixel.valid);
    tick+=VISION_CAMERA_PEER_TIMEOUT_MS-1U;
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed);
    tick++;CHECK(Vision_GetCameraState(1,&state) && !state.confirmed);
    CHECK(state.peer_age_ms>=VISION_CAMERA_PEER_TIMEOUT_MS);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && !pixel.valid);Vision_Service();
    CHECK(Vision_RequestMode(1,3,2,2,20,0)==HAL_OK);Vision_Service();
    packet[2]=(uint8_t)(0x80U|tx_data[0][2]);packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);send_bytes(0,vector_ring,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid);
    send_bytes(0,vector_hb_stop,8);Vision_Service(); /* reboot toSTOP invalidates even with freshXY */
    CHECK(Vision_GetCameraState(1,&state) && !state.confirmed);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && !pixel.valid);
}
static void mode_alignment_integration_tests(void){
    vision_mode_gate_t gate;vision_camera_state_t state;
    vision_align_t align;vision_align_config_t config;
    vision_pixel_t point={0};uint32_t age=0,before;float f,r;unsigned int i;
    uint8_t packet[8]={0x6B,0x81,0,0x80,0,VISION_MODE_PLATE,1,0};
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    tick+=100U;VisionModeGate_Start(&gate,1,VISION_MODE_PLATE,1,0,tick);
    CHECK(Vision_GetCameraState(1,&state));
    CHECK(VisionModeGate_Step(&gate,tick,&state)==V_MODE_SEND);
    CHECK(Vision_RequestMode(1,VISION_MODE_PLATE,1,0,20,0)==HAL_OK);
    CHECK(Vision_GetCameraState(1,&state));VisionModeGate_Sent(&gate,state.request_seq);
    CHECK(VisionModeGate_Step(&gate,tick,&state)==V_MODE_WAIT);Vision_Service();
    send_bytes(0,vector_plate,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&point,&age) && !point.valid);
    packet[2]=(uint8_t)(0x80U|(state.request_seq<<3));packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state));
    CHECK(VisionModeGate_Step(&gate,tick,&state)==V_MODE_READY);
    CHECK(Vision_ClearPixelObservations(1));CHECK(Vision_GetCameraState(1,&state));
    CHECK(VisionModeGate_Matches(&gate,&state)); /* clear does not cancel ACK */
    VisionAlign_DefaultConfig(&config);config.camera=1;config.type=VISION_TYPE_PLATE_PX;
    config.forward_x=0.0007f;config.right_y=0.0007f;
    before=point.observation_id;CHECK(VisionAlign_Start(&align,&config,tick,before));
    VisionAlign_Step(&align,tick,&point,age,1,&f,&r);CHECK(f==0 && r==0);
    tick+=config.settle_ms;VisionAlign_Step(&align,tick,NULL,0,1,&f,&r);
    send_bytes(0,vector_plate,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&point,&age));
    VisionAlign_Step(&align,tick,&point,age,1,&f,&r);
    CHECK(align.state==V_ALIGN_PULSE && f>0 && r>0);
    /* New explicit invalid frame cancels an active command immediately. */
    packet[1]=VISION_TYPE_PLATE_PX;packet[2]=0x08;memset(packet+3,0,4);packet[7]=test_crc8(packet,7);
    tick+=10;send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&point,&age));
    VisionAlign_Step(&align,tick,&point,age,0,&f,&r);CHECK(f==0 && r==0 && align.state==V_ALIGN_SETTLE);
    tick+=10;VisionAlign_Step(&align,tick,&point,age,1,&f,&r);
    tick+=config.settle_ms;VisionAlign_Step(&align,tick,NULL,0,1,&f,&r);
    for(i=0;i<5;i++) {
        packet[2]=(uint8_t)(0x80U|((i+2U)<<3));packet[7]=test_crc8(packet,7);
        send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&point,&age));
        VisionAlign_Step(&align,tick,&point,age,1,&f,&r);
        if(i<4U)CHECK(align.state!=V_ALIGN_DONE);
        tick+=50U;
    }
    CHECK(align.state==V_ALIGN_DONE && f==0 && r==0);
    /* A heartbeat for another station invalidates the same mode's fresh XY. */
    packet[1]=0x82;packet[2]=0x80;packet[3]=1;packet[4]=VISION_MODE_PLATE;packet[5]=2;packet[6]=0;
    packet[7]=test_crc8(packet,7);send_bytes(0,packet,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && !VisionModeGate_Matches(&gate,&state));
    CHECK(state.heartbeat_mode==VISION_MODE_PLATE && state.heartbeat_station==2U);
    CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&point,&age) && !point.valid);
}
static void ack_last(unsigned int camera){
    uint8_t packet[8]={0x6B,0x81,0,0x80,0,0,0,0};
    packet[2]=(uint8_t)(0x80U|tx_data[camera][2]);
    packet[5]=tx_data[camera][3];packet[6]=tx_data[camera][4];packet[7]=test_crc8(packet,7);
    send_bytes(camera,packet,8);Vision_Service();
}
static void ring_scene_pair_tests(void){
    vision_camera_state_t state;vision_pixel_t pixel;vision_ring_count_t scene;
    uint8_t xy[8],count[8];uint32_t age,observation;unsigned int i;
    tick=50000U;tx_complete=1;tx_fail=0;
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,0)==HAL_OK);Vision_Service();ack_last(0);
    send_bytes(0,vector_hb_ring,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state));
    memcpy(xy,vector_ring,8);xy[2]=(uint8_t)(0x80U|(5U<<3)|2U);xy[7]=test_crc8(xy,7);
    count[0]=0x6B;count[1]=VISION_TYPE_RING_COUNT;count[2]=xy[2];count[3]=3U;
    count[4]=2U;count[5]=state.request_seq;count[6]=0U;count[7]=test_crc8(count,7);
    send_bytes(0,count,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    tick+=10U;send_bytes(0,xy,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && pixel.valid && scene.valid && scene.count==3U && age==10U);
    observation=scene.observation_id;
    send_bytes(0,xy,8);send_bytes(0,count,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.observation_id==observation && age==10U);
    tick+=80U;send_bytes(0,count,8);send_bytes(0,xy,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.observation_id==observation && age==90U);
    /* Pixel-first order, and count2 overrides an incorrectly valid pixel. */
    tick+=20U;xy[2]=(uint8_t)(0x80U|(6U<<3)|2U);xy[7]=test_crc8(xy,7);
    count[2]=xy[2];count[3]=2U;count[7]=test_crc8(count,7);
    send_bytes(0,xy,8);send_bytes(0,count,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.valid && scene.count==2U && !pixel.valid);
    /* New zero/one-ring scenes are trustworthy counts but never target data. */
    for(i=0U;i<2U;i++) {
        tick+=10U;xy[2]=(uint8_t)(0x80U|((7U+i)<<3)|2U);xy[7]=test_crc8(xy,7);
        count[2]=xy[2];count[3]=(uint8_t)i;count[7]=test_crc8(count,7);
        send_bytes(0,count,8);send_bytes(0,xy,8);Vision_Service();
        CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.valid && scene.count==i && !pixel.valid);
    }
    CHECK(Vision_ClearPixelObservations(1));Vision_Service();
    xy[2]=(uint8_t)(0x80U|(9U<<3)|2U);xy[7]=test_crc8(xy,7);count[2]=xy[2];count[3]=3U;
    count[4]=3U;count[7]=test_crc8(count,7);send_bytes(0,xy,8);send_bytes(0,count,8);Vision_Service();
    CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age)); /* wrong station */
    count[4]=2U;count[5]=(uint8_t)((state.request_seq+1U)&15U);count[7]=test_crc8(count,7);
    send_bytes(0,count,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    count[5]=state.request_seq;count[7]=test_crc8(count,7);send_bytes(0,count,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.valid && pixel.valid);
    /* >50ms cannot complete an old half-frame. */
    CHECK(Vision_ClearPixelObservations(1));Vision_Service();
    count[2]=(uint8_t)(0x80U|(10U<<3)|2U);count[7]=test_crc8(count,7);
    xy[2]=count[2];xy[7]=test_crc8(xy,7);send_bytes(0,count,8);Vision_Service();
    tick+=51U;send_bytes(0,xy,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    send_bytes(0,count,8);Vision_Service();CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.valid);
    tick+=200U;CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && !scene.valid && !pixel.valid);
    CHECK(Vision_GetRingSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid && pixel.valid && age==200U);
    tick+=200U;CHECK(Vision_GetRingSceneWindow(1,400U,&pixel,&scene,&age) && !scene.valid && !pixel.valid);
    /* STOP clears both halves and published scene; restart uses a new SET. */
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55U;Vision_Service();ack_last(0);
    CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,3,2,20,0)==HAL_OK);Vision_Service();ack_last(0);
    send_bytes(0,count,8);send_bytes(0,xy,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    CHECK(!Vision_GetRingScene(3,&pixel,&scene,&age) && !Vision_GetRingScene(1,NULL,&scene,&age));
}
static void ring_green_scene_tests(void)
{
    vision_pixel_t pixel;vision_ring_count_t scene;vision_camera_state_t state;
    uint8_t xy[8],count[8];uint32_t age;
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55U;Vision_Service();ack_last(0);
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,VISION_RING_GREEN_SEARCH_FLAG)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,3,2,20,VISION_RING_GREEN_SEARCH_FLAG)==HAL_OK);
    Vision_Service();ack_last(0);CHECK(Vision_GetCameraState(1,&state));
    memcpy(xy,vector_ring,8);xy[2]=(uint8_t)((3U<<3)|2U);xy[3]=xy[4]=xy[5]=xy[6]=0;xy[7]=test_crc8(xy,7);
    memcpy(count,vector_ring_green,8);count[5]=state.request_seq;count[7]=test_crc8(count,7);
    CHECK(Vision_ClearPixelObservations(1));send_bytes(0,xy,8);Vision_Service();
    CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age)); /* no independent unpaired authorization */
    send_bytes(0,count,8);Vision_Service();
    CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && scene.green_visible && !scene.valid && scene.count==1 && !pixel.valid);
    tick+=200;CHECK(Vision_GetRingScene(1,&pixel,&scene,&age) && !scene.green_visible && !pixel.valid);
    CHECK(Vision_GetRingSceneWindow(1,400,&pixel,&scene,&age) && scene.green_visible && !pixel.valid);
    CHECK(Vision_ClearPixelObservations(1));xy[2]=(uint8_t)((4U<<3)|2U);xy[7]=test_crc8(xy,7);
    count[2]=xy[2];count[6]=2;count[7]=test_crc8(count,7);
    send_bytes(0,xy,8);send_bytes(0,count,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    count[6]=1;count[5]=(uint8_t)((state.request_seq+1U)&15U);count[7]=test_crc8(count,7);
    send_bytes(0,count,8);Vision_Service();CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55;Vision_Service();ack_last(0);
    CHECK(!Vision_GetRingScene(1,&pixel,&scene,&age));
}
static void ring_axis_triple_tests(void)
{
    const uint8_t golden_set[8]={0x6B,0x80,0x3A,0x03,0x02,0x14,0x01,0xE7};
    const uint8_t golden_ack[8]={0x6B,0x81,0xBA,0x80,0x00,0x03,0x02,0xB1};
    const uint8_t golden_group[24]={0x6B,0x03,0xAA,0x7B,0,0xD3,0xFF,0xF3,
        0x6B,0x0A,0xAA,0x83,0xFF,0x58,0x03,0xD6,
        0x6B,0x83,0xAA,0x03,0x02,0x07,0,0x56};
    uint8_t group[24];vision_camera_state_t state;vision_ring_axis_t a;
    vision_pixel_t pixel;vision_ring_count_t count;uint32_t age,obs;unsigned i;
    tx_complete=1;tx_fail=0;tick=65000;
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    CHECK(Vision_RequestMode(1,3,1,2,20,1)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,3,2,0,20,1)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,3,3,2,20,4)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,2,1,2,20,2)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,3,3,2,20,3)==HAL_OK);Vision_Service();ack_last(0);
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.flags==3 && tx_data[0][6]==3);
    CHECK(Vision_RequestMode(1,3,3,2,20,2)==HAL_OK);Vision_Service();ack_last(0);
    CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.flags==2);
    for(i=0;i<16;i++) {
        CHECK(Vision_RequestMode(1,3,2,2,20,1)==HAL_OK);Vision_Service();ack_last(0);
        CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.flags==1);
        if(state.request_seq==7)break;
    }
    CHECK(i<16 && memcmp(tx_data[0],golden_set,8)==0);
    send_bytes(0,golden_ack,8);Vision_Service();
    memcpy(group,golden_group,24);
    send_bytes(0,group,8);Vision_Service();CHECK(!Vision_GetRingSceneWindow(1,500,&pixel,&count,&age));
    send_bytes(0,group+8,8);Vision_Service();CHECK(!Vision_GetRingAxisWindow(1,500,&a,&age));
    send_bytes(0,group+16,8);Vision_Service();
    CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && a.valid && fabsf(a.axis_deg+1.25f)<0.001f && a.confidence==88 && a.landmarks==3);
    CHECK(Vision_GetRingSceneWindow(1,500,&pixel,&count,&age) && pixel.valid && fabsf(pixel.dx_px-12.3f)<0.001f && fabsf(pixel.dy_px+4.5f)<0.001f);
    CHECK(a.observation_id==count.observation_id && a.station==2 && a.request_seq==7);
    obs=a.observation_id;tick+=20;send_bytes(0,group,24);Vision_Service();
    CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && a.valid && a.observation_id==obs && age==20);
    tick+=60;send_bytes(0,group,24);Vision_Service(); /* Same wire SEQ after missed 16 frames. */
    CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && a.valid && a.observation_id!=obs && age==0);
    /* Independent invalid axis leaves valid same-image XY. */
    tick+=100;for(i=0;i<3;i++){group[i*8+2]=(uint8_t)((group[i*8+2]&0x87U)|(6U<<3));group[i*8+7]=test_crc8(group+i*8,7);}
    group[10]&=0x7FU;memset(group+11,0,4);group[15]=test_crc8(group+8,7);
    send_bytes(0,group,24);Vision_Service();
    CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && !a.valid);
    CHECK(Vision_GetRingSceneWindow(1,500,&pixel,&count,&age) && pixel.valid && count.valid);
    /* Missing/out-of-order member cannot revive the previous angle. */
    tick+=100;memcpy(group,golden_group,24);for(i=0;i<3;i++){group[i*8+2]=(uint8_t)(0x80U|(7U<<3)|2U);group[i*8+7]=test_crc8(group+i*8,7);}
    send_bytes(0,group,8);send_bytes(0,group+16,8);Vision_Service();CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && !a.valid);
    tick+=100;send_bytes(0,group,24);Vision_Service();CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && a.valid);
    /* -90 degrees is legal; +90, bad confidence/source and invalid payload are not. */
    tick+=100;for(i=0;i<3;i++){group[i*8+2]=(uint8_t)(0x80U|(8U<<3)|2U);group[i*8+7]=test_crc8(group+i*8,7);}
    group[11]=0xD8;group[12]=0xDC;group[13]=90;group[15]=test_crc8(group+8,7);
    send_bytes(0,group,24);Vision_Service();CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && a.valid && a.axis_deg==-90.0f);
    tick+=500;CHECK(Vision_GetRingAxisWindow(1,500,&a,&age) && !a.valid);
    for(i=0;i<4;i++) {
        unsigned j;uint32_t errors=Vision_GetParseErrorCount();tick+=100;
        for(j=0;j<3;j++)group[j*8+2]=(uint8_t)(0x80U|((9U+i)<<3)|2U);
        group[11]=0x83;group[12]=0xFF;group[13]=88;group[14]=3;
        if(i==0){group[11]=0x28;group[12]=0x23;} /* +90 forbidden */
        if(i==1)group[13]=101;
        if(i==2)group[14]=2;
        if(i==3)group[10]&=0x7FU; /* INVALID with nonzero payload forbidden */
        for(j=0;j<3;j++)group[j*8+7]=test_crc8(group+j*8,7);
        send_bytes(0,group,24);Vision_Service();
        CHECK(Vision_GetParseErrorCount()>errors && Vision_GetRingAxisWindow(1,500,&a,&age) && !a.valid);
    }

    CHECK(Vision_ClearPixelObservations(1));Vision_Service();
    send_bytes(0,group,8);Vision_Service();tick+=51;send_bytes(0,group+8,16);Vision_Service();
    CHECK(!Vision_GetRingSceneWindow(1,500,&pixel,&count,&age));
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55;Vision_Service();ack_last(0);
    CHECK(!Vision_GetRingAxisWindow(1,500,&a,&age));
}
static void plate_scene_pair_tests(void){
    vision_camera_state_t state;vision_pixel_t pixel;vision_plate_scene_t scene;
    uint8_t xy[8]={0x6B,VISION_TYPE_PLATE_PX,0,0,0,0,0,0},status[8]={0x6B,VISION_TYPE_PLATE_SCENE,0,1,1,0,0,0};
    uint8_t hb[8]={0x6B,0x82,0x80,1,8,1,0,0};uint32_t age,observation,errors;
    tick=60000U;tx_complete=1;tx_fail=0;CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    CHECK(Vision_RequestMode(1,VISION_MODE_PLATE,1,0,20,0)==HAL_OK);Vision_Service();ack_last(0);
    hb[7]=test_crc8(hb,7);send_bytes(0,hb,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state));
    xy[2]=5U<<3;xy[7]=test_crc8(xy,7);status[2]=(uint8_t)(0x80U|xy[2]);status[5]=state.request_seq;status[7]=test_crc8(status,7);
    send_bytes(0,status,8);Vision_Service();CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    tick+=10U;send_bytes(0,xy,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid && scene.status==1U && !pixel.valid && age==10U);
    observation=scene.observation_id;tick+=80U;send_bytes(0,xy,8);send_bytes(0,status,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.observation_id==observation && age==90U);
    /* Acquisition state overrides an incorrectly VALID pixel; both orders work. */
    xy[2]=(uint8_t)(0x80U|(6U<<3));xy[7]=test_crc8(xy,7);status[2]=xy[2];status[3]=2U;status[7]=test_crc8(status,7);
    send_bytes(0,xy,8);send_bytes(0,status,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid && scene.status==2U && !pixel.valid);
    xy[2]=(uint8_t)(0x80U|(7U<<3));xy[3]=123U;xy[7]=test_crc8(xy,7);
    status[2]=xy[2];status[3]=3U;status[7]=test_crc8(status,7);
    send_bytes(0,xy,8);send_bytes(0,status,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid && pixel.valid && fabsf(pixel.dx_px-12.3f)<0.01f);
    tick+=250U;CHECK(Vision_GetPixel(1,VISION_TYPE_PLATE_PX,&pixel,&age) && !pixel.valid);
    CHECK(Vision_GetPixelWindow(1,VISION_TYPE_PLATE_PX,400U,&pixel,&age) && pixel.valid && age==250U);
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid && pixel.valid && age==250U);
    tick+=150U;CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && !pixel.valid && !scene.valid && age==400U);
    CHECK(!Vision_GetPlateSceneWindow(1,501U,&pixel,&scene,&age) && !Vision_GetPixelWindow(1,VISION_TYPE_PLATE_PX,0U,&pixel,&age));
    CHECK(Vision_ClearPixelObservations(1));Vision_Service();
    xy[2]=(uint8_t)(0x80U|(8U<<3));xy[7]=test_crc8(xy,7);status[2]=xy[2];status[4]=2U;status[7]=test_crc8(status,7);
    send_bytes(0,xy,8);send_bytes(0,status,8);Vision_Service();CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    status[4]=1U;status[5]=(uint8_t)((state.request_seq+1U)&15U);status[7]=test_crc8(status,7);
    send_bytes(0,status,8);Vision_Service();CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    status[5]=state.request_seq;status[7]=test_crc8(status,7);send_bytes(0,status,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid);
    CHECK(Vision_ClearPixelObservations(1));Vision_Service();
    xy[2]=(uint8_t)(0x80U|(9U<<3));xy[7]=test_crc8(xy,7);status[2]=xy[2];status[7]=test_crc8(status,7);
    send_bytes(0,xy,8);Vision_Service();tick+=51U;send_bytes(0,status,8);Vision_Service();
    CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    send_bytes(0,xy,8);Vision_Service();CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && scene.valid);
    /* Explicit unhealthy scene is an immediate invalidation, never SEARCH. */
    xy[2]=10U<<3;memset(xy+3,0,4);xy[7]=test_crc8(xy,7);status[2]=xy[2];status[3]=0U;status[7]=test_crc8(status,7);
    send_bytes(0,status,8);send_bytes(0,xy,8);Vision_Service();
    CHECK(Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age) && !scene.valid && scene.status==0U && !pixel.valid);
    errors=Vision_GetParseErrorCount();status[2]|=0x80U;status[7]=test_crc8(status,7);
    send_bytes(0,status,8);Vision_Service();CHECK(Vision_GetParseErrorCount()>errors); /* trusted UNKNOWN is malformed */
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55U;Vision_Service();ack_last(0);
    CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,0)==HAL_OK);Vision_Service();ack_last(0);
    status[2]=xy[2];status[3]=0U;status[7]=test_crc8(status,7);
    send_bytes(0,xy,8);send_bytes(0,status,8);Vision_Service();CHECK(!Vision_GetPlateSceneWindow(1,400U,&pixel,&scene,&age));
    CHECK(!Vision_GetPlateSceneWindow(2,400U,&pixel,&scene,&age));
}
static void reset_camera_tests(void){
    vision_camera_state_t state;vision_pixel_t pixel;uint8_t old[8],packet[8];
    unsigned int calls,abort_calls;uint8_t stop_seq;
    tx_complete=1;tx_fail=0;tick=30000U;
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    CHECK(Vision_RequestStop(3)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,0)==HAL_OK);Vision_Service();ack_last(0);
    send_bytes(0,vector_hb_ring,8);
    memcpy(packet,vector_ring,8);packet[2]=(uint8_t)((packet[2]&0x87U)|(6U<<3));packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid);
    tx_complete=0;
    CHECK(Vision_RequestMode(1,VISION_MODE_PLATE,1,0,20,0)==HAL_OK);Vision_Service();
    memcpy(old,tx_pointer[0],8);calls=tx_calls[0];
    CHECK(Vision_RequestStop(1)==HAL_OK);
    CHECK(memcmp(old,tx_pointer[0],8)==0); /* UART owns this buffer */
    CHECK(Vision_GetCameraState(1,&state) && state.reset_pending && state.pending && !state.confirmed);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && !pixel.valid);
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,0)==HAL_BUSY);
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();CHECK(tx_calls[0]==calls);
    /* Late ACK from cancelled PLATE cannot restore the old mode. */
    packet[0]=0x6B;packet[1]=0x81;packet[2]=(uint8_t)(0x80U|old[2]);packet[3]=0x80;
    packet[4]=0;packet[5]=8;packet[6]=1;packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state) && state.reset_pending && !state.confirmed);
    HAL_UART_TxCpltCallback(&huart1);tx_complete=1;Vision_Service();
    tick+=55U;Vision_Service();CHECK(tx_data[0][3]==0 && tx_data[0][4]==0 && tx_data[0][5]==0);
    CHECK(tx_data[0][2]!=old[2] && test_crc8(tx_data[0],7)==tx_data[0][7]);
    CHECK(Vision_GetCameraState(1,&state));stop_seq=state.request_seq;
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();CHECK(Vision_GetCameraState(1,&state) && state.request_seq==stop_seq);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetCameraState(1,&state) && state.pending && state.reset_pending);
    ack_last(0);send_bytes(0,vector_hb_stop,8);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && !state.pending && !state.reset_pending && state.confirmed && state.mode==0);
    calls=tx_calls[0];CHECK(Vision_RequestStop(1)==HAL_OK);tick+=100U;Vision_Service();CHECK(tx_calls[0]==calls);
    CHECK(Vision_RequestMode(1,VISION_MODE_RING,2,2,20,0)==HAL_OK);Vision_Service();
    CHECK(Vision_GetCameraState(1,&state) && state.request_seq!=stop_seq);ack_last(0);
    send_bytes(0,vector_hb_ring,8);memcpy(packet,vector_ring,8);
    packet[2]=(uint8_t)((packet[2]&0x87U)|(7U<<3));packet[7]=test_crc8(packet,7);
    send_bytes(0,packet,8);Vision_Service();CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid);
    /* A stuck old TX is aborted by the task; failure stays blocked until a
     * new explicit reset succeeds. Reset of CAM2 does not invalidate CAM1. */
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_BINARY8));Vision_Service();tx_complete=0;
    CHECK(Vision_RequestMode(2,VISION_MODE_PLATE,1,0,20,0)==HAL_OK);Vision_Service();
    abort_calls=tx_aborts[1];CHECK(Vision_RequestStop(2)==HAL_OK);
    CHECK(Vision_GetPixel(1,VISION_TYPE_RING_PX,&pixel,NULL) && pixel.valid);
    tick+=51U;Vision_Service();CHECK(tx_aborts[1]==abort_calls+1U);
    tx_complete=1;tick+=55U;Vision_Service();CHECK(tx_data[1][3]==0);
    tick+=500U;Vision_Service();tick+=500U;Vision_Service();tick+=500U;Vision_Service();
    CHECK(Vision_GetCameraState(2,&state) && !state.pending && state.reset_pending && state.result==VISION_RESULT_TIMEOUT);
    CHECK(Vision_RequestMode(2,VISION_MODE_RING,2,2,20,0)==HAL_BUSY);
    ack_last(1);CHECK(Vision_GetCameraState(2,&state) && state.reset_pending && !state.confirmed);
    CHECK(Vision_RequestStop(2)==HAL_OK);Vision_Service();tick+=55U;Vision_Service();ack_last(1);
    CHECK(Vision_GetCameraState(2,&state) && !state.reset_pending && state.confirmed && state.mode==0);
}
static void qr_binary_tests(void)
{
    static const uint8_t groups[4][3]={{1,6,5},{1,2,3},{6,1,5},{2,3,1}};
    const uint8_t *generated[4]={vector_qr_group1,vector_qr_group2,vector_qr_group3,vector_qr_group4};
    vision_camera_state_t s={0};vision_qr_task_t task;uint32_t age,obs,errors;uint8_t p[8],ack[8],i,seq;
    tick=500000U;CHECK(Vision_SetCameraProtocol(2U,VISION_PROTOCOL_BINARY8));Vision_Service();
    {uint8_t hb[8]={0x6B,0x82,0x80,1,0,0,0,0};hb[7]=test_crc8(hb,7);send_bytes(1,hb,8);Vision_Service();}
    CHECK(Vision_RequestMode(1U,VISION_MODE_QR,4U,0U,20U,0U)==HAL_ERROR);
    CHECK(Vision_RequestMode(2U,VISION_MODE_QR,1U,0U,20U,0U)==HAL_ERROR);
    CHECK(Vision_RequestMode(2U,VISION_MODE_QR,4U,0U,20U,0U)==HAL_OK);Vision_Service();
    CHECK(Vision_GetCameraState(2U,&s));seq=s.request_seq;
    for(i=1U;i<=4U;i++) {
        memcpy(p,generated[i-1U],8U);CHECK(memcmp(p+3,groups[i-1U],3U)==0 && p[6]==4U);
        p[2]=(uint8_t)(0x80U|(seq<<3U)|i);p[7]=test_crc8(p,7);
        send_bytes(1,p,8U);Vision_Service();
    }
    CHECK(!Vision_GetQrTask(&task,NULL)); /* no ACK, no business result */
    ack[0]=0x6B;ack[1]=0x81;ack[2]=(uint8_t)(0x80U|(seq<<3U));ack[3]=0x80;
    ack[4]=0U;ack[5]=6U;ack[6]=4U;ack[7]=test_crc8(ack,7);
    send_bytes(1,ack,8U);Vision_Service();CHECK(Vision_GetCameraState(2U,&s) && s.confirmed);
    for(i=1U;i<=4U;i++) {
        p[2]=(uint8_t)(0x80U|(seq<<3U)|i);memcpy(p+3,groups[i-1U],3U);p[7]=test_crc8(p,7);
        send_bytes(0,p,8U);Vision_Service();
    }
    CHECK(!Vision_GetQrTask(&task,NULL)); /* CAM1 cannot fill CAM2 groups */
    for(i=1U;i<=4U;i++) {
        p[2]=(uint8_t)(0x80U|(seq<<3U)|i);memcpy(p+3,groups[i-1U],3U);p[7]=test_crc8(p,7);
        tick+=5U;send_bytes(1,p,8U);Vision_Service();
        CHECK(Vision_GetQrTask(&task,&age)==(i==4U));
    }
    CHECK(strcmp(task.raw,"156+123+516+231")==0 && task.colors[0][1]==6U && task.positions[1][0]==2U);
    obs=task.observation_id;tick+=20U;
    send_bytes(1,p,8U);Vision_Service();
    CHECK(Vision_GetQrTask(&task,&age) && task.observation_id==obs && age==20U);
    errors=Vision_GetParseErrorCount();p[2]=(uint8_t)(0x80U|(seq<<3U)|1U);
    memcpy(p+3,groups[0],3U);p[3]=2U;p[7]=test_crc8(p,7);send_bytes(1,p,8U);Vision_Service();
    CHECK(Vision_GetParseErrorCount()>errors && Vision_GetQrTask(&task,NULL) && task.observation_id==obs);
    CHECK(strcmp(task.raw,"156+123+516+231")==0);
    CHECK(Vision_RequestStop(2U)==HAL_OK && !Vision_GetQrTask(&task,NULL));Vision_Service();
    CHECK(Vision_GetCameraState(2U,&s));ack[2]=(uint8_t)(0x80U|(s.request_seq<<3U));ack[5]=ack[6]=0U;ack[7]=test_crc8(ack,7);
    send_bytes(1,ack,8U);Vision_Service();
    CHECK(Vision_RequestMode(2U,6U,4U,0U,20U,0U)==HAL_OK);Vision_Service();CHECK(Vision_GetCameraState(2U,&s));
    ack[2]=(uint8_t)(0x80U|(s.request_seq<<3U));ack[5]=6U;ack[6]=4U;ack[7]=test_crc8(ack,7);
    send_bytes(1,ack,8U);Vision_Service();
    p[2]=(uint8_t)(0x80U|(seq<<3U)|1U);memcpy(p+3,groups[0],3U);p[7]=test_crc8(p,7);
    send_bytes(1,p,8U);Vision_Service();CHECK(!Vision_GetQrTask(&task,NULL)); /* old session */
    seq=s.request_seq;
    for(i=1U;i<=3U;i++) {
        p[2]=(uint8_t)(0x80U|(seq<<3U)|i);memcpy(p+3,groups[i-1U],3U);p[7]=test_crc8(p,7);
        send_bytes(1,p,8U);Vision_Service();
    }
    tick+=51U;p[2]=(uint8_t)(0x80U|(seq<<3U)|4U);memcpy(p+3,groups[3],3U);p[7]=test_crc8(p,7);
    send_bytes(1,p,8U);Vision_Service();CHECK(!Vision_GetQrTask(&task,NULL));
}
static void color_learning_tests(void)
{
    vision_camera_state_t state;vision_pixel_t pixel;unsigned target;
    tx_complete=1;tx_fail=0;tick=140000;
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
    CHECK(Vision_RequestMode(2,2,1,1,20,VISION_MATERIAL_CALIBRATION_FLAG)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,2,2,1,20,VISION_MATERIAL_CALIBRATION_FLAG)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,3,2,2,20,VISION_MATERIAL_CALIBRATION_FLAG)==HAL_ERROR);
    CHECK(Vision_RequestMode(1,2,1,1,20,VISION_MATERIAL_CALIBRATION_FLAG|1U)==HAL_ERROR);
    for(target=0;target<=6;target++) {
        CHECK(Vision_RequestMode(1,2,1,(uint8_t)target,20,VISION_MATERIAL_CALIBRATION_FLAG)==HAL_OK);
        Vision_Service();ack_last(0);
        CHECK(tx_data[0][6]==VISION_MATERIAL_CALIBRATION_FLAG);
        CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.flags==VISION_MATERIAL_CALIBRATION_FLAG);
        send_bytes(0,vector_material,8);Vision_Service();
        (void)Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL);CHECK(!pixel.valid);
    }
    CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55;Vision_Service();ack_last(0);
    CHECK(Vision_GetCameraState(1,&state) && state.mode==VISION_MODE_STOP);
    CHECK(Vision_RequestMode(1,2,1,3,20,0)==HAL_OK);Vision_Service();ack_last(0);
    send_bytes(0,vector_material,8);Vision_Service();
    CHECK(Vision_GetPixel(1,VISION_TYPE_MAT_PX,&pixel,NULL) && pixel.valid);
}
static void parking_mode_tests(void)
{
 uint8_t group[24]={0x6B,0x03,0xAA,0,0,0,0,0,0x6B,0x0A,0xAA,0,0,95,3,0,0x6B,0x83,0xAA,3,5,0,0,0};
 vision_camera_state_t state;vision_pixel_t pixel;vision_ring_count_t scene;vision_ring_axis_t axis;uint32_t age;unsigned i;
 tx_complete=1;tx_fail=0;tick+=1000;
 CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_BINARY8));Vision_Service();
 CHECK(Vision_RequestMode(1,VISION_MODE_PARK,2,2,20,1)==HAL_ERROR);
 CHECK(Vision_RequestMode(2,VISION_MODE_PARK,5,2,20,1)==HAL_ERROR);
 CHECK(Vision_RequestMode(1,VISION_MODE_PARK,5,2,20,1)==HAL_OK);Vision_Service();ack_last(0);
 CHECK(Vision_GetCameraState(1,&state) && state.confirmed && state.mode==9 && state.station==5);
 group[21]=state.request_seq;
 for(i=0;i<3;i++)group[i*8+7]=test_crc8(group+i*8,7);
 send_bytes(0,group,24);Vision_Service();
 CHECK(Vision_GetRingSceneWindow(1,400,&pixel,&scene,&age) && pixel.valid && scene.valid && scene.station==5);
 CHECK(Vision_GetRingAxisWindow(1,400,&axis,&age) && axis.valid && axis.observation_id==scene.observation_id);
 CHECK(Vision_RequestStop(1)==HAL_OK);Vision_Service();tick+=55;Vision_Service();ack_last(0);
 CHECK(!Vision_GetRingSceneWindow(1,400,&pixel,&scene,&age));
}
int main(void){
    HAL_UART_RxCpltCallback(&huart3);HAL_UART_TxCpltCallback(&huart3);HAL_UART_ErrorCallback(&huart3);
    CHECK(screen_rx_callbacks==1 && screen_tx_callbacks==1 && screen_error_callbacks==1);

    unsigned int i;char digits[16];uint16_t seq;uint32_t age,before;vision_port_stats_t s;
    vision_ring_t r;vision_color_t c;vision_obs_t o;
    huart1.Instance=&regs[0];huart2.Instance=&regs[1];huart10.Instance=&regs[2];
    huart1.hdmarx=&dmas[0];huart2.hdmarx=&dmas[1];huart10.hdmarx=&dmas[2];
    for(i=0;i<3;i++)dmas[i].Instance=&dma_regs[i];
    fail_cam1=1;CHECK(Vision_Init()==HAL_ERROR);
    CHECK(receives[1]==1U && receives[2]==1U && buffers[2]!=NULL); /* IMU still starts */
    fail_cam1=0;Vision_Service();CHECK(Vision_GetPortStats(1,&s) && s.init_status==HAL_OK && s.recoveries==1U);
    before=receives[0];(void)Vision_Init();CHECK(receives[0]==before); /* idempotent */
    CHECK(Vision_SetCameraProtocol(1,VISION_PROTOCOL_ASCII));
    CHECK(Vision_SetCameraProtocol(2,VISION_PROTOCOL_ASCII));Vision_Service();
    if(setjmp(task_return)==0)Vision_TaskRun(NULL); /* one real task iteration */
    tick=100U;frame(0,"QR,123+321,seq=1");
    CHECK(!Vision_GetQR(digits,&seq,&age)); /* callback only copied bytes */
    CHECK(notifications>0U);Vision_Service();
    CHECK(Vision_GetQR(digits,&seq,&age) && strcmp(digits,"123+321")==0 && seq==1U && age==0U);
    tick=125U;CHECK(Vision_GetQR(digits,NULL,&age) && age==25U);
    frame(1,"RING,id=3,x=12.4,y=-1.8,theta=0.6,conf=0.94,seq=21");Vision_Service();
    CHECK(Vision_GetRing(&r,&age) && r.id==3U && r.seq==21U);
    CHECK(fabsf(r.x_mm-12.4f)<0.001f && fabsf(r.y_mm+1.8f)<0.001f);
    CHECK(fabsf(r.theta_rad-0.6f)<0.001f && fabsf(r.conf-0.94f)<0.001f);
    frame(1,"COLOR,id=2,color=6,conf=0.91,seq=22");Vision_Service();
    CHECK(Vision_GetColor(&c,&age) && c.color==6 && c.id==2 && c.seq==22);
    frame(1,"OBS,front=400,left=9999,right=250,valid=1,seq=23");Vision_Service();
    CHECK(Vision_GetObs(&o,&age) && o.front_mm==400 && o.left_mm==9999 && o.right_mm==250 && o.valid==1 && o.seq==23);
    before=Vision_GetParseErrorCount();
    send_bytes(0,(const uint8_t *)"@QR,123,seq=7*00\r\n",18U);Vision_Service();
    CHECK(Vision_GetParseErrorCount()>before && Vision_GetQR(digits,&seq,NULL) && seq==1U);
    frame(1,"RING,id=3,x=bad,y=0,theta=0,conf=1,seq=30");Vision_Service();
    CHECK(Vision_GetRing(&r,NULL) && r.seq==21U); /* malformed number never publishes */
    frame(1,"RING,id=3,x=0,y=0,theta=0,seq=30");Vision_Service();
    CHECK(Vision_GetRing(&r,NULL) && r.seq==21U); /* incomplete packet */
    {
        uint8_t fill[200];memset(fill,'x',sizeof(fill));
        for(i=0;i<10;i++){send_bytes(0,fill,sizeof(fill));Vision_Service();}
    }
    frame(0,"QR,156+123+516+231,seq=12");Vision_Service();
    CHECK(Vision_GetQR(digits,&seq,NULL) && seq==12U && strcmp(digits,"156+123+516+231")==0); /* DMA wrap */
    CHECK(Vision_GetPortStats(1,&s) && s.dropped_bytes==0U && s.rx_bytes>2000U);
    before=receives[0];HAL_UART_ErrorCallback(&huart1);CHECK(receives[0]==before); /* no restart in camera IRQ */
    tick=300U;Vision_Service();CHECK(receives[0]==before+1U && aborts[0]>0U);
    frame(0,"QR,321+123,seq=13");Vision_Service();CHECK(Vision_GetQR(digits,&seq,NULL) && seq==13);
    {
        uint8_t fill[200];memset(fill,'z',sizeof(fill));
        for(i=0;i<25;i++)send_bytes(0,fill,sizeof(fill));
        CHECK(Vision_GetPortStats(1,&s) && s.dropped_bytes>0U && s.queued_bytes==2048U);
        Vision_Service();CHECK(Vision_GetPortStats(1,&s) && s.queued_bytes==0U);
    }
    frame(0,"QR,123+321,seq=14");Vision_Service();CHECK(Vision_GetQR(digits,&seq,NULL) && seq==14);
    CHECK(Vision_ProcessCommand("&camraw 1"));clear_console();
    send_bytes(0,(const uint8_t *)"hello camera\r\n",14U);CHECK(console_length==0U);
    Vision_Service();CHECK(console_length==0U);Vision_DebugPoll(1);CHECK(console_length==0U);
    Vision_DebugPoll(0);CHECK(strstr(console,"[CAM1 RX] hello camera\\r\\n")!=NULL);
    CHECK(Vision_ProcessCommand("camraw 2"));clear_console();
    {
        const uint8_t bytes[]={0x00,0xff,'\r','\n','\\','A'};
        send_bytes(0,bytes,sizeof(bytes));send_bytes(1,bytes,sizeof(bytes));Vision_Service();Vision_DebugPoll(0);
        CHECK(strstr(console,"[CAM1 RX]")==NULL && strstr(console,"[CAM2 RX] \\x00\\xFF\\r\\n\\\\A")!=NULL);
    }
    CHECK(Vision_ProcessCommand("camraw all"));clear_console();
    send_bytes(0,(const uint8_t *)"one",3);send_bytes(1,(const uint8_t *)"two",3);Vision_Service();
    tick+=20;Vision_DebugPoll(0);tick+=20;Vision_DebugPoll(0);
    CHECK(strstr(console,"[CAM1 RX] one")!=NULL && strstr(console,"[CAM2 RX] two")!=NULL);
    for(i=0;i<30;i++){send_bytes(0,(const uint8_t *)"sample",6);Vision_Service();}
    clear_console();CHECK(Vision_ProcessCommand("camstat"));
    CHECK(strstr(console,"task=1")!=NULL && strstr(console,"raw_dropped=0 ")==NULL);
    CHECK(Vision_ProcessCommand("camraw off"));clear_console();
    send_bytes(0,(const uint8_t *)"quiet",5);Vision_Service();tick+=100;Vision_DebugPoll(0);CHECK(console_length==0U);
    CHECK(!Vision_ProcessCommand("camrawx 1"));
    CHECK(!Vision_GetPortStats(3,&s) && !Vision_GetPortStats(1,NULL));
    CHECK(!Vision_GetQR(NULL,NULL,NULL) && !Vision_GetRing(NULL,NULL) && !Vision_GetColor(NULL,NULL) && !Vision_GetObs(NULL,NULL));
    __disable_irq();CHECK(Vision_GetQR(digits,NULL,NULL));CHECK(__get_PRIMASK()==1U);__enable_irq();
    {
        uint8_t fill[200];memset(fill,0x55,sizeof(fill));before=hwt_bytes;
        send_bytes(2,fill,sizeof(fill));CHECK(hwt_bytes-before==sizeof(fill)); /* IMU still handled immediately */
    }
    binary_tests();
    control_tests();
    mode_alignment_integration_tests();
    reset_camera_tests();
    ring_scene_pair_tests();
    ring_green_scene_tests();
    plate_scene_pair_tests();
    qr_binary_tests();
    ring_axis_triple_tests();
    color_learning_tests();
    pc_obstacle_tests();
    parking_mode_tests();
    HAL_UART_TxCpltCallback(&huart4);CHECK(log_tx==1);
    huart4.ErrorCode=HAL_UART_ERROR_ORE;HAL_UART_ErrorCallback(&huart4);CHECK(!log_errors);
    huart4.ErrorCode=HAL_UART_ERROR_DMA;HAL_UART_ErrorCallback(&huart4);CHECK(log_errors==1);
    HAL_UART_AbortTransmitCpltCallback(&huart4);CHECK(log_aborts==1);
    return test_failures?1:0;
}
