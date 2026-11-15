/* Camera DMA transport, deferred parsing and bounded raw console monitoring.
 * USART10/HWT101 retains its existing ISR path to preserve chassis timing. */
#include "vision.h"
#include "vision_obstacle.h"
#include "usart.h"
#include "screen_hmi.h"
#include "console_dma.h"
#include "cmsis_os2.h"
#include "zdt_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DMA_SIZE 512U
#define RX_SIZE 2048U
#define FRAME_SIZE 128U
#define RAW_SIZE 48U
#define RAW_SLOTS 16U
/* ARMCC5 absolute DMA placement matches the existing Keil project. GNU builds
 * need these sections placed in DMA-accessible RAM by their linker script. */
#if defined(__CC_ARM)
#define DMA_AT(addr, name) __attribute__((at(addr)))
#else
#define DMA_AT(addr, name) __attribute__((section(name), aligned(32)))
#endif
static volatile uint8_t rx1[DMA_SIZE] DMA_AT(0x24010000, ".vision_dma1");
static volatile uint8_t rx2[DMA_SIZE] DMA_AT(0x24010200, ".vision_dma2");
static volatile uint8_t rx10[128] DMA_AT(0x24010400, ".vision_dma10");
static volatile uint16_t hwt_last;
static osThreadId_t volatile vision_thread;

typedef struct {
    UART_HandleTypeDef *uart;
    volatile uint8_t *dma;
    uint16_t last;
    uint8_t ring[RX_SIZE];
    uint32_t head, tail, bytes, dropped, errors, recoveries, last_tick;
    uint8_t restart, status, protocol;
    uint32_t generation;
} camera_port_t;
/* All state touched by an ISR is volatile; head/tail use a single producer
 * and consumer. The fixed software rings are CPU-only, not DMA buffers. */
static volatile camera_port_t cam1={.uart=&huart1,.dma=rx1,.protocol=VISION_CAMERA1_PROTOCOL};
static volatile camera_port_t cam2={.uart=&huart2,.dma=rx2,.protocol=VISION_CAMERA2_PROTOCOL};
/* Captured in the DMA ISR for CAM2 only; lets the main task account for the
 * full VisionTask RX queue delay when applying an obstacle observation. */
static volatile uint32_t cam2_rx_tick[RX_SIZE];
typedef struct { uint8_t data[FRAME_SIZE]; uint16_t used; uint8_t active; } parser_t;
static parser_t parsers[2];
typedef struct { uint8_t data[8], used; uint32_t tick,rx_tick[8]; } binary_parser_t;
static binary_parser_t binary_parsers[2];
static vision_pc_obstacle_t pc_obstacle;
static uint32_t pc_obstacle_id,pc_obstacle_clear_tick;
static uint8_t pc_obstacle_seen;
static uint32_t seen_generation[2];
static vision_pixel_t pixels[2][3];
static uint32_t pixel_ticks[2][3];
static uint32_t pixel_observation_id[2];
static uint8_t pixel_seen[2][3], pixel_context_type[2], pixel_context_id[2];
typedef struct {
    vision_pixel_t pixel;
    vision_ring_count_t count;
    vision_ring_axis_t axis;
    uint32_t tick,generation;
    uint8_t parts,seq,request_seq,station;
} ring_pair_t;
static ring_pair_t ring_pairs[2];
static vision_pixel_t ring_scene_pixels[2];
static vision_ring_count_t ring_scene_counts[2];
static uint32_t ring_scene_ticks[2],ring_scene_ids[2];
static uint8_t ring_scene_seen[2];
static vision_ring_axis_t ring_axes[2];
typedef struct {
    vision_pixel_t pixel;vision_plate_scene_t scene;
    uint32_t tick,generation;uint8_t parts,seq,request_seq,station;
} plate_pair_t;
static plate_pair_t plate_pairs[2];
static vision_pixel_t plate_scene_pixels[2];
static vision_plate_scene_t plate_scenes[2];
static uint32_t plate_scene_ticks[2],plate_scene_ids[2];
static uint8_t plate_scene_seen[2];
typedef struct {
    uint8_t request[8], tx[8], tx_busy;
    uint8_t managed, pending, confirmed, previous_confirmed, result, seq, next_seq, attempts;
    uint8_t mode, station, target, hz, flags;
    uint8_t hb_seen, hb_version, hb_mode, hb_station;
    uint8_t stop_requested,stop_blocked;
    uint32_t next_send, tx_tick, peer_tick, hb_tick;
    uint32_t reset_tick;
} camera_control_t;
/* TX completion ISR touches tx_busy; shared controller state is volatile. */
static volatile camera_control_t controllers[2];
static uint32_t seen_drops[2], retry_tick[2];
static vision_qr_t qr;
static vision_ring_t ring;
static vision_color_t color;
static vision_obs_t obs;
static uint32_t qr_tick,ring_tick,color_tick,obs_tick,parse_errors;
static uint8_t qr_valid,ring_valid,color_valid,obs_valid;
static uint8_t qr_source_camera,qr_task_seen;
static vision_qr_assembly_t qr_assembly;
static vision_qr_task_t qr_task;
static uint32_t qr_task_tick,qr_observation_id;
typedef struct { uint8_t camera,length,data[RAW_SIZE]; } raw_record_t;
static volatile raw_record_t raw_queue[RAW_SLOTS];
static volatile uint32_t raw_head,raw_tail,raw_dropped;
static volatile uint8_t raw_mode;
static uint32_t raw_print_tick;
static uint8_t initialized;
static HAL_StatusTypeDef initial_status;
extern void HWT101_OnDmaData(const uint8_t *data,uint16_t len);

static uint32_t lock(void) { uint32_t key=__get_PRIMASK();__disable_irq();return key; }
static void unlock(uint32_t key) { __set_PRIMASK(key); }
static volatile camera_port_t *port(uint8_t camera) { return camera==1U?&cam1:(camera==2U?&cam2:NULL); }
static volatile camera_port_t *find(UART_HandleTypeDef *uart) {
    return uart==&huart1?&cam1:(uart==&huart2?&cam2:NULL);
}
static void notify(void);
static uint8_t crc8(const uint8_t *data,uint8_t n);
static uint8_t mode_type(uint8_t mode) {
    return mode==VISION_MODE_MATERIAL?VISION_TYPE_MAT_PX:((mode==VISION_MODE_RING || mode==VISION_MODE_PARK)?VISION_TYPE_RING_PX:
           (mode==VISION_MODE_PLATE?VISION_TYPE_PLATE_PX:(mode==VISION_MODE_QR?VISION_TYPE_QR_GROUP:0)));
}
static uint8_t control_allows(uint8_t camera,uint8_t type,uint8_t id,uint32_t now) {
    volatile camera_control_t *c=&controllers[camera-1U];
    if(!c->managed)return 1;
    if(c->mode==VISION_MODE_MATERIAL && c->flags==VISION_MATERIAL_CALIBRATION_FLAG)return 0U;
    return (uint8_t)(!c->pending && !c->stop_blocked && c->confirmed && now-c->peer_tick<VISION_CAMERA_PEER_TIMEOUT_MS
            && mode_type(c->mode)==type && (c->target==0U || c->target==id));
}
static int pixel_index(uint8_t type) {
    return type==VISION_TYPE_MAT_PX?0:(type==VISION_TYPE_RING_PX?1:(type==VISION_TYPE_PLATE_PX?2:-1));
}
static void invalidate_pixels(uint8_t camera) {
    uint8_t i;uint32_t key=lock();
    if(camera==2U) {
        /* Pixel/session invalidation must also discard the independent PC
         * obstacle side channel; only a later RX frame can republish it. */
        pc_obstacle_seen=0U;pc_obstacle_clear_tick=HAL_GetTick();
        if(qr_task_seen && qr_source_camera==2U)qr_valid=0U;
        VisionQr_Reset(&qr_assembly);qr_task_seen=0U;
    }
    for(i=0;i<3U;i++) { pixels[camera-1U][i].valid=0;pixels[camera-1U][i].dx_px=0;pixels[camera-1U][i].dy_px=0; }
    pixel_context_type[camera-1U]=0;pixel_context_id[camera-1U]=0;
    ring_pairs[camera-1U].parts=0U;ring_scene_seen[camera-1U]=0U;
    ring_axes[camera-1U].valid=0U;
    plate_pairs[camera-1U].parts=0U;plate_scene_seen[camera-1U]=0U;unlock(key);
}
uint8_t Vision_SetCameraProtocol(uint8_t camera,uint8_t protocol) {
    volatile camera_port_t *p=port(camera);uint32_t key;
    if(p==NULL || (protocol!=VISION_PROTOCOL_BINARY8 && protocol!=VISION_PROTOCOL_ASCII))return 0;
    key=lock();
    if(controllers[camera-1U].tx_busy) { unlock(key);return 0; }
    controllers[camera-1U].managed=0;controllers[camera-1U].pending=0;controllers[camera-1U].confirmed=0;
    controllers[camera-1U].stop_requested=controllers[camera-1U].stop_blocked=0;
    p->protocol=protocol;p->generation++;p->tail=p->head;
    if(p->status==HAL_OK && p->uart->hdmarx!=NULL) {
        uint16_t pos=(uint16_t)(DMA_SIZE-__HAL_DMA_GET_COUNTER(p->uart->hdmarx));
        p->last=pos<DMA_SIZE?pos:0;
    }
    invalidate_pixels(camera);unlock(key);notify();return 1;
}
uint8_t Vision_GetPixelWindow(uint8_t camera,uint8_t type,uint16_t window_ms,vision_pixel_t *out,uint32_t *age) {
    int index=pixel_index(type);uint32_t key,elapsed;
    if(port(camera)==NULL || index<0 || out==NULL || window_ms<VISION_PIXEL_TIMEOUT_MS || window_ms>500U)return 0;
    key=lock();
    if(!pixel_seen[camera-1U][index]) { unlock(key);return 0; }
    *out=pixels[camera-1U][index];elapsed=HAL_GetTick()-pixel_ticks[camera-1U][index];
    if(!control_allows(camera,type,out->id,HAL_GetTick()) || port(camera)->restart ||
        port(camera)->status!=HAL_OK || port(camera)->dropped!=seen_drops[camera-1U]) {
        out->valid=0;out->observation_id=0U;
    }
    if(elapsed>=window_ms || !out->valid) {
        out->valid=0;out->dx_px=0;out->dy_px=0;
    }
    if(age)*age=elapsed;
    unlock(key);return 1;
}
uint8_t Vision_GetPixel(uint8_t camera,uint8_t type,vision_pixel_t *out,uint32_t *age)
{return Vision_GetPixelWindow(camera,type,VISION_PIXEL_TIMEOUT_MS,out,age);}
uint8_t Vision_GetPcObstacle(vision_pc_obstacle_t *out,uint32_t *age)
{
    uint32_t key=lock(),elapsed;
    if(!out || !pc_obstacle_seen) {unlock(key);return 0U;}
    *out=pc_obstacle;elapsed=HAL_GetTick()-pc_obstacle.rx_tick;
    if(age)*age=elapsed+(uint32_t)pc_obstacle.bridge_age_ms;
    unlock(key);return 1U;
}
void Vision_ClearPcObstacle(void)
{
    uint32_t key=lock();pc_obstacle_seen=0U;pc_obstacle_clear_tick=HAL_GetTick();unlock(key);
}
uint8_t Vision_ClearPixelObservations(uint8_t camera) {
    volatile camera_port_t *p=port(camera);uint32_t key;
    if(p==NULL)return 0U;
    key=lock();p->generation++;p->tail=p->head;
    if(p->status==HAL_OK && p->uart->hdmarx!=NULL) {
        uint16_t pos=(uint16_t)(DMA_SIZE-__HAL_DMA_GET_COUNTER(p->uart->hdmarx));
        p->last=pos<DMA_SIZE?pos:0U;
    }
    invalidate_pixels(camera);unlock(key);notify();return 1U;
}
static uint8_t crc8(const uint8_t *data,uint8_t n) {
    uint8_t result=0,i,j;
    for(i=0;i<n;i++) {
        result^=data[i];
        for(j=0;j<8U;j++) {
            uint8_t high=result&0x80U;result=(uint8_t)(result<<1);if(high)result^=0x07U;
        }
    }
    return result;
}
HAL_StatusTypeDef Vision_RequestStop(uint8_t camera) {
    volatile camera_port_t *p=port(camera);volatile camera_control_t *c;uint32_t key,now;
    if(p==NULL || !initialized || p->protocol!=VISION_PROTOCOL_BINARY8)return HAL_ERROR;
    c=&controllers[camera-1U];key=lock();now=HAL_GetTick();
    p->generation++;p->tail=p->head;
    if(p->status==HAL_OK && p->uart->hdmarx!=NULL) {
        uint16_t pos=(uint16_t)(DMA_SIZE-__HAL_DMA_GET_COUNTER(p->uart->hdmarx));
        p->last=pos<DMA_SIZE?pos:0U;
    }
    invalidate_pixels(camera);
    if(c->stop_requested || (c->stop_blocked && (c->pending || c->tx_busy))) {
        unlock(key);notify();return HAL_OK;
    }
    if(!c->pending && !c->tx_busy && c->managed && c->confirmed && c->mode==VISION_MODE_STOP &&
        now-c->peer_tick<VISION_CAMERA_PEER_TIMEOUT_MS) {
        unlock(key);notify();return HAL_OK;
    }
    c->managed=1;c->confirmed=0;c->previous_confirmed=0;c->pending=0;
    c->stop_requested=1;c->stop_blocked=1;c->result=255U;c->reset_tick=now;
    /* Do not mutate request/tx/tx_busy while a UART transfer owns the buffer. */
    unlock(key);notify();return HAL_OK;
}
HAL_StatusTypeDef Vision_RequestMode(uint8_t camera,uint8_t mode,uint8_t station,uint8_t target,uint8_t hz,uint8_t flags) {
    volatile camera_port_t *p=port(camera);volatile camera_control_t *c;uint8_t packet[8],seq,i;uint32_t key;
    if(p==NULL || !initialized || p->status!=HAL_OK || p->protocol!=VISION_PROTOCOL_BINARY8 || station>6U ||
       (flags!=0U && !((camera==1U && mode==VISION_MODE_PARK && station==5U && target==2U && flags==VISION_RING_AXIS_FLAG) || (mode==VISION_MODE_RING && target==2U && station>=2U && station<=3U &&
           !(flags&~(VISION_RING_AXIS_FLAG|VISION_RING_OCCUPIED_FLAG|VISION_RING_GREEN_SEARCH_FLAG)) &&
           (!(flags&VISION_RING_GREEN_SEARCH_FLAG) || station==3U)) ||
          (camera==1U && mode==VISION_MODE_MATERIAL && station==1U && flags==VISION_MATERIAL_CALIBRATION_FLAG))))return HAL_ERROR;
    if(mode!=VISION_MODE_STOP && mode!=VISION_MODE_MATERIAL && mode!=VISION_MODE_RING && mode!=VISION_MODE_PLATE && mode!=VISION_MODE_QR && mode!=VISION_MODE_PARK)return HAL_ERROR;
    if(mode==VISION_MODE_PARK && (camera!=1U || station!=5U || target!=2U || flags!=VISION_RING_AXIS_FLAG))return HAL_ERROR;
    if(mode==VISION_MODE_QR && (camera!=2U || station!=4U || target!=0U))return HAL_ERROR;
    if(target>6U || (mode==VISION_MODE_RING && target!=0U && target!=2U)
            || ((mode==VISION_MODE_PLATE || mode==VISION_MODE_STOP) && target!=0U)
            || (mode==VISION_MODE_STOP?(hz!=0U):(hz==0U || hz>50U)))return HAL_ERROR;
    c=&controllers[camera-1U];key=lock();
    if(c->pending || c->tx_busy || c->stop_blocked || c->stop_requested) { unlock(key);return HAL_BUSY; }
    seq=c->next_seq;c->next_seq=(uint8_t)((seq+1U)&15U);
    packet[0]=0x6BU;packet[1]=0x80U;packet[2]=(uint8_t)((seq<<3)|target);
    packet[3]=mode;packet[4]=station;packet[5]=hz;packet[6]=flags;packet[7]=crc8(packet,7);
    for(i=0;i<8U;i++)c->request[i]=packet[i];
    c->previous_confirmed=c->confirmed;c->confirmed=0;c->managed=1;c->pending=1;
    c->seq=seq;c->attempts=0;c->result=255U;c->next_send=HAL_GetTick();
    invalidate_pixels(camera);unlock(key);notify();return HAL_OK;
}
uint8_t Vision_GetCameraState(uint8_t camera,vision_camera_state_t *out) {
    volatile camera_control_t *c;uint32_t key,now;
    if(port(camera)==NULL || out==NULL)return 0;
    c=&controllers[camera-1U];key=lock();now=HAL_GetTick();
    out->mode=c->mode;out->station=c->station;out->target=c->target;out->hz=c->hz;out->flags=c->flags;
    out->pending=(uint8_t)(c->pending || c->stop_requested);out->reset_pending=c->stop_blocked;
    out->confirmed=(uint8_t)(!c->stop_blocked && c->confirmed && now-c->peer_tick<VISION_CAMERA_PEER_TIMEOUT_MS && !port(camera)->restart);
    out->result=c->result;out->request_seq=c->seq;out->attempts=c->attempts;
    out->heartbeat_seen=c->hb_seen;out->protocol_version=c->hb_version;
    out->heartbeat_age_ms=c->hb_seen?now-c->hb_tick:0xFFFFFFFFU;
    out->heartbeat_mode=c->hb_mode;out->heartbeat_station=c->hb_station;
    out->peer_age_ms=now-c->peer_tick;
    unlock(key);return 1;
}
uint8_t Vision_GetRingSceneWindow(uint8_t camera,uint16_t window_ms,vision_pixel_t *pixel,vision_ring_count_t *count,uint32_t *age) {
    uint32_t key,now,elapsed;uint8_t i;volatile camera_port_t *p=port(camera);
    volatile camera_control_t *c;
    if(!p || !pixel || !count || !age || window_ms<VISION_PIXEL_TIMEOUT_MS || window_ms>500U)return 0U;
    i=(uint8_t)(camera-1U);c=&controllers[i];key=lock();now=HAL_GetTick();
    memset(pixel,0,sizeof(*pixel));memset(count,0,sizeof(*count));*age=UINT32_MAX;
    if(!ring_scene_seen[i]){unlock(key);return 0U;}
    *pixel=ring_scene_pixels[i];*count=ring_scene_counts[i];elapsed=now-ring_scene_ticks[i];*age=elapsed;
    if(!control_allows(camera,VISION_TYPE_RING_PX,2U,now) ||
        c->station!=count->station || c->seq!=count->request_seq || p->restart ||
        p->status!=HAL_OK || p->protocol!=VISION_PROTOCOL_BINARY8 || p->dropped!=seen_drops[i])
        {count->valid=0U;count->green_visible=0U;count->observation_id=0U;pixel->observation_id=0U;}
    if(elapsed>=window_ms){count->valid=0U;count->green_visible=0U;}
    if(!(c->flags&VISION_RING_GREEN_SEARCH_FLAG))count->green_visible=0U;
    if(!count->valid || count->count!=3U)pixel->valid=0U;
    unlock(key);return 1U;
}
uint8_t Vision_GetRingScene(uint8_t camera,vision_pixel_t *pixel,vision_ring_count_t *count,uint32_t *age)
{return Vision_GetRingSceneWindow(camera,VISION_PIXEL_TIMEOUT_MS,pixel,count,age);}
uint8_t Vision_GetRingAxisWindow(uint8_t camera,uint16_t window_ms,vision_ring_axis_t *axis,uint32_t *age)
{
    vision_pixel_t pixel;vision_ring_count_t count;uint32_t key;uint8_t i;
    if(!axis || !age || camera<1U || camera>2U)return 0U;
    memset(axis,0,sizeof(*axis));*age=UINT32_MAX;
    /* Keep scene and axis reads in the same short critical section. */
    key=lock();i=(uint8_t)(camera-1U);
    if(!Vision_GetRingSceneWindow(camera,window_ms,&pixel,&count,age)){unlock(key);return 0U;}
    *axis=ring_axes[i];
    if(!(controllers[i].flags&VISION_RING_AXIS_FLAG) || !pixel.valid || !count.valid || count.count!=3U ||
       axis->observation_id!=count.observation_id || axis->seq!=count.seq ||
       axis->station!=count.station || axis->request_seq!=count.request_seq || *age>=window_ms ||
       (ring_pairs[i].parts && HAL_GetTick()-ring_pairs[i].tick>VISION_SCENE_PAIR_WINDOW_MS))
        axis->valid=0U;
    unlock(key);return 1U;
}
uint8_t Vision_GetPlateSceneWindow(uint8_t camera,uint16_t window_ms,vision_pixel_t *pixel,
                                  vision_plate_scene_t *scene,uint32_t *age)
{
    uint32_t key,now;uint8_t i;volatile camera_port_t *p=port(camera);volatile camera_control_t *c;
    if(!p || !pixel || !scene || !age || window_ms<VISION_PIXEL_TIMEOUT_MS || window_ms>500U)return 0U;
    i=(uint8_t)(camera-1U);c=&controllers[i];key=lock();now=HAL_GetTick();
    memset(pixel,0,sizeof(*pixel));memset(scene,0,sizeof(*scene));*age=UINT32_MAX;
    if(!plate_scene_seen[i]){unlock(key);return 0U;}
    *pixel=plate_scene_pixels[i];*scene=plate_scenes[i];*age=now-plate_scene_ticks[i];
    if(!control_allows(camera,VISION_TYPE_PLATE_PX,0U,now) || c->station!=scene->station ||
        c->seq!=scene->request_seq || p->restart || p->status!=HAL_OK ||
        p->protocol!=VISION_PROTOCOL_BINARY8 || p->dropped!=seen_drops[i]) {
        scene->valid=0U;scene->observation_id=0U;pixel->observation_id=0U;
    }
    if(*age>=window_ms)scene->valid=0U;
    if(!scene->valid || scene->status!=VISION_PLATE_FOUND)pixel->valid=0U;
    unlock(key);return 1U;
}
static uint8_t parse_control(uint8_t camera,const uint8_t *data,uint32_t generation,uint32_t now) {
    volatile camera_control_t *c=&controllers[camera-1U];uint32_t key;
    uint8_t seq=(data[2]>>3)&15U,id=data[2]&7U,valid=(data[2]>>7)&1U;
    if(data[1]==0x82U) {
        if(data[2]!=0x80U || data[4]>VISION_MODE_PARK || data[5]>6U || (data[6]&0xF8U))return 0;
        key=lock();
        if(port(camera)->generation!=generation) { unlock(key);return 1; }
        c->hb_seen=1;c->hb_version=data[3];c->hb_mode=data[4];c->hb_station=data[5];c->hb_tick=now;c->peer_tick=now;
        if(data[3]!=1U || (!c->pending && c->managed && (data[4]!=c->mode || data[5]!=c->station))) {
            c->confirmed=0;c->previous_confirmed=0;invalidate_pixels(camera);
        }
        unlock(key);return 1;
    }
    if(data[3]!=0x80U || data[4]>7U || valid!=(data[4]==0U) || data[5]>VISION_MODE_PARK || data[6]>6U || id>6U)return 0;
    key=lock();
    if(port(camera)->generation!=generation || !c->pending || c->seq!=seq || (c->request[2]&7U)!=id) { unlock(key);return 1; }
    if(valid) {
        if(data[5]!=c->request[3] || data[6]!=c->request[4]) { unlock(key);return 0; }
        c->mode=c->request[3];c->station=c->request[4];c->target=c->request[2]&7U;
        c->hz=c->request[5];c->flags=c->request[6];c->confirmed=(uint8_t)(!c->hb_seen || c->hb_version==1U);
        if(c->mode==VISION_MODE_STOP && c->confirmed)c->stop_blocked=0;
    } else {
        if(data[5]!=c->mode || data[6]!=c->station) { unlock(key);return 0; }
        c->confirmed=(uint8_t)(c->previous_confirmed && (!c->hb_seen || (c->hb_version==1U && c->hb_mode==c->mode && c->hb_station==c->station)));
    }
    c->pending=0;c->result=data[4];c->peer_tick=now;invalidate_pixels(camera);unlock(key);return 1;
}
static void control_service(uint8_t camera,uint32_t now) {
    volatile camera_control_t *c=&controllers[camera-1U];volatile camera_port_t *p=port(camera);
    uint32_t key;uint8_t i,send=0,abort_tx=0;
    key=lock();
    if(c->stop_blocked && (c->stop_requested || c->pending) &&
        now-c->reset_tick>=VISION_CAMERA_MODE_STAGE_TIMEOUT_MS) {
        c->stop_requested=0;c->pending=0;c->confirmed=0;c->result=VISION_RESULT_TIMEOUT;
        invalidate_pixels(camera);
    }
    if(c->managed && c->confirmed && now-c->peer_tick>=VISION_CAMERA_PEER_TIMEOUT_MS) { c->confirmed=0;invalidate_pixels(camera); }
    if(c->tx_busy && now-c->tx_tick>=50U)abort_tx=1;
    unlock(key);
    if(abort_tx) {
        (void)HAL_UART_AbortTransmit(p->uart);
        key=lock();c->tx_busy=0;
        if((int32_t)(c->next_send-(now+55U))<0)c->next_send=now+55U;
        unlock(key);
    }
    key=lock();
    if(c->stop_requested && !c->tx_busy) {
        uint8_t packet[8]={0x6BU,0x80U,0U,0U,0U,0U,0U,0U};
        c->seq=c->next_seq;c->next_seq=(uint8_t)((c->next_seq+1U)&15U);
        packet[2]=(uint8_t)(c->seq<<3);packet[7]=crc8(packet,7U);
        for(i=0;i<8U;i++)c->request[i]=packet[i];
        c->stop_requested=0;c->pending=1;c->attempts=0;c->next_send=now+55U;
    }
    if(c->pending && !c->tx_busy && (int32_t)(now-c->next_send)>=0) {
        if(c->attempts>=VISION_CAMERA_MAX_ATTEMPTS) {
            c->pending=0;c->confirmed=0;c->result=VISION_RESULT_TIMEOUT;invalidate_pixels(camera);
        } else if(!p->restart && p->status==HAL_OK) {
            for(i=0;i<8U;i++)c->tx[i]=c->request[i];
            c->tx_busy=1;c->tx_tick=now;c->attempts++;c->next_send=now+VISION_CAMERA_ACK_WAIT_MS;send=1;
        }
    }
    unlock(key);
    if(send && HAL_UART_Transmit_IT(p->uart,(uint8_t *)c->tx,8)!=HAL_OK) {
        key=lock();c->tx_busy=0;unlock(key);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    ConsoleDma_TxComplete(uart);
    if(uart==&huart3){ScreenHmi_TxComplete(uart);return;}
    if(uart==&huart1)controllers[0].tx_busy=0;
    else if(uart==&huart2)controllers[1].tx_busy=0;
    else return;
    notify();
}
static float signed_pixel(const uint8_t *data) {
    uint32_t value=(uint32_t)data[0]|((uint32_t)data[1]<<8);
    int32_t signed_value=value>=32768U?(int32_t)value-65536:(int32_t)value;
    return (float)signed_value/10.0f;
}
/* Caller holds the RX publication lock. Both orders are accepted; no motion
 * consumes an incomplete or cross-request image group. */
static void ring_pair_part(uint8_t camera,const vision_pixel_t *pixel,
                            const vision_ring_count_t *count,const vision_ring_axis_t *axis,uint32_t generation,uint32_t now) {
    uint8_t i=(uint8_t)(camera-1U),seq=pixel?pixel->seq:(count?count->seq:axis->seq);
    ring_pair_t *s=&ring_pairs[i];volatile camera_control_t *c=&controllers[i];
    if(!c->managed || !control_allows(camera,VISION_TYPE_RING_PX,2U,now))return;
    if(count && (count->station!=c->station || count->request_seq!=c->seq))return;
    if(pixel && pixel->id!=2U)return;
    if(axis && !(c->flags&VISION_RING_AXIS_FLAG))return;
    if(axis && !axis->valid)ring_axes[i].valid=0U;
    /* A newly reported loss/two->three transition stops use of the old scene
     * immediately, but stable pairs do not cause sub-packet motor glitches. */
    if(ring_scene_seen[i] && ((count && (!count->valid || count->count!=ring_scene_counts[i].count)) ||
        (pixel && !pixel->valid && ring_scene_counts[i].count==3U))) {
        ring_scene_counts[i].valid=0U;ring_scene_counts[i].green_visible=0U;ring_scene_pixels[i].valid=0U;
    }
    if(s->parts && (now-s->tick>VISION_SCENE_PAIR_WINDOW_MS || s->seq!=seq ||
        s->generation!=generation || s->request_seq!=c->seq || s->station!=c->station))s->parts=0U;
    if(c->flags&VISION_RING_AXIS_FLAG) {
        if(ring_scene_seen[i] && ring_scene_counts[i].seq==seq && ring_scene_counts[i].request_seq==c->seq &&
           now-ring_scene_ticks[i]<=VISION_SCENE_PAIR_WINDOW_MS)return;
        if(pixel){s->parts=0U;if(!pixel->valid)ring_axes[i].valid=0U;}
        else if((axis && s->parts!=1U) || (count && s->parts!=5U)) {
            s->parts=0U;ring_axes[i].valid=0U;ring_scene_counts[i].valid=0U;ring_scene_counts[i].green_visible=0U;return;
        }
    }
    if(!s->parts){s->tick=now;s->seq=seq;s->generation=generation;s->request_seq=c->seq;s->station=c->station;}
    if(pixel){s->pixel=*pixel;s->parts|=1U;}else if(count){s->count=*count;s->parts|=2U;}
    else{s->axis=*axis;s->parts|=4U;}
    if(s->parts!=((c->flags&VISION_RING_AXIS_FLAG)?7U:3U))return;
    if(ring_scene_seen[i] && ring_scene_counts[i].seq==seq &&
        ring_scene_counts[i].request_seq==c->seq &&
        (!(c->flags&VISION_RING_AXIS_FLAG) || now-ring_scene_ticks[i]<=VISION_SCENE_PAIR_WINDOW_MS)) {
        s->parts=0U;return;
    }
    ring_scene_pixels[i]=s->pixel;ring_scene_counts[i]=s->count;
    if(++ring_scene_ids[i]==0U)ring_scene_ids[i]++;
    if(++pixel_observation_id[i]==0U)pixel_observation_id[i]++;
    ring_scene_counts[i].observation_id=ring_scene_ids[i];
    ring_axes[i]=s->axis;ring_axes[i].observation_id=ring_scene_ids[i];
    ring_axes[i].station=c->station;ring_axes[i].request_seq=c->seq;
    if(!(c->flags&VISION_RING_AXIS_FLAG))ring_axes[i].valid=0U;
    ring_scene_pixels[i].observation_id=pixel_observation_id[i];
    ring_scene_ticks[i]=s->tick;ring_scene_seen[i]=1U;s->parts=0U;
}
static void plate_pair_part(uint8_t camera,const vision_pixel_t *pixel,
                            const vision_plate_scene_t *scene,uint32_t generation,uint32_t now)
{
    uint8_t i=(uint8_t)(camera-1U),seq=pixel?pixel->seq:scene->seq;
    plate_pair_t *s=&plate_pairs[i];volatile camera_control_t *c=&controllers[i];
    if(!c->managed || !control_allows(camera,VISION_TYPE_PLATE_PX,0U,now))return;
    if(scene && (scene->station!=c->station || scene->request_seq!=c->seq))return;
    if(plate_scene_seen[i] && ((scene && (!scene->valid || scene->status!=plate_scenes[i].status)) ||
        (pixel && ((pixel->valid && plate_scenes[i].status!=VISION_PLATE_FOUND) ||
                   (!pixel->valid && plate_scenes[i].status==VISION_PLATE_FOUND))))) {
        plate_scenes[i].valid=0U;plate_scene_pixels[i].valid=0U;
    }
    if(s->parts && (now-s->tick>VISION_SCENE_PAIR_WINDOW_MS || s->seq!=seq ||
        s->generation!=generation || s->request_seq!=c->seq || s->station!=c->station))s->parts=0U;
    if(!s->parts){s->tick=now;s->seq=seq;s->generation=generation;s->request_seq=c->seq;s->station=c->station;}
    if(pixel){s->pixel=*pixel;s->parts|=1U;}else{s->scene=*scene;s->parts|=2U;}
    if(s->parts!=3U)return;
    if(plate_scene_seen[i] && plate_scenes[i].seq==seq && plate_scenes[i].request_seq==c->seq) {
        s->parts=0U;return;
    }
    plate_scene_pixels[i]=s->pixel;plate_scenes[i]=s->scene;
    if(++plate_scene_ids[i]==0U)plate_scene_ids[i]++;
    if(++pixel_observation_id[i]==0U)pixel_observation_id[i]++;
    plate_scenes[i].observation_id=plate_scene_ids[i];
    plate_scene_pixels[i].observation_id=pixel_observation_id[i];
    plate_scene_ticks[i]=s->tick;plate_scene_seen[i]=1U;s->parts=0U;
}
uint8_t Vision_GetQrTask(vision_qr_task_t *out,uint32_t *age)
{
    uint32_t key,now;uint8_t valid;
    if(!out)return 0U;
    key=lock();now=HAL_GetTick();
    valid=(uint8_t)(qr_task_seen && controllers[1].managed && controllers[1].station==4U &&
        controllers[1].seq==qr_task.request_seq && control_allows(2U,VISION_TYPE_QR_GROUP,0U,now) &&
        !cam2.restart && cam2.status==HAL_OK && cam2.dropped==seen_drops[1]);
    if(valid){*out=qr_task;if(age)*age=now-qr_task_tick;}
    unlock(key);return valid;
}
static uint8_t parse_binary(uint8_t camera,const uint8_t *data,uint32_t generation,uint32_t first_rx_tick) {
    int index=pixel_index(data[1]);uint32_t key,now=HAL_GetTick();uint8_t i;
    vision_pixel_t value={0};vision_pixel_t *previous;
    if(data[1]==0x81U || data[1]==0x82U)return parse_control(camera,data,generation,now);
    if(data[1]==0x86U) {
        uint16_t gap=(uint16_t)((uint16_t)data[3]|((uint16_t)data[4]<<8));
        int16_t right=(int16_t)((uint16_t)data[5]|((uint16_t)data[6]<<8));
        if(camera!=2U || data[2]>20U || gap<50U || gap>VOBS_OBSERVE_MAX_MM || right< -190 || right>190)return 1U;
        key=lock();
        if(port(2U)->generation==generation && port(2U)->protocol==VISION_PROTOCOL_BINARY8 &&
           (int32_t)(first_rx_tick-pc_obstacle_clear_tick)>0) {
            if(++pc_obstacle_id==0U)pc_obstacle_id++;
            pc_obstacle.observation_id=pc_obstacle_id;pc_obstacle.rx_tick=first_rx_tick;
            pc_obstacle.front_gap_mm=gap;pc_obstacle.body_right_mm=right;
            pc_obstacle.bridge_age_ms=(uint16_t)data[2]*10U;pc_obstacle_seen=1U;
        }
        unlock(key);return 1U;
    }
    if(data[1]==VISION_TYPE_QR_GROUP) {
        uint8_t result;
        key=lock();
        if(camera!=2U || port(camera)->generation!=generation ||
            port(camera)->protocol!=VISION_PROTOCOL_BINARY8 || !controllers[1].managed ||
            controllers[1].station!=4U || !control_allows(2U,VISION_TYPE_QR_GROUP,0U,now)) {
            unlock(key);return 1U;
        }
        result=VisionQr_Feed(&qr_assembly,data,controllers[1].seq,now);
        if(result==1U) {
            qr_task=qr_assembly.task;if(++qr_observation_id==0U)qr_observation_id++;
            qr_task.observation_id=qr_observation_id;qr_task_tick=now;qr_task_seen=1U;
            memcpy(qr.digits,qr_task.raw,sizeof(qr.digits));qr.seq=qr_task.request_seq;
            qr_tick=now;qr_valid=1U;qr_source_camera=2U;
        }
        unlock(key);return (uint8_t)(result!=255U);
    }
    if(data[1]==VISION_TYPE_PLATE_SCENE) {
        vision_plate_scene_t scene={(data[2]>>3)&15U,data[2]&7U,(data[2]>>7)&1U,data[3],data[4],data[5],0U};
        if(scene.id!=0U || scene.status>3U || (scene.valid?scene.status==0U:scene.status!=0U) ||
            scene.station>6U || scene.request_seq>15U || data[6])return 0U;
        key=lock();
        if(port(camera)->generation==generation && port(camera)->protocol==VISION_PROTOCOL_BINARY8)
            plate_pair_part(camera,NULL,&scene,generation,now);
        unlock(key);return 1U;
    }
    if(data[1]==VISION_TYPE_RING_COUNT) {
        vision_ring_count_t count={0};count.seq=(data[2]>>3)&15U;count.id=data[2]&7U;
        count.valid=(data[2]>>7)&1U;count.count=data[3];count.station=data[4];count.request_seq=data[5];
        count.green_visible=data[6]&1U;
        if(count.id!=2U || count.count>3U || count.station>6U || count.request_seq>15U || (data[6]&~1U))return 0U;
        key=lock();
        if(port(camera)->generation==generation && port(camera)->protocol==VISION_PROTOCOL_BINARY8)
            ring_pair_part(camera,NULL,&count,NULL,generation,now);
        unlock(key);return 1U;
    }
    if(data[1]==VISION_TYPE_RING_AXIS) {
        vision_ring_axis_t axis={0};int16_t raw=(int16_t)((uint16_t)data[3]|((uint16_t)data[4]<<8U));
        axis.id=data[2]&7U;axis.seq=(data[2]>>3U)&15U;axis.valid=data[2]>>7U;
        axis.axis_deg=(float)raw*0.01f;axis.confidence=data[5];axis.landmarks=data[6];
        if(axis.id!=2U || (axis.valid?(raw< -9000 || raw>=9000 || data[5]>100U || data[6]!=3U):
           (data[3] || data[4] || data[5] || data[6])))return 0U;
        key=lock();if(port(camera)->generation==generation && port(camera)->protocol==VISION_PROTOCOL_BINARY8)
            ring_pair_part(camera,NULL,NULL,&axis,generation,now);
        unlock(key);return 1U;
    }
    if(index<0)return 0;
    value.type=data[1];value.id=data[2]&7U;value.seq=(data[2]>>3)&15U;value.valid=(data[2]>>7)&1U;
    if(value.type==VISION_TYPE_PLATE_PX) { if(value.id!=0U)return 0; }
    else if(value.id>6U || (value.valid && value.id==0U))return 0;
    if(!value.valid && (data[3] || data[4] || data[5] || data[6]))return 0;
    value.dx_px=signed_pixel(&data[3]);value.dy_px=signed_pixel(&data[5]);
    key=lock();
    /* A concurrent explicit protocol switch must not publish an old context. */
    if(port(camera)->generation!=generation || port(camera)->protocol!=VISION_PROTOCOL_BINARY8) { unlock(key);return 1; }
    if(!control_allows(camera,value.type,value.id,now)) { unlock(key);return 1; }
    if(value.type==VISION_TYPE_RING_PX)ring_pair_part(camera,&value,NULL,NULL,generation,now);
    if(value.type==VISION_TYPE_PLATE_PX)plate_pair_part(camera,&value,NULL,generation,now);
    previous=&pixels[camera-1U][index];
    if(value.valid && pixel_seen[camera-1U][index] && previous->valid && previous->id==value.id
            && previous->seq==value.seq && now-pixel_ticks[camera-1U][index]<=50U) {
        uint8_t same=(uint8_t)(previous->dx_px==value.dx_px && previous->dy_px==value.dy_px);
        unlock(key);return same; /* duplicates never refresh age */
    }
    if(pixel_context_type[camera-1U]!=value.type || pixel_context_id[camera-1U]!=value.id) {
        for(i=0;i<3U;i++) { pixels[camera-1U][i].valid=0;pixels[camera-1U][i].dx_px=0;pixels[camera-1U][i].dy_px=0; }
    }
    value.observation_id=++pixel_observation_id[camera-1U];
    if(!value.observation_id)value.observation_id=++pixel_observation_id[camera-1U];
    pixels[camera-1U][index]=value;pixel_ticks[camera-1U][index]=now;pixel_seen[camera-1U][index]=1;
    pixel_context_type[camera-1U]=value.type;pixel_context_id[camera-1U]=value.id;
    unlock(key);return 1;
}
static void expire_binary(binary_parser_t *p,uint32_t now) {
    if(p->used && now-p->tick>=50U) { p->used=0;parse_errors++; }
}
static void feed_binary(binary_parser_t *p,uint8_t camera,uint8_t b,uint32_t generation,uint32_t rx_tick) {
    uint32_t now=HAL_GetTick();
    expire_binary(p,now);
    if(!p->used) { if(b!=0x6BU)return;p->tick=now; }
    p->rx_tick[p->used]=rx_tick;p->data[p->used++]=b;
    if(p->used<8U)return;
    if(crc8(p->data,7)==p->data[7] && parse_binary(camera,p->data,generation,p->rx_tick[0])) { p->used=0;return; }
    parse_errors++;
    if(controllers[camera-1U].flags&VISION_RING_AXIS_FLAG) {
        uint32_t key=lock();ring_axes[camera-1U].valid=0U;ring_pairs[camera-1U].parts=0U;unlock(key);
    }
    /* Only drop the first byte. Embedded 6B/CR/LF/@ are ordinary payload. */
    memmove(p->data,p->data+1,7);p->used=7;
    memmove(p->rx_tick,p->rx_tick+1,7U*sizeof(uint32_t));
    while(p->used && p->data[0]!=0x6BU) { p->used--;memmove(p->data,p->data+1,p->used);
        memmove(p->rx_tick,p->rx_tick+1,p->used*sizeof(uint32_t)); }
    p->tick=now;
}
static void notify(void) { if(vision_thread!=NULL)(void)osThreadFlagsSet(vision_thread,1U); }
static uint8_t hex(uint8_t b) {
    if(b>='0' && b<='9')return (uint8_t)(b-'0');
    if(b>='A' && b<='F')return (uint8_t)(b-'A'+10U);
    if(b>='a' && b<='f')return (uint8_t)(b-'a'+10U);
    return 255U;
}
static uint8_t uint_value(const uint8_t *s,uint16_t n,uint16_t *out) {
    uint32_t v=0;uint16_t i;if(!n)return 0;
    for(i=0;i<n;i++) { if(s[i]<'0' || s[i]>'9')return 0;v=v*10U+s[i]-'0';if(v>65535U)return 0; }
    *out=(uint16_t)v;return 1;
}
static uint8_t float_value(const uint8_t *s,uint16_t n,float *out) {
    uint16_t i=0;uint8_t fractional=0,digits=0;float v=0,scale=0.1f,sign=1;
    if(!n)return 0;
    if(s[0]=='-' || s[0]=='+') { sign=s[0]=='-'?-1.0f:1.0f;i++; }
    for(;i<n;i++) {
        if(s[i]=='.' && !fractional) { fractional=1;continue; }
        if(s[i]<'0' || s[i]>'9')return 0;
        digits=1;
        if(fractional) { v+=(s[i]-'0')*scale;scale*=0.1f; }else v=v*10+(s[i]-'0');
    }
    if(!digits || !isfinite(v))return 0;
    *out=v*sign;return 1;
}
static uint8_t word(const uint8_t *s,uint16_t n,const char *text) {
    return (uint8_t)(strlen(text)==n && memcmp(s,text,n)==0);
}
/* Parse to local values and publish once, so readers cannot see half a frame.
 * Fields end at their own comma; numbers must not consume the remaining frame. */
static void parse(parser_t *p,uint8_t camera,uint32_t generation) {
    uint16_t star=0,i,end,type_end,at,mask=0,seq=0;uint8_t sum=0,type=0;
    vision_qr_t q={0};vision_ring_t r={0};vision_color_t c={0};vision_obs_t o={0};
    while(star<p->used && p->data[star]!='*')star++;
    if(star<1U || star+3U!=p->used || hex(p->data[star+1])==255U || hex(p->data[star+2])==255U)goto bad;
    for(i=0;i<star;i++)sum^=p->data[i];
    if(sum!=(uint8_t)(hex(p->data[star+1])*16U+hex(p->data[star+2])))goto bad;
    type_end=0;while(type_end<star && p->data[type_end]!=',')type_end++;
    if(word(p->data,type_end,"QR"))type=1;
    else if(word(p->data,type_end,"RING"))type=2;
    else if(word(p->data,type_end,"COLOR"))type=3;
    else if(word(p->data,type_end,"OBS"))type=4;
    else goto bad;
    at=(uint16_t)(type_end+1U);
    while(at<star) {
        uint16_t eq,n,u=0;const uint8_t *v;float f=0;
        end=at;while(end<star && p->data[end]!=',')end++;
        eq=at;while(eq<end && p->data[eq]!='=')eq++;
        if(type==1 && eq==end) {
            n=(uint16_t)(end-at);if(!n || n>=sizeof(q.digits) || (mask&2U))goto bad;
            for(i=at;i<end;i++)if((p->data[i]<'0' || p->data[i]>'9') && p->data[i]!='+')goto bad;
            memcpy(q.digits,&p->data[at],n);q.digits[n]='\0';mask|=2U;
        } else if(eq<end) {
            v=&p->data[eq+1];n=(uint16_t)(end-eq-1U);
            if(word(&p->data[at],(uint16_t)(eq-at),"seq")) {
                if((mask&1U) || !uint_value(v,n,&seq))goto bad;
                mask|=1U;
            } else if(type==2) {
                if(word(&p->data[at],eq-at,"id")) { if(!uint_value(v,n,&u) || u>255U)goto bad;r.id=(uint8_t)u;mask|=2U; }
                else if(word(&p->data[at],eq-at,"x")) { if(!float_value(v,n,&f))goto bad;r.x_mm=f;mask|=4U; }
                else if(word(&p->data[at],eq-at,"y")) { if(!float_value(v,n,&f))goto bad;r.y_mm=f;mask|=8U; }
                else if(word(&p->data[at],eq-at,"theta")) { if(!float_value(v,n,&f))goto bad;r.theta_rad=f;mask|=16U; }
                else if(word(&p->data[at],eq-at,"conf")) { if(!float_value(v,n,&f) || f<0 || f>1)goto bad;r.conf=f;mask|=32U; }
            } else if(type==3) {
                if(word(&p->data[at],eq-at,"id")) { if(!uint_value(v,n,&u) || u>255U)goto bad;c.id=(uint8_t)u;mask|=2U; }
                else if(word(&p->data[at],eq-at,"color")) { if(!uint_value(v,n,&u) || u>255U)goto bad;c.color=(uint8_t)u;mask|=4U; }
                else if(word(&p->data[at],eq-at,"conf")) { if(!float_value(v,n,&f) || f<0 || f>1)goto bad;c.conf=f;mask|=8U; }
            } else if(type==4) {
                if(word(&p->data[at],eq-at,"front")) { if(!uint_value(v,n,&u))goto bad;o.front_mm=u;mask|=2U; }
                else if(word(&p->data[at],eq-at,"left")) { if(!uint_value(v,n,&u))goto bad;o.left_mm=u;mask|=4U; }
                else if(word(&p->data[at],eq-at,"right")) { if(!uint_value(v,n,&u))goto bad;o.right_mm=u;mask|=8U; }
                else if(word(&p->data[at],eq-at,"valid")) { if(!uint_value(v,n,&u) || u>1U)goto bad;o.valid=(uint8_t)u;mask|=16U; }
            }
        } else goto bad;
        at=(uint16_t)(end+1U);
    }
    if((type==1 && mask!=3U) || (type==2 && mask!=63U) || (type==3 && mask!=15U) || (type==4 && mask!=31U))goto bad;
    {
        uint32_t key=lock(),now=HAL_GetTick();
        if(port(camera)->generation!=generation || port(camera)->protocol!=VISION_PROTOCOL_ASCII) { unlock(key);return; }
        if(type==1) { q.seq=seq;qr=q;qr_tick=now;qr_valid=1;qr_source_camera=camera; }
        else if(type==2) { r.seq=seq;ring=r;ring_tick=now;ring_valid=1; }
        else if(type==3) { c.seq=seq;color=c;color_tick=now;color_valid=1; }
        else { o.seq=seq;obs=o;obs_tick=now;obs_valid=1; }
        unlock(key);
    }
    return;
bad:parse_errors++;
}
static void feed(parser_t *p,uint8_t b,uint8_t camera,uint32_t generation) {
    if(b=='@') { if(p->active && p->used)parse_errors++;p->used=0;p->active=1;return; }
    if(!p->active)return;
    if(b=='\r' || b=='\n') { if(p->used)parse(p,camera,generation);p->active=0;return; }
    if(p->used>=FRAME_SIZE) { p->active=0;parse_errors++;return; }
    p->data[p->used++]=b;
}
static void capture(uint8_t camera,const uint8_t *data,uint8_t n) {
    uint32_t key=lock(),head=raw_head;uint8_t i;
    if(!(raw_mode&(1U<<(camera-1U)))) { unlock(key);return; }
    if(head-raw_tail>=RAW_SLOTS) { raw_dropped+=n;unlock(key);return; }
    raw_queue[head%RAW_SLOTS].camera=camera;raw_queue[head%RAW_SLOTS].length=n;
    for(i=0;i<n;i++)raw_queue[head%RAW_SLOTS].data[i]=data[i];
    __DMB();raw_head=head+1U;unlock(key);
}
/* IRQ path: copy bytes out of DMA before they are overwritten. No parser,
 * UART TX, allocation or waits. Loss is counted and resynchronised in task. */
static void pump(volatile camera_port_t *p,uint16_t pos) {
    uint16_t old=p->last;uint32_t head=p->head,arrival=HAL_GetTick();uint16_t received=0;
    if(pos>=DMA_SIZE)pos=0;
    while(old!=pos) {
        uint8_t b=p->dma[old];old=(uint16_t)((old+1U)%DMA_SIZE);p->bytes++;received++;
        if(head-p->tail<RX_SIZE) { p->ring[head&(RX_SIZE-1U)]=b;
            if(p==&cam2)cam2_rx_tick[head&(RX_SIZE-1U)]=arrival;
            head++;
        }
        else p->dropped++;
    }
    p->last=pos;
    if(received) { __DMB();p->head=head;p->last_tick=arrival;notify(); }
}
static void pump_hwt(uint16_t pos) {
    uint16_t old=hwt_last;if(pos>=sizeof(rx10))pos=0;if(pos==old)return;
    if(pos>old)HWT101_OnDmaData((const uint8_t *)&rx10[old],(uint16_t)(pos-old));
    else { HWT101_OnDmaData((const uint8_t *)&rx10[old],(uint16_t)(sizeof(rx10)-old));HWT101_OnDmaData((const uint8_t *)rx10,pos); }
    hwt_last=pos;
}
static void dispatch(UART_HandleTypeDef *uart) {
    volatile camera_port_t *p=find(uart);
    if(p!=NULL && uart->hdmarx!=NULL && !p->restart)
        pump(p,(uint16_t)(DMA_SIZE-__HAL_DMA_GET_COUNTER(uart->hdmarx)));
    else if(uart==&huart10 && uart->hdmarx!=NULL)
        pump_hwt((uint16_t)(sizeof(rx10)-__HAL_DMA_GET_COUNTER(uart->hdmarx)));
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) { if(uart==&huart3){ScreenHmi_RxComplete(uart);return;}dispatch(uart); }
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart) { dispatch(uart); }
void Vision_CheckIdle(UART_HandleTypeDef *uart) {
    if(find(uart)==NULL && uart!=&huart10)return;
    if(__HAL_UART_GET_FLAG(uart,UART_FLAG_IDLE)) { __HAL_UART_CLEAR_IDLEFLAG(uart);dispatch(uart); }
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *uart){ConsoleDma_AbortComplete(uart);}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if(uart==&huart4){if(uart->ErrorCode&HAL_UART_ERROR_DMA)ConsoleDma_TxError(uart);return;}
    if(uart==&huart3){ScreenHmi_UartError(uart);return;}
    volatile camera_port_t *p=find(uart);
    if(p!=NULL) { p->errors++;p->restart=1;notify(); }
    else if(uart==&huart10) { /* Preserve the pre-existing IMU recovery path. */
        hwt_last=0;(void)HAL_UART_Receive_DMA(uart,(uint8_t *)rx10,sizeof(rx10));
        __HAL_UART_CLEAR_IDLEFLAG(uart);SET_BIT(uart->Instance->CR1,USART_CR1_IDLEIE);
    }
}
static HAL_StatusTypeDef start(volatile camera_port_t *p) {
    HAL_StatusTypeDef s;
    p->last=0;s=HAL_UART_Receive_DMA(p->uart,(uint8_t *)p->dma,DMA_SIZE);p->status=(uint8_t)s;
    if(s==HAL_OK) { __HAL_UART_CLEAR_IDLEFLAG(p->uart);SET_BIT(p->uart->Instance->CR1,USART_CR1_IDLEIE); }
    return s;
}
HAL_StatusTypeDef Vision_Init(void) {
    HAL_StatusTypeDef a,b,h;
    if(initialized)return initial_status;
    a=start(&cam1);b=start(&cam2); /* A failed camera must not skip IMU startup. */
    hwt_last=0;h=HAL_UART_Receive_DMA(&huart10,(uint8_t *)rx10,sizeof(rx10));
    if(h==HAL_OK) { __HAL_UART_CLEAR_IDLEFLAG(&huart10);SET_BIT(huart10.Instance->CR1,USART_CR1_IDLEIE); }
    initial_status=(h!=HAL_OK)?h:(a!=HAL_OK?a:b);initialized=1;
    return initial_status;
}
void Vision_Service(void) {
    uint8_t camera;
    for(camera=1;camera<=2;camera++) {
        volatile camera_port_t *p=port(camera);uint16_t budget=256;uint8_t bytes[RAW_SIZE],n=0;
        uint32_t now=HAL_GetTick();parser_t *parser=&parsers[camera-1U];
        binary_parser_t *binary=&binary_parsers[camera-1U];uint32_t generation=p->generation;
        if(seen_generation[camera-1U]!=generation) {
            parser->active=0;binary->used=0;seen_generation[camera-1U]=generation;
        }
        expire_binary(binary,now);
        if(p->restart || p->status!=HAL_OK) {
            uint32_t error_generation;
            if((int32_t)(now-retry_tick[camera-1U])<0)continue;
            p->restart=1; /* suppress DMA cursor consumption during restart */
            (void)HAL_UART_AbortReceive(p->uart);
            {
                uint32_t key=lock();p->tail=p->head;p->last=0;parser->active=0;binary->used=0;
                invalidate_pixels(camera);
                controllers[camera-1U].confirmed=0;controllers[camera-1U].previous_confirmed=0;
                error_generation=p->errors;
                seen_drops[camera-1U]=p->dropped;unlock(key);
            }
            if(start(p)==HAL_OK) {
                uint32_t key=lock();
                if(p->errors==error_generation) { p->restart=0;p->recoveries++; }
                unlock(key);
            }
            retry_tick[camera-1U]=now+100U;
            continue;
        }
        while(budget-- && p->tail!=p->head) {
            uint8_t b;uint32_t tail,rx_tick;
            if(seen_drops[camera-1U]!=p->dropped || p->restart) {
                uint32_t key=lock();p->tail=p->head;seen_drops[camera-1U]=p->dropped;
                parser->active=0;binary->used=0;invalidate_pixels(camera);parse_errors++;unlock(key);break;
            }
            {
                uint32_t key=lock();
                /* A task protocol switch also writes tail. Do not resurrect
                 * its flushed queue by storing a tail sampled before it. */
                if(p->generation!=generation || p->tail==p->head) { unlock(key);break; }
                tail=p->tail;__DMB();b=p->ring[tail&(RX_SIZE-1U)];
                rx_tick=camera==2U?cam2_rx_tick[tail&(RX_SIZE-1U)]:HAL_GetTick();p->tail=tail+1U;
                unlock(key);
            }
            if(p->generation!=generation)break;
            if(p->protocol==VISION_PROTOCOL_BINARY8)feed_binary(binary,camera,b,generation,rx_tick);
            else if(p->protocol==VISION_PROTOCOL_ASCII)feed(parser,b,camera,generation);
            bytes[n++]=b;
            if(n==RAW_SIZE) { capture(camera,bytes,n);n=0; }
        }
        if(n)capture(camera,bytes,n);
        control_service(camera,HAL_GetTick());
    }
}
void Vision_TaskRun(void *argument) {
    (void)argument;vision_thread=osThreadGetId();
    for(;;) { Vision_Service();(void)osThreadFlagsWait(1U,osFlagsWaitAny,10U); }
}
uint8_t Vision_GetPortStats(uint8_t camera,vision_port_stats_t *out) {
    volatile camera_port_t *p=port(camera);uint32_t key;if(p==NULL || out==NULL)return 0;
    key=lock();out->rx_bytes=p->bytes;out->dropped_bytes=p->dropped;out->uart_errors=p->errors;
    out->recoveries=p->recoveries;out->last_rx_tick=p->last_tick;out->queued_bytes=(uint16_t)(p->head-p->tail);
    out->init_status=p->status;out->protocol=p->protocol;unlock(key);return 1;
}
uint8_t Vision_GetQR(char out[16],uint16_t *seq,uint32_t *age) {
    uint32_t key;if(out==NULL)return 0;key=lock();if(!qr_valid) { unlock(key);return 0; }
    memcpy(out,qr.digits,16);if(seq)*seq=qr.seq;if(age)*age=HAL_GetTick()-qr_tick;unlock(key);return 1;
}
uint8_t Vision_GetRing(vision_ring_t *out,uint32_t *age) {
    uint32_t key;if(out==NULL)return 0;key=lock();if(!ring_valid) { unlock(key);return 0; }
    *out=ring;if(age)*age=HAL_GetTick()-ring_tick;unlock(key);return 1;
}
uint8_t Vision_GetColor(vision_color_t *out,uint32_t *age) {
    uint32_t key;if(out==NULL)return 0;key=lock();if(!color_valid) { unlock(key);return 0; }
    *out=color;if(age)*age=HAL_GetTick()-color_tick;unlock(key);return 1;
}
uint8_t Vision_GetObs(vision_obs_t *out,uint32_t *age) {
    uint32_t key;if(out==NULL)return 0;key=lock();if(!obs_valid) { unlock(key);return 0; }
    *out=obs;if(age)*age=HAL_GetTick()-obs_tick;unlock(key);return 1;
}
uint32_t Vision_GetParseErrorCount(void) { uint32_t key=lock(),n=parse_errors;unlock(key);return n; }
uint8_t Vision_ProcessCommand(const char *cmd) {
    char line[192];uint8_t camera,mode;uint32_t key;
    while(*cmd==' ' || *cmd=='\t' || *cmd=='&')cmd++;
    if(strcmp(cmd,"camstat")==0) {
        (void)snprintf(line,sizeof(line),"[CAM] task=%u raw=%u parse_errors=%lu raw_dropped=%lu bytes\r\n",
            vision_thread!=NULL,raw_mode,(unsigned long)Vision_GetParseErrorCount(),(unsigned long)raw_dropped);
        ZDT_Console_Print(line);
        for(camera=1;camera<=2;camera++) {
            vision_port_stats_t s;(void)Vision_GetPortStats(camera,&s);
            (void)snprintf(line,sizeof(line),"[CAM%u] USART%u 115200 RX=%lu queued=%u dropped=%lu errors=%lu recover=%lu init=%u\r\n",
                camera,camera,(unsigned long)s.rx_bytes,s.queued_bytes,(unsigned long)s.dropped_bytes,
                (unsigned long)s.uart_errors,(unsigned long)s.recoveries,s.init_status);ZDT_Console_Print(line);
            {
                vision_camera_state_t state={0};(void)Vision_GetCameraState(camera,&state);
                (void)snprintf(line,sizeof(line),"[CAM%u] mode=%u station=%u target=%u pending=%u confirmed=%u result=%u seq=%u attempts=%u HBseen=%u ver=%u HBage=%lums reset=%u\r\n",
                    camera,state.mode,state.station,state.target,state.pending,state.confirmed,state.result,
                    state.request_seq,state.attempts,state.heartbeat_seen,state.protocol_version,
                    (unsigned long)state.heartbeat_age_ms,state.reset_pending);
                ZDT_Console_Print(line);
            }
        }
        return 1;
    }
    if(strncmp(cmd,"camraw",6)!=0 || (cmd[6] && cmd[6]!=' ' && cmd[6]!='\t'))return 0;
    cmd+=6;while(*cmd==' ' || *cmd=='\t')cmd++;
    if(!*cmd || strcmp(cmd,"1")==0)mode=1;
    else if(strcmp(cmd,"2")==0)mode=2;
    else if(strcmp(cmd,"all")==0 || strcmp(cmd,"3")==0)mode=3;
    else if(strcmp(cmd,"off")==0 || strcmp(cmd,"0")==0)mode=0;
    else { ZDT_Console_Print("[CAM] usage: camraw 1|2|all|off; camstat\r\n");return 1; }
    key=lock();raw_mode=mode;raw_tail=raw_head;unlock(key);raw_print_tick=HAL_GetTick();
    (void)snprintf(line,sizeof(line),"[CAM] raw=%u; output while chassis idle; binary bytes escaped.\r\n",mode);
    ZDT_Console_Print(line);return 1;
}
void Vision_DebugPoll(uint8_t motion_busy) {
    raw_record_t record;char line[RAW_SIZE*4U+32U];uint32_t key;uint16_t at;uint8_t i;
    static const char digits[]="0123456789ABCDEF";
    if(motion_busy || !raw_mode || raw_tail==raw_head || (int32_t)(HAL_GetTick()-raw_print_tick)<0)return;
    key=lock();record.camera=raw_queue[raw_tail%RAW_SLOTS].camera;record.length=raw_queue[raw_tail%RAW_SLOTS].length;
    for(i=0;i<record.length;i++)record.data[i]=raw_queue[raw_tail%RAW_SLOTS].data[i];
    raw_tail++;unlock(key);
    at=(uint16_t)snprintf(line,sizeof(line),"[CAM%u RX] ",record.camera);
    for(i=0;i<record.length;i++) {
        uint8_t b=record.data[i];
        if(b>=32U && b<=126U && b!='\\')line[at++]=(char)b;
        else if(b=='\r' || b=='\n' || b=='\t' || b=='\\') {
            line[at++]='\\';line[at++]=b=='\r'?'r':(b=='\n'?'n':(b=='\t'?'t':'\\'));
        } else { line[at++]='\\';line[at++]='x';line[at++]=digits[b>>4];line[at++]=digits[b&15U]; }
    }
    line[at++]='\r';line[at++]='\n';line[at]='\0';ZDT_Console_Print(line);raw_print_tick=HAL_GetTick()+20U;
}
