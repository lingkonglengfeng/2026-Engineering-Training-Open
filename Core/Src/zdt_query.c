#include "zdt_query.h"
#include <string.h>
#include <math.h>
static void finish(zdt_query_t *q,const char *reason)
{q->running=0U;if(q->ports.report)q->ports.report(q,reason);}
uint8_t ZDTQuery_Start(zdt_query_t *q,const zdt_query_ports_t *ports,uint8_t address,
                       uint32_t now,uint32_t baseline_seq)
{
    if(!q || !ports || !ports->send || q->running || address<1U || address>6U)return 0U;
    memset(q,0,sizeof(*q));q->ports=*ports;q->address=address;q->started=now;q->baseline_seq=baseline_seq;q->running=1U;
    if(!q->ports.send(address,0x32U) || !q->ports.send(address,0x36U)) {
        finish(q,"query TX rejected");return 0U;
    }
    return 1U;
}
void ZDTQuery_Frame(zdt_query_t *q,uint8_t address,const uint8_t *data,uint8_t length,
                    uint32_t received_tick,uint32_t received_seq)
{
    uint32_t magnitude;int64_t value;uint8_t bit;
    if(!q || !q->running || !data || length!=7U || address!=q->address || data[6]!=0x6BU || data[1]>1U ||
       (data[0]!=0x32U && data[0]!=0x36U) || (int32_t)(received_seq-q->baseline_seq)<=0 ||
       (int32_t)(received_tick-q->started)<0 || received_tick-q->started>400U)return;
    magnitude=((uint32_t)data[2]<<24U)|((uint32_t)data[3]<<16U)|((uint32_t)data[4]<<8U)|data[5];
    value=data[1]?-(int64_t)magnitude:(int64_t)magnitude;
    bit=data[0]==0x36U?ZDT_QUERY_POSITION:ZDT_QUERY_PULSES;
    if(q->mask&bit)return;
    if(bit==ZDT_QUERY_POSITION){q->position_counts=value;q->position_tick=received_tick;}
    else {q->input_pulses=value;q->pulse_tick=received_tick;}
    q->mask|=bit;if(q->mask==3U)finish(q,NULL);
}
void ZDTQuery_Service(zdt_query_t *q,uint32_t now)
{if(q && q->running && now-q->started>=400U)finish(q,"query reply timeout");}
double ZDTQuery_AngleDeg(const zdt_query_t *q)
{return (double)q->position_counts*360.0/65536.0;}
double ZDTQuery_CircleDeg(const zdt_query_t *q)
{double a=fmod(ZDTQuery_AngleDeg(q),360.0);return a<0.0?a+360.0:(a==0.0?0.0:a);}
