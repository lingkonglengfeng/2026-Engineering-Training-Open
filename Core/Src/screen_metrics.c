#include "screen_metrics.h"
#include <stdio.h>
#include <string.h>
static volatile screen_snapshot_t s_screen;
void ScreenMetrics_ClearTask(void){memset((void *)s_screen.task,0,sizeof(s_screen.task));}
uint8_t ScreenMetrics_SetTask(const char *raw)
{
    uint8_t i;
    if(!raw || strlen(raw)!=15U)return 0U;
    for(i=0;i<15;i++)if((i==3U || i==7U || i==11U)?raw[i]!='+':(raw[i]<'1' || raw[i]>'6'))return 0U;
    memcpy((void *)s_screen.task,raw,16U);s_screen.state=SCREEN_SCAN_OK;return 1U;
}
void ScreenMetrics_SetState(screen_state_t state){if((unsigned int)state<=SCREEN_ERROR)s_screen.state=state;}
void ScreenMetrics_ResetCounts(void){s_screen.pick=s_screen.place=0U;}
void ScreenMetrics_Pick(void){if(s_screen.pick<2147483647U)s_screen.pick++;}
void ScreenMetrics_Place(void){if(s_screen.place<2147483647U)s_screen.place++;}
void ScreenMetrics_Snapshot(screen_snapshot_t *out){if(out)memcpy(out,(const void *)&s_screen,sizeof(*out));}
size_t ScreenMetrics_Command(const screen_snapshot_t *s,uint8_t field,uint8_t *out,size_t capacity)
{
    static const char *const states[]={"WAIT QR","SCAN OK","RUNNING","DONE","ERROR"};
    char row[8]={0};int n;
    if(!s || !out || capacity<4U || field>4U || (unsigned int)s->state>SCREEN_ERROR)return 0U;
    if(field<2U) {
        if(s->task[0])memcpy(row,s->task+(field?8U:0U),7U);
        n=snprintf((char *)out,capacity,"tCode%u.txt=\"%s\"",field+1U,row);
    } else if(field<4U)n=snprintf((char *)out,capacity,"%s.val=%lu",field==2U?"nPick":"nPlace",(unsigned long)(field==2U?s->pick:s->place));
    else n=snprintf((char *)out,capacity,"tState.txt=\"%s\"",states[s->state]);
    if(n<0 || (size_t)n+3U>capacity)return 0U;
    out[n]=out[n+1]=out[n+2]=255U;return (size_t)n+3U;
}
