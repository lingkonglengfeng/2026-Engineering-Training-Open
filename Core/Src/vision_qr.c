#include "vision_qr.h"
#include <string.h>
void VisionQr_Reset(vision_qr_assembly_t *q){memset(q,0,sizeof(*q));}
uint8_t VisionQr_CompetitionTaskValid(const vision_qr_task_t *task)
{
    uint8_t round,i,masks[2]={0U,0U};
    if(!task)return 0U;
    for(round=0U;round<2U;round++) {
        uint8_t rings=0U;
        for(i=0U;i<3U;i++) {
            uint8_t c=task->colors[round][i],p=task->positions[round][i];
            if(c<1U || c>6U || p<1U || p>3U || (masks[round]&(1U<<c)) || (rings&(1U<<p)))return 0U;
            masks[round]|=(uint8_t)(1U<<c);rings|=(uint8_t)(1U<<p);
        }
    }
    return masks[0]==masks[1];
}
uint8_t VisionQr_Feed(vision_qr_assembly_t *q,const uint8_t p[8],uint8_t request_seq,uint32_t now)
{
    uint8_t id=(uint8_t)(p[2]&7U),i,j,seen=0U,at=0U;
    if(p[1]!=VISION_TYPE_QR_GROUP || id<1U || id>4U || p[6]!=4U)return 255U;
    if(((p[2]>>3U)&15U)!=request_seq)return 0U;
    if(!(p[2]&0x80U)) {
        if(p[3] || p[4] || p[5])return 255U;
        q->mask=0U;return 0U; /* Do not overwrite a previously complete task. */
    }
    for(i=0U;i<3U;i++) {
        uint8_t v=p[3U+i];
        if(v<1U || v>((id==1U || id==3U)?6U:3U) || (seen&(1U<<v)))return 255U;
        seen|=(uint8_t)(1U<<v);
    }
    if(q->complete)return memcmp(q->groups[id-1U],p+3,3U)?255U:0U;
    if(q->mask && now-q->started>VISION_QR_GROUP_WINDOW_MS)q->mask=0U;
    if(!q->mask)q->started=now;
    if(q->mask&(1U<<(id-1U))) {
        if(memcmp(q->groups[id-1U],p+3,3U)){q->mask=0U;return 255U;}
        return 0U;
    }
    memcpy(q->groups[id-1U],p+3,3U);q->mask|=(uint8_t)(1U<<(id-1U));
    if(q->mask!=15U)return 0U;
    q->task.request_seq=request_seq;
    for(i=0U;i<4U;i++) {
        for(j=0U;j<3U;j++) {
            uint8_t wire=q->groups[i][j],rule=wire;
            if(!(i&1U)) {
                q->task.colors[i/2U][j]=wire;
                if(wire==5U)rule=6U;else if(wire==6U)rule=5U;
            } else q->task.positions[i/2U][j]=wire;
            q->task.raw[at++]=(char)('0'+rule);
        }
        if(i<3U)q->task.raw[at++]='+';
    }
    q->task.raw[at]=0;q->complete=1U;return 1U;
}
