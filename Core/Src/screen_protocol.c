#include "screen_protocol.h"
#include <string.h>
static uint8_t number(const char **cursor,uint16_t *value)
{
    uint32_t n=0;uint8_t digits=0;const char *p=*cursor;
    while(*p==' ' || *p=='\t')p++;
    while(*p>='0' && *p<='9') {if(++digits>4U)return 0U;n=n*10U+(uint8_t)(*p++-'0');}
    if(!digits || n<25U || n>2375U || n%50U!=25U)return 0U;
    *cursor=p;*value=(uint16_t)n;return 1U;
}
uint8_t Screen_ParseBatch(const char *text,screen_batch_t *batch)
{
    screen_batch_t candidate;const char *p=text;
    if(!p || !batch || strncmp(p,"&clear",6U))return 0U;
    memset(&candidate,0,sizeof(candidate));p+=6;
    while(*p) {
        screen_point_mm_t point;
        if(candidate.count>=SCREEN_MAX_OBSTACLES || strncmp(p,"&obs",4U) ||
           (p[4]!=' ' && p[4]!='\t'))return 0U;
        p+=4;
        if(!number(&p,&point.x) || (*p!=' ' && *p!='\t') || !number(&p,&point.y))return 0U;
        while(*p==' ' || *p=='\t')p++;
        if(*p && *p!='&')return 0U;
        candidate.points[candidate.count++]=point;
    }
    *batch=candidate;return 1U;
}
void Screen_ParserInit(screen_parser_t *p){memset(p,0,sizeof(*p));p->page=255U;}
void Screen_ParserDrop(screen_parser_t *p)
{p->length=p->binary_length=p->ff_count=0U;p->dropping=1U;}
screen_event_t Screen_ParserFeed(screen_parser_t *p,uint8_t byte,uint32_t now,screen_batch_t *batch)
{
    uint8_t timed_out=(uint8_t)((p->length || p->binary_length || p->dropping) && now-p->last_byte>=SCREEN_RX_TIMEOUT_MS);
    if(timed_out){p->length=p->binary_length=p->ff_count=p->dropping=0U;}
    p->last_byte=now;
    if(p->dropping) {if(byte=='\n'){p->dropping=0U;return SCREEN_EVENT_REJECT;}return SCREEN_EVENT_NONE;}
    if(p->length) {
        if(byte=='\n') {
            uint8_t ok;
            if(p->length && p->line[p->length-1U]=='\r')p->length--;
            p->line[p->length]='\0';ok=Screen_ParseBatch(p->line,batch);p->length=0;
            return ok?SCREEN_EVENT_BATCH:SCREEN_EVENT_REJECT;
        }
        if((byte<32U && byte!='\r') || byte>126U || p->length>=SCREEN_LINE_CAPACITY-1U) {Screen_ParserDrop(p);return SCREEN_EVENT_NONE;}
        p->line[p->length++]=(char)byte;return SCREEN_EVENT_NONE;
    }
    if(!p->binary_length && byte=='&'){p->line[0]='&';p->length=1;return SCREEN_EVENT_NONE;}
    /* Ignore standard TJC ACK/touch/startup packets; sendme returns
     * 66 PAGE FF FF FF. Never interpret '&' inside a binary return packet. */
    if((byte=='\n' || byte=='\r') && !p->binary_length)return SCREEN_EVENT_NONE;
    if(p->binary_length<sizeof(p->binary))p->binary[p->binary_length++]=byte;
    p->ff_count=byte==255U?(uint8_t)(p->ff_count+1U):0U;
    if(p->ff_count==3U) {
        uint8_t page_packet=(uint8_t)(p->binary_length==5U && p->binary[0]==0x66U);
        if(page_packet)p->page=p->binary[1];
        p->binary_length=p->ff_count=0;
        return page_packet?SCREEN_EVENT_PAGE:SCREEN_EVENT_NONE;
    }
    return SCREEN_EVENT_NONE;
}
