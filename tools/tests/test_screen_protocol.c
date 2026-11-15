#include "screen_protocol.h"
#include "screen_metrics.h"
#include "grid_nav.h"
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);}}while(0)
static screen_event_t feed(screen_parser_t *p,const char *text,uint32_t now,screen_batch_t *b)
{screen_event_t e=SCREEN_EVENT_NONE;while(*text)e=Screen_ParserFeed(p,(uint8_t)*text++,now,b);return e;}
void Test_ScreenProtocol(void)
{
    screen_batch_t b,old;screen_parser_t p;uint16_t x[19],y[19];uint8_t i;uint32_t rev;
    uint8_t command[64];size_t n;screen_snapshot_t s;
    static const char *bad[]={"", "&obs 1225 1225", "&clear&clear", "&clear&obs 25", "&clear&obs -25 25",
      "&clear&obs 2400 25", "&clear&obs 24 25", "&clear&obs 25 26", "&clear&obs 25 25junk",
      "&clear&obs 25 25&ok", "&clear&obs 12345 25", "&clear&obs 25 25&", "&clearx"};
    CHECK(Screen_ParseBatch("&clear&obs 1225 1225&obs 1075 625&obs 175 1225",&b));
    CHECK(b.count==3 && b.points[0].x==1225 && b.points[1].y==625 && b.points[2].x==175);
    old=b;
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++){CHECK(!Screen_ParseBatch(bad[i],&b));CHECK(!memcmp(&old,&b,sizeof(b)));}
    CHECK(Screen_ParseBatch("&clear&obs 25 25",&b) && b.points[0].x==25 && b.points[0].y==25);
    GridNav_Init();GridNav_SetObstacle(12,12,1);rev=GridNav_GetRevision();
    x[0]=25;y[0]=25;CHECK(GridNav_ReplaceObstaclesMm(x,y,1));
    CHECK(GridNav_IsObstacle(0,47) && !GridNav_IsObstacle(12,12) && GridNav_GetRevision()==rev+1U);
    x[0]=2400;CHECK(!GridNav_ReplaceObstaclesMm(x,y,1) && GridNav_IsObstacle(0,47));
    CHECK(GridNav_ReplaceObstaclesMm(NULL,NULL,0) && !GridNav_IsObstacle(0,47));
    CHECK(!GridNav_ReplaceObstaclesMm(NULL,NULL,1));
    Screen_ParserInit(&p);
    CHECK(feed(&p,"&clear&obs 25",100,&b)==SCREEN_EVENT_NONE);
    CHECK(feed(&p," 25\n",150,&b)==SCREEN_EVENT_BATCH && b.count==1);
    CHECK(feed(&p,"&clear\r\n",200,&b)==SCREEN_EVENT_BATCH && b.count==0);
    CHECK(feed(&p,"&clear&obs 12\r25 1225\n",210,&b)==SCREEN_EVENT_REJECT);
    CHECK(feed(&p,"\r\n&clear\n",220,&b)==SCREEN_EVENT_BATCH);
    CHECK(feed(&p,"&clear&obs 25",300,&b)==SCREEN_EVENT_NONE);
    CHECK(feed(&p,"&clear&obs 75 75\n",1400,&b)==SCREEN_EVENT_BATCH && b.points[0].x==75);
    Screen_ParserInit(&p);CHECK(feed(&p,"&clear",UINT32_MAX-20U,&b)==SCREEN_EVENT_NONE);
    CHECK(feed(&p,"\n",5,&b)==SCREEN_EVENT_BATCH); /* valid HAL rollover */
    Screen_ParserInit(&p);Screen_ParserFeed(&p,'&',0,&b);
    for(i=0;i<3;i++){uint16_t j;for(j=0;j<200;j++)Screen_ParserFeed(&p,'x',1,&b);}
    CHECK(Screen_ParserFeed(&p,'\n',2,&b)==SCREEN_EVENT_REJECT);
    CHECK(feed(&p,"&clear\n",3,&b)==SCREEN_EVENT_BATCH);
    {const uint8_t page[]={0x66,1,255,255,255};for(i=0;i<5;i++)Screen_ParserFeed(&p,page[i],4,&b);CHECK(p.page==1);}
    {const uint8_t touch[]={0x65,0x26,2,1,255,255,255};for(i=0;i<7;i++)Screen_ParserFeed(&p,touch[i],5,&b);CHECK(!p.length);}
    ScreenMetrics_ClearTask();ScreenMetrics_ResetCounts();ScreenMetrics_SetState(SCREEN_WAIT_QR);
    CHECK(ScreenMetrics_SetTask("156+123+516+231"));CHECK(!ScreenMetrics_SetTask("156\"123+516+231"));
    ScreenMetrics_Pick();ScreenMetrics_Place();ScreenMetrics_Place();ScreenMetrics_Snapshot(&s);
    CHECK(s.pick==1 && s.place==2 && s.state==SCREEN_SCAN_OK);
    for(i=0;i<5;i++){n=ScreenMetrics_Command(&s,i,command,sizeof(command));CHECK(n>=4 && command[n-1]==255 && command[n-2]==255 && command[n-3]==255);command[n-3]=0;
      if(i==0)CHECK(!strcmp((char *)command,"tCode1.txt=\"156+123\""));
      if(i==1)CHECK(!strcmp((char *)command,"tCode2.txt=\"516+231\""));
      if(i==2)CHECK(!strcmp((char *)command,"nPick.val=1"));
      if(i==3)CHECK(!strcmp((char *)command,"nPlace.val=2"));}
    CHECK(!ScreenMetrics_Command(&s,0,command,4));
    ScreenMetrics_ResetCounts();ScreenMetrics_ClearTask();ScreenMetrics_SetState(SCREEN_WAIT_QR);GridNav_Init();
}
