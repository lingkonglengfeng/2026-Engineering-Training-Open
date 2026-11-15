#include "zdt_query.h"
#include <math.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do {++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static uint32_t sends,reports;static uint8_t reject;static const char *last_reason;
static uint8_t send_query(uint8_t addr,uint8_t cmd)
{CHECK(addr>=1U && addr<=6U && (cmd==0x32U || cmd==0x36U));sends++;return (uint8_t)(sends!=reject);}
static void report_query(const zdt_query_t *q,const char *reason)
{CHECK(q && !q->running);reports++;last_reason=reason;}
static void reply(zdt_query_t *q,uint8_t addr,uint8_t cmd,uint8_t sign,uint32_t magnitude,uint32_t tick,uint32_t seq)
{
    uint8_t p[7]={cmd,sign,(uint8_t)(magnitude>>24U),(uint8_t)(magnitude>>16U),(uint8_t)(magnitude>>8U),(uint8_t)magnitude,0x6BU};
    ZDTQuery_Frame(q,addr,p,7U,tick,seq);
}
void Test_ZdtMotorQuery(void)
{
    zdt_query_t q={0};zdt_query_ports_t ports={send_query,report_query};uint32_t before;uint8_t i;
    CHECK(!ZDTQuery_Start(&q,&ports,0U,0U,0U));CHECK(!ZDTQuery_Start(&q,&ports,7U,0U,0U));
    sends=reports=reject=0U;
    CHECK(ZDTQuery_Start(&q,&ports,5U,1000U,50U));CHECK(sends==2U);
    CHECK(!ZDTQuery_Start(&q,&ports,6U,1000U,50U));
    reply(&q,5U,0x36U,0U,65536U,1005U,50U);CHECK(q.mask==0U); /* pre-request queue sequence */
    reply(&q,5U,0x36U,0U,65536U,999U,51U);CHECK(q.mask==0U); /* old capture time */
    reply(&q,6U,0x36U,0U,65536U,1005U,51U);CHECK(q.mask==0U);
    reply(&q,5U,0x36U,2U,65536U,1005U,51U);CHECK(q.mask==0U);
    reply(&q,5U,0x32U,0U,3200U,1005U,51U);CHECK(q.mask==2U && q.running && reports==0U);
    reply(&q,5U,0x32U,0U,9999U,1006U,52U);CHECK(q.input_pulses==3200); /* duplicate does not overwrite */
    reply(&q,5U,0x36U,1U,65536U,1010U,53U);
    CHECK(q.mask==3U && !q.running && q.position_counts==-65536 && q.input_pulses==3200);
    CHECK(fabs(ZDTQuery_AngleDeg(&q)+360.0)<0.0001 && reports==1U && !last_reason);
    CHECK(ZDTQuery_CircleDeg(&q)==0.0);
    reply(&q,5U,0x36U,0U,123U,1011U,54U);CHECK(reports==1U && q.position_counts==-65536);
    CHECK(ZDTQuery_Start(&q,&ports,1U,2000U,70U));
    reply(&q,1U,0x36U,0U,4294967295U,2010U,71U);
    reply(&q,1U,0x32U,1U,4294967295U,2011U,72U);
    CHECK(q.position_counts==INT64_C(4294967295) && q.input_pulses==-INT64_C(4294967295));
    CHECK(ZDTQuery_AngleDeg(&q)>23592959.0); /* no signed32 overflow */
    CHECK(ZDTQuery_CircleDeg(&q)>=0.0 && ZDTQuery_CircleDeg(&q)<360.0);
    CHECK(ZDTQuery_Start(&q,&ports,2U,3000U,80U));
    reply(&q,2U,0x36U,0U,0U,3001U,81U);before=reports;
    ZDTQuery_Service(&q,3399U);CHECK(q.running);
    ZDTQuery_Service(&q,3400U);CHECK(!q.running && q.mask==1U && reports==before+1U && last_reason);
    ZDTQuery_Service(&q,3401U);CHECK(reports==before+1U);
    CHECK(ZDTQuery_Start(&q,&ports,3U,UINT32_MAX-10U,UINT32_MAX-1U));
    reply(&q,3U,0x36U,0U,65536U,UINT32_MAX-1U,UINT32_MAX);
    reply(&q,3U,0x32U,0U,0U,5U,0U);CHECK(!q.running && q.mask==3U);
    for(i=1U;i<=6U;i++) {
        CHECK(ZDTQuery_Start(&q,&ports,i,4000U,0U));
        reply(&q,i,0x32U,0U,123U,4001U,1U);reply(&q,i,0x36U,0U,131072U,4002U,2U);
        CHECK(!q.running && q.mask==3U && ZDTQuery_AngleDeg(&q)==720.0);
    }
    before=reports;reject=(uint8_t)(sends+2U);
    CHECK(!ZDTQuery_Start(&q,&ports,4U,5000U,10U));CHECK(!q.running && reports==before+1U && last_reason);
    reject=0U;
}
