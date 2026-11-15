#include "qr_scan.h"
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
static void group(uint8_t *p,uint8_t id,uint8_t seq)
{
    static const uint8_t data[4][3]={{1,6,5},{1,2,3},{6,1,5},{2,3,1}};
    memset(p,0,8U);p[1]=VISION_TYPE_QR_GROUP;p[2]=(uint8_t)(0x80U|(seq<<3U)|id);
    memcpy(p+3,data[id-1U],3U);p[6]=4U;
}
void Test_QrScan(void)
{
    vision_qr_assembly_t q;qr_scan_t scan;vision_camera_state_t c={0};uint8_t p[8],id;uint32_t now;
    VisionQr_Reset(&q);
    for(id=4U;id>0U;id--){group(p,id,3U);CHECK(VisionQr_Feed(&q,p,3U,5U*(4U-id))==(id==1U));}
    CHECK(q.complete && strcmp(q.task.raw,"156+123+516+231")==0);
    CHECK(q.task.colors[0][1]==6U && q.task.colors[0][2]==5U && q.task.positions[1][0]==2U);
    group(p,1U,3U);CHECK(VisionQr_Feed(&q,p,3U,30U)==0U);
    p[3]=2U;CHECK(VisionQr_Feed(&q,p,3U,35U)==255U && strcmp(q.task.raw,"156+123+516+231")==0);
    p[2]&=0x7FU;p[3]=p[4]=p[5]=0U;CHECK(VisionQr_Feed(&q,p,3U,40U)==0U && q.complete);
    VisionQr_Reset(&q);group(p,1U,3U);CHECK(VisionQr_Feed(&q,p,4U,0U)==0U && !q.mask);
    CHECK(VisionQr_Feed(&q,p,3U,0U)==0U);CHECK(VisionQr_Feed(&q,p,3U,45U)==0U && q.started==0U);
    group(p,2U,3U);CHECK(VisionQr_Feed(&q,p,3U,51U)==0U && q.mask==2U);
    p[4]=1U;CHECK(VisionQr_Feed(&q,p,3U,52U)==255U);
    group(p,2U,3U);p[3]=4U;CHECK(VisionQr_Feed(&q,p,3U,53U)==255U);
    group(p,1U,3U);p[6]=2U;CHECK(VisionQr_Feed(&q,p,3U,54U)==255U);
    VisionQr_Reset(&q);
    for(id=1U;id<=4U;id++){group(p,id,15U);CHECK(VisionQr_Feed(&q,p,15U,UINT32_MAX-10U+id*5U)==(id==4U));}
    q.task.observation_id=1U;
    /* A new SET is mandatory even if the old camera mode already matches. */
    c.mode=VISION_MODE_QR;c.station=4U;c.hz=20U;c.confirmed=1U;c.request_seq=14U;
    QrScan_Start(&scan,0U);CHECK(QrScan_Step(&scan,0U,&c,&q.task)==QR_SCAN_SEND_MODE);
    QrScan_ModeSent(&scan,15U);c.request_seq=15U;c.pending=1U;
    CHECK(QrScan_Step(&scan,1U,&c,&q.task)==QR_SCAN_WAIT);
    c.pending=0U;CHECK(QrScan_Step(&scan,2U,&c,&q.task)==QR_SCAN_SEND_STOP && scan.phase==QR_SCAN_STOP);
    QrScan_StopSent(&scan);c.mode=0U;c.station=0U;c.hz=0U;c.request_seq=15U;
    CHECK(QrScan_Step(&scan,3U,&c,NULL)==QR_SCAN_WAIT); /* old STOP nonce cannot finish */
    c.request_seq=0U;CHECK(QrScan_Step(&scan,4U,&c,NULL)==QR_SCAN_READY);
    CHECK(strcmp(scan.task.raw,"156+123+516+231")==0);
    QrScan_Cancel(&scan);CHECK(scan.phase==QR_SCAN_IDLE);
    QrScan_Start(&scan,100U);CHECK(QrScan_Step(&scan,2600U,&c,NULL)==QR_SCAN_ERROR);
    QrScan_Start(&scan,UINT32_MAX-5U);QrScan_ModeSent(&scan,2U);
    c.mode=6U;c.station=4U;c.hz=20U;c.request_seq=2U;now=20U;
    CHECK(QrScan_Step(&scan,now,&c,NULL)==QR_SCAN_WAIT && scan.phase==QR_SCAN_READ);
    CHECK(QrScan_Step(&scan,now+15000U,&c,NULL)==QR_SCAN_ERROR);
}
