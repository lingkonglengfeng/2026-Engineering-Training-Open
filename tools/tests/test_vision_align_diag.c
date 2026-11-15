#include "vision_align_diag.h"
#include <math.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
static vision_align_diag_t diag;
void Test_VisionAlignmentDiagnostic(void)
{
    vision_align_config_t c;vision_diag_input_t s={0};vision_pixel_t p={0};
    vision_align_t control,reference;float f,r,rf,rr;uint32_t t,i,sum;uint16_t count;
    int32_t wheels[4]={100,200,-300,-400};char line[384];
    VisionAlign_DefaultConfig(&c);c.camera=1U;c.type=VISION_TYPE_RING_PX;c.id=2U;
    c.forward_x=c.right_y=0.0007f;
    VisionDiag_Init(&diag);CHECK(diag.enabled && !diag.active && !diag.has_data);
    diag.enabled=0U;VisionDiag_Begin(&diag,1U,&c,0U);CHECK(!diag.active);
    diag.enabled=1U;VisionDiag_Begin(&diag,2U,&c,100U);
    CHECK(diag.active && diag.count==0U);
    CHECK(!VisionDiag_Line(&diag,0U,line,sizeof(line))); /* no motion formatting */
    s.now=100U;s.phase=V_DIAG_ACK;s.age=UINT32_MAX;
    VisionDiag_Observe(&diag,&s);
    s.now=150U;s.phase=V_DIAG_BASE;VisionDiag_Observe(&diag,&s);
    VisionDiag_Wheels(&diag,wheels,15U,0U);
    CHECK(diag.start_mask==15U && diag.wheel_start[2]==-300);
    s.now=200U;s.phase=V_DIAG_SETTLE;s.stopped=1U;s.context=1U;
    s.state=V_ALIGN_SETTLE;VisionDiag_Observe(&diag,&s);
    p.observation_id=1U;p.type=c.type;p.id=c.id;p.valid=1U;p.dx_px=50.0f;p.dy_px=-20.0f;
    s.pixel=&p;s.age=0U;s.now=450U;s.phase=V_DIAG_PULSE;s.state=V_ALIGN_PULSE;
    s.forward=0.025f;s.right=-0.01f;s.omega=0.001f;s.yaw=-720.25f;s.gyro=2.5f;
    s.rpm[0]=-9;s.rpm[1]=9;s.stopped=0U;VisionDiag_Observe(&diag,&s);
    CHECK(diag.new_frames==1U && diag.pulses==1U);
    CHECK(diag.last.forward==250 && diag.last.right==-100 && diag.last.yaw==-25);
    count=diag.count;s.now=451U;s.age=1U;VisionDiag_Observe(&diag,&s);
    CHECK(diag.count==count && diag.same_observation_polls==1U);
    /* New frame during a held pulse is recorded independently of control. */
    s.now=500U;s.age=0U;p.observation_id=2U;p.dx_px=35.0f;VisionDiag_Observe(&diag,&s);
    CHECK(diag.count==count+1U && diag.gap_max==50U && diag.gaps[0]==1U);
    s.now=550U;s.phase=V_DIAG_BRAKE;s.state=V_ALIGN_SETTLE;
    s.forward=s.right=s.omega=0.0f;VisionDiag_Observe(&diag,&s);
    s.now=600U;s.phase=V_DIAG_SETTLE;s.stopped=1U;VisionDiag_Observe(&diag,&s);
    s.now=850U;s.phase=V_DIAG_WAIT;s.state=V_ALIGN_WAIT;s.age=350U;p.valid=0U;
    VisionDiag_Observe(&diag,&s);
    s.now=900U;s.phase=V_DIAG_END;VisionDiag_Observe(&diag,&s);
    wheels[0]+=50;VisionDiag_Wheels(&diag,wheels,15U,1U);
    VisionDiag_Finish(&diag,950U,1U,NULL);
    CHECK(!diag.active && diag.elapsed==850U && diag.end_mask==15U && diag.wheel_end[0]==150);
    sum=0U;for(i=0;i<V_DIAG_PHASE_COUNT;i++)sum+=diag.phase_ms[i];CHECK(sum==diag.elapsed);
    CHECK(diag.phase_ms[V_DIAG_ACK]==50U && diag.phase_ms[V_DIAG_PULSE]==100U);
    CHECK(diag.phase_ms[V_DIAG_BRAKE]==50U && diag.phase_ms[V_DIAG_WAIT]==50U);
    CHECK(VisionDiag_Line(&diag,0U,line,sizeof(line)) && strstr(line,"station=2") && strstr(line,"dropped=0"));
    CHECK(VisionDiag_Line(&diag,3U,line,sizeof(line)) && strstr(line,"PULSE=100"));
    CHECK(VisionDiag_Line(&diag,7U,line,sizeof(line)) && strstr(line,"cmd_rpm"));
    for(i=0;i<9U+diag.count;i++)CHECK(VisionDiag_Line(&diag,(uint16_t)i,line,sizeof(line)));
    CHECK(!VisionDiag_Line(&diag,(uint16_t)(9U+diag.count),line,sizeof(line)));
    CHECK(!VisionDiag_Line(&diag,0U,line,3U));
    /* Even saturated counters must export a complete summary line. */
    diag.new_frames=diag.invalid_frames=diag.same_observation_polls=UINT32_MAX;
    diag.gap_max=diag.tick_max=diag.can_failures=UINT32_MAX;
    for(i=0;i<5U;i++)diag.gaps[i]=UINT32_MAX;
    CHECK(VisionDiag_Line(&diag,4U,line,sizeof(line)) && strstr(line,"over:4294967295"));
    count=diag.count;VisionDiag_Observe(&diag,&s);CHECK(diag.count==count);
    /* Repeated velocity fade at 1ms cannot consume 40 rows. Fresh frames,
     * scene changes and state transitions bypass the 20ms command interval. */
    VisionDiag_Begin(&diag,2U,&c,1000U);
    s.phase=V_DIAG_PULSE;s.state=V_ALIGN_SEARCH;s.age=0U;
    s.scene_count=2U;s.scene_valid=1U;s.forward=0.015f;s.right=s.omega=0.0f;
    p.valid=0U;p.observation_id=99U;s.now=1000U;VisionDiag_Observe(&diag,&s);
    for(i=1U;i<=40U;i++) {
        s.now=1000U+i;s.age=i;s.forward=0.015f*(1.0f-i/80.0f);
        VisionDiag_Observe(&diag,&s);
    }
    CHECK(diag.count==3U && diag.rows[1].ms==20U && diag.rows[2].ms==40U);
    count=diag.count;s.now=1041U;s.scene_count=3U;VisionDiag_Observe(&diag,&s);
    CHECK(diag.count==count+1U);
    p.observation_id++;s.now=1042U;VisionDiag_Observe(&diag,&s);CHECK(diag.count==count+2U);
    s.now=1043U;s.state=V_ALIGN_WAIT;VisionDiag_Observe(&diag,&s);CHECK(diag.count==count+3U);
    /* Fixed-size buffer preserves first 511 rows and latest tail; explicit
     * overflow counts every displaced record instead of losing the fault. */
    VisionDiag_Begin(&diag,1U,&c,1000U);s.phase=V_DIAG_WAIT;s.age=0U;
    for(i=0;i<V_ALIGN_DIAG_CAPACITY+7U;i++) {
        s.now=1000U+i*50U;p.observation_id=i+10U;VisionDiag_Observe(&diag,&s);
    }
    CHECK(diag.count==V_ALIGN_DIAG_CAPACITY && diag.overflow==7U);
    CHECK(diag.rows[0].observation==10U && diag.rows[510].observation==520U &&
          diag.rows[511].observation==528U && diag.rows[511].ms==s.now-1000U);
    VisionDiag_Finish(&diag,s.now+1U,0U,"cancel/reset");
    CHECK(VisionDiag_Line(&diag,0U,line,sizeof(line)) && strstr(line,"dropped=7") && strstr(line,"cancel/reset"));
    /* Unsigned tick rollover and absent camera frames remain bounded. */
    VisionDiag_Begin(&diag,2U,&c,UINT32_MAX-20U);s.pixel=NULL;s.now=10U;s.phase=V_DIAG_ACK;
    VisionDiag_Observe(&diag,&s);VisionDiag_Finish(&diag,20U,0U,"no camera");
    CHECK(diag.elapsed==41U && diag.new_frames==0U && diag.phase_ms[V_DIAG_ACK]==41U);
    /* Observe the real controller and compare it against an uninstrumented
     * copy on every tick: diagnostics must not affect speed/state/budgets. */
    CHECK(VisionAlign_Start(&control,&c,0U,0U));reference=control;
    VisionDiag_Begin(&diag,2U,&c,0U);s.pixel=&p;p.valid=1U;p.dx_px=30.0f;p.dy_px=-20.0f;
    for(t=0U;t<=2000U;t+=10U) {
        if(t%50U==0U)p.observation_id++;
        VisionAlign_Step(&control,t,&p,t%50U,1U,&f,&r);
        VisionAlign_Step(&reference,t,&p,t%50U,1U,&rf,&rr);
        s.now=t;s.age=t%50U;s.forward=f;s.right=r;s.state=(uint8_t)control.state;
        s.phase=control.state==V_ALIGN_PULSE?V_DIAG_PULSE:
            (control.state==V_ALIGN_SETTLE?V_DIAG_SETTLE:V_DIAG_WAIT);
        VisionDiag_Observe(&diag,&s);
        CHECK(f==rf && r==rr && memcmp(&control,&reference,sizeof(control))==0);
    }
}
