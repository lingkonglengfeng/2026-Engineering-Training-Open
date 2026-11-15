#include "vision_align.h"
#include "vision_mode_gate.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
static vision_align_config_t config(void)
{
    vision_align_config_t c;VisionAlign_DefaultConfig(&c);
    c.camera=1U;c.type=VISION_TYPE_RING_PX;c.id=2U;
    c.forward_x=0.0007f;c.right_y=0.0007f;return c;
}
static vision_pixel_t pixel(float x,float y,uint32_t observation)
{
    vision_pixel_t p={0};p.type=VISION_TYPE_RING_PX;p.id=2U;p.valid=1U;
    p.dx_px=x;p.dy_px=y;p.observation_id=observation;p.seq=(uint8_t)(observation%16U);return p;
}
static void ready(vision_align_t *a,const vision_align_config_t *c,uint32_t now)
{
    float f,r;CHECK(VisionAlign_Start(a,c,now,1U));
    VisionAlign_Step(a,now,NULL,0U,1U,&f,&r);
    VisionAlign_Step(a,now+c->settle_ms,NULL,0U,1U,&f,&r);
    CHECK(a->state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
}
void Test_VisionAlignment(void)
{
    vision_align_t a;vision_align_config_t c=config();vision_pixel_t p;
    float f,r;uint32_t t,i;
    CHECK(VisionAlign_ConfigValid(&c));
    c.camera=0U;CHECK(!VisionAlign_ConfigValid(&c));c=config();
    c.right_y=0.0f;CHECK(!VisionAlign_ConfigValid(&c));c=config();
    c.max_speed=NAN;CHECK(!VisionAlign_ConfigValid(&c));c=config();
    ready(&a,&c,0U);p=pixel(100.0f,-80.0f,2U);
    VisionAlign_Step(&a,250U,&p,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_PULSE && f>0.0f && r<0.0f);
    CHECK(fabsf(f)+fabsf(r)<=c.max_speed+1e-6f);
    CHECK(fabsf(r/f+0.8f)<0.0001f);
    p.valid=0U;VisionAlign_Step(&a,260U,&p,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_SETTLE && f==0.0f && r==0.0f);
    CHECK(a.drive_ms==10U);
    VisionAlign_Cancel(&a);VisionAlign_Step(&a,270U,&p,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_IDLE && f==0.0f && r==0.0f);

    /* Stale pre-arrival pixels and repeated reads cannot finish an alignment. */
    ready(&a,&c,1000U);p=pixel(0.0f,0.0f,2U);
    VisionAlign_Step(&a,1250U,&p,1U,1U,&f,&r);CHECK(!a.stable_count);
    p.observation_id=3U;VisionAlign_Step(&a,1300U,&p,0U,1U,&f,&r);
    CHECK(a.stable_count==1U);
    for(t=1310U;t<1490U;t+=10U)VisionAlign_Step(&a,t,&p,t-1300U,1U,&f,&r);
    CHECK(a.stable_count==1U && a.state!=V_ALIGN_DONE);
    for(i=0U;i<4U;i++) {
        p.observation_id++;VisionAlign_Step(&a,1500U+i*50U,&p,0U,1U,&f,&r);
    }
    CHECK(a.state==V_ALIGN_DONE && f==0.0f && r==0.0f);

    /* Changed ID or port/type cannot redirect a pulse toward another target. */
    ready(&a,&c,2000U);p=pixel(60.0f,0.0f,2U);p.id=3U;
    VisionAlign_Step(&a,2250U,&p,0U,1U,&f,&r);CHECK(a.state==V_ALIGN_WAIT && f==0.0f);
    p.id=2U;p.type=VISION_TYPE_PLATE_PX;
    VisionAlign_Step(&a,2300U,&p,0U,1U,&f,&r);CHECK(a.state==V_ALIGN_WAIT && f==0.0f);
    p=pixel(NAN,0.0f,3U);VisionAlign_Step(&a,2350U,&p,0U,1U,&f,&r);CHECK(f==0.0f);
    VisionAlign_Step(&a,5300U,NULL,0U,1U,&f,&r);CHECK(a.state==V_ALIGN_FAILED);

    /* Real wire SEQ wraps; distinct local observation IDs remain distinct. */
    ready(&a,&c,6000U);p=pixel(0.0f,0.0f,16U);p.seq=0U;
    VisionAlign_Step(&a,6250U,&p,0U,1U,&f,&r);
    for(i=1U;i<=4U;i++) {
        p.observation_id=16U+16U*i;VisionAlign_Step(&a,6250U+i*100U,&p,0U,1U,&f,&r);
    }
    CHECK(a.state==V_ALIGN_DONE);
    ready(&a,&c,7000U);p=pixel(20.0f,0.0f,2U);
    VisionAlign_Step(&a,7250U,&p,VISION_PIXEL_TIMEOUT_MS,1U,&f,&r);CHECK(f==0.0f);

    /* One-axis plate docking ignores X, but still requires fresh valid frames. */
    c.type=VISION_TYPE_PLATE_PX;c.id=0U;c.axes=2U;c.forward_x=0.0f;
    CHECK(VisionAlign_ConfigValid(&c));ready(&a,&c,8000U);
    p=pixel(200.0f,0.0f,2U);p.type=VISION_TYPE_PLATE_PX;p.id=0U;
    for(i=0U;i<5U;i++){p.observation_id++;VisionAlign_Step(&a,8250U+i*50U,&p,0U,1U,&f,&r);}
    CHECK(a.state==V_ALIGN_DONE);

    /* Physical pixel-response simulation: crossed camera axes, settle and
     * delayed observations. Verify convergence and strict total motion budget. */
    c=config();ready(&a,&c,10000U);
    {
        float x=35.0f,y=-25.0f;uint32_t received=10250U,observation=2U;
        p=pixel(x,y,observation);
        for(t=10250U;t<39000U && a.state!=V_ALIGN_DONE && a.state!=V_ALIGN_FAILED;t+=10U) {
            if(t%50U==0U){received=t;p=pixel(x,y,++observation);}
            VisionAlign_Step(&a,t,&p,t-received,1U,&f,&r);
            x-=f*4000.0f*0.010f;y-=r*4000.0f*0.010f;
            CHECK(fabsf(f)+fabsf(r)<=c.max_speed+1e-6f);
        }
        CHECK(a.state==V_ALIGN_DONE && fabsf(x)<=c.tolerance_px && fabsf(y)<=c.tolerance_px);
    }
    c=config();c.max_drive_ms=100U;ready(&a,&c,40000U);p=pixel(300.0f,200.0f,2U);
    VisionAlign_Step(&a,40250U,&p,0U,1U,&f,&r);
    VisionAlign_Step(&a,40350U,&p,100U,1U,&f,&r);
    VisionAlign_Step(&a,40360U,NULL,0U,1U,&f,&r);
    VisionAlign_Step(&a,40610U,NULL,0U,1U,&f,&r);
    p.observation_id=3U;VisionAlign_Step(&a,40620U,&p,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && r==0.0f);
    ready(&a,&c,UINT32_MAX-100U);
    VisionAlign_Step(&a,3200U,NULL,0U,1U,&f,&r);CHECK(a.state==V_ALIGN_FAILED);
    {
        vision_mode_gate_t gate;vision_camera_state_t state={0};
        /* Same ring algorithm at station2 and station3 is a NEW context.
         * A valid station2 position must never authorize station3 correction. */
        state.mode=VISION_MODE_RING;state.station=2U;state.target=2U;
        state.hz=20U;state.confirmed=1U;state.result=0U;state.request_seq=14U;
        VisionModeGate_Start(&gate,1U,VISION_MODE_RING,3U,2U,0U);
        CHECK(!VisionModeGate_Matches(&gate,&state));
        CHECK(VisionModeGate_Step(&gate,10U,&state)==V_MODE_SEND);
        /* One pending SET from another stage is allowed to finish, not overwritten. */
        state.pending=1U;CHECK(VisionModeGate_Step(&gate,20U,&state)==V_MODE_WAIT);
        state.pending=0U;CHECK(VisionModeGate_Step(&gate,30U,&state)==V_MODE_SEND);
        VisionModeGate_Sent(&gate,15U);state.pending=1U;state.request_seq=15U;
        CHECK(VisionModeGate_Step(&gate,40U,&state)==V_MODE_WAIT);
        state.pending=0U;state.mode=VISION_MODE_RING;state.station=3U;
        CHECK(VisionModeGate_Step(&gate,50U,&state)==V_MODE_READY);
        state.confirmed=0U;CHECK(!VisionModeGate_Matches(&gate,&state));
        /* Reuse ACK only when all source/station/target/flags fields match. */
        state.confirmed=1U;
        VisionModeGate_Start(&gate,1U,VISION_MODE_RING,3U,2U,100U);
        CHECK(VisionModeGate_Step(&gate,100U,&state)==V_MODE_READY && !gate.sent);
        state.reset_pending=1U;CHECK(!VisionModeGate_Matches(&gate,&state));state.reset_pending=0U;
        state.flags=1U;CHECK(!VisionModeGate_Matches(&gate,&state));state.flags=0U;
        VisionModeGate_Start(&gate,1U,VISION_MODE_PLATE,1U,0U,200U);
        CHECK(VisionModeGate_Step(&gate,200U,&state)==V_MODE_SEND);
        VisionModeGate_Sent(&gate,0U);state.request_seq=0U;state.pending=1U;
        CHECK(VisionModeGate_Step(&gate,250U,&state)==V_MODE_WAIT);
        state.pending=0U;state.result=7U;
        CHECK(VisionModeGate_Step(&gate,300U,&state)==V_MODE_FAILED);
        CHECK(gate.reason!=NULL);
        VisionModeGate_Start(&gate,1U,VISION_MODE_PLATE,1U,0U,400U);
        VisionModeGate_Sent(&gate,1U);state.request_seq=1U;state.result=VISION_RESULT_TIMEOUT;
        CHECK(VisionModeGate_Step(&gate,1900U,&state)==V_MODE_FAILED);
        VisionModeGate_Start(&gate,1U,VISION_MODE_PLATE,1U,0U,2000U);
        VisionModeGate_Sent(&gate,2U);state.request_seq=2U;state.pending=1U;
        CHECK(VisionModeGate_Step(&gate,4500U,&state)==V_MODE_FAILED);
        /* A later request cannot complete the saved request, even at same mode. */
        VisionModeGate_Start(&gate,1U,VISION_MODE_RING,3U,2U,5000U);
        VisionModeGate_Sent(&gate,3U);state.request_seq=4U;
        CHECK(VisionModeGate_Step(&gate,5001U,&state)==V_MODE_FAILED);
        VisionModeGate_Start(&gate,1U,VISION_MODE_RING,3U,2U,UINT32_MAX-100U);
        CHECK(VisionModeGate_Step(&gate,2500U,&state)==V_MODE_FAILED);
        VisionModeGate_Start(&gate,0U,VISION_MODE_RING,3U,2U,0U);
        CHECK(gate.action==V_MODE_FAILED);
    }
}

static void Test_StoppedSparseVerification(void)
{
    vision_align_t a;vision_align_config_t c=config();vision_pixel_t p;
    vision_ring_count_t scene={0};float f,r;uint32_t start,t,received,i,age;
    c.mode=V_ALIGN_MODE_SMOOTH;c.station=2U;c.search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
    c.loss_ms=20000U;c.max_drive_ms=10000U;
    scene.id=2U;scene.station=2U;scene.count=3U;scene.request_seq=3U;
    /* Same receive cadence as field camera: 250ms. Expire the published
     * pixel/count at 200ms as Vision_GetRingScene does, without inventing data. */
    ready(&a,&c,100000U);start=received=100250U;p=pixel(2.0f,-3.0f,2U);
    scene.seq=p.seq;scene.observation_id=1U;
    for(t=start;t<=start+1000U;t+=10U) {
        if(t!=start && (t-start)%250U==0U) {
            received=t;p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
        }
        age=t-received;p.valid=scene.valid=(uint8_t)(age<200U);
        VisionAlign_StepScene(&a,t,&p,age,&scene,age,1U,&f,&r);
        CHECK(f==0.0f && r==0.0f && a.drive_ms==0U);
        CHECK(a.stable_count==(uint8_t)((t-start)/250U+1U));
        if(t<start+1000U)CHECK(a.state==V_ALIGN_WAIT);
    }
    CHECK(a.state==V_ALIGN_DONE && a.stable_count==5U);
    /* A duplicated measurement can neither complete nor extend the window. */
    ready(&a,&c,102000U);p=pixel(2.0f,-3.0f,20U);scene.valid=1U;scene.seq=p.seq;
    VisionAlign_StepScene(&a,102250U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==1U);
    p.valid=scene.valid=0U;
    VisionAlign_StepScene(&a,102750U,&p,500U,&scene,500U,1U,&f,&r);
    CHECK(a.stable_count==1U && a.state==V_ALIGN_WAIT && f==0.0f);
    VisionAlign_StepScene(&a,102751U,&p,501U,&scene,501U,1U,&f,&r);
    CHECK(a.stable_count==0U && a.state==V_ALIGN_WAIT && f==0.0f);
    p=pixel(1.0f,1.0f,21U);scene.valid=1U;scene.seq=p.seq;scene.observation_id++;
    VisionAlign_StepScene(&a,102800U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==1U);
    /* Fresh invalid middle position, wrong station, actual wheel motion or
     * an out-of-tolerance frame invalidates the stopped verification chain. */
    p.valid=0U;p.observation_id++;scene.observation_id++;scene.seq=p.seq=6U;
    VisionAlign_StepScene(&a,102850U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==0U);
    p=pixel(2.0f,1.0f,23U);scene.seq=p.seq;scene.observation_id++;
    VisionAlign_StepScene(&a,102900U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==1U);
    VisionAlign_StepScene(&a,102910U,&p,10U,&scene,10U,0U,&f,&r);CHECK(a.stable_count==0U);
    p.observation_id++;scene.observation_id++;scene.seq=p.seq=8U;
    VisionAlign_StepScene(&a,102950U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==1U);
    scene.station=3U;
    VisionAlign_StepScene(&a,102960U,&p,10U,&scene,10U,1U,&f,&r);CHECK(a.stable_count==0U);
    scene.station=2U;p.observation_id++;scene.observation_id++;scene.seq=p.seq=9U;
    VisionAlign_StepScene(&a,103000U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.stable_count==1U);
    p=pixel(6.0f,1.0f,26U);scene.seq=p.seq;scene.observation_id++;
    VisionAlign_StepScene(&a,103050U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.stable_count==0U && a.state==V_ALIGN_TRACK && f>0.0f);
    p.valid=scene.valid=0U;
    VisionAlign_StepScene(&a,103250U,&p,200U,&scene,200U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && f==0.0f && r==0.0f); /* motion still stops at 200ms */
    /* Plate source uses the same stopped-only rule, including tick wrap. */
    c.search_axis=0U;c.type=VISION_TYPE_PLATE_PX;c.id=0U;
    start=UINT32_MAX-600U;ready(&a,&c,start);start+=250U;
    for(i=0U;i<5U;i++) {
        received=start+i*250U;p=pixel(1.0f,2.0f,40U+i);p.type=c.type;p.id=0U;
        VisionAlign_Step(&a,received,&p,0U,1U,&f,&r);CHECK(a.stable_count==i+1U);
        p.valid=0U;VisionAlign_Step(&a,received+210U,&p,210U,1U,&f,&r);
        CHECK(a.stable_count==i+1U && f==0.0f && r==0.0f);
    }
    CHECK(a.state==V_ALIGN_DONE);
}
static void Test_SearchAndCadenceProfiles(void)
{
    vision_align_t a;vision_align_config_t c=config();vision_pixel_t p;
    vision_ring_count_t ring={0};vision_plate_scene_t plate={0};float f,r,held;uint32_t i,t;
    c.mode=V_ALIGN_MODE_SMOOTH;c.max_drive_ms=10000U;c.loss_ms=20000U;
    c.observation_ms=400U;c.fade_start_ms=260U;c.fade_zero_ms=360U;
    c.max_speed=0.040f;c.search_speed=0.020f;
    CHECK(VisionAlign_ConfigValid(&c));ready(&a,&c,200000U);
    p=pixel(30.0f,0.0f,2U);
    for(i=0U;i<5U;i++) {
        t=200250U+i*250U;p.observation_id++;
        VisionAlign_Step(&a,t,&p,0U,1U,&f,&r);held=f;
        CHECK(a.state==V_ALIGN_TRACK && f>0.0f);
        VisionAlign_Step(&a,t+240U,&p,240U,0U,&f,&r);
        CHECK(a.state==V_ALIGN_TRACK && f==held && !a.search_ms);
    }
    VisionAlign_Step(&a,201550U,&p,300U,0U,&f,&r);CHECK(f>0.0f && f<held);
    VisionAlign_Step(&a,201610U,&p,360U,0U,&f,&r);CHECK(f==0.0f);
    VisionAlign_Step(&a,201650U,&p,400U,0U,&f,&r);CHECK(a.state==V_ALIGN_WAIT && f==0.0f);
    p=pixel(30.0f,0.0f,20U);VisionAlign_Step(&a,201651U,&p,0U,1U,&f,&r);CHECK(f>0.0f);
    p.valid=0U;p.observation_id++;VisionAlign_Step(&a,201652U,&p,0U,0U,&f,&r);
    CHECK(f==0.0f && a.state==V_ALIGN_WAIT); /* no grace on explicitly invalid frames */
    c.station=3U;c.search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;ready(&a,&c,202000U);
    p=pixel(0.0f,0.0f,30U);p.valid=0U;
    ring.id=2U;ring.valid=1U;ring.count=2U;ring.station=3U;ring.seq=p.seq;ring.observation_id=1U;
    VisionAlign_StepScene(&a,202250U,&p,0U,&ring,0U,1U,&f,&r);
    CHECK(f==0.020f && r==0.0f && a.state==V_ALIGN_SEARCH);
    VisionAlign_StepScene(&a,202490U,&p,240U,&ring,240U,0U,&f,&r);CHECK(f==0.020f);
    ring.count=3U;p.observation_id++;ring.observation_id++;ring.seq=p.seq=15U;
    VisionAlign_StepScene(&a,202500U,&p,0U,&ring,0U,0U,&f,&r);CHECK(f==0.0f && a.state==V_ALIGN_WAIT);
    /* TEMP may have a valid two-ring count before target-2 has a pixel frame.
     * The image-right search must continue instead of waiting indefinitely. */
    ready(&a,&c,202700U);p.observation_id=0U;p.valid=0U;
    ring.count=2U;ring.observation_id=2U;ring.seq=7U;
    VisionAlign_StepScene(&a,202950U,NULL,UINT32_MAX,&ring,0U,1U,&f,&r);
    CHECK(f==0.020f && r==0.0f && a.state==V_ALIGN_SEARCH);
    ring.count=3U;ring.observation_id=3U;ring.seq=8U;
    VisionAlign_StepScene(&a,203000U,NULL,UINT32_MAX,&ring,0U,0U,&f,&r);
    CHECK(f==0.0f && r==0.0f && a.state==V_ALIGN_WAIT);
    ready(&a,&c,220000U);p=pixel(0.0f,0.0f,80U);p.valid=0U;
    ring.count=2U;ring.seq=p.seq;ring.observation_id++;
    VisionAlign_StepScene(&a,220250U,&p,0U,&ring,0U,1U,&f,&r);CHECK(f==0.020f);
    p=pixel(20.0f,0.0f,81U);ring.count=3U;ring.seq=p.seq;ring.observation_id++;
    VisionAlign_StepScene(&a,220300U,&p,5U,&ring,5U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && f>0.0f); /* same qualified image, no extra stop */
    ready(&a,&c,221000U);p=pixel(0.0f,0.0f,82U);p.valid=0U;
    ring.count=2U;ring.seq=p.seq;ring.observation_id++;
    VisionAlign_StepScene(&a,221250U,&p,0U,&ring,0U,1U,&f,&r);
    p=pixel(2.0f,2.0f,83U);ring.count=3U;ring.seq=p.seq;ring.observation_id++;
    VisionAlign_StepScene(&a,221300U,&p,0U,&ring,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_SETTLE && f==0.0f && r==0.0f); /* already centered: stop/verify */
    /* Plate invalid by itself or ACQUIRE is not permission to search. */
    c.type=VISION_TYPE_PLATE_PX;c.id=0U;c.station=1U;c.search_limit_ms=4000U;ready(&a,&c,203000U);
    plate.id=0U;plate.valid=1U;plate.station=1U;plate.observation_id=1U;
    p=pixel(0.0f,0.0f,50U);p.type=c.type;p.id=0U;p.valid=0U;
    for(i=0U;i<4U;i++) {
        p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)i;
        plate.status=VISION_PLATE_ACQUIRE;
        VisionAlign_StepTargets(&a,203250U+i*100U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);
        CHECK(f==0.0f && r==0.0f && a.plate_missing_frames==0U);
    }
    VisionAlign_Step(&a,203600U,&p,0U,1U,&f,&r);CHECK(f==0.0f);
    plate.status=VISION_PLATE_SEARCH;
    for(i=0U;i<3U;i++) {
        p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)(i+4U);
        VisionAlign_StepTargets(&a,203650U+i*250U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);
        CHECK(i==2U?(f==-0.020f && a.state==V_ALIGN_SEARCH):(f==0.0f));
        if(i==0U) {
            VisionAlign_StepTargets(&a,203651U,&p,1U,NULL,0U,&plate,1U,1U,&f,&r);
            CHECK(a.plate_missing_frames==1U && f==0.0f); /* repeated poll is not a new missing frame */
        }
    }
    p=pixel(20.0f,0.0f,70U);p.type=c.type;p.id=0U;plate.status=VISION_PLATE_FOUND;
    plate.observation_id++;plate.seq=p.seq;
    VisionAlign_StepTargets(&a,204200U,&p,0U,NULL,0U,&plate,0U,0U,&f,&r);
    CHECK(f>0.0f && a.plate_seen && a.state==V_ALIGN_TRACK);
    p.observation_id++;plate.observation_id++;plate.seq=p.seq=7U;
    VisionAlign_StepTargets(&a,204250U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);CHECK(f>0.0f && a.state==V_ALIGN_TRACK);
    p.valid=0U;p.dx_px=0.0f;plate.status=VISION_PLATE_SEARCH;
    for(i=0U;i<5U;i++) {
        p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)(i+8U);
        VisionAlign_StepTargets(&a,204300U+i*50U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);
        CHECK(i<2U?(f==0.0f && a.state==V_ALIGN_WAIT):(f==-c.search_speed && r==0.0f && a.state==V_ALIGN_SEARCH)); /* Three fresh SEARCH frames also restart left search after loss. */
    }
    ready(&a,&c,205000U);plate.valid=0U;plate.status=0U;
    for(i=0U;i<4U;i++) {
        p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)i;
        VisionAlign_StepTargets(&a,205250U+i*50U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);CHECK(f==0.0f);
    }
    plate.valid=1U;plate.status=VISION_PLATE_SEARCH;plate.station=3U;
    VisionAlign_StepTargets(&a,205500U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);CHECK(f==0.0f);
    plate.station=1U;c.search_limit_ms=100U;ready(&a,&c,206000U);
    for(i=0U;i<3U;i++) {
        p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)i;
        VisionAlign_StepTargets(&a,206250U+i*50U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);
    }
    VisionAlign_StepTargets(&a,206450U,&p,100U,NULL,0U,&plate,100U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && strstr(a.reason,"plate search budget"));
}
static void Test_EmptyStationTwo(void)
{
    vision_align_t a;vision_align_config_t c=config();vision_pixel_t p;
    vision_ring_count_t scene={0};float f,r;uint32_t i;
    c.mode=V_ALIGN_MODE_SMOOTH;c.station=2U;c.search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
    c.zero_ring_left=1U;c.search_speed=0.020f;c.max_speed=0.040f;c.max_drive_ms=10000U;
    /* Body left must be independent of the image-right gain sign. */
    c.forward_x=-0.0007f;
    CHECK(VisionAlign_ConfigValid(&c));
    ready(&a,&c,300000U);p=pixel(0.0f,0.0f,2U);p.valid=0U;
    scene.station=2U;scene.id=2U;scene.valid=1U;scene.observation_id=2U;scene.seq=p.seq;
    VisionAlign_StepScene(&a,300250U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_SEARCH && f==0.0f && r==-2.0f*c.search_speed);
    VisionAlign_StepScene(&a,300550U,&p,300U,&scene,300U,0U,&f,&r);
    CHECK(f==0.0f && r==0.0f); /* default fade zero is 160ms */
    VisionAlign_StepScene(&a,300650U,&p,400U,&scene,400U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
    for(i=0U;i<4U;i++) {
        p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(i+3U);
        scene.valid=(uint8_t)(i!=0U);scene.station=i==1U?3U:2U;
        if(i==2U)scene.seq++;
        VisionAlign_StepScene(&a,300700U+i*10U,&p,0U,i==3U?NULL:&scene,0U,1U,&f,&r);
        CHECK(a.state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
    }
    scene.valid=1U;scene.station=2U;scene.seq=p.seq;scene.count=1U;
    VisionAlign_StepScene(&a,300750U,&p,0U,&scene,0U,1U,&f,&r);CHECK(f==0.0f && r==0.0f);
    scene.count=2U;scene.observation_id++;p.observation_id++;scene.seq=p.seq=7U;
    VisionAlign_StepScene(&a,300800U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_SEARCH && f==-c.search_speed && r==0.0f);
    scene.count=3U;scene.observation_id++;p=pixel(20.0f,0.0f,10U);scene.seq=p.seq;
    VisionAlign_StepScene(&a,300850U,&p,0U,&scene,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && f<0.0f && a.search_ms==350U);
    scene.observation_id++;p=pixel(2.0f,0.0f,11U);scene.seq=p.seq;
    VisionAlign_StepScene(&a,300900U,&p,0U,&scene,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_SETTLE && f==0.0f && r==0.0f);
    /* Empty and two-ring episodes share one cumulative search budget. */
    c.search_limit_ms=100U;ready(&a,&c,301000U);scene.count=0U;p.valid=0U;
    VisionAlign_StepScene(&a,301250U,&p,0U,&scene,0U,1U,&f,&r);CHECK(f==0.0f && r==-2.0f*c.search_speed);
    scene.count=2U;scene.observation_id++;p.observation_id++;scene.seq=p.seq=12U;
    VisionAlign_StepScene(&a,301300U,&p,0U,&scene,0U,0U,&f,&r);CHECK(f<0.0f);
    VisionAlign_StepScene(&a,301350U,&p,50U,&scene,50U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && r==0.0f && a.search_ms==100U);
    /* A later lower vtune speed cap still bounds the doubled empty scan. */
    c.max_speed=0.030f;ready(&a,&c,301500U);scene.count=0U;
    VisionAlign_StepScene(&a,301750U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_SEARCH && f==0.0f && r==-c.max_speed);
    c.station=3U;CHECK(!VisionAlign_ConfigValid(&c));
    c.zero_ring_left=0U;ready(&a,&c,302000U);scene.station=3U;scene.count=0U;
    VisionAlign_StepScene(&a,302250U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
    c.station=2U;c.zero_ring_left=2U;CHECK(!VisionAlign_ConfigValid(&c));
    c.zero_ring_left=1U;c.type=VISION_TYPE_PLATE_PX;c.id=0U;CHECK(!VisionAlign_ConfigValid(&c));
}
static void Test_TwelveSecondMotionLimit(void)
{
    vision_align_config_t c=config();vision_align_t a;vision_pixel_t p;
    vision_ring_count_t scene={0};float f,r;uint32_t t;uint8_t trial;
    c.mode=V_ALIGN_MODE_SMOOTH;c.station=2U;c.search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
    c.zero_ring_left=1U;c.search_speed=0.020f;c.max_speed=0.040f;
    c.search_limit_ms=8000U;c.loss_ms=20000U;c.timeout_ms=90000U;
    c.observation_ms=400U;c.fade_start_ms=260U;c.fade_zero_ms=360U;
    scene.station=2U;scene.id=2U;scene.valid=1U;
    /* Field trace: 4155ms of searching shares the motion cap with PID.
     * At 10s total drive the target is still (2.8,-9.4), fresh and G3.
     * Compare the old cap with the new cap on identical observations. */
    for(trial=0U;trial<2U;trial++) {
        c.max_drive_ms=trial?V_ALIGN_STATION_DRIVE_LIMIT_MS:10000U;
        ready(&a,&c,0U);p=pixel(0.0f,0.0f,2U);p.valid=0U;scene.count=0U;
        scene.seq=p.seq;scene.observation_id=p.observation_id;
        VisionAlign_StepScene(&a,250U,&p,0U,&scene,0U,1U,&f,&r);
        CHECK(a.state==V_ALIGN_SEARCH && r==-0.040f);
        for(t=255U;t<=4405U;t+=5U) {
            p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
            if(t==4405U){scene.count=3U;p.valid=1U;p.dx_px=46.0f;p.dy_px=-181.3f;}
            VisionAlign_StepScene(&a,t,&p,0U,&scene,0U,0U,&f,&r);
        }
        CHECK(a.search_ms==4155U && a.state==V_ALIGN_TRACK);
        for(t=4455U;t<=10255U;t+=50U) {
            p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
            p.dx_px=2.8f;p.dy_px=-9.4f;
            VisionAlign_StepScene(&a,t,&p,0U,&scene,0U,0U,&f,&r);
        }
        CHECK(a.drive_ms==10005U);
        if(!trial) {
            CHECK(a.state==V_ALIGN_FAILED && !f && !r && strstr(a.reason,"motion budget"));
            continue;
        }
        CHECK(a.state==V_ALIGN_TRACK && r<0.0f);
        p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
        p.dx_px=2.0f;p.dy_px=-4.0f;
        VisionAlign_StepScene(&a,11005U,&p,0U,&scene,0U,0U,&f,&r);
        CHECK(a.state==V_ALIGN_SETTLE && !f && !r);
        VisionAlign_StepScene(&a,11015U,&p,10U,&scene,10U,1U,&f,&r);
        VisionAlign_StepScene(&a,11265U,&p,260U,&scene,260U,1U,&f,&r);
        for(t=11315U;t<=11515U;t+=50U) {
            p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
            VisionAlign_StepScene(&a,t,&p,0U,&scene,0U,1U,&f,&r);
        }
        CHECK(a.state==V_ALIGN_DONE && a.drive_ms<c.max_drive_ms);
    }
    /* The extra time never makes stale observations usable, and the 12s
     * limit still aborts even if every new G3 frame is otherwise healthy. */
    ready(&a,&c,20000U);p=pixel(2.8f,-9.4f,2U);scene.seq=p.seq;scene.observation_id=2U;
    VisionAlign_StepScene(&a,20250U,&p,0U,&scene,0U,1U,&f,&r);CHECK(r<0.0f);
    VisionAlign_StepScene(&a,20650U,&p,400U,&scene,400U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && !f && !r);
    ready(&a,&c,30000U);
    for(t=30250U;t<=42250U;t+=50U) {
        p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(p.observation_id%16U);
        VisionAlign_StepScene(&a,t,&p,0U,&scene,0U,t==30250U,&f,&r);
    }
    CHECK(a.state==V_ALIGN_FAILED && a.drive_ms==12000U && !f && !r && strstr(a.reason,"motion budget"));
}
static void Test_PlateDeadZoneAndFriction(void)
{
    vision_align_config_t c;vision_align_t a;vision_pixel_t p;float f,r;uint32_t t,received;
    CHECK(VisionAlign_StationConfig(&c,1U) && c.tolerance_px==21.0f && c.stable_frames==3U);
    ready(&a,&c,0U);p=pixel(21.1f,0.0f,2U);p.type=VISION_TYPE_PLATE_PX;p.id=0U;
    VisionAlign_Step(&a,250U,&p,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && fabsf(f-c.min_speed)<1e-6f && r==0.0f);
    VisionAlign_Step(&a,350U,&p,100U,0U,&f,&r);CHECK(fabsf(f-c.min_speed)<1e-6f && r==0.0f);
    VisionAlign_Step(&a,550U,&p,300U,0U,&f,&r);CHECK(f>0.0f && f<c.min_speed);
    VisionAlign_Step(&a,650U,&p,400U,1U,&f,&r);CHECK(f==0.0f && r==0.0f);
    p=pixel(-21.1f,0.0f,3U);p.type=VISION_TYPE_PLATE_PX;p.id=0U;
    VisionAlign_Step(&a,660U,&p,0U,1U,&f,&r);CHECK(fabsf(f+c.min_speed)<1e-6f && r==0.0f);
    ready(&a,&c,1000U);p=pixel(21.0f,-21.0f,2U);p.type=VISION_TYPE_PLATE_PX;p.id=0U;
    for(t=1250U;t<=1450U;t+=100U){
        p.observation_id++;VisionAlign_Step(&a,t,&p,0U,1U,&f,&r);CHECK(!f && !r);
        if(t<1450U){
            CHECK(a.state==V_ALIGN_WAIT && a.stable_count==(t-1250U)/100U+1U);
            VisionAlign_Step(&a,t+25U,&p,25U,1U,&f,&r);
            CHECK(a.state==V_ALIGN_WAIT && a.stable_count==(t-1250U)/100U+1U);
        }
    }
    CHECK(a.state==V_ALIGN_DONE && a.stable_count==3U); /* Three distinct stopped frames and200ms. */
    /* Simulated static friction refuses output below0.028;4Hz fresh frames. */
    ready(&a,&c,2000U);received=2250U;p=pixel(20.0f,-22.0f,2U);p.type=VISION_TYPE_PLATE_PX;p.id=0U;
    {float x=p.dx_px,y=p.dy_px;
      for(t=2250U;t<15000U && a.state!=V_ALIGN_DONE && a.state!=V_ALIGN_FAILED;t+=10U) {
        if(t%250U==0U){p.dx_px=x;p.dy_px=y;p.observation_id++;received=t;}
        VisionAlign_Step(&a,t,&p,t-received,1U,&f,&r);
        if(fabsf(f)+fabsf(r)>=0.028f){x-=f*1065.4f*0.01f;y-=r*1065.4f*0.01f;}
      }
      CHECK(a.state==V_ALIGN_DONE && fabsf(x)<=c.tolerance_px && fabsf(y)<=c.tolerance_px);
    }
    CHECK(VisionAlign_StationConfig(&c,2U) && c.tolerance_px==2.0f);
    c.search_axis=V_ALIGN_SEARCH_OFF;ready(&a,&c,16000U);p=pixel(6.0f,0.0f,2U);
    VisionAlign_Step(&a,16250U,&p,0U,1U,&f,&r);CHECK(f>0.0f && f<c.min_speed);
    /* Both approach sides use signed camera calibration, independent of a
     * body-right search setting. Test identity and rotated camera matrices. */
    {unsigned mode,i,direction,orientation;vision_plate_scene_t plate={0};
     for(orientation=0;orientation<2;orientation++)for(direction=0;direction<2;direction++)for(mode=0;mode<2;mode++){
      CHECK(VisionAlign_StationConfig(&c,1U));
      if(orientation){c.forward_x=0;c.right_x=-0.0014f;c.forward_y=0.0014f;c.right_y=0;}
      c.plate_search_right=(uint8_t)direction;
      c.search_axis=mode?V_ALIGN_SEARCH_BODY_RIGHT:V_ALIGN_SEARCH_IMAGE_RIGHT;
      CHECK(VisionAlign_ConfigValid(&c));ready(&a,&c,18000U);
      p=pixel(0,0,100U);p.type=VISION_TYPE_PLATE_PX;p.id=0;p.valid=0;
      plate.id=0;plate.station=1;plate.valid=1;plate.status=VISION_PLATE_SEARCH;
      for(i=0;i<3;i++){
       p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)(i+1U);
       VisionAlign_StepTargets(&a,18250U+i*50U,&p,0,NULL,0,&plate,0,1,&f,&r);
       CHECK(i<2?(!f && !r):(a.state==V_ALIGN_SEARCH &&
           fabsf(f-(orientation?0:direction?0.03f:-0.03f))<1e-6f &&
           fabsf(r-(orientation?(direction?-0.03f:0.03f):0))<1e-6f));
      }
      VisionAlign_StepTargets(&a,18750U,&p,400,NULL,0,&plate,400,1,&f,&r);
      CHECK(!f && !r && a.state==V_ALIGN_WAIT); /* no motion on stale scene */
     }
     c.plate_search_right=2U;CHECK(!VisionAlign_ConfigValid(&c));c.plate_search_right=0U;
     c.axes=2;c.forward_x=c.right_x=0;CHECK(!VisionAlign_ConfigValid(&c));
    }
}
static void Test_DoubleStationProfiles(void)
{
    vision_align_config_t old,new;vision_align_t a,b;vision_pixel_t p;
    vision_ring_count_t ring={0};vision_plate_scene_t plate={0};
    float f,r,of,or,scale;uint8_t station,i;uint32_t t,received;
    CHECK(!VisionAlign_StationConfig(NULL,1U));
    CHECK(!VisionAlign_StationConfig(&new,0U));
    for(station=1U;station<=3U;station++) {
        CHECK(VisionAlign_StationConfig(&new,station) && VisionAlign_ConfigValid(&new));
        scale=0.5f; /* V124 original for every station, relative to V125 doubled profile. */
        CHECK(fabsf(new.max_speed-0.160f*scale)<1e-6f && fabsf(new.search_speed-(station==1U?0.030f:0.040f))<1e-6f && fabsf(new.min_speed-0.060f*scale)<1e-6f);
        CHECK(new.stable_frames==(station==1U?3U:5U) && new.tolerance_px==(station==1U?21.0f:2.0f) && new.max_drive_ms==12000U);
        CHECK(new.observation_ms==400U && new.fade_start_ms==260U && new.fade_zero_ms==360U);
        old=new;old.max_speed=0.160f;old.search_speed=0.080f;old.min_speed=0.060f;
        old.search_axis=V_ALIGN_SEARCH_OFF; /* Baseline Step() has no scene packet; compare full-target XY only. */
        old.forward_x=old.right_y=0.0028f;old.forward_y=old.right_x=0.0f;
        ring.id=2U;ring.station=station;ring.count=3U;ring.valid=1U;ring.observation_id=2U;
        for(i=0U;i<2U;i++) {
            ready(&a,&old,0U);ready(&b,&new,0U);
            p=pixel(i?100.0f:30.0f,i?-80.0f:-20.0f,2U);p.type=new.type;p.id=new.id;ring.seq=p.seq;
            VisionAlign_Step(&a,250U,&p,0U,1U,&of,&or);
            VisionAlign_StepScene(&b,250U,&p,0U,&ring,0U,1U,&f,&r);
            CHECK(a.state==V_ALIGN_TRACK && b.state==V_ALIGN_TRACK);
            CHECK(fabsf(f-scale*of)<1e-6f && fabsf(r-scale*or)<1e-6f);
            CHECK(fabsf(f)+fabsf(r)<=new.max_speed+1e-6f);
        }
        if(station>1U) {
            ready(&b,&new,1000U);p.valid=0U;p.dx_px=p.dy_px=0.0f;ring.count=2U;
            VisionAlign_StepScene(&b,1250U,&p,0U,&ring,0U,1U,&f,&r);
            CHECK(b.state==V_ALIGN_SEARCH && f==0.040f && r==0.0f);
            if(station==2U) {
                ring.count=0U;ring.observation_id++;p.observation_id++;ring.seq=p.seq=3U;
                VisionAlign_StepScene(&b,1300U,&p,0U,&ring,0U,0U,&f,&r);
                CHECK(b.state==V_ALIGN_SEARCH && f==0.0f && r==-0.080f);
            }
            VisionAlign_StepScene(&b,1700U,&p,400U,&ring,400U,1U,&f,&r);
            CHECK(b.state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
        } else {
            ready(&b,&new,1000U);p.valid=0U;p.dx_px=p.dy_px=0.0f;
            plate.id=0U;plate.station=1U;plate.valid=1U;plate.status=VISION_PLATE_SEARCH;
            for(i=0U;i<3U;i++) {
                p.observation_id++;plate.observation_id++;plate.seq=p.seq=(uint8_t)(i+5U);
                VisionAlign_StepTargets(&b,1250U+i*50U,&p,0U,NULL,0U,&plate,0U,1U,&f,&r);
            }
            CHECK(b.state==V_ALIGN_SEARCH && f==-0.030f && r==0.0f);
        }
        /* Current station gains and caps converge with distinct 4Hz frames;
         * count3 must still reach the stopped five-frame verification. */
        ready(&b,&new,2000U);received=2250U;p=pixel(60.0f,-35.0f,2U);
        p.type=new.type;p.id=new.id;ring.count=3U;ring.seq=p.seq;ring.observation_id=2U;
        {float x=p.dx_px,y=p.dy_px;
          for(t=2250U;t<20000U && b.state!=V_ALIGN_DONE && b.state!=V_ALIGN_FAILED;t+=10U) {
            if(t%250U==0U) {p.dx_px=x;p.dy_px=y;p.observation_id++;ring.observation_id++;
              ring.seq=p.seq=(uint8_t)(p.observation_id%16U);received=t;}
            VisionAlign_StepScene(&b,t,&p,t-received,&ring,t-received,1U,&f,&r);
            x-=f*1065.4f*1.5f*0.01f;y-=r*1065.4f*1.5f*0.01f;
          }
          CHECK(b.state==V_ALIGN_DONE && fabsf(x)<=new.tolerance_px+0.5f && fabsf(y)<=new.tolerance_px+0.5f);
        }
    }
}
static void Test_RingTwoPixelTolerance(void)
{
    unsigned station,k;vision_align_config_t c;vision_align_t a;vision_pixel_t p;vision_ring_count_t scene={0};float f,r;
    for(station=2U;station<=3U;station++) {
        CHECK(VisionAlign_StationConfig(&c,(uint8_t)station) && c.tolerance_px==2.0f);
        scene.id=2U;scene.station=(uint8_t)station;scene.valid=1U;scene.count=3U;
        ready(&a,&c,0U);p=pixel(2.1f,0.0f,2U);
        scene.observation_id=p.observation_id;scene.seq=p.seq;
        VisionAlign_StepScene(&a,250U,&p,0U,&scene,0U,1U,&f,&r);
        CHECK(a.state==V_ALIGN_TRACK && f>0.0f && r==0.0f);
        ready(&a,&c,1000U);p=pixel(2.0f,-2.0f,2U);
        for(k=0U;k<5U;k++) {
            p.observation_id++;p.seq=(uint8_t)(p.observation_id%16U);scene.observation_id=p.observation_id;scene.seq=p.seq;
            VisionAlign_StepScene(&a,1250U+k*100U,&p,0U,&scene,0U,1U,&f,&r);
            CHECK(f==0.0f && r==0.0f);
        }
        CHECK(a.state==V_ALIGN_DONE && a.stable_count==5U);
    }
}
void Test_SmoothVisionAlignment(void)
{
    Test_RingTwoPixelTolerance();
    vision_align_t a;vision_align_config_t c=config();vision_pixel_t p;
    vision_ring_count_t scene={0};float f,r,initial,ix,dx;uint32_t t,i;
    c.mode=V_ALIGN_MODE_SMOOTH;c.max_drive_ms=10000U;
    ready(&a,&c,0U);p=pixel(60.0f,0.0f,2U);
    VisionAlign_Step(&a,250U,&p,0U,1U,&f,&r);initial=f;
    CHECK(a.state==V_ALIGN_TRACK && f>0.0f && fabsf(f)+fabsf(r)<=c.max_speed);
    VisionAlign_Step(&a,350U,&p,100U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && f==initial);
    VisionAlign_Step(&a,390U,&p,140U,0U,&f,&r);CHECK(f>0.0f && f<initial);
    VisionAlign_Step(&a,410U,&p,160U,0U,&f,&r);CHECK(f==0.0f && r==0.0f);
    VisionAlign_Step(&a,450U,&p,200U,0U,&f,&r);CHECK(a.state==V_ALIGN_WAIT && f==0.0f);
    p=pixel(6.0f,0.0f,3U);VisionAlign_Step(&a,451U,&p,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && f>0.0f && f<c.min_speed);
    p=pixel(4.0f,0.0f,4U);VisionAlign_Step(&a,500U,&p,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_SETTLE && f==0.0f);
    VisionAlign_Step(&a,510U,&p,10U,1U,&f,&r);
    VisionAlign_Step(&a,760U,&p,260U,1U,&f,&r);
    for(i=0U;i<5U;i++){p.observation_id++;VisionAlign_Step(&a,800U+i*100U,&p,0U,1U,&f,&r);}
    CHECK(a.state==V_ALIGN_DONE && f==0.0f && r==0.0f);
    c.pid_i=0.5f;c.pid_d=0.02f;c.integral_limit_px=2.0f;ready(&a,&c,2000U);
    p=pixel(100.0f,0.0f,2U);VisionAlign_Step(&a,2250U,&p,0U,1U,&f,&r);
    p.observation_id++;VisionAlign_Step(&a,2350U,&p,0U,0U,&f,&r);
    CHECK(a.integral_x==0.0f);
    p=pixel(10.0f,0.0f,4U);VisionAlign_Step(&a,2450U,&p,0U,0U,&f,&r);
    ix=a.integral_x;dx=a.derivative_x;CHECK(ix>0.0f && ix<=2.0f && dx<0.0f);
    VisionAlign_Step(&a,2500U,&p,50U,0U,&f,&r);CHECK(a.integral_x==ix && a.derivative_x==dx);
    p.valid=0U;VisionAlign_Step(&a,2510U,&p,60U,0U,&f,&r);
    CHECK(f==0.0f && r==0.0f && a.integral_x==0.0f && !a.pid_ready);
    c=config();c.mode=V_ALIGN_MODE_SMOOTH;c.station=2U;c.search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
    c.max_drive_ms=10000U;ready(&a,&c,3000U);
    p=pixel(0.0f,0.0f,2U);p.valid=0U;p.seq=7U;
    scene.id=2U;scene.valid=1U;scene.count=2U;scene.station=2U;scene.seq=7U;scene.observation_id=1U;
    VisionAlign_StepScene(&a,3250U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_SEARCH && f==c.min_speed && r==0.0f);
    scene.count=3U;scene.seq=8U;scene.observation_id++;p.seq=8U;p.observation_id++;
    VisionAlign_StepScene(&a,3300U,&p,0U,&scene,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && f==0.0f && r==0.0f);
    scene.observation_id++;p.observation_id++;scene.seq=p.seq=9U;
    VisionAlign_StepScene(&a,3350U,&p,0U,&scene,0U,1U,&f,&r);CHECK(f==0.0f && r==0.0f);
    p.valid=1U;p.dx_px=30.0f;p.observation_id++;scene.observation_id++;scene.seq=p.seq=10U;
    VisionAlign_StepScene(&a,3400U,&p,0U,&scene,0U,1U,&f,&r);CHECK(a.state==V_ALIGN_TRACK && f>0.0f);
    scene.count=1U;scene.observation_id++;scene.seq=p.seq=11U;p.observation_id++;
    VisionAlign_StepScene(&a,3450U,&p,0U,&scene,0U,0U,&f,&r);CHECK(f==0.0f && r==0.0f);
    c.search_limit_ms=100U;ready(&a,&c,4000U);scene.count=2U;p.valid=0U;scene.seq=p.seq=1U;
    VisionAlign_StepScene(&a,4250U,&p,0U,&scene,0U,1U,&f,&r);
    VisionAlign_StepScene(&a,4350U,&p,100U,&scene,100U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && strstr(a.reason,"search budget"));
    c.search_limit_ms=3000U;ready(&a,&c,5000U);scene.station=3U;
    VisionAlign_StepScene(&a,5250U,&p,0U,&scene,0U,1U,&f,&r);CHECK(f==0.0f && r==0.0f);
    /* Field profile: fresh two-ring search can exceed the old 3s limit.
     * Count3 at the 8s boundary stops search before evaluating its limit;
     * a later new two-ring episode cannot reset the cumulative budget. */
    c.search_limit_ms=8000U;c.loss_ms=20000U;ready(&a,&c,80000U);
    scene.station=2U;scene.count=2U;scene.valid=1U;p.valid=0U;
    for(i=0U;i<160U;i++) {
        p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(i%16U);
        VisionAlign_StepScene(&a,80250U+i*50U,&p,0U,&scene,0U,i==0U,&f,&r);
        CHECK(a.state==V_ALIGN_SEARCH && f>0.0f && r==0.0f);
    }
    scene.count=3U;p.observation_id++;scene.observation_id++;scene.seq=p.seq=0U;
    VisionAlign_StepScene(&a,88250U,&p,0U,&scene,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_WAIT && a.search_ms==8000U && f==0.0f && r==0.0f);
    p.valid=1U;p.dx_px=30.0f;p.observation_id++;scene.observation_id++;scene.seq=p.seq=1U;
    VisionAlign_StepScene(&a,88300U,&p,0U,&scene,0U,1U,&f,&r);
    CHECK(a.state==V_ALIGN_TRACK && f>0.0f && a.search_ms==8000U);
    p.valid=0U;scene.count=2U;p.observation_id++;scene.observation_id++;scene.seq=p.seq=2U;
    VisionAlign_StepScene(&a,88350U,&p,0U,&scene,0U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && r==0.0f && strstr(a.reason,"search budget"));
    ready(&a,&c,90000U);
    for(i=0U;i<160U;i++) {
        p.observation_id++;scene.observation_id++;scene.seq=p.seq=(uint8_t)(i%16U);
        VisionAlign_StepScene(&a,90250U+i*50U,&p,0U,&scene,0U,i==0U,&f,&r);
    }
    VisionAlign_StepScene(&a,98250U,&p,50U,&scene,50U,0U,&f,&r);
    CHECK(a.state==V_ALIGN_FAILED && f==0.0f && r==0.0f && a.search_ms==8000U);
    c=config();c.mode=V_ALIGN_MODE_SMOOTH;c.max_drive_ms=10000U;ready(&a,&c,6000U);
    {
        float x=60.0f,y=-35.0f,oldx=x,oldy=y;uint32_t received=6250U;
        p=pixel(x,y,2U);
        for(t=6250U;t<35000U && a.state!=V_ALIGN_DONE && a.state!=V_ALIGN_FAILED;t+=10U) {
            if(t%100U==0U){p=pixel(oldx,oldy,p.observation_id+1U);oldx=x;oldy=y;received=t;}
            VisionAlign_Step(&a,t,&p,t-received,1U,&f,&r);
            x-=f*1065.4f*1.5f*0.01f;y-=r*1065.4f*1.5f*0.01f;
            CHECK(fabsf(f)+fabsf(r)<=c.max_speed+1e-6f);
        }
        CHECK(a.state==V_ALIGN_DONE && fabsf(x)<7.0f && fabsf(y)<7.0f);
    }
    Test_StoppedSparseVerification();
    Test_SearchAndCadenceProfiles();
    Test_EmptyStationTwo();
    Test_TwelveSecondMotionLimit();
    Test_DoubleStationProfiles();
    Test_PlateDeadZoneAndFriction();
}
