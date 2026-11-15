#include "nav_odometry.h"
#include "nav_frame.h"
#include "nav_feedback.h"
#include <math.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern int16_t ChassisMotor_RampStepRpm(int16_t current,int16_t target,float dt,float decel);
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;} }while(0)
static const double signs[4]={1.0,-1.0,1.0,-1.0};
static const int32_t base[4]={50000,-10000,2345,-999};
static const float mmrev=255.694226f;
volatile uint32_t test_exit_old_ms[3],test_exit_fast_ms[3];
uint32_t test_approach4_ms[3],test_approach8_ms[3];
static uint8_t start(nav_odometry_t *o,float distance,float speed,int8_t f,int8_t r,uint32_t now)
{
    float expected[4];NavFrame_VelocityWheelValues(f,r,expected);
    return NavOdometry_Start(o,distance,speed,65536.0f,mmrev,250.0f,292.0f,base,15U,signs,expected,now);
}
static void positions(const nav_odometry_t *o,float distance,int32_t p[4])
{
    uint8_t i;for(i=0U;i<4U;i++)p[i]=base[i]+(int32_t)lroundf(distance*65536.0f/mmrev*o->expected[i]/(float)signs[i]);
}
void Test_NavOdometrySamples(void)
{
    nav_odometry_t o;int32_t p[4];uint32_t t;
    /* Uploaded25mm offset with6.5deg yaw, both diagonal departures, and
     * a45deg translation where two wheels need not translate at all. */
    {
      const float direction[5][2]={{1,0},{-125.0f/325,300.0f/325},
          {-125.0f/325,-300.0f/325},{0.70710678f,0.70710678f},{0,-1}};
      for(unsigned n=0;n<5;n++) {
        double counts[4]={0};float initial=n==2?-1.0f:1.0f,prev=6.5f;
        float distance=n==0?25.0f:325.0f;
        CHECK(start(&o,distance,.75f,1,0,0));
        CHECK(NavOdometry_SetMapDirection(&o,0,initial,direction[n][0],direction[n][1],prev));
        for(t=40;t<=400;t+=40) {
          float yaw=6.5f-2.0f*t/400,mid=(prev+yaw)*.5f,f,r,w[4];
          NavFrame_MapToBody(0,initial,mid,direction[n][0]*distance/10,
              direction[n][1]*distance/10,&f,&r);NavFrame_WheelValues(f,r,w);
          /* Opposite side yaw deltas must not become map translation. */
          for(unsigned i=0;i<4;i++) {
            counts[i]+=(w[i]+((i==0 || i==2)?1.0f:-1.0f))*65536.0/mmrev/signs[i];
            p[i]=base[i]+(int32_t)lround(counts[i]);
          }
          NavOdometry_ObserveHeading(&o,p,15U,yaw,t);prev=yaw;
          CHECK(o.state!=NAV_ODO_FAILED && fabsf(o.progress_mm-distance*t/400)<.04f);
        }
        CHECK(fabsf(o.map_dx-distance*direction[n][0])<.04f);
        CHECK(fabsf(o.map_dy-distance*direction[n][1])<.04f && o.state==NAV_ODO_BRAKE);
        NavOdometry_ObserveHeading(&o,p,15U,prev,520);(void)NavOdometry_Step(&o,520,1U);
        CHECK(o.state==NAV_ODO_DONE);
      }
    }
    CHECK(start(&o,100.0f,.3f,1,0,0));
    o.rolling_tail_mm=600.0f;positions(&o,100.0f,p);
    NavOdometry_Observe(&o,p,15U,100);CHECK(o.state==NAV_ODO_RUN);
    CHECK(NavOdometry_Step(&o,100,0)>.2f); /* No virtual lane-entry braking. */
    o.rolling_tail_mm=0;NavOdometry_Observe(&o,p,15U,140);CHECK(o.state==NAV_ODO_BRAKE);
    NavOdometry_Cancel(&o);CHECK(o.rolling_tail_mm==0 && o.state==NAV_ODO_IDLE);
    CHECK(start(&o,100.0f,.3f,1,0,0));
    /* Unequal wheel samples exceed the former30mm limit; preserve actual
     * spread/median progress, then finish with fresh stopped samples. */
    for(t=40;t<=400;t+=40) {
        positions(&o,(float)t*.25f,p);p[0]+=10000;p[1]-=10000;
        NavOdometry_Observe(&o,p,15U,t);
        CHECK(o.state!=NAV_ODO_FAILED && !o.reason && o.spread_mm>30.0f);
        CHECK(fabsf(o.progress_mm-((float)t*.25f+19.508f))<.02f);
    }
    CHECK(o.state==NAV_ODO_BRAKE);
    NavOdometry_Observe(&o,p,15U,520);(void)NavOdometry_Step(&o,520,1U);
    CHECK(o.state==NAV_ODO_DONE && o.spread_mm>30.0f && !o.reason);
    /* A single large wheel disagreement is not a median jump. */
    CHECK(start(&o,100.0f,.3f,1,0,0));positions(&o,20.0f,p);p[0]+=100000;
    NavOdometry_Observe(&o,p,15U,40);
    CHECK(o.state==NAV_ODO_RUN && o.spread_mm>300.0f && fabsf(o.progress_mm-20.0f)<.01f);
    CHECK(start(&o,100.0f,.3f,1,0,0));positions(&o,-10.0f,p);
    NavOdometry_Observe(&o,p,15U,40);CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"reverse"));
    CHECK(start(&o,100.0f,.3f,1,0,0));positions(&o,300.0f,p);
    NavOdometry_Observe(&o,p,15U,40);CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"jump"));
    CHECK(start(&o,100.0f,.3f,1,0,0));positions(&o,10.0f,p);
    NavOdometry_Observe(&o,p,7U,40);CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"four fresh"));
    CHECK(start(&o,100.0f,.3f,1,0,0));(void)NavOdometry_Step(&o,200,0U);
    CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"stale"));
}
static uint32_t travel(float distance,float speed,float efficiency,uint8_t axis,uint32_t epoch,float gain)
{
    nav_odometry_t o;int32_t p[4];float actual=0.0f,command;int16_t rpm=0;uint32_t t;
    int8_t f=axis==0U?1:(axis==1U?-1:0),r=axis==2U?1:(axis==3U?-1:0);
    CHECK(start(&o,distance,speed,f,r,epoch));
    o.approach_gain=gain;
    for(t=20U;t<80000U && o.state!=NAV_ODO_DONE && o.state!=NAV_ODO_FAILED;t+=20U) {
        command=NavOdometry_Step(&o,epoch+t,(uint8_t)(rpm==0));
        CHECK(command>=0.0f && command<=speed);
        rpm=ChassisMotor_RampStepRpm(rpm,(int16_t)lroundf(command*250.0f),0.02f,292.0f);
        actual+=rpm*mmrev/60.0f*0.02f*efficiency;
        if(t%40U==0U){positions(&o,actual,p);NavOdometry_Observe(&o,p,15U,epoch+t);}
    }
    CHECK(o.state==NAV_ODO_DONE && rpm==0);
    CHECK(fabsf(actual-distance)<=3.0f && fabsf(o.progress_mm-actual)<0.05f);
    if(efficiency<0.8f && distance>=750.0f)CHECK(t>distance/(speed*250.0f*mmrev/60.0f)*1000.0f);
    return t;
}
void Test_NavOdometry(void)
{
    Test_NavOdometrySamples();
    static const float distances[8]={20.0f,30.0f,50.0f,125.0f,300.0f,750.0f,1650.0f,1775.0f};
    static const float efficiencies[3]={0.55f,0.80f,1.05f};
    uint8_t d,e,axis;nav_odometry_t o;int32_t p[4];uint32_t t;
    for(d=0U;d<8U;d++)for(e=0U;e<3U;e++)for(axis=0U;axis<4U;axis++) {
        (void)travel(distances[d],d<3U?0.10f:0.30f,efficiencies[e],axis,0U,NAV_ODOMETRY_APPROACH_GAIN);
        (void)travel(distances[d],0.35f,efficiencies[e],axis,UINT32_MAX-1000U,NAV_ODOMETRY_APPROACH_GAIN);
    }
    for(e=0U;e<3U;e++)for(axis=0U;axis<4U;axis++) {
        uint32_t old_time=travel(300.0f,0.30f,efficiencies[e],axis,0U,NAV_ODOMETRY_APPROACH_GAIN);
        old_time+=travel(125.0f,0.30f,efficiencies[e],axis,0U,NAV_ODOMETRY_APPROACH_GAIN)+700U;
        t=travel(300.0f,0.35f,efficiencies[e],axis,0U,NAV_ODOMETRY_EXIT_APPROACH_GAIN);
        t+=travel(125.0f,0.35f,efficiencies[e],axis,0U,NAV_ODOMETRY_EXIT_APPROACH_GAIN);
        CHECK(t<old_time);
        if(!axis){test_exit_old_ms[e]=old_time;test_exit_fast_ms[e]=t;}
    }
    /* Current competition speed0.5: both departure legs retain the normal
     * terminal approach, with four directions and wheel efficiencies. */
    CHECK(NAV_ODOMETRY_EXIT_APPROACH_GAIN==NAV_ODOMETRY_APPROACH_GAIN);
    for(e=0U;e<3U;e++)for(axis=0U;axis<4U;axis++) {
        (void)travel(300.0f,0.5f,efficiencies[e],axis,0U,NAV_ODOMETRY_EXIT_APPROACH_GAIN);
        (void)travel(125.0f,0.5f,efficiencies[e],axis,0U,NAV_ODOMETRY_EXIT_APPROACH_GAIN);
    }
    /* Same ramp/speed/distance: the changed approach coefficient shortens
     * the tail while preserving true encoder arrival and final stopped output. */
    {const float distance[3]={20,50,750},speed[3]={.1f,.1f,.8f};
     for(d=0;d<3;d++) {
       test_approach4_ms[d]=travel(distance[d],speed[d],.8f,0U,0U,4.0f);
       test_approach8_ms[d]=travel(distance[d],speed[d],.8f,0U,0U,NAV_ODOMETRY_APPROACH_GAIN);
       CHECK(test_approach8_ms[d]<test_approach4_ms[d]);
     }}
    CHECK(start(&o,100.0f,0.30f,1,0,0U));
    CHECK(NavOdometry_Step(&o,200U,0U)==0.0f && o.state==NAV_ODO_FAILED);
    CHECK(start(&o,100.0f,0.30f,1,0,0U));positions(&o,0.0f,p);NavOdometry_Observe(&o,p,7U,40U);
    CHECK(o.state==NAV_ODO_FAILED);
    CHECK(start(&o,100.0f,0.30f,1,0,0U));positions(&o,-6.0f,p);NavOdometry_Observe(&o,p,15U,40U);
    CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"reverse"));
    CHECK(start(&o,100.0f,0.30f,1,0,0U));positions(&o,300.0f,p);NavOdometry_Observe(&o,p,15U,40U);
    CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"jump"));
    CHECK(start(&o,100.0f,0.30f,1,0,0U));positions(&o,0.0f,p);
    for(t=40U;t<=1520U;t+=40U){NavOdometry_Observe(&o,p,15U,t);(void)NavOdometry_Step(&o,t,0U);}
    CHECK(o.state==NAV_ODO_FAILED && strstr(o.reason,"no progress"));
    CHECK(start(&o,100.0f,0.30f,1,0,0U));positions(&o,0.0f,p);p[0]+=10000;p[1]-=10000;
    NavOdometry_Observe(&o,p,15U,40U);CHECK(o.state==NAV_ODO_RUN && !o.reason && o.spread_mm>30.0f);
    CHECK(start(&o,20.0f,0.30f,1,0,0U));positions(&o,19.0f,p);NavOdometry_Observe(&o,p,15U,40U);
    CHECK(o.state==NAV_ODO_BRAKE);
    positions(&o,67.2f,p);NavOdometry_Observe(&o,p,15U,80U);
    NavOdometry_Observe(&o,p,15U,120U);NavOdometry_Observe(&o,p,15U,200U);
    (void)NavOdometry_Step(&o,200U,1U);CHECK(o.state==NAV_ODO_DONE && !o.reason && fabsf(o.progress_mm-67.2f)<0.02f);
    /* Large residuals are accepted after real stopped/fresh feedback, never
     * by replacing encoder progress with the target. Replay both signs of residual. */
    {const float measured[]={1796.1f,2200.0f,1200.0f};unsigned n;
     for(n=0;n<3;n++) {
       CHECK(start(&o,1748.9f,0.65f,1,0,0U));
       for(t=40;t<=2000;t+=40) {
         float at=fminf(measured[n],t*.6f);positions(&o,at,p);NavOdometry_Observe(&o,p,15U,t);
       }
       /* A BRAKE owner has already issued STOP; test late, arbitrary residual. */
       o.state=NAV_ODO_BRAKE;o.brake_tick=2000;
       for(t=2040;t<=4000 && o.progress_mm<measured[n]-.01f;t+=40) {
         positions(&o,fminf(measured[n],o.progress_mm+30.0f),p);NavOdometry_Observe(&o,p,15U,t);
       }
       positions(&o,measured[n],p);NavOdometry_Observe(&o,p,15U,t);
       NavOdometry_Observe(&o,p,15U,t+120);(void)NavOdometry_Step(&o,t+120,1U);
       CHECK(o.state==NAV_ODO_DONE && !o.reason && fabsf(o.progress_mm-measured[n])<0.02f);
       CHECK(NavFeedback_CheckOdometry(1748.9f,o.progress_mm)==NAV_MOTION_OK);
     }}
    CHECK(NavFeedback_CheckOdometry(100.0f,NAN)==NAV_MOTION_INVALID);
    CHECK(NavFeedback_CheckOdometry(100.0f,-6.0f)==NAV_MOTION_REVERSED);
    CHECK(start(&o,20.0f,0.30f,1,0,0U));NavOdometry_Cancel(&o);
    CHECK(NavOdometry_Step(&o,40U,0U)==0.0f && o.state==NAV_ODO_IDLE);
    CHECK(!start(&o,100.0f,NAN,1,0,0U));
    {float expected[4]={1,1,1,1};CHECK(!NavOdometry_Start(&o,100.0f,0.3f,0.0f,mmrev,250,292,base,15U,signs,expected,0U));}
}
