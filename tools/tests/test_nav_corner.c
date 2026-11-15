#include "nav_corner.h"
#include "nav_frame.h"
#include "nav_blend.h"
#include "grid_nav.h"
#include "chassis_field.h"
#include "chassis_motor.h"
#include <math.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);}}while(0)
volatile float test_corner_metrics[9][8];
volatile float test_corner_exit_overshoot[9];
static uint8_t free_body(float x,float y,float yaw)
{
    static const int dx[5]={0,2,-2,0,0},dy[5]={0,0,0,2,-2};unsigned i;
    for(i=0;i<5;i++) {
      if(x+dx[i]<0 || y+dy[i]<0 || x+dx[i]>2400 || y+dy[i]>2400 ||
         !GridNav_RotatedBodyFreeMm((uint16_t)lroundf(x+dx[i]),(uint16_t)lroundf(y+dy[i]),
           (int16_t)lroundf(NavBlend_AngleError(yaw,0))))return 0;
    }
    return 1;
}
static uint8_t never_free(float x,float y,float yaw){(void)x;(void)y;(void)yaw;return 0;}
static const float cases[9][6]={
    {1150,400,350,400,350,1200},{350,1200,350,400,1150,400},
    {350,1280,350,2100,1250,2100},{1250,2100,350,2100,350,1280},
    {2125,1150,2125,2100,1250,2100},{1250,2100,2125,2100,2125,1150},
    {1250,350,2125,350,2125,1150},{2125,1150,2125,350,1250,350},
    {380,1200,2125,1200,2125,350}}; /* East-side QR return join. */
static uint8_t init_case(nav_corner_t *p,unsigned n,float initial_y)
{
    const float *a=cases[n];float norm=hypotf(a[2]-a[0],a[3]-a[1]);
    float ux=(a[2]-a[0])/norm,uy=(a[3]-a[1])/norm;
    float heading=atan2f(uy,ux)*57.29578f-atan2f(initial_y,0)*57.29578f;
    return NavCorner_Init(p,0,initial_y,a[0],a[1],a[2],a[3],a[4],a[5],ux,uy,NAV_CORNER_DEFAULT_RADIUS_MM,heading+720.0f);
}
static void plant(unsigned n)
{
    nav_corner_t p;nav_corner_control_t c={532.7f,NAV_CORNER_DEFAULT_SPEED_MM_S,498,NAV_CORNER_LATERAL_MM_S2,NAV_CORNER_MAX_YAW_DPS,900.0f*1065.4f/250.0f/300.0f*57.29578f};chassis_field_profile_t field={0};
    float wheel_speed=1065.4f,spin=300.0f,yaw,gyro=0,vx,vy,o,elapsed=0,peak=0,minimum=1e9f,overshoot=0;
    float accw[4]={0},dx=0,dy=0,truth_x,truth_y;unsigned step,i,stable=0;
    float initial_y=n%2?-1.0f:1.0f;
    CHECK(init_case(&p,n,initial_y));yaw=p.last_yaw;truth_x=p.x;truth_y=p.y;
    for(step=0;step<7500 && stable<40;step++) {
      float df,dr,physical_o,actual_yaw,old_yaw=yaw;
      if(step%5==0)NavCorner_Command(&p,&c,yaw,gyro,&vx,&vy,&o);
      ChassisField_Step(&field,vx/wheel_speed,vy/wheel_speed,o*spin/wheel_speed,
        0,initial_y,yaw,0.004f,250,600,292,900,CHASSIS_FIELD_MAX_WHEEL_RPM);
      for(i=0;i<4;i++){CHECK(fabsf(field.wheel[i])*250.0f<=CHASSIS_FIELD_MAX_WHEEL_RPM+0.01f);accw[i]+=field.wheel[i]*wheel_speed*0.004f;}
      df=(field.wheel[0]+field.wheel[1]+field.wheel[2]+field.wheel[3])*0.25f*wheel_speed;
      dr=(field.wheel[0]-field.wheel[1]-field.wheel[2]+field.wheel[3])*0.25f*wheel_speed;
      physical_o=(-field.wheel[0]+field.wheel[1]-field.wheel[2]+field.wheel[3])*0.25f*wheel_speed/spin;
      gyro=physical_o*57.29578f*(n%3==0?0.85f:1.0f);yaw+=gyro*0.004f;
      if(p.progress>=p.in_mm) {
        float end_yaw=atan2f(p.wy,p.wx)*57.29578f;
        float world=atan2f(initial_y,0)*57.29578f+yaw;
        overshoot=fmaxf(overshoot,-p.turn*NavBlend_AngleError(end_yaw,world));
      }
      actual_yaw=old_yaw+0.5f*NavBlend_AngleError(yaw,old_yaw);
      NavFrame_BodyToMap(0,initial_y,actual_yaw,df*0.004f,dr*0.004f,&dx,&dy);
      truth_x+=dx;truth_y+=dy;
      if(step%20==19){NavCorner_Observe(&p,accw,yaw,0.08f);memset(accw,0,sizeof(accw));
        CHECK(free_body(truth_x,truth_y,atan2f(initial_y,0)*57.29578f+yaw));}
      if(p.progress>p.in_mm+15 && p.progress<p.in_mm+p.arc_mm-15)minimum=fminf(minimum,hypotf(df,dr));
      peak=fmaxf(peak,p.cross_mm);
      if(NavCorner_Ready(&p,yaw,gyro) && hypotf(df,dr)<2.0f && fabsf(gyro)<0.5f)stable++;else stable=0;
      elapsed+=0.004f;
    }
    test_corner_metrics[n][0]=elapsed;test_corner_metrics[n][1]=p.cross_mm;test_corner_metrics[n][2]=peak;
    test_corner_metrics[n][3]=hypotf(p.x-p.end_x,p.y-p.end_y);
    test_corner_metrics[n][4]=minimum;test_corner_metrics[n][5]=p.progress;
    test_corner_metrics[n][6]=hypotf(p.x-truth_x,p.y-truth_y);test_corner_metrics[n][7]=stable;
    test_corner_exit_overshoot[n]=overshoot;
    CHECK(stable==40 && elapsed<30.0f && p.cross_mm<=8.0f && peak<20.0f);
    CHECK(hypotf(p.x-p.end_x,p.y-p.end_y)<=3.0f && minimum>30.0f); /* No corner stop. */
    CHECK(hypotf(p.x-truth_x,p.y-truth_y)<5.0f);
    CHECK(p.exit_heading_locked && overshoot<=5.0f);
}
static void exit_heading_cases(void)
{
    unsigned n,j;
    nav_corner_control_t c={532.7f,NAV_CORNER_DEFAULT_SPEED_MM_S,498,NAV_CORNER_LATERAL_MM_S2,NAV_CORNER_MAX_YAW_DPS,
        900.0f*1065.4f/250.0f/300.0f*57.29578f};
    for(n=0;n<9;n++)for(j=0;j<2;j++) {
        nav_corner_t p;float frame=j?-1.0f:1.0f,goal,desired,k,vx,vy,slow,fast,saved_x,saved_y,saved_progress;
        CHECK(init_case(&p,n,frame));
        goal=atan2f(p.wy,p.wx)*57.29578f-atan2f(frame,0)*57.29578f+720.0f;
        /* Early heading noise before the curve must not skip the whole bend. */
        p.progress=p.in_mm-20.0f;NavCorner_Command(&p,&c,goal,0,&vx,&vy,&fast);
        CHECK(!p.exit_heading_locked);
        p.progress=p.in_mm+p.arc_mm*0.95f;
        CHECK(NavCorner_Pose(&p,p.progress,&p.x,&p.y,&desired,&k));
        NavCorner_Command(&p,&c,goal-p.turn*8.0f,0,&vx,&vy,&slow);
        NavCorner_Command(&p,&c,goal-p.turn*8.0f,p.turn*100.0f,&vx,&vy,&fast);
        CHECK(!p.exit_heading_locked && p.turn*slow>0.0f && p.turn*fast<=0.0f);
        CHECK(hypotf(vx,vy)>30.0f); /* Braking yaw never requests a corner stop. */
        saved_x=p.x;saved_y=p.y;saved_progress=p.progress;
        NavCorner_Command(&p,&c,goal,p.turn*80.0f,&vx,&vy,&fast);
        CHECK(p.exit_heading_locked && p.turn*fast<0.0f); /* Brake residual rotation. */
        CHECK(p.x==saved_x && p.y==saved_y && p.progress==saved_progress);
        CHECK(vx*p.wx+vy*p.wy>30.0f && !NavCorner_Ready(&p,goal,0));
        /* Wrap and a small rebound retain heading hold, never arc feedforward. */
        NavCorner_Command(&p,&c,goal-p.turn*3.0f-1080.0f,0,&vx,&vy,&fast);
        CHECK(p.exit_heading_locked && fabsf(fast)<0.15f);
        CHECK(p.x==saved_x && p.y==saved_y && p.progress==saved_progress);
        CHECK(init_case(&p,n,frame) && !p.exit_heading_locked);
    }
}
void Test_NavCorner(void)
{
    nav_corner_t p;float x,y,yaw,k,x0,y0,h0,k0;unsigned n,j;
    GridNav_Init();
    exit_heading_cases();
    /* Actual command on the arc midpoint must retain the requested1.2x,
     * after lateral and yaw limits, not merely change the nominal cap. */
    CHECK(init_case(&p,0,1));p.progress=p.in_mm+p.arc_mm*0.5f;
    CHECK(NavCorner_Pose(&p,p.progress,&p.x,&p.y,&yaw,&k));
    {nav_corner_control_t c={532.7f,NAV_CORNER_DEFAULT_SPEED_MM_S,498,NAV_CORNER_LATERAL_MM_S2,NAV_CORNER_MAX_YAW_DPS,900.0f*1065.4f/250.0f/300.0f*57.29578f};float vx,vy,omega;
     NavCorner_Command(&p,&c,yaw-90.0f,0,&vx,&vy,&omega);
     CHECK(fabsf(hypotf(vx,vy)-255.696f*1.2f)<0.01f);
     CHECK(fabsf(omega-p.turn*306.8352f/NAV_CORNER_DEFAULT_RADIUS_MM)<0.001f);
     NavCorner_Command(&p,&c,yaw-90.0f-p.turn*120.0f,0,&vx,&vy,&omega);
     CHECK(fabsf(omega*57.29578f-p.turn*300.0f)<0.01f); /* New angular ceiling is effective. */
    }
    CHECK(!NavCorner_Init(&p,0,1,1200,1200,1000,1200,1000,2000,-1,0,150,90)); /* insufficient incoming leg */
    CHECK(NavCorner_Init(&p,0,1,1500,1200,1200,1200,1200,1500,-1,0,NAV_CORNER_DEFAULT_RADIUS_MM,90));
    CHECK(NavCorner_Clear(&p,free_body)); /* Interior junction can be rounded when its full footprint fits. */
    CHECK(!NavCorner_Init(&p,0,1,600,400,350,400,350,600,-1,0,150,90)); /* insufficient outgoing leg */
    CHECK(!NavCorner_Init(&p,0,1,1150,400,350,400,350,1200,-1,0,50,90));
    CHECK(!NavCorner_Init(&p,0,1,1150,420,350,400,350,1200,-1,0,150,90));
    CHECK(!NavCorner_Init(&p,0,1,1150,400,350,400,350,1200,-1,0,150,NAN));
    for(n=0;n<9;n++) {
      CHECK(init_case(&p,n,1));CHECK(NavCorner_Clear(&p,free_body));CHECK(!NavCorner_Clear(&p,never_free));
      CHECK(NavCorner_Pose(&p,p.in_mm,&x,&y,&yaw,&k));
      CHECK(hypotf(x-(p.corner_x-p.radius*p.ux),y-(p.corner_y-p.radius*p.uy))<0.001f);
      CHECK(NavCorner_Pose(&p,p.in_mm+p.arc_mm,&x,&y,&yaw,&k));
      CHECK(hypotf(x-(p.corner_x+p.radius*p.wx),y-(p.corner_y+p.radius*p.wy))<0.001f);
      for(j=0;j<2;j++) {
        float s=j?p.in_mm+p.arc_mm:p.in_mm;
        CHECK(NavCorner_Pose(&p,s-0.001f,&x0,&y0,&h0,&k0));
        CHECK(NavCorner_Pose(&p,s+0.001f,&x,&y,&yaw,&k));
        CHECK(hypotf(x-x0,y-y0)<0.01f && fabsf(NavBlend_AngleError(yaw,h0))<0.01f);
      }
      plant(n);
    }
}
