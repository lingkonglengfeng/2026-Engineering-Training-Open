#include "vision_obstacle.h"
#include "grid_nav.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(c) do{checks++;if(!(c)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<0.05f)
void ZDT_Console_Print(const char *text){(void)text;}
static void exact_goal(uint16_t sx,uint16_t sy,uint16_t gx,uint16_t gy,float yaw,uint8_t obstacle)
{
  static grid_nav_segment_t segments[GRID_NAV_MAX_SEGMENTS];uint16_t n=0,i;int32_t x=sx,y=sy;
  GridNav_Init();GridNav_SetPoseMm(sx,sy);
  if(obstacle)GridNav_AddObstacleDiscMm(2019,965,50);
  CHECK(GridNav_PlanCheckedTargetMmHeading(gx,gy,yaw));
  CHECK(GridNav_GetSegments(segments,GRID_NAV_MAX_SEGMENTS,&n));
  for(i=0;i<n;i++) {
    uint16_t d=GridNav_GetSegmentDistanceMm(i);CHECK(d>0);
    x+=segments[i].dx*d;y-=segments[i].dy*d;
  }
  CHECK(x==gx && y==gy);
}
int main(void)
{
  vision_obstacle_t v={0};uint32_t s;float x,y;uint8_t i;
  const float yaw[4]={0,90,180,-90};
  const float want_x[4]={1665,1100,735,1300},want_y[4]={1300,1665,1100,735};
  const int32_t base[4]={0,0,0,0},pos[4]={2500,-2500,2500,-2500};
  const double signs[4]={1,-1,1,-1};
  CHECK(VisionObstacle_Accept(&v,0,1,300,-100,0,1)!=0);
  VisionObstacle_SetArmed(&v,1,1000);s=v.session;CHECK(s!=0 && v.armed);
  CHECK(!strcmp(VisionObstacle_Accept(&v,s+1,1,300,-100,0,1),"SESSION"));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,1,300,-100,201,1),"STALE"));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,1,300,-100,0,0),"INELIGIBLE"));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,1,300,191,0,1),"RANGE"));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,1,1201,0,0,1),"RANGE"));
  CHECK(!v.sequence_valid);
  CHECK(!VisionObstacle_Accept(&v,s,1,500,-100,200,1));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,1,300,-100,0,1),"SEQUENCE"));
  CHECK(!strcmp(VisionObstacle_Accept(&v,s,0,300,-100,0,1),"SEQUENCE"));
  v.last_sequence=0xffffffffU;CHECK(!VisionObstacle_Accept(&v,s,0,200,0,0,1));
  VisionObstacle_SetArmed(&v,0,1100);CHECK(v.session!=s && !v.armed);
  for(i=0;i<4;i++) {
    CHECK(VisionObstacle_Project(1200,1200,yaw[i],300,-100,&x,&y));
    NEAR(x,want_x[i]);NEAR(y,want_y[i]);
    CHECK(VisionObstacle_WheelPose(1200,1200,yaw[i],base,pos,signs,1000,100,&x,&y));
    NEAR(x,1200+250*cosf(yaw[i]*0.01745329252f));
    NEAR(y,1200+250*sinf(yaw[i]*0.01745329252f));
  }
  CHECK(!VisionObstacle_Project(2300,2300,0,300,0,&x,&y));
  CHECK(!VisionObstacle_WheelPose(1200,1200,0,base,pos,signs,0,100,&x,&y));
  CHECK(VisionObstacle_DiscCell(100,100,25,1,46));
  CHECK(VisionObstacle_DiscCell(100,100,25,2,45));
  CHECK(!VisionObstacle_DiscCell(100,100,25,1,1));
  NEAR(VisionObstacle_SpeedRatio(900),1);NEAR(VisionObstacle_SpeedRatio(800),1);
  NEAR(VisionObstacle_SpeedRatio(600),.5f);NEAR(VisionObstacle_SpeedRatio(500),.25f);NEAR(VisionObstacle_SpeedRatio(400),0);
  CHECK(VisionObstacle_PathSegment(1000,400,1200,400,1100,590,190));
  CHECK(!VisionObstacle_PathSegment(1000,400,1200,400,1100,591,190));
  GridNav_Init();
  {uint32_t before=GridNav_GetRevision();uint8_t count=GridNav_AddObstacleDiscMm(100,100,50);
   CHECK(count>=4);CHECK(GridNav_GetRevision()==before+1);
   CHECK(GridNav_IsObstacle(1,46) && GridNav_IsObstacle(2,45));
   CHECK(!GridNav_AddObstacleDiscMm(100,100,50));CHECK(GridNav_GetRevision()==before+1);
   CHECK(!GridNav_AddObstacleDiscMm(2400,100,50));
  }
  GridNav_Init();CHECK(GridNav_AddObstacleDiscMm(300,400,50)>0);
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(300),GridNav_MmToCellY(400)));
  CHECK(!GridNav_IsObstacle(GridNav_MmToCellX(300),GridNav_MmToCellY(2000)));
  exact_goal(2125,620,2125,1200,90,1);
  exact_goal(1190,380,2125,1150,90,0);
  exact_goal(2121,621,2120,1203,90,1);
  exact_goal(1190,380,1194,390,90,0);
  return failures;
}
