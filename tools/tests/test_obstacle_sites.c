#include "vision_obstacle.h"
#include "grid_nav.h"
#include "nav_corner.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(c) do{checks++;if(!(c)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
void ZDT_Console_Print(const char *s){(void)s;}
static void block(uint8_t id,float yaw)
{
  uint16_t x0,y0,x1,y1;CHECK(VisionObstacle_SiteBarrier(id,yaw,&x0,&y0,&x1,&y1));
  CHECK(GridNav_AddObstacleRectMm(x0,y0,x1,y1)>0);
}
static int bfs_distance(uint8_t sx,uint8_t sy,uint8_t gx,uint8_t gy)
{
  static int16_t distance[GRID_NAV_SIZE*GRID_NAV_SIZE];static uint16_t queue[GRID_NAV_SIZE*GRID_NAV_SIZE];
  uint16_t head=0,tail=0;int i;
  for(i=0;i<(int)(GRID_NAV_SIZE*GRID_NAV_SIZE);i++)distance[i]=-1;
  distance[sy*GRID_NAV_SIZE+sx]=0;queue[tail++]=(uint16_t)(sy*GRID_NAV_SIZE+sx);
  while(head<tail) {
    uint16_t id=queue[head++];int x=id%GRID_NAV_SIZE,y=id/GRID_NAV_SIZE;
    if(x==gx && y==gy)return distance[id];
    for(i=0;i<4;i++) {
      int nx=x+(i==0)-(i==1),ny=y+(i==2)-(i==3),next=ny*GRID_NAV_SIZE+nx;
      if(nx<0 || ny<0 || nx>=(int)GRID_NAV_SIZE || ny>=(int)GRID_NAV_SIZE || !GridNav_IsWalkable((uint8_t)nx,(uint8_t)ny))continue;
      if(distance[next]>=0)continue;
      distance[next]=(int16_t)(distance[id]+1);queue[tail++]=(uint16_t)next;
    }
  }
  return -1;
}
static uint16_t inner_steps(const grid_nav_point_t *path,uint16_t length)
{
  uint16_t count=0;
  for(uint16_t i=1;i<length;i++) {
    uint16_t x=GridNav_CellCenterMmX(path[i].x),y=GridNav_CellCenterMmY(path[i].y);
    if(x>=550 && x<1850 && y>=550 && y<1850)count++;
    CHECK(GridNav_IsWalkable(path[i].x,path[i].y));
    CHECK(abs((int)path[i].x-path[i-1].x)+abs((int)path[i].y-path[i-1].y)==1);
  }
  return count;
}
static uint8_t planning_clear(float x,float y,float yaw)
{return GridNav_RotatedBodyFreeMm((uint16_t)lroundf(x),(uint16_t)lroundf(y),(int16_t)lroundf(yaw));}
int main(void)
{
  uint8_t i;uint16_t x0,y0,x1,y1;
  GridNav_Init();
  {
    nav_corner_t curve;
    CHECK(NavCorner_Init(&curve,0,1,1500,1200,1200,1200,1200,1500,-1,0,148.5f,90));
    CHECK(NavCorner_Clear(&curve,planning_clear)); /* A clear central-cross curve remains usable. */
    CHECK(NavCorner_Init(&curve,0,1,1150,400,350,400,350,1200,-1,0,148.5f,90));
    CHECK(!NavCorner_Clear(&curve,planning_clear)); /* Near-yellow tangent plus yaw envelope rejected. */
    CHECK(NavCorner_Init(&curve,0,1,1150,350,350,350,350,1200,-1,0,148.5f,90));
    CHECK(NavCorner_Clear(&curve,planning_clear)); /* Wide outer tangent still rounds at the same radius. */
  }
  CHECK(GRID_NAV_YELLOW_MARGIN_MM==20);
  CHECK(GridNav_RotatedBodyFreeMm(390,800,0));CHECK(!GridNav_RotatedBodyFreeMm(391,800,0));
  CHECK(GridNav_RotatedBodyFreeMm(2010,800,0));CHECK(!GridNav_RotatedBodyFreeMm(2009,800,0));
  CHECK(GridNav_RotatedBodyFreeMm(800,390,0));CHECK(!GridNav_RotatedBodyFreeMm(800,391,0));
  CHECK(GridNav_RotatedBodyFreeMm(800,2010,0));CHECK(!GridNav_RotatedBodyFreeMm(800,2009,0));
  CHECK(GridNav_RotatedBodyFreeMm(1160,800,0));CHECK(!GridNav_RotatedBodyFreeMm(1159,800,0));
  CHECK(GridNav_RotatedBodyFreeMm(1240,800,0));CHECK(!GridNav_RotatedBodyFreeMm(1241,800,0));
  CHECK(GridNav_RotatedBodyFreeMm(430,430,45));CHECK(!GridNav_RotatedBodyFreeMm(450,450,45));
  CHECK(GridNav_RotatedBodyFreeMm(140,300,0)); /* Outer field boundary unchanged. */
  CHECK(GridNav_RotatedBodyFreeMm(1200,2174,0));CHECK(!GridNav_RotatedBodyFreeMm(1200,2175,0)); /* PAD unchanged. */
  for(uint8_t cy=0;cy<GRID_NAV_SIZE;cy++)for(uint8_t cx=0;cx<GRID_NAV_SIZE;cx++)
    if(GridNav_IsWalkable(cx,cy))CHECK(GridNav_RotatedBodyFreeMm(GridNav_CellCenterMmX(cx),GridNav_CellCenterMmY(cy),0));
  CHECK(!VisionObstacle_Site(0));CHECK(!VisionObstacle_Site(16));
  for(i=1;i<=VOBS_SITE_COUNT;i++) {
    const vision_obstacle_site_t *p=VisionObstacle_Site(i);CHECK(p!=0);
    CHECK(VisionObstacle_MatchSite(p->x,p->y,p->x+100,p->y,180)==i);
    GridNav_Init();block(i,90);
    /* All required parking goals stay usable; a blocked road is not a
     * blocked service pose. This includes the user-reported false NO_PATH. */
    CHECK(GridNav_RotatedBodyFreeMm(1250,2100,0));
    CHECK(GridNav_RotatedBodyFreeMm(1200,400,0));
    CHECK(GridNav_RotatedBodyFreeMm(350,1200,0));
    CHECK(GridNav_RotatedBodyFreeMm(2125,1150,0));
    CHECK(GridNav_RotatedBodyFreeMm(2250,150,0));
    CHECK(GridNav_RotatedBodyFreeMm(2250,2250,0));
  }
  CHECK(VisionObstacle_MatchSite(1183,580,1206,1300,-90)==11);
  CHECK(VisionObstacle_MatchSite(2170,1980,2125,830,90)==6);
  CHECK(!VisionObstacle_MatchSite(750,750,1200,1500,-90));
  CHECK(!VisionObstacle_MatchSite(NAN,580,1206,1300,-90));
  CHECK(!VisionObstacle_SiteBarrier(16,0,&x0,&y0,&x1,&y1));
  CHECK(VisionObstacle_SiteBarrier(1,0,&x0,&y0,&x1,&y1));CHECK(x0==225 && x1==325 && y0==1850 && y1==2400);
  CHECK(VisionObstacle_SiteBarrier(1,90,&x0,&y0,&x1,&y1));CHECK(x0==0 && x1==550 && y0==2075 && y1==2175);
  GridNav_Init();GridNav_SetPoseMm(1206,1137);
  CHECK(GridNav_AddObstacleDiscMm(1183,580,50)>0);
  CHECK(!GridNav_RotatedBodyFreeMm(1200,400,-90));
  GridNav_Init();block(11,-90);GridNav_SetPoseMm(1206,1137);
  CHECK(GridNav_RotatedBodyFreeMm(1200,400,-90));
  CHECK(GridNav_PlanCheckedTargetMmHeading(1200,400,-90));
  CHECK(!GridNav_RotatedBodyFreeMm(1200,762,-90));
  CHECK(!GridNav_RotatedBodyFreeMm(1100,762,-90));
  CHECK(!GridNav_RotatedBodyFreeMm(1300,762,-90));
  /* QR-after road closed first, then the lower central road: both station
   * journeys still have checked alternatives with both barriers present. */
  GridNav_Init();block(6,90);GridNav_SetPoseMm(2125,1162);
  CHECK(GridNav_PlanCheckedTargetMmHeading(1250,2100,90));
  block(11,-90);GridNav_SetPoseMm(1206,1327);
  CHECK(GridNav_PlanCheckedTargetMmHeading(1200,400,-90));
  {uint32_t rev=GridNav_GetRevision();CHECK(!GridNav_AddObstacleRectMm(1000,712,1400,812));
   CHECK(GridNav_GetRevision()==rev);CHECK(!GridNav_AddObstacleRectMm(20,50,10,90));
   CHECK(!GridNav_AddObstacleRectMm(0,0,2401,50));}
  GridNav_ClearRuntimeObstacles();CHECK(!GridNav_HasObstacles());
  /* Removing one camera layer keeps the other camera and manual geometry,
   * including manual cells added AFTER the camera already covered them. */
  CHECK(GridNav_AddObstacleRectMm(1000,1000,1100,1100)>0);
  CHECK(GridNav_SetVisionObstacleRectMm(1,950,1000,1200,1100)>0);
  CHECK(GridNav_SetVisionObstacleRectMm(2,1100,1000,1300,1100)>0);
  CHECK(GridNav_ClearVisionObstacle(1)>0);
  CHECK(!GridNav_IsObstacle(GridNav_MmToCellX(975),GridNav_MmToCellY(1050)));
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(1050),GridNav_MmToCellY(1050))==1);
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(1150),GridNav_MmToCellY(1050))==1);
  (void)GridNav_AddObstacleDiscMm(1250,1050,10);
  CHECK(GridNav_ClearVisionObstacle(2)>0);
  CHECK(!GridNav_IsObstacle(GridNav_MmToCellX(1150),GridNav_MmToCellY(1050)));
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(1250),GridNav_MmToCellY(1050))==1);
  CHECK(GridNav_IsObstacle(GridNav_MmToCellX(1050),GridNav_MmToCellY(1050))==1);
  GridNav_ClearRuntimeObstacles();CHECK(!GridNav_HasObstacles());
  CHECK(GridNav_SetVisionObstacleDiscMm(1,1200,1200,50)>0);
  CHECK(GridNav_SetVisionObstacleDiscMm(2,1200,1200,50)==0);
  CHECK(GridNav_ClearVisionObstacle(1)==0 && GridNav_HasObstacles());
  CHECK(GridNav_ClearVisionObstacle(2)>0 && !GridNav_HasObstacles());
  /* Runtime obstacles retain strict BFS distance. Empty-map outer preference
   * may spend at most the explicitly bounded200mm, regardless of turn cost. */
  {const uint16_t pairs[5][4]={{2125,1130,1250,2100},{380,1280,1250,2100},{1190,335,350,1200},{380,1280,2125,350},{1247,2068,1200,355}};
   static grid_nav_point_t path[GRID_NAV_SIZE*GRID_NAV_SIZE];
   for(uint8_t map=0;map<4;map++)for(uint8_t pair=0;pair<5;pair++) {
     uint16_t n=0;uint8_t sx,sy,gx,gy;int expected;GridNav_Init();
     if(map==1 || map==2)block(4,90);
     if(map==2)block(2,0);
     if(map==3){block(6,90);block(11,90);}
     GridNav_SetCostModel(400,800);GridNav_SetInitialHeading(180);
     sx=GridNav_MmToCellX(pairs[pair][0]);sy=GridNav_MmToCellY(pairs[pair][1]);
     gx=GridNav_MmToCellX(pairs[pair][2]);gy=GridNav_MmToCellY(pairs[pair][3]);
     expected=bfs_distance(sx,sy,gx,gy);
     if(expected<0)CHECK(!GridNav_Plan(sx,sy,gx,gy,path,&n));
     else {CHECK(GridNav_Plan(sx,sy,gx,gy,path,&n));
       CHECK((int)n-1>=expected);
       CHECK(map?((int)n-1==expected):((int)n-1<=expected+(int)(GRID_NAV_OUTER_EXTRA_MM/GRID_NAV_CELL_MM)));}
   }}
  {
    static grid_nav_point_t path[GRID_NAV_MAX_PATH];uint16_t n;int shortest;
    /* TEMP->RAW: favour the north/left wide band even when the initial
     * heading and a large turn penalty favour the central cross. */
    GridNav_Init();GridNav_SetInitialHeading(0);GridNav_SetCostModel(1000,2000);
    CHECK(GridNav_PlanMm(350,1240,1220,2130,path,&n));CHECK(inner_steps(path,n)==0);
    for(unsigned goal=0;goal<3;goal++) {
      const uint16_t y[3]={1900,1850,1800};uint8_t sx,sy,gx,gy;
      GridNav_Init();GridNav_SetInitialHeading(0);
      sx=GridNav_MmToCellX(350);sy=GridNav_MmToCellY(1250);
      gx=GridNav_MmToCellX(1200);gy=GridNav_MmToCellY(y[goal]);
      shortest=bfs_distance(sx,sy,gx,gy);CHECK(shortest>=0);
      CHECK(GridNav_Plan(sx,sy,gx,gy,path,&n));
      CHECK((int)n-1<=shortest+(int)(GRID_NAV_OUTER_EXTRA_MM/GRID_NAV_CELL_MM));
      if(goal<2) {CHECK(inner_steps(path,n)==0);CHECK((int)n-1>shortest);}
      else {CHECK(inner_steps(path,n)>0);CHECK((int)n-1==shortest);}
    }
    /* Even an off-route runtime obstacle disables this empty-map preference. */
    GridNav_Init();block(6,90);GridNav_SetInitialHeading(0);
    shortest=bfs_distance(GridNav_MmToCellX(350),GridNav_MmToCellY(1240),
        GridNav_MmToCellX(1220),GridNav_MmToCellY(2130));
    CHECK(GridNav_PlanMm(350,1240,1220,2130,path,&n));CHECK((int)n-1==shortest);
    GridNav_ClearRuntimeObstacles();CHECK(GridNav_PlanMm(350,1240,1220,2130,path,&n));
    CHECK(inner_steps(path,n)==0);
  }
  return failures;
}
