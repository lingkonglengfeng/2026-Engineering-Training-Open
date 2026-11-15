#include "screen_protocol.h"
#include "grid_nav.h"
#include "match_control.h"
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do{++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);}}while(0)
static match_control_t zdt_match;
static uint8_t zdt_script_running,nav_active,turn_active,demand,material,round_job,pose,pending,accepted;
static int16_t rpm[4];static screen_batch_t batch;
static uint8_t ZDT_Nav_IsActive(void){return nav_active;}
static uint8_t ChassisMotor_IsTurnActive(void){return turn_active;}
static uint8_t ChassisMotor_HasMotionDemand(void){return demand;}
static uint8_t ArmMaterial_IsActive(void){return material;}
static uint8_t ArmRound1_IsActive(void){return round_job;}
static uint8_t ArmStartPose_IsActive(void){return pose;}
static void ChassisMotor_GetWheelRpm(int16_t *out){uint8_t i;for(i=0;i<4;i++)out[i]=rpm[i];}
uint8_t ScreenHmi_TakeBatch(screen_batch_t *out){if(!pending)return 0;*out=batch;pending=0;return 1;}
void ScreenHmi_BatchResult(uint8_t value){accepted=value;}
static void ZDT_Console_Print(const char *text){CHECK(text!=0);}
#include "screen_owner_under_test.inc"
void Test_ScreenOwner(void)
{
  uint8_t i;uint32_t rev;
  GridNav_Init();zdt_match.state=MATCH_IDLE;batch.count=1;batch.points[0].x=batch.points[0].y=25;
  pending=1;ZDT_Screen_Service();CHECK(accepted && !pending && GridNav_IsObstacle(0,47));
  for(i=0;i<8;i++) {
    zdt_script_running=nav_active=turn_active=demand=material=round_job=pose=0;rpm[0]=0;
    zdt_script_running=i==0;nav_active=i==1;turn_active=i==2;demand=i==3;
    material=i==4;round_job=i==5;pose=i==6;rpm[0]=i==7;
    rev=GridNav_GetRevision();batch.count=0;pending=1;ZDT_Screen_Service();
    CHECK(!accepted && !pending && GridNav_GetRevision()==rev && GridNav_IsObstacle(0,47));
  }
  zdt_script_running=nav_active=turn_active=demand=material=round_job=pose=0;rpm[0]=0;
  for(i=MATCH_IDLE;i<=MATCH_WAIT_START_OK;i++) {
    zdt_match.state=(match_state_t)i;rev=GridNav_GetRevision();pending=1;ZDT_Screen_Service();
    CHECK(accepted==(i==MATCH_IDLE || i==MATCH_WAIT_START_OK));
    if(!accepted)CHECK(GridNav_GetRevision()==rev);
  }
  zdt_match.state=MATCH_IDLE;batch.count=1;batch.points[0].x=25;batch.points[0].y=25;pending=1;ZDT_Screen_Service();
  batch.points[0].x=2400;rev=GridNav_GetRevision();pending=1;ZDT_Screen_Service();
  CHECK(!accepted && GridNav_GetRevision()==rev && GridNav_IsObstacle(0,47));
  batch.count=20;pending=1;ZDT_Screen_Service();CHECK(!accepted && GridNav_GetRevision()==rev);GridNav_Init();
}
