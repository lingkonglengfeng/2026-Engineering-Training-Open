#include "vision_align.h"
#include "vision_align_diag.h"
#include "vision_yaw.h"
#include "vision_camera.h"
#include "match_control.h"
#include "grid_nav.h"
#include "nav_params.h"
#include "chassis_motor.h"
#include "nav_blend.h"
#include "nav_frame.h"
#include "nav_odometry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
extern volatile uint32_t test_checks,test_failures,test_first_failure;
extern void Test_RecordFailure(const char *,uint32_t);
#define CHECK(x) do { ++test_checks;if(!(x)){++test_failures;if(!test_first_failure)test_first_failure=__LINE__;Test_RecordFailure(__FILE__,__LINE__);} }while(0)
extern void Test_SetTick(uint32_t tick);
static vision_align_diag_t zdt_align_diag;
static vision_align_t zdt_align;
static vision_yaw_t zdt_vyaw;
static uint8_t zdt_ring_profile;
static uint8_t zdt_manual_align_station;
static uint8_t profile_busy;
static uint8_t ZDT_Align_DiagBusy(void){return profile_busy;}
static float zdt_align_heading;
static float zdt_align_measure_heading;
static uint8_t zdt_temp2_still_tracking,zdt_temp2_skip_align;
static uint32_t zdt_temp2_still_tick;
static int8_t zdt_nav_nose_x=0,zdt_nav_nose_y=-1;
extern void Test_SetChassisYaw(float yaw);
static vision_align_config_t zdt_align_config[3];
static match_control_t zdt_match;
static uint8_t zdt_align_phase,zdt_nav_post_align,completed,success;
static const char *reason;
static uint8_t approach_starts;
uint8_t ZDT_Odo_Start(uint8_t owner,float mm,float scale,int8_t f,int8_t r,
    float heading,const int32_t *base,uint8_t mask)
{
  CHECK(owner==3U && mm==30.0f && scale==0.080f && f==0 && r==-1);
  CHECK(heading==zdt_align_heading && base && mask==15U);
  approach_starts++;return 1U;
}
static int32_t zdt_mileage_query_position[4],zdt_align_wheel_start[4];
static const double zdt_mileage_forward_signs[4]={-CHASSIS_WHEEL_SIGN_FL,
    -CHASSIS_WHEEL_SIGN_FR,-CHASSIS_WHEEL_SIGN_RL,-CHASSIS_WHEEL_SIGN_RR};
static void ZDT_Match_Complete(uint8_t ok,const char *why)
{completed=1U;success=ok;reason=why;}
static uint8_t AnchorTest_Pixel(uint8_t camera,uint8_t type,vision_pixel_t *p,uint32_t *age)
{(void)camera;(void)type;memset(p,0,sizeof(*p));*age=UINT32_MAX;return 0U;}
#define Vision_GetPixel AnchorTest_Pixel
static uint8_t AnchorTest_Camera(uint8_t camera,vision_camera_state_t *state)
{(void)camera;memset(state,0,sizeof(*state));state->request_seq=7U;return 1U;}
#define Vision_GetCameraState AnchorTest_Camera
/* These are production functions extracted verbatim by the test runner. */
#include "align_anchor_under_test.inc"
#undef Vision_GetPixel
#undef Vision_GetCameraState
static void reset(match_checkpoint_t checkpoint,uint8_t phase)
{
    uint8_t i;
    GridNav_Init();GridNav_SetPoseMm(2125U,1200U);zdt_match.checkpoint=checkpoint;
    zdt_align_phase=phase;zdt_nav_post_align=0U;completed=success=0U;reason=NULL;
    approach_starts=0U;
    zdt_temp2_still_tracking=zdt_temp2_skip_align=0U;zdt_align_measure_heading=0;
    zdt_align_heading=0.0f;
    memset(&zdt_align,0,sizeof(zdt_align));VisionDiag_Init(&zdt_align_diag);
    VisionYaw_Init(&zdt_vyaw);ChassisMotor_Init();Test_SetChassisYaw(0.0f);
    zdt_nav_nose_x=0;zdt_nav_nose_y=-1;
    Nav_Params_Reset();Test_SetTick(1000U);
    for(i=0U;i<4U;i++) {zdt_align_wheel_start[i]=1000;zdt_mileage_query_position[i]=100000;}
    for(i=0U;i<3U;i++) {
        VisionAlign_DefaultConfig(&zdt_align_config[i]);zdt_align_config[i].camera=1U;
        zdt_align_config[i].type=VISION_TYPE_RING_PX;zdt_align_config[i].id=2U;
        zdt_align_config[i].forward_x=0.0007f;zdt_align_config[i].right_y=0.0007f;
    }
    VisionDiag_Begin(&zdt_align_diag,1U,&zdt_align_config[0],1000U);
}
static void pose(uint16_t expected_x,uint16_t expected_y)
{uint16_t x,y;GridNav_GetPoseMm(&x,&y);CHECK(x==expected_x && y==expected_y);}
void Test_AlignmentNoApproach(void)
{
    static const match_checkpoint_t checkpoints[]={MATCH_RAW1,MATCH_ROUGH1,MATCH_TEMP1,MATCH_RAW2,MATCH_ROUGH2,MATCH_TEMP2};
    static const uint16_t anchors[][2]={{1250,2090},{1190,380},{380,1280}};
    uint8_t i,zone,round;uint16_t count;grid_nav_segment_t segments[4];
    for(i=0;i<6;i++) {
        reset(checkpoints[i],3U);ZDT_Align_OnMileage(15U);
        CHECK(completed && success && !reason && zdt_align_phase==0U && zdt_nav_post_align);
        CHECK(!approach_starts && zdt_align_diag.end_mask==15U);
        pose(anchors[i%3][0],anchors[i%3][1]);
    }
    for(zone=1;zone<=2;zone++)for(round=0;round<2;round++) {
        reset(round?MATCH_RAW2:MATCH_RAW1,3U);
        zdt_nav_nose_y=zone==1?-1:1;zdt_align_heading=zone==1?-90.0f:90.0f;
        ZDT_Align_OnMileage(15U);
        CHECK(completed && success && !approach_starts);pose(1250U,2090U);
        CHECK(fabsf(NavBlend_AngleError(ZDT_Nav_GetHeading(),ZDT_Zone2_RelYaw(0.0f)))<0.001f);
        completed=0;ZDT_Align_OnMileage(15U);CHECK(!completed && !approach_starts);pose(1250U,2090U);
        CHECK(GridNav_PlanRecenterTargetMmHeading(1200,400,0));
        CHECK(GridNav_GetSegments(segments,4,&count) && count==2);
        CHECK(GridNav_GetSegmentDistanceMm(0)==50 && GridNav_GetSegmentDistanceMm(1)==1690);
    }
    reset(MATCH_RAW1,3U);ZDT_Align_OnMileage(7U);
    CHECK(completed && !success && !approach_starts && !zdt_nav_post_align);pose(2125,1200);
    reset(MATCH_QR,3U);ZDT_Align_OnMileage(15U);
    CHECK(completed && !success && !approach_starts);pose(2125,1200);
    /* Only the final automatic TEMP2 ring stage may bypass after2s still. */
    for(i=0;i<6;i++) {
        reset(checkpoints[i],2U);zdt_align.config.type=VISION_TYPE_RING_PX;
        CHECK(!ZDT_Align_Temp2StationaryReady(1000,1));
        CHECK(!ZDT_Align_Temp2StationaryReady(2999,1));
        CHECK(ZDT_Align_Temp2StationaryReady(3000,1)==(i==5));
    }
    reset(MATCH_TEMP2,2U);zdt_align.config.type=VISION_TYPE_RING_PX;
    CHECK(!ZDT_Align_Temp2StationaryReady(1000,1));CHECK(!ZDT_Align_Temp2StationaryReady(2900,0));
    CHECK(!ZDT_Align_Temp2StationaryReady(3000,1));CHECK(!ZDT_Align_Temp2StationaryReady(4999,1));
    CHECK(ZDT_Align_Temp2StationaryReady(5000,1) && zdt_temp2_skip_align);
    Test_SetChassisYaw(-13.0f);
    zdt_align_phase=3;for(i=0;i<4;i++)zdt_mileage_query_position[i]=zdt_align_wheel_start[i];
    ZDT_Align_OnMileage(15);CHECK(completed && success && !zdt_temp2_skip_align && zdt_align_phase==0);pose(2125,1200); /* No fake visual anchor. */
    CHECK(fabsf(ZDT_Nav_GetHeading()+13.0f)<0.001f); /* Bypass cannot establish a visual heading. */
    completed=0;ZDT_Align_OnMileage(15);CHECK(!completed);
    reset(MATCH_TEMP2,2U);zdt_align.config.type=VISION_TYPE_RING_PX;zdt_manual_align_station=3;
    CHECK(!ZDT_Align_Temp2StationaryReady(1000,1) && !ZDT_Align_Temp2StationaryReady(4000,1));zdt_manual_align_station=0;
    reset(MATCH_TEMP2,4U);zdt_align.config.type=VISION_TYPE_RING_PX;CHECK(!ZDT_Align_Temp2StationaryReady(4000,1));
    reset(MATCH_TEMP2,2U);zdt_align.config.type=VISION_TYPE_PLATE_PX;CHECK(!ZDT_Align_Temp2StationaryReady(4000,1));
    reset(MATCH_TEMP2,2U);zdt_align.config.type=VISION_TYPE_RING_PX;
    CHECK(!ZDT_Align_Temp2StationaryReady(UINT32_MAX-1000U,1));CHECK(!ZDT_Align_Temp2StationaryReady(998U,1));
    CHECK(ZDT_Align_Temp2StationaryReady(999U,1));
}
void Test_AlignmentAnchors(void)
{
    static const match_checkpoint_t checkpoints[6]={MATCH_RAW1,MATCH_ROUGH1,MATCH_TEMP1,
        MATCH_RAW2,MATCH_ROUGH2,MATCH_TEMP2};
    static const uint16_t positions[3][2]={{1250U,2090U},{1190U,380U},{380U,1280U}};
    uint8_t i;uint16_t x=17U,y=18U,count;grid_nav_segment_t segments[4];
    CHECK(!VisionAlign_StationPoseMm(0U,&x,&y) && x==17U && y==18U);
    CHECK(!VisionAlign_StationPoseMm(4U,&x,&y));
    CHECK(!VisionAlign_StationPoseMm(1U,NULL,&y));
    CHECK(!VisionAlign_StationPoseMm(1U,&x,NULL));
    /* V225 changes only the RAW2 nominal camera search side in its runtime
     * copy. Other XY/search/confirm parameters and defaults stay intact. */
    {
      unsigned cp;vision_align_config_t c;
      for(cp=MATCH_RAW1;cp<=MATCH_TEMP2;cp++) {
        uint8_t station=(uint8_t)((cp-1U)%3U+1U);
        reset((match_checkpoint_t)cp,1U);
        CHECK(VisionAlign_StationConfig(&zdt_align_config[station-1U],station));
        CHECK(ZDT_Align_ConfigForStage(station,&c));
        CHECK(c.search_axis==V_ALIGN_SEARCH_IMAGE_RIGHT);
        CHECK(c.plate_search_right==(cp==MATCH_RAW2));
        CHECK(!zdt_align_config[station-1U].plate_search_right);
        CHECK(c.search_speed==(station==1U?0.030f:0.040f));
        CHECK(zdt_align_config[station-1U].tolerance_px==(station==1U?21.0f:5.0f));
      }
      reset(MATCH_TEMP2,1U);zdt_align_config[2].search_axis=V_ALIGN_SEARCH_OFF;
      CHECK(ZDT_Align_ConfigForStage(3,&c) && !c.search_axis); /* Explicit search OFF remains honoured. */
    }
    /* Fast empty-ring service everywhere except the occupied TEMP2 layer;
     * axis08 request is independent of occupied detection. */
    {
      unsigned cp;
      for(cp=MATCH_RAW1;cp<=MATCH_TEMP2;cp++) {
        uint8_t station=(uint8_t)((cp-1U)%3U+1U),expected=(cp==MATCH_TEMP2)?3U:1U;
        zdt_match.checkpoint=(match_checkpoint_t)cp;zdt_vyaw.enabled=1;
        CHECK(ZDT_Align_Flags(station==1U?VISION_MODE_PLATE:VISION_MODE_RING,station)==(station==1U?0U:expected));
        zdt_vyaw.enabled=0;CHECK(ZDT_Align_Flags(VISION_MODE_RING,station)==(cp==MATCH_TEMP2?2U:0U));
        CHECK(ZDT_Align_Flags(VISION_MODE_MATERIAL,station)==0U);
      }
      zdt_vyaw.enabled=1;zdt_match.checkpoint=MATCH_ROUGH1;zdt_ring_profile=2;
      CHECK(ZDT_Align_Flags(VISION_MODE_RING,2)==3U);
      zdt_match.checkpoint=MATCH_TEMP2;zdt_ring_profile=1;CHECK(ZDT_Align_Flags(VISION_MODE_RING,3)==1U);
      zdt_ring_profile=0;
      CHECK(ZDT_RingProfileCommand("valign rings fast") && zdt_ring_profile==1);
      CHECK(ZDT_RingProfileCommand("valign rings occupied") && zdt_ring_profile==2);
      profile_busy=1;CHECK(ZDT_RingProfileCommand("valign rings auto") && zdt_ring_profile==2);profile_busy=0;
      CHECK(ZDT_RingProfileCommand("valign rings status") && zdt_ring_profile==2);
      CHECK(ZDT_RingProfileCommand("valign rings invalid") && zdt_ring_profile==2);
      CHECK(!ZDT_RingProfileCommand("valign ringsx") && !ZDT_RingProfileCommand("other"));
      CHECK(ZDT_RingProfileCommand("valign rings auto") && !zdt_ring_profile);
    }
    /* V101: only completed visual yaw can bind actual stopped heading to
     * the station's map direction. Both start zones and both rounds use it. */
    {
      uint8_t zone,station,round;chassis_telemetry_t t;
      for(zone=1U;zone<=2U;zone++)for(round=0U;round<2U;round++)for(station=2U;station<=3U;station++) {
        float expected=(station==2U?180.0f:90.0f)-(zone==1U?90.0f:-90.0f);
        reset((match_checkpoint_t)(station+round*3U),3U);
        zdt_nav_nose_y=zone==1U?-1:1;
        Test_SetChassisYaw(expected+3.5f-720.0f);
        zdt_vyaw.phase=V_YAW_DONE;zdt_vyaw.station=station;
        ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);
        CHECK(completed && success && fabsf(t.heading_deg-(expected-720.0f))<0.001f);
        CHECK(fabsf(zdt_align_heading-t.heading_deg)<0.001f);
        /* Turning later keeps the same corrected coordinate frame. */
        Test_SetChassisYaw(expected+3.5f-630.0f);ChassisMotor_GetTelemetry(&t);
        CHECK(fabsf(t.heading_deg-(expected-630.0f))<0.001f);
      }
      reset(MATCH_ROUGH1,3U);Test_SetChassisYaw(93.5f);
      ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);
      CHECK(success && t.heading_deg==93.5f); /* yaw OFF / plain XY has no authority */
      reset(MATCH_ROUGH1,3U);Test_SetChassisYaw(93.5f);zdt_vyaw.phase=V_YAW_FAILED;
      ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==93.5f);
      reset(MATCH_RAW1,3U);Test_SetChassisYaw(3.5f);zdt_vyaw.phase=V_YAW_DONE;zdt_vyaw.station=2U;
      ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==3.5f);
      reset(MATCH_TEMP1,3U);Test_SetChassisYaw(3.5f);zdt_vyaw.phase=V_YAW_DONE;zdt_vyaw.station=2U;
      ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==3.5f);
      reset(MATCH_ROUGH1,3U);Test_SetChassisYaw(93.5f);zdt_vyaw.phase=V_YAW_DONE;zdt_vyaw.station=2U;
      ZDT_Align_OnMileage(7U);ChassisMotor_GetTelemetry(&t);CHECK(!success && t.heading_deg==93.5f);
      /* API rejects live motion and nonfinite coordinates without changing yaw. */
      reset(MATCH_ROUGH1,3U);Test_SetChassisYaw(93.5f);zdt_vyaw.phase=V_YAW_DONE;zdt_vyaw.station=2U;
      CHECK(ChassisMotor_Enable()==HAL_OK && ChassisMotor_SetVelocity(0.2f,0,0)==HAL_OK);
      ZDT_Align_OnMileage(15U);ChassisMotor_GetTelemetry(&t);
      CHECK(completed && !success && !zdt_nav_post_align && t.heading_deg==93.5f);
      CHECK(ChassisMotor_StopImmediate()==HAL_OK);
      CHECK(ChassisMotor_RebaseHeading(NAN)==HAL_ERROR);
      CHECK(ChassisMotor_RebaseHeading(90.0f)==HAL_OK);
      CHECK(ChassisMotor_RebaseHeading(90.0f)==HAL_OK); /* repeated binds never accumulate */
      ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==90.0f);
      CHECK(ChassisMotor_TurnRelativeFast(90)==HAL_OK);ChassisMotor_GetTelemetry(&t);
      CHECK(ChassisMotor_RebaseHeading(90)==HAL_BUSY);
      Test_SetChassisYaw(183.5f);Test_SetTick(1004U);
      CHECK(ChassisMotor_Update()==HAL_OK);ChassisMotor_GetTelemetry(&t);
      CHECK(t.heading_deg==180.0f && t.target_heading_deg==180.0f && t.pid_error_deg==0.0f);
      CHECK(ChassisMotor_StopImmediate()==HAL_OK);
      Test_SetChassisYaw(NAN);CHECK(ChassisMotor_RebaseHeading(90)==HAL_ERROR);
      Test_SetChassisYaw(93.5f);ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==90.0f);
      ChassisMotor_ClearHeadingReference();ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==93.5f);
      CHECK(ChassisMotor_RebaseHeading(90.0f)==HAL_OK);ChassisMotor_Init();
      ChassisMotor_GetTelemetry(&t);CHECK(t.heading_deg==93.5f);
    }
    /* Stack-only wider centre: actual stage config copied into production
     * alignment, three NEW frames at9,-8px pass; normal TEMP still drives. */
    {
      unsigned cp,k;vision_align_config_t c;
      for(cp=MATCH_RAW1;cp<=MATCH_TEMP2;cp++) {
        uint8_t station=(uint8_t)((cp-1U)%3U+1U);
        reset((match_checkpoint_t)cp,1U);CHECK(ZDT_Align_ConfigForStage(station,&c));
        CHECK(c.tolerance_px==(cp==MATCH_TEMP2?10.0f:5.0f) && c.stable_frames==(cp==MATCH_TEMP2?3U:5U));
        CHECK(zdt_align_config[station-1U].tolerance_px==5.0f && zdt_align_config[station-1U].stable_frames==5U);
      }
      CHECK(!ZDT_Align_ConfigForStage(0,&c) && !ZDT_Align_ConfigForStage(3,NULL));
      for(cp=0;cp<4;cp++) {
        vision_pixel_t p={0};float f=0,r=0;
        reset(cp==0?MATCH_TEMP1:MATCH_TEMP2,1U);ZDT_Align_OnMileage(15U);
        CHECK(zdt_align.config.tolerance_px==(cp==0?5.0f:10.0f));
        VisionAlign_Step(&zdt_align,1000,NULL,0,1,&f,&r);
        VisionAlign_Step(&zdt_align,1250,NULL,0,1,&f,&r);
        p.type=VISION_TYPE_RING_PX;p.id=2;p.valid=1;
        p.dx_px=cp==2?10.0f:cp==3?11.0f:9.0f;p.dy_px=cp==2?-10.0f:-8.0f;
        for(k=0;k<3;k++){p.observation_id=k+1;VisionAlign_Step(&zdt_align,1300+k*100,&p,0,1,&f,&r);}
        if(cp==1 || cp==2)CHECK(zdt_align.state==V_ALIGN_DONE && f==0 && r==0);
        else CHECK(zdt_align.state!=V_ALIGN_DONE);
      }
      reset(MATCH_TEMP2,1U);zdt_align_config[2].tolerance_px=15;
      CHECK(ZDT_Align_ConfigForStage(3,&c) && c.tolerance_px==15); /* retain a manually wider tolerance */
      reset(MATCH_TEMP1,1U);ZDT_Align_OnMileage(15U);CHECK(zdt_align.config.tolerance_px==5 && zdt_align.config.stable_frames==5);
    }
    for(i=0U;i<6U;i++) {
        reset(checkpoints[i],3U);
        if(i%3U==0U)zdt_align_heading=-90.0f; /* initial north, RAW nose east */
        ZDT_Align_OnMileage(0x0FU);
        CHECK(completed && success && !reason && zdt_align_phase==0U && zdt_nav_post_align);
        pose(positions[i%3U][0],positions[i%3U][1]);CHECK(!approach_starts);
        CHECK(zdt_align_diag.end_mask==0x0FU);
    }
    /* Both start zones and both rounds hand off directly at the aligned
     * anchor. A second completed handoff must never start an approach move. */
    {uint8_t zone,round;
      for(zone=1U;zone<=2U;zone++)for(round=0U;round<2U;round++) {
        reset(round?MATCH_RAW2:MATCH_RAW1,3U);zdt_nav_nose_y=zone==1U?-1:1;
        zdt_align_heading=zone==1U?-90.0f:90.0f;ZDT_Align_OnMileage(15U);
        CHECK(completed && success && zdt_align_phase==0U && !approach_starts);pose(1250U,2090U);
        completed=0U;ZDT_Align_OnMileage(15U);
        CHECK(!completed && !approach_starts);pose(1250U,2090U);
      }
    }
    reset(MATCH_RAW1,3U);ZDT_Align_OnMileage(7U);
    CHECK(completed && !success && reason && !zdt_nav_post_align);pose(2125U,1200U);
    reset(MATCH_QR,3U);ZDT_Align_OnMileage(15U);
    CHECK(completed && !success && reason && !zdt_nav_post_align);pose(2125U,1200U);
    reset(MATCH_RAW1,3U);Nav_Params()->counts_per_rev=0U;ZDT_Align_OnMileage(15U);
    CHECK(completed && !success && reason && !zdt_nav_post_align);pose(2125U,1200U);
    reset(MATCH_RAW1,1U);ZDT_Align_OnMileage(15U);
    CHECK(!completed && zdt_align_phase==2U && !zdt_nav_post_align);pose(2125U,1200U);
    reset(MATCH_QR,1U);ZDT_Align_OnMileage(15U);
    CHECK(completed && !success && reason && !zdt_nav_post_align);pose(2125U,1200U);
    /* Execute the real footprint-aware planner from each newly aligned pose. */
    reset(MATCH_RAW1,3U);ZDT_Align_OnMileage(15U);
    zdt_nav_nose_y=-1;zdt_align_heading=-90.0f;
    CHECK(!approach_starts && completed && success);pose(1250U,2090U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(1200U,400U,0.0f));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==50U && GridNav_GetSegmentDistanceMm(1U)==1690U);
    GridNav_SetPoseMm(1250U,2100U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(1200U,400U,0.0f));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==50U && GridNav_GetSegmentDistanceMm(1U)==1700U);
    {int angle;for(angle=0;angle<=180;angle+=3)CHECK(GridNav_RotatedBodyFreeMm(1250U,2090U,(int16_t)angle));}
    reset(MATCH_ROUGH1,3U);ZDT_Align_OnMileage(15U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(350U,400U,180.0f));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==20U && GridNav_GetSegmentDistanceMm(1U)==840U);
    reset(MATCH_TEMP1,3U);ZDT_Align_OnMileage(15U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(350U,2000U,90.0f));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==30U && GridNav_GetSegmentDistanceMm(1U)==720U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,1200U,0.0f));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==80U && GridNav_GetSegmentDistanceMm(1U)==1745U);
    /* Real odometry retains the small end error. Keep an exact-mm lane join
     * for following actions rather than falling back to snapped A* cells. */
    GridNav_Init();GridNav_SetPoseMm(2127U,448U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,1200U,90.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==2U && GridNav_GetSegmentDistanceMm(1U)==752U);
    GridNav_SetPoseMm(351U,1998U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(1200U,2000U,90.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==2U && GridNav_GetSegmentDistanceMm(1U)==849U);
    GridNav_SetPoseMm(2123U,1200U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,350U,-90.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==2U && GridNav_GetSegmentDistanceMm(1U)==850U);
    /* Actual V2 exit and the V1 QR-return end from the uploaded trace. */
    GridNav_SetPoseMm(2123U,452U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,1200U,90.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==2U && GridNav_GetSegmentDistanceMm(1U)==748U);
    GridNav_SetPoseMm(2123U,1202U);
    CHECK(GridNav_PlanRecenterTargetMmHeading(2125U,350U,-809.91f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==2U && GridNav_GetSegmentDistanceMm(1U)==852U);
    GridNav_Init();GridNav_SetPoseMm(380U,1280U);
    GridNav_SetObstacle(GridNav_MmToCellX(1200U),GridNav_MmToCellY(1200U),1U);
    CHECK(!GridNav_PlanRecenterTargetMmHeading(2125U,1200U,0.0f));
    CHECK(GridNav_PlanCheckedTargetMmHeading(2125U,1200U,0.0f));
}
