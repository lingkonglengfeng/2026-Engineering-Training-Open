/* Mocks only IO ports; the command/service/finish bodies below are production. */
static uint32_t zdt_manual_align_token,zdt_manual_align_next,zdt_manual_align_started;
static uint32_t zdt_match_request,zdt_arm_camera_token,now=1000;
enum {MATCH_IO_NONE,MATCH_IO_ALIGN};
static uint8_t zdt_match_io,zdt_qr_active,zdt_align_phase;
static uint8_t busy,flow_active,material_active,round_active,pose_active,camera_ready,camera_success=1,start_ok=1;
static unsigned requests,enable_calls,starts,stops,completed,failed,arm_cancels;
static int zdt_align;
static struct {unsigned active;} zdt_align_diag;
static char last_line[200];
uint32_t HAL_GetTick(void){return now;}
static void ZDT_Console_Print(const char *line){strncpy(last_line,line,sizeof(last_line)-1);}
static uint8_t ZDT_Align_DiagBusy(void){return busy;}
uint8_t MatchControl_IsActive(const match_control_t *flow){(void)flow;return flow_active;}
static uint8_t ArmMaterial_IsActive(void){return material_active;}
static uint8_t ArmRound1_IsActive(void){return round_active;}
static uint8_t ArmStartPose_IsActive(void){return pose_active;}
static HAL_StatusTypeDef ArmTask_RequestCameraDeploy(uint32_t token){CHECK(token&0x80000000U);requests++;return HAL_OK;}
static void ArmTask_RequestStop(void){arm_cancels++;}
static uint8_t ArmTask_GetCameraDeployResult(uint32_t token,uint8_t *ok){CHECK(token==zdt_manual_align_token);*ok=camera_success;return camera_ready;}
static HAL_StatusTypeDef ChassisMotor_Enable(void){enable_calls++;return HAL_OK;}
static HAL_StatusTypeDef ChassisMotor_Stop(void){stops++;return HAL_OK;}
static HAL_StatusTypeDef ChassisMotor_StopImmediate(void){stops++;return HAL_OK;}
static uint8_t ZDT_Align_Start(uint8_t station){CHECK(station==zdt_manual_align_station);starts++;zdt_match_io=MATCH_IO_ALIGN;return start_ok;}
static void VisionDiag_Finish(void *diag,uint32_t tick,uint8_t success,const char *reason){(void)diag;(void)tick;(void)success;(void)reason;}
static void VisionAlign_Cancel(int *align){(void)align;}
void VisionYaw_Cancel(vision_yaw_t *yaw){yaw->phase=V_YAW_IDLE;}
static void ZDT_Match_Complete(uint8_t success,const char *reason);
/* INSERT_MANUAL_FUNCTIONS */
static void ZDT_Match_Complete(uint8_t success,const char *reason)
{CHECK(ZDT_ManualAlignFinish(success,reason));if(success)completed++;else failed++;}
static void test_manual_alignment(void)
{
 unsigned before;match_state_t state;match_checkpoint_t checkpoint;
 zdt_manual_align_station=0;zdt_match_io=MATCH_IO_NONE;zdt_match.state=MATCH_IDLE;zdt_match.checkpoint=MATCH_RAW1;
 state=zdt_match.state;checkpoint=zdt_match.checkpoint;zdt_vyaw.enabled=1;
 CHECK(!ZDT_ManualAlignCommand("arm ui"));
 flow_active=1;before=requests;CHECK(ZDT_ManualAlignCommand("valign run rough") && requests==before);flow_active=0;
 busy=1;CHECK(ZDT_ManualAlignCommand("valign run temp") && requests==before);busy=0;
 pose_active=1;CHECK(ZDT_ManualAlignCommand("valign run rough") && requests==before);pose_active=0;
 CHECK(ZDT_ManualAlignCommand("valign run rough") && zdt_manual_align_station==2 && zdt_manual_align_token && requests==before+1);
 camera_ready=0;ZDT_ManualAlignService();CHECK(!enable_calls && !starts);
 camera_ready=1;ZDT_ManualAlignService();CHECK(enable_calls==1 && starts==1 && !zdt_manual_align_token && zdt_manual_align_station==2);
 ZDT_Match_Complete(1,0);CHECK(completed==1 && !zdt_manual_align_station && zdt_match_io==MATCH_IO_NONE);
 CHECK(zdt_match.state==state && zdt_match.checkpoint==checkpoint && !zdt_match_request && !arm_cancels);
 CHECK(!ZDT_ManualAlignFinish(0,"ordinary match is untouched"));
 CHECK(ZDT_ManualAlignCommand("valign run temp") && zdt_manual_align_station==3);
 CHECK(ZDT_Align_UsesYaw(3));camera_success=0;ZDT_ManualAlignService();
 CHECK(failed==1 && !zdt_manual_align_station && enable_calls==1 && zdt_match.state==state);
 camera_ready=0;camera_success=1;CHECK(ZDT_ManualAlignCommand("valign run rough"));
 now+=120000;ZDT_ManualAlignService();CHECK(failed==2 && !zdt_manual_align_station && !zdt_manual_align_token && arm_cancels==1);
 CHECK(zdt_match.state==state && zdt_match.checkpoint==checkpoint && !zdt_match_request);
 /* PLATE reuses competition station1 without yaw, pickup or MATCH advance. */
 zdt_vyaw.enabled=0;before=requests;camera_ready=0;
 CHECK(ZDT_ManualAlignCommand("valign run plate") && zdt_manual_align_station==1 && requests==before+1);
 CHECK(!ZDT_Align_UsesYaw(1));before=starts;
 ZDT_ManualAlignService();CHECK(starts==before && zdt_manual_align_token);
 camera_ready=1;ZDT_ManualAlignService();CHECK(starts==before+1 && !zdt_manual_align_token);
 CHECK(ZDT_ManualAlignFinish(1,0) && !zdt_manual_align_station && zdt_match_io==MATCH_IO_NONE);
 CHECK(zdt_match.state==state && zdt_match.checkpoint==checkpoint && !zdt_match_request && !material_active && !round_active);
}
