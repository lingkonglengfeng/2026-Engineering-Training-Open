#include "start_button.h"
#include "bt_button.h"
#include "match_control.h"
#include "gpio.h"
#include "fdcan.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile unsigned int test_checks,test_failures,test_first_failure;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
static uint32_t tick;static uint8_t pin_high,pose_ready,imu_ready,enabled,busoff,passive;
static uint32_t input_configs,output_configs;
static uint32_t pin_reads,irq_count,key_logs,edge_logs,release_logs,press_logs,ignored_logs;
static uint8_t irq_context;
static HAL_StatusTypeDef can_result,enable_result;
static match_control_t zdt_match;
static start_button_t zdt_start_button;
static uint8_t zdt_start_debounce_pending;
static struct {uint8_t can_ready;} zdt_status;
static uint8_t zdt_qr_enabled,zdt_qr_locked;
FDCAN_HandleTypeDef hfdcan2;
uint32_t HAL_GetTick(void){return tick;}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin){CHECK(port==GPIOD && pin==GPIO_PIN_1);pin_reads++;return pin_high?GPIO_PIN_SET:GPIO_PIN_RESET;}
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin){CHECK(pin==GPIO_PIN_1);irq_count++;irq_context=1U;HAL_GPIO_EXTI_Callback(pin);irq_context=0U;}
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state){(void)state;if(port==GPIOD)CHECK(!(pin&GPIO_PIN_1));}
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *cfg){
 if(port==GPIOD && (cfg->Pin&GPIO_PIN_1)){CHECK(cfg->Pin==GPIO_PIN_1 && cfg->Mode==GPIO_MODE_IT_RISING_FALLING && cfg->Pull==GPIO_PULLUP);input_configs++;}
 if(port==GPIOA && (cfg->Pin&GPIO_PIN_1))CHECK(cfg->Mode==GPIO_MODE_INPUT && cfg->Pull==GPIO_PULLUP);
 if(port==GPIOD && cfg->Mode==GPIO_MODE_OUTPUT_PP){CHECK(!(cfg->Pin&GPIO_PIN_1));output_configs++;}}
void HAL_NVIC_SetPriority(IRQn_Type irq,uint32_t pre,uint32_t sub){CHECK(irq==EXTI1_IRQn && pre==5 && sub==0);}
void HAL_NVIC_EnableIRQ(IRQn_Type irq){CHECK(irq==EXTI1_IRQn);}
uint8_t ArmTask_IsStartPoseReady(void){return pose_ready;}
uint8_t HWT101_IsZeroDriftReady(void){return imu_ready;}
uint8_t ChassisMotor_IsEnabled(void){return enabled;}
HAL_StatusTypeDef ChassisMotor_Enable(void){CHECK(!irq_context);if(enable_result==HAL_OK)enabled=1;return enable_result;}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p){CHECK(h==&hfdcan2);p->BusOff=busoff;p->ErrorPassive=passive;return can_result;}
void ZDT_Console_Print(const char *text){CHECK(text!=0 && !irq_context);
 if(strstr(text,"[START KEY]"))key_logs++;
 if(strstr(text,"EXTI1 level="))edge_logs++;
 if(strstr(text,"stable HIGH/released"))release_logs++;
 if(strstr(text,"release duration="))press_logs++;
 if(strstr(text,"press ignored"))ignored_logs++;
}
static uint8_t zdt_start_zone,zdt_start_pose_set,zdt_script_running,zdt_poscal_phase,zdt_mileage_query_active,nav_active;
#define ZDT_POSCAL_IDLE 0U
static uint32_t bound_x,bound_y,bias_resets,rebase_calls;static uint8_t nose;
static HAL_StatusTypeDef rebase_result;
static const char *ZDT_SkipSpaces(const char *s){while(*s==' ')s++;return s;}
static uint8_t ZDT_Match_CommandWord(const char *s,const char *w){size_t n=strlen(w);return !strncmp(s,w,n) && (!s[n] || s[n]==' ');}
static uint8_t ZDT_Nav_IsActive(void){return nav_active;}
static void ZDT_Qr_Cancel(void){}
static void ScreenMetrics_ClearTask(void){}
static void ArmTask_RequestRound1Reset(void){}
static void ChassisMotor_ClearHeadingReference(void){}
void HWT101_ResetZeroDrift(void){bias_resets++;imu_ready=0;}
static HAL_StatusTypeDef ChassisMotor_RebaseHeading(float value){CHECK(value==0);rebase_calls++;return rebase_result;}
static void ZDT_Nav_SetNoseDir(uint8_t value){nose=value;}
static void GridNav_SetPoseMm(uint16_t x,uint16_t y){bound_x=x;bound_y=y;}
/* The runner inserts the unchanged production OK branch and PD1 dispatch. */
/* INSERT_PRODUCTION_DISPATCH */
static void setup(uint8_t high){
 uint8_t released;uint32_t changed_tick;
 match_ports_t ports={0};MatchControl_Init(&zdt_match,&ports);zdt_match.automatic=1;zdt_match.zone=1;
 tick=1000;pin_high=high;pose_ready=imu_ready=zdt_status.can_ready=1;enabled=busoff=passive=0;
 can_result=enable_result=rebase_result=HAL_OK;zdt_start_zone=zdt_start_pose_set=0;bound_x=111;bound_y=222;bias_resets=rebase_calls=0;nav_active=zdt_script_running=zdt_poscal_phase=zdt_mileage_query_active=0;zdt_qr_enabled=zdt_qr_locked=0;
 GPIO_StartButtonExti_Reset(&released,&changed_tick);StartButton_Init(&zdt_start_button,released,changed_tick);zdt_start_debounce_pending=1U;
}
static void sample(uint8_t high,uint32_t dt){uint8_t changed=(uint8_t)(pin_high!=high);pin_high=high;tick+=dt;if(changed)EXTI1_IRQHandler();ZDT_StartButton_Service();}
static void ready(void){zdt_match.state=MATCH_WAIT_START_OK;zdt_match.entered_tick=tick;}
static void gesture(uint32_t held){sample(0,1);sample(0,30);CHECK(!enabled);sample(1,held-30);sample(1,30);}
int main(void){
 unsigned i;uint32_t req;
 MX_GPIO_Init();CHECK(input_configs==1 && output_configs==1);
 {unsigned k,j;bt_button_event_t event;const char *commands[]={"&prepare","&OK 1","&OK 2"};const char *expected[]={"prepare","ok 1","ok 2"};
  for(k=0;k<3;k++){BT_Button_Init();for(j=0;commands[k][j];j++){BT_Button_ProcessByte((uint8_t)commands[k][j]);tick+=20;BT_Button_Poll();CHECK(!BT_Button_Pop(&event));}
   BT_Button_ProcessByte('\n');CHECK(BT_Button_Pop(&event) && event.id==BT_BUTTON_CUSTOM && !strcmp(event.text,expected[k]));}}

 setup(1);sample(1,30);CHECK(ZDT_Match_Command("prepare",tick));
 CHECK(zdt_match.state==MATCH_PREPARING && zdt_match.zone==0 && !zdt_start_pose_set);
 CHECK(bound_x==111 && bound_y==222 && bias_resets==1 && !rebase_calls);
 setup(1);sample(1,30);CHECK(ZDT_Match_Command("prepare 2",tick));CHECK(zdt_match.zone==0 && !zdt_start_pose_set && bound_y==222);
 setup(1);sample(1,30);ZDT_Match_Command("prepare",tick);req=zdt_match.active_request;
 tick+=3000;imu_ready=1;zdt_match.pending=0;MatchControl_Result(&zdt_match,req,1,tick);
 CHECK(zdt_match.state==MATCH_WAIT_START_OK && !zdt_start_pose_set && bound_y==222 && bias_resets==1);
 tick+=5000;sample(0,1);sample(0,30);sample(1,970);sample(1,30);
 CHECK(zdt_match.state==MATCH_LEAVING && zdt_match.zone==2 && bound_y==2250 && bias_resets==1);
 zdt_match.state=MATCH_WAIT_OK;zdt_match.checkpoint=MATCH_RAW1;zdt_match.entered_tick=tick;tick++;
 ZDT_Match_Command("ok 1",tick);CHECK(zdt_match.state==MATCH_MOVING && zdt_match.zone==2 && bound_y==2250 && rebase_calls==1);
 /* Boot-held release never launches; only a fresh debounced gesture does. */
 setup(0);ready();sample(0,2000);sample(1,1);sample(1,30);CHECK(!enabled);
 tick++;gesture(100);CHECK(zdt_match.state==MATCH_LEAVING && enabled && zdt_match.zone==1 && bound_y==150 && nose==1);
 for(i=0;i<3;i++){
  setup(1);sample(1,30);ready();gesture(i==0?999:i==1?1000:2000);
  CHECK(zdt_match.state==MATCH_LEAVING && zdt_match.zone==(i?2:1));
  CHECK(zdt_start_zone==(i?2:1) && bound_x==2250 && bound_y==(i?2250:150) && !bias_resets && rebase_calls==1);
  req=zdt_match.active_request;sample(1,100);CHECK(zdt_match.active_request==req);
 }
 /* An incomplete pulse/bounce cannot arm a launch. */
 setup(1);sample(1,30);ready();sample(0,1);sample(1,10);sample(1,30);CHECK(!enabled && !zdt_start_pose_set);
 gesture(100);CHECK(zdt_match.state==MATCH_LEAVING);
 /* Press before READY is stale even if released long after READY. */
 setup(1);sample(1,30);zdt_match.state=MATCH_PREPARING;sample(0,1);sample(0,30);ready();sample(1,1500);sample(1,30);
 CHECK(zdt_match.state==MATCH_WAIT_START_OK && !enabled && !zdt_start_pose_set);
 gesture(1000);CHECK(zdt_match.state==MATCH_LEAVING && zdt_match.zone==2);
 for(i=0;i<3;i++){
  setup(1);ready();tick++;CHECK(ZDT_Match_Command(i==0?"ok":i==1?"ok 1":"ok 2",tick));
  CHECK(zdt_match.state==MATCH_LEAVING && zdt_match.zone==(i==2?2:1) && !bias_resets);
 }
 setup(1);ready();tick++;ZDT_Match_Command("ok 3",tick);CHECK(!enabled && !zdt_start_pose_set);
 setup(1);ready();tick++;rebase_result=HAL_BUSY;ZDT_Match_Command("ok 2",tick);CHECK(zdt_match.state==MATCH_WAIT_START_OK && !zdt_start_pose_set && bound_y==222);
 for(i=0;i<7;i++){
  setup(1);sample(1,30);ready();
  if(i==0)pose_ready=0;else if(i==1)imu_ready=0;else if(i==2)zdt_status.can_ready=0;
  else if(i==3)busoff=1;else if(i==4)passive=1;else if(i==5)can_result=HAL_ERROR;else enable_result=HAL_ERROR;
  gesture(100);CHECK(zdt_match.state==MATCH_WAIT_START_OK && !zdt_start_pose_set);
 }
 {const match_state_t states[]={MATCH_IDLE,MATCH_MOVING,MATCH_FAULT,MATCH_FINISHED,MATCH_WAIT_OK};
  for(i=0;i<5;i++){setup(1);sample(1,30);zdt_match.state=states[i];gesture(100);CHECK(zdt_match.state==states[i] && !enabled);}}
 setup(1);sample(1,30);ready();req=pin_reads;pin_high=0;
 for(i=0;i<20;i++){tick+=100;ZDT_StartButton_Service();}CHECK(pin_reads==req && !enabled);
 setup(1);tick=0xffffffd0U;StartButton_Init(&zdt_start_button,1,tick);sample(1,30);ready();gesture(1000);
 CHECK(zdt_match.state==MATCH_LEAVING && zdt_match.zone==2);
 CHECK(irq_count && key_logs && edge_logs && release_logs && press_logs && ignored_logs);
 return test_failures;
}
