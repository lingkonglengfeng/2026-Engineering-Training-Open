/* Runs the real facade with simulated existing drivers. No board operations. */
#include "arm_driver.h"
#include "arm_speed_defaults.h"
#include "arm_calibration.h"
#include "arm_servo.h"
#include "tim.h"
#include "arm_task.h"
#include "fdcan.h"
#include "zdt_motor.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

/* No host I/O exists in the emulator; provide test-only syscall ports so the
 * newlib formatting/parser dependency does not link nosys warning stubs. */
int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; (void)st; return -1; }
int _getpid(void) { return 1; }
int _isatty(int fd) { (void)fd; return 0; }
int _kill(int pid, int signal) { (void)pid; (void)signal; return -1; }
off_t _lseek(int fd, off_t offset, int whence) { (void)fd; (void)offset; (void)whence; return -1; }
ssize_t _read(int fd, void *buffer, size_t count) { (void)fd; (void)buffer; (void)count; return -1; }
ssize_t _write(int fd, const void *buffer, size_t count) { (void)fd; (void)buffer; return (ssize_t)count; }

volatile unsigned int test_checks;
volatile unsigned int test_failures;
volatile unsigned int test_failure_lines[64];
static uint8_t test_start_pose_active,test_material_active,test_round_active,test_released_place;
static uint8_t test_tray_descent;
static uint8_t test_material_grab_reach,test_release_opening;
void ArmRound1_AbortState(void){test_round_active=0;}
uint8_t ArmRound1_IsActive(void){return test_round_active;}
uint8_t ArmRound1_Command(const char *line){(void)line;return 0;}
#define CHECK(x) do { ++test_checks; if (!(x)) { if (test_failures < 64U) { \
    test_failure_lines[test_failures] = __LINE__; } ++test_failures; } } while (0)
#define NEAR(x, y) (fabsf((x) - (y)) < 0.001f)

FDCAN_HandleTypeDef hfdcan2;
TIM_HandleTypeDef htim4;
static uint32_t pwm_starts;
static uint32_t pwm_stops;
static HAL_StatusTypeDef pwm_start_result = HAL_OK;
static HAL_StatusTypeDef pwm_stop_result = HAL_OK;

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer)
{
    CHECK(timer == &htim4);
    timer->Instance->PSC = timer->Init.Prescaler;
    timer->Instance->ARR = timer->Init.Period;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *oc, uint32_t channel)
{
    CHECK(timer == &htim4);
    CHECK((channel == TIM_CHANNEL_1) || (channel == TIM_CHANNEL_2));
    CHECK(oc->Pulse == 0U);
    __HAL_TIM_SET_COMPARE(timer, channel, oc->Pulse);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK(timer == &htim4);
    ++pwm_starts;
    if (pwm_start_result == HAL_OK)
    {
        timer->Instance->CCER |= (channel == TIM_CHANNEL_1) ? TIM_CCER_CC1E : TIM_CCER_CC2E;
        timer->Instance->CR1 |= TIM_CR1_CEN;
    }
    return pwm_start_result;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK(timer == &htim4);
    ++pwm_stops;
    if (pwm_stop_result == HAL_OK)
    {
        timer->Instance->CCER &= ~((channel == TIM_CHANNEL_1) ? TIM_CCER_CC1E : TIM_CCER_CC2E);
        if ((timer->Instance->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) == 0U) { timer->Instance->CR1 &= ~TIM_CR1_CEN; }
    }
    return pwm_stop_result;
}
static dm_motor_t motor;
static zdt_motor_status_t zdt;
static uint32_t tick = 1000U;
static uint32_t dm_snapshot_delay;
static uint32_t slots = 8U;
static uint8_t bus_off;
static uint8_t arm_ready = 1U;
static uint8_t arm_enabled;
static uint8_t arm_fault;
static HAL_StatusTypeDef move_result = HAL_OK;
static HAL_StatusTypeDef step_enable_result = HAL_OK;
static HAL_StatusTypeDef stop_lift_result = HAL_OK;
static HAL_StatusTypeDef stop_reach_result = HAL_OK,clear_tx_result=HAL_OK;
static HAL_StatusTypeDef dm_tx_result = HAL_OK;
static uint32_t moves;
static uint32_t stops5;
static uint32_t stops6;
static uint32_t disables5;
static uint32_t disables6;
static uint32_t dm_disables;
static uint32_t dm_stops;
static uint32_t dm_moves;
static uint32_t enable_calls;
static uint8_t last_address;
static uint8_t last_buffered;
static int32_t last_pulses;
static uint16_t last_speed;
static uint8_t last_acceleration;
static uint32_t speed_frames;
static uint32_t clear_frames;
static uint8_t speed_direction;
static uint16_t seek_speed;
static uint32_t reports;
static int16_t percent_motor_rpm[2];
static int8_t encoder_sign[2]={1,1};
static float encoder_gain[2]={1.0f,1.0f};
static int64_t simulated_counts[2];
/* Independent ID5 driver plant: ABS wire pulses use the motor's raw zero,
 * never the application's work zero or its logical direction setting. */
static uint8_t fake_lift_active,fake_lift_freeze,fake_lift_reached=1U;
static uint32_t native_lift_moves;
static int64_t fake_lift_target;
static int32_t last_lift_pulses;
static uint16_t fake_lift_speed;
static uint8_t fake_lift_absolute,last_lift_acceleration;
static uint8_t fake_lift_ack_enabled,fake_lift_ack_code=0x02U;
static uint8_t fake_lift_drop_commands,fake_lift_drop_acks;
static uint8_t fake_native_model,fake_native_active,fake_native_reached=1U,fake_native_ack_enabled=1U;
static uint8_t fake_native_ack_code=0x02U;
static uint8_t fake_native_reached_reply_only;
static uint8_t fake_native_drop_remaining;
#ifndef TEST_DEFAULT_ZERO
static uint8_t fake_native_no_reached_flag;
#endif
static int64_t fake_native_target;
static uint16_t fake_native_speed;
static float fake_native_scale=1.0f;
static int8_t native_encoder_sign=-1; /* Old fixture; replacement direction test uses +1. */
static uint8_t injected_reach_stall,auto_clear_reach_stall;
#ifndef TEST_DEFAULT_ZERO
static uint8_t reported_reach_stall; /* Flag-only injection; encoder plant still follows its real commands. */
#endif
static char percent_failure[90],percent_fault[256];
static char camera_status[192];
static char camera_failure_status[192],camera_failure_snapshot[192];
static HAL_StatusTypeDef velocity_result=HAL_OK;
static uint32_t velocity_calls;
static uint8_t saw_unknown_position;
static HAL_StatusTypeDef query_result=HAL_OK,save_result=HAL_OK,verify_result=HAL_OK;
static uint32_t query_calls,save_calls,verify_calls;
static uint32_t read_calls;
static char last_console[256];
static char camera_reach_fault_console[256];
static uint8_t test_camera_native_arrived;
static uint8_t fake_query_model,fake_query_drop_axis,fake_query_saw5;
static uint32_t fake_query_last5,fake_query_dropped;
static int64_t fake_query_position[2]={-3,-3};

void ZDT_Console_Print(const char *line)
{
    if(strstr(line,"[ARM CAMERA] NATIVE ARRIVED"))test_camera_native_arrived=1U;
    CHECK(line != NULL);
    (void)strncpy(last_console,line,sizeof(last_console)-1U);last_console[sizeof(last_console)-1U]='\0';
    if(strstr(line,"at_fault reach")){(void)strncpy(camera_reach_fault_console,line,sizeof(camera_reach_fault_console)-1U);camera_reach_fault_console[sizeof(camera_reach_fault_console)-1U]='\0';}
    if(strncmp(line,"percent ",8U)==0) {
        (void)strncpy(percent_failure,line,sizeof(percent_failure)-1U);
        percent_failure[sizeof(percent_failure)-1U]='\0';
    }
    if(strstr(line," fault dir=")!=NULL) {
        (void)strncpy(percent_fault,line,sizeof(percent_fault)-1U);
        percent_fault[sizeof(percent_fault)-1U]='\0';
    }
    if(strstr(line,"position=UNKNOWN")!=NULL)saw_unknown_position=1U;
    if(strncmp(line,"[ARM CAMERA] state=",19U)==0) {
        (void)strncpy(camera_status,line,sizeof(camera_status)-1U);camera_status[sizeof(camera_status)-1U]='\0';
    }
    if(strncmp(line,"[ARM CAMERA] last_failure ",26U)==0) {
        (void)strncpy(camera_failure_status,line,sizeof(camera_failure_status)-1U);
    }
    if(strstr(line,"[ARM CAMERA] at_fault DM=")!=NULL) {
        (void)strncpy(camera_failure_snapshot,line,sizeof(camera_failure_snapshot)-1U);
    }
    ++reports;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
                                               FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    CHECK(can == &hfdcan2);
    CHECK((header->Identifier == (5U << 8U)) || (header->Identifier == (6U << 8U)));
    CHECK(header->IdType == FDCAN_EXTENDED_ID);
    CHECK(header->FDFormat == FDCAN_CLASSIC_CAN);
    if (data[0] == 0xF6U)
    {
        ++speed_frames;
        speed_direction = data[1];
        seek_speed = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
        CHECK((data[5] == 0U) && (data[6] == 0x6BU));
        CHECK(header->DataLength == FDCAN_DLC_BYTES_7);
    }
    else if (data[0] == 0x0EU) {
        ++clear_frames;CHECK(data[1] == 0x52U);
        if(header->Identifier==(6U<<8U) && auto_clear_reach_stall && clear_tx_result==HAL_OK)injected_reach_stall=0;
        return clear_tx_result;
    }
    else {
        uint8_t id=(uint8_t)(header->Identifier>>8U);
        CHECK((data[0] == 0x3AU) || (data[0] == 0x36U));
        if(fake_query_model) {
            uint8_t reply[7]={data[0],0x83U,0x6BU,0,0,0,0};uint8_t length=3U;
            FDCAN_RxHeaderTypeDef rx={0};
            /* Reproduce the observed asymmetry: ID5 keeps replying while
             * an ID6 request in a tight ID5 burst loses its reply. */
            if(id==5U){fake_query_last5=tick;fake_query_saw5=1U;}
            if(id==fake_query_drop_axis || (id==6U && fake_query_saw5 && tick-fake_query_last5<10U))fake_query_dropped++;
            else {
                rx.Identifier=header->Identifier;rx.IdType=FDCAN_EXTENDED_ID;
                rx.RxFrameType=FDCAN_DATA_FRAME;rx.FDFormat=FDCAN_CLASSIC_CAN;
                if(data[0]==0x36U) {
                    int64_t pos=fake_query_position[id==5U?0U:1U];uint32_t mag=(uint32_t)(pos<0?-pos:pos);
                    length=7U;reply[1]=(uint8_t)(pos<0);reply[2]=(uint8_t)(mag>>24U);reply[3]=(uint8_t)(mag>>16U);
                    reply[4]=(uint8_t)(mag>>8U);reply[5]=(uint8_t)mag;reply[6]=0x6BU;
                }
                CHECK(ArmCalibration_PostCanFrame(&rx,reply,length)==HAL_OK);
            }
        }
    }
    return HAL_OK;
}

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *can,
                                            FDCAN_ProtocolStatusTypeDef *out)
{
    CHECK(can == &hfdcan2);
    out->BusOff = bus_off;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *can)
{
    CHECK(can == &hfdcan2);
    return slots;
}
const zdt_motor_status_t *ZDT_Motor_GetStatus(void) { return &zdt; }
HAL_StatusTypeDef ZDT_Motor_EnableAddr(uint8_t address, uint8_t buffered)
{
    ++enable_calls;
    CHECK((address == 5U) || (address == 6U));
    CHECK(buffered == 0U);
    return step_enable_result;
}
HAL_StatusTypeDef ZDT_Motor_DisableAddr(uint8_t address, uint8_t buffered)
{
    CHECK(buffered == 0U);
    if (address == 5U) { ++disables5; }
    else if (address == 6U) { ++disables6; }
    else { CHECK(0); }
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_StopAddr(uint8_t address)
{
    if (address == 5U) { ++stops5; if(stop_lift_result==HAL_OK){percent_motor_rpm[0]=0;fake_lift_active=0U;}return stop_lift_result; }
    if (address == 6U) { ++stops6;if(stop_reach_result==HAL_OK){percent_motor_rpm[1]=0;fake_native_active=0U;}return stop_reach_result; }
    CHECK(0);
    return HAL_ERROR;
}
HAL_StatusTypeDef ZDT_Motor_SetSpeedRpmAddr(uint8_t address,int16_t rpm,uint8_t buffered)
{
    CHECK(address==5U || address==6U);CHECK(buffered==0U);
    ++velocity_calls;
    if(velocity_result==HAL_OK)percent_motor_rpm[address==5U?0U:1U]=rpm;
    return velocity_result;
}
static HAL_StatusTypeDef Mock_MovePosition(uint8_t address, int32_t pulses,
                                                  uint16_t speed, uint8_t acceleration,
                                                  uint8_t buffered,uint8_t absolute)
{
    ++moves;
    last_address = address;
    last_pulses = pulses;
    last_speed = speed;
    last_acceleration = acceleration;
    last_buffered = buffered;
    CHECK((address == 5U) || (address == 6U));
    CHECK(buffered == 0U);
    if(address==5U && move_result==HAL_OK) {
        native_lift_moves++;last_lift_pulses=pulses;fake_lift_speed=speed;
        fake_lift_absolute=absolute;last_lift_acceleration=acceleration;
        fake_lift_target=(absolute?0:simulated_counts[0])+(int64_t)round((double)pulses*65536.0/3200.0);
        fake_lift_active=1U;fake_lift_reached=0U;percent_motor_rpm[0]=0;
        if(fake_lift_drop_commands){--fake_lift_drop_commands;fake_lift_active=0U;fake_lift_reached=1U;}
        else if(fake_lift_ack_enabled) {
            if(fake_lift_drop_acks)--fake_lift_drop_acks;
            else {
                FDCAN_RxHeaderTypeDef ack={0};uint8_t data[3]={0xFD,fake_lift_ack_code,0x6B};
                ack.Identifier=5U<<8;ack.IdType=FDCAN_EXTENDED_ID;ack.RxFrameType=FDCAN_DATA_FRAME;ack.FDFormat=FDCAN_CLASSIC_CAN;
                CHECK(ArmCalibration_PostCanFrame(&ack,data,3)==HAL_OK);
            }
        }
    }
    if(fake_native_model && address==6U && move_result==HAL_OK) {
        /* Independent replacement fixture: stock1.8deg/16 gives3200p/rev. */
        fake_native_target=(absolute?0:simulated_counts[1])+native_encoder_sign*(int64_t)round((double)pulses*65536.0/3200.0*fake_native_scale);
        fake_native_speed=speed;fake_native_active=1U;fake_native_reached=0U;
        if(fake_native_drop_remaining){--fake_native_drop_remaining;fake_native_active=0U;fake_native_reached=1U;}
        if(fake_native_ack_enabled && !fake_native_reached_reply_only) {
            FDCAN_RxHeaderTypeDef ack={0};uint8_t data[3]={0xFDU,fake_native_ack_code,0x6BU};
            ack.Identifier=6U<<8U;ack.IdType=FDCAN_EXTENDED_ID;ack.RxFrameType=FDCAN_DATA_FRAME;ack.FDFormat=FDCAN_CLASSIC_CAN;
            CHECK(ArmCalibration_PostCanFrame(&ack,data,3U)==HAL_OK);
        }
    }
    return move_result;
}
HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAddr(uint8_t address,int32_t pulses,uint16_t speed,uint8_t acceleration,uint8_t buffered)
{return Mock_MovePosition(address,pulses,speed,acceleration,buffered,0U);}
HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAbsAddr(uint8_t address,int32_t pulses,uint16_t speed,uint8_t acceleration,uint8_t buffered)
{return Mock_MovePosition(address,pulses,speed,acceleration,buffered,1U);}
uint8_t ArmTask_IsReady(void) { return arm_ready; }
uint8_t ArmTask_IsEnabled(void) { return arm_enabled; }
uint8_t ArmTask_IsFaulted(void) { return arm_fault; }
dm_motor_t *ArmTask_GetMotor(void) { return &motor; }
HAL_StatusTypeDef ArmTask_Enable(void)
{
    if(dm_tx_result!=HAL_OK)return dm_tx_result;
    arm_enabled = 1U;arm_fault=0U;
    motor.target_pos_rad = motor.fb.pos_rad;
    return HAL_OK;
}
HAL_StatusTypeDef ArmTask_ClearFault(void) { arm_fault = 0U; return HAL_OK; }
HAL_StatusTypeDef ArmTask_Disable(void)
{
    ++dm_disables;
    arm_enabled = 0U;
    motor.target_pos_rad = 0.0f;
    return HAL_OK;
}
void ArmTask_EmergencyStop(void)
{
    ++dm_stops;
    arm_fault = 1U;
    arm_enabled = 0U;
    motor.target_pos_rad = 0.0f;
}
HAL_StatusTypeDef ArmTask_MoveToRad(float position, float speed)
{
    ++dm_moves;
    motor.target_pos_rad = position + motor.pos_offset_rad;
    motor.target_vel_rads = speed;
    motor.last_err = (uint8_t)dm_tx_result;
    return HAL_OK; /* Existing API hides enqueue failures. */
}
void DM_GetFeedback(const dm_motor_t *m, dm_feedback_t *out) {
    if(dm_snapshot_delay){tick+=dm_snapshot_delay;motor.fb.last_tick=tick;motor.fb.fb_count++;}
    *out = m->fb;
}
HAL_StatusTypeDef DM_Motor_QueryDisabled(dm_motor_t *m,uint32_t timeout,dm_feedback_t *out)
{
    CHECK(m==&motor && timeout==300U);query_calls++;arm_enabled=0U;m->enable_sent=0U;
    if(query_result==HAL_OK) {
        m->fb.fb_count++;m->fb.last_tick=tick;m->fb.enabled=0U;m->fb.err_code=0U;
        if(out)*out=m->fb;
    }
    return query_result;
}
HAL_StatusTypeDef DM_Motor_SavePosZero(dm_motor_t *m)
{CHECK(m==&motor && !arm_enabled && !m->fb.enabled);save_calls++;return save_result;}
HAL_StatusTypeDef DM_Motor_VerifyPosZero(dm_motor_t *m,uint32_t timeout)
{
    CHECK(m==&motor && timeout==1000U);verify_calls++;
    if(verify_result==HAL_OK){m->fb.pos_rad=0.001f;m->fb.last_tick=tick;m->fb.fb_count++;}
    return verify_result;
}
HAL_StatusTypeDef DM_Motor_ReadPosition(dm_motor_t *m,uint32_t timeout,dm_feedback_t *out)
{
    CHECK(m==&motor && timeout==300U);read_calls++;
    m->fb.last_tick=tick;m->fb.fb_count++;m->fb.enabled=arm_enabled;
    *out=m->fb;return query_result;
}

static void Feedback(uint8_t id, int64_t position, uint8_t flags)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t f[3] = {0x3AU, flags, 0x6BU};
    uint32_t magnitude = (uint32_t)((position < 0) ? -position : position);
    uint8_t p[7] = {0x36U, (position < 0) ? 1U : 0U,
                   (uint8_t)(magnitude >> 24U), (uint8_t)(magnitude >> 16U),
                   (uint8_t)(magnitude >> 8U), (uint8_t)magnitude, 0x6BU};
    header.Identifier = (uint32_t)id << 8U;
    header.IdType = FDCAN_EXTENDED_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    CHECK(ArmCalibration_OnCanFrame(&header, f, 3U) == 1U);
    CHECK(ArmCalibration_OnCanFrame(&header, p, 7U) == 1U);
}

static void Sample(uint8_t id, int64_t position, uint8_t flags)
{
    tick += 100U;
    motor.fb.last_tick = tick;
    Feedback(id, position, flags);
    ArmCalibration_Poll();
}

static void TestServos(void)
{
    arm_servo_config_t config;
    arm_servo_config_t invalid;
    arm_servo_status_t grip = {0}, table = {0};
    uint32_t starts;
    uint32_t stop_count;
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP) == HAL_ERROR);
    CHECK(ArmServo_Init(NULL) == HAL_ERROR);
    ArmServo_DefaultConfig(&config);
    CHECK(config.table_slot_us[0] == 606U);
    CHECK(config.table_slot_us[1] == 1106U);
    CHECK(config.table_slot_us[2] == 1606U);
    CHECK(NEAR(config.table.angle_span_deg, 360.0f));
    CHECK(config.grip.angle_span_deg == 360.0f && config.grip.enable_us == 1750U);
    CHECK(config.grip.slew_us_per_second==2145.0f && config.table.slew_us_per_second==1072.5f);
    htim4.Instance = TIM4;
    htim4.State = HAL_TIM_STATE_READY;
    invalid = config; invalid.timer_clock_hz = 275000001U;
    CHECK(ArmServo_Init(&invalid) == HAL_ERROR);
    invalid = config; invalid.grip.slew_us_per_second = NAN;
    CHECK(ArmServo_Init(&invalid) == HAL_ERROR);
    invalid = config; invalid.table_slot_us[1] = 3000U;
    CHECK(ArmServo_Init(&invalid) == HAL_ERROR);
    CHECK(ArmServo_Init(&config) == HAL_OK);
    CHECK(pwm_starts == 0U);
    CHECK((TIM4->CCR1 == 0U) && (TIM4->CCR2 == 0U));
    CHECK((TIM4->PSC == 274U) && (TIM4->ARR == 19999U));
    CHECK(ArmServo_Init(&config) == HAL_BUSY);
    CHECK(ArmServo_TableGo(1U) == HAL_ERROR); /* never auto-enable */
    CHECK(ArmServo_Enable(ARM_SERVO_TABLE) == HAL_OK);
    CHECK(TIM4->CCR2 == 1106U); /* Material slot2=109deg is the first enable pulse. */
    CHECK(ArmServo_TableGo(1U) == HAL_OK);
    ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==606U);
    CHECK(ArmCalibration_ProcessCommand("arm table go 2")==1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==1106U);
    CHECK(ArmCalibration_ProcessCommand("arm table go 3")==1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==1606U);
    CHECK(ArmServo_TableGo(1U)==HAL_OK);
    tick += 10U; ArmServo_Update(tick);
    CHECK(TIM4->CCR2 == 1095U); /* 1072.5us/s ->10.725us/10ms. */
    CHECK(TIM4->CCR1 == 0U); /* independent channel */
    CHECK(ArmServo_TableCapture(1U) == HAL_ERROR); /* cannot save an unfinished target */
    tick += 1000U; ArmServo_Update(tick);
    CHECK(TIM4->CCR2 == 1042U); /* long scheduling gap capped at 50ms */
    CHECK(ArmServo_Stop(ARM_SERVO_TABLE) == HAL_OK);
    tick += 10U; ArmServo_Update(tick);
    CHECK(TIM4->CCR2 == 1042U);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_TABLE, 180.0f) == HAL_OK);
    ArmServo_GetStatus(ARM_SERVO_TABLE, &table);
    CHECK(table.target_us == 1500U);
    CHECK(ArmServo_MaterialSlotDeg(1U)==19.0f && ArmServo_MaterialSlotDeg(2U)==109.0f && ArmServo_MaterialSlotDeg(3U)==199.0f);
    CHECK(isnan(ArmServo_MaterialSlotDeg(0U)) && ArmServo_SelectMaterialSlot(4U)==HAL_ERROR);
    CHECK(ArmServo_SelectMaterialSlot(1U)==HAL_OK);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==606U);
    CHECK(ArmServo_SelectMaterialSlot(2U)==HAL_OK);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==1106U);
    CHECK(ArmServo_SelectMaterialSlot(3U)==HAL_OK);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==1606U);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_TABLE, 361.0f) == HAL_ERROR);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_TABLE, NAN) == HAL_ERROR);
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_TABLE, 2501U) == HAL_ERROR);
    CHECK(ArmServo_TableGo(4U) == HAL_ERROR);
    CHECK(ArmServo_GripOpen() == HAL_ERROR);
    CHECK(ArmServo_GripClose() == HAL_ERROR);
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP) == HAL_OK);
    CHECK(TIM4->CCR1 == 1750U);
    CHECK(ArmServo_GripOpen()==HAL_OK);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.target_us==1750U);
    CHECK(!ArmServo_GripIsClosed());
    CHECK(ArmServo_GripClose()==HAL_OK);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.target_us==2211U);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_GRIP, 90.0f) == HAL_OK);
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.target_us==1000U);
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);starts=pwm_starts;
    CHECK(ArmCalibration_ProcessCommand("arm grip test -1")==1U && pwm_starts==starts);
    CHECK(ArmCalibration_ProcessCommand("arm grip test nan")==1U && pwm_starts==starts);
    CHECK(ArmCalibration_ProcessCommand("arm grip test 361")==1U && pwm_starts==starts);
    CHECK(ArmCalibration_ProcessCommand("arm grip test 238")==1U && pwm_starts==starts+1U);
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.enabled && grip.target_us==1822U && TIM4->CCR1==1822U && !grip.ramping);
    CHECK(ArmCalibration_ProcessCommand("arm grip test 186")==1U);
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.target_us==1533U && grip.ramping);
    {unsigned k;for(k=0;k<60;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(TIM4->CCR1==1533U);
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP, 1550U) == HAL_OK);
    CHECK(ArmServo_GripCapture(0U) == HAL_ERROR);
    tick += 50U; ArmServo_Update(tick);
    tick += 50U; ArmServo_Update(tick);
    CHECK(TIM4->CCR1 == 1550U);
    CHECK(ArmServo_GripCapture(0U) == HAL_OK);
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP, 1650U) == HAL_OK);
    tick += 50U; ArmServo_Update(tick);
    tick += 50U; ArmServo_Update(tick);
    tick += 50U; ArmServo_Update(tick);
    tick += 50U; ArmServo_Update(tick);
    CHECK(ArmServo_GripCapture(1U) == HAL_OK);
    CHECK(ArmServo_GripOpen() == HAL_OK);
    ArmServo_GetStatus(ARM_SERVO_GRIP, &grip);
    CHECK(grip.target_us == 1550U);
    stop_count = pwm_stops;
    CHECK(ArmDriver_StopAll() == HAL_OK);
    CHECK(pwm_stops == stop_count); /* ordinary stop retains both holding pulses */
    ArmServo_GetStatus(ARM_SERVO_GRIP, &grip);
    CHECK(grip.target_us == 1650U);
    CHECK(ArmCalibration_ProcessCommand("arm table go 2") == 1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE, &table);
    CHECK(table.target_us == 1106U);
    CHECK(ArmCalibration_ProcessCommand("arm table angle 90") == 1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE, &table);
    CHECK(table.target_us == 1000U);
    CHECK(ArmCalibration_ProcessCommand("arm table pulse 65536") == 1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE, &table);
    CHECK(table.target_us == 1000U); /* parser must reject before integer narrowing */
    CHECK(ArmServo_SetAngleSpan(ARM_SERVO_GRIP, 270.0f) == HAL_ERROR); /* disable before remapping */
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP) == HAL_OK);
    CHECK(ArmServo_SetAngleSpan(ARM_SERVO_GRIP, 270.0f) == HAL_OK);
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP) == HAL_OK);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_GRIP, 135.0f) == HAL_OK);
    ArmServo_GetStatus(ARM_SERVO_GRIP, &grip);
    CHECK(grip.target_us == 1500U);
    starts = pwm_starts;
    CHECK(ArmCalibration_ProcessCommand("arm table test 385")==1U);
    ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==2639U && table.ramping);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_TABLE,386.0f)==HAL_ERROR);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,385.0f)==HAL_ERROR);
    CHECK(ArmServo_SetAngleDeg(ARM_SERVO_TABLE,385.0f)==HAL_ERROR);
    {unsigned k;for(k=0;k<400;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(TIM4->CCR2==2639U && ArmServo_TableCapture(1U)==HAL_ERROR);
    CHECK(ArmServo_TableGo(2U)==HAL_OK);ArmServo_GetStatus(ARM_SERVO_TABLE,&table);CHECK(table.target_us==1106U);
    {unsigned k;for(k=0;k<400;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(TIM4->CCR2==1106U);
    test_material_active=1U;
    CHECK(ArmCalibration_ProcessCommand("arm grip disable")==1U && !test_material_active);
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(!grip.enabled);
    test_round_active=1U;starts=pwm_starts;
    CHECK(ArmCalibration_ProcessCommand("arm grip test 305")==1U && test_round_active && pwm_starts==starts);
    CHECK(ArmCalibration_ProcessCommand("arm status")==1U && test_round_active);
    CHECK(ArmCalibration_ProcessCommand("arm grip disable")==1U && !test_round_active);
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP)==HAL_OK);starts=pwm_starts;
    {
        const char *const release[]={"arm disable 5","arm disable 6","arm disable dm","dm off","dm disable"};
        unsigned i;
        for(i=0U;i<sizeof(release)/sizeof(release[0]);i++) {
            uint32_t before5=stops5,before6=stops6;
            test_material_active=1U;fake_native_active=1U;
            CHECK(ArmCalibration_ProcessCommand(release[i])==1U && !test_material_active);
            CHECK(stops5>before5 && stops6>before6 && !fake_native_active);
        }
    }

    /* Reuse an OPEN command after50ms, including target changes
     * between update ticks and disable/re-enable at an unchanged pulse. */
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,225.0f)==HAL_OK);
    {unsigned k;for(k=0;k<200;k++){tick+=10;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);if(!grip.ramping)break;}CHECK(k>=20 && k<200);}
    CHECK(ArmServo_GripCapture(0U)==HAL_OK); /* Reuse the explicitly taught OPEN, not a hardcoded angle. */
    tick+=500U;ArmServo_Update(tick); /* OPEN reuse retains its existing settling minimum. */
    CHECK(ArmServo_GripOpenConfirmed());
    ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP,1650)==HAL_OK);
    CHECK(!ArmServo_GripOpenConfirmed());
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP,grip.target_us)==HAL_OK && !ArmServo_GripOpenConfirmed());
    tick+=ARM_SERVO_GRIP_OPEN_MIN_MS-1;ArmServo_Update(tick);CHECK(!ArmServo_GripOpenConfirmed());
    tick+=1;ArmServo_Update(tick);CHECK(ArmServo_GripOpenConfirmed());
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK && !ArmServo_GripOpenConfirmed());
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP)==HAL_OK && !ArmServo_GripOpenConfirmed());
    tick+=1;ArmServo_Update(tick);tick+=500;ArmServo_Update(tick);CHECK(ArmServo_GripOpenConfirmed());
    /* At2145us/s308deg close is incomplete210ms and completes by220ms. */
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);
    CHECK(ArmServo_SetAngleSpan(ARM_SERVO_GRIP,360.0f)==HAL_OK);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG)==HAL_OK && ArmServo_GripCapture(1U)==HAL_OK);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,225.0f)==HAL_OK);
    {unsigned k;for(k=0;k<45;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(!ArmServo_GripIsClosed());
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG)==HAL_OK);
    {unsigned k;for(k=0;k<20;k++){tick+=10;ArmServo_Update(tick);}CHECK(!ArmServo_GripIsClosed());}
    tick+=10;ArmServo_Update(tick);CHECK(!ArmServo_GripIsClosed());
    tick+=10;ArmServo_Update(tick);CHECK(ArmServo_GripIsClosed());
    /* Optional close75% only:1608.75us/s, moving280ms, complete290ms.
     * A concurrent tray command must not reset this grip-only profile. */
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,225.0f)==HAL_OK);
    {unsigned k;for(k=0;k<31;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(ArmServo_GripCloseScaled(0.75f)==HAL_OK);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_TABLE,160.0f)==HAL_OK);
    tick+=10;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    CHECK(fabsf(grip.output_us-1766.0875f)<0.1f && grip.ramping);
    CHECK(ArmServo_GripCloseScaled(NAN)==HAL_ERROR && ArmServo_GripCloseScaled(0)==HAL_ERROR && ArmServo_GripCloseScaled(1.1f)==HAL_ERROR);
    {unsigned k;for(k=0;k<27;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(!ArmServo_GripIsClosed());tick+=10;ArmServo_Update(tick);CHECK(ArmServo_GripIsClosed());
    /* Normal open/close commands restore configured speed automatically. */
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,225.0f)==HAL_OK);
    tick+=10;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(fabsf(grip.output_us-2189.55f)<0.1f);
    {unsigned k;for(k=0;k<29;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG)==HAL_OK);
    {unsigned k;for(k=0;k<30;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(ArmServo_GripIsClosed());
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,225.0f)==HAL_OK);
    {float parameters[5],changed[5];arm_servo_status_t before;
     ArmServo_GetStatus(ARM_SERVO_GRIP,&before);CHECK(ArmServo_GetUiParameters(parameters)==HAL_OK);
     memcpy(changed,parameters,sizeof(changed));changed[0]=230;changed[1]=300;changed[2]=200;
     CHECK(ArmServo_SetUiParameters(changed)==HAL_OK && ArmServo_GripOpenDeg()==230 && ArmServo_GripCloseDeg()==300 && ArmServo_MaterialSlotDeg(1)==200);
     ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.target_us==before.target_us && grip.output_us==before.output_us);
     changed[4]=NAN;CHECK(ArmServo_SetUiParameters(changed)==HAL_ERROR);
     CHECK(ArmServo_GetUiParameters(changed)==HAL_OK && changed[0]==230 && changed[4]==parameters[4]);
     CHECK(ArmServo_SetUiParameters(parameters)==HAL_OK);}
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP)==HAL_OK);
    /* OPEN advances at49/50ms even while the real output keeps ramping.
     * Stable-open and stable-closed predicates retain their old meanings. */
    {const float defaults[5]={225,308,19,109,199};CHECK(ArmServo_SetUiParameters(defaults)==HAL_OK);}
    CHECK(ArmServo_SetSlew(ARM_SERVO_GRIP,2145.0f)==HAL_OK);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,ARM_SERVO_GRIP_CLOSE_DEG)==HAL_OK);
    {unsigned k;for(k=0;k<50;k++){tick+=10;ArmServo_Update(tick);}}
    CHECK(ArmServo_GripIsClosed());
    CHECK(ArmServo_GripOpen()==HAL_OK);
    tick+=49;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    CHECK(!ArmServo_GripOpenConfirmed() && grip.ramping);
    tick+=1;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    CHECK(ArmServo_GripOpenConfirmed() && grip.ramping && !ArmServo_GripIsOpen() && !ArmServo_GripIsClosed());
    {float moving=grip.output_us;tick+=10;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(grip.output_us!=moving && grip.ramping);}
    starts=pwm_starts;ArmServo_RequestSafeStop();CHECK(!ArmServo_GripOpenConfirmed());
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_TABLE, 1500U) == HAL_ERROR); /* pending ISR request blocks new motion */
    tick += 10U; ArmServo_Update(tick);
    CHECK((TIM4->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) == 0U);
    CHECK(ArmServo_Enable(ARM_SERVO_TABLE) == HAL_ERROR);
    CHECK(ArmServo_ClearSafeStop() == HAL_OK);
    CHECK(pwm_starts == starts); /* recover never automatically re-enables */
    pwm_start_result = HAL_ERROR;
    CHECK(ArmServo_Enable(ARM_SERVO_TABLE) == HAL_ERROR);
    ArmServo_GetStatus(ARM_SERVO_TABLE, &table);
    CHECK((table.enabled == 0U) && (table.fault == 1U));
    CHECK(ArmServo_Disable(ARM_SERVO_TABLE) == HAL_OK);
    pwm_start_result = HAL_OK;
    CHECK(ArmServo_Enable(ARM_SERVO_TABLE) == HAL_OK);
    pwm_stop_result = HAL_ERROR;
    CHECK(ArmServo_Disable(ARM_SERVO_TABLE) == HAL_ERROR);
    CHECK(ArmServo_SetPulseUs(ARM_SERVO_TABLE, 1500U) == HAL_ERROR);
    pwm_stop_result = HAL_OK;
    CHECK(ArmServo_Disable(ARM_SERVO_TABLE) == HAL_OK);
    /* The running PWM job captures its rate. A changed context or SAME target
     * resend cannot change it; a genuinely new target captures the new rate. */
    CHECK(ArmServo_SetAngleSpan(ARM_SERVO_GRIP,360)==HAL_OK);
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);
    CHECK(ArmServo_TestAngleDeg(ARM_SERVO_GRIP,90)==HAL_OK);
    CHECK(ArmServo_SetSlew(ARM_SERVO_GRIP,1000)==HAL_OK);
    tick+=10;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);
    {float initial=grip.output_us;
     test_arm_rate_factor[ARM_SPEED_GRIP]=.5f;CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP,2000)==HAL_OK);
     tick+=50;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(NEAR(grip.output_us-initial,25));
     test_arm_rate_factor[ARM_SPEED_GRIP]=2.0f;CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP,2000)==HAL_OK);
     tick+=50;ArmServo_Update(tick);ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(NEAR(grip.output_us-initial,50));
     CHECK(ArmServo_SetPulseUs(ARM_SERVO_GRIP,2100)==HAL_OK);tick+=50;ArmServo_Update(tick);
     ArmServo_GetStatus(ARM_SERVO_GRIP,&grip);CHECK(NEAR(grip.output_us-initial,150));}
    test_arm_rate_factor[ARM_SPEED_GRIP]=1.0f;
    CHECK(ArmServo_SetSlew(ARM_SERVO_GRIP,2145)==HAL_OK);CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);
}

static void LiftPlantStep(uint32_t ms)
{
    if(fake_lift_active && !fake_lift_freeze) {
        int64_t step=(int64_t)round((double)fake_lift_speed*65536.0*ms/60000.0);
        int64_t remaining=fake_lift_target-simulated_counts[0];
        if(llabs(remaining)<=step) {
            simulated_counts[0]=fake_lift_target;fake_lift_active=0U;fake_lift_reached=1U;
        } else simulated_counts[0]+=remaining>0?step:-step;
    }
}
static void PercentStep(void)
{
    unsigned int i;
    tick+=100U;motor.fb.last_tick=tick;
    for(i=0;i<2;i++)simulated_counts[i]+=(int64_t)round((double)percent_motor_rpm[i]*encoder_sign[i]*encoder_gain[i]*65536.0/600.0);
    LiftPlantStep(100U);
    Feedback(5U,simulated_counts[0],fake_lift_reached?0x83U:0x81U);Feedback(6U,simulated_counts[1],0x81U);
    ArmCalibration_Poll();
}
static void PercentArrival(arm_driver_axis_t axis,float target)
{
    unsigned int i;
    float percent;
    CHECK(ArmCalibration_MovePercent(axis,target)==HAL_OK);
    CHECK(ArmCalibration_IsLiftRaisingToTop()==(axis==ARM_DRIVER_LIFT && target==0.0f));
    CHECK(ArmCalibration_MovePercent(axis,target)==HAL_OK); /* same active target is not additive */
    CHECK(ArmCalibration_MovePercent(axis,target==50.0f?60.0f:50.0f)==HAL_BUSY);
    for(i=0;i<140 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++)PercentStep();
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(!ArmCalibration_IsLiftRaisingToTop());
    CHECK(ArmCalibration_GetPercent(axis,&percent)==HAL_OK);
    CHECK(fabsf(percent-target)<=(axis==ARM_DRIVER_LIFT?ArmCalibration_LiftTolerancePercent(target,ARM_LIFT_FULL_TRAVEL_MM):ARM_PERCENT_TOLERANCE));
    CHECK(percent_motor_rpm[0]==0 && percent_motor_rpm[1]==0);
}
static void TestPercent(void)
{
    float percent;
    uint32_t calls;
    arm_enabled=0;arm_fault=0;motor.fb.enabled=0;motor.fb.pos_rad=0;motor.fb.vel_rads=0;
    motor.target_pos_rad=0;motor.fb.last_tick=tick;
    CHECK(ArmCalibration_ProcessCommand("arm span 5 -219505")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm span 6 -132619.377778")==1U);
    CHECK(ArmCalibration_SetTravelMm(ARM_DRIVER_LIFT,240.0f)==HAL_OK);
    CHECK(ArmCalibration_SetTravelMm(ARM_DRIVER_REACH,178.0f)==HAL_OK);
    CHECK(ArmCalibration_UseTopGuard(100.0f,-20.0f,20.0f)==HAL_OK);
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_LIFT,0U)==HAL_OK);
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_REACH,1U)==HAL_OK);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,60U)==HAL_OK);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,60U)==HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm top 5")==1U);
    Sample(5U,0,0x81U);Sample(5U,0,0x81U);Sample(5U,0,0x81U);
    CHECK(ArmCalibration_ProcessCommand("arm zero 6")==1U);
    Sample(6U,0,0x81U);Sample(6U,0,0x81U);Sample(6U,0,0x81U);
    simulated_counts[0]=0;simulated_counts[1]=0;PercentStep();
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && NEAR(percent,0));
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,-1)==HAL_ERROR);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,101)==HAL_ERROR);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,NAN)==HAL_ERROR);
    PercentArrival(ARM_DRIVER_LIFT,50.0f);
    calls=velocity_calls;PercentArrival(ARM_DRIVER_LIFT,50.0f);
    CHECK(velocity_calls==calls); /* repeat after arrival sends no motion */
    PercentArrival(ARM_DRIVER_LIFT,0.0f);
    PercentArrival(ARM_DRIVER_REACH,50.0f);
    PercentArrival(ARM_DRIVER_REACH,100.0f);
    PercentArrival(ARM_DRIVER_REACH,0.0f);
    /* Target below 100mm must respect the rotation sector before moving. */
    motor.fb.pos_rad=-30.0f*1.5f*0.01745329252f;motor.fb.last_tick=tick;
    calls=velocity_calls;CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,50)==HAL_OK);
    PercentStep();CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && velocity_calls==calls);
    motor.fb.pos_rad=0;motor.fb.last_tick=tick;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_OK);
    PercentStep();CHECK(percent_motor_rpm[1]<0);
    CHECK(ArmCalibration_ProcessCommand("arm stop")==1U);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && percent_motor_rpm[1]==0);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_OK);
    PercentStep();tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_ERROR);
    CHECK(ArmCalibration_ProcessCommand("arm status")==1U && saw_unknown_position);
    PercentStep();
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_OK);
    velocity_result=HAL_ERROR;PercentStep();velocity_result=HAL_OK;
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    /* Wrong configured direction is diagnosed from real position feedback. */
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_REACH,0U)==HAL_OK);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_OK);PercentStep();PercentStep();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_REACH,1U)==HAL_OK);
    /* Percent parser and explicit disable cancel the owner transaction. */
    PercentStep();CHECK(ArmCalibration_ProcessCommand("arm pct 6 25")==1U);
    CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE);
    PercentStep();CHECK(ArmCalibration_ProcessCommand("arm disable 6")==1U);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && percent_motor_rpm[1]==0);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_ERROR);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    PercentStep();CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50)==HAL_OK);
    {
        unsigned int i;
        for(i=0;i<200 && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++) {
            tick+=100U;motor.fb.last_tick=tick;
            Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,simulated_counts[1],0x81U);
            ArmCalibration_Poll(); /* Fresh feedback, but a mechanically stuck axis */
        }
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
}

static void TestReachDirectionAndFault(void)
{
    const uint8_t flags[4]={0x85U,0x89U,0x80U,0x81U};
    const char *reasons[4]={"percent motor stalled","percent motor protected",
        "percent motor disabled","percent enable request cleared"};
    uint8_t direction=255U;
    unsigned int i;
    CHECK(ArmDriver_GetLinearDirection(ARM_DRIVER_ROTATION,&direction)==HAL_ERROR);
    CHECK(ArmDriver_GetLinearDirection(ARM_DRIVER_REACH,NULL)==HAL_ERROR);
    /* Opposite physical motor/encoder binding reported by the user. Keep the
     * calibrated negative encoder span; reverse only the outgoing command. */
    encoder_sign[1]=-1;
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 0")==1U);
    CHECK(ArmDriver_GetLinearDirection(ARM_DRIVER_REACH,&direction)==HAL_OK && direction==0U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    simulated_counts[1]=0;PercentStep();
    PercentArrival(ARM_DRIVER_REACH,41.18f);
    CHECK(simulated_counts[1]<0);
    PercentArrival(ARM_DRIVER_REACH,0.0f);
    /* The previous boot binding now runs away and must still stop. */
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 1")==1U);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50.0f)==HAL_OK);
    PercentStep();PercentStep();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
    CHECK(strcmp(percent_failure,"percent direction mismatch; check arm dir")==0);
    CHECK(strstr(percent_fault,"id=6 fault dir=1 flags=0x81")!=NULL);
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 0")==1U);
    for(i=0U;i<4U;i++) {
        CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
        PercentStep();
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,50.0f)==HAL_OK);
        if(i==3U)CHECK(ArmDriver_Disable(ARM_DRIVER_REACH)==HAL_OK);
        tick+=10U;
        Feedback(6U,simulated_counts[1],flags[i]);
        ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && percent_motor_rpm[1]==0);
        CHECK(strcmp(percent_failure,reasons[i])==0);
        CHECK(strstr(percent_fault,"id=6 fault dir=0")!=NULL);
        if(i==0U)CHECK(strstr(percent_fault,"stall=1 protected=0")!=NULL);
        if(i==1U)CHECK(strstr(percent_fault,"stall=0 protected=1")!=NULL);
        if(i==2U)CHECK(strstr(percent_fault,"enabled=0 enable_req=1")!=NULL);
        if(i==3U)CHECK(strstr(percent_fault,"enabled=1 enable_req=0")!=NULL);
    }
    encoder_sign[1]=1;
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 1")==1U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    PercentStep();
}

#ifndef TEST_DEFAULT_ZERO
static void CameraSetup(void)
{
    dm_snapshot_delay=0U;
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm camera set 0 -300 0")==1U); /* Long-stroke regression fixture. */
    CHECK(ArmCalibration_SetCameraPositionMode(0U)==HAL_OK);
    fake_native_model=fake_native_active=0U;fake_native_reached=1U;fake_native_ack_enabled=1U;fake_native_ack_code=0x02U;fake_native_scale=1.0f;
    fake_native_reached_reply_only=0U;
    injected_reach_stall=auto_clear_reach_stall=0U;
    fake_native_no_reached_flag=0U;fake_native_drop_remaining=0U;
    fake_query_model=0U;encoder_sign[0]=1;encoder_sign[1]=-1;
    fake_lift_active=fake_lift_freeze=0U;fake_lift_reached=1U;
    fake_lift_ack_enabled=1U;fake_lift_ack_code=0x02U;fake_lift_drop_commands=fake_lift_drop_acks=0U;
    encoder_gain[0]=encoder_gain[1]=1.0f;
    simulated_counts[0]=simulated_counts[1]=0;
    arm_fault=0U;arm_enabled=0U;motor.active_mode=DM_POS_MODE;
    motor.fb.pos_rad=0;motor.fb.vel_rads=0;motor.fb.enabled=0;motor.fb.err_code=0;
    motor.fb.last_tick=tick;motor.fb.fb_count++;motor.pos_offset_rad=0;
    CHECK(ArmDriver_SetRotationZeroHere()==HAL_OK);
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_LIFT,0U)==HAL_OK);
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_REACH,0U)==HAL_OK);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,60U)==HAL_OK);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,60U)==HAL_OK);
    CHECK(ArmCalibration_UseTopGuard(100.0f,-20.0f,20.0f)==HAL_OK);
    Feedback(5U,0,0x80U);Feedback(6U,0,0x80U);ArmCalibration_Poll();
}
static void CameraStep(uint8_t dm_follows_target)
{
    unsigned int i;
    tick+=100U;
    for(i=0U;i<2U;i++)simulated_counts[i]+=(int64_t)round((double)percent_motor_rpm[i]*encoder_sign[i]*encoder_gain[i]*65536.0/600.0);
    LiftPlantStep(100U);
    if(fake_native_model && fake_native_active && !injected_reach_stall) {
        int64_t step=(int64_t)round((double)fake_native_speed*65536.0/600.0);
        int64_t remaining=fake_native_target-simulated_counts[1];
        if(llabs(remaining)<=step) {
            simulated_counts[1]=fake_native_target;fake_native_active=0U;fake_native_reached=1U;
            if(fake_native_ack_enabled && fake_native_reached_reply_only) {
                FDCAN_RxHeaderTypeDef ack={0};uint8_t data[3]={0xFDU,0x9FU,0x6BU};
                ack.Identifier=6U<<8U;ack.IdType=FDCAN_EXTENDED_ID;ack.RxFrameType=FDCAN_DATA_FRAME;ack.FDFormat=FDCAN_CLASSIC_CAN;
                CHECK(ArmCalibration_PostCanFrame(&ack,data,3U)==HAL_OK);
            }
        }
        else simulated_counts[1]+=remaining>0?step:-step;
    }
    if(dm_follows_target)motor.fb.pos_rad=motor.target_pos_rad;
    motor.fb.last_tick=tick;motor.fb.fb_count++;motor.fb.enabled=arm_enabled;motor.fb.err_code=arm_enabled?1U:0U;
    Feedback(5U,simulated_counts[0],ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT)?(fake_lift_reached?0x83U:0x81U):0x80U);
    Feedback(6U,simulated_counts[1],ArmDriver_IsEnableRequested(ARM_DRIVER_REACH)?
        ((injected_reach_stall || reported_reach_stall)?0x85U:(fake_native_model && fake_native_reached && !fake_native_no_reached_flag?0x83U:0x81U)):0x80U);
    ArmCalibration_Poll();
}
static void CameraAdvanceTo(arm_camera_state_t stage)
{
    unsigned int i;
    for(i=0U;i<120U && ArmCalibration_GetCameraState()!=stage &&
        ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==stage);
}
static void TestCameraDeploy(void)
{
    unsigned int i;
    uint32_t enables,saves,queries,velocities,dm_count;
    float percent;
    arm_cal_axis_status_t lift_before,reach_before,lift_after,reach_after;
    CameraSetup();
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift_before);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach_before);
    saves=save_calls;enables=enable_calls;dm_count=dm_moves;
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    CHECK(ArmCalibration_PostCommand("arm camera")==HAL_OK);
    ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ENABLE && !arm_enabled);
    CHECK(enable_calls==enables+1U && dm_moves==dm_count);enables=enable_calls;
    queries=query_calls;velocities=velocity_calls;dm_count=dm_moves;
    CHECK(ArmCalibration_ProcessCommand("arm camera")==1U); /* duplicate does not re-home */
    CHECK(query_calls==queries && dm_moves==dm_count && enable_calls==enables);
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1U);
    CHECK(strstr(camera_status,"HAL=0")!=NULL);
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 1")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm pulse 6 100")==1U);
    CHECK(ArmCalibration_ProcessCommand("dm home")==1U);
    CHECK(velocity_calls==velocities && dm_moves==dm_count);
    /* Initial lift enable needs fresh status; DM/reach cannot move yet. */
    ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ENABLE && dm_moves==dm_count);
    CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    /* Reusing a cached zero feedback cannot advance the DM step. */
    motor.fb.enabled=1U;motor.fb.err_code=1U;
    for(i=0U;i<3U;i++) {
        tick+=50U;motor.fb.last_tick=tick;
        Feedback(5U,0,0x81U);Feedback(6U,0,0x80U);ArmCalibration_Poll();
    }
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && enable_calls==enables);
    CameraAdvanceTo(ARM_CAMERA_REACH_ENABLE);
    /* An old disabled reach status cannot count as successful enabling. */
    ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_ENABLE);
    CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(simulated_counts[0]==0 && save_calls==saves);
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK);
    CHECK(fabs((double)simulated_counts[1]*360.0/65536.0+300.0)<4.0);
    CHECK(fabsf(percent-41.181766f)<=ARM_PERCENT_TOLERANCE);
    CHECK(percent_motor_rpm[0]==0 && percent_motor_rpm[1]==0 && arm_enabled);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&lift_after);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&reach_after);
    CHECK(lift_after.zero_counts==lift_before.zero_counts && reach_after.zero_counts==reach_before.zero_counts && save_calls==saves);
    /* Deep lift and off-centre DM: raise first without homing/enabling DM.
     * This previously failed the low-height angular guard before lifting. */
    CameraSetup();simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.6);
    CHECK(ArmDriver_Disable(ARM_DRIVER_REACH)==HAL_OK);
    motor.fb.pos_rad=-90.0f*1.5f*0.01745329252f;
    Feedback(5U,simulated_counts[0],0x80U);dm_count=dm_moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    {
        int peak=0;
        for(i=0U;i<120U && ArmCalibration_GetCameraState()!=ARM_CAMERA_DM_HOME &&
            ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
            CameraStep(0U);if(abs(percent_motor_rpm[0])>peak)peak=abs(percent_motor_rpm[0]);
            CHECK(simulated_counts[1]==0 && !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH));
            if(ArmCalibration_GetCameraState()!=ARM_CAMERA_DM_HOME)CHECK(dm_moves==dm_count && !arm_enabled);
        }
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME);
        CHECK(peak>=59 && peak<=60); /* Integer RPM and measured-speed compensation may round down. */
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && fabsf(percent)<=ArmCalibration_LiftTolerancePercent(0.0f,ARM_LIFT_FULL_TRAVEL_MM));
        CHECK(dm_moves==dm_count+1U && arm_enabled);CameraAdvanceTo(ARM_CAMERA_DONE);
    }
    /* If lift feedback is below top, it must return before reach can extend. */
    CameraSetup();simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.1);
    Feedback(5U,simulated_counts[0],0x80U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);
    CHECK(simulated_counts[1]==0);
    CameraAdvanceTo(ARM_CAMERA_REACH_ENABLE);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && fabsf(percent)<=ArmCalibration_LiftTolerancePercent(0.0f,ARM_LIFT_FULL_TRAVEL_MM));
    CHECK(simulated_counts[1]==0);CameraAdvanceTo(ARM_CAMERA_DONE);
    /* Stop cancels every stage and cannot re-enable on the following tick. */
    for(i=ARM_CAMERA_PRECHECK;i<=ARM_CAMERA_REACH_MOVE;i++) {
        CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
        CameraAdvanceTo((arm_camera_state_t)i);
        CHECK(ArmCalibration_ProcessCommand("arm stop")==1U);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && percent_motor_rpm[0]==0 && percent_motor_rpm[1]==0);
        enables=enable_calls;CameraStep(1U);
        CHECK(enable_calls==enables && ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    }
    /* A busy manual percent move is not replaced by deployment. */
    CameraSetup();CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CameraStep(1U);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,20.0f)==HAL_OK);
    CHECK(ArmCalibration_DeployCamera()==HAL_BUSY);CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* Lost initial feedback never causes an enable or move. */
    CameraSetup();tick+=DM_LINK_TIMEOUT_MS_DEFAULT+1U;enables=enable_calls;dm_count=dm_moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);tick+=2001U;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && enable_calls==enables && dm_moves==dm_count);
    /* A new protection fault during DM homing also cancels immediately. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);ArmCalibration_Poll();
    motor.fb.enabled=1U;motor.fb.err_code=1U;motor.fb.fb_count++;motor.fb.last_tick=tick;
    Feedback(5U,0,0x89U);Feedback(6U,0,0x80U);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
    /* DM must really arrive; ZDT6 stays stopped if DM never reaches zero. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);ArmCalibration_Poll();
    motor.fb.pos_rad=-0.5f;enables=enable_calls;
    for(i=0U;i<105U && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(0U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && enable_calls==enables && percent_motor_rpm[1]==0);
    /* Link loss and protection stop the sequence instead of publishing READY. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    motor.fb.last_tick=tick-DM_LINK_TIMEOUT_MS_DEFAULT-1U;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    tick+=100U;motor.fb.last_tick=tick;motor.fb.fb_count++;
    Feedback(5U,0,0x81U);Feedback(6U,simulated_counts[1],0x89U);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    /* Explicit disable also cancels, including before any percent move. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm disable 6")==1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH));
    encoder_sign[1]=1;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void CameraHandoffSample(uint32_t ms,uint8_t flags,uint8_t new_status,int64_t position)
{
    FDCAN_RxHeaderTypeDef h={0};uint8_t f[3]={0x3AU,flags,0x6BU};
    uint32_t n=(uint32_t)(position<0?-position:position);
    uint8_t data[7]={0x36U,position<0?1U:0U,(uint8_t)(n>>24),(uint8_t)(n>>16),(uint8_t)(n>>8),(uint8_t)n,0x6BU};
    tick+=ms;simulated_counts[1]=position;motor.fb.last_tick=tick;motor.fb.fb_count++;motor.fb.enabled=arm_enabled;motor.fb.err_code=1;
    Feedback(5U,simulated_counts[0],0x81U);
    h.Identifier=6U<<8;h.IdType=FDCAN_EXTENDED_ID;h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;
    if(new_status)CHECK(ArmCalibration_OnCanFrame(&h,f,3)==1);
    CHECK(ArmCalibration_OnCanFrame(&h,data,7)==1);ArmCalibration_Poll();
}
static void TestCameraHandoffStall(void)
{
    static const arm_camera_state_t stages[]={ARM_CAMERA_PRECHECK,ARM_CAMERA_LIFT_ENABLE,
        ARM_CAMERA_LIFT_ZERO,ARM_CAMERA_DM_HOME,ARM_CAMERA_REACH_ENABLE};
    unsigned i,j;uint32_t before,started;
    /* Every unowned handoff stage tolerates one fresh stalled status and
     * issues no new6 move. Clearing in a new status resumes the same goal. */
    for(i=0;i<sizeof(stages)/sizeof(stages[0]);i++){
        CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(stages[i]);
        before=moves;CameraHandoffSample(0,0x85,1,simulated_counts[1]);
        CHECK(ArmCalibration_GetCameraState()==stages[i] && moves==before && !percent_motor_rpm[1]);
        for(j=0;j<3;j++){
            CameraHandoffSample(100,0x85,0,simulated_counts[1]); /* cached flags despite real new positions */
            CHECK(ArmCalibration_GetCameraState()==stages[i] && moves==before);
        }
        CameraHandoffSample(10,0x81,1,simulated_counts[1]);
        CHECK(ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED);CameraAdvanceTo(ARM_CAMERA_DONE);
    }
    /* Three new samples alone cannot fail before300ms; at300ms a held
     * stationary stall faults instead of proceeding or suppressing guards. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    before=moves;started=tick;CameraHandoffSample(0,0x85,1,0);
    CameraHandoffSample(100,0x85,1,0);CameraHandoffSample(100,0x85,1,0);
    CameraHandoffSample(99,0x85,1,0);
    CHECK(tick-started==299 && ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && moves==before);
    CameraHandoffSample(1,0x85,1,0);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !strcmp(ArmCalibration_GetCameraFailureReason(),"reach sustained stall during CAMERA handoff"));
    /* Progress delays the no-progress trigger, but cannot mask a persistent
     * latched flag indefinitely while reach should be holding. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CameraHandoffSample(0,0x85,1,0);
    for(j=1;j<=9;j++){CameraHandoffSample(100,0x85,1,(int64_t)j*32);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME);}
    CameraHandoffSample(100,0x85,1,320);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !strcmp(ArmCalibration_GetCameraFailureReason(),"reach stall flag persists during CAMERA handoff"));
    /* Genuine protection still fails immediately. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CameraHandoffSample(0,0x89,1,0);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
    /* Stop cannot leave an observer able to restart a cancelled action. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CameraHandoffSample(0,0x85,1,0);CHECK(ArmCalibration_Cancel()==HAL_OK);before=moves;
    CameraHandoffSample(400,0x81,1,0);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && moves==before);
    /* A new CAMERA command clears prior native ACK/motion diagnostics.
     * Reproduce the field's DM_HOME fault after a prior completed native job. */
    CameraSetup();CHECK(ArmCalibration_SetCameraPositionMode(1)==HAL_OK);fake_native_model=1;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    simulated_counts[1]=-52351;Feedback(6,-52351,0x83);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    motor.fb.pos_rad=-0.384f;motor.fb.vel_rads=1.57509f;
    CameraHandoffSample(0,0x85,1,-52351);
    for(j=0;j<3;j++)CameraHandoffSample(100,0x85,1,-52351);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1);
    CHECK(strstr(camera_failure_status,"phase=2") && strstr(camera_reach_fault_console,"ACK=0/0x00 motion=0"));
    CHECK(strstr(camera_reach_fault_console,"from=39.475%"));
    /* Unsigned tick wrap does not defeat300ms confirmation. */
    tick=UINT32_MAX-500;started=tick;
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CameraHandoffSample(0,0x85,1,0);
    for(j=0;j<3;j++)CameraHandoffSample(100,0x85,1,0);
    CHECK(tick<started && ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED &&
          !strcmp(ArmCalibration_GetCameraFailureReason(),"reach sustained stall during CAMERA handoff"));
}

static void TestArmIoDuringWait(void)
{
    arm_cal_query_status_t a5,a6,b5,b6;arm_cal_axis_status_t p5,p6;uint32_t i,dm_before,vel_before;arm_camera_state_t camera_before;
    CameraSetup();fake_query_model=1;fake_query_drop_axis=0;
    ArmCalibration_GetQueryStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetQueryStatus(ARM_DRIVER_REACH,&a6);
    dm_before=dm_moves;vel_before=velocity_calls;camera_before=ArmCalibration_GetCameraState();
    CHECK(ArmCalibration_PostCommand("arm speed 6 17")==HAL_OK);
    for(i=0;i<1100;i+=5){tick+=5;ArmCalibration_ServiceIo();}
    ArmCalibration_GetQueryStatus(ARM_DRIVER_LIFT,&b5);ArmCalibration_GetQueryStatus(ARM_DRIVER_REACH,&b6);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&p5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&p6);
    CHECK(b5.queued_position>a5.queued_position+2 && b6.queued_position>a6.queued_position+2);
    CHECK(tick-p5.position_tick<200 && tick-p6.position_tick<200 && tick-p5.flags_tick<200 && tick-p6.flags_tick<200);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && ArmCalibration_GetCameraState()==camera_before);
    CHECK(dm_moves==dm_before && velocity_calls==vel_before && ArmDriver_GetLinearSpeed(ARM_DRIVER_REACH)==60);
    ArmCalibration_DiscardCommands();fake_query_model=0;
    /* DM wait must not freeze an existing lift percent controller near TOP.
     * Feed its real encoder plant while only the bounded wait service runs. */
    CameraSetup();CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    Feedback(5,0,0x81);Feedback(6,0,0x81);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,10)==HAL_OK);
    for(i=0;i<300 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++){
        tick+=20;motor.fb.last_tick=tick;motor.fb.fb_count++;
        simulated_counts[0]+=(int64_t)round((double)percent_motor_rpm[0]*65536.0/3000.0);
        Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);ArmCalibration_ServiceWait();
    }
    {float value;CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && !percent_motor_rpm[0]);
     CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&value)==HAL_OK && fabsf(value-10)<=ARM_PERCENT_TOLERANCE);}
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,20)==HAL_OK);
    arm_enabled=1;motor.fb.enabled=1;motor.enable_pending=1;motor.fb.last_tick=tick;
    Feedback(5,simulated_counts[0],0x85);Feedback(6,0,0x81);ArmCalibration_ServiceWait();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && !arm_enabled && !percent_motor_rpm[0]);
    motor.enable_pending=0;
}

static void TestCameraNearTopKick(void)
{
    unsigned i;uint32_t velocities;
    /* Field sequence: START left lift at0.44%; CAMERA must not reset the
     * percent motion controller and kick it again just outside0.5%. */
    CameraSetup();encoder_gain[0]=12.0f;
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,120U)==HAL_OK);
    simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.0044);
    Feedback(5U,simulated_counts[0],0x81U);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    velocities=velocity_calls;
    for(i=0U;i<5U;i++)ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ZERO && velocity_calls==velocities); /* cached samples cannot unlock DM */
    tick+=ARM_MOTION_SETTLE_MS-1U;motor.fb.last_tick=tick;motor.fb.fb_count++;
    Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,simulated_counts[1],0x81U);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ZERO && velocity_calls==velocities);
    tick+=1U;motor.fb.last_tick=tick;motor.fb.fb_count++;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ZERO); /* cached position cannot finish even after50ms */
    Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,simulated_counts[1],0x81U);ArmCalibration_Poll();
    CHECK(velocity_calls==velocities && ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME);
    CameraAdvanceTo(ARM_CAMERA_DONE);
    /* A tiny drift inside the widened arrival band may keep holding,
     * preserving the requested20mm reference guard even with12x encoder gain. */
    CameraSetup();encoder_gain[0]=12.0f;
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,120U)==HAL_OK);
    simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.0044);
    Feedback(5U,simulated_counts[0],0x81U);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);
    simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.0051);
    for(i=0U;i<150U && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
        CameraStep(1U);CHECK(abs(percent_motor_rpm[0])<=1);
        CHECK((double)simulated_counts[0]<=-ARM_LIFT_FULL_TRAVEL_COUNTS*ARM_TOP_REFERENCE_TOLERANCE_MM/ARM_LIFT_FULL_TRAVEL_MM);
    }
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    /* Field raw-1315 is1.44mm belowTOP: hold without a new lift kick.
     * No cached frame can skip the held-feedback handshake. */
    CameraSetup();simulated_counts[0]=-1315;Feedback(5U,-1315,0x81);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    velocities=velocity_calls;CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && velocity_calls==velocities);
    for(i=0;i<10;i++)ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ZERO);
    for(i=0;i<120 && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
        CameraStep(1);CHECK(percent_motor_rpm[0]==0 && simulated_counts[0]==-1315);
    }
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && simulated_counts[0]==-1315);
    /* Direct TOP request must also settle inside8mm without velocity. */
    CameraSetup();simulated_counts[0]=-1315;Feedback(5U,-1315,0x81);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);velocities=velocity_calls;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)==HAL_OK);
    for(i=0;i<10 && ArmCalibration_GetState()!=ARM_CAL_IDLE;i++)CameraStep(0);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && velocity_calls==velocities && simulated_counts[0]==-1315);
    {const float reference_mm[]={11.18f,19.9f,20.1f};unsigned k;
     for(k=0;k<3;k++) {
        CameraSetup();simulated_counts[0]=(int64_t)round(-ARM_LIFT_FULL_TRAVEL_COUNTS*reference_mm[k]/ARM_LIFT_FULL_TRAVEL_MM);
        Feedback(5U,simulated_counts[0],0x81U);
        velocities=velocity_calls;CHECK(ArmCalibration_DeployCamera()==HAL_OK);ArmCalibration_Poll();
        CHECK(velocity_calls==velocities); /* Reference checks cannot themselves move the axis. */
        if(k<2)CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ENABLE);
        else CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED &&
             !strcmp(ArmCalibration_GetCameraFailureReason(),"zero/span/position reference inconsistent"));
     }}
}
/* DM stays live; intentionally omit ZDT frames during a stationary wait. */
static void CameraDmOnly(uint32_t ms,float angle)
{
    tick+=ms;motor.fb.pos_rad=angle;motor.fb.vel_rads=0.0f;
    motor.fb.last_tick=tick;motor.fb.fb_count++;motor.fb.enabled=arm_enabled;motor.fb.err_code=1;
    ArmCalibration_Poll();
}
static void TestCameraIdleFeedbackGap(void)
{
    unsigned wrap,j;uint32_t start,commands,enables;arm_cal_axis_status_t a5,a6;
    for(wrap=0;wrap<2;wrap++) {
        if(wrap)tick=UINT32_MAX-1600U;
        CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
        commands=moves;enables=enable_calls;start=tick;
        /* Reproduce930/970ms and extend beyond the old2s grace: neither
         * ZDT is moving, so losing idle feedback must not fail DM_HOME. */
        for(j=0;j<27;j++) {
            CameraDmOnly(100U,0.10f);
            CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && moves==commands && enable_calls==enables);
            CHECK(percent_motor_rpm[0]==0 && percent_motor_rpm[1]==0);
        }
        ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
        CHECK(tick-a5.position_tick>=2700U && tick-a6.position_tick>=2700U);
        /* DM reaches zero. No reach enable/target from cached ZDT frames. */
        CameraDmOnly(100U,0.0f);CameraDmOnly(100U,0.0f);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && moves==commands && enable_calls==enables);
        for(j=0;j<10;j++){CameraDmOnly(100U,0.0f);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && moves==commands);}
        /* Refresh only5: one live axis cannot authorize the next movement. */
        Feedback(5U,0,0x81);CameraDmOnly(10U,0.0f);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && moves==commands);
        /* Both positions/status are real new frames; continue once, same job. */
        Feedback(6U,0,0x81);CameraDmOnly(10U,0.0f);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_ENABLE && moves==commands);
        CameraAdvanceTo(ARM_CAMERA_DONE);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
        if(wrap)CHECK(tick<start);
    }
    /* Feedback can also lapse after reach enable, before its first target.
     * Reacquisition must not trip the shorter enable-handshake timer. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_ENABLE);
    commands=moves;CameraDmOnly(801U,0.0f);
    for(j=0;j<11;j++){CameraDmOnly(100U,0.0f);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_ENABLE && moves==commands);}
    Feedback(5U,0,0x81);Feedback(6U,0,0x81);CameraDmOnly(10U,0.0f);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_ENABLE && moves==commands);
    CameraAdvanceTo(ARM_CAMERA_DONE);
    /* Permanent absence blocks the new movement, then fails boundedly. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    commands=moves;for(j=0;j<10;j++)CameraDmOnly(100U,0.10f);
    CameraDmOnly(100U,0.0f);CameraDmOnly(100U,0.0f);
    for(j=0;j<22 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;j++)CameraDmOnly(100U,0.0f);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && moves==commands &&
        !strcmp(ArmCalibration_GetCameraFailureReason(),"fresh ZDT feedback unavailable before next motion"));
    /* STOP cancels the pending gate; later feedback cannot resume it. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    for(j=0;j<10;j++)CameraDmOnly(100U,0.10f);
    CameraDmOnly(100U,0.0f);CameraDmOnly(100U,0.0f);CHECK(ArmCalibration_Cancel()==HAL_OK);commands=moves;
    CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && moves==commands);
    /*6 is now moving: losing6 position is still an800ms motion fault. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    commands=stops6;Feedback(5U,0,0x81);CameraDmOnly(801U,0.0f);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && stops6>commands &&
        !strcmp(ArmCalibration_GetCameraFailureReason(),"ZDT feedback missing/stale"));
    /* Losing stationary5 while6 moves does not interrupt6. Return of real
     *5 frames permits final completion; no copied/faked timestamp. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);start=a5.position_tick;
    tick+=801U;Feedback(6U,simulated_counts[1],0x81);CameraDmOnly(0U,0.0f);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);CHECK(a5.position_tick==start);
    CameraAdvanceTo(ARM_CAMERA_DONE);
}
static void TestTopArrivalBand(void)
{
    unsigned i;uint32_t velocities;float percent;
    const float samples[]={2.5104f,7.9f};
    CHECK(fabsf(ArmCalibration_LiftTolerancePercent(0,240)-(8.0f*100.0f/240.0f))<1e-5f);
    CHECK(ArmCalibration_LiftTolerancePercent(30,240)==ARM_PERCENT_TOLERANCE);
    CHECK(ARM_TOP_REFERENCE_TOLERANCE_MM==20.0f);
    for(i=0;i<2;i++) {
        CameraSetup();simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*samples[i]/ARM_LIFT_FULL_TRAVEL_MM);
        Feedback(5,simulated_counts[0],0x81);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
        velocities=velocity_calls;test_material_active=1;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
        CameraStep(1);CameraStep(1);CameraStep(1);CameraStep(1);
        CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && velocity_calls==velocities && !percent_motor_rpm[0]);
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && fabsf(percent)<=(8.0f*100.0f/240.0f));
        test_material_active=0;CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
    /* A wider arrival band does not suppress a real stalled/protected axis,
     * even at the field failure's2.51mm belowTOP. */
    for(i=0;i<2;i++) {
        CameraSetup();simulated_counts[0]=-2296;Feedback(5,-2296,0x81);
        CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
        tick+=100;motor.fb.last_tick=tick;Feedback(5,-2296,i?0x89:0x85);Feedback(6,0,0x81);
        ArmCalibration_Poll();CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && !percent_motor_rpm[0]);
        CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
}
static void TestCameraFeedbackWindows(void)
{
    static const uint32_t ages[]={507U,600U,607U,800U,801U,807U,929U,1999U,2000U,2001U};
    unsigned i,axis;float percent;uint32_t saved;
    for(axis=0U;axis<2U;axis++)for(i=0U;i<sizeof(ages)/sizeof(ages[0]);i++) {
        CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
        CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
        /* Reproduce a delayed5/6 status with fresh positions and other axis/DM.
         * Preserve actual receive timestamps; no synthetic freshness. */
        saved=tick;tick=saved-ages[i];Feedback(axis?6U:5U,0,0x81U);tick=saved;
        {FDCAN_RxHeaderTypeDef h={0};uint8_t data[7]={0x36U,0,0,0,0,0,0x6BU};
         h.Identifier=(axis?6U:5U)<<8U;h.IdType=FDCAN_EXTENDED_ID;
         h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;
         CHECK(ArmCalibration_OnCanFrame(&h,data,7U)==1U);}
        Feedback(axis?5U:6U,simulated_counts[axis?0:1],0x81U);
        motor.fb.last_tick=tick;motor.fb.fb_count++;ArmCalibration_Poll();
        if(axis==0U || ages[i]<=800U) {
            CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
            CameraAdvanceTo(ARM_CAMERA_DONE);
        } else {
            CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
            CHECK(!strcmp(ArmCalibration_GetCameraFailureReason(),"ZDT feedback missing/stale"));
        }
    }
    for(axis=0U;axis<2U;axis++) {
        CameraSetup();saved=tick;Feedback(axis?6U:5U,0,0x81U);
        tick=saved+800U;CHECK(ArmCalibration_GetPercent(axis?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&percent)==HAL_OK);
        tick=saved+801U;CHECK(ArmCalibration_GetPercent(axis?ARM_DRIVER_REACH:ARM_DRIVER_LIFT,&percent)==HAL_ERROR);
    }
}
static void SetTestGrip(uint8_t open)
{
    unsigned j;
    CHECK(ArmServo_Enable(ARM_SERVO_GRIP)==HAL_OK);
    CHECK((open?ArmServo_GripOpen():ArmServo_GripClose())==HAL_OK);
    for(j=0;j<100;j++){tick+=20U;ArmServo_Update(tick);}
    CHECK(open?ArmServo_GripIsOpen():ArmServo_GripIsClosed());
}
static void TestEmptyRotationSpeed(void)
{
    CameraSetup();SetTestGrip(0U);motor.fb.last_tick=tick;
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_OK);
    CHECK(NEAR(ArmDriver_RotationSpeedDps(),231.66f));
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(NEAR(motor.target_vel_rads,2.82743338823f*1.5f*1.1f*1.3f));
    SetTestGrip(1U);motor.fb.last_tick=tick;
    CHECK(NEAR(ArmDriver_RotationSpeedDps(),213.84f));
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(NEAR(motor.target_vel_rads,3.39292006588f*1.5f*1.1f));
    CHECK(ArmDriver_RotateToDegAtSpeed(0.0f,214.0f)==HAL_ERROR);
    CHECK(ArmDriver_RotateToDegAtSpeed(0.0f,30.0f)==HAL_OK);
    CHECK(NEAR(motor.target_vel_rads,0.5235987756f*1.5f));
    CHECK(ArmServo_GripClose()==HAL_OK && !ArmServo_GripIsOpen());
    CHECK(NEAR(ArmDriver_RotationSpeedDps(),231.66f));
    CHECK(ArmDriver_RotateToDegAtSpeed(0.0f,232.0f)==HAL_ERROR);
    SetTestGrip(0U);motor.fb.last_tick=tick;
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(NEAR(motor.target_vel_rads,2.82743338823f*1.5f*1.1f*1.3f));
    CHECK(ArmServo_Disable(ARM_SERVO_GRIP)==HAL_OK);
    CHECK(NEAR(ArmDriver_RotationSpeedDps(),231.66f));
    SetTestGrip(0U);motor.fb.last_tick=tick;
}
static void TestEmptyLiftSpeed(void)
{
    unsigned direction,empty,j;int peak;
    for(direction=0;direction<2;direction++)for(empty=0;empty<2;empty++) {
        CameraSetup();SetTestGrip((uint8_t)empty);
        CHECK(ArmServo_GripIsOpen()==empty);
        CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,238U)==HAL_OK);
        simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*(direction?0.80:0.10));
        Feedback(5,simulated_counts[0],0x81);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
        test_material_active=1U;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,direction?20.0f:90.0f)==HAL_OK);
        CHECK(ArmCalibration_EmptyLiftMotionActive()==empty);peak=0;
        for(j=0;j<45;j++) {
            tick+=100U;Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);
            motor.fb.last_tick=tick;ArmCalibration_Poll();
            if(fake_lift_active && fake_lift_speed>peak)peak=fake_lift_speed;
            CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE);
            CHECK(fake_lift_speed<=(empty?431:410) && percent_motor_rpm[0]==0);
        }
        CHECK(peak==(empty?431:410));
        CHECK(last_lift_acceleration==234U);
        CHECK(ArmDriver_RunLinear(ARM_DRIVER_LIFT,empty?431:410,-50)==HAL_OK);
        CHECK(ArmDriver_RunLinear(ARM_DRIVER_LIFT,empty?432:411,-50)==HAL_ERROR);
        CHECK(ArmDriver_RunLinear(ARM_DRIVER_LIFT,0,-50)==HAL_OK);
        /* Closing after an empty descent starts does not switch the same
         * command; the subsequent loaded raise selects the normal profile. */
        if(empty) {
            SetTestGrip(0U);
            CHECK(!ArmServo_GripIsOpen() && ArmCalibration_EmptyLiftMotionActive());
        }
        CHECK(ArmCalibration_Cancel()==HAL_OK);test_material_active=0U;
        CHECK(!ArmCalibration_EmptyLiftMotionActive());
        Feedback(5,simulated_counts[0],0x81);
        CHECK(ArmDriver_RunLinear(ARM_DRIVER_LIFT,411,-50)==HAL_ERROR); /* manual cap excludes empty boost */
    }
    CameraSetup();SetTestGrip(1U);
    simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*(9.0/ARM_LIFT_FULL_TRAVEL_MM));
    Feedback(5,simulated_counts[0],0x81);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);test_material_active=1;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
    peak=0;
    for(j=0;j<5;j++){tick+=100;Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);ArmCalibration_Poll();
        CHECK(fake_lift_speed<=431 && percent_motor_rpm[0]==0);
        if(fake_lift_active && fake_lift_speed>peak)peak=fake_lift_speed;}
    CHECK(peak==109); /* Configured60RPM empty scale; native driver decelerates. */
    CHECK(ArmCalibration_Cancel()==HAL_OK);test_material_active=0;
}
#ifdef TEST_SPEED_PROFILE_ONLY
static void TestReachLoadAndFastArrival(void)
{
    unsigned fast;uint16_t goal_speed;
    CameraSetup();CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,72)==HAL_OK);
    SetTestGrip(0);CHECK(ArmDriver_ReachMotionSpeedRpm()==222);
    Feedback(6,0,0x81);Feedback(5,0,0x81);CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    goal_speed=ArmDriver_ReachMotionSpeedRpm();CHECK(ArmDriver_ReachAbsolutePulses(100,goal_speed,150)==HAL_OK && last_speed==222);
    {uint32_t generation=ArmDriver_ReachRetryGeneration(),before=moves,old_slots=slots;
     CHECK(ArmDriver_ReplayReachAbsolutePulses(101)==HAL_ERROR && moves==before);
     slots=0;CHECK(ArmDriver_ReplayReachAbsolutePulses(100)==HAL_BUSY && moves==before);
     CHECK(ArmDriver_ReachRetryActive() && ArmDriver_ReachRetryGeneration()==generation);
     slots=old_slots;CHECK(ArmDriver_ReplayReachAbsolutePulses(100)==HAL_OK && moves==before+1);
     CHECK(last_speed==222 && last_acceleration==150 && ArmDriver_ReachRetryGeneration()==generation);
     ArmDriver_CancelReachRetry();CHECK(ArmDriver_ReplayReachAbsolutePulses(100)==HAL_BUSY && moves==before+1);
    }
    ArmDriver_CancelReachRetry();SetTestGrip(1);Feedback(6,0,0x81);Feedback(5,0,0x81);
    CHECK(ArmDriver_ReachMotionSpeedRpm()==166);
    CHECK(ArmDriver_ReachAbsolutePulses(100,ArmDriver_ReachMotionSpeedRpm(),150)==HAL_OK && last_speed==166);
    CHECK(ArmDriver_ReassertReachAbsolutePulses(100,15)==HAL_OK && last_speed==15);
    {arm_driver_config_t defaults;ArmDriver_DefaultConfig(&defaults);
     CHECK(defaults.reach.speed_rpm==79U);
     CHECK(defaults.reach.acceleration==209U && ARM_CAMERA_POSITION_ACCEL==209U);
     CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,defaults.reach.speed_rpm)==HAL_OK);
     CHECK(ArmDriver_ReachMotionSpeedRpm()==182U);
     CHECK(ArmDriver_ReachAbsolutePulses(100,182,150)==HAL_OK && last_speed==182);
     CHECK(ArmDriver_ReachAbsolutePulses(100,183,150)==HAL_ERROR);
     test_material_grab_reach=1U;
     CHECK(ArmDriver_ReachMotionSpeedRpm()==182U); /* Generic/return profile stays unchanged. */
     CHECK(ArmDriver_ReachAbsolutePulses(100,262,209)==HAL_OK && last_speed==262 && last_acceleration==209);
     CHECK(ArmDriver_ReachAbsolutePulses(100,263,209)==HAL_ERROR);
     test_material_grab_reach=0U;
     CHECK(ArmDriver_ReachAbsolutePulses(100,262,209)==HAL_ERROR);
     ArmDriver_CancelReachRetry();SetTestGrip(0);Feedback(6,0,0x81);Feedback(5,0,0x81);
     CHECK(ArmDriver_ReachMotionSpeedRpm()==246U);
     CHECK(ArmDriver_ReachAbsolutePulses(100,246,150)==HAL_OK && last_speed==246);
    }
    ArmDriver_CancelReachRetry();CHECK(ArmCalibration_Cancel()==HAL_OK);
    for(fast=0;fast<2;fast++) {
        CameraSetup();SetTestGrip(0);Feedback(5,0,0x81);Feedback(6,0,0x81);
        CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);test_material_active=(uint8_t)fast;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,50)==HAL_OK);
        tick+=10;Feedback(5,(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*.1),0x81);Feedback(6,0,0x81);motor.fb.last_tick=tick;ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE);
        simulated_counts[0]=(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*.5);
        tick+=1;Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);motor.fb.last_tick=tick;ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()!=ARM_CAL_IDLE); /* Neither fast branch truncates native deceleration. */
        CHECK(percent_motor_rpm[0]==0);
        {unsigned j;for(j=0;j<8 && ArmCalibration_GetState()!=ARM_CAL_IDLE;j++)CameraStep(0);}
        CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
        CHECK(ArmCalibration_Cancel()==HAL_OK);test_material_active=0;
    }
}
static void TestReleasedOpeningLiftProfile(void)
{
    unsigned j;uint32_t before;
    CameraSetup();SetTestGrip(0U);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,238U)==HAL_OK);
    CHECK(ArmServo_GripOpen()==HAL_OK && !ArmServo_GripIsOpen() && ArmServo_GripOpeningOrOpen());
    test_start_pose_active=test_release_opening=1U;
    simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*.8);
    Feedback(5,simulated_counts[0],0x83);Feedback(6,0,0x81);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    before=native_lift_moves;CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
    CHECK(ArmCalibration_EmptyLiftMotionActive());
    for(j=0;j<4 && native_lift_moves==before;j++)CameraStep(0);
    CHECK(native_lift_moves==before+1 && fake_lift_speed==431 && last_lift_acceleration==234);
    test_release_opening=test_start_pose_active=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);SetTestGrip(1U);
}
static void TestLiftMeasuredRangePolicy(void)
{
    const float measured_percent[]={-12.5f,110.0f};
    unsigned i;float percent;
    CHECK(ARM_LIFT_RANGE_STOP_ENABLED==0);
    for(i=0;i<2;i++) {
        CameraSetup();
        simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*measured_percent[i]/100.0f);
        Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,0,0x81U);
        CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
        ArmCalibration_Poll();
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK);
        CHECK(fabsf(percent-measured_percent[i])<0.01f); /* No fake TOP/arrival. */
        CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_OK);
        CHECK(ArmDriver_CheckHeightInterlock()==HAL_OK);
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,-1.0f)==HAL_ERROR);
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,101.0f)==HAL_ERROR);
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,NAN)==HAL_ERROR);
        CHECK(ArmCalibration_DeployCamera()==HAL_OK);
        ArmCalibration_Poll();
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_LIFT_ENABLE);
        CHECK(ArmCalibration_Cancel()==HAL_OK);
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,i?90.0f:0.0f)==HAL_OK);
        tick+=100;Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,0,0x81U);
        motor.fb.last_tick=tick;ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE);
        CHECK(fake_lift_active && (i?fake_lift_target>simulated_counts[0]:fake_lift_target<simulated_counts[0]));
        CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
    /* A real lift stall still faults; a forbidden low-height DM sector still
     * stops the lift. These are distinct from a measured travel excursion. */
    CameraSetup();Feedback(5U,0,0x81U);Feedback(6U,0,0x81U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,50.0f)==HAL_OK);
    tick+=100;Feedback(5U,0,0x85U);motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED);
    CameraSetup();Feedback(5U,(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*1.10),0x81U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);ArmCalibration_Poll();
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_OK);
    motor.fb.pos_rad=60.0f*1.5f*0.01745329252f;motor.fb.last_tick=tick;
    CHECK(ArmDriver_CheckHeightInterlock()==HAL_ERROR);
    CHECK(ArmCalibration_Cancel()==HAL_OK);
}
#endif
static void TestCameraLateSnapshot(void)
{
    unsigned i,wrap;char cause[96];
    for(wrap=0U;wrap<2U;wrap++) {
        if(wrap)tick=UINT32_MAX-350U;
        CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
        CameraAdvanceTo(ARM_CAMERA_DM_HOME);
        /* A real RX interrupt can publish a timestamp AFTER Poll captured now.
         * Each snapshot here is coherent and genuinely fresh, just 3ms newer. */
        dm_snapshot_delay=3U;
        for(i=0U;i<150U && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE &&
            ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
        dm_snapshot_delay=0U;
    }
    /* Genuine stale feedback still fails, and its historical cause survives
     * later good feedback/status reads. No automatic success after failure. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    motor.fb.last_tick=tick-DM_LINK_TIMEOUT_MS_DEFAULT-1U;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strcmp(ArmCalibration_GetCameraFailureReason(),"none"));
    (void)strncpy(cause,ArmCalibration_GetCameraFailureReason(),sizeof(cause)-1U);cause[sizeof(cause)-1U]=0;
    CameraStep(1U);camera_failure_status[0]=camera_failure_snapshot[0]=0;
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1U);
    CHECK(strstr(camera_failure_status,cause) && strstr(camera_failure_snapshot,"age=801ms"));
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK && !strcmp(ArmCalibration_GetCameraFailureReason(),"none"));
    CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraReferenceAndHold(void)
{
    arm_cal_axis_status_t a5,a6;
    uint32_t saves,i,disables,dm_count,velocities;
    float percent;
    CameraSetup();
    simulated_counts[0]=2353;simulated_counts[1]=-20761;
    Feedback(5U,2353,0x81U);Feedback(6U,-20761,0x83U);
    saves=save_calls;
    CHECK(ArmCalibration_PostCommand("arm camera init")==HAL_OK);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE);
    velocities=velocity_calls;dm_count=dm_moves;
    /* Cached frames cannot teach, and stop cancels teaching without changing zeros. */
    for(i=0U;i<3U;i++)ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE && velocity_calls==velocities && dm_moves==dm_count);
    CHECK(ArmCalibration_ProcessCommand("arm stop")==1U);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
    CHECK(a5.zero_counts==0 && a6.zero_counts==0);
    arm_fault=0U;CHECK(ArmCalibration_DeployCameraInit()==HAL_OK);
    for(i=0U;i<3U;i++)CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE);
    CameraStep(1U);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_PRECHECK);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
    CHECK(a5.zero_counts==2353 && a5.upper_counts==2353 && a6.zero_counts==-20761 && a6.lower_counts==-20761);
    CHECK(save_calls==saves && velocity_calls==velocities && dm_moves==dm_count);
    CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);disables=dm_disables;
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(save_calls==saves && dm_disables==disables && arm_enabled && NEAR(motor.target_pos_rad,0));
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-41.181766f)<=ARM_PERCENT_TOLERANCE);
    CHECK(fabs((double)(simulated_counts[1]+20761)*360.0/65536.0+300.0)<4.0);
    /* Restore ordinary zero references for the independent hold tests. */
    CHECK(ArmCalibration_ProcessCommand("arm top 5")==1U);
    Sample(5U,0,0x81U);Sample(5U,0,0x81U);Sample(5U,0,0x81U);
    CHECK(ArmCalibration_ProcessCommand("arm zero 6")==1U);
    Sample(6U,0,0x81U);Sample(6U,0,0x81U);Sample(6U,0,0x81U);
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    disables=dm_disables;motor.fb.pos_rad=-1.5f*1.5f*0.01745329252f;motor.fb.vel_rads=0.12f;
    CameraStep(0U);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && dm_disables==disables && arm_enabled);
    CHECK(abs(percent_motor_rpm[1])<=80);
    for(i=0U;i<120U && ArmCalibration_GetState()!=ARM_CAL_IDLE;i++) {
        CameraStep(0U);CHECK(abs(percent_motor_rpm[1])<=80);
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && arm_enabled);
    /* One soft deviation (even cached for 300ms) is not three new samples. */
    motor.fb.pos_rad=-3.0f*1.5f*0.01745329252f;motor.fb.vel_rads=0.25f;CameraStep(0U);
    for(i=0U;i<3U;i++) {
        tick+=100U;Feedback(5U,simulated_counts[0],0x81U);Feedback(6U,simulated_counts[1],0x81U);ArmCalibration_Poll();
    }
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
    motor.fb.vel_rads=0;CameraStep(1U);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(arm_enabled && dm_disables==disables && NEAR(motor.target_pos_rad,0));
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    motor.fb.pos_rad=-3.0f*1.5f*0.01745329252f;motor.fb.vel_rads=0.25f;
    for(i=0U;i<4U;i++)CameraStep(0U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    motor.fb.pos_rad=-9.0f*1.5f*0.01745329252f;CameraStep(0U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    /* An unsettled/missing teaching session cannot silently overwrite references. */
    CameraSetup();CHECK(ArmCalibration_DeployCameraInit()==HAL_OK);tick+=5001U;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
    CHECK(a5.zero_counts==0 && a6.zero_counts==0 && save_calls==saves);
    encoder_sign[1]=1;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraBraking(void)
{
    unsigned int i;int64_t furthest=0;
    float percent;
    /* Stress actual motion faster than nominal RPM. The old symmetric slew
     * retained outward speed past 300deg; record the entire path, not just end. */
    CameraSetup();encoder_gain[1]=10.0f;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    for(i=0U;i<120U && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE &&
        ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
        CameraStep(1U);if(simulated_counts[1]<furthest)furthest=simulated_counts[1];
    }
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    CHECK(-(double)furthest*360.0/65536.0<=310.0);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-41.181766f)<=ARM_PERCENT_TOLERANCE);
    CHECK(arm_enabled && percent_motor_rpm[1]==0);
    /* If a feedback jump crosses the goal outside the arrival band, command
     * STOP before correction; do not keep outward speed or reverse in motion. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CameraStep(1U);CHECK(percent_motor_rpm[1]>0);
    simulated_counts[1]=(int64_t)round(ARM_REACH_FULL_TRAVEL_COUNTS*0.42);
    CameraStep(1U);CHECK(percent_motor_rpm[1]==0 && ArmCalibration_GetState()==ARM_CAL_PERCENT_SETTLE);
    CameraStep(1U);CHECK(percent_motor_rpm[1]==0);
    /* A large jump cannot result in a full-stroke automatic return. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CameraStep(1U);simulated_counts[1]=(int64_t)ARM_REACH_FULL_TRAVEL_COUNTS;
    CameraStep(1U);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    encoder_gain[1]=1.0f;encoder_sign[1]=1;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraReachSpeed(void)
{
    uint32_t slow_time,fast_time,started;
    unsigned int i;
    int16_t last_rpm,peak_rpm=0;
    CameraSetup();CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,15U)==HAL_OK);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    started=tick;CameraAdvanceTo(ARM_CAMERA_DONE);slow_time=tick-started;
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    started=tick;last_rpm=percent_motor_rpm[1];
    for(i=0U;i<120U && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE &&
        ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
        CameraStep(1U);
        CHECK(abs(percent_motor_rpm[1])<=60);
        if(abs(percent_motor_rpm[1])>abs(last_rpm))CHECK(abs(percent_motor_rpm[1])-abs(last_rpm)<=4);
        last_rpm=percent_motor_rpm[1];if(abs(last_rpm)>peak_rpm)peak_rpm=(int16_t)abs(last_rpm);
    }
    fast_time=tick-started;
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && peak_rpm>15 && fast_time<slow_time);
    CHECK(arm_enabled && NEAR(motor.target_pos_rad,0) && percent_motor_rpm[1]==0);
    encoder_sign[1]=1;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void NativeSetup(void)
{
    CameraSetup();CHECK(ArmCalibration_SetCameraPositionMode(1U)==HAL_OK);fake_native_model=1U;
}
#ifdef TEST_CAMERA_REPLAY_ONLY
static int TestCameraReplay(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_servo_config_t servos;
    uint32_t before,stops,saves;unsigned i;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    zdt.can_ready=1;CHECK(ArmDriver_Init(&cfg)==HAL_OK);
    ArmCalibration_DefaultConfig(&cal);CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    htim4.Instance=TIM4;htim4.State=HAL_TIM_STATE_READY;
    ArmServo_DefaultConfig(&servos);CHECK(ArmServo_Init(&servos)==HAL_OK);
    /* Lost command and ACK: same absolute replay, actual arrival still required. */
    NativeSetup();test_start_pose_active=1;fake_native_drop_remaining=1;
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmDriver_ReachAbsolutePulses(1000,100,209)==HAL_OK);before=moves;stops=stops6;
    for(i=0;i<8 && moves==before;i++)CameraStep(1);
    CHECK(moves==before+1 && last_pulses==1000 && last_speed==100 && last_acceleration==209 && stops6==stops);
    fake_native_active=0;fake_native_reached=1;before=moves;
    for(i=0;i<10 && moves==before;i++)CameraStep(1);
    CHECK(moves==before+1 && last_pulses==1000 && stops6==stops);
    CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);before=moves;
    for(i=0;i<10;i++)CameraStep(1);
    CHECK(moves==before);test_start_pose_active=0;
    NativeSetup();test_start_pose_active=1;simulated_counts[0]=-50000;Feedback(5U,simulated_counts[0],0x81U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    fake_lift_drop_commands=5;before=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
    for(i=0;i<40 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && native_lift_moves>=before+6);test_start_pose_active=0;
    NativeSetup();step_enable_result=HAL_ERROR;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    for(i=0;i<3;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED);
    step_enable_result=HAL_OK;CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    NativeSetup();dm_tx_result=HAL_ERROR;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);
    for(i=0;i<5;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED && !ArmTask_IsEnabled());
    dm_tx_result=HAL_OK;CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && ArmTask_IsEnabled());
    /* CAMERA lift retries beyond the old three-ACK-send limit. */
    NativeSetup();simulated_counts[0]=-50000;Feedback(5U,simulated_counts[0],0x81U);
    fake_lift_drop_commands=5;before=native_lift_moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && native_lift_moves>=before+6);
    /* Accepted lift stalls before its endpoint: resend SAME ABS without STOP. */
    NativeSetup();simulated_counts[0]=-50000;Feedback(5U,simulated_counts[0],0x81U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_LIFT_ZERO);CameraStep(1);
    fake_lift_active=0;fake_lift_reached=1;before=native_lift_moves;stops=stops5;
    for(i=0;i<8 && native_lift_moves==before;i++)CameraStep(1);
    CHECK(native_lift_moves>before && stops5==stops && last_lift_pulses==0);
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    NativeSetup();fake_native_ack_enabled=0;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);before=moves;
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && moves==before); /* Moving despite missing ACK. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    simulated_counts[1]=fake_native_target+700;fake_native_active=0;fake_native_reached=1;before=moves;
    for(i=0;i<10 && moves==before;i++)CameraStep(1);
    CHECK(moves>before && last_pulses==2667); /* Small unarrived residual also replays. */
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    NativeSetup();fake_native_ack_enabled=0;fake_native_drop_remaining=1;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    before=moves;saves=save_calls;CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && moves>before && last_pulses==2667 && save_calls==saves);
    /* Accepted command moved part way, then reports old REACHED far from target. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CameraStep(1);fake_native_active=0;fake_native_reached=1;before=moves;stops=stops6;
    for(i=0;i<12 && moves==before;i++)CameraStep(1);
    CHECK(moves==before+1 && stops6==stops && last_pulses==2667);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    /* Permanently unexecuted commands keep the existing finite action timeout. */
    NativeSetup();fake_native_ack_enabled=0;fake_native_drop_remaining=255;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);before=moves;
    for(i=0;i<125 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && moves-before<=50);
    CHECK(strstr(ArmCalibration_GetCameraFailureReason(),"native reach timeout"));
    /* Cancel never leaves a replay able to restart the axis. */
    NativeSetup();fake_native_ack_enabled=0;fake_native_drop_remaining=255;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);CameraStep(1);
    CHECK(ArmCalibration_Cancel()==HAL_OK);before=moves;
    for(i=0;i<10;i++)CameraStep(1);
    CHECK(moves==before && ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    /* Parking uses the existing CAMERA lift/DM but absolute reach zero,
     * and must not overwrite the normal CAMERA target for subsequent jobs. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CHECK(ArmCalibration_DeployParkingCamera()==HAL_BUSY);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && last_pulses==2667);
    saves=save_calls;CHECK(ArmCalibration_DeployParkingCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    {float pct=99;CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&pct)==HAL_OK && fabsf(pct)<=ARM_PERCENT_TOLERANCE);}
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && last_pulses==0 && save_calls==saves);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && last_pulses==2667 && save_calls==saves);
    return (int)test_failures;
}
#endif
#ifdef TEST_CAMERA_HOLD_ONLY
static void CameraNearHold(float offset)
{
    float target=-300.0f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
    NativeSetup();SetTestGrip(1U);
    simulated_counts[1]=(int64_t)llround((target+offset)*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);
    Feedback(6U,simulated_counts[1],0x81U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(!fake_native_active);
}
static int TestCameraNearHold(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_servo_config_t servos;
    unsigned side,i,stage;uint32_t before_moves,before_velocity,before_stop,before_save;
    float actual,target=-300.0f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    zdt.can_ready=1;CHECK(ArmDriver_Init(&cfg)==HAL_OK);
    ArmCalibration_DefaultConfig(&cal);CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    htim4.Instance=TIM4;htim4.State=HAL_TIM_STATE_READY;
    ArmServo_DefaultConfig(&servos);CHECK(ArmServo_Init(&servos)==HAL_OK);
    for(side=0;side<2;side++) {
        float sign=side?1.0f:-1.0f;
        CameraNearHold(sign*.1f);before_moves=moves;before_velocity=velocity_calls;before_stop=stops6;before_save=save_calls;
        /* Same1.57mm rebound as run009, plus its mirrored direction. */
        simulated_counts[1]=side?(int64_t)llround((target+.8825f)*ARM_REACH_FULL_TRAVEL_COUNTS/100.0):-53443;
        CameraStep(1);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && stops6==before_stop+1 && moves==before_moves);
        for(i=0;i<5;i++)ArmCalibration_Poll();
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && velocity_calls==before_velocity);
        for(i=0;i<80 && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
            CameraStep(1);CHECK(abs(percent_motor_rpm[1])<=6);
        }
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && velocity_calls>before_velocity);
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&actual)==HAL_OK && fabsf(actual-target)<=ARM_PERCENT_TOLERANCE);
        CHECK(moves==before_moves && save_calls==before_save && percent_motor_rpm[1]==0);
        /* A near endpoint that stays held reaches READY without a new move. */
        CameraNearHold(sign*.1f);before_moves=moves;before_velocity=velocity_calls;
        CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(moves==before_moves && velocity_calls==before_velocity);
    }
    /* Run007: native arrival was valid, then the final held sample moved
     * by1.05percentage points. It must no longer fault solely for residual. */
    for(side=0;side<2;side++) {
        test_camera_native_arrived=0;CameraNearHold(0.0f);before_moves=moves;
        for(i=0;i<15 && !test_camera_native_arrived;i++)CameraStep(1);
        CHECK(test_camera_native_arrived && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        simulated_counts[1]=(int64_t)llround((target+(side?1.05f:-1.05f))*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);
        CameraAdvanceTo(ARM_CAMERA_DONE);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && moves==before_moves);
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&actual)==HAL_OK && fabsf(actual-target)>ARM_PERCENT_TOLERANCE);
    }
    /* Hold-only does not waive the existing2% correction bound or travel range. */
    CameraNearHold(-.1f);simulated_counts[1]=(int64_t)llround((target-2.1f)*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strstr(ArmCalibration_GetCameraFailureReason(),"bounded correction range"));
    CameraNearHold(-.1f);simulated_counts[1]=(int64_t)llround(1.02*ARM_REACH_FULL_TRAVEL_COUNTS);CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strstr(ArmCalibration_GetCameraFailureReason(),"wrong direction/range"));
    /* Genuine native commands retain both wrong-direction and overrun checks. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(fake_native_active);fake_native_active=0;simulated_counts[1]=(int64_t)llround(-.008*ARM_REACH_FULL_TRAVEL_COUNTS);CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strstr(ArmCalibration_GetCameraFailureReason(),"wrong direction/range"));
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_active=0;simulated_counts[1]=(int64_t)llround((target+6)*ARM_REACH_FULL_TRAVEL_COUNTS/100.0);CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strstr(ArmCalibration_GetCameraFailureReason(),"overrun"));
    /* Cancelling STOP verification or the correction never resumes it later. */
    for(stage=0;stage<2;stage++) {
        CameraNearHold(-.1f);simulated_counts[1]=-53443;CameraStep(1);before_velocity=velocity_calls;
        if(stage)for(i=0;i<10 && velocity_calls==before_velocity;i++)CameraStep(1);
        CHECK(ArmCalibration_Cancel()==HAL_OK);before_velocity=velocity_calls;before_moves=moves;
        for(i=0;i<10;i++)CameraStep(1);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && !percent_motor_rpm[1] && moves==before_moves && velocity_calls==before_velocity);
    }
    /* Frozen correction uses its existing10s timeout, not an unlimited wait. */
    CameraNearHold(-.1f);simulated_counts[1]=-53443;CameraStep(1);before_velocity=velocity_calls;
    for(i=0;i<10 && velocity_calls==before_velocity;i++)CameraStep(1);
    CHECK(velocity_calls>before_velocity);encoder_gain[1]=0;
    for(i=0;i<110 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && strstr(ArmCalibration_GetCameraFailureReason(),"residual correction timeout"));
    encoder_gain[1]=1;CHECK(ArmCalibration_Cancel()==HAL_OK);
    return (int)test_failures;
}
#endif
static void TestCameraNativePosition(void)
{
    uint32_t before_moves,before_velocity,disables,i;
    float percent;
    NativeSetup();before_moves=moves;before_velocity=velocity_calls;
    {
        int32_t endpoint_pulses;
        float endpoint_pct=0.4669f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
        CHECK(ArmCalibration_GetAbsoluteReachPulses(endpoint_pct,&endpoint_pulses)==HAL_OK && endpoint_pulses==-4);
        endpoint_pct=8.6188f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
        CHECK(ArmCalibration_GetAbsoluteReachPulses(endpoint_pct,&endpoint_pulses)==HAL_OK && endpoint_pulses==-77);
        CHECK(ArmCalibration_GetAbsoluteReachPulses(-2.0f,&endpoint_pulses)==HAL_ERROR);
        CHECK(ArmCalibration_UseTopGuard(ARM_TOP_DROP_LIMIT_MM,-20.0f,20.0f)==HAL_OK);
        Feedback(5U,-91845,0x80U);ArmCalibration_Poll();
        CHECK(ArmDriver_CheckRawRotationTarget(158.8785f*0.01745329252f)==HAL_OK);
        Feedback(5U,-160000,0x80U);ArmCalibration_Poll();
        CHECK(ArmDriver_CheckRawRotationTarget(158.8785f*0.01745329252f)==HAL_ERROR);
        Feedback(5U,-185239,0x80U);ArmCalibration_Poll();
        CHECK(ArmDriver_CheckRawRotationTarget(2.6040f*0.01745329252f)==HAL_OK);
        CHECK(ArmDriver_CheckRawRotationTarget(158.8785f*0.01745329252f)==HAL_ERROR);
        CHECK(ArmCalibration_UseTopGuard(ARM_TOP_DROP_LIMIT_MM,ARM_LOW_ROTATION_MIN_DEG,ARM_LOW_ROTATION_MAX_DEG)==HAL_OK);
        Feedback(5U,-185239,0x80U);ArmCalibration_Poll();
        ArmDriver_UpdateLiftHeight(-200.0f,tick,1U);
        {
            float mapped0,mapped1;
            CHECK(ArmDriver_MapRawRotation(0.0f,&mapped0)==HAL_OK);
            CHECK(ArmDriver_MapRawRotation(1.0f,&mapped1)==HAL_OK);
            /* This facade fixture uses a reversed 1.5 ratio; test arm angles,
             * not unconverted raw motor radians. */
            CHECK(ArmDriver_CheckRawRotationTarget((43.0f-mapped0)/(mapped1-mapped0))==HAL_OK);
            CHECK(ArmDriver_CheckRawRotationTarget((-43.0f-mapped0)/(mapped1-mapped0))==HAL_OK);
            CHECK(ArmDriver_CheckRawRotationTarget((45.0f-mapped0)/(mapped1-mapped0))==HAL_ERROR);
            CHECK(ArmDriver_CheckRawRotationTarget((-45.0f-mapped0)/(mapped1-mapped0))==HAL_ERROR);
        }
    }
    NativeSetup();before_moves=moves;before_velocity=velocity_calls;
    CHECK(ArmCalibration_ProcessCommand("arm camera")==1U);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(moves==before_moves+1U && last_address==6U && last_pulses==2667 && last_speed==60U && last_acceleration==150U);
    CHECK(ArmCalibration_ProcessCommand("arm camera")==1U && moves==before_moves+1U);
    CHECK(ArmCalibration_ProcessCommand("arm camera mode encoder")==1U && moves==before_moves+1U);
    CHECK(ArmCalibration_SetCameraPositionMode(0U)==HAL_BUSY);
    disables=dm_disables;CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(moves==before_moves+1U && velocity_calls==before_velocity && !fake_native_active);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-41.181766f)<=ARM_PERCENT_TOLERANCE);
    CHECK(arm_enabled && dm_disables==disables && NEAR(motor.target_pos_rad,0));
    /* Already at goal: no second relative 300deg command. */
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(moves==before_moves+1U);
    /* From -150deg, the driver travels the remaining distance to the same absolute -300deg target. */
    NativeSetup();simulated_counts[1]=-27307;Feedback(6U,simulated_counts[1],0x83U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);CHECK(last_pulses==2667); /* Absolute goal stays -300deg from a -150deg start. */
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(fabs((double)simulated_counts[1]*360.0/65536.0+300.0)<0.1);
    /* Retract from a position beyond 300deg using the remaining relative move. */
    NativeSetup();simulated_counts[1]=(int64_t)round(ARM_REACH_FULL_TRAVEL_COUNTS*0.60);
    Feedback(6U,simulated_counts[1],0x83U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);CHECK(last_pulses>0); /* Absolute CW coordinate can decrease during retraction. */
    CameraAdvanceTo(ARM_CAMERA_DONE);
    /* Reply None still requires actual new movement and reached/still feedback. */
    NativeSetup();fake_native_ack_enabled=0U;before_moves=moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(moves==before_moves+1U && arm_enabled && !fake_native_active);
    /* No ACK and no startup: SAME ABS retry may recover; HAL_OK alone never arrives. */
    NativeSetup();fake_native_ack_enabled=0U;before_moves=moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);fake_native_active=0U;
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(moves==before_moves+2U && !fake_native_active && last_pulses==2667 && last_speed==30U);
    NativeSetup();fake_native_ack_code=0xEEU;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !fake_native_active);
    /* 9F is a successful reached response, including reached-only reply mode. */
    NativeSetup();fake_native_ack_code=0x9FU;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    NativeSetup();fake_native_reached_reply_only=1U;
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_REACH,20U)==HAL_OK);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(arm_enabled && !fake_native_active);
    /* Reproduce accepted command at -54545 counts, flags81 (no REACHED).
     * Actual goal+stationary data must lead to STOP then new verification,
     * never immediate READY from a cached near-goal frame. */
    NativeSetup();fake_native_no_reached_flag=1U;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_target=-54545;
    {
        uint32_t before_stop=stops6;
        for(i=0U;i<50U && stops6==before_stop && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
        CHECK(stops6==before_stop+1U && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        for(i=0U;i<3U;i++)ArmCalibration_Poll();
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        CameraAdvanceTo(ARM_CAMERA_DONE);
        CHECK(arm_enabled && !fake_native_active && simulated_counts[1]==-54545);
    }
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_ENABLE);
    move_result=HAL_BUSY;CameraStep(1U);move_result=HAL_OK;
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !fake_native_active && !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH));
    /* Wrong factory pulse/input scaling must fail at the observed position. */
    NativeSetup();fake_native_scale=0.5f;CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    for(i=0U;i<70U && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !fake_native_active);
    NativeSetup();fake_native_scale=10.0f;CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    for(i=0U;i<70U && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !fake_native_active);
    /* Stop cancels internal motion; no background speed command takes over. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CameraStep(1U);CHECK(fake_native_active);
    CHECK(ArmCalibration_ProcessCommand("arm stop")==1U && !fake_native_active);
    before_moves=moves;before_velocity=velocity_calls;CameraStep(1U);
    CHECK(moves==before_moves && velocity_calls==before_velocity && ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    CHECK(ArmCalibration_ProcessCommand("arm camera mode encoder")==1U);
    encoder_sign[1]=1;fake_native_model=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraUnstarted(void)
{
    unsigned i;uint32_t before_moves,before_saves,before_velocity,before_clear;float pct;
    /* Field log: old REACHED83, raw231 (+1.269deg), no FD ACK. A tiny
     * near-zero jump must not enter the >2% residual/scale-failure path. */
    NativeSetup();fake_native_ack_enabled=0U;fake_native_drop_remaining=1U;
    before_moves=moves;before_saves=save_calls;before_velocity=velocity_calls;before_clear=clear_frames;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    simulated_counts[1]=231;fake_native_reached=1U;
    for(i=0;i<13;i++){CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && moves==before_moves+1U);}
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(moves==before_moves+2U && last_pulses==2667 && last_speed==30 && velocity_calls==before_velocity);
    CHECK(clear_frames==before_clear && save_calls==before_saves); /* No reverse2mm/zero/flash or blind full-stroke velocity. */
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&pct)==HAL_OK && fabsf(pct-41.1805f)<=ARM_PERCENT_TOLERANCE);
    /* Accepted ACK plus old REACHED still cannot prove startup. All three
     * queued-but-unexecuted commands exhaust the shared budget and STOP. */
    NativeSetup();fake_native_drop_remaining=3U;before_moves=moves;before_saves=save_calls;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);simulated_counts[1]=231;
    for(i=0;i<130 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && moves==before_moves+3U && !fake_native_active);
    CHECK(strstr(ArmCalibration_GetCameraFailureReason(),"failed to start")!=NULL && save_calls==before_saves);
    {camera_reach_fault_console[0]='\0';CHECK(ArmCalibration_ProcessCommand("arm camera status")==1);
     CHECK(strstr(camera_reach_fault_console,"at_fault reach")!=NULL && strstr(camera_reach_fault_console,"raw=231")!=NULL);}
    for(i=0;i<20;i++){CameraStep(1);}CHECK(moves==before_moves+3U);
    /* STOP during pending restart owns cancellation; delayed feedback cannot resend. */
    NativeSetup();fake_native_drop_remaining=1U;before_moves=moves;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    for(i=0;i<25 && !ArmDriver_ReachRetryRecovering();i++)CameraStep(1);
    CHECK(ArmDriver_ReachRetryRecovering());CHECK(ArmCalibration_ProcessCommand("arm stop")==1);
    for(i=0;i<25;i++)CameraStep(1);
    CHECK(moves==before_moves+1U && !fake_native_active && ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    fake_native_drop_remaining=0U;fake_native_model=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraResidual(void)
{
    uint32_t i,before_velocity,before_stop;float pct;
    int64_t endpoints[2]={-53587,-53738};unsigned sample;
    /* Both real logs: three trays already filled, return from near zero,
     * native REACHED at -294/-295deg. Do not fault or declare READY. */
    for(sample=0;sample<2;sample++) {
        NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
        fake_native_target=endpoints[sample];before_velocity=velocity_calls;before_stop=stops6;
        for(i=0;i<30 && stops6==before_stop && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
        CHECK(stops6==before_stop+1 && velocity_calls==before_velocity && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        for(i=0;i<10;i++)ArmCalibration_Poll();
        CHECK(velocity_calls==before_velocity && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        for(i=0;i<4 && velocity_calls==before_velocity;i++)CameraStep(1);
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE); /* no blind READY */
        for(i=0;i<100 && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++) {
            CameraStep(1);CHECK(abs(percent_motor_rpm[1])<=5);
        }
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && velocity_calls>before_velocity);
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&pct)==HAL_OK && fabsf(pct-41.1805f)<=ARM_PERCENT_TOLERANCE);
        CHECK(percent_motor_rpm[1]==0 && arm_enabled && motor.fb.enabled && !ArmTask_IsFaulted());
    }
    /* REACHED and position can be out of sync for a frame. New correct
     * position within the settling interval needs no speed correction. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_target=-53738;before_velocity=velocity_calls;
    for(i=0;i<20 && fake_native_active;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
    simulated_counts[1]=-54613;CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(velocity_calls==before_velocity);
    /* REACHED while still advancing must not cancel the native motion. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_reached=1;before_stop=stops6;before_velocity=velocity_calls;
    for(i=0;i<7;i++){CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE && stops6==before_stop);}
    CameraAdvanceTo(ARM_CAMERA_DONE);CHECK(velocity_calls==before_velocity);
    /* Cancel after native residual STOP and during the correction. No
     * later feedback may restart it; disabled/stale/protected must fail. */
    for(sample=0;sample<5;sample++) {
        NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
        fake_native_target=-53738;before_velocity=velocity_calls;
        for(i=0;i<40 && velocity_calls==before_velocity;i++)CameraStep(1);
        CHECK(velocity_calls>before_velocity && ArmCalibration_GetCameraState()==ARM_CAMERA_REACH_MOVE);
        before_velocity=velocity_calls;
        if(sample==0){CHECK(ArmCalibration_ProcessCommand("arm stop")==1);CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE && velocity_calls==before_velocity);}
        else if(sample==1){tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;motor.fb.last_tick=tick;ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);}
        else if(sample==2){injected_reach_stall=1;CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);}
        else if(sample==3){Feedback(6U,simulated_counts[1],0x89);ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);}
        else {Feedback(6U,simulated_counts[1],0x80);ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED);}
        CHECK(percent_motor_rpm[1]==0);
    }
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_target=-53738;before_stop=stops6;before_velocity=velocity_calls;
    for(i=0;i<30 && stops6==before_stop;i++)CameraStep(1);
    CHECK(stops6>before_stop && velocity_calls==before_velocity);
    CHECK(ArmCalibration_ProcessCommand("arm stop")==1);CameraStep(1);
    CHECK(velocity_calls==before_velocity && ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    /* A healthy-looking but immobile axis cannot make correction hang
     * forever or publish READY. Local correction budget is ten seconds. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    fake_native_target=-53738;before_velocity=velocity_calls;
    for(i=0;i<40 && velocity_calls==before_velocity;i++)CameraStep(1);
    CHECK(velocity_calls>before_velocity);encoder_gain[1]=0.0f;
    for(i=0;i<110 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && percent_motor_rpm[1]==0);
    CHECK(strstr(ArmCalibration_GetCameraFailureReason(),"residual correction timeout")!=NULL);
    encoder_sign[1]=1;fake_native_model=0;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestPickReachAcrossZero(void)
{
    unsigned int i;float target,current;uint8_t retract_seen=0U;
    CameraSetup();simulated_counts[1]=621;
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);CameraStep(0U);
    target=8.6188f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,target)==HAL_ERROR); /* Manual percent stays 0..100. */
    test_start_pose_active=1U;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,-2.0f)==HAL_ERROR); /* Beyond the 3mm allowance. */
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,-0.1f)==HAL_ERROR);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,target)==HAL_OK);
    for(i=0U;i<80U && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++) {
        CameraStep(0U);if(percent_motor_rpm[1]<0)retract_seen=1U;
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && retract_seen && simulated_counts[1]>621);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)==HAL_OK && fabsf(current-target)<=ARM_PERCENT_TOLERANCE);
    CHECK(percent_motor_rpm[1]==0);
    test_start_pose_active=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* New PICK target leaves the innermost stop toward the negative-angle side. */
    CameraSetup();CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);CameraStep(0U);
    target=-4.0f/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f;
    test_start_pose_active=1U;retract_seen=0U;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,target)==HAL_OK);
    for(i=0U;i<80U && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i++) {
        CameraStep(0U);if(percent_motor_rpm[1]>0)retract_seen=1U;
        CHECK(simulated_counts[1]<=0);
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && retract_seen && simulated_counts[1]<0);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&current)==HAL_OK && fabsf(current-target)<=ARM_PERCENT_TOLERANCE);
    CHECK(percent_motor_rpm[1]==0);
    test_start_pose_active=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
static void TestCameraNativeRetry(void)
{
    uint32_t before,clears,saves;unsigned i;
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    before=moves;clears=clear_frames;saves=save_calls;injected_reach_stall=auto_clear_reach_stall=1;
    for(i=0;i<100 && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && moves==before+1 && clear_frames==clears+1);
    CHECK(last_pulses==2667 && save_calls==saves && arm_enabled);
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    before=moves;clears=clear_frames;injected_reach_stall=1;auto_clear_reach_stall=0;
    for(i=0;i<100 && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && moves==before && clear_frames==clears+2);
    CHECK(!fake_native_active);injected_reach_stall=0;
}
static void TestCameraCustomPose(void)
{
    float angle,percent;uint32_t saves;
    CameraSetup();saves=save_calls;
    CHECK(ArmCalibration_ProcessCommand("arm camera set -30 -200 10")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1U && strstr(camera_status,"DM=+10.00"));
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CameraAdvanceTo(ARM_CAMERA_LIFT_FINAL);
    CHECK(ArmDriver_GetRotationDeg(&angle)==HAL_OK && fabsf(angle-10.0f)<0.1f);
    CHECK(fabs((double)simulated_counts[1]*360.0/65536.0+200.0)<4.0);
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK);
    CHECK(fabs((double)simulated_counts[0]*360.0/65536.0+30.0)<7.0 && save_calls==saves);
    CHECK(ArmCalibration_ProcessCommand("arm camera set -800 -200 90")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1U && strstr(camera_status,"DM=+10.00"));
    CHECK(ArmCalibration_ProcessCommand("arm camera defaults")==1U);
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    simulated_counts[0]=-2000; /* ~2.2mm TOP load deflection must not fault. */
    CameraAdvanceTo(ARM_CAMERA_DONE);
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(ArmCalibration_ProcessCommand("arm disable 6")==1U);
    CHECK(arm_enabled && ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT) && !ArmDriver_IsEnableRequested(ARM_DRIVER_REACH));
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_IDLE);
    CHECK(ArmCalibration_ProcessCommand("arm camera defaults")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm camera status")==1U && strstr(camera_status,"DM=-0.02"));
}
static void TestCameraDmHoldOnZdtFailure(void)
{
    uint32_t stops,disables,saves,i;
    float zero_before,zero_after;
    NativeSetup();fake_native_scale=0.5f;
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    CHECK(ArmDriver_GetRotationZeroRaw(&zero_before)==HAL_OK);
    stops=dm_stops;disables=dm_disables;saves=save_calls;
    for(i=0U;i<40U && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;i++)CameraStep(1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !fake_native_active);
    CHECK(arm_enabled && !arm_fault && dm_stops==stops && dm_disables==disables && NEAR(motor.target_pos_rad,0));
    CHECK(ArmDriver_GetRotationZeroRaw(&zero_after)==HAL_OK && NEAR(zero_before,zero_after) && save_calls==saves);
    CHECK(ArmDriver_GetRotationZeroRaw(NULL)==HAL_ERROR);
    /* Missing linear feedback can stop ZDT while healthy DM remains at zero. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    stops=dm_stops;tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;motor.fb.last_tick=tick;motor.fb.fb_count++;
    ArmCalibration_Poll();CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && arm_enabled && dm_stops==stops);
    /* The same failure with lost DM feedback must still release/latch safely. */
    NativeSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_REACH_MOVE);
    motor.fb.last_tick=tick-DM_LINK_TIMEOUT_MS_DEFAULT-1U;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && !arm_enabled);
    /* INIT does not unlock an existing healthy zero hold or rewrite DM zero. */
    NativeSetup();arm_enabled=1U;motor.fb.enabled=1U;motor.fb.err_code=1U;motor.target_pos_rad=0;
    stops=dm_stops;disables=dm_disables;
    CHECK(ArmCalibration_DeployCameraInit()==HAL_OK);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_REFERENCE && arm_enabled && dm_stops==stops && dm_disables==disables);
    CHECK(save_calls==saves && ArmDriver_GetRotationZeroRaw(&zero_after)==HAL_OK && NEAR(zero_after,0));
    tick+=5001U;motor.fb.last_tick=tick;motor.fb.fb_count++;
    ArmCalibration_Poll();
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_FAILED && arm_enabled && dm_disables==disables && dm_stops==stops);
    /* Manual stop is still an explicit release request after a ZDT-only fault. */
    CHECK(ArmCalibration_ProcessCommand("arm stop")==1U && !arm_enabled);
    encoder_sign[1]=1;fake_native_model=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
}
#endif

#ifndef TEST_DEFAULT_ZERO
static void Test_PacedArmReads(void)
{
    arm_cal_axis_status_t a5,a6,b6;arm_cal_query_status_t q5,q6,old5,old6;
    uint32_t i,old_slots=slots;float percent;uint32_t old_moves=moves,old_en=enable_calls;
    CHECK(ArmCalibration_Cancel()==HAL_OK);ArmDriver_ClearHeightRotationLimit();
    fake_query_model=1U;fake_query_drop_axis=fake_query_saw5=0U;fake_query_dropped=0U;
    for(i=0U;i<1200U;i+=5U){tick+=5U;motor.fb.last_tick=tick;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
    ArmCalibration_GetQueryStatus(ARM_DRIVER_LIFT,&q5);ArmCalibration_GetQueryStatus(ARM_DRIVER_REACH,&q6);
    CHECK(tick-a5.position_tick<200U && tick-a6.position_tick<200U);
    CHECK(tick-a5.flags_tick<200U && tick-a6.flags_tick<200U);
    CHECK(q5.queued_position && q5.queued_flags && q6.queued_position && q6.queued_flags);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK);
    CHECK(moves==old_moves && enable_calls==old_en); /* Reads do not enable or move. */
    /* HAL_BUSY cannot refresh feedback. Fair bounded retries reach both axes
     * instead of retaining a failed ID5 forever ahead of ID6. */
    slots=0U;for(i=0U;i<100U;i+=5U){tick+=5U;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);old5=q5;old6=q6;
    for(i=0U;i<800U;i+=5U){tick+=5U;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&b6);
    ArmCalibration_GetQueryStatus(ARM_DRIVER_LIFT,&q5);ArmCalibration_GetQueryStatus(ARM_DRIVER_REACH,&q6);
    CHECK(b6.position_sequence==a6.position_sequence && b6.position_tick==a6.position_tick);
    CHECK(q5.busy>old5.busy && q6.busy>old6.busy);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_ERROR);
    slots=old_slots;
    for(i=0U;i<800U;i+=5U){tick+=5U;motor.fb.last_tick=tick;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);CHECK(tick-a6.position_tick<200U);
    /* HAL enqueue success is not delivery. A missing ID6 reply times out and
     * retries once, yet healthy ID5 continues and stale ID6 remains invalid. */
    fake_query_drop_axis=6U;old6=q6;
    for(i=0U;i<100U;i+=5U){tick+=5U;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);
    for(i=0U;i<800U;i+=5U){tick+=5U;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&b6);ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&a5);
    ArmCalibration_GetQueryStatus(ARM_DRIVER_REACH,&q6);
    CHECK(b6.position_sequence==a6.position_sequence && b6.position_tick==a6.position_tick);
    CHECK(q6.no_reply>old6.no_reply && q6.retries>old6.retries && fake_query_dropped>0U);
    CHECK(tick-a5.position_tick<500U && ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_ERROR);
    fake_query_drop_axis=0U;
    for(i=0U;i<800U;i+=5U){tick+=5U;motor.fb.last_tick=tick;ArmCalibration_Poll();}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a6);CHECK(tick-a6.position_tick<200U);
    /* The actual user target runs with ONLY queued periodic CAN replies;
     * do not inject direct Feedback() to bypass the paced receive path. */
    CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_REACH,1U)==HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,41.18f)==HAL_OK);
    {double position=(double)fake_query_position[1];
      for(i=0U;i<14000U && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;i+=5U) {
        tick+=5U;motor.fb.last_tick=tick;
        position+=(double)percent_motor_rpm[1]*65536.0*5.0/60000.0;
        fake_query_position[1]=(int64_t)round(position);ArmCalibration_Poll();
      }
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && percent_motor_rpm[1]==0);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-41.18f)<=ARM_PERCENT_TOLERANCE);
    fake_query_model=0U;
}
#endif
#ifdef TEST_DEFAULT_ZERO
static int Test_DefaultDriverZero(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_cal_axis_status_t axis;float percent;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180.0f;cfg.rotation_max_deg=180.0f;
    ArmCalibration_DefaultConfig(&cal);cal.reach_reverse=1U;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    motor.fb.fb_count=1U;motor.fb.pos_rad=-33.0f*0.01745329252f;motor.fb.last_tick=tick;motor.fb.vel_rads=0.007f;
    CHECK(ArmDriver_Init(&cfg)==HAL_OK);CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    CHECK(ArmCalibration_UseTopGuard(100.0f,-20.0f,20.0f)==HAL_OK);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&axis);
    CHECK(axis.upper_valid && axis.upper_counts==0 && axis.zero_valid && axis.zero_counts==0 && !axis.position_sequence);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&axis);
    CHECK(axis.lower_valid && axis.lower_counts==0 && axis.zero_valid && axis.zero_counts==0 && !axis.position_sequence);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_ERROR); /* no fake current feedback */
    CHECK(ArmDriver_ReturnRotationZero()==HAL_ERROR && !arm_enabled); /* height still unknown */
    Feedback(5U,0,0U);Feedback(6U,0,0U);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && percent==0.0f);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && percent==0.0f);
    CHECK(ArmDriver_ReturnRotationZero()==HAL_OK && arm_enabled && motor.target_pos_rad==0.0f); /* no manual top5 */
    (void)ArmTask_Disable();
    Feedback(5U,-137190,0U);Feedback(6U,-66310,0U);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-50.0f)<0.01f);
    CHECK(ArmDriver_ReturnRotationZero()==HAL_ERROR && !arm_enabled); /* real descent>100mm */
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&axis);CHECK(axis.upper_counts==0 && axis.zero_counts==0);
    tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_ERROR);
    CHECK(ArmDriver_ReturnRotationZero()==HAL_ERROR && !arm_enabled);
    return (int)test_failures;
}
#endif
#ifdef TEST_DM_INTERNAL_ZERO
static int Test_InternalDmZero(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;float zero;uint32_t saves;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    motor.fb.pos_rad=0.9f;motor.fb.vel_rads=0.12454f;motor.fb.fb_count=1;motor.fb.last_tick=tick;
    CHECK(ARM_DM_SOFTWARE_ZERO_ENABLED==0U);
    CHECK(ArmDriver_Init(&cfg)==HAL_OK);ArmCalibration_DefaultConfig(&cal);CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    CHECK(ArmDriver_GetRotationZeroRaw(&zero)==HAL_OK && zero==0.0f && motor.pos_offset_rad==0.0f);
    CHECK(ArmDriver_ReturnRotationZero()==HAL_OK && arm_enabled && NEAR(motor.target_pos_rad,0.0f));
    CHECK(ArmDriver_Disable(ARM_DRIVER_ROTATION)==HAL_OK);motor.fb.vel_rads=0.6f;
    CHECK(ArmDriver_ReturnRotationZero()==HAL_BUSY && !arm_enabled);
    motor.fb.vel_rads=0.0f;saves=save_calls;
    CHECK(ArmCalibration_ProcessCommand("dm zero")==1U && save_calls==saves);
    CHECK(ArmCalibration_ProcessCommand("dm zero w25 save")==1U && save_calls==saves);
    CHECK(ArmCalibration_ProcessCommand("dm zero save")==1U && save_calls==saves+1U && !arm_enabled);
    CHECK(ArmDriver_GetRotationZeroRaw(&zero)==HAL_OK && zero==0.0f && motor.pos_offset_rad==0.0f);
    CHECK(ArmDriver_ReturnRotationZero()==HAL_OK && NEAR(motor.target_pos_rad,0.0f));
    return (int)test_failures;
}
#endif
#ifdef TEST_REPLACEMENT_REACH
static int TestReplacementReach(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_cal_axis_status_t a;
    int32_t pulses;uint8_t reverse;float percent;
    ArmDriver_DefaultConfig(&cfg);ArmCalibration_DefaultConfig(&cal);
    CHECK(cfg.reach.reverse==1U && cfg.reach.speed_rpm==60U && cfg.lift.reverse==0U);
    cal.reach_reverse=cfg.reach.reverse;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    motor.fb.fb_count=1;motor.fb.last_tick=tick;zdt.can_ready=1U;
    CHECK(ArmDriver_Init(&cfg)==HAL_OK && ArmCalibration_Init(&cal)==HAL_OK);
    Feedback(5U,0,0x81U);Feedback(6U,0,0x81U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmDriver_GetLinearDirection(ARM_DRIVER_REACH,&reverse)==HAL_OK && reverse==1U);
    encoder_sign[1]=1; /* Replacement physical fixture follows wire sign. */
    PercentArrival(ARM_DRIVER_REACH,2.0f);
    CHECK(simulated_counts[1]<0);
    CHECK(ArmCalibration_GetAbsoluteReachPulses(25.416f,&pulses)==HAL_OK && pulses==1646);
    fake_native_model=1;native_encoder_sign=1;
    CHECK(ArmDriver_ReachAbsolutePulses(pulses,60,ARM_CAMERA_POSITION_ACCEL)==HAL_OK);
    CHECK(last_pulses==-pulses && fake_native_target<0);
    CHECK(llabs(fake_native_target+33710)<=12); /* -185deg, not the old-profile -370deg. */
    simulated_counts[1]=-51785; /* Logged -284.464deg start must now move inward. */
    CHECK(ArmDriver_ReachAbsolutePulses(pulses,60,ARM_CAMERA_POSITION_ACCEL)==HAL_OK);
    CHECK(fake_native_target>simulated_counts[1]);
    simulated_counts[1]=fake_native_target;Feedback(6U,simulated_counts[1],0x83U);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&percent)==HAL_OK && fabsf(percent-25.416f)<0.01f);
    fake_native_model=0;CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);
    PercentArrival(ARM_DRIVER_REACH,2.0f); /* opposite wire direction returns inward */
    {
        uint32_t before_moves=moves;
        CHECK(ArmCalibration_ProcessCommand("arm pos 6 -185.1581")==1U);
        CHECK(moves==before_moves+1U && last_pulses==-1646 && last_speed==60);
        CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);before_moves=moves;
        CHECK(ArmCalibration_ProcessCommand("arm pos 6 -10000")==1U && moves==before_moves);
    }
    /* Raw angle sign is not falsified: both signed wire replies are preserved. */
    Feedback(6U,65536,0x81);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);CHECK(a.position_counts==65536);
    Feedback(6U,-65536,0x85);ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&a);
    CHECK(a.position_counts==-65536 && (a.flags&ARM_CAL_FLAG_STALL));
    CHECK(save_calls==0 && a.zero_counts==0);
    CHECK(ArmCalibration_ProcessCommand("arm dir 6 0")==1);
    CHECK(ArmDriver_GetLinearDirection(ARM_DRIVER_REACH,&reverse)==HAL_OK && reverse==0);
    return (int)test_failures;
}
#endif
#ifdef TEST_REACH_RETRY
static void RetryPosition(uint8_t flags,uint32_t ms,int64_t pos)
{
    tick+=ms;Feedback(5U,0,0x81);Feedback(6U,pos,flags);ArmDriver_PollReachRetry();
}
static void RetrySample(uint8_t flags,uint32_t ms){RetryPosition(flags,ms,-1000);}
static void RetryNewJob(void)
{
    (void)ArmCalibration_Cancel();(void)ArmDriver_StopAll();
    stop_reach_result=clear_tx_result=move_result=step_enable_result=HAL_OK;slots=8;bus_off=0;
    Feedback(5U,0,0x81);Feedback(6U,-1000,0x81);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmDriver_ReachAbsolutePulses(2555,60,150)==HAL_OK);
}
static void RetryConfirmStall(void)
{
    RetrySample(0x85,10);CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering());
    RetrySample(0x85,100);RetrySample(0x85,100);RetrySample(0x85,100);
    CHECK(!ArmDriver_ReachStallPending());
}
static void RetryCompleteRecovery(void)
{
    RetryConfirmStall();CHECK(ArmDriver_ReachRetryRecovering());
    RetrySample(0x85,10); /* STOP */
    RetrySample(0x85,150); /* clear */
    RetrySample(0x81,10); /* new clear feedback */
    RetrySample(0x81,10); /* enable */
    RetrySample(0x81,10); /* new enable feedback */
    RetrySample(0x81,10); /* reverse retreat send */
    CHECK(last_speed<=5 && last_pulses==0 && ArmDriver_ReachRetryRecovering());
    RetryPosition(0x81,100,0); /* retreat arrival STOP */
    RetryPosition(0x81,100,0);RetryPosition(0x81,100,0);RetryPosition(0x81,100,0); /* new held samples */
    RetryPosition(0x81,10,0); /* original ABS resend */
    CHECK(!ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachRetryFailed() && last_pulses==-2555);
}
static void RetryRetreatAt(int64_t position,int32_t goal)
{
    unsigned k;
    RetryNewJob();Feedback(6U,position,0x81);
    CHECK(ArmDriver_ReachAbsolutePulses(goal,60,150)==HAL_OK);
    for(k=0;k<4;k++)RetryPosition(0x85,k?100:10,position);
    CHECK(ArmDriver_ReachRetryRecovering());
    RetryPosition(0x85,10,position);RetryPosition(0x85,150,position);
    RetryPosition(0x81,10,position);RetryPosition(0x81,10,position);
    RetryPosition(0x81,10,position);RetryPosition(0x81,10,position);
    CHECK(ArmDriver_ReachRetryRecovering() && last_speed==5);
}
static void TestReverseRetreat(void)
{
    unsigned direction,k,stage;uint32_t before,gen;int32_t retreat;
    int64_t held;double mm=fabs(ARM_REACH_FULL_TRAVEL_COUNTS)/ARM_REACH_FULL_TRAVEL_MM;
    /* Both inward and outward jobs back off opposite their original move.
     * The backoff uses ABS coordinates without replacing the cached goal. */
    for(direction=0;direction<2;direction++) {
        int32_t goal=direction?200:4000;
        RetryRetreatAt(-40000,goal);retreat=last_pulses;
        /* last_pulses is the signed wire target, whereas public goal is positive. */
        held=(int64_t)llround((double)retreat*65536.0/ARM_CAMERA_POSITION_STEPS_REV);
        CHECK(fabs(fabs((double)held+40000)/mm-2.0)<0.03);
        CHECK(direction?held<-40000:held>-40000);
        before=moves;gen=ArmDriver_ReachRetryGeneration();
        tick+=100;ArmDriver_PollReachRetry();CHECK(moves==before && ArmDriver_ReachRetryGeneration()==gen);
        RetryPosition(0x81,10,held); /* STOP at retreat endpoint */
        CHECK(ArmDriver_ReachRetryRecovering() && last_pulses==retreat);
        tick+=250;ArmDriver_PollReachRetry(); /* cached data is not held confirmation */
        CHECK(moves==before && ArmDriver_ReachRetryRecovering());
        for(k=0;k<ARM_MOTION_SETTLE_SAMPLES;k++){RetryPosition(0x81,50,held);CHECK(moves==before);}
        RetryPosition(0x81,10,held);
        CHECK(moves==before+1 && last_pulses==-goal && last_speed==30);
        CHECK(ArmDriver_ReachRetryGeneration()==gen+1 && !ArmDriver_ReachRetryRecovering());
    }
    /* Internal arrival STOP keeps the original monitor alive. Public STOP,
     * disable, protection and stale feedback must still prevent restart. */
    RetryNewJob();CHECK(ArmDriver_StopReachForHold()==HAL_OK && ArmDriver_ReachRetryActive());
    RetryConfirmStall();CHECK(ArmDriver_ReachRetryRecovering());
    for(stage=0;stage<3;stage++) {
        RetryRetreatAt(-40000,4000);held=(int64_t)llround((double)last_pulses*65536.0/ARM_CAMERA_POSITION_STEPS_REV);
        if(stage)RetryPosition(0x81,100,held);
        if(stage==2)for(k=0;k<3;k++)RetryPosition(0x81,100,held);
        CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);before=moves;
        for(k=0;k<15;k++)RetryPosition(0x81,100,held);
        CHECK(moves==before && !ArmDriver_ReachRetryActive());
    }
    RetryRetreatAt(-40000,4000);before=moves;RetryPosition(0x89,10,-40000);
    CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryRetreatAt(-40000,4000);before=moves;tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;ArmDriver_PollReachRetry();
    CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryRetreatAt(-40000,4000);before=moves;RetryPosition(0x80,10,-40000);
    CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryRetreatAt(-40000,4000);before=moves;RetryPosition(0x81,10,-41000);
    CHECK(ArmDriver_ReachRetryFailed() && moves==before); /* wrong reverse direction */
    /* Backoff itself stalled: consume the remaining recovery, never loop
     * forever and never report an unperformed retreat as confirmed. */
    RetryRetreatAt(-40000,4000);before=moves;
    for(k=0;k<4;k++)RetryPosition(0x85,k?100:10,-40000);
    CHECK(ArmDriver_ReachRetryRecovering() && moves==before && ArmDriver_ReachRetrySpeedRpm()==15);
    RetryPosition(0x85,10,-40000);RetryPosition(0x85,150,-40000);
    for(k=0;k<4;k++)RetryPosition(0x81,10,-40000);
    CHECK(moves==before+1 && last_speed==5);
    for(k=0;k<4;k++)RetryPosition(0x85,k?100:10,-40000);
    CHECK(ArmDriver_ReachRetryFailed());
    before=moves;for(k=0;k<20;k++)RetryPosition(0x81,100,-40000);CHECK(moves==before);
}
static int TestReachRetry(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;unsigned k,j;uint32_t before,gen,clears;
    ArmDriver_DefaultConfig(&cfg);ArmCalibration_DefaultConfig(&cal);cal.reach_reverse=cfg.reach.reverse;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    motor.fb.fb_count=1;motor.fb.last_tick=tick;zdt.can_ready=1;
    CHECK(ArmDriver_Init(&cfg)==HAL_OK && ArmCalibration_Init(&cal)==HAL_OK);
    /* Reassertions refine the SAME ABS target without resetting the retry
     * attempts. Reject a different goal, and never overwrite recovery. */
    RetryNewJob();before=moves;gen=ArmDriver_ReachRetryGeneration();
    CHECK(ArmDriver_ReassertReachAbsolutePulses(2556,15)==HAL_ERROR && moves==before);
    CHECK(ArmDriver_ReassertReachAbsolutePulses(2555,15)==HAL_OK && moves==before+1);
    CHECK(last_pulses==-2555 && last_speed==15 && ArmDriver_ReachRetryGeneration()==gen+1);
    RetryCompleteRecovery();
    CHECK(ArmDriver_ReassertReachAbsolutePulses(2555,15)==HAL_OK);
    RetryCompleteRecovery();
    CHECK(ArmDriver_ReassertReachAbsolutePulses(2555,15)==HAL_OK);
    RetryConfirmStall();CHECK(ArmDriver_ReachRetryFailed()); /* Still three attempts total. */
    before=moves;CHECK(ArmDriver_ReassertReachAbsolutePulses(2555,15)==HAL_BUSY && moves==before);
    RetryNewJob();RetryConfirmStall();before=moves;
    CHECK(ArmDriver_ReassertReachAbsolutePulses(2555,15)==HAL_BUSY && moves==before);
    CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);
    /* An isolated/two-frame Cgi flag must not STOP the axis, pause its owner
     * or consume a native retry. Cgp protection remains immediate below. */
    RetryNewJob();before=moves;clears=clear_frames;gen=stops6;
    RetrySample(0x85,10);CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering());
    RetrySample(0x85,100);RetrySample(0x81,100);
    CHECK(!ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering() && moves==before && clear_frames==clears && stops6==gen);
    /* A continuously asserted flag while actual error is shrinking is NOT
     * a confirmed no-progress stall. Use NEW position and flag samples. */
    RetryNewJob();before=moves;clears=clear_frames;gen=stops6;
    RetrySample(0x85,10);
    for(k=1;k<=12;k++) {
        tick+=100;Feedback(5U,0,0x81);Feedback(6U,-1000-(int64_t)k*128,0x85);ArmDriver_PollReachRetry();
        CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachRetryFailed());
        CHECK(moves==before && clear_frames==clears && stops6==gen);
    }
    RetrySample(0x81,10);CHECK(!ArmDriver_ReachStallPending() && stops6==gen);
    RetryNewJob();before=moves;gen=stops6;RetrySample(0x85,10);
    for(k=1;k<=12;k++) {
        tick+=100;Feedback(5U,0,0x81);Feedback(6U,-1000-(int64_t)(k/2U)*20,0x85);ArmDriver_PollReachRetry();
        CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering() && moves==before && stops6==gen);
    }
    RetrySample(0x81,10);CHECK(!ArmDriver_ReachStallPending()); /* One microstep every200ms is genuine progress. */
    /* Cached snapshots cannot satisfy the sample count just because time
     * advances. After real freshness expires, fail without a blind resend. */
    RetryNewJob();before=moves;RetrySample(0x85,10);
    tick+=350;ArmDriver_PollReachRetry();CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering());
    tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS-350U+1U;ArmDriver_PollReachRetry();CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryNewJob();before=moves;gen=stops6;RetrySample(0x85,10);
    {
        FDCAN_RxHeaderTypeDef h={0};uint8_t flags[3]={0x3A,0x85,0x6B};
        h.Identifier=6U<<8U;h.IdType=FDCAN_EXTENDED_ID;h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;
        for(k=0;k<3;k++){tick+=100;CHECK(ArmCalibration_OnCanFrame(&h,flags,3)==1);ArmDriver_PollReachRetry();}
        CHECK(ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering() && moves==before && stops6==gen);
    }
    RetryNewJob();RetrySample(0x85,10);CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);
    before=moves;for(k=0;k<10;k++)RetrySample(0x81,100);
    CHECK(!ArmDriver_ReachStallPending() && !ArmDriver_ReachRetryRecovering() && moves==before);
    RetryNewJob();RetrySample(0x85,10);Feedback(5U,0,0x85);ArmDriver_PollReachRetry();
    CHECK(ArmDriver_ReachRetryFailed()); /* Actual lift fault is immediate during observation. */
    /* Observe across millis wrap; real persistent no-progress still retries. */
    tick=UINT32_MAX-150U;RetryNewJob();RetryConfirmStall();CHECK(ArmDriver_ReachRetryRecovering());
    CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK && !ArmDriver_ReachStallPending());
    RetryNewJob();before=moves;gen=ArmDriver_ReachRetryGeneration();clears=clear_frames;
    CHECK(last_speed==60U && ArmDriver_ReachRetrySpeedRpm()==60U);
    RetryCompleteRecovery();
    CHECK(last_speed==30U && ArmDriver_ReachRetrySpeedRpm()==30U);
    CHECK(ArmDriver_ReachRetryTimeoutMs(655360.0)==45000U);
    CHECK(moves==before+2 && clear_frames==clears+1 && ArmDriver_ReachRetryGeneration()==gen+1);
    RetrySample(0x83,10);CHECK(!ArmDriver_ReachRetryFailed());
    RetryNewJob();CHECK(ArmDriver_ReachAbsolutePulses(2555,15,150)==HAL_OK);RetryCompleteRecovery();
    CHECK(last_speed==15U && last_pulses==-2555); /* Recovery never speeds up a slow requested job. */
    RetryNewJob();before=moves;gen=ArmDriver_ReachRetryGeneration();
    RetryCompleteRecovery();RetryCompleteRecovery();
    CHECK(last_speed==15U && ArmDriver_ReachRetrySpeedRpm()==15U && ArmDriver_ReachRetryTimeoutMs(655360.0)==85000U);
    CHECK(moves==before+4 && ArmDriver_ReachRetryGeneration()==gen+2);
    before=stops6;RetryConfirmStall();
    CHECK(ArmDriver_ReachRetryFailed() && stops6>before && strstr(last_console,"attempts=3/3"));
    before=moves;for(k=0;k<30;k++)RetrySample(0x81,100);CHECK(moves==before);
    before=stops5;ArmCalibration_Poll();CHECK(stops5>before && ArmCalibration_GetState()==ARM_CAL_FAILED);
    /* Clear never succeeds: bounded recovery budget, no blind movement resend. */
    RetryNewJob();before=moves;clears=clear_frames;
    for(k=0;k<60 && !ArmDriver_ReachRetryFailed();k++)RetrySample(0x85,100);
    CHECK(ArmDriver_ReachRetryFailed() && moves==before && clear_frames==clears+2);
    /* Cancel at every recovery phase; queued feedback cannot resurrect motion. */
    for(k=0;k<6;k++){
        RetryNewJob();RetryConfirmStall();
        for(j=0;j<k;j++)RetrySample(j<2?0x85:0x81,j==1?150:10);
        CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);before=moves;
        for(j=0;j<30;j++)RetrySample(0x81,100);
        CHECK(moves==before && !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachRetryFailed());
    }
    RetryNewJob();RetryConfirmStall();CHECK(ArmDriver_Disable(ARM_DRIVER_REACH)==HAL_OK);before=moves;
    for(k=0;k<20;k++){RetrySample(0x80,100);}
    CHECK(moves==before && !ArmDriver_ReachRetryRecovering());
    RetryNewJob();before=moves;RetrySample(0x89,10);CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryNewJob();before=moves;tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;ArmDriver_PollReachRetry();CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryNewJob();RetryConfirmStall();stop_reach_result=HAL_ERROR;RetrySample(0x85,10);
    CHECK(ArmDriver_ReachRetryFailed());stop_reach_result=HAL_OK;
    RetryNewJob();RetryConfirmStall();RetrySample(0x85,10);clear_tx_result=HAL_ERROR;RetrySample(0x85,150);
    CHECK(ArmDriver_ReachRetryFailed());clear_tx_result=HAL_OK;
    /* Old flags cannot confirm clear/enable, even after a time delay. */
    RetryNewJob();RetryConfirmStall();RetrySample(0x85,10);RetrySample(0x85,150);before=moves;
    tick+=10;ArmDriver_PollReachRetry();CHECK(ArmDriver_ReachRetryRecovering() && moves==before);
    RetrySample(0x81,10);RetrySample(0x81,10);
    tick+=10;ArmDriver_PollReachRetry();CHECK(ArmDriver_ReachRetryRecovering() && moves==before);
    RetrySample(0x81,10);slots=0;RetrySample(0x81,10);CHECK(moves==before);
    slots=8;RetrySample(0x81,10);CHECK(moves==before+1 && ArmDriver_ReachRetryRecovering() && last_speed<=5);
    /* During simultaneous descent/extension, clear6 must not cancel or wait
     * for the independent lift calibration transaction. */
    RetryNewJob();
    CHECK(ArmCalibration_UseTopGuard(150,0,44)==HAL_OK);
    CHECK(ArmDriver_CheckUiPose(150,100)==HAL_OK);
    CHECK(ArmDriver_CheckUiPose(150.1f,100)==HAL_ERROR);
    CHECK(ArmDriver_CheckUiPose(200,44)==HAL_OK && ArmDriver_CheckUiPose(200,-0.1f)==HAL_ERROR);
    CHECK(ArmDriver_CheckUiPose(NAN,0)==HAL_ERROR);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,2.0f)==HAL_OK);
    before=ArmDriver_ReachRetryGeneration();RetryCompleteRecovery();
    CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE && ArmDriver_ReachRetryGeneration()==before+1);
    /* Pose recovery holds only lift5; it must not cancel6's cached target,
     * native transaction or retry generation/budget. */
    before=ArmDriver_ReachRetryGeneration();slots=stops6;
    CHECK(ArmCalibration_HoldLiftForReachRecovery()==HAL_OK);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && percent_motor_rpm[0]==0 && stops6==slots && ArmDriver_ReachRetryGeneration()==before);
    CHECK(ArmDriver_ReachRetryActive());
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,3.0f)==HAL_OK);
    stop_lift_result=HAL_ERROR;CHECK(ArmCalibration_HoldLiftForReachRecovery()==HAL_ERROR);
    CHECK(ArmCalibration_GetState()==ARM_CAL_PERCENT_MOVE);stop_lift_result=HAL_OK;
    CHECK(ArmCalibration_HoldLiftForReachRecovery()==HAL_OK && ArmDriver_ReachRetryActive());
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    RetryNewJob();RetryConfirmStall();before=moves;
    tick+=10;Feedback(5U,0,0x85);Feedback(6U,-1000,0x85);ArmDriver_PollReachRetry();
    CHECK(ArmDriver_ReachRetryFailed() && moves==before);
    RetryNewJob();RetryConfirmStall();before=moves;motor.fb.err_code=8;RetrySample(0x85,10);
    CHECK(ArmDriver_ReachRetryFailed() && moves==before);motor.fb.err_code=0;
    TestReverseRetreat();
    return (int)test_failures;
}
#endif
#ifdef TEST_28_STALL_TRIAL
volatile unsigned test_trial_camera_state,test_trial_cal_state;
char test_trial_camera_reason[96];
static int Test28StallTrial(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_cal_axis_status_t snapshot;
    unsigned j;uint32_t before,clears,generation;int32_t goal;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180.0f;cfg.rotation_max_deg=180.0f;
    ArmCalibration_DefaultConfig(&cal);cal.reach_reverse=cfg.reach.reverse;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    motor.fb.fb_count=1;motor.fb.last_tick=tick;zdt.can_ready=1;
    CHECK(ArmDriver_Init(&cfg)==HAL_OK && ArmCalibration_Init(&cal)==HAL_OK);
    CHECK(ARM_28_STALL_IGNORE_TRIAL==1);
    CHECK(!ArmCalibration_StallFault(ARM_DRIVER_REACH,0x85));
    CHECK(ArmCalibration_StallFault(ARM_DRIVER_LIFT,0x85));
    CHECK(ArmCalibration_FaultFlags(ARM_DRIVER_REACH,0x89)==ARM_CAL_FLAG_PROTECTED);
    RetryNewJob();before=moves;clears=clear_frames;generation=ArmDriver_ReachRetryGeneration();goal=last_pulses;
    for(j=0;j<16;j++) {
        RetryPosition(0x85,150,-40000);
        CHECK(ArmDriver_ReachRetryActive() && !ArmDriver_ReachRetryRecovering() && !ArmDriver_ReachRetryFailed());
        CHECK(last_pulses==goal && last_speed==60 && ArmDriver_ReachRetryGeneration()==generation);
    }
    CHECK(clear_frames>=clears+10 && moves>=before+10); /* More than3 clears/replays, no retreat. */
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&snapshot);
    CHECK(snapshot.flags==0x85 && snapshot.position_counts==-40000); /* No forged flags/position. */
    CHECK(ArmDriver_Stop(ARM_DRIVER_REACH)==HAL_OK);before=moves;clears=clear_frames;
    for(j=0;j<8;j++)RetryPosition(0x85,150,-40000);
    CHECK(moves==before && clear_frames==clears); /* STOP has no active goal to revive. */
    /* Busy CAN is retried later; genuine CAN error still fails. */
    RetryNewJob();clear_tx_result=HAL_BUSY;before=moves;RetryPosition(0x85,150,-40000);
    CHECK(!ArmDriver_ReachRetryFailed() && moves==before);
    clear_tx_result=HAL_OK;RetryPosition(0x85,150,-40000);CHECK(moves>before && !ArmDriver_ReachRetryFailed());
    clear_tx_result=HAL_ERROR;RetryPosition(0x85,150,-40000);CHECK(ArmDriver_ReachRetryFailed());
    clear_tx_result=HAL_OK;ArmCalibration_Cancel();
    /* Persistent raw STALL through every CAMERA stage does not pause/fail
     * DM or the real measured reach movement; actual end position required. */
    CameraSetup();reported_reach_stall=1;CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    for(j=0;j<150 && ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE && ArmCalibration_GetCameraState()!=ARM_CAMERA_FAILED;j++)CameraStep(1);
    reported_reach_stall=0;
    test_trial_camera_state=ArmCalibration_GetCameraState();test_trial_cal_state=ArmCalibration_GetState();
    strncpy(test_trial_camera_reason,ArmCalibration_GetCameraFailureReason(),sizeof(test_trial_camera_reason)-1U);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE);
    CHECK(fabs((double)simulated_counts[1]*360.0/65536.0+300.0)<4.0);
    CameraSetup();CHECK(ArmCalibration_SeekEndpoint(ARM_DRIVER_REACH,0)==HAL_ERROR);
    /* Percentage mode does not fail on6 STALL; replayed velocity stays under
     * the configured cap.5 STALL and6 PROTECTED remain real faults. */
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,20)==HAL_OK);
    for(j=0;j<10;j++) {
        tick+=100;motor.fb.last_tick=tick;Feedback(5,0,0x81);Feedback(6,0,0x85);ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()!=ARM_CAL_FAILED && abs(percent_motor_rpm[1])<=60);
    }
    CHECK(ArmCalibration_Cancel()==HAL_OK);CameraSetup();
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,20)==HAL_OK);
    tick+=100;motor.fb.last_tick=tick;Feedback(5,0,0x85);Feedback(6,0,0x81);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED);CHECK(ArmCalibration_Cancel()==HAL_OK);
    CameraSetup();CHECK(ArmDriver_Enable(ARM_DRIVER_REACH)==HAL_OK);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_REACH,20)==HAL_OK);
    tick+=100;motor.fb.last_tick=tick;Feedback(5,0,0x81);Feedback(6,0,0x89);ArmCalibration_Poll();
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED);CHECK(ArmCalibration_Cancel()==HAL_OK);
    RetryNewJob();before=clear_frames;tick+=801;motor.fb.last_tick=tick;ArmDriver_PollReachRetry();
    CHECK(ArmDriver_ReachRetryFailed() && clear_frames==before); /* Missing real feedback is not ignored. */
    return (int)test_failures;
}
#endif
#ifdef TEST_FEEDBACK_WAIT
static int TestFeedbackWait(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_feedback_wait_t wait={0};
    arm_cal_axis_status_t before,after;uint32_t gap;unsigned j;float pct;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    zdt.can_ready=1;
    CHECK(ArmDriver_Init(&cfg)==HAL_OK);ArmCalibration_DefaultConfig(&cal);
    CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    Feedback(5U,0,0x81);Feedback(6U,0,0x83);
    CHECK(ArmCalibration_ZdtFeedbackFresh());
    CHECK(!ArmCalibration_WaitForFeedback(&wait,tick,&gap) && gap==0);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&before);
    tick+=900U;
    CHECK(ArmCalibration_WaitForFeedback(&wait,tick,&gap) && gap==0);
    for(j=0;j<12;j++){tick+=250U;CHECK(ArmCalibration_WaitForFeedback(&wait,tick,&gap) && gap==250U);}
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&after);
    CHECK(after.position_tick==before.position_tick && after.position_sequence==before.position_sequence);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_REACH,&pct)==HAL_ERROR);
    CHECK(ArmCalibration_FeedbackWaiting());
    Feedback(5U,0,0x81);Feedback(6U,0,0x83);
    CHECK(!ArmCalibration_WaitForFeedback(&wait,tick,&gap) && !ArmCalibration_FeedbackWaiting());
    tick+=900U;Feedback(5U,0,0x81);
    CHECK(ArmCalibration_WaitForFeedback(&wait,tick,&gap)); /* Only6 is stale. */
    Feedback(6U,0,0x83);CHECK(!ArmCalibration_WaitForFeedback(&wait,tick,&gap));
    tick+=900U;Feedback(6U,0,0x83);
    CHECK(ArmCalibration_WaitForFeedback(&wait,tick,&gap)); /* Only5 is stale. */
    Feedback(6U,0,0x89);
    CHECK(!ArmCalibration_WaitForFeedback(&wait,tick,&gap)); /* Fresh protection is not masked. */
    Feedback(5U,0,0x81);Feedback(6U,0,0x83);
    /* Reproduce the reported CAMERA pause: missing5/6 replies for longer
     * than2s must not publish FAILED, advance motion or rewrite samples. */
    CameraSetup();CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&before);
    tick+=900U;
    for(j=0;j<20;j++) {
        tick+=250U;motor.fb.last_tick=tick;motor.fb.fb_count++;
        ArmCalibration_Poll();
        CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME);
        CHECK(ArmCalibration_GetState()!=ARM_CAL_FAILED);
    }
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&after);
    CHECK(after.position_sequence==before.position_sequence && after.position_tick==before.position_tick);
    CameraStep(1U);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DONE && !ArmCalibration_FeedbackWaiting());
    /* Native position job keeps its driver-owned endpoint during stale
     * replies; returning telemetry does not STOP/restart a velocity loop. */
    CameraSetup();CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);CameraStep(0);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80.0f)==HAL_OK);
    for(j=0;j<3;j++)CameraStep(0);
    CHECK(fake_lift_active && percent_motor_rpm[0]==0);
    {uint32_t before_stops=stops5,before_moves=native_lift_moves,before_velocity=velocity_calls;
    tick+=900U;motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(fake_lift_active && stops5==before_stops && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    for(j=0;j<12;j++){tick+=250U;motor.fb.last_tick=tick;ArmCalibration_Poll();}
    CHECK(ArmCalibration_GetState()!=ARM_CAL_FAILED && fake_lift_active && stops5==before_stops);
    CameraStep(0);CameraStep(0);CHECK(native_lift_moves==before_moves && velocity_calls==before_velocity);}
    CHECK(ArmCalibration_Cancel()==HAL_OK && !ArmCalibration_FeedbackWaiting());
    return (int)test_failures;
}
#endif
#ifdef TEST_NATIVE_LIFT
static void TestNativeAckSnapshots(void)
{
    FDCAN_RxHeaderTypeDef h={0};uint8_t data[3]={0xFD,0x02,0x6B};
    arm_cal_position_ack_t before={0},after={0},reach={0};uint32_t fence,queued,received;
    h.Identifier=5U<<8;h.IdType=FDCAN_EXTENDED_ID;h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;
    ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&before);fence=ArmCalibration_PositionAckFence(ARM_DRIVER_LIFT);
    received=tick;CHECK(ArmCalibration_PostCanFrame(&h,data,3)==HAL_OK);
    queued=ArmCalibration_PositionAckFence(ARM_DRIVER_LIFT);CHECK(queued-fence==1);
    ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&after);CHECK(after.sequence==before.sequence);
    ArmCalibration_Poll();ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&after);
    CHECK(after.sequence==queued && after.received_tick==received && after.response==0x02);
    /* This queue was fenced before draining: it remains an OLD reply to a
     * subsequent command, rather than being given a new consumer sequence. */
    CHECK((int32_t)(after.sequence-queued)==0);
    h.Identifier=6U<<8;tick++;CHECK(ArmCalibration_PostCanFrame(&h,data,3)==HAL_OK);
    ArmCalibration_Poll();ArmCalibration_GetPositionAck(ARM_DRIVER_REACH,&reach);
    ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&before);
    CHECK(reach.response==0x02 && before.sequence==after.sequence);
    h.Identifier=5U<<8;data[2]=0;fence=ArmCalibration_PositionAckFence(ARM_DRIVER_LIFT);
    CHECK(ArmCalibration_PostCanFrame(&h,data,3)==HAL_OK);
    CHECK(ArmCalibration_PositionAckFence(ARM_DRIVER_LIFT)==fence);
    ArmCalibration_Poll();ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&before);CHECK(before.sequence==after.sequence);
    data[2]=0x6B;h.Identifier=7U<<8;CHECK(ArmCalibration_PostCanFrame(&h,data,3)==HAL_ERROR);
    h.Identifier=5U<<8;CHECK(ArmCalibration_PostCanFrame(&h,data,2)==HAL_ERROR);
    data[1]=0xEE;tick++;CHECK(ArmCalibration_PostCanFrame(&h,data,3)==HAL_OK);
    ArmCalibration_Poll();ArmCalibration_GetPositionAck(ARM_DRIVER_LIFT,&before);
    CHECK((int32_t)(before.sequence-fence)>0 && before.response==0xEE && before.received_tick==tick);
}
static void NativeLiftReference(int64_t zero)
{
    unsigned j;
    CameraSetup();SetTestGrip(0U);
    simulated_counts[0]=zero;Feedback(5U,zero,0x83U);Feedback(6U,0,0x81U);
    CHECK(ArmCalibration_ProcessCommand("arm top 5")==1U);
    for(j=0;j<4;j++) {
        tick+=100;motor.fb.last_tick=tick;Feedback(5U,zero,0x83U);Feedback(6U,0,0x81U);ArmCalibration_Poll();
    }
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(ArmCalibration_UseTopGuard(100.0f,-20.0f,20.0f)==HAL_OK);
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,198U)==HAL_OK);
    ArmCalibration_Poll();
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
}
static void NativeLiftFinish(void)
{
    unsigned j;for(j=0;j<150 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;j++)CameraStep(0);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
}
static void TestLiftCommonAckRetries(void)
{
    unsigned up,lost,j;uint32_t issued,started,stopped;int32_t goal;
    /* Both directions, including the logged return-to-TOP case. Simulate
     * complete command loss (HAL0 but no driver execution or ACK). */
    for(up=0;up<2;up++)for(lost=0;lost<=3;lost++) {
        NativeLiftReference(0);CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,238U)==HAL_OK);
        simulated_counts[0]=up?(int64_t)round(0.75*ARM_LIFT_FULL_TRAVEL_COUNTS):0;
        tick+=10;motor.fb.last_tick=tick;Feedback(5,simulated_counts[0],0x83);Feedback(6,0,0x81);ArmCalibration_Poll();
        fake_lift_drop_commands=(uint8_t)lost;issued=native_lift_moves;stopped=stops5;started=tick;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,up?0:80)==HAL_OK);
        CameraStep(0);goal=last_lift_pulses;CHECK(native_lift_moves==issued+1);
        for(j=0;j<150 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;j++) {
            CameraStep(0);CHECK(last_lift_pulses==goal && fake_lift_speed==410 && last_lift_acceleration==234);
            CHECK(native_lift_moves<=issued+3);
        }
        if(lost<3) {
            CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && native_lift_moves==issued+lost+1);
            CHECK(llabs(simulated_counts[0]-fake_lift_target)<500);
            CHECK(stops5==stopped+1); /* Only original MovePercent pre-stop. */
        } else {
            CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && native_lift_moves==issued+3);
            CHECK(tick-started==15000U); /* Resends never extend the original deadline. */
        }
        CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
    /* ACK lost but target genuinely reached: stable feedback can finish,
     * and near-target convergence must not be kicked by repeated commands. */
    NativeLiftReference(0);fake_lift_ack_enabled=0;issued=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,1)==HAL_OK);NativeLiftFinish();
    CHECK(native_lift_moves==issued+1 && ArmCalibration_GetState()==ARM_CAL_IDLE);
    /* A positive ACK must not replace measured arrival. */
    NativeLiftReference(0);fake_lift_freeze=1;issued=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);
    for(j=0;j<6;j++)CameraStep(0);
    CHECK(ArmCalibration_GetState()!=ARM_CAL_IDLE && native_lift_moves==issued+1);
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* Rejects are not acceptance; still only3 transmissions before the
     * old deadline. Cancellation stops retries and ignores late acceptance. */
    NativeLiftReference(0);fake_lift_freeze=1;fake_lift_ack_code=0xEE;issued=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);
    for(j=0;j<6;j++)CameraStep(0);
    CHECK(native_lift_moves==issued+3 && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    CHECK(ArmCalibration_Cancel()==HAL_OK);issued=native_lift_moves;
    {FDCAN_RxHeaderTypeDef h={0};uint8_t ack[3]={0xFD,0x02,0x6B};h.Identifier=5U<<8;h.IdType=FDCAN_EXTENDED_ID;
     h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;CHECK(ArmCalibration_PostCanFrame(&h,ack,3)==HAL_OK);}
    for(j=0;j<4;j++)CameraStep(0);
    CHECK(native_lift_moves==issued && ArmCalibration_GetState()==ARM_CAL_IDLE);
    /* A busy FIFO does not consume a successful-send slot. */
    NativeLiftReference(0);fake_lift_drop_commands=1;issued=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);CameraStep(0);slots=1;CameraStep(0);
    CHECK(native_lift_moves==issued+1 && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    slots=8;CameraStep(0);NativeLiftFinish();CHECK(native_lift_moves==issued+2);
}
static int TestNativeLift(void)
{
    arm_driver_config_t cfg;arm_cal_config_t cal;arm_servo_config_t servos;
    unsigned j,direction,zero_case;int32_t pulses;uint32_t issued,stopped,velocity;
    int64_t zero,expected;float pct;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
    motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
    zdt.can_ready=1;CHECK(ArmDriver_Init(&cfg)==HAL_OK);
    ArmCalibration_DefaultConfig(&cal);CHECK(ArmCalibration_Init(&cal)==HAL_OK);
    htim4.Instance=TIM4;htim4.State=HAL_TIM_STATE_READY;
    ArmServo_DefaultConfig(&servos);CHECK(ArmServo_Init(&servos)==HAL_OK);
    TestNativeAckSnapshots();
    /* Tray release alone relinquishes the native lift job inside5mm,
     * without STOP or waiting for the ordinary precise/settled arrival. */
    for(direction=0;direction<2;direction++) {
        NativeLiftReference(0);test_tray_descent=1U;fake_lift_freeze=1U;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);
        CameraStep(0);CameraStep(0);stopped=stops5;issued=native_lift_moves;
        expected=(int64_t)llround((double)last_lift_pulses*65536.0/ARM_LIFT_POSITION_STEPS_REV);
        simulated_counts[0]=expected+(int64_t)llround((direction?1:-1)*5.01*fabs(ARM_LIFT_FULL_TRAVEL_COUNTS)/240.0);
        CameraStep(0);CHECK(ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED);
        simulated_counts[0]=expected+(int64_t)llround((direction?1:-1)*4.99*fabs(ARM_LIFT_FULL_TRAVEL_COUNTS)/240.0);
        CameraStep(0);CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
        CHECK(stops5==stopped && native_lift_moves==issued && fake_lift_active);
        test_tray_descent=0U;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK); /* Next owner can raise immediately. */
        CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
    CHECK(ARM_LIFT_POSITION_STEPS_REV==3200U);
    CHECK(ArmDriver_GetLinearAcceleration(ARM_DRIVER_LIFT)==234U);
    CHECK(ArmDriver_GetLinearAcceleration(ARM_DRIVER_REACH)==209U);
    /* Material's timed native descent is not a percent-controller job. It
     * must still get the same empty lift cap, with no extra feedback settle. */
    NativeLiftReference(0);SetTestGrip(1U);
    tick+=10;motor.fb.last_tick=tick;Feedback(5U,0,0x83U);Feedback(6U,0,0x81U);
    ArmCalibration_Poll();
    CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,238U)==HAL_OK);
    CHECK(ArmCalibration_GetAbsoluteLiftPulses(10.0f,&pulses)==HAL_OK);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && !ArmCalibration_EmptyLiftMotionActive());
    CHECK(ArmDriver_LiftAbsoluteRawPulses(pulses,431U,-24.0f)==HAL_ERROR);
    test_material_active=1U;
    CHECK(ArmDriver_LiftAbsoluteRawPulses(pulses,431U,-24.0f)==HAL_OK);
    CHECK(last_lift_acceleration==234U);
    SetTestGrip(0U);tick+=10;motor.fb.last_tick=tick;
    Feedback(5U,simulated_counts[0],0x83U);Feedback(6U,0,0x81U);ArmCalibration_Poll();
    CHECK(ArmDriver_LiftAbsoluteRawPulses(pulses,431U,-24.0f)==HAL_ERROR);
    test_material_active=0U;
    for(zero_case=0;zero_case<3;zero_case++) {
        zero=zero_case==0?0:zero_case==1?65536:-65536;
        NativeLiftReference(zero);
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(0,&pulses)==HAL_OK && pulses==(int32_t)(zero/65536*3200));
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(50,&pulses)==HAL_OK);
        CHECK(pulses==(int32_t)round(((double)zero-219505.0*.5)*3200.0/65536.0));
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(NAN,&pulses)==HAL_ERROR);
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(-1,&pulses)==HAL_ERROR);
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(101,&pulses)==HAL_ERROR);
        CHECK(ArmCalibration_GetAbsoluteLiftPulses(50,NULL)==HAL_ERROR);
        /* RAW absolute coordinates are unaffected by logical jogging direction. */
        CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_LIFT,1U)==HAL_OK);
        CHECK(ArmDriver_LiftAbsoluteRawPulses((int32_t)(zero/65536*3200),198,0)==HAL_OK);
        CHECK(last_lift_pulses==(int32_t)(zero/65536*3200) && fake_lift_absolute);
        CHECK(ArmDriver_Stop(ARM_DRIVER_LIFT)==HAL_OK);
        CHECK(ArmDriver_SetLinearDirection(ARM_DRIVER_LIFT,0U)==HAL_OK);
        for(direction=0;direction<2;direction++) {
            float target=direction?0.0f:80.0f;
            Feedback(5U,simulated_counts[0],0x83U);Feedback(6U,0,0x81U);
            issued=native_lift_moves;velocity=velocity_calls;
            CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,target)==HAL_OK);
            CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,target)==HAL_OK);
            CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,50)==HAL_BUSY);
            for(j=0;j<2 && native_lift_moves==issued;j++)CameraStep(0);
            CHECK(native_lift_moves==issued+1 && fake_lift_absolute);
            CHECK(fake_lift_speed==341 && last_lift_acceleration==234); /* Configured198 x1.2. */
            expected=(int64_t)round(((double)zero+219505.0*(direction?0.0:-.8))*3200.0/65536.0);
            CHECK(last_lift_pulses==(int32_t)expected);
            stopped=stops5;NativeLiftFinish();
            CHECK(native_lift_moves==issued+1 && stops5==stopped && velocity_calls==velocity);
            CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&pct)==HAL_OK && fabsf(pct-target)*2.4f<=.5f);
            CHECK(ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT));
        }
    }
    /* An accepted/reached ACK and even a broad historical arrival window
     * must not cut off the driver's exact-position deceleration. */
    NativeLiftReference(0);simulated_counts[0]=-9146;Feedback(5,-9146,0x83);Feedback(6,0,0x81);
    fake_lift_freeze=1;CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0)==HAL_OK);
    for(j=0;j<2;j++){CameraStep(0);}issued=native_lift_moves;stopped=stops5;velocity=velocity_calls;
    {FDCAN_RxHeaderTypeDef h={0};uint8_t ack[3]={0xFD,0x9F,0x6B};h.Identifier=5U<<8;h.IdType=FDCAN_EXTENDED_ID;
     h.RxFrameType=FDCAN_DATA_FRAME;h.FDFormat=FDCAN_CLASSIC_CAN;CHECK(ArmCalibration_PostCanFrame(&h,ack,3)==HAL_OK);}
    simulated_counts[0]=-1829; /* 2mm belowTOP: inside old8mm band. */
    for(j=0;j<5;j++)CameraStep(0);
    CHECK(ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    CHECK(native_lift_moves==issued && stops5==stopped && velocity_calls==velocity);
    simulated_counts[0]=fake_lift_target;fake_lift_active=0;fake_lift_reached=1;
    /* Cached samples cannot count as fresh stable observations. */
    for(j=0;j<8;j++){ArmCalibration_Poll();}CHECK(ArmCalibration_GetState()!=ARM_CAL_IDLE);
    for(j=0;j<4;j++){CameraStep(0);}CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && stops5==stopped);
    fake_lift_freeze=0;
    /* CAN backpressure retains the accepted goal and retries only its send. */
    NativeLiftReference(0);slots=1;issued=native_lift_moves;velocity=velocity_calls;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,50)==HAL_OK);
    for(j=0;j<4;j++)CameraStep(0);
    CHECK(native_lift_moves==issued && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    slots=8;CameraStep(0);CHECK(native_lift_moves==issued+1);stopped=stops5;NativeLiftFinish();
    CHECK(stops5==stopped && velocity_calls==velocity);
    /* A stale PC-side observation does not stop a self-contained FD job or
     * restart it from a different zero when new encoder packets return. */
    NativeLiftReference(0);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);
    CameraStep(0);issued=native_lift_moves;stopped=stops5;velocity=velocity_calls;
    tick+=900;motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(ArmCalibration_FeedbackWaiting() && ArmCalibration_GetState()!=ARM_CAL_FAILED);
    CHECK(fake_lift_active && stops5==stopped && native_lift_moves==issued);
    for(j=0;j<8;j++){tick+=250;motor.fb.last_tick=tick;ArmCalibration_Poll();}
    CHECK(ArmCalibration_GetState()!=ARM_CAL_FAILED && stops5==stopped);
    NativeLiftFinish();CHECK(native_lift_moves==issued && velocity_calls==velocity);
    /* Explicit cancellation and reach recovery hold retain their original STOP. */
    NativeLiftReference(0);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);CameraStep(0);
    stopped=stops5;CHECK(ArmCalibration_Cancel()==HAL_OK);CHECK(stops5==stopped+1 && !fake_lift_active);
    issued=native_lift_moves;for(j=0;j<4;j++)CameraStep(0);CHECK(native_lift_moves==issued && ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,60)==HAL_OK);CameraStep(0);stopped=stops5;
    CHECK(ArmCalibration_HoldLiftForReachRecovery()==HAL_OK && stops5==stopped+1 && !fake_lift_active);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,60)==HAL_OK);NativeLiftFinish();
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* Retain the pre-existing low-height DM sector and real motor fault
     * checks; native conversion never permits a forbidden descent. */
    NativeLiftReference(0);motor.fb.pos_rad=30.0f*0.01745329252f;motor.fb.last_tick=tick;
    issued=native_lift_moves;velocity=velocity_calls;
    CHECK(ArmDriver_LiftAbsoluteRawPulses(-7000,198,-150)==HAL_ERROR);
    CHECK(native_lift_moves==issued);
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);CameraStep(0);
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && native_lift_moves==issued && velocity_calls==velocity);
    for(j=0;j<2;j++) {
        NativeLiftReference(0);CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);CameraStep(0);
        stopped=stops5;issued=native_lift_moves;
        tick+=100;motor.fb.last_tick=tick;Feedback(5,simulated_counts[0],j?0x89U:0x85U);Feedback(6,0,0x81);ArmCalibration_Poll();
        CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && stops5==stopped+1 && !fake_lift_active);
        CHECK(native_lift_moves==issued);
    }
    NativeLiftReference(0);move_result=HAL_ERROR;issued=native_lift_moves;
    CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,80)==HAL_OK);CameraStep(0);
    CHECK(ArmCalibration_GetState()==ARM_CAL_FAILED && native_lift_moves==issued && !ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT));
    move_result=HAL_OK;CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* CAMERA's intermediate lift returns to true TOP, without
     * rewriting the recorded zero or the configured final photo pose. */
    NativeLiftReference(0);SetTestGrip(1U);
    simulated_counts[0]=(int64_t)round(0.75*ARM_LIFT_FULL_TRAVEL_COUNTS);
    tick+=10;motor.fb.last_tick=tick;Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);ArmCalibration_Poll();
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&pct)==HAL_OK && fabsf(pct*240.0f/100.0f)<0.6f);
    CHECK(last_lift_pulses==0);
    CHECK(last_lift_acceleration==234U);
    {arm_cal_axis_status_t held;ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT,&held);CHECK(held.zero_counts==0);}
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    /* A completed PLACE may turn on its way back to camera, without
     * marking camera ready or replacing the still-running TOP target. */
    NativeLiftReference(0);SetTestGrip(1U);test_released_place=1U;
    simulated_counts[0]=(int64_t)round(0.75*ARM_LIFT_FULL_TRAVEL_COUNTS);
    tick+=10;motor.fb.last_tick=tick;Feedback(5,simulated_counts[0],0x81);Feedback(6,0,0x81);ArmCalibration_Poll();
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);
    CameraAdvanceTo(ARM_CAMERA_DM_HOME);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&pct)==HAL_OK && pct<=37.5f && pct>2.0f);
    CHECK(ArmCalibration_IsLiftRaisingToTop() && last_lift_pulses==0);
    CHECK(ArmCalibration_GetCameraState()!=ARM_CAMERA_DONE);
    CameraStep(1);CHECK(ArmCalibration_GetCameraState()==ARM_CAMERA_DM_HOME && ArmCalibration_IsLiftRaisingToTop());
    CameraAdvanceTo(ARM_CAMERA_REACH_ENABLE);
    CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&pct)==HAL_OK && fabsf(pct*240/100)<0.6f);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(ArmCalibration_Cancel()==HAL_OK);test_released_place=0U;
    TestLiftCommonAckRetries();
    return (int)test_failures;
}
#endif
int main(void)
{
#ifdef TEST_CAMERA_REPLAY_ONLY
    return TestCameraReplay();
#endif
#ifdef TEST_CAMERA_HOLD_ONLY
    return TestCameraNearHold();
#endif
#ifdef TEST_NATIVE_LIFT
    return TestNativeLift();
#endif
#if defined(TEST_SERVO_ONLY) || defined(TEST_SPEED_PROFILE_ONLY)
    {
        arm_driver_config_t cfg;arm_cal_config_t cal;
        ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180;cfg.rotation_max_deg=180;
#ifdef TEST_SPEED_PROFILE_ONLY
        cfg.rotation_ratio=1.5f;
#endif
        motor.active_mode=DM_POS_MODE;motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;
        zdt.can_ready=1;
        CHECK(ArmDriver_Init(&cfg)==HAL_OK);ArmCalibration_DefaultConfig(&cal);
        CHECK(ArmCalibration_Init(&cal)==HAL_OK);TestServos();
#ifdef TEST_SPEED_PROFILE_ONLY
        TestEmptyRotationSpeed();TestEmptyLiftSpeed();TestLiftMeasuredRangePolicy();TestReachLoadAndFastArrival();TestReleasedOpeningLiftProfile();
#endif
        return (int)test_failures;
    }
#endif
#ifdef TEST_FEEDBACK_WAIT
    return TestFeedbackWait();
#endif
#ifdef TEST_28_STALL_TRIAL
    return Test28StallTrial();
#endif
#ifdef TEST_REACH_RETRY
    return TestReachRetry();
#endif
#ifdef TEST_REPLACEMENT_REACH
    return TestReplacementReach();
#endif
#ifdef TEST_DM_INTERNAL_ZERO
    return Test_InternalDmZero();
#endif
    arm_driver_config_t cfg;
    arm_driver_config_t invalid;
    float angle;
    uint32_t before;
    arm_cal_config_t calibration;
    arm_cal_axis_status_t status = {0};
#ifdef TEST_MEASURED_CAMERA
    (void)invalid;(void)angle;(void)before;(void)status;
    ArmDriver_DefaultConfig(&cfg);cfg.rotation_min_deg=-180.0f;cfg.rotation_max_deg=180.0f;
    motor.cfg.pos_limit_rad=DM_P_MAX;motor.cfg.vel_limit_rad=DM_V_MAX;motor.active_mode=DM_POS_MODE;
    motor.fb.fb_count=1U;motor.fb.last_tick=tick;zdt.can_ready=1U;
    ArmCalibration_DefaultConfig(&calibration);
    CHECK(ArmDriver_Init(&cfg)==HAL_OK && ArmCalibration_Init(&calibration)==HAL_OK);
    CameraSetup();CHECK(ArmCalibration_ProcessCommand("arm camera defaults")==1U);
    CHECK(ArmCalibration_SetCameraPositionMode(1U)==HAL_OK);fake_native_model=1U;
    simulated_counts[0]=-137;simulated_counts[1]=-34009;
    motor.fb.pos_rad=2.349516f;motor.target_pos_rad=2.349516f;CameraStep(0U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(ArmDriver_GetRotationDeg(&angle)==HAL_OK && fabsf(angle-ARM_CAMERA_DM_DEG)<0.1f);
    CHECK(fabs((double)simulated_counts[1]*360.0/65536.0-ARM_CAMERA_REACH_DEG)<0.1);
    CHECK(last_pulses==2667); /* Absolute CAMERA target, independent of the initial START angle. */
    CHECK(save_calls==0U); /* Presets never write the motor zero. */
    CHECK(ArmCalibration_Cancel()==HAL_OK);
    simulated_counts[0]=-319;simulated_counts[1]=735;CameraStep(0U);
    CHECK(ArmCalibration_DeployCamera()==HAL_OK);CameraAdvanceTo(ARM_CAMERA_DONE);
    CHECK(last_pulses==2667); /* Boot encoder +4deg still targets absolute -300deg. */
    CHECK(ArmDriver_GetRotationDeg(&angle)==HAL_OK && fabsf(angle-ARM_CAMERA_DM_DEG)<0.1f);
    return (int)test_failures;
#endif
#ifdef TEST_DEFAULT_ZERO
    return Test_DefaultDriverZero();
#endif

    CHECK(ArmDriver_LiftByPulses(1) == HAL_ERROR);
    CHECK(ArmDriver_Init(NULL) == HAL_ERROR);
    ArmDriver_DefaultConfig(&cfg);
    CHECK(NEAR(cfg.rotation_max_deg, 360.0f));
    CHECK(cfg.lift.pulses_per_mm == 0.0f);
    CHECK(cfg.lift.speed_rpm == 180U && NEAR(cfg.rotation_speed_dps,112.5f));
    CHECK(cfg.reach.speed_rpm == 60U);
    invalid = cfg; invalid.rotation_ratio = 10.0f;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR); /* DM range cannot cover a full turn */
    invalid = cfg; invalid.rotation_speed_dps = INFINITY;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    invalid = cfg; invalid.rotation_zero_rad = NAN;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    invalid = cfg; invalid.reach.reverse = 2U;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    invalid = cfg; invalid.lift.pulses_per_mm = -1.0f;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    invalid = cfg; invalid.reach.speed_rpm = 0U;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    invalid = cfg; invalid.lift.max_move_pulses = UINT32_MAX;
    CHECK(ArmDriver_Init(&invalid) == HAL_ERROR);
    cfg.lift.pulses_per_mm = 800.0f;
    cfg.lift.max_move_pulses = 20000U;
    cfg.reach.reverse = 1U;
#ifndef TEST_UNCALIBRATED
    cfg.reach.pulses_per_mm = 320.0f;
#endif
    cfg.rotation_reverse = 1U;
    cfg.rotation_zero_rad = 1.0f;
    cfg.rotation_ratio = 1.5f;
    CHECK(ArmDriver_Init(&cfg) == HAL_OK);
    CHECK((moves == 0U) && (enable_calls == 0U) && (dm_moves == 0U));
    CHECK(ArmDriver_Init(&cfg) == HAL_BUSY);
    CHECK(ArmDriver_Enable((arm_driver_axis_t)99) == HAL_ERROR);
    CHECK(ArmDriver_LiftByPulses(200) == HAL_ERROR);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT) == HAL_ERROR); /* bus not started */
    zdt.can_ready = 1U;
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT) == HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH) == HAL_OK);
    CHECK(ArmDriver_LiftByPulses(1600) == HAL_OK);
    CHECK((last_address == 5U) && (last_pulses == 1600) && (last_speed == 180U));
    CHECK((last_buffered == 0U) && (last_acceleration == 234U));
    CHECK(ArmDriver_LiftByMm(-2.0f) == HAL_OK);
    CHECK((last_address == 5U) && (last_pulses == -1600));
    CHECK(ArmDriver_ReachByPulses(500) == HAL_OK);
    CHECK((last_address == 6U) && (last_pulses == -500));
    CHECK(ArmDriver_ReachByPulses(-500) == HAL_OK);
    CHECK(last_pulses == 500);
#ifdef TEST_UNCALIBRATED
    CHECK(ArmDriver_ReachByMm(1.0f) == HAL_ERROR);
#else
    CHECK(ArmDriver_ReachByMm(1.5f) == HAL_OK);
    CHECK((last_address == 6U) && (last_pulses == -480));
#endif
    before = moves;
    CHECK(ArmDriver_LiftByPulses(INT32_MIN) == HAL_ERROR);
    CHECK(ArmDriver_ReachByPulses(INT32_MIN) == HAL_ERROR);
    CHECK(ArmDriver_LiftByPulses(20001) == HAL_ERROR);
    CHECK(ArmDriver_LiftByMm(NAN) == HAL_ERROR);
    CHECK(ArmDriver_LiftByMm(INFINITY) == HAL_ERROR);
    CHECK(ArmDriver_LiftByMm(1.0e30f) == HAL_ERROR);
    CHECK(ArmDriver_LiftByMm(0.00001f) == HAL_ERROR);
    CHECK(ArmDriver_LiftByPulses(0) == HAL_OK);
    CHECK(moves == before);
    slots = 1U;
    CHECK(ArmDriver_LiftByPulses(100) == HAL_BUSY);
    CHECK(moves == before);
    slots = 8U; bus_off = 1U;
    CHECK(ArmDriver_LiftByPulses(100) == HAL_ERROR);
    CHECK(moves == before);
    bus_off = 0U;
    move_result = HAL_ERROR;
    CHECK(ArmDriver_LiftByPulses(100) == HAL_ERROR);
    CHECK(stops5 == 1U);
    CHECK(ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT) == 0U);
    move_result = HAL_OK;
    CHECK(ArmDriver_LiftByPulses(100) == HAL_ERROR);
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT) == HAL_OK);
    CHECK(ArmDriver_Stop(ARM_DRIVER_LIFT) == HAL_OK);
    CHECK(ArmDriver_IsEnableRequested(ARM_DRIVER_LIFT) == 1U);
    CHECK(disables5 == 0U); /* vertical stop must retain holding torque */
    stop_lift_result = HAL_ERROR;
    before = stops6;
    CHECK(ArmDriver_StopAll() == HAL_ERROR);
    CHECK(stops6 == before + 1U);
    CHECK(dm_disables == 1U); /* stop-all did not return at first error */
    stop_lift_result = HAL_OK;
    CHECK(ArmDriver_Disable(ARM_DRIVER_REACH) == HAL_OK);
    CHECK(disables6 == 1U);
    CHECK(ArmDriver_ReachByPulses(1) == HAL_ERROR);
    step_enable_result = HAL_ERROR;
    CHECK(ArmDriver_Enable(ARM_DRIVER_REACH) == HAL_ERROR);
    CHECK(ArmDriver_IsEnableRequested(ARM_DRIVER_REACH) == 0U);

    motor.active_mode = DM_POS_MODE;
    motor.cfg.pos_limit_rad = DM_P_MAX;
    motor.cfg.vel_limit_rad = DM_V_MAX;
    motor.pos_offset_rad = 0.25f; /* Exercise dm zerohere compensation */
    motor.fb.pos_rad = 1.0f - 1.57079632679f * 1.5f; /* actual arm angle 90 */
    motor.fb.fb_count = 1U;
    motor.fb.last_tick = tick;
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_ERROR); /* no auto enable */
    CHECK(NEAR(ArmDriver_RotationSpeedDps(),112.5f));
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_OK);
    CHECK(ArmDriver_GetRotationDeg(&angle) == HAL_OK);
    CHECK(NEAR(angle, 90.0f));
    CHECK(ArmDriver_RotateToDeg(360.0f) == HAL_OK);
    CHECK(NEAR(motor.target_pos_rad, 1.0f - 6.28318530718f * 1.5f));
    CHECK(NEAR(motor.target_vel_rads, 1.96349540849f * 1.5f));
    CHECK(ArmDriver_RotateToDegAtSpeed(360.0f,30.0f)==HAL_OK);
    CHECK(NEAR(motor.target_vel_rads,0.5235987756f*1.5f));
    CHECK(ArmDriver_RotateToDegAtSpeed(360.0f,0.0f)==HAL_ERROR);
    CHECK(ArmDriver_RotateToDegAtSpeed(360.0f,NAN)==HAL_ERROR);
    CHECK(ArmDriver_RotateToDegAtSpeed(360.0f,113.0f)==HAL_ERROR);
    CHECK(ArmDriver_RotateToDeg(360.0f)==HAL_OK); /* V105 default112.5 */
    CHECK(ArmDriver_RotateByDeg(90.0f) == HAL_OK);
    CHECK(NEAR(motor.target_pos_rad, 1.0f - 3.14159265359f * 1.5f));
    before = dm_moves;
    CHECK(ArmDriver_RotateToDeg(361.0f) == HAL_ERROR);
    CHECK(ArmDriver_RotateToDeg(-1.0f) == HAL_ERROR);
    CHECK(ArmDriver_RotateToDeg(NAN) == HAL_ERROR);
    CHECK(ArmDriver_RotateByDeg(INFINITY) == HAL_ERROR);
    CHECK(dm_moves == before);
    motor.fb.last_tick=tick;
    tick += DM_LINK_TIMEOUT_MS_DEFAULT;
    CHECK(ArmDriver_GetRotationDeg(&angle)==HAL_OK); /* DM800ms inclusive boundary. */
    tick += 1U;
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_ERROR);
    CHECK(ArmDriver_GetRotationDeg(&angle) == HAL_ERROR);
    motor.fb.last_tick = tick;
    motor.cfg.pos_limit_rad = 2.0f;
    CHECK(ArmDriver_RotateToDeg(360.0f) == HAL_ERROR); /* reject instead of clamp */
    motor.cfg.pos_limit_rad = DM_P_MAX;
    dm_tx_result = HAL_BUSY;
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_BUSY);
    CHECK(dm_stops == 1U);
    CHECK(NEAR(motor.target_pos_rad, 0.0f));
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_ERROR);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_BUSY); /* explicit retry still obeys failed CAN transport */

    CHECK(ArmTask_ClearFault() == HAL_OK);
    dm_tx_result = HAL_OK;
    CHECK(ArmDriver_SetRotationLimits(-180.0f, 180.0f) == HAL_OK);
    CHECK(ArmDriver_SetRotationZeroHere() == HAL_OK);
    CHECK(ArmDriver_GetRotationDeg(&angle) == HAL_OK);
    CHECK(NEAR(angle, 0.0f));
    CHECK(ArmDriver_SetRotationLimits(0.0f, 360.0f) == HAL_OK);
    CHECK(ArmDriver_SetRotationLimits(0.0f, 1000.0f) == HAL_ERROR);
    CHECK(ArmDriver_SetHeightRotationLimit(50.0f, 0.0f, 90.0f) == HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_ERROR); /* unknown height */
    ArmDriver_UpdateLiftHeight(10.0f, tick, 1U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_OK);
    CHECK(ArmDriver_SetRotationZeroHere() == HAL_ERROR); /* no moving zero changes */
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_ERROR);
    CHECK(ArmDriver_RotateToDeg(90.0f) == HAL_OK);
    ArmDriver_UpdateLiftHeight(100.0f, tick, 1U);
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_OK);
    ArmDriver_UpdateLiftHeight(10.0f, tick, 1U);
    CHECK(ArmDriver_CheckHeightInterlock() == HAL_ERROR); /* target is outside low-height range */
    CHECK(arm_enabled == 0U);
    ArmDriver_ClearHeightRotationLimit();
    CHECK(ArmDriver_SetLinearScale(ARM_DRIVER_REACH, NAN) == HAL_ERROR);
    CHECK(ArmDriver_SetLinearScale(ARM_DRIVER_REACH, 100.0f) == HAL_OK);

    ArmCalibration_DefaultConfig(&calibration);
    calibration.reach_reverse = 1U;
    CHECK(ArmCalibration_Init(&calibration) == HAL_OK);
    CHECK(ArmCalibration_Init(&calibration) == HAL_BUSY);
    CHECK(ArmCalibration_OnCanFrame(NULL, NULL, 0U) == 0U);
    step_enable_result = HAL_OK;
    CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT) == HAL_OK);
    CHECK(ArmCalibration_MarkEndpoint(ARM_DRIVER_LIFT, 0U) == HAL_OK);
    CHECK(ArmCalibration_SeekEndpoint(ARM_DRIVER_LIFT, 1U) == HAL_BUSY);
    Sample(5U, -65536, 1U); /* Fresh baseline, then two stable post-stop samples */
    CHECK(ArmCalibration_GetState() == ARM_CAL_SETTLING);
    Sample(5U, -65536, 1U);
    Sample(5U, -65536, 1U);
    CHECK(ArmCalibration_GetState() == ARM_CAL_IDLE);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT, &status);
    CHECK((status.lower_valid == 1U) && (status.lower_counts == -65536));

    CHECK(ArmCalibration_SeekEndpoint(ARM_DRIVER_LIFT, 1U) == HAL_OK);
    Sample(5U, -65536, 0x0DU); /* Stale latched stall/protection must not calibrate */
    CHECK(ArmCalibration_GetState() == ARM_CAL_FAILED);
    CHECK(speed_frames == 0U);
    CHECK(ArmCalibration_ClearProtection(ARM_DRIVER_LIFT) == HAL_OK);
    CHECK(clear_frames == 1U);
    CHECK(ArmCalibration_SeekEndpoint(ARM_DRIVER_LIFT, 1U) == HAL_OK);
    Sample(5U, -65536, 1U);
    CHECK(ArmCalibration_GetState() == ARM_CAL_SEEKING);
    CHECK((speed_frames == 1U) && (speed_direction == 0U) && (seek_speed == 20U));
    Sample(5U, 0, 5U); /* Startup stall is ignored */
    CHECK(ArmCalibration_GetState() == ARM_CAL_SEEKING);
    Sample(5U, 10000, 1U);
    Sample(5U, 65536, 5U);
    CHECK(ArmCalibration_GetState() == ARM_CAL_SEEKING); /* one fresh stall only */
    Sample(5U, 65536, 0x0CU); /* protection can remove enable */
    CHECK(ArmCalibration_GetState() == ARM_CAL_SETTLING);
    Sample(5U, 65536, 0x0CU);
    Sample(5U, 65536, 0x0CU);
    CHECK(ArmCalibration_GetState() == ARM_CAL_IDLE);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT, &status);
    CHECK((status.upper_valid == 1U) && (status.upper_counts == 65536));
    CHECK(status.upper_counts - status.lower_counts == 131072); /* exactly 2 Emm revs */
    CHECK(ArmCalibration_SetTravelMm(ARM_DRIVER_LIFT, 200.0f) == HAL_OK);
    Sample(5U, 0, 1U); /* actual height = 100 mm */
    CHECK(ArmDriver_SetHeightRotationLimit(50.0f, 0.0f, 90.0f) == HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_OK);
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_OK);
    Sample(5U, -50000, 1U); /* actual height ~24mm, target 180 must be cancelled */
    CHECK(arm_enabled == 0U);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION) == HAL_OK); /* actual rotation still 0 */
    CHECK(ArmDriver_RotateToDeg(180.0f) == HAL_ERROR);
    ArmDriver_ClearHeightRotationLimit();
    CHECK(ArmTask_Disable() == HAL_OK);

    CHECK(ArmCalibration_SeekEndpoint(ARM_DRIVER_LIFT, 1U) == HAL_OK);
    Sample(5U, 0, 1U);
    tick += ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;
    ArmCalibration_Poll();
    CHECK(ArmCalibration_GetState() == ARM_CAL_FAILED); /* lost feedback stops seek */
    CHECK(ArmCalibration_ProcessCommand("unrelated") == 0U);
    CHECK(ArmCalibration_ProcessCommand("arm status") == 1U);
    CHECK(reports != 0U);
    before = moves;
    CHECK(ArmCalibration_ProcessCommand("arm pulse 5 2147483648") == 1U);
    CHECK(ArmCalibration_ProcessCommand("arm pulse 5 1junk") == 1U);
    CHECK(moves == before);
    CHECK(ArmCalibration_ProcessCommand("arm pulse 5 100") == 1U);
    CHECK(moves == before + 1U);
    CHECK(ArmCalibration_ProcessCommand("arm stop") == 1U);
    {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[3] = {0x3AU, 1U, 0x6BU};
        uint32_t posted_tick = tick;
        header.Identifier = 5U << 8U;
        header.IdType = FDCAN_EXTENDED_ID;
        header.RxFrameType = FDCAN_DATA_FRAME;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        CHECK(ArmCalibration_PostCanFrame(&header, data, 3U) == HAL_OK);
        tick += 600U;
        ArmCalibration_Poll();
        ArmCalibration_GetAxisStatus(ARM_DRIVER_LIFT, &status);
        CHECK(status.flags_tick == posted_tick); /* queue delay must not refresh old data */
        header.Identifier = 1U << 8U;
        CHECK(ArmCalibration_PostCanFrame(&header, data, 3U) == HAL_ERROR);
        CHECK(ArmCalibration_PostCommand("arm status") == HAL_OK);
        CHECK(ArmCalibration_PostCommand("arm status") == HAL_OK);
        CHECK(ArmCalibration_PostCommand("arm status") == HAL_OK);
        CHECK(ArmCalibration_PostCommand("arm status") == HAL_BUSY);
        ArmCalibration_Poll();
        ArmCalibration_Poll();
        ArmCalibration_Poll();
    }
    TestServos();
    /* Highest-point reference does not require a lower endpoint. */
    arm_enabled=0U;motor.fb.enabled=0U;motor.fb.pos_rad=0.0f;motor.fb.vel_rads=0.0f;
    motor.fb.last_tick=tick;motor.fb.fb_count++;motor.target_pos_rad=0.0f;
    CHECK(ArmDriver_SetRotationLimits(-180.0f,180.0f)==HAL_OK);
    CHECK(ArmDriver_SetRotationZeroHere()==HAL_OK);
    CHECK(ArmCalibration_UseTopGuard(100.0f,-20.0f,20.0f)==HAL_OK);
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_OK);
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_ERROR); /* unknown height only permits +/-20 */
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm top 5")==1U);
    Sample(5U,10000,3U);Sample(5U,10000,3U);Sample(5U,10000,3U);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_OK); /* known highest */
    Sample(5U,14000,3U); /* ~4.4mm above reference: acceptable TOP tolerance */
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_OK);
    Sample(5U,10000,3U);
    CHECK(ArmCalibration_ProcessCommand("arm query 5")==1U && strstr(last_console,"angle="));
    Feedback(6U,0,3U);
    CHECK(ArmCalibration_ProcessCommand("arm query 6")==1U && strstr(last_console,"angle="));
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm dropcal 50")==1U);
    Sample(5U,-10000,3U);Sample(5U,-10000,3U);Sample(5U,-10000,3U);
    CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE);
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_OK); /* down 50mm */
    Sample(5U,-34000,3U); /* down 110mm cancels out-of-range rotation */
    CHECK(!ArmTask_IsEnabled());
    CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_OK);
    CHECK(ArmDriver_RotateToDeg(20.0f)==HAL_OK);
    CHECK(ArmDriver_RotateToDeg(21.0f)==HAL_ERROR);
    CHECK(ArmDriver_RotateToDeg(-20.0f)==HAL_OK);
    CHECK(ArmDriver_RotateToDeg(-21.0f)==HAL_ERROR);
    tick+=ARM_ZDT_FEEDBACK_TIMEOUT_MS+1U;motor.fb.last_tick=tick;ArmCalibration_Poll();
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_ERROR); /* stale lift data */
    CHECK(ArmDriver_RotateToDeg(0.0f)==HAL_OK);
    CHECK(ArmCalibration_ProcessCommand("arm top 5")==1U);
    Sample(5U,12345,3U);Sample(5U,12345,3U);Sample(5U,12345,3U);
    CHECK(ArmDriver_RotateToDeg(30.0f)==HAL_OK); /* re-reference highest */
    CHECK(ArmCalibration_ProcessCommand("arm zero 6")==1U);
    Sample(6U,700,3U);Sample(6U,700,3U);Sample(6U,700,3U);
    ArmCalibration_GetAxisStatus(ARM_DRIVER_REACH,&status);
    CHECK(status.zero_valid && status.zero_counts==700 && status.lower_valid && status.lower_counts==0);
    before=dm_moves;
    CHECK(ArmCalibration_PostCommand("arm rotate 10")==HAL_OK);
    ArmCalibration_DiscardCommands();ArmCalibration_Poll();CHECK(dm_moves==before);
    CHECK(ArmDriver_LiftByPulses(10)==HAL_ERROR); /* cannot move above recorded top */
    CHECK(ArmCalibration_ProcessCommand("dm pos 100")==1U); /* raw target cannot bypass */
    CHECK(ArmCalibration_ProcessCommand("arm dir 5 1")==1U);
    CHECK(ArmCalibration_ProcessCommand("arm speed 5 20")==1U);
    TestPercent();
    TestReachDirectionAndFault();
    /* Convenience controls refresh disabled feedback even after idle timeout,
     * stay OFF after zero, and never write flash for bare/legacy soft zero. */
    {
        float angle;uint32_t saves=save_calls,queries=query_calls;
        CHECK(ArmCalibration_Cancel()==HAL_OK);ArmDriver_ClearHeightRotationLimit();
        arm_fault=0U;arm_enabled=0U;motor.active_mode=DM_POS_MODE;
        motor.fb.pos_rad=0.25f;motor.fb.vel_rads=0.0f;motor.fb.enabled=0U;
        tick+=10000U;CHECK(ArmCalibration_ProcessCommand("dm zero")==1U);
        CHECK(query_calls==queries+1U && save_calls==saves && !arm_enabled);
        CHECK(ArmDriver_GetRotationDeg(&angle)==HAL_OK && NEAR(angle,0.0f) && NEAR(motor.pos_offset_rad,0.25f));
        tick+=10000U;CHECK(ArmCalibration_ProcessCommand("dm on")==1U);
        CHECK(arm_enabled && motor.target_pos_rad==motor.fb.pos_rad);
        CHECK(ArmCalibration_ProcessCommand("dm off")==1U && !arm_enabled);
        motor.fb.vel_rads=0.3f;
        CHECK(ArmDriver_ZeroRotation(1U)==HAL_BUSY && save_calls==saves);
        motor.fb.vel_rads=0.0f;query_result=HAL_TIMEOUT;
        CHECK(ArmDriver_ZeroRotation(1U)==HAL_TIMEOUT && save_calls==saves);
        query_result=HAL_OK;verify_result=HAL_TIMEOUT;
        CHECK(ArmCalibration_ProcessCommand("dm zero save")==1U);
        CHECK(save_calls==saves+1U && !ArmDriver_IsRotationReferenceValid() && !arm_enabled);
        CHECK(ArmCalibration_ProcessCommand("dm query")==1U && strstr(last_console,"arm_deg=UNKNOWN"));
        CHECK(ArmDriver_Enable(ARM_DRIVER_ROTATION)==HAL_ERROR);
        CHECK(ArmDriver_CheckRawRotationTarget(0.0f)==HAL_ERROR);
        queries=dm_moves;CHECK(ArmDriver_ReturnRotationZero()==HAL_ERROR && dm_moves==queries && !arm_enabled);
        verify_result=HAL_OK;
        CHECK(ArmCalibration_ProcessCommand("arm zero dm save")==1U);
        CHECK(save_calls==saves+2U && ArmDriver_IsRotationReferenceValid() && !arm_enabled);
        CHECK(NEAR(motor.pos_offset_rad,0.0f) && ArmDriver_GetRotationDeg(&angle)==HAL_OK);
        CHECK(fabsf(angle)<0.1f); /* zero is RAW0, quantized readback is near it */
        saves=save_calls;CHECK(ArmCalibration_ProcessCommand("arm zero dm")==1U);
        CHECK(save_calls==saves+1U && !arm_enabled);
        CHECK(ArmCalibration_ProcessCommand("dm zero confirm")==1U && save_calls==saves+2U);
        /* Query uses fresh feedback without changing zero, target or enable. */
        motor.fb.pos_rad=0.5f;motor.fb.last_tick=0U;arm_enabled=0U;
        queries=read_calls;CHECK(ArmCalibration_ProcessCommand("dm query")==1U);
        CHECK(read_calls==queries+1U && !arm_enabled && strstr(last_console,"raw_deg=") && strstr(last_console,"arm_deg="));
        CHECK(ArmDriver_MapRawRotation(motor.fb.pos_rad,&angle)==HAL_OK);
        CHECK(ArmCalibration_ProcessCommand("dm home")==1U && arm_enabled);
        CHECK(NEAR(motor.target_pos_rad,0.0f) && strstr(last_console,"HOME queued"));
        motor.target_pos_rad=0.75f;saves=save_calls;queries=query_calls;
        CHECK(ArmCalibration_ProcessCommand("dm angle")==1U);
        CHECK(arm_enabled && motor.target_pos_rad==0.75f && save_calls==saves && query_calls==queries);
    }
#ifndef TEST_DEFAULT_ZERO
    Test_PacedArmReads();
    TestCameraDeploy();
    TestCameraHandoffStall();
    TestArmIoDuringWait();
    TestCameraNearTopKick();
    TestCameraIdleFeedbackGap();
    TestTopArrivalBand();
    TestCameraFeedbackWindows();
    TestEmptyRotationSpeed();
    TestEmptyLiftSpeed();
    TestCameraLateSnapshot();
    TestCameraReferenceAndHold();
    TestCameraBraking();
    TestCameraReachSpeed();
    TestCameraNativePosition();
    TestCameraUnstarted();
    TestCameraResidual();
    TestCameraNativeRetry();
    TestPickReachAcrossZero();
    TestCameraCustomPose();
    TestCameraDmHoldOnZdtFailure();
    {
        unsigned int j;int peak=0;float percent;
        CameraSetup();SetTestGrip(0U);simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*0.6);
        Feedback(5U,simulated_counts[0],0x80U);
        CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);CameraStep(0U);
        test_start_pose_active=1U;
        CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,0.0f)==HAL_OK);
        for(j=0U;j<160U && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;j++) {
            CameraStep(0U);if(abs(percent_motor_rpm[0])>peak)peak=abs(percent_motor_rpm[0]);
            CHECK(abs(percent_motor_rpm[0])<=60);
        }
        CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && peak>=59 && peak<=60);
        CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK && fabsf(percent)<=ArmCalibration_LiftTolerancePercent(0.0f,ARM_LIFT_FULL_TRAVEL_MM));
        test_start_pose_active=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
    }
    {
        uint32_t elapsed[2][2];unsigned speed_case,direction,j;float percent;
        for(direction=0;direction<2;direction++)for(speed_case=0;speed_case<2;speed_case++) {
            uint32_t began;int peak=0;
            CameraSetup();SetTestGrip(0U);encoder_gain[0]=10.5f; /* Loaded profile comparison. */
            CHECK(ArmDriver_SetLinearSpeed(ARM_DRIVER_LIFT,speed_case?120U:60U)==HAL_OK);
            simulated_counts[0]=(int64_t)round(ARM_LIFT_FULL_TRAVEL_COUNTS*(direction?0.05:0.60));
            Feedback(5U,simulated_counts[0],0x80U);CHECK(ArmDriver_Enable(ARM_DRIVER_LIFT)==HAL_OK);
            CameraStep(0U);test_material_active=1U;
            CHECK(ArmCalibration_MovePercent(ARM_DRIVER_LIFT,direction?50.0f:0.0f)==HAL_OK);began=tick;
            for(j=0;j<200 && ArmCalibration_GetState()!=ARM_CAL_IDLE && ArmCalibration_GetState()!=ARM_CAL_FAILED;j++) {
                CameraStep(0U);if(abs(percent_motor_rpm[0])>peak)peak=abs(percent_motor_rpm[0]);
                CHECK(abs(percent_motor_rpm[0])<=(speed_case?120:60));
            }
            elapsed[direction][speed_case]=tick-began;
            CHECK(ArmCalibration_GetState()==ARM_CAL_IDLE && peak>0);
            CHECK(ArmCalibration_GetPercent(ARM_DRIVER_LIFT,&percent)==HAL_OK &&
                  fabsf(percent-(direction?50.0f:0.0f))<=ArmCalibration_LiftTolerancePercent(direction?50.0f:0.0f,ARM_LIFT_FULL_TRAVEL_MM));
            test_material_active=0U;CHECK(ArmCalibration_Cancel()==HAL_OK);
        }
        CHECK(elapsed[0][1]<elapsed[0][0] && elapsed[1][1]<elapsed[1][0]);
    }
#endif
    return (int)test_failures;
}

uint8_t DM_Console_ProcessCommand(dm_motor_t *m,const char *line) { (void)m;(void)line;return 1U; }
HAL_StatusTypeDef DM_Motor_DiagStart(dm_motor_t *m){m->diag_active=1U;return HAL_OK;}
void DM_Motor_DiagCancel(dm_motor_t *m){m->diag_active=0U;}
uint8_t DM_Motor_DiagIsActive(const dm_motor_t *m){return m->diag_active;}
uint8_t ArmStartPose_IsReleasedPlace(void){return test_released_place;}
uint8_t ArmStartPose_IsActive(void){return test_start_pose_active;}
uint8_t ArmStartPose_ReachStartupPending(void){return 0U;}
uint8_t ArmStartPose_IsFastTransfer(void){return test_start_pose_active;}
uint8_t ArmStartPose_RingShared(void){return 1U;}
uint8_t ArmStartPose_IsTrayPlaceDescent(void){return test_tray_descent;}
void ArmStartPose_Cancel(void){test_start_pose_active=0U;}
uint8_t ArmStartPose_IsReleaseOpening(void){return test_release_opening;}
void ArmStartPose_ReleaseForCamera(void){ArmStartPose_Cancel();}
uint8_t ArmStartPose_Command(const char *line){(void)line;return 0U;}
void ArmStartPose_PrintUiPoints(void){}
const char *ArmStartPose_BootParameterSource(void){return "DEFAULT";}

uint8_t ArmMaterial_IsActive(void){return test_material_active;}
uint8_t ArmMaterial_GrabReachActive(void){return test_material_grab_reach;}
uint8_t ArmMaterial_Command(const char *line){(void)line;return 0U;}
void ArmMaterial_AbortState(void){test_material_active=0U;}
