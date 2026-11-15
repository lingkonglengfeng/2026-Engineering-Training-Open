void Test_ScreenProtocol(void);
void Test_ScreenOwner(void);
/* Runs the real FSM and real UART '&' tokenizer under a simulated tick. */
#include "match_control.h"
#include "bt_button.h"
#include "nav_params.h"
#include "nav_frame.h"
#include "nav_feedback.h"
#include "grid_nav.h"
#include "nav_blend.h"
#include "chassis_field.h"
#include "zone2_route.h"
#include "w25q64.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_first_failure;
volatile uint32_t test_failure_detail_count;
const char *test_failure_files[64];
uint32_t test_failure_lines[64];
void Test_RecordFailure(const char *file,uint32_t line)
{
    if(test_failure_detail_count<64U){test_failure_files[test_failure_detail_count]=file;test_failure_lines[test_failure_detail_count++]=line;}
}
volatile float test_blend_metrics[6][6];
static unsigned int blend_case_index;
/* Execute the production integer ramp, including its stop-before-reverse path. */
extern int16_t ChassisMotor_RampStepRpm(int16_t current,int16_t target,float dt,float decel);
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
void Test_SetTick(uint32_t now) { tick=now; }
extern void Test_ChassisFieldLifecycle(void);
extern void Test_PostAlignShortMoves(void);
extern void Test_VisionAlignment(void);
extern void Test_SmoothVisionAlignment(void);
extern void Test_VisionAlignmentDiagnostic(void);
extern void Test_Zone2ActionClock(void);
extern void Test_ZdtMotorQuery(void);
extern void Test_ZdtQueryRxRouting(void);
extern void Test_AlignmentAnchors(void);
extern void Test_NavOdometry(void);
extern void Test_OdometryOwner(void);
extern void Test_ReturnDetour(void);
extern void Test_QrScan(void);
extern void Test_QrApplication(void);
extern void Test_NavCorner(void);
extern void Test_NavCornerOwner(void);
void ZDT_Console_Print(const char *text) { (void)text; }
/* Exercise production diagnostic formatting with a bounded emulator-only
 * libc heap. Console transport stays mocked; production uses Keil microlib. */
static uint64_t test_libc_heap[2048];
static ptrdiff_t test_libc_break;
void *_sbrk(ptrdiff_t increment)
{
    ptrdiff_t previous=test_libc_break;
    if(increment< -previous || increment>(ptrdiff_t)sizeof(test_libc_heap)-previous)
        return (void *)-1;
    test_libc_break+=increment;
    return (uint8_t *)test_libc_heap+previous;
}
#define CHECK(x) do { test_checks++; if (!(x)) { test_failures++; \
    if (test_first_failure == 0U) { test_first_failure = __LINE__; } Test_RecordFailure(__FILE__,__LINE__); } } while (0)

/* RAM-backed Flash transport: exercise production record CRC/migration. */
static uint8_t nav_flash[64];
w25q64_status_t W25Q64_Init(w25q64_id_t *id) { (void)id; return W25Q64_OK; }
uint32_t W25Q64_GetCapacity(void) { return W25Q64_CAPACITY_BYTES; }
w25q64_status_t W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    CHECK(address == W25Q64_CAPACITY_BYTES-2U*W25Q64_SECTOR_SIZE);
    if (length > sizeof(nav_flash)) { return W25Q64_ERROR_PARAM; }
    memcpy(data, nav_flash, length); return W25Q64_OK;
}
w25q64_status_t W25Q64_EraseSector(uint32_t address)
{
    CHECK(address == W25Q64_CAPACITY_BYTES-2U*W25Q64_SECTOR_SIZE);
    memset(nav_flash, 0xff, sizeof(nav_flash)); return W25Q64_OK;
}
w25q64_status_t W25Q64_WriteVerified(uint32_t address, const uint8_t *data,
                                    uint32_t length, uint8_t *verify, uint32_t size)
{
    (void)address;
    if ((length > sizeof(nav_flash)) || (length > size)) { return W25Q64_ERROR_PARAM; }
    memcpy(nav_flash, data, length); memcpy(verify, nav_flash, length);
    return W25Q64_OK;
}

static uint32_t calls;
static uint32_t cancellations;
static uint32_t last_request;
static uint8_t last_zone;
static match_motion_t last_motion;
static uint8_t fail_start;
static uint8_t busy_start;
static uint32_t initiated;
static uint8_t start_motion(match_motion_t motion, uint8_t zone, uint32_t request)
{
    calls++;
    last_motion = motion;
    last_zone = zone;
    last_request = request;
    if(fail_start)return MATCH_START_REJECTED;
    if(busy_start)return MATCH_START_BUSY;
    initiated++;
    return MATCH_START_ACCEPTED;
}
static void cancel_motion(void) { cancellations++; }
static void changed(match_state_t state, match_checkpoint_t point)
{ (void)state; (void)point; }
static void init(match_control_t *flow)
{
    const match_ports_t ports = {start_motion, cancel_motion, changed};
    calls = cancellations = last_request = 0U;
    fail_start = 0U;
    busy_start=0U;initiated=0U;
    MatchControl_Init(flow, &ports);
}

static void complete_route(uint8_t zone)
{
    match_control_t flow;
    uint32_t old_request;
    uint32_t now = 100U;
    unsigned int i;
    static const match_motion_t expected[7] = {
        MATCH_MOTION_RAW, MATCH_MOTION_ROUGH, MATCH_MOTION_TEMP,
        MATCH_MOTION_RAW, MATCH_MOTION_ROUGH, MATCH_MOTION_TEMP, MATCH_MOTION_RETURN
    };
    init(&flow);
    CHECK(!MatchControl_Start(&flow, 0U, now));
    CHECK(!MatchControl_Ok(&flow, now));
    CHECK(MatchControl_Start(&flow, zone, now));
    CHECK(!MatchControl_Start(&flow, (uint8_t)(3U-zone), now));
    old_request = flow.active_request;
    MatchControl_Result(&flow, old_request, 1U, now); /* Not yet dispatched. */
    CHECK(flow.state == MATCH_LEAVING);
    MatchControl_Service(&flow, now);
    CHECK(calls == 1U && last_motion == MATCH_MOTION_LEAVE && last_zone == zone);
    MatchControl_Service(&flow, now + 1U);
    CHECK(calls == 1U); /* No repeated on-entry motion. */
    CHECK(!MatchControl_Ok(&flow, now + 2U));
    MatchControl_Result(&flow, old_request + 99U, 1U, now + 3U);
    CHECK(flow.state == MATCH_LEAVING);
    now += 1000U;
    MatchControl_Result(&flow, old_request, 1U, now);
    CHECK(flow.state == MATCH_MOVING && flow.checkpoint == MATCH_QR);
    CHECK(calls == 1U); /* QR is dispatched after leave, from the service loop. */
    MatchControl_Service(&flow, now);
    CHECK(calls == 2U && last_motion == MATCH_MOTION_QR);
    MatchControl_Result(&flow, old_request, 1U, now); /* Duplicate leave result. */
    CHECK(flow.state == MATCH_MOVING);
    now += 1000U;
    MatchControl_Result(&flow, last_request, 1U, now);
    CHECK(flow.state == MATCH_WAIT_OK && flow.checkpoint == MATCH_QR);
    CHECK(!MatchControl_Ok(&flow, now-1U)); /* Old/queued confirmation. */
    CHECK(!MatchControl_Ok(&flow, now));
    for (i = 0U; i < 7U; ++i) {
        uint32_t before = calls;
        old_request = last_request;
        now += 100U;
        CHECK(MatchControl_Ok(&flow, now));
        CHECK(flow.state == MATCH_MOVING && flow.zone == zone);
        CHECK(!MatchControl_Ok(&flow, now+1U)); /* Do not queue a second step. */
        MatchControl_Service(&flow, now);
        CHECK(calls == before+1U && last_motion == expected[i] && last_zone == zone);
        MatchControl_Result(&flow, old_request, 1U, now);
        CHECK(flow.state == MATCH_MOVING);
        now += 1000U;
        MatchControl_Result(&flow, last_request, 1U, now);
        CHECK(flow.checkpoint == (match_checkpoint_t)(i+1U));
        CHECK(flow.state == ((i == 6U) ? MATCH_FINISHED : MATCH_WAIT_OK));
    }
    CHECK(calls == 9U && cancellations == 0U);
    CHECK(!MatchControl_Ok(&flow, now+1U));
    CHECK(!MatchControl_Start(&flow, zone, now+1U));
}

static void failures_and_wrap(void)
{
    match_control_t flow;
    uint32_t stale;
    init(&flow);
    fail_start = 1U;
    CHECK(MatchControl_Start(&flow, 1U, 0U));
    MatchControl_Service(&flow, 1U);
    CHECK(flow.state == MATCH_FAULT && cancellations == 1U);
    MatchControl_Service(&flow, 2U);
    CHECK(calls == 1U);
    MatchControl_Reset(&flow);
    fail_start = 0U;
    CHECK(MatchControl_Start(&flow, 2U, 100U));
    MatchControl_Service(&flow, 100U);
    stale = last_request;
    MatchControl_Result(&flow, stale, 0U, 101U);
    CHECK(flow.state == MATCH_FAULT && !MatchControl_IsActive(&flow));
    MatchControl_Reset(&flow);
    CHECK(MatchControl_Start(&flow, 2U, 200U));
    MatchControl_Service(&flow, 200U);
    MatchControl_Result(&flow, stale, 1U, 201U);
    CHECK(flow.state == MATCH_LEAVING && flow.active_request != stale);
    MatchControl_Abort(&flow); /* STOP. */
    MatchControl_Result(&flow, last_request, 1U, 202U);
    CHECK(flow.state == MATCH_FAULT && flow.active_request == 0U);
    MatchControl_Reset(&flow);
    CHECK(MatchControl_Start(&flow, 1U, 0xFFFFFF00U));
    MatchControl_Service(&flow, 0x00000020U);
    CHECK(flow.state == MATCH_LEAVING); /* Tick wrap is not an instant timeout. */
    MatchControl_Service(&flow, (uint32_t)(0xFFFFFF00U + 15000U));
    CHECK(flow.state == MATCH_FAULT);
    MatchControl_Reset(&flow);
    CHECK(MatchControl_Start(&flow, 1U, 1U));
    MatchControl_Service(&flow, 1U);
    MatchControl_Result(&flow, last_request, 1U, 2U);
    MatchControl_Service(&flow, 2U);
    MatchControl_Service(&flow, 120002U);
    CHECK(flow.state == MATCH_FAULT);
}

static void parser_case(const char *input, const char *expected, int newline)
{
    const char *p;
    uint32_t last_tick = 0U;
    bt_button_event_t event;
    BT_Button_Init(); tick = 100U;
    for (p = input; *p != '\0'; ++p) {
        last_tick = tick;
        CHECK(BT_Button_ProcessByte((uint8_t)*p));
        tick += 20U;
        BT_Button_Poll();
        CHECK(!BT_Button_Pop(&event));
    }
    if (newline) {
        BT_Button_ProcessByte('\r'); BT_Button_ProcessByte('\n');
    } else {
        tick += 301U; BT_Button_Poll();
    }
    CHECK(BT_Button_Pop(&event));
    CHECK(event.id == BT_BUTTON_CUSTOM && strcmp(event.text, expected) == 0);
    CHECK(event.tick_ms == last_tick); /* Idle flush must retain input timestamp. */
    CHECK(!BT_Button_Pop(&event));
}

static void calibrated_defaults_survive_reference(void)
{
    nav_parameters_t *params = Nav_Params();
    CHECK(params->speed_scale == 0.650f && params->time_1000mm_ms == 1930.0f);
    Nav_Params_SetReferencePlanMs(2195.0f); /* Normal boot reference registration. */
    CHECK(params->time_1000mm_ms == 1930.0f);
    params->speed_scale = 0.2f;
    params->time_1000mm_ms = 2050.0f; /* Later runtime/Flash calibration. */
    Nav_Params_SetReferencePlanMs(2300.0f);
    CHECK(params->speed_scale == 0.2f && params->time_1000mm_ms == 2050.0f);
    Nav_Params_Reset(); /* Reset returns to bench calibration, not theory. */
    CHECK(params->speed_scale == 0.650f && params->time_1000mm_ms == 1930.0f);
    CHECK(params->counts_per_rev == 65536.0f && params->position_mode == 0U);
    CHECK(params->turn90_ms == 1700.0f && params->turn180_ms == 2800.0f);
    CHECK(params->arc90_ms == 0.0f && params->arc_spd_ratio == 1.0f);
    CHECK(params->distance_compensation == 0.0f);
}

static void enable_button_parser(const char *command,int newline)
{
    size_t i;bt_button_event_t event;
    BT_Button_Init();tick=1000U;
    for(i=0U;command[i];i++) {
        BT_Button_ProcessByte((uint8_t)command[i]);tick+=20U;BT_Button_Poll();
    }
    if(newline){BT_Button_ProcessByte('\r');BT_Button_ProcessByte('\n');}
    else{tick+=301U;BT_Button_Poll();}
    CHECK(BT_Button_Pop(&event));CHECK(event.id==BT_BUTTON_ENABLE);
    CHECK(!BT_Button_Pop(&event));
    BT_Button_Init();CHECK(BT_Button_ProcessCommand(command));
    CHECK(BT_Button_Pop(&event));CHECK(event.id==BT_BUTTON_ENABLE);
    CHECK(!BT_Button_Pop(&event));
}

static uint32_t fixture_crc(const uint8_t *bytes, uint32_t length)
{
    uint32_t crc = 0xffffffffU;
    uint32_t i;
    unsigned int bit;
    for (i = 0U; i < length; ++i) {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xedb88320U : 0U);
        }
    }
    return ~crc;
}

static void linear_compensation_and_flash(void)
{
    /* Fixed v4 on-disk layout, independent of the new v5 struct. */
    typedef struct { uint32_t magic; uint16_t version, size; uint8_t params[32]; uint32_t crc; } v4_fixture_t;
    typedef struct {uint32_t magic;uint16_t version,size;nav_parameters_t params;uint32_t crc;} v5_fixture_t;
    v4_fixture_t old = {0};
    v5_fixture_t old5={0};
    nav_parameters_t *p = Nav_Params();
    Nav_Params_Reset();
    CHECK(fabsf(Nav_Params_CompensatedDistance(1000.0f)-1000.0f) < 0.01f);
    CHECK(fabsf(Nav_Params_CompensatedDistance(1700.0f)-1700.0f) < 0.01f);
    CHECK(fabsf(Nav_Params_CompensatedDistance(-1000.0f)+1000.0f) < 0.01f);
    CHECK(Nav_Params_CompensatedDistance(0.0f) == 0.0f);
    CHECK(Nav_Params_ProcessCommand("dcomp 0.08"));
    CHECK(fabsf(Nav_Params_CompensatedDistance(1000.0f)-1080.0f) < 0.01f);
    CHECK(Nav_Params_ProcessCommand("dcompnan"));
    CHECK(p->distance_compensation == 0.08f);
    CHECK(Nav_Params_ProcessCommand("dcompinf"));
    CHECK(p->distance_compensation == 0.08f);
    CHECK(Nav_Params_ProcessCommand("dcomp0.31"));
    CHECK(p->distance_compensation == 0.08f);
    CHECK(Nav_Params_ProcessCommand("dcomp0.10x"));
    CHECK(p->distance_compensation == 0.08f);
    CHECK(Nav_Params_ProcessCommand("dcomp0"));
    CHECK(Nav_Params_CompensatedDistance(1000.0f) == 1000.0f);
    CHECK(Nav_Params_ProcessCommand("dcomp-0.05"));
    CHECK(fabsf(Nav_Params_CompensatedDistance(1000.0f)-950.0f) < 0.01f);
    CHECK(Nav_Params_SaveToFlash());
    Nav_Params_Reset();
    CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->distance_compensation == -0.05f);
    /* Flash carrying the former default pair cannot silently override the
     * requested V107 costs; custom costs and other saved fields remain. */
    p->turn90_ms=1000.0f;p->turn180_ms=2000.0f;CHECK(Nav_Params_SaveToFlash());
    memcpy(&old5,nav_flash,sizeof(old5));old5.version=7U;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));memcpy(nav_flash,&old5,sizeof(old5));
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->turn90_ms==1700.0f && p->turn180_ms==2800.0f && p->distance_compensation==-0.05f);
    p->turn90_ms=1100.0f;p->turn180_ms=2500.0f;CHECK(Nav_Params_SaveToFlash());
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());CHECK(p->turn90_ms==1100.0f && p->turn180_ms==2500.0f);
    nav_flash[12] ^= 1U;
    CHECK(!Nav_Params_LoadFromFlash());
    CHECK(p->distance_compensation == -0.05f);
    old.magic = 0x4e415632U; old.version = 4U; old.size = sizeof(old);
    p->speed_scale = 0.30f; p->time_1000mm_ms = 1870.0f;
    p->turn90_ms = 1100.0f; p->arc_spd_ratio = 0.75f;
    CHECK(offsetof(nav_parameters_t, distance_compensation) == 32U);
    memcpy(old.params, p, sizeof(old.params));
    old.crc = fixture_crc((const uint8_t *)&old, offsetof(v4_fixture_t, crc));
    memset(nav_flash, 0xff, sizeof(nav_flash)); memcpy(nav_flash, &old, sizeof(old));
    Nav_Params_Reset();
    CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale == 0.65f && p->time_1000mm_ms == 1870.0f);
    CHECK(p->turn90_ms == 1100.0f && p->arc_spd_ratio == 0.75f);
    CHECK(p->distance_compensation == 0.0f);
    CHECK(Nav_Params_SaveToFlash());
    Nav_Params_Reset(); CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale == 0.65f && p->distance_compensation == 0.0f);
    memcpy(nav_flash, &old, sizeof(old)); nav_flash[8] ^= 1U;
    CHECK(!Nav_Params_LoadFromFlash());
    /* Old v5/v6 speeds migrate to requested .35; retain timing/counts.
     * v5 additionally keeps the established zero-distance migration. */
    old5.magic=0x4e415632U;old5.version=5U;old5.size=sizeof(old5);
    old5.params=*p;old5.params.speed_scale=0.30f;old5.params.time_1000mm_ms=1880.0f;
    old5.params.distance_compensation=0.03f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));
    memset(nav_flash,0xff,sizeof(nav_flash));memcpy(nav_flash,&old5,sizeof(old5));Nav_Params_Reset();
    CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->distance_compensation==0.0f && p->speed_scale==0.65f && p->time_1000mm_ms==1880.0f);
    CHECK(Nav_Params_CompensatedDistance(1000.0f)==1000.0f);
    CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->distance_compensation==0.0f && p->time_1000mm_ms==1880.0f);
    old5.version=6U;old5.params.speed_scale=0.30f;old5.params.distance_compensation=0.02f;
    old5.params.counts_per_rev=64000.0f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));
    memcpy(nav_flash,&old5,sizeof(old5));Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.65f && p->distance_compensation==0.02f && p->counts_per_rev==64000.0f);
    /* Former default in v7 migrates once; an explicit .35 saved by v11 can
     * still be restored for trial rollback. Custom costs/counts survive. */
    old5.version=7U;old5.params=*p;old5.params.speed_scale=0.35f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));memcpy(nav_flash,&old5,sizeof(old5));
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());CHECK(p->speed_scale==0.65f && p->counts_per_rev==64000.0f);
    CHECK(Nav_Params_SaveToFlash());memcpy(&old5,nav_flash,sizeof(old5));CHECK(old5.version==11U);
    old5.version=9U;old5.params=*p;old5.params.speed_scale=0.5f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));memcpy(nav_flash,&old5,sizeof(old5));
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.65f && p->counts_per_rev==64000.0f);
    CHECK(!memcmp(nav_flash,&old5,sizeof(old5)));
    p->speed_scale=0.5f;CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.5f); /* Explicit v11 rollback is retained. */
    old5.version=10U;old5.params=*p;old5.params.speed_scale=0.75f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));memcpy(nav_flash,&old5,sizeof(old5));
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.65f && p->counts_per_rev==64000.0f);
    CHECK(!memcmp(nav_flash,&old5,sizeof(old5))); /* RAM migration only. */
    p->speed_scale=0.75f;CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.75f); /* Explicit v11 tuning is preserved. */
    /* Upgrade only the old default; explicit v8 speeds and new v11 tuning survive. */
    old5.version=8U;old5.params=*p;old5.params.speed_scale=0.4f;
    old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));memcpy(nav_flash,&old5,sizeof(old5));
    Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.65f && p->counts_per_rev==64000.0f);
    CHECK(!memcmp(nav_flash,&old5,sizeof(old5))); /* No implicit Flash write. */
    old5.params.speed_scale=0.32f;old5.crc=fixture_crc((const uint8_t *)&old5,offsetof(v5_fixture_t,crc));
    memcpy(nav_flash,&old5,sizeof(old5));Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.32f && p->counts_per_rev==64000.0f);
    p->speed_scale=0.4f;CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.4f); /* A v11 explicit rollback must stay at0.4. */
    p->speed_scale=0.35f;CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.35f && p->counts_per_rev==64000.0f);
    p->speed_scale=0.32f;CHECK(Nav_Params_SaveToFlash());Nav_Params_Reset();CHECK(Nav_Params_LoadFromFlash());
    CHECK(p->speed_scale==0.32f && p->counts_per_rev==64000.0f); /* later explicit v11 tuning persists */
}

static void return_direction_and_projection(void)
{
    float vx;
    float vy;
    float wheels[4];
    unsigned int i;
    static const float right_feedback[4] = {2.0f, -2.0f, -2.0f, 2.0f};
    static const float left_feedback[4] = {-2.0f, 2.0f, 2.0f, -2.0f};
    /* Top-right departure: initial nose south; forward south, right west. */
    NavFrame_MapToBody(0.0f, -1.0f, 0.0f, 0.0f, -300.0f, &vx, &vy);
    CHECK(fabsf(vx-300.0f) < 0.001f && fabsf(vy) < 0.001f);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 0.0f, -125.0f, 0.0f, 3.0f) == 1);
    /* Top-right return: actual nose NORTH (180 from departure).
     * Target movement north then east is body FORWARD then RIGHT. */
    NavFrame_MapToBody(0.0f, -1.0f, 180.0f, 0.0f, 300.0f, &vx, &vy);
    CHECK(fabsf(vx-300.0f) < 0.001f && fabsf(vy) < 0.001f);
    NavFrame_MapToBody(0.0f, -1.0f, 180.0f, 125.0f, 0.0f, &vx, &vy);
    CHECK(fabsf(vx) < 0.001f && fabsf(vy-125.0f) < 0.001f);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 180.0f, 125.0f, 0.0f, 3.0f) == 1);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, -180.0f, 125.0f, 0.0f, 3.0f) == 1);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 181.5f, 125.0f, 0.0f, 3.0f) == 1);
    /* Before turning around, that SAME map-east vector is body LEFT. */
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 0.0f, 125.0f, 0.0f, 3.0f) == -1);
    /* A quarter turn makes map-east forward, not lateral. */
    NavFrame_MapToBody(0.0f, -1.0f, 90.0f, 125.0f, 0.0f, &vx, &vy);
    CHECK(fabsf(vx-125.0f) < 0.001f && fabsf(vy) < 0.001f);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 90.0f, 125.0f, 0.0f, 3.0f) == 0);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 170.0f, 125.0f, 0.0f, 3.0f) == 0);
    /* Bottom-right departure/return must retain the mirrored directions. */
    CHECK(NavFrame_StrafeSide(0.0f, 1.0f, 0.0f, -125.0f, 0.0f, 3.0f) == -1);
    NavFrame_MapToBody(0.0f, 1.0f, 180.0f, 0.0f, -300.0f, &vx, &vy);
    CHECK(fabsf(vx-300.0f) < 0.001f && fabsf(vy) < 0.001f);
    CHECK(NavFrame_StrafeSide(0.0f, 1.0f, 180.0f, 125.0f, 0.0f, 3.0f) == -1);
    CHECK(NavFrame_StrafeSide(0.0f, 1.0f, 360.0f, -125.0f, 0.0f, 3.0f) == -1);
    CHECK(NavFrame_StrafeSide(0.0f, -1.0f, 180.0f, 0.0f, 0.0f, 3.0f) == 0);
    /* Actual strafe encoder deltas alternate signs; projection must not cancel. */
    NavFrame_WheelValues(0.0f, 1.0f, wheels);
    for (i = 0U; i < 4U; ++i) { CHECK(right_feedback[i]/wheels[i] == 2.0f); }
    NavFrame_WheelValues(0.0f, -1.0f, wheels);
    for (i = 0U; i < 4U; ++i) { CHECK(left_feedback[i]/wheels[i] == 2.0f); }
    NavFrame_WheelValues(1.0f, 0.0f, wheels);
    for (i = 0U; i < 4U; ++i) { CHECK(wheels[i] == 1.0f); }
}

static void coarse_segment_feedback(void)
{
    /* Residuals are diagnostics in the manual serial flow; no slow approach. */
    CHECK(NavFeedback_Check(6.649f*255.694226f, 6.084f*255.694226f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(750.0f, 742.0f) == NAV_MOTION_OK);
    CHECK(NavFeedback_Check(900.0f, 871.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(850.0f, 827.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 0.0f) == NAV_MOTION_NO_MOVEMENT);
    CHECK(NavFeedback_Check(1700.0f, -1556.0f) == NAV_MOTION_REVERSED);
    CHECK(NavFeedback_Check(1700.0f, 400.0f) == NAV_MOTION_NO_MOVEMENT);
    CHECK(NavFeedback_Check(1700.0f, 1450.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 1400.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 2000.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(100.0f, 40.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 1530.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 1680.0f) == NAV_MOTION_OK);
    CHECK(NavFeedback_Check(1700.0f, 1679.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(1700.0f, 1710.0f) == NAV_MOTION_OK);
    CHECK(NavFeedback_Check(1700.0f, 1730.0f) == NAV_MOTION_RESIDUAL);
    CHECK(NavFeedback_Check(0.0f, 0.0f) == NAV_MOTION_INVALID);
    CHECK(NavFeedback_Check(1700.0f, NAN) == NAV_MOTION_INVALID);
    CHECK(NavFeedback_Check(INFINITY, 1700.0f) == NAV_MOTION_INVALID);
    CHECK(NavFeedback_Check(1700.0f, INFINITY) == NAV_MOTION_INVALID);
    CHECK(NavFeedback_SettleCounts(65536.0f, 255.694226f) < 130.0f);
    CHECK(NavFeedback_SettleCounts(65536.0f, 255.694226f) > 125.0f);
    CHECK(NavFeedback_SettleCounts(3200.0f, 255.694226f) < 7.0f);
    /* 20 RPM advances about 437 counts per 20ms at this encoder scale:
     * the legacy 1500-count gate missed that ongoing low-speed motion. */
    CHECK((20.0f/60.0f*0.020f*65536.0f) > NavFeedback_SettleCounts(65536.0f, 255.694226f));
    CHECK(NavFeedback_Check(1700.0f, 1556.0f) != NAV_MOTION_OK);
    CHECK(NavFeedback_Check(1700.0f, 1650.0f) != NAV_MOTION_OK);
    CHECK(NavFeedback_Check(1700.0f, 1690.0f) == NAV_MOTION_OK);
}

static void reverse_recenter_feedback(void)
{
    double rev[4]={-0.316,-0.314,-0.319,-0.316};float values[4],spread,progress;unsigned int i;
    /* Replay 00:43:04: expected BACK79, actual four negative wheel deltas.
     * The same projection and gate used by production must accept +81mm. */
    NavFrame_VelocityWheelValues(-1,0,values);
    progress=NavFeedback_ProjectWheels(rev,values,15U,&spread)*255.694226f;
    CHECK(progress>80.0f && progress<82.0f && spread>0.004f && spread<0.006f);
    CHECK(NavFeedback_Check(79.0f,progress)==NAV_MOTION_OK);
    for(i=0U;i<4U;i++)rev[i]=-rev[i];
    progress=NavFeedback_ProjectWheels(rev,values,15U,&spread)*255.694226f;
    CHECK(NavFeedback_Check(79.0f,progress)==NAV_MOTION_REVERSED); /* genuine wrong way */
    NavFrame_VelocityWheelValues(1,0,values);
    CHECK(NavFeedback_ProjectWheels(rev,values,15U,&spread)>0.31f);
    NavFrame_VelocityWheelValues(-1,1,values); /* lateral overrides forward sign */
    rev[0]=0.31;rev[1]=-0.32;rev[2]=-0.31;rev[3]=0.32;
    CHECK(fabsf(NavFeedback_ProjectWheels(rev,values,15U,&spread)-0.315f)<1e-6f);
    rev[0]=100.0;CHECK(NavFeedback_ProjectWheels(rev,values,15U,&spread)<0.33f);
    CHECK(NavFeedback_ProjectWheels(rev,values,0U,&spread)==0.0f);
    rev[0]=NAN;CHECK(isnan(NavFeedback_ProjectWheels(rev,values,15U,&spread)));
    CHECK(isnan(NavFeedback_ProjectWheels(NULL,values,15U,&spread)));
}

static void body_280_rotation_clearance(void)
{
    int angle;
    CHECK(GRID_NAV_CAR_MM == 280U);
    CHECK(GRID_NAV_CAR_HALF_CELLS == 3U);
    GridNav_Init();
    /* At y=750, both sides of the 400mm corridor border forbidden blocks.
     * The centered 280mm square fits throughout rotation; 3mm drift at 45deg
     * must still fail. Check production SAT geometry, not an offline replica. */
    for (angle = 0; angle <= 180; angle += 5) {
        CHECK(GridNav_RotatedBodyFreeMm(1200U, 750U, (int16_t)angle));
    }
    CHECK(GridNav_RotatedBodyFreeMm(1202U, 750U, 45));
    CHECK(GridNav_RotatedBodyFreeMm(1198U, 750U, 45));
    CHECK(!GridNav_RotatedBodyFreeMm(1203U, 750U, 45));
    CHECK(!GridNav_RotatedBodyFreeMm(1197U, 750U, 45));
    CHECK(!GridNav_RotatedBodyFreeMm(100U, 750U, 0));
}

static uint8_t blend_clearance(uint16_t x,uint16_t y,int16_t yaw)
{
    return GridNav_RotatedBodyFreeMarginMm(x,y,yaw,10U);
}

static const nav_blend_control_t normal_blend_control={
    320.0f,1065.0f,0.4f*1244.0f,0.04f,120.0f/250.0f,0.045f,0.0005f,0.0035f,0.08f,0.02f,
    0.0f,6.0f,0.5f,160.0f,1.0f
};

static void blend_math_and_geometry(void)
{
    nav_blend_t b;
    float w[4]={-30.0f,30.0f,-30.0f,30.0f};
    float f,r,o;
    CHECK(ChassisMotor_RampStepRpm(0,100,0.004f,292.0f)==2);
    CHECK(ChassisMotor_RampStepRpm(100,-100,0.004f,292.0f)==99);
    CHECK(ChassisMotor_RampStepRpm(-1,100,0.004f,292.0f)==0);
    CHECK(ChassisMotor_RampStepRpm(100,0,0.004f,900.0f)==96);
    CHECK(NavBlend_Init(&b,0.0f,1.0f,0.0f,1.0f,175.0f,0.0f,90.0f));
    NavBlend_Integrate(&b,w,45.0f);
    CHECK(fabsf(b.progress_mm)<0.001f && fabsf(b.cross_mm)<0.001f);
    /* 10mm map-forward plus 30mm wheel rotation at 45deg heading. */
    w[0]=14.1421356f-30.0f; w[1]=30.0f;
    w[2]=-30.0f; w[3]=14.1421356f+30.0f;
    NavBlend_Integrate(&b,w,45.0f);
    CHECK(fabsf(b.progress_mm-10.0f)<0.001f && fabsf(b.cross_mm)<0.001f);
    CHECK(!NavBlend_Init(&b,0.0f,1.0f,0.0f,1.0f,74.0f,0.0f,90.0f));
    CHECK(!NavBlend_Init(&b,NAN,1.0f,0.0f,1.0f,175.0f,0.0f,90.0f));
    CHECK(NavBlend_AngleError(179.0f,-179.0f)==-2.0f);
    CHECK(NavBlend_Init(&b,0.0f,-1.0f,0.0f,1.0f,125.0f,0.0f,180.0f));
    CHECK(!NavBlend_Clear(&b,1200.0f,350.0f,blend_clearance));
    b.turn_start_mm=40.0f;
    CHECK(NavBlend_Clear(&b,1200.0f,350.0f,blend_clearance));
    CHECK(!NavBlend_Clear(&b,1200.0f,750.0f,blend_clearance));
    CHECK(NavBlend_Init(&b,0.0f,1.0f,0.0f,-1.0f,125.0f,0.0f,180.0f));
    CHECK(!NavBlend_Clear(&b,1200.0f,2150.0f,blend_clearance)); /*85mm RAW pad still blocks a closer sweep*/
    b.turn_start_mm=40.0f;
    CHECK(NavBlend_Clear(&b,1200.0f,2050.0f,blend_clearance));
    CHECK(!NavBlend_Clear(&b,10.0f,2050.0f,blend_clearance));
    b.progress_mm=b.span_mm; /* Finish yaw without any more translation. */
    NavBlend_Command(&b,170.0f,0.0f,&normal_blend_control,&f,&r,&o);
    CHECK(f==0.0f && r==0.0f && o>0.0f);
    CHECK(NavBlend_Init(&b,0.0f,1.0f,0.0f,1.0f,300.0f,0.0f,180.0f));
    b.progress_mm=150.0f;
    NavBlend_Command(&b,90.0f,0.0f,&normal_blend_control,&f,&r,&o);
    CHECK(fabsf(o-120.0f/250.0f)<0.001f); /* At half-turn, still drive final yaw target. */
    CHECK(fabsf(hypotf(f,r)*1065.0f-320.0f)<0.1f); /* Same cruise cap as spd=0.30. */
    b.progress_mm=299.0f;
    NavBlend_Command(&b,90.0f,0.0f,&normal_blend_control,&f,&r,&o);
    CHECK(f==0.0f && r==0.0f && o>0.4f); /* Translation done must not stall half-yaw. */
    CHECK(!NavBlend_Ready(&b,90.0f,0.0f));
    b.progress_mm=50.0f;
    CHECK(NavBlend_Ready(&b,180.0f,0.0f)); /* Finish yaw early; remaining line is measured, not lost. */
    b.progress_mm=0.0f;
    CHECK(!NavBlend_Ready(&b,180.0f,0.0f));
}

static void blend_simulation(float sweep,float speed,float span,float retreat,float ux,float uy)
{
    nav_blend_t b;
    nav_blend_control_t control=normal_blend_control;
    chassis_field_profile_t profile={0};
    int16_t applied_rpm[4]={0};
    float applied[4]={0}, accumulated[4]={0}, target[4]={0}, peak_cross=0.0f;
    float yaw=-720.0f, previous_yaw=yaw, gyro=0.0f, path=0.0f, cross=0.0f;
    unsigned int step,i;
    float f=0.0f,r=0.0f,o=0.0f;
    float map_vx=0.0f,map_vy=0.0f;
    CHECK(NavBlend_Init(&b,0.0f,1.0f,ux,uy,span,yaw,yaw+sweep));
    b.turn_start_mm=retreat; control.max_mm_s=speed; control.cross_cap_mm_s=speed*0.5f;
    for(step=0U;step<3750U;++step) {
        float actual_f,actual_r,actual_o, mid,c,s,dx,dy;
        if(step%5U==0U) {
            NavBlend_Command(&b,yaw,gyro,&control,&f,&r,&o);
            NavFrame_BodyToMap(0.0f,1.0f,yaw,f,r,&map_vx,&map_vy);
        }
        ChassisField_Step(&profile,map_vx,map_vy,o,0.0f,1.0f,yaw,0.004f,250.0f,600.0f,292.0f,900.0f);
        for(i=0U;i<4U;++i) {
            target[i]=profile.wheel[i]*250.0f;
            int16_t rpm=(int16_t)(target[i]+copysignf(0.5f,target[i]));
            applied_rpm[i]=rpm; /* Field profile owns the ramp; never filter each wheel again. */
            applied[i]=(float)applied_rpm[i]*1065.0f/250.0f;
            accumulated[i]+=applied[i]*0.004f;
        }
        actual_f=(applied[0]+applied[1]+applied[2]+applied[3])*0.25f;
        actual_r=(applied[0]-applied[1]-applied[2]+applied[3])*0.25f;
        actual_o=(-applied[0]+applied[1]-applied[2]+applied[3])*0.25f;
        gyro=actual_o/300.0f*57.29578f;
        mid=yaw+gyro*0.002f; yaw+=gyro*0.004f;
        c=cosf(mid*0.0174532925f); s=sinf(mid*0.0174532925f);
        dx=(-actual_f*s+actual_r*c)*0.004f;
        dy=(actual_f*c+actual_r*s)*0.004f;
        path+=dx*ux+dy*uy; cross+=-dx*uy+dy*ux;
        peak_cross=fmaxf(peak_cross,fabsf(cross));
        if(step%5U==4U) {
            NavBlend_Observe(&b,accumulated,previous_yaw+0.5f*NavBlend_AngleError(yaw,previous_yaw),0.020f);
            memset(accumulated,0,sizeof(accumulated)); previous_yaw=yaw;
            if(NavBlend_Ready(&b,yaw,gyro)) { break; }
        }
    }
    test_blend_metrics[blend_case_index][0]=(float)step;
    test_blend_metrics[blend_case_index][1]=path;
    test_blend_metrics[blend_case_index][2]=cross;
    test_blend_metrics[blend_case_index][3]=NavBlend_AngleError(-720.0f+sweep,yaw);
    test_blend_metrics[blend_case_index][4]=b.progress_mm;
    test_blend_metrics[blend_case_index][5]=peak_cross;
    blend_case_index++;
    CHECK(step<2000U); /* Less than 8s with the actual 4ms quantized wheel ramp. */
    CHECK(fabsf(path-span)<15.0f);
    CHECK(fabsf(cross)<=8.0f);
    CHECK(peak_cross<8.0f);
    CHECK(fabsf(NavBlend_AngleError(-720.0f+sweep,yaw))<=3.0f);
    CHECK(fabsf(path-b.progress_mm)<2.0f); /* Sampled wheel+IMU integration. */
}

static zone2_route_t *zone_flow;
static zone2_action_t zone_action;
static uint32_t zone_id,zone_finished,zone_cancelled;
static uint8_t zone_success,zone_reject,zone_sync;
static uint8_t zone_start(const zone2_action_t *a,uint32_t id)
{
    zone_action=*a;zone_id=id;
    if(zone_sync)Zone2Route_Result(zone_flow,id,1U);
    return !zone_reject;
}
static void zone_cancel(void){++zone_cancelled;}
static void zone_finish(uint8_t success){++zone_finished;zone_success=success;}
static void zone2_routes_and_lifecycle(void)
{
    zone2_route_t r;
    const zone2_ports_t ports={zone_start,zone_cancel,zone_finish};
    const match_motion_t motions[]={MATCH_MOTION_QR,MATCH_MOTION_RAW,MATCH_MOTION_ROUGH,MATCH_MOTION_TEMP,
        MATCH_MOTION_RAW,MATCH_MOTION_ROUGH,MATCH_MOTION_TEMP,MATCH_MOTION_RETURN};
    const match_checkpoint_t checkpoints[]={MATCH_QR,MATCH_RAW1,MATCH_ROUGH1,MATCH_TEMP1,MATCH_RAW2,MATCH_ROUGH2,MATCH_TEMP2,MATCH_RETURN};
    const int yaw[]={0,0,180,90,0,180,90,90};
    unsigned int leg,i;
    Zone2Route_Init(&r,&ports);zone_flow=&r;
    for(leg=0;leg<8;++leg) {
        zone_finished=zone_cancelled=0U;zone_sync=zone_reject=0U;
        CHECK(Zone2Route_Start(&r,motions[leg],checkpoints[leg]));
        CHECK(!Zone2Route_Start(&r,motions[leg],checkpoints[leg]));
        if(motions[leg]==MATCH_MOTION_QR) {
            grid_nav_segment_t segments[4];uint16_t count;float f,right;int angle;
            CHECK(r.count==2U); /* No sideways pre-align or field correction action. */
            CHECK(r.actions[0].kind==ZONE2_NAV && r.actions[0].x==2125U && r.actions[0].y==1150U);
            CHECK(r.actions[1].kind==ZONE2_TURN && r.actions[1].yaw==0);
            GridNav_Init();GridNav_SetPoseMm(2125U,1950U);
            CHECK(GridNav_PlanTargetMm(r.actions[0].x,r.actions[0].y));
            CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
            CHECK(segments[0].dx==0 && segments[0].dy==1 && segments[0].length==16U);
            /* South-facing arrival: LEFT +90 faces the original front east. */
            CHECK(fabsf(NavBlend_AngleError(0.0f,-90.0f)-90.0f)<0.001f);
            NavFrame_MapToBody(0.0f,-1.0f,90.0f,1.0f,0.0f,&f,&right);
            CHECK(fabsf(f-1.0f)<0.001f && fabsf(right)<0.001f);
            for(angle=-90;angle<=0;angle+=3)
                CHECK(GridNav_RotatedBodyFreeMarginMm(2125U,1150U,(int16_t)angle,10U));
        }
        if(motions[leg]==MATCH_MOTION_TEMP) {
            CHECK(r.actions[0].x==350 && r.actions[0].y==400);
            CHECK(r.actions[1].x==350 && r.actions[1].y==1200);
            CHECK(r.actions[0].extra_mm==0U && !r.actions[0].final_point);
            CHECK(r.actions[1].extra_mm==0U && r.actions[1].final_point);
            CHECK(r.actions[2].extra_mm==0U); /* turn has no distance addition */
            /* Arriving north-facing already gives the left arm a west view.
             * Depart ROUGH west-facing without a U-turn; only SW corner turns. */
            CHECK(fabsf(NavBlend_AngleError((float)r.actions[2].yaw,90.0f))<0.001f);
            CHECK(fabsf(NavBlend_AngleError(180.0f,(float)yaw[leg-1U]))<0.001f);
        } else if(checkpoints[leg]!=MATCH_RAW2) {
            for(i=0;i<r.count;++i)CHECK(r.actions[i].extra_mm==0U);
        }
        if(checkpoints[leg]==MATCH_RAW2) {
            CHECK(r.actions[0].x==350 && r.actions[0].y==2000);
            CHECK(r.actions[1].x==1250 && r.actions[1].y==2100);
            CHECK(r.actions[0].extra_mm==0U && !r.actions[0].final_point);
            CHECK(r.actions[1].extra_mm==0U && r.actions[1].final_point);
            CHECK(r.actions[2].extra_mm==0U);
        }
        if(checkpoints[leg]==MATCH_RAW1) {
            CHECK(r.count==2U && r.actions[0].x==1250U && r.actions[0].y==2100U && r.actions[0].final_point);
        }
        if(motions[leg]==MATCH_MOTION_ROUGH) {
            CHECK(r.count==3U && r.actions[0].x==1250U && r.actions[0].y==2100U && r.actions[0].final_point);
            CHECK(r.actions[1].x==1200U && r.actions[1].y==400U && r.actions[1].final_point);
        }
        if(motions[leg]==MATCH_MOTION_RETURN) {
            CHECK(r.count==3U && r.actions[0].x==350U && r.actions[0].y==2000U);
            CHECK(r.actions[1].x==2125U && r.actions[1].y==2000U && r.actions[1].final_point);
        }
        CHECK(r.actions[r.count-1U].yaw==yaw[leg]);
        for(i=0;i<r.count;++i) {
            CHECK(r.actions[i].kind==ZONE2_NAV || r.actions[i].kind==ZONE2_TURN);
            uint32_t prior=zone_id;
            CHECK(r.running && zone_finished==0U);
            Zone2Route_Service(&r,100U+100U*i);
            CHECK(zone_id>prior && r.index==i);
            Zone2Route_Result(&r,zone_id+1U,1U);CHECK(r.index==i);
            Zone2Route_Result(&r,zone_id,1U);
            Zone2Route_Result(&r,zone_id,1U);CHECK(r.index==i+1U);
            if(i+1U<r.count)CHECK(r.running && zone_finished==0U); /* no mid-corner WAIT_OK */
        }
        CHECK(!r.running && zone_finished==1U && zone_success==1U && zone_cancelled==0U);
    }
    zone_finished=zone_cancelled=0U;
    CHECK(Zone2Route_Start(&r,MATCH_MOTION_TEMP,MATCH_TEMP1));
    Zone2Route_Service(&r,0xfffffff0U);Zone2Route_Service(&r,120001U);
    CHECK(!r.running && zone_finished==1U && !zone_success && zone_cancelled==1U);
    CHECK(Zone2Route_Start(&r,MATCH_MOTION_RAW,MATCH_RAW1));
    Zone2Route_Service(&r,100U); { uint32_t old=zone_id;
      Zone2Route_Cancel(&r);CHECK(Zone2Route_Start(&r,MATCH_MOTION_RAW,MATCH_RAW1));
      Zone2Route_Service(&r,200U);Zone2Route_Result(&r,old,1U);CHECK(r.index==0 && zone_id>old); }
    Zone2Route_Result(&r,zone_id,0U);CHECK(!r.running && !zone_success);
    zone_reject=1U;CHECK(Zone2Route_Start(&r,MATCH_MOTION_QR,MATCH_QR));
    Zone2Route_Service(&r,300U);CHECK(!r.running && !zone_success);
    zone_reject=0U;zone_sync=1U;zone_finished=0U;
    CHECK(Zone2Route_Start(&r,MATCH_MOTION_TEMP,MATCH_TEMP1));
    for(i=0;i<3;++i)Zone2Route_Service(&r,400U+i);
    CHECK(!r.running && zone_finished==1U && zone_success);
}

static void busy_dispatch_is_not_a_motion_failure(void)
{
    match_control_t flow;uint32_t request,entered;
    init(&flow);
    CHECK(MatchControl_Start(&flow,2U,100U));MatchControl_Service(&flow,100U);
    MatchControl_Result(&flow,last_request,1U,101U);MatchControl_Service(&flow,101U);
    MatchControl_Result(&flow,last_request,1U,200U);
    CHECK(flow.state==MATCH_WAIT_OK && initiated==2U);
    CHECK(MatchControl_Ok(&flow,201U)); /* immediately after WAIT_OK */
    request=flow.active_request;entered=flow.entered_tick;busy_start=1U;
    MatchControl_Service(&flow,201U);MatchControl_Service(&flow,202U);MatchControl_Service(&flow,250U);
    CHECK(flow.state==MATCH_MOVING && flow.pending && cancellations==0U && initiated==2U);
    CHECK(flow.active_request==request && flow.entered_tick==entered && flow.checkpoint==MATCH_RAW1);
    MatchControl_Result(&flow,request,1U,250U); /* undelivered busy request cannot complete */
    CHECK(flow.pending && flow.state==MATCH_MOVING);
    CHECK(!MatchControl_Ok(&flow,250U));
    busy_start=0U;MatchControl_Service(&flow,251U);MatchControl_Service(&flow,252U);
    CHECK(!flow.pending && initiated==3U && flow.active_request==request);
    MatchControl_Result(&flow,request,1U,300U);
    CHECK(flow.state==MATCH_WAIT_OK && cancellations==0U);
    CHECK(MatchControl_Ok(&flow,301U));request=flow.active_request;busy_start=1U;
    MatchControl_Service(&flow,302U);MatchControl_Reset(&flow);
    MatchControl_Result(&flow,request,1U,303U);
    CHECK(flow.state==MATCH_IDLE && initiated==3U); /* reset cancels pending dispatch */
    busy_start=0U;CHECK(MatchControl_Start(&flow,2U,400U));MatchControl_Service(&flow,400U);
    CHECK(flow.active_request>request && initiated==4U);
    /* Transient waits do not bypass the existing movement deadline. */
    init(&flow);CHECK(MatchControl_Start(&flow,2U,1U));MatchControl_Service(&flow,1U);
    MatchControl_Result(&flow,last_request,1U,0xfffffff0U);busy_start=1U;
    MatchControl_Service(&flow,0xfffffff0U);MatchControl_Service(&flow,0x20U);
    CHECK(flow.state==MATCH_MOVING && flow.pending);
    MatchControl_Service(&flow,(uint32_t)(0xfffffff0U+120000U));
    CHECK(flow.state==MATCH_FAULT && cancellations==1U);
}

static void zone2_straight_routes_and_station_extras(void)
{
    grid_nav_segment_t segments[4];uint16_t count;
    GridNav_Init();GridNav_SetPoseMm(1200U,400U);
    /* Reproduce the log's A* detour through y=300, then prefer the real-mm
     * westbound line at y=400 instead of two 100mm lateral segments. */
    CHECK(GridNav_PlanTargetMm(350U,400U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count>1U);
    CHECK(GridNav_PlanStraightTargetMm(350U,400U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
    CHECK(segments[0].dx==-1 && segments[0].dy==0 && segments[0].length==17U);
    GridNav_SetPoseMm(350U,400U);
    CHECK(GridNav_PlanStraightTargetMm(350U,1200U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
    CHECK(segments[0].dx==0 && segments[0].dy==-1 && segments[0].length==16U);
    GridNav_SetPoseMm(1200U,400U);
    GridNav_SetObstacle(GridNav_MmToCellX(750U),GridNav_MmToCellY(400U),1U);
    CHECK(!GridNav_PlanStraightTargetMm(350U,400U)); /* dynamic obstacles still matter */
    CHECK(GridNav_PlanTargetMm(350U,400U)); /* blocked direct route retains A* fallback */
    GridNav_Init();GridNav_SetPoseMm(1200U,425U);
    CHECK(!GridNav_PlanStraightTargetMm(350U,425U)); /* actual body intersects yellow */
    GridNav_SetPoseMm(1200U,400U);
    CHECK(!GridNav_PlanStraightTargetMm(350U,450U)); /* diagonal */
    CHECK(GridNav_PlanStraightTargetMm(375U,400U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==825U); /* no 50mm rounding */
    CHECK(!GridNav_PlanStraightTargetMm(0U,400U)); /* field edge */
    CHECK(Zone2Route_FixedExtraMm(MATCH_RAW1)==0U && Zone2Route_FixedExtraMm(MATCH_RAW2)==0U);
    CHECK(Zone2Route_FixedExtraMm(MATCH_ROUGH1)==0U && Zone2Route_FixedExtraMm(MATCH_ROUGH2)==0U);
    CHECK(Zone2Route_FixedExtraMm(MATCH_TEMP1)==0U && Zone2Route_FixedExtraMm(MATCH_TEMP2)==0U);
    CHECK(Zone2Route_FixedExtraMm(MATCH_QR)==0U && Zone2Route_FixedExtraMm(MATCH_RETURN)==0U);
}

static void qr_return_exact_geometry(void)
{
    grid_nav_segment_t segments[4];uint16_t count,goal_x,goal_y,x,y;int angle;
    GridNav_Init();GridNav_SetPoseMm(350U,1200U);
    for(angle=90;angle>=0;angle-=3)
        CHECK(GridNav_RotatedBodyFreeMm(350U,1200U,(int16_t)angle));
    CHECK(GridNav_PlanStraightTargetMm(2125U,1200U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
    CHECK(segments[0].dx==1 && segments[0].dy==0);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==1775U);
    GridNav_GetTargetMm(&goal_x,&goal_y);CHECK(goal_x==2125U && goal_y==1200U);
    for(x=350U;x<=2125U;x+=5U)
        CHECK(GridNav_RotatedBodyFreeMarginMm(x,1200U,0,10U));
    for(angle=0;angle>=-90;angle-=3)
        CHECK(GridNav_RotatedBodyFreeMarginMm(2125U,1200U,(int16_t)angle,10U));
    GridNav_SetPoseMm(2125U,1200U);CHECK(GridNav_PlanStraightTargetMm(2125U,350U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
    CHECK(segments[0].dx==0 && segments[0].dy==1);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==850U);
    for(y=350U;y<=1200U;y+=5U)
        CHECK(GridNav_RotatedBodyFreeMarginMm(2125U,y,-90,10U));
    /* A nonzero move inside one cell must execute; a zero move must not. */
    GridNav_SetPoseMm(350U,1200U);CHECK(GridNav_PlanStraightTargetMm(375U,1200U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==1U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==25U);
    CHECK(GridNav_PlanStraightTargetMm(350U,1200U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==0U);
    CHECK(GridNav_GetSegmentDistanceMm(0U)==0U);
    /* Replacing an exact-mm plan with A* must clear its distance override. */
    CHECK(GridNav_PlanStraightTargetMm(2125U,1200U));
    GridNav_SetPoseMm(2125U,1950U);CHECK(GridNav_PlanTargetMm(2125U,1200U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==750U);
    GridNav_SetPoseMm(350U,1200U);
    GridNav_SetObstacle(GridNav_MmToCellX(1200U),GridNav_MmToCellY(1200U),1U);
    CHECK(!GridNav_PlanStraightTargetMm(2125U,1200U));
}

static void post_alignment_lane_geometry(void)
{
    grid_nav_segment_t segments[4];uint16_t count;
    /* Reproduce the real station1 trace: F+68/R+3.6 -> (1268,1997).
     * The real starting footprint is clear although its grid centre clips
     * yellow. Recenter X by 68mm, never the old snapped 50mm step. */
    GridNav_Init();GridNav_SetPoseMm(1268U,1997U);
    CHECK(GridNav_PlanRecenterTargetMm(1200U,400U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(segments[0].dx==-1 && segments[0].dy==0);
    CHECK(NavFrame_ReverseAligned(0.0f,1.0f,-90.0f,segments[0].dx,-segments[0].dy,3.0f));
    CHECK(NavFrame_ReverseAligned(0.0f,-1.0f,90.0f,-1.0f,0.0f,3.0f));
    CHECK(!NavFrame_ReverseAligned(0.0f,1.0f,-86.0f,-1.0f,0.0f,3.0f));
    CHECK(!NavFrame_ReverseAligned(0.0f,1.0f,-90.0f,1.0f,0.0f,3.0f));
    CHECK(!NavFrame_ReverseAligned(0.0f,1.0f,-90.0f,0.0f,-1.0f,3.0f));
    CHECK(!NavFrame_ReverseAligned(0.0f,1.0f,-90.0f,0.0f,0.0f,3.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==68U && GridNav_GetSegmentDistanceMm(1U)==1597U);
    CHECK(segments[1].dx==0 && segments[1].dy==1);
    GridNav_SetPoseMm(1279U,2008U);CHECK(GridNav_PlanRecenterTargetMm(1200U,400U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==79U && GridNav_GetSegmentDistanceMm(1U)==1608U);
    /* At ROUGH, correct the 17mm transverse drift before the 850mm leg.
     * This preserves the corner addition on the transport rather than on a
     * snapped 50mm lateral segment, as seen in the original log. */
    GridNav_SetPoseMm(1200U,383U);CHECK(GridNav_PlanRecenterTargetMm(350U,400U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==17U && GridNav_GetSegmentDistanceMm(1U)==850U);
    /* 02:10 real ROUGH pose: no corner rotation is executed when a north
     * strafe is followed by westbound transport at the same west heading. */
    GridNav_SetPoseMm(1114U,340U);
    CHECK(!GridNav_PlanRecenterTargetMm(350U,400U)); /* old false veto */
    CHECK(GridNav_PlanRecenterTargetMmHeading(350U,400U,180.0f));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==60U && GridNav_GetSegmentDistanceMm(1U)==764U);
    CHECK(!GridNav_PlanRecenterTargetMmHeading(350U,400U,135.0f)); /* actual corner turn unsafe */
    CHECK(!GridNav_PlanRecenterTargetMmHeading(350U,400U,NAN));
    GridNav_SetPoseMm(1200U,383U);CHECK(GridNav_PlanRecenterTargetMm(350U,400U));
    CHECK(GridNav_GetSegments(segments,4U,&count) && count==2U);
    CHECK(segments[0].dx==0 && segments[0].dy==-1 && segments[1].dx==-1);
    /* Station3-to-RAW and QR return keep their prescribed outer/cross lane. */
    GridNav_SetPoseMm(396U,1181U);CHECK(GridNav_PlanRecenterTargetMm(350U,2000U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==46U && GridNav_GetSegmentDistanceMm(1U)==819U);
    GridNav_SetPoseMm(396U,1181U);CHECK(GridNav_PlanRecenterTargetMm(2125U,1200U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==19U && GridNav_GetSegmentDistanceMm(1U)==1729U);
    GridNav_SetPoseMm(1268U,1997U);
    GridNav_SetObstacle(GridNav_MmToCellX(1250U),GridNav_MmToCellY(2000U),1U);
    CHECK(!GridNav_PlanRecenterTargetMm(1200U,400U));
    GridNav_Init();GridNav_SetPoseMm(350U,1200U);
    CHECK(!GridNav_PlanRecenterTargetMm(1200U,2000U)); /* not a small drift */
    CHECK(!GridNav_PlanRecenterTargetMm(0U,1201U)); /* field boundary */
    GridNav_SetPoseMm(2125U,1950U);CHECK(GridNav_PlanTargetMm(2125U,1200U));
    CHECK(GridNav_GetSegmentDistanceMm(0U)==750U); /* cleared exact overrides */
}

static void zone1_terminal_facing(void)
{
    zone2_route_t r;unsigned int i;
    const zone2_ports_t ports={zone_start,zone_cancel,zone_finish};
    const match_checkpoint_t checkpoints[7]={MATCH_QR,MATCH_RAW1,MATCH_ROUGH1,MATCH_TEMP1,
        MATCH_RAW2,MATCH_ROUGH2,MATCH_TEMP2};
    const int expected[7]={0,0,180,90,0,180,90};
    Zone2Route_Init(&r,&ports);zone_flow=&r;
    zone_sync=0U;zone_reject=0U;
    for(i=0U;i<7U;i++) {
        zone_finished=0U;
        CHECK(Zone2Route_StartFacing(&r,checkpoints[i]));
        CHECK(r.count==1U && r.actions[0].kind==ZONE2_TURN && r.actions[0].yaw==expected[i]);
        CHECK(r.actions[0].extra_mm==0U && !r.actions[0].final_point);
        CHECK(!Zone2Route_StartFacing(&r,MATCH_QR));
        Zone2Route_Service(&r,1000U+i);
        CHECK(zone_finished==0U && r.running);
        Zone2Route_Result(&r,zone_id,1U);
        CHECK(!r.running && zone_finished==1U && zone_success==1U);
    }
    CHECK(!Zone2Route_StartFacing(&r,MATCH_RETURN));
    CHECK(!Zone2Route_StartFacing(&r,(match_checkpoint_t)99));
}

static match_control_t *fixed_flow;
static zone2_route_t *fixed_route;
static uint32_t fixed_request;
static unsigned int fixed_actions,fixed_turns;
static uint8_t fixed_start(match_motion_t motion,uint8_t zone,uint32_t request)
{
    fixed_request=request;
    if(motion==MATCH_MOTION_LEAVE) {
        MatchControl_Result(fixed_flow,request,1U,tick);return MATCH_START_ACCEPTED;
    }
    return Zone2Route_StartForZone(fixed_route,motion,fixed_flow->checkpoint,zone)?
        MATCH_START_ACCEPTED:MATCH_START_REJECTED;
}
static void fixed_finish(uint8_t success)
{ MatchControl_Result(fixed_flow,fixed_request,success,tick); }
static uint8_t fixed_action_start(const zone2_action_t *a,uint32_t id)
{
    zone_action=*a;zone_id=id;fixed_actions++;
    if(a->kind==ZONE2_TURN)fixed_turns++;
    return 1U;
}
static void both_zones_fixed_flow(void)
{
    const match_ports_t match_ports={fixed_start,cancel_motion,changed};
    const zone2_ports_t route_ports={fixed_action_start,zone_cancel,fixed_finish};
    unsigned int zone,leg,i;
    for(zone=1U;zone<=2U;zone++) {
        match_control_t flow;zone2_route_t route;
        fixed_flow=&flow;fixed_route=&route;fixed_actions=fixed_turns=0U;
        tick=1000U;Zone2Route_Init(&route,&route_ports);MatchControl_Init(&flow,&match_ports);
        CHECK(MatchControl_Start(&flow,(uint8_t)zone,tick));
        MatchControl_Service(&flow,tick); /* leave synchronously schedules QR */
        CHECK(flow.state==MATCH_MOVING && flow.checkpoint==MATCH_QR);
        for(leg=0U;leg<8U;leg++) {
            tick+=1U;MatchControl_Service(&flow,tick);
            CHECK(route.running && flow.state==MATCH_MOVING);
            CHECK(route.actions[route.count-1U].kind==ZONE2_TURN);
            CHECK(route.actions[route.count-1U].yaw==
                ((leg==7U && zone==1U)?-90:
                ((leg==2U || leg==5U)?180:((leg==3U || leg==6U || leg==7U)?90:0))));
            if(leg==0U) {
                CHECK(route.actions[0].extra_mm==0);
                CHECK(Zone2Route_FixedExtraMm(flow.checkpoint)==0U);
            }
            if(leg==1U) {
                CHECK(route.actions[0].final_point);
                CHECK(route.actions[0].extra_mm+(int)Zone2Route_FixedExtraMm(flow.checkpoint)==
                    0);
            }
            if(leg==2U || leg==5U) {
                CHECK(route.count==3U && route.actions[1].final_point);
                CHECK(route.actions[1].extra_mm+(int)Zone2Route_FixedExtraMm(flow.checkpoint)==
                    0);
            }
            if(leg==3U || leg==6U)CHECK(Zone2Route_FixedExtraMm(flow.checkpoint)==0U);
            if(leg==4U)CHECK(Zone2Route_FixedExtraMm(flow.checkpoint)==0U);
            if(leg==3U || leg==6U) {
                CHECK(route.count==3U);
                CHECK(route.actions[0].kind==ZONE2_NAV && route.actions[0].x==350U && route.actions[0].y==400U);
                CHECK(route.actions[1].x==350U && route.actions[1].y==1200U);
            }
            if(leg==4U) {
                CHECK(route.count==3U && route.actions[0].x==350U && route.actions[0].y==2000U);
                CHECK(route.actions[1].x==1250U && route.actions[1].y==2100U);
            }
            if(leg==7U) {
                if(zone==1U) {
                    CHECK(route.count==5U && route.actions[0].kind==ZONE2_TURN && route.actions[0].yaw==0);
                    CHECK(route.actions[1].kind==ZONE2_NAV && route.actions[1].x==2125U && route.actions[1].y==1200U);
                    CHECK(route.actions[2].kind==ZONE2_TURN && route.actions[2].yaw==-90);
                    CHECK(route.actions[3].kind==ZONE2_NAV && route.actions[3].x==2125U && route.actions[3].y==350U);
                    CHECK(NavBlend_AngleError(route.actions[0].yaw,90.0f)==-90.0f);
                    CHECK(NavBlend_AngleError(route.actions[2].yaw,route.actions[0].yaw)==-90.0f);
                } else {
                    CHECK(route.actions[0].x==350U && route.actions[0].y==2000U);
                    CHECK(route.actions[1].x==2125U && route.actions[1].y==2000U);
                }
            }
            /* NAV completion alone cannot enter WAIT_OK: explicit TURN remains. */
            for(i=0U;i<route.count;i++) {
                tick+=1U;Zone2Route_Service(&route,tick);
                CHECK(flow.state==MATCH_MOVING);
                Zone2Route_Result(&route,zone_id,1U);
                if(i+1U<route.count)CHECK(flow.state==MATCH_MOVING && route.running);
            }
            CHECK(flow.state==(leg==7U?MATCH_FINISHED:MATCH_WAIT_OK));
            if(leg<7U){tick+=1U;CHECK(MatchControl_Ok(&flow,tick));}
        }
        CHECK(fixed_turns==(zone==1U?10U:8U) && fixed_actions==(zone==1U?24U:22U));
        CHECK(!route.running);
    }
    {
        zone2_route_t route;Zone2Route_Init(&route,&route_ports);
        CHECK(!Zone2Route_StartForZone(&route,MATCH_MOTION_QR,MATCH_QR,0U));
        CHECK(!Zone2Route_StartForZone(&route,MATCH_MOTION_QR,MATCH_QR,3U));
    }
}
static void prepare_launch_tests(void)
{
    match_control_t flow;uint32_t request;init(&flow);
    CHECK(MatchControl_Prepare(&flow,1U,100U) && MatchControl_IsActive(&flow));
    CHECK(!MatchControl_Ok(&flow,101U) && flow.checkpoint==MATCH_QR);
    MatchControl_Service(&flow,102U);request=last_request;
    CHECK(last_motion==MATCH_MOTION_PREPARE && initiated==1U && flow.state==MATCH_PREPARING);
    CHECK(!MatchControl_Prepare(&flow,1U,103U));
    MatchControl_Result(&flow,request,1U,200U);
    CHECK(flow.state==MATCH_WAIT_START_OK && MatchControl_IsActive(&flow));
    MatchControl_Service(&flow,999999U);CHECK(initiated==1U); /* waits for human, never auto-leaves */
    CHECK(!MatchControl_Ok(&flow,199U) && !MatchControl_Ok(&flow,200U));
    CHECK(MatchControl_Ok(&flow,1000000U) && flow.state==MATCH_LEAVING && flow.checkpoint==MATCH_QR);
    MatchControl_Service(&flow,1000001U);CHECK(last_motion==MATCH_MOTION_LEAVE && initiated==2U);
    init(&flow);CHECK(MatchControl_Prepare(&flow,2U,100U));MatchControl_Service(&flow,101U);
    MatchControl_Result(&flow,last_request,0U,102U);CHECK(flow.state==MATCH_FAULT);
    init(&flow);CHECK(MatchControl_Prepare(&flow,2U,100U));MatchControl_Service(&flow,101U);request=last_request;
    MatchControl_Reset(&flow);MatchControl_Result(&flow,request,1U,102U);CHECK(flow.state==MATCH_IDLE);
    CHECK(MatchControl_Prepare(&flow,2U,103U));MatchControl_Service(&flow,104U);
    MatchControl_Result(&flow,request,1U,105U);CHECK(flow.state==MATCH_PREPARING);
    MatchControl_Service(&flow,120103U);CHECK(flow.state==MATCH_FAULT);
    init(&flow);busy_start=1U;CHECK(MatchControl_Prepare(&flow,1U,100U));
    MatchControl_Service(&flow,101U);CHECK(flow.state==MATCH_PREPARING && flow.pending && !initiated);
    busy_start=0U;MatchControl_Service(&flow,102U);CHECK(flow.state==MATCH_PREPARING && !flow.pending && initiated==1U);

}
static void automatic_first_round_tests(void)
{
    const match_motion_t expected[]={MATCH_MOTION_LEAVE,MATCH_MOTION_RAW,MATCH_MOTION_ROUGH,MATCH_MOTION_TEMP};
    unsigned zone,stage;uint32_t now,request,old,calls_before;match_control_t flow;
    for(zone=1;zone<=2;zone++) {
        init(&flow);flow.automatic=flow.first_round_only=1U;now=100U;
        CHECK(MatchControl_Prepare(&flow,(uint8_t)zone,now));MatchControl_Service(&flow,++now);
        CHECK(last_motion==MATCH_MOTION_PREPARE);MatchControl_Result(&flow,last_request,1U,++now);
        CHECK(flow.state==MATCH_WAIT_START_OK && !MatchControl_Ok(&flow,now));
        calls_before=calls;MatchControl_Service(&flow,now+1000U);CHECK(calls==calls_before);
        now+=1001U;CHECK(MatchControl_Ok(&flow,now));
        for(stage=0;stage<4;stage++) {
            CHECK(flow.state==(stage?MATCH_MOVING:MATCH_LEAVING) && flow.pending);
            MatchControl_Service(&flow,++now);CHECK(last_motion==expected[stage] && last_zone==zone);
            request=flow.active_request;calls_before=calls;
            CHECK(!MatchControl_Ok(&flow,++now));MatchControl_Service(&flow,++now);CHECK(calls==calls_before);
            MatchControl_Result(&flow,request+99U,1U,++now);CHECK(flow.active_request==request);
            now+=stage?30000U:1000U;MatchControl_Result(&flow,request,1U,now);
            if(stage<3) {
                CHECK(flow.state==MATCH_MOVING && flow.pending && flow.checkpoint==(match_checkpoint_t)(stage+1U));
                old=flow.active_request;MatchControl_Result(&flow,request,1U,++now);CHECK(flow.active_request==old);
            } else CHECK(flow.state==MATCH_FINISHED && flow.checkpoint==MATCH_TEMP1 && !flow.pending && !flow.active_request);
        }
        CHECK(calls==5U && !cancellations && !MatchControl_Ok(&flow,++now));
        MatchControl_Reset(&flow);CHECK(flow.automatic && flow.first_round_only && flow.state==MATCH_IDLE);
        CHECK(MatchControl_Start(&flow,(uint8_t)zone,++now));MatchControl_Service(&flow,now);
        MatchControl_Result(&flow,last_request,1U,++now);CHECK(flow.checkpoint==MATCH_RAW1);
        busy_start=1;request=flow.active_request;MatchControl_Service(&flow,++now);
        CHECK(flow.pending && flow.active_request==request && flow.state==MATCH_MOVING);
        busy_start=0;MatchControl_Service(&flow,++now);CHECK(!flow.pending);
        MatchControl_Result(&flow,request,0U,++now);CHECK(flow.state==MATCH_FAULT && cancellations==2U);
        calls_before=calls;MatchControl_Service(&flow,++now);MatchControl_Result(&flow,request,1U,++now);
        CHECK(flow.state==MATCH_FAULT && calls==calls_before && !MatchControl_Ok(&flow,++now));
    }
}
static void run_timer_tests(void)
{
    match_control_t flow;uint32_t now,start,request;unsigned stage;
    init(&flow);flow.automatic=flow.qr_required=1U;
    CHECK(!flow.run_timer_started && !flow.run_timer_finished && !flow.run_elapsed_ms);
    CHECK(MatchControl_Prepare(&flow,1U,100U));MatchControl_Service(&flow,101U);
    MatchControl_Result(&flow,last_request,1U,200U);
    CHECK(flow.state==MATCH_WAIT_START_OK && !flow.run_timer_started);
    MatchControl_Service(&flow,999U);CHECK(!flow.run_timer_started);
    start=now=1000U;CHECK(MatchControl_Ok(&flow,now));
    CHECK(flow.run_timer_started && flow.run_started_tick==start && !flow.run_timer_finished);
    for(stage=0U;stage<9U;stage++) {
        MatchControl_Service(&flow,now);request=last_request;
        CHECK(flow.run_started_tick==start);
        CHECK(!MatchControl_Ok(&flow,now+1U));
        now+=1000U;MatchControl_Result(&flow,request,1U,now);
        CHECK(flow.milestone_mask&(1U<<stage));
        CHECK(flow.milestone_ms[stage]==(stage+1U)*1000U);
    }
    CHECK(flow.state==MATCH_FINISHED && flow.checkpoint==MATCH_RETURN);
    CHECK(flow.run_timer_finished && flow.run_elapsed_ms==9000U);
    MatchControl_Service(&flow,now+5000U);MatchControl_Result(&flow,request,1U,now+5001U);
    CHECK(flow.run_elapsed_ms==9000U && flow.run_started_tick==start);
    CHECK(flow.milestone_mask==0x1FFU && flow.milestone_ms[8]==9000U);
    MatchControl_Reset(&flow);
    CHECK(!flow.run_timer_started && !flow.run_timer_finished && !flow.run_elapsed_ms);
    CHECK(!flow.milestone_mask && !flow.milestone_ms[8]);
    /* A complete first-round trial crossing the HAL uint32 tick wrap. */
    flow.automatic=flow.first_round_only=1U;flow.qr_required=0U;
    start=now=UINT32_MAX-250U;CHECK(MatchControl_Start(&flow,2U,now));
    for(stage=0U;stage<4U;stage++) {
        MatchControl_Service(&flow,now);request=last_request;
        now+=200U;MatchControl_Result(&flow,request,1U,now);
    }
    CHECK(flow.state==MATCH_FINISHED && flow.checkpoint==MATCH_TEMP1);
    CHECK(flow.run_timer_finished && flow.run_elapsed_ms==800U && flow.run_started_tick==start);
    CHECK(flow.milestone_mask==0x1DU && !flow.milestone_ms[1]); /* QR skipped. */
    CHECK(flow.milestone_ms[0]==200U && flow.milestone_ms[2]==400U && flow.milestone_ms[4]==800U);
    MatchControl_Reset(&flow);CHECK(MatchControl_Start(&flow,1U,0U));
    CHECK(flow.run_timer_started && flow.run_started_tick==0U);
    MatchControl_Abort(&flow);
    CHECK(flow.state==MATCH_FAULT && !flow.run_timer_finished);
    MatchControl_Reset(&flow);
    CHECK(!flow.run_timer_started && !flow.run_timer_finished);
}

int main(void)
{
#ifdef TEST_MATCH_TIMING_ONLY
    run_timer_tests();return (int)test_failures;
#endif
#ifdef TEST_NAV_PARAMS_ONLY
    calibrated_defaults_survive_reference();linear_compensation_and_flash();
    return (int)test_failures;
#endif
    run_timer_tests();
    prepare_launch_tests();
    automatic_first_round_tests();
    /* Library supports full two-round automatic itinerary with a single
     * launch confirmation; success at TEMP1 must proceed to RAW2. */
    {
      const match_motion_t sequence[]={MATCH_MOTION_LEAVE,MATCH_MOTION_QR,MATCH_MOTION_RAW,MATCH_MOTION_ROUGH,MATCH_MOTION_TEMP,MATCH_MOTION_RAW,MATCH_MOTION_ROUGH,MATCH_MOTION_TEMP,MATCH_MOTION_RETURN};
      unsigned zone,stage;uint32_t now;match_control_t flow;
      for(zone=1;zone<=2;zone++){
        init(&flow);flow.automatic=1;flow.qr_required=1;flow.first_round_only=0;now=100;
        CHECK(MatchControl_Prepare(&flow,(uint8_t)zone,now));MatchControl_Service(&flow,++now);
        MatchControl_Result(&flow,last_request,1,++now);CHECK(flow.state==MATCH_WAIT_START_OK);
        CHECK(MatchControl_Ok(&flow,++now));
        for(stage=0;stage<9;stage++){
          MatchControl_Service(&flow,++now);CHECK(last_motion==sequence[stage] && !flow.pending);
          CHECK(!MatchControl_Ok(&flow,++now));now+=1000;MatchControl_Result(&flow,last_request,1,now);
          if(stage<8)CHECK(flow.state==MATCH_MOVING && flow.pending && flow.checkpoint==(match_checkpoint_t)stage);
        }
        CHECK(flow.state==MATCH_FINISHED && flow.checkpoint==MATCH_RETURN && calls==10 && !cancellations);
      }
    }
    int nl;
    complete_route(1U); complete_route(2U);
    failures_and_wrap();
    busy_dispatch_is_not_a_motion_failure();
    calibrated_defaults_survive_reference();
    linear_compensation_and_flash();
    return_direction_and_projection();
    coarse_segment_feedback();
    body_280_rotation_clearance();
    blend_math_and_geometry();
    blend_simulation(90.0f,320.0f,175.0f,0.0f,0.0f,1.0f);
    blend_simulation(-90.0f,373.0f,175.0f,0.0f,0.0f,1.0f);
    blend_simulation(180.0f,320.0f,125.0f,40.0f,0.0f,1.0f);
    blend_simulation(-180.0f,373.0f,125.0f,40.0f,0.0f,1.0f);
    blend_simulation(180.0f,373.0f,75.0f,40.0f,0.0f,1.0f);
    blend_simulation(90.0f,320.0f,200.0f,0.0f,-1.0f,0.0f); /* Actual QR -> raw first leg. */
    Test_ChassisFieldLifecycle();
    Test_PostAlignShortMoves();
    reverse_recenter_feedback();
    Test_VisionAlignment();
    Test_SmoothVisionAlignment();
    Test_AlignmentAnchors();
    Test_NavOdometry();
    Test_OdometryOwner();
    Test_ReturnDetour();
    Test_ScreenProtocol();
    Test_ScreenOwner();
    Test_QrScan();
    Test_QrApplication();
    Test_NavCorner();
    Test_NavCornerOwner();
    Test_VisionAlignmentDiagnostic();
    Test_Zone2ActionClock();
    Test_ZdtMotorQuery();
    Test_ZdtQueryRxRouting();
    zone2_routes_and_lifecycle();
    zone2_straight_routes_and_station_extras();
    qr_return_exact_geometry();
    post_alignment_lane_geometry();
    zone1_terminal_facing();
    both_zones_fixed_flow();
    for (nl = 0; nl < 2; ++nl) {
        enable_button_parser("&EN",nl);
        enable_button_parser("&en",nl);
        enable_button_parser("&enable",nl);
    parser_case("&arm material", "arm material", 0);
    parser_case("&arm material blue", "arm material blue", 0);
    parser_case("&arm material auto on", "arm material auto on", 1);
    parser_case("&arm material window 0 0 12 12", "arm material window 0 0 12 12", 1);
    parser_case("&arm material status", "arm material status", 0);
    parser_case("&arm material stop", "arm material stop", 1);
    parser_case("&arm material loop start", "arm material loop start", 1);
    parser_case("&arm material loop stop", "arm material loop stop", 0);
    parser_case("&arm material loop status", "arm material loop status", 1);
    parser_case("&arm material loop limit 2", "arm material loop limit 2", 1);
    parser_case("&arm round1 status", "arm round1 status", 1);
    parser_case("&arm ring 1", "arm ring 1", 1);
    parser_case("&arm ring 2", "arm ring 2", 0);
    parser_case("&arm ring 3", "arm ring 3", 1);
    parser_case("&arm ring", "arm ring", 0);
    parser_case("&arm ring confirm", "arm ring confirm", 1);
    parser_case("&arm ring status", "arm ring status", 1);
    parser_case("&arm place", "arm place", 0);
    parser_case("&arm place 1", "arm place 1", 0);
    parser_case("&arm place 2", "arm place 2", 0);
    parser_case("&arm place 3", "arm place 3", 0);
    parser_case("&arm place status", "arm place status", 1);
    parser_case("&arm pick", "arm pick", 0);
    parser_case("&arm pick status", "arm pick status", 1);
    parser_case("&arm grip test 186", "arm grip test 186", 0);
    parser_case("&arm grip test 224.5", "arm grip test 224.5", 1);
    parser_case("&arm grip angle 238", "arm grip angle 238", 1);
    parser_case("&vyaw on", "vyaw on", 0);
    parser_case("&vyaw off", "vyaw off", 1);
    parser_case("&vyaw status", "vyaw status", 1);
    parser_case("&outerarc status", "outerarc status", 1);
    parser_case("&outerarc off", "outerarc off", 1);
    parser_case("&outerarc radius 150", "outerarc radius 150", 1);
    parser_case("&outerarc speed 140", "outerarc speed 140", 1);
    parser_case("&vyaw gain 2 -1.25", "vyaw gain 2 -1.25", 0);
    parser_case("&vyaw view 2", "vyaw view 2", 1);
    parser_case("&vyaw zero 2", "vyaw zero 2", 1);
    parser_case("&vyaw sample 2", "vyaw sample 2", 1);
    parser_case("&vyaw solve 2", "vyaw solve 2", 1);
    parser_case("&prepare 1", "prepare 1", 0);
    parser_case("&prepare 2", "prepare 2", 1);
    parser_case("&arm camera set -43.370 -279.440 12.345", "arm camera set -43.370 -279.440 12.345", 1);
    parser_case("&arm query dm", "arm query dm", 0);
    parser_case("&arm color", "arm color", 1);
    parser_case("&arm color 6", "arm color 6", 1);
    parser_case("&arm color status", "arm color status", 1);
    parser_case("&arm color stop", "arm color stop", 1);
    parser_case("&nudge u 50", "nudge u 50", nl);
        parser_case("&nudge l 125", "nudge l 125", nl);
        parser_case("&posemm 2250 150", "posemm 2250 150", nl);
        parser_case("&gotomm 2125 450", "gotomm 2125 450", nl);
        parser_case("&depart", "depart", nl);
        parser_case("&endturn 1", "endturn 1", nl);
        parser_case("&endturn0", "endturn0", nl);
        parser_case("&ok", "ok", nl);
        parser_case("&camraw 1", "camraw 1", nl);
        parser_case("&camraw 2", "camraw 2", nl);
        parser_case("&camraw all", "camraw all", nl);
        parser_case("&camraw off", "camraw off", nl);
        parser_case("&camstat", "camstat", nl);
        parser_case("&valign on", "valign on", nl);
        parser_case("&valign off", "valign off", nl);
        parser_case("&valign status", "valign status", nl);
        parser_case("&valign smooth on", "valign smooth on", nl);
        parser_case("&valign smooth off", "valign smooth off", nl);
        parser_case("&valign search image-right", "valign search image-right", nl);
        parser_case("&valign search off", "valign search off", nl);
        parser_case("&vpid 2 1 0 0", "vpid 2 1 0 0", nl);
        parser_case("&valign trace on", "valign trace on", nl);
        parser_case("&valign trace off", "valign trace off", nl);
        parser_case("&valign trace dump", "valign trace dump", nl);
        parser_case("&valign trace status", "valign trace status", nl);
        parser_case("&valign trace clear", "valign trace clear", nl);
        parser_case("&valign trace cancel", "valign trace cancel", nl);
        parser_case("&vsource 1 1 plate 0 3", "vsource 1 1 plate 0 3", nl);
        parser_case("&dm on", "dm on", nl);
        parser_case("&dm off", "dm off", nl);
        parser_case("&dm zero", "dm zero", nl);
        parser_case("&dm zero save", "dm zero save", nl);
        parser_case("&dm zero confirm", "dm zero confirm", nl);
        parser_case("&arm zero dm save", "arm zero dm save", nl);
        parser_case("&zdt query 1", "zdt query 1", nl);
        parser_case("&zdt query 5", "zdt query 5", nl);
        parser_case("&zdt query 6", "zdt query 6", nl);
        parser_case("&dm query", "dm query", nl);
        parser_case("&dm angle", "dm angle", nl);
        parser_case("&dm home", "dm home", nl);
        parser_case("&qrscan on", "qrscan on", nl);
        parser_case("&qrscan status", "qrscan status", nl);
        parser_case("&vgain 2 .001 0 0 -.001", "vgain 2 .001 0 0 -.001", nl);
        parser_case("&vtune 3 5 .03 100 250", "vtune 3 5 .03 100 250", nl);
        parser_case("&arm top 5", "arm top 5", nl);
        parser_case("&arm guardtop 100 -20 20", "arm guardtop 100 -20 20", nl);
        parser_case("&arm zero 6", "arm zero 6", nl);
        parser_case("&dcomp 0.10", "dcomp 0.10", nl);
        parser_case("&dcomp0.08", "dcomp0.08", nl);
        parser_case("&dcomp -0.05", "dcomp -0.05", nl);
        parser_case("&MATCH STATUS", "match status", nl);
        parser_case("&match reset", "match reset", nl);
        parser_case("&startpose 1", "startpose 1", nl);
        parser_case("&startpose 2", "startpose 2", nl);
        parser_case("&startpose2", "startpose2", nl);
        parser_case("&dm resend 0", "dm resend 0", nl);
        parser_case("&dm txlog 1", "dm txlog 1", nl);
        parser_case("&dm mit 0 0 100 1 0", "dm mit 0 0 100 1 0", nl);
        parser_case("&dm pos 0.2 0.5", "dm pos 0.2 0.5", nl);
    }
    return (int)test_failures;
}
