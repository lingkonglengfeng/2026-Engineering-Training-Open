#include "nav_params.h"
#include "nav_odometry.h"

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_motor.h"
#include "zdt_motor.h"
#include "w25q64.h"

#define NAV_STORAGE_MAGIC      0x4E415632UL   /* "NAV2"：参数结构变了，换 magic */
#define NAV_STORAGE_VERSION    11U  /* Same layout; migrate old v10 default .75 to .65 once. */

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    nav_parameters_t parameters;
    uint32_t crc32;
} nav_storage_record_t;

/* v4 has all current fields up to (but excluding) distance_compensation. */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t parameters[offsetof(nav_parameters_t, distance_compensation)];
    uint32_t crc32;
} nav_storage_v4_t;

/* 2026-10-03 bench calibration. Theory remains the k denominator;
 * it must not overwrite the calibrated factory t1000 at boot/reset. */
#define NAV_SPEED_DEFAULT       0.650f
#define NAV_T1000_DEFAULT_MS     1930.0f
#define NAV_DISTANCE_COMP_DEFAULT 0.0f
#define NAV_DISTANCE_COMP_MIN    (-0.30f)
#define NAV_DISTANCE_COMP_MAX    0.30f

/* 参考时间的合理区间（毫秒），与 `t1000` 的可调范围一致：一个 2400 mm 场地上
 * "走 1000 mm 需要多久"不可能落在 50~20000 ms 之外。
 * 这同时是一道**单位防呆**：曾经把 ZDT_Nav_PlanSeconds() 的返回值（**秒**，2050.8）
 * 当毫秒直接传进来，于是 k 打印成 1023.964、理论值打印成 2ms，而且 `navreset` /
 * flash 记录失效时会把 t1000 设成 2.05 ms（等于车一步不走）。宁可拒绝并报错。 */
#define NAV_REF_PLAN_MIN_MS     50.0f
#define NAV_REF_PLAN_MAX_MS     20000.0f

static float s_nav_ref_plan_ms = 0.0f;   /* spd=0.50 走 1000 mm 的理论毫秒数 */

/* 当前场地标定：90°=1700ms，180°=2800ms，用于A*转向代价。
   现场用 &t90<ms> / &t180<ms> 改，改完 &save。 */
#define NAV_TURN90_DEFAULT_MS   1700.0f
#define NAV_TURN180_DEFAULT_MS  2800.0f

/* 90° 圆弧过弯（2026-09-13 现场要求"设置个参数：90° 圆弧过弯需要的时间，时间越短
   半径越小，以及速度"）。
   默认 **0 = 关闭** —— 出厂行为与以前一模一样，现场发 `&tarc 1000` 才启用。
   为什么默认关：圆弧要求车体在转角处整体旋转，扫过的是 约198 mm半径的外接圆（原地转也是），
   而且外轮转速要 v+300*(v/R) —— 这两件事都必须在场上验证过才敢设为默认。 */
#define NAV_ARC90_DEFAULT_MS    0.0f
#define NAV_ARC_SPD_DEFAULT     1.00f

static nav_parameters_t s_nav_params =
{
    NAV_SPEED_DEFAULT,     /* speed_scale */
    NAV_T1000_DEFAULT_MS,  /* time_1000mm_ms */
    65536.0f,               /* counts_per_rev（0x36 位置单位，厂商 Emm 口径） */
    0U,                     /* position_mode */
    NAV_TURN90_DEFAULT_MS,  /* turn90_ms */
    NAV_TURN180_DEFAULT_MS, /* turn180_ms */
    NAV_ARC90_DEFAULT_MS,   /* arc90_ms */
    NAV_ARC_SPD_DEFAULT,    /* arc_spd_ratio */
    NAV_DISTANCE_COMP_DEFAULT
};

nav_parameters_t *Nav_Params(void)
{
    return &s_nav_params;
}

float Nav_Params_StraightDistance(float nominal_mm)
{
    return nominal_mm * NAV_STRAIGHT_TARGET_SCALE;
}

float Nav_Params_CompensatedDistance(float nominal_mm)
{
    return nominal_mm * (1.0f + s_nav_params.distance_compensation);
}

static void Nav_Print(const char *text)
{
    ZDT_Console_Print(text);
}

static float Nav_T1000DefaultMs(void)
{
    return NAV_T1000_DEFAULT_MS;
}

void Nav_Params_SetReferencePlanMs(float reference_ms)
{
    /* 越界就拒绝，并且**打出来**：单位差了 1000 倍这种错误如果静默接受，现场
       只会看到"距离不对/车不动"这种远症状，根本查不到这里。 */
    if ((reference_ms < NAV_REF_PLAN_MIN_MS) || (reference_ms > NAV_REF_PLAN_MAX_MS))
    {
        char line[120];

        (void)snprintf(line, sizeof(line),
                       "[NAV] WARNING: reference plan %.3fms is out of range"
                       " (%.0f~%.0f); kept t1000=%.0fms\r\n",
                       (double)reference_ms,
                       (double)NAV_REF_PLAN_MIN_MS, (double)NAV_REF_PLAN_MAX_MS,
                       (double)s_nav_params.time_1000mm_ms);
        Nav_Print(line);
        return;
    }

    s_nav_ref_plan_ms = reference_ms;
    /* Keep calibrated/runtime values. Only telemetry uses this reference. */
}

void Nav_Params_Reset(void)
{
    s_nav_params.speed_scale = NAV_SPEED_DEFAULT;
    s_nav_params.time_1000mm_ms = Nav_T1000DefaultMs();
    s_nav_params.counts_per_rev = 65536.0f;
    s_nav_params.position_mode = 0U;
    s_nav_params.turn90_ms = NAV_TURN90_DEFAULT_MS;
    s_nav_params.turn180_ms = NAV_TURN180_DEFAULT_MS;
    s_nav_params.arc90_ms = NAV_ARC90_DEFAULT_MS;
    s_nav_params.arc_spd_ratio = NAV_ARC_SPD_DEFAULT;
    s_nav_params.distance_compensation = NAV_DISTANCE_COMP_DEFAULT;
}

void Nav_Params_Print(void)
{
    char line[176];
    double rpm = (double)s_nav_params.speed_scale * (double)CHASSIS_MAX_WHEEL_RPM;
    double mm_per_s = (rpm / 60.0) * (double)ZDT_MILEAGE_MM_PER_REV;

    (void)snprintf(line,sizeof(line),"[NAV] V177 straight target x%.3f; encoder units unchanged; outer tangent legs included\r\n",
                   (double)NAV_STRAIGHT_TARGET_SCALE);
    Nav_Print(line);

    /* 这里不要再按"1000 mm / t1000"折算一个假速度出来：换梯形规划器之后 t1000
       只是时间微调系数，那个除法会算出与 spd 矛盾的速度（曾经打印出 spd=0.200
       的 213 mm/s 和 t1000 推算的 476 mm/s 并列，看着像参数打架）。改成打印
       偏离理论的系数 k，一眼能看出标定还准不准。 */
    if (s_nav_ref_plan_ms > 1.0f)
    {
        (void)snprintf(line, sizeof(line),
                       "[NAV] spd=%.3f (%.0f RPM, %.0f mm/s)  t1000=%.0fms"
                       "  k=%.3f (theory%.0fms)  cnt=%.0f/rev\r\n",
                       (double)s_nav_params.speed_scale, rpm, mm_per_s,
                       (double)s_nav_params.time_1000mm_ms,
                       (double)s_nav_params.time_1000mm_ms /
                           (double)s_nav_ref_plan_ms,
                       (double)s_nav_ref_plan_ms,
                       (double)s_nav_params.counts_per_rev);
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "[NAV] spd=%.3f (%.0f RPM, %.0f mm/s)  t1000=%.0fms"
                       "  cnt=%.0f/rev\r\n",
                       (double)s_nav_params.speed_scale, rpm, mm_per_s,
                       (double)s_nav_params.time_1000mm_ms,
                       (double)s_nav_params.counts_per_rev);
    }
    Nav_Print(line);
    (void)snprintf(line, sizeof(line),
                   NAV_ODOMETRY_ENABLED?"[NAV] dcomp=%.3f ignored in ODOMETRY mode; nominal 1000mm -> 1000mm (legacy %.0fmm)\r\n":
                   "[NAV] dcomp=%.3f: timed translation D*(1+dcomp); 1000mm -> %.0fmm; no end pulses\r\n",
                   (double)s_nav_params.distance_compensation,
                   (double)Nav_Params_CompensatedDistance(1000.0f));
    Nav_Print(line);
    (void)snprintf(line, sizeof(line),
                   "[NAV] turn cost: t90=%.0fms  t180=%.0fms (used by A* to choose"
                   " routes)\r\n",
                   (double)s_nav_params.turn90_ms,
                   (double)s_nav_params.turn180_ms);
    Nav_Print(line);
    /* 圆弧过弯把"时间旋钮"换算成"半径 + 外轮转速"打出来：现场据此判断
       `&tarc` 调到多少是几何/轮速能接受的（R 太小外轮会先撞 250 RPM 上限）。 */
    if (s_nav_params.arc90_ms > 0.0f)
    {
        double v_arc = (double)s_nav_params.speed_scale *
                       (double)s_nav_params.arc_spd_ratio *
                       (double)CHASSIS_MAX_WHEEL_RPM *
                       ((double)ZDT_MILEAGE_MM_PER_REV / 60.0);
        double radius = (2.0 * v_arc *
                         ((double)s_nav_params.arc90_ms / 1000.0)) /
                        3.14159265358979323846;
        double wheel_rpm = (radius > 1.0) ?
            (v_arc * (1.0 + ((double)CHASSIS_SPIN_ARM_MM / radius)) /
             ((double)ZDT_MILEAGE_MM_PER_REV / 60.0)) : 0.0;

        (void)snprintf(line, sizeof(line),
                       "[NAV] arc: tarc=%.0fms aspd=%.2f -> v=%.0f mm/s R=%.0f mm"
                       " outer %.0f RPM (cap %.0f)\r\n",
                       (double)s_nav_params.arc90_ms,
                       (double)s_nav_params.arc_spd_ratio, v_arc, radius,
                       wheel_rpm, (double)CHASSIS_MAX_WHEEL_RPM);
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "[NAV] arc: OFF (tarc=0) -> every 90 deg corner stops and turns"
                       " in place; `tarc1000` to try arc cornering\r\n");
    }
    Nav_Print(line);
    (void)snprintf(line, sizeof(line),
                   "[NAV] mode=%s  defaults: spd%.3f / t1000%.0f / cnt65536"
                   " / navreset, then `save`\r\n",
                   NAV_ODOMETRY_ENABLED?"VELOCITY+ODOMETRY":(s_nav_params.position_mode != 0U) ? "POSITION(0xFD)"
                                                      : "VELOCITY+TIME",
                   (double)NAV_SPEED_DEFAULT, (double)Nav_T1000DefaultMs());
    Nav_Print(line);
}

/* ---------- flash 存储（独立扇区，与 PID 那份互不影响） ---------- */

static uint32_t Nav_StorageAddress(void)
{
    uint32_t capacity = W25Q64_GetCapacity();

    /* PID 用最后一个扇区，导航参数用倒数第二个。 */
    if (capacity > W25Q64_SECTOR_SIZE)
    {
        return capacity - (2U * W25Q64_SECTOR_SIZE);
    }
    return capacity - W25Q64_SECTOR_SIZE;
}

static uint32_t Nav_Crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return ~crc;
}

static uint8_t Nav_RecordValid(const nav_storage_record_t *record)
{
    uint32_t expected_crc;

    if ((record->magic != NAV_STORAGE_MAGIC) ||
        ((record->version != NAV_STORAGE_VERSION) && (record->version != 10U) && (record->version != 9U) && (record->version != 8U) && (record->version != 7U) && (record->version != 6U) && (record->version != 5U)) ||
        (record->size != sizeof(nav_storage_record_t)))
    {
        return 0U;
    }
    expected_crc = Nav_Crc32((const uint8_t *)record,
                             sizeof(*record) - sizeof(record->crc32));
    return ((expected_crc == record->crc32) &&
            (record->parameters.distance_compensation >= NAV_DISTANCE_COMP_MIN) &&
            (record->parameters.distance_compensation <= NAV_DISTANCE_COMP_MAX)) ? 1U : 0U;
}

static uint8_t Nav_SaveUnlocked(void)
{
    nav_storage_record_t record = {0};
    uint8_t verify_buffer[sizeof(record)];

    record.magic = NAV_STORAGE_MAGIC;
    record.version = NAV_STORAGE_VERSION;
    record.size = sizeof(record);
    record.parameters = s_nav_params;
    record.crc32 = Nav_Crc32((const uint8_t *)&record,
                             sizeof(record) - sizeof(record.crc32));

    if (W25Q64_Init(NULL) != W25Q64_OK)
    {
        return 0U;
    }
    if (W25Q64_EraseSector(Nav_StorageAddress()) != W25Q64_OK)
    {
        return 0U;
    }
    return (W25Q64_WriteVerified(Nav_StorageAddress(),
                                 (const uint8_t *)&record, sizeof(record),
                                 verify_buffer, sizeof(verify_buffer))
            == W25Q64_OK) ? 1U : 0U;
}

uint8_t Nav_Params_SaveToFlash(void)
{uint8_t ok;if(!W25Q64_StorageTryLock())return 0U;ok=Nav_SaveUnlocked();W25Q64_StorageUnlock();return ok;}
static void Nav_UpgradeLegacyTurnDefaults(void)
{
    /* Replace the prior shipped pair after loading, retaining custom values
     * and every other saved parameter. No automatic flash erase/write. */
    if(s_nav_params.turn90_ms==1000.0f && s_nav_params.turn180_ms==2000.0f) {
        s_nav_params.turn90_ms=NAV_TURN90_DEFAULT_MS;s_nav_params.turn180_ms=NAV_TURN180_DEFAULT_MS;
        Nav_Print("[NAV] old turn-cost defaults upgraded: t90=1700ms t180=2800ms; save to persist\r\n");
    }
}
static uint8_t Nav_LoadUnlocked(void)
{
    nav_storage_record_t record;

    if (W25Q64_Init(NULL) != W25Q64_OK)
    {
        return 0U;
    }
    if (W25Q64_Read(Nav_StorageAddress(), (uint8_t *)&record,
                    sizeof(record)) != W25Q64_OK)
    {
        return 0U;
    }
    if (Nav_RecordValid(&record) == 0U)
    {
        if ((record.magic == NAV_STORAGE_MAGIC) && (record.version == 4U))
        {
            nav_storage_v4_t legacy;
            memcpy(&legacy, &record, sizeof(legacy));
            if ((legacy.size == sizeof(legacy)) &&
                (legacy.crc32 == Nav_Crc32((const uint8_t *)&legacy,
                                           offsetof(nav_storage_v4_t, crc32))))
            {
                memcpy(&s_nav_params, legacy.parameters, sizeof(legacy.parameters));
                s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
                s_nav_params.distance_compensation = NAV_DISTANCE_COMP_DEFAULT;
                Nav_UpgradeLegacyTurnDefaults();
                Nav_Print("[NAV] migrated flash v4; spd=0.650, kept timing/counts, dcomp=0.000; save to store v11\r\n");
                return 1U;
            }
        }
        /* 旧版本记录（v2 及以前没有 t90/t180 字段）：**明确报出来**，
           不要让人以为"我保存过的 spd/t1000/cnt 还在"。 */
        if ((record.magic == NAV_STORAGE_MAGIC) &&
            (record.version < NAV_STORAGE_VERSION))
        {
            char line[128];

            (void)snprintf(line, sizeof(line),
                           "[NAV] flash record v%u obsolete (expected v%u);"
                           " keeping current/default params. Set and save again.\r\n",
                           (unsigned int)record.version,
                           (unsigned int)NAV_STORAGE_VERSION);
            Nav_Print(line);
        }
        return 0U;
    }
    s_nav_params = record.parameters;
    if(record.version==5U) {
        s_nav_params.distance_compensation=0.0f;
    }
    if(record.version<7U) {
        s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
        Nav_Print("[NAV] migrated flash v5/v6; spd=0.650, kept timing/counts; save to store v11\r\n");
    }
    if(record.version<8U) {
        if(s_nav_params.speed_scale==0.350f) {
            s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
            Nav_Print("[NAV] old speed default upgraded: spd=0.650; save to store v11\r\n");
        }
        Nav_UpgradeLegacyTurnDefaults();
    }
    if(record.version<9U && s_nav_params.speed_scale==0.400f) {
        s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
        Nav_Print("[NAV] old0.400 default upgraded to spd=0.650; timing/counts retained; save to store v11\r\n");
    }
    if(record.version<10U && s_nav_params.speed_scale==0.500f) {
        s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
        Nav_Print("[NAV] V182 old0.500 default upgraded to spd=0.650; timing/counts retained; save to store v11\r\n");
    }
    if(record.version<11U && s_nav_params.speed_scale==0.750f) {
        s_nav_params.speed_scale=NAV_SPEED_DEFAULT;
        Nav_Print("[NAV] V184 old0.750 default upgraded to spd=0.650; timing/counts retained; save to store v11\r\n");
    }
    return 1U;
}

/* ---------- 命令解析 ---------- */

static uint8_t Nav_ApplyFloat(float *target, const char *command,
                              const char *prefix, float minimum, float maximum)
{
    const char *cursor = command + strlen(prefix);
    char *end;
    float value = strtof(cursor, &end);

    if ((end == cursor) || (*end != '\0'))
    {
        char line[64];

        (void)snprintf(line, sizeof(line),
                       "[NAV] usage: %.16s<value>\r\n", prefix);
        Nav_Print(line);
        return 1U;
    }
    if ((value != value) || (value < minimum) || (value > maximum))
    {
        char line[96];

        (void)snprintf(line, sizeof(line),
                       "[NAV] out of range (%.3f ~ %.3f)\r\n",
                       (double)minimum, (double)maximum);
        Nav_Print(line);
        return 1U;
    }
    *target = value;
    Nav_Print("[NAV] updated; send `save` to store\r\n");
    Nav_Params_Print();
    return 1U;
}

uint8_t Nav_Params_ProcessCommand(const char *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    if (strcmp(command, "nav") == 0)
    {
        Nav_Params_Print();
        return 1U;
    }
    if (strcmp(command, "navreset") == 0)
    {
        Nav_Params_Reset();
        Nav_Print("[NAV] parameters reset to default\r\n");
        Nav_Params_Print();
        return 1U;
    }
    if (strncmp(command, "spd", 3U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.speed_scale, command, "spd",
                              0.02f, 1.00f);
    }
    if (strncmp(command, "t1000", 5U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.time_1000mm_ms, command, "t1000",
                              50.0f, 20000.0f);
    }
    if (strncmp(command, "dcomp", 5U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.distance_compensation, command, "dcomp",
                              NAV_DISTANCE_COMP_MIN, NAV_DISTANCE_COMP_MAX);
    }
    if (strncmp(command, "cnt", 3U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.counts_per_rev, command, "cnt",
                              100.0f, 1000000.0f);
    }
    /* 转向耗时（给 A* 代价模型用）：现场实测后填，改完 `save` 存 flash。
       范围 200~10000 ms —— 比这更小/更大基本是填错了（会算出很离谱的绕路策略）。 */
    if (strncmp(command, "t90", 3U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.turn90_ms, command, "t90",
                              200.0f, 10000.0f);
    }
    if (strncmp(command, "t180", 4U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.turn180_ms, command, "t180",
                              200.0f, 10000.0f);
    }
    /* 90° 圆弧过弯：**0 = 关闭**（默认），否则填"希望这个圆弧走多久"的毫秒数。
       T 越小半径越小（R = 2*v*T/pi），R 小到一定程度外轮会先到 250 RPM 上限
       （按 v=213 mm/s 算，T<600 ms 就开始被限速，实际反而更慢），所以下限给 200 ms
       只是防呆，真正有意义的区间是 600~2000 ms。 */
    if (strncmp(command, "tarc", 4U) == 0)
    {
        const char *cursor = command + 4U;
        char *end;
        float value = strtof(cursor, &end);

        if ((end == cursor) || (*end != '\0') ||
            ((value != 0.0f) && ((value < 200.0f) || (value > 10000.0f))))
        {
            Nav_Print("[NAV] usage: tarc<ms>, 0 = off, or 200~10000 (short = tight)\r\n");
            return 1U;
        }
        s_nav_params.arc90_ms = value;
        Nav_Print("[NAV] updated; send `save` to store\r\n");
        Nav_Params_Print();
        return 1U;
    }
    /* 圆弧阶段的车体速度 = 当前 spd × 本比例（1.00 = 与直走同速）。
       现场若航向跟不住（陀螺残差大 / 打滑），先降这个，而不是降 tarc。 */
    if (strncmp(command, "aspd", 4U) == 0)
    {
        return Nav_ApplyFloat(&s_nav_params.arc_spd_ratio, command, "aspd",
                              0.10f, 1.00f);
    }
    return 0U;
}

uint8_t Nav_Params_LoadFromFlash(void)
{uint8_t ok;if(!W25Q64_StorageTryLock())return 0U;ok=Nav_LoadUnlocked();W25Q64_StorageUnlock();return ok;}
