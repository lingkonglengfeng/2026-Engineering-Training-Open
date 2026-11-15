#include "pid_storage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ips200.h"
#include "spi.h"
#include "zdt_motor.h"
#include "chassis_motor.h"
#include "nav_params.h"
#include "w25q64.h"

#define PID_STORAGE_MAGIC         0x50494431UL
#define PID_STORAGE_VERSION       1U
#define PID_PARAMETER_LIMIT       10000.0f

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float kp;
    float ki;
    float kd;
    uint32_t crc32;
} pid_storage_record_t;

static uint8_t s_display_initialized;

/* 当前生效 PID 的来源：0=编译默认，1=W25Q64（上电自动装载 / &read / 刚 &save），
 * 2=现场 RAM 手改（&kp/&ki/&kd）。
 *
 * 为什么需要这个标记：旧版本上电**不装载** W25 里的 PID，只有 &read 才装载，
 * 所以“页面里调了参数、也按了保存，一断电重启又变成 0.045/0.0035”。
 * 现场排查要能一眼看出当前值到底是哪来的，因此 &pid/&tstat 都带来源。 */
enum { PID_SOURCE_DEFAULT = 0, PID_SOURCE_W25 = 1, PID_SOURCE_MANUAL = 2 };
static uint8_t s_pid_source = PID_SOURCE_DEFAULT;

const char *PID_Storage_Source(void)
{
    return (s_pid_source == PID_SOURCE_W25) ? "W25Q64" :
           (s_pid_source == PID_SOURCE_MANUAL) ? "MANUAL" : "DEFAULT";
}

static uint32_t PID_StorageAddress(void)
{
    return W25Q64_GetCapacity() - W25Q64_SECTOR_SIZE;
}

static void PID_Print(const char *text)
{
    ZDT_Console_Print(text);
}

static uint32_t PID_Crc32(const uint8_t *data, uint32_t length)
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

static uint8_t PID_ValueValid(float value)
{
    /* The comparisons also reject NaN. */
    return (value >= -PID_PARAMETER_LIMIT) &&
           (value <= PID_PARAMETER_LIMIT);
}

static uint8_t PID_ParseValue(const char **cursor, float *value)
{
    char *end;

    *value = strtof(*cursor, &end);
    if ((end == *cursor) || !PID_ValueValid(*value)) { return 0U; }
    *cursor = end;
    return 1U;
}

static uint8_t PID_RecordValid(const pid_storage_record_t *record)
{
    uint32_t expected_crc;

    if ((record->magic != PID_STORAGE_MAGIC) ||
        (record->version != PID_STORAGE_VERSION) ||
        (record->size != sizeof(pid_storage_record_t)) ||
        !PID_ValueValid(record->kp) || !PID_ValueValid(record->ki) ||
        !PID_ValueValid(record->kd))
    {
        return 0U;
    }
    expected_crc = PID_Crc32((const uint8_t *)record,
                             sizeof(*record) - sizeof(record->crc32));
    return (expected_crc == record->crc32) ? 1U : 0U;
}

static uint8_t PID_LoadUnlocked(pid_parameters_t *parameters)
{
    pid_storage_record_t record;

    if (parameters == NULL) { return 0U; }
    if (W25Q64_Init(NULL) != W25Q64_OK) { return 0U; }
    if (W25Q64_Read(PID_StorageAddress(), (uint8_t *)&record,
                    sizeof(record)) != W25Q64_OK)
    {
        return 0U;
    }
    if (!PID_RecordValid(&record)) { return 0U; }
    parameters->kp = record.kp;
    parameters->ki = record.ki;
    parameters->kd = record.kd;
    return 1U;
}

uint8_t PID_Storage_Load(pid_parameters_t *parameters)
{uint8_t ok;if(!W25Q64_StorageTryLock())return 0U;ok=PID_LoadUnlocked(parameters);W25Q64_StorageUnlock();return ok;}
static w25q64_status_t PID_SaveUnlocked(const pid_parameters_t *parameters)
{
    pid_storage_record_t record;
    uint8_t verify_buffer[sizeof(record)];

    record.magic = PID_STORAGE_MAGIC;
    record.version = PID_STORAGE_VERSION;
    record.size = sizeof(record);
    record.kp = parameters->kp;
    record.ki = parameters->ki;
    record.kd = parameters->kd;
    record.crc32 = PID_Crc32((const uint8_t *)&record,
                             sizeof(record) - sizeof(record.crc32));

    if (W25Q64_Init(NULL) != W25Q64_OK) { return W25Q64_ERROR_ID; }
    if (W25Q64_EraseSector(PID_StorageAddress()) != W25Q64_OK)
    {
        return W25Q64_ERROR_SPI;
    }
    return W25Q64_WriteVerified(PID_StorageAddress(),
                                (const uint8_t *)&record, sizeof(record),
                                verify_buffer, sizeof(verify_buffer));
}

static w25q64_status_t PID_Save(const pid_parameters_t *parameters)
{w25q64_status_t result;if(!W25Q64_StorageTryLock())return W25Q64_ERROR_SPI;result=PID_SaveUnlocked(parameters);W25Q64_StorageUnlock();return result;}
static void PID_ShowOnDisplay(const pid_parameters_t *parameters)
{
    char line[40];

    if (s_display_initialized == 0U)
    {
        if (ips200_init(IPS200_ROTATION_LANDSCAPE) != HAL_OK)
        {
            PID_Print("[PID] ERROR: IPS200 initialization failed\r\n");
            return;
        }
        s_display_initialized = 1U;
    }
    (void)ips200_fill_screen(IPS200_COLOR_BLACK);
    (void)ips200_draw_text(20U, 20U, "PID PARAMETERS",
                           IPS200_COLOR_CYAN, IPS200_COLOR_BLACK);
    (void)snprintf(line, sizeof(line), "KP = %.3f", (double)parameters->kp);
    (void)ips200_draw_text(20U, 60U, line,
                           IPS200_COLOR_WHITE, IPS200_COLOR_BLACK);
    (void)snprintf(line, sizeof(line), "KI = %.3f", (double)parameters->ki);
    (void)ips200_draw_text(20U, 90U, line,
                           IPS200_COLOR_WHITE, IPS200_COLOR_BLACK);
    (void)snprintf(line, sizeof(line), "KD = %.3f", (double)parameters->kd);
    (void)ips200_draw_text(20U, 120U, line,
                           IPS200_COLOR_WHITE, IPS200_COLOR_BLACK);
}

static void PID_PrintFlashDiagnostics(void)
{
    w25q64_id_t id = {0U, 0U, 0U};
    w25q64_status_t id_result;
    w25q64_status_t status_result;
    uint8_t status1 = 0U;
    char line[112];

    id_result = W25Q64_ReadJedecId(&id);
    status_result = W25Q64_ReadStatus1(&status1);
    (void)snprintf(line, sizeof(line),
                   "[FLASH] JEDEC=%02X %02X %02X id_rc=%u SR1=%02X sr_rc=%u SPI_err=0x%08lX\r\n",
                   (unsigned int)id.manufacturer_id,
                   (unsigned int)id.memory_type,
                   (unsigned int)id.capacity_id,
                   (unsigned int)id_result, (unsigned int)status1,
                   (unsigned int)status_result,
                   (unsigned long)HAL_SPI_GetError(&hspi1));
    PID_Print(line);
}

static uint8_t PID_ParseSingleValue(const char *command,
                                        const char *prefix,
                                        float *value)
{
    const char *cursor = command + strlen(prefix);

    if (!PID_ParseValue(&cursor, value)) { return 0U; }
    return (*cursor == '\0') ? 1U : 0U;
}

static void PID_PrintCurrent(void)
{
    float kp;
    float ki;
    float kd;
    char line[112];

    ChassisMotor_GetTurnGains(&kp, &ki, &kd);
    (void)snprintf(line, sizeof(line),
                   "[PID] kp=%.4f ki=%.4f kd=%.4f source=%s\r\n",
                   (double)kp, (double)ki, (double)kd, PID_Storage_Source());
    PID_Print(line);
}

/* 上电装载：把 W25 里保存的 PID 装回 RAM。没有有效记录就保持编译默认值。
 * 仅由 ZDT_Motor_Begin() 在 W25 可用后调用一次；返回 1=使用了 W25 记录。
 * 现场顺序：调参 -> &save -> 断电重启 -> 本函数自动装载 -> 直接用调好的值。 */
uint8_t PID_Storage_ApplyBoot(void)
{
    pid_parameters_t parameters;
    float kp;
    float ki;
    float kd;
    char line[128];

    if (PID_Storage_Load(&parameters) != 0U)
    {
        ChassisMotor_SetTurnGains(parameters.kp, parameters.ki, parameters.kd);
        s_pid_source = PID_SOURCE_W25;
        (void)snprintf(line, sizeof(line),
                       "[PID] BOOT source=W25Q64 kp=%.4f ki=%.4f kd=%.4f (saved turn PID active)\r\n",
                       (double)parameters.kp, (double)parameters.ki, (double)parameters.kd);
        PID_Print(line);
        return 1U;
    }

    ChassisMotor_GetTurnGains(&kp, &ki, &kd);
    s_pid_source = PID_SOURCE_DEFAULT;
    (void)snprintf(line, sizeof(line),
                   "[PID] BOOT source=DEFAULT kp=%.4f ki=%.4f kd=%.4f (no valid W25 record)\r\n",
                   (double)kp, (double)ki, (double)kd);
    PID_Print(line);
    return 0U;
}

uint8_t PID_Storage_ProcessCommand(const char *command)
{
    pid_parameters_t parameters;
    w25q64_status_t status;
    char line[96];

    if (command == NULL) { return 0U; }

    if (strcmp(command, "pid") == 0)
    {
        /* &pid：查询**当前生效**（RAM）的转向 PID，供上位机“转向PID”页显示。
         * 与 &show 不同：&show 读的是 W25Q64 里保存的值，不改变运行参数。
         * 上电默认是 chassis_motor.h 里的宏；改过并 &save 之后，只有 &read
         * 才会把 Flash 值装回 RAM（当前版本不在 &EN/上电时自动装载）。 */
        PID_PrintCurrent();
        return 1U;
    }

    if ((strncmp(command, "kp", 2U) == 0) && (command[2] != '@'))
    {
        float kp;
        float ki;
        float kd;
        if (!PID_ParseSingleValue(command, "kp", &kp))
        {
            PID_Print("[PID] usage: &kp1.0\r\n");
            return 1U;
        }
        ChassisMotor_GetTurnGains(NULL, &ki, &kd);
        ChassisMotor_SetTurnGains(kp, ki, kd);
        s_pid_source = PID_SOURCE_MANUAL;
        PID_Print("[PID] kp updated; send &save to store\r\n");
        PID_PrintCurrent();
        return 1U;
    }

    if ((strncmp(command, "ki", 2U) == 0) && (command[2] != '@'))
    {
        float kp;
        float ki;
        float kd;
        if (!PID_ParseSingleValue(command, "ki", &ki))
        {
            PID_Print("[PID] usage: &ki0.01\r\n");
            return 1U;
        }
        ChassisMotor_GetTurnGains(&kp, NULL, &kd);
        ChassisMotor_SetTurnGains(kp, ki, kd);
        s_pid_source = PID_SOURCE_MANUAL;
        PID_Print("[PID] ki updated; send &save to store\r\n");
        PID_PrintCurrent();
        return 1U;
    }

    if ((strncmp(command, "kd", 2U) == 0) && (command[2] != '@'))
    {
        float kp;
        float ki;
        float kd;
        if (!PID_ParseSingleValue(command, "kd", &kd))
        {
            PID_Print("[PID] usage: &kd0.001\r\n");
            return 1U;
        }
        ChassisMotor_GetTurnGains(&kp, &ki, NULL);
        ChassisMotor_SetTurnGains(kp, ki, kd);
        s_pid_source = PID_SOURCE_MANUAL;
        PID_Print("[PID] kd updated; send &save to store\r\n");
        PID_PrintCurrent();
        return 1U;
    }

    if (strcmp(command, "save") == 0)
    {
        ChassisMotor_GetTurnGains(&parameters.kp, &parameters.ki, &parameters.kd);
        status = PID_Save(&parameters);
        if (status == W25Q64_OK)
        {
            /* 保存成功：当前 RAM 与 W25 一致，来源标成 W25；重启后由 BOOT 自动装载。 */
            s_pid_source = PID_SOURCE_W25;
            PID_Print("[PID] saved and verified; survives power cycle (auto-loaded at boot)\r\n");
            PID_PrintCurrent();
        }
        else
        {
            PID_Print("[PID] ERROR: flash save or verify failed; RAM value unchanged, NOT saved\r\n");
        }
        /* 导航行进参数（路程/速度/定时）一起存 */
        PID_Print((Nav_Params_SaveToFlash() != 0U) ?
                  "[NAV] params saved and verified\r\n" :
                  "[NAV] ERROR: flash save or verify failed\r\n");
        return 1U;
    }

    if (strcmp(command, "read") == 0)
    {
        if (PID_Storage_Load(&parameters) == 0U)
        {
            PID_Print("[PID] no valid parameters in flash\r\n");
        }
        else
        {
            ChassisMotor_SetTurnGains(parameters.kp, parameters.ki, parameters.kd);
            s_pid_source = PID_SOURCE_W25;
            PID_Print("[PID] loaded from W25Q64\r\n");
            PID_PrintCurrent();
        }
        /* 导航行进参数（路程/速度/定时）一起读回来 */
        if (Nav_Params_LoadFromFlash() != 0U)
        {
            PID_Print("[NAV] params loaded from W25Q64\r\n");
        }
        else
        {
            PID_Print("[NAV] no valid nav params in flash (using default)\r\n");
        }
        Nav_Params_Print();
        return 1U;
    }
    if (strcmp(command, "flash") == 0)
    {
        if(!W25Q64_StorageTryLock()){PID_Print("[PID] flash storage busy; retry later\r\n");return 1U;}
        PID_PrintFlashDiagnostics();
        W25Q64_StorageUnlock();
        return 1U;
    }
    if (strcmp(command, "delete") == 0)
    {
        if(!W25Q64_StorageTryLock()){PID_Print("[PID] flash storage busy; retry later\r\n");return 1U;}
        status = W25Q64_Init(NULL);
        if (status == W25Q64_OK)
        {
            status = W25Q64_EraseSector(PID_StorageAddress());
        }
        W25Q64_StorageUnlock();
        if (status == W25Q64_OK)
        {
            (void)snprintf(line, sizeof(line),
                           "[PID] sector 0x%06lX erased\r\n",
                           (unsigned long)PID_StorageAddress());
            PID_Print(line);
        }
        else
        {
            PID_Print("[PID] ERROR: sector erase failed\r\n");
        }
        return 1U;
    }
    if (strcmp(command, "show") == 0)
    {
        if (PID_Storage_Load(&parameters) == 0U)
        {
            PID_Print("[PID] no valid parameters in flash\r\n");
            return 1U;
        }
        (void)snprintf(line, sizeof(line),
                       "[PID] kp=%.3f ki=%.3f kd=%.3f\r\n",
                       (double)parameters.kp, (double)parameters.ki,
                       (double)parameters.kd);
        PID_Print(line);
        PID_ShowOnDisplay(&parameters);
        return 1U;
    }
    return 0U;
}
