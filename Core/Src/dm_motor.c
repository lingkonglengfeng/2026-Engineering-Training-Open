/**
  ******************************************************************************
  * @file    dm_motor.c
  * @brief   达妙 DM 关节电机驱动实现（本车：机械臂转盘水平旋转轴）
  *
  *          总线：FDCAN1，PA11 RX / PA12 TX，经典 CAN，1 Mbps，
  *          消息 RAM 偏移 128 word，TDC/过滤器在 DM_Motor_Init 里落位。
  *
  *          本次（2026-09-14）相对旧版的修改，逐条对应审计发现的问题：
  *
  *          [修] 入参被丢弃（旧版把 esc_id/mst_id/mode 全部 (void) 掉再硬编码
  *               0x01/0x11/POS，而 main.c 传的是 0x05/0x00/MIT —— 三处说法互相矛盾）。
  *               现在配置真正生效，默认值走 DM_ESC_ID_DEFAULT / DM_MST_ID_DEFAULT。
  *
  *          [修] **DLC 编码错误（旧版最致命的一处）**：旧版写
  *               `tx.DataLength = (len <= 8) ? dlc_lut[len] : ...`，而 HAL 在经典 CAN
  *               分支里执行 `hfdcan->Instance->TXBCR = ... | ((DataLength & 0xFU) << 16)`，
  *               所以 DataLength 必须是 `FDCAN_DLC_BYTES_n`（= n << 16）。
  *               旧版对 FD 分支给了正确的预移位常量、对经典分支给了裸字节数 n，
  *               两条分支的语义正好相反 —— 经典 CAN 下**任何一帧的 DLC 都是错的**
  *               （len=8 → DLC 字段 0 → DLC=0，len=4 → DLC 字段 4 → DLC=4，
  *                仅当 len 恰好等于 4 或 8 时"看起来能发"）。台架现象是 0x101 时好时坏。
  *               现在统一用 DM_ByteToDlc()，所有长度都编码正确。
  *
  *          [修] 反馈位置被当成**有符号**解码（`(int16_t)(...)`）→ 任何原始值 ≥ 32768
  *               （约 +0.001 rad 以上）都会被解成大负数。达妙规范里 16 位位置是
  *               无符号、以中点为 0（官方例程 `p_int = (rx[1]<<8)|rx[2];` 也是无符号）。
  *               现在按无符号解码。
  *
  *          [修] 每发一帧都阻塞打印 128 字符到 9600 baud 的 UART4（≈133 ms，
  *               超过 100 ms 超时）→ 会把 1 ms 任务循环直接拖死。
  *               现在走 ZDT_Console_Print()：**中断里入队、任务里 flush**，
  *               且默认只打印错误/状态变化（DM_LOG_ALL_TX 打开才逐帧打印）。
  *
  *          [修] RX 缓冲区只有 8 字节，而 RX FIFO 元素尺寸被设成 16 字节 →
  *               一旦切到 CAN FD（>8 字节帧）就是栈溢出。现在缓冲区 64 字节且按
  *               DLC 夹紧拷贝长度。
  *
  *          [新] 失去控制即松劲：使能状态下超过 link_timeout_ms 收不到反馈 →
  *               立即 DM_Motor_Disable() 并打印一条（不刷屏）。符合用户要求
  *               "失去控制直接松，防止出现问题"。
  *
  *          [修] `dm probe` 被动观察：不发送控制帧，保留正常服务与接收中断。
  *               报告位时序、反馈增量与 TEC/REC；无回包时不推断物理链路故障。
  *
  *          [新] 反馈帧里第 7 字节（线圈温度）以前被直接丢掉，现在解析进
  *               fb.t_coil_c（老固件不支持时 temp_valid = 0）。
  ******************************************************************************
  */

#include "dm_motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "fdcan.h"
#include "usart.h"
#include "zdt_motor.h"

/* FDCAN1 diagnostic history: PSR reads reset LEC to 7, including BO polling.
 * Keep observed real errors until this bus is reconfigured/recovered. */
static volatile uint32_t dm_lec_seen;
static volatile uint32_t dm_lec_last;

static void DM_RecordLec(uint32_t lec)
{
    if ((lec > 0U) && (lec < 7U))
    {
        dm_lec_seen |= 1UL << lec;
        dm_lec_last = lec;
    }
}

static uint32_t DM_ReadPsr(FDCAN_HandleTypeDef *can)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t psr;
    __disable_irq();
    psr = can->Instance->PSR;
    DM_RecordLec(psr & FDCAN_PSR_LEC);
    __set_PRIMASK(primask);
    return psr;
}

static HAL_StatusTypeDef DM_GetProtocolStatus(FDCAN_HandleTypeDef *can,
                                             FDCAN_ProtocolStatusTypeDef *protocol)
{
    uint32_t primask = __get_PRIMASK();
    HAL_StatusTypeDef status;
    __disable_irq();
    status = HAL_FDCAN_GetProtocolStatus(can, protocol);
    if (status == HAL_OK) DM_RecordLec(protocol->LastErrorCode);
    __set_PRIMASK(primask);
    return status;
}

/* ================= 编译期开关 ================= */

/* 1 = 每一帧控制帧都打印到调试串口（只在台架短时调试时打开，
 *     9600 baud 下每帧约 30 字符 ≈ 30 ms，会明显拖慢任务）。 */
#ifndef DM_LOG_ALL_TX
#define DM_LOG_ALL_TX               (0U)
#endif

/* 1 = 编译 PSI（位置-速度-电流）混控帧。该模式是 12 字节 FD 帧，
 *     必须总线切到 CAN FD + 电机数据域波特率同步之后才可用。
 *     本车台架实测是经典 CAN 1 Mbps，所以默认 0（关闭时 dm mit 之外的模式不可用）。 */
#ifndef DM_BUS_USE_FD_PSI
#define DM_BUS_USE_FD_PSI           (0U)
#endif

/* Enable is a bounded task-context handshake.  The disable frame is a safe
 * position query on this motor; after a fresh legal reply, the same raw
 * position is held at a small non-zero speed. */
#define DM_ENABLE_QUERY_TIMEOUT_MS   (300U)
#define DM_ENABLE_CONFIRM_TIMEOUT_MS (1000U)
#define DM_ENABLE_HOLD_QUERY_MS      (20U)
#define DM_ENABLE_HOLD_VEL_RAD       (0.10f)

/* 消息 RAM 偏移：zdt 的 FDCAN2 占用 word 0~127（详见 fdcan.c），本模块从 128 起 */
#define DM_RAM_OFFSET               (128U)
#define DM_RX_FIFO_ELMTS            (8U)
#define DM_RX_ELMT_SIZE             FDCAN_DATA_BYTES_16
#define DM_TX_FIFO_ELMTS            (4U)
#define DM_TX_ELMT_SIZE             FDCAN_DATA_BYTES_16

/* 单帧最大长度：为将来切 CAN FD 留出 64 字节余量 */
#define DM_FRAME_MAX_BYTES          (64U)

/* 反馈帧固定 8 字节、状态码 1 = 使能 */
#define DM_FB_STATE_DISABLED        (0U)
#define DM_FB_STATE_ENABLED         (1U)
#define DM_FB_LENGTH                (8U)

/* ================= 内部工具 ================= */

/* 字节数 → HAL 的 DLC 编码。
 *
 * HAL 在经典/FD 两条分支里都是 `(DataLength & 0xFU) << 16`，所以这里必须返回
 * FDCAN_DLC_BYTES_n（n << 16），**不能**返回裸字节数 n。
 * 见 Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c 的
 * HAL_FDCAN_AddMessageToTxFifoQ()。 */
static uint32_t DM_ByteToDlc(uint8_t bytes)
{
    static const uint32_t lut[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };

    switch (bytes)
    {
        case 12U: return FDCAN_DLC_BYTES_12;
        case 16U: return FDCAN_DLC_BYTES_16;
        case 20U: return FDCAN_DLC_BYTES_20;
        case 24U: return FDCAN_DLC_BYTES_24;
        case 32U: return FDCAN_DLC_BYTES_32;
        case 48U: return FDCAN_DLC_BYTES_48;
        case 64U: return FDCAN_DLC_BYTES_64;
        default:
            return (bytes <= 8U) ? lut[bytes] : FDCAN_DLC_BYTES_8;
    }
}

/* 调试串口：ZDT 的打印器自己判上下文（中断入队 / 任务直发），
 * 所以这里在 ISR 和任务里都能安全调用，且绝不会在中断里阻塞。 */
static void DM_Print(const char *text)
{
    ZDT_Console_Print(text);
}

/* 命令解析：取下一个以空格分隔的 token，返回指向 token 起始的指针，
 * 并把 token 后的分隔符改成 '\0'；没有更多 token 时返回 NULL。 */
static char *DM_NextToken(char **cursor)
{
    char *p = *cursor;
    char *start;

    if (p == NULL)
    {
        return NULL;
    }
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    if (*p == '\0')
    {
        *cursor = NULL;
        return NULL;
    }
    start = p;
    while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
    {
        p++;
    }
    if (*p != '\0')
    {
        *p = '\0';
        p++;
    }
    *cursor = p;
    return start;
}

/* 严格解析一个 float：必须整段消费且非空。成功返回 1。 */
static uint8_t DM_ParseFloat(const char *text, float *out)
{
    char *end = NULL;
    float value;

    if ((text == NULL) || (out == NULL) || (*text == '\0'))
    {
        return 0U;
    }
    value = strtof(text, &end);
    if ((end == text) || (*end != '\0'))
    {
        return 0U;
    }
    *out = value;
    return 1U;
}

/* 限幅（用不着 math.h） */
static float DM_Clamp(float value, float limit)
{
    if (value > limit)  { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

/* float <-> uint 线性映射（达妙协议） */
static int DM_FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;

    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    if (span == 0.0f) { return 0; }
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

/* 注意：x 必须是**无符号**含义的值。位置字段 16 位无符号映射，
 * 传 (int16_t) 进来会把正半轴整段解成负数（旧版 bug）。 */
static float DM_UintToFloat(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;

    if (span == 0.0f) { return x_min; }
    return ((float)x) * span / ((float)((1 << bits) - 1)) + x_min;
}

/* ================= 发送 ================= */

static HAL_StatusTypeDef DM_SendFrame(dm_motor_t *motor, uint16_t id,
                                      const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx;
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (motor->cfg.can == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }
    if (len > DM_FRAME_MAX_BYTES)
    {
        len = DM_FRAME_MAX_BYTES;
    }

    (void)memset(&tx, 0, sizeof(tx));
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = DM_ByteToDlc(len);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    /* 本车电机是经典 CAN：不置 FDF/BRS。
     * （参考工程的 fdcanx_send_data() 无条件置 FDCAN_FD_CAN + BRS_ON，
     *   在 CCCR.FDOE = 0 时那两位是 don't-care 才碰巧能跑；这里不照抄该写法。） */
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    status = HAL_FDCAN_AddMessageToTxFifoQ(motor->cfg.can, &tx, (uint8_t *)data);
    motor->tx_count++;
    if (status != HAL_OK)
    {
        motor->tx_fail_count++;
    }
    motor->last_err = (uint8_t)status;

#if (DM_LOG_ALL_TX != 0U)
    {
        char line[128];
        int used;

        used = snprintf(line, sizeof(line), "[DM] TX 0x%03lX len=%u HAL=%d\r\n",
                        (unsigned long)id, (unsigned int)len, (int)status);
        if (used > 0)
        {
            DM_Print(line);
        }
    }
#else
    /* 运行期逐帧打印（`dm txlog 1`）：用来肉眼确认"到底发出去了什么"。
     * 编译期 DM_LOG_ALL_TX 是"一直打"，这个是"需要时才打"。 */
    if ((motor->tx_log != 0U) && (status == HAL_OK))
    {
        char line[128];
        int used = snprintf(line, sizeof(line), "[DM] TX 0x%03lX len=%u DATA=",
                            (unsigned long)id, (unsigned int)len);
        uint32_t index;

        for (index = 0U; (index < len) && (used > 0) &&
                         ((size_t)used < sizeof(line) - 4U); ++index)
        {
            used += snprintf(&line[used], sizeof(line) - (size_t)used,
                             "%02X ", data[index]);
        }
        if (used > 0)
        {
            (void)snprintf(&line[used], sizeof(line) - (size_t)used, "\r\n");
            DM_Print(line);
        }
    }

    /* TX 错误日志**必须限流**（2026-09-14 台架踩过）：
     * 对端不应答时每一帧都会失败，而重发是 100 Hz —— 逐帧打印会在 9600 baud 下
     * 把串口占满，任务被阻塞，现场表现就是"什么指令都没反应"。
     * 现在只打印第一条和每第 50 条（约每 0.5 s 一条），恢复后打一条恢复消息。 */
    if (status != HAL_OK)
    {
        motor->tx_fail_streak++;
        if ((motor->tx_fail_streak == 1U) ||
            ((motor->tx_fail_streak % 50U) == 0U))
        {
            char line[176];
            uint32_t hal_err = HAL_FDCAN_GetError(motor->cfg.can);

            /* 错误位含义（stm32h7xx_hal_fdcan.h，**不要凭印象写**）：
             *   0x00000200 = HAL_FDCAN_ERROR_FIFO_FULL  -> 写不进 TX FIFO
             *   0x00000001 = HAL_FDCAN_ERROR_TIMEOUT    -> 等空位超时
             *   0x08000000 = HAL_FDCAN_ERROR_LOG_OVERFLOW -> 错误日志溢出
             *                （错误累积极快，通常是没有 ACK / 总线异常）
             * HAL 里没有 HAL_FDCAN_ERROR_ACK 这个宏（曾经写错过）。 */
            (void)snprintf(line, sizeof(line),
                           "[DM] TX FAILED x%lu id=0x%03lX err=0x%08lX%s%s%s\r\n",
                           (unsigned long)motor->tx_fail_streak,
                           (unsigned long)id,
                           (unsigned long)hal_err,
                           (DM_IsBusOff(motor) != 0U) ?
                               " BUS-OFF(TX disabled by controller)" : "",
                           ((hal_err & HAL_FDCAN_ERROR_FIFO_FULL) != 0U) ?
                               " FIFO_FULL" : "",
                           ((hal_err & HAL_FDCAN_ERROR_LOG_OVERFLOW) != 0U) ?
                               " ERRLOG_OVERFLOW" : "");
            DM_Print(line);

            /* 关键区分：FIFO_FULL **不一定**是"FIFO 真的满了"。
             * 控制器 bus-off 时 TX 被禁止，HAL_FDCAN_AddMessageToTxFifoQ() 只检查
             * TXFQS.TFQF，于是每帧都报 FIFO_FULL —— 台架实测的 1100+ 条刷屏就是这个：
             * 看起来像 FIFO 满，其实是"控制器停了、自己不会退出"。
             * 所以这里按 now_busoff 分流，给出不同的动作。 */
            if (DM_IsBusOff(motor) != 0U)
            {
                DM_Print("[DM] cause: controller is in BUS-OFF, so TX is disabled. This is\r\n"
                         "     NOT a full FIFO. Auto-recovery handles it (see status for the\r\n"
                         "     bus-off counters); check CANH/CANL, 120R, common ground and the\r\n"
                         "     motor's 24 V supply.\r\n");
            }
            else
            {
                DM_Print("[DM] cause: frames cannot be queued. Run `dm recover`, then\r\n"
                         "     `dm probe` to see whether the motor answers at all.\r\n");
            }
        }
    }
    else if (motor->tx_fail_streak != 0U)
    {
        char line[112];

        (void)snprintf(line, sizeof(line),
                       "[DM] TX recovered after %lu failures\r\n",
                       (unsigned long)motor->tx_fail_streak);
        DM_Print(line);
        motor->tx_fail_streak = 0U;
    }
    else
    {
        /* 正常路径：什么都不打印 */
    }
#endif

    return status;
}

/* 特殊命令帧：`FF FF FF FF FF FF FF cmd`，发到 esc_id + 模式基址 */
static HAL_StatusTypeDef DM_SendSpecialCmd(dm_motor_t *motor, uint8_t cmd)
{
    uint16_t base;
    uint8_t data[8];

    switch (motor->active_mode)
    {
        case DM_MIT_MODE: base = DM_MODE_MIT; break;
        case DM_POS_MODE: base = DM_MODE_POS; break;
        case DM_SPD_MODE: base = DM_MODE_SPD; break;
        case DM_PSI_MODE: base = DM_MODE_PSI; break;
        default:          base = DM_MODE_POS; break;
    }

    (void)memset(data, 0xFF, sizeof(data));
    data[7] = cmd;

    return DM_SendFrame(motor, (uint16_t)(motor->cfg.esc_id + base), data, 8U);
}

/* Wait only in a task context.  fb_count is incremented exclusively after
 * the RX handler has validated standard/data/DLC8/ID, so a count change is a
 * fresh legal motor feedback rather than an unrelated bus frame. */
static void DM_ReadFeedbackSnapshot(const dm_motor_t *motor,
                                    dm_feedback_t *out)
{
    uint32_t primask;

    if ((motor == NULL) || (out == NULL))
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = motor->fb;
    __set_PRIMASK(primask);
}

__weak void DM_WaitService(dm_motor_t *motor){(void)motor;}
static void DM_WaitYield(dm_motor_t *motor,uint32_t ms){DM_WaitService(motor);osDelay(ms);}

static uint8_t DM_EnableWaitFeedback(dm_motor_t *motor,
                                     uint32_t baseline_count,
                                     uint32_t timeout_ms,
                                     uint8_t required_state,
                                     dm_feedback_t *fresh_out)
{
    uint32_t start_tick;
    dm_feedback_t snapshot;

    if (motor == NULL)
    {
        return 0U;
    }

    start_tick = HAL_GetTick();
    for (;;)
    {
        if (motor->enable_pending == 0U)
        {
            return 0U; /* explicit disable/mode change cancelled the attempt */
        }
        DM_ReadFeedbackSnapshot(motor, &snapshot);
        if (snapshot.fb_count != baseline_count)
        {
            /* 0=disabled and 1=enabled are the only legal non-fault states.
             * Any other status is a fresh motor fault and must abort enable. */
            if (snapshot.err_code > DM_FB_STATE_ENABLED)
            {
                return 0U;
            }
            if ((required_state > DM_FB_STATE_ENABLED) ||
                (snapshot.err_code == required_state))
            {
                if (fresh_out != NULL)
                {
                    *fresh_out = snapshot;
                }
                return 1U;
            }
            baseline_count = snapshot.fb_count;
        }
        /* Inspect a reply delivered by the final delay before timing out. */
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return 0U;
        }
        DM_WaitYield(motor,1U);
    }
}

/* These short commits are the only places where the task publishes a new
 * state/target pair.  Interrupts (and therefore a pending FreeRTOS switch)
 * are masked only for the in-memory writes; no HAL call or logging is made in
 * the critical section. */
static uint8_t DM_EnableAbort(dm_motor_t *motor, dm_state_e state,
                              uint8_t fault_latched)
{
    uint32_t primask;

    if (motor == NULL)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor->enable_pending == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    motor->state = state;
    motor->fault_latched = fault_latched;
    motor->enable_sent = 0U;
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
    __DMB();
    motor->enable_pending = 0U;
    motor->resend_hold = 0U;
    __set_PRIMASK(primask);
    return 1U;
}

static uint8_t DM_EnableCommit(dm_motor_t *motor, float hold_pos)
{
    uint32_t primask;

    if (motor == NULL)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor->enable_pending == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    motor->target_pos_rad = hold_pos;
    motor->target_vel_rads = DM_ENABLE_HOLD_VEL_RAD;
    motor->enable_sent = 1U;
    motor->fault_latched = 0U;
    motor->state = DM_STATE_ENABLED;
    motor->resend_enable = 1U;
    motor->resend_next_tick = HAL_GetTick();
    __DMB();
    motor->enable_pending = 0U;
    motor->resend_hold = 0U;
    __set_PRIMASK(primask);
    return 1U;
}

static void DM_DrainRxBeforeEnable(dm_motor_t *motor)
{
    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return;
    }
    /* The FIFO has one consumer.  Temporarily mask the FDCAN1 IRQ while the
     * bounded handler drains queued frames; task and ISR never call HAL's
     * FIFO pop concurrently. */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    DM_RxFifo0Handler(motor);
    __DSB();
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/* MIT 8 字节帧：pos 16 位、vel 12 位、kp 12 位、kd 12 位、tor 12 位 */
static void DM_SendMitFrame(dm_motor_t *motor, float pos, float vel,
                            float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t pos_u = (uint16_t)DM_FloatToUint(pos, -DM_P_MAX, DM_P_MAX, 16);
    uint16_t vel_u = (uint16_t)DM_FloatToUint(vel, -DM_V_MAX, DM_V_MAX, 12);
    uint16_t kp_u  = (uint16_t)DM_FloatToUint(kp, 0.0f, DM_KP_MAX, 12);
    uint16_t kd_u  = (uint16_t)DM_FloatToUint(kd, 0.0f, DM_KD_MAX, 12);
    uint16_t tor_u = (uint16_t)DM_FloatToUint(tor, -DM_T_MAX, DM_T_MAX, 12);

    data[0] = (uint8_t)(pos_u >> 8);
    data[1] = (uint8_t)(pos_u & 0xFFU);
    data[2] = (uint8_t)(vel_u >> 4);
    data[3] = (uint8_t)(((vel_u & 0x0FU) << 4) | (kp_u >> 8));
    data[4] = (uint8_t)(kp_u & 0xFFU);
    data[5] = (uint8_t)(kd_u >> 4);
    data[6] = (uint8_t)(((kd_u & 0x0FU) << 4) | (tor_u >> 8));
    data[7] = (uint8_t)(tor_u & 0xFFU);

    (void)DM_SendFrame(motor, (uint16_t)(motor->cfg.esc_id + DM_MODE_MIT), data, 8U);
}

/* POS 8 字节帧：小端 float 位置 + 小端 float 速度。
 * 用 memcpy 而不是 *(float*)&data[0]：ARMCC 下前者不违反严格别名规则，也不依赖对齐。 */
static const uint8_t DM_DiagRids[]={10U,13U,14U,20U,21U,22U,23U,56U,80U,81U};
static const char *const DM_DiagNames[]={"CTRL_MODE","hw_ver","sw_ver","Gr","PMAX","VMAX","TMAX","m_off","p_m","xout"};
static uint8_t DM_DiagReply(dm_motor_t *motor,const uint8_t *data,uint32_t tick)
{
    uint8_t i,known=0U;
    if(data[0]!=(uint8_t)motor->cfg.esc_id || data[1]!=(uint8_t)(motor->cfg.esc_id>>8U) || data[2]!=0x33U)return 0U;
    for(i=0U;i<sizeof(DM_DiagRids);i++)if(data[3]==DM_DiagRids[i])known=1U;
    if(!known)return 0U;
    /* Parameter frames must NEVER enter packed position/state feedback decode. */
    if(motor->diag_expected && data[3]==motor->diag_rid && (int32_t)(tick-motor->diag_request_tick)>=0) {
        uint32_t raw;memcpy(&raw,data+4U,4U);
        motor->diag_reply_raw=raw;motor->diag_reply_tick=tick;
        __DMB();motor->diag_reply_ready=1U;
    }
    return 1U; /* Discard late/out-of-order parameter replies without refreshing FB. */
}
uint8_t DM_Motor_DiagIsActive(const dm_motor_t *motor) { return motor?motor->diag_active:0U; }
void DM_Motor_DiagCancel(dm_motor_t *motor)
{
    if(!motor || __get_IPSR()!=0U)return;
    motor->diag_active=0U;motor->diag_expected=0U;motor->diag_reply_ready=0U;
}
HAL_StatusTypeDef DM_Motor_DiagStart(dm_motor_t *motor)
{
    if(!motor || !motor->cfg.can || __get_IPSR()!=0U)return HAL_ERROR;
    if(motor->diag_active || motor->enable_pending || motor->resend_hold)return HAL_BUSY;
    if(motor->fb.fb_count && HAL_GetTick()-motor->fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT &&
       (!isfinite(motor->fb.vel_rads) || fabsf(motor->fb.vel_rads)>0.10f))return HAL_BUSY;
    motor->diag_index=0U;motor->diag_expected=0U;motor->diag_reply_ready=0U;
    motor->diag_next_tick=HAL_GetTick();motor->diag_active=1U;
    DM_Print("[DM DIAG] read-only scan started; no enable/zero/mode/flash writes\r\n");
    return HAL_OK;
}
void DM_Motor_DiagPoll(dm_motor_t *motor,uint32_t now)
{
    char line[160];
    if(!motor || !motor->diag_active || __get_IPSR()!=0U)return;
    if(motor->fb.fb_count && now-motor->fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT &&
       (!isfinite(motor->fb.vel_rads) || fabsf(motor->fb.vel_rads)>0.10f)) {
        DM_Motor_DiagCancel(motor);DM_Print("[DM DIAG] cancelled: motor moving; no target change\r\n");return;
    }
    if(motor->diag_expected) {
        uint32_t raw=0U,received=0U,key=__get_PRIMASK();uint8_t ready;
        __disable_irq();ready=motor->diag_reply_ready;
        if(ready) { raw=motor->diag_reply_raw;received=motor->diag_reply_tick;motor->diag_expected=0U;motor->diag_reply_ready=0U; }
        __set_PRIMASK(key);
        if(ready) {
            if(motor->diag_index<3U) {
                uint8_t b0=(uint8_t)(raw>>24U),b1=(uint8_t)(raw>>16U),b2=(uint8_t)(raw>>8U),b3=(uint8_t)raw;
                if(motor->diag_index>0U && b0>=32U && b0<=126U && b1>=32U && b1<=126U && b2>=32U && b2<=126U && b3>=32U && b3<=126U)
                    (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s raw=0x%08lX ascii_hi=%c%c%c%c ascii_lo=%c%c%c%c age=%lums\r\n",
                        DM_DiagRids[motor->diag_index],DM_DiagNames[motor->diag_index],(unsigned long)raw,b0,b1,b2,b3,b3,b2,b1,b0,(unsigned long)(now-received));
                else (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s=%lu raw=0x%08lX age=%lums\r\n",
                        DM_DiagRids[motor->diag_index],DM_DiagNames[motor->diag_index],(unsigned long)raw,(unsigned long)raw,(unsigned long)(now-received));
            } else {
                float value;memcpy(&value,&raw,4U);
                if(isfinite(value))
                    (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s=%.7g raw=0x%08lX age=%lums\r\n",
                        DM_DiagRids[motor->diag_index],DM_DiagNames[motor->diag_index],(double)value,(unsigned long)raw,(unsigned long)(now-received));
                else (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s=INVALID raw=0x%08lX\r\n",
                    DM_DiagRids[motor->diag_index],DM_DiagNames[motor->diag_index],(unsigned long)raw);
            }
            DM_Print(line);motor->diag_index++;motor->diag_next_tick=now+10U;
        } else if(now-motor->diag_request_tick>=150U) {
            motor->diag_expected=0U;
            (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s=UNKNOWN (no register reply)\r\n",
                DM_DiagRids[motor->diag_index],DM_DiagNames[motor->diag_index]);DM_Print(line);
            motor->diag_index++;motor->diag_next_tick=now+10U;
        }
        return;
    }
    if(motor->diag_index>=sizeof(DM_DiagRids)) {
        motor->diag_active=0U;DM_Print("[DM DIAG] DONE; p_m/xout are raw register floats, zero/target unchanged\r\n");return;
    }
    if((int32_t)(now-motor->diag_next_tick)<0)return;
    {
        uint8_t data[4]={(uint8_t)motor->cfg.esc_id,(uint8_t)(motor->cfg.esc_id>>8U),0x33U,DM_DiagRids[motor->diag_index]};
        HAL_StatusTypeDef status;
        motor->diag_rid=data[3];motor->diag_request_tick=now;motor->diag_reply_ready=0U;
        __DMB();motor->diag_expected=1U;
        status=DM_SendFrame(motor,0x7FFU,data,4U);
        if(status!=HAL_OK) {
            motor->diag_expected=0U;
            (void)snprintf(line,sizeof(line),"[DM DIAG] rid=%u %s=UNKNOWN (TX HAL=%u)\r\n",
                data[3],DM_DiagNames[motor->diag_index],(unsigned int)status);DM_Print(line);
            motor->diag_index++;motor->diag_next_tick=now+10U;
        }
    }
}

static HAL_StatusTypeDef DM_SendPosFrame(dm_motor_t *motor, float pos, float vel)
{
    uint8_t data[8];

    (void)memcpy(&data[0], &pos, 4U);
    (void)memcpy(&data[4], &vel, 4U);

    return DM_SendFrame(motor, (uint16_t)(motor->cfg.esc_id + DM_MODE_POS), data, 8U);
}

/* SPD 4 字节帧：小端 float 速度 */
static void DM_SendSpdFrame(dm_motor_t *motor, float vel)
{
    uint8_t data[4];

    (void)memcpy(&data[0], &vel, 4U);

    (void)DM_SendFrame(motor, (uint16_t)(motor->cfg.esc_id + DM_MODE_SPD), data, 4U);
}

#if DM_BUS_USE_FD_PSI
/* PSI 混控为 12 字节 FD 帧，经典 CAN 下不编译 */
static void DM_SendPsiFrame(dm_motor_t *motor, float pos, float vel, float cur)
{
    uint8_t data[12];

    (void)memcpy(&data[0], &pos, 4U);
    (void)memcpy(&data[4], &vel, 4U);
    (void)memcpy(&data[8], &cur, 4U);

    (void)DM_SendFrame(motor, (uint16_t)(motor->cfg.esc_id + DM_MODE_PSI), data, 12U);
}
#endif

/* ================= 中断入口 ================= */

void DM_OnCanError(dm_motor_t *motor, uint32_t interrupts)
{
    if (motor == NULL)
    {
        return;
    }
    /* 中断上下文：只置标志与计数。绝不打印、绝不阻塞。
     * bus-off 的**实际恢复**放在 DM_Motor_Service（任务上下文）里做，
     * 因为恢复要调用 HAL（可能等待），不能在中断里干。 */
    motor->can_error_flags |= interrupts;
    if (motor->cfg.can != NULL) (void)DM_ReadPsr(motor->cfg.can);
    if ((interrupts & FDCAN_IT_BUS_OFF) != 0U)
    {
        motor->bus_off_latched = 1U;
    }
}

void DM_RxFifo0Handler(dm_motor_t *motor)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[DM_FRAME_MAX_BYTES];
    uint8_t guard = 0U;

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return;
    }

    /* 限制单次中断里最多消费 8 帧，避免总线异常时在 ISR 里长时间循环 */
    while (guard < 8U)
    {
        uint32_t dlc_bytes = 0U;

        (void)memset(&rx, 0, sizeof(rx));
        if (HAL_FDCAN_GetRxMessage(motor->cfg.can, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            break;
        }
        guard++;

        if ((rx.IdType != FDCAN_STANDARD_ID) ||
            (rx.RxFrameType != FDCAN_DATA_FRAME) ||
            (rx.FDFormat != FDCAN_CLASSIC_CAN))
        {
            motor->rx_other_count++;
            continue;
        }

        /* 只认本电机的反馈 ID。旧版比较 (uint32_t)s_mst_id 时把范围搞错过一次，
         * 这里显式按 11 位标准 ID 比较。 */
        if ((uint16_t)rx.Identifier != (uint16_t)motor->cfg.mst_id)
        {
            motor->rx_other_count++;
            continue;
        }

        /* 反馈帧必须是经典 CAN 的标准数据帧、精确 DLC=8。 */
        dlc_bytes = (uint32_t)((rx.DataLength >> 16U) & 0x0FU);
        if (dlc_bytes != DM_FB_LENGTH)
        {
            /* 短帧/FD 长度异常：不解析，也不刷新失联时间。 */
            continue;
        }
        if(DM_DiagReply(motor,data,HAL_GetTick()))continue;

        /* 第 4 个低位是电机 ESC ID；只接受这个驱动实例的反馈。 */
        if ((data[0] & 0x0FU) != (motor->cfg.esc_id & 0x0FU))
        {
            motor->rx_other_count++;
            continue;
        }

        /* 只有到这里才是可用于安全联锁的、匹配本电机的合法反馈。 */
        motor->last_rx_tick = HAL_GetTick();

        /* 8 字节反馈：
         *   data[0] = 状态码<<4 | ID
         *   data[1..2] = 位置（16 位**无符号**，中心 0）
         *   data[3..4] = 速度（12 位无符号）
         *   data[4..5] = 扭矩（12 位无符号）
         *   data[6] = MOS 温度 ℃，data[7] = 线圈温度 ℃ */
        motor->fb.err_code = (uint8_t)(data[0] >> 4);
        motor->fb.enabled = (motor->fb.err_code == DM_FB_STATE_ENABLED) ? 1U : 0U;

        motor->fb.pos_rad = DM_UintToFloat(((uint32_t)data[1] << 8) | (uint32_t)data[2],
                                           -DM_P_MAX, DM_P_MAX, 16);
        motor->fb.vel_rads = DM_UintToFloat((((uint32_t)data[3] << 4) |
                                             ((uint32_t)data[4] >> 4)),
                                            -DM_V_MAX, DM_V_MAX, 12);
        motor->fb.tor_nm = DM_UintToFloat(((((uint32_t)data[4] & 0x0FU) << 8) |
                                           (uint32_t)data[5]),
                                          -DM_T_MAX, DM_T_MAX, 12);
        motor->fb.t_mos_c = (float)data[6];
        motor->fb.t_coil_c = (float)data[7];
        /* 老版本固件这两个温度字段填 0（本车电机疑似老版本，用户 2026-09-14 反馈）。
         * 0 ℃ 与"不支持"无法区分，所以非 0 才认为可信。 */
        motor->fb.temp_valid = ((data[6] != 0U) || (data[7] != 0U)) ? 1U : 0U;

        motor->fb.fb_count++;
        motor->fb.last_tick = motor->last_rx_tick;
        motor->rx_count++;
    }
}

/* ================= 总线初始化（供 Init、回环与波特率扫描共用） ================= */

/**
  * @brief  波特率代码 → 名义位时序。取值直接照搬官方例程
  *         `dm_ctrl_h7_fdcan_v1.1/User/bsp_fdcan.c` 的 `bsp_fdcan_set_baud()`
  *         （CAN_CLASS 分支），**不自己算**：那是厂家在同一个 80 MHz FDCAN 内核
  *         上验证过的组合，自己算容易在采样点上引入无关变量。
  *         代码含义见 dm_motor.h 的 DM_BITRATE_* 与官方 bsp_fdcan.h 的 CAN_BR_*。
  */
static void DM_BitrateTiming(uint8_t code, uint32_t *prescaler,
                             uint32_t *seg1, uint32_t *seg2, uint32_t *sjw)
{
    switch (code)
    {
        case DM_BITRATE_125K: *prescaler = 4U; *seg1 = 139U; *seg2 = 20U; *sjw = 20U; break;
        case DM_BITRATE_200K: *prescaler = 2U; *seg1 = 174U; *seg2 = 25U; *sjw = 25U; break;
        case DM_BITRATE_250K: *prescaler = 2U; *seg1 = 139U; *seg2 = 20U; *sjw = 20U; break;
        case DM_BITRATE_500K: *prescaler = 1U; *seg1 = 139U; *seg2 = 20U; *sjw = 20U; break;
        case DM_BITRATE_1M:   *prescaler = 4U; *seg1 = 15U;  *seg2 = 4U;  *sjw = 2U;  break;
        case DM_BITRATE_2M:   *prescaler = 1U; *seg1 = 29U;  *seg2 = 10U; *sjw = 10U; break;
        case DM_BITRATE_2M5:  *prescaler = 1U; *seg1 = 25U;  *seg2 = 6U;  *sjw = 6U;  break;
        case DM_BITRATE_3M2:  *prescaler = 1U; *seg1 = 19U;  *seg2 = 5U;  *sjw = 5U;  break;
        case DM_BITRATE_4M:   *prescaler = 1U; *seg1 = 14U;  *seg2 = 5U;  *sjw = 5U;  break;
        case DM_BITRATE_5M:   *prescaler = 1U; *seg1 = 13U;  *seg2 = 2U;  *sjw = 2U;  break;
        default:              *prescaler = 4U; *seg1 = 15U;  *seg2 = 4U;  *sjw = 2U;  break;
    }
}

const char *DM_BitrateName(uint8_t code)
{
    switch (code)
    {
        case DM_BITRATE_125K: return "125K";
        case DM_BITRATE_200K: return "200K";
        case DM_BITRATE_250K: return "250K";
        case DM_BITRATE_500K: return "500K";
        case DM_BITRATE_1M:   return "1M";
        case DM_BITRATE_2M:   return "2M";
        case DM_BITRATE_2M5:  return "2M5";
        case DM_BITRATE_3M2:  return "3M2";
        case DM_BITRATE_4M:   return "4M";
        case DM_BITRATE_5M:   return "5M";
        default:              return "?";
    }
}

uint8_t DM_GetBitrateCode(const dm_motor_t *motor)
{
    return (motor != NULL) ? motor->cfg.bitrate_code : DM_BITRATE_DEFAULT;
}

/**
  * @brief  按给定模式和当前 cfg.bitrate_code 把 FDCAN1 落位并启动。
  * @note   只动总线，不碰电机状态。
  */
static HAL_StatusTypeDef DM_BusConfigure(dm_motor_t *motor, uint32_t mode)
{
    uint32_t prescaler;
    uint32_t seg1;
    uint32_t seg2;
    uint32_t sjw;

    DM_BitrateTiming(motor->cfg.bitrate_code, &prescaler, &seg1, &seg2, &sjw);

    motor->cfg.can->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    motor->cfg.can->Init.Mode = mode;
    /* ⚠ AutoRetransmission 必须是 DISABLE（2026-09-14 台架实测后改回来）。
     *
     * 曾经为了"通信可靠"把它设成 ENABLE —— **那是个陷阱**：当对端不应答
     * （ACK 错误）时，FDCAN 硬件会**无限重传同一帧**，TX FIFO（4 个元素）被永久
     * 占满，之后每一帧都写入失败（HAL_ERROR + err=0x08000200 = TIMEOUT|ACK）。
     * 后果不只是"发不出去"，而是**连失能帧都发不出去** —— 电机带着力矩却没有
     * 任何办法让它松，属于安全问题。
     * CubeMX 生成的占位值本来就是 DISABLE，现在按"失败就丢帧、把 FIFO 让出来"的
     * 策略执行：宁可丢一帧，也不能把发送通道锁死。 */
    motor->cfg.can->Init.AutoRetransmission = DISABLE;
    motor->cfg.can->Init.TransmitPause = DISABLE;
    motor->cfg.can->Init.ProtocolException = ENABLE;
    /* 名义位（经典 CAN 下唯一生效的一侧），由波特率代码决定 */
    motor->cfg.can->Init.NominalPrescaler = prescaler;
    motor->cfg.can->Init.NominalSyncJumpWidth = sjw;
    motor->cfg.can->Init.NominalTimeSeg1 = seg1;
    motor->cfg.can->Init.NominalTimeSeg2 = seg2;
    /* 数据域位时序：经典 CAN 下不生效，仅为将来切 FD 预留 2.5 Mbps */
    motor->cfg.can->Init.DataPrescaler = 2U;
    motor->cfg.can->Init.DataSyncJumpWidth = 2U;
    motor->cfg.can->Init.DataTimeSeg1 = 11U;
    motor->cfg.can->Init.DataTimeSeg2 = 4U;
    /* 消息 RAM：FDCAN2（ZDT）占用 0~127，本模块从 128 起 */
    motor->cfg.can->Init.MessageRAMOffset = DM_RAM_OFFSET;
    motor->cfg.can->Init.StdFiltersNbr = 1U;
    motor->cfg.can->Init.ExtFiltersNbr = 0U;
    motor->cfg.can->Init.RxFifo0ElmtsNbr = DM_RX_FIFO_ELMTS;
    motor->cfg.can->Init.RxFifo0ElmtSize = DM_RX_ELMT_SIZE;
    motor->cfg.can->Init.RxFifo1ElmtsNbr = 0U;
    motor->cfg.can->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    motor->cfg.can->Init.RxBuffersNbr = 0U;
    motor->cfg.can->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    motor->cfg.can->Init.TxEventsNbr = 0U;
    motor->cfg.can->Init.TxBuffersNbr = 0U;
    motor->cfg.can->Init.TxFifoQueueElmtsNbr = DM_TX_FIFO_ELMTS;
    motor->cfg.can->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    motor->cfg.can->Init.TxElmtSize = DM_TX_ELMT_SIZE;

    if (HAL_FDCAN_Init(motor->cfg.can) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 标准 ID 掩码过滤器 0/0 = 全通过；验收在 DM_RxFifo0Handler 里按 MST ID 做，
     * 这样 `dm probe` 才能看到总线上所有帧，便于诊断 ID 配错。 */
    {
        FDCAN_FilterTypeDef filter;

        (void)memset(&filter, 0, sizeof(filter));
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0U;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0x000U;
        filter.FilterID2 = 0x000U;
        (void)HAL_FDCAN_ConfigFilter(motor->cfg.can, &filter);
    }
    (void)HAL_FDCAN_ConfigGlobalFilter(motor->cfg.can, FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_ConfigFifoWatermark(motor->cfg.can, FDCAN_CFG_RX_FIFO0, 1U);

    /* FDCAN1_IT0 中断优先级 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，
     * 也就是说本 ISR 里**不允许**调 FreeRTOS API。DM_RxFifo0Handler 只用
     * volatile 计数和 ZDT_Console_Print（中断里只入队），满足该约束。 */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    dm_lec_seen = 0U;
    dm_lec_last = 0U;
    if (HAL_FDCAN_Start(motor->cfg.can) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 除 RX FIFO0 外再打开总线错误通知，让 bus-off / error passive 可见。
     * 旧版只开了 RX_FIFO0_WATERMARK，FDCAN1 总线断了固件完全看不见。 */
    return HAL_FDCAN_ActivateNotification(motor->cfg.can,
                                          FDCAN_IT_RX_FIFO0_WATERMARK |
                                          FDCAN_IT_BUS_OFF |
                                          FDCAN_IT_ERROR_WARNING |
                                          FDCAN_IT_ERROR_PASSIVE |
                                          FDCAN_IT_ARB_PROTOCOL_ERROR |
                                          FDCAN_IT_DATA_PROTOCOL_ERROR,
                                          0U);
}

/* 回环测试用：切到指定模式 */
static HAL_StatusTypeDef DM_InitLoopback(dm_motor_t *motor, uint32_t mode)
{
    (void)HAL_FDCAN_DeInit(motor->cfg.can);
    return DM_BusConfigure(motor, mode);
}

/* 回环测试后回到正常总线模式，并清掉测试期间累积的错误状态 */
static HAL_StatusTypeDef DM_ReinitNormal(dm_motor_t *motor)
{
    HAL_StatusTypeDef status;

    (void)HAL_FDCAN_DeInit(motor->cfg.can);
    status = DM_BusConfigure(motor, FDCAN_MODE_NORMAL);
    /* 回环期间控制器不会进 bus-off，但真总线上可能已经进过；重新 Init 已经
     * 复位了 CCCR/TEST，这里把诊断计数清零，避免旧的 busoff 计数误导现场。 */
    motor->can_error_flags = 0U;
    motor->bus_off_count = 0U;
    return status;
}

HAL_StatusTypeDef DM_Motor_SetBitrate(dm_motor_t *motor, uint8_t code)
{
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return HAL_ERROR;
    }
    if (code > DM_BITRATE_CODE_MAX)
    {
        return HAL_ERROR;
    }
    /* 改波特率要重建控制器，运动中做这件事会让电机瞬间丢帧，所以先松劲。
     * 这里的失能是本地语义（保证不带着力矩切总线），不是"故障停机"。 */
    if (motor->enable_sent != 0U)
    {
        (void)DM_SendSpecialCmd(motor, 0xFDU);
        motor->enable_sent = 0U;
        motor->state = DM_STATE_READY;
    }
    motor->enable_pending = 0U;
    motor->resend_hold = 0U;

    motor->cfg.bitrate_code = code;
    (void)HAL_FDCAN_DeInit(motor->cfg.can);
    status = DM_BusConfigure(motor, FDCAN_MODE_NORMAL);

    /* 重新起总线后，旧的反馈时间戳/计数不再有意义，一并清掉，
     * 否则 DM_Motor_Service 的失联判据会拿旧时间戳直接判超时。 */
    motor->fb.fb_count = 0U;
    motor->fb.last_tick = 0U;
    motor->last_rx_tick = 0U;
    motor->rx_count = 0U;
    motor->rx_other_count = 0U;
    motor->can_error_flags = 0U;
    motor->bus_off_count = 0U;
    motor->fault_latched = 0U;

    return status;
}

/* ================= 初始化 ================= */

void DM_SetConfig(dm_motor_t *motor, const dm_config_t *config)
{
    if ((motor == NULL) || (config == NULL))
    {
        return;
    }
    motor->cfg = *config;
    motor->active_mode = config->mode;
    if (motor->cfg.bitrate_code > DM_BITRATE_CODE_MAX)
    {
        motor->cfg.bitrate_code = DM_BITRATE_DEFAULT;
    }
    if (motor->cfg.pos_limit_rad <= 0.0f)
    {
        motor->cfg.pos_limit_rad = DM_POS_LIMIT_RAD;
    }
    if (motor->cfg.vel_limit_rad <= 0.0f)
    {
        motor->cfg.vel_limit_rad = DM_VEL_LIMIT_RAD;
    }
}

HAL_StatusTypeDef DM_Motor_Init(dm_motor_t *motor, const dm_config_t *config)
{
    HAL_StatusTypeDef status;
    dm_feedback_t zero_fb;

    if ((motor == NULL) || (config == NULL) || (config->can == NULL))
    {
        return HAL_ERROR;
    }

    /* 清全部运行时状态，但保留配置 */
    (void)memset(&zero_fb, 0, sizeof(zero_fb));
    motor->fb = zero_fb;
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
    motor->enable_sent = 0U;
    motor->enable_pending = 0U;
    motor->fault_latched = 0U;
    motor->last_err = 0U;
    motor->last_fb_count = 0U;
    motor->tx_count = 0U;
    motor->tx_fail_count = 0U;
    motor->tx_fail_streak = 0U;
    motor->rx_count = 0U;
    motor->rx_other_count = 0U;
    motor->link_lost_count = 0U;
    motor->last_rx_tick = 0U;
    motor->can_error_flags = 0U;
    motor->bus_off_count = 0U;
    motor->bus_off_latched = 0U;
    motor->bus_off_tick = 0U;
    motor->bus_off_recover_count = 0U;
    motor->tx_log = 0U;
    /* 周期重发：编译期默认 DM_PERIODIC_RESEND，运行期可用 `dm resend 0|1` 改。
     * 实测电机不自发反馈，所以默认必须是开（见 dm_motor.h 的说明）。 */
    motor->resend_enable = (uint8_t)DM_PERIODIC_RESEND;
    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();
    motor->stop_generation=0U;
    motor->diag_active=0U;motor->diag_expected=0U;motor->diag_reply_ready=0U;motor->diag_index=0U;
    DM_SetConfig(motor, config);

    /* CubeMX 生成的 FDCAN1 有一个"发不出帧"的坑：
     * fdcan.c 里 TxFifoQueueElmtsNbr = 0、RxFifo0ElmtSize = 8 字节、
     * AutoRetransmission = DISABLE，都是占位值。这里按达妙需求重配。
     *
     * 注意：本函数因此**承担了修 CubeMX 配置的职责**。如果以后 CubeMX 重新生成并
     * 覆盖了 fdcan.c，请把这几项也同步写进 .ioc，否则要记得这里会兜住。
     *
     * 位时序/过滤器/通知的落位统一在 DM_BusConfigure() 里，`dm loopback` 自检
     * 复用同一份配置，避免两处参数漂移。 */
    (void)HAL_FDCAN_DeInit(motor->cfg.can);
    status = DM_BusConfigure(motor, FDCAN_MODE_NORMAL);
    if (status != HAL_OK)
    {
        motor->state = DM_STATE_RESET;
        motor->last_err = (uint8_t)status;
        return status;
    }


    motor->state = DM_STATE_READY;
    motor->last_err = (uint8_t)HAL_OK;

    /* 注意：这里**不发控制帧、不使能电机**（上电不运动约定）。 */
    return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_InitDefault(dm_motor_t *motor, FDCAN_HandleTypeDef *can)
{
    dm_config_t cfg;

    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.can = can;
    cfg.esc_id = DM_ESC_ID_DEFAULT;
    cfg.mst_id = DM_MST_ID_DEFAULT;
    cfg.mode = DM_POS_MODE;
    cfg.link_timeout_ms = DM_LINK_TIMEOUT_MS_DEFAULT;
    cfg.bitrate_code = DM_BITRATE_DEFAULT;
    cfg.pos_limit_rad = DM_POS_LIMIT_RAD;
    cfg.vel_limit_rad = DM_VEL_LIMIT_RAD;

    return DM_Motor_Init(motor, &cfg);
}

/* ================= 使能 / 失能 ================= */

void DM_Motor_SetMode(dm_motor_t *motor, dm_mode_e mode)
{
    if (motor == NULL)
    {
        return;
    }
    /* 模式改变后必须重新使能：使能帧要发到新的模式基址上。
     * （旧版 `dm speed` / `dm pos` 会静默改 s_mode，导致下一次 `dm enable`
     *   发到另一个 ID 上，现象是"使能没反应"。） */
    if (motor->active_mode != mode)
    {
        motor->enable_pending = 0U;
        motor->resend_hold = 0U;
        motor->active_mode = mode;
        motor->cfg.mode = mode;
        motor->enable_sent = 0U;
        if (motor->state == DM_STATE_ENABLED)
        {
            motor->state = DM_STATE_READY;
        }
    }
}

/* FC may still be starting the drive when its first POS reply is produced.
 * Poll with the same captured position, never a zero target or another FC.
 * ArmTask's normal resend is gated during this bounded task-side exchange. */
static HAL_StatusTypeDef DM_EnableConfirmHold(dm_motor_t *motor,
                                             uint32_t baseline_count,
                                             float hold_pos,
                                             dm_feedback_t *last_out)
{
    uint32_t start = HAL_GetTick();
    uint32_t next_query;
    dm_feedback_t fb;
    HAL_StatusTypeDef status;

    if (!motor->enable_pending)
    {
        return HAL_ERROR;
    }
    /* Establish the hold even if FC has already produced an enabled reply. */
    status = DM_SendPosFrame(motor, hold_pos, DM_ENABLE_HOLD_VEL_RAD);
    if (status != HAL_OK)
    {
        return status;
    }
    next_query = HAL_GetTick() + DM_ENABLE_HOLD_QUERY_MS;
    for (;;)
    {
        if (!motor->enable_pending)
        {
            return HAL_ERROR;
        }
        DM_ReadFeedbackSnapshot(motor, &fb);
        *last_out = fb;
        if (fb.fb_count != baseline_count)
        {
            if (fb.err_code > DM_FB_STATE_ENABLED ||
                !isfinite(fb.pos_rad) || !isfinite(fb.vel_rads) ||
                fabsf(fb.pos_rad) > motor->cfg.pos_limit_rad)
            {
                return HAL_ERROR;
            }
            if (fb.err_code == DM_FB_STATE_ENABLED)
            {
                return HAL_OK;
            }
            baseline_count = fb.fb_count;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= DM_ENABLE_CONFIRM_TIMEOUT_MS)
        {
            return HAL_TIMEOUT;
        }
        if ((int32_t)(HAL_GetTick() - next_query) >= 0)
        {
            if (!motor->enable_pending)
            {
                return HAL_ERROR;
            }
            status = DM_SendPosFrame(motor, hold_pos, DM_ENABLE_HOLD_VEL_RAD);
            if (status != HAL_OK)
            {
                return status;
            }
            next_query = HAL_GetTick() + DM_ENABLE_HOLD_QUERY_MS;
        }
        DM_WaitYield(motor,1U);
    }
}

HAL_StatusTypeDef DM_Motor_Enable(dm_motor_t *motor)
{
    HAL_StatusTypeDef status;
    uint32_t primask;
    uint32_t query_fb_count;
    uint32_t confirm_fb_count;
    float hold_pos;
    uint8_t old_fault;
    dm_feedback_t query_fb;
    dm_feedback_t confirm_fb;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    /* This safety handshake is deliberately POS-only.  Do not silently send
     * an FC/FD command on a mode-specific ID selected by another command. */
    if (motor->active_mode != DM_POS_MODE)
    {
        DM_Print("[DM] enable refused: safe holding is supported only in POS mode; "
                 "run `dm pos`/recover first.\r\n");
        return HAL_ERROR;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor->enable_pending != 0U)
    {
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }

    old_fault = motor->fault_latched;
    motor->enable_pending = 1U;
    /* Keep ArmTask_Run from observing a half-written target/state while the
     * console task performs the bounded query/confirmation exchange. */
    motor->resend_hold = 1U;
    motor->enable_sent = 0U;
    motor->state = DM_STATE_READY;
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
    __set_PRIMASK(primask);
    /* Consume at most the bounded ISR batch before taking the baseline, so a
     * stale queued frame cannot satisfy this enable attempt. */
    DM_DrainRxBeforeEnable(motor);
    query_fb_count = motor->fb.fb_count;

    /* In POS mode FF..FD is a harmless query which elicits the current raw
     * position without writing motor flash. */
    status = DM_SendSpecialCmd(motor, 0xFDU);
    if ((status != HAL_OK) ||
        (DM_EnableWaitFeedback(motor, query_fb_count,
                                DM_ENABLE_QUERY_TIMEOUT_MS, DM_FB_STATE_DISABLED,
                                &query_fb) == 0U))
    {
        (void)DM_EnableAbort(motor,
                             (old_fault != 0U) ? DM_STATE_FAULT : DM_STATE_READY,
                             old_fault);
        DM_Print((status == HAL_OK) ?
                 "[DM] enable refused: no fresh legal POS feedback after disable query.\r\n" :
                 "[DM] enable refused: disable query could not be queued.\r\n");
        return HAL_ERROR;
    }

    if (!isfinite(query_fb.pos_rad) || !isfinite(query_fb.vel_rads) ||
        (query_fb.pos_rad > motor->cfg.pos_limit_rad) ||
        (query_fb.pos_rad < -motor->cfg.pos_limit_rad))
    {
        (void)DM_EnableAbort(motor,
                             (old_fault != 0U) ? DM_STATE_FAULT : DM_STATE_READY,
                             old_fault);
        DM_Print("[DM] enable refused: fresh POS feedback is outside the configured "
                 "soft limit.\r\n");
        return HAL_ERROR;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor->enable_pending == 0U)
    {
        __set_PRIMASK(primask);
        return HAL_ERROR;
    }
    hold_pos = query_fb.pos_rad;
    motor->target_pos_rad = hold_pos;
    motor->target_vel_rads = DM_ENABLE_HOLD_VEL_RAD;
    confirm_fb_count = motor->fb.fb_count;
    __set_PRIMASK(primask);

    status = DM_SendSpecialCmd(motor, 0xFCU);
    DM_ReadFeedbackSnapshot(motor, &confirm_fb);
    if (status == HAL_OK)
    {
        status = DM_EnableConfirmHold(motor, confirm_fb_count, hold_pos,
                                      &confirm_fb);
    }
    if (status != HAL_OK)
    {
        if (motor->enable_pending == 0U)
        {
            return HAL_ERROR; /* disable/recover already owns the final state */
        }
        (void)DM_SendSpecialCmd(motor, 0xFDU);
        (void)DM_EnableAbort(motor, DM_STATE_FAULT, 1U);
        {
            char line[192];
            (void)snprintf(line, sizeof(line),
                "[DM] enable failed: HAL=%u confirm fb=%lu->%lu code=%u age=%lums; "
                "disable requested, release unconfirmed.\r\n",
                (unsigned int)status, (unsigned long)confirm_fb_count,
                (unsigned long)confirm_fb.fb_count, (unsigned int)confirm_fb.err_code,
                (unsigned long)(HAL_GetTick()-confirm_fb.last_tick));
            DM_Print(line);
        }
        return HAL_ERROR;
    }

    /* A previous `dm resend 0` must not disable the hold safety path. */
    if (DM_EnableCommit(motor, hold_pos) == 0U)
    {
        return HAL_ERROR; /* cancelled after confirmation */
    }
    {
        char line[144];

        (void)snprintf(line, sizeof(line),
                       "[DM] enable confirmed: POS hold at raw %.3f rad, vel %.2f "
                       "rad/s; periodic resend forced ON.\r\n",
                       (double)hold_pos, (double)DM_ENABLE_HOLD_VEL_RAD);
        DM_Print(line);
    }
    return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_Disable(dm_motor_t *motor)
{
    HAL_StatusTypeDef status;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->resend_hold = 1U;
    motor->enable_pending = 0U;
    if (motor->state == DM_STATE_ENABLED)
    {
        motor->state = DM_STATE_READY;
    }
    status = DM_SendSpecialCmd(motor, 0xFDU);
    motor->enable_sent = 0U;
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
    motor->resend_hold = 0U;
    return status;
}

HAL_StatusTypeDef DM_Motor_ClearError(dm_motor_t *motor)
{
    dm_feedback_t fb;HAL_StatusTypeDef result;uint32_t generation,key;
    if(!motor || __get_IPSR()!=0U || motor->active_mode!=DM_POS_MODE)return HAL_ERROR;
    key=__get_PRIMASK();__disable_irq();generation=motor->stop_generation;__set_PRIMASK(key);
    result=DM_SendSpecialCmd(motor,0xFBU);
    if(result!=HAL_OK)return result; /* Failed TX must not retire the software fault. */
    result=DM_Motor_QueryDisabled(motor,300U,&fb);
    if(result!=HAL_OK)return result;
    key=__get_PRIMASK();__disable_irq();
    if(motor->stop_generation!=generation || fb.enabled || fb.err_code!=0U ||
       HAL_GetTick()-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT || !isfinite(fb.pos_rad) ||
       fabsf(fb.pos_rad)>motor->cfg.pos_limit_rad) {
        __set_PRIMASK(key);return HAL_ERROR;
    }
    /* Fresh hardware confirmation, still OFF. No FC, position command, FE,
     * coordinate edit or automatic resume occurs as part of clearing. */
    motor->fault_latched=0U;motor->state=DM_STATE_READY;motor->enable_sent=0U;
    motor->target_vel_rads=0.0f;__set_PRIMASK(key);return HAL_OK;
}

/* Console-owner convenience: fresh disabled feedback, never FC or movement.
 * Safety/emergency callers continue to use the immediate Disable function. */
HAL_StatusTypeDef DM_Motor_QueryDisabled(dm_motor_t *motor,uint32_t timeout_ms,dm_feedback_t *out)
{
    uint32_t key,baseline;dm_feedback_t fb;HAL_StatusTypeDef status;
    if(!motor || __get_IPSR()!=0U || motor->active_mode!=DM_POS_MODE || !timeout_ms || timeout_ms>500U)return HAL_ERROR;
    key=__get_PRIMASK();__disable_irq();
    if(motor->enable_pending){__set_PRIMASK(key);return HAL_BUSY;}
    motor->enable_pending=1U;motor->resend_hold=1U;motor->enable_sent=0U;
    if(motor->state==DM_STATE_ENABLED)motor->state=DM_STATE_READY;
    motor->target_pos_rad=0.0f;motor->target_vel_rads=0.0f;__set_PRIMASK(key);
    DM_DrainRxBeforeEnable(motor);DM_ReadFeedbackSnapshot(motor,&fb);baseline=fb.fb_count;
    status=DM_SendSpecialCmd(motor,0xFDU);
    if(status==HAL_OK && !DM_EnableWaitFeedback(motor,baseline,timeout_ms,DM_FB_STATE_DISABLED,&fb)) {
        DM_ReadFeedbackSnapshot(motor,&fb);
        status=(!motor->enable_pending || (fb.fb_count!=baseline && fb.err_code>DM_FB_STATE_ENABLED))?HAL_ERROR:HAL_TIMEOUT;
    }
    if(status==HAL_OK && (!isfinite(fb.pos_rad) || !isfinite(fb.vel_rads) || fb.enabled ||
        HAL_GetTick()-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT))status=HAL_ERROR;
    key=__get_PRIMASK();__disable_irq();
    if(!motor->enable_pending && status==HAL_OK)status=HAL_ERROR;
    motor->enable_pending=0U;motor->resend_hold=0U;motor->last_err=(uint8_t)status;__set_PRIMASK(key);
    if(status==HAL_OK && out)*out=fb;
    return status;
}
HAL_StatusTypeDef DM_Motor_VerifyPosZero(dm_motor_t *motor,uint32_t timeout_ms)
{
    uint32_t start=HAL_GetTick(),elapsed;dm_feedback_t fb;HAL_StatusTypeDef status;
    if(!motor || __get_IPSR()!=0U || !timeout_ms || timeout_ms>2000U)return HAL_ERROR;
    while((elapsed=HAL_GetTick()-start)<timeout_ms) {
        uint32_t left=timeout_ms-elapsed;
        status=DM_Motor_QueryDisabled(motor,left>300U?300U:left,&fb);
        if(status==HAL_OK && fabsf(fb.pos_rad)<=0.02f && fabsf(fb.vel_rads)<=0.05f)return HAL_OK;
        if(status!=HAL_OK && status!=HAL_TIMEOUT)return status;
        if(HAL_GetTick()-start>=timeout_ms)break;
        DM_WaitYield(motor,20U);
    }
    return HAL_TIMEOUT;
}
HAL_StatusTypeDef DM_Motor_ReadPosition(dm_motor_t *motor,uint32_t timeout_ms,dm_feedback_t *out)
{
    uint32_t key,baseline,start;dm_feedback_t fb;HAL_StatusTypeDef result;
    float target,velocity;uint8_t enabled;
    if(!motor || !out || __get_IPSR()!=0U || motor->active_mode!=DM_POS_MODE || !timeout_ms || timeout_ms>500U)return HAL_ERROR;
    key=__get_PRIMASK();__disable_irq();
    if(motor->enable_pending || motor->resend_hold){__set_PRIMASK(key);return HAL_BUSY;}
    enabled=(uint8_t)(motor->state==DM_STATE_ENABLED && motor->enable_sent);
    if(!enabled && motor->fb.enabled){__set_PRIMASK(key);return HAL_BUSY;} /* hardware state not reconciled */
    target=motor->target_pos_rad;velocity=motor->target_vel_rads;
    if(enabled && (!isfinite(target) || !isfinite(velocity) || fabsf(target)>motor->cfg.pos_limit_rad ||
        velocity<=0.0f || fabsf(velocity)>motor->cfg.vel_limit_rad)) {__set_PRIMASK(key);return HAL_ERROR;}
    motor->enable_pending=1U;motor->resend_hold=1U;__set_PRIMASK(key);
    DM_DrainRxBeforeEnable(motor);DM_ReadFeedbackSnapshot(motor,&fb);baseline=fb.fb_count;
    result=enabled?DM_SendPosFrame(motor,target,velocity):DM_SendSpecialCmd(motor,0xFDU);
    start=HAL_GetTick();
    if(result==HAL_OK) {
        result=HAL_TIMEOUT;
        while(HAL_GetTick()-start<timeout_ms) {
            if(!motor->enable_pending){result=HAL_ERROR;break;}
            DM_ReadFeedbackSnapshot(motor,&fb);
            if(fb.fb_count!=baseline && isfinite(fb.pos_rad) && isfinite(fb.vel_rads) &&
                HAL_GetTick()-fb.last_tick<=DM_LINK_TIMEOUT_MS_DEFAULT){result=HAL_OK;break;}
            DM_WaitYield(motor,1U);
        }
    }
    key=__get_PRIMASK();__disable_irq();
    if(!motor->enable_pending && result==HAL_OK)result=HAL_ERROR;
    motor->enable_pending=0U;motor->resend_hold=0U;motor->last_err=(uint8_t)result;__set_PRIMASK(key);
    if(result==HAL_OK)*out=fb;
    return result;
}
HAL_StatusTypeDef DM_Motor_SavePosZero(dm_motor_t *motor)
{
    dm_feedback_t fb;
    if (motor == NULL || __get_IPSR()!=0U)
    {
        return HAL_ERROR;
    }
    /* ⚠ 唯一会改电机 Flash 的操作：把当前位置永久设为零点。
     * 仅显式 save/confirm 流程触发；必须新反馈确认失能和静止。 */
    DM_ReadFeedbackSnapshot(motor,&fb);
    if (motor->enable_sent != 0U || motor->enable_pending || motor->active_mode!=DM_POS_MODE ||
        !fb.fb_count || fb.enabled || fb.err_code!=DM_FB_STATE_DISABLED ||
        HAL_GetTick()-fb.last_tick>DM_LINK_TIMEOUT_MS_DEFAULT || !isfinite(fb.pos_rad) ||
        !isfinite(fb.vel_rads) || fabsf(fb.vel_rads)>0.05f)
    {
        return HAL_ERROR;
    }
    return DM_SendSpecialCmd(motor, 0xFEU);
}

/* ================= 控制目标 ================= */

HAL_StatusTypeDef DM_Motor_SetMIT(dm_motor_t *motor, float pos_rad, float vel_rads,
                                  float kp, float kd, float tor_nm)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    if (motor->fault_latched != 0U)
    {
        return HAL_ERROR;
    }
    DM_Motor_SetMode(motor, DM_MIT_MODE);
    motor->target_pos_rad = DM_Clamp(pos_rad, DM_P_MAX);
    motor->target_vel_rads = DM_Clamp(vel_rads, DM_V_MAX);
    DM_SendMitFrame(motor, motor->target_pos_rad, motor->target_vel_rads,
                    kp, kd, tor_nm);
    return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_SetPos(dm_motor_t *motor, float pos_rad, float vel_rads)
{
    float raw_target;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    if (motor->fault_latched != 0U)
    {
        return HAL_ERROR;
    }
    DM_Motor_SetMode(motor, DM_POS_MODE);
    /* 用户坐标 → 原始坐标：位置模式下控制帧发的是电机自己的坐标系，
     * 而反馈的 pos 也是同一坐标系（原始编码器映射）。`dm zerohere` 记下的偏移
     * 让"用户 0"落在当前位置上，所以下发值 = 用户值 + 偏移。 */
    raw_target = pos_rad + motor->pos_offset_rad;
    /* 软限位：先夹紧再下发，别把越界值交给电机去映射 */
    motor->target_pos_rad = DM_Clamp(raw_target, motor->cfg.pos_limit_rad);
    motor->target_vel_rads = DM_Clamp(vel_rads, motor->cfg.vel_limit_rad);
    DM_SendPosFrame(motor, motor->target_pos_rad, motor->target_vel_rads);
    return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_SetSpeed(dm_motor_t *motor, float vel_rads)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    if (motor->fault_latched != 0U)
    {
        return HAL_ERROR;
    }
    DM_Motor_SetMode(motor, DM_SPD_MODE);
    motor->target_vel_rads = DM_Clamp(vel_rads, motor->cfg.vel_limit_rad);
    DM_SendSpdFrame(motor, motor->target_vel_rads);
    return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_SetPSI(dm_motor_t *motor, float pos_rad, float vel_rads,
                                  float cur_a)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
#if DM_BUS_USE_FD_PSI
    DM_Motor_SetMode(motor, DM_PSI_MODE);
    motor->target_pos_rad = DM_Clamp(pos_rad, DM_P_MAX);
    motor->target_vel_rads = DM_Clamp(vel_rads, DM_V_MAX);
    DM_SendPsiFrame(motor, motor->target_pos_rad, motor->target_vel_rads, cur_a);
    return HAL_OK;
#else
    /* 经典 CAN 下 PSI 需要 12 字节 FD 帧，不可用。 */
    (void)pos_rad;
    (void)vel_rads;
    (void)cur_a;
    return HAL_ERROR;
#endif
}

void DM_Motor_ClearCtrl(dm_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
}

/* ================= 周期服务 ================= */

uint8_t DM_IsBusOff(const dm_motor_t *motor)
{
    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return 0U;
    }
    return (uint8_t)((DM_ReadPsr(motor->cfg.can) & FDCAN_PSR_BO) != 0U);
}

/**
  * @brief  退出 bus-off：置 CCCR.INIT（停止）→ 清空 TX 缓冲 → 清 CCCR.INIT（重启）。
  *
  * 为什么不能只靠 HAL：HAL 没有"退出 bus-off"的 API。控制器在 bus-off 状态下
  * TX 被禁止，而 HAL_FDCAN_AddMessageToTxFifoQ() 只检查 TXFQS.TFQF，于是每一帧都
  * 报 FIFO_FULL —— 看起来像"FIFO 满了"，其实是"控制器停了"。
  * 这里同时**清空 TX 缓冲**，因为 bus-off 之前排进去的帧还在，不清掉会把 FIFO 继续占着。
  *
  * 恢复后电机处于"未使能"：不带着力矩凭空恢复，必须重新 `dm enable`（安全侧重）。
  */
static HAL_StatusTypeDef DM_BusOffRecover(dm_motor_t *motor)
{
    FDCAN_ProtocolStatusTypeDef protocol;
    uint32_t guard;

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return HAL_ERROR;
    }

    motor->enable_pending = 0U;
    motor->resend_hold = 1U;
    (void)HAL_FDCAN_Stop(motor->cfg.can);       /* 置 CCCR.INIT 并等待确认 */

    /* 清掉 bus-off 前残留的发送请求，否则 FIFO 仍被占着 */
    motor->cfg.can->Instance->TXBCR = 0xFFFFFFFFU;

    if (HAL_FDCAN_Start(motor->cfg.can) != HAL_OK)
    {
        motor->resend_hold = 0U;
        return HAL_ERROR;
    }

    /* 等控制器真正离开 bus-off（RX 上出现 128 个连续隐性位后才算退出） */
    guard = HAL_GetTick();
    while ((HAL_GetTick() - guard) < 50U)
    {
        if ((DM_ReadPsr(motor->cfg.can) & FDCAN_PSR_BO) == 0U)
        {
            break;
        }
        DM_WaitYield(motor,1U);
    }
    (void)memset(&protocol, 0, sizeof(protocol));
    (void)DM_GetProtocolStatus(motor->cfg.can, &protocol);

    /* 状态复位：FIFO/错误计数已清，使能语义作废 */
    motor->tx_fail_streak = 0U;
    motor->can_error_flags = 0U;
    motor->bus_off_latched = 0U;
    motor->bus_off_recover_count++;
    motor->enable_sent = 0U;
    if (motor->state != DM_STATE_RESET)
    {
        motor->state = DM_STATE_READY;
        motor->fault_latched = 0U;
    }
    motor->fb.fb_count = 0U;
    motor->fb.last_tick = 0U;
    motor->last_rx_tick = 0U;
    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();

    return (protocol.BusOff != 0U) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef DM_Motor_Recover(dm_motor_t *motor)
{
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return HAL_ERROR;
    }

    /* 重建控制器 = 清 TX FIFO / RX FIFO / 错误计数器 / CCCR.INIT。
     * 这是"TX FIFO 卡死"和"bus-off"的共同出路。 */
    motor->enable_pending = 0U;
    motor->resend_hold = 1U;
    (void)HAL_FDCAN_DeInit(motor->cfg.can);
    status = DM_BusConfigure(motor, FDCAN_MODE_NORMAL);

    motor->tx_fail_streak = 0U;
    motor->can_error_flags = 0U;
    motor->bus_off_count = 0U;
    motor->bus_off_latched = 0U;
    motor->bus_off_recover_count = 0U;
    motor->fb.fb_count = 0U;
    motor->fb.last_tick = 0U;
    motor->rx_count = 0U;
    motor->rx_other_count = 0U;
    motor->enable_sent = 0U;      /* 控制器重建后使能状态作废，需重新 enable */
    motor->enable_pending = 0U;
    motor->target_pos_rad = 0.0f;
    motor->target_vel_rads = 0.0f;
    if (motor->state == DM_STATE_FAULT)
    {
        motor->fault_latched = 0U;
    }
    motor->state = DM_STATE_READY;

    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();
    motor->last_rx_tick = 0U;

    return status;
}

uint8_t DM_IsTxBlocked(const dm_motor_t *motor)
{
    return ((motor != NULL) && (motor->tx_fail_streak != 0U)) ? 1U : 0U;
}

void DM_Motor_EmergencyStop(dm_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    {
        uint32_t key=__get_PRIMASK();__disable_irq();motor->stop_generation++;__set_PRIMASK(key);
    }
    DM_Motor_ClearCtrl(motor);
    if ((motor->enable_sent != 0U) || (motor->enable_pending != 0U))
    {
        (void)DM_Motor_Disable(motor);
        DM_Print("[DM] EMERGENCY STOP: disable requested; release not confirmed "
                 "without feedback.\r\n");
    }
    motor->fault_latched = 1U;
    motor->state = DM_STATE_FAULT;
}

void DM_Motor_Service(dm_motor_t *motor, uint32_t now_ms)
{
    if (motor == NULL)
    {
        return;
    }

    /* ---- 0. bus-off 处理（必须排在最前面） ----
     * 控制器一旦 bus-off，TX 就被禁止，而 HAL 只报 FIFO_FULL —— 于是 100 Hz 重发
     * 会永久失败、每 50 条打印一次，把控制台刷哑（台架实测刷了 1100+ 条）。
     * 这里在任务上下文里做两件事：锁存计数，并（默认）自动恢复。 */
    if (motor->bus_off_latched != 0U)
    {
        motor->enable_pending = 0U;
        motor->bus_off_latched = 0U;
        motor->bus_off_count++;
        motor->bus_off_tick = now_ms;
        DM_Print("[DM] FDCAN1 BUS-OFF: TX is disabled by the controller and it does NOT\r\n"
                 "     leave this state by itself. Recovering automatically; disable was\r\n"
                 "     requested and release is not confirmed without feedback.\r\n");

#if (DM_BUSOFF_AUTO_RECOVER != 0U)
        (void)DM_BusOffRecover(motor);
        DM_Print("[DM] bus-off recovery done. Check CANH/CANL, 120R and the motor supply,\r\n"
                 "     then `dm probe`.\r\n");
#else
        DM_Print("[DM] auto-recovery is compiled out (DM_BUSOFF_AUTO_RECOVER=0).\r\n"
                 "     Run `dm recover` manually, or `dm brscan` to test other rates.\r\n");
#endif
        return;
    }

    /* ---- 0b. 已在 bus-off 但还没被处理：跳过发送，别再往残废的 TX 里塞帧 ----
     * 这一条是"刷屏"的直接止血：bus-off 期间一帧都不发。 */
    if ((DM_ReadPsr(motor->cfg.can) & FDCAN_PSR_BO) != 0U)
    {
        motor->enable_pending = 0U;
        motor->bus_off_latched = 1U;
        return;
    }

    /* A console task may be committing a new target/state as part of the
     * enable handshake.  The volatile gate is set before those writes and is
     * cleared only after the final state is complete. */
    if (motor->enable_pending != 0U)
    {
        return;
    }
    DM_Motor_DiagPoll(motor,now_ms);

    /* ---- 1. 可选：周期重发 ----
     * 放在失联判据**之前**：重发是"主动去问"，但只有后续收到合法反馈时
     * RX ISR 才刷新 last_rx_tick；发送本身不能伪造链路健康。
     *
     * 为什么默认要重发（2026-09-14 实测结论）：达妙电机收到一条位置指令只回一帧
     * 反馈，之后**不会自发持续上报**。所以"使能后收不到反馈就松劲"这个保护，
     * 在不重发时必然误触发。重发同时让失联检测变成真正的链路检测。 */
    if ((motor->state == DM_STATE_ENABLED) &&
        (motor->resend_enable != 0U) &&
        (motor->resend_hold == 0U) &&
        ((int32_t)(now_ms - motor->resend_next_tick) >= 0))
    {
        motor->resend_next_tick = now_ms + DM_RESEND_PERIOD_MS;
        switch (motor->active_mode)
        {
            case DM_MIT_MODE:
                DM_SendMitFrame(motor, motor->target_pos_rad,
                                motor->target_vel_rads, 0.0f, 0.0f, 0.0f);
                break;
            case DM_SPD_MODE:
                DM_SendSpdFrame(motor, motor->target_vel_rads);
                break;
            case DM_POS_MODE:
            default:
                DM_SendPosFrame(motor, motor->target_pos_rad,
                                motor->target_vel_rads);
                break;
        }
    }

    /* ---- 2. 失去控制 → 直接松（用户 2026-09-14 要求） ----
     * 判据：已使能 && 超时时间内既没有反馈、也没有刚发过重发。 */
    if ((motor->state == DM_STATE_ENABLED) &&
        (motor->cfg.link_timeout_ms != 0U) &&
        (motor->fb.fb_count != 0U) &&
        ((int32_t)(now_ms - motor->last_rx_tick) >
         (int32_t)motor->cfg.link_timeout_ms))
    {
        motor->link_lost_count++;
        /* 先失能（松劲）再打印：打印在 9600 baud 下要几十毫秒，
         * 顺序反了会让电机多保持几十毫秒的出力。 */
        (void)DM_Motor_Disable(motor);
        motor->state = DM_STATE_FAULT;
        motor->fault_latched = 1U;
        DM_Print("[DM] link lost: disable requested; release not confirmed without feedback.\r\n");
        return;
    }

    /* ---- 2. 电机自报故障码非 0/1 时失能并提示 ----
     * 状态码语义（达妙协议）：0 = 失能，1 = 使能，8~E = 各种保护
     * （过压/欠压/过流/过温/编码器/通讯等）。
     * 这里只失能 + 提示，不自动清故障（清故障由 ClearError 显式触发）。 */
    if ((motor->state == DM_STATE_ENABLED) && (motor->fb.err_code > DM_FB_STATE_ENABLED))
    {
        char line[96];

        motor->fault_latched = 1U;
        (void)snprintf(line, sizeof(line),
                       "[DM] motor fault code %u (0x%X): DISABLED. Use dm clear "
                       "after checking the cause.\r\n",
                       (unsigned int)motor->fb.err_code,
                       (unsigned int)motor->fb.err_code);
        DM_Print(line);
        (void)DM_Motor_Disable(motor);
        motor->state = DM_STATE_FAULT;
        motor->fault_latched = 1U;
        return;
    }
}

/* ================= 状态读取 ================= */

void DM_SetResend(dm_motor_t *motor, uint8_t enable)
{
    if (motor == NULL)
    {
        return;
    }
    motor->resend_enable = (enable != 0U) ? 1U : 0U;
    motor->resend_next_tick = HAL_GetTick();
}

uint8_t DM_GetResend(const dm_motor_t *motor)
{
    return (motor != NULL) ? motor->resend_enable : 0U;
}

float DM_GetPosRad(const dm_motor_t *motor)     { return (motor != NULL) ? motor->fb.pos_rad : 0.0f; }
float DM_GetVelRads(const dm_motor_t *motor)    { return (motor != NULL) ? motor->fb.vel_rads : 0.0f; }
float DM_GetTorqueNm(const dm_motor_t *motor)   { return (motor != NULL) ? motor->fb.tor_nm : 0.0f; }
float DM_GetMosTempC(const dm_motor_t *motor)   { return (motor != NULL) ? motor->fb.t_mos_c : 0.0f; }
float DM_GetCoilTempC(const dm_motor_t *motor)  { return (motor != NULL) ? motor->fb.t_coil_c : 0.0f; }
uint8_t DM_GetErrCode(const dm_motor_t *motor)  { return (motor != NULL) ? motor->fb.err_code : 0U; }
uint32_t DM_GetFbCount(const dm_motor_t *motor) { return (motor != NULL) ? motor->fb.fb_count : 0U; }
uint32_t DM_GetLinkLostCount(const dm_motor_t *motor) { return (motor != NULL) ? motor->link_lost_count : 0U; }
dm_state_e DM_GetState(const dm_motor_t *motor) { return (motor != NULL) ? motor->state : DM_STATE_RESET; }

uint8_t DM_IsMotorReportingEnabled(const dm_motor_t *motor)
{
    return (motor != NULL) ? motor->fb.enabled : 0U;
}

uint8_t DM_IsEnabled(const dm_motor_t *motor)
{
    if (motor == NULL)
    {
        return 0U;
    }
    return ((motor->state == DM_STATE_ENABLED) && (motor->fault_latched == 0U)) ? 1U : 0U;
}

uint8_t DM_IsOnline(const dm_motor_t *motor, uint32_t timeout_ms)
{
    if (motor == NULL)
    {
        return 0U;
    }
    if (motor->fb.fb_count == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - motor->fb.last_tick) <= timeout_ms) ? 1U : 0U;
}

void DM_GetFeedback(const dm_motor_t *motor, dm_feedback_t *out)
{
    if ((motor == NULL) || (out == NULL))
    {
        return;
    }
    DM_ReadFeedbackSnapshot(motor, out);
}

void DM_GetConfig(const dm_motor_t *motor, dm_config_t *out)
{
    if ((motor == NULL) || (out == NULL))
    {
        return;
    }
    *out = motor->cfg;
}

/* ================= 控制台命令 ================= */

/**
  * @brief  回环自检：把"控制器/时钟"和"收发器/接线"分开。
  *
  * 背景：本车台架第一次实测得到 `LEC=5`（bit0 错误）+ bus-off + frames=0。
  * bit0 错误有两个完全不同的成因族，靠猜会来回折腾：
  *   (a) 控制器/时钟/位时序问题
  *   (b) 收发器供电、CANH/CANL 接反、共地、终端电阻
  * 回环模式能把两者切开，而且**不需要第二个节点**：
  *
  *   内部回环（TEST.LBCK=1 且 CCCR.MON=1，HAL 的 FDCAN_MODE_INTERNAL_LOOPBACK）：
  *     TX 在芯片内部直接接回 RX，**引脚和收发器都不参与，也不需要 ACK**。
  *     → 通过 = 控制器 + 内核时钟 + 位时序 + RAM + 过滤器全都正常，
  *       那么线上的 LEC 错误只能是 (b)。
  *     → 失败 = 问题在芯片侧（时钟/配置），别去拆线。
  *
  *   外部回环（只置 TEST.LBCK，HAL 的 FDCAN_MODE_EXTERNAL_LOOPBACK）：
  *     TX 引脚真的驱动出去，走一遍收发器，再从 RX 引脚收回来。
  *     → 通过 = 收发器供电/方向正常、引脚复用正确（但仍不证明对端电机在线）。
  *     → 失败而内部回环通过 = 收发器或引脚/供电问题。
  */
static void DM_Console_RunLoopback(dm_motor_t *motor, uint8_t external)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    uint8_t payload[8] = { 0xA5U, 0x5AU, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U };
    uint32_t mode = external ? FDCAN_MODE_EXTERNAL_LOOPBACK : FDCAN_MODE_INTERNAL_LOOPBACK;
    uint32_t start;
    uint32_t rx_count_before;
    uint32_t rx_other_before;
    uint8_t received = 0U;
    char line[160];

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        DM_Print("[DM] loopback: no bus bound\r\n");
        return;
    }

    /* 诊断期间抑制周期重发：它会往同一条总线上插帧，干扰回环的判定。 */
    motor->resend_hold = 1U;

    (void)snprintf(line, sizeof(line), "[DM] loopback test (%s), no second node needed...\r\n",
                   external ? "EXTERNAL: drives the pins through the transceiver"
                            : "INTERNAL: chip-internal TX->RX, transceiver not involved");
    DM_Print(line);

    /* 以回环模式重新初始化 FDCAN1（会自动重新落位 RAM 偏移/过滤器/启动 RX） */
    if (DM_InitLoopback(motor, mode) != HAL_OK)
    {
        DM_Print("[DM] loopback: HAL_FDCAN_Init failed; cannot run the test\r\n");
        (void)DM_ReinitNormal(motor);
        return;
    }

    /* 发一帧。ID 用 0x7FF 是故意的：它是标准 ID 的最大值，如果这一位
     * 被映射错，回环也会失败 —— 顺便测了 ID 位。
     * 注意 0x7FF != mst_id(0x11)，所以 ISR 会把它算进 rx_other_count 而不是 rx_count，
     * 两个计数都要作为"收到了"的判据。 */
    rx_count_before = motor->rx_count;
    rx_other_before = motor->rx_other_count;
    (void)DM_SendFrame(motor, 0x7FFU, payload, 8U);

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 200U)
    {
        (void)memset(&rx, 0, sizeof(rx));
        if (HAL_FDCAN_GetRxMessage(motor->cfg.can, FDCAN_RX_FIFO0, &rx, data) == HAL_OK)
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] loopback RX: SID=0x%03lX dlc=%lu data=%02X %02X %02X %02X "
                           "%02X %02X %02X %02X\r\n",
                           (unsigned long)rx.Identifier,
                           (unsigned long)((rx.DataLength >> 16U) & 0x0FU),
                           data[0], data[1], data[2], data[3],
                           data[4], data[5], data[6], data[7]);
            DM_Print(line);
            received = 1U;
            break;
        }
        /* 内部回环下这个循环还在等的时候，**RX 中断也会触发**（DM_Motor_Init 打开了
         * RX_FIFO0_WATERMARK，这里没关掉它），ISR 里的 DM_RxFifo0Handler 会把帧抢先
         * 读走 —— 那种情况下本循环永远读不到，会把"通过"误报成"失败"。
         * 所以复查两个计数：任一增加就说明帧确实收到了。 */
        if ((motor->rx_count > rx_count_before) ||
            (motor->rx_other_count > rx_other_before))
        {
            DM_Print("[DM] loopback RX: frame was consumed by the RX interrupt "
                     "(counted as PASS)\r\n");
            received = 1U;
            break;
        }
        DM_WaitYield(motor,1U);
    }

    /* 回到正常总线模式：这一步失败的话电机也回不来，必须显式报告 */
    if (DM_ReinitNormal(motor) != HAL_OK)
    {
        motor->resend_hold = 0U;
        DM_Print("[DM] ERROR: could not return to normal mode; re-init the board\r\n");
        return;
    }

    if (received != 0U)
    {
        if (external != 0U)
        {
            DM_Print("[DM] loopback EXTERNAL = PASS. Transceiver and pin mux are fine,\r\n"
                     "     so a bit0 error on the real bus points at the OTHER node:\r\n"
                     "     CANH/CANL polarity, the motor's baud rate, common ground, or\r\n"
                     "     termination - not at this board.\r\n");
        }
        else
        {
            DM_Print("[DM] loopback INTERNAL = PASS. Controller, clock, bit timing, RAM\r\n"
                     "     and filter are all fine. Now run `dm loopback ext` to test the\r\n"
                     "     transceiver, then check CANH/CANL polarity and the motor side.\r\n");
        }
    }
    else
    {
        if (external != 0U)
        {
            DM_Print("[DM] loopback EXTERNAL = FAIL. If `dm loopback` (internal) passed,\r\n"
                     "     the fault is the transceiver, its supply, or the pin mux -\r\n"
                     "     not the wiring to the motor.\r\n");
        }
        else
        {
            DM_Print("[DM] loopback INTERNAL = FAIL. This does NOT involve the transceiver\r\n"
                     "     or the wiring at all, so stop checking CANH/CANL: the fault is\r\n"
                     "     in the FDCAN1 config or the kernel clock (PLL2Q).\r\n");
        }
    }

    /* 恢复周期重发并重置失联判据起点。 */
    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();
    motor->last_rx_tick = 0U;
}

/**
  * @brief  波特率扫描：依次用每个经典 CAN 波特率向 0x101 发触发帧，看谁能收到反馈。
  *
  * 为什么需要它：本车电机的寄存器 RID_CAN_BR 读出来是 0xFFFFFFFF（未写入），
  * 实际波特率未知；而"对端波特率不匹配"正是 LEC=5(bit0) + bus-off 的典型成因。
  * 用第三方 USB-CAN 工具去猜波特率很慢，而这台机器自己就能扫 —— 每个候选速率
  * 只用很短时间，谁能通就是谁。
  *
  * 判据：只要收到**任何**标准 ID 帧就算命中（不要求 matched=1，因为 ID 也可能不对）。
  * 结束时回到原波特率，避免把现场留在一个"试过的"速率上。
  */
static void DM_Console_BitrateScan(dm_motor_t *motor)
{
    static const uint8_t codes[] = {
        DM_BITRATE_125K, DM_BITRATE_250K, DM_BITRATE_500K, DM_BITRATE_1M
    };
    uint8_t original = motor->cfg.bitrate_code;
    char line[160];
    uint8_t i;
    uint8_t found = 0U;

    if ((motor == NULL) || (motor->cfg.can == NULL))
    {
        return;
    }

    /* 扫描会反复重建 FDCAN1，期间必须停掉周期重发（否则帧会插进扫描判定里）。 */
    motor->resend_hold = 1U;

    DM_Print("[DM] bitrate scan: classic CAN 125K/250K/500K/1M, ~250 ms each.\r\n"
             "     A hit means the motor answered at that rate.\r\n");

    for (i = 0U; i < (uint8_t)(sizeof(codes) / sizeof(codes[0])); ++i)
    {
        uint32_t rx_before;
        uint32_t other_before;
        uint32_t start;
        uint8_t frames = 0U;

        if (DM_Motor_SetBitrate(motor, codes[i]) != HAL_OK)
        {
            DM_Print("[DM] scan: could not switch bitrate; aborting\r\n");
            (void)DM_Motor_SetBitrate(motor, original);
            return;
        }

        /* 这里**直接发底层位置帧**（绕过 DM_Motor_SetPos 的状态判据）：
         * 电机未使能时 DM_Motor_SetPos 会拒绝，而扫描恰恰要在未使能时做。 */
        rx_before = motor->rx_count;
        other_before = motor->rx_other_count;

        for (start = 0U; start < 3U; ++start)
        {
            DM_SendPosFrame(motor, DM_Clamp(motor->fb.pos_rad, motor->cfg.pos_limit_rad),
                            0.0f);
            {
                uint32_t wait = HAL_GetTick();

                while ((HAL_GetTick() - wait) < 60U)
                {
                    if ((motor->rx_count != rx_before) ||
                        (motor->rx_other_count != other_before))
                    {
                        frames = 1U;
                        break;
                    }
                    DM_WaitYield(motor,1U);
                }
            }
            if (frames != 0U)
            {
                break;
            }
        }

        (void)snprintf(line, sizeof(line), "[DM] %-4s -> %s\r\n",
                       DM_BitrateName(codes[i]), (frames != 0U) ? "*** FRAMES SEEN ***" : "no response");
        DM_Print(line);

        if (frames != 0U)
        {
            found = codes[i];
            break;
        }
    }

    if (found != 0U)
    {
        (void)snprintf(line, sizeof(line),
                       "[DM] scan result: motor answered at %s (code %u). "
                       "Left on this rate: run `dm probe` to confirm.\r\n",
                       DM_BitrateName(found), (unsigned int)found);
        DM_Print(line);
    }
    else
    {
        DM_Print("[DM] scan result: no rate produced a frame. The motor is either\r\n"
                 "     not powered, not wired to this bus, or set to a rate outside\r\n"
                 "     this list (FD data rates are not scanned here).\r\n");
        (void)DM_Motor_SetBitrate(motor, original);
        (void)snprintf(line, sizeof(line), "[DM] restored bitrate to %s\r\n",
                       DM_BitrateName(original));
        DM_Print(line);
    }

    /* 恢复周期重发并重置失联判据起点。 */
    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();
    motor->last_rx_tick = 0U;
}

static void DM_PrintStatus(const dm_motor_t *motor)
{
    char line[192];
    FDCAN_ErrorCountersTypeDef counters;
    FDCAN_ProtocolStatusTypeDef protocol;

    if ((motor == NULL) || (motor->cfg.can == NULL)) return;
    (void)snprintf(line, sizeof(line),
                   "[DM] CCCR=0x%08lX TXBRP=0x%08lX (DAR bit6: 1=no auto-retry; FDOE bit8: 1=FD)\r\n",
                   (unsigned long)motor->cfg.can->Instance->CCCR,
                   (unsigned long)motor->cfg.can->Instance->TXBRP);
    DM_Print(line);
    if ((HAL_FDCAN_GetErrorCounters(motor->cfg.can, &counters) == HAL_OK) &&
        (DM_GetProtocolStatus(motor->cfg.can, &protocol) == HAL_OK))
    {
        (void)snprintf(line, sizeof(line),
                       "[DM] CAN TEC=%lu REC=%lu RP=%lu LEC=%lu passive=%lu warning=%lu busoff=%lu\r\n",
                       (unsigned long)counters.TxErrorCnt, (unsigned long)counters.RxErrorCnt,
                       (unsigned long)counters.RxErrorPassive, (unsigned long)protocol.LastErrorCode,
                       (unsigned long)protocol.ErrorPassive, (unsigned long)protocol.Warning,
                       (unsigned long)protocol.BusOff);
        DM_Print(line);
    }
    else
    {
        DM_Print("[DM] CAN status read failed.\r\n");
    }
    (void)snprintf(line, sizeof(line),
                   "[DM] error history: last=%lu seen=0x%02lX (bits 1=stuff 2=form 3=ACK 4=bit1 5=bit0 6=CRC)\r\n",
                   (unsigned long)dm_lec_last, (unsigned long)dm_lec_seen);
    DM_Print(line);

    (void)snprintf(line, sizeof(line),
                   "[DM] state=%u esc=0x%02X mst=0x%02X mode=%u online=%u "
                   "en=%u(code=%u) pos=%.3f vel=%.3f tor=%.3f Tmos=%.0f Tcoil=%.0f "
                   "fb=%lu tx=%lu txfail=%lu lost=%lu\r\n",
                   (unsigned int)motor->state,
                   (unsigned int)motor->cfg.esc_id,
                   (unsigned int)motor->cfg.mst_id,
                   (unsigned int)motor->active_mode,
                   (unsigned int)DM_IsOnline(motor, DM_LINK_TIMEOUT_MS_DEFAULT),
                   (unsigned int)motor->fb.enabled,
                   (unsigned int)motor->fb.err_code,
                   (double)(motor->fb.pos_rad - motor->pos_offset_rad),
                   (double)motor->fb.vel_rads,
                   (double)motor->fb.tor_nm,
                   (double)motor->fb.t_mos_c,
                   (double)motor->fb.t_coil_c,
                   (unsigned long)motor->fb.fb_count,
                   (unsigned long)motor->tx_count,
                   (unsigned long)motor->tx_fail_count,
                   (unsigned long)motor->link_lost_count);
    DM_Print(line);

    /* 单独一行报"目标/偏移/重发/逐帧日志"这几个排障时最需要的量。
     * 位置模式"锁得住但不动"必须能看到 target 到底是多少、有没有在重发。 */
    (void)snprintf(line, sizeof(line),
                   "[DM] target=%.3f rad (user %.3f) offset=%.3f resend=%u txlog=%u "
                   "sanity=%d\r\n",
                   (double)motor->target_pos_rad,
                   (double)(motor->target_pos_rad - motor->pos_offset_rad),
                   (double)motor->pos_offset_rad,
                   (unsigned int)motor->resend_enable,
                   (unsigned int)motor->tx_log,
                   ((motor->target_pos_rad > motor->cfg.pos_limit_rad) ||
                    (motor->target_pos_rad < -motor->cfg.pos_limit_rad)) ? 1 : 0);
    DM_Print(line);

    /* 发送通道健康：txfail_streak>0 表示"当前发不出去"，此时任何控制都是空发。 */
    if (motor->tx_fail_streak != 0U)
    {
        (void)snprintf(line, sizeof(line),
                       "[DM] *** TX BLOCKED: %lu consecutive send failures (total %lu).\r\n"
                       "     Nothing you send is reaching the motor. Run `dm recover`.\r\n",
                       (unsigned long)motor->tx_fail_streak,
                       (unsigned long)motor->tx_fail_count);
        DM_Print(line);
    }
    if ((motor->bus_off_count != 0U) || (DM_IsBusOff(motor) != 0U))
    {
        (void)snprintf(line, sizeof(line),
                       "[DM] bus-off events=%lu auto-recovered=%lu now_busoff=%u\r\n",
                       (unsigned long)motor->bus_off_count,
                       (unsigned long)motor->bus_off_recover_count,
                       (unsigned int)DM_IsBusOff(motor));
        DM_Print(line);
    }

    if (motor->fb.temp_valid == 0U)
    {
        DM_Print("[DM] note: temperature fields read 0 -> this motor firmware may\r\n"
                 "     not report Tmos/Tcoil; ignore the two temperature numbers.\r\n");
    }
}

/* This HAL version does not implement FDCAN in GetPeriphCLKFreq(). */
static uint32_t DM_CanKernelClockHz(void)
{
    PLL1_ClocksTypeDef pll1;
    PLL2_ClocksTypeDef pll2;

    switch (__HAL_RCC_GET_FDCAN_SOURCE())
    {
        case RCC_FDCANCLKSOURCE_PLL:
            HAL_RCCEx_GetPLL1ClockFreq(&pll1);
            return pll1.PLL1_Q_Frequency;
        case RCC_FDCANCLKSOURCE_PLL2:
            HAL_RCCEx_GetPLL2ClockFreq(&pll2);
            return pll2.PLL2_Q_Frequency;
        case RCC_FDCANCLKSOURCE_HSE:
            return (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) ? HSE_VALUE : 0U;
        default:
            return 0U;
    }
}

/* Passive observation: RX IRQ remains the sole FIFO consumer. Normal motor
 * service and its safety timeout keep running; this command sends no frame. */
static void DM_Console_Probe(dm_motor_t *motor)
{
    FDCAN_ErrorCountersTypeDef before, after;
    FDCAN_ProtocolStatusTypeDef protocol;
    uint32_t matched_before, other_before, txfail_before, recover_before;
    uint32_t matched, other, nbtp, brp, seg1, seg2, clock_hz, start_tick;
    char line[160];

    if ((motor == NULL) || (motor->cfg.can == NULL)) return;
    nbtp = motor->cfg.can->Instance->NBTP;
    brp = ((nbtp & FDCAN_NBTP_NBRP) >> FDCAN_NBTP_NBRP_Pos) + 1U;
    seg1 = ((nbtp & FDCAN_NBTP_NTSEG1) >> FDCAN_NBTP_NTSEG1_Pos) + 1U;
    seg2 = ((nbtp & FDCAN_NBTP_NTSEG2) >> FDCAN_NBTP_NTSEG2_Pos) + 1U;
    clock_hz = DM_CanKernelClockHz();
    (void)snprintf(line, sizeof(line),
                   "[DM] NBTP=0x%08lX brp=%lu seg1=%lu seg2=%lu clock=%lu Hz rate=%lu bit/s\r\n",
                   (unsigned long)nbtp, (unsigned long)brp, (unsigned long)seg1,
                   (unsigned long)seg2, (unsigned long)clock_hz,
                   (unsigned long)(clock_hz / (brp * (1U + seg1 + seg2))));
    DM_Print(line);
    if (clock_hz == 0U)
        DM_Print("[DM] Clock unavailable/not ready; the displayed zero rate is not a measured bitrate.\r\n");
    DM_Print("[DM] probe: observe 300 ms; no probe TX. Normal motor service remains active.\r\n");
    matched_before = motor->rx_count;
    other_before = motor->rx_other_count;
    txfail_before = motor->tx_fail_count;
    recover_before = motor->bus_off_recover_count;
    if (HAL_FDCAN_GetErrorCounters(motor->cfg.can, &before) != HAL_OK)
    {
        DM_Print("[DM] probe: cannot read CAN error counters.\r\n");
        return;
    }
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < 300U) DM_WaitYield(motor,1U);
    if ((HAL_FDCAN_GetErrorCounters(motor->cfg.can, &after) != HAL_OK) ||
        (DM_GetProtocolStatus(motor->cfg.can, &protocol) != HAL_OK))
    {
        DM_Print("[DM] probe: cannot read CAN status.\r\n");
        return;
    }
    matched = motor->rx_count - matched_before;
    other = motor->rx_other_count - other_before;
    (void)snprintf(line, sizeof(line),
                   "[DM] probe: matched=%lu other_id=%lu tx_enqueue_fail_delta=%lu\r\n",
                   (unsigned long)matched, (unsigned long)other,
                   (unsigned long)(motor->tx_fail_count - txfail_before));
    DM_Print(line);
    (void)snprintf(line, sizeof(line),
                   "[DM] TEC=%lu->%lu REC=%lu->%lu RP=%lu->%lu\r\n",
                   (unsigned long)before.TxErrorCnt, (unsigned long)after.TxErrorCnt,
                   (unsigned long)before.RxErrorCnt, (unsigned long)after.RxErrorCnt,
                   (unsigned long)before.RxErrorPassive, (unsigned long)after.RxErrorPassive);
    DM_Print(line);
    (void)snprintf(line, sizeof(line),
                   "[DM] LEC=%lu passive=%lu warning=%lu busoff=%lu recover_delta=%lu\r\n",
                   (unsigned long)protocol.LastErrorCode, (unsigned long)protocol.ErrorPassive,
                   (unsigned long)protocol.Warning, (unsigned long)protocol.BusOff,
                   (unsigned long)(motor->bus_off_recover_count - recover_before));
    DM_Print(line);
    DM_Print("[DM] LEC: 0=none 1=stuff 2=form 3=ACK 4=bit1 5=bit0 6=CRC 7=no-change.\r\n");
    if (motor->bus_off_recover_count != recover_before)
        DM_Print("[DM] Controller recovered during observation; counters may have reset.\r\n");
    if ((protocol.BusOff != 0U) || (protocol.ErrorPassive != 0U) ||
        (protocol.Warning != 0U))
        DM_Print("[DM] VERDICT: CAN error state present, even if LEC=7 or feedback arrived.\r\n"
                 "     Check supply, bitrate, wiring and termination; compare after dm recover.\r\n");
    else if ((protocol.LastErrorCode > 0U) && (protocol.LastErrorCode < 7U))
        DM_Print("[DM] VERDICT: CAN error recorded; LEC alone does not date it to this probe.\r\n");
    else if (matched != 0U)
        DM_Print("[DM] VERDICT: matching feedback observed.\r\n");
    else if (other != 0U)
        DM_Print("[DM] VERDICT: other frames observed; verify motor command and feedback IDs.\r\n");
    else
        DM_Print("[DM] VERDICT: no feedback observed; passive observation cannot prove link failure.\r\n"
                 "     A disabled/non-reporting motor may be silent. LEC=7 is not a health check.\r\n");
}
/* `dm raw <id> <hex...>`：给任意 CAN ID 发任意字节，并把回包原样打出来。
 *
 * 为什么要有它：位置模式"锁得住但不动"这种故障，用上一层的 `dm pos` 完全看不出来
 * 是"帧没发出去"、"帧发错了"还是"电机不收"。直接发原始帧 + 打印回包，能一次说清：
 *   - 发 0x101 的 8 字节位置帧 → 有回包 = 电机认这条总线与 ID；
 *   - 用 0x101 发 `FF FF FF FF FF FF FF FC`（使能）→ 有回包 = 特殊命令帧也认；
 *   - 换 ID 试 → 能定位到底是哪个 ID 在起作用。
 * 同时它绕过所有状态判据（不要求使能），所以"什么都没通"时也能用。 */
static void DM_Console_SendRaw(dm_motor_t *motor, char **cursor)
{
    char *id_text = DM_NextToken(cursor);
    uint8_t data[8];
    uint8_t len = 0U;
    float id_value;
    uint32_t start;
    uint32_t matched_before;
    uint32_t other_before;
    char line[160];
    char *byte_text;

    if ((id_text == NULL) || (DM_ParseFloat(id_text, &id_value) == 0U) ||
        (id_value < 0.0f) || (id_value > 2047.0f))
    {
        DM_Print("[DM] usage: dm raw <can_id 0..2047> <b0> [b1] .. [b7]   (hex or dec)\r\n"
                 "     example: dm raw 0x101 0 0 0 0 0 0 0 0\r\n"
                 "     example: dm raw 0x101 255 255 255 255 255 255 255 252  (enable)\r\n");
        return;
    }

    while ((len < 8U) && ((byte_text = DM_NextToken(cursor)) != NULL))
    {
        char *end = NULL;
        long value = strtol(byte_text, &end, 0);   /* base 0: 接受 0x 前缀 */

        if ((end == byte_text) || (*end != '\0') || (value < 0L) || (value > 255L))
        {
            DM_Print("[DM] raw: each byte must be 0..255 (hex like 0xFF also ok)\r\n");
            return;
        }
        data[len++] = (uint8_t)value;
    }
    if (len == 0U)
    {
        DM_Print("[DM] raw: need at least one data byte\r\n");
        return;
    }

    motor->resend_hold = 1U;
    matched_before = motor->rx_count;
    other_before = motor->rx_other_count;

    (void)snprintf(line, sizeof(line), "[DM] raw TX id=0x%03lX len=%u ...\r\n",
                   (unsigned long)(uint16_t)id_value, (unsigned int)len);
    DM_Print(line);
    (void)DM_SendFrame(motor, (uint16_t)id_value, data, len);

    /* 监听 300 ms 并打印所有回包原文 */
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 300U)
    {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t rx_data[DM_FRAME_MAX_BYTES];

        (void)memset(&rx, 0, sizeof(rx));
        if (HAL_FDCAN_GetRxMessage(motor->cfg.can, FDCAN_RX_FIFO0, &rx, rx_data) == HAL_OK)
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] raw RX SID=0x%03lX dlc=%lu  %02X %02X %02X %02X "
                           "%02X %02X %02X %02X\r\n",
                           (unsigned long)rx.Identifier,
                           (unsigned long)((rx.DataLength >> 16U) & 0x0FU),
                           rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                           rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
            DM_Print(line);
        }
        if (!((motor->rx_count == matched_before) &&
              (motor->rx_other_count == other_before)))
        {
            break;
        }
        DM_WaitYield(motor,1U);
    }

    (void)snprintf(line, sizeof(line),
                   "[DM] raw done: replies match=%lu other=%lu "
                   "(counters also fed by the RX interrupt)\r\n",
                   (unsigned long)(motor->rx_count - matched_before),
                   (unsigned long)(motor->rx_other_count - other_before));
    DM_Print(line);

    motor->resend_hold = 0U;
    motor->resend_next_tick = HAL_GetTick();
    motor->last_rx_tick = 0U;
}

uint8_t DM_Console_ProcessCommand(dm_motor_t *motor, const char *command){
    char work[128];
    char *cursor;
    char *verb;
    char line[192];

    if ((motor == NULL) || (command == NULL))
    {
        return 0U;
    }
    /* 只认领以 "dm" 开头的命令；注意旧版用 strncmp(…, 2) 会把 "dmp" 也吃掉，
     * 这里要求 "dm" 后面紧跟 '\0' 或空格。 */
    if ((command[0] != 'd') || (command[1] != 'm'))
    {
        return 0U;
    }
    if ((command[2] != '\0') && (command[2] != ' '))
    {
        return 0U;
    }

    /* 拷到本地再切分：不改调用方的字符串（ZDT 那边还要复用命令行缓冲） */
    (void)strncpy(work, command, sizeof(work) - 1U);
    work[sizeof(work) - 1U] = '\0';
    cursor = work;
    (void)DM_NextToken(&cursor);            /* 丢掉 "dm" */
    verb = DM_NextToken(&cursor);

    if ((verb == NULL) || (strcmp(verb, "help") == 0) || (strcmp(verb, "?") == 0))
    {
        DM_Print("[DM] commands (Damiao turntable joint on FDCAN1):\r\n"
                 "  dm status            state / ids / pos / vel / tor / temps / counters\r\n"
                 "  dm probe             observe CAN for 300 ms (no probe control frame)\r\n"
                 "  dm loopback          INTERNAL loopback: proves controller+clock; the\r\n"
                 "                       transceiver and any second node are NOT involved\r\n"
                 "  dm loopback ext      EXTERNAL loopback: also drives the pins through\r\n"
                 "                       the transceiver\r\n"
                 "  dm brscan            sweep 125K/250K/500K/1M and report which one the\r\n"
                 "                       motor answers on (motor RID_CAN_BR is unset here)\r\n"
                 "  dm br <code>         set local bitrate: 0=125K 1=200K 2=250K 3=500K\r\n"
                 "                       4=1M 5=2M 6=2M5 7=3M2 8=4M 9=5M (RAM only)\r\n"
                  "  dm enable | disable  POS safe hold (fresh query + confirmation) /\r\n"
                  "                       disable requested (release needs feedback)\r\n"
                 "  dm clear             clear motor fault (0xFB)\r\n"
                 "  dm zero confirm      save current position as zero (writes motor flash!)\r\n"
                 "  dm pos <rad> [rad/s] position move. NOTE: <rad> is in the MOTOR's\r\n"
                 "                       own frame, same as the pos in dm status; the\r\n"
                 "                       velocity must be non-zero (default 2.0 rad/s)\r\n"
                  "  dm zerohere          set a RAM soft zero at fresh CURRENT position\r\n"
                 "                       (does not touch motor flash); dm pos 0 = here\r\n"
                 "  dm target            show target / offset / resend / txlog\r\n"
                 "  dm raw <id> <b0..b7> send any frame, print every reply (raw bytes)\r\n"
                 "  dm recover           re-init FDCAN1: clears a wedged TX FIFO or\r\n"
                 "                       bus-off. First thing to try when TX keeps failing\r\n"
                 "  dm txlog 0|1         print every control frame with its payload\r\n"
                 "  dm speed <rad/s>     velocity mode (needs re-enable afterwards)\r\n"
                 "  dm mit <p> <v> <kp> <kd> <t>   MIT mixed control\r\n"
                 "  dm setid <mst> <esc> set feedback/command IDs (RAM only, no flash)\r\n"
                 "  dm resend 0|1        show/set periodic target resend at runtime\r\n");
        return 1U;
    }

    if (strcmp(verb, "status") == 0)
    {
        DM_PrintStatus(motor);
    }
    else if (strcmp(verb, "probe") == 0)
    {
        DM_Console_Probe(motor);
    }
    else if (strcmp(verb, "loopback") == 0)
    {
        /* `dm loopback` = 内部回环；`dm loopback ext` = 外部回环。
         * 这是台架 LEC=5 / bus-off 排障的关键工具：内部回环完全不碰收发器和接线，
         * 能把"芯片侧（时钟/配置）"和"物理层（收发器/接线/对端）"一刀切开。 */
        char *which = DM_NextToken(&cursor);

        if ((which != NULL) && (strcmp(which, "ext") == 0))
        {
            DM_Console_RunLoopback(motor, 1U);
        }
        else if ((which != NULL) && (strcmp(which, "int") != 0))
        {
            DM_Print("[DM] usage: dm loopback [ext]   (no argument = internal)\r\n");
        }
        else
        {
            DM_Console_RunLoopback(motor, 0U);
        }
    }
    else if (strcmp(verb, "brscan") == 0)
    {
        DM_Console_BitrateScan(motor);
    }
    else if (strcmp(verb, "br") == 0)
    {
        char *code_text = DM_NextToken(&cursor);
        float code_value;

        if ((code_text == NULL) || (DM_ParseFloat(code_text, &code_value) == 0U) ||
            (code_value < 0.0f) || (code_value > (float)DM_BITRATE_CODE_MAX) ||
            (code_value != (float)(uint8_t)code_value))
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] usage: dm br <0..%u>  (0=125K 1=200K 2=250K 3=500K "
                           "4=1M 5=2M 6=2M5 7=3M2 8=4M 9=5M)\r\n",
                           (unsigned int)DM_BITRATE_CODE_MAX);
            DM_Print(line);
        }
        else if (DM_Motor_SetBitrate(motor, (uint8_t)code_value) == HAL_OK)
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] local bitrate = %s (code %u). Now run `dm probe`.\r\n",
                           DM_BitrateName((uint8_t)code_value),
                           (unsigned int)code_value);
            DM_Print(line);
        }
        else
        {
            DM_Print("[DM] br failed: could not re-init FDCAN1\r\n");
        }
    }
    else if (strcmp(verb, "recover") == 0)
    {
        /* `dm recover`：重建 FDCAN1，冲掉卡死的 TX FIFO / bus-off。
         * 这是"发不出去、什么指令都没反应"时的第一件事。 */
        if (DM_Motor_Recover(motor) == HAL_OK)
        {
            DM_Print("[DM] bus recovered: TX/RX FIFO cleared, counters reset.\r\n"
                     "     The joint was released and is NOT enabled - send `dm enable`.\r\n");
        }
        else
        {
            DM_Print("[DM] recover FAILED: HAL_FDCAN_Init error; re-init the board.\r\n");
        }
    }
    else if (strcmp(verb, "raw") == 0)
    {
        DM_Console_SendRaw(motor, &cursor);
    }
    else if (strcmp(verb, "txlog") == 0)
    {
        char *value_text = DM_NextToken(&cursor);

        if (value_text == NULL)
        {
            (void)snprintf(line, sizeof(line), "[DM] txlog=%u; usage: dm txlog 0|1\r\n",
                           (unsigned int)motor->tx_log);
            DM_Print(line);
        }
        else if (strcmp(value_text, "0") == 0)
        {
            motor->tx_log = 0U;
            DM_Print("[DM] txlog off\r\n");
        }
        else if (strcmp(value_text, "1") == 0)
        {
            motor->tx_log = 1U;
            DM_Print("[DM] txlog ON: every control frame will be printed with its "
                     "payload.\r\n     Send `dm txlog 0` to stop (printing at 9600 "
                     "baud slows the\r\n     task down, so keep it short).\r\n");
        }
        else
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] txlog: got '%s' (not 0/1). NOTE: if you typed\r\n"
                           "     'dm txlog 1' and the argument went missing, the UART\r\n"
                           "     dropped bytes - check `status` for dropped!=0 and keep\r\n"
                           "     char-echo off (`uecho 0`). Text before that was ignored.\r\n",
                           value_text);
            DM_Print(line);
        }
    }
    else if (strcmp(verb, "zerohere") == 0)
    {
        /* `dm zerohere`：把**当前位置**设为位置模式的零点（软零点，不写电机 flash）。
         * 机制：位置模式下反馈的 pos 是"原始编码器值映射"，而控制帧的目标位置也是
         * 同一个坐标系（直接发 float），所以软零点就是记下当前 pos 作为偏移。 */
        if ((motor->state != DM_STATE_ENABLED) ||
            (motor->fb.enabled == 0U) ||
            (DM_IsOnline(motor, DM_LINK_TIMEOUT_MS_DEFAULT) == 0U))
        {
            DM_Print("[DM] zerohere refused: enable the motor and wait for feedback "
                     "first.\r\n");
        }
        else
        {
            motor->pos_offset_rad = motor->fb.pos_rad;
            motor->target_pos_rad = motor->fb.pos_rad;
            (void)snprintf(line, sizeof(line),
                           "[DM] soft zero set at raw pos %.3f rad. `dm pos 0` now "
                           "means HERE.\r\n", (double)motor->pos_offset_rad);
            DM_Print(line);
            DM_Print("[DM] note: this is a RAM soft-zero; it does not touch the motor "
                     "flash.\r\n");
        }
    }
    else if (strcmp(verb, "target") == 0)
    {
        (void)snprintf(line, sizeof(line),
                       "[DM] target=%.3f rad  offset=%.3f  resend=%u txlog=%u "
                       "tx=%lu\r\n",
                       (double)motor->target_pos_rad,
                       (double)motor->pos_offset_rad,
                       (unsigned int)motor->resend_enable,
                       (unsigned int)motor->tx_log,
                       (unsigned long)motor->tx_count);
        DM_Print(line);
    }
    else if (strcmp(verb, "enable") == 0)
    {
        if (motor->state == DM_STATE_RESET)
        {
            DM_Print("[DM] enable refused: bus not initialised (DM_Motor_Init failed)\r\n");
        }
        else if (DM_Motor_Enable(motor) == HAL_OK)
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] enable confirmed on POS control ID 0x%03X.\r\n",
                           (unsigned int)(motor->cfg.esc_id + DM_MODE_POS));
            DM_Print(line);
        }
        else
        {
            DM_Print("[DM] enable failed: TX FIFO rejected the frame\r\n");
        }
    }
    else if (strcmp(verb, "disable") == 0)
    {
        (void)DM_Motor_Disable(motor);
        DM_Print("[DM] disable requested; motor release is NOT confirmed without feedback.\r\n");
    }
    else if (strcmp(verb, "clear") == 0)
    {
        HAL_StatusTypeDef result=DM_Motor_ClearError(motor);
        if(result==HAL_OK)DM_Print("[DM] CLEAR confirmed: fresh disabled/no-fault feedback; READY and still OFF; retry dm home\r\n");
        else {
            (void)snprintf(line,sizeof(line),"[DM] CLEAR FAILED HAL=%u state=%u latch=%u; need fresh disabled/no-fault feedback, no confirmed recovery\r\n",
                (unsigned int)result,(unsigned int)motor->state,motor->fault_latched);DM_Print(line);
        }
    }
    else if (strcmp(verb, "zero") == 0)
    {
        /* 二次确认：这一步会写电机 Flash 改零点，误触发在台上不可逆。 */
        char *confirm = DM_NextToken(&cursor);

        if ((confirm == NULL) || (strcmp(confirm, "confirm") != 0))
        {
            DM_Print("[DM] zero refused: this OVERWRITES the motor's zero point in its\r\n"
                     "     own flash. If you really mean it, send: dm zero confirm\r\n");
        }
        else if (DM_Motor_SavePosZero(motor) == HAL_OK)
        {
            DM_Print("[DM] save-zero sent (0xFE). Motor will reboot its zero.\r\n");
        }
        else
        {
            DM_Print("[DM] zero refused: disable the motor first (dm disable).\r\n");
        }
    }
    else if (strcmp(verb, "pos") == 0)
    {
        char *pos_text = DM_NextToken(&cursor);
        char *vel_text;
        float pos;
        float vel = DM_VEL_DEFAULT_RAD;

        if ((pos_text == NULL) || (DM_ParseFloat(pos_text, &pos) == 0U))
        {
            DM_Print("[DM] usage: dm pos <rad> [rad/s]\r\n");
        }
        else
        {
            vel_text = DM_NextToken(&cursor);
            if ((vel_text != NULL) && (DM_ParseFloat(vel_text, &vel) == 0U))
            {
                DM_Print("[DM] usage: dm pos <rad> [rad/s]  (velocity must be a number)\r\n");
                return 1U;
            }
            if ((pos < -DM_P_MAX) || (pos > DM_P_MAX) ||
                (vel < 0.0f) || (vel > DM_V_MAX))
            {
                (void)snprintf(line, sizeof(line),
                               "[DM] refused: pos must be in [-%.1f, %.1f] rad, "
                               "vel in [0, %.1f] rad/s\r\n",
                               (double)DM_P_MAX, (double)DM_P_MAX, (double)DM_V_MAX);
                DM_Print(line);
                return 1U;
            }
            /* 电机位置模式下，控制帧的目标位置与反馈的 pos 是**同一个坐标系**
             * （都是编码器原始映射），所以 `dm pos` 是"原始坐标下的绝对位置"。
             * 想看/设定相对当前的位置，用 `dm zerohere` 立一个软零点，
             * 之后 `dm pos 0` 就是"回到刚才那里"。 */
            if ((pos + motor->pos_offset_rad > motor->cfg.pos_limit_rad) ||
                (pos + motor->pos_offset_rad < -motor->cfg.pos_limit_rad))
            {
                (void)snprintf(line, sizeof(line),
                               "[DM] refused: %.3f rad + offset %.3f exceeds the "
                               "+/-%.1f rad range.\r\n"
                               "     Use `dm zerohere` to re-base the soft zero, or "
                               "smaller steps.\r\n",
                               (double)pos, (double)motor->pos_offset_rad,
                               (double)motor->cfg.pos_limit_rad);
                DM_Print(line);
                return 1U;
            }
            if (motor->state != DM_STATE_ENABLED)
            {
                DM_Print("[DM] pos refused: motor is not enabled (send dm enable first).\r\n");
                return 1U;
            }
            (void)DM_Motor_SetPos(motor, pos, vel);
            (void)snprintf(line, sizeof(line),
                           "[DM] pos %.3f rad (raw %.3f) @ %.2f rad/s sent to 0x%03X\r\n",
                           (double)pos,
                           (double)(pos + motor->pos_offset_rad),
                           (double)vel,
                           (unsigned int)(motor->cfg.esc_id + DM_MODE_POS));
            DM_Print(line);
        }
    }
    else if (strcmp(verb, "speed") == 0)
    {
        char *vel_text = DM_NextToken(&cursor);
        float vel;

        if ((vel_text == NULL) || (DM_ParseFloat(vel_text, &vel) == 0U))
        {
            DM_Print("[DM] usage: dm speed <rad/s>\r\n");
        }
        else if ((vel < -DM_V_MAX) || (vel > DM_V_MAX))
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] refused: speed must be in [-%.0f, %.0f] rad/s\r\n",
                           (double)DM_V_MAX, (double)DM_V_MAX);
            DM_Print(line);
        }
        else if (motor->state != DM_STATE_ENABLED)
        {
            DM_Print("[DM] speed refused: motor is not enabled (send dm enable first).\r\n");
        }
        else
        {
            (void)DM_Motor_SetSpeed(motor, vel);
            DM_Print("[DM] speed sent. NOTE: mode changed -> send dm enable again "
                     "before any further dm pos.\r\n");
        }
    }
    else if (strcmp(verb, "mit") == 0)
    {
        char *texts[5];
        float values[5];
        uint8_t i;
        uint8_t ok = 1U;

        for (i = 0U; i < 5U; ++i)
        {
            texts[i] = DM_NextToken(&cursor);
        }
        for (i = 0U; (i < 5U) && (ok != 0U); ++i)
        {
            if ((texts[i] == NULL) || (DM_ParseFloat(texts[i], &values[i]) == 0U))
            {
                ok = 0U;
            }
        }
        if (ok == 0U)
        {
            DM_Print("[DM] usage: dm mit <pos_rad> <vel_rad/s> <kp> <kd> <tor_nm>\r\n");
        }
        else if (motor->state != DM_STATE_ENABLED)
        {
            DM_Print("[DM] mit refused: motor is not enabled (send dm enable first).\r\n");
        }
        else
        {
            (void)DM_Motor_SetMIT(motor, values[0], values[1], values[2], values[3], values[4]);
            DM_Print("[DM] MIT frame sent. NOTE: mode changed -> re-enable before dm pos.\r\n");
        }
    }
    else if (strcmp(verb, "setid") == 0)
    {
        char *mst_text = DM_NextToken(&cursor);
        char *esc_text = DM_NextToken(&cursor);
        float mst_f;
        float esc_f;

        if ((mst_text == NULL) || (esc_text == NULL) ||
            (DM_ParseFloat(mst_text, &mst_f) == 0U) ||
            (DM_ParseFloat(esc_text, &esc_f) == 0U) ||
            (mst_f < 0.0f) || (mst_f > 127.0f) ||
            (esc_f < 0.0f) || (esc_f > 127.0f))
        {
            DM_Print("[DM] usage: dm setid <mst_id 0..127> <esc_id 0..127>\r\n");
        }
        else
        {
            motor->cfg.mst_id = (uint8_t)mst_f;
            motor->cfg.esc_id = (uint8_t)esc_f;
            (void)snprintf(line, sizeof(line),
                           "[DM] RAM only: mst=0x%02X esc=0x%02X. This does NOT change\r\n"
                           "     the motor; configure it in DMTool and it will revert\r\n"
                           "     on reboot.\r\n",
                           (unsigned int)motor->cfg.mst_id,
                           (unsigned int)motor->cfg.esc_id);
            DM_Print(line);
        }
    }
    else if (strcmp(verb, "resend") == 0)
    {
        char *value_text = DM_NextToken(&cursor);

        if (value_text == NULL)
        {
            (void)snprintf(line, sizeof(line),
                           "[DM] periodic resend = %u (period %u ms, compile default %u).\r\n"
                           "     Why it defaults on: the motor does NOT report feedback on its\r\n"
                           "     own, so without a periodic frame the link-loss protection\r\n"
                           "     would fire on a perfectly healthy link.\r\n",
                           (unsigned int)motor->resend_enable,
                           (unsigned int)DM_RESEND_PERIOD_MS,
                           (unsigned int)DM_PERIODIC_RESEND);
            DM_Print(line);
        }
        else if ((strcmp(value_text, "0") == 0) || (strcmp(value_text, "1") == 0))
        {
            DM_SetResend(motor, (uint8_t)(value_text[0] - '0'));
            (void)snprintf(line, sizeof(line),
                           "[DM] periodic resend = %u. WARNING: with 0 the link-loss\r\n"
                           "     protection will fire on a healthy link (see dm resend).\r\n",
                           (unsigned int)motor->resend_enable);
            DM_Print(line);
        }
        else
        {
            DM_Print("[DM] usage: dm resend [0|1]\r\n");
        }
    }
    else
    {
        DM_Print("[DM] unknown command; send 'dm help'\r\n");
    }

    return 1U;
}

/* 让 DM_Motor_Disable 的返回值不至于被 -Wunused 之类的规则盯上；
 * 同时给 arm_task 提供一个"立刻松劲"的语义化名字。 */
uint8_t DM_IsLinkLost(const dm_motor_t *motor)
{
    return ((motor != NULL) && (motor->state == DM_STATE_FAULT)) ? 1U : 0U;
}
