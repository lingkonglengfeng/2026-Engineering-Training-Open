
/**
  ******************************************************************************
  * @file    servo.h
  * @brief   270°舵机驱动接口（TIM4 PWM 输出，50Hz 标准舵机协议）
  * @note    硬件连接（见《接口冻结表_v1.md》第 6.1 节）：
  *          - SERVO_GRIP       PD12 / TIM4_CH1  物料抓取舵机 DS3218（270°）
  *          - SERVO_TURNTABLE  PD13 / TIM4_CH2  转盘舵机（270°）
  *
  *          电气约束：
  *          - 舵机必须由独立电源供电，严禁取自 MCU 3.3V；
  *          - 舵机功率地必须与主控逻辑地单点共地；
  *          - 50Hz / 500~2500us 为 270° 行程的标准脉宽约定，
  *            各舵机零位、方向、极限角必须在整车上重新标定后写入参数表。
  *
  *          使用顺序：
  *          1. main() 中 MX_TIM4_Init() 之后（保证 GPIO 复用已配置）；
  *          2. 任意任务上下文调用 Servo_Init()；
  *          3. 周期（建议 5~20ms）调用 Servo_Update(now_ms) 驱动限速斜坡，
  *             该函数完全非阻塞，可在任意优先级任务中调度；
  *          4. 急停场景在 ISR 中只调用 Servo_RequestSafeStop() 置位标志，
  *             实际 PWM 停止由 Servo_Update() 在任务上下文完成。
  ******************************************************************************
  */

#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

#include "stm32h7xx_hal.h"

/* 舵机通道编号，与 TIM4 硬件通道一一对应 */
typedef enum
{
    SERVO_ID_GRIP = 0,      /* PD12 / TIM4_CH1：物料抓取舵机 */
    SERVO_ID_TURNTABLE = 1, /* PD13 / TIM4_CH2：转盘舵机 */
    SERVO_ID_COUNT          /* 通道总数，仅内部使用 */
} servo_id_t;

/**
  * @brief  初始化舵机驱动：重配 TIM4 为 50Hz 时基并启动两路 PWM
  * @note   上电后两路立即输出各自的安全角（软启动值），
  *         舵机会以自身最大速度转到安全角，属正常现象。
  * @retval HAL_OK 成功；其他值表示 TIM4 配置失败
  */
HAL_StatusTypeDef Servo_Init(void);

/**
  * @brief  设置舵机目标角度（单位：度，0~270）
  * @note   角度会被钳位到该舵机的机械行程内；实际输出由
  *         Servo_Update() 按限速斜坡逐步逼近，不是立即跳变。
  *         安全停锁存期间本调用被忽略。
  * @param  id        舵机通道编号
  * @param  angle_deg 目标角度
  */
void Servo_SetAngle(servo_id_t id, float angle_deg);

/**
  * @brief  按脉宽直接设定目标（单位：微秒，500~2500）
  * @note   不依赖角度标定：500us=舵机自身物理最小位，
  *         2500us=自身物理最大位，适合行程未知的舵机做极限测试。
  *         限速斜坡和安全停约束仍然有效。
  * @param  id       舵机通道编号
  * @param  pulse_us 目标脉宽，超范围值被钳位
  */
void Servo_SetPulseUs(servo_id_t id, uint16_t pulse_us);

/**
  * @brief  修改舵机限速（单位：度/秒）
  * @param  id  舵机通道编号
  * @param  dps 新的角速度上限，范围 [1, 360]，超限值被截断
  */
void Servo_SetSpeedLimit(servo_id_t id, float dps);

/**
  * @brief  舵机周期处理：执行限速斜坡和安全停请求，非阻塞
  * @note   必须周期性调用（建议 5~20ms），否则舵机不会运动。
  *         now_ms 建议传 osKernelGetTickCount()（1ms 节拍）。
  * @param  now_ms 当前系统毫秒时基
  */
void Servo_Update(uint32_t now_ms);

/**
  * @brief  查询舵机当前输出角度（斜坡的瞬时值，非物理反馈）
  */
float Servo_GetCurrentAngle(servo_id_t id);

/**
  * @brief  查询舵机目标角度
  */
float Servo_GetTargetAngle(servo_id_t id);

/**
  * @brief  查询舵机是否仍在向目标角运动
  * @retval 1 运动中；0 已到位
  */
uint8_t Servo_IsBusy(servo_id_t id);

/**
  * @brief  请求安全停（可在 ISR 中直接调用，仅置位 volatile 标志）
  * @note   符合冻结表约束：急停 ISR 只置事件位，不做 HAL 操作；
  *         实际的 PWM 停止由下一次 Servo_Update() 在任务上下文执行。
  *         停止后两路 PWM 关闭，舵机失去保持力矩（脱力）。
  */
void Servo_RequestSafeStop(void);

/**
  * @brief  查询安全停是否已锁存
  * @retval 1 已锁存（PWM 已关、新指令被拒绝）；0 正常
  */
uint8_t Servo_IsSafeStopLatched(void);

/**
  * @brief  退出安全停并恢复输出（必须在人工确认后调用）
  * @note   脱力期间舵机物理位置未知，恢复时直接重新输出安全角。
  *         若此时正处于危险持料状态，应先按冻结表要求人工处理物料。
  * @retval HAL_OK 已恢复；HAL_ERROR 驱动尚未初始化
  */
HAL_StatusTypeDef Servo_RecoverFromSafeStop(void);

#endif /* __SERVO_H__ */
