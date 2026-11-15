# 智能搬运赛项 FreeRTOS 任务安排与设计

> 版本：v1.0<br>
> 日期：2026-08-19<br>
> 平台：STM32H723ZGT6 + CMSIS-RTOS v2 + FreeRTOS<br>
> 规则依据：`../docs/electrical/rules.pdf` 第 1~8 页

## 1. 设计结论

本项目需要同时使用 FreeRTOS 和状态机：

- FreeRTOS 负责控制、通信、传感器、显示等并发活动的调度；
- `MatchTask` 内的主状态机负责比赛流程和规则顺序；
- `MotionTask`、`ActuatorTask` 内的小状态机负责非阻塞动作；
- `ControlTask` 只执行确定周期的控制计算，不感知比赛流程。

禁止把旧代码的 `TASK1...TASK13`、无限 `while` 和 `HAL_Delay` 原样放进任务。所有耗时动作统一改成 `start -> poll/event -> complete/timeout/cancel`。

## 2. 规则到软件设计的映射

| 规则要求 | 软件责任 | 设计措施 |
|---|---|---|
| 一键启动且只有一次启动机会 | `SafetyTask` | 消抖后锁存 `start_consumed`；进入运行后禁止第二次启动 |
| 读取四组三位数任务码 | `PerceptionTask` + `MatchTask` | 校验 `ddd+ddd+ddd+ddd`，颜色和环号仅允许 `1..6` |
| 显示任务码和完成统计 | `DisplayTask` | 独占 USART3，显示请求通过队列发送，字体尺寸由屏幕工程保证不小于 12 mm |
| 原料每次只抓一个，放到载物台后才可继续 | `ActuatorTask` + `MatchTask` | 每次抓取必须收到载物台确认事件；无确认不得离开原料区 |
| 第一批经过粗加工区并转移到暂存区 | `MatchTask` | 固定状态顺序，移动与取放均等待动作结果 |
| 第二批在暂存区同色码垛 | `MatchTask` + `ActuatorTask` | 码垛前校验目标颜色、第一层存在和放置位置 |
| 仅在灰色车道行驶并绕开黑色障碍物 | `MotionTask` + `PerceptionTask` | 障碍物事件优先于普通导航；先减速停车，再选择绕行或等待重识别 |
| 完成后返回抽签确定的启停区 | `MatchTask` + `MotionTask` | 起点编号在启动前锁存，结束状态必须验证回站 |
| 运行中不可遥控或连接上位机 | `SafetyTask` + `TelemetryTask` | 比赛模式关闭所有外部控制入口；调试串口只发送日志，不接收运动命令 |
| 任何故障不得造成持续危险动作 | `SafetyTask` | 急停、通信超时、掉料、限位和任务超时统一进入 `SAFE_STOP` |

## 3. 总体并发结构

```text
专用控制定时器 ISR ----通知----> ControlTask ----最新轮速----> CommsTask ----FDCAN1/2----> 电机
                                     ^                         ^
                                     |                         |
SensorFusionTask ----位姿快照----> MotionTask ----运动目标-----+
 IMU + 光流 + 轮端反馈                 ^
                                     |
PerceptionTask ----视觉事件--------> MatchTask ----动作请求----> ActuatorTask ----机械臂/舵机
 QR + 圆环 + 颜色 + 障碍物             |                         |
                                     +----显示请求------------> DisplayTask

急停/心跳/超时 --------------------> SafetyTask ----停止锁存----> 所有执行任务
```

### 3.1 所有权原则

- 只有 `MatchTask` 能修改比赛主状态；
- 只有 `MotionTask` 能生成车体运动目标；
- 只有 `ActuatorTask` 能推进机械动作序列和更新 TIM4 舵机目标；
- 只有 `CommsTask` 能从任务上下文发送普通 CAN 帧；
- 只有 `DisplayTask` 能向 USART3 发送屏幕指令；
- ISR 只收集数据、置硬件安全信号和唤醒任务，不执行解析、PID 或流程跳转。

## 4. 任务安排

CMSIS-RTOS v2 数值越大优先级越高。以下栈大小是首版预算，必须通过 `uxTaskGetStackHighWaterMark()` 实测调整。

| 任务 | 触发/周期 | 优先级 | 初始栈 | 最长连续执行 | 主要职责 |
|---|---:|---:|---:|---:|---|
| `SafetyTask` | 事件 + 10 ms | `osPriorityRealtime` 48 | 768 B | 100 us | 启动锁存、急停、故障、心跳、看门狗和统一停机 |
| `ControlTask` | 专用定时器 1 kHz 通知 | `osPriorityHigh` 40 | 1024 B | 300 us | 航向/速度控制、限幅、斜坡和轮速目标计算 |
| `SensorFusionTask` | 5 ms | `osPriorityAboveNormal2` 34 | 1536 B | 500 us | IMU、光流、轮端反馈质量判断和短时位姿估计 |
| `CommsTask` | 事件 + 5 ms | `osPriorityAboveNormal1` 33 | 1024 B | 500 us | CAN 收发、紧急停止帧、反馈时间戳和链路健康 |
| `MotionTask` | 10 ms + 请求 | `osPriorityAboveNormal` 32 | 1536 B | 800 us | 导航、避障、到站判定、圆环视觉精定位 |
| `ActuatorTask` | 10 ms + 请求 | `osPriorityNormal3` 27 | 1536 B | 800 us | 取料、放载物台、粗加工放置、暂存转移和码垛动作状态机 |
| `PerceptionTask` | DMA/IDLE 事件 | `osPriorityNormal2` 26 | 1536 B | 1 ms | 二维码、颜色、圆环和障碍物协议解析与有效期管理 |
| `MatchTask` | 比赛事件 + 10 ms 兜底 | `osPriorityNormal` 24 | 2048 B | 1 ms | 比赛主状态机、动作编排、规则检查和统计 |
| `DisplayTask` | 事件 + 100 ms | `osPriorityLow3` 11 | 1024 B | 2 ms | 显示任务码、阶段、统计和故障码 |
| `TelemetryTask` | 100 ms | `osPriorityLow` 8 | 1536 B | 2 ms | 调试日志、栈水位、队列峰值和性能统计 |

十个任务初始栈预算约 13.5 KiB，低于当前 128 KiB FreeRTOS heap，但队列、控制块和动态分配也会占用 heap。联调稳定后建议将 RTOS 对象改为静态创建。

## 5. RTOS 对象设计

### 5.1 事件组 `eg_system`

| 位 | 含义 | 设置者 | 清除者 |
|---|---|---|---|
| `EV_SELF_TEST_OK` | 上电自检通过 | `SafetyTask` | 复位 |
| `EV_START_LATCHED` | 唯一启动已确认 | `SafetyTask` | 复位 |
| `EV_MISSION_VALID` | 任务码有效 | `PerceptionTask` | 新一轮复位 |
| `EV_RUNNING` | 比赛流程运行中 | `MatchTask` | `MatchTask/SafetyTask` |
| `EV_PAYLOAD_ON_PLATFORM` | 物料已在载物台 | `ActuatorTask` | 取走物料后 |
| `EV_OBSTACLE` | 障碍物进入安全距离 | `PerceptionTask` | 连续安全帧确认后 |
| `EV_ESTOP` | 急停锁存 | 硬件/`SafetyTask` | 仅系统复位 |
| `EV_FAULT` | 故障锁存 | 任意任务 | 仅系统复位 |
| `EV_FINISHED` | 比赛流程完成 | `MatchTask` | 复位 |

### 5.2 队列和数据通道

| 对象 | 深度 | 数据 | 生产者 -> 消费者 | 满载策略 |
|---|---:|---|---|---|
| `q_match_event` | 16 | `MatchEvent` | 所有服务 -> `MatchTask` | 故障事件走独立通知；普通重复事件可合并 |
| `q_motion_request` | 4 | `MotionRequest` | `MatchTask` -> `MotionTask` | 拒绝新请求并报告 `BUSY` |
| `q_actuator_request` | 4 | `ActuatorRequest` | `MatchTask` -> `ActuatorTask` | 拒绝新请求并报告 `BUSY` |
| `q_can_rx` | 32 | `CanFrame` | FDCAN ISR -> `CommsTask` | 丢弃最旧普通反馈、锁存溢出计数 |
| `q_can_tx` | 16 | `CanFrame` | 服务 -> `CommsTask` | 普通命令返回失败，禁止静默丢弃 |
| `q_can_urgent` | 4 | `CanFrame` | `SafetyTask` -> `CommsTask` | 预留急停容量，始终先于普通队列 |
| `q_display` | 8 | `DisplayMessage` | 各任务 -> `DisplayTask` | 同类型消息覆盖旧值 |
| `sb_vision_rx` | 512 B | 字节流 | UART DMA ISR -> `PerceptionTask` | 记录溢出并使当前帧失效 |
| `sb_qr_rx` | 128 B | 字节流 | UART DMA ISR -> `PerceptionTask` | 丢弃到下一帧边界 |

控制目标、位姿和感知结果使用单写者版本化快照，不让 1 kHz `ControlTask` 等待互斥锁：

```text
MotionTask      -> MotionSetpoint 快照 -> ControlTask
SensorFusionTask-> NavEstimate 快照    -> MotionTask/ControlTask
PerceptionTask  -> Perception 快照     -> MotionTask/MatchTask
ControlTask     -> WheelSetpoint 快照  -> CommsTask
```

写入时在极短临界区内更新 `sequence + payload`，读取方读前后两次序号一致才接受。

### 5.3 任务通知

- 控制定时器通过线程标志/直接任务通知唤醒 `ControlTask`；
- `SafetyTask` 用紧急通知唤醒 `CommsTask`，避免急停帧被普通 CAN 队列阻塞；
- 每个关键任务周期性设置自己的 heartbeat 位；
- 动作完成统一转换为 `MatchEvent`，不直接调用 `MatchTask` 函数。

## 6. 比赛主状态机

```text
BOOT
 -> SELF_TEST
 -> WAIT_START
 -> READ_MISSION
 -> PICK_BATCH1_ITEM_1/2/3
 -> MOVE_TO_PROCESSING_1
 -> PLACE_BATCH1_PROCESSING
 -> TRANSFER_BATCH1_TO_TEMP
 -> RETURN_TO_RAW
 -> PICK_BATCH2_ITEM_1/2/3
 -> MOVE_TO_PROCESSING_2
 -> PLACE_BATCH2_PROCESSING
 -> MOVE_TO_TEMP_2
 -> STACK_BATCH2
 -> RETURN_TO_START
 -> FINISH

任意运行状态 --急停/严重故障/动作超时--> SAFE_STOP
```

### 6.1 状态进入规范

每个状态只能在 `on_entry` 中发送一次动作请求，随后等待带 `request_id` 的结果：

```text
on_entry:  生成 request_id，发送动作请求，记录 deadline
running:   处理完成/失败/障碍物/超时事件
success:   更新统计并进入下一状态
failure:   锁存故障并进入 SAFE_STOP，或执行该状态明确允许的有限重试
```

不得通过循环等待电机到位，也不得用 `osDelay()` 表示机械动作完成。

### 6.2 规则门禁

- `READ_MISSION -> PICK_BATCH1`：必须已解析并显示有效任务码；
- `PICK_ITEM -> 下一物料`：必须确认本次物料已放到载物台；
- `离开原料区`：载物台计数必须等于本批计划数量；
- `PLACE_PROCESSING`：颜色、顺序和环号必须与任务码一致；
- `STACK_BATCH2`：目标第一层存在、颜色一致、第一层放置正确；
- `FINISH`：回到起始启停区且底盘停止，完成统计已送达屏幕。

## 7. 运动与传感器设计

### 7.1 运动控制分层

```text
MatchTask：我要去粗加工区
MotionTask：规划当前段、避障、粗定位、视觉精定位
ControlTask：根据速度目标和位姿误差计算四轮目标
CommsTask：以固定发送周期下发 ZDT 同步命令并接收反馈
```

`ControlTask` 运行 1 kHz，但底盘 CAN 不必每个控制周期都发送。建议 `CommsTask` 以 200 Hz 发送最新四轮目标，利用 ZDT 内部闭环，防止 CAN 队列累积历史控制量。

### 7.2 IMU、光流和视觉

- IMU 提供航向和角速度，不积分加速度作为长期位置；
- 光流提供短时间 `dx/dy`，质量不足时禁止积分；固定离地高度时按标定比例换算，补光只能改善纹理质量，不能补偿高度尺度变化；
- 轮端反馈提供相对运动和打滑判据；
- 圆环/颜色视觉提供作业点绝对校正；
- `SensorFusionTask` 输出 `x/y/yaw/quality/timestamp`，旧数据超过有效期不得用于闭环。

首版使用质量门控的互补融合，不必直接上 EKF。完成单传感器误差测试后，再决定是否升级滤波器。

## 8. 安全和故障处理

### 8.1 故障等级

| 等级 | 示例 | 处理 |
|---|---|---|
| `WARN` | 单帧校验失败、瞬时低光流质量 | 计数、降级，不中止流程 |
| `RECOVERABLE` | 视觉短时丢失、一次动作未到位 | 停止当前动作，有限次数重试 |
| `FATAL` | 急停、CAN 总线关闭、任务栈溢出、载物掉落、限位冲突 | 立即进入 `SAFE_STOP`，复位前不恢复 |

### 8.2 安全停止顺序

1. 硬件急停优先切断执行器使能或驱动允许信号；
2. 锁存 `EV_ESTOP/EV_FAULT`，将运动目标原子置零；
3. `CommsTask` 优先发送底盘和机械臂立即停止帧；
4. `ActuatorTask` 取消动作并保持不致掉料的安全姿态；
5. 屏幕和 ERR_LED 显示故障码；
6. 保存状态、任务、请求号和传感器时间戳，等待系统复位。

独立看门狗只能在所有关键任务 heartbeat 正常时由 `SafetyTask` 喂养，禁止在 Idle Hook 无条件喂狗。

## 9. 中断与 STM32H7 注意事项

1. 当前 TIM7 是 HAL 1 ms 时基且中断优先级为 0，不能用于控制任务通知；新增 TIM6/TIM2 等专用控制定时器。
2. `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`。调用 RTOS `FromISR` API 的 UART、FDCAN、DMA 和控制定时器中断，NVIC 数值必须大于等于 5。
3. 如果急停中断使用高于 5 的硬实时优先级，它只能操作硬件安全输出和普通 `volatile` 锁存，不能调用 RTOS API。
4. H7 开启 D-Cache 后，UART DMA 缓冲区必须放到 DMA 可访问且缓存策略正确的区域，或严格执行 Cache clean/invalidate；否则会出现随机旧数据。
5. FDCAN 必须配置 RX FIFO、标准/扩展帧过滤器、消息 RAM 偏移和错误中断。当前生成代码的 FIFO/过滤器数量为 0，不能直接收发业务帧。
6. 所有 ISR 都要有执行时间上限，不在 ISR 中解析视觉帧、打印日志或运行 PID。

## 10. 实施计划

### 阶段 1：RTOS 空骨架

- 建立所有消息结构、事件位和任务句柄；
- 创建任务但仅运行 heartbeat；
- 接入栈溢出、Malloc 失败和断言故障记录；
- 验证 30 分钟无复位、栈水位大于预算的 25%。

### 阶段 2：通信和安全

- 完成 FDCAN FIFO/过滤器和 UART DMA + IDLE；
- 完成急停、唯一启动、看门狗和 CAN 紧急停止；
- 注入断线、错帧、队列满和总线关闭故障。

### 阶段 3：运动和传感器

- 移植 ZDT/达妙协议，不移植旧阻塞流程；
- 接入 IMU、光流和轮端反馈；
- 分别验证航向、1 m/2.4 m 位移、横移、旋转和打滑检测。

### 阶段 4：动作和比赛状态机

- 将取料、放置、转移和码垛实现为异步动作状态机；
- 接入任务码、圆环和颜色识别；
- 按正常、超时、误码、掉料、障碍物和急停路径回归。

### 阶段 5：性能冻结

- 测量 CPU 占用、任务最坏执行时间、栈水位、队列峰值和 CAN 总线负载；
- 固化比赛参数和版本，关闭外部控制入口；
- 连续完成多轮全流程，并保留每轮故障与耗时记录。

## 11. 验收条件

- [ ] `MatchTask` 中没有 `HAL_Delay` 和无限等待；
- [ ] 每个动作都有唯一请求号、完成条件、失败条件和 deadline；
- [ ] 急停触发后一个控制周期内运动目标归零；
- [ ] 普通队列满不会阻塞 `SafetyTask` 和 `ControlTask`；
- [ ] 视觉、光流、IMU 和电机反馈都有时间戳和有效期；
- [ ] CAN 断线或视觉丢失不会继续使用旧数据盲动；
- [ ] 物料未确认在载物台时不能离开原料区；
- [ ] 第二批只在同色且正确的第一层物料上码垛；
- [ ] 完成统计与实际动作结果一致，并在回站后保持显示；
- [ ] 比赛模式无法通过调试串口、Wi-Fi 或其他接口控制运动。

## 12. 当前工程需要的 CubeMX 变更

- 删除空转的 `defaultTask`，或只让它调用 `AppTasks_Init()` 后自删除；
- 新增专用控制定时器和对应 NVIC；
- 为 USART1/2/3、UART4 按实际用途配置 DMA + IDLE；
- 为 FDCAN1/2 配置 RX FIFO、TX FIFO、过滤器和 NVIC；
- 增加独立看门狗；
- 核对 DMA 缓冲区内存区与 D-Cache；
- 若注册所有队列用于调试，将 `configQUEUE_REGISTRY_SIZE` 从 8 提高到至少 16；
- 保留 `configCHECK_FOR_STACK_OVERFLOW=2`，增加 Malloc 失败钩子和可追溯的断言处理。

这些变更应先在 `.ioc` 中完成，再生成代码；不要只手改生成的 `fdcan.c/usart.c`，否则下一次 CubeMX 生成会丢失配置。
