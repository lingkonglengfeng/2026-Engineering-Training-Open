# 达妙与 ZDT 电机通信调试总结

日期：2026-09-09  
工程：`Engineering_Training_Project`  
MCU：STM32H723ZGT6
<br>
> 2026-09-11 更新：当前工程 UART4 已改为 `9600 8N1`，用于 JDY-23 蓝牙模块；下文 `115200` 参数属于修改前的历史测试状态。
>
> **2026-09-14 更新**：本文件描述的**电机参数（FDCAN1 / 0x01 / 0x11 / POS / 1 Mbps 经典 CAN）至今有效、未变**。
> 变的是代码结构：驱动改为句柄式（`DM_Motor_Init(&motor, &cfg)`）、新增 `armTask`（10 ms）、
> 新增 `dm probe` 自检命令、失去控制自动松劲。**命令表与调试流程请以
> [达妙转盘移植说明](达妙转盘移植说明_2026-09-14.md) 为准**（本文件第 4 节是旧版命令表，
> `dm pos` 的默认速度和量程检查等行为已变）。本次同时修掉了旧驱动里
> **CAN 帧 DLC 编码全错**、位置反馈有符号解码、逐帧阻塞打印等 5 个 bug，详见移植说明第 3 节。

## 1. 当前总线分配

| 设备 | FDCAN 外设 | MCU 引脚 | 波特率 | CAN ID 类型 |
|---|---|---|---|---|
| 达妙电机 | FDCAN1 | PA11 = RX，PA12 = TX | Nominal 1 Mbps | 标准 ID |
| ZDT 电机 | FDCAN2 | PB12 = RX，PB13 = TX | 500 kbps | 扩展 ID |

UART4 使用 `PC10/PC11`，参数为 `115200 8N1`，作为电机控制台。

## 2. 达妙电机配置

- 控制/ESC ID：`0x01`
- 反馈/Master ID：`0x11`
- 控制模式：POS 位置模式
- FDCAN 帧格式：Classic CAN
- 位置控制帧 ID：`0x101`
- 速度控制帧 ID：`0x201`
- 反馈帧 ID：`0x11`

POS 模式使能帧：

```text
ID：0x101
DATA：FF FF FF FF FF FF FF FC
```

位置帧为 8 字节：前 4 字节是小端 float 位置（rad），后 4 字节是小端 float 速度（rad/s）。速度帧 `0x201` 为 4 字节小端 float 速度。

## 3. CubeMX 配置要点

FDCAN1 使用 `PA11=FDCAN1_RX`、`PA12=FDCAN1_TX`、`AF9`、Normal、Classic CAN。主要参数如下：

| 参数 | 值 |
|---|---:|
| Auto Retransmission | Enable |
| Protocol Exception | Enable |
| Nominal Prescaler / SJW / Seg1 / Seg2 | 4 / 2 / 15 / 4 |
| Message RAM Offset | 128 |
| Std Filters Nbr | 1 |
| Rx FIFO0 Elmts Nbr / Size | 8 / 8 bytes |
| Tx FIFO Queue Elmts Nbr / Size | 4 / 8 bytes |

启用 `FDCAN1_IT0_IRQn`。FDCAN2 的 `PB12/PB13` 和 500 kbps 设置必须保留。

## 4. UART4 控制命令

发送命令后按回车：

```text
dm help
dm status
dm enable
dm disable
dm clear
dm zero
dm pos <position_rad> [velocity_rad_s]
dm speed <velocity_rad_s>
```

推荐测试流程：

```text
dm status
dm enable
dm pos 0 0.2
dm pos 0.1 0.2
dm disable
```

`online=1` 表示收到反馈，`fb` 应持续增加。ZDT 原有 `help`、`ver`、`status`、`enable`、`disable`、`run <rpm>`、`stop` 命令仍可用。

## 5. 关键问题与修复

1. `tasks.c` 存在多余字符 `5`，删除后解决 FreeRTOS 语法错误。
2. UART4 原来按任务周期读单字节，长命令只能收到首字符；改为 RXNE 中断环形缓冲，并增加任务轮询兜底。
3. CubeMX 重新生成后重复生成 `FDCAN1_IT0_IRQHandler()`；删除重复实现。
4. 达妙使能曾发到 `0x001`，而位置帧发到 `0x101`；原因是仍处于 MIT 模式。驱动现强制使用 POS 模式，使能和位置帧统一为 `0x101`。
5. CubeMX 默认 FDCAN1 的 FIFO 数量为 0、时序为占位值；已按 80 MHz 内核时钟配置 1 Mbps、消息 RAM、RX FIFO、TX FIFO 和标准过滤器。

## 6. 验证结果

Keil 编译结果：

```text
0 Error(s), 0 Warning(s)
```

已验证 ZDT FDCAN2 版本查询、UART4 命令行和达妙使能/位置控制正常，两个电机总线互不冲突。

## 7. 后续排查顺序

1. `dm status` 检查 `online=1`、`fb` 是否增加。
2. 确认 `dm enable` 日志为 `SID=0x101`。
3. 确认发送日志 `HAL=0` 且错误码为 0。
4. 检查 CANH/CANL、收发器供电、公共地和终端电阻。
5. 确认电机仍为 ID `0x01/0x11`、1 Mbps、Classic CAN。
6. 先用小角度、低速度测试，避免直接发送大角度或高速命令。
