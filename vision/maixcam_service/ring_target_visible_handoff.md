# TEMP2目标2可见：相机/电控交接（2026-10-15）

用户明确“绿色环”为算法绿色标记的目标2，而非实际印刷绿色。版本CAM1 **1.1.15**，BUILD `2026-10-15-ring-target-visible-v29`，配套主控V169。原灰度模型、PWM圆环15%/其他50%、三点取色、独立采样与CAM2保持。

## 协议

- SET：mode3/station3/target2，新增flags bit3=0x08，仅最后TEMP2使用；保留原轴角0x01、occupied0x02。电控使用flags11（0x0B）。station2及其他mode拒绝0x08，未知位拒绝。
- RING_COUNT TYPE0x83，payload仍 `[count,station,set_seq,flags]`。**packet[6] bit0=当前可靠目标2可见**，其他bit0；META的image_seq/target2/场景可信含义不变。03/0A/83仍同帧成组。
- 原flags0..3第四payload字节严格0，不输出新标志。presence变化纳入发送状态，及时发送/失效，不沿用旧写组。
- **标志只用于停止搜索，不代表可放置。** 部分组可见时count仍为真实1/2，位置03与角度0A仍无效，不伪造count3/trusted/坐标。

## 证据与边界

标志从当前MiddleTracker的observed派生，在三环位置资格判定之前计算。必须已建立目标2锁、没有身份拒绝，且observed.ring确实是当前groups中的测量对象；仅缓存旧observed不行，计数非零也不行。使用既有唯一匹配、模板拒绝、短时跟踪规则：中心匹配距≤max(8px,半径×0.65)，半径比0.60..1.50，记忆窗口随帧间隔在0.35..1.0秒间；没有新增RGB阈值、OCR或跨SET缓存。

冷启动只见一两个同形灰度环时无法可靠认2，标志false。先在同会话完整三环确认后，随后遮挡/偏出视野仍能可靠跟踪2时，可置位并保持XY/角度无效。失去2、匹配不唯一、身份拒绝、异常或本轮处理超过500ms，输出0；STOP、新SET/换站仍清全部锁。对原跟踪本身的身份误判概率没有实机保证，本次没有改几何模型。

相机不锁存“曾见”，每帧重新报告。主控V169约定新鲜同帧count3（无论XY是否锁定）或可见位=1即停盲搜，并在本次纠偏含yaw重启中保持已见状态；放置仍要求原完整三环/有效XY。这个控制逻辑由电控负责。

## 验证与使用

301项相机物料/圆盘/圆环/取色/补光回归通过，新增9项覆盖：

- 真实灰度CV：冷启动2环不认2；确认3环后当前2环仍跟踪2、标志1，但XY/axis无效；随后2离开立即标志0。
- 缓存旧观测、STOP/新SET后仅2环均不能置位。
- TYPE83 count2不改成3，序号与SET绑定一致；旧flags字节0、错identity拒绝、flags仅station3。
- 慢处理/异常清0，短写期间新会话先ACK，不补旧presence尾组。

安装 `vision/maixcam_service/dist/maix-v1-v1.1.15.zip`；单文件入口 `vision/maixcam_service/main.py`。交付目录 `output/cam1_ring_target_visible_2026-10-15`及同名ZIP，内有验证、tests、manifest、README和指南。仅更新相机不足以改变旧主控搜索逻辑，需配套V169。进入新开关应看到 `T2:VISIBLE` 或 `T2:--`，不能只凭旧程序的绿色预览认定已发送新bit。

本次未改Core、未烧录/上传设备、未实车；真实TEMP2日志和录屏需后续确认。
