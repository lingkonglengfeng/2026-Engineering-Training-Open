# TEMP 第二圆环 station=3 数量链路核对（2026-10-15）

结论：当前 CAM1 1.1.16 / `2026-10-15-occupied-resources-v30` 已满足两环搜索所需的上报行为，无需修改识别或发送代码。仅新增 `test_temp2_ring_count.py` 回归测试。本次未修改电控、未上传/烧录设备、未实车验证。

## 相机行为及电控接收约定

- SET 使用 mode=3、station=3、target=2；flags 支持 0/1/2/3，带物料且输出轴角用 flags=3。当前版本不支持历史 v29 的 flags=0x08/0x0B，TYPE83 最后一个载荷字节为0。
- 两个当前可靠目标时，按 SET 的 hz 上限及实际检测速度持续提交观测组：`RING_PX(03)` 无效/坐标全零；开启轴角时 `RING_AXIS(0A)` 也无效；`RING_COUNT(83)` 有效/count=2/station=3。坐标无效不代表数量无效。
- 三个当前可靠目标时，立即发送有效 count=3，原三帧圆心确认/身份检查通过后恢复有效坐标与可用轴角。不能要求第一帧 count=3 时坐标必定已有效。
- 03/可选0A/83同组的 META.ID=2、图像 SEQ 相同，有效位各自独立；每个提交的新观测组递增图像 SEQ（模16）。83 payload 为 `[count, station, successful_SET_seq, 0]`，绑定当前 SET。
- `observation_id` 不在8字节线上另加字段；现有主控 `ring_pair_part()` 接收到完整同序号观测组后生成新的本地 observation_id，位置无效仍可组装有效 count=2。
- 已确认三环后少一环，当帧切换 count=2 并取消坐标；完全消失则报告当前可信 count=0。检测异常或单轮处理超过500ms时数量失效，不能授权搜索。不能靠上一帧 count=2 续期。
- 相机仅输出视觉观测和 ACK/心跳，不发送车辆平移指令。图像右侧慢移搜索由电控根据新鲜 count=2 决定。

## 核对位置

`service.py` 的 `MaixBackend.observe()` 始终把当前 groups 数量与 scene_trusted 放入返回值；`len(groups)!=3` 仅清坐标。`VisionService.tick()` 的 `count!=3` 同样只清坐标，仍生成有效数量帧并持续调度，数量变化会绕过常规限流立即提交。

`middle_ring.py` 的 `RingDetector.process_scene()` 在三组局部重测不足时当帧回退全图，返回当前真实 groups；全图可靠两组会设置 scene_trusted=True，不把两组丢掉，也不把历史三组充作当前三组。统一服务配置 require_scene=True。

## 验证

- 36项针对数量/场景/轴角/档位的源码测试通过。
- 生成入口 main.py 的2项新增完整链路测试通过，直接使用入口内嵌 MaixBackend 和圆环算法。
- 新增测试使用真实 OpenCV 合成圆环图，经 backend、service、串口编码与 CRC 解码验证：冷启动两环持续22组含 SEQ 回绕；flags0/1/2/3；两环恢复三环及有效坐标/轴角；三环丢一环立即切两环并持续；随后全失报0；station/SET绑定与各帧有效位。
- 内嵌 material/plate/ring 源码与当前各模块一致，安装包 `dist/maix-v1-v1.1.16.zip` 的 main.py 与当前生成入口逐字节一致。
- 日志及源码/入口/安装包 SHA256：`output/temp2_ring_count_audit_2026-10-15/tests.log`、`validation.json`。桌面验证不能证明设备检测频率或车辆搜索行为。
