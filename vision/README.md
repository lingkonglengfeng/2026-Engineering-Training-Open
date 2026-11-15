# 双路视觉

| 目录 | 作用 |
| --- | --- |
| [pc_vision](pc_vision/README.md) | Windows 图传、识别、调试与标定 |
| [maixcam_bridge](maixcam_bridge/README.md) | CAM1 物料、圆环、圆盘桥接 1.3.6 |
| [maixcam_qr_bridge](maixcam_qr_bridge/README.md) | CAM2 扫码、前方障碍桥接 1.4.1 |
| [maixcam_service](maixcam_service/README.md) | 旧相机本地统一服务及 PC 复用算法 |
| [maixcam_material](maixcam_material/README.md) · [maixcam_ring](maixcam_ring/README.md) · [maixcam_qr](maixcam_qr/README.md) | 专项识别与对照 |
| [maixcam_light](maixcam_light/README.md) · [maixcam_sampler](maixcam_sampler/README.md) | 补光试验、颜色取样 |

两台相机各有 IP，通过 Wi-Fi 向 PC 提供图像和任务会话。桥接把识别结果转发给各自 UART。TCP 9000 图传、9001 控制，UDP 9002 发现。支持小屏回显的桥使用 TCP 9003，CAM1 1.3.6 的补光控制使用 TCP 9004。共享协议入口在 `protocol8.py`。桥接生成和安装见 [工具](../tools/README.md)。
