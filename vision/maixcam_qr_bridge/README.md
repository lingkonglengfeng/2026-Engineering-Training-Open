# CAM2 MaixCAM 桥接 1.4.1

二维码识别与前方障碍观测。PC 计算结果后通过会话转给主控，距离标定见 PC 视觉。 图像和任务结果通过局域网传输；串口连接主控对应通道。

1. 在仓库根目录执行 `python tools/package_pc_vision_bridge.py --camera qr`。
2. 将本目录 `dist/` 生成的 ZIP 安装到对应相机，启动应用。需要自动接入时在设备端设置该应用开机启动。
3. 确认两台相机与 PC 可通过同一局域网通信，并在上位机选择对应设备。

`app.yaml` 记录当前应用版本，`main.py` 为设备入口。TCP 9000 图传、9001 结果，UDP 9002 设备发现。详细流程见 [双路视觉](../README.md)和 [上位机](../pc_vision/README.md)。安装工具参见 [Sipeed 应用说明](https://wiki.sipeed.com/maixpy/doc/zh/basic/app.html)。
