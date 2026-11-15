# CAM1 MaixCAM 桥接 1.3.6

物料判色、顶面、圆环与圆盘对齐。A18 PWM 补光由上位机切换，自动档为物料 25%、圆盘 60%、圆环 30%。 图像和任务结果通过局域网传输；串口连接主控对应通道。

1. 在仓库根目录执行 `python tools/package_pc_vision_bridge.py --camera material`。
2. 将本目录 `dist/` 生成的 ZIP 安装到对应相机，启动应用。需要自动接入时在设备端设置该应用开机启动。
3. 确认两台相机与 PC 可通过同一局域网通信，并在上位机选择对应设备。

`app.yaml` 记录当前应用版本，`main.py` 为设备入口。TCP 9000 图传、9001 结果，UDP 9002 设备发现。详细流程见 [双路视觉](../README.md)和 [上位机](../pc_vision/README.md)。安装工具参见 [Sipeed 应用说明](https://wiki.sipeed.com/maixpy/doc/zh/basic/app.html)。
