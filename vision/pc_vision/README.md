# Windows 双路视觉上位机 2.6.35

对应 初赛主控 V306。使用 [图传检查](../../一键图传检查.cmd)预览，使用 [视觉上位机](../../视觉上位机.cmd)联动。启动器管理 Python 虚拟环境，依赖为 NumPy、OpenCV、pyserial，界面使用 Tkinter。查看 [环境检查](../../检查电脑视觉环境.cmd)。

手动启动预览：

```powershell
python -m pip install -r vision/pc_vision/requirements.txt
python vision/pc_vision/gui.py --mode preview
```

首次在 CAM1/CAM2 页选择设备。若发现失败，检查热点设备隔离、相机应用是否启动及 Windows 防火墙，可按实际 IP 配置。预览只显示结果，联动任务由主控切换。串口页选择当前 USB 转串口端口和 115200 波特率，常用于日志、准备和发车。

## 功能与标定

- CAM1 显示物料、圆环和圆盘检测。颜色取样在现场光照下进行，每个颜色取多帧。用主控当前抓取窗口与画面叠加对照。
- CAM2 显示二维码和前方圆柱。二维码结果参与物料任务。障碍距离使用车头前沿到圆柱近侧的定义，换相机位置后重新标定。
- 「机械臂控制」读取反馈、单轴调试、圆盘姿态与圆环微调。先用预览点核对落点，再应用到主控 RAM。需要掉电保留时使用对应的保存按钮并等主控回执。
- 「转向 PID」用于读取、调节和观察转向反馈。参数变更应记录固件版本、场地和验证结果。
- 「底盘参数」读写速度、加减速、直行纠偏和衔接参数。先应用到 RAM，确认后保存到 W25。
- 「工位地图」读写工位偏移；手动标记最多两个障碍会关闭视觉障碍联动，恢复视觉时清除手动点。手动点存在 RAM。
- 圆环页可选择 ROUGH／TEMP 共用或独立参数，共用以 ROUGH 为准。保存后模式和数据一起恢复。

## 文件入口

`gui.py` 是界面，`backend.py` 与 `qr_backend.py` 是两路算法入口，`transport.py` 管理网络连接，`serial_monitor.py` 管理串口和日志。各面板把参数转换为主控文本协议。无设备测试见 [测试说明](../../tools/tests/README.md)。

`config.example.json` 与 `config.cam2.example.json` 为配置模板。生成的 `config.json`、`config.cam2.json`、`calibration.json`、`serial_config.json`、`logs/` 和 `screenshots/` 由本机维护。反馈问题时只提供对应时段的必要内容。
