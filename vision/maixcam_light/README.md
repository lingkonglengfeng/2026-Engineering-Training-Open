# A19 50%硬件PWM补光试验

本目录用于相机端算法开发、专项调试或历史对照。整车双路部署从 [视觉总览](../README.md)进入。

当前统一入口已升级v12（2026-10-05-plate-scene-v12），继续20kHz/50%PWM，增加0x85圆盘健康/搜索状态与0x09位置同图配对；1号搜索需配套新主控。未增加灯控命令，也不每帧自动调亮暗。以下v11灯控参数保持。

最新：用户指定直接用统一入口并用USB实车测试。当前vision/maixcam_service/main.py已是v11，默认A19/PWM7、20kHz/50%，包含弱中环修正与0x83，无需另选main_pwm50.py。本文下方v10的“统一标准GPIO恒亮”说明已被该要求替代；独立标准文件仍恒亮。主控灯命令尚未实现，亮度是启动固定设置。203项检查通过，未操作真实设备。

用户已确认灯板支持PWM并授权选择频率，初始试验选20kHz、50%；实际光学效果尚未验证。已有包含识别算法的单文件副本：

| 用途 | 文件 |
|---|---|
| 主控统一服务（含v10弱环、SET/ACK/0x83） | main_pwm50.py |
| 独立六色物料 | material_pwm50.py |
| 独立圆盘切点 | plate_pwm50.py |
| 独立圆环（含v10弱环，不接SET） | middle_ring_pwm50.py |

一次只运行一个程序，先停车/停止原程序，用USB测试。vision目录正常源程序默认LIGHT_PWM_ENABLED=False，仍GPIO恒亮；试验副本True，启动硬件PWM，退出/初始化失败先禁用PWM，再复用已持有GPIO恢复低电平，不重复申请引脚。无需另传辅助py，生成工具tools/build_light_pwm_trial.py。

和电控讨论结论：主控决定工位/流程的开关及目标亮度，Maix执行、AE和质量观测；首版固定亮度，不每帧自动调灯。将来独立LIGHT消息与ACK状态不能占SET flags或覆盖成功SET序号，当前尚未新增灯控协议。备用MCU硬件PWM需核对真实TIM引脚，PD1/PD2不能当PWM，TIM4已有50Hz舵机，不改变它的频率。

单独打开并上传`pwm50_preview.py`，先停止其他视觉程序，用USB连接MaixVision。默认PWM7、占空比50%、频率20kHz（可修改试验值，需先确认灯板允许范围）。不使用Python循环翻转GPIO，也不发送车辆串口。显示无标记的彩色相机画面；60秒或停止程序后尽力禁用PWM、恢复A19为GPIO低电平关闭。

这是硬件/灯板适配试验，尚未在设备上执行；50%占空比不保证灯板实测亮度正好50%。当前正式视觉入口仍沿用启动恒亮配置，PWM试验副本独立提供。

[官方PWM说明](https://wiki.sipeed.com/maixpy/doc/zh/peripheral/pwm.html)明确普通MaixCAM A19对应PWM7，同时提示PWM4~9不建议在Wi-Fi运行时使用。程序不自动改系统Wi-Fi配置，也不承诺PWM关闭后一定恢复网络；因此使用USB做该试验。
