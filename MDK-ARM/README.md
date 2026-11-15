# Keil 固件构建

1. 安装 Keil MDK、STM32H7 Device Pack 与支持 ARMCC 5.06 的工具链。
2. 打开 [Engineering_Training_Project.uvprojx](Engineering_Training_Project.uvprojx)，选择同名 target，执行 Build。
3. 检查构建日志。HEX 在 `Engineering_Training_Project/Engineering_Training_Project.hex`。
4. 使用 ST-Link/SWD 烧录到你的 STM32H723ZGT6。重启后在上位机读取主控版本，确认是 V245 决赛，再准备车辆。

命令行也可在本目录运行 `UV4.exe -b Engineering_Training_Project.uvprojx -j0 -o build.log`。仅构建不需要接车。Flash 参数分配见 [电控说明](../docs/electrical/README.md)。
