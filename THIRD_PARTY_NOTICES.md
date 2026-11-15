# 第三方组件与出处

本仓库根目录的 MIT 适用于项目原创内容。带第三方版权声明的文件沿用文件头和下列许可证。

| 组件 | 仓库位置与许可 |
| --- | --- |
| STMicroelectronics STM32H7 HAL | [HAL 许可证](Drivers/STM32H7xx_HAL_Driver/License.md) |
| Arm CMSIS Core | [CMSIS 许可证](Drivers/CMSIS/LICENSE.txt) |
| STM32H7 Device、ST 启动和生成代码 | 文件头的 STMicroelectronics 声明及 [BSD 3-Clause](LICENSES/BSD-3-Clause.txt) |
| FreeRTOS 内核与移植层 | [源文件中的原始 MIT 声明](Middlewares/Third_Party/FreeRTOS/Source/tasks.c) |
| CMSIS-RTOS2 适配 | [源文件声明](Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c)及 [Apache-2.0](Drivers/CMSIS/LICENSE.txt) |
| Microsoft Fluent Emoji 相机图标 | [图标出处](vision/pc_vision/assets/SOURCE.md)及 [MIT 声明](vision/pc_vision/assets/LICENSE-fluentui-emoji.txt) |

Python 包依赖通过 [requirements.txt](vision/pc_vision/requirements.txt) 安装。MaixPy 运行环境由设备提供，Keil 和 MATLAB 由使用者安装。比赛规则以主办方发布的文件为准，器件手册从各厂商获取。图像回归使用 [回归样本](vision/pc_vision/test_data/README.md) 中注明用途的素材。
