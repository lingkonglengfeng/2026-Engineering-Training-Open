# STM32 主控

工程入口在 [MDK-ARM](../MDK-ARM/README.md)，任务调度从 `Src/freertos.c` 开始。生成引脚、时钟和外设配置以 [CubeMX 工程](../Engineering_Training_Project.ioc) 为入口。

| 子系统 | 修改入口 |
| --- | --- |
| 比赛任务 | `match_control.c` 状态机、`zone2_route.c` 任务阶段与工位 |
| 地图和导航 | `grid_nav.c` 地图、障碍和栅格规划，`nav_frame.c` 坐标，`nav_odometry.c` 里程，`nav_blend.c` 圆弧 |
| 底盘驱动 | `chassis_motor.c` 四轮输出，`zdt_motor.c` CAN 与导航调度 |
| 机械臂 | `arm_start_pose.c` 准备姿态，`arm_material.c` 圆盘取料，`arm_round1.c` 圆环取放，`arm_driver.c`／`dm_motor.c` 电机，`arm_servo.c` 舵机 |
| 视觉 | `vision.c`、`vision_qr.c` 协议，`vision_align.c` 纠偏，`vision_obstacle.c` 障碍观测与几何 |
| 参数 | `arm_calibration.c` 标定与参数区，`nav_params.c` 底盘参数，`w25q64.c` Flash 读写，`competition_profile.h` 赛制默认值 |

上层运动逻辑应走现有底盘／机械臂接口，串口字符串和字段由 PC 与固件共同消费。地图单位是毫米，X 向右、Y 向上。像素图像 Y 向下。电机接口可能采用 RPM、编码器计数或电机角度，不能和车体速度或行程直接互换。
