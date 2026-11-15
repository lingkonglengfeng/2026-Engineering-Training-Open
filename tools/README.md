# 打包、分析与验证工具

从仓库根目录运行脚本。图像工具使用 [PC 依赖](../vision/pc_vision/requirements.txt)，固件离线回归另需 [测试依赖](tests/requirements.txt)。

| 目标 | 命令或入口 |
| --- | --- |
| 生成 CAM1 和 CAM2 桥接包 | `python tools/package_pc_vision_bridge.py` |
| 生成单一相机桥包 | 同上追加 `--camera material` 或 `--camera qr` |
| 查看圆环或物料识别 | `validate_ring_*.py`、`analyze_material_surfaces.py` 等，使用测试样本或脚本参数 |
| 查看地图与单位 | `tools/check_map.py`、`check_frame_signs.py`、`check_nav_units.py` |
| 串口日志 | [串口工具](serial_console/README.md)和上位机串口日志页 |
| 自动检查 | [测试入口与依赖](tests/README.md) |
| MATLAB 导航模型 | [仿真说明](../sim/README.md) |

生成的安装包保存在组件的 `dist/`，报告通常写入 `output/` 或 `sim/out/`。更新相机公共协议后重新运行打包，核对生成包的应用角色和版本。
