# MATLAB 导航模型

`mapspec/field_mapspec.json` 描述场地，`matlab/` 实现栅格、车体运动学和导航仿真。模型用于对比路线和转角，可与现有固件规划的不同版本分别核对。

从仓库根目录运行 `matlab -batch "addpath('sim/matlab'); test_build_grid; test_kinematics"`。画图可改为 `sim_png`。在 Windows 上也可修改 `tools/run_sim.bat` 的 MATLAB 安装路径，再用 `test`、`report` 或 `png` 参数运行。输出在 `sim/out/`。

图示是模型输出。实车打滑、装载偏移和时序需在车辆上单独测量。
