# 离线测试

在仓库根目录执行。Python 测试使用提交中的图像样本和模拟链路。

```powershell
python -m pip install -r vision/pc_vision/requirements.txt
python -m unittest discover -s tools/tests -p test_pc_vision.py
python -m unittest discover -s tools/tests -p test_pc_obstacles.py
```

固件回归需将 `arm-none-eabi-gcc` 加入 PATH，并执行 `python -m pip install -r tools/tests/requirements.txt`。按改动选择 `run_*.py`。这些运行器使用 Unicorn 加载测试程序，Keil 构建另见 [固件构建](../../MDK-ARM/README.md)。记录通过数、失败、测试提交和执行环境。
