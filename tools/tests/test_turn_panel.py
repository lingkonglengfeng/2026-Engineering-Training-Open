"""转向 PID 面板的“调参有没有真的下发”检查（Tk 无头，不连串口）。

覆盖现场最容易踩的坑：
  * 只改输入框没点“应用到RAM” → 点转向必须被拦截，不能悄悄按旧 PID 跑；
  * 主控回报的来源（DEFAULT/W25Q64/MANUAL）要在状态行里显示；
  * 应用/读取后允许正常转向。

运行：
    python -m unittest discover -s tools/tests -p test_turn_panel.py
"""
from pathlib import Path
import sys
import threading
import tkinter as tk
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'vision/pc_vision'))
from turn_panel import TurnPanel  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.connected = True
        self.stop_event = threading.Event()
        self.session = 's1'
        self.sent = []
        self.status = dict(enabled=1, active=0, imu=1, trace=1, rpm=180.0,
                           tgt=90.0, act=89.5, err=0.5, out=0.1,
                           kp=0.05, ki=0.0004, kd=0.003, src='W25Q64', received=1.0)
        self.pid = dict(kp=0.05, ki=0.0004, kd=0.003, source='W25Q64', received=1.0)

    def send(self, text):
        self.sent.append(text)

    def turn_snapshot(self):
        return {'samples': [], 'status': self.status, 'start': None, 'done': None,
                'pid': self.pid, 'pid_note': None, 'trace': None, 'reject': None,
                'active': False, 'supported': True}

    def turn_clear(self):
        pass


class TurnPanelGains(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.logger = FakeLogger()
        self.panel = TurnPanel(self.root, self.logger)
        self.root.update()

    def tearDown(self):
        self.panel.closed = True
        if self.panel._after is not None:
            try:
                self.root.after_cancel(self.panel._after)
            except tk.TclError:
                pass
            self.panel._after = None
        self.root.destroy()

    def test_unapplied_edit_blocks_turn_command(self):
        self.panel.applied = dict(kp=0.05, ki=0.0004, kd=0.003)
        self.panel.gains['kp'].set('0.060')          # 只改输入框，没点应用
        before = len(self.logger.sent)
        with self.assertRaises(ValueError) as ctx:
            self.panel.send_turn_command('&L90')
        self.assertIn('应用到RAM', str(ctx.exception))
        self.assertEqual(len(self.logger.sent), before)  # 一条都没发出去

    def test_applied_edit_allows_turn_command(self):
        self.panel.applied = dict(kp=0.05, ki=0.0005, kd=0.0035)
        self.panel.gains['kp'].set('0.050')
        self.panel.gains['ki'].set('0.0005')
        self.panel.gains['kd'].set('0.0035')
        self.panel.send_turn_command('&L90')
        self.assertEqual(self.logger.sent[-1], '&L90')
        self.assertTrue(self.panel.busy)

    def test_apply_sends_three_no_space_commands_and_marks_applied(self):
        self.panel.gains['kp'].set('0.050')
        self.panel.gains['ki'].set('0.0004')
        self.panel.gains['kd'].set('0.003')
        self.panel.apply_gains()
        self.assertEqual(self.logger.sent, ['&kp0.05', '&ki0.0004', '&kd0.003'])
        self.assertAlmostEqual(self.panel.applied['kp'], 0.05)

    def test_source_text_is_visible_in_status(self):
        self.panel.refresh()
        text = self.panel.status.get()
        self.assertIn('W25 已保存', text)
        self.assertIn('180', text)                   # 轮速上限

    def test_pid_reply_updates_applied_and_entries(self):
        self.logger.pid = dict(kp=0.06, ki=0.0003, kd=0.002, source='MANUAL', received=2.0)
        self.panel.refresh()
        self.assertAlmostEqual(self.panel.applied['kp'], 0.06)
        self.assertAlmostEqual(float(self.panel.gains['kp'].get()), 0.06)
        self.assertEqual(self.panel.source, 'MANUAL')


if __name__ == '__main__':
    unittest.main()
