"""Arm UI model, UART acknowledgements and Tk callbacks; no real devices."""
from pathlib import Path
import gc
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'vision/pc_vision'), str(ROOT/'vision/maixcam_service')]
from arm_control import ArmState, ArmActions, axis_command, axis_steps, move_command, point_pose, displacement
from serial_monitor import SerialLog, command_packet


def fixture(log=None, prefix=''):
    state = ArmState() if log is None else log.arm
    now = time.perf_counter()
    lines = [
        '[ARM UI BEGIN] v=1 firmware=196',
        '[ARM UI STATE] busy=0 pose=0 calibration=0 material=0 round=0',
        '[ARM UI AXIS] id=5 valid=1 angle=-10 span=-219505 travel=240 enabled=1',
        '[ARM UI AXIS] id=6 valid=1 angle=-200 span=-132619.377777778 travel=178 enabled=1',
        '[ARM UI ROT] valid=1 angle=0 min=-180 max=185 enabled=1',
        '[ARM UI GUARD] enabled=1 top=1 threshold=-150 min=-44 max=44 height=-2 valid=1',
        '[ARM] grip enabled=1 pwm_angle=225 target_angle=225 ramping=0',
        '[ARM] table enabled=1 pwm_angle=295 target_angle=295 ramping=0',
        '[ARM PRESETS] open=225 close=308 table1=205 table2=295 table3=25',
        '[ARM UI END]',
    ]
    for area in ('rough', 'temp'):
        for ring, reach, dm in ((1, -545.2084, 41.3044), (2, -212.0919, 4.604), (3, -290.4869, -39.5218)):
            lines.append('[ARM POINT] area=%s ring=%s base_l=-1032.6 base_r=%s base_d=%s dl=0 dr=0 dd=0' % (area, ring, reach, dm))
    lines.append('[ARM POINTS] END')
    for line in lines:
        if log is None: state.receive(prefix+line, now)
        else: log._record('RX', prefix+line)
    return state


class ModelTests(unittest.TestCase):
    def test_hardware_enabled_axis_still_requests_software_enable_before_target(self):
        for axis, target, expected in (('5', -282.085439, 'arm pct 5 23.3945'),
                                       ('6', -155.948012, 'arm pos 6 -155.948')):
            with self.subTest(axis=axis):
                state = fixture(); sent = []; chain = ArmActions(sent.append)
                # V196 hardware enabled=1 does not imply enable_req=1.
                self.assertEqual(state.data['axes'][axis]['enabled'], '1')
                chain.start(axis_steps(state.data, axis, target, lambda: state.data), state.sequence)
                self.assertEqual(sent, ['arm enable '+axis])
                state.receive('ZDT> [ARM] HAL=0 (0=queued,1=error,2=busy,3=timeout)', time.perf_counter())
                chain.update(state.snapshot())
                self.assertEqual(sent, ['arm enable '+axis, 'arm ui'])
                self.assertNotIn(expected, sent)
                state.data = fixture().data
                state.receive('[ARM UI END]', time.perf_counter()); chain.update(state.snapshot())
                self.assertEqual(sent[-1], expected)

    def test_disabled_rotation_is_enabled_then_uses_new_actual_angle(self):
        state = fixture(); state.data['rotation'].update(valid='0', enabled='0', angle='0')
        sent = []; chain = ArmActions(sent.append)
        chain.start(axis_steps(state.data, 'dm', 24.8949172, lambda: state.data), state.sequence)
        self.assertEqual(sent, ['arm enable dm'])
        state.receive('[DM] ON confirmed; motor enabled.', time.perf_counter()); chain.update(state.snapshot())
        self.assertEqual(sent[-1], 'arm ui')
        state.data = fixture().data
        state.data['rotation'].update(angle='134.5850', valid='1', enabled='1')
        state.receive('[ARM UI END]', time.perf_counter()); chain.update(state.snapshot())
        self.assertEqual(sent, ['arm enable dm', 'arm ui', 'arm rotate 24.895'])

    def test_enable_failure_or_invalid_new_feedback_never_sends_motion(self):
        state = fixture(); sent = []; chain = ArmActions(sent.append)
        chain.start(axis_steps(state.data, 'dm', 25, lambda: state.data), state.sequence)
        state.receive('[DM] ON FAILED HAL=1 state=0 fb=0', time.perf_counter()); chain.update(state.snapshot())
        self.assertEqual(sent, ['arm enable dm']); self.assertIsNone(chain.active)
        self.assertIn('ON FAILED', chain.message)
        state = fixture(); sent = []; chain = ArmActions(sent.append)
        chain.start(axis_steps(state.data, 'dm', 25, lambda: state.data), state.sequence)
        state.receive('[DM] ON confirmed; motor enabled.', time.perf_counter()); chain.update(state.snapshot())
        state.data = fixture().data; state.data['rotation']['valid'] = '0'
        state.receive('[ARM UI END]', time.perf_counter()); chain.update(state.snapshot())
        self.assertEqual(sent, ['arm enable dm', 'arm ui']); self.assertIsNone(chain.active)
        self.assertIn('旋转实际角度无效', chain.message)

    def test_low_descent_queries_actual_rotation_and_keeps_height_interlock(self):
        state = fixture(); sent = []; chain = ArmActions(sent.append)
        chain.start(axis_steps(state.data, '5', -1000, lambda: state.data), state.sequence)
        self.assertEqual(sent, ['dm query'])
        for line in ('[DM] QUERY raw_deg=70 arm_deg=70', '[ARM] HAL=0'):
            state.receive(line, time.perf_counter()); chain.update(state.snapshot())
        self.assertEqual(sent, ['dm query', 'arm enable 5', 'arm ui'])
        state.data = fixture().data; state.data['rotation']['angle'] = '70'
        state.receive('[ARM UI END]', time.perf_counter()); chain.update(state.snapshot())
        self.assertFalse(any(command.startswith('arm pct') for command in sent))
        self.assertIsNone(chain.active); self.assertIn('允许的角度区间', chain.message)

    def test_single_lift_does_not_validate_unrelated_reach_position(self):
        data = fixture().data
        data['axes']['6'].update(angle='1.5', valid='0')
        data['rotation'].update(valid='0', enabled='0')
        self.assertEqual(axis_command(data, '5', -282.085439), 'arm pct 5 23.3945')
        with self.assertRaises(ValueError): axis_steps(data, 'dm', 200, lambda: data)

    def test_old_prompt_first_byte_time_does_not_discard_completed_enable_reply(self):
        with tempfile.TemporaryDirectory() as temp:
            log = SerialLog(temp); fixture(log)
            sent = []; chain = ArmActions(sent.append)
            chain.start(axis_steps(log.arm.data, '5', -50, lambda: log.arm_snapshot()['data']), log.arm.sequence)
            first_byte = {'perf_s': time.perf_counter()-10, 'wall_time': 'old prompt'}
            log._record('RX', 'ZDT> [ARM] HAL=0', first_byte)
            chain.update(log.arm_snapshot())
            self.assertEqual(sent, ['arm enable 5', 'arm ui'])
            self.assertEqual(log.history[-1]['perf_s'], first_byte['perf_s'])

    def test_real_uart_prompts_and_interleaved_obstacles_publish_complete_snapshot(self):
        original = fixture().snapshot()
        state = ArmState()
        for prefix in ('ZDT> ', 'ZDT> ZDT> ', '  ZDT>  '):
            for sequence, now, line in original['events']:
                if line.startswith('[ARM UI ROT]'):
                    line = '[ARM UI ROT] valid=0 angle=0.0000 min=-180.0000 max=185.0000 enabled=0'
                state.receive(prefix+line, now)
                state.receive('[VOBS STATE] v=1 session=29462 armed=1 eligible=0 x=2250 y=150 yaw=89.6', now)
            snapshot = state.snapshot()
            self.assertEqual(snapshot['data']['version']['firmware'], '196')
            self.assertEqual(snapshot['data']['axes']['5']['angle'], '-10')
            self.assertEqual(snapshot['data']['axes']['6']['angle'], '-200')
            self.assertEqual(snapshot['data']['rotation']['valid'], '0')
            self.assertEqual(snapshot['data']['presets']['open'], '225')
            self.assertEqual(len(snapshot['points']), 6)
        # Strip only leading console prompts, never markers quoted in other output.
        fresh = ArmState()
        fresh.receive('[OTHER] ZDT> [ARM UI BEGIN] v=1 firmware=196', 1)
        fresh.receive('[ARM UI END]', 2)
        self.assertFalse(fresh.data)

    def test_prompted_action_reply_advances_chain_and_log_keeps_original_text(self):
        with tempfile.TemporaryDirectory() as temp:
            log = SerialLog(temp)
            sent = []; chain = ArmActions(sent.append)
            chain.start([('arm trim rough 1 0 0 0', '[ARM TRIM] HAL=0'),
                         ('arm points', '[ARM POINTS] END')], 0)
            raw = 'ZDT> ZDT> [ARM TRIM] HAL=0; absolute per-area delta in RAM'
            log._record('RX', raw)
            chain.update(log.arm_snapshot())
            self.assertEqual(sent, ['arm trim rough 1 0 0 0', 'arm points'])
            self.assertEqual(log.history[-1]['text'], raw)
            log._record('RX', 'ZDT> [ARM POINTS] END')
            chain.update(log.arm_snapshot())
            self.assertIsNone(chain.active)

    def test_snapshot_is_complete_and_does_not_consume_serial_log(self):
        with tempfile.TemporaryDirectory() as temp:
            log = SerialLog(temp); fixture(log)
            first = log.arm_snapshot()
            self.assertEqual(len(first['points']), 6)
            self.assertEqual(first['data']['servos']['grip']['pwm_angle'], '225')
            self.assertGreater(len(log.drain()), 10)
            self.assertEqual(log.arm_snapshot()['data'], first['data'])
            first['data']['rotation']['angle'] = '99'
            self.assertEqual(log.arm_snapshot()['data']['rotation']['angle'], '0')
            log.arm.receive('[ARM UI BEGIN] v=1 firmware=196', time.perf_counter())
            log.arm.receive('[ARM UI AXIS] id=5 valid=1 angle=99', time.perf_counter())
            self.assertEqual(log.arm_snapshot()['data']['axes']['5']['angle'], '-10')
            log.arm.receive('[ARM UI END]', time.perf_counter())
            self.assertEqual(log.arm_snapshot()['data']['axes']['5']['angle'], '-10')

    def test_signed_encoder_angles_and_final_height_guard(self):
        data = fixture().data
        full5 = -219505*360/65536
        self.assertEqual(axis_command(data, '5', full5/2), 'arm pct 5 50.0000')
        self.assertAlmostEqual(displacement(data, '5', full5), 240)
        self.assertEqual(axis_command(data, '6', -200), 'arm pos 6 -200.000')
        self.assertTrue(move_command(data, (-10, -200, 120)).startswith('arm move'))
        with self.assertRaises(ValueError): move_command(data, (full5*.9, -200, 120))
        for bad in (float('nan'), float('inf'), 1, full5-1):
            with self.assertRaises(ValueError): axis_command(data, '5', bad)
        data['guard']['height'] = '-180'
        self.assertEqual(axis_command(data, 'dm', 44), 'arm rotate 44.000')
        for angle in (-44.01, 44.01, 186):
            with self.assertRaises(ValueError): axis_command(data, 'dm', angle)
        data['guard']['valid'] = '0'
        with self.assertRaises(ValueError): axis_command(data, 'dm', 0)

    def test_offsets_are_from_original_not_repeatedly_accumulated(self):
        point = fixture().points[('rough', '1')]
        base, pose = point_pose(point, (-2, -3, .2))
        self.assertAlmostEqual(pose[0]-base[0], -2)
        self.assertEqual(point_pose(point, (-2, -3, .2)), (base, pose))

    def test_ack_chain_waits_for_completion_and_cancels_failures(self):
        sent = []; chain = ArmActions(sent.append)
        chain.start([('arm pick', '[ARM RESULT]'), ('arm ring rough 1', '[ARM RESULT]')], 0)
        stamp = time.perf_counter()
        chain.update({'events': [(1, stamp, '[ARM PICK] request accepted')]})
        self.assertEqual(sent, ['arm pick'])
        chain.update({'events': [(2, time.perf_counter(), '[ARM RESULT] COMPLETE')]})
        self.assertEqual(sent, ['arm pick', 'arm ring rough 1'])
        chain.update({'events': [(3, time.perf_counter(), '[ARM RING] FAILED')]})
        self.assertIsNone(chain.active)
        self.assertIn('主控未完成', chain.message)
        chain.start([('arm trim rough 1 0 0 0', '[ARM TRIM] HAL=0'), ('arm ring rough 1', '[ARM RESULT]')], 3)
        chain.update({'events': [(4, time.perf_counter(), '[ARM TRIM] HAL=1')]})
        self.assertEqual(len(sent), 3)
        self.assertFalse(chain.steps)
        chain.start([('arm pick', '[ARM RESULT]'), ('arm ring rough 1', '[ARM RESULT]')], 4)
        chain.update({'events': [(5, time.perf_counter(), '[PC MANUAL CANCEL] &STOP')]})
        self.assertIsNone(chain.active)
        self.assertEqual(len(sent), 4)

    def test_generated_packets_fit_existing_uart(self):
        data = fixture().data
        for command in (move_command(data, (-1032.6, -545.2084, 41.3044)),
                        'arm trim rough 1 -80.000 -80.000 -12.000',
                        'arm table presets 205.000 295.000 25.000', 'valign run rough', 'arm params save'):
            self.assertLessEqual(len(command_packet(command)[1]), 65)


class PanelTests(unittest.TestCase):
    def tearDown(self): gc.collect()

    def test_ui_previews_without_motion_and_sends_selected_axis_only(self):
        import tkinter as tk
        from tkinter import ttk
        from arm_panel import ArmPanel
        root = tk.Tk(); root.withdraw()
        with tempfile.TemporaryDirectory() as temp:
            log = SerialLog(temp); log.connected = True; log.session = 'fake'
            fixture(log, prefix='ZDT> ')
            tabs = ttk.Notebook(root); tabs.pack()
            page = ttk.Frame(tabs); tabs.add(page, text='机械臂控制')
            panel = ArmPanel(page, log)
            try:
                self.assertEqual([value.get() for value in panel.slots], [19, 109, 199])
                self.assertEqual(panel.servos['table'].get(), 109)
                panel.last_poll = time.perf_counter()
                panel.poll()
                self.assertEqual(log.outbound.qsize(), 0)
                log.arm.data['state'].update(shared_ring='1', boot_params='W25Q64'); panel.poll()
                self.assertIn('启动参数：W25Q64 已保存参数', panel.status.get())
                labels = [panel.target_canvas.itemcget(item, 'text') for item in panel.target_canvas.find_all() if panel.target_canvas.type(item) == 'text']
                self.assertTrue(any('共用三点放置参数' in label for label in labels))
                panel.angles['dm'].set(20); panel.offsets['6'].set(-2); root.update_idletasks()
                self.assertEqual(log.outbound.qsize(), 0)
                panel.send_axis('dm')
                self.assertEqual(log.outbound.get_nowait()[0], 'arm enable dm')
                log._record('RX', '[DM] ON confirmed; motor enabled.'); panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm ui')
                fixture(log); panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm rotate 20.000')
                log._record('RX', '[ARM] HAL=0'); panel.poll()
                self.assertIsNone(panel.actions.active)
                self.assertIn('实际到位', panel.message.get())
                panel.angles['5'].set(-1000)
                panel.angles['dm'].set(120)
                with self.assertRaises(ValueError): panel.move()
                self.assertTrue(log.outbound.empty())
                panel.area.set('TEMP'); panel.ring.set('3'); panel.select_point()
                panel.offsets['dm'].set(.2)
                panel.apply_trim()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm trim temp 3 0.000 0.000 0.200')
                self.assertIsNotNone(panel.actions.active)
                log.connected = False; panel.poll()
                self.assertIsNone(panel.actions.active)
                self.assertEqual(panel.data, {})
                self.assertEqual(len(panel.target_canvas.find_all()) > 0, True)
            finally:
                root.destroy()

    def test_manual_refresh_queries_dm_without_enabling_or_moving(self):
        import tkinter as tk
        from tkinter import ttk
        from arm_panel import ArmPanel
        root = tk.Tk(); root.withdraw()
        with tempfile.TemporaryDirectory() as temp:
            log = SerialLog(temp); log.connected = True; log.session = 'fake'; fixture(log)
            tabs = ttk.Notebook(root); tabs.pack(); page = ttk.Frame(tabs); tabs.add(page, text='机械臂')
            panel = ArmPanel(page, log)
            try:
                panel.request_all()
                self.assertEqual([log.outbound.get_nowait()[0] for _ in range(3)], ['dm query', 'arm ui', 'arm points'])
                self.assertTrue(log.outbound.empty())
            finally: root.destroy()

    def test_full_application_exposes_control_tab_without_opening_uart(self):
        import tkinter as tk
        import gui
        root = tk.Tk(); root.withdraw()
        with tempfile.TemporaryDirectory() as temp, patch.object(gui, 'HERE', Path(temp)):
            app = gui.DualApp(root, Path(temp)/'cam1.json', Path(temp)/'cam2.json', autoconnect=False)
            try:
                self.assertEqual(len(app.notebook.tabs()), 5)
                self.assertEqual(app.notebook.tab(app.notebook.tabs()[-2], 'text'), '机械臂控制')
                self.assertIsNone(app.serial_log.thread)
            finally: app.close()


if __name__ == '__main__': unittest.main()
