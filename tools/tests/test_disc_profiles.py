"""Final-gripper protocol/UI checks with fake serial, never real hardware."""
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'vision/pc_vision'), str(ROOT/'vision/maixcam_service')]
from arm_control import ArmState, move_command, point_pose
from serial_monitor import SerialLog, command_packet
from test_arm_control import fixture


def publish(state, overrides=None):
    state.receive('[ARM DISC POINTS] BEGIN', time.perf_counter())
    rows = [('wait', -562.5272, -185.1581, -2.0217), ('grab', -562.5272, -287.476, -2.0217), ('tray', -405.2134, 1, 162.825)]
    if state.data.get('version', {}).get('disc_pose_count') == '4': rows.append(('traypick', -480.59, -4, 162.825))
    for kind, l, r, d in rows:
        line = '[ARM DISC POINT] pose=%s base_l=%s base_r=%s base_d=%s dl=0 dr=0 dd=0' % (kind, l, r, d)
        state.receive((overrides or {}).get(kind, line), time.perf_counter())
    state.receive('[ARM DISC POINTS] END', time.perf_counter())


class DiscProfiles(unittest.TestCase):
    def test_parameter_profile_button_prepares_matching_code_without_motion(self):
        import tkinter as tk
        from tkinter import ttk
        from arm_panel import ArmPanel
        root = tk.Tk(); root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log = SerialLog(temp); log.connected = True; log.session = 'fake'
                state = fixture(log); state.data['version'].update(firmware='243', disc_pose_count='4'); publish(state)
                tabs = ttk.Notebook(root); page = ttk.Frame(tabs); tabs.add(page, text='机械臂')
                panel = ArmPanel(page, log); panel.last_poll = time.perf_counter(); panel.poll()
                self.assertTrue(log.outbound.empty()); panel.switch_profile()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm profile prelim')
                self.assertTrue(log.outbound.empty())
                log._record('RX', '[ARM PROFILE] READY target=PRELIM active=FINAL HAL=0; no motion')
                panel.actions.update(log.arm_snapshot()); self.assertIsNone(panel.actions.active)
                self.assertIn('当前仍运行决赛代码', panel.actions.message)
                panel.profile_choice.set('决赛参数（供决赛代码）'); panel.switch_profile()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm profile final')
                self.assertTrue(log.outbound.empty())
                panel.actions.cancel(); panel.data['version'].update(firmware='245', variant='PRELIM')
                panel.switch_profile()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm profile final')
                log._record('RX', '[ARM PROFILE] READY target=FINAL active=PRELIM HAL=0; no motion')
                panel.actions.update(log.arm_snapshot())
                self.assertIn('当前仍运行初赛代码', panel.actions.message)
                self.assertTrue(log.outbound.empty())
                panel.profile_choice.set('初赛参数（供初赛代码）'); panel.switch_profile()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm profile prelim')
                log._record('RX', '[ARM PROFILE] READY target=PRELIM active=PRELIM HAL=0; no motion')
                panel.actions.update(log.arm_snapshot())
                self.assertEqual(log.outbound.get_nowait()[0], 'arm points')
                panel.actions.cancel(); panel.data['version']['firmware']='237'
                with self.assertRaises(ValueError): panel.switch_profile()
                self.assertTrue(log.outbound.empty()); self.assertIsNone(log.thread)
        finally: root.destroy()

    def test_v239_waits_for_all_four_poses(self):
        state = fixture(); state.data['version'].update(firmware='239', disc_pose_count='4')
        state.receive('[ARM DISC POINTS] BEGIN', time.perf_counter())
        for kind in ('wait', 'grab', 'tray'):
            state.receive('[ARM DISC POINT] pose=%s base_l=-400 base_r=-10 base_d=0 dl=0 dr=0 dd=0' % kind, time.perf_counter())
        state.receive('[ARM DISC POINTS] END', time.perf_counter())
        self.assertEqual(state.disc_points, {})
        publish(state)
        self.assertEqual(set(state.disc_points), {'wait', 'grab', 'tray', 'traypick'})

    def test_atomic_complete_profile_and_invalid_refresh_keeps_previous(self):
        state = ArmState()
        state.receive('[ARM DISC POINTS] BEGIN', 1)
        state.receive('[ARM DISC POINT] pose=wait base_l=-500 base_r=-180 base_d=0 dl=0 dr=0 dd=0', 1)
        state.receive('[ARM DISC POINTS] END', 1)
        self.assertEqual(state.disc_points, {})
        publish(state)
        self.assertEqual(set(state.snapshot()['disc_points']), {'wait', 'grab', 'tray'})
        previous = state.snapshot()['disc_points']
        publish(state, {'grab': '[ARM DISC POINT] pose=grab base_l=nan base_r=-280 base_d=0 dl=0 dr=0 dd=0'})
        self.assertEqual(state.disc_points, previous)

    def test_reference_allowance_is_limited_to_explicit_tray_preview(self):
        data = fixture().data
        with self.assertRaises(ValueError): move_command(data, (-405, 1, 162))
        self.assertEqual(move_command(data, (-405, 1, 162), True), 'arm move -405.000 1.000 162.000')
        with self.assertRaises(ValueError): move_command(data, (-405, 20, 162), True)

    def test_new_commands_fit_existing_uart(self):
        for command in ('valign run plate', 'arm disc params save', 'arm disc params load', 'arm disc pose tray',
                        'arm disc trim grab -300.000 -300.000 -180.000', 'arm material once 6 3'):
            self.assertEqual(command_packet(command)[1], (command+'\r\n').encode('ascii'))

    def test_panel_preview_apply_and_named_actions_do_not_open_serial(self):
        import tkinter as tk
        from tkinter import ttk
        from arm_panel import ArmPanel
        root = tk.Tk(); root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log = SerialLog(temp); log.connected = True; log.session = 'fake'
                state = fixture(log); state.data['version'].update(firmware='239', disc_pose_count='4'); publish(state)
                tabs = ttk.Notebook(root); page = ttk.Frame(tabs); tabs.add(page, text='机械臂')
                panel = ArmPanel(page, log); panel.last_poll = time.perf_counter(); panel.poll()
                self.assertIsNone(log.thread); self.assertTrue(log.outbound.empty())
                self.assertIn('圆盘 / 圆环取放', [panel.tabs.tab(t, 'text') for t in panel.tabs.tabs()])
                panel.disc_offsets['6'].set(-10)
                self.assertTrue(log.outbound.empty())
                base, target = panel.disc_target(); self.assertEqual(target[1], base[1]-10)
                panel.apply_disc()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm disc trim wait 0.000 -10.000 0.000')
                self.assertTrue(log.outbound.empty())
                panel.actions.cancel(); panel.disc_kind.set('放回载物盘'); panel.select_disc(); panel.move_disc()
                self.assertTrue(log.outbound.get_nowait()[0].startswith('arm move -405.213 1.000'))
                panel.actions.cancel(); panel.align_plate()
                self.assertEqual(log.outbound.get_nowait()[0], 'valign run plate')
                self.assertTrue(log.outbound.empty())
                log._record('RX', '[MANUAL ALIGN] DONE station=1 reason=complete')
                panel.actions.update(log.arm_snapshot())
                self.assertEqual(log.outbound.get_nowait()[0], 'arm disc pose wait')
                self.assertIsNotNone(panel.actions.active)
                log._record('RX', '[ARM MOVE] READY: requested three-axis pose reached; motors hold')
                panel.actions.update(log.arm_snapshot())
                self.assertIsNone(panel.actions.active)
                self.assertTrue(log.outbound.empty())
                panel.actions.cancel(); panel.disc_color.set('6 黑色'); panel.disc_slot.set('3'); panel.disc_cycle()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm material once 6 3')
                panel.actions.cancel(); panel.disc_kind.set('载物盘取料（圆环放置前）'); panel.select_disc()
                panel.disc_offsets['6'].set(-5); panel.apply_disc()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm disc trim traypick 0.000 -5.000 0.000')
                panel.actions.cancel(); panel.move_disc_stored('traypick')
                self.assertEqual(log.outbound.get_nowait()[0], 'arm disc pose traypick')
        finally: root.destroy()

    def test_failed_plate_alignment_does_not_move_to_wait_pose(self):
        from arm_control import ArmActions
        commands = []
        actions = ArmActions(commands.append)
        actions.start([('valign run plate', '[MANUAL ALIGN] DONE'), ('arm disc pose wait', '[ARM MOVE] READY')], 0)
        state = ArmState()
        state.receive('[MANUAL ALIGN] FAILED station=1 reason=camera pose failed', time.perf_counter())
        actions.update(state.snapshot())
        self.assertEqual(commands, ['valign run plate'])
        self.assertIsNone(actions.active)


if __name__ == '__main__': unittest.main()
