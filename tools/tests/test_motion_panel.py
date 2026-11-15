import sys,time,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
from motion_panel import FIELDS,MotionState,MotionPanel,apply_command
from serial_monitor import SerialLog,command_packet


def rows(accel=600):
    return ['[MOTION BEGIN] firmware=250 variant=PRELIM source=W25Q64 sequence=2']+[
        '[MOTION VALUE] key=%s value=%s ref=%s'%(key,accel if key=='accel' else ref,ref) for key,_,_,ref,*_ in FIELDS]+['[MOTION END]']


class MotionTests(unittest.TestCase):
    def test_atomic_complete_status(self):
        state=MotionState()
        for line in rows():state.receive(line)
        self.assertEqual(state.data['values']['straight'],1.03)
        previous=state.data
        state.receive('[MOTION BEGIN] firmware=250')
        state.receive('[MOTION VALUE] key=spd value=nan')
        state.receive('[MOTION END]')
        self.assertIs(state.data,previous)
    def test_command_units_and_invalid_batch(self):
        values={key:ref for key,_,_,ref,*_ in FIELDS}
        command=apply_command(values)
        self.assertIn('0.65 306.8352 600 292 900 1.03 500 300 1930',command)
        self.assertLessEqual(len(command),127)
        self.assertTrue(command_packet(command)[1].endswith(b'\r\n'))
        largest={key:hi-.000001 for key,_,_,_,_,hi in FIELDS}
        long_command=apply_command(largest)
        self.assertGreater(len(long_command),63);self.assertLessEqual(len(long_command),127)
        command_packet(long_command)
        values['straight']='nan'
        with self.assertRaises(ValueError):apply_command(values)
    def test_panel_never_sends_until_click_and_waits_for_ack(self):
        import tkinter as tk
        from tkinter import ttk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log=SerialLog(temp);log.connected=True;log.session='fake'
                log.arm.data={'version':{'firmware':'250','variant':'PRELIM'}}
                parent=ttk.Frame(root);parent.pack();panel=MotionPanel(parent,log)
                self.assertTrue(log.outbound.empty())
                panel.references();self.assertTrue(log.outbound.empty())
                panel.read();self.assertEqual(log.outbound.get_nowait()[0],'arm ui')
                log._record('RX','[ARM UI END]');panel.parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0],'motion status')
                for line in rows():log._record('RX',line)
                panel.parent.after_cancel(panel._after);panel.poll()
                self.assertIsNone(panel.actions.active)
                self.assertEqual(panel.current['accel'].get(),'600')
                panel.entries['accel'].set('700');panel.apply_all()
                self.assertIn(' 700 ',log.outbound.get_nowait()[0])
                self.assertTrue(log.outbound.empty())
                log._record('RX','[MOTION APPLY] OK; RAM only')
                panel.parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0],'motion status')
                for line in rows(700):log._record('RX',line)
                panel.parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(panel.current['accel'].get(),'700')
                panel.save();self.assertEqual(log.outbound.get_nowait()[0],'motion save')
                log._record('RX','[MOTION SAVE] FAILED')
                panel.parent.after_cancel(panel._after);panel.poll()
                self.assertIsNone(panel.actions.active);self.assertTrue(log.outbound.empty())
                log.connected=False
                with self.assertRaises(ValueError):panel.load()
                self.assertTrue(log.outbound.empty());self.assertIsNone(log.thread)
        finally:root.destroy()

    def test_unknown_firmware_is_queried_and_old_firmware_is_rejected(self):
        import tkinter as tk
        from tkinter import ttk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log=SerialLog(temp);log.connected=True;log.session='fresh'
                parent=ttk.Frame(root);panel=MotionPanel(parent,log)
                panel.read();self.assertEqual(log.outbound.get_nowait()[0],'arm ui')
                log.arm.data={'version':{'firmware':'249','variant':'PRELIM'}}
                log._record('RX','[ARM UI END]')
                panel.parent.after_cancel(panel._after);panel.poll()
                self.assertIsNone(panel.actions.active);self.assertTrue(log.outbound.empty())
                self.assertIn('V250',panel.actions.message)
        finally:root.destroy()

    def test_switch_report_validation_and_independence_from_motion_W25(self):
        state=MotionState()
        for line in rows():state.receive(line)
        motion=state.data
        for arc in (0,1):
            for avoid in (0,1):
                state.receive('[NAV SWITCH] v=1 firmware=271 variant=PRELIM arc=%d avoid=%d trigger_mm=420 min_rpm=15'%(arc,avoid))
                self.assertEqual(state.switches['arc'],str(arc));self.assertEqual(state.switches['avoid'],str(avoid))
                self.assertIs(state.data,motion)
        previous=state.switches
        for line in ('[NAV SWITCH] REJECTED: busy','[NAV SWITCH] v=1 firmware=270 variant=PRELIM arc=0 avoid=0',
                     '[NAV SWITCH] v=1 firmware=271 variant=FINAL arc=0 avoid=0','[NAV SWITCH] v=1 firmware=271 variant=PRELIM arc=0 avoid=2'):
            state.receive(line);self.assertIs(state.switches,previous)
    def test_switch_controls_only_apply_after_click_and_controller_confirmation(self):
        import tkinter as tk
        from tkinter import ttk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log=SerialLog(temp);log.connected=True;log.session='fake'
                log.arm.data={'version':{'firmware':'271','variant':'PRELIM'}}
                parent=ttk.Frame(root);panel=MotionPanel(parent,log)
                panel.arc_enabled.set(False);self.assertTrue(log.outbound.empty())
                with self.assertRaises(ValueError):panel.apply_switches()
                panel.read_switches();self.assertEqual(log.outbound.get_nowait()[0],'arm ui')
                log._record('RX','[ARM UI END]');parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0],'navswitch status')
                log._record('RX','[NAV SWITCH] v=1 firmware=298 variant=PRELIM arc=1 avoid=1 trigger_mm=420 min_rpm=15 stop_mode=RAMP')
                parent.after_cancel(panel._after);panel.poll();self.assertIsNone(panel.actions.active)
                self.assertIn('开启',panel.switch_status.get())
                self.assertIn('按普通减速度斜坡停车',panel.switch_status.get())
                for arc,avoid in ((0,0),(0,1),(1,0),(1,1)):
                    panel.arc_enabled.set(bool(arc));panel.avoid_enabled.set(bool(avoid));self.assertTrue(log.outbound.empty())
                    panel.apply_switches();self.assertEqual(log.outbound.get_nowait()[0],'navswitch set %d %d'%(arc,avoid))
                    log._record('RX','[NAV SWITCH] v=1 firmware=271 variant=PRELIM arc=%d avoid=%d trigger_mm=420 min_rpm=15'%(arc,avoid))
                    parent.after_cancel(panel._after);panel.poll()
                    self.assertIsNone(panel.actions.active);self.assertEqual(panel.state.switches['avoid'],str(avoid))
                panel.arc_enabled.set(False);panel.apply_switches();log.outbound.get_nowait()
                log._record('RX','[NAV SWITCH] REJECTED: idle chassis required')
                parent.after_cancel(panel._after);panel.poll()
                self.assertIsNone(panel.actions.active);self.assertEqual(panel.state.switches['arc'],'1')
                self.assertTrue(log.outbound.empty())
                log.session='new';parent.after_cancel(panel._after);panel.poll()
                self.assertIsNone(panel.state.switches)
                with self.assertRaises(ValueError):panel.apply_switches()
        finally:root.destroy()
    def test_old_firmware_switch_controls_are_rejected_without_sending_unknown_command(self):
        import tkinter as tk
        from tkinter import ttk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log=SerialLog(temp);log.connected=True;log.session='fake'
                log.arm.data={'version':{'firmware':'270','variant':'PRELIM'}}
                parent=ttk.Frame(root);panel=MotionPanel(parent,log)
                panel.read_switches();self.assertEqual(log.outbound.get_nowait()[0],'arm ui')
                log._record('RX','[ARM UI END]');parent.after_cancel(panel._after);panel.poll()
                self.assertTrue(log.outbound.empty());self.assertIsNone(panel.actions.active)
                self.assertIn('V271',panel.actions.message)
        finally:root.destroy()

    def test_v272_requires_all_five_heading_values(self):
        state=MotionState();data=rows();data[0]=data[0].replace('250','272')
        for line in data:state.receive(line)
        self.assertEqual(state.data['values']['hold_kp'],.03)
        self.assertEqual(state.data['values']['turn_stable'],100)
        other=MotionState()
        for line in data[:-6]+[data[-1]]:other.receive(line)
        self.assertIsNone(other.data)

    def test_v272_apply_all_sends_heading_batch_before_status_and_save(self):
        import tkinter as tk
        from tkinter import ttk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log=SerialLog(temp);log.connected=True;log.session='fake'
                log.arm.data={'version':{'firmware':'272','variant':'PRELIM'}}
                parent=ttk.Frame(root);panel=MotionPanel(parent,log)
                for line in rows():panel.state.receive(line.replace('firmware=250','firmware=272'))
                panel.entries['hold_kp'].set('.04');panel.entries['turn_stable'].set('150');panel.apply_all()
                self.assertTrue(log.outbound.get_nowait()[0].startswith('motion apply '))
                log._record('RX','[MOTION APPLY] OK');parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0],'motion heading 0.04 0.05 0.2 1 150')
                log._record('RX','[MOTION HEADING] OK');parent.after_cancel(panel._after);panel.poll()
                self.assertEqual(log.outbound.get_nowait()[0],'motion status')
        finally:root.destroy()

if __name__=='__main__':unittest.main()
