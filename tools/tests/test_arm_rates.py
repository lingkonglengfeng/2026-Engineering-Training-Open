"""Rate packets, atomic readback and the actual Tk point editor; no devices."""
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
from arm_rates import RateState,POINTS,AXES,rate_command,timeout_factor
from arm_control import ArmState,ArmActions

def packet(state,now=1.,pct=100):
    state.receive('[ARM RATES BEGIN] v=1 firmware=268 variant=PRELIM source=W25Q64 sequence=9',now)
    for p,_ in POINTS:state.receive('[ARM RATE] point='+p+' '+' '.join(k+'='+str(pct) for k,_ in AXES),now)
    state.receive('[ARM RATES END]',now)

class RateTests(unittest.TestCase):
    def test_atomic_packets_do_not_replace_complete_state_with_partial(self):
        s=RateState();packet(s);old=s.snapshot()
        s.receive('[ARM RATES BEGIN] v=1',2);s.receive('[ARM RATE] point=wait lift=nan reach=100 dm=100 grip=100 table=100',2);s.receive('[ARM RATES END]',2)
        self.assertEqual(s.snapshot(),old)
        s.receive('[ARM RATES BEGIN] v=99',3);s.receive('[ARM RATES END]',3);self.assertEqual(s.snapshot(),old)
    def test_commands_are_atomic_bounded_and_fit_existing_console(self):
        for p,_ in POINTS:
            cmd=rate_command(p,[300]*5);self.assertLessEqual(len(cmd),63)
            self.assertEqual(cmd.split()[:4],['arm','rates','set',p])
        for values in ([100]*4,[100,100,float('nan'),100,100],[0]*5,[301]*5):
            with self.assertRaises(ValueError):rate_command('wait',values)
    def test_uart_prompt_demux_and_complete_model_snapshot(self):
        s=ArmState();packet(s,pct=120);self.assertEqual(s.snapshot()['rates']['points']['grab']['reach'],120)
        snap=s.snapshot();snap['rates']['points']['grab']['reach']=1
        self.assertEqual(s.snapshot()['rates']['points']['grab']['reach'],120)
    def test_slow_motion_wait_budget_does_not_send_a_stop(self):
        sent=[];a=ArmActions(sent.append);s=ArmState();packet(s,pct=50)
        with patch('arm_control.time.perf_counter',return_value=1):a.start([('arm ring 1','DONE')],0)
        with patch('arm_control.time.perf_counter',return_value=200):a.update(s.snapshot())
        self.assertIsNotNone(a.active);self.assertEqual(sent,['arm ring 1'])
        with patch('arm_control.time.perf_counter',return_value=260):a.update(s.snapshot())
        self.assertIsNone(a.active);self.assertEqual(sent,['arm ring 1'])
    def test_tk_editor_build_and_explicit_set_then_read(self):
        import tkinter as tk
        from tkinter import ttk
        from arm_panel import ArmPanel
        from test_arm_control import fixture
        state=fixture();state.data['version']['firmware']='268';packet(state,time.perf_counter())
        class Event:
            def is_set(self):return False
        class Log:
            connected=True;session='rate-test';stop_event=Event()
            def __init__(self):self.sent=[]
            def send(self,cmd):self.sent.append(cmd)
            def arm_snapshot(self):return state.snapshot()
        log=Log();root=tk.Tk();root.withdraw();page=ttk.Frame(root);page.pack()
        try:
            panel=ArmPanel(page,log);root.update_idletasks()
            self.assertLessEqual(panel.rates_frame.winfo_reqwidth(),1120);self.assertLessEqual(panel.rates_frame.winfo_reqheight(),650)
            panel.last_poll=time.perf_counter();panel.poll();panel.rate_point.set(POINTS[1][1]);panel.select_rates()
            panel.rate_values['reach'].set(120);self.assertFalse(log.sent)
            panel.apply_rates();self.assertEqual(log.sent[-1],'arm rates set grab 100.0 120.0 100.0 100.0 100.0')
            state.receive('[ARM RATES ACK] op=set HAL=0; no motion',time.perf_counter());panel.poll()
            self.assertEqual(log.sent[-1],'arm rates status')
            self.assertTrue(panel.rate_dirty) # Readback, not sending, commits the editor.
            state.receive('[ARM RATES BEGIN] v=1 firmware=268 variant=PRELIM source=RAM sequence=9',time.perf_counter())
            for p,_ in POINTS:state.receive('[ARM RATE] point='+p+' lift=100 reach='+('120' if p=='grab' else '100')+' dm=100 grip=100 table=100',time.perf_counter())
            state.receive('[ARM RATES END]',time.perf_counter());panel.poll();self.assertFalse(panel.rate_dirty)
        finally:root.destroy()

if __name__=='__main__':unittest.main()
