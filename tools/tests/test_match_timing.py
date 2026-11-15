import sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
from match_timing import MatchTiming
from serial_monitor import SerialLog


class TimingTests(unittest.TestCase):
    def test_live_exact_stages_total_and_duplicate_end_summary(self):
        s=MatchTiming();s.receive('[MATCH] LEAVING QR zone=1 request=1',10)
        self.assertEqual(s.snapshot(11)['current_ms'],1000)
        leave='[MATCH STAGE] LEAVE from_start_ms=1500 stage_ms=1500 completed=0:01.500'
        s.receive(leave,11.51);s.receive('[MATCH] MOVING QR zone=1 request=2',11.52)
        s.receive('[MATCH STAGE] QR from_start_ms=4000 stage_ms=2500 completed=0:04.000',14)
        s.receive('[MATCH] MOVING RAW_BATCH1 zone=1 request=3',14.01)
        self.assertEqual(s.snapshot(15)['rows']['LEAVE']['source'],'主控')
        self.assertEqual(s.snapshot(15)['rows']['QR']['duration_ms'],2500)
        s.receive('[MATCH] FINISHED RAW_BATCH1 zone=1 request=0',20)
        s.receive('[MATCH TIME] total_ms=9900 elapsed=0:09.900 endpoint=TEST',20.1)
        self.assertEqual(s.snapshot(100)['elapsed_ms'],9900)
        s.receive(leave,20.11);self.assertEqual(len(s.rows),3)

    def test_fault_does_not_claim_completion_and_new_run_clears_rows(self):
        s=MatchTiming();s.receive('[MATCH] LEAVING QR zone=2 request=1',10)
        s.receive('[MATCH] FAULT QR zone=2 request=0',12)
        self.assertEqual(s.snapshot(90)['current_ms'],2000);self.assertFalse(s.rows)
        s.receive('[MATCH] LEAVING QR zone=1 request=5',100)
        self.assertEqual(s.snapshot(101)['elapsed_ms'],1000)

    def test_legacy_estimates_replaceable_and_midrun_unknown_is_explicit(self):
        s=MatchTiming();s.receive('[MATCH] MOVING ROUGH_BATCH1 zone=1 request=5',50)
        self.assertIsNone(s.snapshot(60)['elapsed_ms'])
        s.receive('[MATCH] FINISHED ROUGH_BATCH1 zone=1 request=0',60)
        self.assertFalse(s.rows)
        s.receive('[MATCH] LEAVING QR zone=1 request=10',100)
        s.receive('[MATCH] MOVING QR zone=1 request=11',102)
        self.assertEqual(s.rows['LEAVE']['source'],'PC估算')
        s.receive('[MATCH STAGE] LEAVE from_start_ms=1900 stage_ms=1900 completed=0:01.900',102.01)
        self.assertEqual(s.rows['LEAVE']['duration_ms'],1900)
        s.receive('[MATCH STAGE] QR from_start_ms=1 stage_ms=2 completed=bad',104)
        self.assertNotIn('QR',s.rows)

    def test_serial_cache_and_real_map_view_do_not_send_motion(self):
        import tkinter as tk
        from tkinter import ttk
        from field_map_panel import FieldMapPanel
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as folder:
                log=SerialLog(folder)
                for line in ('[MATCH] LEAVING QR zone=1 request=1',
                    '[MATCH STAGE] LEAVE from_start_ms=1500 stage_ms=1500 completed=0:01.500',
                    '[MATCH] MOVING QR zone=1 request=2',
                    '[MATCH STAGE] QR from_start_ms=4000 stage_ms=2500 completed=0:04.000',
                    '[MATCH] MOVING RAW_BATCH1 zone=1 request=3'):
                    log._record('RX',line)
                for _ in range(1100):log._record('RX','diagnostic')
                frame=ttk.Frame(root);frame.pack();panel=FieldMapPanel(frame,log)
                frame.after_cancel(panel._after);panel.poll()
                self.assertIn('2.50s',panel.timing_labels['QR'].get())
                self.assertIn('4.00s',panel.timing_labels['QR'].get())
                self.assertTrue(panel.canvas.find_withtag('stage_time'))
                snap=log.match_timing_snapshot();snap['rows'].clear()
                self.assertEqual(len(log.match_timing_snapshot()['rows']),2)
                self.assertTrue(log.outbound.empty())
        finally:root.destroy()


if __name__=='__main__':unittest.main()
