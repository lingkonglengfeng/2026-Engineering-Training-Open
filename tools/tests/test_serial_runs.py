"""Per-match serial files, without opening a serial port or sending commands."""
from contextlib import contextmanager
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'vision/pc_vision'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from serial_monitor import SerialLog, timestamp
from extract_latest_match_run import extract, latest_slice

@contextmanager
def recorder(folder):
    log=SerialLog(folder);log.session='fake_COM10';log.port='COM10'
    log.path=log._idle_path=Path(folder)/'fake_idle.log'
    log._file=log.path.open('w',encoding='utf-8')
    try:yield log
    finally:
        log._record('STATUS','串口接收已结束')
        log._file.close();log._file=None

class MatchRunLogs(unittest.TestCase):
    def index(self,folder):return json.loads((Path(folder)/'runs/latest.json').read_text(encoding='utf-8'))
    def rx(self,log,text):return log._record('RX',text)

    def test_each_launch_has_one_file_including_final_timing(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            self.rx(log,'idle before')
            self.rx(log,'ZDT> [MATCH] LEAVING QR zone=1 request=3');first=log.path
            self.rx(log,'[MAT] first run')
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=3')
            self.assertEqual(first,log.path)  # Repeated status is not another launch.
            self.rx(log,'[MATCH] FINISHED RETURN_START zone=1 request=0')
            self.rx(log,'[MATCH TIME] total_ms=1000')
            self.rx(log,'[MATCH STAGE] LEAVE from_start_ms=100')
            self.rx(log,'[MATCH STAGE] RETURN_START from_start_ms=1000')
            self.assertIsNone(log._run_info)
            self.rx(log,'idle between')
            self.rx(log,'[MATCH] LEAVING QR zone=2 request=8');second=log.path
            self.rx(log,'[MAT] second run')
            self.rx(log,'[MATCH] FAULT RAW_BATCH1 zone=2 request=0')
            self.rx(log,'[MATCH] flow stopped. Re-place car, match reset, startpose N.')
            self.rx(log,'idle after');log._flush()
            self.assertNotEqual(first,second)
            self.assertEqual(len(list((Path(folder)/'runs').glob('*.log'))),2)
            a=first.read_text(encoding='utf-8');b=second.read_text(encoding='utf-8')
            self.assertIn('RETURN_START from_start_ms=1000',a)
            self.assertNotIn('second run',a);self.assertNotIn('idle',a)
            self.assertNotIn('first run',b);self.assertNotIn('idle',b)
            idle=log._idle_path.read_text(encoding='utf-8')
            self.assertIn('idle between',idle);self.assertNotIn('[MAT]',idle)
            self.assertEqual(self.index(folder)['outcome'],'FAULT')
            self.assertEqual(Path(self.index(folder)['path']),second)

    def test_first_round_trial_closes_after_temp1_statistics(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=2');path=log.path
            self.rx(log,'[MATCH] FINISHED TEMP_BATCH1 zone=1 request=0')
            self.rx(log,'[MATCH TIME] total_ms=800')
            self.rx(log,'[MATCH STAGE] TEMP_BATCH1 from_start_ms=800')
            self.assertEqual(self.index(folder)['outcome'],'FINISHED')
            self.assertIn('TEMP_BATCH1 from_start_ms=800',path.read_text(encoding='utf-8'))

    def test_legacy_finished_without_stage_lines_has_bounded_tail(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=2');path=log.path
            self.rx(log,'[MATCH] FINISHED RETURN_START zone=1 request=0')
            self.rx(log,'[MATCH TIME] total_ms=700')
            with patch('serial_monitor.time.perf_counter',return_value=log._run_finish_due+1):log._flush()
            self.assertIsNone(log._run_info)
            self.assertIn('total_ms=700',path.read_text(encoding='utf-8'))

    def test_disconnect_reset_and_restart_do_not_mix_runs(self):
        for ending,outcome in (('[MATCH] IDLE QR zone=0 request=0','RESET'),
                               ('[ARM] CAMERA_DEPLOY_V203: boot','CONTROLLER_RESTART')):
            with self.subTest(outcome=outcome),tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
                self.rx(log,'[MATCH] LEAVING QR zone=1 request=2');self.rx(log,ending)
                self.assertEqual(self.index(folder)['outcome'],outcome)
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=2')
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=3')
            self.assertEqual(len(list((Path(folder)/'runs').glob('*.log'))),2)
            log._record('STATUS','串口接收已结束')
            self.assertEqual(self.index(folder)['outcome'],'DISCONNECTED')

    def test_partial_state_is_not_a_boundary_and_snapshot_uses_run_path(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            log._record('RX','[MATCH] LEAVING QR zone=1 request=2',partial=True)
            self.assertIsNone(log._run_info)
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=2');path=log.path
            context=log.mark_snapshot('CAM1',1,timestamp()['perf_s'],'image.png',timestamp())
            self.assertEqual(context['log_path'],str(path))
            self.assertEqual(context['run_log_path'],str(path))
            self.assertIn('SNAPSHOT',path.read_text(encoding='utf-8'))

    def test_connection_ends_after_finish_preserves_completed_outcome(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            self.rx(log,'[MATCH] LEAVING QR zone=1 request=2')
            self.rx(log,'[MATCH] FINISHED RETURN_START zone=1 request=0')
            log._record('STATUS','串口接收已结束')
            self.assertEqual(self.index(folder)['outcome'],'FINISHED')

    def test_legacy_extraction_returns_only_the_last_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'old.log'
            prefix=b'old unrelated data\n'*100000
            old=b'[old] [pc=1] #1 RX [MATCH] LEAVING QR zone=1 request=2\nold run\n'
            run=(b'[new] [pc=2] #9 RX [MATCH] LEAVING QR zone=1 request=3\n'
                 b'[end] [pc=3] #10 RX [MATCH] FINISHED RETURN_START zone=1 request=0\n'
                 b'[end] [pc=3] #11 RX [MATCH TIME] total_ms=1000\n'
                 b'[end] [pc=3] #12 RX [MATCH STAGE] RETURN_START from_start_ms=1000\n')
            path.write_bytes(prefix+old+run+b'later idle\n')
            data,offset,size=latest_slice(path)
            self.assertEqual(offset,len(prefix+old));self.assertEqual(data,run+b'later idle\n')
            self.assertEqual(size,path.stat().st_size)
            info=extract(path)
            self.assertEqual(Path(info['path']).read_bytes(),run)
            self.assertEqual(info['outcome'],'FINISHED')
            self.assertEqual(self.index(folder)['path'],info['path'])

    def test_legacy_missing_start_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'empty.log';path.write_text('no start\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'No MATCH LEAVING'):latest_slice(path)

    def test_locked_index_does_not_interrupt_actual_run_logging(self):
        with tempfile.TemporaryDirectory() as folder,recorder(folder) as log:
            with patch.object(Path,'replace',side_effect=PermissionError('reader has index open')):
                self.rx(log,'[MATCH] LEAVING QR zone=1 request=2')
                self.rx(log,'[MAT] current run continues')
            self.assertIsNotNone(log.run_index_error)
            log._flush()
            self.assertIsNone(log.run_index_error)
            self.assertEqual(self.index(folder)['outcome'],'RUNNING')
            self.assertIn('current run continues',log.path.read_text(encoding='utf-8'))

if __name__=='__main__':unittest.main()
