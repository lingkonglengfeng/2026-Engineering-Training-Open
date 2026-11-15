"""转向遥测解析测试：不需要串口、显示器或 Tk 窗口，只把行文本喂给 SerialLog._record。"""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
from serial_monitor import SerialLog,TURN_SAMPLE_LIMIT


class TurnTraceParsing(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.log=SerialLog(self.folder.name)

    def tearDown(self):
        self.folder.cleanup()

    def rx(self,text):
        return self.log._record('RX',text)

    def snap(self):
        return self.log.turn_snapshot()

    def test_wave_samples_tolerate_reordered_fields_negatives_and_extra_markers(self):
        self.assertFalse(self.snap()['supported'])
        self.rx('[TURNW] t=125 tgt=+90.00 act=+12.34 err=+77.66 out=+0.650')
        snapshot=self.snap()
        self.assertTrue(snapshot['supported'])
        self.assertEqual(len(snapshot['samples']),1)
        stamp,t_ms,tgt,act,err,out=snapshot['samples'][0]
        self.assertIsInstance(stamp,float)
        self.assertEqual(t_ms,125.0)
        self.assertAlmostEqual(tgt,90.0);self.assertAlmostEqual(act,12.34)
        self.assertAlmostEqual(err,77.66);self.assertAlmostEqual(out,.650)
        # 字段乱序、负号、行尾多余标记都要照常解析。
        self.rx('[TURNW] out=-0.650 TRAILING err=-77.66 act=-12.34 tgt=-90.00 t=250')
        second=self.snap()['samples'][1]
        self.assertEqual(second[1],250.0)
        self.assertAlmostEqual(second[2],-90.0);self.assertAlmostEqual(second[3],-12.34)
        self.assertAlmostEqual(second[4],-77.66);self.assertAlmostEqual(second[5],-.650)
        # 重复行按到达顺序保留，不崩也不去重。
        self.rx('[TURNW] t=250 tgt=-90.00 act=-12.34 err=-77.66 out=-0.650')
        self.assertEqual(len(self.snap()['samples']),3)
        # 缺字段/非数值的畸形行被忽略。
        self.rx('[TURNW] t=abc tgt=1 act=2 err=3 out=4')
        self.rx('[TURNW] t=300 tgt=nan act=2 err=3 out=4')
        self.assertEqual(len(self.snap()['samples']),3)

    def test_status_start_done_and_pid(self):
        self.rx('[TURN] status enabled=1 active=0 trace=1 tgt=+0.00 act=+12.34 err=+0.000 out=+0.000 kp=0.0450 ki=0.0005 kd=0.0035')
        status=self.snap()['status']
        self.assertEqual(status['enabled'],1.0)
        self.assertEqual(status['active'],0.0)
        self.assertEqual(status['trace'],1.0)
        self.assertAlmostEqual(status['act'],12.34)
        self.assertAlmostEqual(status['kp'],.045)
        self.assertIn('[TURN] status',status['raw'])
        self.assertTrue(self.snap()['supported'])
        self.rx('[PID] kp=0.0450 ki=0.0005 kd=0.0035 source=DEFAULT')
        pid=self.snap()['pid']
        self.assertAlmostEqual(pid['kp'],.045)
        self.assertAlmostEqual(pid['ki'],.0005)
        self.assertAlmostEqual(pid['kd'],.0035)
        self.assertEqual(pid['source'],'DEFAULT')
        # V236：来源随 &read/&save 变化；状态行也带 src。
        self.rx('[PID] kp=0.0500 ki=0.0004 kd=0.0030 source=W25Q64')
        self.assertEqual(self.snap()['pid']['source'],'W25Q64')
        self.assertAlmostEqual(self.snap()['pid']['kp'],.05)
        self.rx('[TURN] status enabled=1 active=0 imu=1 trace=0 rpm=180 tgt=+0.00 act=+0.00 '
                'err=+0.000 out=+0.000 kp=0.0500 ki=0.0004 kd=0.0030 src=W25Q64')
        status=self.snap()['status']
        self.assertEqual(status['src'],'W25Q64')
        self.assertEqual(status['rpm'],180.0)
        self.rx('[TURN] start tgt=+90.00 act=+0.32 profile=PRECISE')
        snapshot=self.snap();start=snapshot['start']
        self.assertTrue(snapshot['active'])
        self.assertAlmostEqual(start['tgt'],90.0)
        self.assertAlmostEqual(start['act'],.32)
        self.assertEqual(start['profile'],'PRECISE')
        self.assertEqual(snapshot['samples'],[])  # 新转向从空波形开始
        self.rx('[TURNW] t=25 tgt=+90.00 act=+0.50 err=+89.50 out=+0.700')
        self.assertEqual(len(self.snap()['samples']),1)
        self.rx('[TURN] done 1180ms target=+90.00 err=+0.008 overshoot=0.014 peak_o=1.00  FAST FB ST OVER-3s')
        snapshot=self.snap();done=snapshot['done']
        self.assertFalse(snapshot['active'])
        self.assertEqual(done['ms'],1180.0)
        self.assertAlmostEqual(done['target'],90.0)
        self.assertAlmostEqual(done['err'],.008)
        self.assertAlmostEqual(done['overshoot'],.014)
        self.assertAlmostEqual(done['peak_o'],1.0)
        self.assertEqual(done['flags'],['FAST','FB','ST','OVER-3s'])
        self.assertEqual(len(snapshot['samples']),1)  # done 不丢波形

    def test_trace_receipt_and_rejected_and_pid_note(self):
        self.rx('[TURN] trace on period=25ms')
        trace=self.snap()['trace']
        self.assertTrue(trace['on'])
        self.assertEqual(trace['period_ms'],25.0)
        self.rx('[TURN] trace off')
        trace=self.snap()['trace']
        self.assertFalse(trace['on'])
        self.assertIsNone(trace['period_ms'])
        self.rx('[TURN] rejected: need &EN and IMU zero-drift ready')
        reject=self.snap()['reject']
        self.assertEqual(reject['reason'],'need &EN and IMU zero-drift ready')
        self.rx('[PID] saved and verified')
        self.assertIn('saved',self.snap()['pid_note']['text'])
        # 只带部分字段的 done 也不能让解析抛异常。
        self.rx('[TURN] done 980ms target=-90.00')
        done=self.snap()['done']
        self.assertEqual(done['ms'],980.0)
        self.assertIsNone(done['err'])
        self.assertEqual(done['flags'],[])
        self.rx('[TURN] done')  # 缺毫秒数的畸形行被忽略
        self.assertEqual(self.snap()['done']['ms'],980.0)

    def test_clear_sample_limit_and_snapshot_copy(self):
        for index in range(TURN_SAMPLE_LIMIT+100):
            self.rx('[TURNW] t=%d tgt=0 act=0 err=0 out=0'%index)
        samples=self.snap()['samples']
        self.assertEqual(len(samples),TURN_SAMPLE_LIMIT)
        self.assertEqual(samples[0][1],100.0)       # 先进先出丢掉最早 100 点
        self.assertEqual(samples[-1][1],float(TURN_SAMPLE_LIMIT+99))
        self.log.turn_clear()
        self.assertEqual(self.snap()['samples'],[])
        self.rx('[TURNW] t=7 tgt=1 act=2 err=3 out=4')
        self.assertEqual(len(self.snap()['samples']),1)
        # 快照是副本：面板改快照不能影响解析状态。
        self.rx('[PID] kp=0.1 ki=0.2 kd=0.3')
        snapshot=self.snap()
        snapshot['pid']['kp']=999.0
        snapshot['samples'].append('mutated')
        fresh=self.snap()
        self.assertAlmostEqual(fresh['pid']['kp'],.1)
        self.assertEqual(len(fresh['samples']),1)

    def test_turn_snapshot_does_not_consume_serial_events(self):
        self.rx('[TURNW] t=1 tgt=2 act=3 err=4 out=5')
        self.rx('[TURN] start tgt=2.00 act=3.00 profile=PRECISE')
        records=self.log.drain()
        self.assertEqual(len(records),2)                 # drain 语义保持不变
        self.assertTrue(self.log.turn_snapshot()['supported'])
        self.assertEqual(len(self.log.turn_snapshot()['samples']),0)  # start 已清空


if __name__=='__main__':
    unittest.main(verbosity=2)
