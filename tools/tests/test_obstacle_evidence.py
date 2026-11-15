"""Automatic evidence with saved/fake frames and fake UART only."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'vision/pc_vision'))
from obstacle_evidence import ObstacleEvidence
from obstacle_link import parse_state
from serial_monitor import SerialLog
from test_pc_obstacles import FakeUart, state, snapshot

def frame(value=70,number=4):
    data=snapshot(time.perf_counter(),number)
    data.update(raw_image=np.full((120,320,3),value,np.uint8),image=np.full((120,320,3),value+10,np.uint8),frame=number)
    return data

def ack(seq=1,site='P02',result='DEFERRED',obj=1):
    return '[VOBS ACK] session=21 seq=%d result=%s obstacle=%d site=%s merged=0 raw=683,2173 plan=750,2125 car=1100,2000 yaw=90 frame_car=1080,2000 frame_yaw=90 age=50 queue=10 cap=1.0'%(seq,result,obj,site)

class EvidenceTests(unittest.TestCase):
    def test_evicted_slot_reuse_saves_same_site_again(self):
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder);obj.offer(frame())
            try:
                self.receive(obj,ack());self.wait(lambda:obj.status()['saved']==1)
                self.receive(obj,'[VOBS EVICT] session=21 obstacle=1 old_site=P02 new_site=P06 removed=20')
                self.assertNotIn((21,1),obj.confirmed)
                self.receive(obj,ack(2));self.wait(lambda:obj.status()['saved']==2)
            finally:obj.close()
    def wait(self,predicate):
        end=time.perf_counter()+3
        while not predicate() and time.perf_counter()<end:time.sleep(.005)
        self.assertTrue(predicate())
    def receive(self,obj,line):
        obj.receive(line,{'wall_time':'2026-11-03T14:00:00+08:00','perf_s':1.}, {'run_log_path':'run001.log','session':'session_test'})
    def test_exact_frame_raw_overlay_coordinates_and_per_run_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder)
            try:
                first=frame();obj.offer(first);obj.sent('vobs 21 1 500 0 50',(0,4,4000));obj.offer(frame(100,5))
                self.receive(obj,ack());self.wait(lambda:obj.status()['saved']==1)
                paths=list((Path(folder)/'run001').glob('*.json'));self.assertEqual(len(paths),1)
                meta=json.loads(paths[0].read_text(encoding='utf-8'))
                self.assertTrue(meta['exact_observation_frame']);self.assertEqual(meta['image']['frame'],4)
                self.assertEqual(meta['car_current_mm'],[1100,2000]);self.assertEqual(meta['obstacle_plan_mm'],[750,2125])
                self.assertEqual(meta['obstacle_estimate_mm'],[683,2173]);self.assertEqual(meta['car_at_frame_mm'],[1080,2000])
                raw=cv2.imdecode(np.fromfile(str(paths[0].with_name(paths[0].stem+'_raw.png')),np.uint8),1)
                overlay=cv2.imdecode(np.fromfile(str(paths[0].with_name(paths[0].stem+'_overlay.png')),np.uint8),1)
                self.assertTrue(np.array_equal(raw,first['raw_image']));self.assertFalse(np.array_equal(raw,overlay))
                self.assertTrue(np.all(first['image']==80))
            finally:obj.close()
    def test_dedup_site_change_and_map_with_confirmation_context(self):
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder);obj.offer(frame())
            try:
                self.receive(obj,ack());self.receive(obj,ack(2));self.receive(obj,ack(3,site='P03'))
                self.receive(obj,'[VOBS MAP] session=21 obstacle=1 site=P03 x=1750 y=2125 car=1200,2100 yaw=90')
                self.wait(lambda:obj.status()['saved']==3)
                paths=list(Path(folder).rglob('*map*.json'));meta=json.loads(paths[0].read_text(encoding='utf-8'))
                self.assertFalse(meta['exact_observation_frame']);self.assertEqual(meta['obstacle_plan_mm'],[1750,2125])
                self.assertEqual(meta['obstacle_estimate_mm'],[683,2173]);self.assertEqual(meta['controller']['site'],'P03')
            finally:obj.close()
    def test_wireless_ack_uses_current_frame_explicitly(self):
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder);obj.offer(frame())
            try:
                self.receive(obj,ack(result='KEEP_NEARER',obj=2));self.wait(lambda:obj.status()['saved']==1)
                meta=json.loads(Path(obj.status()['last_path']).read_text(encoding='utf-8'))
                self.assertFalse(meta['exact_observation_frame']);self.assertEqual(meta['image_kind'],'current_camera_view')
            finally:obj.close()
    def test_disabled_missing_preview_and_bad_packets_never_save(self):
        obj=ObstacleEvidence();obj.offer(frame());self.receive(obj,ack());self.assertFalse(obj.status()['enabled'])
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder);obj.offer(frame());obj.offer(None)
            self.receive(obj,ack());self.receive(obj,'[VOBS ACK] session=no seq=? result=ACCEPTED')
            self.assertEqual(obj.status()['saved'],0);self.assertIn('CAM2',obj.status()['error']);obj.close()
    def test_slow_disk_never_blocks_receive_and_limit_evidence_is_bounded(self):
        import threading
        with tempfile.TemporaryDirectory() as folder:
            obj=ObstacleEvidence(folder);obj.offer(frame());gate=threading.Event();started=threading.Event()
            original=obj._save
            def slow(*args):started.set();gate.wait(2);return original(*args)
            obj._save=slow
            try:
                self.receive(obj,ack());self.assertTrue(started.wait(1))
                start=time.perf_counter();self.receive(obj,ack(2,result='LIMIT2',obj=0));self.assertLess(time.perf_counter()-start,.1)
                self.receive(obj,ack(3,result='LIMIT2',obj=0));gate.set();self.wait(lambda:obj.status()['saved']==2)
            finally:gate.set();obj.close()
    def test_disk_error_is_reported_without_control_io(self):
        with tempfile.TemporaryDirectory() as folder:
            notes=[];obj=ObstacleEvidence(folder,notes.append);obj.offer(frame())
            try:
                with patch.object(obj,'_save',side_effect=OSError('disk test')):
                    self.receive(obj,ack());self.wait(lambda:obj.status()['error']=='disk test')
                self.assertEqual(obj.status()['saved'],0);self.assertTrue(notes)
            finally:obj.close()
    def test_real_serial_record_dispatch_and_automatic_TX_association(self):
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            obj=logger.enable_obstacle_evidence(Path(folder)/'screens');logger.start('FAKE_ONLY',115200)
            try:
                self.wait(lambda:logger.connected);device.publish(21);self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                data=frame();obj.offer(data);logger.update_obstacle(data)
                self.wait(lambda:bool(device.writes));command=device.writes[-1][1];seq=int(command.split()[2])
                device.incoming.put((ack(seq)+'\r\n').encode());self.wait(lambda:obj.status()['saved']==1)
                meta=json.loads(Path(obj.status()['last_path']).read_text(encoding='utf-8'))
                self.assertTrue(meta['exact_observation_frame']);self.assertEqual(meta['tx_command'],command)
                self.assertFalse(any(text in ('&STOP','vobs retry','vobs on') for _,text in device.writes))
            finally:logger.stop();logger.thread.join(2);obj.close()
    def test_state_count_is_optional_and_backwards_compatible(self):
        self.assertNotIn('obstacles',parse_state(state(),1))
        self.assertEqual(parse_state(state()+' obstacles=2 phase=1',1)['obstacles'],2)
        self.assertNotIn('obstacles',parse_state(state()+' obstacles=3',1))

if __name__=='__main__':unittest.main()
