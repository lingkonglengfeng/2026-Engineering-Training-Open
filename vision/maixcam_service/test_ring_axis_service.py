"""可选0A三帧观测，旧格式兼容、失效、短写及开关会话隔离。"""
import math
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import time
import service
from test_service import Clock,Serial,Backend,frames
from test_service import NativeImage


class AxisServiceTests(unittest.TestCase):
    def setup(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        base=self.backend.observe
        self.backend.observe=lambda now:dict(base(now),ring_axis=dict(degrees=-1.25,confidence=88,source=3))
        self.server=service.VisionService(self.serial,self.backend,self.clock)
    def configure(self,flags=1,seq=7):
        self.serial.rx.extend(service.encode_set(3,2,2,20,seq,flags))
        self.server.tick()
    def observations(self):
        return [f for f in frames(self.serial.output)if f[1]in (3,0x0a,0x83)]
    def test_legacy_off_remains_two_frames(self):
        self.setup();self.configure(flags=0)
        self.assertEqual([f[1]for f in self.observations()],[3,0x83])
    def test_on_atomic_three_same_sequence_id_and_station_binding(self):
        self.setup();self.configure()
        pos,axis,count=self.observations()
        self.assertEqual([pos[1],axis[1],count[1]],[3,0x0a,0x83])
        self.assertEqual(pos[2],axis[2]);self.assertEqual(pos[2],count[2])
        self.assertEqual(struct.unpack('<hBB',axis[3:7]),(-125,88,3))
        self.assertEqual(count[3:7],bytes((3,2,7,0)))
        self.assertTrue(axis[2]&0x80)
    def test_axis_is_independently_invalid_without_damaging_xy(self):
        self.setup()
        base=self.backend.observe
        self.backend.observe=lambda now:dict(base(now),ring_axis=None)
        self.configure()
        pos,axis,count=self.observations()
        self.assertTrue(pos[2]&0x80);self.assertFalse(axis[2]&0x80)
        self.assertEqual(axis[3:7],bytes(4));self.assertTrue(count[2]&0x80)
    def test_count_or_position_invalid_never_leaks_axis(self):
        for change in ({'ring_count':2},{'ring_count_trusted':False},{'coordinates':None}):
            self.setup();base=self.backend.observe
            self.backend.observe=lambda now:dict(base(now),**change)
            self.configure();axis=self.observations()[1]
            self.assertFalse(axis[2]&0x80);self.assertEqual(axis[3:7],bytes(4))
    def test_more_than_500ms_processing_invalidates_axis_and_count(self):
        self.setup()
        self.backend.on_read=lambda:setattr(self.clock,'value',self.clock.value+.6)
        self.configure()
        pos,axis,count=self.observations()
        self.assertFalse(any(f[2]&0x80 for f in (pos,axis,count)))
    def test_invalid_values_and_wrap_quantization(self):
        for change in ({'degrees':float('nan')},{'degrees':90},{'degrees':-91},{'confidence':101},{'confidence':float('nan')},{'source':0},{'source':3.0}):
            measurement=dict(degrees=0,confidence=90,source=3);measurement.update(change)
            frame=service.encode_ring_axis(measurement,2)
            self.assertFalse(frame[2]&0x80);self.assertEqual(frame[3:7],bytes(4))
        frame=service.encode_ring_axis(dict(degrees=89.999,confidence=90,source=3),2)
        self.assertEqual(struct.unpack('<h',frame[3:5])[0],-9000)
    def test_profile_only_accepted_in_ring_station_2_or_3(self):
        for mode,station,flags,expected in [(3,2,1,0),(3,3,1,0),(3,1,1,6),(3,2,2,0),(3,3,3,0),(3,2,4,6),(8,1,1,6),(0,0,1,6)]:
            target=2 if mode==3 else 0
            self.assertEqual(service.validate_set(service.encode_set(mode,station,target,20 if mode else 0,0,flags)),expected)
        self.assertEqual(service.validate_set(service.encode_set(3,2,0,20,0,1)),6)
    def test_short_write_does_not_interleave_heartbeat(self):
        self.setup();self.configure();self.serial.output.clear()
        self.serial.writes=[8,0,8,0,8]
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.server.writer.control(service.make_frame(0x82,0,0,True,bytes((1,3,2,0))),'heartbeat')
        for now in (.11,.12,.13,.14):self.clock.value=now;self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual([f[1]for f in sent[:4]],[3,0x0a,0x83,0x82])

    def test_stop_and_flag_off_discard_old_tail_before_ack(self):
        for mode,flags in ((0,0),(3,0)):
            for cut in (8,16):
                self.setup();self.configure();self.serial.output.clear();self.serial.writes=[cut]
                self.clock.value=.1;self.server.tick()
                self.clock.value=.11
                self.serial.rx.extend(service.encode_set(mode,2 if mode else 0,2 if mode else 0,20 if mode else 0,8,flags))
                self.server.tick();self.clock.value=.17;self.server.tick()
                tail=frames(self.serial.output[cut:])
                self.assertEqual(tail[0][1],service.ACK)
                self.assertFalse(any(f[1]==0x0a for f in tail))
    def test_single_byte_writes_keep_complete_triple_before_heartbeat(self):
        self.setup();self.configure();self.serial.output.clear();self.serial.writes=[1]*40
        self.clock.value=.1;self.server.tick()
        self.server.writer.control(service.make_frame(0x82,0,0,True,bytes((1,3,2,0))),'heartbeat')
        for i in range(1,28):
            self.clock.value=.1+i*.001;self.server.tick()
        self.assertEqual([f[1]for f in frames(self.serial.output)],[3,0x0a,0x83,0x82])
    def test_mode_request_during_detection_discards_entire_old_triple(self):
        self.setup();self.configure();self.serial.output.clear();self.clock.value=.1
        self.backend.on_read=lambda:self.serial.rx.extend(service.encode_set(3,3,2,20,8,0))
        self.server.tick()
        self.assertEqual(self.observations(),[])
    def test_24byte_writer_has_same_50ms_deadline_and_recovery(self):
        self.setup();self.configure();self.serial.output.clear();self.serial.writes=[8,0,0]
        self.clock.value=.1;self.server.tick()
        self.clock.value=.151;self.server.tick()
        self.assertGreaterEqual(self.server.writer.resume_at,.206)
        self.assertEqual(len(self.serial.output),8)


class AxisBackendTests(unittest.TestCase):
    def backend(self,gray,flags):
        import numpy as np
        scene=[np.repeat(gray[:,:,None],3,axis=2)]
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_string(self,*args,**kwargs):pass
        class Camera:
            def __init__(self,*args,**kwargs):pass
            def read(self,**kwargs):return Frame(scene[0])
            def close(self):pass
            def clear_buff(self):pass
        image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),Fit=SimpleNamespace(FIT_CONTAIN='contain'),
            COLOR_WHITE=0,Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)),
            image2cv=lambda frame,**kwargs:frame.array,cv2image=lambda array,**kwargs:Frame(array.copy()))
        display=SimpleNamespace(width=lambda:320,height=lambda:240,show=lambda *a,**k:0)
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,display)
        backend.configure((3,2,2,20,flags))
        return backend,scene
    def test_true_cv_backend_confirms_then_reports_signed_axis_and_drops_on_loss(self):
        import cv2,numpy as np
        from test_middle_ring import make_pattern
        angle=math.radians(3)
        centers=tuple((round(240+k*145*math.cos(angle)),round(180-k*145*math.sin(angle)))for k in (-1,0,1))
        gray=make_pattern(size=(480,360),centers=centers,radius=42)
        backend,scene=self.backend(gray,1)
        try:
            obs=backend.observe(time.monotonic());self.assertIsNone(obs['ring_axis'])
            obs=backend.observe(time.monotonic());self.assertIsNone(obs['ring_axis'])
            obs=backend.observe(time.monotonic())
            self.assertIsNotNone(obs['coordinates'])
            self.assertAlmostEqual(obs['ring_axis']['degrees'],3,delta=.25)
            backend.preview(backend.config,obs)
            scene[0]=scene[0].copy();scene[0][:,330:]=220
            obs=backend.observe(time.monotonic())
            self.assertEqual(obs['ring_count'],2);self.assertIsNone(obs['ring_axis'])
        finally:backend.close()
    def test_off_does_not_compute_or_attach_axis(self):
        from test_middle_ring import make_pattern
        backend,_=self.backend(make_pattern(size=(480,360),centers=((80,180),(240,180),(400,180)),radius=45),0)
        try:
            with patch.object(backend.module,'ring_axis_measurement',side_effect=AssertionError('axis disabled')):
                for _ in range(3):obs=backend.observe(time.monotonic())
            self.assertNotIn('ring_axis',obs)
        finally:backend.close()
    def test_user_boundary_two_layer_picture_keeps_xy_but_axis_invalid(self):
        import cv2,numpy as np
        path=Path(__file__).resolve().parents[1]/'maixcam_ring/test_data/occupied_5_cleaned.png'
        gray=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
        backend,_=self.backend(gray,3)
        try:
            for _ in range(4):obs=backend.observe(time.monotonic())
            self.assertEqual(obs['ring_count'],3)
            self.assertIsNotNone(obs['coordinates'])
            self.assertIsNone(obs['ring_axis'])
        finally:backend.close()


if __name__=='__main__':unittest.main()
