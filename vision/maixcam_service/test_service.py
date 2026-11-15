"""真实8字节控制流与假设备验证；不代替MaixCAM实际切换耗时。"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace, ModuleType
import unittest
import time
import weakref
from unittest.mock import patch

import service
service.DIAGNOSTIC_LOG_PATH = str(Path(__file__).resolve().parents[2]/'tmp/vision_service_simulated_errors.log')


class Clock:
    def __init__(self): self.value=0.0
    def __call__(self): return self.value


class Serial:
    def __init__(self): self.rx=bytearray();self.output=bytearray();self.writes=[]
    def read(self,n,timeout=0):
        assert timeout==0
        result=bytes(self.rx[:n]);del self.rx[:n];return result
    def write(self,packet):
        amount=self.writes.pop(0) if self.writes else len(packet)
        if isinstance(amount,Exception): raise amount
        if amount>0:self.output.extend(packet[:amount])
        return amount


class Backend:
    def __init__(self): self.configs=[];self.reads=0;self.previews=0;self.config=(0,0,0,0,0);self.on_read=None;self.fail=None
    def configure(self,config):
        if self.fail==config[0]:raise RuntimeError('camera open failure')
        self.config=config;self.configs.append(config)
    def observe(self,now):
        self.reads+=1
        if self.on_read:self.on_read()
        return {'coordinates':(60,-40),'id':2 if self.config[0]==3 else self.config[2], 'scale':2,
                'ring_count':3,'ring_count_trusted':True,'plate_scene':3,'plate_scene_trusted':True}
    def preview(self,*args):self.previews+=1


def frames(stream):
    result=service.FrameParser().feed(stream,0)
    for frame in result: assert service.crc8(frame[:7])==frame[7]
    return result


class NativeImage:
    def format(self):
        return 'rgb' if self.array.ndim==3 and self.array.shape[2]==3 else 'gray'
    def resize(self,width,height):
        import cv2
        return type(self)(cv2.resize(self.array,(width,height)))
    def to_format(self,fmt):
        import numpy as np
        if fmt=='rgb' and self.array.ndim==2:
            return type(self)(np.repeat(self.array[:,:,None],3,axis=2))
        raise AssertionError('unexpected conversion')


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        self.server=service.VisionService(self.serial,self.backend,self.clock)
    def command(self,mode,target=0,station=1,hz=20,sequence=0,flags=0):
        self.serial.rx.extend(service.encode_set(mode,station,target,hz,sequence,flags))
        self.server.tick()
    def test_default_stop_heartbeat_and_no_camera(self):
        self.server.tick();self.assertEqual(self.backend.reads,0)
        hb=frames(self.serial.output)[0]
        self.assertEqual(hb,service.make_frame(service.HEARTBEAT,0,0,True,bytes((1,0,0,0))))
    def test_set_ack_before_new_position_and_stop(self):
        self.command(2,target=3,sequence=2)
        sent=frames(self.serial.output)
        self.assertEqual([p[1] for p in sent],[service.ACK,service.HEARTBEAT,service.MAT_PX])
        self.assertEqual(sent[0][2],0x93)
        self.assertEqual(sent[-1][3:7],bytes.fromhex('2c 01 c8 00'))
        self.clock.value=.1;self.command(0,hz=0,sequence=3)
        reads=self.backend.reads
        self.clock.value=.6;self.server.tick()
        self.assertEqual(reads,self.backend.reads)
        self.assertEqual(frames(self.serial.output)[-1][3:7],bytes((1,0,1,0)))
    def test_plate_id_zero_is_valid(self):
        self.command(8)
        pixel=[p for p in frames(self.serial.output) if p[1]==service.PLATE_PX][-1]
        self.assertEqual(pixel[1],service.PLATE_PX)
        self.assertTrue(pixel[2]&0x80)

    def test_all_modes_reset_old_data_and_select_ring_two(self):
        for i,mode in enumerate((2,8,3,0,2)):
            self.clock.value=i*.2
            self.command(mode,target=3 if mode==2 else 0,hz=0 if mode==0 else 20,sequence=i)
        self.assertEqual([c[0] for c in self.backend.configs],[2,8,3,0,2])
        self.assertIn(3,[p[1] for p in frames(self.serial.output)])
    def test_identical_retry_replies_without_reconfiguring(self):
        self.command(3,sequence=4)
        self.clock.value=.5;self.command(3,sequence=4)
        self.assertEqual(len(self.backend.configs),1)
        acks=[p for p in frames(self.serial.output) if p[1]==service.ACK]
        self.assertEqual(len(acks),2);self.assertEqual(acks[0],acks[1])
    def test_bad_requests_preserve_config_and_report_errors(self):
        self.command(2,target=1)
        for i,(mode,target,station,hz,flags,expected) in enumerate([
                (7,0,1,20,0,1),(3,1,1,20,0,2),(4,1,1,20,0,3),
                (2,1,7,20,0,5),(2,1,1,51,0,6),(2,1,1,20,1,6)]):
            self.clock.value=.1+i*.1
            self.command(mode,target,station,hz,i+1,flags)
            ack=[p for p in frames(self.serial.output) if p[1]==service.ACK][-1]
            self.assertEqual(ack[4],expected);self.assertFalse(ack[2]&0x80)
            self.assertEqual(self.server.config[:3],(2,1,1))
        self.assertEqual(len(self.backend.configs),1)
    def test_camera_failure_ack_and_restore_previous_config(self):
        self.command(2,target=1)
        self.backend.fail=3;self.clock.value=.1;self.command(3,sequence=1)
        ack=[p for p in frames(self.serial.output) if p[1]==service.ACK][-1]
        self.assertEqual(ack[4],7);self.assertEqual(ack[5:7],bytes((2,1)))
        self.assertEqual(self.backend.config[0],2)
    def test_set_during_detection_discards_old_result(self):
        self.command(2,target=3)
        self.serial.output.clear();self.clock.value=.1
        self.backend.on_read=lambda:self.serial.rx.extend(service.encode_set(3,2,2,20,1))
        self.server.tick()
        self.assertEqual(frames(self.serial.output),[])
        self.backend.on_read=None;self.clock.value=.2;self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual([p[1] for p in sent],[service.ACK,service.RING_PX,service.RING_COUNT])

    def test_stop_during_detection_discards_result_and_never_sends_more_pixels(self):
        self.command(3,target=2)
        self.serial.output.clear();self.clock.value=.1
        self.backend.on_read=lambda:self.serial.rx.extend(service.encode_set(0,0,0,0,1))
        self.server.tick()
        self.assertEqual(frames(self.serial.output),[])
        self.backend.on_read=None
        self.clock.value=.2;self.server.tick()
        self.assertEqual([p[1] for p in frames(self.serial.output)],[service.ACK])
        self.assertEqual(frames(self.serial.output)[0][5:7],bytes((0,0)))
        reads=self.backend.reads
        self.clock.value=.7;self.server.tick()
        self.assertEqual(self.backend.reads,reads)
        self.assertEqual([p[1] for p in frames(self.serial.output)],[service.ACK,service.HEARTBEAT])
    def test_partial_ack_blocks_observations_until_complete(self):
        self.serial.writes=[3,0]
        self.command(2,target=1)
        self.assertEqual(self.backend.reads,0)
        self.clock.value=.01;self.server.tick()
        self.assertEqual(frames(self.serial.output)[0][1],service.ACK)
    def test_switch_abandons_partial_old_pixel_without_crlf(self):
        self.command(2,target=1)
        self.serial.output.clear();self.serial.writes=[3];self.clock.value=.1;self.server.tick()
        self.assertEqual(len(self.serial.output),3)
        self.clock.value=.11;self.command(3,sequence=1)
        self.assertEqual(len(self.serial.output),3)
        self.clock.value=.17;self.server.tick()
        tail=frames(self.serial.output[3:])
        self.assertEqual([p[1] for p in tail],[service.ACK,service.RING_PX,service.RING_COUNT])
    def test_parser_split_crc_noise_markers_and_timeout(self):
        parser=service.FrameParser();packet=service.encode_set(2,1,3,20,13)
        self.assertEqual(packet[2],0x6B)
        self.assertEqual(parser.feed(b'noise'+packet[:3],0),[])
        self.assertEqual(parser.feed(packet[3:],.01),[packet])
        broken=bytearray(packet);broken[-1]^=1
        self.assertEqual(parser.feed(broken+packet,.02),[packet])
        parser.feed(packet[:4],.03);self.assertEqual(parser.feed(packet,.09),[packet])
        self.assertGreater(parser.errors,0)
    def test_rate_limit_and_heartbeat_independent(self):
        self.command(2,target=1,hz=5)
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len([p for p in frames(self.serial.output) if p[1]==2]),1)
        self.clock.value=.2;self.server.tick()
        self.assertEqual(len([p for p in frames(self.serial.output) if p[1]==2]),2)
        self.clock.value=.6;self.server.tick()
        self.assertEqual(len([p for p in frames(self.serial.output) if p[1]==service.HEARTBEAT]),2)
    def test_control_queue_bounded(self):
        self.serial.writes=[0]*20
        for i in range(20):
            self.server.request(service.encode_set(2,1,1,20,i%16),0)
        self.assertLessEqual(len(self.server.writer.controls),8)

    def test_observation_exception_sends_invalid_and_resets_backend(self):
        self.command(3,target=2)
        resets=[]
        self.backend.invalidate=lambda:resets.append(True)
        self.backend.observe=lambda now:(_ for _ in ()).throw(RuntimeError('sensor frame'))
        self.clock.value=.1;self.server.tick()
        record=[p for p in frames(self.serial.output) if p[1]==service.RING_PX][-1]
        self.assertEqual(record[1],service.RING_PX)
        self.assertFalse(record[2]&0x80)
        self.assertEqual(record[3:7],bytes(4))
        self.assertEqual(resets,[True])

    def test_preview_failure_after_ack_keeps_positions_and_heartbeat_running(self):
        fallback=[]
        self.backend.preview=lambda *args:(_ for _ in ()).throw(RuntimeError('cv2image SDK failure'))
        self.backend.preview_failure=lambda:fallback.append(True)
        self.command(3,target=2)
        self.clock.value=.6;self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual(sent[0][1],service.ACK)
        self.assertEqual(len([p for p in sent if p[1]==service.RING_PX and p[2]&0x80]),2)
        self.assertEqual(len([p for p in sent if p[1]==service.HEARTBEAT]),2)
        self.assertEqual(self.server.preview_errors,2)
        self.assertEqual(fallback,[True,True])

    def test_preview_error_fallback_also_failing_does_not_exit_service(self):
        self.backend.preview=lambda *args:(_ for _ in ()).throw(RuntimeError('display failure'))
        self.backend.preview_failure=lambda:(_ for _ in ()).throw(RuntimeError('fallback display failure'))
        self.command(8)
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len([p for p in frames(self.serial.output) if p[1]==service.PLATE_PX]),2)


class BackendTests(unittest.TestCase):
    def test_light_on_initializes_output_high_then_turns_off_on_exit(self):
        events=[]
        def create(pin,mode,pull):
            events.append(('create',pin,mode,pull))
            return SimpleNamespace(value=lambda value:events.append(('value',value)))
        gpio=SimpleNamespace(GPIO=create,Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_DOWN=2,PULL_UP=1))
        pinmap=SimpleNamespace(set_pin_function=lambda pin,func:events.append(('map',pin,func)))
        err=SimpleNamespace(check_raise=lambda *args:None)
        with patch.object(service,'LIGHT_PWM_ENABLED',False):
            light=service.init_light_on(gpio,pinmap,err)
        service.light_off_on_exit(light)
        self.assertEqual(events,[('map','A19','GPIOA19'),('create','GPIOA19',2,1),('value',1),('value',0)])

    def test_light_setup_precedes_detector_preloading(self):
        import ast
        source=ast.parse(Path(service.__file__).read_text(encoding='utf-8'))
        device=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='run_device')
        setup=next(n for n in ast.walk(device) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='init_light_on')
        preload=next(n for n in ast.walk(device) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='detector_module')
        self.assertLess(setup.lineno,preload.lineno)

    def test_device_entry_opens_ring_then_stops_and_releases_devices(self):
        import cv2,numpy as np
        root=Path(__file__).resolve().parents[1]
        picture=cv2.imdecode(np.fromfile(str(root/'maixcam_ring/test_data/live_complete.png'),np.uint8),cv2.IMREAD_GRAYSCALE)
        picture=cv2.resize(picture,(480,360))
        cameras=[];closed=[];iterations=[0]
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_string(self,*args,**kwargs):pass
        class Camera:
            def __init__(self,*args,**kwargs):self.reads=0;cameras.append(self)
            def read(self,**kwargs):
                assert kwargs=={'block':True,'block_ms':service.CAMERA_READ_TIMEOUT_MS}
                self.reads+=1;return Frame(picture)
            def close(self):closed.append('camera')
        serial=Serial();serial.closed=False;serial.stop_sent=False
        initial_read=serial.read
        def read(n,timeout=0):
            if cameras and cameras[0].reads>=4 and not serial.stop_sent:
                serial.rx.extend(service.encode_set(0,0,0,0,1));serial.stop_sent=True
            return initial_read(n,timeout)
        serial.read=read;serial.available=lambda **kwargs:0
        serial.close=lambda:closed.append('serial')
        serial.rx.extend(service.encode_set(3,2,2,20,0))
        disp=SimpleNamespace(width=lambda:552,height=lambda:368,show=lambda *a,**k:None,
                             close=lambda:closed.append('display'))
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray',FMT_RGB888='rgb'),
                Fit=SimpleNamespace(FIT_CONTAIN='contain'),COLOR_WHITE=0,
                Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)),
                image2cv=lambda frame,**kwargs:frame.array,cv2image=lambda rgb,**kwargs:Frame(rgb))
        def need_exit():iterations[0]+=1;return iterations[0]>8
        maix=ModuleType('maix');maix.app=SimpleNamespace(need_exit=need_exit)
        maix.camera=SimpleNamespace(Camera=Camera);maix.image=image
        maix.display=SimpleNamespace(Display=lambda:disp,set_trans_image_quality=lambda value:None)
        maix.err=SimpleNamespace(check_raise=lambda *args:None)
        uart=SimpleNamespace(UART=lambda *a,**k:serial,BITS=SimpleNamespace(BITS_8=8),
                PARITY=SimpleNamespace(PARITY_NONE=0),STOP=SimpleNamespace(STOP_1=1))
        peripheral=ModuleType('maix.peripheral');peripheral.uart=uart
        light_events=[]
        def pin_function(*args):light_events.append(('pinmap',args));return 0
        peripheral.pinmap=SimpleNamespace(set_pin_function=pin_function)
        peripheral.gpio=SimpleNamespace(Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_DOWN=2,PULL_UP=1),
            GPIO=lambda *args:SimpleNamespace(value=lambda level:light_events.append(('value',level))))
        class PWM:
            def __init__(self,channel,**kwargs):light_events.append(('pwm',channel,kwargs))
            def enable(self):light_events.append(('pwm_enable',));return 0
            def duty(self,level):light_events.append(('pwm_duty',level));return level
            def disable(self):light_events.append(('pwm_disable',));return 0
        peripheral.pwm=SimpleNamespace(PWM=PWM)
        with patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}):
            service.run_device()
        self.assertEqual(cameras[0].reads,4)
        self.assertCountEqual(closed,['camera','serial','display'])
        sent=frames(serial.output)
        self.assertTrue(any(p[1]==3 and p[2]&0x80 for p in sent))
        self.assertTrue(any(p[1]==service.ACK and p[5]==0 for p in sent))
        self.assertEqual(light_events[0],('pinmap',('A19','GPIOA19')))
        self.assertEqual([event[1] for event in light_events if event[0]=='value'],[0,0])
        self.assertIn(('pwm',7,{'freq':20000,'duty':50.0,'enable':False}),light_events)
        self.assertIn(('pwm_disable',),light_events)

    def test_actual_ring_detector_works_through_service_backend(self):
        import cv2,numpy as np
        root=Path(__file__).resolve().parents[1]
        ring_gray=cv2.imdecode(np.fromfile(str(root/'maixcam_ring/test_data/live_complete.png'),np.uint8),cv2.IMREAD_GRAYSCALE)
        ring_gray=cv2.resize(ring_gray,(480,360))
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_ellipse(self,*args,**kwargs):pass
            def draw_line(self,*args,**kwargs):pass
            def draw_string(self,*args,**kwargs):pass
        class Camera:
            def __init__(self,w,h,fmt,**kwargs):self.fmt=fmt;self.closed=False
            def read(self,**kwargs):return Frame(ring_gray)
            def close(self):self.closed=True
        shown=[]
        display=SimpleNamespace(width=lambda:552,height=lambda:368,show=lambda *args,**kwargs:shown.append(args[0]))
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray',FMT_RGB888='rgb'),
                Fit=SimpleNamespace(FIT_CONTAIN='contain'),COLOR_WHITE=0,COLOR_YELLOW=1,COLOR_GREEN=2,COLOR_RED=3,
                Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)),
                image2cv=lambda frame,**kwargs:frame.array,cv2image=lambda rgb,**kwargs:Frame(rgb))
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,display)
        backend.configure((3,2,2,20,0))
        for i in range(4):obs=backend.observe(time.monotonic())
        self.assertIsNotNone(obs['coordinates']);self.assertEqual(obs['id'],2)
        backend.preview(backend.config,obs)
        self.assertEqual(shown[-1].width(),service.PREVIEW_WIDTH)
        backend.close()

    def test_camera_ownership_and_trackers_reset_on_switch(self):
        cameras=[]
        class Camera:
            def __init__(self,*args,**kwargs):
                self.closed=False;cameras.append(self)
                self.assertion=sum(not c.closed for c in cameras)
            def close(self):self.closed=True
        module=SimpleNamespace(CAM_WIDTH=320,CAM_HEIGHT=240,CAM_FPS=30,
                    load_samples=lambda:{},PlateTracker=lambda:object(),MiddleTracker=lambda:object(),
                    RingDetector=lambda:SimpleNamespace(),AlignmentState=lambda:object(),
                    cv2=SimpleNamespace(setNumThreads=lambda n:None))
        api=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray',FMT_RGB888='rgb'))
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),api,None)
        with patch.object(service,'detector_module',return_value=module):
            backend.configure((2,1,1,20,0));backend.configure((2,1,3,20,0))
            self.assertEqual(len(cameras),1)
            backend.configure((3,2,2,20,0));first_tracker=backend.ring_tracker
            backend.configure((3,3,2,20,0))
            self.assertIsNot(first_tracker,backend.ring_tracker)
            backend.configure((0,0,0,0,0))
            self.assertFalse(cameras[0].closed)
            self.assertEqual(len(cameras),1)
            backend.close()
        self.assertTrue(all(c.closed for c in cameras))
        self.assertTrue(all(c.assertion==1 for c in cameras))

    def test_plate_frames_released_and_camera_reused_on_ring_switch(self):
        import numpy as np
        exports=[];events=[];native_open=[0]
        class Frame(NativeImage):
            def __init__(self,array=None):self.array=np.ones((8,8,3),np.uint8) if array is None else array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_string(self,*args,**kwargs):pass
        class Camera:
            def __init__(self,*args,**kwargs):
                assert native_open[0]==0,'old camera native object still alive'
                native_open[0]+=1;events.append('open')
            def close(self):
                assert all(ref() is None for ref in exports),'old camera frame still held'
                events.append('close')
            def __del__(self):native_open[0]-=1;events.append('delete')
        last_shown=[]
        def show(frame,**kwargs):last_shown[:]=[frame]
        module=SimpleNamespace(CAM_WIDTH=320,CAM_HEIGHT=240,CAM_FPS=30,
                    PlateTracker=lambda:object(),MiddleTracker=lambda:object(),
                    RingDetector=lambda:SimpleNamespace(),AlignmentState=lambda:object(),
                    cv2=SimpleNamespace(setNumThreads=lambda n:None))
        api=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray',FMT_RGB888='rgb'),
                    Image=lambda *args:Frame(),COLOR_WHITE=0,Fit=SimpleNamespace(FIT_CONTAIN='contain'))
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),api,SimpleNamespace(show=show))
        with patch.object(service,'detector_module',return_value=module):
            backend.configure((8,1,0,20,0))
            old=Frame();exports.append(weakref.ref(old))
            backend.view=backend.displayed=old;last_shown[:]=[old];del old
            backend.configure((3,2,2,20,0))
            self.assertEqual(events,['open'])
            self.assertTrue(all(ref() is None for ref in exports))
            backend.close()
        self.assertEqual(native_open[0],0)

    def test_plate_to_ring_to_plate_actual_processing_and_preview(self):
        import cv2,numpy as np
        root=Path(__file__).resolve().parents[1]
        gray=cv2.imdecode(np.fromfile(str(root/'maixcam_ring/test_data/live_complete.png'),np.uint8),cv2.IMREAD_GRAYSCALE)
        gray=cv2.resize(gray,(480,360))
        plate=cv2.imdecode(np.fromfile(str(root/'maixcam_material/test_data/plate_lit.png'),np.uint8),cv2.IMREAD_COLOR)
        rgb=cv2.cvtColor(cv2.resize(plate,(480,360)),cv2.COLOR_BGR2RGB)
        ring_rgb=cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)
        scene=[rgb]
        active=[];reads=[];shown=[];copies=[]
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_string(self,*args,**kwargs):pass
            def draw_ellipse(self,*args,**kwargs):pass
            def draw_line(self,*args,**kwargs):pass
        class Camera:
            def __init__(self,w,h,fmt,**kwargs):
                assert not active
                active.append(self);self.fmt=fmt
            def read(self,block,block_ms):
                reads.append((block,block_ms));return Frame(scene[0])
            def close(self):active.remove(self)
        def wrap(array,bgr,copy):
            copies.append(copy);return Frame(array.copy() if copy else array)
        display=SimpleNamespace(width=lambda:552,height=lambda:368,show=lambda frame,**kwargs:shown.append(frame.array.shape))
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray',FMT_RGB888='rgb'),
                Fit=SimpleNamespace(FIT_CONTAIN='contain'),COLOR_WHITE=0,COLOR_YELLOW=1,COLOR_GREEN=2,COLOR_RED=3,
                Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)),
                image2cv=lambda frame,**kwargs:frame.array,cv2image=wrap)
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,display)
        for mode in (8,3,8,3):
            scene[0]=ring_rgb if mode==3 else rgb
            backend.configure((mode,1 if mode==8 else 2,0 if mode==8 else 2,20,0))
            for i in range(4):obs=backend.observe(time.monotonic())
            self.assertIsNotNone(obs['coordinates'])
            backend.preview(backend.config,obs)
            self.assertEqual(shown[-1],(service.PREVIEW_HEIGHT,service.PREVIEW_WIDTH,3))
        self.assertEqual(copies,[True,True])
        self.assertTrue(all(block and ms==250 for block,ms in reads))
        backend.close();self.assertEqual(active,[])

    def test_ring_converts_rgb_to_luminance_not_first_channel(self):
        import cv2,numpy as np
        class Frame:
            def width(self):return 40
            def height(self):return 30
        rgb=np.zeros((30,40,3),np.uint8);rgb[:,:,1]=200
        detector=SimpleNamespace(identity_rejected=False,used_roi=False)
        inspected=[]
        def process(gray,tracker,now):
            inspected.append(gray.copy());return [],None,gray,'SEARCH_THREE'
        detector.process=process;detector.accept=lambda *args:None
        tracker=SimpleNamespace(status='SEARCH_THREE',update=lambda *args:None)
        image=SimpleNamespace(image2cv=lambda *args,**kwargs:rgb)
        backend=service.MaixBackend(None,image,SimpleNamespace(width=lambda:40,height=lambda:30))
        backend.config=(3,2,2,20,0);backend.cam=SimpleNamespace(read=lambda **kwargs:Frame())
        backend.module=SimpleNamespace(cv2=cv2,np=np);backend.ring_detector=detector;backend.ring_tracker=tracker
        backend.alignment=SimpleNamespace(update=lambda coords:False)
        backend.observe(time.monotonic())
        np.testing.assert_array_equal(inspected[0],cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY))
        self.assertGreater(float(inspected[0].mean()),100)

    def test_same_ring_config_and_station_change_never_reopen_camera(self):
        opened=[];closed=[]
        class Camera:
            def __init__(self,w,h,fmt,**kwargs):opened.append((w,h,fmt))
            def close(self):closed.append(True)
        module=SimpleNamespace(MiddleTracker=lambda:object(),RingDetector=lambda:SimpleNamespace(),
                AlignmentState=lambda:object(),cv2=SimpleNamespace(setNumThreads=lambda n:None))
        image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'))
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,None)
        with patch.object(service,'detector_module',return_value=module):
            config=(3,2,2,20,0)
            backend.configure(config);tracker=backend.ring_tracker
            backend.configure(config)
            self.assertIsNot(backend.ring_tracker,tracker)
            backend.configure((3,3,2,20,0))
            self.assertIsNot(backend.ring_tracker,tracker)
            backend.configure((0,0,0,0,0))
            backend.configure((3,2,2,20,0))
        self.assertEqual(opened,[(480,360,'rgb')])
        self.assertEqual(closed,[])
        backend.close();self.assertEqual(closed,[True])

    def test_stop_clears_all_visual_state_but_keeps_camera_and_calibration(self):
        closed=[]
        backend=service.MaixBackend(None,None,None)
        camera=SimpleNamespace(close=lambda:closed.append(True))
        backend.cam=camera;backend.config=(3,2,2,20,0)
        backend.view=backend.displayed=object();backend.displayed_array=object()
        backend.ring_tracker=SimpleNamespace(lock=object(),count=3)
        backend.ring_detector=SimpleNamespace(template=object())
        backend.plate_tracker=SimpleNamespace(count=3,previous_y=120)
        backend.alignment=SimpleNamespace(count=5)
        backend.selected_color=6;backend.samples={'6':[[50,2,-40]]}
        backend.configure((0,0,0,0,0))
        for attr in ['ring_tracker','ring_detector','plate_tracker','alignment']:
            self.assertFalse(hasattr(backend,attr))
        self.assertIsNone(backend.view);self.assertIsNone(backend.displayed)
        self.assertEqual(backend.selected_color,0)
        self.assertEqual(backend.samples,{'6':[[50,2,-40]]})
        self.assertIs(backend.cam,camera);self.assertEqual(closed,[])
        # 第二个STOP也不能因同配置优化而跳过清理。
        backend.selected_color=3;backend.configure((0,0,0,0,0))
        self.assertEqual(backend.selected_color,0)

    def test_new_set_sequence_resets_same_mode_but_retry_preserves_acquisition(self):
        server=service.VisionService(Serial(),Backend(),Clock())
        request=service.encode_set(3,2,2,20,1)
        server.request(request,0)
        self.assertEqual(len(server.backend.configs),1)
        server.request(request,.5)
        self.assertEqual(len(server.backend.configs),1)
        server.request(service.encode_set(3,2,2,20,2),.6)
        self.assertEqual(len(server.backend.configs),2)

    def test_ring_reacquires_three_frames_after_stop_with_fresh_buffer(self):
        import cv2,numpy as np
        root=Path(__file__).resolve().parents[1]
        gray=cv2.imdecode(np.fromfile(str(root/'maixcam_ring/test_data/live_complete.png'),np.uint8),cv2.IMREAD_GRAYSCALE)
        picture=cv2.cvtColor(cv2.resize(gray,(480,360)),cv2.COLOR_GRAY2RGB)
        buffers=[];opened=[];closed=[]
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
        class Camera:
            def __init__(self,*args,**kwargs):opened.append(True)
            def read(self,**kwargs):return Frame(picture)
            def clear_buff(self):buffers.append(True)
            def close(self):closed.append(True)
        image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),image2cv=lambda frame,**kwargs:frame.array)
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,None)
        # 无显示分支，但observe需要屏幕几何；只给只读宽高，不显示。
        display=SimpleNamespace(width=lambda:552,height=lambda:368)
        config=(3,2,2,20,0)
        backend.configure(config);backend.display=display
        for _ in range(3):observation=backend.observe(time.monotonic())
        self.assertIsNotNone(observation['coordinates'])
        self.assertIsNotNone(backend.ring_tracker.lock)
        backend.display=None;backend.configure((0,0,0,0,0))
        self.assertFalse(hasattr(backend,'ring_detector'))
        backend.configure(config);backend.display=display
        for _ in range(2):
            observation=backend.observe(time.monotonic())
            self.assertIsNone(observation['coordinates'])
        self.assertIsNotNone(backend.observe(time.monotonic())['coordinates'])
        self.assertEqual(opened,[True]);self.assertEqual(closed,[])
        self.assertEqual(buffers,[True])
        backend.close()

    def test_plate_same_config_new_acquisition_clears_confirmation(self):
        module=service.detector_module('plate')
        camera=SimpleNamespace(clear_buff=lambda:None)
        backend=service.MaixBackend(None,None,None)
        backend.cam=camera;backend.config=(8,1,0,20,0)
        backend.plate_tracker=module.PlateTracker()
        backend.plate_tracker.count=3;backend.plate_tracker.previous_y=140
        previous=backend.plate_tracker
        backend.configure(backend.config)
        self.assertIsNot(backend.plate_tracker,previous)
        self.assertEqual(backend.plate_tracker.count,0)
        self.assertIsNone(backend.plate_tracker.previous_y)

    def test_nonzero_native_display_return_is_detected(self):
        import numpy as np
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
        api=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),Fit=SimpleNamespace(FIT_CONTAIN='contain'))
        backend=service.MaixBackend(None,api,SimpleNamespace(show=lambda *args,**kwargs: -1))
        with self.assertRaisesRegex(RuntimeError,'native VO failure'):
            backend.show_frame(Frame(np.zeros((240,320,3),np.uint8)))
        self.assertEqual(backend.stage,'display_show')

    def test_all_display_sizes_and_formats_are_identical(self):
        import numpy as np
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
        outputs=[]
        api=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),Fit=SimpleNamespace(FIT_CONTAIN='contain'))
        backend=service.MaixBackend(None,api,SimpleNamespace(show=lambda frame,**kwargs:outputs.append((frame.width(),frame.height(),frame.format()))))
        for shape in ((360,480,3),(240,320),(240,320,3)):
            backend.show_frame(Frame(np.zeros(shape,np.uint8)))
        self.assertEqual(outputs,[(320,240,'rgb')]*3)

    def test_missing_camera_frame_reports_read_stage_and_service_survives(self):
        api=SimpleNamespace()
        backend=service.MaixBackend(None,api,None)
        backend.config=(3,2,2,20,0);backend.module=SimpleNamespace()
        calls=[]
        def read(**kwargs):calls.append(kwargs);return None
        backend.cam=SimpleNamespace(read=read)
        with self.assertRaisesRegex(RuntimeError,'camera returned no frame'):
            backend.observe(time.monotonic())
        self.assertEqual(backend.stage,'camera_read')
        self.assertEqual(calls,[{'block':True,'block_ms':250}])

    def test_status_image_replaces_previous_camera_image_when_invalid(self):
        messages=[];shown=[]
        class StatusFrame:
            def draw_string(self,x,y,text,color):messages.append(text)
            def format(self):return 'rgb'
            def width(self):return service.PREVIEW_WIDTH
            def height(self):return service.PREVIEW_HEIGHT
        api=SimpleNamespace(Image=lambda *args:StatusFrame(),Format=SimpleNamespace(FMT_RGB888='rgb'),
                Fit=SimpleNamespace(FIT_CONTAIN='contain'),COLOR_WHITE=1)
        backend=service.MaixBackend(None,api,SimpleNamespace(show=lambda frame,**kwargs:shown.append(frame)))
        backend.config=(3,2,2,20,0);backend.status='Visual unavailable'
        backend.preview(backend.config,{'coordinates':None})
        self.assertIn('X:-- Y:-- Visual unavailable',messages)
        self.assertIs(backend.displayed,shown[-1])
        self.assertIsNone(backend.displayed_array)
    def test_generated_single_file_loads_detectors_without_side_files(self):
        build_path=Path(__file__).resolve().parents[2]/'tools/build_vision_service.py'
        builder_spec=importlib.util.spec_from_file_location('service_builder',build_path)
        builder=importlib.util.module_from_spec(builder_spec);builder_spec.loader.exec_module(builder)
        path=builder.build()
        spec=importlib.util.spec_from_file_location('generated_service',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertEqual(set(module._BUNDLED_DETECTORS),{'material','plate','ring'})
        with patch.object(module.importlib.util,'spec_from_file_location',side_effect=AssertionError('side file')):
            for name in module._BUNDLED_DETECTORS:
                sys.modules.pop('service_detector_'+name,None)
                detector=module.detector_module(name)
                self.assertTrue(hasattr(detector,'CAM_WIDTH'))


if __name__=='__main__':unittest.main()
