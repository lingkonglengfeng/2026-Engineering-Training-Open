"""Real loopback TCP + saved image fixtures; no robot or camera is commanded."""
import csv
import gc
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'vision/pc_vision'),str(ROOT/'vision/maixcam_service')]
import cv2
import numpy as np
import wire
sys.modules['pc_wire'] = wire
import backend
import transport
import discovery
from serial_monitor import SerialLog,timestamp,command_packet
import service
from material_surface import SurfaceAnalyzer
from color_classifier import BluePairModel,pc_calibrated_threshold,black_pixel_gate
from ring_refinement import refine_occupied
from motion_prediction import MotionPredictor, BackgroundMotion
from settings import motion_settings
from qr_backend import PcQrBackend,qr_service


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


bridge=load('tested_bridge','vision/maixcam_bridge/bridge.py')
desktop=load('tested_desktop','vision/pc_vision/main.py')


def photo(name='vision/maixcam_ring/test_data/live_complete.png'):
    bgr=cv2.imdecode(np.fromfile(str(ROOT/name),np.uint8),cv2.IMREAD_COLOR)
    return cv2.resize(bgr,(480,360))


def qr_photo(index=1,two=False):
    paths=sorted((ROOT/'vision/pc_vision/test_data/qr').glob('*.png'))
    image=np.full((480,640,3),255,np.uint8)
    if two:
        for path,x in ((paths[0],20),(paths[1],380)):
            code=cv2.imdecode(np.fromfile(str(path),np.uint8),1)
            image[120:360,x:x+240]=cv2.resize(code,(240,240))
    else:
        code=cv2.imdecode(np.fromfile(str(paths[index-1]),np.uint8),1)
        image[85:395,165:475]=cv2.resize(code,(310,310))
    return cv2.rotate(image,cv2.ROTATE_180)


class Loopback:
    def __init__(self,pixels=None,role='material'):
        self.state=bridge.BridgeState(role)
        self.video,self.control=bridge.server_socket(0),bridge.server_socket(0)
        self.video_port,self.control_port=self.video.getsockname()[1],self.control.getsockname()[1]
        pixels=photo() if pixels is None else pixels
        class Jpeg:
            def to_bytes(self):return cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tobytes()
        class Frame:
            def to_jpeg(self,quality):return Jpeg()
            def width(self):return pixels.shape[1]
            def height(self):return pixels.shape[0]
        class Camera:
            def __init__(self,*a,**k):pass
            def read(self,**k):
                time.sleep(.004)
                return Frame()
            def clear_buff(self):pass
            def close(self):pass
        self.threads=[threading.Thread(target=bridge.video_server,args=(self.state,self.video,SimpleNamespace(Camera=Camera),SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0))),daemon=True),
                      threading.Thread(target=bridge.control_server,args=(self.state,self.control),daemon=True)]
        for thread in self.threads:thread.start()

    def close(self):
        self.state.stop.set()
        for thread in self.threads:thread.join(1.5)
        self.video.close();self.control.close()


class WireTests(unittest.TestCase):
    def test_fragmented_tcp_and_size_bound(self):
        left,right=socket.socketpair()
        try:
            packet=wire.encode(wire.REQUEST,b'123456789012')
            def send():
                for value in packet:right.send(bytes([value]))
            thread=threading.Thread(target=send);thread.start()
            self.assertEqual(wire.receive(left,1),(wire.REQUEST,b'123456789012'))
            thread.join()
            right.sendall(wire.HEADER.pack(wire.MAGIC,wire.FRAME,wire.MAX_IMAGE+1))
            with self.assertRaises(ValueError):wire.receive(left,1)
        finally:left.close();right.close()

    def test_partial_disconnect(self):
        left,right=socket.socketpair()
        right.sendall(b'PC');right.close()
        try:
            with self.assertRaises(ConnectionError):wire.receive(left,.2)
        finally:left.close()

    def test_frame_geometry_checked(self):
        payload=wire.FRAME_META.pack(1,2,3,320,240,0)+b'jpeg'
        with self.assertRaises(ValueError):wire.unpack_frame(payload)


class BridgeTests(unittest.TestCase):
    def test_qr_group_is_atomic_sequence_bound_and_material_bridge_rejects_it(self):
        state=bridge.BridgeState('qr');state.connection=1
        state.accept_request(qr_service.encode_set(6,4,20,5),1)
        stamp=1_000_000_000;state.remember_frame(state.epoch,7,stamp)
        packet=qr_service.encode_task('156+123+516+231',5)
        payload=wire.RESULT_META.pack(state.epoch,7,stamp)+packet
        self.assertTrue(bridge.valid_group(packet,'qr'))
        self.assertFalse(bridge.valid_group(packet,'material'))
        self.assertEqual(state.result_current(1,payload,stamp+1),packet)
        self.assertEqual(state.display_task,'156+123+516+231')
        bad=qr_service.encode_task('156+123+516+231',6)
        self.assertIsNone(state.result_current(1,wire.RESULT_META.pack(state.epoch,7,stamp)+bad,stamp+1))
        self.assertFalse(bridge.valid_group(packet[:24],'qr'))
        self.assertFalse(bridge.valid_group(packet[:-1]+bytes([packet[-1]^1]),'qr'))
        material_packet=service.encode_pixel_frame(2,1,(10,20),5)
        self.assertFalse(bridge.valid_group(material_packet,'qr'))
        state.accept_request(qr_service.encode_set(0,4,0,6),1.1)
        self.assertIsNone(state.result_current(1,payload,stamp+1))
        self.assertEqual(state.display_task,'156+123+516+231')
        state.accept_request(qr_service.encode_set(6,4,20,7),1.2)
        self.assertIsNone(state.display_task)

    def test_duplicate_requests_and_new_epoch(self):
        state=bridge.BridgeState();packet=service.encode_set(2,1,1,20,1)
        state.accept_request(packet,1);epoch=state.epoch
        state.accept_request(packet,1.5);self.assertEqual(state.epoch,epoch)
        state.accept_request(service.encode_set(0,0,0,0,2),1.6)
        self.assertNotEqual(state.epoch,epoch)

    def test_result_epoch_timestamp_connection_and_crc(self):
        state=bridge.BridgeState();state.connection=1;state.epoch=2
        stamp=1_000_000_000;state.remember_frame(2,7,stamp)
        packet=service.encode_pixel_frame(2,1,(10,20),1)
        payload=wire.RESULT_META.pack(2,7,stamp)+packet
        self.assertEqual(state.result_current(1,payload,stamp+10_000_000),packet)
        self.assertIsNone(state.result_current(2,payload,stamp+10_000_000))
        self.assertIsNone(state.result_current(1,payload,stamp+501_000_000))
        self.assertIsNone(state.result_current(1,wire.RESULT_META.pack(2,8,stamp)+packet,stamp))
        self.assertIsNone(state.result_current(1,payload[:-1]+bytes([packet[-1]^1]),stamp))
        state.accept_request(service.encode_set(0,0,0,0,3),2)
        self.assertIsNone(state.result_current(1,payload,stamp))


class BackendTests(unittest.TestCase):
    def make_backend(self,bgr):
        class Video:
            last=0
            def get(self,epoch):
                self.last+=1
                return (epoch,self.last,1_000_000_000+self.last*33_333_333,480,360,500),cv2.imencode('.jpg',bgr)[1].tobytes(),3.
        control=SimpleNamespace(hello={'calibration':{}},epoch=0,frame=(0,0,0))
        return backend.PcBackend(Video(),control,Path(tempfile.gettempdir())/'nonexistent_pcvision_fixture_samples.json',False)

    def test_decode_render_ring_service(self):
        obj=self.make_backend(photo());obj.configure((3,2,2,20,1))
        result=None
        for _ in range(5):result=obj.observe(time.monotonic())
        self.assertEqual(obj.image.shape,(360,480,3))
        self.assertTrue(result['ring_count_trusted'])
        self.assertEqual(result['ring_count'],3)
        self.assertIsNotNone(result['coordinates'])
        self.assertEqual(obj.render().shape,(690,800,3))

    def test_plate_real_fixture(self):
        obj=self.make_backend(photo('vision/maixcam_material/test_data/plate_lit.png'))
        obj.configure((8,1,0,20,0))
        for _ in range(4):result=obj.observe(time.monotonic())
        self.assertEqual(result['plate_scene'],3)
        self.assertIsNotNone(result['coordinates'])

    def test_legacy_material_transmits_on_second_distinct_frame(self):
        obj=self.make_backend(photo());obj.surface_enabled=False
        obj.configure((2,1,1,20,0))
        with patch.object(backend.material,'detect_material_center',return_value=((160.,100.,25.),None)):
            self.assertIsNone(obj.observe(time.monotonic())['coordinates'])
            self.assertEqual(obj.observe(time.monotonic())['coordinates'],(0.,20.))
        with patch.object(backend.material,'detect_material_center',return_value=(None,None)):
            self.assertIsNone(obj.observe(time.monotonic())['coordinates'])
        with patch.object(backend.material,'detect_material_center',return_value=((160.,100.,25.),None)):
            self.assertIsNone(obj.observe(time.monotonic())['coordinates'])
            self.assertEqual(obj.observe(time.monotonic())['coordinates'],(0.,20.))

    def test_material_confirmation_rejects_repeated_frame_and_resets_color_gap(self):
        obj=self.make_backend(photo());point=(2.,3.)
        self.assertIsNone(obj.confirm_material(1,point,100.))
        self.assertIsNone(obj.confirm_material(1,point,100.))
        self.assertEqual(obj.confirm_material(1,point,100.03),point)
        self.assertIsNone(obj.confirm_material(2,point,100.06))
        self.assertEqual(obj.confirm_material(2,point,100.09),point)
        self.assertIsNone(obj.confirm_material(2,point,101.))

    def test_plate_target_75_percent_matches_error_and_visible_line(self):
        target={'tangent_x':160.,'tangent_y':180.}
        self.assertEqual(backend.plate.tangent_alignment_error(target,320,240,320,240),(0.,0.))
        target['tangent_y']=156.
        self.assertEqual(backend.plate.tangent_alignment_error(target,320,240,320,240),(0.,24.))
        self.assertEqual(backend.plate.tangent_alignment_error(target,320,240,480,360),(0.,36.))
        class Frame:
            def __init__(self):self.lines=[]
            def width(self):return 320
            def height(self):return 240
            def draw_line(self,*args,**kwargs):self.lines.append((args,kwargs))
        frame=Frame();backend.plate.draw_plate_overlay(frame,None,SimpleNamespace(COLOR_WHITE='white'))
        self.assertEqual(frame.lines[0][0],(0,180,319,180,'white'))
        obj=self.make_backend(np.full((360,480,3),120,np.uint8));obj.configure((8,1,0,20,0))
        result=obj.observe(time.monotonic());self.assertIsNone(result['coordinates'])
        self.assertTrue(np.all(obj.display_image[180]==255))
        self.assertTrue(np.all(obj.display_image[156]==120))

    def plate_photo_backend(self,images):
        class Video:
            last=0
            def get(self,epoch):
                bgr=images[self.last%len(images)];self.last+=1
                # Lossless transport preserves the pixels of these saved,
                # already JPEG-decoded camera frames for exact regression.
                return (epoch,self.last,1_000_000_000+self.last*33_333_333,480,360,500),cv2.imencode('.png',bgr)[1].tobytes(),3.
        control=SimpleNamespace(hello={'calibration':{}},epoch=0,frame=(0,0,0))
        obj=backend.PcBackend(Video(),control,Path(tempfile.gettempdir())/'nonexistent_pcvision_fixture_samples.json',False)
        obj.configure((8,1,0,20,0));return obj

    def test_soft_plate_photos_confirm_without_relaxing_geometry(self):
        self.assertEqual(backend.plate.PROCESS_WIDTH,224)  # Device default is preserved.
        for name in ('a','b','c'):
            with self.subTest(photo=name):
                bgr=photo('vision/pc_vision/test_data/plate_soft_edge_'+name+'.png')
                obj=self.plate_photo_backend([bgr])
                first=obj.observe(time.monotonic());second=obj.observe(time.monotonic());third=obj.observe(time.monotonic())
                self.assertIsNone(first['coordinates']);self.assertIsNone(second['coordinates'])
                self.assertEqual(third['plate_scene'],3)
                self.assertIsNotNone(third['coordinates'])
                self.assertTrue(20<third['coordinates'][0]<28)  # Return measured error, not zero.
                self.assertLess(abs(third['coordinates'][1]-24),6)  # Same real edge, target moved down24px.

    def test_three_different_soft_plate_images_pass_temporal_confirmation(self):
        images=[photo('vision/pc_vision/test_data/plate_soft_edge_'+name+'.png') for name in ('a','b','c')]
        obj=self.plate_photo_backend(images)
        results=[obj.observe(time.monotonic()) for _ in images]
        self.assertEqual([r['plate_scene'] for r in results],[2,2,3])
        self.assertIsNotNone(results[-1]['coordinates'])

    def test_full_pc_plate_fit_rejects_blank_and_straight_floor_edges(self):
        floor=np.full((360,480,3),180,np.uint8)
        cv2.line(floor,(0,132),(479,132),(130,130,130),2)
        for image in (np.zeros_like(floor),np.full_like(floor,210),floor):
            obj=self.plate_photo_backend([image])
            for _ in range(4):result=obj.observe(time.monotonic())
            self.assertIsNone(result['coordinates'])
            self.assertEqual(result['plate_scene'],1)

    def test_rough_temp_photos_report_current_ring_mode_and_ground_centers(self):
        for name,station,expected in [('rough_occupied_raw.png',2,(348.4,163.3)),
                                      ('temp_occupied_a_raw.png',3,(253.9,145.0)),
                                      ('temp_occupied_b_raw.png',3,(237.5,178.1))]:
            with self.subTest(image=name):
                image=cv2.imdecode(np.fromfile(str(ROOT/'vision/pc_vision/test_data'/name),np.uint8),1)
                obj=self.make_backend(image);obj.configure((3,station,2,20,3))
                for i in range(4):result=obj.observe(100+i/30)
                self.assertEqual(result['ring_count'],3)
                self.assertTrue(result['ring_count_trusted'])
                self.assertIsNotNone(result['coordinates'])
                self.assertAlmostEqual(result['coordinates'][0],expected[0]-240,delta=3)
                self.assertAlmostEqual(result['coordinates'][1],180-expected[1],delta=3)
                self.assertEqual(obj.ring_report['profile'],'occupied')
                self.assertTrue(obj.ring_report['coordinates_valid'])
                self.assertIsNotNone(result.get('ring_axis'))
                import gui
                self.assertIn('圆环定位',gui.task_text(obj.config))
                self.assertIn('圆盘定位',gui.task_text((8,1,0,20,0)))

    def test_material_color_mapping_and_sign(self):
        bgr=np.full((360,480,3),120,np.uint8)
        cv2.circle(bgr,(290,140),32,(0,0,240),-1)
        obj=self.make_backend(bgr);obj.configure((2,1,1,20,0))
        self.assertIsNone(obj.observe(time.monotonic())['coordinates'])
        result=obj.observe(time.monotonic())
        self.assertEqual(result['id'],1)
        self.assertGreater(result['coordinates'][0],0)
        self.assertGreater(result['coordinates'][1],0)  # internal y-up; protocol encoder flips y
        self.assertEqual(backend.material.PROTOCOL_COLOR_IDS[5:7],(6,5))

    def test_legacy_black_return_rejects_cyan_shadow_and_keeps_true_black(self):
        data=ROOT/'vision/pc_vision/test_data'
        samples=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))['samples']
        image=np.full((360,480,3),190,np.uint8)
        cv2.circle(image,(240,160),35,(90,65,30),-1)  # cyan side inside OLD black LAB box
        old,_=backend.material.calibrated_threshold(backend.material.COLORS[4],samples)
        old_target,_=backend.material.detect_material_center(backend.MaterialFrame(cv2.resize(image,(320,240))),old,backend.MATERIAL_API,{})
        self.assertIsNotNone(old_target)
        for enhanced in (False,True):
            obj=self.make_backend(image);obj.samples=samples;obj.use_surface_output=enhanced
            obj.configure((2,1,6,20,0))
            for _ in range(4):result=obj.observe(time.perf_counter())
            self.assertIsNone(result['coordinates'])
        cyan=self.make_backend(image);cyan.samples=samples;cyan.configure((2,1,5,20,0))
        self.assertIsNone(cyan.observe(time.perf_counter())['coordinates'])
        cyan_result=cyan.observe(time.perf_counter())
        self.assertEqual(cyan_result['id'],5)
        self.assertIsNotNone(cyan_result['coordinates'])
        image[:]=190;cv2.circle(image,(240,160),35,(28,28,28),-1)
        obj=self.make_backend(image);obj.samples=samples;obj.configure((2,1,6,20,0))
        self.assertIsNone(obj.observe(time.perf_counter())['coordinates'])
        result=obj.observe(time.perf_counter())
        self.assertEqual(result['id'],6)
        self.assertIsNotNone(result['coordinates'])
        self.assertEqual(obj.surface_report['output'],'legacy')

    def test_real_photo_black_mask_excludes_cyan_dark_side(self):
        data=ROOT/'vision/pc_vision/test_data'
        samples=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))['samples']
        image=cv2.imdecode(np.fromfile(str(data/'black_cyan_raw.png'),np.uint8),1)
        lab=backend.lab_values(image);hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        old,_=backend.material.calibrated_threshold(backend.material.COLORS[4],samples)
        new,_=pc_calibrated_threshold(backend.material.COLORS[4],samples,backend.material)
        before=cv2.inRange(lab,np.float32(old[::2]),np.float32(old[1::2]))
        after=cv2.inRange(lab,np.float32(new[::2]),np.float32(new[1::2]))
        after[~black_pixel_gate(lab,hsv)]=0
        cyan=(slice(134,147),slice(239,252))
        black=(slice(165,178),slice(442,455))
        self.assertGreater(np.mean(before[cyan]>0),.40)
        self.assertEqual(np.count_nonzero(after[cyan]),0)
        self.assertGreater(np.mean(after[black]>0),.95)
        self.assertEqual(new,[0,32,-12,12,-12,12])
        cyan_image=cv2.imdecode(np.fromfile(str(data/'cyan_only_raw.png'),np.uint8),1)
        cyan_lab=backend.lab_values(cyan_image)
        cyan_mask=cv2.inRange(cyan_lab,np.float32(new[::2]),np.float32(new[1::2]))
        cyan_mask[~black_pixel_gate(cyan_lab,cv2.cvtColor(cyan_image,cv2.COLOR_BGR2HSV))]=0
        self.assertEqual(np.count_nonzero(cyan_mask[129:142,246:259]),0)

    def test_calibration_headless_rejected_without_success_ack(self):
        obj=self.make_backend(photo())
        with self.assertRaises(ValueError):obj.configure((2,1,1,20,4))

    def test_stop_preview_consumes_every_available_frame_and_yields_to_set(self):
        obj=self.make_backend(photo())
        obj.idle_preview();first=obj.frame_count
        obj.idle_preview()
        self.assertEqual(obj.frame_count,first+1)
        obj.configure((8,1,0,20,0))
        with patch.object(obj.video,'get',side_effect=transport.StreamContextChanged('new SET')):
            obj.idle_preview()  # no failure or wait for the abandoned context
            self.assertIsNone(obj.observe(time.perf_counter())['coordinates'])

    def test_surface_output_uses_320_pixel_units_and_current_frame(self):
        bgr=np.full((360,480,3),180,np.uint8)
        cv2.circle(bgr,(290,140),30,(0,0,240),-1)
        obj=self.make_backend(bgr);obj.preview_only=True;obj.configure((2,1,1,20,0))
        first=obj.observe(time.perf_counter())
        self.assertIsNone(first['coordinates'])
        for _ in range(3):result=obj.observe(time.perf_counter())
        self.assertAlmostEqual(result['coordinates'][0],290*320/480-160,delta=1)
        self.assertAlmostEqual(result['coordinates'][1],120-140*240/360,delta=1)
        self.assertEqual(obj.surface_report['output'],'surface')


class DiscoveryTests(unittest.TestCase):
    def test_udp_discovers_both_roles_without_opening_control_or_changing_state(self):
        states=[bridge.BridgeState('material'),bridge.BridgeState('qr')]
        sockets=[bridge.discovery_socket(0),bridge.discovery_socket(0)]
        threads=[]
        try:
            for state,sock in zip(states,sockets):
                thread=threading.Thread(target=bridge.discovery_server,args=(state,sock),daemon=True)
                threads.append(thread);thread.start()
            targets=[('127.0.0.1',sock.getsockname()[1]) for sock in sockets]
            devices=discovery.scan_devices(timeout=.25,targets=targets)
            self.assertEqual({device['role'] for device in devices},{'material','qr'})
            self.assertEqual(len(devices),2)
            self.assertTrue(all(device['host']=='127.0.0.1' and device['control_port']==9001 for device in devices))
            for state in states:
                self.assertEqual(state.connection,0);self.assertEqual(state.epoch,0)
                self.assertIsNone(state.request);self.assertFalse(state.capture_active)
                self.assertTrue(state.results.empty())
        finally:
            for state in states:state.stop.set()
            for thread in threads:thread.join(1)
            for sock in sockets:sock.close()

    def test_discovery_ignores_wrong_nonce_and_bad_endpoints_and_can_cancel(self):
        value={'protocol':wire.DISCOVERY_RESPONSE,'nonce':'0123456789abcdef','role':'material',
               'name':'maixcam-test','build':'fixture','control_port':9001,'video_port':9000}
        payload=json.dumps(value).encode()
        self.assertIsNotNone(discovery.parse_device(payload,('127.0.0.1',9002),value['nonce']))
        self.assertIsNone(discovery.parse_device(payload,('127.0.0.1',9002),'bad-nonce'))
        for change in ({'role':'unrelated'},{'control_port':0},{'video_port':'9000'}):
            bad=json.dumps(dict(value,**change)).encode()
            self.assertIsNone(discovery.parse_device(bad,('127.0.0.1',9002),value['nonce']))
        cancel=threading.Event();cancel.set()
        self.assertEqual(discovery.scan_devices(cancel=cancel,targets=[]),[])

    def test_windows_adapter_broadcasts_include_wifi_and_usb_and_saved_ip(self):
        adapters=[{'IPAddress':'192.168.31.5','PrefixLength':24},{'IPAddress':'10.131.167.100','PrefixLength':24},
                  {'IPAddress':'127.0.0.1','PrefixLength':8}]
        result=SimpleNamespace(returncode=0,stdout=json.dumps(adapters))
        with patch.object(discovery.subprocess,'run',return_value=result):
            targets=discovery.broadcast_targets(['192.168.31.44','maixcam.local'])
        for host in ('255.255.255.255','192.168.31.255','10.131.167.255','192.168.31.44'):
            self.assertIn((host,9002),targets)
        self.assertNotIn(('127.255.255.255',9002),targets)


class SerialMonitorTests(unittest.TestCase):
    class Device:
        def __init__(self,chunks=(),fail=False):
            self.chunks=list(chunks);self.fail=fail;self.opened=self.closed=False
            self.port=None;self.dtr=self.rts=True
        @property
        def in_waiting(self):return len(self.chunks[0]) if self.chunks else 0
        def open(self):
            if self.fail:raise PermissionError('COM10 busy')
            self.opened=True
        def read(self,count):
            if self.chunks:return self.chunks.pop(0)
            time.sleep(.01);return b''
        def write(self,*args):raise AssertionError('Log receiver must never transmit')
        def close(self):self.closed=True

    def test_manual_commands_finish_partial_writes_once_and_are_timestamped(self):
        class Writer(self.Device):
            def __init__(self):super().__init__();self.sent=bytearray()
            def write(self,data):
                part=data[:3];self.sent.extend(part);return len(part)
        with tempfile.TemporaryDirectory() as temp:
            device=Writer();log=SerialLog(temp,serial_factory=lambda:device)
            with self.assertRaises(RuntimeError):log.send('&OK')
            log.start('COM10',115200)
            try:
                deadline=time.perf_counter()+1
                while not log.connected and time.perf_counter()<deadline:time.sleep(.01)
                self.assertEqual(device.sent,b'')
                log.send('&prepare 1');log.send('&OK')
                deadline=time.perf_counter()+1
                while time.perf_counter()<deadline:
                    with log.lock:tx=[r for r in log.history if r['kind']=='TX']
                    if len(tx)==2:break
                    time.sleep(.01)
                self.assertEqual(device.sent,b'&prepare 1\r\n&OK\r\n')
                self.assertEqual([r['text'] for r in tx],['&prepare 1','&OK'])
                frame=time.perf_counter();context=log.mark_snapshot('CAM1',42,frame,'image.png',timestamp())
                self.assertEqual(context['last_tx_before_frame']['text'],'&OK')
                self.assertIn('TX &OK',log.path.read_text(encoding='utf-8'))
                for bad in ('&OK\n&prepare 1','中文','&'+'x'*31,'x'*64):
                    with self.assertRaises(ValueError):command_packet(bad)
            finally:log.stop();log.thread.join(1)

    def test_fragmented_serial_lines_and_snapshot_marker_share_a_clock(self):
        device=self.Device([b'[MATCH] MOVING TEMP_BAT',b'CH1 zone=1\r\n[CAM1] request RING station=3\r\n',
                            b'\xe4',b'\xb8\xad\xe6\x96\x87\r\n'])
        with tempfile.TemporaryDirectory() as temp:
            log=SerialLog(temp,serial_factory=lambda:device)
            try:
                self.assertIsNone(log.thread)
                log.start('COM10',115200);deadline=time.perf_counter()+1
                while time.perf_counter()<deadline:
                    with log.lock:lines=[r['text'] for r in log.history]
                    if '中文' in lines:break
                    time.sleep(.01)
                self.assertIn('中文',lines)
                frame_time=time.perf_counter();stamp=timestamp()
                context=log.mark_snapshot('CAM1',42,frame_time,Path(temp)/'frame_raw.png',stamp)
                self.assertTrue(context['connected'])
                self.assertIn('TEMP_BATCH1',context['last_match_before_frame']['text'])
                self.assertIn('RING station=3',context['last_camera_request_before_frame']['text'])
                text=log.path.read_text(encoding='utf-8')
                self.assertIn('#%d SNAPSHOT'%context['marker_record_id'],text)
                self.assertIn('"frame_pc_perf_s": '+str(frame_time),text)
                self.assertFalse(device.dtr);self.assertFalse(device.rts)
                self.assertEqual(device.port,'COM10');self.assertIsNone(log.error)
            finally:log.stop();log.thread.join(1)
            self.assertTrue(device.closed)

    def test_frame_context_precedes_later_controller_stage_and_failed_port_is_reported(self):
        with tempfile.TemporaryDirectory() as temp:
            log=SerialLog(temp)
            log._record('RX','[MATCH] MOVING TEMP_BATCH1',{'perf_s':10.,'wall_time':'time10'})
            log._record('RX','[CAM1] request RING station=3',{'perf_s':10.1,'wall_time':'time10.1'})
            log._record('RX','[MATCH] automatic continuation',{'perf_s':10.2,'wall_time':'time10.2'})
            log._record('RX','[MATCH] MOVING RAW_BATCH2',{'perf_s':11.,'wall_time':'time11'})
            context=log.mark_snapshot('CAM1',42,10.5,'image.png',{'perf_s':11.5,'wall_time':'time11.5'})
            self.assertIn('TEMP_BATCH1',context['last_stage_before_frame']['text'])
            self.assertEqual(len(context['nearby_serial']),4)
            self.assertFalse(context['connected']);self.assertIsNone(context['log_path'])
            device=self.Device(fail=True);failed=SerialLog(temp,serial_factory=lambda:device)
            failed.start('COM10',115200);failed.thread.join(1)
            self.assertIn('COM10 busy',failed.error)
            self.assertFalse(failed.connected);self.assertIsNone(failed.path)


class QrBackendTests(unittest.TestCase):
    def make_backend(self,pixels):
        class Video:
            frames=0
            def get(self,epoch):
                self.frames+=1
                return (epoch,self.frames,time.monotonic_ns(),640,480,100,100,100),cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tobytes(),0.
        return PcQrBackend(Video(),SimpleNamespace(hello={'rotation':180},epoch=0,frame=(0,0,0)),
                           Path(tempfile.gettempdir())/'nonexistent_pcvision_qr_samples.json')

    def test_eight_actual_tasks_require_three_full_frames_and_preserve_rules(self):
        paths=sorted((ROOT/'vision/pc_vision/test_data/qr').glob('*.png'))
        for index,path in enumerate(paths,1):
            with self.subTest(image=path.name):
                obj=self.make_backend(qr_photo(index));obj.configure((6,4,0,20,0))
                expected='+'.join(path.stem.split('_')[1:])
                self.assertIsNone(obj.observe(time.perf_counter())['task'])
                self.assertIsNone(obj.observe(time.perf_counter())['task'])
                self.assertEqual(obj.observe(time.perf_counter())['task'],expected)
                self.assertEqual(obj.raw_image.shape,(480,640,3))
                self.assertEqual(obj.qr_report['decode_size'],[640,480])
                self.assertEqual(obj.qr_report['rotation'],180)
                obj.preview(obj.config,None,expected)
                self.assertEqual(obj.qr_report['task'],expected)

    def test_two_legal_codes_are_ambiguous_and_latched_view_skips_redecoding(self):
        obj=self.make_backend(qr_photo(two=True));obj.configure((6,4,0,20,0))
        for _ in range(4):self.assertIsNone(obj.observe(time.perf_counter())['task'])
        self.assertEqual(obj.tracker.status,'AMBIGUOUS')
        obj=self.make_backend(qr_photo());obj.configure((6,4,0,20,0))
        for _ in range(3):task=obj.observe(time.perf_counter())['task']
        obj.preview(obj.config,None,task)
        def forbidden(*args):raise AssertionError('Locked QR must not run the decoder again')
        obj.decoder=SimpleNamespace(detectAndDecodeMulti=forbidden)
        obj.observe(time.perf_counter(),scan=False)
        self.assertEqual(obj.qr_report['task'],task)
        obj.configure((0,4,0,0,0));obj.preview(obj.config,None,task)
        self.assertEqual(obj.task_display,task)
        obj.configure((6,4,0,20,0))
        self.assertIsNone(obj.task_display)
        with patch.object(obj.video,'get',side_effect=transport.StreamContextChanged('next SET')):
            self.assertIsNone(obj.observe(time.perf_counter())['task'])


class GeometryTests(unittest.TestCase):
    def test_material_surface_confirms_on_frame_two_and_reacquires_after_loss(self):
        image=np.full((240,320,3),180,np.uint8);cv2.circle(image,(160,100),25,(0,0,240),-1)
        analyzer=SurfaceAnalyzer(backend.material)
        self.assertIsNone(analyzer.process(image,{},1,100.,False)['target'])
        self.assertIsNotNone(analyzer.process(image,{},1,100.03,False)['target'])
        self.assertIsNone(analyzer.process(np.full_like(image,180),{},1,100.06,False)['target'])
        self.assertIsNone(analyzer.process(image,{},1,100.09,False)['target'])
        self.assertIsNotNone(analyzer.process(image,{},1,100.12,False)['target'])

    def surface_scene(self,image,analyzer=None):
        analyzer=analyzer or SurfaceAnalyzer(backend.material)
        for i in range(4):scene=analyzer.process(image,{},1,100+i/30,False)
        return analyzer,scene

    def test_cylinder_top_is_not_whole_body_centroid(self):
        image=np.full((240,320,3),180,np.uint8)
        cv2.rectangle(image,(140,90),(180,155),(0,0,120),-1)
        cv2.ellipse(image,(160,155),(20,12),0,0,180,(0,0,120),-1)
        cv2.circle(image,(160,90),20,(0,0,240),-1)
        _,scene=self.surface_scene(image)
        self.assertIsNotNone(scene['target'])
        self.assertLess(np.linalg.norm(np.array(scene['target']['center'])-[160,90]),2)
        self.assertGreater(scene['target']['purity'],.9)

    def test_same_color_ambiguity_rectangle_clipping_and_loss(self):
        image=np.full((240,320,3),180,np.uint8)
        cv2.circle(image,(80,100),22,(0,0,240),-1);cv2.circle(image,(240,100),22,(0,0,240),-1)
        _,scene=self.surface_scene(image)
        self.assertIsNone(scene['target'])
        self.assertTrue(any(item['reason']=='SAME_COLOR_AMBIGUOUS' for item in scene['surfaces']))
        rectangle=np.full_like(image,180);cv2.rectangle(rectangle,(70,80),(240,125),(0,0,240),-1)
        _,scene=self.surface_scene(rectangle);self.assertIsNone(scene['target'])
        clipped=np.full_like(image,180);cv2.circle(clipped,(0,100),25,(0,0,240),-1)
        _,scene=self.surface_scene(clipped);self.assertIsNone(scene['target'])
        circle=np.full_like(image,180);cv2.circle(circle,(160,100),22,(0,0,240),-1)
        analyzer,scene=self.surface_scene(circle);self.assertIsNotNone(scene['target'])
        lost=analyzer.process(np.full_like(image,180),{},1,100.14,False)
        self.assertIsNone(lost['target']);self.assertNotIn(1,analyzer.tracks)

    def test_blue_identity_requires_paired_calibration(self):
        image=np.full((240,320,3),180,np.uint8);cv2.circle(image,(160,100),25,(240,0,0),-1)
        analyzer=SurfaceAnalyzer(backend.material)
        scene=analyzer.process(image,{},3,100,False)
        self.assertIsNone(scene['target'])
        self.assertEqual(scene['reason'],'BLUE_PAIR_CALIBRATION')

    def test_blue_chroma_classifier_handles_exposure_and_rejects_neutrals(self):
        samples={'3':[[48,14,-33]]*3,'6':[[50,-9,-27]]*3}
        model=BluePairModel(samples)
        pixels=np.uint8([[[101,32,13],[161,122,65],[103,74,43],[80,80,80],[90,130,50]]])
        for gain in (.55,.8,1.,1.25,1.5):
            image=np.clip(pixels.astype(np.float32)*gain,0,255).astype(np.uint8)
            labels,family,margins=model.classify(backend.lab_values(image),cv2.cvtColor(image,cv2.COLOR_BGR2HSV))
            self.assertEqual(labels.tolist(),[[3,6,6,0,0]])
            self.assertFalse(family[0,3]);self.assertFalse(family[0,4])
        # A physically intermediate chroma gets no forced blue/light-blue vote.
        middle=model.centers[3]+model.centers[6];middle=middle/np.linalg.norm(middle)*30
        lab=np.float32([[[45,*middle]]]);hsv=np.uint8([[[110,150,150]]])
        self.assertEqual(int(model.classify(lab,hsv)[0][0,0]),0)
        overlap=BluePairModel({'3':[[30,10,-30]],'6':[[65,11,-31]]})
        self.assertEqual(overlap.state,'CHROMA_OVERLAP')
        self.assertFalse(np.any(overlap.classify(lab,hsv)[0]))

    def test_latest_blue_photo_finds_faces_instead_of_body_shadow(self):
        path=ROOT/'vision/pc_vision/test_data'
        raw=cv2.imdecode(np.fromfile(str(path/'blue_lightblue_raw.png'),np.uint8),cv2.IMREAD_COLOR)
        samples=json.loads((path/'blue_lightblue_calibration.json').read_text())['samples']
        image=cv2.resize(raw,(320,240),interpolation=cv2.INTER_AREA)
        analyzer=SurfaceAnalyzer(backend.material)
        for i in range(4):scene=analyzer.process(image,samples,3,100+i/30,False,raw)
        self.assertEqual([r['color'] for r in scene['color_regions']],[3,6])
        self.assertIsNotNone(scene['target'])
        self.assertEqual(scene['target']['color'],3)
        # Approximate visible face centers were marked by viewing the raw photo;
        # these are regression bounds, not a calibrated mechanical error metric.
        self.assertLess(np.linalg.norm(np.array(scene['target']['center'])-[70,121]),4)
        light=next(item for item in scene['surfaces'] if item['color']==6)
        self.assertTrue(light['confirmed'])
        self.assertLess(np.linalg.norm(np.array(light['center'])-[209,117]),4)
        self.assertGreater(scene['target']['uniformity'],.8)

    def test_color_region_report_is_independent_of_top_confirmation(self):
        image=np.full((240,320,3),180,np.uint8)
        cv2.rectangle(image,(100,90),(200,130),(101,32,13),-1)
        samples={'3':[[48,14,-33]],'6':[[50,-9,-27]]}
        scene=SurfaceAnalyzer(backend.material).process(image,samples,3,100,False)
        self.assertIsNone(scene['target'])
        report=SurfaceAnalyzer.describe(scene,3)
        self.assertEqual(report['colors'][0]['color'],3)
        gui=load('tested_color_gui','vision/pc_vision/gui.py')
        self.assertIn('蓝色×1',gui.color_text(report))
        # A light-blue candidate must not be reported as the selected blue center.
        other=dict(scene,target=None,surfaces=[{'color':6,'quality':.9}])
        self.assertIsNone(SurfaceAnalyzer.describe(other,3)['center'])

    def test_actual_black_and_cyan_faces_stay_on_tops_not_body_centers(self):
        data=ROOT/'vision/pc_vision/test_data'
        cases=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))
        for case in cases['cases']:
            with self.subTest(image=case['image']):
                raw=cv2.imdecode(np.fromfile(str(data/case['image']),np.uint8),1)
                analyzer=SurfaceAnalyzer(backend.material)
                # Small frame-to-frame translations exercise changing pixel data.
                for index in range(4):
                    dx,dy=index*.5,index*.25
                    moved=cv2.warpAffine(raw,np.float32([[1,0,dx],[0,1,dy]]),(480,360),borderMode=cv2.BORDER_REPLICATE)
                    image=cv2.resize(moved,(320,240),interpolation=cv2.INTER_AREA)
                    scene=analyzer.process(image,cases['samples'],case['color'],100+index/30,False,moved)
                target=scene['target']
                self.assertIsNotNone(target)
                center=np.array(target['center'])*1.5
                expected=np.array(case['approximate_top_raw_px'])+[dx,dy]
                self.assertLess(np.linalg.norm(center-expected),4)
                self.assertLess(max(target['axes'])*1.5,70)
                if target.get('coverage') is not None:
                    self.assertGreaterEqual(target['coverage'],.82)
                    self.assertLess(target['gap_deg'],61)

    def test_uniform_capsule_and_partial_arc_are_not_confirmed_as_a_top(self):
        samples={'5':[[6,-2,2],[24,0,-4],[6,-2,2]]}
        raw=np.full((360,480,3),190,np.uint8)
        # Uniform silhouette has no measured internal top rim. Its external
        # capsule can fit an ellipse well enough to fool the previous rim path.
        cv2.rectangle(raw,(160,150),(220,210),(12,12,12),-1)
        cv2.circle(raw,(160,180),30,(12,12,12),-1)
        cv2.circle(raw,(220,180),30,(12,12,12),-1)
        analyzer=SurfaceAnalyzer(backend.material)
        for i in range(4):
            image=cv2.resize(raw,(320,240),interpolation=cv2.INTER_AREA)
            scene=analyzer.process(image,samples,5,100+i/30,False,raw)
        self.assertIsNone(scene['target'])
        clipped=np.full_like(raw,190);cv2.circle(clipped,(0,180),30,(12,12,12),-1)
        scene=analyzer.process(cv2.resize(clipped,(320,240)),samples,5,101,False,clipped)
        self.assertIsNone(scene['target'])

    def test_dark_or_compressed_faces_never_fall_back_to_whole_body_centers(self):
        data=ROOT/'vision/pc_vision/test_data'
        cases=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))
        for case in cases['cases']:
            raw=cv2.imdecode(np.fromfile(str(data/case['image']),np.uint8),1)
            for gain,quality in ((.8,85),(1.2,85),(1.,60)):
                with self.subTest(image=case['image'],gain=gain,quality=quality):
                    modified=np.clip(raw.astype(np.float32)*gain,0,255).astype(np.uint8)
                    modified=cv2.imdecode(cv2.imencode('.jpg',modified,[cv2.IMWRITE_JPEG_QUALITY,quality])[1],1)
                    image=cv2.resize(modified,(320,240),interpolation=cv2.INTER_AREA)
                    analyzer=SurfaceAnalyzer(backend.material)
                    for i in range(4):scene=analyzer.process(image,cases['samples'],case['color'],100+i/30,False,modified)
                    if scene['target'] is not None:
                        center=np.array(scene['target']['center'])*1.5
                        self.assertLess(np.linalg.norm(center-case['approximate_top_raw_px']),6)
                    blank=np.full_like(modified,190)
                    lost=analyzer.process(cv2.resize(blank,(320,240)),cases['samples'],case['color'],100.14,False,blank)
                    self.assertIsNone(lost['target'])
                    self.assertNotIn(case['color'],analyzer.tracks)

    def test_refined_top_backend_preserves_protocol_scale_and_prediction_input(self):
        data=ROOT/'vision/pc_vision/test_data'
        cases=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))
        case=cases['cases'][0]
        raw=cv2.imdecode(np.fromfile(str(data/case['image']),np.uint8),1)
        obj=BackendTests().make_backend(raw)
        obj.samples=cases['samples'];obj.use_surface_output=True
        obj.configure_motion({'enabled':True})
        obj.configure((2,1,6,20,0))  # wire6 = black; UI5 = black
        for _ in range(4):result=obj.observe(time.perf_counter())
        self.assertIsNotNone(result['coordinates'])
        expected=np.array(case['approximate_top_raw_px'])/1.5
        self.assertAlmostEqual(result['coordinates'][0],expected[0]-160,delta=3)
        self.assertAlmostEqual(result['coordinates'][1],120-expected[1],delta=3)
        self.assertEqual(result['id'],6)
        self.assertEqual(obj.surface_report['source'],'local_rim')
        self.assertLess(np.linalg.norm(np.array(obj.motion_report['point'])-expected),3)

    def test_occupied_joint_center_with_off_center_material_and_noise(self):
        gray=np.full((360,480),205,np.uint8)
        for fraction in (65/95,75/95,85/95,1.):cv2.circle(gray,(240,180),round(55*fraction),30,2,cv2.LINE_AA)
        bgr=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        cv2.circle(bgr,(224,166),31,(90,25,10),-1)
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        seed={'center':(242.,179.),'radius':55.,'sampling_radius':55.,'ellipse':((242.,179.),(110.,110.),0.),'levels':4,'score':.9,'partial':False,'occluded':True}
        centers=[]
        for seed_value in range(4):
            noise=np.random.default_rng(seed_value).normal(0,2,gray.shape)
            noisy=np.uint8(np.clip(gray.astype(float)+noise,0,255))
            measured,quality=refine_occupied(noisy,bgr,seed,backend.ring)
            self.assertIsNotNone(measured)
            centers.append(measured['center'])
            self.assertLess(np.linalg.norm(np.array(measured['center'])-[240,180]),.7)
        self.assertLess(float(np.std(centers,axis=0).max()),.3)
        measured,_=refine_occupied(np.full_like(gray,200),np.full_like(bgr,200),seed,backend.ring)
        self.assertIsNone(measured)


class MotionTests(unittest.TestCase):
    def test_default_window_moves_down_fifteen_more_pixels_without_resizing(self):
        predictor=MotionPredictor({'enabled':True})
        options=predictor.options
        self.assertEqual((options['grasp_x'],options['grasp_y']),(160.,115.))
        self.assertEqual((options['half_width'],options['half_height']),(40.,40.))
        canvas=predictor.annotate(np.zeros((240,320,3),np.uint8),{})
        self.assertEqual(tuple(canvas[75,120]),(200,90,230))
        self.assertEqual(tuple(canvas[155,200]),(200,90,230))
        self.assertFalse(canvas[60,120].any())
        # Explicit operator calibration keeps its configured position.
        self.assertEqual(motion_settings({'grasp_y':90.})['grasp_y'],90.)

    @staticmethod
    def trace(omega=.8,axes=(60,60),noise=.3,dt=.04,count=110):
        rng=np.random.default_rng(42)
        predictor=MotionPredictor({'enabled':True})
        for i in range(count):
            stamp=i*dt
            point=np.array([160.,120.])+np.array(axes)*[np.cos(omega*stamp),np.sin(omega*stamp)]+rng.normal(0,noise,2)
            report=predictor.update(point,stamp,1,0)
        return predictor,report,stamp

    def test_circle_and_ellipse_direction_future_error_and_validation(self):
        for omega,axes in ((.8,(60,60)),(-.8,(60,60)),(.8,(85,45))):
            predictor,report,stamp=self.trace(omega,axes)
            self.assertTrue(report['ready'])
            self.assertEqual(report['direction'],'clockwise' if omega>0 else 'counterclockwise')
            if axes[0]!=axes[1]:self.assertEqual(report['model'],'ellipse')
            future_time=stamp+(predictor.options['lead_ms']+predictor.options['latency_ms'])/1000
            truth=np.array([160,120])+np.array(axes)*[np.cos(omega*future_time),np.sin(omega*future_time)]
            self.assertLess(np.linalg.norm(truth-report['future']),2)
            self.assertGreaterEqual(report['validation_count'],5)
            self.assertLess(report['error_p95_px'],3)

    def test_stop_reverse_missing_target_and_new_identity(self):
        predictor,_,stamp=self.trace(noise=0)
        fixed=np.array([160,120])+60*np.array([np.cos(.8*stamp),np.sin(.8*stamp)])
        for i in range(1,15):report=predictor.update(fixed,stamp+i*.04,1,0)
        self.assertEqual(report['state'],'STOPPED');self.assertFalse(report['grasp_advice'])
        for i in range(15,45):
            phase=.8*stamp-.8*(i-14)*.04
            report=predictor.update(np.array([160,120])+60*np.array([np.cos(phase),np.sin(phase)]),stamp+i*.04,1,0)
        self.assertEqual(report['direction'],'counterclockwise')
        report=predictor.update(None,stamp+45*.04,None,0)
        self.assertIsNone(report['future']);self.assertFalse(report['grasp_advice'])
        report=predictor.update(fixed,stamp+46*.04,2,0)
        self.assertFalse(report['ready']);self.assertEqual(report['observations'],1)

    def test_straight_path_small_arc_and_camera_motion_not_circular(self):
        predictor=MotionPredictor({'enabled':True})
        for i in range(80):report=predictor.update((20+i*.8,100),i*.04,1,0)
        self.assertIsNone(report['future']);self.assertEqual(report['state'],'LEARNING_ORBIT')
        predictor,report,stamp=self.trace(count=8)
        self.assertIsNone(report['future'])
        report=predictor.update((150,100),stamp+.04,1,4)
        self.assertEqual(report['state'],'CAMERA_MOVING');self.assertFalse(report['ready'])
        background=BackgroundMotion()
        rng=np.random.default_rng(3);image=np.uint8(rng.integers(0,256,(240,320,3)))
        self.assertIsNone(background.update(image))
        translated=cv2.warpAffine(image,np.float32([[1,0,6],[0,1,3]]),(320,240))
        self.assertGreater(background.update(translated),2)

    def test_window_eta_and_lookahead_advice_are_observation_only(self):
        predictor,report,stamp=self.trace(noise=0)
        next_point=np.array([160,120])+60*np.array([np.cos(.8*(stamp+.3)),np.sin(.8*(stamp+.3))])
        predictor.options.update(grasp_x=float(next_point[0]),grasp_y=float(next_point[1]),half_width=3.,half_height=3.)
        stamp+=.04
        point=np.array([160,120])+60*np.array([np.cos(.8*stamp),np.sin(.8*stamp)])
        report=predictor.update(point,stamp,1,0)
        self.assertTrue(report['ready']);self.assertTrue(report['future_in_window']);self.assertTrue(report['grasp_advice'])
        self.assertGreater(report['eta_ms'],100);self.assertLess(report['eta_ms'],400)
        self.assertNotIn('command',report)
        self.assertEqual(predictor.annotate(np.zeros((240,320,3),np.uint8),report).shape,(240,320,3))

    def test_parameters_and_time_validation(self):
        for bad in ({'lead_ms':float('nan')},{'lead_ms':500,'latency_ms':300},{'grasp_x':400},{'enabled':'yes'}):
            with self.assertRaises(ValueError):motion_settings(bad)
        predictor=MotionPredictor({'enabled':True})
        predictor.update((100,100),1,1)
        report=predictor.update((101,100),1,1)
        self.assertEqual(report['state'],'STALE_OBSERVATION')

    def test_motion_does_not_change_measured_coordinate_result(self):
        image=np.full((360,480,3),180,np.uint8);cv2.circle(image,(290,140),30,(0,0,240),-1)
        case=BackendTests()
        disabled=case.make_backend(image);enabled=case.make_backend(image)
        enabled.configure_motion({'enabled':True})
        for obj in (disabled,enabled):obj.preview_only=True;obj.configure((2,1,1,20,0))
        for _ in range(5):
            expected=disabled.observe(time.perf_counter());actual=enabled.observe(time.perf_counter())
            self.assertEqual(actual,expected)
        self.assertIsNotNone(enabled.motion_report)


class LoopbackTests(unittest.TestCase):
    def test_headless_qr_entry_uses_cam2_contract(self):
        network=Loopback(qr_photo(),role='qr')
        try:
            network.state.accept_request(qr_service.encode_set(6,4,20,7),time.monotonic())
            with tempfile.TemporaryDirectory() as temp:
                args=SimpleNamespace(mode='run',headless=True,max_frames=5,config=Path(temp)/'config.cam2.json')
                config=dict(desktop.DEFAULT,host='127.0.0.1',video_port=network.video_port,
                            control_port=network.control_port,camera_role='qr')
                desktop.run_connected(config,args,Path(temp))
                packets=[]
                while not network.state.results.empty():
                    _,payload=network.state.results.get_nowait();packets.append(payload[wire.RESULT_META.size:])
                self.assertIn(qr_service.encode_task('421+132+142+321',7),packets)
        finally:network.close()

    def test_new_epoch_interrupts_old_wait_without_poisoning_stream(self):
        left,right=socket.socketpair()
        video=None
        try:
            with patch.object(transport,'connect',return_value=left):
                video=transport.VideoClient('unused',1,mode='push-v2')
            meta=(2,11,time.monotonic_ns(),480,360,10,20,30)
            right.sendall(wire.encode(wire.FRAME_V2,wire.FRAME_META_V2.pack(*meta)+b'jpeg'))
            start=time.perf_counter()
            with self.assertRaises(transport.StreamContextChanged):video.get(1)
            self.assertLess(time.perf_counter()-start,.4)
            self.assertIsNone(video.error)
            current,_,_=video.get(2)
            self.assertEqual(current[:2],(2,11))
        finally:
            if video:video.close()
            else:left.close()
            right.close()

    def test_pending_set_interrupts_before_new_video_arrives_and_old_cache_waits(self):
        left,right=socket.socketpair();pending=threading.Event();entered=threading.Event();video=None
        class Control:
            def has_new_context(self,epoch):
                entered.set()
                return pending.is_set()
        try:
            with patch.object(transport,'connect',return_value=left):
                video=transport.VideoClient('unused',1,mode='push-v2',control=Control())
            def interrupt():
                entered.wait(1);pending.set()
            helper=threading.Thread(target=interrupt);helper.start()
            start=time.perf_counter()
            with self.assertRaises(transport.StreamContextChanged):video.get(0)
            helper.join(1)
            self.assertLess(time.perf_counter()-start,.4)
            self.assertIsNone(video.error)
            pending.clear()
            # A cached older frame is not an interruption when awaiting epoch1.
            old=(0,10,time.monotonic_ns(),480,360,10,20,30)
            right.sendall(wire.encode(wire.FRAME_V2,wire.FRAME_META_V2.pack(*old)+b'old'))
            def deliver():
                time.sleep(.03)
                new=(1,11,time.monotonic_ns(),480,360,10,20,30)
                right.sendall(wire.encode(wire.FRAME_V2,wire.FRAME_META_V2.pack(*new)+b'new'))
            helper=threading.Thread(target=deliver);helper.start()
            current,jpeg,_=video.get(1);helper.join(1)
            self.assertEqual(current[:2],(1,11));self.assertEqual(jpeg,b'new')
            self.assertIsNone(video.error)
        finally:
            if video:video.close()
            else:left.close()
            right.close()

    def test_plate_to_material_set_during_fetch_acknowledges_without_reconnect(self):
        pixels=np.full((360,480,3),180,np.uint8)
        cv2.circle(pixels,(290,140),32,(0,0,240),-1)
        network=Loopback(pixels);control=video=None
        try:
            control=transport.ControlClient('127.0.0.1',network.control_port)
            video=transport.VideoClient('127.0.0.1',network.video_port,mode='push-v2',control=control)
            obj=backend.PcBackend(video,control,Path(tempfile.gettempdir())/'nonexistent_pcvision_fixture_samples.json',False)
            engine=service.VisionService(control,obj,clock=time.perf_counter)
            network.state.accept_request(service.encode_set(8,1,0,20,4),time.monotonic())
            packets=[]
            def receive():
                while not network.state.results.empty():
                    connection,payload=network.state.results.get_nowait()
                    packet=network.state.result_current(connection,payload,time.monotonic_ns())
                    if packet:packets.extend(packet[i:i+8] for i in range(0,len(packet),8))
            deadline=time.perf_counter()+2
            while time.perf_counter()<deadline:
                engine.tick();receive()
                if engine.config[0]==8 and obj.frame_count>=3:break
            self.assertEqual(engine.config[0],8)
            old_epoch=control.epoch;original=video.get;switched=[False]
            def request_while_fetching(epoch):
                if not switched[0] and epoch==old_epoch:
                    switched[0]=True
                    network.state.accept_request(service.encode_set(2,1,1,20,5),time.monotonic())
                    deadline=time.perf_counter()+.3
                    while not control.has_new_context(epoch) and time.perf_counter()<deadline:time.sleep(.001)
                    self.assertTrue(control.has_new_context(epoch))
                return original(epoch)
            with patch.object(video,'get',side_effect=request_while_fetching):
                start=time.perf_counter();deadline=start+.5
                while time.perf_counter()<deadline:
                    engine.tick();receive()
                    if any(p[1]==0x81 and (p[2]>>3)&15==5 and p[2]&128 and p[5]==2 for p in packets):break
                elapsed=time.perf_counter()-start
            self.assertTrue(switched[0]);self.assertEqual(engine.config[0],2)
            self.assertTrue(any(p[1]==0x81 and (p[2]>>3)&15==5 and p[2]&128 and p[5]==2 for p in packets))
            self.assertLess(elapsed,.5)
            self.assertIsNone(video.error)
            self.assertEqual(network.state.connection,1)
            deadline=time.perf_counter()+1
            while time.perf_counter()<deadline and not any(p[1]==2 and p[2]&128 for p in packets):
                engine.tick();receive()
            self.assertTrue(any(p[1]==2 and p[2]&128 for p in packets))
        finally:
            if video:video.close()
            if control:control.close()
            network.close()

    def test_continuous_stream_skips_old_frames_and_changes_epoch(self):
        network=Loopback();control=video=None
        try:
            control=transport.ControlClient('127.0.0.1',network.control_port)
            self.assertEqual(control.hello['video_mode'],'push-v2')
            video=transport.VideoClient('127.0.0.1',network.video_port,mode='push-v2')
            first,jpeg,elapsed=video.get(control.epoch)
            self.assertEqual(len(first),8)
            self.assertGreater(first[6],0)
            time.sleep(.12)  # slow UI consumer must not accumulate a FIFO of JPEGs
            second,_,_=video.get(control.epoch)
            self.assertGreater(second[1],first[1]+1)
            self.assertGreater(video.skipped,0)
            self.assertLess(video.last_receive_age_ms,100)
            network.state.accept_request(service.encode_set(3,2,2,20,7,1),time.monotonic())
            changed,_,_=video.get(network.state.epoch)
            self.assertEqual(changed[0],network.state.epoch)
            self.assertGreater(changed[1],second[1])
        finally:
            if video:video.close()
            if control:control.close()
            network.close()

    def test_connections_can_be_reestablished(self):
        network=Loopback()
        try:
            last_id=0
            for _ in range(2):
                control=transport.ControlClient('127.0.0.1',network.control_port)
                video=transport.VideoClient('127.0.0.1',network.video_port)
                try:
                    meta,jpeg,elapsed=video.get(control.epoch)
                    self.assertGreater(meta[1],last_id)
                    self.assertIsNotNone(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR))
                    last_id=meta[1]
                finally:video.close();control.close()
        finally:network.close()

    def test_preview_receives_and_decodes_no_uart_results(self):
        network=Loopback()
        try:
            with tempfile.TemporaryDirectory() as temp:
                args=SimpleNamespace(mode='preview',headless=True,max_frames=4,config=Path(temp)/'config.json')
                config=dict(desktop.DEFAULT,host='127.0.0.1',video_port=network.video_port,control_port=network.control_port)
                desktop.run_connected(config,args,Path(temp))
                with next(Path(temp).glob('frames_*.csv')).open(encoding='utf-8') as stream:
                    rows=list(csv.reader(stream))
                self.assertEqual(len(rows),5)
                self.assertTrue(network.state.results.empty())
        finally:network.close()

    def test_set_ack_and_real_ring_observation_over_tcp(self):
        network=Loopback();control=video=None
        try:
            control=transport.ControlClient('127.0.0.1',network.control_port)
            video=transport.VideoClient('127.0.0.1',network.video_port,mode=control.hello['video_mode'])
            obj=backend.PcBackend(video,control,Path(tempfile.gettempdir())/'nonexistent_pcvision_fixture_samples.json',False)
            engine=service.VisionService(control,obj)
            service.PREVIEW_ENABLED=False
            network.state.accept_request(service.encode_set(3,2,2,20,4,1),time.monotonic())
            deadline=time.monotonic()+2
            packets=[]
            while time.monotonic()<deadline:
                engine.tick();time.sleep(.012)
                while not network.state.results.empty():
                    connection,payload=network.state.results.get_nowait()
                    accepted=network.state.result_current(connection,payload,time.monotonic_ns())
                    if accepted:packets.extend(accepted[i:i+8] for i in range(0,len(accepted),8))
                if any(p[1]==3 and p[2]&128 for p in packets):break
            self.assertTrue(any(p[1]==0x81 and p[2]&128 for p in packets))
            self.assertTrue(any(p[1]==3 and p[2]&128 for p in packets))
            self.assertTrue(any(p[1]==0x83 for p in packets))
        finally:
            if video:video.close()
            if control:control.close()
            network.close()


class GuiTests(unittest.TestCase):
    def tearDown(self):
        # Tk variables in widget/callback cycles must be finalized on the GUI
        # thread, before the next TCP camera fixture starts allocating objects.
        gc.collect()

    def test_serial_tab_and_camera_screenshot_share_session_and_nearby_lines(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw();dual=None
        with tempfile.TemporaryDirectory() as temp,patch.object(gui,'HERE',Path(temp)):
            try:
                settings=Path(temp)/'serial_config.json';settings.write_text(json.dumps({'port':'COM10','baud':115200}))
                dual=gui.DualApp(root,Path(temp)/'config.json',Path(temp)/'config.cam2.json',autoconnect=False)
                self.assertIsNone(dual.serial_log.thread)
                self.assertEqual(dual.serial_panel.port.get(),'COM10')
                self.assertEqual(dual.serial_panel.baud.get(),'115200')
                class Writer(SerialMonitorTests.Device):
                    def __init__(self):super().__init__([b'[MATCH] MOVING TEMP_BATCH1\r\n[CAM1] request RING station=3\r\n']);self.sent=[]
                    def write(self,data):self.sent.append(bytes(data));return len(data)
                device=Writer()
                dual.serial_log.serial_factory=lambda:device
                dual.serial_panel.connect_button.invoke()
                deadline=time.perf_counter()+1
                while time.perf_counter()<deadline:
                    root.update()
                    if 'RING station=3' in dual.serial_panel.text.get('1.0','end'):break
                    time.sleep(.01)
                self.assertIn('RING station=3',dual.serial_panel.text.get('1.0','end'))
                dual.serial_panel.command.set('1号区发车 / 继续')
                dual.serial_panel.command_combo.event_generate('<<ComboboxSelected>>')
                root.update();self.assertEqual(device.sent,[])
                dual.serial_panel.send_button.invoke()
                deadline=time.perf_counter()+1
                while not device.sent and time.perf_counter()<deadline:root.update();time.sleep(.01)
                self.assertEqual(device.sent,[b'&OK 1\r\n'])
                panel=dual.panels[0];image=np.full((360,480,3),190,np.uint8)
                frame=timestamp()
                panel.latest={'frame':42,'image':image.copy(),'raw_image':image,'config':(3,3,2,20,3),
                              'pc_frame_perf_s':frame['perf_s'],'pc_frame_wall_time':frame['wall_time']}
                panel.save_screenshot()
                folder=Path(temp)/'screenshots'
                data=json.loads(next(folder.glob('*.json')).read_text(encoding='utf-8'))
                self.assertEqual(data['pc_frame_wall_time'],frame['wall_time'])
                self.assertGreaterEqual(data['pc_saved_perf_s'],data['pc_frame_perf_s'])
                self.assertIn('TEMP_BATCH1',data['serial']['last_match_before_frame']['text'])
                self.assertEqual(data['serial']['port'],'COM10')
                self.assertEqual(data['serial']['last_tx_before_frame']['text'],'&OK 1')
                self.assertTrue(next(folder.glob('*_serial.txt')).exists())
                self.assertIn('SNAPSHOT',Path(data['serial']['log_path']).read_text(encoding='utf-8'))
                self.assertEqual(len(dual.notebook.tabs()),5)  # CAM1/CAM2/串口/机械臂/转向PID
            finally:
                if dual:
                    dual.close()
                    if dual.serial_log.thread:dual.serial_log.thread.join(1)
                else:root.destroy()

    def test_camera_dropdown_filters_role_and_does_not_retarget_active_worker(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                path=Path(temp)/'config.json';path.write_text(json.dumps({'host':'192.168.31.44'}))
                app=gui.App(root,path,autoconnect=False)
                material={'host':'192.168.31.50','role':'material','name':'cam1','control_port':9101,'video_port':9100}
                qr=dict(material,host='192.168.31.51',role='qr',name='cam2')
                app.apply_discovered_devices([material,qr])
                self.assertEqual(app.host.get(),material['host'])
                self.assertIn(material['host'],app.host_entry['values'])
                self.assertIn('192.168.31.44',app.host_entry['values'])
                self.assertNotIn(qr['host'],app.host_entry['values'])
                self.assertEqual(app.control_port.get(),'9101')
                app.worker=SimpleNamespace()
                app.apply_discovered_devices([dict(material,host='192.168.31.52',control_port=9201)])
                self.assertEqual(app.host.get(),material['host'])
                self.assertEqual(app.control_port.get(),'9101')
                app.worker=None;root.update_idletasks()
        finally:root.destroy()

    def test_background_scan_autoconnects_single_camera_without_typing(self):
        import tkinter as tk
        import gui
        release=threading.Event();entered=threading.Event();created=[]
        device={'host':'192.168.31.50','role':'material','name':'cam1','control_port':9101,'video_port':9100}
        def scan(*args):entered.set();release.wait(1);return [device]
        def factory(config,path,mode):
            worker=SimpleNamespace(config=config,thread=None,stop_event=threading.Event(),
                                   command=lambda *args:None,start=lambda:None,snapshot=lambda:(None,[]))
            created.append(worker);return worker
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp,patch.object(gui,'scan_devices',side_effect=scan):
                path=Path(temp)/'config.json'
                app=gui.App(root,path,autoconnect=False,worker_factory=factory)
                start=time.perf_counter();app.scan_cameras(autoconnect=True)
                self.assertLess(time.perf_counter()-start,.15)
                self.assertTrue(entered.wait(.3))
                root.update_idletasks();self.assertEqual(created,[])
                release.set();deadline=time.perf_counter()+1
                while not created and time.perf_counter()<deadline:
                    root.update();time.sleep(.01)
                self.assertEqual(len(created),1)
                self.assertEqual(created[0].config['host'],device['host'])
                self.assertEqual(created[0].config['control_port'],9101)
                self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['recent_hosts'],[device['host']])
                app.worker=None;root.update_idletasks()
        finally:
            release.set();root.destroy()

    def test_dual_panels_keep_configuration_commands_and_screenshots_separate(self):
        import tkinter as tk
        import gui
        class Worker:
            def __init__(self,config,path,mode):
                self.config=config;self.mode=mode;self.commands=[];self.thread=None;self.stop_event=threading.Event()
            def start(self):pass
            def stop(self):self.stop_event.set()
            def command(self,*args):self.commands.append(args)
            def snapshot(self):return None,[]
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp,patch.object(gui,'HERE',Path(temp)):
                dual=gui.DualApp(root,Path(temp)/'config.json',Path(temp)/'config.cam2.json',autoconnect=False,worker_factory=Worker)
                first,second=dual.panels
                first.host.set('127.0.0.1');second.host.set('127.0.0.2')
                first.connect();second.connect()
                self.assertEqual(first.worker.config['camera_role'],'material')
                self.assertEqual(second.worker.config['camera_role'],'qr')
                self.assertEqual(set(second.mode_buttons),{'原始画面','二维码识别'})
                first.worker.commands.clear();second.worker.commands.clear()
                second.select_mode('二维码识别')
                self.assertEqual(first.worker.commands,[])
                self.assertEqual(second.worker.commands,[('select',('二维码识别',0))])
                second.latest={'image':qr_photo(),'raw_image':qr_photo(),'frame':42,'camera_role':'qr',
                               'qr_report':{'task':'421+132+142+321','rotation':180}}
                second.screenshot_button.invoke()
                metadata=json.loads(next((Path(temp)/'screenshots/cam2').glob('*.json')).read_text(encoding='utf-8'))
                self.assertEqual(metadata['camera_role'],'qr')
                self.assertEqual(metadata['qr_report']['rotation'],180)
                self.assertEqual(len(dual.notebook.tabs()),5)  # V234 新增“转向PID”页
                first.worker.stop();second.worker.stop()
                root.update_idletasks()
        finally:root.destroy()

    def test_dual_real_streams_route_qr_and_pixels_and_stop_independently(self):
        import gui
        pixels=np.full((360,480,3),180,np.uint8);cv2.circle(pixels,(290,140),32,(0,0,240),-1)
        material_net=Loopback(pixels);qr_net=Loopback(qr_photo(),role='qr')
        tempdir=tempfile.TemporaryDirectory();workers=[]
        try:
            with patch.object(gui,'HERE',Path(tempdir.name)):
                configs=[]
                for role,network in (('material',material_net),('qr',qr_net)):
                    config=dict(desktop.DEFAULT,host='127.0.0.1',video_port=network.video_port,control_port=network.control_port,camera_role=role)
                    path=Path(tempdir.name)/('config.cam2.json' if role=='qr' else 'config.json')
                    configs.append(config);worker=gui.VisionWorker(config,path,'run');workers.append(worker)
                material_net.state.accept_request(service.encode_set(2,1,1,20,4),time.monotonic())
                qr_net.state.accept_request(qr_service.encode_set(6,4,20,5),time.monotonic())
                for worker in workers:worker.start()
                accepted=[[],[]];snapshots=[None,None]
                def collect():
                    for index,network in enumerate((material_net,qr_net)):
                        while not network.state.results.empty():
                            connection,payload=network.state.results.get_nowait()
                            packet=network.state.result_current(connection,payload,time.monotonic_ns())
                            if packet:accepted[index].append(packet)
                        snapshots[index],_=workers[index].snapshot()
                deadline=time.perf_counter()+5
                while time.perf_counter()<deadline:
                    collect()
                    if any(p[1]==2 and p[2]&128 for p in accepted[0]) and any(p[1]==7 for p in accepted[1]):break
                    time.sleep(.005)
                qr_packet=next(p for p in accepted[1] if p[1]==7)
                self.assertEqual(qr_packet,qr_service.encode_task('421+132+142+321',5))
                self.assertFalse(any(p[1]==7 for p in accepted[0]))
                self.assertFalse(any(p[1]==2 for p in accepted[1]))
                self.assertEqual(snapshots[1]['raw_image'].shape,(480,640,3))
                self.assertEqual(snapshots[1]['qr_report']['task'],'421+132+142+321')
                self.assertGreater(snapshots[1]['return_report']['groups'],0)
                qr_net.state.accept_request(qr_service.encode_set(0,4,0,6),time.monotonic())
                deadline=time.perf_counter()+2
                while time.perf_counter()<deadline:
                    collect()
                    if snapshots[1] and snapshots[1]['config'][0]==0:break
                    time.sleep(.005)
                self.assertEqual(snapshots[1]['config'][0],0)
                self.assertEqual(snapshots[1]['qr_report']['task'],'421+132+142+321')
                old_count=snapshots[0]['decode_ok']
                workers[1].stop();workers[1].thread.join(3)
                self.assertFalse(workers[1].thread.is_alive())
                deadline=time.perf_counter()+1
                while time.perf_counter()<deadline:
                    collect()
                    if snapshots[0] and snapshots[0]['decode_ok']>=old_count+3:break
                    time.sleep(.005)
                self.assertGreaterEqual(snapshots[0]['decode_ok'],old_count+3)
                self.assertEqual(snapshots[0]['config'][0],2)
        finally:
            for worker in workers:
                worker.stop()
                if worker.thread:worker.thread.join(3)
            material_net.close();qr_net.close();tempdir.cleanup()

    def test_run_button_waits_for_preview_worker_before_connecting(self):
        import tkinter as tk
        import gui
        created=[]
        class Worker:
            def __init__(self,config,path,mode):
                self.mode=mode;self.thread=None;self.stop_event=threading.Event();created.append(self)
            def start(self):pass
            def stop(self):self.stop_event.set()
            def command(self,*args):pass
            def snapshot(self):return None,[]
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False,worker_factory=Worker)
                app.host.set('127.0.0.1');app.connect()
                previous=app.worker;alive=[True]
                previous.thread=SimpleNamespace(is_alive=lambda:alive[0])
                app.run_button.invoke()
                self.assertTrue(previous.stop_event.is_set())
                self.assertEqual(len(created),1)
                root.update();time.sleep(.06);root.update()
                self.assertEqual(len(created),1)
                alive[0]=False
                deadline=time.perf_counter()+1
                while len(created)<2 and time.perf_counter()<deadline:
                    root.update();time.sleep(.01)
                self.assertEqual(len(created),2)
                self.assertEqual(app.worker.mode,'run')
                self.assertEqual(app.operation.get(),'主控联动')
                self.assertEqual(str(app.mode_buttons['物料识别']['state']),'disabled')
                self.assertEqual(str(app.screenshot_button['state']),'normal')
                self.assertFalse(app._disconnecting)
        finally:
            root.update_idletasks();root.destroy()

    def test_linked_gui_returns_wire_coordinates_while_displaying_and_saving(self):
        import tkinter as tk
        import gui
        pixels=np.full((360,480,3),180,np.uint8)
        cv2.circle(pixels,(290,140),32,(0,0,240),-1)
        network=Loopback(pixels);root=tk.Tk();root.withdraw();app=worker=None
        tempdir=tempfile.TemporaryDirectory()
        try:
            temp=tempdir.name
            with patch.object(gui,'HERE',Path(temp)):
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                app.host.set('127.0.0.1');app.video_port.set(str(network.video_port));app.control_port.set(str(network.control_port))
                network.state.accept_request(service.encode_set(2,1,1,20,6),time.monotonic())
                app.run_button.invoke();worker=app.worker
                accepted=[]
                def receive():
                    while not network.state.results.empty():
                        connection,payload=network.state.results.get_nowait()
                        packet=network.state.result_current(connection,payload,time.monotonic_ns())
                        if packet:accepted.append(packet)
                deadline=time.perf_counter()+4
                while time.perf_counter()<deadline:
                    root.update();receive()
                    if app.latest and app.photo is not None:
                        report=app.latest['return_report'];last=report['last']
                        if last and last['valid'] and any(p[1]==2 and p[2]&0x80 for p in accepted):break
                    time.sleep(.01)
                self.assertIsNotNone(app.latest)
                self.assertIsNotNone(app.photo)
                self.assertEqual(app.latest['operation'],'run')
                self.assertEqual(app.latest['image'].shape,(240,320,3))
                self.assertEqual(app.latest['raw_image'].shape,(360,480,3))
                report=app.latest['return_report'];self.assertGreater(report['groups'],0)
                packet=next(p for p in accepted if p[1]==2 and p[2]&0x80)
                self.assertEqual(packet[-1],service.crc8(packet[:7]))
                x,y=struct.unpack_from('<hh',packet,3)
                self.assertAlmostEqual(x/10,290*320/480-160,delta=1)
                self.assertAlmostEqual(y/10,140*240/360-120,delta=1)
                self.assertAlmostEqual(report['last']['dx_px'],x/10,delta=1)
                self.assertAlmostEqual(report['last']['dy_px'],y/10,delta=1)
                self.assertIn('坐标已发往相机桥',app.return_status.get())
                app.screenshot_button.invoke()
                metadata=json.loads(next((Path(temp)/'screenshots').glob('*.json')).read_text(encoding='utf-8'))
                self.assertEqual(metadata['operation'],'run')
                self.assertTrue(metadata['return_report']['last']['valid'])
                self.assertEqual(len(list((Path(temp)/'screenshots').glob('*.png'))),2)
                # A new STOP must not keep displaying the preceding task's coordinates.
                network.state.accept_request(service.encode_set(0,0,0,0,7),time.monotonic())
                deadline=time.perf_counter()+2
                while time.perf_counter()<deadline:
                    root.update();receive()
                    if app.latest and app.latest['config'][0]==0:break
                    time.sleep(.01)
                self.assertEqual(app.latest['config'][0],0)
                self.assertIsNone(app.latest['return_report']['last'])
                self.assertIn('等待STM32',app.return_status.get())
                self.assertIsNotNone(app.photo)
        finally:
            if app:app.close()
            else:root.destroy()
            if worker:worker.stop();worker.thread.join(3)
            network.close()
            tempdir.cleanup()

    def test_return_status_distinguishes_invalid_preview_and_pending_results(self):
        import gui
        snapshot={'operation':'preview','config':(2,1,1,20,0)}
        self.assertIn('未启用',gui.return_text(snapshot))
        snapshot['operation']='run'
        self.assertIn('等待本任务',gui.return_text(snapshot))
        snapshot['return_report']={'groups':3,'age_ms':10,'last':{'valid':False}}
        self.assertIn('无效观测',gui.return_text(snapshot))
        snapshot['return_report']['age_ms']=600
        self.assertIn('最近发送距今',gui.return_text(snapshot))
        snapshot['calibration']=True
        self.assertIn('不发送抓取坐标',gui.return_text(snapshot))

    def test_motion_dialog_persists_options_and_clicks_without_actuator_commands(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                original=tk.Toplevel
                def hidden(*args,**kwargs):
                    dialog=original(*args,**kwargs);dialog.withdraw();return dialog
                with patch.object(gui.tk,'Toplevel',side_effect=hidden):app.open_motion()
                root.update_idletasks()
                commands=[]
                app.worker=SimpleNamespace(command=lambda *args:commands.append(args),config={})
                app.motion_vars['enabled'].set(True)
                app.motion_vars['lead_ms'].set('240')
                app.latest={'config':(2,1,1,20,0),'calibration':False}
                app.image_geometry=(10,20,2,320,240)
                app.select_grasp();app.click_image(SimpleNamespace(x=210,y=200))
                self.assertFalse(app.selecting_grasp)
                app.apply_motion()
                values=gui.read_config(app.config_path)['motion']
                self.assertEqual(values['grasp_x'],100);self.assertEqual(values['grasp_y'],90)
                self.assertEqual(values['lead_ms'],240);self.assertTrue(values['enabled'])
                self.assertEqual([item[0] for item in commands],['surface_enabled','motion_config'])
                self.assertFalse(app.use_surface_output.get())
                app.close_motion()
                root.update_idletasks()
        finally:root.destroy()

    def test_repeated_frames_do_not_reconfigure_menu_state(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                with patch.object(app.options_menu,'entryconfigure',wraps=app.options_menu.entryconfigure) as update:
                    app.set_controls(True,False)
                    for _ in range(20):app.set_controls(True,False)
                    self.assertEqual(update.call_count,1)
                    app.set_controls(True,True)
                    self.assertEqual(update.call_count,2)
        finally:root.destroy()

    def test_popup_pauses_ui_refresh_and_resumes_without_stopping_worker(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                snapshot=unittest.mock.Mock(return_value=(None,[]))
                stop_event=threading.Event()
                app.worker=SimpleNamespace(snapshot=snapshot,stop_event=stop_event)
                with patch.object(app.options_menu,'winfo_ismapped',return_value=True):
                    app._menu_posted();root.update_idletasks()
                    app.refresh()
                    snapshot.assert_not_called()
                    self.assertFalse(stop_event.is_set())
                    app._menu_closed()
                    app.refresh()
                    snapshot.assert_called_once()
        finally:root.destroy()

    def test_video_updates_reuse_photo_and_canvas_item(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                with patch.object(app.canvas,'winfo_width',return_value=80),patch.object(app.canvas,'winfo_height',return_value=60):
                    app.draw_image(np.zeros((60,80,3),np.uint8))
                    item,image_name=app._canvas_image,str(app.photo)
                    app.draw_image(np.full((60,80,3),255,np.uint8))
                    self.assertEqual(app._canvas_image,item)
                    self.assertEqual(str(app.photo),image_name)
                    self.assertEqual(app.photo.get(0,0),(255,255,255))
                    self.assertEqual(app.canvas.find_all(),(item,))
                with patch.object(app.canvas,'winfo_width',return_value=40),patch.object(app.canvas,'winfo_height',return_value=30):
                    app.draw_image(np.zeros((60,80,3),np.uint8))
                    self.assertEqual((app.photo.width(),app.photo.height()),(40,30))
                    self.assertEqual(app._canvas_image,item)
        finally:root.destroy()

    def test_preview_buttons_and_linked_mode_authority(self):
        import tkinter as tk
        import gui
        class FakeWorker:
            def __init__(self,*args):self.commands=[];self.thread=None;self.stop_event=threading.Event()
            def start(self):pass
            def stop(self):self.stop_event.set()
            def command(self,*args):self.commands.append(args)
            def snapshot(self):return None,[]
        for mode in ('preview','run'):
            root=tk.Tk();root.withdraw()
            try:
                with tempfile.TemporaryDirectory() as temp:
                    app=gui.App(root,Path(temp)/'config.json',mode,False,FakeWorker)
                    app.host.set('127.0.0.1');app.connect()
                    app.worker.commands.clear()
                    app.select_mode('空圆环')
                    if mode=='preview':
                        self.assertEqual(app.worker.commands[-1],('select',('空圆环',0)))
                        self.assertEqual(str(app.mode_buttons['空圆环']['state']),'normal')
                    else:
                        self.assertEqual(app.worker.commands,[])
                        self.assertEqual(str(app.mode_buttons['空圆环']['state']),'disabled')
            finally:root.destroy()

    def test_ppm_image_and_calibration_click_coordinates(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                app.draw_image(np.zeros((360,480,3),np.uint8))
                self.assertIsNotNone(app.photo)
                commands=[]
                app.worker=SimpleNamespace(command=lambda *args:commands.append(args))
                app.latest={'calibration':True}
                app.image_geometry=(10,20,2,320,240)
                app.click_image(SimpleNamespace(x=110,y=140))
                self.assertEqual(commands,[('sample',(50,60))])
        finally:root.destroy()

    def test_screenshot_preserves_raw_pixels_separately(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp,patch.object(gui,'HERE',Path(temp)):
                app=gui.App(root,Path(temp)/'config.json',autoconnect=False)
                raw=np.zeros((360,480,3),np.uint8)
                overlay=raw.copy();cv2.circle(overlay,(100,100),30,(0,255,0),2)
                app.latest={'image':overlay,'raw_image':raw,'frame':42}
                app.save_screenshot()
                raw_file=next((Path(temp)/'screenshots').glob('*_raw.png'))
                overlay_file=next((Path(temp)/'screenshots').glob('*_overlay.png'))
                recovered=cv2.imdecode(np.fromfile(str(raw_file),np.uint8),1)
                self.assertTrue(np.array_equal(recovered,raw))
                self.assertFalse(np.array_equal(cv2.imdecode(np.fromfile(str(overlay_file),np.uint8),1),raw))
                self.assertTrue(list((Path(temp)/'screenshots').glob('*.json')))
        finally:root.destroy()

    def test_gui_worker_real_stream_preview_and_button_switch(self):
        import gui
        network=Loopback()
        worker=None
        try:
            with tempfile.TemporaryDirectory() as temp,patch.object(gui,'HERE',Path(temp)):
                config=dict(desktop.DEFAULT,host='127.0.0.1',video_port=network.video_port,control_port=network.control_port)
                worker=gui.VisionWorker(config,Path(temp)/'config.json','preview')
                worker.start()
                worker.command('select',('空圆环',0))
                deadline=time.perf_counter()+3
                snap=None
                while time.perf_counter()<deadline:
                    snap,_=worker.snapshot()
                    if snap and snap['config'][0]==3 and snap['decode_ok']>=5:break
                    time.sleep(.01)
                self.assertIsNotNone(snap)
                self.assertEqual(snap['config'][0],3)
                self.assertGreaterEqual(snap['decode_ok'],5)
                self.assertTrue(network.state.results.empty())
                worker.stop();worker.thread.join(3)
                self.assertFalse(worker.thread.is_alive())
        finally:
            if worker:worker.stop();worker.thread.join(3)
            network.close()


if __name__=='__main__':unittest.main(verbosity=2)
