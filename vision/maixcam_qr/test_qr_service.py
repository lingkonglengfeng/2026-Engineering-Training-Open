"""真实QR图像离线解码+模拟Maix/UART；不代替CAM2实机扫码验证。"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace,ModuleType
import unittest
from unittest.mock import patch

import cv2
import numpy as np

SOURCE=Path(__file__).parent/'service.py'
spec=importlib.util.spec_from_file_location('qr_service_impl',SOURCE)
qr=importlib.util.module_from_spec(spec);spec.loader.exec_module(qr)
TASK='156+123+516+231'
OTHER='123+321+321+123'


class Serial:
    def __init__(self):self.rx=bytearray();self.output=bytearray();self.writes=[];self.closed=False
    def read(self,n,timeout=0):
        assert timeout==0
        data=bytes(self.rx[:n]);del self.rx[:n];return data
    def write(self,data):
        count=self.writes.pop(0) if self.writes else len(data)
        if isinstance(count,Exception):raise count
        if count>0:self.output.extend(data[:count])
        return count
    def available(self,timeout=0):return 0
    def close(self):self.closed=True


class Clock:
    def __init__(self):self.value=0.
    def __call__(self):return self.value


def frames(data):return qr.FrameParser().feed(data,0)


class Backend:
    def __init__(self):self.config=None;self.configs=[];self.tracker=qr.TaskTracker();self.payloads=[TASK];self.calls=0;self.callback=None
    def configure(self,config):self.config=config;self.configs.append(config);self.tracker.reset()
    def observe(self,now,scan=True):
        self.calls+=1
        if self.callback:self.callback()
        return {'task':self.tracker.update(self.payloads) if scan else None}
    def invalidate(self):self.tracker.reset()
    def preview(self,*args):pass


class TaskTests(unittest.TestCase):
    def test_rules_and_four_group_semantics(self):
        self.assertEqual(qr.parse_task(TASK),(TASK,((1,5,6),(1,2,3),(5,1,6),(2,3,1))))
        self.assertEqual(qr.parse_task(b' \t'+TASK.encode()+b'\r\n')[0],TASK)
        self.assertIsNotNone(qr.parse_task('123+123+456+231')) # 不从例子额外强加两批同颜色集合。
    def test_malformed_duplicate_color_or_wrong_slot_rejected(self):
        for text in ('123+321','111+123+516+231','156+112+516+231','156+456+516+231',
                     '156+123+516+230','156+123+516+231+123','156＋123＋516＋231','１５６+123+516+231',
                     'https://example.com','156+123+516+231;stop',b'\xff',None):
            self.assertIsNone(qr.parse_task(text),text)
    def test_wire_mapping_only_color_groups_and_nonce_as_result_sequence(self):
        encoded=qr.encode_task(TASK,5)
        self.assertEqual(len(encoded),32)
        packets=frames(encoded)
        self.assertEqual([p[2]&7 for p in packets],[1,2,3,4])
        self.assertEqual([(p[2]>>3)&15 for p in packets],[5]*4)
        self.assertEqual([tuple(p[3:7])for p in packets],[(1,6,5,4),(1,2,3,4),(6,1,5,4),(2,3,1,4)])
        self.assertTrue(all(p[1]==7 and p[2]&0x80 and qr.crc8(p[:7])==p[7] for p in packets))
    def test_invalid_encoder_fields(self):
        for text,seq in [('111+123+516+231',0),(TASK,16),(TASK,-1)]:
            with self.assertRaises(ValueError):qr.encode_task(text,seq)
    def test_three_same_frames_and_missing_frame_restart(self):
        tracker=qr.TaskTracker()
        self.assertIsNone(tracker.update([TASK]));self.assertIsNone(tracker.update([TASK]))
        self.assertIsNone(tracker.update([]));self.assertIsNone(tracker.update([TASK]))
        self.assertIsNone(tracker.update([TASK]));self.assertEqual(tracker.update([TASK]),TASK)
    def test_ambiguous_legal_codes_not_selected_and_duplicate_same_text_allowed(self):
        tracker=qr.TaskTracker()
        for _ in range(4):self.assertIsNone(tracker.update([TASK,OTHER]))
        self.assertEqual(tracker.status,'AMBIGUOUS')
        for _ in range(3):result=tracker.update([TASK,TASK,'unrelated'])
        self.assertEqual(result,TASK)


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        self.service=qr.QrService(self.serial,self.backend,self.clock)
    def command(self,mode=6,seq=5,station=4,hz=20):
        self.serial.rx.extend(qr.encode_set(mode,station,hz,seq));self.service.tick()
    def acquire(self):
        self.command()
        self.clock.value=.1;self.service.tick();self.clock.value=.2;self.service.tick()
        self.assertEqual(self.service.locked_task,TASK)
    def groups(self):return [f for f in frames(self.serial.output) if f[1]==qr.QR_GROUP]
    def test_default_stop_no_camera_observation_and_heartbeat(self):
        self.service.tick();self.assertEqual(self.backend.calls,0)
        self.assertEqual(frames(self.serial.output)[0][3:7],bytes((1,0,0,0)))
    def test_ack_before_groups_and_confirm_then_atomic_32_bytes(self):
        self.command();self.assertEqual(self.groups(),[])
        self.clock.value=.1;self.service.tick();self.assertEqual(self.groups(),[])
        self.clock.value=.2;self.service.tick()
        sent=frames(self.serial.output)
        self.assertEqual(sent[0][1],qr.ACK);self.assertEqual(sent[0][5:7],bytes((6,4)))
        self.assertEqual(len(self.groups()),4)
        self.assertEqual([(p[2]>>3)&15 for p in self.groups()],[5]*4)
    def test_latch_does_not_switch_task_and_repeat_keeps_set_nonce(self):
        self.acquire();self.backend.payloads=[OTHER]
        self.clock.value=.3;self.service.tick()
        self.assertEqual(self.service.locked_task,TASK)
        self.assertEqual(self.serial.output[-32:],qr.encode_task(TASK,5))
    def test_same_set_retry_is_idempotent_new_set_clears_latch(self):
        self.acquire();self.clock.value=.3;self.command()
        self.assertEqual(len(self.backend.configs),1);self.assertEqual(self.service.locked_task,TASK)
        self.clock.value=.4;self.command(seq=6)
        self.assertIsNone(self.service.locked_task);self.assertEqual(len(self.backend.configs),2)
        self.clock.value=.5;self.service.tick();self.clock.value=.6;self.service.tick()
        self.assertEqual(self.serial.output[-32:],qr.encode_task(TASK,6))
    def test_stop_no_more_groups_but_task_display_saved(self):
        self.acquire();self.clock.value=.3;self.command(0,6,0,0)
        count=len(self.groups());calls=self.backend.calls
        self.clock.value=.8;self.service.tick()
        self.assertEqual(len(self.groups()),count);self.assertEqual(self.backend.calls,calls)
        self.assertEqual(self.service.saved_display,TASK);self.assertIsNone(self.service.packet)
    def test_no_bad_or_partial_task_groups_sent(self):
        self.backend.payloads=['156+123+516','111+123+516+231']
        self.command()
        for i in range(1,5):self.clock.value=i*.1;self.service.tick()
        self.assertEqual(self.groups(),[])
    def test_decode_failure_does_not_create_task_or_stop_heartbeat(self):
        self.backend.observe=lambda *a,**k:(_ for _ in ()).throw(RuntimeError('decode failure'))
        self.command();self.clock.value=.6;self.service.tick()
        self.assertEqual(self.groups(),[])
        self.assertEqual(len([p for p in frames(self.serial.output)if p[1]==qr.HEARTBEAT]),2)
    def test_short_group_complete_without_control_interleaving(self):
        self.command();self.clock.value=.1;self.service.tick()
        self.serial.writes=[8];self.clock.value=.2;self.service.tick()
        self.assertEqual(len(self.service.writer.pending),24)
        self.service.writer.control(qr.make_frame(qr.HEARTBEAT,0,0,True,bytes((1,6,4,0))),'heartbeat')
        self.clock.value=.21;self.service.tick()
        sent=frames(self.serial.output)
        first=next(i for i,p in enumerate(sent)if p[1]==7)
        self.assertEqual([p[1]for p in sent[first:first+5]],[7,7,7,7,qr.HEARTBEAT])
    def test_stop_discards_old_tail_waits_gap_then_ack(self):
        self.command();self.clock.value=.1;self.service.tick()
        self.serial.output.clear();self.serial.writes=[8];self.clock.value=.2;self.service.tick()
        self.clock.value=.21;self.command(0,6,0,0)
        self.assertEqual(len(self.serial.output),8)
        self.clock.value=.27;self.service.tick()
        self.assertEqual([p[1]for p in frames(self.serial.output[8:])],[qr.ACK])
    def test_timeout_abandons_group_not_requeued_as_control(self):
        self.command();self.clock.value=.1;self.service.tick()
        self.serial.output.clear();self.serial.writes=[8,0,0]
        self.clock.value=.2;self.service.tick();self.clock.value=.22;self.service.tick()
        self.clock.value=.251;self.service.tick()
        self.assertFalse(self.service.writer.pending)
        self.assertFalse(any(p[1]=='qr' for p in self.service.writer.controls))
        self.assertEqual(len(self.serial.output),8)
        self.clock.value=.31;self.service.tick();self.assertEqual(self.serial.output[8:],qr.encode_task(TASK,5))
    def test_set_during_scan_drops_old_result(self):
        self.command();self.clock.value=.1;self.service.tick()
        self.serial.output.clear()
        self.backend.callback=lambda:self.serial.rx.extend(qr.encode_set(0,0,0,6))
        self.clock.value=.2;self.service.tick()
        self.assertEqual(self.serial.output,b'');self.assertIsNone(self.service.locked_task)
    def test_bad_mode_station_target_flags_preserve_current_session(self):
        self.acquire()
        for payload,meta in [(bytes((7,4,20,0)),0),(bytes((6,1,20,0)),0),(bytes((6,4,20,0)),1),(bytes((6,4,20,1)),0)]:
            p=qr.make_frame(0x80,7,meta,False,payload)
            self.service.request(p,.3)
            self.assertEqual(self.service.config[:2],(6,4));self.assertEqual(self.service.locked_task,TASK)
    def test_parser_crc_split_timeout_and_embedded_header(self):
        parser=qr.FrameParser();packet=qr.make_frame(0x80,13,3,False,bytes((6,4,20,0)))
        self.assertEqual(packet[2],0x6B)
        self.assertEqual(parser.feed(b'x'+packet[:3],0),[]);self.assertEqual(parser.feed(packet[3:],.01),[packet])
        broken=bytearray(packet);broken[7]^=1
        self.assertEqual(parser.feed(broken+packet,.02),[packet])
        parser.feed(packet[:3],.03);self.assertEqual(parser.feed(packet,.09),[packet])


def code_image(text=TASK):
    code=cv2.QRCodeEncoder_create().encode(text)
    code=cv2.resize(code,(290,290),interpolation=cv2.INTER_NEAREST)
    frame=np.full((480,640,3),255,np.uint8)
    frame[95:385,175:465]=np.repeat(code[:,:,None],3,axis=2)
    return frame


class PhotoAndBackendTests(unittest.TestCase):
    def test_installed_device_entry_bypasses_cli_for_launcher_arguments(self):
        for argv in ([],['--app_id','cam2_qr'],['cam2_qr','--start-param',''],['--image'],['--help']):
            with patch.object(qr.Path,'is_dir',return_value=True), \
                 patch.object(qr,'run_device',return_value='device') as device, \
                 patch.object(qr.argparse.ArgumentParser,'parse_args',side_effect=AssertionError('device CLI forbidden')):
                self.assertEqual(qr.main(argv),'device')
                device.assert_called_once_with()

    def test_desktop_photo_cli_kept_separate_from_device(self):
        with patch.object(qr.Path,'is_dir',return_value=False),patch.object(qr,'check_photo') as photo,patch.object(qr,'run_device') as device:
            qr.main(['--image','test.png'])
            photo.assert_called_once_with('test.png');device.assert_not_called()
            qr.main([]);device.assert_called_once_with()

    def test_real_generated_qr_and_rotated_scaled_images_decode_expected_task(self):
        image=code_image()
        versions=[image,cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE),cv2.resize(image,(480,360)),
            cv2.GaussianBlur(image,(3,3),.5)]
        detector=cv2.QRCodeDetector()
        for version in versions:
            payload,_,_=detector.detectAndDecode(version)
            self.assertEqual(qr.parse_task(payload)[0],TASK)
    def test_native_decoder_old_keyword_fallback_and_no_decode_after_latch(self):
        calls=[]
        class Frame:
            def __init__(self,w=640,h=480):self.w,self.h=w,h
            def width(self):return self.w
            def height(self):return self.h
            def resize(self,w,h):return Frame(w,h)
            def rotate(self,angle):
                self.assert_angle=angle
                if angle!=180:raise AssertionError('Expected CAM2 rotation180')
                return self
            def find_qrcodes(self,**kwargs):
                calls.append(kwargs)
                if kwargs:raise TypeError('old firmware no decoder_type')
                return [SimpleNamespace(payload=lambda:TASK)]
        cam=SimpleNamespace(read=lambda **kw:Frame())
        image=SimpleNamespace(QRCodeDecoderType=SimpleNamespace(QRCODE_DECODER_TYPE_ZBAR=0))
        backend=qr.MaixQrBackend(None,image,None);backend.cam=cam
        self.assertIsNone(backend.observe(0)['task'])
        self.assertIsNone(backend.observe(.1)['task']);self.assertEqual(backend.observe(.2)['task'],TASK)
        before=len(calls);backend.observe(.3,scan=False);self.assertEqual(len(calls),before)
        self.assertEqual(calls[0],{'decoder_type':0});self.assertFalse(backend.allow_decoder_keyword)
    def test_device_entry_real_decode_uart_and_stop_display_without_gpio(self):
        decoded=cv2.QRCodeDetector();pixels=code_image();shown=[];pin_events=[];cameras=[];rotations=[]
        # Asymmetric corner proves the image is upright, not merely a QR that can decode upside down.
        pixels[430:460,20:70]=(25,80,170)
        raw_pixels=cv2.rotate(pixels,cv2.ROTATE_180)
        class Frame:
            def __init__(self,array):self.array=array;self.strings=[]
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def rotate(self,angle):
                rotations.append(angle)
                return Frame(cv2.rotate(self.array,cv2.ROTATE_180))
            def find_qrcodes(self,**kwargs):
                np.testing.assert_array_equal(self.array,pixels)
                text,_,_=decoded.detectAndDecode(self.array)
                return [] if not text else [SimpleNamespace(payload=lambda:text)]
            def resize(self,w,h):return Frame(cv2.resize(self.array,(w,h)))
            def draw_rect(self,*a,**k):pass
            def draw_string(self,x,y,text,*a):self.strings.append(text)
        class Camera:
            def __init__(self,w,h,fmt,**kwargs):self.reads=0;self.closed=False;cameras.append(self)
            def read(self,**kw):self.reads+=1;return Frame(raw_pixels.copy())
            def close(self):self.closed=True
        serial=Serial();serial.rx.extend(qr.encode_set(6,4,20,5));reader=serial.read;stopped=[False]
        def read(n,timeout=0):
            if not stopped[0] and any(p[1]==7 and p[2]&7==4 for p in frames(serial.output)):
                serial.rx.extend(qr.encode_set(0,0,0,6));stopped[0]=True
            return reader(n,timeout)
        serial.read=read
        disp=SimpleNamespace(show=lambda frame,**k:shown.append(frame),close=lambda:None)
        api=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),Fit=SimpleNamespace(FIT_CONTAIN=0),
            COLOR_BLACK=0,COLOR_WHITE=1,Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)))
        iterations=[0]
        def need_exit():iterations[0]+=1;return iterations[0]>8
        maix=ModuleType('maix');maix.app=SimpleNamespace(need_exit=need_exit);maix.camera=SimpleNamespace(Camera=Camera)
        maix.image=api;maix.display=SimpleNamespace(Display=lambda:disp,set_trans_image_quality=lambda *a:None)
        maix.err=SimpleNamespace(check_raise=lambda *a:None)
        uart=SimpleNamespace(UART=lambda *a,**k:serial,BITS=SimpleNamespace(BITS_8=8),
            PARITY=SimpleNamespace(PARITY_NONE=0),STOP=SimpleNamespace(STOP_1=1))
        peripheral=ModuleType('maix.peripheral');peripheral.uart=uart
        peripheral.pinmap=SimpleNamespace(set_pin_function=lambda *args:pin_events.append(args))
        with patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}):qr.run_device()
        self.assertEqual(pin_events,[('A16','UART0_TX'),('A17','UART0_RX')])
        self.assertTrue(serial.closed and cameras[0].closed)
        self.assertEqual(cameras[0].reads,3)
        self.assertEqual(rotations,[180]*3)  # Frozen release corrects capture before decoding.
        expected=cv2.resize(pixels,(qr.PREVIEW_WIDTH,qr.PREVIEW_HEIGHT))
        self.assertTrue(any(np.array_equal(frame.array,expected) for frame in shown))
        self.assertTrue(any(TASK in frame.strings for frame in shown))
        self.assertEqual(len([p for p in frames(serial.output)if p[1]==7]),4)
    def test_generated_single_file_imports_without_project_protocol(self):
        tools=Path(__file__).resolve().parents[2]/'tools/build_qr_service.py'
        spec=importlib.util.spec_from_file_location('build_qr_file',tools)
        build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)
        output=build.build();source=output.read_text(encoding='utf-8')
        self.assertNotIn('from protocol8 import',source)
        spec=importlib.util.spec_from_file_location('generated_cam2',output)
        generated=importlib.util.module_from_spec(spec);spec.loader.exec_module(generated)
        self.assertEqual(generated.encode_task(TASK,5),qr.encode_task(TASK,5))
        self.assertEqual(generated.CAMERA_ROTATION_DEG,180)

    def test_rotation_failure_does_not_decode_uncorrected_frame(self):
        for rotated in (None,SimpleNamespace(width=lambda:480,height=lambda:640)):
            frame=SimpleNamespace(width=lambda:640,height=lambda:480,rotate=lambda angle:rotated,
                find_qrcodes=lambda:self.fail('Must not decode after failed rotation'))
            backend=qr.MaixQrBackend(None,SimpleNamespace(),None)
            backend.cam=SimpleNamespace(read=lambda **kw:frame)
            with self.assertRaisesRegex(RuntimeError,'rotation failed'):backend.observe(0)

    def test_rotation_runs_even_after_latch_and_no_mirroring(self):
        pixels=code_image();pixels[10:30,40:60]=(19,28,47)
        events=[]
        class Frame:
            def __init__(self,array):self.array=array
            def width(self):return 640
            def height(self):return 480
            def rotate(self,angle):
                events.append(angle);return Frame(cv2.rotate(self.array,cv2.ROTATE_180))
            def find_qrcodes(self):raise AssertionError('Latch must skip decoder')
        backend=qr.MaixQrBackend(None,SimpleNamespace(),None)
        backend.cam=SimpleNamespace(read=lambda **kw:Frame(cv2.rotate(pixels,cv2.ROTATE_180)))
        self.assertIsNone(backend.observe(0,scan=False)['task'])
        np.testing.assert_array_equal(backend.frame.array,pixels)
        self.assertEqual(events,[180])






if __name__=='__main__':unittest.main()
