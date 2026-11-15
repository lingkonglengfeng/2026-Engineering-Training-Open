"""Camera LCD/raw/PC-annotation channel with fake displays and loopback sockets."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import socket,struct,sys,threading,time,unittest
from unittest.mock import patch
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'vision/pc_vision'),str(ROOT/'vision')]
import wire
sys.modules['pc_wire']=wire
spec=importlib.util.spec_from_file_location('screen_bridge',ROOT/'vision/maixcam_bridge/bridge.py')
bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)
from screen_preview import ScreenPreviewClient,SCREEN_HEADER,SCREEN_MAGIC

def until(check,seconds=2):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if check():return True
        time.sleep(.005)
    return False

class Frame:
    def __init__(self,identity=0):self.identity=identity;self.labels=[];self.copies=0
    def copy(self):self.copies+=1;return Frame(self.identity)
    def draw_string(self,*args):self.labels.append(args)
    def width(self):return 480
    def height(self):return 360
    def to_jpeg(self,quality):return SimpleNamespace(to_bytes=lambda:b'RAW-JPEG')

class Display:
    def __init__(self,state):self.state=state;self.frames=[];self.closed=False
    def show(self,frame):self.frames.append(frame);self.state.stop.set()
    def close(self):self.closed=True

IMAGE=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0,FMT_JPEG=1),COLOR_WHITE=1)

class CameraScreen(unittest.TestCase):
    def test_protocol_layout_matches_and_old_bridge_never_connects(self):
        self.assertEqual(SCREEN_HEADER.format,bridge.SCREEN_HEADER.format)
        sender=ScreenPreviewClient('127.0.0.1',{})
        self.assertIsNone(sender.thread);sender.submit(np.zeros((240,320,3),np.uint8),(1,2,3));sender.close()
        self.assertIsNone(sender.latest)

    def test_raw_preview_rate_limit_latest_slot_and_overlay_copy(self):
        state=bridge.BridgeState('qr');state.screen_enabled=True;source=Frame(1)
        with patch.object(bridge.time,'monotonic',return_value=1):
            bridge.publish_screen_frame(state,source);bridge.publish_screen_frame(state,source)
        self.assertEqual(source.copies,1)
        newer=Frame(2)
        with patch.object(bridge.time,'monotonic',return_value=1.2):bridge.publish_screen_frame(state,newer)
        self.assertEqual(state.screen_latest.identity,2)
        state.display_task='123+321';disp=Display(state);bridge.display_frames(state,disp,IMAGE)
        self.assertTrue(disp.closed);self.assertEqual(source.labels,[]);self.assertEqual(newer.labels,[])
        self.assertIn('123+321',disp.frames[0].labels[1]);self.assertIsNone(state.capture_error)

    def test_display_error_only_disables_display(self):
        state=bridge.BridgeState();state.screen_enabled=True;state.screen_latest=Frame()
        class Broken(Display):
            def show(self,frame):raise RuntimeError('LCD unavailable')
        disp=Broken(state);bridge.display_frames(state,disp,IMAGE)
        self.assertFalse(state.screen_enabled);self.assertIsNone(state.capture_error)
        self.assertFalse(state.stop.is_set());self.assertTrue(disp.closed)

    def test_pc_annotation_is_used_without_second_overlay(self):
        state=bridge.BridgeState('qr');state.screen_enabled=True;stamp=time.monotonic_ns()
        state.screen_annotated=(state.epoch,42,stamp,b'ANNOTATED');decoded=Frame(42)
        image=SimpleNamespace(Format=IMAGE.Format,from_bytes=lambda w,h,f,data:SimpleNamespace(to_format=lambda fmt:decoded))
        disp=Display(state);bridge.display_frames(state,disp,image)
        self.assertEqual(disp.frames,[decoded]);self.assertEqual(decoded.labels,[])

    def test_annotation_epoch_and_age(self):
        state=bridge.BridgeState();now=time.monotonic_ns()
        self.assertTrue(bridge.screen_annotation_current(state,(0,1,now,b'x')))
        self.assertFalse(bridge.screen_annotation_current(state,(1,1,now,b'x')))
        self.assertFalse(bridge.screen_annotation_current(state,(0,1,now-2_000_000_000,b'x')))

    def test_roundtrip_annotated_pixels_and_no_uart_results(self):
        state=bridge.BridgeState();state.screen_enabled=True
        listener=bridge.server_socket(0);state.screen_port=listener.getsockname()[1]
        thread=threading.Thread(target=bridge.screen_server,args=(state,listener),daemon=True);thread.start()
        sender=ScreenPreviewClient('127.0.0.1',state.hello())
        try:
            stamp=time.monotonic_ns();state.remember_frame(0,42,stamp)
            frame=np.zeros((240,320,3),np.uint8);cv2.circle(frame,(160,120),50,(0,255,0),3)
            sender.submit(frame,(0,42,stamp));self.assertTrue(until(lambda:state.screen_annotated is not None))
            result=cv2.imdecode(np.frombuffer(state.screen_annotated[3],np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(result.shape,(240,320,3));self.assertGreater(int(result[120,210,1]),150)
            self.assertTrue(state.results.empty());self.assertEqual(state.frames,0);self.assertIsNone(state.capture_error)
        finally:
            sender.close();state.stop.set();listener.close();thread.join(2)
        self.assertFalse(thread.is_alive());self.assertFalse(sender.thread.is_alive())

    def test_receiver_rejects_unknown_frame_without_touching_control(self):
        state=bridge.BridgeState();state.screen_enabled=True
        listener=bridge.server_socket(0);thread=threading.Thread(target=bridge.screen_server,args=(state,listener),daemon=True);thread.start()
        try:
            with socket.create_connection(('127.0.0.1',listener.getsockname()[1]),timeout=1) as sock:
                sock.sendall(SCREEN_HEADER.pack(SCREEN_MAGIC,0,999,time.monotonic_ns(),320,240,3)+b'abc')
                time.sleep(.03)
            self.assertIsNone(state.screen_annotated);self.assertTrue(state.results.empty())
        finally:state.stop.set();listener.close();thread.join(2)

    def test_local_capture_without_pc_does_not_jpeg_encode(self):
        state=bridge.BridgeState();state.screen_enabled=True
        class Camera:
            def __init__(self,*a,**kw):self.closed=False
            def read(self,**kw):state.stop.set();return Frame(7)
            def close(self):self.closed=True
        bridge.capture_frames(state,SimpleNamespace(Camera=Camera),IMAGE)
        self.assertEqual(state.frames,0);self.assertIsNone(state.latest)
        self.assertEqual(state.screen_latest.identity,7);self.assertIsNone(state.capture_error)

    def test_slow_lcd_keeps_single_camera_and_capture_progress(self):
        state=bridge.BridgeState();state.screen_enabled=state.capture_active=True
        entered,release=threading.Event(),threading.Event();instances=[]
        class Camera:
            def __init__(self,*args,**kwargs):self.count=0;self.closed=False;instances.append(self)
            def read(self,**kwargs):time.sleep(.003);self.count+=1;return Frame(self.count)
            def close(self):self.closed=True
        class Slow(Display):
            def show(self,frame):entered.set();release.wait(2)
        capture=threading.Thread(target=bridge.capture_frames,args=(state,SimpleNamespace(Camera=Camera),IMAGE),daemon=True)
        lcd=threading.Thread(target=bridge.display_frames,args=(state,Slow(state),IMAGE),daemon=True)
        capture.start();lcd.start()
        try:
            self.assertTrue(entered.wait(1));before=state.frames
            self.assertTrue(until(lambda:state.frames>=before+20))
            self.assertEqual(len(instances),1);self.assertIsNone(state.capture_error)
            self.assertTrue(state.results.empty());self.assertIsNotNone(state.latest)
        finally:state.stop.set();release.set();capture.join(2);lcd.join(2)
        self.assertTrue(instances[0].closed);self.assertFalse(capture.is_alive());self.assertFalse(lcd.is_alive())

if __name__=='__main__':unittest.main()
