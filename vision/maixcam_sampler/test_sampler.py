import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('sampler_generated', Path(__file__).with_name('main.py'))
sample = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sample)


class Frame:
    def __init__(self, width=480, height=360):
        self.w, self.h, self.events = width, height, []
    def width(self): return self.w
    def height(self): return self.h
    def save(self, path):
        self.events.append(('save', path))
        Path(path).write_bytes(('raw_%dx%d_no_overlays' % (self.w, self.h)).encode())
        return 0
    def resize(self, width, height):
        self.events.append(('resize', width, height))
        return Frame(width, height)


class SamplerTests(unittest.TestCase):
    def ready(self,state):
        return state.flush_if_ready(SimpleNamespace(clear_buff=lambda:None))
    def test_native_png_metadata_and_plate_processing_same_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            tick = [0.0]; duties = []
            s = sample.Sampler(SimpleNamespace(set_brightness=duties.append), directory, lambda: tick[0])
            frame = Frame()
            self.assertFalse(s.capture(frame))
            self.assertEqual(frame.events, [])
            tick[0] = 0.5
            self.ready(s)
            self.assertTrue(s.capture(frame))
            s.switch('plate'); self.assertFalse(s.capture(frame)); tick[0] = 1.0
            self.ready(s)
            self.assertTrue(s.capture(frame))
            rows = [json.loads(line) for line in (Path(directory) / 'samples.jsonl').read_text().splitlines()]
            self.assertEqual(duties, [15.0, 50.0])
            self.assertEqual([r['lamp_percent'] for r in rows], [15.0, 50.0])
            self.assertTrue(all(r['capture'] == [480, 360] and not r['annotations'] for r in rows))
            self.assertEqual((Path(directory) / rows[0]['file']).read_bytes(), b'raw_480x360_no_overlays')
            self.assertEqual((Path(directory) / rows[1]['processing_file']).read_bytes(), b'raw_320x240_no_overlays')
            self.assertFalse(list(Path(directory).rglob('*.pending.png')))
            self.assertEqual(s.count, 2)

    def test_light_failure_cannot_select_wrong_profile(self):
        def duty(value):
            if value == 50: raise RuntimeError('PWM rejected')
        with tempfile.TemporaryDirectory() as directory:
            s = sample.Sampler(SimpleNamespace(set_brightness=duty), directory)
            with self.assertRaises(RuntimeError): s.switch('plate')
            self.assertEqual(s.mode, 'ring')

    def test_save_error_does_not_count_or_publish_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            tick=[0.0]
            s=sample.Sampler(SimpleNamespace(set_brightness=lambda value: None),directory,lambda:tick[0])
            tick[0]=1.0
            self.ready(s)
            frame=Frame();frame.save=lambda path: -1
            with self.assertRaises(RuntimeError): s.capture(frame)
            self.assertEqual(s.count,0)
            self.assertFalse((Path(directory)/'samples.jsonl').exists())

    def test_buttons_light_settle_auto_limit_and_bad_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            tick=[0.0];duties=[]
            s=sample.Sampler(SimpleNamespace(set_brightness=duties.append),directory,lambda:tick[0])
            frame=Frame();s.action(200,8,frame);s.action(200,220,frame)
            self.assertEqual(s.mode,'plate');self.assertTrue(s.auto)
            self.assertFalse(s.capture(frame));tick[0]=1.0
            self.ready(s)
            s.action(10,220,frame);self.assertEqual(s.count,1)
            with self.assertRaises(RuntimeError): s.capture(Frame(640,480))
            s.count=sample.MAX_SAMPLES;self.assertFalse(s.capture(frame));self.assertFalse(s.auto)

    def test_device_start_failure_turns_light_off_and_never_opens_uart(self):
        maix=ModuleType('maix');peripheral=ModuleType('maix.peripheral');off=[]
        def bad_camera(*args,**kwargs):raise RuntimeError('Camera unavailable')
        maix.app=SimpleNamespace();maix.camera=SimpleNamespace(Camera=bad_camera)
        maix.display=SimpleNamespace();maix.image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0))
        maix.err=SimpleNamespace();maix.touchscreen=SimpleNamespace()
        peripheral.gpio=peripheral.pinmap=SimpleNamespace()  # No UART API even available.
        light=SimpleNamespace(set_brightness=lambda value: None)
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}), \
             patch.object(sample,'SAVE_ROOT',directory),patch.object(sample,'init_light_on',return_value=light), \
             patch.object(sample,'light_off_on_exit',side_effect=off.append):
            with self.assertRaises(RuntimeError):sample.run_device()
        self.assertEqual(off,[light])

    def test_frame_epoch_and_after_settle_flush_reject_old_read(self):
        with tempfile.TemporaryDirectory() as directory:
            tick=[0.0];state=sample.Sampler(SimpleNamespace(set_brightness=lambda v:None),directory,lambda:tick[0])
            old_epoch=state.epoch
            tick[0]=.5;self.ready(state)
            self.assertFalse(state.capture(Frame(),.49,old_epoch))
            state.switch('plate');tick[0]=1.0
            self.assertFalse(state.capture(Frame(),1.0,state.epoch)) # not flushed yet
            self.ready(state)
            self.assertFalse(state.capture(Frame(),1.0,old_epoch))
            self.assertTrue(state.capture(Frame(),1.0,state.epoch))

    def test_unsupported_clear_buff_and_failed_pwm_inhibit_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            tick=[0.0]
            def duty(value):
                if value==50:raise RuntimeError('unknown actual lamp state')
            state=sample.Sampler(SimpleNamespace(set_brightness=duty),directory,lambda:tick[0])
            tick[0]=1
            self.assertFalse(state.flush_if_ready(SimpleNamespace()))
            self.assertFalse(state.capture(Frame()))
            self.ready(state)
            with self.assertRaises(RuntimeError):state.switch('plate')
            self.assertFalse(state.capture(Frame()));self.assertFalse(state.light_ok)

    def test_scaled_or_metadata_failure_no_orphans_and_retry_never_overwrites(self):
        for stage in ('scaled','metadata'):
            with tempfile.TemporaryDirectory() as directory:
                tick=[0.0];state=sample.Sampler(SimpleNamespace(set_brightness=lambda v:None),directory,lambda:tick[0])
                state.switch('plate');tick[0]=1;self.ready(state)
                frame=Frame()
                if stage=='scaled':frame.resize=lambda *args:SimpleNamespace(save=lambda path:-1)
                else:state.append_record=lambda row:(_ for _ in ()).throw(OSError('disk full'))
                with self.assertRaises((RuntimeError,OSError)):state.capture(frame)
                self.assertEqual(state.count,0);self.assertFalse(list(Path(directory).rglob('*.png')))
                self.assertTrue(list(Path(directory).rglob('*.reserved')))
                state.append_record=sample.Sampler.append_record.__get__(state,sample.Sampler)
                self.assertTrue(state.capture(Frame()))
                rows=(Path(directory)/'samples.jsonl').read_text().splitlines()
                self.assertEqual(len(rows),1);self.assertIn('000002',json.loads(rows[0])['file'])

    def test_existing_sample_name_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            profile=Path(directory)/'ring';profile.mkdir()
            existing=profile/'ring_000001.png';existing.write_bytes(b'keep existing')
            tick=[0.0];state=sample.Sampler(SimpleNamespace(set_brightness=lambda v:None),directory,lambda:tick[0])
            tick[0]=1;self.ready(state);state.capture(Frame())
            self.assertEqual(existing.read_bytes(),b'keep existing')
            row=json.loads((Path(directory)/'samples.jsonl').read_text())
            self.assertIn('000002',row['file'])

    def test_short_metadata_append_rolls_back_without_damaging_previous_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'samples.jsonl';prefix=b'{"file":"previous.png"}\n';path.write_bytes(prefix)
            state=sample.Sampler(SimpleNamespace(set_brightness=lambda v:None),directory)
            original_open=Path.open
            class ShortWriter:
                def __init__(self,stream):self.stream=stream
                def __enter__(self):return self
                def __exit__(self,*args):self.stream.close()
                def tell(self):return self.stream.tell()
                def write(self,data):return self.stream.write(data[:5])
                def flush(self):return self.stream.flush()
                def seek(self,pos):return self.stream.seek(pos)
                def truncate(self,pos):return self.stream.truncate(pos)
            def short_open(target,*args,**kwargs):
                stream=original_open(target,*args,**kwargs)
                return ShortWriter(stream) if target==path and args and args[0]=='ab' else stream
            with patch.object(Path,'open',short_open):
                with self.assertRaises(OSError):state.append_record({'file':'new.png'})
            self.assertEqual(path.read_bytes(),prefix)

    def test_real_png_pixels_and_scaled_same_frame_are_unannotated(self):
        from PIL import Image
        class Pixels:
            def __init__(self,image):self.image=image
            def width(self):return self.image.width
            def height(self):return self.image.height
            def save(self,path):self.image.save(path);return 0
            def resize(self,w,h):return Pixels(self.image.resize((w,h)))
        with tempfile.TemporaryDirectory() as directory:
            tick=[0.0];state=sample.Sampler(SimpleNamespace(set_brightness=lambda v:None),directory,lambda:tick[0])
            state.switch('plate');tick[0]=1;self.ready(state)
            original=Image.new('RGB',(480,360),(70,120,210));original.putpixel((50,70),(255,0,0))
            state.capture(Pixels(original))
            row=json.loads((Path(directory)/'samples.jsonl').read_text())
            with Image.open(Path(directory)/row['file']) as saved:
                self.assertEqual(saved.tobytes(),original.tobytes())
            with Image.open(Path(directory)/row['processing_file']) as saved:
                self.assertEqual(saved.tobytes(),original.resize((320,240)).tobytes())

    def test_physical_touch_contain_mapping_and_normal_exit_cleanup(self):
        tick=[0.0];events=[];reads=[0]
        class DrawFrame(Frame):
            def resize(self,w,h):return DrawFrame(w,h)
            def draw_rect(self,*a,**k):events.append(('draw',self.w,self.h))
            def draw_string(self,*a,**k):pass
        def read(**kw):reads[0]+=1;tick[0]+=.6;return DrawFrame()
        def need_exit():return reads[0]>=3
        delivered=[False,False]
        def available():return reads[0] in (2,3) and not delivered[reads[0]-2]
        def touch_read():
            delivered[reads[0]-2]=True
            return (80*1.15,138+224*1.15,reads[0]==2)
        maix=ModuleType('maix');peripheral=ModuleType('maix.peripheral')
        maix.app=SimpleNamespace(need_exit=need_exit)
        maix.camera=SimpleNamespace(Camera=lambda *a,**k:SimpleNamespace(read=read,
            clear_buff=lambda:events.append(('clear',tick[0])),close=lambda:events.append(('camclose',))))
        maix.display=SimpleNamespace(Display=lambda:SimpleNamespace(width=lambda:368,height=lambda:552,
            show=lambda *a,**k:0,close=lambda:events.append(('displayclose',))))
        maix.touchscreen=SimpleNamespace(TouchScreen=lambda:SimpleNamespace(available=available,read0=touch_read,
            close=lambda:events.append(('touchclose',))))
        maix.image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0),Fit=SimpleNamespace(FIT_CONTAIN=0),COLOR_BLACK=0,COLOR_WHITE=1)
        maix.err=SimpleNamespace(check_raise=lambda *a:None);peripheral.gpio=peripheral.pinmap=SimpleNamespace()
        light=SimpleNamespace(set_brightness=lambda v:events.append(('lamp',v)))
        with tempfile.TemporaryDirectory() as directory,patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}), \
             patch.object(sample,'SAVE_ROOT',directory),patch.object(sample,'init_light_on',return_value=light), \
             patch.object(sample,'light_off_on_exit',side_effect=lambda v:events.append(('off',))), \
             patch.object(sample.time,'monotonic',side_effect=lambda:tick[0]):
            sample.run_device()
            paths=list(Path(directory).rglob('samples.jsonl'));self.assertEqual(len(paths),1)
            rows=paths[0].read_text().splitlines();self.assertEqual(len(rows),1)
        self.assertIn(('clear',.6),events)
        self.assertEqual(events[-4:],[('off',),('camclose',),('touchclose',),('displayclose',)])


if __name__=='__main__':unittest.main()
