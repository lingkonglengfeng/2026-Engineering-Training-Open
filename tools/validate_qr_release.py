"""CAM2 offline release check using the electrical thread's eight actual QR samples.

Requires OpenCV/NumPy (local tmp/maixcam_cv_deps is used when present).
Exercises MaixQrBackend + QrService with simulated SDK/UART; not a hardware test.
"""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tmp/maixcam_cv_deps'), str(ROOT/'vision/maixcam_qr')]
import cv2
import numpy as np
import test_qr_service as tests
qr = tests.qr
SAMPLES = ROOT/'output/qr_samples_v103_2026-10-09'
OUTPUT = ROOT/'output/cam2_fixed_rollback_v103_2026-10-13'


def validate():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    log = io.StringIO()
    results = []
    with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        suite = unittest.defaultTestLoader.loadTestsFromModule(tests)
        unit = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
        if not unit.wasSuccessful():
            (OUTPUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
            raise AssertionError('CAM2 unit checks failed; see tests.log')
        samples = json.loads((SAMPLES/'manifest.json').read_text(encoding='utf-8'))['samples']
        assert len(samples)==8
        for sequence, sample in enumerate(samples):
            path = SAMPLES/sample['png']
            assert hashlib.sha256(path.read_bytes()).hexdigest()==sample['png_sha256']
            pixels = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)
            detector = cv2.QRCodeDetector()
            for original in (pixels, cv2.rotate(pixels, cv2.ROTATE_180)):
                assert detector.detectAndDecode(original)[0]==sample['payload']
            upright = np.full((480,640,3), 255, np.uint8)
            upright[50:430,130:510] = cv2.resize(pixels,(380,380),interpolation=cv2.INTER_NEAREST)
            upright[435:460,20:75] = (17,93,151)  # asymmetric orientation marker
            raw = cv2.rotate(upright,cv2.ROTATE_180)
            previews = []

            class Frame:
                def __init__(self,array):self.array=array
                def width(self):return self.array.shape[1]
                def height(self):return self.array.shape[0]
                def rotate(self,angle):
                    assert angle==180
                    return Frame(cv2.rotate(self.array,cv2.ROTATE_180))
                def find_qrcodes(self):
                    np.testing.assert_array_equal(self.array,upright)
                    payload = detector.detectAndDecode(self.array)[0]
                    return [SimpleNamespace(payload=lambda:payload)] if payload else []
                def resize(self,w,h):return Frame(cv2.resize(self.array,(w,h)))
                def draw_rect(self,*args,**kwargs):pass
                def draw_string(self,*args,**kwargs):pass

            image = SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),
                Fit=SimpleNamespace(FIT_CONTAIN=0),COLOR_BLACK=0,COLOR_WHITE=1,
                Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)))
            camera = SimpleNamespace(Camera=lambda *a,**kw:SimpleNamespace(
                read=lambda **kw:Frame(raw.copy()),close=lambda:None))
            backend=qr.MaixQrBackend(camera,image,SimpleNamespace(show=lambda frame,**kw:previews.append(frame)))
            serial,clock = tests.Serial(),tests.Clock()
            service = qr.QrService(serial,backend,clock)
            serial.rx.extend(qr.encode_set(6,4,20,sequence))
            for now in (0,.1,.2):clock.value=now;service.tick()
            assert service.locked_task==sample['payload']
            groups=[p for p in tests.frames(serial.output) if p[1]==7]
            assert len(groups)==4 and b''.join(groups)==qr.encode_task(sample['payload'],sequence)
            expected_groups = [tuple(sample['colors_rule'][0]),tuple(sample['positions'][0]),
                               tuple(sample['colors_rule'][1]),tuple(sample['positions'][1])]
            for index, packet in enumerate(groups):
                expected=expected_groups[index]
                if index in (0,2):expected=tuple((0,1,2,3,4,6,5)[c] for c in expected)
                assert tuple(packet[3:7])==(*expected,4)
                assert (packet[2]>>3)&15==sequence and packet[2]&0x80
            expected_preview=cv2.resize(upright,(320,240))
            assert any(np.array_equal(f.array,expected_preview) for f in previews)
            clock.value=.3;serial.rx.extend(qr.encode_set(0,0,0,(sequence+1)%16));service.tick()
            assert service.config[0]==0 and service.packet is None
            assert service.saved_display==sample['payload']
            backend.close()
            results.append({'payload':sample['payload'],'png_sha256':sample['png_sha256'],
                            'decode_upright':True,'decode_rotated180':True,
                            'backend_service_rotated':True,'same_corrected_preview':True,
                            'stop_clears_packet':True,'sequence':sequence,
                            'qr_group_hex':b''.join(groups).hex(' ')})
    assert len(results)==8, 'All frozen QR samples must run'
    (OUTPUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    validation={'build':qr.BUILD,'unit_tests':unit.testsRun,'unit_failures':len(unit.failures),
                'unit_errors':len(unit.errors),'sample_cases':results,
                'opencv_version':cv2.__version__,
                'limitation':'Offline OpenCV + simulated Maix/UART; no hardware, real ZBAR/FPS or motor validation'}
    (OUTPUT/'validation.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return validation


if __name__=='__main__':
    result=validate()
    print('CAM2: %d unit tests, %d actual QR backend/service cases PASS'%(result['unit_tests'],len(result['sample_cases'])))
