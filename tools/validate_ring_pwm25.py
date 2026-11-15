"""Offline mode-switched RING PWM25 / other modes PWM50 regressions."""
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_material'),
             str(ROOT/'vision/maixcam_ring'),str(ROOT/'vision/maixcam_service'),str(ROOT/'vision/maixcam_light')]
import service
OUT=ROOT/'output/cam1_ring_pwm25_2026-10-12'


def validate():
    OUT.mkdir(parents=True,exist_ok=True);log=io.StringIO()
    with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        loader=unittest.TestLoader()
        suite=unittest.TestSuite(loader.discover(str(ROOT/p),pattern='test*.py',top_level_dir=str(ROOT/p))
            for p in ('vision/maixcam_ring','vision/maixcam_service','vision/maixcam_light'))
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    (OUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    record={'build':service.SERVICE_BUILD,'tests':result.testsRun,'failures':len(result.failures),
            'errors':len(result.errors),'hardware_run':False,
            'lamp':{'pin':service.LIGHT_PIN,'pwm':service.LIGHT_PWM_ID,
                    'frequency_hz':service.LIGHT_PWM_FREQ_HZ,'default_duty_percent':service.LIGHT_BRIGHTNESS_PERCENT,
                    'ring_duty_percent':service.RING_LIGHT_BRIGHTNESS_PERCENT},
            'isp_changes':False,'luma50_version':'1.1.9 withdrawn after user clarified LED PWM',
            'checks':['Ring profiles and cross-mode/close never call camera luma/exposure/gain/AE',
                      'Same PWM channel: all RING flags25, MATERIAL/PLATE/STOP50; active-low inversion',
                      'Failed duty change rolls back and returns failed ACK, no false acceptance',
                      'Existing PWM initialization, active-level inversion, cleanup and failures',
                      'Existing ring/service positioning and same-frame protocol regressions'],
            'limits':['No hardware lamp optical brightness or FPS measurement']}
    (OUT/'validation.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    if not result.wasSuccessful():raise AssertionError(log.getvalue()[-5000:])
    print('%d tests PASS; RING PWM25 / other modes PWM50; no ISP changes'%result.testsRun)
    return record


if __name__=='__main__':validate()
