"""Offline CAM1 manual calibration + prior material/ring/service/light regressions."""
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
OUT=ROOT/'output/cam1_ring_target_visible_2026-10-15'


def validate():
    OUT.mkdir(parents=True,exist_ok=True);log=io.StringIO()
    with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        loader=unittest.TestLoader()
        suite=unittest.TestSuite(loader.discover(str(ROOT/p),pattern='test*.py',top_level_dir=str(ROOT/p))
            for p in ('vision/maixcam_material','vision/maixcam_ring','vision/maixcam_service','vision/maixcam_light'))
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    (OUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    record={'build':service.SERVICE_BUILD,'tests':result.testsRun,'failures':len(result.failures),
            'errors':len(result.errors),'hardware_run':False,
            'calibration_set':{'mode':2,'station':1,'flag':4,'target':'wire color0..6; 0 starts RED'},
            'file':'~/maixcam_material_colors.json; original version1 atomic format',
            'checks':['exact flag context; old ring flags0..3 preserved',
                      'TEMP station3 flag08 opt-in: current tracked target2 presence independent from 3-ring XY',
                      'cold 1/2 rings, cached-only observation, wrong identity, slow processing and faults rejected',
                      'partial count not inflated; XY/axis still invalid; same image/SET binding and short-write isolation',
                      'real file save/reload, replacing only selected color; preserve other colors',
                      'blue/lightblue pair separator and rejection of glare; exactly3 new points',
                      'first1/2 points never replace old threshold; incomplete Save/Exit blocked; Cancel preserves old',
                      'RAW/MASK comparison uses non-destructive native-style binary image copy',
                      'three independent presses, no minimum-distance rule; no center Sample button',
                      'portrait contain touch mapping, held press, outside-image clicks',
                      'touch opened before successful ACK; no material detector during calibration',
                      '02 VALID0 and zero payload even if backend accidentally returns coordinates',
                      'Save errors keep dirty UI; Exit cannot claim done until save succeeds',
                      'Exit waits for MCU STOP; STOP discards unsaved RAM and closes touch',
                      'touch creation failure -> failed ACK, not false ready',
                      'existing material/ring/plate/PWM regressions'],
            'light':{'ring_percent':service.RING_LIGHT_BRIGHTNESS_PERCENT,'other_percent':service.LIGHT_BRIGHTNESS_PERCENT},
            'limits':['No physical touch/camera/MCU/vehicle run',
                      'Offline UI preview approximates Maix fonts, not a screenshot of hardware']}
    (OUT/'validation.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    if not result.wasSuccessful():raise AssertionError(log.getvalue()[-5000:])
    print('%d tests PASS; manual calibration hardware run not performed'%result.testsRun)
    return record


if __name__=='__main__':validate()
