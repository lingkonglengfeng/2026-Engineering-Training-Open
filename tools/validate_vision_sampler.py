"""Build and record the camera/light-only sampler's offline checks."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest
from build_vision_sampler import build,ROOT

OUT=ROOT/'output/vision_sampler_checked_2026-10-13'


def validate():
    build();OUT.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('sampler_checks',ROOT/'vision/maixcam_sampler/test_sampler.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    log=io.StringIO()
    with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
    (OUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    data={'build':module.sample.SAMPLER_BUILD,'tests':result.testsRun,'failures':len(result.failures),
          'errors':len(result.errors),'hardware_tested':False,'capture':[480,360],
          'lamp':{'ring':15,'plate':50},
          'checks':['raw PNG pixels and same-frame320x240 image',
                    'light settle then flush then new read; reject old read/profile epoch',
                    'bad PWM and unavailable clear_buff inhibit capture',
                    'reserved numbering preserves previous files; failed groups cleaned',
                    'physical touch contain mapping and normal/failed startup cleanup'],
          'limits':['Mock Maix SDK and real local files; not device FPS/ZBAR/physical lamp testing']}
    (OUT/'validation.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    if not result.wasSuccessful():raise AssertionError(log.getvalue()[-5000:])
    print('%d sampler checks PASS'%result.testsRun)
    return data


if __name__=='__main__':validate()
