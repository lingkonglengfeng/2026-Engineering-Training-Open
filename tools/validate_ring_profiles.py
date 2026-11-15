"""Compare same-resolution empty vs occupied profiles on frozen screenshots.

Synthetic translations/brightness are not captured video. Desktop CPU timing
cannot establish MaixCAM FPS. Writes only the named CAM1 validation folder.
"""
import contextlib
import io
import json
import math
from pathlib import Path
import statistics
import sys
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_material'),
             str(ROOT/'vision/maixcam_ring'),str(ROOT/'vision/maixcam_service')]
import cv2
import numpy as np
import middle_ring as ring

OUT=ROOT/'output/cam1_ring_profiles_v105_2026-10-09'


def validate():
    OUT.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    records=[];log=io.StringIO()
    with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        loader=unittest.TestLoader()
        suite=unittest.TestSuite(loader.discover(str(ROOT/p),pattern='test*.py',top_level_dir=str(ROOT/p))
            for p in ('vision/maixcam_ring','vision/maixcam_service'))
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
        if not result.wasSuccessful():raise AssertionError('Ring/service tests failed: '+log.getvalue()[-5000:])
    (OUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    for fixture in ('live_complete','live_cropped','live_glare_1_cleaned','live_glare_2_cleaned',
                    'occupied_2_cleaned','occupied_3_cleaned','occupied_4_cleaned','occupied_5_cleaned'):
        path=ROOT/'vision/maixcam_ring/test_data'/f'{fixture}.png'
        source=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
        # Frozen screenshots have removed UI rows. Do not stretch/pad their clipped
        # boundaries: compare both profiles on exactly the same native screenshot.
        height,width=source.shape
        comparison=[]
        for allow in (False,True):
            detector,tracker=ring.RingDetector(),ring.MiddleTracker()
            detector.allow_occupied=allow;detector.require_scene=True
            centers=[];elapsed=[];modes=[];counts=[]
            with patch.object(ring,'recover_occupied_rings',wraps=ring.recover_occupied_rings) as recover, \
                 patch.object(ring,'occupied_sample_group',wraps=ring.occupied_sample_group) as sample:
                for frame in range(12):
                    transform=np.float32([[1,0,math.sin(frame*.5)],[0,1,.8*math.cos(frame*.6)]])
                    image=cv2.warpAffine(source,transform,(width,height),borderValue=180)
                    image=np.clip(image.astype(np.float32)*(1+.05*math.sin(frame*.7)),0,255).astype(np.uint8)
                    now=frame*.08;stats={};start=time.perf_counter()
                    groups,triple,binary,status=detector.process(image,tracker,now,stats)
                    observed=tracker.update(groups,triple,now,status)
                    detector.accept(observed,triple,image,now)
                    elapsed.append((time.perf_counter()-start)*1000)
                    centers.append(list(observed['ring']['center']) if observed and len(groups)==3 else None)
                    modes.append(stats['mode']);counts.append(len(groups))
                if not allow:assert recover.call_count==sample.call_count==0
                records.append({'fixture':fixture,'occupied':allow,'processing_size':[width,height],
                    'confirmed_after_acquire':sum(c is not None for c in centers[3:]),'frames_after_acquire':9,
                    'desktop_median_ms':statistics.median(elapsed),'desktop_max_ms':max(elapsed),
                    'occupied_recover_calls':recover.call_count,'occupied_sample_calls':sample.call_count,
                    'counts':counts,'centers':centers,'modes':modes})
            observation=None if triple is None else {'ring':triple['ring']}
            picture=ring.annotate(image,groups,observation,status='OCCUPIED' if allow else 'EMPTY')
            comparison.append(picture)
        cv2.imencode('.png',np.hstack(comparison))[1].tofile(str(OUT/f'{fixture}_comparison.png'))
    report={'build':'2026-10-09-ring-profiles-v22','tests':result.testsRun,'failures':0,'errors':0,
            'desktop_opencv':cv2.__version__,'hardware_run':False,
            'limitations':['Cleaned screenshots and synthetic perturbations, not captured video',
                           'Desktop CPU timings do not prove MaixCAM FPS',
                           'Comparison images: left EMPTY, right OCCUPIED; same native screenshot size',
                           'Production capture/detection stays 480x360; cleaned screenshot sizes differ'],
            'records':records}
    (OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'{result.testsRun} tests PASS; {len(records)} screenshot/profile sequences')
    for record in records:
        print(record['fixture'],'OCC' if record['occupied'] else 'EMPTY',
              f"valid={record['confirmed_after_acquire']}/9 median_ms={record['desktop_median_ms']:.2f}",
              f"recovery={record['occupied_recover_calls']} samples={record['occupied_sample_calls']}")
    return report


if __name__=='__main__':validate()
