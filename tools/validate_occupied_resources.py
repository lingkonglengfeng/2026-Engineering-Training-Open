"""Frozen1.1.14 comparison on identical frames; desktop CPU/allocation metrics only."""
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import statistics
import sys
import time
import tracemalloc
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_material'),
             str(ROOT/'vision/maixcam_ring'),str(ROOT/'vision/maixcam_service'),str(ROOT/'vision/maixcam_light')]
import cv2
import numpy as np
import middle_ring as new
OUT=ROOT/'output/ring_occupied_opt_2026-10-15'


def run_sequence(module,images,occupied):
    detector,tracker=module.RingDetector(),module.MiddleTracker()
    detector.require_scene=True;detector.allow_occupied=occupied
    times=[];positions=[];counts=[];modes=[]
    for index,frame in enumerate(images):
        now=index*.08;stats={};start=time.perf_counter()
        groups,triple,binary,status=detector.process(frame,tracker,now,stats)
        if detector.identity_rejected and triple is None:tracker.reset()
        observed=tracker.update(groups,triple,now,status);detector.accept(observed,triple,frame,now)
        times.append(1000*(time.perf_counter()-start))
        positions.append(list(observed['ring']['center']) if observed and len(groups)==3 else None)
        counts.append(len(groups));modes.append(stats['mode'])
    return dict(desktop_median_ms=statistics.median(times),desktop_p95_ms=sorted(times)[math.ceil(.95*len(times))-1],
                positions=positions,counts=counts,modes=modes)


def validate():
    cv2.setNumThreads(1);OUT.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('frozen_ring_114',OUT/'before_middle_ring.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    records=[]
    for name,occupied in [('live_complete',False),('live_cropped',False),
        *[(f'occupied_{index}_cleaned',True) for index in (2,3,4,5)],('occupied_6_live',True)]:
        source=cv2.imdecode(np.fromfile(str(ROOT/'vision/maixcam_ring/test_data'/f'{name}.png'),np.uint8),0)
        images=[]
        for index in range(40):
            transform=np.float32([[1,0,1.2*math.sin(index*.6)],[0,1,math.cos(index*.5)]])
            frame=cv2.warpAffine(source,transform,(source.shape[1],source.shape[0]),borderValue=180)
            images.append(np.clip(frame.astype(np.float32)*(1+.08*math.sin(index*.7)),0,255).astype(np.uint8))
        # Two alternating orders reduce warmup/order bias; both use identical frames.
        trials=[(run_sequence(old,images,occupied),run_sequence(new,images,occupied))]
        newer=run_sequence(new,images,occupied);older=run_sequence(old,images,occupied);trials.append((older,newer))
        before,after=trials[-1]
        assert before['counts']==after['counts'],(name,'count changed')
        assert [p is not None for p in before['positions']]==[p is not None for p in after['positions']],(name,'validity changed')
        differences=[math.dist(a,b) for a,b in zip(before['positions'],after['positions']) if a and b]
        max_delta=max(differences,default=0.)
        assert max_delta<.2,(name,max_delta)
        if not occupied:assert max_delta==0.,(name,'empty path changed')
        records.append(dict(fixture=name,occupied=occupied,frames=40,
            valid_after_acquire=sum(p is not None for p in after['positions'][3:]),frames_after_acquire=37,
            max_center_difference_px=max_delta,
            before_ms=statistics.median(t[0]['desktop_median_ms'] for t in trials),
            after_ms=statistics.median(t[1]['desktop_median_ms'] for t in trials),
            before_p95_ms=before['desktop_p95_ms'],after_p95_ms=after['desktop_p95_ms']))
        if name=='occupied_6_live':
            pictures=[]
            for module,label in ((old,'FROZEN 1.1.14'),(new,'OCCUPIED OPT')):
                groups,triple,_,status=module.detect_rings(source)
                shown=module.annotate(source,groups,None if triple is None else {'ring':triple['ring']},status=label)
                pictures.append(shown)
            cv2.imencode('.png',np.hstack(pictures))[1].tofile(str(OUT/'latest_photo_comparison.png'))
    allocations=[]
    source=cv2.imdecode(np.fromfile(str(ROOT/'vision/maixcam_ring/test_data/occupied_6_live.png'),np.uint8),0)
    for label,module in (('before',old),('after',new)):
        tracemalloc.start();run_sequence(module,[source]*40,True)
        current,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        allocations.append(dict(version=label,python_numpy_traced_peak_bytes=peak))
    log=io.StringIO()
    with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        loader=unittest.TestLoader()
        suite=unittest.TestSuite(loader.discover(str(ROOT/p),pattern='test*.py',top_level_dir=str(ROOT/p))
            for p in ('vision/maixcam_material','vision/maixcam_ring','vision/maixcam_service','vision/maixcam_light'))
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    (OUT/'tests.log').write_text(log.getvalue(),encoding='utf-8')
    assert result.wasSuccessful(),log.getvalue()[-5000:]
    report=dict(build='2026-10-15-occupied-resources-v30',baseline='frozen CAM1 1.1.14 ring source',
                tests=result.testsRun,failures=0,errors=0,records=records,allocations=allocations,hardware_tested=False,
                limits=['desktop timings cannot prove MaixCAM FPS','synthetic perturbations, not captured video',
                        'tracemalloc excludes much native OpenCV/SDK memory and is not device RAM measurement',
                        'sampling density/layers/thresholds/resolution/3-ring qualification unchanged'])
    (OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('%d tests PASS'%result.testsRun)
    for row in records:print(row['fixture'],round(row['before_ms'],3),'->',round(row['after_ms'],3),'ms',
                           'max_delta',round(row['max_center_difference_px'],5))
    print(allocations)
    return report


if __name__=='__main__':validate()
