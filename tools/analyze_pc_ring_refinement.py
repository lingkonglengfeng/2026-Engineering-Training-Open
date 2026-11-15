"""Inspect current-frame ring fits on archived screenshots; no device IO."""
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
import cv2
import numpy as np
from backend import ring
from ring_refinement import refine_occupied


def run():
    cv2.setNumThreads(1)
    output=ROOT/'output/occupied_joint_fit_2026-10-17';output.mkdir(parents=True,exist_ok=True)
    records=[]
    for path in sorted((ROOT/'vision/pc_vision/screenshots').glob('20261017_0409*.png')):
        bgr=cv2.imdecode(np.fromfile(str(path),np.uint8),1);gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        occupied=path.name>='20261017_040936'
        detector,tracker=ring.RingDetector(),ring.MiddleTracker()
        detector.allow_occupied,detector.require_scene=occupied,True
        for i in range(5):
            stamp=100+i/30
            groups,triple,_,status=detector.process(gray,tracker,stamp)
            observed=tracker.update(groups,triple,stamp,status)
            detector.accept(observed,triple,gray,stamp)
        measured=[];view=bgr.copy();started=time.perf_counter()
        for group in groups:
            candidate,report=refine_occupied(gray,bgr,group,ring)
            measured.append(dict(report,baseline_center=group['center'],refined_center=candidate['center'] if candidate else None))
            if candidate:
                cv2.drawMarker(view,tuple(round(v) for v in candidate['center']),(255,255,0),cv2.MARKER_CROSS,13,1)
        elapsed=(time.perf_counter()-started)*1000
        cv2.imencode('.png',view)[1].tofile(str(output/(path.stem+'_joint.png')))
        records.append({'input':str(path.relative_to(ROOT)),'occupied':occupied,'groups':len(groups),'status':status,'refine_ms':elapsed,'measurements':measured})
        print(path.name,status,[(item['reason'],round(item.get('shift_px',0),2),round(item.get('p95_px',0),2)) for item in measured])
    (output/'analysis.json').write_text(json.dumps({'limits':['Old screenshots contain overlays; cannot establish true center error or live jitter.','Only current-frame ring-line samples support refinement; top material center is never used.'],'records':records},ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':run()
