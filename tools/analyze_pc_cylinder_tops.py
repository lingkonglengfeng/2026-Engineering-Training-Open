"""Offline current-frame rim validation on the user's saved cylinder photographs."""
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
import cv2
import numpy as np
from backend import material
from material_surface import SurfaceAnalyzer


def run():
    cv2.setNumThreads(1)
    data=ROOT/'vision/pc_vision/test_data'
    cases=json.loads((data/'cylinder_top_cases.json').read_text(encoding='utf-8'))
    out=ROOT/'output/cylinder_tops_2026-10-26';out.mkdir(parents=True,exist_ok=True)
    records=[];comparisons=[]
    for case in cases['cases']:
        raw=cv2.imdecode(np.fromfile(str(data/case['image']),np.uint8),1)
        image=cv2.resize(raw,(320,240),interpolation=cv2.INTER_AREA)
        analyzer=SurfaceAnalyzer(material);times=[]
        for i in range(7):
            start=time.perf_counter();scene=analyzer.process(image,cases['samples'],case['color'],100+i/30,False,raw)
            times.append((time.perf_counter()-start)*1000)
        target=scene['target']
        report=analyzer.describe(scene,case['color'])
        report.update(image=case['image'],median_ms=statistics.median(times[2:]),
                      raw_center=(np.array(target['center'])*1.5).tolist() if target else None,
                      old_center=(np.array(case['previous_center_processing_px'])*1.5).tolist())
        records.append(report)
        before=raw.copy();old=tuple(round(v) for v in report['old_center'])
        cv2.drawMarker(before,old,(0,180,255),cv2.MARKER_CROSS,21,2)
        cv2.putText(before,'OLD center',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,180,255),2)
        after=cv2.resize(analyzer.annotate(image,scene),(480,360))
        cv2.putText(after,'NEW measured rim',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,220,80),2)
        comparison=np.hstack((before,after));comparisons.append(comparison)
        cv2.imencode('.png',comparison)[1].tofile(str(out/(Path(case['image']).stem+'_comparison.png')))
        print(case['image'],scene['reason'],'center',report['raw_center'],'median_ms',round(report['median_ms'],2))
    cv2.imencode('.png',np.vstack((comparisons[0],comparisons[-1])))[1].tofile(str(out/'comparison.png'))
    # Degraded inputs may remain unconfirmed; they must not revive whole-body centers.
    variants=[]
    for case in cases['cases']:
        raw=cv2.imdecode(np.fromfile(str(data/case['image']),np.uint8),1)
        for gain,quality in ((.8,85),(1.2,85),(1.,60)):
            modified=np.clip(raw.astype(np.float32)*gain,0,255).astype(np.uint8)
            modified=cv2.imdecode(cv2.imencode('.jpg',modified,[cv2.IMWRITE_JPEG_QUALITY,quality])[1],1)
            image=cv2.resize(modified,(320,240),interpolation=cv2.INTER_AREA)
            analyzer=SurfaceAnalyzer(material)
            for i in range(4):scene=analyzer.process(image,cases['samples'],case['color'],100+i/30,False,modified)
            target=scene['target']
            variants.append({'image':case['image'],'gain':gain,'jpeg_quality':quality,'status':scene['reason'],
                             'raw_center':(np.array(target['center'])*1.5).tolist() if target else None})
    (out/'analysis.json').write_text(json.dumps({'records':records,'variants':variants,
        'limits':['Static replay and numeric transforms do not measure live-video accuracy or grasp success.',
                  'Original centers were wrong detections, not calibration ground truth.',
                  'Geometric and color scores are heuristics, not correctness probabilities.']},ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':run()
