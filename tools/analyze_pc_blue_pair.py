"""Repeatable blue-pair analysis on a saved raw photo; never connects to devices."""
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
    raw=cv2.imdecode(np.fromfile(str(data/'blue_lightblue_raw.png'),np.uint8),cv2.IMREAD_COLOR)
    samples=json.loads((data/'blue_lightblue_calibration.json').read_text())['samples']
    output=ROOT/'output/blue_pair_2026-10-17';output.mkdir(parents=True,exist_ok=True)
    records=[]
    for gain,quality in ((1.,100),(.65,100),(.8,100),(1.25,100),(1.5,100),(1.,60),(1.,85)):
        changed=np.clip(raw.astype(np.float32)*gain,0,255).astype(np.uint8)
        if quality<100:
            encoded=cv2.imencode('.jpg',changed,[cv2.IMWRITE_JPEG_QUALITY,quality])[1]
            changed=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
        image=cv2.resize(changed,(320,240),interpolation=cv2.INTER_AREA)
        analyzer=SurfaceAnalyzer(material);times=[]
        for i in range(8):
            start=time.perf_counter();scene=analyzer.process(image,samples,3,100+i/30,False,changed)
            times.append((time.perf_counter()-start)*1000)
        fields=('color','center','source','reason','quality','stable')
        record={'gain':gain,'jpeg_quality':quality,'median_ms':statistics.median(times[2:]),
                'colors':scene['color_regions'],'surfaces':[{k:c[k] for k in fields} for c in scene['surfaces']]}
        records.append(record)
        if gain==1. and quality==100:
            corrected=analyzer.annotate(image,scene)
            cv2.imencode('.png',cv2.resize(corrected,(960,720),interpolation=cv2.INTER_NEAREST))[1].tofile(str(output/'corrected.png'))
            original=ROOT/'vision/pc_vision/screenshots/20261017_164558_3128_overlay.png'
            if original.exists():
                before=cv2.imdecode(np.fromfile(str(original),np.uint8),cv2.IMREAD_COLOR)
                comparison=np.hstack((cv2.resize(before,(640,480)),cv2.resize(corrected,(640,480))))
                cv2.imencode('.png',comparison)[1].tofile(str(output/'before_after.png'))
        print('gain',gain,'jpeg',quality,'regions',[c['color'] for c in scene['color_regions']],
              'faces',[(c['color'],tuple(round(v,1) for v in c['center']),c['reason']) for c in scene['surfaces']],
              'median_ms',round(record['median_ms'],2))
    result={'input':'20261017_164558_3128_raw.png','records':records,
            'limitations':['One still photo; not dynamic recognition or grasp success rate.',
                           'Digital brightness and JPEG variants are sensitivity checks only.',
                           'Quality and vote fractions are not calibrated probabilities.']}
    (output/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':run()
