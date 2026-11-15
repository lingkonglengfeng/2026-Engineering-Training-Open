"""Run visible-face analysis on saved photos without opening network/device sessions."""
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
    output=ROOT/'output/material_surface_2026-10-17';output.mkdir(parents=True,exist_ok=True)
    paths=sorted((ROOT/'vision/pc_vision/screenshots').glob('20261017_035*.png'))
    paths += sorted((ROOT/'vision/maixcam_material/test_data').glob('materials_overview_*.png'))
    samples=material.load_samples(str(ROOT/'vision/pc_vision/calibration.json'))
    records=[]
    for path in paths:
        bgr=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
        analyzer=SurfaceAnalyzer(material)
        times=[]
        for i in range(8):
            start=time.perf_counter()
            scene=analyzer.process(bgr,samples,stamp=100+i/30)
            times.append((time.perf_counter()-start)*1000)
        preview=analyzer.annotate(bgr,scene)
        cv2.imencode('.png',preview)[1].tofile(str(output/(path.stem+'_surfaces.png')))
        fields=('center','axes','quality','reason','color','source','purity','residual','edge_support','uniformity')
        record={'input':str(path.relative_to(ROOT)),'median_ms':round(statistics.median(times[2:]),3),
                'surfaces':[{key:item[key] for key in fields} for item in scene['surfaces']]}
        records.append(record)
        print(path.name,round(record['median_ms'],2),[(item['color'],tuple(round(v,1) for v in item['center']),round(item['quality'],2),item['reason'],item['source']) for item in scene['surfaces']])
    (output/'analysis.json').write_text(json.dumps({'limits':['Static photos do not validate dynamic success rate.','Old screenshots include prior green overlays.','Scores are heuristics, not calibrated probabilities.'],'records':records},ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':run()
