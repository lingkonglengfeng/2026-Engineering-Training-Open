"""同一截图清理帧的v13/v14位移/亮度序列对比；不推算设备FPS。"""
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_ring')]
import cv2
import numpy as np
import middle_ring as new
OUT=ROOT/'output/ring_stability_2026-10-07'


def run():
    cv2.setNumThreads(1)
    spec=importlib.util.spec_from_file_location('before_ring_stability',OUT/'before_middle_ring.py')
    old=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    records=[]
    for index in (2,3,4,5):
        source=cv2.imdecode(np.fromfile(str(ROOT/'vision/maixcam_ring/test_data'/('occupied_%d_cleaned.png'%index)),np.uint8),0)
        pictures=[]
        for version,module in [('v13',old),('v14',new)]:
            detector,tracker=module.RingDetector(),module.MiddleTracker()
            detector.require_scene=True
            times,valid,modes,counts,positions=[],[],[],[],[]
            for i in range(30):
                matrix=np.float32([[1,0,1.2*math.sin(i*.6)],[0,1,math.cos(i*.5)]])
                image=cv2.warpAffine(source,matrix,(source.shape[1],source.shape[0]),borderValue=180)
                image=np.clip(image.astype(np.float32)*(1+.08*math.sin(i*.7)),0,255).astype(np.uint8)
                now=i*.08
                stats={}
                start=time.perf_counter()
                groups,triple,binary,status=detector.process(image,tracker,now,stats)
                times.append(1000*(time.perf_counter()-start))
                if detector.identity_rejected and triple is None:
                    tracker.reset()
                observed=tracker.update(groups,triple,now,status)
                detector.accept(observed,triple,image,now)
                valid.append(observed is not None and len(groups)==3)
                counts.append(len(groups))
                modes.append(stats['mode'])
                positions.append(None if observed is None else observed['ring']['center'])
            records.append(dict(image=index,version=version,valid_after_acquire=sum(valid[3:]),
                total_after_acquire=27,counts=counts,desktop_median_ms=statistics.median(times),
                fast_frames=sum(x=='ARC3_FAST' for x in modes),positions=positions))
            groups,triple,_,status=module.detect_rings(source)
            observation=None if triple is None else dict(ring=triple['ring'])
            canvas=module.annotate(source,groups,observation,status=status)
            header=np.zeros((28,source.shape[1],3),np.uint8)
            cv2.putText(header,'%s G=%d %s'%(version,len(groups),'MIDDLE OK' if triple is not None else 'NO MIDDLE'),(6,19),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
            pictures.append(np.vstack((header,canvas)))
        cv2.imencode('.png',np.hstack(pictures))[1].tofile(str(OUT/('comparison_%d.png'%index)))
    report=dict(hardware_run=False,synthetic_variations=True,variation='translation <=1.2px, brightness +/-8%',
        limits=['两版本输入相同清理截图，不能把离线旧版结果等同原屏幕实时结果','合成扰动序列不是实拍视频','桌面耗时不能换算MaixCAM FPS'],results=records)
    (OUT/'sequence_comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for row in records:
        print({k:v for k,v in row.items() if k not in ('positions','counts')})


if __name__=='__main__':run()
