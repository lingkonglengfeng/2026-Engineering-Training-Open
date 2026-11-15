"""复现两张带覆盖层补光截图的离线对比；不模拟PWM后的真实画面。"""
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_ring')]
import cv2
import numpy as np
import middle_ring as new
cv2.setNumThreads(1)
OUT=ROOT/'output/ring_glare_2026-10-05'


def run():
    for index,top,bottom,row,col,width in ((1,59,325,178,225,450),(2,53,317,171,217,442)):
        source=cv2.imdecode(np.fromfile(str(OUT/('screenshot_%d.png'%index)),np.uint8),cv2.IMREAD_COLOR)
        gray=cv2.cvtColor(source,cv2.COLOR_BGR2GRAY)
        mask=(source.max(2).astype(int)-source.min(2).astype(int)>12).astype(np.uint8)*255
        mask[row,:]=255;mask[:,col:col+2]=255
        cleaned=cv2.inpaint(gray,mask,2,cv2.INPAINT_NS)[top:bottom,:width]
        target=ROOT/'vision/maixcam_ring/test_data'/('live_glare_%d_cleaned.png'%index)
        cv2.imencode('.png',cleaned)[1].tofile(str(target))
    spec=importlib.util.spec_from_file_location('before_ring_glare',OUT/'before_middle_ring.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    results=[]
    for name,filename in [('glare_1','live_glare_1_cleaned.png'),('glare_2','live_glare_2_cleaned.png'),
                          ('old_complete','live_complete.png'),('old_partial','live_cropped.png')]:
        path=ROOT/'vision/maixcam_ring/test_data'/filename
        gray=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)
        records=[];pictures=[]
        for module in (old,new):
            groups,triple,_,status=module.detect_rings(gray)
            observation=None if triple is None else dict(ring=triple['ring'],angle=triple['angle'],angle_valid=True,score=triple['score'])
            error=None if observation is None else module.screen_error(observation['ring']['center'],
                (gray.shape[1],gray.shape[0]),(gray.shape[1],gray.shape[0]))
            pictures.append(module.annotate(gray,groups,observation,status=status,error=error))
            timings=[]
            for i in range(45):
                start=time.perf_counter();module.detect_rings(gray);timings.append(1000*(time.perf_counter()-start))
            records.append(dict(groups=len(groups),middle=None if triple is None else triple['ring']['center'],
                weak=any(g.get('weak_recovered',False)for g in groups),desktop_median_ms=statistics.median(timings)))
        cv2.imencode('.png',np.hstack(pictures))[1].tofile(str(OUT/(name+'_before_after.png')))
        results.append(dict(name=name,before=records[0],after=records[1]))
    report=dict(results=results,hardware_run=False,limits=[
        '截图十字与彩色覆盖按同一规则清理，不是原始传感器帧',
        '这不是PWM调光后的实拍结果，桌面耗时不能推算设备FPS'])
    (OUT/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))


if __name__=='__main__':run()
