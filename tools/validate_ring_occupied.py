"""截图覆盖层统一清理后的前后比较；不等同原始帧或设备帧率。"""
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'), str(ROOT/'vision/maixcam_ring')]
import cv2
import numpy as np
import middle_ring as new
cv2.setNumThreads(1)
OUT=ROOT/'output/ring_occupied_2026-10-07'


def run():
    spec=importlib.util.spec_from_file_location('before_occupied_ring',OUT/'before_middle_ring.py')
    old=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    records=[]
    for index,top,bottom,row in ((1,4,269,123),(2,7,271,125),(3,0,262,117)):
        source=cv2.imdecode(np.fromfile(str(OUT/('screenshot_%d.png'%index)),np.uint8),cv2.IMREAD_COLOR)
        gray=cv2.cvtColor(source,cv2.COLOR_BGR2GRAY)
        mask=(source.max(2).astype(int)-source.min(2).astype(int)>12).astype(np.uint8)*255
        mask[row,:]=255
        mask[:,224:226]=255
        gray=cv2.inpaint(gray,mask,2,cv2.INPAINT_NS)[top:bottom]
        cv2.imencode('.png',gray)[1].tofile(str(OUT/('clean_%d.png'%index)))
        images=[]
        comparison=[]
        for label,module in (('BEFORE',old),('AFTER',new)):
            groups,triple,_,status=module.detect_rings(gray)
            observation=None if triple is None else dict(ring=triple['ring'],angle=triple['angle'],angle_valid=True,score=triple['score'])
            canvas=module.annotate(gray,groups,observation,status=status)
            header=np.zeros((28,gray.shape[1],3),np.uint8)
            cv2.putText(header,'%s G=%d %s'%(label,len(groups),'NO MIDDLE' if triple is None else 'MIDDLE OK'),(6,19),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
            images.append(np.vstack((header,canvas)))
            timings=[]
            for _ in range(25):
                start=time.perf_counter()
                module.detect_rings(gray)
                timings.append(1000*(time.perf_counter()-start))
            comparison.append(dict(groups=len(groups),middle=None if triple is None else triple['ring']['center'],
                occupied=sum(g.get('occluded',False) for g in groups),partial=sum(g.get('partial',False) for g in groups),desktop_median_ms=statistics.median(timings)))
        cv2.imencode('.png',np.hstack(images))[1].tofile(str(OUT/('comparison_%d.png'%index)))
        records.append(dict(image=index,before=comparison[0],after=comparison[1]))
    report=dict(results=records,hardware_run=False,limits=[
        '原图为带十字/彩色标记的截图；两算法输入相同清理帧，无法替代原始相机验证',
        '第1张仅2组当前证据，位置仍无效；不会凭历史/中点补出不存在的第三组',
        '桌面耗时不能推算MaixCAM帧率；遮挡分支比原完整环路径增加工作'])
    (OUT/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':run()
