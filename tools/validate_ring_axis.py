"""生成可复查的三环方向/黄金帧记录；截图不是角度真值标定。"""
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_ring'),str(ROOT/'vision/maixcam_service'),str(ROOT/'vision')]
import cv2
import numpy as np
import middle_ring as ring
import service


def run():
    out=ROOT/'output/ring_yaw_2026-10-07'
    out.mkdir(exist_ok=True)
    cv2.setNumThreads(1)
    records=[]
    for filename in ('live_complete.png','occupied_3_cleaned.png','occupied_4_cleaned.png','occupied_5_cleaned.png'):
        gray=cv2.imdecode(np.fromfile(str(ROOT/'vision/maixcam_ring/test_data'/filename),np.uint8),0)
        detector,tracker=ring.RingDetector(),ring.MiddleTracker()
        detector.require_scene=True
        for i in range(3):
            groups,triple,_,status=detector.process(gray,tracker,i*.08)
            observation=tracker.update(groups,triple,i*.08,status)
            detector.accept(observation,triple,gray,i*.08)
        timing=[]
        for _ in range(1000):
            start=time.perf_counter()
            axis=ring.ring_axis_measurement(groups,triple,observation)
            timing.append(1000*(time.perf_counter()-start))
        records.append(dict(image=filename,axis=axis,desktop_axis_median_ms=statistics.median(timing),
            axis_frame=service.encode_ring_axis(axis,5).hex(' ').upper()))
    report=dict(hardware_run=False,images_are_cleaned_screenshots=True,
        limits=['图像轴角不是车体应转角','必须现场教正对零位与方向比例','截图无真实角度真值，不用于机械精度结论'],results=records)
    (out/'axis_photo_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(records,indent=2))


if __name__=='__main__':run()
