"""Read-only desktop hotspot probe on the user's new occupied-ring frame."""
import cProfile
import io
from pathlib import Path
import pstats
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_ring')]
import cv2
import numpy as np
import middle_ring as ring


def run():
    path=Path('C:/Users/14449/AppData/Local/Temp/codex-clipboard-1dd7847d-f251-4a02-b404-8c1759dbbb91.png')
    gray=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
    print('image shape:',gray.shape)
    cv2.setNumThreads(1)
    detector,tracker=ring.RingDetector(),ring.MiddleTracker();detector.require_scene=True
    times=[];modes=[];counts=[];positions=[];stats={};profile=cProfile.Profile()
    profile.enable()
    for index in range(40):
        start=time.perf_counter();now=index*.1
        groups,triple,_,status=detector.process(gray,tracker,now,stats)
        observation=tracker.update(groups,triple,now,status)
        detector.accept(observation,triple,gray,now)
        times.append((time.perf_counter()-start)*1000)
        modes.append(stats['mode']);counts.append(len(groups))
        positions.append(observation['position'] if observation else None)
    profile.disable()
    print('desktop median_ms:',statistics.median(times),'modes:',{m:modes.count(m) for m in set(modes)},
          'counts:',set(counts),'last_position:',positions[-1])
    text=io.StringIO();pstats.Stats(profile,stream=text).sort_stats('cumtime').print_stats(16)
    print(text.getvalue())


if __name__=='__main__':run()
