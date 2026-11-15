"""Numerical forecast checks on known paths. Does not connect to any device."""
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
import numpy as np
from motion_prediction import MotionPredictor


def run():
    records=[]
    for name,axes,omega in [('circle_cw',(60,60),.8),('circle_ccw',(60,60),-.8),('ellipse_cw',(85,45),.8)]:
        predictor=MotionPredictor({'enabled':True})
        rng=np.random.default_rng(42)
        times=[];errors=[];reports=[]
        for i in range(140):
            stamp=i/25
            truth=np.array([160,120])+np.array(axes)*[np.cos(omega*stamp),np.sin(omega*stamp)]
            point=truth+rng.normal(0,.3,2)
            started=time.perf_counter()
            report=predictor.update(point,stamp,1,0)
            times.append(1000*(time.perf_counter()-started))
            if report['future'] is not None:
                future_time=stamp+report['horizon_ms']/1000
                actual_future=np.array([160,120])+np.array(axes)*[np.cos(omega*future_time),np.sin(omega*future_time)]
                error=float(np.linalg.norm(actual_future-report['future']))
                if report['ready']:errors.append(error)
            reports.append(report)
        record={'case':name,'point_noise_sigma_px':.3,'input_fps':25,'horizon_ms':260,
                'median_model_ms':round(statistics.median(times[10:]),4),'p95_model_ms':round(float(np.percentile(times[10:],95)),4),
                'validated_forecasts':len(errors),'mean_true_path_error_px':round(statistics.mean(errors),4) if errors else None,
                'p95_true_path_error_px':round(float(np.percentile(errors,95)),4) if errors else None,
                'last_report':reports[-1]}
        records.append(record)
        print(name,'model median ms=',record['median_model_ms'],'numerical p95 error px=',record['p95_true_path_error_px'])
    output=ROOT/'output/motion_prediction_2026-10-17';output.mkdir(parents=True,exist_ok=True)
    (output/'validation.json').write_text(json.dumps({'limits':[
        'Known numerical trajectories with Gaussian measurement noise, not live camera measurements.',
        'Timing excludes vision, optical flow, network, UI and actuator delays.',
        'No actuator command or prediction coordinate is sent to STM32.'
    ],'cases':records},ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


if __name__=='__main__':run()
