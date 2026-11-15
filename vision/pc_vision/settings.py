"""Shared desktop settings; avoid importing a generic main.py from camera code."""
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
DEFAULT={"host":"","video_port":9000,"control_port":9001,"light_port":9004,
         "calibration_file":"calibration.json","reconnect_seconds":2.0,"camera_role":"material",
         "fill_light":None}
MOTION_DEFAULTS={'enabled':False,'lead_ms':180.,'latency_ms':80.,'grasp_x':160.,'grasp_y':100.,
                 'half_width':105.,'half_height':100.,'error_limit_px':6.}


def motion_settings(value=None):
    if value is not None and not isinstance(value,dict):raise ValueError('运动预测配置应为对象')
    result=dict(MOTION_DEFAULTS,**(value or {}))
    if not isinstance(result['enabled'],bool):raise ValueError('enabled应为布尔值')
    limits={'lead_ms':(0,500),'latency_ms':(0,300),'grasp_x':(0,319),'grasp_y':(0,239),
            'half_width':(1,160),'half_height':(1,120),'error_limit_px':(.5,30)}
    for key,(lower,upper) in limits.items():
        number=result[key]
        if isinstance(number,bool) or not isinstance(number,(int,float)) or not math.isfinite(number) or not lower<=number<=upper:
            raise ValueError('%s应在%s～%s之间'%(key,lower,upper))
        result[key]=float(number)
    if result['lead_ms']+result['latency_ms']>600:raise ValueError('动作时间与图像延迟合计暂不超过600ms')
    return {key:result[key] for key in MOTION_DEFAULTS}


def read_config(path):
    config=DEFAULT.copy()
    if path.exists():config.update(json.loads(path.read_text(encoding="utf-8-sig")))
    for key in ("video_port","control_port","light_port"):
        if not isinstance(config[key],int) or isinstance(config[key],bool) or not 1<=config[key]<=65535:raise ValueError("Invalid "+key)
    if not isinstance(config["host"],str) or not isinstance(config["calibration_file"],str):raise ValueError("Invalid host/calibration_file")
    if config['camera_role'] not in ('material','qr'):raise ValueError('camera_role must be material or qr')
    light=config['fill_light']
    if light is not None:
        if isinstance(light,bool) or not isinstance(light,int) or not 0<=light<=100:
            raise ValueError('fill_light must be null (auto) or an integer 0..100')
    recent=config.get('recent_hosts',[])
    if not isinstance(recent,list) or any(not isinstance(host,str) for host in recent):raise ValueError('recent_hosts must be a list of addresses')
    config['recent_hosts']=list(dict.fromkeys(host for host in recent if host))[:8]
    delay=config["reconnect_seconds"]
    if not isinstance(delay,(int,float)) or not .1<=delay<=60:raise ValueError("reconnect_seconds must be 0.1..60")
    config['motion']=motion_settings(config.get('motion'))
    return config
