"""Per-point rate protocol. Editing or parsing never sends motor commands."""
import copy
import math
import re

POINTS=(('wait','圆盘：等待物料'),('grab','圆盘：夹取物料'),('tray','放回载物盘'),
        ('traypick','载物盘取料'),('ring1','圆环1号（ROUGH/TEMP共用）'),
        ('ring2','圆环2号（ROUGH/TEMP共用）'),('ring3','圆环3号（ROUGH/TEMP共用）'))
AXES=(('lift','5号升降'),('reach','6号伸缩'),('dm','DM旋转'),('grip','夹爪'),('table','载物盘'))

def rate_command(point, values):
    if point not in dict(POINTS):raise ValueError('未知机械臂速率点位')
    values=[float(v) for v in values]
    if len(values)!=5 or any(not math.isfinite(v) or not 10<=v<=300 for v in values):
        raise ValueError('五路速率必须为10%～300%的有限数值')
    return 'arm rates set '+point+' '+' '.join('%.1f'%v for v in values)

def timeout_factor(snapshot):
    records=(snapshot.get('rates') or {}).get('points',{})
    values=[v for row in records.values() for v in row.values() if isinstance(v,(int,float)) and math.isfinite(v) and 10<=v<=300]
    return 100./min([100.]+values)

class RateState:
    def __init__(self):self.data={};self.pending=None
    def receive(self,line,now):
        if line.startswith('[ARM RATES BEGIN]'):
            h=dict(re.findall(r'(\w+)=([^\s]+)',line))
            self.pending={'header':h,'points':{}} if h.get('v')=='1' else None
        elif line.startswith('[ARM RATE]') and self.pending is not None:
            h=dict(re.findall(r'(\w+)=([^\s]+)',line));p=h.get('point')
            try:
                values={key:float(h[key]) for key,_ in AXES}
                if p in dict(POINTS) and all(math.isfinite(v) and 10<=v<=300 for v in values.values()):self.pending['points'][p]=values
            except (ValueError,KeyError):pass
        elif line=='[ARM RATES END]':
            if self.pending is not None and set(self.pending['points'])==set(dict(POINTS)):
                self.data=self.pending;self.data['received']=now
            self.pending=None
    def snapshot(self):return copy.deepcopy(self.data)
