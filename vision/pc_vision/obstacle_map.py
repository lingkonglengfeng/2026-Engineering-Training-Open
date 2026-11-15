"""Read-only live map of controller-confirmed obstacle records."""
import copy
import math
import re


class ObstacleMapState:
    def __init__(self):
        self.session=None;self.rows={};self.car=None;self.phase=0;self.revision=0

    def receive(self,line):
        if not line.startswith(('[VOBS STATE]','[VOBS ACK]','[VOBS MAP]','[VOBS EVICT]')):return
        f=dict(re.findall(r'(\w+)=([^\s;]+)',line))
        try:
            session=int(f['session'])
            if not 0<=session<=0xffffffff:return
            state=line.startswith('[VOBS STATE]')
            obj=None if state else int(f.get('obstacle','0'))
            if not state and obj not in (1,2):return
            if not state and self.session is not None and session!=self.session:return
            if line.startswith('[VOBS ACK]') and f.get('result') not in ('WATCH','SLOW','DEFERRED','ACCEPTED','KEEP_NEARER','MERGED'):return
            def point(value,n=2):
                p=tuple(float(v) for v in value.split(','))
                if len(p)!=n or not all(math.isfinite(v) and 0<=v<=2400 for v in p):raise ValueError()
                return p
            if state:
                car=point(f['x']+','+f['y'])+(float(f['yaw']),)
                if not math.isfinite(car[2]):return
                phase=int(f.get('phase','0'));armed=int(f['armed'])
                if phase not in (0,1,2) or armed not in (0,1):return
            elif not line.startswith('[VOBS EVICT]'):
                mapped=line.startswith('[VOBS MAP]')
                plan=point(f['x']+','+f['y']) if mapped else point(f['plan'])
                raw=None if mapped else point(f['raw'])
                rect=point(f['rect'],4) if 'rect' in f else None
                if rect and (rect[0]>=rect[2] or rect[1]>=rect[3]):return
                radius=float(f.get('radius','50'))
                if not math.isfinite(radius) or not 0<=radius<=100:return
        except (KeyError,ValueError,OverflowError):return
        before=(self.session,copy.deepcopy(self.rows),self.car,self.phase)
        if session!=self.session:self.session=session;self.rows={};self.car=None;self.phase=0
        if state:
            self.car=car;self.phase=phase
            if not armed:self.rows.clear()
        elif line.startswith('[VOBS EVICT]'):self.rows.pop(obj,None)
        else:
            row=dict(self.rows.get(obj,{}));row.update(id=obj,site=f.get('site','P00'),plan=plan)
            if raw is not None:row['raw']=raw
            if mapped:row.update(mapped=True,rect=rect,radius=radius)
            else:row.setdefault('mapped',False)
            self.rows[obj]=row
        if before!=(self.session,self.rows,self.car,self.phase):self.revision+=1

    def snapshot(self):
        return dict(session=self.session,rows=copy.deepcopy(list(self.rows.values())),
                    car=self.car,phase=self.phase,revision=self.revision)


def draw_obstacles(canvas,xy,scale,snapshot):
    """Draw exact controller positions/barriers; no camera-to-map estimation."""
    for row in snapshot.get('rows',[]):
        color='#dc2626' if row.get('mapped') else '#d97706'
        rect=row.get('rect')
        if rect:
            canvas.create_rectangle(*xy(rect[0],rect[3]),*xy(rect[2],rect[1]),
                fill='#fecaca',stipple='gray25',outline=color,width=2,tags='live_obstacle')
        elif row.get('mapped'):
            x,y=xy(*row['plan']);r=row.get('radius',50)*scale
            canvas.create_oval(x-r,y-r,x+r,y+r,outline=color,dash=(3,2),tags='live_obstacle')
        x,y=xy(*row['plan']);raw=row.get('raw')
        if raw:
            rx,ry=xy(*raw);canvas.create_line(rx-4,ry,rx+4,ry,fill='#64748b',tags='live_obstacle')
            canvas.create_line(rx,ry-4,rx,ry+4,fill='#64748b',tags='live_obstacle')
            canvas.create_line(rx,ry,x,y,fill='#94a3b8',dash=(2,3),tags='live_obstacle')
        canvas.create_oval(x-7,y-7,x+7,y+7,outline=color,fill='white',width=2,tags='live_obstacle')
        canvas.create_text(x+10,y-11,anchor='sw',text='%s:%s %s'%(row['id'],row['site'],'已落图' if row.get('mapped') else '已确认'),fill=color,tags='live_obstacle')
    car=snapshot.get('car')
    if car:
        x,y,yaw=car;a=math.radians(yaw);c,s=math.cos(a),math.sin(a)
        corners=[xy(x+u*c-v*s,y+u*s+v*c) for u,v in ((140,140),(-140,140),(-140,-140),(140,-140))]
        canvas.create_polygon(*[v for p in corners for v in p],outline='#0891b2',fill='',width=2,tags='live_car')
        canvas.create_line(*xy(x,y),*xy(x+100*c,y+100*s),arrow='last',fill='#0891b2',width=3,tags='live_car')
