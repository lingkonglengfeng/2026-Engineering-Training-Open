"""Blue parking reference from visible top edge, independent of occluded bottom."""
import json,math
from pathlib import Path
import cv2
import numpy as np
REFERENCE=Path(__file__).with_name('parking_reference.json')

def features(bgr):
    im=cv2.resize(bgr,(480,360),interpolation=cv2.INTER_AREA)
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array((100,65,25)),np.array((140,255,255)))
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    candidates=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>1200 and stats[i,cv2.CC_STAT_WIDTH]>120 and stats[i,cv2.CC_STAT_TOP]>35]
    if not candidates:return None
    i=max(candidates,key=lambda i:stats[i,cv2.CC_STAT_AREA]);x,y,w,h=stats[i,:4]
    if x<=2 or x+w>=478:return None
    points=[]
    for u in range(x,x+w):
        rows=np.flatnonzero(labels[:,u]==i)
        if len(rows):points.append((u,int(rows[0])))
    pts=np.array(points,np.float32)
    pts=pts[pts[:,1]<np.percentile(pts[:,1],35)+10]
    if len(pts)<100:return None
    for _ in range(3):
        vx,vy,px,py=cv2.fitLine(pts,cv2.DIST_HUBER,0,.01,.01).reshape(-1)
        if abs(vx)<.8:return None
        m=float(vy/vx);b=float(py-m*px)
        pts=pts[np.abs(pts[:,1]-m*pts[:,0]-b)<3]
        if len(pts)<100:return None
    left,right=np.percentile(pts[:,0],(1,99));cx=float((left+right)/2);top=float(m*cx+b)
    if right-left<120 or abs(m)>.35:return None
    return dict(center_x=cx,top_y=top,left=float(left),right=float(right),angle=-math.degrees(math.atan(m)),width=float(right-left),points=len(pts))

class ParkingDetector:
    def __init__(self):self.reference=json.loads(REFERENCE.read_text(encoding='utf-8'))
    def observe(self,bgr):
        im=cv2.resize(bgr,(480,360),interpolation=cv2.INTER_AREA);out=im.copy();f=features(im);r=self.reference
        cv2.line(out,(round(r['left']),round(r['top_y'])),(round(r['right']),round(r['top_y'])),(255,150,0),1)
        report=dict(valid=False,reason='BLUE_EDGE_MISSING_OR_CLIPPED',reference=r)
        if not f:return None,None,out,report
        dx,dy=f['center_x']-r['center_x'],f['top_y']-r['top_y'];angle=f['angle']-r['angle']
        report.update(valid=True,reason='BLUE_EDGE',features=f,dx_px=dx,dy_px=dy,angle_deg=angle)
        m=-math.tan(math.radians(f['angle']))
        a=(round(f['left']),round(f['top_y']+m*(f['left']-f['center_x'])))
        b=(round(f['right']),round(f['top_y']+m*(f['right']-f['center_x'])))
        cv2.line(out,a,b,(0,255,0),2);cv2.circle(out,(round(f['center_x']),round(f['top_y'])),4,(0,255,255),-1)
        cv2.putText(out,'PARK dx=%+.1f dy=%+.1f yaw=%+.2f'%(dx,dy,angle),(8,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
        axis=dict(degrees=angle,confidence=95,source=3)
        return (dx,-dy),axis,out,report # Existing protocol encoder converts screen-up to image-down.
