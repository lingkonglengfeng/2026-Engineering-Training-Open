"""Experimental image-orbit forecasting. Reports only; never generates actuator commands."""
import collections
import math

import cv2
import numpy as np

from settings import motion_settings


class Orbit:
    def __init__(self,center,axes,angle,kind,rms):
        self.center=np.asarray(center,float)
        self.axes=np.asarray(axes,float)
        cosine,sine=math.cos(angle),math.sin(angle)
        self.rotation=np.array([[cosine,-sine],[sine,cosine]])
        self.kind,self.rms=kind,float(rms)

    def phase(self,points):
        local=(np.asarray(points)-self.center)@self.rotation/self.axes
        return np.arctan2(local[...,1],local[...,0])

    def position(self,phase):
        phase=np.asarray(phase)
        local=np.stack((np.cos(phase),np.sin(phase)),axis=-1)*self.axes
        return local@self.rotation.T+self.center

    def distance(self,points):
        local=(np.asarray(points)-self.center)@self.rotation/self.axes
        return np.abs(np.linalg.norm(local,axis=-1)-1)*math.sqrt(float(np.prod(self.axes)))


def circle_orbit(points):
    points=np.asarray(points,float)
    origin=points.mean(0);scale=max(1.,float(np.std(points)))
    local=(points-origin)/scale
    design=np.column_stack((2*local,np.ones(len(local))))
    if np.linalg.cond(design)>500:return None
    params,_,rank,_=np.linalg.lstsq(design,np.sum(local*local,axis=1),rcond=None)
    if rank<3:return None
    radius_squared=float(params[2]+params[:2]@params[:2])
    if radius_squared<=0:return None
    center=origin+params[:2]*scale;radius=math.sqrt(radius_squared)*scale
    for _ in range(5):
        delta=center-points;distance=np.linalg.norm(delta,axis=1)
        if np.any(distance<1e-4):return None
        residual=distance-radius
        weight=np.minimum(1,1.2/np.maximum(np.abs(residual),.01))
        jacobian=np.column_stack((delta/distance[:,None],-np.ones(len(points))))
        step,_,rank,_=np.linalg.lstsq(jacobian*np.sqrt(weight[:,None]),-residual*np.sqrt(weight),rcond=None)
        if rank<3 or np.linalg.norm(step)>max(20,radius):return None
        center+=step[:2];radius+=step[2]
        if np.linalg.norm(step)<1e-4:break
    if not 8<=radius<=640:return None
    orbit=Orbit(center,(radius,radius),0,'circle',0)
    residual=orbit.distance(points)
    phases=np.unwrap(orbit.phase(points))
    if np.ptp(phases)<math.radians(35) or np.percentile(residual,95)>4:return None
    orbit.rms=float(np.sqrt(np.mean(residual**2)))
    return orbit


def fit_orbit(points):
    circle=circle_orbit(points)
    ellipse=None
    if len(points)>=28:
        fitted=cv2.fitEllipseDirect(np.asarray(points,np.float32).reshape(-1,1,2))
        center,diameters,angle=fitted
        axes=np.asarray(diameters,float)/2
        if np.all(np.isfinite([*center,*axes,angle])) and min(axes)>=8 and max(axes)<=640 and min(axes)/max(axes)>=.3:
            candidate=Orbit(center,axes,math.radians(angle),'ellipse',0)
            residual=candidate.distance(points)
            phase=np.unwrap(candidate.phase(points))
            if np.ptp(phase)>=math.radians(110) and np.percentile(residual,95)<=3:
                candidate.rms=float(np.sqrt(np.mean(residual**2)))
                if candidate.rms<2:ellipse=candidate
    # Extra ellipse parameters must improve fit substantially, not just absorb noise.
    if ellipse and (circle is None or ellipse.rms<.7*circle.rms and circle.rms>.65):return ellipse
    return circle


class PhaseFilter:
    def __init__(self,phase,omega,variance):
        self.state=np.array([phase,omega],float)
        self.covariance=np.diag([variance,.2**2])

    def update(self,phase,dt,variance):
        transition=np.array([[1.,dt],[0.,1.]])
        process=4*np.array([[dt**4/4,dt**3/2],[dt**3/2,dt**2]])
        self.state=transition@self.state
        self.covariance=transition@self.covariance@transition.T+process
        phase+=2*math.pi*round((self.state[0]-phase)/(2*math.pi))
        innovation=phase-self.state[0]
        gain=self.covariance[:,0]/(self.covariance[0,0]+variance)
        self.state+=gain*innovation
        correction=np.eye(2)-np.outer(gain,[1.,0.])
        self.covariance=correction@self.covariance@correction.T+np.outer(gain,gain)*variance


class BackgroundMotion:
    """Only peripheral background features vote; target pixels are excluded by location."""
    def __init__(self):self.previous=None
    def reset(self):self.previous=None

    def update(self,bgr):
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        previous=self.previous;self.previous=gray.copy()
        if previous is None or previous.shape!=gray.shape:return None
        h,w=gray.shape;mask=np.full_like(gray,255);mask[round(h*.22):round(h*.78),round(w*.22):round(w*.78)]=0
        points=cv2.goodFeaturesToTrack(previous,70,.02,8,mask=mask)
        if points is None or len(points)<12:return None
        current,status,_=cv2.calcOpticalFlowPyrLK(previous,gray,points,None,winSize=(15,15),maxLevel=2)
        if current is None:return None
        valid=status.ravel()!=0
        if valid.sum()<12:return None
        transform,inliers=cv2.estimateAffinePartial2D(points[valid],current[valid],method=cv2.RANSAC,ransacReprojThreshold=1.5)
        if transform is None or inliers is None or int(inliers.sum())<12:return None
        corners=np.array([[0,0],[w,0],[0,h],[w,h]],float)
        moved=corners@transform[:,:2].T+transform[:,2]-corners
        return float(np.median(np.linalg.norm(moved,axis=1)))


class MotionPredictor:
    def __init__(self,options=None):
        self.options=motion_settings(options)
        self.reset()

    def configure(self,options):
        self.options=motion_settings(options);self.reset()

    def reset(self):
        self.history=collections.deque(maxlen=140)
        self.pending=collections.deque(maxlen=120)
        self.errors=collections.deque(maxlen=40)
        self.model=None;self.phase_filter=None;self.uid=None;self.last_fit=-1e10
        self.previous=None;self.stopped=False;self.reverse_count=0
        self.transient_until=-1e10

    def base_report(self,state,point=None):
        return {'state':state,'ready':False,'point':list(point) if point is not None else None,
                'future':None,'direction':'unknown','omega_deg_s':None,'eta_ms':None,
                'horizon_ms':self.options['lead_ms']+self.options['latency_ms'],
                'validation_count':len(self.errors),'error_p95_px':float(np.percentile(self.errors,95)) if self.errors else None,
                'future_in_window':False,'grasp_advice':False,'observations':len(self.history),
                'model':self.model.kind if self.model else None,'forecast_path':[],
                'observation_stamp_s':self.history[-1][0] if self.history else None}

    def in_window(self,points):
        points=np.asarray(points)
        return (np.abs(points[...,0]-self.options['grasp_x'])<=self.options['half_width']) & (np.abs(points[...,1]-self.options['grasp_y'])<=self.options['half_height'])

    def update(self,point,stamp,uid,camera_motion=None):
        if not self.options['enabled']:return self.base_report('DISABLED')
        if camera_motion is not None and camera_motion>2.:
            self.reset();return self.base_report('CAMERA_MOVING',point)
        if point is None:
            if self.history and stamp-self.history[-1][0]>.25:self.reset()
            self.previous=None
            return self.base_report('TARGET_UNCONFIRMED')
        point=np.asarray(point,float)
        if not np.all(np.isfinite(point)) or not math.isfinite(stamp):return self.base_report('INVALID_OBSERVATION')
        if uid!=self.uid:
            self.reset();self.uid=uid
        if self.history:
            dt=stamp-self.history[-1][0]
            if dt<=0:return self.base_report('STALE_OBSERVATION',point)
            if dt>.25 or np.linalg.norm(point-np.asarray(self.history[-1][1:]))>max(25,450*dt):
                self.reset();self.uid=uid
        # Evaluate forecasts against later observations, never against the model itself.
        if self.previous is not None:
            previous_time,previous_point=self.previous
            while self.pending and self.pending[0][0]<=stamp:
                future_time,forecast=self.pending.popleft()
                if previous_time<future_time<=stamp and stamp-previous_time<=.15:
                    weight=(future_time-previous_time)/(stamp-previous_time)
                    actual=previous_point+(point-previous_point)*weight
                    self.errors.append(float(np.linalg.norm(actual-forecast)))
        self.previous=(stamp,point.copy())
        self.history.append((stamp,float(point[0]),float(point[1])))
        while self.history and stamp-self.history[0][0]>4:self.history.popleft()
        recent=np.asarray(list(self.history)[-9:],float)
        velocity=np.zeros(2)
        if len(recent)>=4 and recent[-1,0]-recent[0,0]>=.12:
            design=np.column_stack((recent[:,0]-stamp,np.ones(len(recent))))
            velocity=np.linalg.lstsq(design,recent[:,1:],rcond=None)[0][0]
        if len(recent)>=6 and recent[-1,0]-recent[0,0]>=.25 and np.ptp(recent[:,1:],axis=0).max()<1.5:
            if not self.stopped:self.pending.clear();self.errors.clear()
            self.stopped=True
            if self.phase_filter is not None:self.phase_filter.state[1]=0
            return self.base_report('STOPPED',point)
        self.stopped=False
        report=self.base_report('LEARNING_ORBIT',point)
        report['velocity_px_s']=velocity.tolist()
        if len(self.history)<12 or stamp-self.history[0][0]<.5:return report
        observations=np.asarray(self.history,float)
        refreshed=False
        if stamp-self.last_fit>=.3:
            candidate=fit_orbit(observations[:,1:])
            self.last_fit=stamp
            if candidate is not None:
                self.model=candidate;refreshed=True
        if self.model is None:return report
        if float(self.model.distance(point))>6:
            self.model=None;self.phase_filter=None;self.pending.clear();self.errors.clear()
            return self.base_report('ORBIT_MISMATCH',point)
        phases=np.unwrap(self.model.phase(recent[:,1:]))
        design=np.column_stack((recent[:,0]-stamp,np.ones(len(recent))))
        omega,phase=np.linalg.lstsq(design,phases,rcond=None)[0]
        radius=math.sqrt(float(np.prod(self.model.axes)))
        variance=(max(.6,self.model.rms)/radius)**2
        if self.phase_filter is None or refreshed:
            if self.phase_filter is not None and omega*self.phase_filter.state[1]<0:
                self.errors.clear();self.pending.clear();self.transient_until=stamp+.35
            self.phase_filter=PhaseFilter(phase,omega,variance)
        else:
            if omega*self.phase_filter.state[1]<0 and abs(omega)>.12:self.reverse_count+=1
            else:self.reverse_count=0
            if self.reverse_count>=2:
                self.phase_filter=PhaseFilter(phase,omega,variance)
                self.errors.clear();self.pending.clear();self.reverse_count=0
                self.transient_until=stamp+.35
            else:
                self.phase_filter.update(float(self.model.phase(point)),stamp-observations[-2,0],variance)
        phase,omega=self.phase_filter.state
        horizon=(self.options['lead_ms']+self.options['latency_ms'])/1000
        future=self.model.position(phase+omega*horizon)
        validation_horizon=max(.1,horizon)
        self.pending.append((stamp+validation_horizon,self.model.position(phase+omega*validation_horizon)))
        error=float(np.percentile(self.errors,95)) if self.errors else None
        ready=len(self.errors)>=5 and error<=self.options['error_limit_px'] and abs(omega)>.05 and stamp>=self.transient_until
        eta_horizon=min(.8,max(.6,horizon+.2))
        times=np.arange(0,eta_horizon+.005,.005)
        trajectory=self.model.position(phase+omega*times)
        entry=np.flatnonzero(self.in_window(trajectory))
        in_future=bool(self.in_window(future))
        tangent=self.model.rotation@(self.model.axes*np.array([-math.sin(phase),math.cos(phase)]))*omega
        uncertainty=radius*math.sqrt(max(0,float(np.array([1,horizon])@self.phase_filter.covariance@np.array([1,horizon]))))+self.model.rms
        report.update(state='PREDICTING' if ready else 'CHECKING_MODEL',ready=ready,model=self.model.kind,
                      direction='clockwise' if omega>0 else 'counterclockwise',omega_deg_s=math.degrees(float(omega)),
                      velocity_px_s=tangent.tolist(),future=future.tolist(),eta_ms=float(times[entry[0]]*1000) if len(entry) else None,
                      validation_count=len(self.errors),error_p95_px=error,fit_rms_px=self.model.rms,
                      eta_horizon_ms=eta_horizon*1000,
                      future_in_window=in_future,grasp_advice=bool(ready and in_future),uncertainty_px=float(uncertainty),
                      forecast_path=self.model.position(phase+omega*np.linspace(0,horizon,15)).tolist(),
                      orbit_center=self.model.center.tolist(),camera_motion_px=camera_motion)
        return report

    def annotate(self,image,report):
        if not self.options['enabled']:return image
        canvas=image.copy()
        history=np.asarray(self.history,float)
        if len(history)>=2:cv2.polylines(canvas,[np.rint(history[:,1:]).astype(np.int32)],False,(180,180,180),1)
        p=self.options
        left,top=round(p['grasp_x']-p['half_width']),round(p['grasp_y']-p['half_height'])
        right,bottom=round(p['grasp_x']+p['half_width']),round(p['grasp_y']+p['half_height'])
        cv2.rectangle(canvas,(left,top),(right,bottom),(200,90,230),1)
        cv2.drawMarker(canvas,(round(p['grasp_x']),round(p['grasp_y'])),(200,90,230),cv2.MARKER_CROSS,8,1)
        point=report.get('point')
        if point is not None and report.get('velocity_px_s') is not None:
            current=np.asarray(point);next_point=current+np.asarray(report['velocity_px_s'])*.12
            cv2.arrowedLine(canvas,tuple(np.rint(current).astype(int)),tuple(np.rint(next_point).astype(int)),(255,230,50),1,tipLength=.25)
        if report.get('future') is not None:
            color=(240,80,230) if report['ready'] else (0,175,255)
            curve=np.rint(report['forecast_path']).astype(np.int32)
            cv2.polylines(canvas,[curve],False,color,1)
            future=tuple(np.rint(report['future']).astype(int))
            cv2.circle(canvas,future,4,color,-1)
            cv2.putText(canvas,'+%dms'%report['horizon_ms'],(future[0]+4,future[1]-4),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
        return canvas
