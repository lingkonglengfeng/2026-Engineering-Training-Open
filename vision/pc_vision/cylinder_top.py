"""Local, measured rim proposals for cylinder faces; no assumed light direction.

Hough circles only seed a search. Actual normal gradients around most of the
rim must support the final ellipse, including the weak rim against the side.
All returned contours use the caller's processing coordinates.
"""
import math

import cv2
import numpy as np


ANGLES=np.linspace(0,2*math.pi,96,endpoint=False).astype(np.float32)
NORMALS=np.column_stack((np.cos(ANGLES),np.sin(ANGLES)))


def longest_gap(valid):
    doubled=np.r_[~valid,~valid]
    maximum=current=0
    for missing in doubled:
        current=current+1 if missing else 0
        maximum=max(maximum,current)
    return min(len(valid),maximum)*360/len(valid)


def measured_rim(gradients,seed):
    cx,cy,radius=map(float,seed)
    offsets=np.linspace(-max(2.5,radius*.10),max(2.5,radius*.10),11,dtype=np.float32)
    radii=radius+offsets
    x=cx+NORMALS[:,0,None]*radii
    y=cy+NORMALS[:,1,None]*radii
    response=np.zeros_like(x)
    for gx,gy in gradients:
        dx=cv2.remap(gx,x,y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        dy=cv2.remap(gy,x,y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        normal=np.abs(dx*NORMALS[:,0,None]+dy*NORMALS[:,1,None])
        magnitude=np.hypot(dx,dy)
        response=np.maximum(response,np.where(normal>=.70*magnitude,normal,0))
    # Don't let the bright silhouette overwhelm the weaker top/side boundary.
    best=np.argmax(response,axis=1)
    strength=response[np.arange(len(ANGLES)),best]
    threshold=max(6.,float(np.percentile(strength,30))*.15)
    valid=strength>=threshold
    coverage=float(np.mean(valid));gap=longest_gap(valid)
    if coverage<.82 or gap>60:return None
    points=np.column_stack((x[np.arange(len(ANGLES)),best],y[np.arange(len(ANGLES)),best]))[valid]
    ellipse=cv2.fitEllipseAMS(points.reshape(-1,1,2))
    (ex,ey),(a,b),angle=ellipse
    if not all(math.isfinite(v) for v in (ex,ey,a,b,angle)):return None
    if min(a,b)<radius*1.45 or max(a,b)>radius*2.35 or min(a,b)/max(a,b)<.85:return None
    if math.hypot(ex-cx,ey-cy)>radius*.22:return None
    theta=math.radians(angle);xy=points-np.array([ex,ey])
    u=xy[:,0]*math.cos(theta)+xy[:,1]*math.sin(theta)
    v=-xy[:,0]*math.sin(theta)+xy[:,1]*math.cos(theta)
    residual=np.abs(np.sqrt((u/(a/2))**2+(v/(b/2))**2)-1)
    if float(np.percentile(residual,95))>.10:return None
    return points,{'coverage':coverage,'gap_deg':gap,'rim_residual':float(np.sqrt(np.mean(residual**2)))}


def find_top_rims(raw,body_contour,processing_size,tracked_seeds=()):
    width,height=processing_size
    sx,sy=raw.shape[1]/width,raw.shape[0]/height
    contour=body_contour.astype(np.float32)*np.array([sx,sy],np.float32)
    x,y,w,h=cv2.boundingRect(contour)
    if min(w,h)<20 or max(w,h)>raw.shape[1]*.65:return []
    hull=cv2.contourArea(cv2.convexHull(contour))
    if hull<=0 or cv2.contourArea(contour)/hull<.85:return []
    # Chassis fragments meeting two frame borders are not complete material faces.
    if sum((x<=1,y<=1,x+w>=raw.shape[1]-1,y+h>=raw.shape[0]-1))>=2:return []
    x0,y0=max(0,x-8),max(0,y-8)
    roi=raw[y0:min(raw.shape[0],y+h+8),x0:min(raw.shape[1],x+w+8)]
    gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    lab=cv2.cvtColor(roi,cv2.COLOR_BGR2LAB)
    gradients=[]
    for channel in (gray,lab[:,:,1],lab[:,:,2]):
        smooth=cv2.GaussianBlur(channel,(3,3),.65).astype(np.float32)
        gradients.append((cv2.Sobel(smooth,cv2.CV_32F,1,0),cv2.Sobel(smooth,cv2.CV_32F,0,1)))
    rmin=max(6,round(min(w,h)*.20));rmax=max(rmin+1,round(min(w,h)*.54))
    seeds=[];output=[]
    # History only proposes a search location. Re-measure the current image;
    # never return the previous center when the current rim is missing.
    for cx,cy,radius in tracked_seeds:
        seed=np.float32([cx*sx-x0,cy*sy-y0,radius*math.sqrt(sx*sy)])
        if not rmin<=seed[2]<=rmax:continue
        result=measured_rim(gradients,seed)
        if result is not None:
            points,evidence=result
            points=(points+np.array([x0,y0],np.float32))/np.array([sx,sy],np.float32)
            output.append((points.reshape(-1,1,2).astype(np.float32),evidence))
            seeds.append(seed)
    for image in (gray,cv2.createCLAHE(3,(4,4)).apply(gray)):
        circles=cv2.HoughCircles(cv2.GaussianBlur(image,(3,3),0),cv2.HOUGH_GRADIENT,
                                1,max(8,rmin*.65),param1=30,param2=13,
                                minRadius=rmin,maxRadius=rmax)
        if circles is None:continue
        for seed in circles[0][:10]:
            if any(np.linalg.norm(seed[:2]-old[:2])<seed[2]*.18 and .8<seed[2]/old[2]<1.25 for old in seeds):continue
            seeds.append(seed)
            result=measured_rim(gradients,seed)
            if result is None:continue
            points,evidence=result
            points+=np.array([x0,y0],np.float32)
            points/=np.array([sx,sy],np.float32)
            output.append((points.reshape(-1,1,2).astype(np.float32),evidence))
    return output
