"""Current-frame occupied ring refinement using a robust shared-center conic model."""
import math

import cv2
import numpy as np


def solve_joint(points,layers,weights,radius):
    unique=np.unique(layers)
    x,y=points.T
    design=np.column_stack((x*x-y*y,2*x*y,2*x,2*y,*[(layers==layer).astype(float) for layer in unique]))
    rhs=-y*y
    weight=weights.copy()
    for _ in range(4):
        weighted=design*np.sqrt(weight[:,None])
        solution,_,rank,_=np.linalg.lstsq(weighted,rhs*np.sqrt(weight),rcond=None)
        if rank<design.shape[1] or np.linalg.cond(weighted)>1e4:return None
        a,b,d,e=solution[:4];q=np.array([[a,b],[b,1-a]])
        eigen=np.linalg.eigvalsh(q)
        if eigen[0]<=.1 or eigen[1]/eigen[0]>1.45:return None
        gradient=np.hypot(2*(a*x+b*y+d),2*(b*x+(1-a)*y+e))
        residual=np.abs(design@solution-rhs)/np.maximum(gradient,1e-5)*radius
        weight=weights*np.minimum(1,1/np.maximum(residual,.01))
    center=-np.linalg.solve(q,np.array([d,e]))
    squared=center@q@center-solution[4:]
    if np.any(squared<=0):return None
    return center,np.sqrt(2*squared),residual,unique


def refine_occupied(gray,bgr,seed,ring):
    radius=seed.get('sampling_radius',seed['radius'])
    if radius<15:return None,{'reason':'TOO_SMALL'}
    angles,radial,nx,ny,mx,my=ring.occupied_sampling_grid(seed,ring.partial_normalization,step=2)
    values=cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    height,width=gray.shape
    valid=(mx>3)&(mx<width-4)&(my>3)&(my<height-4)
    # Chroma marks material in live RGB and annotations in old grayscale screenshots.
    chroma=bgr.max(2).astype(np.int16)-bgr.min(2).astype(np.int16)
    colored=cv2.dilate(np.uint8(chroma>40),np.ones((3,3),np.uint8))
    valid &= cv2.remap(colored,mx,my,cv2.INTER_NEAREST)==0
    step=float(radial[1]-radial[0]);offset=max(2,round(max(1.3,radius*.027)/step))
    if values.shape[1]<2*offset+5:return None,{'reason':'NARROW_GRID'}
    # A ring line is a narrow dark valley with lighter pixels on BOTH sides.
    # A material silhouette or shadow edge generally has only one light side.
    valley=np.full_like(values,-1000)
    valley[:,offset:-offset]=np.minimum(values[:,:-2*offset],values[:,2*offset:])-values[:,offset:-offset]
    valid[:,:offset]=False;valid[:,-offset:]=False
    valley[~valid]=-1000
    points=[];layers=[];weights=[];fractions=(65/95,75/95,85/95,1.)
    rows=np.arange(len(angles))
    for layer,fraction in enumerate(fractions):
        band=np.flatnonzero(np.abs(radial-fraction*radius)<=max(2.,radius*.055))
        if len(band)<3:continue
        indices=band[np.argmax(valley[:,band],axis=1)]
        strength=valley[rows,indices]
        supported=(strength>=7)&valid[rows,indices]
        available=int(valid[:,band].any(1).sum())
        if int(supported.sum())<max(60,math.ceil(.55*available)):continue
        measured_angles=angles[supported]
        gap=float(np.diff(np.r_[measured_angles,measured_angles[0]+2*math.pi]).max())
        if 360-math.degrees(gap)<220:continue
        selected=indices[supported]
        points.append(np.column_stack((nx[rows[supported],selected],ny[rows[supported],selected]))/radius)
        layers.extend([layer]*int(supported.sum()))
        weights.extend(np.clip(strength[supported]/30,.3,1.5)/max(int(supported.sum()),1))
    if len(points)<3:return None,{'reason':'NEED_THREE_VISIBLE_LAYERS','layers':len(points)}
    points=np.vstack(points).astype(float);layers=np.array(layers);weights=np.array(weights)
    solution=solve_joint(points,layers,weights,radius)
    if solution is None:return None,{'reason':'ILL_CONDITIONED'}
    local,radii,residual,unique=solution
    scales=radii/np.array([fractions[int(i)] for i in unique])
    if np.ptp(scales)>.08 or not .90<=float(np.median(scales))<=1.10:
        return None,{'reason':'RING_SPACING_MISMATCH'}
    inverse=np.array(ring.partial_normalization(seed)[1]).reshape(2,2)
    center=np.array(seed['center'])+inverse@(local*radius)
    shift=float(np.linalg.norm(center-np.array(seed['center'])))
    p95=float(np.percentile(residual,95))
    if shift>max(4.,radius*.08) or p95>1.5 or float(np.median(residual))>.55:
        return None,{'reason':'INCONSISTENT_ARCS','shift_px':shift,'p95_px':p95}
    # Leave-one-layer-out agreement catches a single displaced ring/shadow solution.
    centers=[]
    for layer in unique:
        keep=layers!=layer
        estimate=solve_joint(points[keep],layers[keep],weights[keep],radius)
        if estimate is not None:centers.append(inverse@(estimate[0]*radius))
    spread=max((float(np.linalg.norm(c-inverse@(local*radius))) for c in centers),default=100.)
    if len(centers)<len(unique)-1 or spread>max(1.3,radius*.025):
        return None,{'reason':'LAYER_DISAGREEMENT','spread_px':spread}
    result=dict(seed,center=tuple(float(v) for v in center),joint_refined=True)
    result['ellipse']=(result['center'],seed['ellipse'][1],seed['ellipse'][2])
    return result,{'reason':'JOINT_RING_OK','layers':len(unique),'p95_px':p95,'spread_px':spread,'shift_px':shift}
