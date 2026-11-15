"""Visible cylinder-face analysis. Image ellipses are 2-D evidence, not a 3-D pose."""
import math
import time

import cv2
import numpy as np
from color_classifier import BluePairModel,pc_calibrated_threshold,black_pixel_gate
from cylinder_top import find_top_rims

TAU = np.linspace(0,2*math.pi,96,endpoint=False)


def ellipse_residual(points,ellipse):
    (cx,cy),(width,height),angle=ellipse
    theta=math.radians(angle)
    xy=points.reshape(-1,2).astype(np.float32)-np.array([cx,cy],np.float32)
    u=xy[:,0]*math.cos(theta)+xy[:,1]*math.sin(theta)
    v=-xy[:,0]*math.sin(theta)+xy[:,1]*math.cos(theta)
    return np.abs(np.sqrt((u/(width/2))**2+(v/(height/2))**2)-1)


def ellipse_polygon(ellipse,factor=1):
    (cx,cy),(width,height),angle=ellipse
    theta=math.radians(angle)
    x,y=np.cos(TAU)*width*factor/2,np.sin(TAU)*height*factor/2
    return np.column_stack((cx+x*math.cos(theta)-y*math.sin(theta),cy+x*math.sin(theta)+y*math.cos(theta))).astype(np.float32)


def contour_parts(mask,light,saturation,lab):
    """Whole silhouette and native Otsu subregions; no assumed fixed light direction."""
    yield "outline",mask
    selected=mask!=0
    for name,channel,spread in (("light",light,12),("saturation",saturation,25)):
        values=channel[selected]
        if values.size<80 or np.percentile(values,90)-np.percentile(values,10)<spread:
            continue
        cut,_=cv2.threshold(values.reshape(-1,1),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        for high in (True,False):
            part=np.uint8(selected & ((channel>cut) if high else (channel<=cut)))*255
            part=cv2.morphologyEx(part,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
            yield name+("+" if high else "-"),part
    # A dark blue top can differ from its side by only a few a*/b* units.
    # Lightness Otsu then splits the cast shadow instead of the face. Use several
    # data-relative chroma cuts; the existing ellipse/rim tests must validate them.
    for axis in (1, 2):
        channel = lab[:, :, axis]
        values = channel[selected]
        if values.size < 80 or np.percentile(values, 90) - np.percentile(values, 10) < 2:
            continue
        for cut in np.unique(np.quantile(values, [.35, .65, .8])):
            for high in (True, False):
                part = np.uint8(selected & ((channel > cut) if high else (channel <= cut))) * 255
                part = cv2.morphologyEx(part, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                part = cv2.morphologyEx(part, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                yield 'chroma%d'%axis, part
    values=lab[selected]
    if len(values)>=150 and float(np.max(np.std(values,axis=0)))>=3:
        # Three deterministic LAB regions can isolate a face whose brightness lies
        # between a bright side wall and its shadow (binary Otsu cannot do that).
        sample=values[::max(1,len(values)//1000)]
        axis=int(np.argmax(np.std(sample,axis=0)))
        ordered=sample[np.argsort(sample[:,axis])]
        centers=ordered[np.rint(np.array([.1,.5,.9])*(len(ordered)-1)).astype(int)].copy()
        for _ in range(8):
            labels=np.argmin(np.sum((sample[:,None,:]-centers[None,:,:])**2,axis=2),axis=1)
            updated=np.array([sample[labels==i].mean(0) if np.any(labels==i) else centers[i] for i in range(3)])
            if np.max(np.abs(updated-centers))<.15:break
            centers=updated
        labels=np.argmin(np.sum((values[:,None,:]-centers[None,:,:])**2,axis=2),axis=1)
        for index in range(3):
            part=np.zeros_like(mask);part[selected]=np.uint8(labels==index)*255
            part=cv2.morphologyEx(part,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
            part=cv2.morphologyEx(part,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            yield 'lab%d'%index,part



class SurfaceAnalyzer:
    def __init__(self,material_module,confirm_frames=2):
        self.material=material_module
        self.confirm_frames=confirm_frames
        self.reset()

    def reset(self):
        self.tracks={}
        self.next_uid=1

    def fit_candidate(self,contour,source,body_id,color_ids,body_contour,masks,lab,edges,scale):
        area=cv2.contourArea(contour)
        if len(contour)<8 or area<80*scale*scale:
            return None
        x,y,w,h=cv2.boundingRect(contour)
        height,width=lab.shape[:2]
        if x<=1 or y<=1 or x+w>=width-1 or y+h>=height-1:
            return None  # no claim of a complete face when its boundary is clipped
        hull=cv2.contourArea(cv2.convexHull(contour))
        convexity=area/hull if hull else 0
        if convexity<.88:
            return None
        ellipse=cv2.fitEllipseAMS(contour[::max(1,len(contour)//240)])
        (cx,cy),(a,b),angle=ellipse
        if not all(math.isfinite(v) for v in (cx,cy,a,b,angle)) or min(a,b)<9*scale or max(a,b)>220*scale:
            return None
        aspect=min(a,b)/max(a,b)
        if aspect<.48 or cv2.pointPolygonTest(body_contour,(cx,cy),False)<0:
            return None
        residual=ellipse_residual(contour,ellipse)
        rms=float(np.sqrt(np.mean(residual**2)))
        p95=float(np.percentile(residual,95))
        fill=area/max(math.pi*a*b/4,1)
        if rms>.09 or p95>.17 or not .78<=fill<=1.12:
            return None
        # Evaluate real color pixels inside the fitted face, avoiding its boundary.
        inner=np.zeros((height,width),np.uint8)
        cv2.fillPoly(inner,[np.rint(ellipse_polygon(ellipse,.72)).astype(np.int32)],255)
        inside=inner!=0
        if np.count_nonzero(inside)<30*scale*scale:
            return None
        votes={key:float(np.mean(mask[inside]!=0)) for key,mask in masks.items()}
        color_id=max(color_ids,key=lambda key:votes[key])
        purity=votes[color_id]
        other=max((value for key,value in votes.items() if key!=color_id),default=0.)
        margin=purity-other
        if purity<.45:
            return None
        values=lab[inside]
        spread=np.percentile(values,90,axis=0)-np.percentile(values,10,axis=0)
        uniform=math.exp(-float(spread[0])/24-float(np.linalg.norm(spread[1:]))/65)
        points=ellipse_polygon(ellipse)
        px=np.clip(np.rint(points[:,0]).astype(int),0,width-1)
        py=np.clip(np.rint(points[:,1]).astype(int),0,height-1)
        edge_support=float(np.mean(edges[py,px]>0))
        geometry=max(0.,1-rms/.12)
        quality=.30*geometry+.24*purity+.17*uniform+.18*edge_support+.11*min(1.,margin)
        if quality<.69 or edge_support<.35:
            return None
        reason='ACQUIRE'
        if purity<.65 or margin<.15:reason='COLOR_AMBIGUOUS'
        if source=='outline' and (uniform<.80 or aspect<.70 or rms>.045):reason='TOP_UNRESOLVED'
        if source=='rim' and uniform<.70 and rms>.045:reason='TOP_UNRESOLVED'
        # A connected shadow/chassis can enlarge the color component and make
        # area/body checks meaningless. A visibly elongated, non-elliptical rim
        # is still a silhouette even when it occupies only a small component part.
        if source!='local_rim' and aspect<.85 and rms>.03:
            reason='TOP_UNRESOLVED'
        # A thresholded shadow that still covers almost the entire mixed body is
        # not an independent top face, even if the external outline is elliptical.
        body_area = cv2.contourArea(body_contour)
        if area > .72 * body_area and (uniform < .80 or aspect < .88 or rms > .035):
            reason='TOP_UNRESOLVED'
        return {"ellipse":ellipse,"center":(float(cx),float(cy)),"radius":math.sqrt(a*b)/2,
                "axes":(float(max(a,b)),float(min(a,b))),"angle":float(angle),
                "area":float(math.pi*a*b/4),"contour":contour,"body_contour":body_contour,
                "color":color_id,"body":body_id,"source":source,"quality":quality,
                "purity":purity,"margin":margin,"residual":rms,"edge_support":edge_support,
                "uniformity":uniform,"stable":0,"confirmed":False,"uid":None,"reason":reason}

    @staticmethod
    def duplicate(a,b):
        distance=math.dist(a['center'],b['center'])
        return distance<.25*min(a['radius'],b['radius']) and .7<a['radius']/b['radius']<1.4

    def process(self,bgr,samples,requested=0,stamp=None,all_colors=True,raw_bgr=None):
        if bgr.ndim!=3 or bgr.shape[2]!=3:
            raise ValueError("Expected BGR image")
        stamp=time.perf_counter() if stamp is None else stamp
        height,width=bgr.shape[:2];scale=width/320
        raw_bgr=bgr if raw_bgr is None else raw_bgr
        lab=cv2.cvtColor(bgr.astype(np.float32)/255,cv2.COLOR_BGR2LAB)
        light=np.clip(lab[:,:,0]*2.55,0,255).astype(np.uint8)
        hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
        # LAB edges retain boundaries between equally bright colors.
        edges=np.zeros((height,width),np.uint8)
        for channel in (light,np.clip(lab[:,:,1]+128,0,255).astype(np.uint8),
                        np.clip(lab[:,:,2]+128,0,255).astype(np.uint8)):
            edges|=cv2.Canny(cv2.GaussianBlur(channel,(3,3),0),12,36)
        thin_edges=edges.copy()
        edges=cv2.dilate(edges,np.ones((3,3),np.uint8))
        masks={};calibration={}
        for color in self.material.COLORS:
            threshold,state=pc_calibrated_threshold(color,samples,self.material)
            mask=cv2.inRange(lab,np.array(threshold[::2],np.float32),np.array(threshold[1::2],np.float32))
            if state=="OVERLAP":mask[:]=0
            if color[0]==5:mask[~black_pixel_gate(lab,hsv)]=0
            masks[color[0]]=mask;calibration[color[0]]=state
        blue_model = BluePairModel(samples)
        blue_labels, blue_family, blue_margins = blue_model.classify(lab, hsv)
        if blue_model.state == 'CHROMA_OK':
            for key in (3, 6):
                masks[key] = np.uint8(blue_labels == key) * 255
                calibration[key] = 'CHROMA_OK'
            # Dark saturated blue must not simultaneously vote as calibrated black.
            for key in (1, 2, 4, 5):
                masks[key][blue_family] = 0
        elif blue_model.state == 'CHROMA_OVERLAP':
            masks[3][:] = masks[6][:] = 0
            calibration[3] = calibration[6] = 'OVERLAP'
        candidates=[];unresolved=[];regions=[];rejected=0;body_id=0
        for family in ((1,),(2,),(3,6),(4,),(5,)):
            if requested and not all_colors and requested not in family:
                continue
            mask=np.bitwise_or.reduce([masks[key] for key in family])
            if family == (3, 6) and blue_model.state in ('CHROMA_OK', 'CHROMA_OVERLAP'):
                mask = np.uint8(blue_family) * 255
            clean=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            count,labels,stats,_=cv2.connectedComponentsWithStats(clean,connectivity=8)
            components=sorted(range(1,count),key=lambda k:int(stats[k,4]),reverse=True)[:6]
            for index in components:
                area=int(stats[index,4])
                if area<100*scale*scale or area>width*height*.5:
                    continue
                body=np.uint8(labels==index)*255
                contours=cv2.findContours(body,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
                if not contours:continue
                body_contour=max(contours,key=cv2.contourArea)
                body_id+=1
                if family == (3, 6) and min(stats[index,2:4]) >= .35 * max(stats[index,2:4]):
                    core = cv2.erode(body, np.ones((3, 3), np.uint8))
                    region = blue_model.describe_region(core, blue_labels, blue_margins)
                    region.update(bounds=tuple(int(v) for v in stats[index, :4]), area=area)
                    regions.append(region)
                options=[]
                for source,part in contour_parts(body,light,hsv[:,:,1],lab):
                    for contour in sorted(cv2.findContours(part,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2],key=cv2.contourArea,reverse=True)[:3]:
                        candidate=self.fit_candidate(contour,source,body_id,family,body_contour,masks,lab,edges,scale)
                        if candidate and not any(self.duplicate(candidate,existing) and candidate['quality']<=existing['quality'] for existing in options):
                            options=[existing for existing in options if not self.duplicate(candidate,existing)]
                            options.append(candidate)
                rim=thin_edges & cv2.dilate(body,np.ones((5,5),np.uint8))
                for contour in sorted(cv2.findContours(rim,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[-2],key=cv2.contourArea,reverse=True)[:12]:
                    candidate=self.fit_candidate(contour,'rim',body_id,family,body_contour,masks,lab,edges,scale)
                    if candidate and not any(self.duplicate(candidate,existing) and candidate['quality']<=existing['quality'] for existing in options):
                        options=[existing for existing in options if not self.duplicate(candidate,existing)]
                        options.append(candidate)
                seeds=[]
                for key in family:
                    previous=self.tracks.get(key)
                    if previous and 0<stamp-previous['stamp']<=.25 and cv2.pointPolygonTest(body_contour,previous['center'],False)>=0:
                        seeds.append((*previous['center'],previous['radius']))
                for contour,evidence in find_top_rims(raw_bgr,body_contour,(width,height),seeds):
                    candidate=self.fit_candidate(contour,'local_rim',body_id,family,body_contour,masks,lab,edges,scale)
                    if candidate and not any(self.duplicate(candidate,existing) and candidate['quality']<=existing['quality'] for existing in options):
                        candidate.update(evidence)
                        options=[existing for existing in options if not self.duplicate(candidate,existing)]
                        options.append(candidate)
                options.sort(key=lambda c:c['quality'],reverse=True)
                if not options:
                    rejected+=1
                    unresolved.append({'body_contour':body_contour,'family':family})
                    continue
                best=options[0]
                # Different plausible faces inside one colored body must not be resolved by area alone.
                # A coherent subregion with a real rim outranks a mixed wall/face silhouette.
                resolved=[item for item in options if item['reason'] in ('ACQUIRE','COLOR_AMBIGUOUS')]
                if resolved:best=resolved[0]
                rivals=[item for item in resolved if item is not best]
                if rivals and rivals[0]['quality']>best['quality']-.035:
                    best['reason']='FACE_AMBIGUOUS'
                if 3 in family and not (samples.get('3') and samples.get('6')):
                    best['reason']='BLUE_PAIR_CALIBRATION'
                candidates.append(best)
        chosen=[]
        for color_id in masks:
            pool=sorted([item for item in candidates if item['color']==color_id],key=lambda item:item['quality'],reverse=True)
            if len(pool)>1 and pool[1]['quality']>=pool[0]['quality']-.08:
                for item in pool:item['reason']='SAME_COLOR_AMBIGUOUS'
            if not pool or pool[0]['reason']!='ACQUIRE':
                self.tracks.pop(color_id,None)
                continue
            item=pool[0];previous=self.tracks.get(color_id)
            stable=1;uid=self.next_uid
            if previous:
                elapsed=stamp-previous['stamp']
                radius_ratio=item['radius']/previous['radius']
                if 0<elapsed<=.25 and .65<radius_ratio<1.5 and math.dist(item['center'],previous['center'])<=max(12*scale,item['radius']*.9):
                    stable=previous['stable']+1;uid=previous['uid']
            if stable==1:self.next_uid+=1
            item.update(stable=stable,uid=uid,confirmed=stable>=self.confirm_frames,
                        reason='SURFACE_OK' if stable>=self.confirm_frames else 'ACQUIRE')
            self.tracks[color_id]={"stamp":stamp,"center":item['center'],"radius":item['radius'],"stable":stable,"uid":uid}
            chosen.append(item)
        target=None
        eligible=[item for item in chosen if item['confirmed'] and (not requested or item['color']==requested)]
        if len(eligible)==1:target=eligible[0]
        elif requested and eligible:target=max(eligible,key=lambda item:item['quality'])
        reason='SURFACE_OK' if target else 'NO_CONFIRMED_SURFACE'
        if requested:
            pool=[item for item in candidates if item['color']==requested]
            if pool and target is None:reason=pool[0]['reason']
            if not pool and any(requested in item['family'] for item in unresolved):reason='NO_TOP_EVIDENCE'
            if requested in (3,6) and not(samples.get('3') and samples.get('6')):
                reason='BLUE_PAIR_CALIBRATION'
            if calibration[requested]=='OVERLAP':reason='COLOR_OVERLAP'
        return {"target":target,"surfaces":candidates,"reason":reason,"rejected":rejected,
                "unresolved":unresolved,"processing_size":(width,height),
                "color_regions":regions,"blue_model":{'state':blue_model.state,'separation':blue_model.separation}}

    @staticmethod
    def describe(scene,requested):
        item=scene['target']
        if item is None:
            matching=[item for item in scene['surfaces'] if not requested or item['color']==requested]
            if matching:item=max(matching,key=lambda item:item['quality'])
        return {'status':scene['reason'],'count':len(scene['surfaces']),
                'confirmed':scene['target'] is not None,'quality':item['quality'] if item else None,
                'center':item['center'] if item else None,'axes':item['axes'] if item else None,
                'area':item['area'] if item else None,'angle':item['angle'] if item else None,
                'purity':item['purity'] if item else None,'stable':item['stable'] if item else 0,
                'source':item['source'] if item else None,'rim_coverage':item.get('coverage') if item else None,
                'colors':scene.get('color_regions', []),'blue_model':scene.get('blue_model')}

    def annotate(self,bgr,scene):
        canvas=bgr.copy()
        for region in scene.get('color_regions', []):
            x,y,w,h=region['bounds']
            name={3:'BLUE',6:'LIGHTBLUE'}.get(region['color'],'BLUE?')
            cv2.rectangle(canvas,(x-2,y-2),(x+w+1,y+h+1),(220,180,70),1)
            cv2.putText(canvas,name+' region',(max(0,x-3),min(canvas.shape[0]-3,y+h+12)),
                        cv2.FONT_HERSHEY_SIMPLEX,.33,(220,180,70),1,cv2.LINE_AA)
        for item in scene.get('unresolved',[]):
            cv2.drawContours(canvas,[item['body_contour']],-1,(0,130,230),1)
        for item in scene['surfaces']:
            color=(0,220,80) if item['confirmed'] else (0,180,255)
            cv2.drawContours(canvas,[item['body_contour']],-1,(210,170,50),1)
            overlay=canvas.copy()
            cv2.fillPoly(overlay,[np.rint(ellipse_polygon(item['ellipse'])).astype(np.int32)],color)
            canvas=cv2.addWeighted(overlay,.18,canvas,.82,0)
            cv2.ellipse(canvas,item['ellipse'],color,1,cv2.LINE_AA)
            point=tuple(round(v) for v in item['center'])
            cv2.drawMarker(canvas,point,color,cv2.MARKER_CROSS,9,1)
            text='%s TOP%s Q%.0f'%(self.material.COLORS[item['color']-1][1],
                                  '' if item['confirmed'] else '?',100*item['quality'])
            cv2.putText(canvas,text,(max(0,point[0]-35),max(12,point[1]-round(item['radius'])-4)),cv2.FONT_HERSHEY_SIMPLEX,.33,color,1,cv2.LINE_AA)
        return canvas
