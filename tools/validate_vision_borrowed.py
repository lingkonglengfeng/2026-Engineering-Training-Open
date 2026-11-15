"""与修改前快照比较旧实拍及由实拍生成的运动/裁切序列（不等同实车视频）。"""
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_material'),str(ROOT/'vision/maixcam_ring')]
import cv2
import numpy as np
cv2.setNumThreads(1)
OUT=ROOT/'output/vision_borrowed_improvements'


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def read(path):
    result=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
    if result is None: raise ValueError(str(path))
    return result


def save(name,array):
    cv2.imencode('.png',array)[1].tofile(str(OUT/name))


class Blob:
    def __init__(self,stats,center):self.stats,self.center=stats,center
    def x(self):return int(self.stats[0])
    def y(self):return int(self.stats[1])
    def w(self):return int(self.stats[2])
    def h(self):return int(self.stats[3])
    def pixels(self):return int(self.stats[4])
    def cx(self):return float(self.center[0])
    def cy(self):return float(self.center[1])


def material(module,mask):
    _,_,stats,centers=cv2.connectedComponentsWithStats(mask,8)
    blobs=[Blob(s,c) for s,c in zip(stats[1:],centers[1:])]
    native=SimpleNamespace(width=lambda:mask.shape[1],height=lambda:mask.shape[0],find_blobs=lambda *a,**k:blobs)
    frame=SimpleNamespace(binary=lambda *a,**k:None,to_format=lambda f:native)
    api=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray'),image2cv=lambda *a,**k:mask)
    details={};target,_=module.detect_material_center(frame,[],api,details)
    return target,details


def mask_for(bgr,threshold):
    lab=cv2.cvtColor(bgr,cv2.COLOR_BGR2LAB).astype(np.float32)
    lab[:,:,0]*=100/255;lab[:,:,1:]-=128
    return cv2.inRange(lab,np.array(threshold[::2],np.float32),np.array(threshold[1::2],np.float32))


def timed(call,n=60):
    for _ in range(5):call()
    samples=[]
    for _ in range(n):
        start=time.perf_counter();call();samples.append(1000*(time.perf_counter()-start))
    return statistics.median(samples)


def run():
    before=OUT/'before'
    oldmat=load('before_material',before/'main.py');newmat=load('after_material',ROOT/'vision/maixcam_material/main.py')
    oldring=load('before_ring',before/'middle_ring.py');newring=load('after_ring',ROOT/'vision/maixcam_ring/middle_ring.py')
    oldplate=load('before_plate',before/'plate_tangent.py');newplate=load('after_plate',ROOT/'vision/maixcam_material/plate_tangent.py')
    report={'limits':['输入是历史压缩截图裁切，不是原始相机帧。',
                      '物料LAB阈值在电脑以OpenCV近似，连通域为OpenCV替身，未测Maix原生算子。',
                      '平移、裁切、曝光扰动是从实拍图生成的模拟序列，未验证真实运动和光照。',
                      '单张照片没有精确人工标注，不能声称绝对精度提高。电脑耗时不等于设备FPS。'],
            'materials':[],'rings':[],'plates':[],'sequences':[]}
    material_paths=[('materials_1','codex-clipboard-9048003b-8d51-4a8b-a2b6-fb9328b92279.png',(0,106,449,287)),
                    ('materials_2','codex-clipboard-792cc5d1-585c-4d58-b47d-4d79b1c97849.png',(0,202,442,384))]
    for name,file,(left,top,right,bottom) in material_paths:
        # 已将可见相机区域存为回归样本，原Temp附件过期后也可复现。
        fixture=ROOT/'vision/maixcam_material/test_data'/('materials_overview_'+name[-1]+'.png')
        canvas=read(fixture)
        save(name+'_input.png',canvas)
        pair=[]
        results=[]
        for module in (oldmat,newmat):
            drawn=canvas.copy();records=[]
            for color in module.COLORS:
                mask=mask_for(canvas,color[2]);target,details=material(module,mask)
                records.append({'color':color[1],'center':None if target is None else list(target[:2]),'reason':details.get('reason')})
                if target:
                    cv2.circle(drawn,tuple(round(v) for v in target[:2]),round(target[2]),(0,255,0),1)
                    cv2.putText(drawn,color[1],tuple(round(v) for v in target[:2]),cv2.FONT_HERSHEY_SIMPLEX,.30,(0,0,255),1)
            results.append(records);pair.append(drawn)
        save(name+'_before_after.png',np.hstack(pair))
        report['materials'].append({'name':name,'before':results[0],'after':results[1]})
        # 全部四种有完整候选的实拍物料都验证裁切，不能只挑成功的一个。
        for color in oldmat.COLORS[:4]:
            mask=mask_for(canvas,color[2]);target,_=material(oldmat,mask)
            if target:
                shift=round(2-target[0]);half=cv2.warpAffine(mask,np.float32([[1,0,shift],[0,1,0]]),(320,240),borderValue=0)
                a,da=material(oldmat,half);b,db=material(newmat,half)
                save(name+'_'+color[1].lower()+'_half_mask.png',half)
                report['materials'].append({'name':name+'_translated_'+color[1]+'_half','generated':True,'before':a,'after':b,
                                           'before_reason':da.get('reason'),'after_reason':db.get('reason')})
    for name in ('live_complete.png','live_cropped.png'):
        source=read(ROOT/'vision/maixcam_ring/test_data'/name);gray=cv2.cvtColor(source,cv2.COLOR_BGR2GRAY)
        records=[];pair=[]
        for module in (oldring,newring):
            groups,triple,_,status=module.detect_rings(gray)
            observation=None if triple is None else {'ring':triple['ring'],'angle':triple['angle'],'angle_valid':True,'score':triple['score']}
            records.append({'groups':len(groups),'status':status,'center':None if triple is None else triple['ring']['center'],
                            'median_desktop_ms':timed(lambda:module.detect_rings(gray))})
            pair.append(module.annotate(gray,groups,observation,status=status))
        save(Path(name).stem+'_before_after.png',np.hstack(pair))
        report['rings'].append({'name':name,'before':records[0],'after':records[1]})
        reference=oldring.detect_rings(gray)[1]['ring']['center']
        for scenario in ('jitter','moving','lost'):
            records=[]
            for module,require_scene in ((oldring,False),(newring,False),(newring,True)):
                detector,tracker=module.RingDetector(),module.MiddleTracker()
                if require_scene: detector.require_scene=True
                rows=[]
                for i in range(65):
                    dx,dy=(0.,0.) if i<3 else ((float(np.sin(i*2.1)*1.2),float(np.cos(i*1.7)*.8)) if scenario=='jitter' else (float(min(i-3,42)*2),0.))
                    picture=cv2.warpAffine(gray,np.float32([[1,0,dx],[0,1,dy]]),(gray.shape[1],gray.shape[0]),borderValue=int(np.median(gray)))
                    hidden=scenario=='lost' and 25<=i<33
                    if hidden:picture[:]=int(np.median(gray))
                    start=time.perf_counter();groups,triple,_,status=detector.process(picture,tracker,i*.08)
                    if detector.identity_rejected and triple is None:tracker.reset()
                    observed=tracker.update(groups,triple,i*.08,status);detector.accept(observed,triple,picture,i*.08)
                    elapsed=1000*(time.perf_counter()-start)
                    measured=None if observed is None else observed.get('position',observed['ring']['center'])
                    truth=np.array(reference)+(dx,dy)
                    error=None if measured is None else float(np.linalg.norm(np.array(measured)-truth))
                    rows.append({'frame':i,'hidden':hidden,'valid':observed is not None,'roi':detector.used_roi,'error':error,'ms':elapsed,
                                 'current_group_count':len(groups),'wire_valid':observed is not None and (not require_scene or len(groups)==3),
                                 'position':measured,'truth':truth.tolist()})
                valid=[r for r in rows[5:] if r['valid'] and not r['hidden']]
                records.append({'valid_frames':sum(r['valid'] for r in rows),'invalid_hidden_frames':sum(not r['valid'] for r in rows if r['hidden']),
                                'wire_valid_frames':sum(r['wire_valid'] for r in rows),'count_required':require_scene,
                                'hidden_frames':sum(r['hidden'] for r in rows),'rms_error_px':float(np.sqrt(np.mean([r['error']**2 for r in valid]))),
                                'rms_output_jitter_about_static_center_px':float(np.sqrt(np.mean([np.linalg.norm(np.array(r['position'])-reference)**2 for r in valid]))) if scenario=='jitter' else None,
                                'median_desktop_ms':statistics.median(r['ms'] for r in rows),'roi_frames':sum(r['roi'] for r in rows),'frames':rows})
            report['sequences'].append({'name':name+'_'+scenario,'generated':True,'before':records[0],'after':records[1],'service_after':records[2]})
    for name in ('plate_lit.png','plate_unlit.png','plate_weak_rim.png'):
        bgr=cv2.resize(read(ROOT/'vision/maixcam_material/test_data'/name),(320,240),interpolation=cv2.INTER_AREA)
        records=[]
        for module in (oldplate,newplate):
            target,_=module.fit_plate_preview_array(bgr,order='BGR')
            records.append(None if target is None else (target['tangent_x'],target['tangent_y']))
        report['plates'].append({'name':name,'before':records[0],'after':records[1]})
    (OUT/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    compact=dict(report);compact['sequences']=[{**r,'before':{k:v for k,v in r['before'].items() if k!='frames'},
                                                 'after':{k:v for k,v in r['after'].items() if k!='frames'},
                                                 'service_after':{k:v for k,v in r['service_after'].items() if k!='frames'}} for r in report['sequences']]
    print(json.dumps(compact,ensure_ascii=False,indent=2))


if __name__=='__main__':run()
