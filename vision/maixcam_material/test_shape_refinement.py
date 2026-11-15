"""OpenCV真实ROI几何：半圆补全、凹形拒绝和轻量原生分支的接口接入。"""
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import main


class Blob:
    def __init__(self, binary, label=1):
        count, labels, stats, centers = cv2.connectedComponentsWithStats(binary, 8)
        self.box = stats[label]
        self.center = centers[label]
    def x(self): return int(self.box[0])
    def y(self): return int(self.box[1])
    def w(self): return int(self.box[2])
    def h(self): return int(self.box[3])
    def pixels(self): return int(self.box[4])
    def cx(self): return float(self.center[0])
    def cy(self): return float(self.center[1])


class ShapeTests(unittest.TestCase):
    def refine(self, binary):
        blob = Blob(binary)
        details = {}
        initial = main.blob_target(blob, binary.shape[1], binary.shape[0], details, allow_clipped=True)
        if initial is None:
            return None, details
        target, _ = main.refine_material_shape(binary, blob, initial, details)
        return target, details

    def test_half_circle_on_each_single_border_uses_arc_not_centroid(self):
        for center in ((2,120),(317,120),(160,2),(160,237)):
            with self.subTest(center=center):
                binary=np.zeros((240,320),np.uint8)
                cv2.circle(binary,center,30,255,-1)
                target, details=self.refine(binary)
                self.assertIsNotNone(target)
                self.assertEqual(details['reason'],'HALF_OK')
                self.assertLess(np.linalg.norm(np.array(target[:2])-center),1)

    def test_corner_fragment_and_quarter_arc_rejected(self):
        for center in ((2,2),(-23,120)):
            binary=np.zeros((240,320),np.uint8)
            cv2.circle(binary,center,30,255,-1)
            target, _=self.refine(binary)
            self.assertIsNone(target)

    def test_concave_region_passes_old_density_but_fails_new_shape(self):
        binary=np.zeros((240,320),np.uint8)
        cv2.circle(binary,(160,120),30,255,-1)
        cv2.rectangle(binary,(158,90),(169,127),0,-1)
        blob=Blob(binary)
        self.assertIsNotNone(main.blob_target(blob,320,240))
        target, details=self.refine(binary)
        self.assertIsNone(target)
        self.assertIn(details['reason'],('CIRCULARITY','CONVEXITY'))

    def test_holes_and_moderate_ellipse_still_accepted(self):
        binary=np.zeros((240,320),np.uint8)
        cv2.ellipse(binary,(160,120),(34,24),10,0,360,255,-1)
        cv2.circle(binary,(157,115),3,0,-1)
        self.assertIsNotNone(self.refine(binary)[0])

    def test_actual_detection_converts_mask_once_and_caps_candidates(self):
        binary=np.zeros((240,320),np.uint8)
        cv2.circle(binary,(2,120),30,255,-1)
        blob=Blob(binary)
        calls=[]
        mask=SimpleNamespace(width=lambda:320,height=lambda:240,find_blobs=lambda *a,**k:[blob])
        frame=SimpleNamespace(binary=lambda *a,**k:None,to_format=lambda fmt:mask)
        def image2cv(image,**kwargs):
            self.assertIs(image,mask);calls.append(kwargs);return binary[:,:,None]
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray'),image2cv=image2cv)
        target, shown=main.detect_material_center(frame,main.COLORS[0][2],image)
        self.assertIs(shown,mask)
        self.assertIsNotNone(target)
        self.assertEqual(len(calls),1)
        self.assertAlmostEqual(target[0],2,delta=1)

    def test_shape_ranking_prefers_round_material_over_larger_concave_blob(self):
        binary=np.zeros((240,320),np.uint8)
        cv2.circle(binary,(80,120),22,255,-1)
        cv2.circle(binary,(230,120),32,255,-1)
        cv2.rectangle(binary,(229,88),(242,125),0,-1)
        _,_,stats,centers=cv2.connectedComponentsWithStats(binary,8)
        class NativeBlob:
            def __init__(self,s,c): self.box,self.center=s,c
            x=Blob.x;y=Blob.y;w=Blob.w;h=Blob.h;pixels=Blob.pixels;cx=Blob.cx;cy=Blob.cy
        blobs=[NativeBlob(s,c) for s,c in zip(stats[1:],centers[1:])]
        mask=SimpleNamespace(width=lambda:320,height=lambda:240,find_blobs=lambda *a,**k:blobs)
        frame=SimpleNamespace(binary=lambda *a,**k:None,to_format=lambda fmt:mask)
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray'),image2cv=lambda *a,**k:binary)
        target,_=main.detect_material_center(frame,[],image)
        self.assertAlmostEqual(target[0],80,delta=1)

    def test_weak_complete_shape_keeps_native_fallback_but_clipped_does_not(self):
        binary=np.zeros((240,320),np.uint8)
        cv2.circle(binary,(160,120),30,255,-1)
        cv2.rectangle(binary,(158,90),(169,127),0,-1)
        blob=Blob(binary)
        mask=SimpleNamespace(width=lambda:320,height=lambda:240,find_blobs=lambda *a,**k:[blob])
        frame=SimpleNamespace(binary=lambda *a,**k:None,to_format=lambda fmt:mask)
        image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray'),image2cv=lambda *a,**k:binary)
        details={};target,_=main.detect_material_center(frame,[],image,details)
        self.assertIsNotNone(target)
        self.assertEqual(details['reason'],'WEAK_SHAPE')

    def test_old_color_photos_keep_all_previous_complete_detections(self):
        # 彩色截图裁切/居中；LAB与连通域以OpenCV近似，不能替代设备阈值复核。
        for name,expected in [('materials_overview_1.png',set(range(6))),('materials_overview_2.png',set(range(4)))]:
            path=Path(__file__).parent/'test_data'/name
            bgr=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
            lab=cv2.cvtColor(bgr,cv2.COLOR_BGR2LAB).astype(np.float32)
            lab[:,:,0]*=100/255;lab[:,:,1:]-=128
            found=set()
            for i,color in enumerate(main.COLORS):
                binary=cv2.inRange(lab,np.array(color[2][::2],np.float32),np.array(color[2][1::2],np.float32))
                _,_,stats,centers=cv2.connectedComponentsWithStats(binary,8)
                class NativeBlob:
                    def __init__(self,s,c): self.box,self.center=s,c
                    x=Blob.x;y=Blob.y;w=Blob.w;h=Blob.h;pixels=Blob.pixels;cx=Blob.cx;cy=Blob.cy
                blobs=[NativeBlob(s,c) for s,c in zip(stats[1:],centers[1:])]
                mask=SimpleNamespace(width=lambda:320,height=lambda:240,find_blobs=lambda *a,**k:blobs)
                frame=SimpleNamespace(binary=lambda *a,**k:None,to_format=lambda fmt:mask)
                image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE='gray'),image2cv=lambda *a,**k:binary)
                if main.detect_material_center(frame,color[2],image)[0] is not None:found.add(i)
            self.assertEqual(found,expected)


if __name__=='__main__': unittest.main()
