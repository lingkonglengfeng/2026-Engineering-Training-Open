"""真实CV+假Maix接口：原物料算法已内嵌，Plate65切红色不串目标偏置。"""
import ast
from pathlib import Path
import struct
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import service
from test_service import NativeImage


class MaterialIntegrationTests(unittest.TestCase):
    def test_upload_contains_exact_original_material_source(self):
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'maixcam_service/main.py').read_text(encoding='utf-8'))
        assignment=next(n for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name)and t.id=='_BUNDLED_DETECTORS' for t in n.targets))
        bundled=ast.literal_eval(assignment.value)
        self.assertEqual(bundled['material'],(root/'maixcam_material/main.py').read_text(encoding='utf-8'))
    def test_plate65_switches_to_red_and_reports_center_then_loss(self):
        root=Path(__file__).resolve().parents[1]
        plate=cv2.imdecode(np.fromfile(str(root/'maixcam_material/test_data/plate_lit.png'),np.uint8),1)
        scene=[cv2.resize(cv2.cvtColor(plate,cv2.COLOR_BGR2RGB),(480,360))]
        cameras=[]
        class Blob:
            def __init__(self,stats,center):self.stats,self.center=stats,center
            def x(self):return int(self.stats[0])
            def y(self):return int(self.stats[1])
            def w(self):return int(self.stats[2])
            def h(self):return int(self.stats[3])
            def pixels(self):return int(self.stats[4])
            def cx(self):return float(self.center[0])
            def cy(self):return float(self.center[1])
        class Frame(NativeImage):
            def __init__(self,array):self.array=array
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def copy(self):return Frame(self.array.copy())
            def draw_string(self,*args,**kwargs):pass
            def draw_circle(self,*args,**kwargs):pass
            def draw_ellipse(self,*args,**kwargs):pass
            def draw_line(self,*args,**kwargs):pass
            def draw_rect(self,*args,**kwargs):pass
            def binary(self,thresholds,copy=False):
                lab=cv2.cvtColor(self.array,cv2.COLOR_RGB2LAB).astype(np.float32)
                lab[:,:,0]*=100/255;lab[:,:,1:]-=128
                mask=np.zeros(lab.shape[:2],bool)
                for l0,l1,a0,a1,b0,b1 in thresholds:
                    mask|=(lab[:,:,0]>=l0)&(lab[:,:,0]<=l1)&(lab[:,:,1]>=a0)&(lab[:,:,1]<=a1)&(lab[:,:,2]>=b0)&(lab[:,:,2]<=b1)
                self.array=np.repeat((mask.astype(np.uint8)*255)[:,:,None],3,axis=2)
                return self
            def to_format(self,fmt):
                if fmt==self.format():return self
                if fmt=='gray':return Frame(cv2.cvtColor(self.array,cv2.COLOR_RGB2GRAY))
                return Frame(np.repeat(self.array[:,:,None],3,axis=2))
            def find_blobs(self,thresholds,**kwargs):
                count,labels,stats,centers=cv2.connectedComponentsWithStats(self.array,8)
                return [Blob(stats[i],centers[i])for i in range(1,count) if stats[i,4]>=kwargs['pixels_threshold']]
        class Camera:
            def __init__(self,*args,**kwargs):cameras.append(self)
            def read(self,**kwargs):return Frame(scene[0].copy())
            def close(self):pass
            def clear_buff(self):pass
        image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb',FMT_GRAYSCALE='gray'),
            Fit=SimpleNamespace(FIT_CONTAIN='contain'),COLOR_WHITE=255,COLOR_YELLOW=1,COLOR_RED=2,COLOR_GREEN=3,
            Image=lambda w,h,fmt:Frame(np.zeros((h,w,3),np.uint8)),
            image2cv=lambda frame,**kwargs:frame.array,cv2image=lambda array,**kwargs:Frame(array.copy()))
        display=SimpleNamespace(width=lambda:552,height=lambda:368,show=lambda *args,**kwargs:0)
        backend=service.MaixBackend(SimpleNamespace(Camera=Camera),image,display)
        material=service.detector_module('material')
        try:
            backend.configure((8,1,0,20,0))
            for _ in range(3):plate_observation=backend.observe(time.monotonic())
            self.assertIsNotNone(plate_observation['coordinates'])
            with patch.object(material,'load_samples',return_value={}):
                backend.configure((2,1,1,20,0))
            def set_scene(red_center):
                scene[0]=np.full((360,480,3),180,np.uint8)
                cv2.circle(scene[0],(380,180),40,(30,180,50),-1)
                if red_center is not None:cv2.circle(scene[0],red_center,30,(220,40,40),-1)
            for center,expected in (((240,180),(0,0)),((300,225),(40,30))):
                set_scene(center)
                # 原物料模式首帧输出，不额外等待三帧而错过运动目标。
                observation=backend.observe(time.monotonic())
                self.assertEqual(observation['id'],1)
                self.assertIsNotNone(observation['coordinates'])
                wire=service.encode_pixel_frame(2,1,observation['coordinates'],3,observation['scale'])
                x,y=struct.unpack('<hh',wire[3:7])
                self.assertAlmostEqual(x/10,expected[0],delta=1)
                self.assertAlmostEqual(y/10,expected[1],delta=1)
                backend.preview(backend.config,observation)
            set_scene(None)
            observation=backend.observe(time.monotonic())
            self.assertIsNone(observation['coordinates'])
            self.assertEqual(observation['id'],1)
            self.assertEqual(len(cameras),1)
        finally:backend.close()


if __name__=='__main__':unittest.main()
