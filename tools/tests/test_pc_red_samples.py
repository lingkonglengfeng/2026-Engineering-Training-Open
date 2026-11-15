"""Saved red photos: calibration-only update, real legacy and top detectors."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'vision/pc_vision'),str(ROOT/'tools')]
import cv2
import numpy as np
import backend
from update_pc_red_samples import merge_red_samples,apply,REFERENCE

REFERENCE_DATA=json.loads(REFERENCE.read_text(encoding='utf-8'))


class RedSampleTests(unittest.TestCase):
    def test_merge_and_apply_preserve_other_colors_and_original_backup(self):
        original={'version':1,'samples':{'1':[[31,41,22]],'2':[[73,-12,72]],'6':[[50,-9,-27]]},'operator':'kept'}
        result=merge_red_samples(original)
        self.assertEqual(result['samples']['1'],REFERENCE_DATA['samples']['1'])
        self.assertEqual(original['samples']['1'],[[31,41,22]])
        self.assertEqual({k:v for k,v in result['samples'].items() if k!='1'},
                         {k:v for k,v in original['samples'].items() if k!='1'})
        self.assertEqual(result['operator'],'kept')
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'calibration.json';path.write_text(json.dumps(original),encoding='utf-8')
            apply(path);apply(path)
            self.assertEqual(json.loads(path.read_text()),result)
            self.assertEqual(json.loads(path.with_name('calibration.before_red.json').read_text()),original)

    def test_reference_patches_and_both_production_detectors(self):
        samples=REFERENCE_DATA['samples']
        threshold,_=backend.pc_calibrated_threshold(backend.material.COLORS[0],samples,backend.material)
        measured=[]
        for case in REFERENCE_DATA['sources']:
            with self.subTest(photo=case['photo']):
                raw=cv2.imdecode(np.fromfile(str(REFERENCE.parent/case['photo']),np.uint8),1)
                image=cv2.resize(raw,(320,240),interpolation=cv2.INTER_AREA)
                lab=backend.lab_values(image)
                for x,y in case['source_centers']:
                    x,y=round(x*2/3),round(y*2/3)
                    measured.append(np.median(lab[y-3:y+4,x-3:x+4].reshape(-1,3),axis=0))
                diagnostic={}
                target,_=backend.material.detect_material_center(backend.MaterialFrame(image),threshold,backend.MATERIAL_API,diagnostic)
                self.assertIsNotNone(target,diagnostic)
                analyzer=backend.SurfaceAnalyzer(backend.material)
                for frame in range(4):scene=analyzer.process(image,samples,1,stamp=1+frame*.04,all_colors=False,raw_bgr=raw)
                self.assertIsNotNone(scene['target'],analyzer.describe(scene,1))
                self.assertEqual(scene['target']['color'],1)
                self.assertLess(np.linalg.norm(np.array(scene['target']['center'])-case['top_center_320']),6)
        np.testing.assert_allclose(measured,samples['1'],atol=.001)

    def test_red_calibration_rejects_non_red_objects(self):
        threshold,_=backend.pc_calibrated_threshold(backend.material.COLORS[0],REFERENCE_DATA['samples'],backend.material)
        for color in [(0,0,0),(180,180,180),(255,255,255),(0,220,220),(255,0,0),(200,180,0),(0,180,0)]:
            with self.subTest(color=color):
                image=np.full((240,320,3),180,np.uint8);cv2.circle(image,(160,100),30,color,-1)
                target,_=backend.material.detect_material_center(backend.MaterialFrame(image),threshold,backend.MATERIAL_API,{})
                self.assertIsNone(target)

    def test_latest_photo_stop_has_no_detection_but_material_mode_does(self):
        raw=cv2.imdecode(np.fromfile(str(REFERENCE.parent/REFERENCE_DATA['sources'][-1]['photo']),np.uint8),1)
        class Video:
            frame=0
            def get(self,epoch):
                self.frame+=1
                return (epoch,self.frame,1_000_000_000+self.frame*33_333_333,480,360,500),cv2.imencode('.png',raw)[1].tobytes(),1.
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'calibration.json';path.write_text(json.dumps(REFERENCE_DATA),encoding='utf-8')
            obj=backend.PcBackend(Video(),SimpleNamespace(hello={'calibration':{}},epoch=0,frame=(0,0,0)),path,False)
            obj.configure((0,0,0,0,0));result=obj.observe(1.)
            self.assertIsNone(result['coordinates']);self.assertIsNone(obj.surface_report)
            self.assertEqual(obj.status,'STOP / waiting for STM32 SET')
            obj.configure((2,1,1,20,0))
            for i in range(4):result=obj.observe(2.+i*.04)
            self.assertIsNotNone(result['coordinates'])
            self.assertEqual(result['id'],1)
            self.assertTrue(obj.surface_report['confirmed'])
            self.assertEqual(obj.surface_report['output'],'legacy')


if __name__=='__main__':unittest.main()
