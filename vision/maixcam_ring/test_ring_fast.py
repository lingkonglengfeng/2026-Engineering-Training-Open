"""放料后抖动/曝光序列与当前像素丢失；截图覆盖层已清理。"""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import middle_ring as ring


class FastRingTests(unittest.TestCase):
    def image(self,index):
        return cv2.imdecode(np.fromfile(str(Path(__file__).parent/'test_data'/('occupied_%d_cleaned.png'%index)),np.uint8),0)
    def tick(self,d,t,image,now):
        stats={}
        groups,triple,_,status=d.process(image,t,now,stats)
        if d.identity_rejected and triple is None:t.reset()
        observed=t.update(groups,triple,now,status)
        d.accept(observed,triple,image,now)
        return groups,triple,observed,stats
    def test_latest_two_photos_acquire_and_remain_valid_in_varied_sequence(self):
        for index in (4,5):
            for unified in (False,True):
                source=self.image(index)
                d,t=ring.RingDetector(),ring.MiddleTracker();d.require_scene=unified
                fast=0
                for i in range(30):
                    matrix=np.float32([[1,0,1.2*math.sin(i*.6)],[0,1,math.cos(i*.5)]])
                    image=cv2.warpAffine(source,matrix,(source.shape[1],source.shape[0]),borderValue=180)
                    image=np.clip(image.astype(np.float32)*(1+.08*math.sin(i*.7)),0,255).astype(np.uint8)
                    groups,triple,observed,stats=self.tick(d,t,image,i*.08)
                    if i>=3:
                        self.assertEqual(len(groups),3,(index,unified,i))
                        self.assertIsNotNone(observed,(index,unified,i))
                    fast+=stats['mode']=='ARC3_FAST'
                self.assertGreater(fast,20)
    def test_fast_measurement_skips_contours_but_never_reuses_missing_ring(self):
        image=self.image(4);d,t=ring.RingDetector(),ring.MiddleTracker();d.require_scene=True
        for i in range(3):self.tick(d,t,image,i*.08)
        with patch.object(ring,'detect_rings',side_effect=AssertionError('should skip full contours')):
            groups,triple,observed,stats=self.tick(d,t,image,.24)
        self.assertEqual(stats['mode'],'ARC3_FAST')
        self.assertIsNotNone(observed)
        missing=image.copy();missing[:,330:]=180
        groups,triple,observed,stats=self.tick(d,t,missing,.32)
        self.assertEqual(len(groups),2)
        self.assertIsNone(triple)
        self.assertTrue(d.scene_trusted)
    def test_radius_does_not_shrink_over_repeated_partial_layer_measurements(self):
        source=self.image(4);d,t=ring.RingDetector(),ring.MiddleTracker();d.require_scene=True
        radii=[]
        for i in range(16):
            _,triple,_,_=self.tick(d,t,source,i*.08)
            self.assertIsNotNone(triple)
            radii.append(triple['ring'].get('sampling_radius',triple['ring']['radius']))
        self.assertLess(max(radii)-min(radii),3)
    def test_boundary_two_layers_cannot_qualify_without_current_row_context(self):
        source=np.full((240,320),220,np.uint8);center=(345,120)
        for r in (43,48):cv2.circle(source,center,r,25,1,cv2.LINE_AA)
        seed=dict(center=center,radius=48,ellipse=(center,(96,96),0))
        self.assertIsNone(ring.occupied_sample_group(source,seed,ring.partial_normalization,ring.group_concentric,ring.fit_sample_circle))


if __name__=='__main__':unittest.main()
