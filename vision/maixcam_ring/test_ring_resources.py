"""原生数值归约、批量采样及稀疏快测的精度/当前证据保护。"""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import middle_ring as ring


class ResourceTests(unittest.TestCase):
    def test_native_circle_solver_recovers_known_circle_and_rejects_line(self):
        rng=np.random.default_rng(28)
        for coverage in (105,210,355):
            angles=np.linspace(0,math.radians(coverage),70,dtype=np.float32)
            x=17.2+42*np.cos(angles)+rng.normal(0,.03,len(angles))
            y=-8.7+42*np.sin(angles)+rng.normal(0,.03,len(angles))
            cx,cy,r=ring.fit_sample_circle(x,y)
            self.assertLess(math.dist((cx,cy),(17.2,-8.7)),.15)
            self.assertAlmostEqual(r,42,delta=.15)
        self.assertIsNone(ring.fit_sample_circle(np.arange(40,dtype=np.float32),np.zeros(40,np.float32)))

    def test_batch_remap_calls_native_once_and_matches_independent_values(self):
        gray=np.random.default_rng(2).integers(0,255,(240,320),dtype=np.uint8)
        seeds=[dict(center=(x,120),radius=30,ellipse=((x,120),(60,60),0)) for x in (60,160,320)]
        real_remap=cv2.remap
        with patch.object(cv2,'remap',wraps=real_remap) as remap:
            batches=ring.occupied_sample_batch(gray,seeds,ring.partial_normalization)
            self.assertEqual(remap.call_count,1)
        self.assertEqual([len(b[0])for b in batches],[60,60,90])
        for sample in batches:
            expected=real_remap(gray,sample[-3],sample[-2],cv2.INTER_LINEAR).astype(np.float32)
            np.testing.assert_array_equal(sample[-1],expected)

    def test_cached_sparse_scene_never_survives_blank_noise_or_floor_lines(self):
        path=Path(__file__).parent/'test_data/occupied_3_cleaned.png'
        original=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
        for kind in ('blank','noise','lines'):
            d,t=ring.RingDetector(),ring.MiddleTracker();d.require_scene=True
            for i in range(3):
                g,tr,_,s=d.process(original,t,i*.08)
                o=t.update(g,tr,i*.08,s);d.accept(o,tr,original,i*.08)
            image=np.full_like(original,180)
            if kind=='noise':image=np.clip(image.astype(np.float32)+np.random.default_rng(9).normal(0,25,image.shape),0,255).astype(np.uint8)
            if kind=='lines':
                for y in range(0,image.shape[0],10):cv2.line(image,(0,y),(image.shape[1]-1,y),25,1)
            groups,triple,_,_=d.process(image,t,.24)
            self.assertIsNone(triple,kind)
            self.assertLess(len(groups),3,kind)


if __name__=='__main__':unittest.main()
