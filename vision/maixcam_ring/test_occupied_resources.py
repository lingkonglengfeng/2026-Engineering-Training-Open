"""Occupied-only equivalent math and no double fast scan on successful full verification."""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import middle_ring as ring


class OccupiedResourcesTests(unittest.TestCase):
    def test_solver_same_center_and_radius_with_fewer_native_calls(self):
        rng=np.random.default_rng(10)
        for count in (60,90):
            for arc in (105,210,355):
                angles=np.linspace(0,math.radians(arc),count)
                x=17+42*np.cos(angles)+rng.normal(0,.03,count)
                y=-9+42*np.sin(angles)+rng.normal(0,.03,count)
                old=ring.fit_sample_circle(x,y)
                with patch.object(cv2,'gemm',wraps=cv2.gemm) as gemm:
                    new=ring.fit_occupied_circle(x,y)
                self.assertEqual(gemm.call_count,1)
                self.assertLess(math.dist(old[:2],new[:2]),.0002)
                self.assertLess(abs(old[2]-new[2]),.0002)
        self.assertIsNone(ring.fit_occupied_circle(np.arange(40,dtype=np.float32),np.zeros(40,np.float32)))

    def acquire(self):
        image=cv2.imdecode(np.fromfile(str(Path(__file__).parent/'test_data/occupied_6_live.png'),np.uint8),0)
        detector,tracker=ring.RingDetector(),ring.MiddleTracker();detector.require_scene=True
        for index in range(3):
            now=index*.1
            groups,triple,_,status=detector.process(image,tracker,now)
            observed=tracker.update(groups,triple,now,status)
            detector.accept(observed,triple,image,now)
        self.assertIsNotNone(observed)
        return image,detector,tracker

    def test_periodic_full_success_does_not_first_compute_fast_scene(self):
        image,detector,tracker=self.acquire();detector.last_global=-3
        with patch.object(detector,'fast_occupied_scene',side_effect=AssertionError('duplicated expensive scan')):
            groups,triple,_,_=detector.process(image,tracker,.3)
        self.assertEqual(len(groups),3);self.assertIsNotNone(triple)

    def test_full_failure_retains_current_frame_fast_fallback(self):
        image,detector,tracker=self.acquire();detector.last_global=-3
        with patch.object(ring,'detect_rings',return_value=([],None,None,'NEED_THREE')), \
             patch.object(detector,'fast_occupied_scene',wraps=detector.fast_occupied_scene) as fast:
            groups,triple,_,_=detector.process(image,tracker,.3)
        self.assertEqual(fast.call_count,1);self.assertEqual(len(groups),3);self.assertIsNotNone(triple)


if __name__=='__main__':unittest.main()
