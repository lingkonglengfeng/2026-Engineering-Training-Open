"""有计数需求时每帧必须重新检测三组，第三环消失当帧全图核对。"""
from pathlib import Path
import unittest

import cv2
import numpy as np
import middle_ring as ring
from test_middle_ring import make_pattern


class SceneCountTests(unittest.TestCase):
    def tick(self,picture,now):
        stats={}
        groups,triple,_,status=self.detector.process(picture,self.tracker,now,stats)
        if self.detector.identity_rejected and triple is None:self.tracker.reset()
        observed=self.tracker.update(groups,triple,now,status)
        self.detector.accept(observed,triple,picture,now)
        return groups,observed,stats
    def acquire(self,picture):
        self.detector,self.tracker=ring.RingDetector(),ring.MiddleTracker()
        self.detector.require_scene=True
        for i in range(3):groups,observed,stats=self.tick(picture,i*.05)
        self.assertIsNotNone(observed)

    def test_three_current_rois_report_three_not_single_tracker_roi(self):
        picture=make_pattern();self.acquire(picture)
        groups,observed,stats=self.tick(picture,.15)
        self.assertEqual(len(groups),3);self.assertIsNotNone(observed)
        self.assertEqual(stats['mode'],'SCENE_ROI')
        self.assertTrue(self.detector.scene_trusted)

    def test_disappearing_third_cannot_reuse_cached_three(self):
        picture=make_pattern();self.acquire(picture)
        picture[:,225:]=220
        groups,observed,stats=self.tick(picture,.15)
        self.assertEqual(len(groups),2)
        self.assertEqual(stats['mode'],'SCENE_FULL')
        self.assertTrue(self.detector.scene_trusted)

    def test_roi_failure_falls_back_for_newly_visible_third(self):
        picture=make_pattern();self.acquire(picture)
        moved=make_pattern(centers=((65,150),(165,150),(265,150)))
        groups,observed,stats=self.tick(moved,.15)
        self.assertEqual(len(groups),3)
        self.assertTrue(self.detector.scene_trusted)

    def test_old_complete_and_partial_photos_have_fresh_three_each_frame(self):
        for name in ('live_complete.png','live_cropped.png'):
            with self.subTest(name=name):
                path=Path(__file__).parent/'test_data'/name
                picture=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)
                self.acquire(picture)
                for i in range(12):
                    groups,observed,_=self.tick(picture,.15+i*.05)
                    self.assertEqual(len(groups),3)
                    self.assertIsNotNone(observed)
                    self.assertTrue(self.detector.scene_trusted)

    def test_blank_is_current_trusted_zero_never_three(self):
        picture=make_pattern();self.acquire(picture)
        groups,observed,stats=self.tick(np.full_like(picture,220),.15)
        self.assertEqual(groups,[]);self.assertIsNone(observed)
        self.assertEqual(stats['mode'],'SCENE_FULL');self.assertTrue(self.detector.scene_trusted)


if __name__=='__main__':unittest.main()
