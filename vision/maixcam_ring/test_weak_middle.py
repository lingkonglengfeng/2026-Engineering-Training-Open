"""补光弱中环：必须当前像素支持多层圆；不凭两侧中点补出不存在的环。"""
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import middle_ring as ring
from test_middle_ring import make_pattern,group


class WeakMiddleTests(unittest.TestCase):
    def fixture(self,index):
        path=Path(__file__).parent/'test_data'/('live_glare_%d_cleaned.png'%index)
        return cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)

    def test_both_glare_screenshots_recover_measured_middle(self):
        # 截图十字/彩色标注已按同一方法清理；并非原始传感器图。
        for index,center in [(1,(252,119)),(2,(193,118))]:
            with self.subTest(index=index):
                image=self.fixture(index)
                with patch.object(ring,'WEAK_MIDDLE_RECOVERY_ENABLED',False), patch.object(ring,'OCCUPIED_RING_RECOVERY_ENABLED',False):
                    before,old,_,_=ring.detect_rings(image)
                self.assertEqual(len(before),2);self.assertIsNone(old)
                groups,middle,_,_=ring.detect_rings(image)
                self.assertEqual(len(groups),3);self.assertIsNotNone(middle)
                self.assertTrue(middle['ring']['weak_recovered'])
                self.assertLess(math.dist(middle['ring']['center'],center),1.5)

    def test_recovery_then_scene_tracking_always_counts_fresh_three(self):
        detector,tracker=ring.RingDetector(),ring.MiddleTracker()
        detector.require_scene=True
        image=self.fixture(2)
        for i in range(8):
            groups,triple,_,status=detector.process(image,tracker,i*.08)
            observed=tracker.update(groups,triple,i*.08,status)
            detector.accept(observed,triple,image,i*.08)
            self.assertEqual(len(groups),3)
            if i>=2:self.assertIsNotNone(observed)
        # 抹掉中心证据后必须退回当前两环，不可保留刚恢复的第三组。
        hidden=image.copy();hidden[55:183,129:257]=180
        groups,triple,_,_=detector.process(hidden,tracker,.70)
        self.assertEqual(len(groups),2);self.assertIsNone(triple)

    def test_glare_fixture_brightness_changes_keep_current_middle(self):
        for index in (1,2):
            original=self.fixture(index)
            for gain in (.8,1.12):
                image=np.clip(original.astype(float)*gain,0,255).astype(np.uint8)
                groups,middle,_,_=ring.detect_rings(image)
                self.assertEqual(len(groups),3,(index,gain))
                self.assertIsNotNone(middle,(index,gain))

    def side_scene(self):
        picture=np.full((360,480),220,np.uint8)
        for center in ((90,180),(390,180)):
            for ratio in (.54,.60,.68,.78,.89,1):
                cv2.circle(picture,center,round(48*ratio),25,1,cv2.LINE_AA)
        return picture,[group(90,180,48),group(390,180,48)]

    def test_no_middle_single_double_rings_and_digit_do_not_create_third(self):
        for variant in ('blank','one','two','digit','filled'):
            with self.subTest(variant=variant):
                image,groups=self.side_scene()
                if variant in ('one','two'):cv2.circle(image,(240,180),43,80,1)
                if variant=='two':cv2.circle(image,(240,180),37,80,1)
                if variant=='digit':cv2.putText(image,'2',(228,193),cv2.FONT_HERSHEY_SIMPLEX,1,30,2)
                if variant=='filled':cv2.circle(image,(240,180),48,50,-1)
                self.assertIsNone(ring.recover_weak_middle(image,groups))

    def test_noise_and_floor_lines_cannot_supply_multilayer_evidence(self):
        for sigma in (2,5,10):
            for seed in range(8):
                image,groups=self.side_scene()
                noise=np.random.default_rng(seed).normal(0,sigma,(360,480))
                image=np.clip(image.astype(float)+noise,0,255).astype(np.uint8)
                self.assertIsNone(ring.recover_weak_middle(image,groups),(sigma,seed))
        image,groups=self.side_scene()
        for y in range(120,241,8):cv2.line(image,(175,y),(305,y),70,1)
        self.assertIsNone(ring.recover_weak_middle(image,groups))

    def test_adjacent_two_rings_never_use_their_midpoint(self):
        image,groups=self.side_scene()
        groups[1]=group(240,180,48)
        self.assertIsNone(ring.recover_weak_middle(image,groups))

    def test_short_visible_middle_arc_is_rejected(self):
        image,groups=self.side_scene()
        for fraction in (65/95,75/95,85/95,1):
            cv2.ellipse(image,(240,180),(round(48*fraction),round(48*fraction)),0,0,100,30,1)
        self.assertIsNone(ring.recover_weak_middle(image,groups))

    def test_output_follows_real_ring_center_not_geometry_midpoint(self):
        image,groups=self.side_scene()
        for fraction in (65/95,75/95,85/95,1):
            cv2.circle(image,(242,179),round(47.5*fraction),80,1,cv2.LINE_AA)
        result=ring.recover_weak_middle(image,groups)
        self.assertIsNotNone(result)
        self.assertLess(math.dist(result['center'],(242,179)),1)
        self.assertGreater(math.dist(result['center'],(240,180)),1)


if __name__=='__main__':unittest.main()
