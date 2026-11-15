"""遮挡外环当前像素测量、非环反例与放料后的身份跟踪。"""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import middle_ring as ring
from test_middle_ring import make_pattern, group


class OccupiedRingTests(unittest.TestCase):
    def fixture(self, index):
        path = Path(__file__).parent/'test_data'/('occupied_%d_cleaned.png'%index)
        return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE)

    def test_user_occupied_scene_and_cropped_scene_recover_middle(self):
        for index, expected in ((2, (300, 127)), (3, (223, 123))):
            with self.subTest(index=index):
                image = self.fixture(index)
                with patch.object(ring, 'OCCUPIED_RING_RECOVERY_ENABLED', False):
                    _, before, _, _ = ring.detect_rings(image)
                self.assertIsNone(before)
                groups, triple, _, _ = ring.detect_rings(image)
                self.assertEqual(len(groups), 3)
                self.assertIsNotNone(triple)
                self.assertLess(math.dist(triple['ring']['center'], expected), 2)
                self.assertTrue(any(g.get('occluded') for g in groups))
                if index == 2:
                    self.assertTrue(any(g['partial'] for g in groups))

    def test_two_visible_occupied_groups_never_invent_missing_third(self):
        groups, triple, _, _ = ring.detect_rings(self.fixture(1))
        self.assertEqual(len(groups), 2)
        self.assertIsNone(triple)

    def test_occupied_and_cropped_scenes_at_device_processing_resolution(self):
        for index,center in ((2,(300,127)),(3,(223,123))):
            image=self.fixture(index)
            height,width=image.shape
            expected=(center[0]*480/width,center[1]*360/height)
            image=cv2.resize(image,(480,360))
            groups,triple,_,_=ring.detect_rings(image)
            self.assertEqual(len(groups),3)
            self.assertIsNotNone(triple)
            self.assertLess(math.dist(triple['ring']['center'],expected),3)

    def test_screen_coordinates_follow_printed_ring_not_offset_material_top(self):
        image = make_pattern(size=(480,360), centers=((80,180),(240,180),(400,180)), radius=50)
        for center in ((80,180),(240,180),(400,180)):
            cv2.circle(image, center, 28, 30, -1)
            cv2.circle(image, (center[0]+35, center[1]), 20, 110, -1)
        groups, triple, _, _ = ring.detect_rings(image)
        self.assertEqual(len(groups), 3)
        self.assertLess(math.dist(triple['ring']['center'], (240,180)), 2)
        self.assertLess(math.hypot(*ring.screen_error(triple['ring']['center'], (480,360),(320,240))), 2)

    def test_light_variation_on_occupied_screenshot(self):
        original = self.fixture(3)
        for gain in (.8, 1.1):
            image = np.clip(original.astype(float)*gain, 0, 255).astype(np.uint8)
            _, triple, _, _ = ring.detect_rings(image)
            self.assertIsNotNone(triple)
            self.assertLess(math.dist(triple['ring']['center'], (223,123)), 3)

    def sample(self, image, seed):
        return ring.occupied_sample_group(image, seed, ring.partial_normalization, ring.group_concentric, ring.fit_sample_circle)

    def test_missing_pixels_do_not_turn_historical_seed_into_a_target(self):
        image = np.full((240,320), 180, np.uint8)
        for x in (160, -10, 325):
            self.assertIsNone(self.sample(image, group(x,120,40)))

    def test_single_double_and_solid_circles_are_not_multilayer_markers(self):
        for variant in ('one','two','solid'):
            image = np.full((240,320), 220, np.uint8)
            if variant == 'solid':
                cv2.circle(image,(160,120),40,25,-1)
            else:
                for radius in ((40,) if variant=='one' else (40,31)):
                    cv2.circle(image,(160,120),radius,25,1,cv2.LINE_AA)
            self.assertIsNone(self.sample(image,group(160,120,40)),variant)

    def test_noise_and_floor_stripes_not_outer_layers(self):
        for sigma in (3,10,25):
            for seed in range(6):
                image = np.clip(180+np.random.default_rng(seed).normal(0,sigma,(240,320)),0,255).astype(np.uint8)
                self.assertIsNone(self.sample(image,group(160,120,40)))
        image=np.full((240,320),220,np.uint8)
        for y in range(60,180,8):
            cv2.line(image,(100,y),(220,y),25,1)
        self.assertIsNone(self.sample(image,group(160,120,40)))

    def test_short_arc_and_quarter_at_corner_not_sufficient(self):
        for center in ((160,120),(0,0)):
            image=np.full((240,320),220,np.uint8)
            for fraction in (65/95,75/95,85/95,1):
                cv2.ellipse(image,center,(round(40*fraction),)*2,0,0,90,25,1)
            self.assertIsNone(self.sample(image,group(*center,40)))

    def test_full_scene_roi_survives_material_hiding_digits(self):
        image=make_pattern()
        detector, tracker=ring.RingDetector(),ring.MiddleTracker()
        detector.require_scene=True
        for i in range(3):
            groups,triple,_,status=detector.process(image,tracker,i*.05)
            observed=tracker.update(groups,triple,i*.05,status)
            detector.accept(observed,triple,image,i*.05)
        occupied=image.copy()
        for x in (60,160,260):
            cv2.circle(occupied,(x,120),13,35,-1)
        # 三环均来自当前帧，即使中心图案被遮挡/无可用模板也可保持全组三环ROI。
        self.assertLess(detector.template_score(occupied,tracker.lock),ring.ROI_TEMPLATE_MIN_SCORE)
        detector.template=None
        groups,triple,_,_=detector.process(occupied,tracker,.15)
        self.assertEqual(len(groups),3)
        self.assertIsNotNone(triple)
        self.assertTrue(detector.used_roi)
        self.assertTrue(detector.scene_trusted)
        # 抹掉一环后不能复用ROI历史位置；该帧全图确认数量2。
        occupied[:,225:]=220
        groups,triple,_,_=detector.process(occupied,tracker,.20)
        self.assertEqual(len(groups),2)
        self.assertIsNone(triple)
        self.assertFalse(detector.used_roi)

    def test_unchanged_complete_scene_skips_occupied_algorithm(self):
        with patch.object(ring,'recover_occupied_rings',side_effect=AssertionError('unexpected slow fallback')):
            self.assertIsNotNone(ring.detect_rings(make_pattern())[1])

    def test_helper_embedding_matches_source(self):
        source=(Path(__file__).parent/'occupied_ring.py').read_text(encoding='utf-8')
        standalone=(Path(__file__).parent/'middle_ring.py').read_text(encoding='utf-8')
        helper=source[source.index('def occupied_sample_group'):].rstrip()
        embedded=standalone.split('# BEGIN OCCUPIED_RING\n',1)[1].split('# END OCCUPIED_RING\n',1)[0].rstrip()
        self.assertEqual(helper,embedded)


if __name__=='__main__':
    unittest.main()
