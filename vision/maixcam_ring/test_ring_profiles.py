"""Opt-in occupied work; empty mode never runs the occupied fallback or sampler."""
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import middle_ring as ring
from test_middle_ring import make_pattern


class RingProfileTests(unittest.TestCase):
    def test_empty_full_roi_and_single_tracking_skip_all_occupied_work(self):
        picture=make_pattern(size=(480,360),centers=((80,180),(240,180),(400,180)),radius=45)
        for require_scene in (False,True):
            detector,tracker=ring.RingDetector(),ring.MiddleTracker()
            detector.allow_occupied=False;detector.require_scene=require_scene
            with (patch.object(ring,'recover_occupied_rings',side_effect=AssertionError('empty fallback')),
                 patch.object(ring,'occupied_sample_group',side_effect=AssertionError('empty sample')),
                 patch.object(detector,'fast_occupied_scene',side_effect=AssertionError('empty fast path'))):
                for now in (0,.08,.16,.24):
                    groups,triple,_,status=detector.process(picture,tracker,now)
                    observed=tracker.update(groups,triple,now,status)
                    detector.accept(observed,triple,picture,now)
                self.assertIsNotNone(observed)
                groups,triple,_,_=detector.process(picture,tracker,1.3)
                self.assertEqual(len(groups),3)
                self.assertIsNotNone(triple)
                missing=picture.copy();missing[:,330:]=220
                groups,triple,_,_=detector.process(missing,tracker,1.4)
                self.assertIsNone(triple)

    def test_explicit_empty_does_not_recover_occupied_fixture(self):
        path=Path(__file__).parent/'test_data/occupied_2_cleaned.png'
        gray=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
        with patch.object(ring,'recover_occupied_rings',side_effect=AssertionError('must skip')):
            groups,triple,_,_=ring.detect_rings(gray,occupied=False)
        self.assertIsNone(triple)
        groups,triple,_,_=ring.detect_rings(gray,occupied=True)
        self.assertEqual(len(groups),3);self.assertIsNotNone(triple)
        self.assertTrue(any(g.get('occluded') for g in groups))


if __name__=='__main__':unittest.main()
