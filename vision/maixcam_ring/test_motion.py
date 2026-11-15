"""运动预测仅用于ROI；平滑必须跟随当前有效测量，丢失/复位不输出历史点。"""
import math
import unittest
from unittest.mock import patch

import middle_ring as ring
from test_middle_ring import group


class MotionTests(unittest.TestCase):
    def acquire(self):
        tracker=ring.MiddleTracker()
        for i in range(3):
            groups=[group(x,120,30) for x in (60,160,260)]
            triple,status=ring.choose_middle(groups)
            tracker.update(groups,triple,i*.05,status)
        return tracker

    def test_small_jitter_reduced_without_changing_raw_center(self):
        tracker=self.acquire()
        raw, smooth=[],[]
        for i in range(40):
            target=group(160+(1 if i%2 else -1),120,30)
            observed=tracker.update([target],None,.15+i*.05)
            self.assertEqual(observed['ring']['center'],target['center'])
            self.assertLessEqual(math.dist(observed['position'],target['center']),ring.SMOOTH_MAX_LAG_PX+1e-9)
            raw.append(target['center'][0]-160);smooth.append(observed['position'][0]-160)
        self.assertLess(sum(x*x for x in smooth),.6*sum(x*x for x in raw))

    def test_fast_movement_bypasses_smoothing_and_search_leads_motion(self):
        tracker=self.acquire()
        for i in range(4):
            target=group(164+4*i,120,30)
            observed=tracker.update([target],None,.15+i*.05)
            self.assertEqual(observed['position'],target['center'])
        predicted=tracker.predicted_center(.35)
        self.assertGreater(predicted[0],tracker.lock['center'][0])
        self.assertLessEqual(math.dist(predicted,tracker.lock['center']),10.5)

    def test_loss_never_returns_prediction_and_reacquisition_starts_fresh(self):
        tracker=self.acquire()
        tracker.update([group(164,120,30)],None,.15)
        self.assertIsNone(tracker.update([],None,.20))
        self.assertIsNone(tracker.filtered_center)
        self.assertEqual(tracker.velocity,(0,0))
        observed=tracker.update([group(166,120,30)],None,.25)
        self.assertEqual(observed['position'],(166,120))
        tracker.reset()
        self.assertIsNone(tracker.predicted_center(.30))
        self.assertIsNone(tracker.filter_time)

    def test_ambiguity_discards_filter_and_velocity(self):
        tracker=self.acquire()
        tracker.update([group(164,120,30)],None,.15)
        self.assertIsNone(tracker.update([],None,.20,'AMBIGUOUS'))
        self.assertIsNone(tracker.filtered_center)

    def test_low_update_rate_does_not_build_extra_delay(self):
        tracker=self.acquire()
        observed=tracker.update([group(161,120,30)],None,.29)
        self.assertEqual(observed['position'],(161,120))

    def test_prediction_is_bounded_even_with_anomalous_velocity(self):
        tracker=self.acquire();tracker.velocity=(1e6,-1e6)
        self.assertLessEqual(math.dist(tracker.predicted_center(10),tracker.lock['center']),10.5+1e-9)


if __name__=='__main__': unittest.main()
