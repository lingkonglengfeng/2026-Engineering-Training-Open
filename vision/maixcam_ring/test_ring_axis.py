"""图像轴方向的符号/180周期/三环新证据，尚不做车体零位转换。"""
import math
import unittest
import cv2
import numpy as np
import middle_ring as ring
from test_middle_ring import group,make_pattern


class AxisTests(unittest.TestCase):
    def measure(self,groups,confirmed=True):
        triple,_=ring.choose_middle(groups)
        observation=None if not confirmed else dict(angle_valid=True)
        return ring.ring_axis_measurement(groups,triple,observation)
    def test_horizontal_and_known_signed_angles(self):
        for degrees in (-88,-8,-1,0,1,8,88):
            a=math.radians(degrees)
            groups=[group(160+k*95*math.cos(a),120-k*95*math.sin(a),25)for k in (-1,0,1)]
            measurement=self.measure(groups)
            self.assertIsNotNone(measurement)
            self.assertAlmostEqual(measurement['degrees'],degrees,places=6)
            self.assertGreaterEqual(measurement['confidence'],80)
            reverse=self.measure(groups[::-1])
            self.assertAlmostEqual(reverse['degrees'],degrees,places=6)
    def test_same_unoriented_line_does_not_flip_by_180(self):
        points=[group(30,150),group(130,145),group(230,140)]
        self.assertAlmostEqual(self.measure(points)['degrees'],math.degrees(math.atan2(10,200)),places=6)
    def test_two_rings_pending_tracking_only_and_boundary_two_layers_invalid(self):
        groups=[group(x,120,25)for x in (60,160,260)]
        self.assertIsNone(self.measure(groups,False))
        self.assertIsNone(self.measure(groups[:2]))
        groups[2]['boundary_verified']=True
        self.assertIsNone(self.measure(groups))
        groups[2].pop('boundary_verified');groups[2]['levels']=2
        self.assertIsNone(self.measure(groups))
    def test_poor_geometry_short_baseline_and_nonfinite_invalid(self):
        self.assertIsNone(self.measure([group(60,120),group(160,130),group(260,120)]))
        self.assertIsNone(self.measure([group(10,120,8),group(40,120,8),group(70,120,8)]))
        g=[group(x,120,25)for x in (60,160,260)]
        g[1]['score']=float('nan')
        self.assertIsNone(self.measure(g))
    def test_real_cv_on_rotated_marker_rows(self):
        for expected in (-4,0,4):
            a=math.radians(expected)
            centers=tuple((round(240+k*145*math.cos(a)),round(180-k*145*math.sin(a)))for k in (-1,0,1))
            image=make_pattern(size=(480,360),centers=centers,radius=42)
            groups,triple,_,status=ring.detect_rings(image)
            tracker=ring.MiddleTracker()
            for i in range(3):observed=tracker.update(groups,triple,i*.05,status)
            measured=ring.ring_axis_measurement(groups,triple,observed)
            self.assertIsNotNone(measured)
            self.assertAlmostEqual(measured['degrees'],expected,delta=.25)


if __name__=='__main__':unittest.main()
