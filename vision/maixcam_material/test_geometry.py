"""电脑端测试：几何、触摸及二值连通域筛选；不代替 MaixCAM 实测。"""

import math
import unittest
import tempfile
import os
import sys
from types import SimpleNamespace
from unittest.mock import patch

import main


class RasterBlob:
    """从合成二值区域计算形状统计，提供与原生 Blob 相同的方法。"""

    def __init__(self, points):
        self.points = points
        self.left = min(x for x, y in points)
        self.top = min(y for x, y in points)
        self.width = max(x for x, y in points) - self.left + 1
        self.height = max(y for x, y in points) - self.top + 1
        self.center_x = sum(x for x, y in points) / len(points)
        self.center_y = sum(y for x, y in points) / len(points)

    def x(self):
        return self.left

    def y(self):
        return self.top

    def w(self):
        return self.width

    def h(self):
        return self.height

    def pixels(self):
        return len(self.points)

    def cx(self):
        return self.center_x

    def cy(self):
        return self.center_y

    def density(self):
        return len(self.points) / (self.width * self.height)

    def roundness(self):
        # 用二阶矩构造测试形状统计；不是 MaixCAM 算法的替代实现。
        vx = sum((x - self.center_x) ** 2 for x, y in self.points) / len(self.points)
        vy = sum((y - self.center_y) ** 2 for x, y in self.points) / len(self.points)
        cov = sum((x - self.center_x) * (y - self.center_y)
                  for x, y in self.points) / len(self.points)
        delta = math.hypot(vx - vy, 2 * cov)
        return (vx + vy - delta) / (vx + vy + delta) if vx + vy else 0


def disk(cx, cy, radius):
    return RasterBlob([(x, y)
                       for y in range(cy - radius, cy + radius + 1)
                       for x in range(cx - radius, cx + radius + 1)
                       if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2])


class GeometryTests(unittest.TestCase):
    def test_light_on_before_camera_and_off_even_if_camera_start_fails(self):
        events=[]
        def camera(*args,**kwargs):
            self.assertIn(('light',1),events)
            raise RuntimeError('camera startup failed')
        gpio=SimpleNamespace(Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_UP=1,PULL_DOWN=2),
            GPIO=lambda *args:SimpleNamespace(value=lambda level:events.append(('light',level))))
        peripheral=SimpleNamespace(gpio=gpio,uart=SimpleNamespace(),
            pinmap=SimpleNamespace(set_pin_function=lambda *args:0))
        display=SimpleNamespace(width=lambda:320,height=lambda:240,close=lambda:None)
        maix=SimpleNamespace(app=SimpleNamespace(),camera=SimpleNamespace(Camera=camera),
            display=SimpleNamespace(Display=lambda:display),image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
            touchscreen=SimpleNamespace(),err=SimpleNamespace(check_raise=lambda *args:None))
        with patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}),patch.object(main,'UART_ENABLED',False):
            with self.assertRaisesRegex(RuntimeError,'camera startup failed'):main.main()
        self.assertEqual(events,[('light',1),('light',0)])

    def test_center_and_signs_with_letterbox(self):
        geometry = main.fit_geometry(320, 240, 552, 368)
        center = main.screen_point(160, 120, 320, 240, geometry)
        self.assertEqual(main.centered_coordinates(*center, 552, 368), (0, 0))
        point = main.screen_point(180, 100, 320, 240, geometry)
        x, y = main.centered_coordinates(*point, 552, 368)
        self.assertGreater(x, 0)
        self.assertGreater(y, 0)

    def test_bottom_left_negative(self):
        self.assertEqual(main.centered_coordinates(240, 220, 552, 368), (-36, -36))

    def test_touch_inverse_and_black_border(self):
        geometry = main.fit_geometry(320, 240, 552, 368)
        screen = main.screen_point(100, 215, 320, 240, geometry)
        fx, fy = main.frame_point(*screen, 320, 240, geometry)
        self.assertAlmostEqual(fx, 100)
        self.assertAlmostEqual(fy, 215)
        self.assertEqual(main.frame_point(0, 200, 320, 240, geometry), (-1, -1))

    def test_touch_inverse_with_top_bottom_border(self):
        geometry = main.fit_geometry(320, 240, 400, 400)
        screen = main.screen_point(100, 215, 320, 240, geometry)
        self.assertEqual(main.frame_point(*screen, 320, 240, geometry), (100, 215))
        self.assertEqual(main.frame_point(200, 20, 320, 240, geometry), (-1, -1))

    def test_touch_hold_release_and_drag(self):
        control = main.TouchButtons()
        buttons = {"color": (0, 200, 100, 40)}
        self.assertEqual(control.update(20, 210, True, 0, buttons), "color")
        for now in (0.1, 0.2, 1):
            self.assertIsNone(control.update(20, 210, True, now, buttons))
        control.update(20, 210, False, 1.1, buttons)
        self.assertEqual(control.update(20, 210, True, 1.2, buttons), "color")
        control.update(20, 210, False, 1.3, buttons)
        self.assertIsNone(control.update(20, 100, True, 1.4, buttons))
        self.assertIsNone(control.update(20, 210, True, 1.5, buttons))
        # 在黑边松开也必须恢复按键状态。
        control.update(-1, -1, False, 1.6, buttons)
        self.assertEqual(control.update(20, 210, True, 1.7, buttons), "color")

    def test_touch_bounce_and_boundaries(self):
        control = main.TouchButtons()
        buttons = {"color": (10, 10, 40, 40)}
        self.assertEqual(control.update(10, 10, True, 0, buttons), "color")
        control.update(10, 10, False, 0.02, buttons)
        self.assertIsNone(control.update(10, 10, True, 0.05, buttons))
        self.assertFalse(main.inside(50, 30, buttons["color"]))

    def test_rule_color_ids(self):
        self.assertEqual([(c[0], c[1]) for c in main.COLORS],
                         [(1, "RED"), (2, "YELLOW"), (3, "BLUE"),
                          (4, "GREEN"), (5, "BLACK"), (6, "LIGHTBLUE")])


class BinaryRegionTests(unittest.TestCase):
    def test_accept_small_and_bottom_circles(self):
        for radius in (20, 34):
            target = main.blob_target(disk(160, 120, radius), 320, 240)
            self.assertIsNotNone(target)
            self.assertEqual(target[:2], (160, 120))
            self.assertAlmostEqual(target[2], radius + 0.5)

    def test_reject_square_region(self):
        square = RasterBlob([(x, y) for y in range(100, 141) for x in range(140, 181)])
        self.assertIsNone(main.blob_target(square, 320, 240))

    def test_reject_thin_ellipse(self):
        ellipse = RasterBlob([(x, y) for y in range(112, 129) for x in range(120, 201)
                              if ((x - 160) / 40) ** 2 + ((y - 120) / 8) ** 2 <= 1])
        self.assertIsNone(main.blob_target(ellipse, 320, 240))

    def test_reject_noise_and_outside_size(self):
        self.assertIsNone(main.blob_target(disk(160, 120, 3), 320, 240))
        self.assertIsNone(main.blob_target(disk(160, 120, 115), 320, 240))

    def test_reject_clipped_region(self):
        clipped = RasterBlob([(x, y) for x, y in disk(5, 120, 20).points if x >= 0])
        self.assertIsNone(main.blob_target(clipped, 320, 240))

    def test_binary_path_no_heavy_calls_and_no_stale_result(self):
        class Mask:
            def __init__(self, blobs):
                self.blobs = blobs
                self.calls = 0

            def width(self):
                return 320

            def height(self):
                return 240

            def find_blobs(self, thresholds, **kwargs):
                self.calls += 1
                self.thresholds = thresholds
                return self.blobs

            def find_circles(self, *args, **kwargs):
                raise AssertionError("Hough must not run")

            def find_edges(self, *args, **kwargs):
                raise AssertionError("Canny must not run")

            def get_pixel(self, *args):
                raise AssertionError("Per-pixel Python sampling must not run")

        class Frame:
            def __init__(self, mask):
                self.mask = mask
                self.binary_done = False

            def binary(self, thresholds, copy):
                self.thresholds = thresholds
                self.copy = copy
                self.binary_done = True
                return self

            def to_format(self, fmt):
                if not self.binary_done:
                    raise AssertionError("Threshold before region detection")
                return self.mask

        mask = Mask([disk(80, 90, 20), disk(180, 130, 34)])
        frame = Frame(mask)
        api = SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE="gray"))
        target, displayed = main.detect_material_center(frame, main.COLORS[0][2], api)
        self.assertEqual(target[:2], (180, 130))
        self.assertIs(displayed, mask)
        self.assertFalse(frame.copy)
        self.assertEqual(mask.calls, 1)
        self.assertEqual(mask.thresholds, [[128, 255]])
        mask.blobs = []
        target, displayed = main.detect_material_center(frame, main.COLORS[1][2], api)
        self.assertIsNone(target)
        self.assertIs(displayed, mask)
        self.assertEqual(frame.thresholds, [main.COLORS[1][2]])

    def test_sdk_roundness_and_density_do_not_reject_valid_region(self):
        region = disk(160, 120, 30)
        region.roundness = lambda: 0
        region.density = lambda: 0
        details = {}
        target = main.blob_target(region, 320, 240, details)
        self.assertEqual(target[:2], (160, 120))
        self.assertEqual(details["reason"], "OK")
        self.assertGreater(details["density"], 0.70)

    def test_rejection_reason(self):
        region = RasterBlob([(x, y) for y in range(100, 141) for x in range(140, 181)])
        details = {}
        self.assertIsNone(main.blob_target(region, 320, 240, details))
        self.assertEqual(details["reason"], "DENSITY")


class ColorCalibrationTests(unittest.TestCase):
    @staticmethod
    def accepts(threshold, lab):
        return all(threshold[2*i] <= lab[i] <= threshold[2*i+1] for i in range(3))

    def test_prefers_chroma_over_brightness_and_disjoint(self):
        samples = {"3": [[25, 20, -45], [30, 24, -48]],
                   "6": [[75, -20, -35], [78, -24, -37]]}
        blue, status = main.calibrated_threshold(main.COLORS[2], samples)
        light, other = main.calibrated_threshold(main.COLORS[5], samples)
        self.assertEqual(status, "LAB_A")
        self.assertEqual(other, status)
        for value in samples["3"]:
            self.assertTrue(self.accepts(blue, value))
            self.assertFalse(self.accepts(light, value))
        for value in samples["6"]:
            self.assertTrue(self.accepts(light, value))
            self.assertFalse(self.accepts(blue, value))
        self.assertTrue(self.accepts(blue, [70, 22, -46]))
        self.assertTrue(self.accepts(light, [20, -22, -36]))

    def test_l_only_is_explicit(self):
        samples = {"3": [[25, 5, -40]], "6": [[65, 6, -41]]}
        _, status = main.calibrated_threshold(main.COLORS[2], samples)
        self.assertEqual(status, "LAB_L")

    def test_missing_pair_and_overlapping_samples(self):
        self.assertEqual(main.blue_separator({"3": [[20, 10, -40]]})[1], "NEED_PAIR")
        samples = {"3": [[25, 10, -40], [50, 15, -30]],
                   "6": [[30, 12, -38], [45, 14, -32]]}
        self.assertEqual(main.blue_separator(samples), (None, "OVERLAP"))

    def test_save_load_and_invalid_data(self):
        samples = {"3": [[25, 20, -45]], "6": [[75, -20, -35]]}
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "colors.json")
            self.assertEqual(main.load_samples(path), {})
            main.save_samples(samples, path)
            self.assertEqual(main.load_samples(path), samples)
        with self.assertRaises(ValueError):
            main.valid_samples({"3": [[25, float("nan"), -40]]})

    def test_sampling_uses_rgb_roi_medians(self):
        class Frame:
            def width(self): return 320
            def height(self): return 240
            def get_statistics(self, roi):
                self.roi = roi
                return SimpleNamespace(l_median=lambda: 42, a_median=lambda: 20,
                                       b_median=lambda: -50)
        frame = Frame()
        self.assertEqual(main.sample_lab(frame, 100, 150), [42, 20, -50])
        self.assertEqual(frame.roi, [94, 144, 12, 12])


if __name__ == "__main__":
    unittest.main()
