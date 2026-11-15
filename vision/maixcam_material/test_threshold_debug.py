"""Check the generated debug app uses the production detector and shows live limits."""
from pathlib import Path
from types import SimpleNamespace
import unittest

import material_threshold_debug as debug


class FakeImage:
    COLOR_BLACK = 0
    COLOR_WHITE = 1
    COLOR_GREEN = 2
    COLOR_YELLOW = 3

    def __init__(self):
        self.strings = []
        self.rects = []

    def width(self):
        return 320

    def height(self):
        return 240

    def draw_rect(self, *args, **kwargs):
        self.rects.append((args, kwargs))

    def draw_circle(self, *args, **kwargs):
        pass

    def draw_line(self, *args, **kwargs):
        pass

    def draw_string(self, x, y, value, color):
        self.strings.append(value)


class ThresholdDebugTests(unittest.TestCase):
    def test_bundled_detector_is_exact_production_source(self):
        source = (Path(__file__).parent / "main.py").read_text(encoding="utf-8")
        self.assertEqual(debug._BUNDLED_MATERIAL, source)

    def test_threshold_display_uses_calibrated_values(self):
        core = debug.load_detector()
        color = core["COLORS"][0]
        samples = {"1": [[50, 55, 22]]}
        threshold, status = core["calibrated_threshold"](color, samples)
        self.assertEqual(threshold, [30, 70, 37, 73, 4, 40])
        self.assertEqual(status, "SAMPLED")
        first, second = debug.threshold_lines(color, threshold, status)
        self.assertIn("L:30..70 A:37..73", first)
        self.assertIn("B:4..40", second)
        self.assertIn("SAMPLED", second)

    def test_result_and_thresholds_stay_visible_during_sampling(self):
        shown = FakeImage()
        buttons = {name: (i * 80, 202, 76, 34)
                   for i, name in enumerate(("color", "view", "learn", "lock"))}
        debug.draw_view(shown, (170, 115, 20), {"reason": "OK"},
                        (1, "RED", []), [15, 85, 25, 127, -10, 100], "DEFAULT",
                        10.0, True, True, False, 2, "SAMPLE 50,55,22",
                        FakeImage, buttons)
        self.assertTrue(any("L:15..85" in line for line in shown.strings))
        self.assertTrue(any("dx:+10.0 dy:-5.0 OK" in line for line in shown.strings))
        self.assertTrue(any("TX:OFF" in line for line in shown.strings))
        self.assertIn("SAMPLE 50,55,22", shown.strings)
        self.assertIn("Color", shown.strings)
        self.assertIn("Sample", shown.strings)

    def test_window_is_visible_in_raw_and_binary_and_does_not_claim_grab(self):
        detector = debug.load_detector()
        for raw in (True, False):
            shown = FakeImage()
            debug.draw_view(shown, (160, 90, 20), {"reason": "OK"},
                            (1, "RED", []), [15, 85, 25, 127, -10, 100], "DEFAULT",
                            10, raw, False, False, 0, "", FakeImage, {},
                            detector["draw_pick_window"])
            self.assertTrue(any(args[:4] == (130, 50, 81, 81) for args, kw in shown.rects))
            self.assertIn("RED IN (VIEW ONLY)", shown.strings)
            self.assertIn("WIN 10,-30 +/-40,40 LOCAL", shown.strings)
            self.assertEqual(shown.rects[0][0][:4], (0, 0, 320, 50))


if __name__ == "__main__":
    unittest.main()
