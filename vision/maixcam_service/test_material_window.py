"""Preview geometry, mode isolation and unchanged MAT_PX bytes."""
from types import SimpleNamespace
import unittest
import service


class Frame:
    def __init__(self, width=320, height=240, fmt="rgb"):
        self.w, self.h, self.fmt = width, height, fmt
        self.rects, self.lines, self.strings = [], [], []
    def width(self): return self.w
    def height(self): return self.h
    def format(self): return self.fmt
    def to_format(self, fmt):
        self.fmt = fmt
        return self
    def draw_rect(self, *args, **kw): self.rects.append((args, kw))
    def draw_line(self, *args, **kw): self.lines.append((args, kw))
    def draw_circle(self, *args, **kw): pass
    def draw_string(self, x, y, text, color): self.strings.append(text)


IMAGE = SimpleNamespace(COLOR_GREEN=1, COLOR_YELLOW=2, COLOR_WHITE=3,
                        Format=SimpleNamespace(FMT_RGB888="rgb"),
                        Fit=SimpleNamespace(FIT_CONTAIN="contain"),
                        Image=lambda w, h, fmt: Frame(w, h, fmt))


class MaterialWindowTests(unittest.TestCase):
    def setUp(self): self.material = service.detector_module("material")

    def test_inclusive_mcu_boundaries_and_outside(self):
        self.assertEqual(self.material.pick_window_geometry(320, 240), (170, 90, 40, 40))
        for x, y in ((130, 50), (210, 130), (130, 130), (210, 50), (170, 90)):
            self.assertEqual(self.material.pick_window_state((x, y, 8), 1, 320, 240), "IN")
        for x, y in ((129.9, 90), (210.1, 90), (170, 49.9), (170, 130.1)):
            self.assertEqual(self.material.pick_window_state((x, y, 8), 1, 320, 240), "OUT")
        self.assertEqual(self.material.pick_window_state(None, 1, 320, 240), "NO TARGET")
        self.assertEqual(self.material.pick_window_state((160, 90, 8), 2, 320, 240), "OTHER COLOR")

    def test_box_and_center_follow_processing_image_scale(self):
        for width, height, box, cross in ((320, 240, (130, 50, 81, 81), (164, 90, 176, 90)),
                                          (640, 480, (260, 100, 161, 161), (334, 180, 346, 180))):
            frame = Frame(width, height)
            self.material.draw_pick_window(frame, None, 1, IMAGE)
            self.assertEqual(frame.rects[0][0][:4], box)
            self.assertEqual(frame.lines[0][0][:4], cross)
            self.assertIn("RED NO TARGET (VIEW ONLY)", frame.strings)

    def test_contain_mapping_with_portrait_and_landscape_black_bars(self):
        for sw, sh in ((552, 368), (368, 552), (640, 480)):
            geometry = self.material.fit_geometry(320, 240, sw, sh)
            for point in ((120, 50), (200, 130), (160, 90)):
                screen = self.material.screen_point(*point, 320, 240, geometry)
                recovered = self.material.frame_point(*screen, 320, 240, geometry)
                for actual, expected in zip(recovered, point):
                    self.assertAlmostEqual(actual, expected)

    def test_preview_converts_mask_to_rgb_and_never_changes_wire_position(self):
        output = []
        backend = service.MaixBackend(None, IMAGE,
            SimpleNamespace(show=lambda frame, **kw: output.append((frame, kw))))
        backend.config = (2, 1, 1, 20, 0)
        backend.module = self.material
        backend.selected_color = 1
        backend.material_target = (160, 90, 8)
        backend.view = Frame(fmt="gray")
        observation = {"coordinates": (0, 46), "scale": 46/30, "id": 1}
        before = service.encode_pixel_frame(2, 1, observation["coordinates"], 3, observation["scale"])
        backend.preview(backend.config, observation)
        self.assertEqual(backend.displayed.format(), "rgb")
        self.assertIn("RED IN (VIEW ONLY)", backend.displayed.strings)
        self.assertEqual(output[0][1]["fit"], "contain")
        after = service.encode_pixel_frame(2, 1, observation["coordinates"], 3, observation["scale"])
        self.assertEqual(before, after)
        self.assertEqual(before[3:7], bytes.fromhex("00 00 d4 fe"))  # dx0, dy-30 x10
        backend.material_target = None
        backend.view = Frame()
        backend.preview(backend.config, {"coordinates": None})
        self.assertIn("RED NO TARGET (VIEW ONLY)", backend.displayed.strings)

    def test_stop_plate_and_invalidation_clear_previous_material_marker(self):
        backend = service.MaixBackend(None, IMAGE, SimpleNamespace(show=lambda *a, **kw: None))
        backend.material_target = (160, 90, 8)
        backend.view = Frame()
        backend.configure((0, 0, 0, 0, 0))
        self.assertIsNone(backend.material_target)
        self.assertIsNone(backend.view)
        backend.config = (8, 1, 0, 20, 0)
        backend.view = Frame()
        backend.preview(backend.config, {"coordinates": None})
        self.assertFalse(backend.displayed.rects)
        backend.material_target = (160, 90, 8)
        backend.invalidate()
        self.assertIsNone(backend.material_target)


if __name__ == "__main__": unittest.main()
