"""覆盖切到 PLATE 后首次绘图、单文件运行和异常失效路径。"""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import main
import plate_tangent as plate
from test_uart import FakeUART, decode, packets


class Frame:
    def __init__(self, rgb=None):
        self.rgb = rgb
        self.ellipses = []
        self.texts = []

    def width(self):
        return 320

    def height(self):
        return 240

    # 按 MaixPy v4 的必填参数定义，旧调用会直接抛 TypeError。
    def draw_ellipse(self, x, y, a, b, angle, start_angle, end_angle, color, thickness=1):
        self.ellipses.append((start_angle, end_angle, color))

    def draw_line(self, *args, **kwargs):
        pass

    def draw_rect(self, *args, **kwargs):
        pass

    def draw_string(self, *args, **kwargs):
        self.texts.append(args[2])


def image_api():
    return SimpleNamespace(COLOR_WHITE="white", COLOR_BLACK="black", COLOR_YELLOW="yellow",
                           COLOR_GREEN="green", COLOR_RED="red",
                           Fit=SimpleNamespace(FIT_CONTAIN="contain"),
                           Format=SimpleNamespace(FMT_RGB888="rgb"),
                           Image=lambda *args: Frame(),
                           image2cv=lambda frame, ensure_bgr, copy: frame.rgb)


def fixture(name):
    path = Path(__file__).parent / "test_data" / name
    return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)


class StandaloneTests(unittest.TestCase):
    def test_weak_rim_screenshot_resists_scale_and_brightness_changes(self):
        base=cv2.resize(fixture('plate_weak_rim.png'),(320,240),interpolation=cv2.INTER_AREA)
        versions=(base,np.clip(base.astype(float)*.80,0,255).astype(np.uint8),
                  np.clip(base.astype(float)*1.12,0,255).astype(np.uint8),cv2.GaussianBlur(base,(3,3),.7))
        for width in (224,320):
            for index,array in enumerate(versions):
                with self.subTest(width=width,index=index),patch.object(plate,'PROCESS_WIDTH',width):
                    details={};result,_=plate.fit_plate_preview_array(array,details,order='BGR')
                    self.assertIsNotNone(result)
                    self.assertAlmostEqual(result['tangent_y'],109,delta=2)
                    self.assertAlmostEqual(result['tangent_x'],186,delta=3)
                    self.assertEqual(details['search'],'ARC_WINDOW')

    def test_window_fallback_reuses_precomputed_edge_filters(self):
        with patch.object(plate,'PLATE_SEARCH_TOP_FRACTION',.90), \
                patch.object(cv2,'Canny',wraps=cv2.Canny) as canny, \
                patch.object(cv2,'Sobel',wraps=cv2.Sobel) as sobel:
            details={};result,_=plate.fit_plate_color_array(fixture('plate_lit.png'),details,order='BGR')
        self.assertIsNotNone(result)
        self.assertEqual(details['search'],'FULL_FALLBACK')
        self.assertEqual(canny.call_count,1)
        self.assertEqual(sobel.call_count,2)

    def test_combined_first_plate_draw_matches_maixpy_signature(self):
        result, _ = main.fit_plate_color_array(fixture("plate_lit.png"), order="BGR")
        frame = Frame()
        buttons = {name: (0, 200, 100, 32) for name in ("color", "learn", "lock")}
        main.draw_screen(frame, None, main.fit_geometry(320, 240, 552, 368),
                         552, 368, (0,"PLATE",main.PLATE_WHITE_LAB), 20, buttons, image_api(),
                         {"count": 1, "reason": "EDGE_OK"}, False, "COLOR_EDGE", 0,
                         False, "", plate_target=result)
        self.assertEqual(frame.ellipses, [(0, 360, "yellow")])

    def test_standalone_first_plate_draw_matches_maixpy_signature(self):
        result, _ = plate.fit_plate_color_array(fixture("plate_lit.png"), order="BGR")
        frame = Frame()
        plate.draw_plate_overlay(frame, result, image_api())
        self.assertEqual(frame.ellipses, [(0, 360, "yellow")])

    def test_both_real_photos_keep_existing_tangent(self):
        for name, expected in (("plate_lit.png", 138), ("plate_unlit.png", 136)):
            with self.subTest(name=name):
                original, _ = main.fit_plate_color_array(fixture(name), order="BGR")
                independent, _ = plate.fit_plate_color_array(fixture(name), order="BGR")
                self.assertIsNotNone(independent)
                self.assertAlmostEqual(independent["tangent_y"], expected, delta=4)
                self.assertEqual(independent, original)

    def test_single_file_has_no_project_imports(self):
        import ast
        tree = ast.parse(Path(plate.__file__).read_text(encoding="utf-8"))
        imports = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        imports.update(alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names)
        self.assertFalse(imports.intersection({"main", "test_uart", "test_plate"}))

    def test_fast_photos_restore_original_coordinates_and_keep_input(self):
        for name in ("plate_lit.png", "plate_unlit.png"):
            with self.subTest(name=name):
                array = fixture(name)
                before = array.copy()
                reference, _ = plate.fit_plate_color_array(array, order="BGR")
                fast, edges = plate.fit_plate_preview_array(array, order="BGR", edges_view=True)
                self.assertIsNotNone(fast)
                self.assertAlmostEqual(fast["tangent_y"], reference["tangent_y"], delta=1)
                self.assertEqual(edges.shape, array.shape[:2])
                np.testing.assert_array_equal(array, before)

    def test_fast_partial_plate_holes_and_occlusion(self):
        from test_plate import half_plate
        for kind in ("partial", "holes", "occlusion"):
            with self.subTest(kind=kind):
                mask = half_plate()
                if kind == "holes":
                    cv2.circle(mask, (90, 50), 24, 0, -1)
                    cv2.circle(mask, (230, 55), 24, 0, -1)
                elif kind == "occlusion":
                    cv2.rectangle(mask, (137, 158), (190, 220), 0, -1)
                rgb = np.repeat(np.where(mask > 0, 210, 170).astype(np.uint8)[:, :, None], 3, axis=2)
                result, _ = plate.fit_plate_preview_array(rgb)
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result["tangent_y"], 190, delta=2)

    def test_fast_rejects_blank_rectangle_short_arc_and_ambiguity(self):
        from test_plate import half_plate
        rectangle, two_disks = np.zeros((240, 320), np.uint8), np.zeros((240, 320), np.uint8)
        cv2.rectangle(rectangle, (10, 0), (310, 170), 255, -1)
        cv2.circle(two_disks, (80, 30), 75, 255, -1)
        cv2.circle(two_disks, (240, 30), 75, 255, -1)
        masks = (np.zeros((240, 320), np.uint8), rectangle, two_disks,
                 half_plate(center=(-100, 20), radius=180), half_plate(center=(160, 80), radius=190))
        for index, mask in enumerate(masks):
            with self.subTest(index=index):
                rgb = np.repeat(np.where(mask > 0, 210, 170).astype(np.uint8)[:, :, None], 3, axis=2)
                result, _ = plate.fit_plate_preview_array(rgb)
                self.assertIsNone(result)

    def test_full_resolution_option_bypasses_resize(self):
        array = fixture("plate_unlit.png")
        reference, edges = plate.fit_plate_color_array(array, order="BGR")
        with patch.object(plate, "PROCESS_WIDTH", 320):
            result, preview = plate.fit_plate_preview_array(array, order="BGR")
        self.assertEqual(result, reference)
        np.testing.assert_array_equal(preview, edges)

    def test_rotated_ellipse_scaling_keeps_horizontal_tangent(self):
        import math
        original, _ = plate.fit_plate_color_array(fixture("plate_unlit.png"), order="BGR")
        result = plate.scale_plate_result(original, 1.3, 2.1)
        (cx, cy), (ew, eh), angle = result["ellipse"]
        sn, cs = math.sin(math.radians(angle)), math.cos(math.radians(angle))
        a, b = ew / 2, eh / 2
        height = math.hypot(a * sn, b * cs)
        self.assertAlmostEqual(result["tangent_y"], cy + height, places=5)
        self.assertAlmostEqual(result["tangent_x"], cx + (a*a - b*b) * sn * cs / height, places=5)

    def test_serial_partial_frame_loss_and_force_resynchronizes(self):
        serial=FakeUART([3]);sender=plate.PlateSender(serial)
        sender.send((30,-40),0)
        self.assertTrue(sender.pending)
        self.assertFalse(sender.send(None,.01))
        self.assertTrue(sender.send(None,.07))
        self.assertFalse(decode(serial.stream[-8:])['valid'])
        self.assertEqual(plate.encode_plate_frame((30,-40),12),main.encode_plate_frame((30,-40),12))
        serial=FakeUART([3]);sender=plate.PlateSender(serial)
        sender.send((30,-40),0);sender.send((31,-41),.01,force=True)
        self.assertEqual(len(serial.stream),3)
        sender.send((32,-42),.07,force=True)
        self.assertEqual((decode(serial.stream[-8:])['x'],decode(serial.stream[-8:])['y']),(32,42))

    def test_tangent_x_uses_touch_point_and_screen_scale(self):
        result = {"tangent_x": 200, "tangent_y": 170, "ellipse": ((120, 20), (200, 300), 25)}
        x, y = plate.tangent_screen_coordinates(result, 320, 240, 552, 368)
        self.assertAlmostEqual(x, 40 * 368 / 240)
        self.assertAlmostEqual(y, -50 * 368 / 240)
        self.assertEqual(plate.tangent_screen_coordinates({"tangent_x": 160, "tangent_y": 120}, 320, 240, 552, 368), (0, 0))

    def test_standalone_xy_packet_crc_and_numeric_invalidity(self):
        record=decode(plate.encode_plate_frame((-30,40),12))
        self.assertEqual((record['x'],record['y'],record['valid']),(-30,-40,True))
        self.assertEqual(decode(plate.encode_plate_frame((-.01,0),0))['x'],0)
        self.assertFalse(decode(plate.encode_plate_frame(None,0))['valid'])
        for coordinates in [(float('nan'),0),(0,float('inf'))]:
            self.assertFalse(decode(plate.encode_plate_frame(coordinates,0))['valid'])

    def device_environment(self):
        rgb = cv2.cvtColor(fixture("plate_lit.png"), cv2.COLOR_BGR2RGB)
        serial = FakeUART([])
        frames, closed = [], []
        class Device:
            def close(self):
                closed.append(self)
        class Camera(Device):
            width, height = lambda self: 320, lambda self: 240
            def read(self):
                frame = Frame(rgb.copy())
                frames.append(frame)
                return frame
        class Display(Device):
            width, height = lambda self: 552, lambda self: 368
            def show(self, *args, **kwargs):
                pass
        class Touch(Device):
            def available(self):
                return False
        count = [0]
        def need_exit():
            count[0] += 1
            return count[0] > 5
        maix = SimpleNamespace(app=SimpleNamespace(need_exit=need_exit),
                               camera=SimpleNamespace(Camera=lambda *args, **kwargs: Camera()),
                               display=SimpleNamespace(Display=Display), image=image_api(),
                               touchscreen=SimpleNamespace(TouchScreen=Touch),
                               err=SimpleNamespace(check_raise=lambda *args: None))
        uart = SimpleNamespace(UART=lambda *args, **kwargs: serial,
                               BITS=SimpleNamespace(BITS_8=8), PARITY=SimpleNamespace(PARITY_NONE=0),
                               STOP=SimpleNamespace(STOP_1=1), FLOW_CTRL=SimpleNamespace(FLOW_CTRL_NONE=0))
        gpio = SimpleNamespace(Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_DOWN=2,PULL_UP=1),
                              GPIO=lambda *args:SimpleNamespace(value=lambda level:None))
        peripheral = SimpleNamespace(uart=uart, gpio=gpio, pinmap=SimpleNamespace(set_pin_function=lambda *args: None))
        serial.close = lambda: closed.append(serial)
        return {"maix": maix, "maix.peripheral": peripheral}, serial, frames, closed

    def test_device_loop_confirms_draws_sends_and_invalidates_on_exit(self):
        modules, serial, frames, closed = self.device_environment()
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
            plate.run_device()
        self.assertEqual(len(frames), 5)
        self.assertFalse(frames[1].ellipses)
        self.assertTrue(frames[2].ellipses)
        self.assertTrue(any(record['valid'] for record in packets(serial.stream)))
        result, _ = plate.fit_plate_preview_array(frames[2].rgb)
        coordinates = plate.tangent_screen_coordinates(result, 320, 240, 552, 368)
        error = plate.tangent_alignment_error(result, 320, 240, 552, 368)
        record=next(record for record in packets(serial.stream) if record['valid'])
        self.assertAlmostEqual(record['x'],error[0]/(368/240),delta=.051)
        self.assertAlmostEqual(record['y'],-error[1]/(368/240),delta=.051)
        self.assertIn("X:%+.1f Y:%+.1f px" % coordinates, frames[2].texts)
        self.assertFalse(packets(serial.stream)[-1]['valid'])
        self.assertEqual(len(closed), 4)

    def test_detection_error_keeps_preview_and_invalidates_previous_measurement(self):
        modules, serial, frames, closed = self.device_environment()
        real_detect = plate.detect_plate_color
        calls = [0]
        def fail_after_confirmation(*args, **kwargs):
            calls[0] += 1
            if calls[0] == 4:
                raise TypeError("simulated SDK mismatch")
            return real_detect(*args, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "error.log")
            with patch.dict(sys.modules, modules), patch.object(plate, "ERROR_LOG_PATH", path), \
                    patch.object(plate, "detect_plate_color", side_effect=fail_after_confirmation), \
                    contextlib.redirect_stdout(io.StringIO()):
                plate.run_device()
            self.assertIn("simulated SDK mismatch", Path(path).read_text(encoding="utf-8"))
        self.assertEqual(len(frames), 5)
        self.assertEqual(calls[0], 4)  # 故障锁存，避免每帧重复异常和日志。
        self.assertTrue(any(record['valid'] for record in packets(serial.stream)))
        self.assertFalse(packets(serial.stream)[-1]['valid'])
        self.assertEqual(len(closed), 4)

    def test_uart_setup_failure_does_not_stop_camera_detection(self):
        modules, serial, frames, closed = self.device_environment()
        modules["maix.peripheral"].uart.UART = lambda *args, **kwargs: (_ for _ in ()).throw(OSError("port busy"))
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "error.log")
            with patch.dict(sys.modules, modules), patch.object(plate, "ERROR_LOG_PATH", path), \
                    contextlib.redirect_stdout(io.StringIO()):
                plate.run_device()
            self.assertIn("port busy", Path(path).read_text(encoding="utf-8"))
        self.assertEqual(len(frames), 5)
        self.assertTrue(frames[2].ellipses)
        self.assertEqual(len(closed), 3)


if __name__ == "__main__":
    unittest.main()
