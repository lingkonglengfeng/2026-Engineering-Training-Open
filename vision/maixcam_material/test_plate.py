"""白色圆盘半圆/底部切线测试：真实 OpenCV，合成图不代替实拍。"""
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import main
from test_uart import FakeUART, decode


def fit_from_mask(mask, diagnostics=None):
    # 背景和盘都属于白/灰色：L 阈值无法分开，使用实际边缘算法。
    rgb=np.repeat(np.where(mask>0,210,170).astype(np.uint8)[:,:,None],3,axis=2)
    result,_=main.fit_plate_color_array(rgb,diagnostics)
    return result


def half_plate(center=(165, -15), radius=205):
    mask = np.zeros((240, 320), np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask


class PlateGeometryTests(unittest.TestCase):
    def test_half_circle_with_center_outside_frame(self):
        details = {}
        result = fit_from_mask(half_plate(), details)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'], 190, delta=1.5)
        self.assertEqual(details['reason'], 'EDGE_OK')
        self.assertGreater(result['arc_deg'], 35)

    def test_only_visible_lower_half(self):
        result = fit_from_mask(half_plate(center=(160, 0), radius=170))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'], 170, delta=1.5)

    def test_material_holes_do_not_shift_outer_edge(self):
        mask = half_plate()
        cv2.circle(mask, (90, 50), 24, 0, -1)
        cv2.circle(mask, (230, 55), 24, 0, -1)
        result = fit_from_mask(mask)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'], 190, delta=1.5)

    def test_small_center_occlusion_still_fits_flanks(self):
        mask = half_plate()
        cv2.rectangle(mask, (137, 158), (190, 220), 0, -1)
        result = fit_from_mask(mask)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'], 190, delta=2)

    def test_rectangle_does_not_count_as_round_plate(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.rectangle(mask, (10, 0), (310, 170), 255, -1)
        self.assertIsNone(fit_from_mask(mask))

    def test_all_white_unseparated_background_invalid(self):
        self.assertIsNone(fit_from_mask(np.full((240, 320), 255, np.uint8)))

    def test_missing_or_short_arc_invalid(self):
        self.assertIsNone(fit_from_mask(np.zeros((240, 320), np.uint8)))
        self.assertIsNone(fit_from_mask(half_plate(center=(-100, 20), radius=180)))
        self.assertIsNone(fit_from_mask(half_plate(center=(160, -730), radius=900)))

    def test_tangent_outside_frame_invalid(self):
        self.assertIsNone(fit_from_mask(half_plate(center=(160, 80), radius=190)))

    def test_two_similar_white_disks_ambiguous(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.circle(mask, (80, 30), 75, 255, -1)
        cv2.circle(mask, (240, 30), 75, 255, -1)
        details = {}
        self.assertIsNone(fit_from_mask(mask, details))
        self.assertEqual(details['reason'], 'AMBIGUOUS_EDGE')

    def test_confirmation_loss_and_jump(self):
        tracker = main.PlateTracker()
        result = {'tangent_y': 170}
        for _ in range(main.PLATE_CONFIRM_FRAMES - 1):
            self.assertIsNone(tracker.update(result))
        self.assertIs(tracker.update(result), result)
        self.assertIsNone(tracker.update(None))
        self.assertIsNone(tracker.update(result))
        self.assertIsNone(tracker.update({'tangent_y': 220}))
        self.assertEqual(tracker.count, 1)

    def test_line_centered_screen_y(self):
        geometry = main.fit_geometry(320, 240, 552, 368)
        sy = main.screen_point(160, 170, 320, 240, geometry)[1]
        self.assertAlmostEqual(368 / 2 - sy, -50 * 368 / 240)


class ColorPlatePhotoTests(unittest.TestCase):
    @staticmethod
    def fixture(name):
        path=Path(__file__).parent/'test_data'/name
        return cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)

    def test_lit_color_photo_finds_top_rim_tangent(self):
        diagnostics={}
        result,edges=main.fit_plate_color_array(self.fixture('plate_lit.png'),diagnostics,order='BGR')
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'],138,delta=4)
        self.assertEqual(diagnostics['reason'],'EDGE_OK')
        self.assertGreater(result['arc_deg'],100)

    def test_unlit_color_photo_merges_two_broken_flanks(self):
        result,edges=main.fit_plate_color_array(self.fixture('plate_unlit.png'),order='BGR')
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['tangent_y'],136,delta=4)
        self.assertGreater(result['arc_deg'],70)

    def test_runtime_plate_path_keeps_color_and_never_binarizes(self):
        rgb=cv2.cvtColor(self.fixture('plate_lit.png'),cv2.COLOR_BGR2RGB)
        class Frame:
            def binary(self,*args,**kwargs): raise AssertionError('No white thresholding')
            def to_format(self,*args): raise AssertionError('Keep color frame')
        frame=Frame()
        api=SimpleNamespace(image2cv=lambda source,ensure_bgr,copy:rgb)
        result,source=main.detect_plate_color(frame,api,{})
        self.assertIsNotNone(result)
        self.assertIs(source,frame)

    def test_edge_preview_owns_copy_of_array(self):
        rgb=cv2.cvtColor(self.fixture('plate_unlit.png'),cv2.COLOR_BGR2RGB)
        copied=[]
        def cv2image(edges,bgr,copy):
            copied.append(copy)
            return 'owned-edge-preview'
        api=SimpleNamespace(image2cv=lambda source,ensure_bgr,copy:rgb,cv2image=cv2image)
        result,source=main.detect_plate_color(object(),api,{},edges_view=True)
        self.assertIsNotNone(result)
        self.assertEqual(source,'owned-edge-preview')
        self.assertEqual(copied,[True])


class PlateSerialTests(unittest.TestCase):
    def test_xy_crc_valid_origin_and_loss(self):
        packet=main.encode_plate_frame((30,-40),12)
        self.assertEqual(packet,bytes.fromhex('6B 09 E0 2C 01 90 01 9E'))
        record=decode(packet)
        self.assertEqual((record['type'],record['id'],record['x'],record['y']),(9,0,30,40))
        self.assertTrue(decode(main.encode_plate_frame((0,0),0))['valid'])
        self.assertFalse(decode(main.encode_plate_frame(None,0))['valid'])
        for coordinates in [(math.nan,0),(0,math.inf)]:
            self.assertFalse(decode(main.encode_plate_frame(coordinates,0))['valid'])

    def test_switch_from_partial_material_frame_waits_for_resync(self):
        serial=FakeUART([3]);sender=main.CoordinateSender(serial)
        sender.send(3,(10,20),0)
        self.assertFalse(sender.send_plate((30,-40),.01))
        self.assertTrue(sender.send_plate((30,-40),.07))
        self.assertEqual(decode(serial.stream[-8:])['type'],9)
        self.assertEqual(sender.last_state,(0,True))
        sender.send_plate(None,.08)
        self.assertFalse(decode(serial.stream[-8:])['valid'])

    def test_plate_sample_persistence_accepts_mode_zero(self):
        self.assertEqual(main.valid_samples({'0': [[75, 1, 2]]}), {'0': [[75, 1, 2]]})
        self.assertEqual(len(main.COLORS), 6)
        self.assertEqual(main.MODES, main.COLORS)
        self.assertNotIn('PLATE', [mode[1] for mode in main.MODES])


if __name__ == '__main__':
    unittest.main()
