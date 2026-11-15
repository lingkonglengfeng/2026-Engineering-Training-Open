"""真实 OpenCV 上验证轮廓识别、三组选择、跟踪及串口；不等同设备实测。"""
import math
import unittest
from pathlib import Path
from unittest import mock
from types import SimpleNamespace, ModuleType
import sys

import cv2
import numpy as np
import middle_ring as ring


def make_pattern(size=(320, 240), centers=((60, 120), (160, 120), (260, 120)), radius=29):
    width, height = size
    picture = np.full((height, width), 220, np.uint8)
    for i, center in enumerate(centers):
        for factor in (0.54, 0.60, 0.68, 0.78, 0.89, 1.0):
            cv2.circle(picture, center, round(radius * factor), 25, 1, cv2.LINE_AA)
        cv2.putText(picture, str(i + 1), (center[0] - 5, center[1] + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, 30, 1)
    return picture


def group(x, y, radius=20):
    return {"center": (x, y), "radius": radius, "levels": 6,
            "score": 0.9, "ellipse": ((x, y), (radius * 2, radius * 2), 0)}


class DetectionTests(unittest.TestCase):
    @staticmethod
    def fixture(name):
        path = Path(__file__).parent / "test_data" / name
        return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)

    def test_user_live_view_complete_finds_middle_two(self):
        picture = self.fixture("live_complete.png")
        self.assertIsNotNone(picture)
        groups, middle, _, status = ring.detect_rings(picture)
        self.assertEqual(len(groups), 3)
        self.assertEqual(status, "THREE_FOUND")
        # 预览区域去掉截图顶部 18 px，使用预览自身的坐标。
        self.assertLess(math.dist(middle["ring"]["center"], (237, 140)), 3)

    def test_user_live_view_cropped_completes_side_rings(self):
        picture = self.fixture("live_cropped.png")
        self.assertIsNotNone(picture)
        groups, middle, _, status = ring.detect_rings(picture)
        self.assertEqual(len(groups), 3)
        self.assertIsNotNone(middle)
        self.assertEqual(sum(g["partial"] for g in groups), 2)
        self.assertLess(math.dist(middle["ring"]["center"], (229, 169)), 3)
        tracker = ring.MiddleTracker()
        for i in range(ring.ACQUIRE_FRAMES):
            observed = tracker.update(groups, middle, i * .03, status)
        self.assertIsNotNone(observed)
        self.assertEqual(tracker.status, "MIDDLE_PARTIAL")

    def test_user_live_views_track_middle_after_complete_acquisition(self):
        # 统一到设备处理尺寸，模拟先看全三组、靠近后两侧被裁切。
        complete = cv2.resize(self.fixture("live_complete.png"), (480, 360))
        cropped = cv2.resize(self.fixture("live_cropped.png"), (480, 360))
        groups, triple, _, status = ring.detect_rings(complete)
        tracker = ring.MiddleTracker()
        for i in range(ring.ACQUIRE_FRAMES):
            observed = tracker.update(groups, triple, i * 0.03, status)
        self.assertIsNotNone(observed)
        groups, triple, _, status = ring.detect_rings(cropped)
        self.assertIsNotNone(triple)
        observed = tracker.update(groups, triple, 0.10, status)
        self.assertIsNotNone(observed)
        self.assertEqual(observed["mode"], "THREE_PARTIAL")
        self.assertLess(math.dist(observed["ring"]["center"], (245, 179)), 3)

    def test_horizontal_six_layer_markers(self):
        groups, middle, _, status = ring.detect_rings(make_pattern())
        self.assertEqual(len(groups), 3)
        self.assertEqual(status, "THREE_FOUND")
        self.assertLess(math.dist(middle["ring"]["center"], (160, 120)), 1.5)

    def test_vertical_markers(self):
        picture = make_pattern(centers=((160, 40), (160, 120), (160, 200)), radius=22)
        groups, middle, _, _ = ring.detect_rings(picture)
        self.assertEqual(len(groups), 3)
        self.assertIsNotNone(middle)
        self.assertLess(math.dist(middle["ring"]["center"], (160, 120)), 1.5)

    def test_rotated_row(self):
        centers = tuple((round(160 + k * 80 * math.cos(math.radians(35))),
                         round(120 + k * 80 * math.sin(math.radians(35)))) for k in (-1, 0, 1))
        picture = make_pattern(centers=centers, radius=24)
        groups, middle, _, _ = ring.detect_rings(picture)
        self.assertEqual(len(groups), 3)
        self.assertIsNotNone(middle)
        self.assertAlmostEqual(middle["angle"], -35, delta=1)

    def test_brightness_gradient_and_noise(self):
        picture = make_pattern()
        gradient = np.linspace(0.45, 1.05, 320)[None, :]
        noisy = np.clip(picture.astype(float) * gradient
                        + np.random.default_rng(5).normal(0, 2, picture.shape), 0, 255).astype(np.uint8)
        _, middle, _, _ = ring.detect_rings(noisy)
        self.assertIsNotNone(middle)
        self.assertLess(math.dist(middle["ring"]["center"], (160, 120)), 2)

    def test_blank_or_three_solid_circles_not_ring_markers(self):
        blank = np.full((240, 320), 220, np.uint8)
        self.assertEqual(ring.detect_rings(blank)[0], [])
        for x in (60, 160, 260):
            cv2.circle(blank, (x, 120), 29, 0, 2)
        self.assertIsNone(ring.detect_rings(blank)[1])

    def test_middle_selection_is_not_nearest_screen_center(self):
        groups = [group(260, 80), group(140, 80), group(200, 80)]
        middle, _ = ring.choose_middle(groups)
        self.assertEqual(middle["ring"]["center"], (200, 80))

    def test_ambiguous_four_markers_rejected(self):
        middle, status = ring.choose_middle([group(x, 100) for x in (40, 100, 160, 220)])
        self.assertIsNone(middle)
        self.assertEqual(status, "AMBIGUOUS")

    def test_non_collinear_markers_rejected(self):
        self.assertIsNone(ring.choose_middle([group(60, 100), group(160, 170), group(260, 100)])[0])


class PartialRingTests(unittest.TestCase):
    def assert_middle(self, picture, expected):
        groups, middle, _, _ = ring.detect_rings(picture)
        self.assertEqual(len(groups), 3)
        self.assertIsNotNone(middle)
        self.assertTrue(any(g["partial"] for g in groups))
        self.assertLess(math.dist(middle["ring"]["center"], expected), 2)
        return groups, middle

    def test_left_outer_half_visible(self):
        self.assert_middle(make_pattern(centers=((0,120),(90,120),(180,120)),radius=28), (90,120))

    def test_right_outer_half_visible(self):
        self.assert_middle(make_pattern(centers=((140,120),(230,120),(320,120)),radius=28), (230,120))

    def test_both_outer_halves_visible(self):
        groups,_ = self.assert_middle(make_pattern(centers=((0,120),(160,120),(320,120)),radius=50), (160,120))
        self.assertEqual(sum(g["partial"] for g in groups),2)

    def test_center_of_partial_outer_ring_can_be_just_outside_frame(self):
        self.assert_middle(make_pattern(centers=((-5,120),(85,120),(175,120)),radius=28), (85,120))

    def test_vertical_top_and_bottom_halves(self):
        self.assert_middle(make_pattern(centers=((160,0),(160,120),(160,240)),radius=38), (160,120))

    def test_all_three_top_halves_without_intact_reference(self):
        self.assert_middle(make_pattern(centers=((60,0),(160,0),(260,0)),radius=29), (160,0))

    def test_partial_ellipse_uses_intact_ring_shape(self):
        # 保留足够的像素分开 6 层细环，再验证明显椭圆投影的补全。
        picture=make_pattern(centers=((0,120),(140,120),(280,120)),radius=42)
        picture=cv2.warpAffine(picture,np.array([[1.,0,0],[0,.7,36]],np.float32),
                               (320,240),borderValue=220)
        self.assert_middle(picture,(140,120))

    def test_quarter_arc_insufficient(self):
        picture=make_pattern(centers=((0,0),(80,80),(160,160)),radius=28)
        self.assertIsNone(ring.detect_rings(picture)[1])

    def test_only_two_groups_do_not_invent_third(self):
        picture=make_pattern(centers=((0,120),(90,120)),radius=28)
        groups,middle,_,status=ring.detect_rings(picture)
        self.assertIsNone(middle)
        self.assertIsNone(ring.MiddleTracker().update(groups,middle,0,status))

    def test_clipped_single_circle_is_not_multilayer_ring(self):
        picture=np.full((240,320),220,np.uint8)
        cv2.circle(picture,(0,120),29,0,2)
        self.assertEqual(ring.detect_rings(picture)[0],[])

    def test_clipped_nested_rectangles_do_not_fake_ring(self):
        picture=make_pattern(centers=((90,120),(180,120)),radius=28)
        for radius in (16,20,24,28):
            cv2.rectangle(picture,(-radius,120-radius),(radius,120+radius),0,1)
        self.assertIsNone(ring.detect_rings(picture)[1])


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.groups = [group(80, 120), group(140, 120), group(200, 120)]
        self.triple, _ = ring.choose_middle(self.groups)
        self.tracker = ring.MiddleTracker()

    def acquire(self):
        result = None
        for i in range(ring.ACQUIRE_FRAMES):
            result = self.tracker.update(self.groups, self.triple, i * 0.03)
        self.assertIsNotNone(result)
        return result

    def test_wait_for_confirmed_three_frames(self):
        for i in range(ring.ACQUIRE_FRAMES - 1):
            self.assertIsNone(self.tracker.update(self.groups, self.triple, i * 0.03))
        self.assertIsNotNone(self.tracker.update(self.groups, self.triple, 0.09))

    def test_single_marker_cannot_initially_acquire(self):
        self.assertIsNone(self.tracker.update([self.groups[1]], None, 0))
        self.assertEqual(self.tracker.status, "SHOW_ALL_3")

    def test_tracks_locked_middle_when_neighbors_disappear(self):
        self.acquire()
        observed = self.tracker.update([group(144, 122)], None, 0.1)
        self.assertEqual(observed["ring"]["center"], (144, 122))
        self.assertFalse(observed["angle_valid"])
        self.assertEqual(observed["mode"], "TRACK")

    def test_missing_middle_does_not_switch_to_end_marker(self):
        self.acquire()
        self.assertIsNone(self.tracker.update([self.groups[2]], None, 0.1))
        self.assertEqual(self.tracker.status, "LOST_STOP")

    def test_old_lock_expires_and_requires_three_again(self):
        self.acquire()
        self.assertIsNone(self.tracker.update([self.groups[1]], None, 0.5))
        self.assertIsNone(self.tracker.lock)

    def test_ambiguous_frame_not_used(self):
        self.acquire()
        self.assertIsNone(self.tracker.update(self.groups, None, 0.1, "AMBIGUOUS"))

    def test_follows_same_middle_into_half_view(self):
        tracker=ring.MiddleTracker()
        for frame_index, shift in enumerate([0,0,0]+list(range(6,175,6))):
            picture=make_pattern(centers=((80-shift,120),(170-shift,120),(260-shift,120)),radius=28)
            groups,triple,_,status=ring.detect_rings(picture)
            observation=tracker.update(groups,triple,frame_index*.03,status)
            if frame_index>=ring.ACQUIRE_FRAMES-1:
                self.assertIsNotNone(observation)
                self.assertLess(math.dist(observation['ring']['center'],(170-shift,120)),2)
        self.assertEqual(observation['mode'],'TRACK_PARTIAL')
        self.assertFalse(observation['angle_valid'])
        self.assertLess(observation['ring']['center'][0],0)

    def test_partial_initial_acquisition_outputs_valid_middle_coordinates(self):
        tracker=ring.MiddleTracker()
        picture=make_pattern(centers=((0,120),(90,120),(180,120)),radius=28)
        groups,triple,_,status=ring.detect_rings(picture)
        for i in range(ring.ACQUIRE_FRAMES):
            observation=tracker.update(groups,triple,i*.03,status)
        self.assertIsNotNone(observation)
        error=ring.screen_error(observation['ring']['center'],(320,240),(320,240))
        self.assertAlmostEqual(error[0],-70,delta=2)
        self.assertAlmostEqual(error[1],0,delta=2)
        packet=ring.encode_rloc(error,observation['angle'],observation['angle_valid'],
                               False,observation['score'],1)
        self.assertEqual(packet[2]&7,2)
        self.assertTrue(packet[2]&0x80)

    def test_two_fps_does_not_reacquire_every_frame(self):
        tracker=ring.MiddleTracker()
        for i in range(6):
            observed=tracker.update(self.groups,self.triple,i*.5)
            if i>=ring.ACQUIRE_FRAMES-1:
                self.assertIsNotNone(observed)
                self.assertEqual(tracker.status,'MIDDLE_LOCKED')
        self.assertIsNotNone(tracker.update([self.groups[1]],None,3.0))
        self.assertIsNone(tracker.update([],None,3.5))
        self.assertIsNone(tracker.update([self.groups[1]],None,5.0))
        self.assertIsNone(tracker.lock)

    def test_consistent_fresh_layout_survives_periodic_scan_delay(self):
        self.acquire()
        observed=self.tracker.update(self.groups,self.triple,.45)
        self.assertIsNotNone(observed)
        self.assertEqual(self.tracker.status,'MIDDLE_LOCKED')


class PerformancePathTests(unittest.TestCase):
    def test_closed_ring_native_path_avoids_generic_ellipse_solver(self):
        picture=DetectionTests.fixture('live_complete.png')
        with mock.patch.object(ring.cv2,'fitEllipse',side_effect=AssertionError('No generic solve')):
            groups,middle,_,_=ring.detect_rings(picture)
        self.assertEqual(len(groups),3)
        self.assertIsNotNone(middle)

    def test_moment_geometry_for_rotated_translated_ellipse(self):
        picture=np.zeros((240,320),np.uint8)
        cv2.ellipse(picture,(174,116),(49,27),37,0,360,255,-1,cv2.LINE_8)
        contour=max(cv2.findContours(picture,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[-2],key=cv2.contourArea)
        candidate=ring.ellipse_candidate(contour,picture.shape)
        self.assertIsNotNone(candidate)
        self.assertLess(math.dist(candidate['center'],(174,116)),.3)
        self.assertAlmostEqual(candidate['ellipse'][1][0],98,delta=1)
        self.assertAlmostEqual(candidate['ellipse'][1][1],54,delta=1)
        self.assertAlmostEqual(candidate['ellipse'][2],37,delta=1)

    def test_native_and_generic_paths_agree_on_real_centers(self):
        for name in ['live_complete.png','live_cropped.png']:
            picture=DetectionTests.fixture(name)
            fast_groups,fast_middle,_,_=ring.detect_rings(picture)
            with mock.patch.object(ring,'MOMENT_ELLIPSES_ENABLED',False):
                old_groups,old_middle,_,_=ring.detect_rings(picture)
            self.assertEqual(len(fast_groups),len(old_groups))
            self.assertLess(math.dist(fast_middle['ring']['center'],old_middle['ring']['center']),.05)
            for g in fast_groups:
                self.assertLess(min(math.dist(g['center'],other['center']) for other in old_groups),.25)

    def test_interior_rings_do_not_enter_partial_fitting(self):
        with mock.patch.object(ring,'clipped_arc_candidates',side_effect=AssertionError('Interior arc')):
            groups,middle,_,_=ring.detect_rings(make_pattern())
        self.assertEqual(len(groups),3)
        self.assertIsNotNone(middle)

    def test_valid_complete_layout_skips_background_border_arcs(self):
        picture=make_pattern()
        cv2.line(picture,(0,200),(319,215),20,2)
        with mock.patch.object(ring,'clipped_arc_candidates',side_effect=AssertionError('Already have three')):
            groups,middle,_,_=ring.detect_rings(picture)
        self.assertEqual(len(groups),3)
        self.assertIsNotNone(middle)

    def test_many_small_noise_contours_are_filtered_before_fitting(self):
        picture=make_pattern()
        picture[175:238:4,1:319:4]=0
        diagnostics={}
        _,middle,_,_=ring.detect_rings(picture,diagnostics)
        self.assertIsNotNone(middle)
        self.assertGreater(diagnostics['raw'],1000)
        self.assertLess(diagnostics['kept'],100)

    def test_arc_endpoint_fast_path_matches_original_coverage(self):
        for start,end in [(-85,85),(170,370),(80,-110),(-170,170)]:
            angles=np.deg2rad(np.linspace(start,end,80))
            dx,dy=np.cos(angles),np.sin(angles)
            self.assertAlmostEqual(ring.arc_coverage_degrees(dx,dy),abs(end-start),places=5)

    def test_refolded_arc_uses_safe_fallback(self):
        angles=np.deg2rad(np.r_[np.linspace(-85,85,40),np.linspace(85,-85,40)])
        self.assertAlmostEqual(ring.arc_coverage_degrees(np.cos(angles),np.sin(angles)),170,places=5)

    def test_detector_reports_substage_times_and_counts(self):
        diagnostics={}
        picture=DetectionTests.fixture('live_cropped.png')
        groups,middle,_,_=ring.detect_rings(picture,diagnostics)
        self.assertIsNotNone(middle)
        self.assertEqual(len(groups),3)
        self.assertLessEqual(diagnostics['kept'],diagnostics['raw'])
        self.assertGreater(diagnostics['arcs'],0)
        for name in ['B','K','E','P','G']:
            self.assertGreaterEqual(diagnostics[name],0)

    def test_partial_detector_never_calls_matrix_inverse_or_svd(self):
        picture=DetectionTests.fixture('live_cropped.png')
        with mock.patch.object(ring.np.linalg,'inv',side_effect=AssertionError('No inverse')), \
             mock.patch.object(ring.np.linalg,'lstsq',side_effect=AssertionError('No SVD')):
            groups,middle,_,_=ring.detect_rings(picture)
        self.assertEqual(len(groups),3)
        self.assertIsNotNone(middle)
        # 原实拍中心作为几何回归基准；面积矩与通用拟合容许极小的亚像素差异。
        self.assertLess(math.dist(middle['ring']['center'],(228.86643088,168.61478447)),.05)

    def test_profiler_does_not_hide_slow_preview_with_skipped_frames(self):
        profiler=ring.LoopProfiler()
        profiler.record(.050,.025,.014,.001,.010,True)
        profiler.record(.040,.025,.014,.001,0,False)
        stats=profiler.snapshot()
        self.assertAlmostEqual(stats['fps'],2/.09)
        self.assertEqual(stats['C'],25)
        self.assertEqual(stats['D'],14)
        self.assertEqual(stats['U'],1)
        self.assertEqual(stats['V'],10)
        self.assertEqual(stats['previews'],1)
        profiler.reset_window()
        self.assertEqual(profiler.snapshot(),stats)

    def test_rgb_render_preserves_colors_without_extra_swap(self):
        gray=np.zeros((120,160),np.uint8)
        groups=[group(80,75,20)]
        observed={'ring':groups[0]}
        bgr=ring.annotate(gray,groups,observed,status='OK')
        rgb=ring.annotate(gray,groups,observed,status='OK',rgb=True)
        np.testing.assert_array_equal(rgb,bgr[:,:,::-1])

    def run_preview_fixture(self, enabled=False):
        picture=DetectionTests.fixture('live_complete.png')
        class Frame:
            def width(self): return picture.shape[1]
            def height(self): return picture.shape[0]
        class Device:
            def __init__(self): self.reads=0;self.shows=0;self.closed=False
            def width(self): return picture.shape[1]
            def height(self): return picture.shape[0]
            def read(self): self.reads+=1;return Frame()
            def close(self): self.closed=True
            def show(self,*args,**kwargs): self.shows+=1
            def available(self): return False
        cam,disp,touch=Device(),Device(),Device()
        flags=iter([False]*(12 if enabled else 6)+[True])
        fake=ModuleType('maix')
        fake.app=SimpleNamespace(need_exit=lambda:next(flags))
        fake.camera=SimpleNamespace(Camera=lambda *args,**kwargs:cam)
        fake.display=SimpleNamespace(Display=lambda:disp,set_trans_image_quality=lambda quality:None)
        fake.touchscreen=SimpleNamespace(TouchScreen=lambda:touch)
        fake.image=SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE=0),
                                   Fit=SimpleNamespace(FIT_CONTAIN=0),
                                   image2cv=lambda *args,**kwargs:picture,
                                   cv2image=lambda rendered,**kwargs:rendered)
        fake.err=SimpleNamespace(check_raise=lambda *args:None)
        peripheral=ModuleType('maix.peripheral')
        light_values=[]
        peripheral.uart=SimpleNamespace()
        peripheral.pinmap=SimpleNamespace(set_pin_function=lambda *args:0)
        peripheral.gpio=SimpleNamespace(Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_UP=1,PULL_DOWN=2),
            GPIO=lambda *args:SimpleNamespace(value=lambda level:light_values.append(level)))
        timer=SimpleNamespace(now=0.)
        def slow_process(*args,**kwargs):
            timer.now+=.25
            return [],None,None,'NEED_THREE'
        with mock.patch.dict(sys.modules,{'maix':fake,'maix.peripheral':peripheral}), \
             mock.patch.object(ring,'UART_ENABLED',False), \
             mock.patch.object(ring,'PREVIEW_ENABLED',enabled), \
             mock.patch.object(ring.time,'monotonic',lambda:timer.now), \
             mock.patch.object(ring.RingDetector,'process',side_effect=slow_process), \
             mock.patch.object(ring,'PERF_LOG_INTERVAL_S',0):
            ring.run_maixcam()
        self.assertTrue(cam.closed and disp.closed and touch.closed)
        self.assertEqual(light_values,[1,0])
        return cam,disp

    def test_main_can_skip_preview_while_continuing_detection(self):
        cam,disp=self.run_preview_fixture(False)
        self.assertEqual(cam.reads,6)
        self.assertEqual(disp.shows,0)

    def test_main_slow_detection_still_updates_display(self):
        cam,disp=self.run_preview_fixture(True)
        self.assertEqual(cam.reads,12)
        self.assertGreaterEqual(disp.shows,2)
        self.assertLessEqual(disp.shows,4)


class RoiTrackingTests(unittest.TestCase):
    @staticmethod
    def tick(detector,tracker,picture,now):
        diagnostics={}
        groups,triple,_,status=detector.process(picture,tracker,now,diagnostics)
        if detector.identity_rejected and triple is None:
            tracker.reset()
        observed=tracker.update(groups,triple,now,status)
        detector.accept(observed,triple,picture,now)
        return observed,diagnostics

    def acquire(self,picture):
        detector,tracker=ring.RingDetector(),ring.MiddleTracker()
        for i in range(ring.ACQUIRE_FRAMES):
            observed,_=self.tick(detector,tracker,picture,i*.03)
        self.assertIsNotNone(observed)
        return detector,tracker,observed

    def test_roi_measures_fresh_center_in_global_coordinates(self):
        original=make_pattern()
        detector,tracker,_=self.acquire(original)
        moved=make_pattern(centers=((64,122),(164,122),(264,122)))
        observed,stats=self.tick(detector,tracker,moved,.09)
        self.assertIsNotNone(observed)
        self.assertTrue(detector.used_roi)
        self.assertEqual(stats['mode'],'ROI')
        self.assertLess(math.dist(observed['ring']['center'],(164,122)),1)
        self.assertFalse(observed['angle_valid'])

    def test_periodic_full_layout_verification(self):
        picture=make_pattern()
        detector,tracker,_=self.acquire(picture)
        observed,stats=self.tick(detector,tracker,picture,1.10)
        self.assertFalse(detector.used_roi)
        self.assertEqual(stats['mode'],'FULL')

    def test_wrong_neighbor_in_previous_position_is_not_used(self):
        detector,tracker,_=self.acquire(make_pattern())
        # 图案 1 替代原图案 2 的位置，第三组完全离开视野；不能凭形状缓存误认。
        moved=make_pattern(centers=((160,120),(260,120),(360,120)))
        observed,stats=self.tick(detector,tracker,moved,.09)
        self.assertTrue(detector.identity_rejected)
        self.assertIsNone(observed)
        self.assertIsNone(tracker.lock)

    def test_roi_handles_brightness_change(self):
        picture=make_pattern()
        detector,tracker,_=self.acquire(picture)
        darker=np.clip(picture.astype(float)*.80+20,0,255).astype(np.uint8)
        observed,stats=self.tick(detector,tracker,darker,.09)
        self.assertIsNotNone(observed)
        self.assertTrue(detector.used_roi)

    def test_hidden_center_pattern_falls_back_to_layout(self):
        picture=make_pattern()
        detector,tracker,_=self.acquire(picture)
        covered=picture.copy()
        cv2.rectangle(covered,(150,110),(170,130),220,-1)
        observed,stats=self.tick(detector,tracker,covered,.09)
        self.assertFalse(detector.used_roi)
        self.assertEqual(stats['mode'],'FULL_R')
        self.assertIsNotNone(observed)
        self.assertLess(math.dist(observed['ring']['center'],(160,120)),2)

    def test_roi_boundary_not_treated_as_camera_boundary(self):
        picture=make_pattern(centers=((100,120),(200,120),(300,120)),radius=29)
        groups,_,_,_=ring.detect_rings(picture,roi=(0,0,115,240))
        self.assertEqual(groups,[])

    def test_half_middle_falls_back_without_losing_identity(self):
        initial=make_pattern(centers=((80,120),(170,120),(260,120)),radius=28)
        detector,tracker,_=self.acquire(initial)
        for i,shift in enumerate(range(6,175,6)):
            picture=make_pattern(centers=((80-shift,120),(170-shift,120),(260-shift,120)),radius=28)
            observed,_=self.tick(detector,tracker,picture,.09+i*.03)
            self.assertIsNotNone(observed)
            self.assertLess(math.dist(observed['ring']['center'],(170-shift,120)),2)
        self.assertFalse(detector.used_roi)
        self.assertTrue(observed['ring']['partial'])


class AlignmentTests(unittest.TestCase):
    def test_screen_center_and_signs(self):
        self.assertEqual(ring.screen_error((160, 120), (320, 240), (552, 368)), (0, 0))
        x, y = ring.screen_error((170, 110), (320, 240), (552, 368))
        self.assertGreater(x, 0)
        self.assertGreater(y, 0)

    def test_alignment_needs_fresh_consecutive_valid_frames(self):
        state = ring.AlignmentState()
        for _ in range(ring.ALIGN_STABLE_FRAMES - 1):
            self.assertFalse(state.update((0, 0)))
        self.assertTrue(state.update((0, 0)))
        self.assertFalse(state.update(None))
        self.assertFalse(state.update((20, 0)))

    def test_unconfigured_or_lost_target_zero_velocity(self):
        self.assertEqual(ring.correction_velocity((20, 10), None), ((0, 0), False))
        self.assertEqual(ring.correction_velocity(None, [[1, 0], [0, 1]]), ((0, 0), False))

    def test_calibrated_correction_direction_and_speed_limit(self):
        velocity, valid = ring.correction_velocity((100, 0), [[0, 0.5], [-0.5, 0]], kp=1, max_speed_mm_s=20)
        self.assertTrue(valid)
        self.assertAlmostEqual(velocity[0], 0)
        self.assertAlmostEqual(velocity[1], -20)
        with self.assertRaises(ValueError):
            ring.correction_velocity((1, 2), [[1, 1], [1, 1]])


class SerialTests(unittest.TestCase):
    def test_binary_pixel_frame_crc_and_invalid_zeroing(self):
        data=ring.encode_rloc((30,-20),5,True,False,.9,12)
        self.assertEqual(data,bytes.fromhex('6B 03 E2 2C 01 C8 00 E4'))
        self.assertEqual(ring.crc8(data[:7]),data[7])
        invalid=ring.encode_rloc(None,50,True,True,1,13)
        self.assertEqual(invalid[2]&0x80,0)
        self.assertEqual(invalid[3:7],bytes(4))
        self.assertEqual(ring.encode_rloc((30,-20),-70,False,True,.2,12),data)

    def test_send_loss_bypasses_rate_limit(self):
        class Serial:
            def __init__(self): self.data = []
            def write(self, data):
                self.data.append(data)
                return len(data)
        serial = Serial()
        sender = ring.RlocSender(serial)
        observed = {"angle": 0, "angle_valid": True, "score": 0.9}
        sender.send((10, 20), observed, False, 0)
        sender.send((11, 21), observed, False, 0.01)
        self.assertEqual(len(serial.data), 1)
        sender.send(None, None, False, 0.02)
        self.assertEqual(len(serial.data), 2)
        self.assertFalse(serial.data[-1][2]&0x80)


if __name__ == "__main__":
    unittest.main()
