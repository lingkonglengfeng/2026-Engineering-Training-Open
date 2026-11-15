"""SET occupied bit is separate from axis; mode changes discard prior acquisition."""
import time
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import service
import test_ring_axis_service as axis_tests
from test_service import Clock,Serial,Backend,frames
from test_middle_ring import make_pattern


class ProfileServiceTests(unittest.TestCase):
    def test_each_flag_combination_keeps_same_frame_wire_fields(self):
        for flags in (0,1,2,3):
            clock,serial,backend=Clock(),Serial(),Backend()
            base=backend.observe
            backend.observe=lambda now:dict(base(now),ring_axis=dict(degrees=1.25,confidence=88,source=3))
            server=service.VisionService(serial,backend,clock)
            serial.rx.extend(service.encode_set(3,3,2,20,7,flags));server.tick()
            packets=[p for p in frames(serial.output) if p[1] in (3,0x0a,0x83)]
            self.assertEqual([p[1] for p in packets],[3,0x0a,0x83] if flags&1 else [3,0x83])
            self.assertEqual(len({p[2] for p in packets}),1)
            self.assertEqual(packets[-1][3:7],bytes((3,3,7,0)))

    def test_non_ring_unknown_bits_and_unsupported_context_fail_ack(self):
        for flags in (2,3,4,255):
            for mode,target,hz in ((0,0,0),(2,1,20),(8,0,20)):
                expected=0 if mode==2 and flags==4 else 6
                self.assertEqual(service.validate_set(service.encode_set(mode,1,target,hz,0,flags)),expected)
        for station,target,flags in ((1,2,2),(2,0,2),(2,2,4),(3,2,128)):
            self.assertEqual(service.validate_set(service.encode_set(3,station,target,20,0,flags)),6)

    def test_backend_empty_bypasses_sampler_and_flag_switch_reacquires(self):
        picture=make_pattern(size=(480,360),centers=((80,180),(240,180),(400,180)),radius=45)
        backend,_=axis_tests.AxisBackendTests().backend(picture,1)
        try:
            with patch.object(backend.module,'recover_occupied_rings',side_effect=AssertionError('disabled')):
                for _ in range(3):observation=backend.observe(time.monotonic())
            self.assertIsNotNone(observation['coordinates'])
            old_detector,old_tracker,old_camera=backend.ring_detector,backend.ring_tracker,backend.cam
            self.assertFalse(old_detector.allow_occupied)
            backend.configure((3,3,2,20,3))
            self.assertTrue(backend.ring_detector.allow_occupied)
            self.assertIsNot(backend.ring_detector,old_detector)
            self.assertIsNot(backend.ring_tracker,old_tracker)
            self.assertIs(backend.cam,old_camera)
            self.assertIsNone(backend.ring_tracker.lock)
            self.assertIsNone(backend.observe(time.monotonic())['coordinates'])
            backend.configure((3,2,2,20,1))
            self.assertFalse(backend.ring_detector.allow_occupied)
            self.assertIsNone(backend.ring_detector.scene_layout)
            self.assertIsNone(backend.observe(time.monotonic())['ring_axis'])
        finally:backend.close()

    def test_real_occupied_backend_only_recovers_with_bit1(self):
        path=Path(__file__).resolve().parents[1]/'maixcam_ring/test_data/occupied_2_cleaned.png'
        gray=cv2.imdecode(np.fromfile(str(path),np.uint8),0)
        backend,_=axis_tests.AxisBackendTests().backend(gray,3)
        try:
            with patch.object(backend.module,'recover_occupied_rings',wraps=backend.module.recover_occupied_rings) as recover:
                for _ in range(3):observation=backend.observe(time.monotonic())
            self.assertGreater(recover.call_count,0)
            self.assertEqual(observation['ring_count'],3)
            self.assertIsNotNone(observation['coordinates'])
            backend.configure((3,2,2,20,1))
            with patch.object(backend.module,'recover_occupied_rings',side_effect=AssertionError('disabled')):
                for _ in range(3):observation=backend.observe(time.monotonic())
            self.assertIsNone(observation['coordinates'])
            self.assertIsNone(observation['ring_axis'])
        finally:backend.close()


if __name__=='__main__':unittest.main()
