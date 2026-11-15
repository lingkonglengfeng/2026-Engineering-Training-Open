"""TEMP station3: real CV -> backend -> fresh binary groups, including two rings."""
import unittest
from unittest.mock import patch

import numpy as np
import service
import test_ring_axis_service as axis_tests
from test_middle_ring import make_pattern
from test_service import Clock, Serial, frames


class Temp2RingCountTests(unittest.TestCase):
    def start(self, flags, two=True):
        self.complete = make_pattern(size=(480, 360),
            centers=((80, 180), (240, 180), (400, 180)), radius=45)
        self.partial = self.complete.copy()
        self.partial[:, 330:] = 220
        self.clock, self.serial = Clock(), Serial()
        # Also exercise the embedded backend/detector when testing generated main.py.
        with patch.object(axis_tests, 'service', service):
            self.backend, self.scene = axis_tests.AxisBackendTests().backend(
                self.partial if two else self.complete, flags)
        self.addCleanup(self.backend.close)
        self.server = service.VisionService(self.serial, self.backend, self.clock)
        self.serial.rx.extend(service.encode_set(3, 3, 2, 20, 7, flags))
        return self.tick(advance=False)

    def tick(self, advance=True):
        if advance:
            self.clock.value += .06  # Fresh images at a rate below SET's 20 Hz.
        self.serial.output.clear()
        self.server.tick()
        packets = frames(self.serial.output)
        # The camera only publishes observations/control replies, never motor commands.
        self.assertTrue(all(p[1] in (service.ACK, service.HEARTBEAT,
            service.RING_PX, service.RING_AXIS, service.RING_COUNT) for p in packets))
        group = [p for p in packets if p[1] in
            (service.RING_PX, service.RING_AXIS, service.RING_COUNT)]
        self.assertEqual([p[1] for p in group],
            [3, 0x0a, 0x83] if self.server.config[4] & 1 else [3, 0x83])
        self.assertEqual(len({p[2] & 0x7f for p in group}), 1)
        self.assertEqual(group[-1][3:7], bytes((group[-1][3], 3, 7, 0)))
        return group

    def set_scene(self, gray):
        self.scene[0] = np.repeat(gray[:, :, None], 3, axis=2)

    def assert_count(self, group, count):
        self.assertTrue(group[-1][2] & 0x80)
        self.assertEqual(group[-1][3], count)
        if count != 3:
            for packet in group[:-1]:
                self.assertFalse(packet[2] & 0x80)
                self.assertEqual(packet[3:7], bytes(4))

    def test_cold_two_rings_keep_publishing_across_sequence_wrap_then_recover(self):
        for flags in (0, 1, 2, 3):
            with self.subTest(flags=flags), patch.object(service, 'PREVIEW_ENABLED', False):
                # Deterministic frame time also drives the actual detector/tracker.
                clock_patch = patch.object(service.time, 'monotonic',
                    side_effect=lambda: self.clock.value)
                self.clock = Clock()
                with clock_patch:
                    group = self.start(flags)
                    sequences = []
                    for _ in range(21):
                        self.assert_count(group, 2)
                        sequences.append((group[-1][2] >> 3) & 15)
                        group = self.tick()
                    self.assertEqual(sequences, [i & 15 for i in range(21)])
                    self.assert_count(group, 2)
                    self.set_scene(self.complete)
                    for _ in range(4):
                        group = self.tick()
                        self.assert_count(group, 3)
                    self.assertTrue(group[0][2] & 0x80)
                    if flags & 1:
                        self.assertTrue(group[1][2] & 0x80)

    def test_locked_three_losing_one_reports_two_immediately_and_continuously(self):
        for flags in (0, 3):
            with self.subTest(flags=flags), patch.object(service, 'PREVIEW_ENABLED', False):
                self.clock = Clock()
                with patch.object(service.time, 'monotonic', side_effect=lambda: self.clock.value):
                    self.start(flags, two=False)
                    for _ in range(3):
                        group = self.tick()
                    self.assertTrue(group[0][2] & 0x80)
                    self.set_scene(self.partial)
                    # The transition bypasses the normal interval immediately.
                    self.clock.value += .01
                    group = self.tick(advance=False)
                    self.assert_count(group, 2)
                    for _ in range(8):
                        self.assert_count(self.tick(), 2)
                    self.set_scene(np.full_like(self.complete, 220))
                    self.assert_count(self.tick(), 0)


if __name__ == '__main__':
    unittest.main()
