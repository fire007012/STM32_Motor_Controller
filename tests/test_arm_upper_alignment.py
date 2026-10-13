import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from arm_upper_alignment import ArmUpperAlignment


def frame(sensor, mm, *, sequence=1, status=1):
    return bytes([1, sensor, mm & 255, mm >> 8, 255, 255, status, sequence])


class UpperArmAlignment(unittest.TestCase):
    def setUp(self):
        self.control = ArmUpperAlignment()
        self.control.set_enabled(True)

    def pair(self, left, right, *, now=1.0, sequence=1):
        self.control.update_can_frame(0x110, frame(3, left, sequence=sequence), now)
        self.control.update_can_frame(0x110, frame(4, right, sequence=sequence), now)

    def test_stage_gate_and_fresh_pair_after_enabling(self):
        self.control.set_enabled(False)
        self.pair(500, 1000)
        self.assertEqual(self.control.lateral_direction(1.01), 0)
        self.control.set_enabled(True)
        self.assertEqual(self.control.lateral_direction(1.02), 0)
        self.pair(500, 1000, now=1.03)
        self.assertEqual(self.control.lateral_direction(1.04), 1)
        self.control.set_enabled(False)
        self.assertEqual(self.control.lateral_direction(1.05), 0)

    def test_move_toward_shorter_side_and_stop_at_equal(self):
        self.pair(500, 1000)
        self.assertEqual(self.control.lateral_direction(1.01), 1)
        self.pair(1000, 500, now=1.05, sequence=2)
        self.assertEqual(self.control.lateral_direction(1.06), -1)
        self.pair(500, 500, now=1.1, sequence=3)
        self.assertEqual(self.control.lateral_direction(1.11), 0)

    def test_hysteresis_avoids_noise_and_wrong_direction(self):
        self.pair(500, 530)
        self.assertEqual(self.control.lateral_direction(1.01), 0)
        self.pair(500, 560, now=1.02, sequence=2)
        self.assertEqual(self.control.lateral_direction(1.03), 1)
        self.pair(500, 530, now=1.04, sequence=3)
        self.assertEqual(self.control.lateral_direction(1.05), 1)
        self.pair(530, 500, now=1.06, sequence=4)
        self.assertEqual(self.control.lateral_direction(1.07), 0)
        self.pair(600, 500, now=1.08, sequence=5)
        self.assertEqual(self.control.lateral_direction(1.09), -1)
        self.pair(515, 500, now=1.1, sequence=6)
        self.assertEqual(self.control.lateral_direction(1.11), 0)

    def test_invalid_out_of_range_or_emergency_cannot_drive(self):
        for sensor in (3, 4):
            for mm, status in [(65535, 4), (600, 0), (600, 0x11),
                               (600, 0x21), (0, 1), (3001, 1)]:
                with self.subTest(sensor=sensor, mm=mm, status=status):
                    self.control = ArmUpperAlignment()
                    self.control.set_enabled(True)
                    self.pair(500, 1000)
                    self.assertEqual(self.control.lateral_direction(1.01), 1)
                    self.control.update_can_frame(0x110, frame(sensor, mm, status=status,
                                                               sequence=2), 1.02)
                    self.assertEqual(self.control.lateral_direction(1.03), 0)

    def test_stale_missing_or_skewed_pair_stops(self):
        self.control.update_can_frame(0x110, frame(3, 500), 1.0)
        self.assertEqual(self.control.lateral_direction(1.01), 0)
        self.control.update_can_frame(0x110, frame(4, 1000), 1.12)
        self.assertEqual(self.control.lateral_direction(1.13), 0)
        self.pair(500, 1000, now=1.2, sequence=2)
        self.assertEqual(self.control.lateral_direction(1.21), 1)
        self.assertEqual(self.control.lateral_direction(1.41), 0)

    def test_duplicate_or_backward_time_cannot_refresh_sample(self):
        self.pair(500, 1000)
        for now in (1.1, 1.25):
            self.assertFalse(self.control.update_can_frame(0x110, frame(3, 500), now))
            self.assertFalse(self.control.update_can_frame(0x110, frame(4, 1000), now))
        self.assertEqual(self.control.lateral_direction(1.26), 0)
        self.assertFalse(self.control.update_can_frame(0x110, frame(3, 500, sequence=2), 0.9))

    def test_independent_sequence_wrap_is_accepted(self):
        self.pair(500, 1000, sequence=255)
        self.assertEqual(self.control.lateral_direction(1.01), 1)
        self.pair(1000, 500, now=1.02, sequence=0)
        self.assertEqual(self.control.lateral_direction(1.03), -1)

    def test_other_sensors_and_malformed_frames_do_not_affect_alignment(self):
        self.pair(500, 1000)
        for sid in (0, 1, 2, 5):
            self.assertFalse(self.control.update_can_frame(0x110, frame(sid, 600), 1.02))
        for kwargs in ({'is_extended_id': True}, {'is_remote_frame': True}):
            self.assertFalse(self.control.update_can_frame(0x110, frame(3, 900), 1.02, **kwargs))
        self.assertFalse(self.control.update_can_frame(0x111, frame(3, 600), 1.02))
        self.assertFalse(self.control.update_can_frame(0x110, b'\x01\x03', 1.02))
        self.assertEqual(self.control.lateral_direction(1.03), 1)


if __name__ == '__main__':
    unittest.main()
