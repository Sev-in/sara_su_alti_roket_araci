"""Donanim ve ROS 2 gerektirmeyen dalga isleme birim testleri."""

import math
import unittest

from sara_control.wave_processing import (
    WAVE_CALM,
    WAVE_MODERATE,
    WaveEstimatorCore,
    WaveSample,
    quaternion_to_roll_pitch,
    world_vertical_acceleration,
)


class WaveProcessingTests(unittest.TestCase):
    """Dalga ozelligi cikarma ve siniflandirma testleri."""

    def test_level_quaternion_has_zero_roll_pitch(self):
        angles = quaternion_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
        self.assertIsNotNone(angles)
        self.assertAlmostEqual(angles[0], 0.0)
        self.assertAlmostEqual(angles[1], 0.0)

    def test_gravity_is_removed_from_vertical_acceleration(self):
        vertical = world_vertical_acceleration(
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            9.80665,
        )
        self.assertIsNotNone(vertical)
        self.assertAlmostEqual(vertical, 0.0)

    def test_calm_data_remains_calm(self):
        estimator = WaveEstimatorCore(
            window_s=4.0,
            sample_rate_hz=10.0,
            minimum_fill_ratio=0.75,
            enter_hold_s=0.5,
            exit_hold_s=0.5,
        )
        estimate = None
        for index in range(100):
            time_s = index * 0.1
            estimate = estimator.update(
                WaveSample(
                    time_s=time_s,
                    roll_rad=math.radians(0.1)
                    * math.sin(2.0 * math.pi * time_s / 3.0),
                    pitch_rad=0.0,
                    vertical_accel_ms2=0.01,
                    depth_m=0.001,
                )
            )
        self.assertTrue(estimate.ready)
        self.assertEqual(estimate.severity, WAVE_CALM)

    def test_correlated_wave_becomes_moderate(self):
        estimator = WaveEstimatorCore(
            window_s=4.0,
            sample_rate_hz=10.0,
            minimum_fill_ratio=0.75,
            enter_hold_s=0.5,
            exit_hold_s=0.5,
        )
        estimate = None
        for index in range(140):
            time_s = index * 0.1
            phase = 2.0 * math.pi * time_s / 2.5
            estimate = estimator.update(
                WaveSample(
                    time_s=time_s,
                    roll_rad=math.radians(6.0) * math.sin(phase),
                    pitch_rad=math.radians(5.0) * math.sin(
                        phase + 0.4),
                    vertical_accel_ms2=0.55 * math.sin(
                        phase + 1.2),
                    depth_m=0.14 * math.sin(phase),
                )
            )
        self.assertTrue(estimate.ready)
        self.assertGreaterEqual(estimate.severity, WAVE_MODERATE)
        self.assertGreater(estimate.confidence, 0.7)
        self.assertAlmostEqual(
            estimate.dominant_period_s,
            2.5,
            delta=0.25,
        )

    def test_single_noisy_channel_is_capped_at_light(self):
        estimator = WaveEstimatorCore(
            window_s=4.0,
            sample_rate_hz=10.0,
            minimum_fill_ratio=0.75,
            enter_hold_s=0.0,
            exit_hold_s=0.0,
        )
        estimate = None
        for index in range(100):
            time_s = index * 0.1
            estimate = estimator.update(
                WaveSample(
                    time_s=time_s,
                    roll_rad=math.radians(20.0)
                    * math.sin(2.0 * math.pi * time_s / 2.0),
                    pitch_rad=0.0,
                    vertical_accel_ms2=0.0,
                    depth_m=0.0,
                )
            )
        self.assertTrue(estimate.ready)
        self.assertLessEqual(estimate.severity, 1)


if __name__ == '__main__':
    unittest.main()
