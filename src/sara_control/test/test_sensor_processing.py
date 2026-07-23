"""Saf sensor matematik ve filtreleme birim testleri."""

import math
import unittest

from sara_control.sensor_processing import (
    DebouncedBoolean,
    HampelFilter,
    SlidingLinearRegression,
    TimeAwareLowPass,
    normalize_quaternion,
    pressure_to_depth,
    robust_median,
    sen0257_voltage_to_pressure,
    voltage_divider_reconstruct,
)


class SensorProcessingTests(unittest.TestCase):
    """Donanim veya ROS2 gerektirmeyen sensor isleme testleri."""

    def test_sen0257_16_mpa_transfer_function(self):
        self.assertAlmostEqual(sen0257_voltage_to_pressure(0.5), 0.0)
        self.assertAlmostEqual(
            sen0257_voltage_to_pressure(2.5), 800000.0)
        self.assertAlmostEqual(
            sen0257_voltage_to_pressure(4.5), 1600000.0)

    def test_sen0257_transfer_clamps_electrical_overrange(self):
        self.assertAlmostEqual(sen0257_voltage_to_pressure(0.0), 0.0)
        self.assertAlmostEqual(
            sen0257_voltage_to_pressure(5.0), 1600000.0)

    def test_voltage_divider_reconstruction(self):
        adc_voltage = 2.5 * 20000.0 / 30000.0
        reconstructed = voltage_divider_reconstruct(
            adc_voltage,
            r1_ohm=10000.0,
            r2_ohm=20000.0,
        )
        self.assertAlmostEqual(reconstructed, 2.5)

    def test_pressure_to_depth_uses_surface_reference(self):
        rho = 1000.0
        gravity = 9.80665
        surface = 101325.0
        pressure = surface + rho * gravity * 2.0
        depth = pressure_to_depth(
            pressure,
            surface,
            rho,
            gravity,
        )
        self.assertAlmostEqual(depth, 2.0)

    def test_quaternion_validation_and_normalization(self):
        self.assertIsNone(normalize_quaternion(0.0, 0.0, 0.0, 2.0))
        normalized = normalize_quaternion(0.0, 0.0, 0.0, 0.99)
        self.assertIsNotNone(normalized)
        for actual, expected in zip(normalized, (0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)

    def test_time_aware_low_pass_matches_nominal_alpha(self):
        low_pass = TimeAwareLowPass(
            nominal_alpha=0.2,
            nominal_dt_s=0.05,
        )
        self.assertAlmostEqual(low_pass.update(0.0, 0.05), 0.0)
        self.assertAlmostEqual(low_pass.update(1.0, 0.05), 0.2)

    def test_hampel_rejects_single_pressure_spike(self):
        hampel = HampelFilter(
            window_size=7,
            sigma=3.0,
            minimum_deviation=20.0,
        )
        for pressure in (1000.0, 1002.0, 999.0, 1001.0):
            _, is_outlier = hampel.update(pressure)
            self.assertFalse(is_outlier)

        filtered, is_outlier = hampel.update(5000.0)
        self.assertTrue(is_outlier)
        self.assertAlmostEqual(filtered, 1000.5)

    def test_robust_median_excludes_calibration_spike(self):
        center, accepted, span = robust_median(
            [101320.0, 101325.0, 101323.0, 150000.0],
            sigma=3.0,
            minimum_deviation=10.0,
        )
        self.assertAlmostEqual(center, 101323.0)
        self.assertEqual(len(accepted), 3)
        self.assertAlmostEqual(span, 5.0)

    def test_water_sensor_debounce_requires_stable_time(self):
        debounce = DebouncedBoolean(debounce_s=0.3)
        self.assertIsNone(debounce.update(False, 0.0))
        self.assertIsNone(debounce.update(False, 0.29))
        self.assertFalse(debounce.update(False, 0.30))

        self.assertFalse(debounce.update(True, 0.31))
        self.assertFalse(debounce.update(True, 0.60))
        self.assertTrue(debounce.update(True, 0.61))

    def test_sliding_regression_estimates_vertical_speed(self):
        regression = SlidingLinearRegression(
            window_s=0.5,
            max_samples=20,
        )
        slope = None
        for index in range(11):
            timestamp = index * 0.05
            depth = 0.4 * timestamp
            slope = regression.update(timestamp, depth)
        self.assertIsNotNone(slope)
        self.assertAlmostEqual(slope, 0.4)

    def test_filters_reject_non_finite_inputs(self):
        with self.assertRaises(ValueError):
            sen0257_voltage_to_pressure(math.nan)
        with self.assertRaises(ValueError):
            pressure_to_depth(math.inf, 0.0, 1000.0, 9.80665)


if __name__ == '__main__':
    unittest.main()
