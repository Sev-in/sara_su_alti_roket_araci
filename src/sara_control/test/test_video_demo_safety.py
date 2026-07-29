import math
import unittest

from sara_control.video_demo_safety import video_demo_surface_gate


class VideoDemoSafetyTests(unittest.TestCase):

    def _valid_inputs(self):
        return {
            'demo_mode': True,
            'ignition_enabled': True,
            'depth_valid': True,
            'surface_detected': True,
            'depth_m': 0.05,
            'max_depth_m': 0.15,
            'pitch_rad': math.radians(2.0),
            'max_abs_pitch_rad': math.radians(10.0),
            'forward_speed_ms': 0.02,
            'max_speed_ms': 0.10,
            'actuator_status_fresh': True,
            'measurement_mode': True,
        }

    def test_all_conditions_allow_measurement_demo(self):
        self.assertTrue(video_demo_surface_gate(**self._valid_inputs()))

    def test_each_boolean_safety_condition_is_required(self):
        for key in (
            'demo_mode',
            'ignition_enabled',
            'depth_valid',
            'surface_detected',
            'actuator_status_fresh',
            'measurement_mode',
        ):
            values = self._valid_inputs()
            values[key] = False
            with self.subTest(key=key):
                self.assertFalse(video_demo_surface_gate(**values))

    def test_depth_pitch_and_speed_limits_are_required(self):
        invalid_values = {
            'depth_m': 0.16,
            'pitch_rad': math.radians(10.1),
            'forward_speed_ms': 0.11,
        }
        for key, value in invalid_values.items():
            values = self._valid_inputs()
            values[key] = value
            with self.subTest(key=key):
                self.assertFalse(video_demo_surface_gate(**values))

    def test_non_finite_sensor_value_is_rejected(self):
        values = self._valid_inputs()
        values['depth_m'] = math.nan
        self.assertFalse(video_demo_surface_gate(**values))


if __name__ == '__main__':
    unittest.main()
