import math
import unittest

from sara_control.math_utils import (
    angle_diff_deg,
    clamp,
    euler_to_quaternion,
    quaternion_multiply,
    quaternion_to_euler,
    relative_quaternion,
    wrap_0_2pi,
    wrap_pi,
)


class MathUtilsTests(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)
        self.assertEqual(clamp(0.25, -1.0, 1.0), 0.25)
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)

    def test_angle_wrapping(self):
        self.assertAlmostEqual(wrap_pi(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_0_2pi(-0.5 * math.pi), 1.5 * math.pi)
        self.assertAlmostEqual(angle_diff_deg(190.0, 0.0), -170.0)
        self.assertAlmostEqual(angle_diff_deg(-190.0, 0.0), 170.0)

    def test_identity_quaternion(self):
        roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_euler_quaternion_round_trip(self):
        expected = (math.radians(15.0), math.radians(-20.0), math.radians(35.0))
        quaternion = euler_to_quaternion(*expected)
        actual = quaternion_to_euler(*quaternion)
        for actual_angle, expected_angle in zip(actual, expected):
            self.assertAlmostEqual(actual_angle, expected_angle, places=9)

    def test_relative_quaternion_recovers_pitch_with_rotated_reference(self):
        reference = euler_to_quaternion(
            math.radians(90.0),
            math.radians(15.0),
            math.radians(40.0),
        )
        expected_delta = euler_to_quaternion(
            0.0,
            math.radians(20.0),
            0.0,
        )
        current = quaternion_multiply(reference, expected_delta)

        actual_delta = relative_quaternion(reference, current)
        roll, pitch, yaw = quaternion_to_euler(*actual_delta)

        self.assertAlmostEqual(math.degrees(roll), 0.0, places=9)
        self.assertAlmostEqual(math.degrees(pitch), 20.0, places=9)
        self.assertAlmostEqual(math.degrees(yaw), 0.0, places=9)

    def test_relative_quaternion_rejects_zero_norm(self):
        with self.assertRaises(ValueError):
            relative_quaternion(
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main()
