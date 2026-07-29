import math
import unittest

from sara_control.math_utils import (
    angle_diff_deg,
    clamp,
    euler_to_quaternion,
    quaternion_to_euler,
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


if __name__ == "__main__":
    unittest.main()
