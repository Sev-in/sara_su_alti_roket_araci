import math
import unittest

from sara_control.mission_geometry import (
    forward_distance_from_origin,
    is_at_or_beyond_distance,
)


class TestMissionGeometry(unittest.TestCase):

    def test_forward_distance_ignores_lateral_displacement(self):
        distance = forward_distance_from_origin(
            x=30.0,
            y=40.0,
            origin_x=0.0,
            origin_y=0.0,
            reference_heading=0.0,
        )
        self.assertAlmostEqual(distance, 30.0)

    def test_forward_distance_uses_reference_heading(self):
        distance = forward_distance_from_origin(
            x=5.0,
            y=12.0,
            origin_x=5.0,
            origin_y=2.0,
            reference_heading=math.pi / 2.0,
        )
        self.assertAlmostEqual(distance, 10.0)

    def test_safe_zone_starts_at_minimum_without_upper_limit(self):
        self.assertFalse(is_at_or_beyond_distance(29.99, 30.0))
        self.assertTrue(is_at_or_beyond_distance(30.0, 30.0))
        self.assertTrue(is_at_or_beyond_distance(100.0, 30.0))


if __name__ == '__main__':
    unittest.main()
