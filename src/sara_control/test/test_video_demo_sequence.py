import unittest

from sara_control.video_demo_sequence import (
    MANEUVER_STEPS,
    maneuver_step_for_elapsed,
    total_maneuver_duration_s,
    validate_maneuver_duration,
)


class VideoDemoSequenceTests(unittest.TestCase):

    def test_maneuver_sequence_exceeds_one_minute(self):
        self.assertGreaterEqual(total_maneuver_duration_s(), 65.0)
        self.assertEqual(total_maneuver_duration_s(), 68.0)

    def test_sequence_contains_route_and_pitch_demonstrations(self):
        headings = {step.heading_offset_deg for step in MANEUVER_STEPS}
        pitches = {step.pitch_deg for step in MANEUVER_STEPS}
        self.assertIn(25.0, headings)
        self.assertIn(-25.0, headings)
        self.assertIn(8.0, pitches)
        self.assertIn(-8.0, pitches)

    def test_elapsed_time_selects_expected_steps(self):
        self.assertEqual(
            maneuver_step_for_elapsed(0.0)[1].phase,
            'VIDEO_DEMO_DUZ_SEYIR_1',
        )
        self.assertEqual(
            maneuver_step_for_elapsed(12.0)[1].phase,
            'VIDEO_DEMO_ROTA_SAG',
        )
        self.assertEqual(
            maneuver_step_for_elapsed(67.9)[1].phase,
            'VIDEO_DEMO_DUZ_SEYIR_2',
        )
        self.assertIsNone(maneuver_step_for_elapsed(68.0))

    def test_minimum_duration_validation_rejects_short_sequence(self):
        with self.assertRaises(ValueError):
            validate_maneuver_duration(69.0)


if __name__ == '__main__':
    unittest.main()
