import unittest

from sara_control.keyboard_teleop_logic import (
    FORWARD,
    LEFT,
    RIGHT,
    STOP,
    command_for_key,
)


class KeyboardTeleopLogicTests(unittest.TestCase):

    def test_motion_key_mapping(self):
        self.assertEqual(command_for_key('w'), FORWARD)
        self.assertEqual(command_for_key('a'), LEFT)
        self.assertEqual(command_for_key('d'), RIGHT)

    def test_mapping_is_case_insensitive(self):
        self.assertEqual(command_for_key('W'), FORWARD)

    def test_stop_keys(self):
        self.assertEqual(command_for_key('s'), STOP)
        self.assertEqual(command_for_key(' '), STOP)

    def test_turn_sign_matches_project_heading_convention(self):
        self.assertEqual(LEFT.turn_sign, -1)
        self.assertEqual(RIGHT.turn_sign, 1)

    def test_unknown_key_is_ignored(self):
        self.assertIsNone(command_for_key('x'))


if __name__ == '__main__':
    unittest.main()

