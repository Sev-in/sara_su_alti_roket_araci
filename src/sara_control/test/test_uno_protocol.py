import math
import unittest

from sara_control.uno_protocol import (
    gauge_kpa_to_absolute_pa,
    normalized_fin_to_servo,
    normalized_thrust,
    parse_telemetry_line,
)


class UnoProtocolTests(unittest.TestCase):

    def test_telemetry_parser(self):
        values = parse_telemetry_line(
            'voltage_v=0.750000,pressure_kpa=9.807,'
            'motor_state=4,heartbeat_ok=1,ignition_active=1')
        self.assertAlmostEqual(values['pressure_kpa'], 9.807)
        self.assertEqual(values['motor_state'], 4.0)
        self.assertEqual(values['ignition_active'], 1.0)

    def test_status_line_is_not_telemetry(self):
        self.assertEqual(parse_telemetry_line('OK,MOTOR_ARMED'), {})
        self.assertEqual(parse_telemetry_line('ERR,HEARTBEAT_LOST'), {})

    def test_malformed_fields_are_ignored(self):
        values = parse_telemetry_line(
            'voltage_v=0.46,pressure_kpa=,motor_state=broken,'
            'heartbeat_ok=1')
        self.assertEqual(values['voltage_v'], 0.46)
        self.assertEqual(values['heartbeat_ok'], 1.0)
        self.assertNotIn('pressure_kpa', values)
        self.assertNotIn('motor_state', values)

    def test_gauge_pressure_conversion(self):
        pressure = gauge_kpa_to_absolute_pa(9.80665, 101325.0)
        self.assertAlmostEqual(pressure, 111131.65)

    def test_fin_mapping_and_clamp(self):
        self.assertAlmostEqual(
            normalized_fin_to_servo(0.0, 90.0, 30.0, -1.0), 90.0)
        self.assertAlmostEqual(
            normalized_fin_to_servo(1.0, 90.0, 30.0, -1.0), 60.0)
        self.assertAlmostEqual(
            normalized_fin_to_servo(2.0, 90.0, 30.0, 1.0), 120.0)

    def test_thrust_clamp_and_invalid(self):
        self.assertEqual(normalized_thrust(-1.0), 0.0)
        self.assertEqual(normalized_thrust(2.0), 1.0)
        with self.assertRaises(ValueError):
            normalized_thrust(math.nan)


if __name__ == '__main__':
    unittest.main()
