#!/usr/bin/env python3
"""Pixhawk IMU ile dogrudan servo ve sinirli motor bench testi.

Servo davranisi, sahada iyi calistigi bildirilen standalone kodla aynidir:

* CH1/pitch: referansa gore pitch farki, +-35 derece
* CH2/yaw: roll ve yaw farkindan mutlak degeri buyuk olan, +-35 derece
* CH3/burun: burun kuru ve kuyruk islaksa acik, aksi halde kapali

Bu dugum donanimi dogrudan surmez. Normalize komutlar yine
safety -> actuator_driver -> Uno/PCA9685 zincirinden gecer. Burun kapagi,
siringa ve atesleme her dongude kapali tutulur.
"""

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64

from sara_control.sensor_processing import DebouncedBoolean


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_diff_deg(current, reference):
    difference = current - reference
    while difference > 180.0:
        difference -= 360.0
    while difference < -180.0:
        difference += 360.0
    return difference


def quaternion_to_euler_deg(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    sin_pitch = clamp(
        2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class ImuActuatorBench(Node):
    def __init__(self):
        super().__init__('imu_actuator_bench')

        self.declare_parameter('enable_motor', False)
        self.declare_parameter('confirm_motor_test_safe', False)
        self.declare_parameter('motor_test_speed_ms', 0.15)
        self.declare_parameter('max_motor_test_speed_ms', 0.25)
        self.declare_parameter('max_calibrated_speed_ms', 1.076)
        self.declare_parameter('pitch_max_angle_deg', 35.0)
        self.declare_parameter('yaw_max_angle_deg', 35.0)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('sign_pitch', -1.0)
        self.declare_parameter('sign_roll', -1.0)
        self.declare_parameter('sign_yaw', -1.0)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('water_timeout_s', 1.0)
        self.declare_parameter('water_debounce_s', 0.3)
        self.declare_parameter('publish_rate_hz', 20.0)

        enable_motor = bool(self.get_parameter('enable_motor').value)
        confirmed = bool(
            self.get_parameter('confirm_motor_test_safe').value)
        requested_speed = float(
            self.get_parameter('motor_test_speed_ms').value)
        max_speed = max(
            0.0,
            float(self.get_parameter('max_motor_test_speed_ms').value),
        )
        calibrated_speed = max(
            1e-6,
            float(self.get_parameter('max_calibrated_speed_ms').value),
        )
        self._motor_enabled = enable_motor and confirmed
        self._motor_speed = (
            max(0.0, min(requested_speed, max_speed))
            if self._motor_enabled
            else 0.0
        )
        self._thrust_request = clamp(
            self._motor_speed / calibrated_speed, 0.0, 1.0)

        self._pitch_max = max(
            1e-6, float(self.get_parameter('pitch_max_angle_deg').value))
        self._yaw_max = max(
            1e-6, float(self.get_parameter('yaw_max_angle_deg').value))
        self._gain = float(self.get_parameter('gain').value)
        self._sign_pitch = float(
            self.get_parameter('sign_pitch').value)
        self._sign_roll = float(self.get_parameter('sign_roll').value)
        self._sign_yaw = float(self.get_parameter('sign_yaw').value)
        self._imu_timeout = max(
            0.1, float(self.get_parameter('imu_timeout_s').value))
        self._water_timeout = max(
            0.1, float(self.get_parameter('water_timeout_s').value))
        water_debounce = max(
            0.0, float(self.get_parameter('water_debounce_s').value))

        self._navigation_ready = False
        self._reference = None
        self._last_odom_time = None
        self._pitch_offset = 0.0
        self._yaw_offset = 0.0
        self._water_1 = None
        self._water_2 = None
        self._last_water_1_time = None
        self._last_water_2_time = None
        self._water_1_debounce = DebouncedBoolean(water_debounce)
        self._water_2_debounce = DebouncedBoolean(water_debounce)

        self.create_subscription(
            Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(
            DiagnosticStatus,
            '/sara/navigation/status',
            self._on_navigation_status,
            10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Bool, '/sara/water_detect_1', self._on_water_1, sensor_qos)
        self.create_subscription(
            Bool, '/sara/water_detect_2', self._on_water_2, sensor_qos)

        self._thrust_pub = self.create_publisher(
            Float64, '/sara/control/thrust_request', 10)
        self._fin_pub = self.create_publisher(
            Vector3, '/sara/control/fin_request', 10)
        self._buoyancy_pub = self.create_publisher(
            Float64, '/sara/control/buoyancy_request', 10)
        self._permission_pub = self.create_publisher(
            Bool, '/sara/mission_start/motion_permission', 10)
        self._cap_pub = self.create_publisher(
            Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(
            Bool, '/sara/guidance/launch_request', 10)
        self._complete_pub = self.create_publisher(
            Bool, '/sara/guidance/task_complete', 10)
        self._status_pub = self.create_publisher(
            DiagnosticStatus, '/sara/bench/status', 10)

        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish_commands)

        if enable_motor and not confirmed:
            self.get_logger().warn(
                'Motor istendi ancak confirm_motor_test_safe=False; '
                'motor komutu 0 tutulacak.')
        if self._motor_enabled:
            self.get_logger().warn(
                f'BENCH MOTOR TESTI ETKIN: hedef hiz={self._motor_speed:.3f} '
                'm/s. Pervane sokulmus veya arac guvenli duzenekte '
                'sabitlenmis olmalidir.')
        self.get_logger().info(
            'Standalone ile uyumlu servo modu: CH1=pitch +-'
            f'{self._pitch_max:.0f} deg, CH2=baskin roll/yaw +-'
            f'{self._yaw_max:.0f} deg.')

    def _on_navigation_status(self, msg):
        values = {item.key: item.value for item in msg.values}
        self._navigation_ready = (
            values.get('pixhawk_connected', 'False') == 'True')

    def _on_water_1(self, msg):
        now = self.get_clock().now()
        stable = self._water_1_debounce.update(
            msg.data, now.nanoseconds * 1e-9)
        if stable is not None:
            self._water_1 = stable
        self._last_water_1_time = now

    def _on_water_2(self, msg):
        now = self.get_clock().now()
        stable = self._water_2_debounce.update(
            msg.data, now.nanoseconds * 1e-9)
        if stable is not None:
            self._water_2 = stable
        self._last_water_2_time = now

    def _on_odom(self, msg):
        self._last_odom_time = self.get_clock().now()
        angles = quaternion_to_euler_deg(msg.pose.pose.orientation)

        if self._reference is None:
            if not self._navigation_ready:
                return
            self._reference = angles
            self.get_logger().info(
                'IMU referansi alindi: '
                f'roll={angles[0]:.1f}, pitch={angles[1]:.1f}, '
                f'yaw={angles[2]:.1f}.')

        delta_roll = angle_diff_deg(angles[0], self._reference[0])
        delta_pitch = angle_diff_deg(angles[1], self._reference[1])
        delta_yaw = angle_diff_deg(angles[2], self._reference[2])

        self._pitch_offset = clamp(
            self._sign_pitch * self._gain * delta_pitch,
            -self._pitch_max,
            self._pitch_max,
        )
        roll_command = self._sign_roll * self._gain * delta_roll
        yaw_command = self._sign_yaw * self._gain * delta_yaw
        dominant = (
            roll_command
            if abs(roll_command) >= abs(yaw_command)
            else yaw_command
        )
        self._yaw_offset = clamp(
            dominant, -self._yaw_max, self._yaw_max)

        self.get_logger().info(
            f'dRoll={delta_roll:6.1f} dPitch={delta_pitch:6.1f} '
            f'dYaw={delta_yaw:6.1f} | CH1={90.0 + self._pitch_offset:5.1f} '
            f'CH2={90.0 + self._yaw_offset:5.1f}',
            throttle_duration_sec=0.3,
        )

    def _imu_fresh(self):
        if self._last_odom_time is None:
            return False
        age = (
            self.get_clock().now() - self._last_odom_time
        ).nanoseconds * 1e-9
        return age <= self._imu_timeout

    def _water_stamp_fresh(self, stamp):
        if stamp is None:
            return False
        age = (
            self.get_clock().now() - stamp
        ).nanoseconds * 1e-9
        return age <= self._water_timeout

    def _nose_open_from_water(self):
        sensors_fresh = (
            self._water_stamp_fresh(self._last_water_1_time)
            and self._water_stamp_fresh(self._last_water_2_time)
        )
        # Standalone kodla ayni karar:
        # burun kuru (False) + kuyruk islak (True) -> CH3=90 derece.
        return bool(
            sensors_fresh
            and self._water_1 is False
            and self._water_2 is True
        )

    def _publish_commands(self):
        ready = (
            self._navigation_ready
            and self._reference is not None
            and self._imu_fresh()
        )

        disabled = Bool()
        disabled.data = False
        self._launch_pub.publish(disabled)
        self._complete_pub.publish(disabled)

        nose_open = self._nose_open_from_water()
        cap = Bool()
        cap.data = nose_open
        self._cap_pub.publish(cap)

        permitted = Bool()
        permitted.data = bool(ready and self._motor_enabled)
        self._permission_pub.publish(permitted)

        thrust = Float64()
        thrust.data = self._thrust_request if ready else 0.0
        self._thrust_pub.publish(thrust)

        fin = Vector3()
        if ready:
            # Bench launch actuator_driver sign'larini +1 yapar. Bu nedenle
            # normalize deger, standalone koddaki fiziksel offseti korur.
            fin.x = self._pitch_offset / self._pitch_max
            fin.y = self._yaw_offset / self._yaw_max
        self._fin_pub.publish(fin)

        buoyancy = Float64()
        buoyancy.data = 0.0
        self._buoyancy_pub.publish(buoyancy)

        status = DiagnosticStatus()
        status.name = 'sara_imu_actuator_bench'
        status.hardware_id = 'pixhawk_uno_pca9685'
        status.level = (
            DiagnosticStatus.OK if ready else DiagnosticStatus.WARN)
        status.message = (
            'Nominal' if ready else 'Pixhawk/IMU referansi bekleniyor')
        status.values = [
            KeyValue(key='imu_ready', value=str(ready)),
            KeyValue(
                key='motor_enabled',
                value=str(self._motor_enabled)),
            KeyValue(
                key='thrust_request',
                value=f'{thrust.data:.3f}'),
            KeyValue(
                key='pitch_servo_deg',
                value=f'{90.0 + self._pitch_offset:.1f}'),
            KeyValue(
                key='yaw_servo_deg',
                value=f'{90.0 + self._yaw_offset:.1f}'),
            KeyValue(
                key='water_1_nose',
                value=str(self._water_1)),
            KeyValue(
                key='water_2_tail',
                value=str(self._water_2)),
            KeyValue(
                key='ch3_nose_open',
                value=str(nose_open)),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ImuActuatorBench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
