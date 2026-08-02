#!/usr/bin/env python3
"""SARA icin guvenlik zincirini atlamayan klavye gudum dugumu.

Bu dugum dogrudan servo veya ESC komutu uretmez. Standart guidance
topic'lerini yayinlar; komutlar autopilot ve safety katmanlarindan gecerek
actuator_driver'a ulasir. Hareket tusu yenilenmezse deadman zaman asimi
motor istegini otomatik olarak sifirlar.
"""

import math
import os
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from sara_control.keyboard_teleop_logic import STOP, command_for_key
from sara_control.math_utils import (
    euler_to_quaternion,
    quaternion_to_yaw_pitch_roll,
    wrap_pi,
)


class KeyboardTeleopGuidanceNode(Node):
    """W/A/D komutlarini standart gudum hedeflerine donusturur."""

    def __init__(self):
        super().__init__('guidance_node')

        self.declare_parameter('forward_speed_ms', 0.18)
        self.declare_parameter('turn_offset_deg', 25.0)
        self.declare_parameter('command_timeout_s', 0.8)
        self.declare_parameter('nav_status_timeout_s', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self.forward_speed = float(
            self.get_parameter('forward_speed_ms').value)
        self.turn_offset = math.radians(float(
            self.get_parameter('turn_offset_deg').value))
        self.command_timeout = float(
            self.get_parameter('command_timeout_s').value)
        self.nav_status_timeout = float(
            self.get_parameter('nav_status_timeout_s').value)

        if self.forward_speed <= 0.0:
            raise ValueError('forward_speed_ms pozitif olmali')
        if not 0.0 < self.command_timeout <= 2.0:
            raise ValueError('command_timeout_s 0 ile 2 saniye arasinda olmali')
        if not 0.0 < self.turn_offset <= math.radians(45.0):
            raise ValueError('turn_offset_deg 0 ile 45 derece arasinda olmali')

        self._depth = 0.0
        self._heading = 0.0
        self._target_depth = 0.0
        self._target_heading = 0.0
        self._odom_received = False
        self._pixhawk_connected = False
        self._depth_valid = False
        self._nav_status_level = DiagnosticStatus.STALE
        self._last_nav_status_time = None
        self._motion_permission = False
        self._emergency_stop = False
        self._leak_detected = False
        self._active_command = STOP
        self._last_motion_key_time = None
        self._tty_stream = None
        self._tty_settings = None
        self._keyboard_error = '-'

        self.create_subscription(
            Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(
            DiagnosticStatus, '/sara/navigation/status',
            self._on_nav_status, 10)
        self.create_subscription(
            Bool, '/sara/mission_start/motion_permission',
            self._on_motion_permission, 10)
        self.create_subscription(
            Bool, '/sara/safety/emergency_stop',
            self._on_emergency_stop, 10)
        self.create_subscription(
            Bool, '/sara/safety/leak_detected', self._on_leak, 10)

        self._target_pub = self.create_publisher(
            PoseStamped, '/sara/guidance/target_pose', 10)
        self._phase_pub = self.create_publisher(
            String, '/sara/guidance/mission_phase', 10)
        self._forward_pub = self.create_publisher(
            Bool, '/sara/guidance/forward_motion_request', 10)
        self._speed_pub = self.create_publisher(
            Float64, '/sara/guidance/target_speed', 10)
        self._nose_pub = self.create_publisher(
            Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(
            Bool, '/sara/guidance/launch_request', 10)
        self._complete_pub = self.create_publisher(
            Bool, '/sara/guidance/task_complete', 10)
        self._status_pub = self.create_publisher(
            DiagnosticStatus, '/sara/guidance/status', 10)

        self._open_keyboard()
        rate = max(
            5.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().warning(
            'KLAVYE TELEOP ETKIN: W=ileri, A=sol, D=sag, '
            'S/SPACE=dur. Hareket icin tusu basili tutun; '
            f'{self.command_timeout:.1f} sn veri gelmezse motor durur.')

    def _open_keyboard(self):
        try:
            fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self._tty_stream = os.fdopen(fd, 'r', buffering=1)
            self._tty_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except (OSError, termios.error) as exc:
            self._keyboard_error = str(exc)
            self.get_logger().error(
                f'Klavye terminali acilamadi: {exc}. Hareket engellendi.')
            self._close_keyboard()

    def _close_keyboard(self):
        if self._tty_stream is None:
            return
        try:
            if self._tty_settings is not None:
                termios.tcsetattr(
                    self._tty_stream.fileno(),
                    termios.TCSADRAIN,
                    self._tty_settings,
                )
        except (OSError, termios.error):
            pass
        try:
            self._tty_stream.close()
        except OSError:
            pass
        self._tty_stream = None

    def _on_odom(self, msg):
        self._depth = float(msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        self._heading, _pitch, _roll = quaternion_to_yaw_pitch_roll(
            q.x, q.y, q.z, q.w)
        if not self._odom_received:
            self._target_depth = self._depth
            self._target_heading = self._heading
            self._odom_received = True

    def _on_nav_status(self, msg):
        self._last_nav_status_time = self.get_clock().now()
        self._nav_status_level = msg.level
        values = {item.key: item.value for item in msg.values}
        self._pixhawk_connected = (
            values.get('pixhawk_connected', 'False') == 'True')
        self._depth_valid = values.get('depth_valid', 'False') == 'True'

    def _on_motion_permission(self, msg):
        self._motion_permission = bool(msg.data)

    def _on_emergency_stop(self, msg):
        self._emergency_stop = bool(msg.data)

    def _on_leak(self, msg):
        self._leak_detected = bool(msg.data)

    def _nav_ready(self):
        if not self._odom_received or self._last_nav_status_time is None:
            return False
        age = (
            self.get_clock().now() - self._last_nav_status_time
        ).nanoseconds * 1e-9
        return (
            age < self.nav_status_timeout
            and self._nav_status_level != DiagnosticStatus.ERROR
            and self._pixhawk_connected
            and self._depth_valid
        )

    def _poll_keyboard(self):
        if self._tty_stream is None:
            return
        while select.select([self._tty_stream], [], [], 0.0)[0]:
            key = self._tty_stream.read(1)
            command = command_for_key(key)
            if command is None:
                continue
            if command is STOP:
                self._stop_motion()
                continue
            if self._active_command is STOP:
                self._target_depth = self._depth
                self._target_heading = self._heading
            self._active_command = command
            self._last_motion_key_time = self.get_clock().now()

    def _stop_motion(self):
        self._active_command = STOP
        self._last_motion_key_time = None
        self._target_depth = self._depth
        self._target_heading = self._heading

    def _motion_key_fresh(self):
        if self._last_motion_key_time is None:
            return False
        age = (
            self.get_clock().now() - self._last_motion_key_time
        ).nanoseconds * 1e-9
        return age < self.command_timeout

    def _on_timer(self):
        self._poll_keyboard()

        allowed = (
            self._tty_stream is not None
            and self._nav_ready()
            and self._motion_permission
            and not self._emergency_stop
            and not self._leak_detected
            and self._motion_key_fresh()
        )
        if not allowed:
            self._stop_motion()

        moving = allowed and self._active_command.forward
        if moving and self._active_command.turn_sign:
            self._target_heading = wrap_pi(
                self._heading
                + self._active_command.turn_sign * self.turn_offset)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'sara_odom'
        pose.pose.position.z = float(self._target_depth)
        qx, qy, qz, qw = euler_to_quaternion(
            0.0, 0.0, self._target_heading)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._target_pub.publish(pose)

        phase = String()
        phase.data = f'KEYBOARD_TELEOP_{self._active_command.name}'
        self._phase_pub.publish(phase)

        forward = Bool()
        forward.data = bool(moving)
        self._forward_pub.publish(forward)

        speed = Float64()
        speed.data = self.forward_speed if moving else 0.0
        self._speed_pub.publish(speed)
        self._nose_pub.publish(Bool(data=False))
        self._launch_pub.publish(Bool(data=False))
        self._complete_pub.publish(Bool(data=False))

        status = DiagnosticStatus()
        status.name = 'sara_guidance'
        status.hardware_id = 'jetson_orin_nano_keyboard_teleop'
        ready = self._tty_stream is not None and self._nav_ready()
        status.level = (
            DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR)
        status.message = (
            f'KLAVYE_TELEOP - {self._active_command.name}'
            if ready else 'KLAVYE_TELEOP hazir degil')
        status.values = [
            KeyValue(key='teleop_mode', value='True'),
            KeyValue(key='command', value=self._active_command.name),
            KeyValue(key='moving', value=str(moving)),
            KeyValue(
                key='motion_permission',
                value=str(self._motion_permission)),
            KeyValue(key='nav_ready', value=str(self._nav_ready())),
            KeyValue(
                key='keyboard_ready',
                value=str(self._tty_stream is not None)),
            KeyValue(key='keyboard_error', value=self._keyboard_error),
            KeyValue(
                key='target_speed_ms', value=f'{speed.data:.3f}'),
            KeyValue(
                key='target_heading_deg',
                value=f'{math.degrees(self._target_heading):.1f}'),
            KeyValue(
                key='target_depth_m', value=f'{self._target_depth:.2f}'),
            KeyValue(
                key='deadman_timeout_s',
                value=f'{self.command_timeout:.2f}'),
        ]
        self._status_pub.publish(status)

    def destroy_node(self):
        self._stop_motion()
        self._close_keyboard()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopGuidanceNode()
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

