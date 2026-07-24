#!/usr/bin/env python3
"""Safe Pixhawk -> PID -> servo and limited motor bench-test coordinator.

This node does not drive hardware directly. It only publishes fixed attitude
targets and a limited test speed through the normal chain:

    navigation -> autopilot -> safety -> actuator_driver -> Uno/PCA9685

Nose-cap, buoyancy and launch requests are always kept disabled.
"""

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class ImuActuatorBench(Node):
    def __init__(self):
        super().__init__('imu_actuator_bench')

        self.declare_parameter('enable_motor', False)
        self.declare_parameter('confirm_motor_test_safe', False)
        self.declare_parameter('motor_test_speed_ms', 0.15)
        self.declare_parameter('max_motor_test_speed_ms', 0.25)
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
        self._motor_enabled = enable_motor and confirmed
        self._motor_speed = (
            max(0.0, min(requested_speed, max_speed))
            if self._motor_enabled
            else 0.0
        )

        self._reference_pose = None
        self._navigation_ready = False

        self.create_subscription(
            Odometry,
            '/sara/navigation/odom',
            self._on_odom,
            10,
        )
        self.create_subscription(
            DiagnosticStatus,
            '/sara/navigation/status',
            self._on_navigation_status,
            10,
        )

        self._target_pose_pub = self.create_publisher(
            PoseStamped, '/sara/guidance/target_pose', 10)
        self._forward_pub = self.create_publisher(
            Bool, '/sara/guidance/forward_motion_request', 10)
        self._speed_pub = self.create_publisher(
            Float64, '/sara/guidance/target_speed', 10)
        self._phase_pub = self.create_publisher(
            String, '/sara/guidance/mission_phase', 10)
        self._permission_pub = self.create_publisher(
            Bool, '/sara/mission_start/motion_permission', 10)
        self._cap_pub = self.create_publisher(
            Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(
            Bool, '/sara/guidance/launch_request', 10)
        self._complete_pub = self.create_publisher(
            Bool, '/sara/guidance/task_complete', 10)

        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish_targets)

        if enable_motor and not confirmed:
            self.get_logger().warn(
                'Motor istendi ancak confirm_motor_test_safe=False. '
                'Motor komutu guvenlik geregi 0 tutulacak.')
        if self._motor_enabled:
            self.get_logger().warn(
                f'BENCH MOTOR TESTI ETKIN: hedef hiz={self._motor_speed:.3f} '
                'm/s. Pervane sokulmus veya arac guvenli test duzeneginde '
                'sabitlenmis olmalidir.')
        else:
            self.get_logger().info(
                'Pixhawk/PID servo bench testi etkin; motor devre disi.')

    def _on_navigation_status(self, msg: DiagnosticStatus):
        values = {item.key: item.value for item in msg.values}
        self._navigation_ready = (
            msg.level == DiagnosticStatus.OK
            and values.get('pixhawk_connected', 'False') == 'True'
            and values.get('depth_valid', 'False') == 'True'
        )

    def _on_odom(self, msg: Odometry):
        if self._reference_pose is not None or not self._navigation_ready:
            return

        target = PoseStamped()
        target.header.frame_id = msg.header.frame_id
        target.pose.position.x = msg.pose.pose.position.x
        target.pose.position.y = msg.pose.pose.position.y
        target.pose.position.z = msg.pose.pose.position.z
        target.pose.orientation = msg.pose.pose.orientation
        self._reference_pose = target
        self.get_logger().info(
            'Pixhawk baslangic yonelimi referans alindi. Araci pitch/yaw '
            'ekseninde hareket ettirdiginizde PID servolari merkeze '
            'dondurmeye calisacak.')

    def _publish_targets(self):
        disabled = Bool()
        disabled.data = False
        self._cap_pub.publish(disabled)
        self._launch_pub.publish(disabled)
        self._complete_pub.publish(disabled)

        phase = String()
        phase.data = 'IMU_ACTUATOR_BENCH_TEST'
        self._phase_pub.publish(phase)

        permitted = Bool()
        permitted.data = self._motor_enabled
        self._permission_pub.publish(permitted)

        forward = Bool()
        forward.data = self._motor_enabled
        self._forward_pub.publish(forward)

        speed = Float64()
        speed.data = self._motor_speed
        self._speed_pub.publish(speed)

        if self._reference_pose is None:
            return

        self._reference_pose.header.stamp = self.get_clock().now().to_msg()
        self._target_pose_pub.publish(self._reference_pose)


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
