#!/usr/bin/env python3
"""Pixhawk IMU ve basinc derinliginden dalga etkisi kestiren ROS 2 dugumu.

Bu dugum mevcut navigasyon ve kontrol zincirine komut yazmaz. Yalnizca
dalga seviyesi, guven puani, onerilen hiz carpani ve tani durumu yayinlar.
Boylece deniz verileriyle esikler dogrulanmadan gorev davranisi degismez.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt8

from sara_control.wave_processing import (
    WAVE_CALM,
    WAVE_LIGHT,
    WAVE_MODERATE,
    WAVE_NAMES,
    WAVE_SEVERE,
    WaveEstimatorCore,
    WaveSample,
    quaternion_to_roll_pitch,
    world_vertical_acceleration,
)


class WaveEstimatorNode(Node):
    """Sensor fuzyonlu ve monitor-modlu dalga kestirim dugumu."""

    def __init__(self):
        super().__init__('wave_estimator_node')

        self.declare_parameter('imu_topic', '/sara/imu/data')
        self.declare_parameter('odom_topic', '/sara/navigation/odom')
        self.declare_parameter(
            'thrust_topic',
            '/sara/control/thrust_command',
        )
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('window_s', 8.0)
        self.declare_parameter('minimum_fill_ratio', 0.75)
        self.declare_parameter('gravity_ms2', 9.80665)
        self.declare_parameter('imu_timeout_s', 1.0)
        self.declare_parameter('odom_timeout_s', 1.0)
        self.declare_parameter('thrust_timeout_s', 1.0)
        self.declare_parameter('thrust_change_threshold', 0.12)
        self.declare_parameter('maneuver_ignore_s', 1.5)
        self.declare_parameter('roll_rms_light_deg', 2.0)
        self.declare_parameter('pitch_rms_light_deg', 2.0)
        self.declare_parameter(
            'vertical_accel_rms_light_ms2',
            0.20,
        )
        self.declare_parameter(
            'depth_peak_to_peak_light_m',
            0.06,
        )
        self.declare_parameter('moderate_score', 2.0)
        self.declare_parameter('severe_score', 4.0)
        self.declare_parameter('enter_hold_s', 2.0)
        self.declare_parameter('exit_hold_s', 4.0)
        self.declare_parameter('min_period_s', 1.0)
        self.declare_parameter('max_period_s', 10.0)
        self.declare_parameter('light_speed_scale', 1.0)
        self.declare_parameter('moderate_speed_scale', 0.8)
        self.declare_parameter('severe_speed_scale', 0.0)

        self.sample_rate_hz = float(
            self.get_parameter('sample_rate_hz').value)
        self.gravity_ms2 = float(
            self.get_parameter('gravity_ms2').value)
        self.imu_timeout_s = float(
            self.get_parameter('imu_timeout_s').value)
        self.odom_timeout_s = float(
            self.get_parameter('odom_timeout_s').value)
        self.thrust_timeout_s = float(
            self.get_parameter('thrust_timeout_s').value)
        self.thrust_change_threshold = float(
            self.get_parameter('thrust_change_threshold').value)
        self.maneuver_ignore_s = float(
            self.get_parameter('maneuver_ignore_s').value)
        self.speed_scales = {
            WAVE_CALM: 1.0,
            WAVE_LIGHT: float(
                self.get_parameter('light_speed_scale').value),
            WAVE_MODERATE: float(
                self.get_parameter('moderate_speed_scale').value),
            WAVE_SEVERE: float(
                self.get_parameter('severe_speed_scale').value),
        }

        self._core = WaveEstimatorCore(
            window_s=float(self.get_parameter('window_s').value),
            sample_rate_hz=self.sample_rate_hz,
            minimum_fill_ratio=float(
                self.get_parameter('minimum_fill_ratio').value),
            roll_rms_light_deg=float(
                self.get_parameter('roll_rms_light_deg').value),
            pitch_rms_light_deg=float(
                self.get_parameter('pitch_rms_light_deg').value),
            vertical_accel_rms_light_ms2=float(
                self.get_parameter(
                    'vertical_accel_rms_light_ms2').value),
            depth_peak_to_peak_light_m=float(
                self.get_parameter(
                    'depth_peak_to_peak_light_m').value),
            moderate_score=float(
                self.get_parameter('moderate_score').value),
            severe_score=float(
                self.get_parameter('severe_score').value),
            enter_hold_s=float(
                self.get_parameter('enter_hold_s').value),
            exit_hold_s=float(
                self.get_parameter('exit_hold_s').value),
            min_period_s=float(
                self.get_parameter('min_period_s').value),
            max_period_s=float(
                self.get_parameter('max_period_s').value),
        )

        self._roll = None
        self._pitch = None
        self._vertical_accel = None
        self._depth = None
        self._thrust = 0.0
        self._last_imu_time = None
        self._last_odom_time = None
        self._last_thrust_time = None
        self._maneuver_until_s = 0.0
        self._last_estimate = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self._on_imu,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            sensor_qos,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter('thrust_topic').value),
            self._on_thrust,
            sensor_qos,
        )

        self._level_pub = self.create_publisher(
            UInt8,
            '/sara/environment/wave_level',
            10,
        )
        self._confidence_pub = self.create_publisher(
            Float64,
            '/sara/environment/wave_confidence',
            10,
        )
        self._speed_scale_pub = self.create_publisher(
            Float64,
            '/sara/environment/recommended_speed_scale',
            10,
        )
        self._status_pub = self.create_publisher(
            DiagnosticStatus,
            '/sara/environment/wave_status',
            10,
        )

        self.create_timer(1.0 / self.sample_rate_hz, self._on_timer)
        self.get_logger().info(
            'wave_estimator baslatildi: yalnizca izleme/telemetri; '
            'gorev komutlarina dogrudan mudahale etmez.')

    def _now_s(self):
        """ROS saatini saniye olarak dondur."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        now_s = self._now_s()
        orientation = msg.orientation
        angles = quaternion_to_roll_pitch(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        acceleration = msg.linear_acceleration
        vertical_accel = world_vertical_acceleration(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            acceleration.x,
            acceleration.y,
            acceleration.z,
            self.gravity_ms2,
        )
        if angles is None or vertical_accel is None:
            return
        self._roll, self._pitch = angles
        self._vertical_accel = vertical_accel
        self._last_imu_time = now_s

    def _on_odom(self, msg):
        depth = float(msg.pose.pose.position.z)
        if not math.isfinite(depth):
            return
        self._depth = depth
        self._last_odom_time = self._now_s()

    def _on_thrust(self, msg):
        now_s = self._now_s()
        thrust = float(msg.data)
        if not math.isfinite(thrust):
            return
        if abs(thrust - self._thrust) >= self.thrust_change_threshold:
            self._maneuver_until_s = now_s + self.maneuver_ignore_s
        self._thrust = thrust
        self._last_thrust_time = now_s

    @staticmethod
    def _fresh(now_s, stamp_s, timeout_s):
        """Bir girdinin zaman asimina ugramadigini kontrol et."""
        return (
            stamp_s is not None
            and now_s - stamp_s <= timeout_s
        )

    def _on_timer(self):
        now_s = self._now_s()
        imu_fresh = self._fresh(
            now_s,
            self._last_imu_time,
            self.imu_timeout_s,
        )
        odom_fresh = self._fresh(
            now_s,
            self._last_odom_time,
            self.odom_timeout_s,
        )
        thrust_fresh = self._fresh(
            now_s,
            self._last_thrust_time,
            self.thrust_timeout_s,
        )
        inputs_ready = (
            imu_fresh
            and odom_fresh
            and self._roll is not None
            and self._depth is not None
        )
        if not inputs_ready:
            self._publish_stale(
                imu_fresh,
                odom_fresh,
                thrust_fresh,
            )
            return

        maneuver_suppressed = now_s < self._maneuver_until_s
        estimate = self._core.update(
            WaveSample(
                time_s=now_s,
                roll_rad=self._roll,
                pitch_rad=self._pitch,
                vertical_accel_ms2=self._vertical_accel,
                depth_m=self._depth,
            ),
            maneuver_suppressed=maneuver_suppressed,
        )
        self._last_estimate = estimate
        self._publish_estimate(
            estimate,
            imu_fresh,
            odom_fresh,
            thrust_fresh,
        )

    def _publish_stale(self, imu_fresh, odom_fresh, thrust_fresh):
        """Eksik girdide guvenli ve acik tani mesaji yayinla."""
        level = UInt8()
        level.data = WAVE_CALM
        self._level_pub.publish(level)

        confidence = Float64()
        confidence.data = 0.0
        self._confidence_pub.publish(confidence)

        speed_scale = Float64()
        speed_scale.data = 1.0
        self._speed_scale_pub.publish(speed_scale)

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = 'sara_wave_estimator'
        status.hardware_id = 'pixhawk_sen0257'
        status.message = 'Dalga kestirimi icin sensor verisi bekleniyor'
        status.values = [
            KeyValue(key='ready', value='False'),
            KeyValue(key='imu_fresh', value=str(imu_fresh)),
            KeyValue(key='odom_depth_fresh', value=str(odom_fresh)),
            KeyValue(key='thrust_fresh', value=str(thrust_fresh)),
            KeyValue(key='control_influence', value='False'),
        ]
        self._status_pub.publish(status)

    def _publish_estimate(
        self,
        estimate,
        imu_fresh,
        odom_fresh,
        thrust_fresh,
    ):
        """Dalga kestirim topic'lerini yayinla."""
        level = UInt8()
        level.data = int(estimate.severity)
        self._level_pub.publish(level)

        confidence = Float64()
        confidence.data = float(estimate.confidence)
        self._confidence_pub.publish(confidence)

        speed_scale = Float64()
        speed_scale.data = float(
            self.speed_scales.get(estimate.severity, 1.0))
        self._speed_scale_pub.publish(speed_scale)

        status = DiagnosticStatus()
        status.name = 'sara_wave_estimator'
        status.hardware_id = 'pixhawk_sen0257'
        if not estimate.ready:
            status.level = DiagnosticStatus.WARN
            status.message = 'Dalga analiz penceresi doluyor'
        elif estimate.severity == WAVE_SEVERE:
            status.level = DiagnosticStatus.WARN
            status.message = 'SIDDETLI dalga etkisi kestirildi'
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                'Dalga seviyesi: '
                + WAVE_NAMES.get(estimate.severity, 'BILINMIYOR')
            )
        period_value = (
            '-'
            if not math.isfinite(estimate.dominant_period_s)
            else f'{estimate.dominant_period_s:.2f}'
        )
        status.values = [
            KeyValue(key='ready', value=str(estimate.ready)),
            KeyValue(
                key='wave_level',
                value=str(estimate.severity),
            ),
            KeyValue(
                key='wave_name',
                value=WAVE_NAMES.get(
                    estimate.severity,
                    'BILINMIYOR',
                ),
            ),
            KeyValue(
                key='raw_wave_level',
                value=str(estimate.raw_severity),
            ),
            KeyValue(
                key='confidence',
                value=f'{estimate.confidence:.3f}',
            ),
            KeyValue(key='score', value=f'{estimate.score:.3f}'),
            KeyValue(
                key='roll_rms_deg',
                value=f'{estimate.roll_rms_deg:.3f}',
            ),
            KeyValue(
                key='pitch_rms_deg',
                value=f'{estimate.pitch_rms_deg:.3f}',
            ),
            KeyValue(
                key='vertical_accel_rms_ms2',
                value=(
                    f'{estimate.vertical_accel_rms_ms2:.3f}'
                ),
            ),
            KeyValue(
                key='depth_peak_to_peak_m',
                value=f'{estimate.depth_peak_to_peak_m:.3f}',
            ),
            KeyValue(
                key='dominant_period_s',
                value=period_value,
            ),
            KeyValue(
                key='maneuver_suppressed',
                value=str(estimate.maneuver_suppressed),
            ),
            KeyValue(
                key='sample_count',
                value=str(estimate.sample_count),
            ),
            KeyValue(
                key='window_span_s',
                value=f'{estimate.window_span_s:.2f}',
            ),
            KeyValue(key='imu_fresh', value=str(imu_fresh)),
            KeyValue(
                key='odom_depth_fresh',
                value=str(odom_fresh),
            ),
            KeyValue(key='thrust_fresh', value=str(thrust_fresh)),
            KeyValue(key='control_influence', value='False'),
        ]
        self._status_pub.publish(status)


def main(args=None):
    """ROS 2 giris noktasi."""
    rclpy.init(args=args)
    node = WaveEstimatorNode()
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
