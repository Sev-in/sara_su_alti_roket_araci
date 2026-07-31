#!/usr/bin/env python3
"""Havuz videosu icin sinirli, otonom ve guvenlik-katmanli gudum dugumu.

Bu dugum normal ``guidance`` yerine yalnizca ``video_demo.launch.py`` ile
calisir. En az bir dakikalik su alti manevrasini tamamlar, burun kapagi
kapaliyken kontrollu olarak yuzeye cikar ve acikca etkinlestirilmisse sadece
olcum firmware'iyle burun kapagi/3 V sinyal gosterimini yapar.
"""

import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from sara_control.math_utils import (
    euler_to_quaternion,
    quaternion_to_yaw_pitch_roll,
    wrap_pi,
)
from sara_control.video_demo_sequence import (
    MANEUVER_STEPS,
    validate_maneuver_duration,
)


PHASE_REFERANSLAMA = 'VIDEO_DEMO_REFERANSLAMA'
PHASE_DALIS = 'VIDEO_DEMO_1M_DALIS'
PHASE_YUZEYE_CIKIS = 'VIDEO_DEMO_YUZEYE_CIKIS_KAPAK_KAPALI'
PHASE_YUZEY_KANIT = 'VIDEO_DEMO_YUZEYDE_KAPAK_KAPALI_KANIT'
PHASE_TAMAMLANDI = 'VIDEO_DEMO_TAMAMLANDI'
PHASE_GUVENLI_SONLANDIRMA = 'VIDEO_DEMO_GUVENLI_SONLANDIRMA'

MANEUVER_PHASES = {step.phase for step in MANEUVER_STEPS}
TERMINAL_PHASES = {PHASE_TAMAMLANDI, PHASE_GUVENLI_SONLANDIRMA}


class VideoDemoGuidanceNode(Node):
    """VIDEO_DEMO hedeflerini standart gudum arayuzunden yayinlar."""

    def __init__(self):
        super().__init__('guidance_node')

        self.declare_parameter('minimum_depth_m', 1.0)
        self.declare_parameter('target_depth_m', 1.10)
        self.declare_parameter('depth_tolerance_m', 0.15)
        self.declare_parameter('minimum_maneuver_duration_s', 65.0)
        self.declare_parameter('maneuver_speed_ms', 0.18)
        self.declare_parameter('dive_speed_ms', 0.22)
        self.declare_parameter('surface_speed_ms', 0.15)
        self.declare_parameter('surface_pitch_deg', 0.0)
        self.declare_parameter('condition_hold_s', 2.0)
        self.declare_parameter('tracking_hold_s', 1.0)
        self.declare_parameter('heading_tracking_tolerance_deg', 15.0)
        self.declare_parameter('pitch_tracking_tolerance_deg', 10.0)
        self.declare_parameter('maneuver_step_timeout_s', 35.0)
        self.declare_parameter('surface_cap_closed_hold_s', 5.0)
        self.declare_parameter('dive_timeout_s', 35.0)
        self.declare_parameter('surface_timeout_s', 35.0)
        self.declare_parameter('max_demo_duration_s', 180.0)
        self.declare_parameter('nav_status_timeout_s', 2.0)
        self.declare_parameter('motion_permission_timeout_s', 1.0)
        self.declare_parameter('startup_grace_period_s', 8.0)
        self.declare_parameter('control_rate_hz', 10.0)

        self.minimum_depth = float(
            self.get_parameter('minimum_depth_m').value)
        self.target_depth = float(
            self.get_parameter('target_depth_m').value)
        self.depth_tolerance = float(
            self.get_parameter('depth_tolerance_m').value)
        self.minimum_maneuver_duration = float(
            self.get_parameter('minimum_maneuver_duration_s').value)
        self.maneuver_speed = float(
            self.get_parameter('maneuver_speed_ms').value)
        self.dive_speed = float(self.get_parameter('dive_speed_ms').value)
        self.surface_speed = float(
            self.get_parameter('surface_speed_ms').value)
        self.surface_pitch = math.radians(float(
            self.get_parameter('surface_pitch_deg').value))
        self.condition_hold = float(
            self.get_parameter('condition_hold_s').value)
        self.tracking_hold = float(
            self.get_parameter('tracking_hold_s').value)
        self.heading_tracking_tolerance = math.radians(float(
            self.get_parameter(
                'heading_tracking_tolerance_deg').value))
        self.pitch_tracking_tolerance = math.radians(float(
            self.get_parameter(
                'pitch_tracking_tolerance_deg').value))
        self.maneuver_step_timeout = float(
            self.get_parameter('maneuver_step_timeout_s').value)
        self.surface_cap_closed_hold = float(
            self.get_parameter('surface_cap_closed_hold_s').value)
        self.dive_timeout = float(
            self.get_parameter('dive_timeout_s').value)
        self.surface_timeout = float(
            self.get_parameter('surface_timeout_s').value)
        self.max_demo_duration = float(
            self.get_parameter('max_demo_duration_s').value)
        self.nav_status_timeout = float(
            self.get_parameter('nav_status_timeout_s').value)
        self.motion_permission_timeout = float(
            self.get_parameter('motion_permission_timeout_s').value)
        self.startup_grace_period = float(
            self.get_parameter('startup_grace_period_s').value)

        if self.minimum_depth < 1.0:
            raise ValueError(
                'Video gereksinimi nedeniyle minimum_depth_m >= 1.0 olmali.')
        if self.target_depth < self.minimum_depth:
            raise ValueError(
                'target_depth_m, minimum_depth_m degerinden kucuk olamaz.')
        self.sequence_duration = validate_maneuver_duration(
            self.minimum_maneuver_duration)

        self._phase = PHASE_REFERANSLAMA
        self._node_start_time = self.get_clock().now()
        self._phase_start_time = self.get_clock().now()
        self._maneuver_start_time = None
        self._maneuver_step_index = 0
        self._maneuver_step_active_start = None
        self._maneuver_completed_s = 0.0
        self._condition_start_time = None
        self._reference_heading = 0.0
        self._reference_captured = False
        self._failsafe_reason = '-'

        self._depth = 0.0
        self._heading = 0.0
        self._pitch = 0.0
        self._surface_detected = False
        self._minimum_depth_reached = False
        self._surface_cap_closed_verified = False
        self._pixhawk_connected = False
        self._depth_valid = False
        self._last_nav_status_time = None
        self._nav_status_level = DiagnosticStatus.STALE
        self._motion_permission = False
        self._last_motion_permission_time = None
        self._emergency_stop = False
        self._leak_detected = False
        self._target_depth = 0.0
        self._target_heading = 0.0
        self._target_pitch = 0.0
        self._forward_motion = False
        self._target_speed = 0.0
        self._nose_cap_open_request = False
        self._launch_request = False

        self.create_subscription(
            Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(
            Bool, '/sara/navigation/surface_detected',
            self._on_surface, 10)
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
        self._nose_cap_pub = self.create_publisher(
            Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(
            Bool, '/sara/guidance/launch_request', 10)
        self._task_complete_pub = self.create_publisher(
            Bool, '/sara/guidance/task_complete', 10)
        self._status_pub = self.create_publisher(
            DiagnosticStatus, '/sara/guidance/status', 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'VIDEO_DEMO gudumu baslatildi: '
            f'su alti manevrasi={self.sequence_duration:.1f} sn, '
            f'hedef derinlik={self.target_depth:.2f} m. '
            'Burun ayirma/3 V gosterimi karadaki bench testine ayrildi.')

    def _on_odom(self, msg):
        self._depth = float(msg.pose.pose.position.z)
        if self._depth >= self.minimum_depth:
            self._minimum_depth_reached = True
        q = msg.pose.pose.orientation
        self._heading, self._pitch, _roll = quaternion_to_yaw_pitch_roll(
            q.x, q.y, q.z, q.w)

    def _on_surface(self, msg):
        self._surface_detected = bool(msg.data)

    def _on_nav_status(self, msg):
        self._last_nav_status_time = self.get_clock().now()
        self._nav_status_level = msg.level
        values = {kv.key: kv.value for kv in msg.values}
        self._pixhawk_connected = (
            values.get('pixhawk_connected', 'False') == 'True')
        self._depth_valid = values.get('depth_valid', 'False') == 'True'

    def _on_motion_permission(self, msg):
        self._motion_permission = bool(msg.data)
        self._last_motion_permission_time = self.get_clock().now()

    def _on_emergency_stop(self, msg):
        self._emergency_stop = bool(msg.data)

    def _on_leak(self, msg):
        self._leak_detected = bool(msg.data)

    def _age_s(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _nav_ok(self):
        return (
            self._age_s(self._last_nav_status_time)
            <= self.nav_status_timeout
            and self._nav_status_level != DiagnosticStatus.ERROR
            and self._pixhawk_connected
            and self._depth_valid
        )

    def _motion_permission_fresh(self):
        return (
            self._age_s(self._last_motion_permission_time)
            <= self.motion_permission_timeout
        )

    def _elapsed_since(self, stamp):
        if stamp is None:
            return 0.0
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _phase_elapsed(self):
        return self._elapsed_since(self._phase_start_time)

    def _demo_elapsed(self):
        return self._elapsed_since(self._node_start_time)

    def _maneuver_elapsed(self):
        active_elapsed = 0.0
        if (
            self._maneuver_step_active_start is not None
            and self._maneuver_step_index < len(MANEUVER_STEPS)
        ):
            active_elapsed = min(
                self._elapsed_since(self._maneuver_step_active_start),
                MANEUVER_STEPS[self._maneuver_step_index].duration_s,
            )
        return self._maneuver_completed_s + active_elapsed

    def _condition_held(self, condition, hold_s=None):
        if not condition:
            self._condition_start_time = None
            return False
        if self._condition_start_time is None:
            self._condition_start_time = self.get_clock().now()
        required = self.condition_hold if hold_s is None else float(hold_s)
        return self._elapsed_since(self._condition_start_time) >= required

    def _goto_phase(self, phase):
        if phase == self._phase:
            return
        self.get_logger().info(
            f'VIDEO_DEMO fazi: {self._phase} -> {phase} '
            f'({self._phase_elapsed():.1f} sn)')
        self._phase = phase
        self._phase_start_time = self.get_clock().now()
        self._condition_start_time = None

    def _safe_terminate(self, reason):
        if self._phase in TERMINAL_PHASES:
            return
        self._failsafe_reason = reason
        self.get_logger().error(
            f'VIDEO_DEMO guvenli sonlandirildi: {reason}')
        self._goto_phase(PHASE_GUVENLI_SONLANDIRMA)

    def _reset_requests(self):
        self._target_depth = self._depth
        self._target_heading = (
            self._reference_heading
            if self._reference_captured else self._heading)
        self._target_pitch = 0.0
        self._forward_motion = False
        self._target_speed = 0.0
        self._nose_cap_open_request = False
        self._launch_request = False

    def _run_fsm(self):
        if self._phase in TERMINAL_PHASES:
            return

        if self._emergency_stop:
            self._safe_terminate('acil_durdurma')
            return
        if self._leak_detected:
            self._safe_terminate('su_kacagi')
            return
        if self._demo_elapsed() > self.max_demo_duration:
            self._safe_terminate('azami_demo_suresi_asildi')
            return

        startup_grace_done = self._demo_elapsed() > self.startup_grace_period
        mission_started = self._phase != PHASE_REFERANSLAMA
        if startup_grace_done and not self._nav_ok():
            self._safe_terminate('navigasyon_veya_pixhawk_gecersiz')
            return
        if (
            mission_started
            and (
                not self._motion_permission
                or not self._motion_permission_fresh()
            )
        ):
            self._safe_terminate('hareket_izni_kayip')
            return

        if self._phase == PHASE_REFERANSLAMA:
            if self._nav_ok() and not self._reference_captured:
                self._reference_heading = self._heading
                self._reference_captured = True
                self.get_logger().info(
                    'VIDEO_DEMO baslangic heading referansi alindi.')
            if (
                self._reference_captured
                and self._motion_permission
                and self._motion_permission_fresh()
            ):
                self._goto_phase(PHASE_DALIS)
            return

        if self._phase == PHASE_DALIS:
            self._target_depth = self.target_depth
            self._target_heading = self._reference_heading
            self._forward_motion = True
            self._target_speed = self.dive_speed
            depth_ready = (
                self._depth >= self.minimum_depth
                and abs(self._depth - self.target_depth)
                <= self.depth_tolerance
            )
            if self._condition_held(depth_ready):
                self._maneuver_start_time = self.get_clock().now()
                self._maneuver_step_index = 0
                self._maneuver_step_active_start = None
                self._maneuver_completed_s = 0.0
                self._goto_phase(MANEUVER_STEPS[0].phase)
            elif self._phase_elapsed() > self.dive_timeout:
                self._safe_terminate('1m_derinlige_ulasilamadi')
            return

        if self._phase in MANEUVER_PHASES:
            if self._maneuver_step_index >= len(MANEUVER_STEPS):
                if self._maneuver_elapsed() < self.minimum_maneuver_duration:
                    self._safe_terminate('manevra_suresi_yetersiz')
                else:
                    self._goto_phase(PHASE_YUZEYE_CIKIS)
                return

            step = MANEUVER_STEPS[self._maneuver_step_index]
            if step.phase != self._phase:
                self._goto_phase(step.phase)
            self._target_depth = self.target_depth
            self._target_heading = wrap_pi(
                self._reference_heading
                + math.radians(step.heading_offset_deg))
            self._target_pitch = math.radians(step.pitch_deg)
            self._forward_motion = True
            self._target_speed = self.maneuver_speed

            tracking_ok = all((
                self._depth >= self.minimum_depth,
                abs(self._depth - self.target_depth)
                <= self.depth_tolerance,
                abs(wrap_pi(self._target_heading - self._heading))
                <= self.heading_tracking_tolerance,
                abs(self._target_pitch - self._pitch)
                <= self.pitch_tracking_tolerance,
            ))
            if not tracking_ok:
                self._maneuver_step_active_start = None
                self._condition_held(False)
            elif self._maneuver_step_active_start is None:
                if self._condition_held(True, self.tracking_hold):
                    self._maneuver_step_active_start = (
                        self.get_clock().now())
            elif (
                self._elapsed_since(self._maneuver_step_active_start)
                >= step.duration_s
            ):
                self._maneuver_completed_s += step.duration_s
                self._maneuver_step_index += 1
                self._maneuver_step_active_start = None
                self._condition_start_time = None
                if self._maneuver_step_index >= len(MANEUVER_STEPS):
                    if (
                        self._maneuver_completed_s
                        >= self.minimum_maneuver_duration
                    ):
                        self._goto_phase(PHASE_YUZEYE_CIKIS)
                    else:
                        self._safe_terminate(
                            'manevra_suresi_yetersiz')
                else:
                    self._goto_phase(
                        MANEUVER_STEPS[
                            self._maneuver_step_index].phase)

            if (
                self._phase in MANEUVER_PHASES
                and self._phase_elapsed()
                > step.duration_s + self.maneuver_step_timeout
            ):
                self._safe_terminate(
                    f'{step.phase}_takip_zaman_asimi')
            return

        if self._phase == PHASE_YUZEYE_CIKIS:
            self._target_depth = 0.0
            self._target_heading = self._reference_heading
            self._target_pitch = self.surface_pitch
            self._forward_motion = True
            self._target_speed = self.surface_speed
            surface_ready = (
                self._surface_detected
                and self._depth <= self.depth_tolerance
            )
            if self._condition_held(surface_ready):
                self._goto_phase(PHASE_YUZEY_KANIT)
            elif self._phase_elapsed() > self.surface_timeout:
                self._safe_terminate('guvenli_yuzeye_cikis_zaman_asimi')
            return

        if self._phase == PHASE_YUZEY_KANIT:
            self._target_depth = 0.0
            self._target_heading = self._reference_heading
            # Bu fazda kapak kesinlikle kapali kalir.
            if not self._surface_detected:
                self._safe_terminate('yuzey_kaniti_sirasinda_yuzey_kaybi')
            elif self._phase_elapsed() >= self.surface_cap_closed_hold:
                self._surface_cap_closed_verified = True
                self._goto_phase(PHASE_TAMAMLANDI)
            return

    def _publish(self):
        now = self.get_clock().now()
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'sara_odom'
        pose.pose.position.z = float(self._target_depth)
        qx, qy, qz, qw = euler_to_quaternion(
            0.0, self._target_pitch, self._target_heading)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._target_pub.publish(pose)

        phase = String()
        phase.data = self._phase
        self._phase_pub.publish(phase)

        forward = Bool()
        forward.data = bool(self._forward_motion)
        self._forward_pub.publish(forward)

        speed = Float64()
        speed.data = (
            float(self._target_speed) if self._forward_motion else 0.0)
        self._speed_pub.publish(speed)

        cap = Bool()
        cap.data = bool(self._nose_cap_open_request)
        self._nose_cap_pub.publish(cap)

        launch = Bool()
        launch.data = bool(self._launch_request)
        self._launch_pub.publish(launch)

        complete = Bool()
        complete.data = self._phase in TERMINAL_PHASES
        self._task_complete_pub.publish(complete)

        maneuver_elapsed = min(
            self._maneuver_elapsed(), self.sequence_duration)
        status = DiagnosticStatus()
        status.name = 'sara_guidance'
        status.hardware_id = 'jetson_orin_nano_video_demo'
        status.level = (
            DiagnosticStatus.ERROR
            if self._phase == PHASE_GUVENLI_SONLANDIRMA
            else DiagnosticStatus.OK
        )
        status.message = f'VIDEO_DEMO - Faz: {self._phase}'
        status.values = [
            KeyValue(key='demo_mode', value='True'),
            KeyValue(key='phase', value=self._phase),
            KeyValue(
                key='maneuver_elapsed_s',
                value=f'{maneuver_elapsed:.1f}'),
            KeyValue(
                key='maneuver_required_s',
                value=f'{self.minimum_maneuver_duration:.1f}'),
            KeyValue(
                key='maneuver_requirement_met',
                value=str(
                    maneuver_elapsed >= self.minimum_maneuver_duration)),
            KeyValue(key='depth_m', value=f'{self._depth:.2f}'),
            KeyValue(
                key='minimum_depth_reached',
                value=str(self._minimum_depth_reached)),
            KeyValue(
                key='surface_detected',
                value=str(self._surface_detected)),
            KeyValue(
                key='nose_cap_open_request',
                value=str(self._nose_cap_open_request)),
            KeyValue(
                key='surface_cap_closed_verified',
                value=str(self._surface_cap_closed_verified)),
            KeyValue(key='nose_separation_enabled', value='False'),
            KeyValue(key='ignition_demo_enabled', value='False'),
            KeyValue(
                key='motion_permission',
                value=str(self._motion_permission)),
            KeyValue(key='failsafe_reason', value=self._failsafe_reason),
        ]
        self._status_pub.publish(status)

    def _on_timer(self):
        self._reset_requests()
        self._run_fsm()
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDemoGuidanceNode()
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
