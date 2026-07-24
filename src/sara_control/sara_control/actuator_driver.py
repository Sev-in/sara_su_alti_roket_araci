#!/usr/bin/env python3
"""Arduino Uno/PCA9685 donanim koprusu.

Bu node kontrol karari URETMEZ. Yalnizca safety_node'un onayladigi nihai
``*_command`` topic'lerini Uno seri protokolune cevirir. Boylece ekli
standalone IMU->servo kodundaki gibi autopilot ve guvenlik katmanlari
baypas edilmez.

Uno ayni USB baglantisindan SEN0257 telemetrisi de yollar. Gauge basinci
yuzey referansina eklenerek ``/sara/pressure`` topic'ine verilir; mevcut
navigation.py Hampel/median, zaman-duyarli LPF, aykiri deger reddi ve
guvenli P0 kalibrasyonunu uygulamaya devam eder.
"""

import importlib
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, Float64

from sara_control.uno_protocol import (
    gauge_kpa_to_absolute_pa,
    normalized_fin_to_servo,
    normalized_thrust,
    parse_telemetry_line,
)


class ActuatorDriverNode(Node):
    """Onayli ROS komutlarini guvenli Uno komutlarina donusturur."""

    def __init__(self):
        super().__init__('actuator_driver')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('uno_ready_timeout_s', 15.0)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('heartbeat_rate_hz', 5.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('reconnect_interval_s', 1.0)

        self.declare_parameter('servo_center_deg', 90.0)
        self.declare_parameter('pitch_max_offset_deg', 30.0)
        self.declare_parameter('yaw_max_offset_deg', 20.0)
        self.declare_parameter('pitch_sign', -1.0)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('nose_closed_deg', 0.0)
        self.declare_parameter('nose_open_deg', 90.0)
        self.declare_parameter('buoyancy_surface_threshold', -0.05)

        self.declare_parameter('surface_reference_pa', 101325.0)
        self.declare_parameter('pressure_variance_pa2', 0.0)
        self.declare_parameter('pressure_min_pa', 80000.0)
        self.declare_parameter('pressure_max_pa', 1700000.0)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('serial_baud').value)
        self.ready_timeout = float(
            self.get_parameter('uno_ready_timeout_s').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout_s').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_s').value)

        self.servo_center = float(
            self.get_parameter('servo_center_deg').value)
        self.pitch_max_offset = float(
            self.get_parameter('pitch_max_offset_deg').value)
        self.yaw_max_offset = float(
            self.get_parameter('yaw_max_offset_deg').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.nose_closed = float(
            self.get_parameter('nose_closed_deg').value)
        self.nose_open = float(
            self.get_parameter('nose_open_deg').value)
        self.surface_threshold = float(
            self.get_parameter('buoyancy_surface_threshold').value)

        self.surface_reference = float(
            self.get_parameter('surface_reference_pa').value)
        self.pressure_variance = float(
            self.get_parameter('pressure_variance_pa2').value)
        self.pressure_min = float(
            self.get_parameter('pressure_min_pa').value)
        self.pressure_max = float(
            self.get_parameter('pressure_max_pa').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pressure_pub = self.create_publisher(
            FluidPressure, '/sara/pressure', sensor_qos)
        self._status_pub = self.create_publisher(
            DiagnosticStatus, '/sara/actuators/status', 10)

        self.create_subscription(
            Float64, '/sara/control/thrust_command',
            self._on_thrust, 10)
        self.create_subscription(
            Vector3, '/sara/control/fin_command',
            self._on_fin, 10)
        self.create_subscription(
            Float64, '/sara/control/buoyancy_command',
            self._on_buoyancy, 10)
        self.create_subscription(
            Bool, '/sara/control/nose_cap_command',
            self._on_nose, 10)
        self.create_subscription(
            Bool, '/sara/control/launch_command',
            self._on_launch, 10)
        self.create_subscription(
            Bool, '/sara/mission_start/motion_permission',
            self._on_motion_permission, 10)
        self.create_subscription(
            Bool, '/sara/safety/emergency_stop',
            self._on_emergency_stop, 10)
        self.create_subscription(
            Bool, '/sara/safety/main_power_cutoff_command',
            self._on_power_cutoff, 10)

        self._serial_module = None
        self._serial = None
        self._serial_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._uno_ready = False
        self._uno_ready_deadline = math.inf
        self._ready_timeout_reported = False
        self._last_connect_attempt = 0.0
        self._last_rx_time = None
        self._last_pressure_time = None
        self._last_error = '-'
        self._error_count = 0
        self._connection_count = 0
        self._motor_state = 'DISCONNECTED'
        self._arm_requested = False
        self._motor_armed = False

        self._thrust = 0.0
        self._fin = Vector3()
        self._buoyancy = 0.0
        self._nose_open_request = False
        self._launch_request = False
        self._motion_permission = False
        self._emergency_stop = False
        self._power_cutoff = False
        self._last_control_time = None
        self._surface_running = False
        self._ignition_commanded = False
        self._ignition_confirmed = False

        self._connect()

        command_rate = max(
            1.0, float(self.get_parameter('command_rate_hz').value))
        heartbeat_rate = max(
            1.0, float(self.get_parameter('heartbeat_rate_hz').value))
        self.create_timer(1.0 / command_rate, self._command_tick)
        self.create_timer(1.0 / heartbeat_rate, self._heartbeat_tick)
        self.create_timer(0.02, self._read_serial)
        self.create_timer(self.reconnect_interval, self._reconnect_tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'actuator_driver baslatildi: Uno={self.port}@{self.baud}. '
            'Yalnizca safety_node *_command ciktilari uygulanir.')

    def _connect(self):
        self._last_connect_attempt = time.monotonic()
        try:
            if self._serial_module is None:
                self._serial_module = importlib.import_module('serial')
            connection = self._serial_module.Serial(
                self.port, self.baud, timeout=0.0)
            with self._serial_lock:
                self._serial = connection
                self._rx_buffer.clear()
            # Uno, seri port acilinca DTR nedeniyle yeniden baslayabilir.
            # READY gelene kadar heartbeat/eyleyici komutu gonderilmez.
            self._uno_ready = False
            self._uno_ready_deadline = (
                time.monotonic() + max(1.0, self.ready_timeout))
            self._ready_timeout_reported = False
            self._motor_state = 'WAITING_FOR_READY'
            self._last_error = '-'
            self._connection_count += 1
            self.get_logger().info(
                f'Uno seri baglantisi acildi: {self.port}@{self.baud} '
                f'(baglanti #{self._connection_count})')
        except Exception as exc:
            self._disconnect(str(exc))

    def _disconnect(self, reason):
        with self._serial_lock:
            connection = self._serial
            self._serial = None
            self._rx_buffer.clear()
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if connection is not None and reason != 'node kapatildi':
            self.get_logger().error(
                f'Uno seri baglantisi kesildi: {reason}')
        self._motor_armed = False
        self._arm_requested = False
        self._ignition_commanded = False
        self._ignition_confirmed = False
        self._uno_ready = False
        self._uno_ready_deadline = math.inf
        self._ready_timeout_reported = False
        self._motor_state = 'DISCONNECTED'
        self._last_error = str(reason)
        self._error_count += 1

    def _serial_ready(self):
        return (
            self._serial is not None
            and getattr(self._serial, 'is_open', False)
            and self._uno_ready
        )

    def _write(self, command):
        if not self._serial_ready():
            return False
        try:
            payload = (command + '\n').encode('ascii')
            with self._serial_lock:
                if self._serial is None:
                    return False
                self._serial.write(payload)
            return True
        except Exception as exc:
            self._disconnect(f'seri yazma: {exc}')
            return False

    def _on_thrust(self, msg):
        self._thrust = normalized_thrust(msg.data)
        self._touch_control()

    def _on_fin(self, msg):
        self._fin = msg
        self._touch_control()

    def _on_buoyancy(self, msg):
        self._buoyancy = float(msg.data)
        self._touch_control()

    def _on_nose(self, msg):
        self._nose_open_request = bool(msg.data)
        self._touch_control()

    def _on_launch(self, msg):
        self._launch_request = bool(msg.data)
        self._touch_control()

    def _on_motion_permission(self, msg):
        self._motion_permission = bool(msg.data)
        if not self._motion_permission:
            self._arm_requested = False

    def _on_emergency_stop(self, msg):
        self._emergency_stop = bool(msg.data)

    def _on_power_cutoff(self, msg):
        self._power_cutoff = bool(msg.data)

    def _touch_control(self):
        self._last_control_time = self.get_clock().now()

    def _controls_fresh(self):
        if self._last_control_time is None:
            return False
        age = (
            self.get_clock().now() - self._last_control_time
        ).nanoseconds * 1e-9
        return age <= self.command_timeout

    def _heartbeat_tick(self):
        self._write('HB')

    def _command_tick(self):
        if not self._serial_ready():
            return

        blocked = (
            self._emergency_stop
            or self._power_cutoff
            or not self._controls_fresh()
        )
        if blocked:
            self._write('ALL_STOP')
            self._surface_running = False
            self._ignition_commanded = False
            self._ignition_confirmed = False
            self._motor_armed = False
            self._arm_requested = False
            return

        # ESC, hareket izni gelmeden once NOTR komutta arm edilir. Arming
        # hareket degildir; safety thrust_command=0 tuttugu icin 60 sn
        # inhibit korunur. Boylece izin aninda 3 sn ESC beklemesi Gorev 2
        # yarismasinin 60 sn sayacindan yenmez.
        if not self._motor_armed:
            if not self._arm_requested:
                self._write('MOTOR_ARM')
                self._arm_requested = True

        thrust = self._thrust if (
            self._motion_permission and self._motor_armed) else 0.0
        self._write(f'THRUST={thrust:.3f}')

        pitch_deg = normalized_fin_to_servo(
            self._fin.x,
            self.servo_center,
            self.pitch_max_offset,
            self.pitch_sign,
        )
        yaw_deg = normalized_fin_to_servo(
            self._fin.y,
            self.servo_center,
            self.yaw_max_offset,
            self.yaw_sign,
        )
        self._write(f'PITCH={pitch_deg:.1f}')
        self._write(f'YAW={yaw_deg:.1f}')

        nose_deg = (
            self.nose_open if self._nose_open_request
            else self.nose_closed
        )
        self._write(f'NOSE={nose_deg:.1f}')

        surface_requested = self._buoyancy < self.surface_threshold
        if surface_requested != self._surface_running:
            self._write(
                'SURFACE_START' if surface_requested else 'SURFACE_STOP')
            self._surface_running = surface_requested

        ignition_requested = (
            self._launch_request
            and self._motion_permission
            and not self._emergency_stop
        )
        if ignition_requested != self._ignition_commanded:
            self._write(
                'IGNITION=1' if ignition_requested else 'IGNITION=0')
            self._ignition_commanded = ignition_requested

    def _read_serial(self):
        if self._serial is None:
            return
        try:
            with self._serial_lock:
                if self._serial is None:
                    return
                waiting = self._serial.in_waiting
                if waiting <= 0:
                    return
                self._rx_buffer.extend(self._serial.read(waiting))
        except Exception as exc:
            self._disconnect(f'seri okuma: {exc}')
            return

        # timeout=0 olan seri portta readline(), satirin tamami gelmeden
        # parcali veri dondurebilir. Yalnizca '\\n' ile tamamlanmis satirlar
        # ayrisir; eksik parca bir sonraki timer turuna saklanir.
        while b'\n' in self._rx_buffer:
            raw, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            line = raw.rstrip(b'\r').decode(
                'utf-8', errors='replace').strip()
            if not line:
                continue
            self._last_rx_time = self.get_clock().now()
            try:
                self._handle_serial_line(line)
            except Exception as exc:
                # Bozuk tek bir telemetri satiri USB kopmasi degildir.
                # Portu kapatmak DTR ile Uno'yu resetleyecegi icin satir
                # reddedilir, baglanti acik tutulur.
                self._last_error = f'Bozuk Uno telemetrisi: {exc}'
                self._error_count += 1
                self.get_logger().warning(self._last_error)

        if len(self._rx_buffer) > 4096:
            self._rx_buffer.clear()
            self._last_error = 'Uno seri satiri 4096 bayti asti'
            self._error_count += 1
            self.get_logger().warning(self._last_error)

    def _handle_serial_line(self, line):
        if line == 'INFO,CALIBRATION_START':
            self._uno_ready = False
            self._motor_state = 'CALIBRATING'
            return
        if line == 'INFO,CALIBRATION_DONE':
            self._motor_state = 'WAITING_FOR_READY'
            return
        if line == 'READY,SARA_UNO_CONTROLLER_V2':
            self._uno_ready = True
            self._ready_timeout_reported = False
            self._motor_armed = False
            self._arm_requested = False
            self._surface_running = False
            self._ignition_commanded = False
            self._ignition_confirmed = False
            self._motor_state = 'READY'
            self._last_error = '-'
            self.get_logger().info(
                'Uno kalibrasyonu tamamlandi; komut iletimi etkin.')
            return
        if line == 'OK,MOTOR_ARMED':
            self._motor_armed = True
            self._arm_requested = False
            self._motor_state = 'ARMED'
            return
        if line == 'OK,IGNITION_ACTIVE':
            # Bu yalnizca komutun kabul edildigini bildirir. Gercek aktif
            # durumu, Uno telemetrisindeki ignition_active alanindan gelir.
            return
        if line == 'OK,IGNITION_OFF':
            self._ignition_confirmed = False
            return
        if line.startswith('ERR,'):
            self._last_error = line
            self._error_count += 1
            if line in ('ERR,HEARTBEAT_LOST', 'ERR,MOTOR_NOT_ARMED'):
                self._motor_armed = False
                self._arm_requested = False
            if 'IGNITION' in line or 'HEARTBEAT' in line:
                self._ignition_confirmed = False
            self.get_logger().warning(line)
            return
        if line.startswith('READY,'):
            self._last_error = f'Desteklenmeyen Uno firmware surumu: {line}'
            self._error_count += 1
            self.get_logger().error(self._last_error)
            return

        values = parse_telemetry_line(line)
        if not values:
            return
        if 'motor_state' in values:
            try:
                self._motor_state = str(int(values['motor_state']))
            except (TypeError, ValueError, OverflowError):
                pass
        if 'ignition_active' in values:
            try:
                self._ignition_confirmed = (
                    int(values['ignition_active']) == 1)
            except (TypeError, ValueError, OverflowError):
                self._ignition_confirmed = False
        if 'pressure_kpa' not in values:
            return
        try:
            pressure_pa = gauge_kpa_to_absolute_pa(
                values['pressure_kpa'], self.surface_reference)
            if (
                not math.isfinite(pressure_pa)
                or pressure_pa < self.pressure_min
                or pressure_pa > self.pressure_max
            ):
                raise ValueError(
                    f'Uno basinci aralik disi: {pressure_pa:.1f} Pa')

            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sara_pressure_sensor_uno'
            msg.fluid_pressure = pressure_pa
            msg.variance = max(0.0, self.pressure_variance)
            self._pressure_pub.publish(msg)
            self._last_pressure_time = self.get_clock().now()
            self._last_error = '-'
        except Exception as exc:
            self._last_error = str(exc)
            self._error_count += 1

    def _reconnect_tick(self):
        if self._serial is not None:
            if (
                not self._uno_ready
                and not self._ready_timeout_reported
                and time.monotonic() >= self._uno_ready_deadline
            ):
                self._ready_timeout_reported = True
                self._last_error = 'Uno READY mesaji zaman asimina ugradi'
                self._error_count += 1
                self.get_logger().error(
                    'Uno READY mesaji gelmedi; guvenlik icin komut '
                    'gonderilmiyor. Seri port acik birakildi.')
            return
        if time.monotonic() - self._last_connect_attempt >= self.reconnect_interval:
            self._connect()

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (
            self.get_clock().now() - stamp
        ).nanoseconds * 1e-9

    def _publish_status(self):
        serial_ok = self._serial_ready()
        pressure_fresh = self._age(self._last_pressure_time) < 1.0
        status = DiagnosticStatus()
        status.name = 'sara_actuator_driver'
        status.hardware_id = 'arduino_uno_pca9685'
        status.level = (
            DiagnosticStatus.OK
            if serial_ok and pressure_fresh
            else DiagnosticStatus.ERROR
        )
        status.message = (
            'Nominal' if status.level == DiagnosticStatus.OK
            else 'Uno seri baglanti/basinc telemetrisi hazir degil'
        )
        status.values = [
            KeyValue(key='serial_ready', value=str(serial_ok)),
            KeyValue(
                key='serial_rx_age_s',
                value=(
                    '-' if self._last_rx_time is None
                    else f'{self._age(self._last_rx_time):.2f}')),
            KeyValue(
                key='pressure_age_s',
                value=(
                    '-' if self._last_pressure_time is None
                    else f'{self._age(self._last_pressure_time):.2f}')),
            KeyValue(key='motor_state', value=self._motor_state),
            KeyValue(key='motor_armed', value=str(self._motor_armed)),
            KeyValue(
                key='surface_running',
                value=str(self._surface_running)),
            KeyValue(
                key='ignition_active',
                value=str(self._ignition_confirmed)),
            KeyValue(
                key='ignition_commanded',
                value=str(self._ignition_commanded)),
            KeyValue(key='error_count', value=str(self._error_count)),
            KeyValue(
                key='connection_count',
                value=str(self._connection_count)),
            KeyValue(key='last_error', value=self._last_error),
        ]
        self._status_pub.publish(status)

    def destroy_node(self):
        if self._serial_ready():
            self._write('IGNITION=0')
            self._write('SURFACE_STOP')
            self._write('ALL_STOP')
        self._disconnect('node kapatildi')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriverNode()
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
