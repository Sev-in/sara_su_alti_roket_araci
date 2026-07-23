#!/usr/bin/env python3
"""
navigation.py
==============
SARA platformu - Navigasyon Katmani (sara_control paketi icinde)
Ontasarim Raporu Tablo 13 uygulamasi - SADECE STANDART ROS2 MESAJLARI

Aktif girdiler:
    Pixhawk 6X IMU       (/sara/imu/data)
    SEN0257 basinc       (/sara/pressure)
    SEN0368 su var/yok x2 (/sara/water_detect_1, _2)

Gercek donanim modunda bu topic'leri sensor_get_data.py, test modunda
vehicle_sim.py yayinlar. Bir sensor eksik veya gecersizse dugum cokmez;
status/depth_valid alanlari fail-safe olacak sekilde gecersiz isaretlenir.

Veri akisi (Tablo 13 semasi):
    Veri Toplama & Kalibrasyon
        -> Filtreleme & Sensor Fuzyonu (hareketli ortalama, LPF)
        -> Derinlik Kestirimi h=(P-P0)/(rho*g)
           Yonelim Kestirimi (roll, pitch, yaw/heading, acisal hiz)
        -> Yaklasik Ilerleme / Hareket Tutarliligi (MUTLAK KONUM DEGIL,
           GPS/DVL yok - sadece gorev fazi/guvenli atis bolgesi karari
           icin yardimci kestirim)
        -> Durum Bilgisi Uretimi -> nav_msgs/Odometry + Bool + DiagnosticStatus

Cikti topic'leri:
    /sara/navigation/odom     (nav_msgs/Odometry)
        pose.position.z    = derinlik [m] (pozitif = su altina dogru)
        pose.position.x/y  = yaklasik ilerleme x_yaklasik / y_yaklasik [m]
        pose.orientation   = quaternion (roll, pitch, heading/yaw)
        twist.linear.x     = itki tabanli yaklasik ileri hiz
        twist.linear.z     = basinc gecmisinden kestirilen dikey hiz
        twist.angular      = filtrelenmis acisal hizlar (roll/pitch/yaw rate)
    /sara/navigation/surface_detected  (std_msgs/Bool)
    /sara/navigation/status            (diagnostic_msgs/DiagnosticStatus)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Empty, Float64
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from sara_control.sensor_processing import (
    DebouncedBoolean,
    HampelFilter,
    SlidingLinearRegression,
    TimeAwareLowPass,
    normalize_quaternion,
    pressure_to_depth,
    robust_median,
)


def wrap_0_2pi(angle: float) -> float:
    """Heading'i [0, 2*pi) araligina sarar (0/360 gecis hatalarini onler)."""
    return angle % (2.0 * math.pi)


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """(roll, pitch, yaw) [rad] -> quaternion (x, y, z, w), aerospace ZYX sirasi."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """Quaternion -> (roll, pitch, yaw) [rad], aerospace (ZYX) sirasi."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MovingAverage:
    """Sabit pencereli hareketli ortalama filtresi."""

    def __init__(self, window: int):
        self._buf = deque(maxlen=max(1, window))

    def update(self, x: float) -> float:
        self._buf.append(x)
        return sum(self._buf) / len(self._buf)

    def reset(self):
        self._buf.clear()


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # ---------------- Parametreler ----------------
        self.declare_parameter('rho', 1000.0)                    # su yogunlugu [kg/m^3]
        self.declare_parameter('g', 9.80665)
        self.declare_parameter('pressure_calib_samples', 50)
        self.declare_parameter('lpf_alpha_pressure', 0.2)
        self.declare_parameter('pressure_min_pa', -100000.0)
        self.declare_parameter('pressure_max_pa', 1650000.0)
        self.declare_parameter('pressure_hampel_window', 7)
        self.declare_parameter('pressure_hampel_sigma', 3.0)
        self.declare_parameter('pressure_hampel_min_deviation_pa', 2000.0)
        self.declare_parameter('pressure_max_rate_pa_s', 50000.0)
        self.declare_parameter('pressure_jump_floor_pa', 2000.0)
        self.declare_parameter('pressure_calibration_stable_s', 1.0)
        self.declare_parameter('pressure_calibration_max_span_pa', 1000.0)
        self.declare_parameter('pressure_calibration_min_valid_ratio', 0.8)
        self.declare_parameter(
            'require_water_dry_for_calibration', True)
        self.declare_parameter(
            'require_imu_still_for_calibration', True)
        self.declare_parameter('depth_rate_window_s', 0.5)
        self.declare_parameter('depth_rate_lpf_alpha', 0.3)
        self.declare_parameter('max_vertical_speed_ms', 2.0)
        self.declare_parameter('imu_gyro_window', 5)
        self.declare_parameter('max_calibrated_speed_ms', 1.076)  # DUZELTME: KTR'deki ana seyir hizi
                                                                       # (tam itkide, thrust=1.0). Yaklasik
                                                                       # ilerleme artik SABIT v_kalibre yerine
                                                                       # GERCEK itki seviyesiyle ORANTILI
                                                                       # hesaplaniyor - fazlar arasi buyuk hiz
                                                                       # farklarini (0.895 vs 1.076 m/s) dogru
                                                                       # yansitmasi icin.
        self.declare_parameter('v_kalibre', 0.9)                  # [m/s] - ARTIK SADECE thrust_command hic
                                                                       # gelmediyse (eski/uyumluluk) yedek deger
        self.declare_parameter('enable_xy_estimate', True)
        self.declare_parameter('pixhawk_timeout_sec', 1.0)
        self.declare_parameter('pressure_timeout_sec', 2.0)
        self.declare_parameter('water_sensor_timeout_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('water_fusion_mode', 'or')         # 'or' -> temkinli
        self.declare_parameter('water_debounce_s', 0.3)
        self.declare_parameter('accel_still_threshold', 0.6)
        self.declare_parameter('gyro_still_threshold', 0.15)
        self.declare_parameter('gyro_consistent_threshold', 2.0)
        self.declare_parameter('imu_min_quaternion_norm', 0.5)
        self.declare_parameter('imu_max_quaternion_norm', 1.5)
        self.declare_parameter('imu_max_accel_ms2', 100.0)
        self.declare_parameter('imu_max_gyro_rads', 10.0)

        self.declare_parameter('imu_topic', '/sara/imu/data')     # DUZELTME: artik sensor_get_data.py'nin
                                                                       # ciktisini dinliyoruz (Sensor Veri Alma
                                                                       # Katmani), mavros'a DOGRUDAN DEGIL - katmanli
                                                                       # mimariye uygunluk icin (Tablo 11).
                                                                       # Test modunda vehicle_sim.py de ayni
                                                                       # topic'e yayin yapar.
        self.declare_parameter('thrust_active_threshold', 0.05)  # bu esigin altinda itki "yok" sayilir

        self.rho = self.get_parameter('rho').value
        self.g = self.get_parameter('g').value
        self.calib_samples_needed = int(self.get_parameter('pressure_calib_samples').value)
        self.pressure_min = self.get_parameter('pressure_min_pa').value
        self.pressure_max = self.get_parameter('pressure_max_pa').value
        self.pressure_max_rate = self.get_parameter(
            'pressure_max_rate_pa_s').value
        self.pressure_jump_floor = self.get_parameter(
            'pressure_jump_floor_pa').value
        self.calibration_stable_s = self.get_parameter(
            'pressure_calibration_stable_s').value
        self.calibration_max_span = self.get_parameter(
            'pressure_calibration_max_span_pa').value
        self.calibration_min_valid_ratio = self.get_parameter(
            'pressure_calibration_min_valid_ratio').value
        self.require_water_dry = self.get_parameter(
            'require_water_dry_for_calibration').value
        self.require_imu_still = self.get_parameter(
            'require_imu_still_for_calibration').value
        self.max_vertical_speed = self.get_parameter(
            'max_vertical_speed_ms').value
        self.v_kalibre = self.get_parameter('v_kalibre').value
        self.max_calibrated_speed = self.get_parameter('max_calibrated_speed_ms').value
        self.enable_xy = self.get_parameter('enable_xy_estimate').value
        self.pixhawk_timeout = self.get_parameter('pixhawk_timeout_sec').value
        self.pressure_timeout = self.get_parameter('pressure_timeout_sec').value
        self.water_timeout = self.get_parameter('water_sensor_timeout_sec').value
        self.water_fusion_mode = self.get_parameter('water_fusion_mode').value
        self.accel_still_th = self.get_parameter('accel_still_threshold').value
        self.gyro_still_th = self.get_parameter(
            'gyro_still_threshold').value
        self.gyro_consistent_th = self.get_parameter('gyro_consistent_threshold').value
        self.imu_min_q_norm = self.get_parameter(
            'imu_min_quaternion_norm').value
        self.imu_max_q_norm = self.get_parameter(
            'imu_max_quaternion_norm').value
        self.imu_max_accel = self.get_parameter(
            'imu_max_accel_ms2').value
        self.imu_max_gyro = self.get_parameter(
            'imu_max_gyro_rads').value
        self.thrust_active_threshold = self.get_parameter('thrust_active_threshold').value
        self.imu_topic = self.get_parameter('imu_topic').value
        pub_rate = float(self.get_parameter('publish_rate_hz').value)

        # ---------------- Filtreler ----------------
        nominal_dt = 1.0 / pub_rate
        self._pressure_lpf = TimeAwareLowPass(
            self.get_parameter('lpf_alpha_pressure').value,
            nominal_dt,
        )
        self._pressure_hampel = HampelFilter(
            int(self.get_parameter('pressure_hampel_window').value),
            self.get_parameter('pressure_hampel_sigma').value,
            self.get_parameter(
                'pressure_hampel_min_deviation_pa').value,
        )
        self._depth_rate_estimator = SlidingLinearRegression(
            self.get_parameter('depth_rate_window_s').value,
            max_samples=max(10, int(pub_rate * 2.0)),
        )
        self._depth_rate_lpf = TimeAwareLowPass(
            self.get_parameter('depth_rate_lpf_alpha').value,
            nominal_dt,
        )
        gyro_win = int(self.get_parameter('imu_gyro_window').value)
        self._gyro_x_avg = MovingAverage(gyro_win)
        self._gyro_y_avg = MovingAverage(gyro_win)
        self._gyro_z_avg = MovingAverage(gyro_win)

        # ---------------- Kalibrasyon ----------------
        self._p0 = None
        self._calib_buffer = deque(maxlen=max(3, self.calib_samples_needed))
        self._calibrated = False
        self._calibration_gate_since = None
        self._calibration_state = 'sensor kosullari bekleniyor'

        # ---------------- Anlik durum ----------------
        self._depth = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_rate = 0.0
        self._motion_consistent = False
        self._imu_still = False
        self._thrust_active = False
        self._last_thrust_value = 0.0
        self._imu_rejected_count = 0

        self._water_1 = None
        self._water_2 = None
        self._surface_detected = False
        water_debounce = self.get_parameter('water_debounce_s').value
        self._water_1_debounce = DebouncedBoolean(water_debounce)
        self._water_2_debounce = DebouncedBoolean(water_debounce)

        self._last_imu_rx_stamp = None
        self._last_imu_valid_stamp = None
        self._last_pressure_rx_stamp = None
        self._last_pressure_valid_stamp = None
        self._last_pressure_filter_stamp = None
        self._last_water_1_stamp = None
        self._last_water_2_stamp = None

        self._last_accepted_pressure = None
        self._filtered_pressure = None
        self._pressure_variance_pa2 = 0.0
        self._pressure_rejected_count = 0
        self._last_pressure_rejection = '-'
        self._depth_rate = 0.0
        self._depth_rate_valid = False
        self._orientation_covariance = [0.0] * 9

        self._approx_distance = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0
        self._last_tick_time = self.get_clock().now()

        # ---------------- QoS ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Abonelikler (Girdi) ----------------
        
        self.create_subscription(Imu, self.imu_topic, self._on_imu, sensor_qos)


        # Sensor yayini kesilirse node calismaya devam eder; status ve
        # depth_valid alanlari fail-safe olarak gecersiz olur.
        self.create_subscription(FluidPressure, '/sara/pressure', self._on_pressure, sensor_qos)
        self.create_subscription(Bool, '/sara/water_detect_1', self._on_water_1, sensor_qos)
        self.create_subscription(Bool, '/sara/water_detect_2', self._on_water_2, sensor_qos)

        # Gorev yonetiminden "referanslama / kalibrasyonu yenile" komutu (Tablo 12)
        self.create_subscription(Empty, '/sara/navigation/recalibrate', self._on_recalibrate, 10)

        # DUZELTME: ivmeolcer, SABIT HIZDA hareketi DURMA halinden AYIRT EDEMEZ
        # (ikisinde de net ivme ~0). Bu yuzden "hareket tutarliligi" tek basina
        # yaklasik ilerleme biriktirmek icin YETERLI DEGIL - guvenlik onayli
        # GERCEK itki komutunu da (thrust_command>0) kontrol ediyoruz.
        self.create_subscription(Float64, '/sara/control/thrust_command', self._on_thrust_command, sensor_qos)

        # ---------------- Yayinlar (Cikti) ----------------
        self._odom_pub = self.create_publisher(Odometry, '/sara/navigation/odom', 10)
        self._surface_pub = self.create_publisher(Bool, '/sara/navigation/surface_detected', 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/navigation/status', 10)

        # ---------------- Zamanlayici ----------------
        self.create_timer(1.0 / pub_rate, self._on_timer)

        self.get_logger().info(
            f'navigation_node baslatildi. IMU aktif ({self.imu_topic}). '
            f'Basinc kalibrasyonu icin {self.calib_samples_needed} ornek bekleniyor '
            '(basinc sensoru baglanmadiysa depth_valid=False kalacaktir).'
        )

        # Abonelik/yayin listesini acikca logla - "Subscribers bos gorunuyor"
        # gibi build/cache kaynakli sorunlari hizli teshis etmek icin.
        subs = [
            f'{self.imu_topic} [sensor_msgs/Imu]',
            '/sara/pressure [sensor_msgs/FluidPressure]',
            '/sara/water_detect_1 [std_msgs/Bool]',
            '/sara/water_detect_2 [std_msgs/Bool]',
            '/sara/navigation/recalibrate [std_msgs/Empty]',
        ]
        pubs = [
            '/sara/navigation/odom [nav_msgs/Odometry]',
            '/sara/navigation/surface_detected [std_msgs/Bool]',
            '/sara/navigation/status [diagnostic_msgs/DiagnosticStatus]',
        ]
        self.get_logger().info('Abone olunan topicler: ' + ', '.join(subs))
        self.get_logger().info('Yayinlanan topicler: ' + ', '.join(pubs))

    # ======================================================================
    # Kalibrasyon
    # ======================================================================
    def _on_recalibrate(self, _msg: Empty):
        self.get_logger().warn(
            'Yeniden kalibrasyon istegi alindi; guvenli yuzey kosullari '
            'yeniden beklenecek.')
        self._p0 = None
        self._calib_buffer.clear()
        self._calibrated = False
        self._calibration_gate_since = None
        self._calibration_state = 'sensor kosullari bekleniyor'
        self._pressure_lpf.reset()
        self._pressure_hampel.reset()
        self._depth_rate_estimator.reset()
        self._depth_rate_lpf.reset()
        self._last_accepted_pressure = None
        self._last_pressure_filter_stamp = None
        self._filtered_pressure = None
        self._depth = 0.0
        self._depth_rate = 0.0
        self._depth_rate_valid = False
        self._approx_distance = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0

    def _on_thrust_command(self, msg: Float64):
        self._thrust_active = abs(msg.data) > self.thrust_active_threshold
        self._last_thrust_value = msg.data

    # ======================================================================
    # Sensor callback'leri
    # ======================================================================
    def _on_imu(self, msg: Imu):
        now = self.get_clock().now()
        self._last_imu_rx_stamp = now

        q = msg.orientation
        normalized_q = normalize_quaternion(
            q.x,
            q.y,
            q.z,
            q.w,
            self.imu_min_q_norm,
            self.imu_max_q_norm,
        )
        gyro = msg.angular_velocity
        accel = msg.linear_acceleration
        vector_values = (
            gyro.x,
            gyro.y,
            gyro.z,
            accel.x,
            accel.y,
            accel.z,
        )
        vectors_finite = all(
            math.isfinite(value) for value in vector_values)
        gyro_in_range = vectors_finite and max(
            abs(gyro.x), abs(gyro.y), abs(gyro.z)) <= self.imu_max_gyro
        accel_norm = (
            math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2)
            if vectors_finite else float('inf')
        )
        accel_in_range = accel_norm <= self.imu_max_accel
        orientation_available = not (
            len(msg.orientation_covariance) >= 1
            and msg.orientation_covariance[0] < 0.0
        )

        if (
            normalized_q is None
            or not gyro_in_range
            or not accel_in_range
            or not orientation_available
        ):
            self._imu_rejected_count += 1
            self._imu_still = False
            self._motion_consistent = False
            return

        self._last_imu_valid_stamp = now
        qx, qy, qz, qw = normalized_q
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self._roll = roll
        self._pitch = pitch
        self._heading = wrap_0_2pi(yaw)

        self._roll_rate = self._gyro_x_avg.update(gyro.x)
        self._pitch_rate = self._gyro_y_avg.update(gyro.y)
        self._yaw_rate = self._gyro_z_avg.update(gyro.z)

        accel_dev = abs(accel_norm - self.g)
        gyro_mag = math.sqrt(
            self._roll_rate ** 2
            + self._pitch_rate ** 2
            + self._yaw_rate ** 2
        )

        self._imu_still = (
            accel_dev < self.accel_still_th
            and gyro_mag < self.gyro_still_th
            and not self._thrust_active
        )
        self._motion_consistent = (
            accel_dev < self.accel_still_th * 3.0
            and gyro_mag < self.gyro_consistent_th
        )
        if len(msg.orientation_covariance) == 9:
            self._orientation_covariance = list(
                msg.orientation_covariance)

    def _on_pressure(self, msg: FluidPressure):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        self._last_pressure_rx_stamp = now
        pressure = float(msg.fluid_pressure)

        rejection = self._validate_pressure(pressure, now)
        if rejection is not None:
            self._reject_pressure(rejection)
            return

        variance = float(msg.variance)
        self._pressure_variance_pa2 = (
            variance
            if math.isfinite(variance) and variance >= 0.0
            else 0.0
        )
        if not self._calibrated:
            self._last_pressure_valid_stamp = now
            if not self._calibration_gate_ready(now):
                return

        robust_pressure, is_outlier = self._pressure_hampel.update(pressure)
        if is_outlier:
            self._reject_pressure('Hampel aykiri degeri')
            return

        self._last_pressure_valid_stamp = now
        self._last_accepted_pressure = robust_pressure

        if not self._calibrated:
            self._calib_buffer.append(robust_pressure)
            if len(self._calib_buffer) >= self.calib_samples_needed:
                self._try_finish_pressure_calibration()
            return

        filter_dt = None
        if self._last_pressure_filter_stamp is not None:
            filter_dt = (
                now - self._last_pressure_filter_stamp
            ).nanoseconds * 1e-9
        self._last_pressure_filter_stamp = now
        filtered = self._pressure_lpf.update(
            robust_pressure,
            filter_dt,
        )
        self._filtered_pressure = filtered
        self._depth = pressure_to_depth(
            filtered,
            self._p0,
            self.rho,
            self.g,
        )

        raw_depth_rate = self._depth_rate_estimator.update(
            now_s,
            self._depth,
        )
        if (
            raw_depth_rate is not None
            and abs(raw_depth_rate) <= self.max_vertical_speed
        ):
            self._depth_rate = self._depth_rate_lpf.update(
                raw_depth_rate,
                filter_dt,
            )
            self._depth_rate_valid = True
        elif raw_depth_rate is not None:
            self._depth_rate_valid = False

    def _on_water_1(self, msg: Bool):
        now = self.get_clock().now()
        self._last_water_1_stamp = now
        stable = self._water_1_debounce.update(
            msg.data,
            now.nanoseconds * 1e-9,
        )
        if stable is not None:
            self._water_1 = stable
        self._fuse_water_sensors()

    def _on_water_2(self, msg: Bool):
        now = self.get_clock().now()
        self._last_water_2_stamp = now
        stable = self._water_2_debounce.update(
            msg.data,
            now.nanoseconds * 1e-9,
        )
        if stable is not None:
            self._water_2 = stable
        self._fuse_water_sensors()

    def _fuse_water_sensors(self):
        if self._water_1 is None or self._water_2 is None:
            return
        if self.water_fusion_mode == 'and':
            water_present = self._water_1 and self._water_2
        else:  # 'or' (varsayilan, guvenli taraf)
            water_present = self._water_1 or self._water_2
        self._surface_detected = not water_present

    def _validate_pressure(self, pressure: float, now):
        if not math.isfinite(pressure):
            return 'NaN/Inf basinc'
        if pressure < self.pressure_min or pressure > self.pressure_max:
            return 'fiziksel basinc araligi disi'
        if (
            self._calibrated
            and
            self._last_accepted_pressure is not None
            and self._last_pressure_valid_stamp is not None
        ):
            dt = (
                now - self._last_pressure_valid_stamp
            ).nanoseconds * 1e-9
            if 0.0 < dt < self.pressure_timeout:
                allowed_jump = max(
                    self.pressure_jump_floor,
                    self.pressure_max_rate * dt,
                )
                if (
                    abs(pressure - self._last_accepted_pressure)
                    > allowed_jump
                ):
                    return 'fiziksel olmayan basinc degisim hizi'
        return None

    def _reject_pressure(self, reason: str):
        self._pressure_rejected_count += 1
        self._last_pressure_rejection = reason

    def _calibration_gate_ready(self, now) -> bool:
        water_dry = (
            self._water_1 is False
            and self._water_2 is False
            and self._is_fresh(
                self._last_water_1_stamp, self.water_timeout)
            and self._is_fresh(
                self._last_water_2_stamp, self.water_timeout)
        )
        imu_ready = (
            self._imu_still
            and self._is_fresh(
                self._last_imu_valid_stamp, self.pixhawk_timeout)
        )
        allowed = (
            not self._thrust_active
            and (water_dry or not self.require_water_dry)
            and (imu_ready or not self.require_imu_still)
        )

        if not allowed:
            if self._calibration_gate_since is not None:
                self._pressure_hampel.reset()
            self._calibration_gate_since = None
            self._calib_buffer.clear()
            reasons = []
            if self._thrust_active:
                reasons.append('itki aktif')
            if self.require_water_dry and not water_dry:
                reasons.append('iki su sensoru kuru/aktif degil')
            if self.require_imu_still and not imu_ready:
                reasons.append('IMU hareketsiz/aktif degil')
            self._calibration_state = ', '.join(reasons)
            return False

        if self._calibration_gate_since is None:
            self._calibration_gate_since = now
            self._pressure_hampel.reset()
            self._last_accepted_pressure = None
            self._calibration_state = 'yuzey kosullari kararlilasiyor'
            return False

        stable_for = (
            now - self._calibration_gate_since
        ).nanoseconds * 1e-9
        if stable_for < self.calibration_stable_s:
            self._calibration_state = 'yuzey kosullari kararlilasiyor'
            return False
        self._calibration_state = 'basinc ornekleri toplaniyor'
        return True

    def _try_finish_pressure_calibration(self):
        p0, accepted, span = robust_median(
            self._calib_buffer,
            sigma=3.0,
            minimum_deviation=1.0,
        )
        valid_ratio = len(accepted) / len(self._calib_buffer)
        if (
            span > self.calibration_max_span
            or valid_ratio < self.calibration_min_valid_ratio
        ):
            self._calibration_state = (
                f'basinc kararsiz: span={span:.1f} Pa, '
                f'gecerli_oran={valid_ratio:.2f}'
            )
            return

        self._p0 = p0
        self._calibrated = True
        self._calibration_state = 'tamamlandi'
        self._pressure_lpf.reset(p0)
        self._filtered_pressure = p0
        self._last_pressure_filter_stamp = self.get_clock().now()
        self._depth_rate_estimator.reset()
        self._depth_rate_lpf.reset(0.0)
        self._depth = 0.0
        self._depth_rate = 0.0
        self._depth_rate_valid = False
        self.get_logger().info(
            f'Guvenli yuzey referansi P0={self._p0:.1f} Pa belirlendi '
            f'(span={span:.1f} Pa, n={len(accepted)}).')

    # ======================================================================
    # Zamanlayici - Yaklasik Ilerleme + Durum Bilgisi Uretimi
    # ======================================================================
    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        # --- Yaklasik Ilerleme / Hareket Tutarliligi ---
        # DUZELTME: sadece IMU stabil diye degil, GERCEKTEN itki uygulaniyorken
        # (thrust_active) ilerleme biriktir - aksi halde durgun araç bile
        # zamanla "ilerlemis" gibi hesaplanir (ivmeolcer sabit hizi tespit edemez).
        #
        # DUZELTME 2 (onemli): ARTIK SABIT v_kalibre ile DEGIL, GERCEK itki
        # seviyesiyle ORANTILI hesaplaniyor: hiz_tahmini = thrust_command *
        # max_calibrated_speed. Boylece gudumun farkli fazlarda farkli itki
        # seviyeleri istemesi (orn. yavas kalibrasyon fazi %29 itki, hizli
        # seyir fazi %100 itki) doğru mesafe tahminine yansir - KTR'deki
        # 0.895 m/s ile 1.076 m/s arasindaki BUYUK fark artik dogru temsil
        # ediliyor (eski sabit-hiz varsayimi bunu yapamiyordu).
        if self._motion_consistent and self._thrust_active and dt > 0.0:
            estimated_speed = abs(self._last_thrust_value) * self.max_calibrated_speed
            self._approx_distance += estimated_speed * dt
            if self.enable_xy:
                self._approx_x += estimated_speed * math.cos(self._heading) * dt
                self._approx_y += estimated_speed * math.sin(self._heading) * dt

        # --- Sensor "canlilik" kontrolleri (henuz baglanmamis sensorler icin) ---
        pixhawk_connected = self._is_fresh(
            self._last_imu_valid_stamp, self.pixhawk_timeout)
        pressure_active = self._is_fresh(
            self._last_pressure_valid_stamp, self.pressure_timeout)
        water_1_active = self._is_fresh(
            self._last_water_1_stamp, self.water_timeout)
        water_2_active = self._is_fresh(
            self._last_water_2_stamp, self.water_timeout)
        water_active = water_1_active and water_2_active
        depth_valid = self._calibrated and pressure_active

        # --- Odometry mesaji ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'sara_odom'
        odom.child_frame_id = 'sara_base_link'

        odom.pose.pose.position.x = float(self._approx_x)
        odom.pose.pose.position.y = float(self._approx_y)
        odom.pose.pose.position.z = float(self._depth)  # derinlik, pozitif = su altina dogru

        qx, qy, qz, qw = euler_to_quaternion(self._roll, self._pitch, self._heading)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        if depth_valid:
            depth_variance = (
                self._pressure_variance_pa2
                / (self.rho * self.g) ** 2
            )
            odom.pose.covariance[14] = float(depth_variance)
        if len(self._orientation_covariance) == 9:
            odom.pose.covariance[21] = float(
                self._orientation_covariance[0])
            odom.pose.covariance[28] = float(
                self._orientation_covariance[4])
            odom.pose.covariance[35] = float(
                self._orientation_covariance[8])

        # DUZELTME: twist.linear.x, yukaridaki mesafe birikimiyle (Satir
        # ~359-361) AYNI tahmini hizi raporlamali. Eskiden sabit v_kalibre
        # kullaniliyordu - bu, gercek itki seviyesinden bagimsiz, YANLIS/
        # tutarsiz bir deger raporluyordu (orn. kalibrasyon fazinda 0.895
        # m/s ilerlerken telemetri hala 0.9 sabit degerini gosteriyordu,
        # tesadufen yakindi ama ana seyir fazinda 1.076 m/s'de de ayni 0.9
        # gorunuyordu - yanlis).
        estimated_speed_for_twist = (
            abs(self._last_thrust_value) * self.max_calibrated_speed
            if (self._motion_consistent and self._thrust_active) else 0.0
        )
        odom.twist.twist.linear.x = float(estimated_speed_for_twist)
        odom.twist.twist.linear.z = float(
            self._depth_rate if self._depth_rate_valid else 0.0)
        odom.twist.twist.angular.x = float(self._roll_rate)
        odom.twist.twist.angular.y = float(self._pitch_rate)
        odom.twist.twist.angular.z = float(self._yaw_rate)

        # covariance[0] alanini depth_valid/motion_consistent gostergesi olarak
        # KULLANMIYORUZ (yanlis okunmaya acik) - onun yerine ayri status mesaji var.
        self._odom_pub.publish(odom)

        # --- Yuzey konumu ---
        surface_msg = Bool()
        surface_msg.data = bool(self._surface_detected)
        self._surface_pub.publish(surface_msg)

        # --- Durum / tani mesaji ---
        status = DiagnosticStatus()
        status.name = 'sara_navigation'
        status.hardware_id = 'jetson_orin_nano'

        if not pixhawk_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Pixhawk IMU baglantisi yok / zaman asimi'
        elif self._calibrated and not pressure_active:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                'Basinc verisi gecersiz / zaman asimi; derinlik guvenilmez')
        elif not water_active:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                'SEN0368 sensorlerinden biri veya ikisi veri gondermiyor')
        elif not self._calibrated:
            status.level = DiagnosticStatus.WARN
            status.message = (
                'Derinlik kalibrasyonu bekleniyor: '
                + self._calibration_state
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Nominal'

        status.values = [
            KeyValue(key='pixhawk_connected', value=str(pixhawk_connected)),
            KeyValue(key='depth_valid', value=str(depth_valid)),
            KeyValue(key='pressure_sensor_active', value=str(pressure_active)),
            KeyValue(key='water_sensors_active', value=str(water_active)),
            KeyValue(key='water_sensor_1_active', value=str(water_1_active)),
            KeyValue(key='water_sensor_2_active', value=str(water_2_active)),
            KeyValue(key='motion_consistent', value=str(self._motion_consistent)),
            KeyValue(key='imu_still', value=str(self._imu_still)),
            KeyValue(key='thrust_active', value=str(self._thrust_active)),
            KeyValue(key='surface_detected', value=str(self._surface_detected)),
            KeyValue(key='approx_distance_m', value=f'{self._approx_distance:.2f}'),
            KeyValue(key='surface_reference_pa', value=(
                '-' if self._p0 is None else f'{self._p0:.1f}')),
            KeyValue(key='filtered_pressure_pa', value=(
                '-' if self._filtered_pressure is None
                else f'{self._filtered_pressure:.1f}')),
            KeyValue(key='depth_rate_ms', value=f'{self._depth_rate:.3f}'),
            KeyValue(
                key='depth_rate_valid', value=str(self._depth_rate_valid)),
            KeyValue(
                key='calibration_state', value=self._calibration_state),
            KeyValue(
                key='pressure_rejected_count',
                value=str(self._pressure_rejected_count),
            ),
            KeyValue(
                key='last_pressure_rejection',
                value=self._last_pressure_rejection,
            ),
            KeyValue(
                key='imu_rejected_count',
                value=str(self._imu_rejected_count),
            ),
        ]
        self._status_pub.publish(status)

    def _is_fresh(self, stamp, timeout_sec) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < timeout_sec


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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
