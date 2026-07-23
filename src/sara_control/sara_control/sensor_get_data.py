#!/usr/bin/env python3
"""SARA gercek donanim sensor veri alma katmani.

Bu dugum mevcut sistem arayuzlerini degistirmeden uc isi yapar:

* ADS1115 uzerinden SEN0257 (0--1.6 MPa) analog basincini okur ve
  ``/sara/pressure`` topic'ine Pa cinsinden yayinlar.
* Jetson GPIO uzerinden iki SEN0368 su sensorunu okuyup mevcut
  ``/sara/water_detect_1`` ve ``/sara/water_detect_2`` topic'lerine
  yayinlar.
* Pixhawk/MAVROS IMU mesajini ``/mavros/imu/data`` topic'inden katmanli
  mimarinin kullandigi ``/sara/imu/data`` topic'ine kopruler.

Donanim kutuphaneleri modul yuklenirken degil, node baslatilirken dinamik
olarak yuklenir. Bu nedenle test/simulasyon modu Jetson kutuphaneleri
kurulu olmayan bilgisayarlarda etkilenmez.

ADS1115 notu:
    SEN0257 5 V ile beslenir ve 4.5 V'a kadar cikis verebilir. ADS1115
    3.3 V ile calisiyorsa gerilim bolucu zorunludur. Varsayilan
    R1=10k/R2=20k ile 4.5 V, ADC tarafinda 3.0 V olur.
"""

import importlib
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool

from sara_control.sensor_processing import (
    sen0257_voltage_to_pressure,
    voltage_divider_reconstruct,
    water_sensor_logic,
)


class SensorGetDataNode(Node):
    """Jetson'a bagli sensorleri standart SARA topic'lerine uyarlar."""

    def __init__(self):
        super().__init__('sensor_data_node')

        # ---------------- Genel ----------------
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('imu_bridge_enabled', True)
        self.declare_parameter('mavros_imu_topic', '/mavros/imu/data')

        # ---------------- ADS1115 / SEN0257 ----------------
        self.declare_parameter('pressure_enabled', True)
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('ads1115_address', 0x48)
        self.declare_parameter('ads1115_channel', 0)
        self.declare_parameter('ads1115_gain', 1.0)
        self.declare_parameter('sen0257_zero_offset_v', 0.5)
        self.declare_parameter('sen0257_output_span_v', 4.0)
        self.declare_parameter('sen0257_full_scale_pa', 1600000.0)
        self.declare_parameter('voltage_divider_r1_ohm', 10000.0)
        self.declare_parameter('voltage_divider_r2_ohm', 20000.0)
        self.declare_parameter('sensor_voltage_min_v', 0.3)
        self.declare_parameter('sensor_voltage_max_v', 4.7)
        self.declare_parameter('pressure_variance_pa2', 0.0)

        # ---------------- SEN0368 x2 ----------------
        self.declare_parameter('water_sensors_enabled', True)
        self.declare_parameter('gpio_numbering_mode', 'BOARD')
        self.declare_parameter('water_sensor_1_pin', 15)
        self.declare_parameter('water_sensor_2_pin', 16)
        self.declare_parameter('water_sensor_1_active_high', True)
        self.declare_parameter('water_sensor_2_active_high', True)

        self.pressure_enabled = self.get_parameter(
            'pressure_enabled').value
        self.water_enabled = self.get_parameter(
            'water_sensors_enabled').value
        self.imu_bridge_enabled = self.get_parameter(
            'imu_bridge_enabled').value

        self.zero_offset_v = self.get_parameter(
            'sen0257_zero_offset_v').value
        self.output_span_v = self.get_parameter(
            'sen0257_output_span_v').value
        self.full_scale_pa = self.get_parameter(
            'sen0257_full_scale_pa').value
        self.r1_ohm = self.get_parameter(
            'voltage_divider_r1_ohm').value
        self.r2_ohm = self.get_parameter(
            'voltage_divider_r2_ohm').value
        self.sensor_voltage_min = self.get_parameter(
            'sensor_voltage_min_v').value
        self.sensor_voltage_max = self.get_parameter(
            'sensor_voltage_max_v').value
        self.pressure_variance = self.get_parameter(
            'pressure_variance_pa2').value

        self.water_1_pin = int(self.get_parameter(
            'water_sensor_1_pin').value)
        self.water_2_pin = int(self.get_parameter(
            'water_sensor_2_pin').value)
        self.water_1_active_high = self.get_parameter(
            'water_sensor_1_active_high').value
        self.water_2_active_high = self.get_parameter(
            'water_sensor_2_active_high').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pressure_pub = self.create_publisher(
            FluidPressure, '/sara/pressure', sensor_qos)
        self._water_1_pub = self.create_publisher(
            Bool, '/sara/water_detect_1', sensor_qos)
        self._water_2_pub = self.create_publisher(
            Bool, '/sara/water_detect_2', sensor_qos)
        self._imu_pub = self.create_publisher(
            Imu, '/sara/imu/data', sensor_qos)
        self._status_pub = self.create_publisher(
            DiagnosticStatus, '/sara/sensors/status', 10)

        if self.imu_bridge_enabled:
            mavros_topic = self.get_parameter('mavros_imu_topic').value
            self.create_subscription(
                Imu,
                mavros_topic,
                self._on_mavros_imu,
                sensor_qos,
            )

        self._ads_ready = False
        self._gpio_ready = False
        self._last_imu_stamp = None
        self._last_pressure_error = '-'
        self._last_water_error = '-'
        self._pressure_error_count = 0
        self._water_error_count = 0
        self._last_sensor_voltage = None
        self._last_pressure_pa = None

        self._analog_input = None
        self._i2c = None
        self._gpio = None

        if self.pressure_enabled:
            self._initialize_ads1115()
        if self.water_enabled:
            self._initialize_gpio()

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate), self._on_timer)
        self.get_logger().info(
            'sensor_get_data baslatildi: '
            f'pressure_ready={self._ads_ready}, '
            f'water_ready={self._gpio_ready}, '
            f'imu_bridge={self.imu_bridge_enabled}')

    def _initialize_ads1115(self):
        try:
            extended_bus = importlib.import_module(
                'adafruit_extended_bus')
            ads_module = importlib.import_module(
                'adafruit_ads1x15.ads1115')
            analog_module = importlib.import_module(
                'adafruit_ads1x15.analog_in')

            bus_id = int(self.get_parameter('i2c_bus').value)
            address = int(self.get_parameter('ads1115_address').value)
            channel_index = int(
                self.get_parameter('ads1115_channel').value)
            gain = float(self.get_parameter('ads1115_gain').value)

            self._i2c = extended_bus.ExtendedI2C(bus_id)
            ads = ads_module.ADS1115(self._i2c, address=address)
            ads.gain = gain
            channels = (
                ads_module.P0,
                ads_module.P1,
                ads_module.P2,
                ads_module.P3,
            )
            if channel_index not in range(len(channels)):
                raise ValueError('ADS1115 channel 0--3 araliginda olmali')
            self._analog_input = analog_module.AnalogIn(
                ads, channels[channel_index])
            self._ads_ready = True
        except Exception as exc:
            self._ads_ready = False
            self._last_pressure_error = str(exc)
            self.get_logger().error(
                'ADS1115/SEN0257 baslatilamadi: '
                f'{exc}. Hardware modunda adafruit-blinka, '
                'adafruit-circuitpython-ads1x15 ve '
                'adafruit-extended-bus kurulmus olmali.')

    def _initialize_gpio(self):
        try:
            gpio = importlib.import_module('Jetson.GPIO')
            mode_name = str(self.get_parameter(
                'gpio_numbering_mode').value).upper()
            if mode_name == 'BCM':
                gpio.setmode(gpio.BCM)
            else:
                gpio.setmode(gpio.BOARD)
            gpio.setup(self.water_1_pin, gpio.IN)
            gpio.setup(self.water_2_pin, gpio.IN)
            self._gpio = gpio
            self._gpio_ready = True
        except Exception as exc:
            self._gpio_ready = False
            self._last_water_error = str(exc)
            self.get_logger().error(
                'SEN0368 GPIO baslatilamadi: '
                f'{exc}. Hardware modunda Jetson.GPIO kurulmus olmali.')

    def _on_mavros_imu(self, msg: Imu):
        self._last_imu_stamp = self.get_clock().now()
        self._imu_pub.publish(msg)

    def _on_timer(self):
        if self.pressure_enabled and self._ads_ready:
            self._read_and_publish_pressure()
        if self.water_enabled and self._gpio_ready:
            self._read_and_publish_water()
        self._publish_status()

    def _read_and_publish_pressure(self):
        try:
            adc_voltage = float(self._analog_input.voltage)
            sensor_voltage = voltage_divider_reconstruct(
                adc_voltage,
                self.r1_ohm,
                self.r2_ohm,
            )
            if (
                not math.isfinite(sensor_voltage)
                or sensor_voltage < self.sensor_voltage_min
                or sensor_voltage > self.sensor_voltage_max
            ):
                raise ValueError(
                    f'SEN0257 gerilimi aralik disi: {sensor_voltage:.3f} V')

            pressure_pa = sen0257_voltage_to_pressure(
                sensor_voltage,
                zero_offset_v=self.zero_offset_v,
                full_scale_pa=self.full_scale_pa,
                output_span_v=self.output_span_v,
                clamp_to_range=False,
            )
            message = FluidPressure()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'sara_pressure_sensor'
            message.fluid_pressure = float(pressure_pa)
            message.variance = float(max(0.0, self.pressure_variance))
            self._pressure_pub.publish(message)
            self._last_sensor_voltage = sensor_voltage
            self._last_pressure_pa = pressure_pa
            self._last_pressure_error = '-'
        except Exception as exc:
            self._pressure_error_count += 1
            self._last_pressure_error = str(exc)

    def _read_and_publish_water(self):
        try:
            raw_1 = self._gpio.input(self.water_1_pin)
            raw_2 = self._gpio.input(self.water_2_pin)
            water_1 = Bool()
            water_1.data = water_sensor_logic(
                raw_1, self.water_1_active_high)
            water_2 = Bool()
            water_2.data = water_sensor_logic(
                raw_2, self.water_2_active_high)
            self._water_1_pub.publish(water_1)
            self._water_2_pub.publish(water_2)
            self._last_water_error = '-'
        except Exception as exc:
            self._water_error_count += 1
            self._last_water_error = str(exc)

    def _publish_status(self):
        imu_fresh = not self.imu_bridge_enabled
        if self.imu_bridge_enabled and self._last_imu_stamp is not None:
            age = (
                self.get_clock().now() - self._last_imu_stamp
            ).nanoseconds * 1e-9
            imu_fresh = age < 1.0

        pressure_ok = not self.pressure_enabled or self._ads_ready
        water_ok = not self.water_enabled or self._gpio_ready
        status = DiagnosticStatus()
        status.name = 'sara_sensor_get_data'
        status.hardware_id = 'jetson_orin_nano'
        status.level = (
            DiagnosticStatus.OK
            if pressure_ok and water_ok and imu_fresh
            else DiagnosticStatus.ERROR
        )
        status.message = (
            'Nominal' if status.level == DiagnosticStatus.OK
            else 'Sensor katmani hazir degil'
        )
        status.values = [
            KeyValue(key='ads1115_ready', value=str(self._ads_ready)),
            KeyValue(key='gpio_ready', value=str(self._gpio_ready)),
            KeyValue(key='imu_bridge_fresh', value=str(imu_fresh)),
            KeyValue(key='sensor_voltage_v', value=(
                '-' if self._last_sensor_voltage is None
                else f'{self._last_sensor_voltage:.4f}')),
            KeyValue(key='pressure_pa', value=(
                '-' if self._last_pressure_pa is None
                else f'{self._last_pressure_pa:.1f}')),
            KeyValue(
                key='pressure_error_count',
                value=str(self._pressure_error_count),
            ),
            KeyValue(
                key='water_error_count',
                value=str(self._water_error_count),
            ),
            KeyValue(
                key='last_pressure_error',
                value=self._last_pressure_error,
            ),
            KeyValue(
                key='last_water_error',
                value=self._last_water_error,
            ),
        ]
        self._status_pub.publish(status)

    def destroy_node(self):
        if self._gpio is not None:
            try:
                self._gpio.cleanup(
                    [self.water_1_pin, self.water_2_pin])
            except Exception:
                pass
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorGetDataNode()
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
