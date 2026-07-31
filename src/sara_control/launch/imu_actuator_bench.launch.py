#!/usr/bin/env python3
"""Pixhawk hareketi -> dogrudan servo ve sinirli motor donanim testi.

Bu launch mission_start ve guidance dugumlerini baslatmaz. Burun kapagi
su sensoru desenini izler. Siringa, enable_surface_servo aciksa surekli
1900 us disari itme ve 1500 us notr desenini tekrarlar.
Manuel 3 V olcum testi ayrica enable_ignition_manual_test ile acikca
etkinlestirilebilir;
otomatik gorev atesleme komutu bench'te False tutulur. Ana guc kesme
testi iki ayri onay acildiktan
sonra /sara/bench/power_cutoff_request=True ile testin son adimi olarak
tetiklenebilir. Komutlar normal safety_node ve actuator_driver zincirinden
gecmeye devam eder.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sara_control')
    config_dir = os.path.join(pkg_share, 'config')

    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor',
        default_value='false',
        description='Sinirli bench motor komutunu etkinlestir',
    )
    confirm_arg = DeclareLaunchArgument(
        'confirm_motor_test_safe',
        default_value='false',
        description=(
            'Pervanenin sokuldugunu veya aracin guvenli duzenekte '
            'sabitlendigini operator onayi'
        ),
    )
    speed_arg = DeclareLaunchArgument(
        'motor_test_speed_ms',
        default_value='0.15',
        description='Bench motor hedef hizi; kod tarafinda en fazla 0.70 m/s',
    )
    surface_servo_arg = DeclareLaunchArgument(
        'enable_surface_servo',
        default_value='false',
        description=(
            'Uno D6 siringa servosunda 1000 ms 1900 us disari itme / '
            '200 ms 1500 us notr desenini tekrarlar; '
            'actuator_driver motoru ve ateslemeyi zorla kapali tutar'
        ),
    )
    surface_servo_pulse_arg = DeclareLaunchArgument(
        'surface_servo_pulse_us',
        default_value='1900',
        description=(
            'D6 bench test darbesi: 1100=iceri cekme, 1900=disari itme; '
            'baska degerler guvenlik nedeniyle kabul edilmez'
        ),
    )
    ignition_manual_test_arg = DeclareLaunchArgument(
        'enable_ignition_manual_test',
        default_value='false',
        description=(
            'SADECE MULTIMETRE/LED YUKUYLE: D2 basiliyken 3 V manuel '
            'olcum testini etkinlestir; bu mod motoru zorla kapatir'
        ),
    )
    power_cutoff_arg = DeclareLaunchArgument(
        'enable_power_cutoff_test',
        default_value='false',
        description=(
            'Testin sonunda /sara/bench/power_cutoff_request ile Uno D5 '
            'ana guc kesmesini denemeye izin ver'
        ),
    )
    power_cutoff_confirm_arg = DeclareLaunchArgument(
        'confirm_power_cutoff_test_safe',
        default_value='false',
        description=(
            'Guc kesme kartinin yalniz guvenli olcum yukune bagli oldugunu '
            've kesme sonrasinda testin bitecegini operator onayi'
        ),
    )

    enable_motor = LaunchConfiguration('enable_motor')
    confirmed = LaunchConfiguration('confirm_motor_test_safe')
    motor_speed = LaunchConfiguration('motor_test_speed_ms')
    enable_surface_servo = LaunchConfiguration('enable_surface_servo')
    surface_servo_pulse_us = LaunchConfiguration('surface_servo_pulse_us')
    enable_ignition_manual_test = LaunchConfiguration(
        'enable_ignition_manual_test')
    enable_power_cutoff = LaunchConfiguration(
        'enable_power_cutoff_test')
    confirm_power_cutoff = LaunchConfiguration(
        'confirm_power_cutoff_test_safe')

    return LaunchDescription([
        enable_motor_arg,
        confirm_arg,
        speed_arg,
        surface_servo_arg,
        surface_servo_pulse_arg,
        ignition_manual_test_arg,
        power_cutoff_arg,
        power_cutoff_confirm_arg,
        Node(
            package='sara_control',
            executable='actuator_driver',
            name='actuator_driver',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'actuator_driver_params.yaml'),
                # Standalone kod servo acisini dogrudan 90+offset olarak
                # hesaplar. Bench dugumu normalize offset yayinladigi icin
                # ikinci bir isaret terslemesi uygulanmaz.
                {
                    'pitch_sign': 1.0,
                    'yaw_sign': 1.0,
                    'surface_servo_test_mode': ParameterValue(
                        enable_surface_servo, value_type=bool),
                    'surface_servo_test_pulse_us': ParameterValue(
                        surface_servo_pulse_us, value_type=int),
                    'ignition_manual_test_mode': ParameterValue(
                        enable_ignition_manual_test, value_type=bool),
                },
            ],
        ),
        Node(
            package='sara_control',
            executable='sensor_get_data',
            name='sensor_data_node',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'sensor_get_data_params.yaml'),
            ],
        ),
        Node(
            package='sara_control',
            executable='navigation',
            name='navigation_node',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'navigation_params.yaml'),
            ],
        ),
        Node(
            package='sara_control',
            executable='safety',
            name='safety_node',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'safety_params.yaml'),
                # Bench testinde CH3 su sensoru desenini anlik takip eder.
                {'nose_cap_latching': False},
            ],
        ),
        Node(
            package='sara_control',
            executable='imu_actuator_bench',
            name='imu_actuator_bench',
            output='screen',
            parameters=[
                os.path.join(
                    config_dir, 'imu_actuator_bench_params.yaml'),
                {
                    'enable_motor': ParameterValue(
                        enable_motor, value_type=bool),
                    'confirm_motor_test_safe': ParameterValue(
                        confirmed, value_type=bool),
                    'motor_test_speed_ms': ParameterValue(
                        motor_speed, value_type=float),
                    'enable_power_cutoff_test': ParameterValue(
                        enable_power_cutoff, value_type=bool),
                    'confirm_power_cutoff_test_safe': ParameterValue(
                        confirm_power_cutoff, value_type=bool),
                },
            ],
        ),
    ])
