#!/usr/bin/env python3
"""Pixhawk hareketi -> dogrudan servo ve sinirli motor donanim testi.

Bu launch mission_start ve guidance dugumlerini baslatmaz. Burun kapagi,
siringa ve atesleme istekleri her dongude False tutulur. Komutlar normal
safety_node ve actuator_driver zincirinden gecmeye devam eder.
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
        description='Bench motor hedef hizi; kod tarafinda en fazla 0.25 m/s',
    )

    enable_motor = LaunchConfiguration('enable_motor')
    confirmed = LaunchConfiguration('confirm_motor_test_safe')
    motor_speed = LaunchConfiguration('motor_test_speed_ms')

    return LaunchDescription([
        enable_motor_arg,
        confirm_arg,
        speed_arg,
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
                {'pitch_sign': 1.0, 'yaw_sign': 1.0},
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
                },
            ],
        ),
    ])
