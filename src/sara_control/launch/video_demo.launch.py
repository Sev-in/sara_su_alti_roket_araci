#!/usr/bin/env python3
"""SARA havuz gorev kabiliyeti videosu - guvenli baslatma launch'i."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    start_mavros = LaunchConfiguration('start_mavros')
    fcu_url = LaunchConfiguration('fcu_url')
    enable_telemetry = LaunchConfiguration('enable_telemetry')
    log_directory = LaunchConfiguration('log_directory')
    auto_start = LaunchConfiguration('auto_start')
    motor_inhibit = LaunchConfiguration('motor_inhibit_duration_s')

    sara_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sara_control'),
                'launch',
                'sara_system.launch.py',
            ])
        ),
        launch_arguments={
            'mode': mode,
            'guidance_mode': 'video_demo',
            'start_mavros': start_mavros,
            'fcu_url': fcu_url,
            'enable_telemetry': enable_telemetry,
            'log_directory': log_directory,
            'auto_start': auto_start,
            'motor_inhibit_duration_s': motor_inhibit,
            'surface_servo_test_mode': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='test',
            description=(
                "'test' simulator; 'hardware' gercek havuz sistemi"
            ),
        ),
        DeclareLaunchArgument(
            'start_mavros',
            default_value='true',
            description='Hardware modunda MAVROS baglantisini baslat',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value=(
                'serial:///dev/serial/by-id/'
                'usb-Auterion_PX4_FMU_v6C.x_0-if00:921600'
            ),
        ),
        DeclareLaunchArgument(
            'enable_telemetry',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'log_directory',
            default_value='~/sara_logs/video_demo',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description=(
                'Guvenlik icin varsayilan false; start_command bekler'
            ),
        ),
        DeclareLaunchArgument(
            'motor_inhibit_duration_s',
            default_value='3.0',
            description=(
                'Yalniz video testi icin kisa baslangic beklemesi'
            ),
        ),
        sara_system,
    ])
