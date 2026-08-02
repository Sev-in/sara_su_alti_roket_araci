#!/usr/bin/env python3
"""SARA W/A/D uzaktan klavye kontrolu icin guvenli launch dosyasi."""

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
    esc_auto_prepare = LaunchConfiguration('esc_auto_direction_prepare')

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
            'guidance_mode': 'keyboard_teleop',
            'start_mavros': start_mavros,
            'fcu_url': fcu_url,
            'enable_telemetry': enable_telemetry,
            'log_directory': log_directory,
            'auto_start': auto_start,
            'motor_inhibit_duration_s': motor_inhibit,
            'surface_servo_test_mode': 'false',
            'esc_auto_direction_prepare': esc_auto_prepare,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='test',
            description="'test' simulator; 'hardware' gercek arac",
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
        DeclareLaunchArgument('enable_telemetry', default_value='true'),
        DeclareLaunchArgument(
            'log_directory',
            default_value='~/sara_logs/keyboard_teleop',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Varsayilan false; hareket izni operatorce verilir',
        ),
        DeclareLaunchArgument(
            'motor_inhibit_duration_s',
            default_value='60.0',
            description='Motor hareket izni oncesi guvenlik beklemesi',
        ),
        DeclareLaunchArgument(
            'esc_auto_direction_prepare',
            default_value='true',
            description=(
                'Hardware klavye kontrolunde ilk W/A/D komutunda '
                'cekme-notr-itme ESC hazirligini uygula'
            ),
        ),
        sara_system,
    ])
