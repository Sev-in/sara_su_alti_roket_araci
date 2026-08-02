#!/usr/bin/env python3
"""
sara_system.launch.py
=======================
SARA platformu - TUM sistemi TEK KOMUTLA baslatan launch dosyasi.

KULLANIM:

  Test modu (vehicle_sim ile, donanim GEREKMEZ):
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=1
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=2
    ros2 launch sara_control video_demo.launch.py mode:=test

  Gercek donanim modu (Pixhawk bagli olmali; MAVROS varsayilan olarak
  bu launch dosyasi tarafindan baslatilir):
    ros2 launch sara_control sara_system.launch.py mode:=hardware mission_id:=1

  Hizli test iterasyonu (60 sn motor inhibit suresini gecici dusurmek icin -
  SADECE TEST, yarisma/sahada MUTLAKA varsayilan 60.0 ile calistirin):
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=1 \
      motor_inhibit_duration_s:=2.0

ARGUMANLAR:
    mission_id     : '1' (Seyir Gorevi) veya '2' (Atis Gorevi). Varsayilan: '1'
    guidance_mode  : 'mission' normal Gorev 1/2, 'video_demo' otonom havuz
                     gosterimi veya 'keyboard_teleop' uzaktan klavye kontrolu.
                     VIDEO_DEMO icin dogrudan video_demo.launch.py kullanin.
    VIDEO_DEMO burun ayirma/3 V gosterimi icermez; bu test karada yapilir.
    mode           : 'test' (vehicle_sim) veya 'hardware' (gercek sensor/eyleyici
                     suruculeri). Varsayilan: 'test'
    enable_telemetry : CSV telemetri kaydini ac/kapat. Varsayilan: 'true'
    log_directory  : telemetri CSV'lerinin kaydedilecegi klasor.
                     Varsayilan: '~/sara_logs'
    motor_inhibit_duration_s : mission_start_node'un 60 sn motor-inhibit
                     suresi. Varsayilan: '60.0' (sartname 4.2 - SABIT
                     deger, YARISMADA/SAHADA DEGISTIRILMEMELIDIR). Sadece
                     hizli iterasyon icin dusurulebilir bir launch
                     argumani olarak sunulmustur.
    auto_start     : Ana enerjilendirmede motor-inhibit geri sayimini
                     otomatik baslatir. Varsayilan: 'true'. 'false' verilirse
                     /sara/mission_start/start_command beklenir.
    start_mavros   : Hardware modunda MAVROS'u ayni launch icinde baslatir.
                     Varsayilan: 'true'. MAVROS zaten elle calisiyorsa
                     port cakismamasi icin 'false' verilmelidir.
    fcu_url        : Pixhawk seri portu ve baud hizi.

Hardware modunda sensor_get_data basinc, su sensorleri ve MAVROS IMU
koprusunu tek noktada toplar. actuator_driver ise yalnizca safety
katmaninin onayladigi komutlari Uno/PCA9685'e iletir.

mission_id, LaunchConfiguration string degerinden ParameterValue ile
acikca integer tipine donusturulur.

Her node, once kendi config/*.yaml dosyasini okur, SONRA launch
argumanlari (orn. mission_id) bu degerlerin UZERINE yazar.

NOT: MAVROS zaten ayri bir terminalde calisiyorsa:
    ros2 launch sara_control sara_system.launch.py mode:=hardware \
      mission_id:=1 start_mavros:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('sara_control')
    config_dir = os.path.join(pkg_share, 'config')

    mission_id_arg = DeclareLaunchArgument(
        'mission_id', default_value='1',
        description="Kosulacak gorev: '1' (Seyir) veya '2' (Atis)"
    )
    guidance_mode_arg = DeclareLaunchArgument(
        'guidance_mode', default_value='mission',
        description=(
            "'mission' normal Gorev 1/2 gudumu; 'video_demo' havuz "
            "gorev kabiliyeti gosterimi; 'keyboard_teleop' W/A/D kontrolu"
        ),
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='test',
        description="'test' (vehicle_sim) veya 'hardware' (gercek sensor/eyleyici suruculeri)"
    )
    enable_telemetry_arg = DeclareLaunchArgument(
        'enable_telemetry', default_value='true',
        description='CSV telemetri kaydini ac/kapat'
    )
    log_directory_arg = DeclareLaunchArgument(
        'log_directory', default_value='~/sara_logs',
        description='Telemetri CSV kayit klasoru'
    )
    motor_inhibit_arg = DeclareLaunchArgument(
        'motor_inhibit_duration_s', default_value='60.0',
        description=(
            "mission_start_node 60 sn motor-inhibit suresi (sartname 4.2 - "
            "SABIT deger). SADECE hizli test iterasyonu icin dusurulmelidir; "
            "yarisma/saha kosusunda MUTLAKA varsayilan '60.0' ile birakin."
        )
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='true',
        description=(
            'Ana enerjilendirmede motor-inhibit geri sayimini otomatik '
            'baslat; false ise start_command bekle'
        )
    )
    surface_servo_test_arg = DeclareLaunchArgument(
        'surface_servo_test_mode', default_value='false',
        description=(
            'SADECE TEZGAH TESTI: Uno D6 siringa servosunda heartbeat boyunca '
            '1800 us disari itme / 1500 us notr desenini tekrarlar; '
            'motor ve '
            'atesleme actuator_driver '
            'tarafinda zorla guvenli tutulur'
        )
    )
    surface_servo_pulse_arg = DeclareLaunchArgument(
        'surface_servo_test_pulse_us', default_value='1800',
        description=(
            'Yalniz test modunda D6 darbesi: 1200=iceri cek, '
            '1800=disari it; normal gorev daima 1800 kullanir'
        )
    )
    esc_auto_prepare_arg = DeclareLaunchArgument(
        'esc_auto_direction_prepare', default_value='false',
        description=(
            'Ilk pozitif itki isteginde ESC cekme-notr-itme hazirligini '
            'uygula; yalniz fiziksel olarak dogrulanmis hardware '
            'kullaniminda true yapilmalidir'
        )
    )
    start_mavros_arg = DeclareLaunchArgument(
        'start_mavros', default_value='true',
        description=(
            'Hardware modunda MAVROS PX4 baglantisini bu launch icinde '
            'baslat; MAVROS zaten calisiyorsa false kullan'
        )
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value=(
            'serial:///dev/serial/by-id/'
            'usb-Auterion_PX4_FMU_v6C.x_0-if00:921600'
        ),
        description='Pixhawk MAVROS seri baglanti URLsi',
    )

    mission_id = LaunchConfiguration('mission_id')
    guidance_mode = LaunchConfiguration('guidance_mode')
    mode = LaunchConfiguration('mode')
    enable_telemetry = LaunchConfiguration('enable_telemetry')
    log_directory = LaunchConfiguration('log_directory')
    motor_inhibit_duration_s = LaunchConfiguration('motor_inhibit_duration_s')
    auto_start = LaunchConfiguration('auto_start')
    surface_servo_test_mode = LaunchConfiguration(
        'surface_servo_test_mode')
    surface_servo_test_pulse_us = LaunchConfiguration(
        'surface_servo_test_pulse_us')
    esc_auto_direction_prepare = LaunchConfiguration(
        'esc_auto_direction_prepare')
    start_mavros = LaunchConfiguration('start_mavros')
    fcu_url = LaunchConfiguration('fcu_url')

    is_test_mode = PythonExpression(["'", mode, "' == 'test'"])
    is_hardware_mode = PythonExpression(["'", mode, "' == 'hardware'"])
    is_mission_guidance = PythonExpression([
        "'", guidance_mode, "' == 'mission'",
    ])
    is_video_demo_guidance = PythonExpression([
        "'", guidance_mode, "' == 'video_demo'",
    ])
    is_keyboard_teleop_guidance = PythonExpression([
        "'", guidance_mode, "' == 'keyboard_teleop'",
    ])
    is_video_or_teleop_guidance = PythonExpression([
        "'", guidance_mode,
        "' == 'video_demo' or '", guidance_mode,
        "' == 'keyboard_teleop'",
    ])
    start_hardware_mavros = PythonExpression([
        "'", mode, "' == 'hardware' and '",
        start_mavros, "'.lower() == 'true'",
    ])

    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch',
            ])
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
        condition=IfCondition(start_hardware_mavros),
    )

    # ================= TEST MODU - sahte sensor/fizik simulatoru =================
    vehicle_sim_node = Node(
        package='sara_control',
        executable='vehicle_sim',
        name='vehicle_sim',
        output='screen',
        parameters=[os.path.join(config_dir, 'vehicle_sim_params.yaml')],
        condition=IfCondition(is_test_mode),
    )

    # ================= GERCEK DONANIM MODU =================
    # Sensor okuma ve eyleyici uygulama sorumluluklari ayri dugumlerdedir.
    actuator_driver_node = Node(
        package='sara_control',
        executable='actuator_driver',
        name='actuator_driver',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'actuator_driver_params.yaml'),
            {
                'surface_servo_test_mode': ParameterValue(
                    surface_servo_test_mode, value_type=bool),
                'surface_servo_test_pulse_us': ParameterValue(
                    surface_servo_test_pulse_us, value_type=int),
                'esc_auto_direction_prepare': ParameterValue(
                    esc_auto_direction_prepare, value_type=bool),
            },
        ],
        condition=IfCondition(is_hardware_mode),
    )
    sensor_get_data_node = Node(
        package='sara_control',
        executable='sensor_get_data',
        name='sensor_data_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'sensor_get_data_params.yaml'),
        ],
        condition=IfCondition(is_hardware_mode),
    )

    # ================= HER IKI MODDA DA CALISAN CEKIRDEK NODE'LAR =================
    # Her node: once kendi yaml'ini okur, SONRA (varsa) launch argumani UZERINE yazar.
    navigation_node = Node(
        package='sara_control',
        executable='navigation',
        name='navigation_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'navigation_params.yaml')],
    )

    mission_start_node = Node(
        package='sara_control',
        executable='mission_start',
        name='mission_start_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'mission_start_params.yaml'),
            # DUZELTME: motor_inhibit_duration_s artik yaml'daki sabit
            # degerin UZERINE, launch argumaniyla (varsayilan yine 60.0)
            # yazilabiliyor - hizli test iterasyonu icin.
            {'motor_inhibit_duration_s': ParameterValue(motor_inhibit_duration_s, value_type=float)},
            {'auto_start': ParameterValue(auto_start, value_type=bool)},
        ],
    )

    guidance_node = Node(
        package='sara_control',
        executable='guidance',
        name='guidance_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'guidance_params.yaml'),
            # DUZELTME: LaunchConfiguration HER ZAMAN string uretir;
            # guidance.py'de mission_id INTEGER olarak declare edilmis.
            # ParameterValue ile ACIKCA int'e cevrilmeden birakilirsa
            # ROS2 tip uyusmazligi hatasi verebilir.
            {'mission_id': ParameterValue(mission_id, value_type=int)},
        ],
        condition=IfCondition(is_mission_guidance),
    )

    video_demo_guidance_node = Node(
        package='sara_control',
        executable='video_demo_guidance',
        name='guidance_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'video_demo_params.yaml')],
        condition=IfCondition(is_video_demo_guidance),
    )

    keyboard_teleop_node = Node(
        package='sara_control',
        executable='keyboard_teleop',
        name='guidance_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'keyboard_teleop_params.yaml'),
        ],
        condition=IfCondition(is_keyboard_teleop_guidance),
    )

    autopilot_node = Node(
        package='sara_control',
        executable='autopilot',
        name='autopilot_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'autopilot_params.yaml')],
    )

    safety_node = Node(
        package='sara_control',
        executable='safety',
        name='safety_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'safety_params.yaml'),
            {
                'video_demo_mode': ParameterValue(
                    is_video_or_teleop_guidance,
                    value_type=bool,
                ),
                'video_demo_ignition_enabled': False,
            },
        ],
    )

    # Pixhawk IMU + navigasyon derinligi ile dalga etkisini izler.
    # Bu dugum yalnizca tani/telemetri yayinlar; gorev komutlarini
    # degistirmez. Esikler deniz rosbag kayitlariyla dogrulanmadan
    # kontrol zincirine baglanmamalidir.
    wave_estimator_node = Node(
        package='sara_control',
        executable='wave_estimator',
        name='wave_estimator_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'wave_estimator_params.yaml'),
        ],
    )

    telemetry_node = Node(
        package='sara_control',
        executable='telemetry',
        name='telemetry_node',
        output='screen',
        parameters=[{'log_directory': log_directory}],
        condition=IfCondition(enable_telemetry),
    )

    return LaunchDescription([
        mission_id_arg,
        guidance_mode_arg,
        mode_arg,
        enable_telemetry_arg,
        log_directory_arg,
        motor_inhibit_arg,
        auto_start_arg,
        surface_servo_test_arg,
        surface_servo_pulse_arg,
        esc_auto_prepare_arg,
        start_mavros_arg,
        fcu_url_arg,

        mavros_node,
        vehicle_sim_node,
        actuator_driver_node,
        sensor_get_data_node,

        navigation_node,
        mission_start_node,
        guidance_node,
        video_demo_guidance_node,
        keyboard_teleop_node,
        autopilot_node,
        safety_node,
        wave_estimator_node,
        telemetry_node,
    ])
