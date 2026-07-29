import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'sara_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='sevin',
    maintainer_email='sevin@todo.todo',
    description='SARA su alti roket araci kontrol ve donanim entegrasyonu',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'actuator_driver = sara_control.actuator_driver:main',
            'autopilot = sara_control.autopilot:main',
            'guidance = sara_control.guidance:main',
            'imu_actuator_bench = sara_control.imu_actuator_bench:main',
            'mission_start = sara_control.mission_start:main',
            'navigation = sara_control.navigation:main',
            'safety = sara_control.safety:main',
            'sara_viz_bridge = sara_control.sara_viz_bridge:main',
            'sensor_get_data = sara_control.sensor_get_data:main',
            'telemetry = sara_control.telemetry:main',
            'vehicle_sim = sara_control.vehicle_sim:main',
            'wave_estimator = sara_control.wave_estimator:main',
        ],
    },
)
