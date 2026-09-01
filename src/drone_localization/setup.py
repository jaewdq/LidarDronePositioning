from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyup',
    maintainer_email='jaeyup01@naver.com',
    description='LiDAR-based drone 3D localization (outside-in, EKF)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_drone_tracker = drone_localization.lidar_drone_tracker:main',
            'lidar_drone_tracker_ekf = drone_localization.lidar_drone_tracker_ekf:main',
            'lidar_drone_tracker_seq = drone_localization.lidar_drone_tracker_seq:main',
            'estimate_viz = drone_localization.estimate_viz:main',
            'flight_recorder = drone_localization.flight_recorder:main',
            'plot_flight = drone_localization.plot_flight:main',
            'analyze_waypoints = drone_localization.analyze_waypoints:main',
            'plot_flight_3d = drone_localization.plot_flight_3d:main',
            'plot_run = drone_localization.plot_run:main',
        ],
    },
)
