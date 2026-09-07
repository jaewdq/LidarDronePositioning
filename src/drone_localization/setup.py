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
            'analyze_waypoints = drone_localization.analyze_waypoints:main',
            'plot_run = drone_localization.plot_run:main',
            'run_report = drone_localization.run_report:main',
        ],
    },
)
