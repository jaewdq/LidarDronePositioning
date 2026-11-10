import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'grid_sensor_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyup',
    maintainer_email='jaeyup01@naver.com',
    description='Indoor calibration of two ground LiDARs: ground plane (z, roll, pitch) and target points (x, y, yaw)',
    license='Apache-2.0',
    # colcon selects its pytest runner based on this declaration.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_calibration_node = '
            'grid_sensor_calibration.sensor_calibration_node:main',
            'pointcloud_target_node = '
            'grid_sensor_calibration.pointcloud_target_node:main',
            'ground_plane_node = '
            'grid_sensor_calibration.ground_plane_node:main',
            'apply_calibration = '
            'grid_sensor_calibration.apply_calibration_node:main',
            'calibration_gui = '
            'grid_sensor_calibration.calibration_gui:main',
        ],
    },
)
