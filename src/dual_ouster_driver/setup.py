from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dual_ouster_driver'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            # xml 도 포함해야 한다. fastdds_large.xml 이 여기 있고,
            # 이게 없으면 2대 구성에서 10Hz -> 1.4Hz 로 무너진다.
            glob('config/*.yaml') + glob('config/*.xml')
        ),
        # rviz 설정도 설치한다. 안 하면 런치에서 소스 경로를 추측해야 하는데
        # build/ 와 install/ 의 폴더 구조가 달라서 신뢰할 수 없다.
        (
            os.path.join(
                'share',
                package_name,
                'rviz'
            ),
            glob('rviz/*.rviz')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    entry_points={
        'console_scripts': [
            'roi_filter = dual_ouster_driver.roi_filter:main',
            'axis_probe = dual_ouster_driver.axis_probe:main',
            'cluster_median = dual_ouster_driver.cluster_median:main',
            'room_survey = dual_ouster_driver.room_survey:main',
            'marker_fit = dual_ouster_driver.marker_fit:main',
            'pose_solve = dual_ouster_driver.pose_solve:main',
            'calibrate = dual_ouster_driver.calibrate:main',
            'flight_recorder = dual_ouster_driver.flight_recorder:main',
            'flight_report = dual_ouster_driver.flight_report:main',
            'verify_pose = dual_ouster_driver.verify_pose:main',
            'tilt_check = dual_ouster_driver.tilt_check:main',
        ],
    },

    zip_safe=True,

    maintainer='drone',
    maintainer_email='drone@example.com',

    description='Dual Ouster OS1-128 driver launch',

    license='Apache-2.0',
)
