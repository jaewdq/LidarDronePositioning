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
            glob('config/*.xml')
        ),
    ],

    install_requires=[
        'setuptools'
    ],


    zip_safe=True,

    maintainer='drone',
    maintainer_email='drone@example.com',

    description='Two Ouster LiDARs (OS0-SR) with PTP time and phase lock',

    license='Apache-2.0',
)
