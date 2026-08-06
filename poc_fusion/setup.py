import os
from glob import glob

from setuptools import setup

package_name = 'poc_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.lib'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan Yang',
    maintainer_email='yangethan2006@gmail.com',
    description='POC: fuses the LanderPi arm-mounted Aurora depth camera with the '
                 'LD19 LiDAR into one Nav2 local costmap.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_preprocess_node = poc_fusion.depth_preprocess_node:main',
            'costmap_stop_monitor_node = poc_fusion.costmap_stop_monitor_node:main',
            'latency_recorder_node = poc_fusion.latency_recorder_node:main',
        ],
    },
)
