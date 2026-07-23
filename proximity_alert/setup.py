from setuptools import setup

package_name = 'proximity_alert'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan Yang',
    maintainer_email='yangethan2006@gmail.com',
    description='LiDAR-based ADAS proximity alert: A-to-B path driving with reactive '
                 'obstacle avoidance, and per-trial transit time / odometry / slippage logging.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_tracker = proximity_alert.path_tracker:main',
            'trial_logger = proximity_alert.trial_logger:main',
            'decision_logger = proximity_alert.decision_logger:main',
            'floor_test_reconcile = proximity_alert.floor_test_reconcile:main',
            'scan_trace_logger = proximity_alert.scan_trace_logger:main',
        ],
    },
)
