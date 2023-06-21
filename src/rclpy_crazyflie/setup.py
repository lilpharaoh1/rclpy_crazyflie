from setuptools import setup
import os
from glob import glob

package_name = 'rclpy_crazyflie'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'crazyflie_server', 'crazyflie_client', 'motion_commander'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob(os.path.join('data', '*.json'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emran Yasser Moustafa',
    maintainer_email='moustafe@tcd.ie',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = crazyflie_server.crazyflie_server:main',
            'control = crazyflie_server.crazyflie_control:main',
            'log = crazyflie_server.crazyflie_log:main',
            'client = crazyflie_client.crazyflie_client:main'
        ],
    },
)
