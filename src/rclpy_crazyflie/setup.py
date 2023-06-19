from setuptools import setup
import os
from glob import glob

package_name = 'crazyflie_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'motion_commander'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitcraze',
    maintainer_email='bitcraze@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = crazyflie_server.crazyflie_server:main',
            'control = crazyflie_server.crazyflie_control:main'
        ],
    },
)
