from setuptools import setup
import os
from glob import glob

package_name = 'avatar_robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agrobot',
    maintainer_email='sahand.ajami@gmail.com',
    description='Agrobot chassis base: Modbus RTU driver, dead-reckoning odometry, '
                'path trace and keyboard teleop for the T3/T13/T17E variants',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_base_node = avatar_robot_base.robot_base_node:main',
            'odom_calculation = avatar_robot_base.odom_calculation:main',
            'path_publisher = avatar_robot_base.path_publisher:main',
            'teleop_keyboard = avatar_robot_base.teleop_keyboard:main',
        ],
    },
)
