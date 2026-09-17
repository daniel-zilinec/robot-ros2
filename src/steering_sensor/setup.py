from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'steering_sensor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dano',
    maintainer_email='dano@robot.local',
    description='Lidar-based steering angle estimation and closed-loop steering control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'steering_estimator_node = steering_sensor.steering_estimator_node:main',
            'steering_controller_node = steering_sensor.steering_controller_node:main',
        ],
    },
)
