from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dano',
    maintainer_email='dano@robot.local',
    description='Obstacle avoidance for Ackermann robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avoider_node = obstacle_avoidance.avoider_node:main',
        ],
    },
)
