from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dano',
    maintainer_email='dano@todo.todo',
    description='Motor driver nodes for DFRobot DRI0042 H-bridge control.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_node = motor_driver.motor_node:main',
        ],
    },
)
