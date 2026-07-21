from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    motor_launch = Path(get_package_share_directory('motor_driver')) / 'launch' / 'motor_launch.py'
    lidar_launch = Path(get_package_share_directory('lidar')) / 'launch' / 'lidar_launch.py'

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_motor = LaunchConfiguration('enable_motor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Start the lidar launch file.',
        ),
        DeclareLaunchArgument(
            'enable_motor',
            default_value='true',
            description='Start the motor launch file.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch)),
            condition=IfCondition(enable_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(motor_launch)),
            condition=IfCondition(enable_motor),
        ),
    ])
