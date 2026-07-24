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
    enable_steering_motor = LaunchConfiguration('enable_steering_motor')
    steering_soft_start_stop = LaunchConfiguration('steering_soft_start_stop')
    steering_soft_start_stop_rate_per_s = LaunchConfiguration('steering_soft_start_stop_rate_per_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Start the lidar launch file.',
        ),
        DeclareLaunchArgument(
            'enable_motor',
            default_value='true',
            description='Start the drive motor launch instance.',
        ),
        DeclareLaunchArgument(
            'enable_steering_motor',
            default_value='false',
            description='Start a second motor instance for steering.',
        ),
        DeclareLaunchArgument(
            'steering_soft_start_stop',
            default_value='false',
            description='Enable soft start/stop on steering motor.',
        ),
        DeclareLaunchArgument(
            'steering_soft_start_stop_rate_per_s',
            default_value='0.4',
            description='Steering ramp rate in command units per second when soft ramp is enabled.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch)),
            condition=IfCondition(enable_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(motor_launch)),
            condition=IfCondition(enable_motor),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(motor_launch)),
            launch_arguments={
                'node_name': 'steering_motor_node',
                'pin_pwm': '12',
                'pin_in1': '20',
                'pin_in2': '21',
                'topic_cmd': '/steering_motor_cmd',
                'soft_start_stop': steering_soft_start_stop,
                'soft_start_stop_rate_per_s': steering_soft_start_stop_rate_per_s,
                'debug_ramp': 'false',
            }.items(),
            condition=IfCondition(enable_steering_motor),
        ),
    ])
