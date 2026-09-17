from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('steering_sensor'),
        'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='steering_sensor',
            executable='steering_estimator_node',
            name='steering_estimator',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='steering_sensor',
            executable='steering_controller_node',
            name='steering_controller',
            output='screen',
            parameters=[params],
        ),
    ])
