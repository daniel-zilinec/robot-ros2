from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('obstacle_avoidance'),
        'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='avoider_node',
            name='ackermann_obstacle_avoider',
            output='screen',
            parameters=[params],
        ),
    ])
