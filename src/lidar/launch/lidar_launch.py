from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_c1',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_c1'
            ]
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node_c1',
            namespace='lidar_c1',
            parameters=[{
                'serial_port': '/dev/ttyUSB_lidar_c1',
                'serial_baudrate': 460800,
                'frame_id': 'laser_c1',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('/lidar_c1/scan', '/scan_c1')],
            output='screen'
        ),
    ])
