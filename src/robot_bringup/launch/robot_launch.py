from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    motor_launch = Path(get_package_share_directory('motor_driver')) / 'launch' / 'motor_launch.py'
    lidar_launch = Path(get_package_share_directory('lidar')) / 'launch' / 'lidar_launch.py'
    gamepad_launch = Path(get_package_share_directory('gamepad_teleop')) / 'launch' / 'gamepad_launch.py'

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_traction_motor = LaunchConfiguration('enable_traction_motor')
    enable_steering_motor = LaunchConfiguration('enable_steering_motor')
    enable_gamepad = LaunchConfiguration('enable_gamepad')
    enable_camera = LaunchConfiguration('enable_camera')
    gamepad_device_path = LaunchConfiguration('gamepad_device_path')
    gamepad_axis_traction = LaunchConfiguration('gamepad_axis_traction')
    gamepad_axis_steering = LaunchConfiguration('gamepad_axis_steering')
    gamepad_deadman_button = LaunchConfiguration('gamepad_deadman_button')
    steering_soft_start_stop = LaunchConfiguration('steering_soft_start_stop')
    steering_soft_start_stop_rate_per_s = LaunchConfiguration('steering_soft_start_stop_rate_per_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Start the lidar launch file.',
        ),
        DeclareLaunchArgument(
            'enable_traction_motor',
            default_value='true',
            description='Start the traction motor launch instance.',
        ),
        DeclareLaunchArgument(
            'enable_steering_motor',
            default_value='true',
            description='Start a second motor instance for steering.',
        ),
        DeclareLaunchArgument(
            'enable_gamepad',
            default_value='true',
            description='Start gamepad teleop node for drive and steering topics.',
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Start the USB camera node.',
        ),
        DeclareLaunchArgument(
            'gamepad_device_path',
            default_value='/dev/input/js0',
            description='Linux joystick device path for gamepad teleop.',
        ),
        DeclareLaunchArgument(
            'gamepad_axis_traction',
            default_value='1',
            description='Joystick axis index for traction command.',
        ),
        DeclareLaunchArgument(
            'gamepad_axis_steering',
            default_value='3',
            description='Joystick axis index for steering command.',
        ),
        DeclareLaunchArgument(
            'gamepad_deadman_button',
            default_value='-1',
            description='Deadman button index; -1 disables deadman requirement.',
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
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'best_effort_qos_topic_whitelist': ['.*compressed.*', '.*image_raw.*'],
                'message_backlog_size': 5,
                'send_buffer_limit': 500000,   # 500 KB ≈ 8 frames
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch)),
            condition=IfCondition(enable_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(motor_launch)),
            launch_arguments={
                'node_name': 'traction_motor_node',
                'topic_cmd': '/traction_motor_cmd',
            }.items(),
            condition=IfCondition(enable_traction_motor),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gamepad_launch)),
            launch_arguments={
                'device_path': gamepad_device_path,
                'topic_traction': '/traction_motor_cmd',
                'topic_steering': '/steering_motor_cmd',
                'axis_traction': gamepad_axis_traction,
                'axis_steering': gamepad_axis_steering,
                'deadman_button': gamepad_deadman_button,
            }.items(),
            condition=IfCondition(enable_gamepad),
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'publish_format': 'compressed',
                'jpeg_quality': 50,

                # Reliability overrides
                'qos_overrides./image_raw.publisher.reliability': 'best_effort',
                'qos_overrides./image_raw/compressed.publisher.reliability': 'best_effort',

                # Durability overrides — VOLATILE = no cache, no replay
                'qos_overrides./image_raw.publisher.durability': 'volatile',
                'qos_overrides./image_raw/compressed.publisher.durability': 'volatile',

                # History: only latest frame matters
                'qos_overrides./image_raw.publisher.history': 'keep_last',
                'qos_overrides./image_raw/compressed.publisher.history': 'keep_last',
                'qos_overrides./image_raw.publisher.depth': 1,
                'qos_overrides./image_raw/compressed.publisher.depth': 1,
            }],
            condition=IfCondition(enable_camera),
        ),

        # --- GNSS ---
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver_node',
            parameters=[{
                'port': '/dev/ttyUSB_gnss',
                'baud': 9600,
                'frame_id': 'gnss_link',
                'use_GNSS_time': False,
                'useRMC': False
            }],
            output='screen'
        ),

    ])
