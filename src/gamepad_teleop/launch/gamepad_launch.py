from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_path = LaunchConfiguration('device_path')
    topic_traction = LaunchConfiguration('topic_traction')
    topic_steering = LaunchConfiguration('topic_steering')
    axis_traction = LaunchConfiguration('axis_traction')
    axis_steering = LaunchConfiguration('axis_steering')
    invert_traction = LaunchConfiguration('invert_traction')
    invert_steering = LaunchConfiguration('invert_steering')
    max_traction = LaunchConfiguration('max_traction')
    max_steering = LaunchConfiguration('max_steering')
    deadzone = LaunchConfiguration('deadzone')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    deadman_button = LaunchConfiguration('deadman_button')

    return LaunchDescription([
        DeclareLaunchArgument('device_path', default_value='/dev/input/js0'),
        DeclareLaunchArgument('topic_traction', default_value='/traction_motor_cmd'),
        DeclareLaunchArgument('topic_steering', default_value='/steering_motor_cmd'),
        DeclareLaunchArgument('axis_traction', default_value='1'),
        DeclareLaunchArgument('axis_steering', default_value='0'),
        DeclareLaunchArgument('invert_traction', default_value='true'),
        DeclareLaunchArgument('invert_steering', default_value='false'),
        DeclareLaunchArgument('max_traction', default_value='0.6'),
        DeclareLaunchArgument('max_steering', default_value='0.8'),
        DeclareLaunchArgument('deadzone', default_value='0.08'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('deadman_button', default_value='-1'),
        Node(
            package='gamepad_teleop',
            executable='gamepad_node',
            name='gamepad_teleop_node',
            output='screen',
            parameters=[{
                'device_path': device_path,
                'topic_traction': topic_traction,
                'topic_steering': topic_steering,
                'axis_traction': ParameterValue(axis_traction, value_type=int),
                'axis_steering': ParameterValue(axis_steering, value_type=int),
                'invert_traction': ParameterValue(invert_traction, value_type=bool),
                'invert_steering': ParameterValue(invert_steering, value_type=bool),
                'max_traction': ParameterValue(max_traction, value_type=float),
                'max_steering': ParameterValue(max_steering, value_type=float),
                'deadzone': ParameterValue(deadzone, value_type=float),
                'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                'deadman_button': ParameterValue(deadman_button, value_type=int),
            }],
        )
    ])
