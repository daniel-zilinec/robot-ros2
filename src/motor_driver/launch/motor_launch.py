from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    node_name = LaunchConfiguration('node_name')
    gpio_chip = LaunchConfiguration('gpio_chip')
    pin_pwm = LaunchConfiguration('pin_pwm')
    pin_in1 = LaunchConfiguration('pin_in1')
    pin_in2 = LaunchConfiguration('pin_in2')
    pwm_frequency_hz = LaunchConfiguration('pwm_frequency_hz')
    watchdog_timeout_s = LaunchConfiguration('watchdog_timeout_s')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    topic_cmd = LaunchConfiguration('topic_cmd')
    invert_direction = LaunchConfiguration('invert_direction')
    soft_start_stop = LaunchConfiguration('soft_start_stop')
    soft_start_stop_rate_per_s = LaunchConfiguration('soft_start_stop_rate_per_s')
    debug_ramp = LaunchConfiguration('debug_ramp')
    debug_ramp_period_s = LaunchConfiguration('debug_ramp_period_s')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='motor_node'),
        DeclareLaunchArgument('gpio_chip', default_value='/dev/gpiochip0'),
        DeclareLaunchArgument('pin_pwm', default_value='18'),
        DeclareLaunchArgument('pin_in1', default_value='23'),
        DeclareLaunchArgument('pin_in2', default_value='24'),
        DeclareLaunchArgument('pwm_frequency_hz', default_value='1000.0'),
        DeclareLaunchArgument('watchdog_timeout_s', default_value='2.0'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('topic_cmd', default_value='/motor_cmd'),
        DeclareLaunchArgument('invert_direction', default_value='false'),
        DeclareLaunchArgument('soft_start_stop', default_value='true'),
        DeclareLaunchArgument('soft_start_stop_rate_per_s', default_value='1'),
        DeclareLaunchArgument('debug_ramp', default_value='false'),
        DeclareLaunchArgument('debug_ramp_period_s', default_value='0.5'),
        Node(
            package='motor_driver',
            executable='motor_node',
            name=node_name,
            output='screen',
            parameters=[{
                'gpio_chip': gpio_chip,
                'pin_pwm': ParameterValue(pin_pwm, value_type=int),
                'pin_in1': ParameterValue(pin_in1, value_type=int),
                'pin_in2': ParameterValue(pin_in2, value_type=int),
                'pwm_frequency_hz': ParameterValue(pwm_frequency_hz, value_type=float),
                'watchdog_timeout_s': ParameterValue(watchdog_timeout_s, value_type=float),
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'topic_cmd': topic_cmd,
                'invert_direction': ParameterValue(invert_direction, value_type=bool),
                'soft_start_stop': ParameterValue(soft_start_stop, value_type=bool),
                'soft_start_stop_rate_per_s': ParameterValue(soft_start_stop_rate_per_s, value_type=float),
                'debug_ramp': ParameterValue(debug_ramp, value_type=bool),
                'debug_ramp_period_s': ParameterValue(debug_ramp_period_s, value_type=float),
            }],
        )
    ])
