from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_driver',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{
                'gpio_chip': '/dev/gpiochip0',
                'pin_pwm': 18,
                'pin_in1': 23,
                'pin_in2': 24,
                'pwm_frequency_hz': 1000.0,
                'watchdog_timeout_s': 2.0,
                'control_rate_hz': 50.0,
                'topic_cmd': '/motor_cmd',
                'invert_direction': False,
                'soft_start_stop': True,
                'soft_start_stop_rate_per_s': 0.1,
                'debug_ramp': False,
                'debug_ramp_period_s': 0.5,
            }],
        )
    ])
