#!/usr/bin/env python3
"""
Closed-loop steering controller.

Reads the measured steering angle (from steering_estimator_node) and a
target steering command, and drives the steering motor to close the gap.

Topics:
  Subscribe: /steering_angle      (std_msgs/Float32, -1..1, measured)
  Subscribe: /steering_cmd        (std_msgs/Float32, -1..1, target)
  Publish:   /steering_motor_cmd  (std_msgs/Float32, -1..1)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteeringController(Node):

    def __init__(self):
        super().__init__('steering_controller')

        self.declare_parameter('kp', 1.5)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_watchdog_timeout_s', 1.0)
        self.declare_parameter('measured_watchdog_timeout_s', 0.5)
        # +1.0 or -1.0 — flips output polarity to match actual wiring.
        # Measured 2026-09-17: increasing /steering_motor_cmd moves the wheel
        # towards -1 on /steering_angle, so the two are inverted here.
        self.declare_parameter('motor_sign', -1.0)
        # Below this |target-measured|, output 0 instead of dithering on lidar noise.
        self.declare_parameter('error_deadzone', 0.04)

        self.kp = self.get_parameter('kp').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.cmd_watchdog_timeout_s = self.get_parameter('cmd_watchdog_timeout_s').value
        self.measured_watchdog_timeout_s = self.get_parameter('measured_watchdog_timeout_s').value
        self.motor_sign = self.get_parameter('motor_sign').value
        self.error_deadzone = self.get_parameter('error_deadzone').value

        self.target = 0.0
        self.measured = 0.0
        self.target_rx_s = None
        self.measured_rx_s = None

        self.create_subscription(Float32, '/steering_angle', self._on_measured, 10)
        self.create_subscription(Float32, '/steering_cmd', self._on_target, 10)
        self.motor_pub = self.create_publisher(Float32, '/steering_motor_cmd', 10)

        self.create_timer(1.0 / self.control_rate_hz, self._on_timer)

        self.get_logger().info('Steering controller started, kp=%.3f' % self.kp)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_measured(self, msg: Float32):
        self.measured = msg.data
        self.measured_rx_s = self._now_s()

    def _on_target(self, msg: Float32):
        self.target = msg.data
        self.target_rx_s = self._now_s()

    def _on_timer(self):
        now = self._now_s()

        # No feedback yet, or feedback gone stale — refuse to drive open-loop.
        if self.measured_rx_s is None or (now - self.measured_rx_s) > self.measured_watchdog_timeout_s:
            self.motor_pub.publish(Float32(data=0.0))
            return

        # No recent target — hold center rather than the last command.
        target = self.target
        if self.target_rx_s is None or (now - self.target_rx_s) > self.cmd_watchdog_timeout_s:
            target = 0.0

        error = target - self.measured
        if abs(error) < self.error_deadzone:
            output = 0.0
        else:
            output = max(-1.0, min(1.0, self.motor_sign * self.kp * error))
        self.motor_pub.publish(Float32(data=output))


def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
