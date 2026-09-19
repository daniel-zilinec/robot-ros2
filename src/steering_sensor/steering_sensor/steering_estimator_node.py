#!/usr/bin/env python3
"""
Estimates the Ackermann front-wheel steering angle from the lidar scan.

The lidar sits between the front wheels and sees the wheel/steering-linkage
as a close-range blob whose angle shifts with the steering position. This
node isolates that blob and maps its angle to a normalized steering value.

Topics:
  Subscribe: /scan_c1                       (sensor_msgs/LaserScan)
  Publish:   /steering_angle                (std_msgs/Float32, -1..1, +1 = full left)
  Publish:   /steering_sensor/debug_raw_angle_deg (std_msgs/Float32, degrees)
                                             — for calibration only
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class SteeringEstimator(Node):

    def __init__(self):
        super().__init__('steering_estimator')

        # ── Parameters ────────────────────────────────────────────────
        # Window matched on signed angle — the wheel blob measured by ruler
        # sits on one side only (see steering_feedback_plan.md); using |angle|
        # instead pulls in the opposite side too and cancels the signal.
        self.declare_parameter('blob_angle_min_deg', -155.0)
        self.declare_parameter('blob_angle_max_deg', -105.0)
        self.declare_parameter('blob_range_min_m', 0.15)
        self.declare_parameter('blob_range_max_m', 0.40)
        # Raw corrected blob angle (degrees) measured at each steering lock —
        # fill these in during calibration (see steering_feedback_plan.md).
        self.declare_parameter('calib_left_angle_deg', 20.0)
        self.declare_parameter('calib_center_angle_deg', 0.0)
        self.declare_parameter('calib_right_angle_deg', -20.0)

        self.blob_angle_min = math.radians(self.get_parameter('blob_angle_min_deg').value)
        self.blob_angle_max = math.radians(self.get_parameter('blob_angle_max_deg').value)
        self.blob_range_min = self.get_parameter('blob_range_min_m').value
        self.blob_range_max = self.get_parameter('blob_range_max_m').value
        self.calib_left   = self.get_parameter('calib_left_angle_deg').value
        self.calib_center = self.get_parameter('calib_center_angle_deg').value
        self.calib_right  = self.get_parameter('calib_right_angle_deg').value

        # ── ROS 2 interfaces ───────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_c1',
            self.scan_callback,
            qos_profile_sensor_data)

        self.angle_pub = self.create_publisher(Float32, '/steering_angle', 10)
        self.debug_pub = self.create_publisher(
            Float32, '/steering_sensor/debug_raw_angle_deg', 10)

        self.get_logger().info('Steering estimator started')

    @staticmethod
    def corrected_angle(raw_angle):
        """Same TF correction used in obstacle_avoidance: 180deg yaw + roll."""
        return math.atan2(math.sin(-raw_angle + math.pi), math.cos(-raw_angle + math.pi))

    def scan_callback(self, msg: LaserScan):
        angles = []
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and self.blob_range_min <= r <= self.blob_range_max):
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            angle = self.corrected_angle(raw_angle)
            if self.blob_angle_min <= angle <= self.blob_angle_max:
                angles.append(angle)

        if not angles:
            self.get_logger().warning(
                'No blob points in window — steering angle not published', throttle_duration_sec=2.0)
            return

        raw_angle_deg = math.degrees(sum(angles) / len(angles))
        self.debug_pub.publish(Float32(data=raw_angle_deg))

        steering = self._map_to_normalized(raw_angle_deg)
        self.angle_pub.publish(Float32(data=steering))

    def _map_to_normalized(self, raw_angle_deg):
        """Piecewise-linear map: calib_right -> -1, calib_center -> 0, calib_left -> +1.

        Handles calib_left/calib_right being either above or below calib_center
        (angle doesn't necessarily increase towards "left").
        """
        offset = raw_angle_deg - self.calib_center
        towards_left = self.calib_left - self.calib_center
        towards_right = self.calib_right - self.calib_center

        # Pick whichever side offset is heading towards (same sign as offset).
        if offset == 0 or (towards_left != 0 and offset * towards_left > 0):
            span = towards_left
            sign = 1.0
        else:
            span = towards_right
            sign = -1.0

        frac = 0.0 if span == 0 else offset / span
        return sign * max(0.0, min(1.0, frac))


def main(args=None):
    rclpy.init(args=args)
    node = SteeringEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
