#!/usr/bin/env python3
"""Simple GPS waypoint steering helper.

Publishes:
  - /heading  (std_msgs/Float32, radians, robot heading estimate from /vel)
  - /gps_steering_bias (std_msgs/Float32, normalized -1..1, waypoint heading bias)

Reads:
  - /fix       (sensor_msgs/NavSatFix)
  - /vel       (geometry_msgs/TwistStamped)
  - /road_follower/error (optional, for blending when the road follower is active)

This node intentionally does NOT publish directly to /steering_cmd by default,
so it stays safe and does not race with the obstacle avoider / road-following logic.
It is meant to be blended upstream or used as a bias source during testing.
"""

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


class GpsWaypointNode(Node):
    def __init__(self):
        super().__init__('gps_waypoint_node')

        self.declare_parameter('target_file', '/home/dano/robot-ros2/gps_target.txt')
        self.declare_parameter('target_lat', 50.105009)
        self.declare_parameter('target_lon', 14.426937)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('speed_threshold_mps', 0.25)
        self.declare_parameter('tolerance_m', 2.0)
        self.declare_parameter('heading_gain', 1.0)
        self.declare_parameter('max_bias', 1.0)
        self.declare_parameter('publish_to_steering_cmd', False)

        self.target_file = str(self.get_parameter('target_file').value)
        self.target_lat = float(self.get_parameter('target_lat').value)
        self.target_lon = float(self.get_parameter('target_lon').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.speed_threshold_mps = float(self.get_parameter('speed_threshold_mps').value)
        self.tolerance_m = float(self.get_parameter('tolerance_m').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.max_bias = float(self.get_parameter('max_bias').value)
        self.publish_to_steering_cmd = bool(self.get_parameter('publish_to_steering_cmd').value)

        self.fix_msg = None
        self.vel_msg = None
        self.last_heading = 0.0
        self.last_fix = None
        self.target_loaded = False
        self.origin_lat = None
        self.origin_lon = None

        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.vel_sub = self.create_subscription(TwistStamped, '/vel', self.vel_callback, 10)
        self.heading_pub = self.create_publisher(Float32, '/heading', 10)
        self.bias_pub = self.create_publisher(Float32, '/gps_steering_bias', 10)
        if self.publish_to_steering_cmd:
            self.steering_pub = self.create_publisher(Float32, '/steering_cmd', 10)
        else:
            self.steering_pub = None

        self._read_target_file_if_present()
        self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            'GPS waypoint node ready: target=(%.6f, %.6f), tolerance=%.1fm, gain=%.2f',
            self.target_lat, self.target_lon, self.tolerance_m, self.heading_gain)

    def _read_target_file_if_present(self):
        p = Path(self.target_file)
        if not p.exists():
            self.get_logger().warning('Target file not found: %s', self.target_file)
            return
        try:
            lines = [line.strip() for line in p.read_text().splitlines() if line.strip()]
            for line in lines:
                if line.startswith('#'):
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) >= 2:
                    self.target_lat = float(parts[0])
                    self.target_lon = float(parts[1])
                    self.target_loaded = True
                    self.get_logger().info('Loaded target from %s: %.6f, %.6f', self.target_file, self.target_lat, self.target_lon)
                    return
        except Exception as exc:  # pragma: no cover
            self.get_logger().error('Failed to parse target file %s: %s', self.target_file, exc)

    def _wrap_pi(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _enu_xy(self, lat_deg: float, lon_deg: float, lat0: float, lon0: float):
        earth_radius_m = 6371000.0
        lat0_rad = math.radians(lat0)
        x = (math.radians(lon_deg - lon0)) * earth_radius_m * math.cos(lat0_rad)
        y = (math.radians(lat_deg - lat0)) * earth_radius_m
        return x, y

    def fix_callback(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        self.fix_msg = msg
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = float(msg.latitude)
            self.origin_lon = float(msg.longitude)

    def vel_callback(self, msg: TwistStamped):
        self.vel_msg = msg

    def _estimate_heading(self):
        if self.vel_msg is None:
            return None
        vx = float(self.vel_msg.twist.linear.x)
        vy = float(self.vel_msg.twist.linear.y)
        speed = math.hypot(vx, vy)
        if speed < self.speed_threshold_mps:
            return self.last_heading
        heading = math.atan2(vx, vy)
        self.last_heading = heading
        return heading

    def _distance_to_target_m(self, robot_lat: float, robot_lon: float):
        if self.target_loaded is False:
            return None
        dx, dy = self._enu_xy(robot_lat, robot_lon, self.origin_lat, self.origin_lon)
        tx, ty = self._enu_xy(self.target_lat, self.target_lon, self.origin_lat, self.origin_lon)
        return math.hypot(tx - dx, ty - dy)

    def timer_callback(self):
        if self.fix_msg is None:
            return
        if self.origin_lat is None or self.origin_lon is None:
            return

        robot_lat = float(self.fix_msg.latitude)
        robot_lon = float(self.fix_msg.longitude)
        tx, ty = self._enu_xy(self.target_lat, self.target_lon, self.origin_lat, self.origin_lon)
        dx, dy = self._enu_xy(robot_lat, robot_lon, self.origin_lat, self.origin_lon)

        target_bearing = math.atan2(tx - dx, ty - dy)
        heading = self._estimate_heading()
        if heading is None:
            return

        heading_error = self._wrap_pi(target_bearing - heading)
        steering_bias = math.tanh(heading_error / (math.radians(30.0)))
        steering_bias = max(-self.max_bias, min(self.max_bias, self.heading_gain * steering_bias))

        msg = Float32()
        msg.data = float(steering_bias)
        self.bias_pub.publish(msg)

        heading_msg = Float32()
        heading_msg.data = float(heading)
        self.heading_pub.publish(heading_msg)

        if self.steering_pub is not None:
            self.steering_pub.publish(msg)

        distance = math.hypot(tx - dx, ty - dy)
        if distance < self.tolerance_m:
            self.get_logger().info('Target reached: distance=%.2fm', distance)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
