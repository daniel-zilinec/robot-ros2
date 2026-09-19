#!/usr/bin/env python3
"""Simple GPS waypoint steering helper.

Publishes:
  - /heading  (std_msgs/Float32, radians, robot heading estimate from /vel)
  - /gps_steering_bias (std_msgs/Float32, normalized -1..1, waypoint heading bias)
    - /gps_osm/intersection_active (std_msgs/Bool)
    - /gps_osm/intersection_distance_m (std_msgs/Float32)
    - /gps_osm/heading_error (std_msgs/Float32, radians)
    - /gps_osm/target_distance_m (std_msgs/Float32)

Reads:
  - /fix       (sensor_msgs/NavSatFix)
  - /vel       (geometry_msgs/TwistStamped)
  - /road_follower/error (optional, for blending when the road follower is active)

This node intentionally does NOT publish directly to /steering_cmd by default,
so it stays safe and does not race with the obstacle avoider / road-following logic.
It is meant to be blended upstream or used as a bias source during testing.
"""

import math
import gzip
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32


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
        self.declare_parameter('osm_file', '/home/dano/robot-ros2/stromovka.2026.trimmed.osm.gz')
        self.declare_parameter('intersection_radius_m', 8.0)
        self.declare_parameter('minimum_intersection_degree', 3)

        self.target_file = str(self.get_parameter('target_file').value)
        self.target_lat = float(self.get_parameter('target_lat').value)
        self.target_lon = float(self.get_parameter('target_lon').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.speed_threshold_mps = float(self.get_parameter('speed_threshold_mps').value)
        self.tolerance_m = float(self.get_parameter('tolerance_m').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.max_bias = float(self.get_parameter('max_bias').value)
        self.publish_to_steering_cmd = bool(self.get_parameter('publish_to_steering_cmd').value)
        self.osm_file = str(self.get_parameter('osm_file').value)
        self.intersection_radius_m = float(self.get_parameter('intersection_radius_m').value)
        self.minimum_intersection_degree = int(self.get_parameter('minimum_intersection_degree').value)

        self.fix_msg = None
        self.vel_msg = None
        self.last_heading = 0.0
        self.last_fix = None
        self.target_loaded = False
        self.origin_lat = None
        self.origin_lon = None
        self.intersections = []
        self.was_at_intersection = False

        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.vel_sub = self.create_subscription(TwistStamped, '/vel', self.vel_callback, 10)
        self.heading_pub = self.create_publisher(Float32, '/heading', 10)
        self.bias_pub = self.create_publisher(Float32, '/gps_steering_bias', 10)
        self.intersection_active_pub = self.create_publisher(
            Bool, '/gps_osm/intersection_active', 10)
        self.intersection_distance_pub = self.create_publisher(
            Float32, '/gps_osm/intersection_distance_m', 10)
        self.heading_error_pub = self.create_publisher(
            Float32, '/gps_osm/heading_error', 10)
        self.target_distance_pub = self.create_publisher(
            Float32, '/gps_osm/target_distance_m', 10)
        if self.publish_to_steering_cmd:
            self.steering_pub = self.create_publisher(Float32, '/steering_cmd', 10)
        else:
            self.steering_pub = None

        self._read_target_file_if_present()
        self._load_osm_intersections()
        self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            f'GPS waypoint node ready: target=({self.target_lat:.6f}, '
            f'{self.target_lon:.6f}), tolerance={self.tolerance_m:.1f}m, '
            f'gain={self.heading_gain:.2f}')

    def _load_osm_intersections(self):
        """Infer junctions from highway-way topology in the offline OSM map."""
        path = Path(self.osm_file)
        if not path.exists():
            self.get_logger().warning(f'OSM map not found: {self.osm_file}')
            return
        try:
            with gzip.open(path, 'rb') as stream:
                root = ET.parse(stream).getroot()
            nodes = {
                element.attrib['id']: (
                    float(element.attrib['lat']),
                    float(element.attrib['lon']))
                for element in root.findall('node')
            }
            neighbors = {}
            for way in root.findall('way'):
                tags = {tag.attrib.get('k'): tag.attrib.get('v')
                        for tag in way.findall('tag')}
                if tags.get('highway') in ('steps', 'elevator'):
                    continue
                if 'highway' not in tags:
                    continue
                refs = [nd.attrib['ref'] for nd in way.findall('nd')]
                for first, second in zip(refs, refs[1:]):
                    if first in nodes and second in nodes:
                        neighbors.setdefault(first, set()).add(second)
                        neighbors.setdefault(second, set()).add(first)
            self.intersections = [nodes[node_id] for node_id, adjacent in neighbors.items()
                                  if len(adjacent) >= self.minimum_intersection_degree]
            self.get_logger().info(
                f'Loaded {len(self.intersections)} OSM intersections from {self.osm_file}')
        except (OSError, ET.ParseError, KeyError, ValueError) as exc:
            self.get_logger().error(f'Failed to parse OSM map {self.osm_file}: {exc}')

    def _read_target_file_if_present(self):
        p = Path(self.target_file)
        if not p.exists():
            self.get_logger().warning(f'Target file not found: {self.target_file}')
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
                    self.get_logger().info(
                        f'Loaded target from {self.target_file}: '
                        f'{self.target_lat:.6f}, {self.target_lon:.6f}')
                    return
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed to parse target file {self.target_file}: {exc}')

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

    def _distance_to_nearest_intersection_m(self, robot_lat: float, robot_lon: float):
        if not self.intersections:
            return float('inf')
        return min(
            math.hypot(*self._enu_xy(lat, lon, robot_lat, robot_lon))
            for lat, lon in self.intersections)

    def timer_callback(self):
        if self.fix_msg is None:
            return
        if self.origin_lat is None or self.origin_lon is None:
            return

        robot_lat = float(self.fix_msg.latitude)
        robot_lon = float(self.fix_msg.longitude)
        tx, ty = self._enu_xy(self.target_lat, self.target_lon, self.origin_lat, self.origin_lon)
        dx, dy = self._enu_xy(robot_lat, robot_lon, self.origin_lat, self.origin_lon)
        distance = math.hypot(tx - dx, ty - dy)
        distance_to_intersection = self._distance_to_nearest_intersection_m(
            robot_lat, robot_lon)
        at_intersection = distance_to_intersection <= self.intersection_radius_m

        self.intersection_active_pub.publish(Bool(data=at_intersection))
        self.intersection_distance_pub.publish(
            Float32(data=float(distance_to_intersection)))
        self.target_distance_pub.publish(Float32(data=float(distance)))

        target_bearing = math.atan2(tx - dx, ty - dy)
        heading = self._estimate_heading()
        if heading is None:
            self.heading_error_pub.publish(Float32(data=0.0))
            self.bias_pub.publish(Float32(data=0.0))
            return

        heading_error = self._wrap_pi(target_bearing - heading)
        self.heading_error_pub.publish(Float32(data=float(heading_error)))
        if at_intersection != self.was_at_intersection:
            gate_state = 'ACTIVE' if at_intersection else 'inactive'
            self.get_logger().info(
                f'OSM intersection gate: {gate_state} '
                f'(distance={distance_to_intersection:.1f}m)')
            self.was_at_intersection = at_intersection

        steering_bias = math.tanh(heading_error / (math.radians(30.0)))
        if not at_intersection:
            steering_bias = 0.0
        steering_bias = max(-self.max_bias, min(self.max_bias, self.heading_gain * steering_bias))

        msg = Float32()
        msg.data = float(steering_bias)
        self.bias_pub.publish(msg)

        heading_msg = Float32()
        heading_msg.data = float(heading)
        self.heading_pub.publish(heading_msg)

        if self.steering_pub is not None:
            self.steering_pub.publish(msg)

        if distance < self.tolerance_m:
            self.get_logger().info(f'Target reached: distance={distance:.2f}m')


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
