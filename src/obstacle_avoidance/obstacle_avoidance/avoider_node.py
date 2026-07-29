#!/usr/bin/env python3
"""
Ackermann obstacle avoidance node for open-loop DC steering motor.
No /cmd_vel — uses /traction_motor_cmd and /steering_motor_cmd directly.

Topics:
  Subscribe: /scan_c1       (sensor_msgs/LaserScan)
  Publish:   /traction_motor_cmd  (std_msgs/Float32, range -1.0 to +1.0)
  Publish:   /steering_motor_cmd  (std_msgs/Float32, range -1.0 to +1.0)
             +1.0 = full left, -1.0 = full right, 0.0 = neutral power

States:
  DRIVING      → forward at cruise speed, steering neutral (0.0)
  AVOIDING     → forward at avoid speed, full steering lock one direction
  STRAIGHTENING→ forward at avoid speed, full steering lock OTHER direction
                 for same duration as AVOIDING (with correction factor)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from enum import Enum


class State(Enum):
    DRIVING       = 'DRIVING'
    AVOIDING      = 'AVOIDING'
    STRAIGHTENING = 'STRAIGHTENING'


class AckermannObstacleAvoider(Node):

    def __init__(self):
        super().__init__('ackermann_obstacle_avoider')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('forward_power',       0.40)  # cruise power (0–1)
        self.declare_parameter('avoid_power',         0.30)  # power during avoid/straighten
        self.declare_parameter('steer_power',         1.00)  # full lock during avoid
        self.declare_parameter('front_half_angle',   45.0)   # degrees each side of fwd
        self.declare_parameter('stop_distance',       1.00)  # m — trigger avoidance
        self.declare_parameter('resume_distance',     1.40)  # m — hysteresis
        self.declare_parameter('avoid_duration',      2.00)  # s — how long to turn
        self.declare_parameter('straighten_factor',   0.5)  # × avoid_duration for return
        # straighten_factor < 1.0 because motor doesn't stop instantly

        self.forward_power    = self.get_parameter('forward_power').value
        self.avoid_power      = self.get_parameter('avoid_power').value
        self.steer_power      = self.get_parameter('steer_power').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle').value)
        self.stop_distance    = self.get_parameter('stop_distance').value
        self.resume_distance  = self.get_parameter('resume_distance').value
        self.avoid_duration   = self.get_parameter('avoid_duration').value
        self.straighten_factor = self.get_parameter('straighten_factor').value

        # ── State ──────────────────────────────────────────────────────
        self.state            = State.DRIVING
        self.steer_direction  = 0.0   # +1.0 = left, -1.0 = right (locked per episode)
        self.state_start_time = None  # rclpy.Time when current timed state began

        # ── ROS 2 interfaces ───────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_c1',
            self.scan_callback,
            qos_profile_sensor_data)          # best-effort QoS for sensor data

        self.traction_pub = self.create_publisher(
            Float32, '/traction_motor_cmd', 10)
        self.steering_pub = self.create_publisher(
            Float32, '/steering_motor_cmd', 10)

        self.get_logger().info('Obstacle avoider started — State: DRIVING')

    # ── Helpers ──────────────────────────────────────────────────────────

    def get_sector_min(self, msg, angle_start_rad, angle_end_rad):
        """
        Minimum valid range in an angular sector.
        Applies the same correction as the static TF transform:
        - 180 deg yaw  (lidar mounted backwards)
        - 180 deg roll (lidar mounted upside down)
        Combined effect on scan angles: corrected = -raw + pi
        """
        ranges = []
        for i, r in enumerate(msg.ranges):
            raw_angle = msg.angle_min + i * msg.angle_increment
            # Apply same correction as TF: negate + shift 180°
            corrected_angle = -raw_angle + math.pi
            # Normalise to [-pi, +pi]
            corrected_angle = math.atan2(
                math.sin(corrected_angle),
                math.cos(corrected_angle))
            if angle_start_rad <= corrected_angle <= angle_end_rad:
                if math.isfinite(r) and r > 0.05:
                    ranges.append(r)
        return min(ranges) if ranges else float('inf')

    def elapsed(self, msg):
        """Seconds elapsed since state_start_time."""
        now = self.get_clock().now()
        return (now - self.state_start_time).nanoseconds / 1e9

    def publish(self, traction, steering):
        t = Float32(); t.data = float(traction)
        s = Float32(); s.data = float(steering)
        self.traction_pub.publish(t)
        self.steering_pub.publish(s)

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f'→ State: {new_state.value}')

    # ── Main callback ─────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):

        # ── Emergency stop ─────────────────────────────────────────────
        # Stop immediately if anything in the forward cone is closer than 0.30m.
        # Uses corrected angles — so car body (behind the sensor) is excluded.
        forward_close = []
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            corrected_angle = math.atan2(
                math.sin(-raw_angle + math.pi),
                math.cos(-raw_angle + math.pi))
            if -self.front_half_angle <= corrected_angle <= self.front_half_angle:
                forward_close.append(r)

        forward_min = min(forward_close, default=float('inf'))
        if forward_min < 0.30:
            self.publish(0.0, 0.0)
            self.get_logger().error(f'EMERGENCY STOP — {forward_min:.2f}m')
            return

    # ── Scan analysis ──────────────────────────────────────────────


        # ── Scan analysis ──────────────────────────────────────────────
        front_min = self.get_sector_min(
            msg, -self.front_half_angle, self.front_half_angle)
        left_min  = self.get_sector_min(
            msg, 0.0, self.front_half_angle)
        right_min = self.get_sector_min(
            msg, -self.front_half_angle, 0.0)

        # ── State machine ──────────────────────────────────────────────

        if self.state == State.DRIVING:
            if front_min < self.stop_distance:
                # Lock steering toward the clearer side
                self.steer_direction = -1.0 if left_min >= right_min else 1.0
                self.get_logger().info(
                    f'Obstacle {front_min:.2f}m → '
                    f'turning {"LEFT" if self.steer_direction > 0 else "RIGHT"}')
                self.set_state(State.AVOIDING)
            else:
                self.publish(self.forward_power, 0.0)

        elif self.state == State.AVOIDING:
            if self.elapsed(msg) >= self.avoid_duration:
                # Switch to opposite steering to straighten wheels
                self.set_state(State.STRAIGHTENING)
            else:
                self.publish(
                    self.avoid_power,
                    self.steer_direction * self.steer_power)

        elif self.state == State.STRAIGHTENING:
            straighten_duration = self.avoid_duration * self.straighten_factor
            if self.elapsed(msg) >= straighten_duration:
                # Check if path is actually clear before resuming
                if front_min >= self.resume_distance:
                    self.get_logger().info('Straightened + path clear → DRIVING')
                    self.set_state(State.DRIVING)
                    self.publish(self.forward_power, 0.0)
                else:
                    # Path still blocked — avoid again (same direction)
                    self.get_logger().warning(
                        f'Path still blocked ({front_min:.2f}m) → re-avoiding')
                    self.set_state(State.AVOIDING)
            else:
                # Apply opposite steering to undo the turn
                self.publish(
                    self.avoid_power,
                    -self.steer_direction * self.steer_power)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
