#!/usr/bin/env python3
"""
Ackermann obstacle avoidance node for open-loop DC traction / closed-loop steering.
Uses /traction_motor_cmd directly; steering goes through /steering_cmd, which
is consumed by steering_sensor's closed-loop steering_controller_node.

Topics:
  Subscribe: /scan_c1              (sensor_msgs/LaserScan)
  Subscribe: /road_follower/error  (std_msgs/Float32, -1..1, road center offset)
  Publish:   /traction_motor_cmd   (std_msgs/Float32, range -1.0 to +1.0)
  Publish:   /steering_cmd         (std_msgs/Float32, range -1.0 to +1.0)
             +1.0 = full left, -1.0 = full right, 0.0 = neutral power

States:
  DRIVING      → forward at cruise speed, steering follows road_follower error
  AVOIDING     → forward at avoid speed, full steering lock one direction
  STRAIGHTENING→ forward at avoid speed, full steering lock OTHER direction
                 for same duration as AVOIDING (with correction factor)
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float32, Bool
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
        self.declare_parameter('road_steer_gain',     1.00)  # scales road_follower error into steering cmd
        self.declare_parameter('road_error_timeout_s', 1.00)  # treat stale road error as 0 (drive straight)
        self.declare_parameter('gps_steer_gain',       0.35)  # OSM-gated GPS correction in DRIVING
        self.declare_parameter('gps_bias_timeout_s',   1.00)
        self.declare_parameter('enable_debug_image',  True)  # publish bird's-eye lidar view for Foxglove
        self.declare_parameter('debug_image_size_px', 500)   # square canvas side length
        self.declare_parameter('debug_image_max_range_m', 3.0)  # canvas edge = this many meters

        self.forward_power    = self.get_parameter('forward_power').value
        self.avoid_power      = self.get_parameter('avoid_power').value
        self.steer_power      = self.get_parameter('steer_power').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle').value)
        self.stop_distance    = self.get_parameter('stop_distance').value
        self.resume_distance  = self.get_parameter('resume_distance').value
        self.avoid_duration   = self.get_parameter('avoid_duration').value
        self.straighten_factor = self.get_parameter('straighten_factor').value
        self.road_steer_gain  = self.get_parameter('road_steer_gain').value
        self.road_error_timeout_s = self.get_parameter('road_error_timeout_s').value
        self.gps_steer_gain = self.get_parameter('gps_steer_gain').value
        self.gps_bias_timeout_s = self.get_parameter('gps_bias_timeout_s').value
        self.enable_debug_image = self.get_parameter('enable_debug_image').value
        self.debug_image_size_px = self.get_parameter('debug_image_size_px').value
        self.debug_image_max_range_m = self.get_parameter('debug_image_max_range_m').value
        self.declare_parameter('autonomy_enabled_timeout_s', 1.00)  # start/pause watchdog
        self.autonomy_enabled_timeout_s = self.get_parameter('autonomy_enabled_timeout_s').value

        # ── State ──────────────────────────────────────────────────────
        self.state            = State.DRIVING
        self.steer_direction  = 0.0   # +1.0 = left, -1.0 = right (locked per episode)
        self.state_start_time = None  # rclpy.Time when current timed state began
        self.road_error       = 0.0
        self.road_error_rx_s  = None
        self.gps_bias = 0.0
        self.gps_bias_rx_s = None
        # Paused until run_state_node says otherwise (fail-safe default).
        self.autonomy_enabled    = False
        self.autonomy_enabled_rx_s = None
        self._was_driving         = False

        # ── ROS 2 interfaces ───────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_c1',
            self.scan_callback,
            qos_profile_sensor_data)          # best-effort QoS for sensor data

        self.road_error_sub = self.create_subscription(
            Float32, '/road_follower/error',
            self.road_error_callback, 10)

        self.gps_bias_sub = self.create_subscription(
            Float32, '/gps_steering_bias',
            self.gps_bias_callback, 10)

        self.autonomy_enabled_sub = self.create_subscription(
            Bool, '/autonomy_enabled',
            self.autonomy_enabled_callback, 10)

        self.traction_pub = self.create_publisher(
            Float32, '/traction_motor_cmd', 10)
        self.steering_pub = self.create_publisher(
            Float32, '/steering_cmd', 10)
        self.debug_image_pub = self.create_publisher(
            CompressedImage, '/obstacle_avoidance/debug_image/compressed', 10)

        self.get_logger().info('Obstacle avoider started — State: DRIVING')

    def road_error_callback(self, msg: Float32):
        self.road_error = msg.data
        self.road_error_rx_s = self.get_clock().now().nanoseconds * 1e-9

    def autonomy_enabled_callback(self, msg: Bool):
        self.autonomy_enabled = msg.data
        self.autonomy_enabled_rx_s = self.get_clock().now().nanoseconds * 1e-9

    def gps_bias_callback(self, msg: Float32):
        self.gps_bias = max(-1.0, min(1.0, float(msg.data)))
        self.gps_bias_rx_s = self.get_clock().now().nanoseconds * 1e-9

    def is_autonomy_enabled(self):
        """False if never received or stale — fail safe (paused) on watchdog loss."""
        if self.autonomy_enabled_rx_s is None:
            return False
        age_s = self.get_clock().now().nanoseconds * 1e-9 - self.autonomy_enabled_rx_s
        if age_s > self.autonomy_enabled_timeout_s:
            return False
        return self.autonomy_enabled

    def get_road_steer(self):
        """Road-following steering target, or 0.0 (drive straight) if stale/absent."""
        if self.road_error_rx_s is None:
            return 0.0
        age_s = self.get_clock().now().nanoseconds * 1e-9 - self.road_error_rx_s
        if age_s > self.road_error_timeout_s:
            return 0.0
        return max(-1.0, min(1.0, self.road_error * self.road_steer_gain))

    def get_driving_steer(self):
        road_steer = self.get_road_steer()
        if self.gps_bias_rx_s is None:
            return road_steer
        age_s = self.get_clock().now().nanoseconds * 1e-9 - self.gps_bias_rx_s
        if age_s > self.gps_bias_timeout_s:
            return road_steer
        return max(-1.0, min(1.0, road_steer + self.gps_bias * self.gps_steer_gain))

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

    # ── Debug visualization ─────────────────────────────────────────────

    def publish_debug_image(self, msg: LaserScan):
        """Bird's-eye render of the corrected scan, for Foxglove (self-contained —
        does not touch/depend on the control-flow state below)"""
        size = self.debug_image_size_px
        max_range = self.debug_image_max_range_m
        px_per_m = size / (2.0 * max_range)
        origin = (size // 2, int(size * 2 / 3))  # lower 1/3 — leaves room to show behind the robot
        canvas = np.zeros((size, size, 3), dtype=np.uint8)

        front_close = []
        left_close = []
        right_close = []
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(-raw_angle + math.pi),
                                math.cos(-raw_angle + math.pi))

            if r < self.stop_distance:
                color = (0, 0, 255)      # red — inside stop threshold
            elif r < self.resume_distance:
                color = (0, 165, 255)    # orange — inside resume threshold
            else:
                color = (0, 220, 0)      # green — clear

            # +x forward (up on canvas), +y left (matches steering convention, +1=left)
            px = int(origin[0] - r * math.sin(angle) * px_per_m)
            py = int(origin[1] - r * math.cos(angle) * px_per_m)
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(canvas, (px, py), 2, color, -1)

            if -self.front_half_angle <= angle <= self.front_half_angle:
                front_close.append(r)
            if 0.0 <= angle <= self.front_half_angle:
                left_close.append(r)
            if -self.front_half_angle <= angle <= 0.0:
                right_close.append(r)

        front_min = min(front_close, default=float('inf'))
        left_min = min(left_close, default=float('inf'))
        right_min = min(right_close, default=float('inf'))

        # Robot marker + heading arrow
        cv2.circle(canvas, origin, 6, (255, 255, 255), -1)
        cv2.arrowedLine(canvas, origin, (origin[0], origin[1] - 30), (255, 255, 255), 2, tipLength=0.4)

        # Front half-angle cone
        cone_len = max_range * px_per_m
        for a in (-self.front_half_angle, self.front_half_angle):
            end = (int(origin[0] - cone_len * math.sin(a)),
                   int(origin[1] - cone_len * math.cos(a)))
            cv2.line(canvas, origin, end, (128, 128, 128), 1)

        # Stop/resume distance circles
        cv2.circle(canvas, origin, int(self.stop_distance * px_per_m), (0, 0, 180), 1)
        cv2.circle(canvas, origin, int(self.resume_distance * px_per_m), (0, 120, 180), 1)

        autonomy_txt = 'ENABLED' if self.is_autonomy_enabled() else 'PAUSED'
        lines = [
            f'State: {self.state.value}  [{autonomy_txt}]',
            f'front={front_min:.2f}m left={left_min:.2f}m right={right_min:.2f}m',
        ]
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (8, 20 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        ok, buffer = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            out = CompressedImage()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.format = 'jpeg'
            out.data = buffer.tobytes()
            self.debug_image_pub.publish(out)

    # ── Main callback ─────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):

        if self.enable_debug_image:
            try:
                self.publish_debug_image(msg)
            except Exception as e:
                self.get_logger().error(f'debug image render failed: {e}')

        # ── Start/Pause gate ────────────────────────────────────────────
        # While paused, stay hands-off entirely (gamepad/human owns the motors).
        if not self.is_autonomy_enabled():
            if self._was_driving:
                self.publish(0.0, 0.0)
                self.get_logger().info('Autonomy disabled → stopped, standing by')
            self._was_driving = False
            return
        self._was_driving = True

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
                self.publish(self.forward_power, self.get_driving_steer())

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
                    self.publish(self.forward_power, self.get_driving_steer())
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
