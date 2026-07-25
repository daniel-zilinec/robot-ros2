from __future__ import annotations

import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
_JS_EVENT_FORMAT = 'IhBB'
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FORMAT)


class GamepadTeleopNode(Node):
    """Publish traction and steering motor commands from a Linux joystick device."""

    def __init__(self) -> None:
        super().__init__('gamepad_teleop_node')

        self.declare_parameter('device_path', '/dev/input/js0')
        self.declare_parameter('topic_traction', '/traction_motor_cmd')
        self.declare_parameter('topic_steering', '/steering_motor_cmd')
        self.declare_parameter('axis_traction', 1)
        self.declare_parameter('axis_steering', 2)
        self.declare_parameter('invert_traction', True)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('max_traction', 1.0)
        self.declare_parameter('max_steering', 1.0)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('device_retry_s', 1.0)

        self.device_path = str(self.get_parameter('device_path').value)
        self.topic_traction = str(self.get_parameter('topic_traction').value)
        self.topic_steering = str(self.get_parameter('topic_steering').value)
        self.axis_traction = int(self.get_parameter('axis_traction').value)
        self.axis_steering = int(self.get_parameter('axis_steering').value)
        self.invert_traction = bool(self.get_parameter('invert_traction').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.max_traction = max(0.0, min(1.0, float(self.get_parameter('max_traction').value)))
        self.max_steering = max(0.0, min(1.0, float(self.get_parameter('max_steering').value)))
        self.deadzone = max(0.0, min(0.5, float(self.get_parameter('deadzone').value)))
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.device_retry_s = max(0.2, float(self.get_parameter('device_retry_s').value))

        self._axis_values: dict[int, float] = {}
        self._button_values: dict[int, int] = {}
        self._lock = threading.Lock()
        self._running = True
        self._last_connect_log_s = 0.0

        self._pub_traction = self.create_publisher(Float32, self.topic_traction, 10)
        self._pub_steering = self.create_publisher(Float32, self.topic_steering, 10)
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            'Gamepad teleop ready: device=%s traction_topic=%s steering_topic=%s axis_traction=%d axis_steering=%d deadman=%d'
            % (
                self.device_path,
                self.topic_traction,
                self.topic_steering,
                self.axis_traction,
                self.axis_steering,
                self.deadman_button,
            )
        )

    def _reader_loop(self) -> None:
        while self._running:
            try:
                with open(self.device_path, 'rb', buffering=0) as dev:
                    self.get_logger().info('Connected to joystick device: %s' % self.device_path)
                    while self._running:
                        packet = dev.read(_JS_EVENT_SIZE)
                        if len(packet) != _JS_EVENT_SIZE:
                            break
                        _, value, event_type, number = struct.unpack(_JS_EVENT_FORMAT, packet)
                        event_type = event_type & ~_JS_EVENT_INIT
                        if event_type == _JS_EVENT_AXIS:
                            with self._lock:
                                self._axis_values[number] = float(value) / 32767.0
                        elif event_type == _JS_EVENT_BUTTON:
                            with self._lock:
                                self._button_values[number] = 1 if value else 0
            except FileNotFoundError:
                self._throttled_warn('Joystick device not found: %s' % self.device_path)
            except PermissionError:
                self._throttled_warn(
                    'Permission denied for %s. Add user to input group: sudo usermod -aG input $USER'
                    % self.device_path
                )
            except OSError as exc:
                self._throttled_warn('Joystick read error on %s: %s' % (self.device_path, exc))

            if self._running:
                time.sleep(self.device_retry_s)

    def _throttled_warn(self, msg: str) -> None:
        now_s = time.monotonic()
        if (now_s - self._last_connect_log_s) >= 5.0:
            self._last_connect_log_s = now_s
            self.get_logger().warn(msg)

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        if value > 0.0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        return (value + self.deadzone) / (1.0 - self.deadzone)

    def _axis(self, axis_num: int) -> float:
        with self._lock:
            return float(self._axis_values.get(axis_num, 0.0))

    def _deadman_pressed(self) -> bool:
        if self.deadman_button < 0:
            return True
        with self._lock:
            return bool(self._button_values.get(self.deadman_button, 0))

    def _on_timer(self) -> None:
        traction = self._axis(self.axis_traction)
        steering = self._axis(self.axis_steering)

        traction = self._apply_deadzone(traction)
        steering = self._apply_deadzone(steering)

        if self.invert_traction:
            traction = -traction
        if self.invert_steering:
            steering = -steering

        traction *= self.max_traction
        steering *= self.max_steering

        if not self._deadman_pressed():
            traction = 0.0
            steering = 0.0

        self._pub_traction.publish(Float32(data=float(traction)))
        self._pub_steering.publish(Float32(data=float(steering)))

    def destroy_node(self) -> bool:
        self._running = False
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GamepadTeleopNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
