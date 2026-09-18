#!/usr/bin/env python3
"""
Start/Pause control via two GPIO buttons on the RPi5.

Buttons wired active-low to GND with internal pull-up:
  - START (default GPIO17): enable autonomous driving.
  - PAUSE (default GPIO27): disable autonomous driving (default state at boot).

Publishes:
  /autonomy_enabled (std_msgs/Bool) — every tick, so subscribers (e.g.
  obstacle_avoidance) can detect a stale/dead publisher and fail safe.

Optional bonus (off by default, see `manage_gamepad_process`): PAUSE
kills-then-respawns `gamepad.sh` fresh on every press (also fixes the case
where the gamepad was powered on after the process already gave up on
/dev/input/js0); START kills it so it doesn't race with autonomy.
"""

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import lgpio
except ImportError:  # pragma: no cover
    lgpio = None


class RunStateNode(Node):

    def __init__(self) -> None:
        super().__init__('run_state_node')

        self.declare_parameter('gpio_chip', '/dev/gpiochip0')
        self.declare_parameter('start_pin', 17)
        self.declare_parameter('pause_pin', 27)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('debounce_s', 0.03)
        self.declare_parameter('manage_gamepad_process', False)
        self.declare_parameter('gamepad_script_path', '/home/dano/robot-ros2/gamepad.sh')
        self.declare_parameter('gamepad_kill_timeout_s', 2.0)

        if lgpio is None:
            self.get_logger().error(
                'Python package "lgpio" is not installed. Install it before running this node.'
            )
            raise RuntimeError('lgpio is required')

        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.start_pin = int(self.get_parameter('start_pin').value)
        self.pause_pin = int(self.get_parameter('pause_pin').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.debounce_s = float(self.get_parameter('debounce_s').value)
        self.manage_gamepad_process = bool(self.get_parameter('manage_gamepad_process').value)
        self.gamepad_script_path = str(self.get_parameter('gamepad_script_path').value)
        self.gamepad_kill_timeout_s = float(self.get_parameter('gamepad_kill_timeout_s').value)

        self._chip_handle = lgpio.gpiochip_open(0)
        try:
            lgpio.gpio_claim_input(self._chip_handle, self.start_pin, lgpio.SET_PULL_UP)
            lgpio.gpio_claim_input(self._chip_handle, self.pause_pin, lgpio.SET_PULL_UP)
        except Exception as exc:
            lgpio.gpiochip_close(self._chip_handle)
            raise RuntimeError(
                'Failed to claim GPIO pins (start=%d, pause=%d): %s'
                % (self.start_pin, self.pause_pin, exc)
            )

        # Debounce tracking: raw pin level + how long it's been stable, per button.
        self._start_raw_pressed = False
        self._start_stable_s = 0.0
        self._start_confirmed_pressed = False
        self._pause_raw_pressed = False
        self._pause_stable_s = 0.0
        self._pause_confirmed_pressed = False

        self.enabled = False  # start PAUSED
        self._gamepad_proc: 'subprocess.Popen | None' = None

        self._pub = self.create_publisher(Bool, '/autonomy_enabled', 10)
        self._timer = self.create_timer(1.0 / self.poll_rate_hz, self._on_timer)

        self.get_logger().info(
            'run_state_node ready: start_pin=%d, pause_pin=%d, manage_gamepad_process=%s — starting PAUSED'
            % (self.start_pin, self.pause_pin, self.manage_gamepad_process)
        )

    def _debounced_press(self, raw_pressed: bool, was_raw: bool, stable_s: float,
                          confirmed: bool) -> tuple[bool, float, bool, bool]:
        """Returns (new_was_raw, new_stable_s, new_confirmed, rising_edge_to_confirmed_press)."""
        dt = 1.0 / self.poll_rate_hz
        if raw_pressed == was_raw:
            stable_s += dt
        else:
            stable_s = 0.0
        edge = False
        if stable_s >= self.debounce_s and raw_pressed != confirmed:
            confirmed = raw_pressed
            edge = confirmed  # only report edges into the pressed state
        return raw_pressed, stable_s, confirmed, edge

    def _on_timer(self) -> None:
        start_raw = lgpio.gpio_read(self._chip_handle, self.start_pin) == 0
        pause_raw = lgpio.gpio_read(self._chip_handle, self.pause_pin) == 0

        (self._start_raw_pressed, self._start_stable_s,
         self._start_confirmed_pressed, start_edge) = self._debounced_press(
            start_raw, self._start_raw_pressed, self._start_stable_s,
            self._start_confirmed_pressed)

        (self._pause_raw_pressed, self._pause_stable_s,
         self._pause_confirmed_pressed, pause_edge) = self._debounced_press(
            pause_raw, self._pause_raw_pressed, self._pause_stable_s,
            self._pause_confirmed_pressed)

        if start_edge:
            self._on_start_pressed()
        if pause_edge:
            self._on_pause_pressed()

        self._pub.publish(Bool(data=self.enabled))

    def _on_start_pressed(self) -> None:
        self.enabled = True
        self.get_logger().info('START pressed → autonomy ENABLED')
        if self.manage_gamepad_process:
            self._kill_gamepad()

    def _on_pause_pressed(self) -> None:
        self.enabled = False
        self.get_logger().info('PAUSE pressed → autonomy DISABLED')
        if self.manage_gamepad_process:
            self._kill_gamepad()
            self._spawn_gamepad()

    def _spawn_gamepad(self) -> None:
        self._gamepad_proc = subprocess.Popen(
            ['bash', self.gamepad_script_path],
            preexec_fn=os.setsid,
        )
        self.get_logger().info('Spawned gamepad process (pid=%d)' % self._gamepad_proc.pid)

    def _kill_gamepad(self) -> None:
        proc = self._gamepad_proc
        if proc is None or proc.poll() is not None:
            self._gamepad_proc = None
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=self.gamepad_kill_timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=self.gamepad_kill_timeout_s)
        except ProcessLookupError:
            pass
        finally:
            self._gamepad_proc = None

    def destroy_node(self) -> bool:
        if self.manage_gamepad_process:
            self._kill_gamepad()
        if self._chip_handle is not None:
            try:
                lgpio.gpiochip_close(self._chip_handle)
            except Exception:
                pass
            self._chip_handle = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RunStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
