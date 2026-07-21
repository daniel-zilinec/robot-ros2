from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import lgpio
except ImportError:  # pragma: no cover
    lgpio = None


@dataclass
class CommandState:
    value: float = 0.0
    last_rx_s: float = 0.0


class Dri0042MotorNode(Node):
    """Open-loop single DC motor control for DFRobot DRI0042 using GPIO."""

    def __init__(self) -> None:
        super().__init__('dri0042_motor_node')
        self._chip_handle: int | None = None
        self._chip_index_in_use: int | None = None
        self._gpio_claimed = False

        self.declare_parameter('gpio_chip', '/dev/gpiochip0')
        self.declare_parameter('pin_pwm', 18)
        self.declare_parameter('pin_in1', 23)
        self.declare_parameter('pin_in2', 24)
        self.declare_parameter('pwm_frequency_hz', 1000.0)
        self.declare_parameter('watchdog_timeout_s', 2.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('topic_cmd', '/motor_cmd')
        self.declare_parameter('invert_direction', False)
        self.declare_parameter('soft_start_stop', True)
        self.declare_parameter('soft_start_stop_rate_per_s', 0.1)
        self.declare_parameter('debug_ramp', False)
        self.declare_parameter('debug_ramp_period_s', 0.5)

        if lgpio is None:
            self.get_logger().error(
                'Python package "lgpio" is not installed. Install it before running this node.'
            )
            raise RuntimeError('lgpio is required')

        self.gpio_chip = self.get_parameter('gpio_chip').value
        self.pin_pwm = int(self.get_parameter('pin_pwm').value)
        self.pin_in1 = int(self.get_parameter('pin_in1').value)
        self.pin_in2 = int(self.get_parameter('pin_in2').value)
        self.pwm_frequency_hz = max(10, int(float(self.get_parameter('pwm_frequency_hz').value)))
        self.watchdog_timeout_s = float(self.get_parameter('watchdog_timeout_s').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.topic_cmd = str(self.get_parameter('topic_cmd').value)
        self.invert_direction = bool(self.get_parameter('invert_direction').value)
        self.soft_start_stop = bool(self.get_parameter('soft_start_stop').value)
        self.soft_start_stop_rate_per_s = max(
            0.01,
            float(self.get_parameter('soft_start_stop_rate_per_s').value),
        )
        self.debug_ramp = bool(self.get_parameter('debug_ramp').value)
        self.debug_ramp_period_s = max(
            0.05,
            float(self.get_parameter('debug_ramp_period_s').value),
        )
        self._pwm_failed = False
        self._applied_cmd = 0.0
        self._last_ramp_debug_log_s = 0.0

        self._chip_handle, self._chip_index_in_use = self._open_gpiochip(self.gpio_chip)
        try:
            lgpio.gpio_claim_output(self._chip_handle, self.pin_in1, 0)
            lgpio.gpio_claim_output(self._chip_handle, self.pin_in2, 0)
            lgpio.gpio_claim_output(self._chip_handle, self.pin_pwm, 0)
            self._gpio_claimed = True
        except Exception as exc:
            if self._chip_handle is not None:
                try:
                    lgpio.gpiochip_close(self._chip_handle)
                except Exception:
                    pass
            self._chip_handle = None
            raise RuntimeError(
                'Failed to claim GPIO pins (PWM=%d, IN1=%d, IN2=%d): %s'
                % (self.pin_pwm, self.pin_in1, self.pin_in2, exc)
            )

        self._cmd = CommandState(value=0.0, last_rx_s=self._now_s())

        self.create_subscription(Float32, self.topic_cmd, self._on_cmd, 10)
        self._timer = self.create_timer(1.0 / self.control_rate_hz, self._on_timer)

        self.get_logger().info(
            'DRI0042 motor node ready: cmd=%s, chip=%s (idx=%d), PWM pin=%d, IN1=%d, IN2=%d, ramp=%s rate=%.3f/s step=%.5f/tick debug=%s'
            % (
                self.topic_cmd,
                self.gpio_chip,
                self._chip_index_in_use,
                self.pin_pwm,
                self.pin_in1,
                self.pin_in2,
                self.soft_start_stop,
                self.soft_start_stop_rate_per_s,
                self.soft_start_stop_rate_per_s / self.control_rate_hz,
                self.debug_ramp,
            )
        )

    @staticmethod
    def _chip_index(chip: str) -> int:
        text = chip.strip()
        if text.startswith('/dev/gpiochip'):
            suffix = text.replace('/dev/gpiochip', '', 1)
            return int(suffix)
        if text.startswith('gpiochip'):
            suffix = text.replace('gpiochip', '', 1)
            return int(suffix)
        return int(text)

    def _open_gpiochip(self, requested_chip: str) -> tuple[int, int]:
        candidates = [requested_chip]
        if requested_chip not in ('/dev/gpiochip0', 'gpiochip0', '0'):
            candidates.append('/dev/gpiochip0')

        last_error: Exception | None = None
        for candidate in candidates:
            idx = self._chip_index(candidate)
            try:
                handle = lgpio.gpiochip_open(idx)
                if candidate != requested_chip:
                    self.get_logger().warn(
                        'Requested gpio chip %s not openable, falling back to %s.'
                        % (requested_chip, candidate)
                    )
                return handle, idx
            except Exception as exc:  # pragma: no cover
                last_error = exc

        raise RuntimeError(
            'Unable to open gpiochip. Tried %s. Last error: %s'
            % (candidates, last_error)
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Float32) -> None:
        # Command range is normalized to [-1.0, 1.0].
        val = max(-1.0, min(1.0, float(msg.data)))
        if not math.isfinite(val):
            self.get_logger().warn('Ignoring non-finite motor command: %s' % msg.data)
            return
        self._cmd.value = val
        self._cmd.last_rx_s = self._now_s()
        if self.debug_ramp:
            self.get_logger().info('motor_cmd rx: target=%.3f' % val)

    def _on_timer(self) -> None:
        age_s = self._now_s() - self._cmd.last_rx_s
        target_cmd = 0.0 if age_s > self.watchdog_timeout_s else self._cmd.value
        self._apply_output(target_cmd)

    def _apply_output(self, cmd: float) -> None:
        if self._chip_handle is None or not self._gpio_claimed:
            return
        ramped_cmd = self._slew_command(cmd)
        if abs(ramped_cmd) < 1e-4:
            self._applied_cmd = 0.0
            self._coast()
            return

        forward = ramped_cmd > 0.0
        if self.invert_direction:
            forward = not forward

        if forward:
            lgpio.gpio_write(self._chip_handle, self.pin_in1, 1)
            lgpio.gpio_write(self._chip_handle, self.pin_in2, 0)
        else:
            lgpio.gpio_write(self._chip_handle, self.pin_in1, 0)
            lgpio.gpio_write(self._chip_handle, self.pin_in2, 1)

        duty_pct = max(0.0, min(100.0, abs(ramped_cmd) * 100.0))
        self._maybe_log_ramp(cmd, ramped_cmd, duty_pct)
        self._set_pwm(duty_pct)

    def _maybe_log_ramp(self, target_cmd: float, ramped_cmd: float, duty_pct: float) -> None:
        if not self.debug_ramp:
            return
        now_s = self._now_s()
        if (now_s - self._last_ramp_debug_log_s) < self.debug_ramp_period_s:
            return
        self._last_ramp_debug_log_s = now_s
        self.get_logger().info(
            'ramp: target=%.3f applied=%.3f duty=%.1f%%'
            % (target_cmd, ramped_cmd, duty_pct)
        )

    def _slew_command(self, target_cmd: float) -> float:
        if not self.soft_start_stop:
            self._applied_cmd = target_cmd
            return self._applied_cmd

        max_delta = self.soft_start_stop_rate_per_s / self.control_rate_hz
        delta = target_cmd - self._applied_cmd
        if abs(delta) <= max_delta:
            self._applied_cmd = target_cmd
        else:
            self._applied_cmd += max_delta if delta > 0.0 else -max_delta
        return self._applied_cmd

    def _set_pwm(self, duty_pct: float) -> None:
        if self._chip_handle is None or not self._gpio_claimed or self._pwm_failed:
            return
        try:
            lgpio.tx_pwm(self._chip_handle, self.pin_pwm, self.pwm_frequency_hz, duty_pct)
        except Exception as exc:  # pragma: no cover
            self._pwm_failed = True
            self.get_logger().error(
                'Failed to configure PWM on pin %d at %d Hz: %s'
                % (self.pin_pwm, self.pwm_frequency_hz, exc)
            )
            try:
                lgpio.gpio_write(self._chip_handle, self.pin_pwm, 0)
            except Exception:
                pass

    def _coast(self) -> None:
        if self._chip_handle is None or not self._gpio_claimed:
            return
        self._set_pwm(0.0)
        lgpio.gpio_write(self._chip_handle, self.pin_in1, 0)
        lgpio.gpio_write(self._chip_handle, self.pin_in2, 0)

    def destroy_node(self) -> bool:
        try:
            try:
                self._coast()
            except Exception:
                pass
        finally:
            try:
                if lgpio is not None and self._chip_handle is not None:
                    lgpio.gpiochip_close(self._chip_handle)
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Dri0042MotorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
