#!/usr/bin/env python3

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from vmc_quadruped_controller.msg import MoveCmd


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class CmdVelToMoveCmd(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_move_cmd')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('move_cmd_topic', '/move_cmd')
        self.declare_parameter('linear_x_scale', 1.0)
        self.declare_parameter('angular_z_scale', 0.45)
        self.declare_parameter('invert_linear_x', True)
        self.declare_parameter('invert_angular_z', True)
        self.declare_parameter('max_angular_z', 0.18)
        self.declare_parameter('max_angular_z_accel', 0.25)
        self.declare_parameter('min_nonzero_angular_z', 0.03)
        self.declare_parameter('min_nonzero_angular_linear_x_threshold', 0.03)
        self.declare_parameter('max_step_x', 1.0)
        self.declare_parameter('max_step_y', 1.0)
        self.declare_parameter('deadzone', 0.01)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('max_step_x_rate', 1.8)
        self.declare_parameter('max_step_y_rate', 1.8)
        self.declare_parameter('cmd_vel_timeout_sec', 0.25)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._move_cmd_topic = self.get_parameter('move_cmd_topic').value
        self._linear_x_scale = float(self.get_parameter('linear_x_scale').value)
        self._angular_z_scale = float(self.get_parameter('angular_z_scale').value)
        self._invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self._invert_angular_z = bool(self.get_parameter('invert_angular_z').value)
        self._max_angular_z = float(self.get_parameter('max_angular_z').value)
        self._max_angular_z_accel = float(
            self.get_parameter('max_angular_z_accel').value
        )
        self._min_nonzero_angular_z = max(
            0.0, float(self.get_parameter('min_nonzero_angular_z').value)
        )
        self._min_nonzero_angular_linear_x_threshold = max(
            0.0,
            float(
                self.get_parameter('min_nonzero_angular_linear_x_threshold').value
            ),
        )
        self._max_step_x = float(self.get_parameter('max_step_x').value)
        self._max_step_y = float(self.get_parameter('max_step_y').value)
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self._max_step_x_rate = float(self.get_parameter('max_step_x_rate').value)
        self._max_step_y_rate = float(self.get_parameter('max_step_y_rate').value)
        self._cmd_vel_timeout_sec = float(
            self.get_parameter('cmd_vel_timeout_sec').value
        )

        self._last_angular_z = 0.0
        self._last_step_x = 0.0
        self._last_step_y = 0.0
        self._last_stamp: Optional[float] = None
        self._last_cmd_vel_rx_time: Optional[float] = None
        self._stopped_by_timeout = False

        self._publisher = self.create_publisher(MoveCmd, self._move_cmd_topic, 20)
        self._subscriber = self.create_subscription(
            Twist, self._cmd_vel_topic, self._cmd_vel_callback, 20
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_callback)
        self.get_logger().info(
            f'Bridge {self._cmd_vel_topic} -> {self._move_cmd_topic} started. '
            f'invert_linear_x={self._invert_linear_x}, '
            f'invert_angular_z={self._invert_angular_z}, '
            f'linear_x_scale={self._linear_x_scale}, angular_z_scale={self._angular_z_scale}, '
            f'min_nonzero_angular_z={self._min_nonzero_angular_z}, '
            f'min_nonzero_angular_linear_x_threshold={self._min_nonzero_angular_linear_x_threshold}, '
            f'cmd_vel_timeout_sec={self._cmd_vel_timeout_sec}'
        )

    def _publish_stop(self) -> None:
        self._last_angular_z = 0.0
        self._last_step_x = 0.0
        self._last_step_y = 0.0
        out = MoveCmd()
        out.step_x = 0.0
        out.step_y = 0.0
        self._publisher.publish(out)

    def _watchdog_callback(self) -> None:
        if self._last_cmd_vel_rx_time is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_cmd_vel_rx_time) <= self._cmd_vel_timeout_sec:
            self._stopped_by_timeout = False
            return
        if self._stopped_by_timeout:
            return
        self._publish_stop()
        self._stopped_by_timeout = True
        self.get_logger().warn('No /cmd_vel received recently, publish zero /move_cmd.')

    def _convert(self, msg: Twist) -> Tuple[float, float]:
        linear_x = -msg.linear.x if self._invert_linear_x else msg.linear.x
        angular_z = -msg.angular.z if self._invert_angular_z else msg.angular.z

        step_y = clamp(
            linear_x * self._linear_x_scale, -self._max_step_y, self._max_step_y
        )
        step_x = clamp(
            angular_z * self._angular_z_scale, -self._max_step_x, self._max_step_x
        )
        if abs(step_x) < self._deadzone:
            step_x = 0.0
        if abs(step_y) < self._deadzone:
            step_y = 0.0
        return step_x, step_y

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_vel_rx_time = self.get_clock().now().nanoseconds / 1e9
        self._stopped_by_timeout = False

        # Commanded stop should take effect immediately, not through smoothing ramp-down.
        if (
            abs(msg.linear.x) < self._deadzone
            and abs(msg.linear.y) < self._deadzone
            and abs(msg.linear.z) < self._deadzone
            and abs(msg.angular.x) < self._deadzone
            and abs(msg.angular.y) < self._deadzone
            and abs(msg.angular.z) < self._deadzone
        ):
            self._publish_stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_stamp is None:
            dt = 0.05
        else:
            dt = max(0.001, min(0.2, now - self._last_stamp))
        self._last_stamp = now

        linear_x = -msg.linear.x if self._invert_linear_x else msg.linear.x
        angular_z = -msg.angular.z if self._invert_angular_z else msg.angular.z
        angular_z = clamp(angular_z, -self._max_angular_z, self._max_angular_z)
        in_place_turn = (
            abs(linear_x) <= self._min_nonzero_angular_linear_x_threshold
        )

        max_dw = self._max_angular_z_accel * dt
        angular_z = clamp(
            angular_z,
            self._last_angular_z - max_dw,
            self._last_angular_z + max_dw,
        )
        if in_place_turn and 0.0 < abs(angular_z) < self._min_nonzero_angular_z:
            angular_z = self._min_nonzero_angular_z if angular_z > 0.0 else -self._min_nonzero_angular_z
        self._last_angular_z = angular_z

        target_step_y = clamp(
            linear_x * self._linear_x_scale, -self._max_step_y, self._max_step_y
        )
        target_step_x = clamp(
            angular_z * self._angular_z_scale, -self._max_step_x, self._max_step_x
        )
        if abs(target_step_x) < self._deadzone:
            target_step_x = 0.0
        if abs(target_step_y) < self._deadzone:
            target_step_y = 0.0

        # Low-pass filtering to reduce stop-go jitter from high-frequency cmd_vel changes.
        alpha = clamp(self._smoothing_alpha, 0.0, 1.0)
        step_x = self._last_step_x + alpha * (target_step_x - self._last_step_x)
        step_y = self._last_step_y + alpha * (target_step_y - self._last_step_y)

        # Slew-rate limit to avoid abrupt gait command jumps.
        max_dx = self._max_step_x_rate * dt
        max_dy = self._max_step_y_rate * dt
        step_x = clamp(step_x, self._last_step_x - max_dx, self._last_step_x + max_dx)
        step_y = clamp(step_y, self._last_step_y - max_dy, self._last_step_y + max_dy)

        # Non-zero turn command should not be too small to execute:
        # either zero, or at least the configured minimum.
        min_step_x = self._min_nonzero_angular_z * self._angular_z_scale
        if (
            in_place_turn
            and min_step_x > 0.0
            and abs(target_step_x) >= self._deadzone
            and abs(step_x) < min_step_x
        ):
            step_x = min_step_x if target_step_x > 0.0 else -min_step_x

        if abs(step_x) < self._deadzone:
            step_x = 0.0
        if abs(step_y) < self._deadzone:
            step_y = 0.0

        self._last_step_x = step_x
        self._last_step_y = step_y

        out = MoveCmd()
        out.step_x = float(step_x)
        out.step_y = float(step_y)
        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToMoveCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
