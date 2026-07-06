#!/usr/bin/env python3

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vmc_quadruped_controller.msg import MoveCmd


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CmdVelToMoveCmd(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_move_cmd')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('move_cmd_topic', '/move_cmd')
        self.declare_parameter('linear_x_scale', 1.0)
        self.declare_parameter('angular_z_scale', 0.45)
        self.declare_parameter('invert_linear_x', True)
        self.declare_parameter('invert_angular_z', True)
        self.declare_parameter('max_angular_z', 0.14)
        self.declare_parameter('max_angular_z_accel', 0.20)
        self.declare_parameter('min_nonzero_linear_x', 0.06)
        self.declare_parameter('min_nonzero_angular_z', 0.04)
        self.declare_parameter('min_nonzero_angular_linear_x_threshold', 0.03)
        self.declare_parameter('max_step_x', 1.0)
        self.declare_parameter('max_step_y', 1.0)
        self.declare_parameter('deadzone', 0.01)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('max_step_x_rate', 1.8)
        self.declare_parameter('max_step_y_rate', 1.8)
        self.declare_parameter('cmd_vel_timeout_sec', 0.25)
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('preset_goal_pose_topic', '/preset_current_goal')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('global_plan_topic', '/global_plan')
        self.declare_parameter('motion_lock_topic', '/base_motion/lock')
        self.declare_parameter('status_topic', '/cmd_vel_to_move_cmd/status')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('final_align_enabled', False)
        self.declare_parameter('final_align_xy_trigger', 0.08)
        self.declare_parameter('final_align_yaw_trigger', 0.18)
        self.declare_parameter('final_align_yaw_exit', 0.08)
        self.declare_parameter('final_align_linear_scale', 0.15)
        self.declare_parameter('final_align_max_linear_x', 0.02)
        self.declare_parameter('final_align_angular_kp', 1.2)
        self.declare_parameter('final_align_lookup_timeout_sec', 0.05)
        self.declare_parameter('goal_stop_guard_enabled', True)
        self.declare_parameter('goal_stop_xy_tolerance', 0.10)

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
        self._min_nonzero_linear_x = max(
            0.0, float(self.get_parameter('min_nonzero_linear_x').value)
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
        self._goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self._preset_goal_pose_topic = self.get_parameter(
            'preset_goal_pose_topic'
        ).value
        self._plan_topic = self.get_parameter('plan_topic').value
        self._global_plan_topic = self.get_parameter('global_plan_topic').value
        self._motion_lock_topic = self.get_parameter('motion_lock_topic').value
        self._status_topic = self.get_parameter('status_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._final_align_enabled = bool(
            self.get_parameter('final_align_enabled').value
        )
        self._final_align_xy_trigger = float(
            self.get_parameter('final_align_xy_trigger').value
        )
        self._final_align_yaw_trigger = float(
            self.get_parameter('final_align_yaw_trigger').value
        )
        self._final_align_yaw_exit = float(
            self.get_parameter('final_align_yaw_exit').value
        )
        self._final_align_linear_scale = float(
            self.get_parameter('final_align_linear_scale').value
        )
        self._final_align_max_linear_x = float(
            self.get_parameter('final_align_max_linear_x').value
        )
        self._final_align_angular_kp = float(
            self.get_parameter('final_align_angular_kp').value
        )
        self._final_align_lookup_timeout_sec = float(
            self.get_parameter('final_align_lookup_timeout_sec').value
        )
        self._goal_stop_guard_enabled = bool(
            self.get_parameter('goal_stop_guard_enabled').value
        )
        self._goal_stop_xy_tolerance = max(
            0.0, float(self.get_parameter('goal_stop_xy_tolerance').value)
        )

        self._last_angular_z = 0.0
        self._last_step_x = 0.0
        self._last_step_y = 0.0
        self._last_stamp: Optional[float] = None
        self._last_cmd_vel_rx_time: Optional[float] = None
        self._stopped_by_timeout = False
        self._motion_locked = False
        self._final_align_active = False
        self._goal_pose: Optional[PoseStamped] = None
        self._goal_pose_rx_time: float = 0.0
        self._plan_goal_pose: Optional[PoseStamped] = None
        self._plan_goal_pose_rx_time: float = 0.0
        self._last_status_pub_sec = 0.0
        self._last_status = {
            'final_align_active': False,
            'goal_available': False,
            'goal_frame': '',
            'distance_m': None,
            'yaw_error_rad': None,
            'yaw_error_deg': None,
            'linear_x_cmd': 0.0,
            'angular_z_cmd': 0.0,
            'step_x': 0.0,
            'step_y': 0.0,
            'reason': 'startup',
        }

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._publisher = self.create_publisher(MoveCmd, self._move_cmd_topic, 20)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._subscriber = self.create_subscription(
            Twist, self._cmd_vel_topic, self._cmd_vel_callback, 20
        )
        self._motion_lock_sub = self.create_subscription(
            Bool, self._motion_lock_topic, self._motion_lock_callback, 10
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, self._goal_pose_topic, self._goal_pose_callback, 10
        )
        self._preset_goal_sub = self.create_subscription(
            PoseStamped,
            self._preset_goal_pose_topic,
            self._goal_pose_callback,
            10,
        )
        self._plan_sub = self.create_subscription(
            Path, self._plan_topic, self._plan_callback, 10
        )
        self._global_plan_sub = self.create_subscription(
            Path, self._global_plan_topic, self._plan_callback, 10
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_callback)
        self.get_logger().info(
            f'Bridge {self._cmd_vel_topic} -> {self._move_cmd_topic} started. '
            f'invert_linear_x={self._invert_linear_x}, '
            f'invert_angular_z={self._invert_angular_z}, '
            f'linear_x_scale={self._linear_x_scale}, angular_z_scale={self._angular_z_scale}, '
            f'min_nonzero_linear_x={self._min_nonzero_linear_x}, '
            f'min_nonzero_angular_z={self._min_nonzero_angular_z}, '
            f'min_nonzero_angular_linear_x_threshold={self._min_nonzero_angular_linear_x_threshold}, '
            f'cmd_vel_timeout_sec={self._cmd_vel_timeout_sec}, '
            f'motion_lock_topic={self._motion_lock_topic}, '
            f'final_align_enabled={self._final_align_enabled}'
        )

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        self._goal_pose = msg
        self._goal_pose_rx_time = self.get_clock().now().nanoseconds / 1e9

    def _plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._plan_goal_pose = msg.poses[-1]
        if not self._plan_goal_pose.header.frame_id:
            self._plan_goal_pose.header.frame_id = msg.header.frame_id
        self._plan_goal_pose_rx_time = self.get_clock().now().nanoseconds / 1e9

    def _get_active_goal_pose(self) -> Optional[PoseStamped]:
        if self._goal_pose is None:
            return self._plan_goal_pose
        if self._plan_goal_pose is None:
            return self._goal_pose

        same_frame = self._goal_pose.header.frame_id == self._plan_goal_pose.header.frame_id
        if same_frame:
            dx = self._goal_pose.pose.position.x - self._plan_goal_pose.pose.position.x
            dy = self._goal_pose.pose.position.y - self._plan_goal_pose.pose.position.y
            if math.hypot(dx, dy) <= 0.30:
                return self._goal_pose

        if self._goal_pose_rx_time >= self._plan_goal_pose_rx_time:
            return self._goal_pose
        return self._plan_goal_pose

    def _set_align_status(
        self,
        *,
        final_align_active: bool,
        reason: str,
        linear_x_cmd: float,
        angular_z_cmd: float,
        goal_pose: Optional[PoseStamped] = None,
        distance_m: Optional[float] = None,
        yaw_error_rad: Optional[float] = None,
    ) -> None:
        self._last_status.update(
            {
                'final_align_active': bool(final_align_active),
                'goal_available': goal_pose is not None,
                'goal_frame': goal_pose.header.frame_id if goal_pose is not None else '',
                'distance_m': None if distance_m is None else round(distance_m, 4),
                'yaw_error_rad': None if yaw_error_rad is None else round(yaw_error_rad, 4),
                'yaw_error_deg': None
                if yaw_error_rad is None
                else round(math.degrees(yaw_error_rad), 2),
                'linear_x_cmd': round(linear_x_cmd, 4),
                'angular_z_cmd': round(angular_z_cmd, 4),
                'reason': reason,
            }
        )

    def _publish_status(
        self,
        step_x: float,
        step_y: float,
        *,
        force: bool = False,
    ) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if not force and (now - self._last_status_pub_sec) < 0.20:
            return
        self._last_status_pub_sec = now
        self._last_status['step_x'] = round(step_x, 4)
        self._last_status['step_y'] = round(step_y, 4)
        msg = String()
        msg.data = json.dumps(self._last_status, ensure_ascii=False)
        self._status_pub.publish(msg)

    def _maybe_apply_final_alignment(
        self, linear_x: float, angular_z: float
    ) -> Tuple[float, float]:
        if not self._final_align_enabled:
            self._final_align_active = False
            self._set_align_status(
                final_align_active=False,
                reason='disabled',
                linear_x_cmd=linear_x,
                angular_z_cmd=angular_z,
            )
            return linear_x, angular_z

        goal_pose = self._get_active_goal_pose()
        if goal_pose is None or not goal_pose.header.frame_id:
            self._final_align_active = False
            self._set_align_status(
                final_align_active=False,
                reason='no_goal',
                linear_x_cmd=linear_x,
                angular_z_cmd=angular_z,
                goal_pose=goal_pose,
            )
            return linear_x, angular_z

        try:
            transform = self._tf_buffer.lookup_transform(
                goal_pose.header.frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._final_align_lookup_timeout_sec),
            )
        except TransformException:
            self._final_align_active = False
            self._set_align_status(
                final_align_active=False,
                reason='tf_unavailable',
                linear_x_cmd=linear_x,
                angular_z_cmd=angular_z,
                goal_pose=goal_pose,
            )
            return linear_x, angular_z

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        dist = math.hypot(goal_x - robot_x, goal_y - robot_y)

        robot_q = transform.transform.rotation
        robot_yaw = quaternion_to_yaw(robot_q.x, robot_q.y, robot_q.z, robot_q.w)
        goal_q = goal_pose.pose.orientation
        goal_yaw = quaternion_to_yaw(goal_q.x, goal_q.y, goal_q.z, goal_q.w)
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        if self._final_align_active:
            # 已经进入 final_align：只检查角度，不管位置偏移。
            final_align = abs(yaw_error) >= self._final_align_yaw_exit
        else:
            # 第一次进入：只要距离到位就触发 final_align，不管角度多大。
            # 避免 Nav2 RPP 在终点附近因朝向不对而绕圈。
            final_align = dist <= self._final_align_xy_trigger

        if not final_align:
            self._final_align_active = False
            self._set_align_status(
                final_align_active=False,
                reason='tracking_path',
                linear_x_cmd=linear_x,
                angular_z_cmd=angular_z,
                goal_pose=goal_pose,
                distance_m=dist,
                yaw_error_rad=yaw_error,
            )
            return linear_x, angular_z

        # 纯原地旋转，线速度强制为 0
        linear_x = 0.0

        # 超调保护：如果误差已经小于 min_step 能分辨的精度，直接停止。
        # 避免 "最小步长大于 exit 阈值" 导致的永久震荡。
        min_effective_yaw = max(
            self._final_align_yaw_exit,
            self._min_nonzero_angular_z / max(self._final_align_angular_kp, 0.1),
        )
        if abs(yaw_error) <= min_effective_yaw:
            self._final_align_active = False
            self._set_align_status(
                final_align_active=False,
                reason='aligned_within_effective_deadband',
                linear_x_cmd=linear_x,
                angular_z_cmd=0.0,
                goal_pose=goal_pose,
                distance_m=dist,
                yaw_error_rad=yaw_error,
            )
            return linear_x, 0.0

        min_turn = max(self._min_nonzero_angular_z, self._deadzone)
        angular_mag = clamp(
            abs(self._final_align_angular_kp * yaw_error),
            min_turn,
            self._max_angular_z,
        )
        angular_z = angular_mag if yaw_error >= 0.0 else -angular_mag

        if not self._final_align_active:
            self.get_logger().info(
                f'Final alignment assist active: dist={dist:.3f} m, yaw_error={yaw_error:.3f} rad'
            )
        self._final_align_active = True
        self._set_align_status(
            final_align_active=True,
            reason='final_align',
            linear_x_cmd=linear_x,
            angular_z_cmd=angular_z,
            goal_pose=goal_pose,
            distance_m=dist,
            yaw_error_rad=yaw_error,
        )
        return linear_x, angular_z

    def _publish_stop(self) -> None:
        self._last_angular_z = 0.0
        self._last_step_x = 0.0
        self._last_step_y = 0.0
        out = MoveCmd()
        out.step_x = 0.0
        out.step_y = 0.0
        self._publisher.publish(out)
        self._set_align_status(
            final_align_active=False,
            reason='stop',
            linear_x_cmd=0.0,
            angular_z_cmd=0.0,
        )
        self._publish_status(0.0, 0.0, force=True)

    def _distance_to_active_goal(self) -> Tuple[Optional[PoseStamped], Optional[float]]:
        goal_pose = self._get_active_goal_pose()
        if goal_pose is None or not goal_pose.header.frame_id:
            return goal_pose, None

        try:
            transform = self._tf_buffer.lookup_transform(
                goal_pose.header.frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._final_align_lookup_timeout_sec),
            )
        except TransformException:
            return goal_pose, None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        return goal_pose, math.hypot(goal_x - robot_x, goal_y - robot_y)

    def _motion_lock_callback(self, msg: Bool) -> None:
        locked = bool(msg.data)
        if locked and not self._motion_locked:
            self.get_logger().info('Base motion locked; publishing zero /move_cmd.')
        elif not locked and self._motion_locked:
            self.get_logger().info('Base motion unlocked.')
        self._motion_locked = locked
        if self._motion_locked:
            self._publish_stop()

    def _watchdog_callback(self) -> None:
        if self._motion_locked:
            self._publish_stop()
            return
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

        if self._motion_locked:
            self._publish_stop()
            return

        linear_requested = (
            abs(msg.linear.x) >= self._deadzone
            or abs(msg.linear.y) >= self._deadzone
            or abs(msg.linear.z) >= self._deadzone
        )
        if self._goal_stop_guard_enabled and linear_requested:
            goal_pose, goal_dist = self._distance_to_active_goal()
            if goal_dist is not None and goal_dist <= self._goal_stop_xy_tolerance:
                self._set_align_status(
                    final_align_active=False,
                    reason='goal_stop_guard',
                    linear_x_cmd=0.0,
                    angular_z_cmd=0.0,
                    goal_pose=goal_pose,
                    distance_m=goal_dist,
                )
                self._publish_stop()
                return

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
        linear_x, angular_z = self._maybe_apply_final_alignment(linear_x, angular_z)
        mostly_straight = abs(angular_z) <= self._min_nonzero_angular_linear_x_threshold
        if mostly_straight and 0.0 < abs(linear_x) < self._min_nonzero_linear_x:
            linear_x = self._min_nonzero_linear_x if linear_x > 0.0 else -self._min_nonzero_linear_x
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
        self._publish_status(step_x, step_y)


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
