#!/usr/bin/env python3

from typing import Optional

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class OdomReadyActivator(Node):
    def __init__(self) -> None:
        super().__init__('wait_for_odom_activate_nav2')

        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('check_period_sec', 0.2)
        self.declare_parameter('transform_timeout_sec', 0.2)
        self.declare_parameter(
            'lifecycle_service', '/lifecycle_manager_navigation/manage_nodes'
        )
        self.declare_parameter(
            'fallback_lifecycle_service', '/manage_nodes'
        )
        self.declare_parameter('retry_cooldown_sec', 3.0)

        self._target_frame = str(self.get_parameter('target_frame').value)
        self._source_frame = str(self.get_parameter('source_frame').value)
        self._check_period = float(self.get_parameter('check_period_sec').value)
        self._transform_timeout = float(
            self.get_parameter('transform_timeout_sec').value
        )
        lifecycle_service = str(self.get_parameter('lifecycle_service').value)
        fallback_lifecycle_service = str(
            self.get_parameter('fallback_lifecycle_service').value
        )
        self._retry_cooldown_sec = float(self.get_parameter('retry_cooldown_sec').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._primary_client = self.create_client(ManageLifecycleNodes, lifecycle_service)
        self._fallback_client = self.create_client(
            ManageLifecycleNodes, fallback_lifecycle_service
        )
        self._active_client: Optional[rclpy.client.Client] = None

        self._request_sent = False
        self._startup_completed = False
        self._next_retry_time: Optional[Time] = None
        self._last_wait_log_time: Optional[Time] = None
        self._need_reset_before_startup = False

        self._timer = self.create_timer(self._check_period, self._tick)
        self.get_logger().info(
            f'Waiting for TF {self._target_frame} -> {self._source_frame} before Nav2 startup.'
        )
        self.get_logger().info(
            f'Lifecycle services candidates: {lifecycle_service}, {fallback_lifecycle_service}'
        )

    def _resolve_lifecycle_client(self) -> bool:
        if self._primary_client.wait_for_service(timeout_sec=0.0):
            self._active_client = self._primary_client
            return True
        if self._fallback_client.wait_for_service(timeout_sec=0.0):
            self._active_client = self._fallback_client
            return True
        self._active_client = None
        return False

    def _tick(self) -> None:
        if self._startup_completed:
            return

        if self._request_sent:
            return

        now = self.get_clock().now()
        if self._next_retry_time is not None and now < self._next_retry_time:
            return

        if self._last_wait_log_time is None or (
            now - self._last_wait_log_time
        ) > Duration(seconds=2.0):
            if not self._resolve_lifecycle_client():
                self.get_logger().info('Waiting lifecycle manager service...')
                self._last_wait_log_time = now
                return
            self._last_wait_log_time = now

        if self._active_client is None:
            return

        if not self._active_client.service_is_ready():
            return

        try:
            self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException:
            return

        req = ManageLifecycleNodes.Request()
        if self._need_reset_before_startup:
            req.command = ManageLifecycleNodes.Request().RESET
            self.get_logger().warn(
                'Detected partial ACTIVE lifecycle state, sending Nav2 RESET before STARTUP.'
            )
        else:
            req.command = ManageLifecycleNodes.Request().STARTUP
            self.get_logger().info(
                f'TF ready ({self._target_frame} -> {self._source_frame}), sending Nav2 STARTUP.'
            )
        future = self._active_client.call_async(req)
        future.add_done_callback(self._on_startup_result)
        self._request_sent = True

    def _on_startup_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed calling lifecycle manager: {exc}')
            self._request_sent = False
            self._next_retry_time = self.get_clock().now() + Duration(
                seconds=self._retry_cooldown_sec
            )
            return

        if response is not None and response.success:
            if self._need_reset_before_startup:
                self.get_logger().info('Nav2 RESET accepted. Will send STARTUP after cooldown.')
                self._need_reset_before_startup = False
                self._request_sent = False
                self._next_retry_time = self.get_clock().now() + Duration(
                    seconds=self._retry_cooldown_sec
                )
            else:
                self.get_logger().info('Nav2 startup request accepted.')
                self.get_logger().info(
                    'Lifecycle manager accepted STARTUP. Wait for "Managed nodes are active"; '
                    'then preset_waypoint_mission should send the first waypoint.'
                )
                self._startup_completed = True
            return

        self.get_logger().error('Nav2 startup request failed, will retry after cooldown.')
        self._request_sent = False
        self._next_retry_time = self.get_clock().now() + Duration(
            seconds=self._retry_cooldown_sec
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomReadyActivator()
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
