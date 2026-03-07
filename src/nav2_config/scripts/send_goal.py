#!/usr/bin/env python3

import argparse
import math
import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavigateGoalSender(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__('nav2_goal_sender')
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

    def _resolve_goal_yaw(
        self,
        requested_yaw: Optional[float],
        frame_id: str,
        base_frame: str,
    ) -> float:
        if requested_yaw is not None:
            return requested_yaw

        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'No yaw provided, using current heading yaw={yaw:.3f} rad.'
            )
            return yaw
        except TransformException as exc:
            self.get_logger().warn(
                f'Failed to get current heading ({frame_id} <- {base_frame}): {exc}. '
                'Fallback to yaw=0.0.'
            )
            return 0.0

    def send_goal(
        self,
        x: float,
        y: float,
        yaw: Optional[float],
        frame_id: str,
        base_frame: str,
        timeout_sec: float,
    ) -> int:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('NavigateToPose action server not available.')
            return 2

        final_yaw = self._resolve_goal_yaw(yaw, frame_id, base_frame)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(final_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(final_yaw / 2.0)

        self.get_logger().info(
            f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={final_yaw:.3f} rad, frame={frame_id}'
        )

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to send goal to action server.')
            return 3
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return 4

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('No result returned from action server.')
            return 5

        status = wrapped_result.status
        status_text = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(status, str(status))

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            return 0

        self.get_logger().error(f'Navigation finished with status: {status_text}')
        return 6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send a NavigateToPose goal to Nav2.'
    )
    parser.add_argument('--x', type=float, required=True, help='Goal x (meters)')
    parser.add_argument('--y', type=float, required=True, help='Goal y (meters)')
    parser.add_argument(
        '--yaw',
        type=float,
        default=None,
        help='Goal yaw (radians). If omitted, keep current heading.',
    )
    parser.add_argument(
        '--frame',
        type=str,
        default='map',
        help='Target frame for goal pose',
    )
    parser.add_argument(
        '--base-frame',
        type=str,
        default='base_link',
        help='Robot base frame used to infer heading when --yaw is omitted',
    )
    parser.add_argument(
        '--action-name',
        type=str,
        default='/navigate_to_pose',
        help='NavigateToPose action name',
    )
    parser.add_argument(
        '--server-timeout',
        type=float,
        default=10.0,
        help='Seconds to wait for action server',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = NavigateGoalSender(args.action_name)
    try:
        return node.send_goal(
            x=args.x,
            y=args.y,
            yaw=args.yaw,
            frame_id=args.frame,
            base_frame=args.base_frame,
            timeout_sec=args.server_timeout,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
