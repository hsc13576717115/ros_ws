#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


def yaw_to_quat_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PresetWaypointMission(Node):
    def __init__(self) -> None:
        super().__init__('preset_waypoint_mission')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('spin_action_name', '/spin')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('loop_mission', False)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('retry_per_waypoint', 1)
        self.declare_parameter('pause_after_reach_sec', 0.2)
        self.declare_parameter('same_position_tolerance', 0.05)
        self.declare_parameter('same_yaw_tolerance_deg', 5.0)
        self.declare_parameter('spin_time_allowance_sec', 20.0)
        self.declare_parameter('spin_lookup_timeout_sec', 0.10)
        self.declare_parameter('markers_topic', '/preset_waypoints')
        self.declare_parameter('route_topic', '/preset_route')
        self.declare_parameter('current_goal_topic', '/preset_current_goal')
        self.declare_parameter('marker_point_scale', 0.22)
        self.declare_parameter('marker_text_scale', 0.18)

        self._waypoint_file = str(self.get_parameter('waypoint_file').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._action_name = str(self.get_parameter('action_name').value)
        self._spin_action_name = str(self.get_parameter('spin_action_name').value)
        self._auto_start = self._to_bool(self.get_parameter('auto_start').value)
        self._loop_mission = self._to_bool(self.get_parameter('loop_mission').value)
        self._stop_on_failure = self._to_bool(
            self.get_parameter('stop_on_failure').value
        )
        self._retry_per_waypoint = int(self.get_parameter('retry_per_waypoint').value)
        self._pause_after_reach_sec = float(
            self.get_parameter('pause_after_reach_sec').value
        )
        self._same_position_tolerance = max(
            0.0, float(self.get_parameter('same_position_tolerance').value)
        )
        self._same_yaw_tolerance = math.radians(
            float(self.get_parameter('same_yaw_tolerance_deg').value)
        )
        self._spin_time_allowance_sec = max(
            0.0, float(self.get_parameter('spin_time_allowance_sec').value)
        )
        self._spin_lookup_timeout_sec = max(
            0.0, float(self.get_parameter('spin_lookup_timeout_sec').value)
        )
        self._marker_point_scale = float(
            self.get_parameter('marker_point_scale').value
        )
        self._marker_text_scale = float(self.get_parameter('marker_text_scale').value)

        markers_topic = str(self.get_parameter('markers_topic').value)
        route_topic = str(self.get_parameter('route_topic').value)
        current_goal_topic = str(self.get_parameter('current_goal_topic').value)

        self._marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._route_pub = self.create_publisher(Path, route_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, current_goal_topic, 10)
        self._action_client = ActionClient(self, NavigateToPose, self._action_name)
        self._spin_action_client = ActionClient(self, Spin, self._spin_action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._waypoints: List[Waypoint] = self._load_waypoints(self._waypoint_file)
        self._index = 0
        self._retry_count = 0
        self._mission_started = self._auto_start
        self._mission_done = len(self._waypoints) == 0
        self._goal_in_flight = False
        self._next_send_time_sec = self._now_sec()
        self._last_wait_server_log_sec = 0.0
        self._active_goal_kind = 'navigate'

        self.create_timer(0.2, self._tick)
        self.create_timer(0.5, self._publish_visualization)

        self.get_logger().info(
            f'Preset waypoint mission ready. waypoints={len(self._waypoints)}, '
            f'action={self._action_name}, spin_action={self._spin_action_name}, '
            f'frame={self._frame_id}, auto_start={self._auto_start}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _to_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _load_waypoints(self, file_path: str) -> List[Waypoint]:
        if not file_path:
            self.get_logger().error('Parameter waypoint_file is empty.')
            return []
        if not os.path.exists(file_path):
            self.get_logger().error(f'Waypoint file does not exist: {file_path}')
            return []

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to load waypoint file: {exc}')
            return []

        raw_items = data.get('waypoints', data if isinstance(data, list) else [])
        if not isinstance(raw_items, list):
            self.get_logger().error('Invalid waypoint file format: waypoints must be a list.')
            return []

        waypoints: List[Waypoint] = []
        for i, item in enumerate(raw_items):
            if not isinstance(item, dict):
                continue
            if not self._to_bool(item.get('enabled', True)):
                continue
            try:
                x = float(item.get('x', 0.0))
                y = float(item.get('y', 0.0))
                if 'yaw' in item:
                    yaw = float(item['yaw'])
                else:
                    yaw = math.radians(float(item.get('yaw_deg', 0.0)))
            except Exception:
                self.get_logger().warn(f'Skip invalid waypoint at index {i}.')
                continue
            name = str(item.get('name', f'P{i + 1:02d}'))
            waypoints.append(Waypoint(name=name, x=x, y=y, yaw=yaw))

        return waypoints

    def _build_pose(self, wp: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat_z_w(wp.yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _get_previous_waypoint(self) -> Optional[Waypoint]:
        if self._index <= 0:
            return None
        return self._waypoints[self._index - 1]

    def _should_spin_to_current_waypoint(self) -> bool:
        previous = self._get_previous_waypoint()
        if previous is None or self._index >= len(self._waypoints):
            return False

        current = self._waypoints[self._index]
        position_delta = math.hypot(current.x - previous.x, current.y - previous.y)
        yaw_delta = abs(normalize_angle(current.yaw - previous.yaw))
        return (
            position_delta <= self._same_position_tolerance
            and yaw_delta > self._same_yaw_tolerance
        )

    def _get_robot_yaw(self) -> Optional[float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._spin_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Failed to lookup {self._frame_id} -> {self._base_frame} for spin goal: {exc}'
            )
            return None

        q = transform.transform.rotation
        return quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _wait_for_current_action_server(self, now_sec: float) -> bool:
        action_client = self._spin_action_client if self._should_spin_to_current_waypoint() else self._action_client
        action_name = self._spin_action_name if action_client is self._spin_action_client else self._action_name
        action_label = 'Spin' if action_client is self._spin_action_client else 'NavigateToPose'

        if action_client.wait_for_server(timeout_sec=0.0):
            return True

        if (now_sec - self._last_wait_server_log_sec) > 2.0:
            self._last_wait_server_log_sec = now_sec
            self.get_logger().info(
                f'Waiting for {action_label} action server: {action_name}'
            )
        return False

    def _tick(self) -> None:
        if self._mission_done or not self._mission_started or not self._waypoints:
            return
        if self._goal_in_flight:
            return
        now_sec = self._now_sec()
        if now_sec < self._next_send_time_sec:
            return

        if self._index >= len(self._waypoints):
            if self._loop_mission:
                self._index = 0
                self._retry_count = 0
                self.get_logger().info('Mission loop enabled, restarting from waypoint 1.')
            else:
                self._mission_done = True
                self.get_logger().info('Preset waypoint mission completed.')
                return

        if not self._wait_for_current_action_server(now_sec):
            return

        self._send_current_goal()

    def _send_current_goal(self) -> None:
        wp = self._waypoints[self._index]
        goal_pose = self._build_pose(wp)
        self._goal_pub.publish(goal_pose)

        if self._should_spin_to_current_waypoint():
            self._send_spin_goal(wp)
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(
            f'Sending waypoint [{self._index + 1}/{len(self._waypoints)}] '
            f'{wp.name}: x={wp.x:.3f}, y={wp.y:.3f}, yaw={wp.yaw:.3f}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._active_goal_kind = 'navigate'
        self._goal_in_flight = True

    def _send_spin_goal(self, wp: Waypoint) -> None:
        robot_yaw = self._get_robot_yaw()
        if robot_yaw is None:
            self._goal_in_flight = False
            self._next_send_time_sec = self._now_sec() + 1.0
            return

        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(normalize_angle(wp.yaw - robot_yaw))
        spin_goal.time_allowance = Duration(
            seconds=self._spin_time_allowance_sec
        ).to_msg()
        self.get_logger().info(
            f'Sending spin waypoint [{self._index + 1}/{len(self._waypoints)}] '
            f'{wp.name}: x={wp.x:.3f}, y={wp.y:.3f}, target_yaw={wp.yaw:.3f}, '
            f'spin_delta={spin_goal.target_yaw:.3f}'
        )
        future = self._spin_action_client.send_goal_async(spin_goal)
        future.add_done_callback(self._on_spin_goal_response)
        self._active_goal_kind = 'spin'
        self._goal_in_flight = True

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send waypoint goal: {exc}')
            self._goal_in_flight = False
            self._next_send_time_sec = self._now_sec() + 1.0
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected by Nav2.')
            self._handle_goal_failure('rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_spin_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send spin goal: {exc}')
            self._goal_in_flight = False
            self._next_send_time_sec = self._now_sec() + 1.0
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected by Nav2.')
            self._handle_goal_failure('spin_rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN
        except Exception as exc:
            self.get_logger().error(f'Failed to get waypoint result: {exc}')
            self._handle_goal_failure('result_exception')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            wp = self._waypoints[self._index]
            self.get_logger().info(
                f'Waypoint reached [{self._index + 1}/{len(self._waypoints)}] '
                f'({self._active_goal_kind}): {wp.name}'
            )
            self._index += 1
            self._retry_count = 0
            self._goal_in_flight = False
            self._active_goal_kind = 'navigate'
            self._next_send_time_sec = self._now_sec() + self._pause_after_reach_sec
            return

        self._handle_goal_failure(f'status={status}')

    def _handle_goal_failure(self, reason: str) -> None:
        wp = self._waypoints[self._index]
        if self._retry_count < self._retry_per_waypoint:
            self._retry_count += 1
            self._goal_in_flight = False
            self._next_send_time_sec = self._now_sec() + 1.0
            self.get_logger().warn(
                f'Waypoint failed ({reason}), retry {self._retry_count}/{self._retry_per_waypoint}: {wp.name}'
            )
            return

        self.get_logger().error(f'Waypoint failed ({reason}): {wp.name}')
        self._goal_in_flight = False
        self._retry_count = 0
        self._active_goal_kind = 'navigate'

        if self._stop_on_failure:
            self._mission_done = True
            self.get_logger().error('Mission stopped due to waypoint failure.')
            return

        self._index += 1
        self._next_send_time_sec = self._now_sec() + 0.2

    def _publish_visualization(self) -> None:
        if not self._waypoints:
            return

        stamp = self.get_clock().now().to_msg()

        route = Path()
        route.header.stamp = stamp
        route.header.frame_id = self._frame_id
        for wp in self._waypoints:
            route.poses.append(self._build_pose(wp))
        self._route_pub.publish(route)

        markers = MarkerArray()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self._frame_id
        clear.ns = 'preset_clear'
        clear.id = 0
        clear.type = Marker.SPHERE
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self._frame_id
        line.ns = 'preset_route'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.a = 0.95
        line.color.r = 0.0
        line.color.g = 0.9
        line.color.b = 0.95
        for wp in self._waypoints:
            pt = Point()
            pt.x = wp.x
            pt.y = wp.y
            pt.z = 0.05
            line.points.append(pt)
        markers.markers.append(line)

        points = Marker()
        points.header.stamp = stamp
        points.header.frame_id = self._frame_id
        points.ns = 'preset_waypoints'
        points.id = 2
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = self._marker_point_scale
        points.scale.y = self._marker_point_scale
        points.scale.z = self._marker_point_scale
        points.color.a = 0.95
        points.color.r = 0.1
        points.color.g = 0.35
        points.color.b = 1.0
        for wp in self._waypoints:
            pt = Point()
            pt.x = wp.x
            pt.y = wp.y
            pt.z = 0.10
            points.points.append(pt)
        markers.markers.append(points)

        for i, wp in enumerate(self._waypoints):
            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self._frame_id
            text.ns = 'preset_waypoint_text'
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp.x
            text.pose.position.y = wp.y
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = self._marker_text_scale
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.0
            text.text = f'{i + 1:02d}:{wp.name}'
            markers.markers.append(text)

        if not self._mission_done and self._index < len(self._waypoints):
            current = self._waypoints[self._index]
            current_marker = Marker()
            current_marker.header.stamp = stamp
            current_marker.header.frame_id = self._frame_id
            current_marker.ns = 'preset_current_goal'
            current_marker.id = 3
            current_marker.type = Marker.ARROW
            current_marker.action = Marker.ADD
            current_marker.pose.position.x = current.x
            current_marker.pose.position.y = current.y
            current_marker.pose.position.z = 0.20
            qz, qw = yaw_to_quat_z_w(current.yaw)
            current_marker.pose.orientation.z = qz
            current_marker.pose.orientation.w = qw
            current_marker.scale.x = 0.55
            current_marker.scale.y = 0.12
            current_marker.scale.z = 0.12
            current_marker.color.a = 1.0
            current_marker.color.r = 1.0
            current_marker.color.g = 0.2
            current_marker.color.b = 0.1
            markers.markers.append(current_marker)

        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PresetWaypointMission()
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
