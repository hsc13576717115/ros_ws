#!/usr/bin/env python3

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class YoloTask:
    enabled: bool = False
    settle_sec: float = 1.0
    timeout_sec: float = 4.0
    min_score: float = 0.5


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    yolo: YoloTask = field(default_factory=YoloTask)


@dataclass
class YoloObservation:
    count: int = 0
    best_score: float = 0.0


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
        self.declare_parameter('yolo_enable_topic', '/yolo/enable')
        self.declare_parameter('yolo_detection_topic', '/yolo/detections')
        self.declare_parameter('yolo_result_topic', '/preset_yolo_result')
        self.declare_parameter('yolo_result_text_scale', 0.14)

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
        self._yolo_result_text_scale = float(
            self.get_parameter('yolo_result_text_scale').value
        )

        markers_topic = str(self.get_parameter('markers_topic').value)
        route_topic = str(self.get_parameter('route_topic').value)
        current_goal_topic = str(self.get_parameter('current_goal_topic').value)
        yolo_enable_topic = str(self.get_parameter('yolo_enable_topic').value)
        yolo_detection_topic = str(self.get_parameter('yolo_detection_topic').value)
        yolo_result_topic = str(self.get_parameter('yolo_result_topic').value)

        self._marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._route_pub = self.create_publisher(Path, route_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, current_goal_topic, 10)
        self._yolo_enable_pub = self.create_publisher(Bool, yolo_enable_topic, 10)
        self._yolo_result_pub = self.create_publisher(String, yolo_result_topic, 10)
        self._yolo_sub = self.create_subscription(
            Detection2DArray,
            yolo_detection_topic,
            self._yolo_detection_callback,
            20,
        )
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
        self._yolo_enabled = False
        self._yolo_task_active = False
        self._yolo_collect_start_sec = 0.0
        self._yolo_collect_deadline_sec = 0.0
        self._yolo_task_index: Optional[int] = None
        self._yolo_observations: Dict[str, YoloObservation] = {}
        self._yolo_result_by_waypoint: Dict[str, str] = {}
        self._yolo_has_detection_by_waypoint: Dict[str, bool] = {}
        self._yolo_default_task = YoloTask()

        self._set_yolo_enabled(False, force=True)

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

    def _parse_yolo_defaults(self, data) -> YoloTask:
        cfg = data if isinstance(data, dict) else {}
        settle_sec = max(0.0, float(cfg.get('settle_sec', 1.0)))
        timeout_sec = max(0.1, float(cfg.get('timeout_sec', 4.0)))
        min_score = float(cfg.get('min_score', 0.5))
        min_score = max(0.0, min(1.0, min_score))
        return YoloTask(
            enabled=False,
            settle_sec=settle_sec,
            timeout_sec=timeout_sec,
            min_score=min_score,
        )

    def _parse_yolo_task(self, item: dict) -> YoloTask:
        defaults = self._yolo_default_task
        raw_cfg = item.get('yolo', item.get('yolo_enabled', False))
        enabled = self._to_bool(raw_cfg)
        settle_sec = defaults.settle_sec
        timeout_sec = defaults.timeout_sec
        min_score = defaults.min_score

        # Keep compatibility with the previous waypoint-local object form.
        if isinstance(raw_cfg, dict):
            enabled = self._to_bool(raw_cfg.get('enabled', False))
            settle_sec = max(
                0.0,
                float(raw_cfg.get('settle_sec', item.get('yolo_settle_sec', settle_sec))),
            )
            timeout_sec = max(
                0.1,
                float(raw_cfg.get('timeout_sec', item.get('yolo_timeout_sec', timeout_sec))),
            )
            min_score = float(raw_cfg.get('min_score', item.get('yolo_min_score', min_score)))
            min_score = max(0.0, min(1.0, min_score))

        return YoloTask(
            enabled=enabled,
            settle_sec=settle_sec,
            timeout_sec=timeout_sec,
            min_score=min_score,
        )

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

        self._yolo_default_task = self._parse_yolo_defaults(
            data.get('yolo', {}) if isinstance(data, dict) else {}
        )

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
            waypoints.append(
                Waypoint(
                    name=name,
                    x=x,
                    y=y,
                    yaw=yaw,
                    yolo=self._parse_yolo_task(item),
                )
            )

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
        action_client = (
            self._spin_action_client
            if self._should_spin_to_current_waypoint()
            else self._action_client
        )
        action_name = (
            self._spin_action_name
            if action_client is self._spin_action_client
            else self._action_name
        )
        action_label = (
            'Spin' if action_client is self._spin_action_client else 'NavigateToPose'
        )

        if action_client.wait_for_server(timeout_sec=0.0):
            return True

        if (now_sec - self._last_wait_server_log_sec) > 2.0:
            self._last_wait_server_log_sec = now_sec
            self.get_logger().info(
                f'Waiting for {action_label} action server: {action_name}'
            )
        return False

    def _set_yolo_enabled(self, enabled: bool, force: bool = False) -> None:
        if not force and self._yolo_enabled == enabled:
            return

        self._yolo_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self._yolo_enable_pub.publish(msg)
        self.get_logger().info(
            f'YOLO detection {"enabled" if enabled else "disabled"} for mission flow.'
        )

    def _tick(self) -> None:
        if self._mission_done or not self._mission_started or not self._waypoints:
            return
        if self._goal_in_flight:
            return
        if self._yolo_task_active:
            self._tick_yolo_task()
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
                self._set_yolo_enabled(False)
                self.get_logger().info('Preset waypoint mission completed.')
                return

        if not self._wait_for_current_action_server(now_sec):
            return

        self._send_current_goal()

    def _tick_yolo_task(self) -> None:
        now_sec = self._now_sec()
        if now_sec < self._yolo_collect_deadline_sec:
            return
        self._finish_yolo_task()

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

    def _start_yolo_task(self, wp: Waypoint) -> None:
        now_sec = self._now_sec()
        self._yolo_task_active = True
        self._yolo_task_index = self._index
        self._yolo_collect_start_sec = now_sec + wp.yolo.settle_sec
        self._yolo_collect_deadline_sec = self._yolo_collect_start_sec + wp.yolo.timeout_sec
        self._yolo_observations.clear()
        self._active_goal_kind = 'yolo'
        self._set_yolo_enabled(True)
        self.get_logger().info(
            f'Starting YOLO task at waypoint {wp.name}: '
            f'settle={wp.yolo.settle_sec:.1f}s, '
            f'window={wp.yolo.timeout_sec:.1f}s, '
            f'min_score={wp.yolo.min_score:.2f}'
        )

    def _build_yolo_result_payload(self, wp: Waypoint) -> dict:
        detections = [
            {
                'class_id': class_id,
                'count': observation.count,
                'best_score': round(observation.best_score, 3),
            }
            for class_id, observation in sorted(
                self._yolo_observations.items(),
                key=lambda item: (-item[1].best_score, item[0]),
            )
        ]
        has_detection = bool(detections)
        summary = (
            ', '.join(
                f'{item["class_id"]}@{item["best_score"]:.2f} x{item["count"]}'
                for item in detections
            )
            if detections
            else 'none'
        )

        return {
            'waypoint': wp.name,
            'x': round(wp.x, 3),
            'y': round(wp.y, 3),
            'yaw': round(wp.yaw, 3),
            'success': True,
            'has_detection': has_detection,
            'min_score': round(wp.yolo.min_score, 3),
            'detections': detections,
            'summary': summary,
        }

    def _finish_yolo_task(self) -> None:
        if self._yolo_task_index is None or self._yolo_task_index >= len(self._waypoints):
            self._yolo_task_active = False
            self._set_yolo_enabled(False)
            return

        wp = self._waypoints[self._yolo_task_index]
        payload = self._build_yolo_result_payload(wp)
        payload_json = json.dumps(payload, ensure_ascii=False)

        msg = String()
        msg.data = payload_json
        self._yolo_result_pub.publish(msg)

        self._yolo_result_by_waypoint[wp.name] = payload['summary']
        self._yolo_has_detection_by_waypoint[wp.name] = bool(payload['has_detection'])

        if payload['has_detection']:
            self.get_logger().info(
                f'YOLO task finished at {wp.name}: {payload["summary"]}'
            )
        else:
            self.get_logger().info(
                f'YOLO task finished at {wp.name}: no valid detection in the time window.'
            )

        self._yolo_task_active = False
        self._yolo_task_index = None
        self._yolo_observations.clear()
        self._set_yolo_enabled(False)
        self._index += 1
        self._retry_count = 0
        self._active_goal_kind = 'navigate'
        self._next_send_time_sec = self._now_sec() + self._pause_after_reach_sec

    def _yolo_detection_callback(self, msg: Detection2DArray) -> None:
        if not self._yolo_task_active or self._yolo_task_index is None:
            return
        if self._yolo_task_index >= len(self._waypoints):
            return

        now_sec = self._now_sec()
        if now_sec < self._yolo_collect_start_sec:
            return

        wp = self._waypoints[self._yolo_task_index]
        for detection in msg.detections:
            if not detection.results:
                continue
            hypothesis = max(
                detection.results,
                key=lambda result: float(result.hypothesis.score),
            )
            score = float(hypothesis.hypothesis.score)
            if score < wp.yolo.min_score:
                continue
            class_id = hypothesis.hypothesis.class_id.strip() or 'unknown'
            observation = self._yolo_observations.setdefault(class_id, YoloObservation())
            observation.count += 1
            observation.best_score = max(observation.best_score, score)

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
            self._goal_in_flight = False

            if wp.yolo.enabled:
                self._start_yolo_task(wp)
                return

            self._index += 1
            self._retry_count = 0
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
        self._set_yolo_enabled(False)
        self._yolo_task_active = False
        self._yolo_task_index = None
        self._yolo_observations.clear()

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

        yolo_points = Marker()
        yolo_points.header.stamp = stamp
        yolo_points.header.frame_id = self._frame_id
        yolo_points.ns = 'preset_yolo_waypoints'
        yolo_points.id = 4
        yolo_points.type = Marker.SPHERE_LIST
        yolo_points.action = Marker.ADD
        yolo_points.pose.orientation.w = 1.0
        yolo_points.scale.x = self._marker_point_scale * 0.75
        yolo_points.scale.y = self._marker_point_scale * 0.75
        yolo_points.scale.z = self._marker_point_scale * 0.75
        yolo_points.color.a = 0.98
        yolo_points.color.r = 1.0
        yolo_points.color.g = 0.45
        yolo_points.color.b = 0.0
        for wp in self._waypoints:
            if not wp.yolo.enabled:
                continue
            pt = Point()
            pt.x = wp.x
            pt.y = wp.y
            pt.z = 0.18
            yolo_points.points.append(pt)
        if yolo_points.points:
            markers.markers.append(yolo_points)

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
            label = f'{i + 1:02d}:{wp.name}'
            if wp.yolo.enabled:
                label += ' [YOLO]'
            text.text = label
            markers.markers.append(text)

            if not wp.yolo.enabled:
                continue

            result = Marker()
            result.header.stamp = stamp
            result.header.frame_id = self._frame_id
            result.ns = 'preset_yolo_result'
            result.id = 300 + i
            result.type = Marker.TEXT_VIEW_FACING
            result.action = Marker.ADD
            result.pose.position.x = wp.x
            result.pose.position.y = wp.y
            result.pose.position.z = 0.55
            result.pose.orientation.w = 1.0
            result.scale.z = self._yolo_result_text_scale
            result.color.a = 1.0
            summary = self._yolo_result_by_waypoint.get(wp.name)
            if self._yolo_task_active and self._yolo_task_index == i:
                result.color.r = 1.0
                result.color.g = 0.6
                result.color.b = 0.0
                result.text = 'YOLO: running...'
            elif summary:
                if self._yolo_has_detection_by_waypoint.get(wp.name, False):
                    result.color.r = 0.1
                    result.color.g = 1.0
                    result.color.b = 0.1
                else:
                    result.color.r = 0.8
                    result.color.g = 0.8
                    result.color.b = 0.8
                result.text = f'YOLO: {summary}'
            else:
                result.color.r = 0.8
                result.color.g = 0.8
                result.color.b = 0.8
                result.text = 'YOLO: pending'
            markers.markers.append(result)

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
