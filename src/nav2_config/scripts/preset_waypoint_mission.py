#!/usr/bin/env python3

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

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
from r2_arm_control.srv import SetArmState
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
class ArmTask:
    enabled: bool = False
    state: str = ''
    timeout_sec: float = 20.0
    continue_on_failure: bool = False


@dataclass
class PlaceTask:
    enabled: bool = False
    target: str = 'auto'
    transfer_x: float = 3.85
    side_aisle_y: float = 1.55
    return_to_next_pick: bool = True


@dataclass
class PlaceTarget:
    name: str
    x: float
    y: float
    yaw: float
    aliases: List[str] = field(default_factory=list)


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    yolo: YoloTask = field(default_factory=YoloTask)
    arm: ArmTask = field(default_factory=ArmTask)
    place: PlaceTask = field(default_factory=PlaceTask)


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
        self.declare_parameter('skip_already_reached_nav_goals', True)
        self.declare_parameter('already_reached_xy_tolerance', 0.18)
        self.declare_parameter('already_reached_yaw_tolerance_deg', 12.0)
        self.declare_parameter('spin_time_allowance_sec', 20.0)
        self.declare_parameter('spin_lookup_timeout_sec', 0.10)
        self.declare_parameter('markers_topic', '/preset_waypoints')
        self.declare_parameter('route_topic', '/preset_route')
        self.declare_parameter('current_goal_topic', '/preset_current_goal')
        self.declare_parameter('marker_point_scale', 0.22)
        self.declare_parameter('marker_text_scale', 0.18)
        self.declare_parameter('publish_field_layout', True)
        self.declare_parameter('publish_task_item_zones', True)
        self.declare_parameter('yolo_enable_topic', '/yolo/enable')
        self.declare_parameter('yolo_detection_topic', '/yolo/detections')
        self.declare_parameter('yolo_result_topic', '/preset_yolo_result')
        self.declare_parameter('yolo_result_text_scale', 0.14)
        self.declare_parameter('arm_state_service_name', '/r2/arm/set_state')
        self.declare_parameter('arm_state_topic', '/r2/arm/state_machine_state')
        self.declare_parameter('arm_default_timeout_sec', 20.0)

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
        self._skip_already_reached_nav_goals = self._to_bool(
            self.get_parameter('skip_already_reached_nav_goals').value
        )
        self._already_reached_xy_tolerance = max(
            0.0, float(self.get_parameter('already_reached_xy_tolerance').value)
        )
        self._already_reached_yaw_tolerance = math.radians(
            float(self.get_parameter('already_reached_yaw_tolerance_deg').value)
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
        self._publish_field_layout = self._to_bool(
            self.get_parameter('publish_field_layout').value
        )
        self._publish_task_item_zones = self._to_bool(
            self.get_parameter('publish_task_item_zones').value
        )
        self._yolo_result_text_scale = float(
            self.get_parameter('yolo_result_text_scale').value
        )

        markers_topic = str(self.get_parameter('markers_topic').value)
        route_topic = str(self.get_parameter('route_topic').value)
        current_goal_topic = str(self.get_parameter('current_goal_topic').value)
        yolo_enable_topic = str(self.get_parameter('yolo_enable_topic').value)
        yolo_detection_topic = str(self.get_parameter('yolo_detection_topic').value)
        yolo_result_topic = str(self.get_parameter('yolo_result_topic').value)
        arm_state_service_name = str(
            self.get_parameter('arm_state_service_name').value
        )
        self._arm_state_topic = str(self.get_parameter('arm_state_topic').value)
        self._arm_default_timeout_sec = max(
            1.0, float(self.get_parameter('arm_default_timeout_sec').value)
        )

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
        self._arm_state_sub = self.create_subscription(
            String,
            self._arm_state_topic,
            self._arm_state_callback,
            10,
        )
        self._arm_client = self.create_client(SetArmState, arm_state_service_name)
        self._action_client = ActionClient(self, NavigateToPose, self._action_name)
        self._spin_action_client = ActionClient(self, Spin, self._spin_action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._place_targets: Dict[str, PlaceTarget] = {}
        self._class_alias_to_place_target: Dict[str, str] = {}
        self._dynamic_place_inserted_for: Set[str] = set()
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
        self._yolo_best_class_by_waypoint: Dict[str, str] = {}
        self._yolo_default_task = YoloTask()
        self._arm_task_active = False
        self._arm_task_index: Optional[int] = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_deadline_sec = 0.0
        self._arm_future = None
        self._arm_current_state = ''
        self._arm_state_update_sec = 0.0
        self._last_wait_arm_service_log_sec = 0.0

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

    def _parse_arm_task(self, item: dict) -> ArmTask:
        raw_cfg = item.get('arm', item.get('arm_state', False))
        enabled = False
        state = ''
        timeout_sec = self._arm_default_timeout_sec
        continue_on_failure = False

        if isinstance(raw_cfg, dict):
            state = str(raw_cfg.get('state', '')).strip().lower()
            enabled = self._to_bool(raw_cfg.get('enabled', bool(state)))
            timeout_sec = max(
                1.0, float(raw_cfg.get('timeout_sec', timeout_sec))
            )
            continue_on_failure = self._to_bool(
                raw_cfg.get('continue_on_failure', False)
            )
        elif isinstance(raw_cfg, str):
            state = raw_cfg.strip().lower()
            enabled = state not in ('', 'none', 'false', 'off', 'skip', 'null')
        else:
            enabled = self._to_bool(raw_cfg)
            state = str(item.get('arm_state', '')).strip().lower()

        if enabled and not state:
            state = 'idle'

        return ArmTask(
            enabled=enabled,
            state=state,
            timeout_sec=timeout_sec,
            continue_on_failure=continue_on_failure,
        )

    def _parse_place_task(self, item: dict) -> PlaceTask:
        raw_cfg = item.get('place', item.get('place_after_pick', False))
        enabled = False
        target = 'auto'
        transfer_x = 3.85
        side_aisle_y = 1.55
        return_to_next_pick = True

        if isinstance(raw_cfg, dict):
            enabled = self._to_bool(raw_cfg.get('enabled', False))
            target = str(raw_cfg.get('target', target)).strip()
            transfer_x = float(raw_cfg.get('transfer_x', transfer_x))
            side_aisle_y = float(raw_cfg.get('side_aisle_y', side_aisle_y))
            return_to_next_pick = self._to_bool(
                raw_cfg.get('return_to_next_pick', return_to_next_pick)
            )
        elif isinstance(raw_cfg, str):
            target = raw_cfg.strip()
            enabled = target.lower() not in ('', 'none', 'false', 'off', 'skip', 'null')
        else:
            enabled = self._to_bool(raw_cfg)

        if enabled and not target:
            target = 'auto'

        return PlaceTask(
            enabled=enabled,
            target=target,
            transfer_x=transfer_x,
            side_aisle_y=side_aisle_y,
            return_to_next_pick=return_to_next_pick,
        )

    @staticmethod
    def _normalize_class_key(value: str) -> str:
        return str(value).strip().lower().replace(' ', '').replace('_', '').replace('-', '')

    def _is_arm_task_complete(self, requested_state: str) -> bool:
        if self._arm_current_state == requested_state:
            return True
        # Support auto-chain: pick -> store, place -> idle
        if requested_state == 'pick' and self._arm_current_state == 'store':
            return True
        if requested_state == 'place' and self._arm_current_state == 'idle':
            return True
        return False

    def _parse_place_targets(self, data) -> None:
        self._place_targets.clear()
        self._class_alias_to_place_target.clear()

        if not isinstance(data, dict):
            return

        for name, raw_target in data.items():
            if not isinstance(raw_target, dict):
                continue
            try:
                x = float(raw_target.get('x', 0.0))
                y = float(raw_target.get('y', 0.0))
                if 'yaw' in raw_target:
                    yaw = float(raw_target['yaw'])
                else:
                    yaw = math.radians(float(raw_target.get('yaw_deg', 0.0)))
            except Exception:
                self.get_logger().warn(f'Skip invalid place target: {name}')
                continue

            aliases = raw_target.get('aliases', [])
            if isinstance(aliases, str):
                aliases = [aliases]
            aliases = [str(alias) for alias in aliases if str(alias).strip()]

            target = PlaceTarget(str(name), x, y, yaw, aliases)
            self._place_targets[target.name] = target
            for alias in [target.name] + aliases:
                self._class_alias_to_place_target[self._normalize_class_key(alias)] = target.name

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
        self._parse_place_targets(
            data.get('place_targets', {}) if isinstance(data, dict) else {}
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
                    arm=self._parse_arm_task(item),
                    place=self._parse_place_task(item),
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

    @staticmethod
    def _heading_from_delta(dx: float, dy: float, fallback: float = 0.0) -> float:
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return fallback
        return math.atan2(dy, dx)

    def _resolve_place_target(self, wp: Waypoint) -> Optional[PlaceTarget]:
        target_key = wp.place.target.strip()
        if target_key and target_key.lower() not in ('auto', 'detected'):
            target = self._place_targets.get(target_key)
            if target is not None:
                return target
            mapped_name = self._class_alias_to_place_target.get(
                self._normalize_class_key(target_key)
            )
            if mapped_name:
                return self._place_targets.get(mapped_name)
            self.get_logger().warn(
                f'Place target "{target_key}" from waypoint {wp.name} is not configured.'
            )

        detected_class = self._yolo_best_class_by_waypoint.get(wp.name, '')
        mapped_name = self._class_alias_to_place_target.get(
            self._normalize_class_key(detected_class)
        )
        if mapped_name:
            return self._place_targets.get(mapped_name)

        fallback = self._place_targets.get('UNKNOWN_PLACE')
        if fallback is not None:
            self.get_logger().warn(
                f'No place target mapping for detection "{detected_class}" at {wp.name}; '
                f'using UNKNOWN_PLACE.'
            )
            return fallback

        self.get_logger().error(
            f'No place target mapping for detection "{detected_class}" at {wp.name}.'
        )
        return None

    def _make_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        arm_state: str = '',
    ) -> Waypoint:
        return Waypoint(
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            yolo=YoloTask(enabled=False),
            arm=ArmTask(enabled=bool(arm_state), state=arm_state),
            place=PlaceTask(enabled=False),
        )

    def _insert_dynamic_place_route(self, wp: Waypoint) -> bool:
        if not wp.place.enabled or wp.name in self._dynamic_place_inserted_for:
            return False
        if self._index >= len(self._waypoints):
            return False

        target = self._resolve_place_target(wp)
        if target is None:
            return False

        next_wp = (
            self._waypoints[self._index + 1]
            if (self._index + 1) < len(self._waypoints)
            else None
        )

        transfer_x = wp.place.transfer_x
        side_aisle_y = wp.place.side_aisle_y
        route: List[Waypoint] = []

        route.append(
            self._make_waypoint(
                f'{wp.name}_STORE',
                wp.x,
                wp.y,
                wp.yaw,
                'store',
            )
        )
        route.append(
            self._make_waypoint(
                f'{wp.name}_TO_TRANSFER',
                transfer_x,
                wp.y,
                self._heading_from_delta(transfer_x - wp.x, 0.0, wp.yaw),
            )
        )
        route.append(
            self._make_waypoint(
                f'{wp.name}_ALIGN_{target.name}',
                transfer_x,
                target.y,
                self._heading_from_delta(0.0, target.y - wp.y, 0.0),
            )
        )
        route.append(
            self._make_waypoint(
                f'{wp.name}_PLACE_{target.name}',
                target.x,
                target.y,
                target.yaw,
                'place',
            )
        )

        if wp.place.return_to_next_pick and next_wp is not None:
            route.append(
                self._make_waypoint(
                    f'{wp.name}_EXIT_PLACE',
                    transfer_x,
                    target.y,
                    self._heading_from_delta(transfer_x - target.x, 0.0, target.yaw),
                )
            )
            route.append(
                self._make_waypoint(
                    f'{wp.name}_SIDE_AISLE',
                    transfer_x,
                    side_aisle_y,
                    self._heading_from_delta(0.0, side_aisle_y - target.y, 0.0),
                )
            )
            route.append(
                self._make_waypoint(
                    f'{wp.name}_NEXT_ROW',
                    next_wp.x,
                    side_aisle_y,
                    self._heading_from_delta(next_wp.x - transfer_x, 0.0, math.pi),
                )
            )
            route.append(
                self._make_waypoint(
                    f'{wp.name}_ALIGN_NEXT',
                    next_wp.x,
                    next_wp.y,
                    self._heading_from_delta(0.0, next_wp.y - side_aisle_y, 0.0),
                )
            )

        insert_at = self._index + 1
        self._waypoints[insert_at:insert_at] = route
        self._dynamic_place_inserted_for.add(wp.name)
        self.get_logger().info(
            f'Inserted dynamic place route after {wp.name}: target={target.name}, '
            f'inserted_waypoints={len(route)}'
        )
        return True

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
        pose = self._get_robot_pose_for_skip(log_on_failure=True)
        if pose is None:
            return None
        return pose[2]

    def _get_robot_pose_for_skip(
        self,
        log_on_failure: bool = False,
    ) -> Optional[tuple[float, float, float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._spin_lookup_timeout_sec),
            )
        except TransformException as exc:
            if log_on_failure:
                self.get_logger().warn(
                    f'Failed to lookup {self._frame_id} -> {self._base_frame}: {exc}'
                )
            return None

        translation = transform.transform.translation
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return translation.x, translation.y, yaw

    def _maybe_skip_already_reached_waypoint(self) -> bool:
        if not self._skip_already_reached_nav_goals:
            return False
        if self._index >= len(self._waypoints):
            return False

        wp = self._waypoints[self._index]
        if wp.yolo.enabled or wp.arm.enabled:
            return False

        pose = self._get_robot_pose_for_skip()
        if pose is None:
            return False

        robot_x, robot_y, robot_yaw = pose
        xy_delta = math.hypot(wp.x - robot_x, wp.y - robot_y)
        yaw_delta = abs(normalize_angle(wp.yaw - robot_yaw))
        if (
            xy_delta <= self._already_reached_xy_tolerance
            and yaw_delta <= self._already_reached_yaw_tolerance
        ):
            self.get_logger().info(
                f'Skipping already reached waypoint [{self._index + 1}/{len(self._waypoints)}] '
                f'{wp.name}: distance={xy_delta:.3f}m, yaw_delta={yaw_delta:.3f}rad'
            )
            self._complete_waypoint()
            return True

        return False

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
        if self._arm_task_active:
            self._tick_arm_task()
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

        if self._maybe_skip_already_reached_waypoint():
            return

        if not self._wait_for_current_action_server(now_sec):
            return

        self._send_current_goal()

    def _tick_yolo_task(self) -> None:
        now_sec = self._now_sec()
        if now_sec < self._yolo_collect_deadline_sec:
            return
        self._finish_yolo_task()

    def _tick_arm_task(self) -> None:
        if self._arm_task_index is None or self._arm_task_index >= len(self._waypoints):
            self._arm_task_active = False
            return

        now_sec = self._now_sec()
        wp = self._waypoints[self._arm_task_index]
        if now_sec > self._arm_deadline_sec:
            self._handle_arm_failure(wp, 'timeout')
            return

        if not self._arm_request_sent:
            if not self._arm_client.wait_for_service(timeout_sec=0.0):
                if (now_sec - self._last_wait_arm_service_log_sec) > 2.0:
                    self._last_wait_arm_service_log_sec = now_sec
                    self.get_logger().info(
                        'Waiting for arm state service: /r2/arm/set_state'
                    )
                return

            request = SetArmState.Request()
            request.state = wp.arm.state
            self._arm_future = self._arm_client.call_async(request)
            self._arm_request_sent = True
            self._arm_request_sent_sec = now_sec
            self.get_logger().info(
                f'Sending arm task at waypoint {wp.name}: state={wp.arm.state}'
            )
            return

        if self._arm_future is not None and not self._arm_future.done():
            return

        if self._arm_future is not None:
            try:
                response = self._arm_future.result()
            except Exception as exc:
                self._handle_arm_failure(wp, f'service exception: {exc}')
                return

            if response is None:
                self._handle_arm_failure(wp, 'empty response')
                return

            if not response.accepted:
                self._handle_arm_failure(wp, response.message)
                return

            self._arm_future = None
            self.get_logger().info(
                f'Arm task accepted at {wp.name}: requested={wp.arm.state}, '
                f'current={response.state}, {response.message}'
            )
            return

        if (
            self._is_arm_task_complete(wp.arm.state)
            and self._arm_state_update_sec >= self._arm_request_sent_sec
        ):
            self.get_logger().info(
                f'Arm task finished at {wp.name}: state={self._arm_current_state}'
            )
            if wp.arm.state == 'pick':
                self._insert_dynamic_place_route(wp)
            self._finish_arm_task()

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
        best_class = detections[0]['class_id'] if detections else ''
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
            'best_class': best_class,
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
        self._yolo_best_class_by_waypoint[wp.name] = str(payload.get('best_class', ''))

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
        if wp.arm.enabled:
            self._start_arm_task(wp)
            return

        self._complete_waypoint()

    def _start_arm_task(self, wp: Waypoint) -> None:
        now_sec = self._now_sec()
        self._arm_task_active = True
        self._arm_task_index = self._index
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_future = None
        self._arm_deadline_sec = now_sec + wp.arm.timeout_sec
        self._active_goal_kind = 'arm'
        self.get_logger().info(
            f'Starting arm task at waypoint {wp.name}: '
            f'state={wp.arm.state}, timeout={wp.arm.timeout_sec:.1f}s'
        )

    def _finish_arm_task(self) -> None:
        self._arm_task_active = False
        self._arm_task_index = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_future = None
        self._complete_waypoint()

    def _complete_waypoint(self) -> None:
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

    def _arm_state_callback(self, msg: String) -> None:
        self._arm_current_state = msg.data.strip().lower()
        self._arm_state_update_sec = self._now_sec()

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

            if wp.arm.enabled:
                self._start_arm_task(wp)
                return

            self._complete_waypoint()
            return

        self._handle_goal_failure(f'status={status}')

    def _handle_arm_failure(self, wp: Waypoint, reason: str) -> None:
        self.get_logger().error(
            f'Arm task failed at waypoint {wp.name}: state={wp.arm.state}, reason={reason}'
        )
        self._arm_task_active = False
        self._arm_task_index = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_future = None

        if wp.arm.continue_on_failure:
            self.get_logger().warn(
                f'Continuing mission after arm failure at waypoint {wp.name}.'
            )
            self._complete_waypoint()
            return

        self._mission_done = True
        self._set_yolo_enabled(False)
        self.get_logger().error('Mission stopped due to arm task failure.')

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
        self._arm_task_active = False
        self._arm_task_index = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_future = None

        if self._stop_on_failure:
            self._mission_done = True
            self.get_logger().error('Mission stopped due to waypoint failure.')
            return

        self._index += 1
        self._next_send_time_sec = self._now_sec() + 0.2

    def _append_cube_marker(
        self,
        markers: MarkerArray,
        stamp,
        ns: str,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        scale_x: float,
        scale_y: float,
        scale_z: float,
        color: tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        markers.markers.append(marker)

    def _append_text_marker(
        self,
        markers: MarkerArray,
        stamp,
        ns: str,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        text: str,
        scale: float = 0.16,
        color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
    ) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        markers.markers.append(marker)

    def _append_line_strip_marker(
        self,
        markers: MarkerArray,
        stamp,
        ns: str,
        marker_id: int,
        points: List[tuple[float, float, float]],
        scale: float,
        color: tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for x, y, z in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            marker.points.append(pt)
        markers.markers.append(marker)

    def _append_field_layout_markers(self, markers: MarkerArray, stamp) -> None:
        # Task-field dimensions from 2026 V2.0 figure 3/4, in meters.
        # The start-zone center is map (0, 0); the 6m x 4m task field starts at x=0.5.
        start_size = 1.0
        field_length = 6.0
        field_width = 4.0
        field_min_x = start_size * 0.5
        field_max_x = field_min_x + field_length
        field_min_y = -field_width * 0.5
        field_max_y = field_width * 0.5
        field_center_x = (field_min_x + field_max_x) * 0.5
        field_center_y = (field_min_y + field_max_y) * 0.5
        speed_bump_x = field_max_x - 2.50
        speed_bump_width = 0.35

        self._append_cube_marker(
            markers,
            stamp,
            'task_field_floor',
            1,
            field_center_x,
            field_center_y,
            -0.015,
            field_max_x - field_min_x,
            field_max_y - field_min_y,
            0.02,
            (1.0, 0.92, 0.12, 0.18),
        )

        self._append_line_strip_marker(
            markers,
            stamp,
            'task_field_boundary',
            2,
            [
                (field_min_x, field_min_y, 0.04),
                (field_max_x, field_min_y, 0.04),
                (field_max_x, field_max_y, 0.04),
                (field_min_x, field_max_y, 0.04),
                (field_min_x, field_min_y, 0.04),
            ],
            0.04,
            (1.0, 0.55, 0.0, 1.0),
        )

        wall_color = (1.0, 0.55, 0.0, 0.45)
        wall_thickness = 0.05
        self._append_cube_marker(
            markers,
            stamp,
            'task_field_wall',
            1,
            field_center_x,
            field_max_y + wall_thickness * 0.5,
            0.04,
            field_length,
            wall_thickness,
            0.08,
            wall_color,
        )
        self._append_cube_marker(
            markers,
            stamp,
            'task_field_wall',
            2,
            field_center_x,
            field_min_y - wall_thickness * 0.5,
            0.04,
            field_length,
            wall_thickness,
            0.08,
            wall_color,
        )
        self._append_cube_marker(
            markers,
            stamp,
            'task_field_wall',
            3,
            field_max_x + wall_thickness * 0.5,
            field_center_y,
            0.04,
            wall_thickness,
            field_width,
            0.08,
            wall_color,
        )
        entrance_segment = (field_width - start_size) * 0.5
        self._append_cube_marker(
            markers,
            stamp,
            'task_field_wall',
            4,
            field_min_x - wall_thickness * 0.5,
            field_min_y + entrance_segment * 0.5,
            0.04,
            wall_thickness,
            entrance_segment,
            0.08,
            wall_color,
        )
        self._append_cube_marker(
            markers,
            stamp,
            'task_field_wall',
            5,
            field_min_x - wall_thickness * 0.5,
            field_max_y - entrance_segment * 0.5,
            0.04,
            wall_thickness,
            entrance_segment,
            0.08,
            wall_color,
        )

        self._append_cube_marker(
            markers,
            stamp,
            'task_field_start',
            3,
            0.0,
            0.0,
            0.015,
            start_size,
            start_size,
            0.03,
            (1.0, 0.95, 0.0, 0.18),
        )
        self._append_text_marker(
            markers,
            stamp,
            'task_field_text',
            3,
            0.0,
            -0.62,
            0.18,
            'START',
            0.18,
            (1.0, 0.95, 0.0, 1.0),
        )

        self._append_text_marker(
            markers,
            stamp,
            'task_field_dimension_text',
            1,
            field_center_x,
            field_max_y + 0.35,
            0.18,
            'TASK FIELD 6.000m x 4.000m',
            0.16,
            (0.0, 0.0, 0.0, 1.0),
        )
        self._append_text_marker(
            markers,
            stamp,
            'task_field_dimension_text',
            2,
            field_min_x - 0.35,
            field_center_y,
            0.18,
            '6.000m',
            0.14,
            (0.0, 0.0, 0.0, 1.0),
        )
        self._append_text_marker(
            markers,
            stamp,
            'task_field_dimension_text',
            3,
            0.0,
            0.62,
            0.18,
            'START 1.000m x 1.000m',
            0.12,
            (0.0, 0.0, 0.0, 1.0),
        )

        stripe_count = 14
        stripe_width_y = (field_max_y - field_min_y) / stripe_count
        for i in range(stripe_count):
            y = field_min_y + stripe_width_y * (i + 0.5)
            color = (1.0, 0.85, 0.0, 0.75) if i % 2 == 0 else (0.02, 0.02, 0.02, 0.75)
            self._append_cube_marker(
                markers,
                stamp,
                'task_field_speed_bump',
                10 + i,
                speed_bump_x,
                y,
                0.035,
                speed_bump_width,
                stripe_width_y,
                0.07,
                color,
            )
        self._append_text_marker(
            markers,
            stamp,
            'task_field_text',
            4,
            speed_bump_x,
            -1.75,
            0.22,
            'SPEED BUMP',
            0.14,
            (0.0, 0.0, 0.0, 1.0),
        )
        self._append_text_marker(
            markers,
            stamp,
            'task_field_dimension_text',
            4,
            speed_bump_x,
            1.75,
            0.22,
            '2.500m FROM FAR WALL',
            0.11,
            (0.0, 0.0, 0.0, 1.0),
        )

        if self._publish_task_item_zones:
            storage_size = 0.25
            storage_center_spacing = storage_size + 0.60
            storage_near_speed_bump_x = field_min_x + 1.35
            storage_centers_x = [
                storage_near_speed_bump_x,
                storage_near_speed_bump_x + storage_center_spacing,
            ]
            storage_centers_y = [
                1.275,
                1.275 - storage_center_spacing,
                1.275 - storage_center_spacing * 2.0,
                1.275 - storage_center_spacing * 3.0,
            ]
            place_size = 0.40
            place_gap = 0.40
            place_center_spacing = place_size + place_gap
            place_center_x = field_max_x - 1.00
            place_centers_y = [
                1.20,
                1.20 - place_center_spacing,
                1.20 - place_center_spacing * 2.0,
                1.20 - place_center_spacing * 3.0,
            ]
            place_specs = [
                ('FOOD_PLACE', '0 FOOD', place_centers_y[0], (0.22, 0.78, 0.18, 0.75)),
                ('TOOL_PLACE', '1 TOOL', place_centers_y[1], (0.55, 0.55, 0.55, 0.75)),
                ('INSTRUMENT_PLACE', '2 INST', place_centers_y[2], (0.1, 0.28, 0.78, 0.75)),
                ('MEDICINE_PLACE', '3 MED', place_centers_y[3], (0.85, 0.05, 0.02, 0.75)),
            ]
            for i, (_name, label, y, color) in enumerate(place_specs):
                self._append_cube_marker(
                    markers,
                    stamp,
                    'task_field_place_zones',
                    100 + i,
                    place_center_x,
                    y,
                    0.02,
                    place_size,
                    place_size,
                    0.04,
                    color,
                )
                self._append_text_marker(
                    markers,
                    stamp,
                    'task_field_text',
                    100 + i,
                    place_center_x,
                    y,
                    0.28,
                    label,
                    0.13,
                    (1.0, 1.0, 1.0, 1.0),
                )

            storage_index = 0
            for row_index, box_center_x in enumerate(storage_centers_x, start=1):
                for col_index, box_center_y in enumerate(storage_centers_y, start=1):
                    storage_index += 1
                    self._append_cube_marker(
                        markers,
                        stamp,
                        'task_field_storage_zones',
                        200 + storage_index,
                        box_center_x,
                        box_center_y,
                        0.025,
                        storage_size,
                        storage_size,
                        0.05,
                        (1.0, 1.0, 1.0, 0.70),
                    )
                    self._append_text_marker(
                        markers,
                        stamp,
                        'task_field_text',
                        200 + storage_index,
                        box_center_x,
                        box_center_y,
                        0.22,
                        f'R{row_index}C{col_index}',
                        0.095,
                        (0.0, 0.0, 0.0, 1.0),
                    )
        for i, wp in enumerate([wp for wp in self._waypoints if wp.name.endswith('_SCAN_PICK')]):
            self._append_cube_marker(
                markers,
                stamp,
                'task_field_pick_stops',
                300 + i,
                wp.x,
                wp.y,
                0.03,
                0.08,
                0.08,
                0.06,
                (0.0, 0.25, 1.0, 0.65),
            )

    def _publish_visualization(self) -> None:
        if not self._waypoints and not self._publish_field_layout:
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

        if self._publish_field_layout:
            self._append_field_layout_markers(markers, stamp)

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
            if wp.arm.enabled:
                label += f' [ARM:{wp.arm.state}]'
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
