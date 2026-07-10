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
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from r2_arm_control.srv import SetArmState
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from vmc_quadruped_controller.msg import MoveCmd
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
    start_delay_sec: float = 1.0
    continue_on_failure: bool = False


@dataclass
class DirectMotionTask:
    enabled: bool = False
    step_x: float = 0.0
    step_y: float = 0.0
    duration_sec: float = 0.0


@dataclass
class PlaceViaPoint:
    name: str
    x: float
    y: float
    yaw: Optional[float] = None


@dataclass
class PlaceTask:
    enabled: bool = False
    target: str = 'auto'
    transfer_x: float = 3.85
    column_mid_x: float = 2.275
    side_aisle_y: float = 1.55
    centerline_y: float = 0.0
    return_to_next_pick: bool = True
    via_points: List[PlaceViaPoint] = field(default_factory=list)


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
    align_yaw: bool = False
    standoff_m: float = 0.0
    approach_yaw: Optional[float] = None
    carry_mode: str = ''
    target_class: str = ''
    yolo: YoloTask = field(default_factory=YoloTask)
    arm: ArmTask = field(default_factory=ArmTask)
    direct_motion: DirectMotionTask = field(default_factory=DirectMotionTask)
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
        self.declare_parameter('wait_for_nav2_active', True)
        self.declare_parameter(
            'navigate_lifecycle_state_services',
            ['/bt_navigator/get_state'],
        )
        self.declare_parameter(
            'spin_lifecycle_state_services',
            ['/behavior_server/get_state'],
        )
        self.declare_parameter('auto_start', True)
        self.declare_parameter('loop_mission', False)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('retry_per_waypoint', 1)
        self.declare_parameter('pause_after_reach_sec', 0.2)
        self.declare_parameter('same_position_spin_enabled', False)
        self.declare_parameter('same_position_tolerance', 0.05)
        self.declare_parameter('same_yaw_tolerance_deg', 5.0)
        self.declare_parameter('skip_already_reached_nav_goals', True)
        self.declare_parameter('already_reached_xy_tolerance', 0.18)
        self.declare_parameter('already_reached_yaw_tolerance_deg', 12.0)
        self.declare_parameter('verify_reached_pose', True)
        self.declare_parameter('reached_xy_tolerance', 0.25)
        self.declare_parameter('spin_time_allowance_sec', 20.0)
        self.declare_parameter('spin_lookup_timeout_sec', 0.10)
        self.declare_parameter('markers_topic', '/preset_waypoints')
        self.declare_parameter('route_topic', '/preset_route')
        self.declare_parameter('current_goal_topic', '/preset_current_goal')
        self.declare_parameter('marker_point_scale', 0.22)
        self.declare_parameter('marker_text_scale', 0.13)
        self.declare_parameter('publish_field_layout', True)
        self.declare_parameter('publish_task_item_zones', True)
        self.declare_parameter('yolo_enable_topic', '/yolo/enable')
        self.declare_parameter('yolo_detection_topic', '/yolo/detections')
        self.declare_parameter('yolo_result_topic', '/preset_yolo_result')
        self.declare_parameter('yolo_result_text_scale', 0.12)
        self.declare_parameter('high_score_zone_topic', '/mission/high_score_zone')
        self.declare_parameter('arm_state_service_name', '/r2/arm/set_state')
        self.declare_parameter('arm_state_topic', '/r2/arm/state_machine_state')
        self.declare_parameter('arm_default_timeout_sec', 20.0)
        self.declare_parameter('arm_start_settle_sec', 1.0)
        self.declare_parameter('base_motion_lock_topic', '/base_motion/lock')
        self.declare_parameter('move_cmd_topic', '/move_cmd')

        self._waypoint_file = str(self.get_parameter('waypoint_file').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._action_name = str(self.get_parameter('action_name').value)
        self._spin_action_name = str(self.get_parameter('spin_action_name').value)
        self._wait_for_nav2_active = self._to_bool(
            self.get_parameter('wait_for_nav2_active').value
        )
        self._navigate_lifecycle_state_services = self._to_string_list(
            self.get_parameter('navigate_lifecycle_state_services').value
        )
        self._spin_lifecycle_state_services = self._to_string_list(
            self.get_parameter('spin_lifecycle_state_services').value
        )
        self._auto_start = self._to_bool(self.get_parameter('auto_start').value)
        self._loop_mission = self._to_bool(self.get_parameter('loop_mission').value)
        self._stop_on_failure = self._to_bool(
            self.get_parameter('stop_on_failure').value
        )
        self._retry_per_waypoint = int(self.get_parameter('retry_per_waypoint').value)
        self._pause_after_reach_sec = float(
            self.get_parameter('pause_after_reach_sec').value
        )
        self._same_position_spin_enabled = self._to_bool(
            self.get_parameter('same_position_spin_enabled').value
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
        self._verify_reached_pose = self._to_bool(
            self.get_parameter('verify_reached_pose').value
        )
        self._reached_xy_tolerance = max(
            0.0, float(self.get_parameter('reached_xy_tolerance').value)
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
        high_score_zone_topic = str(
            self.get_parameter('high_score_zone_topic').value
        )
        arm_state_service_name = str(
            self.get_parameter('arm_state_service_name').value
        )
        self._arm_state_topic = str(self.get_parameter('arm_state_topic').value)
        self._arm_default_timeout_sec = max(
            1.0, float(self.get_parameter('arm_default_timeout_sec').value)
        )
        self._arm_start_settle_sec = max(
            0.0, float(self.get_parameter('arm_start_settle_sec').value)
        )
        base_motion_lock_topic = str(
            self.get_parameter('base_motion_lock_topic').value
        )
        move_cmd_topic = str(self.get_parameter('move_cmd_topic').value)

        self._marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._route_pub = self.create_publisher(Path, route_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, current_goal_topic, 10)
        self._yolo_enable_pub = self.create_publisher(Bool, yolo_enable_topic, 10)
        self._yolo_result_pub = self.create_publisher(String, yolo_result_topic, 10)
        self._base_motion_lock_pub = self.create_publisher(
            Bool, base_motion_lock_topic, 10
        )
        self._move_cmd_pub = self.create_publisher(MoveCmd, move_cmd_topic, 10)
        self._yolo_sub = self.create_subscription(
            Detection2DArray,
            yolo_detection_topic,
            self._yolo_detection_callback,
            20,
        )
        self._high_score_zone_sub = self.create_subscription(
            Int32,
            high_score_zone_topic,
            self._high_score_zone_callback,
            10,
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
        lifecycle_services = set(self._navigate_lifecycle_state_services)
        lifecycle_services.update(self._spin_lifecycle_state_services)
        self._nav2_lifecycle_clients = {
            name: self.create_client(GetState, name)
            for name in lifecycle_services
            if name
        }
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
        self._last_wait_nav2_active_log_sec = 0.0
        self._nav2_lifecycle_futures = {}
        self._nav2_lifecycle_state_ids = {}
        self._nav2_lifecycle_last_query_sec = {}
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
        self._last_detected_class_global: str = ''
        self._high_score_zone: int = -1
        self._yolo_default_task = YoloTask()
        self._arm_task_active = False
        self._arm_task_index: Optional[int] = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_start_ready_sec = 0.0
        self._arm_deadline_sec = 0.0
        self._arm_future = None
        self._arm_current_state = ''
        self._arm_state_update_sec = 0.0
        self._last_wait_arm_service_log_sec = 0.0
        self._direct_motion_task_active = False
        self._direct_motion_task_index: Optional[int] = None
        self._direct_motion_deadline_sec = 0.0
        self._direct_motion_step_x = 0.0
        self._direct_motion_step_y = 0.0
        self._base_motion_locked = False

        self._set_yolo_enabled(False, force=True)
        self._set_base_motion_locked(False, force=True)

        self.create_timer(0.2, self._tick)
        self.create_timer(0.05, self._publish_arm_base_stop)
        self.create_timer(0.05, self._publish_direct_motion_cmd)
        self.create_timer(0.5, self._publish_visualization)

        self.get_logger().info(
            f'Preset waypoint mission ready. waypoints={len(self._waypoints)}, '
            f'action={self._action_name}, spin_action={self._spin_action_name}, '
            f'frame={self._frame_id}, auto_start={self._auto_start}, '
            f'high_score_zone_topic={high_score_zone_topic}'
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

    @staticmethod
    def _to_string_list(value) -> List[str]:
        if isinstance(value, (list, tuple)):
            return [str(item).strip() for item in value if str(item).strip()]
        if isinstance(value, str):
            return [item.strip() for item in value.split(',') if item.strip()]
        return []

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
        start_delay_sec = max(
            0.0, float(item.get('arm_start_settle_sec', self._arm_start_settle_sec))
        )
        continue_on_failure = False

        if isinstance(raw_cfg, dict):
            state = str(raw_cfg.get('state', '')).strip().lower()
            enabled = self._to_bool(raw_cfg.get('enabled', bool(state)))
            timeout_sec = max(
                1.0, float(raw_cfg.get('timeout_sec', timeout_sec))
            )
            start_delay_sec = max(
                0.0,
                float(
                    raw_cfg.get(
                        'start_delay_sec',
                        raw_cfg.get('settle_sec', raw_cfg.get('delay_sec', start_delay_sec)),
                    )
                ),
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
            start_delay_sec=start_delay_sec,
            continue_on_failure=continue_on_failure,
        )

    def _parse_direct_motion_task(self, item: dict) -> DirectMotionTask:
        raw_cfg = item.get('direct_motion', item.get('motion', False))
        enabled = False
        step_x = 0.0
        step_y = 0.0
        duration_sec = 0.0

        if isinstance(raw_cfg, dict):
            enabled = self._to_bool(raw_cfg.get('enabled', True))
            step_x = float(raw_cfg.get('step_x', item.get('step_x', step_x)))
            step_y = float(raw_cfg.get('step_y', item.get('step_y', step_y)))
            duration_sec = max(
                0.0,
                float(
                    raw_cfg.get(
                        'duration_sec',
                        raw_cfg.get('duration', item.get('duration_sec', duration_sec)),
                    )
                ),
            )
        else:
            enabled = self._to_bool(raw_cfg)
            step_x = float(item.get('step_x', step_x))
            step_y = float(item.get('step_y', step_y))
            duration_sec = max(0.0, float(item.get('duration_sec', duration_sec)))

        return DirectMotionTask(
            enabled=enabled and duration_sec > 0.0,
            step_x=max(-1.0, min(1.0, step_x)),
            step_y=max(-1.0, min(1.0, step_y)),
            duration_sec=duration_sec,
        )

    def _parse_place_via_points(self, raw_cfg: dict) -> List[PlaceViaPoint]:
        raw_via = raw_cfg.get('via', raw_cfg.get('via_points', []))
        if not isinstance(raw_via, list):
            return []

        via_points: List[PlaceViaPoint] = []
        for i, raw_point in enumerate(raw_via):
            if not isinstance(raw_point, dict):
                self.get_logger().warn(f'Skip invalid place via point at index {i}.')
                continue
            try:
                x = float(raw_point['x'])
                y = float(raw_point['y'])
                if 'yaw' in raw_point:
                    yaw = float(raw_point['yaw'])
                elif 'yaw_deg' in raw_point:
                    yaw = math.radians(float(raw_point['yaw_deg']))
                else:
                    yaw = None
            except Exception:
                self.get_logger().warn(f'Skip invalid place via point at index {i}.')
                continue
            name = str(raw_point.get('name', f'VIA_{i + 1:02d}')).strip()
            via_points.append(PlaceViaPoint(name or f'VIA_{i + 1:02d}', x, y, yaw))
        return via_points

    def _parse_place_task(self, item: dict) -> PlaceTask:
        raw_cfg = item.get('place', item.get('place_after_pick', False))
        enabled = False
        target = 'auto'
        transfer_x = 3.85
        column_mid_x = 2.275
        side_aisle_y = 1.55
        centerline_y = 0.0
        return_to_next_pick = True
        via_points: List[PlaceViaPoint] = []

        if isinstance(raw_cfg, dict):
            enabled = self._to_bool(raw_cfg.get('enabled', False))
            target = str(raw_cfg.get('target', target)).strip()
            transfer_x = float(raw_cfg.get('transfer_x', transfer_x))
            column_mid_x = float(raw_cfg.get('column_mid_x', column_mid_x))
            side_aisle_y = float(raw_cfg.get('side_aisle_y', side_aisle_y))
            centerline_y = float(raw_cfg.get('centerline_y', centerline_y))
            return_to_next_pick = self._to_bool(
                raw_cfg.get('return_to_next_pick', return_to_next_pick)
            )
            via_points = self._parse_place_via_points(raw_cfg)
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
            column_mid_x=column_mid_x,
            side_aisle_y=side_aisle_y,
            centerline_y=centerline_y,
            return_to_next_pick=return_to_next_pick,
            via_points=via_points,
        )

    def _apply_waypoint_role_defaults(self, item: dict) -> dict:
        role = str(
            item.get('role', item.get('type', item.get('kind', '')))
        ).strip().lower()
        if not role:
            return item

        cfg = dict(item)

        def set_if_missing(key: str, value) -> None:
            if key not in cfg:
                cfg[key] = value

        def has_any_key(*keys: str) -> bool:
            return any(key in cfg for key in keys)

        if role in ('observe', 'scan', 'yolo'):
            set_if_missing('align_yaw', True)
            set_if_missing('carry_mode', 'empty')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = True
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'none'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False
        elif role in ('pick', 'pickup', 'grab'):
            set_if_missing('align_yaw', True)
            set_if_missing('carry_mode', 'empty')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'pick'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False
        elif role in ('pick_auto_place', 'pick_place'):
            set_if_missing('align_yaw', True)
            set_if_missing('carry_mode', 'empty')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'pick'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = {
                    'enabled': True,
                    'target': str(cfg.get('target', 'auto')).strip() or 'auto',
                    'transfer_x': float(cfg.get('transfer_x', 3.85)),
                    'column_mid_x': float(cfg.get('column_mid_x', 2.275)),
                    'side_aisle_y': float(cfg.get('side_aisle_y', 1.55)),
                    'centerline_y': float(cfg.get('centerline_y', 0.0)),
                    'return_to_next_pick': self._to_bool(
                        cfg.get('return_to_next_pick', True)
                    ),
                }
        elif role in ('carry', 'via', 'carry_via', 'transfer', 'approach_place'):
            set_if_missing('align_yaw', False)
            set_if_missing('carry_mode', 'carry')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'none'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False
        elif role in ('place', 'place_auto', 'drop'):
            set_if_missing('align_yaw', True)
            set_if_missing('carry_mode', 'place')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'place'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = {
                    'enabled': True,
                    'target': str(cfg.get('target', 'auto')).strip() or 'auto',
                }
        elif role in ('return', 'empty', 'exit', 'transit'):
            set_if_missing('align_yaw', False)
            set_if_missing('carry_mode', 'empty')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'none'
            if not has_any_key('direct_motion', 'motion'):
                cfg['direct_motion'] = False
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False
        elif role in ('backoff', 'backup', 'reverse'):
            set_if_missing('align_yaw', False)
            set_if_missing('carry_mode', 'empty')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'none'
            if not has_any_key('direct_motion', 'motion'):
                cfg['direct_motion'] = {
                    'enabled': True,
                    'step_x': float(cfg.get('step_x', 0.0)),
                    'step_y': float(cfg.get('step_y', 0.35)),
                    'duration_sec': float(cfg.get('duration_sec', 1.0)),
                }
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False
        elif role == 'store':
            set_if_missing('align_yaw', False)
            set_if_missing('carry_mode', 'carry')
            if not has_any_key('yolo', 'yolo_enabled'):
                cfg['yolo'] = False
            if not has_any_key('arm', 'arm_state'):
                cfg['arm'] = 'store'
            if not has_any_key('place', 'place_after_pick'):
                cfg['place'] = False

        return cfg

    @staticmethod
    def _normalize_class_key(value: str) -> str:
        return str(value).strip().lower().replace(' ', '').replace('_', '').replace('-', '')

    def _is_arm_task_complete(self, requested_state: str) -> bool:
        # The arm state machine auto-chains pick -> store and place -> idle. Mission
        # flow should wait for the chained safe/carry state before moving the base.
        if requested_state == 'pick':
            return self._arm_current_state == 'store'
        if requested_state == 'place':
            return self._arm_current_state == 'idle'
        if self._arm_current_state == requested_state:
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
            item = self._apply_waypoint_role_defaults(item)
            if not self._to_bool(item.get('enabled', True)):
                continue
            try:
                x = float(item.get('x', 0.0))
                y = float(item.get('y', 0.0))
                if 'yaw' in item:
                    yaw = float(item['yaw'])
                else:
                    yaw = math.radians(float(item.get('yaw_deg', 0.0)))
                if 'approach_yaw' in item:
                    approach_yaw = float(item['approach_yaw'])
                elif 'approach_yaw_deg' in item:
                    approach_yaw = math.radians(float(item['approach_yaw_deg']))
                else:
                    approach_yaw = None
                standoff_m = max(0.0, float(item.get('standoff_m', 0.0)))
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
                    align_yaw=self._to_bool(item.get('align_yaw', False)),
                    standoff_m=standoff_m,
                    approach_yaw=approach_yaw,
                    carry_mode=str(item.get('carry_mode', '')).strip().lower(),
                    target_class=str(item.get('target_class', '')).strip(),
                    yolo=self._parse_yolo_task(item),
                    arm=self._parse_arm_task(item),
                    direct_motion=self._parse_direct_motion_task(item),
                    place=self._parse_place_task(item),
                )
            )

        return waypoints

    @staticmethod
    def _effective_yaw(wp: Waypoint) -> float:
        return wp.approach_yaw if wp.approach_yaw is not None else wp.yaw

    def _build_pose(self, wp: Waypoint) -> PoseStamped:
        x, y = wp.x, wp.y
        yaw = self._effective_yaw(wp)

        # 如果航点是 place 且 target 为 auto/detected，动态解析放置坐标
        # 支持 "observe → pick → nav → place" 分离流程：place 坐标根据最近一次 YOLO 结果确定
        if wp.arm.state == 'place' and wp.place.enabled:
            target_key = wp.place.target.strip().lower()
            if target_key in ('auto', 'detected'):
                target = self._resolve_place_target(wp)
                if target is not None:
                    x, y, yaw = target.x, target.y, target.yaw
                    self.get_logger().info(
                        f'Dynamic place for {wp.name}: target={target.name}, '
                        f'pos=({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}°'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to resolve dynamic place target for {wp.name}, '
                        f'using configured coordinates ({wp.x:.3f}, {wp.y:.3f}).'
                    )

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat_z_w(yaw)
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

    @staticmethod
    def _place_side_from_y(y: float) -> str:
        return 'left' if y >= 0.0 else 'right'

    @staticmethod
    def _back_pick_params(side: str) -> tuple[float, float, float, float]:
        if side == 'left':
            return 4.70, 3.70, 1.125, math.pi * 0.5
        return 4.70, 3.70, -1.275, -math.pi * 0.5

    @staticmethod
    def _is_back_pick_waypoint(wp: Waypoint) -> bool:
        if 'BACK' in wp.name.upper():
            return True
        return abs(normalize_angle(wp.yaw - math.pi)) < math.radians(35.0)

    @staticmethod
    def _is_extreme_place_for_side(target: PlaceTarget, side: str) -> bool:
        if side == 'left':
            return target.y > 0.75
        return target.y < -0.75

    @staticmethod
    def _is_leftmost_place_target(target: PlaceTarget) -> bool:
        return target.y > 0.75

    def _retarget_next_back_pick_waypoints(self, start_index: int, side: str) -> None:
        observe_x, pick_x, pick_y, _side_yaw = self._back_pick_params(side)
        suffix = side.upper()
        for index in range(start_index, min(start_index + 2, len(self._waypoints))):
            wp = self._waypoints[index]
            if wp.yolo.enabled:
                wp.name = f'OBSERVE_{suffix}_BACK'
                wp.x = observe_x
                wp.y = pick_y
                wp.yaw = math.pi
                wp.approach_yaw = None
                wp.align_yaw = True
                self.get_logger().info(
                    f'Retarget next observe waypoint to {wp.name}: '
                    f'x={wp.x:.3f}, y={wp.y:.3f}, yaw=180.0deg'
                )
            elif wp.arm.enabled and wp.arm.state == 'pick':
                wp.name = f'PICK_{suffix}_BACK'
                wp.x = pick_x
                wp.y = pick_y
                wp.yaw = math.pi
                wp.approach_yaw = None
                wp.align_yaw = True
                self.get_logger().info(
                    f'Retarget next pick waypoint to {wp.name}: '
                    f'x={wp.x:.3f}, y={wp.y:.3f}, yaw=180.0deg'
                )

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

        # 1) 显式 target_class 优先，适合赛前已知箱子类别或复盘调试。
        detected_class = wp.target_class
        # 2) 再尝试当前航点自己的 YOLO 检测结果。
        if not detected_class:
            detected_class = self._yolo_best_class_by_waypoint.get(wp.name, '')
        # 3) 如果当前航点没有做过 YOLO，回退到最近一次检测（支持 observe → pick → ... → place 分离流程）。
        if not detected_class:
            detected_class = self._last_detected_class_global

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
        carry_mode: str = '',
        target_class: str = '',
        direct_motion: Optional[DirectMotionTask] = None,
    ) -> Waypoint:
        return Waypoint(
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            align_yaw=arm_state in ('pick', 'place'),
            carry_mode=carry_mode,
            target_class=target_class,
            yolo=YoloTask(enabled=False),
            arm=ArmTask(
                enabled=bool(arm_state),
                state=arm_state,
                start_delay_sec=self._arm_start_settle_sec,
            ),
            direct_motion=direct_motion or DirectMotionTask(),
            place=PlaceTask(enabled=False),
        )

    @staticmethod
    def _safe_waypoint_suffix(value: str, fallback: str) -> str:
        suffix = ''.join(
            char.upper() if char.isalnum() else '_'
            for char in str(value).strip()
        ).strip('_')
        return suffix or fallback

    def _append_nav_waypoint_if_distinct(
        self,
        route: List[Waypoint],
        name: str,
        x: float,
        y: float,
        yaw: float,
        carry_mode: str,
        target_class: str = '',
        min_distance: float = 0.08,
    ) -> None:
        if route:
            prev_x = route[-1].x
            prev_y = route[-1].y
        elif self._index < len(self._waypoints):
            prev_x = self._waypoints[self._index].x
            prev_y = self._waypoints[self._index].y
        else:
            prev_x = x
            prev_y = y

        if math.hypot(x - prev_x, y - prev_y) < min_distance:
            return

        route.append(
            self._make_waypoint(
                name,
                x,
                y,
                yaw,
                carry_mode=carry_mode,
                target_class=target_class,
            )
        )

    def _append_spin_waypoint_if_needed(
        self,
        route: List[Waypoint],
        name: str,
        x: float,
        y: float,
        yaw: float,
        carry_mode: str,
        target_class: str = '',
    ) -> None:
        if route:
            prev_x = route[-1].x
            prev_y = route[-1].y
            prev_yaw = self._effective_yaw(route[-1])
        elif self._index < len(self._waypoints):
            prev = self._waypoints[self._index]
            prev_x = prev.x
            prev_y = prev.y
            prev_yaw = self._effective_yaw(prev)
        else:
            prev_x = x
            prev_y = y
            prev_yaw = yaw

        if math.hypot(x - prev_x, y - prev_y) > self._same_position_tolerance:
            return
        if abs(normalize_angle(yaw - prev_yaw)) <= self._same_yaw_tolerance:
            return

        route.append(
            self._make_waypoint(
                name,
                x,
                y,
                yaw,
                carry_mode=carry_mode,
                target_class=target_class,
            )
        )

    def _build_back_pick_place_route(
        self,
        wp: Waypoint,
        target: PlaceTarget,
        transfer_x: float,
    ) -> List[Waypoint]:
        route: List[Waypoint] = []
        carry_yaw = math.pi
        pick_side = self._place_side_from_y(wp.y)
        same_extreme_side = self._is_extreme_place_for_side(target, pick_side)

        route.append(
            self._make_waypoint(
                f'{wp.name}_BACK_OVER_BUMP',
                max(wp.x + 0.20, transfer_x - 0.20),
                wp.y,
                carry_yaw,
                carry_mode='carry',
                target_class=target.name,
                direct_motion=DirectMotionTask(
                    enabled=True,
                    step_x=0.0,
                    step_y=0.35,
                    duration_sec=2.0,
                ),
            )
        )
        self._append_nav_waypoint_if_distinct(
            route,
            f'{wp.name}_AFTER_BUMP',
            transfer_x,
            wp.y,
            carry_yaw,
            'carry',
            target.name,
        )

        if same_extreme_side:
            self._append_nav_waypoint_if_distinct(
                route,
                f'{wp.name}_PLACE_BEHIND_{target.name}',
                transfer_x,
                target.y,
                carry_yaw,
                'carry',
                target.name,
            )
            self._append_spin_waypoint_if_needed(
                route,
                f'{wp.name}_TURN_180_PLACE_{target.name}',
                transfer_x,
                target.y,
                target.yaw,
                'carry',
                target.name,
            )
        else:
            place_side_yaw = math.pi * 0.5 if target.y > wp.y else -math.pi * 0.5
            self._append_spin_waypoint_if_needed(
                route,
                f'{wp.name}_TURN_90_PLACE_ROW_{target.name}',
                transfer_x,
                wp.y,
                place_side_yaw,
                'carry',
                target.name,
            )
            self._append_nav_waypoint_if_distinct(
                route,
                f'{wp.name}_PLACE_BEHIND_{target.name}',
                transfer_x,
                target.y,
                place_side_yaw,
                'carry',
                target.name,
            )
            self._append_spin_waypoint_if_needed(
                route,
                f'{wp.name}_TURN_90_FACE_PLACE_{target.name}',
                transfer_x,
                target.y,
                target.yaw,
                'carry',
                target.name,
            )

        route.append(
            self._make_waypoint(
                f'{wp.name}_PLACE_{target.name}',
                target.x,
                target.y,
                target.yaw,
                'place',
                carry_mode='place',
                target_class=target.name,
            )
        )
        return route

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
        column_mid_x = wp.place.column_mid_x
        side_aisle_y = wp.place.side_aisle_y
        centerline_y = wp.place.centerline_y
        route: List[Waypoint] = []

        if self._is_back_pick_waypoint(wp):
            route = self._build_back_pick_place_route(wp, target, transfer_x)
            insert_at = self._index + 1
            self._waypoints[insert_at:insert_at] = route
            self._dynamic_place_inserted_for.add(wp.name)
            route_names = ' -> '.join(item.name for item in route)
            self.get_logger().info(
                f'Inserted back-pick place route after {wp.name}: target={target.name}, '
                f'inserted_waypoints={len(route)}, route={route_names}'
            )
            return True

        if wp.place.via_points:
            for i, via in enumerate(wp.place.via_points):
                if i + 1 < len(wp.place.via_points):
                    next_x = wp.place.via_points[i + 1].x
                    next_y = wp.place.via_points[i + 1].y
                else:
                    next_x = target.x
                    next_y = target.y
                route.append(
                    self._make_waypoint(
                        f'{wp.name}_{self._safe_waypoint_suffix(via.name, f"VIA_{i + 1:02d}")}',
                        via.x,
                        via.y,
                        via.yaw
                        if via.yaw is not None
                        else self._heading_from_delta(next_x - via.x, next_y - via.y, wp.yaw),
                        carry_mode='carry',
                        target_class=target.name,
                    )
                )
        else:
            self._append_nav_waypoint_if_distinct(
                route,
                f'{wp.name}_COLUMN_MID',
                column_mid_x,
                centerline_y,
                0.0,
                'carry',
                target.name,
            )
            self._append_nav_waypoint_if_distinct(
                route,
                f'{wp.name}_TRANSFER',
                transfer_x,
                centerline_y,
                0.0,
                'carry',
                target.name,
            )
            if target.x >= transfer_x:
                place_approach_x = min(target.x, transfer_x + 0.25)
            else:
                place_approach_x = max(target.x, transfer_x - 0.25)
            place_approach_yaw = self._heading_from_delta(
                place_approach_x - transfer_x,
                target.y - centerline_y,
                0.0,
            )
            self._append_nav_waypoint_if_distinct(
                route,
                f'{wp.name}_APPROACH_PLACE_{target.name}',
                place_approach_x,
                target.y,
                place_approach_yaw,
                'carry',
                target.name,
            )
        route.append(
            self._make_waypoint(
                f'{wp.name}_PLACE_{target.name}',
                target.x,
                target.y,
                target.yaw,
                'place',
                carry_mode='place',
                target_class=target.name,
            )
        )

        if wp.place.return_to_next_pick and next_wp is not None:
            backoff_distance_m = 0.20
            post_place_y_offset_m = -0.15
            post_backoff_turn_step_x = 0.18
            post_backoff_turn_angle_rad = math.radians(80.0)
            post_backoff_turn_duration_sec = 3.6
            backoff_x = target.x - math.cos(target.yaw) * backoff_distance_m
            backoff_y = (
                target.y
                - math.sin(target.yaw) * backoff_distance_m
                + post_place_y_offset_m
            )
            pickup_side = 'left'
            observe_x, _pick_x, pickup_y, pickup_side_yaw = self._back_pick_params(pickup_side)
            self._retarget_next_back_pick_waypoints(self._index + 1, pickup_side)
            route.append(
                self._make_waypoint(
                    f'{wp.name}_BACKOFF_20CM',
                    backoff_x,
                    backoff_y,
                    target.yaw,
                    carry_mode='empty',
                    target_class=target.name,
                    direct_motion=DirectMotionTask(
                        enabled=True,
                        step_x=0.0,
                        step_y=0.35,
                        duration_sec=0.7,
                    ),
                )
            )
            route.append(
                self._make_waypoint(
                    f'{wp.name}_TURN_LEFT_80_FIXED',
                    backoff_x,
                    backoff_y,
                    normalize_angle(target.yaw + post_backoff_turn_angle_rad),
                    carry_mode='empty',
                    target_class=target.name,
                    direct_motion=DirectMotionTask(
                        enabled=True,
                        step_x=post_backoff_turn_step_x,
                        step_y=0.0,
                        duration_sec=post_backoff_turn_duration_sec,
                    ),
                )
            )
            if not self._is_leftmost_place_target(target):
                self._append_nav_waypoint_if_distinct(
                    route,
                    f'{wp.name}_GO_LEFT_LANE',
                    backoff_x,
                    pickup_y,
                    pickup_side_yaw,
                    'empty',
                    target.name,
                )

        insert_at = self._index + 1
        self._waypoints[insert_at:insert_at] = route
        self._dynamic_place_inserted_for.add(wp.name)
        route_names = ' -> '.join(item.name for item in route)
        self.get_logger().info(
            f'Inserted dynamic place route after {wp.name}: target={target.name}, '
            f'inserted_waypoints={len(route)}, route={route_names}'
        )
        return True

    def _should_spin_to_current_waypoint(self) -> bool:
        if not self._same_position_spin_enabled:
            return False

        previous = self._get_previous_waypoint()
        if previous is None or self._index >= len(self._waypoints):
            return False

        current = self._waypoints[self._index]
        position_delta = math.hypot(current.x - previous.x, current.y - previous.y)
        yaw_delta = abs(
            normalize_angle(self._effective_yaw(current) - self._effective_yaw(previous))
        )
        return (
            current.align_yaw
            and position_delta <= self._same_position_tolerance
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
        yaw_delta = abs(normalize_angle(self._effective_yaw(wp) - robot_yaw))
        if (
            self._same_position_spin_enabled
            and wp.align_yaw
            and xy_delta <= self._same_position_tolerance
            and yaw_delta > self._same_yaw_tolerance
        ):
            return False

        yaw_reached = (not wp.align_yaw) or yaw_delta <= self._already_reached_yaw_tolerance
        if (
            xy_delta <= self._already_reached_xy_tolerance
            and yaw_reached
        ):
            self.get_logger().info(
                f'Skipping already reached waypoint [{self._index + 1}/{len(self._waypoints)}] '
                f'{wp.name}: distance={xy_delta:.3f}m, yaw_delta={yaw_delta:.3f}rad'
            )
            self._complete_waypoint()
            return True

        return False

    def _verify_nav_result_pose(self, wp: Waypoint) -> bool:
        if not self._verify_reached_pose:
            return True

        pose = self._get_robot_pose_for_skip(log_on_failure=True)
        if pose is None:
            self.get_logger().warn(
                f'Nav2 reported success for {wp.name}, but current robot pose could not be checked.'
            )
            return False

        robot_x, robot_y, _ = pose
        goal_pose = self._build_pose(wp)
        goal_x = float(goal_pose.pose.position.x)
        goal_y = float(goal_pose.pose.position.y)
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if distance <= self._reached_xy_tolerance:
            self.get_logger().info(
                f'Verified waypoint pose {wp.name}: robot=({robot_x:.3f}, {robot_y:.3f}), '
                f'goal=({goal_x:.3f}, {goal_y:.3f}), distance={distance:.3f}m'
            )
            return True

        self.get_logger().warn(
            f'Nav2 reported success for {wp.name}, but robot is still {distance:.3f}m '
            f'from the waypoint: robot=({robot_x:.3f}, {robot_y:.3f}), '
            f'goal=({goal_x:.3f}, {goal_y:.3f}), tolerance={self._reached_xy_tolerance:.3f}m'
        )
        return False

    def _verify_spin_result_yaw(self, wp: Waypoint) -> bool:
        pose = self._get_robot_pose_for_skip(log_on_failure=True)
        if pose is None:
            self.get_logger().warn(
                f'Spin reported success for {wp.name}, but current robot pose could not be checked.'
            )
            return False

        robot_x, robot_y, robot_yaw = pose
        yaw_delta = abs(normalize_angle(self._effective_yaw(wp) - robot_yaw))
        distance = math.hypot(wp.x - robot_x, wp.y - robot_y)
        if yaw_delta > self._same_yaw_tolerance:
            self.get_logger().warn(
                f'Spin reported success for {wp.name}, but yaw error is still '
                f'{yaw_delta:.3f}rad; tolerance={self._same_yaw_tolerance:.3f}rad'
            )
            return False

        if distance > self._reached_xy_tolerance:
            self.get_logger().warn(
                f'Spin drifted during {wp.name}: robot=({robot_x:.3f}, {robot_y:.3f}), '
                f'goal=({wp.x:.3f}, {wp.y:.3f}), distance={distance:.3f}m. '
                'Continuing because spin waypoints verify yaw only.'
            )
        else:
            self.get_logger().info(
                f'Verified spin yaw {wp.name}: yaw_delta={yaw_delta:.3f}rad, '
                f'drift={distance:.3f}m'
            )
        return True

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

    def _required_lifecycle_services_for_current_goal(self) -> List[str]:
        if not self._wait_for_nav2_active:
            return []
        if self._should_spin_to_current_waypoint():
            return self._spin_lifecycle_state_services
        return self._navigate_lifecycle_state_services

    def _lifecycle_service_active(self, service_name: str, now_sec: float) -> bool:
        client = self._nav2_lifecycle_clients.get(service_name)
        if client is None:
            return False

        if not client.wait_for_service(timeout_sec=0.0):
            return False

        future = self._nav2_lifecycle_futures.get(service_name)
        if future is not None:
            if not future.done():
                return (
                    self._nav2_lifecycle_state_ids.get(service_name)
                    == State.PRIMARY_STATE_ACTIVE
                )

            try:
                response = future.result()
                state_id = response.current_state.id if response is not None else 0
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to query Nav2 lifecycle state from {service_name}: {exc}'
                )
                state_id = 0

            self._nav2_lifecycle_state_ids[service_name] = state_id
            self._nav2_lifecycle_futures.pop(service_name, None)
            if state_id == State.PRIMARY_STATE_ACTIVE:
                return True

        if (
            self._nav2_lifecycle_state_ids.get(service_name)
            == State.PRIMARY_STATE_ACTIVE
        ):
            return True

        last_query_sec = self._nav2_lifecycle_last_query_sec.get(service_name, 0.0)
        if (now_sec - last_query_sec) >= 0.5:
            self._nav2_lifecycle_futures[service_name] = client.call_async(
                GetState.Request()
            )
            self._nav2_lifecycle_last_query_sec[service_name] = now_sec
        return False

    def _wait_for_nav2_lifecycle_active(self, now_sec: float) -> bool:
        required_services = self._required_lifecycle_services_for_current_goal()
        if not required_services:
            return True

        pending_services = [
            service_name
            for service_name in required_services
            if not self._lifecycle_service_active(service_name, now_sec)
        ]
        if not pending_services:
            return True

        if (now_sec - self._last_wait_nav2_active_log_sec) > 2.0:
            self._last_wait_nav2_active_log_sec = now_sec
            self.get_logger().info(
                'Waiting for Nav2 lifecycle ACTIVE before sending waypoint: '
                + ', '.join(pending_services)
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

    def _publish_base_stop(self) -> None:
        msg = MoveCmd()
        msg.step_x = 0.0
        msg.step_y = 0.0
        self._move_cmd_pub.publish(msg)

    def _set_base_motion_locked(self, locked: bool, force: bool = False) -> None:
        if not force and self._base_motion_locked == locked:
            return

        self._base_motion_locked = locked
        msg = Bool()
        msg.data = locked
        self._base_motion_lock_pub.publish(msg)
        self._publish_base_stop()
        self.get_logger().info(
            f'Base motion {"locked" if locked else "unlocked"} for arm task.'
        )

    def _publish_arm_base_stop(self) -> None:
        if not self._arm_task_active:
            return
        msg = Bool()
        msg.data = True
        self._base_motion_lock_pub.publish(msg)
        self._publish_base_stop()

    def _publish_direct_motion_cmd(self) -> None:
        if not self._direct_motion_task_active:
            return
        msg = MoveCmd()
        msg.step_x = float(self._direct_motion_step_x)
        msg.step_y = float(self._direct_motion_step_y)
        self._move_cmd_pub.publish(msg)

    def _start_direct_motion_task(self, wp: Waypoint) -> None:
        now_sec = self._now_sec()
        self._direct_motion_task_active = True
        self._direct_motion_task_index = self._index
        self._direct_motion_deadline_sec = now_sec + wp.direct_motion.duration_sec
        self._direct_motion_step_x = wp.direct_motion.step_x
        self._direct_motion_step_y = wp.direct_motion.step_y
        self._active_goal_kind = 'direct_motion'
        self._publish_direct_motion_cmd()
        self.get_logger().info(
            f'Starting direct motion [{self._index + 1}/{len(self._waypoints)}] '
            f'{wp.name}: step_x={self._direct_motion_step_x:.3f}, '
            f'step_y={self._direct_motion_step_y:.3f}, '
            f'duration={wp.direct_motion.duration_sec:.2f}s'
        )

    def _tick_direct_motion_task(self) -> None:
        if (
            self._direct_motion_task_index is None
            or self._direct_motion_task_index >= len(self._waypoints)
        ):
            self._finish_direct_motion_task()
            return

        self._publish_direct_motion_cmd()
        if self._now_sec() < self._direct_motion_deadline_sec:
            return

        wp = self._waypoints[self._direct_motion_task_index]
        self.get_logger().info(
            f'Direct motion finished at {wp.name}: '
            f'step_x={self._direct_motion_step_x:.3f}, '
            f'step_y={self._direct_motion_step_y:.3f}'
        )
        self._finish_direct_motion_task()

    def _finish_direct_motion_task(self) -> None:
        self._direct_motion_task_active = False
        self._direct_motion_task_index = None
        self._direct_motion_deadline_sec = 0.0
        self._direct_motion_step_x = 0.0
        self._direct_motion_step_y = 0.0
        self._publish_base_stop()
        self._complete_waypoint()

    def _high_score_zone_callback(self, msg: Int32) -> None:
        zone = int(msg.data)
        if zone < 0 or zone > 3:
            if self._high_score_zone != -1:
                self.get_logger().info('High-score zone cleared.')
            self._high_score_zone = -1
            return
        if zone != self._high_score_zone:
            self.get_logger().info(f'High-score zone updated: class_{zone}')
        self._high_score_zone = zone

    def _class_matches_high_score(self, class_id: str) -> bool:
        if self._high_score_zone < 0:
            return False
        normalized = self._normalize_class_key(class_id)
        return normalized in (
            f'class{self._high_score_zone}',
            str(self._high_score_zone),
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
        if self._direct_motion_task_active:
            self._tick_direct_motion_task()
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

        if self._index < len(self._waypoints):
            wp = self._waypoints[self._index]
            if wp.direct_motion.enabled:
                self._start_direct_motion_task(wp)
                return

        if self._maybe_skip_already_reached_waypoint():
            return

        if not self._wait_for_current_action_server(now_sec):
            return

        if not self._wait_for_nav2_lifecycle_active(now_sec):
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

        if not self._arm_request_sent and now_sec < self._arm_start_ready_sec:
            self._publish_arm_base_stop()
            return

        if not self._arm_request_sent:
            if self._is_arm_task_complete(wp.arm.state) and self._arm_state_update_sec > 0.0:
                self.get_logger().info(
                    f'Arm task already satisfied at {wp.name}: '
                    f'requested={wp.arm.state}, current={self._arm_current_state}'
                )
                self._finish_arm_task()
                return

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
            if wp.arm.state == 'pick' and wp.place.enabled:
                if not self._insert_dynamic_place_route(wp):
                    self._handle_dynamic_place_route_failure(wp)
                    return
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
            f'{wp.name}: x={wp.x:.3f}, y={wp.y:.3f}, yaw={self._effective_yaw(wp):.3f}, '
            f'carry_mode={wp.carry_mode or "none"}, target_class={wp.target_class or "auto"}'
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
        target_yaw = self._effective_yaw(wp)
        spin_goal.target_yaw = float(normalize_angle(target_yaw - robot_yaw))
        spin_goal.time_allowance = Duration(
            seconds=self._spin_time_allowance_sec
        ).to_msg()
        self.get_logger().info(
            f'Sending spin waypoint [{self._index + 1}/{len(self._waypoints)}] '
            f'{wp.name}: x={wp.x:.3f}, y={wp.y:.3f}, target_yaw={target_yaw:.3f}, '
            f'spin_delta={spin_goal.target_yaw:.3f}'
        )
        future = self._spin_action_client.send_goal_async(spin_goal)
        future.add_done_callback(self._on_spin_goal_response)
        self._active_goal_kind = 'spin'
        self._goal_in_flight = True

    def _send_final_yaw_spin_if_needed(self, wp: Waypoint) -> bool:
        if not wp.align_yaw:
            return False

        robot_yaw = self._get_robot_yaw()
        if robot_yaw is None:
            return False

        yaw_delta = abs(normalize_angle(self._effective_yaw(wp) - robot_yaw))
        if yaw_delta <= self._same_yaw_tolerance:
            return False

        self.get_logger().info(
            f'Final yaw alignment needed at {wp.name}: yaw_delta={yaw_delta:.3f}rad'
        )
        self._send_spin_goal(wp)
        return True

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
        best_class = str(payload.get('best_class', ''))
        self._yolo_best_class_by_waypoint[wp.name] = best_class
        if best_class:
            self._last_detected_class_global = best_class

        if payload['has_detection']:
            high_score_note = (
                ' (high-score class)'
                if self._class_matches_high_score(best_class)
                else ''
            )
            self.get_logger().info(
                f'YOLO task finished at {wp.name}: {payload["summary"]}{high_score_note}'
            )
        else:
            self.get_logger().info(
                f'YOLO task finished at {wp.name}: no valid detection in the time window.'
            )

        self._yolo_task_active = False
        self._yolo_task_index = None
        self._yolo_observations.clear()
        self._set_yolo_enabled(False)
        if not payload['has_detection']:
            self._mission_done = True
            self.get_logger().error(
                f'Mission stopped at {wp.name}: no box class detected, avoiding blind pick/place.'
            )
            return
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
        self._arm_start_ready_sec = now_sec + wp.arm.start_delay_sec
        self._arm_deadline_sec = self._arm_start_ready_sec + wp.arm.timeout_sec
        self._active_goal_kind = 'arm'
        self._set_base_motion_locked(True)
        self.get_logger().info(
            f'Starting arm task at waypoint {wp.name}: '
            f'state={wp.arm.state}, settle={wp.arm.start_delay_sec:.1f}s, '
            f'timeout={wp.arm.timeout_sec:.1f}s'
        )

    def _finish_arm_task(self) -> None:
        self._arm_task_active = False
        self._arm_task_index = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_start_ready_sec = 0.0
        self._arm_future = None
        self._set_base_motion_locked(False)
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
            if self._active_goal_kind == 'navigate' and not self._verify_nav_result_pose(wp):
                self._goal_in_flight = False
                self._handle_goal_failure('pose_verify_failed')
                return

            if self._active_goal_kind == 'spin' and not self._verify_spin_result_yaw(wp):
                self._goal_in_flight = False
                self._handle_goal_failure('spin_yaw_verify_failed')
                return

            self.get_logger().info(
                f'Waypoint reached [{self._index + 1}/{len(self._waypoints)}] '
                f'({self._active_goal_kind}): {wp.name}'
            )
            self._goal_in_flight = False

            if self._active_goal_kind == 'navigate' and self._send_final_yaw_spin_if_needed(wp):
                return

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
        self._arm_start_ready_sec = 0.0
        self._arm_future = None
        self._set_base_motion_locked(False)

        if wp.arm.continue_on_failure:
            self.get_logger().warn(
                f'Continuing mission after arm failure at waypoint {wp.name}.'
            )
            self._complete_waypoint()
            return

        self._mission_done = True
        self._set_yolo_enabled(False)
        self.get_logger().error('Mission stopped due to arm task failure.')

    def _handle_dynamic_place_route_failure(self, wp: Waypoint) -> None:
        self.get_logger().error(
            f'Failed to insert dynamic place route after {wp.name}; '
            'mission stopped to avoid carrying a box without a valid place target.'
        )
        self._arm_task_active = False
        self._arm_task_index = None
        self._arm_request_sent = False
        self._arm_request_sent_sec = 0.0
        self._arm_start_ready_sec = 0.0
        self._arm_future = None
        self._set_base_motion_locked(False)
        self._set_yolo_enabled(False)
        self._mission_done = True

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
        self._arm_start_ready_sec = 0.0
        self._arm_future = None
        self._direct_motion_task_active = False
        self._direct_motion_task_index = None
        self._direct_motion_deadline_sec = 0.0
        self._direct_motion_step_x = 0.0
        self._direct_motion_step_y = 0.0
        self._publish_base_stop()
        self._set_base_motion_locked(False)

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

    @staticmethod
    def _waypoint_text_style(wp: Waypoint, index: int) -> tuple[float, float, tuple[float, float, float, float], str]:
        if wp.yolo.enabled:
            return -0.18, 0.22, (0.0, 0.95, 1.0, 1.0), 'YOLO'
        if wp.direct_motion.enabled:
            return 0.0, 0.24, (1.0, 0.95, 0.1, 1.0), 'BACK'
        if wp.arm.enabled:
            if wp.arm.state == 'pick':
                return 0.18, -0.22, (1.0, 0.45, 0.05, 1.0), 'PICK'
            if wp.arm.state == 'place':
                return 0.20, -0.25, (0.1, 1.0, 0.35, 1.0), 'PLACE'
            if wp.arm.state == 'store':
                return 0.0, 0.24, (0.75, 0.65, 1.0, 1.0), 'STORE'
            return 0.18, -0.22, (1.0, 0.75, 0.15, 1.0), wp.arm.state.upper()
        if wp.carry_mode == 'carry':
            return 0.0, 0.24 + 0.08 * (index % 2), (0.35, 0.75, 1.0, 1.0), 'CARRY'
        if wp.carry_mode == 'place':
            return 0.20, -0.25, (0.1, 1.0, 0.35, 1.0), 'PLACE'
        return 0.0, 0.20 + 0.08 * (index % 2), (0.95, 0.95, 0.95, 1.0), 'NAV'

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
            text_dx, text_dy, text_color, text_badge = self._waypoint_text_style(wp, i)
            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self._frame_id
            text.ns = 'preset_waypoint_text'
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp.x + text_dx
            text.pose.position.y = wp.y + text_dy
            text.pose.position.z = 0.42
            text.pose.orientation.w = 1.0
            text.scale.z = self._marker_text_scale
            text.color.r = text_color[0]
            text.color.g = text_color[1]
            text.color.b = text_color[2]
            text.color.a = text_color[3]
            label = f'{i + 1:02d} {wp.name}\n{text_badge}'
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
            result.pose.position.x = wp.x + 0.26
            result.pose.position.y = wp.y + 0.34
            result.pose.position.z = 0.62
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
