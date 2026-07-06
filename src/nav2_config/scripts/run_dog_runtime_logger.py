#!/usr/bin/env python3

import csv
import json
import math
import os
import time
from datetime import datetime
from typing import Any, Dict, Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vmc_quadruped_controller.msg import MoveCmd


CSV_COLUMNS = [
    'wall_time',
    'ros_time_sec',
    'elapsed_sec',
    'map_x',
    'map_y',
    'map_yaw_rad',
    'odom_tf_x',
    'odom_tf_y',
    'odom_tf_yaw_rad',
    'odom_msg_x',
    'odom_msg_y',
    'odom_msg_yaw_rad',
    'odom_msg_vx',
    'odom_msg_vy',
    'odom_msg_wz',
    'cmd_nav_age_sec',
    'cmd_nav_vx',
    'cmd_nav_vy',
    'cmd_nav_wz',
    'cmd_vel_age_sec',
    'cmd_vel_vx',
    'cmd_vel_vy',
    'cmd_vel_wz',
    'move_cmd_age_sec',
    'move_step_x',
    'move_step_y',
    'goal_age_sec',
    'goal_frame',
    'goal_x',
    'goal_y',
    'goal_yaw_rad',
    'scan_age_sec',
    'scan_min_m',
    'scan_front_min_m',
    'bridge_status_age_sec',
    'bridge_reason',
    'bridge_final_align_active',
    'bridge_distance_m',
    'bridge_yaw_error_deg',
    'bridge_linear_x_cmd',
    'bridge_angular_z_cmd',
]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def finite_min(values: Iterable[float]) -> Optional[float]:
    best: Optional[float] = None
    for value in values:
        if not math.isfinite(value):
            continue
        if best is None or value < best:
            best = value
    return best


def fmt(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        if not math.isfinite(value):
            return ''
        return f'{value:.6f}'
    return str(value)


class RunDogRuntimeLogger(Node):
    def __init__(self) -> None:
        super().__init__('run_dog_runtime_logger')

        self.declare_parameter('log_root', '/home/orangepi/run_dog_logs')
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('tf_timeout_sec', 0.02)
        self.declare_parameter('front_scan_window_deg', 30.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('move_cmd_topic', '/move_cmd')
        self.declare_parameter('goal_topic', '/preset_current_goal')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('bridge_status_topic', '/cmd_vel_to_move_cmd/status')

        self._log_root = os.path.expanduser(str(self.get_parameter('log_root').value))
        self._sample_rate_hz = max(0.5, self._param_float('sample_rate_hz', 10.0))
        self._tf_timeout_sec = max(0.0, self._param_float('tf_timeout_sec', 0.02))
        self._front_scan_window_rad = math.radians(
            max(1.0, self._param_float('front_scan_window_deg', 30.0))
        )
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._start_wall_time = time.time()
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_dir = os.path.join(self._log_root, stamp)
        os.makedirs(self._log_dir, exist_ok=True)

        self._csv_path = os.path.join(self._log_dir, 'samples.csv')
        self._csv_file = open(self._csv_path, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._row_count = 0

        self._last_cmd_nav: Optional[Twist] = None
        self._last_cmd_nav_time: Optional[float] = None
        self._last_cmd_vel: Optional[Twist] = None
        self._last_cmd_vel_time: Optional[float] = None
        self._last_move_cmd: Optional[MoveCmd] = None
        self._last_move_cmd_time: Optional[float] = None
        self._last_goal: Optional[PoseStamped] = None
        self._last_goal_time: Optional[float] = None
        self._last_odom: Optional[Odometry] = None
        self._last_odom_time: Optional[float] = None
        self._last_scan_time: Optional[float] = None
        self._scan_min: Optional[float] = None
        self._scan_front_min: Optional[float] = None
        self._last_bridge_status: Dict[str, Any] = {}
        self._last_bridge_status_time: Optional[float] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        odom_topic = str(self.get_parameter('odom_topic').value)
        cmd_vel_nav_topic = str(self.get_parameter('cmd_vel_nav_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        move_cmd_topic = str(self.get_parameter('move_cmd_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        bridge_status_topic = str(self.get_parameter('bridge_status_topic').value)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 20)
        self.create_subscription(Twist, cmd_vel_nav_topic, self._cmd_nav_callback, 20)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 20)
        self.create_subscription(MoveCmd, move_cmd_topic, self._move_cmd_callback, 20)
        self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 10)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            bridge_status_topic,
            self._bridge_status_callback,
            10,
        )

        self._write_metadata(
            {
                'started_at': datetime.now().isoformat(timespec='seconds'),
                'sample_rate_hz': self._sample_rate_hz,
                'map_frame': self._map_frame,
                'odom_frame': self._odom_frame,
                'base_frame': self._base_frame,
                'odom_topic': odom_topic,
                'cmd_vel_nav_topic': cmd_vel_nav_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'move_cmd_topic': move_cmd_topic,
                'goal_topic': goal_topic,
                'scan_topic': scan_topic,
                'bridge_status_topic': bridge_status_topic,
                'csv_path': self._csv_path,
            }
        )

        self.create_timer(1.0 / self._sample_rate_hz, self._sample_timer_callback)
        self.get_logger().info(
            f'run_dog runtime logger writing {self._csv_path} '
            f'at {self._sample_rate_hz:.1f} Hz'
        )

    def _param_float(self, name: str, default: float) -> float:
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _age(self, now_sec: float, stamp: Optional[float]) -> Optional[float]:
        if stamp is None:
            return None
        return max(0.0, now_sec - stamp)

    def _write_metadata(self, values: Dict[str, Any]) -> None:
        metadata_path = os.path.join(self._log_dir, 'metadata.txt')
        with open(metadata_path, 'w') as metadata_file:
            for key, value in values.items():
                metadata_file.write(f'{key}: {value}\n')

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_time = self._now_sec()

    def _cmd_nav_callback(self, msg: Twist) -> None:
        self._last_cmd_nav = msg
        self._last_cmd_nav_time = self._now_sec()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self._now_sec()

    def _move_cmd_callback(self, msg: MoveCmd) -> None:
        self._last_move_cmd = msg
        self._last_move_cmd_time = self._now_sec()

    def _goal_callback(self, msg: PoseStamped) -> None:
        self._last_goal = msg
        self._last_goal_time = self._now_sec()

    def _scan_callback(self, msg: LaserScan) -> None:
        self._last_scan_time = self._now_sec()
        valid_ranges = [
            value
            for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        ]
        self._scan_min = finite_min(valid_ranges)

        half_window = self._front_scan_window_rad * 0.5
        front_ranges = []
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(value) or value < msg.range_min or value > msg.range_max:
                continue
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if abs(angle) <= half_window:
                front_ranges.append(value)
        self._scan_front_min = finite_min(front_ranges)

    def _bridge_status_callback(self, msg: String) -> None:
        self._last_bridge_status_time = self._now_sec()
        try:
            decoded = json.loads(msg.data)
        except json.JSONDecodeError:
            self._last_bridge_status = {'reason': msg.data}
            return
        if isinstance(decoded, dict):
            self._last_bridge_status = decoded
        else:
            self._last_bridge_status = {'reason': str(decoded)}

    def _lookup_tf_pose(self, target_frame: str, source_frame: str) -> Tuple[
        Optional[float],
        Optional[float],
        Optional[float],
    ]:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException:
            return None, None, None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    def _sample_timer_callback(self) -> None:
        now_sec = self._now_sec()
        wall_time = datetime.now().isoformat(timespec='milliseconds')
        map_x, map_y, map_yaw = self._lookup_tf_pose(self._map_frame, self._base_frame)
        odom_tf_x, odom_tf_y, odom_tf_yaw = self._lookup_tf_pose(
            self._odom_frame,
            self._base_frame,
        )

        row: Dict[str, Any] = {
            'wall_time': wall_time,
            'ros_time_sec': now_sec,
            'elapsed_sec': time.time() - self._start_wall_time,
            'map_x': map_x,
            'map_y': map_y,
            'map_yaw_rad': map_yaw,
            'odom_tf_x': odom_tf_x,
            'odom_tf_y': odom_tf_y,
            'odom_tf_yaw_rad': odom_tf_yaw,
            'cmd_nav_age_sec': self._age(now_sec, self._last_cmd_nav_time),
            'cmd_vel_age_sec': self._age(now_sec, self._last_cmd_vel_time),
            'move_cmd_age_sec': self._age(now_sec, self._last_move_cmd_time),
            'goal_age_sec': self._age(now_sec, self._last_goal_time),
            'scan_age_sec': self._age(now_sec, self._last_scan_time),
            'scan_min_m': self._scan_min,
            'scan_front_min_m': self._scan_front_min,
            'bridge_status_age_sec': self._age(
                now_sec,
                self._last_bridge_status_time,
            ),
            'bridge_reason': self._last_bridge_status.get('reason'),
            'bridge_final_align_active': self._last_bridge_status.get(
                'final_align_active'
            ),
            'bridge_distance_m': self._last_bridge_status.get('distance_m'),
            'bridge_yaw_error_deg': self._last_bridge_status.get('yaw_error_deg'),
            'bridge_linear_x_cmd': self._last_bridge_status.get('linear_x_cmd'),
            'bridge_angular_z_cmd': self._last_bridge_status.get('angular_z_cmd'),
        }

        self._add_odom_fields(row)
        self._add_twist_fields(row, 'cmd_nav', self._last_cmd_nav)
        self._add_twist_fields(row, 'cmd_vel', self._last_cmd_vel)
        self._add_move_cmd_fields(row)
        self._add_goal_fields(row)

        self._writer.writerow({key: fmt(row.get(key)) for key in CSV_COLUMNS})
        self._row_count += 1
        if self._row_count % 10 == 0:
            self._csv_file.flush()

    def _add_odom_fields(self, row: Dict[str, Any]) -> None:
        if self._last_odom is None:
            return
        pose = self._last_odom.pose.pose
        twist = self._last_odom.twist.twist
        row['odom_msg_x'] = pose.position.x
        row['odom_msg_y'] = pose.position.y
        row['odom_msg_yaw_rad'] = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        row['odom_msg_vx'] = twist.linear.x
        row['odom_msg_vy'] = twist.linear.y
        row['odom_msg_wz'] = twist.angular.z

    def _add_twist_fields(
        self,
        row: Dict[str, Any],
        prefix: str,
        msg: Optional[Twist],
    ) -> None:
        if msg is None:
            return
        row[f'{prefix}_vx'] = msg.linear.x
        row[f'{prefix}_vy'] = msg.linear.y
        row[f'{prefix}_wz'] = msg.angular.z

    def _add_move_cmd_fields(self, row: Dict[str, Any]) -> None:
        if self._last_move_cmd is None:
            return
        row['move_step_x'] = self._last_move_cmd.step_x
        row['move_step_y'] = self._last_move_cmd.step_y

    def _add_goal_fields(self, row: Dict[str, Any]) -> None:
        if self._last_goal is None:
            return
        pose = self._last_goal.pose
        row['goal_frame'] = self._last_goal.header.frame_id
        row['goal_x'] = pose.position.x
        row['goal_y'] = pose.position.y
        row['goal_yaw_rad'] = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def destroy_node(self) -> bool:
        try:
            self._csv_file.flush()
            self._csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RunDogRuntimeLogger()
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
