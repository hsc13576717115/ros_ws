#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CurrentPosePrinter(Node):
    def __init__(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float,
    ) -> None:
        super().__init__('print_current_pose')
        self._target_frame = target_frame
        self._source_frame = source_frame
        self._timeout_sec = timeout_sec
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

    def lookup_pose(self) -> tuple[float, float, float]:
        deadline_sec = time.monotonic() + self._timeout_sec
        last_exc: TransformException | None = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    self._source_frame,
                    Time(),
                )
                break
            except TransformException as exc:
                last_exc = exc
                if time.monotonic() >= deadline_sec:
                    raise exc
        else:
            if last_exc is not None:
                raise last_exc
            raise TransformException('rclpy shutdown while waiting for transform')

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw


def format_waypoint(
    name: str,
    role: str,
    x: float,
    y: float,
    yaw: float,
    precision: int,
) -> str:
    yaw_deg = math.degrees(yaw)
    return (
        f"- {{name: {name}, role: {role}, "
        f"x: {x:.{precision}f}, y: {y:.{precision}f}, yaw_deg: {yaw_deg:.1f}}}"
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Print current robot pose and a preset_waypoints.yaml line.'
    )
    parser.add_argument('--frame', default='map', help='Target frame, usually map.')
    parser.add_argument('--base', default='base_link', help='Robot base frame.')
    parser.add_argument('--name', default='NEW_POINT', help='Waypoint name to print.')
    parser.add_argument('--role', default='carry', help='Waypoint role to print.')
    parser.add_argument('--rate', type=float, default=1.0, help='Print rate in Hz.')
    parser.add_argument('--timeout', type=float, default=0.5, help='TF lookup timeout in seconds.')
    parser.add_argument('--precision', type=int, default=3, help='x/y decimal places.')
    parser.add_argument('--once', action='store_true', help='Print once and exit.')
    args, _unknown = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = CurrentPosePrinter(args.frame, args.base, args.timeout)
    period_sec = 1.0 / max(args.rate, 0.1)

    try:
        while rclpy.ok():
            try:
                x, y, yaw = node.lookup_pose()
            except TransformException as exc:
                node.get_logger().warn(
                    f'Waiting for TF {args.frame} -> {args.base}: {exc}'
                )
                time.sleep(period_sec)
                continue

            yaw_deg = math.degrees(yaw)
            print(
                f"{args.frame}->{args.base}: "
                f"x={x:.{args.precision}f}, y={y:.{args.precision}f}, "
                f"yaw={yaw:.3f} rad ({yaw_deg:.1f} deg)"
            )
            print(format_waypoint(args.name, args.role, x, y, yaw, args.precision))
            print()
            if args.once:
                return 0
            time.sleep(period_sec)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
