#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_point(point: Tuple[float, float, float], quaternion) -> Tuple[float, float, float]:
    x, y, z = point
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w

    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return rx, ry, rz


class PointCloudToScanNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_to_scan")

        self.declare_parameter("input_topic", "/fastlio/cloud_registered_body")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("point_skip", 1)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("min_height", 0.05)
        self.declare_parameter("max_height", 0.60)
        self.declare_parameter("range_min", 0.45)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("inf_epsilon", 0.01)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._queue_size = max(1, int(self.get_parameter("queue_size").value))
        self._point_skip = max(1, int(self.get_parameter("point_skip").value))
        self._transform_timeout_sec = float(
            self.get_parameter("transform_timeout_sec").value
        )
        self._min_height = float(self.get_parameter("min_height").value)
        self._max_height = float(self.get_parameter("max_height").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._angle_increment = float(self.get_parameter("angle_increment").value)
        self._scan_time = float(self.get_parameter("scan_time").value)
        self._use_inf = bool(self.get_parameter("use_inf").value)
        self._inf_epsilon = float(self.get_parameter("inf_epsilon").value)

        self._range_count = int(
            math.floor((self._angle_max - self._angle_min) / self._angle_increment)
        ) + 1
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._publisher = self.create_publisher(LaserScan, self._output_topic, 10)
        self._subscriber = self.create_subscription(
            PointCloud2, self._input_topic, self._pointcloud_callback, self._queue_size
        )

        self.get_logger().info(
            f"Projecting {self._input_topic} -> {self._output_topic} in frame {self._target_frame}"
        )

    def _lookup_transform(self, source_frame: str):
        if source_frame == self._target_frame:
            return None
        return self._tf_buffer.lookup_transform(
            self._target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self._transform_timeout_sec),
        )

    def _transform_point(
        self, point: Tuple[float, float, float], transform
    ) -> Tuple[float, float, float]:
        if transform is None:
            return point
        rx, ry, rz = rotate_point(point, transform.transform.rotation)
        return (
            rx + transform.transform.translation.x,
            ry + transform.transform.translation.y,
            rz + transform.transform.translation.z,
        )

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            transform = self._lookup_transform(msg.header.frame_id)
        except TransformException as exc:
            self.get_logger().warn(
                f"Skipping point cloud because transform {msg.header.frame_id} -> {self._target_frame} is unavailable: {exc}"
            )
            return

        ranges = [math.inf] * self._range_count
        fallback_range = self._range_max + self._inf_epsilon

        for index, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % self._point_skip != 0:
                continue

            x, y, z = self._transform_point((float(point[0]), float(point[1]), float(point[2])), transform)

            if z < self._min_height or z > self._max_height:
                continue

            radius = math.hypot(x, y)
            if radius < self._range_min or radius > self._range_max:
                continue

            angle = math.atan2(y, x)
            if angle < self._angle_min or angle > self._angle_max:
                continue

            bucket = int((angle - self._angle_min) / self._angle_increment)
            if 0 <= bucket < self._range_count and radius < ranges[bucket]:
                ranges[bucket] = radius

        if not self._use_inf:
            ranges = [value if math.isfinite(value) else fallback_range for value in ranges]

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self._target_frame
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self._scan_time
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges

        self._publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
