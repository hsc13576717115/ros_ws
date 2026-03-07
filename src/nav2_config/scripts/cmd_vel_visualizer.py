#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CmdVelVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_visualizer')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/cmd_vel_arrow')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_to_lateral_scale', 0.35)
        self.declare_parameter('min_length', 0.02)
        self.declare_parameter('show_text', True)
        self.declare_parameter('text_height', 0.32)
        self.declare_parameter('text_scale', 0.16)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._marker_topic = self.get_parameter('marker_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._linear_scale = float(self.get_parameter('linear_scale').value)
        self._angular_to_lateral_scale = float(
            self.get_parameter('angular_to_lateral_scale').value
        )
        self._min_length = float(self.get_parameter('min_length').value)
        self._show_text = self._as_bool(self.get_parameter('show_text').value)
        self._text_height = float(self.get_parameter('text_height').value)
        self._text_scale = float(self.get_parameter('text_scale').value)

        self._marker_pub = self.create_publisher(Marker, self._marker_topic, 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, self._cmd_vel_topic, self._cmd_vel_callback, 20
        )
        self.get_logger().info(
            f'Visualizer {self._cmd_vel_topic} -> {self._marker_topic} started.'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.15

        end = Point()
        end.x = msg.linear.x * self._linear_scale
        end.y = msg.angular.z * self._angular_to_lateral_scale
        end.z = 0.15

        length = (end.x ** 2 + end.y ** 2) ** 0.5
        if length < self._min_length:
            end.x = self._min_length
            end.y = 0.0

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = 'cmd_vel'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 0.95
        marker.color.r = 0.15
        marker.color.g = 0.9
        marker.color.b = 0.25
        marker.points = [start, end]
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000

        self._marker_pub.publish(marker)

        if not self._show_text:
            return

        text = Marker()
        text.header.stamp = marker.header.stamp
        text.header.frame_id = self._frame_id
        text.ns = 'cmd_vel'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = self._text_height
        text.pose.orientation.w = 1.0
        text.scale.z = self._text_scale
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = f'vx={msg.linear.x:+.2f} m/s  wz={msg.angular.z:+.2f} rad/s'
        text.lifetime.sec = 0
        text.lifetime.nanosec = 300_000_000

        self._marker_pub.publish(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelVisualizer()
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
