#!/usr/bin/env python3
"""
Nav2 碰撞诊断工具
实时监控代价地图和控制器状态，输出详细的碰撞诊断信息
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class CollisionDiagnostics(Node):
    def __init__(self):
        super().__init__('collision_diagnostics')

        # 声明参数
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/collision_diagnostics')
        self.declare_parameter('warning_threshold', 200)  # 代价值警告阈值
        self.declare_parameter('critical_threshold', 230)  # 代价值危险阈值

        # 获取参数
        self.costmap_topic = self.get_parameter('local_costmap_topic').value
        self.marker_topic = self.get_parameter('collision_diagnostics').value
        self.warning_thresh = self.get_parameter('warning_threshold').value
        self.critical_thresh = self.get_parameter('critical_threshold').value

        # QoS 配置 - 与 Nav2 代价地图匹配
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 订阅代价地图
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            qos
        )

        # 发布可视化标记
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        # 状态变量
        self.current_costmap = None
        self.high_cost_detected = False
        self.critical_cost_detected = False

        self.get_logger().info('Collision Diagnostics Started')
        self.get_logger().info(f'Listening to: {self.costmap_topic}')
        self.get_logger().info(f'Publishing markers to: {self.marker_topic}')
        self.get_logger().info(f'Warning threshold: {self.warning_thresh}')
        self.get_logger().info(f'Critical threshold: {self.critical_thresh}')

    def costmap_callback(self, msg: OccupancyGrid):
        """处理代价地图更新"""
        self.current_costmap = msg

        # 分析代价地图
        stats = self.analyze_costmap(msg)

        # 检测高代价区域
        self.detect_collision_risks(msg, stats)

        # 发布可视化
        self.publish_markers(msg, stats)

    def analyze_costmap(self, msg: OccupancyGrid) -> dict:
        """分析代价地图统计信息"""
        data = np.array(msg.data)

        # 过滤掉 -1 (未知区域)
        valid_data = data[data != -1]

        if len(valid_data) == 0:
            return {
                'min': 0,
                'max': 0,
                'mean': 0,
                'free_pixels': 0,
                'obstacle_pixels': 0,
                'unknown_pixels': len(data) - len(valid_data)
            }

        return {
            'min': int(np.min(valid_data)),
            'max': int(np.max(valid_data)),
            'mean': float(np.mean(valid_data)),
            'free_pixels': int(np.sum(valid_data == 0)),
            'obstacle_pixels': int(np.sum(valid_data >= 100)),
            'unknown_pixels': int(np.sum(data == -1))
        }

    def detect_collision_risks(self, msg: OccupancyGrid, stats: dict):
        """检测碰撞风险"""
        max_cost = stats['max']

        # 只在变化时输出
        new_critical = max_cost >= self.critical_thresh
        new_warning = max_cost >= self.warning_thresh

        if new_critical != self.critical_cost_detected or new_warning != self.high_cost_detected:
            if new_critical:
                self.get_logger().error(
                    f'🔴 CRITICAL COLLISION RISK: Max cost = {max_cost} '
                    f'(threshold: {self.critical_thresh})'
                )
            elif new_warning:
                self.get_logger().warning(
                    f'🟡 COLLISION WARNING: Max cost = {max_cost} '
                    f'(threshold: {self.warning_thresh})'
                )
            elif self.high_cost_detected or self.critical_cost_detected:
                self.get_logger().info(f'✅ Cost reduced to {max_cost} - Safe')

            self.critical_cost_detected = new_critical
            self.high_cost_detected = new_warning

    def publish_markers(self, msg: OccupancyGrid, stats: dict):
        """发布 RViz 可视化标记"""
        marker_array = MarkerArray()

        # 1. 代价地图边界框
        bbox_marker = Marker()
        bbox_marker.header = msg.header
        bbox_marker.ns = 'costmap_boundary'
        bbox_marker.id = 0
        bbox_marker.type = Marker.LINE_STRIP
        bbox_marker.action = Marker.ADD
        bbox_marker.pose.orientation.w = 1.0
        bbox_marker.scale.x = 0.05
        bbox_marker.color.a = 0.5
        bbox_marker.color.r = 0.0
        bbox_marker.color.g = 1.0
        bbox_marker.color.b = 0.0

        # 添加边界点
        width = msg.info.width * msg.info.resolution
        height = msg.info.height * msg.info.resolution
        corners = [
            (0, 0), (width, 0), (width, height), (0, height), (0, 0)
        ]
        for x, y in corners:
            p = PointStamped()
            p.point.x = msg.info.origin.position.x + x
            p.point.y = msg.info.origin.position.y + y
            bbox_marker.points.append(p.point)

        marker_array.markers.append(bbox_marker)

        # 2. 状态文本标记
        text_marker = Marker()
        text_marker.header = msg.header
        text_marker.ns = 'status_text'
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = msg.info.origin.position.x
        text_marker.pose.position.y = msg.info.origin.position.y + height + 0.2
        text_marker.pose.position.z = 0.3
        text_marker.scale.z = 0.15
        text_marker.color.a = 1.0

        max_cost = stats['max']
        if max_cost >= self.critical_thresh:
            text_marker.text = f"CRITICAL: max_cost={max_cost}"
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
        elif max_cost >= self.warning_thresh:
            text_marker.text = f"WARNING: max_cost={max_cost}"
            text_marker.color.r = 1.0
            text_marker.color.g = 0.5
            text_marker.color.b = 0.0
        else:
            text_marker.text = f"SAFE: max_cost={max_cost}"
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0

        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
