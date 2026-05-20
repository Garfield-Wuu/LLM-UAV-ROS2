#!/usr/bin/env python3
"""Publish a nav_msgs/Path trail from odometry for RViz visualization."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_path_node')

        self.declare_parameter('input_topic', '/uav/odom_enu')
        self.declare_parameter('output_topic', '/uav/flight_path')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('min_distance_m', 0.25)
        self.declare_parameter('max_points', 3000)
        self.declare_parameter('publish_rate_hz', 5.0)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._min_distance_m = float(self.get_parameter('min_distance_m').value)
        self._max_points = max(10, int(self.get_parameter('max_points').value))

        self._path = Path()
        self._path.header.frame_id = self._frame_id
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._last_z: float | None = None
        self._have_pose = False

        self._path_pub = self.create_publisher(Path, self._output_topic, 10)
        self.create_subscription(Odometry, self._input_topic, self._on_odom, 20)

        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._publish_path)

        self.get_logger().info(
            f'OdomPathNode: {self._input_topic} -> {self._output_topic} '
            f'(frame={self._frame_id}, min_step={self._min_distance_m}m, '
            f'max_points={self._max_points})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)

        if self._have_pose:
            dx = x - self._last_x
            dy = y - self._last_y
            dz = z - self._last_z
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self._min_distance_m:
                return

        stamp = msg.header.stamp if (
            msg.header.stamp.sec or msg.header.stamp.nanosec
        ) else self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        pose.pose = msg.pose.pose

        self._path.poses.append(pose)
        if len(self._path.poses) > self._max_points:
            self._path.poses = self._path.poses[-self._max_points:]

        self._last_x, self._last_y, self._last_z = x, y, z
        self._have_pose = True
        self._path.header.stamp = stamp

    def _publish_path(self) -> None:
        if not self._path.poses:
            return
        self._path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(self._path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
