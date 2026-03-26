#!/usr/bin/env python3
"""
Relay Ego-Planner B-spline trajectory to TwistStamped velocity commands.

This relay provides a lightweight compatibility bridge:
traj_utils/Bspline -> geometry_msgs/TwistStamped
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from traj_utils.msg import Bspline


class EgoBsplineToTwistRelay(Node):
    def __init__(self) -> None:
        super().__init__('ego_bspline_to_twist_relay')
        self.declare_parameter('input_bspline_topic', '/uav/ego_planner/bspline')
        self.declare_parameter('output_twist_topic', '/uav/planner_cmd_vel_stamped')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('bspline_timeout_sec', 0.5)
        self.declare_parameter('max_vx', 6.0)
        self.declare_parameter('max_vy', 6.0)
        self.declare_parameter('max_vz', 3.0)

        self.input_bspline_topic = str(self.get_parameter('input_bspline_topic').value)
        self.output_twist_topic = str(self.get_parameter('output_twist_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.bspline_timeout_sec = float(self.get_parameter('bspline_timeout_sec').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)

        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 20)
        self.create_subscription(Bspline, self.input_bspline_topic, self.on_bspline, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.on_timer)

        self.latest_bspline: Optional[Bspline] = None
        self.latest_recv_ns: int = 0

        self.get_logger().info(
            f'Bspline relay started: {self.input_bspline_topic} -> {self.output_twist_topic}'
        )

    def on_bspline(self, msg: Bspline) -> None:
        self.latest_bspline = msg
        self.latest_recv_ns = self.get_clock().now().nanoseconds

    def on_timer(self) -> None:
        if self.latest_bspline is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.latest_recv_ns > 0:
            age_sec = (now_ns - self.latest_recv_ns) / 1e9
            if age_sec > self.bspline_timeout_sec:
                return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        vel = self._estimate_velocity(self.latest_bspline)
        msg.twist.linear.x = vel[0]
        msg.twist.linear.y = vel[1]
        msg.twist.linear.z = vel[2]
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

    def _estimate_velocity(self, bspline: Bspline) -> tuple[float, float, float]:
        pts = bspline.pos_pts
        knots = bspline.knots
        if len(pts) < 3 or len(knots) < 2:
            return 0.0, 0.0, 0.0

        dt = abs(float(knots[1] - knots[0]))
        if dt < 1e-4:
            dt = 0.05

        # Central finite difference around early segment for smooth feed-forward.
        p0 = pts[0]
        p1 = pts[1]
        p2 = pts[2]
        vx = (float(p2.x) - float(p0.x)) / (2.0 * dt)
        vy = (float(p2.y) - float(p0.y)) / (2.0 * dt)
        vz = (float(p2.z) - float(p0.z)) / (2.0 * dt)

        return (
            self._clamp(vx, -self.max_vx, self.max_vx),
            self._clamp(vy, -self.max_vy, self.max_vy),
            self._clamp(vz, -self.max_vz, self.max_vz),
        )

    @staticmethod
    def _clamp(v: float, low: float, high: float) -> float:
        return max(low, min(high, float(v)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EgoBsplineToTwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
