#!/usr/bin/env python3
"""
Acceptance helper for Ego-Planner integration.

Measures:
1) Latency from /uav/target_goal publish to first /uav/ego_planner/bspline.
2) Coarse obstacle clearance check using latest point cloud against
   received B-spline control points.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import Bspline


@dataclass
class PlannerFeedbackResult:
    received_goal: bool = False
    received_bspline: bool = False
    latency_ms: Optional[float] = None
    min_clearance_m: Optional[float] = None
    clearance_ok: Optional[bool] = None


class PlannerFeedbackTester(Node):
    def __init__(self) -> None:
        super().__init__('test_planner_feedback')
        self.declare_parameter('goal_topic', '/uav/target_goal')
        self.declare_parameter('bspline_topic', '/uav/ego_planner/bspline')
        self.declare_parameter('cloud_topic', '/uav/camera/points')
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('required_clearance_m', 0.30)
        self.declare_parameter('max_cloud_points', 3000)

        goal_topic = str(self.get_parameter('goal_topic').value)
        bspline_topic = str(self.get_parameter('bspline_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.required_clearance_m = float(self.get_parameter('required_clearance_m').value)
        self.max_cloud_points = int(self.get_parameter('max_cloud_points').value)

        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_subscription(Bspline, bspline_topic, self.on_bspline, 10)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 10)

        self.start_ns = self.get_clock().now().nanoseconds
        self.goal_ts_ns: Optional[int] = None
        self.latest_cloud: List[Tuple[float, float, float]] = []
        self.result = PlannerFeedbackResult()
        self.done = False

        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            f'Listening goal={goal_topic}, bspline={bspline_topic}, cloud={cloud_topic}'
        )

    def on_goal(self, _msg: PoseStamped) -> None:
        if self.goal_ts_ns is None:
            self.goal_ts_ns = self.get_clock().now().nanoseconds
            self.result.received_goal = True
            self.get_logger().info('Received target goal, waiting planner output...')

    def on_bspline(self, msg: Bspline) -> None:
        if self.goal_ts_ns is None:
            return
        if self.result.received_bspline:
            return

        now_ns = self.get_clock().now().nanoseconds
        self.result.latency_ms = (now_ns - self.goal_ts_ns) / 1e6
        self.result.received_bspline = True

        ctrl_pts = [(p.x, p.y, p.z) for p in msg.pos_pts]
        min_dist = self._compute_min_distance(ctrl_pts, self.latest_cloud)
        self.result.min_clearance_m = min_dist
        self.result.clearance_ok = (
            None if min_dist is None else bool(min_dist >= self.required_clearance_m)
        )
        self._finish()

    def on_cloud(self, msg: PointCloud2) -> None:
        points = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append((float(p[0]), float(p[1]), float(p[2])))
            if len(points) >= self.max_cloud_points:
                break
        self.latest_cloud = points

    def on_timer(self) -> None:
        if self.done:
            return
        elapsed = (self.get_clock().now().nanoseconds - self.start_ns) / 1e9
        if elapsed > self.timeout_sec:
            self.get_logger().error('Timeout: no planner feedback in expected window.')
            self._finish()

    def _finish(self) -> None:
        if self.done:
            return
        self.done = True
        self.get_logger().info(f'Planner feedback result: {self.result}')
        rclpy.shutdown()

    @staticmethod
    def _compute_min_distance(
        traj_points: List[Tuple[float, float, float]],
        cloud_points: List[Tuple[float, float, float]],
    ) -> Optional[float]:
        if not traj_points or not cloud_points:
            return None
        min_d2 = math.inf
        for tx, ty, tz in traj_points:
            for cx, cy, cz in cloud_points:
                dx = tx - cx
                dy = ty - cy
                dz = tz - cz
                d2 = dx * dx + dy * dy + dz * dz
                if d2 < min_d2:
                    min_d2 = d2
        return math.sqrt(min_d2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerFeedbackTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
