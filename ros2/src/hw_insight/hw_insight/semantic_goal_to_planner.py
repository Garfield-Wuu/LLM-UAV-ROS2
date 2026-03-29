#!/usr/bin/env python3
"""semantic_goal_to_planner – forward the best world-frame target to EGO-Planner.

Subscribes to /uav/semantic_targets_world, picks the highest-scoring target,
and publishes it as a geometry_msgs/PoseStamped on /uav/target_goal so that
EGO-Planner can consume it via the existing planner_integration.launch.py.

This node is intentionally thin – it does not alter the planning algorithm.
Enable it only after /uav/semantic_targets_world is verified stable.

Parameters
----------
  world_targets_topic   (str)   default /uav/semantic_targets_world
  target_goal_topic     (str)   default /uav/target_goal
  min_score             (float) default 0.3   – ignore low-confidence targets
  goal_publish_rate_hz  (float) default 1.0   – rate-limit forwarding
  altitude_m            (float) default -1.0
    if >= 0 override z with this ENU altitude; useful to hold a safe height
"""

import json
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class SemanticGoalToPlanner(Node):
    """Forward best semantic target to EGO-Planner goal topic."""

    def __init__(self) -> None:
        super().__init__('semantic_goal_to_planner')

        self.declare_parameter('world_targets_topic', '/uav/semantic_targets_world')
        self.declare_parameter('target_goal_topic', '/uav/target_goal')
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('goal_publish_rate_hz', 1.0)
        self.declare_parameter('altitude_m', -1.0)

        p = self.get_parameter
        self._world_topic = str(p('world_targets_topic').value)
        self._goal_topic = str(p('target_goal_topic').value)
        self._min_score = float(p('min_score').value)
        rate = max(0.1, float(p('goal_publish_rate_hz').value))
        self._publish_interval = 1.0 / rate
        self._altitude = float(p('altitude_m').value)

        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self.create_subscription(
            String, self._world_topic, self._on_world_targets, 10
        )

        self._pending: Optional[Dict] = None
        self._last_publish_time = 0.0

        self.create_timer(0.1, self._poll_publish)
        self.get_logger().info(
            f'SemanticGoalToPlanner ready | '
            f'in={self._world_topic} out={self._goal_topic} '
            f'min_score={self._min_score} alt={self._altitude}'
        )

    def _on_world_targets(self, msg: String) -> None:
        try:
            payload: Dict = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        targets: List[Dict] = payload.get('targets', [])
        candidates = [t for t in targets if t.get('score', 0.0) >= self._min_score]
        if not candidates:
            return
        best = max(candidates, key=lambda t: t['score'])
        self._pending = best

    def _poll_publish(self) -> None:
        if self._pending is None:
            return
        now = time.monotonic()
        if now - self._last_publish_time < self._publish_interval:
            return

        tgt = self._pending
        x = float(tgt['x_world'])
        y = float(tgt['y_world'])
        z = float(tgt['z_world']) if self._altitude < 0 else self._altitude

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

        self._last_publish_time = now
        self.get_logger().info(
            f'[PLANNER_GOAL] "{tgt.get("label","")}" '
            f'score={tgt.get("score", 0):.2f} '
            f'-> ({x:.2f},{y:.2f},{z:.2f})'
        )


def main(args=None) -> None:
    """Entrypoint registered in setup.py."""
    rclpy.init(args=args)
    node = SemanticGoalToPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
