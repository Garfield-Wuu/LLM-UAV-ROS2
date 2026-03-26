#!/usr/bin/env python3
"""
PlannerVelocityBridge

Bridge planner velocity output (ROS2 geometry_msgs) into the existing
HWSimpleKeyboardInfo control channel used by move_velocity.

Design goals:
- Keep current hw_insight control chain unchanged by default.
- Be safe when planner stream is lost (publish one-shot zero command).
- Avoid interfering with manual/LLM control when planner is idle.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from hw_interface.msg import HWSimpleKeyboardInfo


class PlannerVelocityBridge(Node):
    def __init__(self) -> None:
        super().__init__('planner_velocity_bridge')

        self.declare_parameter('planner_twist_topic', '/uav/planner_cmd_vel')
        self.declare_parameter('planner_twist_stamped_topic', '/uav/planner_cmd_vel_stamped')
        self.declare_parameter('output_topic', '/hw_insight/keyboard_velocity')
        self.declare_parameter('prefer_stamped', True)
        self.declare_parameter('command_timeout_sec', 0.35)
        self.declare_parameter('max_vx', 15.0)
        self.declare_parameter('max_vy', 15.0)
        self.declare_parameter('max_vz', 5.0)
        self.declare_parameter('max_yaw_rate', 1.5)

        self.planner_twist_topic = self.get_parameter('planner_twist_topic').value
        self.planner_twist_stamped_topic = self.get_parameter('planner_twist_stamped_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.prefer_stamped = bool(self.get_parameter('prefer_stamped').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.output_pub = self.create_publisher(HWSimpleKeyboardInfo, self.output_topic, 10)

        # Subscribe both interfaces to maximize compatibility with third-party planners.
        self.create_subscription(TwistStamped, self.planner_twist_stamped_topic, self.on_twist_stamped, 10)
        self.create_subscription(Twist, self.planner_twist_topic, self.on_twist, 10)

        self.last_cmd: Optional[HWSimpleKeyboardInfo] = None
        self.last_cmd_ns: int = 0
        self.stream_active: bool = False

        self.create_timer(0.05, self.on_timer)  # 20Hz relay
        self.get_logger().info(
            'PlannerVelocityBridge started: '
            f'twist={self.planner_twist_topic}, '
            f'twist_stamped={self.planner_twist_stamped_topic}, '
            f'output={self.output_topic}'
        )

    def on_twist_stamped(self, msg: TwistStamped) -> None:
        if not self.prefer_stamped and self.last_cmd is not None:
            return
        self._store_cmd(msg.twist)

    def on_twist(self, msg: Twist) -> None:
        if self.prefer_stamped and self.last_cmd is not None:
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self.last_cmd_ns) / 1e9 <= self.command_timeout_sec:
                return
        self._store_cmd(msg)

    def _store_cmd(self, twist: Twist) -> None:
        cmd = HWSimpleKeyboardInfo()
        cmd.x = self._clamp(twist.linear.x, -self.max_vx, self.max_vx)
        cmd.y = self._clamp(twist.linear.y, -self.max_vy, self.max_vy)
        cmd.z = self._clamp(twist.linear.z, -self.max_vz, self.max_vz)
        cmd.yaw = self._clamp(twist.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        self.last_cmd = cmd
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_cmd is None:
            return

        age_sec = (now_ns - self.last_cmd_ns) / 1e9
        if age_sec <= self.command_timeout_sec:
            self.output_pub.publish(self.last_cmd)
            self.stream_active = True
            return

        # If planner stream timed out after being active, publish one-shot zero
        # command to safely stop motion, then stay silent.
        if self.stream_active:
            zero = HWSimpleKeyboardInfo()
            zero.x = 0.0
            zero.y = 0.0
            zero.z = 0.0
            zero.yaw = 0.0
            self.output_pub.publish(zero)
            self.stream_active = False

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerVelocityBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
