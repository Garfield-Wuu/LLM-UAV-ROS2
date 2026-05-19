"""odom_ned_to_enu_node – 将 AirSim PX4 NED odometry 转换为 ENU odometry。

AirSim /airsim_node/PX4/odom_local_ned 使用 NED 坐标系：
  x=North, y=East, z=Down, 偏航从北顺时针为正

EGO-Planner 规划层使用 ENU 坐标系（world frame）：
  x=East, y=North, z=Up, 偏航从东逆时针为正

Position 转换：
  enu.x = ned.y
  enu.y = ned.x
  enu.z = -ned.z

Orientation 转换：
  将绕 z 轴的 -90° 旋转与 body 姿态复合，再做 z-flip 变换：
  R_ned2enu = Rz(90°) @ Rx(180°) -> 等效四元数 q_ned2enu
  q_enu_world = q_ned2enu ⊗ q_ned_body

Parameters:
  input_topic   (str)  default /airsim_node/PX4/odom_local_ned
  output_topic  (str)  default /uav/odom_enu
  frame_id      (str)  default world
  child_frame   (str)  default base_link
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


_SQ2_2 = math.sqrt(2.0) / 2.0  # √2/2 ≈ 0.7071


def _ned_to_enu_quat(qx, qy, qz, qw):
    """Convert body-to-NED quaternion to body-to-ENU quaternion.

    Correct formula: q_ENU = q_NED2ENU ⊗ q_NED

    NED→ENU rotation matrix:
      [[0, 1, 0],
       [1, 0, 0],
       [0, 0,-1]]
    Equivalent quaternion: q_NED2ENU = (w=0, x=√2/2, y=√2/2, z=0)
    (180° rotation about the northeast diagonal axis (1,1,0)/√2)

    Applying Hamilton product with q_NED2ENU = (0, s, s, 0) where s=√2/2:
      w_out = -s*(qx + qy)
      x_out =  s*(qw + qz)
      y_out =  s*(qw - qz)
      z_out =  s*(qy - qx)

    Verification – level drone facing north (q_NED=identity):
      Result: (x=s, y=s, z=0, w=0) → body +x maps to ENU +y (north) ✓
    Verification – level drone facing east (q_NED = 90° CW yaw in NED):
      Result: (x=1, y=0, z=0, w=0) → body +x maps to ENU +x (east) ✓
    """
    s = _SQ2_2
    w = -s * (qx + qy)
    x =  s * (qw + qz)
    y =  s * (qw - qz)
    z =  s * (qy - qx)
    return (x, y, z, w)  # returned as (x, y, z, w)


class OdomNedToEnuNode(Node):
    def __init__(self):
        super().__init__('odom_ned_to_enu_node')
        self.declare_parameter('input_topic',  '/airsim_node/PX4/odom_local_ned')
        self.declare_parameter('output_topic', '/uav/odom_enu')
        self.declare_parameter('frame_id',     'world')
        self.declare_parameter('child_frame',  'base_link')

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._frame_id    = self.get_parameter('frame_id').get_parameter_value().string_value
        self._child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # 使用 BEST_EFFORT QoS，匹配 EGO-Planner grid_map 内部的 odom subscriber 设置
        #   message_filters odom_sub_:   rclcpp::QoS(100).best_effort()
        #   indep_odom_sub_:             rclcpp::QoS(100).best_effort()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Odometry, out_topic, qos)
        # AirSim ROS wrapper 发布 odom 默认 reliable，sub 用 reliable 没问题
        self._sub = self.create_subscription(Odometry, in_topic, self._cb, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._n_published = 0

        self.get_logger().info(
            f'odom_ned_to_enu_node ready: {in_topic} -> {out_topic} (QoS=BEST_EFFORT)'
        )

    def _cb(self, msg: Odometry):
        ned_p = msg.pose.pose.position
        ned_v = msg.twist.twist.linear
        ned_av = msg.twist.twist.angular
        q = msg.pose.pose.orientation

        enu = Odometry()
        # 使用 ROS2 wall clock 时间戳，而不是 PX4 的时间戳。
        # PX4 反复出现 "time jump detected"，时钟会突然跳变几十秒，
        # 导致 odom 时间戳与 AirSim 深度图（wall clock）相差过大，
        # ApproximateTimeSynchronizer 无法匹配 → 地图停止更新。
        enu.header.stamp    = self.get_clock().now().to_msg()
        enu.header.frame_id = self._frame_id
        enu.child_frame_id  = self._child_frame

        # Position: NED → ENU
        enu.pose.pose.position.x =  ned_p.y
        enu.pose.pose.position.y =  ned_p.x
        enu.pose.pose.position.z = -ned_p.z

        # Orientation: NED body → ENU world
        ex, ey, ez, ew = _ned_to_enu_quat(q.x, q.y, q.z, q.w)
        enu.pose.pose.orientation.x = ex
        enu.pose.pose.orientation.y = ey
        enu.pose.pose.orientation.z = ez
        enu.pose.pose.orientation.w = ew

        # Linear velocity: PX4 /odom_local_ned twist.linear is in NED world frame.
        # Convert NED world velocity to ENU world velocity.
        enu.twist.twist.linear.x =  ned_v.y   # east  = NED y
        enu.twist.twist.linear.y =  ned_v.x   # north = NED x
        enu.twist.twist.linear.z = -ned_v.z   # up    = -NED z

        # Angular velocity: PX4 odom twist is in body (FRD) frame,
        # which doesn't change coordinate system when we switch world to ENU.
        # Pass through unchanged; EGO-Planner doesn't use twist anyway.
        enu.twist.twist.angular.x = ned_av.x
        enu.twist.twist.angular.y = ned_av.y
        enu.twist.twist.angular.z = ned_av.z

        # Forward pose covariance as-is (diagonal is still meaningful)
        enu.pose.covariance  = msg.pose.covariance
        enu.twist.covariance = msg.twist.covariance

        self._pub.publish(enu)

        # 广播 world -> base_link TF，让 RViz / EGO-Planner 相机变换能找到根帧
        tf_msg = TransformStamped()
        tf_msg.header.stamp = enu.header.stamp
        tf_msg.header.frame_id = self._frame_id
        tf_msg.child_frame_id = self._child_frame
        tf_msg.transform.translation.x = enu.pose.pose.position.x
        tf_msg.transform.translation.y = enu.pose.pose.position.y
        tf_msg.transform.translation.z = enu.pose.pose.position.z
        tf_msg.transform.rotation = enu.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

        self._n_published += 1
        # 每秒打印一次诊断：ENU 位置 + 朝向（前 10 条 + 之后每 250 条 ≈ 10s）
        if self._n_published <= 10 or self._n_published % 250 == 0:
            self.get_logger().info(
                f'[#{self._n_published}] enu pos=({enu.pose.pose.position.x:.2f}, '
                f'{enu.pose.pose.position.y:.2f}, {enu.pose.pose.position.z:.2f}) '
                f'quat=({ex:.3f}, {ey:.3f}, {ez:.3f}, {ew:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomNedToEnuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
