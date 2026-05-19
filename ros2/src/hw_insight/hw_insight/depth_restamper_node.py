"""depth_restamper_node – 将深度图的 header.stamp 替换为 ROS2 wall clock。

AirSim 使用 SteppableClock + LockStep，深度图 stamp 是仿真时钟；
而 odom_ned_to_enu_node 为规避 PX4 时钟跳变，改用 wall clock 发布 odom。
两者时钟源不同，ApproximateTimeSynchronizer 窗口（10s）随仿真推进会被突破
→ depthOdomCallback 停止触发 → 地图冻结 → 体素与真实场景不对应。

本节点订阅原始深度图，仅替换 header.stamp 为 wall clock 后转发，
使深度图与 odom 时钟源一致，让 synchronizer 稳定工作。

Parameters:
  input_topic   (str)  default /airsim_node/PX4/CameraDepth1/DepthPlanar
  output_topic  (str)  default /uav/depth_restamped
  camera_info_input_topic   (str) default /airsim_node/PX4/CameraDepth1/camera_info
  camera_info_output_topic  (str) default /uav/camera_info_restamped
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


_QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

_QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class DepthRestamperNode(Node):
    def __init__(self):
        super().__init__('depth_restamper_node')

        self.declare_parameter('input_topic',  '/airsim_node/PX4/CameraDepth1/DepthPlanar')
        self.declare_parameter('output_topic', '/uav/depth_restamped')
        self.declare_parameter('camera_info_input_topic',  '/airsim_node/PX4/CameraDepth1/camera_info')
        self.declare_parameter('camera_info_output_topic', '/uav/camera_info_restamped')

        in_depth  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_depth = self.get_parameter('output_topic').get_parameter_value().string_value
        in_info   = self.get_parameter('camera_info_input_topic').get_parameter_value().string_value
        out_info  = self.get_parameter('camera_info_output_topic').get_parameter_value().string_value

        self._depth_pub = self.create_publisher(Image,      out_depth, _QOS_SENSOR)
        self._info_pub  = self.create_publisher(CameraInfo, out_info,  _QOS_RELIABLE)

        self._depth_sub = self.create_subscription(Image,      in_depth, self._depth_cb, _QOS_SENSOR)
        self._info_sub  = self.create_subscription(CameraInfo, in_info,  self._info_cb,  _QOS_RELIABLE)

        self._n = 0
        self.get_logger().info(
            f'depth_restamper_node ready:\n'
            f'  depth: {in_depth} -> {out_depth}\n'
            f'  info:  {in_info} -> {out_info}'
        )

    def _depth_cb(self, msg: Image):
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        self._depth_pub.publish(msg)
        self._n += 1
        if self._n <= 5 or self._n % 200 == 0:
            self.get_logger().info(
                f'[#{self._n}] depth restamped: encoding={msg.encoding} '
                f'{msg.width}x{msg.height} stamp={now.sec}.{now.nanosec:09d}'
            )

    def _info_cb(self, msg: CameraInfo):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRestamperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
