"""YOLO-World 检测器单节点测试 launch 文件。

仅启动 yolo_world_detector，方便独立验证对 AirSim 视频流的检测效果。

用法:
  ros2 launch hw_insight yolo_world_test.launch.py
  ros2 launch hw_insight yolo_world_test.launch.py texts:="person,car,drone"
  ros2 launch hw_insight yolo_world_test.launch.py score_thr:=0.3 interval:=1.0

测试监控（另开终端）:
  ros2 topic echo /uav/detections_2d
  ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: 'person,car'"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'texts',
            default_value='person,car,drone,truck,bicycle',
            description='逗号分隔的检测类别，支持任意开放词汇',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/airsim_node/PX4/CameraDepth1/Scene',
            description='AirSim RGB 图像话题',
        ),
        DeclareLaunchArgument(
            'score_thr',
            default_value='0.25',
            description='检测置信度阈值',
        ),
        DeclareLaunchArgument(
            'interval',
            default_value='0.5',
            description='推理最小间隔（秒），CPU 模式建议设为 1.0~2.0',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            description="推理设备: 'auto'(自动降级) | 'cuda:0' | 'cpu'",
        ),
        DeclareLaunchArgument(
            'inference_mode',
            default_value='continuous',
            description="continuous: 按 interval+新帧轮询 | on_query: 每条 target_query 只推理一次",
        ),
        Node(
            package='hw_insight',
            executable='yolo_world_detector',
            name='yolo_world_detector',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'texts': LaunchConfiguration('texts'),
                'score_thr': LaunchConfiguration('score_thr'),
                'min_inference_interval_sec': LaunchConfiguration('interval'),
                'detections_topic': '/uav/detections_2d',
                'device': LaunchConfiguration('device'),
                'inference_mode': LaunchConfiguration('inference_mode'),
            }],
        ),
    ])
