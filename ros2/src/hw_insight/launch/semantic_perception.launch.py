"""Launch file for the semantic perception chain.

Starts three nodes in sequence:
  1. yolo_world_detector  – YOLO-World open-vocabulary detection
  2. target_grounding_node – depth-based camera-frame 3D grounding
  3. semantic_target_tf_node – camera-frame -> world-frame transform

This launch is intentionally independent of planner_integration.launch.py
so that the perception chain can be brought up / debugged separately.

Usage:
  ros2 launch hw_insight semantic_perception.launch.py
    # 默认：inference_mode=on_query, publish_target_goal=false, texts=car
    # → 启动后不自动跑 YOLO；每发一条 /uav/target_query 才检测一次；不向 planner 发 /uav/target_goal
  ros2 launch hw_insight semantic_perception.launch.py texts:="person,car"
  ros2 launch hw_insight semantic_perception.launch.py inference_mode:=continuous publish_target_goal:=true
    # 恢复旧行为：持续检测 + 检出后自动发目标给规划
  ros2 launch hw_insight semantic_perception.launch.py interval:=10.0 score_thr:=0.15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    texts = LaunchConfiguration('texts')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    publish_target_goal = LaunchConfiguration('publish_target_goal')
    score_thr = LaunchConfiguration('score_thr')
    interval = LaunchConfiguration('interval')
    device = LaunchConfiguration('device')
    inference_mode = LaunchConfiguration('inference_mode')

    yolo_detector = Node(
        package='hw_insight',
        executable='yolo_world_detector',
        name='yolo_world_detector',
        output='screen',
        parameters=[{
            'rgb_topic': rgb_topic,
            'texts': texts,
            'score_thr': score_thr,
            'min_inference_interval_sec': interval,
            'device': device,
            'inference_mode': inference_mode,
        }],
    )

    grounding = Node(
        package='hw_insight',
        executable='target_grounding_node',
        name='target_grounding_node',
        output='screen',
        parameters=[{
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
        }],
    )

    tf_node = Node(
        package='hw_insight',
        executable='semantic_target_tf_node',
        name='semantic_target_tf_node',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'publish_target_goal': publish_target_goal,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'texts',
            default_value='car',
            description='初始检测词；on_query 下仅作占位，实际以 /uav/target_query 为准',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/airsim_node/PX4/CameraDepth1/Scene',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/airsim_node/PX4/CameraDepth1/DepthPlanar',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/airsim_node/PX4/CameraDepth1/camera_info',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/airsim_node/PX4/odom_local_ned',
        ),
        DeclareLaunchArgument(
            'publish_target_goal',
            default_value='false',
            description='是否在检出后向 /uav/target_goal 发布 PoseStamped（接 planner 时改为 true）',
        ),
        DeclareLaunchArgument('score_thr', default_value='0.25'),
        DeclareLaunchArgument(
            'interval',
            default_value='0.3',
            description='continuous 模式下最小推理间隔（秒）；on_query 下不用于轮询',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            description="YOLO 推理设备: 'auto' | 'cuda:0' | 'cpu'",
        ),
        DeclareLaunchArgument(
            'inference_mode',
            default_value='on_query',
            description="YOLO 调度: on_query（默认，每条 /uav/target_query 触发一次）| continuous（按 interval+新帧）",
        ),
        yolo_detector,
        grounding,
        tf_node,
    ])
