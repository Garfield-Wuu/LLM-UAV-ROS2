"""Launch file for the semantic perception chain.

Starts three nodes in sequence:
  1. yolo_world_detector  – YOLO-World open-vocabulary detection
  2. target_grounding_node – depth-based camera-frame 3D grounding
  3. semantic_target_tf_node – camera-frame -> world-frame transform

This launch is intentionally independent of planner_integration.launch.py
so that the perception chain can be brought up / debugged separately.

Usage:
  ros2 launch hw_insight semantic_perception.launch.py
  ros2 launch hw_insight semantic_perception.launch.py texts:="person,car"
  ros2 launch hw_insight semantic_perception.launch.py publish_target_goal:=false
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

    yolo_detector = Node(
        package='hw_insight',
        executable='yolo_world_detector',
        name='yolo_world_detector',
        output='screen',
        parameters=[{
            'rgb_topic': rgb_topic,
            'texts': texts,
            'score_thr': score_thr,
            'min_inference_interval_sec': 0.3,
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
        DeclareLaunchArgument('texts', default_value='red car'),
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
        DeclareLaunchArgument('publish_target_goal', default_value='true'),
        DeclareLaunchArgument('score_thr', default_value='0.25'),
        yolo_detector,
        grounding,
        tf_node,
    ])
