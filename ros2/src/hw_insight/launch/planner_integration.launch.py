import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    enable_detection_overlay = LaunchConfiguration('enable_detection_overlay')

    airsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('airsim_ros_pkgs'),
                'launch',
                'airsim_node.launch.py',
            )
        )
    )

    ego_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hw_insight'),
                'launch',
                'ego_planner_integration.launch.py',
            )
        ),
        launch_arguments={
            # odom_topic 现在是 ENU odom 输出话题（由内置 odom_ned_to_enu_node 产生）
            'odom_topic':     '/uav/odom_enu',
            # ned_odom_topic 是 AirSim 原始 NED odom，由桥接节点消费
            'ned_odom_topic': '/airsim_node/PX4/odom_local_ned',
            'cloud_topic':    '/uav/camera/points',
            # depth_raw / camera_info_raw: AirSim 原始话题（仿真时钟），由 depth_restamper 接收
            'depth_raw_topic':        '/airsim_node/PX4/CameraDepth1/DepthPlanar',
            'camera_info_raw_topic':  '/airsim_node/PX4/CameraDepth1/camera_info',
            # depth_topic / camera_info_topic: depth_restamper 输出（wall clock），供 ego_planner 使用
            'depth_topic':            '/uav/depth_restamped',
            'camera_info_topic':      '/uav/camera_info_restamped',
            'goal_topic':     '/uav/target_goal',
            'bspline_topic':  '/uav/ego_planner/bspline',
            'planner_cmd_vel_topic': '/uav/planner_cmd_vel_stamped',
            'enable_velocity_bridge': 'true',
            'use_rviz': use_rviz,
            'enable_detection_overlay': enable_detection_overlay,
        }.items(),
    )

    # Static TF: world -> world_ned (identity) so that the ego_planner's
    # 'world'-frame occupancy data can coexist with AirSim's NED frames.
    world_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'PX4',
        ],
        output='screen',
    )

    move_velocity_node = Node(
        package='hw_insight',
        executable='move_velocity',
        name='move_velocity',
        output='screen',
        parameters=[
            {'command_topic': '/hw_insight/keyboard_velocity'},
        ],
    )

    text_command_bridge_node = Node(
        package='hw_insight',
        executable='text_command_bridge',
        name='text_command_bridge',
        output='screen',
        parameters=[
            {'publish_target_goal_on_goto': True},
            {'planner_mode_for_goto': True},
            {'target_goal_topic': '/uav/target_goal'},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'enable_detection_overlay',
            default_value='true',
            description='传给 ego_planner_integration：是否启动 YOLO 检测框 RViz 叠图节点',
        ),
        airsim_launch,
        world_tf,
        move_velocity_node,
        text_command_bridge_node,
        ego_planner_launch,
    ])
