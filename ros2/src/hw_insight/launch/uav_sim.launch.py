"""uav_sim.launch.py — AirSim 仿真一键启动入口

用法：
  # 纯手动/LLM 直接速度控制（默认）
  ros2 launch hw_insight uav_sim.launch.py

  # 开启 EGO-Planner 避障规划
  ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true

  # 不开 RViz（SSH / 无头环境）
  ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true use_rviz:=false

包含节点：
  [始终] airsim_ros_pkgs   — AirSim ROS 桥接
  [始终] move_velocity      — PX4 Offboard 速度执行
  [始终] text_command_bridge — 自然语言/JSON 指令解析
  [begin_ego] depth_restamper_node  — 深度图时钟对齐
  [ego]  odom_ned_to_enu_node       — NED→ENU 里程计转换
  [ego]  ego_planner_node           — 局部轨迹规划
  [ego]  ego_bspline_to_twist_relay — B 样条→速度指令
  [ego]  planner_velocity_bridge    — 规划速度→控制通道
  [ego]  rviz2                      — 可视化（可选）
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    enable_ego_planner = LaunchConfiguration('enable_ego_planner')
    use_rviz           = LaunchConfiguration('use_rviz')
    max_vel            = LaunchConfiguration('max_vel')
    max_acc            = LaunchConfiguration('max_acc')

    # ── AirSim ROS 桥接 ───────────────────────────────────────────────────────
    airsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('airsim_ros_pkgs'),
                'launch',
                'airsim_node.launch.py',
            )
        )
    )

    # ── 静态 TF: world -> PX4（让 RViz 有根帧）────────────────────────────────
    world_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'PX4',
        ],
        output='screen',
    )

    # ── PX4 Offboard 速度执行节点 ─────────────────────────────────────────────
    move_velocity_node = Node(
        package='hw_insight',
        executable='move_velocity',
        name='move_velocity',
        output='screen',
        parameters=[
            {'command_topic': '/hw_insight/keyboard_velocity'},
        ],
    )

    # ── 自然语言/JSON 指令桥接 ────────────────────────────────────────────────
    # planner_mode_for_goto: 当 enable_ego_planner=true 时，GOTO_NED 发目标给
    # EGO-Planner 并停止自身发速度，把控制权交给 planner_velocity_bridge。
    text_command_bridge_node = Node(
        package='hw_insight',
        executable='text_command_bridge',
        name='text_command_bridge',
        output='screen',
        parameters=[
            {'publish_target_goal_on_goto': True},
            {'planner_mode_for_goto':       enable_ego_planner},
            {'target_goal_topic':           '/uav/target_goal'},
        ],
    )

    # ── EGO-Planner 子系统（仅 enable_ego_planner=true 时启动）───────────────
    ego_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hw_insight'),
                'launch',
                'ego_planner_integration.launch.py',
            )
        ),
        launch_arguments={
            'odom_topic':            '/uav/odom_enu',
            'ned_odom_topic':        '/airsim_node/PX4/odom_local_ned',
            'cloud_topic':           '/uav/camera/points',
            'depth_raw_topic':       '/airsim_node/PX4/CameraDepth1/DepthPlanar',
            'camera_info_raw_topic': '/airsim_node/PX4/CameraDepth1/camera_info',
            'depth_topic':           '/uav/depth_restamped',
            'camera_info_topic':     '/uav/camera_info_restamped',
            'goal_topic':            '/uav/target_goal',
            'bspline_topic':         '/uav/ego_planner/bspline',
            'planner_cmd_vel_topic': '/uav/planner_cmd_vel_stamped',
            'enable_velocity_bridge': 'true',
            'use_rviz':              use_rviz,
            'max_vel':               max_vel,
            'max_acc':               max_acc,
            'map_size_x':            '80.0',
            'map_size_y':            '80.0',
            'map_size_z':            '60.0',
        }.items(),
        condition=IfCondition(enable_ego_planner),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_ego_planner',
            default_value='false',
            description='true = 启用 EGO-Planner 避障规划；false = 纯速度控制',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否打开 RViz 可视化（无显示器时设 false）',
        ),
        DeclareLaunchArgument(
            'max_vel',
            default_value='1.5',
            description='EGO-Planner 最大速度 (m/s)',
        ),
        DeclareLaunchArgument(
            'max_acc',
            default_value='1.5',
            description='EGO-Planner 最大加速度 (m/s²)',
        ),
        airsim_launch,
        world_tf,
        move_velocity_node,
        text_command_bridge_node,
        ego_planner_launch,
    ])
