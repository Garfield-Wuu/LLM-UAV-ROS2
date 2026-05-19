import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    ned_odom_topic = LaunchConfiguration('ned_odom_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    bspline_topic = LaunchConfiguration('bspline_topic')
    planner_cmd_vel_topic = LaunchConfiguration('planner_cmd_vel_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_velocity_bridge = LaunchConfiguration('enable_velocity_bridge')
    enable_detection_overlay = LaunchConfiguration('enable_detection_overlay')

    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    planning_horizon = LaunchConfiguration('planning_horizon')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    map_resolution = LaunchConfiguration('map_resolution')
    local_update_range_x = LaunchConfiguration('local_update_range_x')
    local_update_range_y = LaunchConfiguration('local_update_range_y')
    local_update_range_z = LaunchConfiguration('local_update_range_z')
    skip_pixel = LaunchConfiguration('skip_pixel')
    depth_filter_maxdist = LaunchConfiguration('depth_filter_maxdist')
    max_ray_length = LaunchConfiguration('max_ray_length')

    depth_topic = LaunchConfiguration('depth_topic')
    depth_raw_topic = LaunchConfiguration('depth_raw_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_info_raw_topic = LaunchConfiguration('camera_info_raw_topic')

    # depth_restamper: 将 AirSim 仿真时钟的深度图/camera_info stamp 替换为 wall clock，
    # 与 odom_ned_to_enu_node 的 wall clock odom 对齐，避免 ApproximateTimeSynchronizer
    # 因双时钟源漂移（实测初始差值 ~9s，随仿真推进超过 10s）导致地图冻结。
    depth_restamper_node = Node(
        package='hw_insight',
        executable='depth_restamper_node',
        name='depth_restamper_node',
        output='screen',
        parameters=[
            {'input_topic':               depth_raw_topic},
            {'output_topic':              depth_topic},
            {'camera_info_input_topic':   camera_info_raw_topic},
            {'camera_info_output_topic':  camera_info_topic},
        ],
    )

    # goal relay：ego_planner 内部硬编码订阅 /goal_pose（绝对路径），
    # launch remapping 对绝对路径无效，改用 topic relay 转发
    goal_relay = Node(
        package='topic_tools',
        executable='relay',
        name='goal_relay',
        output='screen',
        arguments=[goal_topic, '/goal_pose'],
    )

    # NED → ENU odom 桥接节点：将 AirSim NED odom 转换为 ENU world frame
    odom_ned_to_enu_node = Node(
        package='hw_insight',
        executable='odom_ned_to_enu_node',
        name='odom_ned_to_enu_node',
        output='screen',
        parameters=[
            {'input_topic':  ned_odom_topic},
            {'output_topic': odom_topic},
            {'frame_id':     'world'},
            {'child_frame':  'base_link'},
        ],
    )

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world',    odom_topic),
            ('grid_map/odom', odom_topic),
            # cloud 路径暂时禁用：depth_image_proc 输出的点云仍是 camera frame
            # 等坐标系稳定后再启用，改用 world frame 点云
            # ('grid_map/cloud', cloud_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/camera_info', camera_info_topic),
            # ego_planner 内部用绝对路径 /goal_pose，ROS2 launch remapping 对绝对路径无效
            # 改为用 topic_tools relay 把 goal_topic 转发到 /goal_pose（见下方 goal_relay 节点）
            ('planning/bspline', bspline_topic),
        ],
        parameters=[
            {'fsm/flight_type': 1},  # MANUAL_TARGET
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            # ── Grid map (RealSense D435 style: 87°×58°, 0.25-30m) ──
            {'grid_map/resolution': map_resolution},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': local_update_range_x},
            {'grid_map/local_update_range_y': local_update_range_y},
            {'grid_map/local_update_range_z': local_update_range_z},
            {'grid_map/obstacles_inflation': 0.15},
            {'grid_map/local_map_margin': 5},
            # ground_height = 0.0：地图 Z 轴从 0m 开始（AirSim Blocks 地面在 ENU z≈0）。
            # 不用此参数过滤地面——改用 maxdist 限制来排除地面（见下方）。
            {'grid_map/ground_height': 0.0},
            # use_depth_filter=True：启用空间距离过滤（mindist/maxdist），
            # 注意：EGO-Planner 原代码中时序一致性检验已被 if(false) 禁用，
            # 所以 dt<0 对此分支没有任何影响，可以安全开启。
            # 关闭此项会导致 use_depth_filter=false 路径中的 row_ptr++ 列偏移 bug：
            # u 按 skip_pix=4 步进但指针每次只 +1，深度值与投影列严重不对应。
            {'grid_map/use_depth_filter': True},
            {'grid_map/frame_id': 'world'},
            {'grid_map/pose_type': 2},
            # 宽松超时：避免 AirSim 短暂卡帧触发 flag_depth_odom_timeout_ 锁死状态机
            {'grid_map/odom_depth_timeout': 30.0},
            # Camera intrinsics fallback: 320x240, FOV=90° → fx=fy=160, cx=160, cy=120
            # formula: fx = (width/2) / tan(FOV/2) = 160/tan(45°) = 160
            # Overridden at runtime by grid_map/camera_info subscription.
            {'grid_map/fx': 160.0},
            {'grid_map/fy': 160.0},
            {'grid_map/cx': 160.0},
            {'grid_map/cy': 120.0},
            # AirSim DepthPlanar is float32 in metres; scale to uint16 mm
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': skip_pixel},
            # mindist=0.3m: 排除无人机本体近场噪声。
            # maxdist 默认放宽到 22m：AirSim Blocks 中远处墙/球常落在 12-20m；
            # 该值必须是 float（加 .0），ROS2 参数类型严格，整数会导致节点 abort。
            {'grid_map/depth_filter_maxdist': depth_filter_maxdist},
            {'grid_map/depth_filter_mindist': 0.3},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_margin': 2},
            # Raycasting range
            {'grid_map/min_ray_length': 0.25},
            {'grid_map/max_ray_length': max_ray_length},
            # ENU 可视化：显示 z < 10m 以内的体素（覆盖 Blocks 环境方块高度约 8m）
            {'grid_map/visualization_truncate_height': 25.0},
            # 虚拟天花板 / 围墙：关闭（设小于 -0.5 即可关闭，见 grid_map.cpp line 691, 903）
            # 这些参数会在 z=ceil 平面/y 边界整层填充占据体素，干扰可视化判读，
            # 飞行高度上限改用 manager/optimization 层的安全限速去约束
            {'grid_map/virtual_ceil_height': -10.0},
            {'grid_map/virtual_ceil_yp':     -10.0},
            {'grid_map/virtual_ceil_yn':     -10.0},
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': True},
            {'manager/drone_id': 0},
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.3},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},
        ],
    )

    bspline_relay = Node(
        package='hw_insight',
        executable='ego_bspline_to_twist_relay',
        name='ego_bspline_to_twist_relay',
        output='screen',
        parameters=[
            {'input_bspline_topic': bspline_topic},
            {'output_twist_topic': planner_cmd_vel_topic},
            {'publish_rate_hz': 20.0},
            {'bspline_timeout_sec': 1.5},
            {'max_vx': 6.0},
            {'max_vy': 6.0},
            {'max_vz': 3.0},
        ],
    )

    planner_velocity_bridge = Node(
        package='hw_insight',
        executable='planner_velocity_bridge',
        name='planner_velocity_bridge',
        output='screen',
        condition=IfCondition(enable_velocity_bridge),
        parameters=[
            {'planner_twist_stamped_topic': planner_cmd_vel_topic},
            {'planner_twist_topic': '/uav/planner_cmd_vel'},
            {'prefer_stamped': True},
            {'output_topic': '/hw_insight/keyboard_velocity'},
            {'command_timeout_sec': 1.0},
        ],
    )

    detections_overlay_node = Node(
        package='hw_insight',
        executable='detections_image_overlay',
        name='detections_image_overlay',
        output='screen',
        condition=IfCondition(enable_detection_overlay),
        parameters=[{
            'rgb_topic': '/airsim_node/PX4/CameraDepth1/Scene',
            'detections_topic': '/uav/detections_2d',
            'output_topic': '/uav/camera/detections_overlay',
            'semantic_targets_topic': '/uav/semantic_targets_camera',
            'use_semantic_depth': True,
            'stamp_tolerance_sec': 0.35,
        }],
    )

    rviz_config = os.path.join(
        get_package_share_directory('hw_insight'),
        'rviz',
        'ego_planner_debug.rviz',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='ego_planner_rviz',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        # odom_topic 接收 ENU odom（由 odom_ned_to_enu_node 输出）
        DeclareLaunchArgument('odom_topic',    default_value='/uav/odom_enu'),
        # ned_odom_topic 是 AirSim 原始 NED odom 输入
        DeclareLaunchArgument('ned_odom_topic', default_value='/airsim_node/PX4/odom_local_ned'),
        DeclareLaunchArgument('cloud_topic',   default_value='/uav/camera/points'),
        # depth_topic / camera_info_topic: restamper 的输出（wall clock），供 ego_planner 消费
        DeclareLaunchArgument('depth_topic',         default_value='/uav/depth_restamped'),
        DeclareLaunchArgument('camera_info_topic',   default_value='/uav/camera_info_restamped'),
        # depth_raw_topic / camera_info_raw_topic: AirSim 原始话题（仿真时钟），restamper 的输入
        DeclareLaunchArgument('depth_raw_topic',
            default_value='/airsim_node/PX4/CameraDepth1/DepthPlanar'),
        DeclareLaunchArgument('camera_info_raw_topic',
            default_value='/airsim_node/PX4/CameraDepth1/camera_info'),
        DeclareLaunchArgument('goal_topic',    default_value='/uav/target_goal'),
        DeclareLaunchArgument('bspline_topic', default_value='/uav/ego_planner/bspline'),
        DeclareLaunchArgument('planner_cmd_vel_topic', default_value='/uav/planner_cmd_vel_stamped'),
        DeclareLaunchArgument('enable_velocity_bridge', default_value='true'),
        DeclareLaunchArgument('use_rviz',      default_value='true'),
        DeclareLaunchArgument(
            'enable_detection_overlay',
            default_value='true',
            description='若 true，将 /uav/detections_2d 叠到 RGB 上发布 /uav/camera/detections_overlay（需运行 yolo_world_detector 或 semantic_perception）',
        ),
        # 初期保守参数：地图可信前不用高速
        DeclareLaunchArgument('max_vel',       default_value='0.8'),
        DeclareLaunchArgument('max_acc',       default_value='1.5'),
        DeclareLaunchArgument('planning_horizon', default_value='5.0'),
        DeclareLaunchArgument('map_size_x',    default_value='300.0'),
        DeclareLaunchArgument('map_size_y',    default_value='300.0'),
        # ENU 下地图 z 范围：ground_height=-0.5，z 上限=-0.5+12=11.5m
        DeclareLaunchArgument('map_size_z',    default_value='25.0'),
        # AirSim Blocks 调试默认：更远的障碍可见，同时降低体素/点云密度减轻 RViz 压力
        DeclareLaunchArgument('map_resolution', default_value='0.30'),
        DeclareLaunchArgument('local_update_range_x', default_value='40.0'),
        DeclareLaunchArgument('local_update_range_y', default_value='40.0'),
        DeclareLaunchArgument('local_update_range_z', default_value='12.0'),
        DeclareLaunchArgument('skip_pixel', default_value='6'),
        DeclareLaunchArgument('depth_filter_maxdist', default_value='22.0'),
        DeclareLaunchArgument('max_ray_length', default_value='28.0'),
        goal_relay,
        depth_restamper_node,
        odom_ned_to_enu_node,
        ego_planner_node,
        bspline_relay,
        planner_velocity_bridge,
        detections_overlay_node,
        rviz_node,
    ])
