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
    goal_topic = LaunchConfiguration('goal_topic')
    bspline_topic = LaunchConfiguration('bspline_topic')
    planner_cmd_vel_topic = LaunchConfiguration('planner_cmd_vel_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_velocity_bridge = LaunchConfiguration('enable_velocity_bridge')

    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    planning_horizon = LaunchConfiguration('planning_horizon')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')

    depth_topic = LaunchConfiguration('depth_topic')

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', odom_topic),
            ('grid_map/odom', odom_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/depth', depth_topic),
            ('/goal_pose', goal_topic),
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
            {'grid_map/resolution': 0.2},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 8.0},
            {'grid_map/local_update_range_y': 8.0},
            {'grid_map/local_update_range_z': 6.0},
            {'grid_map/obstacles_inflation': 0.3},
            {'grid_map/local_map_margin': 5},
            {'grid_map/ground_height': -10.0},
            {'grid_map/use_depth_filter': True},
            {'grid_map/frame_id': 'world'},
            {'grid_map/pose_type': 2},
            {'grid_map/odom_depth_timeout': 5.0},
            # Camera intrinsics – must match AirSim settings.json.
            # AirSim CameraDepth1: 640×480, FOV=90°
            # fx = fy = (640/2) / tan(45°) = 320.0
            {'grid_map/fx': 320.0},
            {'grid_map/fy': 320.0},
            {'grid_map/cx': 320.0},
            {'grid_map/cy': 240.0},
            # AirSim DepthPlanar is float32 in metres; scale to uint16 mm
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 4},
            # RealSense-like depth range: 0.25 m – 15 m (planner effective)
            {'grid_map/depth_filter_maxdist': 15.0},
            {'grid_map/depth_filter_mindist': 0.25},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_margin': 2},
            # Raycasting range
            {'grid_map/min_ray_length': 0.25},
            {'grid_map/max_ray_length': 15.0},
            # Visualisation: NED ground is z≈0; hide ground voxels from display
            {'grid_map/visualization_truncate_height': -0.3},
            {'grid_map/virtual_ceil_height': -0.5},
            {'grid_map/virtual_ceil_yp': -0.5},
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
            {'optimization/dist0': 0.6},
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
            {'command_timeout_sec': 0.35},
        ],
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
        DeclareLaunchArgument('odom_topic', default_value='/vins/odometry'),
        DeclareLaunchArgument('cloud_topic', default_value='/uav/camera/points'),
        DeclareLaunchArgument('depth_topic', default_value='/airsim_node/PX4/CameraDepth1/DepthPlanar'),
        DeclareLaunchArgument('goal_topic', default_value='/uav/target_goal'),
        DeclareLaunchArgument('bspline_topic', default_value='/uav/ego_planner/bspline'),
        DeclareLaunchArgument('planner_cmd_vel_topic', default_value='/uav/planner_cmd_vel_stamped'),
        DeclareLaunchArgument('enable_velocity_bridge', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('max_vel', default_value='2.0'),
        DeclareLaunchArgument('max_acc', default_value='3.0'),
        DeclareLaunchArgument('planning_horizon', default_value='7.5'),
        DeclareLaunchArgument('map_size_x', default_value='30.0'),
        DeclareLaunchArgument('map_size_y', default_value='30.0'),
        DeclareLaunchArgument('map_size_z', default_value='10.0'),
        ego_planner_node,
        bspline_relay,
        planner_velocity_bridge,
        rviz_node,
    ])
