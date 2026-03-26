# 集成可行性评估报告

## 1. 结论概览

- `ego-planner-ros2-sim` 不是核心算法仓库，本质是 Gazebo/PX4 启动与桥接脚本集合。
- 核心算法实际来自 `ego-swarm-ros2` 的 `planner/*` 与 `utils/quadrotor_msgs`。
- 对你当前 `AirSim + PX4 + ROS2 Humble` 工作区，推荐仅迁移“核心算法层”，不引入 Gazebo 与 mavros2 路径。

## 2. 第三方仓库结构识别

### 2.1 `ego-planner-ros2-sim`（仿真/适配壳）

- `px4_sitl_ros2.launch.py`: 启动 mavros、Gazebo Harmonic、ros_gz_bridge。
- `simulation-gazebo`, `ego.sdf`: Gazebo 资源下载与世界文件。
- `depth_gz_bridge.py`: Gazebo 图像桥接脚本。

这部分归类为 **Gazebo/仿真环境适配**，不是规划算法实现。

### 2.2 `ego-swarm-ros2`（核心算法实现）

核心规划算法包（建议迁移）：

- `planner/ego_planner`（等价于原 ROS1 的 plan_manage 角色）
- `planner/path_searching`
- `planner/bspline_opt`
- `planner/plan_env`
- `planner/traj_utils`
- `utils/quadrotor_msgs`（轨迹/控制消息）

仿真/控制适配包（建议不迁移到核心子工作区）：

- `uav_simulator/*`（map_generator, local_sensing, so3_control, fake_drone, mockamap）
- `px4_ego_agent`
- `utils/odom_visualization`, `utils/waypoint_generator`, `utils/pose_utils`, `utils/uav_utils`, `utils/multi_map_server`

## 3. 依赖清单（基于 package.xml + CMakeLists.txt）

### 3.1 核心算法层直接依赖

- ROS2: `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `builtin_interfaces`, `message_filters`, `rosidl_default_generators`
- 点云/视觉: `PCL`, `pcl_conversions`, `cv_bridge`, `OpenCV`
- 数学库: `Eigen3`
- 内部包: `traj_utils`, `plan_env`, `path_searching`, `bspline_opt`, `quadrotor_msgs`

### 3.2 关于 `nlopt/fftw/uav_utils`

- `nlopt`: 本 ROS2 版本未在核心包中显式使用（`find_package` 与源码 include 均未发现）。
- `fftw`: 本 ROS2 版本未在核心包中显式使用。
- `uav_utils`: 存在于仓库但核心规划链未直接依赖（主要被仿真控制包使用）。

## 4. 与当前工作区冲突点评估

### 4.1 Topic/消息模型冲突

- 规划器输出原生为 `traj_utils/msg/Bspline`，你的执行层消费 `HWSimpleKeyboardInfo`。
- 已通过 `ego_bspline_to_twist_relay` + `planner_velocity_bridge` 解决：  
  `Bspline -> TwistStamped -> HWSimpleKeyboardInfo`。

### 4.2 坐标系冲突

- 规划层默认 `world`（ENU）语义；你的 PX4 执行层底层是 NED。
- 当前策略：在规划/感知层保持 `world(ENU)`，在执行桥接层处理 ENU/NED（与 PRD 分层一致）。

### 4.3 运行时中间件冲突

- 在 WSL2/FastDDS 环境下，默认 SHM 传输可能报错。  
- 启动时建议设置：`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

### 4.4 代码稳定性风险

- `ego_planner` 原代码在 ROS2 回调里存在潜在执行器冲突/崩溃路径。  
- 已在本地迁移副本修复（非阻塞状态切换 + odom 未就绪时忽略目标）。

## 5. 建议迁移的核心包清单

建议保留到 `src/external/ego_planner_core`：

- `planner/ego_planner`
- `planner/path_searching`
- `planner/bspline_opt`
- `planner/plan_env`
- `planner/traj_utils`
- `utils/quadrotor_msgs`

## 6. 缺失依赖清单

本次在当前环境已完成编译通过，**未出现阻断型缺失依赖**。  
建议按“可选增强项”跟进：

- `sensor_msgs_py`（用于 Python 验收脚本点云解析）
- 若后续启用完整仿真包，再补齐：`pcl_ros`, `laser_geometry`, `tf2_geometry_msgs`, `mavros2`（仅仿真链需要）
