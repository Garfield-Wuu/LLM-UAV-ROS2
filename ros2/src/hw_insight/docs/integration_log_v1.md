# Ego-Planner 集成日志 v1

## 1. 迁移目录与工作区规划

已建立独立子工作区目录：

```text
ros2/src/external/ego_planner_core/
├── planner/
│   ├── ego_planner
│   ├── path_searching
│   ├── bspline_opt
│   ├── plan_env
│   ├── traj_utils
│   └── drone_detect
└── utils/
    └── quadrotor_msgs
```

并对原始外部仓库设置了 `COLCON_IGNORE`，避免污染主工作区扫描。

## 2. 接口对齐与配置

- 映射配置文件：`hw_insight/config/mapping_config.yaml`
- 目标输入链路：`/uav/target_goal -> /goal_pose`
- 状态输入链路：`/vins/odometry -> {odom_world, grid_map/odom}`
- 感知输入链路：`/uav/camera/points -> grid_map/cloud`
- 输出执行链路：
  - `/uav/ego_planner/bspline` (`traj_utils/Bspline`)
  - `ego_bspline_to_twist_relay` -> `/uav/planner_cmd_vel_stamped`
  - `planner_velocity_bridge` -> `/hw_insight/keyboard_velocity`

## 3. 启动与集成文件

新增：

- `hw_insight/launch/ego_planner_integration.launch.py`
- `hw_insight/hw_insight/ego_bspline_to_twist_relay.py`
- `hw_insight/hw_insight/test_planner_feedback.py`

说明：

- `ego_planner_integration.launch.py` 仅启动 `ego_planner_node`（核心规划）及必要 relay。
- 参数已按 AirSim 场景提供初值（`max_vel/max_acc/planning_horizon`、局部栅格尺寸等）。

## 4. 编译过程关键修复日志

### 4.1 编译结果

执行：

```bash
colcon build --packages-select quadrotor_msgs traj_utils plan_env path_searching bspline_opt ego_planner hw_insight
```

结果：

- 7 个包编译通过（含核心规划链与 `hw_insight`）
- 主要为 warning（弃用 API、签名比较、VLA 等），无阻断错误

### 4.2 运行时崩溃修复

发现并修复两处导致规划触发后崩溃/异常的问题：

1. **执行器冲突**
   - 现象：`Node '/ego_planner_node' has already been added to an executor`
   - 修复：移除回调内嵌 `spin_some(node_)` 的阻塞等待，改为非阻塞 FSM 状态切换。
   - 文件：`external/ego_planner_core/planner/ego_planner/src/ego_replan_fsm.cpp`

2. **odom 未就绪时目标触发**
   - 现象：目标过早触发导致 FSM 进入异常重规划路径。
   - 修复：`waypointCallback` 增加 `have_odom_` 守卫，未就绪时忽略目标并告警。
   - 文件：`external/ego_planner_core/planner/ego_planner/src/ego_replan_fsm.cpp`

### 4.3 DDS 传输兼容

- 在当前环境建议使用：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

避免 SHM 共享内存权限问题影响可观测性。

## 5. 验收测试结果

### 5.1 静态检查（已通过）

- `ros2 node list` 可见：
  - `/ego_planner_node`
  - `/ego_bspline_to_twist_relay`
  - `/planner_velocity_bridge`
- `ros2 node info /ego_planner_node` 可见关键订阅/发布：
  - 订阅：`/uav/target_goal`, `/vins/odometry`, `/uav/camera/points`
  - 发布：`/uav/ego_planner/bspline`, `grid_map/*`, 可视化 marker

### 5.2 链路验证（已通过）

- 发布目标后，收到规划输出：
  - `ros2 topic echo --once /uav/ego_planner/bspline`
  - 已捕获非空 B-spline 数据（`knots` + `pos_pts`）。

### 5.3 闭环验收脚本（部分通过）

- `test_planner_feedback.py` 已可运行并输出结果。
- 当前“自动测延迟”流程仍受目标发布时间与 odom 就绪时序影响，存在偶发 `received_goal=False` 的 race。
- 建议作为 v1 已知待优化项（见下节）。

## 6. 已验收通过 / 待调优参数

### 已验收通过

- 核心包迁移与隔离工作区完成
- 核心规划链编译通过
- `ego_planner_node` 成功启动并消费目标
- 输出 `Bspline` 被 relay 到统一执行输入链

### 待调优参数

- 目标触发时序（确保 odom 就绪后再发首个 goal）
- `grid_map/*` 参数（分辨率、更新范围、膨胀半径）按 AirSim 场景做系统调参
- `manager/max_vel`, `manager/max_acc`, `optimization/*` 按机体动力学和安全约束收敛
- 验收脚本延迟统计可增加“主动发布目标并记录时间戳”的单进程模式以消除 race
