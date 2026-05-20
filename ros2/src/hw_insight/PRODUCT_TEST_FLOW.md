# 开发测试流程与完成度矩阵

## 1. 文档范围

本文件用于验证当前代码库中**已经实现的能力**，并区分：

- 当前版本可验收能力
- 新 PRD 中已规划但尚未落地的能力

判定依据：

- `PRD_text_command_flight_mvp.md`（**§9** 主流程与 **T_YOLO**）
- `COMMAND_PROTOCOL.md`
- `README_text_command_test.md`
- `docs/yolo_world_airsim_online_test.md`（视觉链终端与 RViz）
- `docs/DOCUMENTATION_INDEX.md`（文档分工，避免重复维护）

## 2. 完成度矩阵

| 能力项 | 目标架构状态 | 当前实现状态 | 证据 |
|---|---|---|---|
| 自然语言转飞行动作 | 目标主链 | ✅ 已完成 | `llm_client.py` |
| Ollama 本地推理入口 | 目标主链 | ✅ 已具备 | `llm_client.py` 支持 `llm_provider:=ollama` |
| Groq 远端推理入口 | 辅助能力 | ✅ 已完成 | `llm_client.py` |
| 12 个动作协议（含 FIND_AND_GOTO）| 当前主链 | ✅ 已完成 | `text_command_bridge.py` / `llm_client.py` |
| 多步 `plan` 顺序执行 | 当前主链 | ✅ 已完成 | `llm_client.py` |
| PX4 Offboard 安全闸门 | 当前主链 | ✅ 已完成 | `move_velocity.py` |
| 状态反馈话题 `/uav/llm_task_status` | 当前主链 | ✅ 已完成 | `text_command_bridge.py` |
| TUI 监控 | 当前主链 | ✅ 已完成 | `gcs_dashboard.py` |
| 闭环回归测试 | 当前主链 | ✅ 已完成 | `flight_regression_runner.py` |
| YOLO-World 开放词汇检测（GPU）| 目标架构 | ✅ 已实现 | `yolo_world_detector.py`；torch cu128 / CUDA 12.9 已验证 |
| 深度图视觉定位支撑 | 支撑能力 | ✅ 已实现 | `target_grounding_node.py`；论文主体不作为核心贡献展开 |
| `/uav/target_goal` 目标发布 | 目标架构 | ✅ 已实现 | `semantic_target_tf_node.py` / `semantic_goal_to_planner.py` |
| LLM → 视觉搜索 → 自动飞行（FIND_AND_GOTO）| 目标架构 | ✅ 已实现 | `llm_client.py` + `text_command_bridge.py` S11 |
| AirSim/PX4 位姿与坐标桥接 | 当前主链 | ✅ 已实现 | `/airsim_node/PX4/odom_local_ned` → `/uav/odom_enu`；不接入 VINS |
| Ego-Planner 仿真避障链 | 目标架构 | ⚠️ 仿真部分接入 | `uav_sim.launch.py`（推荐）/ `planner_integration.launch.py` / `ego_planner_integration.launch.py`；`planner_velocity_bridge` ENU→NED |
| 统一仿真 launch（飞控 + 可选 EGO） | 当前主链 | ✅ 已具备 | `launch/uav_sim.launch.py`（`enable_ego_planner`、`use_rviz`） |
| MAVROS 2 桥接主链 | 目标架构 | ❌ 未实现 | 当前主链为 `px4_msgs + uXRCE-DDS` |

## 2.1 历史测试项留痕

以下能力虽然不是未来所有阶段的最终形态，但它们是项目演进中已经实现并验证过的重要基线，因此应继续保留在测试文档中：

| 历史测试项 | 当前状态 | 保留原因 | 后续关系 |
|------------|----------|----------|----------|
| 中文文本指令测试 | ⚠️ 兼容保留 | 适合快速人工验证桥接层 | 后续继续作为兼容层 smoke test |
| JSON 动作协议测试 | ✅ 长期保留 | 是动作层最稳定的分层测试入口 | 后续仍应作为 planner 以下基线测试 |
| Groq 推理链测试 | ⚠️ 保留 | 可用于和本地 Ollama 做效果与格式对照 | 后续作为 fallback / 对照测试 |
| Ollama 推理链测试 | ✅ 主测试项 | 是后续本地 LLM 主链的核心验证对象 | 将持续扩展 |
| 单步动作回归 | ✅ 长期保留 | 能快速定位桥接层、执行层问题 | 后续依旧必须保留 |
| 多步 `plan` 回归 | ✅ 长期保留 | 是任务编排能力的前置验证 | 后续可升级为 mission 级测试 |

原则上，新阶段测试文档应在旧阶段测试项基础上扩展，而不是直接移除旧阶段基线。

## 3. 当前标准运行拓扑

1. AirSim（Windows Unreal Engine）
2. PX4 SITL（`none_iris`）
3. XRCE Agent（`MicroXRCEAgent udp4 -p 8888`）
4. ROS 主链（**三选一，勿重复起 AirSim / move_velocity**）
   - 仅手飞/LLM：`text_command_test.launch.py`
   - **仿真 + 避障规划（推荐）**：`uav_sim.launch.py enable_ego_planner:=true`
   - 历史组合：`planner_integration.launch.py`
5. 语义感知链（可选叠加，`ros2 launch hw_insight semantic_perception.launch.py`）
6. LLM 终端（`ros2 run hw_insight llm_client ...`）
7. TUI（可选）
8. 回归 runner（按需执行）

## 4. 当前阶段测试分层

### Stage A：基础启动健康检查

- [ ] PX4 输出 `Simulator connected on TCP port 4560`
- [ ] PX4 输出 `Ready for takeoff!`
- [ ] XRCE Agent 输出 `session established`
- [ ] 主链日志出现 `Text command bridge ready`
- [ ] TUI 显示 `LIVE`

### Stage B：动作协议冒烟测试

- [ ] 手动发送一个 `TAKEOFF` JSON
- [ ] 观察状态流经过 `RECEIVED -> MAPPED -> PUBLISHED`
- [ ] 观察持续 `TELEMETRY`
- [ ] 飞机成功到达目标高度附近

### Stage C：自然语言链路测试

- [ ] 启动 `llm_client.py`
- [ ] 输入"起飞到 8 米"
- [ ] 输入"向前飞 10 米"
- [ ] 输入"降落"
- [ ] 确认自然语言被正确转换为动作 JSON

### Stage D：多步计划测试

- [ ] 输入"先上升 5 米，再向右飞 10 米，然后降落"
- [ ] 确认输出为 `plan`
- [ ] 每一步按顺序执行
- [ ] 上一步完成前不会进入下一步

### Stage E：安全行为测试

- [ ] 发送未知命令
- [ ] 观察 `UNKNOWN_COMMAND`
- [ ] 确认未触发危险运动
- [ ] 执行 `LAND` 或 `RTL` 后系统进入安全状态

### Stage F：回归序列测试

- [ ] 运行 `flight_regression_runner`
- [ ] 所有步骤均为 `PASS`
- [ ] 飞机最终处于落地 / 安全状态

### Stage G：语义感知链冒烟测试

- [ ] 启动 `semantic_perception.launch.py`（默认 **`inference_mode=on_query`**：启动后**不会**持续跑 GPU；需发 `/uav/target_query` 才推理）
- [ ] 另开终端：`ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"`（或 `"data: 'person'"`），日志出现推理与 JSON 输出
- [ ] `yolo_world_detector` 正常订阅 RGB，并向 `/uav/detections_2d` 发布 JSON（GPU 模式，日志无 CPU fallback）
- [ ] `target_grounding_node` 能基于深度图输出 `/uav/semantic_targets_camera`
- [ ] `semantic_target_tf_node` 能输出 `/uav/semantic_targets_world`
- [ ] RViz 中 `/uav/semantic_target_marker` 位置与目标大致一致
- [ ] 动态 prompt 可使用通用语义目标，如 `person`、`yellow clothes person`，而不是只测固定 `red car`

### Stage H：规划桥接联调测试

- [ ] `/uav/target_goal` 只有一个节点在发布，避免 `semantic_target_tf_node` 与 `semantic_goal_to_planner` 双写
- [ ] 在 planner 模式下，目标点更新后轨迹生成链路无报错
- [ ] LLM / 手动动作链与 planner 模式不存在互相抢占
- [ ] 使用 `uav_sim.launch.py enable_ego_planner:=true` 时：先发 `TAKEOFF` JSON，确认 `nav_state` 为 Offboard（见 TELEMETRY），再设目标或 RViz 2D Goal
- [ ] RViz 设点后 `text_command_bridge` 日志出现 **`[PLANNER] 外部目标点到达`**（`planner_mode_for_goto` 下订阅 `/uav/target_goal`，避免 bridge hover 压住规划速度）
- [ ] `/uav/planner_cmd_vel_stamped` 与飞机运动方向一致（`planner_velocity_bridge` 已将 EGO **ENU** 线速度转为 PX4 **NED**）

### Stage I：FIND_AND_GOTO 端到端视觉任务测试

- [ ] 同时运行 `semantic_perception.launch.py` + 主链 + LLM 终端
- [ ] 在 LLM 终端输入视觉任务指令，如"飞到穿黄色衣服的人头顶"
- [ ] `llm_client` 输出 `{"action":"FIND_AND_GOTO","params":{"query":"person wearing yellow..."}}` 到 `/uav/user_command`
- [ ] `text_command_bridge` 日志显示 `[FIND_AND_GOTO] Searching for "..."` 并发布 `/uav/target_query`
- [ ] TELEMETRY 中 `command` 字段为 `SEARCHING`，并携带 `searching_query` 和 `search_remaining_sec`
- [ ] YOLO-World 检测到目标，`/uav/detections_2d` 有置信度 ≥ 0.15 的结果
- [ ] `/uav/semantic_targets_world` 发布有效 world frame 坐标
- [ ] bridge 收到后状态事件 `SEARCH_FOUND` 出现，自动转为 `GOTO_NED`
- [ ] 无人机飞向目标 XY 位置（当前高度不变）
- [ ] 若 20s 内未找到目标，状态变 `SEARCH_FAILED` 并回落 HOVER

## 5. 当前版本 Release Gate

仅当以下条件全部满足时，才可将"文本飞控基线"标记为 **GO**：

- Stage A-F 全部通过
- TUI 无持续 `STALE / NO_LINK`
- `llm_client.py`、`text_command_bridge.py`、`move_velocity.py`、`gcs_dashboard.py` 无崩溃
- 回归 runner 总结果为 `PASS`
- launch 后无人机不会自动解锁

若要将"语义感知链联调版本"标记为 **GO**，还应额外满足：

- Stage G 全部通过
- Stage I 中 FIND_AND_GOTO 基本链路（LLM → 检测 → GOTO_NED）可演示
- 若接入 planner，则 Stage H 全部通过
- 目标点 topic 不出现明显跳变、卡死或双发布竞争
- GPU 推理路径已验证（`torch.cuda.is_available() = True`，cu128 匹配驱动 CUDA 12.9）

## 6. 非当前阶段测试项

以下测试项属于当前版本之后的后续阶段，或属于论文核心问题之外的系统支撑验证，不应纳入本轮发布门槛：

- 复杂属性 prompt 的泛化精度系统验证（不同场景、目标遮挡、距离变化下的稳定性）
- AirSim/PX4 odom 下 world frame 一致性与坐标跳变评估
- Ego-Planner 在复杂动态障碍下的系统级验收（当前仿真已可演示局部避障；FIND_AND_GOTO 仍多为直线 GOTO_NED，与 planner 深度联动待产品化）
- 多目标同时出现时 FIND_AND_GOTO 的目标选择策略优化

说明：

- 上述项目虽然暂不纳入本阶段发布门槛，但未来接入时应新增独立测试章节。
- 不应因为进入下一阶段，就删除当前文档中已有的动作层、安全层、LLM 层回归测试。

## 7. 常用命令

```bash
# 主链（无 EGO-Planner）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py

# 仿真 + 可选 EGO-Planner（与上一命令二选一）
ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true use_rviz:=true

# 本地 LLM
ros2 run hw_insight llm_client \
  --ros-args -p llm_provider:=ollama -p ollama_model:=llama3.2

# TUI
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0

# 一键回归
ros2 run hw_insight flight_regression_runner

# 语义感知链（GPU 推理，需与主链同时运行；默认 on_query）
ros2 launch hw_insight semantic_perception.launch.py
# 触发一次检测（与 PRD §9、yolo_world_airsim_online_test.md 一致）
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"
```

## 8. 下一阶段建议测试入口

当前已进入视觉层并打通 FIND_AND_GOTO 闭环，下一阶段建议把测试继续细化为：

1. FIND_AND_GOTO 在场景中有多目标时的选择行为验证
2. 目标运动时 GOTO_NED 的跟踪响应验证（当前为单次触发）
3. 深度图读数有效性与时间戳对齐验证
4. `/uav/semantic_targets_world` 稳定性验证（无人机运动时 world 点是否跳变）
5. EGO-Planner 与 FIND_AND_GOTO 联动（语义发现目标后稳定走 `/uav/target_goal` + planner 模式，避免与直线 GOTO 语义混淆）
6. 论文主实验回归：JSON 指令生成成功率、任务原语识别准确率、任务完成率
