# 开发测试流程与完成度矩阵

## 1. 文档范围

本文件用于验证当前代码库中**已经实现的能力**，并区分：

- 当前版本可验收能力
- 新 PRD 中已规划但尚未落地的能力

判定依据：

- `PRD_text_command_flight_mvp.md`
- `COMMAND_PROTOCOL.md`
- `README_text_command_test.md`

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
| 深度图 3D Grounding | 目标架构 | ✅ 已实现 | `target_grounding_node.py` |
| `/uav/target_goal` 目标发布 | 目标架构 | ✅ 已实现 | `semantic_target_tf_node.py` / `semantic_goal_to_planner.py` |
| LLM → 视觉搜索 → 自动飞行（FIND_AND_GOTO）| 目标架构 | ✅ 已实现 | `llm_client.py` + `text_command_bridge.py` S11 |
| VINS 状态估计 | 目标架构 | 📋 待接入 | 当前 world pose 先使用 AirSim `odom_local_ned` |
| Ego / Fast Planner | 目标架构 | ⚠️ 仿真部分接入 | `planner_integration.launch.py` / `ego_planner_integration.launch.py` |
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
4. ROS 主链（`ros2 launch hw_insight text_command_test.launch.py`）
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

- [ ] 启动 `semantic_perception.launch.py`
- [ ] `yolo_world_detector` 正常订阅 RGB，并向 `/uav/detections_2d` 发布 JSON（GPU 模式，日志无 CPU fallback）
- [ ] `target_grounding_node` 能基于深度图输出 `/uav/semantic_targets_camera`
- [ ] `semantic_target_tf_node` 能输出 `/uav/semantic_targets_world`
- [ ] RViz 中 `/uav/semantic_target_marker` 位置与目标大致一致
- [ ] 动态 prompt 可使用通用语义目标，如 `person`、`yellow clothes person`，而不是只测固定 `red car`

### Stage H：规划桥接联调测试

- [ ] `/uav/target_goal` 只有一个节点在发布，避免 `semantic_target_tf_node` 与 `semantic_goal_to_planner` 双写
- [ ] 在 planner 模式下，目标点更新后轨迹生成链路无报错
- [ ] LLM / 手动动作链与 planner 模式不存在互相抢占

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

以下测试项属于当前版本之后的后续阶段，不应纳入本轮发布门槛：

- VINS-Fusion 位姿接入后的稳定性验证
- 复杂属性 prompt 的泛化精度系统验证（不同场景、目标遮挡、距离变化下的稳定性）
- VINS 漂移评估
- Ego-Planner / Fast-Planner 动态避障验证（当前 FIND_AND_GOTO 为直线 GOTO_NED，未走 planner 避障）
- 多目标同时出现时 FIND_AND_GOTO 的目标选择策略优化

说明：

- 上述项目虽然暂不纳入本阶段发布门槛，但未来接入时应新增独立测试章节。
- 不应因为进入下一阶段，就删除当前文档中已有的动作层、安全层、LLM 层回归测试。

## 7. 常用命令

```bash
# 主链
cd /home/hw/hw-ros2/ros2
source install/setup.bash
ros2 launch hw_insight text_command_test.launch.py

# 本地 LLM
ros2 run hw_insight llm_client \
  --ros-args -p llm_provider:=ollama -p ollama_model:=llama3.2

# TUI
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0

# 一键回归
ros2 run hw_insight flight_regression_runner

# 语义感知链（GPU 推理，需与主链同时运行）
ros2 launch hw_insight semantic_perception.launch.py
```

## 8. 下一阶段建议测试入口

当前已进入视觉层并打通 FIND_AND_GOTO 闭环，下一阶段建议把测试继续细化为：

1. FIND_AND_GOTO 在场景中有多目标时的选择行为验证
2. 目标运动时 GOTO_NED 的跟踪响应验证（当前为单次触发）
3. 深度图读数有效性与时间戳对齐验证
4. `/uav/semantic_targets_world` 稳定性验证（无人机运动时 world 点是否跳变）
5. EGO-Planner 与 FIND_AND_GOTO 联动（将 GOTO_NED 替换为 planner 路径规划）
6. VINS-Fusion 接入后 world frame 精度提升验证
