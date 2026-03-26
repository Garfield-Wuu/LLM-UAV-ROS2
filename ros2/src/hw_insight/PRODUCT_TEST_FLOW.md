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
| 11 个动作协议 | 当前主链 | ✅ 已完成 | `text_command_bridge.py` |
| 多步 `plan` 顺序执行 | 当前主链 | ✅ 已完成 | `llm_client.py` |
| PX4 Offboard 安全闸门 | 当前主链 | ✅ 已完成 | `move_velocity.py` |
| 状态反馈话题 `/uav/llm_task_status` | 当前主链 | ✅ 已完成 | `text_command_bridge.py` |
| TUI 监控 | 当前主链 | ✅ 已完成 | `gcs_dashboard.py` |
| 闭环回归测试 | 当前主链 | ✅ 已完成 | `flight_regression_runner.py` |
| YOLOv8 检测 | 目标架构 | ❌ 未实现 | 当前仓库无相关节点 |
| 深度图 3D 对齐 | 目标架构 | ❌ 未实现 | 当前仓库无 `vision_grounding_node` |
| `/uav/target_goal` 目标发布 | 目标架构 | ❌ 未实现 | 当前仓库无稳定接口 |
| VINS 状态估计 | 目标架构 | ❌ 未实现 | 当前仓库无相关节点 |
| Ego / Fast Planner | 目标架构 | ❌ 未实现 | 当前仓库无相关节点 |
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
5. LLM 终端（`ros2 run hw_insight llm_client ...`）
6. TUI（可选）
7. 回归 runner（按需执行）

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
- [ ] 输入“起飞到 8 米”
- [ ] 输入“向前飞 10 米”
- [ ] 输入“降落”
- [ ] 确认自然语言被正确转换为动作 JSON

### Stage D：多步计划测试

- [ ] 输入“先上升 5 米，再向右飞 10 米，然后降落”
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

## 5. 当前版本 Release Gate

仅当以下条件全部满足时，才可将当前阶段标记为 **GO**：

- Stage A-F 全部通过
- TUI 无持续 `STALE / NO_LINK`
- `llm_client.py`、`text_command_bridge.py`、`move_velocity.py`、`gcs_dashboard.py` 无崩溃
- 回归 runner 总结果为 `PASS`
- launch 后无人机不会自动解锁

## 6. 非当前阶段测试项

以下测试项属于新 PRD 的后续阶段，不应纳入本阶段发布门槛：

- YOLOv8 检测精度验证
- RGB + Depth 对齐精度验证
- `/uav/target_goal` 跟踪稳定性
- VINS 漂移评估
- Ego-Planner / Fast-Planner 动态避障验证
- 基于目标类别与属性的自主搜索 / 跟踪

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
```

## 8. 下一阶段建议测试入口

当开始做视觉层时，建议新增单独测试阶段：

1. 图像话题连通性验证
2. 深度图读数有效性验证
3. YOLO 检测结果可视化验证
4. 2D 到 3D 投影误差验证
5. `/uav/target_goal` 发布一致性验证

在这些通过前，不建议直接接入 Planner。
