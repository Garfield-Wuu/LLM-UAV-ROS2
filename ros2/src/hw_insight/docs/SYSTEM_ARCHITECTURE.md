# 系统架构（hw_insight）

> **定位**：基于大语言模型的低空经济无人机自主决策系统（仿真验证平台）  
> **论文主线**：自然语言任务 → JSON 任务原语 → PX4/AirSim 物理飞行行为的可靠闭环转换  
> **位姿方案**：**不接入 VINS-Fusion**；统一使用 **AirSim / PX4 里程计** + **NED↔ENU 桥接**  
> **运维入口**：日常启动见 [`PRD_text_command_flight_mvp.md`](../PRD_text_command_flight_mvp.md) §9；动作协议见 [`COMMAND_PROTOCOL.md`](../COMMAND_PROTOCOL.md)

---

## 1. 研究背景

面向低空经济中的物流、巡检与低空任务执行，系统探索「**自然语言 → 结构化任务原语 → PX4/AirSim 飞行行为**」的可靠闭环转换。论文重构后不再以展示完整无人机系统栈为主线，而是聚焦开源 LLM 如何通过 ROS 2 agent 稳定接入 PX4/AirSim 闭环；姿态控制、导航稳定与安全保护由 **PX4** 负责，视觉与规划仅作为支撑能力出现。

核心研究问题：如何将开源大语言模型可靠地集成到基于 ROS 2 与 PX4 的无人机闭环系统中，实现从自然语言任务到物理飞行行为的稳定转换？

---

## 2. 技术栈总览

| 层级 | 技术选型 | 状态 |
|------|----------|------|
| 仿真环境 | AirSim + Unreal Engine（WSL2） | ✅ |
| 飞控 | PX4 Autopilot SITL | ✅ |
| 中间件 | ROS 2 Humble | ✅ |
| 飞控桥接 | Micro-XRCE-DDS + `px4_msgs` | ✅（主链） |
| 任务理解 | Groq API / Ollama（意图槽位 + 动作编译） | ✅ |
| 视觉感知支撑 | YOLO-World v2-S（开放词汇，`on_query`） | ✅（非核心贡献） |
| 视觉定位支撑 | RGB + DepthPlanar + 相机内参逆投影 | ✅（非论文主体） |
| **位姿与坐标** | **AirSim `odom_local_ned` → `odom_ned_to_enu_node` → ENU `world`** | ✅ |
| 局部规划 | EGO-Planner（可选，`uav_sim.launch.py`） | ⚠️ 仿真已联调 |
| 执行 | `move_velocity` → PX4 Offboard | ✅ |
| 监控 | TUI、`/uav/llm_task_status`、QGroundControl | ✅ |

**明确不纳入论文主体 / 技术路线**：VINS-Fusion、真实无人机实验、QGroundControl 实验监控、将 YOLO/EGO 作为核心贡献或主要评价对象。Micro-XRCE-DDS 仅作为 PX4 与 ROS 2 的底层通信背景，不作为方法贡献展开。

---

## 3. 五层架构

```
┌─────────────────────────────────────────────────────────────────┐
│ L1  交互与任务理解                                               │
│     llm_client（Groq/Ollama · intent 槽位 · 规则快路径）          │
│     → consistency_guard · command_compiler · safety_validator    │
│     → /uav/user_command（JSON action / plan）                    │
├─────────────────────────────────────────────────────────────────┤
│ L2  语义感知（可选叠加 semantic_perception.launch.py）            │
│     /uav/target_query → yolo_world_detector → detections_2d      │
│     → target_grounding_node → semantic_target_tf_node            │
│     → /uav/semantic_targets_world（ENU）                         │
├─────────────────────────────────────────────────────────────────┤
│ L3  位姿与坐标服务（仿真/真机统一：PX4 系里程计，无 VINS）          │
│     /airsim_node/PX4/odom_local_ned（NED）                       │
│     → odom_ned_to_enu_node → /uav/odom_enu + TF world↔base_link  │
│     视觉/规划/执行层仅在桥接处做 NED↔ENU 转换                      │
├─────────────────────────────────────────────────────────────────┤
│ L4  任务执行与规划                                               │
│     text_command_bridge（状态机 · FIND_AND_GOTO · GOTO_NED）      │
│     可选：EGO-Planner ← /uav/odom_enu + 点云 → planner_velocity  │
│     → /hw_insight/keyboard_velocity                              │
├─────────────────────────────────────────────────────────────────┤
│ L5  飞控执行                                                     │
│     move_velocity（Offboard · 安全闸门）→ PX4 SITL → AirSim       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. 两条并行数据流

### 4.1 主链：语言飞控（必启）

```text
自然语言 / JSON
  → llm_client
  → text_command_bridge（MOVE_REL / GOTO_NED / TAKEOFF …）
  → move_velocity
  → PX4 Offboard
  → AirSim
```

### 4.2 视觉链：语言找目标（须另启 semantic_perception）

```text
自然语言「前往红色汽车」
  → llm_client → FIND_AND_GOTO { query: "red,car" }
  → text_command_bridge 发布 /uav/target_query
  → YOLO-World（on_query，单帧推理）
  → 视觉定位 + TF（依赖 odom_local_ned，非 VINS）
  → bridge 取最高置信度目标 → ENU→NED → GOTO_NED
  → （可选）planner_mode 时发 /uav/target_goal → EGO-Planner
```

**编排原则**：当前为「**LLM 先定找什么，再触发 YOLO**」；未实现「先全图检测再把结果塞进 LLM prompt」。

---

## 5. 坐标系约定

| 坐标系 | 用途 | 来源 |
|--------|------|------|
| NED | PX4 Offboard、机体相对移动、`GOTO_NED` | AirSim `odom_local_ned` |
| ENU `world` | 语义目标、EGO-Planner、RViz | `odom_ned_to_enu_node` 输出 |

所有跨层接口禁止混用轴向；转换集中在 `text_command_bridge`、`semantic_target_tf_node`、`planner_velocity_bridge`。

---

## 6. 安全与职责边界

- **LLM**：只输出白名单 action / intent 槽位，不输出 PID、原始 MAVLink、姿态角。
- **安全层**：`safety_validator`、`consistency_guard`、`visual_query` 简化；失败 → `HOVER` / 拒绝 / 追问。
- **飞控层**：`flight_enabled` 闸门；`RTL` / `EMERGENCY_STOP` 最高优先级。
- **FIND_AND_GOTO**：搜索超时（默认 20s）回退悬停；`on_query` 避免无指令 GPU 空转。

---

## 7. 阶段与后续（无 VINS）

| 阶段 | 内容 | 状态 |
|------|------|------|
| Phase 0 | 自然语言/JSON → PX4 最小闭环 | ✅ |
| Phase 1 | LLM 工程化（双后端、JSON 容错、TUI） | ✅ |
| Phase 2 | 视觉语义 + FIND_AND_GOTO | ✅ 代码闭环，场景验收中 |
| Phase 3 | EGO-Planner 与语义任务深度联动、多目标策略 | 🟡 进行中 |
| Phase 4 | 真机 PX4 + 外参标定（仍用飞控/仿真里程计） | 📋 可选 |

**不在路线内**：VINS-Fusion 接入、真实无人机实验、先视觉后进 LLM 的全自动编排（除非单独立项）。

---

## 8. 论文重构口径

论文正文建议围绕两个研究子问题组织：

- **RQ1：结构化闭环转换机制**。说明 ROS 2 agent 如何把自然语言任务转换为 JSON 任务原语，并映射到 PX4/AirSim 飞行行为。
- **RQ2：开源 LLM 转换可靠性评估**。比较 Ollama 开源模型在 AirSim 导航任务中的 JSON 指令生成成功率、任务原语识别准确率和任务完成率。

写作边界：

- 保留 Ollama、ROS 2 agent、JSON 鲁棒提取、参数校验、任务原语、PX4 Offboard、AirSim 仿真。
- YOLO-World 与 EGO-Planner 只作为支撑能力简述，不做算法推导和主要评价。
- 删除或弱化几何定位、VINS-Fusion、深度逆投影等与论文核心问题不直接相关的算法展开。
