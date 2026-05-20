# 开发 PRD：基于本地 LLM 与视觉对齐的无人机自主决策系统

## 1. 文档信息

| 字段 | 内容 |
|------|------|
| 文档版本 | v5.6 |
| 原始版本 | v1.0（2026-03-21） |
| 本次更新 | 2026-05-19：§4.2 增补 **语义感知链 + FIND_AND_GOTO** 端到端数据流；§6.1 与 Phase 2 表述与代码对齐（**已实现** LLM→`FIND_AND_GOTO`→`/uav/target_query`→YOLO→规划；**未实现**「先视觉摘要再进 LLM prompt」）；明确 **`planner_integration` 不含 YOLO**，须叠加 `semantic_perception.launch.py` |
| 上一版摘要 | v5.5：§14、T_YOLO、`on_query` 默认、RViz 叠图与文档索引 |
| 适用项目 | `hw_insight`（PX4 + AirSim + ROS 2 Humble） |
| 代码仓库 | `git@github.com:Garfield-Wuu/LLM-UAV-ROS2.git`（branch: main） |
| 当前阶段 | Phase 0/1 已完成；**Phase 2 进行中**：语义感知链已落地；**AirSim + EGO-Planner 仿真避障与执行链已可联调**（`uav_sim.launch.py`、ENU/NED 桥、RViz）；位姿统一为 **AirSim/PX4 里程计**（**不接入 VINS-Fusion**）；当前重点为视觉任务协议深化与规划链联动 |
| 文档定位 | 同时描述目标架构、当前实现和阶段化开发路线；**日常命令以 §9 为准**。维护边界见 [`docs/DOCUMENTATION_INDEX.md`](docs/DOCUMENTATION_INDEX.md)。 |

---

## 2. 项目定位

本系统面向“低空经济”场景，目标是构建一个具备自然语言理解、目标感知对齐、局部避障规划和 PX4 闭环执行能力的无人机自主决策系统。

系统的最终形态是：

1. 使用本地部署 LLM（优先 Ollama）完成语义理解和任务原语生成。
2. 使用 RGB + Depth 视觉链路完成目标识别、属性过滤和 3D 空间定位。
3. 使用高频状态估计与局部规划实现复杂环境下的安全导航。
4. 使用 PX4 Offboard 执行轨迹或速度控制，并保留独立安全闸门。

需要特别说明的是：**当前代码库已经完成的是“自然语言/结构化指令 -> 安全执行 -> 飞行反馈”的最小闭环，不等于整个目标架构已经全部落地。**

---

## 3. 目标技术栈与当前落地状态

| 层级 | 目标技术方案 | 当前状态 | 说明 |
|------|--------------|----------|------|
| 大脑（Cognition） | Ollama（Llama 3 / Gemma） | ⚠️ 部分完成 | `llm_client.py` 已支持 Groq / Ollama 双后端、交互式模型选择、远端 Ollama 环境变量、冷启动预热、鲁棒 JSON 提取与 `<think>` 推理日志；**S11 新增**：`FIND_AND_GOTO` 视觉语义搜索 action（12 个 action，含 8 个别名），LLM 可直接输出视觉任务指令 |
| 视觉（Perception） | **YOLO-World** + AirSim Depth Camera | ✅ 核心链路已实现 | YOLO-World 已在 `/home/hw/YOLO-World/` 本地部署；ROS 2 侧 **4 节点链**（检测 / grounding / TF / 可选 goal）；**`yolo_world_detector`** 支持 **`inference_mode=on_query`**（按 `/uav/target_query` 单次推理）；**`semantic_perception.launch.py` 默认 `on_query` + `publish_target_goal=false`**，避免联调时误发规划目标；**`detections_image_overlay`** 随 **`ego_planner_integration`** 在 RViz 叠画 bbox，并可叠加 **`depth_m`**（订阅 `/uav/semantic_targets_camera`）；联调手册见 **`hw_insight/docs/yolo_world_airsim_online_test.md`**；`FIND_AND_GOTO` 仍经 `text_command_bridge` → `/uav/target_query` → 视觉 → `GOTO_NED` |
| 位姿与坐标 | **AirSim/PX4 里程计** + `odom_ned_to_enu_node` | ✅ 已实现 | NED：`/airsim_node/PX4/odom_local_ned`；ENU：`/uav/odom_enu` + TF `world`→`base_link`；**不接入 VINS-Fusion** |
| 规划（Planning） | **EGO-Planner**（已选定） | ⚠️ 部分接入 | EGO-Planner（ROS 2）已接入仿真最小闭环；可用 `uav_sim.launch.py enable_ego_planner:=true` 与飞控链一并启动；`planner_velocity_bridge` 将规划器 **ENU world** 线速度转换为 **PX4 NED** 再送入 `move_velocity`；位姿输入为 `/uav/odom_enu`（由 AirSim NED odom 桥接） |
| 通信与框架 | ROS 2 Humble + MAVROS 2 | ⚠️ 部分完成 | ROS 2 Humble 已落地；当前飞控通信主链实际使用 `px4_msgs + uXRCE-DDS`，尚未接入 MAVROS 2 |
| 执行与仿真 | PX4 Offboard + AirSim | ✅ 已实现 | 已形成稳定可复现的最小闭环 |
| 可观测性 | TUI + 状态回传 + 回归测试 | ✅ 已实现 | 已有地面站、状态流、回归 runner |

---

## 4. 分层架构定义

### 4.1 目标架构（最终形态，无 VINS）

> 架构详图见 [`docs/SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md)。

```
用户自然语言指令
    │
    ▼
llm_client.py（任务理解层）
    │  intent 槽位 / JSON action + 安全校验
    ▼
text_command_bridge.py（任务执行层）
    ├── 飞行动作：MOVE_REL / GOTO_NED / FIND_AND_GOTO …
    └── FIND_AND_GOTO → /uav/target_query
            │
            ▼
yolo_world_detector + target_grounding + semantic_target_tf（可选语义链）
    ├── YOLO-World（RGB + 文本 prompt）
    ├── DepthPlanar 逆投影 → camera 3D
    └── 依赖 AirSim odom（NED）→ ENU world 目标点
            │
            ▼
EGO-Planner（可选，输入 /uav/odom_enu + 点云）
    │  B-spline → planner_velocity_bridge（ENU→NED）
    ▼
move_velocity.py + PX4 SITL（控制执行层）
    │
    ▼
AirSim / 真机（真机仍用 PX4 里程计 + 外参，不依赖 VINS）
```

### 4.2 当前已实现架构（代码现状）

```
用户自然语言（stdin / 话题）
    │
    ▼
llm_client.py
    ├── Groq / Ollama 推理（交互式选择 provider / model）
    ├── Prompt 注入状态、安全约束、动作白名单
    ├── 冷启动预热、离线遥测拦截、`<think>` 日志留存
    ├── 鲁棒 JSON 提取（代码围栏 / `<think>` / 平衡括号 / 正则兜底）
    ├── 输出单步 action 或多步 plan
    └── 发布到 /uav/user_command
            │
            ▼
text_command_bridge.py
    ├── 中文文本 / JSON 解析
    ├── 11 个动作映射
    ├── 机体坐标 -> NED 变换
    ├── TAKEOFF / GOTO_NED / ORBIT / YAW_TO 控制逻辑
    └── 发布 /uav/llm_task_status
            │
            ▼
move_velocity.py
    ├── flight_enabled 安全闸门
    ├── Offboard 使能与解锁
    └── 向 PX4 发布 setpoint / vehicle_command
            │
            ▼
PX4 SITL + AirSim
```

#### 4.2.1 语义感知链与 `FIND_AND_GOTO`（可选叠加，与主链同一 ROS 图）

`planner_integration.launch.py` / `uav_sim.launch.py` **不**启动 YOLO；须**另开终端**运行 `semantic_perception.launch.py`（或仅 2D 的 `yolo_world_test.launch.py`）。`ego_planner_integration` 内默认带 **`detections_image_overlay`**，RViz **YOLOOverlay** 订阅 `/uav/camera/detections_overlay`（无检测节点时面板无框属正常）。

**已实现：LLM 先决策「去找什么」，再触发视觉（与「先检测再给 LLM」不同）**

```text
用户自然语言（如「前往红色的汽车」）
  → llm_client → LLM 输出 FIND_AND_GOTO { "query": "red car" }（或多步 plan 中含该步）
  → /uav/user_command
  → text_command_bridge 解析 FIND_AND_GOTO
  → 发布 /uav/target_query = "red car"（及 SEARCHING 等 /uav/llm_task_status）
  → yolo_world_detector（默认 inference_mode=on_query：每条 query 对当前缓存 RGB 推理一次）
  → /uav/detections_2d（JSON）
  → target_grounding_node → /uav/semantic_targets_camera（dets=0 时 grounding 不发布 camera/world）
  → semantic_target_tf_node → /uav/semantic_targets_world（ENU）
  → bridge 取最高置信度目标 → ENU→NED → GOTO_NED；planner_mode_for_goto 时向 /uav/target_goal 发点
  → EGO-Planner → planner_velocity_bridge → move_velocity → PX4
```

**未实现（产品化可选项）**：用户一发话就先跑 YOLO，把 `label/score/bbox/depth/world` 拼成自然语言或结构化块 **再** 调 LLM 出 `TAKEOFF`/`GOTO_NED` 等——当前 `llm_client` **不**订阅 `/uav/semantic_targets_world` 做 prompt 注入。

**Launch 修改提醒**：`ros2 launch hw_insight …` 使用 **`install/share`** 下已安装的 launch；只改 `src` 后须 `colcon build --packages-select hw_insight && source install/setup.bash`。

结论：当前系统已具备语义理解层、动作执行层与基础可观测层，并在仿真侧接入了规划链路；**在叠加语义感知链时**，`FIND_AND_GOTO` 经 `/uav/target_query` 的端到端闭环已接通（场景与阈值联调仍属验收范畴）。位姿统一为 **AirSim/PX4 里程计**（**不接入 VINS-Fusion**）。**「先视觉后进 LLM」**编排与真机级规划闭环仍为可选后续项。

---

## 5. 当前已经实现的能力清单

### 5.1 LLM 语义理解与任务输出

已实现：

- `llm_client.py` 支持 `Groq` 与 `Ollama` 双后端。
- 交互式终端启动时可用方向键选择 `Groq` / `Ollama` 与具体模型，Groq 支持自定义模型名。
- 支持交互式终端输入和 ROS 话题输入。
- 支持自然语言转结构化 JSON 动作。
- 支持多步 `plan` 顺序执行。
- 支持基础安全过滤：未知动作降级、参数裁剪、地面自动补 TAKEOFF。
- 支持 Ollama 冷启动预热：首次启动先发测试请求，待模型加载完成后再开放用户输入。
- 支持离线遥测合理性判断：若长时间未收到 `/uav/llm_task_status` 中的 TELEMETRY，则默认阻断普通指令。
- 支持动态获取远端 Ollama 已安装模型（`/api/tags`）；失败时自动回退到预设模型列表。
- 支持 `<think>...</think>` 推理保留：终端显示摘要，完整内容写入 ROS2 log。
- 支持更鲁棒的 JSON 解析：动作别名库、平衡括号提取、正则兜底、参数 schema 强制转换。

当前限制：

- 还没有独立的 `llm_orchestrator_node` 命名和任务原语 schema。
- 当前任务原语仍以内嵌动作白名单 / 参数 schema 为主，尚未独立沉淀为单独的 `mission primitives` 文档或接口包。
- 还没有“跟踪红衣人”这类视觉绑定任务的稳定输出协议。
- 还没有对话历史记忆与任务上下文复用。

### 5.2 动作执行与飞行控制

已实现：

- `text_command_bridge.py` 已支持 11 个动作：
  `TAKEOFF`、`LAND`、`HOVER`、`MOVE_VELOCITY`、`MOVE_REL`、`GOTO_NED`、`ORBIT`、`YAW_TO`、`RTL`、`EMERGENCY_STOP`、`SET_SPEED`。
- `move_velocity.py` 已实现 PX4 Offboard 控制链路。
- 已修复自动解锁、偏航反向、机体系平移方向错误等关键问题。
- 多步动作可以按序等待完成后再进入下一步。

当前限制：

- 仍以速度控制 / 目标点控制为主，不是完整轨迹规划执行。
- 没有独立 mission manager 抢占与优先级调度。
- 尚未接入 MAVROS 2，当前桥接实现偏轻量。

### 5.3 观测、测试与操作员能力

已实现：

- `gcs_dashboard.py` 提供紧凑 TUI。
- `/uav/llm_task_status` 提供 `RECEIVED`、`MAPPED`、`PUBLISHED`、`TELEMETRY` 等状态流。
- `flight_regression_runner.py` 提供闭环回归测试。
- 已形成可执行的启动手册和回归操作流程。

当前限制：

- TUI 还没有 WARN / ERROR 分级和关键事件闪烁。
- 回归 runner 还没有导出 JSON / Markdown 报告。

### 5.4 仿真视觉联调与 RViz 叠图

- **手册**：`hw_insight/docs/yolo_world_airsim_online_test.md`——阶段 0～4、`semantic_perception` **默认** `inference_mode=on_query`、`publish_target_goal=false`、话题预检、与 `planner_integration` 并行、`YOLOOverlay` 与深度字端说明。
- **`detections_image_overlay`**：RGB + `/uav/detections_2d` → `/uav/camera/detections_overlay`；可选 `/uav/semantic_targets_camera` 在标签上叠加 **`d=…m`**（与 `depth_m` 一致）。随 **`ego_planner_integration`**（`enable_detection_overlay`，默认 true）启动；`planner_integration.launch.py` 向 ego **转发**该参数。
- **RViz**：`rviz/ego_planner_debug.rviz` 中 **YOLOOverlay** 订阅 `/uav/camera/detections_overlay`。

---

## 6. 未实现但已纳入目标架构的模块

以下模块中，视觉语义感知链已完成最小闭环；后续重点转为视觉任务协议与 EGO-Planner 深度联动（位姿沿用 AirSim/PX4 里程计，见 [`docs/SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md)）：

### 6.1 视觉语义识别与几何 Grounding 层

当前节点：`yolo_world_detector.py`、`target_grounding_node.py`、`semantic_target_tf_node.py`；**仿真 RViz 叠图**：`detections_image_overlay.py`（非感知链一环，仅可视化）。

核心技术选型：**YOLO-World**（open-vocabulary detection）

当前职责：

- 订阅 AirSim RGB 图像（默认 `/airsim_node/PX4/CameraDepth1/Scene`）与深度图（`DepthPlanar`）。
- **`yolo_world_detector`**：支持 **`continuous`** / **`on_query`**（`launch` 参数 `inference_mode`）；**`semantic_perception.launch.py` 默认 `on_query`**，避免无指令时 GPU 持续推理。
- 接收来自任务理解层或联调终端的文本 prompt（`/uav/target_query`），使用 YOLO-World 进行开放词汇目标检测，输出 bbox / 置信度。
- 对 bbox 区域提取深度信息，取 **中位数** 作为鲁棒深度估计，结合相机内参 `(fx, fy, cx, cy)` 完成逆投影，恢复 camera frame 3D 坐标。
- 通过 `camera frame → body frame → world frame` 完成坐标变换（位姿源：**AirSim `odom_local_ned`**，经 NED→ENU 桥接供规划与 RViz 使用）。
- 发布 `/uav/semantic_targets_world`，并可选发布 `/uav/target_goal`（`PoseStamped`）。

选型说明：YOLO-World 的 `prompt-then-detect` 范式将词汇嵌入重参数化进模型权重，支持任意类别文本描述，推理效率接近标准 YOLO，适合 `person`、`yellow clothes person`、`vehicle near building` 等开放表达目标。

当前状态：**YOLO-World 本地部署 + ROS 2 语义感知链均已实现**；联调步骤与 RViz 叠图见 **`hw_insight/docs/yolo_world_airsim_online_test.md`**。**`FIND_AND_GOTO`** 路径下 **`LLM → text_command_bridge → /uav/target_query → 视觉链 → GOTO_NED →（planner 模式）/uav/target_goal`** 已具备代码闭环，须在运行 **`planner_integration` / `uav_sim` 等主链的同时另启 `semantic_perception`**。待产品化/增强项：**检测结果先进 LLM prompt 再规划**、多目标/超时/抢占策略、开放词（如 `red car`）在各类场景下的稳定验收。

### 6.2 位姿与坐标服务层

目标节点：`odom_ned_to_enu_node`；位姿输入话题 `/airsim_node/PX4/odom_local_ned`

核心技术选型：**AirSim / PX4 里程计** + **NED↔ENU 集中桥接**（**不接入 VINS-Fusion**）

职责：

- 为几何 Grounding、EGO-Planner、RViz 提供统一 **ENU `world`** 帧与 **`/uav/odom_enu`**。
- 飞控执行层仍使用 **NED**；跨层转换仅在桥接节点与 `text_command_bridge` 内完成。
- 真机阶段沿用 **PX4 本地位置/里程计** + 外参标定，不引入 VINS 路线。

当前状态：**已实现**（仿真）；真机外参标定为可选后续。

### 6.3 局部规划层

目标节点：`ego_planner_node`

核心技术选型：**EGO-Planner**（ESDF-free、gradient-based local planner，面向四旋翼）

计划职责：

- 接收 `target_position_world`（来自 `semantic_target_tf_node.py` 或 `semantic_goal_to_planner.py` 发布的 `/uav/target_goal`）作为导航目标。
- 融合深度点云（AirSim DepthPlanar / 点云）生成局部无碰撞轨迹，持续重规划。
- 输出局部可飞轨迹（B-spline → Twist → `keyboard_velocity`）。
- 使用 `/uav/odom_enu`（由 `odom_local_ned` 桥接）驱动规划。

当前状态：**EGO-Planner 已在仿真侧最小接入**（`planner_integration.launch.py` / `uav_sim.launch.py`）；Fast-Planner 不作为候选。

---

## 7. 历史实现留痕与文件状态

本节用于保留前阶段已经做过的功能和文件演进痕迹。原则不是删除痕迹，而是明确标注：

- 是否仍为当前主链
- 是否已降级为兼容入口
- 是否已废弃
- 废弃或降级的原因
- 后续升级方向

### 7.1 历史阶段能力留痕

| 阶段 | 能力/成果 | 当前状态 | 说明 |
|------|-----------|----------|------|
| v1.0 | 文本指令飞行最小闭环 | ✅ 仍在使用 | 是当前系统的执行基座 |
| v1.0 | `/uav/llm_task_status` 状态回传 | ✅ 仍在使用 | 当前 TUI、LLM 上下文、回归测试都依赖它 |
| v1.0 | 中文文本命令输入 | ⚠️ 兼容保留 | 仍可用，但当前推荐 JSON 或 LLM 输出 |
| v1.0 | `flight_regression_runner.py` 回归链路 | ✅ 仍在使用 | 当前阶段稳定性验证主入口 |
| v2.0 | `flight_enabled` 安全闸门 | ✅ 仍在使用 | 仍是 launch 后不自动解锁的核心机制 |
| v2.0 | TAKEOFF ARM 阶段 | ✅ 仍在使用 | 当前已从 0.8s 升级为 2.5s |
| v2.0 | 紧凑 TUI | ✅ 仍在使用 | 后续只做告警分级增强，不推翻 |
| v3.0 | Groq LLM 联调链路 | ⚠️ 保留为对照/回退路径 | 当前建议日常开发优先 Ollama，本地模式优先 |
| v3.0 | Ollama 本地推理入口 | ✅ 仍在使用 | 已纳入后续主链方向 |
| v3.0 | 多步 `plan` 顺序执行 | ✅ 仍在使用 | 是后续 mission manager 的前置基础能力 |
| v3.0 | 机体坐标系平移修正 | ✅ 仍在使用 | 仍是 MOVE 系列动作的基础假设 |

### 7.2 历史文件状态矩阵

| 文件/入口 | 历史作用 | 当前状态 | 不再作为默认入口的原因 | 后续方向 |
|-----------|----------|----------|-------------------------|----------|
| `hw_insight/text_command_bridge.py` | 文本/JSON 到飞行动作桥接 | ✅ 主链文件 | 无 | 后续可演进为更清晰的 action bridge / mission adapter |
| `hw_insight/move_velocity.py` | PX4 Offboard 底层执行 | ✅ 主链文件 | 无 | 后续可与独立安全层、轨迹执行层解耦 |
| `hw_insight/gcs_dashboard.py` | 地面站 TUI | ✅ 主链文件 | 无 | 增加 WARN / ERROR 分级与闪烁提示 |
| `hw_insight/flight_regression_runner.py` | 自动化闭环回归 | ✅ 主链文件 | 无 | 增加 JSON / Markdown 报告输出 |
| `hw_insight/llm_client.py` | 自然语言到动作 JSON | ✅ 主链文件 | 无 | 后续可拆分为 `llm_orchestrator_node` + 上下文管理 / think 审计 / 任务原语管理 |
| `launch/text_command_test.launch.py` | 主飞控链启动入口 | ✅ 当前默认入口 | 无 | 后续按新架构拆分 launch 组合 |
| `launch/gcs_dashboard.launch.py` | TUI 独立启动 | ✅ 保留可用 | 非主入口，但仍有运维价值 | 后续可纳入统一 bringup |
| `launch/llm_flight.launch.py` | 早期 LLM 启动入口 | ❌ 已停用 | `ros2 launch` 捕获 stdin，交互式输入不可用 | 后续若做非交互式编排，可用新 launch 重新引入 |
| `keyboard_velocity.py` / `keyboard_position.py` | 早期人工控制 / 教学验证 | ⚠️ 保留留痕 | 当前已被更高层桥接链替代，不是主流程 | 可作为底层调试与教学工具保留 |
| `move_position.py` / `offboard.py` / `px4_test.py` | 早期 PX4 / ROS 实验脚本 | ⚠️ 保留留痕 | 已不承担产品主链职责 | 后续可整理到 examples / legacy |
| `lesson3.launch.py` / `lesson4.launch.py` / `lesson4_color.launch.py` | 深度/点云相关实验入口 | ⚠️ 保留留痕 | 不是当前主链，但对视觉阶段有参考价值 | 后续可复用到语义感知链调试 |

### 7.3 第七次会话新增（S7）

| 文件/入口 | 说明 |
|-----------|------|
| `/home/hw/hw-ros2/.gitignore` | Git 忽略规则（排除 build/install/log、第三方包、AirSim C++ 库） |
| `/home/hw/hw-ros2/README.md` | 项目中文说明文档（技术路线 + 完整工程结构 + 快速开始） |
| Git 仓库 `/home/hw/hw-ros2/` | 在 `hw-ros2/` 层初始化；SSH 方式推送到 `Garfield-Wuu/LLM-UAV-ROS2`；首次提交 129 个文件 |

### 7.4 第八次会话新增（S8）

| 文件/操作 | 说明 |
|-----------|------|
| `/home/hw/YOLO-World/test_yolo_world.py` | CPU/GPU 自动适配推理脚本；支持自定义词汇表；结果输出到 `outputs/` 目录 |
| `/home/hw/YOLO-World/weights/yolo_world_v2_s_stage1.pth` | YOLO-World-S 预训练权重（305MB，HuggingFace 下载） |
| `/home/hw/YOLO-World/clip_tokenizer/` | CLIP ViT-B/32 完整模型（tokenizer + pytorch_model.bin），离线化避免联网 |
| `mmengine/optim/optimizer/builder.py` patch | 修复 torch 2.11 与 mmengine 0.10.3 `Adafactor` 重复注册冲突 |
| `yolo_world/models/detectors/yolo_world.py` patch | 修复 `cannot assign to None` 语法错误（`None` → `_`） |
| 依赖安装 | `mmengine==0.10.3`、`mmcv-lite==2.0.1`、`mmdet==3.0.0`、`mmyolo==0.6.0`、`timm==0.6.13`、`transformers==4.36.2`、`supervision==0.19.0`、`lvis` |
| 推理验证 | bus.jpg：3人+1公交（score 0.70–0.81）；zidane.jpg：2领带+1人（score 0.27–0.72） |

### 7.5 文档留痕原则

后续更新文档时，针对历史功能和文件不应直接删除记录，而应优先采用以下标注方式：

1. `仍在使用`
2. `兼容保留`
3. `已停用`
4. `计划升级`

如果某入口不再推荐使用，需要同时说明：

- 为什么不再作为默认入口
- 当前替代路径是什么
- 未来是否会以新形式回归

---

## 8. 阶段化开发路线

### Phase 0：语义指令飞行 MVP

目标：打通自然语言到飞行动作的最小闭环。

已完成项：

- LLM 输入链路
- JSON 动作协议
- 11 个动作执行
- PX4 Offboard 安全闸门
- TUI 可观测性
- 回归测试

状态：✅ 已完成

### Phase 1：本地 LLM 主链切换

目标：将日常开发链路默认切到 Ollama / Groq 可切换模式，并提升交互与解析鲁棒性。

计划项：

- 固化 Ollama / Groq 双后端启动 SOP
- 对齐本地与远端模型 prompt
- 验证 `llama3.x` / `gemma` / `qwen` / `deepseek` 在动作输出格式上的稳定性
- 保留 Groq 作为对比和 fallback
- 增强交互终端：模型选择、冷启动预热、离线判断、推理日志与 JSON 提取鲁棒性

状态：🟡 已启动（工程化增强中）

### Phase 2：视觉语义识别 + 几何 Grounding + world frame 目标生成

目标：支持“飞向某物体 / 搜索某目标 / 跟踪某类目标”，实现从语言目标到 world frame 坐标的完整链路。

技术选型（已锁定）：
- 视觉语义识别：**YOLO-World**（open-vocabulary detection）
- 深度类型：**DepthPlanar**（须与逆投影公式匹配）
- 位姿对齐：**AirSim/PX4 里程计**（`odom_local_ned` + NED→ENU 桥接）

计划项：

- 已实现 `yolo_world_detector.py`、`target_grounding_node.py`、`semantic_target_tf_node.py`；**默认 launch 行为**（`semantic_perception`）为 **`on_query` + `publish_target_goal=false`**，便于与 `planner_integration` 安全并行联调；需要语义节点**直接**喂规划目标时可 `publish_target_goal:=true`（注意与 bridge 双写冲突）。
- 已接入 AirSim RGB + DepthPlanar + `camera_info`
- 已实现 YOLO-World 开放词汇检测、bbox 中位数深度、逆投影与 world 点生成
- 已实现 **RViz 检测叠图**（`detections_image_overlay` + `ego_planner_debug.rviz` / **YOLOOverlay**）
- **已接通**：`FIND_AND_GOTO` → `/uav/target_query` → 感知链 → `GOTO_NED` / `/uav/target_goal`（见 §4.2.1）；待强化：场景验收、**先视觉后进 LLM** 的编排、planner 与语义任务的抢占/互斥策略

状态：🟡 进行中（感知子链与 FIND_AND_GOTO 执行链已落地；下一步是场景级验收、以及「视觉摘要 → LLM」若需则单独开发）

### Phase 3：EGO-Planner 与语义任务深度联动

目标：在 **AirSim/PX4 里程计** 位姿基线上，将 Phase 2 的 `target_position_world` / `/uav/target_goal` 与 EGO-Planner、FIND_AND_GOTO 编排打通，形成可验收的避障飞行闭环。

技术选型（已锁定）：
- 位姿：**AirSim `odom_local_ned` → `odom_ned_to_enu_node`**
- 局部规划：**EGO-Planner**（ESDF-free，仿真已最小接入）

计划项：

- EGO-Planner 接收语义 `/uav/target_goal` → 局部轨迹 → PX4 执行
- FIND_AND_GOTO 与 planner 模式的抢占 / 互斥 / 超时策略
- 安全层：规划速度上限、地理围栏、丢目标 / 低电量策略

状态：🟡 已启动（EGO-Planner 仿真最小接入；语义—规划编排待产品化）

当前进展（2026-03 ~ 2026-03-26）：

- `text_command_bridge`：在 `GOTO_NED` 时发布 `/uav/target_goal`（`PoseStamped`，**frame_id=`world`**，NED→ENU 平移分量已对齐）；`planner_mode_for_goto:=true` 时 **订阅同一 `target_goal_topic`**：RViz「2D Goal Pose」等外部来源写入目标后自动将 `planner_control_active` 置真，避免 bridge 持续发 hover 覆盖规划速度。
- `planner_velocity_bridge`：订阅 planner `Twist` / `TwistStamped`（**EGO-Planner 为 ENU world 线速度**），在 `_store_cmd` 内转换为 **NED**（`keyboard.x = twist.y` 北、`keyboard.y = twist.x` 东、`keyboard.z = -twist.z` 下），限幅与超时后零速，发布到 `/hw_insight/keyboard_velocity`。
- `ego_bspline_to_twist_relay`：`traj_utils/Bspline` → `TwistStamped`（无有效 B-spline 时不发零速，避免干扰手飞）。
- `odom_ned_to_enu_node`：将 `odom_local_ned` 转为 ENU `Odometry` 供规划器；并广播 **`world` → `base_link`** TF，供 RViz 与栅格对齐。
- **`uav_sim.launch.py`（推荐仿真入口）**：组合 `text_command_test` 等价节点 + 可选 `ego_planner_integration`（`enable_ego_planner`）、静态 TF `world`→`PX4`；与 `text_command_test` + `ego_planner_integration` 双开互斥，**同一会话只起一个**。
- `planner_integration.launch.py`：与上类似的 AirSim + 飞控 + 规划组合（历史入口；新仿真联调优先 `uav_sim.launch.py`）。
- `ego_planner_integration.launch.py`：`ego_planner_node` 参数（栅格分辨率、深度范围、相机内参须与 **AirSim `settings.json` 一致**）、可选 RViz `ego_planner_debug.rviz`；**默认**启动 **`detections_image_overlay`**（`enable_detection_overlay`），与 **`planner_integration.launch.py`** 转发参数对齐。
- 核心包来源：`ros2/src/external/ego_planner_core/`（`ego_planner`、`plan_env`、`traj_utils` 等）；完整上游仓库以 `COLCON_IGNORE` 排除避免重复编译。
- 已修复项（摘要）：`ego_replan_fsm` 回调内 `spin_some` 导致 executor 冲突；无 odom 时忽略 goal；`grid_map` 深度投影缓冲区过小导致 segfault；NED 地图 Z 范围与 `ground_height` 不匹配导致起飞后栅格“消失”；WSL2 建议 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`；`depth_image_proc` 与 subscriber QoS 对齐；规划速度曾误按 x↔x 直通 PX4 导致「目标在前、飞机横飞」——已在 `planner_velocity_bridge` 按上式修正。

### Phase 4：任务编排与长期安全

目标：从动作级飞控升级到任务级自治。

计划项：

- mission manager
- 任务抢占与优先级
- 地理围栏 / 电量 / 丢目标安全策略
- 持续对话上下文

状态：🔲 未开始

---

## 9. 当前开发主流程（以现阶段代码为准）

### 9.1 启动拓扑

```bash
# T1 AirSim（Windows Unreal Engine 内点击 Play）

# T2 PX4 SITL
cd /home/hw/px4v1.15.2
make px4_sitl_default none_iris

# T3 XRCE Agent
MicroXRCEAgent udp4 -p 8888

# T4 ROS 主链（仅手飞 / LLM，无 EGO-Planner）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py

# T4' 仿真推荐：飞控 + AirSim + 可选 EGO-Planner（与 T4 二选一）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true use_rviz:=true

# T4'' 历史入口：规划接入（与 T4 / T4' 勿重复启动 airsim / move_velocity）
ros2 launch hw_insight planner_integration.launch.py use_rviz:=true

# T5 LLM 交互终端（推荐先将环境变量写入 ~/.bashrc）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# 可选：持久化 Groq / Ollama 配置后直接运行，启动时使用方向键选择后端与模型
#   export GROQ_API_KEY="gsk_..."
#   export ANTHROPIC_BASE_URL="http://<host>:<port>"
#   export ANTHROPIC_MODEL="qwen3-coder:30b"
ros2 run hw_insight llm_client

# T6 可选 TUI
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0
```

**T_YOLO：YOLO-World 检测（终端）**

与 T4～T6 **共用** `hw-ros2/ros2` 下的 `source` 与 `FASTDDS`。**前提**：AirSim 已 Play，且 **`airsim_node` 已发布 RGB**（全链还需 **DepthPlanar** 与相机信息）。通常由 **T4'** 或 **T4''** 拉起仿真与相机，**不要**再开第二套 `airsim_node`。

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 语义全链：默认 inference_mode=on_query、publish_target_goal=false（不自动占 GPU、默认不发规划目标）
ros2 launch hw_insight semantic_perception.launch.py

# 另开终端：每条非空 /uav/target_query 触发一次推理（开放词汇，与 YOLO-World 一致）
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: 'red car'"

# 观察输出（可只开其一）
ros2 topic echo /uav/detections_2d
ros2 topic echo /uav/semantic_targets_camera
ros2 topic echo /uav/semantic_targets_world

# 仅 2D 检测节点（默认 continuous，按 interval 轮询；与全链默认不一致时显式写 on_query）
ros2 launch hw_insight yolo_world_test.launch.py
# ros2 launch hw_insight yolo_world_test.launch.py inference_mode:=on_query score_thr:=0.25 interval:=1.0
# on_query 下单节点同样用 ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: …" 触发
```

规划 + RViz 叠图（`detections_image_overlay`）与话题预检、深度与坐标说明见 **`hw_insight/docs/yolo_world_airsim_online_test.md`**。

### 9.2 当前标准工作流

1. 先验证 PX4、XRCE、ROS 主链和 TUI 是否健康。
2. 启动 `llm_client.py`，按方向键选择 `Groq` 或 `Ollama`，确认模型后进入终端。
3. 若选择 Ollama，等待冷启动预热完成后再输入第一条真实指令。
4. 用自然语言或 JSON 验证单步动作链路。
5. 验证多步 `plan` 顺序执行与离线安全拦截逻辑。
6. 用 `flight_regression_runner.py` 做回归。
7. 需要验证视觉链时：在 **T4' / T4''** 已出图的前提下按 **T_YOLO** 启动 `semantic_perception` 或 `yolo_world_test`，用 **`/uav/target_query`** 触发 **`on_query`** 推理并对照 **`yolo_world_airsim_online_test.md`**。
8. 在此基础上再引入新模块，避免一次性同时改 LLM、视觉、规划、飞控。

### 9.3 当前不应直接切入的开发方式

- 不建议在未固定 Phase 0 回归前直接接 EGO-Planner 全链。
- 不建议把视觉模块直接耦合进 `llm_client.py`。
- 不建议让 LLM 直接输出底层姿态角、原始 MAVLink 指令或 PID 参数。

---

## 10. 当前验收标准（仅针对已实现部分）

### 10.1 功能验收

| 编号 | 验收项 | 状态 |
|------|--------|------|
| F-1 | 自然语言可转换为动作 JSON 并发布到 `/uav/user_command` | ✅ |
| F-2 | `TAKEOFF`、`MOVE_REL`、`LAND` 可完成闭环执行 | ✅ |
| F-3 | 多步 `plan` 可按序等待执行 | ✅ |
| F-4 | 未知命令不会触发危险动作 | ✅ |
| F-5 | `Ollama` 模式可作为本地推理入口使用 | ✅ |
| F-6 | 启动时可交互式选择 `Groq` / `Ollama` 与具体模型 | ✅ |
| F-7 | 系统离线时普通飞行指令会被前置安全层拦截 | ✅ |

### 10.2 安全验收

| 编号 | 验收项 | 状态 |
|------|--------|------|
| S-1 | launch 后飞机不自动解锁 | ✅ |
| S-2 | 仅在飞行指令触发后进入 Offboard 解锁链 | ✅ |
| S-3 | 落地后 `flight_enabled` 自动重置 | ✅ |
| S-4 | 紧急停止和返航可走最短安全路径 | ✅ |
| S-5 | 未收到新鲜遥测时，普通指令默认不进入执行层 | ✅ |

### 10.3 非本阶段验收

以下暂不作为当前版本 DoD：

- 视觉目标检测准确率
- 深度对齐误差
- 里程计漂移 / 坐标系一致性指标（AirSim/PX4 odom 基线）
- Ego-Planner / Fast-Planner 实时性
- 动态障碍物绕飞效果

规划接入最小验收（新增）：

- 发布 `GOTO_NED` 后，`/uav/target_goal` 可收到目标点。
- planner 发布速度指令到 `/uav/planner_cmd_vel` 或 `/uav/planner_cmd_vel_stamped` 后，`move_velocity` 可执行对应速度。
- planner 指令超时后，桥接层会发布一次零速度以安全刹停。
- **仿真建图**：`ego_planner_node` 存活；`/grid_map/occupancy`（或 inflate）在 RViz 中可见；深度话题约 ≥1 Hz 时 `odom_depth_timeout` 不持续报错。
- **AirSim 修改相机后**：须重启 Unreal/AirSim，且 `grid_map/fx,fy,cx,cy` 与 `settings.json` 分辨率与 FOV 一致。

---

## 11. 风险与设计约束

| 项目 | 当前判断 | 说明 |
|------|----------|------|
| WSL2 时间同步抖动 | ⚠️ 持续存在 | 已通过延长 ARM 窗口缓解，但未根治 |
| 本地 / 远端 LLM 格式稳定性 | ⚠️ 需验证 | 已增加 JSON 模式、别名库与鲁棒提取器，但不同模型的输出一致性仍需回归 |
| 视觉模块与飞控耦合风险 | ⚠️ 高 | 必须保持感知、规划、执行分层，不把视觉逻辑写进执行节点 |
| MAVROS 2 与当前链路兼容性 | ⚠️ 待评估 | 当前主链是 `px4_msgs + uXRCE-DDS`，如切换需重新定义桥接层边界 |
| 任务复杂度上升 | ⚠️ 高 | 后续应引入独立 `mission_manager` 和安全过滤层 |
| 交互式终端依赖 TTY | ⚠️ 存在 | 方向键菜单仅适用于独立终端；`ros2 launch`/非 TTY 下自动退回参数模式 |

---

## 12. 近期开发优先级

### P0：同步本地 LLM 主链

- 将 Ollama / Groq 双后端使用方式固化到标准 SOP
- 整理本地 / 远端推荐模型配置与 `~/.bashrc` 环境变量模板
- 验证不同模型在中文指令、多步 plan、坐标方向和 JSON 模式下的稳定性

### P1：完善当前阶段工程化

- TUI 加入 WARN / ERROR 分级
- `flight_regression_runner` 输出 JSON / Markdown 报告
- 将 `/uav/llm_task_status` 的关键字段稳定化
- 评估是否将 `<think>` 日志同步到独立 ROS topic / 本地审计文件

### P2：视觉语义识别 ROS 2 联调与升级（YOLO-World + Grounding + TF）

> **YOLO-World 本地推理与 ROS 2 语义感知链已完成**（S8~S9）。

- ✅ YOLO-World 本地部署与推理测试（`test_yolo_world.py`，S8 完成）
- ✅ `yolo_world_detector.py`：RGB + prompt → `/uav/detections_2d`
- ✅ `target_grounding_node.py`：DepthPlanar + 中位数深度 → camera frame 3D 点
- ✅ `semantic_target_tf_node.py`：AirSim odom 驱动 camera→world 变换
- ✅ `detections_image_overlay.py`：RViz 叠画 bbox，可选叠加 `depth_m`（`/uav/camera/detections_overlay`）
- ✅ **AirSim 在线联调手册**与阶段预检脚本（见 `docs/yolo_world_airsim_online_test.md`）
- ✅ **`semantic_perception.launch.py` 默认 `on_query` + `publish_target_goal=false`**
- 🔲 修复当前 Python 环境中的 GPU 版本错配，恢复稳定 GPU 推理
- 🔲 将测试目标从固定 `red car` 升级为动态语义 prompt（如 `person`、`yellow clothes person`）

### P3：EGO-Planner 与语义任务编排

- EGO-Planner 仿真链路已打通；后续：语义目标与 `FIND_AND_GOTO` 深度联动、安全层与 `mission_manager`、动态障碍与性能 profiling
- 定义 `target_position_world` → EGO-Planner → PX4 bridge 完整数据流（当前经 B-spline→Twist→`keyboard_velocity`）
- 验收目标：输入 `"找到目标并前往"` 后，无人机能在 AirSim 仿真中自主定位并接近目标

---

## 13. Ego-Planner / AirSim 集成工程记录（运维必读）

### 13.1 关键路径与话题（摘要）

| 用途 | 话题或节点 |
|------|------------|
| 规划器里程计 | `/airsim_node/PX4/odom_local_ned` → `ego_planner_node` `odom_world` / `grid_map/odom` |
| 深度图 | `/airsim_node/PX4/CameraDepth1/DepthPlanar` → `grid_map/depth` |
| 辅助点云（可选） | `depth_image_proc/point_cloud_xyz_node` → `/uav/camera/points` → `grid_map/cloud` |
| 语义检测 JSON | `/uav/detections_2d`（`std_msgs/String`） |
| 相机系语义目标 | `/uav/semantic_targets_camera` |
| 世界系语义目标 | `/uav/semantic_targets_world` |
| RViz 语义目标 Marker | `/uav/semantic_target_marker` |
| RViz 检测叠图 | `/uav/camera/detections_overlay`（`detections_image_overlay`） |
| 目标 | `/uav/target_goal` → `/goal_pose` |
| 规划输出 | `/uav/ego_planner/bspline`；经 `ego_bspline_to_twist_relay` → `/uav/planner_cmd_vel_stamped`（**Twist 线速度为 ENU world**）→ **`planner_velocity_bridge` 内 ENU→NED** → `/hw_insight/keyboard_velocity` |
| ENU 里程计 + TF | `odom_ned_to_enu_node`：发布 ENU `Odometry` + 广播 `world`→`base_link`（供 RViz / 规划器） |
| 占用栅格可视化 | `/grid_map/occupancy`、`/grid_map/occupancy_inflate`（RViz Fixed Frame: `world`） |
| 静态 TF | `world` → `PX4`（identity），与 AirSim NED 帧对齐 |

### 13.2 AirSim 相机（须与 `ego_planner_integration.launch.py` 内参一致）

- 配置文件位置（Windows）：`%USERPROFILE%\Documents\AirSim\settings.json`（WSL 下常见为 `/mnt/c/Users/<用户>/Documents/AirSim/settings.json`）。
- **修改 FOV、分辨率或相机安装位姿后必须重启 AirSim**，并同步更新 `grid_map/fx, fy, cx, cy` 与 `plan_env` 中 `cam2body_`（光学 RDF → NED 机体 + 安装平移），否则会出现体素错位、“大平面”、与 RGB 上下颠倒等假象。
- 当前推荐方向：相机在桨平面上方约 **5–10 cm**（NED 中 `Z` 为负表示向上）、**Pitch≈0** 平视；FOV 约 **90°** 量级；分辨率 **640×480** 以减轻带宽与 RViz 负载。

### 13.3 WSL2 与 DDS

- 若出现节点互不可见或 SHM 报错：启动前执行 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

### 13.4 构建范围

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_env ego_planner hw_insight
```

修改 `external/ego_planner_core` 下 C++ 后须重新编译 `plan_env` / `ego_planner`。

### 13.5 相关仓库与文档

- **包内文档地图（四份主文档分工）**：`docs/DOCUMENTATION_INDEX.md`
- **系统技术实现白皮书（写作/归档）**：`docs/系统技术实现白皮书.md`
- **第四章关键技术实现审计报告**：`docs/第四章_系统关键技术实现审计报告.md`
- 可行性/迁移说明：`docs/ego_planner_feasibility_report.md`
- 集成过程与排障：`docs/integration_log_v1.md`
- 话题与帧约定：`config/mapping_config.yaml`
- **YOLO × AirSim 在线测试与 RViz 叠图**：`hw_insight/docs/yolo_world_airsim_online_test.md`
- RViz：`rviz/ego_planner_debug.rviz`（占用体素 AxisColor、DepthPoints 默认关闭以减负；**YOLOOverlay** 显示 `/uav/camera/detections_overlay`）

---

## 14. 结论

本项目已经完成“无人机自主决策系统”的第一阶段基础设施：**语义解析、动作协议、安全执行、状态反馈、测试回归**。  
在仿真侧已增加 **EGO-Planner 局部规划与占用栅格建图** 的可运行最小闭环；**推荐**使用 `uav_sim.launch.py` 单入口启动飞控链与可选规划（勿与 `text_command_test` 重复起 AirSim）。

**Phase 2 技术路线已锁定**，完整闭环为：

> LLM 任务理解 → **YOLO-World** 开放词汇视觉识别 → AirSim DepthPlanar 几何 Grounding → **AirSim/PX4 里程计** world 对齐 → `target_position_world` → **EGO-Planner** 局部轨迹规划 → PX4 执行

接下来的演进重点按优先级排列：

1. **视觉产品化**：在已落地的 **`FIND_AND_GOTO`**、**`/uav/target_query`** 与 **`semantic_perception`（默认 `on_query` + `publish_target_goal=false`）** 上，做 **Stage I** 场景级回归；按需补充多目标选择、超时、抢占，以及可选的 **「先视觉摘要再进 LLM」** 编排（当前未实现）。
2. **坐标系与精度**：camera → body → world 在机动飞行下的稳定度验证；位姿基线固定为 **`odom_local_ned` + NED→ENU 桥接**（不接入 VINS）。
3. **EGO-Planner 与语义目标编排**：语义发现后的 **`/uav/target_goal`** 与 **`planner_mode_for_goto`** 深度联动、安全抢占与任务级状态机（仿真链已可联调；**主链须另起 `semantic_perception`**，勿假设单条 launch 内含 YOLO）。
4. 任务级自治、持续重规划、安全层与长期任务编排。
5. 真机 PX4 外参与里程计标定（可选，仍不引入 VINS）。
