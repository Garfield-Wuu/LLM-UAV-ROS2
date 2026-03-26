# 开发 PRD：基于本地 LLM 与视觉对齐的无人机自主决策系统

## 1. 文档信息

| 字段 | 内容 |
|------|------|
| 文档版本 | v5.0 |
| 原始版本 | v1.0（2026-03-21） |
| 本次更新 | 2026-03-27（技术路线锁定：YOLO-World + VINS-Fusion + EGO-Planner 完整闭环） |
| 适用项目 | `hw_insight`（PX4 + AirSim + ROS 2 Humble） |
| 当前阶段 | Phase 0/1 已完成；**Phase 2 技术选型已锁定（YOLO-World + VINS-Fusion + EGO-Planner）**；EGO-Planner 仿真最小闭环已接入 |
| 文档定位 | 同时描述目标架构、当前实现和阶段化开发路线 |

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
| 大脑（Cognition） | Ollama（Llama 3 / Gemma） | ⚠️ 部分完成 | `llm_client.py` 已支持 Groq / Ollama 双后端、交互式模型选择、远端 Ollama 环境变量、冷启动预热、鲁棒 JSON 提取与 `<think>` 推理日志 |
| 视觉（Perception） | **YOLO-World** + AirSim Depth Camera | ❌ 未实现 | 已选定 YOLO-World（open-vocabulary detection，`prompt-then-detect` 范式），支持任意文本描述的目标检测；`vision_grounding_node` 尚未实现 |
| 定位（State Estimation） | **VINS-Fusion**（已选定） | ❌ 未实现 | 多传感器优化状态估计，提供 world frame 高精度位姿；仿真阶段以 `odom_local_ned` 替代，真机阶段接入 VINS-Fusion mono + IMU |
| 规划（Planning） | **EGO-Planner**（已选定） | ⚠️ 部分接入 | EGO-Planner（ROS 2）已接入仿真最小闭环；Fast-Planner 不再作为候选；完整路线下将切换至 VINS-Fusion 位姿驱动，形成 `target_position_world` → EGO-Planner → PX4 完整链路 |
| 通信与框架 | ROS 2 Humble + MAVROS 2 | ⚠️ 部分完成 | ROS 2 Humble 已落地；当前飞控通信主链实际使用 `px4_msgs + uXRCE-DDS`，尚未接入 MAVROS 2 |
| 执行与仿真 | PX4 Offboard + AirSim | ✅ 已实现 | 已形成稳定可复现的最小闭环 |
| 可观测性 | TUI + 状态回传 + 回归测试 | ✅ 已实现 | 已有地面站、状态流、回归 runner |

---

## 4. 分层架构定义

### 4.1 目标架构（最终形态）

```
用户自然语言指令
    │
    ▼
llm_client.py（任务理解层）
    │  结构化任务描述：target_category / target_attribute / action
    ▼
vision_grounding_node（视觉语义识别 + 几何 Grounding）
    ├── YOLO-World 开放词汇检测（输入 AirSim RGB + 文本 prompt）
    ├── AirSim DepthPlanar + 相机内参逆投影（camera frame 3D 点）
    └── tf2 坐标变换（camera → body → world）
    │  目标 world 坐标 /uav/target_goal
    ▼
VINS-Fusion（状态估计层，提供 world frame 位姿）
    │  高精度位姿 /vins_estimator/odometry
    ▼
EGO-Planner（局部轨迹规划层）
    │  局部可飞轨迹 → Twist → /hw_insight/keyboard_velocity
    ▼
move_velocity.py + PX4 SITL（控制执行层）
    │
    ▼
AirSim / Real Drone
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

结论：**当前系统已具备“语义理解层 + 动作执行层 + 基础可观测层”，并在仿真侧接入了基础规划链路；但“视觉对齐层 + VINS 层 + 真机规划闭环”仍未落地。**

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

---

## 6. 未实现但已纳入目标架构的模块

以下模块应被视为**后续开发目标**，当前不能在流程文档中当作已完成能力使用：

### 6.1 视觉语义识别与几何 Grounding 层

目标节点：`vision_grounding_node`

核心技术选型：**YOLO-World**（open-vocabulary detection）

计划职责：

- 订阅 AirSim RGB 图像（`/airsim_node/PX4/Scene`）与深度图（`DepthPlanar`）。
- 接收来自任务理解层的文本 prompt（如 `red car`），使用 YOLO-World 进行开放词汇目标检测，输出 bbox / 置信度。
- 对 bbox 区域提取深度信息，取中位数作为鲁棒深度估计，结合相机内参 `(fx, fy, cx, cy)` 完成逆投影，恢复 camera frame 3D 坐标。
- 通过 tf2 完成 `camera frame → body frame → world frame` 坐标变换（固定外参 + VINS-Fusion 实时位姿）。
- 发布 `target_position_world` 到 `/uav/target_goal`（`PoseStamped`）。

选型说明：YOLO-World 的 `prompt-then-detect` 范式将词汇嵌入重参数化进模型权重，支持任意类别文本描述，推理效率接近标准 YOLO，适合 `red car`、`vehicle near building` 等开放表达目标。

当前状态：**未开始实现**。

### 6.2 状态估计层

目标节点：`vins_estimator`（VINS-Fusion）

核心技术选型：**VINS-Fusion**（基于优化的多传感器状态估计器，官方支持 mono/stereo + IMU）

计划职责：

- 提供无人机在 world/map frame 下的高频、连续、鲁棒位姿估计。
- 为几何 Grounding 层的 `camera → world` 坐标变换提供实时位姿。
- 在 GPS 不可靠或复杂室内/室外环境中增强导航稳定性。
- 仿真阶段以 `odom_local_ned`（AirSim 提供）作为位姿源替代；真机阶段接入 VINS-Fusion mono + IMU，通过 remap 切换，其余节点不变。

当前状态：**未开始实现**（仿真阶段以 AirSim odom 替代）。

### 6.3 局部规划层

目标节点：`ego_planner_node`

核心技术选型：**EGO-Planner**（ESDF-free、gradient-based local planner，面向四旋翼）

计划职责：

- 接收 `target_position_world`（来自 `vision_grounding_node` 发布的 `/uav/target_goal`）作为导航目标。
- 融合深度点云（AirSim DepthPlanar / 点云）生成局部无碰撞轨迹，持续重规划。
- 输出局部可飞轨迹（B-spline → Twist → `keyboard_velocity`）。
- 完整路线下使用 VINS-Fusion 提供的 world frame 位姿驱动规划。

当前状态：**EGO-Planner 已在仿真侧最小接入**（`planner_integration.launch.py`，以 `odom_local_ned` 驱动）；Fast-Planner 不作为候选；VINS-Fusion 位姿接入为后续工作。

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
| `lesson3.launch.py` / `lesson4.launch.py` / `lesson4_color.launch.py` | 深度/点云相关实验入口 | ⚠️ 保留留痕 | 不是当前主链，但对视觉阶段有参考价值 | 后续可复用到 `vision_grounding_node` 开发 |

### 7.3 文档留痕原则

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
- 位姿对齐：**VINS-Fusion**（仿真阶段以 `odom_local_ned` 替代）

计划项：

- 新建 `vision_grounding_node`（YOLO-World 推理 + 深度 Grounding + tf2 坐标变换）
- 接入 AirSim RGB + DepthPlanar，固定相机内参 `(fx, fy, cx, cy)`
- YOLO-World 开放词汇检测，输出 bbox
- bbox 区域中位数深度 + 逆投影 → camera frame 3D 点
- tf2 变换链：camera → body → world（固定外参 + VINS 位姿）
- 发布 `target_position_world` 到 `/uav/target_goal`

状态：🔲 未开始（技术选型已锁定）

### Phase 3：VINS-Fusion 位姿接入 + EGO-Planner 完整闭环

目标：将 Phase 2 生成的 `target_position_world` 接入 EGO-Planner，并用 VINS-Fusion 替换仿真位姿源，实现完整避障飞行闭环。

技术选型（已锁定）：
- 状态估计：**VINS-Fusion**（mono + IMU）
- 局部规划：**EGO-Planner**（ESDF-free，已有仿真最小接入）

计划项：

- VINS-Fusion mono + IMU 接入，发布 `/vins_estimator/odometry`
- `vision_grounding_node` tf2 变换切换到 VINS 位姿（仅改 remap）
- EGO-Planner 接收 `target_position_world` → 生成局部轨迹 → PX4 执行
- 定义目标更新 → 重规划 → 执行的动态闭环
- 安全层：规划速度上限、地理围栏、丢目标 / 低电量策略

状态：🟡 已启动（EGO-Planner 仿真最小接入；VINS-Fusion 和完整闭环待实现）

当前进展（2026-03 ~ 2026-03-26）：

- `text_command_bridge` 已支持在 `GOTO_NED` 时发布 `/uav/target_goal`（`geometry_msgs/PoseStamped`）；可选 `planner_mode_for_goto` 避免与规划速度抢通道。
- `planner_velocity_bridge`：订阅 planner `Twist` / `TwistStamped`，限幅与超时后零速，发布到 `/hw_insight/keyboard_velocity`。
- `ego_bspline_to_twist_relay`：`traj_utils/Bspline` → `TwistStamped`（无有效 B-spline 时不发零速，避免干扰手飞）。
- `planner_integration.launch.py`：AirSim + `move_velocity` + `text_command_bridge` + `depth_image_proc/point_cloud_xyz_node` + `ego_planner_integration.launch.py` + 静态 TF `world`→`PX4`。
- `ego_planner_integration.launch.py`：`ego_planner_node` 参数（栅格分辨率、深度范围、相机内参须与 **AirSim `settings.json` 一致**）、可选 RViz `ego_planner_debug.rviz`。
- 核心包来源：`ros2/src/external/ego_planner_core/`（`ego_planner`、`plan_env`、`traj_utils` 等）；完整上游仓库以 `COLCON_IGNORE` 排除避免重复编译。
- 已修复项（摘要）：`ego_replan_fsm` 回调内 `spin_some` 导致 executor 冲突；无 odom 时忽略 goal；`grid_map` 深度投影缓冲区过小导致 segfault；NED 地图 Z 范围与 `ground_height` 不匹配导致起飞后栅格“消失”；WSL2 建议 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`；`depth_image_proc` 与 subscriber QoS 对齐。

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

### 8.1 启动拓扑

```bash
# T1 AirSim（Windows Unreal Engine 内点击 Play）

# T2 PX4 SITL
cd /home/hw/px4v1.15.2
make px4_sitl_default none_iris

# T3 XRCE Agent
MicroXRCEAgent udp4 -p 8888

# T4 ROS 主链
cd /home/hw/hw-ros2/ros2
source install/setup.bash
ros2 launch hw_insight text_command_test.launch.py

# T4' ROS 主链（规划接入模式，可选；须先 source /opt/ros/humble + install）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
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

### 8.2 当前标准工作流

1. 先验证 PX4、XRCE、ROS 主链和 TUI 是否健康。
2. 启动 `llm_client.py`，按方向键选择 `Groq` 或 `Ollama`，确认模型后进入终端。
3. 若选择 Ollama，等待冷启动预热完成后再输入第一条真实指令。
4. 用自然语言或 JSON 验证单步动作链路。
5. 验证多步 `plan` 顺序执行与离线安全拦截逻辑。
6. 用 `flight_regression_runner.py` 做回归。
7. 在此基础上再引入新模块，避免一次性同时改 LLM、视觉、规划、飞控。

### 8.3 当前不应直接切入的开发方式

- 不建议在未固定 Phase 0 回归前直接接 VINS / Planner。
- 不建议把视觉模块直接耦合进 `llm_client.py`。
- 不建议让 LLM 直接输出底层姿态角、原始 MAVLink 指令或 PID 参数。

---

## 10. 当前验收标准（仅针对已实现部分）

### 9.1 功能验收

| 编号 | 验收项 | 状态 |
|------|--------|------|
| F-1 | 自然语言可转换为动作 JSON 并发布到 `/uav/user_command` | ✅ |
| F-2 | `TAKEOFF`、`MOVE_REL`、`LAND` 可完成闭环执行 | ✅ |
| F-3 | 多步 `plan` 可按序等待执行 | ✅ |
| F-4 | 未知命令不会触发危险动作 | ✅ |
| F-5 | `Ollama` 模式可作为本地推理入口使用 | ✅ |
| F-6 | 启动时可交互式选择 `Groq` / `Ollama` 与具体模型 | ✅ |
| F-7 | 系统离线时普通飞行指令会被前置安全层拦截 | ✅ |

### 9.2 安全验收

| 编号 | 验收项 | 状态 |
|------|--------|------|
| S-1 | launch 后飞机不自动解锁 | ✅ |
| S-2 | 仅在飞行指令触发后进入 Offboard 解锁链 | ✅ |
| S-3 | 落地后 `flight_enabled` 自动重置 | ✅ |
| S-4 | 紧急停止和返航可走最短安全路径 | ✅ |
| S-5 | 未收到新鲜遥测时，普通指令默认不进入执行层 | ✅ |

### 9.3 非本阶段验收

以下暂不作为当前版本 DoD：

- 视觉目标检测准确率
- 深度对齐误差
- VINS 漂移指标
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

### P2：启动视觉语义识别最小链路（YOLO-World）

- 新建 `vision_grounding_node`，集成 YOLO-World 推理接口
- 固定使用 DepthPlanar，实现 bbox 区域中位数深度提取
- 基于相机内参完成逆投影，输出 camera frame 3D 点
- 配置 tf2 外参（camera→body），用 AirSim odom 替代 VINS 先跑通坐标变换
- 先打通 `/uav/target_goal`，再接 Planner

### P3：VINS-Fusion 接入 + EGO-Planner 完整闭环

- EGO-Planner 仿真链路已打通；后续：VINS-Fusion 位姿接入（remap 切换）、安全层与 `mission_manager`、动态障碍与性能 profiling
- 定义 `target_position_world` → EGO-Planner → PX4 bridge 完整数据流（当前经 B-spline→Twist→`keyboard_velocity`）
- 验收目标：输入 `"找到红色汽车并前往"` 后，无人机能在 AirSim 仿真中自主定位并接近目标

---

## 13. Ego-Planner / AirSim 集成工程记录（运维必读）

### 13.1 关键路径与话题（摘要）

| 用途 | 话题或节点 |
|------|------------|
| 规划器里程计 | `/airsim_node/PX4/odom_local_ned` → `ego_planner_node` `odom_world` / `grid_map/odom` |
| 深度图 | `/airsim_node/PX4/CameraDepth1/DepthPlanar` → `grid_map/depth` |
| 辅助点云（可选） | `depth_image_proc/point_cloud_xyz_node` → `/uav/camera/points` → `grid_map/cloud` |
| 目标 | `/uav/target_goal` → `/goal_pose` |
| 规划输出 | `/uav/ego_planner/bspline`；经 `ego_bspline_to_twist_relay` → `/uav/planner_cmd_vel_stamped` → `planner_velocity_bridge` → `/hw_insight/keyboard_velocity` |
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

- 可行性/迁移说明：`docs/ego_planner_feasibility_report.md`
- 集成过程与排障：`docs/integration_log_v1.md`
- 话题与帧约定：`config/mapping_config.yaml`
- RViz：`rviz/ego_planner_debug.rviz`（占用体素 AxisColor、DepthPoints 默认关闭以减负）

---

## 14. 结论

本项目已经完成“无人机自主决策系统”的第一阶段基础设施：**语义解析、动作协议、安全执行、状态反馈、测试回归**。  
在仿真侧已增加 **EGO-Planner 局部规划与占用栅格建图** 的可运行最小闭环（与 `text_command_test` 并行，不强制替代）。

**Phase 2 技术路线已锁定**，完整闭环为：

> LLM 任务理解 → **YOLO-World** 开放词汇视觉识别 → AirSim DepthPlanar 几何 Grounding → **VINS-Fusion** world frame 位姿对齐 → `target_position_world` → **EGO-Planner** 局部轨迹规划 → PX4 执行

接下来的演进重点按优先级排列：

1. **视觉语义识别**：`vision_grounding_node` 实现（YOLO-World + 深度逆投影 + tf2）。
2. **坐标系闭环**：camera → body → world 完整变换链验证，先以 AirSim odom 替代 VINS。
3. **VINS-Fusion 接入**：mono + IMU 位姿估计，remap 切换位姿源，其余节点不变。
4. **EGO-Planner 完整闭环**：VINS 位姿驱动，`target_position_world` → 规划 → 执行验收。
5. 任务级自治、持续重规划、安全层与长期任务编排。
