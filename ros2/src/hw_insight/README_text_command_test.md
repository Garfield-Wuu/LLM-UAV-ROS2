# 无人机自主决策系统开发流程指南

> 与 **PRD**（`PRD_text_command_flight_mvp.md` **v5.6**）和 **会话交接**（`SESSION_HANDOVER.md`）保持一致；LLM 启动方式以本文 **§3 终端 D** 为准。  
> **文档谁看谁先看**：[`docs/DOCUMENTATION_INDEX.md`](docs/DOCUMENTATION_INDEX.md)（四份文档分工与单一事实源）。**系统架构**：[`docs/SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md)（五层框架，无 VINS）。

## 1. 当前阶段说明

本仓库当前处于"Phase 0：语义指令飞行 MVP 已完成；Phase 1：LLM 工程化增强已落地；Phase 2：视觉语义 + FIND_AND_GOTO 已落地；**AirSim 侧 EGO-Planner 避障仿真链已可联调**（`uav_sim.launch.py`、ENU→NED 速度桥、RViz 目标与 TF）；位姿统一为 **AirSim/PX4 里程计**（**不接入 VINS-Fusion**），架构见 [`docs/SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md)"阶段。

已经具备的能力：

- 自然语言或 JSON 指令转飞行动作
- PX4 Offboard 执行闭环
- **12 个动作协议**（含 `FIND_AND_GOTO` 视觉语义搜索 action）
- 多步 `plan` 顺序执行
- TUI 状态监控
- 回归测试
- YOLO-World ROS 2 实时检测（**GPU 推理**，torch cu128 / CUDA 12.9 已验证；`/uav/target_query` 动态 prompt 或静态 `texts` 参数）
- 深度图视觉定位支撑（bbox 区域中位数深度 + `camera_info` 逆投影；论文主体不作为核心贡献展开）
- camera frame → world frame 目标点转换（ENU 输出）
- `/uav/target_goal` 语义目标发布与 RViz marker 可视化
- **LLM → `FIND_AND_GOTO` → YOLO-World 检测 → ENU→NED 转换 → `GOTO_NED` 自动飞行**（视觉任务端到端闭环已接通，等待场景联调验证）

尚未完成的能力（下一阶段目标）：

- **EGO-Planner** 与 `FIND_AND_GOTO` / `/uav/target_goal` 的深度联动（仿真已可走 planner 避障；语义任务与规划抢占策略待产品化）
- 复杂属性 prompt 的泛化精度验证（不同场景、目标遮挡、多目标选择）
- 真机 PX4 外参与里程计标定（仍使用飞控位姿，**不计划接入 VINS-Fusion**）

已部分接入（仿真侧，可选）：

- **EGO-Planner（ROS 2）**：推荐 **`launch/uav_sim.launch.py`**（`enable_ego_planner:=true` 合并飞控链 + `ego_planner_integration`）；或历史入口 `planner_integration.launch.py`。`odom_ned_to_enu_node` 发布 ENU 里程计并广播 **`world`→`base_link`** TF；`planner_velocity_bridge` 将规划器 **ENU world** 线速度转为 **NED** 再送 `move_velocity`。与仅手飞链 **二选一** 启动，勿双开 `airsim_node`。

因此，当前开发流程应以**先保证现有语义飞控链稳定，再验证语义感知链稳定，最后再做自然语言视觉任务闭环**为原则。

## 1.1 历史能力与兼容入口说明

为了保留项目演进痕迹，以下历史能力和入口仍在文档中保留，不代表它们都是当前默认主流程：

| 能力/入口 | 当前状态 | 说明 |
|-----------|----------|------|
| 中文文本指令 | ⚠️ 兼容保留 | 仍可用，用于快速手测和回归对照 |
| JSON 动作协议 | ✅ 当前推荐 | 仍是最稳定的直接测试入口 |
| Groq 推理模式 | ✅ 常用 | 启动 `llm_client` 时用方向键可选；需 `GROQ_API_KEY`（建议写入 `~/.bashrc`） |
| Ollama 推理模式 | ✅ 常用 | 本地或远端均可；远端默认读 `ANTHROPIC_BASE_URL` / `ANTHROPIC_MODEL`（建议写入 `~/.bashrc`） |
| `launch/llm_flight.launch.py` | ❌ 已停用 | 因 `ros2 launch` 捕获 stdin，交互式输入不可用 |
| 早期底层脚本如 `keyboard_velocity.py` | ⚠️ 保留留痕 | 不再是主流程，但对底层调试仍有参考价值 |

后续如果某个入口不再推荐使用，文档会保留其留痕，并明确写出停用原因和替代路径，而不是直接删除。

## 2. 启动前提

- AirSim 场景已在 Windows 侧启动
- PX4 SITL 使用 `none_iris`
- `MicroXRCEAgent` 已可直接调用
- WSL2 环境建议在**每个 ROS 终端**执行：`export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`（避免 FastDDS 共享内存问题）
- 使用 **Groq**：已设置 `GROQ_API_KEY`（推荐写入 `~/.bashrc`，避免新开终端丢失）
- 使用 **远端 Ollama**：已设置 `ANTHROPIC_BASE_URL`、`ANTHROPIC_MODEL`；网关若要求可再设 `ANTHROPIC_API_KEY` / `ANTHROPIC_AUTH_TOKEN`（同上建议持久化）
- 使用 **本机 Ollama**：已安装并可 `ollama serve`，且本机 `ollama pull` 过所选模型

## 3. 标准启动顺序

建议至少打开 **5** 个终端（A、B、**C / C' / C'' 三选一**、D、可选 E）。若要做 **`FIND_AND_GOTO` 自然语言找车/找人**、或要在 RViz **YOLOOverlay** 看检测框，还须再开 **第 6 个终端 F（语义感知链）**——**`planner_integration.launch.py` / `uav_sim.launch.py` 均不内置 YOLO**，须与主链并行启动 `semantic_perception.launch.py`（详见 PRD §4.2.1、§9 `T_YOLO`）。

修改过 `hw_insight` 的 `launch/*.py` 后，须在本工作区执行 **`colcon build --packages-select hw_insight && source install/setup.bash`**，否则 `ros2 launch hw_insight …` 仍可能用 **install 目录里的旧 launch**。

### 终端 A：PX4 SITL

```bash
cd /home/hw/px4v1.15.2
make px4_sitl_default none_iris
```

预期信号：

- `Simulator connected on TCP port 4560.`
- `Ready for takeoff!`

### 终端 B：XRCE Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

预期信号：

- `session established`

### 终端 C：主飞控链（**C / C' / C'' 三选一**，勿双开 `airsim_node`）

**C — 仅手飞 / LLM（无 EGO-Planner）**

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py
```

**C' — 仿真 + 可选 EGO-Planner（避障联调推荐）**

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true use_rviz:=true
# 可调：max_vel:=1.5 max_acc:=1.5
```

预期信号：

- `Text command bridge ready on /uav/user_command`
- 若启用规划：`PlannerVelocityBridge started`、`ego_planner_node` 等节点出现；RViz Fixed Frame 建议 **`world`**

**C'' — `planner_integration`（与 C / C' 同类互斥，勿双开 `airsim_node`）**

与 `uav_sim` 二选一或按项目习惯选用；已含 AirSim、`move_velocity`、`text_command_bridge`、`ego_planner_integration`（RViz、`detections_image_overlay` 叠图节点等）。**仍不含** `yolo_world_detector`。

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight planner_integration.launch.py use_rviz:=true
# 不需要 RViz 叠图时可：enable_detection_overlay:=false
```

### 终端 F：语义感知链（**FIND_AND_GOTO / YOLO RViz 叠图需要**）

在 **终端 C / C' / C'' 已成功出图**（`/airsim_node/PX4/CameraDepth1/Scene` 有频率）之后再起。默认 **`semantic_perception.launch.py`**：`inference_mode=on_query`（启动后不自动跑 GPU；**每条非空** `/uav/target_query` 推理一次）、`publish_target_goal=false`（不向 `/uav/target_goal` 抢发，避免与 bridge 规划目标冲突；若你要语义节点直驱 planner 再显式改参）。

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight semantic_perception.launch.py
# 可选：score_thr:=0.15 device:=auto
```

端到端自然语言「前往红色汽车」：**终端 C'' + F + D**（先起飞/有遥测，再发话）。观察：`/uav/target_query`、`/uav/detections_2d`、`/uav/semantic_targets_world`、`/uav/llm_task_status`（`SEARCHING` → `SEARCH_FOUND` → `GOTO_NED` 等）。若 `dets=0`，`/uav/semantic_targets_camera` 与 world 可能无输出（见 `docs/yolo_world_airsim_online_test.md`）。

### 终端 D：LLM 交互终端（**必须独立终端**，勿放进 `ros2 launch`）

`llm_client` 会在**启动时**用**方向键**选择推理后端与模型；Ollama 会先做一次 **warm-up**（冷启动加载模型），完成后再出现输入提示符。

**推荐：先把密钥/远端地址写入 `~/.bashrc`，再 `source ~/.bashrc`**

```bash
# Groq（示例）
echo 'export GROQ_API_KEY="gsk_你的key"' >> ~/.bashrc

# 远端 Ollama 网关（与 Claude-Code 兼容的一组变量名，示例）
echo 'export ANTHROPIC_BASE_URL="http://你的主机:端口"' >> ~/.bashrc
echo 'export ANTHROPIC_MODEL="qwen3-coder:30b"' >> ~/.bashrc
echo 'export ANTHROPIC_API_KEY="ollama"' >> ~/.bashrc
echo 'export ANTHROPIC_AUTH_TOKEN="ollama"' >> ~/.bashrc

source ~/.bashrc
```

**启动命令**

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hw_insight llm_client
```

**交互流程简述**

1. 方向键选择 **Groq Cloud** 或 **Ollama**，Enter 确认。
2. **Groq**：方向键选预设模型或「自定义…」手动输入模型名；若已检测到 `GROQ_API_KEY`，回车即可沿用。
3. **Ollama**：可回车保留 `ANTHROPIC_BASE_URL` 显示的主机；程序会尝试从 `{主机}/api/tags` **拉取已安装模型列表**，失败则回退预设列表；方向键选模型或「自定义…」。
4. 选 **Ollama** 时会先 **预热**（终端显示进度），再打印欢迎横幅与**状态感知提示符**（例如 `[离线] ▶`、`[HOVERING|解锁] ▶`）。

**离线与安全**

- 若长时间未收到 `/uav/llm_task_status` 中的新鲜 **TELEMETRY**，普通飞行类自然语言会被**前置拦截**（避免“飞机未连接仍出 JSON”）。
- `RTL` / `LAND` / `HOVER` / `EMERGENCY_STOP` 仍可在离线时下发；需要专家绕过时可在指令前加 `!`（详见节点内提示）。
- 支持模型 **`<think>...</think>` 推理块**：终端一行摘要，**完整内容写入 ROS2 日志**（`~/.ros/log/...`），便于复盘。

**非交互 / 自动化场景**

- 方向键菜单依赖 **TTY**。若从脚本、`ros2 launch` 子进程或非 TTY 启动，会自动**跳过菜单**，改用 ROS 参数与环境变量（与旧版一致）。
- 显式指定 provider 时也会跳过菜单，例如：

```bash
ros2 run hw_insight llm_client --ros-args -p llm_provider:=ollama
```

**本机 Ollama 补充**

```bash
ollama serve   # 另开终端常驻
# 默认地址 http://127.0.0.1:11434 ，无需改 ANTHROPIC_BASE_URL 即可在菜单里选本机模型
```

历史说明：

- 早期曾通过 `launch/llm_flight.launch.py` 把 LLM 放进 launch，因 **stdin 被捕获**无法交互，该 launch 已删除。
- 当前统一为独立终端 `ros2 run hw_insight llm_client`；后续若要做非交互编排，应新写 launch 而非恢复旧文件。

### 终端 E：可选 TUI

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0
```

## 4. 当前推荐开发流程

### 4.1 Phase 0 能力回归

每次引入新改动前，先确认以下链路健康：

1. PX4 与 AirSim 成功连接
2. XRCE Agent 已建立 session
3. `text_command_bridge.py` 正常启动
4. `/uav/llm_task_status` 能持续回传状态
5. TUI 显示 `LIVE`

### 4.2 单步动作验证

优先验证结构化 JSON，而不是一开始就依赖自然语言。

```bash
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"TAKEOFF\",\"params\":{\"altitude\":6.0}}'}"

ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"MOVE_REL\",\"params\":{\"dx\":5.0,\"dy\":0.0,\"dz\":0.0,\"duration\":2.5}}'}"

ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"LAND\",\"params\":{}}'}"
```

### 4.3 自然语言验证

在 JSON 链路稳定后，再测试 `llm_client.py`（须先保证终端 C 已起、`TELEMETRY` 正常，否则提示符多为 `[离线] ▶`，普通指令会被拦截）：

**基础飞行动作**：
```text
起飞到 8 米
向前飞 10 米
先向右飞 5 米，再悬停 3 秒，然后降落
返航
```

**视觉语义任务（FIND_AND_GOTO）**：

> 前提：需同时运行 `semantic_perception.launch.py`（语义感知链），无人机已起飞。

```text
飞到穿黄色衣服的人头顶
找到红色汽车并飞过去
搜索前方的行人
```

系统行为：LLM 解析 → `FIND_AND_GOTO query="..."` → bridge 发布 `/uav/target_query` → YOLO-World GPU 检测 → 深度视觉定位支撑 → world 坐标 → ENU→NED → `GOTO_NED` → 飞行到目标。

观察指标：
- `TELEMETRY` 中 `command` 字段变为 `SEARCHING`，有 `searching_query` 和 `search_remaining_sec`
- 检测成功时出现 `SEARCH_FOUND` 事件，随后 `command` 变为 `GOTO_NED`
- 超时（20s 无结果）出现 `SEARCH_FAILED`，回落 `HOVER`

### 4.4 多步计划验证

重点验证：

- 多步 `plan` 是否按顺序执行
- `duration` 型动作是否按时结束
- `GOTO_NED` / `YAW_TO` 是否能正确等待 `IDLE`

### 4.5 回归测试

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hw_insight flight_regression_runner
```

### 4.6 再引入下一层模块

当前已完成：LLM → FIND_AND_GOTO → 视觉搜索 → GOTO_NED 基础闭环。

后续建议开发顺序：

1. 联调验证 FIND_AND_GOTO 在实际 AirSim 场景中的端到端链路（Stage I 测试）
2. 验证 `/uav/semantic_targets_world` 在无人机运动时的坐标稳定性
3. 将 FIND_AND_GOTO 与 EGO-Planner 编排（发现目标后稳定走 planner 目标与 `planner_mode_for_goto`，与纯 GOTO_NED 测试分层）
4. 多目标场景下 FIND_AND_GOTO 目标选择策略优化
5. 论文实验优先回到“自然语言 → JSON 任务原语 → PX4/AirSim 行为”的转换可靠性评估

不要同时改 LLM、视觉、规划和飞控执行层。

## 5. 当前可用输入方式

### 5.1 中文文本（兼容模式）

```bash
ros2 topic pub --once /uav/user_command std_msgs/msg/String "{data: 起飞 6}"
ros2 topic pub --once /uav/user_command std_msgs/msg/String "{data: 前进 2 2}"
ros2 topic pub --once /uav/user_command std_msgs/msg/String "{data: 悬停 2}"
ros2 topic pub --once /uav/user_command std_msgs/msg/String "{data: 降落}"
```

说明：

- 这是项目早期阶段的重要输入方式，当前仍保留用于快速人工验证。
- 但随着 LLM 和结构化协议稳定，中文文本直发不再是主推荐入口。
- 后续若继续保留，建议只作为兼容层存在，而不是扩展新的复杂语义。

### 5.2 JSON 协议（推荐）

```bash
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"TAKEOFF\",\"params\":{\"altitude\":6.0}}'}"

ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"MOVE_VELOCITY\",\"params\":{\"vx\":2.0,\"vy\":0.0,\"vz\":0.0,\"yaw_rate\":0.0,\"duration\":2.0}}'}"

ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"LAND\",\"params\":{}}'}"
```

详细协议见：`COMMAND_PROTOCOL.md`

说明：

- 这是从 v1.0 到当前阶段始终保留的稳定入口。
- 即便后续接入视觉和规划，动作层调试仍建议保留这一入口，作为分层排障基线。

## 6. 状态反馈与观察点

状态话题：

- `/uav/llm_task_status` (`std_msgs/String`)

查看方式：

```bash
ros2 topic echo /uav/llm_task_status
```

当前关键状态：

- `READY`
- `RECEIVED`
- `MAPPED`
- `PUBLISHED`
- `TELEMETRY`
- `UNKNOWN_COMMAND`

`TELEMETRY` 目前包含：

- `position`
- `velocity`
- `heading_deg`
- `arming_state`
- `nav_state`
- `target_z_ned`

## 7. 当前阶段验收清单

- 能从自然语言生成合法动作 JSON
- 能完成 `TAKEOFF -> MOVE_REL / MOVE_VELOCITY -> LAND`
- 多步计划能按顺序执行
- 未知命令安全降级
- TUI 显示链路、模式、位置和动作事件
- 回归 runner 可稳定完成整套序列

## 8. 后续模块接入建议（Phase 2 技术路线）

当前目标技术闭环为（**已实现主路径：`FIND_AND_GOTO`**；括号内为尚未实现的可选编排）：

```
用户自然语言指令
  → LLM 任务解析（含 FIND_AND_GOTO 与 query）
  → text_command_bridge：收到 FIND_AND_GOTO 后发布 `/uav/target_query`
  → （未实现：若要先视觉再规划，应在此插入「一次检测 → 摘要进 LLM prompt」）
  → YOLO-World 开放词汇目标检测（输出 bbox）
  → AirSim 深度图 + 相机内参逆投影（camera frame 3D 点）
  → AirSim/PX4 odom + NED/ENU 桥接 → world frame 对齐
  → bridge 生成 GOTO_NED；planner 模式下 `/uav/target_goal` → EGO-Planner 局部轨迹规划
  → PX4 执行
```

语义感知链节点保持独立 launch（`semantic_perception.launch.py`），**逻辑上**与 `llm_client` / `text_command_bridge` 解耦；**运行期**由 bridge 通过 `/uav/target_query` 串联。

论文重构口径：本节属于系统支撑能力说明，YOLO-World 与 EGO-Planner 不作为核心研究贡献，也不进入主要实验评价指标；论文主体应优先围绕 Ollama 开源 LLM、ROS 2 agent、JSON 结构化输出、参数校验与 PX4/AirSim 闭环执行展开。

### 8.1 视觉语义识别层（YOLO-World）

当前已实现独立节点 `yolo_world_detector.py`。

职责：
- 订阅 AirSim RGB 图像（默认 `/airsim_node/PX4/CameraDepth1/Scene`）
- 接收来自任务理解层的文本 prompt（`/uav/target_query`）或静态参数 `texts`
- 调用 **YOLO-World** 进行开放词汇目标检测
- 输出 `/uav/detections_2d`（JSON：`label`、`score`、`bbox`、`stamp`、`frame_id`）

YOLO-World 选型说明：支持 `prompt-then-detect` 范式，将词汇嵌入重参数化进模型权重，推理效率接近标准 YOLO，同时支持任意文本描述的开放类别检测，适合 `"yellow clothes person"`、`"person"`、`"car"`、`"vehicle near building"` 等场景，不应把系统目标限制在固定 `red car`。

```bash
# 推荐：整条语义感知链（默认 on_query + 默认不向 /uav/target_goal 发点；详见 launch 参数）
ros2 launch hw_insight semantic_perception.launch.py

# on_query：另开终端发 prompt，每条触发一次推理（与 PRD §9 T_YOLO 一致）
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"

# 仅压测 2D 节点（默认 continuous；与全链默认不一致时需 inference_mode:=on_query）
# ros2 launch hw_insight yolo_world_test.launch.py
```

联调步骤、RViz 叠图、深度与话题预检：[`docs/yolo_world_airsim_online_test.md`](docs/yolo_world_airsim_online_test.md)。

### 8.2 视觉定位支撑层（深度图 + 逆投影 + 坐标变换）

当前由 `target_grounding_node.py` 独立负责。

关键实现要点：

1. **深度类型选定**：固定使用 `DepthPlanar` 或 `DepthPerspective` 之一，并保证逆投影公式与之匹配（二者几何含义不同，不可混用）。
2. **稳定深度提取**：根据 YOLO-World 输出的 bbox 提取对应深度区域，去除无效值后取**中位数**作为鲁棒深度估计。
3. **像素 → 相机系 3D**：基于相机内参 `(fx, fy, cx, cy)` 完成逆投影，得到目标在 camera frame 下的三维坐标。
4. **坐标系变换链**：`camera frame → body frame（固定外参）→ world frame（使用 AirSim/PX4 odom，不接入 VINS-Fusion）`。

```bash
# 检查视觉定位支撑输出
ros2 topic echo /uav/semantic_targets_camera --once
```

### 8.3 位姿与坐标服务层（AirSim/PX4 odom）

当前由 `semantic_target_tf_node.py` 使用 AirSim `odom_local_ned` 完成 world 点验证；系统路线不再切换到 VINS-Fusion。

当前实现要点：
- 订阅 `/uav/semantic_targets_camera`
- 订阅 `/airsim_node/PX4/odom_local_ned`
- 完成 `camera_optical -> body -> world` 变换，并处理 NED → ENU
- 发布 `/uav/semantic_targets_world` 与 `/uav/semantic_target_marker`

```bash
# 检查 world 点输出
ros2 topic echo /uav/semantic_targets_world --once
```

### 8.4 轨迹规划层（EGO-Planner）

EGO-Planner 在 **`world`（ENU：x 东、y 北、z 上）** 下规划；PX4 Offboard 速度 setpoint 为 **NED**。链路上必须在 `planner_velocity_bridge` 做 **ENU→NED**（`keyboard.x = twist.y` 北，`keyboard.y = twist.x` 东，`keyboard.z = -twist.z` 下），不可把 `Twist.linear` 当 NED 直通。

当前已提供最小接入链路（ROS 2）：

- **统一 launch（推荐）**：`uav_sim.launch.py` 组合 AirSim、`move_velocity`、`text_command_bridge`、可选 `ego_planner_integration`（参数 `enable_ego_planner`、`use_rviz` 等）。
- **目标**：`/uav/target_goal`（`PoseStamped`，`frame_id` 为 **`world`**）。来源可为 `text_command_bridge`（`GOTO_NED` 且 `publish_target_goal_on_goto`）、RViz「2D Goal Pose」（经 `goal_relay` 等 remap）、或语义链节点。**当 `planner_mode_for_goto:=true` 时**，bridge 会**订阅** `target_goal_topic`：外部目标到达后置 `planner_control_active`，避免持续 hover 覆盖规划速度。
- **里程计 / TF**：`odom_ned_to_enu_node` 将 `odom_local_ned` 转为 ENU 并广播 **`world`→`base_link`**，供规划器与 RViz。
- **规划→执行**：`/uav/ego_planner/bspline` → `ego_bspline_to_twist_relay` → `/uav/planner_cmd_vel_stamped`（ENU）→ **`planner_velocity_bridge`（ENU→NED）** → `/hw_insight/keyboard_velocity` → `move_velocity`。

Phase 2 仍支持由 `semantic_target_tf_node.py` 或 `semantic_goal_to_planner.py` 发布 `/uav/target_goal`；与 bridge 自发布二选一，**避免双写**。

```bash
# 推荐：单入口仿真 + 规划
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight uav_sim.launch.py enable_ego_planner:=true use_rviz:=true

# 历史入口（与 uav_sim / text_command_test 勿重复起 AirSim）
# ros2 launch hw_insight planner_integration.launch.py use_rviz:=true

# 1) 先 TAKEOFF，解锁并进入 Offboard（move_velocity 安全闸门）
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"TAKEOFF\",\"params\":{\"altitude\":6.0}}'}"

# 2) 再 GOTO_NED 或 RViz 2D Goal；观察 bridge 日志 [PLANNER] 外部目标点到达（RViz 场景）
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"GOTO_NED\",\"params\":{\"x\":20.0,\"y\":5.0,\"altitude\":6.0}}'}"

# 调试：直接发 ENU 速度（仅供排查；真链路由 relay 发 stamped）
# ros2 topic pub /uav/planner_cmd_vel_stamped geometry_msgs/msg/TwistStamped "..."
```

### 8.5 执行层（保持不变）

继续保持现有设计原则：

- LLM 负责高层任务语义与动作规划
- EGO-Planner 负责局部轨迹生成，不替代 LLM 的语义理解
- 执行层负责 setpoint 与 Offboard 心跳（`move_velocity.py`）
- 安全层独立于 LLM 存在，运行期持续检查碰撞风险、地理围栏、低电量等异常状态
- 视觉任务协议已接通 `FIND_AND_GOTO`；与 planner 深度编排（自动切避障航迹）仍可继续产品化。

## 9. 常见问题

- `micro_ros_agent: command not found`  
  使用 `MicroXRCEAgent udp4 -p 8888`。

- 飞机解锁后不爬升  
  先检查 PX4、XRCE 和主飞控链是否都已稳定，再发起飞命令。

- 没有 `/fmu/out/*` 话题  
  优先检查 XRCE Agent 是否已建立 session。

- **Groq 提示未设置 API Key**  
  当前 shell 未 `export GROQ_API_KEY`，或从未写入 `~/.bashrc`。写入后 `source ~/.bashrc` 再开 `llm_client`；交互界面若已检测到 key，直接回车即可沿用。

- **Ollama 菜单里主机是 `localhost:11434`，不是远端**  
  当前终端没有 `ANTHROPIC_BASE_URL`（新终端未继承临时 export）。把远端地址写入 `~/.bashrc` 后 `source ~/.bashrc`。

- **没有方向键菜单 / 想全自动启动**  
  非 TTY 或管道启动会跳过菜单；或命令行已带 `llm_provider` 等参数时也会跳过。需要菜单时请用真实终端执行 `ros2 run hw_insight llm_client`。

- **LLM 请求 timed out**  
  远端大模型首次推理较慢，节点已默认较长超时并带 Ollama warm-up；仍不够时可：`ros2 run hw_insight llm_client --ros-args -p llm_timeout_sec:=180.0`。

- **Ollama 输出夹杂说明文字、JSON 难解析**  
  当前已启用 Groq `json_object`、Ollama `format=json` 及鲁棒提取与动作别名；若仍失败，先用 JSON 直发 `/uav/user_command` 做动作层回归，再换模型或看 ROS 日志中的 `[THINK]` / `[LLM RAW]`（需 `-p verbose:=true`）。

- **YOLO-World 当前退回 CPU**  
  节点本身支持 GPU 优先；若当前环境出现 CPU fallback，通常是 Python 环境中的 `torch` CUDA 版本与 NVIDIA 驱动不匹配。当前已定位到一类典型情况：`torch 2.11.0+cu130` 高于驱动支持的 CUDA 12.9，会导致 `torch.cuda.is_available()` 为 `False`。这属于环境问题，不代表系统只能使用 CPU 推理。

- **RViz 有轨迹但飞机不动**  
  先 `TAKEOFF`；确认 Offboard；`planner_mode_for_goto` 开启时需有 **`[PLANNER] 外部目标点到达`** 或先发 `GOTO_NED`，避免 bridge hover 压住 `keyboard_velocity`。

- **目标在前、飞机往侧后方飘**  
  多为规划速度 **ENU 被误当 NED**；确认使用当前 `planner_velocity_bridge`（已做 ENU→NED）。排查：`ros2 topic echo /uav/planner_cmd_vel_stamped` 与机头方向对照。

- 想做“飞到穿黄色衣服人的头顶”  
  启动 `semantic_perception.launch.py` + 主链，用自然语言或 `FIND_AND_GOTO` JSON；详见 §4.3 与 `COMMAND_PROTOCOL.md`。
