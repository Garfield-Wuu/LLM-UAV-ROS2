# 无人机自主决策系统开发流程指南

> 与 **PRD**（`PRD_text_command_flight_mvp.md` v5.3）和 **会话交接**（`SESSION_HANDOVER.md`）保持一致；LLM 启动方式以本文 **§3 终端 D** 为准。

## 1. 当前阶段说明

本仓库当前处于"Phase 0：语义指令飞行 MVP 已完成；Phase 1：LLM 工程化增强（双后端菜单、远端 Ollama、鲁棒解析）已落地；Phase 2：视觉语义识别 + 几何 Grounding + FIND_AND_GOTO 视觉任务闭环已落地，当前进入联调验证与下一阶段（VINS + EGO-Planner 路径规划）准备"阶段。

已经具备的能力：

- 自然语言或 JSON 指令转飞行动作
- PX4 Offboard 执行闭环
- **12 个动作协议**（含 `FIND_AND_GOTO` 视觉语义搜索 action）
- 多步 `plan` 顺序执行
- TUI 状态监控
- 回归测试
- YOLO-World ROS 2 实时检测（**GPU 推理**，torch cu128 / CUDA 12.9 已验证；`/uav/target_query` 动态 prompt 或静态 `texts` 参数）
- 深度图几何 Grounding（bbox 区域中位数深度 + `camera_info` 逆投影）
- camera frame → world frame 目标点转换（ENU 输出）
- `/uav/target_goal` 语义目标发布与 RViz marker 可视化
- **LLM → `FIND_AND_GOTO` → YOLO-World 检测 → ENU→NED 转换 → `GOTO_NED` 自动飞行**（视觉任务端到端闭环已接通，等待场景联调验证）

尚未完成的能力（下一阶段目标）：

- **VINS-Fusion** 位姿估计与 world frame 对齐（多传感器状态估计，替代当前 AirSim `odom_local_ned`）
- **EGO-Planner** 与 VINS 位姿驱动的完整局部轨迹规划闭环（当前 FIND_AND_GOTO 为直线 GOTO_NED，无避障）
- 复杂属性 prompt 的泛化精度验证（不同场景、目标遮挡、多目标选择）

已部分接入（仿真侧，可选）：

- **EGO-Planner（ROS 2）**：见 `planner_integration.launch.py`、`SESSION_HANDOVER.md` 与 PRD §13；目前以深度图 + `odom_local_ned` 驱动，完整路线下将切换至 VINS-Fusion 位姿驱动；与 `text_command_test` 二选一启动，勿双开。

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

建议至少打开 5 个终端。

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

### 终端 C：主飞控链

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py
```

预期信号：

- `Text command bridge ready on /uav/user_command`

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

系统行为：LLM 解析 → `FIND_AND_GOTO query="..."` → bridge 发布 `/uav/target_query` → YOLO-World GPU 检测 → 深度 grounding → world 坐标 → ENU→NED → `GOTO_NED` → 飞行到目标。

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
3. 将 FIND_AND_GOTO 内部的 GOTO_NED 替换为 EGO-Planner 避障规划（当前无避障）
4. 接入 VINS-Fusion 替换 `odom_local_ned`，提升 world 坐标精度
5. 多目标场景下 FIND_AND_GOTO 目标选择策略优化

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

当前目标技术闭环为：

```
用户自然语言指令
  → LLM 任务解析（生成 target_category / target_attribute / 任务动作）
  → 视觉任务协议生成 `/uav/target_query`
  → YOLO-World 开放词汇目标检测（输出 bbox）
  → AirSim 深度图 + 相机内参逆投影（camera frame 3D 点）
  → AirSim odom / 后续 VINS-Fusion 位姿 → world frame 对齐
  → `/uav/target_goal` → EGO-Planner 局部轨迹规划
  → PX4 执行
```

当前已落地的语义感知链保持独立 ROS 2 节点，不混入现有 `llm_client.py` / `text_command_bridge.py`。

### 8.1 视觉语义识别层（YOLO-World）

当前已实现独立节点 `yolo_world_detector.py`。

职责：
- 订阅 AirSim RGB 图像（默认 `/airsim_node/PX4/CameraDepth1/Scene`）
- 接收来自任务理解层的文本 prompt（`/uav/target_query`）或静态参数 `texts`
- 调用 **YOLO-World** 进行开放词汇目标检测
- 输出 `/uav/detections_2d`（JSON：`label`、`score`、`bbox`、`stamp`、`frame_id`）

YOLO-World 选型说明：支持 `prompt-then-detect` 范式，将词汇嵌入重参数化进模型权重，推理效率接近标准 YOLO，同时支持任意文本描述的开放类别检测，适合 `"yellow clothes person"`、`"person"`、`"car"`、`"vehicle near building"` 等场景，不应把系统目标限制在固定 `red car`。

```bash
# 仅验证检测节点
ros2 run hw_insight yolo_world_detector --ros-args -p texts:="person"

# 推荐直接启动整条语义感知链
ros2 launch hw_insight semantic_perception.launch.py texts:="person"
```

### 8.2 几何 Grounding 层（深度图 + 逆投影 + 坐标变换）

当前由 `target_grounding_node.py` 独立负责。

关键实现要点：

1. **深度类型选定**：固定使用 `DepthPlanar` 或 `DepthPerspective` 之一，并保证逆投影公式与之匹配（二者几何含义不同，不可混用）。
2. **稳定深度提取**：根据 YOLO-World 输出的 bbox 提取对应深度区域，去除无效值后取**中位数**作为鲁棒深度估计。
3. **像素 → 相机系 3D**：基于相机内参 `(fx, fy, cx, cy)` 完成逆投影，得到目标在 camera frame 下的三维坐标。
4. **坐标系变换链**：`camera frame → body frame（固定外参）→ world frame（当前先用 AirSim odom，后续切 VINS-Fusion）`。

```bash
# 检查 grounding 输出
ros2 topic echo /uav/semantic_targets_camera --once
```

### 8.3 状态估计层（VINS-Fusion）

当前由 `semantic_target_tf_node.py` 使用 AirSim `odom_local_ned` 先完成 world 点验证；后续再切换到 VINS-Fusion。

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

EGO-Planner 是面向四旋翼的 ESDF-free、gradient-based 局部规划器，输入为当前位姿、局部障碍信息和目标 world 坐标，输出局部可飞行轨迹。

当前已提供最小接入链路（ROS 2）：

- 目标输入：`text_command_bridge` 在 `GOTO_NED` 时发布 `/uav/target_goal`（`PoseStamped`）
- 规划输出：`planner_velocity_bridge` 订阅 `/uav/planner_cmd_vel`（`Twist`）或 `/uav/planner_cmd_vel_stamped`（`TwistStamped`）
- 执行入口：桥接后统一发布到 `/hw_insight/keyboard_velocity`，复用现有 `move_velocity`

Phase 2 当前已支持由 `semantic_target_tf_node.py` 或 `semantic_goal_to_planner.py` 发布 `/uav/target_goal`。两者二选一，避免双写。

```bash
# 启动规划接入模式（当前仿真可用）
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight planner_integration.launch.py use_rviz:=true

# 快速自测：手动发布目标点
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
"{data: '{\"action\":\"GOTO_NED\",\"params\":{\"x\":20.0,\"y\":5.0,\"altitude\":8.0}}'}"

# 模拟 planner 输出速度
ros2 topic pub /uav/planner_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### 8.5 执行层（保持不变）

继续保持现有设计原则：

- LLM 负责高层任务语义与动作规划
- EGO-Planner 负责局部轨迹生成，不替代 LLM 的语义理解
- 执行层负责 setpoint 与 Offboard 心跳（`move_velocity.py`）
- 安全层独立于 LLM 存在，运行期持续检查碰撞风险、地理围栏、低电量等异常状态
- 当前缺口不在检测节点本身，而在 `LLM -> 视觉任务协议 -> prompt / target_goal` 这一段

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

- 想做“飞到穿黄色衣服人的头顶”  
  当前检测链和空间 grounding 已具备基础，但还需要补上 `LLM -> 视觉任务协议 -> /uav/target_query -> /uav/target_goal` 的动作与消息设计，才能从自然语言任务稳定驱动感知与规划。
