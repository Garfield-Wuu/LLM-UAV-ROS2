# 会话交接文档

> **用途**：将全部开发会话的变更、当前状态、待续任务完整记录，供下一个对话无缝继续。  
> **最后更新**：2026-03-27（第八次：YOLO-World 本地部署完成，推理验证通过）  
> **关联 PRD**：`PRD_text_command_flight_mvp.md`（v5.2）  
> **历史会话 ID**：
> - 第一次：`bc9aee3a-a535-45ff-ac7a-90ed74709c37`（v1.0~v2.0 基础实现）
> - 第二次：同上延续（LLM 推理层接入）
> - 第三次：LLM 联调修复 + 坐标系修正 + 多步顺序执行
> - 第四次：`58a7005f-b681-4d1c-9553-6da32b3d45d3`（EGO-Planner、栅格地图、RViz、WSL2 DDS、AirSim 相机）
> - 第五次：`llm_client.py` 交互式 provider/model 选择、Ollama warm-up、`<think>` 日志、鲁棒 JSON 提取、文档同步
> - 第六次：技术路线锁定（YOLO-World + VINS-Fusion + EGO-Planner）；三份核心文档全面同步
> - 第七次：Git 仓库初始化 `/home/hw/hw-ros2/`；SSH 配置；推送至 GitHub `Garfield-Wuu/LLM-UAV-ROS2`；README 编写
> - 第八次：当前（YOLO-World 本地部署；依赖安装与 patch；CLIP 模型离线化；推理测试脚本；CPU/GPU 验证）

---

## 1. 系统概述

**项目名称**：基于 LLM 的低空经济无人机自主决策系统（MVP 阶段）

**技术栈**：

| 层 | 技术 | 状态 |
|----|------|------|
| 仿真 | AirSim + Unreal Engine（WSL2 环境）| ✅ 运行中 |
| 飞控 | PX4 SITL（`/home/hw/px4v1.15.2`）| ✅ 运行中 |
| 中间件 | ROS 2 Humble | ✅ 运行中 |
| 桥接 | uXRCE-DDS（`MicroXRCEAgent udp4 -p 8888`）| ✅ 运行中 |
| LLM | Groq API / Ollama（含远程服务；默认可读 `ANTHROPIC_BASE_URL` / `ANTHROPIC_MODEL`）| ✅ **已接入并增强** |
| 视觉语义识别 | **YOLO-World**（open-vocabulary 目标检测，`prompt-then-detect` 范式）| ⚠️ 本地部署完成，ROS 2 集成待开发 |
| 几何 Grounding | AirSim DepthPlanar/DepthPerspective + 逆投影 + tf2 坐标变换链 | 📋 规划中 |
| 状态估计 | **VINS-Fusion**（多传感器优化估计，提供 world frame 位姿）| 📋 规划中 |
| 局部轨迹规划 | **EGO-Planner**（ESDF-free gradient-based，VINS 位姿驱动）| ⚠️ 仿真部分接入 |
| 地面站 | QGroundControl（可选监控）| 可选 |

**工作目录**：`/home/hw/hw-ros2/ros2/src/hw_insight/`  
**PX4 目录**：`/home/hw/px4v1.15.2`（注意：不是 `~/px4`）  
**Git 仓库**：`/home/hw/hw-ros2/`（根目录）→ remote: `git@github.com:Garfield-Wuu/LLM-UAV-ROS2.git`  
**密钥**：Groq 使用 `GROQ_API_KEY`；勿将密钥提交仓库。远程 Ollama 可沿用 `ANTHROPIC_BASE_URL`、`ANTHROPIC_MODEL`、`ANTHROPIC_API_KEY`、`ANTHROPIC_AUTH_TOKEN`。当前建议把这些变量写入 `~/.bashrc`，避免每次开新终端后丢失。

---

## 1.1 目标系统完整技术路线（Phase 2 主线）

本系统的最终目标是构建"语言目标 → 视觉识别 → 几何 Grounding → world frame 目标 → 局部规划 → 飞行执行"的完整闭环。下表为各层技术选型与职责划分：

| 层级 | 技术选型 | 核心职责 |
|------|---------|---------|
| 任务理解层 | LLM（Groq / Ollama） | 自然语言 → 结构化任务描述（target_category / target_attribute / action） |
| 视觉语义识别层 | **YOLO-World** | open-vocabulary 目标检测，输出 bbox / 置信度 |
| 几何 Grounding 层 | AirSim DepthPlanar + 逆投影 + tf2 | 2D bbox → camera frame 3D 点；中位数深度鲁棒估计 |
| 坐标系变换层 | tf2（camera→body→world） | 固定外参 + 实时位姿完成坐标系对齐 |
| 状态估计层 | **VINS-Fusion** | world/map frame 高精度位姿，用于目标世界坐标生成 |
| 轨迹规划层 | **EGO-Planner** | `target_position_world` → 局部可飞轨迹（ESDF-free，梯度优化） |
| 控制执行层 | PX4 SITL + move_velocity | Offboard setpoint 跟踪，飞控底层执行 |

**完整数据流**：

```
用户自然语言指令
  │
  ▼
llm_client.py（任务理解）
  ├── 输出结构化任务描述：task_type / target_category / target_attribute
  └── → /uav/target_query (text prompt, e.g. "red car")
  │
  ▼
vision_grounding_node（视觉语义识别 + 几何 Grounding）
  ├── 订阅 /airsim_node/PX4/Scene（RGB）
  ├── YOLO-World 检测 → bbox（像素坐标）
  ├── 订阅 /airsim_node/PX4/CameraDepth1/DepthPlanar（深度图）
  ├── bbox 区域中位数深度 + 相机内参逆投影 → camera frame 3D 点
  └── tf2 变换（camera→body→world，依赖 VINS-Fusion 位姿）
  │
  ▼
target_position_world = (x, y, z)
  │
  ├─→ /uav/target_goal（PoseStamped，发给 EGO-Planner）
  │
  ▼
ego_planner_node（局部轨迹规划）
  ├── 输入：target_goal + odom（VINS-Fusion） + depth points
  └── 输出：/uav/ego_planner/bspline → TwistStamped → keyboard_velocity
  │
  ▼
move_velocity.py → PX4 SITL → AirSim
```

**坐标系与深度类型注意事项**：

- 深度类型须固定使用 `DepthPlanar`（相机平面深度）或 `DepthPerspective`（沿射线方向深度）之一；逆投影公式必须与之匹配，不可混用。
- 内部规划使用统一坐标系（ENU / world frame），与 AirSim / PX4 交互时集中做 NED/ENU 转换。
- VINS-Fusion 仿真验证阶段可先用 `odom_local_ned` 替代，接入 VINS 后只需 remap，不改节点逻辑。

**论文表述建议**：

> 本文构建了一套面向自然语言任务执行的无人机自主决策系统。系统首先利用大语言模型对用户指令进行语义解析，生成目标类别、属性与动作需求等结构化任务描述；随后，视觉语义识别模块采用 YOLO-World 实现开放词汇目标检测，从机载 RGB 图像中识别符合语言描述的目标实体；在此基础上，结合 AirSim 深度图像与相机成像模型，完成目标的三维几何 grounding，并借助 VINS-Fusion 输出的位姿信息将目标从相机坐标系对齐到世界坐标系；最后，将目标 world frame 坐标输入 EGO-Planner，生成局部可飞行轨迹，并由飞控系统完成轨迹跟踪与任务执行。

---

## 2. 全量文件变更清单（累计至第七次会话）

### 2.1 新增文件（Created）

| 文件 | 创建会话 | 说明 |
|------|---------|------|
| `hw_insight/text_command_bridge.py` | S1 | 核心桥接：文本/JSON → 速度指令，11 个 action |
| `hw_insight/gcs_dashboard.py` | S1 | 地面站 TUI（≤15 行，4Hz 刷新）|
| `hw_insight/flight_regression_runner.py` | S1 | 闭环回归测试 |
| `hw_insight/llm_client.py` | S2 | LLM 推理节点：NL → Groq/Ollama → `/uav/user_command` |
| `launch/text_command_test.launch.py` | S1 | 飞控链启动入口（airsim + move_velocity + bridge）|
| `launch/gcs_dashboard.launch.py` | S1 | 独立启动地面站 TUI |
| `COMMAND_PROTOCOL.md` | S1 | 指令协议文档 v2.0（11 个 action + 状态 schema）|
| `PRD_text_command_flight_mvp.md` | S1 | PRD 文档（持续更新至 v3.0）|
| `README_text_command_test.md` | S1 | 操作手册 |
| `PRODUCT_TEST_FLOW.md` | S1 | 产品化测试流程 & 验收矩阵 |
| `SESSION_HANDOVER.md` | S1 | **本文档** |
| `launch/planner_integration.launch.py` | S4 | AirSim + 飞控桥 + `ego_planner` 集成入口；`depth_image_proc`；`world`→`PX4` 静态 TF |
| `launch/ego_planner_integration.launch.py` | S4 | `ego_planner_node` 参数、remap、RViz、`enable_velocity_bridge` |
| `hw_insight/planner_velocity_bridge.py` | S4 | Planner Twist → `HWSimpleKeyboardInfo` / `keyboard_velocity` |
| `hw_insight/ego_bspline_to_twist_relay.py` | S4 | `traj_utils/Bspline` → `TwistStamped`；超时不发零速 |
| `hw_insight/test_planner_feedback.py` | S4 | 目标→B-spline 延迟粗测 |
| `rviz/ego_planner_debug.rviz` | S4 | 栅格/轨迹/RGB/深度；性能向配置（AxisColor、DepthPoints 默认关等） |
| `config/mapping_config.yaml` | S4 | 话题与帧映射说明 |
| `docs/ego_planner_feasibility_report.md` | S4 | 第三方 ROS2 ego-planner 方案评估 |
| `docs/integration_log_v1.md` | S4 | 集成与排障流水账 |
| `../external/ego_planner_core/`（相对 `hw_insight` 包：工作空间 `ros2/src/external/ego_planner_core/`）| S4 | 精简编译的 Ego-Planner 核心包；上游整库 `COLCON_IGNORE` |
| `/home/hw/hw-ros2/.gitignore` | S7 | Git 忽略规则：排除 build/install/log、第三方 ROS2 包、AirSim C++ 库、rpclib |
| `/home/hw/hw-ros2/README.md` | S7 | 项目中文 README：技术路线、完整工程结构（含第三方依赖 clone 地址）、快速开始 |
| `/home/hw/YOLO-World/test_yolo_world.py` | S8 | YOLO-World 推理测试脚本；CPU/GPU 自动适配；自定义词汇表；结果输出到 `outputs/` |
| `/home/hw/YOLO-World/weights/yolo_world_v2_s_stage1.pth` | S8 | YOLO-World-S 预训练权重（305MB，YOLO-World-V2.1，HuggingFace 下载） |
| `/home/hw/YOLO-World/clip_tokenizer/` | S8 | CLIP ViT-B/32 完整模型离线化（tokenizer + pytorch_model.bin）|

### 2.2 修改文件（Modified）

| 文件 | 会话 | 修改内容 |
|------|------|---------|
| `hw_insight/move_velocity.py` | S1 | 新增 `flight_enabled` 安全闸门；落地自动重置；else 分支维持零速 |
| `hw_insight/move_velocity.py` | S3 | **`msg.yawspeed = -yawspeed`** — 修复 AirSim PX4 偏航方向反转 |
| `hw_insight/text_command_bridge.py` | S1 | 11 个 action 全量实现；TAKEOFF ARM 阶段；TELEMETRY 扩展 |
| `hw_insight/text_command_bridge.py` | S3 | `ARM_TRIGGER_SEC` 0.8→2.5s（WSL2 时间同步缓解）；MOVE_VELOCITY 加入机体→NED 坐标变换 |
| `hw_insight/llm_client.py` | S2 | 首次创建：Groq+Ollama 双模，stdin+话题双输入，安全层，自动 TAKEOFF 插入 |
| `hw_insight/llm_client.py` | S3 | **① User-Agent 修复**（curl/7.81.0，绕过 Cloudflare 403）；**②** position/velocity null 守卫；**③** plan 格式多步顺序执行（`_extract_plan` / `_execute_plan` / `_wait_command_done`）；**④** System Prompt 轴方向表 + 机体坐标系说明；max_tokens 150→500 |
| `setup.py` | S2 | 注册 `llm_client` console_script |
| `package.xml` | S1 | 新增 exec_depend |
| `COMMAND_PROTOCOL.md` | S1 | v1.0→v2.0（11 个 action）|
| `hw_insight/text_command_bridge.py` | S4 | `GOTO_NED`：`/uav/target_goal`；`planner_mode_for_goto` 等参数 |
| `hw_insight/move_velocity.py` | S4 | `command_topic` 参数（便于 remap） |
| `hw_insight/llm_client.py` | S4 | 默认 `ollama_host` / `ollama_model` 可读 `ANTHROPIC_BASE_URL` / `ANTHROPIC_MODEL` |
| `hw_insight/llm_client.py` | S5 | **①** 启动前方向键选择 `Groq` / `Ollama` 与具体模型（Groq 支持自定义模型） **②** Ollama 冷启动 warm-up 后再开放输入 **③** 动态读取 `/api/tags` 获取已安装 Ollama 模型，失败回退预设 **④** 遥测离线拦截与状态感知提示符 **⑤** `<think>` 推理摘要打印并写入 ROS2 log **⑥** 动作别名库 + 平衡括号 / 正则 JSON 提取 + 参数 schema 强制转换 **⑦** Groq `response_format=json_object` / Ollama `format=json` |
| `hw_insight/setup.py` | S4 | 新入口脚本；`launch/*.py` 打包；`config` |
| `package.xml` | S4 | `geometry_msgs`、`traj_utils`、`sensor_msgs_py` 等 |
| `PRD_text_command_flight_mvp.md` | S4~S5 | v4.2：规划/建图状态、LLM 交互终端增强、§13 集成记录 |
| `SESSION_HANDOVER.md` | S4 | 本文档持续更新 |
| `external/ego_planner_core/planner/ego_planner/src/ego_replan_fsm.cpp` | S4 | 去掉回调内 `spin_some`；无 odom 时忽略 goal |
| `external/ego_planner_core/planner/plan_env/src/library/grid_map.cpp` | S4 | `proj_points_` 缓冲区；**`cam2body_` 光学→NED + 相机安装（须与 AirSim 一致）** |
| Windows `Documents/AirSim/settings.json` | S4 | `CameraDepth1`：FOV、分辨率、X/Y/Z、Pitch（相机位姿） |
| `/home/hw/.local/lib/python3.10/site-packages/mmengine/optim/optimizer/builder.py` | S8 | patch：`Adafactor` 重复注册修复（torch 2.11 + mmengine 0.10.3 兼容） |
| `/home/hw/YOLO-World/yolo_world/models/detectors/yolo_world.py` | S8 | patch：`self.text_feats, None = ...` → `self.text_feats, _ = ...`（SyntaxError 修复） |

### 2.3 删除文件（Deleted）

| 文件 | 原因 |
|------|------|
| `launch/llm_flight.launch.py` | S3 删除：`ros2 launch` 会捕获 stdin 导致 llm_client 无法交互；删除后统一使用 `text_command_test.launch.py`，llm_client 独立终端运行 |

### 2.4 未修改文件（Unchanged）

`keyboard_position.py` / `keyboard_velocity.py` / `move_position.py` / `offboard.py` / `px4_test.py` / `lesson3.launch.py` / `lesson4.launch.py` / `msg_px4_fmu_out_vehicle_status.py` 等原有文件均未改动。

---

## 3. 架构现状（当前数据流）

```
用户自然语言（中文/英文）
    │  stdin（交互式终端）或 /uav/nl_input (std_msgs/String)
    ▼
llm_client.py  ←── TELEMETRY context ←──────────────────────────────┐
    ├── build_prompt（状态 + 11 action + 安全约束 + 机体坐标说明）      │
    ├── 启动前方向键选择 provider/model；Ollama 先 warm-up 再开放输入   │
    ├── 调用 Groq API（JSON mode）或 Ollama（format=json）             │
    ├── `<think>` 摘要打印 + 完整推理写入 ROS2 log                     │
    ├── _extract_plan / _extract_json_robust：平衡括号 / 正则兜底      │
    ├── _execute_plan：顺序发布，_wait_command_done 等待每步完成       │
    ├── 安全层：离线遥测拦截 + 高度/速度截断 + 未知 action→HOVER       │
    └── 发布 JSON → /uav/user_command                                 │
                                                                      │
    │  /uav/user_command  (std_msgs/String, JSON)                     │
    ▼                                                                  │
text_command_bridge.py ─────────────── /uav/llm_task_status ─────────┘
    ├── 解析 JSON / 中文文本
    ├── 11 个 action dispatch
    ├── MOVE_VELOCITY：机体坐标 vx/vy → NED（cos/sin heading 旋转）
    ├── 10Hz 控制环（TAKEOFF althold / GOTO_NED / ORBIT / YAW_TO）
    ├── 直发 VehicleCommand（RTL / EMERGENCY_STOP）
    └── 发布 TELEMETRY（5Hz，含 position/velocity/heading）
         │
         │  /hw_insight/keyboard_velocity (HWSimpleKeyboardInfo)
         ▼
move_velocity.py
    ├── flight_enabled 安全闸门
    ├── z < -5 → arm() + engage_offboard_mode()
    ├── yawspeed 取反（AirSim PX4 符号约定修正）
    └── 落地 DISARMED → flight_enabled 自动重置
         │
         │  /fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}
         ▼
PX4 SITL → AirSim

### 3.1 规划接入模式（`planner_integration.launch.py`，可选）

```
AirSim RGB/Depth + odom_local_ned
    │
    ├── depth_image_proc → /uav/camera/points（辅助；QoS reliable）
    ├── ego_planner_node ← depth + odom（grid_map/*）
    │       └── /grid_map/occupancy(_inflate) → RViz
    │       └── /uav/ego_planner/bspline
    └── ego_bspline_to_twist_relay → /uav/planner_cmd_vel_stamped
              └── planner_velocity_bridge → /hw_insight/keyboard_velocity → move_velocity

/uav/target_goal ← text_command_bridge（GOTO_NED，可选 planner 模式）
```

静态 TF：`world` → `PX4`（identity）。WSL2 建议：`export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

---

/uav/llm_task_status  ──→  gcs_dashboard.py (TUI, 4Hz)
/fmu/out/vehicle_*    ──→  gcs_dashboard.py (TUI)
/fmu/out/vehicle_*    ──→  flight_regression_runner.py (闭环验证)
```

---

## 4. 当前 Action 协议全集（11 个）

| # | Action | 状态 | 主要参数 | 坐标系 | 控制算法 |
|---|--------|------|---------|--------|---------|
| 1 | `TAKEOFF` | ✅ | `altitude` (m) | ENU alt | ARM 阶段(2.5s, z=-6) + P 高度保持 |
| 2 | `LAND` | ✅ | — | — | z=+8 → PX4 AUTO_LAND |
| 3 | `HOVER` | ✅ | `duration` (s) | — | 零速度保持 |
| 4 | `MOVE_VELOCITY` | ✅ | `vx/vy/vz/yaw_rate/duration` | **机体坐标** | 机体→NED 变换后发布 |
| 5 | `MOVE_REL` | ✅ | `dx/dy/dz/duration` | **机体坐标** | dx/dy→vx/vy→机体→NED |
| 6 | `GOTO_NED` | ✅ | `x/y/altitude` | NED 世界坐标 | 10Hz P控制 vx/vy/vz，≤0.5m 完成 |
| 7 | `ORBIT` | ✅ | `cx/cy/radius/speed/duration` | NED 世界坐标 | 切向+半径误差 P 修正 |
| 8 | `YAW_TO` | ✅ | `angle` (deg, 0=North CW) | NED | 10Hz P控制 yaw_rate，≤3° 完成 |
| 9 | `RTL` | ✅ | — | — | 直发 VEHICLE_CMD_NAV_RETURN_TO_LAUNCH |
| 10 | `EMERGENCY_STOP` | ✅ | — | — | 直发 VEHICLE_CMD_COMPONENT_ARM_DISARM(0) |
| 11 | `SET_SPEED` | ✅ | `speed` (m/s, 0.5-15) | — | 更新 default_speed_xy/z |

**坐标系说明**：
- `MOVE_VELOCITY`/`MOVE_REL`：vx/dx = 机头方向，vy/dy = 机身右方（机体坐标），vz/dz = NED 垂直（正=下，负=上）
- `GOTO_NED`/`ORBIT`：x=北, y=东（NED 世界坐标），altitude 正=上（ENU）
- 转换在 `text_command_bridge.py` 的 MOVE_VELOCITY dispatch 中完成

---

## 5. PRD 完成情况（v4.2）

### v1.0 原始目标（全部完成）

| 里程碑 | 验收标准 | 状态 |
|--------|---------|------|
| M1 状态反馈话题 | `/uav/llm_task_status` 可见，0.5s 内回传 | ✅ |
| M2 回归序列测试 | `flight_regression_runner` 闭环通过 | ✅ |
| M3 README 手册 | `README_text_command_test.md` 完成 | ✅ |

### v2.0 追加目标（全部完成）

| 里程碑 | 验收标准 | 状态 |
|--------|---------|------|
| M4 安全闸门 | launch 后不自动解锁 | ✅ |
| M5 TAKEOFF 主动解锁 | ARM 阶段 z=-6.0 | ✅ |
| M6 TUI 紧凑化 | ≤15 行，飞行阶段标签 | ✅ |
| M7 闭环回归验证 | 等待真实物理条件 | ✅ |
| M8 action 扩展 | 11 个 action 全实现 | ✅ |

### v3.0 本次会话新增目标（全部完成）

| 里程碑 | 验收标准 | 状态 |
|--------|---------|------|
| M9 LLM 接入并联调 | Groq API 联通，`起飞/前进/返航` 正确转换为 JSON 发布 | ✅ |
| M10 多步顺序执行 | plan 格式输出，按序等待每步完成后再执行下一步 | ✅ |
| M11 运动方向修正 | 前进/后退/左移/右移/左旋/右旋 与实际运动一致 | ✅ |
| M12 WSL2 解锁稳定性 | ARM 窗口延长到 2.5s，时间同步波动期间解锁成功率提升 | ✅ |

### v4.0~v4.2 第四至第五次会话（Ego-Planner / 建图 / LLM 终端增强）

| 里程碑 | 验收标准 | 状态 |
|--------|---------|------|
| M13 Ego-Planner 编译与启动 | `colcon build` 通过；`ros2 node list` 可见 `/ego_planner_node` | ✅ |
| M14 深度→占用栅格 | RViz 可见障碍物体素；NED 地图 Z 范围与地面过滤参数合理 | ✅（仿真） |
| M15 规划→执行桥 | B-spline 或速度桥可驱动 `move_velocity`（注意与手飞/LLM 抢占） | ⚠️ 仿真验证 |
| M16 AirSim 相机与内参一致 | `settings.json` 与 `grid_map/fx,fy,cx,cy`、`cam2body_` 同步 | ✅（须改相机后重启 UE） |

---

## 6. Bug & 修复完整记录

| # | Bug | 根因 | 修复文件 | 会话 | 状态 |
|---|-----|------|---------|------|------|
| 1 | launch 即自动解锁 | `move_velocity` 无条件广播心跳 | `move_velocity.py` → `flight_enabled` | S1 | ✅ |
| 2 | TAKEOFF 解锁链路断裂 | altitude hold 限幅永不触发 z<-5 | `text_command_bridge.py` → ARM 阶段 | S1 | ✅ |
| 3 | 回归测试假通过 | 只看 ACK 不等物理 | `flight_regression_runner.py` | S1 | ✅ |
| 4 | TUI 行数过多 | 两大表格 30+ 行 | `gcs_dashboard.py` 完全重写 | S1 | ✅ |
| 5 | Groq API HTTP 403 | Cloudflare 拦截 Python urllib UA | `llm_client.py` → `User-Agent: curl/7.81.0` | S3 | ✅ |
| 6 | llm_client AttributeError crash | TELEMETRY position 字段有时为 null | `llm_client.py` → `or {}` null 守卫 | S3 | ✅ |
| 7 | stdin 无法输入（ros2 launch 捕获） | launch 子进程不继承 stdin | 删除 `llm_flight.launch.py`，llm_client 独立终端 | S3 | ✅ |
| 8 | 多步指令同时叠加执行 | MOVE_REL 支持 dx/dy/dz 同步，LLM 合并 | `llm_client.py` → plan 格式 + `_execute_plan` | S3 | ✅ |
| 9 | Z 轴方向错误（上升变下降） | System Prompt 未明确 dz 符号 | `llm_client.py` → 轴方向表 | S3 | ✅ |
| 10 | 所有平移方向错误 | NED 世界坐标 ≠ 机体坐标，drone 朝向≠北 | `text_command_bridge.py` → 机体→NED 旋转变换 | S3 | ✅ |
| 11 | 偏航方向反转 | AirSim PX4 yawspeed 符号与标准 NED 相反 | `move_velocity.py` → `yawspeed = -yawspeed` | S3 | ✅ |
| 12 | 频繁解锁失败 `Arming denied` | WSL2 时钟跳变导致 PX4 时间同步失效 | `text_command_bridge.py` ARM_TRIGGER_SEC 0.8→2.5 | S3 | ⚠️ 缓解（非根治）|
| 13 | `ego_planner_node` executor 崩溃 | 回调内 `spin_some` | `ego_replan_fsm.cpp` 移除阻塞 spin | S4 | ✅ |
| 14 | 深度 800×600 投影 segfault | `proj_points_` 容量按 640×480 写死 | `grid_map.cpp` 扩大缓冲区 | S4 | ✅ |
| 15 | 起飞后栅格消失 / “odom or depth lost” | NED 下 `ground_height`+`map_size_z` 不包含飞行高度；timeout 过短；QoS 不一致 | launch 参数 + `odom_depth_timeout`；`depth_image_proc` reliable | S4 | ✅ |
| 16 | RViz 深度全黑 | 32FC1 当 0–255 显示 | RViz Image `Normalize Range: false` + 固定 Max | S4 | ✅ |
| 17 | 无 B-spline 仍发零速 | relay 持续发 0 | `ego_bspline_to_twist_relay` 超时停止发布 | S4 | ✅ |
| 18 | 体素大平面 / 与画面对不上 | 120° FOV + 相机下倾 + `cam2body` 未建模 | AirSim FOV/位姿 + `cam2body_` + 内参同步 | S4 | ✅ |
| 19 | FastDDS SHM 权限 | WSL2 共享内存 | `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | S4 | ✅ |

> **注**：Bug 12 的根治方案是在 Windows 端运行 `w32tm /resync /force` 修复时间同步。

---

## 7. 标准启动流程（操作员 SOP）

> **重要**：PX4 目录为 `/home/hw/px4v1.15.2`，不是 `~/px4`

```
终端分配：
  T1  AirSim（Windows UE Play 按钮；修改 settings.json 后须冷启动）
  T2  PX4 SITL
  T3  uXRCE-DDS 桥接
  T4  ROS2 飞控底层（launch）— 二选一：
        • 仅手飞/LLM：`text_command_test.launch.py`
        • 规划+建图+RViz：`planner_integration.launch.py use_rviz:=true`
  T5  LLM 交互终端（独立，有 ▶ 提示符）
  T6  地面站 TUI（可选）
```

```bash
# ── T2：PX4 SITL ──────────────────────────────────
cd /home/hw/px4v1.15.2
make px4_sitl_default none_iris
# 等待出现 "Ready for takeoff!"

# ── T3：uXRCE-DDS 桥接 ────────────────────────────
MicroXRCEAgent udp4 -p 8888
# 等待出现大量 create_topic 日志

# ── T4a：ROS2 飞控底层（仅手飞 / LLM，默认）──────────
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py

# ── T4b：规划 + 建图 + RViz（与 T4a 二选一，勿双开冲突）──
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight planner_integration.launch.py use_rviz:=true
# 约 10s 后：ros2 node list | grep ego_planner

# ── T5：LLM 交互终端（必须独立终端！）──────────────
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# 建议先将以下变量写入 ~/.bashrc，再 source ~/.bashrc：
#   export GROQ_API_KEY="gsk_..."
#   export ANTHROPIC_BASE_URL="http://<host>:<port>"
#   export ANTHROPIC_MODEL="qwen3-coder:30b"
#   export ANTHROPIC_API_KEY="ollama"
#   export ANTHROPIC_AUTH_TOKEN="ollama"
ros2 run hw_insight llm_client
# 启动后：
#   1) 用方向键选择 Groq / Ollama 与模型
#   2) 若为 Ollama，等待 warm-up 完成
#   3) 出现状态提示符（如 [离线] ▶ / [HOVERING|解锁] ▶）后再输入指令

# ── T6：地面站 TUI（可选）────────────────────────────
cd ~/hw-ros2/ros2 && source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0
```

**规划模式快速检查**：

```bash
ros2 topic hz /airsim_node/PX4/CameraDepth1/DepthPlanar
ros2 topic echo /grid_map/occupancy --once   # 有订阅者且地图更新时才有数据
```

**T5 交互示例**：
```
▶ 起飞到8米
[→ UAV] TAKEOFF          {'altitude': 8.0}
▶ 向前飞20米
[→ UAV] MOVE_REL         {'dx': 20, 'dy': 0, 'dz': 0, 'duration': 5}
▶ 以5米半径绕当前位置盘旋30秒
[→ UAV] ORBIT            {'radius': 5.0, 'speed': 2.0, 'duration': 30.0}
▶ 先上升5米，再向右飞10米，然后降落
[计划 1/3] MOVE_REL  {'dx': 0, 'dy': 0, 'dz': -5, 'duration': 2.5}
[计划 2/3] MOVE_REL  {'dx': 0, 'dy': 10, 'dz': 0, 'duration': 2.5}
[计划 3/3] LAND      {}
▶ 返航
[→ UAV] RTL              {}
```

**Ollama 本地模式**（无需 API Key）：
```bash
ollama serve & && ollama pull llama3.2
ros2 run hw_insight llm_client \
  --ros-args -p llm_provider:=ollama -p ollama_model:=llama3.2
```

**手动 JSON 测试**（无需 LLM）：
```bash
source ~/hw-ros2/ros2/install/setup.bash
ros2 topic pub --once /uav/user_command std_msgs/msg/String \
  "{data: '{\"action\":\"TAKEOFF\",\"params\":{\"altitude\":6.0}}'}"
```

**一键回归测试**：
```bash
ros2 run hw_insight flight_regression_runner
```

---

## 8. 下期待续任务（Backlog）

### ✅ P0 — LLM 接入（已完成）

Groq API 联通验证，`起飞/前进/返航/盘旋` 等指令完整链路测试通过，方向问题全部修复。

### ✅ P1 — 多步顺序执行（已完成）

plan 格式已实现。LLM 可输出 `{"plan": [...]}` 多步计划，节点按序执行，每步完成后（基于 duration 或 TELEMETRY command=IDLE）才触发下一步。

### P2（下期优先）— 告警分级 + 测试报告

- TUI 加入 WARN/ERROR 分级，关键事件闪烁
- `flight_regression_runner` 输出 JSON/Markdown 报告
- GPS 坐标导航（`GOTO_GPS`，需 GPS→NED 转换）
- 评估将 `<think>` 推理日志同步为独立审计 topic / 文件，而不是仅写 ROS2 log

### P3（长期）— 持续对话上下文

当前 LLM 每次调用无历史上下文（stateless）。后续可加入对话历史（最近 N 轮），支持"刚才那个动作再做一次"等指代。

### P5（进行中）— Phase 2 完整闭环：视觉语义识别 + 几何 Grounding + 位姿对齐 + 局部规划

#### P5-A：YOLO-World 检测接口

> **S8 已完成**：YOLO-World 本地部署与推理验证，see `/home/hw/YOLO-World/test_yolo_world.py`

- ✅ YOLO-World-S 模型部署（权重 + CLIP 离线化，CPU/GPU 自动适配，推理耗时 CPU≈4–6s）
- ✅ 开放词汇推理验证（bus.jpg 3人+1公交，zidane.jpg 2领带+1人）
- 🔲 新建 `vision_grounding_node.py`，将推理接口包装为 ROS 2 订阅/发布节点
- 🔲 输入：`/airsim_node/PX4/Scene`（RGB）+ `/uav/target_query`（文本 prompt）
- 🔲 输出：`/uav/detection_bbox`（像素坐标 bbox + 置信度）
- 🔲 验收：输入 `"red car"` 能在 AirSim RGB 图像中稳定检测到目标

#### P5-B：深度图 RGB-D 配准与几何 Grounding

- 固定深度类型：**DepthPlanar**（优先）或 DepthPerspective，并对齐逆投影公式
- 实现 bbox 区域中位数深度提取，过滤无效深度值（0 / NaN / 超量程）
- 基于 AirSim `settings.json` 中相机内参 `(fx, fy, cx, cy)` 完成逆投影，输出 camera frame 3D 点
- 验收：已知距离目标，camera frame 坐标估计误差 <20%

#### P5-C：tf2 外参管理与坐标系变换

- 配置 camera→body frame 固定外参（与 `settings.json` 相机安装位姿一致）
- 配置 body→world frame 动态变换（订阅 VINS-Fusion / odom 位姿广播 TF）
- 验收：`ros2 run tf2_tools view_frames` 能看到完整变换链

#### P5-D：VINS-Fusion 位姿接入

- 仿真阶段：用 `odom_local_ned` remap 替代 VINS-Fusion（零代码修改，仅 launch remap）
- 真机/高精度阶段：接入 VINS-Fusion mono + IMU，订阅 `/vins_estimator/odometry`
- 验收：`target_position_world` 在无人机移动期间保持相对稳定（目标不随机体抖动）

#### P5-E：target_position_world → EGO-Planner 接口封装

- 将 P5-B/C/D 生成的 `target_position_world` 封装为 `/uav/target_goal`（`PoseStamped`）发布
- EGO-Planner 消费此目标，生成局部轨迹
- 位姿源由 `odom_local_ned` 切到 VINS 时的 remap 与延迟预算验证
- `mission_manager`：LLM 速度模式 vs planner 模式互斥与抢占策略
- 安全层：规划速度上限、地理围栏、与 `ego_bspline_to_twist_relay` 超时策略联调
- AirSim `settings.json` 与 `grid_map` 内参 / `cam2body_` 改动的**同步检查表**（避免体素镜像、大平面）
- 验收：输入 `"找到红色汽车并前往"` 后，无人机能在 AirSim 仿真中自主定位并接近目标

### P4（长期）— 任务链编排

支持 LLM 规划更复杂的任务链，例如：
```json
{
  "thought": "巡逻路线：起飞→A点→B点→C点→返航",
  "plan": [
    {"action": "TAKEOFF", "params": {"altitude": 10}},
    {"action": "GOTO_NED", "params": {"x": 20, "y": 0, "altitude": 10}},
    {"action": "GOTO_NED", "params": {"x": 20, "y": 20, "altitude": 10}},
    {"action": "GOTO_NED", "params": {"x": 0, "y": 20, "altitude": 10}},
    {"action": "RTL", "params": {}}
  ]
}
```

---

## 9. 关键文件路径速查

```
/home/hw/hw-ros2/ros2/src/hw_insight/
├── hw_insight/
│   ├── text_command_bridge.py    ← 核心桥接（11 action；GOTO_NED→/uav/target_goal）
│   ├── move_velocity.py          ← PX4 执行器（command_topic 可 remap）
│   ├── planner_velocity_bridge.py← Planner Twist → keyboard_velocity
│   ├── ego_bspline_to_twist_relay.py ← Bspline → TwistStamped
│   ├── test_planner_feedback.py  ← 规划反馈粗测脚本
│   ├── gcs_dashboard.py          ← TUI（4Hz）
│   ├── flight_regression_runner.py
│   └── llm_client.py             ← Groq / Ollama；交互式模型选择；ANTHROPIC_* 默认主机与模型；离线拦截与 think 日志
├── launch/
│   ├── text_command_test.launch.py   ← 默认手飞+LLM 链
│   ├── planner_integration.launch.py ← AirSim+飞控+ego_planner+RViz
│   ├── ego_planner_integration.launch.py ← 规划器参数与 remap 集中处
│   └── gcs_dashboard.launch.py
├── rviz/
│   └── ego_planner_debug.rviz
├── config/
│   └── mapping_config.yaml
├── docs/
│   ├── ego_planner_feasibility_report.md
│   └── integration_log_v1.md
├── COMMAND_PROTOCOL.md
├── PRD_text_command_flight_mvp.md ← v5.1
├── README_text_command_test.md
├── PRODUCT_TEST_FLOW.md
└── SESSION_HANDOVER.md           ← 本文档

/home/hw/hw-ros2/ros2/src/external/ego_planner_core/  ← EGO-Planner 核心 C++ 包（plan_env 等）
Windows: %USERPROFILE%\Documents\AirSim\settings.json  ← CameraDepth1 FOV/分辨率/安装位姿

# Git 仓库（S7 新增）
/home/hw/hw-ros2/                 ← Git 仓库根目录
├── .gitignore                    ← 排除规则（build/install/第三方包/AirSim库）
└── README.md                     ← 项目中文说明文档
Remote: git@github.com:Garfield-Wuu/LLM-UAV-ROS2.git
Branch: main

# YOLO-World（S8 新增）
/home/hw/YOLO-World/
├── test_yolo_world.py            ← 推理测试脚本（CPU/GPU 自动适配，词汇表可配）
├── weights/
│   └── yolo_world_v2_s_stage1.pth ← YOLO-World-S 权重（305MB）
├── clip_tokenizer/               ← CLIP ViT-B/32 离线模型（tokenizer + 权重）
└── outputs/                      ← 推理结果图片目录（<input>_result.jpg）
```

---

## 10. 构建命令

```bash
cd /home/hw/hw-ros2/ros2
source /opt/ros/humble/setup.bash

# 仅 Python / launch / rviz 变更
colcon build --packages-select hw_insight

# 修改了 external/ego_planner_core 下 C++（plan_env / ego_planner）
colcon build --packages-select plan_env ego_planner hw_insight

source install/setup.bash
```

`hw_insight` 构建通常数秒；`plan_env`+`ego_planner` 全量约 1–2 分钟量级。

---

## 11. 重要设计决策记录

| 决策 | 选择 | 备选方案 | 原因 |
|------|------|---------|------|
| ARM 触发机制 | `z < -5 m/s` 速度阈值 | 专用 ARM 话题 | 零新增话题，与 keyboard_velocity 接口兼容 |
| ARM 窗口时长 | 2.5s（从 0.8s 延长）| 重试机制 | WSL2 时间同步抖动期间需要多次 ARM 尝试 |
| RTL/ESTOP 路径 | `text_command_bridge` 直发 VehicleCommand | 经 move_velocity 中转 | 安全指令最短路径 |
| MOVE_VELOCITY 坐标系 | **机体坐标**（vx=机头，vy=右侧）| NED 世界坐标 | 用户说"前进"期望相对机头，与实际运动一致 |
| 坐标变换位置 | `text_command_bridge.apply_command` | `move_velocity` | Bridge 已有 heading 数据；GOTO_NED 等无需转换 |
| yawspeed 修正 | `move_velocity` 取反 `-yawspeed` | Bridge 层取反 | 最底层修正，不影响 YAW_TO P 控制器符号 |
| llm_client 启动方式 | 独立终端 `ros2 run` | ros2 launch 子进程 | launch 捕获 stdin，子进程无法接收键盘输入 |
| llm_client provider/model 选择 | 启动前方向键菜单 | 全部依赖 `--ros-args` / 硬编码 | 对操作员更友好；非 TTY 场景仍自动退回参数模式 |
| LLM HTTP 客户端 | `urllib.request`（stdlib）+ `User-Agent: curl/7.81.0` | openai/requests 库 | 零额外依赖；curl UA 绕过 Cloudflare 过滤 |
| 首次 Ollama 冷启动 | 启动即 warm-up 后再开放输入 | 让第一条真实指令承担冷启动延迟 | 降低首次交互超时概率，改善操作体验 |
| 多步执行等待策略 | 有 duration → 等 duration+1.2s；事件型 → 轮询 TELEMETRY command=IDLE | 固定 sleep | 兼顾时间确定性和事件响应 |
| GOTO_NED 完成判定 | 3D 距离 ≤ 0.5m | 分轴判断 | 简单可靠，适合速度控制模式下的漂移余量 |
| TUI 事件过滤 | 只显示非 TELEMETRY 事件 | 显示全部 | TELEMETRY 5Hz 会淹没有效信息 |
| 规划执行通道复用 | B-spline→Twist→`planner_velocity_bridge`→`/hw_insight/keyboard_velocity` | 新 MAV 话题 | 复用现有 Offboard 速度链，改动面小 |
| 远程 Ollama 默认 URL | `llm_client` 读 `ANTHROPIC_BASE_URL` / `ANTHROPIC_MODEL` | 硬编码 localhost | 与现有 Claude-Code 兼容环境变量对齐；建议写入 `~/.bashrc` |
| LLM 幻觉 JSON 处理 | API JSON mode + 动作别名库 + 平衡括号 / 正则提取 | 仅 `find('{')/rfind('}')` | 提升混合文本、围栏代码块、`<think>` 包裹场景下的解析鲁棒性 |
| `<think>` 推理处理 | 终端摘要 + ROS2 log 持久化 | 直接剥离丢弃 | 保留可解释性，便于后续审计与问题复盘 |
| 栅格地图 NED | `ground_height`、`map_size_z`、`visualization_truncate_height` 按 NED 调 | 沿用 ENU 默认 | 否则起飞后地图空、或地面整块显示 |
| 相机投影 | `grid_map/fx,fy,cx,cy` 与 AirSim `settings.json` 一致；`cam2body_` 含安装平移/姿态 | 仅默认内参 | 否则体素与 RGB 对不齐、“倒像”、大平面 |
| WSL2 DDS | `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | 默认 SHM | 避免 SHM 权限/发现失败 |

---

## 12. 已知局限与注意事项

1. **WSL2 时钟跳变**：Windows 休眠/唤醒后 PX4 时间同步会短暂失效（`time sync no longer converged`），此时解锁会被拒绝。等待 `time sync converged` 日志出现后再发 TAKEOFF，或在 Windows 端执行 `w32tm /resync /force`。

2. **LLM 延迟**：Groq API 通常 <2s，但网络波动时可达 5s+。远端 Ollama 大模型首次冷启动通常更慢，当前已增加 warm-up；`llm_busy` 锁保证不并发，但用户感知仍可能有延迟。

3. **坐标系说明**：`MOVE_VELOCITY`/`MOVE_REL` 的 vx/vy 是机体坐标（相对机头），vz 仍是 NED（正=下）。`GOTO_NED` 的 x/y 是绝对 NED 世界坐标（x=北，y=东）。两套坐标并存，LLM Prompt 已有说明。

4. **多步 plan 等待时序**：event-based actions（GOTO_NED、YAW_TO）依赖 TELEMETRY `command` 字段回到 IDLE，若飞控链路延迟则等待超时（30s）后强行进入下一步。

5. **AirSim 坐标原点**：AirSim 世界坐标原点在 UE 场景中 Player Start 位置，GOTO_NED 坐标以此为参考。

6. **修改 `settings.json` 后必须重启 AirSim（UE）**，并同步更新 `ego_planner_integration.launch.py` 中 `grid_map/fx,fy,cx,cy` 与 `plan_env` 的 `cam2body_`（见 PRD §13）。二者不一致时表现为体素错位、与相机画面“上下颠倒”或整片地面占用。

7. **`text_command_test` 与 `planner_integration` 不要同时启动**（会重复起 `airsim_node`、`move_velocity` 等）。同一仿真会话二选一。

8. **规划速度 vs LLM/键盘**：`planner_velocity_bridge` 与 `text_command_bridge` 可能同时写 `/hw_insight/keyboard_velocity`；使用 `planner_mode_for_goto` 等参数避免 GOTO 时双写，或临时停一侧节点。

9. **交互式模型选择依赖 TTY**：方向键菜单仅适用于 `ros2 run hw_insight llm_client` 的独立终端。若从 `ros2 launch`、脚本管道或非 TTY 环境启动，会自动跳过交互，退回 ROS 参数 / 环境变量模式。

10. **环境变量需要持久化**：`GROQ_API_KEY`、`ANTHROPIC_BASE_URL`、`ANTHROPIC_MODEL` 等若只在单个终端里 `export`，新终端不会继承。当前推荐统一写入 `~/.bashrc`。

11. **离线拦截语义**：若长时间未收到新鲜 TELEMETRY，普通动作会被前置安全层拦截；`RTL` / `LAND` / `HOVER` / `EMERGENCY_STOP` 仍允许通过，且可用 `!` 前缀触发专家绕过路径。
