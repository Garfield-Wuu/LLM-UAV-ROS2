# LLM-UAV-ROS2

基于大语言模型的无人机自主决策系统（AirSim + PX4 + ROS 2 Humble）

---

## 项目简介

本项目构建了一套面向自然语言任务执行的无人机自主决策系统，核心目标是让用户输入自然语言指令（例如"找到红色汽车并前往它"），系统通过完整的感知-决策-规划-执行闭环完成任务。

### 完整技术路线

```
用户自然语言指令
  → LLM 任务语义解析（Groq / Ollama）
  → YOLO-World 开放词汇目标检测
  → AirSim 深度图 + 几何 Grounding（camera frame 3D 点恢复）
  → VINS-Fusion 位姿估计（world frame 对齐）
  → EGO-Planner 局部轨迹规划
  → PX4 Offboard 飞行执行
```

---

## 当前实现状态

| 模块 | 技术 | 状态 |
|------|------|------|
| 仿真环境 | AirSim + Unreal Engine | ✅ 运行中 |
| 飞控 | PX4 SITL (`none_iris`) | ✅ 运行中 |
| 中间件 | ROS 2 Humble + uXRCE-DDS | ✅ 运行中 |
| 任务理解 | LLM（Groq API / Ollama 双后端） | ✅ 已接入 |
| 飞行执行 | 11 个动作协议 + PX4 Offboard | ✅ 已接入 |
| 局部规划 | EGO-Planner（仿真最小闭环） | ⚠️ 部分接入 |
| 视觉识别 | YOLO-World（open-vocabulary） | 📋 规划中 |
| 状态估计 | VINS-Fusion | 📋 规划中 |

---

## 完整工程结构

下表列出新机器复现时所需的**全部目录**，标注了哪些已在本仓库中、哪些需要另外获取。

### 本仓库包含（`git clone` 后自动就位）

```
hw-ros2/                                      ← 本仓库根目录
├── .gitignore
├── README.md
├── setup/
│   ├── setup.sh                              # 一键安装 ROS 2 + 依赖脚本
│   └── ros-archive-keyring.gpg
└── ros2/
    └── src/
        ├── hw_insight/                       # ★ 核心自研包
        │   ├── hw_insight/                   # Python 节点
        │   │   ├── llm_client.py             #   LLM 推理（Groq / Ollama 双后端）
        │   │   ├── text_command_bridge.py    #   指令解析 + 11 个动作分发
        │   │   ├── move_velocity.py          #   PX4 Offboard 执行器
        │   │   ├── gcs_dashboard.py          #   地面站 TUI（4Hz）
        │   │   ├── flight_regression_runner.py  # 闭环回归测试
        │   │   ├── planner_velocity_bridge.py   # Planner Twist → keyboard_velocity
        │   │   └── ego_bspline_to_twist_relay.py
        │   ├── launch/                       # 启动文件
        │   │   ├── text_command_test.launch.py      # 默认手飞 / LLM 链
        │   │   ├── planner_integration.launch.py    # AirSim + EGO-Planner + RViz
        │   │   ├── ego_planner_integration.launch.py
        │   │   └── gcs_dashboard.launch.py
        │   ├── config/mapping_config.yaml    # 话题与坐标帧映射说明
        │   ├── rviz/ego_planner_debug.rviz   # RViz 调试配置
        │   ├── docs/                         # 集成文档与排障日志
        │   ├── COMMAND_PROTOCOL.md           # 指令协议规范
        │   ├── PRD_text_command_flight_mvp.md
        │   ├── README_text_command_test.md
        │   ├── SESSION_HANDOVER.md
        │   └── PRODUCT_TEST_FLOW.md
        ├── hw_interface/                     # ★ 自研 ROS 2 消息接口
        │   └── msg/HWSimpleKeyboardInfo.msg
        └── external/
            └── ego_planner_core/             # ★ EGO-Planner 核心（含本项目修改）
                └── planner/
                    ├── ego_planner/          #   规划主逻辑
                    ├── plan_env/             #   栅格地图（已修改 cam2body + 缓冲区）
                    ├── bspline_opt/          #   B 样条优化
                    ├── path_searching/       #   A* 路径搜索
                    └── traj_utils/           #   轨迹工具 + Bspline.msg
```

### 需要另外获取（不在本仓库，新机器必须手动安装）

```
ros2/src/                                     ← clone 到同一 ros2/src/ 目录下
├── px4_msgs/                                 # PX4 uORB 消息定义
│   └── [clone] https://github.com/PX4/px4_msgs  (branch: main, tag: v2.0.1)
├── px4_ros_com/                              # PX4 ROS 2 通信工具
│   └── [clone] https://github.com/PX4/px4_ros_com
├── airsim_ros_pkgs/                          # AirSim ROS 2 节点
│   └── [clone] https://github.com/microsoft/AirSim  (取 ros2/src/airsim_ros_pkgs)
├── airsim_interfaces/                        # AirSim ROS 2 消息接口
│   └── 同上 AirSim 仓库 (取 ros2/src/airsim_interfaces)
└── px4-ros2-interface-lib-1.4.0/             # PX4 ROS 2 接口库（可选）
    └── [clone] https://github.com/Auterion/px4-ros2-interface-lib  (v1.4.0)

~/                                            ← 放在 home 目录，不在本仓库
├── px4v1.15.2/                               # PX4 SITL 飞控
│   └── [clone] https://github.com/PX4/PX4-Autopilot  (v1.15.2)
├── Micro-XRCE-DDS-Agent/                     # uXRCE-DDS 桥接 Agent
│   └── [clone] https://github.com/eProsima/Micro-XRCE-DDS-Agent
├── YOLO-World/                               # 视觉语义识别（Phase 2）
│   └── [clone] https://github.com/AILab-CVC/YOLO-World
├── QGroundControlV4.4.4.AppImage            # 地面站（可选监控）
│   └── 下载自 https://github.com/mavlink/qgroundcontrol/releases
└── AirSim（Windows 侧）                      # Unreal Engine 仿真环境
    └── 下载自 https://github.com/microsoft/AirSim/releases
```

### 构建产物（不提交，本地生成）

```
ros2/
├── build/     # colcon build 生成，约 600 MB
├── install/   # colcon build 生成，约 160 MB
└── log/       # 构建日志
```

---

## 环境依赖

| 依赖 | 版本 | 获取方式 |
|------|------|---------|
| Ubuntu | 22.04 (WSL2 可用) | — |
| ROS 2 | Humble | `setup/setup.sh` 或[官方文档](https://docs.ros.org/en/humble/Installation.html) |
| PX4 Autopilot | v1.15.2 | [PX4 GitHub](https://github.com/PX4/PX4-Autopilot) |
| AirSim | latest | [AirSim GitHub](https://github.com/microsoft/AirSim) |
| Micro-XRCE-DDS-Agent | latest | [eProsima GitHub](https://github.com/eProsima/Micro-XRCE-DDS-Agent) |
| px4_msgs | 对应 PX4 版本 | [px4_msgs GitHub](https://github.com/PX4/px4_msgs) |
| px4_ros_com | 对应 PX4 版本 | [px4_ros_com GitHub](https://github.com/PX4/px4_ros_com) |
| airsim_ros_pkgs | latest | [AirSim ROS2](https://github.com/microsoft/AirSim/tree/main/ros2) |
| Groq API Key | — | [console.groq.com](https://console.groq.com) |

---

## 快速开始

### 1. 克隆仓库并安装第三方依赖

```bash
git clone git@github.com:Garfield-Wuu/LLM-UAV-ROS2.git ~/hw-ros2
cd ~/hw-ros2

# 将第三方 ROS 2 包 clone 到 ros2/src/ 下（参考各包官方文档）
# px4_msgs / px4_ros_com / airsim_ros_pkgs / airsim_interfaces 等
```

### 2. 配置环境变量（写入 `~/.bashrc`）

```bash
# Groq API（二选一）
export GROQ_API_KEY="gsk_你的key"

# 远端 Ollama（二选一）
export ANTHROPIC_BASE_URL="http://你的主机:端口"
export ANTHROPIC_MODEL="qwen3-coder:30b"
export ANTHROPIC_API_KEY="ollama"

# WSL2 FastDDS 稳定性
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source ~/.bashrc
```

### 3. 编译

```bash
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select hw_interface hw_insight
source install/setup.bash
```

> 若需要 EGO-Planner：
> ```bash
> colcon build --packages-select plan_env ego_planner hw_insight
> ```

### 4. 启动系统

依次打开 6 个终端：

```bash
# T1：AirSim（Windows 侧点击 Unreal Play）

# T2：PX4 SITL
cd /home/hw/px4v1.15.2
make px4_sitl_default none_iris

# T3：uXRCE-DDS 桥接
MicroXRCEAgent udp4 -p 8888

# T4：ROS 2 主链
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hw_insight text_command_test.launch.py

# T5：LLM 交互终端（方向键选择 Groq / Ollama 与模型）
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hw_insight llm_client

# T6：地面站 TUI（可选）
ros2 run hw_insight gcs_dashboard --ros-args -p refresh_rate_hz:=4.0
```

---

## 支持的飞行动作（11 个）

| 动作 | 说明 |
|------|------|
| `TAKEOFF` | 起飞到指定高度 |
| `LAND` | 降落 |
| `HOVER` | 悬停指定时长 |
| `MOVE_VELOCITY` | 机体坐标系速度控制 |
| `MOVE_REL` | 机体坐标系相对位移 |
| `GOTO_NED` | NED 世界坐标系绝对目标点 |
| `ORBIT` | 圆形轨道飞行 |
| `YAW_TO` | 偏航到指定角度 |
| `RTL` | 返航 |
| `EMERGENCY_STOP` | 紧急停止 |
| `SET_SPEED` | 设置飞行速度 |

---

## 文档索引

| 文档 | 说明 |
|------|------|
| `ros2/src/hw_insight/README_text_command_test.md` | 完整操作手册与启动流程 |
| `ros2/src/hw_insight/SESSION_HANDOVER.md` | 开发会话交接文档（技术路线 + 变更记录） |
| `ros2/src/hw_insight/PRD_text_command_flight_mvp.md` | 产品需求文档（v5.0） |
| `ros2/src/hw_insight/COMMAND_PROTOCOL.md` | 指令协议规范 |
| `ros2/src/hw_insight/docs/` | EGO-Planner 集成报告与排障日志 |

---

## License

MIT
