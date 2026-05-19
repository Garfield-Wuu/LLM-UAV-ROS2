# YOLO-World × AirSim 在线检测测试手册

面向 **不接 LLM** 的联调：验证 AirSim RGB → YOLO-World 2D → DepthPlanar 中值深度 → 相机系 3D → **ENU world** 的语义链；并说明与 **`planner_integration` + RViz** 叠图显示的关系。

---

## 1. 文档与代码索引

| 内容 | 路径 |
|------|------|
| 包内文档地图（与其它手册分工） | [`DOCUMENTATION_INDEX.md`](DOCUMENTATION_INDEX.md) |
| 当前开发主流程（含 **T_YOLO**） | `hw_insight/PRD_text_command_flight_mvp.md` **§9** |
| 本手册 | `hw_insight/docs/yolo_world_airsim_online_test.md` |
| 话题预检脚本 | `hw_insight/scripts/check_airsim_camera_topics.sh` |
| 语义链 launch | `hw_insight/launch/semantic_perception.launch.py` |
| 单节点 2D launch | `hw_insight/launch/yolo_world_test.launch.py` |
| 检测 + RViz 叠图节点 | `hw_insight/hw_insight/detections_image_overlay.py`（由 `ego_planner_integration` 随 `enable_detection_overlay` 启动） |
| RViz 配置 | `hw_insight/rviz/ego_planner_debug.rviz`（含 **YOLOOverlay** → `/uav/camera/detections_overlay`） |
| YOLO-World 环境与权重 | 工作区 [`YOLO-World/INTEGRATION.md`](../../../../../YOLO-World/INTEGRATION.md)（相对本文件上溯至 `hw/`） |

---

## 2. 环境与构建

**前提**

- Windows 侧 AirSim 已运行；WSL2 中 `airsim_node` 已发布图像（常见为与 `planner_integration` 同启）。
- 每个 ROS 终端建议：`export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- 工作区：`cd /home/hw/hw-ros2/ros2`，`source /opt/ros/humble/setup.bash && source install/setup.bash`

**修改 launch / 脚本 / RViz / Python 节点后必须安装到 `install`：**

```bash
colcon build --packages-select hw_insight
source install/setup.bash
```

**常见命令笔误**

- `ros2 pkg prefix` 与包名之间**必须有空格**：`ros2 pkg prefix hw_insight`
- 检测词话题名：**`/uav/target_query`**（带下划线，勿写成 `targetquery`）
- `ros2 topic echo` 与话题之间**必须有空格**

---

## 3. 默认参数与推理模式

### 3.1 `semantic_perception.launch.py`（全链）

| Launch 参数 | 默认 | 含义 |
|-------------|------|------|
| `inference_mode` | **`on_query`** | 仅在有非空 `/uav/target_query` 时对**当前缓存 RGB** 推理一次；非持续轮询 |
| `publish_target_goal` | **`false`** | 不向 `/uav/target_goal` 发点，避免误驱动规划器 |
| `texts` | `car` | 占位初始词；`on_query` 下以 `target_query` 为准 |
| `interval` | `0.3` | 仅在 **`continuous`** 模式下作为最小推理间隔（秒） |
| `device` | `auto` | `auto` / `cuda:0` / `cpu` |

恢复旧行为（持续检测 + 自动发规划目标）：

```bash
ros2 launch hw_insight semantic_perception.launch.py \
  inference_mode:=continuous \
  publish_target_goal:=true \
  texts:="red car"
```

### 3.2 `yolo_world_detector` 参数（launch 透传）

- 支持 **`inference_mode:=on_query`**（或别名 `once`）：每条 `target_query` 触发一次推理。
- **`yolo_world_test.launch.py` 默认仍为 `continuous`**，单节点压测需显式写 `inference_mode:=on_query` 才与语义链默认一致。

`on_query` 下若推理抛错，本次触发已消费，需**再发一条** `target_query`。若发指令时模型仍在加载，建议在日志出现 **`YoloWorldDetector ready`** 后再发。

---

## 4. 数据流与主要话题

```text
AirSim Scene ─────────────────────────────► yolo_world_detector
/uav/target_query ────────────────────────►（更新 prompt / on_query 触发）
yolo_world_detector ─ /uav/detections_2d ─► target_grounding_node
DepthPlanar + camera_info ────────────────►（bbox 内深度中值 → x_cam,y_cam,z_cam）
target_grounding ─ /uav/semantic_targets_camera ─► semantic_target_tf_node
odom_local_ned ───────────────────────────►（camera→body→world，NED→ENU）
semantic_target_tf ─ /uav/semantic_targets_world
                    └ /uav/semantic_target_marker（RViz 球体，可选）
                    └ /uav/target_goal（publish_target_goal:=true 时）
```

| 话题 | 类型 | 说明 |
|------|------|------|
| `/airsim_node/PX4/CameraDepth1/Scene` | `sensor_msgs/Image` | RGB |
| `/airsim_node/PX4/CameraDepth1/DepthPlanar` | `sensor_msgs/Image` | 深度（米，`32FC1` 等） |
| `/airsim_node/PX4/CameraDepth1/camera_info` | `sensor_msgs/CameraInfo` | 内参 |
| `/airsim_node/PX4/odom_local_ned` | `nav_msgs/Odometry` | 机体位姿（TF 链输入） |
| `/uav/target_query` | `std_msgs/String` | 检测词 / 动态 prompt |
| `/uav/detections_2d` | `std_msgs/String` | JSON：prompt、bbox、score 等 |
| `/uav/semantic_targets_camera` | `std_msgs/String` | JSON：depth_m、x_cam、y_cam、z_cam… |
| `/uav/semantic_targets_world` | `std_msgs/String` | JSON：x_world、y_world、z_world（ENU）… |
| `/uav/camera/detections_overlay` | `sensor_msgs/Image` | RViz 叠图（框 + 可选 `d=…m`） |

---

## 5. 深度与坐标（结算含义摘要）

- **深度 `depth_m`**：在 2D **bbox** 与深度图对齐区域内，按 `depth_sample_stride` 采样，过滤无效值后取 **中位数**（鲁棒）；不是严格的「无人机到目标欧氏距离」，而是 **该 patch 沿视线方向的典型距离**，再与内参一起做针孔反投影。
- **相机系**：`x_cam,y_cam,z_cam`（光学系，`z_cam` 与用于反投影的深度尺度一致）。
- **世界系**：`x_world,y_world,z_world` 为 **ENU**（默认由 NED odom 经节点内约定旋转得到），便于与 EGO-Planner / RViz `world` 对齐。

若 `/uav/detections_2d` 中 **`detections` 为空**（日志 `dets=0`），`target_grounding_node` **不会**发布 `/uav/semantic_targets_camera`，下游 world 与叠图上的 **`d=`** 也不会更新——属预期；可调低 `score_thr`、简化词（如 `car`）或换视角后再发 `target_query`。

---

## 6. 分阶段测试

### 阶段 0：话题预检（不启 YOLO）

| 话题 | 用途 |
|------|------|
| `.../CameraDepth1/Scene` | RGB |
| `.../CameraDepth1/DepthPlanar` | 深度 |
| `.../CameraDepth1/camera_info` | 内参 |
| `.../odom_local_ned` | 里程计 |

```bash
bash "$(ros2 pkg prefix hw_insight)/share/hw_insight/scripts/check_airsim_camera_topics.sh"
# 源码树：bash src/hw_insight/scripts/check_airsim_camera_topics.sh [超时秒]
```

四个均 `OK` 后再进入后续阶段。

### 阶段 1：仅 2D（`yolo_world_test`）

```bash
ros2 launch hw_insight yolo_world_test.launch.py \
  device:=auto \
  interval:=5.0 \
  texts:="car,person,truck" \
  score_thr:=0.25
```

另开终端：`ros2 topic echo /uav/detections_2d`。**`on_query`** 下单次触发：

```bash
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"
```

**`continuous`**（`yolo_world_test` 默认）下主要靠 `texts` 与 `interval`，一般无需发 `target_query`。

### 阶段 2：全链（默认 `on_query` + 不发 goal）

```bash
ros2 launch hw_insight semantic_perception.launch.py \
  score_thr:=0.15 \
  interval:=0.5
```

监听：`/uav/detections_2d`、`/uav/semantic_targets_camera`、`/uav/semantic_targets_world`。

另开终端触发（默认 **`on_query`**，每条非空 query 推理一次）：

```bash
ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"
```

### 阶段 3：与 `planner_integration` 并行

勿重复启动第二套 `airsim_node`。在已有 `planner_integration` 的 ROS 图里另开 `semantic_perception.launch.py` 即可。

### 阶段 4：故障顺序

1. RGB 无数据 → AirSim `settings.json` 相机名与话题。  
2. 无 `Prompt updated` → `/uav/target_query` 与 QoS。  
3. `dets=0` → 降低 `score_thr`、简化词。  
4. 有 2D 无 3D → 深度编码、`max_depth_age_sec`、时间戳。  
5. 有 camera 无 world → `odom_topic`、`max_odom_age_sec`。

---

## 7. RViz：检测框与深度字端（`detections_image_overlay`）

- **`ego_planner_integration`**（被 `planner_integration`、`uav_sim` 等包含）在 **`enable_detection_overlay:=true`**（默认）时启动 **`detections_image_overlay`**。
- RViz 配置 **`ego_planner_debug.rviz`** 中 **Display「YOLOOverlay」** 订阅 **`/uav/camera/detections_overlay`**（`bgr8`）。
- 叠图逻辑：订阅 RGB + `/uav/detections_2d` 画框；若同时有 **`/uav/semantic_targets_camera`** 且 `image_stamp` 在 **`stamp_tolerance_sec`**（默认 0.35 s）内对齐，则在标签上追加 **`d=…m`**（与 grounding 的 **`depth_m`** 一致）。**仅 2D、无 grounding 时**无 `d=` 字端。
- 关闭叠图：顶层 `planner_integration` 传 **`enable_detection_overlay:=false`**（会转发给 ego）。关闭深度字端而保留框：节点参数 **`use_semantic_depth:=false`** 或 **`semantic_targets_topic:=""`**（需在 launch 中扩展透传，当前默认全开语义深度）。

---

## 8. 一键命令速查

| 目的 | 命令 |
|------|------|
| 话题预检 | `bash "$(ros2 pkg prefix hw_insight)/share/hw_insight/scripts/check_airsim_camera_topics.sh"` |
| 仅 2D（持续） | `ros2 launch hw_insight yolo_world_test.launch.py interval:=5.0 ...` |
| 仅 2D（按指令一次） | `... yolo_world_test.launch.py inference_mode:=on_query ...` |
| 触发一次检测（`on_query`） | `ros2 topic pub --once /uav/target_query std_msgs/msg/String "data: car"` |
| 全链（默认 on_query + 不发 goal） | `ros2 launch hw_insight semantic_perception.launch.py` |
| 全链 + 持续 + 发 goal | `... semantic_perception.launch.py inference_mode:=continuous publish_target_goal:=true` |
| 规划 + RViz（含叠图节点） | `ros2 launch hw_insight planner_integration.launch.py use_rviz:=true` |

更通用的 YOLO-World 环境与权重说明：[`../../../../../YOLO-World/INTEGRATION.md`](../../../../../YOLO-World/INTEGRATION.md)

---

## 9. 变更记录

| 日期 | 说明 |
|------|------|
| 2026-05-19 | 全文重组：索引表、默认参数、数据流、深度说明、RViz 叠图与深度字端、命令速查；与当前 launch/节点行为对齐；索引表内 `INTEGRATION.md` 与 §8 使用相同相对链接。同日补充：`ros2 topic pub --once` + YAML `data:` 触发 `on_query`；§6 阶段 1/2 与 §8 表格；与 PRD §9 **T_YOLO** 对齐。 |
