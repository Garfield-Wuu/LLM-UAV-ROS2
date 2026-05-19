# 测试会话报告（重启段）

## 1. 测试范围与时间
- 日志来源：`/home/hw/hw-ros2/ros2/src/hw_insight/TEST_SESSION_LOG.md`
- 取样范围：最新“会话重新开始”段落（第 218 行）至文件末尾
- 测试时间范围：`2026-05-19 22:46:19` ~ `2026-05-19 22:49:25`
- 覆盖日志行数：`255`

## 2. 操作时间线（按时间排序）
- - [2026-05-19 22:46:19] [monitor] [系统] 监控进程已启动，开始增量采集终端关键事件
- - [2026-05-19 22:46:20] [9.txt] [系统] 发现新终端文件，开始监控
- - [2026-05-19 22:46:21] [9.txt] [用户输入指令] ros2 run hw_insight llm_client
- - [2026-05-19 22:46:22] [3.txt] [用户输入指令] cd /home/hw/hw-ros2/ros2source /opt/ros/humble/setup.bashsource install/setup.bashexport FASTDDS_BUILTIN_TRANSPORTS=UDPv4ros2 launch hw_insight planner_integration.launch.py use_rviz:=true
- - [2026-05-19 22:47:19] [9.txt] [LLM推理开始] 22:47:17  LLM      推理完成  0.8s
- - [2026-05-19 22:47:19] [9.txt] [UAV动作] 22:47:17  UAV      TAKEOFF          {'altitude': 6.0}
- - [2026-05-19 22:47:22] [9.txt] [UAV动作] USER  uav[TAKEOFF|武装] 请输入指令 ›
- - [2026-05-19 22:48:02] [9.txt] [LLM推理开始] 22:48:01  LLM      推理完成  0.9s
- - [2026-05-19 22:48:21] [9.txt] [LLM推理开始] 22:48:20  LLM      推理完成  0.9s
- - [2026-05-19 22:48:23] [9.txt] [UAV动作] USER  uav[TAKEOFF|武装] 请输入指令 › 前进10米再升高4米
- - [2026-05-19 22:48:41] [9.txt] [LLM推理开始] 22:48:40  LLM      推理完成  0.8s
- - [2026-05-19 22:46:19] [1.txt] [用户输入指令] (metadata) clear
- - [2026-05-19 22:46:19] [2.txt] [用户输入指令] (metadata) clear
- - [2026-05-19 22:46:19] [3.txt] [用户输入指令] (metadata) clear
- - [2026-05-19 22:46:19] [4.txt] [用户输入指令] (metadata) clear
- - [2026-05-19 22:46:21] [4.txt] [ERROR/WARN] [yolo_world_detector-1]   warnings.warn("Unable toimport Axes3D. This may be due to multiple versions of "
- - [2026-05-19 22:46:21] [9.txt] [用户输入指令] cd /home/hw/hw-ros2/ros2
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.470550805] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.470692737] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.470780746] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.470899884] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.471071314] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.471227682] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.471329923] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.471410183] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:48:55] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202135.357638648] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:48:58] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202138.373381139] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:04] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202144.388926815] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:07] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202147.399567217] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:10] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202150.405055861] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:16] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202156.424674802] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- - [2026-05-19 22:49:25] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202165.447749967] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.

## 3. 关键日志摘录（保留关键字段）
- 指令字段（用户输入指令）：
  - - [2026-05-19 22:46:19] [1.txt] [用户输入指令] (metadata) clear
  - - [2026-05-19 22:46:19] [2.txt] [用户输入指令] (metadata) clear
  - - [2026-05-19 22:46:19] [3.txt] [用户输入指令] (metadata) clear
  - - [2026-05-19 22:46:19] [4.txt] [用户输入指令] (metadata) clear
  - - [2026-05-19 22:46:21] [9.txt] [用户输入指令] cd /home/hw/hw-ros2/ros2
  - - [2026-05-19 22:46:21] [9.txt] [用户输入指令] source /opt/ros/humble/setup.bash
- LLM字段（LLM推理/推理耗时）：
  - - [2026-05-19 22:46:23] [9.txt] [LLM推理开始] 选择 LLM 推理后端
  - - [2026-05-19 22:47:18] [9.txt] [LLM推理开始] 22:47:17  LLM      LLM 推理中…
  - - [2026-05-19 22:47:19] [9.txt] [LLM推理开始] 22:47:17  LLM      推理完成  0.8s
  - - [2026-05-19 22:48:01] [9.txt] [LLM推理开始] 22:48:00  LLM      LLM 推理中…
  - - [2026-05-19 22:48:02] [9.txt] [LLM推理开始] 22:48:01  LLM      推理完成  0.9s
  - - [2026-05-19 22:48:20] [9.txt] [LLM推理开始] 22:48:19  LLM      LLM 推理中…
  - - [2026-05-19 22:48:21] [9.txt] [LLM推理开始] 22:48:20  LLM      推理完成  0.9s
  - - [2026-05-19 22:48:40] [9.txt] [LLM推理开始] 22:48:39  LLM      LLM 推理中…
- UAV动作字段：
  - - [2026-05-19 22:47:19] [9.txt] [UAV动作] 22:47:17  UAV      TAKEOFF          {'altitude': 6.0}
  - - [2026-05-19 22:47:21] [1.txt] [UAV动作] INFO  [commander] Takeoff detected
  - - [2026-05-19 22:47:22] [3.txt] [UAV动作] [move_velocity-4] [INFO] [1779202038.063692847] [move_velocity]: Flight requested — activating Offboard stream
  - - [2026-05-19 22:47:22] [9.txt] [UAV动作] USER  uav[TAKEOFF|武装] 请输入指令 ›
  - - [2026-05-19 22:48:23] [9.txt] [UAV动作] USER  uav[TAKEOFF|武装] 请输入指令 › 前进10米再升高4米
- ERROR/WARN字段：
  - - [2026-05-19 22:46:21] [4.txt] [ERROR/WARN] [yolo_world_detector-1]   warnings.warn("Unable toimport Axes3D. This may be due to multiple versions of "
  - - [2026-05-19 22:46:35] [1.txt] [ERROR/WARN] WARN  [timesync] time jump detected. Resetting time synchroniser.
  - - [2026-05-19 22:46:35] [1.txt] [ERROR/WARN] WARN  [uxrce_dds_client] time sync no longer converged
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.305000136] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.315553246] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.326294394] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.337106001] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.347540013] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.358237696] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:49] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202129.368791609] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:55] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202135.346587172] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
  - - [2026-05-19 22:48:55] [3.txt] [ERROR/WARN] [ego_planner_node-9] [WARN] [1779202135.356904654] [check_collision_and_rebound]: WARN! terminal point of the current trajectory is in obstacle, skip this planning.
- 节点状态字段：
  - 本段未检索到显式“节点状态”字段日志。

## 4. 异常/告警统计
- `[WARN]` 出现次数：`158`
- `[ERROR]` 出现次数：`0`
- `[ERROR/WARN]` 标签行数：`162`
- “终端文件被截断，已重置读取位置”出现次数：`65`
- 高频告警签名（Top 5）：
  - `158` 次：`WARN! terminal point of the current trajectory is in obstacle, skip this planning.`
  - `1` 次：`- [2026-05-19 22:46:21] [4.txt] [ERROR/WARN] [yolo_world_detector-1]   warnings.warn("Unable toimport Axes3D. This may be due to multiple versions of "`
  - `1` 次：`- [2026-05-19 22:46:35] [1.txt] [ERROR/WARN] WARN  [timesync] time jump detected. Resetting time synchroniser.`
  - `1` 次：`- [2026-05-19 22:46:35] [1.txt] [ERROR/WARN] WARN  [uxrce_dds_client] time sync no longer converged`
  - `1` 次：`Can't find the new base points at the opposite within the threshold. i=0, j=0`

## 5. 结论
- 本次重启会话中，指令链路、LLM推理链路与TAKEOFF相关动作日志均可观测，主流程已被触发。
- 会话后半段出现大量 `ego_planner_node-9` 碰撞相关 WARN（终点落入障碍物），路径规划未稳定收敛。
- 日志中存在高频“终端文件被截断”事件，影响监控连续性与时序可读性，建议优化采集方式。

## 6. 后续建议
- 优先排查 `check_collision_and_rebound` 告警：检查目标点可达性、地图障碍膨胀参数和规划终点约束。
- 在 AirSim/PX4 联调中复核坐标系转换（NED/ENU）与目标点发布逻辑，避免终点落入障碍。
- 优化监控脚本对终端截断的处理（如文件轮转检测、增量偏移持久化、去重），降低噪声。
- 增加“节点状态”结构化上报（例如 topic 心跳/模式状态），补齐状态级证据链。
