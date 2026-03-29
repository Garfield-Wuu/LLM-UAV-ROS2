# UAV Command Protocol v2.2

> **For LLM integration** — current flight actions use JSON over `/uav/user_command`.  
> Designed for `llm_client.py`, `text_command_bridge.py`, and the current PX4 Offboard chain.

---

## Input

| Field | Value |
|-------|-------|
| Topic | `/uav/user_command` |
| Type  | `std_msgs/String` |
| Format | JSON string (preferred) or Chinese text (back-compat) |

```json
{
  "action": "ACTION_NAME",
  "params": { "key": value, ... }
}
```

---

## Current Scope

本协议定义 **12 个 action**，涵盖飞行控制动作（11 个）与视觉语义搜索动作（1 个），对应 `llm_client.py` 全部支持 action。

语义感知链相关 topic（`FIND_AND_GOTO` 为上层入口，bridge 自动驱动下层）：

| Topic | Type | Purpose |
|-------|------|---------|
| `/uav/target_query` | `std_msgs/String` | `FIND_AND_GOTO` 触发后由 bridge 发布 prompt |
| `/uav/detections_2d` | `std_msgs/String` (JSON) | YOLO-World 2D 检测结果 |
| `/uav/semantic_targets_camera` | `std_msgs/String` (JSON) | camera frame 3D 目标点 |
| `/uav/semantic_targets_world` | `std_msgs/String` (JSON) | world frame 语义目标点（bridge 订阅后触发内部 GOTO_NED）|
| `/uav/target_goal` | `geometry_msgs/PoseStamped` | planner 目标点 |

---

## Output

| Topic | Type | Content |
|-------|------|---------|
| `/hw_insight/keyboard_velocity` | `hw_interface/HWSimpleKeyboardInfo` | Real-time velocity setpoint to move_velocity |
| `/uav/llm_task_status` | `std_msgs/String` (JSON) | Status events & rich telemetry |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Direct PX4 commands (RTL, EMERGENCY_STOP only) |

---

## Action Reference

### 1) TAKEOFF
```json
{"action": "TAKEOFF", "params": {"altitude": 6.0}}
```
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `altitude` | float (m) | 6.0 | Target hover altitude (ENU, positive = up) |

**Behavior**:  
1. ARM phase (2.5 s): sends z = −6.0 m/s → triggers `move_velocity` arm + Offboard switch  
2. Altitude hold: P-control z until within 0.35 m of target, then zero velocity

---

### 2) LAND
```json
{"action": "LAND", "params": {}}
```
No params. Sends z = +8.0 m/s → `move_velocity` triggers PX4 AUTO_LAND.

---

### 3) HOVER
```json
{"action": "HOVER", "params": {"duration": 3.0}}
```
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `duration` | float (s) | 0 | 0 = hover indefinitely; >0 = then auto-IDLE |

---

### 4) MOVE_VELOCITY
```json
{"action": "MOVE_VELOCITY", "params": {"vx": 2.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0, "duration": 3.0}}
```
| Param | Type | Description |
|-------|------|-------------|
| `vx` | float (m/s) | Body forward (+) / back (−), relative to drone heading |
| `vy` | float (m/s) | Body right (+) / left (−), relative to drone heading |
| `vz` | float (m/s) | NED down (+) / up (−) |
| `yaw_rate` | float (rad/s) | Positive means right-turn intent; low layer handles PX4/AirSim sign convention |
| `duration` | float (s) | After duration → IDLE |

`text_command_bridge.py` 会把 `vx/vy` 从**机体坐标**实时旋转到 NED 世界坐标再下发。

---

### 5) MOVE_REL
```json
{"action": "MOVE_REL", "params": {"dx": 5.0, "dy": 0.0, "dz": 0.0, "duration": 2.5}}
```
将**机体相对位移**转换为 `MOVE_VELOCITY`（`vx = dx / duration` 等）。

- `dx`: 沿机头方向前后
- `dy`: 沿机身左右
- `dz`: 仍使用 NED 垂直方向，负值上升、正值下降

---

### 6) GOTO_NED ⭐ new
```json
{"action": "GOTO_NED", "params": {"x": 10.0, "y": 5.0, "altitude": 6.0}}
```
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `x` | float (m) | current | NED x (North) |
| `y` | float (m) | current | NED y (East) |
| `altitude` | float (m) | current | ENU altitude (positive = up) |

**Behavior**: P-controller continuously computes velocity toward target. Auto-completes when within 0.5 m.  
Gain: `GOTO_KP = 0.8` — speed proportional to distance, clamped to `default_speed_xy`.

说明：

- `GOTO_NED` 的 `x/y` 是**绝对世界坐标**，不是相对机头方向。
- `altitude` 仍使用人类更直观的 ENU 高度正值，内部转换为 NED `z = -altitude`。

---

### 7) ORBIT ⭐ new
```json
{"action": "ORBIT", "params": {"cx": 0.0, "cy": 5.0, "radius": 5.0, "speed": 2.0, "duration": 30.0}}
```
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `cx` | float (m) | current x | NED orbit center x |
| `cy` | float (m) | current y | NED orbit center y |
| `radius` | float (m) | 5.0 | Orbit radius |
| `speed` | float (m/s) | half of default | Tangential speed |
| `duration` | float (s) | 0 | 0 = orbit indefinitely |

**Algorithm** (from LLM-controlled-drone):  
```
θ += ω × dt          (ω = speed/radius rad/s)
v_tangential = (−speed·sin θ,  speed·cos θ)
v_correction = Kp × (desired_pos − current_pos)   [Kp = 1.0]
v_z          = altitude P-hold
```
Altitude is held at the altitude when ORBIT was commanded.

---

### 8) YAW_TO ⭐ new
```json
{"action": "YAW_TO", "params": {"angle": 90.0}}
```
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `angle` | float (deg) | 0.0 | Target NED heading (0 = North, 90 = East, clockwise) |

P-controller on yaw error. Auto-completes when within 3°.

---

### 9) RTL ⭐ new
```json
{"action": "RTL", "params": {}}
```
Sends `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` directly to PX4. Drone returns to takeoff point and lands autonomously.

---

### 10) EMERGENCY_STOP ⭐ new
```json
{"action": "EMERGENCY_STOP", "params": {}}
```
Immediately sends `VEHICLE_CMD_COMPONENT_ARM_DISARM` (disarm) to PX4. **Use only when drone is on ground or in extreme emergency.**

---

### 11) SET_SPEED ⭐ new
```json
{"action": "SET_SPEED", "params": {"speed": 3.0}}
```
| Param | Type | Range | Description |
|-------|------|-------|-------------|
| `speed` | float (m/s) | 0.5 – 15.0 | New default horizontal speed |

Also updates `default_speed_z = speed × 0.6`.  
Affects `MOVE_REL`, `GOTO_NED`, and `ORBIT` related default speed behavior.

---

### 12) FIND_AND_GOTO ⭐ new (v2.2)
```json
{"action": "FIND_AND_GOTO", "params": {"query": "person wearing yellow clothes"}}
```
| Param | Type | Required | Description |
|-------|------|---------|-------------|
| `query` | string | ✅ 必填 | 用英文描述目标的外观特征，如 `"red car"` / `"person wearing yellow jacket"` |

**行为流程**：

1. bridge 发布 `query` 到 `/uav/target_query` → 触发 YOLO-World GPU 实时检测
2. 无人机悬停等待，状态置为 `SEARCHING`
3. 收到 `/uav/semantic_targets_world` 中置信度 ≥ 0.15 的目标后，取最高分目标
4. 坐标转换：ENU world（`x_world=East, y_world=North`）→ NED（`x=North, y=East`）
5. 自动转为内部 `GOTO_NED`，在当前无人机高度飞至目标 XY 坐标上方
6. 到达后进入 HOVER 状态

**超时机制**：20 秒内未检测到置信度 ≥ 0.15 的目标，状态转为 `SEARCH_FAILED` 并回落 HOVER。

**支持别名**（LLM 可使用任意一个）：
`FIND`, `SEARCH`, `LOCATE`, `FLY_TO_TARGET`, `SEEK`, `VISUAL_GOTO`, `FIND_AND_FLY`, `TRACK_AND_GOTO`

**依赖**：需同时运行 `semantic_perception.launch.py`（yolo_world_detector + target_grounding_node + semantic_target_tf_node）。

---

## Chinese Text Commands (back-compat)

| Text | Equivalent action |
|------|------------------|
| `起飞 [alt]` | `TAKEOFF` |
| `降落` | `LAND` |
| `悬停 [dur]` | `HOVER` |
| `前进 [dur] [spd]` | `MOVE_VELOCITY vx=+` |
| `后退 [dur] [spd]` | `MOVE_VELOCITY vx=−` |
| `左移 [dur] [spd]` | `MOVE_VELOCITY vy=−` |
| `右移 [dur] [spd]` | `MOVE_VELOCITY vy=+` |
| `上升 [dur] [spd]` | `MOVE_VELOCITY vz=−` |
| `下降 [dur] [spd]` | `MOVE_VELOCITY vz=+` |
| `左转 [dur] [rate]` | `MOVE_VELOCITY yaw_rate=−` |
| `右转 [dur] [rate]` | `MOVE_VELOCITY yaw_rate=+` |
| `返航` | `RTL` |
| `紧急停止` / `急停` | `EMERGENCY_STOP` |
| `设速 [m/s]` | `SET_SPEED` |

---

## Status Schema (`/uav/llm_task_status`)

### Event types

| status | When emitted |
|--------|-------------|
| `READY` | Node startup |
| `RECEIVED` | Command string received on topic |
| `MAPPED` | Parsed and dispatched to controller |
| `PUBLISHED` | First velocity/command published for this action |
| `TELEMETRY` | 5 Hz heartbeat with full drone state |
| `UNKNOWN_COMMAND` | Unrecognized action (falls back to HOVER) |
| `SEARCH_FOUND` | FIND_AND_GOTO 找到目标，即将飞往 |
| `SEARCH_FAILED` | FIND_AND_GOTO 超时或 query 为空 |

### TELEMETRY payload (enriched for LLM context)

```json
{
  "status": "TELEMETRY",
  "command": "SEARCHING",
  "ts": 1774123456789,
  "position":    {"x": 3.1, "y": 1.2, "z": -5.9},
  "velocity":    {"vx": 0.0, "vy": 0.0, "vz": 0.0},
  "heading_deg": 45.0,
  "arming_state": 2,
  "nav_state":   14,
  "target_z_ned": null,
  "default_speed_xy": 4.0,
  "searching_query": "person wearing yellow clothes",
  "search_remaining_sec": 14.3
}
```

正常 GOTO_NED 时包含：

```json
{
  "status": "TELEMETRY",
  "command": "GOTO_NED",
  "goto_target":     {"x": 10.0, "y": 0.0, "altitude": 6.0},
  "goto_distance_m": 6.9
}
```

---

## Coordinate System Notes

| Axis | PX4 NED | ROS ENU |
|------|---------|---------|
| Forward | +x (North) | +x (East) |
| Right    | +y (East)  | +y (North) |
| Up       | −z         | +z |

使用约定应额外注意：

1. `MOVE_VELOCITY` / `MOVE_REL` 的 `vx/vy` 是**机体坐标**，不是 NED 世界坐标。
2. `MOVE_VELOCITY` / `MOVE_REL` 的 `vz/dz` 仍沿用 NED：正=下，负=上。
3. `GOTO_NED` / `ORBIT` 的 `x/y` 使用 PX4 NED 世界坐标。
4. `altitude` 参数始终使用 ENU 高度正值，内部统一转成 NED `z = -altitude`。
5. `FIND_AND_GOTO` 的 world 目标来自 `semantic_target_tf_node`，坐标为 ENU；bridge 内部统一完成 ENU→NED 转换后才调用 GOTO_NED。

这也是为什么用户说"向前飞"应转成机头方向运动，而"飞到 NED 点"则不随当前朝向改变。

---

## Safety Rules (enforced in code)

1. Unknown action → hover + `UNKNOWN_COMMAND` status (no exception)
2. All velocities hard-clamped: horizontal ≤ 15 m/s, vertical ≤ 5 m/s, yaw ≤ 1.5 rad/s
3. `GOTO_NED` / `ORBIT` require `has_local_position = True` before accepting
4. `RTL` / `EMERGENCY_STOP` bypass velocity layer, go direct to PX4 `VehicleCommand`
5. `flight_enabled` gate in `move_velocity` ensures no PX4 signal until explicit TAKEOFF
6. `FIND_AND_GOTO` with empty query → falls back to HOVER; timeout 20s → HOVER + SEARCH_FAILED
7. Any non-FIND_AND_GOTO command received while searching cancels the active search immediately
