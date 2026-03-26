# UAV Command Protocol v2.0

> **For LLM integration** — all actions use JSON over `/uav/user_command`.  
> Designed to be directly callable by Groq / Ollama planners.

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
1. ARM phase (0.8 s): sends z = −6.0 m/s → triggers `move_velocity` arm + Offboard switch  
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
| `vx` | float (m/s) | NED forward (+) / back (−) |
| `vy` | float (m/s) | NED right (+) / left (−) |
| `vz` | float (m/s) | NED down (+) / up (−) |
| `yaw_rate` | float (rad/s) | CW positive in NED |
| `duration` | float (s) | After duration → IDLE |

---

### 5) MOVE_REL
```json
{"action": "MOVE_REL", "params": {"dx": 5.0, "dy": 0.0, "dz": 0.0, "duration": 2.5}}
```
Converts relative NED displacement to `MOVE_VELOCITY` (vx = dx/duration, etc.).

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
Affects MOVE_VELOCITY default, GOTO_NED, and ORBIT.

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

### TELEMETRY payload (enriched for LLM context)

```json
{
  "status": "TELEMETRY",
  "command": "GOTO_NED",
  "ts": 1774123456789,
  "position":    {"x": 3.1, "y": 1.2, "z": -5.9},
  "velocity":    {"vx": 1.5, "vy": 0.0, "vz": 0.0},
  "heading_deg": 45.0,
  "arming_state": 2,
  "nav_state":   14,
  "target_z_ned": -6.0,
  "default_speed_xy": 4.0,
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

All `GOTO_NED` x/y use PX4 NED. `altitude` parameters always use ENU (positive = up) for human readability; internally converted to NED z = −altitude.

---

## Safety Rules (enforced in code)

1. Unknown action → hover + `UNKNOWN_COMMAND` status (no exception)
2. All velocities hard-clamped: horizontal ≤ 15 m/s, vertical ≤ 5 m/s, yaw ≤ 1.5 rad/s
3. `GOTO_NED` / `ORBIT` require `has_local_position = True` before accepting
4. `RTL` / `EMERGENCY_STOP` bypass velocity layer, go direct to PX4 `VehicleCommand`
5. `flight_enabled` gate in `move_velocity` ensures no PX4 signal until explicit TAKEOFF
