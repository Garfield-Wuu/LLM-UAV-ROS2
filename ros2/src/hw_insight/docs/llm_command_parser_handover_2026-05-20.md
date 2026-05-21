# LLM 无人机自然语言指令解析交接文档

> 日期：2026-05-20  
> 项目：`hw_insight` / LLM-UAV-ROS2  
> 交接主题：自然语言飞控指令解析稳定性、LLM 幻觉控制与生产化改造方案  
> 当前结论：现有链路可用于 AirSim/PX4 仿真演示与工程验证，但不建议直接进入生产或真机无人值守环境。

---

## 1. Executive Summary

当前项目已经具备“自然语言/JSON 指令 → ROS 2 任务桥接 → PX4 Offboard → AirSim 仿真飞行”的最小闭环，并叠加了 YOLO-World 语义感知、`FIND_AND_GOTO` 视觉搜索动作、EGO-Planner 仿真规划桥接等能力。

本轮排查的核心问题不是单个 prompt 写法，而是：**LLM 直接输出飞控 action 与底层参数时，容易在方向、坐标系、速度、距离、视觉目标属性上产生不可接受的不稳定性**。这类错误在无人机控制场景中属于安全关键风险，不能仅靠提示词修复。

建议将当前链路升级为：

```text
用户自然语言
  -> 确定性基础指令解析 fast path
  -> LLM intent parser（只输出语义槽位，不输出飞控参数）
  -> intent normalizer / consistency guard
  -> command compiler（代码生成 MOVE_REL / MOVE_VELOCITY / GOTO_NED 等）
  -> safety validator
  -> text_command_bridge / move_velocity / PX4
```

短期目标是把 LLM 从“直接控制飞控参数”降级为“语义理解模块”，由确定性代码负责坐标、符号、速度、时长和安全边界。

---

## 2. 项目背景与当前阶段

### 2.1 项目定位

项目面向低空经济无人机自主决策系统，目标场景包括物流、巡检、低空任务执行等。当前默认技术栈为：

- 仿真：AirSim + Unreal Engine
- 飞控：PX4 Autopilot SITL
- 通信：ROS 2 Humble + `px4_msgs` + uXRCE-DDS
- LLM：Groq / Ollama 双后端，当前黑盒测试使用远端 Ollama `gemma3:4b`
- 视觉：YOLO-World + AirSim RGB / DepthPlanar
- 规划：EGO-Planner 仿真链路已可联调，ENU→NED 速度桥已实现

### 2.2 当前已实现能力

已实现并可联调的能力包括：

- 自然语言或 JSON 指令转飞行动作
- PX4 Offboard 执行闭环
- 多步 `plan` 顺序执行
- TUI 状态监控与 `/uav/llm_task_status` 遥测回传
- `TAKEOFF`、`LAND`、`HOVER`、`MOVE_REL`、`MOVE_VELOCITY`、`GOTO_NED`、`ORBIT`、`YAW_TO`、`RTL`、`EMERGENCY_STOP`、`SET_SPEED`、`FIND_AND_GOTO` 等动作协议
- YOLO-World 开放词汇检测、深度视觉定位支撑、目标 world 坐标发布（论文中仅作为支撑能力）
- `FIND_AND_GOTO -> /uav/target_query -> YOLO -> semantic target -> GOTO_NED` 基础闭环
- EGO-Planner AirSim 仿真侧规划桥接

### 2.3 尚未完成能力

当前尚未完成或不成熟的能力：

- AirSim/PX4 odom 下的坐标一致性与外参标定
- 真机级闭环与真实传感器标定
- 独立 mission manager / 任务抢占 / 优先级调度
- LLM intent schema 与任务原语接口包
- 视觉查询与视觉执行动作分离
- 大规模自然语言回归测试集
- LLM 输出一致性校验、反幻觉检测、生产级 safety validator

---

## 3. 当前运行链路

### 3.1 已实现主链

```text
用户自然语言 / JSON
  -> llm_client.py
     - Groq / Ollama 推理
     - 当前 prompt 注入动作白名单、坐标规则、遥测状态
     - JSON 提取、参数 schema 初步转换
     - 发布 /uav/user_command
  -> text_command_bridge.py
     - 解析 JSON action
     - 机体系 vx/vy 转 NED world
     - TAKEOFF / GOTO_NED / ORBIT / YAW_TO / FIND_AND_GOTO 状态逻辑
     - 发布 /hw_insight/keyboard_velocity
  -> move_velocity.py
     - PX4 Offboard 心跳
     - 解锁与模式切换
     - 发布 TrajectorySetpoint / VehicleCommand
  -> PX4 SITL + AirSim
```

### 3.2 语义感知链

```text
FIND_AND_GOTO {"query": "..."}
  -> text_command_bridge 发布 /uav/target_query
  -> yolo_world_detector 按 query 推理
  -> /uav/detections_2d
  -> target_grounding_node
  -> /uav/semantic_targets_camera
  -> semantic_target_tf_node
  -> /uav/semantic_targets_world
  -> bridge 转 GOTO_NED 或 /uav/target_goal
```

当前 `planner_integration.launch.py` / `uav_sim.launch.py` 不内置 YOLO，需要另启 `semantic_perception.launch.py`。

---

## 4. 本轮问题现象

### 4.1 下降指令方向错误

用户输入“降低3米”，LLM 输出：

```json
{"action":"MOVE_VELOCITY","params":{"vx":0.0,"vy":0.0,"vz":-1.0,"yaw_rate":0.0,"duration":2.0}}
```

问题：

- 当前 NED 约定中，`vz < 0` 表示上升，`vz > 0` 表示下降。
- 模型把“下降”输出成了上升。
- “3米”被转成 `1m/s * 2s = 2m`，位移量也错误。

结论：LLM 直接输出 `vz` 和 `duration` 不可靠。

### 4.2 视觉查询出现示例污染

用户输入“前面有人吗”，模型输出：

```json
{"action":"FIND_AND_GOTO","params":{"query":"person wearing yellow clothes"}}
```

问题：

- prompt 示例中曾写有 `person wearing yellow clothes`。
- 模型把示例属性“yellow clothes”带入用户未提到的查询。
- 用户是“询问是否有人”，不应触发“找到并飞过去”。

结论：视觉查询与视觉执行混用，且示例污染会导致目标属性幻觉。

### 4.3 右转/左转方向与底层符号不一致

用户输入“右转90度”，上层经过兜底输出：

```json
{"action":"MOVE_VELOCITY","params":{"yaw_rate":0.6,"duration":2.62}}
```

实测出现反向转动。排查发现：

- 上层语义约定：右转 / 顺时针 `yaw_rate > 0`
- `move_velocity.py` 到 PX4 yawspeed 边界存在符号取反问题
- 当前 AirSim + PX4 SITL 栈的实际偏航符号需要在硬件边界统一处理

结论：偏航符号不应由 LLM 负责，必须由代码在固定边界处理。

### 4.4 左右平移被误解为前后

用户输入“左飞5米”，模型输出：

```json
{"action":"MOVE_REL","params":{"dx":5.0,"dy":0.0,"dz":0.0,"duration":2.5}}
```

问题：

- `dx=5` 表示向前，不是向左。
- 模型对“左飞/右飞/往左靠/朝右挪”等自然语言变体不稳定。

结论：LLM 直接输出 `dx/dy` 不可靠。

---

## 5. 已尝试的修复方向

### 5.1 Prompt 优化

已将 prompt 从具体示例改为更中性的协议说明：

- 去除 `person wearing yellow clothes` 这类污染示例
- 明确视觉查询和视觉执行的区别
- 明确 NED 坐标符号
- 明确相对动作和绝对动作的使用边界
- 强制纯 JSON 输出

效果：

- 对部分视觉 hallucination 有改善。
- 但模型仍会在左右、转向、下降等安全关键参数上出错。

结论：prompt 优化必要但不充分。

### 5.2 规则兜底

已在 `llm_client.py` 中加入部分确定性解析逻辑，覆盖：

- 起飞、降落、返航、急停、悬停
- 前/后/左/右/上/下相对位移
- 左/右转角
- 阿拉伯数字和简单中文数字

命中规则时直接输出 `RULE`，跳过 LLM。

效果：

- 明确基础飞控指令更稳定。
- 但用户模糊表达仍需要 LLM 理解，如“稍微靠左一点”“有点太高了”“去那边看看”。

结论：规则 fast path 适合基础指令，但不能覆盖所有自然语言。

### 5.3 黑盒测试：直接 action prompt

测试环境：

- 后端：Ollama
- 主机：`http://39.108.60.130:6300`
- 模型：`gemma3:4b`
- 温度：0.1 或 0.0
- 输出格式：Ollama `format=json`

典型测试结果：

| 输入 | 模型输出摘要 | 评价 |
|---|---|---|
| `左飞5米` | `MOVE_REL dx=-5, dy=0` | 错，把左飞映射到前后轴 |
| `往左边靠一点` | `MOVE_REL dx=-2, dy=0` | 错 |
| `朝右边挪三米` | `MOVE_REL dx=3, dy=0` | 错 |
| `右转90度` | `MOVE_VELOCITY yaw_rate=-10` | 错，方向和速度均危险 |
| `顺时针转一下` | `yaw_rate=-10` | 错 |
| `降低三米` | `SET_SPEED speed=-1` | 错，动作类型错误 |
| `有点太高了` | `SET_SPEED speed=0.5` | 错，语义不安全 |
| `前面有人吗` | `HOVER` | 本次正确，但依赖 prompt |
| `飞到前面那个人旁边` | `FIND_AND_GOTO query=person` | 可接受 |
| `去那边看看` | `FIND_AND_GOTO query=over there` | 错，模糊目标不应移动 |

结论：直接让 `gemma3:4b` 输出飞控 action，不适合生产。

### 5.4 黑盒测试：intent prompt

将 LLM 输出改为 intent / slot，而不是 action 参数：

```json
{
  "intent": "relative_move",
  "direction": "left",
  "distance_m": 5,
  "turn_direction": null,
  "angle_deg": null,
  "query": null,
  "should_move": true,
  "needs_clarification": false,
  "confidence": 0.95
}
```

intent v2 测试效果明显更好：

| 输入 | 输出摘要 | 评价 |
|---|---|---|
| `左飞5米` | `relative_move direction=left distance=5` | 正确 |
| `往左边靠一点` | `relative_move direction=left magnitude=small` | 正确 |
| `朝右边挪三米` | `relative_move direction=right distance=3` | 正确 |
| `右转90度` | `relative_turn turn_direction=right angle=90` | 正确 |
| `顺时针转一下` | `relative_turn turn_direction=right magnitude=small` | 正确 |
| `降低三米` | `relative_move direction=down distance=3` | 正确 |
| `有点太高了` | 一版曾误判，增强反例后可修正 | 需要 guard |
| `前面有人吗` | `visual_search query=person should_move=false` | 正确 |
| `飞到前面那个人旁边` | `visual_goto query=person should_move=true` | 正确 |
| `去那边看看` | 不稳定，有时 `unknown`，有时 `visual_search` | 需要 guard |

结论：intent 方案显著优于 action 方案，但仍必须加代码校验和一致性修正。

---

## 6. 当前主要风险

### 6.1 Safety Critical 风险

当前最大风险是 LLM 可能直接生成错误飞控参数：

- 左右轴错误
- 上下符号错误
- 偏航方向错误
- 速度过大
- duration 不合理
- 模糊指令被强行执行
- 视觉询问误触发飞向目标

这些错误在仿真中表现为飞错方向，在真机环境中可能导致碰撞、越界或失控。

### 6.2 Prompt 依赖过强

当前 prompt 同时承担：

- 协议说明
- 坐标系约束
- 动作选择
- 安全约束
- 视觉任务边界
- 模糊指令处理

这导致 prompt 越长，模型越可能遗忘或冲突。`gemma3:4b` 黑盒测试中已经观察到：增加规则和反例后，部分样本反而回归。

### 6.3 缺少统一安全校验层

当前已有部分参数裁剪，但仍不完整：

- `vz`、`yaw_rate`、`duration`、`dx/dy/dz` 等没有形成统一 validator
- plan 路径和单步路径的校验逻辑不完全一致
- 视觉 query 的反幻觉校验不足
- `needs_clarification` 机制尚未接入执行层

### 6.4 测试集不足

当前测试主要是人工黑盒小样本，尚未形成：

- 200+ 基础指令变体集
- 视觉查询/视觉执行边界集
- 模糊指令拒绝/澄清集
- 多步 plan 集
- 负样本与危险指令集
- 自动评测脚本和 CI gate

---

## 7. 推荐目标架构

### 7.1 核心原则

1. **LLM 不直接输出飞控参数。**  
   LLM 只输出 intent 与语义槽位。

2. **坐标、符号、速度、时长由代码编译。**  
   `direction=left` 到 `dy=-distance` 的映射必须是确定性代码。

3. **模糊指令默认保守。**  
   可安全默认的“小幅调整”可以执行；目标、方向、动作不明确时必须追问或 HOVER。

4. **视觉查询和视觉执行必须分离。**  
   “有人吗”是 `visual_search`，不移动；“飞到那个人旁边”才是 `visual_goto`。

5. **所有 LLM 输出必须过 normalizer + validator。**  
   不允许模型输出直接进入执行层。

### 7.2 建议模块拆分

```text
llm_client.py
  -> 只负责 I/O、provider 调用、TUI、调度

rule_parser.py
  -> 基础确定性飞控指令 fast path

llm_intent_parser.py
  -> 调用 LLM，输出 intent JSON

intent_schema.py
  -> intent 字段、枚举、默认值、类型校验

intent_normalizer.py
  -> 修正 "null" 字符串、英文 query、枚举归一化

consistency_guard.py
  -> 原文与 intent 一致性检查

command_compiler.py
  -> intent -> 现有 action JSON

safety_validator.py
  -> 动作白名单、限速、限高、限时、地理围栏、视觉动作边界

regression/
  -> 自然语言样本集 + 期望 intent/action + 自动评测
```

### 7.3 intent schema 建议

```json
{
  "intent": "relative_move",
  "direction": "left",
  "distance_m": 5.0,
  "turn_direction": null,
  "angle_deg": null,
  "altitude_m": null,
  "speed_mps": null,
  "magnitude": null,
  "query": null,
  "should_move": true,
  "needs_clarification": false,
  "confidence": 0.95
}
```

允许 intent：

- `takeoff`
- `land`
- `hover`
- `return_home`
- `emergency_stop`
- `relative_move`
- `relative_turn`
- `set_speed`
- `goto_position`
- `visual_search`
- `visual_goto`
- `unknown`

### 7.4 compiler 映射示例

| intent | 输入槽位 | 编译结果 |
|---|---|---|
| `relative_move` | `direction=left, distance_m=5` | `MOVE_REL dy=-5` |
| `relative_move` | `direction=down, distance_m=3` | `MOVE_REL dz=3` |
| `relative_move` | `direction=left, magnitude=small` | `MOVE_REL dy=-1` |
| `relative_turn` | `turn_direction=right, angle_deg=90` | `MOVE_VELOCITY yaw_rate=0.6 duration=2.62` |
| `visual_search` | `query=person, should_move=false` | 只触发感知或状态查询，不进入 `FIND_AND_GOTO` |
| `visual_goto` | `query=person, should_move=true` | `FIND_AND_GOTO query=person` |

### 7.5 consistency guard 规则

建议至少实现：

- 原文含“左”，LLM 输出 `direction=right/forward` 时拒绝或纠正。
- 原文含“右”，LLM 输出 `direction=left/forward` 时拒绝或纠正。
- 原文含“降低/下降/太高”，LLM 输出 `direction=up` 时纠正为 `down`。
- 原文含“上升/升高/太低”，LLM 输出 `direction=down` 时纠正为 `up`。
- 原文含“有人吗/看到吗/是什么”，`should_move` 必须为 false。
- `intent=unknown` 或 `needs_clarification=true` 时，`should_move` 必须为 false。
- query 中出现原文未提到的颜色、衣服、数量、位置属性时，降级为基础目标词或拒绝。
- confidence 低于阈值时不执行，返回 HOVER 或追问。

---

## 8. 生产化评估

### 8.1 当前是否可生产

不建议。

当前状态适合：

- AirSim 仿真演示
- PX4 SITL 工程验证
- LLM 指令解析方案验证
- 视觉链路与规划链路集成测试

当前不适合：

- 真机无人值守
- 有人/障碍物附近自主执行
- 低空经济真实业务场景
- 无人工接管的自动任务

### 8.2 生产前 Release Gate 建议

至少满足：

- 基础动作解析准确率 > 99%
- intent 解析综合准确率 > 95%
- 所有失败样本必须安全降级为 HOVER / 拒绝 / 追问
- 视觉查询不得误触发飞行动作
- LLM 不得直接输出底层速度、坐标符号、yaw_rate
- 所有 action 通过统一 safety validator
- AirSim 回归测试覆盖 200+ 自然语言样本
- SITL 连续回归无崩溃、无意外解锁、无越界动作
- 人工接管 / LAND / RTL / EMERGENCY_STOP 始终最高优先级

---

## 9. 建议落地计划

### Phase 1：止血与稳定基线

目标：保证基础飞控指令不再依赖 LLM。

任务：

- 保留 rule parser fast path
- 固化起降、悬停、返航、急停、相对位移、相对转向解析
- 给 `move_velocity` 的 yaw sign 建立仿真测试和文档说明
- 建立 50 条基础指令回归集

验收：

- 基础指令不出现 LLM 调用
- 左右、上下、转向 100% 通过小样本回归

### Phase 2：引入 intent parser

目标：让 LLM 只输出 intent。

任务：

- 新建 intent schema
- 新建 intent prompt
- 新建 normalizer
- 新建 compiler
- `llm_client` 增加 `parser_mode:=action|intent` 参数，先灰度 intent

验收：

- `gemma3:4b` 在 100 条 intent 测试中综合准确率 > 90%
- 错误样本不直接进入执行层

### Phase 3：安全校验与反幻觉

目标：把错误挡在执行层之前。

任务：

- 新建 `safety_validator.py`
- 统一单步和 plan 校验
- 增加 query 反幻觉校验
- 增加 `needs_clarification` 响应机制
- 增加视觉查询不移动动作

验收：

- 视觉询问不会触发 `FIND_AND_GOTO`
- 模糊指令不会被强制移动
- 超限速度、超限高度、非法 duration 被拒绝或裁剪

### Phase 4：测试体系与 CI

目标：建立可持续评估能力。

任务：

- 建立 `tests/fixtures/nl_commands/*.jsonl`
- 包含基础动作、模糊动作、视觉任务、危险指令、多步任务
- 自动调用 Ollama/Groq 或 mock LLM
- 统计 intent accuracy、safe failure rate、JSON validity、latency

验收：

- 回归集 > 200 条
- JSON validity > 99%
- safe failure rate = 100%
- CI 可复现

---

## 10. 建议优先级

P0：

- 禁止 LLM 直接输出飞控底层参数进入执行层。
- 实现 intent parser + compiler 的最小版本。
- 增加统一 safety validator。

P1：

- 视觉查询与视觉执行拆分。
- 引入 `needs_clarification` 的 TUI 反馈。
- 建立自然语言回归测试集。

P2：

- 多轮上下文与任务记忆。
- 多目标选择策略。
- AirSim/PX4 odom 基线下的坐标稳定性评估。
- EGO-Planner 与语义目标任务级编排。

---

## 11. 需要高级工程师重点决策的问题

1. 是否接受 intent 中间层作为正式架构方向？
2. intent schema 是否应放入 `uav_interfaces`，未来升级为 ROS 2 msg/srv？
3. 视觉查询是否新增独立 action，例如 `VISUAL_QUERY` / `LOOK_FOR`？
4. 模糊指令是默认小幅动作，还是默认追问？
5. 生产目标模型是否继续使用 `gemma3:4b`，还是切换更强模型并做本地量化部署？
6. safety validator 是放在 `llm_client` 内，还是独立为 `uav_safety` 节点？
7. `FIND_AND_GOTO` 是否应走 planner target goal，而不是直接 `GOTO_NED`？

---

## 12. 当前临时改动说明

本轮会话中已经进行过若干试验性修复：

- 优化 `llm_client.py` 当前 action prompt，去除视觉示例污染。
- 加入基础飞控规则解析 fast path。
- 加入相对平移/相对高度/相对转向兜底。
- 调整 `move_velocity.py` yaw 边界符号处理。
- 对远端 Ollama `gemma3:4b` 做黑盒 prompt 对比测试。

这些改动改善了部分仿真表现，但不应视为最终生产架构。建议后续以 intent parser / compiler / validator 重构当前试验代码。

---

## 13. 附录：推荐 intent prompt 方向

建议后续 prompt 只要求模型输出语义槽位，不出现飞控 action 和底层参数：

```text
你是无人机自然语言意图解析器，只输出一个 JSON 对象。
你不能输出飞控 action，也不能输出 dx/dy/dz/vx/vy/vz/yaw_rate。
你的任务是把用户话语解析为安全的中间语义 intent，后续程序会把 intent 编译成飞控命令。

允许 intent：
takeoff, land, hover, return_home, emergency_stop,
relative_move, relative_turn, set_speed, goto_position,
visual_search, visual_goto, unknown

字段：
intent, direction, distance_m, turn_direction, angle_deg,
altitude_m, speed_mps, magnitude, query,
should_move, needs_clarification, confidence

规则：
1. 用户只是问“有没有/看一下/前面有人吗/是什么”时，用 visual_search，should_move=false。
2. 用户明确说“飞到/靠近/前往/跟随 + 具体目标”时，用 visual_goto，should_move=true。
3. “左/右/前/后/上/下 + 飞/移/挪/靠/走/移动/一点”是 relative_move。
4. 只有出现“转/旋/顺时针/逆时针/航向”才是 relative_turn。
5. 目标、方向或动作不明确时 unknown + needs_clarification=true + should_move=false。
6. query 只保留用户明确提到的目标属性，不得补充颜色、衣服、数量。
```

注意：即使使用上述 prompt，仍必须由代码做 normalizer、consistency guard 和 safety validator。

