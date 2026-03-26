#!/usr/bin/env python3
"""
LLMClient — Natural language → UAV flight commands via Groq / Ollama.

Data flow:
  stdin  ─────────────┐
  /uav/nl_input ──────┤──▶  build_prompt  ──▶  LLM call  ──▶  validate  ──▶  /uav/user_command
                      │
  /uav/llm_task_status ──▶  TELEMETRY state (LLM context)

Design decisions:
  - Pure stdlib HTTP (urllib.request) — zero extra Python deps, works offline for Ollama
  - ThreadPoolExecutor(max_workers=1): LLM calls run off the ROS2 spin thread, serialised
    to prevent overlapping requests
  - JSON passthrough: auto-inserted follow-up commands skip LLM entirely
  - Safety layer clamps altitude/speed and blocks unknown actions before publishing
  - Auto-TAKEOFF: if drone is GROUND and a flight action arrives, TAKEOFF is inserted first
"""

import json
import math
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── Preset model lists for interactive selection ──────────────────────────────
_GROQ_PRESETS = [
    ('llama-3.3-70b-versatile', '综合最优，推荐用于复杂任务规划'),
    ('llama-3.1-8b-instant',    '极速响应，适合简单控制指令'),
    ('mixtral-8x7b-32768',      '混合专家模型，长上下文推理'),
    ('gemma2-9b-it',            'Google Gemma2，轻量高效'),
]

_OLLAMA_PRESETS = [
    ('llama3.2',         '轻量通用模型，3B 参数'),
    ('qwen2.5:7b',       '通义千问 7B，中文优化'),
    ('qwen3-coder:30b',  '千问编程 30B，复杂规划首选'),
    ('mistral:7b',       'Mistral 7B，通用推理'),
    ('deepseek-r1:7b',   'DeepSeek-R1，推理增强'),
]

# ── Allowed action set (must match COMMAND_PROTOCOL.md v2.0) ─────────────────
ALLOWED_ACTIONS = frozenset({
    'TAKEOFF', 'LAND', 'HOVER', 'MOVE_VELOCITY', 'MOVE_REL',
    'GOTO_NED', 'ORBIT', 'YAW_TO', 'RTL', 'EMERGENCY_STOP', 'SET_SPEED',
})

# Safety-critical actions are never blocked by altitude/speed validation
SAFETY_PASS_ACTIONS = frozenset({'RTL', 'EMERGENCY_STOP', 'LAND', 'HOVER'})

# Telemetry freshness threshold: if no message within this many seconds, system is OFFLINE
_TELEMETRY_STALE_SEC = 3.0

# ── Task primitive alias library ──────────────────────────────────────────────
# Maps hallucinated / paraphrased action names → canonical action name.
# Keys are UPPERCASE; matching is case-insensitive in the extractor.
_ACTION_ALIASES: Dict[str, str] = {
    # TAKEOFF variants
    'TAKE_OFF':        'TAKEOFF',
    'LIFTOFF':         'TAKEOFF',
    'LAUNCH':          'TAKEOFF',
    'ASCEND':          'TAKEOFF',
    # LAND variants
    'LANDING':         'LAND',
    'DESCEND':         'LAND',
    'SET_DOWN':        'LAND',
    # HOVER variants
    'STOP':            'HOVER',
    'WAIT':            'HOVER',
    'HOLD':            'HOVER',
    'STAY':            'HOVER',
    'PAUSE':           'HOVER',
    # MOVE_REL variants
    'MOVE':            'MOVE_REL',
    'MOVE_FORWARD':    'MOVE_REL',
    'MOVE_RELATIVE':   'MOVE_REL',
    'FLY':             'MOVE_REL',
    'FLY_RELATIVE':    'MOVE_REL',
    'GO':              'MOVE_REL',
    'TRANSLATE':       'MOVE_REL',
    # GOTO_NED variants
    'GOTO':            'GOTO_NED',
    'GO_TO':           'GOTO_NED',
    'FLY_TO':          'GOTO_NED',
    'NAVIGATE':        'GOTO_NED',
    'NAVIGATE_TO':     'GOTO_NED',
    'WAYPOINT':        'GOTO_NED',
    # MOVE_VELOCITY variants
    'VELOCITY':        'MOVE_VELOCITY',
    'SET_VELOCITY':    'MOVE_VELOCITY',
    'MOVE_VEL':        'MOVE_VELOCITY',
    # YAW_TO variants
    'ROTATE':          'YAW_TO',
    'TURN':            'YAW_TO',
    'YAW':             'YAW_TO',
    'HEADING':         'YAW_TO',
    # RTL variants
    'RETURN':          'RTL',
    'RETURN_HOME':     'RTL',
    'GO_HOME':         'RTL',
    'HOME':            'RTL',
    # EMERGENCY_STOP variants
    'EMERGENCY':       'EMERGENCY_STOP',
    'ABORT':           'EMERGENCY_STOP',
    'KILL':            'EMERGENCY_STOP',
    'E_STOP':          'EMERGENCY_STOP',
    'ESTOP':           'EMERGENCY_STOP',
}

# ── Parameter schema per action ───────────────────────────────────────────────
# Format: {param_name: (type_fn, min, max, required)}
# type_fn coerces value; min/max are None if unconstrained.
_PARAM_SCHEMA: Dict[str, list] = {
    'TAKEOFF':       [('altitude',  float, 0.5,  None, False)],
    'LAND':          [],
    'HOVER':         [('duration',  float, 0.0,  None, False)],
    'MOVE_VELOCITY': [('vx',        float, None, None, False),
                      ('vy',        float, None, None, False),
                      ('vz',        float, None, None, False),
                      ('yaw_rate',  float, None, None, False),
                      ('duration',  float, 0.0,  None, False)],
    'MOVE_REL':      [('dx',        float, None, None, False),
                      ('dy',        float, None, None, False),
                      ('dz',        float, None, None, False),
                      ('duration',  float, 0.0,  None, False)],
    'GOTO_NED':      [('x',         float, None, None, False),
                      ('y',         float, None, None, False),
                      ('altitude',  float, 0.5,  None, False)],
    'ORBIT':         [('cx',        float, None, None, False),
                      ('cy',        float, None, None, False),
                      ('radius',    float, 1.0,  None, False),
                      ('speed',     float, 0.1,  None, False),
                      ('duration',  float, 0.0,  None, False)],
    'YAW_TO':        [('angle',     float, None, None, True)],
    'RTL':           [],
    'EMERGENCY_STOP': [],
    'SET_SPEED':     [('speed',     float, 0.1,  None, True)],
}

_SYSTEM_PROMPT_TEMPLATE = """\
你是专业无人机飞控 AI。将用户自然语言指令转换为 JSON 飞行指令。

可用 Actions（只能从此列表选择）：
  TAKEOFF        {{"action":"TAKEOFF","params":{{"altitude":6.0}}}}
  LAND           {{"action":"LAND","params":{{}}}}
  HOVER          {{"action":"HOVER","params":{{"duration":5.0}}}}
  MOVE_VELOCITY  {{"action":"MOVE_VELOCITY","params":{{"vx":2,"vy":0,"vz":0,"yaw_rate":0,"duration":3}}}}
  MOVE_REL       {{"action":"MOVE_REL","params":{{"dx":5,"dy":0,"dz":0,"duration":2.5}}}}
  GOTO_NED       {{"action":"GOTO_NED","params":{{"x":10,"y":5,"altitude":6}}}}
  ORBIT          {{"action":"ORBIT","params":{{"cx":0,"cy":0,"radius":5,"speed":2,"duration":30}}}}
  YAW_TO         {{"action":"YAW_TO","params":{{"angle":90}}}}
  RTL            {{"action":"RTL","params":{{}}}}
  EMERGENCY_STOP {{"action":"EMERGENCY_STOP","params":{{}}}}
  SET_SPEED      {{"action":"SET_SPEED","params":{{"speed":3.0}}}}

━━━ 坐标轴方向（严格遵守，错方向等于撞机）━━━
MOVE_VELOCITY / MOVE_REL 使用机体坐标（相对无人机机头方向）：
  方向词     →  参数      符号
  前进/向前  →  vx/dx    正(+)   ← 沿机头方向飞
  后退/向后  →  vx/dx    负(-)   ← 沿机尾方向飞
  右移/向右  →  vy/dy    正(+)   ← 沿机身右侧飞
  左移/向左  →  vy/dy    负(-)   ← 沿机身左侧飞
  上升/向上  →  vz/dz    负(-) ← 垂直上升（z轴朝下，上升为负！）
  下降/向下  →  vz/dz    正(+)   ← 垂直下降
altitude 参数（TAKEOFF/GOTO_NED）始终正值，向上为正，无需转换。
GOTO_NED 的 x/y 是 NED 世界坐标（x=北, y=东），与机头方向无关。

━━━ 安全约束 ━━━
  最大飞行高度：{max_altitude} 米  |  最大速度：{max_speed} m/s

━━━ 当前无人机状态 ━━━
  武装: {arming_state}   飞行阶段: {flight_phase}
{telemetry_block}
━━━ 输出规则（严格执行）━━━
  1. 单步指令 → 输出一行纯 JSON：{{"action":"...","params":{{...}}}}
  2. 多步指令 → 输出 plan 格式（系统会按序执行，每步完成后再执行下一步）：
     {{"plan":[{{"action":"...","params":{{...}}}},{{"action":"...","params":{{...}}}}]}}
  3. 指令不明确或不安全 → 返回 {{"action":"HOVER","params":{{}}}}
  4. 无人机在地面(GROUND)且用户要求飞行 → plan 第一步必须是 TAKEOFF
  5. 禁止任何解释文字，只输出 JSON"""


class LLMClient(Node):
    """LLM inference node: natural language → /uav/user_command JSON."""

    def __init__(self, config_override: Optional[Dict[str, Any]] = None) -> None:
        super().__init__('llm_client')
        cfg = config_override or {}

        # Environment-based defaults (for remote Ollama gateways that expose
        # Anthropic/OpenAI-compatible env contracts).
        env_ollama_model = os.environ.get('ANTHROPIC_MODEL', 'llama3.2')
        env_ollama_host = os.environ.get('ANTHROPIC_BASE_URL', 'http://localhost:11434')

        # ── ROS parameters ───────────────────────────────────────────────────
        self.declare_parameter('llm_provider',          'groq')
        self.declare_parameter('groq_model',            'llama-3.3-70b-versatile')
        self.declare_parameter('groq_api_key',          '')
        self.declare_parameter('ollama_model',          env_ollama_model)
        self.declare_parameter('ollama_host',           env_ollama_host)
        self.declare_parameter('max_altitude_m',        120.0)
        self.declare_parameter('max_speed_ms',          15.0)
        self.declare_parameter('stdin_mode',            True)
        self.declare_parameter('verbose',               False)
        self.declare_parameter('auto_takeoff_altitude', 6.0)
        # Groq 通常数秒内返回；远程 Ollama + 大模型（如 30B）+ 长飞控 prompt 常 >20s，易误报 timed out
        self.declare_parameter('llm_timeout_sec',       120.0)

        p = self.get_parameter
        self._provider     = str(p('llm_provider').value)
        self._groq_model   = str(p('groq_model').value)
        self._ollama_model = str(p('ollama_model').value)
        self._ollama_host  = str(p('ollama_host').value)
        self._max_alt      = float(p('max_altitude_m').value)
        self._max_spd      = float(p('max_speed_ms').value)
        self._stdin_mode   = bool(p('stdin_mode').value)
        self._verbose      = bool(p('verbose').value)
        self._auto_alt     = float(p('auto_takeoff_altitude').value)
        self._timeout      = float(p('llm_timeout_sec').value)

        # ── Apply interactive selection overrides (highest priority) ─────────
        # Priority: interactive selection > ROS params > environment variables
        self._provider     = cfg.get('provider',     self._provider)
        self._groq_model   = cfg.get('groq_model',   self._groq_model)
        self._ollama_model = cfg.get('ollama_model', self._ollama_model)
        self._ollama_host  = cfg.get('ollama_host',  self._ollama_host)

        # Groq API key: interactive > ROS param > env variable
        api_key_param = str(p('groq_api_key').value)
        self._groq_api_key = (
            cfg.get('groq_api_key', '')
            or api_key_param
            or os.environ.get('GROQ_API_KEY', '')
        )

        # ── State ────────────────────────────────────────────────────────────
        self._latest_telemetry: Dict[str, Any] = {}
        self._arming_state = 'UNKNOWN'
        self._flight_phase = 'UNKNOWN'
        self._is_armed     = False
        self._last_telemetry_time: Optional[float] = None  # None = never received
        self._input_q: queue.Queue = queue.Queue()
        self._llm_busy     = False
        self._executor     = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='llm_worker',
        )

        # ── ROS pub/sub ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(String, '/uav/user_command', 10)
        self.create_subscription(
            String, '/uav/llm_task_status', self._on_status, 10,
        )
        self.create_subscription(
            String, '/uav/nl_input', self._on_nl_input, 10,
        )

        # ── Cold-start warm-up flag (Ollama only) ────────────────────────────
        # Ollama 大模型首次推理（冷启动）远比后续慢，先发一条 ping 请求预热；
        # warm-up 完成前不开放用户输入，避免第一条真实指令因超时白费。
        self._warmup_done = (self._provider != 'ollama')

        # ── 10 Hz queue drainer ───────────────────────────────────────────────
        self.create_timer(0.1, self._drain_queue)

        # ── stdin reader thread + warm-up/welcome banner ─────────────────────
        if self._stdin_mode:
            t = threading.Thread(target=self._stdin_loop, daemon=True)
            t.start()
            if self._provider == 'ollama':
                # 启动后台线程做预热，完成后再打印欢迎 Banner
                threading.Thread(target=self._warmup_ollama, daemon=True).start()
            else:
                self._welcome_timer = self.create_timer(0.5, self._print_welcome)

        provider_info = (
            f'Groq / {self._groq_model}' if self._provider == 'groq'
            else f'Ollama / {self._ollama_model} @ {self._ollama_host}'
        )
        self.get_logger().info(
            f'LLMClient ready  [{provider_info}]  llm_timeout_sec={self._timeout:g}'
        )
        if self._provider == 'ollama':
            self.get_logger().info(
                f'Ollama defaults env: ANTHROPIC_BASE_URL={env_ollama_host}, '
                f'ANTHROPIC_MODEL={env_ollama_model}'
            )
        if self._provider == 'groq' and not self._groq_api_key:
            self.get_logger().warn(
                'GROQ_API_KEY 未设置！请 export GROQ_API_KEY=<key> 或'
                ' 传入 --ros-args -p groq_api_key:=<key>'
            )

    # ── ROS subscriptions ─────────────────────────────────────────────────────

    def _on_status(self, msg: String) -> None:
        """Track TELEMETRY for LLM context."""
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('status') != 'TELEMETRY':
            return
        self._latest_telemetry = d
        self._last_telemetry_time = time.time()
        arm = d.get('arming_state', 0)
        self._is_armed     = (arm == 2)
        self._arming_state = 'ARMED' if self._is_armed else 'DISARMED'
        # Derive human-readable flight phase from telemetry
        # Use `or {}` to guard against explicit null values in the JSON payload
        cmd   = d.get('command', 'IDLE')
        pos   = d.get('position') or {}
        alt_m = -float(pos.get('z', 0))  # NED z → ENU alt
        if not self._is_armed or alt_m < 0.3:
            self._flight_phase = 'GROUND'
        elif cmd == 'TAKEOFF':
            self._flight_phase = 'TAKEOFF'
        elif cmd in ('LAND', 'RTL'):
            self._flight_phase = 'LANDING'
        elif cmd in ('HOVER', 'IDLE'):
            self._flight_phase = 'HOVERING'
        else:
            self._flight_phase = 'MOVING'

    def _on_nl_input(self, msg: String) -> None:
        """Accept NL text from ROS topic (programmatic / remote control)."""
        text = msg.data.strip()
        if text:
            self._input_q.put(text)

    # ── stdin reader (daemon thread) ─────────────────────────────────────────

    def _stdin_loop(self) -> None:
        while True:
            try:
                line = sys.stdin.readline()
                if not line:  # EOF
                    break
                text = line.strip()
                if text:
                    self._input_q.put(text)
            except (EOFError, KeyboardInterrupt):
                break

    # ── System online detection ───────────────────────────────────────────────

    def _system_online(self) -> bool:
        """True if a TELEMETRY message was received within _TELEMETRY_STALE_SEC."""
        if self._last_telemetry_time is None:
            return False
        return (time.time() - self._last_telemetry_time) < _TELEMETRY_STALE_SEC

    def _prompt(self) -> str:
        """Return a status-aware input prompt string."""
        if self._system_online():
            arm = '武装' if self._is_armed else '解锁'
            return f'[{self._flight_phase}|{arm}] ▶ '
        if self._last_telemetry_time is None:
            return '[离线] ▶ '
        return '[离线-中断] ▶ '

    def _print_offline_warn(self, action: str = '') -> None:
        """Print a clear offline warning with recovery instructions."""
        action_hint = f'"{action}" ' if action else ''
        print(f'\r\033[K')
        print(f'  ⚠  系统离线 — 指令 {action_hint}已被安全层拦截')
        if self._last_telemetry_time is None:
            print('  原因：从未收到 PX4/AirSim 遥测数据')
        else:
            gap = time.time() - self._last_telemetry_time
            print(f'  原因：遥测中断 {gap:.0f}s（最后收到：{gap:.0f}s 前）')
        print('  请确认以下服务已启动：')
        print('    T1  PX4 SITL         → make px4_sitl_default none_iris')
        print('    T2  XRCE-DDS Agent  → MicroXRCEAgent ...')
        print('    T3  AirSim          → 已在 Windows 端运行')
        print('    T4  ROS 主链        → ros2 launch hw_insight ...')
        print('  安全动作（RTL / EMERGENCY_STOP / LAND / HOVER）可在离线时强制发送。')
        print('  如需绕过安全检查，在指令前加 ! 前缀（如: !紧急降落）')

    def _print_welcome(self) -> None:
        """One-shot welcome banner printed after node finishes initialising."""
        provider_info = (
            f'Groq / {self._groq_model}' if self._provider == 'groq'
            else f'Ollama / {self._ollama_model}'
        )
        online_str = '● 在线' if self._system_online() else '○ 离线（等待遥测）'
        print(f'\n{"─"*62}')
        print(f'  UAV LLM 飞控终端  [{provider_info}]')
        print(f'  系统状态: {online_str}')
        print(f'  输入自然语言指令后按回车发送   |   Ctrl+C 退出')
        print(f'  远程话题: ros2 topic pub /uav/nl_input std_msgs/msg/String ...')
        print(f'{"─"*62}')
        print(self._prompt(), end='', flush=True)
        if hasattr(self, '_welcome_timer'):
            self._welcome_timer.cancel()

    # ── Ollama cold-start warm-up ─────────────────────────────────────────────

    def _warmup_ollama(self) -> None:
        """Send a tiny ping to Ollama before opening user input.

        Ollama 大模型首次调用（冷启动）需要将权重从磁盘加载到 VRAM，
        可能需要 10–60 秒；后续调用通常在 5–10 秒内完成。
        此函数在后台线程内运行预热请求，并用动态进度条告知用户。
        """
        model_name = self._ollama_model
        host = self._ollama_host
        SPIN_CHARS = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'

        # ── 状态行打印 helper（覆写同一行）────────────────────────────────
        def _status(msg: str) -> None:
            print(f'\r\033[K{msg}', end='', flush=True)

        print(f'\n  正在预热 Ollama 模型 [{model_name}]，首次加载请稍候…', flush=True)

        start = __import__('time').time()
        result: dict = {}
        done_evt = threading.Event()

        def _do_request() -> None:
            try:
                payload = json.dumps({
                    'model': model_name,
                    'messages': [{'role': 'user', 'content': 'hi'}],
                    'stream': False,
                    'options': {'temperature': 0, 'num_predict': 4},
                }).encode()
                req = urllib.request.Request(
                    f'{host}/api/chat',
                    data=payload,
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urllib.request.urlopen(req, timeout=int(self._timeout)) as resp:
                    result['ok'] = json.loads(resp.read())
            except Exception as exc:  # noqa: BLE001
                result['err'] = exc
            finally:
                done_evt.set()

        req_thread = threading.Thread(target=_do_request, daemon=True)
        req_thread.start()

        # ── 动态 spinner，每 0.2s 更新一次直到请求完成 ────────────────────
        idx = 0
        while not done_evt.wait(timeout=0.2):
            elapsed = __import__('time').time() - start
            spin = SPIN_CHARS[idx % len(SPIN_CHARS)]
            _status(f'  {spin} 预热中… {elapsed:.0f}s  （首次冷启动，请稍候）')
            idx += 1

        elapsed = __import__('time').time() - start

        if 'err' in result:
            _status('')
            err = result['err']
            print(f'\n  ⚠️  预热失败（{elapsed:.1f}s）：{err}')
            print('  模型可能未加载，首条指令响应可能较慢。仍可正常使用，按 Ctrl+C 退出重试。')
        else:
            _status('')
            print(f'\n  ✅  模型已就绪！预热耗时 {elapsed:.1f}s')

        self._warmup_done = True
        self._print_welcome()

    # ── Queue drainer (10 Hz ROS timer) ──────────────────────────────────────

    def _drain_queue(self) -> None:
        """Pick up one NL input per tick; LLM calls run in background thread."""
        if not self._warmup_done:
            return  # 预热完成前不处理任何输入
        if self._llm_busy:
            return
        try:
            text = self._input_q.get_nowait()
        except queue.Empty:
            return

        # ── Force-override prefix: '!' lets experts bypass offline guard ──────
        # Example: "!紧急降落" or "!EMERGENCY_STOP"
        stripped = text.lstrip()
        force_override = stripped.startswith('!')
        if force_override:
            stripped = stripped[1:].lstrip()
            text = stripped

        # ── JSON passthrough: auto-inserted follow-up commands bypass LLM ─────
        if stripped.startswith('{'):
            try:
                parsed = json.loads(stripped)
                action = str(parsed.get('action', '')).upper()
                if action in ALLOWED_ACTIONS:
                    # Offline guard for non-safety JSON commands
                    if (not self._system_online()
                            and action not in SAFETY_PASS_ACTIONS
                            and not force_override):
                        self._print_offline_warn(action)
                        if self._stdin_mode:
                            print(self._prompt(), end='', flush=True)
                        return
                    self._publish_cmd(parsed)
                    return
                # plan passthrough
                if 'plan' in parsed:
                    if not self._system_online() and not force_override:
                        self._print_offline_warn()
                        if self._stdin_mode:
                            print(self._prompt(), end='', flush=True)
                        return
                    self._llm_busy = True
                    self._executor.submit(self._execute_plan, parsed['plan'])
                    return
            except json.JSONDecodeError:
                pass

        # ── Natural-language input: offline guard (saves LLM quota) ──────────
        if not self._system_online() and not force_override:
            self._print_offline_warn()
            if self._stdin_mode:
                print(self._prompt(), end='', flush=True)
            return

        self._llm_busy = True
        self._executor.submit(self._process_nl, text)

    # ── LLM inference pipeline ────────────────────────────────────────────────

    def _process_nl(self, text: str) -> None:
        """Runs in thread pool: call LLM, validate, then publish or execute plan."""
        try:
            system, user = self._build_messages(text)
            if self._verbose:
                self.get_logger().info(f'[PROMPT SYS]\n{system}')
                self.get_logger().info(f'[PROMPT USR] {user}')

            raw = self._call_llm(system, user)

            # ── Extract and persist <think> reasoning blocks ──────────────
            # DeepSeek-R1, QwQ and similar models embed reasoning in
            # <think>…</think> tags.  We log them for explainability and
            # strip them before JSON extraction so the parser never sees them.
            raw, think = self._extract_think(raw)
            if think:
                self._log_think(text, think)

            if self._verbose:
                self.get_logger().info(f'[LLM RAW] {raw}')

            # Try plan format first, then single command
            plan = self._extract_plan(raw)
            if plan is not None:
                self._execute_plan(plan)
            else:
                cmd = self._parse_and_validate(raw)
                if cmd:
                    self._publish_cmd(cmd)
                else:
                    print(f'\r[LLM] 响应解析失败: {raw[:100]}')
                    if self._stdin_mode:
                        print(self._prompt(), end='', flush=True)

        except urllib.error.HTTPError as exc:
            body = exc.read().decode(errors='replace')
            print(f'\r[LLM HTTP {exc.code}] {body[:120]}')
            self.get_logger().error(f'LLM HTTP {exc.code}: {body[:200]}')
            if self._stdin_mode:
                print(self._prompt(), end='', flush=True)
        except Exception as exc:
            print(f'\r[LLM 错误] {exc}')
            self.get_logger().error(f'LLM call failed: {exc}')
            if self._stdin_mode:
                print(self._prompt(), end='', flush=True)
        finally:
            self._llm_busy = False

    def _build_messages(self, user_text: str) -> Tuple[str, str]:
        """Construct system prompt with current TELEMETRY context."""
        tele = self._latest_telemetry
        if tele:
            pos = tele.get('position') or {}
            vel = tele.get('velocity') or {}
            alt_m = round(-float(pos.get('z', 0)), 1)
            tele_block = (
                f'  位置(NED): x={pos.get("x", 0):.1f}m  '
                f'y={pos.get("y", 0):.1f}m  高度={alt_m}m\n'
                f'  速度: vx={vel.get("vx", 0):.1f}  '
                f'vy={vel.get("vy", 0):.1f}  '
                f'vz={vel.get("vz", 0):.1f} m/s\n'
                f'  航向: {tele.get("heading_deg", 0):.0f}°   '
                f'当前指令: {tele.get("command", "IDLE")}'
            )
        else:
            tele_block = '  （尚未收到遥测 — 请先启动飞控链路）'

        system = _SYSTEM_PROMPT_TEMPLATE.format(
            max_altitude=self._max_alt,
            max_speed=self._max_spd,
            arming_state=self._arming_state,
            flight_phase=self._flight_phase,
            telemetry_block=tele_block,
        )
        return system, user_text

    # ── LLM provider calls ────────────────────────────────────────────────────

    def _call_llm(self, system: str, user: str) -> str:
        if self._provider == 'groq':
            return self._call_groq(system, user)
        return self._call_ollama(system, user)

    def _call_groq(self, system: str, user: str) -> str:
        if not self._groq_api_key:
            raise RuntimeError(
                'GROQ_API_KEY 未设置。请 export GROQ_API_KEY=<key> 或'
                ' --ros-args -p groq_api_key:=<key>'
            )
        payload = json.dumps({
            'model': self._groq_model,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user',   'content': user},
            ],
            'max_tokens': 500,
            'temperature': 0.1,
            # Force pure JSON output — eliminates hallucinated explanation text.
            # Requires the word "json" to appear in the system prompt (satisfied above).
            'response_format': {'type': 'json_object'},
        }).encode()
        req = urllib.request.Request(
            'https://api.groq.com/openai/v1/chat/completions',
            data=payload,
            headers={
                'Authorization': f'Bearer {self._groq_api_key}',
                'Content-Type':  'application/json',
                'User-Agent':    'curl/7.81.0',
            },
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=int(self._timeout)) as resp:
            body = json.loads(resp.read())
        return body['choices'][0]['message']['content'].strip()

    def _call_ollama(self, system: str, user: str) -> str:
        payload = json.dumps({
            'model': self._ollama_model,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user',   'content': user},
            ],
            'stream': False,
            # format:"json" forces pure JSON output on models that support it.
            # Models with <think> reasoning blocks (e.g. DeepSeek-R1) may wrap
            # JSON in reasoning; _extract_json_robust strips <think> blocks.
            'format': 'json',
            'options': {'temperature': 0.1},
        }).encode()
        req = urllib.request.Request(
            f'{self._ollama_host}/api/chat',
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=int(self._timeout)) as resp:
            body = json.loads(resp.read())
        return body['message']['content'].strip()

    # ── Robust JSON extraction ────────────────────────────────────────────────

    @staticmethod
    def _balanced_json_candidates(text: str) -> list:
        """Extract all top-level balanced {...} substrings from text.

        Returns candidates in order of appearance; longest first within ties.
        Handles nested braces correctly.
        """
        candidates = []
        depth = 0
        start = -1
        for i, ch in enumerate(text):
            if ch == '{':
                if depth == 0:
                    start = i
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0 and start != -1:
                    candidates.append(text[start:i + 1])
                    start = -1
        # Sort: longer candidates first (usually contain more context)
        candidates.sort(key=len, reverse=True)
        return candidates

    @staticmethod
    def _strip_fences(text: str) -> str:
        """Remove markdown code fences and <think> blocks; return clean text."""
        import re as _re
        text = _re.sub(r'```(?:json)?\s*', '', text)
        # <think> blocks are logged separately by _extract_think; strip any remainder
        text = _re.sub(r'<think>.*?</think>', '', text, flags=_re.DOTALL)
        return text.strip()

    @staticmethod
    def _extract_think(raw: str) -> Tuple[str, str]:
        """Separate <think>...</think> reasoning blocks from the LLM response.

        Returns:
            (text_without_think, joined_think_content)
        think_content is an empty string when no blocks are present.
        """
        import re
        blocks = re.findall(r'<think>(.*?)</think>', raw, flags=re.DOTALL)
        think = '\n\n'.join(b.strip() for b in blocks if b.strip())
        cleaned = re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()
        return cleaned, think

    def _log_think(self, user_text: str, think: str) -> None:
        """Persist <think> reasoning to ROS2 logger and print a summary to terminal.

        Full reasoning is written to the ROS2 log file (~/.ros/log/...) for
        post-hoc inspection.  The terminal only shows a one-line digest so as
        not to interrupt the conversational flow.
        """
        if not think:
            return

        lines      = [l for l in think.splitlines() if l.strip()]
        first_line = lines[0][:90] + ('…' if len(lines[0]) > 90 else '') if lines else ''
        char_count = len(think)
        line_count = len(lines)

        # ── Terminal: compact one-liner ───────────────────────────────────────
        if self._stdin_mode:
            print(f'\r\033[90m[思考 {char_count}字/{line_count}行] {first_line}\033[0m')

        # ── ROS2 logger: full content (persisted to ~/.ros/log/) ─────────────
        sep = '─' * 56
        self.get_logger().info(
            f'[THINK] 指令="{user_text}"  ({char_count}字)\n'
            f'{sep}\n'
            f'{think}\n'
            f'{sep}'
        )

    @staticmethod
    def _normalize_action(raw_action: str) -> str:
        """Resolve alias → canonical action name. Returns UPPERCASE canonical or raw."""
        upper = raw_action.strip().upper()
        return _ACTION_ALIASES.get(upper, upper)

    @staticmethod
    def _coerce_params(action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """Apply schema-driven type coercion and range clamping to params dict.

        Unknown params are passed through unchanged.
        """
        schema = _PARAM_SCHEMA.get(action, [])
        out = dict(params)
        for name, type_fn, lo, hi, _ in schema:
            if name in out:
                try:
                    v = type_fn(out[name])
                    if lo is not None:
                        v = max(lo, v)
                    if hi is not None:
                        v = min(hi, v)
                    out[name] = v
                except (ValueError, TypeError):
                    pass  # keep original; validation layer will catch it
        return out

    def _extract_json_robust(self, raw: str) -> Optional[Dict[str, Any]]:
        """Multi-strategy JSON extractor from LLM response.

        Strategy order (stops at first success):
          S1 — Direct parse after fence/think-block stripping
          S2 — Try each balanced {...} candidate (longest first)
          S3 — Regex for bare "action" / "plan" key patterns
        """
        import re

        text = self._strip_fences(raw)

        # S1: direct parse
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        # S2: balanced brace candidates
        for cand in self._balanced_json_candidates(text):
            try:
                obj = json.loads(cand)
                if isinstance(obj, dict):
                    # Accept if it looks like a UAV command or plan
                    if 'action' in obj or 'plan' in obj:
                        return obj
            except json.JSONDecodeError:
                continue

        # S3: regex fallback — look for {"action":"...", ...} or {"plan":[...]}
        patterns = [
            r'\{[^{}]*"action"\s*:\s*"[^"]+?"[^{}]*\}',          # single cmd
            r'\{\s*"plan"\s*:\s*\[[\s\S]*?\]\s*\}',               # plan block
        ]
        for pat in patterns:
            m = re.search(pat, text, re.DOTALL)
            if m:
                try:
                    obj = json.loads(m.group())
                    if isinstance(obj, dict):
                        return obj
                except json.JSONDecodeError:
                    continue

        return None

    # ── Safety validation ─────────────────────────────────────────────────────

    def _extract_plan(self, raw: str) -> Optional[list]:
        """Return list of commands if LLM responded with plan format, else None."""
        obj = self._extract_json_robust(raw)
        if obj is None:
            return None
        if 'plan' in obj and isinstance(obj['plan'], list):
            return obj['plan']
        return None

    def _parse_and_validate_single(self, cmd: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Validate a single command dict (already parsed). Used by _execute_plan."""
        raw_action = str(cmd.get('action', ''))
        action = self._normalize_action(raw_action)
        if action not in ALLOWED_ACTIONS:
            self.get_logger().warn(f'plan 步骤包含未知 action "{raw_action}"，跳过')
            return None
        params: Dict[str, Any] = self._coerce_params(action, dict(cmd.get('params', {})))
        if action in SAFETY_PASS_ACTIONS:
            return {'action': action, 'params': params}
        if 'altitude' in params:
            alt = float(params['altitude'])
            if alt > self._max_alt:
                params['altitude'] = self._max_alt
        for k in ('vx', 'vy', 'speed'):
            if k in params:
                v = float(params[k])
                if abs(v) > self._max_spd:
                    params[k] = math.copysign(self._max_spd, v)
        return {'action': action, 'params': params}

    def _parse_and_validate(self, raw: str) -> Optional[Dict[str, Any]]:
        """Extract JSON from LLM response and apply safety guardrails."""
        obj = self._extract_json_robust(raw)
        if obj is None:
            return None

        raw_action = str(obj.get('action', '')).strip()
        action = self._normalize_action(raw_action)

        if action not in ALLOWED_ACTIONS:
            self.get_logger().warn(
                f'LLM 输出未知 action "{raw_action}"'
                + (f' (别名候选: {action})' if action != raw_action else '')
                + '，回退到 HOVER'
            )
            return {'action': 'HOVER', 'params': {}}

        params: Dict[str, Any] = self._coerce_params(
            action, dict(obj.get('params', {}))
        )

        # Safety-critical actions bypass all parameter checks
        if action in SAFETY_PASS_ACTIONS:
            return {'action': action, 'params': params}

        # Clamp altitude
        if 'altitude' in params:
            alt = float(params['altitude'])
            if alt > self._max_alt:
                self.get_logger().warn(
                    f'高度请求 {alt}m 超限 {self._max_alt}m，已截断'
                )
                params['altitude'] = self._max_alt

        # Clamp horizontal speeds and orbit speed
        for k in ('vx', 'vy', 'speed'):
            if k in params:
                v = float(params[k])
                if abs(v) > self._max_spd:
                    clamped = math.copysign(self._max_spd, v)
                    self.get_logger().warn(f'{k}={v} 超限，已截断至 {clamped}')
                    params[k] = clamped

        # Auto-TAKEOFF: inject TAKEOFF before any flight action when grounded
        if self._flight_phase == 'GROUND' and action != 'TAKEOFF':
            self.get_logger().info(
                f'无人机在地面，自动插入 TAKEOFF 后续执行 {action}'
            )
            self._input_q.put(json.dumps({'action': action, 'params': params}))
            return {'action': 'TAKEOFF', 'params': {'altitude': self._auto_alt}}

        return {'action': action, 'params': params}

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, cmd: Dict[str, Any]) -> None:
        action = cmd.get('action', '?')
        params = cmd.get('params', {})
        # Final safety gate: block non-safety commands when system is offline
        if not self._system_online() and action not in SAFETY_PASS_ACTIONS:
            print(f'\r\033[K  ⚠  安全门拦截：系统离线，{action} 未发送')
            self.get_logger().warn(f'Blocked offline command: {action}')
            if self._stdin_mode:
                print(self._prompt(), end='', flush=True)
            return
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self._cmd_pub.publish(msg)
        print(f'\r[→ UAV] {action:<16s} {params}')
        if self._stdin_mode:
            print(self._prompt(), end='', flush=True)
        self.get_logger().info(f'Published → {action}  {params}')

    # ── Sequential plan execution ─────────────────────────────────────────────

    def _wait_command_done(self, action: str, params: Dict[str, Any]) -> None:
        """Block (in thread pool) until the command completes or times out."""
        if action in ('SET_SPEED', 'EMERGENCY_STOP'):
            return  # immediate; no wait needed

        duration = float(params.get('duration', 0.0))

        # Timed actions: wait for duration + stability buffer
        if action in ('MOVE_VELOCITY', 'MOVE_REL', 'HOVER', 'ORBIT') and duration > 0:
            time.sleep(duration + 1.2)
            return

        if action in ('RTL', 'LAND'):
            # Wait until DISARMED (drone has landed)
            deadline = time.time() + 60.0
            while time.time() < deadline:
                if not self._is_armed:
                    time.sleep(1.0)  # extra settle
                    return
                time.sleep(0.5)
            return

        if action == 'TAKEOFF':
            target_alt = float(params.get('altitude', self._auto_alt))
            deadline = time.time() + 20.0
            time.sleep(1.0)
            while time.time() < deadline:
                pos = (self._latest_telemetry.get('position') or {}) \
                    if self._latest_telemetry else {}
                alt_m = -float(pos.get('z', 0))
                if abs(alt_m - target_alt) < 0.6:
                    time.sleep(0.5)
                    return
                time.sleep(0.3)
            return

        # Event-based (GOTO_NED, YAW_TO, ORBIT without duration):
        # watch TELEMETRY until command transitions back to IDLE/HOVER
        timeout = 30.0
        deadline = time.time() + timeout
        time.sleep(0.5)  # let text_command_bridge register the new command
        while time.time() < deadline:
            tele = self._latest_telemetry
            if tele and tele.get('command', '') in ('IDLE', 'HOVER'):
                return
            time.sleep(0.3)

    def _execute_plan(self, commands: list) -> None:
        """Run in thread pool: publish each command and wait for completion."""
        total = len(commands)
        for i, cmd in enumerate(commands, 1):
            validated = self._parse_and_validate_single(cmd)
            if not validated:
                print(f'\r[计划] 第{i}/{total}步校验失败，跳过: {cmd}')
                continue
            action = validated.get('action', '?')
            print(f'\r[计划 {i}/{total}] {action}  {validated.get("params", {})}')
            if self._stdin_mode:
                print(self._prompt(), end='', flush=True)
            self._publish_cmd(validated)
            if i < total:  # no wait after last step
                self._wait_command_done(action, validated.get('params', {}))


def _read_line(prompt: str, default: str = '') -> str:
    """Read a line from stdin, returning default on EOFError / empty input."""
    try:
        val = input(prompt).strip()
        return val if val else default
    except (EOFError, KeyboardInterrupt):
        return default


def _arrow_select(prompt: str, options: list, default_idx: int = 0) -> int:
    """Interactive arrow-key single-choice menu.

    options: list of str or (label, detail) tuples.
    Returns the selected index, or default_idx if stdin is not a TTY.
    """
    try:
        import termios
        import tty
    except ImportError:
        # Non-Unix fallback: just return default
        return default_idx

    if not sys.stdin.isatty():
        return default_idx

    n = len(options)
    idx = max(0, min(default_idx, n - 1))

    def _label(o: Any) -> str:
        return o[0] if isinstance(o, tuple) else str(o)

    def _detail(o: Any) -> str:
        return o[1] if isinstance(o, tuple) and len(o) > 1 else ''

    pad = max(len(_label(o)) for o in options)

    def _render(cur: int) -> list:
        rows = []
        for i, o in enumerate(options):
            lbl = _label(o)
            det = _detail(o)
            if i == cur:
                row = f'  \033[32m❯\033[0m \033[1m{lbl:<{pad}}\033[0m'
            else:
                row = f'    {lbl:<{pad}}'
            if det:
                row += f'  \033[90m{det}\033[0m'
            rows.append(row)
        rows.append('\033[90m  ↑↓ 移动   Enter 确认   Ctrl+C 取消\033[0m')
        return rows

    print(f'\n  {prompt}\n')
    lines = _render(idx)
    for line in lines:
        print(line)
    total = len(lines)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            key = sys.stdin.read(1)
            if key == '\r':
                break
            if key == '\x03':
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                sys.stdout.write('\r\n')
                sys.stdout.flush()
                raise KeyboardInterrupt
            if key == '\x1b':
                seq = sys.stdin.read(2)
                if seq == '[A':
                    idx = (idx - 1) % n
                elif seq == '[B':
                    idx = (idx + 1) % n
                else:
                    continue
            else:
                continue
            # Redraw: cursor up total lines, overwrite each line
            sys.stdout.write(f'\033[{total}A')
            for line in _render(idx):
                sys.stdout.write(f'\r\033[K{line}\r\n')
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    print(f'\n  \033[90m已选择\033[0m  \033[1;32m{_label(options[idx])}\033[0m\n')
    return idx


def _fetch_ollama_models(host: str, timeout: float = 5.0) -> Optional[list]:
    """Fetch installed models from Ollama /api/tags.

    Returns list of (name, size_str) tuples sorted by size descending,
    or None on any error (caller should fall back to preset list).
    """
    try:
        req = urllib.request.Request(
            f'{host}/api/tags',
            headers={'Accept': 'application/json'},
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read())
        models = data.get('models', [])
        if not models:
            return None
        result = []
        for m in sorted(models, key=lambda x: x.get('size', 0), reverse=True):
            name = m.get('name', '')
            if not name:
                continue
            size_b = m.get('size', 0)
            size_str = (f'{size_b / 1e9:.1f} GB' if size_b >= 1e9
                        else f'{size_b / 1e6:.0f} MB')
            result.append((name, size_str))
        return result or None
    except Exception:
        return None


def _interactive_select() -> Dict[str, Any]:
    """Interactive provider/model selection, runs synchronously before ROS init.

    Returns a dict with the user's configuration choices.
    Returns an empty dict if stdin is not a TTY or provider is already specified
    via command line (non-interactive launch context).
    """
    if not sys.stdin.isatty():
        return {}
    if any('llm_provider' in arg for arg in sys.argv):
        return {}

    env_ollama_host  = os.environ.get('ANTHROPIC_BASE_URL', 'http://localhost:11434')
    env_ollama_model = os.environ.get('ANTHROPIC_MODEL',    'llama3.2')
    env_groq_key     = os.environ.get('GROQ_API_KEY',       '')

    print('\n  UAV LLM 飞控终端\n')

    # ── Provider selection ────────────────────────────────────────────────────
    provider_opts = [
        ('Groq Cloud', '云端推理，低延迟，需要 API Key'),
        ('Ollama',     '本地 / 远程私有化部署，无需 API Key'),
    ]
    p_idx = _arrow_select('选择 LLM 推理后端', provider_opts, default_idx=1)
    provider = 'groq' if p_idx == 0 else 'ollama'

    cfg: Dict[str, Any] = {'provider': provider}

    # ── Groq branch ──────────────────────────────────────────────────────────
    if provider == 'groq':
        custom_opt = ('自定义…', '手动输入模型名')
        groq_opts  = list(_GROQ_PRESETS) + [custom_opt]
        m_idx = _arrow_select('选择 Groq 模型', groq_opts, default_idx=0)
        if m_idx == len(_GROQ_PRESETS):
            cfg['groq_model'] = _read_line('  模型名称: ', _GROQ_PRESETS[0][0])
        else:
            cfg['groq_model'] = _GROQ_PRESETS[m_idx][0]

        if env_groq_key:
            masked = env_groq_key[:8] + '…' + env_groq_key[-4:]
            #print(f'  \033[90m已检测到 GROQ_API_KEY 环境变量  ({masked})\033[0m')
            raw = _read_line('  按 Enter 使用，或输入新 Key: ', '')
            cfg['groq_api_key'] = raw or env_groq_key
        else:
            print('  \033[33m未检测到 GROQ_API_KEY 环境变量\033[0m')
            print('  \033[90m永久配置方法（推荐）:\033[0m')
            print('  \033[90m  echo \'export GROQ_API_KEY="gsk_..."\' >> ~/.bashrc\033[0m')
            print('  \033[90m  source ~/.bashrc\033[0m')
            raw = _read_line('  本次输入 Key（直接回车跳过）: ', '')
            cfg['groq_api_key'] = raw
            if not raw:
                print('  \033[33m⚠  未提供 API Key，发送指令时会报错\033[0m')

    # ── Ollama branch ─────────────────────────────────────────────────────────
    else:
        print(f'  Ollama 主机地址  \033[90m当前: {env_ollama_host}\033[0m')
        raw = _read_line('  按 Enter 保留，或输入新地址: ', '')
        cfg['ollama_host'] = raw or env_ollama_host
        print()

        # Fetch installed models; fall back to presets on failure
        print(f'  \033[90m正在从 {cfg["ollama_host"]} 获取已安装模型…\033[0m', end='', flush=True)
        fetched = _fetch_ollama_models(cfg['ollama_host'])
        if fetched:
            print(f'\r\033[K  \033[90m获取到 {len(fetched)} 个已安装模型\033[0m')
            ollama_opts: list = fetched
        else:
            print(f'\r\033[K  \033[90m获取失败，使用预设模型列表\033[0m')
            ollama_opts = list(_OLLAMA_PRESETS)

        # Mark the env-var model and compute its default index
        marked_opts = []
        default_m = 0
        for i, (name, detail) in enumerate(ollama_opts):
            if name == env_ollama_model:
                marked_opts.append((name, f'{detail}  \033[33m◀ 当前\033[0m'))
                default_m = i
            else:
                marked_opts.append((name, detail))
        marked_opts.append(('自定义…', '手动输入模型名'))

        m_idx = _arrow_select('选择 Ollama 模型', marked_opts, default_idx=default_m)
        if m_idx == len(ollama_opts):
            cfg['ollama_model'] = _read_line('  模型名称: ', env_ollama_model)
        else:
            cfg['ollama_model'] = ollama_opts[m_idx][0]

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f'  \033[90m─────────────────────────────\033[0m')
    print(f'  后端   {cfg["provider"].upper()}')
    if provider == 'groq':
        print(f'  模型   {cfg.get("groq_model", "?")}')
        print(f'  Key    {"✓ 已设置" if cfg.get("groq_api_key") else "✗ 未设置 ⚠"}')
    else:
        print(f'  主机   {cfg.get("ollama_host", "?")}')
        print(f'  模型   {cfg.get("ollama_model", "?")}')
    print(f'  \033[90m─────────────────────────────\033[0m\n')

    return cfg


def main(args=None) -> None:
    # Interactive provider/model selection runs BEFORE ROS init so stdin is
    # available for synchronous blocking I/O without interfering with the spin loop.
    config = _interactive_select()

    rclpy.init(args=args)
    node = LLMClient(config_override=config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
