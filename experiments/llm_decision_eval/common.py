#!/usr/bin/env python3
"""Shared utilities for thesis Section 5.4 experiments."""

from __future__ import annotations

import json
import math
import os
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


OLLAMA_HOST = os.environ.get("ANTHROPIC_BASE_URL", "http://39.108.60.130:6300").rstrip("/")
MODEL_MAP = {
    "Qwen2.5-7B-Instruct": "qwen2.5:7b-instruct",
    "Llama 3.2-3B": "llama3.2:3b",
    "Phi-3.5-mini-Instruct": "phi3.5:latest",
}

ALLOWED_ACTIONS = frozenset(
    {
        "TAKEOFF",
        "LAND",
        "HOVER",
        "MOVE_VELOCITY",
        "MOVE_REL",
        "GOTO_NED",
        "ORBIT",
        "YAW_TO",
        "RTL",
        "EMERGENCY_STOP",
        "SET_SPEED",
    }
)

ACTION_ALIASES = {
    "TAKE_OFF": "TAKEOFF",
    "LIFTOFF": "TAKEOFF",
    "LAUNCH": "TAKEOFF",
    "ASCEND": "TAKEOFF",
    "LANDING": "LAND",
    "DESCEND": "LAND",
    "SET_DOWN": "LAND",
    "STOP": "HOVER",
    "WAIT": "HOVER",
    "HOLD": "HOVER",
    "STAY": "HOVER",
    "PAUSE": "HOVER",
    "MOVE": "MOVE_REL",
    "MOVE_FORWARD": "MOVE_REL",
    "MOVE_RELATIVE": "MOVE_REL",
    "FLY": "MOVE_REL",
    "FLY_RELATIVE": "MOVE_REL",
    "GO": "MOVE_REL",
    "TRANSLATE": "MOVE_REL",
    "GOTO": "GOTO_NED",
    "GO_TO": "GOTO_NED",
    "FLY_TO": "GOTO_NED",
    "NAVIGATE": "GOTO_NED",
    "NAVIGATE_TO": "GOTO_NED",
    "WAYPOINT": "GOTO_NED",
    "VELOCITY": "MOVE_VELOCITY",
    "SET_VELOCITY": "MOVE_VELOCITY",
    "MOVE_VEL": "MOVE_VELOCITY",
    "ROTATE": "YAW_TO",
    "TURN": "YAW_TO",
    "YAW": "YAW_TO",
    "HEADING": "YAW_TO",
    "RETURN": "RTL",
    "RETURN_HOME": "RTL",
    "GO_HOME": "RTL",
    "HOME": "RTL",
    "EMERGENCY": "EMERGENCY_STOP",
    "ABORT": "EMERGENCY_STOP",
    "KILL": "EMERGENCY_STOP",
    "E_STOP": "EMERGENCY_STOP",
    "ESTOP": "EMERGENCY_STOP",
}

PARAM_SCHEMA = {
    "TAKEOFF": [("altitude", float, True)],
    "LAND": [],
    "HOVER": [("duration", float, False)],
    "MOVE_VELOCITY": [
        ("vx", float, False),
        ("vy", float, False),
        ("vz", float, False),
        ("yaw_rate", float, False),
        ("duration", float, True),
    ],
    "MOVE_REL": [
        ("dx", float, False),
        ("dy", float, False),
        ("dz", float, False),
        ("duration", float, True),
    ],
    "GOTO_NED": [("x", float, True), ("y", float, True), ("altitude", float, True)],
    "ORBIT": [
        ("cx", float, True),
        ("cy", float, True),
        ("radius", float, True),
        ("speed", float, True),
        ("duration", float, True),
    ],
    "YAW_TO": [("angle", float, True)],
    "RTL": [],
    "EMERGENCY_STOP": [],
    "SET_SPEED": [("speed", float, True)],
}

SYSTEM_PROMPT_TEMPLATE = """\
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
  前进/向前  →  vx/dx    正(+)
  后退/向后  →  vx/dx    负(-)
  右移/向右  →  vy/dy    正(+)
  左移/向左  →  vy/dy    负(-)
  上升/向上  →  vz/dz    负(-)
  下降/向下  →  vz/dz    正(+)
altitude 参数（TAKEOFF/GOTO_NED）始终正值，向上为正，无需转换。
GOTO_NED 的 x/y 是 NED 世界坐标（x=北, y=东），与机头方向无关。

━━━ 安全约束 ━━━
  最大飞行高度：120.0 米  |  最大速度：15.0 m/s

━━━ 当前无人机状态 ━━━
  武装: ARMED   飞行阶段: HOVERING
  位置(NED): x=0.0m  y=0.0m  高度=1.0m
  速度: vx=0.0  vy=0.0  vz=0.0 m/s
  航向: 0°   当前指令: HOVER

━━━ 输出规则（严格执行）━━━
  1. 单步指令 -> 输出一行纯 JSON：{{"action":"...","params":{{...}}}}
  2. 多步指令 -> 输出 plan 格式：
     {{"plan":[{{"action":"...","params":{{...}}}},{{"action":"...","params":{{...}}}}]}}
  3. 指令不明确或不安全 -> 返回 {{"action":"HOVER","params":{{}}}}
  4. 不要输出任何解释、注释、前后缀、Markdown 代码块
"""


@dataclass
class EvalResult:
    parser_name: str
    ok: bool
    parsed: Optional[Dict[str, Any]]
    error: str = ""


def build_messages(user_text: str) -> Tuple[str, str]:
    return SYSTEM_PROMPT_TEMPLATE, user_text


def call_ollama(
    model_name: str,
    user_text: str,
    timeout_sec: float = 120.0,
    retries: int = 3,
    retry_sleep_sec: float = 2.0,
    json_mode: bool = False,
) -> Dict[str, Any]:
    system, user = build_messages(user_text)
    payload_dict = {
        "model": MODEL_MAP[model_name],
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": user},
        ],
        "stream": False,
        "options": {"temperature": 0.1},
    }
    if json_mode:
        payload_dict["format"] = "json"
    payload = json.dumps(payload_dict).encode()
    req = urllib.request.Request(
        f"{OLLAMA_HOST}/api/chat",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    last_error = ""
    for attempt in range(1, retries + 1):
        t0 = time.time()
        try:
            with urllib.request.urlopen(req, timeout=int(timeout_sec)) as resp:
                body = json.loads(resp.read())
            return {
                "raw_output": body["message"]["content"].strip(),
                "latency_sec": round(time.time() - t0, 3),
                "attempts": attempt,
            }
        except (urllib.error.URLError, ConnectionError, TimeoutError, OSError) as exc:
            last_error = str(exc)
            if attempt == retries:
                break
            time.sleep(retry_sleep_sec)
    raise RuntimeError(f"Ollama request failed after {retries} attempts: {last_error}")


def strip_fences(text: str) -> str:
    text = re.sub(r"```(?:json)?\s*", "", text)
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
    return text.strip()


def balanced_json_candidates(text: str) -> List[str]:
    out: List[str] = []
    depth = 0
    start = -1
    for idx, ch in enumerate(text):
        if ch == "{":
            if depth == 0:
                start = idx
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start != -1:
                out.append(text[start : idx + 1])
                start = -1
    out.sort(key=len, reverse=True)
    return out


def balanced_list_candidates(text: str) -> List[str]:
    out: List[str] = []
    depth = 0
    start = -1
    for idx, ch in enumerate(text):
        if ch == "[":
            if depth == 0:
                start = idx
            depth += 1
        elif ch == "]":
            depth -= 1
            if depth == 0 and start != -1:
                out.append(text[start : idx + 1])
                start = -1
    out.sort(key=len, reverse=True)
    return out


def _looks_like_plan_list(obj: Any) -> bool:
    return isinstance(obj, list) and all(isinstance(item, dict) and "action" in item for item in obj)


def _as_command_dict(obj: Any) -> Optional[Dict[str, Any]]:
    if isinstance(obj, dict):
        return obj
    if _looks_like_plan_list(obj):
        return {"plan": obj}
    return None


def _repair_common_json_noise(text: str) -> str:
    repaired = text.strip()
    repaired = re.sub(r"^\{\{", "{", repaired)
    repaired = re.sub(r"\}\}$", "}", repaired)
    repaired = repaired.replace('"params":{{', '"params":{')
    repaired = repaired.replace('{{"action"', '{"action"')
    return repaired


def direct_parse(raw: str) -> EvalResult:
    try:
        parsed = json.loads(raw.strip())
    except Exception as exc:
        return EvalResult("baseline", False, None, str(exc))
    if not isinstance(parsed, dict):
        return EvalResult("baseline", False, None, "not_a_dict")
    return EvalResult("baseline", True, parsed)


def robust_parse(raw: str) -> EvalResult:
    text = strip_fences(raw)
    try:
        parsed = json.loads(text)
        parsed_dict = _as_command_dict(parsed)
        if parsed_dict is not None:
            return EvalResult("proposed", True, parsed_dict)
    except Exception:
        pass

    repaired = _repair_common_json_noise(text)
    if repaired != text:
        try:
            parsed = json.loads(repaired)
            parsed_dict = _as_command_dict(parsed)
            if parsed_dict is not None:
                return EvalResult("proposed", True, parsed_dict)
        except Exception:
            pass

    for cand in balanced_json_candidates(text):
        try:
            parsed = json.loads(cand)
        except Exception:
            continue
        parsed_dict = _as_command_dict(parsed)
        if parsed_dict is not None and ("action" in parsed_dict or "plan" in parsed_dict):
            return EvalResult("proposed", True, parsed_dict)

    for cand in balanced_json_candidates(repaired):
        try:
            parsed = json.loads(cand)
        except Exception:
            continue
        parsed_dict = _as_command_dict(parsed)
        if parsed_dict is not None and ("action" in parsed_dict or "plan" in parsed_dict):
            return EvalResult("proposed", True, parsed_dict)

    for cand in balanced_list_candidates(text):
        try:
            parsed = json.loads(cand)
        except Exception:
            continue
        parsed_dict = _as_command_dict(parsed)
        if parsed_dict is not None:
            return EvalResult("proposed", True, parsed_dict)

    for pattern in (
        r'\{[^{}]*"action"\s*:\s*"[^"]+?"[^{}]*\}',
        r'\{\s*"plan"\s*:\s*\[[\s\S]*?\]\s*\}',
        r'\[[\s\S]*?\]',
    ):
        match = re.search(pattern, text, re.DOTALL)
        if not match:
            continue
        try:
            parsed = json.loads(match.group())
        except Exception:
            continue
        parsed_dict = _as_command_dict(parsed)
        if parsed_dict is not None:
            return EvalResult("proposed", True, parsed_dict)

    return EvalResult("proposed", False, None, "no_valid_json_found")


def normalize_action(raw_action: str) -> str:
    upper = str(raw_action).strip().upper()
    return ACTION_ALIASES.get(upper, upper)


def coerce_params(action: str, params: Dict[str, Any]) -> Dict[str, Any]:
    schema = PARAM_SCHEMA.get(action, [])
    out = dict(params)
    for name, caster, _required in schema:
        if name in out:
            try:
                out[name] = caster(out[name])
            except Exception:
                out.pop(name, None)
    return out


def _expected_params(action: str) -> Dict[str, Tuple[type, bool]]:
    return {name: (caster, required) for name, caster, required in PARAM_SCHEMA.get(action, [])}


def _is_numeric_json_value(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _strict_params(action: str, params: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    expected = _expected_params(action)
    if set(params.keys()) - set(expected.keys()):
        return None

    out: Dict[str, Any] = {}
    for name, (caster, required) in expected.items():
        if required and name not in params:
            return None
        if name not in params:
            continue
        value = params[name]
        if caster is float:
            if not _is_numeric_json_value(value):
                return None
            out[name] = float(value)
        else:
            if not isinstance(value, caster):
                return None
            out[name] = value
    return out


def _tolerant_params(action: str, params: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    expected = _expected_params(action)
    out = coerce_params(action, params)
    for name, (_caster, required) in expected.items():
        if required and name not in out:
            return None
    if set(out.keys()) - set(expected.keys()):
        return None
    return out


def validate_baseline_command(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if "plan" in obj and isinstance(obj["plan"], list):
        validated_plan = []
        for item in obj["plan"]:
            if not isinstance(item, dict):
                return None
            action = str(item.get("action", "")).strip()
            if action not in ALLOWED_ACTIONS:
                return None
            raw_params = item.get("params", {})
            if not isinstance(raw_params, dict):
                return None
            params = _strict_params(action, raw_params)
            if params is None:
                return None
            validated_plan.append({"action": action, "params": params})
        return {"plan": validated_plan}

    action = str(obj.get("action", "")).strip()
    if action not in ALLOWED_ACTIONS:
        return None
    raw_params = obj.get("params", {})
    if not isinstance(raw_params, dict):
        return None
    params = _strict_params(action, raw_params)
    if params is None:
        return None
    return {"action": action, "params": params}


def validate_proposed_command(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if "plan" in obj and isinstance(obj["plan"], list):
        validated_plan = []
        for item in obj["plan"]:
            if not isinstance(item, dict):
                return None
            raw_action = item.get("action", "")
            action = normalize_action(raw_action)
            if action not in ALLOWED_ACTIONS:
                return None
            raw_params = item.get("params", {})
            if not isinstance(raw_params, dict):
                return None
            params = _tolerant_params(action, raw_params)
            if params is None:
                return None
            validated_plan.append({"action": action, "params": params})
        return {"plan": validated_plan}

    raw_action = obj.get("action", "")
    action = normalize_action(raw_action)
    if action not in ALLOWED_ACTIONS:
        return None
    raw_params = obj.get("params", {})
    if not isinstance(raw_params, dict):
        return None
    params = _tolerant_params(action, raw_params)
    if params is None:
        return None
    return {"action": action, "params": params}


def validate_parsed_command(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return validate_proposed_command(obj)


def compare_numeric(actual: Any, expected: Any, tolerance: float = 0.25) -> bool:
    try:
        return math.isclose(float(actual), float(expected), abs_tol=tolerance)
    except Exception:
        return False


def compare_params(actual: Dict[str, Any], expected: Dict[str, Any], tolerance: float = 0.25) -> bool:
    for key, value in expected.items():
        if key not in actual:
            return False
        if isinstance(value, (int, float)):
            if not compare_numeric(actual[key], value, tolerance):
                return False
        else:
            if str(actual[key]).strip().lower() != str(value).strip().lower():
                return False
    return True


def tpa_match(task: Dict[str, Any], parsed: Dict[str, Any]) -> bool:
    if "plan" in task:
        if "plan" not in parsed or len(parsed["plan"]) != len(task["plan"]):
            return False
        for gold_step, pred_step in zip(task["plan"], parsed["plan"]):
            if normalize_action(pred_step.get("action", "")) != gold_step["action"]:
                return False
            if not compare_params(pred_step.get("params", {}), gold_step.get("params", {}), gold_step.get("tolerance", 0.25)):
                return False
        return True

    if normalize_action(parsed.get("action", "")) != task["action"]:
        return False
    return compare_params(parsed.get("params", {}), task.get("params", {}), task.get("tolerance", 0.25))


def flatten_command(parsed: Dict[str, Any]) -> List[Dict[str, Any]]:
    if "plan" in parsed:
        return list(parsed["plan"])
    return [{"action": parsed["action"], "params": parsed.get("params", {})}]
