"""Final safety validation for compiled UAV action commands."""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple


SAFETY_ACTIONS = frozenset({'LAND', 'HOVER', 'RTL', 'EMERGENCY_STOP'})


def _float_param(params: Dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        return float(params.get(key, default))
    except (TypeError, ValueError):
        return default


def validate_command(
    cmd: Dict[str, Any],
    *,
    max_altitude_m: float,
    max_speed_ms: float,
    max_vertical_speed_ms: float = 2.0,
    max_yaw_rate: float = 1.0,
    max_duration_sec: float = 20.0,
    max_relative_distance_m: float = 20.0,
) -> Tuple[Dict[str, Any], list[str]]:
    """Clamp or reject unsafe compiled commands."""
    action = str(cmd.get('action', '')).upper()
    params = dict(cmd.get('params') or {})
    warnings: list[str] = []

    if action in SAFETY_ACTIONS:
        return {'action': action, 'params': params}, warnings

    if action == 'TAKEOFF':
        altitude = abs(_float_param(params, 'altitude', 0.0))
        if altitude <= 0.0:
            return {'action': 'HOVER', 'params': {}}, ['takeoff missing altitude']
        if altitude > max_altitude_m:
            warnings.append(f'altitude clamped {altitude}->{max_altitude_m}')
            altitude = max_altitude_m
        return {'action': action, 'params': {'altitude': round(altitude, 2)}}, warnings

    if action == 'SET_SPEED':
        speed = abs(_float_param(params, 'speed', 0.0))
        if speed <= 0.0:
            return {'action': 'HOVER', 'params': {}}, ['set_speed missing speed']
        if speed > max_speed_ms:
            warnings.append(f'speed clamped {speed}->{max_speed_ms}')
            speed = max_speed_ms
        return {'action': action, 'params': {'speed': round(speed, 2)}}, warnings

    if action == 'MOVE_REL':
        dx = _float_param(params, 'dx')
        dy = _float_param(params, 'dy')
        dz = _float_param(params, 'dz')
        duration = max(0.5, _float_param(params, 'duration', 1.0))
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= 0.0:
            return {'action': 'HOVER', 'params': {}}, ['zero relative move']
        if dist > max_relative_distance_m:
            scale = max_relative_distance_m / dist
            dx, dy, dz = dx * scale, dy * scale, dz * scale
            warnings.append(f'relative distance clamped {dist:.2f}->{max_relative_distance_m}')
        if duration > max_duration_sec:
            warnings.append(f'duration clamped {duration}->{max_duration_sec}')
            duration = max_duration_sec
        return {
            'action': action,
            'params': {
                'dx': round(dx, 2),
                'dy': round(dy, 2),
                'dz': round(dz, 2),
                'duration': round(duration, 2),
            },
        }, warnings

    if action == 'MOVE_VELOCITY':
        vx = max(-max_speed_ms, min(max_speed_ms, _float_param(params, 'vx')))
        vy = max(-max_speed_ms, min(max_speed_ms, _float_param(params, 'vy')))
        vz = max(-max_vertical_speed_ms, min(max_vertical_speed_ms, _float_param(params, 'vz')))
        yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, _float_param(params, 'yaw_rate')))
        duration = max(0.1, min(max_duration_sec, _float_param(params, 'duration', 1.0)))
        return {
            'action': action,
            'params': {
                'vx': round(vx, 2),
                'vy': round(vy, 2),
                'vz': round(vz, 2),
                'yaw_rate': round(yaw_rate, 2),
                'duration': round(duration, 2),
            },
        }, warnings

    if action == 'FIND_AND_GOTO':
        from hw_insight.visual_query import simplify_visual_query

        query = str(params.get('query', '')).strip()
        if not query:
            return {'action': 'HOVER', 'params': {}}, ['empty visual query']
        simplified, simp_reasons = simplify_visual_query(query)
        warnings.extend(simp_reasons)
        return {'action': action, 'params': {'query': simplified}}, warnings

    return {'action': 'HOVER', 'params': {}}, [f'unsupported action {action}']
