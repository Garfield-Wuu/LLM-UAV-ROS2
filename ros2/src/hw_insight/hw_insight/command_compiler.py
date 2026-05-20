"""Compile semantic UAV intents into the existing action JSON protocol."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple


MAGNITUDE_DISTANCE_M = {
    'small': 1.0,
    'medium': 3.0,
    'large': 5.0,
}

MAGNITUDE_ANGLE_DEG = {
    'small': 30.0,
    'medium': 60.0,
    'large': 90.0,
}


def _distance(intent: Dict[str, Any], default: float = 1.0) -> float:
    value = intent.get('distance_m')
    if isinstance(value, (int, float)) and value > 0:
        return float(value)
    magnitude = intent.get('magnitude')
    return MAGNITUDE_DISTANCE_M.get(magnitude, default)


def _angle(intent: Dict[str, Any], default: float = 30.0) -> float:
    value = intent.get('angle_deg')
    if isinstance(value, (int, float)) and value > 0:
        return float(value)
    magnitude = intent.get('magnitude')
    return MAGNITUDE_ANGLE_DEG.get(magnitude, default)


def _compile_relative_move(intent: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    direction = intent.get('direction')
    distance = _distance(intent)
    dx = dy = dz = 0.0
    if direction == 'forward':
        dx = distance
    elif direction == 'back':
        dx = -distance
    elif direction == 'right':
        dy = distance
    elif direction == 'left':
        dy = -distance
    elif direction == 'up':
        dz = -distance
    elif direction == 'down':
        dz = distance
    else:
        return None

    speed = 1.0 if direction in ('up', 'down') else 2.0
    return {
        'action': 'MOVE_REL',
        'params': {
            'dx': round(dx, 2),
            'dy': round(dy, 2),
            'dz': round(dz, 2),
            'duration': round(max(0.5, distance / speed), 2),
        },
    }


def _compile_relative_turn(intent: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    direction = intent.get('turn_direction')
    if direction not in ('left', 'right'):
        return None
    angle_deg = max(1.0, min(360.0, _angle(intent)))
    sign = 1.0 if direction == 'right' else -1.0
    yaw_rate = 0.6 * sign
    duration = max(0.5, math.radians(angle_deg) / abs(yaw_rate))
    return {
        'action': 'MOVE_VELOCITY',
        'params': {
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'yaw_rate': round(yaw_rate, 2),
            'duration': round(duration, 2),
        },
    }


def compile_intent(intent: Dict[str, Any], auto_takeoff_altitude: float = 6.0) -> Tuple[Dict[str, Any], str]:
    """Compile an intent into an action command and return (cmd, reason)."""
    name = intent.get('intent')

    if name == 'takeoff':
        altitude = intent.get('altitude_m')
        if not isinstance(altitude, (int, float)) or altitude <= 0:
            altitude = auto_takeoff_altitude
        return {'action': 'TAKEOFF', 'params': {'altitude': round(float(altitude), 2)}}, 'takeoff'

    if name == 'land':
        return {'action': 'LAND', 'params': {}}, 'land'
    if name == 'hover':
        return {'action': 'HOVER', 'params': {}}, 'hover'
    if name == 'return_home':
        return {'action': 'RTL', 'params': {}}, 'return_home'
    if name == 'emergency_stop':
        return {'action': 'EMERGENCY_STOP', 'params': {}}, 'emergency_stop'

    if name == 'set_speed':
        speed = intent.get('speed_mps')
        if not isinstance(speed, (int, float)) or speed <= 0:
            return {'action': 'HOVER', 'params': {}}, 'set_speed_missing_speed'
        return {'action': 'SET_SPEED', 'params': {'speed': round(float(speed), 2)}}, 'set_speed'

    if name == 'relative_move':
        cmd = _compile_relative_move(intent)
        return (cmd, 'relative_move') if cmd else ({'action': 'HOVER', 'params': {}}, 'relative_move_missing_direction')

    if name == 'relative_turn':
        cmd = _compile_relative_turn(intent)
        return (cmd, 'relative_turn') if cmd else ({'action': 'HOVER', 'params': {}}, 'relative_turn_missing_direction')

    if name == 'visual_goto':
        query = intent.get('query')
        if isinstance(query, str) and query.strip() and intent.get('should_move') is True:
            return {'action': 'FIND_AND_GOTO', 'params': {'query': query.strip()}}, 'visual_goto'
        return {'action': 'HOVER', 'params': {}}, 'visual_goto_missing_query'

    if name == 'visual_search':
        return {'action': 'HOVER', 'params': {}}, 'visual_search_no_motion_action'

    return {'action': 'HOVER', 'params': {}}, 'unknown_or_unsupported_intent'
