"""Intent schema helpers for natural-language UAV commands.

The LLM is allowed to fill only this semantic schema. Flight-control actions
and low-level parameters are produced later by deterministic code.
"""

from __future__ import annotations

from typing import Any, Dict, Optional


ALLOWED_INTENTS = frozenset({
    'takeoff',
    'land',
    'hover',
    'return_home',
    'emergency_stop',
    'relative_move',
    'relative_turn',
    'set_speed',
    'goto_position',
    'visual_search',
    'visual_goto',
    'unknown',
})

ALLOWED_DIRECTIONS = frozenset({'forward', 'back', 'left', 'right', 'up', 'down'})
ALLOWED_TURN_DIRECTIONS = frozenset({'left', 'right'})
ALLOWED_MAGNITUDES = frozenset({'small', 'medium', 'large'})


DEFAULT_INTENT: Dict[str, Any] = {
    'intent': 'unknown',
    'direction': None,
    'distance_m': None,
    'turn_direction': None,
    'angle_deg': None,
    'altitude_m': None,
    'speed_mps': None,
    'magnitude': None,
    'query': None,
    'should_move': False,
    'needs_clarification': True,
    'confidence': 0.0,
}


def _none_if_nullish(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped or stripped.lower() in {'null', 'none', 'nil', 'unknown', 'n/a'}:
            return None
        return stripped
    return value


def _coerce_float(value: Any) -> Optional[float]:
    value = _none_if_nullish(value)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _coerce_bool(value: Any, default: bool = False) -> bool:
    value = _none_if_nullish(value)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.lower()
        if lowered in {'true', 'yes', 'y', '1'}:
            return True
        if lowered in {'false', 'no', 'n', '0'}:
            return False
    return default


def _coerce_enum(value: Any, allowed: frozenset[str]) -> Optional[str]:
    value = _none_if_nullish(value)
    if not isinstance(value, str):
        return None
    normalized = value.strip().lower()
    return normalized if normalized in allowed else None


def normalize_intent(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Return a normalized intent dict with all schema keys present."""
    out = dict(DEFAULT_INTENT)
    if not isinstance(raw, dict):
        return out

    intent = _coerce_enum(raw.get('intent'), ALLOWED_INTENTS)
    out['intent'] = intent or 'unknown'
    out['direction'] = _coerce_enum(raw.get('direction'), ALLOWED_DIRECTIONS)
    out['turn_direction'] = _coerce_enum(raw.get('turn_direction'), ALLOWED_TURN_DIRECTIONS)
    out['magnitude'] = _coerce_enum(raw.get('magnitude'), ALLOWED_MAGNITUDES)

    for key in ('distance_m', 'angle_deg', 'altitude_m', 'speed_mps'):
        value = _coerce_float(raw.get(key))
        out[key] = value if value is None else abs(value)

    query = _none_if_nullish(raw.get('query'))
    out['query'] = query if isinstance(query, str) else None
    out['should_move'] = _coerce_bool(raw.get('should_move'), False)
    out['needs_clarification'] = _coerce_bool(raw.get('needs_clarification'), False)

    confidence = _coerce_float(raw.get('confidence'))
    if confidence is None:
        confidence = 0.0
    out['confidence'] = max(0.0, min(1.0, confidence))

    if out['intent'] == 'unknown':
        out['needs_clarification'] = True
    if out['needs_clarification']:
        out['should_move'] = False

    return out
