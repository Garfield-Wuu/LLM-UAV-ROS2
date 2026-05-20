"""Consistency checks between user text and LLM intent slots."""

from __future__ import annotations

from typing import Any, Dict, Tuple

from hw_insight.visual_query import simplify_visual_query


def _compact(text: str) -> str:
    return ''.join(text.lower().split())


def _has_any(text: str, terms: tuple[str, ...]) -> bool:
    return any(term in text for term in terms)


def guard_intent(user_text: str, intent: Dict[str, Any]) -> Tuple[Dict[str, Any], list[str]]:
    """Correct obvious contradictions and mark unsafe ambiguity."""
    text = _compact(user_text)
    out = dict(intent)
    reasons: list[str] = []

    if out.get('intent') == 'unknown' or out.get('needs_clarification'):
        out['should_move'] = False
        return out, reasons

    if _has_any(text, ('太高', '高了', '有点高')):
        if out.get('intent') != 'relative_move' or out.get('direction') != 'down':
            reasons.append('altitude complaint corrected to relative_move down')
        out.update({
            'intent': 'relative_move',
            'direction': 'down',
            'distance_m': None,
            'turn_direction': None,
            'angle_deg': None,
            'speed_mps': None,
            'magnitude': out.get('magnitude') or 'small',
            'query': None,
            'should_move': True,
            'needs_clarification': False,
        })

    if _has_any(text, ('太低', '低了', '有点低')):
        if out.get('intent') != 'relative_move' or out.get('direction') != 'up':
            reasons.append('altitude complaint corrected to relative_move up')
        out.update({
            'intent': 'relative_move',
            'direction': 'up',
            'distance_m': None,
            'turn_direction': None,
            'angle_deg': None,
            'speed_mps': None,
            'magnitude': out.get('magnitude') or 'small',
            'query': None,
            'should_move': True,
            'needs_clarification': False,
        })

    if _has_any(text, ('去那边', '到那边', '过去看看', '那边看看')):
        reasons.append('vague deictic target requires clarification')
        out.update({
            'intent': 'unknown',
            'direction': None,
            'distance_m': None,
            'turn_direction': None,
            'angle_deg': None,
            'magnitude': None,
            'query': None,
            'should_move': False,
            'needs_clarification': True,
        })
        return out, reasons

    visual_question_terms = ('有人吗', '有没有', '看一下', '看看', '看到吗', '是什么', '前面有什么')
    visual_motion_terms = ('飞到', '靠近', '前往', '跟随', '过去', '到')
    if _has_any(text, visual_question_terms) and not _has_any(text, visual_motion_terms):
        if out.get('intent') == 'visual_goto':
            reasons.append('visual question downgraded from visual_goto to visual_search')
        out['intent'] = 'visual_search'
        out['should_move'] = False

    if _has_any(text, ('左', '往左', '向左', '左边', '左侧')) and out.get('intent') == 'relative_move':
        if out.get('direction') not in (None, 'left'):
            reasons.append(f'direction corrected to left from {out.get("direction")}')
        out['direction'] = 'left'
    if _has_any(text, ('右', '往右', '向右', '右边', '右侧')) and out.get('intent') == 'relative_move':
        if out.get('direction') not in (None, 'right'):
            reasons.append(f'direction corrected to right from {out.get("direction")}')
        out['direction'] = 'right'
    if _has_any(text, ('降低', '下降', '向下', '往下', '太高', '高了')) and out.get('intent') == 'relative_move':
        if out.get('direction') not in (None, 'down'):
            reasons.append(f'direction corrected to down from {out.get("direction")}')
        out['direction'] = 'down'
    if _has_any(text, ('升高', '上升', '向上', '往上', '太低', '低了')) and out.get('intent') == 'relative_move':
        if out.get('direction') not in (None, 'up'):
            reasons.append(f'direction corrected to up from {out.get("direction")}')
        out['direction'] = 'up'

    if _has_any(text, ('左转', '逆时针', '左旋')) and out.get('intent') == 'relative_turn':
        out['turn_direction'] = 'left'
    if _has_any(text, ('右转', '顺时针', '右旋')) and out.get('intent') == 'relative_turn':
        out['turn_direction'] = 'right'

    if out.get('intent') in ('visual_search', 'visual_goto'):
        query = out.get('query')
        if isinstance(query, str):
            lowered_query = query.lower()
            if 'yellow' in lowered_query and '黄' not in text and 'yellow' not in text:
                lowered_query = lowered_query.replace('yellow', '').strip()
                reasons.append('removed hallucinated yellow attribute from query')
            if 'clothes' in lowered_query and '衣' not in text and 'clothes' not in text:
                lowered_query = lowered_query.replace('clothes', '').strip()
                reasons.append('removed hallucinated clothes attribute from query')
            if lowered_query.endswith('wearing'):
                lowered_query = lowered_query[:-len('wearing')].strip()
                reasons.append('removed dangling wearing from query')
            lowered_query = ' '.join(lowered_query.split()) or None
            if lowered_query:
                simplified, simp_reasons = simplify_visual_query(lowered_query, user_text)
                reasons.extend(simp_reasons)
                out['query'] = simplified or None
            else:
                out['query'] = None

    if out.get('needs_clarification') or out.get('intent') == 'unknown':
        out['should_move'] = False

    return out, reasons
