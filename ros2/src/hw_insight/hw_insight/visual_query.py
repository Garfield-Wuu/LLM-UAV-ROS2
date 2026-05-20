"""Normalize visual search queries for YOLO-World open-vocabulary detection."""

from __future__ import annotations

import re
from typing import List, Tuple

# YOLO-World 对短标签更稳；逗号分隔会拆成多个独立 text prompt。
_BASE_OBJECTS = (
    'person', 'people', 'man', 'woman', 'child', 'human',
    'car', 'truck', 'bus', 'van', 'bicycle', 'bike', 'motorcycle',
    'dog', 'cat', 'bird', 'drone', 'cone', 'barrel', 'box',
)

_OBJECT_ALIASES = {
    'people': 'person',
    'man': 'person',
    'woman': 'person',
    'child': 'person',
    'human': 'person',
    'bike': 'bicycle',
}

_COLORS_EN = (
    'red', 'blue', 'green', 'yellow', 'white', 'black', 'orange', 'purple', 'gray', 'grey',
)

_COLOR_ZH = (
    ('红色', 'red'), ('红衣服', 'red'), ('红衣', 'red'), ('红的', 'red'), ('红', 'red'),
    ('黄色', 'yellow'), ('黄衣服', 'yellow'), ('黄衣', 'yellow'), ('黄', 'yellow'),
    ('蓝色', 'blue'), ('蓝衣服', 'blue'), ('蓝衣', 'blue'), ('蓝', 'blue'),
    ('绿色', 'green'), ('绿', 'green'),
    ('白色', 'white'), ('白', 'white'),
    ('黑色', 'black'), ('黑', 'black'),
    ('橙色', 'orange'), ('橙', 'orange'),
)

# 冗长短语 → 丢弃（只保留 base + 至多一个 color）
_NOISE_WORDS = re.compile(
    r'\b(wearing|wears|wear|clothes|clothing|shirt|jacket|coat|pants|dress|'
    r'uniform|outfit|attire|standing|sitting|walking|near|the|a|an|with|in|on|at)\b',
    re.IGNORECASE,
)


def _tokens(query: str) -> List[str]:
    cleaned = _NOISE_WORDS.sub(' ', query.lower())
    return [t for t in re.split(r'[\s,]+', cleaned) if t]


def _base_from_tokens(tokens: List[str]) -> str | None:
    for tok in tokens:
        if tok in _BASE_OBJECTS:
            return _OBJECT_ALIASES.get(tok, tok)
    return None


def _color_from_tokens(tokens: List[str]) -> str | None:
    for tok in tokens:
        if tok in _COLORS_EN:
            return tok
    return None


def _color_from_user_text(user_text: str) -> str | None:
    for zh, en in _COLOR_ZH:
        if zh in user_text:
            return en
    lowered = user_text.lower()
    for color in _COLORS_EN:
        if color in lowered:
            return color
    return None


def _is_already_simple(query: str) -> bool:
    parts = [p.strip() for p in query.split(',') if p.strip()]
    if not parts or len(parts) > 2:
        return False
    for part in parts:
        toks = part.split()
        if len(toks) > 2:
            return False
        if len(toks) == 2:
            a, b = toks
            if not ((a in _COLORS_EN and b in _BASE_OBJECTS) or (b in _COLORS_EN and a in _BASE_OBJECTS)):
                return False
        elif len(toks) == 1 and toks[0] not in _BASE_OBJECTS and toks[0] not in _COLORS_EN:
            return False
    return True


def simplify_visual_query(query: str, user_text: str = '') -> Tuple[str, List[str]]:
    """Reduce LLM query to short comma-separated labels for YOLO-World.

    Examples:
        "person wearing red clothes" + "前往穿红色衣服的人" -> "person,red"
        "person" -> "person"
        "red car" -> "red,car"
    """
    reasons: List[str] = []
    raw = (query or '').strip()
    if not raw:
        return raw, reasons

    if _is_already_simple(raw):
        # 统一 people/man 等别名
        parts = []
        for part in raw.split(','):
            part = part.strip().lower()
            toks = part.split()
            if len(toks) == 2 and toks[0] in _COLORS_EN and toks[1] in _BASE_OBJECTS:
                parts.append(f'{toks[0]},{_OBJECT_ALIASES.get(toks[1], toks[1])}')
            elif len(toks) == 2 and toks[1] in _COLORS_EN and toks[0] in _BASE_OBJECTS:
                base = _OBJECT_ALIASES.get(toks[0], toks[0])
                parts.append(f'{toks[1]},{base}')
            else:
                tok = toks[0] if toks else part
                parts.append(_OBJECT_ALIASES.get(tok, tok))
        simplified = ','.join(dict.fromkeys(parts))
        if simplified != raw.lower().replace(' ', ''):
            reasons.append(f'visual query alias-normalized: {raw!r} -> {simplified!r}')
        return simplified, reasons

    tokens = _tokens(raw)
    base = _base_from_tokens(tokens)
    color = _color_from_tokens(tokens) or _color_from_user_text(user_text)

    if not base and ('人' in user_text or 'person' in raw.lower()):
        base = 'person'
        reasons.append('inferred base object person from user text')

    if not base:
        # 无法解析时保留去噪后的最短片段
        fallback = ' '.join(tokens[:2]) if tokens else raw
        if fallback != raw:
            reasons.append(f'visual query trimmed (no base noun): {raw!r} -> {fallback!r}')
        return fallback, reasons

    labels: List[str] = [base]
    if color and color not in labels:
        labels.insert(0, color)

    simplified = ','.join(labels)
    if simplified != raw:
        reasons.append(f'visual query simplified for YOLO: {raw!r} -> {simplified!r}')
    return simplified, reasons
