#!/usr/bin/env python3
"""Collect raw model outputs once and cache them for parser ablation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

from common import call_ollama


def load_tasks(path: Path) -> List[Dict[str, Any]]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_jsonl(path: Path, records: List[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in records:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", required=True)
    parser.add_argument("--result-jsonl", required=True)
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent
    tasks = load_tasks(base_dir / args.tasks)
    if args.limit > 0:
        tasks = tasks[: args.limit]

    result_path = base_dir / args.result_jsonl
    result_path.parent.mkdir(parents=True, exist_ok=True)

    records: List[Dict[str, Any]] = []
    for model_name in args.models:
        for task in tasks:
            resp = call_ollama(model_name, task["text"], json_mode=False)
            row = {
                "task_id": task["id"],
                "task_text": task["text"],
                "model_name": model_name,
                "latency_sec": resp["latency_sec"],
                "call_attempts": resp.get("attempts", 1),
                "raw_output": resp["raw_output"],
            }
            records.append(row)
            write_jsonl(result_path, records)
            print(f"[collect] {model_name} {task['id']}")

    print(f"Wrote {len(records)} cached raw outputs to {args.result_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
