#!/usr/bin/env python3
"""Offline evaluator for thesis Section 5.4."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List

from common import (
    direct_parse,
    robust_parse,
    tpa_match,
    validate_baseline_command,
    validate_proposed_command,
)


def load_tasks(path: Path) -> List[Dict[str, Any]]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_jsonl(path: Path, records: List[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in records:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def summarize(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    by_model: Dict[str, List[Dict[str, Any]]] = {}
    for row in records:
        by_model.setdefault(row["model_name"], []).append(row)

    out = []
    for model_name, rows in by_model.items():
        total = len(rows)
        jsr_baseline = sum(1 for r in rows if r["baseline_json_ok"]) / total * 100.0
        jsr_proposed = sum(1 for r in rows if r["proposed_json_ok"]) / total * 100.0
        proposed_valid_rows = [r for r in rows if r["proposed_json_ok"]]
        if proposed_valid_rows:
            tpa = sum(1 for r in proposed_valid_rows if r["proposed_tpa_ok"]) / len(proposed_valid_rows) * 100.0
        else:
            tpa = 0.0
        avg_latency = sum(float(r["latency_sec"]) for r in rows) / total
        out.append(
            {
                "model_name": model_name,
                "task_count": total,
                "jsr_baseline_pct": round(jsr_baseline, 2),
                "jsr_proposed_pct": round(jsr_proposed, 2),
                "tpa_pct": round(tpa, 2),
                "avg_latency_sec": round(avg_latency, 3),
            }
        )
    return out


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_eval(
    tasks: List[Dict[str, Any]],
    model_names: List[str],
    raw_rows: List[Dict[str, Any]],
    jsonl_path: Path,
    csv_path: Path,
) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    raw_map = {(row["model_name"], row["task_id"]): row for row in raw_rows}
    for model_name in model_names:
        for task in tasks:
            try:
                resp = raw_map[(model_name, task["id"])]
                raw_output = resp["raw_output"]

                baseline = direct_parse(raw_output)
                proposed = robust_parse(raw_output)

                baseline_valid = validate_baseline_command(baseline.parsed) if baseline.ok else None
                proposed_valid = validate_proposed_command(proposed.parsed) if proposed.ok else None

                row = {
                    "task_id": task["id"],
                    "task_text": task["text"],
                    "model_name": model_name,
                    "latency_sec": resp["latency_sec"],
                    "call_attempts": resp.get("attempts", 1),
                    "raw_output": raw_output,
                    "baseline_json_ok": baseline_valid is not None,
                    "baseline_error": "" if baseline_valid is not None else baseline.error,
                    "baseline_parsed": baseline_valid,
                    "proposed_json_ok": proposed_valid is not None,
                    "proposed_error": "" if proposed_valid is not None else proposed.error,
                    "proposed_parsed": proposed_valid,
                    "proposed_tpa_ok": bool(proposed_valid is not None and tpa_match(task, proposed_valid)),
                }
            except Exception as exc:
                row = {
                    "task_id": task["id"],
                    "task_text": task["text"],
                    "model_name": model_name,
                    "latency_sec": 0.0,
                    "call_attempts": 0,
                    "raw_output": "",
                    "baseline_json_ok": False,
                    "baseline_error": f"LLM_CALL_FAILED: {exc}",
                    "baseline_parsed": None,
                    "proposed_json_ok": False,
                    "proposed_error": f"LLM_CALL_FAILED: {exc}",
                    "proposed_parsed": None,
                    "proposed_tpa_ok": False,
                }
            records.append(row)
            write_jsonl(jsonl_path, records)
            write_csv(csv_path, summarize(records))
    return records


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", default="tasks_offline.json")
    parser.add_argument("--raw-cache", required=True)
    parser.add_argument("--result-jsonl", required=True)
    parser.add_argument("--result-csv", required=True)
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent
    tasks = load_tasks(base_dir / args.tasks)
    raw_rows = read_jsonl(base_dir / args.raw_cache)
    if args.limit > 0:
        tasks = tasks[: args.limit]

    jsonl_path = base_dir / args.result_jsonl
    csv_path = base_dir / args.result_csv
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    records = run_eval(tasks, args.models, raw_rows, jsonl_path, csv_path)
    write_jsonl(jsonl_path, records)
    write_csv(csv_path, summarize(records))
    print(f"Wrote {len(records)} records to {args.result_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
