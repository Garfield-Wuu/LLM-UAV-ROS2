#!/usr/bin/env python3
"""Build thesis-ready artifacts from offline/online evaluation results."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: List[Dict[str, str]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def build_table(offline_rows: List[Dict[str, str]], online_rows: List[Dict[str, str]]) -> List[Dict[str, str]]:
    offline_map = {row["model_name"]: row for row in offline_rows}
    online_map: Dict[tuple, Dict[str, str]] = {}
    for row in online_rows:
        online_map[(row["model_name"], row["group"])] = row

    rows: List[Dict[str, str]] = []
    for model_name, offline in offline_map.items():
        baseline_online = online_map.get((model_name, "baseline"), {})
        proposed_online = online_map.get((model_name, "proposed"), {})
        rows.append(
            {
                "model_name": model_name,
                "JSR_baseline_pct": offline.get("jsr_baseline_pct", ""),
                "JSR_proposed_pct": offline.get("jsr_proposed_pct", ""),
                "TPA_pct": offline.get("tpa_pct", ""),
                "MCR_baseline_pct": baseline_online.get("mcr_pct", ""),
                "MCR_proposed_pct": proposed_online.get("mcr_pct", ""),
                "ASC": proposed_online.get("asc", ""),
            }
        )
    return rows


def build_discussion(table_rows: List[Dict[str, str]]) -> str:
    if not table_rows:
        return "No experiment rows found.\n"

    ordered = sorted(table_rows, key=lambda r: float(r["TPA_pct"] or 0.0), reverse=True)
    best = ordered[0]
    jsr_deltas = [
        (row["model_name"], float(row["JSR_proposed_pct"] or 0.0) - float(row["JSR_baseline_pct"] or 0.0))
        for row in table_rows
    ]
    mcr_deltas = [
        (row["model_name"], float(row["MCR_proposed_pct"] or 0.0) - float(row["MCR_baseline_pct"] or 0.0))
        for row in table_rows
    ]
    best_jsr = max(jsr_deltas, key=lambda x: x[1])
    best_mcr = max(mcr_deltas, key=lambda x: x[1])
    lines = [
        "# Section 5.6 Discussion Notes",
        "",
        f"- 综合 `TPA` 表现最优的模型是 `{best['model_name']}`。",
        f"- `JSR` 提升最明显的模型是 `{best_jsr[0]}`，增幅为 `{best_jsr[1]:.2f}` 个百分点。",
        f"- `MCR` 提升最明显的模型是 `{best_mcr[0]}`，增幅为 `{best_mcr[1]:.2f}` 个百分点。",
        "- 本轮补充实验关闭了 API 级 `format=json`，并将 baseline 改为严格解析 + 严格校验，因此更能真实反映鲁棒提取算法对结构化可执行性的贡献。",
        "- 如果 `JSR` 提升明显且 `MCR` 同步提升，可在 5.6 中强调：鲁棒提取算法不仅提升了解析成功率，也提升了可执行任务的闭环成功率。",
        "- 如果个别模型 `JSR` 提升大于 `MCR` 提升，应说明：解析恢复只是必要条件，最终闭环仍受轨迹、姿态调整与动作序列合理性影响。",
        "",
        "## Table Snapshot",
        "",
    ]
    for row in table_rows:
        lines.append(
            f"- {row['model_name']}: JSR {row['JSR_baseline_pct']} -> {row['JSR_proposed_pct']}, "
            f"TPA {row['TPA_pct']}, MCR {row['MCR_baseline_pct']} -> {row['MCR_proposed_pct']}, ASC {row['ASC']}"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    base_dir = Path(__file__).resolve().parent
    results_dir = base_dir / "results"
    offline_rows = read_csv(results_dir / "offline_summary.csv")
    online_rows = read_csv(results_dir / "online_summary.csv")
    table_rows = build_table(offline_rows, online_rows)
    write_csv(results_dir / "table_5_1.csv", table_rows)
    # Lightweight chart source for Figure 5.2
    chart_rows: List[Dict[str, str]] = []
    for row in table_rows:
        chart_rows.extend(
            [
                {"model_name": row["model_name"], "metric": "JSR", "group": "baseline", "value_pct": row["JSR_baseline_pct"]},
                {"model_name": row["model_name"], "metric": "JSR", "group": "proposed", "value_pct": row["JSR_proposed_pct"]},
                {"model_name": row["model_name"], "metric": "MCR", "group": "baseline", "value_pct": row["MCR_baseline_pct"]},
                {"model_name": row["model_name"], "metric": "MCR", "group": "proposed", "value_pct": row["MCR_proposed_pct"]},
            ]
        )
    write_csv(results_dir / "figure_5_2_data.csv", chart_rows)
    (results_dir / "discussion_notes.md").write_text(build_discussion(table_rows), encoding="utf-8")
    print("Artifacts written to results/table_5_1.csv, results/figure_5_2_data.csv and results/discussion_notes.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
