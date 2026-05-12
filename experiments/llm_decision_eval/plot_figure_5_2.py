#!/usr/bin/env python3
"""Plot thesis Figure 5.2 from aggregated JSR/MCR CSV data."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


RESULTS_DIR = Path(__file__).resolve().parent / "results"
INPUT_CSV = RESULTS_DIR / "figure_5_2_data.csv"
OUTPUT_PNG = RESULTS_DIR / "figure_5_2.png"
OUTPUT_PDF = RESULTS_DIR / "figure_5_2.pdf"

METRIC_ORDER = ["JSR", "MCR"]
GROUP_ORDER = ["baseline", "proposed"]
GROUP_LABELS = {"baseline": "Baseline", "proposed": "Proposed"}
COLORS = {"baseline": "#B0B7C3", "proposed": "#2F6DB2"}
HATCHES = {"baseline": "//", "proposed": None}
MODEL_LABELS = {
    "Qwen2.5-7B-Instruct": "Qwen2.5-7B",
    "Llama 3.2-3B": "Llama 3.2-3B",
    "Phi-3.5-mini-Instruct": "Phi-3.5-mini",
}


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def build_metric_table(rows: List[Dict[str, str]]) -> Dict[str, Dict[str, Dict[str, float]]]:
    table: Dict[str, Dict[str, Dict[str, float]]] = {}
    for row in rows:
        model_name = row["model_name"]
        metric = row["metric"]
        group = row["group"]
        value = float(row["value_pct"])
        table.setdefault(metric, {}).setdefault(model_name, {})[group] = value
    return table


def add_bar_labels(ax: plt.Axes, bars, values: np.ndarray) -> None:
    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height() + 1.2,
            f"{value:.0f}",
            ha="center",
            va="bottom",
            fontsize=9,
            color="#1E2430",
        )


def add_uplift_labels(ax: plt.Axes, x: np.ndarray, baseline: np.ndarray, proposed: np.ndarray, width: float) -> None:
    for xi, base, prop in zip(x, baseline, proposed):
        uplift = prop - base
        ax.text(
            xi + width / 2.0,
            max(base, prop) + 7.0,
            f"+{uplift:.0f}",
            ha="center",
            va="bottom",
            fontsize=9,
            fontweight="bold",
            color=COLORS["proposed"],
        )


def plot_metric(ax: plt.Axes, metric_name: str, panel_label: str, metric_rows: Dict[str, Dict[str, float]]) -> None:
    models = list(metric_rows.keys())
    labels = [MODEL_LABELS.get(model, model) for model in models]
    baseline = np.array([metric_rows[model].get("baseline", 0.0) for model in models], dtype=float)
    proposed = np.array([metric_rows[model].get("proposed", 0.0) for model in models], dtype=float)

    x = np.arange(len(models))
    width = 0.34

    baseline_bars = ax.bar(
        x - width / 2.0,
        baseline,
        width=width,
        label=GROUP_LABELS["baseline"],
        color=COLORS["baseline"],
        edgecolor="#4A5568",
        linewidth=0.8,
        hatch=HATCHES["baseline"],
        zorder=3,
    )
    proposed_bars = ax.bar(
        x + width / 2.0,
        proposed,
        width=width,
        label=GROUP_LABELS["proposed"],
        color=COLORS["proposed"],
        edgecolor="#27496D",
        linewidth=0.8,
        zorder=3,
    )

    add_bar_labels(ax, baseline_bars, baseline)
    add_bar_labels(ax, proposed_bars, proposed)
    add_uplift_labels(ax, x, baseline, proposed, width)

    ax.set_title(f"({panel_label}) {metric_name}", fontsize=13, fontweight="bold")
    ax.set_xticks(x, labels, fontsize=9)
    ax.set_ylim(0, 112)
    ax.set_yticks(np.arange(0, 101, 20))
    ax.set_ylabel("Percentage (%)", fontsize=10)
    ax.grid(axis="y", linestyle="--", linewidth=0.8, alpha=0.35, zorder=0)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)


def main() -> int:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.unicode_minus": False,
            "figure.dpi": 180,
            "savefig.dpi": 300,
        }
    )

    rows = read_rows(INPUT_CSV)
    table = build_metric_table(rows)

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.6), constrained_layout=True)
    for ax, metric_name, panel_label in zip(axes, METRIC_ORDER, ("a", "b")):
        plot_metric(ax, metric_name, panel_label, table[metric_name])

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.02))

    fig.savefig(OUTPUT_PNG, bbox_inches="tight")
    fig.savefig(OUTPUT_PDF, bbox_inches="tight")
    print(f"Wrote {OUTPUT_PNG}")
    print(f"Wrote {OUTPUT_PDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
