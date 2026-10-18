"""Plot representative settings from existing data without replacing original plots.

python analyze_review.py --output fig/bench_review_selected_metrics.pdf
"""
import argparse
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
SELECTED = [
    ("RTP_gb0.2", "RTP", "#2a78d6"),
    ("RRT_gb0.05_r1.0", "RRT", "#eb6834"),
    ("RRTConnect_r1.0", "RRT-C", "#1baf7a"),
    ("PRM_k10", "PRM", "#eda100"),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "fig/bench_review_selected_metrics.pdf")
    args = parser.parse_args()
    with sqlite3.connect((ROOT / "results/benchmark.db").as_uri() + "?mode=ro", uri=True) as con:
        con.row_factory = sqlite3.Row
        groups = [list(con.execute("SELECT runs.* FROM runs JOIN plannerConfigs "
                                  "ON plannerConfigs.id=runs.plannerid WHERE plannerConfigs.name=?",
                                  ("geometric_" + name,))) for name, _, _ in SELECTED]
    assert all(len(group) == 25 for group in groups)
    exact = [[r for r in group if r["solved"] and not r["approximate_solution"]] for group in groups]
    metrics = [("time", "Planning time (s)", False, 1),
               ("solution_length", "Path length (rad)", True, 1),
               ("solution_smoothness", "Smoothness", True, 1),
               ("minimum_clearance", "Sampled min. clearance (cm)", True, 100)]
    plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7})
    fig, axes = plt.subplots(2, 2, figsize=(6.5, 3.1), layout="constrained")
    for axis, (metric, label, exact_only, factor) in zip(axes.flat, metrics):
        values = [[r[metric] * factor for r in group] for group in (exact if exact_only else groups)]
        boxes = axis.boxplot(values, patch_artist=True, widths=.55,
            medianprops={"color": "black", "linewidth": 1},
            flierprops={"marker": ".", "markersize": 3, "markeredgecolor": "#555555"})
        for patch, (_, _, color) in zip(boxes["boxes"], SELECTED):
            patch.set_facecolor(color)
        axis.set_xticks(range(1, 5), [f"{label}\n{len(group)}/25" for (_, label, _), group in zip(SELECTED, exact)])
        axis.set_ylabel(label)
        axis.grid(axis="y", alpha=.25)
        axis.set_axisbelow(True)
        if metric in ("time", "solution_smoothness"):
            axis.set_yscale("log")
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, metadata={"Creator": "analyze_review.py; existing benchmark.db, no new trials"})
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
