#!/usr/bin/env python3
"""
Linear time-series plot for stationary UWB / Crazyflie data acquisition.

Plots all 6 tracked variables (x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg)
against time on ONE set of axes (as requested — no separate subplots).

Note: position (meters) and orientation (degrees) share the same y-axis by
default, which is what was asked for, but it means degree-scale curves can
visually dominate small position values. Pass --twin-axis to instead put
position (m) on the left y-axis and orientation (deg) on the right y-axis
while still keeping everything in a single plot/figure.

Multiple CSV files (repeated iterations) can be passed at once; each run's
set of curves uses a distinct line style so runs stay distinguishable.

Usage
-----
    python plot_stationary_linear.py stationary_north_east_corner_1.csv stationary_north_east_corner_2.csv
    python plot_stationary_linear.py run1.csv --twin-axis
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

VARIABLES = ["x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg"]
POSITION_VARS = ["x_m", "y_m", "z_m"]
ORIENTATION_VARS = ["roll_deg", "pitch_deg", "yaw_deg"]
LINESTYLES = ["-", "--", ":", "-."]
COLORS = plt.cm.tab10.colors


def load_runs(csv_paths, labels=None):
    runs = []
    for i, p in enumerate(csv_paths):
        df = pd.read_csv(p)
        missing = [c for c in ["time_s", *VARIABLES] if c not in df.columns]
        if missing:
            raise ValueError(f"{p} is missing expected column(s): {missing}")
        label = labels[i] if labels and i < len(labels) else Path(p).stem
        runs.append((label, df))
    return runs


def plot_linear(runs, out_path=None, title=None, twin_axis=False):
    fig, ax_left = plt.subplots(figsize=(13, 6))
    ax_right = ax_left.twinx() if twin_axis else None

    lines, labels_ = [], []
    for run_idx, (label, df) in enumerate(runs):
        ls = LINESTYLES[run_idx % len(LINESTYLES)]
        for var_idx, var in enumerate(VARIABLES):
            target_ax = ax_left
            if twin_axis and var in ORIENTATION_VARS:
                target_ax = ax_right
            line, = target_ax.plot(
                df["time_s"], df[var], linestyle=ls,
                color=COLORS[var_idx % len(COLORS)],
                linewidth=1.3,
            )
            lines.append(line)
            labels_.append(f"{var} ({label})")

    ax_left.set_xlabel("time (s)")
    if twin_axis:
        ax_left.set_ylabel("position (m)")
        ax_right.set_ylabel("orientation (deg)")
    else:
        ax_left.set_ylabel("value (m or deg — see legend)")

    ax_left.set_title(title or "Stationary acquisition — all variables vs. time")
    ax_left.grid(True, alpha=0.3)
    ax_left.legend(lines, labels_, loc="upper right", fontsize=8, ncol=2)

    fig.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"Saved: {out_path}")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_files", nargs="+", help="One or more stationary-acquisition CSV files")
    parser.add_argument("--labels", nargs="*", default=None, help="Optional labels for each CSV (same order as csv_files)")
    parser.add_argument("--out", default="stationary_linear_plot.png", help="Output image path")
    parser.add_argument("--title", default=None, help="Custom figure title")
    parser.add_argument("--twin-axis", action="store_true", help="Put position (m) and orientation (deg) on separate y-axes")
    parser.add_argument("--show", action="store_true", help="Show the plot interactively instead of/in addition to saving")
    args = parser.parse_args()

    runs = load_runs(args.csv_files, args.labels)
    plot_linear(runs, out_path=args.out, title=args.title, twin_axis=args.twin_axis)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()