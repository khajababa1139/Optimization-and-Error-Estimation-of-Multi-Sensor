#!/usr/bin/env python3
"""
Circular (polar) plots for stationary UWB / Crazyflie data acquisition.

For each of the 6 tracked variables (x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg)
this produces one polar subplot where:
  - the origin (r = 0) represents the MEAN value of that variable over the run
  - each sample sits at radius = |value - mean(value)|   (deviation magnitude)
  - angle = elapsed-time fraction of the run, mapped onto a full 0-360 sweep
  - point color encodes elapsed time (one shared colorbar for the whole figure)

All 6 variables are laid out as 6 subplots in a single figure.
Multiple CSV files (e.g. repeated iterations of the same stationary test) can
be overlaid on the same subplots, each iteration gets its own marker shape.

Usage
-----
    python plot_stationary_circular.py stationary_north_east_corner_1.csv stationary_north_east_corner_2.csv
    python plot_stationary_circular.py run1.csv run2.csv --labels "Iter 1" "Iter 2" --out circular.png
    python plot_stationary_circular.py run1.csv --show
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

VARIABLES = ["x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg"]
UNITS = {
    "x_m": "m", "y_m": "m", "z_m": "m",
    "roll_deg": "deg", "pitch_deg": "deg", "yaw_deg": "deg",
}
MARKERS = ["o", "^", "s", "D", "v", "P", "X"]


def load_runs(csv_paths, labels=None):
    """Read each CSV into a (label, DataFrame) pair, preserving input order."""
    runs = []
    for i, p in enumerate(csv_paths):
        df = pd.read_csv(p)
        missing = [c for c in ["time_s", *VARIABLES] if c not in df.columns]
        if missing:
            raise ValueError(f"{p} is missing expected column(s): {missing}")
        label = labels[i] if labels and i < len(labels) else Path(p).stem
        runs.append((label, df))
    return runs


def plot_circular(runs, out_path=None, title=None):
    fig, axes = plt.subplots(2, 3, subplot_kw={"projection": "polar"}, figsize=(16, 10))
    axes = axes.flatten()

    sc = None  # last scatter handle, reused for the shared colorbar
    legend_handles, legend_labels = [], []

    for ax, var in zip(axes, VARIABLES):
        for run_idx, (label, df) in enumerate(runs):
            t = df["time_s"].to_numpy()
            v = df[var].to_numpy()
            mean_v = np.mean(v)

            t_span = t.max() - t.min()
            theta = 2 * np.pi * (t - t.min()) / t_span if t_span > 0 else np.zeros_like(t)
            r = np.abs(v - mean_v)

            sc = ax.scatter(
                theta, r, c=t, cmap="viridis", s=20,
                marker=MARKERS[run_idx % len(MARKERS)],
                alpha=0.75, edgecolors="none",
            )
            if var == VARIABLES[0]:
                # collect one legend entry per run (marker shape only, color is time)
                proxy = ax.scatter([], [], c="gray", marker=MARKERS[run_idx % len(MARKERS)], label=label)
                legend_handles.append(proxy)
                legend_labels.append(label)

        # explicit mean marker at the origin
        mean_marker = ax.scatter([0], [0], c="red", s=90, marker="x", zorder=5, linewidths=2)
        ax.set_title(f"{var}  [{UNITS[var]}]", fontsize=11, pad=16)
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.tick_params(labelsize=7)

    legend_handles.append(mean_marker)
    legend_labels.append("mean (center)")
    fig.legend(legend_handles, legend_labels, loc="lower center",
               ncol=len(legend_labels), frameon=False, bbox_to_anchor=(0.5, -0.02))

    if sc is not None:
        cbar = fig.colorbar(sc, ax=axes.tolist(), shrink=0.6, pad=0.03)
        cbar.set_label("time (s)")

    fig.suptitle(
        title or "Stationary acquisition — deviation from mean (radius) vs. elapsed time (angle)",
        fontsize=14,
    )

    if out_path:
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"Saved: {out_path}")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_files", nargs="+", help="One or more stationary-acquisition CSV files")
    parser.add_argument("--labels", nargs="*", default=None, help="Optional labels for each CSV (same order as csv_files)")
    parser.add_argument("--out", default="stationary_circular_plot.png", help="Output image path")
    parser.add_argument("--title", default=None, help="Custom figure title")
    parser.add_argument("--show", action="store_true", help="Show the plot interactively instead of/in addition to saving")
    args = parser.parse_args()

    runs = load_runs(args.csv_files, args.labels)
    plot_circular(runs, out_path=args.out, title=args.title)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()