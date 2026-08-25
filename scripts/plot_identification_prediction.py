#!/usr/bin/env python3
"""Plot per-joint simulation, true-regressor, and identified torques."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def parse_args() -> argparse.Namespace:
    """Parse prediction CSV and output image paths."""
    parser = argparse.ArgumentParser()
    parser.add_argument("prediction_csv", type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def load_predictions(path: Path) -> dict[int, dict[str, list[float]]]:
    """Load long-form prediction rows grouped by one-based joint index."""
    grouped: dict[int, dict[str, list[float]]] = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["included"] != "1":
                continue
            joint = int(row["joint"])
            values = grouped.setdefault(
                joint,
                {
                    "time": [],
                    "simulation": [],
                    "true": [],
                    "identified": [],
                },
            )
            values["time"].append(float(row["time"]))
            values["simulation"].append(float(row["tau_simulation"]))
            values["true"].append(float(row["tau_regressor_true"]))
            values["identified"].append(float(row["tau_identified"]))
    return grouped


def main() -> None:
    """Render torque traces and identified residuals for every joint."""
    args = parse_args()
    import matplotlib.pyplot as plt

    grouped = load_predictions(args.prediction_csv)
    figure, axes = plt.subplots(len(grouped), 2, figsize=(14, 3 * len(grouped)))
    if len(grouped) == 1:
        axes = [axes]
    for row_index, joint in enumerate(sorted(grouped)):
        values = grouped[joint]
        torque_axis, error_axis = axes[row_index]
        torque_axis.plot(values["time"], values["simulation"], label="simulation")
        torque_axis.plot(values["time"], values["true"], "--", label="Y theta_true")
        torque_axis.plot(
            values["time"], values["identified"], ":", label="Y beta_hat"
        )
        torque_axis.set_ylabel(f"joint {joint} torque [Nm]")
        torque_axis.legend(loc="best")
        residual = [
            estimate - measured
            for estimate, measured in zip(
                values["identified"], values["simulation"], strict=True
            )
        ]
        error_axis.plot(values["time"], residual)
        error_axis.set_ylabel(f"joint {joint} error [Nm]")
    axes[-1][0].set_xlabel("time [s]")
    axes[-1][1].set_xlabel("time [s]")
    figure.tight_layout()
    output = args.output or args.prediction_csv.with_suffix(".png")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
