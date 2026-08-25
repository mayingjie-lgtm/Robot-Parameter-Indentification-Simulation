#!/usr/bin/env python3
"""Plot and summarize MuJoCo truth against recorded finite differences."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List


def parse_args() -> argparse.Namespace:
    """Parse the simulation-truth CSV and output directory arguments."""
    parser = argparse.ArgumentParser(
        description="Analyze qdd_diff-qdd_mujoco and tau_cmd-tau_effort."
    )
    parser.add_argument("csv", type=Path, help="Simulation-truth CSV path")
    parser.add_argument(
        "--output-dir", type=Path, help="Directory for plots and metrics"
    )
    return parser.parse_args()


def numbered_columns(fieldnames: List[str], prefix: str) -> List[str]:
    """Return numeric-suffix columns for a prefix in joint-index order."""
    columns = [name for name in fieldnames if name.startswith(prefix)]
    try:
        return sorted(columns, key=lambda name: int(name[len(prefix) :]))
    except ValueError as exc:
        raise ValueError(f"Invalid numeric column for prefix {prefix!r}") from exc


def load_columns(path: Path) -> Dict[str, List[float]]:
    """Load every numeric CSV column while rejecting missing or invalid rows."""
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("CSV header is missing")
        data = {name: [] for name in reader.fieldnames}
        for row_number, row in enumerate(reader, start=2):
            for name in reader.fieldnames:
                try:
                    value = float(row[name])
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Invalid value in row {row_number}, column {name}"
                    ) from exc
                if not math.isfinite(value):
                    raise ValueError(
                        f"Non-finite value in row {row_number}, column {name}"
                    )
                data[name].append(value)
    return data


def error_stats(values: List[float]) -> tuple[float, float, float]:
    """Return mean, root-mean-square, and maximum absolute error."""
    if not values:
        raise ValueError("Cannot summarize an empty error vector")
    mean = sum(values) / len(values)
    rmse = math.sqrt(sum(value * value for value in values) / len(values))
    max_abs = max(abs(value) for value in values)
    return mean, rmse, max_abs


def plot_joint_errors(
    times: List[float], errors: List[List[float]], title: str, ylabel: str, path: Path
) -> None:
    """Save one shared-axis subplot per joint for an error signal."""
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(errors), 1, figsize=(12, 2.2 * len(errors)), sharex=True)
    if len(errors) == 1:
        axes = [axes]
    for joint, (axis, values) in enumerate(zip(axes, errors)):
        axis.plot(times, values, linewidth=0.7)
        axis.set_ylabel(f"J{joint + 1}\n{ylabel}")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> int:
    """Analyze a schema-v2 CSV and write metrics plus two diagnostic plots."""
    args = parse_args()
    output_dir = args.output_dir or args.csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    data = load_columns(args.csv)
    fieldnames = list(data)

    qdd_mujoco = numbered_columns(fieldnames, "qdd_mujoco")
    qdd_diff = numbered_columns(fieldnames, "qdd_diff")
    tau_cmd = numbered_columns(fieldnames, "tau_cmd")
    tau_effort = numbered_columns(fieldnames, "tau_effort")
    if not qdd_mujoco or not (
        len(qdd_mujoco) == len(qdd_diff) == len(tau_cmd) == len(tau_effort)
    ):
        raise ValueError("CSV does not contain a complete simulation-truth schema")

    times = data["time_begin"]
    qdd_errors: List[List[float]] = []
    tau_errors: List[List[float]] = []
    metrics = [f"samples: {len(times)}", f"dof: {len(qdd_mujoco)}"]
    dt_values = [end - begin for begin, end in zip(data["time_begin"], data["time_end"])]
    dt_mean, dt_rmse, dt_max = error_stats(dt_values)
    metrics.append(f"dt_mean: {dt_mean:.17g}")
    metrics.append(f"dt_rms: {dt_rmse:.17g}")
    metrics.append(f"dt_max: {dt_max:.17g}")

    for joint in range(len(qdd_mujoco)):
        qdd_error = [
            diff - exact
            for diff, exact in zip(data[qdd_diff[joint]], data[qdd_mujoco[joint]])
        ]
        tau_error = [
            command - effort
            for command, effort in zip(data[tau_cmd[joint]], data[tau_effort[joint]])
        ]
        qdd_errors.append(qdd_error)
        tau_errors.append(tau_error)
        qdd_mean, qdd_rmse, qdd_max = error_stats(qdd_error)
        tau_mean, tau_rmse, tau_max = error_stats(tau_error)
        metrics.append(
            f"joint_{joint + 1}_qdd_error: mean={qdd_mean:.9g}, "
            f"rmse={qdd_rmse:.9g}, max_abs={qdd_max:.9g}"
        )
        metrics.append(
            f"joint_{joint + 1}_tau_error: mean={tau_mean:.9g}, "
            f"rmse={tau_rmse:.9g}, max_abs={tau_max:.9g}"
        )

    stem = args.csv.stem
    metrics_path = output_dir / f"{stem}_truth_metrics.txt"
    metrics_path.write_text("\n".join(metrics) + "\n", encoding="utf-8")
    plot_joint_errors(
        times,
        qdd_errors,
        "Finite-difference acceleration minus MuJoCo qacc",
        "rad/s²",
        output_dir / f"{stem}_qdd_error.png",
    )
    plot_joint_errors(
        times,
        tau_errors,
        "Command torque minus MuJoCo actuator effort",
        "Nm",
        output_dir / f"{stem}_tau_error.png",
    )
    print("\n".join(metrics))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
