#!/usr/bin/env python3
"""Create deterministic noisy identification columns without altering truth."""

from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path


def parse_args() -> argparse.Namespace:
    """Parse deterministic state, torque, and sparse-outlier settings."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dof", type=int, default=6)
    parser.add_argument("--seed", type=int, default=20260826)
    parser.add_argument("--sigma-q", type=float, default=0.0)
    parser.add_argument("--sigma-qd", type=float, default=0.0)
    parser.add_argument("--sigma-qdd", type=float, default=0.0)
    parser.add_argument("--sigma-tau", type=float, default=0.0)
    parser.add_argument("--outlier-fraction", type=float, default=0.0)
    parser.add_argument("--outlier-amplitude", type=float, default=1.0)
    return parser.parse_args()


def read_truth(path: Path, dof: int) -> tuple[list[str], list[dict[str, str]]]:
    """Read the trusted schema and require every source column explicitly."""
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("input CSV has no header")
        required = []
        for prefix in ("q", "qd", "qdd_mujoco", "tau_effort"):
            required.extend(f"{prefix}{joint}" for joint in range(dof))
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"input CSV is missing columns: {missing}")
        return list(reader.fieldnames), list(reader)


def add_noise(
    rows: list[dict[str, str]], args: argparse.Namespace
) -> tuple[list[list[float]], set[int]]:
    """Generate noisy values and apply exact sparse sample-joint torque spikes."""
    generator = random.Random(args.seed)
    noisy: list[list[float]] = []
    for row in rows:
        values: list[float] = []
        for prefix, sigma in (
            ("q", args.sigma_q),
            ("qd", args.sigma_qd),
            ("qdd_mujoco", args.sigma_qdd),
            ("tau_effort", args.sigma_tau),
        ):
            for joint in range(args.dof):
                truth = float(row[f"{prefix}{joint}"])
                value = truth + (generator.gauss(0.0, sigma) if sigma else 0.0)
                if not math.isfinite(value):
                    raise ValueError("noise generation produced a non-finite value")
                values.append(value)
        noisy.append(values)

    observation_count = len(rows) * args.dof
    outlier_count = round(args.outlier_fraction * observation_count)
    if outlier_count < 0 or outlier_count > observation_count:
        raise ValueError("outlier fraction must be within [0, 1]")
    outliers = set(generator.sample(range(observation_count), outlier_count))
    torque_offset = 3 * args.dof
    for flat_index in outliers:
        sample, joint = divmod(flat_index, args.dof)
        sign = -1.0 if generator.getrandbits(1) == 0 else 1.0
        noisy[sample][torque_offset + joint] += sign * args.outlier_amplitude
    return noisy, outliers


def write_dataset(
    header: list[str],
    rows: list[dict[str, str]],
    noisy: list[list[float]],
    outliers: set[int],
    args: argparse.Namespace,
) -> None:
    """Append explicit noisy prefixes and per-joint outlier indicators."""
    args.output.parent.mkdir(parents=True, exist_ok=True)
    extra_header = [
        f"{prefix}{joint}"
        for prefix in ("q_noisy", "qd_noisy", "qdd_noisy", "tau_noisy")
        for joint in range(args.dof)
    ] + [f"outlier{joint}" for joint in range(args.dof)]
    if any(name in header for name in extra_header):
        raise ValueError("output columns already exist in the input dataset")
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header + extra_header)
        for sample, (row, values) in enumerate(zip(rows, noisy, strict=True)):
            original = [row[name] for name in header]
            formatted = [format(value, ".17g") for value in values]
            indicators = [
                "1" if sample * args.dof + joint in outliers else "0"
                for joint in range(args.dof)
            ]
            writer.writerow(original + formatted + indicators)


def write_metadata(args: argparse.Namespace, outlier_count: int) -> None:
    """Write exact corruption settings next to the derived CSV."""
    metadata = args.output.with_suffix(".noise.meta.yaml")
    with metadata.open("w") as stream:
        stream.write("schema_version: 1\n")
        stream.write(f'input: "{args.input}"\n')
        stream.write(f"seed: {args.seed}\n")
        stream.write(f"sigma_q: {args.sigma_q:.17g}\n")
        stream.write(f"sigma_qd: {args.sigma_qd:.17g}\n")
        stream.write(f"sigma_qdd: {args.sigma_qdd:.17g}\n")
        stream.write(f"sigma_tau: {args.sigma_tau:.17g}\n")
        stream.write(f"outlier_fraction: {args.outlier_fraction:.17g}\n")
        stream.write(f"outlier_amplitude: {args.outlier_amplitude:.17g}\n")
        stream.write(f"outlier_observations: {outlier_count}\n")


def main() -> None:
    """Generate one reproducible noisy training dataset and its metadata."""
    args = parse_args()
    header, rows = read_truth(args.input, args.dof)
    noisy, outliers = add_noise(rows, args)
    write_dataset(header, rows, noisy, outliers, args)
    write_metadata(args, len(outliers))
    print(f"wrote {len(rows)} samples with {len(outliers)} outliers to {args.output}")


if __name__ == "__main__":
    main()
