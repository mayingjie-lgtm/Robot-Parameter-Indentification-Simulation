#!/usr/bin/env python3
"""Verify reBot-DM excitation safety and forward-friction data semantics.

This verifier checks recorded rollout data only. Constraint-row decomposition is
validated separately by ``rebot_constraint_force_diagnostic``; that diagnostic
proves that J1-J6 ``tau_constraint`` contains only DOF-friction contribution in
the accepted excitation runtime scene.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path
from typing import Iterable

import yaml

DOF = 6
SLIDING_SPEED_THRESHOLD = 0.05
TIME_TOL = 1e-12
VALUE_TOL = 1e-10


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of one experiment artifact."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rms(values: Iterable[float]) -> float:
    """Return root-mean-square while rejecting an empty sequence."""
    data = list(values)
    if not data:
        raise ValueError("cannot compute RMS of an empty sequence")
    return math.sqrt(sum(value * value for value in data) / len(data))


def _joint_values(rows: list[dict[str, str]], prefix: str, joint: int) -> list[float]:
    """Read one numbered joint column from parsed CSV rows."""
    key = f"{prefix}{joint}"
    try:
        return [float(row[key]) for row in rows]
    except KeyError as exc:
        raise RuntimeError(f"CSV missing required column: {key}") from exc


def _vector(config: dict, key: str) -> list[float]:
    """Read one six-element numeric vector from YAML configuration."""
    values = [float(value) for value in config.get(key, [])]
    if len(values) != DOF:
        raise RuntimeError(f"{key} must contain exactly {DOF} values")
    return values


def main() -> int:
    """Run the execution/data-quality gate for one reBot excitation CSV."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument(
        "--controller-config",
        type=Path,
        default=Path("config/rebot_dm_excitation_controller.yaml"),
    )
    parser.add_argument("--expected-duration", type=float, default=30.0)
    parser.add_argument("--expected-dt", type=float, default=0.001)
    args = parser.parse_args()

    csv_path = args.csv.resolve()
    metadata_path = csv_path.with_suffix(".meta.yaml")
    if not csv_path.is_file():
        raise RuntimeError(f"CSV does not exist: {csv_path}")
    if not metadata_path.is_file():
        raise RuntimeError(f"metadata does not exist: {metadata_path}")

    controller_config = yaml.safe_load(args.controller_config.read_text())
    metadata = yaml.safe_load(metadata_path.read_text())
    rows = list(csv.DictReader(csv_path.open(newline="")))
    if not rows:
        raise RuntimeError("CSV contains no samples")

    lower = _vector(controller_config, "joint_lower_limits")
    upper = _vector(controller_config, "joint_upper_limits")
    velocity_limits = _vector(controller_config, "joint_velocity_safety_limits")
    torque_limits = _vector(controller_config, "joint_torque_limits")
    frictionloss = [float(value) for value in metadata.get("joint_frictionloss", [])]
    if len(frictionloss) != DOF:
        raise RuntimeError("metadata joint_frictionloss must contain six values")

    required_scalars = ["time_begin", "time_end", "saturated", "contact_count"]
    for key in required_scalars:
        if key not in rows[0]:
            raise RuntimeError(f"CSV missing required column: {key}")

    all_numeric_columns = required_scalars.copy()
    for prefix in (
        "q",
        "qd",
        "qdd_mujoco",
        "tau_cmd",
        "tau_effort",
        "tau_constraint",
    ):
        all_numeric_columns.extend(f"{prefix}{joint}" for joint in range(DOF))
    for row_index, row in enumerate(rows, start=2):
        for key in all_numeric_columns:
            try:
                value = float(row[key])
            except (KeyError, ValueError) as exc:
                raise RuntimeError(f"invalid {key} at CSV row {row_index}") from exc
            if not math.isfinite(value):
                raise RuntimeError(f"non-finite {key} at CSV row {row_index}")

    time_begin = [float(row["time_begin"]) for row in rows]
    time_end = [float(row["time_end"]) for row in rows]
    if any(b <= a for a, b in zip(time_begin, time_begin[1:])):
        raise RuntimeError("time_begin is not strictly monotonic")
    dt_values = [end - begin for begin, end in zip(time_begin, time_end)]
    if any(abs(dt - args.expected_dt) > TIME_TOL for dt in dt_values):
        raise RuntimeError("per-sample dt differs from expected simulation dt")
    duration = time_end[-1] - time_begin[0]
    if abs(duration - args.expected_duration) > 5e-10:
        raise RuntimeError(
            f"duration {duration:.12f} does not match {args.expected_duration:.12f}"
        )

    saturated_count = sum(int(float(row["saturated"])) != 0 for row in rows)
    unexpected_contact_count = sum(int(float(row["contact_count"])) for row in rows)
    if saturated_count != 0:
        raise RuntimeError(f"saturation_count={saturated_count}, expected 0")
    if unexpected_contact_count != 0:
        raise RuntimeError(
            f"unexpected_contact_count={unexpected_contact_count}, expected 0"
        )

    q_stats: list[tuple[float, float]] = []
    qd_stats: list[tuple[float, float]] = []
    qdd_stats: list[tuple[float, float]] = []
    tau_stats: list[tuple[float, float]] = []
    coverage: list[tuple[float, float, float, float, float, float, float]] = []
    tau_rmse: list[float] = []
    tau_max_abs: list[float] = []
    friction_bound_max_violation: list[float] = []
    sliding_direction_max_violation: list[float] = []
    sliding_sample_counts: list[int] = []
    saturated_sliding_counts: list[int] = []

    for joint in range(DOF):
        q = _joint_values(rows, "q", joint)
        qd = _joint_values(rows, "qd", joint)
        qdd = _joint_values(rows, "qdd_mujoco", joint)
        tau_cmd = _joint_values(rows, "tau_cmd", joint)
        tau_effort = _joint_values(rows, "tau_effort", joint)
        tau_constraint = _joint_values(rows, "tau_constraint", joint)

        if min(q) < lower[joint] - VALUE_TOL or max(q) > upper[joint] + VALUE_TOL:
            raise RuntimeError(f"joint {joint + 1} position exceeds configured limits")
        if max(abs(value) for value in qd) > velocity_limits[joint] + VALUE_TOL:
            raise RuntimeError(f"joint {joint + 1} velocity exceeds safety limit")
        if max(abs(value) for value in tau_cmd) > torque_limits[joint] + VALUE_TOL:
            raise RuntimeError(f"joint {joint + 1} torque command exceeds actuator limit")

        torque_error = [command - effort for command, effort in zip(tau_cmd, tau_effort)]
        tau_rmse.append(_rms(torque_error))
        tau_max_abs.append(max(abs(value) for value in torque_error))

        bound_violations: list[float] = []
        direction_violations: list[float] = []
        moving_samples = 0
        saturated_moving_samples = 0
        for velocity, constraint in zip(qd, tau_constraint):
            bound_violations.append(
                max(0.0, abs(constraint) - frictionloss[joint])
            )
            if abs(velocity) >= SLIDING_SPEED_THRESHOLD:
                moving_samples += 1
                force_along_motion = constraint * (1.0 if velocity > 0.0 else -1.0)
                direction_violations.append(max(0.0, force_along_motion))
                if abs(abs(constraint) - frictionloss[joint]) <= VALUE_TOL:
                    saturated_moving_samples += 1

        friction_bound_max_violation.append(max(bound_violations, default=0.0))
        sliding_direction_max_violation.append(
            max(direction_violations, default=0.0)
        )
        sliding_sample_counts.append(moving_samples)
        saturated_sliding_counts.append(saturated_moving_samples)

        q_stats.append((min(q), max(q)))
        qd_stats.append((min(qd), max(qd)))
        qdd_stats.append((min(qdd), max(qdd)))
        tau_stats.append((min(tau_cmd), max(tau_cmd)))
        coverage.append(
            (
                max(q) - min(q),
                _rms(qd),
                max(abs(value) for value in qd),
                _rms(qdd),
                max(abs(value) for value in qdd),
                _rms(tau_cmd),
                max(abs(value) for value in tau_cmd),
            )
        )

    quality_failures: list[str] = []
    if max(tau_max_abs) > VALUE_TOL:
        quality_failures.append(
            "tau_cmd and tau_effort are not identical within tolerance"
        )
    if max(friction_bound_max_violation) > VALUE_TOL:
        quality_failures.append("friction constraint exceeds configured frictionloss bound")
    if max(sliding_direction_max_violation) > VALUE_TOL:
        quality_failures.append("friction constraint assists motion in the sliding subset")
    if any(count == 0 for count in sliding_sample_counts):
        quality_failures.append("at least one joint has no sliding observations")
    if any(count == 0 for count in saturated_sliding_counts):
        quality_failures.append("at least one joint has no saturated sliding observations")

    trajectory_path = Path(metadata.get("trajectory_output_file", ""))
    if not trajectory_path.is_file():
        raise RuntimeError(f"trajectory coefficient file missing: {trajectory_path}")
    coefficient_sha = _sha256(trajectory_path)
    if coefficient_sha != metadata.get("trajectory_sha256"):
        raise RuntimeError("trajectory coefficient SHA does not match metadata")
    dataset_sha = _sha256(csv_path)

    requested_scale = float(controller_config["trajectory_coefficient_scale"])
    accepted_scale = float(metadata.get("trajectory_coefficient_scale", math.nan))
    accepted_attempt = int(metadata.get("trajectory_accepted_attempt", 0))
    seed = int(metadata.get("trajectory_seed", -1))

    print(f"csv={csv_path}")
    print(f"sample_count={len(rows)} duration={duration:.12f} dt={args.expected_dt:.12f}")
    print(
        f"seed={seed} requested_scale={requested_scale:.12g} "
        f"accepted_scale={accepted_scale:.12g} accepted_attempt={accepted_attempt}"
    )
    print(f"coefficient_sha256={coefficient_sha}")
    print(f"dataset_sha256={dataset_sha}")
    print(
        f"saturation_count={saturated_count} "
        f"unexpected_contact_count={unexpected_contact_count}"
    )
    print(f"tau_cmd_minus_tau_effort_rmse={tau_rmse}")
    print(f"tau_cmd_minus_tau_effort_max_abs={tau_max_abs}")
    print(f"global_tau_max_abs={max(tau_max_abs):.12e}")
    print(f"friction_bound_max_violation={friction_bound_max_violation}")
    print(f"sliding_direction_max_violation={sliding_direction_max_violation}")
    print(f"sliding_sample_counts={sliding_sample_counts}")
    print(f"saturated_sliding_counts={saturated_sliding_counts}")

    for joint in range(DOF):
        q_span, qd_rms, qd_max, qdd_rms, qdd_max, tau_rms_value, tau_max = coverage[joint]
        moving_fraction = sliding_sample_counts[joint] / len(rows)
        saturated_moving_fraction = saturated_sliding_counts[joint] / len(rows)
        print(
            f"J{joint + 1} "
            f"q_min={q_stats[joint][0]:.12g} q_max={q_stats[joint][1]:.12g} q_span={q_span:.12g} "
            f"qd_min={qd_stats[joint][0]:.12g} qd_max={qd_stats[joint][1]:.12g} "
            f"qd_rms={qd_rms:.12g} qd_max_abs={qd_max:.12g} "
            f"qdd_min={qdd_stats[joint][0]:.12g} qdd_max={qdd_stats[joint][1]:.12g} "
            f"qdd_rms={qdd_rms:.12g} qdd_max_abs={qdd_max:.12g} "
            f"tau_min={tau_stats[joint][0]:.12g} tau_max={tau_stats[joint][1]:.12g} "
            f"tau_rms={tau_rms_value:.12g} tau_max_abs={tau_max:.12g} "
            f"sliding_fraction={moving_fraction:.12g} "
            f"saturated_sliding_fraction={saturated_moving_fraction:.12g}"
        )

    if quality_failures:
        for failure in quality_failures:
            print(f"[FAIL] {failure}")
        print("[FAIL] reBot-DM excitation data-quality gate")
        return 1

    print("[PASS] reBot-DM excitation data-quality gate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
