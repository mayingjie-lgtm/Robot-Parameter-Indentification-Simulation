#!/usr/bin/env python3
"""Verify Phase 4B reBot-DM runtime and simulation-data semantics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List

import yaml


DOF = 6
REQUIRED_COLUMNS = (
    ["time_begin", "time_end"]
    + [f"q{i}" for i in range(DOF)]
    + [f"qd{i}" for i in range(DOF)]
    + [f"qdd_mujoco{i}" for i in range(DOF)]
    + [f"qdd_diff{i}" for i in range(DOF)]
    + [f"tau_cmd{i}" for i in range(DOF)]
    + [f"tau_effort{i}" for i in range(DOF)]
    + [f"tau_constraint{i}" for i in range(DOF)]
    + [f"q_next{i}" for i in range(DOF)]
    + [f"qd_next{i}" for i in range(DOF)]
    + ["saturated", "contact_count"]
)
EXPECTED_SOURCES = {
    "q_source": "mujoco_qpos_pre_integration",
    "qd_source": "mujoco_qvel_pre_integration",
    "qdd_mujoco_source": "mujoco_qacc_pre_integration",
    "qdd_diff_source": "forward_velocity_difference",
    "tau_cmd_source": "control_command_torque",
    "tau_effort_source": "mujoco_qfrc_actuator",
    "tau_constraint_source": "mujoco_qfrc_constraint",
}


def load_flat_config(path: Path) -> Dict[str, object]:
    """Load the repository's flat YAML controller configuration."""
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid YAML mapping: {path}")
    return data


def require_vector(data: Dict[str, object], key: str, size: int) -> List[float]:
    """Return one finite numeric YAML vector with an exact expected size."""
    value = data.get(key)
    if not isinstance(value, list) or len(value) != size:
        raise RuntimeError(f"{key} must contain exactly {size} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f"{key} contains non-finite values")
    return result


def parse_args() -> argparse.Namespace:
    """Parse Phase 4B verifier command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument(
        "--controller-config",
        type=Path,
        default=Path("config/rebot_dm_force_controller_node.yaml"),
    )
    return parser.parse_args()


def main() -> int:
    """Run all Phase 4B runtime/data gates and print quantitative diagnostics."""
    args = parse_args()
    csv_path = args.csv
    meta_path = csv_path.with_suffix(".meta.yaml")
    if not csv_path.is_file():
        raise RuntimeError(f"CSV does not exist: {csv_path}")
    if not meta_path.is_file():
        raise RuntimeError(f"metadata does not exist: {meta_path}")

    controller = load_flat_config(args.controller_config)
    lower = require_vector(controller, "joint_lower_limits", DOF)
    upper = require_vector(controller, "joint_upper_limits", DOF)
    velocity_limits = require_vector(controller, "joint_velocity_safety_limits", DOF)
    torque_limits = require_vector(controller, "joint_torque_limits", DOF)

    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        headers = reader.fieldnames or []
        missing = [name for name in REQUIRED_COLUMNS if name not in headers]
        if missing:
            raise RuntimeError(f"missing required CSV headers: {missing}")
        rows = list(reader)
    if not rows:
        raise RuntimeError("smoke CSV contains no samples")

    numeric_columns = list(REQUIRED_COLUMNS)
    values: Dict[str, List[float]] = {name: [] for name in numeric_columns}
    for row_index, row in enumerate(rows):
        for name in numeric_columns:
            try:
                value = float(row[name])
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"row {row_index} invalid {name}") from exc
            if not math.isfinite(value):
                raise RuntimeError(f"row {row_index} non-finite {name}")
            values[name].append(value)

    for index in range(1, len(rows)):
        if values["time_begin"][index] <= values["time_begin"][index - 1]:
            raise RuntimeError("time_begin is not strictly monotonic")
    for begin, end in zip(values["time_begin"], values["time_end"]):
        if end <= begin:
            raise RuntimeError("time_end must be greater than time_begin")

    q_min: List[float] = []
    q_max: List[float] = []
    qd_min: List[float] = []
    qd_max: List[float] = []
    qdd_min: List[float] = []
    qdd_max: List[float] = []
    tau_min: List[float] = []
    tau_max: List[float] = []
    per_joint_rmse: List[float] = []
    per_joint_max_abs: List[float] = []
    all_tau_errors: List[float] = []
    max_constraint = 0.0

    for joint in range(DOF):
        q = values[f"q{joint}"]
        qd = values[f"qd{joint}"]
        qdd = values[f"qdd_mujoco{joint}"]
        tau_cmd = values[f"tau_cmd{joint}"]
        tau_effort = values[f"tau_effort{joint}"]
        constraint = values[f"tau_constraint{joint}"]

        if min(q) < lower[joint] - 1e-12 or max(q) > upper[joint] + 1e-12:
            raise RuntimeError(f"J{joint + 1} q exceeded configured limits")
        if max(abs(item) for item in qd) > velocity_limits[joint] + 1e-12:
            raise RuntimeError(f"J{joint + 1} qd exceeded smoke safety limit")
        if max(abs(item) for item in tau_cmd) > torque_limits[joint] + 1e-12:
            raise RuntimeError(f"J{joint + 1} tau_cmd exceeded torque limit")

        errors = [command - effort for command, effort in zip(tau_cmd, tau_effort)]
        all_tau_errors.extend(errors)
        per_joint_rmse.append(math.sqrt(sum(item * item for item in errors) / len(errors)))
        per_joint_max_abs.append(max(abs(item) for item in errors))
        max_constraint = max(max_constraint, max(abs(item) for item in constraint))
        q_min.append(min(q))
        q_max.append(max(q))
        qd_min.append(min(qd))
        qd_max.append(max(qd))
        qdd_min.append(min(qdd))
        qdd_max.append(max(qdd))
        tau_min.append(min(tau_cmd))
        tau_max.append(max(tau_cmd))

    aggregate_rmse = math.sqrt(
        sum(item * item for item in all_tau_errors) / len(all_tau_errors)
    )
    global_max_abs = max(abs(item) for item in all_tau_errors)
    if global_max_abs > 1e-12:
        raise RuntimeError(f"tau_cmd != tau_effort, max abs={global_max_abs}")
    if any(int(value) != 0 for value in values["saturated"]):
        raise RuntimeError("smoke contains actuator saturation")
    if any(int(value) != 0 for value in values["contact_count"]):
        raise RuntimeError("smoke contains unexpected MuJoCo contacts")
    if max_constraint > 1e-10:
        raise RuntimeError(
            f"J1-J6 qfrc_constraint is not near zero: max={max_constraint}"
        )

    metadata = load_flat_config(meta_path)
    if metadata.get("robot") != "rebot_dm" or metadata.get("backend") != "sim":
        raise RuntimeError("metadata robot/backend mismatch")
    if metadata.get("controller_mode") != "hold_position":
        raise RuntimeError("Phase 4B smoke must use hold_position")
    if metadata.get("git_commit") in (None, "", "unknown"):
        raise RuntimeError("metadata must record a git commit")
    if require_vector(metadata, "gripper_lock_position", 2) != [0.05, 0.05]:
        raise RuntimeError("metadata gripper lock is not [0.05, 0.05]")
    if require_vector(metadata, "armature_truth", DOF) != [0.0] * DOF:
        raise RuntimeError("metadata armature truth changed")
    if require_vector(metadata, "damping_truth", DOF) != [0.0] * DOF:
        raise RuntimeError("metadata damping truth changed")
    if require_vector(metadata, "joint_frictionloss", DOF) != [0.0] * DOF:
        raise RuntimeError("metadata friction truth changed")
    for key, expected in EXPECTED_SOURCES.items():
        if metadata.get(key) != expected:
            raise RuntimeError(f"metadata {key} mismatch: {metadata.get(key)!r}")

    print(f"rows={len(rows)}")
    print(
        f"duration={values['time_end'][-1] - values['time_begin'][0]:.9f} s "
        f"dt={values['time_end'][0] - values['time_begin'][0]:.9f} s"
    )
    print(f"q_min={q_min}")
    print(f"q_max={q_max}")
    print(f"qd_min={qd_min}")
    print(f"qd_max={qd_max}")
    print(f"qdd_mujoco_min={qdd_min}")
    print(f"qdd_mujoco_max={qdd_max}")
    print(f"tau_cmd_min={tau_min}")
    print(f"tau_cmd_max={tau_max}")
    print(f"tau_cmd_minus_tau_effort_rmse={per_joint_rmse}")
    print(f"tau_cmd_minus_tau_effort_max_abs={per_joint_max_abs}")
    print(f"aggregate_tau_rmse={aggregate_rmse:.12e}")
    print(f"global_tau_max_abs={global_max_abs:.12e}")
    print(f"max_abs_J1_J6_constraint={max_constraint:.12e}")
    print("saturation_count=0")
    print("unexpected_contact_count=0")
    print("[PASS] reBot-DM Phase 4B runtime smoke semantics")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
