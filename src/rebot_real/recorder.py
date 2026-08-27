from __future__ import annotations

import csv
import math
from pathlib import Path
import subprocess
import time
from typing import Any, Iterable

import yaml

from .sdk_adapter import detect_sdk_version
from .state_capture import CaptureSample, JOINT_COUNT, map_sdk_state


SCHEMA_VERSION = "rebot_hardware_state_v1"
CANONICAL_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, JOINT_COUNT + 1))
SDK_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, JOINT_COUNT + 1))

CSV_COLUMNS = (
    "sample_index",
    "timestamp_host_rx_ns",
    "timestamp_lower_ns",
    "udp_sequence",
    "udp_sequence_gap",
    *(f"q{joint}" for joint in range(JOINT_COUNT)),
    *(f"qd{joint}" for joint in range(JOINT_COUNT)),
    *(f"effort_reported{joint}" for joint in range(JOINT_COUNT)),
    *(f"feedback_valid{joint}" for joint in range(JOINT_COUNT)),
    *(f"torque_valid{joint}" for joint in range(JOINT_COUNT)),
    *(f"feedback_age_ms{joint}" for joint in range(JOINT_COUNT)),
    "robot_mode",
    "safety_state",
    "primary_fault_code",
    "servo_active",
    "servo_mode",
)


def sample_to_row(sample: CaptureSample) -> dict[str, Any]:
    row: dict[str, Any] = {
        "sample_index": sample.sample_index,
        "timestamp_host_rx_ns": sample.timestamp_host_rx_ns,
        "timestamp_lower_ns": sample.timestamp_lower_ns,
        "udp_sequence": sample.udp_sequence,
        "udp_sequence_gap": sample.udp_sequence_gap,
        "robot_mode": sample.robot_mode,
        "safety_state": sample.safety_state,
        "primary_fault_code": sample.primary_fault_code,
        "servo_active": int(sample.servo_active),
        "servo_mode": sample.servo_mode,
    }
    for joint in range(JOINT_COUNT):
        row[f"q{joint}"] = sample.q[joint]
        row[f"qd{joint}"] = sample.qd[joint]
        row[f"effort_reported{joint}"] = sample.effort_reported[joint]
        row[f"feedback_valid{joint}"] = int(sample.feedback_valid[joint])
        row[f"torque_valid{joint}"] = int(sample.torque_valid[joint])
        row[f"feedback_age_ms{joint}"] = sample.feedback_age_ms[joint]
    return row


def repository_git_commit(repo_root: str | Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=Path(repo_root),
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def build_metadata(
    *,
    repo_root: str | Path,
    sdk_root: str | Path,
    requested_duration_s: float,
    observed_sample_count: int,
    git_commit: str | None = None,
    sdk_version: str | None = None,
) -> dict[str, Any]:
    sdk_path = Path(sdk_root).expanduser().resolve()
    return {
        "schema_version": SCHEMA_VERSION,
        "robot": "rebot_dm",
        "backend": "rebot_sdk_state_only",
        "git_commit": git_commit or repository_git_commit(repo_root),
        "sdk_root": str(sdk_path),
        "sdk_version": sdk_version or detect_sdk_version(sdk_path),
        "capture_mode": "state_only",
        "requested_duration_s": float(requested_duration_s),
        "observed_sample_count": int(observed_sample_count),
        "timestamp_lower_source": (
            "SDK JointState.monotonic_time_ns; lower host std::chrono::steady_clock; "
            "latest valid driver feedback receive timestamp, not device hardware time"
        ),
        "timestamp_host_rx_source": "recorder host time.monotonic_ns immediately after UDP recvfrom",
        "q_source": "SDK public JointState.position_rad; joint-side calibrated feedback",
        "qd_source": "SDK public JointState.velocity_rad_s; joint-side calibrated feedback",
        "effort_reported_source": "SDK public JointState.torque_nm from DM feedback torque field",
        "effort_reported_semantics": (
            "joint-side DM torque estimate after SDK direction/gear mapping; "
            "not independently calibrated joint torque sensing"
        ),
        "hardware_timestamp_available": False,
        "q_raw_available": False,
        "current_available": False,
        "tau_cmd_available": False,
        "feedback_sequence_supported": False,
        "joint_mapping_status": "unverified",
        "sdk_joint_names": list(SDK_JOINT_NAMES),
        "canonical_joint_names": list(CANONICAL_JOINT_NAMES),
        "j1_convention": "UNRESOLVED",
        "state_only_transport": "UDP-only public-state subscriber; no TCP control session",
        "sdk_sequence_semantics": (
            "lower ArmController state sequence carried in each UDP datagram; "
            "increments at lower control-loop rate, not once per UDP publication"
        ),
        "udp_missing_snapshot_count_inferable": False,
        "invalid_measurement_policy": (
            "q/qd/feedback_age are NaN when feedback_valid=false; effort_reported is NaN "
            "when feedback_valid=false or torque_valid=false"
        ),
    }


def write_metadata(path: str | Path, metadata: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def capture_udp_state(
    subscriber: Any,
    *,
    duration_s: float,
    csv_path: str | Path,
    metadata_path: str | Path,
    repo_root: str | Path,
    sdk_root: str | Path,
) -> dict[str, Any]:
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive and finite")

    output = Path(csv_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    observed = 0
    previous_sequence: int | None = None
    deadline_ns = time.monotonic_ns() + int(duration_s * 1e9)

    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        while time.monotonic_ns() < deadline_ns:
            received = subscriber.receive()
            if received is None:
                continue
            timestamp_host_rx_ns, state = received
            sample = map_sdk_state(
                state,
                sample_index=observed,
                timestamp_host_rx_ns=timestamp_host_rx_ns,
                previous_sequence=previous_sequence,
            )
            writer.writerow(sample_to_row(sample))
            previous_sequence = sample.udp_sequence
            observed += 1

    metadata = build_metadata(
        repo_root=repo_root,
        sdk_root=sdk_root,
        requested_duration_s=duration_s,
        observed_sample_count=observed,
    )
    write_metadata(metadata_path, metadata)
    return metadata


def _parse_bool(value: str, name: str) -> bool:
    if value in {"1", "true", "True"}:
        return True
    if value in {"0", "false", "False"}:
        return False
    raise ValueError(f"{name} must be boolean/0/1, got {value!r}")


def _finite_stats(values: Iterable[float]) -> tuple[float | None, float | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None, None
    return min(finite), max(finite)


def _effort_stats(values: Iterable[float]) -> dict[str, float | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"min": None, "max": None, "mean": None, "std": None}
    mean = sum(finite) / len(finite)
    variance = sum((value - mean) ** 2 for value in finite) / len(finite)
    return {
        "min": min(finite),
        "max": max(finite),
        "mean": mean,
        "std": math.sqrt(variance),
    }


def analyze_capture(csv_path: str | Path) -> dict[str, Any]:
    path = Path(csv_path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(CSV_COLUMNS):
            raise ValueError("hardware state CSV schema does not match rebot_hardware_state_v1")
        rows = list(reader)

    host_times = [int(row["timestamp_host_rx_ns"]) for row in rows]
    lower_times = [int(row["timestamp_lower_ns"]) for row in rows]
    sequences = [int(row["udp_sequence"]) for row in rows]
    duration = (host_times[-1] - host_times[0]) * 1e-9 if len(rows) >= 2 else 0.0
    rate = (len(rows) - 1) / duration if len(rows) >= 2 and duration > 0.0 else 0.0
    deltas = [current - previous for previous, current in zip(sequences, sequences[1:])]

    summary: dict[str, Any] = {
        "sample_count": len(rows),
        "duration": duration,
        "observed_udp_rate_hz": rate,
        "timestamp_host_monotonic": all(
            current > previous for previous, current in zip(host_times, host_times[1:])
        ),
        "timestamp_lower_monotonic": all(
            current >= previous for previous, current in zip(lower_times, lower_times[1:])
        ),
        "udp_sequence_gap_count": sum(delta > 1 for delta in deltas),
        "udp_missing_snapshot_count": None,
        "udp_missing_snapshot_count_reason": (
            "not inferable: SDK sequence increments at lower control-loop rate, not UDP publish rate"
        ),
        "udp_duplicate_or_out_of_order_count": sum(delta <= 0 for delta in deltas),
        "non_finite_count": 0,
        "joints": {},
    }

    for joint in range(JOINT_COUNT):
        feedback_valid = [
            _parse_bool(row[f"feedback_valid{joint}"], f"feedback_valid{joint}")
            for row in rows
        ]
        torque_valid = [
            _parse_bool(row[f"torque_valid{joint}"], f"torque_valid{joint}")
            for row in rows
        ]
        q = [float(row[f"q{joint}"]) for row in rows]
        qd = [float(row[f"qd{joint}"]) for row in rows]
        effort = [float(row[f"effort_reported{joint}"]) for row in rows]
        age = [float(row[f"feedback_age_ms{joint}"]) for row in rows]

        non_finite = 0
        for index in range(len(rows)):
            if feedback_valid[index]:
                non_finite += int(not math.isfinite(q[index]))
                non_finite += int(not math.isfinite(qd[index]))
                non_finite += int(not math.isfinite(age[index]))
            if feedback_valid[index] and torque_valid[index]:
                non_finite += int(not math.isfinite(effort[index]))
        summary["non_finite_count"] += non_finite

        valid_q = [value for value, valid in zip(q, feedback_valid) if valid]
        valid_qd = [value for value, valid in zip(qd, feedback_valid) if valid]
        valid_age = [value for value, valid in zip(age, feedback_valid) if valid and math.isfinite(value)]
        valid_effort = [
            value
            for value, feedback_ok, torque_ok in zip(effort, feedback_valid, torque_valid)
            if feedback_ok and torque_ok
        ]
        q_min, q_max = _finite_stats(valid_q)
        qd_min, qd_max = _finite_stats(valid_qd)
        effort_stats = _effort_stats(valid_effort)
        summary["joints"][f"J{joint + 1}"] = {
            "feedback_invalid_count": sum(not valid for valid in feedback_valid),
            "torque_invalid_count": sum(not valid for valid in torque_valid),
            "mean_feedback_age_ms": (
                sum(valid_age) / len(valid_age) if valid_age else None
            ),
            "max_feedback_age_ms": max(valid_age) if valid_age else None,
            "q_min": q_min,
            "q_max": q_max,
            "qd_min": qd_min,
            "qd_max": qd_max,
            "effort_reported_min": effort_stats["min"],
            "effort_reported_max": effort_stats["max"],
            "effort_reported_mean": effort_stats["mean"],
            "effort_reported_std": effort_stats["std"],
        }

    return summary
