from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Sequence

import yaml

from .recorder import repository_git_commit
from .sdk_adapter import detect_sdk_version
from .state_capture import CaptureSample, JOINT_COUNT


SCHEMA_VERSION = "rebot_hardware_experiment_v1"

CSV_COLUMNS = (
    "sample_index",
    "timestamp_host_rx_ns",
    "timestamp_lower_ns",
    "timestamp_host_command_ns",
    "servo_sequence",
    *(f"q{joint}" for joint in range(JOINT_COUNT)),
    *(f"qd{joint}" for joint in range(JOINT_COUNT)),
    *(f"effort_reported{joint}" for joint in range(JOINT_COUNT)),
    *(f"q_cmd{joint}" for joint in range(JOINT_COUNT)),
    *(f"feedback_valid{joint}" for joint in range(JOINT_COUNT)),
    *(f"torque_valid{joint}" for joint in range(JOINT_COUNT)),
    *(f"feedback_age_ms{joint}" for joint in range(JOINT_COUNT)),
    "robot_mode",
    "safety_state",
    "primary_fault_code",
    "servo_active",
    "servo_mode",
    "command_valid",
    "control_mode",
)


class HardwareExperimentRecorder:
    """Stream explicit reBot hardware experiment samples and metadata to disk."""

    def __init__(
        self,
        csv_path: str | Path,
        *,
        config: dict[str, Any],
        repo_root: str | Path,
        backend: str,
    ) -> None:
        self.csv_path = Path(csv_path)
        self.metadata_path = self.csv_path.with_suffix(".meta.yaml")
        self.config = dict(config)
        self.repo_root = Path(repo_root)
        self.backend = str(backend)
        self.sample_count = 0
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.csv_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._closed = False

    def record(
        self,
        sample: CaptureSample,
        *,
        q_cmd: Sequence[float] | None,
        timestamp_host_command_ns: int | None,
        servo_sequence: int | None,
        command_valid: bool,
        control_mode: str,
    ) -> None:
        """Write one state sample plus the exact position-command contract fields."""

        if self._closed:
            raise RuntimeError("hardware recorder is closed")
        command = _optional_six(q_cmd, "q_cmd")
        row: dict[str, Any] = {
            "sample_index": self.sample_count,
            "timestamp_host_rx_ns": sample.timestamp_host_rx_ns,
            "timestamp_lower_ns": sample.timestamp_lower_ns,
            "timestamp_host_command_ns": (
                "" if timestamp_host_command_ns is None else int(timestamp_host_command_ns)
            ),
            "servo_sequence": "" if servo_sequence is None else int(servo_sequence),
            "robot_mode": sample.robot_mode,
            "safety_state": sample.safety_state,
            "primary_fault_code": sample.primary_fault_code,
            "servo_active": int(sample.servo_active),
            "servo_mode": sample.servo_mode,
            "command_valid": int(bool(command_valid)),
            "control_mode": str(control_mode),
        }
        for joint in range(JOINT_COUNT):
            row[f"q{joint}"] = sample.q[joint]
            row[f"qd{joint}"] = sample.qd[joint]
            row[f"effort_reported{joint}"] = sample.effort_reported[joint]
            row[f"q_cmd{joint}"] = math.nan if command is None else command[joint]
            row[f"feedback_valid{joint}"] = int(sample.feedback_valid[joint])
            row[f"torque_valid{joint}"] = int(sample.torque_valid[joint])
            row[f"feedback_age_ms{joint}"] = sample.feedback_age_ms[joint]
        self._writer.writerow(row)
        self.sample_count += 1

    def close(self) -> dict[str, Any]:
        """Close the CSV and write a metadata sidecar exactly once."""

        if not self._closed:
            self._stream.close()
            self._closed = True
            metadata = build_hardware_metadata(
                config=self.config,
                repo_root=self.repo_root,
                backend=self.backend,
                observed_sample_count=self.sample_count,
            )
            self.metadata_path.write_text(
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
                encoding="utf-8",
            )
            return metadata
        return yaml.safe_load(self.metadata_path.read_text(encoding="utf-8"))


def build_hardware_metadata(
    *,
    config: dict[str, Any],
    repo_root: str | Path,
    backend: str,
    observed_sample_count: int,
) -> dict[str, Any]:
    """Build provenance metadata without inventing unavailable physical signals."""

    sdk_root_text = str(config.get("sdk_root", "")).strip()
    sdk_root = Path(sdk_root_text).expanduser() if sdk_root_text else None
    sdk_version = (
        detect_sdk_version(sdk_root) if sdk_root is not None and sdk_root.exists() else "unknown"
    )
    safety_snapshot = _read_sdk_safety_snapshot(sdk_root)
    return {
        "schema_version": SCHEMA_VERSION,
        "robot": "rebot_dm",
        "backend": backend,
        "git_commit": repository_git_commit(repo_root),
        "sdk_root": sdk_root_text,
        "sdk_version": sdk_version,
        "host": str(config["host"]),
        "tcp_port": int(config["tcp_port"]),
        "udp_port": int(config["udp_port"]),
        "control_mode": str(config["control_mode"]),
        "control_rate_hz": float(config["control_rate_hz"]),
        "observed_sample_count": int(observed_sample_count),
        "allow_hardware": bool(config["allow_hardware"]),
        "allow_motion": bool(config["allow_motion"]),
        "joint_mapping_verified": bool(config["joint_mapping_verified"]),
        "j1_convention": str(config["j1_convention"]),
        "joint_direction": list(config["joint_direction"]),
        "joint_offset_rad": list(config["joint_offset_rad"]),
        "joint_position_min_rad": list(config["joint_position_min_rad"]),
        "joint_position_max_rad": list(config["joint_position_max_rad"]),
        "maximum_command_velocity_rad_s": list(config["maximum_command_velocity_rad_s"]),
        "maximum_feedback_age_ms": float(config["maximum_feedback_age_ms"]),
        "connect_timeout_s": float(config["connect_timeout_s"]),
        "command_timeout_s": float(config["command_timeout_s"]),
        "state_timeout_s": float(config["state_timeout_s"]),
        "timestamp_lower_source": (
            "SDK JointState.monotonic_time_ns; lower-host steady-clock timestamp for latest "
            "valid driver feedback; not device hardware time"
        ),
        "timestamp_host_rx_source": (
            "SDK StateStore ActualJointState.received_monotonic_ns on the upper host; "
            "recorded when the decoded UDP state is published to StateStore"
        ),
        "timestamp_host_command_source": (
            "upper-host time.monotonic_ns passed explicitly to ArmClient.servo_joint"
        ),
        "q_source": (
            "SDK public JointState.position_rad mapped as q=joint_direction*q_sdk+joint_offset_rad"
        ),
        "qd_source": "SDK public JointState.velocity_rad_s mapped by joint_direction",
        "effort_reported_source": (
            "SDK public JointState.torque_nm joint-side DM feedback estimate mapped by joint_direction"
        ),
        "q_cmd_source": (
            "runner-coordinate position target; adapter inversely maps it before "
            "ArmClient.servo_joint(target_position_rad)"
        ),
        "servo_command_fields": [
            "servo_sequence",
            "host_timestamp_ns",
            "target_position_rad[6]",
        ],
        "hardware_timestamp_available": False,
        "q_raw_available": False,
        "current_available": False,
        "tau_cmd_available": False,
        "feedback_sequence_supported": False,
        "trajectory_source": str(config.get("trajectory_source", "none")),
        "trajectory_hash": config.get("trajectory_hash"),
        "upper_safety_gate": {
            "six_dof_validation": True,
            "finite_check": True,
            "mapping_verified_gate": True,
            "command_position_limit": True,
            "velocity_derived_delta_limit": True,
            "feedback_validity": True,
            "feedback_age_limit_ms": float(config["maximum_feedback_age_ms"]),
            "primary_fault_gate": True,
            "servo_state_gate": True,
            "communication_timeout_s": float(config["state_timeout_s"]),
        },
        "sdk_safety_config_snapshot": safety_snapshot,
        "hardware_acceptance": {
            "joint_mapping_hardware_verification": "PENDING",
            "j1_convention": "UNRESOLVED" if not bool(config["joint_mapping_verified"]) else str(config["j1_convention"]),
            "hardware_state_only_acceptance": "PENDING",
            "servo_hold_hardware_acceptance": "PENDING",
            "excitation_hardware_acceptance": "PENDING",
        },
    }


def _read_sdk_safety_snapshot(sdk_root: Path | None) -> dict[str, Any] | None:
    if sdk_root is None:
        return None
    path = sdk_root / "config" / "safety.json"
    if not path.is_file():
        return None
    try:
        parsed = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    keys = (
        "profile",
        "robot_model_id",
        "joint_position_min_rad",
        "joint_position_max_rad",
        "command_velocity_limits_rad_s",
        "command_acceleration_limits_rad_s2",
        "feedback_timeout_ms",
        "communication_failure_grace_ms",
        "measured_position_monitor_enabled",
        "torque_monitor_enabled",
        "hot_reload_allowed",
    )
    return {key: parsed.get(key) for key in keys}


def _optional_six(values: Sequence[float] | None, name: str) -> tuple[float, ...] | None:
    if values is None:
        return None
    parsed = tuple(float(value) for value in values)
    if len(parsed) != JOINT_COUNT:
        raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain finite values")
    return parsed
