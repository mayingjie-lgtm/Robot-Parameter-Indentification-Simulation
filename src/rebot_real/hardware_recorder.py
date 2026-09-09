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


SCHEMA_VERSION = "rebot_hardware_experiment_v2"

CSV_COLUMNS = (
    "sample_index",
    "timestamp_host_rx_ns",
    "timestamp_lower_ns",
    "timestamp_host_command_ns",
    "actual_dispatch_timestamp_ns",
    "actual_dispatch_interval_ns",
    "reference_dispatch_skew_ns",
    "state_snapshot_age_ms",
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
        self.run_result: dict[str, Any] | None = None
        self.preposition_result: dict[str, Any] | None = None
        self.failure_result: dict[str, Any] | None = None
        self.shutdown_result: dict[str, Any] | None = None
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
        actual_dispatch_timestamp_ns: int | None = None,
        actual_dispatch_interval_ns: int | None = None,
        reference_dispatch_skew_ns: int | None = None,
        state_snapshot_age_ms: float | None = None,
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
            "actual_dispatch_timestamp_ns": (
                ""
                if actual_dispatch_timestamp_ns is None
                else int(actual_dispatch_timestamp_ns)
            ),
            "actual_dispatch_interval_ns": (
                "" if actual_dispatch_interval_ns is None else int(actual_dispatch_interval_ns)
            ),
            "reference_dispatch_skew_ns": (
                "" if reference_dispatch_skew_ns is None else int(reference_dispatch_skew_ns)
            ),
            "state_snapshot_age_ms": (
                "" if state_snapshot_age_ms is None else float(state_snapshot_age_ms)
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
            if self.run_result is not None:
                metadata["joint_jog_result"] = self.run_result
                metadata["joint_jog_config"] = self.config["joint_jog"]
                metadata["joint_mapping_scope"] = self.config["joint_mapping_scope"]
            if self.preposition_result is not None:
                metadata["preposition"] = self.preposition_result
            if self.failure_result is not None:
                metadata["failure"] = self.failure_result
            if self.shutdown_result is not None:
                metadata["shutdown"] = self.shutdown_result
            timing = self._build_dispatch_timing_metadata()
            if timing is not None:
                metadata["dispatch_timing"] = timing
            self.metadata_path.write_text(
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
                encoding="utf-8",
            )
            return metadata
        return yaml.safe_load(self.metadata_path.read_text(encoding="utf-8"))

    def _build_dispatch_timing_metadata(self) -> dict[str, Any] | None:
        """Summarize the recorded fixed-reference and actual dispatch clocks."""

        if not self.csv_path.is_file():
            return None
        with self.csv_path.open("r", encoding="utf-8", newline="") as stream:
            rows = [
                row
                for row in csv.DictReader(stream)
                if row["control_mode"] == "excitation" and row["command_valid"] == "1"
            ]
        if not rows:
            return None
        references = [int(row["timestamp_host_command_ns"]) for row in rows]
        dispatches = [int(row["actual_dispatch_timestamp_ns"]) for row in rows]
        reference_intervals = [
            current - previous for previous, current in zip(references, references[1:])
        ]
        dispatch_intervals = [int(row["actual_dispatch_interval_ns"]) for row in rows]
        skews = [int(row["reference_dispatch_skew_ns"]) for row in rows]
        period_ns = int(round(1e9 / float(self.config["control_rate_hz"])))
        below_period_count = sum(value < period_ns for value in dispatch_intervals)
        return {
            "reference_timestamp_start_ns": references[0],
            "reference_timestamp_end_ns": references[-1],
            "reference_interval_min_ns": (
                period_ns if not reference_intervals else min(reference_intervals)
            ),
            "reference_interval_max_ns": (
                period_ns if not reference_intervals else max(reference_intervals)
            ),
            "actual_dispatch_start_ns": dispatches[0],
            "actual_dispatch_end_ns": dispatches[-1],
            "actual_dispatch_interval_min_ns": min(dispatch_intervals),
            "actual_dispatch_interval_max_ns": max(dispatch_intervals),
            "reference_dispatch_skew_min_ns": min(skews),
            "reference_dispatch_skew_max_ns": max(skews),
            "interval_below_nominal_count": below_period_count,
            "catch_up_burst_count": below_period_count,
            "nominal_period_ns": period_ns,
        }


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
        "joint_mapping_scope": str(config.get("joint_mapping_scope", "servo_hold_only")),
        "j1_convention": str(config["j1_convention"]),
        "joint_direction": list(config["joint_direction"]),
        "joint_offset_rad": list(config["joint_offset_rad"]),
        "joint_position_min_rad": list(config["joint_position_min_rad"]),
        "joint_position_max_rad": list(config["joint_position_max_rad"]),
        "maximum_command_velocity_rad_s": list(config["maximum_command_velocity_rad_s"]),
        "maximum_tracking_error_rad": list(config["maximum_tracking_error_rad"]),
        "maximum_servo_target_delta_rad": float(
            config["maximum_servo_target_delta_rad"]
        ),
        "movej_max_velocity_rad_s": list(config["movej_max_velocity_rad_s"]),
        "movej_max_acceleration_rad_s2": list(config["movej_max_acceleration_rad_s2"]),
        "movej_max_jerk_rad_s3": list(config["movej_max_jerk_rad_s3"]),
        "movej_timeout_s": float(config["movej_timeout_s"]),
        "controlled_park_before_disable": bool(
            config.get("controlled_park_before_disable", False)
        ),
        "start_position_tolerance_rad": float(config["start_position_tolerance_rad"]),
        "preposition_settle_velocity_tolerance_rad_s": float(
            config["preposition_settle_velocity_tolerance_rad_s"]
        ),
        "maximum_feedback_age_ms": float(config["maximum_feedback_age_ms"]),
        "maximum_disabled_feedback_age_ms": float(
            config["maximum_disabled_feedback_age_ms"]
        ),
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
            "host timestamp passed explicitly to ArmClient.servo_joint; excitation uses a "
            "fixed-cadence reference grid anchored at the initial Servo hold, while other "
            "modes use the current upper-host time.monotonic_ns"
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
            "servo_target_delta_limit_rad": float(
                config["maximum_servo_target_delta_rad"]
            ),
            "tracking_error_limit_rad": list(config["maximum_tracking_error_rad"]),
            "tracking_error_divided_by_control_rate": False,
            "feedback_validity": True,
            "disabled_feedback_age_limit_ms": float(
                config["maximum_disabled_feedback_age_ms"]
            ),
            "enabled_feedback_age_limit_ms": float(config["maximum_feedback_age_ms"]),
            "primary_fault_gate": True,
            "servo_state_gate": True,
            "preposition_fresh_feedback_gate": True,
            "preposition_start_position_gate": True,
            "preposition_settle_velocity_gate": True,
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
        "measured_position_tolerance_rad",
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
