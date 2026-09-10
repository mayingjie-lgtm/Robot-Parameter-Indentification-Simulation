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


SCHEMA_VERSION = "rebot_hardware_experiment_v3"

CSV_COLUMNS = (
    "sample_index",
    "timestamp_host_rx_ns",
    "timestamp_lower_ns",
    "timestamp_host_command_ns",
    "actual_dispatch_timestamp_ns",
    "actual_dispatch_interval_ns",
    "reference_dispatch_skew_ns",
    "state_snapshot_age_ms",
    "trajectory_time_s",
    "trajectory_interval_index",
    "trajectory_interval_ratio",
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
        self.runtime_envelope_result: dict[str, Any] | None = None
        self.motion_status = "unknown"
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
        trajectory_time_s: float | None = None,
        trajectory_interval_index: int | None = None,
        trajectory_interval_ratio: float | None = None,
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
            "trajectory_time_s": (
                "" if trajectory_time_s is None else float(trajectory_time_s)
            ),
            "trajectory_interval_index": (
                "" if trajectory_interval_index is None else int(trajectory_interval_index)
            ),
            "trajectory_interval_ratio": (
                "" if trajectory_interval_ratio is None else float(trajectory_interval_ratio)
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
            if self.runtime_envelope_result is not None:
                metadata["runtime_servo_envelope"] = self.runtime_envelope_result
            metadata["motion_status"] = self.motion_status
            timing = self._build_dispatch_timing_metadata()
            if timing is not None:
                metadata["dispatch_timing"] = timing
            tracking = self._build_tracking_quality_metadata()
            if tracking is not None:
                metadata["tracking_quality"] = tracking
                if self.motion_status != "completed":
                    quality_status = "not_accepted"
                    quality_reason = "motion_not_completed"
                elif tracking["status"] == "warning":
                    quality_status = "warning"
                    quality_reason = (
                        "tracking_warning_threshold_exceeded; offline identification "
                        "acceptance is still required"
                    )
                else:
                    quality_status = "not_accepted"
                    quality_reason = "offline_identification_acceptance_required"
                metadata["identification_data_quality"] = {
                    "status": quality_status,
                    "accepted": False,
                    "reason": quality_reason,
                    "tracking_quality_status": tracking["status"],
                }
            cadence = self._build_feedback_cadence_metadata()
            if cadence is not None:
                metadata["feedback_cadence"] = cadence
            self.metadata_path.write_text(
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
                encoding="utf-8",
            )
            return metadata
        return yaml.safe_load(self.metadata_path.read_text(encoding="utf-8"))

    def _build_dispatch_timing_metadata(self) -> dict[str, Any] | None:
        """Summarize actual-time replay and real dispatch timing."""

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
        command_timestamps = [int(row["timestamp_host_command_ns"]) for row in rows]
        dispatches = [int(row["actual_dispatch_timestamp_ns"]) for row in rows]
        dispatch_intervals = [int(row["actual_dispatch_interval_ns"]) for row in rows]
        skews = [int(row["reference_dispatch_skew_ns"]) for row in rows]
        trajectory_references = [
            dispatch - skew for dispatch, skew in zip(dispatches, skews)
        ]
        reference_intervals = [
            current - previous
            for previous, current in zip(trajectory_references, trajectory_references[1:])
        ]
        period_ns = int(round(1e9 / float(self.config["control_rate_hz"])))
        below_period_count = sum(value < period_ns for value in dispatch_intervals)
        late_cycle_count = sum(value > period_ns for value in dispatch_intervals)
        mean_interval_ns = sum(dispatch_intervals) / len(dispatch_intervals)
        return {
            "timestamp_semantics": (
                "timestamp_host_command_ns is the real upper-host monotonic dispatch "
                "timestamp passed to ArmClient.servo_joint"
            ),
            "replay_semantics": (
                "approved continuous quintic path resampled at real dispatch time; "
                "no catch-up burst"
            ),
            "trajectory_reference_timestamp_start_ns": trajectory_references[0],
            "trajectory_reference_timestamp_end_ns": trajectory_references[-1],
            "nominal_reference_interval_min_ns": (
                period_ns if not reference_intervals else min(reference_intervals)
            ),
            "nominal_reference_interval_max_ns": (
                period_ns if not reference_intervals else max(reference_intervals)
            ),
            "actual_dispatch_start_ns": dispatches[0],
            "actual_dispatch_end_ns": dispatches[-1],
            "actual_dispatch_interval_min_ns": min(dispatch_intervals),
            "actual_dispatch_interval_mean_ns": mean_interval_ns,
            "actual_dispatch_interval_p95_ns": _percentile(dispatch_intervals, 95.0),
            "actual_dispatch_interval_max_ns": max(dispatch_intervals),
            "effective_command_rate_hz": (
                1e9 / mean_interval_ns if mean_interval_ns > 0.0 else None
            ),
            "reference_dispatch_skew_min_ns": min(skews),
            "reference_dispatch_skew_max_ns": max(skews),
            "interval_below_nominal_count": below_period_count,
            "late_cycle_count": late_cycle_count,
            "catch_up_burst_count": below_period_count,
            "command_dispatch_timestamp_mismatch_count": sum(
                command != dispatch
                for command, dispatch in zip(command_timestamps, dispatches)
            ),
            "nominal_period_ns": period_ns,
            "nominal_rate_hz": float(self.config["control_rate_hz"]),
        }

    def _build_tracking_quality_metadata(self) -> dict[str, Any] | None:
        """Summarize excitation q_ref-q lag without turning it into an abort gate."""

        if self.config.get("control_mode") != "excitation" or not self.csv_path.is_file():
            return None
        with self.csv_path.open("r", encoding="utf-8", newline="") as stream:
            command_rows = [
                row
                for row in csv.DictReader(stream)
                if row["control_mode"] == "excitation" and row["command_valid"] == "1"
            ]
        if not command_rows:
            return None
        lower_timeout_ms = float(self.config["lower_feedback_timeout_ms"])
        rows = [
            row
            for row in command_rows
            if all(row[f"feedback_valid{joint}"] == "1" for joint in range(JOINT_COUNT))
            and all(
                float(row[f"feedback_age_ms{joint}"]) <= lower_timeout_ms
                for joint in range(JOINT_COUNT)
            )
        ]
        unusable_count = len(command_rows) - len(rows)
        if not rows:
            return {
                "semantics": (
                    "monitor_only_during_excitation; rows with invalid or Lower-timeout-stale "
                    "measurements are excluded from q_ref-q quality checks"
                ),
                "status": "no_usable_measurements",
                "command_row_count": len(command_rows),
                "measurement_usable_row_count": 0,
                "measurement_unusable_row_count": unusable_count,
                "lower_feedback_timeout_ms": lower_timeout_ms,
                "warning_threshold_rad": list(self.config["maximum_tracking_error_rad"]),
                "threshold_exceed_sample_count": 0,
                "per_joint": {},
            }

        thresholds = [float(value) for value in self.config["maximum_tracking_error_rad"]]
        def trajectory_time(row: dict[str, str]) -> float:
            value = row.get("trajectory_time_s", "")
            return (
                float(value)
                if value not in (None, "")
                else int(row["sample_index"]) / float(self.config["control_rate_hz"])
            )
        per_joint: dict[str, Any] = {}
        any_exceed_rows: list[int] = []
        overall = (-1.0, None, None)
        for joint in range(JOINT_COUNT):
            errors = [
                abs(float(row[f"q_cmd{joint}"]) - float(row[f"q{joint}"]))
                for row in rows
            ]
            exceed_indices = [
                index
                for index, error in enumerate(errors)
                if error > thresholds[joint] + 1e-12
            ]
            if exceed_indices:
                any_exceed_rows.extend(exceed_indices)
            max_index = max(range(len(errors)), key=errors.__getitem__)
            if errors[max_index] > overall[0]:
                overall = (errors[max_index], joint, max_index)
            first = exceed_indices[0] if exceed_indices else None
            per_joint[f"J{joint + 1}"] = {
                "max_abs_rad": max(errors),
                "p95_abs_rad": _percentile(errors, 95.0),
                "warning_threshold_rad": thresholds[joint],
                "threshold_exceed_count": len(exceed_indices),
                "first_threshold_exceed_sample_index": (
                    None if first is None else int(rows[first]["sample_index"])
                ),
                "first_threshold_exceed_artifact_time_s": (
                    None
                    if first is None
                    else trajectory_time(rows[first])
                ),
            }

        unique_exceed_rows = sorted(set(any_exceed_rows))
        first_any = unique_exceed_rows[0] if unique_exceed_rows else None
        overall_error, overall_joint, overall_index = overall
        return {
            "semantics": (
                "monitor_only_during_excitation; threshold exceedance is a "
                "control/identification-quality warning, not an immediate fail-safe; "
                "invalid or Lower-timeout-stale measurement rows are excluded"
            ),
            "status": "warning" if unique_exceed_rows else "within_warning_thresholds",
            "command_row_count": len(command_rows),
            "measurement_usable_row_count": len(rows),
            "measurement_unusable_row_count": unusable_count,
            "lower_feedback_timeout_ms": lower_timeout_ms,
            "warning_threshold_rad": thresholds,
            "threshold_exceed_sample_count": len(unique_exceed_rows),
            "first_threshold_exceed_sample_index": (
                None if first_any is None else int(rows[first_any]["sample_index"])
            ),
            "first_threshold_exceed_artifact_time_s": (
                None
                if first_any is None
                else trajectory_time(rows[first_any])
            ),
            "overall_max_abs_rad": overall_error,
            "overall_max_joint": (
                None if overall_joint is None else f"J{overall_joint + 1}"
            ),
            "overall_max_sample_index": (
                None
                if overall_index is None
                else int(rows[overall_index]["sample_index"])
            ),
            "per_joint": per_joint,
        }

    def _build_feedback_cadence_metadata(self) -> dict[str, Any] | None:
        """Separate raw row, UDP publication, lower feedback, and signal-update cadence."""

        if not self.csv_path.is_file():
            return None
        with self.csv_path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            return None

        host_times = [int(row["timestamp_host_rx_ns"]) for row in rows]
        lower_times = [int(row["timestamp_lower_ns"]) for row in rows]

        def event_times_for_key(key_fn: Any) -> list[int]:
            events: list[int] = []
            previous: Any = object()
            for row in rows:
                current = key_fn(row)
                if not events or current != previous:
                    events.append(int(row["timestamp_host_rx_ns"]))
                    previous = current
            return events

        lower_event_times = event_times_for_key(lambda row: int(row["timestamp_lower_ns"]))
        q_event_times = event_times_for_key(
            lambda row: tuple(float(row[f"q{joint}"]) for joint in range(JOINT_COUNT))
        )
        qd_event_times = event_times_for_key(
            lambda row: tuple(float(row[f"qd{joint}"]) for joint in range(JOINT_COUNT))
        )
        effort_event_times = event_times_for_key(
            lambda row: tuple(
                float(row[f"effort_reported{joint}"]) for joint in range(JOINT_COUNT)
            )
        )
        snapshot_keys = [
            (
                int(row["timestamp_lower_ns"]),
                tuple(float(row[f"q{joint}"]) for joint in range(JOINT_COUNT)),
                tuple(float(row[f"qd{joint}"]) for joint in range(JOINT_COUNT)),
                tuple(
                    float(row[f"effort_reported{joint}"])
                    for joint in range(JOINT_COUNT)
                ),
            )
            for row in rows
        ]
        ages = [
            float(row[f"feedback_age_ms{joint}"])
            for row in rows
            for joint in range(JOINT_COUNT)
            if math.isfinite(float(row[f"feedback_age_ms{joint}"]))
        ]
        unique_host_times = []
        for value in host_times:
            if not unique_host_times or value != unique_host_times[-1]:
                unique_host_times.append(value)
        duplicate_snapshots = sum(
            current == previous
            for previous, current in zip(snapshot_keys, snapshot_keys[1:])
        )
        return {
            "raw_rows": len(rows),
            "unique_timestamp_host_rx_ns": len(set(host_times)),
            "unique_timestamp_lower_ns": len(set(lower_times)),
            "timestamp_host_rx_cadence": _event_cadence(unique_host_times),
            "timestamp_lower_update_cadence_on_host": _event_cadence(lower_event_times),
            "q_update_cadence_on_host": _event_cadence(q_event_times),
            "qd_update_cadence_on_host": _event_cadence(qd_event_times),
            "effort_update_cadence_on_host": _event_cadence(effort_event_times),
            "duplicate_state_snapshot_count": duplicate_snapshots,
            "feedback_age_ms": {
                "min": None if not ages else min(ages),
                "mean": None if not ages else sum(ages) / len(ages),
                "p50": None if not ages else _percentile(ages, 50.0),
                "p95": None if not ages else _percentile(ages, 95.0),
                "max": None if not ages else max(ages),
            },
            "rate_semantics": (
                "command dispatch rate, CSV row rate, UDP host-receive publication cadence, "
                "and lower/signal feedback update cadence are distinct quantities; a 100 Hz "
                "CSV does not imply 100 Hz independent physical measurements"
            ),
        }


def _percentile(values: Sequence[float | int], percentile: float) -> float:
    parsed = sorted(float(value) for value in values)
    if not parsed:
        raise ValueError("percentile requires at least one value")
    position = (len(parsed) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return parsed[lower]
    fraction = position - lower
    return parsed[lower] * (1.0 - fraction) + parsed[upper] * fraction


def _event_cadence(event_host_times_ns: Sequence[int]) -> dict[str, Any]:
    times = [int(value) for value in event_host_times_ns]
    intervals = [
        current - previous
        for previous, current in zip(times, times[1:])
        if current > previous
    ]
    duration_ns = times[-1] - times[0] if len(times) >= 2 else 0
    effective_rate_hz = (
        (len(times) - 1) * 1e9 / duration_ns
        if len(times) >= 2 and duration_ns > 0
        else 0.0
    )
    return {
        "event_count": len(times),
        "effective_rate_hz": effective_rate_hz,
        "interval_ns_min": None if not intervals else min(intervals),
        "interval_ns_mean": (
            None if not intervals else sum(intervals) / len(intervals)
        ),
        "interval_ns_p95": None if not intervals else _percentile(intervals, 95.0),
        "interval_ns_max": None if not intervals else max(intervals),
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
        "maximum_command_acceleration_rad_s2": list(
            config["maximum_command_acceleration_rad_s2"]
        ),
        "maximum_command_jerk_rad_s3": list(
            config["maximum_command_jerk_rad_s3"]
        ),
        "maximum_tracking_error_rad": list(config["maximum_tracking_error_rad"]),
        "maximum_servo_target_delta_rad": float(
            config["maximum_servo_target_delta_rad"]
        ),
        "minimum_servo_timestamp_interval_s": float(
            config["minimum_servo_timestamp_interval_s"]
        ),
        "maximum_servo_timestamp_interval_s": float(
            config["maximum_servo_timestamp_interval_s"]
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
        "maximum_feedback_age_ms_semantics": (
            "deprecated compatibility alias for motion_ready_feedback_max_age_ms; "
            "not an excitation runtime abort threshold"
        ),
        "motion_ready_feedback_max_age_ms": float(
            config["motion_ready_feedback_max_age_ms"]
        ),
        "maximum_disabled_feedback_age_ms": float(
            config["maximum_disabled_feedback_age_ms"]
        ),
        "lower_feedback_timeout_ms": float(config["lower_feedback_timeout_ms"]),
        "transient_feedback_invalid_recovery_ms": float(
            config["transient_feedback_invalid_recovery_ms"]
        ),
        "connect_timeout_s": float(config["connect_timeout_s"]),
        "command_timeout_s": float(config["command_timeout_s"]),
        "state_timeout_s": float(config["state_timeout_s"]),
        "host_state_snapshot_timeout_s": float(
            config["host_state_snapshot_timeout_s"]
        ),
        "timestamp_lower_source": (
            "SDK JointState.monotonic_time_ns; lower-host steady-clock timestamp for latest "
            "valid driver feedback; not device hardware time"
        ),
        "timestamp_host_rx_source": (
            "SDK StateStore ActualJointState.received_monotonic_ns on the upper host; "
            "recorded when the decoded UDP state is published to StateStore"
        ),
        "timestamp_host_command_source": (
            "actual upper-host time.monotonic_ns at Servo dispatch; this exact timestamp "
            "is passed explicitly to ArmClient.servo_joint"
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
        "trajectory_replay_mode": config.get("trajectory_replay_mode"),
        "upper_safety_gate": {
            "six_dof_validation": True,
            "finite_check": True,
            "mapping_verified_gate": True,
            "command_position_limit": True,
            "velocity_derived_delta_limit": True,
            "actual_timestamp_velocity_gate": True,
            "actual_timestamp_acceleration_gate": True,
            "actual_timestamp_jerk_gate": True,
            "commit_envelope_history_after_sdk_acceptance": True,
            "servo_target_delta_limit_rad": float(
                config["maximum_servo_target_delta_rad"]
            ),
            "tracking_error_threshold_rad": list(config["maximum_tracking_error_rad"]),
            "tracking_error_semantics": (
                "monitor_only_quality_warning"
                if str(config["control_mode"]) == "excitation"
                else "runtime_gate"
            ),
            "tracking_error_immediate_fail_safe": (
                str(config["control_mode"]) != "excitation"
            ),
            "tracking_error_divided_by_control_rate": False,
            "feedback_validity": True,
            "disabled_feedback_age_limit_ms": float(
                config["maximum_disabled_feedback_age_ms"]
            ),
            "motion_ready_feedback_age_limit_ms": float(
                config["motion_ready_feedback_max_age_ms"]
            ),
            "legacy_maximum_feedback_age_ms": {
                "value": float(config["maximum_feedback_age_ms"]),
                "semantics": (
                    "deprecated compatibility alias for pre-motion freshness; "
                    "not used as the excitation runtime abort threshold"
                ),
            },
            "lower_feedback_timeout_ms": float(config["lower_feedback_timeout_ms"]),
            "lower_feedback_freshness_semantics": (
                "measurements are usable only while feedback_valid is true and age is "
                "within this audited Lower timeout"
            ),
            "transient_feedback_invalid_recovery_ms": float(
                config["transient_feedback_invalid_recovery_ms"]
            ),
            "transient_feedback_invalid_semantics": (
                "during excitation only, fresh-host snapshots with no primary/safety/Servo "
                "fault may keep the command stream alive inside the audited recovery window; "
                "unusable measurements remain flagged and are excluded from quality checks"
            ),
            "host_state_snapshot_timeout_s": float(
                config["host_state_snapshot_timeout_s"]
            ),
            "host_state_snapshot_semantics": (
                "upper-host timestamp_host_rx_ns age; expiration is a true state-stream "
                "communication failure and immediate fail-safe"
            ),
            "primary_fault_semantics": "immediate_fail_safe",
            "servo_reject_semantics": "immediate_fail_safe",
            "servo_state_gate": True,
            "controlled_park_policy": (
                "required_explicit_true_for_real_excitation; recoverable software-side "
                "abort parks only after fresh healthy enabled feedback is reacquired"
            ),
            "preposition_fresh_feedback_gate": True,
            "preposition_start_position_gate": True,
            "preposition_settle_velocity_gate": True,
            "blocking_state_read_timeout_s": float(config["state_timeout_s"]),
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
