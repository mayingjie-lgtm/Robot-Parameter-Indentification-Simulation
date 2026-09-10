#!/usr/bin/env python3
"""Run one approved reBot real A/B experiment and its offline identification pipeline."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Callable

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebot_real.mock_client import MockArmClient
from rebot_real.recorder import repository_git_commit
from rebot_real.runner import RebotHardwareRunner
from rebot_real.trajectory_artifact import (
    TRAJECTORY_REPLAY_MODE,
    ReplayArtifact,
    load_replay_artifact,
    sha256_file,
    validate_preview_acceptance,
    validate_replay_runtime_limits,
)
from run_rebot_identification import run_pipeline

SCHEMA_VERSION = "rebot_real_ab_v1"
ROOT_KEYS = {
    "schema_version", "experiment", "connection", "authorization", "mapping",
    "safety", "feedback", "movej", "trajectories", "identification",
}
SECTION_KEYS = {
    "experiment": {"name", "output_root", "control_rate_hz"},
    "connection": {"sdk_root", "host", "tcp_port", "udp_port", "connect_timeout_s", "command_timeout_s", "state_timeout_s"},
    "authorization": {"allow_hardware", "allow_motion"},
    "mapping": {"joint_mapping_verified", "joint_mapping_scope", "j1_convention", "joint_direction", "joint_offset_rad"},
    "safety": {
        "joint_position_min_rad", "joint_position_max_rad", "maximum_command_velocity_rad_s",
        "maximum_command_acceleration_rad_s2", "maximum_command_jerk_rad_s3",
        "maximum_tracking_error_rad", "maximum_servo_target_delta_rad",
        "minimum_servo_timestamp_interval_s", "maximum_servo_timestamp_interval_s",
    },
    "feedback": {
        "motion_ready_feedback_max_age_ms", "maximum_disabled_feedback_age_ms",
        "lower_feedback_timeout_ms", "transient_feedback_invalid_recovery_ms",
        "host_state_snapshot_timeout_s",
    },
    "movej": {
        "max_velocity_rad_s", "max_acceleration_rad_s2", "max_jerk_rad_s3", "timeout_s",
        "controlled_park_before_disable", "start_position_tolerance_rad", "settle_velocity_tolerance_rad_s",
    },
    "trajectories": {"A", "B"},
    "identification": {
        "model_file", "identify_binary", "rank_relative_tolerance", "friction_velocity_threshold",
        "enable_armature_columns", "enable_damping_columns", "enable_friction_columns", "preprocessing",
    },
}
TRAJECTORY_KEYS = {"artifact", "metadata", "preview_acceptance"}
PREPROCESSING_KEYS = {"cutoff_hz", "filter_order", "edge_trim_s", "maximum_resample_rate_hz", "gap_periods"}


class DeterministicMockClock:
    def __init__(self) -> None:
        self.now_s = 1.0

    def monotonic(self) -> float:
        return self.now_s

    def monotonic_ns(self) -> int:
        return int(round(self.now_s * 1e9))

    def sleep(self, duration_s: float) -> None:
        self.now_s += max(0.0, float(duration_s))


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a YAML mapping")
    return value


def _exact_keys(value: dict[str, Any], expected: set[str], name: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown:
        raise ValueError(f"unknown {name} keys: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing {name} keys: {sorted(missing)}")


def _positive(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return parsed


def _nonnegative(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be non-negative and finite")
    return parsed


def _port(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer port")
    parsed = int(value)
    if float(parsed) != float(value) or not 1 <= parsed <= 65535:
        raise ValueError(f"{name} must be an integer in [1, 65535]")
    return parsed


def _bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be a YAML boolean")
    return value


def _six(value: Any, name: str, *, positive: bool = False) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{name} must contain exactly six values")
    parsed = [float(item) for item in value]
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{name} must contain finite values")
    if positive and any(item <= 0.0 for item in parsed):
        raise ValueError(f"{name} values must be positive")
    return parsed


def _text(value: Any, name: str) -> str:
    parsed = str(value).strip()
    if not parsed:
        raise ValueError(f"{name} must not be empty")
    return parsed


def _repo_path(value: Any, name: str) -> Path:
    path = Path(_text(value, name)).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()

def load_config(config_path: str | Path) -> dict[str, Any]:
    """Load the only operator-facing real A/B config with a closed schema."""
    path = Path(config_path).expanduser().resolve()
    config = _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), "reBot real A/B config")
    _exact_keys(config, ROOT_KEYS, "top-level config")
    if config["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
    config = deepcopy(config)
    for section, expected in SECTION_KEYS.items():
        _exact_keys(_mapping(config[section], section), expected, section)

    experiment = config["experiment"]
    experiment["name"] = _text(experiment["name"], "experiment.name")
    experiment["output_root"] = str(_repo_path(experiment["output_root"], "experiment.output_root"))
    experiment["control_rate_hz"] = _positive(experiment["control_rate_hz"], "experiment.control_rate_hz")

    connection = config["connection"]
    connection["sdk_root"] = str(_repo_path(connection["sdk_root"], "connection.sdk_root"))
    connection["host"] = _text(connection["host"], "connection.host")
    connection["tcp_port"] = _port(connection["tcp_port"], "connection.tcp_port")
    connection["udp_port"] = _port(connection["udp_port"], "connection.udp_port")
    for name in ("connect_timeout_s", "command_timeout_s", "state_timeout_s"):
        connection[name] = _positive(connection[name], f"connection.{name}")

    for name in SECTION_KEYS["authorization"]:
        config["authorization"][name] = _bool(config["authorization"][name], f"authorization.{name}")

    mapping = config["mapping"]
    mapping["joint_mapping_verified"] = _bool(mapping["joint_mapping_verified"], "mapping.joint_mapping_verified")
    mapping["joint_mapping_scope"] = _text(mapping["joint_mapping_scope"], "mapping.joint_mapping_scope")
    if mapping["joint_mapping_scope"] not in {"excitation_smoke", "excitation"}:
        raise ValueError("mapping.joint_mapping_scope must be excitation_smoke or excitation")
    mapping["j1_convention"] = _text(mapping["j1_convention"], "mapping.j1_convention")
    if mapping["j1_convention"] == "UNRESOLVED":
        raise ValueError("mapping.j1_convention must be explicitly resolved")
    mapping["joint_direction"] = _six(mapping["joint_direction"], "mapping.joint_direction")
    if any(abs(item) != 1.0 for item in mapping["joint_direction"]):
        raise ValueError("mapping.joint_direction values must be exactly -1 or 1")
    mapping["joint_offset_rad"] = _six(mapping["joint_offset_rad"], "mapping.joint_offset_rad")

    safety = config["safety"]
    for name in ("joint_position_min_rad", "joint_position_max_rad"):
        safety[name] = _six(safety[name], f"safety.{name}")
    for name in ("maximum_command_velocity_rad_s", "maximum_command_acceleration_rad_s2", "maximum_command_jerk_rad_s3", "maximum_tracking_error_rad"):
        safety[name] = _six(safety[name], f"safety.{name}", positive=True)
    for lower, upper in zip(safety["joint_position_min_rad"], safety["joint_position_max_rad"]):
        if lower >= upper:
            raise ValueError("each safety joint position minimum must be below maximum")
    for name in ("maximum_servo_target_delta_rad", "minimum_servo_timestamp_interval_s", "maximum_servo_timestamp_interval_s"):
        safety[name] = _positive(safety[name], f"safety.{name}")
    if safety["minimum_servo_timestamp_interval_s"] >= safety["maximum_servo_timestamp_interval_s"]:
        raise ValueError("minimum Servo timestamp interval must be below maximum")

    feedback = config["feedback"]
    for name in SECTION_KEYS["feedback"]:
        feedback[name] = _positive(feedback[name], f"feedback.{name}")
    if feedback["motion_ready_feedback_max_age_ms"] > feedback["lower_feedback_timeout_ms"]:
        raise ValueError("motion-ready feedback age must not exceed lower feedback timeout")

    movej = config["movej"]
    movej["max_velocity_rad_s"] = _six(movej["max_velocity_rad_s"], "movej.max_velocity_rad_s", positive=True)
    movej["max_acceleration_rad_s2"] = _six(movej["max_acceleration_rad_s2"], "movej.max_acceleration_rad_s2", positive=True)
    movej["max_jerk_rad_s3"] = _six(movej["max_jerk_rad_s3"], "movej.max_jerk_rad_s3", positive=True)
    movej["timeout_s"] = _positive(movej["timeout_s"], "movej.timeout_s")
    movej["controlled_park_before_disable"] = _bool(movej["controlled_park_before_disable"], "movej.controlled_park_before_disable")
    if not movej["controlled_park_before_disable"]:
        raise ValueError("movej.controlled_park_before_disable must remain true for formal real excitation")
    movej["start_position_tolerance_rad"] = _positive(movej["start_position_tolerance_rad"], "movej.start_position_tolerance_rad")
    movej["settle_velocity_tolerance_rad_s"] = _positive(movej["settle_velocity_tolerance_rad_s"], "movej.settle_velocity_tolerance_rad_s")

    for label in ("A", "B"):
        item = _mapping(config["trajectories"][label], f"trajectories.{label}")
        _exact_keys(item, TRAJECTORY_KEYS, f"trajectories.{label}")
        for name in TRAJECTORY_KEYS:
            item[name] = str(_repo_path(item[name], f"trajectories.{label}.{name}"))

    identification = config["identification"]
    identification["model_file"] = str(_repo_path(identification["model_file"], "identification.model_file"))
    identification["identify_binary"] = str(_repo_path(identification["identify_binary"], "identification.identify_binary"))
    identification["rank_relative_tolerance"] = _positive(identification["rank_relative_tolerance"], "identification.rank_relative_tolerance")
    identification["friction_velocity_threshold"] = _nonnegative(identification["friction_velocity_threshold"], "identification.friction_velocity_threshold")
    for name in ("enable_armature_columns", "enable_damping_columns", "enable_friction_columns"):
        identification[name] = _bool(identification[name], f"identification.{name}")
    preprocessing = _mapping(identification["preprocessing"], "identification.preprocessing")
    _exact_keys(preprocessing, PREPROCESSING_KEYS, "identification.preprocessing")
    preprocessing["cutoff_hz"] = _positive(preprocessing["cutoff_hz"], "identification.preprocessing.cutoff_hz")
    if isinstance(preprocessing["filter_order"], bool) or int(preprocessing["filter_order"]) != float(preprocessing["filter_order"]) or int(preprocessing["filter_order"]) <= 0:
        raise ValueError("identification.preprocessing.filter_order must be a positive integer")
    preprocessing["filter_order"] = int(preprocessing["filter_order"])
    preprocessing["edge_trim_s"] = _nonnegative(preprocessing["edge_trim_s"], "identification.preprocessing.edge_trim_s")
    preprocessing["maximum_resample_rate_hz"] = _positive(preprocessing["maximum_resample_rate_hz"], "identification.preprocessing.maximum_resample_rate_hz")
    preprocessing["gap_periods"] = _positive(preprocessing["gap_periods"], "identification.preprocessing.gap_periods")
    return config


def _sdk_client_exists(sdk_root: Path) -> bool:
    return any((sdk_root / prefix / "wlsea_arm_sdk" / "client.py").is_file() for prefix in (Path("upper/python"), Path("src")))


def materialize_hardware_config(config: dict[str, Any], label: str, artifact: ReplayArtifact, output_csv: Path) -> dict[str, Any]:
    connection, mapping = config["connection"], config["mapping"]
    safety, feedback, movej = config["safety"], config["feedback"], config["movej"]
    trajectory, authorization = config["trajectories"][label], config["authorization"]
    return {
        "sdk_root": connection["sdk_root"], "host": connection["host"], "tcp_port": connection["tcp_port"], "udp_port": connection["udp_port"],
        "control_mode": "excitation", "control_rate_hz": config["experiment"]["control_rate_hz"], "duration_s": artifact.duration_s,
        "max_samples": None, "output_csv": str(output_csv.resolve()), "allow_hardware": authorization["allow_hardware"], "allow_motion": authorization["allow_motion"],
        "joint_mapping_verified": mapping["joint_mapping_verified"], "joint_mapping_scope": mapping["joint_mapping_scope"], "j1_convention": mapping["j1_convention"],
        "joint_direction": list(mapping["joint_direction"]), "joint_offset_rad": list(mapping["joint_offset_rad"]),
        "joint_position_min_rad": list(safety["joint_position_min_rad"]), "joint_position_max_rad": list(safety["joint_position_max_rad"]),
        "maximum_command_velocity_rad_s": list(safety["maximum_command_velocity_rad_s"]), "maximum_command_acceleration_rad_s2": list(safety["maximum_command_acceleration_rad_s2"]),
        "maximum_command_jerk_rad_s3": list(safety["maximum_command_jerk_rad_s3"]), "maximum_tracking_error_rad": list(safety["maximum_tracking_error_rad"]),
        "maximum_servo_target_delta_rad": safety["maximum_servo_target_delta_rad"], "minimum_servo_timestamp_interval_s": safety["minimum_servo_timestamp_interval_s"],
        "maximum_servo_timestamp_interval_s": safety["maximum_servo_timestamp_interval_s"], "movej_max_velocity_rad_s": list(movej["max_velocity_rad_s"]),
        "movej_max_acceleration_rad_s2": list(movej["max_acceleration_rad_s2"]), "movej_max_jerk_rad_s3": list(movej["max_jerk_rad_s3"]),
        "movej_timeout_s": movej["timeout_s"], "controlled_park_before_disable": movej["controlled_park_before_disable"],
        "start_position_tolerance_rad": movej["start_position_tolerance_rad"], "preposition_settle_velocity_tolerance_rad_s": movej["settle_velocity_tolerance_rad_s"],
        "motion_ready_feedback_max_age_ms": feedback["motion_ready_feedback_max_age_ms"], "maximum_feedback_age_ms": feedback["motion_ready_feedback_max_age_ms"],
        "maximum_disabled_feedback_age_ms": feedback["maximum_disabled_feedback_age_ms"], "lower_feedback_timeout_ms": feedback["lower_feedback_timeout_ms"],
        "transient_feedback_invalid_recovery_ms": feedback["transient_feedback_invalid_recovery_ms"], "connect_timeout_s": connection["connect_timeout_s"],
        "command_timeout_s": connection["command_timeout_s"], "state_timeout_s": connection["state_timeout_s"], "host_state_snapshot_timeout_s": feedback["host_state_snapshot_timeout_s"],
        "trajectory_source": "frozen_replay_artifact", "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE, "trajectory_hash": artifact.sha256,
        "trajectory_artifact": trajectory["artifact"], "trajectory_metadata": trajectory["metadata"], "trajectory_preview_acceptance": trajectory["preview_acceptance"],
    }

def preflight(config: dict[str, Any]) -> dict[str, Any]:
    """Validate every offline dependency/evidence item before any ArmClient can exist."""
    sdk_root = Path(config["connection"]["sdk_root"])
    if not sdk_root.is_dir() or not _sdk_client_exists(sdk_root):
        raise FileNotFoundError(f"configured SDK root is missing or incomplete: {sdk_root}")
    model = Path(config["identification"]["model_file"])
    binary = Path(config["identification"]["identify_binary"])
    if not model.is_file():
        raise FileNotFoundError(f"identification model is missing: {model}")
    if not binary.is_file():
        raise FileNotFoundError(f"identify binary is missing: {binary}")

    artifacts: dict[str, ReplayArtifact] = {}
    qualification, acceptances = {}, {}
    for label in ("A", "B"):
        item = config["trajectories"][label]
        artifact = load_replay_artifact(item["artifact"], item["metadata"])
        hardware = materialize_hardware_config(
            config, label, artifact, ROOT / "data" / ".preflight-unused.csv"
        )
        qualification[label] = validate_replay_runtime_limits(artifact, hardware)
        acceptances[label] = validate_preview_acceptance(
            item["preview_acceptance"], artifact, repo_root=ROOT,
            require_hardware_acceptance=True, replay_mode=TRAJECTORY_REPLAY_MODE,
        )
        artifacts[label] = artifact
    if artifacts["A"].sha256 == artifacts["B"].sha256:
        raise ValueError("A/B trajectory hashes must be different")
    return {
        "artifacts": artifacts,
        "qualification": qualification,
        "acceptances": acceptances,
        "dependencies": {
            "sdk_root": str(sdk_root),
            "model_file": str(model),
            "model_sha256": sha256_file(model),
            "identify_binary": str(binary),
            "identify_binary_sha256": sha256_file(binary),
        },
    }


def _new_run_directory(output_root: Path, name: str, now: datetime | None = None) -> Path:
    moment = now or datetime.now()
    run_dir = output_root / f"{moment.strftime('%Y%m%d_%H%M%S_%f')}_{name}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def _write_yaml(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(value, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )


def _provenance(
    config_path: Path, config: dict[str, Any], checked: dict[str, Any]
) -> dict[str, Any]:
    files: dict[str, Any] = {
        "experiment_config": {"path": str(config_path), "sha256": sha256_file(config_path)},
        "model_file": {
            "path": checked["dependencies"]["model_file"],
            "sha256": checked["dependencies"]["model_sha256"],
        },
        "identify_binary": {
            "path": checked["dependencies"]["identify_binary"],
            "sha256": checked["dependencies"]["identify_binary_sha256"],
        },
    }
    for label in ("A", "B"):
        item = config["trajectories"][label]
        files[f"trajectory_{label}"] = {
            "artifact": item["artifact"],
            "artifact_sha256": checked["artifacts"][label].sha256,
            "metadata": item["metadata"],
            "metadata_sha256": sha256_file(item["metadata"]),
            "preview_acceptance": item["preview_acceptance"],
            "preview_acceptance_sha256": sha256_file(item["preview_acceptance"]),
        }
    return {
        "schema_version": "rebot_real_ab_provenance_v1",
        "git_commit": repository_git_commit(ROOT),
        "files": files,
    }


def _print_trajectory_gate(
    label: str,
    hardware: dict[str, Any],
    artifact: ReplayArtifact,
    *,
    mock: bool,
) -> None:
    mode = "MOCK EXECUTION" if mock else "REAL HARDWARE EXECUTION"
    print(f"\n=== {mode} GATE ===")
    print(f"Trajectory: {label}")
    print(f"Duration: {artifact.duration_s:.6g} s")
    print(f"Control rate: {hardware['control_rate_hz']:.6g} Hz")
    print(f"Trajectory hash: {artifact.sha256}")
    print(
        f"Host: {hardware['host']}:{hardware['tcp_port']} / UDP {hardware['udp_port']}"
    )
    print(f"Output: {hardware['output_csv']}")


def _default_trajectory_executor(
    label: str, hardware: dict[str, Any], mock: bool
) -> dict[str, Any]:
    if not mock:
        return RebotHardwareRunner(hardware, repo_root=ROOT, mock_backend=False).run()

    clock = DeterministicMockClock()

    def client_factory(**kwargs: Any) -> MockArmClient:
        return MockArmClient(
            **kwargs,
            position_rad=[math.pi, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
            follow_movej_targets=True,
            follow_servo_targets=True,
            monotonic_ns_fn=clock.monotonic_ns,
        )

    return RebotHardwareRunner(
        hardware,
        repo_root=ROOT,
        client_factory=client_factory,
        mock_backend=True,
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        monotonic_ns_fn=clock.monotonic_ns,
    ).run()


def _validate_completed_metadata(
    metadata: dict[str, Any], hardware: dict[str, Any], artifact: ReplayArtifact
) -> None:
    if metadata.get("motion_status") != "completed":
        raise RuntimeError(
            f"motion_status={metadata.get('motion_status')!r}, expected completed"
        )
    if int(metadata.get("observed_sample_count", 0)) <= 0:
        raise RuntimeError("hardware run produced no samples")
    if metadata.get("trajectory_hash") != artifact.sha256:
        raise RuntimeError("recorded trajectory hash differs from preflight artifact")
    shutdown = metadata.get("shutdown") or {}
    if (
        shutdown.get("cleanup_errors")
        or shutdown.get("disable_status") != "completed"
        or shutdown.get("close_status") != "completed"
    ):
        raise RuntimeError("hardware cleanup did not complete successfully")
    envelope = metadata.get("runtime_servo_envelope") or {}
    if not math.isclose(
        float(envelope.get("trajectory_time_end_s", -1.0)),
        artifact.duration_s,
        abs_tol=1e-8,
    ):
        raise RuntimeError("excitation did not reach the frozen trajectory endpoint")
    timing = metadata.get("dispatch_timing") or {}
    if (
        timing.get("catch_up_burst_count") != 0
        or timing.get("command_dispatch_timestamp_mismatch_count") != 0
    ):
        raise RuntimeError("dispatch metadata failed no-catch-up/timestamp checks")
    raw = Path(hardware["output_csv"])
    if not raw.is_file() or not raw.with_suffix(".meta.yaml").is_file():
        raise RuntimeError("hardware raw CSV or metadata sidecar is missing")


def _trajectory_summary(
    metadata: dict[str, Any], artifact: ReplayArtifact
) -> dict[str, Any]:
    cadence = metadata.get("feedback_cadence") or {}
    lower = cadence.get("timestamp_lower_update_cadence_on_host") or {}
    tracking = metadata.get("tracking_quality") or {}
    runtime = metadata.get("runtime_servo_envelope") or {}
    return {
        "motion_status": metadata.get("motion_status"),
        "sample_count": metadata.get("observed_sample_count"),
        "duration_s": artifact.duration_s,
        "trajectory_sha256": artifact.sha256,
        "dispatch_rate_hz": (metadata.get("dispatch_timing") or {}).get(
            "effective_command_rate_hz"
        ),
        "lower_feedback_update_rate_hz": lower.get("effective_rate_hz"),
        "tracking_quality": tracking.get("status"),
        "tracking_max_abs_rad": tracking.get("overall_max_abs_rad"),
        "primary_fault_code": (
            0
            if "failure" not in metadata
            else (metadata["failure"] or {}).get("primary_fault_code")
        ),
        "servo_ownership": (
            "released"
            if (metadata.get("shutdown") or {}).get("exit_servo_status")
            in {"completed", "not_needed"}
            else "unknown"
        ),
        "cleanup_status": (
            "completed"
            if not (metadata.get("shutdown") or {}).get("cleanup_errors")
            else "failed"
        ),
        "measurement_unusable_sample_count": (
            runtime.get("feedback_runtime") or {}
        ).get("measurement_unusable_sample_count"),
    }

def _identification_config(
    config: dict[str, Any], run_dir: Path
) -> dict[str, Any]:
    source = config["identification"]
    return {
        "training_raw_csv": str((run_dir / "A" / "raw.csv").resolve()),
        "validation_raw_csv": str((run_dir / "B" / "raw.csv").resolve()),
        "model_file": source["model_file"],
        "identify_binary": source["identify_binary"],
        "output_directory": str((run_dir / "identification").resolve()),
        "rank_relative_tolerance": source["rank_relative_tolerance"],
        "friction_velocity_threshold": source["friction_velocity_threshold"],
        "enable_armature_columns": source["enable_armature_columns"],
        "enable_damping_columns": source["enable_damping_columns"],
        "enable_friction_columns": source["enable_friction_columns"],
        "preprocessing": deepcopy(source["preprocessing"]),
    }


def _relocate_identification_config(pending: Path, output: Path) -> None:
    if pending.exists() and output.exists():
        target = output / "config.yaml"
        if target.exists():
            raise FileExistsError(
                f"identification config destination already exists: {target}"
            )
        pending.replace(target)


def _identification_summary(
    report: dict[str, Any], output: Path, *, status: str = "completed"
) -> dict[str, Any]:
    return {
        "status": status,
        "base_parameter_rank": report.get("base_parameter_rank"),
        "full_parameter_count": report.get("full_parameter_count"),
        "training_reported_effort_rmse": (
            report.get("training_error") or {}
        ).get("aggregate_rmse"),
        "validation_reported_effort_rmse": (
            report.get("validation_error") or {}
        ).get("aggregate_rmse"),
        "result_yaml": str((output / "result.yaml").resolve()),
        "validation_png": str((output / "validation.png").resolve()),
        "effort_source": "SDK reported effort",
        "physical_torque_calibration": "unresolved",
        "interpretation": report.get(
            "result_interpretation",
            "Uncalibrated reported-effort prediction, not physical parameter recovery",
        ),
    }


def _run_mock_fixture_validation(
    config: dict[str, Any], run_dir: Path
) -> dict[str, Any]:
    """Validate offline identification if ideal Mock effort is rank-degenerate."""
    from generate_rebot_identification_demo import generate

    fixture_root = run_dir / "identification_fixture"
    generate(
        fixture_root,
        Path(config["identification"]["identify_binary"]).parent,
    )
    report = run_pipeline(fixture_root / "pipeline.yaml")
    return _identification_summary(
        report,
        fixture_root / "identified",
        status="fixture_pass_mock_reported_effort_not_fit",
    )


def _print_final_summary(summary: dict[str, Any]) -> None:
    print("\n=== reBot A/B SUMMARY ===")
    print(f"Experiment: {summary['experiment_status']}")
    print(f"Run directory: {summary.get('run_directory')}")
    for label in ("A", "B"):
        item = (summary.get("trajectories") or {}).get(label)
        if not item:
            print(f"\nTrajectory {label}: NOT RUN")
            continue
        print(f"\nTrajectory {label}:")
        print(f"  motion status: {item.get('motion_status')}")
        print(f"  sample count: {item.get('sample_count')}")
        print(f"  duration: {item.get('duration_s')} s")
        print(f"  dispatch rate: {item.get('dispatch_rate_hz')} Hz")
        print(
            "  lower feedback update rate: "
            f"{item.get('lower_feedback_update_rate_hz')} Hz"
        )
        print(f"  tracking quality: {item.get('tracking_quality')}")
        print(f"  fault: {item.get('primary_fault_code')}")
    identification = summary.get("identification")
    if identification:
        print("\nIdentification:")
        print(f"  status: {identification.get('status')}")
        print(
            "  base rank / full parameter count: "
            f"{identification.get('base_parameter_rank')}/"
            f"{identification.get('full_parameter_count')}"
        )
        print(
            "  A reported-effort RMSE: "
            f"{identification.get('training_reported_effort_rmse')}"
        )
        print(
            "  B reported-effort RMSE: "
            f"{identification.get('validation_reported_effort_rmse')}"
        )
        print(f"  result.yaml: {identification.get('result_yaml')}")
        print(f"  validation.png: {identification.get('validation_png')}")
    print("\nImportant:")
    print("  effort source = SDK reported effort")
    print("  physical torque calibration = unresolved")
    if summary.get("error"):
        print(f"\nError: {summary['error']}")


def run_experiment(
    config_path: str | Path,
    *,
    mock: bool = False,
    preflight_only: bool = False,
    trajectory_executor: Callable[
        [str, dict[str, Any], bool], dict[str, Any]
    ]
    | None = None,
    identification_executor: Callable[[Path], dict[str, Any]] | None = None,
    confirmation_fn: Callable[[str], str] = input,
    now: datetime | None = None,
) -> dict[str, Any]:
    """Run preflight, A, B, then A-fit/B-validation as one fail-closed flow."""
    config_file = Path(config_path).expanduser().resolve()
    config = load_config(config_file)
    checked = preflight(config)
    if preflight_only:
        result = {
            "experiment_status": "PREFLIGHT_PASS",
            "run_directory": None,
            "hardware_contacted": False,
            "trajectories": {
                label: {
                    "trajectory_sha256": checked["artifacts"][label].sha256,
                    "sample_count": len(checked["artifacts"][label].samples),
                    "sample_rate_hz": checked["artifacts"][label].sample_rate_hz,
                    "duration_s": checked["artifacts"][label].duration_s,
                    "preview_accepted_for_hardware": checked["acceptances"][
                        label
                    ].get("accepted_for_hardware"),
                    "qualification": "PASS",
                }
                for label in ("A", "B")
            },
        }
        _print_final_summary(result)
        return result

    run_dir = _new_run_directory(
        Path(config["experiment"]["output_root"]),
        config["experiment"]["name"],
        now=now,
    )
    shutil.copyfile(config_file, run_dir / "experiment_config.snapshot.yaml")
    _write_yaml(
        run_dir / "provenance.yaml",
        _provenance(config_file, config, checked),
    )
    summary: dict[str, Any] = {
        "schema_version": "rebot_real_ab_summary_v1",
        "experiment_status": "RUNNING",
        "run_directory": str(run_dir.resolve()),
        "backend": "mock" if mock else "real",
        "hardware_contacted": False if mock else None,
        "trajectories": {},
        "identification": None,
    }
    execute = trajectory_executor or _default_trajectory_executor
    identify = identification_executor or run_pipeline
    try:
        for label in ("A", "B"):
            artifact = checked["artifacts"][label]
            hardware = materialize_hardware_config(
                config, label, artifact, run_dir / label / "raw.csv"
            )
            _write_yaml(run_dir / label / "hardware.yaml", hardware)
            _print_trajectory_gate(label, hardware, artifact, mock=mock)
            if not mock:
                confirmation_fn(
                    f"Press ENTER to execute trajectory {label}, or Ctrl-C to abort."
                )
            metadata = execute(label, hardware, mock)
            _validate_completed_metadata(metadata, hardware, artifact)
            summary["trajectories"][label] = _trajectory_summary(
                metadata, artifact
            )
            if not mock:
                summary["hardware_contacted"] = True

        identification_config = _identification_config(config, run_dir)
        pending_config = run_dir / "identification.config.pending.yaml"
        output = run_dir / "identification"
        _write_yaml(pending_config, identification_config)
        try:
            report = identify(pending_config)
        except Exception as exc:
            _relocate_identification_config(pending_config, output)
            if not mock or identification_executor is not None:
                raise
            summary["mock_reported_effort_identification"] = {
                "status": "not_meaningful",
                "error": f"{type(exc).__name__}: {exc}",
                "reason": (
                    "ideal MockArmClient does not provide identification-grade "
                    "physical effort/velocity dynamics"
                ),
            }
            summary["identification"] = _run_mock_fixture_validation(
                config, run_dir
            )
        else:
            _relocate_identification_config(pending_config, output)
            summary["identification"] = _identification_summary(
                report,
                output,
                status=(
                    "mock_pipeline_completed_not_physical"
                    if mock
                    else "completed"
                ),
            )
        summary["experiment_status"] = "PASS"
        _write_yaml(run_dir / "summary.yaml", summary)
    except BaseException as exc:
        summary["experiment_status"] = "FAIL"
        summary["error"] = f"{type(exc).__name__}: {exc}"
        _write_yaml(run_dir / "summary.yaml", summary)
        _print_final_summary(summary)
        raise
    _print_final_summary(summary)
    return summary

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "rebot_real_ab.yaml")
    parser.add_argument("--mock", action="store_true", help="use only in-process MockArmClient; never create a real SDK client")
    parser.add_argument("--preflight-only", action="store_true", help="validate config/artifacts/dependencies only; create no hardware client")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_experiment(args.config, mock=args.mock, preflight_only=args.preflight_only)
    except KeyboardInterrupt:
        print("\nAborted by operator.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"reBot A/B experiment failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    if args.mock:
        print("mock_only=true; no real hardware was contacted")
    if args.preflight_only:
        print("preflight_only=true; no hardware client was created")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
