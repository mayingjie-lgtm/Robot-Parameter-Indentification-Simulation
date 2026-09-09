from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any

import yaml

from .state_capture import JOINT_COUNT


SCHEMA_VERSION = "rebot_replay_trajectory_v1"
CSV_COLUMNS = (
    "time",
    *(f"q_ref{joint}" for joint in range(JOINT_COUNT)),
    *(f"qd_ref{joint}" for joint in range(JOINT_COUNT)),
    *(f"qdd_ref{joint}" for joint in range(JOINT_COUNT)),
)


@dataclass(frozen=True)
class ReplaySample:
    time: float
    q_ref: tuple[float, ...]
    qd_ref: tuple[float, ...]
    qdd_ref: tuple[float, ...]


@dataclass(frozen=True)
class ReplayArtifact:
    path: Path
    metadata_path: Path
    sha256: str
    sample_rate_hz: float
    duration_s: float
    samples: tuple[ReplaySample, ...]
    metadata: dict[str, Any]

    @property
    def q_start(self) -> tuple[float, ...]:
        return self.samples[0].q_ref


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_provenance_file(
    metadata: dict[str, Any],
    path_key: str,
    hash_key: str,
) -> None:
    value = metadata.get(path_key)
    expected_hash = metadata.get(hash_key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"trajectory metadata {path_key} is missing")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ValueError(f"trajectory metadata {hash_key} must be a SHA-256 digest")
    path = Path(value).expanduser()
    if not path.is_file():
        raise ValueError(f"trajectory provenance file is missing: {path}")
    if sha256_file(path) != expected_hash:
        raise ValueError(f"trajectory metadata {hash_key} mismatch for {path_key}")


def _finite(row: dict[str, str], name: str, line_number: int) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"artifact line {line_number}: invalid {name}") from exc
    if not math.isfinite(value):
        raise ValueError(f"artifact line {line_number}: {name} must be finite")
    return value


def _six(row: dict[str, str], prefix: str, line_number: int) -> tuple[float, ...]:
    return tuple(
        _finite(row, f"{prefix}{joint}", line_number)
        for joint in range(JOINT_COUNT)
    )


def load_replay_artifact(
    artifact_path: str | Path,
    metadata_path: str | Path,
) -> ReplayArtifact:
    """Read one frozen sampled trajectory without evaluating or interpolating Fourier math."""

    path = Path(artifact_path)
    meta_path = Path(metadata_path)
    if not path.is_file():
        raise FileNotFoundError(f"trajectory artifact not found: {path}")
    if not meta_path.is_file():
        raise FileNotFoundError(f"trajectory metadata not found: {meta_path}")

    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(CSV_COLUMNS):
            raise ValueError(
                f"trajectory artifact header must be exactly {list(CSV_COLUMNS)}"
            )
        samples: list[ReplaySample] = []
        for line_number, row in enumerate(reader, start=2):
            samples.append(
                ReplaySample(
                    time=_finite(row, "time", line_number),
                    q_ref=_six(row, "q_ref", line_number),
                    qd_ref=_six(row, "qd_ref", line_number),
                    qdd_ref=_six(row, "qdd_ref", line_number),
                )
            )

    if len(samples) < 2:
        raise ValueError("trajectory artifact requires at least two samples")
    if abs(samples[0].time) > 1e-12:
        raise ValueError("trajectory artifact must start at time=0")

    intervals = []
    for previous, current in zip(samples, samples[1:]):
        dt = current.time - previous.time
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError(
                "trajectory artifact timestamps must be strictly increasing"
            )
        intervals.append(dt)
    nominal_dt = intervals[0]
    dt_tolerance = max(1e-12, nominal_dt * 1e-8)
    if any(abs(dt - nominal_dt) > dt_tolerance for dt in intervals[1:]):
        raise ValueError(
            "trajectory artifact timestamps must use one fixed sampling grid"
        )
    sample_rate_hz = 1.0 / nominal_dt
    duration_s = samples[-1].time

    metadata = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("trajectory metadata must be a YAML mapping")
    digest = sha256_file(path)

    required = {
        "schema_version": SCHEMA_VERSION,
        "robot": "rebot_dm",
        "dof": JOINT_COUNT,
    }
    for name, expected in required.items():
        if metadata.get(name) != expected:
            raise ValueError(
                f"trajectory metadata {name}={metadata.get(name)!r}, "
                f"expected {expected!r}"
            )
    if metadata.get("trajectory_sha256") != digest:
        raise ValueError("trajectory metadata hash does not match artifact bytes")
    if int(metadata.get("sample_count", -1)) != len(samples):
        raise ValueError("trajectory metadata sample_count mismatch")
    if not math.isclose(
        float(metadata.get("sample_rate_hz", math.nan)),
        sample_rate_hz,
        rel_tol=1e-8,
        abs_tol=1e-8,
    ):
        raise ValueError("trajectory metadata sample_rate_hz mismatch")
    if not math.isclose(
        float(metadata.get("duration_s", math.nan)),
        duration_s,
        rel_tol=1e-10,
        abs_tol=1e-10,
    ):
        raise ValueError("trajectory metadata duration_s mismatch")

    q_start = metadata.get("q_start")
    if not isinstance(q_start, list) or len(q_start) != JOINT_COUNT:
        raise ValueError("trajectory metadata q_start must contain six values")
    if any(
        not math.isclose(float(q_start[index]), samples[0].q_ref[index], abs_tol=1e-12)
        for index in range(JOINT_COUNT)
    ):
        raise ValueError("trajectory metadata q_start mismatch")

    if metadata.get("source_type") != "cxx_fourier_trajectory":
        raise ValueError(
            "trajectory metadata source_type must be cxx_fourier_trajectory"
        )
    if metadata.get("source_controller_precheck") != "PASS":
        raise ValueError(
            "trajectory metadata source_controller_precheck must be PASS"
        )
    if metadata.get("collision_precheck") != "PASS":
        raise ValueError("trajectory metadata collision_precheck must be PASS")
    if int(metadata.get("collision_precheck_sample_count", -1)) != len(samples):
        raise ValueError(
            "trajectory metadata collision_precheck_sample_count mismatch"
        )
    if not str(metadata.get("sample_rate_status", "")).startswith("candidate_"):
        raise ValueError(
            "trajectory metadata must mark replay rate as candidate, "
            "not hardware-certified"
        )

    _validate_provenance_file(
        metadata, "source_coefficient_file", "source_coefficient_sha256"
    )
    _validate_provenance_file(metadata, "model_file", "model_hash")
    _validate_provenance_file(
        metadata, "limits_config_file", "limits_config_hash"
    )
    _validate_provenance_file(
        metadata, "collision_model_file", "collision_model_hash"
    )

    return ReplayArtifact(
        path=path,
        metadata_path=meta_path,
        sha256=digest,
        sample_rate_hz=sample_rate_hz,
        duration_s=duration_s,
        samples=tuple(samples),
        metadata=metadata,
    )


def _six_limit(config: dict[str, Any], name: str) -> tuple[float, ...]:
    value = config.get(name)
    if not isinstance(value, list) or len(value) != JOINT_COUNT:
        raise ValueError(f"{name} must contain exactly six values")
    parsed = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{name} must contain finite values")
    return parsed


def qualify_replay_artifact(
    artifact: ReplayArtifact,
    config: dict[str, Any],
) -> dict[str, Any]:
    """Qualify the sampled artifact itself; jerk is finite-differenced from stored qdd."""

    lower = _six_limit(config, "joint_position_min_rad")
    upper = _six_limit(config, "joint_position_max_rad")
    max_qd = _six_limit(config, "maximum_command_velocity_rad_s")
    max_qdd = _six_limit(config, "maximum_command_acceleration_rad_s2")
    max_jerk = _six_limit(config, "maximum_command_jerk_rad_s3")
    max_target_delta = float(config["maximum_servo_target_delta_rad"])
    if not math.isfinite(max_target_delta) or max_target_delta <= 0.0:
        raise ValueError("maximum_servo_target_delta_rad must be positive and finite")
    design_target_raw = config.get("servo_target_delta_design_limit_rad")
    design_target_delta = (
        float(design_target_raw) if design_target_raw is not None else None
    )
    if design_target_delta is not None:
        if (
            not math.isfinite(design_target_delta)
            or design_target_delta <= 0.0
            or design_target_delta >= max_target_delta
        ):
            raise ValueError(
                "servo_target_delta_design_limit_rad must be positive, finite, "
                "and strictly below maximum_servo_target_delta_rad"
            )
    expected_start = _six_limit(config, "expected_start_position_rad")
    for joint in range(JOINT_COUNT):
        if not lower[joint] < upper[joint]:
            raise ValueError(
                "qualification position minimum must be below maximum"
            )
        if min(max_qd[joint], max_qdd[joint], max_jerk[joint]) <= 0:
            raise ValueError("qualification dynamic limits must be positive")

    expected_rate = float(config["expected_sample_rate_hz"])
    expected_duration = float(config["expected_duration_s"])
    expected_sample_count = int(config["expected_sample_count"])
    rate_tol = float(config.get("sample_rate_tolerance_hz", 1e-8))
    duration_tol = float(config.get("duration_tolerance_s", 1e-9))
    start_position_tol = float(config["start_position_tolerance_rad"])
    start_velocity_tol = float(config["start_velocity_tolerance_rad_s"])
    start_acceleration_tol = float(config["start_acceleration_tolerance_rad_s2"])
    require_end_match = bool(config.get("require_end_matches_start", False))
    endpoint_position_tol = float(
        config.get("endpoint_position_tolerance_rad", start_position_tol)
    )
    endpoint_velocity_tol = float(
        config.get("endpoint_velocity_tolerance_rad_s", start_velocity_tol)
    )
    endpoint_acceleration_tol = float(
        config.get(
            "endpoint_acceleration_tolerance_rad_s2",
            start_acceleration_tol,
        )
    )

    failures: list[str] = []
    if abs(artifact.sample_rate_hz - expected_rate) > rate_tol:
        failures.append(
            f"sample_rate_hz={artifact.sample_rate_hz:.17g} differs from "
            f"expected {expected_rate:.17g}"
        )
    if abs(artifact.duration_s - expected_duration) > duration_tol:
        failures.append(
            f"duration_s={artifact.duration_s:.17g} differs from "
            f"expected {expected_duration:.17g}"
        )
    if len(artifact.samples) != expected_sample_count:
        failures.append(
            f"sample_count={len(artifact.samples)} differs from "
            f"expected {expected_sample_count}"
        )

    per_joint = []
    dt = 1.0 / artifact.sample_rate_hz
    for joint in range(JOINT_COUNT):
        q_values = [sample.q_ref[joint] for sample in artifact.samples]
        qd_values = [sample.qd_ref[joint] for sample in artifact.samples]
        qdd_values = [sample.qdd_ref[joint] for sample in artifact.samples]
        jerk_values = [
            (current - previous) / dt
            for previous, current in zip(qdd_values, qdd_values[1:])
        ]
        target_delta_values = [
            abs(current - previous)
            for previous, current in zip(q_values, q_values[1:])
        ]
        q_min = min(q_values)
        q_max = max(q_values)
        qd_abs_max = max(abs(value) for value in qd_values)
        qdd_abs_max = max(abs(value) for value in qdd_values)
        jerk_abs_max = max((abs(value) for value in jerk_values), default=0.0)
        if target_delta_values:
            target_delta_abs_max_sample_index, target_delta_abs_max = max(
                enumerate(target_delta_values, start=1),
                key=lambda item: item[1],
            )
        else:
            target_delta_abs_max_sample_index, target_delta_abs_max = 0, 0.0
        minimum_margin = min(
            min(value - lower[joint], upper[joint] - value)
            for value in q_values
        )
        metrics = {
            "joint": joint + 1,
            "q_min": q_min,
            "q_max": q_max,
            "qd_abs_max": qd_abs_max,
            "qdd_abs_max": qdd_abs_max,
            "jerk_abs_max": jerk_abs_max,
            "servo_target_delta_abs_max": target_delta_abs_max,
            "servo_target_delta_abs_max_sample_index": target_delta_abs_max_sample_index,
            "servo_target_delta_margin_rad": max_target_delta - target_delta_abs_max,
            "servo_target_delta_design_limit_rad": design_target_delta,
            "servo_target_delta_design_margin_rad": (
                None
                if design_target_delta is None
                else design_target_delta - target_delta_abs_max
            ),
            "servo_target_delta_design_pass": (
                None
                if design_target_delta is None
                else target_delta_abs_max <= design_target_delta + 1e-12
            ),
            "minimum_position_limit_margin": minimum_margin,
            "start_position": q_values[0],
            "end_position": q_values[-1],
            "start_velocity": qd_values[0],
            "end_velocity": qd_values[-1],
            "start_acceleration": qdd_values[0],
            "end_acceleration": qdd_values[-1],
        }
        per_joint.append(metrics)

        if q_min < lower[joint] or q_max > upper[joint]:
            failures.append(f"J{joint + 1} position limit violated")
        if qd_abs_max > max_qd[joint] + 1e-12:
            failures.append(f"J{joint + 1} velocity limit violated")
        if qdd_abs_max > max_qdd[joint] + 1e-12:
            failures.append(f"J{joint + 1} acceleration limit violated")
        if jerk_abs_max > max_jerk[joint] + 1e-9:
            failures.append(f"J{joint + 1} jerk limit violated")
        if target_delta_abs_max > max_target_delta + 1e-12:
            failures.append(
                f"J{joint + 1} ServoCore target delta gate violated: "
                f"{target_delta_abs_max:.9g} > {max_target_delta:.9g} rad"
            )
        if minimum_margin < 0:
            failures.append(f"J{joint + 1} negative position-limit margin")
        if abs(q_values[0] - expected_start[joint]) > start_position_tol:
            failures.append(f"J{joint + 1} start position mismatch")
        if abs(qd_values[0]) > start_velocity_tol:
            failures.append(f"J{joint + 1} start velocity not near zero")
        if abs(qdd_values[0]) > start_acceleration_tol:
            failures.append(f"J{joint + 1} start acceleration not near zero")
        if require_end_match:
            if abs(q_values[-1] - q_values[0]) > endpoint_position_tol:
                failures.append(
                    f"J{joint + 1} endpoint position does not return to start"
                )
            if abs(qd_values[-1]) > endpoint_velocity_tol:
                failures.append(
                    f"J{joint + 1} endpoint velocity not near zero"
                )
            if abs(qdd_values[-1]) > endpoint_acceleration_tol:
                failures.append(
                    f"J{joint + 1} endpoint acceleration not near zero"
                )

    return {
        "schema_version": "rebot_trajectory_preview_report_v1",
        "trajectory_sha256": artifact.sha256,
        "artifact_schema_version": SCHEMA_VERSION,
        "preview_status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "sample_count": len(artifact.samples),
        "duration_s": artifact.duration_s,
        "sample_rate_hz": artifact.sample_rate_hz,
        "maximum_servo_target_delta_rad": max_target_delta,
        "servo_target_delta_design_limit_rad": design_target_delta,
        "servo_target_delta_design_status": (
            None
            if design_target_delta is None
            else (
                "PASS"
                if all(
                    item["servo_target_delta_design_pass"] is True
                    for item in per_joint
                )
                else "FAIL"
            )
        ),
        "jerk_method": (
            "forward_difference_of_stored_qdd_on_fixed_artifact_grid"
        ),
        "source_controller_precheck": artifact.metadata.get(
            "source_controller_precheck"
        ),
        "collision_precheck": artifact.metadata.get("collision_precheck"),
        "model_file": artifact.metadata.get("model_file"),
        "model_hash": artifact.metadata.get("model_hash"),
        "limits_config_file": artifact.metadata.get("limits_config_file"),
        "limits_config_hash": artifact.metadata.get("limits_config_hash"),
        "source_provenance": artifact.metadata.get("source_provenance"),
        "source_seed": artifact.metadata.get("source_seed"),
        "source_accepted_attempt": artifact.metadata.get("source_accepted_attempt"),
        "source_coefficient_file": artifact.metadata.get("source_coefficient_file"),
        "source_coefficient_sha256": artifact.metadata.get(
            "source_coefficient_sha256"
        ),
        "per_joint": per_joint,
    }


def validate_replay_runtime_limits(
    artifact: ReplayArtifact,
    config: dict[str, Any],
) -> dict[str, Any]:
    """Fail closed on the exact frozen samples before any client is created."""

    lower = _six_limit(config, "joint_position_min_rad")
    upper = _six_limit(config, "joint_position_max_rad")
    max_qd = _six_limit(config, "maximum_command_velocity_rad_s")
    max_qdd = _six_limit(config, "maximum_command_acceleration_rad_s2")
    max_jerk = _six_limit(config, "maximum_command_jerk_rad_s3")
    max_target_delta = float(config["maximum_servo_target_delta_rad"])
    if not math.isfinite(max_target_delta) or max_target_delta <= 0.0:
        raise ValueError("maximum_servo_target_delta_rad must be positive and finite")

    control_rate = float(config["control_rate_hz"])
    if not math.isfinite(control_rate) or control_rate <= 0:
        raise ValueError("control_rate_hz must be positive and finite")
    if not math.isclose(
        artifact.sample_rate_hz,
        control_rate,
        rel_tol=1e-10,
        abs_tol=1e-10,
    ):
        raise ValueError(
            "control_rate_hz must exactly match frozen artifact sample_rate_hz"
        )
    configured_duration = float(config["duration_s"])
    if not math.isclose(
        artifact.duration_s,
        configured_duration,
        rel_tol=1e-10,
        abs_tol=1e-10,
    ):
        raise ValueError(
            "duration_s must exactly match frozen artifact duration_s"
        )
    expected_sample_count = int(round(control_rate * configured_duration)) + 1
    if len(artifact.samples) != expected_sample_count:
        raise ValueError(
            f"artifact sample_count={len(artifact.samples)} does not match fixed grid "
            f"expectation {expected_sample_count}"
        )

    dt = 1.0 / artifact.sample_rate_hz
    max_seen_jerk = [0.0] * JOINT_COUNT
    max_seen_target_delta = [0.0] * JOINT_COUNT
    max_seen_target_delta_sample_index = [0] * JOINT_COUNT
    previous_q = artifact.q_start
    for sample_index, sample in enumerate(artifact.samples):
        for joint in range(JOINT_COUNT):
            if (
                sample.q_ref[joint] < lower[joint]
                or sample.q_ref[joint] > upper[joint]
            ):
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} "
                    "violates position limits"
                )
            if abs(sample.qd_ref[joint]) > max_qd[joint] + 1e-12:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} "
                    "violates velocity limit"
                )
            if abs(sample.qdd_ref[joint]) > max_qdd[joint] + 1e-12:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} "
                    "violates acceleration limit"
                )
            target_delta = abs(sample.q_ref[joint] - previous_q[joint])
            if target_delta > max_seen_target_delta[joint]:
                max_seen_target_delta[joint] = target_delta
                max_seen_target_delta_sample_index[joint] = sample_index
            if target_delta > max_target_delta + 1e-12:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} target delta "
                    f"{target_delta:.9g} rad exceeds lower ServoCore fixed gate "
                    f"{max_target_delta:.9g} rad"
                )
        previous_q = sample.q_ref
        if sample_index == 0:
            continue
        previous = artifact.samples[sample_index - 1]
        for joint in range(JOINT_COUNT):
            jerk = abs(
                (sample.qdd_ref[joint] - previous.qdd_ref[joint]) / dt
            )
            max_seen_jerk[joint] = max(max_seen_jerk[joint], jerk)
            if jerk > max_jerk[joint] + 1e-9:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} "
                    "violates jerk limit"
                )

    return {
        "trajectory_sha256": artifact.sha256,
        "sample_count": len(artifact.samples),
        "sample_rate_hz": artifact.sample_rate_hz,
        "duration_s": artifact.duration_s,
        "jerk_method": (
            "forward_difference_of_stored_qdd_on_fixed_artifact_grid"
        ),
        "jerk_abs_max": max_seen_jerk,
        "servo_target_delta_abs_max": max_seen_target_delta,
        "servo_target_delta_abs_max_sample_index": max_seen_target_delta_sample_index,
        "servo_target_delta_margin_rad": [
            max_target_delta - value for value in max_seen_target_delta
        ],
        "maximum_servo_target_delta_rad": max_target_delta,
    }


def write_preview_outputs(
    *,
    report: dict[str, Any],
    report_path: str | Path,
    acceptance_path: str | Path,
    preview_mp4: str | Path,
) -> None:
    report_path = Path(report_path)
    acceptance_path = Path(acceptance_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    acceptance_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    acceptance = {
        "schema_version": "rebot_trajectory_preview_acceptance_v1",
        "trajectory_sha256": report["trajectory_sha256"],
        "preview_report": str(report_path),
        "preview_mp4": str(preview_mp4),
        "operator": "",
        "review_date": None,
        "accepted_for_hardware": False,
        "notes": (
            (
                "Default is false. A human operator must watch the exact-command "
                "MP4 and manually accept this exact artifact hash."
            )
            if report["preview_status"] == "PASS"
            else (
                "Numerical qualification is FAIL. Do not set accepted_for_hardware=true; "
                "regenerate a qualifying artifact before any hardware authorization."
            )
        ),
    }
    acceptance_path.write_text(
        yaml.safe_dump(acceptance, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _resolve_evidence_path(value: Any, *, repo_root: Path) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else repo_root / path


def validate_preview_acceptance(
    acceptance_path: str | Path,
    artifact: ReplayArtifact,
    *,
    repo_root: str | Path,
    require_hardware_acceptance: bool = True,
) -> dict[str, Any]:
    path = Path(acceptance_path)
    if not path.is_file():
        raise PermissionError("preview acceptance file is missing")
    parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError("preview acceptance must be a YAML mapping")
    if parsed.get("schema_version") != "rebot_trajectory_preview_acceptance_v1":
        raise ValueError("unsupported preview acceptance schema")
    if parsed.get("trajectory_sha256") != artifact.sha256:
        raise PermissionError(
            "preview acceptance hash does not match trajectory artifact"
        )
    if require_hardware_acceptance and parsed.get("accepted_for_hardware") is not True:
        raise PermissionError(
            "preview acceptance is not accepted_for_hardware=true"
        )

    root = Path(repo_root)
    report_path = _resolve_evidence_path(
        parsed.get("preview_report", ""),
        repo_root=root,
    )
    mp4_path = _resolve_evidence_path(
        parsed.get("preview_mp4", ""),
        repo_root=root,
    )
    if not report_path.is_file():
        raise PermissionError("preview acceptance report file is missing")
    if not mp4_path.is_file():
        raise PermissionError("preview acceptance MP4 file is missing")

    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    if not isinstance(report, dict):
        raise PermissionError("preview report is invalid")
    if report.get("trajectory_sha256") != artifact.sha256:
        raise PermissionError(
            "preview report hash does not match trajectory artifact"
        )
    if report.get("preview_status") != "PASS":
        raise PermissionError("preview report is not PASS")
    return parsed
