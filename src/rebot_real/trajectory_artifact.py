from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import math
from bisect import bisect_right
from pathlib import Path
from typing import Any

import yaml

from .state_capture import JOINT_COUNT


SCHEMA_VERSION = "rebot_replay_trajectory_v1"
TRAJECTORY_REPLAY_MODE = "actual_time_quintic_v1"
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

    @property
    def q_end(self) -> tuple[float, ...]:
        return self.samples[-1].q_ref


@dataclass(frozen=True)
class ResampledReplayTarget:
    """One point on the C2 quintic path represented by a frozen artifact."""

    trajectory_time_s: float
    interval_index: int
    interval_ratio: float
    q_ref: tuple[float, ...]
    qd_ref: tuple[float, ...]
    qdd_ref: tuple[float, ...]
    jerk_ref: tuple[float, ...]


def resample_actual_time_quintic(
    artifact: ReplayArtifact,
    trajectory_time_s: float,
) -> ResampledReplayTarget:
    """Evaluate the approved path using endpoint q/qd/qdd quintic Hermite data.

    Exact artifact timestamps return the stored q/qd/qdd values bit-for-bit.  The
    caller must not use this function to extrapolate beyond the approved path.
    """

    query = float(trajectory_time_s)
    if not math.isfinite(query):
        raise ValueError("trajectory_time_s must be finite")
    tolerance = max(1e-12, artifact.duration_s * 1e-14)
    if query < -tolerance or query > artifact.duration_s + tolerance:
        raise ValueError("trajectory_time_s is outside the frozen artifact")
    query = min(artifact.duration_s, max(0.0, query))
    times = tuple(sample.time for sample in artifact.samples)

    knot = bisect_right(times, query) - 1
    if knot >= 0 and abs(query - times[knot]) <= tolerance:
        sample = artifact.samples[knot]
        interval = min(knot, len(artifact.samples) - 2)
        ratio = 1.0 if knot == len(artifact.samples) - 1 else 0.0
        jerk = _quintic_interval_values(artifact.samples[interval], artifact.samples[interval + 1], ratio)[3]
        return ResampledReplayTarget(
            trajectory_time_s=sample.time,
            interval_index=interval,
            interval_ratio=ratio,
            q_ref=sample.q_ref,
            qd_ref=sample.qd_ref,
            qdd_ref=sample.qdd_ref,
            jerk_ref=jerk,
        )

    interval = min(max(knot, 0), len(artifact.samples) - 2)
    left = artifact.samples[interval]
    right = artifact.samples[interval + 1]
    ratio = (query - left.time) / (right.time - left.time)
    q, qd, qdd, jerk = _quintic_interval_values(left, right, ratio)
    return ResampledReplayTarget(
        trajectory_time_s=query,
        interval_index=interval,
        interval_ratio=ratio,
        q_ref=q,
        qd_ref=qd,
        qdd_ref=qdd,
        jerk_ref=jerk,
    )


def _quintic_interval_values(
    left: ReplaySample,
    right: ReplaySample,
    ratio: float,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Return q, qd, qdd and jerk for one normalized quintic interval."""

    s = float(ratio)
    if not math.isfinite(s) or s < -1e-12 or s > 1.0 + 1e-12:
        raise ValueError("quintic interval ratio must be in [0, 1]")
    s = min(1.0, max(0.0, s))
    h = right.time - left.time
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("quintic interval duration must be positive and finite")
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    jerks: list[float] = []
    for joint in range(JOINT_COUNT):
        q0, q1 = left.q_ref[joint], right.q_ref[joint]
        v0, v1 = left.qd_ref[joint], right.qd_ref[joint]
        a0, a1 = left.qdd_ref[joint], right.qdd_ref[joint]
        delta = q1 - q0
        coefficients = (
            q0,
            h * v0,
            0.5 * h * h * a0,
            10.0 * delta - h * (6.0 * v0 + 4.0 * v1) - h * h * (1.5 * a0 - 0.5 * a1),
            -15.0 * delta + h * (8.0 * v0 + 7.0 * v1) + h * h * (1.5 * a0 - a1),
            6.0 * delta - 3.0 * h * (v0 + v1) - 0.5 * h * h * (a0 - a1),
        )
        c0, c1, c2, c3, c4, c5 = coefficients
        positions.append(c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5)))))
        velocities.append((c1 + s * (2.0 * c2 + s * (3.0 * c3 + s * (4.0 * c4 + s * 5.0 * c5)))) / h)
        accelerations.append((2.0 * c2 + s * (6.0 * c3 + s * (12.0 * c4 + s * 20.0 * c5))) / (h * h))
        jerks.append((6.0 * c3 + s * (24.0 * c4 + s * 60.0 * c5)) / (h * h * h))
    return tuple(positions), tuple(velocities), tuple(accelerations), tuple(jerks)


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
    """Qualify the knots and a dense evaluation of the approved quintic path."""

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
    replay_mode = config.get("trajectory_replay_mode", TRAJECTORY_REPLAY_MODE)
    if replay_mode != TRAJECTORY_REPLAY_MODE:
        failures.append(
            f"trajectory_replay_mode={replay_mode!r} is not {TRAJECTORY_REPLAY_MODE}"
        )
    density = int(config.get("continuous_check_subdivisions_per_interval", 10))
    if density < 2:
        raise ValueError("continuous_check_subdivisions_per_interval must be at least 2")
    dense_targets = [
        resample_actual_time_quintic(
            artifact,
            left.time
            + (right.time - left.time) * subdivision / density,
        )
        for left, right in zip(artifact.samples, artifact.samples[1:])
        for subdivision in range(density)
    ]
    dense_targets.append(resample_actual_time_quintic(artifact, artifact.duration_s))
    expected_dense_count = (len(artifact.samples) - 1) * density + 1
    collision_status = artifact.metadata.get("continuous_quintic_collision_precheck")
    collision_count = int(
        artifact.metadata.get("continuous_quintic_collision_precheck_sample_count", -1)
    )
    collision_density = int(
        artifact.metadata.get("continuous_quintic_collision_subdivisions_per_interval", -1)
    )
    if collision_status != "PASS":
        failures.append("continuous quintic collision precheck is not PASS")
    if collision_density < density or collision_count < expected_dense_count:
        failures.append(
            "continuous quintic collision precheck density/count is insufficient"
        )
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
        q_values = [sample.q_ref[joint] for sample in dense_targets]
        qd_values = [sample.qd_ref[joint] for sample in dense_targets]
        qdd_values = [sample.qdd_ref[joint] for sample in dense_targets]
        jerk_values = [sample.jerk_ref[joint] for sample in dense_targets]
        knot_q_values = [sample.q_ref[joint] for sample in artifact.samples]
        target_delta_values = [
            abs(current - previous)
            for previous, current in zip(knot_q_values, knot_q_values[1:])
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
            "start_position": knot_q_values[0],
            "end_position": knot_q_values[-1],
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
        if abs(knot_q_values[0] - expected_start[joint]) > start_position_tol:
            failures.append(f"J{joint + 1} start position mismatch")
        if abs(qd_values[0]) > start_velocity_tol:
            failures.append(f"J{joint + 1} start velocity not near zero")
        if abs(qdd_values[0]) > start_acceleration_tol:
            failures.append(f"J{joint + 1} start acceleration not near zero")
        if require_end_match:
            if abs(knot_q_values[-1] - knot_q_values[0]) > endpoint_position_tol:
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

    timing_profiles = [
        evaluate_actual_time_timing_profile(
            artifact,
            config,
            [1.0 / artifact.sample_rate_hz],
            profile_name="nominal",
        ),
        evaluate_actual_time_timing_profile(
            artifact,
            config,
            [0.010, 0.013],
            profile_name="alternating_10_13_ms",
        ),
    ]
    measured_timing_path = config.get("measured_timing_csv")
    if measured_timing_path:
        timing_path = Path(str(measured_timing_path))
        with timing_path.open("r", encoding="utf-8", newline="") as stream:
            timing_rows = list(csv.DictReader(stream))
        measured_intervals = [
            int(row["actual_dispatch_interval_ns"]) * 1e-9
            for row in timing_rows
            if row.get("control_mode") == "excitation"
            and row.get("command_valid") == "1"
            and row.get("actual_dispatch_interval_ns") not in (None, "")
        ]
        measured_report = evaluate_actual_time_timing_profile(
            artifact,
            config,
            measured_intervals,
            profile_name="A_run02_measured_dispatch_intervals",
            repeat_profile_to_endpoint=False,
        )
        measured_report["source_csv"] = str(timing_path)
        measured_report["source_sha256"] = sha256_file(timing_path)
        timing_profiles.append(measured_report)
    for timing_report in timing_profiles:
        if timing_report["status"] != "PASS":
            failures.append(
                f"timing replay {timing_report['profile']} failed: "
                f"{timing_report['failure']}"
            )

    return {
        "schema_version": "rebot_trajectory_preview_report_v2",
        "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
        "trajectory_sha256": artifact.sha256,
        "artifact_schema_version": SCHEMA_VERSION,
        "preview_status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "sample_count": len(artifact.samples),
        "duration_s": artifact.duration_s,
        "sample_rate_hz": artifact.sample_rate_hz,
        "continuous_check_subdivisions_per_interval": density,
        "continuous_path_sample_count": len(dense_targets),
        "continuous_quintic_collision_precheck": collision_status,
        "continuous_quintic_collision_precheck_sample_count": collision_count,
        "timing_replay_reports": timing_profiles,
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
        "jerk_method": "analytic_third_derivative_of_piecewise_quintic",
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
    """Fail closed on the entire approved quintic path before creating a client."""

    lower = _six_limit(config, "joint_position_min_rad")
    upper = _six_limit(config, "joint_position_max_rad")
    max_qd = _six_limit(config, "maximum_command_velocity_rad_s")
    max_qdd = _six_limit(config, "maximum_command_acceleration_rad_s2")
    max_jerk = _six_limit(config, "maximum_command_jerk_rad_s3")
    max_target_delta = float(config["maximum_servo_target_delta_rad"])
    if not math.isfinite(max_target_delta) or max_target_delta <= 0.0:
        raise ValueError("maximum_servo_target_delta_rad must be positive and finite")
    if config.get("trajectory_replay_mode") != TRAJECTORY_REPLAY_MODE:
        raise ValueError(
            f"trajectory_replay_mode must be {TRAJECTORY_REPLAY_MODE}"
        )
    density = int(config.get("continuous_check_subdivisions_per_interval", 10))
    if density < 2:
        raise ValueError("continuous_check_subdivisions_per_interval must be at least 2")
    if artifact.metadata.get("continuous_quintic_collision_precheck") != "PASS":
        raise ValueError("artifact lacks a PASS continuous quintic collision precheck")
    expected_collision_count = (len(artifact.samples) - 1) * density + 1
    if (
        int(artifact.metadata.get("continuous_quintic_collision_subdivisions_per_interval", -1))
        < density
        or int(artifact.metadata.get("continuous_quintic_collision_precheck_sample_count", -1))
        < expected_collision_count
    ):
        raise ValueError("artifact continuous quintic collision precheck is too sparse")

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

    max_seen_jerk = [0.0] * JOINT_COUNT
    max_seen_target_delta = [0.0] * JOINT_COUNT
    max_seen_target_delta_sample_index = [0] * JOINT_COUNT
    previous_q = artifact.q_start
    for sample_index, sample in enumerate(artifact.samples):
        for joint in range(JOINT_COUNT):
            if sample.q_ref[joint] < lower[joint] or sample.q_ref[joint] > upper[joint]:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} violates position limits"
                )
            if abs(sample.qd_ref[joint]) > max_qd[joint] + 1e-12:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} violates velocity limit"
                )
            if abs(sample.qdd_ref[joint]) > max_qdd[joint] + 1e-12:
                raise ValueError(
                    f"artifact sample {sample_index} J{joint + 1} violates acceleration limit"
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
    dense_targets = [
        resample_actual_time_quintic(
            artifact,
            left.time + (right.time - left.time) * subdivision / density,
        )
        for left, right in zip(artifact.samples, artifact.samples[1:])
        for subdivision in range(density)
    ]
    dense_targets.append(resample_actual_time_quintic(artifact, artifact.duration_s))
    for sample_index, sample in enumerate(dense_targets):
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
            jerk = abs(sample.jerk_ref[joint])
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
        "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
        "continuous_path_sample_count": len(dense_targets),
        "jerk_method": "analytic_third_derivative_of_piecewise_quintic",
        "jerk_abs_max": max_seen_jerk,
        "servo_target_delta_abs_max": max_seen_target_delta,
        "servo_target_delta_abs_max_sample_index": max_seen_target_delta_sample_index,
        "servo_target_delta_margin_rad": [
            max_target_delta - value for value in max_seen_target_delta
        ],
        "maximum_servo_target_delta_rad": max_target_delta,
    }


def evaluate_actual_time_timing_profile(
    artifact: ReplayArtifact,
    config: dict[str, Any],
    dispatch_intervals_s: list[float] | tuple[float, ...],
    *,
    profile_name: str,
    repeat_profile_to_endpoint: bool = True,
) -> dict[str, Any]:
    """Offline replay of the same packet finite-difference envelope used at runtime."""

    intervals = tuple(float(value) for value in dispatch_intervals_s)
    if not intervals or not all(math.isfinite(value) and value > 0.0 for value in intervals):
        raise ValueError("dispatch timing profile requires positive finite intervals")
    lower = _six_limit(config, "joint_position_min_rad")
    upper = _six_limit(config, "joint_position_max_rad")
    velocity_limit = _six_limit(config, "maximum_command_velocity_rad_s")
    acceleration_limit = _six_limit(config, "maximum_command_acceleration_rad_s2")
    jerk_limit = _six_limit(config, "maximum_command_jerk_rad_s3")
    delta_limit = float(config["maximum_servo_target_delta_rad"])
    minimum_dt = float(config.get("minimum_servo_timestamp_interval_s", 0.0005))
    maximum_dt = float(config.get("maximum_servo_timestamp_interval_s", 0.1))
    maxima = {name: [0.0] * JOINT_COUNT for name in ("delta_q", "qd", "qdd", "jerk")}
    previous_q = artifact.q_start
    previous_qd = (0.0,) * JOINT_COUNT
    previous_qdd = (0.0,) * JOINT_COUNT
    trajectory_time = 0.0
    command_count = 0
    interval_cursor = 0
    failure: dict[str, Any] | None = None
    while True:
        dt = (
            intervals[interval_cursor % len(intervals)]
            if repeat_profile_to_endpoint or interval_cursor < len(intervals)
            else 1.0 / artifact.sample_rate_hz
        )
        interval_cursor += 1
        target = resample_actual_time_quintic(artifact, trajectory_time)
        delta = tuple(target.q_ref[j] - previous_q[j] for j in range(JOINT_COUNT))
        qd = tuple(value / dt for value in delta)
        qdd = tuple((qd[j] - previous_qd[j]) / dt for j in range(JOINT_COUNT))
        jerk = tuple((qdd[j] - previous_qdd[j]) / dt for j in range(JOINT_COUNT))
        if dt < minimum_dt or dt > maximum_dt:
            failure = {
                "quantity": "timestamp_interval_s",
                "joint": None,
                "value": dt,
                "limit": minimum_dt if dt < minimum_dt else maximum_dt,
                "trajectory_time_s": trajectory_time,
            }
        for joint in range(JOINT_COUNT):
            for name, values in (("delta_q", delta), ("qd", qd), ("qdd", qdd), ("jerk", jerk)):
                maxima[name][joint] = max(maxima[name][joint], abs(values[joint]))
            checks = (
                ("position_rad", target.q_ref[joint], lower[joint], upper[joint]),
                ("delta_q_rad", abs(delta[joint]), 0.0, delta_limit),
                ("qd_rad_s", abs(qd[joint]), 0.0, velocity_limit[joint]),
                ("qdd_rad_s2", abs(qdd[joint]), 0.0, acceleration_limit[joint]),
                ("jerk_rad_s3", abs(jerk[joint]), 0.0, jerk_limit[joint]),
            )
            for quantity, value, minimum, maximum in checks:
                if value < minimum - 1e-12 or value > maximum + 1e-9:
                    failure = {
                        "quantity": quantity,
                        "joint": joint + 1,
                        "value": value,
                        "limit": minimum if value < minimum else maximum,
                        "dt_s": dt,
                        "trajectory_time_s": trajectory_time,
                    }
                    break
            if failure is not None:
                break
        if failure is not None:
            break
        previous_q, previous_qd, previous_qdd = target.q_ref, qd, qdd
        command_count += 1
        if trajectory_time >= artifact.duration_s:
            break
        next_interval = (
            intervals[interval_cursor % len(intervals)]
            if repeat_profile_to_endpoint or interval_cursor < len(intervals)
            else 1.0 / artifact.sample_rate_hz
        )
        trajectory_time = min(
            artifact.duration_s,
            trajectory_time + next_interval,
        )

    per_joint = {}
    for joint in range(JOINT_COUNT):
        per_joint[f"J{joint + 1}"] = {
            "delta_q_abs_max_rad": maxima["delta_q"][joint],
            "qd_abs_max_rad_s": maxima["qd"][joint],
            "qdd_abs_max_rad_s2": maxima["qdd"][joint],
            "jerk_abs_max_rad_s3": maxima["jerk"][joint],
            "delta_q_margin_rad": delta_limit - maxima["delta_q"][joint],
            "qd_margin_rad_s": velocity_limit[joint] - maxima["qd"][joint],
            "qdd_margin_rad_s2": acceleration_limit[joint] - maxima["qdd"][joint],
            "jerk_margin_rad_s3": jerk_limit[joint] - maxima["jerk"][joint],
        }
    return {
        "profile": profile_name,
        "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
        "status": "PASS" if failure is None else "FAIL",
        "failure": failure,
        "timing_sample_count": len(intervals),
        "timing_repeated_to_endpoint": repeat_profile_to_endpoint,
        "dispatch_interval_min_s": min(intervals),
        "dispatch_interval_max_s": max(intervals),
        "actual_command_count": command_count,
        "trajectory_time_end_s": trajectory_time,
        "per_joint": per_joint,
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
    preview_mp4 = Path(preview_mp4)
    if not preview_mp4.is_file():
        raise FileNotFoundError(
            "preview MP4 must be rendered before writing v2 acceptance"
        )
    report_path.write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    acceptance = {
        "schema_version": "rebot_trajectory_preview_acceptance_v2",
        "trajectory_sha256": report["trajectory_sha256"],
        "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
        "trajectory_metadata_sha256": report.get("trajectory_metadata_sha256"),
        "git_head_at_preview": report.get("git_head_at_preview"),
        "model_hash": report.get("model_hash"),
        "limits_config_hash": report.get("limits_config_hash"),
        "qualification_config_sha256": report.get("qualification_config_sha256"),
        "preview_visualization": report.get("preview_visualization"),
        "preview_report": str(report_path),
        "preview_report_sha256": sha256_file(report_path),
        "preview_mp4": str(preview_mp4),
        "preview_mp4_sha256": sha256_file(preview_mp4),
        "operator": "",
        "review_date": None,
        "accepted_for_hardware": False,
        "notes": (
            (
                "Default is false. A human operator must watch the continuous-quintic "
                "MP4 and manually accept this artifact, strategy, report and MP4 hash."
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
    replay_mode: str = TRAJECTORY_REPLAY_MODE,
) -> dict[str, Any]:
    path = Path(acceptance_path)
    if not path.is_file():
        raise PermissionError("preview acceptance file is missing")
    parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError("preview acceptance must be a YAML mapping")
    if parsed.get("schema_version") != "rebot_trajectory_preview_acceptance_v2":
        raise ValueError(
            "preview acceptance v2 is required; v1 cannot authorize actual-time replay"
        )
    if parsed.get("trajectory_sha256") != artifact.sha256:
        raise PermissionError(
            "preview acceptance hash does not match trajectory artifact"
        )
    if parsed.get("trajectory_replay_mode") != replay_mode:
        raise PermissionError("preview acceptance replay strategy mismatch")
    if require_hardware_acceptance and parsed.get("accepted_for_hardware") is not True:
        raise PermissionError(
            "preview acceptance is not accepted_for_hardware=true"
        )
    if require_hardware_acceptance:
        if not str(parsed.get("operator", "")).strip() or not parsed.get("review_date"):
            raise PermissionError(
                "preview acceptance lacks human operator or review_date"
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
    if parsed.get("preview_report_sha256") != sha256_file(report_path):
        raise PermissionError("preview acceptance report hash mismatch")
    if parsed.get("preview_mp4_sha256") != sha256_file(mp4_path):
        raise PermissionError("preview acceptance MP4 hash mismatch")

    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    if not isinstance(report, dict):
        raise PermissionError("preview report is invalid")
    if report.get("trajectory_sha256") != artifact.sha256:
        raise PermissionError(
            "preview report hash does not match trajectory artifact"
        )
    if report.get("schema_version") != "rebot_trajectory_preview_report_v2":
        raise PermissionError("preview report v2 is required")
    if report.get("trajectory_replay_mode") != replay_mode:
        raise PermissionError("preview report replay strategy mismatch")
    if report.get("continuous_quintic_collision_precheck") != "PASS":
        raise PermissionError("preview report continuous collision precheck is not PASS")
    if report.get("preview_status") != "PASS":
        raise PermissionError("preview report is not PASS")
    for key in (
        "trajectory_metadata_sha256",
        "git_head_at_preview",
        "model_hash",
        "limits_config_hash",
        "qualification_config_sha256",
        "preview_visualization",
    ):
        if key in parsed and parsed.get(key) != report.get(key):
            raise PermissionError(f"preview acceptance {key} does not match report")
    return parsed
