from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import yaml

from .control_adapter import RebotControlAdapter, RebotControlError
from .hardware_recorder import HardwareExperimentRecorder
from .joint_jog import jog_target_at, validate_joint_jog
from .state_capture import CaptureSample, JOINT_COUNT
from .trajectory_artifact import (
    TRAJECTORY_REPLAY_MODE,
    ReplayArtifact,
    ResampledReplayTarget,
    load_replay_artifact,
    resample_actual_time_quintic,
    validate_preview_acceptance,
    validate_replay_runtime_limits,
)


CONTROL_MODES = {"state_only", "servo_hold", "joint_jog", "excitation"}


@dataclass(frozen=True)
class ServoEnvelopePrediction:
    timestamp_ns: int
    dt_s: float
    q: tuple[float, ...]
    delta_q: tuple[float, ...]
    qd: tuple[float, ...]
    qdd: tuple[float, ...]
    jerk: tuple[float, ...]


class ServoEnvelopeViolation(RebotControlError):
    """A candidate packet that must not be passed to ArmClient."""

    def __init__(self, *, quantity: str, joint: int | None, value: float,
                 limit: float, prediction: ServoEnvelopePrediction) -> None:
        label = "timestamp interval" if joint is None else f"J{joint + 1} {quantity}"
        super().__init__(
            f"local Servo envelope {label}={value:.9g} violates limit {limit:.9g}"
        )
        self.quantity = quantity
        self.joint = joint
        self.value = float(value)
        self.limit = float(limit)
        self.prediction = prediction


class ActualTimeServoEnvelopeTracker:
    """Predict packet derivatives from accepted targets and real timestamps."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.lower = tuple(float(value) for value in config["joint_position_min_rad"])
        self.upper = tuple(float(value) for value in config["joint_position_max_rad"])
        self.velocity_limit = tuple(float(value) for value in config["maximum_command_velocity_rad_s"])
        self.acceleration_limit = tuple(float(value) for value in config["maximum_command_acceleration_rad_s2"])
        self.jerk_limit = tuple(float(value) for value in config["maximum_command_jerk_rad_s3"])
        self.delta_limit = float(config["maximum_servo_target_delta_rad"])
        self.minimum_dt_s = float(config["minimum_servo_timestamp_interval_s"])
        self.maximum_dt_s = float(config["maximum_servo_timestamp_interval_s"])
        self.previous_q: tuple[float, ...] | None = None
        self.previous_qd = (0.0,) * JOINT_COUNT
        self.previous_qdd = (0.0,) * JOINT_COUNT
        self.previous_timestamp_ns: int | None = None
        self.accepted_count = 0
        self.max_abs_delta_q = [0.0] * JOINT_COUNT
        self.max_abs_qd = [0.0] * JOINT_COUNT
        self.max_abs_qdd = [0.0] * JOINT_COUNT
        self.max_abs_jerk = [0.0] * JOINT_COUNT

    def initialize(self, q: Sequence[float], timestamp_ns: int) -> None:
        if self.previous_q is not None:
            raise RuntimeError("Servo envelope tracker is already initialized")
        self.previous_q = _six_finite(q, "accepted initial Servo target")
        self.previous_timestamp_ns = int(timestamp_ns)
        if self.previous_timestamp_ns <= 0:
            raise ValueError("accepted initial Servo timestamp must be positive")

    def predict(self, q: Sequence[float], timestamp_ns: int) -> ServoEnvelopePrediction:
        if self.previous_q is None or self.previous_timestamp_ns is None:
            raise RuntimeError("Servo envelope tracker must be initialized")
        target = _six_finite(q, "candidate Servo target")
        stamp = int(timestamp_ns)
        dt_s = (stamp - self.previous_timestamp_ns) * 1e-9
        delta = tuple(target[j] - self.previous_q[j] for j in range(JOINT_COUNT))
        if dt_s > 0.0:
            qd = tuple(value / dt_s for value in delta)
            qdd = tuple((qd[j] - self.previous_qd[j]) / dt_s for j in range(JOINT_COUNT))
            jerk = tuple((qdd[j] - self.previous_qdd[j]) / dt_s for j in range(JOINT_COUNT))
        else:
            qd = qdd = jerk = (math.inf,) * JOINT_COUNT
        prediction = ServoEnvelopePrediction(stamp, dt_s, target, delta, qd, qdd, jerk)
        if dt_s < self.minimum_dt_s - 1e-12:
            raise ServoEnvelopeViolation(quantity="dt_min_s", joint=None, value=dt_s,
                                         limit=self.minimum_dt_s, prediction=prediction)
        if dt_s > self.maximum_dt_s + 1e-12:
            raise ServoEnvelopeViolation(quantity="dt_max_s", joint=None, value=dt_s,
                                         limit=self.maximum_dt_s, prediction=prediction)
        for joint in range(JOINT_COUNT):
            if target[joint] < self.lower[joint] or target[joint] > self.upper[joint]:
                limit = self.lower[joint] if target[joint] < self.lower[joint] else self.upper[joint]
                raise ServoEnvelopeViolation(quantity="position_rad", joint=joint,
                                             value=target[joint], limit=limit,
                                             prediction=prediction)
            checks = (
                ("delta_q_rad", abs(delta[joint]), self.delta_limit),
                ("qd_rad_s", abs(qd[joint]), self.velocity_limit[joint]),
                ("qdd_rad_s2", abs(qdd[joint]), self.acceleration_limit[joint]),
                ("jerk_rad_s3", abs(jerk[joint]), self.jerk_limit[joint]),
            )
            for quantity, value, limit in checks:
                if value > limit + (1e-9 if quantity == "jerk_rad_s3" else 1e-12):
                    raise ServoEnvelopeViolation(quantity=quantity, joint=joint,
                                                 value=value, limit=limit,
                                                 prediction=prediction)
        return prediction

    def commit(self, prediction: ServoEnvelopePrediction) -> None:
        if prediction.timestamp_ns == self.previous_timestamp_ns:
            raise RuntimeError("cannot commit a duplicate Servo timestamp")
        self.previous_q = prediction.q
        self.previous_qd = prediction.qd
        self.previous_qdd = prediction.qdd
        self.previous_timestamp_ns = prediction.timestamp_ns
        self.accepted_count += 1
        for target, values in (
            (self.max_abs_delta_q, prediction.delta_q),
            (self.max_abs_qd, prediction.qd),
            (self.max_abs_qdd, prediction.qdd),
            (self.max_abs_jerk, prediction.jerk),
        ):
            for joint, value in enumerate(values):
                target[joint] = max(target[joint], abs(value))

    def summary(self) -> dict[str, Any]:
        per_joint: dict[str, Any] = {}
        for joint in range(JOINT_COUNT):
            maxima = {
                "delta_q_abs_max_rad": self.max_abs_delta_q[joint],
                "qd_abs_max_rad_s": self.max_abs_qd[joint],
                "qdd_abs_max_rad_s2": self.max_abs_qdd[joint],
                "jerk_abs_max_rad_s3": self.max_abs_jerk[joint],
            }
            per_joint[f"J{joint + 1}"] = {
                **maxima,
                "delta_q_margin_rad": self.delta_limit - maxima["delta_q_abs_max_rad"],
                "qd_margin_rad_s": self.velocity_limit[joint] - maxima["qd_abs_max_rad_s"],
                "qdd_margin_rad_s2": self.acceleration_limit[joint] - maxima["qdd_abs_max_rad_s2"],
                "jerk_margin_rad_s3": self.jerk_limit[joint] - maxima["jerk_abs_max_rad_s3"],
            }
        return {
            "strategy": TRAJECTORY_REPLAY_MODE,
            "derivative_semantics": "finite_difference_of_sdk_accepted_targets_using_real_host_timestamps",
            "accepted_excitation_command_count": self.accepted_count,
            "minimum_servo_timestamp_interval_s": self.minimum_dt_s,
            "maximum_servo_timestamp_interval_s": self.maximum_dt_s,
            "per_joint": per_joint,
        }


def load_hardware_config(
    config_path: str | Path,
    *,
    repo_root: str | Path | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load and strictly validate the small reBot hardware experiment YAML."""

    path = Path(config_path)
    parsed = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(parsed, dict):
        raise ValueError("reBot hardware config must be a YAML mapping")
    config = {
        "sdk_root": "",
        "host": "127.0.0.1",
        "tcp_port": 5000,
        "udp_port": 5001,
        "control_mode": "state_only",
        "control_rate_hz": 100.0,
        "duration_s": 0.1,
        "max_samples": None,
        "output_csv": "data/rebot_real/hardware_experiment.csv",
        "allow_hardware": False,
        "allow_motion": False,
        "joint_mapping_verified": False,
        "j1_convention": "UNRESOLVED",
        "joint_mapping_scope": "servo_hold_only",
        "joint_jog": None,
        "joint_direction": [1.0] * JOINT_COUNT,
        "joint_offset_rad": [0.0] * JOINT_COUNT,
        "joint_position_min_rad": [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14],
        "joint_position_max_rad": [2.8, 0.0, 0.0, 1.57, 1.57, 3.14],
        "maximum_command_velocity_rad_s": [0.05] * JOINT_COUNT,
        "maximum_command_acceleration_rad_s2": [2.0] * JOINT_COUNT,
        "maximum_command_jerk_rad_s3": [10.0] * JOINT_COUNT,
        "maximum_tracking_error_rad": [0.01] * JOINT_COUNT,
        # Lower ServoCore fixed one-packet target jump gate:
        # 0.5 rad/s * 5 ms * 1.25 = 0.003125 rad.
        "maximum_servo_target_delta_rad": 0.003125,
        # ServoCore host timestamp acceptance interval (fault 500117).
        "minimum_servo_timestamp_interval_s": 0.0005,
        "maximum_servo_timestamp_interval_s": 0.1,
        # Snapshot of the external SDK's current high-level MoveJ runtime policy.
        # These bounds are only for preposition; excitation limits remain separate.
        "movej_max_velocity_rad_s": [
            math.radians(value) for value in (18.0, 16.0, 15.0, 18.0, 15.0, 15.0)
        ],
        "movej_max_acceleration_rad_s2": [
            math.radians(value) for value in (39.99, 31.99, 39.99, 47.99, 47.99, 59.99)
        ],
        "movej_max_jerk_rad_s3": [math.radians(600.0)] * JOINT_COUNT,
        "movej_timeout_s": 60.0,
        "controlled_park_before_disable": False,
        "start_position_tolerance_rad": 0.01,
        "preposition_settle_velocity_tolerance_rad_s": 0.01,
        # Legacy compatibility alias. New configs should use
        # motion_ready_feedback_max_age_ms for pre-motion freshness only.
        "maximum_feedback_age_ms": 50.0,
        "motion_ready_feedback_max_age_ms": None,
        "maximum_disabled_feedback_age_ms": None,
        # Mirror the audited Lower production feedback policy without changing it.
        "lower_feedback_timeout_ms": 250.0,
        "transient_feedback_invalid_recovery_ms": 100.0,
        "connect_timeout_s": 3.0,
        "command_timeout_s": 3.0,
        # Blocking read timeout and latest-snapshot host-age timeout are distinct.
        "state_timeout_s": 0.25,
        "host_state_snapshot_timeout_s": None,
        "trajectory_source": "frozen_replay_artifact",
        "trajectory_replay_mode": None,
        "trajectory_hash": None,
        "trajectory_artifact": "results/rebot_trajectory_preview/trajectory_preview.csv",
        "trajectory_metadata": "results/rebot_trajectory_preview/trajectory_preview.meta.yaml",
        "trajectory_preview_acceptance": "results/rebot_trajectory_preview/preview_acceptance.yaml",
    }
    unknown = set(parsed) - set(config)
    if unknown:
        raise ValueError(f"unknown reBot hardware config keys: {sorted(unknown)}")
    config.update(parsed)
    if overrides:
        unknown_overrides = set(overrides) - set(config)
        if unknown_overrides:
            raise ValueError(f"unknown reBot hardware override keys: {sorted(unknown_overrides)}")
        config.update({key: value for key, value in overrides.items() if value is not None})

    config["sdk_root"] = str(config["sdk_root"]).strip()
    config["host"] = str(config["host"]).strip()
    if not config["host"]:
        raise ValueError("host must not be empty")
    config["tcp_port"] = _port(config["tcp_port"], "tcp_port")
    config["udp_port"] = _port(config["udp_port"], "udp_port")
    config["control_mode"] = str(config["control_mode"])
    if config["control_mode"] not in CONTROL_MODES:
        raise ValueError(f"control_mode must be one of {sorted(CONTROL_MODES)}")
    config["control_rate_hz"] = _positive(config["control_rate_hz"], "control_rate_hz")
    config["duration_s"] = _positive(config["duration_s"], "duration_s")
    if config["max_samples"] is not None:
        config["max_samples"] = int(config["max_samples"])
        if config["max_samples"] <= 0:
            raise ValueError("max_samples must be positive when set")
    for name in (
        "allow_hardware",
        "allow_motion",
        "joint_mapping_verified",
        "controlled_park_before_disable",
    ):
        if not isinstance(config[name], bool):
            raise ValueError(f"{name} must be a YAML boolean")
    config["j1_convention"] = str(config["j1_convention"]).strip() or "UNRESOLVED"
    if config["joint_mapping_scope"] not in {
        "servo_hold_only", "joint_jog", "excitation_smoke", "excitation"
    }:
        raise ValueError(
            "joint_mapping_scope must be servo_hold_only, joint_jog, excitation_smoke, or excitation"
        )
    config["joint_direction"] = list(_six_finite(config["joint_direction"], "joint_direction"))
    if any(abs(value) != 1.0 for value in config["joint_direction"]):
        raise ValueError("joint_direction values must be exactly -1 or 1")
    for name in (
        "joint_offset_rad",
        "joint_position_min_rad",
        "joint_position_max_rad",
        "maximum_command_velocity_rad_s",
        "maximum_command_acceleration_rad_s2",
        "maximum_command_jerk_rad_s3",
        "maximum_tracking_error_rad",
        "movej_max_velocity_rad_s",
        "movej_max_acceleration_rad_s2",
        "movej_max_jerk_rad_s3",
    ):
        config[name] = list(_six_finite(config[name], name))
    for lower, upper in zip(config["joint_position_min_rad"], config["joint_position_max_rad"]):
        if lower >= upper:
            raise ValueError("each joint position minimum must be smaller than maximum")
    for name in (
        "maximum_command_velocity_rad_s",
        "maximum_command_acceleration_rad_s2",
        "maximum_command_jerk_rad_s3",
        "maximum_tracking_error_rad",
        "movej_max_velocity_rad_s",
        "movej_max_acceleration_rad_s2",
        "movej_max_jerk_rad_s3",
    ):
        if any(value <= 0.0 for value in config[name]):
            raise ValueError(f"{name} values must be positive")
    config["maximum_servo_target_delta_rad"] = _positive(
        config["maximum_servo_target_delta_rad"],
        "maximum_servo_target_delta_rad",
    )
    config["minimum_servo_timestamp_interval_s"] = _positive(
        config["minimum_servo_timestamp_interval_s"],
        "minimum_servo_timestamp_interval_s",
    )
    config["maximum_servo_timestamp_interval_s"] = _positive(
        config["maximum_servo_timestamp_interval_s"],
        "maximum_servo_timestamp_interval_s",
    )
    if (
        config["minimum_servo_timestamp_interval_s"]
        >= config["maximum_servo_timestamp_interval_s"]
    ):
        raise ValueError("minimum Servo timestamp interval must be below maximum")
    config["movej_timeout_s"] = _positive(config["movej_timeout_s"], "movej_timeout_s")
    config["start_position_tolerance_rad"] = _positive(
        config["start_position_tolerance_rad"],
        "start_position_tolerance_rad",
    )
    config["preposition_settle_velocity_tolerance_rad_s"] = _positive(
        config["preposition_settle_velocity_tolerance_rad_s"],
        "preposition_settle_velocity_tolerance_rad_s",
    )
    for name in ("maximum_feedback_age_ms", "connect_timeout_s", "command_timeout_s", "state_timeout_s"):
        config[name] = _positive(config[name], name)
    motion_ready_age = config["motion_ready_feedback_max_age_ms"]
    config["motion_ready_feedback_max_age_ms"] = (
        config["maximum_feedback_age_ms"]
        if motion_ready_age is None
        else _positive(motion_ready_age, "motion_ready_feedback_max_age_ms")
    )
    if "motion_ready_feedback_max_age_ms" in parsed and "maximum_feedback_age_ms" not in parsed:
        config["maximum_feedback_age_ms"] = config["motion_ready_feedback_max_age_ms"]
    if (
        "maximum_feedback_age_ms" in parsed
        and "motion_ready_feedback_max_age_ms" in parsed
        and not math.isclose(
            float(parsed["maximum_feedback_age_ms"]),
            float(parsed["motion_ready_feedback_max_age_ms"]),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise ValueError(
            "maximum_feedback_age_ms is a deprecated alias for "
            "motion_ready_feedback_max_age_ms; conflicting values are not allowed"
        )
    disabled_age = config["maximum_disabled_feedback_age_ms"]
    config["maximum_disabled_feedback_age_ms"] = (
        config["motion_ready_feedback_max_age_ms"]
        if disabled_age is None
        else _positive(disabled_age, "maximum_disabled_feedback_age_ms")
    )
    config["lower_feedback_timeout_ms"] = _positive(
        config["lower_feedback_timeout_ms"], "lower_feedback_timeout_ms"
    )
    config["transient_feedback_invalid_recovery_ms"] = _positive(
        config["transient_feedback_invalid_recovery_ms"],
        "transient_feedback_invalid_recovery_ms",
    )
    host_snapshot_timeout = config["host_state_snapshot_timeout_s"]
    config["host_state_snapshot_timeout_s"] = (
        config["state_timeout_s"]
        if host_snapshot_timeout is None
        else _positive(host_snapshot_timeout, "host_state_snapshot_timeout_s")
    )
    if config["motion_ready_feedback_max_age_ms"] > config["lower_feedback_timeout_ms"]:
        raise ValueError(
            "motion_ready_feedback_max_age_ms must not exceed lower_feedback_timeout_ms"
        )

    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[2]
    output = Path(str(config["output_csv"])).expanduser()
    if not output.is_absolute():
        output = root / output
    config["output_csv"] = str(output)
    config["trajectory_source"] = str(config["trajectory_source"])
    if config["trajectory_replay_mode"] is not None:
        config["trajectory_replay_mode"] = str(config["trajectory_replay_mode"])
    for name in (
        "trajectory_artifact",
        "trajectory_metadata",
        "trajectory_preview_acceptance",
    ):
        evidence_path = Path(str(config[name])).expanduser()
        if not evidence_path.is_absolute():
            evidence_path = root / evidence_path
        config[name] = str(evidence_path)
    if config["control_mode"] == "joint_jog":
        validate_joint_jog(config)
    if (
        config["control_mode"] == "excitation"
        and config["trajectory_replay_mode"] != TRAJECTORY_REPLAY_MODE
    ):
        raise ValueError(
            f"excitation requires trajectory_replay_mode={TRAJECTORY_REPLAY_MODE}"
        )
    if config["control_mode"] == "excitation" and config["allow_hardware"]:
        if "controlled_park_before_disable" not in parsed:
            raise ValueError(
                "real excitation requires an explicit controlled_park_before_disable policy"
            )
        if config["controlled_park_before_disable"] is not True:
            raise ValueError(
                "real excitation requires controlled_park_before_disable=true"
            )
    return config


class RebotHardwareRunner:
    """Run one gated reBot hardware-control experiment or the same lifecycle on a Mock client."""

    def __init__(
        self,
        config: dict[str, Any],
        *,
        repo_root: str | Path,
        client_factory: Callable[..., Any] | None = None,
        mock_backend: bool = False,
        sleep_fn: Callable[[float], None] = time.sleep,
        monotonic_fn: Callable[[], float] = time.monotonic,
        monotonic_ns_fn: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = dict(config)
        self.repo_root = Path(repo_root)
        self.client_factory = client_factory
        self.mock_backend = bool(mock_backend)
        self.sleep_fn = sleep_fn
        self.monotonic_fn = monotonic_fn
        self.monotonic_ns_fn = monotonic_ns_fn

    def run(self) -> dict[str, Any]:
        """Execute the selected mode with deterministic best-effort safe shutdown."""

        mode = str(self.config["control_mode"])
        self._assert_session_authorized(mode)
        if mode == "joint_jog":
            output = Path(self.config["output_csv"])
            if output.exists() or output.with_suffix(".meta.yaml").exists():
                raise FileExistsError("joint_jog requires new output paths; existing evidence is not overwritten")
        replay_artifact: ReplayArtifact | None = None
        if mode == "excitation":
            if self.config.get("trajectory_source") != "frozen_replay_artifact":
                raise ValueError(
                    "excitation requires trajectory_source=frozen_replay_artifact"
                )
            if self.config.get("trajectory_replay_mode") != TRAJECTORY_REPLAY_MODE:
                raise ValueError(
                    f"excitation requires trajectory_replay_mode={TRAJECTORY_REPLAY_MODE}"
                )
            replay_artifact = load_replay_artifact(
                self.config["trajectory_artifact"],
                self.config["trajectory_metadata"],
            )
            configured_hash = self.config.get("trajectory_hash")
            if configured_hash not in (None, "", replay_artifact.sha256):
                raise ValueError(
                    "configured trajectory_hash does not match frozen artifact"
                )
            validate_replay_runtime_limits(replay_artifact, self.config)
            validate_preview_acceptance(
                self.config["trajectory_preview_acceptance"],
                replay_artifact,
                repo_root=self.repo_root,
                require_hardware_acceptance=not self.mock_backend,
                replay_mode=TRAJECTORY_REPLAY_MODE,
            )
            self.config["trajectory_hash"] = replay_artifact.sha256
            self.config["duration_s"] = replay_artifact.duration_s

        adapter = RebotControlAdapter(
            sdk_root=self.config["sdk_root"],
            host=self.config["host"],
            tcp_port=self.config["tcp_port"],
            udp_port=self.config["udp_port"],
            joint_direction=self.config["joint_direction"],
            joint_offset_rad=self.config["joint_offset_rad"],
            connect_timeout_s=self.config["connect_timeout_s"],
            command_timeout_s=self.config["command_timeout_s"],
            state_timeout_s=self.config["state_timeout_s"],
            client_factory=self.client_factory,
            sleep_fn=self.sleep_fn,
            monotonic_ns_fn=self.monotonic_ns_fn,
        )
        recorder = HardwareExperimentRecorder(
            self.config["output_csv"],
            config=self.config,
            repo_root=self.repo_root,
            backend="rebot_sdk_mock" if self.mock_backend else "rebot_sdk",
        )
        session = {"connected": False, "enabled": False, "servo": False, "motion_lifecycle": False}
        try:
            adapter.connect()
            session["connected"] = True
            if mode == "state_only":
                self._run_state_only(adapter, recorder)
            elif mode == "servo_hold":
                self._run_servo_hold(adapter, recorder, session)
            elif mode == "joint_jog":
                self._run_joint_jog(adapter, recorder, session)
            elif mode == "excitation":
                if replay_artifact is None:
                    raise AssertionError("validated excitation artifact is missing")
                self._run_excitation(adapter, recorder, session, replay_artifact)
            else:
                raise AssertionError(f"unhandled control mode {mode}")
        except BaseException as exc:
            if recorder.failure_result is None:
                recorder.failure_result = {
                    "stage": mode,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            cleanup_errors, shutdown_result = self._shutdown(
                adapter,
                session,
                error_path=True,
                cause=exc,
            )
            recorder.shutdown_result = shutdown_result
            if recorder.run_result is not None:
                recorder.run_result.update(status="aborted", error=f"{type(exc).__name__}: {exc}", cleanup_errors=cleanup_errors)
            recorder.motion_status = "aborted"
            metadata = recorder.close()
            if cleanup_errors:
                raise RebotControlError(
                    f"{exc}; cleanup failures: {'; '.join(cleanup_errors)}"
                ) from exc
            raise
        cleanup_errors, shutdown_result = self._shutdown(
            adapter,
            session,
            error_path=False,
            cause=None,
        )
        recorder.shutdown_result = shutdown_result
        if recorder.run_result is not None:
            recorder.run_result.update(status="aborted" if cleanup_errors else "completed", cleanup_errors=cleanup_errors)
        recorder.motion_status = "aborted" if cleanup_errors else "completed"
        metadata = recorder.close()
        if cleanup_errors:
            raise RebotControlError(f"cleanup failures: {'; '.join(cleanup_errors)}")
        return metadata

    def _run_state_only(
        self,
        adapter: RebotControlAdapter,
        recorder: HardwareExperimentRecorder,
    ) -> None:
        """Observe and record state; this function contains no motor-changing command call."""

        start = self.monotonic_fn()
        samples = 0
        while not self._done(start, samples):
            state = adapter.read_state()
            self._validate_state(
                state,
                require_servo_active=None,
                feedback_age_limit_key="maximum_disabled_feedback_age_ms",
            )
            recorder.record(
                state,
                q_cmd=None,
                timestamp_host_command_ns=None,
                servo_sequence=None,
                command_valid=False,
                control_mode="state_only",
            )
            samples += 1
            self._sleep_period()

    def _run_servo_hold(
        self,
        adapter: RebotControlAdapter,
        recorder: HardwareExperimentRecorder,
        session: dict[str, bool],
    ) -> None:
        """Hold the freshly re-read current position through the audited Servo lifecycle."""

        initial = adapter.read_state()
        self._validate_state(
            initial,
            require_servo_active=False,
            feedback_age_limit_key="maximum_disabled_feedback_age_ms",
        )
        # Mark intent before waiting for the TCP ACK: enable can take effect on
        # the lower controller even if the reply is lost or the wait times out.
        session["enabled"] = True
        adapter.enable()

        # TCP enable can complete before the UDP state stream has published the
        # lower FSM transition to Ready/idle. Wait for that explicit motion-ready
        # state before claiming Servo ownership, matching the SDK high-level policy.
        hold_state = self._wait_until_motion_ready(adapter)
        hold_target = tuple(hold_state.q)
        state, timestamp_ns, sequence = self._enter_servo_with_initial_hold(
            adapter, session, hold_target
        )
        start = self.monotonic_fn()
        samples = 0
        next_deadline = start
        while not self._done(start, samples):
            if samples > 0:
                state = adapter.read_state()
                self._validate_state(state, require_servo_active=True)
                next_deadline = self._pace_deadline(next_deadline)
                self._validate_target_step(hold_target, hold_target)
                self._validate_tracking_error(hold_target, state.q)
                timestamp_ns, sequence = adapter.send_servo_target(hold_target)
            recorder.record(
                state,
                q_cmd=hold_target,
                timestamp_host_command_ns=timestamp_ns,
                servo_sequence=sequence,
                command_valid=True,
                control_mode="servo_hold",
            )
            samples += 1
            next_deadline += 1.0 / float(self.config["control_rate_hz"])

        adapter.exit_servo()
        session["servo"] = False

    def _run_joint_jog(
        self,
        adapter: RebotControlAdapter,
        recorder: HardwareExperimentRecorder,
        session: dict[str, bool],
    ) -> None:
        """Run one signed excursion and return, with evidence and no inferred zero."""
        jog = validate_joint_jog(self.config)
        joint = jog["joint"] - 1
        result: dict[str, Any] = {
            "status": "running", "identification_ready": False,
            "physical_mapping_verified_by_test": False,
            "joint": joint + 1, "authorization_reference": jog["authorization_reference"],
            "requested_displacement_rad": jog["displacement_rad"],
            "phase_sample_ranges": {}, "stationary_windows": {},
            "maximum_abs_drift_from_origin_rad": [0.0] * JOINT_COUNT,
            "maximum_abs_error_to_previous_command_rad": [0.0] * JOINT_COUNT,
        }
        recorder.run_result = result
        initial = adapter.read_state()
        self._validate_state(initial, require_servo_active=False,
                             feedback_age_limit_key="maximum_disabled_feedback_age_ms")
        if initial.robot_mode != "disabled" or initial.safety_state != "disabled":
            raise RebotControlError("joint_jog requires an initially disabled robot")
        self._validate_jog_envelope(initial.q, jog)
        # Set intent first: an ACK timeout can occur after lower has acted.
        session["enabled"] = True
        adapter.enable()
        state = self._wait_until_motion_ready(adapter)
        self._validate_jog_envelope(state.q, jog)
        for index in range(JOINT_COUNT):
            if abs(state.q[index] - initial.q[index]) > jog["maximum_other_joint_drift_rad"]:
                raise RebotControlError("joint_jog position changed excessively during enable")
        origin = tuple(state.q)
        result["origin_rad"] = list(origin)
        recorder.config["trajectory_source"] = "single_joint_quintic_round_trip_v1"
        recorder.config["trajectory_hash"] = hashlib.sha256(json.dumps({
            "origin_rad": origin, "joint_jog": self.config["joint_jog"],
            "joint_direction": self.config["joint_direction"],
            "joint_offset_rad": self.config["joint_offset_rad"],
        }, sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest()
        state, _, _ = self._enter_servo_with_initial_hold(adapter, session, origin)
        # Keep the first jog profile target one nominal period after the
        # transition hold, matching the lower ServoCore real-dt envelope.
        self._sleep_period()
        start = self.monotonic_fn()
        previous_time: float | None = None
        previous_velocity = [0.0] * JOINT_COUNT
        previous_dt = 1 / self.config["control_rate_hz"]
        previous_target = origin
        window_m2: dict[str, list[float]] = {}
        while True:
            now = self.monotonic_fn()
            elapsed = now - start
            phase, target, stationary_sample = jog_target_at(jog, origin, elapsed)
            if now - start >= self.config["duration_s"]:
                raise RebotControlError("joint_jog timeout; round trip incomplete")
            dt = 1 / self.config["control_rate_hz"] if previous_time is None else now - previous_time
            if dt <= 0 or dt > jog["maximum_command_gap_s"]:
                raise RebotControlError("joint_jog command gap exceeded; no catch-up or automatic return")
            self._validate_state(state, require_servo_active=True)
            self._validate_target_step(target, previous_target)
            self._validate_tracking_error(target, state.q)
            velocity = [(target[i] - previous_target[i]) / dt for i in range(JOINT_COUNT)]
            for index in range(JOINT_COUNT):
                if abs(velocity[index]) > self.config["maximum_command_velocity_rad_s"][index] + 1e-12:
                    raise RebotControlError("joint_jog timed command velocity exceeded")
                acceleration_dt = (dt + previous_dt) / 2
                if abs(velocity[index] - previous_velocity[index]) / acceleration_dt > jog["maximum_acceleration_rad_s2"] + 1e-12:
                    raise RebotControlError("joint_jog timed command acceleration exceeded")
            timestamp_ns, sequence = adapter.send_servo_target(target)
            recorder.record(state, q_cmd=target, timestamp_host_command_ns=timestamp_ns,
                            servo_sequence=sequence, command_valid=True, control_mode="joint_jog")
            bounds = result["phase_sample_ranges"].setdefault(phase, [recorder.sample_count - 1, recorder.sample_count - 1])
            bounds[1] = recorder.sample_count - 1
            self._sleep_period()
            state = adapter.read_state()
            self._validate_state(state, require_servo_active=True)
            # Preserve the observation that caused an abort; no new command is implied.
            recorder.record(state, q_cmd=None, timestamp_host_command_ns=None,
                            servo_sequence=None, command_valid=False, control_mode="joint_jog")
            bounds[1] = recorder.sample_count - 1
            after_read = self.monotonic_fn()
            if after_read - start >= self.config["duration_s"]:
                raise RebotControlError("joint_jog timeout; round trip incomplete")
            if after_read - now > jog["maximum_command_gap_s"]:
                raise RebotControlError("joint_jog command gap exceeded; no catch-up or automatic return")
            for index in range(JOINT_COUNT):
                drift = abs(state.q[index] - origin[index])
                error = abs(state.q[index] - target[index])
                result["maximum_abs_drift_from_origin_rad"][index] = max(result["maximum_abs_drift_from_origin_rad"][index], drift)
                result["maximum_abs_error_to_previous_command_rad"][index] = max(result["maximum_abs_error_to_previous_command_rad"][index], error)
                if index != joint and drift > jog["maximum_other_joint_drift_rad"]:
                    raise RebotControlError(f"joint_jog non-target joint {index + 1} drift exceeded")
            if stationary_sample:
                if any(abs(state.q[i] - target[i]) > jog["settle_position_tolerance_rad"] for i in range(JOINT_COUNT)):
                    raise RebotControlError(f"joint_jog {phase}: position not settled")
                if any(abs(value) > jog["settle_velocity_tolerance_rad_s"] for value in state.qd):
                    raise RebotControlError(f"joint_jog {phase}: velocity not settled")
                window = result["stationary_windows"].setdefault(phase, {
                    "sample_count": 0, "mean_rad": [0.0] * JOINT_COUNT,
                    "stddev_rad": [0.0] * JOINT_COUNT,
                })
                m2 = window_m2.setdefault(phase, [0.0] * JOINT_COUNT)
                window["sample_count"] += 1
                for index, value in enumerate(state.q):
                    delta = value - window["mean_rad"][index]
                    window["mean_rad"][index] += delta / window["sample_count"]
                    m2[index] += delta * (value - window["mean_rad"][index])
                    window["stddev_rad"][index] = math.sqrt(max(0.0, m2[index] / window["sample_count"]))
            previous_target, previous_time, previous_velocity = target, now, velocity
            previous_dt = dt
            if elapsed >= jog["nominal_duration_s"]:
                break
        if any(result["stationary_windows"].get(phase, {}).get("sample_count", 0) < 2
               for phase in ("baseline", "endpoint", "returned")):
            raise RebotControlError("joint_jog insufficient stationary samples")
        baseline = result["stationary_windows"]["baseline"]["mean_rad"]
        endpoint = result["stationary_windows"]["endpoint"]["mean_rad"]
        returned = result["stationary_windows"]["returned"]["mean_rad"]
        result["measured_displacement_rad"] = [endpoint[i] - baseline[i] for i in range(JOINT_COUNT)]
        result["return_error_rad"] = [returned[i] - baseline[i] for i in range(JOINT_COUNT)]
        adapter.exit_servo()
        session["servo"] = False

    def _run_excitation(
        self,
        adapter: RebotControlAdapter,
        recorder: HardwareExperimentRecorder,
        session: dict[str, bool],
        artifact: ReplayArtifact,
    ) -> None:
        """Follow the approved C2 path at real Servo dispatch timestamps."""

        initial = adapter.read_state()
        self._validate_state(
            initial,
            require_servo_active=False,
            feedback_age_limit_key="maximum_disabled_feedback_age_ms",
        )
        if initial.robot_mode != "disabled" or initial.safety_state != "disabled":
            raise RebotControlError("excitation requires an initially disabled robot")
        # ArmClient is the lower-level API used by this project. Match the SDK's
        # high-level Robot.enable() policy by applying its existing MoveJ PVT
        # constants while the robot is still disabled; do not duplicate them here.
        adapter.configure_movej_pvt()
        # Treat enable as potentially effective before its TCP reply is observed.
        session["enabled"] = True
        adapter.enable()
        ready = self._wait_until_motion_ready(adapter)

        preposition_start = tuple(ready.q)
        movej_sent = False
        if self._excitation_at_start_and_settled(ready, artifact.q_start):
            final = ready
            preposition_status = "already_at_start"
        else:
            # A MoveJ timeout/reject can be ambiguous after dispatch. Mark this as
            # a motion lifecycle before the synchronous call so error cleanup sends stop.
            session["motion_lifecycle"] = True
            adapter.movej_to(
                artifact.q_start,
                max_velocity_rad_s=self.config["movej_max_velocity_rad_s"],
                max_acceleration_rad_s2=self.config["movej_max_acceleration_rad_s2"],
                max_jerk_rad_s3=self.config["movej_max_jerk_rad_s3"],
                timeout_s=self.config["movej_timeout_s"],
            )
            movej_sent = True
            final = self._wait_until_preposition_settled(adapter, artifact.q_start)
            preposition_status = "completed"

        if final.robot_mode != "idle":
            raise RebotControlError(
                f"preposition final robot_mode={final.robot_mode}; expected idle before Servo"
            )
        self._validate_excitation_start(final.q, artifact.q_start)
        self._validate_preposition_settle(final.qd)
        recorder.preposition_result = {
            "method": "sdk_movej",
            "start_q": list(preposition_start),
            "target_q": list(artifact.q_start),
            "status": preposition_status,
            "movej_command_count": int(movej_sent),
            "final_q": list(final.q),
            "max_position_error": max(
                abs(actual - expected)
                for actual, expected in zip(final.q, artifact.q_start)
            ),
            "max_velocity_after_move": max(abs(value) for value in final.qd),
        }

        state, initial_hold_timestamp_ns, _ = self._enter_servo_with_initial_hold(
            adapter, session, artifact.q_start
        )
        tracker = ActualTimeServoEnvelopeTracker(self.config)
        tracker.initialize(artifact.q_start, initial_hold_timestamp_ns)
        period_s = 1.0 / float(self.config["control_rate_hz"])
        period_ns = int(round(period_s * 1e9))
        if period_ns <= 0:
            raise RebotControlError("invalid frozen Servo reference period")

        previous_dispatch_timestamp_ns = int(initial_hold_timestamp_ns)
        next_deadline_ns = previous_dispatch_timestamp_ns + period_ns
        previous_target = artifact.q_start
        trajectory_start_timestamp_ns: int | None = None
        last_trajectory_time_s: float | None = None
        feedback_unusable_since_ns: int | None = None
        feedback_unusable_sample_count = 0
        feedback_recovery_event_count = 0
        maximum_runtime_feedback_age_ms = 0.0
        command_index = 0
        while True:
            attempted_sequence = adapter.next_servo_sequence
            state: CaptureSample | None = None
            actual_dispatch_timestamp_ns: int | None = None
            target: ResampledReplayTarget | None = None
            prediction: ServoEnvelopePrediction | None = None
            stage = "excitation_dispatch_pacing"
            try:
                deadline_ns = max(
                    next_deadline_ns,
                    previous_dispatch_timestamp_ns + period_ns,
                )
                self._pace_deadline_ns(deadline_ns)

                # Servo replay deliberately reuses the latest published UDP state.
                # Freshness is guarded independently by both SDK feedback age and
                # the host receive timestamp, so UDP cadence cannot throttle sends.
                stage = "excitation_state_snapshot"
                state = adapter.latest_state
                if state is None:
                    raise RebotControlError("state snapshot unavailable during excitation")
                gate_timestamp_ns = self.monotonic_ns_fn()
                was_unusable = feedback_unusable_since_ns is not None
                feedback_unusable_since_ns, measurement_usable = (
                    self._validate_excitation_runtime_state(
                        state,
                        current_monotonic_ns=gate_timestamp_ns,
                        feedback_unusable_since_ns=feedback_unusable_since_ns,
                    )
                )
                maximum_runtime_feedback_age_ms = max(
                    maximum_runtime_feedback_age_ms,
                    max(float(value) for value in state.feedback_age_ms),
                )
                if not measurement_usable:
                    feedback_unusable_sample_count += 1
                    if not was_unusable:
                        feedback_recovery_event_count += 1
                # During excitation, q_ref-q is identification/control-quality
                # evidence only. It is recorded in q/q_cmd and summarized offline;
                # unlike servo_hold/joint_jog it is not a runtime abort gate.

                actual_dispatch_timestamp_ns = self.monotonic_ns_fn()
                if trajectory_start_timestamp_ns is None:
                    trajectory_time_s = 0.0
                else:
                    trajectory_time_s = min(
                        artifact.duration_s,
                        (actual_dispatch_timestamp_ns - trajectory_start_timestamp_ns)
                        * 1e-9,
                    )
                target = resample_actual_time_quintic(artifact, trajectory_time_s)
                stage = "excitation_runtime_envelope"
                prediction = tracker.predict(
                    target.q_ref, actual_dispatch_timestamp_ns
                )
                actual_dispatch_interval_ns = (
                    actual_dispatch_timestamp_ns - previous_dispatch_timestamp_ns
                )
                stage = "excitation_servo_send"
                timestamp_ns, sequence = adapter.send_servo_target(
                    target.q_ref,
                    host_timestamp_ns=actual_dispatch_timestamp_ns,
                )
                # send_servo_target returns only after the SDK confirms acceptance.
                tracker.commit(prediction)
            except RebotControlError as exc:
                failure_observed_timestamp_ns = self.monotonic_ns_fn()
                if target is None:
                    failure_trajectory_time_s = (
                        0.0
                        if trajectory_start_timestamp_ns is None
                        else min(
                            artifact.duration_s,
                            (failure_observed_timestamp_ns - trajectory_start_timestamp_ns)
                            * 1e-9,
                        )
                    )
                    target = resample_actual_time_quintic(
                        artifact, failure_trajectory_time_s
                    )
                envelope = tracker.summary()
                envelope["trajectory_time_start_s"] = (
                    None if tracker.accepted_count == 0 else 0.0
                )
                envelope["trajectory_time_end_s"] = last_trajectory_time_s
                envelope["trajectory_duration_s"] = artifact.duration_s
                envelope["feedback_runtime"] = {
                    "measurement_unusable_sample_count": feedback_unusable_sample_count,
                    "transient_recovery_event_count": feedback_recovery_event_count,
                    "maximum_feedback_age_ms": maximum_runtime_feedback_age_ms,
                    "lower_feedback_timeout_ms": float(
                        self.config["lower_feedback_timeout_ms"]
                    ),
                    "transient_feedback_invalid_recovery_ms": float(
                        self.config["transient_feedback_invalid_recovery_ms"]
                    ),
                }
                recorder.runtime_envelope_result = envelope
                recorder.failure_result = self._excitation_failure_evidence(
                    stage=stage,
                    sample_index=command_index,
                    target=target,
                    previous_target=previous_target,
                    state=state,
                    actual_dispatch_timestamp_ns=actual_dispatch_timestamp_ns,
                    failure_observed_timestamp_ns=failure_observed_timestamp_ns,
                    servo_sequence=attempted_sequence,
                    error=exc,
                    prediction=prediction,
                )
                recorder.failure_result.update(
                    self._best_effort_servo_reject_diagnostics(adapter)
                )
                if stage == "excitation_servo_send":
                    raise RebotControlError(
                        f"excitation command {command_index} Servo send rejected: {exc}"
                    ) from exc
                raise
            assert state is not None and target is not None
            if trajectory_start_timestamp_ns is None:
                trajectory_start_timestamp_ns = actual_dispatch_timestamp_ns
            recorder.record(
                state,
                q_cmd=target.q_ref,
                timestamp_host_command_ns=timestamp_ns,
                actual_dispatch_timestamp_ns=actual_dispatch_timestamp_ns,
                actual_dispatch_interval_ns=actual_dispatch_interval_ns,
                reference_dispatch_skew_ns=(
                    actual_dispatch_timestamp_ns
                    - trajectory_start_timestamp_ns
                    - int(round(target.trajectory_time_s * 1e9))
                ),
                state_snapshot_age_ms=self._state_snapshot_age_ms(
                    state, actual_dispatch_timestamp_ns
                ),
                servo_sequence=sequence,
                command_valid=True,
                control_mode="excitation",
                trajectory_time_s=target.trajectory_time_s,
                trajectory_interval_index=target.interval_index,
                trajectory_interval_ratio=target.interval_ratio,
            )
            previous_dispatch_timestamp_ns = actual_dispatch_timestamp_ns
            previous_target = target.q_ref
            last_trajectory_time_s = target.trajectory_time_s
            next_deadline_ns = deadline_ns + period_ns
            command_index += 1
            if target.trajectory_time_s >= artifact.duration_s:
                break

        envelope = tracker.summary()
        envelope["trajectory_time_start_s"] = 0.0
        envelope["trajectory_time_end_s"] = artifact.duration_s
        envelope["trajectory_duration_s"] = artifact.duration_s
        envelope["feedback_runtime"] = {
            "measurement_unusable_sample_count": feedback_unusable_sample_count,
            "transient_recovery_event_count": feedback_recovery_event_count,
            "maximum_feedback_age_ms": maximum_runtime_feedback_age_ms,
            "lower_feedback_timeout_ms": float(self.config["lower_feedback_timeout_ms"]),
            "transient_feedback_invalid_recovery_ms": float(
                self.config["transient_feedback_invalid_recovery_ms"]
            ),
        }
        recorder.runtime_envelope_result = envelope

        adapter.exit_servo()
        session["servo"] = False

    def _best_effort_servo_reject_diagnostics(
        self,
        adapter: RebotControlAdapter,
    ) -> dict[str, Any]:
        """Capture one non-blocking post-failure snapshot without delaying cleanup."""

        try:
            state = adapter.latest_state
        except Exception as exc:
            return {"diagnostic_state_error": f"latest_state: {type(exc).__name__}: {exc}"}
        if state is None:
            return {"diagnostic_state_error": "latest_state unavailable"}

        result: dict[str, Any] = {
            "servo_mode": state.servo_mode,
            "servo_last_accepted_sequence": state.servo_last_accepted_sequence,
            "servo_target_age_ns": state.servo_target_age_ns,
            "servo_accepted_targets": state.servo_accepted_targets,
            "servo_rejected_targets": state.servo_rejected_targets,
            "servo_target_jump_rejects": state.servo_target_jump_rejects,
            "servo_velocity_rejects": state.servo_velocity_rejects,
            "servo_acceleration_rejects": state.servo_acceleration_rejects,
            "servo_jerk_rejects": state.servo_jerk_rejects,
            "primary_fault_code": state.primary_fault_code,
            "servo_reject_counters": {
                "accepted_targets": state.servo_accepted_targets,
                "rejected_targets": state.servo_rejected_targets,
                "target_jump_rejects": state.servo_target_jump_rejects,
                "velocity_rejects": state.servo_velocity_rejects,
                "acceleration_rejects": state.servo_acceleration_rejects,
                "jerk_rejects": state.servo_jerk_rejects,
            },
        }
        if state.primary_fault_code:
            result["primary_fault_code_symbolic_mapping"] = (
                "unresolved_in_available_sdk_checkout"
            )
        return result

    def _excitation_failure_evidence(
        self,
        *,
        stage: str,
        sample_index: int,
        target: ResampledReplayTarget | None,
        previous_target: Sequence[float],
        state: CaptureSample | None,
        actual_dispatch_timestamp_ns: int | None,
        failure_observed_timestamp_ns: int,
        servo_sequence: int,
        error: BaseException,
        prediction: ServoEnvelopePrediction | None,
    ) -> dict[str, Any]:
        """Build the complete per-sample evidence required for any replay gate failure."""

        measured_q = None if state is None else list(state.q)
        q_ref = None if target is None else list(target.q_ref)
        tracking_error_q = (
            None
            if state is None or target is None
            else [
                target.q_ref[index] - state.q[index]
                for index in range(JOINT_COUNT)
            ]
        )
        violation = error if isinstance(error, ServoEnvelopeViolation) else None
        predicted = violation.prediction if violation is not None else prediction
        counters = {
            "accepted_targets": 0 if state is None else state.servo_accepted_targets,
            "rejected_targets": 0 if state is None else state.servo_rejected_targets,
            "target_jump_rejects": 0 if state is None else state.servo_target_jump_rejects,
            "velocity_rejects": 0 if state is None else state.servo_velocity_rejects,
            "acceleration_rejects": 0 if state is None else state.servo_acceleration_rejects,
            "jerk_rejects": 0 if state is None else state.servo_jerk_rejects,
        }
        return {
            "stage": stage,
            "sample_index": int(sample_index),
            "trajectory_time_s": None if target is None else target.trajectory_time_s,
            "artifact_time_s": None if target is None else target.trajectory_time_s,
            "trajectory_interval_index": None if target is None else target.interval_index,
            "trajectory_interval_ratio": None if target is None else target.interval_ratio,
            "q_ref": q_ref,
            "previous_q_ref": list(previous_target),
            "measured_q": measured_q,
            "target_step_delta_q": None if predicted is None else list(predicted.delta_q),
            "predicted_qd": None if predicted is None else list(predicted.qd),
            "predicted_qdd": None if predicted is None else list(predicted.qdd),
            "predicted_jerk": None if predicted is None else list(predicted.jerk),
            "predicted_dt_s": None if predicted is None else predicted.dt_s,
            "violating_joint": None if violation is None or violation.joint is None else violation.joint + 1,
            "violating_quantity": None if violation is None else violation.quantity,
            "calculated_value": None if violation is None else violation.value,
            "configured_limit": None if violation is None else violation.limit,
            "tracking_error_q": tracking_error_q,
            "actual_dispatch_timestamp_ns": (
                None
                if actual_dispatch_timestamp_ns is None
                else int(actual_dispatch_timestamp_ns)
            ),
            "failure_observed_timestamp_ns": int(failure_observed_timestamp_ns),
            "servo_sequence": int(servo_sequence),
            "feedback_valid": None if state is None else list(state.feedback_valid),
            "feedback_age_ms": None if state is None else list(state.feedback_age_ms),
            "state_snapshot_age_ms": (
                None
                if state is None
                else self._safe_state_snapshot_age_ms(
                    state, failure_observed_timestamp_ns
                )
            ),
            "primary_fault_code": 0 if state is None else state.primary_fault_code,
            "safety_state": None if state is None else state.safety_state,
            "servo_active": None if state is None else bool(state.servo_active),
            "servo_reject_counters": counters,
            "error": f"{type(error).__name__}: {error}",
        }

    def _excitation_at_start_and_settled(
        self,
        state: CaptureSample,
        q_start: Sequence[float],
    ) -> bool:
        start = _six_finite(q_start, "artifact q_start")
        position_tolerance = float(self.config["start_position_tolerance_rad"])
        velocity_tolerance = float(
            self.config["preposition_settle_velocity_tolerance_rad_s"]
        )
        return all(
            abs(actual - expected) <= position_tolerance
            for actual, expected in zip(state.q, start)
        ) and all(abs(value) <= velocity_tolerance for value in state.qd)

    def _validate_preposition_settle(self, measured_qd: Sequence[float]) -> None:
        velocity = _six_finite(measured_qd, "preposition measured_qd")
        tolerance = float(self.config["preposition_settle_velocity_tolerance_rad_s"])
        for index, value in enumerate(velocity):
            if abs(value) > tolerance:
                raise RebotControlError(
                    f"preposition settle J{index + 1} velocity exceeds {tolerance} rad/s"
                )

    def _wait_until_preposition_settled(
        self,
        adapter: RebotControlAdapter,
        q_start: Sequence[float],
    ) -> CaptureSample:
        """Wait for fresh post-MoveJ UDP states to settle before claiming Servo."""

        return self._wait_until_target_settled(
            adapter,
            q_start,
            position_tolerance_rad=float(self.config["start_position_tolerance_rad"]),
            velocity_tolerance_rad_s=float(
                self.config["preposition_settle_velocity_tolerance_rad_s"]
            ),
            label="preposition",
        )

    def _wait_until_target_settled(
        self,
        adapter: RebotControlAdapter,
        q_target: Sequence[float],
        *,
        position_tolerance_rad: float,
        velocity_tolerance_rad_s: float,
        label: str,
        feedback_age_limit_key: str = "motion_ready_feedback_max_age_ms",
    ) -> CaptureSample:
        """Require three consecutive fresh idle snapshots around one MoveJ target."""

        target = _six_finite(q_target, f"{label} target")
        position_tolerance = float(position_tolerance_rad)
        velocity_tolerance = float(velocity_tolerance_rad_s)
        required_consecutive = 3
        max_attempts = max(
            required_consecutive,
            int(
                math.ceil(
                    float(self.config["command_timeout_s"])
                    * float(self.config["control_rate_hz"])
                )
            ),
        )
        consecutive = 0
        last: CaptureSample | None = None
        last_fresh: CaptureSample | None = None
        last_stale_error: RebotControlError | None = None
        for _ in range(max_attempts):
            state = adapter.read_state()
            last = state
            try:
                self._validate_state(
                    state,
                    require_servo_active=False,
                    feedback_age_limit_key=feedback_age_limit_key,
                )
            except RebotControlError as exc:
                # A single post-MoveJ UDP snapshot can still report feedback ages
                # from the just-finished motion. Never use that stale q/qd for a
                # settle decision; reset the consecutive-fresh count and wait for
                # a new packet inside the existing bounded settle window.
                if str(exc).startswith("feedback stale"):
                    last_stale_error = exc
                    consecutive = 0
                    continue
                raise
            last_fresh = state
            if state.robot_mode in {"disabled", "fault"}:
                raise RebotControlError(
                    f"{label} settle lost enabled state: robot_mode={state.robot_mode}"
                )
            position_ok = all(
                abs(actual - expected) <= position_tolerance
                for actual, expected in zip(state.q, target)
            )
            velocity_ok = all(
                abs(value) <= velocity_tolerance for value in state.qd
            )
            if (
                state.robot_mode == "idle"
                and state.safety_state == "ready"
                and position_ok
                and velocity_ok
            ):
                consecutive += 1
                if consecutive >= required_consecutive:
                    return state
            else:
                consecutive = 0

        if last is None:
            raise RebotControlError(f"{label} settle received no post-MoveJ state")
        if last_fresh is None and last_stale_error is not None:
            raise RebotControlError(
                f"{label} settle timed out waiting for fresh feedback; "
                f"last error: {last_stale_error}"
            )
        diagnostic = last_fresh if last_fresh is not None else last
        max_position_error = max(
            abs(actual - expected) for actual, expected in zip(diagnostic.q, target)
        )
        max_velocity = max(abs(value) for value in diagnostic.qd)
        stale_suffix = (
            f"; most recent stale observation: {last_stale_error}"
            if last_stale_error is not None
            else ""
        )
        raise RebotControlError(
            f"{label} settle timed out before three consecutive idle settled states "
            f"(robot_mode={diagnostic.robot_mode}, safety_state={diagnostic.safety_state}, "
            f"max_position_error={max_position_error:.6f} rad, "
            f"max_velocity={max_velocity:.6f} rad/s){stale_suffix}"
        )

    def _validate_excitation_start(
        self,
        measured_q: Sequence[float],
        q_start: Sequence[float],
    ) -> None:
        measured = _six_finite(measured_q, "measured_q")
        start = _six_finite(q_start, "artifact q_start")
        tolerance = float(self.config["start_position_tolerance_rad"])
        for index, (actual, expected) in enumerate(zip(measured, start)):
            if abs(actual - expected) > tolerance:
                raise RebotControlError(
                    f"excitation start J{index + 1} differs from frozen artifact "
                    f"q_ref[0] by more than {tolerance} rad"
                )

    def _validate_jog_envelope(self, origin: Sequence[float], jog: dict[str, Any]) -> None:
        for index, value in enumerate(origin):
            endpoint = value + (jog["displacement_rad"] if index == jog["joint"] - 1 else 0.0)
            lower = self.config["joint_position_min_rad"][index] + jog["position_margin_rad"]
            upper = self.config["joint_position_max_rad"][index] - jog["position_margin_rad"]
            if not lower <= min(value, endpoint) <= max(value, endpoint) <= upper:
                raise RebotControlError(f"joint_jog envelope joint {index + 1} violates position limits/margin")

    def _assert_session_authorized(self, mode: str) -> None:
        if not self.mock_backend and not self.config["allow_hardware"]:
            raise PermissionError("allow_hardware=false blocks creation of a real ArmClient session")
        if mode in {"servo_hold", "joint_jog", "excitation"}:
            if not self.config["allow_motion"]:
                raise PermissionError("allow_motion=false blocks all motor-changing commands")
            if not self.config["joint_mapping_verified"]:
                raise PermissionError("joint_mapping_verified=false blocks all motor-changing commands")
            if self.config["j1_convention"] == "UNRESOLVED":
                raise PermissionError("j1_convention=UNRESOLVED blocks all motor-changing commands")
        if mode == "joint_jog":
            validate_joint_jog(self.config)
            if self.config.get("joint_mapping_scope") != "joint_jog":
                raise PermissionError("joint_jog requires joint_mapping_scope=joint_jog; hold-only acceptance is insufficient")
            if self.config["j1_convention"] == "PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907":
                raise PermissionError("historical J1 convention is accepted for servo_hold smoke only")
            if not self.mock_backend and self.config["j1_convention"].startswith("MOCK"):
                raise PermissionError("Mock mapping cannot authorize real joint_jog")
        if mode == "excitation":
            if not self.mock_backend and self.config.get("controlled_park_before_disable") is not True:
                raise PermissionError(
                    "real excitation requires controlled_park_before_disable=true before connect"
                )
            scope = self.config.get("joint_mapping_scope")
            if scope not in {"excitation_smoke", "excitation"}:
                raise PermissionError(
                    "excitation requires joint_mapping_scope=excitation_smoke or excitation; "
                    "hold/jog acceptance is insufficient"
                )
            if (
                scope == "excitation"
                and self.config["j1_convention"] == "PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907"
            ):
                raise PermissionError(
                    "historical visual-only J1 convention can authorize excitation_smoke only"
                )
            if not self.mock_backend and self.config["j1_convention"].startswith("MOCK"):
                raise PermissionError("Mock mapping cannot authorize real excitation")

    def _wait_until_motion_ready(self, adapter: RebotControlAdapter) -> CaptureSample:
        """Wait for Ready/idle plus feedback fresh enough for enabled motion."""

        deadline = self.monotonic_fn() + float(self.config["command_timeout_s"])
        last_mode = "unknown"
        last_safety = "unknown"
        last_age_ms = math.inf
        enabled_age_limit = float(self.config["motion_ready_feedback_max_age_ms"])
        while True:
            state = adapter.read_state()
            # Immediately after ENABLE, the first post-command UDP frame may still
            # carry feedback ages from the disabled phase. Keep all fault/limit
            # checks active, but use the separately configured disabled-age ceiling
            # until a genuinely fresh Ready/idle frame arrives.
            self._validate_state(
                state,
                require_servo_active=False,
                feedback_age_limit_key="maximum_disabled_feedback_age_ms",
            )
            last_mode = state.robot_mode
            last_safety = state.safety_state
            last_age_ms = max(state.feedback_age_ms)
            if (
                state.robot_mode == "idle"
                and state.safety_state == "ready"
                and last_age_ms <= enabled_age_limit
            ):
                return state
            now = self.monotonic_fn()
            if now >= deadline:
                break
            self.sleep_fn(min(0.02, max(0.0, deadline - now)))
        raise RebotControlError(
            "timed out waiting for motion-ready fresh feedback after enable "
            f"(robot_mode={last_mode}, safety_state={last_safety}, "
            f"max_feedback_age_ms={last_age_ms:.3f}, required<={enabled_age_limit:.3f})"
        )

    def _wait_until_servo_active(self, adapter: RebotControlAdapter) -> CaptureSample:
        """Wait for UDP state to confirm Servo ownership after the initial hold target."""

        deadline = self.monotonic_fn() + float(self.config["command_timeout_s"])
        last_mode = "unknown"
        last_safety = "unknown"
        last_servo_mode = "unknown"
        while True:
            state = adapter.read_state()
            self._validate_state(state, require_servo_active=None)
            last_mode = state.robot_mode
            last_safety = state.safety_state
            last_servo_mode = state.servo_mode
            if state.servo_active:
                return state
            now = self.monotonic_fn()
            if now >= deadline:
                break
            self.sleep_fn(min(0.02, max(0.0, deadline - now)))
        raise RebotControlError(
            "timed out waiting for servo_active=true after enter_servo "
            f"(robot_mode={last_mode}, safety_state={last_safety}, servo_mode={last_servo_mode})"
        )

    def _enter_servo_with_initial_hold(
        self,
        adapter: RebotControlAdapter,
        session: dict[str, bool],
        hold_target: Sequence[float],
    ) -> tuple[CaptureSample, int, int]:
        """Enter Servo and immediately arm its target watchdog with a hold target."""

        target = _six_finite(hold_target, "initial Servo hold target")
        # Both enter_servo and its first target can take effect before a lost TCP
        # reply is observed. Mark the full cleanup intent before either call.
        session["servo"] = True
        session["motion_lifecycle"] = True
        adapter.enter_servo()
        timestamp_ns, sequence = adapter.send_servo_target(target)
        state = self._wait_until_servo_active(adapter)
        self._validate_state(state, require_servo_active=True)
        return state, timestamp_ns, sequence

    @staticmethod
    def _non_catchup_deadline(
        scheduled_deadline_s: float,
        previous_dispatch_start_s: float,
        period_s: float,
    ) -> float:
        """Never schedule the next Servo dispatch earlier than one full period."""

        if not all(
            math.isfinite(value)
            for value in (scheduled_deadline_s, previous_dispatch_start_s, period_s)
        ) or period_s <= 0.0:
            raise ValueError("Servo dispatch timing must be finite with positive period")
        return max(scheduled_deadline_s, previous_dispatch_start_s + period_s)

    def _pace_deadline(self, deadline: float) -> float:
        """Sleep only the remaining control period; never issue catch-up bursts."""

        now = self.monotonic_fn()
        if now < deadline:
            self.sleep_fn(deadline - now)
            return deadline
        return now

    def _pace_deadline_ns(self, deadline_ns: int) -> int:
        """Wait until an integer monotonic deadline, retrying if sleep returns early."""

        deadline = int(deadline_ns)
        if deadline <= 0:
            raise ValueError("Servo dispatch deadline must be positive")
        while True:
            now_ns = int(self.monotonic_ns_fn())
            if now_ns >= deadline:
                return now_ns
            self.sleep_fn((deadline - now_ns) * 1e-9)

    def _validate_excitation_runtime_state(
        self,
        state: CaptureSample,
        *,
        current_monotonic_ns: int,
        feedback_unusable_since_ns: int | None,
    ) -> tuple[int | None, bool]:
        """Classify one fresh-host Servo snapshot without defeating Lower recovery."""

        if int(state.primary_fault_code) != 0:
            raise RebotControlError(f"primary fault active: {state.primary_fault_code}")
        if state.safety_state in {"protective_stop", "fault_latched", "emergency_stop"}:
            raise RebotControlError(f"unsafe lower safety_state={state.safety_state}")
        if len(state.feedback_valid) != JOINT_COUNT:
            raise RebotControlError("feedback_valid must contain six values")
        if len(state.feedback_age_ms) != JOINT_COUNT or not all(
            math.isfinite(value) for value in state.feedback_age_ms
        ):
            raise RebotControlError("feedback_age_ms feedback must contain six finite values")
        if len(state.q) != JOINT_COUNT or len(state.qd) != JOINT_COUNT:
            raise RebotControlError("q/qd feedback must contain six values")
        for index, valid in enumerate(state.feedback_valid):
            if valid and (
                not math.isfinite(state.q[index]) or not math.isfinite(state.qd[index])
            ):
                raise RebotControlError(
                    f"valid joint feedback J{index + 1} contains non-finite q/qd"
                )
        snapshot_age_ms = self._state_snapshot_age_ms(state, current_monotonic_ns)
        host_timeout_ms = float(self.config["host_state_snapshot_timeout_s"]) * 1000.0
        if snapshot_age_ms > host_timeout_ms:
            raise RebotControlError(
                "state snapshot stale: host receive age "
                f"{snapshot_age_ms:.3f} ms exceeds host_state_snapshot_timeout_s="
                f"{float(self.config['host_state_snapshot_timeout_s']):.6g}"
            )
        if not bool(state.servo_active):
            raise RebotControlError("servo_active=False but expected True")

        lower_timeout_ms = float(self.config["lower_feedback_timeout_ms"])
        recovery_ms = float(self.config["transient_feedback_invalid_recovery_ms"])
        max_age_ms = max(float(value) for value in state.feedback_age_ms)
        measurement_usable = all(state.feedback_valid) and max_age_ms <= lower_timeout_ms
        if measurement_usable:
            for index, value in enumerate(state.q):
                lower = self.config["joint_position_min_rad"][index]
                upper = self.config["joint_position_max_rad"][index]
                if not lower <= value <= upper:
                    raise RebotControlError(
                        f"feedback joint {index + 1} outside configured position limits"
                    )
            return None, True

        # Do not grant a fresh recovery window to a snapshot that already proves
        # the Lower freshness + grace budget has elapsed.
        if max_age_ms > lower_timeout_ms + recovery_ms:
            raise RebotControlError(
                "transient feedback invalid exceeded lower recovery budget: "
                f"max_feedback_age_ms={max_age_ms:.3f} > "
                f"lower_feedback_timeout_ms+recovery_ms="
                f"{lower_timeout_ms + recovery_ms:.3f}"
            )
        started_ns = (
            int(current_monotonic_ns)
            if feedback_unusable_since_ns is None
            else int(feedback_unusable_since_ns)
        )
        invalid_duration_ms = (int(current_monotonic_ns) - started_ns) * 1e-6
        if invalid_duration_ms >= recovery_ms:
            raise RebotControlError(
                "transient feedback invalid exceeded recovery window: "
                f"duration_ms={invalid_duration_ms:.3f} >= {recovery_ms:.3f}"
            )
        return started_ns, False

    def _validate_state(
        self,
        state: CaptureSample,
        *,
        require_servo_active: bool | None,
        feedback_age_limit_key: str = "motion_ready_feedback_max_age_ms",
        current_monotonic_ns: int | None = None,
    ) -> None:
        if not all(state.feedback_valid):
            invalid = ", ".join(
                f"J{index + 1}"
                for index, valid in enumerate(state.feedback_valid)
                if not valid
            )
            raise RebotControlError(
                f"invalid joint feedback blocks hardware runner (invalid joints: {invalid})"
            )
        if int(state.primary_fault_code) != 0:
            raise RebotControlError(f"primary fault active: {state.primary_fault_code}")
        if state.safety_state in {"protective_stop", "fault_latched", "emergency_stop"}:
            raise RebotControlError(f"unsafe lower safety_state={state.safety_state}")
        for name, values in (
            ("q", state.q),
            ("qd", state.qd),
            ("feedback_age_ms", state.feedback_age_ms),
        ):
            if len(values) != JOINT_COUNT or not all(math.isfinite(value) for value in values):
                raise RebotControlError(f"{name} feedback must contain six finite values")
        max_age = float(self.config[feedback_age_limit_key])
        if any(value > max_age for value in state.feedback_age_ms):
            raise RebotControlError(
                f"feedback stale: age exceeds {feedback_age_limit_key}={max_age}"
            )
        now_ns = (
            int(self.monotonic_ns_fn())
            if current_monotonic_ns is None
            else int(current_monotonic_ns)
        )
        snapshot_age_ms = self._state_snapshot_age_ms(state, now_ns)
        if snapshot_age_ms > float(self.config["host_state_snapshot_timeout_s"]) * 1000.0:
            raise RebotControlError(
                "state snapshot stale: host receive age "
                f"{snapshot_age_ms:.3f} ms exceeds host_state_snapshot_timeout_s="
                f"{float(self.config['host_state_snapshot_timeout_s']):.6g}"
            )
        for index, value in enumerate(state.q):
            lower = self.config["joint_position_min_rad"][index]
            upper = self.config["joint_position_max_rad"][index]
            if not lower <= value <= upper:
                raise RebotControlError(f"feedback joint {index + 1} outside configured position limits")
        if require_servo_active is not None and bool(state.servo_active) != require_servo_active:
            raise RebotControlError(
                f"servo_active={state.servo_active} but expected {require_servo_active}"
            )

    @staticmethod
    def _state_snapshot_age_ms(
        state: CaptureSample,
        current_monotonic_ns: int,
    ) -> float:
        timestamp_ns = int(state.timestamp_host_rx_ns)
        now_ns = int(current_monotonic_ns)
        if timestamp_ns <= 0:
            raise RebotControlError("state snapshot host receive timestamp must be positive")
        if now_ns < timestamp_ns:
            raise RebotControlError("state snapshot host receive timestamp is in the future")
        return (now_ns - timestamp_ns) * 1e-6

    @classmethod
    def _safe_state_snapshot_age_ms(
        cls,
        state: CaptureSample,
        current_monotonic_ns: int,
    ) -> float | None:
        try:
            return cls._state_snapshot_age_ms(state, current_monotonic_ns)
        except RebotControlError:
            return None

    def _validate_target_step(
        self,
        q_target: Sequence[float],
        previous_q_target: Sequence[float],
    ) -> None:
        """Check command position and consecutive-target motion limits only."""

        target = _six_finite(q_target, "q_target")
        previous = _six_finite(previous_q_target, "previous_q_target")
        rate = float(self.config["control_rate_hz"])
        servo_delta_limit = float(self.config["maximum_servo_target_delta_rad"])
        for index, value in enumerate(target):
            lower = self.config["joint_position_min_rad"][index]
            upper = self.config["joint_position_max_rad"][index]
            if not lower <= value <= upper:
                raise RebotControlError(f"command joint {index + 1} outside configured position limits")
            max_delta = self.config["maximum_command_velocity_rad_s"][index] / rate
            delta = abs(value - previous[index])
            if delta > max_delta + 1e-12:
                raise RebotControlError(
                    f"target step J{index + 1} exceeds velocity-derived limit {max_delta} rad"
                )
            if delta > servo_delta_limit + 1e-12:
                raise RebotControlError(
                    f"target step J{index + 1} exceeds ServoCore fixed limit "
                    f"{servo_delta_limit} rad"
                )

    def _validate_tracking_error(
        self,
        q_target: Sequence[float],
        measured_q: Sequence[float],
    ) -> None:
        """Check target-to-measurement lag against its independent position limit."""

        target = _six_finite(q_target, "q_target")
        measured = _six_finite(measured_q, "measured_q")
        for index, (command, actual) in enumerate(zip(target, measured)):
            limit = self.config["maximum_tracking_error_rad"][index]
            if abs(command - actual) > limit + 1e-12:
                raise RebotControlError(
                    f"tracking error J{index + 1} exceeds independent limit {limit} rad"
                )

    def _shutdown(
        self,
        adapter: RebotControlAdapter,
        session: dict[str, bool],
        *,
        error_path: bool,
        cause: BaseException | None,
    ) -> tuple[list[str], dict[str, Any]]:
        errors: list[str] = []
        result: dict[str, Any] = {
            "strategy": "no_motion",
            "controlled_park_requested": bool(
                self.config.get("controlled_park_before_disable", False)
            ),
            "park_attempted": False,
            "park_status": "skipped",
            "park_target_source": None,
            "park_target_q": None,
            "park_movej_command_count": 0,
            "park_final_q": None,
            "park_max_position_error": None,
            "park_max_velocity": None,
            "stop_attempted": False,
            "stop_status": "not_needed",
            "disable_attempted": False,
            "disable_status": "not_needed",
            "close_attempted": False,
            "close_status": "not_needed",
        }

        def attempt(name: str, action: Callable[[], None]) -> bool:
            try:
                action()
                return True
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                return False

        if session["servo"]:
            result["exit_servo_attempted"] = True
            result["exit_servo_status"] = (
                "completed" if attempt("exit_servo", adapter.exit_servo) else "failed"
            )
            session["servo"] = False
        else:
            result["exit_servo_attempted"] = False
            result["exit_servo_status"] = "not_needed"

        controlled_park = (
            bool(self.config.get("controlled_park_before_disable", False))
            and session["enabled"]
            and session["motion_lifecycle"]
        )
        if controlled_park:
            if error_path and cause is not None and self._requires_immediate_fail_safe(cause):
                result["strategy"] = "fail_safe"
                result["park_status"] = "skipped_fail_safe"
                result["park_skip_reason"] = f"{type(cause).__name__}: {cause}"
            else:
                try:
                    health = self._wait_until_park_health(adapter)
                except Exception as exc:
                    result["strategy"] = "fail_safe"
                    result["park_status"] = "skipped_fail_safe"
                    result["park_skip_reason"] = f"{type(exc).__name__}: {exc}"
                    errors.append(f"controlled_park_health: {exc}")
                else:
                    stop_ok = True
                    if error_path and session["motion_lifecycle"]:
                        result["stop_attempted"] = True
                        stop_ok = attempt("stop", adapter.stop)
                        result["stop_status"] = "completed" if stop_ok else "failed"
                    if not stop_ok:
                        result["strategy"] = "fail_safe"
                        result["park_status"] = "skipped_stop_failed"
                    else:
                        try:
                            ready = self._wait_until_park_ready(adapter)
                        except Exception as exc:
                            result["strategy"] = "fail_safe"
                            result["park_status"] = "skipped_fail_safe"
                            result["park_skip_reason"] = f"{type(exc).__name__}: {exc}"
                            errors.append(f"controlled_park_ready: {exc}")
                        else:
                            try:
                                park = adapter.park_policy()
                                target = _six_finite(park["target_q"], "SDK park target_q")
                                self._validate_target_position_limits(target)
                                tolerance = float(park["position_tolerance_rad"])
                                result["strategy"] = "controlled_park"
                                result["park_target_source"] = park["source"]
                                result["park_target_q"] = list(target)
                                result["park_policy"] = {
                                    "max_velocity_rad_s": list(park["max_velocity_rad_s"]),
                                    "max_acceleration_rad_s2": list(
                                        park["max_acceleration_rad_s2"]
                                    ),
                                    "max_jerk_rad_s3": list(park["max_jerk_rad_s3"]),
                                    "position_tolerance_rad": tolerance,
                                }
                                initial_error = max(
                                    abs(actual - expected)
                                    for actual, expected in zip(ready.q, target)
                                )
                                if initial_error <= tolerance:
                                    final = ready
                                    result["park_status"] = "already_at_park"
                                else:
                                    result["park_attempted"] = True
                                    adapter.movej_to(
                                        target,
                                        max_velocity_rad_s=park["max_velocity_rad_s"],
                                        max_acceleration_rad_s2=park[
                                            "max_acceleration_rad_s2"
                                        ],
                                        max_jerk_rad_s3=park["max_jerk_rad_s3"],
                                        timeout_s=self.config["movej_timeout_s"],
                                    )
                                    result["park_movej_command_count"] = 1
                                    final = self._wait_until_target_settled(
                                        adapter,
                                        target,
                                        position_tolerance_rad=tolerance,
                                        velocity_tolerance_rad_s=float(
                                            self.config[
                                                "preposition_settle_velocity_tolerance_rad_s"
                                            ]
                                        ),
                                        label="park",
                                        feedback_age_limit_key="lower_feedback_timeout_ms",
                                    )
                                    result["park_status"] = "completed"
                                result["park_final_q"] = list(final.q)
                                result["park_max_position_error"] = max(
                                    abs(actual - expected)
                                    for actual, expected in zip(final.q, target)
                                )
                                result["park_max_velocity"] = max(
                                    abs(value) for value in final.qd
                                )
                            except Exception as exc:
                                result["strategy"] = "controlled_park"
                                result["park_status"] = "failed"
                                result["park_failure"] = f"{type(exc).__name__}: {exc}"
                                errors.append(f"controlled_park: {exc}")
        elif error_path and session["enabled"] and session["motion_lifecycle"]:
            result["stop_attempted"] = True
            result["stop_status"] = (
                "completed" if attempt("stop", adapter.stop) else "failed"
            )

        if session["enabled"]:
            result["disable_attempted"] = True
            result["disable_status"] = (
                "completed" if attempt("disable", adapter.disable) else "failed"
            )
            session["enabled"] = False
        if session["connected"]:
            result["close_attempted"] = True
            result["close_status"] = (
                "completed" if attempt("close", adapter.close) else "failed"
            )
            session["connected"] = False
        result["cleanup_errors"] = list(errors)
        return errors, result

    def _wait_until_park_health(self, adapter: RebotControlAdapter) -> CaptureSample:
        """Obtain a fresh trustworthy enabled state before any cleanup stop/park motion."""

        deadline = self.monotonic_fn() + float(self.config["command_timeout_s"])
        last_stale: RebotControlError | None = None
        while True:
            state = adapter.read_state()
            try:
                self._validate_state(
                    state,
                    require_servo_active=None,
                    feedback_age_limit_key="lower_feedback_timeout_ms",
                )
            except RebotControlError as exc:
                if self._is_recoverable_feedback_error(exc):
                    last_stale = exc
                    if self.monotonic_fn() < deadline:
                        self.sleep_fn(0.01)
                        continue
                raise
            if state.robot_mode in {"disabled", "fault"}:
                raise RebotControlError(
                    f"park health check rejected robot_mode={state.robot_mode}"
                )
            return state
        if last_stale is not None:
            raise last_stale
        raise RebotControlError("park health check could not obtain fresh state")

    def _wait_until_park_ready(self, adapter: RebotControlAdapter) -> CaptureSample:
        """Wait for a fresh enabled idle non-Servo state before dispatching park MoveJ."""

        deadline = self.monotonic_fn() + float(self.config["command_timeout_s"])
        last: CaptureSample | None = None
        while True:
            state = adapter.read_state()
            self._validate_state(
                state,
                require_servo_active=None,
                feedback_age_limit_key="lower_feedback_timeout_ms",
            )
            last = state
            if (
                state.robot_mode == "idle"
                and state.safety_state == "ready"
                and not state.servo_active
            ):
                return state
            if state.robot_mode in {"disabled", "fault"}:
                raise RebotControlError(
                    f"park readiness lost enabled state: robot_mode={state.robot_mode}"
                )
            now = self.monotonic_fn()
            if now >= deadline:
                break
            self.sleep_fn(min(0.02, max(0.0, deadline - now)))
        if last is None:
            raise RebotControlError("park readiness received no fresh state")
        raise RebotControlError(
            "timed out waiting for enabled idle fresh feedback before park "
            f"(robot_mode={last.robot_mode}, safety_state={last.safety_state}, "
            f"servo_active={last.servo_active})"
        )

    def _validate_target_position_limits(self, q_target: Sequence[float]) -> None:
        target = _six_finite(q_target, "MoveJ target")
        for index, value in enumerate(target):
            lower = self.config["joint_position_min_rad"][index]
            upper = self.config["joint_position_max_rad"][index]
            if not lower <= value <= upper:
                raise RebotControlError(
                    f"MoveJ target joint {index + 1} outside configured position limits"
                )

    @staticmethod
    def _is_recoverable_feedback_error(cause: BaseException) -> bool:
        message = str(cause)
        return message.startswith("feedback stale") or message.startswith(
            "invalid joint feedback"
        ) or message.startswith("transient feedback invalid exceeded")

    @staticmethod
    def _requires_immediate_fail_safe(cause: BaseException) -> bool:
        message = str(cause)
        markers = (
            "primary fault active",
            "unsafe lower safety_state",
            "state timeout",
            "state snapshot",
            "q feedback must contain six finite",
            "qd feedback must contain six finite",
            "feedback joint ",
            "lost enabled state",
            "servo_active=",
            "Servo send rejected",
            "servo_joint failed",
        )
        return any(marker in message for marker in markers)

    def _done(self, start: float, samples: int) -> bool:
        max_samples = self.config.get("max_samples")
        if max_samples is not None and samples >= int(max_samples):
            return True
        return self.monotonic_fn() - start >= float(self.config["duration_s"])

    def _sleep_period(self) -> None:
        self.sleep_fn(1.0 / float(self.config["control_rate_hz"]))


def _six_finite(values: Sequence[float], name: str) -> tuple[float, ...]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != JOINT_COUNT:
        raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain only finite values")
    return parsed


def _positive(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return parsed


def _port(value: Any, name: str) -> int:
    parsed = int(value)
    if not 0 < parsed <= 65535:
        raise ValueError(f"{name} must be in [1, 65535]")
    return parsed
