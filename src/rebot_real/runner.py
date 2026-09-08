from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import yaml

from .control_adapter import RebotControlAdapter, RebotControlError
from .hardware_recorder import HardwareExperimentRecorder
from .joint_jog import jog_target_at, validate_joint_jog
from .state_capture import CaptureSample, JOINT_COUNT
from .trajectory_artifact import ReplayArtifact, ReplaySample, load_replay_artifact, validate_preview_acceptance, validate_replay_runtime_limits


CONTROL_MODES = {"state_only", "servo_hold", "joint_jog", "excitation"}


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
        "start_position_tolerance_rad": 0.01,
        "maximum_feedback_age_ms": 50.0,
        "maximum_disabled_feedback_age_ms": None,
        "connect_timeout_s": 3.0,
        "command_timeout_s": 3.0,
        "state_timeout_s": 0.25,
        "trajectory_source": "frozen_replay_artifact",
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
    for name in ("allow_hardware", "allow_motion", "joint_mapping_verified"):
        if not isinstance(config[name], bool):
            raise ValueError(f"{name} must be a YAML boolean")
    config["j1_convention"] = str(config["j1_convention"]).strip() or "UNRESOLVED"
    if config["joint_mapping_scope"] not in {"servo_hold_only", "joint_jog"}:
        raise ValueError("joint_mapping_scope must be servo_hold_only or joint_jog")
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
    ):
        config[name] = list(_six_finite(config[name], name))
    for lower, upper in zip(config["joint_position_min_rad"], config["joint_position_max_rad"]):
        if lower >= upper:
            raise ValueError("each joint position minimum must be smaller than maximum")
    for name in (
        "maximum_command_velocity_rad_s",
        "maximum_command_acceleration_rad_s2",
        "maximum_command_jerk_rad_s3",
    ):
        if any(value <= 0.0 for value in config[name]):
            raise ValueError(f"{name} values must be positive")
    config["start_position_tolerance_rad"] = _positive(
        config["start_position_tolerance_rad"],
        "start_position_tolerance_rad",
    )
    for name in ("maximum_feedback_age_ms", "connect_timeout_s", "command_timeout_s", "state_timeout_s"):
        config[name] = _positive(config[name], name)
    disabled_age = config["maximum_disabled_feedback_age_ms"]
    config["maximum_disabled_feedback_age_ms"] = (
        config["maximum_feedback_age_ms"]
        if disabled_age is None
        else _positive(disabled_age, "maximum_disabled_feedback_age_ms")
    )

    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[2]
    output = Path(str(config["output_csv"])).expanduser()
    if not output.is_absolute():
        output = root / output
    config["output_csv"] = str(output)
    config["trajectory_source"] = str(config["trajectory_source"])
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
            cleanup_errors = self._shutdown(adapter, session, error_path=True)
            if recorder.run_result is not None:
                recorder.run_result.update(status="aborted", error=f"{type(exc).__name__}: {exc}", cleanup_errors=cleanup_errors)
            metadata = recorder.close()
            if cleanup_errors:
                raise RebotControlError(
                    f"{exc}; cleanup failures: {'; '.join(cleanup_errors)}"
                ) from exc
            raise
        cleanup_errors = self._shutdown(adapter, session, error_path=False)
        if recorder.run_result is not None:
            recorder.run_result.update(status="aborted" if cleanup_errors else "completed", cleanup_errors=cleanup_errors)
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
        adapter.enable()
        session["enabled"] = True

        # Re-read immediately before Servo entry. This exact measured q is the hold
        # target; no configured home pose is used for hardware hold.
        hold_state = adapter.read_state()
        self._validate_state(hold_state, require_servo_active=False)
        hold_target = tuple(hold_state.q)
        adapter.enter_servo()
        session["servo"] = True
        session["motion_lifecycle"] = True

        state = adapter.read_state()
        self._validate_state(state, require_servo_active=True)
        start = self.monotonic_fn()
        samples = 0
        while not self._done(start, samples):
            self._validate_command(hold_target, state.q)
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
            self._sleep_period()
            if self._done(start, samples):
                break
            state = adapter.read_state()
            self._validate_state(state, require_servo_active=True)

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
        state = adapter.read_state()
        self._validate_state(state, require_servo_active=False)
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
        session["servo"] = session["motion_lifecycle"] = True
        adapter.enter_servo()
        state = adapter.read_state()
        self._validate_state(state, require_servo_active=True)
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
            # Keep both existing measured-state delta and consecutive-command gates.
            self._validate_command(target, state.q)
            self._validate_command(target, previous_target)
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
        """Replay every frozen q_ref sample exactly once; never interpolate or resample."""

        initial = adapter.read_state()
        self._validate_state(
            initial,
            require_servo_active=False,
            feedback_age_limit_key="maximum_disabled_feedback_age_ms",
        )
        if initial.robot_mode != "disabled" or initial.safety_state != "disabled":
            raise RebotControlError("excitation requires an initially disabled robot")
        self._validate_excitation_start(initial.q, artifact.q_start)

        adapter.enable()
        session["enabled"] = True
        ready = adapter.read_state()
        self._validate_state(ready, require_servo_active=False)
        self._validate_excitation_start(ready.q, artifact.q_start)

        adapter.enter_servo()
        session["servo"] = True
        session["motion_lifecycle"] = True
        state = adapter.read_state()
        self._validate_state(state, require_servo_active=True)

        previous_sample: ReplaySample | None = None
        previous_target = artifact.q_start
        for sample_index, sample in enumerate(artifact.samples):
            self._validate_state(state, require_servo_active=True)
            self._validate_excitation_dynamics(sample, previous_sample, artifact.sample_rate_hz)
            self._validate_command(sample.q_ref, state.q)
            if sample_index > 0:
                self._validate_command(sample.q_ref, previous_target)

            timestamp_ns, sequence = adapter.send_servo_target(sample.q_ref)
            recorder.record(
                state,
                q_cmd=sample.q_ref,
                timestamp_host_command_ns=timestamp_ns,
                servo_sequence=sequence,
                command_valid=True,
                control_mode="excitation",
            )
            previous_sample = sample
            previous_target = sample.q_ref
            if sample_index + 1 < len(artifact.samples):
                self._sleep_period()
                state = adapter.read_state()

        adapter.exit_servo()
        session["servo"] = False

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

    def _validate_excitation_dynamics(
        self,
        sample: ReplaySample,
        previous: ReplaySample | None,
        sample_rate_hz: float,
    ) -> None:
        for index in range(JOINT_COUNT):
            if (
                abs(sample.qd_ref[index])
                > self.config["maximum_command_velocity_rad_s"][index] + 1e-12
            ):
                raise RebotControlError(
                    f"excitation frozen J{index + 1} velocity limit exceeded"
                )
            if (
                abs(sample.qdd_ref[index])
                > self.config["maximum_command_acceleration_rad_s2"][index] + 1e-12
            ):
                raise RebotControlError(
                    f"excitation frozen J{index + 1} acceleration limit exceeded"
                )
            if previous is not None:
                jerk = abs(
                    (sample.qdd_ref[index] - previous.qdd_ref[index])
                    * sample_rate_hz
                )
                if (
                    jerk
                    > self.config["maximum_command_jerk_rad_s3"][index] + 1e-9
                ):
                    raise RebotControlError(
                        f"excitation frozen J{index + 1} jerk limit exceeded"
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

    def _validate_state(
        self,
        state: CaptureSample,
        *,
        require_servo_active: bool | None,
        feedback_age_limit_key: str = "maximum_feedback_age_ms",
    ) -> None:
        if not all(state.feedback_valid):
            raise RebotControlError("invalid joint feedback blocks hardware runner")
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
        for index, value in enumerate(state.q):
            lower = self.config["joint_position_min_rad"][index]
            upper = self.config["joint_position_max_rad"][index]
            if not lower <= value <= upper:
                raise RebotControlError(f"feedback joint {index + 1} outside configured position limits")
        if require_servo_active is not None and bool(state.servo_active) != require_servo_active:
            raise RebotControlError(
                f"servo_active={state.servo_active} but expected {require_servo_active}"
            )

    def _validate_command(self, q_target: Sequence[float], q_reference: Sequence[float]) -> None:
        target = _six_finite(q_target, "q_target")
        reference = _six_finite(q_reference, "q_reference")
        rate = float(self.config["control_rate_hz"])
        for index, value in enumerate(target):
            lower = self.config["joint_position_min_rad"][index]
            upper = self.config["joint_position_max_rad"][index]
            if not lower <= value <= upper:
                raise RebotControlError(f"command joint {index + 1} outside configured position limits")
            max_delta = self.config["maximum_command_velocity_rad_s"][index] / rate
            if abs(value - reference[index]) > max_delta + 1e-12:
                raise RebotControlError(
                    f"command joint {index + 1} delta exceeds velocity-derived limit {max_delta} rad"
                )

    def _shutdown(
        self,
        adapter: RebotControlAdapter,
        session: dict[str, bool],
        *,
        error_path: bool,
    ) -> list[str]:
        errors: list[str] = []

        def attempt(name: str, action: Callable[[], None]) -> None:
            try:
                action()
            except Exception as exc:
                errors.append(f"{name}: {exc}")

        if session["servo"]:
            attempt("exit_servo", adapter.exit_servo)
            session["servo"] = False
        if error_path and session["enabled"] and session["motion_lifecycle"]:
            attempt("stop", adapter.stop)
        if session["enabled"]:
            attempt("disable", adapter.disable)
            session["enabled"] = False
        if session["connected"]:
            attempt("close", adapter.close)
            session["connected"] = False
        return errors

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
