from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import yaml

from .control_adapter import RebotControlAdapter, RebotControlError
from .hardware_recorder import HardwareExperimentRecorder
from .state_capture import CaptureSample, JOINT_COUNT


CONTROL_MODES = {"state_only", "servo_hold", "excitation"}


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
        "joint_direction": [1.0] * JOINT_COUNT,
        "joint_offset_rad": [0.0] * JOINT_COUNT,
        "joint_position_min_rad": [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14],
        "joint_position_max_rad": [2.8, 0.0, 0.0, 1.57, 1.57, 3.14],
        "maximum_command_velocity_rad_s": [0.05] * JOINT_COUNT,
        "maximum_feedback_age_ms": 50.0,
        "connect_timeout_s": 3.0,
        "command_timeout_s": 3.0,
        "state_timeout_s": 0.25,
        "trajectory_source": "pending_cxx_fourier_integration",
        "trajectory_hash": None,
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
    config["joint_direction"] = list(_six_finite(config["joint_direction"], "joint_direction"))
    if any(abs(value) != 1.0 for value in config["joint_direction"]):
        raise ValueError("joint_direction values must be exactly -1 or 1")
    for name in (
        "joint_offset_rad",
        "joint_position_min_rad",
        "joint_position_max_rad",
        "maximum_command_velocity_rad_s",
    ):
        config[name] = list(_six_finite(config[name], name))
    for lower, upper in zip(config["joint_position_min_rad"], config["joint_position_max_rad"]):
        if lower >= upper:
            raise ValueError("each joint position minimum must be smaller than maximum")
    if any(value <= 0.0 for value in config["maximum_command_velocity_rad_s"]):
        raise ValueError("maximum_command_velocity_rad_s values must be positive")
    for name in (
        "maximum_feedback_age_ms",
        "connect_timeout_s",
        "command_timeout_s",
        "state_timeout_s",
    ):
        config[name] = _positive(config[name], name)

    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[2]
    output = Path(str(config["output_csv"])).expanduser()
    if not output.is_absolute():
        output = root / output
    config["output_csv"] = str(output)
    config["trajectory_source"] = str(config["trajectory_source"])
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
        if mode == "excitation":
            raise RuntimeError(
                "excitation trajectory source integration remains pending; the trusted Fourier "
                "implementation is C++ and is intentionally not duplicated in Python"
            )

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
            else:
                raise AssertionError(f"unhandled control mode {mode}")
        except Exception as exc:
            cleanup_errors = self._shutdown(adapter, session, error_path=True)
            metadata = recorder.close()
            if cleanup_errors:
                raise RebotControlError(
                    f"{exc}; cleanup failures: {'; '.join(cleanup_errors)}"
                ) from exc
            raise
        cleanup_errors = self._shutdown(adapter, session, error_path=False)
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
            self._validate_state(state, require_servo_active=None)
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
        self._validate_state(initial, require_servo_active=False)
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

    def _assert_session_authorized(self, mode: str) -> None:
        if not self.mock_backend and not self.config["allow_hardware"]:
            raise PermissionError("allow_hardware=false blocks creation of a real ArmClient session")
        if mode in {"servo_hold", "excitation"}:
            if not self.config["allow_motion"]:
                raise PermissionError("allow_motion=false blocks all motor-changing commands")
            if not self.config["joint_mapping_verified"]:
                raise PermissionError("joint_mapping_verified=false blocks all motor-changing commands")
            if self.config["j1_convention"] == "UNRESOLVED":
                raise PermissionError("j1_convention=UNRESOLVED blocks all motor-changing commands")

    def _validate_state(
        self,
        state: CaptureSample,
        *,
        require_servo_active: bool | None,
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
        max_age = float(self.config["maximum_feedback_age_ms"])
        if any(value > max_age for value in state.feedback_age_ms):
            raise RebotControlError(
                f"feedback stale: age exceeds maximum_feedback_age_ms={max_age}"
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
