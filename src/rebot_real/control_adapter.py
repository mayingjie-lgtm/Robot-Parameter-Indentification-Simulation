from __future__ import annotations

from dataclasses import replace
import importlib
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Sequence

from .state_capture import CaptureSample, JOINT_COUNT, map_sdk_state


class RebotControlError(RuntimeError):
    """Normalize external SDK control/transport failures for the hardware runner."""


class RebotControlAdapter:
    """Minimal adapter around the audited reBot ``ArmClient`` public API.

    Public runner coordinates are mapped as ``q = direction * q_sdk + offset``.
    Motion is never authorized by this class; the runner owns the hardware/motion
    gates and calls this adapter only after those gates pass.
    """

    def __init__(
        self,
        *,
        sdk_root: str | Path,
        host: str,
        tcp_port: int,
        udp_port: int,
        joint_direction: Sequence[float],
        joint_offset_rad: Sequence[float],
        connect_timeout_s: float,
        command_timeout_s: float,
        state_timeout_s: float,
        client_factory: Callable[..., Any] | None = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        monotonic_ns_fn: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.sdk_root = Path(sdk_root).expanduser() if str(sdk_root) else Path()
        self.host = str(host)
        self.tcp_port = int(tcp_port)
        self.udp_port = int(udp_port)
        self.joint_direction = _six_finite(joint_direction, "joint_direction")
        self.joint_offset_rad = _six_finite(joint_offset_rad, "joint_offset_rad")
        if any(abs(value) != 1.0 for value in self.joint_direction):
            raise ValueError("joint_direction values must be exactly -1 or 1")
        self.connect_timeout_s = _positive_finite(connect_timeout_s, "connect_timeout_s")
        self.command_timeout_s = _positive_finite(command_timeout_s, "command_timeout_s")
        self.state_timeout_s = _positive_finite(state_timeout_s, "state_timeout_s")
        self._sleep_fn = sleep_fn
        self._monotonic_ns_fn = monotonic_ns_fn
        self._client_factory = client_factory
        self._client: Any | None = None
        self._last_state_version: int | None = None
        self._last_state_sequence: int | None = None
        self._next_servo_sequence = 1

    def connect(self) -> None:
        """Create and connect the audited SDK client without enabling motors."""

        if self._client is not None:
            return
        try:
            factory = self._client_factory or _load_arm_client_class(self.sdk_root)
            self._client = factory(
                host=self.host,
                tcp_port=self.tcp_port,
                udp_port=self.udp_port,
                disable_on_close=False,
            )
            self._client.connect(timeout_s=self.connect_timeout_s)
        except Exception as exc:
            self._client = None
            raise RebotControlError(f"connect failed: {exc}") from exc

    @property
    def latest_state(self) -> CaptureSample | None:
        """Return the newest SDK state snapshot without waiting for a new packet."""

        if self._client is None:
            raise RebotControlError("ArmClient is not connected")
        snapshot = _state_snapshot(self._client)
        if snapshot is None:
            return None
        state, host_rx_ns, version = snapshot
        return self._map_state(state, host_rx_ns, version, update_last=False)

    def read_state(self) -> CaptureSample:
        """Wait for a new public state snapshot and map it into runner coordinates."""

        if self._client is None:
            raise RebotControlError("ArmClient is not connected")
        deadline_ns = self._monotonic_ns_fn() + int(self.state_timeout_s * 1e9)
        last_error: Exception | None = None
        while self._monotonic_ns_fn() < deadline_ns:
            try:
                snapshot = _state_snapshot(self._client)
            except Exception as exc:
                last_error = exc
                break
            if snapshot is not None:
                state, host_rx_ns, version = snapshot
                if self._is_new_state(state, version):
                    return self._map_state(state, host_rx_ns, version, update_last=True)
            self._sleep_fn(min(0.002, self.state_timeout_s / 10.0))
        detail = f": {last_error}" if last_error is not None else ""
        raise RebotControlError(f"state timeout after {self.state_timeout_s:.3f}s{detail}")

    def configure_movej_pvt(self) -> None:
        """Apply the SDK's existing six-axis MoveJ PVT policy before enable."""

        client = self._require_client()
        try:
            policy = getattr(client, "movej_pvt_policy", None)
            if policy is None:
                policy = _load_movej_pvt_policy(self.sdk_root)
            reply = client.configure_pvt(
                current_bandwidth_hz=_six_finite(
                    policy["current_bandwidth_hz"], "PVT current_bandwidth_hz"
                ),
                velocity_kp=_six_finite(policy["velocity_kp"], "PVT velocity_kp"),
                velocity_ki=_six_finite(policy["velocity_ki"], "PVT velocity_ki"),
                position_kp=_six_finite(policy["position_kp"], "PVT position_kp"),
                position_ki=_six_finite(policy["position_ki"], "PVT position_ki"),
                current_limit_normalized=_six_finite(
                    policy["current_limit_normalized"], "PVT current_limit_normalized"
                ),
                timeout_s=self.command_timeout_s,
            )
        except Exception as exc:
            raise RebotControlError(f"configure_pvt failed: {exc}") from exc
        status = getattr(getattr(reply, "status", None), "value", getattr(reply, "status", None))
        if status not in (None, "done"):
            raise RebotControlError(f"configure_pvt returned unexpected status {status!r}")

    def enable(self) -> None:
        """Enable the lower driver through the SDK command/reply path."""

        self._call_reply("enable", timeout_s=self.command_timeout_s)

    def movej_to(
        self,
        q_target: Sequence[float],
        *,
        max_velocity_rad_s: Sequence[float],
        max_acceleration_rad_s2: Sequence[float],
        max_jerk_rad_s3: Sequence[float],
        timeout_s: float,
    ) -> None:
        """Synchronously preposition to one canonical target via SDK ``ArmClient.movej``."""

        client = self._require_client()
        q_public = _six_finite(q_target, "q_target")
        velocity = _six_positive(max_velocity_rad_s, "max_velocity_rad_s")
        acceleration = _six_positive(max_acceleration_rad_s2, "max_acceleration_rad_s2")
        jerk = _six_positive(max_jerk_rad_s3, "max_jerk_rad_s3")
        move_timeout = _positive_finite(timeout_s, "movej timeout_s")
        q_sdk = tuple(
            (q_public[index] - self.joint_offset_rad[index]) / self.joint_direction[index]
            for index in range(JOINT_COUNT)
        )
        try:
            reply = client.movej(
                q_sdk,
                max_velocity_rad_s=velocity,
                max_acceleration_rad_s2=acceleration,
                max_jerk_rad_s3=jerk,
                timeout_s=move_timeout,
            )
        except Exception as exc:
            raise RebotControlError(f"movej failed: {exc}") from exc
        status = getattr(getattr(reply, "status", None), "value", getattr(reply, "status", None))
        if status not in (None, "done"):
            raise RebotControlError(f"movej returned unexpected status {status!r}")

    def enter_servo(self) -> None:
        """Claim lower Servo ownership and reset the explicit local sequence."""

        self._call_reply("enter_servo", timeout_s=self.command_timeout_s)
        self._next_servo_sequence = 1

    def send_servo_target(self, q_target: Sequence[float]) -> tuple[int, int]:
        """Send one six-axis position target with explicit timestamp and sequence.

        Returns ``(host_timestamp_ns, servo_sequence)`` exactly matching the values
        passed to ``ArmClient.servo_joint``. No velocity, gain, or torque command is
        synthesized because the audited Servo contract accepts position only.
        """

        client = self._require_client()
        q_public = _six_finite(q_target, "q_target")
        q_sdk = tuple(
            (q_public[index] - self.joint_offset_rad[index]) / self.joint_direction[index]
            for index in range(JOINT_COUNT)
        )
        timestamp_ns = int(self._monotonic_ns_fn())
        sequence = self._next_servo_sequence
        if timestamp_ns <= 0:
            raise RebotControlError("host servo timestamp must be positive")
        try:
            _, returned_sequence = client.servo_joint(
                q_sdk,
                host_timestamp_ns=timestamp_ns,
                servo_sequence=sequence,
            )
        except Exception as exc:
            raise RebotControlError(f"servo_joint failed: {exc}") from exc
        if int(returned_sequence) != sequence:
            raise RebotControlError(
                f"servo_joint sequence mismatch: sent {sequence}, SDK returned {returned_sequence}"
            )
        self._next_servo_sequence += 1
        return timestamp_ns, sequence

    def exit_servo(self) -> None:
        """Release lower Servo ownership through the audited SDK command."""

        self._call_reply("exit_servo", timeout_s=self.command_timeout_s)

    def stop(self) -> None:
        """Request the lower controller safe stop command."""

        self._call_reply("stop", timeout_s=self.command_timeout_s)

    def disable(self) -> None:
        """Disable motors through the audited SDK command/reply path."""

        self._call_reply("disable", timeout_s=self.command_timeout_s)

    def close(self) -> None:
        """Close SDK sockets; no implicit upper-client disable is requested."""

        client, self._client = self._client, None
        if client is None:
            return
        try:
            client.close()
        except Exception as exc:
            raise RebotControlError(f"close failed: {exc}") from exc

    def _require_client(self) -> Any:
        if self._client is None:
            raise RebotControlError("ArmClient is not connected")
        return self._client

    def _call_reply(self, method_name: str, *, timeout_s: float) -> Any:
        client = self._require_client()
        method = getattr(client, method_name)
        try:
            reply = method(timeout_s=timeout_s)
        except Exception as exc:
            raise RebotControlError(f"{method_name} failed: {exc}") from exc
        status = getattr(getattr(reply, "status", None), "value", getattr(reply, "status", None))
        if status not in (None, "done"):
            raise RebotControlError(f"{method_name} returned unexpected status {status!r}")
        return reply

    def _is_new_state(self, state: Any, version: int | None) -> bool:
        if version is not None:
            return self._last_state_version is None or version > self._last_state_version
        sequence = int(state.sequence)
        return self._last_state_sequence is None or sequence != self._last_state_sequence

    def _map_state(
        self,
        state: Any,
        host_rx_ns: int,
        version: int | None,
        *,
        update_last: bool,
    ) -> CaptureSample:
        mapped = map_sdk_state(
            state,
            sample_index=0,
            timestamp_host_rx_ns=int(host_rx_ns),
            previous_sequence=self._last_state_sequence,
        )
        q = tuple(
            self.joint_direction[index] * mapped.q[index] + self.joint_offset_rad[index]
            for index in range(JOINT_COUNT)
        )
        qd = tuple(
            self.joint_direction[index] * mapped.qd[index] for index in range(JOINT_COUNT)
        )
        effort = tuple(
            self.joint_direction[index] * mapped.effort_reported[index]
            for index in range(JOINT_COUNT)
        )
        if update_last:
            self._last_state_version = version
            self._last_state_sequence = mapped.udp_sequence
        return replace(mapped, q=q, qd=qd, effort_reported=effort)


def _load_arm_client_class(sdk_root: Path) -> Callable[..., Any]:
    """Import ``ArmClient`` from the configured external SDK root without copying it."""

    if not str(sdk_root):
        raise FileNotFoundError("sdk_root is empty")
    sdk_root_path = sdk_root.expanduser().resolve()
    python_roots = (
        sdk_root_path / "upper" / "python",
        sdk_root_path / "src",
    )
    client_path = next(
        (
            python_root / "wlsea_arm_sdk" / "client.py"
            for python_root in python_roots
            if (python_root / "wlsea_arm_sdk" / "client.py").is_file()
        ),
        None,
    )
    if client_path is None:
        expected_paths = ", ".join(
            str(python_root / "wlsea_arm_sdk" / "client.py")
            for python_root in python_roots
        )
        raise FileNotFoundError(f"SDK ArmClient not found; checked: {expected_paths}")
    python_root = client_path.parent.parent

    existing = sys.modules.get("wlsea_arm_sdk.client")
    if existing is not None:
        existing_path = Path(getattr(existing, "__file__", "")).resolve()
        if existing_path != client_path.resolve():
            raise RuntimeError(
                "wlsea_arm_sdk.client is already imported from a different SDK root: "
                f"{existing_path}"
            )
        return getattr(existing, "ArmClient")

    inserted = str(python_root) not in sys.path
    if inserted:
        sys.path.insert(0, str(python_root))
    try:
        module = importlib.import_module("wlsea_arm_sdk.client")
    finally:
        if inserted:
            try:
                sys.path.remove(str(python_root))
            except ValueError:
                pass
    return getattr(module, "ArmClient")


def _load_movej_pvt_policy(sdk_root: Path) -> dict[str, tuple[float, ...]]:
    """Load the authoritative PVT constants from this SDK checkout at runtime."""

    if not str(sdk_root):
        raise FileNotFoundError("sdk_root is empty; cannot load SDK MoveJ PVT policy")
    sdk_root_path = sdk_root.expanduser().resolve()
    python_roots = (sdk_root_path / "upper" / "python", sdk_root_path / "src")
    module_path = next(
        (
            root / "wlsea_arm_sdk" / "movej_runtime.py"
            for root in python_roots
            if (root / "wlsea_arm_sdk" / "movej_runtime.py").is_file()
        ),
        None,
    )
    if module_path is None:
        raise FileNotFoundError("SDK wlsea_arm_sdk/movej_runtime.py was not found")
    python_root = module_path.parent.parent
    existing = sys.modules.get("wlsea_arm_sdk.movej_runtime")
    if existing is not None:
        existing_path = Path(getattr(existing, "__file__", "")).resolve()
        if existing_path != module_path.resolve():
            raise RuntimeError(
                "wlsea_arm_sdk.movej_runtime is already imported from a different SDK root: "
                f"{existing_path}"
            )
    inserted = str(python_root) not in sys.path
    if inserted:
        sys.path.insert(0, str(python_root))
    try:
        module = importlib.import_module("wlsea_arm_sdk.movej_runtime")
    finally:
        if inserted:
            try:
                sys.path.remove(str(python_root))
            except ValueError:
                pass
    return {
        "current_bandwidth_hz": _six_finite(
            module.PVT_CURRENT_BANDWIDTH_HZ, "SDK PVT_CURRENT_BANDWIDTH_HZ"
        ),
        "velocity_kp": _six_finite(module.PVT_VELOCITY_KP, "SDK PVT_VELOCITY_KP"),
        "velocity_ki": _six_finite(module.PVT_VELOCITY_KI, "SDK PVT_VELOCITY_KI"),
        "position_kp": _six_finite(module.PVT_POSITION_KP, "SDK PVT_POSITION_KP"),
        "position_ki": _six_finite(module.PVT_POSITION_KI, "SDK PVT_POSITION_KI"),
        "current_limit_normalized": _six_finite(
            module.PVT_CURRENT_LIMIT_NORMALIZED, "SDK PVT_CURRENT_LIMIT_NORMALIZED"
        ),
    }


def _state_snapshot(client: Any) -> tuple[Any, int, int | None] | None:
    """Read the SDK StateStore timestamp when available, with a safe compatibility fallback."""

    store = getattr(client, "state_store", None)
    snapshot = getattr(store, "latest", None) if store is not None else None
    if snapshot is not None:
        actual = snapshot.actual
        return actual.state, int(actual.received_monotonic_ns), int(snapshot.version)
    state = getattr(client, "latest_state", None)
    if state is None:
        return None
    # Older/Mock-compatible clients may not expose StateStore receive timing. This
    # fallback is explicitly disclosed in metadata by the runner when used.
    return state, time.monotonic_ns(), None


def _six_finite(values: Sequence[float], name: str) -> tuple[float, ...]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != JOINT_COUNT:
        raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain only finite values")
    return parsed


def _six_positive(values: Sequence[float], name: str) -> tuple[float, ...]:
    parsed = _six_finite(values, name)
    if any(value <= 0.0 for value in parsed):
        raise ValueError(f"{name} values must be positive")
    return parsed


def _positive_finite(value: float, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return parsed
