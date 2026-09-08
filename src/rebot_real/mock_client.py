from __future__ import annotations

import math
from types import SimpleNamespace
import time
from typing import Iterable, Sequence


JOINT_COUNT = 6


class MockArmClient:
    """Deterministic in-process ArmClient substitute with fault injection for offline tests."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        tcp_port: int = 5000,
        udp_port: int = 5001,
        disable_on_close: bool = False,
        *,
        fail_on: Iterable[str] = (),
        no_state: bool = False,
        feedback_valid: Sequence[bool] | None = None,
        torque_valid: Sequence[bool] | None = None,
        feedback_age_ms: Sequence[float] | None = None,
        position_rad: Sequence[float] | None = None,
        velocity_rad_s: Sequence[float] | None = None,
        torque_nm: Sequence[float] | None = None,
        primary_fault_code: int = 0,
        safety_state: str = "disabled",
        disconnect_after_state_reads: int | None = None,
        follow_servo_targets: bool = False,
        follow_movej_targets: bool = False,
        settle_after_movej: bool = True,
        after_movej_feedback_valid: Sequence[bool] | None = None,
        after_movej_feedback_age_ms: Sequence[float] | None = None,
        after_movej_primary_fault_code: int | None = None,
    ) -> None:
        self.host = host
        self.tcp_port = int(tcp_port)
        self.udp_port = int(udp_port)
        self.disable_on_close = bool(disable_on_close)
        self.fail_on = set(fail_on)
        self.no_state = bool(no_state)
        self.feedback_valid = tuple(feedback_valid or [True] * JOINT_COUNT)
        self.torque_valid = tuple(torque_valid or [True] * JOINT_COUNT)
        self.feedback_age_ms = tuple(feedback_age_ms or [1.0] * JOINT_COUNT)
        self.position_rad = list(position_rad or [0.0] * JOINT_COUNT)
        self.velocity_rad_s = list(velocity_rad_s or [0.0] * JOINT_COUNT)
        self.torque_nm = list(torque_nm or [0.0] * JOINT_COUNT)
        self.primary_fault_code = int(primary_fault_code)
        self.safety_state = str(safety_state)
        self.disconnect_after_state_reads = disconnect_after_state_reads
        self.follow_servo_targets = bool(follow_servo_targets)
        self.follow_movej_targets = bool(follow_movej_targets)
        self.settle_after_movej = bool(settle_after_movej)
        self.after_movej_feedback_valid = (
            None if after_movej_feedback_valid is None else tuple(after_movej_feedback_valid)
        )
        self.after_movej_feedback_age_ms = (
            None if after_movej_feedback_age_ms is None else tuple(after_movej_feedback_age_ms)
        )
        self.after_movej_primary_fault_code = after_movej_primary_fault_code
        self.calls: list[str] = []
        self.pvt_configurations: list[dict[str, tuple[float, ...]]] = []
        self.movej_targets: list[tuple[float, ...]] = []
        self.movej_command_records: list[dict[str, object]] = []
        self.servo_targets: list[tuple[float, ...]] = []
        self.servo_command_records: list[tuple[int, int, tuple[float, ...]]] = []
        self.connected = False
        self.enabled = False
        self.servo_active = False
        self.servo_mode = "disabled"
        self.robot_mode = "disabled"
        self._state_reads = 0
        self._request_id = 1
        self._last_servo_sequence = 0
        self.movej_pvt_policy = {
            "current_bandwidth_hz": (1000.0,) * JOINT_COUNT,
            "velocity_kp": (0.01,) * JOINT_COUNT,
            "velocity_ki": (0.001,) * JOINT_COUNT,
            "position_kp": (100.0,) * JOINT_COUNT,
            "position_ki": (0.0,) * JOINT_COUNT,
            "current_limit_normalized": (0.1,) * JOINT_COUNT,
        }

        for name, values in (
            ("feedback_valid", self.feedback_valid),
            ("torque_valid", self.torque_valid),
            ("feedback_age_ms", self.feedback_age_ms),
            ("position_rad", self.position_rad),
            ("velocity_rad_s", self.velocity_rad_s),
            ("torque_nm", self.torque_nm),
        ):
            if len(values) != JOINT_COUNT:
                raise ValueError(f"{name} must contain six values")

    def connect(self, timeout_s: float = 3.0) -> None:
        """Record a successful connection or inject a connect failure."""

        self.calls.append("connect")
        self._fail("connect")
        self.connected = True

    @property
    def state_store(self):
        """Return a fresh StateStore-compatible snapshot on every runner read."""

        self.calls.append("read_state")
        self._state_reads += 1
        if self.disconnect_after_state_reads is not None and self._state_reads > self.disconnect_after_state_reads:
            raise ConnectionError("mock connection lost")
        if self.no_state:
            return SimpleNamespace(latest=None)
        now_ns = time.monotonic_ns()
        state = self._make_state(now_ns)
        actual = SimpleNamespace(state=state, received_monotonic_ns=now_ns)
        snapshot = SimpleNamespace(version=self._state_reads, actual=actual)
        return SimpleNamespace(latest=snapshot)

    @property
    def latest_state(self):
        if self.no_state:
            return None
        return self._make_state(time.monotonic_ns())

    def configure_pvt(
        self,
        current_bandwidth_hz,
        velocity_kp,
        velocity_ki,
        position_kp,
        position_ki,
        current_limit_normalized,
        timeout_s: float = 5.0,
    ):
        """Record the six-axis PVT configuration used before MoveJ."""

        self.calls.append("configure_pvt")
        self._fail("configure_pvt")
        self.pvt_configurations.append({
            "current_bandwidth_hz": tuple(float(v) for v in current_bandwidth_hz),
            "velocity_kp": tuple(float(v) for v in velocity_kp),
            "velocity_ki": tuple(float(v) for v in velocity_ki),
            "position_kp": tuple(float(v) for v in position_kp),
            "position_ki": tuple(float(v) for v in position_ki),
            "current_limit_normalized": tuple(float(v) for v in current_limit_normalized),
        })
        return self._done_reply()

    def enable(self, timeout_s: float = 5.0):
        """Simulate lower enable and READY transition."""

        self.calls.append("enable")
        self._fail("enable")
        self.enabled = True
        self.robot_mode = "idle"
        self.safety_state = "ready"
        return self._done_reply()

    def movej(
        self,
        target_position_rad: Iterable[float],
        max_velocity_rad_s=0.5,
        max_acceleration_rad_s2=1.0,
        max_jerk_rad_s3=40.0,
        timeout_s: float = 30.0,
        sent_callback=None,
        accepted_callback=None,
    ):
        """Record an SDK-style synchronous MoveJ independently from Servo targets."""

        self.calls.append("movej")
        if "movej_timeout" in self.fail_on:
            raise TimeoutError("mock movej timeout")
        self._fail("movej")
        if not self.enabled or self.servo_active:
            raise RuntimeError("mock movej requires enabled non-Servo state")
        target = tuple(float(value) for value in target_position_rad)
        if len(target) != JOINT_COUNT or not all(math.isfinite(value) for value in target):
            raise ValueError("mock MoveJ target must contain six finite values")
        velocity = _joint_parameter(max_velocity_rad_s)
        acceleration = _joint_parameter(max_acceleration_rad_s2)
        jerk = _joint_parameter(max_jerk_rad_s3)
        self.movej_targets.append(target)
        self.movej_command_records.append({
            "target": target,
            "max_velocity_rad_s": velocity,
            "max_acceleration_rad_s2": acceleration,
            "max_jerk_rad_s3": jerk,
            "timeout_s": float(timeout_s),
        })
        if sent_callback is not None:
            sent_callback()
        if accepted_callback is not None:
            accepted_callback()
        if self.follow_movej_targets:
            self.position_rad = list(target)
        if self.settle_after_movej:
            self.velocity_rad_s = [0.0] * JOINT_COUNT
        if self.after_movej_feedback_valid is not None:
            self.feedback_valid = tuple(self.after_movej_feedback_valid)
        if self.after_movej_feedback_age_ms is not None:
            self.feedback_age_ms = tuple(self.after_movej_feedback_age_ms)
        if self.after_movej_primary_fault_code is not None:
            self.primary_fault_code = int(self.after_movej_primary_fault_code)
        self.robot_mode = "idle"
        self.safety_state = "ready"
        self.servo_active = False
        return self._done_reply()

    def enter_servo(self, timeout_s: float = 3.0):
        """Simulate Servo ownership after enable."""

        self.calls.append("enter_servo")
        self._fail("enter_servo")
        if not self.enabled:
            raise RuntimeError("mock enter_servo requires enable")
        self.servo_active = True
        self.servo_mode = "ready"
        return self._done_reply()

    def servo_joint(
        self,
        target_position_rad: Iterable[float],
        *,
        host_timestamp_ns: int | None = None,
        servo_sequence: int | None = None,
    ) -> tuple[int, int]:
        """Record exact position-only Servo fields or inject a send failure."""

        self.calls.append("servo_joint")
        self._fail("servo_joint")
        if not self.servo_active:
            raise RuntimeError("mock servo_joint requires active Servo")
        target = tuple(float(value) for value in target_position_rad)
        if len(target) != JOINT_COUNT or not all(math.isfinite(value) for value in target):
            raise ValueError("mock Servo target must contain six finite values")
        sequence = int(servo_sequence or (self._last_servo_sequence + 1))
        timestamp = int(host_timestamp_ns or time.monotonic_ns())
        self._last_servo_sequence = sequence
        self.servo_mode = "servo"
        self.servo_targets.append(target)
        self.servo_command_records.append((timestamp, sequence, target))
        if self.follow_servo_targets:
            # Ideal encoder response only: no dynamics, gravity, backlash or noise.
            self.position_rad = list(target)
        request_id = self._request_id
        self._request_id += 1
        return request_id, sequence

    def exit_servo(self, timeout_s: float = 3.0):
        """Simulate normal Servo ownership release."""

        self.calls.append("exit_servo")
        self._fail("exit_servo")
        self.servo_active = False
        self.servo_mode = "disabled"
        return self._done_reply()

    def stop(self, timeout_s: float = 3.0):
        """Record the best-effort stop command."""

        self.calls.append("stop")
        self._fail("stop")
        return self._done_reply()

    def disable(self, timeout_s: float = 5.0):
        """Simulate motor disable."""

        self.calls.append("disable")
        self._fail("disable")
        self.enabled = False
        self.servo_active = False
        self.servo_mode = "disabled"
        self.robot_mode = "disabled"
        self.safety_state = "disabled"
        return self._done_reply()

    def close(self) -> None:
        """Close the Mock connection without synthesizing another disable call."""

        self.calls.append("close")
        self._fail("close")
        self.connected = False

    def _make_state(self, now_ns: int):
        return SimpleNamespace(
            position_rad=tuple(self.position_rad),
            velocity_rad_s=tuple(self.velocity_rad_s),
            torque_nm=tuple(self.torque_nm),
            feedback_valid=tuple(self.feedback_valid),
            torque_valid=tuple(self.torque_valid),
            feedback_age_ms=tuple(self.feedback_age_ms),
            sequence=self._state_reads,
            monotonic_time_ns=int(now_ns),
            robot_mode=self.robot_mode,
            safety_state=self.safety_state,
            primary_fault_code=self.primary_fault_code,
            servo_active=self.servo_active,
            servo_mode=self.servo_mode,
        )

    def _fail(self, operation: str) -> None:
        if operation in self.fail_on:
            raise RuntimeError(f"mock {operation} failure")

    @staticmethod
    def _done_reply():
        return SimpleNamespace(status=SimpleNamespace(value="done"))


def _joint_parameter(value) -> tuple[float, ...]:
    if isinstance(value, (int, float)):
        values = (float(value),) * JOINT_COUNT
    else:
        values = tuple(float(v) for v in value)
    if len(values) != JOINT_COUNT or not all(math.isfinite(v) and v > 0.0 for v in values):
        raise ValueError("mock MoveJ limits must contain six positive finite values")
    return values
