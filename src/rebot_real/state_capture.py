from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence


JOINT_COUNT = 6


@dataclass(frozen=True)
class CaptureSample:
    """One normalized public-state UDP sample for the hardware capture CSV."""

    sample_index: int
    timestamp_host_rx_ns: int
    timestamp_lower_ns: int
    udp_sequence: int
    udp_sequence_gap: int
    q: tuple[float, ...]
    qd: tuple[float, ...]
    effort_reported: tuple[float, ...]
    feedback_valid: tuple[bool, ...]
    torque_valid: tuple[bool, ...]
    feedback_age_ms: tuple[float, ...]
    robot_mode: str
    safety_state: str
    primary_fault_code: int
    servo_active: bool
    servo_mode: str
    servo_last_accepted_sequence: int = 0
    servo_target_age_ns: int = 0
    servo_accepted_targets: int = 0
    servo_rejected_targets: int = 0
    servo_target_jump_rejects: int = 0
    servo_velocity_rejects: int = 0
    servo_acceleration_rejects: int = 0
    servo_jerk_rejects: int = 0


def _six(values: Sequence[Any], name: str) -> tuple[Any, ...]:
    parsed = tuple(values)
    if len(parsed) != JOINT_COUNT:
        raise ValueError(f"{name} must contain exactly {JOINT_COUNT} values")
    return parsed


def _enum_text(value: Any) -> str:
    enum_value = getattr(value, "value", value)
    return str(enum_value)


def map_sdk_state(
    state: Any,
    *,
    sample_index: int,
    timestamp_host_rx_ns: int,
    previous_sequence: int | None,
) -> CaptureSample:
    """Map one SDK public JointState without inventing unavailable signals.

    Invalid public measurements are written as NaN rather than stale/default zero.
    ``udp_sequence_gap`` is the number of lower-state sequence values skipped between
    two received UDP datagrams. The SDK increments this source sequence in the lower
    control loop, so this field is diagnostic only and is not an UDP packet-loss count.
    """

    if sample_index < 0:
        raise ValueError("sample_index must be non-negative")
    if timestamp_host_rx_ns <= 0:
        raise ValueError("timestamp_host_rx_ns must be positive")

    position = _six(state.position_rad, "position_rad")
    velocity = _six(state.velocity_rad_s, "velocity_rad_s")
    torque = _six(state.torque_nm, "torque_nm")
    feedback_valid = tuple(bool(value) for value in _six(state.feedback_valid, "feedback_valid"))
    torque_valid = tuple(bool(value) for value in _six(state.torque_valid, "torque_valid"))
    feedback_age = _six(state.feedback_age_ms, "feedback_age_ms")

    sequence = int(state.sequence)
    if sequence < 0:
        raise ValueError("SDK sequence must be non-negative")
    sequence_gap = 0
    if previous_sequence is not None and sequence > previous_sequence:
        sequence_gap = max(sequence - previous_sequence - 1, 0)

    q: list[float] = []
    qd: list[float] = []
    effort_reported: list[float] = []
    age_ms: list[float] = []
    for joint in range(JOINT_COUNT):
        if feedback_valid[joint]:
            q.append(float(position[joint]))
            qd.append(float(velocity[joint]))
            age_ms.append(float(feedback_age[joint]))
        else:
            q.append(math.nan)
            qd.append(math.nan)
            age_ms.append(math.nan)
        if feedback_valid[joint] and torque_valid[joint]:
            effort_reported.append(float(torque[joint]))
        else:
            effort_reported.append(math.nan)

    return CaptureSample(
        sample_index=sample_index,
        timestamp_host_rx_ns=int(timestamp_host_rx_ns),
        timestamp_lower_ns=int(state.monotonic_time_ns),
        udp_sequence=sequence,
        udp_sequence_gap=sequence_gap,
        q=tuple(q),
        qd=tuple(qd),
        effort_reported=tuple(effort_reported),
        feedback_valid=feedback_valid,
        torque_valid=torque_valid,
        feedback_age_ms=tuple(age_ms),
        robot_mode=_enum_text(state.robot_mode),
        safety_state=_enum_text(state.safety_state),
        primary_fault_code=int(state.primary_fault_code),
        servo_active=bool(state.servo_active),
        servo_mode=str(state.servo_mode),
        servo_last_accepted_sequence=int(
            getattr(state, "servo_last_accepted_sequence", 0)
        ),
        servo_target_age_ns=int(getattr(state, "servo_target_age_ns", 0)),
        servo_accepted_targets=int(getattr(state, "servo_accepted_targets", 0)),
        servo_rejected_targets=int(getattr(state, "servo_rejected_targets", 0)),
        servo_target_jump_rejects=int(
            getattr(state, "servo_target_jump_rejects", 0)
        ),
        servo_velocity_rejects=int(getattr(state, "servo_velocity_rejects", 0)),
        servo_acceleration_rejects=int(
            getattr(state, "servo_acceleration_rejects", 0)
        ),
        servo_jerk_rejects=int(getattr(state, "servo_jerk_rejects", 0)),
    )
