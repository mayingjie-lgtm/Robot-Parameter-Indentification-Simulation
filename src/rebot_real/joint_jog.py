"""A bounded single-joint round trip; no homing or mapping inference."""

from __future__ import annotations

import math
from typing import Any


def validate_joint_jog(config: dict[str, Any]) -> dict[str, Any]:
    """Require explicit test limits, including for Mock, before any connection."""
    fields = {
        "joint", "displacement_rad", "ramp_duration_s", "settle_duration_s",
        "maximum_acceleration_rad_s2", "maximum_other_joint_drift_rad",
        "settle_position_tolerance_rad", "settle_velocity_tolerance_rad_s",
        "position_margin_rad", "maximum_command_gap_s", "authorization_reference",
    }
    parsed = config.get("joint_jog")
    if not isinstance(parsed, dict) or set(parsed) != fields:
        raise ValueError(f"joint_jog must specify exactly {sorted(fields)}")
    jog = dict(parsed)
    if type(jog["joint"]) is not int or not 1 <= jog["joint"] <= 6:
        raise ValueError("joint_jog.joint must be an integer in [1, 6]")
    reference = jog["authorization_reference"]
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("joint_jog.authorization_reference must identify the test approval record")
    for key in fields - {"joint", "authorization_reference"}:
        value = jog[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"joint_jog.{key} must be finite numeric")
        invalid_sign = value == 0 if key == "displacement_rad" else value <= 0
        if invalid_sign:
            raise ValueError(f"joint_jog.{key} must be nonzero/positive")
        jog[key] = float(value)
    period = 1.0 / config["control_rate_hz"]
    if min(jog["ramp_duration_s"], jog["settle_duration_s"]) < 4 * period:
        raise ValueError("joint_jog ramp and settle must each span at least four periods")
    if jog["maximum_command_gap_s"] <= period:
        raise ValueError("joint_jog maximum_command_gap_s must exceed the control period")
    if jog["settle_position_tolerance_rad"] >= abs(jog["displacement_rad"]) / 2:
        raise ValueError("joint_jog settle tolerance must be less than half the displacement")
    # h(u)=10u^3-15u^4+6u^5; max h'=15/8; max |h''|=10/sqrt(3).
    peak_velocity = 1.875 * abs(jog["displacement_rad"]) / jog["ramp_duration_s"]
    peak_acceleration = (10 / math.sqrt(3)) * abs(jog["displacement_rad"]) / jog["ramp_duration_s"] ** 2
    if peak_velocity > config["maximum_command_velocity_rad_s"][jog["joint"] - 1]:
        raise ValueError("joint_jog ramp exceeds configured command velocity")
    if peak_acceleration > jog["maximum_acceleration_rad_s2"]:
        raise ValueError("joint_jog ramp exceeds configured command acceleration")
    jog["nominal_duration_s"] = 2 * jog["ramp_duration_s"] + 3 * jog["settle_duration_s"]
    if config["max_samples"] is not None:
        raise ValueError("joint_jog requires max_samples: null to avoid truncating the return")
    if config["duration_s"] <= jog["nominal_duration_s"]:
        raise ValueError("joint_jog duration_s must exceed the nominal round-trip duration (timeout budget)")
    return jog


def jog_target_at(jog: dict[str, Any], origin: tuple[float, ...], elapsed_s: float) -> tuple[str, tuple[float, ...], bool]:
    """Evaluate the smooth path at actual elapsed time, without assuming exact 100 Hz.

    The final flag selects the latter half of each stationary sampling window.
    A new run is required for the opposite direction or a different joint.
    """
    joint = jog["joint"] - 1
    elapsed = max(0.0, elapsed_s)
    for phase in ("baseline", "outbound", "endpoint", "return", "returned"):
        moving = phase in {"outbound", "return"}
        duration = jog["ramp_duration_s"] if moving else jog["settle_duration_s"]
        if elapsed < duration or phase == "returned":
            if moving:
                u = elapsed / duration
                h = u ** 3 * (10 + u * (-15 + 6 * u))
                fraction = h if phase == "outbound" else 1 - h
            else:
                fraction = 1.0 if phase == "endpoint" else 0.0
            target = list(origin)
            target[joint] += fraction * jog["displacement_rad"]
            return phase, tuple(target), not moving and elapsed >= duration / 2
        elapsed -= duration
    raise AssertionError("unreachable jog phase")
