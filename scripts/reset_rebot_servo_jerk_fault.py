#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


EXPECTED_FAULT = 500115
EXPECTED_NAME = "SERVO_COMMAND_JERK_EXCEEDED"
EXPECTED_SOURCE = "servo_target_validator"
MAX_RESET_FEEDBACK_AGE_MS = 250.0
DEFAULT_SDK_ROOT = Path("/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904")


def _load_robot(sdk_root: Path):
    root = sdk_root.expanduser().resolve()
    if not (root / "src" / "wlsea_arm" / "robot.py").is_file():
        raise FileNotFoundError(f"wlsea_arm/robot.py not found under {root / 'src'}")
    sys.path.insert(0, str(root / "src"))
    from wlsea_arm import Robot  # type: ignore

    return Robot


def _enum_text(value) -> str:
    return str(getattr(value, "value", value)).lower()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Explicitly clear only the latched reBot 500115 "
            "SERVO_COMMAND_JERK_EXCEEDED fault. This script never enables motors "
            "and never sends MoveJ/Servo commands."
        )
    )
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--host", default="192.168.50.24")
    parser.add_argument("--tcp-port", type=int, default=5000)
    parser.add_argument("--udp-port", type=int, default=5001)
    parser.add_argument(
        "--confirm",
        required=True,
        help="must be exactly RESET-500115",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirm != "RESET-500115":
        raise SystemExit("--confirm must be exactly RESET-500115")

    Robot = _load_robot(args.sdk_root)
    print(
        "FAULT_RESET_ONLY: connect/read/get_active_faults/reset_fault/close; "
        "no enable/disable/MoveJ/Servo commands"
    )

    with Robot(
        args.host,
        tcp_port=args.tcp_port,
        udp_port=args.udp_port,
    ) as arm:
        state = arm.get_state()
        if state is None:
            raise RuntimeError("no UDP state received")

        mode = _enum_text(state.robot_mode)
        safety = _enum_text(state.safety_state)
        faults = tuple(int(code) for code in (state.active_fault_codes or ()))
        valid = tuple(bool(flag) for flag in state.feedback_valid)
        ages = tuple(float(age) for age in state.feedback_age_ms)

        print(
            f"pre_reset mode={mode} safety={safety} "
            f"primary_fault={int(state.primary_fault_code)} active_faults={faults}"
        )
        print(f"pre_reset feedback_valid={valid}")
        print(f"pre_reset feedback_age_ms={ages}")

        if faults == ():
            print("RESULT: no active latched fault; nothing to reset")
            return 0
        if faults != (EXPECTED_FAULT,):
            raise RuntimeError(
                f"refusing reset: expected only fault {EXPECTED_FAULT}, got {faults}"
            )
        if mode not in {"fault", "disabled"}:
            raise RuntimeError(
                f"refusing reset: robot_mode must be fault/disabled, got {mode}"
            )
        if safety in {"ready", "moving", "enabling", "protective_stop"}:
            raise RuntimeError(
                f"refusing reset: safety_state={safety} is not a motors-off reset state"
            )
        if len(valid) != 6 or not all(valid):
            raise RuntimeError(
                f"refusing reset: six-axis feedback is not fully valid: {valid}"
            )
        if len(ages) != 6 or max(ages) > MAX_RESET_FEEDBACK_AGE_MS:
            raise RuntimeError(
                "refusing reset: feedback is not fresh enough for lower reset policy; "
                f"max_age_ms={max(ages) if ages else float('inf'):.3f}"
            )

        active = arm.get_active_faults()
        print(f"active_fault_detail={active}")
        matching = [
            fault
            for fault in active
            if int(fault.get("code", 0)) == EXPECTED_FAULT
        ]
        if len(matching) != 1:
            raise RuntimeError(
                "refusing reset: runtime fault detail does not contain exactly one "
                f"{EXPECTED_FAULT} entry"
            )
        detail = matching[0]
        if str(detail.get("name", "")) != EXPECTED_NAME:
            raise RuntimeError(
                "refusing reset: runtime mapping mismatch for 500115; "
                f"expected {EXPECTED_NAME}, got {detail.get('name')!r}"
            )
        if str(detail.get("source", "")) != EXPECTED_SOURCE:
            raise RuntimeError(
                "refusing reset: runtime source mismatch for 500115; "
                f"expected {EXPECTED_SOURCE}, got {detail.get('source')!r}"
            )
        if not bool(detail.get("latched", False)):
            raise RuntimeError("refusing reset: 500115 is not reported as latched")

        arm.reset_fault(timeout_s=5.0)

        state = arm.get_state()
        if state is None:
            raise RuntimeError("reset returned but no post-reset UDP state is available")
        post_faults = tuple(int(code) for code in (state.active_fault_codes or ()))
        post_mode = _enum_text(state.robot_mode)
        post_safety = _enum_text(state.safety_state)
        print(
            f"post_reset mode={post_mode} safety={post_safety} "
            f"primary_fault={int(state.primary_fault_code)} active_faults={post_faults}"
        )
        print(
            "post_reset feedback_valid="
            + str(tuple(bool(flag) for flag in state.feedback_valid))
        )
        print(
            "post_reset feedback_age_ms="
            + str(tuple(float(age) for age in state.feedback_age_ms))
        )
        if post_faults:
            raise RuntimeError(f"fault still active after reset: {post_faults}")
        if int(state.primary_fault_code) != 0:
            raise RuntimeError(
                f"primary fault still active after reset: {state.primary_fault_code}"
            )
        if post_mode != "disabled":
            raise RuntimeError(
                "reset cleared the fault but did not confirm robot_mode=disabled"
            )

    print("RESULT: 500115 cleared; robot remains disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
