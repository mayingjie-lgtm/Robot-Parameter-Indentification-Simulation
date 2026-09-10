#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SDK_ROOT = Path("/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904")


def _load_arm_client(sdk_root: Path):
    root = sdk_root.expanduser().resolve()
    candidates = (root / "src", root / "upper" / "python")
    python_root = next(
        (candidate for candidate in candidates if (candidate / "wlsea_arm_sdk" / "client.py").is_file()),
        None,
    )
    if python_root is None:
        checked = ", ".join(str(path / "wlsea_arm_sdk" / "client.py") for path in candidates)
        raise FileNotFoundError(f"wlsea_arm_sdk/client.py not found; checked: {checked}")
    sys.path.insert(0, str(python_root))
    from wlsea_arm_sdk import ArmClient  # type: ignore
    return ArmClient


def _enum_text(value) -> str:
    return str(getattr(value, "value", value))


def _tuple_text(values, *, scale: float = 1.0, digits: int = 3) -> str:
    return "[" + ", ".join(f"{float(value) * scale:.{digits}f}" for value in values) + "]"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read raw reBot SDK feedback diagnostics without sending any motor command. "
            "The client is created with disable_on_close=False."
        )
    )
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--host", default="192.168.50.24")
    parser.add_argument("--tcp-port", type=int, default=5000)
    parser.add_argument("--udp-port", type=int, default=5001)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--period", type=float, default=0.10)
    parser.add_argument(
        "--no-fault-query",
        action="store_true",
        help="skip read-only get_active_faults/fault_explain TCP queries",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0 or args.period <= 0.0:
        raise SystemExit("--duration and --period must be positive")

    ArmClient = _load_arm_client(args.sdk_root)
    arm = ArmClient(
        args.host,
        args.tcp_port,
        udp_port=args.udp_port,
        disable_on_close=False,
    )
    print(
        "READ_ONLY_DIAGNOSTIC: connect/read/fault-query/close only; "
        "no enable/disable/reset_fault/MoveJ/Servo commands"
    )
    arm.connect(timeout_s=3.0)
    deadline = time.monotonic() + args.duration
    last_sequence = None
    seen = 0
    observed_fault_codes: set[int] = set()
    try:
        while time.monotonic() < deadline:
            state = arm.latest_state
            if state is None:
                time.sleep(min(args.period, 0.02))
                continue
            sequence = int(state.sequence)
            if sequence == last_sequence:
                time.sleep(args.period)
                continue
            last_sequence = sequence
            seen += 1
            valid = tuple(bool(value) for value in state.feedback_valid)
            invalid = [f"J{i + 1}" for i, value in enumerate(valid) if not value]
            print(
                f"seq={sequence} "
                f"mode={_enum_text(state.robot_mode)} "
                f"safety={_enum_text(state.safety_state)} "
                f"error={_enum_text(state.error_code)} "
                f"primary_fault={int(state.primary_fault_code)} "
                f"active_faults={tuple(state.active_fault_codes)}"
            )
            print(f"  feedback_valid={valid}; invalid={invalid or 'none'}")
            print(f"  feedback_age_ms={_tuple_text(state.feedback_age_ms)}")
            print(
                "  packet_rx_1s="
                + str(tuple(int(value) for value in state.packet_received_frames_1s))
                + "; packet_expected_1s="
                + str(tuple(int(value) for value in state.packet_expected_frames_1s))
            )
            print(
                f"  bus_loss_valid_1s={bool(state.estimated_bus_loss_valid_1s)}; "
                f"bus_loss_1s={float(state.estimated_bus_loss_rate_1s):.6f}; "
                f"diagnostic={state.diagnostic_message!r}"
            )
            observed_fault_codes.update(
                int(code) for code in (state.active_fault_codes or ()) if int(code) != 0
            )
            primary_fault = int(state.primary_fault_code)
            if primary_fault != 0:
                observed_fault_codes.add(primary_fault)
            time.sleep(args.period)
    finally:
        if observed_fault_codes and not args.no_fault_query:
            print("FAULT_QUERY: read-only Lower fault detail")
            try:
                active = arm.get_active_faults(timeout_s=3.0)
            except Exception as exc:
                print(f"  get_active_faults failed: {type(exc).__name__}: {exc}")
            else:
                print(f"  active_fault_detail={active}")
            for code in sorted(observed_fault_codes):
                try:
                    detail = arm.fault_explain(code, timeout_s=3.0)
                except Exception as exc:
                    print(f"  fault_explain({code}) failed: {type(exc).__name__}: {exc}")
                else:
                    print(f"  fault_explain({code})={detail}")
        arm.close()

    if seen == 0:
        print("RESULT: no SDK state received")
        return 2

    print(f"RESULT: observed {seen} distinct SDK state frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
