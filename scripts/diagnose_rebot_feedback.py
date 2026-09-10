#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import yaml


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


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _offline_report(metadata_path: Path, csv_path: Path | None) -> int:
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    cadence = metadata.get("feedback_cadence") or {}
    timing = metadata.get("dispatch_timing") or {}
    if csv_path is None:
        name = metadata_path.name
        csv_path = (
            metadata_path.with_name(name[: -len(".meta.yaml")] + ".csv")
            if name.endswith(".meta.yaml")
            else None
        )
    ages: list[float] = []
    if csv_path is not None and csv_path.is_file():
        with csv_path.open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                for joint in range(6):
                    try:
                        value = float(row[f"feedback_age_ms{joint}"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if math.isfinite(value):
                        ages.append(value)

    def rate(section: str) -> float | None:
        value = cadence.get(section, {}).get("effective_rate_hz")
        return None if value is None else float(value)

    print("OFFLINE_ONLY: no SDK import, connection, or hardware command")
    print(f"metadata={metadata_path}")
    if csv_path is not None:
        print(f"csv={csv_path}")
    print(f"command_dispatch_rate_hz={timing.get('effective_command_rate_hz')}")
    print(f"udp_host_publication_rate_hz={rate('timestamp_host_rx_cadence')}")
    print(f"lower_timestamp_update_rate_hz={rate('timestamp_lower_update_cadence_on_host')}")
    print(f"q_update_rate_hz={rate('q_update_cadence_on_host')}")
    print(f"qd_update_rate_hz={rate('qd_update_cadence_on_host')}")
    print(f"effort_update_rate_hz={rate('effort_update_cadence_on_host')}")
    print(f"duplicate_snapshot_count={cadence.get('duplicate_state_snapshot_count')}")
    print(f"feedback_age_p50_ms={_percentile(ages, 50.0)}")
    print(f"feedback_age_p95_ms={_percentile(ages, 95.0)}")
    print(f"feedback_age_max_ms={max(ages) if ages else None}")
    print("rate_semantics=CSV/command rate is not the independent physical measurement rate")

    failure = metadata.get("failure") or {}
    failure_ages = [
        float(value)
        for value in (failure.get("feedback_age_ms") or [])
        if value is not None and math.isfinite(float(value))
    ]
    sdk_safety = metadata.get("sdk_safety_config_snapshot") or {}
    lower_timeout_ms = float(
        metadata.get(
            "lower_feedback_timeout_ms",
            sdk_safety.get("feedback_timeout_ms", 250.0),
        )
    )
    host_timeout_s = float(
        metadata.get(
            "host_state_snapshot_timeout_s",
            metadata.get("state_timeout_s", 0.25),
        )
    )
    if (
        failure.get("stage") == "excitation_state_snapshot"
        and failure_ages
        and max(failure_ages) <= lower_timeout_ms
        and float(failure.get("state_snapshot_age_ms", math.inf))
        <= host_timeout_s * 1000.0
        and int(failure.get("primary_fault_code", -1)) == 0
        and (
            failure.get("servo_active") is True
            or failure.get("servo_mode") == "servo"
        )
    ):
        print(
            "historical_failure_classification=legacy_100ms_host_gate_false_positive; "
            "this snapshot alone should not be an immediate abort under the repaired semantics"
        )
        print(
            "classification_scope=does_not_predict_that_the_remaining excitation would "
            "complete without a later real fault"
        )
    return 0


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
        "--offline-metadata",
        type=Path,
        help="analyze an existing raw.meta.yaml without importing or connecting the SDK",
    )
    parser.add_argument(
        "--offline-csv",
        type=Path,
        help="optional raw.csv paired with --offline-metadata for feedback-age percentiles",
    )
    parser.add_argument(
        "--no-fault-query",
        action="store_true",
        help="skip read-only get_active_faults/fault_explain TCP queries",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.offline_metadata is not None:
        return _offline_report(args.offline_metadata, args.offline_csv)
    if args.offline_csv is not None:
        raise SystemExit("--offline-csv requires --offline-metadata")
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
