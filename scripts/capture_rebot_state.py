#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.recorder import capture_udp_state
from rebot_real.sdk_adapter import UdpStateSubscriber


DEFAULT_SDK_ROOT = Path(
    "/home/wlsea1/桌面/机械臂sdk/wlsea_arm_sdk_sim_handoff_20260826"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture reBot-DM public UDP state only. This program never opens the "
            "SDK TCP command channel and never sends motor commands."
        )
    )
    parser.add_argument("--output", type=Path, required=True, help="capture CSV path")
    parser.add_argument("--duration", type=float, required=True, help="capture duration in seconds")
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=5001)
    parser.add_argument("--socket-timeout", type=float, default=0.2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.suffix.lower() != ".csv":
        raise SystemExit("--output must use a .csv suffix")
    metadata_path = args.output.with_suffix(".meta.yaml")
    with UdpStateSubscriber(
        args.sdk_root,
        bind_ip=args.bind_ip,
        udp_port=args.udp_port,
        timeout_s=args.socket_timeout,
    ) as subscriber:
        metadata = capture_udp_state(
            subscriber,
            duration_s=args.duration,
            csv_path=args.output,
            metadata_path=metadata_path,
            repo_root=REPO_ROOT,
            sdk_root=args.sdk_root,
        )

    print(f"capture_csv={args.output}")
    print(f"metadata={metadata_path}")
    print(f"observed_sample_count={metadata['observed_sample_count']}")
    if metadata["observed_sample_count"] == 0:
        print(
            "No UDP state was received. Current SDK lower publishes public UDP only "
            "while a TCP session exists; do not create a TCP control session merely "
            "to make this state-only recorder receive data.",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
