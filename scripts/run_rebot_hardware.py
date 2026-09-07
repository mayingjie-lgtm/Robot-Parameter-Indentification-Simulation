#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.mock_client import MockArmClient
from rebot_real.runner import RebotHardwareRunner, load_hardware_config


def parse_args() -> argparse.Namespace:
    """Parse the intentionally small hardware-runner CLI surface."""

    parser = argparse.ArgumentParser(
        description=(
            "Run the gated reBot hardware integration path. Default configuration denies "
            "real hardware and motion; use --mock for offline integration smoke."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT / "config" / "rebot_real_experiment.yaml",
    )
    parser.add_argument("--mock", action="store_true", help="use the in-process MockArmClient")
    parser.add_argument("--sdk-root", type=Path, help="override sdk_root from YAML")
    parser.add_argument("--host", help="override robot/lower host from YAML")
    parser.add_argument("--tcp-port", type=int, help="override TCP command port")
    parser.add_argument("--udp-port", type=int, help="override UDP state port")
    parser.add_argument("--output", type=Path, help="override hardware experiment CSV path")
    return parser.parse_args()


def main() -> int:
    """Load config, select real/Mock ArmClient with one if/else, and run once."""

    args = parse_args()
    overrides = {
        "sdk_root": str(args.sdk_root) if args.sdk_root is not None else None,
        "host": args.host,
        "tcp_port": args.tcp_port,
        "udp_port": args.udp_port,
        "output_csv": str(args.output) if args.output is not None else None,
    }
    config = load_hardware_config(args.config, repo_root=REPO_ROOT, overrides=overrides)
    client_factory = None
    if args.mock:
        client_factory = lambda **kwargs: MockArmClient(
            **kwargs, follow_servo_targets=config["control_mode"] == "joint_jog"
        )
    runner = RebotHardwareRunner(
        config,
        repo_root=REPO_ROOT,
        client_factory=client_factory,
        mock_backend=args.mock,
    )
    metadata = runner.run()
    output = Path(config["output_csv"])
    print(f"backend={metadata['backend']}")
    print(f"control_mode={metadata['control_mode']}")
    print(f"schema_version={metadata['schema_version']}")
    print(f"sample_count={metadata['observed_sample_count']}")
    print(f"csv={output}")
    print(f"metadata={output.with_suffix('.meta.yaml')}")
    if "joint_jog_result" in metadata:
        result = metadata["joint_jog_result"]
        print(f"joint_jog_status={result['status']}; physical_mapping_verified_by_test=false; identification_ready=false")
        print(f"measured_displacement_rad={result.get('measured_displacement_rad')}")
        print(f"return_error_rad={result.get('return_error_rad')}")
    if args.mock:
        print("mock_only=true; no real hardware was contacted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
