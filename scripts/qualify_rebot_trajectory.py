#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.trajectory_artifact import (
    load_replay_artifact,
    qualify_replay_artifact,
    write_preview_outputs,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Numerically qualify one frozen reBot replay artifact and create a "
            "human-review acceptance template. This script never evaluates Fourier math."
        )
    )
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--acceptance", type=Path, required=True)
    parser.add_argument("--preview-mp4", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("qualification config must be a YAML mapping")
    artifact = load_replay_artifact(args.artifact, args.metadata)
    report = qualify_replay_artifact(artifact, config)
    write_preview_outputs(
        report=report,
        report_path=args.report,
        acceptance_path=args.acceptance,
        preview_mp4=args.preview_mp4,
    )
    print(f"trajectory_sha256={artifact.sha256}")
    print(f"preview_status={report['preview_status']}")
    print(f"report={args.report}")
    print(f"acceptance={args.acceptance}")
    print("accepted_for_hardware=false")
    if report["failures"]:
        for failure in report["failures"]:
            print(f"FAIL: {failure}")
    return 0 if report["preview_status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
