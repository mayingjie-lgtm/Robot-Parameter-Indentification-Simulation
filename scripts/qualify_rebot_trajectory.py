#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.trajectory_artifact import (
    load_replay_artifact,
    qualify_replay_artifact,
    sha256_file,
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
    parser.add_argument(
        "--preposition-start-q",
        help="six comma-separated joints used only to document the rendered SDK MoveJ semantic preposition",
    )
    parser.add_argument("--preposition-source", default="")
    parser.add_argument("--preposition-duration", type=float, default=10.0)
    parser.add_argument("--preposition-settle", type=float, default=1.0)
    parser.add_argument("--end-hold", type=float, default=1.0)
    return parser.parse_args()


def parse_joint_vector(value: str) -> list[float]:
    values = [float(item) for item in value.split(",")]
    if len(values) != 6:
        raise ValueError("--preposition-start-q must contain exactly six values")
    return values


def main() -> int:
    args = parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("qualification config must be a YAML mapping")
    artifact = load_replay_artifact(args.artifact, args.metadata)
    report = qualify_replay_artifact(artifact, config)
    report["git_head_at_preview"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
    ).strip()
    report["trajectory_metadata_sha256"] = sha256_file(args.metadata)
    report["qualification_config_file"] = str(args.config.resolve())
    report["qualification_config_sha256"] = sha256_file(args.config)
    if args.preposition_start_q:
        if args.preposition_duration <= 0 or args.preposition_settle < 0 or args.end_hold < 0:
            raise ValueError("preview preposition/settle/end-hold durations are invalid")
        report["preview_visualization"] = {
            "current_start_q": parse_joint_vector(args.preposition_start_q),
            "current_start_source": args.preposition_source,
            "preposition_method": "sdk_movej_semantic_minimum_jerk_visual_only",
            "preposition_duration_s": args.preposition_duration,
            "settle_at_q0_s": args.preposition_settle,
            "end_hold_s": args.end_hold,
            "preposition_not_part_of_frozen_excitation": True,
            "excitation_source": "exact frozen q_ref/qd_ref/qdd_ref artifact",
            "excitation_replay_mode": "actual_time_quintic_v1",
        }
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
