#!/usr/bin/env python3
"""Prepare independent frozen A/B candidates and disabled hardware templates, offline."""
import argparse
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rebot_real.trajectory_artifact import load_replay_artifact, qualify_replay_artifact, write_preview_outputs

SOURCES = {
    "A": (20260826, 6, "86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529"),
    "B": (20260829, 21, "67ccf33c832951fe72e4d81928727fcf7c2580f0600f5ef33e046761ce810d31"),
}


def prepare(output: Path, build: Path, render: bool = True):
    output, build = output.resolve(), build.resolve()
    if output.exists():
        raise FileExistsError(f"use a new candidate directory: {output}")
    output.mkdir(parents=True)
    qualify = yaml.safe_load((ROOT / "config/rebot_trajectory_preview.yaml").read_text())
    template = yaml.safe_load((ROOT / "config/rebot_real_experiment.yaml").read_text())
    manifest = dict(status="preparing", hardware_authorized=False, candidates={})
    for label, (seed, attempt, coefficient_hash) in SOURCES.items():
        run = output / label
        run.mkdir()
        subprocess.run([str(build / "rebot_trajectory_exporter"),
                        "--generate-coefficients", str(run / "coefficients.csv"),
                        "--output", str(run / "trajectory.csv"), "--metadata", str(run / "trajectory.meta.yaml"),
                        "--seed", str(seed), "--accepted-attempt", str(attempt),
                        "--expected-coefficient-sha256", coefficient_hash, "--sample-rate-hz", "100"],
                       cwd=ROOT, check=True)
        artifact = load_replay_artifact(run / "trajectory.csv", run / "trajectory.meta.yaml")
        report = qualify_replay_artifact(artifact, qualify)
        write_preview_outputs(report=report, report_path=run / "preview_report.yaml",
                              acceptance_path=run / "preview_acceptance.yaml", preview_mp4=run / "preview.mp4")
        if report["preview_status"] != "PASS":
            raise ValueError(f"{label} qualification failed: {report['failures']}")
        if render:
            subprocess.run([str(build / "rebot_trajectory_renderer"), "--input", str(artifact.path),
                            "--output", str(run / "preview.mp4")], cwd=ROOT, check=True)
        config = dict(template)
        config.update(control_mode="excitation", duration_s=30.0, max_samples=None,
                      output_csv=f"data/rebot_real/{label}_run01/raw.csv",
                      trajectory_source="frozen_replay_artifact", trajectory_hash=artifact.sha256,
                      trajectory_artifact=str(artifact.path), trajectory_metadata=str(artifact.metadata_path),
                      trajectory_preview_acceptance=str(run / "preview_acceptance.yaml"),
                      allow_hardware=False, allow_motion=False, joint_mapping_verified=False,
                      j1_convention="UNRESOLVED")
        # Keep real template limits unresolved; do not copy permissive Mock limits.
        (run / "hardware.pending.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
        manifest["candidates"][label] = dict(seed=seed, accepted_attempt=attempt,
                                             coefficient_sha256=coefficient_hash, trajectory_sha256=artifact.sha256,
                                             preview_rendered=render, accepted_for_hardware=False)
    manifest["status"] = "offline_candidates_prepared"
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(f"Prepared {output}; templates remain disabled and require hardware commissioning.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, default=ROOT / "results/rebot_real_ab")
    parser.add_argument("--build-directory", type=Path, default=ROOT / "build_rebot")
    parser.add_argument("--skip-video", action="store_true", help="headless numerical preparation only")
    args = parser.parse_args()
    prepare(args.output_directory, args.build_directory, not args.skip_video)
