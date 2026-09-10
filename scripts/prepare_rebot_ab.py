#!/usr/bin/env python3
"""Prepare independent frozen A/B candidates and disabled hardware templates, offline."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rebot_real.trajectory_artifact import load_replay_artifact, qualify_replay_artifact, write_preview_outputs

def _renderer_env() -> dict[str, str]:
    env = dict(os.environ)
    if shutil.which("ffmpeg", path=env.get("PATH")):
        return env
    try:
        import imageio_ffmpeg
    except ImportError:
        return env
    env["REBOT_FFMPEG_BIN"] = imageio_ffmpeg.get_ffmpeg_exe()
    return env


SOURCES = {
    "A": (20260826, 6, "86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529"),
    "B": (20260829, 21, "67ccf33c832951fe72e4d81928727fcf7c2580f0600f5ef33e046761ce810d31"),
}


def prepare(
    output: Path,
    build: Path,
    render: bool = True,
    sample_rate_hz: float = 100.0,
):
    output, build = output.resolve(), build.resolve()
    if output.exists():
        raise FileExistsError(f"use a new candidate directory: {output}")
    output.mkdir(parents=True)
    qualify = yaml.safe_load((ROOT / "config/rebot_trajectory_preview.yaml").read_text())
    qualify["expected_sample_rate_hz"] = float(sample_rate_hz)
    qualify["expected_sample_count"] = int(round(30.0 * sample_rate_hz)) + 1
    manifest = dict(status="preparing", hardware_authorized=False, candidates={})
    for label, (seed, attempt, coefficient_hash) in SOURCES.items():
        run = output / label
        run.mkdir()
        frozen_coefficients = ROOT / "results" / "rebot_real_ab" / label / "coefficients.csv"
        if not frozen_coefficients.is_file():
            raise FileNotFoundError(f"frozen coefficient artifact is missing: {frozen_coefficients}")
        shutil.copyfile(frozen_coefficients, run / "coefficients.csv")
        subprocess.run([str(build / "rebot_trajectory_exporter"),
                        "--coefficients", str(run / "coefficients.csv"),
                        "--output", str(run / "trajectory.csv"), "--metadata", str(run / "trajectory.meta.yaml"),
                        "--seed", str(seed), "--accepted-attempt", str(attempt),
                        "--expected-coefficient-sha256", coefficient_hash,
                        "--sample-rate-hz", str(float(sample_rate_hz))],
                       cwd=ROOT, check=True)
        artifact = load_replay_artifact(run / "trajectory.csv", run / "trajectory.meta.yaml")
        report = qualify_replay_artifact(artifact, qualify)
        if render:
            subprocess.run([str(build / "rebot_trajectory_renderer"), "--input", str(artifact.path),
                            "--output", str(run / "preview.mp4")], cwd=ROOT,
                           env=_renderer_env(), check=True)
            write_preview_outputs(report=report, report_path=run / "preview_report.yaml",
                                  acceptance_path=run / "preview_acceptance.yaml", preview_mp4=run / "preview.mp4")
        else:
            (run / "preview_report.yaml").write_text(
                yaml.safe_dump(report, sort_keys=False, allow_unicode=True)
            )
        if report["preview_status"] != "PASS":
            manifest["candidates"][label] = dict(
                seed=seed,
                accepted_attempt=attempt,
                coefficient_sha256=coefficient_hash,
                trajectory_sha256=artifact.sha256,
                preview_rendered=render,
                qualification_status="FAIL",
                qualification_failures=list(report["failures"]),
                accepted_for_hardware=False,
            )
            manifest["status"] = "blocked_by_offline_qualification"
            (output / "manifest.yaml").write_text(
                yaml.safe_dump(manifest, sort_keys=False)
            )
            raise ValueError(f"{label} qualification failed: {report['failures']}")
        smoke_template_path = (
            ROOT / "results" / "rebot_real_ab" / label / "hardware.smoke.yaml"
        )
        if not smoke_template_path.is_file():
            raise FileNotFoundError(
                f"validated historical smoke config is missing: {smoke_template_path}"
            )
        config = yaml.safe_load(smoke_template_path.read_text())
        config.update(control_mode="excitation", control_rate_hz=float(sample_rate_hz),
                      duration_s=30.0, max_samples=None,
                      output_csv=f"data/rebot_real/{label}_run01/raw.csv",
                      trajectory_source="frozen_replay_artifact", trajectory_hash=artifact.sha256,
                      trajectory_replay_mode="actual_time_quintic_v1",
                      trajectory_artifact=str(artifact.path), trajectory_metadata=str(artifact.metadata_path),
                      trajectory_preview_acceptance=str(run / "preview_acceptance.yaml"),
                      allow_hardware=False, allow_motion=False, joint_mapping_verified=False,
                      joint_mapping_scope="excitation", j1_convention="UNRESOLVED",
                      controlled_park_before_disable=True,
                      motion_ready_feedback_max_age_ms=100.0,
                      maximum_disabled_feedback_age_ms=50.0,
                      lower_feedback_timeout_ms=250.0,
                      transient_feedback_invalid_recovery_ms=100.0,
                      host_state_snapshot_timeout_s=0.25)
        # New pending excitation configs use physical names. Keep the loader's
        # legacy maximum_feedback_age_ms alias only for historical files.
        config.pop("maximum_feedback_age_ms", None)
        config["maximum_servo_target_delta_rad"] = float(
            qualify["maximum_servo_target_delta_rad"]
        )
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
    parser.add_argument("--sample-rate-hz", type=float, default=100.0)
    args = parser.parse_args()
    prepare(
        args.output_directory,
        args.build_directory,
        not args.skip_video,
        args.sample_rate_hz,
    )
