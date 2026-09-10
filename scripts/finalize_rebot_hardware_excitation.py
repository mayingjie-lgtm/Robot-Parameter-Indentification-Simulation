#!/usr/bin/env python3
"""Finalize one offline reBot hardware-compatible excitation artifact.

This helper never authorizes hardware. It qualifies the already-frozen C++
trajectory, renders the exact-command zero-order-hold preview, writes an
acceptance record that remains false, and prepares a Mock-only replay config.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebot_real.trajectory_artifact import (
    load_replay_artifact,
    qualify_replay_artifact,
    sha256_file,
    write_preview_outputs,
)


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


def _repo_relative(path: Path) -> str:
    return str(path.resolve().relative_to(ROOT.resolve()))


def _mock_replay_summary(run_dir: Path, artifact) -> dict[str, object]:
    """Validate an existing full Mock replay before promoting its manifest status."""
    metadata_path = run_dir / "mock_replay.meta.yaml"
    if not metadata_path.exists():
        return {"mock_replay_status": "PENDING"}

    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("mock replay metadata must be a mapping")
    expected_rate = float(artifact.sample_rate_hz)
    expected_count = len(artifact.samples)
    required = {
        "backend": "rebot_sdk_mock",
        "control_mode": "excitation",
        "motion_status": "completed",
        "trajectory_hash": artifact.sha256,
    }
    for key, expected in required.items():
        if metadata.get(key) != expected:
            raise ValueError(
                f"mock replay metadata {key} mismatch: "
                f"{metadata.get(key)!r} != {expected!r}"
            )
    if float(metadata.get("control_rate_hz", 0.0)) != expected_rate:
        raise ValueError("mock replay control rate does not match frozen artifact")
    if int(metadata.get("observed_sample_count", -1)) <= 0:
        raise ValueError("mock replay contains no accepted excitation commands")

    timing = metadata.get("dispatch_timing") or {}
    nominal_period_ns = int(round(1_000_000_000.0 / expected_rate))
    if int(timing.get("nominal_period_ns", -1)) != nominal_period_ns:
        raise ValueError("mock replay nominal period does not match frozen artifact rate")
    if float(timing.get("nominal_rate_hz", 0.0)) != expected_rate:
        raise ValueError("mock replay nominal rate does not match frozen artifact rate")
    if int(timing.get("actual_dispatch_interval_min_ns", -1)) < nominal_period_ns:
        raise ValueError("mock replay contains a below-nominal catch-up dispatch interval")
    if int(timing.get("catch_up_burst_count", -1)) != 0:
        raise ValueError("mock replay contains a catch-up burst")
    if int(timing.get("command_dispatch_timestamp_mismatch_count", -1)) != 0:
        raise ValueError("mock replay command dispatch timestamps do not match commands")
    replay_semantics = str(timing.get("replay_semantics", ""))
    for phrase in ("continuous quintic path", "real dispatch time", "no catch-up burst"):
        if phrase not in replay_semantics:
            raise ValueError(f"mock replay semantics missing required phrase: {phrase}")
    envelope = metadata.get("runtime_servo_envelope") or {}
    if envelope.get("strategy") != "actual_time_quintic_v1":
        raise ValueError("mock replay runtime envelope strategy mismatch")

    upper_gate = metadata.get("upper_safety_gate") or {}
    if upper_gate.get("tracking_error_semantics") != "monitor_only_quality_warning":
        raise ValueError("excitation tracking error is not recorded as monitor-only")
    if upper_gate.get("tracking_error_immediate_fail_safe") is not False:
        raise ValueError("excitation tracking lag unexpectedly remains an immediate fail-safe")

    return {
        "mock_replay_status": "PASS",
        "mock_motion_status": metadata["motion_status"],
        "mock_observed_sample_count": metadata["observed_sample_count"],
        "mock_trajectory_time_end_s": envelope.get("trajectory_time_end_s"),
        "mock_catch_up_burst_count": timing["catch_up_burst_count"],
        "mock_command_dispatch_timestamp_mismatch_count": timing[
            "command_dispatch_timestamp_mismatch_count"
        ],
    }


def refresh_manifest_mock_status(run_dir: Path) -> None:
    """Refresh only Mock evidence fields; never authorize preview or hardware."""
    run_dir = run_dir.resolve()
    trajectory = run_dir / "trajectory.csv"
    metadata = run_dir / "trajectory.meta.yaml"
    manifest_path = run_dir / "manifest.yaml"
    acceptance_path = run_dir / "preview_acceptance.yaml"
    for path in (trajectory, metadata, manifest_path, acceptance_path):
        if not path.exists():
            raise FileNotFoundError(path)

    artifact = load_replay_artifact(trajectory, metadata)
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    acceptance = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or not isinstance(acceptance, dict):
        raise ValueError("manifest and preview acceptance must be mappings")
    if manifest.get("schema_version") != "rebot_hardware_excitation_manifest_v1":
        raise ValueError("unexpected excitation manifest schema")
    if manifest.get("trajectory_sha256") != artifact.sha256:
        raise ValueError("manifest trajectory hash does not match frozen artifact")
    if acceptance.get("trajectory_sha256") != artifact.sha256:
        raise ValueError("preview acceptance hash does not match frozen artifact")
    for key in ("hardware_authorized", "motion_authorized", "accepted_for_hardware"):
        if manifest.get(key) is not False:
            raise ValueError(f"offline manifest must keep {key}=false")
    if acceptance.get("accepted_for_hardware") is not False:
        raise ValueError("offline refresh refuses an accepted hardware preview")

    manifest.update(_mock_replay_summary(run_dir, artifact))
    manifest["hardware_authorized"] = False
    manifest["motion_authorized"] = False
    manifest["accepted_for_hardware"] = False
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    print(f"trajectory_sha256={artifact.sha256}")
    print(f"mock_replay_status={manifest['mock_replay_status']}")
    print("accepted_for_hardware=false")
    print(f"manifest={manifest_path}")


def finalize(run_dir: Path, renderer: Path, qualification_config: Path) -> None:
    run_dir = run_dir.resolve()
    renderer = renderer.resolve()
    qualification_config = qualification_config.resolve()

    coefficients = run_dir / "coefficients.csv"
    trajectory = run_dir / "trajectory.csv"
    metadata = run_dir / "trajectory.meta.yaml"
    search_report_path = run_dir / "search_report.yaml"
    for path in (
        coefficients,
        trajectory,
        metadata,
        search_report_path,
        qualification_config,
        renderer,
    ):
        if not path.exists():
            raise FileNotFoundError(path)

    search_report = yaml.safe_load(search_report_path.read_text(encoding="utf-8"))
    if search_report.get("status") != "PASS":
        raise ValueError("search_report status is not PASS")
    selected = search_report.get("selected") or {}
    if selected.get("coefficient_sha256") != sha256_file(coefficients):
        raise ValueError("selected coefficient hash does not match frozen coefficients")

    artifact = load_replay_artifact(trajectory, metadata)
    qualification = yaml.safe_load(qualification_config.read_text(encoding="utf-8"))
    report = qualify_replay_artifact(artifact, qualification)
    if report.get("preview_status") != "PASS":
        raise ValueError(f"numerical qualification failed: {report.get('failures')}")
    if report.get("servo_target_delta_design_status") != "PASS":
        raise ValueError(
            "frozen artifact passes the Lower gate but misses the stricter design target"
        )
    if report.get("collision_precheck") != "PASS":
        raise ValueError("frozen artifact collision/model precheck is not PASS")

    preview_report = run_dir / "preview_report.yaml"
    acceptance = run_dir / "preview_acceptance.yaml"
    preview_mp4 = run_dir / "preview.mp4"
    subprocess.run(
        [str(renderer), "--input", str(trajectory), "--output", str(preview_mp4)],
        cwd=ROOT,
        env=_renderer_env(),
        check=True,
    )
    write_preview_outputs(
        report=report,
        report_path=preview_report,
        acceptance_path=acceptance,
        preview_mp4=preview_mp4,
    )

    acceptance_data = yaml.safe_load(acceptance.read_text(encoding="utf-8"))
    if acceptance_data.get("accepted_for_hardware") is not False:
        raise RuntimeError("offline finalization must leave hardware acceptance false")

    mock_template_path = ROOT / "config" / "rebot_excitation_mock.yaml"
    mock = yaml.safe_load(mock_template_path.read_text(encoding="utf-8"))
    mock.update(
        control_rate_hz=float(artifact.sample_rate_hz),
        duration_s=float(artifact.duration_s),
        max_samples=None,
        output_csv=str(run_dir / "mock_replay.csv"),
        allow_hardware=False,
        allow_motion=True,
        trajectory_replay_mode="actual_time_quintic_v1",
        trajectory_hash=artifact.sha256,
        trajectory_artifact=_repo_relative(trajectory),
        trajectory_metadata=_repo_relative(metadata),
        trajectory_preview_acceptance=_repo_relative(acceptance),
    )
    mock_config = run_dir / "mock_replay.yaml"
    mock_config.write_text(
        yaml.safe_dump(mock, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    manifest = {
        "schema_version": "rebot_hardware_excitation_manifest_v1",
        "status": "preview_pending_human_acceptance",
        "hardware_authorized": False,
        "motion_authorized": False,
        "robot": "rebot_dm",
        "sample_rate_hz": artifact.sample_rate_hz,
        "sample_count": len(artifact.samples),
        "duration_s": artifact.duration_s,
        "source_type": "cxx_fourier_trajectory",
        "source_seed": artifact.metadata.get("source_seed"),
        "source_accepted_attempt": artifact.metadata.get("source_accepted_attempt"),
        "coefficient_sha256": sha256_file(coefficients),
        "trajectory_sha256": artifact.sha256,
        "lower_hard_gate_rad": float(
            qualification["maximum_servo_target_delta_rad"]
        ),
        "design_target_rad": float(
            qualification["servo_target_delta_design_limit_rad"]
        ),
        "numerical_qualification": report["preview_status"],
        "design_target_status": report["servo_target_delta_design_status"],
        "collision_precheck": report.get("collision_precheck"),
        "preview_method": "actual_time_quintic_v1_continuous_path",
        "preview_mp4": _repo_relative(preview_mp4),
        "preview_report": _repo_relative(preview_report),
        "preview_acceptance": _repo_relative(acceptance),
        "accepted_for_hardware": False,
        "search_report": _repo_relative(search_report_path),
        "mock_replay_config": _repo_relative(mock_config),
    }
    manifest.update(_mock_replay_summary(run_dir, artifact))
    manifest_path = run_dir / "manifest.yaml"
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    print(f"trajectory_sha256={artifact.sha256}")
    print(f"coefficient_sha256={manifest['coefficient_sha256']}")
    print(f"numerical_qualification={report['preview_status']}")
    print(f"design_target_status={report['servo_target_delta_design_status']}")
    print("accepted_for_hardware=false")
    print(f"preview_mp4={preview_mp4}")
    print(f"manifest={manifest_path}")
    print(f"mock_config={mock_config}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument(
        "--renderer",
        type=Path,
        default=ROOT / "build_rebot" / "rebot_trajectory_renderer",
    )
    parser.add_argument(
        "--qualification-config",
        type=Path,
        default=ROOT / "config" / "rebot_trajectory_preview.yaml",
    )
    parser.add_argument(
        "--refresh-mock-status",
        action="store_true",
        help="validate existing mock_replay.meta.yaml and update only manifest Mock status",
    )
    args = parser.parse_args()
    if args.refresh_mock_status:
        refresh_manifest_mock_status(args.run_directory)
    else:
        finalize(args.run_directory, args.renderer, args.qualification_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
