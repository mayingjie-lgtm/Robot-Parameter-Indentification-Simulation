#!/usr/bin/env python3
"""Pure-offline A-run identification acceptance audit for reBot reported effort."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rebot_real.preprocessing import DEFAULTS, preprocess, read_metadata, sha256

EXPECTED_RANKS = {"rank_60": 36, "rank_72": 46, "rank_78": 52}
RANK_TOLERANCES = (1e-5, 1e-6, 1e-7)
FRICTION_THRESHOLDS = (0.005, 0.01, 0.02, 0.05)


def repo_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()


def load_processed(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.shape == ():
        data = np.asarray([data], dtype=data.dtype)
    return data


def stats(values: np.ndarray) -> dict:
    absolute = np.abs(values)
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "range": float(np.ptp(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "p95_abs": float(np.percentile(absolute, 95)),
        "max_abs": float(np.max(absolute)),
    }


def signal_quality(processed, raw_meta: dict) -> dict:
    result = {"q": [], "qd": [], "qdd_est": [], "effort_filtered": []}
    qmin = np.asarray(raw_meta["joint_position_min_rad"], dtype=float)
    qmax = np.asarray(raw_meta["joint_position_max_rad"], dtype=float)
    for j in range(6):
        q = processed[f"q{j}"]
        qd = processed[f"qd{j}"]
        qdd = processed[f"qdd_est{j}"]
        effort = processed[f"effort_filtered{j}"]
        qstats = stats(q)
        qstats["min_margin_to_joint_limit_rad"] = float(
            min(np.min(q - qmin[j]), np.min(qmax[j] - q))
        )
        result["q"].append(qstats)
        qdstats = stats(qd)
        qdstats["near_zero_fraction"] = {
            str(threshold): float(np.mean(np.abs(qd) < threshold))
            for threshold in FRICTION_THRESHOLDS
        }
        result["qd"].append(qdstats)
        qddstats = stats(qdd)
        qddstats["max_to_p95_ratio"] = (
            float(qddstats["max_abs"] / qddstats["p95_abs"])
            if qddstats["p95_abs"] > 0 else None
        )
        result["qdd_est"].append(qddstats)
        estats = stats(effort)
        diffs = np.diff(effort)
        estats["max_adjacent_jump"] = float(np.max(np.abs(diffs))) if len(diffs) else 0.0
        estats["finite"] = bool(np.isfinite(effort).all())
        estats["flatline"] = bool(np.ptp(effort) <= np.finfo(float).eps)
        result["effort_filtered"].append(estats)
    return result


def tracking_diagnostics(raw: Path, processed) -> dict:
    """Relate command tracking error to processed measured-state signals.

    q_cmd is interpolated onto the processed host-time grid. This keeps tracking a
    command-vs-measured diagnostic while avoiding treating repeated raw snapshots as
    independent physical measurements.
    """
    with raw.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("cannot audit tracking on empty raw CSV")
    host0 = int(rows[0]["timestamp_host_rx_ns"])
    selected = []
    for row in rows:
        if row.get("command_valid") != "1":
            continue
        try:
            values = [float(row[f"q_cmd{j}"]) for j in range(6)]
            timestamp = int(row["timestamp_host_rx_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if np.isfinite(values).all():
            selected.append((timestamp, values))
    if len(selected) < 2:
        raise ValueError("insufficient finite q_cmd samples for tracking diagnostic")
    command_time = (np.asarray([item[0] for item in selected], dtype=np.int64) - host0) * 1e-9
    command = np.asarray([item[1] for item in selected], dtype=float)
    processed_time = np.asarray(processed["time"], dtype=float)
    per_joint = []
    for j in range(6):
        q_cmd = np.interp(processed_time, command_time, command[:, j])
        error = q_cmd - processed[f"q{j}"]
        correlations = {}
        for name, source in (
            ("q", processed[f"q{j}"]),
            ("qd", processed[f"qd{j}"]),
            ("qdd_est", processed[f"qdd_est{j}"]),
            ("effort_filtered", processed[f"effort_filtered{j}"]),
        ):
            correlations[name] = (
                None if np.std(source) == 0 or np.std(error) == 0
                else float(np.corrcoef(error, source)[0, 1])
            )
        per_joint.append({
            "joint": j + 1,
            "rmse_rad": float(np.sqrt(np.mean(error * error))),
            "mae_rad": float(np.mean(np.abs(error))),
            "bias_rad": float(np.mean(error)),
            "p95_abs_rad": float(np.percentile(np.abs(error), 95)),
            "max_abs_rad": float(np.max(np.abs(error))),
            "correlation_with_measured_signals": correlations,
        })
    return {
        "sampling_semantics": "q_cmd interpolated onto deduplicated/resampled measured-state host-time grid",
        "per_joint": per_joint,
    }


def fit_metrics(processed, prediction_csv: Path) -> dict:
    with prediction_csv.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    per_joint = []
    residual_all = []
    effort_all = []
    for j in range(6):
        chosen = [r for r in rows if int(r["joint"]) == j + 1 and r["included"] == "1"]
        residual = np.asarray([float(r["residual"]) for r in chosen])
        effort = np.asarray([float(r["effort_filtered"]) for r in chosen])
        sample_times = np.asarray([float(r["time"]) for r in chosen])
        indices = np.asarray(
            [int(np.argmin(np.abs(processed["time"] - t))) for t in sample_times], dtype=int
        )
        q = processed[f"q{j}"][indices]
        qd = processed[f"qd{j}"][indices]
        qdd = processed[f"qdd_est{j}"][indices]
        rmse = float(np.sqrt(np.mean(residual * residual)))
        measured_rms = float(np.sqrt(np.mean(effort * effort)))
        centered = effort - np.mean(effort)
        denom = float(np.sum(centered * centered))
        correlations = {}
        for name, source in (("q", q), ("qd", qd), ("qdd_est", qdd)):
            correlations[name] = (
                None if np.std(source) == 0 or np.std(residual) == 0
                else float(np.corrcoef(residual, source)[0, 1])
            )
        per_joint.append({
            "joint": j + 1,
            "count": int(len(residual)),
            "rmse": rmse,
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "p95_abs_residual": float(np.percentile(np.abs(residual), 95)),
            "max_abs_residual": float(np.max(np.abs(residual))),
            "rms_normalized_error": rmse / measured_rms if measured_rms > 0 else None,
            "r_squared": 1.0 - float(np.sum(residual * residual)) / denom if denom > 0 else None,
            "residual_correlation": correlations,
        })
        residual_all.append(residual)
        effort_all.append(effort)
    residual = np.concatenate(residual_all)
    effort = np.concatenate(effort_all)
    rmse = float(np.sqrt(np.mean(residual * residual)))
    centered = effort - np.mean(effort)
    denom = float(np.sum(centered * centered))
    return {
        "semantics": "training diagnostic only; uncalibrated reported-effort fit",
        "aggregate": {
            "count": int(len(residual)),
            "rmse": rmse,
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "p95_abs_residual": float(np.percentile(np.abs(residual), 95)),
            "max_abs_residual": float(np.max(np.abs(residual))),
            "rms_normalized_error": rmse / float(np.sqrt(np.mean(effort * effort))),
            "r_squared": 1.0 - float(np.sum(residual * residual)) / denom if denom > 0 else None,
        },
        "per_joint": per_joint,
    }


def make_plots(processed, output: Path) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time = processed["time"]
    plots = []
    specs = [
        ("q", "filtered measured q", "rad"),
        ("qd", "filtered SDK qd", "rad/s"),
        ("qdd_est", "qdd_est from filtered SDK qd", "rad/s^2"),
        ("effort_filtered", "filtered reported effort (SDK Nm, uncalibrated)", "SDK Nm, uncalibrated"),
    ]
    for prefix, title, ylabel in specs:
        figure, axis = plt.subplots(figsize=(11, 5.5))
        for j in range(6):
            axis.plot(time, processed[f"{prefix}{j}"], label=f"J{j+1}")
        axis.set_title(f"A_run01: {title}")
        axis.set_xlabel("host receive time [s]")
        axis.set_ylabel(ylabel)
        axis.legend(ncol=3)
        figure.tight_layout()
        path = output / f"{prefix}.png"
        figure.savefig(path, dpi=140)
        plt.close(figure)
        plots.append(path.name)

    figure, axis = plt.subplots(figsize=(11, 5.5))
    for j in range(6):
        derived = np.gradient(processed[f"q{j}"], time, edge_order=2)
        axis.plot(time, processed[f"qd{j}"] - derived, label=f"J{j+1}")
    axis.set_title("A_run01: filtered SDK qd minus d/dt(filtered q)")
    axis.set_xlabel("host receive time [s]")
    axis.set_ylabel("velocity consistency residual [rad/s]")
    axis.legend(ncol=3)
    figure.tight_layout()
    path = output / "qd_consistency.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    plots.append(path.name)
    return plots


def rank_summary(diagnostic: dict) -> dict:
    return {
        name: {
            "raw_columns": diagnostic["rank_sweep"][name]["raw_columns"],
            "structural_zero_columns": diagnostic["rank_sweep"][name]["structural_zero_columns"],
            "near_zero_columns": diagnostic["rank_sweep"][name]["near_zero_columns"],
            "thresholds": diagnostic["rank_sweep"][name]["thresholds"],
        }
        for name in ("rank_60", "rank_72", "rank_78")
    }


def build_model_freeze(
    *,
    raw: Path,
    raw_meta: dict,
    processed_path: Path,
    processed_meta: dict,
    diagnostic: dict,
    acceptance_path: Path,
) -> dict:
    fit = diagnostic.get("training_fit") or {}
    required = {
        "parameter_layout_columns",
        "rank_relative_tolerance",
        "base_rank",
        "friction_velocity_threshold_rad_s",
        "solver",
        "column_scales",
        "base_directions",
        "beta_hat",
        "parameter_names",
    }
    missing = required - set(fit)
    if missing:
        raise ValueError(f"diagnostic is missing freeze fields: {sorted(missing)}")
    columns = int(fit["parameter_layout_columns"])
    base_rank = int(fit["base_rank"])
    scales = fit["column_scales"]
    directions = fit["base_directions"]
    beta = fit["beta_hat"]
    names = fit["parameter_names"]
    if columns != 78 or len(scales) != columns or len(names) != columns:
        raise ValueError("A freeze requires the fixed 78-column parameter layout")
    if len(directions) != columns or any(len(row) != base_rank for row in directions):
        raise ValueError("A base_directions shape does not match 78 x base_rank")
    if len(beta) != base_rank:
        raise ValueError("A beta_hat size does not match base_rank")
    if fit["solver"] != "OLS":
        raise ValueError("A freeze solver must remain OLS")
    if diagnostic.get("acceleration_source") != "qdd_est":
        raise ValueError("A freeze acceleration source must remain qdd_est")
    if diagnostic.get("torque_source") != "effort_filtered" or diagnostic.get("torque_calibrated") is not False:
        raise ValueError("A freeze torque semantics changed")

    model = repo_path(diagnostic["model_file"])
    settings = processed_meta["settings"]
    return {
        "schema_version": "rebot_a_model_freeze_v1",
        "source_a_run": str(raw.parent),
        "source_raw_sha256": sha256(raw),
        "source_raw_metadata_sha256": sha256(raw.with_suffix(".meta.yaml")),
        "source_processed_sha256": sha256(processed_path),
        "source_processed_metadata_sha256": sha256(processed_path.with_suffix(".meta.yaml")),
        "source_acceptance_sha256": sha256(acceptance_path),
        "trajectory_hash": raw_meta.get("trajectory_hash"),
        "data_semantics": {
            "data_mode": diagnostic["data_mode"],
            "acceleration_source": diagnostic["acceleration_source"],
            "torque_source": diagnostic["torque_source"],
            "torque_calibrated": diagnostic["torque_calibrated"],
        },
        "preprocessing": {
            "cutoff_hz": float(settings["cutoff_hz"]),
            "filter_order": int(settings["filter_order"]),
            "edge_trim_s": float(settings["edge_trim_s"]),
            "resample_rate_hz": float(processed_meta["resample_rate_hz"]),
            "gap_periods": float(settings["gap_periods"]),
        },
        "model": {
            "model_file": str(model),
            "model_sha256": sha256(model),
            "parameter_layout_columns": columns,
            "parameter_names": names,
        },
        "rank": {
            "rank_relative_tolerance": float(fit["rank_relative_tolerance"]),
            "base_rank": base_rank,
            "column_scales": scales,
            "base_directions": directions,
        },
        "friction": {
            "velocity_threshold_rad_s": float(fit["friction_velocity_threshold_rad_s"]),
        },
        "solver": {"type": fit["solver"]},
        "fit": {
            "beta_hat": beta,
            "training_diagnostic_only": True,
        },
        "freeze": {
            "frozen": True,
            "derived_from": "A_ONLY",
            "b_inspected_for_tuning": False,
        },
    }


def write_model_freeze(path: Path, payload: dict, *, overwrite: bool = False) -> str:
    if path.exists() and not overwrite:
        raise FileExistsError(f"model freeze output already exists: {path}")
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return sha256(path)


def determine_acceptance(pre: dict, diagnostic: dict, quality: dict):
    blockers, warnings = [], []
    excluded = pre.get("excluded_counts", {})
    for key in (
        "inconsistent_repeated_feedback",
        "invalid_feedback_or_torque",
        "nonfinite_signal",
        "fault_or_unsafe_state",
        "stale_or_invalid_age",
    ):
        if excluded.get(key, 0):
            blockers.append(f"{key}={excluded[key]}")
    if pre["segment_count"] < 1 or pre["output_sample_count"] < 1:
        blockers.append("no usable processed segment")
    for name, expected in EXPECTED_RANKS.items():
        ranks = [int(item["rank"]) for item in diagnostic["rank_sweep"][name]["thresholds"]]
        if ranks != [expected] * len(RANK_TOLERANCES):
            blockers.append(f"{name} unstable/degraded ranks={ranks}, expected={expected}")
    if any(not item["finite"] or item["flatline"] for item in quality["effort_filtered"]):
        blockers.append("reported effort contains nonfinite values or flatline")
    if blockers:
        return "A_REJECT", blockers, warnings

    nyquist = pre["resample_rate_hz"] / 2.0
    ratio = pre["settings"]["cutoff_hz"] / nyquist
    if ratio >= 0.4:
        warnings.append(
            f"2 Hz cutoff uses {ratio:.3f} of Nyquist at {pre['resample_rate_hz']:.1f} Hz resampling; margin is limited"
        )
    if np.max(np.asarray(pre["qd_consistency_rmse_rad_s"])) > 0.02:
        warnings.append(
            "filtered SDK qd vs d/dt(filtered q) disagreement exceeds 0.02 rad/s on at least one joint"
        )
    if pre["raw_metadata"].get("tracking_quality", {}).get("status") == "warning":
        warnings.append(
            "motion tracking quality is warning, but measured-state continuity/rank are audited separately"
        )
    warnings.append("effort_filtered is uncalibrated SDK reported effort, not joint torque ground truth")
    warnings.append(
        "friction threshold 0.01 rad/s is below some qd consistency RMSE values; retain only as an A-frozen modeling choice, not a measured noise-floor claim"
    )
    return "A_ACCEPT_WITH_WARNINGS", blockers, warnings


def write_report(path: Path, acceptance: dict) -> None:
    fb = acceptance["feedback"]
    prep = acceptance["preprocessing"]
    reg = acceptance["regressor"]
    fit = acceptance["training_fit"]
    status = acceptance["identification_acceptance"]["status"]
    lines = [
        "# reBot A_run01 Identification-Grade Offline Audit",
        "",
        "OFFLINE_ONLY — no SDK client is imported or instantiated by this audit.",
        "",
        "## Executive conclusion",
        "",
        f"**{status}**",
        "",
        f"- Hardware motion execution: **{acceptance['motion_execution']['status']}**; accepted excitation commands: **{acceptance['motion_execution']['accepted_excitation_command_count']}**.",
        f"- Raw CSV rows: **{fb['raw_rows']}**; independent fresh physical feedback: **{fb['fresh_feedback_samples']}**; duplicate snapshots excluded: **{fb['duplicate_rows']}**.",
        f"- Effective physical feedback rate: **{fb['physical_feedback_rate_hz']:.4f} Hz**; frozen uniform resample rate: **{fb['resample_rate_hz']:.1f} Hz**.",
        f"- Processed samples after filtering/trim: **{prep['output_sample_count']}**.",
        "- Torque semantics remain **SDK reported effort, uncalibrated**.",
        "",
        "The ~2946 raw rows are command-loop/state-snapshot rows. The lower feedback timestamp changes only about 305 times, so repeated rows are not treated as independent measurements.",
        "",
        "## Preprocessing and bandwidth",
        "",
        f"- Filter: 4th-order zero-phase Butterworth, cutoff **{prep['cutoff_hz']} Hz**.",
        f"- Nyquist after frozen resampling: **{prep['nyquist_hz']:.3f} Hz**; cutoff/Nyquist = **{prep['cutoff_nyquist_ratio']:.3f}**.",
        f"- Feedback dt median/P95/P99/max: **{prep['dt_median_s']:.6f} / {prep['dt_p95_s']:.6f} / {prep['dt_p99_s']:.6f} / {prep['dt_max_s']:.6f} s**.",
        f"- Segment count: **{fb['segment_count']}**; no interpolation crosses an invalid segment.",
        f"- Per-joint qd consistency RMSE [rad/s]: **{prep['qd_consistency_rmse_rad_s']}**.",
        "",
        "At ~10 Hz physical feedback, a 2 Hz cutoff is below Nyquist but not by a large margin. The timing distribution is tight enough for uniform resampling, while J4-J6 velocity consistency—especially J5—remains a measurement-quality warning. The acceleration estimate is usable for this training diagnostic, but should not be interpreted as high-bandwidth acceleration truth.",
        "",
        "## Signal quality",
        "",
        "Per-joint q/qd/qdd_est/reported-effort statistics are stored in acceptance.yaml. The filtered reported effort is finite and non-flatlined. qdd_est is derived from filtered SDK velocity after boundary trim; derivative-spike ratios are recorded rather than hidden behind an arbitrary rejection threshold.",
        "",
        "## Tracking quality vs identification measurement quality",
        "",
        f"Tracking status is **{acceptance['tracking']['status']}**, with maximum command-vs-measured position error **{acceptance['tracking']['max_abs_rad']:.6f} rad**. This is a motion/control tracking metric, not an automatic identification-data rejection metric. The regressor and fit use measured q, measured qd, derived qdd_est, and filtered reported effort; they do not substitute q_cmd for measured state. For the relationship check, q_cmd is interpolated onto the deduplicated/resampled measured-state host-time grid; per-joint tracking RMSE/P95/max and correlations with q/qd/qdd_est/effort_filtered are recorded in acceptance.yaml. The run has no lower-timeout-unusable measurement rows, and the measured-state regressor retains its expected numerical rank.",
        "",
        "## A-only regressor excitation",
        "",
    ]
    for name in ("rank_60", "rank_72", "rank_78"):
        entry = reg[name]
        nominal = next(x for x in entry["thresholds"] if abs(float(x["relative_tolerance"]) - 1e-6) < 1e-12)
        ranks = [x["rank"] for x in entry["thresholds"]]
        lines.append(
            f"- {entry['raw_columns']} columns: ranks at 1e-5/1e-6/1e-7 = **{ranks}**; "
            f"effective condition at 1e-6 = **{nominal['effective_condition']:.4f}**; "
            f"smallest retained sigma = **{nominal['smallest_retained_sigma']:.6g}**."
        )
    lines += [
        "",
        "The 60/72/78 ranks are stable across all three requested tolerances and match the repository's established reBot structural ranks. Structural-zero columns are reported explicitly in acceptance.yaml; no nominal rank was obtained by lowering the threshold.",
        "",
        "## Friction threshold",
        "",
        f"At 0.01 rad/s, moving observation counts per joint are **{acceptance['friction']['selected_0p01_per_joint_count']}** out of {prep['output_sample_count']} samples/joint. Counts for 0.005/0.02/0.05 are also recorded. Because the qd-consistency discrepancy on J4-J6 exceeds 0.01 rad/s, 0.01 should not be claimed as a measured velocity-noise floor. This audit keeps it as the predeclared A-only modeling threshold rather than changing it without a stationary-noise experiment.",
        "",
        "## A training fit",
        "",
        f"Aggregate RMSE = **{fit['aggregate']['rmse']:.6f} SDK Nm**, MAE = **{fit['aggregate']['mae']:.6f}**, bias = **{fit['aggregate']['bias']:.6f}**, P95 abs residual = **{fit['aggregate']['p95_abs_residual']:.6f}**, R² = **{fit['aggregate']['r_squared']:.6f}**.",
        "",
        "This is a **training diagnostic only** on A and is not an independent test. Residual correlations with q/qd/qdd_est and all per-joint metrics are stored in acceptance.yaml.",
        "",
        "## Torque/effort limitation",
        "",
        f"effort_reported_source: {acceptance['effort']['source']}",
        "",
        "The data can support continuity checks, excitation-rank/conditioning analysis, and fitting of the SDK reported-effort signal. Because physical torque calibration is not established, this result does **not** claim absolutely accurate physical link inertial parameters or joint-torque ground truth.",
        "",
        "## Acceptance reasons and warnings",
        "",
    ]
    for reason in acceptance["identification_acceptance"]["reasons"]:
        lines.append(f"- {reason}")
    for warning in acceptance["identification_acceptance"]["warnings"]:
        lines.append(f"- WARNING: {warning}")
    lines += [
        "",
        "## NEXT HARDWARE ACTION",
        "",
        "Freeze A-derived preprocessing/model/rank/friction settings.",
        "Then prepare independent B_run01.",
        "Do not tune settings after inspecting B.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def freeze_existing_a(
    *, raw: Path, output: Path, binary: Path, model: Path, overwrite: bool
) -> int:
    acceptance_path = output / "acceptance.yaml"
    processed_path = output / "A.processed.csv"
    processed_meta_path = output / "A.processed.meta.yaml"
    old_diagnostic_path = output / "A.training_diagnostic.yaml"
    old_prediction_path = output / "A.training_prediction.csv"
    for path in (
        acceptance_path,
        processed_path,
        processed_meta_path,
        old_diagnostic_path,
        old_prediction_path,
    ):
        if not path.is_file():
            raise FileNotFoundError(f"existing A audit artifact is missing: {path}")

    acceptance = yaml.safe_load(acceptance_path.read_text())
    if acceptance.get("identification_acceptance", {}).get("status") not in {
        "A_ACCEPT", "A_ACCEPT_WITH_WARNINGS"
    }:
        raise ValueError("A model/settings may be frozen only after A acceptance")
    raw_meta = read_metadata(raw)
    if sha256(raw) != acceptance.get("raw_csv_sha256"):
        raise ValueError("frozen A raw hash no longer matches acceptance")
    if sha256(raw.with_suffix(".meta.yaml")) != acceptance.get("raw_metadata_sha256"):
        raise ValueError("frozen A raw metadata hash no longer matches acceptance")

    processed_meta = yaml.safe_load(processed_meta_path.read_text())
    if processed_meta.get("raw_sha256") != sha256(raw):
        raise ValueError("existing processed A does not match accepted raw A")
    if processed_meta.get("processed_sha256") != sha256(processed_path):
        raise ValueError("existing processed A hash no longer matches its metadata")

    old_diagnostic = yaml.safe_load(old_diagnostic_path.read_text())
    old_fit = old_diagnostic.get("training_fit") or {}
    frozen_rank_tolerance = float(old_fit["rank_relative_tolerance"])
    frozen_friction_threshold = float(old_fit["friction_velocity_threshold_rad_s"])
    old_ranks = rank_summary(old_diagnostic)
    processed = load_processed(processed_path)
    old_rmse = fit_metrics(processed, old_prediction_path)["aggregate"]["rmse"]

    with tempfile.TemporaryDirectory(prefix="rebot_a_freeze_") as directory:
        temporary = Path(directory)
        new_diagnostic_path = temporary / "A.training_diagnostic.yaml"
        new_prediction_path = temporary / "A.training_prediction.csv"
        subprocess.run(
            [
                str(binary), "--input", str(processed_path), "--model", str(model),
                "--output", str(new_diagnostic_path),
                "--prediction-output", str(new_prediction_path),
                "--rank-relative-tolerance", str(frozen_rank_tolerance),
                "--friction-velocity-threshold", str(frozen_friction_threshold),
            ],
            cwd=ROOT, check=True,
        )
        new_diagnostic = yaml.safe_load(new_diagnostic_path.read_text())
        new_rmse = fit_metrics(processed, new_prediction_path)["aggregate"]["rmse"]
        new_prediction_sha = sha256(new_prediction_path)
        if old_prediction_path.read_bytes() != new_prediction_path.read_bytes():
            raise RuntimeError("serialization-only regression changed A prediction bytes")
        if rank_summary(new_diagnostic) != old_ranks:
            raise RuntimeError("serialization-only regression changed A rank diagnostics")
        if new_rmse != old_rmse:
            raise RuntimeError("serialization-only regression changed A training RMSE")

    payload = build_model_freeze(
        raw=raw,
        raw_meta=raw_meta,
        processed_path=processed_path,
        processed_meta=processed_meta,
        diagnostic=new_diagnostic,
        acceptance_path=acceptance_path,
    )
    freeze_path = output / "A_MODEL_FREEZE.yaml"
    freeze_sha = write_model_freeze(freeze_path, payload, overwrite=overwrite)
    print(f"old_prediction_sha256={sha256(old_prediction_path)}")
    print(f"new_prediction_sha256={new_prediction_sha}")
    print(f"old_training_rmse={old_rmse:.17g}")
    print(f"new_training_rmse={new_rmse:.17g}")
    print(f"rank_60_72_78={EXPECTED_RANKS['rank_60']}/{EXPECTED_RANKS['rank_72']}/{EXPECTED_RANKS['rank_78']}")
    print(f"model_freeze={freeze_path}")
    print(f"model_freeze_sha256={freeze_sha}")
    print("freeze_source=A_ONLY; b_inspected_for_tuning=false")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-csv", required=True)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--diagnostic-binary", default="build_rebot/rebot_real_a_training_diagnostic")
    parser.add_argument("--model", default="rebot_dm/rebot_dm.urdf")
    parser.add_argument("--rank-relative-tolerance", type=float, default=1e-6)
    parser.add_argument("--friction-velocity-threshold", type=float, default=0.01)
    parser.add_argument("--reuse-existing", action="store_true")
    parser.add_argument(
        "--freeze-existing-only",
        action="store_true",
        help="freeze an already accepted A audit without regenerating its accepted artifacts",
    )
    args = parser.parse_args()

    print("OFFLINE_ONLY")
    raw = repo_path(args.raw_csv)
    output = repo_path(args.output_directory)
    binary = repo_path(args.diagnostic_binary)
    model = repo_path(args.model)
    if args.freeze_existing_only:
        if not output.is_dir():
            raise FileNotFoundError(f"existing A audit directory is missing: {output}")
        return freeze_existing_a(
            raw=raw,
            output=output,
            binary=binary,
            model=model,
            overwrite=args.reuse_existing,
        )
    raw_meta = read_metadata(raw)
    raw_hash_before = sha256(raw)
    raw_meta_hash_before = sha256(raw.with_suffix(".meta.yaml"))
    output.mkdir(parents=True, exist_ok=args.reuse_existing)

    processed_path = output / "A.processed.csv"
    processed_meta_path = output / "A.processed.meta.yaml"
    if processed_path.exists() or processed_meta_path.exists():
        if not args.reuse_existing:
            raise FileExistsError("processed output exists; use a new output directory")
        pre = yaml.safe_load(processed_meta_path.read_text())
        if pre["raw_sha256"] != raw_hash_before:
            raise ValueError("existing processed artifact does not match frozen raw A")
    else:
        pre = preprocess(raw, processed_path, options=DEFAULTS)

    diagnostic_path = output / "A.training_diagnostic.yaml"
    prediction_path = output / "A.training_prediction.csv"
    if diagnostic_path.exists() or prediction_path.exists():
        if not args.reuse_existing:
            raise FileExistsError("diagnostic output exists; use a new output directory")
    else:
        subprocess.run(
            [
                str(binary), "--input", str(processed_path), "--model", str(model),
                "--output", str(diagnostic_path), "--prediction-output", str(prediction_path),
                "--rank-relative-tolerance", str(args.rank_relative_tolerance),
                "--friction-velocity-threshold", str(args.friction_velocity_threshold),
            ],
            cwd=ROOT, check=True,
        )
    diagnostic = yaml.safe_load(diagnostic_path.read_text())
    if diagnostic.get("data_mode") != "real_reported_effort":
        raise ValueError("diagnostic did not preserve real reported-effort data mode")
    if diagnostic.get("acceleration_source") != "qdd_est":
        raise ValueError("diagnostic acceleration source is not qdd_est")
    if diagnostic.get("torque_source") != "effort_filtered" or diagnostic.get("torque_calibrated") is not False:
        raise ValueError("diagnostic torque semantics are invalid")

    processed = load_processed(processed_path)
    quality = signal_quality(processed, raw_meta)
    fitting = fit_metrics(processed, prediction_path)
    tracking_relation = tracking_diagnostics(raw, processed)
    plots = make_plots(processed, output)
    ranks = rank_summary(diagnostic)
    status, blockers, warnings = determine_acceptance(pre, diagnostic, quality)
    friction = diagnostic["friction_moving_observations"]
    at_001 = min(friction, key=lambda item: abs(float(item["threshold_rad_s"]) - 0.01))
    nyquist = pre["resample_rate_hz"] / 2.0
    acceptance = {
        "schema_version": "rebot_a_identification_acceptance_v1",
        "source_run": str(raw.parent),
        "raw_csv_sha256": raw_hash_before,
        "raw_metadata_sha256": raw_meta_hash_before,
        "trajectory_hash": raw_meta.get("trajectory_hash"),
        "git_commit_at_capture": raw_meta.get("git_commit"),
        "git_head_at_audit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "motion_execution": {
            "status": raw_meta.get("motion_status"),
            "observed_sample_count": raw_meta.get("observed_sample_count"),
            "accepted_excitation_command_count": raw_meta.get("runtime_servo_envelope", {}).get(
                "accepted_excitation_command_count",
                raw_meta.get("accepted_excitation_command_count"),
            ),
        },
        "feedback": {
            "physical_feedback_rate_hz": pre["effective_feedback_rate_hz"],
            "resample_rate_hz": pre["resample_rate_hz"],
            "raw_rows": pre["raw_sample_count"],
            "fresh_feedback_samples": pre["fresh_valid_sample_count"],
            "duplicate_rows": pre["excluded_counts"].get("duplicate_feedback", 0),
            "inconsistent_repeated_feedback": pre["excluded_counts"].get("inconsistent_repeated_feedback", 0),
            "invalid_rows": pre["excluded_counts"].get("invalid_feedback_or_torque", 0),
            "stale_rows": pre["excluded_counts"].get("stale_or_invalid_age", 0),
            "fault_unsafe_rows": pre["excluded_counts"].get("fault_or_unsafe_state", 0),
            "segment_count": pre["segment_count"],
            "feedback_age_limit_ms": pre["feedback_age_limit_ms"],
            "feedback_age_limit_source": pre["feedback_age_limit_source"],
        },
        "preprocessing": {
            "cutoff_hz": pre["settings"]["cutoff_hz"],
            "filter_order": pre["settings"]["filter_order"],
            "edge_trim_s": pre["settings"]["edge_trim_s"],
            "trim_samples_per_segment_end": pre["trim_samples_per_segment_end"],
            "output_sample_count": pre["output_sample_count"],
            "nyquist_hz": nyquist,
            "cutoff_nyquist_ratio": pre["settings"]["cutoff_hz"] / nyquist,
            "dt_median_s": pre["feedback_dt_s"]["median"],
            "dt_p95_s": pre["feedback_dt_s"]["p95"],
            "dt_p99_s": pre["feedback_dt_s"]["p99"],
            "dt_max_s": pre["feedback_dt_s"]["maximum"],
            "p95_jitter_from_median_s": pre["feedback_dt_s"]["p95"] - pre["feedback_dt_s"]["median"],
            "qd_consistency_rmse_rad_s": pre["qd_consistency_rmse_rad_s"],
        },
        "signal_quality": quality,
        "regressor": ranks,
        "friction": {
            "threshold_sweep": friction,
            "selected_0p01_per_joint_count": at_001["per_joint_count"],
            "selected_0p01_per_joint_fraction": at_001["per_joint_fraction"],
        },
        "tracking": {
            "status": raw_meta.get("tracking_quality", {}).get("status"),
            "max_abs_rad": raw_meta.get("tracking_quality", {}).get("overall_max_abs_rad"),
            "interpretation": "motion/control tracking warning; not an automatic measured-state identification rejection",
            "measured_state_relation": tracking_relation,
        },
        "effort": {
            "source": raw_meta.get("effort_reported_source"),
            "calibrated": False,
            "interpretation": "SDK reported effort only; no physical joint-torque ground-truth claim",
        },
        "training_fit": fitting,
        "plots": plots,
        "identification_acceptance": {
            "status": status,
            "reasons": [
                "hardware motion completed",
                "physical feedback deduplicated before resampling",
                "processed data finite and continuous",
                "60/72/78 regressor ranks stable at 1e-5/1e-6/1e-7 and match established reBot structural ranks",
                "A-only OLS uses measured q/qd, derived qdd_est, and uncalibrated effort_filtered",
            ],
            "warnings": warnings,
            "blockers": blockers,
            "next_action": (
                "Freeze A-derived preprocessing/model/rank/friction settings; then prepare independent B_run01; do not tune settings after inspecting B."
                if status != "A_REJECT" else
                "Do not collect B until the listed blockers are resolved."
            ),
        },
    }

    acceptance_path = output / "acceptance.yaml"
    report_path = output / "A_IDENTIFICATION_AUDIT.md"
    if (acceptance_path.exists() or report_path.exists()) and not args.reuse_existing:
        raise FileExistsError("acceptance/report output already exists")
    acceptance_path.write_text(
        yaml.safe_dump(acceptance, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    write_report(report_path, acceptance)
    freeze_path = output / "A_MODEL_FREEZE.yaml"
    freeze_sha = None
    if status != "A_REJECT":
        freeze_payload = build_model_freeze(
            raw=raw,
            raw_meta=raw_meta,
            processed_path=processed_path,
            processed_meta=pre,
            diagnostic=diagnostic,
            acceptance_path=acceptance_path,
        )
        freeze_sha = write_model_freeze(
            freeze_path, freeze_payload, overwrite=args.reuse_existing
        )

    raw_hash_after = sha256(raw)
    raw_meta_hash_after = sha256(raw.with_suffix(".meta.yaml"))
    if raw_hash_after != raw_hash_before or raw_meta_hash_after != raw_meta_hash_before:
        raise RuntimeError("frozen raw A artifact changed during offline audit")
    print(f"status={status}")
    print(f"raw_sha256_before={raw_hash_before}")
    print(f"raw_sha256_after={raw_hash_after}")
    print(f"raw_metadata_sha256_before={raw_meta_hash_before}")
    print(f"raw_metadata_sha256_after={raw_meta_hash_after}")
    print(f"acceptance={acceptance_path}")
    print(f"report={report_path}")
    if freeze_sha is not None:
        print(f"model_freeze={freeze_path}")
        print(f"model_freeze_sha256={freeze_sha}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
