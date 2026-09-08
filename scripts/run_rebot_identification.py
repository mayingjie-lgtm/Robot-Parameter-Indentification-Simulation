#!/usr/bin/env python3
"""Run offline A/B preprocessing, C++ OLS, provenance capture and reported-effort plots."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rebot_real.preprocessing import preprocess, read_metadata, sha256
from rebot_real.recorder import repository_git_commit


def repo_path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()


def plot_report(prediction_csv: Path, output_dir: Path, reports: dict):
    """Plot each split separately and never join across excluded time segments."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with prediction_csv.open() as stream:
        rows = list(csv.DictReader(stream))
    for split in ("training", "validation"):
        figure, axes = plt.subplots(6, 2, figsize=(14, 15), squeeze=False)
        for j in range(6):
            selected = [r for r in rows if r["split"] == split and int(r["joint"]) == j+1]
            t = np.array([float(r["time"]) for r in selected])
            measured = np.array([float(r["effort_filtered"]) for r in selected])
            predicted = np.array([float(r["effort_predicted"]) for r in selected])
            included = np.array([r["included"] == "1" for r in selected])
            cuts = np.flatnonzero(np.diff(t) > 1.5 / reports[split]["resample_rate_hz"]) + 1
            for part in np.split(np.arange(len(t)), cuts):
                first = part[0] == 0
                axes[j, 0].plot(t[part], measured[part], label="filtered reported effort" if first else None)
                axes[j, 0].plot(t[part], predicted[part], "--", label="prediction" if first else None)
                axes[j, 1].plot(t[part], (predicted-measured)[part])
            axes[j, 1].scatter(t[~included], (predicted-measured)[~included], s=5, color="gray", label="excluded near zero speed")
            axes[j, 0].set_ylabel(f"J{j+1} SDK Nm (uncalibrated)")
            axes[j, 1].set_ylabel("prediction - effort")
        axes[0, 0].legend()
        axes[0, 1].legend()
        axes[-1, 0].set_xlabel("host receive time [s]")
        axes[-1, 1].set_xlabel("host receive time [s]")
        figure.suptitle(f"{split}: reported-effort prediction; not physical torque calibration")
        figure.tight_layout()
        figure.savefig(output_dir / f"{split}.png", dpi=140)
        plt.close(figure)


def run_pipeline(config_path: Path, output_override: Path | None = None) -> dict:
    config = yaml.safe_load(Path(config_path).read_text())
    allowed = {"training_raw_csv", "validation_raw_csv", "model_file", "identify_binary", "output_directory",
               "preprocessing", "rank_relative_tolerance", "friction_velocity_threshold",
               "enable_armature_columns", "enable_damping_columns", "enable_friction_columns"}
    if not isinstance(config, dict) or set(config) - allowed:
        raise ValueError("invalid/unknown offline pipeline config keys")
    a = repo_path(config["training_raw_csv"])
    b = repo_path(config["validation_raw_csv"])
    raw_meta = {"training": read_metadata(a), "validation": read_metadata(b)}
    if a == b or sha256(a) == sha256(b):
        raise ValueError("A/B must be different raw recordings, not renamed copies")
    for key in ("joint_direction", "joint_offset_rad", "j1_convention", "backend"):
        if key not in raw_meta["training"] or raw_meta["training"][key] != raw_meta["validation"].get(key):
            raise ValueError(f"A/B metadata mismatch: {key}")
    hashes = [m.get("trajectory_hash") for m in raw_meta.values()]
    if any(not isinstance(h, str) or len(h) != 64 or any(c not in "0123456789abcdef" for c in h) for h in hashes):
        raise ValueError("A/B require recorded frozen trajectory SHA-256 hashes")
    if hashes[0] == hashes[1]:
        raise ValueError("A/B must use independent excitation trajectory hashes")
    model = repo_path(config.get("model_file", "rebot_dm/rebot_dm.urdf"))
    binary = repo_path(config.get("identify_binary", "build_rebot/identify"))
    if not model.is_file() or not binary.is_file():
        raise FileNotFoundError("model or identify binary missing; build identify first")
    output = repo_path(output_override or config["output_directory"])
    if output.exists():
        raise FileExistsError(f"use a new output directory: {output}")
    for name in ("enable_armature_columns", "enable_damping_columns", "enable_friction_columns"):
        if name in config and type(config[name]) is not bool:
            raise ValueError(f"{name} must be a YAML boolean")
    output.mkdir(parents=True)
    reports = {}
    reports["training"] = preprocess(a, output / "A.csv", options=config.get("preprocessing"))
    reports["validation"] = preprocess(b, output / "B.csv", options=config.get("preprocessing"),
                                        frozen_rate_hz=reports["training"]["resample_rate_hz"])
    # A chooses the only data-derived preprocessing setting (rate). All other
    # hyperparameters are supplied before fitting; B cannot alter any A decision.
    identifier_config = dict(data_mode="real_reported_effort", robot="rebot_dm", algorithm=1,
                             training_data_file=str(output / "A.csv"), validation_data_file=str(output / "B.csv"),
                             basis_data_file=str(output / "A.csv"), model_file=str(model),
                             output_file=str(output / "result.yaml"),
                             friction_observation_mode="moving",
                             rank_relative_tolerance=config.get("rank_relative_tolerance", 1e-6),
                             friction_velocity_threshold=config.get("friction_velocity_threshold", 0.01))
    for name in ("enable_armature_columns", "enable_damping_columns", "enable_friction_columns"):
        identifier_config[name] = config.get(name, True)
    generated = output / "identify.yaml"
    generated.write_text(yaml.safe_dump(identifier_config, sort_keys=False))
    result = subprocess.run([str(binary), "--config", str(generated)], cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output / "identify.log").write_text(result.stdout)
    if result.returncode:
        raise RuntimeError(f"identify failed; see {output / 'identify.log'}\n{result.stdout}")
    result_path = output / "result.yaml"
    report = yaml.safe_load(result_path.read_text())
    report.update(run_status="computed", raw_backend=raw_meta["training"]["backend"],
                  git_commit=repository_git_commit(ROOT),
                  model_sha256=sha256(model), identify_binary_sha256=sha256(binary),
                  config_sha256=sha256(config_path), pipeline_config=config,
                  preprocessing=reports, unresolved_model_calibration=True,
                  result_interpretation="Uncalibrated reported-effort prediction, not physical parameter recovery")
    result_path.write_text(yaml.safe_dump(report, sort_keys=False))
    plot_report(Path(report["prediction_file"]), output, reports)
    report["run_status"] = "completed"
    result_path.write_text(yaml.safe_dump(report, sort_keys=False))
    print(f"result: {result_path}\nA rank: {report['base_parameter_rank']}/{report['full_parameter_count']}")
    print(f"B reported-effort RMSE: {report['validation_error']['aggregate_rmse']:.8g} (SDK Nm, uncalibrated)")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/rebot_real_identification.yaml")
    parser.add_argument("--output-directory", type=Path)
    args = parser.parse_args()
    run_pipeline(args.config, args.output_directory)


if __name__ == "__main__":
    main()
