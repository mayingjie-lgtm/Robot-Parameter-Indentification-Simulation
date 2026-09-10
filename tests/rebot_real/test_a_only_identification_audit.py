from pathlib import Path
import csv
import subprocess
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src"))

from audit_rebot_hardware_identification_A import load_processed, tracking_diagnostics
from generate_rebot_identification_demo import generate
from rebot_real.preprocessing import preprocess
from run_rebot_identification import run_pipeline

BUILD = ROOT / "build_rebot"
BINARY = BUILD / "rebot_real_a_training_diagnostic"
pytestmark = pytest.mark.skipif(
    not (
        (BUILD / "rebot_reported_effort_fixture").is_file()
        and (BUILD / "identify").is_file()
        and BINARY.is_file()
    ),
    reason="build rebot_reported_effort_fixture, identify, and rebot_real_a_training_diagnostic first",
)


def run_a_only(processed: Path, output_dir: Path):
    output_dir.mkdir(parents=True)
    result_yaml = output_dir / "diagnostic.yaml"
    prediction_csv = output_dir / "prediction.csv"
    completed = subprocess.run(
        [
            str(BINARY),
            "--input",
            str(processed),
            "--model",
            str(ROOT / "rebot_dm/rebot_dm.urdf"),
            "--output",
            str(result_yaml),
            "--prediction-output",
            str(prediction_csv),
            "--rank-relative-tolerance",
            "1e-6",
            "--friction-velocity-threshold",
            "0.01",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed, yaml.safe_load(result_yaml.read_text()), prediction_csv


def test_a_only_diagnostic_uses_explicit_real_columns_and_has_no_b_dependency(tmp_path):
    demo = tmp_path / "demo"
    generate(demo, BUILD)
    processed = tmp_path / "A.processed.csv"
    preprocess(demo / "A.raw.csv", processed)

    (demo / "B.raw.csv").unlink()
    (demo / "B.raw.meta.yaml").unlink()

    completed, report, prediction = run_a_only(processed, tmp_path / "a_only")
    assert completed.stdout.startswith("OFFLINE_ONLY")
    assert report["data_mode"] == "real_reported_effort"
    assert report["acceleration_source"] == "qdd_est"
    assert report["torque_source"] == "effort_filtered"
    assert report["torque_calibrated"] is False
    assert report["training_diagnostic_only"] is True
    text = yaml.safe_dump(report, sort_keys=False).lower()
    assert "validation" not in text
    assert "generalization" not in text
    with prediction.open() as stream:
        header = next(csv.reader(stream))
    assert header == [
        "time",
        "joint",
        "effort_filtered",
        "effort_predicted",
        "residual",
        "included",
    ]


def test_a_only_training_math_matches_existing_real_ab_a_side(tmp_path):
    demo = tmp_path / "demo"
    generate(demo, BUILD)
    pipeline = run_pipeline(demo / "pipeline.yaml")
    processed = demo / "identified/A.csv"

    _, diagnostic, prediction = run_a_only(processed, tmp_path / "a_only")
    fit = diagnostic["training_fit"]
    assert fit["base_rank"] == pipeline["base_parameter_rank"]
    assert fit["effective_condition"] == pytest.approx(
        pipeline["effective_condition_number"], rel=1e-10, abs=1e-12
    )

    rows = np.genfromtxt(prediction, delimiter=",", names=True)
    included = rows["included"] > 0.5
    rmse = float(np.sqrt(np.mean(rows["residual"][included] ** 2)))
    assert rmse == pytest.approx(
        pipeline["training_error"]["aggregate_rmse"], rel=1e-10, abs=1e-12
    )


def test_tracking_diagnostic_uses_processed_measured_state_grid(tmp_path):
    demo = tmp_path / "demo"
    generate(demo, BUILD)
    processed_path = tmp_path / "A.processed.csv"
    preprocess(demo / "A.raw.csv", processed_path)
    relation = tracking_diagnostics(demo / "A.raw.csv", load_processed(processed_path))
    assert "deduplicated/resampled measured-state" in relation["sampling_semantics"]
    assert len(relation["per_joint"]) == 6
    for joint in relation["per_joint"]:
        assert joint["rmse_rad"] >= 0.0
        assert set(joint["correlation_with_measured_signals"]) == {
            "q", "qd", "qdd_est", "effort_filtered"
        }


def test_a_only_diagnostic_is_deterministic(tmp_path):
    demo = tmp_path / "demo"
    generate(demo, BUILD)
    processed = tmp_path / "A.processed.csv"
    preprocess(demo / "A.raw.csv", processed)

    _, first, first_prediction = run_a_only(processed, tmp_path / "first")
    _, second, second_prediction = run_a_only(processed, tmp_path / "second")
    first["training_fit"].pop("prediction_file", None)
    second["training_fit"].pop("prediction_file", None)
    assert first == second
    assert first_prediction.read_bytes() == second_prediction.read_bytes()


def test_offline_audit_sources_do_not_import_or_name_hardware_client():
    sources = [
        ROOT / "scripts/audit_rebot_hardware_identification_A.py",
        ROOT / "src/identification/src/rebot_real_a_training_diagnostic.cpp",
    ]
    combined = "\n".join(path.read_text() for path in sources)
    assert "ArmClient" not in combined
    assert "wlsea_arm_sdk" not in combined
    assert "192.168.50.24" not in combined
