from pathlib import Path
import csv
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from rebot_real.preprocessing import preprocess
from generate_rebot_identification_demo import convert_truth


def write_signal(tmp_path, *, duration=8.0, rate=100.0):
    truth = tmp_path / "truth.csv"
    t = np.arange(int(duration * rate)+1)/rate
    w = 2*np.pi*.3
    with truth.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time", *[f"{p}{j}" for p in ("q", "qd", "qdd", "tau") for j in range(6)]])
        for time in t:
            writer.writerow([time, *([np.sin(w*time)]*6), *([w*np.cos(w*time)]*6),
                             *([-w*w*np.sin(w*time)]*6), *([2*np.sin(w*time)]*6)])
    raw = tmp_path / "raw.csv"
    convert_truth(truth, raw, "analytic")
    return raw


def edit_rows(raw, fn):
    with raw.open() as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames
        rows = list(reader)
    fn(rows)
    with raw.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def test_analytic_derivative_and_duplicate_feedback(tmp_path):
    raw = write_signal(tmp_path)
    report = preprocess(raw, tmp_path / "processed.csv")
    data = np.genfromtxt(tmp_path / "processed.csv", delimiter=",", names=True)
    # Relative to first host timestamp; injected host jitter is <= 0.2 ms.
    expected = -(2*np.pi*.3)**2*np.sin(2*np.pi*.3*data['time'])
    assert np.sqrt(np.mean((data['qdd_est0']-expected)**2)) < .02
    assert report['excluded_counts']['duplicate_feedback'] > 0
    assert report['effective_feedback_rate_hz'] < 101
    assert report['fresh_valid_sample_count'] == 801
    assert report['torque_calibrated'] is False
    assert max(report['qd_consistency_rmse_rad_s']) < .01
    with pytest.raises(FileExistsError):
        preprocess(raw, tmp_path / "processed.csv")


@pytest.mark.parametrize("schema", [
    "rebot_hardware_experiment_v1",
    "rebot_hardware_experiment_v2",
    "rebot_hardware_experiment_v3",
])
def test_raw_schema_versions_remain_preprocessing_compatible(tmp_path, schema):
    raw = write_signal(tmp_path)
    metadata_path = raw.with_suffix(".meta.yaml")
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["schema_version"] = schema
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    report = preprocess(raw, tmp_path / f"processed_{schema}.csv")
    assert report["raw_metadata"]["schema_version"] == schema
    assert report["qdd_source"] == "derivative_of_zero_phase_filtered_sdk_qd"


@pytest.mark.parametrize('column', ['timestamp_host_rx_ns', 'timestamp_lower_ns'])
def test_timestamp_rollback_rejected(tmp_path, column):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: rows[30].update({column: str(int(rows[29][column])-1)}))
    with pytest.raises(ValueError, match='backwards'):
        preprocess(raw, tmp_path / 'output.csv')


@pytest.mark.parametrize('field,value,reason', [
    ('q0','nan','nonfinite_signal'), ('torque_valid1','0','invalid_feedback_or_torque'),
    ('feedback_age_ms2','80','stale_or_invalid_age'), ('primary_fault_code','4','fault_or_unsafe_state')])
def test_bad_measurement_splits_segments(tmp_path, field, value, reason):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: rows[len(rows)//2+4].update({field: value}))
    report = preprocess(raw, tmp_path / 'output.csv')
    data = np.genfromtxt(tmp_path / 'output.csv', delimiter=',', names=True)
    assert report['segment_count'] == 2
    assert report['excluded_counts'][reason] == 1
    assert np.max(np.diff(data['time'])) > 0.9


def test_long_gap_not_interpolated(tmp_path):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: rows.__delitem__(slice(300, 420)))
    report = preprocess(raw, tmp_path / 'output.csv')
    data = np.genfromtxt(tmp_path / 'output.csv', delimiter=',', names=True)
    assert report['segment_count'] == 2
    assert np.max(np.diff(data['time'])) > 1.5


def test_changed_duplicate_group_excluded(tmp_path):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: rows[1].update(q0='42'))
    report = preprocess(raw, tmp_path / 'output.csv')
    assert report['excluded_counts']['inconsistent_repeated_feedback'] == 2


def test_v3_uses_lower_feedback_timeout_not_deprecated_motion_ready_age(tmp_path):
    raw = write_signal(tmp_path)
    metadata_path = raw.with_suffix(".meta.yaml")
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["schema_version"] = "rebot_hardware_experiment_v3"
    metadata["lower_feedback_timeout_ms"] = 250.0
    metadata["maximum_feedback_age_ms"] = 50.0
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    edit_rows(raw, lambda rows: rows[200].update(feedback_age_ms0="100.0"))
    report = preprocess(raw, tmp_path / "output.csv")
    assert report["feedback_age_limit_source"] == "lower_feedback_timeout_ms"
    assert report["feedback_age_limit_ms"] == 250.0
    assert report["excluded_counts"].get("stale_or_invalid_age", 0) == 0


def test_duplicate_snapshot_age_does_not_create_physical_segment_break(tmp_path):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: rows[1].update(feedback_age_ms0="1000.0"))
    report = preprocess(raw, tmp_path / "output.csv")
    assert report["segment_count"] == 1
    assert report["excluded_counts"]["duplicate_feedback"] > 0
    assert report["fresh_valid_sample_count"] == 801


def test_short_segment_rejected(tmp_path):
    raw = write_signal(tmp_path, duration=.4)
    with pytest.raises(ValueError, match='long enough'):
        preprocess(raw, tmp_path / 'output.csv')


def test_frozen_rate_rejects_insufficient_feedback(tmp_path):
    raw = write_signal(tmp_path, rate=10)
    with pytest.raises(ValueError, match='bandwidth'):
        preprocess(raw, tmp_path / 'output.csv', frozen_rate_hz=100)


def test_filter_rejects_unusable_cutoff(tmp_path):
    raw = write_signal(tmp_path, rate=3)
    with pytest.raises(ValueError, match='filter cutoff'):
        preprocess(raw, tmp_path / 'output.csv')


def test_state_only_is_not_excitation(tmp_path):
    raw = write_signal(tmp_path)
    edit_rows(raw, lambda rows: [r.update(control_mode='state_only',command_valid='0') for r in rows])
    with pytest.raises(ValueError, match='insufficient'):
        preprocess(raw, tmp_path / 'output.csv')
