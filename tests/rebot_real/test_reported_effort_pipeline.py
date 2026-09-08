"""Integration requires the built identify and offline RNEA fixture targets."""
from pathlib import Path
import csv
import shutil
import subprocess
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from generate_rebot_identification_demo import generate
from run_rebot_identification import run_pipeline

BUILD = ROOT / 'build_rebot'
pytestmark = pytest.mark.skipif(not (BUILD / 'rebot_reported_effort_fixture').is_file() or not (BUILD / 'identify').is_file(),
                                reason='build identify and rebot_reported_effort_fixture first')


@pytest.fixture(scope='module')
def demo(tmp_path_factory):
    directory = tmp_path_factory.mktemp('reported_effort') / 'demo'
    generate(directory, BUILD)
    report = run_pipeline(directory / 'pipeline.yaml')
    return directory, report


def test_rnea_to_processed_to_independent_prediction(demo):
    directory, report = demo
    assert report['run_status'] == 'completed'
    assert report['raw_backend'] == 'synthetic_rnea_fixture'
    assert report['full_parameter_count'] == 78
    assert report['base_parameter_rank'] > 30
    assert report['validation_error']['aggregate_rmse'] < .02
    assert report['torque_calibrated'] is False
    assert 'theta_true' not in report and 'beta_true' not in report
    assert 'beta_relative_error' not in report and 'oracle_model_error' not in report
    assert len(report['parameter_names']) == 78
    for split in ('training', 'validation'):
        assert all(m['count'] > 100 for m in report[f'{split}_per_joint'])
        assert (directory / 'identified' / f'{split}.png').stat().st_size > 10000
    assert report['preprocessing']['training']['excluded_counts']['duplicate_feedback'] > 0
    # Reconstruct predictions solely from saved columns/coordinates, not model truth.
    scales = np.asarray(report['column_scales'])
    directions = np.asarray(report['base_directions'])
    beta = np.asarray(report['beta_hat'])
    np.testing.assert_allclose((directions @ beta) / scales, report['full_minimum_norm_parameters'])
    assert report['preprocessing']['training']['resample_rate_hz'] == report['preprocessing']['validation']['resample_rate_hz']


def test_b_does_not_change_a_basis_or_parameters(demo, tmp_path):
    directory, previous = demo
    raw_b = tmp_path / 'changed_B.csv'
    with (directory / 'B.raw.csv').open() as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    for row in rows:
        for j in range(6): row[f'effort_reported{j}'] = str(float(row[f'effort_reported{j}']) + .4)
    with raw_b.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    shutil.copyfile(directory / 'B.raw.meta.yaml', raw_b.with_suffix('.meta.yaml'))
    config = yaml.safe_load((directory / 'pipeline.yaml').read_text())
    config.update(validation_raw_csv=str(raw_b), output_directory=str(tmp_path / 'changed_fit'))
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    changed = run_pipeline(path)
    for key in ('beta_hat', 'column_scales', 'base_directions', 'base_parameter_rank'):
        assert changed[key] == previous[key]
    assert changed['validation_error']['aggregate_rmse'] > .35


@pytest.mark.parametrize('case', ['same_path','same_bytes','same_trajectory','mapping','backend'])
def test_independence_and_metadata_rejected(demo, tmp_path, case):
    directory, _ = demo
    config = yaml.safe_load((directory / 'pipeline.yaml').read_text())
    raw = tmp_path / 'B.csv'
    source = directory / ('A.raw.csv' if case == 'same_bytes' else 'B.raw.csv')
    shutil.copyfile(source, raw)
    meta = yaml.safe_load(source.with_suffix('.meta.yaml').read_text())
    if case == 'same_trajectory':
        meta['trajectory_hash'] = yaml.safe_load((directory / 'A.raw.meta.yaml').read_text())['trajectory_hash']
    if case == 'mapping': meta['joint_offset_rad'][0] = 3.14
    if case == 'backend': meta['backend'] = 'rebot_sdk'
    raw.with_suffix('.meta.yaml').write_text(yaml.safe_dump(meta))
    config.update(validation_raw_csv=str(directory / 'A.raw.csv' if case == 'same_path' else raw),
                  output_directory=str(tmp_path / 'rejected'))
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError): run_pipeline(path)
    assert not (tmp_path / 'rejected').exists()


@pytest.mark.parametrize('mutation,message', [
    ({'joint_frictionloss':[.1]*6}, 'simulation truth'),
    ({'friction_observation_mode':'saturated_sliding'},'simulation truth'),
    ({'friction_velocity_threshold':100},'valid moving observations'),
    ({'data_mode':'typo'},'unknown data_mode')])
def test_cpp_real_mode_refuses_invalid_semantics(demo, tmp_path, mutation, message):
    directory, _ = demo
    config = yaml.safe_load((directory / 'identified/identify.yaml').read_text())
    config.update(mutation, output_file=str(tmp_path / 'rejected.yaml'))
    path = tmp_path / 'identify.yaml'
    array = config.pop('joint_frictionloss', None)
    text = yaml.safe_dump(config)
    if array is not None: text += f'joint_frictionloss: {array}\n'
    path.write_text(text)
    run = subprocess.run([str(BUILD / 'identify'), '--config', str(path)], capture_output=True, text=True)
    assert run.returncode != 0 and message in run.stderr
    assert not (tmp_path / 'rejected.yaml').exists()


def test_saved_mapping_reproduces_cpp_predictions(demo, tmp_path):
    # Save/load determinism through the actual C++ solver path; same preprocessed A/B
    # must reproduce predictions byte for byte in a distinct output directory.
    directory, original = demo
    config = yaml.safe_load((directory / 'identified/identify.yaml').read_text())
    config['output_file'] = str(tmp_path / 'repeat.yaml')
    path = tmp_path / 'identify.yaml'
    path.write_text(yaml.safe_dump(config))
    subprocess.run([str(BUILD/'identify'),'--config',str(path)], check=True, capture_output=True)
    repeated = yaml.safe_load((tmp_path / 'repeat.yaml').read_text())
    assert repeated['beta_hat'] == original['beta_hat']
    assert (tmp_path / 'repeat.prediction.csv').read_bytes() == (directory/'identified/result.prediction.csv').read_bytes()
