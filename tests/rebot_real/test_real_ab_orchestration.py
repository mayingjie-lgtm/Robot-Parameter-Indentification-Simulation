from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timedelta
from pathlib import Path
import shutil
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import run_rebot_real_ab as real_ab

PRODUCTION_CONFIG = ROOT / "config" / "rebot_real_ab.yaml"


def write_config(tmp_path: Path, mutate=None) -> Path:
    config = yaml.safe_load(PRODUCTION_CONFIG.read_text())
    config["experiment"]["output_root"] = str(tmp_path / "runs")
    if mutate is not None:
        mutate(config)
    path = tmp_path / "rebot_real_ab.yaml"
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    return path


def fake_success(label: str, hardware: dict, mock: bool) -> dict:
    raw = Path(hardware["output_csv"])
    raw.parent.mkdir(parents=True, exist_ok=True)
    raw.write_text("mock\n")
    raw.with_suffix(".meta.yaml").write_text("schema_version: test\n")
    return {
        "motion_status": "completed",
        "observed_sample_count": 1,
        "trajectory_hash": hardware["trajectory_hash"],
        "shutdown": {
            "cleanup_errors": [],
            "disable_status": "completed",
            "close_status": "completed",
            "exit_servo_status": "completed",
        },
        "runtime_servo_envelope": {
            "trajectory_time_end_s": hardware["duration_s"],
            "feedback_runtime": {"measurement_unusable_sample_count": 0},
        },
        "dispatch_timing": {
            "catch_up_burst_count": 0,
            "command_dispatch_timestamp_mismatch_count": 0,
            "effective_command_rate_hz": hardware["control_rate_hz"],
        },
        "feedback_cadence": {
            "timestamp_lower_update_cadence_on_host": {"effective_rate_hz": 10.0}
        },
        "tracking_quality": {"status": "within_warning_thresholds", "overall_max_abs_rad": 0.001},
    }


def fake_identification(config_path: Path) -> dict:
    config = yaml.safe_load(config_path.read_text())
    output = Path(config["output_directory"])
    output.mkdir(parents=True, exist_ok=False)
    (output / "result.yaml").write_text("run_status: completed\n")
    (output / "validation.png").write_bytes(b"png")
    return {
        "base_parameter_rank": 12,
        "full_parameter_count": 78,
        "training_error": {"aggregate_rmse": 0.1},
        "validation_error": {"aggregate_rmse": 0.2},
        "result_interpretation": "test reported-effort fit",
    }


def test_total_config_loads() -> None:
    config = real_ab.load_config(PRODUCTION_CONFIG)
    assert config["experiment"]["control_rate_hz"] == 100.0
    assert config["authorization"]["allow_hardware"] is True
    assert config["movej"]["controlled_park_before_disable"] is True


def test_unknown_top_level_key_fails(tmp_path: Path) -> None:
    path = write_config(tmp_path, lambda c: c.update(typo=True))
    with pytest.raises(ValueError, match="unknown top-level"):
        real_ab.load_config(path)


def test_unknown_nested_key_fails(tmp_path: Path) -> None:
    path = write_config(tmp_path, lambda c: c["connection"].update(typo=1))
    with pytest.raises(ValueError, match="unknown connection"):
        real_ab.load_config(path)


def test_missing_sdk_fails_preflight(tmp_path: Path) -> None:
    path = write_config(tmp_path, lambda c: c["connection"].update(sdk_root=str(tmp_path / "missing-sdk")))
    with pytest.raises(FileNotFoundError, match="SDK root"):
        real_ab.preflight(real_ab.load_config(path))


@pytest.mark.parametrize("label", ["A", "B"])
def test_missing_artifact_fails_preflight(tmp_path: Path, label: str) -> None:
    path = write_config(tmp_path, lambda c: c["trajectories"][label].update(artifact=str(tmp_path / f"missing-{label}.csv")))
    with pytest.raises(FileNotFoundError, match="trajectory artifact"):
        real_ab.preflight(real_ab.load_config(path))


def test_same_a_b_hash_fails_preflight(tmp_path: Path) -> None:
    def mutate(config):
        config["trajectories"]["B"] = deepcopy(config["trajectories"]["A"])
    path = write_config(tmp_path, mutate)
    with pytest.raises(ValueError, match="hashes must be different"):
        real_ab.preflight(real_ab.load_config(path))


def test_preview_not_accepted_fails_preflight(tmp_path: Path) -> None:
    acceptance = yaml.safe_load((ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A/preview_acceptance.yaml").read_text())
    acceptance["accepted_for_hardware"] = False
    path_accept = tmp_path / "acceptance.yaml"
    path_accept.write_text(yaml.safe_dump(acceptance, sort_keys=False))
    path = write_config(tmp_path, lambda c: c["trajectories"]["A"].update(preview_acceptance=str(path_accept)))
    with pytest.raises(PermissionError, match="accepted_for_hardware"):
        real_ab.preflight(real_ab.load_config(path))


def test_artifact_hash_mismatch_fails_preflight(tmp_path: Path) -> None:
    source = ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A/trajectory.csv"
    metadata = ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A/trajectory.meta.yaml"
    artifact_copy = tmp_path / "trajectory.csv"
    metadata_copy = tmp_path / "trajectory.meta.yaml"
    shutil.copyfile(source, artifact_copy)
    shutil.copyfile(metadata, metadata_copy)
    artifact_copy.write_bytes(artifact_copy.read_bytes() + b"\n")
    def mutate(config):
        config["trajectories"]["A"]["artifact"] = str(artifact_copy)
        config["trajectories"]["A"]["metadata"] = str(metadata_copy)
    path = write_config(tmp_path, mutate)
    with pytest.raises(ValueError, match="hash does not match"):
        real_ab.preflight(real_ab.load_config(path))


def test_unique_run_directories_and_existing_not_overwritten(tmp_path: Path) -> None:
    first_time = datetime(2026, 9, 10, 20, 0, 0, 1)
    first = real_ab._new_run_directory(tmp_path, "test", first_time)
    second = real_ab._new_run_directory(tmp_path, "test", first_time + timedelta(microseconds=1))
    assert first != second and first.is_dir() and second.is_dir()
    with pytest.raises(FileExistsError):
        real_ab._new_run_directory(tmp_path, "test", first_time)


def test_materialize_a_b_share_site_parameters_and_keep_distinct_trajectories() -> None:
    config = real_ab.load_config(PRODUCTION_CONFIG)
    checked = real_ab.preflight(config)
    a = real_ab.materialize_hardware_config(config, "A", checked["artifacts"]["A"], ROOT / "data/A-test.csv")
    b = real_ab.materialize_hardware_config(config, "B", checked["artifacts"]["B"], ROOT / "data/B-test.csv")
    for key in ("sdk_root", "host", "tcp_port", "udp_port", "joint_direction", "joint_offset_rad", "maximum_servo_target_delta_rad", "motion_ready_feedback_max_age_ms"):
        assert a[key] == b[key]
    assert a["trajectory_hash"] != b["trajectory_hash"]
    assert a["trajectory_artifact"] != b["trajectory_artifact"]
    assert a["output_csv"] != b["output_csv"]

def test_output_path_is_derived_not_user_configured(tmp_path: Path) -> None:
    config = real_ab.load_config(write_config(tmp_path))
    checked = real_ab.preflight(config)
    output = tmp_path / "run" / "A" / "raw.csv"
    hardware = real_ab.materialize_hardware_config(config, "A", checked["artifacts"]["A"], output)
    assert hardware["output_csv"] == str(output.resolve())
    assert "output_csv" not in config["trajectories"]["A"]


def test_duration_hash_and_replay_mode_are_derived() -> None:
    config = real_ab.load_config(PRODUCTION_CONFIG)
    checked = real_ab.preflight(config)
    artifact = checked["artifacts"]["A"]
    hardware = real_ab.materialize_hardware_config(config, "A", artifact, ROOT / "data/A-test.csv")
    assert hardware["duration_s"] == artifact.duration_s
    assert hardware["trajectory_hash"] == artifact.sha256
    assert hardware["trajectory_replay_mode"] == real_ab.TRAJECTORY_REPLAY_MODE
    assert hardware["max_samples"] is None


def test_a_failure_stops_before_b_and_identification(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    calls = []
    identified = []
    def fail_a(label, hardware, mock):
        calls.append(label)
        raise RuntimeError("A injected failure")
    def identify(config_path):
        identified.append(config_path)
        return {}
    with pytest.raises(RuntimeError, match="A injected failure"):
        real_ab.run_experiment(path, mock=True, trajectory_executor=fail_a, identification_executor=identify)
    assert calls == ["A"]
    assert identified == []
    summaries = list((tmp_path / "runs").glob("*/summary.yaml"))
    assert len(summaries) == 1
    assert yaml.safe_load(summaries[0].read_text())["experiment_status"] == "FAIL"


def test_b_failure_stops_before_identification(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    calls = []
    identified = []
    def execute(label, hardware, mock):
        calls.append(label)
        if label == "B":
            raise RuntimeError("B injected failure")
        return fake_success(label, hardware, mock)
    def identify(config_path):
        identified.append(config_path)
        return {}
    with pytest.raises(RuntimeError, match="B injected failure"):
        real_ab.run_experiment(path, mock=True, trajectory_executor=execute, identification_executor=identify)
    assert calls == ["A", "B"]
    assert identified == []


def test_preflight_only_never_calls_trajectory_or_identification(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError("executor must not be called")
    result = real_ab.run_experiment(path, preflight_only=True, trajectory_executor=forbidden, identification_executor=forbidden)
    assert result["experiment_status"] == "PREFLIGHT_PASS"
    assert result["hardware_contacted"] is False
    assert not (tmp_path / "runs").exists()


def test_mock_with_injected_executors_marks_no_hardware_contact(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    result = real_ab.run_experiment(path, mock=True, trajectory_executor=fake_success, identification_executor=fake_identification)
    assert result["experiment_status"] == "PASS"
    assert result["hardware_contacted"] is False
    assert set(result["trajectories"]) == {"A", "B"}


def test_run_materializes_hardware_configs_and_identification_config(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    result = real_ab.run_experiment(path, mock=True, trajectory_executor=fake_success, identification_executor=fake_identification)
    run_dir = Path(result["run_directory"])
    for label in ("A", "B"):
        hardware_path = run_dir / label / "hardware.yaml"
        assert hardware_path.is_file()
        hardware = yaml.safe_load(hardware_path.read_text())
        assert hardware["output_csv"] == str((run_dir / label / "raw.csv").resolve())
        assert hardware["trajectory_hash"] == result["trajectories"][label]["trajectory_sha256"]
    identification = yaml.safe_load((run_dir / "identification/config.yaml").read_text())
    assert identification["training_raw_csv"] == str((run_dir / "A/raw.csv").resolve())
    assert identification["validation_raw_csv"] == str((run_dir / "B/raw.csv").resolve())


def test_snapshot_and_provenance_are_saved(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    result = real_ab.run_experiment(path, mock=True, trajectory_executor=fake_success, identification_executor=fake_identification)
    run_dir = Path(result["run_directory"])
    assert (run_dir / "experiment_config.snapshot.yaml").read_bytes() == path.read_bytes()
    provenance = yaml.safe_load((run_dir / "provenance.yaml").read_text())
    assert len(provenance["git_commit"]) == 40
    assert provenance["files"]["trajectory_A"]["artifact_sha256"] != provenance["files"]["trajectory_B"]["artifact_sha256"]


def test_summary_yaml_contains_operator_facing_metrics(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    result = real_ab.run_experiment(path, mock=True, trajectory_executor=fake_success, identification_executor=fake_identification)
    summary = yaml.safe_load((Path(result["run_directory"]) / "summary.yaml").read_text())
    assert summary["experiment_status"] == "PASS"
    assert summary["trajectories"]["A"]["dispatch_rate_hz"] == 100.0
    assert summary["trajectories"]["A"]["lower_feedback_update_rate_hz"] == 10.0
    assert summary["identification"]["base_parameter_rank"] == 12
    assert summary["identification"]["validation_reported_effort_rmse"] == 0.2
    assert summary["identification"]["physical_torque_calibration"] == "unresolved"


def test_authorization_boolean_type_fails(tmp_path: Path) -> None:
    path = write_config(tmp_path, lambda c: c["authorization"].update(allow_hardware="true"))
    with pytest.raises(ValueError, match="YAML boolean"):
        real_ab.load_config(path)


def test_invalid_position_limits_fail(tmp_path: Path) -> None:
    def mutate(config):
        config["safety"]["joint_position_min_rad"][0] = 1.0
        config["safety"]["joint_position_max_rad"][0] = 0.0
    path = write_config(tmp_path, mutate)
    with pytest.raises(ValueError, match="minimum must be below maximum"):
        real_ab.load_config(path)


def test_controlled_park_cannot_be_silently_disabled(tmp_path: Path) -> None:
    path = write_config(tmp_path, lambda c: c["movej"].update(controlled_park_before_disable=False))
    with pytest.raises(ValueError, match="must remain true"):
        real_ab.load_config(path)


def test_real_mode_has_two_in_process_confirmation_gates(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    prompts = []
    def confirm(prompt):
        prompts.append(prompt)
        return ""
    result = real_ab.run_experiment(path, mock=False, trajectory_executor=fake_success, identification_executor=fake_identification, confirmation_fn=confirm)
    assert result["experiment_status"] == "PASS"
    assert len(prompts) == 2
    assert "trajectory A" in prompts[0]
    assert "trajectory B" in prompts[1]


def test_mock_default_path_never_loads_real_arm_client(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    import rebot_real.control_adapter as adapter
    path = write_config(tmp_path)
    def forbidden_loader(*args, **kwargs):
        raise AssertionError("real ArmClient loader must not be called in --mock")
    monkeypatch.setattr(adapter, "_load_arm_client_class", forbidden_loader)
    result = real_ab.run_experiment(path, mock=True, identification_executor=fake_identification)
    assert result["experiment_status"] == "PASS"
    assert result["hardware_contacted"] is False
    assert result["trajectories"]["A"]["sample_count"] == 3001
    assert result["trajectories"]["B"]["sample_count"] == 3001


def test_full_mock_falls_back_to_offline_fixture_when_mock_effort_is_not_fit_grade(tmp_path: Path) -> None:
    path = write_config(tmp_path)
    result = real_ab.run_experiment(path, mock=True)
    assert result["experiment_status"] == "PASS"
    assert result["hardware_contacted"] is False
    assert result["mock_reported_effort_identification"]["status"] == "not_meaningful"
    assert result["identification"]["status"] == "fixture_pass_mock_reported_effort_not_fit"
    assert result["identification"]["base_parameter_rank"] == 52
    assert Path(result["identification"]["validation_png"]).is_file()
