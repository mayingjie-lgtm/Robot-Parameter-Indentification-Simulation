from __future__ import annotations

import csv
from pathlib import Path
import sys

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from rebot_real.control_adapter import RebotControlError
from rebot_real.joint_jog import jog_target_at, validate_joint_jog
from rebot_real.mock_client import MockArmClient
from rebot_real.runner import RebotHardwareRunner, load_hardware_config


class Clock:
    def __init__(self):
        self.now = 1.0

    def sleep(self, seconds):
        self.now += seconds


@pytest.fixture
def config(tmp_path):
    return load_hardware_config(
        REPO_ROOT / "config/rebot_joint_jog_mock.yaml", repo_root=REPO_ROOT,
        overrides={"output_csv": str(tmp_path / "jog.csv")},
    )


def runner(config, fake, clock=None, *, real=False):
    clock = clock or Clock()
    return RebotHardwareRunner(
        config, repo_root=REPO_ROOT, mock_backend=not real,
        client_factory=lambda **kwargs: fake, sleep_fn=clock.sleep,
        monotonic_fn=lambda: clock.now,
    )


@pytest.mark.parametrize("joint,sign,direction", [(1, 1, 1), (3, -1, 1), (6, 1, -1)])
def test_round_trip_and_evidence(config, joint, sign, direction):
    config["joint_jog"]["joint"] = joint
    config["joint_jog"]["displacement_rad"] *= sign
    config["joint_direction"][joint - 1] = direction
    config["joint_offset_rad"][joint - 1] = 0.2
    fake = MockArmClient(follow_servo_targets=True)
    metadata = runner(config, fake).run()
    targets = fake.servo_targets
    for q in targets:
        assert all(q[i] == 0 for i in range(6) if i != joint - 1)
    assert targets[0] == pytest.approx([0] * 6)
    assert targets[-1] == pytest.approx([0] * 6)
    assert max(abs(q[joint - 1]) for q in targets) == pytest.approx(abs(config["joint_jog"]["displacement_rad"]))
    assert any(q[joint - 1] * sign * direction > 0 for q in targets)
    result = metadata["joint_jog_result"]
    assert result["status"] == "completed"
    assert result["identification_ready"] is False
    assert result["physical_mapping_verified_by_test"] is False
    assert result["measured_displacement_rad"][joint - 1] == pytest.approx(config["joint_jog"]["displacement_rad"])
    assert result["return_error_rad"] == pytest.approx([0] * 6, abs=1e-14)
    assert fake.calls[-3:] == ["exit_servo", "disable", "close"]
    with Path(config["output_csv"]).open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2 * len(targets)
    assert rows[-1]["command_valid"] == "0"
    assert rows[-1]["q_cmd0"] == "nan"
    for phase in ("baseline", "endpoint", "returned"):
        assert result["stationary_windows"][phase]["sample_count"] >= 49


@pytest.mark.parametrize("key,value,error", [
    ("allow_motion", False, "allow_motion"),
    ("joint_mapping_verified", False, "joint_mapping_verified"),
    ("joint_mapping_scope", "servo_hold_only", "hold-only"),
    ("j1_convention", "UNRESOLVED", "UNRESOLVED"),
    ("j1_convention", "PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907", "smoke only"),
    ("max_samples", 5, "max_samples"),
    ("duration_s", 1.0, "duration_s"),
])
def test_gates_before_connect(config, key, value, error):
    config[key] = value
    fake = MockArmClient()
    with pytest.raises((ValueError, PermissionError), match=error):
        runner(config, fake).run()
    assert fake.calls == []


def test_mock_config_cannot_connect_hardware(config):
    fake = MockArmClient()
    with pytest.raises(PermissionError, match="allow_hardware"):
        runner(config, fake, real=True).run()
    config["allow_hardware"] = True
    with pytest.raises(PermissionError, match="Mock mapping"):
        runner(config, fake, real=True).run()
    assert fake.calls == []


@pytest.mark.parametrize("key,value", [
    ("joint", True), ("joint", 7), ("joint", 1.5),
    ("displacement_rad", 0), ("displacement_rad", float("nan")),
    ("authorization_reference", ""), ("ramp_duration_s", 0.1),
    ("maximum_acceleration_rad_s2", 1e-6),
    ("maximum_command_gap_s", 0.001),
    ("settle_position_tolerance_rad", 1.0),
])
def test_invalid_plan_rejected_before_connect(config, key, value):
    config["joint_jog"][key] = value
    fake = MockArmClient()
    with pytest.raises(ValueError):
        runner(config, fake).run()
    assert fake.calls == []


def test_plan_smoothness(config):
    jog = validate_joint_jog(config)
    period = 1 / config["control_rate_hz"]
    targets = [jog_target_at(jog, (0,) * 6, i * period)[1][0] for i in range(701)]
    velocity = [(b - a) / period for a, b in zip(targets, targets[1:])]
    acceleration = [(b - a) / period for a, b in zip(velocity, velocity[1:])]
    assert max(map(abs, velocity)) < config["maximum_command_velocity_rad_s"][0]
    assert max(map(abs, acceleration)) < jog["maximum_acceleration_rad_s2"]


def test_envelope_checked_before_enable(config):
    fake = MockArmClient(position_rad=[0.989, 0, 0, 0, 0, 0])
    with pytest.raises(RebotControlError, match="envelope"):
        runner(config, fake).run()
    assert "enable" not in fake.calls
    assert fake.calls[-1] == "close"


@pytest.mark.parametrize("failure", ["drift", "no_follow", "fault", "stale", "invalid", "interrupt", "gap", "disconnect"])
def test_active_failure_aborts_and_preserves_evidence(config, failure):
    clock = Clock()

    class Faulty(MockArmClient):
        def servo_joint(self, *args, **kwargs):
            reply = super().servo_joint(*args, **kwargs)
            if len(self.servo_targets) == 150:
                if failure == "drift":
                    self.position_rad[1] += 0.0003
                elif failure == "fault":
                    self.primary_fault_code = 500119
                elif failure == "stale":
                    self.feedback_age_ms = [100] * 6
                elif failure == "invalid":
                    self.feedback_valid = [False] * 6
                elif failure == "interrupt":
                    raise KeyboardInterrupt()
                elif failure == "gap":
                    clock.now += 0.1
                elif failure == "disconnect":
                    self.disconnect_after_state_reads = 0
            return reply

    fake = Faulty(follow_servo_targets=failure != "no_follow")
    with pytest.raises((RebotControlError, KeyboardInterrupt)):
        runner(config, fake, clock).run()
    assert fake.calls[-4:] == ["exit_servo", "stop", "disable", "close"]
    result = yaml.safe_load(Path(config["output_csv"]).with_suffix(".meta.yaml").read_text())["joint_jog_result"]
    assert result["status"] == "aborted"
    assert result["identification_ready"] is False
    assert "returned" not in result["stationary_windows"]


@pytest.mark.parametrize("operation", ["enable", "enter_servo", "exit_servo"])
def test_uncertain_ack_and_cleanup_failure_still_disable(config, operation):
    fake = MockArmClient(fail_on={operation}, follow_servo_targets=True)
    with pytest.raises(RebotControlError):
        runner(config, fake).run()
    assert fake.calls[-2:] == ["disable", "close"]
    result = yaml.safe_load(Path(config["output_csv"]).with_suffix(".meta.yaml").read_text())["joint_jog_result"]
    assert result["status"] == "aborted"


def test_existing_evidence_not_overwritten(config):
    Path(config["output_csv"]).write_text("keep me")
    fake = MockArmClient()
    with pytest.raises(FileExistsError):
        runner(config, fake).run()
    assert fake.calls == []
    assert Path(config["output_csv"]).read_text() == "keep me"


def test_irregular_command_intervals_follow_smooth_elapsed_time_path(config):
    class JitterClock(Clock):
        def __init__(self):
            super().__init__()
            self.cycles = 0

        def sleep(self, seconds):
            self.cycles += 1
            # Fixed-index targets produced artificial accelerations under this jitter.
            self.now += seconds + (0.012 if self.cycles % 3 == 0 else 0.0)

    fake = MockArmClient(follow_servo_targets=True)
    result = runner(config, fake, JitterClock()).run()["joint_jog_result"]
    assert result["status"] == "completed"
    assert result["return_error_rad"] == pytest.approx([0] * 6)


def test_enabled_robot_rejected_without_claiming_control(config):
    fake = MockArmClient()
    fake.robot_mode = "idle"
    with pytest.raises(RebotControlError, match="initially disabled"):
        runner(config, fake).run()
    assert fake.calls == ["connect", "read_state", "close"]


def test_envelope_rechecked_after_enable(config):
    class MovesDuringEnable(MockArmClient):
        def enable(self, **kwargs):
            reply = super().enable(**kwargs)
            self.position_rad[0] = 0.989
            return reply

    fake = MovesDuringEnable()
    with pytest.raises(RebotControlError, match="envelope"):
        runner(config, fake).run()
    assert "enter_servo" not in fake.calls
    assert fake.calls[-2:] == ["disable", "close"]


@pytest.mark.parametrize("operation", ["disable", "close"])
def test_shutdown_failure_is_not_marked_completed(config, operation):
    fake = MockArmClient(follow_servo_targets=True, fail_on={operation})
    with pytest.raises(RebotControlError, match="cleanup failures"):
        runner(config, fake).run()
    result = yaml.safe_load(Path(config["output_csv"]).with_suffix(".meta.yaml").read_text())["joint_jog_result"]
    assert result["status"] == "aborted"
    assert result["cleanup_errors"]


def test_timeout_does_not_report_partial_trip_as_completed(config):
    # Valid nominal budget, but insufficient for the final feedback observation.
    config["duration_s"] = 7.001
    instance = runner(config, MockArmClient(follow_servo_targets=True))
    with pytest.raises(RebotControlError, match="timeout"):
        instance.run()
    result = yaml.safe_load(Path(config["output_csv"]).with_suffix(".meta.yaml").read_text())["joint_jog_result"]
    assert result["status"] == "aborted"
