from __future__ import annotations

import csv
import inspect
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.control_adapter import RebotControlError
from rebot_real.mock_client import MockArmClient
from rebot_real.runner import RebotHardwareRunner, load_hardware_config
from rebot_real.trajectory_artifact import (
    TRAJECTORY_REPLAY_MODE,
    load_replay_artifact,
    sha256_file,
    validate_preview_acceptance,
    validate_replay_runtime_limits,
)


class FakeClock:
    def __init__(self, start_ns: int = 1_000_000_000) -> None:
        self.now_ns = int(start_ns)

    def monotonic(self) -> float:
        return self.now_ns * 1e-9

    def monotonic_ns(self) -> int:
        return self.now_ns

    def sleep(self, duration_s: float) -> None:
        self.now_ns += max(0, int(math.ceil(float(duration_s) * 1e9)))

    def advance_ms(self, duration_ms: float) -> None:
        self.now_ns += int(round(duration_ms * 1e6))


class RebotHardwareRunnerTest(unittest.TestCase):
    def _config(self, directory: str, *, mode: str = "state_only", samples: int = 1) -> dict:
        config = load_hardware_config(
            REPO_ROOT / "config" / "rebot_real_experiment.yaml",
            repo_root=REPO_ROOT,
        )
        config["control_mode"] = mode
        config["duration_s"] = 100.0
        config["max_samples"] = samples
        config["state_timeout_s"] = 0.01
        config["output_csv"] = str(Path(directory) / f"{mode}.csv")
        return config

    def _servo_config(self, directory: str, *, samples: int = 1) -> dict:
        config = self._config(directory, mode="servo_hold", samples=samples)
        config["allow_motion"] = True
        config["joint_mapping_verified"] = True
        config["j1_convention"] = "CANONICAL_VERIFIED"
        return config

    def _excitation_config(self, directory: str, *, rate_hz: float = 100.0) -> dict:
        config = load_hardware_config(
            REPO_ROOT / "config" / "rebot_excitation_mock.yaml",
            repo_root=REPO_ROOT,
        )
        config["sdk_root"] = ""
        config["allow_hardware"] = False
        config["j1_convention"] = "MOCK_CANONICAL_REBOT_DM"
        config["joint_mapping_scope"] = "excitation"
        config["output_csv"] = str(Path(directory) / "excitation.csv")
        config["controlled_park_before_disable"] = True
        config["joint_position_min_rad"][0] = -math.pi
        config["joint_position_max_rad"][0] = math.pi
        if rate_hz == 100.0:
            source_root = REPO_ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A"
            config["trajectory_artifact"] = str(source_root / "trajectory.csv")
            config["trajectory_metadata"] = str(source_root / "trajectory.meta.yaml")
            config["trajectory_hash"] = None
            config["control_rate_hz"] = 100.0
        source_trajectory = Path(config["trajectory_artifact"])
        source_metadata = Path(config["trajectory_metadata"])
        trajectory = Path(directory) / "trajectory.csv"
        metadata = Path(directory) / "trajectory.meta.yaml"
        report = Path(directory) / "preview_report.yaml"
        mp4 = Path(directory) / "preview.mp4"
        acceptance = Path(directory) / "preview_acceptance.yaml"
        shutil.copyfile(source_trajectory, trajectory)
        artifact_metadata = yaml.safe_load(source_metadata.read_text())
        sample_count = int(artifact_metadata["sample_count"])
        artifact_metadata.update(
            continuous_quintic_collision_precheck="PASS",
            continuous_quintic_collision_subdivisions_per_interval=10,
            continuous_quintic_collision_precheck_sample_count=(sample_count - 1) * 10 + 1,
        )
        metadata.write_text(yaml.safe_dump(artifact_metadata, sort_keys=False))
        report.write_text(yaml.safe_dump({
            "schema_version": "rebot_trajectory_preview_report_v2",
            "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
            "trajectory_sha256": sha256_file(trajectory),
            "continuous_quintic_collision_precheck": "PASS",
            "preview_status": "PASS",
        }, sort_keys=False))
        mp4.write_bytes(b"mock actual-time preview")
        acceptance.write_text(yaml.safe_dump({
            "schema_version": "rebot_trajectory_preview_acceptance_v2",
            "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
            "trajectory_sha256": sha256_file(trajectory),
            "preview_report": str(report),
            "preview_report_sha256": sha256_file(report),
            "preview_mp4": str(mp4),
            "preview_mp4_sha256": sha256_file(mp4),
            "accepted_for_hardware": False,
        }, sort_keys=False))
        config["trajectory_replay_mode"] = TRAJECTORY_REPLAY_MODE
        config["trajectory_artifact"] = str(trajectory)
        config["trajectory_metadata"] = str(metadata)
        config["trajectory_preview_acceptance"] = str(acceptance)
        return config

    def _runner(self, config: dict, fake: MockArmClient, *, mock_backend: bool = True) -> RebotHardwareRunner:
        clock = FakeClock()
        fake._monotonic_ns_fn = clock.monotonic_ns
        return RebotHardwareRunner(
            config,
            repo_root=REPO_ROOT,
            client_factory=lambda **_: fake,
            mock_backend=mock_backend,
            sleep_fn=clock.sleep,
            monotonic_fn=clock.monotonic,
            monotonic_ns_fn=clock.monotonic_ns,
        )

    def test_config_parse_keeps_safe_defaults(self) -> None:
        config = load_hardware_config(
            REPO_ROOT / "config" / "rebot_real_experiment.yaml",
            repo_root=REPO_ROOT,
        )
        self.assertEqual(config["control_mode"], "state_only")
        self.assertFalse(config["allow_hardware"])
        self.assertFalse(config["allow_motion"])
        self.assertFalse(config["joint_mapping_verified"])
        self.assertEqual(config["j1_convention"], "UNRESOLVED")
        self.assertEqual(len(config["joint_direction"]), 6)
        self.assertEqual(len(config["joint_offset_rad"]), 6)
        self.assertEqual(
            config["maximum_disabled_feedback_age_ms"],
            config["maximum_feedback_age_ms"],
        )

    def test_non_catchup_deadline_preserves_100hz_period_after_late_dispatch(self) -> None:
        period_s = 0.010
        for elapsed_s in (0.0115, 0.014, 0.022):
            with self.subTest(elapsed_s=elapsed_s):
                previous_dispatch_start_s = 1.0 + elapsed_s
                deadline = RebotHardwareRunner._non_catchup_deadline(
                    scheduled_deadline_s=1.005,
                    previous_dispatch_start_s=previous_dispatch_start_s,
                    period_s=period_s,
                )
                self.assertAlmostEqual(
                    deadline - previous_dispatch_start_s,
                    period_s,
                )

    def test_real_backend_default_deny_hardware_creates_no_session(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient()
            with self.assertRaisesRegex(PermissionError, "allow_hardware=false"):
                self._runner(config, fake, mock_backend=False).run()
            self.assertEqual(fake.calls, [])

    def test_state_only_mock_never_calls_motor_changing_commands(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient()
            metadata = self._runner(config, fake).run()
            self.assertEqual(metadata["observed_sample_count"], 1)
            self.assertEqual(fake.calls, ["connect", "read_state", "close"])
            forbidden = {"configure_pvt", "enable", "movej", "enter_servo", "servo_joint", "stop", "disable"}
            self.assertTrue(forbidden.isdisjoint(fake.calls))

    def test_real_state_only_with_explicit_hardware_gate_still_has_no_upper_motor_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            config["allow_hardware"] = True
            fake = MockArmClient()
            metadata = self._runner(config, fake, mock_backend=False).run()
            self.assertEqual(metadata["backend"], "rebot_sdk")
            self.assertEqual(fake.calls, ["connect", "read_state", "close"])
            forbidden = {"configure_pvt", "enable", "movej", "enter_servo", "servo_joint", "stop", "disable"}
            self.assertTrue(forbidden.isdisjoint(fake.calls))

    def test_state_only_function_has_no_motor_changing_call_site(self) -> None:
        source = inspect.getsource(RebotHardwareRunner._run_state_only)
        for forbidden in (
            ".configure_movej_pvt(",
            ".enable(",
            ".movej_to(",
            ".enter_servo(",
            ".send_servo_target(",
            ".stop(",
            ".disable(",
        ):
            self.assertNotIn(forbidden, source)

    def test_allow_motion_false_blocks_servo_before_connect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["allow_hardware"] = True
            config["allow_motion"] = False
            fake = MockArmClient()
            with self.assertRaisesRegex(PermissionError, "allow_motion=false"):
                self._runner(config, fake, mock_backend=False).run()
            self.assertEqual(fake.calls, [])

    def test_unresolved_mapping_blocks_servo_before_connect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["allow_hardware"] = True
            config["joint_mapping_verified"] = False
            fake = MockArmClient()
            with self.assertRaisesRegex(PermissionError, "joint_mapping_verified=false"):
                self._runner(config, fake, mock_backend=False).run()
            self.assertEqual(fake.calls, [])

    def test_unresolved_j1_blocks_servo_before_connect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["allow_hardware"] = True
            config["j1_convention"] = "UNRESOLVED"
            fake = MockArmClient()
            with self.assertRaisesRegex(PermissionError, "j1_convention=UNRESOLVED"):
                self._runner(config, fake, mock_backend=False).run()
            self.assertEqual(fake.calls, [])

    def test_servo_hold_normal_lifecycle_uses_fresh_current_q(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            current_q = [0.3, -1.0, -1.2, 0.2, -0.3, 0.4]
            fake = MockArmClient(position_rad=current_q)
            metadata = self._runner(config, fake).run()
            self.assertEqual(
                fake.calls,
                [
                    "connect",
                    "read_state",
                    "enable",
                    "read_state",
                    "enter_servo",
                    "servo_joint",
                    "read_state",
                    "exit_servo",
                    "disable",
                    "close",
                ],
            )
            self.assertEqual(fake.servo_targets, [tuple(current_q)])
            self.assertEqual(metadata["observed_sample_count"], 1)
            with Path(config["output_csv"]).open("r", encoding="utf-8", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual([float(row[f"q_cmd{i}"]) for i in range(6)], current_q)
            self.assertEqual(row["command_valid"], "1")

    def test_servo_hold_waits_for_udp_ready_and_servo_state_transitions(self) -> None:
        class DelayedTransitionsMock(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._ready_reads = 0
                self._servo_reads = 0
                self._servo_transition_pending = False

            def enable(self, timeout_s: float = 5.0):
                reply = super().enable(timeout_s=timeout_s)
                self.safety_state = "enabling"
                self.robot_mode = "idle"
                self._ready_reads = 0
                return reply

            def enter_servo(self, timeout_s: float = 3.0):
                reply = super().enter_servo(timeout_s=timeout_s)
                self.servo_active = False
                self.servo_mode = "ready"
                self._servo_reads = 0
                self._servo_transition_pending = True
                return reply

            @property
            def state_store(self):
                if self.enabled and self.safety_state == "enabling":
                    if self._ready_reads >= 1:
                        self.safety_state = "ready"
                    self._ready_reads += 1
                elif self._servo_transition_pending:
                    if self._servo_reads >= 1:
                        self.servo_active = True
                        self._servo_transition_pending = False
                    self._servo_reads += 1
                return MockArmClient.state_store.fget(self)

        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            current_q = [0.3, -1.0, -1.2, 0.2, -0.3, 0.4]
            fake = DelayedTransitionsMock(position_rad=current_q)
            metadata = self._runner(config, fake).run()
            enter_index = fake.calls.index("enter_servo")
            servo_index = fake.calls.index("servo_joint")
            self.assertEqual(servo_index, enter_index + 1)
            self.assertGreaterEqual(fake.calls[servo_index + 1:].count("read_state"), 2)
            self.assertEqual(fake.servo_targets, [tuple(current_q)])
            self.assertEqual(metadata["observed_sample_count"], 1)

    def test_command_position_limit_rejection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient()
            runner = self._runner(config, fake)
            q_reference = [
                0.5 * (lower + upper)
                for lower, upper in zip(
                    config["joint_position_min_rad"],
                    config["joint_position_max_rad"],
                )
            ]
            q_target = list(q_reference)
            q_reference[0] = config["joint_position_max_rad"][0]
            q_target[0] = config["joint_position_max_rad"][0] + 0.1
            with self.assertRaisesRegex(RebotControlError, "outside configured position limits"):
                runner._validate_target_step(q_target, q_reference)

    def test_command_velocity_derived_delta_rejection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_command_velocity_rad_s"] = [0.05] * 6
            config["control_rate_hz"] = 100.0
            runner = self._runner(config, MockArmClient())
            with self.assertRaisesRegex(RebotControlError, "velocity-derived limit"):
                runner._validate_target_step([0.01, -1, -1, 0, 0, 0], [0.0, -1, -1, 0, 0, 0])

    def test_tracking_lag_is_not_reinterpreted_as_target_velocity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["control_rate_hz"] = 200.0
            config["maximum_command_velocity_rad_s"] = [0.5] * 6
            runner = self._runner(config, MockArmClient())
            previous = [0.0, -1.0, -1.0, 0.0, 0.0, 0.0]
            target = [0.0002, -1.0, -1.0, 0.0, 0.0, 0.0]
            measured = list(target)
            measured[3] -= 0.002512
            runner._validate_target_step(target, previous)
            runner._validate_tracking_error(target, measured)

    def test_independent_tracking_error_limit_rejects_excess_lag(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_tracking_error_rad"] = [0.003] * 6
            runner = self._runner(config, MockArmClient())
            with self.assertRaisesRegex(RebotControlError, "tracking error J4"):
                runner._validate_tracking_error(
                    [0.0, -1.0, -1.0, 0.004, 0.0, 0.0],
                    [0.0, -1.0, -1.0, 0.0, 0.0, 0.0],
                )

    def test_stale_feedback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(feedback_age_ms=[101.0] * 6)
            with self.assertRaisesRegex(RebotControlError, "feedback stale"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls[-1], "close")

    def test_servo_uses_separate_disabled_and_enabled_feedback_age_limits(self) -> None:
        class FreshAfterEnableMock(MockArmClient):
            def enable(self, timeout_s: float = 5.0):
                reply = super().enable(timeout_s=timeout_s)
                self.feedback_age_ms = (1.0,) * 6
                return reply

        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_disabled_feedback_age_ms"] = 110.0
            config["maximum_feedback_age_ms"] = 50.0
            fake = FreshAfterEnableMock(feedback_age_ms=[100.0] * 6)
            self._runner(config, fake).run()
            self.assertIn("enable", fake.calls)
            self.assertIn("servo_joint", fake.calls)

    def test_servo_waits_through_one_transitional_stale_post_enable_frame(self) -> None:
        class FreshOnSecondEnabledFrame(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._enabled_reads = 0

            @property
            def state_store(self):
                if self.enabled:
                    if self._enabled_reads >= 1:
                        self.feedback_age_ms = (1.0,) * 6
                    self._enabled_reads += 1
                return MockArmClient.state_store.fget(self)

        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_disabled_feedback_age_ms"] = 110.0
            config["maximum_feedback_age_ms"] = 50.0
            fake = FreshOnSecondEnabledFrame(feedback_age_ms=[100.0] * 6)
            self._runner(config, fake).run()
            enable_index = fake.calls.index("enable")
            servo_index = fake.calls.index("enter_servo")
            self.assertGreaterEqual(fake.calls[enable_index + 1:servo_index].count("read_state"), 2)

    def test_servo_rejects_feedback_that_never_becomes_fresh_after_enable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_disabled_feedback_age_ms"] = 110.0
            config["maximum_feedback_age_ms"] = 50.0
            config["command_timeout_s"] = 0.01
            fake = MockArmClient(feedback_age_ms=[100.0] * 6)
            with self.assertRaisesRegex(
                RebotControlError,
                "motion-ready fresh feedback",
            ):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls[0:3], ["connect", "read_state", "enable"])
            self.assertNotIn("enter_servo", fake.calls)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])

    def test_invalid_feedback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(feedback_valid=[False, True, True, True, True, True])
            with self.assertRaisesRegex(RebotControlError, "invalid joint feedback"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls[-1], "close")

    def test_nan_feedback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(position_rad=[math.nan, -1, -1, 0, 0, 0])
            with self.assertRaisesRegex(RebotControlError, "q feedback must contain six finite"):
                self._runner(config, fake).run()

    def test_inf_feedback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(velocity_rad_s=[math.inf, 0, 0, 0, 0, 0])
            with self.assertRaisesRegex(RebotControlError, "qd feedback must contain six finite"):
                self._runner(config, fake).run()

    def test_primary_fault_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(primary_fault_code=500119)
            with self.assertRaisesRegex(RebotControlError, "primary fault active"):
                self._runner(config, fake).run()

    def test_torque_invalid_is_recorded_without_inventing_effort(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(torque_valid=[False] * 6, torque_nm=[1, 2, 3, 4, 5, 6])
            self._runner(config, fake).run()
            with Path(config["output_csv"]).open("r", encoding="utf-8", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertTrue(all(math.isnan(float(row[f"effort_reported{i}"])) for i in range(6)))
            self.assertTrue(all(row[f"torque_valid{i}"] == "0" for i in range(6)))

    def test_connect_failure_has_no_motor_cleanup_calls(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(fail_on={"connect"})
            with self.assertRaisesRegex(RebotControlError, "connect failed"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls, ["connect"])

    def test_enable_failure_attempts_disable_for_ambiguous_ack(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"enable"})
            with self.assertRaisesRegex(RebotControlError, "enable failed"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls, ["connect", "read_state", "enable", "disable", "close"])

    def test_enter_servo_failure_disables_and_closes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"enter_servo"})
            with self.assertRaisesRegex(RebotControlError, "enter_servo failed"):
                self._runner(config, fake).run()
            self.assertEqual(
                fake.calls,
                [
                    "connect", "read_state", "enable", "read_state", "enter_servo",
                    "exit_servo", "stop", "disable", "close",
                ],
            )

    def test_servo_command_failure_runs_full_error_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"servo_joint"})
            with self.assertRaisesRegex(RebotControlError, "servo_joint failed"):
                self._runner(config, fake).run()
            self.assertEqual(
                fake.calls[-5:],
                ["servo_joint", "exit_servo", "stop", "disable", "close"],
            )

    def test_cleanup_failure_does_not_skip_remaining_stop_disable_close(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"servo_joint", "exit_servo"})
            with self.assertRaisesRegex(RebotControlError, "cleanup failures: exit_servo"):
                self._runner(config, fake).run()
            self.assertEqual(
                fake.calls[-5:],
                ["servo_joint", "exit_servo", "stop", "disable", "close"],
            )

    def test_state_timeout_in_state_only_closes_without_motor_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            config["state_timeout_s"] = 0.002
            fake = MockArmClient(no_state=True)
            with self.assertRaisesRegex(RebotControlError, "state timeout"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls[0], "connect")
            self.assertEqual(fake.calls[-1], "close")
            self.assertNotIn("enable", fake.calls)

    def test_connection_loss_during_servo_runs_full_error_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory, samples=2)
            fake = MockArmClient(
                position_rad=[0.2, -1.0, -1.0, 0.0, 0.0, 0.0],
                disconnect_after_state_reads=3,
            )
            with self.assertRaisesRegex(RebotControlError, "state timeout"):
                self._runner(config, fake).run()
            self.assertIn("servo_joint", fake.calls)
            self.assertEqual(fake.calls[-4:], ["exit_servo", "stop", "disable", "close"])

    def test_feedback_timeout_after_enter_servo_runs_full_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(disconnect_after_state_reads=2)
            with self.assertRaisesRegex(RebotControlError, "state timeout"):
                self._runner(config, fake).run()
            self.assertIn("servo_joint", fake.calls)
            self.assertEqual(fake.calls[-4:], ["exit_servo", "stop", "disable", "close"])

    def test_historical_100hz_A_requires_new_replay_mode_at_config_load(self) -> None:
        with self.assertRaisesRegex(
            ValueError, "trajectory_replay_mode=actual_time_quintic_v1"
        ):
            load_hardware_config(
                REPO_ROOT / "results" / "rebot_real_ab" / "A" / "hardware.smoke.yaml",
                repo_root=REPO_ROOT,
            )

    def test_servo_safe_200hz_v1_acceptance_cannot_authorize_new_replay(self) -> None:
        for trajectory_name in ("A", "B"):
            with self.subTest(trajectory_name=trajectory_name):
                root = REPO_ROOT / "results" / "rebot_real_ab_servo_safe_200hz" / trajectory_name
                config = load_hardware_config(
                    root / "hardware.pending.yaml",
                    repo_root=REPO_ROOT,
                    overrides={"trajectory_replay_mode": TRAJECTORY_REPLAY_MODE},
                )
                artifact = load_replay_artifact(
                    config["trajectory_artifact"], config["trajectory_metadata"]
                )
                with self.assertRaisesRegex(ValueError, "acceptance v2 is required"):
                    validate_preview_acceptance(
                        config["trajectory_preview_acceptance"], artifact, repo_root=REPO_ROOT
                    )

    def test_excitation_normal_completion_parks_without_polluting_raw_commands(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._runner(config, fake).run()
            self.assertEqual(metadata["observed_sample_count"], 3001)
            self.assertEqual(len(fake.movej_targets), 2)
            self.assertEqual(metadata["shutdown"]["strategy"], "controlled_park")
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")
            self.assertEqual(metadata["shutdown"]["park_movej_command_count"], 1)
            with Path(config["output_csv"]).open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3001)
            self.assertTrue(all(row["control_mode"] == "excitation" for row in rows))
            self.assertTrue(all(row["command_valid"] == "1" for row in rows))
            reference_timestamps = [int(row["timestamp_host_command_ns"]) for row in rows]
            self.assertTrue(
                all(
                    current - previous == 10_000_000
                    for previous, current in zip(
                        reference_timestamps, reference_timestamps[1:]
                    )
                )
            )
            self.assertTrue(
                all(int(row["actual_dispatch_interval_ns"]) >= 10_000_000 for row in rows)
            )
            self.assertEqual(metadata["dispatch_timing"]["catch_up_burst_count"], 0)
            envelope = metadata["runtime_servo_envelope"]
            self.assertEqual(envelope["strategy"], TRAJECTORY_REPLAY_MODE)
            self.assertEqual(envelope["accepted_excitation_command_count"], 3001)
            self.assertEqual(envelope["trajectory_time_start_s"], 0.0)
            self.assertEqual(envelope["trajectory_time_end_s"], 30.0)
            for joint in envelope["per_joint"].values():
                self.assertGreaterEqual(joint["delta_q_margin_rad"], 0.0)
                self.assertGreaterEqual(joint["qd_margin_rad_s"], 0.0)
                self.assertGreaterEqual(joint["qdd_margin_rad_s2"], 0.0)
                self.assertGreaterEqual(joint["jerk_margin_rad_s3"], 0.0)
            artifact = load_replay_artifact(
                config["trajectory_artifact"], config["trajectory_metadata"]
            )
            expected_sdk_targets = [
                tuple(
                    (sample.q_ref[index] - config["joint_offset_rad"][index])
                    / config["joint_direction"][index]
                    for index in range(6)
                )
                for sample in artifact.samples
            ]
            self.assertEqual(fake.servo_targets[1:], expected_sdk_targets)
            self.assertEqual(float(rows[0]["trajectory_time_s"]), 0.0)
            self.assertEqual(float(rows[-1]["trajectory_time_s"]), 30.0)
            final_q_cmd = [float(rows[-1][f"q_cmd{i}"]) for i in range(6)]
            self.assertNotEqual(final_q_cmd, metadata["shutdown"]["park_target_q"])

    def test_excitation_dispatches_at_5ms_while_state_updates_at_100ms(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory, rate_hz=200.0)
            config["maximum_tracking_error_rad"] = [1.0] * 6
            clock = FakeClock()
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                state_update_period_s=0.1,
                monotonic_ns_fn=clock.monotonic_ns,
            )
            runner = RebotHardwareRunner(
                config,
                repo_root=REPO_ROOT,
                client_factory=lambda **_: fake,
                mock_backend=True,
                sleep_fn=clock.sleep,
                monotonic_fn=clock.monotonic,
                monotonic_ns_fn=clock.monotonic_ns,
            )
            metadata = runner.run()
            self.assertEqual(metadata["observed_sample_count"], 6001)
            self.assertEqual(len(fake.servo_targets), 6002)
            self.assertLess(fake._state_version, 400)
            self.assertEqual(
                metadata["dispatch_timing"]["actual_dispatch_interval_min_ns"],
                5_000_000,
            )
            self.assertEqual(metadata["dispatch_timing"]["catch_up_burst_count"], 0)
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")

    def test_excitation_stops_within_state_timeout_when_snapshot_updates_stop(self) -> None:
        class FreezeAfterFirstArtifactTarget(MockArmClient):
            def servo_joint(self, *args, **kwargs):
                reply = super().servo_joint(*args, **kwargs)
                if len(self.servo_targets) >= 2:
                    self.state_updates_enabled = False
                return reply

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            config["state_timeout_s"] = 0.02
            fake = FreezeAfterFirstArtifactTarget(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            with self.assertRaisesRegex(RebotControlError, "state snapshot stale"):
                self._runner(config, fake).run()
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            failure = metadata["failure"]
            self.assertEqual(failure["stage"], "excitation_state_snapshot")
            self.assertLessEqual(
                failure["artifact_time_s"],
                config["state_timeout_s"] + 1.5 / config["control_rate_hz"],
            )
            self.assertGreater(failure["state_snapshot_age_ms"], 20.0)
            for key in (
                "sample_index", "q_ref", "previous_q_ref", "measured_q",
                "target_step_delta_q", "tracking_error_q",
                "trajectory_time_s", "trajectory_interval_index",
                "trajectory_interval_ratio", "actual_dispatch_timestamp_ns",
                "servo_sequence", "feedback_age_ms", "primary_fault_code",
                "servo_reject_counters",
            ):
                self.assertIn(key, failure)
            self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")
            self.assertEqual(fake.calls[-3:], ["exit_servo", "disable", "close"])

    def test_late_cycles_never_create_short_catch_up_dispatches(self) -> None:
        for cycle_cost_ms in (6.5, 8.0, 12.0):
            with self.subTest(cycle_cost_ms=cycle_cost_ms), tempfile.TemporaryDirectory() as directory:
                clock = FakeClock()

                class CostlyServoMock(MockArmClient):
                    def servo_joint(self, *args, **kwargs):
                        reply = super().servo_joint(*args, **kwargs)
                        clock.advance_ms(cycle_cost_ms)
                        return reply

                config = self._excitation_config(directory)
                fake = CostlyServoMock(
                    position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                    follow_movej_targets=True,
                    follow_servo_targets=True,
                    monotonic_ns_fn=clock.monotonic_ns,
                )
                runner = RebotHardwareRunner(
                    config,
                    repo_root=REPO_ROOT,
                    client_factory=lambda **_: fake,
                    mock_backend=True,
                    sleep_fn=clock.sleep,
                    monotonic_fn=clock.monotonic,
                    monotonic_ns_fn=clock.monotonic_ns,
                )
                metadata = runner.run()
                timing = metadata["dispatch_timing"]
                self.assertGreaterEqual(
                    timing["actual_dispatch_interval_min_ns"], 10_000_000
                )
                self.assertEqual(timing["interval_below_nominal_count"], 0)
                self.assertEqual(timing["catch_up_burst_count"], 0)

    def test_excitation_preposition_waits_through_transient_velocity_then_parks(self) -> None:
        class TransientVelocityMock(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._transient_reads = 0

            def movej(self, *args, **kwargs):
                reply = super().movej(*args, **kwargs)
                self.velocity_rad_s = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
                self._transient_reads = 1
                return reply

            @property
            def state_store(self):
                if self._transient_reads > 0:
                    self._transient_reads -= 1
                else:
                    self.velocity_rad_s = [0.0] * 6
                return MockArmClient.state_store.fget(self)

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = TransientVelocityMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._runner(config, fake).run()
            self.assertEqual(metadata["observed_sample_count"], 3001)
            self.assertEqual(metadata["preposition"]["status"], "completed")
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")

    def test_excitation_settle_skips_one_transient_stale_frame_for_preposition_and_park(self) -> None:
        class OneStaleFrameAfterEachMoveJ(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._stale_reads_remaining = 0

            def movej(self, *args, **kwargs):
                reply = super().movej(*args, **kwargs)
                self.feedback_age_ms = (100.0,) * 6
                self._stale_reads_remaining = 1
                return reply

            @property
            def state_store(self):
                store = MockArmClient.state_store.fget(self)
                if self._stale_reads_remaining > 0:
                    self._stale_reads_remaining -= 1
                else:
                    self.feedback_age_ms = (1.0,) * 6
                return store

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = OneStaleFrameAfterEachMoveJ(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._runner(config, fake).run()
            self.assertEqual(metadata["observed_sample_count"], 3001)
            self.assertEqual(metadata["preposition"]["status"], "completed")
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")
            self.assertEqual(len(fake.movej_targets), 2)

    def test_preposition_settle_software_timeout_stops_then_parks(self) -> None:
        class FirstMoveNeverSettlesMock(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._movej_attempts = 0

            def movej(self, *args, **kwargs):
                self._movej_attempts += 1
                self.settle_after_movej = self._movej_attempts > 1
                if self._movej_attempts == 1:
                    self.velocity_rad_s = [0.02] * 6
                return super().movej(*args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            config["command_timeout_s"] = 0.02
            fake = FirstMoveNeverSettlesMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            with self.assertRaisesRegex(RebotControlError, "preposition settle timed out"):
                self._runner(config, fake).run()
            self.assertEqual(len(fake.movej_targets), 2)
            self.assertIn("stop", fake.calls)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "controlled_park")
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")

    def test_excitation_runtime_validation_error_stops_then_parks(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeClock()

            class LongAckDelayMock(MockArmClient):
                def servo_joint(self, *args, **kwargs):
                    reply = super().servo_joint(*args, **kwargs)
                    if len(self.servo_targets) == 2:
                        clock.advance_ms(101.0)
                    return reply

            config = self._excitation_config(directory)
            fake = LongAckDelayMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                monotonic_ns_fn=clock.monotonic_ns,
            )
            runner = RebotHardwareRunner(
                config,
                repo_root=REPO_ROOT,
                client_factory=lambda **_: fake,
                mock_backend=True,
                sleep_fn=clock.sleep,
                monotonic_fn=clock.monotonic,
                monotonic_ns_fn=clock.monotonic_ns,
            )
            with self.assertRaisesRegex(RebotControlError, "timestamp interval"):
                runner.run()
            self.assertEqual(len(fake.servo_targets), 2)
            self.assertIn("exit_servo", fake.calls)
            self.assertIn("stop", fake.calls)
            self.assertEqual(len(fake.movej_targets), 2)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["failure"]["stage"], "excitation_runtime_envelope")
            self.assertEqual(metadata["failure"]["violating_quantity"], "dt_max_s")
            self.assertGreater(metadata["failure"]["predicted_dt_s"], 0.1)
            self.assertEqual(metadata["shutdown"]["park_status"], "completed")

    def test_excitation_tracking_lag_is_monitor_only_and_records_quality_summary(self) -> None:
        class InjectTrackingLag(MockArmClient):
            def servo_joint(self, *args, **kwargs):
                reply = super().servo_joint(*args, **kwargs)
                if len(self.servo_targets) == 2:
                    self.position_rad[3] -= 0.02
                return reply

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            artifact = load_replay_artifact(
                config["trajectory_artifact"], config["trajectory_metadata"]
            )
            fake = InjectTrackingLag(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._runner(config, fake).run()
            self.assertEqual(metadata["observed_sample_count"], len(artifact.samples))
            self.assertEqual(len(fake.servo_targets), len(artifact.samples) + 1)
            self.assertNotIn("failure", metadata)
            self.assertEqual(metadata["motion_status"], "completed")
            self.assertEqual(metadata["shutdown"]["strategy"], "controlled_park")
            quality = metadata["tracking_quality"]
            self.assertEqual(quality["status"], "warning")
            self.assertGreater(quality["per_joint"]["J4"]["max_abs_rad"], 0.01)
            self.assertGreaterEqual(
                quality["per_joint"]["J4"]["threshold_exceed_count"], 1
            )
            self.assertEqual(
                quality["per_joint"]["J4"]["first_threshold_exceed_sample_index"], 1
            )
            self.assertEqual(metadata["identification_data_quality"]["status"], "warning")
            self.assertFalse(metadata["identification_data_quality"]["accepted"])
            self.assertEqual(fake.calls[-2:], ["disable", "close"])

    def test_excitation_runtime_freshness_and_fault_gates_keep_fail_safe_cleanup_order(self) -> None:
        for injection, expected_error in (
            ("freshness", "feedback stale"),
            ("fault", "primary fault active"),
        ):
            with self.subTest(injection=injection), tempfile.TemporaryDirectory() as directory:
                class InjectRuntimeStateFailure(MockArmClient):
                    def servo_joint(self, *args, **kwargs):
                        reply = super().servo_joint(*args, **kwargs)
                        if len(self.servo_targets) == 2:
                            if injection == "freshness":
                                self.feedback_age_ms = (101.0,) * 6
                            else:
                                self.primary_fault_code = 200204
                        return reply

                config = self._excitation_config(directory)
                fake = InjectRuntimeStateFailure(
                    position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                    follow_movej_targets=True,
                    follow_servo_targets=True,
                )
                with self.assertRaisesRegex(RebotControlError, expected_error):
                    self._runner(config, fake).run()
                metadata = yaml.safe_load(
                    Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
                )
                self.assertEqual(
                    metadata["failure"]["stage"], "excitation_state_snapshot"
                )
                self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")
                self.assertEqual(fake.calls[-3:], ["exit_servo", "disable", "close"])

    def test_excitation_servo_reject_records_attempt_and_sdk_diagnostics(self) -> None:
        class RejectFirstArtifactTargetMock(MockArmClient):
            def servo_joint(self, *args, **kwargs):
                if self.servo_accepted_targets >= 1:
                    self.calls.append("servo_joint")
                    self.servo_rejected_targets = 1
                    self.servo_target_jump_rejects = 1
                    self.servo_target_age_ns = 5_000_000
                    self.primary_fault_code = 500115
                    raise RuntimeError("status=rejected code=1002")
                return super().servo_joint(*args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = RejectFirstArtifactTargetMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            with self.assertRaisesRegex(RebotControlError, "Servo send rejected"):
                self._runner(config, fake).run()
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            failure = metadata["failure"]
            self.assertEqual(failure["stage"], "excitation_servo_send")
            self.assertEqual(failure["sample_index"], 0)
            self.assertEqual(failure["artifact_time_s"], 0.0)
            self.assertEqual(failure["servo_sequence"], 2)
            self.assertEqual(failure["servo_last_accepted_sequence"], 1)
            self.assertEqual(failure["servo_rejected_targets"], 1)
            self.assertEqual(failure["servo_target_jump_rejects"], 1)
            self.assertEqual(failure["primary_fault_code"], 500115)
            self.assertEqual(
                metadata["runtime_servo_envelope"]["accepted_excitation_command_count"],
                0,
            )
            self.assertEqual(
                failure["primary_fault_code_symbolic_mapping"],
                "unresolved_in_available_sdk_checkout",
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")

    def test_excitation_invalid_feedback_forces_fail_safe_without_park(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                after_movej_feedback_valid=[False, True, True, True, True, True],
            )
            with self.assertRaisesRegex(RebotControlError, "invalid joint feedback"):
                self._runner(config, fake).run()
            self.assertEqual(len(fake.movej_targets), 1)
            self.assertNotIn("stop", fake.calls)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")
            self.assertEqual(metadata["shutdown"]["park_status"], "skipped_fail_safe")

    def test_excitation_primary_fault_forces_fail_safe_without_park(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                after_movej_primary_fault_code=200204,
            )
            with self.assertRaisesRegex(RebotControlError, "primary fault active"):
                self._runner(config, fake).run()
            self.assertEqual(len(fake.movej_targets), 1)
            self.assertNotIn("stop", fake.calls)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")
            self.assertEqual(metadata["shutdown"]["park_status"], "skipped_fail_safe")

    def test_excitation_connection_loss_forces_fail_safe_without_park(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                disconnect_after_state_reads=3,
            )
            with self.assertRaisesRegex(RebotControlError, "state timeout"):
                self._runner(config, fake).run()
            self.assertEqual(len(fake.movej_targets), 1)
            self.assertNotIn("stop", fake.calls)
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "fail_safe")
            self.assertEqual(metadata["shutdown"]["park_status"], "skipped_fail_safe")

    def test_park_movej_failure_is_recorded_and_disable_close_still_run(self) -> None:
        class FailSecondMoveJMock(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._movej_attempts = 0

            def movej(self, *args, **kwargs):
                self._movej_attempts += 1
                if self._movej_attempts == 2:
                    self.calls.append("movej")
                    raise RuntimeError("mock park movej failure")
                return super().movej(*args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            config = self._excitation_config(directory)
            fake = FailSecondMoveJMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            with self.assertRaisesRegex(RebotControlError, "cleanup failures: controlled_park"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls[-2:], ["disable", "close"])
            metadata = yaml.safe_load(
                Path(config["output_csv"]).with_suffix(".meta.yaml").read_text()
            )
            self.assertEqual(metadata["shutdown"]["strategy"], "controlled_park")
            self.assertEqual(metadata["shutdown"]["park_status"], "failed")
            self.assertEqual(metadata["shutdown"]["disable_status"], "completed")
            self.assertEqual(metadata["shutdown"]["close_status"], "completed")

    def test_already_at_park_skips_duplicate_movej(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["controlled_park_before_disable"] = True
            fake = MockArmClient(
                position_rad=[math.pi, 0.0, 0.0, 0.0, 0.0, math.pi / 2]
            )
            metadata = self._runner(config, fake).run()
            self.assertNotIn("movej", fake.calls)
            self.assertEqual(metadata["shutdown"]["strategy"], "controlled_park")
            self.assertEqual(metadata["shutdown"]["park_status"], "already_at_park")
            self.assertEqual(metadata["shutdown"]["park_movej_command_count"], 0)

    def test_excitation_smoke_allows_historical_visual_mapping_without_claiming_formal_scope(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory, mode="excitation")
            config["allow_hardware"] = True
            config["allow_motion"] = True
            config["joint_mapping_verified"] = True
            config["j1_convention"] = "PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907"
            config["joint_mapping_scope"] = "excitation_smoke"
            self._runner(config, MockArmClient(), mock_backend=False)._assert_session_authorized(
                "excitation"
            )

    def test_formal_excitation_rejects_historical_visual_mapping(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory, mode="excitation")
            config["allow_hardware"] = True
            config["allow_motion"] = True
            config["joint_mapping_verified"] = True
            config["j1_convention"] = "PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907"
            config["joint_mapping_scope"] = "excitation"
            with self.assertRaisesRegex(PermissionError, "excitation_smoke only"):
                self._runner(config, MockArmClient(), mock_backend=False)._assert_session_authorized(
                    "excitation"
                )

    def test_excitation_rejects_non_frozen_source_before_connect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory, mode="excitation")
            config["allow_motion"] = True
            config["joint_mapping_verified"] = True
            config["j1_convention"] = "MOCK_CANONICAL_REBOT_DM"
            config["joint_mapping_scope"] = "excitation"
            fake = MockArmClient()
            with self.assertRaisesRegex(
                ValueError,
                "trajectory_source=frozen_replay_artifact",
            ):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls, [])


if __name__ == "__main__":
    unittest.main()
