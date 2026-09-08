from __future__ import annotations

import csv
import inspect
import math
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.control_adapter import RebotControlError
from rebot_real.mock_client import MockArmClient
from rebot_real.runner import RebotHardwareRunner, load_hardware_config


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

    def _runner(self, config: dict, fake: MockArmClient, *, mock_backend: bool = True) -> RebotHardwareRunner:
        return RebotHardwareRunner(
            config,
            repo_root=REPO_ROOT,
            client_factory=lambda **_: fake,
            mock_backend=mock_backend,
            sleep_fn=lambda _: None,
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
                    "read_state",
                    "servo_joint",
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
                runner._validate_command(q_target, q_reference)

    def test_command_velocity_derived_delta_rejection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_command_velocity_rad_s"] = [0.05] * 6
            config["control_rate_hz"] = 100.0
            runner = self._runner(config, MockArmClient())
            with self.assertRaisesRegex(RebotControlError, "velocity-derived limit"):
                runner._validate_command([0.01, -1, -1, 0, 0, 0], [0.0, -1, -1, 0, 0, 0])

    def test_stale_feedback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory)
            fake = MockArmClient(feedback_age_ms=[100.0] * 6)
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

    def test_servo_rejects_stale_feedback_after_enable_and_disables(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            config["maximum_disabled_feedback_age_ms"] = 110.0
            config["maximum_feedback_age_ms"] = 50.0
            fake = MockArmClient(feedback_age_ms=[100.0] * 6)
            with self.assertRaisesRegex(
                RebotControlError,
                "maximum_feedback_age_ms=50.0",
            ):
                self._runner(config, fake).run()
            self.assertEqual(
                fake.calls,
                ["connect", "read_state", "enable", "read_state", "disable", "close"],
            )

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

    def test_enable_failure_closes_without_false_disable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"enable"})
            with self.assertRaisesRegex(RebotControlError, "enable failed"):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls, ["connect", "read_state", "enable", "close"])

    def test_enter_servo_failure_disables_and_closes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._servo_config(directory)
            fake = MockArmClient(fail_on={"enter_servo"})
            with self.assertRaisesRegex(RebotControlError, "enter_servo failed"):
                self._runner(config, fake).run()
            self.assertEqual(
                fake.calls,
                ["connect", "read_state", "enable", "read_state", "enter_servo", "disable", "close"],
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
            self.assertNotIn("servo_joint", fake.calls)
            self.assertEqual(fake.calls[-4:], ["exit_servo", "stop", "disable", "close"])

    def test_excitation_rejects_non_frozen_source_before_connect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = self._config(directory, mode="excitation")
            config["allow_motion"] = True
            config["joint_mapping_verified"] = True
            config["j1_convention"] = "MOCK_CANONICAL_REBOT_DM"
            fake = MockArmClient()
            with self.assertRaisesRegex(
                ValueError,
                "trajectory_source=frozen_replay_artifact",
            ):
                self._runner(config, fake).run()
            self.assertEqual(fake.calls, [])


if __name__ == "__main__":
    unittest.main()
