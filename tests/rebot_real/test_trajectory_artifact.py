from __future__ import annotations

import csv
from dataclasses import replace
import math
from pathlib import Path
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
    CSV_COLUMNS,
    load_replay_artifact,
    sha256_file,
    validate_replay_runtime_limits,
)

Q0 = [0.0, -1.0, -1.0, 0.0, 0.0, -0.6]


class ReplayFixture:
    def __init__(self, directory: str) -> None:
        self.root = Path(directory)
        self.artifact = self.root / "trajectory.csv"
        self.metadata = self.root / "trajectory.meta.yaml"
        self.coefficients = self.root / "coefficients.csv"
        self.model = self.root / "model.xml"
        self.limits = self.root / "limits.yaml"
        self.collision_model = self.root / "collision.xml"
        self.report = self.root / "preview_report.yaml"
        self.mp4 = self.root / "preview.mp4"
        self.acceptance = self.root / "acceptance.yaml"
        self.output = self.root / "mock_replay.csv"
        for path, content in (
            (self.coefficients, "accepted coefficients\n"),
            (self.model, "<mujoco/>\n"),
            (self.limits, "limits: fixture\n"),
            (self.collision_model, "<mujoco model='collision'/>\n"),
        ):
            path.write_text(content, encoding="utf-8")
        self.rows = self.default_rows()
        self.write_artifact()

    @staticmethod
    def default_rows() -> list[list[float]]:
        rows = []
        for index, delta in enumerate((0.0, 0.001, 0.002)):
            q = list(Q0)
            q[0] += delta
            qd = [0.0] * 6
            qd[0] = 0.1 if index else 0.0
            qdd = [0.0] * 6
            rows.append([index * 0.01, *q, *qd, *qdd])
        return rows

    def write_artifact(self, *, rows=None, header=None, metadata_overrides=None) -> None:
        rows = self.rows if rows is None else rows
        with self.artifact.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(list(CSV_COLUMNS) if header is None else header)
            writer.writerows(rows)
        metadata = {
            "schema_version": "rebot_replay_trajectory_v1",
            "robot": "rebot_dm",
            "dof": 6,
            "duration_s": float(rows[-1][0]),
            "sample_rate_hz": 1.0 / float(rows[1][0] - rows[0][0]),
            "sample_count": len(rows),
            "source_type": "cxx_fourier_trajectory",
            "source_controller_precheck": "PASS",
            "collision_precheck": "PASS",
            "collision_precheck_sample_count": len(rows),
            "sample_rate_status": "candidate_replay_rate_not_hardware_certified",
            "q_start": list(rows[0][1:7]),
            "trajectory_sha256": sha256_file(self.artifact),
            "source_coefficient_file": str(self.coefficients),
            "source_coefficient_sha256": sha256_file(self.coefficients),
            "model_file": str(self.model),
            "model_hash": sha256_file(self.model),
            "limits_config_file": str(self.limits),
            "limits_config_hash": sha256_file(self.limits),
            "collision_model_file": str(self.collision_model),
            "collision_model_hash": sha256_file(self.collision_model),
        }
        if metadata_overrides:
            metadata.update(metadata_overrides)
        self.metadata.write_text(
            yaml.safe_dump(metadata, sort_keys=False),
            encoding="utf-8",
        )

    def write_preview_evidence(
        self,
        *,
        accepted: bool = True,
        acceptance_hash: str | None = None,
        report_status: str = "PASS",
        report_hash: str | None = None,
        create_report: bool = True,
        create_mp4: bool = True,
    ) -> None:
        artifact_hash = sha256_file(self.artifact)
        if create_report:
            self.report.write_text(
                yaml.safe_dump({
                    "schema_version": "rebot_trajectory_preview_report_v1",
                    "trajectory_sha256": artifact_hash if report_hash is None else report_hash,
                    "preview_status": report_status,
                }, sort_keys=False),
                encoding="utf-8",
            )
        if create_mp4:
            self.mp4.write_bytes(b"mock preview")
        self.acceptance.write_text(
            yaml.safe_dump({
                "schema_version": "rebot_trajectory_preview_acceptance_v1",
                "trajectory_sha256": artifact_hash if acceptance_hash is None else acceptance_hash,
                "preview_report": str(self.report),
                "preview_mp4": str(self.mp4),
                "operator": "test",
                "review_date": "2026-09-08",
                "accepted_for_hardware": accepted,
            }, sort_keys=False),
            encoding="utf-8",
        )

    def runtime_config(self) -> dict:
        config = load_hardware_config(
            REPO_ROOT / "config" / "rebot_real_experiment.yaml",
            repo_root=REPO_ROOT,
        )
        config.update({
            "control_rate_hz": 100.0,
            "duration_s": 0.02,
            "joint_position_min_rad": [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14],
            "joint_position_max_rad": [2.8, 0.0, 0.0, 1.57, 1.57, 3.14],
            "maximum_command_velocity_rad_s": [1.0] * 6,
            "maximum_command_acceleration_rad_s2": [2.0] * 6,
            "maximum_command_jerk_rad_s3": [10.0] * 6,
        })
        return config

    def replay_config(self) -> dict:
        config = self.runtime_config()
        config["control_mode"] = "excitation"
        config["duration_s"] = 0.02
        config["max_samples"] = None
        config["output_csv"] = str(self.output)
        config["allow_motion"] = True
        config["joint_mapping_verified"] = True
        config["j1_convention"] = "MOCK_CANONICAL_REBOT_DM"
        config["joint_mapping_scope"] = "excitation"
        config["start_position_tolerance_rad"] = 0.01
        config["trajectory_source"] = "frozen_replay_artifact"
        config["trajectory_hash"] = None
        config["trajectory_artifact"] = str(self.artifact)
        config["trajectory_metadata"] = str(self.metadata)
        config["trajectory_preview_acceptance"] = str(self.acceptance)
        return config


class TrajectoryArtifactTest(unittest.TestCase):
    def test_artifact_sha_correct(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            self.assertEqual(artifact.sha256, sha256_file(fixture.artifact))

    def test_nan_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[1][1] = float("nan")
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(ValueError, "must be finite"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_timestamp_non_increasing_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[2][0] = rows[1][0]
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_non_fixed_grid_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[2][0] = 0.021
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(ValueError, "fixed sampling grid"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_wrong_header_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            header = list(CSV_COLUMNS)
            header[1] = "wrong"
            fixture.write_artifact(header=header)
            with self.assertRaisesRegex(ValueError, "header must be exactly"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_wrong_dof_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_artifact(metadata_overrides={"dof": 7})
            with self.assertRaisesRegex(ValueError, "metadata dof"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_metadata_artifact_hash_mismatch_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_artifact(metadata_overrides={"trajectory_sha256": "0" * 64})
            with self.assertRaisesRegex(ValueError, "hash does not match"):
                load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_provenance_hash_mismatches_rejected(self) -> None:
        for attribute, pattern in (
            ("coefficients", "source_coefficient_sha256 mismatch"),
            ("model", "model_hash mismatch"),
            ("limits", "limits_config_hash mismatch"),
            ("collision_model", "collision_model_hash mismatch"),
        ):
            with self.subTest(attribute=attribute), tempfile.TemporaryDirectory() as directory:
                fixture = ReplayFixture(directory)
                getattr(fixture, attribute).write_text("changed\n", encoding="utf-8")
                with self.assertRaisesRegex(ValueError, pattern):
                    load_replay_artifact(fixture.artifact, fixture.metadata)

    def test_metadata_sample_shape_mismatches_rejected(self) -> None:
        for override, pattern in (
            ({"sample_count": 99}, "sample_count mismatch"),
            ({"sample_rate_hz": 99.0}, "sample_rate_hz mismatch"),
            ({"duration_s": 1.0}, "duration_s mismatch"),
        ):
            with self.subTest(override=override), tempfile.TemporaryDirectory() as directory:
                fixture = ReplayFixture(directory)
                fixture.write_artifact(metadata_overrides=override)
                with self.assertRaisesRegex(ValueError, pattern):
                    load_replay_artifact(fixture.artifact, fixture.metadata)

    def _validate_runtime(self, fixture: ReplayFixture, **updates) -> None:
        artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
        config = fixture.runtime_config()
        config.update(updates)
        validate_replay_runtime_limits(artifact, config)

    def test_position_violation_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            with self.assertRaisesRegex(ValueError, "violates position limits"):
                self._validate_runtime(
                    fixture,
                    joint_position_max_rad=[-0.1, 0, 0, 1.57, 1.57, 3.14],
                )

    def test_velocity_violation_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            with self.assertRaisesRegex(ValueError, "violates velocity limit"):
                self._validate_runtime(
                    fixture,
                    maximum_command_velocity_rad_s=[0.05] * 6,
                )

    def test_acceleration_violation_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[1][13] = 3.0
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(ValueError, "violates acceleration limit"):
                self._validate_runtime(fixture)

    def test_jerk_violation_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[1][13] = 0.2
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(ValueError, "violates jerk limit"):
                self._validate_runtime(fixture)

    def test_control_rate_mismatch_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            with self.assertRaisesRegex(ValueError, "exactly match"):
                self._validate_runtime(fixture, control_rate_hz=50.0)

    def test_runtime_duration_mismatch_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            with self.assertRaisesRegex(ValueError, "duration_s must exactly match"):
                self._validate_runtime(fixture, duration_s=30.0)

    def test_runtime_sample_count_mismatch_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()[:-1]
            fixture.write_artifact(rows=rows)
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            artifact = replace(artifact, duration_s=0.02)
            config = fixture.runtime_config()
            with self.assertRaisesRegex(ValueError, "sample_count=.*fixed grid"):
                validate_replay_runtime_limits(artifact, config)

    def _run_replay(self, fixture: ReplayFixture, *, factory):
        return RebotHardwareRunner(
            fixture.replay_config(),
            repo_root=REPO_ROOT,
            client_factory=factory,
            mock_backend=True,
            sleep_fn=lambda _: None,
        ).run()

    def test_acceptance_false_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(accepted=False)
            factory_calls = []

            def factory(**kwargs):
                factory_calls.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "preview acceptance is not"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(factory_calls, [])

    def test_acceptance_hash_mismatch_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(acceptance_hash="0" * 64)
            factory_calls = []

            def factory(**kwargs):
                factory_calls.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "acceptance hash"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(factory_calls, [])

    def test_report_fail_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(report_status="FAIL")
            called = []

            def factory(**kwargs):
                called.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "report is not PASS"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(called, [])

    def test_report_hash_mismatch_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(report_hash="0" * 64)
            called = []

            def factory(**kwargs):
                called.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "report hash"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(called, [])

    def test_missing_report_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(create_report=False)
            called = []

            def factory(**kwargs):
                called.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "report file is missing"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(called, [])

    def test_missing_mp4_rejected_before_client_factory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(create_mp4=False)
            called = []

            def factory(**kwargs):
                called.append(kwargs)
                return MockArmClient(**kwargs)

            with self.assertRaisesRegex(PermissionError, "MP4"):
                self._run_replay(fixture, factory=factory)
            self.assertEqual(called, [])

    def test_matching_preview_mock_replay_is_exact(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            fake = MockArmClient(
                position_rad=artifact.q_start,
                follow_servo_targets=True,
            )
            metadata = self._run_replay(fixture, factory=lambda **_: fake)
            expected = [sample.q_ref for sample in artifact.samples]
            self.assertEqual(metadata["observed_sample_count"], len(expected))
            self.assertEqual(metadata["preposition"]["status"], "already_at_start")
            self.assertEqual(metadata["preposition"]["movej_command_count"], 0)
            self.assertEqual(fake.movej_targets, [])
            self.assertEqual(len(fake.servo_targets), len(expected) + 1)
            self.assertEqual(fake.servo_targets[0], artifact.q_start)
            self.assertEqual(fake.servo_targets[1:], expected)
            with fixture.output.open("r", encoding="utf-8", newline="") as stream:
                rows = [
                    row
                    for row in csv.DictReader(stream)
                    if row["command_valid"] == "1"
                ]
            self.assertEqual(len(rows), len(expected))
            for row, q_ref in zip(rows, expected):
                self.assertEqual(
                    tuple(float(row[f"q_cmd{joint}"]) for joint in range(6)),
                    q_ref,
                )

    def test_park_pose_movej_then_excitation_passes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.movej_targets, [artifact.q_start])
            self.assertEqual(metadata["preposition"]["movej_command_count"], 1)
            self.assertEqual(metadata["preposition"]["target_q"], list(artifact.q_start))
            self.assertEqual(metadata["preposition"]["final_q"], list(artifact.q_start))
            self.assertEqual(metadata["preposition"]["max_position_error"], 0.0)
            self.assertEqual(metadata["preposition"]["max_velocity_after_move"], 0.0)
            self.assertEqual(len(fake.servo_targets), len(artifact.samples) + 1)
            self.assertEqual(fake.servo_targets[0], artifact.q_start)
            self.assertEqual(fake.servo_targets[1:], [sample.q_ref for sample in artifact.samples])

    def test_movej_transient_velocity_waits_for_settle_before_servo(self) -> None:
        class OneTransientVelocityFrameMock(MockArmClient):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._post_movej = False
                self._post_movej_reads = 0

            def movej(self, *args, **kwargs):
                reply = super().movej(*args, **kwargs)
                self.velocity_rad_s = [0.02, 0, 0, 0, 0, 0]
                self._post_movej = True
                self._post_movej_reads = 0
                return reply

            @property
            def state_store(self):
                if self._post_movej:
                    if self._post_movej_reads >= 1:
                        self.velocity_rad_s = [0.0] * 6
                    self._post_movej_reads += 1
                return MockArmClient.state_store.fget(self)

        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            fake = OneTransientVelocityFrameMock(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._run_replay(fixture, factory=lambda **_: fake)
            movej_index = fake.calls.index("movej")
            servo_index = fake.calls.index("enter_servo")
            self.assertGreaterEqual(
                fake.calls[movej_index + 1:servo_index].count("read_state"),
                4,
            )
            self.assertEqual(metadata["preposition"]["movej_command_count"], 1)
            self.assertEqual(metadata["preposition"]["max_velocity_after_move"], 0.0)
            self.assertEqual(fake.servo_targets[0], artifact.q_start)

    def test_movej_reject_blocks_all_servo_commands(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2], fail_on={"movej"}
            )
            with self.assertRaisesRegex(RebotControlError, "movej failed"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_timeout_blocks_all_servo_commands(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2], fail_on={"movej_timeout"}
            )
            with self.assertRaisesRegex(RebotControlError, "movej failed"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_final_position_outside_tolerance_blocks_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(position_rad=[0, 0, 0, 0, 0, math.pi / 2])
            with self.assertRaisesRegex(RebotControlError, "preposition settle"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(len(fake.movej_targets), 1)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_unsettled_velocity_blocks_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2],
                velocity_rad_s=[0.02, 0, 0, 0, 0, 0],
                follow_movej_targets=True,
                settle_after_movej=False,
            )
            with self.assertRaisesRegex(RebotControlError, "preposition settle"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_invalid_feedback_blocks_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2],
                follow_movej_targets=True,
                after_movej_feedback_valid=[False, True, True, True, True, True],
            )
            with self.assertRaisesRegex(RebotControlError, "invalid joint feedback"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_stale_feedback_blocks_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2],
                follow_movej_targets=True,
                after_movej_feedback_age_ms=[101.0] * 6,
            )
            with self.assertRaisesRegex(RebotControlError, "feedback stale"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_movej_fault_feedback_blocks_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2],
                follow_movej_targets=True,
                after_movej_primary_fault_code=500119,
            )
            with self.assertRaisesRegex(RebotControlError, "primary fault active"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.servo_targets, [])

    def test_configure_pvt_failure_blocks_movej_and_servo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            fake = MockArmClient(
                position_rad=[0, 0, 0, 0, 0, math.pi / 2], fail_on={"configure_pvt"}
            )
            with self.assertRaisesRegex(RebotControlError, "configure_pvt failed"):
                self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(fake.movej_targets, [])
            self.assertEqual(fake.servo_targets, [])

    def test_real_frozen_artifact_park_movej_then_exact_3001_replay(self) -> None:
        artifact_path = REPO_ROOT / "results/rebot_trajectory_preview/trajectory_preview.csv"
        metadata_path = REPO_ROOT / "results/rebot_trajectory_preview/trajectory_preview.meta.yaml"
        report_path = REPO_ROOT / "results/rebot_trajectory_preview/trajectory_preview_report.yaml"
        mp4_path = REPO_ROOT / "results/rebot_trajectory_preview/trajectory_preview.mp4"
        artifact = load_replay_artifact(artifact_path, metadata_path)
        self.assertEqual(len(artifact.samples), 3001)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            acceptance = root / "acceptance.yaml"
            acceptance.write_text(
                yaml.safe_dump({
                    "schema_version": "rebot_trajectory_preview_acceptance_v1",
                    "trajectory_sha256": artifact.sha256,
                    "preview_report": str(report_path),
                    "preview_mp4": str(mp4_path),
                    "operator": "offline-mock-test",
                    "review_date": "2026-09-08",
                    "accepted_for_hardware": True,
                }, sort_keys=False),
                encoding="utf-8",
            )
            config = load_hardware_config(
                REPO_ROOT / "config/rebot_excitation_mock.yaml", repo_root=REPO_ROOT
            )
            output = root / "full_replay.csv"
            config["output_csv"] = str(output)
            config["control_rate_hz"] = 100.0
            config["duration_s"] = 30.0
            config["trajectory_artifact"] = str(artifact_path)
            config["trajectory_metadata"] = str(metadata_path)
            config["trajectory_preview_acceptance"] = str(acceptance)
            # This test verifies exact replay mechanics of the historical artifact;
            # production qualification of its 100 Hz ServoCore jump gate is tested separately.
            config["maximum_servo_target_delta_rad"] = 0.01
            clock_ns = [1_000_000_000]

            def monotonic_ns() -> int:
                return clock_ns[0]

            def monotonic() -> float:
                return clock_ns[0] * 1e-9

            def sleep(duration_s: float) -> None:
                clock_ns[0] += max(0, int(math.ceil(duration_s * 1e9)))

            fake = MockArmClient(
                position_rad=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2],
                follow_movej_targets=True,
                follow_servo_targets=True,
                monotonic_ns_fn=monotonic_ns,
            )
            metadata = RebotHardwareRunner(
                config,
                repo_root=REPO_ROOT,
                client_factory=lambda **_: fake,
                mock_backend=True,
                sleep_fn=sleep,
                monotonic_fn=monotonic,
                monotonic_ns_fn=monotonic_ns,
            ).run()
            expected = [sample.q_ref for sample in artifact.samples]
            self.assertEqual(fake.movej_targets, [artifact.q_start])
            self.assertEqual(metadata["preposition"]["movej_command_count"], 1)
            self.assertEqual(len(fake.servo_targets), 3002)
            self.assertEqual(fake.servo_targets[0], artifact.q_start)
            self.assertEqual(fake.servo_targets[1:], expected)
            with output.open("r", encoding="utf-8", newline="") as stream:
                rows = [row for row in csv.DictReader(stream) if row["command_valid"] == "1"]
            self.assertEqual(len(rows), 3001)
            for row, q_ref in zip(rows, expected):
                self.assertEqual(
                    tuple(float(row[f"q_cmd{joint}"]) for joint in range(6)), q_ref
                )


if __name__ == "__main__":
    unittest.main()
