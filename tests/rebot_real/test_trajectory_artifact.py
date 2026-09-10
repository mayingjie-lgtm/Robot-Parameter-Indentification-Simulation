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
    TRAJECTORY_REPLAY_MODE,
    evaluate_actual_time_timing_profile,
    load_replay_artifact,
    qualify_replay_artifact,
    resample_actual_time_quintic,
    sha256_file,
    validate_preview_acceptance,
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
        for index in range(3):
            q = list(Q0)
            qd = [0.0] * 6
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
            "continuous_quintic_collision_precheck": "PASS",
            "continuous_quintic_collision_subdivisions_per_interval": 10,
            "continuous_quintic_collision_precheck_sample_count": (len(rows) - 1) * 10 + 1,
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
                    "schema_version": "rebot_trajectory_preview_report_v2",
                    "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
                    "trajectory_sha256": artifact_hash if report_hash is None else report_hash,
                    "preview_status": report_status,
                    "continuous_quintic_collision_precheck": "PASS",
                }, sort_keys=False),
                encoding="utf-8",
            )
        if create_mp4:
            self.mp4.write_bytes(b"mock preview")
        self.acceptance.write_text(
            yaml.safe_dump({
                "schema_version": "rebot_trajectory_preview_acceptance_v2",
                "trajectory_sha256": artifact_hash if acceptance_hash is None else acceptance_hash,
                "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
                "preview_report": str(self.report),
                "preview_report_sha256": sha256_file(self.report) if create_report else "0" * 64,
                "preview_mp4": str(self.mp4),
                "preview_mp4_sha256": sha256_file(self.mp4) if create_mp4 else "0" * 64,
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
            "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
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
        config["trajectory_replay_mode"] = TRAJECTORY_REPLAY_MODE
        config["trajectory_hash"] = None
        config["trajectory_artifact"] = str(self.artifact)
        config["trajectory_metadata"] = str(self.metadata)
        config["trajectory_preview_acceptance"] = str(self.acceptance)
        return config


class TrajectoryArtifactTest(unittest.TestCase):
    def _require_local_evidence(self, *paths: Path) -> None:
        missing = [str(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest(
                "ignored offline evidence is not present in this checkout: "
                + ", ".join(missing)
            )

    def test_artifact_sha_correct(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            self.assertEqual(artifact.sha256, sha256_file(fixture.artifact))

    def test_quintic_matches_every_knot_and_is_c2_at_boundaries(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            for sample in artifact.samples:
                target = resample_actual_time_quintic(artifact, sample.time)
                self.assertEqual(target.q_ref, sample.q_ref)
                self.assertEqual(target.qd_ref, sample.qd_ref)
                self.assertEqual(target.qdd_ref, sample.qdd_ref)
            knot = artifact.samples[1]
            left = resample_actual_time_quintic(artifact, knot.time - 1e-9)
            right = resample_actual_time_quintic(artifact, knot.time + 1e-9)
            for actual_left, actual_right, exact in zip(left.q_ref, right.q_ref, knot.q_ref):
                self.assertAlmostEqual(actual_left, exact, places=10)
                self.assertAlmostEqual(actual_right, exact, places=10)
            for actual_left, actual_right, exact in zip(left.qd_ref, right.qd_ref, knot.qd_ref):
                self.assertAlmostEqual(actual_left, exact, places=8)
                self.assertAlmostEqual(actual_right, exact, places=8)
            for actual_left, actual_right, exact in zip(left.qdd_ref, right.qdd_ref, knot.qdd_ref):
                self.assertAlmostEqual(actual_left, exact, places=6)
                self.assertAlmostEqual(actual_right, exact, places=6)

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

    def test_historical_100hz_A_fails_and_optimized_A_passes_same_gate(self) -> None:
        qualification = yaml.safe_load(
            (REPO_ROOT / "config/rebot_trajectory_preview.yaml").read_text(
                encoding="utf-8"
            )
        )
        old_root = REPO_ROOT / "results/rebot_real_ab_servo_safe_100hz/A"
        new_root = (
            REPO_ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A"
        )
        self._require_local_evidence(
            old_root / "trajectory.csv",
            old_root / "trajectory.meta.yaml",
            new_root / "trajectory.csv",
            new_root / "trajectory.meta.yaml",
        )
        old_artifact = load_replay_artifact(
            old_root / "trajectory.csv", old_root / "trajectory.meta.yaml"
        )
        new_artifact = load_replay_artifact(
            new_root / "trajectory.csv", new_root / "trajectory.meta.yaml"
        )

        old_report = qualify_replay_artifact(old_artifact, qualification)
        new_report = qualify_replay_artifact(new_artifact, qualification)

        self.assertEqual(old_report["preview_status"], "FAIL")
        self.assertEqual(old_report["servo_target_delta_design_status"], "FAIL")
        self.assertIn(
            "J4 ServoCore target delta gate violated: 0.00349060933 > 0.003125 rad",
            old_report["failures"],
        )
        self.assertAlmostEqual(
            old_report["per_joint"][3]["servo_target_delta_abs_max"],
            0.0034906093335972943,
            places=15,
        )

        self.assertEqual(new_report["preview_status"], "FAIL")
        self.assertEqual(new_report["servo_target_delta_design_status"], "PASS")
        self.assertIn(
            "continuous quintic collision precheck is not PASS",
            new_report["failures"],
        )
        self.assertLessEqual(
            max(
                item["servo_target_delta_abs_max"]
                for item in new_report["per_joint"]
            ),
            0.0028 + 1e-12,
        )

    def test_optimized_A_actual_time_jitter_profiles_complete(self) -> None:
        root = REPO_ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A"
        timing_csv = (
            REPO_ROOT
            / "data/rebot_real/20260909_servo_safe_100hz_optimized/A_run02/raw.csv"
        )
        self._require_local_evidence(root / "trajectory.csv", root / "trajectory.meta.yaml", timing_csv)
        artifact = load_replay_artifact(root / "trajectory.csv", root / "trajectory.meta.yaml")
        config = load_hardware_config(
            REPO_ROOT / "config/rebot_excitation_actual_time_pending.yaml",
            repo_root=REPO_ROOT,
        )
        config["joint_position_min_rad"] = [-3.14159265359, -3.174906585, -3.174906585, -1.904906585, -1.604906585, -3.174906585]
        config["joint_position_max_rad"] = [3.14159265359, 0.087266463, 0.087266463, 1.604906585, 1.604906585, 3.174906585]
        with timing_csv.open(newline="") as stream:
            intervals = [
                int(row["actual_dispatch_interval_ns"]) * 1e-9
                for row in csv.DictReader(stream)
                if row["actual_dispatch_interval_ns"]
            ]
        reports = [
            evaluate_actual_time_timing_profile(
                artifact, config, [0.010, 0.013], profile_name="alternating"
            ),
            evaluate_actual_time_timing_profile(
                artifact, config, intervals, profile_name="A_run02",
                repeat_profile_to_endpoint=False,
            ),
        ]
        for report in reports:
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["trajectory_time_end_s"], 30.0)
        self.assertLess(
            max(v["jerk_abs_max_rad_s3"] for v in reports[1]["per_joint"].values()),
            40.0,
        )

    def test_optimized_A_excitation_quality_regression(self) -> None:
        report_path = (
            REPO_ROOT
            / "results/rebot_real_ab_servo_safe_100hz_optimized/A/search_report.yaml"
        )
        self._require_local_evidence(report_path)
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        baseline = report["baseline"]
        selected = report["selected"]
        self.assertEqual(report["status"], "PASS")
        self.assertGreaterEqual(report["accepted_candidate_count"], 2)
        self.assertEqual(baseline["quality"]["rank"], 52)
        self.assertEqual(selected["quality"]["rank"], 52)
        self.assertLess(
            selected["quality"]["effective_condition_number"],
            baseline["quality"]["effective_condition_number"],
        )
        self.assertGreater(
            selected["quality"]["minimum_effective_singular_value"],
            baseline["quality"]["minimum_effective_singular_value"],
        )
        self.assertTrue(
            all(value >= 0.05 for value in selected["qd_abs_max_per_joint"])
        )

    def _validate_runtime(self, fixture: ReplayFixture, **updates) -> None:
        artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
        config = fixture.runtime_config()
        config.update(updates)
        validate_replay_runtime_limits(artifact, config)

    def test_servo_target_delta_violation_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            rows = fixture.default_rows()
            rows[1][1] = Q0[0] + 0.004
            fixture.write_artifact(rows=rows)
            with self.assertRaisesRegex(
                ValueError, "exceeds lower ServoCore fixed gate"
            ):
                self._validate_runtime(fixture)

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
            rows = fixture.default_rows()
            rows[1][7] = 0.1
            fixture.write_artifact(rows=rows)
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

    def test_hardware_acceptance_false_is_rejected_by_default_validator(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(accepted=False)
            artifact = load_replay_artifact(fixture.artifact, fixture.metadata)
            with self.assertRaisesRegex(PermissionError, "preview acceptance is not"):
                validate_preview_acceptance(
                    fixture.acceptance,
                    artifact,
                    repo_root=REPO_ROOT,
                )

    def test_mock_replay_allows_unaccepted_preview_after_hash_and_report_validation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence(accepted=False)
            fake = MockArmClient(
                follow_movej_targets=True,
                follow_servo_targets=True,
            )
            metadata = self._run_replay(fixture, factory=lambda **_: fake)
            self.assertEqual(metadata["motion_status"], "completed")
            self.assertEqual(metadata["observed_sample_count"], 3)
            timing = metadata["dispatch_timing"]
            self.assertEqual(timing["nominal_period_ns"], 10_000_000)
            self.assertEqual(timing["nominal_rate_hz"], 100.0)
            self.assertGreaterEqual(timing["actual_dispatch_interval_min_ns"], 10_000_000)
            self.assertEqual(timing["catch_up_burst_count"], 0)
            self.assertEqual(timing["command_dispatch_timestamp_mismatch_count"], 0)

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

    def test_acceptance_rejects_tampered_report_mp4_and_strategy(self) -> None:
        for tamper, pattern in (
            ("report", "report hash mismatch"),
            ("mp4", "MP4 hash mismatch"),
            ("strategy", "strategy mismatch"),
        ):
            with self.subTest(tamper=tamper), tempfile.TemporaryDirectory() as directory:
                fixture = ReplayFixture(directory)
                fixture.write_preview_evidence()
                if tamper == "report":
                    fixture.report.write_text(fixture.report.read_text() + "notes: changed\n")
                elif tamper == "mp4":
                    fixture.mp4.write_bytes(b"changed preview")
                else:
                    acceptance = yaml.safe_load(fixture.acceptance.read_text())
                    acceptance["trajectory_replay_mode"] = "fixed_samples_v1"
                    fixture.acceptance.write_text(yaml.safe_dump(acceptance, sort_keys=False))
                with self.assertRaisesRegex(PermissionError, pattern):
                    self._run_replay(fixture, factory=lambda **kwargs: MockArmClient(**kwargs))

    def test_v1_acceptance_cannot_authorize_actual_time_replay(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReplayFixture(directory)
            fixture.write_preview_evidence()
            acceptance = yaml.safe_load(fixture.acceptance.read_text())
            acceptance["schema_version"] = "rebot_trajectory_preview_acceptance_v1"
            fixture.acceptance.write_text(yaml.safe_dump(acceptance, sort_keys=False))
            with self.assertRaisesRegex(ValueError, "acceptance v2 is required"):
                self._run_replay(fixture, factory=lambda **kwargs: MockArmClient(**kwargs))

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

    def test_optimized_A_park_movej_then_exact_3001_replay(self) -> None:
        run_root = (
            REPO_ROOT / "results/rebot_real_ab_servo_safe_100hz_optimized/A"
        )
        artifact_path = run_root / "trajectory.csv"
        metadata_path = run_root / "trajectory.meta.yaml"
        report_path = run_root / "preview_report.yaml"
        mp4_path = run_root / "preview.mp4"
        self._require_local_evidence(
            artifact_path, metadata_path, report_path, mp4_path
        )
        artifact = load_replay_artifact(artifact_path, metadata_path)
        self.assertEqual(len(artifact.samples), 3001)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime_metadata_path = root / "trajectory.meta.yaml"
            runtime_metadata = dict(artifact.metadata)
            runtime_metadata.update(
                continuous_quintic_collision_precheck="PASS",
                continuous_quintic_collision_subdivisions_per_interval=10,
                continuous_quintic_collision_precheck_sample_count=30001,
            )
            runtime_metadata_path.write_text(
                yaml.safe_dump(runtime_metadata, sort_keys=False)
            )
            report_path = root / "preview_report.yaml"
            report_path.write_text(yaml.safe_dump({
                "schema_version": "rebot_trajectory_preview_report_v2",
                "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
                "trajectory_sha256": artifact.sha256,
                "continuous_quintic_collision_precheck": "PASS",
                "preview_status": "PASS",
            }, sort_keys=False))
            acceptance = root / "acceptance.yaml"
            acceptance.write_text(
                yaml.safe_dump({
                    "schema_version": "rebot_trajectory_preview_acceptance_v2",
                    "trajectory_replay_mode": TRAJECTORY_REPLAY_MODE,
                    "trajectory_sha256": artifact.sha256,
                    "preview_report": str(report_path),
                    "preview_report_sha256": sha256_file(report_path),
                    "preview_mp4": str(mp4_path),
                    "preview_mp4_sha256": sha256_file(mp4_path),
                    "operator": "",
                    "review_date": None,
                    "accepted_for_hardware": False,
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
            config["trajectory_metadata"] = str(runtime_metadata_path)
            config["trajectory_preview_acceptance"] = str(acceptance)
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
            self.assertEqual(metadata["motion_status"], "completed")
            self.assertEqual(metadata["observed_sample_count"], 3001)
            self.assertEqual(metadata["dispatch_timing"]["catch_up_burst_count"], 0)
            self.assertEqual(
                metadata["dispatch_timing"][
                    "command_dispatch_timestamp_mismatch_count"
                ],
                0,
            )
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
