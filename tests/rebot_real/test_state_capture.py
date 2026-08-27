from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.recorder import (
    CSV_COLUMNS,
    analyze_capture,
    build_metadata,
    capture_udp_state,
    sample_to_row,
)
from rebot_real.state_capture import map_sdk_state


@dataclass
class MockState:
    sequence: int = 10
    monotonic_time_ns: int = 1_000_000
    position_rad: tuple[float, ...] = (0.1, -0.2, -0.3, 0.4, 0.5, -0.6)
    velocity_rad_s: tuple[float, ...] = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    torque_nm: tuple[float, ...] = (0.5, 0.4, 0.3, 0.2, 0.1, -0.1)
    feedback_valid: tuple[bool, ...] = (True,) * 6
    torque_valid: tuple[bool, ...] = (True,) * 6
    feedback_age_ms: tuple[float, ...] = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    robot_mode: str = "disabled"
    safety_state: str = "disabled"
    primary_fault_code: int = 0
    servo_active: bool = False
    servo_mode: str = "disabled"


def make_sample(
    state: MockState | None = None,
    *,
    index: int = 0,
    host_ns: int = 2_000_000,
    previous_sequence: int | None = None,
):
    return map_sdk_state(
        state or MockState(),
        sample_index=index,
        timestamp_host_rx_ns=host_ns,
        previous_sequence=previous_sequence,
    )


def write_samples(path: Path, samples) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample_to_row(sample))


class RebotStateCaptureTest(unittest.TestCase):
    def test_state_mapping(self) -> None:
        sample = make_sample()
        self.assertEqual(sample.q, MockState.position_rad)
        self.assertEqual(sample.qd, MockState.velocity_rad_s)
        self.assertEqual(sample.effort_reported, MockState.torque_nm)
        self.assertEqual(sample.robot_mode, "disabled")
        self.assertEqual(sample.safety_state, "disabled")
        self.assertEqual(sample.udp_sequence, 10)

    def test_six_dof_shape_rejected(self) -> None:
        state = MockState(position_rad=(0.0,) * 5)
        with self.assertRaisesRegex(ValueError, "exactly 6"):
            make_sample(state)

    def test_sequence_increment_has_no_skipped_lower_state(self) -> None:
        sample = make_sample(MockState(sequence=11), previous_sequence=10)
        self.assertEqual(sample.udp_sequence_gap, 0)

    def test_sequence_gap_records_skipped_lower_states(self) -> None:
        sample = make_sample(MockState(sequence=13), previous_sequence=10)
        self.assertEqual(sample.udp_sequence_gap, 2)

    def test_duplicate_and_out_of_order_are_reported(self) -> None:
        samples = [
            make_sample(MockState(sequence=10), index=0, host_ns=1_000_000),
            make_sample(MockState(sequence=10), index=1, host_ns=2_000_000, previous_sequence=10),
            make_sample(MockState(sequence=9), index=2, host_ns=3_000_000, previous_sequence=10),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, samples)
            summary = analyze_capture(path)
        self.assertEqual(summary["udp_duplicate_or_out_of_order_count"], 2)
        self.assertEqual(summary["udp_missing_snapshot_count"], None)

    def test_lower_timestamp_regression_is_detected(self) -> None:
        samples = [
            make_sample(MockState(monotonic_time_ns=200), index=0, host_ns=1_000_000),
            make_sample(
                MockState(sequence=11, monotonic_time_ns=199),
                index=1,
                host_ns=2_000_000,
                previous_sequence=10,
            ),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, samples)
            summary = analyze_capture(path)
        self.assertFalse(summary["timestamp_lower_monotonic"])

    def test_feedback_invalid_becomes_nan_with_validity_flag(self) -> None:
        state = MockState(feedback_valid=(False, True, True, True, True, True))
        sample = make_sample(state)
        self.assertFalse(sample.feedback_valid[0])
        self.assertTrue(math.isnan(sample.q[0]))
        self.assertTrue(math.isnan(sample.qd[0]))
        self.assertTrue(math.isnan(sample.feedback_age_ms[0]))
        self.assertTrue(math.isnan(sample.effort_reported[0]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, [sample])
            summary = analyze_capture(path)
        self.assertEqual(summary["non_finite_count"], 0)

    def test_torque_invalid_becomes_nan_without_invalidating_q(self) -> None:
        state = MockState(torque_valid=(False, True, True, True, True, True))
        sample = make_sample(state)
        self.assertTrue(sample.feedback_valid[0])
        self.assertFalse(sample.torque_valid[0])
        self.assertEqual(sample.q[0], state.position_rad[0])
        self.assertTrue(math.isnan(sample.effort_reported[0]))

    def test_nan_and_inf_on_valid_channels_count_as_non_finite(self) -> None:
        state = MockState(
            position_rad=(math.nan, -0.2, -0.3, 0.4, 0.5, -0.6),
            torque_nm=(math.inf, 0.4, 0.3, 0.2, 0.1, -0.1),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, [make_sample(state)])
            summary = analyze_capture(path)
        self.assertEqual(summary["non_finite_count"], 2)

    def test_metadata_declares_unavailable_signals(self) -> None:
        metadata = build_metadata(
            repo_root=REPO_ROOT,
            sdk_root="/tmp/mock_sdk",
            requested_duration_s=2.0,
            observed_sample_count=20,
            git_commit="deadbeef",
            sdk_version="0.1.0",
        )
        self.assertFalse(metadata["hardware_timestamp_available"])
        self.assertFalse(metadata["q_raw_available"])
        self.assertFalse(metadata["current_available"])
        self.assertFalse(metadata["tau_cmd_available"])
        self.assertFalse(metadata["feedback_sequence_supported"])
        self.assertEqual(metadata["joint_mapping_status"], "unverified")
        self.assertEqual(metadata["j1_convention"], "UNRESOLVED")
        self.assertEqual(metadata["capture_mode"], "state_only")

    def test_csv_schema_is_exact_and_contains_no_qdd_or_unavailable_signals(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, [make_sample()])
            with path.open("r", encoding="utf-8", newline="") as stream:
                header = next(csv.reader(stream))
        self.assertEqual(header, list(CSV_COLUMNS))
        self.assertFalse(any(column.startswith("qdd") for column in header))
        self.assertFalse(any(column.startswith("current") for column in header))
        self.assertFalse(any(column.startswith("q_raw") for column in header))
        self.assertFalse(any(column.startswith("tau_cmd") for column in header))

    def test_summary_statistics(self) -> None:
        first = MockState(
            sequence=10,
            monotonic_time_ns=10,
            position_rad=(1.0,) * 6,
            velocity_rad_s=(2.0,) * 6,
            torque_nm=(3.0,) * 6,
            feedback_age_ms=(4.0,) * 6,
        )
        second = MockState(
            sequence=20,
            monotonic_time_ns=20,
            position_rad=(2.0,) * 6,
            velocity_rad_s=(4.0,) * 6,
            torque_nm=(5.0,) * 6,
            feedback_age_ms=(6.0,) * 6,
        )
        samples = [
            make_sample(first, index=0, host_ns=1_000_000_000),
            make_sample(second, index=1, host_ns=2_000_000_000, previous_sequence=10),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            write_samples(path, samples)
            summary = analyze_capture(path)
        self.assertEqual(summary["sample_count"], 2)
        self.assertAlmostEqual(summary["duration"], 1.0)
        self.assertAlmostEqual(summary["observed_udp_rate_hz"], 1.0)
        self.assertTrue(summary["timestamp_host_monotonic"])
        self.assertTrue(summary["timestamp_lower_monotonic"])
        self.assertEqual(summary["udp_sequence_gap_count"], 1)
        j1 = summary["joints"]["J1"]
        self.assertEqual(j1["q_min"], 1.0)
        self.assertEqual(j1["q_max"], 2.0)
        self.assertEqual(j1["qd_min"], 2.0)
        self.assertEqual(j1["qd_max"], 4.0)
        self.assertEqual(j1["effort_reported_mean"], 4.0)
        self.assertEqual(j1["effort_reported_std"], 1.0)
        self.assertEqual(j1["mean_feedback_age_ms"], 5.0)
        self.assertEqual(j1["max_feedback_age_ms"], 6.0)

    def test_mock_subscriber_end_to_end_capture_and_analysis(self) -> None:
        class FakeSubscriber:
            def __init__(self) -> None:
                self.samples = [
                    (1_000_000_000, MockState(sequence=10, monotonic_time_ns=100)),
                    (1_010_000_000, MockState(sequence=20, monotonic_time_ns=110)),
                ]

            def receive(self):
                return self.samples.pop(0) if self.samples else None

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sdk_root = root / "sdk"
            package = sdk_root / "upper" / "python" / "wlsea_arm_sdk"
            package.mkdir(parents=True)
            (package / "__init__.py").write_text('__version__ = "mock-1"\n', encoding="utf-8")
            csv_path = root / "capture.csv"
            meta_path = root / "capture.meta.yaml"
            metadata = capture_udp_state(
                FakeSubscriber(),
                duration_s=0.001,
                csv_path=csv_path,
                metadata_path=meta_path,
                repo_root=REPO_ROOT,
                sdk_root=sdk_root,
            )
            summary = analyze_capture(csv_path)
            meta_exists = meta_path.is_file()
        self.assertEqual(metadata["observed_sample_count"], 2)
        self.assertEqual(metadata["sdk_version"], "mock-1")
        self.assertTrue(meta_exists)
        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["joints"]["J1"]["effort_reported_mean"], 0.5)

    def test_state_only_transport_has_no_control_channel_calls(self) -> None:
        source_paths = [
            REPO_ROOT / "src" / "rebot_real" / "sdk_adapter.py",
            REPO_ROOT / "src" / "rebot_real" / "recorder.py",
            REPO_ROOT / "scripts" / "capture_rebot_state.py",
        ]
        text = "\n".join(path.read_text(encoding="utf-8") for path in source_paths)
        for forbidden in (
            "socket.create_connection(",
            "ArmClient(",
            ".enable(",
            ".disable(",
            ".configure_pvt(",
            ".movej(",
            ".enter_servo(",
            ".servo_joint(",
            ".exit_servo(",
            ".gripper_position(",
        ):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
