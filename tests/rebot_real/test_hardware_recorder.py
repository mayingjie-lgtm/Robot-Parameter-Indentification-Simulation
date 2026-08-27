from __future__ import annotations

import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.hardware_recorder import CSV_COLUMNS, HardwareExperimentRecorder, SCHEMA_VERSION
from rebot_real.runner import load_hardware_config
from rebot_real.state_capture import CaptureSample


class HardwareExperimentRecorderTest(unittest.TestCase):
    def _sample(self, *, torque_valid: bool = True) -> CaptureSample:
        return CaptureSample(
            sample_index=0,
            timestamp_host_rx_ns=100,
            timestamp_lower_ns=90,
            udp_sequence=1,
            udp_sequence_gap=0,
            q=(0.0, -1.0, -1.0, 0.0, 0.0, 0.0),
            qd=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
            effort_reported=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) if torque_valid else (math.nan,) * 6,
            feedback_valid=(True,) * 6,
            torque_valid=(torque_valid,) * 6,
            feedback_age_ms=(1.0,) * 6,
            robot_mode="idle",
            safety_state="ready",
            primary_fault_code=0,
            servo_active=False,
            servo_mode="disabled",
        )

    def _config(self, output: Path) -> dict:
        config = load_hardware_config(
            REPO_ROOT / "config" / "rebot_real_experiment.yaml",
            repo_root=REPO_ROOT,
        )
        config["output_csv"] = str(output)
        return config

    def test_recorder_schema_is_exact_and_excludes_untrusted_fields(self) -> None:
        forbidden = {"qdd", "tau_cmd", "current", "q_raw"}
        self.assertFalse(any(column in forbidden for column in CSV_COLUMNS))
        self.assertFalse(any(column.startswith("qdd") for column in CSV_COLUMNS))
        self.assertFalse(any(column.startswith("tau_cmd") for column in CSV_COLUMNS))
        self.assertEqual(CSV_COLUMNS[0], "sample_index")
        self.assertIn("q_cmd0", CSV_COLUMNS)
        self.assertIn("command_valid", CSV_COLUMNS)
        self.assertIn("control_mode", CSV_COLUMNS)

    def test_command_target_and_contract_fields_are_recorded_exactly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "experiment.csv"
            recorder = HardwareExperimentRecorder(
                output,
                config=self._config(output),
                repo_root=REPO_ROOT,
                backend="rebot_sdk_mock",
            )
            target = [0.1, -1.1, -1.2, 0.2, -0.3, 0.4]
            recorder.record(
                self._sample(),
                q_cmd=target,
                timestamp_host_command_ns=1234,
                servo_sequence=7,
                command_valid=True,
                control_mode="servo_hold",
            )
            recorder.close()
            with output.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(int(rows[0]["timestamp_host_command_ns"]), 1234)
            self.assertEqual(int(rows[0]["servo_sequence"]), 7)
            self.assertEqual([float(rows[0][f"q_cmd{i}"]) for i in range(6)], target)
            self.assertEqual(rows[0]["command_valid"], "1")

    def test_state_only_uses_nan_q_cmd_and_no_fake_command_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "state.csv"
            recorder = HardwareExperimentRecorder(
                output,
                config=self._config(output),
                repo_root=REPO_ROOT,
                backend="rebot_sdk_mock",
            )
            recorder.record(
                self._sample(),
                q_cmd=None,
                timestamp_host_command_ns=None,
                servo_sequence=None,
                command_valid=False,
                control_mode="state_only",
            )
            recorder.close()
            with output.open("r", encoding="utf-8", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["timestamp_host_command_ns"], "")
            self.assertEqual(row["servo_sequence"], "")
            self.assertTrue(all(math.isnan(float(row[f"q_cmd{i}"])) for i in range(6)))
            self.assertEqual(row["command_valid"], "0")

    def test_metadata_declares_unavailable_signals_and_pending_hardware_acceptance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "experiment.csv"
            recorder = HardwareExperimentRecorder(
                output,
                config=self._config(output),
                repo_root=REPO_ROOT,
                backend="rebot_sdk_mock",
            )
            metadata = recorder.close()
            self.assertEqual(metadata["schema_version"], SCHEMA_VERSION)
            self.assertFalse(metadata["hardware_timestamp_available"])
            self.assertFalse(metadata["q_raw_available"])
            self.assertFalse(metadata["current_available"])
            self.assertFalse(metadata["tau_cmd_available"])
            self.assertFalse(metadata["feedback_sequence_supported"])
            self.assertEqual(metadata["hardware_acceptance"]["joint_mapping_hardware_verification"], "PENDING")
            self.assertEqual(metadata["hardware_acceptance"]["hardware_state_only_acceptance"], "PENDING")
            self.assertEqual(metadata["hardware_acceptance"]["servo_hold_hardware_acceptance"], "PENDING")
            self.assertEqual(metadata["hardware_acceptance"]["excitation_hardware_acceptance"], "PENDING")
            self.assertEqual(
                metadata["servo_command_fields"],
                ["servo_sequence", "host_timestamp_ns", "target_position_rad[6]"],
            )


if __name__ == "__main__":
    unittest.main()
