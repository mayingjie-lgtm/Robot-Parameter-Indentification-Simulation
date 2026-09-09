from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.control_adapter import RebotControlAdapter, RebotControlError
from rebot_real.mock_client import MockArmClient


class RebotControlAdapterTest(unittest.TestCase):
    def _adapter(self, fake: MockArmClient, **overrides) -> RebotControlAdapter:
        values = {
            "sdk_root": "",
            "host": "127.0.0.1",
            "tcp_port": 5000,
            "udp_port": 5001,
            "joint_direction": [1.0] * 6,
            "joint_offset_rad": [0.0] * 6,
            "connect_timeout_s": 0.1,
            "command_timeout_s": 0.1,
            "state_timeout_s": 0.05,
            "client_factory": lambda **_: fake,
            "sleep_fn": lambda _: None,
        }
        values.update(overrides)
        return RebotControlAdapter(**values)

    def test_state_mapping_applies_explicit_direction_and_offset(self) -> None:
        fake = MockArmClient(
            position_rad=[1.0, -1.0, -1.2, 0.2, -0.3, 0.4],
            velocity_rad_s=[0.1, 0.2, -0.3, 0.4, -0.5, 0.6],
            torque_nm=[1, 2, 3, 4, 5, 6],
        )
        adapter = self._adapter(
            fake,
            joint_direction=[-1, 1, 1, 1, 1, 1],
            joint_offset_rad=[0.25, 0, 0, 0, 0, 0],
        )
        adapter.connect()
        state = adapter.read_state()
        self.assertAlmostEqual(state.q[0], -0.75)
        self.assertAlmostEqual(state.qd[0], -0.1)
        self.assertAlmostEqual(state.effort_reported[0], -1.0)
        self.assertEqual(state.q[1:], (-1.0, -1.2, 0.2, -0.3, 0.4))

    def test_servo_target_preserves_explicit_timestamp_sequence_and_inverse_mapping(self) -> None:
        fake = MockArmClient()
        adapter = self._adapter(
            fake,
            joint_direction=[-1, 1, 1, 1, 1, 1],
            joint_offset_rad=[0.2, 0, 0, 0, 0, 0],
            monotonic_ns_fn=lambda: 123456789,
        )
        adapter.connect()
        adapter.enable()
        adapter.enter_servo()
        timestamp, sequence = adapter.send_servo_target([0.0] * 6)
        self.assertEqual(timestamp, 123456789)
        self.assertEqual(sequence, 1)
        self.assertEqual(fake.servo_command_records[0][0], timestamp)
        self.assertEqual(fake.servo_command_records[0][1], sequence)
        self.assertAlmostEqual(fake.servo_targets[0][0], 0.2)
        self.assertEqual(fake.servo_targets[0][1:], (0.0, 0.0, 0.0, 0.0, 0.0))

    def test_servo_target_accepts_explicit_reference_timestamp(self) -> None:
        fake = MockArmClient()
        adapter = self._adapter(fake, monotonic_ns_fn=lambda: 111)
        adapter.connect()
        adapter.enable()
        adapter.enter_servo()
        timestamp, sequence = adapter.send_servo_target(
            [0.0] * 6,
            host_timestamp_ns=987654321,
        )
        self.assertEqual(timestamp, 987654321)
        self.assertEqual(sequence, 1)
        self.assertEqual(fake.servo_command_records[0][0], 987654321)

    def test_servo_target_rejects_non_six_dof_shape(self) -> None:
        fake = MockArmClient()
        adapter = self._adapter(fake)
        adapter.connect()
        adapter.enable()
        adapter.enter_servo()
        with self.assertRaisesRegex(ValueError, "exactly 6"):
            adapter.send_servo_target([0.0] * 5)
        self.assertNotIn("servo_joint", fake.calls)

    def test_configure_movej_pvt_uses_sdk_policy_surface(self) -> None:
        fake = MockArmClient()
        adapter = self._adapter(fake)
        adapter.connect()
        adapter.configure_movej_pvt()
        self.assertEqual(fake.calls, ["connect", "configure_pvt"])
        self.assertEqual(len(fake.pvt_configurations), 1)
        self.assertEqual(fake.pvt_configurations[0], fake.movej_pvt_policy)

    def test_movej_target_uses_canonical_inverse_mapping_with_j1_minus_pi_offset(self) -> None:
        fake = MockArmClient(follow_movej_targets=True)
        adapter = self._adapter(
            fake,
            joint_direction=[1, 1, 1, 1, 1, 1],
            joint_offset_rad=[-math.pi, 0, 0, 0, 0, 0],
        )
        adapter.connect()
        adapter.enable()
        adapter.movej_to(
            [0.0, -1.0, -1.0, 0.0, 0.0, -0.6],
            max_velocity_rad_s=[0.2] * 6,
            max_acceleration_rad_s2=[0.4] * 6,
            max_jerk_rad_s3=[5.0] * 6,
            timeout_s=10.0,
        )
        self.assertEqual(len(fake.movej_targets), 1)
        self.assertAlmostEqual(fake.movej_targets[0][0], math.pi)
        self.assertEqual(fake.movej_targets[0][1:], (-1.0, -1.0, 0.0, 0.0, -0.6))
        self.assertEqual(fake.servo_targets, [])

    def test_park_policy_maps_sdk_space_into_runner_canonical_space(self) -> None:
        fake = MockArmClient()
        adapter = self._adapter(
            fake,
            joint_direction=[1, 1, 1, 1, 1, 1],
            joint_offset_rad=[-math.pi, 0, 0, 0, 0, 0],
        )
        adapter.connect()
        policy = adapter.park_policy()
        self.assertEqual(policy["source"], "mock_movej_runtime_policy")
        self.assertAlmostEqual(policy["target_q"][0], 0.0)
        self.assertEqual(policy["target_q"][1:5], (0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(policy["target_q"][5], math.pi / 2)
        self.assertAlmostEqual(policy["max_velocity_rad_s"][0], math.radians(10.0))
        self.assertAlmostEqual(policy["position_tolerance_rad"], math.radians(1.0))

    def test_sdk_command_failure_is_normalized(self) -> None:
        fake = MockArmClient(fail_on={"enable"})
        adapter = self._adapter(fake)
        adapter.connect()
        with self.assertRaisesRegex(RebotControlError, "enable failed"):
            adapter.enable()

    def test_latest_state_transport_failure_is_normalized_without_waiting(self) -> None:
        fake = MockArmClient(disconnect_after_state_reads=0)
        adapter = self._adapter(fake)
        adapter.connect()
        with self.assertRaisesRegex(RebotControlError, "latest state snapshot failed"):
            _ = adapter.latest_state

    def test_configure_pvt_failure_is_normalized(self) -> None:
        fake = MockArmClient(fail_on={"configure_pvt"})
        adapter = self._adapter(fake)
        adapter.connect()
        with self.assertRaisesRegex(RebotControlError, "configure_pvt failed"):
            adapter.configure_movej_pvt()

    def test_movej_failure_is_normalized(self) -> None:
        fake = MockArmClient(fail_on={"movej"})
        adapter = self._adapter(fake)
        adapter.connect()
        adapter.enable()
        with self.assertRaisesRegex(RebotControlError, "movej failed"):
            adapter.movej_to(
                [0.0] * 6,
                max_velocity_rad_s=[0.2] * 6,
                max_acceleration_rad_s2=[0.4] * 6,
                max_jerk_rad_s3=[5.0] * 6,
                timeout_s=10.0,
            )


if __name__ == "__main__":
    unittest.main()
