# reBot-DM Hardware State Data Contract

Last updated: 2026-09-10
Scope: reBot-DM J1-J6 public state/data semantics used by real-hardware identification
Status: real A/B acquisition has been completed; `effort_reported` physical torque calibration remains unresolved

## 1. Purpose and boundary

This contract defines the first raw hardware-state dataset that may later enter reBot-DM
system-identification preprocessing. It deliberately does **not** connect the SDK to
`ExperimentBackend`, does not reuse the simulation CSV schema, and does not generate
acceleration online.

The allowed data flow is:

```text
reBot lower public JointState UDP
        -> UDP-only subscriber
        -> source-semantic mapping
        -> capture.csv + capture.meta.yaml
        -> offline acceptance analyzer
```

The following paths are out of scope:

```text
SDK -> ExperimentBackend
SDK -> ForceController
SDK -> motor command
hardware CSV -> rebot_trajectory_renderer
```

No component in this path may call `enable`, `disable`, `configure_pvt`, `movej`,
`enter_servo`, `servo_joint`, `exit_servo`, or a gripper command.

## 2. Audited SDK source of truth

The current accepted external SDK checkout is audited read-only at:

```text
/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904
```

Earlier handoff-package observations remain historical background only; current code/runtime behavior in the accepted checkout takes precedence.

Primary files checked:

- `README.md`
- `README_SIMULATION_HANDOFF_ZH.md`
- `PACKAGE_MANIFEST.md`
- `docs/TCP_UDP_CONTROL_PROTOCOL_ZH.md`
- `docs/movej_chain.md`
- `config/damiao_motors.csv`
- `config/safety.json`
- `config/joint_limits.yaml`
- `shared/cpp/include/wlsea_arm_sdk/protocol.hpp`
- `lower/cpp/include/wlsea_arm_sdk/lower/arm_driver.hpp`
- `lower/cpp/include/wlsea_arm_sdk/lower/damiao_canfd_driver.hpp`
- `lower/cpp/src/damiao_protocol.cpp`
- `lower/cpp/src/damiao_canfd_driver.cpp`
- `lower/cpp/src/arm_controller.cpp`
- `lower/cpp/src/lower_server.cpp`
- `upper/python/wlsea_arm_sdk/client.py`
- `upper/python/wlsea_arm_sdk/protocol.py`

Actual source code takes precedence over prose documentation where they disagree.

The handoff package reports Python SDK version `0.1.0` and protocol version `1`.
`PACKAGE_MANIFEST.md` explicitly excludes the vendor DM Device SDK and
`libdm_device.so`; no `dmcan.h` or `libdm_device.so` is present in this handoff.
Therefore direct USB2CANFD execution on this workstation is not an offline acceptance
gate.

## 3. SDK architecture and state source

The audited architecture is:

```text
upper ArmClient / protocol
      TCP commands + UDP feedback
                |
                v
lower LowerServer
      -> ArmController / SafetySupervisor / ServoCore
      -> ArmDriver
      -> DamiaoCanFdDriver
      -> USB2CANFD
      -> J1-J6
```

Identification must stay on the upper/public state boundary. It must not bypass the SDK
safety layer to read the vendor CAN transport directly.

The public state contains J1-J6:

```text
position_rad
velocity_rad_s
torque_nm
feedback_valid
torque_valid
feedback_age_ms
robot_mode
safety_state
primary_fault_code
servo_active
servo_mode
```

The Damiao driver maps decoded motor feedback to joint-side quantities as:

```text
joint_position = direction * (motor_position - zero_offset) / gear_ratio
joint_velocity = direction * motor_velocity / gear_ratio
joint_torque   = direction * motor_torque * gear_ratio
```

The public `q`, `qd`, and reported effort are therefore joint-side SDK quantities.

## 4. SDK -> identification field mapping

| Identification field | SDK/public source | Semantics |
|---|---|---|
| `timestamp_host_rx_ns` | recorder `time.monotonic_ns()` immediately after `recvfrom` | recorder-host receive timestamp |
| `timestamp_lower_ns` | `JointState.monotonic_time_ns` | lower-host steady/monotonic time associated with latest valid driver feedback; not motor hardware time |
| `q0..q5` | `JointState.position_rad` | joint-side calibrated position; valid only with `feedback_valid` |
| `qd0..qd5` | `JointState.velocity_rad_s` | joint-side calibrated velocity; valid only with `feedback_valid` |
| `effort_reported0..5` | `JointState.torque_nm` | joint-side DM feedback torque estimate; valid only with `feedback_valid && torque_valid` |
| `feedback_valid0..5` | `JointState.feedback_valid` | public feedback freshness/validity |
| `torque_valid0..5` | `JointState.torque_valid` | public torque feedback validity |
| `feedback_age_ms0..5` | `JointState.feedback_age_ms` | lower-host feedback age; only meaningful with valid feedback |
| `robot_mode` | `JointState.robot_mode` | lower robot mode |
| `safety_state` | `JointState.safety_state` | lower safety state |
| `primary_fault_code` | `JointState.primary_fault_code` | current primary safety fault code |
| `servo_active` | `JointState.servo_active` | observation only; recorder never enters servo |
| `servo_mode` | `JointState.servo_mode` | observation only |

`qdd` is intentionally absent. Acceleration must be produced later by an explicit
preprocessing stage that owns filtering, differentiation, time alignment, and resampling.

## 5. Stable CSV schema

Schema version:

```text
rebot_hardware_state_v1
```

Columns, in order:

```text
sample_index

timestamp_host_rx_ns
timestamp_lower_ns

udp_sequence
udp_sequence_gap

q0 q1 q2 q3 q4 q5
qd0 qd1 qd2 qd3 qd4 qd5

effort_reported0 effort_reported1 effort_reported2
effort_reported3 effort_reported4 effort_reported5

feedback_valid0 ... feedback_valid5
torque_valid0 ... torque_valid5
feedback_age_ms0 ... feedback_age_ms5

robot_mode
safety_state
primary_fault_code
servo_active
servo_mode
```

No `qdd`, `q_raw`, `current`, or `tau_cmd` column is added to this raw CSV.

### 5.1 Invalid public feedback policy

The SDK can retain/default numeric storage even when a validity flag is false. The raw
identification capture therefore uses an explicit normalization to prevent an invalid zero
or stale value from being misread as a physical measurement:

- if `feedback_valid[i] == false`, write `NaN` for `q[i]`, `qd[i]`, and
  `feedback_age_ms[i]`;
- if `feedback_valid[i] == false` or `torque_valid[i] == false`, write `NaN` for
  `effort_reported[i]`;
- always write the validity flags themselves.

This is a recorder representation rule, not an attempt to repair or interpolate the source.

## 6. Unavailable signals

Every metadata sidecar must declare:

```yaml
hardware_timestamp_available: false
q_raw_available: false
current_available: false
tau_cmd_available: false
feedback_sequence_supported: false
```

Interpretation:

- **hardware timestamp**: unavailable. `monotonic_time_ns` comes from lower-host
  `std::chrono::steady_clock`, not a motor/device clock.
- **q_raw**: unavailable through public `JointState`. The Damiao driver has internal
  position-frame handling, including private periodic-position state, but it is not a public
  J1-J6 raw encoder/count API and must not be bypassed.
- **current**: unavailable. `current_limit_normalized` is a PVT command/configuration limit,
  not measured motor current.
- **tau_cmd**: unavailable in state-only capture. The public torque feedback is not the
  command channel.
- **motor feedback sequence**: unavailable. Current DM PVT feedback has no verified frame
  sequence; `feedback_sequence_supported=false`.

Unavailable physical quantities are never represented as zero.

## 7. Torque semantics

`lower/cpp/src/damiao_protocol.cpp` decodes the DM feedback torque field using the
configured motor torque range. `DamiaoCanFdDriver::read_state()` then applies direction and
gear-ratio mapping before publishing it as `JointState.torque_nm`.

This proves the source and joint-side mapping, but it does **not** prove an independently
calibrated physical joint torque sensor. The SDK safety configuration also marks torque
protection as disabled pending vendor thresholds/calibration evidence.

Therefore the identification name is:

```text
effort_reported
```

and never:

```text
tau_measured
```

The state-only acceptance analyzer reports its statistics but never declares torque
calibration PASS.

## 8. Timestamp semantics

Two monotonic clocks are retained:

### `timestamp_lower_ns`

Source:

```text
SDK JointState.monotonic_time_ns
```

The Damiao receive callback records lower-host `monotonic_time_ns()` using
`std::chrono::steady_clock`; `read_state()` publishes the most recent valid receive time.
This is a lower-host timestamp and can repeat when no newer valid feedback has arrived.
It is not named `timestamp_device`.

### `timestamp_host_rx_ns`

Source:

```text
recorder time.monotonic_ns()
```

It is sampled immediately after the UDP `recvfrom()` call returns and before JSON decode.
It gives a separate recorder-host receive clock for later alignment/latency analysis.

Neither clock is converted to wall time in the raw capture.

## 9. Sequence and packet-loss semantics

The SDK prose documentation calls `JointState.sequence` a UDP snapshot sequence. Actual
source code is more specific:

```text
ArmController::update()     -> ++state_.sequence at lower control rate (~1 kHz)
LowerServer UDP publisher   -> copies latest state at feedback rate (~100 Hz)
```

Therefore the value carried in a UDP datagram is a **lower controller-state sequence**,
not a sequence incremented once per UDP send.

The CSV keeps the requested names for compatibility:

```text
udp_sequence
udp_sequence_gap
```

with these exact meanings:

- `udp_sequence`: raw public `JointState.sequence` carried by that received UDP datagram;
- `udp_sequence_gap`: `max(current_sequence - previous_sequence - 1, 0)`, i.e. skipped
  lower controller-state sequence values between two received datagrams.

Because a normal 100 Hz UDP stream samples a ~1 kHz lower state, a positive
`udp_sequence_gap` is expected even with zero network loss. It must **not** be converted to
missing UDP packets.

The analyzer therefore reports:

```text
udp_sequence_gap_count
udp_duplicate_or_out_of_order_count
udp_missing_snapshot_count: null
```

with a reason explaining that missing UDP snapshots are not inferable from the current
source sequence.

This is separate from lower motor-feedback loss telemetry. The SDK exposes estimated
expected/received/lost counters for 1 s / 10 s / cumulative windows, but also explicitly
sets `feedback_sequence_supported=false`; those estimates are intentionally not expanded
into the first raw identification schema.

## 10. Why the recorder is UDP-only

`ArmClient.connect()` itself opens sockets and does not send enable/PVT/MoveJ. However, the
lower server only publishes UDP while a TCP client session exists, and actual source code
runs:

```text
TCP session disconnect
-> ArmController::handle_client_disconnect()
-> disable(...)
```

unconditionally.

A TCP session is therefore not a strictly side-effect-free observation contract. The legacy UDP-only recorder deliberately does **not** create a TCP session and does not instantiate `ArmClient`; it binds only the public UDP state port and decodes the public protocol.

This creates an important boundary: with the current lower server, an independent UDP-only recorder may receive no data unless the lower software provides a side-effect-free public-state publication/subscription mode. The formal A/B experiment does not rely on that UDP-only recorder; it uses the audited `ArmClient`/`StateStore` path and records the resulting public state together with explicit control-session metadata.

## 11. Metadata sidecar

For every:

```text
capture.csv
```

write:

```text
capture.meta.yaml
```

Required metadata includes:

```yaml
schema_version: rebot_hardware_state_v1
robot: rebot_dm
backend: rebot_sdk_state_only
git_commit: <repository commit>
sdk_root: <external SDK root>
sdk_version: <SDK package version>
capture_mode: state_only
requested_duration_s: <seconds>
observed_sample_count: <count>

timestamp_lower_source: <explicit lower-host steady-clock description>
timestamp_host_rx_source: <recorder monotonic receive-clock description>
q_source: <public JointState.position_rad description>
qd_source: <public JointState.velocity_rad_s description>
effort_reported_source: <public JointState.torque_nm description>
effort_reported_semantics: <DM torque-estimate semantics>

hardware_timestamp_available: false
q_raw_available: false
current_available: false
tau_cmd_available: false
feedback_sequence_supported: false

joint_mapping_status: unverified
sdk_joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
canonical_joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
j1_convention: UNRESOLVED
```

## 12. Joint mapping audit

Canonical limits come from this repository's `rebot_dm/rebot_dm.xml`. SDK motor mapping
comes from the external handoff `config/damiao_motors.csv`.

| Joint | Canonical | SDK | Canonical min | Canonical max | SDK min | SDK max | direction | zero_offset rad | gear_ratio | status |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| J1 | `joint1` | `joint_1` | -2.8 | 2.8 | 0.0 | 6.283185307180 | 1 | -3.409197418052 | 1.0 | **UNRESOLVED** |
| J2 | `joint2` | `joint_2` | -3.14 | 0.0 | -3.14 | 0.0 | 1 | -0.006675688869 | 1.0 | limit-aligned; hardware convention still unverified |
| J3 | `joint3` | `joint_3` | -3.14 | 0.0 | -3.14 | 0.0 | 1 | -0.012016822080 | 1.0 | limit-aligned; hardware convention still unverified |
| J4 | `joint4` | `joint_4` | -1.87 | 1.57 | -1.87 | 1.57 | 1 | 0.081444639963 | 1.0 | limit-aligned; hardware convention still unverified |
| J5 | `joint5` | `joint_5` | -1.57 | 1.57 | -1.57 | 1.57 | 1 | 0.002480031205 | 1.0 | limit-aligned; hardware convention still unverified |
| J6 | `joint6` | `joint_6` | -3.14 | 3.14 | -3.14 | 3.14 | 1 | -4.694126923598 | 1.0 | limit-aligned; hardware convention still unverified |

J1 has additional conflicting SDK-side evidence:

- `config/joint_limits.yaml`: `[0, 2*pi]`, `UNVERIFIED_ON_HARDWARE`;
- `upper/robot_models/rebot_b601_dm/limits.yaml`: `[0, 2*pi]`;
- single-arm URDF `arm_joint1`: `[-2.8, 2.8]`;
- dual-arm URDF `left_joint1/right_joint1`: approximately
  `[-3.228859, 3.228859]`.

No current source proves which convention is the hardware identification convention.
Accordingly this phase must not:

```text
wrap_to_pi
q -= 2*pi
q += 2*pi
modify canonical URDF/MJCF
modify SDK motor CSV
```

The metadata remains `joint_mapping_status: unverified` until a hardware source of truth is
established.

## 13. Offline state-only acceptance analyzer

`scripts/analyze_rebot_state_capture.py` reports at least:

```text
sample_count
duration
observed_udp_rate_hz

timestamp_host_monotonic
timestamp_lower_monotonic

udp_sequence_gap_count
udp_missing_snapshot_count
udp_duplicate_or_out_of_order_count

non_finite_count
```

and for every J1-J6:

```text
feedback_invalid_count
torque_invalid_count
mean_feedback_age_ms
max_feedback_age_ms
q_min
q_max
qd_min
qd_max
effort_reported_min
effort_reported_max
effort_reported_mean
effort_reported_std
```

`timestamp_host_monotonic` is strict. `timestamp_lower_monotonic` is non-decreasing because
several UDP snapshots may legitimately carry the same latest lower feedback timestamp.

`non_finite_count` only counts non-finite values on channels whose corresponding validity
flag says the value should be usable. Intentional NaNs created for invalid feedback are
reported through the invalid counters rather than treated as extra corruption.

## 14. Usage

The recorder itself is intentionally simple:

```bash
python3 scripts/capture_rebot_state.py \
  --sdk-root /home/wlsea1/桌面/机械臂sdk/wlsea_arm_sdk_sim_handoff_20260826 \
  --duration 10 \
  --output /tmp/rebot_state_only/capture.csv
```

Analyze the result:

```bash
python3 scripts/analyze_rebot_state_capture.py \
  --csv /tmp/rebot_state_only/capture.csv \
  --output /tmp/rebot_state_only/capture.summary.yaml
```

The recorder never creates a TCP command session. With the current SDK lower-server
implementation, receiving zero UDP samples is therefore a possible and expected hardware
interface limitation. The capture CLI returns non-zero after writing the zero-sample CSV and
metadata, and explicitly instructs the operator **not** to create a TCP control session just
to make state-only capture work.

Future real-hardware topology remains:

```text
Orin
  lower server
  USB2CANFD
  robot

wlsea1
  UDP-only state recorder
```

but the lower/public-state publication contract must first support truly observation-only
state delivery.

## 15. Legacy UDP-only recorder boundary

The offline implementation can pass when:

- the source mapping and six-axis shape are tested;
- sequence increment, sequence gap, duplicate/out-of-order, and lower timestamp regression
  are tested;
- invalid feedback, invalid torque, NaN/Inf, metadata availability, CSV schema, and summary
  statistics are tested;
- static inspection proves the recorder path contains no motor-changing command call;
- prior simulation identification and renderer regressions remain green.

This UDP-only path is a diagnostic/data-source boundary and does not authorize motion by itself. Real A/B excitation authorization is defined by `REBOT_HARDWARE_CONTROL_CONTRACT.md` and the accepted frozen trajectory/preview evidence.

Current real A/B recordings confirm that the formal `ArmClient` path can capture the required public `q`, `qd`, `effort_reported`, validity, freshness, mode, fault and Servo-state fields through a complete excitation. They do **not** resolve the remaining interpretation boundary: `effort_reported` is still the SDK joint-side torque estimate, not an independently calibrated physical torque sensor measurement.
