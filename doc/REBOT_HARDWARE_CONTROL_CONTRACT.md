# reBot-DM Hardware Control Contract

Date: 2026-08-27
Scope: offline implementation of reBot-DM hardware control integration
Status: software/Mock acceptance only; real hardware acceptance is pending

## 1. Boundary

This path is intentionally separate from the existing C++ `ExperimentBackend` and legacy
`ExperimentRecorder` because their command/data semantics do not match the audited reBot
Servo interface.

```text
config/rebot_real_experiment.yaml
            |
            v
RebotControlAdapter
            |
            v
external wlsea_arm_sdk ArmClient
      +-----+-----+
      |           |
      v           v
Servo q_target   JointState
      |           |
      +-----+-----+
            v
RebotHardwareRunner
  +---------+---------+
  |         |         |
state_only servo_hold excitation
                      (trajectory source pending)
            |
            v
rebot_hardware_experiment_v1 CSV
            + metadata
```

The Phase 6A raw state contract remains independent:

```text
public UDP -> UdpStateSubscriber -> rebot_hardware_state_v1
```

Phase 6B does not change that raw capture schema or its UDP-only semantics.

## 2. Audited SDK source of truth

The external SDK was inspected read-only at:

```text
/home/wlsea1/桌面/机械臂sdk/wlsea_arm_sdk_sim_handoff_20260826
```

Primary source files:

- `upper/python/wlsea_arm_sdk/client.py`
- `upper/python/wlsea_arm_sdk/protocol.py`
- `upper/python/wlsea_arm_sdk/state_store.py`
- `lower/cpp/src/arm_controller.cpp`
- `lower/cpp/src/lower_server.cpp`
- `config/safety.json`
- `config/damiao_motors.csv`

Actual code remains authoritative over prose documentation.

## 3. SDK control API mapping

The adapter maps only public API that exists in the audited SDK:

| Adapter operation | ArmClient API | Meaning |
|---|---|---|
| `connect()` | `ArmClient.connect()` | open UDP feedback + TCP command session |
| `read_state()` | `ArmClient.state_store/latest_state` | read public `JointState` |
| `enable()` | `ArmClient.enable()` | motor-changing command |
| `enter_servo()` | `ArmClient.enter_servo()` | claim lower Servo ownership |
| `send_servo_target(q)` | `ArmClient.servo_joint(...)` | send position-only Servo target |
| `exit_servo()` | `ArmClient.exit_servo()` | release Servo ownership |
| `stop()` | `ArmClient.stop()` | best-effort lower stop |
| `disable()` | `ArmClient.disable()` | disable motors |
| `close()` | `ArmClient.close()` | close upper sockets |

The adapter does not call `MoveJ`, `configure_pvt`, or gripper APIs.

## 4. Servo command semantics

The audited command is exactly:

```text
servo_sequence
host_timestamp_ns
target_position_rad[6]
```

Therefore:

```text
Servo sends q_target only
```

It does not send:

```text
qd command
kp
kd
tau_cmd
```

`tau_cmd_available` is therefore always `false` for this contract. The existence of
`force_node::ControlCommand.torque` in the C++ simulation/legacy path is not evidence of a
real reBot torque command.

`RebotControlAdapter` supplies an explicit monotonically increasing Servo sequence and an
upper-host `time.monotonic_ns()` timestamp to `ArmClient.servo_joint`. The audited Python
`ArmClient.servo_joint` sends the TCP payload and returns `(request_id, sequence)` without
waiting for a synchronous `CommandReply`; therefore the adapter does not invent a lower-side
acceptance result. The runner re-checks subsequent state/fault/Servo status, while the lower
Servo safety layer remains authoritative. There is no implicit second command contract.

## 5. Coordinate mapping contract

Runner coordinates are configured explicitly:

```text
q_runner = joint_direction * q_sdk + joint_offset_rad
qd_runner = joint_direction * qd_sdk
effort_runner = joint_direction * effort_sdk
```

The inverse mapping is used only when a verified motion target is passed to the SDK:

```text
q_sdk_cmd = (q_runner_cmd - joint_offset_rad) / joint_direction
```

This capability does not resolve the hardware convention by itself. Default configuration
remains:

```yaml
joint_mapping_verified: false
j1_convention: UNRESOLVED
```

The known J1 conflict is preserved:

```text
canonical model: [-2.8, 2.8]
current SDK deployment config: [0, 2*pi]
```

No automatic wrapping, `+/- 2*pi`, canonical URDF/MJCF edit, or SDK motor CSV edit is
performed.

## 6. Hardware authorization gates

Real `ArmClient` creation requires:

```text
allow_hardware == true
```

Every motor-changing runner mode additionally requires all of:

```text
allow_motion == true
joint_mapping_verified == true
j1_convention != UNRESOLVED
```

The explicit `--mock` path is not a real hardware session and may run while
`allow_hardware=false`; motion-like Mock lifecycle still requires `allow_motion`, verified
mapping, and resolved J1 so that the software gates themselves are exercised.

The upper layer implements only fail-fast checks:

- exact 6-DOF shape;
- finite values;
- explicit mapping gate;
- configured position limits;
- velocity-derived per-cycle command delta;
- feedback validity;
- feedback age;
- primary fault and lower safety-state rejection;
- Servo-active state check;
- state/command communication timeout.

It does not duplicate lower Servo ownership, watchdog, sequence/timestamp validation,
position/velocity/acceleration limits, protective stop, or fault supervision. The lower SDK
remains the authoritative safety layer.

## 7. TCP observation caveat

The audited `ArmClient.connect()` does not itself issue enable/PVT/MoveJ. However the lower
server publishes UDP during a TCP client session, and on TCP disconnect calls:

```text
ArmController::handle_client_disconnect()
  -> disable(...)
```

Accordingly Phase 6B `state_only` means:

```text
no upper enable / enter_servo / servo_joint / MoveJ / configure_pvt / gripper call
```

It does **not** mean a TCP session is a completely side-effect-free hardware observation
contract. The Phase 6A UDP-only recorder remains the preferred raw observation contract and
is not replaced by this runner.

## 8. Control modes

### `state_only`

Implemented offline and runnable with `--mock`.

```text
connect
  -> wait new/fresh state
  -> validate
  -> record
  -> close
```

No upper motor-changing command is reachable from `_run_state_only`.

Real-hardware state acceptance remains `PENDING`.

### `servo_hold`

The complete software lifecycle is implemented and Mock-tested:

```text
connect
  -> read fresh state
  -> enable
  -> re-read fresh current q
  -> enter_servo
  -> verify Servo-active state
  -> repeatedly send hold q_target
  -> exit_servo
  -> disable
  -> close
```

The hold target is the fresh measured current `q` read immediately before Servo entry. It is
never a hard-coded home pose.

On an exception after Servo ownership is acquired, cleanup is best-effort and independent:

```text
exit_servo
  -> stop
  -> disable
  -> close
```

A failure in one cleanup action does not skip the remaining actions.

Real Servo hold remains `PENDING` and is denied by the default configuration.

### `excitation`

The mode and authorization boundary exist, but trajectory source integration is explicitly
pending.

The trusted reBot Fourier source of truth is the existing C++ `ForceController` /
`trajectory::FourierTrajectory`. Phase 6B does not copy that mathematics into a second
Python implementation. Until a small semantics-preserving replay/provider interface is
available, `excitation` fails before a hardware session is created with:

```text
trajectory source integration remains pending
```

Real Fourier excitation is `NOT STARTED`.

## 9. Hardware experiment CSV

Schema:

```text
rebot_hardware_experiment_v1
```

Columns:

```text
sample_index

timestamp_host_rx_ns
timestamp_lower_ns
timestamp_host_command_ns
servo_sequence

q0..q5
qd0..qd5
effort_reported0..5
q_cmd0..q_cmd5

feedback_valid0..5
torque_valid0..5
feedback_age_ms0..5

robot_mode
safety_state
primary_fault_code
servo_active
servo_mode

command_valid
control_mode
```

For `state_only`, command timestamp/sequence are empty and `q_cmd` is `NaN`; zeros are not
invented.

The schema intentionally does not contain:

```text
qdd
tau_cmd
current
q_raw
```

`qdd` remains an offline preprocessing responsibility.

## 10. Metadata

Every experiment writes `<csv>.meta.yaml` including:

- schema, robot, backend, git commit;
- SDK root/version and host/ports;
- control mode/rate;
- `allow_hardware` / `allow_motion`;
- mapping status, direction, offset, J1 convention;
- configured joint/command/freshness thresholds;
- timestamp/q/qd/effort/q_cmd source descriptions;
- explicit Servo command fields;
- relevant `config/safety.json` snapshot when the configured SDK is present;
- trajectory source/hash status;
- pending real-hardware acceptance status.

Unavailable-signal flags are fixed to:

```yaml
hardware_timestamp_available: false
q_raw_available: false
current_available: false
tau_cmd_available: false
feedback_sequence_supported: false
```

The Phase 6B `timestamp_host_rx_ns` source is the current SDK `StateStore` upper-host receive
snapshot (`ActualJointState.received_monotonic_ns`), which is recorded when the decoded UDP
state is published into the store. This is distinct from Phase 6A's timestamp sampled
immediately after raw `recvfrom`.

## 11. Offline smoke

Safe default Mock smoke:

```bash
python3 scripts/run_rebot_hardware.py \
  --config config/rebot_real_experiment.yaml \
  --mock
```

The default YAML has:

```yaml
control_mode: state_only
allow_hardware: false
allow_motion: false
joint_mapping_verified: false
j1_convention: UNRESOLVED
```

so it cannot create a real `ArmClient` session or move a robot by accident.

## 12. Real-machine values that must be filled/verified later

Before any real motor-changing acceptance, verify or update at least:

```text
sdk_root
host
tcp_port
udp_port
joint_mapping_verified
j1_convention
joint_direction
joint_offset_rad
joint_position_min_rad
joint_position_max_rad
maximum_command_velocity_rad_s
maximum_feedback_age_ms
maximum_disabled_feedback_age_ms
control_rate_hz
```

`maximum_disabled_feedback_age_ms` 仅适用于失能状态观测和 Servo 使能前检查；省略或
设为 `null` 时回退到 `maximum_feedback_age_ms`，保持旧配置行为。使能完成后以及整个
Servo 生命周期始终使用更严格的 `maximum_feedback_age_ms`。

Recommended real acceptance order remains:

```text
state observation
  -> hardware mapping/J1 confirmation
  -> Servo hold
  -> very small bounded motion
  -> excitation only after trajectory-source integration and acceptance
```

## 13. Current status

```text
simulation identification                 = PASS
reBot state capture implementation        = PASS
reBot hardware control adapter offline    = PASS
reBot hardware runner offline             = PASS
Mock Servo lifecycle                      = PASS
Mock fault injection                      = PASS

real hardware state acceptance            = PENDING
joint mapping verification                = PENDING
J1 convention                             = UNRESOLVED
real Servo hold                           = PENDING
real small-motion validation              = NOT STARTED
real Fourier excitation                   = NOT STARTED
C++ ExperimentBackend integration         = DEFERRED
```

`ExperimentBackend` remains deferred because both command semantics and recorder semantics
mismatch the audited reBot hardware path:

```text
ControlCommand(position, velocity, kp, kd, torque)
              !=
ServoJoint(target_position_rad only)
```

and the existing non-simulation C++ recorder still assigns finite-difference `qdd` and
`command.torque` to a legacy ambiguous schema. Mock PASS is not real-hardware validation.
