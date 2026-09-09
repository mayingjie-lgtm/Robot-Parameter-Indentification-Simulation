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
                      (frozen replay artifact)
            |
            v
rebot_hardware_experiment_v2 CSV
            + metadata
```

The Phase 6A raw state contract remains independent:

```text
public UDP -> UdpStateSubscriber -> rebot_hardware_state_v1
```

Phase 6B does not change that raw capture schema or its UDP-only semantics.

2026-09-07 addition: `joint_jog` implements a bounded, single-joint quintic excursion
and return through the same Servo adapter. It is software/Mock tested; hardware
acceptance remains pending. It neither homes the arm nor unlocks `excitation`.

## 2. Audited SDK source of truth

The current external SDK was inspected read-only at:

```text
/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904
```

Primary source files:

- `src/wlsea_arm/robot.py`
- `src/wlsea_arm_sdk/client.py`
- `src/wlsea_arm_sdk/protocol.py`
- `src/wlsea_arm_sdk/movej_runtime.py`
- `src/wlsea_arm_sdk/state_store.py`
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
| `latest_state` | `ArmClient.state_store.latest` | reuse the newest public state without waiting for a new UDP frame |
| `configure_movej_pvt()` | `ArmClient.configure_pvt(...)` | apply the SDK `movej_runtime` PVT policy before preposition |
| `enable()` | `ArmClient.enable()` | motor-changing command |
| `movej_to(q)` | `ArmClient.movej(...)` | synchronous SDK-native preposition to the frozen artifact start |
| `enter_servo()` | `ArmClient.enter_servo()` | claim lower Servo ownership |
| `send_servo_target(q)` | `ArmClient.servo_joint(...)` | send position-only Servo target |
| `exit_servo()` | `ArmClient.exit_servo()` | release Servo ownership |
| `stop()` | `ArmClient.stop()` | best-effort lower stop |
| `disable()` | `ArmClient.disable()` | disable motors |
| `close()` | `ArmClient.close()` | close upper sockets |

`state_only` and `servo_hold` do not call MoveJ/PVT. `excitation` may call the two new adapter operations only for the preposition gate; gripper APIs remain outside this contract. Production PVT values are loaded from the configured external SDK's `wlsea_arm_sdk.movej_runtime` constants rather than copied into this repository.

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

`joint_jog` additionally requires `joint_mapping_scope: joint_jog` (old configs default
to `servo_hold_only`) and a nonempty `joint_jog.authorization_reference` identifying
the bounded-motion test record. The historical `PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907`
hold-only convention is rejected for jog; a Mock convention is rejected on a real backend.
These strings record operator evidence, not automatic proof of hardware certification.
All numeric jog parameters must be explicit; `max_samples` must be null and `duration_s`
is a timeout budget exceeding the planned round trip. Before enable, the runner checks
disabled state and the full excursion envelope with a margin. After enable it rechecks
the envelope and enable-time drift before claiming Servo ownership.

The jog path is evaluated against actual monotonic elapsed time, with zero endpoint
velocity/acceleration quintic ramps. Planned peak velocity/acceleration, measured-state
command delta, consecutive-target delta, timed command velocity/acceleration, other-axis
drift, stationary-window position/velocity and command gap are checked. A fault aborts
without an automatic return. Jog enable/Servo intent is tracked before sending requests
so uncertain ACK failures still attempt cleanup; KeyboardInterrupt also invokes cleanup.

Jog preserves the CSV schema, recording alternating pre-command and post-command state
rows; post-command observations have `command_valid=0` and no `q_cmd`. `joint_jog_result`
in metadata contains completion/abort status, phase row ranges, stationary snapshot
statistics, excursion and return error. These are encoder-space observations, not proof
of physical joint mapping or backlash. `physical_mapping_verified_by_test` and
`identification_ready` remain false. Existing CSV/metadata output paths are refused for jog.
The complete operator procedure and parameter meanings are in the runbook §6.2.

The upper layer implements only fail-fast checks:

- exact 6-DOF shape;
- finite values;
- explicit mapping gate;
- configured position limits;
- consecutive-target velocity-derived delta (`maximum_command_velocity_rad_s / control_rate_hz`);
- consecutive-target lower ServoCore fixed delta (`maximum_servo_target_delta_rad`);
- target-to-measured tracking error (`maximum_tracking_error_rad`, never divided by rate): runtime gate for hold/jog, but monitor-only quality evidence during `excitation`;
- feedback validity;
- feedback age;
- upper-host UDP snapshot age against `state_timeout_s`;
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

The trusted reBot Fourier source of truth remains the existing C++ `ForceController` /
`trajectory::FourierTrajectory`; the Python hardware path does not copy that mathematics.
`rebot_trajectory_exporter` freezes the accepted C++ trajectory into the
`rebot_replay_trajectory_v1` CSV/metadata artifact, and the runner replays each stored
`q_ref` sample exactly once.

Before any `RebotControlAdapter`, client factory, `ArmClient`, or connection is created,
`excitation` validates the artifact SHA/schema/provenance, controller/collision precheck
status, fixed sample grid, q/qd/qdd/jerk runtime limits, control-rate equality, and a
matching-hash PASS preview report plus MP4 and `accepted_for_hardware: true` acceptance.
The runner performs no interpolation, resampling, smoothing, Fourier evaluation, or
automatic correction of `q_ref`.

After all artifact/preview gates have passed and a client is created, excitation uses this distinct preposition sequence:

```text
fresh disabled feedback
  -> configure_pvt using SDK movej_runtime policy
  -> enable
  -> if already at artifact q_ref[0] and settled: skip MoveJ
     else ArmClient.movej(artifact q_ref[0])
  -> fresh feedback
  -> verify feedback/fault/safety/limits/servo_active=false
  -> verify q within start_position_tolerance_rad
  -> verify |qd| <= preposition_settle_velocity_tolerance_rad_s
  -> enter_servo
  -> wait for each non-catch-up nominal replay deadline (current hardware candidate: 10 ms / 100 Hz)
  -> reuse adapter.latest_state without waiting for a new UDP frame
  -> gate snapshot age / feedback age / fault / Servo ownership
  -> record q_ref-q as tracking/data-quality evidence without aborting on ordinary lag
  -> replay exactly the original frozen sample
```

MoveJ is only a preposition operation. It is not appended to the frozen artifact, is not counted as an excitation command, and is not written as identification `command_valid=true` data. Metadata records a separate `preposition` block with start/target/final state and error/settle evidence. Blocking state waits remain limited to initial disabled validation, enable/Servo transitions, MoveJ settle, and cleanup/park settle; the Servo replay hot loop never calls `read_state()`.

The frozen artifact still defines the exact nominal reference grid and q_ref order, but
`timestamp_host_command_ns` now records the **actual upper-host monotonic dispatch time**
passed to `ArmClient.servo_joint()`. The nominal reference time is retained diagnostically
through `reference_dispatch_skew_ns`; actual dispatch start/interval min/mean/P95/max,
effective command rate, late-cycle count and catch-up count are summarized separately.
A late cycle delays the next target; it never causes a short-period catch-up send.

This is software/Mock capability only. Real Fourier excitation remains prohibited until
real-hardware mapping/geometry, replay-rate/limit certification and human preview
acceptance are complete.

## 9. Hardware experiment CSV

Schema:

```text
rebot_hardware_experiment_v2
```

Columns:

```text
sample_index

timestamp_host_rx_ns
timestamp_lower_ns
timestamp_host_command_ns
actual_dispatch_timestamp_ns
actual_dispatch_interval_ns
reference_dispatch_skew_ns
state_snapshot_age_ms
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
- `motion_status` separated from `identification_data_quality`;
- excitation tracking-quality summary (per-joint max/P95/threshold exceed count/first exceedance);
- feedback cadence diagnostics separating command dispatch, host UDP publication and lower/signal update cadence;
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
movej_max_velocity_rad_s
movej_max_acceleration_rad_s2
movej_max_jerk_rad_s3
movej_timeout_s
start_position_tolerance_rad
preposition_settle_velocity_tolerance_rad_s
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
Mock SDK MoveJ preposition                = PASS
Mock frozen exact replay semantics        = PASS
Mock 100 Hz no-catch-up timing             = PASS
Mock tracking monitor-only behavior        = PASS
Mock hard-fault/stale/reject injection     = PASS

A@100 Hz numerical qualification           = BLOCKED (J4 target step 0.003490609 rad > 0.003125 rad fixed gate)
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
