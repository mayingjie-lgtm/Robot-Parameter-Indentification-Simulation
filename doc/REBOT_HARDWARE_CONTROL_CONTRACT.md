# reBot-DM Hardware Control Contract

Last updated: 2026-09-10
Scope: reBot-DM real-hardware control integration and safety semantics
Status: A/B excitation path validated on real hardware; physical identification acceptance remains a separate offline decision

## 1. Boundary

This path is intentionally separate from the existing C++ `ExperimentBackend` and legacy
`ExperimentRecorder` because their command/data semantics do not match the audited reBot
Servo interface.

```text
config/rebot_real_ab.yaml
            |
            v
scripts/run_rebot_real_ab.py
            |
            v
RebotHardwareRunner
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
rebot_hardware_experiment_v3 CSV
            + metadata
```

The legacy UDP-only raw state recorder remains independent:

```text
public UDP -> UdpStateSubscriber -> rebot_hardware_state_v1
```

The current formal A/B workflow does not modify that raw capture schema. `joint_jog` remains a bounded diagnostic mode and is not part of the normal A/B operator workflow.

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
upper-host `time.monotonic_ns()` timestamp to `ArmClient.servo_joint`. In the currently audited
SDK, public `ArmClient.servo_joint()` calls `_send_and_wait(...)`: every Servo packet waits for
the matching TCP accept/reject before the Python call returns, and a reject/command failure is
raised instead of being treated as a successful dispatch. This synchronous ACK behavior is why
the A_run03 configured at 200 Hz only sustained about 100 Hz command dispatch. The runner still
re-checks subsequent state/fault/Servo status, and the lower Servo safety layer remains
authoritative; no second command contract is invented above it.

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
- excitation candidate position and consecutive-target fixed delta (`maximum_servo_target_delta_rad`);
- excitation packet `qd/qdd/jerk`, recursively finite-differenced from SDK-accepted targets using their real host timestamps;
- ServoCore timestamp interval `[0.5 ms, 100 ms]` before dispatch;
- target-to-measured tracking error (`maximum_tracking_error_rad`, never divided by rate): runtime gate for hold/jog, but monitor-only quality evidence during `excitation`;
- feedback validity and Lower-reported feedback age, with mode-specific semantics rather than one shared abort threshold;
- `motion_ready_feedback_max_age_ms` for conservative pre-motion / ordinary Servo state validation;
- during `excitation`, `lower_feedback_timeout_ms` defines the Lower freshness boundary and `transient_feedback_invalid_recovery_ms` permits only a bounded transient-invalid recovery window; invalid rows retain `feedback_age_ms` telemetry while `q/qd` remain NaN and are excluded from tracking/identification quality;
- upper-host UDP snapshot age against `host_state_snapshot_timeout_s`; `state_timeout_s` remains the blocking state-read timeout and is not the excitation hot-loop freshness threshold;
- primary fault and lower safety-state rejection;
- Servo-active state check;
- state/command communication timeout.

Candidates are committed to the upper derivative history only after the synchronous SDK
call confirms acceptance. A local envelope violation is recorded and is never passed to
`servo_joint`; Servo is exited and a controlled park is attempted only from fresh healthy
state, otherwise the runner disables directly. The lower SDK remains the authoritative
safety layer for ownership, watchdog, protocol and fault supervision.

## 7. TCP observation caveat

The audited `ArmClient.connect()` does not itself issue enable/PVT/MoveJ. However the lower
server publishes UDP during a TCP client session, and on TCP disconnect calls:

```text
ArmController::handle_client_disconnect()
  -> disable(...)
```

Accordingly the runner `state_only` mode means:

```text
no upper enable / enter_servo / servo_joint / MoveJ / configure_pvt / gripper call
```

It does **not** mean a TCP session is a completely side-effect-free hardware observation contract. The legacy UDP-only recorder remains the strictly observation-oriented raw path and is not replaced by this runner.

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

`state_only` remains available as a diagnostic mode. The formal A/B workflow uses the full excitation lifecycle and does not require a separate state-only run each time.

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

`servo_hold` remains available as a diagnostic mode. Formal A/B excitation has already exercised Servo ownership on real hardware; the normal operator workflow no longer requires a separate hold run before every A/B experiment.

### `excitation`

The trusted reBot Fourier source of truth remains the existing C++ `ForceController` /
`trajectory::FourierTrajectory`; the Python hardware path does not copy that mathematics.
`rebot_trajectory_exporter` freezes the accepted C++ trajectory into the
`rebot_replay_trajectory_v1` CSV/metadata artifact. The runner uses the approved
`actual_time_quintic_v1` continuous path represented by the frozen q/qd/qdd knots and
evaluates the target at actual Servo dispatch elapsed time; it does not regenerate Fourier coefficients online.

Before any `RebotControlAdapter`, client factory, `ArmClient`, or connection is created,
`excitation` validates the artifact SHA/schema/provenance, controller/collision precheck
status, fixed sample grid, q/qd/qdd/jerk runtime limits, control-rate equality, and a
matching-hash PASS preview report plus MP4 and `accepted_for_hardware: true` acceptance.
The runner performs no Fourier regeneration, smoothing, trajectory retuning, or automatic correction of the approved path. The only runtime interpolation is the audited piecewise quintic Hermite evaluation defined by `actual_time_quintic_v1`.

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
  -> evaluate the approved frozen continuous quintic path at actual dispatch elapsed time
```

MoveJ is only a preposition operation. It is not appended to the frozen artifact, is not counted as an excitation command, and is not written as identification `command_valid=true` data. Metadata records a separate `preposition` block with start/target/final state and error/settle evidence. Blocking state waits remain limited to initial disabled validation, enable/Servo transitions, MoveJ settle, and cleanup/park settle; the Servo replay hot loop never calls `read_state()`.

The frozen artifact still defines the exact nominal reference grid and q_ref order, but
`timestamp_host_command_ns` now records the **actual upper-host monotonic dispatch time**
passed to `ArmClient.servo_joint()`. The nominal reference time is retained diagnostically
through `reference_dispatch_skew_ns`; actual dispatch start/interval min/mean/P95/max,
effective command rate, late-cycle count and catch-up count are summarized separately.
A late cycle delays the next target; it never causes a short-period catch-up send.

For the current hardware-compatible A, the frozen grid is 100 Hz / 3001 samples / 30 s. The
Lower ServoCore per-packet target-jump gate remains fixed at `0.003125 rad`; `0.0028 rad` is
only the stricter search/design margin and is reported separately. Lowering replay rate for
the same continuous Fourier function increases the interval between frozen packets, so the
adjacent target step generally increases approximately with `dt`; continuous-curve safety
alone therefore does not certify per-packet Servo safety.

The 2026-09-09 offline evidence is frozen as follows:

```text
historical A coefficient SHA   86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529
historical A trajectory SHA    81be01bbfa44b933c194d9bb6d5755b4efb81bff9bfc0008020e1efba394d8e9
historical A J4 max step        0.0034906093335972943 rad @ sample 1164
historical A blocking gate      J4 Lower ServoCore fixed per-packet target-step gate only

optimized A seed / attempt      20260918 / 18
optimized coefficient SHA       d7c492ce56d7f7fb02b001bf9505c78679fa5f8a8f2f9488e4b5234a96fe9289
optimized trajectory SHA        be2faa1d58fc834efbca030aea7ec9fbb28d57895aae8e449f36164e6141f131
rank old -> new                 52 -> 52
effective condition old -> new 201.64041794 -> 62.40655638
sigma_min effective old -> new 0.01650671617 -> 0.05271473722
```

The optimized A was found by multi-seed/attempt C++ Fourier search, not by uniform scaling,
CSV downsampling, Python interpolation, or post-processing of frozen `q_ref`. Its per-joint
100 Hz maximum target steps are J1 `0.0017004399271`, J2 `0.0018011981750`, J3
`0.0015910576092`, J4 `0.0013838798796`, J5 `0.0023868853864`, J6 `0.0009089373027` rad;
all pass both the `0.0028 rad` design target and the unchanged `0.003125 rad` Lower gate.
Under `actual_time_quintic_v1`, nominal 100 Hz still evaluates all 3001 knots exactly.
With ACK delay, the command count may change: targets are evaluated on the same approved C2 path at real dispatch elapsed time, with one exact endpoint command at 30 s.

The optimized A was executed successfully on real hardware on 2026-09-10 with 2946 accepted excitation commands and no hard fault. The independently accepted B trajectory `73c0ad08360b0723ed13ba9e0e78d6cc924715bc567548fd5201c77a58457cc4` was also executed successfully with 2940 accepted excitation commands. Both runs completed MoveJ preposition and controlled park/disable/close. These execution results authorize the current control-path description; they do not by themselves prove physical torque calibration or identification parameter correctness.

## 9. Hardware experiment CSV

Schema:

```text
rebot_hardware_experiment_v3
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
trajectory_time_s
trajectory_interval_index
trajectory_interval_ratio
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

For `state_only`, command/trajectory fields are empty and `q_cmd` is `NaN`; zeros are not
invented. For excitation, `q_cmd` is the target actually accepted for dispatch, not the
nearest frozen knot.

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
- explicit real-run motion, tracking-quality, feedback-cadence and cleanup status.

Unavailable-signal flags are fixed to:

```yaml
hardware_timestamp_available: false
q_raw_available: false
current_available: false
tau_cmd_available: false
feedback_sequence_supported: false
```

`timestamp_host_rx_ns` uses the current SDK `StateStore` upper-host receive snapshot (`ActualJointState.received_monotonic_ns`), recorded when the decoded UDP state is published into the store. This differs from the legacy UDP-only recorder, which samples host receive timing directly around raw `recvfrom`.

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

## 12. Formal real-machine configuration

The following values are centralized in `config/rebot_real_ab.yaml` and must be reviewed whenever the robot/network/SDK/trajectory setup changes:

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
motion_ready_feedback_max_age_ms
maximum_disabled_feedback_age_ms
lower_feedback_timeout_ms
transient_feedback_invalid_recovery_ms
host_state_snapshot_timeout_s
state_timeout_s
controlled_park_before_disable
control_rate_hz
```

`maximum_feedback_age_ms` 现在只作为旧配置兼容别名；新 excitation 配置不得把它解释成
runtime 的统一硬中止门限。`maximum_disabled_feedback_age_ms` 只用于失能状态观测和使能前
检查；`motion_ready_feedback_max_age_ms` 用于运动准备/普通 Servo 状态检查。当前 audited
Lower freshness boundary 为 `lower_feedback_timeout_ms=250 ms`，并配合
`transient_feedback_invalid_recovery_ms=100 ms` 的有限恢复窗口。`host_state_snapshot_timeout_s`
独立限制上位机最新 UDP snapshot 的接收年龄；`state_timeout_s` 只控制阻塞式状态读取等待。

真实 `excitation` 还必须在创建硬件会话之前显式配置
`controlled_park_before_disable: true`。缺失或为 false 都 fail closed；这不代表任何 hard fault
都必须 MoveJ park，primary fault、unsafe safety state、host state-stream loss、Servo reject 等
仍走 fail-safe 路径并跳过自动 park。

For a new robot, remapped joints, changed SDK, or newly generated excitation, re-run the appropriate commissioning checks before formal A/B execution. For the currently configured robot and accepted A/B artifacts, normal operation is `--preflight-only` followed by the single formal A/B entry point.

## 13. Current status

```text
simulation identification                    = PASS
reBot hardware adapter / runner tests         = PASS
Mock MoveJ / Servo / fault injection          = PASS
A numerical qualification + human preview     = PASS
B numerical qualification + human preview     = PASS
A real excitation 2026-09-10                  = PASS, 2946 commands, ~98.15 Hz
B real excitation 2026-09-10                  = PASS, 2940 commands, ~97.95 Hz
A/B controlled cleanup                        = PASS
lower/signal independent update cadence       = ~10 Hz on both successful real runs
physical torque calibration                   = UNRESOLVED
physical parameter recovery acceptance        = UNRESOLVED
C++ ExperimentBackend integration             = DEFERRED
```

`ExperimentBackend` remains deferred because its command and recorder semantics do not match the audited reBot Servo path:

```text
ControlCommand(position, velocity, kp, kd, torque)
              !=
ServoJoint(target_position_rad only)
```

The dedicated Python reBot runner therefore remains the authoritative real-hardware path. Mock PASS is software validation; the 2026-09-10 A/B metadata are the current real-execution evidence.
