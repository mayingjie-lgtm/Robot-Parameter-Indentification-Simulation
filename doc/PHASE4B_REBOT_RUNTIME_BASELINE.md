# Phase 4B — reBot-DM Runtime / Data-Semantics Baseline

> Date: 2026-08-25
> Phase 3 baseline commit: `785dd313d1429a9203ffa4b77bc4b78bb4a5a048`
> Phase 4A status: **PASS**
> Phase 4B status: **BLOCKED — repository-local binary STL assets are not yet vendored**

## 1. Scope

Phase 4B connects the Phase 4A reBot-DM canonical model to the existing unified simulation experiment path:

```text
run_experiment
  -> ForceController
  -> SimulationBackend
  -> PandaSimulator (historical class name retained)
  -> MuJoCo
  -> ExperimentRecorder
  -> CSV + .meta.yaml
```

This phase does **not** run reBot Fourier identification excitation, parameter identification, a real backend, ROS, or online Pinocchio inverse dynamics.

## 2. Phase 4A frozen baseline

Before Phase 4B changes the following gates were rerun and passed:

```text
ctest: 4 / 4 PASS
piper_regressor_consistency: PASS
fourier_trajectory_reproducibility: PASS
rebot_mujoco_model_sanity: PASS
rebot_model_consistency: PASS

Phase 3:
clean=True
gaussian=True
outlier=True
friction=True
all=True
```

Phase 4A numerical closure remains at machine precision:

```text
random gravity global max       6.217248937901e-15 Nm
M(q) max abs                    3.885780586188e-16 kg*m^2
inverse dynamics global max     6.217248937901e-15 Nm
Pinocchio Y*theta global max    2.664535259100e-15 Nm
```

The Phase 4 simulation truth remains:

```text
armature     = [0, 0, 0, 0, 0, 0]
damping      = [0, 0, 0, 0, 0, 0]
frictionloss = [0, 0, 0, 0, 0, 0]
gripper lock = [0.05, 0.05] m
```

## 3. Repository runtime model structure

Phase 4B adds the text-side runtime structure:

```text
rebot_dm/
├── rebot_dm.urdf
├── rebot_dm.xml
├── scene_identification.xml
├── rebot_dm_runtime.xml
├── scene_runtime.xml
└── assets/
    └── README.md
```

`rebot_dm_runtime.xml` preserves the Phase 4A masses, COMs, principal inertias, body transforms, J1-J6 limits, zero armature/damping/friction truth, six direct torque actuators, and gripper mass/inertia. Geometry uses `density=0`, so it does not infer or replace any rigid-body inertial parameter.

The runtime MJCF uses only:

```text
meshdir="assets"
file="base_link.STL"
...
file="gripper_right.STL"
```

There is no `/home/...` or `package://...` runtime mesh reference in the repository runtime XML/configuration.

### Blocking asset gate

The ten accepted DM STL files are binary files and are **not yet physically present** under `rebot_dm/assets/`. `rebot_dm/assets/README.md` records all accepted source SHA-256 values so the byte-for-byte copy can be verified.

Therefore the canonical repository command currently stops with:

```text
Mujoco load error: Error opening file 'assets/base_link.STL'
```

This unresolved gate alone prevents declaring `PHASE 4B = PASS`.

## 4. Collision model and safe runtime pose

The original Phase 4A arm pose:

```text
[0, -1, -1, 0, 0, 0]
```

was rechecked with the complete accepted DM meshes. It produced:

```text
4 x base_link <-> link1
1 x link4 <-> gripper_link
```

The four base/link1 contacts are fixed overlap at the adjacent mounting interface. A deterministic local scan found that changing only J6 removes the non-adjacent self-collision:

```text
runtime arm pose = [0, -1, -1, 0, 0, -0.6]
```

At this pose the raw complete-mesh model has only the four fixed `base_link <-> link1` contacts. The runtime model therefore contains one explicit structural exclusion only:

```xml
<exclude body1="base_link" body2="link1"/>
```

No general self-collision category is disabled.

With that one structural exclusion, the complete-mesh runtime pose has:

```text
unexpected contact_count = 0
```

## 5. Gripper geometry gate

The source DM mesh was explicitly rechecked at:

```text
gripper_joint1 = 0.05 m
gripper_joint2 = 0.05 m
```

Observed pair counts:

```text
gripper_left  <-> gripper_right = 0
gripper_left  <-> gripper_link  = 0
gripper_right <-> gripper_link  = 0
total gripper self contacts     = 0
```

The runtime scene keeps both equality locks at `[0.05, 0.05] m`; the gripper mass and inertia remain in the dynamics.

## 6. Runtime scene

`rebot_dm/scene_runtime.xml` includes the runtime model, applies the same two gripper equality constraints used by Phase 4A, and defines:

```text
runtime_home qpos = [0, -1, -1, 0, 0, -0.6, 0.05, 0.05]
```

The scene does not add a new dynamics model or Pinocchio runtime path.

## 7. Controller configuration

New file:

```text
config/rebot_dm_force_controller_node.yaml
```

The controller is explicitly marked:

```text
robot: rebot_dm
controller_mode: hold_position
```

Phase 4B adds a minimal configurable `hold_position` branch to the existing `ForceController`. When that mode is selected, Fourier coefficients are not generated, checked, saved, or executed.

The smoke controller uses zero feedback gains and a constant safe-pose gravity feedforward torque previously evaluated from the accepted Phase 4A canonical rigid-body model:

```text
q_hold = [0, -1, -1, 0, 0, -0.6]

tau_hold =
[-3.2014213502407074e-10,
  6.9803246271066577e-01,
 -2.7890575824450243e+00,
 -6.7273504906954651e-01,
  1.6426378890940739e-06,
 -1.0879085259603370e-04] Nm
```

This is a deterministic static runtime smoke command only. It is **not** a Fourier identification excitation and it does not add online Pinocchio to `ForceController` or `SimulationBackend`.

Controller startup validates the configured J1-J6 position and torque limits against the loaded MuJoCo model. The smoke-only velocity safety envelope is also checked at runtime.

## 8. Simulation configuration

New file:

```text
config/rebot_dm_sim_node.yaml
```

It declares:

```text
robot: rebot_dm
dof: 6
simulation_rate_hz: 1000
initial_keyframe: runtime_home
validate_unit_torque_actuators: true
joint_frictionloss: [0,0,0,0,0,0]
model: rebot_dm/rebot_dm_runtime.xml
scene: rebot_dm/scene_runtime.xml
```

When `validate_unit_torque_actuators` is enabled, the existing historical `PandaSimulator` verifies at startup that:

```text
nu == 6
actuator 0 -> joint1
...
actuator 5 -> joint6
gear = [1,0,0,0,0,0] for every actuator
```

No simulator class rename or new backend hierarchy was introduced.

## 9. run_experiment branch

`defaultExperimentConfigForRobot()` now accepts:

```text
robot == rebot_dm
```

and selects the reBot controller, simulator config, runtime scene, and collision model. Phase 4B also enforces:

```text
rebot_dm backend == sim
controller.armDOF() == 6
```

The existing `SimulationBackend`, experiment loop, and `ExperimentRecorder` are reused.

## 10. Smoke commands

### 10.1 Canonical repository command

This is the intended final command:

```bash
./build/run_experiment \
  --experiment-config config/rebot_dm_smoke_experiment.yaml \
  --headless \
  --output data/phase4b/rebot_dm_repository_smoke.csv
```

It is currently blocked only because the binary files under `rebot_dm/assets/` have not yet been vendored.

### 10.2 Complete-geometry validation used during implementation

To validate the runtime/contact/data semantics before the local binary copy was possible, an untracked build-only mirror of the same canonical runtime MJCF was pointed at the accepted external DM asset directory. This diagnostic is not the repository baseline and must not replace the repository-local asset gate.

The complete-geometry smoke produced:

```text
rows     = 500
duration = 0.500000000 s
dt       = 0.001000000 s
```

## 11. Complete-geometry smoke state ranges

Per-joint minima and maxima:

```text
q_min =
[-2.577952465825776e-09,
 -1.0000000000207097,
 -1.0,
 -5.582490527278864e-08,
  0.0,
 -0.6000005151049764]

q_max =
[ 8.607805402113548e-11,
 -0.9999999999601709,
 -0.9999999836364674,
  0.0,
  1.1058868958026496e-08,
 -0.6]

qd_min =
[-1.261035252380346e-08,
 -5.205586111499482e-09,
  0.0,
 -1.1963560919370543e-06,
 -5.3276595013690886e-08,
 -2.082233316020295e-06]

qd_max =
[2.233283290664814e-08,
 2.2613739589811966e-10,
 1.5577734195126024e-07,
 0.0,
 2.7711369154404163e-06,
 0.0]

qdd_mujoco_min =
[-5.604694543149253e-06,
 -5.2055861114994825e-06,
 -3.893336022570715e-05,
 -1.1963560919370543e-03,
 -6.928890279462017e-04,
 -4.2493373173295465e-06]

qdd_mujoco_max =
[2.233283290664814e-05,
 1.3016353794019485e-06,
 1.5577734195126023e-04,
 2.990624571957408e-04,
 2.771136915440416e-03,
 -5.467073926144718e-07] rad/s^2
```

All q values remain inside the configured joint limits and all velocities remain far below the smoke safety limits.

## 12. Torque semantics

The complete-geometry smoke command range is constant and equals the hold torque:

```text
tau_cmd_min == tau_cmd_max == tau_hold
```

For all 500 samples:

```text
per-joint RMSE(tau_cmd - tau_effort) = [0,0,0,0,0,0] Nm
per-joint max abs                    = [0,0,0,0,0,0] Nm
aggregate RMSE                       = 0 Nm
global max abs                       = 0 Nm
saturation count                     = 0
```

Therefore unit-gear runtime actuator semantics are exactly:

```text
ControlCommand.torque[0..5]
=
MuJoCo ctrl[0..5]
=
MuJoCo qfrc_actuator(J1..J6)
```

for this unsaturated smoke.

## 13. Contact / constraint result

Complete-geometry runtime smoke:

```text
unexpected contact_count = 0
max abs qfrc_constraint(J1:J6) = 0 Nm
```

The simulator now prints `geom1`, `geom2`, time, and the six arm joint positions if a future runtime contact occurs.

## 14. Geometry does not change rigid-body dynamics

The Phase 4A dynamics-only canonical MJCF and the complete-geometry runtime MJCF were compared over 100 deterministic random arm states with contacts/equalities disabled for the rigid-body comparison.

Observed:

```text
max abs inverse-dynamics difference = 0.000000000000e+00 Nm
```

The runtime mesh geoms therefore do not modify the accepted Phase 4A rigid-body dynamics parameters.

## 15. CSV schema

The reBot simulation reuses the Phase 3 simulation-truth recorder schema exactly:

```text
time_begin
time_end
q0..q5
qd0..qd5
qdd_mujoco0..qdd_mujoco5
qdd_diff0..qdd_diff5
tau_cmd0..tau_cmd5
tau_effort0..tau_effort5
tau_constraint0..tau_constraint5
q_next0..q_next5
qd_next0..qd_next5
saturated
contact_count
```

The old ambiguous `q/qd/qdd/tau` simulation schema was not reintroduced.

## 16. Metadata semantics

The simulation metadata writer now also records model provenance, Git commit, controller mode, gripper lock, armature truth, and damping truth. The reBot smoke metadata includes:

```text
robot: rebot_dm
backend: sim
model: <runtime collision model>
git_commit: 785dd313d1429a9203ffa4b77bc4b78bb4a5a048
controller_mode: hold_position
time_step: 0.001
controller_config: <reBot controller config>
simulation_config: <reBot sim config>
gripper_lock_position: [0.05, 0.05]
armature_truth: [0,0,0,0,0,0]
damping_truth: [0,0,0,0,0,0]
joint_frictionloss: [0,0,0,0,0,0]
```

Source fields remain:

```text
q_source: mujoco_qpos_pre_integration
qd_source: mujoco_qvel_pre_integration
qdd_mujoco_source: mujoco_qacc_pre_integration
qdd_diff_source: forward_velocity_difference
tau_cmd_source: control_command_torque
tau_effort_source: mujoco_qfrc_actuator
tau_constraint_source: mujoco_qfrc_constraint
```

## 17. Automated smoke verifier

New script:

```text
scripts/verify_rebot_phase4b_smoke.py
```

It checks:

```text
CSV/meta existence
rows > 0
required Phase 3 headers
all values finite
strictly monotonic time
q within limits
qd within smoke safety limits
tau_cmd within torque limits
saturation == 0
unexpected contacts == 0
J1-J6 constraint torque near zero
tau_cmd ~= tau_effort
robot == rebot_dm
backend == sim
controller_mode == hold_position
Git commit present
gripper lock == [0.05,0.05]
armature/damping/friction truth == 0
explicit source semantics
```

It intentionally does not test identified parameters, regressor rank, or validation RMSE.

## 18. CTest / Piper regression

After the runtime changes, project compilation and the existing CTest suite continue to pass. The final Phase 4B closure must rerun these gates after the repository-local binary assets are physically present.

Expected unchanged regression gate:

```text
ctest: 4 / 4 PASS
Phase 3: clean=True gaussian=True outlier=True friction=True all=True
```

## 19. Known limitations / blocker

1. The ten accepted binary STL files are not yet present under `rebot_dm/assets/`; this is the only current Phase 4B completion blocker.
2. The complete-geometry runtime/data checks were performed against an untracked build-only mirror using the accepted external STL bytes. Those checks are evidence for geometry/runtime semantics, but they do not satisfy the repository-local asset requirement.
3. The static hold feedforward is only a Phase 4B smoke command at one pose; it is not a controller-performance design and not an identified model.
4. Phase 4B does not run reBot Fourier excitation or any identification solver.
5. No reBot real backend or ROS path is added.

## 20. Phase status

Current status:

```text
[PASS] Phase 4A consistency remains valid
[PASS] Phase 3 Piper regression remains valid
[BLOCKED] repository-local reBot binary STL assets physically present
[PASS] repository runtime XML has no absolute/package mesh path
[PASS] complete accepted mesh geometry loads in diagnostic mirror
[PASS] gripper lock = [0.05, 0.05]
[PASS] all three gripper internal contact pairs = 0
[PASS] six controlled arm DOF
[PASS] J1->J6 actuator order / gear=1 runtime gate
[PASS] deterministic 0.5 s static smoke with complete geometry
[PASS] no saturation
[PASS] no unexpected contact
[PASS] tau_cmd == tau_effort
[PASS] Phase 3 simulation CSV semantics preserved
[PASS] metadata source semantics explicit
[PASS] all recorded values finite
[PASS] q inside limits
```

Because the repository-local binary asset gate is not closed:

```text
PHASE 4B != PASS
```

Do not start Phase 4C yet. Once the ten STL files are copied byte-for-byte into `rebot_dm/assets/`, verify their SHA-256 values, rerun the canonical repository smoke command and `verify_rebot_phase4b_smoke.py`, then rerun CTest + Phase 4A + Phase 3 gates. Only then may Phase 4B be marked PASS.
