# Phase 4A — reBot-DM MuJoCo / Pinocchio Model Baseline

> Status: **PASS**
> Date: 2026-08-25
> Phase 3 baseline commit: `785dd313d1429a9203ffa4b77bc4b78bb4a5a048`

## 1. Scope

Phase 4A establishes one repository-reproducible reBot-DM rigid-body dynamics definition and proves that MuJoCo and Pinocchio represent the same six-arm-DOF system before any excitation or identification is allowed.

This phase does **not** run reBot Fourier excitation, parameter identification, real hardware, ROS, or a reBot experiment backend.

## 2. Source model

Only the DM assets below are used as source truth:

```text
/home/wlsea1/j_ws/src/robot_assets/rov_rebot/reBot_B601_DM_with_gripper.urdf
/home/wlsea1/j_ws/src/robot_assets/rov_rebot/reBot_B601_DM_with_gripper.xml
/home/wlsea1/j_ws/src/robot_assets/description/meshes_b601_gripper/
```

`reBot-Isaacsim/mjcf/rebot_devarm` and RobStride parameters are not used.

## 3. Canonical repository model

Phase 4A adds:

```text
rebot_dm/
├── rebot_dm.urdf
├── rebot_dm.xml
└── scene_identification.xml
```

The Phase 4A canonical files are dynamics-only. Visual/collision meshes are intentionally not copied into the repository in this phase; collision/scene integration remains a Phase 4B task. No canonical URDF/MJCF field depends on `/home/...` or `package://...` mesh paths.

The canonical URDF preserves the DM source values for:

```text
mass
COM
full inertia tensor
joint origin / rotation
joint axis
joint limits
J1-J6 effort limits
gripper mass and inertia
```

The canonical MJCF uses the same high-precision inertial tensors, represented as high-precision principal inertia + quaternion values.

### Why principal inertia is stored explicitly

The first implementation used MuJoCo `fullinertia`. Gravity matched Pinocchio to machine precision, but the compiled MuJoCo inertia reconstruction for link2/link3 differed from the source tensor by about `1e-9`, which produced:

```text
M(q) max abs error          2.0813e-9 kg*m^2
inverse-dynamics max error  3.8635e-9 Nm
```

This was not accepted by widening the gate.

Instead, each source URDF inertia tensor was eigendecomposed once at full double precision and written to MJCF as:

```text
quat + diaginertia
```

The reconstructed compiled MuJoCo tensors then matched Pinocchio to approximately `1e-18` for the largest affected links, and the cross-engine dynamics errors dropped to machine precision.

This is a representation conversion only; no inertial value was tuned to make a test pass.

## 4. J1-J6 mapping

The final runtime mapping is:

| Joint | MuJoCo joint id | MuJoCo qpos | MuJoCo dof | Pinocchio joint id | Pin q | Pin v | axis | lower | upper | effort |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|
| joint1 | 0 | 0 | 0 | 1 | 0 | 0 | `0 0 1` | -2.8 | 2.8 | 27 Nm |
| joint2 | 1 | 1 | 1 | 2 | 1 | 1 | `0 0 -1` | -3.14 | 0 | 27 Nm |
| joint3 | 2 | 2 | 2 | 3 | 2 | 2 | `0 0 1` | -3.14 | 0 | 27 Nm |
| joint4 | 3 | 3 | 3 | 4 | 3 | 3 | `0 0 1` | -1.87 | 1.57 | 7 Nm |
| joint5 | 4 | 4 | 4 | 5 | 4 | 4 | `0 0 1` | -1.57 | 1.57 | 7 Nm |
| joint6 | 5 | 5 | 5 | 6 | 5 | 5 | `0 0 1` | -3.14 | 3.14 | 7 Nm |

Pinocchio local Jacobian extraction confirms the signed axes, including `joint2 = 0 0 -1`.

J2/J5/J6 still must not be interpreted from the local axis vector alone because their parent transforms contain rotations.

Full Pinocchio model:

```text
nq = 8
nv = 8
njoints = 9
```

After locking the two gripper joints:

```text
nq = 6
nv = 6
njoints = 7   # universe + J1..J6
```

## 5. Gripper handling

The source model contains:

```text
gripper_joint1
gripper_joint2
```

The first candidate was:

```text
[0.0, 0.0] m
```

A manual source-geometry MuJoCo check showed four `gripper_left <-> gripper_right` contacts at that opening, so zero was rejected.

A scan of symmetric openings found:

```text
0.041 m -> 4 self contacts
0.042 m -> 0 self contacts
```

The Phase 4A fixed configuration therefore uses a margin above the first collision-free threshold:

```text
gripper_lock_position = [0.05, 0.05] m
```

At `[0.05, 0.05]` the source DM mesh model reports:

```text
gripper self-contact count = 0
```

The canonical identification scene locks both prismatic joints at `0.05 m`. Pinocchio performs:

```text
load full URDF
→ locate gripper_joint1 / gripper_joint2
→ set reference q = [0.05, 0.05]
→ buildReducedModel(...)
→ require nq=6 nv=6
```

`gripper_link`, `gripper_left`, and `gripper_right` mass/inertia remain part of J1-J6 dynamics. They are not deleted or manually folded into link6 by project-specific code.

## 6. Explicit simulation truth

No trusted real-hardware values for armature, viscous damping, or Coulomb friction were present in the accepted DM source model.

Phase 4A therefore defines the following **simulation truth baseline**:

```text
J1-J6 armature     = [0, 0, 0, 0, 0, 0]
J1-J6 damping      = [0, 0, 0, 0, 0, 0]
J1-J6 frictionloss = [0, 0, 0, 0, 0, 0]
```

These are not claimed to be real reBot identified parameters.

The first consistency closure intentionally proves rigid-body dynamics only. Armature, damping, and friction can be added later as explicit independent terms once a trusted simulation/real truth is defined.

## 7. MuJoCo actuator semantics

The canonical MJCF contains exactly six motor actuators:

```text
actuator1 → joint1, gear=1, ctrlrange=[-27, 27] Nm
actuator2 → joint2, gear=1, ctrlrange=[-27, 27] Nm
actuator3 → joint3, gear=1, ctrlrange=[-27, 27] Nm
actuator4 → joint4, gear=1, ctrlrange=[-7, 7] Nm
actuator5 → joint5, gear=1, ctrlrange=[-7, 7] Nm
actuator6 → joint6, gear=1, ctrlrange=[-7, 7] Nm
```

Runtime dimensions:

```text
MuJoCo nq=8 nv=8 nu=6
```

Unsaturated direct-torque check:

```text
max |qfrc_actuator(J1:J6) - ctrl(J1:J6)| = 0 Nm
```

## 8. Pinocchio installation and C++ integration

Installed package:

```text
ros-humble-pinocchio 4.0.0-2jammy.20260606.100000
```

Observed Python module version:

```text
pinocchio 4.0.0
```

CMake package:

```text
/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio/pinocchioConfig.cmake
```

Before configuring on this host:

```bash
source /opt/ros/humble/setup.bash
```

The project now uses:

```cmake
find_package(pinocchio REQUIRED)
```

Only the small reBot Pinocchio target and the reBot consistency executable link Pinocchio; existing Piper identification targets are not rewritten around Pinocchio.

Implementation:

```text
src/identification/include/rebot_pinocchio_dynamics.hpp
src/identification/src/robot/rebot_pinocchio_dynamics.cpp
```

The wrapper provides only:

```text
full/reduced model access
RNEA
CRBA
torque regressor
rigid-body theta
local joint-axis query
```

No RobotFactory, RegressorFactory, plugin manager, or generic dynamics framework was introduced.

## 9. Consistency gate results

All numerical cross-engine gates use explicit rigid-body semantics:

```text
armature=0
damping=0
frictionloss=0
constraints disabled for unconstrained dynamics comparison
gripper=[0.05,0.05]
```

The final tolerance is `1e-12` for mapping, gravity torque, mass-matrix absolute error, inverse dynamics, regressor identity, and mass-matrix relative Frobenius error.

The tolerance was chosen **after** observing machine-precision closure; it was not widened to hide model disagreement.

### Gate 1 — joint mapping

```text
PASS
full Pinocchio nq=8 nv=8
reduced Pinocchio nq=6 nv=6
J1-J6 order/axis/limits/effort all match MuJoCo
```

### Gate 2 — static gravity

Safe pose:

```text
q   = [0, -1, -1, 0, 0, 0]
qd  = 0
qdd = 0
```

Result:

```text
max abs error = 2.664535259100e-15 Nm
```

### Gate 3 — 100 random-pose gravity states

Seed:

```text
20260825
```

Result:

```text
aggregate RMSE = 1.095486267202e-15 Nm
global max abs = 6.217248937901e-15 Nm
```

Per-joint maximum absolute errors stay at machine-precision scale.

### Gate 4 — M(q), 100 states

Seed:

```text
20260826
```

Result:

```text
MuJoCo vs reduced Pin max abs       = 3.885780586188e-16 kg*m^2
MuJoCo vs full Pin max abs          = 3.885780586188e-16 kg*m^2
full Pin vs reduced Pin max abs     = 8.326672684689e-17 kg*m^2
max relative Frobenius error        = 1.480344322576e-15
```

Because Phase 4A armature truth is zero, no armature subtraction/addition ambiguity exists in this gate.

### Gate 5 — inverse dynamics, 100 states

Seed:

```text
20260826
```

Result:

```text
aggregate RMSE                      = 1.246681590358e-15 Nm
global max abs                      = 6.217248937901e-15 Nm
MuJoCo vs full Pin max abs          = 6.217248937901e-15 Nm
full Pin vs reduced Pin max abs     = 1.776356839400e-15 Nm
```

Explicit contribution semantics:

```text
rigid body = compared
armature   = 0
damping    = 0
friction   = 0
```

### Gate 6 — Pinocchio Y*theta identity

`theta` contains six rigid-body blocks x 10 parameters:

```text
theta size = 60
```

Result over the same 100 dynamic states:

```text
aggregate RMSE = 4.006027756863e-16 Nm
global max abs = 2.664535259100e-15 Nm
```

The identity is therefore:

```text
Pinocchio rigid-body Y*theta ≈ Pinocchio rigid-body RNEA
```

No armature/damping/friction column is disguised as a Pinocchio rigid-body parameter.

## 10. Build and regression result

Final build/test sequence:

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
python3 scripts/verify_phase3_gates.py
```

Observed CTest result:

```text
piper_regressor_consistency           PASS
fourier_trajectory_reproducibility    PASS
rebot_mujoco_model_sanity             PASS
rebot_model_consistency               PASS

4 / 4 PASS
```

Piper Phase 3 regression remains:

```text
clean=True
gaussian=True
outlier=True
friction=True
all=True
```

## 11. Source-geometry gripper validation command

This manual check is intentionally not part of repository `ctest` because the source asset is outside the repository:

```bash
./build/rebot_mujoco_model_sanity \
  --source-gripper-check \
  /home/wlsea1/j_ws/src/robot_assets/rov_rebot/reBot_B601_DM_with_gripper.xml \
  0.05
```

Observed:

```text
source geometry gripper self-contact count at [0.05, 0.05] = 0
PASS
```

## 12. Known limitations

1. Phase 4A canonical files are dynamics-only; Phase 4B still needs repository-local scene/collision assets.
2. `armature/damping/frictionloss=0` is only a simulation truth baseline, not a real-hardware claim.
3. Phase 4A has not connected reBot to `run_experiment`, controller config, recorder semantics, or identification.
4. No reBot Fourier trajectory has been run.
5. No real reBot backend has been added.

## 13. Phase status

```text
PHASE 4A = PASS
```

The next phase is **Phase 4B** only:

```text
reBot-DM
→ controller config
→ sim config
→ repository-local scene/collision model
→ minimal run_experiment rebot_dm branch
→ recorder/data-semantics smoke
```

Do not jump directly to reBot Fourier identification or real hardware.
