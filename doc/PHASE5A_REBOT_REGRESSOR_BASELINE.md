# Phase 5A — reBot-DM Pinocchio Augmented Regressor / Simulation-Truth Baseline

> Date: 2026-08-25
> Phase 4B frozen baseline commit: `1a58c901ad944adef4ebfb0a05911249e0ec6cb0`
> Phase 4B status: **PASS**
> Phase 5A status: **PASS**

## 1. Scope and completion criterion

Phase 5A proves that the six-arm-DOF reBot-DM identification regressor and the MuJoCo simulation plant implement the same dynamics model before any excitation-design or parameter-fitting experiment is allowed.

The numerical identity under test is:

```text
MuJoCo simulation truth
    <->
Pinocchio rigid-body regressor
+ armature compensation
+ damping compensation
+ frictionloss compensation
```

This phase does **not** run Fourier trajectory A/B, OLS, IRLS, parameter recovery, noisy/outlier experiments, or a real reBot backend.

The reduced dynamics model continues to control only:

```text
joint1 ... joint6
```

while:

```text
gripper_joint1 = 0.05 m
gripper_joint2 = 0.05 m
```

remain locked and their link masses/inertias remain inside the reduced Pinocchio model.

## 2. Parameter ordering

The first 60 columns are exactly Pinocchio's native rigid-body parameter ordering returned by `computeJointTorqueRegressor` and `Inertia::toDynamicParameters()` for the six reduced bodies. Phase 5A does not reorder or reinterpret those columns.

Without dry friction:

```text
[ rigid_body_parameters,       # 60
  armature_1 ... armature_6,   # 6
  damping_1 ... damping_6 ]    # 6

raw parameter count = 72
```

With dry friction:

```text
[ rigid_body_parameters,          # 60
  armature_1 ... armature_6,      # 6
  damping_1 ... damping_6,        # 6
  frictionloss_1 ... frictionloss_6 ]  # 6

raw parameter count = 78
```

The explicit parameter-layout gate reports:

```text
rigid=60
augmented=72
with_friction=78
passed=true
```

## 3. Pinocchio rigid-body regressor semantics

`ReBotPinocchioRegressor` reuses `ReBotPinocchioDynamics` and obtains the rigid-body block directly from:

```cpp
pinocchio::computeJointTorqueRegressor(model, data, q, qd, qdd)
```

No hand-written reBot rigid-body regressor was added.

For the rigid-only model:

```text
tau_rigid = Y_rigid(q, qd, qdd) * theta_rigid
```

The 60-element truth vector comes from the same reduced Pinocchio model using each body's `toDynamicParameters()` ordering.

## 4. Armature column semantics

For joint `i`:

```text
Y_armature(i, i) = qdd_i
```

so the required actuator contribution is:

```text
tau_armature_i = armature_i * qdd_i
```

This matches the existing Piper Phase 3 actuator-compensation convention.

## 5. Damping sign semantics

MuJoCo passive viscous damping contributes:

```text
-damping_i * qd_i
```

to the passive generalized force. Therefore the actuator torque required to reproduce the prescribed acceleration is:

```text
+damping_i * qd_i
```

and the regressor column is:

```text
Y_damping(i, i) = +qd_i
```

## 6. Frictionloss sign semantics

Away from zero velocity, MuJoCo dry friction opposes motion through the constraint solver:

```text
qfrc_constraint_i = -frictionloss_i * sign(qd_i)
```

Therefore the required actuator compensation is:

```text
+frictionloss_i * sign(qd_i)
```

and the Phase 5A regressor uses:

```text
Y_friction(i, i) = sign(qd_i)
```

The deterministic numerical gate samples every joint with:

```text
|qd_i| >= 0.10 rad/s
```

which is stricter than the explicit reporting threshold:

```text
0.05 rad/s
```

and therefore avoids the zero-speed stick/slip discontinuity.

## 7. Injected simulation truth

Phase 4A/4B canonical truth remains unchanged at zero in `rebot_dm/rebot_dm.xml` and `rebot_dm/rebot_dm_runtime.xml`.

Phase 5A uses a separate simulator config:

```text
config/rebot_dm_phase5a_sim_node.yaml
```

with the following **simulation-only test truth**:

```text
armature =
[0.020, 0.025, 0.018, 0.010, 0.008, 0.006]

damping =
[0.040, 0.035, 0.030, 0.020, 0.015, 0.010]

frictionloss =
[0.080, 0.070, 0.060, 0.040, 0.030, 0.020]
```

These values are deliberately moderate numerical test values and are **not** claimed to be real reBot hardware parameters.

`PandaSimulator` remains the historical simulator class name. It now optionally parses:

```text
joint_armature
joint_damping
joint_frictionloss
```

and, after model loading, writes the values to the six arm DOFs and immediately reads each value back. The Phase 4B zero config remains zero.

The Phase 5A metadata smoke confirmed that the injected values are written to:

```text
armature_truth
damping_truth
joint_frictionloss
```

while rerunning the Phase 4B canonical config still writes six zeros for all three truth groups.

## 8. Gate A — rigid only

Deterministic seed:

```text
20260825
```

Number of random legal states:

```text
128
```

Truth:

```text
armature = 0
damping = 0
frictionloss = 0
```

Result:

```text
max |Y*theta - MuJoCo inverse actuator requirement|
= 7.105427357601e-15 Nm

max friction/constraint-semantics error
= 0 Nm
```

The Phase 4A machine-precision rigid-body identity is preserved.

## 9. Gate B — rigid + armature

Injected non-zero armature with zero damping/frictionloss:

```text
max |Y*theta - MuJoCo inverse actuator requirement|
= 7.105427357601e-15 Nm

max constraint-semantics error
= 0 Nm
```

Therefore:

```text
tau = tau_rigid + armature .* qdd
```

is numerically identical to the MuJoCo plant to machine precision.

## 10. Gate C — rigid + armature + damping

Injected non-zero armature and damping with zero frictionloss:

```text
max |Y*theta - MuJoCo inverse actuator requirement|
= 7.105427357601e-15 Nm

max constraint-semantics error
= 0 Nm
```

Therefore the damping actuator-compensation sign is confirmed as:

```text
+damping .* qd
```

not `-damping .* qd`.

## 11. Gate D — + frictionloss

All three explicit simulation-truth terms enabled:

```text
max |Y*theta - MuJoCo inverse actuator requirement|
= 7.993605777301e-15 Nm

max |qfrc_constraint - (-frictionloss*sign(qd))|
= 0 Nm
```

No tolerance was widened to obtain this result. The gate uses:

```text
torque tolerance     = 1e-10 Nm
constraint tolerance = 1e-10 Nm
```

while the observed errors remain at double-precision machine scale.

## 12. Identification dispatch gate

The existing `Identification` class now uses an explicit `rebot_dm` branch rather than a factory or registry.

A direct comparison against `ReBotPinocchioRegressor` reports:

```text
numParameters(NONE)              = 60
numParameters(ALL)               = 72
numParameters(ALL_WITH_FRICTION) = 78

ground-truth parameter max abs difference = 0
observation-matrix max abs difference      = 0
```

`DataLoader`, OLS, IRLS, and the existing base-space/SVD code were not copied or rewritten.

## 13. Rank / SVD facts

Rank analysis uses a separate deterministic set of 256 random legal states, column-norm scaling, and the same relative threshold convention used by the trusted identification path:

```text
relative rank threshold = 1e-6 * sigma_max
```

Observed results:

```text
60 raw columns:
  numerical rank       = 36
  effective condition  = 7.041583782780
  null/near-null count = 24

72 raw columns:
  numerical rank       = 46
  effective condition  = 7.53538424
  null/near-null count = 26

78 raw columns:
  numerical rank       = 52
  effective condition  = 9.99003132
  null/near-null count = 26
```

The numerical gate therefore intentionally does **not** require rank 60, 72, or 78.

### 13.1 60-column singular values

```text
[2.53949280e+00, 2.46688822e+00, 1.91510077e+00, 1.82983829e+00,
 1.73098153e+00, 1.47923084e+00, 1.42213379e+00, 1.33505249e+00,
 1.22540624e+00, 1.17748382e+00, 1.15460262e+00, 1.10018568e+00,
 1.08898538e+00, 1.06279030e+00, 1.05562915e+00, 1.00902503e+00,
 1.00097124e+00, 9.92721718e-01, 9.68782076e-01, 9.47554076e-01,
 9.18590465e-01, 9.12794136e-01, 9.04338600e-01, 8.50095123e-01,
 8.35535056e-01, 8.05303991e-01, 7.80528364e-01, 7.64533714e-01,
 7.22845105e-01, 6.87088477e-01, 6.68571210e-01, 6.37083587e-01,
 6.28167006e-01, 5.68576370e-01, 5.51067247e-01, 3.60642276e-01,
 2.13324506e-14, 9.66891083e-15, 2.94378626e-15, 2.14626600e-15,
 8.40767823e-16, 6.15610128e-16, 5.19978221e-16, 5.13039168e-16,
 4.61679032e-16, 4.44445927e-16, 4.15470538e-16, 3.18141209e-16,
 3.16376494e-16, 2.90654253e-16, 2.13797939e-16,
 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

### 13.2 72-column singular values

```text
[2.71423070e+00, 2.49323472e+00, 1.92014819e+00, 1.87877517e+00,
 1.83453428e+00, 1.48606542e+00, 1.46387370e+00, 1.34697730e+00,
 1.29881860e+00, 1.25691268e+00, 1.21802387e+00, 1.17701612e+00,
 1.13521072e+00, 1.11115765e+00, 1.08485989e+00, 1.07659077e+00,
 1.06916490e+00, 1.05008563e+00, 1.04022414e+00, 1.02529042e+00,
 1.00621312e+00, 9.96058832e-01, 9.94406577e-01, 9.85111169e-01,
 9.81023173e-01, 9.69873994e-01, 9.48030566e-01, 9.36192555e-01,
 9.23294129e-01, 9.09394902e-01, 9.01974572e-01, 8.93990698e-01,
 8.44637380e-01, 8.16971044e-01, 8.05093465e-01, 7.82336689e-01,
 7.57496724e-01, 7.22340680e-01, 6.81942513e-01, 6.64914898e-01,
 6.34888019e-01, 5.96999665e-01, 5.47866980e-01, 5.33121208e-01,
 4.92761303e-01, 3.60198049e-01, 1.18084561e-06,
 2.18297161e-14, 9.69568837e-15, 2.99714034e-15, 2.17385474e-15,
 1.25367924e-15, 7.77795815e-16, 6.24608817e-16, 6.20027650e-16,
 5.64352568e-16, 5.37425059e-16, 4.29475749e-16, 3.79384117e-16,
 3.49665554e-16, 2.81249054e-16, 2.17216190e-16, 5.78724180e-31,
 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

### 13.3 78-column singular values

```text
[2.71915260e+00, 2.49452510e+00, 1.92728653e+00, 1.87999837e+00,
 1.84609875e+00, 1.50163971e+00, 1.47680934e+00, 1.41862594e+00,
 1.40091105e+00, 1.39480478e+00, 1.38983455e+00, 1.38038202e+00,
 1.34190419e+00, 1.29637620e+00, 1.28813767e+00, 1.23810472e+00,
 1.20872254e+00, 1.16957862e+00, 1.10898234e+00, 1.09987878e+00,
 1.06422393e+00, 1.05673720e+00, 1.05183754e+00, 1.03774073e+00,
 1.01652013e+00, 9.91588802e-01, 9.82122980e-01, 9.61395404e-01,
 9.35055391e-01, 9.13042372e-01, 9.06862433e-01, 9.04505318e-01,
 8.47441276e-01, 8.18221861e-01, 8.07635671e-01, 7.83584669e-01,
 7.59754556e-01, 7.22781459e-01, 6.82252246e-01, 6.65027193e-01,
 6.35885510e-01, 5.97472942e-01, 5.48749286e-01, 5.33500566e-01,
 4.93539684e-01, 3.60313277e-01, 3.05663973e-01, 3.04096128e-01,
 3.01848722e-01, 2.98465414e-01, 2.87624081e-01, 2.72186594e-01,
 1.18080941e-06, 2.17873490e-14, 9.70268038e-15, 2.94477154e-15,
 2.10799779e-15, 8.63217198e-16, 6.25987223e-16, 6.09565949e-16,
 6.05682680e-16, 5.68459459e-16, 5.41838650e-16, 4.81092603e-16,
 4.46253257e-16, 3.56452501e-16, 3.39985613e-16, 3.08602636e-16,
 7.35292757e-31,
 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

## 14. Known structurally unidentifiable / weak directions

For all three raw layouts, columns 0 through 8 are structurally zero over the tested model/state set:

```text
joint1_m
joint1_mx
joint1_my
joint1_mz
joint1_Ixx
joint1_Ixy
joint1_Ixz
joint1_Iyy
joint1_Iyz
```

No additional column has a near-zero column norm under the rank-test threshold, but column independence is still limited by coupled base-parameter relationships.

Important consequences:

- 60 raw rigid parameters span only rank 36 at the declared threshold.
- adding armature+damping increases rank from 36 to 46, not by all 12 raw columns;
- therefore Phase 5B/5C must not require every armature/damping scalar to be independently recovered merely because each raw column is non-zero;
- adding the six friction columns increases rank from 46 to 52 in this deterministic random-state gate, so all six added friction directions contribute independent dimensions relative to the 72-column model for this sampled space;
- a weak singular direction around `1.18e-6` exists in both the 72- and 78-column scaled matrices and falls below the declared `1e-6 * sigma_max` rank threshold.

These results establish the actual base-parameter-space fact; they do not redefine the project rank threshold to force a desired rank.

## 15. Reproduction commands

Phase 4B closure and regressions:

```bash
source /opt/ros/humble/setup.bash

cmake -S . -B build
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure

./build/rebot_mujoco_model_sanity \
  --source-gripper-check \
  /home/wlsea1/j_ws/src/robot_assets/rov_rebot/reBot_B601_DM_with_gripper.xml \
  0.05

./build/rebot_model_consistency_test

./build/run_experiment \
  --experiment-config config/rebot_dm_smoke_experiment.yaml \
  --headless \
  --output /tmp/rebot_dm_repository_smoke.csv

python3 scripts/verify_rebot_phase4b_smoke.py \
  --csv /tmp/rebot_dm_repository_smoke.csv

python3 scripts/verify_phase3_gates.py
```

Phase 5A numerical gate:

```bash
./build/rebot_phase5a_regressor_test
```

Phase 5A simulator-truth metadata injection smoke:

```bash
./build/run_experiment \
  --experiment-config config/rebot_dm_smoke_experiment.yaml \
  --sim-config config/rebot_dm_phase5a_sim_node.yaml \
  --headless \
  --output /tmp/rebot_dm_phase5a_truth_smoke.csv
```

The Phase 4B canonical smoke must then be rerun without the override config to prove the frozen zero-truth baseline remains unchanged.

## 16. Regression status and limitations

Final regression status:

```text
ctest: 4 / 4 PASS
reBot MuJoCo model sanity: PASS
reBot Phase 4A MuJoCo <-> Pinocchio consistency: PASS
reBot Phase 4B canonical 500-sample smoke: PASS
Piper Phase 3: clean=True gaussian=True outlier=True friction=True all=True
```

Known limitations:

1. The Phase 5A random-state rank is a model/base-space diagnostic, not an excitation-trajectory quality result.
2. No OLS/IRLS fit was run, so there is no `beta_hat` and no parameter-recovery claim in this phase.
3. Raw inertial parameter equality is not a future success criterion because the 60-column rigid system is rank deficient.
4. Future parameter-recovery acceptance must be based on base-parameter recovery plus independent trajectory-B torque prediction.
5. Individual armature/damping/frictionloss recovery may only be required when rank analysis for the actual excitation proves the corresponding directions identifiable.
6. The injected armature/damping/frictionloss values are simulation test truth only and are not hardware estimates.
7. No Fourier excitation, real reBot backend, ROS path, or hardware identification was introduced.

## 17. Phase status and next task

All required Phase 5A numerical and regression gates pass:

```text
PHASE 5A = PASS
```

The next allowed task is:

```text
Phase 5B:
reBot Fourier trajectory A/B excitation design and data-quality gate
```

Phase 5B should design independent excitation trajectories and prove safety/data quality/rank coverage before any formal OLS/IRLS recovery closure. It must not reinterpret the Phase 5A raw rank as full parameter identifiability.
