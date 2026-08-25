# Phase 5B — reBot-DM Fourier Excitation / Data-Quality Baseline

> Date: 2026-08-25  
> Frozen Phase 5A commit: `348eeae59cd073bddfcf3a49c0d3c4c2fd18c015`  
> Phase 5A status: **PASS**  
> Phase 5B status: **PASS**

## 1. Scope

Phase 5B proves that the Phase 5A-validated reBot regressor can be supplied by executable, safe, deterministic Fourier trajectories that excite the established base-parameter space, and that the recorded MuJoCo forward-rollout constraint force has a physically correct friction data-quality interpretation.

This phase does **not** run OLS, IRLS, `beta_hat` recovery, trajectory-B torque prediction, noise/outlier experiments, real reBot identification, ROS, or a real backend.

The tested chain is only:

```text
Fourier trajectory
-> ForceController
-> MuJoCo reBot runtime
-> q / qd / qdd_mujoco / tau / qfrc_constraint
-> constraint-row decomposition
-> safety / friction data-quality gates
-> actual-trajectory ReBotPinocchioRegressor rank/SVD diagnostics
```

## 2. Frozen model and simulation truth

The canonical models remain unchanged:

```text
rebot_dm/rebot_dm.xml
rebot_dm/rebot_dm_runtime.xml
```

The excitation simulator config injects the Phase 5A synthetic simulation-only truth:

```text
armature     = [0.020, 0.025, 0.018, 0.010, 0.008, 0.006]
damping      = [0.040, 0.035, 0.030, 0.020, 0.015, 0.010]
frictionloss = [0.080, 0.070, 0.060, 0.040, 0.030, 0.020]
```

These values are numerical simulation truth only and are **not** claimed to be reBot hardware parameters. The runtime scene still contains the two soft equality constraints that lock the gripper at `[0.05, 0.05] m`.

No joint limit, torque limit, velocity limit, friction truth, armature truth, damping truth, equality setting, canonical MJCF, or rank threshold was changed in Phase 5B closure.

## 3. Functional naming

Stage numbers remain only in baseline/milestone documents. The Phase 5B implementation is functionally named:

```text
config/rebot_dm_excitation_experiment.yaml
config/rebot_dm_excitation_controller.yaml
config/rebot_dm_excitation_sim_node.yaml
scripts/verify_rebot_excitation_data.py
src/identification/src/rebot_excitation_data_quality.cpp
src/identification/src/rebot_constraint_force_diagnostic.cpp
```

CMake executables:

```text
rebot_excitation_data_quality
rebot_constraint_force_diagnostic
```

The frozen Phase 4 / Phase 5A historical names are intentionally unchanged.

## 4. Minimal controller safety change

The existing `FourierTrajectory`, fixed-seed coefficient generator, deterministic safe-search, collision checker, coefficient save/replay, joint-position checks, runtime velocity/torque checks, and experiment recorder are reused.

The only controller-code change is that `ForceController::checkTrajectory()` also rejects a desired Fourier trajectory when:

```text
abs(qd_i) > joint_velocity_safety_limits_i
```

No manager/factory/plugin abstraction was introduced.

## 5. Frozen excitation design

```text
q0        = [0.0, -1.0, -1.0, 0.0, 0.0, -0.6]
duration  = 30 s
harmonics = 5
requested coefficient scale = 0.04
control/simulation rate = 1000 Hz

kp = [110, 110, 90, 68, 52, 35]
kd = [9, 9, 7.1, 5.6, 4.5, 3.5]

velocity safety = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5] rad/s
```

The gains are accepted only as a simulation excitation controller because the actual rollout passes the safety/data gates. They are not real-hardware gains.

## 6. Final deterministic trajectories

### Trajectory A

```text
seed               = 20260826
requested scale    = 0.04
accepted scale     = 0.04
accepted attempt   = 6
samples            = 30000
coefficient SHA256 = 86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529
dataset SHA256     = fdebf3cc5218ec53ffa3e0bd742802f7854105b7b563cb23dc52ab5bf4fb2487
```

An independent A rerun is byte-identical:

```text
repeat coefficient SHA256 = 86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529
repeat dataset SHA256     = fdebf3cc5218ec53ffa3e0bd742802f7854105b7b563cb23dc52ab5bf4fb2487
```

### Trajectory B

The earlier candidate `20260827` was safe only after reducing the scale to about `0.02395` and had condition numbers above 320. Deterministic seed comparison retained the better-conditioned full-scale candidate:

```text
seed               = 20260829
requested scale    = 0.04
accepted scale     = 0.04
accepted attempt   = 21
samples            = 30000
coefficient SHA256 = 67ccf33c832951fe72e4d81928727fcf7c2580f0600f5ef33e046761ce810d31
dataset SHA256     = fb94e9881974da0164e1dc4eeeeb57a520d5656c367a6de56b21a620768cdaac
```

A and B coefficient hashes differ, so the trajectories are independently generated deterministic excitations.

## 7. Safety and actuator semantics

Both final trajectories pass:

```text
sample count               = 30000
all recorded values finite = true
time monotonic             = true
dt                         = 0.001 s
duration                   = 30.000000000014 s
q inside limits            = true
qd inside safety limits    = true
tau_cmd inside limits      = true
saturation_count           = 0
unexpected_contact_count   = 0
tau_cmd == tau_effort      = exact
```

For both trajectories:

```text
per-joint RMSE(tau_cmd - tau_effort) = [0,0,0,0,0,0] Nm
global max abs                        = 0 Nm
```

## 8. Moving-data coverage

The `|qd| >= 0.05 rad/s` fractions are:

```text
A = [0.699800, 0.631467, 0.246567, 0.789033, 0.404333, 0.115500]
B = [0.665633, 0.517500, 0.476200, 0.462467, 0.356267, 0.268133]
```

The subset that is both `|qd| >= 0.05 rad/s` and at the frictionloss force limit is:

```text
A = [0.699800, 0.631467, 0.246567, 0.789033, 0.404333, 0.040567]
B = [0.665633, 0.517500, 0.476200, 0.462467, 0.356267, 0.121233]
```

Every J1-J6 therefore has both sliding observations and saturated-sliding observations. J6 remains the weakest friction excitation on A, but it is not absent.

## 9. Original friction gate failure — preserved evidence

The first forward-rollout verifier required:

```text
when |qd_i| >= 0.05 rad/s:
qfrc_constraint_i == -frictionloss_i * sign(qd_i)
```

That rule failed reproducibly only on J6:

```text
A J6 maximum error = 7.44148847033544e-03 Nm
B J6 maximum error = 7.480498736017629e-03 Nm
```

For A, the failure decreased continuously with velocity magnitude:

```text
threshold 0.050 -> max error 0.0074415 Nm
threshold 0.055 -> max error 0.0061746 Nm
threshold 0.060 -> max error 0.0048963 Nm
threshold 0.065 -> max error 0.0036161 Nm
threshold 0.070 -> max error 0.0023409 Nm
threshold 0.075 -> max error 0.0010570 Nm
threshold 0.080 -> max error 0 Nm
```

The closure did **not** change the threshold from `0.05` to `0.08`, widen tolerance, delete J6, delete samples, or modify `frictionloss/solref/solimp` to make this failure disappear.

## 10. Constraint-force decomposition method

A dedicated diagnostic reproduces the same deterministic closed-loop runtime and uses MuJoCo's solver data directly:

```text
mjData::efc_type
mjData::efc_id
mjData::efc_state
mjData::efc_force
mj_mulJacTVec(...)
```

For every simulation step, the diagnostic classifies each solver row as:

```text
friction DOF/tendon
equality
joint/tendon limit
contact
other
```

It creates one constraint-space force vector per category, leaves all unrelated rows at zero, and calls `mj_mulJacTVec` to compute:

```text
qfrc_friction
qfrc_equality
qfrc_limit
qfrc_contact
qfrc_other
```

The reconstruction gate is:

```text
qfrc_constraint
==
qfrc_friction
+ qfrc_equality
+ qfrc_limit
+ qfrc_contact
+ qfrc_other
```

No constraint-row ordering is assumed manually.

## 11. Decomposition result

For both A and B, over all 30,000 forward-rollout samples:

```text
J1-J6 max equality generalized-force contribution = 0 Nm
J1-J6 max limit generalized-force contribution    = 0 Nm
J1-J6 max contact generalized-force contribution  = 0 Nm
J1-J6 max other generalized-force contribution    = 0 Nm

global max reconstruction error                   = 0 Nm
global max friction-bound violation                = 0 Nm
global max |qd|>=0.05 direction violation          = 0 Nm
```

The maximum friction generalized-force magnitudes are exactly the configured limits:

```text
[0.08, 0.07, 0.06, 0.04, 0.03, 0.02] Nm
```

The two gripper equality rows do carry nonzero `efc_force` in constraint space, but their Jacobian-transpose projection onto controlled J1-J6 is exactly zero in these runtime states. Therefore the J6 discrepancy is **not** caused by equality force leaking into J6.

This numerically selects root cause **B**:

> J6 `qfrc_constraint` comes from its friction DOF row, while MuJoCo forward soft friction can remain inside the frictionloss force bound instead of being exactly saturated at every sample above the old arbitrary 0.05 rad/s cutoff.

## 12. J6 representative numerical evidence

Trajectory A:

| `|qd6|` target | actual `qd6` | `qfrc_total6` | friction | equality | limit | contact | other | hard-Coulomb magnitude |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | -0.0499982 | +0.0127371 | +0.0127371 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.06 | -0.0599855 | +0.0153388 | +0.0153388 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.07 | -0.0699941 | +0.0178599 | +0.0178599 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.08 | -0.0800011 | +0.0200000 | +0.0200000 | 0 | 0 | 0 | 0 | 0.0200000 |

Trajectory B:

| `|qd6|` target | actual `qd6` | `qfrc_total6` | friction | equality | limit | contact | other | hard-Coulomb magnitude |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | -0.0499952 | +0.0127224 | +0.0127224 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.06 | -0.0600066 | +0.0150618 | +0.0150618 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.07 | +0.0699997 | -0.0179201 | -0.0179201 | 0 | 0 | 0 | 0 | 0.0200000 |
| 0.08 | +0.0800100 | -0.0200000 | -0.0200000 | 0 | 0 | 0 | 0 | 0.0200000 |

The corresponding J6 friction row is frequently in MuJoCo's `quadratic` solver state in the unsaturated region and reaches a linear force-limited regime as the force saturates. The continuous force transition is therefore a solver semantic, not missing friction truth.

## 13. Phase 5A inverse vs Phase 5B forward semantics

Phase 5A and Phase 5B intentionally test different physics paths:

```text
Phase 5A
prescribed q / qd / qdd
mj_inverse
random states use |qd| >= 0.10 rad/s
identity against explicit actuator compensation

Phase 5B
closed-loop forward rollout
mj_step
full runtime scene
soft equality constraints present
frictionloss solved by the forward constraint solver
```

Phase 5A samples are deliberately away from stick/slip and prove the augmented regressor sign convention. Phase 5B asks whether the real forward dataset obeys the runtime solver semantics. Therefore an exact Phase 5A `±Fc` result does not imply that every Phase 5B forward sample at an arbitrary lower speed cutoff must already be at the force limit.

## 14. Final friction data-quality contract

The accepted forward-rollout contract is:

```text
1. Constraint-row diagnostic must reconstruct qfrc_constraint from solver rows.
2. On controlled J1-J6, equality/limit/contact/other contribution must be zero
   within the unchanged 1e-10 Nm tolerance.
3. For every sample:
      abs(qfrc_friction_i) <= frictionloss_i + 1e-10 Nm
4. For the unchanged sliding diagnostic subset |qd_i| >= 0.05 rad/s:
      qfrc_friction_i * sign(qd_i) <= 1e-10 Nm
   i.e. friction may not assist the motion.
5. Every J1-J6 must contain nonzero sliding observations.
6. Every J1-J6 must contain nonzero saturated-sliding observations where
      abs(abs(qfrc_friction_i) - frictionloss_i) <= 1e-10 Nm.
```

Because decomposition proves that the accepted A/B CSV `tau_constraint(J1:J6)` is friction-only, the Python dataset verifier may apply the force-bound/direction checks directly to that channel for these frozen runtime data.

This is a semantics correction, not a relaxed gate:

- `0.05 rad/s` remains the sliding diagnostic threshold;
- `1e-10 Nm` remains the numerical tolerance;
- J6 remains included;
- no failed row is removed;
- the friction force is required to stay inside the exact configured force bound;
- the direction is required to oppose motion in the declared sliding subset;
- the solver decomposition independently rejects any unexpected equality/limit/contact/other contribution;
- every joint must demonstrate actual force-limit saturation somewhere in the sliding data.

The invalid assumption that was removed is only:

```text
|qd| >= 0.05  =>  friction DOF must already be exactly saturated
```

which is not a MuJoCo forward soft-constraint invariant.

## 15. Actual-trajectory rank / condition gate

Every recorded trajectory sample participates. Singular values are computed from the column-scaled Gram matrix `W_scaled^T W_scaled`; no rows are dropped or subsampled.

The frozen relative rank threshold remains:

```text
1e-6 * sigma_max
```

Final results:

| trajectory | raw columns | rank | effective condition | required rank |
|---|---:|---:|---:|---:|
| A | 60 | **36** | 122.651689 | 36 |
| A | 72 | **46** | 148.314546 | 46 |
| A | 78 | **52** | 150.360202 | 52 |
| B | 60 | **36** | 111.341200 | 36 |
| B | 72 | **46** | 133.462310 | 46 |
| B | 78 | **52** | 135.752106 | 52 |

The known structural zero columns remain the first nine joint1 rigid-body columns. No new structural excitation loss was introduced.

## 16. Reproducibility gate

Independent rerun of A (`seed=20260826`) gives:

```text
coefficient SHA identical = true
complete CSV SHA identical = true
```

A/B independence gives:

```text
A coefficient SHA != B coefficient SHA = true
```

The gate is byte-level SHA-256 equality, not numerical approximation.

## 17. Regression result

Final regression sequence passed:

```text
ctest 4 / 4 PASS
rebot_mujoco_model_sanity PASS
rebot_model_consistency_test PASS
rebot_phase5a_regressor_test PASS
reBot Phase 4B repository smoke PASS
Piper Phase 3 clean=True gaussian=True outlier=True friction=True all=True
```

No old baseline numerical behavior was changed.

## 18. Reproduction commands

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build --parallel 4

./build/rebot_constraint_force_diagnostic 20260826
./build/rebot_constraint_force_diagnostic 20260829

./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260826 \
  --headless \
  --output /tmp/rebot_dm_trajectory_A.csv \
  --trajectory-output /tmp/rebot_dm_trajectory_A.trajectory.csv

./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260829 \
  --headless \
  --output /tmp/rebot_dm_trajectory_B.csv \
  --trajectory-output /tmp/rebot_dm_trajectory_B.trajectory.csv

python3 scripts/verify_rebot_excitation_data.py \
  --csv /tmp/rebot_dm_trajectory_A.csv
python3 scripts/verify_rebot_excitation_data.py \
  --csv /tmp/rebot_dm_trajectory_B.csv

./build/rebot_excitation_data_quality /tmp/rebot_dm_trajectory_A.csv
./build/rebot_excitation_data_quality /tmp/rebot_dm_trajectory_B.csv
```

A reproducibility:

```bash
./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260826 \
  --headless \
  --output /tmp/rebot_dm_trajectory_A_repeat.csv \
  --trajectory-output /tmp/rebot_dm_trajectory_A_repeat.trajectory.csv

sha256sum \
  /tmp/rebot_dm_trajectory_A.csv \
  /tmp/rebot_dm_trajectory_A_repeat.csv \
  /tmp/rebot_dm_trajectory_A.trajectory.csv \
  /tmp/rebot_dm_trajectory_A_repeat.trajectory.csv
```

Frozen regression:

```bash
ctest --test-dir build --output-on-failure
./build/rebot_mujoco_model_sanity
./build/rebot_model_consistency_test
./build/rebot_phase5a_regressor_test

./build/run_experiment \
  --experiment-config config/rebot_dm_smoke_experiment.yaml \
  --headless \
  --output /tmp/rebot_dm_repository_smoke.csv
python3 scripts/verify_rebot_phase4b_smoke.py \
  --csv /tmp/rebot_dm_repository_smoke.csv
python3 scripts/verify_phase3_gates.py
```

## 19. Phase status

All required Phase 5B gates pass:

```text
[PASS] constraint decomposition understood
[PASS] reconstruction
[PASS] no unexpected equality/limit/contact/other contribution on J1-J6
[PASS] physically justified forward friction semantics
[PASS] friction force bound and sliding-direction contract
[PASS] A safety/data quality
[PASS] B safety/data quality
[PASS] A reproducibility
[PASS] A/B independence
[PASS] A rank 36 / 46 / 52
[PASS] B rank 36 / 46 / 52
[PASS] old regressions
```

```text
PHASE 5B = PASS
```

The next **allowed** task is the parameter-recovery / independent-validation phase, beginning with a written plan for clean A-only estimation and B-only torque prediction. It must not start automatically from this baseline-closing task.
