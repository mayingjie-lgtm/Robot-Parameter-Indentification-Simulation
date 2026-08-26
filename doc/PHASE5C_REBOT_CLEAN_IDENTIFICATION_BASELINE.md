# Phase 5C — reBot-DM Clean Identification / Independent Validation Baseline

> Date: 2026-08-26
> Frozen Phase 5B commit: `a5ee5429b0b1bcb09ce8e206aeebef3869b8760c`
> Phase 5B status: **PASS**
> Phase 5C status: **PASS**

## 1. Scope

Phase 5C closes the reBot-DM clean-simulation identification loop using the deterministic Phase 5B trajectories:

```text
trajectory A
-> MuJoCo q / qd / qdd_mujoco / tau_effort / tau_constraint
-> sample-quality validation
-> saturated-sliding observation-row selection
-> ReBotPinocchioRegressor 78-column W_A
-> A-only scaled SVD base space
-> clean OLS beta_hat

trajectory B
-> same sample/row semantics
-> A scales + A base directions + A beta_hat
-> independent torque prediction
```

This phase does **not** use IRLS, Gaussian noise, outlier injection, NLS friction, trajectory C, ROS, a real backend, or real reBot data. The configured armature, damping, and frictionloss values remain simulation-only truth and must not be reported as hardware parameters.

## 2. Frozen deterministic A/B artifacts

Trajectory A:

```text
seed               = 20260826
requested scale    = 0.04
accepted scale     = 0.04
accepted attempt   = 6
samples            = 30000
coefficient SHA256 = 86a5481c0c2459bb6e3f01d0aee4c247f4f0ed71aa444f77b6d1458c1f7cd529
dataset SHA256     = fdebf3cc5218ec53ffa3e0bd742802f7854105b7b563cb23dc52ab5bf4fb2487
```

Trajectory B:

```text
seed               = 20260829
requested scale    = 0.04
accepted scale     = 0.04
accepted attempt   = 21
samples            = 30000
coefficient SHA256 = 67ccf33c832951fe72e4d81928727fcf7c2580f0600f5ef33e046761ce810d31
dataset SHA256     = fb94e9881974da0164e1dc4eeeeb57a520d5656c367a6de56b21a620768cdaac
```

Phase 5C regenerated both artifacts from the functional excitation config and reproduced all four hashes byte-for-byte. The generated CSV/trajectory/result files remain ignored runtime artifacts and are not committed.

## 3. Frozen simulation truth

```text
armature     = [0.020, 0.025, 0.018, 0.010, 0.008, 0.006]
damping      = [0.040, 0.035, 0.030, 0.020, 0.015, 0.010]
frictionloss = [0.080, 0.070, 0.060, 0.040, 0.030, 0.020]
```

These values are used only to close a known simulation model. In particular, the frictionloss values are used by the Phase 5C row selector as a **simulation-only oracle**. Future real-reBot friction identification must define a data strategy that does not assume frictionloss is known in advance.

## 4. Why the old Piper friction filter cannot be reused directly

The old Piper clean-friction path treated

```text
|qd| >= 0.05
```

as equivalent to

```text
tau_constraint == -frictionloss * sign(qd).
```

Phase 5B proved that this equality is not a MuJoCo forward-runtime invariant for reBot-DM. The valid reBot forward semantics are:

```text
abs(tau_constraint_j) <= frictionloss_j

if abs(qd_j) >= 0.05:
    tau_constraint_j must not assist qd_j
```

Some moving rows, especially J6, remain in MuJoCo's quadratic/soft-friction regime and do not reach the force limit. Raising the velocity threshold to `0.08`, dropping J6, or deleting the entire sample would hide this runtime behavior rather than model it correctly; none of those changes was made.

Phase 5B also established for the frozen A/B runtime scene that equality/limit/contact/other constraint rows project exactly zero generalized force onto J1-J6. Therefore `tau_constraint(J1:J6)` may be interpreted as friction-only for this simulation baseline. This statement is not generalized to future backends.

## 5. Sample quality and observation fitting are different concepts

### 5.1 Sample-level quality

A reBot sample is retained when it is finite, unsaturated, contact-free, and every J1-J6 constraint force satisfies the Phase 5B friction bounds/direction semantics:

```text
finite(q, qd, qdd_mujoco, tau_effort, tau_constraint, time)
saturated == 0
contact_count == 0
abs(tau_constraint_j) <= frictionloss_j + 1e-10

if abs(qd_j) >= 0.05:
    tau_constraint_j * sign(qd_j) <= 1e-10
```

No entire sample is rejected merely because one joint has not reached its friction force limit. Frozen A/B both retain all `30000` samples.

`PreparedData` now preserves the retained samples' original `tau_constraint` values instead of replacing them with zeros, because row-validity semantics depend on those measurements.

### 5.2 Hard-Coulomb observation-row selection

The 78-column reBot regressor uses

```text
Y_friction_j = sign(qd_j).
```

That column is exactly compatible with the frozen MuJoCo forward plant only when the corresponding DOF friction force has reached its limit. For sample `k`, joint `j`, Phase 5C therefore defines a `saturated_sliding` observation row as:

```text
abs(qd[k,j]) >= 0.05
abs(abs(tau_constraint[k,j]) - frictionloss[j]) <= 1e-10
tau_constraint[k,j] opposes qd[k,j]
```

This mask is joint-row-specific. A non-saturated J6 row does not delete J1-J5 rows from the same time sample.

The same `saturated_sliding` policy is used for A basis construction, A OLS fitting, formal B metrics, and the prediction CSV `included` flag.

## 6. Observation counts

Formal selected rows:

| trajectory | total | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|---:|
| A / training | 84353 | 20994 | 18944 | 7397 | 23671 | 12130 | 1217 |
| B / validation | 77979 | 19969 | 15525 | 14286 | 13874 | 10688 | 3637 |

Every joint therefore has nonzero training and validation coverage.

These counts are recomputed by the project code; they are not hardcoded into the solver or verifier.

## 7. Oracle model closure before estimation

Before solving OLS, Phase 5C evaluates

```text
W_A * theta_true vs tau_effort_A
W_B * theta_true vs tau_effort_B
```

on the model-valid saturated-sliding rows.

Results:

| metric | A | B |
|---|---:|---:|
| aggregate RMSE [Nm] | 3.6647315338e-7 | 3.9634950594e-7 |
| global max error [Nm] | 3.5219679187e-5 | 3.5219592519e-5 |
| worst sample | 3 | 3 |
| worst joint | J3 | J3 |
| worst time [s] | 0.003 | 0.003 |

The non-machine-precision maximum is therefore a reproducible model floor and was investigated instead of hidden by a larger tolerance or by dropping startup rows.

## 8. Oracle floor root cause

The functional diagnostic

```text
rebot_identification_model_closure
```

replays the same deterministic runtime and compares three quantities on the same saturated-sliding arm rows:

1. reduced six-DoF Pinocchio with gripper joints fixed at `[0.05, 0.05] m`;
2. full eight-DoF Pinocchio with the actual MuJoCo gripper `q/qd/qdd`;
3. MuJoCo inverse-required generalized force for the exact forward state acceleration.

### 8.1 Major startup floor: soft gripper motion

Trajectory A:

```text
reduced fixed-gripper RMSE = 3.664731533828e-7 Nm
reduced fixed-gripper max  = 3.521967918663e-5 Nm
full actual-gripper RMSE   = 6.509464702579e-9 Nm
full actual-gripper max    = 1.252353463377e-6 Nm
```

At the worst reduced state:

```text
sample = 3
joint  = J3
time   = 0.003 s
reduced error = 3.521967918663e-5 Nm

gripper q   = [0.05000000333501, 0.04999999666504] m
gripper qd  = [ 8.970544238149e-7, -8.970386092582e-7] m/s
gripper qdd = [-1.659947515767e-3,  1.659943335747e-3] m/s^2
```

At this exact state the full Pinocchio prediction equals the MuJoCo actuator value to the printed precision. The dominant `3.52e-5 Nm` startup floor is therefore caused by the reduced model's fixed-gripper approximation: position motion is nanometer-scale, but the soft equality allows millimeter-per-second-squared gripper acceleration, whose inertial coupling reaches the arm.

Trajectory B reproduces the same effect:

```text
reduced fixed-gripper max = 3.521959251940e-5 Nm
full actual-gripper max   = 1.286573877851e-7 Nm
```

### 8.2 Remaining floor: MuJoCo forward/inverse numerical residual

For A:

```text
MuJoCo inverse-required vs forward actuator:
RMSE = 6.509464697429e-9 Nm
max  = 1.252353461600e-6 Nm

full Pinocchio vs MuJoCo inverse:
RMSE = 9.485563508875e-16 Nm
max  = 5.773159728051e-15 Nm
```

For B:

```text
MuJoCo inverse-required vs forward actuator:
RMSE = 6.321800010541e-10 Nm
max  = 1.286573877435e-7 Nm

full Pinocchio vs MuJoCo inverse:
RMSE = 1.169068967535e-15 Nm
max  = 5.773159728051e-15 Nm
```

Thus the full eight-DoF Pinocchio model and MuJoCo inverse dynamics agree at machine precision. The remaining `~1e-6`/`~1e-7 Nm` forward discrepancy is the MuJoCo forward constraint-solve versus inverse-required-force numerical residual for the realized acceleration, not a Pinocchio/MuJoCo model mismatch.

The Phase 5C oracle floor is therefore understood as:

```text
reduced fixed-gripper approximation
+ MuJoCo forward/inverse constraint-solver numerical residual.
```

No tolerance was enlarged and no startup rows were removed to reach this conclusion.

## 9. A-only base-parameter space

Only trajectory A saturated-sliding rows construct the formal base space. The implementation rejects the Phase 5C mode unless

```text
basis_data_file == training_data_file
```

and B is not loaded into any scaling/basis/fit operation.

The existing Piper-proven process is reused unchanged:

```text
column norm scaling
-> SVD
-> relative rank threshold = 1e-6 * sigma_max
```

Formal A result:

```text
raw parameter columns = 78
base rank            = 52
effective condition  = 304.5172418299353
```

B is evaluated only after fitting as an independent diagnostic:

```text
B rank diagnostic           = 52
B effective condition       = 501.7342445109660
```

The full column scales, singular values, and `78 x 52` base directions are written to the structured result YAML for audit.

The rank is intentionally `52`, not `78`; the raw parameter space remains structurally rank deficient.

## 10. Clean OLS and base-parameter recovery

Only unregularized OLS is used. No IRLS/WLS/TLS/ridge/NLS/noise/outlier path is invoked.

The identified base coordinates satisfy:

```text
beta_true = V_base^T * scale * theta_true
beta_hat  = OLS(W_A_base, tau_A)
```

Formal result:

```text
base_parameter_relative_error = 1.031623182296e-6
```

Acceptance gate:

```text
<= 1e-4  -> PASS
```

A selected-row training residual:

```text
aggregate RMSE = 4.773011433153e-8 Nm
global max     = 2.848977025849e-6 Nm
worst          = sample 5, J3, t=0.005 s
```

## 11. Raw minimum-norm parameters are audit-only

The YAML still records

```text
full_minimum_norm_parameters
```

for reproducibility and audit, but equality of all 78 raw scalar parameters is **not** a Phase 5C success condition. The 78-column observation matrix has rank 52, so multiple raw parameter vectors map to the same identifiable base dynamics.

Phase 5C therefore does not claim that every inertial, armature, or damping scalar is independently identifiable.

## 12. Frictionloss recovery

Phase 5A and the frozen excitation rank analysis showed that adding the six friction columns changes the numerical rank from `46` to `52`; those six columns contribute six new independent directions.

The clean minimum-norm reconstruction gives:

```text
true = [
  0.080000000000,
  0.070000000000,
  0.060000000000,
  0.040000000000,
  0.030000000000,
  0.020000000000
]

hat = [
  0.0800000375223,
  0.0700000423732,
  0.0599999889455,
  0.0399999822293,
  0.0299999942593,
  0.0200015075093
]

relative error = 1.130839103907e-5
```

The auxiliary verifier requires this relative error to remain below `1e-4`; it passes. This individual-recovery statement is specific to the six independent friction directions and is not transferred to armature, damping, or raw inertial scalars.

## 13. Independent trajectory-B prediction

B never participates in column scaling, A rank selection, base-direction construction, OLS fitting, or threshold tuning. Formal B prediction uses only:

```text
A column scales
A base directions
A beta_hat
```

Aggregate model-valid B result:

```text
RMSE       = 5.914782183802e-7 Nm
global max = 3.600941670796e-6 Nm
```

Acceptance:

```text
aggregate RMSE <= 1e-5 Nm -> PASS
worst-joint RMSE <= 1e-5 Nm -> PASS
global max <= 1e-4 Nm -> PASS
```

Per-joint results:

| joint | rows | RMSE [Nm] | MAE [Nm] | bias [Nm] | max [Nm] | R^2 |
|---|---:|---:|---:|---:|---:|---:|
| J1 | 19969 | 1.429959079e-7 | 1.197404223e-7 | 5.471636544e-9 | 3.897008623e-7 | 0.999999999997 |
| J2 | 15525 | 1.396505642e-7 | 9.785169164e-8 | -7.599920819e-8 | 9.139464935e-7 | 0.99999999999998 |
| J3 | 14286 | 1.293559484e-7 | 8.945340469e-8 | 1.348630186e-8 | 2.848676404e-6 | 0.99999999999953 |
| J4 | 13874 | 1.435926738e-7 | 1.119095123e-7 | 3.905351978e-8 | 8.940839336e-7 | 0.99999999999721 |
| J5 | 10688 | 1.084125567e-7 | 9.685818308e-8 | -9.081394580e-10 | 2.299628450e-7 | 0.99999999998137 |
| J6 | 3637 | 2.669538710e-6 | 2.139800516e-6 | 1.848643875e-6 | 3.600941671e-6 | 0.999999983492 |

J6 is the worst RMSE but remains approximately `3.75x` below the formal `1e-5 Nm` per-joint gate.

## 14. Full-B hard-Coulomb diagnostic is not a PASS metric

The same A-derived hard-Coulomb model was additionally evaluated on all `180000` B observation rows:

```text
aggregate RMSE = 0.0118106011915 Nm
global max     = 0.139995730279 Nm
```

This larger discrepancy is expected because unsaturated forward-solver rows do not satisfy `Fc * sign(qd)`. It is recorded only as a model-mismatch diagnostic and must not be interpreted as parameter-estimation failure.

The formal validation gate remains the `77979` saturated-sliding rows where the 78-column model semantics are valid.

## 15. Structured result and verifier

Functional artifacts:

```text
config/rebot_dm_clean_identification.yaml
scripts/verify_rebot_clean_identification.py
src/identification/src/rebot_identification_model_closure.cpp
```

The result YAML explicitly separates:

```text
oracle_model_error
parameter_estimation_error
independent_validation_error
validation_model_valid_subset
validation_full_forward_diagnostic
```

The verifier reads structured YAML and also checks the generated prediction CSV inclusion flags. It enforces:

```text
robot == rebot_dm
algorithm == OLS
A != B
basis == A
raw parameter count == 78
A rank == 52
rank threshold == 1e-6
training policy == saturated_sliding
validation policy == saturated_sliding
J1-J6 training and validation counts > 0
base parameter relative error <= 1e-4
B aggregate RMSE <= 1e-5 Nm
B each-joint RMSE <= 1e-5 Nm
B global/per-joint max <= 1e-4 Nm
frictionloss auxiliary relative error <= 1e-4
prediction CSV inclusion == validation mask
IRLS/downweighting diagnostics == 0
```

## 16. Reproduction commands

Generate the frozen data:

```bash
./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260826 \
  --headless \
  --output data/rebot_dm/excitation_A.csv \
  --trajectory-output data/rebot_dm/excitation_A.trajectory.csv

./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260829 \
  --headless \
  --output data/rebot_dm/excitation_B.csv \
  --trajectory-output data/rebot_dm/excitation_B.trajectory.csv
```

Explain the oracle floor:

```bash
./build/rebot_identification_model_closure 20260826
./build/rebot_identification_model_closure 20260829
```

Run formal identification and validation:

```bash
./build/identify --config config/rebot_dm_clean_identification.yaml

python3 scripts/verify_rebot_clean_identification.py \
  --result results/rebot_dm_clean_identification.yaml
```

## 17. Regression status

The Phase 5C implementation preserves all prior baselines:

```text
cmake configure/build                         PASS
ctest                                        4/4 PASS
rebot_mujoco_model_sanity                    PASS
rebot_model_consistency_test / Phase 4A      PASS
rebot_phase5a_regressor_test                 PASS
rebot_constraint_force_diagnostic A/B        PASS
verify_rebot_excitation_data.py A/B          PASS
rebot_excitation_data_quality A/B            PASS
reBot Phase 4B repository smoke              PASS
Piper Phase 3                                clean=True gaussian=True
                                               outlier=True friction=True
                                               all=True
```

The new observation-selection branch is explicitly opt-in through `friction_observation_mode: saturated_sliding`; Piper's default `moving` behavior remains unchanged.

## 18. Limitations

Phase 5C closes only a synthetic clean-simulation problem.

Known limitations:

1. `saturated_sliding` selection uses known simulation frictionloss and is therefore an oracle strategy, not a real-hardware strategy.
2. The formal six-DoF reBot regressor fixes both gripper joints; the runtime soft equalities allow tiny gripper motion, producing a small understood oracle floor.
3. MuJoCo forward constraint solve can have a small forward/inverse generalized-force consistency residual even when full Pinocchio and MuJoCo inverse dynamics agree at machine precision.
4. Raw 78-dimensional parameters remain structurally non-identifiable; the base parameters are the primary clean-success quantity.
5. No measurement noise, outliers, filtering sensitivity, or robust-estimator comparison has been tested yet.
6. Nothing in this phase validates real reBot hardware parameters, SDK timing, current/torque calibration, or ROS transport.

## 19. Phase 5C acceptance result

All formal conditions are satisfied:

```text
oracle model floor understood                         PASS
sample-quality semantics                              PASS
saturated-sliding row semantics                       PASS
A J1-J6 selected observations > 0                    PASS
B J1-J6 selected observations > 0                    PASS
A-only basis                                          PASS
A rank = 52                                           PASS
B excluded from scaling/basis/rank decision/fit      PASS
clean OLS                                             PASS
base parameter relative error <= 1e-4                PASS
independent B aggregate RMSE <= 1e-5 Nm              PASS
independent B worst-joint RMSE <= 1e-5 Nm            PASS
independent B global max <= 1e-4 Nm                  PASS
old regressions                                       PASS
```

Therefore:

```text
PHASE 5C = PASS
```

## 20. Exact next task

The next task is **simulation robustness identification**, not real-hardware identification:

```text
freeze this clean OLS baseline
-> add controlled Gaussian torque noise and deterministic outlier cases to A
-> keep B as an independent validation trajectory
-> compare OLS vs IRLS robustness in the same fixed A-derived model semantics
-> report degradation/recovery relative to this Phase 5C clean baseline
```

Do not begin that work as part of Phase 5C.
