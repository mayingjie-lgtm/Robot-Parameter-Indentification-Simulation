# reBot A_run01 Identification-Grade Offline Audit

OFFLINE_ONLY — no SDK client is imported or instantiated by this audit.

## Executive conclusion

**A_ACCEPT_WITH_WARNINGS**

- Hardware motion execution: **completed**; accepted excitation commands: **2946**.
- Raw CSV rows: **2946**; independent fresh physical feedback: **305**; duplicate snapshots excluded: **2641**.
- Effective physical feedback rate: **9.9993 Hz**; frozen uniform resample rate: **9.0 Hz**.
- Processed samples after filtering/trim: **240**.
- Torque semantics remain **SDK reported effort, uncalibrated**.

The ~2946 raw rows are command-loop/state-snapshot rows. The lower feedback timestamp changes only about 305 times, so repeated rows are not treated as independent measurements.

## Preprocessing and bandwidth

- Filter: 4th-order zero-phase Butterworth, cutoff **2.0 Hz**.
- Nyquist after frozen resampling: **4.500 Hz**; cutoff/Nyquist = **0.444**.
- Feedback dt median/P95/P99/max: **0.100007 / 0.100890 / 0.110018 / 0.119155 s**.
- Segment count: **1**; no interpolation crosses an invalid segment.
- Per-joint qd consistency RMSE [rad/s]: **[0.006590645628161155, 0.0073031426476142984, 0.008797471345322223, 0.024697069114123477, 0.04207360080818798, 0.027170167603377874]**.

At ~10 Hz physical feedback, a 2 Hz cutoff is below Nyquist but not by a large margin. The timing distribution is tight enough for uniform resampling, while J4-J6 velocity consistency—especially J5—remains a measurement-quality warning. The acceleration estimate is usable for this training diagnostic, but should not be interpreted as high-bandwidth acceleration truth.

## Signal quality

Per-joint q/qd/qdd_est/reported-effort statistics are stored in acceptance.yaml. The filtered reported effort is finite and non-flatlined. qdd_est is derived from filtered SDK velocity after boundary trim; derivative-spike ratios are recorded rather than hidden behind an arbitrary rejection threshold.

## Tracking quality vs identification measurement quality

Tracking status is **warning**, with maximum command-vs-measured position error **0.090446 rad**. This is a motion/control tracking metric, not an automatic identification-data rejection metric. The regressor and fit use measured q, measured qd, derived qdd_est, and filtered reported effort; they do not substitute q_cmd for measured state. For the relationship check, q_cmd is interpolated onto the deduplicated/resampled measured-state host-time grid; per-joint tracking RMSE/P95/max and correlations with q/qd/qdd_est/effort_filtered are recorded in acceptance.yaml. The run has no lower-timeout-unusable measurement rows, and the measured-state regressor retains its expected numerical rank.

## A-only regressor excitation

- 60 columns: ranks at 1e-5/1e-6/1e-7 = **[36, 36, 36]**; effective condition at 1e-6 = **35.5253**; smallest retained sigma = **0.082174**.
- 72 columns: ranks at 1e-5/1e-6/1e-7 = **[46, 46, 46]**; effective condition at 1e-6 = **39.7185**; smallest retained sigma = **0.0773349**.
- 78 columns: ranks at 1e-5/1e-6/1e-7 = **[52, 52, 52]**; effective condition at 1e-6 = **40.1599**; smallest retained sigma = **0.0765215**.

The 60/72/78 ranks are stable across all three requested tolerances and match the repository's established reBot structural ranks. Structural-zero columns are reported explicitly in acceptance.yaml; no nominal rank was obtained by lowering the threshold.

## Friction threshold

At 0.01 rad/s, moving observation counts per joint are **[219, 226, 233, 226, 218, 197]** out of 240 samples/joint. Counts for 0.005/0.02/0.05 are also recorded. Because the qd-consistency discrepancy on J4-J6 exceeds 0.01 rad/s, 0.01 should not be claimed as a measured velocity-noise floor. This audit keeps it as the predeclared A-only modeling threshold rather than changing it without a stationary-noise experiment.

## A training fit

Aggregate RMSE = **0.234329 SDK Nm**, MAE = **0.137955**, bias = **-0.006479**, P95 abs residual = **0.479815**, R² = **0.995277**.

This is a **training diagnostic only** on A and is not an independent test. Residual correlations with q/qd/qdd_est and all per-joint metrics are stored in acceptance.yaml.

## Torque/effort limitation

effort_reported_source: SDK public JointState.torque_nm joint-side DM feedback estimate mapped by joint_direction

The data can support continuity checks, excitation-rank/conditioning analysis, and fitting of the SDK reported-effort signal. Because physical torque calibration is not established, this result does **not** claim absolutely accurate physical link inertial parameters or joint-torque ground truth.

## Acceptance reasons and warnings

- hardware motion completed
- physical feedback deduplicated before resampling
- processed data finite and continuous
- 60/72/78 regressor ranks stable at 1e-5/1e-6/1e-7 and match established reBot structural ranks
- A-only OLS uses measured q/qd, derived qdd_est, and uncalibrated effort_filtered
- WARNING: 2 Hz cutoff uses 0.444 of Nyquist at 9.0 Hz resampling; margin is limited
- WARNING: filtered SDK qd vs d/dt(filtered q) disagreement exceeds 0.02 rad/s on at least one joint
- WARNING: motion tracking quality is warning, but measured-state continuity/rank are audited separately
- WARNING: effort_filtered is uncalibrated SDK reported effort, not joint torque ground truth
- WARNING: friction threshold 0.01 rad/s is below some qd consistency RMSE values; retain only as an A-frozen modeling choice, not a measured noise-floor claim

## NEXT HARDWARE ACTION

Freeze A-derived preprocessing/model/rank/friction settings.
Then prepare independent B_run01.
Do not tune settings after inspecting B.
