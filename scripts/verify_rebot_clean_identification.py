#!/usr/bin/env python3
"""Verify the reBot-DM clean A-only OLS and independent B validation result."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import yaml

DOF = 6
EXPECTED_RAW_PARAMETERS = 78
EXPECTED_BASE_RANK = 52
EXPECTED_RANK_TOLERANCE = 1e-6
EXPECTED_FRICTION = [0.080, 0.070, 0.060, 0.040, 0.030, 0.020]
BASE_PARAMETER_ERROR_LIMIT = 1e-4
VALIDATION_RMSE_LIMIT_NM = 1e-5
VALIDATION_MAX_ERROR_LIMIT_NM = 1e-4
FRICTION_RELATIVE_ERROR_LIMIT = 1e-4


def _require(condition: bool, message: str) -> None:
    """Raise a gate failure with one concise diagnostic message."""
    if not condition:
        raise RuntimeError(message)


def _finite(value: Any, name: str) -> float:
    """Convert one result value to finite float or fail with its field name."""
    numeric = float(value)
    _require(math.isfinite(numeric), f"{name} must be finite")
    return numeric


def _six_positive_counts(result: dict[str, Any], key: str) -> list[int]:
    """Read one six-joint observation-count vector and require nonzero coverage."""
    values = [int(value) for value in result.get(key, [])]
    _require(len(values) == DOF, f"{key} must contain six values")
    _require(all(value > 0 for value in values), f"{key} must be nonzero on J1-J6")
    return values


def _same_path(left: Any, right: Any) -> bool:
    """Compare two result paths without requiring that generated files still exist."""
    return Path(str(left)).expanduser().resolve() == Path(str(right)).expanduser().resolve()


def _verify_prediction_inclusion(
    prediction_path: Path, expected_counts: list[int], expected_total: int
) -> None:
    """Require prediction CSV inclusion flags to reproduce the YAML validation mask."""
    _require(prediction_path.is_file(), f"prediction CSV missing: {prediction_path}")
    counts = [0] * DOF
    total = 0
    with prediction_path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            included = int(row["included"])
            _require(included in (0, 1), "prediction included must be 0 or 1")
            if not included:
                continue
            joint = int(row["joint"])
            _require(1 <= joint <= DOF, "prediction joint index must be in J1-J6")
            counts[joint - 1] += 1
            total += 1
    _require(total == expected_total, "prediction included total differs from YAML mask")
    _require(counts == expected_counts, "prediction per-joint inclusion differs from YAML mask")


def main() -> int:
    """Run all formal clean-identification acceptance gates from structured output."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", required=True, type=Path)
    args = parser.parse_args()

    result_path = args.result.resolve()
    _require(result_path.is_file(), f"result YAML missing: {result_path}")
    result = yaml.safe_load(result_path.read_text())
    _require(isinstance(result, dict), "result YAML root must be a mapping")

    _require(result.get("robot") == "rebot_dm", "robot must be rebot_dm")
    _require(result.get("algorithm") == "OLS", "clean closure must use OLS")
    _require(int(result.get("full_parameter_count", -1)) == EXPECTED_RAW_PARAMETERS,
             "raw parameter count must be 78")
    _require(int(result.get("base_parameter_rank", -1)) == EXPECTED_BASE_RANK,
             "A-only base rank must be 52")
    _require(math.isclose(
        _finite(result.get("rank_relative_tolerance"), "rank_relative_tolerance"),
        EXPECTED_RANK_TOLERANCE,
        rel_tol=0.0,
        abs_tol=1e-15,
    ), "rank_relative_tolerance must remain 1e-6")

    training = result.get("training_data_file")
    validation = result.get("validation_data_file")
    basis = result.get("basis_data_file")
    _require(training and validation and basis, "A/B/basis paths must all be recorded")
    _require(not _same_path(training, validation), "trajectory A and B must be different files")
    _require(_same_path(training, basis), "basis data must be trajectory A only")

    _require(result.get("training_observation_policy") == "saturated_sliding",
             "training observation policy must be saturated_sliding")
    _require(result.get("validation_observation_policy") == "saturated_sliding",
             "validation observation policy must be saturated_sliding")
    _require(result.get("friction_observation_mode") == "saturated_sliding",
             "friction_observation_mode must be saturated_sliding")

    training_counts = _six_positive_counts(
        result, "training_observation_count_per_joint"
    )
    validation_counts = _six_positive_counts(
        result, "validation_observation_count_per_joint"
    )
    training_total = int(result.get("training_observation_count", -1))
    validation_total = int(result.get("validation_observation_count", -1))
    _require(sum(training_counts) == training_total,
             "training per-joint counts do not sum to total")
    _require(sum(validation_counts) == validation_total,
             "validation per-joint counts do not sum to total")

    _require(int(result.get("validation_rank_diagnostic", -1)) == EXPECTED_BASE_RANK,
             "B rank diagnostic must remain 52")
    _finite(result.get("effective_condition_number"), "A effective condition")
    _finite(result.get("validation_condition_diagnostic"), "B effective condition")

    beta_error = _finite(
        result.get("base_parameter_relative_error"),
        "base_parameter_relative_error",
    )
    _require(beta_error <= BASE_PARAMETER_ERROR_LIMIT,
             f"base parameter relative error {beta_error:.3e} exceeds 1e-4")

    parameter_error = result.get("parameter_estimation_error", {})
    training_residual = parameter_error.get("training_residual", {})
    _finite(training_residual.get("aggregate_rmse"), "A training residual RMSE")
    _finite(training_residual.get("global_max_error"), "A training residual max")

    oracle = result.get("oracle_model_error", {})
    for trajectory in ("A", "B"):
        metrics = oracle.get(trajectory, {})
        _finite(metrics.get("aggregate_rmse"), f"oracle {trajectory} RMSE")
        _finite(metrics.get("global_max_error"), f"oracle {trajectory} max")
        _require(int(metrics.get("included_observation_count", -1)) > 0,
                 f"oracle {trajectory} must record selected observations")

    validation_metrics = result.get("independent_validation_error", {})
    validation_rmse = _finite(
        validation_metrics.get("aggregate_rmse"), "B aggregate RMSE"
    )
    validation_max = _finite(
        validation_metrics.get("global_max_error"), "B global max error"
    )
    _require(validation_rmse <= VALIDATION_RMSE_LIMIT_NM,
             f"B aggregate RMSE {validation_rmse:.3e} exceeds 1e-5 Nm")
    _require(validation_max <= VALIDATION_MAX_ERROR_LIMIT_NM,
             f"B global max {validation_max:.3e} exceeds 1e-4 Nm")
    _require(int(validation_metrics.get("included_observation_count", -1)) == validation_total,
             "B aggregate included count differs from validation mask")

    per_joint = result.get("validation_per_joint", [])
    _require(len(per_joint) == DOF, "validation_per_joint must contain J1-J6")
    for index, metrics in enumerate(per_joint):
        joint = index + 1
        _require(int(metrics.get("joint", -1)) == joint,
                 "validation_per_joint ordering must be J1-J6")
        _require(int(metrics.get("included_observation_count", -1)) == validation_counts[index],
                 f"J{joint} included count differs from validation mask")
        rmse = _finite(metrics.get("tau_identified_rmse"), f"J{joint} B RMSE")
        _finite(metrics.get("tau_identified_mae"), f"J{joint} B MAE")
        _finite(metrics.get("tau_identified_bias"), f"J{joint} B bias")
        joint_max = _finite(
            metrics.get("tau_identified_max_error"), f"J{joint} B max error"
        )
        _finite(metrics.get("tau_identified_r_squared"), f"J{joint} B R2")
        _require(rmse <= VALIDATION_RMSE_LIMIT_NM,
                 f"J{joint} B RMSE {rmse:.3e} exceeds 1e-5 Nm")
        _require(joint_max <= VALIDATION_MAX_ERROR_LIMIT_NM,
                 f"J{joint} B max {joint_max:.3e} exceeds 1e-4 Nm")

    friction_true = [float(value) for value in result.get("frictionloss_true", [])]
    friction_hat = [float(value) for value in result.get("frictionloss_hat", [])]
    _require(len(friction_true) == DOF and len(friction_hat) == DOF,
             "frictionloss true/estimated vectors must contain six values")
    _require(all(math.isfinite(value) for value in friction_hat),
             "frictionloss_hat must be finite")
    _require(all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-15)
                 for a, b in zip(friction_true, EXPECTED_FRICTION)),
             "frictionloss_true differs from frozen simulation truth")
    friction_error = _finite(
        result.get("frictionloss_relative_error"), "frictionloss_relative_error"
    )
    _require(friction_error <= FRICTION_RELATIVE_ERROR_LIMIT,
             f"frictionloss relative error {friction_error:.3e} exceeds 1e-4")

    robust = result.get("robust_diagnostics", {})
    _require(int(robust.get("iterations", -1)) == 0,
             "clean OLS must not run IRLS iterations")
    _require(float(robust.get("downweighted_fraction", math.nan)) == 0.0,
             "clean OLS must not downweight observations")

    full_forward = result.get("validation_full_forward_diagnostic", {})
    _finite(full_forward.get("aggregate_rmse"), "full-B diagnostic RMSE")
    _finite(full_forward.get("global_max_error"), "full-B diagnostic max")
    _require(int(full_forward.get("included_observation_count", -1)) > validation_total,
             "full-B diagnostic must include rows outside model-valid subset")

    prediction_path = Path(str(result.get("prediction_file", ""))).expanduser()
    _verify_prediction_inclusion(prediction_path, validation_counts, validation_total)

    print(f"result={result_path}")
    print(f"A_observations={training_total} per_joint={training_counts}")
    print(f"B_observations={validation_total} per_joint={validation_counts}")
    print(
        "A_rank=52 "
        f"A_condition={float(result['effective_condition_number']):.9f} "
        f"B_rank={int(result['validation_rank_diagnostic'])} "
        f"B_condition={float(result['validation_condition_diagnostic']):.9f}"
    )
    print(f"base_parameter_relative_error={beta_error:.12e}")
    print(f"frictionloss_relative_error={friction_error:.12e}")
    print(f"B_aggregate_rmse={validation_rmse:.12e}")
    print(f"B_global_max_error={validation_max:.12e}")
    print("[PASS] reBot-DM clean A-only OLS and independent B validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
