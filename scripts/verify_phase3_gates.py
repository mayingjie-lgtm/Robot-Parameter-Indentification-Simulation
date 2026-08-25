#!/usr/bin/env python3
"""Verify the numerical phase-3 gates from generated result YAML files."""

from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def parse_result(path: Path) -> tuple[dict[str, float], list[dict[str, float]]]:
    """Parse scalar and per-joint numeric fields from the emitted YAML subset."""
    scalars: dict[str, float] = {}
    joints: list[dict[str, float]] = []
    current: dict[str, float] | None = None
    for raw_line in path.read_text().splitlines():
        stripped = raw_line.strip()
        if stripped.startswith("- joint:"):
            current = {"joint": float(stripped.split(":", 1)[1])}
            joints.append(current)
            continue
        if ":" not in stripped or stripped.startswith("-"):
            continue
        key, value = (part.strip() for part in stripped.split(":", 1))
        try:
            numeric = float(value)
        except ValueError:
            continue
        if current is not None and raw_line.startswith("    "):
            current[key] = numeric
        elif not raw_line.startswith(" "):
            scalars[key] = numeric
    return scalars, joints


def aggregate_rmse(joints: list[dict[str, float]]) -> float:
    """Combine equal-length per-joint RMSE values into one observation RMSE."""
    values = [joint["tau_identified_rmse"] for joint in joints]
    return math.sqrt(sum(value * value for value in values) / len(values))


def write_summary(
    clean: tuple[dict[str, float], list[dict[str, float]]],
    clean_irls: tuple[dict[str, float], list[dict[str, float]]],
    gaussian: tuple[dict[str, float], list[dict[str, float]]],
    gaussian_irls: tuple[dict[str, float], list[dict[str, float]]],
    outlier: tuple[dict[str, float], list[dict[str, float]]],
    outlier_irls: tuple[dict[str, float], list[dict[str, float]]],
    friction: tuple[dict[str, float], list[dict[str, float]]],
) -> bool:
    """Evaluate gates and write compact noise/outlier comparison artifacts."""
    clean_scalars, clean_joints = clean
    clean_rmse = aggregate_rmse(clean_joints)
    clean_irls_rmse = aggregate_rmse(clean_irls[1])
    gaussian_rmse = aggregate_rmse(gaussian[1])
    gaussian_irls_rmse = aggregate_rmse(gaussian_irls[1])
    outlier_rmse = aggregate_rmse(outlier[1])
    outlier_irls_rmse = aggregate_rmse(outlier_irls[1])
    friction_scalars, friction_joints = friction

    clean_pass = (
        clean_scalars["effective_condition_number"] < 1e6
        and clean_scalars["beta_relative_error"] <= 1e-6
        and all(joint["tau_identified_rmse"] <= 1e-6 for joint in clean_joints)
        and all(joint["tau_identified_max_error"] <= 1e-5 for joint in clean_joints)
    )
    clean_irls_pass = clean_irls_rmse <= 1.05 * clean_rmse
    gaussian_ratio = gaussian_irls_rmse / gaussian_rmse
    gaussian_pass = 0.70 <= gaussian_ratio <= 1.05
    outlier_ratio = outlier_irls_rmse / outlier_rmse
    outlier_pass = outlier_ratio <= 0.70
    friction_pass = (
        friction_scalars["frictionloss_relative_error"] <= 0.01
        and all(
            joint["tau_regressor_true_max_error"] <= 1e-5
            for joint in friction_joints
        )
        and all(joint["tau_identified_rmse"] <= 1e-4 for joint in friction_joints)
    )

    noise_path = RESULTS / "piper_phase3_noise.yaml"
    noise_path.write_text(
        "schema_version: 1\n"
        f"clean_ols_aggregate_rmse: {clean_rmse:.17g}\n"
        f"clean_irls_aggregate_rmse: {clean_irls_rmse:.17g}\n"
        f"clean_irls_pass: {str(clean_irls_pass).lower()}\n"
        f"gaussian_ols_aggregate_rmse: {gaussian_rmse:.17g}\n"
        f"gaussian_irls_aggregate_rmse: {gaussian_irls_rmse:.17g}\n"
        f"gaussian_irls_to_ols_ratio: {gaussian_ratio:.17g}\n"
        f"gaussian_pass: {str(gaussian_pass).lower()}\n"
    )
    outlier_path = RESULTS / "piper_phase3_outlier.yaml"
    outlier_path.write_text(
        "schema_version: 1\n"
        f"ols_aggregate_rmse: {outlier_rmse:.17g}\n"
        f"irls_aggregate_rmse: {outlier_irls_rmse:.17g}\n"
        f"irls_to_ols_ratio: {outlier_ratio:.17g}\n"
        f"rmse_improvement_fraction: {1.0 - outlier_ratio:.17g}\n"
        f"pass: {str(outlier_pass).lower()}\n"
    )
    all_pass = (
        clean_pass
        and clean_irls_pass
        and gaussian_pass
        and outlier_pass
        and friction_pass
    )
    print(
        f"clean={clean_pass} gaussian={gaussian_pass} "
        f"outlier={outlier_pass} friction={friction_pass} all={all_pass}"
    )
    return all_pass


def main() -> None:
    """Load all fixed experiments and fail when any phase gate is violated."""
    names = [
        "piper_phase3_clean.yaml",
        "piper_phase3_clean_irls.yaml",
        "piper_phase3_gaussian_ols.yaml",
        "piper_phase3_gaussian_irls.yaml",
        "piper_phase3_outlier_ols.yaml",
        "piper_phase3_outlier_irls.yaml",
        "piper_phase3_friction.yaml",
    ]
    results = [parse_result(RESULTS / name) for name in names]
    if not write_summary(*results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
