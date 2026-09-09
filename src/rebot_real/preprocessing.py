"""Offline processing of reported-effort experiments; never imports a control client."""
from __future__ import annotations

from collections import Counter
import csv
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
from scipy.signal import butter, sosfiltfilt
import yaml

SCHEMA = "rebot_reported_effort_identification_v1"
SIGNALS = [f"{prefix}{j}" for prefix in ("q", "qd", "effort_reported") for j in range(6)]
COLUMNS = ["time", "segment_id", *[f"{p}{j}" for p in ("q", "qd", "qdd_est", "effort_filtered") for j in range(6)]]
DEFAULTS = dict(cutoff_hz=2.0, filter_order=4, edge_trim_s=0.5,
                maximum_resample_rate_hz=100.0, gap_periods=3.0)


def sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_metadata(csv_path: str | Path) -> dict[str, Any]:
    path = Path(csv_path).with_suffix(".meta.yaml")
    meta = yaml.safe_load(path.read_text())
    supported_schemas = {
        "rebot_hardware_experiment_v1",
        "rebot_hardware_experiment_v2",
    }
    if not isinstance(meta, dict) or meta.get("schema_version") not in supported_schemas:
        raise ValueError(f"{path}: expected supported rebot hardware experiment metadata")
    if meta.get("robot") != "rebot_dm":
        raise ValueError("only six-axis rebot_dm experiments are supported")
    for name in ("joint_direction", "joint_offset_rad"):
        values = np.asarray(meta.get(name, []), dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError(f"metadata requires six finite {name} values")
    if not np.isin(meta["joint_direction"], [-1, 1]).all():
        raise ValueError("joint_direction must contain signs")
    return meta


def _settings(options: dict | None) -> dict:
    options = options or {}
    if set(options) - set(DEFAULTS):
        raise ValueError(f"unknown preprocessing settings: {set(options) - set(DEFAULTS)}")
    result = {**DEFAULTS, **options}
    if any(not np.isfinite(float(v)) or float(v) <= 0 for v in result.values()):
        raise ValueError("preprocessing settings must be positive and finite")
    if result["filter_order"] != 4:
        raise ValueError("v1 uses filter_order=4")
    if result["gap_periods"] <= 1:
        raise ValueError("gap_periods must exceed one normal feedback period")
    return result


def preprocess(csv_path: str | Path, output: str | Path, *, options: dict | None = None,
               frozen_rate_hz: float | None = None) -> dict:
    """Select valid six-axis segments, resample, filter and differentiate measured qd.

    Host receive time is the time base. Lower time only detects repeated publications;
    neither timestamp is claimed to be synchronized per-motor hardware sampling time.
    A bad row breaks a segment, so interpolation cannot turn invalid data into valid data.
    """
    csv_path, output = Path(csv_path).resolve(), Path(output).resolve()
    sidecar = output.with_suffix(".meta.yaml")
    if output.exists() or sidecar.exists():
        raise FileExistsError(f"processed output already exists: {output}")
    meta = read_metadata(csv_path)
    settings = _settings(options)
    age_limit = float(meta["maximum_feedback_age_ms"])
    if not np.isfinite(age_limit) or age_limit <= 0:
        raise ValueError("invalid maximum_feedback_age_ms in raw metadata")
    with csv_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {*SIGNALS, "timestamp_host_rx_ns", "timestamp_lower_ns", "control_mode",
                    "command_valid", "servo_active", "primary_fault_code", "safety_state",
                    *[f"{p}{j}" for p in ("feedback_valid", "torque_valid", "feedback_age_ms") for j in range(6)]}
        if not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError("missing or duplicate raw CSV header")
        if required - set(reader.fieldnames):
            raise ValueError(f"missing raw columns: {sorted(required - set(reader.fieldnames))}")
        rows = list(reader)
    if not rows:
        raise ValueError("empty raw experiment")
    host = np.array([int(r["timestamp_host_rx_ns"]) for r in rows], dtype=np.int64)
    lower = np.array([int(r["timestamp_lower_ns"]) for r in rows], dtype=np.int64)
    if (host <= 0).any() or (lower <= 0).any() or (np.diff(host) < 0).any() or (np.diff(lower) < 0).any():
        raise ValueError("nonpositive or backwards host/lower timestamp; never reorder a run")
    signals = np.array([[float(r[k]) for k in SIGNALS] for r in rows])
    counts: Counter = Counter()
    accepted = []
    segment = 0
    # Group repeated lower timestamps before emitting any sample. Inconsistent groups
    # are discarded in full rather than retaining an arbitrary changed measurement.
    first = 0
    while first < len(rows):
        end = first + 1
        while end < len(rows) and lower[end] == lower[first]:
            end += 1
        group = signals[first:end]
        if not np.allclose(group, group[0], rtol=0, atol=0, equal_nan=True):
            counts["inconsistent_repeated_feedback"] += end - first
            segment += 1
            first = end
            continue
        used = False
        for i in range(first, end):
            row = rows[i]
            reason = None
            if row["control_mode"] != "excitation" or row["command_valid"] != "1" or row["servo_active"] != "1":
                reason = "outside_active_excitation"
            elif int(row["primary_fault_code"]) != 0 or row["safety_state"] not in {"ready", "moving"}:
                reason = "fault_or_unsafe_state"
            elif any(row[f"{p}{j}"] != "1" for p in ("feedback_valid", "torque_valid") for j in range(6)):
                reason = "invalid_feedback_or_torque"
            elif not np.isfinite(signals[i]).all():
                reason = "nonfinite_signal"
            else:
                ages = np.array([float(row[f"feedback_age_ms{j}"]) for j in range(6)])
                if not np.isfinite(ages).all() or (ages < 0).any() or (ages > age_limit).any():
                    reason = "stale_or_invalid_age"
                elif i and host[i] == host[i - 1]:
                    reason = "duplicate_host_timestamp"
            if reason:
                counts[reason] += 1
                segment += 1
            elif used:
                counts["duplicate_feedback"] += 1
            else:
                accepted.append((i, segment))
                used = True
        first = end
    if len(accepted) < 3:
        raise ValueError(f"insufficient fresh valid excitation feedback: {dict(counts)}")
    indices = np.array([x[0] for x in accepted])
    groups = np.array([x[1] for x in accepted])
    times = (host[indices] - host[0]).astype(float) * 1e-9
    deltas = np.diff(times)
    local_deltas = deltas[(np.diff(groups) == 0) & (deltas > 0)]
    if local_deltas.size == 0:
        raise ValueError("no contiguous valid feedback segment")
    period = float(np.median(local_deltas))
    effective_rate = 1 / period
    # Round down to a whole Hz to avoid reacting to microsecond network jitter.
    candidate = float(min(settings["maximum_resample_rate_hz"], np.floor(effective_rate)))
    rate = candidate if frozen_rate_hz is None else float(frozen_rate_hz)
    if not np.isfinite(rate) or rate <= 0 or rate > settings["maximum_resample_rate_hz"]:
        raise ValueError("invalid frozen resampling rate")
    if rate > effective_rate * 1.01:
        raise ValueError("B feedback bandwidth cannot support A's frozen resampling rate")
    if settings["cutoff_hz"] >= rate / 2:
        raise ValueError("feedback rate cannot support configured filter cutoff")
    sos = butter(4, settings["cutoff_hz"], fs=rate, output="sos")
    padlen = 3 * (2 * len(sos) + 1)
    trim = max(int(np.ceil(settings["edge_trim_s"] * rate)), padlen)
    cuts = np.flatnonzero((np.diff(groups) != 0) | (deltas > settings["gap_periods"] * period)) + 1
    chunks = np.split(np.arange(len(indices)), cuts)
    output_rows = []
    consistency = []
    kept_segments = 0
    for chunk in chunks:
        t = times[chunk]
        grid = t[0] + np.arange(int(np.floor((t[-1] - t[0]) * rate)) + 1) / rate
        if len(grid) <= 2 * trim + 2:
            counts["short_segment_fresh_samples"] += len(chunk)
            continue
        values = signals[indices[chunk]]
        uniform = np.column_stack([np.interp(grid, t, values[:, j]) for j in range(18)])
        filtered = sosfiltfilt(sos, uniform, axis=0, padlen=padlen)
        qdd = np.gradient(filtered[:, 6:12], 1 / rate, axis=0, edge_order=2)
        velocity_from_q = np.gradient(filtered[:, :6], 1 / rate, axis=0, edge_order=2)
        valid = slice(trim, -trim)
        consistency.append((velocity_from_q - filtered[:, 6:12])[valid])
        result = np.column_stack((grid, np.full(len(grid), kept_segments),
                                  filtered[:, :12], qdd, filtered[:, 12:18]))[valid]
        output_rows.extend(result.tolist())
        kept_segments += 1
    if not output_rows:
        raise ValueError(f"no segment long enough for filtering and boundary trim: {dict(counts)}")
    result_array = np.asarray(output_rows)
    if not np.isfinite(result_array).all():
        raise ValueError("preprocessing produced nonfinite data")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(COLUMNS)
        writer.writerows(output_rows)
    differences = np.concatenate(consistency)
    report = dict(schema_version=SCHEMA, robot="rebot_dm", torque_source="reported_effort",
                  torque_calibrated=False, qdd_source="derivative_of_zero_phase_filtered_sdk_qd",
                  time_source="timestamp_host_rx_ns", timestamp_synchronization="not_device_synchronized",
                  coordinate_policy="preserve_runner_coordinates_no_second_mapping",
                  raw_csv=str(csv_path), raw_sha256=sha256(csv_path),
                  raw_metadata_sha256=sha256(csv_path.with_suffix(".meta.yaml")),
                  raw_metadata=meta, processed_sha256=sha256(output), settings=settings,
                  resample_rate_hz=rate, effective_feedback_rate_hz=effective_rate,
                  raw_sample_count=len(rows), fresh_valid_sample_count=len(indices),
                  output_sample_count=len(output_rows), segment_count=kept_segments,
                  excluded_counts=dict(counts), trim_samples_per_segment_end=trim,
                  feedback_dt_s=dict(median=period, p95=float(np.percentile(deltas, 95)),
                                     p99=float(np.percentile(deltas, 99)), maximum=float(deltas.max())),
                  qd_consistency_rmse_rad_s=np.sqrt(np.mean(differences ** 2, axis=0)).tolist())
    sidecar.write_text(yaml.safe_dump(report, sort_keys=False))
    return report
