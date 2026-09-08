#!/usr/bin/env python3
"""Generate synthetic hardware-format A/B from offline RNEA truth (no SDK)."""
from pathlib import Path
import argparse
import csv
import hashlib
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rebot_real.hardware_recorder import CSV_COLUMNS


def convert_truth(truth: Path, output: Path, label: str):
    with truth.open() as stream:
        rows = list(csv.DictReader(stream))
    records = []
    for i, source in enumerate(rows):
        stamp = 10**12 + round(float(source["time"]) * 1e9)
        # +/- 0.1 ms host receive jitter and occasional repeated publications.
        host_stamp = stamp + (i % 5 - 2) * 50000
        record = dict(sample_index=len(records), timestamp_host_rx_ns=host_stamp,
                      timestamp_lower_ns=stamp+10**11, timestamp_host_command_ns=host_stamp+1000,
                      servo_sequence=i+1, robot_mode="servo", safety_state="moving",
                      primary_fault_code=0, servo_active=1, servo_mode="position",
                      command_valid=1, control_mode="excitation")
        for j in range(6):
            record.update({f"q{j}": source[f"q{j}"], f"qd{j}": source[f"qd{j}"],
                           f"effort_reported{j}": source[f"tau{j}"], f"q_cmd{j}": source[f"q{j}"],
                           f"feedback_valid{j}": 1, f"torque_valid{j}": 1, f"feedback_age_ms{j}": 1.0})
        records.append(record)
        if i % 20 == 0:
            duplicate = dict(record, sample_index=len(records), timestamp_host_rx_ns=host_stamp+1000000)
            records.append(duplicate)
    with output.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(records)
    metadata = dict(schema_version="rebot_hardware_experiment_v1", robot="rebot_dm",
                    backend="synthetic_rnea_fixture", synthetic_data=True,
                    control_mode="excitation", control_rate_hz=100.0,
                    maximum_feedback_age_ms=50.0, joint_direction=[1]*6, joint_offset_rad=[0]*6,
                    j1_convention="SYNTHETIC_CANONICAL", joint_mapping_verified=False,
                    trajectory_hash=hashlib.sha256((label+truth.read_text()).encode()).hexdigest(),
                    q_source="analytic synthetic position", qd_source="analytic synthetic velocity",
                    effort_reported_source="synthetic RNEA + known actuator terms; not hardware",
                    observed_sample_count=len(records))
    output.with_suffix(".meta.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))


def generate(output: Path, build: Path):
    output, build = output.resolve(), build.resolve()
    if output.exists():
        raise FileExistsError("demo output directory already exists")
    output.mkdir(parents=True)
    subprocess.run([str(build / "rebot_reported_effort_fixture"), str(output / "truth")], check=True)
    for label in ("A", "B"):
        convert_truth(output / "truth" / f"{label}.truth.csv", output / f"{label}.raw.csv", label)
    config = yaml.safe_load((ROOT / "config/rebot_real_identification.yaml").read_text())
    config.update(training_raw_csv=str(output / "A.raw.csv"), validation_raw_csv=str(output / "B.raw.csv"),
                  identify_binary=str(build / "identify"), output_directory=str(output / "identified"))
    (output / "pipeline.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    print(f"Synthetic data only. Run: /usr/bin/python3 scripts/run_rebot_identification.py --config {output / 'pipeline.yaml'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--build-directory", type=Path, default=ROOT / "build_rebot")
    args = parser.parse_args()
    generate(args.output_directory, args.build_directory)
