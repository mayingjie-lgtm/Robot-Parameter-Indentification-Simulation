#!/usr/bin/env python3
"""Preprocess one recorded excitation without importing or connecting an SDK."""
import argparse
from pathlib import Path
import sys
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from rebot_real.preprocessing import preprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path, help="YAML preprocessing settings")
    parser.add_argument("--frozen-rate-hz", type=float, help="use A's frozen rate for B")
    args = parser.parse_args()
    report = preprocess(args.csv, args.output,
                        options=yaml.safe_load(args.config.read_text()) if args.config else None,
                        frozen_rate_hz=args.frozen_rate_hz)
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == "__main__":
    main()
