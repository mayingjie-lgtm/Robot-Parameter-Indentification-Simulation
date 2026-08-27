#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rebot_real.recorder import analyze_capture


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze a reBot-DM state-only capture CSV")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="optional YAML summary path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze_capture(args.csv)
    text = yaml.safe_dump(summary, sort_keys=False, allow_unicode=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
