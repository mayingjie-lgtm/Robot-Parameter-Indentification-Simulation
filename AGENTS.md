# Project Instructions

This repository implements robot dynamics parameter identification for simulation and reBot real-hardware data.

## Mandatory reading before changes

Read, in order:

1. `doc/ARCHITECTURE.md`
2. `doc/DEVELOPMENT_RULES.md`
3. `doc/REBOT_HARDWARE_CONTROL_CONTRACT.md` when touching real-hardware execution
4. `doc/REBOT_HARDWARE_DATA_CONTRACT.md` when touching real-hardware data semantics
5. `doc/REBOT_SYSTEM_IDENTIFICATION_GUIDE.md` for the supported operator workflow

Historical development-phase documents are not source of truth and are not required reading.

## External reBot SDK

The accepted external SDK checkout is configured by the operator-facing YAML. The current field value is:

`/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904`

Do not vendor, edit, or silently patch that external SDK from this repository.

## Source-of-truth priority

When descriptions conflict, use this order:

1. actual code/runtime behavior and captured evidence;
2. `AGENTS.md`;
3. `doc/ARCHITECTURE.md`;
4. `doc/DEVELOPMENT_RULES.md`;
5. hardware control/data contracts;
6. `doc/REBOT_SYSTEM_IDENTIFICATION_GUIDE.md`;
7. `README.md`;
8. historical Git history or deleted development notes.

Successful real runs and their `hardware.yaml`, `raw.meta.yaml`, frozen-artifact hashes, and preview-acceptance evidence override stale historical status prose.

## Before coding

For non-trivial changes, first state:

1. current behavior;
2. problem;
3. minimal proposed change;
4. files to modify;
5. files intentionally not modified;
6. risks;
7. validation method.

Do not ask the user for facts that can be established from code, configuration, or recorded evidence.

## High-risk semantics

Do not silently change the meaning of:

- `q`, `qd`, acceleration, effort/torque, timestamps, joint mapping, units, or frames;
- A-only fitting versus B-only validation;
- frozen trajectory artifacts, SHA-256 provenance, or preview acceptance;
- MoveJ preposition semantics;
- Servo position-command semantics;
- feedback freshness, Servo ownership, hard faults, or controlled cleanup;
- tracking warning semantics (monitor-only during formal excitation);
- recorder schema or preprocessing/identification target columns.

Any required semantic change must be explicit, documented, tested, and justified from source evidence.

## Architecture preference

Prefer the smallest compatible change. Reuse existing runner, recorder, preprocessing, and identification code. Do not introduce factory/manager/plugin frameworks or copy a verified Servo loop merely to make a new entry point look cleaner.

For formal reBot A/B operation, the supported operator-facing entry is:

```bash
python3 scripts/run_rebot_real_ab.py --config config/rebot_real_ab.yaml
```

The lower-level hardware, preprocessing, diagnostic, and identification scripts remain available for advanced/debug use. They are not the normal operator workflow.

## Real-hardware safety

Never infer hardware authorization from a simulation result. The formal real workflow must preserve:

- accepted frozen trajectory + matching metadata + accepted preview;
- preflight before client creation;
- explicit per-trajectory operator ENTER gate;
- MoveJ to excitation `q0`, settle, Servo excitation, controlled cleanup;
- immediate fail-safe for hard faults, communication/state-stream loss, invalid/stale feedback beyond the audited recovery semantics, Servo reject, or unexpected Servo ownership loss;
- A failure blocks B; B failure blocks identification.

`--mock` must never instantiate the real SDK client. `--preflight-only` must never instantiate any hardware client.

## Interpretation boundary

Real `effort_reported` is the SDK-reported joint-side effort estimate used as the current offline fitting target. It is not yet independently calibrated physical torque. Low reported-effort prediction error must not be reported as proof of physically correct inertial/friction parameters.
