# Project Instructions

This repository is for robot dynamics parameter identification.

## Mandatory reading before work

Before modifying code, always read:

1. `doc/ARCHITECTURE.md`
2. `doc/DEVELOPMENT_RULES.md`

For the current development phase, also read:

3. `doc/PHASE1_MINIMAL_PLAN.md`
4. `doc/PHASE1_BASELINE.md` if it exists

Use `README.md` for build/run instructions.

Do not treat `plan.md` as the current architecture unless explicitly requested.

## Source-of-truth priority

When documents disagree, use this priority:

1. actual current code and reproducible runtime behavior
2. `AGENTS.md`
3. `doc/ARCHITECTURE.md`
4. `doc/DEVELOPMENT_RULES.md`
5. current phase plan
6. `README.md`
7. historical plans

Never silently assume a document describes current behavior.
If documentation and code disagree, report the mismatch before modifying code.

## Before coding

First report:

1. Current behavior
2. Problem
3. Minimal proposed change
4. Files to modify
5. Files intentionally not modified
6. Risks
7. Validation method

Do not begin large refactors before this analysis.

## Parameter-identification safety

Treat these as high-risk semantics:

- q
- qd
- qdd
- tau_cmd
- tau_effort
- torque source
- regressor
- parameter ordering
- inverse dynamics
- gravity/friction models

Never change or reinterpret these silently.

## Current project principle

Prefer:

understand baseline
→ verify data semantics
→ fix P0/P1 issues
→ integrate reBot minimally
→ refactor only when demonstrated necessary

Do not introduce factories, plugin systems, or generic robot frameworks only for possible future use.