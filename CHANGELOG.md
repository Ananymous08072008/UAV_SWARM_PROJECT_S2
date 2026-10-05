# Changelog

All notable changes to this project are recorded here. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased] - Stage 2

### Added
- `docs/stage2.md`: Stage 2 plan, deliverable checklist and reproducibility steps.
- Repository governance: issue forms, `CODEOWNERS`,
  Dependabot config and `SECURITY.md`.

### Fixed
- CI smoke test ran `main.py --duration 30`, which the scenario validator now
  rejects (random PoI spawn window reaches 1800 s), so every CI run failed with
  exit code 2. It now runs a full headless mission in both `adaptive` and
  `baseline` modes.

### Changed
- CI: 25-minute job timeout, concurrent-run cancellation, read-only token permissions.
- Repository renamed for Stage 2 (`UAV_SWARM_PROJECT_S2`); Stage 1 history is kept intact.

## [stage1-baseline] - 2026-09-27

State of the Stage 1 submission (`UAV_SWARM_PROJECT_S1`, commit `1722209`).
Full suite: 262 tests passing.

### Added
- Resilient UAV swarm simulation platform with swarm intelligence engine,
  MAVLink gateway for Mission Planner and live dashboard.
- Mission studio (multi-session web app), Docker packaging, LAN access, Render blueprint.
- "Download data" export of runs as zip/xlsx with logs and metrics.
- Priority-based minimal fleet sizing; deferral of the farthest PoI for a
  dedicated multi-hop relay escort.
- Mission constraints (45-minute mission, 1000 x 1000 m area) and safe take-off/landing traffic.

### Changed
- Default mission duration 3600 s -> 2700 s; faster physics step for quicker runs.
