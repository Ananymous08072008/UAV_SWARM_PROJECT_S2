# Stage 2: Technical Challenge

PUSHPAK Grand Challenge 2026, "UAV-X: Resilient BVLOS Swarm Challenge".
Source: the official problem statement.

## Timeline

| Milestone | Date (2026) |
|---|---|
| Stage 2 work time | 3 Oct - 2 Dec |
| **Submission deadline** | **2 Dec** (on or before) |
| Results | 6 Dec |
| Grand Finale (IIT Bombay Techfest) | 16 - 18 Dec |

Qualified teams receive advanced disaster scenarios with **hidden disturbances**:
UAV failures, communication outages, packet loss and new emergency tasks.
The organizers also provide a standardized log format, to be adopted when published.

## Deliverables and where they live

| Required | Where / how |
|---|---|
| Final source code | this repository, tagged `v2.0.0` at submission |
| Technical report | `docs/proposal/proposal_outline.md` (extend into the final report) |
| Simulation logs | `results/<run>/` (`events.jsonl`, `events.csv`, `timeseries.csv`, `summary.json`) and `results/runs.sqlite` |
| Reproducibility instructions | section below, plus `docs/running.md` |
| Demonstration video | `python main.py --demo` (dashboard + MAVLink + real time); script in `docs/demo/demo_script.md` |

## Evaluation criteria and the metrics that cover them

| Criterion | Weight | Evidence in this repo |
|---|---|---|
| Mission completion | 25 % | completion rate / time, priority-weighted score in `summary.json` |
| Communication resilience | 25 % | PDR, latency, connectivity availability, downtime |
| Autonomous relay and role management | 20 % | relay changes / handovers / replans |
| Fault recovery and reconfiguration | 15 % | fault detection time, recovery time, reconnected UAVs |
| Safety and collision avoidance | 10 % | collisions, minimum separation, geofence violations |
| Innovation and technical merit | 5 % | `docs/architecture.md`, adaptive vs `--mode baseline` comparison |

## Reproducing a result

```bash
python -m venv .venv && .venv\Scripts\activate      # Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q                                  # 262 tests
python main.py --seed 7 --scenario scenarios/uav_failure_rth.yaml        # adaptive
python main.py --seed 7 --scenario scenarios/uav_failure_rth.yaml --mode baseline
python experiments/run_experiments.py --seeds 3      # every scenario x adaptive/baseline
python experiments/plot_results.py                   # figures next to the CSV
```

Every run writes its own folder under `results/`; the same `--seed` reproduces the same run.

## High-fidelity stack: Gazebo + ArduPilot + ROS 2

`ros2/` holds the same mission on real ArduPilot SITL, Gazebo Harmonic and ROS 2 Humble. It is
built up level by level (0 bench, 1 flight, 2 camera, 3 companion computer, 4 two UAVs +
communication), plus a 3-D town map with hidden disturbances (`demo3d`). Each level runs with
`bash ros2/run_level.sh <level>` and ends with a referee verdict. Setup, results and the
Level 5 (relay) design are in [ros2/README.md](../ros2/README.md). The radio model and the
mission constraints are the same as here.

## Working plan

- [ ] Add hidden-disturbance scenarios (UAV failure bursts, comm outage windows, packet-loss spikes, mid-mission emergency PoIs)
- [ ] Adopt the organizers' standard log format once released
- [ ] Re-run the full experiment matrix and refresh result tables for the report
- [ ] Pin exact dependency versions (`pip freeze`) for the submission snapshot
- [ ] Extend the technical report, record the demo video, tag `v2.0.0`

Track each item as an issue (use the "Feature or task" form) and land it through a pull request.
