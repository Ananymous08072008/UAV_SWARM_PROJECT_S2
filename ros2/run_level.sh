#!/usr/bin/env bash
# Run one UAV-X level end to end and print its verdict.
#
#   bash ros2/run_level.sh 0                  # bench check
#   bash ros2/run_level.sh 4 --gui            # two UAVs + radio, with the Gazebo window
#   bash ros2/run_level.sh demo3d --seed 42   # 3-D town, hidden disturbances, replayable
#   bash ros2/run_level.sh my.yaml            # any scenario file
#
# Each run gets a fresh directory (~/uavx_runs/<name>_<time>/): ArduPilot eeprom and
# logs, launch.log, events.jsonl, timeseries.csv, packets.csv, images/, map3d.png,
# map2d.png, hidden_plan.json (if any) and summary.json.
set -uo pipefail

LEVEL="${1:?usage: run_level.sh <0-4 | scenario name | scenario.yaml> [--gui] [--seed N]}"
shift
GUI=false
SEED=-1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI=true ;;
    --seed) SEED="${2:?--seed needs a number}"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

WS="${UAVX_WS:-$HOME/uavx_ws}"
set +u
source "${ARDU_ENV:-$HOME/ardu_ws/env.sh}"
source "$WS/install/setup.bash"
set -u

if [[ "$LEVEL" =~ ^[0-9]+$ ]]; then SCENARIO="level$LEVEL"; NAME="level$LEVEL"
elif [[ -f "$LEVEL" ]]; then SCENARIO="$(realpath "$LEVEL")"; NAME="$(basename "$LEVEL" .yaml)"
else SCENARIO="$LEVEL"; NAME="$LEVEL"; fi           # a scenario in uavx_swarm/config
RUN_DIR="$HOME/uavx_runs/${NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

# Leftovers from an interrupted run would steal the SITL / MAVLink ports.
pkill -x arducopter 2>/dev/null; pkill -f mavproxy.py 2>/dev/null; pkill -f micro_ros_agent 2>/dev/null
pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; sleep 1

echo "UAV-X $NAME -> $RUN_DIR"
START=$(date +%s)
setsid ros2 launch uavx_swarm uavx.launch.py scenario:="$SCENARIO" gui:="$GUI" run_dir:="$RUN_DIR" seed:="$SEED" \
  > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT -- -$LAUNCH_PID 2>/dev/null' INT TERM
# Live view of the level's own nodes (the simulator's output stays in launch.log).
tail -n +1 -f "$RUN_DIR/launch.log" --pid=$LAUNCH_PID \
  | grep --line-buffered -E "\[(bench_check|companion|gcs|radio_net|scenario_manager|mission_monitor|poi_detector)" \
  | sed -u -E 's/\x1b\[[0-9;]*m//g' &
wait $LAUNCH_PID
sleep 2
# make sure nothing is left behind
kill -TERM -- -$LAUNCH_PID 2>/dev/null; sleep 2; kill -KILL -- -$LAUNCH_PID 2>/dev/null
pkill -x arducopter 2>/dev/null; pkill -f mavproxy.py 2>/dev/null; pkill -f micro_ros_agent 2>/dev/null
pkill -f "gz sim" 2>/dev/null

echo
echo "wall time: $(( $(date +%s) - START )) s   run dir: $RUN_DIR"
if [[ -f "$RUN_DIR/summary.json" ]]; then
  python3 - "$RUN_DIR/summary.json" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
print(f"LEVEL {s.get('level')} {s.get('name')}: {'PASS' if s.get('pass') else 'FAIL'}")
for c in s.get("results", s.get("criteria", [])):
    print(f"  {'PASS' if c['pass'] else 'FAIL'}  {c['check']:<34} {c.get('value')}")
PY
  python3 -c "import json,sys; sys.exit(0 if json.load(open('$RUN_DIR/summary.json')).get('pass') else 1)"
else
  echo "no summary.json - see $RUN_DIR/launch.log"
  exit 1
fi
