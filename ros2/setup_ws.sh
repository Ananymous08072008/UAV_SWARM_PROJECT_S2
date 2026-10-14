#!/usr/bin/env bash
# Build the UAV-X ROS 2 workspace (run inside WSL Ubuntu-22.04 / Ubuntu 22.04).
#
#   bash ros2/setup_ws.sh            # first time and after changing msgs / CMake packages
#
# Creates ~/uavx_ws whose src/ links to this repository's ros2/src, so Python edits
# made in the repo (Windows or Linux side) take effect without rebuilding
# (--symlink-install). Needs the ArduPilot + Gazebo Harmonic + ROS 2 Humble workspace
# from ~/ardu_ws (env.sh) - see ros2/README.md.
set -euo pipefail

REPO_ROS2="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${UAVX_WS:-$HOME/uavx_ws}"
ARDU_ENV="${ARDU_ENV:-$HOME/ardu_ws/env.sh}"

if [[ ! -f "$ARDU_ENV" ]]; then
  echo "error: $ARDU_ENV not found (ArduPilot/Gazebo/ROS 2 workspace)" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "$ARDU_ENV"
set -u

mkdir -p "$WS/src"
ln -sfn "$REPO_ROS2/src" "$WS/src/uavx"
cd "$WS"
colcon build --symlink-install \
  --packages-select uavx_interfaces uavx_gazebo uavx_swarm \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo
echo "Built. In every new shell:  source $ARDU_ENV && source $WS/install/setup.bash"
