#!/usr/bin/env bash
# Headless pick-and-place check: starts the pick-scene sim and the block detector, runs the
# behaviour tree and exits with the tree's result (0 = block placed and arm home). `ros2 launch`
# does not pass a node's exit code through, so the tree runs here in the foreground instead of
# inside the launch. The tree and scene are saved to log/pick_check.rrd instead of a spawned Rerun
# viewer; replay with `pixi run rerun log/pick_check.rrd`.
# Usage: pick_check.sh [timeout_s] (default 240).
set -eo pipefail  # No -u: colcon setup scripts read unset variables.

source install/setup.bash
limit="${1:-240}"
mkdir -p log
simlog="log/pick_check_sim.log"
# Own process group, so the cleanup reaches every node the launch started.
setsid ros2 launch emma_behaviors pick_and_place.launch.py headless:=true rviz:=false \
  tree:=false > "${simlog}" 2>&1 &
sim=$!
cleanup() {
  kill -INT -- "-${sim}" 2>/dev/null || true
  for _ in $(seq 20); do kill -0 "${sim}" 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- "-${sim}" 2>/dev/null || true
}
trap cleanup EXIT

status=0
timeout "${limit}" ros2 run emma_behaviors pick_and_place --save log/pick_check.rrd \
  --ros-args -p use_sim_time:=true \
  || status=$?
if [[ ${status} -eq 0 ]]; then
  echo "pick-check: PASS (recording in log/pick_check.rrd)"
elif [[ ${status} -eq 124 ]]; then
  echo "pick-check: FAIL (timed out after ${limit} s)"
else
  echo "pick-check: FAIL (tree exit code ${status}; sim log in ${simlog})"
fi
exit "${status}"
