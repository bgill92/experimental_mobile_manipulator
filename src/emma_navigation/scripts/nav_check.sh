#!/usr/bin/env bash
# Headless navigation check: starts the living-room sim with Nav2 on the saved map, sends one goal
# and exits 0 only if Nav2 succeeds and the ground-truth pose matches the goal. `ros2 launch` does
# not pass a node's exit code through, so the goal is sent from here in the foreground.
# Usage: nav_check.sh [timeout_s] (default 180).
set -eo pipefail  # No -u: colcon setup scripts read unset variables.

source install/setup.bash
limit="${1:-180}"
mkdir -p log
simlog="log/nav_check_sim.log"
# Own process group, so the cleanup reaches every node the launch started.
setsid ros2 launch emma_navigation navigation.launch.py headless:=true rviz:=false \
  > "${simlog}" 2>&1 &
sim=$!
cleanup() {
  kill -INT -- "-${sim}" 2>/dev/null || true
  for _ in $(seq 20); do kill -0 "${sim}" 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- "-${sim}" 2>/dev/null || true
}
trap cleanup EXIT

# Map frame = spawn pose (x forward, y left). The goal is 2.3 m to the right, in the passage
# between the sofa and the dining chairs, turned to face the spawn point.
status=0
# --kill-after: rclpy handles SIGTERM, and a check stuck waiting for Nav2 can ignore it.
timeout --kill-after=10 "${limit}" python "$(dirname "$0")/nav_check.py" -0.4 -2.3 1.5708 || status=$?
if [[ ${status} -eq 0 ]]; then
  echo "nav-check: PASS"
elif [[ ${status} -eq 124 || ${status} -eq 137 ]]; then
  echo "nav-check: FAIL (timed out after ${limit} s; sim log in ${simlog})"
else
  echo "nav-check: FAIL (sim log in ${simlog})"
fi
exit "${status}"
