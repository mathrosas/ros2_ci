#! /bin/bash
# ROS 2 FastBot Waypoints CI entrypoint

set -euo pipefail
set -x

# -------------------------
# 0) Env & helpers
# -------------------------
source /opt/ros/humble/setup.bash

# (Optional but helpful for headless runners)
export QT_QPA_PLATFORM=offscreen
export LIBGL_ALWAYS_SOFTWARE=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# Make sure we always clean up background processes
pids=()
cleanup() {
  for p in "${pids[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "PWD: $(pwd)"
echo "PATH: $PATH"

# -------------------------
# 1) Build fastbot_waypoints (as in README §6)
# -------------------------
cd /ros2_ws
colcon build --packages-select fastbot_waypoints
source /ros2_ws/install/setup.bash

# -------------------------
# 2) Launch simulation (README §4)
# -------------------------
echo "$(date +'[%F %T]') Launching Gazebo world..."
ros2 launch fastbot_gazebo one_fastbot_room.launch.py &
pids+=($!)
# Wait for ROS graph to respond
timeout 90s bash -c 'until ros2 node list >/dev/null 2>&1; do sleep 1; done'

# Wait for Gazebo to be up
timeout 120s bash -c 'until ros2 node list | grep -q "/gazebo"; do sleep 1; done' || {
  echo "Gazebo node not detected"; exit 20;
}

# Unpause physics if service exists (best-effort)
if ros2 service list | grep -q "/gazebo/unpause_physics"; then
  ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}" || true
fi

# Wait for /fastbot/odom topic to exist
timeout 120s bash -c 'until ros2 topic list | grep -qx "/fastbot/odom"; do sleep 1; done' || {
  echo "Topic /fastbot/odom not found. Available odom-like topics:"
  ros2 topic list | grep -i odom || true
  exit 21
}

# Wait to receive at least ONE odom message (confirms publishers + ticking sim)
timeout 60s bash -c 'ros2 topic echo /fastbot/odom -n 1 >/dev/null' || {
  echo "No odom data received on /fastbot/odom"; exit 22;
}

# -------------------------
# 3) Run action server (README §5)
# -------------------------
echo "$(date +'[%F %T]') Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server &
pids+=($!)

# Wait for the action server name to appear
timeout 120s bash -c 'until ros2 action list | grep -qx "/fastbot_as"; do sleep 1; done' || {
  echo "Action server /fastbot_as not available"; exit 23;
}

# (Optional) sanity probe
ros2 action info /fastbot_as || true

# -------------------------
# 4) Run tests (README §6)
# -------------------------
echo "$(date +'[%F %T]') Running tests..."
set +e
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
# Make the overall exit code reflect test results:
colcon test-result --all --verbose --fail-on-failed
TEST_RESULT=$?
set -e

echo "==== TEST SUMMARY ===="
colcon test-result --all || true
echo "======================"

echo " * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"

# -------------------------
# 5) Exit with real test status
# -------------------------
exit $TEST_RESULT
