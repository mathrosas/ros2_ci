#!/usr/bin/env bash
# entrypoint.sh for FastBot ROS 2 CI (Humble)

# --- Environment ---
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

set -e
set -x

# PIDs for cleanup
XVFB_PID=""
GAZEBO_PID=""
AS_PID=""
TEST_RESULT=1  # assume fail unless proven otherwise

cleanup() {
  echo "[$(date '+%Y-%m-%d %T')] Cleaning up..."
  [[ -n "$AS_PID" ]] && kill "$AS_PID" 2>/dev/null || true
  [[ -n "$GAZEBO_PID" ]] && kill "$GAZEBO_PID" 2>/dev/null || true
  [[ -n "$XVFB_PID" ]] && kill "$XVFB_PID" 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
}
trap cleanup EXIT

echo "PWD: $(pwd)"
echo "PATH: $PATH"
echo "DISPLAY: ${DISPLAY:-<unset>}; CI=${CI:-<unset>}"

# --- Headless display ---
if [[ -z "${DISPLAY:-}" || "${CI:-0}" = "1" ]]; then
  echo "[$(date '+%Y-%m-%d %T')] Starting Xvfb on :1 (headless mode)..."
  Xvfb :1 -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
  XVFB_PID=$!
  export DISPLAY=:1
fi

# --- Launch Gazebo world ---
echo "[$(date '+%Y-%m-%d %T')] Starting fastbot_gazebo..."
ros2 launch fastbot_gazebo one_fastbot_room.launch.py >/tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

# Wait for ROS graph
echo "[$(date '+%Y-%m-%d %T')] Waiting for ROS graph..."
for _ in {1..60}; do
  if ros2 node list >/dev/null 2>&1; then break; fi
  sleep 1
done

# Wait for Gazebo nodes (/gazebo) or /clock
echo "[$(date '+%Y-%m-%d %T')] Waiting for Gazebo (/gazebo or /clock)..."
for _ in {1..120}; do
  if ros2 node list | grep -q '/gazebo'; then break; fi
  if ros2 topic list | grep -q '^/clock$'; then break; fi
  sleep 1
done

# Unpause physics if service exists (Gazebo Classic)
if ros2 service list | grep -qx '/gazebo/unpause_physics'; then
  echo "[$(date '+%Y-%m-%d %T')] Unpausing Gazebo physics..."
  # Empty request for std_srvs/srv/Empty
  ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty '{}' || true
fi

# Wait for /fastbot/odom topic to appear
echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot/odom topic..."
for _ in {1..60}; do
  if ros2 topic list | grep -qx '/fastbot/odom'; then break; fi
  sleep 1
done

# Ensure at least ONE odom message is published before tests
echo "[$(date '+%Y-%m-%d %T')] Waiting for first /fastbot/odom message..."
timeout 60s bash -c 'ros2 topic echo /fastbot/odom | head -n 1 >/dev/null' || {
  echo "No odom data received on /fastbot/odom"
  exit 22
}
# Small settle to ensure steady publishing
sleep 2

# --- Launch action server ---
echo "[$(date '+%Y-%m-%d %T')] Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server >/tmp/action_server.log 2>&1 &
AS_PID=$!

echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot_as action..."
for _ in {1..120}; do
  if ros2 action list | grep -qx '/fastbot_as'; then break; fi
  sleep 1
done
sleep 1

# --- Run tests ---
echo "[$(date '+%Y-%m-%d %T')] Running colcon tests..."
set +e
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all --verbose
TEST_RESULT=$?   # this is non-zero if any test failed
set -e

echo "[$(date '+%Y-%m-%d %T')] * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"
exit $TEST_RESULT
