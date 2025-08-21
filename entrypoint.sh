#!/usr/bin/env bash
# entrypoint.sh for FastBot ROS 2 CI (Humble)
# Works with /ros2_ws/src/fastbot and /ros2_ws/src/fastbot_waypoints

# --- Environment ---
# Source ROS 2 + your workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

set -x
set -e

# PIDs for cleanup
XVFB_PID=""
GAZEBO_PID=""
AS_PID=""
TEST_RESULT=1  # default to failure unless tests set it to 0

cleanup() {
  echo "[$(date '+%Y-%m-%d %T')] Cleaning up..."
  [[ -n "$AS_PID" ]] && kill "$AS_PID" 2>/dev/null || true
  [[ -n "$GAZEBO_PID" ]] && kill "$GAZEBO_PID" 2>/dev/null || true
  [[ -n "$XVFB_PID" ]] && kill "$XVFB_PID" 2>/dev/null || true
  # Just in case Gazebo left anything behind
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
}
trap cleanup EXIT

echo "PWD: $(pwd)"
echo "PATH: $PATH"
echo "DISPLAY: ${DISPLAY:-<unset>}; CI=${CI:-<unset>}"

# --- Headless display (only if needed) ---
# If running in Jenkins/CI or no DISPLAY is set, start Xvfb.
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

# Wait for ROS graph and Gazebo to be ready (with timeouts)
echo "[$(date '+%Y-%m-%d %T')] Waiting for ROS graph..."
for i in {1..60}; do
  if ros2 node list >/dev/null 2>&1; then break; fi
  sleep 1
done

echo "[$(date '+%Y-%m-%d %T')] Waiting for Gazebo (/gazebo node or /clock topic)..."
for i in {1..120}; do
  if ros2 node list | grep -q '/gazebo'; then break; fi
  if ros2 topic list | grep -q '^/clock$'; then break; fi
  sleep 1
done

# --- Launch action server ---
echo "[$(date '+%Y-%m-%d %T')] Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server >/tmp/action_server.log 2>&1 &
AS_PID=$!

echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot_as action..."
for i in {1..120}; do
  if ros2 action list | grep -q '^/fastbot_as$'; then break; fi
  sleep 1
done

echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot/odom topic..."
for _ in {1..120}; do
  ros2 topic list | grep -qx "/fastbot/odom" && break
  sleep 1
done

echo "[$(date '+%Y-%m-%d %T')] Waiting for first /fastbot/odom message..."
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
class W(Node):
  def __init__(self):
    super().__init__('wait_for_odom_once')
    self.create_subscription(Odometry, '/fastbot/odom', self.cb, 10)
  def cb(self, _):
    print("Received first /fastbot/odom", flush=True)
    rclpy.shutdown()
rclpy.init()
r=W(); rclpy.spin(r)
PY


# --- Run tests ---
echo "[$(date '+%Y-%m-%d %T')] Running colcon tests..."
set +e
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
TEST_RESULT=$?
colcon test-result --all || true
set -e

echo "[$(date '+%Y-%m-%d %T')] * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"

# Normal exit path (trap will still clean up)
exit $TEST_RESULT
