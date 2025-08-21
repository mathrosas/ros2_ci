#!/usr/bin/env bash
# entrypoint.sh for FastBot ROS 2 CI (Humble)
# Works with /ros2_ws/src/fastbot and /ros2_ws/src/fastbot_waypoints

# --- Environment ---
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

set -x
set -e

# (helps headless Gazebo behave)
export QT_QPA_PLATFORM=offscreen
export LIBGL_ALWAYS_SOFTWARE=1

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
for i in {1..60}; do
  if ros2 node list >/dev/null 2>&1; then break; fi
  sleep 1
done

# Wait for Gazebo (/gazebo or /clock)
echo "[$(date '+%Y-%m-%d %T')] Waiting for Gazebo (/gazebo node or /clock topic)..."
for i in {1..120}; do
  if ros2 node list | grep -q '/gazebo'; then break; fi
  if ros2 topic list | grep -q '^/clock$'; then break; fi
  sleep 1
done

# Unpause physics if available (Gazebo Classic starts paused sometimes)
if ros2 service list | grep -qx '/gazebo/unpause_physics'; then
  echo "[$(date '+%Y-%m-%d %T')] Unpausing Gazebo physics..."
  ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty '{}' || true
fi

# Wait for /fastbot/odom topic to appear
echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot/odom topic..."
for i in {1..120}; do
  if ros2 topic list | grep -qx '/fastbot/odom'; then break; fi
  sleep 1
done

# Wait for the FIRST /fastbot/odom message (no BrokenPipe)
echo "[$(date '+%Y-%m-%d %T')] Waiting for first /fastbot/odom message..."
set +e
python3 - <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

got_msg = False

class WaitOnce(Node):
    def __init__(self):
        super().__init__('wait_for_odom_once')
        self.sub = self.create_subscription(Odometry, '/fastbot/odom', self.cb, 10)
    def cb(self, msg):
        global got_msg
        got_msg = True
        # shutdown from within callback
        self.get_logger().info('Received first /fastbot/odom')
        rclpy.shutdown()

rclpy.init()
node = WaitOnce()
t_end = time.time() + 60.0  # 60s timeout
while rclpy.ok() and time.time() < t_end and not got_msg:
    rclpy.spin_once(node, timeout_sec=0.5)

# Exit 0 if we saw a message, else 1
sys.exit(0 if got_msg else 1)
PY
odom_rc=$?
set -e
if [[ $odom_rc -ne 0 ]]; then
  echo "No odom data received on /fastbot/odom"
  exit 22
fi

# Small settle to ensure steady publishing
sleep 2

# --- Launch action server ---
echo "[$(date '+%Y-%m-%d %T')] Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server >/tmp/action_server.log 2>&1 &
AS_PID=$!

echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot_as action..."
for i in {1..120}; do
  if ros2 action list | grep -qx '^/fastbot_as$'; then break; fi
  sleep 1
done
sleep 1

# --- Run tests ---
echo "[$(date '+%Y-%m-%d %T')] Running colcon tests..."
set +e
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all --verbose
TEST_RESULT=$?
set -e

echo "[$(date '+%Y-%m-%d %T')] * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"

# Normal exit path (trap will still clean up)
exit $TEST_RESULT
