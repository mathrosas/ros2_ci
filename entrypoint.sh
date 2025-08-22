#!/usr/bin/env bash
# entrypoint.sh for FastBot ROS 2 CI (Humble)
# Works with /ros2_ws/src/fastbot and /ros2_ws/src/fastbot_waypoints

# --- Environment ---
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Optional: make Qt/GL happy in headless containers
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}

set -x
set -e

# PIDs for cleanup
XVFB_PID=""
GAZEBO_PID=""
AS_PID=""
TEST_RESULT=1  # default to failure unless tests set it to 0

cleanup() {
  echo "[$(date '+%Y-%m-%d %T')] Cleaning up..."
  [[ -n "$AS_PID"     ]] && kill "$AS_PID"     2>/dev/null || true
  [[ -n "$GAZEBO_PID" ]] && kill "$GAZEBO_PID" 2>/dev/null || true
  [[ -n "$XVFB_PID"   ]] && kill "$XVFB_PID"   2>/dev/null || true
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
for _ in {1..60}; do
  if ros2 node list >/dev/null 2>&1; then break; fi
  sleep 1
done

echo "[$(date '+%Y-%m-%d %T')] Waiting for Gazebo (/gazebo node or /clock topic)..."
for _ in {1..120}; do
  if ros2 node list  | grep -q '/gazebo'; then break; fi
  if ros2 topic list | grep -q '^/clock$'; then break; fi
  sleep 1
done

# --- Launch action server ---
echo "[$(date '+%Y-%m-%d %T')] Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server >/tmp/action_server.log 2>&1 &
AS_PID=$!

echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot_as action..."
for _ in {1..120}; do
  if ros2 action list | grep -q '^/fastbot_as$'; then break; fi
  sleep 1
done

# --- Ensure odom is really publishing once (robust, non-hanging) ---
echo "[$(date '+%Y-%m-%d %T')] Waiting for /fastbot/odom topic..."
for _ in {1..120}; do
  if ros2 topic list | grep -qx "/fastbot/odom"; then break; fi
  sleep 1
done

echo "[$(date '+%Y-%m-%d %T')] Waiting for first /fastbot/odom message..."
set +e
timeout 30s python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from nav_msgs.msg import Odometry

def main():
    rclpy.init()
    node = Node('wait_for_odom_once')
    got = Future()

    def cb(_msg):
        print("Received first /fastbot/odom", flush=True)
        if not got.done():
            got.set_result(True)

    sub = node.create_subscription(Odometry, '/fastbot/odom', cb, 10)
    try:
        rclpy.spin_until_future_complete(node, got, timeout_sec=25.0)
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PY
ODOM_RC=$?
set -e

if [[ $ODOM_RC -ne 0 ]]; then
  echo "[$(date '+%Y-%m-%d %T')] WARN: odom wait script exited with $ODOM_RC (timeout or early exit). Continuing…"
fi

# --- Introspect: print the goal used by the test + one odom snapshot + deltas ---
echo "[$(date '+%Y-%m-%d %T')] Detecting test goal from source + sampling odom…"
set +e
timeout 40s python3 - <<'PY'
import re, math, pathlib, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# -------- 1) Parse goal_x / goal_y from the test source (non-commented lines) --------
goal_x = None
goal_y = None
src = pathlib.Path('/ros2_ws/src/fastbot_waypoints/test/test_waypoints.cpp')
if src.exists():
    for line in src.read_text().splitlines():
        s = line.strip()
        if s.startswith('//'):
            continue
        m = re.search(r'\bdouble\s+goal_x\s*=\s*([+-]?\d+(?:\.\d+)?)', s)
        if m:
            goal_x = float(m.group(1)); continue
        m = re.search(r'\bdouble\s+goal_y\s*=\s*([+-]?\d+(?:\.\d+)?)', s)
        if m:
            goal_y = float(m.group(1)); continue

print(f"TEST GOAL (from source): goal_x={goal_x} goal_y={goal_y}", flush=True)

# -------- 2) Take one odom sample and compute yaw --------
def yaw_from_quat(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Once(Node):
    def __init__(self):
        super().__init__('ci_odom_snapshot')
        self.sub = self.create_subscription(Odometry, '/fastbot/odom', self.cb, 10)
        self.pose = None

    def cb(self, msg):
        if self.pose is None:
            self.pose = msg.pose.pose

rclpy.init()
n = Once()
end = n.get_clock().now().nanoseconds + int(30e9)
while rclpy.ok() and n.pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.get_clock().now().nanoseconds > end:
        break

if n.pose is None:
    print("ODOM: no sample received within 30s", flush=True)
else:
    p = n.pose.position
    o = n.pose.orientation
    yaw = yaw_from_quat(o)
    print(f"ODOM now: x={p.x:.3f} y={p.y:.3f} yaw={math.degrees(yaw):.1f} deg", flush=True)

    if (goal_x is not None) and (goal_y is not None):
        dx = goal_x - p.x
        dy = goal_y - p.y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi
        print(f"Δ to goal: dx={dx:.3f} dy={dy:.3f} dist={dist:.3f} m ; heading_err={math.degrees(yaw_err):.1f} deg", flush=True)

# Cleanup
n.destroy_node()
rclpy.shutdown()
PY
ODOM_INTROSPECT_RC=$?
set -e
[[ $ODOM_INTROSPECT_RC -ne 0 ]] && echo "WARN: goal/odom introspection exited with rc=${ODOM_INTROSPECT_RC}"

# --- Run tests ---
echo "[$(date '+%Y-%m-%d %T')] Running colcon tests..."
set +e
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
# Prefer letting test-result decide pass/fail via its return code:
colcon test-result --all --verbose
TEST_RESULT=$?
set -e

# (Optional) Post-test odom snapshot to see where the robot ended up
echo "[$(date '+%Y-%m-%d %T')] Post-test odom snapshot…"
set +e
timeout 10s python3 - <<'PY'
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def yaw_from_quat(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Once(Node):
    def __init__(self):
        super().__init__('ci_odom_after_tests')
        self.sub = self.create_subscription(Odometry, '/fastbot/odom', self.cb, 10)
        self.pose = None
    def cb(self, msg):
        if self.pose is None:
            self.pose = msg.pose.pose

rclpy.init()
n = Once()
end = n.get_clock().now().nanoseconds + int(8e9)
while rclpy.ok() and n.pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.get_clock().now().nanoseconds > end:
        break

if n.pose is None:
    print("ODOM(after): no sample", flush=True)
else:
    p = n.pose.position; o = n.pose.orientation
    yaw = yaw_from_quat(o)
    print(f"ODOM(after): x={p.x:.3f} y={p.y:.3f} yaw={math.degrees(yaw):.1f} deg", flush=True)

n.destroy_node()
rclpy.shutdown()
PY
set -e

echo "[$(date '+%Y-%m-%d %T')] * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"

# Normal exit path (trap will still clean up)
exit $TEST_RESULT
