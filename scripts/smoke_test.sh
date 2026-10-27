#!/usr/bin/env bash
# Launches the pretend arm without RViz and checks the controllers come up and the arm follows a command.
# Run in a sourced ROS 2 workspace:  scripts/smoke_test.sh
# NOT TESTED yet: the first run is in CI.
set -u

LOG="$(mktemp)"
ros2 launch sixeyes_bringup sixeyes.launch.py hardware:=mock use_rviz:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() { kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 2; kill -KILL "$LAUNCH_PID" 2>/dev/null; }
trap cleanup EXIT

fail() { echo "SMOKE TEST FAILED: $1"; echo "--- launch log (last 60 lines) ---"; tail -n 60 "$LOG"; exit 1; }

echo "Waiting for the controllers to become active (up to 90 s)..."
for _ in $(seq 1 45); do
  active=$(ros2 control list_controllers 2>/dev/null | grep -c "active")
  [ "$active" -ge 3 ] && break
  sleep 2
done
[ "${active:-0}" -ge 3 ] || fail "fewer than three controllers are active"
echo "All three controllers are active."

echo "Sending a small trajectory to the arm..."
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [base, shoulder, elbow, wrist_pitch, wrist_roll], points: [{positions: [0.3, 0.2, 0.4, 0.0, 0.0], time_from_start: {sec: 2}}]}" \
  > /dev/null 2>&1 || fail "could not publish the trajectory"
sleep 5

state=$(timeout 10 ros2 topic echo --once /joint_states 2>/dev/null) || fail "no /joint_states"
echo "$state" | grep -q "base" || fail "/joint_states has no base joint"
position=$(STATE="$state" python3 - <<'PY'
import os, re
text = os.environ["STATE"]
names = re.findall(r"^- (\w+)$", text.split("position:")[0], re.M)
values = [float(v) for v in re.findall(r"^- (-?[0-9.e+-]+)$", text.split("position:")[1].split("velocity:")[0], re.M)]
print(values[names.index("base")] if "base" in names and values else "nan")
PY
)
echo "base is at $position (asked for 0.3)"
python3 -c "import sys; sys.exit(0 if abs(float('$position') - 0.3) < 0.05 else 1)" || fail "the arm did not follow the command"
echo "SMOKE TEST PASSED"
