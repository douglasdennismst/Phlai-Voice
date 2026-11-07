#!/usr/bin/env bash
# Launch the whole demo: Gazebo sim -> ros_gz bridge -> drone adapter -> voice GUI.
#   bash run_demo.sh                    run everything
#   bash run_demo.sh -strict            run test_safety_matcher.py first, then launch app_strict.py
#   bash run_demo.sh --no-ros           other args are passed to the app (combine with -strict if you like)
#   WORLD=/path/to/world.sdf bash run_demo.sh      use a specific world file
# Closing the voice window (or Ctrl-C) shuts everything else down. Logs are in ./logs/.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$DIR/logs"
mkdir -p "$LOGS"
TWIST_TOPIC="${TWIST_TOPIC:-/X3/gazebo/command/twist}"
ENABLE_TOPIC="${ENABLE_TOPIC:-/X3/enable}"

# Pull -strict / --strict out of the arguments; everything else goes to the app.
STRICT=0
APP_ARGS=()
for arg in "$@"; do
  case "$arg" in
    -strict|--strict) STRICT=1 ;;
    *) APP_ARGS+=("$arg") ;;
  esac
done
APP="app.py"
[ "$STRICT" -eq 1 ] && APP="app_strict.py"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ROS 2 Jazzy not found at /opt/ros/jazzy. Run setup_linux.sh first."; exit 1
fi
# Two interpreters, on purpose:
#   PY     = project .venv  -> voice app + tests (faster-whisper, sounddevice, numpy, rclpy)
#   SYS_PY = system Python  -> drone adapter (rclpy + Gazebo trail bindings)
# The adapter must NOT run in the .venv: faster-whisper pulls in a pip protobuf (7.x) that
# shadows Ubuntu's protobuf, and Gazebo's python3-gz-msgs10 bindings break under it
# (the trail silently stops drawing). The adapter needs nothing from the .venv anyway.
PY="$DIR/.venv/bin/python3"
SYS_PY="/usr/bin/python3"
if [ ! -x "$PY" ]; then
  echo "No .venv here. Run setup_linux.sh first."; exit 1
fi
if ! "$PY" -c "import faster_whisper, sounddevice, numpy, rclpy" 2>/dev/null; then
  echo "The .venv is missing a package. Check with:"
  echo "  $PY -c 'import faster_whisper, sounddevice, numpy, rclpy'"
  echo "Fix with: $DIR/.venv/bin/pip install faster-whisper sounddevice numpy"; exit 1
fi
if ! "$SYS_PY" -c "import rclpy" 2>/dev/null; then
  echo "System Python can't import rclpy (needed by drone_adapter.py). Is ROS 2 Jazzy installed?"; exit 1
fi
if ! "$SYS_PY" -c "import gz.transport13, gz.msgs10.marker_pb2" 2>/dev/null; then
  echo "Note: Gazebo trail bindings missing, so no trail effect."
  echo "      sudo apt install python3-gz-transport13 python3-gz-msgs10"
fi

WORLD="${WORLD:-$(find /usr /opt -name multicopter_velocity_control.sdf 2>/dev/null | head -n1)}"
if [ -z "$WORLD" ] || [ ! -f "$WORLD" ]; then
  echo "Could not find multicopter_velocity_control.sdf."
  echo "Find it yourself and run:  WORLD=/path/to/it.sdf bash run_demo.sh"; exit 1
fi

# Strict mode: the safety tests must pass before anything is launched.
if [ "$STRICT" -eq 1 ]; then
  for f in test_safety_matcher.py app_strict.py; do
    if [ ! -f "$DIR/$f" ]; then
      echo "Strict mode needs $f in $DIR"; exit 1
    fi
  done
  echo "Strict mode: running test_safety_matcher.py ..."
  if ! (cd "$DIR" && "$PY" -B test_safety_matcher.py); then
    echo "Safety matcher tests FAILED - nothing was launched."; exit 1
  fi
  echo "Safety matcher tests passed."
fi

PIDS=()
start() {  # start <name> <command...>   (own process group so cleanup can kill children too)
  local name="$1"; shift
  setsid "$@" > "$LOGS/$name.log" 2>&1 &
  PIDS+=($!)
  echo "  started $name (pid $!, log: logs/$name.log)"
}
cleanup() {
  trap - EXIT INT TERM
  echo; echo "Shutting down..."
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

echo "1/4 Gazebo: $WORLD"
start sim gz sim -r "$WORLD"

echo "    waiting for $TWIST_TOPIC ..."
for _ in $(seq 1 120); do
  gz topic -l 2>/dev/null | grep -qx "$TWIST_TOPIC" && break
  sleep 0.5
done
if ! gz topic -l 2>/dev/null | grep -qx "$TWIST_TOPIC"; then
  echo "Topic $TWIST_TOPIC never appeared. Last lines of the sim log:"
  tail -n 15 "$LOGS/sim.log"
  echo "Check 'gz topic -l' and rerun with TWIST_TOPIC=/your/topic bash run_demo.sh"
  exit 1
fi
gz topic -t "$ENABLE_TOPIC" -m gz.msgs.Boolean -p "data: true" > /dev/null 2>&1

echo "2/4 Bridge"
start bridge ros2 run ros_gz_bridge parameter_bridge "$TWIST_TOPIC@geometry_msgs/msg/Twist]gz.msgs.Twist"
sleep 2

echo "3/4 Drone adapter"
start adapter "$SYS_PY" "$DIR/drone_adapter.py" --ros-args -p cmd_vel_topic:="$TWIST_TOPIC"
sleep 1

echo "4/4 Voice app: $APP (close its window to stop everything)"
cd "$DIR"
"$PY" "$APP" "${APP_ARGS[@]}"
