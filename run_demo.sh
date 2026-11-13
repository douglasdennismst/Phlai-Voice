#!/usr/bin/env bash
# Launch the whole demo: RotorPy sim window + voice GUI. No ROS or Gazebo needed.
#   bash run_demo.sh                    run everything
#   bash run_demo.sh -strict            run test_safety_matcher.py first, then launch app_strict.py
#   bash run_demo.sh --vehicle crazyflie    sim options (--vehicle, --drones, --ros) go to rotorpy_sim.py
#   bash run_demo.sh --showcase         play the choreographed multi-drone demo (sim only, no voice)
#   bash run_demo.sh --demo quick       play any demo from demos.py, or a list like takeoff,1:star,wait,all:land
#   other args are passed to the voice app
# The voice app writes every accepted command to output/latest_command.json; rotorpy_sim.py
# watches that file. Closing the voice window (or Ctrl-C) stops the sim too. Logs are in ./logs/.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$DIR/logs"
mkdir -p "$LOGS"

STRICT=0
DEMO=""
APP_ARGS=()
SIM_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -strict|--strict) STRICT=1 ;;
    --vehicle|--drones) SIM_ARGS+=("$1" "$2"); shift ;;
    --showcase) DEMO="showcase" ;;
    --demo) DEMO="$2"; shift ;;
    --ros) SIM_ARGS+=("$1") ;;
    *) APP_ARGS+=("$1") ;;
  esac
  shift
done
APP="app.py"
[ "$STRICT" -eq 1 ] && APP="app_strict.py"

# Two interpreters, on purpose:
#   PY     = .venv          -> voice app + tests (faster-whisper, sounddevice)
#   SIM_PY = .venv-rotorpy  -> RotorPy sim (rotorpy pins its own numpy/scipy/matplotlib)
PY="$DIR/.venv/bin/python3"
SIM_PY="$DIR/.venv-rotorpy/bin/python3"
# Demo mode: just the sim window playing a script. No microphone or Whisper needed.
if [ -n "$DEMO" ]; then
  if ! "$SIM_PY" -c "import rotorpy, matplotlib, tkinter" 2>/dev/null; then
    echo "The .venv-rotorpy isn't set up. Run setup_linux.sh first."; exit 1
  fi
  echo "Playing demo '$DEMO' (close the window to stop)"
  cd "$DIR"
  exec "$SIM_PY" rotorpy_sim.py --demo "$DEMO" "${SIM_ARGS[@]}"
fi

if [ ! -x "$PY" ] || [ ! -x "$SIM_PY" ]; then
  echo "Missing .venv or .venv-rotorpy. Run setup_linux.sh first."; exit 1
fi
if ! "$PY" -c "import faster_whisper, sounddevice, numpy" 2>/dev/null; then
  echo "The .venv is missing a package. Fix with: $PY -m pip install -r $DIR/requirements.txt"; exit 1
fi
if ! "$SIM_PY" -c "import rotorpy, matplotlib, tkinter" 2>/dev/null; then
  echo "The .venv-rotorpy can't import rotorpy/matplotlib/tkinter."
  echo "Fix with: $SIM_PY -m pip install -r $DIR/requirements-rotorpy.txt  (and: sudo apt install python3-tk)"; exit 1
fi

if [ "$STRICT" -eq 1 ]; then
  echo "Strict mode: running test_safety_matcher.py ..."
  if ! (cd "$DIR" && "$PY" -B test_safety_matcher.py); then
    echo "Safety matcher tests FAILED - nothing was launched."; exit 1
  fi
  echo "Safety matcher tests passed."
fi

SIM_PID=""
cleanup() {
  trap - EXIT INT TERM
  [ -n "$SIM_PID" ] && kill -TERM -- "-$SIM_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "1/2 RotorPy sim (log: logs/sim.log)"
cd "$DIR"
setsid "$SIM_PY" rotorpy_sim.py "${SIM_ARGS[@]}" > "$LOGS/sim.log" 2>&1 &
SIM_PID=$!
sleep 2
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  echo "The sim exited at startup. Last lines of logs/sim.log:"; tail -n 15 "$LOGS/sim.log"; exit 1
fi

echo "2/2 Voice app: $APP (close its window to stop everything)"
"$PY" "$APP" "${APP_ARGS[@]}"
