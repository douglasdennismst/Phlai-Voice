#!/usr/bin/env bash
# What the double-click launcher runs: set up on first use, then start the demo. If anything
# fails, the terminal stays open so you can read why.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

ready() {
  "$DIR/.venv/bin/python3" -c "import faster_whisper, sounddevice, numpy" 2> /dev/null &&
  "$DIR/.venv-rotorpy/bin/python3" -c "import rotorpy, matplotlib, tkinter" 2> /dev/null
}

if ! ready; then
  echo "First run: setting up (a few minutes; needs internet)."
  bash "$DIR/setup_linux.sh" || { echo; read -rp "Setup failed (see above). Press Enter to close."; exit 1; }
fi

bash "$DIR/run_demo.sh" "$@"
status=$?
if [ $status -ne 0 ]; then
  echo; read -rp "The demo stopped with an error (see above, and logs/). Press Enter to close."
fi
exit $status
