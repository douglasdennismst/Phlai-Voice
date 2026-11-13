#!/usr/bin/env bash
# One-shot setup for Ubuntu/Debian (including WSL2): the voice app's Python environment, the
# RotorPy simulator's, and a double-click launcher. No ROS or Gazebo needed. Safe to re-run:
# anything already in place is skipped.   Usage:  bash setup_linux.sh
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

APT_PKGS=(python3-tk python3-venv libportaudio2)
MISSING=()
for pkg in "${APT_PKGS[@]}"; do
  dpkg -s "$pkg" > /dev/null 2>&1 || MISSING+=("$pkg")
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "==> System packages: ${MISSING[*]} (asks for your password)"
  sudo apt-get update
  sudo apt-get install -y "${MISSING[@]}"
else
  echo "==> System packages already installed"
fi

if "$PROJECT_DIR/.venv/bin/python3" -c "import faster_whisper, sounddevice, numpy" 2> /dev/null; then
  echo "==> .venv (voice app) already set up"
else
  echo "==> .venv: voice app"
  python3 -m venv "$PROJECT_DIR/.venv"
  "$PROJECT_DIR/.venv/bin/pip" install --upgrade pip
  "$PROJECT_DIR/.venv/bin/pip" install -r "$PROJECT_DIR/requirements.txt"
fi

if "$PROJECT_DIR/.venv-rotorpy/bin/python3" -c "import rotorpy, matplotlib, tkinter" 2> /dev/null; then
  echo "==> .venv-rotorpy (simulator) already set up"
else
  echo "==> .venv-rotorpy: RotorPy simulator (kept separate: it needs its own numpy/scipy/matplotlib)"
  rm -rf "$PROJECT_DIR/.venv-rotorpy"
  python3 -m venv "$PROJECT_DIR/.venv-rotorpy"
  "$PROJECT_DIR/.venv-rotorpy/bin/pip" install --upgrade pip
  "$PROJECT_DIR/.venv-rotorpy/bin/pip" install -r "$PROJECT_DIR/requirements-rotorpy.txt"
fi

echo "==> Double-click launcher"
bash "$PROJECT_DIR/make_launcher.sh" || echo "(could not create the launcher; run: bash run_demo.sh)"

cat <<MSG

Done. Double-click "PHLAI RotorPy" on your Desktop (or find it in the app menu), or run:
  bash "$PROJECT_DIR/run_demo.sh"
MSG
