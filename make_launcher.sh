#!/usr/bin/env bash
# Creates a double-click "PHLAI RotorPy" launcher on the Desktop and in the app menu.
# It opens a terminal running launch.sh (which sets everything up on first use, then runs the demo).
# setup_linux.sh calls this; re-run it if you move the project folder.   Usage:  bash make_launcher.sh
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="phlai-rotorpy.desktop"

# Desktop Entry spec: inside a quoted Exec argument, escape " ` $ and \ with a backslash.
quoted() { printf '"%s"' "$(printf '%s' "$1" | sed 's/[\\"`$]/\\&/g')"; }

entry() {
  cat <<ENTRY
[Desktop Entry]
Type=Application
Version=1.0
Name=PHLAI RotorPy
Comment=Voice-controlled quadcopter in the RotorPy simulator
Exec=bash $(quoted "$PROJECT_DIR/launch.sh")
Path=$PROJECT_DIR
Icon=$PROJECT_DIR/phlai_icon.svg
Terminal=true
Categories=Science;
StartupNotify=true
ENTRY
}

APPS_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$APPS_DIR"
entry > "$APPS_DIR/$NAME"
chmod +x "$APPS_DIR/$NAME"
echo "  app menu: $APPS_DIR/$NAME"

DESKTOP_DIR="$(xdg-user-dir DESKTOP 2> /dev/null || echo "$HOME/Desktop")"
if [ -d "$DESKTOP_DIR" ]; then
  entry > "$DESKTOP_DIR/$NAME"
  chmod +x "$DESKTOP_DIR/$NAME"
  # GNOME only runs Desktop launchers marked trusted ("Allow Launching").
  gio set "$DESKTOP_DIR/$NAME" metadata::trusted true 2> /dev/null || true
  echo "  desktop:  $DESKTOP_DIR/$NAME"
fi
