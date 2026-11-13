#!/usr/bin/env bash
# Creates double-click launchers on the Desktop and in the app menu:
#   "PHLAI RotorPy" (voice demo) and "PHLAI Showcase" (choreographed multi-drone demo).
# It opens a terminal running launch.sh (which sets everything up on first use, then runs the demo).
# setup_linux.sh calls this; re-run it if you move the project folder.   Usage:  bash make_launcher.sh
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Desktop Entry spec: inside a quoted Exec argument, escape " ` $ and \ with a backslash.
quoted() { printf '"%s"' "$(printf '%s' "$1" | sed 's/[\\"`$]/\\&/g')"; }

entry() {  # entry <name> <comment> [launch.sh args...]
  local name="$1" comment="$2"; shift 2
  local args=""
  for a in "$@"; do args+=" $(quoted "$a")"; done
  cat <<ENTRY
[Desktop Entry]
Type=Application
Version=1.0
Name=$name
Comment=$comment
Exec=bash $(quoted "$PROJECT_DIR/launch.sh")$args
Path=$PROJECT_DIR
Icon=$PROJECT_DIR/phlai_icon.svg
Terminal=true
Categories=Science;
StartupNotify=true
ENTRY
}

install() {  # install <file name> <entry args...>
  local file="$1"; shift
  local apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
  mkdir -p "$apps"
  entry "$@" > "$apps/$file"
  chmod +x "$apps/$file"
  echo "  app menu: $apps/$file"

  local desk
  desk="$(xdg-user-dir DESKTOP 2> /dev/null || echo "$HOME/Desktop")"
  if [ -d "$desk" ]; then
    entry "$@" > "$desk/$file"
    chmod +x "$desk/$file"
    # GNOME only runs Desktop launchers marked trusted ("Allow Launching").
    gio set "$desk/$file" metadata::trusted true 2> /dev/null || true
    echo "  desktop:  $desk/$file"
  fi
}

install phlai-rotorpy.desktop "PHLAI RotorPy" "Voice-controlled quadcopters in the RotorPy simulator"
install phlai-showcase.desktop "PHLAI Showcase" "Choreographed 3-drone RotorPy demo (no voice needed)" --showcase
