# PHLAI Voice Command

Hold SPACE -> speak -> release. Audio goes to Whisper, the text is checked against a
whitelist, and an accepted command is written out as JSON. A [RotorPy](https://github.com/spencerfolk/rotorpy)
simulator picks it up and flies a simulated quadcopter in a live 3D window, leaving a glowing
trail where the "pen" is down (e.g. spell "PHLAI" in the air).

**This is the `rotorpy` branch.** It replaces Gazebo, the ros_gz bridge and ROS 2 with RotorPy,
a pure-Python quadrotor simulator. The Gazebo version lives on `main`.

    mic -> WAV -> Whisper -> whitelist (commands.py) -> output/latest_command.json
                                                              |
                          rotorpy_sim.py: RotorPy physics + SE3 controller + live 3D view/trail

## Files
| File | Job |
|---|---|
| `commands.py` | **The whitelist.** Spoken phrases -> command name + params. Edit this to add commands. |
| `config.py` | Paths, mic, Whisper model/device, number of drones, (optional) ROS topic. |
| `matcher.py` | Whitelist matching (pure Python). `python3 matcher.py` lets you type phrases to test. |
| `recorder.py` / `transcriber.py` | Mic capture / Whisper (faster-whisper by default). |
| `sinks.py` | Output: JSON files (always) + ROS 2 publisher (only if rclpy is available; not needed here). |
| `app.py` | The voice GUI. Run this. |
| `rotorpy_sim.py` | The simulator: watches the JSON file, flies the drone in RotorPy, draws it and the trail. |
| `command_schedule.py` | Turns a command's params into a timed velocity schedule (shared, pure Python). |
| `launch.sh` | What the desktop icon runs: sets up on first use, then `run_demo.sh`. |
| `run_demo.sh` | Launches the sim and the voice GUI in one go. |
| `setup_linux.sh` | One-shot install of both Python environments + the desktop icon (safe to re-run). |
| `make_launcher.sh` / `phlai_icon.svg` | Creates the "PHLAI RotorPy" Desktop/app-menu icon. |
| `test_matcher.py` | Offline whitelist tests. |
| `preview_maneuvers.py` | Plots every drawing maneuver to `maneuvers.png` (needs matplotlib). |
| `matcher_strict.py` / `app_strict.py` / `test_safety_matcher.py` | Experimental strict matcher, its GUI, and its tests (see below). |

## One click
Run `bash setup_linux.sh` once (or just `bash launch.sh`, which runs it for you). After that,
double-click **PHLAI RotorPy** on the Desktop or open it from the app menu. A terminal opens and shows
progress, then the sim and voice windows appear. If something fails, the terminal stays open so you
can read why. Moved the folder? Run `bash make_launcher.sh` again.

If GNOME shows the icon as a plain text file, right-click it and choose **Allow Launching**.

## Setup (Linux, including WSL2)
`bash setup_linux.sh` installs everything, and skips anything already in place. Manual equivalent:

    sudo apt install python3-tk python3-venv libportaudio2
    python3 -m venv .venv          && .venv/bin/pip install -r requirements.txt
    python3 -m venv .venv-rotorpy  && .venv-rotorpy/bin/pip install -r requirements-rotorpy.txt

The first voice run downloads the Whisper model, so do it once while online.

### Two venvs, on purpose
- **`.venv`**: voice app and tests (faster-whisper, sounddevice).
- **`.venv-rotorpy`**: the simulator. RotorPy pulls in numpy 2 / scipy / matplotlib, kept apart from
  the voice app. Create it **without** `--system-site-packages`: Ubuntu's apt matplotlib is built
  for numpy 1.x and crashes on import next to RotorPy's numpy.

`run_demo.sh` uses the right one for each.

## Running the demo
One command: `bash run_demo.sh` (add `-strict` for the strict matcher, `--vehicle crazyflie` for a
tiny drone). Two windows open: the RotorPy view and the voice GUI. Closing the voice window stops
both; the sim log goes to `logs/sim.log`.

Manual (two terminals):
1. Sim: `.venv-rotorpy/bin/python3 rotorpy_sim.py`
2. Voice: `.venv/bin/python3 app.py --no-ros`

**Test the sim without your voice:**

    .venv-rotorpy/bin/python3 rotorpy_sim.py --demo takeoff,spell_phlai,land

Headless (no window, ~4x faster than real time, saves a picture of the result):

    .venv-rotorpy/bin/python3 rotorpy_sim.py --headless --demo takeoff,up,star --save star.png

In the 3D view: each dark X is a drone with its number under it, the coloured line from its centre
is its nose, the + is the setpoint it is tracking, the faint line is its recent path and the dots are
its pen trail. Drag to rotate.

## Several drones
Three drones start side by side, 3 m apart (drone 1 on the left as seen from behind). Name the
drones at the start or end of what you say:

| Say | Who flies it |
|---|---|
| "take off" | every drone (no drone named = all) |
| "drone two, forward" | drone 2 |
| "drones one and three, land" / "drone 1 2 up" | drones 1 and 3 / 1 and 2 |
| "all drones spell fly" / "everyone hover" | every drone |
| "drone five, land" | nobody: rejected, because there is no drone 5 |

Whisper often writes "drone to" for "drone two" or "drone won" for "drone one". These sound-alikes
are understood right after the word "drone" only, so "drone one to the left" still means drone 1.
The voice window shows who a command went to, e.g. `→ FORWARD (DRONE 2)`.

Change the number of drones with `DRONE_COUNT` (and `DRONE_SPACING`) in `config.py`. Both the voice
app and the sim read it. Each drone gets its own trail colour, and "clear trail" only clears the
drones it is sent to. RotorPy simulates each drone on its own: there are no drone-to-drone
collisions or downwash, so drones can fly through each other.

Try targeting without your voice: `.venv/bin/python3 matcher.py` is the voice app's matcher
(type phrases), and the sim's `--demo` takes drone prefixes:

    .venv-rotorpy/bin/python3 rotorpy_sim.py --demo takeoff,1:letter_p,2:star,3:heart,1+3:land

The sim runs about 3x faster than real time with 3 drones on a typical laptop. If it falls behind,
the sim log says so; use fewer drones.

### Still want ROS?
`rotorpy_sim.py --ros` subscribes to `/phlai/voice_command` instead of watching the file (needs ROS 2
sourced; then run the voice app without `--no-ros`).

## Notes
- Commands are timed velocity schedules (e.g. takeoff = climb 1 m/s for 3 s). The sim integrates them into
  a position/yaw setpoint that RotorPy's SE3 controller tracks, so the drone holds position between
  commands instead of drifting. Tune speeds/durations at the top of `commands.py`.
- Bare words match anywhere in the sentence ("that's all right" -> `right`). Fine for a scripted demo; say the commands cleanly.
- Silence is skipped before Whisper (`SILENCE_RMS` in `config.py`) because Whisper can hallucinate text on silence.
- The trail follows the drone's real simulated position, so tracking error shows up in the drawing.
- Recording tip for the video: screen-record the GUI and the RotorPy window side by side (OBS); the banner turns red while you hold SPACE.

## Experimental: strict matcher
`matcher_strict.py` only accepts complete, approved requests (work in progress). To try it:

    .venv/bin/python3 -B test_safety_matcher.py
    .venv/bin/python3 app_strict.py --no-ros

or `bash run_demo.sh -strict`, which runs the tests first and only launches if they pass.

## License
MIT - see [LICENSE](LICENSE).
