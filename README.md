# PHLAI Voice Command

Hold SPACE -> speak -> release. Audio goes to Whisper, the text is checked against a
whitelist, and an accepted command is published to ROS 2 as JSON. A ROS 2 adapter turns it
into velocity commands for a simulated Gazebo quadcopter, which can also draw glowing trails
(e.g. spell "PHLAI" in the air).

    mic -> WAV -> Whisper -> whitelist (commands.py) -> JSON file + /phlai/voice_command
                                                              |
                                          drone_adapter.py -> Twist -> Gazebo quadcopter
                                                              |
                                                gz_trail.py -> trail markers in Gazebo

## Files
| File | Job |
|---|---|
| `commands.py` | **The whitelist.** Spoken phrases -> command name + params. Edit this to add commands. |
| `config.py` | Paths, mic, Whisper model/device, ROS topic. |
| `matcher.py` | Whitelist matching (pure Python). `python3 matcher.py` lets you type phrases to test. |
| `recorder.py` / `transcriber.py` | Mic capture / Whisper (faster-whisper by default). |
| `sinks.py` | Output: JSON files (always) + ROS 2 publisher (if rclpy is available). |
| `app.py` | The GUI.|
| `drone_adapter.py` | ROS 2 node: voice JSON -> `geometry_msgs/Twist`. The only drone-specific file. |
| `gz_trail.py` | Optional Gazebo trail effect, used by `drone_adapter.py` when the Gazebo Python bindings are installed. |
| `run_demo.sh` | Launches sim, bridge, adapter and GUI in one go. |
| `setup_linux.sh` | One-shot install of ROS 2 Jazzy, Gazebo Harmonic, ros_gz and the Python environment. |
| `test_matcher.py` | Offline whitelist tests. |
| `preview_maneuvers.py` | Plots every drawing maneuver to `maneuvers.png` (needs matplotlib). |
| `matcher_strict.py` / `app_strict.py` / `test_safety_matcher.py` | Experimental strict matcher, its GUI, and its tests (see below). |

## Setup (Ubuntu 24.04 or WSL2 Ubuntu 24.04)
Easiest: `bash setup_linux.sh` installs ROS 2 Jazzy, Gazebo Harmonic, the ros_gz bridge and the Python environment in one go.
Manual equivalent:

    sudo apt install python3-tk python3-venv libportaudio2
    sudo apt install python3-gz-transport13 python3-gz-msgs10   # optional: trail effect
    source /opt/ros/jazzy/setup.bash
    python3 -m venv --system-site-packages .venv
    .venv/bin/pip install -r requirements.txt

The first run downloads the Whisper model, so do it once while online.
The GUI also runs without ROS (file output only) on any OS: `python3 app.py --no-ros`.

### Two Pythons, on purpose
- **Voice app and tests** run in the project `.venv` (`.venv/bin/python3`), which has faster-whisper.
- **`drone_adapter.py`** runs on the **system** Python (`/usr/bin/python3`). faster-whisper pulls a
  newer pip `protobuf` into the `.venv`, and that breaks Gazebo's apt-installed Python bindings
  (the trail silently stops drawing). The adapter needs nothing from the `.venv`.

`run_demo.sh` already does this for you.

## Running the demo
One command: `bash run_demo.sh` (add `-strict` for the strict matcher). Closing the voice window
stops everything; logs go to `logs/`.

Manual (four terminals, ROS sourced in each):
1. Sim: `gz sim -r multicopter_velocity_control.sdf` (Gazebo's quadcopter example world;
   `run_demo.sh` finds it automatically, or set `WORLD=/path/to/world.sdf`)
2. Bridge: `ros2 run ros_gz_bridge parameter_bridge /X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist`
3. Adapter: `/usr/bin/python3 drone_adapter.py`
4. Voice: `.venv/bin/python3 app.py`

Confirm the Twist topic name with `gz topic -l` and override if different:
`/usr/bin/python3 drone_adapter.py --ros-args -p cmd_vel_topic:=/your/topic` (and match it in the bridge).
The trail follows the model named `x3` by default (`-p trail_model:=<name>` to change it).

**Test the sim side before using your voice:**

    ros2 topic pub --once /phlai/voice_command std_msgs/msg/String \
      "{data: '{\"params\": {\"linear\": [0,0,1.0], \"angular_z\": 0.0, \"duration\": 3.0}}'}"

If the drone doesn't move, the velocity controller may need enabling:
`gz topic -t /X3/enable -m gz.msgs.Boolean -p "data: true"` (`run_demo.sh` does this).

## Notes
- Commands are timed and open-loop (e.g. takeoff = climb 1 m/s for 3 s). Tune speeds/durations at the top of `commands.py`.
- Bare words match anywhere in the sentence ("that's all right" -> `right`).
- Silence is skipped before Whisper (`SILENCE_RMS` in `config.py`) because Whisper can hallucinate text on silence.
- The trail is drawn with Gazebo visual markers, so it only shows in the Gazebo GUI window.
py
## Experimental: strict matcher
`matcher_strict.py` only accepts complete, approved requests. To try it:

    .venv/bin/python3 -B test_safety_matcher.py
    .venv/bin/python3 app_strict.py --no-ros

or `bash run_demo.sh -strict`, which runs the tests first and only launches if they pass.

## License
MIT - see [LICENSE](LICENSE).
