"""Central settings for the PHLAI voice pipeline. Edit here, not in the other files."""
from pathlib import Path

APP_NAME = "PHLAI Voice Command"

BASE_DIR = Path(__file__).resolve().parent
RECORDINGS_DIR = BASE_DIR / "recordings"
OUTPUT_DIR = BASE_DIR / "output"
LATEST_WAV = RECORDINGS_DIR / "latest_recording.wav"
LATEST_JSON = OUTPUT_DIR / "latest_command.json"
HISTORY_JSONL = OUTPUT_DIR / "command_history.jsonl"

# Audio
SAMPLE_RATE = 16000          # Whisper's native rate
CHANNELS = 1
MIC_DEVICE = None            # None = system default; or an index/name from sounddevice.query_devices()
MIN_RECORD_SECONDS = 0.3     # shorter taps are ignored
SILENCE_RMS = 0.004          # quieter than this is treated as silence (Whisper hallucinates on silence)

# Whisper
WHISPER_MODEL = "base"
WHISPER_LANGUAGE = "en"
WHISPER_DEVICE = "cpu"       # "cpu", "cuda" or "auto" (try GPU, fall back to CPU)
WHISPER_BACKEND = "faster"
WHISPER_COMPUTE_TYPE = "int8"
WHISPER_BEAM_SIZE = 1
WHISPER_CPU_THREADS = 0

# Drones. Say "drone two, forward", "drones one and three, take off" or "all drones, land".
# A command with no drone named goes to every drone. Naming a drone above DRONE_COUNT is rejected.
DRONE_COUNT = 3
DRONE_SPACING = 3.0          # m between drones' start positions (side by side, along y)

# ROS 2 output (ignored automatically if rclpy is not importable)
ROS_ENABLED = True
ROS_NODE_NAME = "phlai_voice"
ROS_COMMAND_TOPIC = "/phlai/voice_command"   # std_msgs/String containing JSON
