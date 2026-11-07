"""
PHLAI Voice Command - hold SPACE to talk, release to send.

  mic -> WAV -> Whisper -> whitelist match -> JSON file + ROS 2 topic

Run:  python3 app.py            (add --no-ros to skip ROS even if rclpy is installed)
"""
import argparse
import queue
import threading
import tkinter as tk
import wave
from tkinter import messagebox

import numpy as np

import config
from commands import COMMANDS
from matcher import make_message, match_command
from recorder import Recorder
from sinks import build_sinks
from transcriber import Transcriber

STATES = {
    "loading": ("#555555", "Loading Whisper..."),
    "ready": ("#37474f", "Hold SPACE to talk"),
    "recording": ("#c62828", "\u25cf RECORDING"),
    "processing": ("#b8860b", "Transcribing..."),
    "error": ("#4e342e", "Error"),
}
RELEASE_DEBOUNCE_MS = 40  # Linux/X11 fires fake release+press pairs while a key is held


class App:
    def __init__(self, root, use_ros=True):
        self.root = root
        root.title(config.APP_NAME)
        root.geometry("680x360")
        root.minsize(520, 300)

        self.ui_queue = queue.Queue()
        self.ready = False
        self.busy = False
        self.recording = False
        self.space_down = False
        self._release_job = None

        self.transcriber = Transcriber(config.WHISPER_MODEL, config.WHISPER_LANGUAGE)
        self.sinks, ros_status = build_sinks(use_ros)
        self.recorder = None

        self.banner = tk.Label(root, font=("Helvetica", 30, "bold"), fg="white", pady=34)
        self.banner.pack(fill="x")
        self.heard_var = tk.StringVar(value="")
        self.cmd_var = tk.StringVar(value="")
        tk.Label(root, textvariable=self.heard_var, font=("Helvetica", 14),
                 wraplength=620, pady=10).pack()
        tk.Label(root, textvariable=self.cmd_var, font=("Helvetica", 18, "bold")).pack()
        names = " \u00b7 ".join(c.name.replace("_", " ") for c in COMMANDS)
        tk.Label(root, text=ros_status, font=("Helvetica", 10), fg="#555").pack(side="bottom", pady=(0, 8))
        tk.Label(root, text=f"Commands: {names}", font=("Helvetica", 10), fg="#555",
                 wraplength=640).pack(side="bottom")

        root.bind_all("<KeyPress-space>", self._on_space_press)
        root.bind_all("<KeyRelease-space>", self._on_space_release)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.focus_force()

        self._set_state("loading")
        try:
            self.recorder = Recorder(config.SAMPLE_RATE, config.CHANNELS, config.MIC_DEVICE)
            self.recorder.open()
        except Exception as exc:
            self._set_state("error", "Microphone error")
            self.heard_var.set(str(exc))
        else:
            threading.Thread(target=self._load_model, daemon=True).start()
        self._poll()

    # ---------- UI helpers ----------
    def _set_state(self, state, text=None):
        color, default_text = STATES[state]
        self.banner.configure(bg=color, text=text or default_text)

    def _poll(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()
                if kind == "ready":
                    self.ready = True
                    self._set_state("ready")
                    self.heard_var.set(f"Whisper '{config.WHISPER_MODEL}' ready on {self.transcriber.device_label}")
                elif kind == "load_error":
                    self._set_state("error", "Whisper failed to load")
                    messagebox.showerror("Whisper", payload)
                elif kind == "result":
                    self._show_result(payload)
                elif kind == "error":
                    self.busy = False
                    self._set_state("ready")
                    self.heard_var.set(f"Processing failed: {payload}")
                    self.cmd_var.set("")
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _show_result(self, message):
        self.busy = False
        self._set_state("ready")
        heard = message["transcript"] or "(nothing heard)"
        self.heard_var.set(f'Heard: "{heard}"')
        if message["status"] == "accepted":
            self.cmd_var.set(f"\u2192 {message['command'].upper()}")
        else:
            self.cmd_var.set("No whitelisted command")

    # ---------- spacebar handling ----------
    def _on_space_press(self, event):
        if self._release_job is not None:      # auto-repeat: cancel the fake release
            self.root.after_cancel(self._release_job)
            self._release_job = None
            return
        if self.space_down:                    # auto-repeat (Windows/mac): ignore
            return
        self.space_down = True
        self._begin_recording()

    def _on_space_release(self, event):
        if not self.space_down:
            return
        self._release_job = self.root.after(RELEASE_DEBOUNCE_MS, self._finish_release)

    def _finish_release(self):
        self._release_job = None
        self.space_down = False
        self._end_recording()

    # ---------- recording / processing ----------
    def _begin_recording(self):
        if not self.ready or self.busy or self.recording:
            return
        self.recorder.start()
        self.recording = True
        self._set_state("recording")

    def _end_recording(self):
        if not self.recording:
            return
        self.recording = False
        audio = self.recorder.stop()
        if len(audio) < int(config.SAMPLE_RATE * config.MIN_RECORD_SECONDS):
            self._set_state("ready")
            self.heard_var.set("Too short - hold SPACE while you speak")
            self.cmd_var.set("")
            return
        self.busy = True
        self._set_state("processing")
        threading.Thread(target=self._process, args=(audio,), daemon=True).start()

    def _load_model(self):
        try:
            self.transcriber.load()
            self.ui_queue.put(("ready", None))
        except Exception as exc:
            self.ui_queue.put(("load_error", f"{type(exc).__name__}: {exc}"))

    def _save_wav(self, audio):
        config.RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        pcm = (np.clip(audio, -1.0, 1.0) * 32767).astype(np.int16)
        with wave.open(str(config.LATEST_WAV), "wb") as wf:
            wf.setnchannels(config.CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(config.SAMPLE_RATE)
            wf.writeframes(pcm.tobytes())

    def _process(self, audio):
        try:
            rms = float(np.sqrt(np.mean(np.square(audio))))
            self._save_wav(audio)
            transcript = "" if rms < config.SILENCE_RMS else self.transcriber.transcribe(audio)
            command, normalized = match_command(transcript)
            message = make_message(transcript, normalized, command, rms)
            for sink in self.sinks:
                try:
                    sink.publish(message)
                except Exception as exc:
                    print(f"[sink error] {type(sink).__name__}: {exc}")
            self.ui_queue.put(("result", message))
        except Exception as exc:
            self.ui_queue.put(("error", f"{type(exc).__name__}: {exc}"))

    def _on_close(self):
        if self.recorder:
            self.recorder.close()
        for sink in self.sinks:
            sink.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-ros", action="store_true", help="file output only")
    args = parser.parse_args()
    root = tk.Tk()
    App(root, use_ros=not args.no_ros)
    root.mainloop()


if __name__ == "__main__":
    main()
