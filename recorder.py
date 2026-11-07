"""Microphone capture. The stream stays open; audio is only kept while start()..stop() is active."""
import threading

import numpy as np
import sounddevice as sd


class Recorder:
    def __init__(self, sample_rate, channels=1, device=None):
        self._chunks = []
        self._lock = threading.Lock()
        self._recording = False
        self._stream = sd.InputStream(
            samplerate=sample_rate,
            channels=channels,
            dtype="float32",
            device=device,
            callback=self._callback,
        )

    def open(self):
        self._stream.start()

    def close(self):
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass

    def _callback(self, indata, frames, time_info, status):
        if self._recording:
            with self._lock:
                self._chunks.append(indata[:, 0].copy())

    def start(self):
        with self._lock:
            self._chunks = []
        self._recording = True

    def stop(self) -> np.ndarray:
        self._recording = False
        with self._lock:
            chunks, self._chunks = self._chunks, []
        if not chunks:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(chunks).astype(np.float32)
