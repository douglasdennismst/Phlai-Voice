"""Speech-to-text wrapper. Takes a float32 16 kHz mono numpy array (no ffmpeg needed).

Backends (config.WHISPER_BACKEND):
  "faster" - faster-whisper (CTranslate2). Same Whisper models, much quicker on CPU. Default.
  "openai" - the original openai-whisper + PyTorch.
All settings are read from config.py with safe defaults, so older config files still work.
"""
import numpy as np

import config


class Transcriber:
    def __init__(self, model_name="tiny", language="en", device=None):
        self.model_name = model_name
        self.language = language
        if device is None:
            device = getattr(config, "WHISPER_DEVICE", "auto")
        self.requested_device = str(device).lower()   # "auto", "cpu" or "cuda"
        self.backend = getattr(config, "WHISPER_BACKEND", "faster")
        self.compute_type = getattr(config, "WHISPER_COMPUTE_TYPE", "int8")
        self.cpu_threads = getattr(config, "WHISPER_CPU_THREADS", 0)   # 0 = library default
        self.beam_size = getattr(config, "WHISPER_BEAM_SIZE", 1)       # 1 = greedy = fastest
        self.model = None
        self.device = "cpu"
        self.device_label = "CPU"

    # ---------- loading ----------
    def load(self):
        if self.backend == "openai":
            self._load_openai()
        else:
            self._load_faster()

    def _warmup(self, model):
        """First inference is slow (allocations); do it now so the first real command isn't."""
        segments, _ = model.transcribe(np.zeros(16000, dtype=np.float32),
                                       language=self.language, beam_size=1)
        list(segments)

    def _load_faster(self):
        from faster_whisper import WhisperModel

        if self.requested_device in ("auto", "cuda"):
            try:
                model = WhisperModel(self.model_name, device="cuda", compute_type=self.compute_type)
                self._warmup(model)           # CUDA problems often only show up here
                self.model, self.device = model, "cuda"
                self.device_label = f"GPU ({self.compute_type}, faster-whisper)"
                return
            except Exception:
                pass                          # fall back to CPU

        model = WhisperModel(self.model_name, device="cpu",
                             compute_type=self.compute_type, cpu_threads=self.cpu_threads)
        self._warmup(model)
        self.model, self.device = model, "cpu"
        self.device_label = f"CPU ({self.compute_type}, faster-whisper)"

    def _load_openai(self):
        import torch
        import whisper

        if self.requested_device in ("auto", "cuda") and self._cuda_usable(torch):
            try:
                self.model = whisper.load_model(self.model_name, device="cuda")
                self.device = "cuda"
                self.device_label = torch.cuda.get_device_name(0)
                return
            except Exception:
                pass
        self.model = whisper.load_model(self.model_name, device="cpu")
        self.device = "cpu"
        gpu_present = torch.cuda.is_available() and self.requested_device != "cpu"
        self.device_label = "CPU (GPU not usable with this PyTorch)" if gpu_present else "CPU"

    @staticmethod
    def _cuda_usable(torch):
        if not torch.cuda.is_available():
            return False
        try:
            x = torch.ones(8, device="cuda")
            (x + x).sum().item()
            return True
        except Exception:
            return False

    # ---------- transcribing ----------
    def transcribe(self, audio) -> str:
        # No initial_prompt on purpose: on silent audio Whisper can echo the prompt back,
        # which would turn silence into a whitelisted command.
        if self.backend == "openai":
            result = self.model.transcribe(
                audio, language=self.language, fp16=(self.device == "cuda"),
                temperature=0.0, condition_on_previous_text=False, verbose=None,
            )
            return result.get("text", "").strip()

        segments, _ = self.model.transcribe(
            audio, language=self.language, beam_size=self.beam_size,
            temperature=0.0, condition_on_previous_text=False,
        )
        return "".join(seg.text for seg in segments).strip()   # segments is a lazy generator
