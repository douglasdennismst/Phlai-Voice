"""Whitelist matching: transcript text -> a VoiceCommand or None. Pure Python, no dependencies."""
import re
import uuid
from datetime import datetime

from commands import COMMANDS


def normalize(text: str) -> str:
    text = text.lower().replace("-", " ")
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def match_command(transcript: str, commands=COMMANDS):
    """Return (VoiceCommand or None, normalized_text).

    Whole-word matching against the whitelist. A phrase that sits inside a longer
    matched phrase is ignored ("turn left" beats "left"). If several different
    commands are spoken, the earliest one wins.
    """
    norm = normalize(transcript)
    hits = []
    for cmd in commands:
        for phrase in cmd.phrases:
            p = normalize(phrase)
            for m in re.finditer(rf"\b{re.escape(p)}\b", norm):
                hits.append((m.start(), m.end(), cmd))

    def inside_longer(h):
        return any(
            o is not h and o[0] <= h[0] and h[1] <= o[1] and (o[1] - o[0]) > (h[1] - h[0])
            for o in hits
        )

    kept = [h for h in hits if not inside_longer(h)]
    if not kept:
        return None, norm
    kept.sort(key=lambda h: (h[0], -(h[1] - h[0])))
    return kept[0][2], norm


def make_message(transcript, normalized, command, rms=None) -> dict:
    return {
        "id": str(uuid.uuid4()),
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "transcript": transcript,
        "normalized": normalized,
        "status": "accepted" if command else "rejected",
        "command": command.name if command else None,
        "params": dict(command.params) if command else {},
        "audio_rms": None if rms is None else round(float(rms), 4),
    }


if __name__ == "__main__":
    # Type phrases to test the whitelist without a microphone or Whisper.
    while True:
        try:
            line = input("say> ")
        except (EOFError, KeyboardInterrupt):
            break
        cmd, norm = match_command(line)
        print(f"  -> {cmd.name if cmd else 'no whitelisted command'}  ({norm!r})")
