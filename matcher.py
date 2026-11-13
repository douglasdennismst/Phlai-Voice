"""Whitelist matching: transcript text -> a VoiceCommand or None. Pure Python, no dependencies."""
import re
import uuid
from datetime import datetime

import config
from commands import COMMANDS

# Spoken drone numbers, including words Whisper hears instead ("drone to" = drone 2).
NUMBER_WORDS = {
    "1": 1, "one": 1, "won": 1,
    "2": 2, "two": 2, "to": 2, "too": 2,
    "3": 3, "three": 3, "tree": 3, "free": 3,
    "4": 4, "four": 4, "for": 4, "fore": 4,
    "5": 5, "five": 5, "6": 6, "six": 6, "7": 7, "seven": 7,
    "8": 8, "eight": 8, "ate": 8, "9": 9, "nine": 9,
}
SOUND_ALIKES = {"won", "to", "too", "tree", "free", "for", "fore", "ate"}
_NUM = "|".join(sorted(NUMBER_WORDS, key=len, reverse=True))
_REAL_NUM = "|".join(sorted(set(NUMBER_WORDS) - SOUND_ALIKES, key=len, reverse=True))
# "drone two", "drones one and three", "drone 1 2", "drones one, two and three".
# Sound-alikes only count right after "drone": "drone one to the left" is drone 1, not 1 and 2.
_TARGET_RE = re.compile(rf"\b(?:drone|drones|drown|droned)\s+((?:{_NUM})(?:\s+(?:and\s+)?(?:{_REAL_NUM}))*)\b")
_ALL_RE = re.compile(r"\b(?:all|every|both)\s+(?:the\s+)?drones?\b|\b(?:everyone|everybody|all drones)\b")


def normalize(text: str) -> str:
    text = text.lower().replace("-", " ")
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def match_target(normalized: str, drone_count=None):
    """Find which drone(s) a normalized transcript is addressed to.

    Returns (target, remaining_text, error):
      target          "all" or a sorted list of drone numbers (1-based)
      remaining_text  the text with the drone words removed, for command matching
      error           None, or why the target is invalid (e.g. "there is no drone 5")
    No drone named means "all".
    """
    count = config.DRONE_COUNT if drone_count is None else drone_count
    numbers = set()
    for m in _TARGET_RE.finditer(normalized):
        numbers.update(NUMBER_WORDS[w] for w in m.group(1).split() if w != "and")
    rest = _TARGET_RE.sub(" ", normalized)
    said_all = bool(_ALL_RE.search(rest))
    rest = re.sub(r"\s+", " ", _ALL_RE.sub(" ", rest)).strip()
    if not numbers:
        return "all", rest, None
    if said_all:
        return "all", rest, None
    bad = sorted(n for n in numbers if n > count)
    if bad:
        which = ", ".join(str(n) for n in bad)
        return sorted(numbers), rest, f"there is no drone {which} (drones are 1-{count})"
    return sorted(numbers), rest, None


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


def match(transcript: str, drone_count=None):
    """Target + command in one go. Returns (command or None, target, normalized, error)."""
    norm = normalize(transcript)
    target, rest, error = match_target(norm, drone_count)
    command, _ = match_command(rest)
    if error:
        command = None
    return command, target, norm, error


def make_message(transcript, normalized, command, rms=None, target="all", error=None) -> dict:
    message = {
        "id": str(uuid.uuid4()),
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "transcript": transcript,
        "normalized": normalized,
        "status": "accepted" if command else "rejected",
        "command": command.name if command else None,
        "params": dict(command.params) if command else {},
        "target": target,
        "audio_rms": None if rms is None else round(float(rms), 4),
    }
    if error:
        message["reason"] = error
    return message


if __name__ == "__main__":
    # Type phrases to test the whitelist without a microphone or Whisper.
    while True:
        try:
            line = input("say> ")
        except (EOFError, KeyboardInterrupt):
            break
        cmd, target, norm, error = match(line)
        who = "all drones" if target == "all" else "drone " + ", ".join(map(str, target))
        result = f"{cmd.name} -> {who}" if cmd else (error or "no whitelisted command")
        print(f"  -> {result}  ({norm!r})")
