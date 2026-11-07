"""Experimental complete-request matcher; Douglas's matcher.py is unchanged.

Accept a configured alias, optionally with "please" before or after it.
Directional moves also allow "go" or "move", as in "please move forward".
Allow case differences, horizontal whitespace, final periods/exclamations,
and the existing "take-off" spelling. Reject other phrasing and punctuation.

This is deterministic text validation, not a natural-language safety guarantee.
It cannot recover negation omitted by speech recognition. Rejection returns no
new command; it does not create an emergency-stop command or a drawing plan.
"""
import json
import re
import uuid
from datetime import datetime

from commands import COMMANDS


_DIRECTIONAL_NAMES = frozenset(
    ("forward", "backward", "left", "right", "up", "down")
)


def normalize(text: str) -> str:
    # Keep the original normalization contract for comparable JSON logs.
    text = text.lower().replace("-", " ")
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def _request_key(transcript):
    """Normalize approved formatting without erasing unapproved syntax."""
    if re.search(r"[^A-Za-z \t.!-]", transcript) is not None:
        return None
    text = transcript.strip(" \t").lower()
    text = re.sub(r"\btake-off\b", "take off", text)
    text = re.sub(r"[.!]+$", "", text).rstrip(" \t")
    # Check BEFORE generic normalization: quotes, question marks, slashes,
    # symbols, and internal punctuation must not disappear into an alias.
    if re.fullmatch(r"[a-z]+(?:[ \t]+[a-z]+)*", text) is None:
        return None
    return " ".join(text.split())


def _request_forms(command):
    """Finite, explicitly supported full requests for one command."""
    forms = set()
    for phrase in command.phrases:
        alias = normalize(phrase)
        if not alias:
            continue
        cores = [alias]
        if command.name in _DIRECTIONAL_NAMES:
            cores.extend(("go " + alias, "move " + alias))
        for core in cores:
            for prefix in ("", "please "):
                for suffix in ("", " please"):
                    forms.add(prefix + core + suffix)
    return forms


def match_command(transcript: str, commands=COMMANDS):
    """Return (one unambiguous VoiceCommand or None, normalized_text).

    The entire approved request must match. No substring selection, command
    sequencing, fallback motion, or conversion of rejected requests to hover.
    """
    normalized = normalize(transcript)
    request = _request_key(transcript)
    if request is None:
        return None, normalized
    matches = [cmd for cmd in commands if request in _request_forms(cmd)]
    if len(matches) != 1:
        return None, normalized
    return matches[0], normalized


def make_message(transcript, normalized, command, rms=None) -> dict:
    # Preserve the original JSON keys and command payloads.
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


def _print_result(transcript):
    command, normalized = match_command(transcript)
    print(json.dumps(make_message(transcript, normalized, command), indent=2))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Offline text-matcher experiment")
    parser.add_argument("transcript", nargs="?", help="One complete spoken request")
    args = parser.parse_args()
    if args.transcript is not None:
        _print_result(args.transcript)
    else:
        print("Offline text only. Press Ctrl-D or Ctrl-C to finish.")
        while True:
            try:
                _print_result(input("say> "))
            except (EOFError, KeyboardInterrupt):
                print()
                break

