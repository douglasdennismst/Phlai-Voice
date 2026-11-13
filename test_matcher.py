"""Offline check of the whitelist:  python3 test_matcher.py"""
from matcher import match, match_command

CASES = {
    "Take off.": "takeoff",
    "take-off": "takeoff",
    "Takeoff!": "takeoff",
    "Land": "land",
    "please hover": "hover",
    "Stop.": "hover",
    "Move forward": "forward",
    "go back": "backward",
    "Left.": "left",
    "turn left": "turn_left",
    "Turn right.": "turn_right",
    "go up": "up",
    "descend": "down",
    "forward then land": "forward",
    "that's all right": "right",   # known limitation: bare words match anywhere
    "draw a square": "square",
    "do a triangle": "triangle",
    "make a star": "star",
    "draw a heart": "heart",
    "circle": "circle",
    "draw the letter p": "letter_p",
    "Letter H.": "letter_h",
    "letter eye": "letter_i",
    "spell fly": "spell_phlai",
    "Spell PHLAI": "spell_phlai",
    "letter M": "letter_m",
    "Letter S.": "letter_s",
    "letter tee": "letter_t",
    "Spell MST.": "spell_mst",
    "spell M.S.T.": "spell_mst",
    "clear trail": "clear_trail",
    "Clear.": "clear_trail",
    "erase": "clear_trail",
    "orbit": "orbit",
    "figure eight": "figure_eight",
    "Figure 8.": "figure_eight",
    "spiral": "spiral",
    "say yes": "nod",
    "say no": "shake",
    "banana": None,
    "": None,
}

bad = 0
for text, want in CASES.items():
    cmd, _ = match_command(text)
    got = cmd.name if cmd else None
    flag = "ok " if got == want else "FAIL"
    bad += got != want
    print(f"{flag} {text!r:22} -> {got}")

# Drone targeting (3 drones): text -> (command, target, rejected?)
TARGET_CASES = {
    "take off": ("takeoff", "all", False),
    "Drone two, forward.": ("forward", [2], False),
    "drone 3 land": ("land", [3], False),
    "drones one and three, take off": ("takeoff", [1, 3], False),
    "drone one and drone two go up": ("up", [1, 2], False),
    "drone 1 2 hover": ("hover", [1, 2], False),
    "forward, drone one": ("forward", [1], False),
    "all drones spell fly": ("spell_phlai", "all", False),
    "everyone land": ("land", "all", False),
    "both drones descend": ("down", "all", False),
    "drone to, turn left": ("turn_left", [2], False),       # Whisper's "to" = two
    "drone won hover": ("hover", [1], False),
    "drone one to the left": ("left", [1], False),          # this "to" is not drone 2
    "drone two for ward": (None, [2], False),
    "drone five, land": (None, [5], True),                  # no such drone: rejected
    "drone four forward": (None, [4], True),
    "drone two": (None, [2], False),                        # target but no command
    "drone": (None, "all", False),
}
for text, (want_cmd, want_target, want_err) in TARGET_CASES.items():
    cmd, target, _, error = match(text, drone_count=3)
    got = (cmd.name if cmd else None, target, bool(error))
    ok = got == (want_cmd, want_target, want_err)
    bad += not ok
    print(f"{'ok ' if ok else 'FAIL'} {text!r:36} -> {got[0]} @ {got[1]}{'  (' + error + ')' if error else ''}")
raise SystemExit(1 if bad else 0)
