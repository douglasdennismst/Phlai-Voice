"""Offline check of the whitelist:  python3 test_matcher.py"""
from matcher import match_command

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
raise SystemExit(1 if bad else 0)
