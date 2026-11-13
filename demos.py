"""
Choreographed demos for the RotorPy sim:  .venv-rotorpy/bin/python3 rotorpy_sim.py --demo showcase

Each demo is a list of items (see rotorpy_sim.parse_demo):
  "[drones:]command"  a command from commands.py for those drones ("2:star", "1+3:land", "takeoff" = all).
                      It starts as soon as ITS drones are free, so commands for different drones overlap.
  "wait"              wait until every drone is free
  "+N"                pause the script N seconds (drones already flying carry on)
  "[drones:]home"     fly back to the start spot, facing forward, 3 m up
  "# text"            caption shown above the 3D view
  team_word(...)      a word split across several drones (see below)

Written for 3 drones (config.DRONE_COUNT) starting side by side: 1 left, 2 middle, 3 right as seen
from behind. Shapes are drawn toward the drone's right, so each drone's drawings stay in its own lane.
"""
import config
from commands import LETTER_W, LETTERS, _trace


def team_word(word, drones_for_letters, gap=0.7, count=None, spacing=None):
    """Spell `word` with several drones at once, each drawing its own letters in place.

    drones_for_letters: one drone number per letter, e.g. "PHLAI" with [1, 1, 2, 2, 3].
    The word is centred on the middle of the line of drones, letters `gap` m apart (wider than
    spell_phlai's 0.4 m so neighbouring drones keep clear of each other). Each drone must be at
    its start spot, facing forward, at the same height ("home" does that).
    Returns one ready-made message per drone, for a demo list.
    """
    count = config.DRONE_COUNT if count is None else count
    spacing = config.DRONE_SPACING if spacing is None else spacing
    pitch = LETTER_W + gap
    first = -(len(word) * pitch - gap) / 2              # word centred on u = 0
    polys = {}
    for i, (ch, n) in enumerate(zip(word, drones_for_letters)):
        lane = -((count - 1) / 2 - (n - 1)) * spacing   # drone n's start, as u (= -y)
        off = first + i * pitch - lane
        polys.setdefault(n, []).append([(u + off, v) for u, v in LETTERS[ch]])
    return [{"target": [n], "command": f"spell_{word.lower()}_part",
             "params": {"steps": _trace(p), "clear_trail": True}} for n, p in polys.items()]


SHOWCASE = [
    "# Act 1 - launch and say hello",
    "1:takeoff", "+0.8", "2:takeoff", "+0.8", "3:takeoff",
    "wait",
    "1:nod", "+0.5", "2:nod", "+0.5", "3:nod",
    "wait",

    "# Act 2 - three drawings at once",
    "1:heart", "2:star", "3:circle",
    "wait",

    "# Act 3 - orbit, figure eight and spiral",
    "2:up",                               # drone 2 climbs above the others first
    "1:orbit",
    "+1.5",
    "3:spiral",
    "2:up",
    "2:figure_eight",
    "wait",

    "# Act 4 - drone 2 spells MST up high, the others draw below",
    "2:spell_mst",
    "3:down", "1:turn_left", "1:turn_right",   # drone 3 comes down off the spiral (6 m -> 4 m)
    "1:triangle", "3:star",
    "wait",

    "# Act 5 - line up and spell PHLAI together",
    "all:home",                           # back to the start line, facing forward, 3 m up
    "wait",                               # so all three start writing together
    *team_word("PHLAI", [1, 1, 2, 2, 3]),
    "wait",

    "# Act 6 - take a bow and land in turn",
    "all:nod",
    "wait",
    "3:land", "+1.2", "2:land", "+1.2", "1:land",
    "wait",
    "# Thanks for watching!",
]

# A short one for checking that everything works.
QUICK = [
    "# Quick check",
    "takeoff", "wait",
    "1:heart", "2:star", "3:circle", "wait",
    "all:land", "wait",
]

# Just the word: take off, spell PHLAI together, land.
PHLAI = [
    "# P H L A I",
    "takeoff", "wait",
    *team_word("PHLAI", [1, 1, 2, 2, 3]),
    "wait",
    "all:land", "wait",
]

DEMOS = {"showcase": SHOWCASE, "quick": QUICK, "phlai": PHLAI}
