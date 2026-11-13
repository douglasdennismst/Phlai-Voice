"""
The voice whitelist: the ONLY place spoken words are tied to commands.

Each VoiceCommand has:
  name    - what gets published as "command"
  phrases - spoken phrases that trigger it (matched as whole words, case-insensitive)
  params  - data handed to the simulator (rotorpy_sim.py) or any other backend.

The simulator understands two shapes of params (all velocities in the drone's BODY frame:
x forward, y left, z up; yaw rad/s, positive = counter-clockwise = turn left):

  single move:  {"linear": [x, y, z], "angular_z": w, "duration": seconds}   (duration 0 = hold)
  sequence:     {"steps": [ {linear, angular_z, duration}, ... ]}             (played in order)

To retarget this at a different system, change/add entries here and write a small
adapter that understands your params (command_schedule.py parses them). Nothing else needs to change.
"""
import math
from dataclasses import dataclass, field

# ---------------- basic moves ----------------
SPEED = 1.0           # m/s for directional moves
STEP_SECONDS = 2.0    # each move lasts this long (~2 m)
YAW_RATE = 0.8        # rad/s
TURN_SECONDS = 2.0    # ~90 degrees
TAKEOFF_SPEED = 1.0
TAKEOFF_SECONDS = 3.0
LAND_SPEED = 0.5
LAND_SECONDS = 6.0

# ---------------- drawing maneuvers ----------------
DRAW_SPEED = 0.8      # m/s while tracing letters/shapes (slower = sharper corners)
CORNER_PAUSE = 0.25   # s of hover after each straight segment so the drone settles
PEN_LIFT = 0.6        # m the drone backs away from the "canvas" between strokes (pen up)
LETTER_H = 1.5        # letter height, m
LETTER_W = 0.9        # letter width, m
LETTER_GAP = 0.4      # space between letters when spelling a word, m


@dataclass(frozen=True)
class VoiceCommand:
    name: str
    phrases: tuple
    params: dict = field(default_factory=dict)


def _step(x=0.0, y=0.0, z=0.0, yaw=0.0, seconds=0.0, pen=None):
    """pen=True leaves a trail in the sim while this step runs; False/None does not."""
    step = {"linear": [round(x, 4), round(y, 4), round(z, 4)],
            "angular_z": round(yaw, 4), "duration": round(seconds, 4)}
    if pen is not None:
        step["pen"] = pen
    return step


def _move(name, phrases, x=0.0, y=0.0, z=0.0, yaw=0.0, seconds=STEP_SECONDS):
    s = _step(x, y, z, yaw, seconds)
    return VoiceCommand(name=name, phrases=tuple(phrases), params=s)


def _seq(name, phrases, steps, clear=False):
    """clear=True wipes the previous trail before this command starts."""
    params = {"steps": steps}
    if clear:
        params["clear_trail"] = True
    return VoiceCommand(name=name, phrases=tuple(phrases), params=params)


# ---- tracing in the vertical "canvas" plane ------------------------------------------
# u = to the viewer's right, v = up, both in metres, measured from where the drone is
# when the command starts. Best viewed from behind the drone. "Right" is -y in body frame.
# The drone's nose direction (x) is the pen axis: backing away from the canvas lifts the pen.

def _segment(du=0.0, dv=0.0, dx=0.0, speed=DRAW_SPEED, pause=CORNER_PAUSE, pen=True):
    length = math.sqrt(du * du + dv * dv + dx * dx)
    if length < 1e-9:
        return []
    t = length / speed
    steps = [_step(x=dx / t, y=-du / t, z=dv / t, seconds=t, pen=pen)]
    if pause > 0:
        steps.append(_step(seconds=pause, pen=pen))
    return steps


def _pen_up(frm, to):
    du, dv = to[0] - frm[0], to[1] - frm[1]
    if abs(du) < 1e-9 and abs(dv) < 1e-9:
        return []
    return (_segment(dx=-PEN_LIFT, pen=False) + _segment(du, dv, pen=False)
            + _segment(dx=PEN_LIFT, pen=False))


def _trace(polylines):
    """Draw each polyline (list of (u, v) points) in order, lifting the pen between them,
    then lift and return to the starting point so commands can be chained."""
    steps, cur = [], (0.0, 0.0)
    for pts in polylines:
        steps += _pen_up(cur, pts[0])
        cur = pts[0]
        for p in pts[1:]:
            steps += _segment(p[0] - cur[0], p[1] - cur[1])
            cur = p
    steps += _pen_up(cur, (0.0, 0.0))
    return steps


def _circle_vertical(radius=1.0, n=24, speed=DRAW_SPEED):
    """Smooth vertical circle starting at its bottom, going right then up (counter-clockwise)."""
    dt = (2 * math.pi * radius / n) / speed
    steps = []
    for k in range(n):
        phi = (k + 0.5) * 2 * math.pi / n
        du, dv = speed * math.cos(phi), speed * math.sin(phi)
        steps.append(_step(y=-du, z=dv, seconds=dt, pen=True))
    return steps


# ---- letters: single continuous polylines, origin = bottom-left corner ---------------
H, W = LETTER_H, LETTER_W
_cross_v = 0.45 * H
_a_half = (W / 2) * (1 - _cross_v / H)
LETTERS = {
    "P": [(0, 0), (0, H), (W, H), (W, H / 2), (0, H / 2)],
    "H": [(0, 0), (0, H), (0, H / 2), (W, H / 2), (W, H), (W, 0)],
    "L": [(0, H), (0, 0), (W, 0)],
    "A": [(0, 0), (W / 2, H), (W, 0), (W / 2 + _a_half, _cross_v), (W / 2 - _a_half, _cross_v)],
    "I": [(0, H), (W, H), (W / 2, H), (W / 2, 0), (0, 0), (W, 0)],
    "M": [(0, 0), (0, H), (W / 2, 0.5 * H), (W, H), (W, 0)],
    "T": [(0, H), (W, H), (W / 2, H), (W / 2, 0)],
    # S as a rounded zig-zag, points given as fractions of (width, height)
    "S": [(W * a, H * b) for a, b in [(1.0, 0.82), (0.8, 1.0), (0.2, 1.0), (0.0, 0.82), (0.0, 0.68),
                                       (0.25, 0.52), (0.75, 0.48), (1.0, 0.32), (1.0, 0.18),
                                       (0.8, 0.0), (0.2, 0.0), (0.0, 0.18)]],
}


def _letter(ch):
    return _trace([LETTERS[ch]])


def _word(text):
    polys = []
    for i, ch in enumerate(text):
        off = i * (LETTER_W + LETTER_GAP)
        polys.append([(u + off, v) for (u, v) in LETTERS[ch]])
    return _trace(polys)


# ---- shapes ---------------------------------------------------------------------------
_S = 1.5
SQUARE = [(0, 0), (0, _S), (_S, _S), (_S, 0), (0, 0)]
TRIANGLE = [(0, 0), (2.0, 0), (1.0, 1.732), (0, 0)]

_verts = [(math.cos(math.radians(90 + 72 * k)), math.sin(math.radians(90 + 72 * k))) for k in range(5)]
# start at the lower-left point so the whole star is drawn above the drone's starting position
STAR = [(_verts[i][0] - _verts[2][0], _verts[i][1] - _verts[2][1]) for i in (2, 4, 1, 3, 0, 2)]

HEART = [(0, 0), (-0.75, 0.8), (-0.75, 1.3), (-0.45, 1.6), (-0.15, 1.5), (0, 1.2),
         (0.15, 1.5), (0.45, 1.6), (0.75, 1.3), (0.75, 0.8), (0, 0)]

# ---- creative flight patterns (horizontal, use yaw) -----------------------------------
_T = 2 * math.pi / 0.5   # time for one full turn at 0.5 rad/s


COMMANDS = [
    # --- basics ---
    _move("takeoff", ["take off", "takeoff", "launch"], z=TAKEOFF_SPEED, seconds=TAKEOFF_SECONDS),
    _move("land", ["land", "landing"], z=-LAND_SPEED, seconds=LAND_SECONDS),
    _move("hover", ["hover", "hold", "stop"], seconds=0.0),
    _move("forward", ["forward", "forwards", "ahead"], x=SPEED),
    _move("backward", ["backward", "backwards", "back", "reverse"], x=-SPEED),
    _move("left", ["left"], y=SPEED),
    _move("right", ["right"], y=-SPEED),
    _move("up", ["up", "ascend", "rise"], z=SPEED),
    _move("down", ["down", "descend"], z=-SPEED),
    _move("turn_left", ["turn left", "rotate left", "yaw left"], yaw=YAW_RATE, seconds=TURN_SECONDS),
    _move("turn_right", ["turn right", "rotate right", "yaw right"], yaw=-YAW_RATE, seconds=TURN_SECONDS),
    # wipes the trail (and holds position)
    _seq("clear_trail", ["clear", "clear trail", "clear the trail", "erase"], [_step(seconds=0.0)], clear=True),

    # --- letters (say "letter p", etc.; spelling variants are best-effort for speech-to-text) ---
    _seq("letter_p", ["letter p", "letter pee", "letter pea"], _letter("P"), clear=True),
    _seq("letter_h", ["letter h", "letter age", "letter aitch"], _letter("H"), clear=True),
    _seq("letter_l", ["letter l", "letter el", "letter elle"], _letter("L"), clear=True),
    _seq("letter_a", ["letter a"], _letter("A"), clear=True),
    _seq("letter_i", ["letter i", "letter eye"], _letter("I"), clear=True),
    _seq("letter_m", ["letter m", "letter em"], _letter("M"), clear=True),
    _seq("letter_s", ["letter s", "letter ess"], _letter("S"), clear=True),
    _seq("letter_t", ["letter t", "letter tee", "letter tea"], _letter("T"), clear=True),
    _seq("spell_mst", ["spell mst", "spell m s t", "spell emm ess tee"], _word("MST"), clear=True),
    _seq("spell_phlai", ["spell phlai", "spell fly", "spell play", "spell phly", "spell flai"], _word("PHLAI"), clear=True),

    # --- shapes (vertical plane, view from behind the drone) ---
    _seq("square", ["square"], _trace([SQUARE]), clear=True),
    _seq("triangle", ["triangle"], _trace([TRIANGLE]), clear=True),
    _seq("star", ["star"], _trace([STAR]), clear=True),
    _seq("heart", ["heart"], _trace([HEART]), clear=True),
    _seq("circle", ["circle"], _circle_vertical(radius=1.0), clear=True),

    # --- creative ---
    # Strafe right while yawing left = orbit a point 2 m ahead while always facing it.
    _seq("orbit", ["orbit"], [_step(y=-1.0, yaw=0.5, seconds=_T, pen=True)], clear=True),
    # Two opposite horizontal loops.
    _seq("figure_eight", ["figure eight", "figure 8", "figure of eight"],
         [_step(x=1.0, yaw=0.5, seconds=_T, pen=True), _step(x=1.0, yaw=-0.5, seconds=_T, pen=True)],
         clear=True),
    # Climbing spiral.
    _seq("spiral", ["spiral"], [_step(x=0.8, z=0.25, yaw=0.6, seconds=12.0, pen=True)], clear=True),
    # Nod yes (bob down/up twice) and shake no (yaw left-right).
    _seq("nod", ["nod", "say yes"],
         [_step(z=-1.0, seconds=0.5), _step(z=1.0, seconds=0.5),
          _step(z=-1.0, seconds=0.5), _step(z=1.0, seconds=0.5)]),
    _seq("shake", ["shake", "say no", "shake head"],
         [_step(yaw=1.0, seconds=0.4), _step(yaw=-1.0, seconds=0.8),
          _step(yaw=1.0, seconds=0.8), _step(yaw=-1.0, seconds=0.4)]),
]
