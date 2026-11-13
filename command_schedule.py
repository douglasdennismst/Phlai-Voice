"""
Turns a voice command's params into a timed schedule of body-frame velocity steps.
Pure Python (no ROS, no simulator), shared by every backend.

params is either one move {"linear": [x, y, z], "angular_z": w, "duration": s, "pen": bool}
or a sequence {"steps": [move, ...]}. duration 0 = hold until the next command (and ends the
sequence). Once a timed sequence is over the drone hovers (zero velocity, pen up).
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Step:
    end: float | None          # seconds after the command started; None = hold forever
    linear: tuple              # (x forward, y left, z up) m/s, BODY frame
    angular_z: float           # rad/s, positive = turn left
    pen: bool


HOVER = Step(end=None, linear=(0.0, 0.0, 0.0), angular_z=0.0, pen=False)


def parse(params):
    """params dict -> list[Step]. Raises ValueError/TypeError on malformed input."""
    params = params or {}
    steps = params.get("steps")
    if steps is None:
        steps = [params]                       # a single move
    sequence, t_end = [], 0.0
    for raw in steps:
        x, y, z = (float(v) for v in raw.get("linear", (0.0, 0.0, 0.0)))
        step = dict(linear=(x, y, z), angular_z=float(raw.get("angular_z", 0.0)),
                    pen=bool(raw.get("pen", False)))
        duration = float(raw.get("duration", 0.0))
        if duration <= 0:                      # open-ended hold: last step
            sequence.append(Step(end=None, **step))
            break
        t_end += duration
        sequence.append(Step(end=t_end, **step))
    return sequence


def step_at(sequence, elapsed):
    """The step active `elapsed` seconds after the command started (HOVER once it is over)."""
    for step in sequence:
        if step.end is None or elapsed < step.end:
            return step
    return HOVER


def total_duration(sequence):
    """Seconds until the sequence ends, or None if it holds until the next command."""
    if not sequence:
        return 0.0
    return sequence[-1].end
