"""Plot every drawing maneuver in commands.py to maneuvers.png (needs: pip install matplotlib).
Shows what the drone would trace (pen-down only) as seen from behind it. Handy after adding letters."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from commands import COMMANDS, PEN_LIFT

DRAW = [c for c in COMMANDS if c.name.startswith(("letter_", "spell_")) or
        c.name in ("square", "triangle", "star", "heart", "circle")]


def segments(steps):
    x = y = z = 0.0
    segs, cur = [], []
    for s in steps:
        lx, ly, lz = s["linear"]
        n = max(1, int(s["duration"] / 0.02))
        dt = s["duration"] / n
        for _ in range(n):
            x += lx * dt; y += ly * dt; z += lz * dt
            if x > -PEN_LIFT / 2:
                cur.append((-y, z))
            elif cur:
                segs.append(cur); cur = []
    if cur:
        segs.append(cur)
    return segs


cols = 4
rows = (len(DRAW) + cols - 1) // cols
fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3.3 * rows))
for ax in axes.flat:
    ax.axis("off")
for ax, cmd in zip(axes.flat, DRAW):
    ax.axis("on")
    for seg in segments(cmd.params["steps"]):
        us, vs = zip(*seg)
        ax.plot(us, vs, lw=2)
    ax.set_title(cmd.name); ax.set_aspect("equal"); ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("maneuvers.png", dpi=70)
print("wrote maneuvers.png")
