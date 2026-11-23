"""Hero GIF: attitude of the linearised model following the roll/pitch reference (simulation, Python re-implementation)."""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from common import *
from scenario import *

style()
BG, TXT = "#f6f8fa", "#24292f"  # opaque card so the GIF reads the same in light and dark mode
mpl.rcParams.update({"figure.facecolor": BG, "savefig.facecolor": BG, "axes.facecolor": BG, "text.color": TXT,
                     "axes.labelcolor": TXT, "xtick.color": TXT, "ytick.color": TXT, "axes.edgecolor": TXT})

K, Ad, Bd = dlqr()
s = simulate(K, Ad, Bd, T_END, ref, KICKS)
psi = np.cumsum(s["x"][:, 4]) * TS  # yaw angle integrated from the yaw-rate state (rad)

# arm geometry from report eq. 10 (x forward, y left, z up); motor i = 1..4
L = 1.0
MOT = np.array([[1, -1], [-1, -1], [-1, 1], [1, 1]]) * L / np.sqrt(2)
MC = [BLUE, ORANGE, VERM, GREEN]


def rot(phi, th, ps):  # R = Rz Ry Rx (report eq. 6)
    cx, sx, cy, sy, cz, sz = np.cos(phi), np.sin(phi), np.cos(th), np.sin(th), np.cos(ps), np.sin(ps)
    return np.array([[cz * cy, cz * sy * sx - sz * cx, sz * sx + cz * sy * cx],
                     [sz * cy, cz * cx + sz * sy * sx, sz * sy * cx - cz * sx], [-sy, cy * sx, cy * cx]])


FPS, STEP = 15, int(round(1 / (15 * TS)))
idx = np.arange(0, len(s["t"]), STEP)
fig = plt.figure(figsize=(9, 4.2), dpi=70)
gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.15], hspace=0.35, wspace=0.18, left=0.02, right=0.98, top=0.86, bottom=0.12)
a3 = fig.add_subplot(gs[:, 0], projection="3d"); ar = fig.add_subplot(gs[0, 1]); ap = fig.add_subplot(gs[1, 1], sharex=ar)
fig.suptitle("Crazyflie attitude, discrete LQR, linearised model (simulation, Python re-implementation)", fontsize=10, fontweight="bold", y=0.97)
for a, key, rk, name in [(ar, "roll", "r_roll", "roll (deg)"), (ap, "pitch", "r_pitch", "pitch (deg)")]:
    a.plot(s["t"], s[rk], color=TXT, alpha=0.45, lw=1.2, drawstyle="steps-post", label="reference")
    a.plot(s["t"], s[key], color=BLUE if key == "roll" else VERM, lw=1.6, label="LQR")
    a.set_ylabel(name); a.set_ylim(-9, 14); a.grid(alpha=0.25)
    a.axvline(11.0 if key == "roll" else 13.0, color=TXT, alpha=0.4, ls="-.", lw=1)
ap.set_xlabel("time (s)"); ar.legend(loc="upper right", fontsize=8, frameon=False); ar.tick_params(labelbottom=False)
ar.text(11.1, 11.5, "rate kick", fontsize=7); ap.text(13.1, 11.5, "rate kick", fontsize=7)
cur = [a.axvline(0, color=TXT, lw=1) for a in (ar, ap)]
dots = [ar.plot([], [], "o", color=BLUE)[0], ap.plot([], [], "o", color=VERM)[0]]


def body(R, scale=1.0):
    return [R @ np.array([x, y, 0]) * scale for x, y in MOT]


def draw(f):
    k = idx[f]
    a3.clear(); a3.set_axis_off()
    a3.set_xlim(-0.8, 0.8); a3.set_ylim(-0.8, 0.8); a3.set_zlim(-0.5, 0.5); a3.set_box_aspect((1, 1, 0.7)); a3.view_init(22, -62)
    for name, (p, q, ps, col, al, lw) in {"ref": (np.radians(s["r_roll"][k]), np.radians(s["r_pitch"][k]), 0, TXT, 0.35, 1.2),
                                           "sim": (np.radians(s["roll"][k]), np.radians(s["pitch"][k]), psi[k], BLUE, 1, 2.6)}.items():
        R = rot(p, q, ps); pts = body(R)
        for i in (0, 2): a3.plot(*zip(pts[i], pts[i + 1] if False else pts[(i + 2) % 4]), color=col, alpha=al, lw=lw)
        for i in (1, 3): a3.plot(*zip(pts[i], pts[(i + 2) % 4]), color=col, alpha=al, lw=lw)
        if name == "sim":
            for i in range(4):
                a3.scatter(*pts[i], s=70, color=MC[i], depthshade=False, zorder=5)
                a3.text(*(pts[i] + np.array([0, 0, 0.12])), str(i + 1), fontsize=7, color=TXT, ha="center")
            a3.plot(*zip(np.zeros(3), R @ np.array([0, 0, 0.35])), color=GREEN, lw=1.5)  # body z axis
    a3.plot([-0.9, 0.9], [0, 0], [-0.6, -0.6], color=TXT, alpha=0.2, lw=0.8); a3.plot([0, 0], [-0.9, 0.9], [-0.6, -0.6], color=TXT, alpha=0.2, lw=0.8)
    a3.text2D(0.5, 0.0, f"t = {s['t'][k]:4.1f} s   roll {s['roll'][k]:5.1f} deg   pitch {s['pitch'][k]:5.1f} deg", transform=a3.transAxes, ha="center", fontsize=8)
    for c in cur: c.set_xdata([s["t"][k]] * 2)
    dots[0].set_data([s["t"][k]], [s["roll"][k]]); dots[1].set_data([s["t"][k]], [s["pitch"][k]])


anim = FuncAnimation(fig, draw, frames=len(idx), interval=1000 / FPS)
anim.save(MEDIA / "hero.gif", writer=PillowWriter(fps=FPS), dpi=70)
print("frames", len(idx), "size MB", round((MEDIA / "hero.gif").stat().st_size / 1e6, 2))
