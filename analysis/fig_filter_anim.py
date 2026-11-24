"""Animation of the complementary filter on synthetic sensors (Python): why each sensor alone fails and what the fusion does."""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from common import *

style()
BG, TXT = "#f6f8fa", "#24292f"
mpl.rcParams.update({"figure.facecolor": BG, "savefig.facecolor": BG, "axes.facecolor": BG, "text.color": TXT,
                     "axes.labelcolor": TXT, "xtick.color": TXT, "ytick.color": TXT, "axes.edgecolor": TXT})
H, GAMMA, T_END = 0.005, 0.9, 40.0
ref = lambda t: (10.0 if 2 <= t < 8 else (-10.0 if 20 <= t < 26 else 0.0), 0.0)
t, roll, acc, gyro = synth_sensors(ref, T_END, H)
gyro_int = np.cumsum(gyro) * H
fused = fuse(acc, gyro, GAMMA, H)
err = {k: np.abs(v - roll) for k, v in [("acc", acc), ("gyro", gyro_int), ("fused", fused)]}
sm = lambda x, n=200: np.convolve(x, np.ones(n) / n, mode="same")  # 1 s moving average of |error|

FPS, STEP = 20, int(round(0.2 / H))  # one frame per 0.2 s of sensor time
idx = np.arange(STEP, len(t), STEP)
fig, (a, b) = plt.subplots(2, 1, figsize=(8, 4.8), dpi=70, gridspec_kw={"height_ratios": [1.6, 1]}, sharex=True)
fig.subplots_adjust(left=0.09, right=0.98, top=0.82, bottom=0.11, hspace=0.12)
fig.suptitle("Complementary filter on synthetic IMU data (Python)", fontsize=10, fontweight="bold", y=0.98)
fig.text(0.5, 0.905, r"$\theta_k = (1-\gamma)\,\theta_{acc,k} + \gamma\,(\theta_{k-1} + h\,\omega_{gyro,k})$,   $\gamma$ = 0.9,  h = 5 ms",
         ha="center", fontsize=10)
lines = {}
a.plot(t, roll, color=TXT, alpha=0.35, lw=1.2, label="true roll")
for key, y, c, lw, lab in [("acc", acc, GREY, 0.7, "accelerometer: noisy"), ("gyro", gyro_int, ORANGE, 1.6, "gyro integrated: drifts"),
                           ("fused", fused, BLUE, 1.8, "fused")]:
    lines[key] = (a.plot([], [], color=c, lw=lw, label=lab)[0], y)
a.set_ylim(-14, 16); a.set_xlim(0, T_END); a.set_ylabel("roll (deg)"); a.legend(loc="lower left", fontsize=7, ncol=2, frameon=False); a.grid(alpha=0.25)
elines = {}
for key, c, lab in [("acc", GREY, "accelerometer"), ("gyro", ORANGE, "gyro"), ("fused", BLUE, "fused")]:
    elines[key] = (b.plot([], [], color=c, lw=1.6, label=lab)[0], sm(err[key]))
b.set_ylim(0, 4.6); b.set_ylabel("|error| (deg, 1 s avg)"); b.set_xlabel("time (s)"); b.grid(alpha=0.25); b.legend(loc="upper left", fontsize=7, ncol=3, frameon=False)
cur = [ax.axvline(0, color=TXT, lw=1) for ax in (a, b)]


def draw(f):
    k = idx[f]
    for ln, y in list(lines.values()) + list(elines.values()):
        ln.set_data(t[:k], y[:k])
    for c in cur: c.set_xdata([t[k]] * 2)


FuncAnimation(fig, draw, frames=len(idx), interval=1000 / FPS).save(MEDIA / "filter.gif", writer=PillowWriter(fps=FPS), dpi=70)
print("frames", len(idx), "MB", round((MEDIA / "filter.gif").stat().st_size / 1e6, 2),
      {k: round(float(v.mean()), 3) for k, v in err.items()}, "final gyro err", round(float(err["gyro"][-1]), 2))
