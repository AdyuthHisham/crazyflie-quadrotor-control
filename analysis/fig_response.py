"""Roll/pitch response to a reference pulse and a rate disturbance; LQR variants vs. no feedback."""
import matplotlib.pyplot as plt
from common import *
from scenario import *

style()
K, Ad, Bd = dlqr()
runs = [
    ("LQR, report Q/R", dlqr()[0], BLUE, "-"),
    ("LQR, R x 1e6 (softer)", dlqr(R=1e6 * R_REPORT)[0], ORANGE, "-"),
    ("report K (eq. 25) on derived plant", K_REPORT, GREEN, "--"),
    ("no feedback", None, GREY, ":"),
]
sims = {n: simulate(Kx, Ad, Bd, T_END, ref, KICKS) for n, Kx, *_ in runs}
base = sims["LQR, report Q/R"]

fig, ax = plt.subplots(3, 1, figsize=(8, 7.2), sharex=True, gridspec_kw={"height_ratios": [1, 1, 0.8]})
for a, key, rkey, title in [(ax[0], "roll", "r_roll", "Roll"), (ax[1], "pitch", "r_pitch", "Pitch")]:
    a.plot(base["t"], base[rkey], color="k", alpha=0.45, lw=1.2, drawstyle="steps-post", label="reference")
    for n, _, c, ls in runs:
        a.plot(sims[n]["t"], sims[n][key], color=c, ls=ls, label=n)
    a.set_ylabel(f"{title} (deg)"); a.set_ylim(-9, 14)
    a.axvline(11.0 if key == "roll" else 13.0, color=FG, alpha=0.4, lw=1, ls="-.")
    a.text(11.1 if key == "roll" else 13.1, 12.3, "rate kick (0.5 rad/s)", color=FG, fontsize=8)
ax[0].text(0.5, -7.5, "'no feedback' leaves the plot after the kick: roll rate integrates freely", fontsize=8, color=FG)
ax[0].legend(ncol=3, fontsize=8, loc="lower center", bbox_to_anchor=(0.5, 1.12))
for i, c in enumerate([BLUE, ORANGE, VERM, GREEN]):
    ax[2].plot(base["t"], base["u"][:, i] * 1000, color=c, lw=1.2, label=f"motor {i + 1}")
ax[2].set_ylabel("thrust deviation (mN)"); ax[2].set_xlabel("time (s)"); ax[2].legend(ncol=4, fontsize=8)
fig.suptitle("Python re-implementation of the linearised model (simulation)", y=1.0, color=FG, fontweight="bold")
fig.savefig(MEDIA / "response.png")

m_roll = step_metrics(base["t"], base["roll"], PULSE_DEG, 1.0, 4.0)
m_pitch = step_metrics(base["t"], base["pitch"], PULSE_DEG, 6.0, 9.0)
res = dict(pulse_deg=PULSE_DEG, kicks_rad_s=KICKS, roll_pulse=m_roll, pitch_pulse=m_pitch,
           peak_abs_thrust_deviation_mN=float(np.abs(base["u"]).max() * 1000),
           max_angle_after_roll_kick_deg=float(np.abs(base["roll"][(base["t"] > 11) & (base["t"] < 13)]).max()),
           max_angle_after_roll_kick_no_feedback_deg=float(np.abs(sims["no feedback"]["roll"][(base["t"] > 11) & (base["t"] < 13)]).max()),
           roll_after_kick_end_no_feedback_deg=float(sims["no feedback"]["roll"][-1]),
           roll_after_kick_end_lqr_deg=float(base["roll"][-1]))
save_metrics("response", res); print(json.dumps(res, indent=2))
