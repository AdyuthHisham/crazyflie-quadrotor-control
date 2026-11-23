"""Closed-loop pole map (Ts = 0.01 s) and R-weight trade-off."""
import matplotlib.pyplot as plt
from common import *
from scenario import *

style()
K, Ad, Bd = dlqr()
fig, (a, b) = plt.subplots(1, 2, figsize=(10, 4.3), gridspec_kw={"width_ratios": [1, 1.25]})

th = np.linspace(0, 2 * np.pi, 400); a.plot(np.cos(th), np.sin(th), color=FG, lw=1)
ol = np.linalg.eigvals(Ad); cl = np.linalg.eigvals(Ad - Bd @ K)
a.plot(ol.real, ol.imag, "x", color=GREY, ms=10, mew=2, label="open loop (5 poles at z = 1)")
a.plot(cl.real, cl.imag, "o", color=BLUE, ms=7, label="closed loop, report Q/R")
a.plot(*np.array([[p.real, p.imag] for p in np.linalg.eigvals(Ad - Bd @ K_REPORT)]).T, "+", color=GREEN,
       ms=10, mew=1.5, label="closed loop, report K (eq. 25)")
a.set_xlim(-0.15, 1.15); a.set_ylim(-1.15, 1.15); a.set_aspect("equal"); a.text(0.97, 0.12, "two poles at z = 0.9745", fontsize=8, color=FG, ha="right"); a.set_xlabel("Re(z)"); a.set_ylabel("Im(z)"); a.legend(fontsize=7.5, loc="lower left")
a.set_title("Discrete poles, Ts = 0.01 s")

rhos = np.logspace(0, 9, 28); rows = []
for rho in rhos:
    Kr = dlqr(R=rho * R_REPORT)[0]
    s = simulate(Kr, Ad, Bd, 12.0, lambda t: (PULSE_DEG, 0.0))
    rows.append((rho, step_metrics(s["t"], s["roll"], PULSE_DEG, 0.0, 12.0)["rise_time_s"],
                 float(np.abs(s["u"]).max() * 1000), float(np.abs(np.linalg.eigvals(Ad - Bd @ Kr)).max())))
rows = np.array(rows)
b.plot(rows[:, 2], rows[:, 1], "-o", color=BLUE, ms=4)
i1 = np.argmin(np.abs(np.log(rows[:, 0])))
b.plot(rows[i1, 2], rows[i1, 1], "o", color=VERM, ms=10, mfc="none", mew=2)
b.annotate("report choice (R = I)", (rows[i1, 2], rows[i1, 1]), (-10, 14), ha="right", textcoords="offset points", color=FG)
b.annotate("R x 1", (rows[0, 2], rows[0, 1]), (8, 0), textcoords="offset points", color=FG, fontsize=8)
b.annotate("R x 1e9", (rows[-1, 2], rows[-1, 1]), (8, -3), textcoords="offset points", color=FG, fontsize=8)
b.set_xscale("log"); b.set_yscale("log")
b.set_xlabel("peak thrust deviation per motor for a 10 deg roll step (mN)"); b.set_ylabel("roll rise time, 10-90 % (s)")
b.set_title("R-weight trade-off (R = rho I, Q fixed)")
from matplotlib.ticker import ScalarFormatter
for ax_ in (b.xaxis, b.yaxis): ax_.set_major_formatter(ScalarFormatter()); ax_.set_minor_formatter(ScalarFormatter())
b.set_yticks([1, 2, 3, 4]); b.set_xticks([0.01, 0.1, 1]); b.minorticks_off()
fig.savefig(MEDIA / "poles_qr.png")
res = dict(closed_loop_poles_report_QR=sorted([complex(p).__repr__() for p in cl]),
           max_pole_magnitude=float(np.abs(cl).max()), min_pole_magnitude=float(np.abs(cl).min()),
           sweep=[dict(rho=float(r[0]), rise_time_s=float(r[1]), peak_thrust_mN=float(r[2]), max_pole_mag=float(r[3])) for r in rows])
save_metrics("poles_qr", res); print({k: v for k, v in res.items() if k != "sweep"}); print(rows[[0, i1, -1]])
