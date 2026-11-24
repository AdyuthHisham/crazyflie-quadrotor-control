"""Complementary filter: (a) real run of our C filter in the FreeRTOS desktop simulation,
(b) the same filter equation in Python on synthetic sensors, gamma = 0.9 (report) vs 0.9992 (as in filter.c)."""
import matplotlib.pyplot as plt
import numpy as np
from common import *
from scenario import *

style()
rng = np.random.default_rng(0)

# (a) C simulation log (analysis/data/c_sim_simulation.csv, produced by firmware/filter.c + controller.c + course dynamic model)
cs = np.genfromtxt(ROOT / "analysis/data/c_sim_simulation.csv", delimiter=",", names=True)
sel = cs["time"] > 0.5  # first logged row is the initial zero state
err_r = cs["estimated_roll"][sel] - cs["actual_roll"][sel]
err_p = cs["estimated_pitch"][sel] - cs["actual_pitch"][sel]

# (b) synthetic sensors on the Python closed-loop roll trajectory
H, BIAS, ACC_NOISE = 0.005, 0.1, 0.1
t, roll, acc_roll, gyro = synth_sensors(lambda t: (10.0 if 2 <= t < 8 else 0.0, 0.0), 14.0, H, BIAS, ACC_NOISE)
gyro_int = np.cumsum(gyro) * H
f09, f0999 = fuse(acc_roll, gyro, 0.9, H), fuse(acc_roll, gyro, 6 / (H + 6), H)
rms = lambda x: float(np.sqrt(np.mean((x - roll) ** 2)))

fig, (a, b) = plt.subplots(2, 1, figsize=(8, 7), gridspec_kw={"height_ratios": [1, 1.2]})
a.plot(cs["time"][sel], cs["reference_roll"][sel], color="k", alpha=0.4, lw=1.2, label="reference")
a.plot(cs["time"][sel], cs["actual_roll"][sel], color=BLUE, label="actual roll (course dynamic model)")
a.plot(cs["time"][sel], cs["estimated_roll"][sel], color=VERM, lw=1.2, ls="--", label="estimate from firmware/filter.c")
a.set_xlabel("time (s)"); a.set_ylabel("roll (deg)"); a.set_ylim(-48, 48); a.legend(fontsize=8, ncol=3, loc="upper center")
a.set_title("(a) Our C filter in the FreeRTOS desktop simulation (logged run)")

b.plot(t, acc_roll, color=GREY, lw=0.5, alpha=0.7, label="accelerometer only (noisy)")
b.plot(t, gyro_int, color=ORANGE, label="gyro integrated (0.1 deg/s bias drifts)")
b.plot(t, roll, color="k", alpha=0.5, lw=1.2, label="true roll")
b.plot(t, f09, color=BLUE, label="fused, gamma = 0.9")
b.plot(t, f0999, color=VERM, ls="--", label="fused, gamma = 0.9992 (alpha = 6 s)")
b.set_ylim(-2, 19); b.set_xlabel("time (s)"); b.set_ylabel("roll (deg)"); b.legend(fontsize=8, loc="upper right", ncol=2)
b.set_title("(b) Filter equation on synthetic sensors (Python)")
fig.tight_layout(); fig.savefig(MEDIA / "filter.png")

res = dict(c_sim_rms_error_deg=dict(roll=float(np.sqrt(np.mean(err_r ** 2))), pitch=float(np.sqrt(np.mean(err_p ** 2)))),
           c_sim_max_abs_error_deg=dict(roll=float(np.abs(err_r).max()), pitch=float(np.abs(err_p).max())),
           c_sim_duration_s=float(cs["time"][-1]), c_sim_rows=int(sel.sum()),
           synthetic_rms_error_deg=dict(accel_only=rms(acc_roll), gyro_only=rms(gyro_int), fused_gamma_0p9=rms(f09), fused_gamma_0p9992=rms(f0999)),
           gamma_in_filter_c=6 / (H + 6), filter_period_s=H, gyro_bias_deg_s=BIAS, accel_noise_m_s2=ACC_NOISE)
save_metrics("filter", res); print(json.dumps(res, indent=2))
