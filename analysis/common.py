"""Shared model, LQR design and plotting style.

Python re-implementation of the linearised quadrotor model in the SSY191 report
(eq. 12-22, 24). Constants are the public Crazyflie 2.1 values from simulink/params.m.
"""
import json
from pathlib import Path

import control
import matplotlib as mpl
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MEDIA = ROOT / "media"
METRICS = ROOT / "docs" / "metrics"
MEDIA.mkdir(exist_ok=True)
METRICS.mkdir(parents=True, exist_ok=True)

# --- parameters (simulink/params.m, report appendix A) ---
m, g, d = 0.027, 9.81, 0.046
lift, drag = 1.9796e-9, 2.5383e-11
Jx, Jy, Jz = 1.1463e-5, 1.6993e-5, 2.9944e-5
TS = 0.01  # report, section V

# report eq. 24
Q_REPORT = np.diag([2, 0.3, 2, 0.3, 0.5])
R_REPORT = np.eye(4)
# report eq. 25
K_REPORT = np.array([
    [-0.0284, -0.0112, -0.0420, -0.0166, -0.0701],
    [-0.0284, -0.0112, 0.0420, 0.0166, 0.0701],
    [0.0284, 0.0112, 0.0420, 0.0166, -0.0701],
    [0.0284, 0.0112, -0.0420, -0.0166, 0.0701],
])


def linear_model(b_scale=(1.0, 1.0)):
    """x = [phi, dphi, theta, dtheta, dpsi], u = [dT1..dT4] (N, around hover). Report eq. 18."""
    a = d * np.cos(np.pi / 4)
    kb = drag / lift
    A = np.zeros((5, 5)); A[0, 1] = 1; A[2, 3] = 1
    B = np.zeros((5, 4))
    B[1] = np.array([-a, -a, a, a]) / Jx
    B[3] = np.array([-a, a, a, -a]) / Jy
    B[4] = np.array([-kb, kb, -kb, kb]) / Jz
    B[:4] *= b_scale[0]; B[4] *= b_scale[1]
    return A, B


def discretise(A, B, ts=TS):
    sd = control.c2d(control.ss(A, B, np.eye(5), np.zeros((5, 4))), ts)  # ZOH, eq. 21-22
    return sd.A, sd.B


def dlqr(Q=Q_REPORT, R=R_REPORT, b_scale=(1.0, 1.0)):
    Ad, Bd = discretise(*linear_model(b_scale))
    K, _, _ = control.dlqr(Ad, Bd, Q, R)
    return K, Ad, Bd


def simulate(K, Ad, Bd, t_end, ref, kick=(), x0=None):
    """Closed loop x+ = Ad x + Bd u, u = -K (x - x_ref).
    ref(t) -> (roll_deg, pitch_deg); kick = [(t, state_index, value)] instantaneous state disturbances.
    Returns dict of arrays (angles in deg, thrust deviation in N)."""
    n = int(round(t_end / TS)) + 1
    x = np.zeros(5) if x0 is None else np.array(x0, float)
    kicks = {int(round(t / TS)): (i, v) for t, i, v in kick}
    out = {k: np.zeros(n) for k in ("t", "roll", "pitch", "yawrate", "r_roll", "r_pitch")}
    out["u"] = np.zeros((n, 4)); out["x"] = np.zeros((n, 5))
    for k in range(n):
        if k in kicks:
            i, v = kicks[k]; x[i] += v
        t = k * TS
        rr, rp = ref(t)
        xr = np.array([np.radians(rr), 0, np.radians(rp), 0, 0])
        u = -K @ (x - xr) if K is not None else np.zeros(4)
        out["t"][k] = t; out["x"][k] = x; out["u"][k] = u
        out["roll"][k], out["pitch"][k] = np.degrees(x[0]), np.degrees(x[2])
        out["yawrate"][k] = np.degrees(x[4]); out["r_roll"][k], out["r_pitch"][k] = rr, rp
        x = Ad @ x + Bd @ u
    return out


def step_metrics(t, y, y_ref, t0, t1):
    """10-90 % rise time, 2 % settling time, overshoot (%) of a step starting at t0 to level y_ref."""
    s = (t >= t0) & (t < t1)
    tt, yy = t[s] - t0, y[s]
    hit = lambda f: tt[np.argmax(yy >= f * y_ref)] if (yy >= f * y_ref).any() else np.nan
    rise = hit(0.9) - hit(0.1)
    out_band = np.abs(yy - y_ref) > 0.02 * y_ref
    settle = tt[min(np.where(out_band)[0][-1] + 1, len(tt) - 1)] if out_band.any() else 0.0
    return dict(rise_time_s=float(rise), settling_time_s=float(settle),
                overshoot_pct=float(max(0.0, (yy.max() - y_ref) / y_ref * 100)))


def save_metrics(name, d_):
    (METRICS / f"{name}.json").write_text(json.dumps(d_, indent=2, sort_keys=True) + "\n")


# --- plotting style: Okabe-Ito colours, mid-grey text, transparent background (light + dark mode) ---
BLUE, ORANGE, VERM, GREEN, GREY, SKY, PURPLE = "#0072B2", "#E69F00", "#D55E00", "#009E73", "#8c8c8c", "#56B4E9", "#CC79A7"
FG = "#7d8590"


def style():
    mpl.rcParams.update({
        "figure.facecolor": "none", "axes.facecolor": "none", "savefig.facecolor": "none",
        "text.color": FG, "axes.labelcolor": FG, "axes.edgecolor": FG, "xtick.color": FG, "ytick.color": FG,
        "grid.color": FG, "grid.alpha": 0.25, "axes.grid": True, "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
        "legend.frameon": False, "lines.linewidth": 1.8, "savefig.dpi": 150, "savefig.bbox": "tight",
    })


def synth_sensors(ref, t_end, h=0.005, bias=0.1, acc_noise=0.1, seed=0):
    """Synthetic IMU on the Python closed-loop trajectory, using the course simulator's sensor model
    (main.c: 0.1 deg/s gyro bias, +-0.1 m/s^2 uniform accel noise). Returns t, true roll (deg), accel roll (deg), gyro roll rate (deg/s)."""
    rng = np.random.default_rng(seed)
    K, Ad, Bd = dlqr()
    s = simulate(K, Ad, Bd, t_end, ref)
    t = np.arange(0, t_end, h)
    roll, pitch = np.interp(t, s["t"], s["roll"]), np.interp(t, s["t"], s["pitch"])
    phi, th = np.radians(roll), np.radians(pitch)
    acc = g * np.stack([-np.sin(th), np.sin(phi) * np.cos(th), np.cos(phi) * np.cos(th)], 1) + rng.uniform(-acc_noise, acc_noise, (len(t), 3))
    return t, roll, np.degrees(np.arctan2(acc[:, 1], acc[:, 2])), np.gradient(roll, h) + bias


def fuse(acc_roll, gyro, gamma, h=0.005):
    """Complementary filter, same recursion as firmware/filter.c: e = (1-gamma) acc + gamma (e_prev + h gyro)."""
    e = np.zeros(len(acc_roll))
    for k in range(1, len(e)):
        e[k] = (1 - gamma) * acc_roll[k] + gamma * (e[k - 1] + h * gyro[k])
    return e
