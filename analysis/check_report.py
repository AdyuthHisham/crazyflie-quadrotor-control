"""Cross-check the Python re-implementation against the report (eq. 24-25)."""
import numpy as np
from common import *

np.set_printoptions(precision=4, suppress=True)
K, Ad, Bd = dlqr()
err = np.abs(K - K_REPORT).max()
# empirical: how much must B be scaled to reproduce eq. 25?  (no physical cause identified)
from scipy.optimize import minimize
f = lambda p: np.sum((dlqr(b_scale=tuple(p))[0] - K_REPORT) ** 2)
r = minimize(f, [0.8, 0.8], method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-14))
Kfit = dlqr(b_scale=tuple(r.x))[0]
cl = lambda Kx: np.abs(np.linalg.eigvals(Ad - Bd @ Kx))
res = dict(
    K_from_eq18_row1=K[0].round(4).tolist(), K_report_row1=K_REPORT[0].tolist(),
    max_abs_diff_vs_report=float(err),
    ratio_report_over_derived_row1=(K_REPORT[0] / K[0]).round(3).tolist(),
    B_scale_that_reproduces_report=dict(roll_pitch=float(r.x[0]), yaw=float(r.x[1]),
                                        max_abs_diff=float(np.abs(Kfit - K_REPORT).max())),
    max_closed_loop_pole_magnitude_derived_K=float(cl(K).max()),
    max_closed_loop_pole_magnitude_report_K_on_derived_plant=float(cl(K_REPORT).max()),
    note="B scaled by ~0.79-0.80 reproduces eq. 25; cause not identified (see docs/results.md).",
)
save_metrics("check_report", res)
print(json.dumps(res, indent=2))
