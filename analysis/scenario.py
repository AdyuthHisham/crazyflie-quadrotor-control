"""Test scenario used for all simulation figures (our choice; the report gives no pulse amplitude)."""
PULSE_DEG = 10.0
# roll pulse 1-4 s, pitch pulse 6-9 s, then rate kicks (rad/s) on roll rate at 11 s and pitch rate at 13 s
def ref(t):
    return (PULSE_DEG if 1 <= t < 4 else 0.0, PULSE_DEG if 6 <= t < 9 else 0.0)
KICKS = [(11.0, 1, 0.5), (13.0, 3, -0.5)]  # (time s, state index, rad/s)
T_END = 16.0
