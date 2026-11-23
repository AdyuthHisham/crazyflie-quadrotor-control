# Model and LQR design notes

Source: the SSY191 group report (equations 1-25 and appendix A). The Python re-implementation in
`analysis/common.py` follows these equations. It is not the Simscape/Simulink model the team built.

## Plant

Crazyflie 2.1, X configuration, body frame x forward, y left, z up. Constants (`simulink/params.m`, report appendix A):

| Symbol | Value |
|---|---|
| m | 0.027 kg |
| d (centre to rotor) | 0.046 m |
| Jx, Jy, Jz | 1.1463e-5, 1.6993e-5, 2.9944e-5 kg m^2 |
| lift, drag | 1.9796e-9, 2.5383e-11 |

The torque model (report eq. 10, appendix A) is

```
tau_x = d cos(pi/4) (T3 + T4 - T1 - T2)
tau_y = d cos(pi/4) (T2 + T3 - T1 - T4)
tau_z = drag/lift   (T2 + T4 - T1 - T3)
J w_dot = tau - w x (J w)
```

## Linearisation (report eq. 12-18)

States `x = [phi, dphi, theta, dtheta, dpsi]` (roll, roll rate, pitch, pitch rate, yaw rate), inputs
`u = [dT1..dT4]` as deviations from the hover thrust `m g / 4` per rotor. At hover the gyroscopic cross terms
vanish, so roll and pitch are double integrators and yaw rate is a single integrator:

```
A = [0 1 0 0 0; 0 0 0 0 0; 0 0 0 1 0; 0 0 0 0 0; 0 0 0 0 0]
B rows 2, 4, 5 = torque coefficients above divided by Jx, Jy, Jz
```

Yaw angle is not a state, so it is not stabilised. Only yaw rate is regulated.

## Discretisation and LQR (report eq. 19-25)

Zero-order hold at Ts = 0.01 s, then `K = dlqr(Ad, Bd, Q, R)` with the report's weights

```
Q = diag(2, 0.3, 2, 0.3, 0.5),   R = I (4x4)
```

Control law: `u = -K (x - x_ref)`, with `x_ref = [roll_ref, 0, pitch_ref, 0, 0]`.

## Things worth knowing about this design

- **R = I has almost no effect.** The thrust deviations are millinewtons while the angle weights are order 1, so the
  control-effort term is tiny. The closed-loop poles sit near the zeros of the weighted output, not near a trade-off
  with R. The slow pair at z = 0.9745 = exp(-sqrt(2/0.3) * 0.01) is set by the ratio of the angle weight (2) to the
  rate weight (0.3). R only starts to matter above roughly R = 1e3 * I (see `media/poles_qr.png`).
- **Gain mismatch with the report.** See `docs/results.md`, "Mismatches".
