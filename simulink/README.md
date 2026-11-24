# Simulink / Simscape

The team's models are **not included**:

- The `.slx`, `plant.ssc`, `sensors.sscp` and `.tlc` files in the course repository are the course template. The copy of
  `plant.ssc` I checked still contains the template's placeholder equations, and the `.slx` files carry template dates,
  so the model the team actually built is not in the repositories I have. Redistributing the template is not clearly allowed.
- There is no `closed_loop_script.m` (the Breach script), so I make no claim about Breach/STL testing.

What is here:

- `params.m`: the public Crazyflie 2.1 constants (mass, arm length, lift and drag constants), as in the course file.

What the model contained, from the report (appendix A and section IV): a Simscape component with inputs
`ctrl = [T1..T4]` (N) and outputs position, velocity, acceleration, angle and angular velocity; rotation `R = Rz Ry Rx`;
`m a = R [0 0 sum(T)] - m g - F v`; `J w_dot = tau - w x (J w)`; plus a Simulink complementary filter and the discrete LQR
block. Its linearisation is re-implemented in `analysis/common.py`.

To use the course models yourself, open the template repository from the course, run `params.m`, then open
`crazyflie.slx` in MATLAB with Simulink, Simscape and Embedded Coder.

The final team `.slx` and `.ssc` files are no longer available to me, so they cannot be added.
