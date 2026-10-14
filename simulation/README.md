# simulation/ — fast closed-loop simulation in plain MATLAB

| File | What it does |
|---|---|
| `simulate_pendulum.m` | Closed-loop simulation without Simulink: the plant is integrated with RK4 at 1 kHz, the controller runs every `Ts = 10 ms` with zero-order hold. Returns trajectories and metrics. |

Options (`opts` struct):

| Field | Meaning |
|---|---|
| `z0` | initial state (default: hanging down, 3° off) |
| `Pplant` | parameters of the *real* plant, to test model mismatch |
| `kick` | `[t0, dthetadot]`: sudden push on the pendulum at time `t0` |
| `noise` | `[std_x, std_theta]`: Gaussian measurement noise; velocities are then obtained by finite differences, as on real hardware |

Metrics returned: time of the catch (switch to balance), whether the pendulum is balanced at the end, number of mode switches, max/RMS angle after the catch, RMS cart position, settling time (|θ| < 1°), control effort ∫F² dt, max force, max cart excursion.

This simulation is used by `main_compare.m` because it is much faster than Simulink and makes the four test scenarios easy to repeat.
