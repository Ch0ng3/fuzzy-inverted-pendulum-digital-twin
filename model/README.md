# model/ — physical model of the cart-pendulum

| File | What it does |
|---|---|
| `pendulum_params.m` | All parameters in one struct `P`: masses, lengths, friction, force limit, sampling time, swing-up gains, switching thresholds, initial state. **Change the system here.** |
| `pendulum_dynamics.m` | Nonlinear equations of motion `dz = f(z, F)`, state `z = [x; xdot; theta; thetadot]`. |
| `design_lqr.m` | Numerical linearization around the upright position and an LQR gain `K`. The LQR is **not** used to control the system: it is only the reference used to scale the fuzzy controllers (see `control/`). |

## Conventions

- `x` cart position [m], positive to the right.
- `theta` pendulum angle [rad]: `0` = upright, `pi` = hanging down; positive when the pendulum tip moves towards `+x`.
- `F` horizontal force on the cart [N], saturated at `P.Fmax`.

## Equations

With a uniform rod (centre of mass at distance `l` from the pivot, inertia `I` about the centre of mass):

```
(M + m) xdd + m l cos(th) thdd - m l thd^2 sin(th) + b xd = F
m l cos(th) xdd + (I + m l^2) thdd - m g l sin(th) + c thd = 0
```

Default values: `M = 0.5 kg`, `m = 0.2 kg`, `l = 0.3 m` (rod 0.6 m), `b = 0.1 N s/m`, `c = 0.002 N m s/rad`, `Fmax = 15 N`, `Ts = 10 ms`.

Linearized around the upright position the system has one unstable pole at about +5.5 rad/s.
