# control/ — swing-up and fuzzy balancing controllers

| File | What it does |
|---|---|
| `build_fis.m` | Builds the four fuzzy inference systems (angle and cart loop, each as **Mamdani** and **Sugeno**) and computes their scaling gains from the LQR. |
| `hybrid_controller.m` | One controller step: switching logic (swing-up / balance with hysteresis), energy-based swing-up, fuzzy balancing. Used by the MATLAB simulation; the Simulink model contains the same logic in blocks. |

## Structure of the controller

```
             |theta| < 20 deg                       |theta| > 46 deg
 SWING-UP  ------------------->   BALANCE   ------------------->  back to SWING-UP
 (energy)                         (fuzzy)
```

**Swing-up (Åström–Furuta energy control).** The pendulum energy (0 at the upright rest position)
`E = 1/2 (I + m l^2) thd^2 + m g l (cos(th) - 1)` is pumped towards a value slightly above the upright energy:
`a = ke (E - E_ref) thd cos(th)`, saturated at `amax`, plus a centering term `- kx x - kv xd` that keeps the cart on the track.
The cart acceleration `a` is turned into a force by inverse dynamics.

**Balancing (fuzzy, PD-like).** Two fuzzy controllers are summed:

```
F = Ku_th * FLC_theta(theta / th_max, thetadot / thd_max) + Ku_x * FLC_cart(x / x_max, xdot / xd_max)
```

Each FLC has:
- 2 normalized inputs in [-1, 1] (error and its derivative), 7 linguistic values NB NM NS Z PS PM PB
  (triangles inside, trapezoidal shoulders at the ends);
- the classical PD-like rule table `u = sat(e + de)` (49 rules);
- **Mamdani**: AND = min, implication = min, aggregation = max, defuzzification = centroid (COG);
- **Sugeno**: AND = min, zero-order (constant) consequents at the same centres, weighted average.

The two variants use exactly the same rule table and the same membership functions, so the comparison isolates the inference/defuzzification method.

**Scaling from the LQR.** Around the upright position the slope of each FLC is measured numerically
(Mamdani 1.39, Sugeno 1.00) and the normalization ranges / output gains are chosen so that, for small errors,
the fuzzy controller has the same gains as the LQR (`F = 10 x + 11 xdot + 60 theta + 10.3 thetadot`).
Far from the origin the fuzzy controller saturates smoothly instead of growing linearly.

The generated FIS files are saved in `outputs/results/*.fis` (open them with `fuzzyLogicDesigner`).
