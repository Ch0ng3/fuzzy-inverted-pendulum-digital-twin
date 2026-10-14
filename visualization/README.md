# visualization/ — animation and figures

| File | What it does |
|---|---|
| `animate_pendulum.m` | 2D animated dashboard: cart and pendulum on the track (with the force arrow, the trail of the rod tip and the current mode), plus live plots of θ, x and F. Can record an MP4. |
| `make_gif.m` | Converts an MP4 into a small looping GIF (used for the README on GitHub). |
| `export_model_figures.m` | Saves screenshots of the Simulink model (top level, controller, fuzzy subsystem, plant) in `outputs/figures`. |

The 3D view of the digital twin is provided by Simscape Multibody's **Mechanics Explorer**, which opens automatically when the Simulink model runs.
