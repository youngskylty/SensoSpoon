# The Liquid / Soup Slosh Model (`src/fluid/`)

This is a separate, self-contained add-on model answering a specific question: if there's soup in the bowl, does the actuated (or unactuated) motion cause it to slosh around or spill over the rim? It is explicitly **not** a full fluid-dynamics (CFD) simulation — it's a "reduced-order" or "two-mode" analytical approximation, and the docs (`docs/LIQUID_MODEL.md`) are careful to describe exactly what it does and doesn't capture.

## One-way coupling — the key structural fact

The mechanical/electrical closed-loop simulation (from `03_CONTROL_SYSTEM.md`/`02_SIMULINK_MODELS.md`) is run **first**, completely independently, using the liquid's mass as a fixed payload (`H.food_mass_kg`) in the rigid-body dynamics. Only *afterward* is the liquid-surface model run, driven by the already-computed spoon motion. The liquid's own sloshing forces are never fed back into the mechanical simulation, and mass lost to spillage doesn't reduce the payload mass used by the mechanical model either. This is what "one-way coupled" means, and it's a deliberate approximation — reasonable when the liquid mass is small relative to everything else, but explicitly not physically complete.

## Files

- `rcflex_liquid_parameters.m` — a handful of tunable constants: fluid density, damping ratio, spatial grid size, integration step, output rate, weir (overflow) coefficient.
- `rcflex_liquid_geometry.m` — samples the bowl's actual interior shape (from the same analytic profile function used by the 3D mesh generator) onto a grid, for numerical integration of volume and for tracking the rim outline.
- `rcflex_liquid_level.m` — given a target volume and a current surface tilt (slope), solves for the flat surface height that holds exactly that volume (bisection search).
- `rcflex_liquid_motion.m` — extracts the bowl's position, orientation, velocity, angular velocity, and accelerations at a given time step, for either the controlled or uncontrolled ("locked") case.
- `simulate_rcflex_liquid.m` — the actual slosh simulation: computes free-surface dynamics and rim overflow over time.
- `run_rcflex_soup_demo.m`, `run_rcflex_strong_soup.m`, `screen_rcflex_strong_soup.m` — driver scripts that set up a scenario, run the mechanical model, run the fluid model, and produce comparison videos/tables.

## The physical approximation, plain-language

Think of the liquid surface as a single tilted flat plane (not a fully detailed 3D free surface) whose two tilt angles (`slope`, one per horizontal axis) behave like a damped spring-mass system driven by the effective gravity direction in the bowl's own reference frame:

```matlab
accel = wn^2 * (target_tilt - slope) - 2*damping_ratio*wn*rate + angular_acceleration_term
```

- `wn` (natural frequency of the sloshing mode) is computed from the classic shallow-water/wave dispersion relation `omega^2 = g*k*tanh(k*h)` (`k = pi/wet_length`), which is a well-known formula from slosh literature (the docs cite a NASA reference on equivalent mechanical models of liquid slosh) — this is the standard way to estimate the natural frequency of a sloshing liquid mode from the container's wetted dimensions and depth.
- `target_tilt` is the direction "down" points in the bowl's own coordinate frame right now (accounting for the bowl's own rotation, translation, and centripetal/angular acceleration — i.e., "effective gravity" as felt inside a non-inertial, accelerating, rotating container, exactly like what you'd feel on a turning, accelerating vehicle).
- The extra `angular_acceleration_term` (`[alpha_y; -alpha_x]`) accounts for the fact that if the bowl itself is angularly accelerating, the liquid surface has inertia relative to the container and lags/leads accordingly.

Given the current tilt, `rcflex_liquid_level.m` finds (by bisection) the flat-plane height that makes the currently-remaining liquid volume match, respecting the bowl's actual (non-flat, non-rectangular) interior shape.

## Overflow (the weir law)

Wherever the tilted surface height exceeds the bowl rim height at any point around the perimeter, a **weir law** — a standard hydraulics formula for flow over the edge of a container — estimates the outflow rate at that point: `dQ = (2/3) * Cd * sqrt(2g) * head^1.5 * dl`, integrated around the whole rim outline (`G.rim_ds`, sampled from the same outline function that defines the bowl's 3D shape). `Cd = 0.6` is called out explicitly as "an uncalibrated assumption," not a measured discharge coefficient. Total volume lost this way is subtracted from the remaining liquid volume, with an explicit mass-balance check (spilled + remaining must equal initial, tested down to `1e-12` in `tests/test_rcflex_liquid.m`).

## Visualization droplets

Once outflow is computed for a rim segment, "droplet" particles are spawned purely for animation purposes: they inherit the local outflow velocity and then follow a simple ballistic (gravity-only) trajectory. The docs are explicit that these illustrate discharged volume, not a physically validated droplet-size or breakup model.

## What the model explicitly does NOT capture

Per `docs/LIQUID_MODEL.md` and code comments: no surface tension, no contact angle / wetting films, no splash breakup, no higher-order surface waves (only two flat-plane tilt modes), and no two-way fluid-structure coupling (fluid forces don't push back on the mechanism). The model raises an error if effective gravity in the bowl frame drops below 0.5 m/s² or if surface slope exceeds the small-tilt range it was designed for (`simulate_rcflex_liquid.m`'s `SensoSpoon:FluidSeparation` / `SensoSpoon:FluidNonlinear` errors) — i.e., it deliberately refuses to produce a number outside the regime it's valid for, rather than silently extrapolating.

## The demo scripts

- **`run_rcflex_soup_demo.m`** — recomputes the mechanical simulation with a specified liquid volume (default 5 mL) as payload, runs the fluid model, and produces a side-by-side comparison animation (`animate_rcflex_comparison`, see `05_GEOMETRY_AND_VISUALIZATION.md`) showing an uncompensated (gimbal rigidly locked) bowl next to the compensated (closed-loop controlled) bowl, both driven by identical motion and identical liquid physics.
- **`run_rcflex_strong_soup.m`** — a single fixed, more aggressive scenario (4 Hz, 5°/axis, 8 mL, 4 seconds), also checking that all the hardware limits (current, stroke, joint angle, power, side load) still hold, then repeating the fluid calculation at half grid spacing/time step as a numerical-sensitivity check, and producing `soup_strong_tremor_comparison.mp4`.
- **`screen_rcflex_strong_soup.m`** — a small sweep over amplitude (3°, 4°, 5°) using the *fast* control integrator (not native Simulink) to screen which amplitudes stay within hardware limits before committing to the (slower) native-Simulink strong-tremor case, writing `strong_soup_screen.csv`.

## Reported numeric results (from `docs/LIQUID_MODEL.md`)

- 5 mL case: neither the uncompensated nor compensated side loses any liquid over 2 seconds at 8 Hz, 1°/axis.
- 8 mL case: the uncompensated side loses about 0.0016 mL (a trace amount); compensated side loses none.
- Strong-tremor case (4 Hz, 5°/axis, 8 mL, 4 s): uncompensated side loses about 2.728 mL (~34.1% of the initial fill); compensated side loses none computed. Attitude RMS error drops from 4.999° to 0.371° (a 92.58% reduction) — but bowl **position** RMS stays at 4.06 mm regardless, illustrating the documented point that "no overflow" is not the same as "no vibration": a 2-axis rotation-only gimbal cannot fully stop the bowl's translational motion.
- Grid/step refinement (halving both) changes these numbers only slightly (~1.7% relative difference in the strong case), used as a numerical-sensitivity check, not as evidence of physical accuracy.
