# Analysis Scripts (`src/analysis/`)

These scripts don't run the closed-loop controller at all — they ask a different, narrower question: *even with perfect, ideal control, what would the hardware be asked to do, and is that physically achievable given the actuators' current/voltage/thermal/side-load limits?* This is an **inverse-dynamics feasibility study**, not a simulation of the actual control algorithm succeeding or failing. The docs and code comments repeatedly stress this distinction (e.g. `evaluate_rcflex_envelope.m`: "NOT an implemented controller or simulation of successful suppression").

## Files

- `rcflex_tremor_input.m` — generates a synthetic single-frequency sinusoidal hand-tremor disturbance (rotation + a small translation), in the same 15-column "base motion" format used everywhere else.
- `rcflex_world_metrics.m` — computes world-frame (not relative-to-handle) attitude and bowl-position metrics from a joint-angle history and a base-motion history — independent of whichever simulation produced the joint angles.
- `evaluate_rcflex_envelope.m` — the core inverse-dynamics feasibility sweep: for a grid of payload/frequency/amplitude combinations, assumes *perfect* attitude tracking (the spoon exactly cancels the tremor) and works backward to compute what current, voltage, and side-load that would require, then checks it against hardware limits.
- `evaluate_rcflex_robustness.m` — runs the same envelope evaluation twice: once with nominal hardware parameters, once with deliberately worsened ("adverse") parameters, to see how sensitive the feasibility conclusions are to parameter uncertainty.
- `run_tremor_platform_check.m` — a basic dynamic smoke test: apply a tremor-like disturbance with the motor voltage forced to zero (a "shorted/braking" bridge state) and confirm the whole simulation stays numerically well-behaved.

## `rcflex_tremor_input.m` — what question it answers

"What does a clean, single-frequency, one-axis-phase-offset tremor disturbance look like, in the same input format the rest of the project expects?" It's explicitly labeled "a reproducible synthetic scenario, NOT a clinical patient recording" — there's no real tremor-patient data anywhere in this project; every test signal is synthetic. It outputs rotation (angle/velocity/acceleration) and a small proportional translation (position/velocity/acceleration, three different amplitude/phase combinations per axis so the translation isn't perfectly in-phase with rotation) in the standard 15-column base format.

## `rcflex_world_metrics.m` — what question it answers

"Given some joint-angle history and some base-motion history, how much does the spoon *actually* move and tilt in absolute world coordinates (not just relative to the handle)?" It computes the world position of the bowl's measurement point, the "tilt" (angle between the bowl's local z-axis and true vertical — essentially "how far from level is the bowl"), and the full 3D attitude angle, then reports RMS summaries for each. This is used both by the envelope-feasibility study and directly inside a unit test (`test_rcflex_translation.m`) to distinguish "the spoon stayed level" from "the spoon stayed in the same physical place" — these are not the same claim, and the whole project is careful to keep them separate (a rotation-only 2-axis gimbal can, in principle, keep the bowl level while its physical position still moves around in space with the hand).

## `evaluate_rcflex_envelope.m` — the feasibility sweep, in detail

For each combination of food payload (5 g, 15 g), tremor frequency (4, 8, 12 Hz), and amplitude (1°, 3°, 5°), it:
1. Assumes the spoon joint angle exactly cancels the tremor (`q = -base(:,[2 1])`, i.e. perfect, instantaneous, ideal tracking — not something any real controller achieves).
2. Uses the *inverse* of the dynamics (`rcflex_mass_bias.m`) to compute what joint torque would be required to produce that assumed motion — this is standard inverse dynamics, the same physics equations used in the forward simulation, just solved for torque given motion rather than motion given torque/force.
3. Converts that torque into required actuator force, then current, then voltage (accounting for the force-constant curve and back-EMF), and checks: does peak current exceed `H.current_limit_A`? Does required voltage exceed available bus voltage? Does average copper loss exceed `H.coil_continuous_power_W`? Does the estimated shaft side load exceed `H.shaft_side_load_N`? Does required actuator stroke exceed `H.vca_stroke_mm`?
4. Also computes, using `rcflex_world_metrics.m`, how much the bowl's *position* would still move even under "ideal attitude tracking" — reinforcing the point that leveling the bowl doesn't stop it from moving through space.

The result is a big feasibility table (`tremor_hardware_envelope.csv`) plus a 2-panel figure: required peak current vs. frequency for two payload levels (compared against the hardware current limit), and a bar chart contrasting "locked spoon" vs. "ideal attitude tracking" position RMS at one particular frequency/amplitude — visually making the "leveling isn't the same as stopping" point.

This tells you: is the actuator/driver/battery combination even *in principle* strong enough for the range of tremor scenarios being considered, ignoring for the moment whether any real controller can actually achieve that perfect tracking? It's a hardware-sizing sanity check, run independently of (and prior to) the actual control-algorithm evaluation in `03_CONTROL_SYSTEM.md`.

## `evaluate_rcflex_robustness.m` — what question it answers

"How much does the feasibility conclusion change if several hardware parameters are simultaneously worse than nominal?" It reruns the same envelope sweep with force constant reduced 15%, coil resistance increased 15%, bus voltage dropped to 6.4 V, flexure stiffness increased 10%, and guide friction doubled — a bundle of "deliberately adverse engineering assumptions," per the code comment, explicitly not meant to represent a guaranteed manufacturer tolerance but a stress-test of the conclusion's sensitivity. Results for both the nominal and adverse cases are concatenated into one table (`tremor_robustness.csv`) with a `scenario` column distinguishing them, and a one-line summary reports how many of the sampled cases remain feasible under each assumption set.

## `run_tremor_platform_check.m` — what question it answers

A basic sanity/smoke test, not a suppression evaluation: apply a tremor-like combined rotation+translation disturbance with the motor bridge held at zero voltage (`mode='off'`, meaning shorted/braking, not open-circuit — see `docs/RCFLEX_MODEL_GUIDE.md`), and simply confirm nothing blows up numerically (`assert(all(isfinite(...)))`). This is a passive-response check — "does the un-actuated mechanism behave sensibly under combined rotation and translation disturbance" — explicitly not a stabilization or suppression score (the fprintf message says so directly).
