# Flat File Index

One paragraph per file, for quick lookup. See the topic files (`01`–`08`) for deeper explanations and how files connect to each other.

## Root

| File | Purpose |
|---|---|
| `START_PROJECT.m` | The single entry point. Sets up MATLAB paths and Simulink cache locations, then dispatches to one of: `setup`, `preview`, `test`, `simulate`, `control`, `compare`, `soup`, `benchmark`, `analysis`, `all`. Rebuilds the 3D geometry assets automatically if the hardware parameters have changed since the last cached build. |
| `src/project_path.m` | Returns absolute paths to project areas (`assets`, `models`, `figures`, `results`, `cache`) computed from this file's own location (`mfilename('fullpath')`), so every script's file I/O works regardless of MATLAB's current working folder. |

## `src/model/` — core physics

| File | Purpose |
|---|---|
| `rcflex_parameters.m` | Returns struct `H`: every geometric dimension, actuator/driver/battery/flexure/IMU spec, and linkage constant describing the current design. The single source of truth for numeric parameters. |
| `rcflex_linkage.m` | Given joint angles `q=[pitch;roll]` and `H`, computes the exact direct-pushrod kinematics: rod attachment points, slider positions along the actuator axis, and the transmission Jacobian `J` relating joint-angle rates to actuator-slider rates. |
| `rcflex_mass_bias.m` | Assembles the 2×2 rigid-body mass matrix and the "bias" vector (gravity, centripetal, gyroscopic, and moving-base terms) for the whole mechanism, using per-body kinematics computed by an internal `kin(...)` helper and complex-step derivatives for velocity terms. |
| `rcflex_body_properties.m` | Loads the generated mesh assembly and computes mass/center-of-mass/inertia for each of the six rigid bodies (pitch carrier, spoon carrier + payload, two actuator movers, two pushrods). |
| `rcflex_physics.m` | The core state-derivative function: given state `x` (joint angles/rates, coil currents, coil temperatures) and input `u` (hand motion, applied voltage, world translation), returns `dx` for integration plus a fixed 26-column diagnostic output vector. Implements the RL electrical circuit, force-vs-stroke curve, flexure spring/damping, compliant end stops, friction, current chopping, and thermal model. |
| `rcflex_sensor_truth.m` | Computes the exact (noise-free) readings two body-mounted IMUs (handle, spoon) would produce given the true plant state and base motion — gyro and accelerometer, including gravity and true linear acceleration. |

## `src/simulink/` — plant-only Simulink model

| File | Purpose |
|---|---|
| `build_rcflex_analytic.m` | Programmatically constructs `models/SensoSpoon_Analytic.slx`: voltage/base-motion source blocks, the plant S-function, sensor S-function, and logging sinks; sets the `ode15s` solver configuration. |
| `rcflex_plant_sfun.m` | Level-2 MATLAB S-function wrapping `rcflex_physics.m` as a continuous-state Simulink block (8 continuous states, `Derivatives`/`Outputs` callbacks). |
| `rcflex_sensor_sfun.m` | Level-2 S-function wrapping the IMU/current sampling at a fixed 800 Hz rate, holding sampled/filtered state in Simulink Dworks. |
| `run_rcflex_analytic.m` | The callable wrapper: builds default open-loop voltage test signals if none supplied, runs the analytic model via `sim(...)`, and returns/saves a result struct `S`. |

## `src/geometry/` — parametric 3D geometry

| File | Purpose |
|---|---|
| `generate_rcflex_assets.m` | Builds the entire parts list (shell, bezel, actuators, flexure sleeves, pushrods, spoon bowl/neck, electronics, battery, etc.) as parametric triangle meshes, cleans up each mesh (weld vertices, fix winding), writes one STL per part plus `assembly.mat` and `parts_manifest.csv`. |
| `rcflex_mesh_mass_properties.m` | Exact tetrahedral-decomposition mass-property calculation (centroid, inertia tensor, volume) for a closed triangle mesh given an assumed total mass. |
| `rcflex_spoon_outline.m` | Defines the bowl's plan-view half-width as a function of distance from the tip (elliptical nose, blended shoulder, cubic-Hermite neck transition) — shared by the 3D mesh generator, the liquid-geometry sampler, and the dimension-verification preview. |

## `src/visualization/` — previews, animations, plots

| File | Purpose |
|---|---|
| `animate_rcflex_analytic.m` | Animates the full assembled/cutaway 3D model following a single simulation's solved joint angles and actuator strokes; optionally exports an MPEG-4 video. Used for both open-loop and closed-loop single-run demos. |
| `animate_rcflex_comparison.m` | Draws two synchronized copies of the assembly (locked baseline vs. controlled) side by side under identical base motion, with live error-trace plots, optional overlaid liquid-surface/droplet visualization, and real-time-then-slow-motion video export. |
| `plot_rcflex_plant.m` | 6-panel static plot of an open-loop analytic run (voltage, current, angle, stroke, force, copper loss). |
| `preview_rcflex_design.m` | Static 3D preview of the whole assembly in `assembled`, `cutaway`, `exploded`, or `orthographic` (2×2 multi-view) mode. |
| `preview_spoon_closeup.m` | A tight-camera static render isolating the bowl/neck/carrier/collar. |
| `preview_spoon_dimensions.m` | A 2D dimensioned-drawing-style figure validating bowl dimensions against `rcflex_spoon_outline.m`. |

## `src/analysis/` — feasibility studies, independent of the control algorithm

| File | Purpose |
|---|---|
| `rcflex_tremor_input.m` | Generates a synthetic single-frequency sinusoidal tremor disturbance (rotation + small translation) in standard 15-column base-motion format. |
| `rcflex_world_metrics.m` | Computes world-frame bowl position and attitude/tilt metrics from a joint-angle history and base-motion history, independent of the source simulation. |
| `evaluate_rcflex_envelope.m` | Inverse-dynamics feasibility sweep: assuming *perfect* attitude tracking, computes required current/voltage/power/side-load/stroke across payload/frequency/amplitude combinations and checks against hardware limits — not a simulation of the actual controller. |
| `evaluate_rcflex_robustness.m` | Reruns the envelope sweep with several hardware parameters deliberately worsened, to gauge sensitivity of the feasibility conclusion. |
| `run_tremor_platform_check.m` | A passive-response smoke test: applies a mixed rotation+translation disturbance with zero motor voltage and confirms the simulation stays numerically finite. |

## `src/control/` — the closed-loop controller

| File | Purpose |
|---|---|
| `rcflex_control_parameters.m` | Returns struct `C`: all controller gains and settings (loop rates, intent-filter design, PID/PI gains, joint/rate limits, sensor-model noise settings). |
| `build_rcflex_control.m` | Copies the analytic Simulink model, removes the external voltage source, and wires in the attitude-controller, current-ADC, and current-PI S-function blocks to form `models/SensoSpoon_Control.slx`. |
| `rcflex_controller_sfun.m` | S-function wrapper for the 800 Hz outer attitude loop, calling `rcflex_attitude_step.m` each tick and storing its 100-element state vector in a Simulink Dwork. |
| `rcflex_current_sfun.m` | S-function wrapper for the 10 kHz inner current PI loop, calling `rcflex_current_step.m`. |
| `rcflex_current_adc_sfun.m` | S-function wrapper for the 10 kHz delayed current/bus-voltage ADC sampling, calling `rcflex_current_sample.m`. |
| `rcflex_attitude_step.m` | The outer-loop control law: dual complementary-filter attitude estimation, fixed notch/lead/lowpass intent filter, PID with feedforward, thermal/side-load-aware current limiting, and a 9-case bounded least-squares current allocator across the two coils. |
| `rcflex_current_step.m` | The inner-loop PI current controller with resistance/back-EMF feedforward and back-calculation anti-windup. |
| `rcflex_current_sample.m` | Simulates the delayed, noisy, quantized 10 kHz current and bus-voltage ADC. |
| `rcflex_imu_sample.m` | Simulates the sampled, filtered, biased, quantized, range-limited 800 Hz dual-IMU output, given the true sensor reading. |
| `rcflex_calibrate_control.m` | Simulates a 2-second stationary-level calibration, averaging noisy IMU samples to estimate gyro/accelerometer bias before the control loop starts. |
| `rcflex_prepare_control.m` | Precomputes fixed second-order-section filter coefficients (five notches, phase-lead, low-pass rolloff, or the alternative simple lowpass) once, before runtime. |
| `rcflex_control_case.m` | Generates named synthetic test scenarios (`sine`, `chirp`, `mixed`, `irregular`, `step`, `intent`, `combined`) as handle base-motion trajectories, with a smooth quintic startup ramp. |
| `rcflex_control_metrics.m` | Scores a completed simulation run against ground truth (used only for evaluation, never inside the control loop): attitude RMS, suppression %, position RMS, current/voltage/stroke/power/side-load peaks. |
| `run_rcflex_control.m` | Runs the native `SensoSpoon_Control.slx` Simulink model (via `sim(...)`, `ode23t` solver) for a given scenario and saves the result. |
| `run_rcflex_control_fast.m` | A hand-written multirate MATLAB loop reimplementing the identical control-law calls but with a much cheaper (fixed-step, held-acceleration) mechanical integration — used for fast benchmarking/tuning, cross-checked against the native model. |
| `benchmark_rcflex_control.m` | Runs 20 held-out scenarios (varying frequency, amplitude, payload, bus voltage, parameter errors, out-of-band frequency) with frozen gains and unseen seeds, checking hardware limits and a 90% suppression target; writes `control_benchmark.csv`. |
| `tune_rcflex_control.m` | An early/exploratory grid search over intent lowpass cutoff and attitude bandwidth on a small training set; writes `control_training.csv`. |
| `plot_rcflex_control.m` | 6-panel closed-loop response plot (errors, currents, stroke, position error, copper loss) for one run. |
| `write_rcflex_control_report.m` | Reads the benchmark CSV and native/numerical-verification results and writes the human-readable `output/results/CONTROL_RESULTS.md`. |

## `src/fluid/` — liquid slosh model (one-way coupled add-on)

| File | Purpose |
|---|---|
| `rcflex_liquid_parameters.m` | Returns struct `P`: liquid density, damping ratio, spatial grid size, integration step, output rate, weir (overflow) coefficient. |
| `rcflex_liquid_geometry.m` | Samples the bowl's analytic interior profile onto a grid for volume quadrature, and builds a rim outline for overflow calculations, reusing `rcflex_spoon_outline.m`. |
| `rcflex_liquid_level.m` | Given a target liquid volume and a current surface tilt, solves (by bisection) the flat-surface height that holds exactly that volume within the actual bowl shape. |
| `rcflex_liquid_motion.m` | Extracts the bowl-frame origin, orientation, velocity, angular velocity, and linear/angular acceleration at a given time step, for either the locked or the controlled motion path. |
| `simulate_rcflex_liquid.m` | The main fluid simulation: two-mode damped tilt dynamics driven by effective in-bowl gravity, weir-law rim overflow, mass-balance bookkeeping, and outflow "droplet" event generation, for both the locked and controlled sides. |
| `run_rcflex_soup_demo.m` | Sets up a mechanical run with a given liquid payload volume, runs the fluid model, and produces a comparison animation. |
| `run_rcflex_strong_soup.m` | A fixed, more aggressive 4 Hz/5°/8 mL scenario with hardware-limit checks, grid-refinement sensitivity check, and comparison video. |
| `screen_rcflex_strong_soup.m` | A fast-integrator amplitude sweep (3°/4°/5°) screening hardware feasibility and spill before committing to the slower native strong-tremor case. |

## `tests/` — validation suite

| File | Purpose |
|---|---|
| `run_validation.m` | Runs all eleven check functions in sequence, times them, and writes `validation_summary.csv`. |
| `test_rcflex_mesh_properties.m` | Confirms `rcflex_mesh_mass_properties.m` matches a known-box's textbook inertia, and that every generated part mesh is a closed (manifold) surface. |
| `test_spoon_reference.m` | Confirms the generated bowl mesh's dimensions match `rcflex_parameters.m` and that `rcflex_spoon_outline.m` reproduces the intended reference widths and peak-width station. |
| `test_rcflex_packaging.m` | Confirms handle-mounted actuator/battery/electronics parts stay inside the outer shell profile. |
| `test_rcflex_shell_clearance.m` | Sweeps gimbal angle and confirms moving parts never intersect the fixed handle shell (plus an inactive optional "guard" check). |
| `test_rcflex_front_cap.m` | Confirms both pushrods pass cleanly through their front-cap clearance windows, and the moving gimbal body never crosses the front cap plane, across the angle range. |
| `test_rcflex_spherical_clearance.m` | Confirms the moving spherical carrier never intersects the fixed spherical bezel across the angle range. |
| `test_rcflex_physics.m` | Core dynamics validation: rod-closure accuracy, stroke feasibility, Jacobian accuracy (finite-difference and complex-step cross-checks), positive-definite mass matrix, electromechanical energy balance, force-constant knockout, and passive decay/solver-convergence checks. |
| `test_rcflex_hardware.m` | Component-level electrical/mechanical acceptance tests: locked-rotor RL step response, reversed-polarity symmetry, current chopping, bus-voltage clamp, copper heating, payload sensitivity, accelerometer gravity reading. |
| `test_rcflex_translation.m` | Confirms base translation excites the dynamics and both IMUs correctly, and that "level" (near-zero attitude error) is distinct from "physically stationary" (nonzero position RMS). |
| `test_rcflex_control.m` | Confirms the inner current loop tracks/reverses/recovers correctly, the intent filter is stable with unity DC gain, the attitude controller is deterministic, current stays within its limit, and the feedback sign is stabilizing (not destabilizing). |
| `test_rcflex_liquid.m` | Confirms volume reconstruction, no slosh/spill in still water, identical results under identical forcing, natural overflow from a deliberately tilted near-full fixture, monotonically non-increasing liquid volume, and mass-balance/droplet-accounting consistency. |
| `verify_rcflex_control_numerics.m` | Standalone (not part of `run_validation.m`): cross-checks the fast multirate control integrator against the native Simulink reference at three mechanical step sizes, feeding the results into `CONTROL_RESULTS.md`. |
