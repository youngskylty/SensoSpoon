# The Control System (`src/control/`)

This is the algorithm that tries to keep the spoon bowl level despite hand tremor. It's built from two nested control loops (attitude, then current), a sensor-fusion/estimation step, a calibration procedure, and a substantial set of test/benchmark/tuning scripts.

## What "closed loop" means here, concretely

At runtime, nothing in the controller ever sees the "true" state of the plant or the "true" hand motion. It only ever sees:
- The 14-column sampled/noisy/delayed sensor vector (`rcflex_imu_sample.m` output plus quantized current) — two IMUs' worth of gyro+accel, delivered at 800 Hz.
- The 3-column delayed current+bus-voltage ADC reading, delivered at 10 kHz.

Everything the controller "believes" about hand intent, spoon orientation, and current state is *estimated* from these two noisy, delayed, sampled signals — never read directly off the simulation's internal state. Ground truth is used *only* for scoring after the fact (`rcflex_control_metrics.m`), never fed back into the control law. Several code comments emphasize this explicitly (e.g. `rcflex_attitude_step.m`: "No plant truth, base input or future samples").

## Files at a glance

| File | Role |
|---|---|
| `rcflex_control_parameters.m` | All tunable controller gains/settings, bundled as struct `C`. |
| `rcflex_calibrate_control.m` | Two-second stationary calibration to estimate sensor bias before running. |
| `rcflex_prepare_control.m` | Precomputes the fixed intent-estimation filter coefficients. |
| `rcflex_imu_sample.m` | Simulates what the two BMI270 IMUs actually output: noise, bias, quantization, internal filtering. |
| `rcflex_current_sample.m` | Simulates the delayed current/bus-voltage ADC. |
| `rcflex_attitude_step.m` | The outer-loop control law: attitude estimation + PID + current allocation. |
| `rcflex_current_step.m` | The inner-loop PI current controller. |
| `rcflex_controller_sfun.m`, `rcflex_current_sfun.m`, `rcflex_current_adc_sfun.m` | Simulink S-function wrappers around the above, for use inside `SensoSpoon_Control.slx`. |
| `rcflex_control_case.m` | Generates synthetic test scenarios (sine, chirp, mixed, step, intent, etc.). |
| `rcflex_control_metrics.m` | Scores a simulation run against ground truth (never used inside the loop). |
| `run_rcflex_control.m`, `run_rcflex_control_fast.m` | The two ways to run a closed-loop simulation (native Simulink vs. fast benchmark integrator — see `02_SIMULINK_MODELS.md`). |
| `benchmark_rcflex_control.m` | Runs 20 held-out test scenarios with frozen gains and reports pass/fail against hardware limits. |
| `tune_rcflex_control.m` | A small grid search over filter cutoff / bandwidth on a declared training set. |
| `plot_rcflex_control.m` | Produces the 6-panel response plot for one run. |
| `write_rcflex_control_report.m` | Assembles `output/results/CONTROL_RESULTS.md` from the saved benchmark/native results. |

## Step 1: Calibration (`rcflex_calibrate_control.m`)

Before the loop starts, the system assumes **two seconds of the spoon sitting stationary and level**. During this window, it repeatedly calls the noisy IMU sampling function (`rcflex_imu_sample.m`) on a known "level and still" ground truth and averages 1600 samples (2 s at 800 Hz) to estimate each IMU's gyro and accelerometer bias. This mimics what you'd do with real hardware: you can't know a chip's exact bias offset in advance, so you average noisy samples at a known reference orientation. The estimated biases are stored in `C.gyro_bias` / `C.accel_bias` for use by the attitude estimator afterward. This is explicitly *not* the same as just reading the "true" hidden bias constants that the sensor model uses internally — it's a simulated calibration procedure using the same noisy measurement path a real device would use.

## Step 2: Attitude estimation and the outer loop (`rcflex_attitude_step.m`)

This single function is the most information-dense file in the project. It runs once per 800 Hz tick (`dt = 1/C.outer_hz`) and does all of the following:

### a) Attitude estimation (complementary filter, one per IMU)
For each of the two IMUs (handle, spoon), it maintains a 3×3 rotation matrix `R{j}` representing that body's estimated orientation. Each step:
- Removes the calibrated gyro bias, then integrates the (bias-corrected) angular rate over `dt` using a proper rotation-matrix exponential update (`increment(gyro*dt)` — a standard small-rotation update using Rodrigues' formula, avoiding the small-angle linearization error you'd get from naively adding angles).
- Applies a *complementary correction*: it estimates how much to "trust" the accelerometer as a gravity reference based on how close its magnitude is to 1 g (`trust = max(0, 1 - |‖acc‖-g| / (0.12g))`) — if the sensor is experiencing large non-gravity acceleration (e.g. during a fast shake), the accelerometer reading is untrustworthy as a "down" reference, so its correction gets down-weighted or ignored. This is the classic **complementary filter** idea: gyroscopes are accurate short-term but drift slowly; accelerometers give an absolute "down" reference but are corrupted by any real acceleration; blending them (gyro-primary, slow accelerometer correction) gives a stable estimate. The correction rate is set by `C.complement_hz` (0.05 Hz — i.e. a very slow, gentle correction).

### b) Relative angle and hand-intent extraction
`rel = Rh' * Rs` gives the spoon's orientation relative to the handle, decomposed back into pitch/roll angles `q`. Separately, `hand` extracts the handle's own absolute pitch/roll (this is the signal from which "what is the person intentionally doing with their hand" must be estimated).

### c) The intent filter (fixed IIR chain)
The handle's estimated orientation is passed through a fixed chain of **second-order sections** (SOS — a numerically well-behaved way to implement a higher-order digital filter as a cascade of simple 2nd-order stages) computed once in advance by `rcflex_prepare_control.m`:
- Five **notch filters** at 4, 6, 8, 10, 12 Hz (`C.notch_frequencies_Hz`), each attenuating a narrow band around a candidate tremor frequency, since essential tremor commonly falls in the 4–12 Hz range.
- A bounded **phase-lead** section (`C.intent_lead_s`), which nudges the phase of the surviving signal to partially compensate for the delay the notches and low-pass introduce.
- A second-order **10 Hz low-pass rolloff** (`C.intent_rolloff_Hz`) to remove anything left above the intended tracking band.

The result, `ref`, is the estimate of "what the hand is intentionally doing" (the reference the spoon *should* follow), with tremor frequencies notched out. Because this is a **fixed, causal, offline-designed filter** — coefficients computed once before the simulation starts, applied with ordinary recursive filtering at runtime — it can never look at future samples or achieve perfect (zero-phase) separation; this is explicitly called out in `docs/CONTROL.md` ("The estimator never reads the test input's true intent, future data, or zero-phase filtering results").

### d) Reference limiting and rate limiting
The desired relative angle is clamped to `±C.joint_limit` (14°, deliberately inside the ±15° physical hard stop) and rate-limited (`C.reference_rate`, 180°/s) — both applied as memory (state `z(52:53)`) so the *desired* trajectory itself never asks for something physically impossible, even before any tracking error appears.

### e) PID attitude control
Error is computed as a proper 3D rotation-vector error between the desired orientation `Rd` and actual spoon orientation `Rs` (using the antisymmetric part of `Rd' * Rs`, a standard way to get a small-angle rotation error vector from two rotation matrices), then decomposed into the two physical joint axes via `axisMap`. The core control law is a **PID-like structure written directly in terms of desired angular acceleration**:

```matlab
accel = -wn^2 * e - 2*C.damping*wn*rateErr - C.integral_gain * integral
```

This is a second-order (proportional + derivative, i.e. "PD" gains expressed via natural frequency `wn = 2*pi*C.natural_hz` and damping ratio `C.damping`) term plus an integral term, which is the classic way engineers parameterize a PID loop when you want direct control over closed-loop bandwidth and damping rather than raw P/I/D gains. The integrator is clamped to ±0.02 (rad, an anti-windup limit) and explicitly **frozen** (not updated) whenever the actuator allocation saturates (see below) — a common and important anti-windup technique: if the actuators can't deliver what's being commanded, don't let the integral term keep growing.

### f) Feedforward compensation
Rather than relying purely on feedback, the desired acceleration is converted to torque using the actual mass matrix and gravity/flexure compensation: `torque = M*accel + gravity + flexure_stiffness.*q + flexure_damping*qd + friction_term`. This is **model-based feedforward** — since the physics model (`rcflex_mass_bias.m`, `rcflex_linkage.m`) is known, the controller can directly cancel out gravity, spring, and friction effects rather than asking the feedback loop to fight them, which generally gives much better tracking than pure feedback alone.

### g) Current limiting, thermal derating, and side-load limiting
Before deciding how much current to request, the controller estimates coil power (from measured current, not ground truth) and integrates a lumped thermal estimate (`z(40:41)`) to decide whether continuous-duty derating is needed (dropping the allowed current below the 1.5 A hard limit if average power has exceeded the rated continuous power, `H.coil_continuous_power_W`). It also computes a side-load-based current limit (`0.95 * shaft_side_load_N / (ratio * K)`) — since the actuator rods can also experience a lateral force which the shaft bearing has a rated limit for, and that lateral force scales with the actuator's own commanded force.

### h) Allocation: the "two coils, two axes" problem
Two actuators produce torque about two joint axes through a fixed geometric Jacobian, so converting "desired torque" into "how much current in each coil" is a small linear-algebra problem, but with the added complication that each coil's current is individually bounded. The code solves this by literally **enumerating all nine combinations** of each coil being free, saturated positive, or saturated negative (`a1, a2 ∈ {-1, 0, 1}`, so 3×3 = 9 cases — this is a textbook "active-set" approach to a small bounded least-squares problem), picking whichever feasible combination minimizes tracking error. The comment notes this deliberately avoids needing MATLAB's Optimization Toolbox — it's small enough (2 unknowns) to brute-force.

### i) Output
The function returns the current command (2 values) plus an estimated back-EMF feedforward term (`K.*sd`, since the inner current loop uses this too), plus a 16-value diagnostic vector (joint angle, reference, error, current command, limits, thermal/power state, saturation flag, and the linkage Jacobian's condition number — a measure of how close the mechanism is to a kinematic singularity).

## Step 3: The inner current loop (`rcflex_current_step.m`)

A much simpler, faster (10 kHz) **PI controller with feedforward**:

```matlab
raw = (R_coil + R_driver)*command_current + back_EMF_feedforward + Kp*error + integral
v = clamp(raw, -bus, bus)
integral += (Ki*error + Kaw*(v - raw)) / current_hz
```

The `Kp*error + integral` part is the classic PI structure; the resistance and back-EMF terms are **feedforward** (since both are already known/estimated, directly cancelling them reduces how much work the PI loop itself has to do); and `Kaw*(v - raw)` is **back-calculation anti-windup** — a standard technique (referenced directly in `docs/CONTROL.md`, linking to MathWorks' documentation on it) where, if the commanded voltage had to be clamped due to saturation, the integral term is corrected in proportion to how much clamping occurred, preventing it from winding up uselessly.

## Sensor and ADC sampling models

- **`rcflex_imu_sample.m`**: given the "true" sensor reading, applies a single-pole low-pass filter (approximating the sensor chip's own internal filter, at 100 Hz), adds Gaussian noise scaled by the datasheet noise-density spec (`H.imu_gyro_noise_dps_sqrtHz`, `H.imu_acc_noise_g_sqrtHz`) times `sqrt(fs/2)` (the standard way to convert a noise-density spec into a per-sample noise standard deviation), adds a fixed per-axis bias, and finally quantizes to the sensor's real bit resolution and clamps to its measurement range. Each call uses a seeded random stream keyed off simulation time, so results are reproducible run-to-run for a given seed.
- **`rcflex_current_sample.m`**: similarly adds offset and noise to the true coil current, quantizes to a 12-bit ADC resolution, and separately quantizes an estimated bus voltage — all seeded and reproducible.

## Test scenario generation (`rcflex_control_case.m`)

A single flexible generator that builds synthetic "handle motion" scenarios by name: `'sine'` (single tremor frequency), `'chirp'` (frequency sweeping over the run), `'mixed'`/`'irregular'` (multiple superimposed frequencies, mimicking a less clean tremor), `'step'` (an amplitude doubling partway through, testing transient response to a sudden disturbance change), `'intent'`/`'combined'` (adds a slow intentional hand-motion component on top of or instead of tremor, testing whether the controller correctly lets intentional motion through while rejecting tremor). It always builds a smooth (quintic, i.e. 5th-order polynomial) startup ramp so nothing switches on with a discontinuity. Comment explicitly notes: "Intent truth is never supplied to the controller" — this ground truth is only used later for scoring.

## Scoring (`rcflex_control_metrics.m`)

This is where ground truth *is* legitimately used — strictly for after-the-fact evaluation, never inside the loop. It computes:
- **Attitude RMS error**: the true rotation-vector error between the intended spoon orientation and the actual spoon orientation, RMS'd over a "steady" scoring window (`t >= 1.5 s`, letting startup transients settle).
- **Baseline RMS error**: what the same error would have been with the gimbal rigidly locked (no actuation at all) — the natural "no compensation" reference.
- **Suppression percent**: `100 * (1 - controlled_RMS / baseline_RMS)` — the headline metric quoted throughout the docs and `CONTROL_RESULTS.md`.
- Position RMS (how much the bowl's physical location in space still moves, since a 2-axis rotation-only gimbal cannot fully cancel translational disturbance — this is explicitly called out as a fundamental limitation, not a bug).
- Peak/RMS current, voltage, stroke, joint angle, average power, temperature rise, allocation-saturation percentage, and peak estimated shaft side load — all the hardware-limit checks used by the benchmark suite.

## Benchmarking, tuning, and reporting

- **`tune_rcflex_control.m`** — a small, explicitly-labeled *training* grid search (varying the old lowpass-filter cutoff and attitude bandwidth) over a 3-frequency tremor set plus one "intent-only" case, writing `control_training.csv`. The docs are clear that the *final* selected design uses the notch-filter approach, not the lowpass filter this script explores — this script is a record of an earlier design exploration, and the lowpass filter is kept only "as a reproducible baseline" (`C.intent_filter='lowpass'`).
- **`benchmark_rcflex_control.m`** — the actual validation suite: 20 named scenarios (varying frequency, amplitude, payload, bus voltage, parameter errors, and out-of-band frequencies) run with **frozen** gains and **unseen** random seeds (`C.seed = 9100+k`), explicitly meant to test generalization beyond the training grid. For each case it checks whether results stay "within limits" (joint angle, stroke, current, side load, power) and whether suppression reaches a 90% target, writing per-case `.mat` files and a summary `control_benchmark.csv`.
- **`write_rcflex_control_report.m`** — reads the benchmark CSV plus the native-Simulink numerical-verification results and writes the human-readable `output/results/CONTROL_RESULTS.md` table (reproduced and explained in `08_GENERATED_OUTPUTS.md`).
- **`plot_rcflex_control.m`** — a 6-panel figure (pitch/roll error vs. locked baseline, coil currents, actuator stroke, bowl position error, copper loss) for a single run, saved as `output/figures/control_response.png` (or a custom filename).

## What the benchmark results actually show (from `CONTROL_RESULTS.md`)

At the frozen, final gain set: single-frequency tremor within the 4–12 Hz design band (cases 1–3, 16–19) is suppressed roughly 90–98%. Larger amplitude (3°/axis, cases 4–5) still suppresses ~89–92% but with more current draw. Combined intentional-motion-plus-tremor (case 9) suppresses only ~51.5% — the intent/tremor separation is imperfect when both are present simultaneously, which the docs call out directly. Intent-only tracking (case 10) has no "suppression" defined (baseline error is zero by construction) and is reported as a direct tracking error instead. A step change in amplitude (case 14) and an out-of-band 20 Hz disturbance (case 20) both *fail* the within-limits check and show large negative "suppression" (meaning the controlled case is worse than doing nothing) — these are explicitly retained as documented failure modes, not hidden.
