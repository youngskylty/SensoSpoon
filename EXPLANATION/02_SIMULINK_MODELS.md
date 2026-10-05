# The Simulink Models (`src/simulink/`, `models/*.slx`)

The two `.slx` files (`SensoSpoon_Analytic.slx` and `SensoSpoon_Control.slx`) are binary Simulink model files — not human-readable as text. But both are built entirely *programmatically*, by MATLAB scripts that call Simulink's block-construction API (`add_block`, `add_line`, `set_param`, etc.). Reading those builder scripts (`build_rcflex_analytic.m`, `build_rcflex_control.m`) tells you exactly what's inside each model, block by block and wire by wire — which is what this document does.

## `SensoSpoon_Analytic.slx` — the open-loop physical plant

Built by `src/simulink/build_rcflex_analytic.m`. Structure:

```
Base_motion (From Workspace) ----\
                                   -> Electromechanical_plant (S-function) -> Physical_states_and_loads (To Workspace)
Voltage_A_B (From Workspace) -----/                |
                                                     -> BMI270_pair_and_current_ADC (S-function) -> Sensor_readings (To Workspace)
                                    (Base_motion also feeds into the sensor block, port 2)
```

Block by block:

- **`Base_motion`** — a "From Workspace" source block, reading a `timeseries` variable called `base_motion` out of the model's own workspace (a `Simulink.ModelWorkspace`, an isolated variable scope attached to the model rather than the base MATLAB workspace — this is why `build_rcflex_analytic.m` uses `w = get_param(mdl,'ModelWorkspace'); w.assignin('base_motion', ...)`). This supplies the 15-column "hand disturbance" trajectory described in `01_PHYSICS_AND_MODEL.md`.
- **`Voltage_A_B`** — another From Workspace block, supplying the commanded voltage for the two actuator coils (2 columns). In the analytic (open-loop) model, this voltage is whatever test signal you hand it directly — no controller is involved.
- **`Electromechanical_plant`** — a Level-2 MATLAB S-function block running `rcflex_plant_sfun.m`. This is the plant: it owns the 8 continuous states (`block.NumContStates = 8`), and its `Derivatives` callback calls `rcflex_physics(...)` to get `dx` for Simulink's ODE solver, while its `Outputs` callback calls the same function again to produce the 26-column diagnostic output vector `Y`. Its dialog parameters are `H` (hardware struct), `B` (body properties), `mode` (a string like `'voltage'` or `'off'`, passed straight through to `rcflex_physics`).
- **`Physical_states_and_loads`** — a "To Workspace" sink logging the plant's full 26-column output as a `timeseries` named `analytic_log`.
- **`BMI270_pair_and_current_ADC`** — a second Level-2 S-function, `rcflex_sensor_sfun.m`, running at a fixed 800 Hz sample rate (`b.SampleTimes = [1/800 0]`) rather than continuously — this is a *discrete* block, sampling the continuous plant state at 800 Hz and producing 14 columns (two IMUs' gyro+accel, 6 each, plus 2 quantized current readings). It uses two Dworks (Simulink's term for a persistent memory cell inside a block, short for "discrete work vector" — think of it as the block's private state variable between calls): one holding the last sampled/quantized measurement, one holding an internal low-pass-filtered IMU state.
- **`Sensor_readings`** — To Workspace sink logging the sampled sensor output as `sensor_log`.

The model's solver is configured as `ode15s` (a variable-step implicit solver good for the kind of "stiff" systems that mix fast electrical dynamics with slower mechanical dynamics — appropriate here since the electrical RL time constant is much faster than the mechanical response), with `MaxStep=0.001`, tight relative/absolute tolerances, and `StopTime` set to the requested duration.

An `Simulink.Annotation` block (a text note placed directly on the diagram canvas, purely for a human looking at the model in the Simulink editor) documents the causal chain in a single paragraph.

### Running it: `run_rcflex_analytic.m`

This wrapper function is what you actually call from the Command Window. It:
1. Ensures the 3D geometry assets exist (rebuilding via `generate_rcflex_assets` if the hardware parameters `H` have changed since the cached `assembly.mat` was written).
2. Computes body properties `B`.
3. Builds a default voltage test sequence if you don't supply one — the default is a sequence of common-mode and differential-mode voltage steps (zero, then negative, then positive, then differential in each direction, then zero) explicitly meant to "demonstrate the unregulated plant," per the code comment.
4. Loads the model, injects `H`, `B`, `mode`, `base_motion`, `voltage_command` into the model workspace, sets `StopTime`, and calls `sim(mdl)` — MATLAB's function to actually run a Simulink simulation programmatically (as opposed to clicking Run in the Simulink GUI).
5. Interpolates the logged output onto a uniform time vector, builds a result struct `S`, computes summary stats (peak current, RMS current, whether continuous power rating was exceeded), and saves everything to `output/results/analytic_<mode>.mat`.

## `SensoSpoon_Control.slx` — the closed loop

Built by `src/control/build_rcflex_control.m`, which starts by **copying** `SensoSpoon_Analytic.slx` (`load_system(source); save_system(...)` under the new name) and then modifies it — so it reuses the exact same continuous plant and sensor blocks, rather than reimplementing them. Concretely, it:

1. Deletes the wire from `Voltage_A_B` into the plant, and deletes the `Voltage_A_B` source block entirely — voltage is no longer an external input; it will now be computed internally by the controller.
2. Adds three more S-function blocks:
   - **`Attitude_PID_800Hz`** (`rcflex_controller_sfun.m`) — the outer attitude-control loop, sampled at `C.outer_hz` (800 Hz). Reads the 14-column sensor output, produces two outputs: a 4-column "command" (current references for the two coils, plus estimated back-EMF feedforward terms) and a 16-column "diagnostic" vector for logging.
   - **`Current_bus_ADC_10kHz`** (`rcflex_current_adc_sfun.m`) — samples the plant's current/bus-voltage state at 10 kHz, with its own internal delay/quantization, feeding the current controller.
   - **`Current_PI_10kHz`** (`rcflex_current_sfun.m`) — the inner current-control loop, running at `C.current_hz` (10 kHz), taking the ADC reading and the attitude loop's current command, and producing the actual voltage sent to `Electromechanical_plant`'s voltage input.
   - **`Controller_diagnostics`** — a To Workspace sink logging the attitude controller's diagnostic output as `control_log`.
3. Wires it all together: sensor readings feed the attitude controller; the attitude controller's command feeds the current controller; the current ADC reading also feeds the current controller; the current controller's voltage output feeds back into the plant's voltage input, closing the loop.

So the signal path in the closed-loop model is exactly the loop described in `00_START_HERE.md`: plant state → sensors (800 Hz sampled) → attitude PID (800 Hz) → current command → current PI (10 kHz, using its own faster-sampled current feedback) → voltage → plant.

Two separate hardware parameter structs are used deliberately: `H`/`B` (the "true" plant, potentially with off-nominal parameters for robustness testing) versus `Hnom`/`Bnom` (the "nominal" parameters the *controller* is designed around — i.e. what the controller believes about the hardware). This lets you simulate a controller tuned on nominal assumptions running against a plant with parameter errors, without the controller "cheating" by using the true (possibly different) parameters. `Bnom`/`Hnom` are what get passed as the S-function dialog parameters for the controller blocks; `H`/`B` go to the plant block.

### Running it: `run_rcflex_control.m` (native) vs. `run_rcflex_control_fast.m` (benchmark)

There are **two independent ways to run the closed loop**, and this project deliberately keeps both and cross-checks them (see `verify_rcflex_control_numerics.m` in `07_TESTS.md`):

- **`run_rcflex_control.m`** — runs the actual `SensoSpoon_Control.slx` Simulink model via `sim(mdl)`, with the continuous plant integrated by `ode23t` (an implicit solver suited to mildly stiff systems). This is the authoritative, slower reference implementation.
- **`run_rcflex_control_fast.m`** — a hand-written MATLAB loop that reimplements the *same* control logic (calling the exact same `rcflex_attitude_step`, `rcflex_current_step`, `rcflex_imu_sample`, `rcflex_current_sample`, `rcflex_physics` functions that the S-functions call) but integrates the mechanical dynamics with a much simpler fixed-step scheme (a "held acceleration" multirate approach: acceleration is only recomputed every `mechanicalStep` seconds, other loops run at their own fixed rates, and a closed-form exact solution is used for the RL circuit at each electrical step rather than an ODE solver). This runs far faster than driving full Simulink, which matters because the benchmark/tuning/screening scripts (`benchmark_rcflex_control.m`, `tune_rcflex_control.m`, `screen_rcflex_strong_soup.m`) need to run dozens of scenarios quickly. The code comments are explicit that this is "not a Simulink result" and must be validated against the native model.

Both paths call the identical control-law functions in `src/control/`, so the "controller" itself is never duplicated — only the plant-integration method differs between the two run paths.
