# Closed-Loop Control and Validation

## Running

```matlab
START_PROJECT('test')           % Physical model and controller checks
S = START_PROJECT('control')    % Native Simulink closed loop, plots, and animation
T = START_PROJECT('benchmark')  % Independent test set using the faster multirate integrator
```

The closed-loop model is `models/SensoSpoon_Control.slx`. `SensoSpoon_Analytic.slx` remains available for voltage excitation and physical-model validation. Both reuse the same continuous electrical, mechanical, and thermal plant rather than idealized replacement actuators.

## Control architecture

1. Two IMUs sample at 800 Hz, including noise, fixed bias, a 100 Hz single-pole internal filter, quantization, and one-sample delay. Complementary attitude estimation relies primarily on gyroscopes with slow accelerometer correction. Large linear accelerations can contaminate the gravity estimate.
2. Handle attitude passes through a fixed IIR chain to estimate intent: five notches at 4, 6, 8, 10, and 12 Hz, each with a 7 Hz bandwidth parameter, followed by bounded phase-lead compensation and a second-order 10 Hz low-pass filter. Coefficients are computed before startup; runtime uses fixed recurrences without online frequency searches. The estimator never reads the test input's true intent, future data, or zero-phase filtering results.
3. Attitude PID uses spoon attitude error and gyroscope angular velocity, with nominal mass-matrix, gravity, flexure, and guide-friction compensation. The two-axis transmission Jacobian allocates torque to the two coils. A two-variable bounded least-squares allocator enumerates nine constraint states without requiring Optimization Toolbox.
4. The 10 kHz current PI loop uses independent current and bus-voltage ADC measurements with one-sample delay, quantization, noise, and current offset. Resistance and back-EMF feedforward reduce the PI workload. Voltage saturation uses back-calculation anti-windup.
5. Commanded voltage drives the RL, back-EMF, stroke-dependent force constant, continuous mechanical dynamics, and thermal model. Animations display solved motion only.

Parameters are centralized in `src/control/rcflex_control_parameters.m`. Relative-angle references are limited to ±14°, with physical compliant stops at ±15°. Reference limiting does not prevent actual overshoot; tests also check actual angles, stroke, and shaft side loads.

## Hardware utilization and boundaries

- The existing mechanism and two GVCM-016-019-01M voice coils are retained. Force constants and moving masses were not altered to improve results.
- The driver design limits current to 1.5 A, with 2% command margin. The manufacturer's 4.6 N rating is a peak force, not a continuous output.
- The allocator uses both coils' combined torque, limits estimated shaft side load at the current linkage geometry, and derates continuous current using estimated copper loss and thermal state.
- Thermal capacity, heat dissipation, friction, flexure nonlinearity, force-constant curves, and sensor-noise scenarios still include unmeasured parameters. Short-run average power and temperature rise do not establish long-term thermal safety or fatigue life.
- Reports include actual current/voltage peaks, RMS values, stroke, average copper loss, side load, and saturation fraction. Unused current capacity alone does not justify higher gains when dynamic errors remain.

## Calibration and test definitions

Startup assumes **two seconds of dual-IMU calibration in a stationary, level fixture**. Calibration averages noisy samples rather than reading model bias constants directly. Simulation starts from a known level attitude; this is not an arbitrary-orientation startup method. Hardware requires corresponding calibration, sensor-alignment identification, and current-loop identification.

Training compares the declared low-pass cutoff frequencies and attitude bandwidths. After parameters are fixed, independent noise seeds are used for tests covering 4/8/12 Hz, intermediate 5/7/9/11 Hz frequencies, an out-of-band 20 Hz case, larger angles, multitone and chirp inputs, translation, voluntary motion, payload changes, low voltage, and parameter errors.

Suppression is defined as `100 × (1 − controlled attitude-error RMS / rigidly connected spoon attitude-error RMS under the same input)`. RMS uses the three-dimensional rotation error relative to true intent without prefiltering unfavorable errors. The baseline has locked joints, rather than a freely drooping gimbal or shorted actuator. Steady-state scoring starts at 1.5 seconds; startup RMS and full-run peaks are reported separately.

The voluntary-motion-only baseline has zero error, so suppression is NaN and tracking error is reported directly. Attitude stability and spoon-position stability are assessed separately. A two-DOF mechanism cannot simultaneously cancel arbitrary three-dimensional translation and rotation, and cannot guarantee 100% motion cancellation under every tremor. Slow voluntary motion and low-frequency tremor also overlap spectrally and cannot be perfectly separated using only two IMUs.

## Numerical validation

`run_rcflex_control_fast` uses a 50 microsecond electrical step, independent multirate sampling, and a default 0.5 ms mechanical-acceleration update for tuning and batch tests. It is not a Simulink result. Compare it with smaller mechanical steps and the continuous implicit-trapezoidal `ode23t` solution from `run_rcflex_control`. Reports identify the solver; fast-integrator results are not physical-prototype validation.

Independent tests follow final selection of fixed notch/phase compensation and a 35 Hz attitude bandwidth. The original second-order 1 Hz low-pass filter remains available as a reproducible baseline (`C.intent_filter='lowpass'`); `notch` is the default. Voluntary-motion tracking errors, saturation under large disturbances, and out-of-band failures must remain visible. Single-frequency results do not establish universal effectiveness.

The intended frequency band is 4–12 Hz. Spectral overlap, IMU delay, and actuator saturation remain limiting factors.

Primary results are in `output/results/control_training.csv`, `control_benchmark.csv`, and `control_native.mat`. The response plot is `output/figures/control_response.png`.

## References

- [MathWorks: Back-calculation and clamping anti-windup](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
- [Moticont: GVCM-016-019 force and stroke specifications](https://www.moticont.com/GVCM-016-019-01.htm)

Nominal controller parameters do not constitute completed MCU deployment. These MATLAB functions currently support analytical simulation; 800 Hz is the simulated sampling rate, and embedded execution-time compliance has not been demonstrated.
