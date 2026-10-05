# Closed-loop validation results

Fixed five-notch intent filter (4/6/8/10/12 Hz, bandwidth parameter 7 Hz), bounded phase lead and 10 Hz rolloff; attitude bandwidth 35 Hz; 800 Hz outer and 10 kHz inner loop.

Not all scenarios meet the target: movement tracking, larger amplitudes and out-of-band disturbance remain limitations. Inspect failed cases, not only nominal single-frequency results.

Attitude scoring includes intention tracking error. Baseline: rigidly attached spoon under identical base input. Steady window: t >= 1.5 s; no scoring filter.

## Headline, by arm

`mean_feasible` is the headline: it excludes cases the hardware could not serve under ANY controller, judged by an inverse-dynamics bound that knows nothing about the controller (`rcflex_case_feasible.m`). All three means are shown so the exclusion is visible rather than silent. Case 10 carries no tremor, so its suppression is undefined and it is scored by attitude RMS only.

| Arm | sEMG fusion | Mean suppression, all scored (%) | Mean suppression, feasible cases (%) | Worst feasible case (%) | Case 10 voluntary-motion RMS (deg) | Cases meeting 90% |
|---|---|---:|---:|---:|---:|---:|
| simple_pid | off | 64.19 | 90.23 | 51.51 | 0.4583 | 15 of 16 |
| ekf_cv | off | 71.03 | 92.12 | 85.88 | 0.2008 | 11 of 16 |
| ekf_cv_legacy | off | 68.75 | 88.96 | 30.75 | 1.2390 | 11 of 16 |
| ekf_cv_osc2 | off | 71.04 | 92.13 | 85.90 | 0.2008 | 11 of 16 |
| ekf_cv_qfix | intent_gate_fixed | 65.62 | 85.18 | -41.41 | 3.0418 | 15 of 16 |
| ekf_cv_semg | intent_gate | 72.63 | 93.04 | 85.31 | 0.2070 | 15 of 16 |

## Every case, every arm

| Arm | Case | Scenario | Hz | Input amplitude/axis (deg) | Attitude RMS (deg) | Reduction (%) | Position RMS (mm) | Peak current (A) | Constraints | Inside hardware envelope |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| simple_pid | 1 | sine | 4 | 1.0 | 0.015 | 98.5 | 0.81 | 0.57 | 1 | 1 |
| simple_pid | 2 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.83 | 0.69 | 1 | 1 |
| simple_pid | 3 | sine | 12 | 1.0 | 0.087 | 91.3 | 0.87 | 0.90 | 1 | 1 |
| simple_pid | 4 | sine | 6 | 3.0 | 0.234 | 92.2 | 2.46 | 0.84 | 1 | 1 |
| simple_pid | 5 | sine | 10 | 3.0 | 0.322 | 89.3 | 2.57 | 1.33 | 1 | 0 |
| simple_pid | 6 | mixed | 8 | 2.0 | 0.106 | 92.4 | 1.14 | 0.80 | 1 | 1 |
| simple_pid | 7 | chirp | 8 | 2.0 | 0.183 | 90.9 | 1.72 | 1.26 | 1 | 1 |
| simple_pid | 8 | irregular | 8 | 2.0 | 0.105 | 92.5 | 1.63 | 0.85 | 1 | 1 |
| simple_pid | 9 | combined | 8 | 1.0 | 0.485 | 51.5 | 1.46 | 0.81 | 1 | 1 |
| simple_pid | 10 | intent | 8 | 0.0 | 0.458 | NaN | 0.37 | 0.57 | 1 | 1 |
| simple_pid | 11 | sine | 8 | 1.0 | 0.073 | 92.7 | 0.83 | 0.63 | 1 | 1 |
| simple_pid | 12 | sine | 8 | 1.0 | 0.065 | 93.5 | 0.85 | 0.89 | 1 | 1 |
| simple_pid | 13 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.83 | 0.69 | 1 | 1 |
| simple_pid | 14 | step | 8 | 5.0 | 19.652 | -116.6 | 21.48 | 1.44 | 0 | 0 |
| simple_pid | 15 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.84 | 0.82 | 1 | 1 |
| simple_pid | 16 | sine | 5 | 1.0 | 0.086 | 91.4 | 0.78 | 0.59 | 1 | 1 |
| simple_pid | 17 | sine | 7 | 1.0 | 0.072 | 92.8 | 0.84 | 0.65 | 1 | 1 |
| simple_pid | 18 | sine | 9 | 1.0 | 0.085 | 91.5 | 0.87 | 0.74 | 1 | 1 |
| simple_pid | 19 | sine | 11 | 1.0 | 0.094 | 90.6 | 0.88 | 0.85 | 1 | 1 |
| simple_pid | 20 | sine | 20 | 1.0 | 2.967 | -196.7 | 3.26 | 1.35 | 0 | 0 |
| ekf_cv | 1 | sine | 4 | 1.0 | 0.025 | 97.5 | 0.80 | 0.57 | 1 | 1 |
| ekf_cv | 2 | sine | 8 | 1.0 | 0.062 | 93.8 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv | 3 | sine | 12 | 1.0 | 0.102 | 89.8 | 0.89 | 0.94 | 1 | 1 |
| ekf_cv | 4 | sine | 6 | 3.0 | 0.237 | 92.1 | 2.46 | 0.84 | 1 | 1 |
| ekf_cv | 5 | sine | 10 | 3.0 | 0.388 | 87.1 | 2.64 | 1.33 | 1 | 0 |
| ekf_cv | 6 | mixed | 8 | 2.0 | 0.167 | 88.0 | 1.16 | 0.81 | 1 | 1 |
| ekf_cv | 7 | chirp | 8 | 2.0 | 0.229 | 88.6 | 1.76 | 1.28 | 1 | 1 |
| ekf_cv | 8 | irregular | 8 | 2.0 | 0.170 | 87.8 | 1.65 | 0.88 | 1 | 1 |
| ekf_cv | 9 | combined | 8 | 1.0 | 0.141 | 85.9 | 1.40 | 0.82 | 1 | 1 |
| ekf_cv | 10 | intent | 8 | 0.0 | 0.201 | NaN | 0.17 | 0.59 | 1 | 1 |
| ekf_cv | 11 | sine | 8 | 1.0 | 0.075 | 92.5 | 0.83 | 0.63 | 1 | 1 |
| ekf_cv | 12 | sine | 8 | 1.0 | 0.066 | 93.4 | 0.85 | 0.90 | 1 | 1 |
| ekf_cv | 13 | sine | 8 | 1.0 | 0.063 | 93.7 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv | 14 | step | 8 | 5.0 | 19.900 | -119.3 | 21.69 | 1.43 | 0 | 0 |
| ekf_cv | 15 | sine | 8 | 1.0 | 0.064 | 93.6 | 0.85 | 0.83 | 1 | 1 |
| ekf_cv | 16 | sine | 5 | 1.0 | 0.017 | 98.3 | 0.81 | 0.60 | 1 | 1 |
| ekf_cv | 17 | sine | 7 | 1.0 | 0.052 | 94.8 | 0.83 | 0.65 | 1 | 1 |
| ekf_cv | 18 | sine | 9 | 1.0 | 0.070 | 93.0 | 0.85 | 0.74 | 1 | 1 |
| ekf_cv | 19 | sine | 11 | 1.0 | 0.087 | 91.3 | 0.87 | 0.86 | 1 | 1 |
| ekf_cv | 20 | sine | 20 | 1.0 | 1.921 | -92.1 | 2.08 | 1.35 | 0 | 0 |
| ekf_cv_legacy | 1 | sine | 4 | 1.0 | 0.027 | 97.3 | 0.81 | 0.57 | 1 | 1 |
| ekf_cv_legacy | 2 | sine | 8 | 1.0 | 0.062 | 93.8 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_legacy | 3 | sine | 12 | 1.0 | 0.100 | 90.0 | 0.89 | 0.94 | 1 | 1 |
| ekf_cv_legacy | 4 | sine | 6 | 3.0 | 0.235 | 92.2 | 2.46 | 0.84 | 1 | 1 |
| ekf_cv_legacy | 5 | sine | 10 | 3.0 | 0.389 | 87.0 | 2.64 | 1.33 | 1 | 0 |
| ekf_cv_legacy | 6 | mixed | 8 | 2.0 | 0.141 | 89.8 | 1.16 | 0.81 | 1 | 1 |
| ekf_cv_legacy | 7 | chirp | 8 | 2.0 | 0.224 | 88.8 | 1.75 | 1.28 | 1 | 1 |
| ekf_cv_legacy | 8 | irregular | 8 | 2.0 | 0.145 | 89.6 | 1.65 | 0.88 | 1 | 1 |
| ekf_cv_legacy | 9 | combined | 8 | 1.0 | 0.692 | 30.7 | 1.53 | 1.33 | 1 | 1 |
| ekf_cv_legacy | 10 | intent | 8 | 0.0 | 1.239 | NaN | 0.89 | 0.88 | 1 | 1 |
| ekf_cv_legacy | 11 | sine | 8 | 1.0 | 0.074 | 92.6 | 0.83 | 0.63 | 1 | 1 |
| ekf_cv_legacy | 12 | sine | 8 | 1.0 | 0.066 | 93.4 | 0.85 | 0.89 | 1 | 1 |
| ekf_cv_legacy | 13 | sine | 8 | 1.0 | 0.062 | 93.8 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_legacy | 14 | step | 8 | 5.0 | 19.879 | -119.1 | 21.68 | 1.43 | 0 | 0 |
| ekf_cv_legacy | 15 | sine | 8 | 1.0 | 0.063 | 93.7 | 0.84 | 0.83 | 1 | 1 |
| ekf_cv_legacy | 16 | sine | 5 | 1.0 | 0.017 | 98.3 | 0.81 | 0.60 | 1 | 1 |
| ekf_cv_legacy | 17 | sine | 7 | 1.0 | 0.052 | 94.8 | 0.83 | 0.65 | 1 | 1 |
| ekf_cv_legacy | 18 | sine | 9 | 1.0 | 0.069 | 93.1 | 0.85 | 0.74 | 1 | 1 |
| ekf_cv_legacy | 19 | sine | 11 | 1.0 | 0.086 | 91.4 | 0.87 | 0.86 | 1 | 1 |
| ekf_cv_legacy | 20 | sine | 20 | 1.0 | 1.850 | -85.0 | 1.99 | 1.35 | 0 | 0 |
| ekf_cv_osc2 | 1 | sine | 4 | 1.0 | 0.025 | 97.5 | 0.80 | 0.57 | 1 | 1 |
| ekf_cv_osc2 | 2 | sine | 8 | 1.0 | 0.062 | 93.8 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_osc2 | 3 | sine | 12 | 1.0 | 0.102 | 89.8 | 0.89 | 0.94 | 1 | 1 |
| ekf_cv_osc2 | 4 | sine | 6 | 3.0 | 0.237 | 92.1 | 2.46 | 0.84 | 1 | 1 |
| ekf_cv_osc2 | 5 | sine | 10 | 3.0 | 0.388 | 87.1 | 2.64 | 1.33 | 1 | 0 |
| ekf_cv_osc2 | 6 | mixed | 8 | 2.0 | 0.167 | 88.0 | 1.16 | 0.81 | 1 | 1 |
| ekf_cv_osc2 | 7 | chirp | 8 | 2.0 | 0.229 | 88.6 | 1.76 | 1.28 | 1 | 1 |
| ekf_cv_osc2 | 8 | irregular | 8 | 2.0 | 0.170 | 87.8 | 1.65 | 0.88 | 1 | 1 |
| ekf_cv_osc2 | 9 | combined | 8 | 1.0 | 0.141 | 85.9 | 1.40 | 0.82 | 1 | 1 |
| ekf_cv_osc2 | 10 | intent | 8 | 0.0 | 0.201 | NaN | 0.17 | 0.58 | 1 | 1 |
| ekf_cv_osc2 | 11 | sine | 8 | 1.0 | 0.075 | 92.5 | 0.83 | 0.63 | 1 | 1 |
| ekf_cv_osc2 | 12 | sine | 8 | 1.0 | 0.067 | 93.3 | 0.85 | 0.90 | 1 | 1 |
| ekf_cv_osc2 | 13 | sine | 8 | 1.0 | 0.063 | 93.7 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_osc2 | 14 | step | 8 | 5.0 | 19.843 | -118.7 | 21.69 | 1.43 | 0 | 0 |
| ekf_cv_osc2 | 15 | sine | 8 | 1.0 | 0.064 | 93.6 | 0.85 | 0.83 | 1 | 1 |
| ekf_cv_osc2 | 16 | sine | 5 | 1.0 | 0.017 | 98.3 | 0.81 | 0.60 | 1 | 1 |
| ekf_cv_osc2 | 17 | sine | 7 | 1.0 | 0.052 | 94.8 | 0.83 | 0.65 | 1 | 1 |
| ekf_cv_osc2 | 18 | sine | 9 | 1.0 | 0.070 | 93.0 | 0.85 | 0.74 | 1 | 1 |
| ekf_cv_osc2 | 19 | sine | 11 | 1.0 | 0.087 | 91.3 | 0.87 | 0.86 | 1 | 1 |
| ekf_cv_osc2 | 20 | sine | 20 | 1.0 | 1.926 | -92.6 | 2.07 | 1.35 | 0 | 0 |
| ekf_cv_qfix | 1 | sine | 4 | 1.0 | 0.016 | 98.4 | 0.81 | 0.57 | 1 | 1 |
| ekf_cv_qfix | 2 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.84 | 0.69 | 1 | 1 |
| ekf_cv_qfix | 3 | sine | 12 | 1.0 | 0.090 | 91.0 | 0.88 | 0.92 | 1 | 1 |
| ekf_cv_qfix | 4 | sine | 6 | 3.0 | 0.235 | 92.2 | 2.46 | 0.84 | 1 | 1 |
| ekf_cv_qfix | 5 | sine | 10 | 3.0 | 0.389 | 87.0 | 2.64 | 1.33 | 1 | 0 |
| ekf_cv_qfix | 6 | mixed | 8 | 2.0 | 0.099 | 92.9 | 1.15 | 0.80 | 1 | 1 |
| ekf_cv_qfix | 7 | chirp | 8 | 2.0 | 0.179 | 91.0 | 1.72 | 1.26 | 1 | 1 |
| ekf_cv_qfix | 8 | irregular | 8 | 2.0 | 0.101 | 92.8 | 1.63 | 0.86 | 1 | 1 |
| ekf_cv_qfix | 9 | combined | 8 | 1.0 | 1.414 | -41.4 | 1.82 | 0.98 | 1 | 1 |
| ekf_cv_qfix | 10 | intent | 8 | 0.0 | 3.042 | NaN | 2.36 | 0.61 | 1 | 1 |
| ekf_cv_qfix | 11 | sine | 8 | 1.0 | 0.074 | 92.6 | 0.83 | 0.63 | 1 | 1 |
| ekf_cv_qfix | 12 | sine | 8 | 1.0 | 0.066 | 93.4 | 0.85 | 0.89 | 1 | 1 |
| ekf_cv_qfix | 13 | sine | 8 | 1.0 | 0.061 | 93.9 | 0.84 | 0.69 | 1 | 1 |
| ekf_cv_qfix | 14 | step | 8 | 5.0 | 19.565 | -115.7 | 21.50 | 1.43 | 0 | 0 |
| ekf_cv_qfix | 15 | sine | 8 | 1.0 | 0.061 | 93.9 | 0.84 | 0.82 | 1 | 1 |
| ekf_cv_qfix | 16 | sine | 5 | 1.0 | 0.016 | 98.4 | 0.81 | 0.59 | 1 | 1 |
| ekf_cv_qfix | 17 | sine | 7 | 1.0 | 0.052 | 94.8 | 0.83 | 0.65 | 1 | 1 |
| ekf_cv_qfix | 18 | sine | 9 | 1.0 | 0.068 | 93.2 | 0.84 | 0.74 | 1 | 1 |
| ekf_cv_qfix | 19 | sine | 11 | 1.0 | 0.081 | 91.9 | 0.87 | 0.84 | 1 | 1 |
| ekf_cv_qfix | 20 | sine | 20 | 1.0 | 1.875 | -87.5 | 2.00 | 1.35 | 0 | 0 |
| ekf_cv_semg | 1 | sine | 4 | 1.0 | 0.017 | 98.3 | 0.81 | 0.57 | 1 | 1 |
| ekf_cv_semg | 2 | sine | 8 | 1.0 | 0.061 | 93.9 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_semg | 3 | sine | 12 | 1.0 | 0.091 | 90.9 | 0.88 | 0.94 | 1 | 1 |
| ekf_cv_semg | 4 | sine | 6 | 3.0 | 0.236 | 92.1 | 2.46 | 0.84 | 1 | 1 |
| ekf_cv_semg | 5 | sine | 10 | 3.0 | 0.388 | 87.1 | 2.64 | 1.33 | 1 | 0 |
| ekf_cv_semg | 6 | mixed | 8 | 2.0 | 0.103 | 92.6 | 1.15 | 0.81 | 1 | 1 |
| ekf_cv_semg | 7 | chirp | 8 | 2.0 | 0.188 | 90.6 | 1.73 | 1.26 | 1 | 1 |
| ekf_cv_semg | 8 | irregular | 8 | 2.0 | 0.100 | 92.8 | 1.63 | 0.88 | 1 | 1 |
| ekf_cv_semg | 9 | combined | 8 | 1.0 | 0.147 | 85.3 | 1.40 | 0.82 | 1 | 1 |
| ekf_cv_semg | 10 | intent | 8 | 0.0 | 0.207 | NaN | 0.17 | 0.58 | 1 | 1 |
| ekf_cv_semg | 11 | sine | 8 | 1.0 | 0.074 | 92.6 | 0.83 | 0.63 | 1 | 1 |
| ekf_cv_semg | 12 | sine | 8 | 1.0 | 0.066 | 93.4 | 0.85 | 0.90 | 1 | 1 |
| ekf_cv_semg | 13 | sine | 8 | 1.0 | 0.061 | 93.9 | 0.84 | 0.70 | 1 | 1 |
| ekf_cv_semg | 14 | step | 8 | 5.0 | 19.769 | -117.9 | 21.63 | 1.43 | 0 | 0 |
| ekf_cv_semg | 15 | sine | 8 | 1.0 | 0.061 | 93.9 | 0.84 | 0.83 | 1 | 1 |
| ekf_cv_semg | 16 | sine | 5 | 1.0 | 0.016 | 98.4 | 0.81 | 0.59 | 1 | 1 |
| ekf_cv_semg | 17 | sine | 7 | 1.0 | 0.051 | 94.9 | 0.83 | 0.65 | 1 | 1 |
| ekf_cv_semg | 18 | sine | 9 | 1.0 | 0.069 | 93.1 | 0.84 | 0.74 | 1 | 1 |
| ekf_cv_semg | 19 | sine | 11 | 1.0 | 0.081 | 91.9 | 0.87 | 0.86 | 1 | 1 |
| ekf_cv_semg | 20 | sine | 20 | 1.0 | 1.779 | -78.0 | 1.89 | 1.35 | 0 | 0 |

The final column is the independent hardware-envelope flag. Cases 5, 14 and 20 fail it: the inverse-dynamics demand exceeds the bus voltage (cases 5 and 14) or the current, voltage and continuous copper power together (case 20). A suppression figure on those cases measures the actuator, not the algorithm.

Cases 11/12: 0/15 g payload. Case 13: 5.8 V bus. Case 14: amplitude doubles after 2 s. Case 15: +20% resistance, -15% force constant, double guide friction. Case 10: intentional motion only, so reduction is undefined. Case 9 combines intentional motion and tremor. Cases 16-19 test 5/7/9/11 Hz between notch centers; case 20 is a 20 Hz out-of-band test. Cases 14 and 20 use a finer 0.1 ms mechanical step for boundary checks; other cases use 0.5 ms.

Fast versus native Simulink: nominal 8 Hz, 1 deg/axis, 2 s; default-step joint difference RMS 0.00250 deg; reduction difference 0.074 percentage points. This validates one nominal case, not every boundary case.

Native continuous reference (8 Hz, 1 deg/axis, 5 g, seed 1701, scoring 1.5-2.0 s): attitude RMS 0.0339 deg, reduction 96.61 percent, peak current 0.681 A, peak stroke 0.300 mm. The independent suite above uses longer runs and different noise seeds.

Constraints column checks observed joint angle, stroke, current, estimated side load and average copper power. Short-run passes do not certify long-term temperature, fatigue or robustness. Controller assumes a 2-second stationary level-fixture calibration before startup.

These are simulated results with nominal/custom and partly unmeasured parameters, not hardware experimental results. Two controlled axes cannot reject arbitrary translational and rotational disturbance simultaneously; 100% rejection of arbitrary tremor is not achieved.
