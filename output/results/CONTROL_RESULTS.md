# Closed-loop validation results

Fixed five-notch intent filter (4/6/8/10/12 Hz, bandwidth parameter 7 Hz), bounded phase lead and 10 Hz rolloff; attitude bandwidth 35 Hz; 800 Hz outer and 10 kHz inner loop.

Not all scenarios meet the target: movement tracking, larger amplitudes and out-of-band disturbance remain limitations. Inspect failed cases, not only nominal single-frequency results.

Attitude scoring includes intention tracking error. Baseline: rigidly attached spoon under identical base input. Steady window: t >= 1.5 s; no scoring filter.

| Case | Scenario | Hz | Input amplitude/axis (deg) | Attitude RMS (deg) | Reduction (%) | Position RMS (mm) | Peak current (A) | Constraints |
|---|---|---:|---:|---:|---:|---:|---:|---|
| 1 | sine | 4 | 1.0 | 0.015 | 98.5 | 0.81 | 0.57 | 1 |
| 2 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.83 | 0.69 | 1 |
| 3 | sine | 12 | 1.0 | 0.087 | 91.3 | 0.87 | 0.90 | 1 |
| 4 | sine | 6 | 3.0 | 0.234 | 92.2 | 2.46 | 0.84 | 1 |
| 5 | sine | 10 | 3.0 | 0.322 | 89.3 | 2.57 | 1.33 | 1 |
| 6 | mixed | 8 | 2.0 | 0.106 | 92.4 | 1.14 | 0.80 | 1 |
| 7 | chirp | 8 | 2.0 | 0.183 | 90.9 | 1.72 | 1.26 | 1 |
| 8 | irregular | 8 | 2.0 | 0.105 | 92.5 | 1.63 | 0.85 | 1 |
| 9 | combined | 8 | 1.0 | 0.485 | 51.5 | 1.46 | 0.81 | 1 |
| 10 | intent | 8 | 0.0 | 0.458 | NaN | 0.37 | 0.57 | 1 |
| 11 | sine | 8 | 1.0 | 0.073 | 92.7 | 0.83 | 0.63 | 1 |
| 12 | sine | 8 | 1.0 | 0.065 | 93.5 | 0.85 | 0.89 | 1 |
| 13 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.83 | 0.69 | 1 |
| 14 | step | 8 | 5.0 | 19.652 | -116.6 | 21.48 | 1.44 | 0 |
| 15 | sine | 8 | 1.0 | 0.060 | 94.0 | 0.84 | 0.82 | 1 |
| 16 | sine | 5 | 1.0 | 0.086 | 91.4 | 0.78 | 0.59 | 1 |
| 17 | sine | 7 | 1.0 | 0.072 | 92.8 | 0.84 | 0.65 | 1 |
| 18 | sine | 9 | 1.0 | 0.085 | 91.5 | 0.87 | 0.74 | 1 |
| 19 | sine | 11 | 1.0 | 0.094 | 90.6 | 0.88 | 0.85 | 1 |
| 20 | sine | 20 | 1.0 | 2.967 | -196.7 | 3.26 | 1.35 | 0 |

Cases 11/12: 0/15 g payload. Case 13: 5.8 V bus. Case 14: amplitude doubles after 2 s. Case 15: +20% resistance, -15% force constant, double guide friction. Case 10: intentional motion only, so reduction is undefined. Case 9 combines intentional motion and tremor. Cases 16-19 test 5/7/9/11 Hz between notch centers; case 20 is a 20 Hz out-of-band test. Cases 14 and 20 use a finer 0.1 ms mechanical step for boundary checks; other cases use 0.5 ms.

Fast versus native Simulink: nominal 8 Hz, 1 deg/axis, 2 s; default-step joint difference RMS 0.00250 deg; reduction difference 0.074 percentage points. This validates one nominal case, not every boundary case.

Native continuous reference (8 Hz, 1 deg/axis, 5 g, seed 1701, scoring 1.5-2.0 s): attitude RMS 0.0339 deg, reduction 96.61 percent, peak current 0.681 A, peak stroke 0.300 mm. The independent suite above uses longer runs and different noise seeds.

Constraints column checks observed joint angle, stroke, current, estimated side load and average copper power. Short-run passes do not certify long-term temperature, fatigue or robustness. Controller assumes a 2-second stationary level-fixture calibration before startup.

These are simulated results with nominal/custom and partly unmeasured parameters, not hardware experimental results. Two controlled axes cannot reject arbitrary translational and rotational disturbance simultaneously; 100% rejection of arbitrary tremor is not achieved.
