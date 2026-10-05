# Recorded human tremor: replay results (E4)

Tremor input taken from PhysioNet `tremordb` (Beuter, Titcombe & Glass), every
stimulation-off, medication-off record: 60 recordings from 14 patients. The recordings
supply waveform SHAPE only and are RMS-matched to a unit sine, because fingertip
translation cannot be converted into hand attitude without inventing a moment arm.
Amplitude, duration, solver step, sensor seed and metrics window are identical to the
synthetic benchmark.

**This is evidence about the control algorithm meeting a real waveform. It is not
evidence about human physiology, and it says nothing about the sEMG stream, which is
synthetic throughout and is generated from the same ground truth as before.**

## The input, against the synthetic model it replaces

| | Synthetic sine | Recorded tremor |
|---|---:|---:|
| Crest factor, median (range) | 1.414 | 2.77 (1.74-6.26) |
| Dominant frequency, median (range) Hz | as commanded | 4.48 (2.41-8.41) |
| Records inside the declared 4-20 Hz envelope | - | 46 of 60 |

## Headline, by arm

| Arm | Mean, all 60 records (%) | Mean, in 4-20 Hz band (%) | Worst in band (%) | Meets 90% | Attitude RMS mean (deg) |
|---|---:|---:|---:|---:|---:|
| `simple_pid` | 79.10 | 81.62 | 52.65 | 8 of 46 | 0.2080 |
| `ekf_cv` | 69.28 | 73.16 | 29.58 | 8 of 46 | 0.3073 |
| `ekf_cv_semg` | 85.36 | 86.81 | 62.62 | 21 of 46 | 0.1464 |

## Paired against a synthetic sine at the same frequency

Each row holds the record, the seed, the amplitude and the dominant frequency fixed
and changes only the waveform, so the delta is attributable to shape alone.

| Arm | Synthetic (%) | Recorded (%) | Change (points) | Worst change | Records where recorded is worse |
|---|---:|---:|---:|---:|---:|
| `simple_pid` | 80.94 | 81.07 | +0.13 | -23.22 | 10 of 14 |
| `ekf_cv` | 95.50 | 71.71 | -23.79 | -41.29 | 14 of 14 |
| `ekf_cv_semg` | 96.11 | 86.57 | -9.54 | -22.94 | 14 of 14 |

## Every record

| Record | Subject | Group | Dominant Hz | Crest | In band | simple_pid (%) | ekf_cv (%) | ekf_cv_semg (%) |
|---|---|---|---:|---:|---|---:|---:|---:|
| g2rof.rit | g2 | high | 5.92 | 1.77 | 1 | 93.05 | 92.65 | 94.48 |
| g2r30of.rit | g2 | high | 5.93 | 2.65 | 1 | 89.48 | 86.02 | 90.36 |
| g2r45of.rit | g2 | high | 6.00 | 3.74 | 1 | 90.64 | 83.30 | 91.94 |
| g2r60of.rit | g2 | high | 6.10 | 2.46 | 1 | 93.95 | 91.65 | 94.72 |
| g2r15of.rit | g2 | high | 6.16 | 1.76 | 1 | 92.34 | 91.55 | 93.01 |
| s6r45of.let | s6 | high | 4.75 | 2.50 | 1 | 90.47 | 88.84 | 94.84 |
| s6rof.let | s6 | high | 4.81 | 2.19 | 1 | 91.97 | 91.71 | 96.12 |
| s6r15of.let | s6 | high | 4.86 | 1.74 | 1 | 88.96 | 91.38 | 96.24 |
| s6r60of.let | s6 | high | 5.13 | 2.37 | 1 | 88.54 | 83.37 | 91.91 |
| s6r30of.let | s6 | high | 5.65 | 2.15 | 1 | 90.09 | 87.38 | 93.91 |
| s7r30of.rit | s7 | high | 4.10 | 1.98 | 1 | 87.56 | 87.37 | 93.61 |
| s7rof.rit | s7 | high | 4.26 | 2.23 | 1 | 87.18 | 88.47 | 94.57 |
| s7r60of.rit | s7 | high | 4.37 | 1.91 | 1 | 82.61 | 84.41 | 92.82 |
| s8r15of.let | s8 | high | 4.44 | 1.80 | 1 | 89.80 | 91.03 | 93.01 |
| s8r60of.let | s8 | high | 4.46 | 1.91 | 1 | 89.36 | 88.45 | 91.41 |
| s8r45of.let | s8 | high | 4.46 | 1.88 | 1 | 89.24 | 91.28 | 92.84 |
| s8rof.let | s8 | high | 4.54 | 2.34 | 1 | 86.69 | 82.74 | 90.81 |
| s8r30of.let | s8 | high | 4.57 | 1.75 | 1 | 90.62 | 93.46 | 95.43 |
| v4rof.rit | v4 | high | 3.72 | 2.22 | 0 | 90.56 | 86.15 | 93.47 |
| v5rof.let | v5 | high | 7.84 | 2.69 | 1 | 77.05 | 65.18 | 79.57 |
| g10r15of.let | g10 | low | 4.44 | 3.14 | 1 | 71.37 | 59.53 | 82.07 |
| g10r45of.let | g10 | low | 4.49 | 2.54 | 1 | 71.68 | 65.77 | 85.01 |
| g10r30of.let | g10 | low | 5.95 | 4.07 | 1 | 79.86 | 68.41 | 86.42 |
| g10rof.let | g10 | low | 5.95 | 2.73 | 1 | 81.28 | 73.79 | 87.69 |
| g10r60of.let | g10 | low | 6.04 | 2.64 | 1 | 76.73 | 65.38 | 83.27 |
| g11r30of.let | g11 | low | 2.41 | 3.52 | 0 | 71.31 | 58.51 | 82.60 |
| g11r45of.let | g11 | low | 2.42 | 3.45 | 0 | 71.47 | 61.73 | 81.03 |
| g11r60of.let | g11 | low | 2.63 | 3.36 | 0 | 59.71 | 44.72 | 77.76 |
| g11rof.let | g11 | low | 2.82 | 2.91 | 0 | 73.47 | 60.85 | 82.28 |
| g11r15of.let | g11 | low | 4.43 | 2.58 | 1 | 72.17 | 60.32 | 85.28 |
| g12rof.rit | g12 | low | 2.41 | 4.16 | 0 | 66.46 | 47.75 | 79.61 |
| g12r45of.rit | g12 | low | 2.41 | 3.84 | 0 | 70.74 | 60.05 | 80.76 |
| g12r30of.rit | g12 | low | 2.74 | 3.07 | 0 | 66.94 | 47.80 | 72.98 |
| g12r15of.rit | g12 | low | 3.31 | 3.47 | 0 | 70.42 | 54.16 | 80.88 |
| g12r60of.rit | g12 | low | 4.42 | 2.79 | 1 | 76.54 | 56.15 | 77.19 |
| g13r30of.rit | g13 | low | 2.41 | 3.07 | 0 | 59.23 | 51.44 | 82.56 |
| g13r60of.rit | g13 | low | 2.43 | 3.24 | 0 | 68.79 | 47.12 | 73.65 |
| g13rof.rit | g13 | low | 2.50 | 6.12 | 0 | 74.29 | 55.06 | 78.54 |
| g13r15of.rit | g13 | low | 7.29 | 6.26 | 1 | 52.65 | 29.58 | 62.62 |
| g13r45of.rit | g13 | low | 8.41 | 3.50 | 1 | 75.15 | 60.51 | 82.64 |
| g9r30of.rit | g9 | low | 4.00 | 3.51 | 1 | 72.95 | 50.10 | 77.46 |
| g9rof.rit | g9 | low | 4.96 | 2.89 | 1 | 71.26 | 56.51 | 74.94 |
| g9r45of.rit | g9 | low | 6.51 | 2.98 | 1 | 86.46 | 70.82 | 86.40 |
| g9r15of.rit | g9 | low | 6.57 | 2.85 | 1 | 85.88 | 73.98 | 86.86 |
| g9r60of.rit | g9 | low | 6.92 | 4.50 | 1 | 76.64 | 54.81 | 80.17 |
| s14rof.rit | s14 | low | 4.33 | 3.17 | 1 | 77.11 | 59.53 | 85.08 |
| s14r15of.rit | s14 | low | 4.40 | 3.22 | 1 | 74.34 | 56.10 | 73.95 |
| s14r60of.rit | s14 | low | 4.46 | 3.11 | 1 | 75.66 | 61.92 | 82.63 |
| s14r45of.rit | s14 | low | 4.49 | 2.74 | 1 | 69.79 | 50.45 | 76.23 |
| s14r30of.rit | s14 | low | 6.99 | 3.75 | 1 | 80.07 | 62.93 | 81.76 |
| s15r30of.rit | s15 | low | 3.00 | 2.66 | 0 | 77.88 | 63.61 | 81.94 |
| s15r15of.rit | s15 | low | 3.17 | 3.27 | 0 | 69.86 | 52.33 | 80.21 |
| s15rof.rit | s15 | low | 4.00 | 3.65 | 1 | 75.36 | 56.96 | 81.37 |
| s15r60of.rit | s15 | low | 4.01 | 2.67 | 1 | 82.46 | 65.44 | 81.35 |
| s15r45of.rit | s15 | low | 4.04 | 2.53 | 1 | 71.30 | 57.96 | 82.11 |
| s16rof.let | s16 | low | 5.11 | 2.23 | 1 | 89.29 | 86.60 | 93.43 |
| s16r45of.let | s16 | low | 5.16 | 2.39 | 1 | 86.64 | 84.04 | 91.09 |
| s16r60of.let | s16 | low | 5.56 | 3.21 | 1 | 63.21 | 59.43 | 84.13 |
| s16r30of.let | s16 | low | 5.60 | 3.12 | 1 | 83.62 | 78.66 | 90.80 |
| s16r15of.let | s16 | low | 5.66 | 2.01 | 1 | 85.63 | 79.45 | 89.82 |
