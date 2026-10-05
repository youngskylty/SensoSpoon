# Simulation, Testing and Feasibility Study

## A. Scope of the Evidence

The evidence presented in this section is of two kinds, and they are kept apart deliberately
because they support different claims.

The first is **algorithmic simulation evidence**: a controlled comparison of controller
variants over a fixed scenario set, in a simulated plant, driven by synthetic tremor. This
establishes whether the control architecture works, and it is the evidence against which the
delivered controller was designed and tuned.

The second is **public real-data evidence**: analysis of recordings made from human subjects
by other researchers and released publicly. It is used in two distinct ways — to test the
surface electromyography (sEMG) assumptions the fusion rests on, and to replace the synthetic
tremor input with a recorded one.

There is no hardware tier. No physical prototype was built, no bench measurement was taken,
and no human subject was recorded for this project. Every actuator, coil and flexure
parameter is a manufacturer data-sheet value applied to a structure that was never
manufactured. The claim this work is entitled to make is therefore bounded, and is stated
here in the form used throughout:

> The control algorithm was validated in simulation, and public real-data analysis was used
> both to test the sEMG assumptions it rests on and to drive the plant with recorded human
> tremor. Physical closed-loop human validation was outside the scope of this project.

Every result below is reproducible from the repository through a single entry point; the
commands are given in Section H.

## B. Simulation Environment and Its Verification

The plant is a two-degree-of-freedom gimballed spoon driven by voice-coil actuators through a
flexure transmission. The simulation integrates the mechanism kinematics, actuator electrical
dynamics, payload inertia and sensor noise at a 0.5 ms mechanical step, with the outer
attitude loop at 800 Hz and the inner current loop at 10 kHz.

Three things support treating this model as a usable development baseline. First, the
mechanism is verified analytically: rod-closure error is 3.5 × 10⁻¹⁸ m, the Jacobian error
4.5 × 10⁻¹² and the energy-balance residual 3.7 × 10⁻¹² W, so the kinematic and power
bookkeeping are exact to numerical precision. Second, the fast fixed-step solver used for the
sweeps is checked against a continuous-time Simulink reference on a nominal case, and the two
agree to within a small residual in joint angle and suppression. Third, the model is covered
by an automated suite of seventeen validation groups — mesh properties, packaging clearances,
physics, hardware limits, control behaviour, the estimator, the sEMG front end, the tremor
input path and the liquid model — which runs in approximately 80 s and must pass before any
result is generated.

What the model does **not** contain matters equally. There is no hysteresis, no creep and no
backlash in the flexure, and no friction model beyond a simple guide term. The plant is
therefore an algorithm-development baseline, not a validated digital twin, and absolute
performance figures inherit that limitation.

## C. Test Protocol and Metric Definitions

The benchmark comprises twenty scenarios spanning four questions: a tremor-frequency sweep at
4, 5, 7, 8, 9, 11 and 12 Hz; tremor waveform and amplitude (a mixed multi-frequency tremor, a
chirp, an irregular tremor, and a 3° amplitude case); voluntary motion, both alone and
superimposed on tremor; and plant robustness (no payload, 15 g payload, a 5.8 V bus in place
of 7.4 V, and a degraded actuator with +20 % coil resistance, −15 % force constant and
doubled guide friction).

**Tremor suppression** is the percentage reduction in attitude RMS relative to a baseline in
which the spoon is rigidly attached to the hand and subjected to the identical base input.
Scoring begins at *t* = 1.5 s to exclude the start-up transient, and no scoring filter is
applied. Because the metric is referred to an uncontrolled baseline, it is negative when a
controller does worse than no control at all — a property retained rather than clipped,
because it is diagnostic.

**Attitude error RMS** is reported alongside, and is the only meaningful metric on the
voluntary-motion-only scenario, where there is no tremor to suppress and the suppression
percentage is undefined.

Every variant is run over identical scenarios with identical sensor seeds, so the only
difference between blocks of results is the variant definition. This makes the comparison a
controlled experiment rather than a collection of separately tuned runs.

**Hardware feasibility.** Three of the twenty scenarios demand more of the actuator than it
can supply, judged by an inverse-dynamics bound that assumes perfect knowledge and exact
tracking and asks what current, voltage, copper power, travel and side load the input would
require. Case 5 (10 Hz, 3°) needs 7.10 V of a 7.4 V bus; case 14 (8 Hz, 5°) needs 7.50 V;
case 20 (20 Hz, 1°) needs 1.84 A against a 1.5 A limit, 9.75 V, and 5.28 W against a 4.4 W
continuous rating. No controller could have succeeded on these, so scoring them measures the
actuator rather than the algorithm. They are excluded from the headline mean, and all three
averages — all scored scenarios, feasible scenarios only, and feasible scenarios that also
stayed within limits — are always reported together so the exclusion is visible rather than
silent.

## D. Controller Comparison on Synthetic Tremor

Table I gives the six-variant comparison. The variants differ only in the estimator and in
whether the sEMG stream is used; plant, scenarios and seeds are identical throughout.

**TABLE I. CONTROLLER COMPARISON, 20 SCENARIOS**

| Variant | Description | All scored (%) | **Feasible (%)** | Worst feasible (%) | Voluntary-only error (°) | Meets 90 % |
|---|---|---:|---:|---:|---:|---:|
| `simple_pid` | Fixed notch chain (original) | 64.19 | 90.23 | 51.51 | 0.4583 | 15/16 |
| `ekf_cv_legacy` | First EKF, with the intent-propagation defect | 68.75 | 88.96 | 30.75 | 1.2390 | 11/16 |
| `ekf_cv` | Corrected EKF, IMU only | 71.03 | 92.12 | 85.88 | 0.2008 | 11/16 |
| `ekf_cv_osc2` | As above, plus a second tremor oscillator | 71.04 | 92.13 | 85.90 | 0.2008 | 11/16 |
| `ekf_cv_qfix` | Ablation: same mechanism, no sEMG | 65.62 | 85.18 | −41.41 | 3.0418 | 15/16 |
| **`ekf_cv_semg`** | **sEMG + IMU (delivered)** | **72.63** | **93.04** | 85.31 | 0.2070 | **15/16** |

The delivered controller averages **93.04 % tremor suppression** over the sixteen feasible
scenarios and meets the 90 % design target on fifteen of them, while holding voluntary-motion
error to 0.207° against 0.458° for the original controller. This is the result the controller
was designed and tuned against, and it is the figure that justified the architecture: it is
the first variant that is not a trade-off, matching the original controller on plain tremor,
matching the IMU-only estimator on deliberate movement, and beating both on multi-frequency
tremor. Fig. 1 gives the scenario-by-scenario comparison.

![**Fig. 1.** Tremor suppression across the sixteen hardware-feasible benchmark scenarios, for the original controller, the IMU-only estimator and the delivered sEMG + IMU system, grouped by tremor frequency, waveform and amplitude, voluntary motion, and plant robustness. The dashed line is the 90 % design target. Three scenarios exceeding the actuator's inverse-dynamics feasibility bound, and one carrying no tremor, are excluded.](../output/figures/report/fig_benchmark_suppression.png)

Two properties of this result matter as much as the headline. First, it is not purchased with
actuator effort: peak current across the whole sweep is 1.4320 A for the delivered controller
against 1.4305 A for the IMU-only estimator, and peak mean coil power 5.719 W against
5.723 W — differences of about 0.1 %, comfortably inside the thermal and battery budget. A
suppression gain bought with extra current would not have survived those limits. Second, the
gain is concentrated where the original controller was weakest: on the scenario combining
voluntary movement with tremor it collapses to 51.51 %, while the delivered controller holds
85.31 %.

## E. Attribution: the Sensor, not the Mechanism

sEMG enters the estimator as a per-axis, per-sample scaling of the process noise assigned to
the voluntary-motion state — in effect a confidence signal deciding how freely the estimator
may attribute motion to intent. A reasonable objection is that the benefit comes from that
scaling and not from the muscle signal at all. The `ekf_cv_qfix` variant exists to answer it:
it applies the identical scaling with the confidence held constant and no muscle signal
present.

**TABLE II. ABLATION — WHAT THE SENSOR CONTRIBUTES**

| Measure | Ablation (no sEMG) | sEMG + IMU | Attributable to the sensor |
|---|---:|---:|---|
| Feasible-case mean (%) | 85.18 | 93.04 | +7.86 points |
| Movement + tremor (%) | −41.41 | 85.31 | +126.7 points |
| Voluntary-only error (°) | 3.0418 | 0.2070 | 14.7× lower |

The ablation behaves as predicted: holding the confidence at a constant favourable value
improves multi-frequency tremor suppression and destroys everything involving deliberate
movement. Any gain the delivered controller shows over *both* the IMU-only estimator and this
ablation is therefore attributable to the sEMG stream and to nothing else. Inspecting the
estimator confirms the designed mechanism directly: tremor mis-attributed to the voluntary
state falls by 2.6× to 3.1× on the multi-frequency scenarios (0.171° → 0.066°, 0.170° →
0.055°, 0.038° → 0.014°) with no loss of movement tracking. Fig. 2 shows both halves of the
trade.

![**Fig. 2.** The sEMG ablation. (a) Tremor mis-attributed to the voluntary-motion state. (b) Voluntary motion the estimator failed to follow. The ablation applies the identical process-noise scaling with no muscle signal present; it wins the left panel and loses the right one, which is the trade the sensor exists to break. Logarithmic ordinates; lower is better in both panels.](../output/figures/report/fig_semg_ablation.png)

## F. Limitations Identified by Testing

Three tests were run specifically to find failure modes, and all three found one. They are
reported in full, because a test that can only confirm is not a test.

**1) Fast voluntary movement.** The twenty-case benchmark contains no *fast* voluntary
movement — every voluntary motion in it is a slow 5 s oscillation. A 0.8 s reach superimposed
on tremor was therefore added, and a response-time target of 350 ms was declared in advance.
The delivered gate achieves **407 ms** (404–418 ms over four sensor seeds). In closed loop the
cost is real: attitude error on the reach is 0.697° against 0.289° for the IMU-only estimator,
about **2.4× worse**, bought for roughly 0.6 points of tremor suppression (92.26 % against
91.64 %). The headline figures in Table I do not include this cost, because the benchmark
contains no such scenario, and any quotation of 93.04 % should carry it alongside.

![**Fig. 4.** The fast-reach scenario and the gate's response to it. (a) A 0.8 s voluntary reach superimposed on 8 Hz tremor. The reach is not separable by eye in the measured hand signal, which is what the estimator has to work from; both estimators recover it, the sEMG + IMU estimate lagging slightly. (b) The gate confidence that scales the estimator's intent process noise. The gate is held open during a 1 s warm-up so that it cannot assert an absence of muscle drive before its filters have settled. Against the 350 ms target declared in advance, the gate reaches half confidence 403 ms after movement onset at the seed shown; over the four declared seeds the experiment records 407 ms (404–418 ms).](../output/figures/report/fig_reach_gate.png)

**2) The rate-of-change detector, and a retracted measurement.** An earlier revision of this
work recommended replacing the amplitude detector with a rate-of-change detector, on an
informal observation of a single trace quoted as roughly fourfold earlier detection. Tested
properly over three detector families and four corner frequencies, with thresholds derived by
the same rule used for the delivered detector and the reach scenario held out, that
recommendation does not survive. The rate observable separates the two classes **the wrong
way round**: against a requirement of at least 4:1, the measured separations are 0.69:1 at a
2 Hz corner, 0.27:1 at 4 Hz, 0.14:1 at 8 Hz and 0.13:1 at 16 Hz. The cause is structural
rather than a matter of tuning — the rate of muscle drive is largest for the fastest thing in
the signal, and in a tremulous hand that is the tremor. The admissible region of the
latency-versus-separation plane is empty. **No candidate was adopted and the 350 ms target was
not moved**; the earlier measurement is recorded as retracted rather than deleted.

**3) Real forearm sEMG does not separate both contraction directions.** The class separation
the fusion depends on cannot be measured on public data, because no public dataset records
forearm sEMG together with a measured tremor. What can be measured is voluntary contraction
against quiescent rest, which bounds the intent-versus-tremor separation from above. Run
through the delivered gate front end on two GRABMyo participants [4], wrist **flexion**
separates as designed — 10.1:1 and 5.1:1, taken as the roll-axis median drive against the
resting median — but wrist **extension** reads 2.5:1 and 0.46:1 by the same statistic. The
declared pooled rule is stricter still, taking the worst case across both conditions and both
axes, and on that rule the two participants give 0.8:1 and 0.1:1 against a 6.3:1 synthetic
figure and a 4:1 requirement. Two mechanisms absent from the synthetic model account for it: a standing DC
offset of 7–20 % of a strong contraction's drive in the differential envelope at rest, and
genuine agonist/antagonist co-activation of 0.63–0.78. Subtracting the measured resting
baseline was tested as a remedy and lifts the pooled ratios only to 1.1 and 1.9 — a real
improvement, still far short. Neither mechanism is fixed here, and the synthetic separation
figure must be read as optimistic in a way that was not previously visible.

## G. Feasibility Study: Recorded Human Tremor

### 1) Motivation

Every figure in Sections D and E was measured against tremor this project generated itself —
sines, chirps and sums of a few sines, which are smooth, stationary, and have a crest factor
of 1.414. Real pathological tremor is none of those things. Since no public dataset pairs
forearm sEMG with a measured tremor, the muscle signal must remain synthetic; but tremor
recordings on their own are plentiful, so there is no reason for the *tremor* to be synthetic
as well. This study replaces it.

### 2) Method

The tremor input is taken from the PhysioNet `tremordb` database [1], [2]: index-fingertip
velocity recorded by laser transducer at 100 Hz from patients with Parkinson's disease. Every
stimulation-off, medication-off record was used — **60 recordings from 14 patients**, tremor
with neither deep brain stimulation nor L-dopa acting on it, which is the most natural tremor
the database contains. All 60 were verified against the published SHA-256 checksums.

Each recording is band-pass filtered over 2–25 Hz, integrated from velocity to displacement,
band-pass filtered again to remove the integration ramp, and resampled, with five seconds
skipped at each end to exclude filter transients and transducer settling.

Two decisions bound what the study may claim. First, the recordings supply **waveform shape
only** and are amplitude-normalised. They are fingertip translation, whereas the plant input
is hand attitude; nothing in the database permits converting one into the other, and assuming
a moment arm would present an assumption as a measurement. What transfers is the temporal
structure — amplitude modulation, harmonic content, frequency wander, non-stationarity — and
that is scale-free. Second, each record is normalised to the RMS of a **unit-amplitude sine**
rather than to unit RMS, so that a recorded scenario and a synthetic scenario of the same
nominal amplitude present the same tremor energy; without this the comparison would measure
the scaling rather than the waveform.

Amplitude, duration, solver step, sensor seed and scoring window are identical to the
synthetic benchmark. For the fourteen primary records a paired synthetic control is also run
at **that record's own dominant frequency**, so the comparison isolates waveform structure
from frequency instead of confounding the two.

### 3) What the Synthetic Model Was Missing

**TABLE III. TREMOR INPUT PROPERTIES**

| Property | Synthetic sine | Recorded tremor |
|---|---:|---:|
| Crest factor, median (range) | 1.414 | **2.77 (1.74–6.26)** |
| Dominant frequency, median (range) | as commanded | 4.48 Hz (2.41–8.41) |
| Records inside the declared 4–20 Hz envelope | — | 46 of 60 |

Fourteen records fall below the 4 Hz design floor. Thirteen of those come from the
low-amplitude tremor group — subjects receiving stimulation for rigidity or dyskinesia rather
than for tremor — whose residual tremor is small enough that the dominant spectral peak
reflects low-frequency content rather than a tremor proper. Results are therefore reported
both over all 60 records and over the 46 inside the declared envelope, so that a statement
about the design brief is not folded into a statement about waveform shape.

### 4) Results

**TABLE IV. SUPPRESSION ON RECORDED TREMOR, 46 IN-BAND RECORDS**

| Variant | Mean (%) | Worst (%) | Meets 90 % | Attitude RMS (°) |
|---|---:|---:|---:|---:|
| `simple_pid` | 81.62 | 52.65 | 8 of 46 | 0.2080 |
| `ekf_cv` (IMU only) | 73.16 | 29.58 | 8 of 46 | 0.3073 |
| **`ekf_cv_semg`** | **86.81** | 62.62 | **21 of 46** | **0.1464** |

**TABLE V. PAIRED AGAINST A SYNTHETIC SINE AT THE SAME FREQUENCY**

| Variant | Synthetic (%) | Recorded (%) | Change (points) | Worst change |
|---|---:|---:|---:|---:|
| `simple_pid` | 80.94 | 81.07 | +0.13 | −23.22 |
| `ekf_cv` | 95.50 | 71.71 | **−23.79** | −41.29 |
| `ekf_cv_semg` | 96.11 | 86.57 | **−9.54** | −22.94 |

Fig. 3 shows a representative recording against its RMS-matched sine, and the paired
comparison.

![**Fig. 3.** Recorded human tremor. (a) Two seconds of a conditioned `tremordb` recording against the RMS-matched synthetic sine at the same dominant frequency. (b) Mean tremor suppression for each controller over the paired in-band records, synthetic against recorded; error bars are one standard deviation across records.](../output/figures/report/fig_real_tremor.png)

### 5) Interpretation

Three findings follow, and the first two are unfavourable.

**The synthetic benchmark is materially optimistic.** The 93.04 % of Table I is a figure for
smooth, stationary tremor. Against recorded tremor the delivered controller averages 86.81 %
and meets its own 90 % target on fewer than half the records. Table I is not withdrawn — it
measures what it says it measures, and it is the basis on which the controller was designed —
but it must not be quoted as a figure for real tremor.

**The estimator is what degrades, not the plant.** The IMU-only estimator loses 23.79 points
where the fixed-notch baseline loses essentially none. The explanation is structural: the
estimator carries a single-frequency tremor model, and real tremor wanders in frequency and
amplitude in a way a sine does not, so the model is mis-specified precisely where the
baseline's broad notch chain is indifferent to the difference. This is the same weakness
identified in the estimator's design analysis, reappearing under a harder input.

**The muscle signal helps most where the estimator is weakest.** The margin of the delivered
controller over the IMU-only estimator widens from **+0.92 points** on synthetic tremor to
**+13.65 points** on recorded tremor. This is consistent with the mechanism of Section E: the
worse the tremor model fits the data, the more tremor is absorbed into the voluntary-motion
state, and the intent gate is what prevents that absorption. On this evidence the argument for
fusing sEMG is stronger against real tremor than against the synthetic tremor the design was
tuned on.

### 6) What This Does Not Establish

This is a real-signal robustness test of the control algorithm. It is not a clinical result,
and it is not evidence about the sEMG assumptions, which stand exactly where Section F leaves
them — the muscle signal remains synthetic throughout and is generated from the same ground
truth that drives the simulated hand. The recordings are Parkinsonian **rest** tremor of the
index finger, not postural tremor of the wrist during self-feeding. A single-axis recording
drives both controlled axes, the second being a quarter-period-shifted copy. The precise claim
supported is narrow: *the control algorithm was tested against the recorded temporal structure
of real Parkinsonian tremor; its advantage over the IMU-only estimator is larger there than on
synthetic tremor, while its absolute performance is materially worse than the synthetic
benchmark indicates.*

## H. Threats to Validity and Reproduction

The principal threats to validity, in order of how much they would change the conclusions,
are: the absence of any hardware measurement, which leaves every actuator and flexure
parameter unverified; the synthetic muscle signal, which carries the information asked of it
by construction and therefore makes every sEMG figure an upper bound; the plant model's lack
of hysteresis, creep and backlash; the unmeasured 40 ms electromechanical delay on which the
rationale for sEMG rests, taken from the literature [3] rather than measured here; and the
fact that a sustained isometric hold would be read as deliberate movement, which is untested
and is the safe direction of failure.

Closing these requires one experiment that was out of scope: a short synchronised recording of
forearm sEMG and limb motion from a person, alternating rest, tremor and deliberate reaching.
Twenty minutes of such data would close four of the six open validation stages and produce the
first real measurement of the class separation on which the fusion depends.

All results are reproduced from the repository with:

```matlab
START_PROJECT('setup')        % paths and output folders
START_PROJECT('test')         % 17 validation groups
START_PROJECT('benchmark')    % Tables I, II and Figs. 1, 2
START_PROJECT('fastmove')     % Sections F.1 and F.2
START_PROJECT('realdata')     % Section F.3
START_PROJECT('realtremor')   % Tables III-V and Fig. 3
START_PROJECT('all')          % everything, in dependency order
```

## References

[1] A. Beuter, M. S. Titcombe, F. Richer, C. Gross, and D. Guehl, "Effect of deep brain
stimulation on amplitude and frequency characteristics of rest tremor in Parkinson's disease,"
*Thalamus & Related Systems*, vol. 1, no. 3, pp. 203–211, 2001.

[2] A. L. Goldberger *et al.*, "PhysioBank, PhysioToolkit, and PhysioNet: components of a new
research resource for complex physiologic signals," *Circulation*, vol. 101, no. 23,
pp. e215–e220, 2000.

[3] P. R. Cavanagh and P. V. Komi, "Electromechanical delay in human skeletal muscle under
concentric and eccentric contractions," *European Journal of Applied Physiology*, vol. 42,
no. 3, pp. 159–163, 1979.

[4] A. Pradhan, J. He, and N. Jiang, "Multi-day dataset of forearm and wrist electromyogram
for hand gesture recognition and biometrics," *PhysioNet*, v1.1.0, 2022,
doi: 10.13026/rzbf-ts96. (GRABMyo.)

[5] E. Rocon, J. M. Belda-Lois, A. F. Ruiz, M. Manto, J. C. Moreno, and J. L. Pons, "Design
and validation of a rehabilitation robotic exoskeleton for tremor assessment and suppression,"
*IEEE Trans. Neural Syst. Rehabil. Eng.*, vol. 15, no. 3, pp. 367–378, 2007.

[6] Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, *Estimation with Applications to Tracking and
Navigation*. New York, NY, USA: Wiley, 2001.

[7] J. A. Gallego, E. Rocon, J. O. Roa, J. C. Moreno, and J. L. Pons, "Real-time estimation of
pathological tremor parameters from gyroscope data," *Sensors*, vol. 10, no. 3, pp. 2129–2149,
2010.

[8] H. J. Hermens, B. Freriks, C. Disselhorst-Klug, and G. Rau, "Development of recommendations
for SEMG sensors and sensor placement procedures," *Journal of Electromyography and
Kinesiology*, vol. 10, no. 5, pp. 361–374, 2000.

[9] C. N. Riviere, R. S. Rader, and N. V. Thakor, "Adaptive cancelling of physiological tremor
for improved precision in microsurgery," *IEEE Trans. Biomed. Eng.*, vol. 45, no. 7,
pp. 839–846, 1998.
