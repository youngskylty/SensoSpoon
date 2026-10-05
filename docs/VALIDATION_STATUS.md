# What Has and Has Not Been Validated

Read this before quoting any number from this repository.

Everything here falls into exactly one of four tiers, and the tier decides what a number is
allowed to claim. A result from tier 1 says something about the *algorithm*. A result from
tier 2 says something about the *signal*. Nothing in this project is tier 3, and tier 4 is
the list of things still taken on trust.

> **The sentence this project is entitled to write:** the control algorithm was validated in
> simulation, and public real-data analysis was used to test the sEMG assumptions it rests
> on. Physical closed-loop human validation was outside the scope of this project. Stronger
> wording than that is not supported.

---

## Tier 1 — Algorithmic simulation evidence

Scored against simulation ground truth, on a plant model that is an algorithm-development
baseline rather than a validated digital twin. Reproduce with `START_PROJECT('benchmark')`,
`('fusion')`, `('fastmove')`, `('jointtravel')`.

### 1.1 The controlled benchmark

Six arms × 20 cases, identical scenarios and identical sensor seeds (`9100+k`) for every
arm, so the only difference between blocks of rows is the arm definition. Cases 5, 14 and 20
are outside the hardware envelope by an inverse-dynamics bound that assumes perfect
knowledge and exact tracking (`rcflex_case_feasible.m`), and are excluded from the headline
mean rather than dragging it down — no controller could have served them.

| Arm | What it is | Mean suppression, feasible cases | Worst feasible case |
|---|---|---:|---:|
| `simple_pid` | fixed notch chain, the pre-EKF baseline | 90.23 % | 51.51 % |
| `ekf_cv_legacy` | 13-state EKF, un-propagated intent mean | 88.96 % | 30.75 % |
| `ekf_cv` | 13-state EKF, repaired intent model. **IMU-only control group** | 92.12 % | 85.88 % |
| `ekf_cv_osc2` | widened 18-state filter | 92.13 % | 85.90 % |
| `ekf_cv_qfix` | **IMU-only ablation** of the sEMG mechanism | 85.18 % | −41.41 % |
| `ekf_cv_semg` | **the shipped sEMG+IMU system** | **93.04 %** | 85.31 % |

`ekf_cv_semg` beats both the IMU-only control group *and* the IMU-only ablation of its own
mechanism, which is what makes the gain attributable to the sensor rather than to the
mechanism. `ekf_cv_qfix` — the classic "just lower `q_intent`" fix — is the arm that
collapses on voluntary motion (case 10 voluntary RMS 3.04° against 0.21°), exactly as
predicted.

### 1.2 The sEMG intent gate (E2)

The fusion mechanism, its calibration, and the controlled three-arm comparison:
`docs/SEMG_INTENT_FUSION.md`. Synthetic class separation 6.3 : 1 on the slow path and
3.0 : 1 on the fast path, against a ≥ 4 : 1 requirement declared in advance.

### 1.3 Fast voluntary movement (E3) — a documented negative result

`START_PROJECT('fastmove')`. Criteria declared in advance: latency < 350 ms **and**
confidence ≤ 0.10 on tremor-only **and** ≥ 0.50 on intent-bearing.

**No candidate met both.** The shipped amplitude gate opens at 407 ms (404–418 ms over four
seeds) and keeps the separation. Every rate-based arm that reached the latency target lost
the separation.

The reason is structural, not a matter of tuning: the rate of muscle drive is largest for
the *fastest* thing in the signal, and in a tremulous hand that is the tremor. Measured by
the project's own geometric-mean rule the rate observable separates the classes **the wrong
way round** — 0.69 : 1 at a 2 Hz corner falling to 0.13 : 1 at 16 Hz. A separating threshold
exists at all only at 2 Hz, where a fast reach peaks 2.3× above the worst tremor-only rate;
at 4 Hz and above, no threshold separates them at any value.

The target was not moved and nothing was adopted. The shipped design is retained, and the
cost stands: on `'reach'`, **0.697° attitude RMS against 0.289° for the IMU-only estimator**
(4 seeds), for +0.6 points of tremor suppression on the tremor-only cases. **The 20-case
benchmark contains no fast voluntary movement, so the tier 1.1 table does not include this
cost.** Anyone quoting 93.04 % should quote this alongside it.

### 1.4 The joint-travel authority limiter — repaired, and measurably inert

The travel branch of `rcflex_authority_manager` was inoperative from the day it was written:
the predicted correction angle was clipped to `C.joint_limit` before being compared against
`C.joint_limit`. It is repaired, and `tests/test_rcflex_authority_manager.m` fails if the
defect returns.

Measured effect: **none on the benchmark.** Re-running all five EKF arms with the repair
changed **zero of 960 metric cells** across the 120-row sweep. A sweep to 24° of tremor
(`START_PROJECT('jointtravel')`) shows why: the *current* branch is the binding constraint
at every amplitude that matters, and has already collapsed authority to 0.001 by the time
the travel branch would engage. The travel branch first binds at 24° of tremor, where it
moves γ from 0.0012 to 0.0009 and changes suppression not at all.

So the missing protection was already covered by another protection. That is the finding:
the repair is a correctness fix with no performance consequence, and saying so required
measuring it.

---

## Tier 2 — Public real-data evidence

Real human forearm sEMG, run through the *identical* preprocessing, feature and gate code
the controller uses. Reproduce with `START_PROJECT('realdata')`. Dataset provenance in
`docs/SEMG_DATASET.md`; the survey of what else was considered and rejected is in
`docs/SEMG_PUBLIC_DATA_SURVEY.md`.

**Dataset: GRABMyo (PhysioNet, CC BY 4.0), participants 1 and 2, session 1.** Healthy
subjects, 16 forearm electrodes at 2048 Hz, wrist flexion / wrist extension / rest. **No
tremor condition and no IMU.**

### 2.1 Validated directly

| Gate | Result |
|---|---|
| 1 Acquisition health | **PASS** both participants, unmodified thresholds. No clipping, 12.0–27.3 dB headroom, in-band fraction 0.95–0.98, mains −19 to −21 dB |
| 3 Raw cross-talk \|r\| | **P1 FAILS at 0.460**, P2 passes at 0.262, against a 0.30 provisional gate. Cross-talk decays monotonically with electrode separation, which is the signature of volume conduction. **Placement, not physiology, is the binding constraint** |

Two instrumentation facts that simulation could not have produced: real amplitudes are
16.6–130 µV rms against the synthetic model's ~700 µV, and real crest factors are 11.5–23.2
against the 7–10 assumed. Both change hardware decisions.

### 2.2 Partially validated — and it found a real problem

The class-separation claim the whole fusion rests on cannot be tested on public data, because
no public dataset records forearm sEMG with a measured tremor. What *can* be tested is
**voluntary contraction against quiescent rest**, which bounds the intent/tremor separation
**from above** — tremor is real muscle activity, rest is its absence, so a tremor segment can
only sit higher than a rest segment.

Measured through the shipped gate front end, by the same worst-case geometric-mean rule the
synthetic floor uses:

| | P1 | P2 | Synthetic | Target |
|---|---:|---:|---:|---:|
| **Flexion** vs rest (roll) | 10.1 : 1 | 5.1 : 1 | — | ≥ 4 : 1 |
| **Extension** vs rest (roll) | 2.5 : 1 | **0.46 : 1** | — | ≥ 4 : 1 |
| **Pooled worst case** (the declared rule) | **0.8 : 1** | **0.1 : 1** | 6.3 : 1 | ≥ 4 : 1 |

**Flexion behaves as designed. Extension does not, and the pooled rule therefore fails.**
Two mechanisms are visible in the data, and both are absent from the synthetic model:

1. **A standing DC offset in the differential envelope.** At rest the agonist−antagonist
   difference should be ≈ 0; on real electrodes it is 7–20 % of a strong contraction's
   drive, subject-specific, and the rectify-and-lowpass envelope passes it straight through
   the gate's 1 Hz filter. The synthetic source generates both channels with identical
   statistics, so its resting difference really is ≈ 0.
2. **Agonist/antagonist co-activation.** Stage 3b measures envelope correlation at 0.63–0.78
   on these recordings: the two muscles rise *together*, so their difference cancels much of
   the drive whenever the antagonist leads. The synthetic source generates its channels
   independently, so its reciprocity is perfect by construction.

The hypothesis that the offset alone explains the failure **was tested and rejected**:
subtracting the measured resting baseline lifts the pooled ratio from 0.8 to 1.1 and from
0.1 to 1.9 — real improvement, still far short of 4 : 1.

**What this does not establish.** It is not a refutation of the fusion. These are hand
gestures, not wrist stabilisation; the channel pairs are selected empirically; and the pitch
pair is an acknowledged substitute. What it establishes is that the synthetic 6.3 : 1 is
**not reproduced on real muscle in one of the two contraction directions**, and it names two
specific mechanisms the model omits. Neither is fixed in this revision, because fixing them
is an architectural change and the experiment that would justify it has not been run.

### 2.3 Not validated by any public data

| Gate | Why |
|---|---|
| 2 Burst SNR | Needs sEMG during tremor. No public dataset has it |
| 4 sEMG lead over motion | Needs synchronised motion. **NinaPro DB7 could answer a modified version and was not run** — see the survey §4 |
| 5 Frequency agreement | Needs synchronised motion *during tremor*. Nothing public has both |
| 6 Intent/tremor discrimination | Needs a labelled tremor class. Nothing public has one. §2.2 bounds it; it does not measure it |

### 2.4 Real tremor kinematics as the plant input (E4)

*Added after the sEMG work above, and a different kind of real data: this section is about
the CONTROLLER meeting a real waveform, not about the muscle signal. The sEMG stream is
synthetic throughout E4 and is generated from the same ground truth as before, so nothing
here strengthens or weakens §2.1–§2.3.*

Until E4, both halves of the evidence were synthetic: a synthetic muscle signal and a
synthetic tremor. Only one of those had to be. No public dataset pairs forearm sEMG with a
measured tremor (survey §2.4), but tremor recordings on their own are plentiful, and they
are enough to replace the sine.

**Dataset: PhysioNet `tremordb`** — "Effect of Deep Brain Stimulation on Parkinsonian
Tremor" (Beuter, Titcombe & Glass), Open Access. Index-fingertip velocity by laser
transducer, 100 Hz. Every stimulation-off, medication-off record was taken: **60 recordings
from 14 patients**, tremor with neither DBS nor L-dopa acting on it. All 60 verified against
the published SHA-256 checksums. Reproduce with `START_PROJECT('realtremor')`.

**What the recordings are allowed to supply.** Shape only. They are fingertip *translation*
and the plant input is hand *attitude*; nothing in the database converts one to the other,
and assuming a moment arm would present an assumption as a measurement. Each record is
therefore RMS-matched to a unit sine and the scenario states the amplitude, exactly as for a
synthetic case. What transfers is the temporal structure, and that is scale-free.

**What the synthetic model was missing, measured:**

| | Synthetic sine | Recorded tremor |
|---|---:|---:|
| Crest factor, median (range) | 1.414 | **2.77 (1.74–6.26)** |
| Dominant frequency, median (range) | as commanded | 4.48 Hz (2.41–8.41) |
| Records inside the declared 4–20 Hz envelope | — | 46 of 60 |

**The result, and it is not a flattering one.** Over the 46 records whose tremor falls inside
the 4–20 Hz envelope the device was designed for:

| Arm | Mean suppression (%) | Worst (%) | Meets the 90 % target |
|---|---:|---:|---:|
| `simple_pid` | 81.62 | 52.65 | **8 of 46** |
| `ekf_cv` (IMU only) | 73.16 | 29.58 | **8 of 46** |
| `ekf_cv_semg` (shipped) | **86.81** | 62.62 | **21 of 46** |

Paired against a synthetic sine at each record's *own* dominant frequency — same record,
seed, amplitude and duration, so only the waveform differs:

| Arm | Synthetic (%) | Recorded (%) | Change |
|---|---:|---:|---:|
| `simple_pid` | 80.94 | 81.07 | +0.13 |
| `ekf_cv` | 95.50 | 71.71 | **−23.79** |
| `ekf_cv_semg` | 96.11 | 86.57 | **−9.54** |

**Three things follow, and the first two are uncomfortable.**

1. **The 20-case synthetic benchmark is materially optimistic.** The headline 93.0 % of
   Tier 1 is a figure against smooth stationary tremor. Against recorded tremor the shipped
   system averages 86.8 % and meets its own 90 % target on fewer than half the records. The
   Tier 1 number is not withdrawn — it measures what it says it measures — but it must not
   be quoted as a figure for real tremor, and this section is why.
2. **The estimator is what degrades, not the plant.** The IMU-only EKF loses 23.8 points
   where the fixed-notch baseline loses none. The EKF carries a single-frequency tremor
   model, and real tremor wanders in frequency and amplitude in a way a sine never does, so
   the model is mis-specified exactly where the baseline's broad notch chain is indifferent.
   This is the same structural weakness V2 §6 identified, showing up again under a harder
   input.
3. **sEMG helps most where the estimator is weakest.** The gain over IMU-only widens from
   +0.9 points on the synthetic benchmark to **+13.7 points** on recorded tremor. The
   mechanism is consistent with §1.2: the worse the tremor model fits, the more tremor is
   absorbed into the intent state, and the gate is what stops that happening.

**What this does not establish.** It is not a clinical result and it is not evidence about
the sEMG assumptions. The recordings are Parkinsonian REST tremor of the index finger, not
postural tremor of the wrist while a person is feeding themselves; 14 of the 60 records have
a dominant peak below the 4 Hz design floor and are reported separately for that reason. A
single-axis recording drives both axes, the pitch axis being a quarter-period-shifted copy.
The claim this section supports is narrow and worth stating exactly: *the control algorithm
was tested against the recorded temporal structure of real Parkinsonian tremor, and its
advantage over the IMU-only estimator is larger there than on synthetic tremor, while its
absolute performance is materially worse than the synthetic benchmark suggests.*

---

## Tier 3 — Hardware / physical validation

**None. No physical prototype was built, and no human subject was recorded.**

There is no bench measurement of the actuator, no thermal measurement, no hysteresis or
creep measurement, no measured sEMG from the intended electrode placement, and no
closed-loop trial with a person holding the device. Every actuator, coil and flexure
parameter comes from manufacturer data sheets (`docs/sources/`) applied to a custom
structure that was never built.

---

## Tier 4 — Remaining assumptions

Each of these is load-bearing and untested. They are listed so that nothing on the list can
be quietly promoted to a result.

1. **The plant model has no hysteresis, dead zone or creep.** The project's standing caveat.
   It is an algorithm-development baseline, not a digital twin.
2. **`C.semg_emd_s = 40 ms`, the electromechanical delay**, is the entire reason sEMG is
   worth adding, and it rests on the literature (Cavanagh & Komi 1979) rather than on any
   measurement made here. Gate 4 is the gate that would test it.
3. **The synthetic sEMG is generated from the same intent truth that drives the plant**, so
   its envelope carries the voluntary drive by construction. Every tier 1.2 figure is an
   upper bound and the fusion result is a conditional.
4. **Reciprocal agonist/antagonist activation.** Tier 2.2 shows real co-activation of
   0.63–0.78, which the model does not have. This is now a measured gap, not an assumption,
   but the design has not been changed to accommodate it.
5. **A sustained isometric hold reads as voluntary drive.** Holding a loaded spoon still
   against gravity produces drive without motion, so the gate stays open and the system
   reverts to IMU-only behaviour. That is the safe direction of failure, and it is untested.
6. **The 0.30 cross-talk gate is provisional**, derived from the distance-decay structure of
   two participants, not from a clinical standard.
7. **The sEMG stream is not wired into the Simulink model**, only into the MATLAB
   controller. Deferred deliberately; it needs a `.slx` port change.
8. **Electrode shift, fatigue-driven spectral compression, and inter-session calibration
   stability** are all unmeasured.

---

## What would change the answer

One recording, roughly twenty minutes, per `SEMG_SUBSYSTEM.md` §6.3: forearm sEMG and an IMU
worn together, alternating rest, tremor-like shaking and deliberate reaching. It would close
gates 2, 4, 5 and 6, produce the first real measurement of the separation the whole fusion
depends on, and tell us whether the co-activation problem found in tier 2.2 also afflicts
tremor. It is the only outstanding item that changes what the result *means* rather than how
well it is characterised.

Gate 3's failure adds a cheap prerequisite to that session: site the agonist/antagonist pairs
on **opposite aspects** of the forearm and confirm raw zero-lag \|r\| < 0.30 on an isolated
contraction *before* recording the tremor trials.
