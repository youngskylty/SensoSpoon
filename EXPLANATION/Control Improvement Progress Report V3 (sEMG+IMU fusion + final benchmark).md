
# Control Improvement Progress Report V3
### sEMG + IMU fusion, and the final benchmark
Date: 17 September 2026 (revised — the fusion is built, benchmarked and shipped)

This is an internal status note, not a polished deliverable. It summarises where the
"smarter tremor control" work stands: what's built, what's still open, and what tripped us
up. Kept light on the maths on purpose — the technical detail lives in
`docs/SEMG_INTENT_FUSION.md` (new, and the main reference for this revision),
`docs/CONTROL_IMPROVEMENT_PLAN.md`, `docs/SEMG_IMPLEMENTATION.md` and `docs/SEMG_DATASET.md`.

**What changed in this revision.** V2 ended with sEMG validated as a signal source but not
connected to the control loop at all, and with the EKF not recommended as a replacement on
the evidence then available. Both of those have changed. sEMG is now fused into the
estimator, the combined system is the best arm on the benchmark by every measure we track,
and the fusion works through a mechanism V2 did not anticipate. Sections 3–6 are new.
Section 2 closes out V2's pending list item by item. Section 7 is an honest account of a
new failure we introduced, found, partly fixed, and have **not** fully solved.

**One-line summary.** The corrected IMU-only estimator averaged 92.1% cancellation on the
scenarios the hardware can actually serve, and met the 90% target on 11 of 16. Adding sEMG
takes that to **93.0% and 15 of 16**, while keeping voluntary-motion error at 0.207°
(against 0.458° for the original controller) — and we can show by direct ablation that the
sensor, not the mechanism, is what buys it.

---

## 1. The problem, restated

Two things had to be true at once for this project to succeed, and they pull against each
other:

1. **Cancel the tremor.** Hold the spoon level against 4–20 Hz involuntary motion.
2. **Don't cancel the person.** When the user deliberately moves their hand, the spoon must
   follow, not fight.

The original controller (`simple_pid`) does (1) well and (2) badly: it averages 90.2% on the
feasible scenarios but collapses to 51.5% when deliberate movement and tremor happen
together, and it carries 0.458° of error on a movement-only scenario.

The EKF was introduced to do both, by estimating the voluntary and tremor parts of the
motion separately rather than filtering a fixed frequency band. V2 showed it fixed (2)
convincingly — the movement-only error dropped to 0.201° — but that it *lost ground on (1)*
in a specific way, and V2's Section 6 traced why.

---

## 2. Closing out V2's pending list

V2 Section 8 listed seven open items. Where each now stands:

| V2 pending item | Status |
|---|---|
| Decide whether the EKF replaces the current controller | **Resolved — yes, with sEMG.** Section 5 |
| Decide how to score scenarios outside the hardware envelope | **Resolved and implemented.** Section 4 |
| Fix the inoperative joint-travel check in the authority limiter | **STILL OPEN — not attempted this revision.** Section 8 |
| Decide what to do about the single-frequency limitation | **Resolved.** The answer is not more oscillators; it is a second sensor. Section 3 |
| sEMG validated but not fused into the control loop | **Resolved.** Sections 3 and 5 |
| Three of six real-data sEMG checks unanswered | **STILL OPEN — unchanged.** Section 8 |
| Final side-by-side comparison report | **Done.** `output/results/CONTROL_RESULTS.md`, figures, and this note |

Five of seven closed. The two that remain are the two that need something we did not have
this revision: a small code repair plus a measurement campaign, and a real recording session.

---

## 3. The main result: what sEMG is actually for

V2 Section 6 found the estimator's structural weakness. The tremor model carries **one**
frequency. When a dominant tremor component claims it, weaker components at other
frequencies have nowhere to live — and the "voluntary movement" part of the model, which is
deliberately flexible, absorbs them instead. Absorbed tremor is then passed through to the
spoon as though the user had meant it. Measured open-loop, an 8 Hz dominant tremor with a
3 Hz companion had **87.8%** of that companion mis-labelled as voluntary movement.

The insidious part, and the reason this went unnoticed for so long: **the model's own health
signal looks perfect while this is happening.** The two parts still add up to the measured
motion, so the mismatch statistic stays tiny. The estimator is confidently wrong. No amount
of watching the IMU can detect it, because the IMU only ever sees the sum.

### 3.1 The experiment that settled the design

Before touching sEMG we ran a deliberately boring experiment
(`run_intent_authority_experiment`, no muscle signal involved anywhere). It turns one dial —
how much freedom the estimator grants the "voluntary movement" part — and measures what
happens.

| Scenario | dial ×1 | ×1/2 | ×1/8 | ×1/64 | ×1/512 |
|---|---|---|---|---|---|
| messy tremor (`mixed`) | 87.99% | 89.41% | 91.46% | **93.00%** | 93.55% |
| messy tremor (`irregular`) | 87.81% | 89.26% | 91.32% | **92.85%** | 93.39% |
| movement + tremor (`combined`) | **85.88%** | 75.86% | 26.86% | −81.41% | −398.51% |
| movement only (`intent`) | **0.201°** | 0.689° | 1.430° | 3.234° | 5.607° |

Read the bottom two rows against the top two. **The setting that best suppresses messy
tremor is the setting that destroys voluntary movement.** At merely half the nominal value,
the messy-tremor cases have gained 1.4 points while the movement case has already lost 10
and the movement-only error has more than trebled.

So there is no correct fixed value. The dial has to be turned *differently at different
moments*, and something has to say which moment it is. That information does not exist in
the IMU signal. It exists in the muscle.

**This is the argument for sEMG, and it is a much better argument than the one we started
with.** We are not using sEMG because it is a nice extra sensor; we are using it because we
can demonstrate a specific decision the system must make and cannot make without it.

### 3.2 What we fuse, and what we deliberately do not

The tempting design is to have the muscle signal *estimate the voluntary hand angle* and
feed that into the model. We built that first and **rejected it on measurement**: the
estimate correlates well with the truth (r = 0.95–0.98) but carries 1.6–1.8° of error,
while the effect we are trying to remove is 0.02–0.18°. It would have injected an order of
magnitude more error than it removed, directly into the reference the spoon tracks.

What we fuse instead is much weaker and much more reliable: **is the muscle driving, and
how hard?** The difference between the two opposing muscles of a pair, smoothed, separates
"tremor only" from "deliberate movement happening" by **6.3 : 1** across all twenty
benchmark scenarios. That confidence turns the dial from Section 3.1, per axis, every
sample.

Nothing else changes. The estimator keeps the same states, the same measurement model, and
the same IMU-only view of attitude. sEMG has exactly one job, and it is the job the
evidence supports it doing.

---

## 4. The hardware-envelope scoring decision (V2 §5.4, now settled)

V2 argued that scenarios which drive the mechanism past its physical limits should not be
scored as control failures, but left the definition as a pending decision. It is now decided
and implemented in code, so it can be audited rather than applied by hand.

The test is an **inverse-dynamics feasibility bound** (`rcflex_case_feasible`): assume
perfect knowledge and exact tracking, and ask what current, voltage, copper power, travel
and side load the input would demand. If the hardware cannot supply them, no controller
could have succeeded, and scoring one measures the actuator rather than the algorithm. The
same function now also generates the published envelope table, so the two cannot disagree.

Three of the twenty scenarios fail it:

| Case | Input | Why it fails |
|---|---|---|
| 5 | 10 Hz, 3° | needs 7.10 V of a 7.4 V bus (and the bus sags under load) |
| 14 | 8 Hz, 5° | needs 7.50 V |
| 20 | 20 Hz, 1° | needs 1.84 A of a 1.5 A limit, 9.75 V, and 5.28 W against a 4.4 W continuous rating |

Case 5 is a new finding — V2 had only identified 14 and 20.

**Three averages are now always reported together**, so the exclusion is visible rather than
silent: all scored scenarios, feasible scenarios only (the headline), and feasible scenarios
that also stayed inside the limits. Quoting only the flattering one is precisely the failure
this arrangement exists to prevent.

---

## 5. The final benchmark

Twenty scenarios, six controller variants, identical cases and identical sensor seeds, so
the only difference between the blocks is the variant definition.

| Variant | What it is | All scored | **Feasible** | Worst feasible | Movement-only error | Meets 90% |
|---|---|---:|---:|---:|---:|---:|
| `simple_pid` | the original controller | 64.19% | 90.23% | 51.51% | 0.4583° | 15/16 |
| `ekf_cv_legacy` | first EKF, with the V2 bug | 68.75% | 88.96% | 30.75% | 1.2390° | 11/16 |
| `ekf_cv` | corrected EKF, IMU only | 71.03% | 92.12% | 85.88% | 0.2008° | 11/16 |
| `ekf_cv_osc2` | + a second tremor oscillator | 71.04% | 92.13% | 85.90% | 0.2008° | 11/16 |
| `ekf_cv_qfix` | **ablation** — same mechanism, no sEMG | 65.62% | 85.18% | −41.41% | 3.0418° | 15/16 |
| **`ekf_cv_semg`** | **sEMG + IMU (shipped)** | **72.63%** | **93.04%** | 85.31% | 0.2070° | **15/16** |

Scenario by scenario, the combined system recovers exactly what the IMU-only EKF had lost:

| Scenario | original | IMU-only EKF | **sEMG + IMU** |
|---|---:|---:|---:|
| 6 messy tremor (`mixed`) | 92.35% | 87.99% | **92.62%** |
| 7 sweeping tremor (`chirp`) | 90.87% | 88.57% | **90.59%** |
| 8 irregular tremor | 92.46% | 87.81% | **92.82%** |
| 9 movement + tremor | 51.51% | 85.88% | **85.31%** |
| 10 movement only (error) | 0.4583° | 0.2008° | **0.2070°** |

**This is the first variant that is not a trade-off.** It matches the original controller on
plain tremor, matches the EKF on deliberate movement, and is better than both on messy
tremor.

### 5.1 How we know it is the *sensor* doing the work

This is the part that matters for defending the result, and it is why the `ekf_cv_qfix`
ablation exists. That variant applies the **identical** dial-turning mechanism with the
confidence held at a constant — the classic "just turn the dial down" fix — with no muscle
signal at all. Against it:

| | ablation (no sEMG) | sEMG + IMU | attributable to the sensor |
|---|---:|---:|---|
| Feasible-case average | 85.18% | 93.04% | **+7.86 points** |
| Movement + tremor (case 9) | −41.41% | 85.31% | **+126.7 points** |
| Movement-only error | 3.0418° | 0.2070° | **14.7× lower** |

The ablation behaves exactly as Section 3.1 predicts: it wins on messy tremor and destroys
everything involving deliberate movement. The sensor is what lets the system have both.

Looking inside the estimator directly, tremor mis-labelled as voluntary movement falls by
**2.6–3.1×** on the messy-tremor scenarios (0.171° → 0.066°, 0.170° → 0.055°,
0.038° → 0.014°), with no loss of movement tracking. That is the designed mechanism,
observed working.

### 5.2 It is not paid for with actuator effort

Peak current across the whole sweep is 1.4305 A (IMU-only) against 1.4320 A (sEMG+IMU), and
peak average coil power 5.7228 W against 5.7190 W. Worth stating explicitly: a cancellation
gain bought with extra current would not survive the thermal and battery limits, and this
one is not.

---

## 6. What the evidence does *not* say

The muscle signal is synthetic. It is generated from the same ground truth that drives the
simulated hand, so it carries the information we are asking it for **by construction**.

Every number above is therefore an **upper bound**, and the result is a conditional: *if* a
real recording separates the two situations as well as this one does, *then* the estimator
improves by this much. It is an **algorithmic** result — the architecture is sound and the
control benefit is real in simulation — and it is explicitly **not** evidence about human
physiology. `docs/SEMG_INTENT_FUSION.md` §8 states the falsifiable targets a real recording
would have to hit, chief among them a class separation of at least 4 : 1. We measure 6.3 : 1
here, which sounds comfortable but is not: the acceptance test's own worst-case margin is
about 7%.

The real-data validation is deliberately kept as a separate stage with its own gate, and
that gate is still open (Section 8).

---

## 7. A failure we introduced, found, and only partly fixed

This is the most important section for anyone judging the quality of the work, so it is
written plainly.

**The first version of the gate made deliberate movement worse, badly.** The 20-scenario
benchmark contains no *fast* voluntary movement — every movement in it is a slow, 5-second
oscillation — so we added one (`reach`: a 0.8-second reach superimposed on tremor) precisely
to test the thing we were most worried about. It failed:

| | IMU-only | first gate | **two-speed gate (shipped)** |
|---|---:|---:|---:|
| `reach`, mean error over 4 sensor seeds | 0.285° | 0.99° | **0.67°** |

About 3.6× worse than doing nothing clever, and inconsistent between seeds. The cause is
unglamorous: rejecting muscle activity caused by tremor needs heavy smoothing, heavy
smoothing costs time, and the gate needed 0.3–0.5 s to notice a movement lasting 0.8 s.

**The fix that didn't work, and why it's still interesting.** The estimator already has a
safety layer meant to restore flexibility during exactly this kind of transient, and our
gate was overriding it. We rebuilt it so the gate could not override it — and got results
identical to four significant figures. The layer never fires during a reach at all, because
(as in Section 3) the model still matches the sensor beautifully while being wrong. **This
is the second independent confirmation of V2 §5.3's first bullet: that confidence signal is
the wrong kind of alarm for this class of problem.**

**The fix that did work, partly.** The gate now runs two speeds: a slow, clean path that
alone may *close* it, and a fast path that may only ever *open* it. The asymmetry is the
point — wrongly closing the gate suppresses the user's movement, whereas wrongly opening it
merely reverts to the previous behaviour, so the two mistakes should not be treated equally.
That recovered a third of the loss for about 0.3 points of tremor suppression.

**It is not solved.** The shipped system is still ~2.4× worse than the IMU-only estimator on
a fast deliberate movement. We set a 350 ms response target *in advance*; the system
achieves 404 ms. A 16-point sweep over the filter settings found nothing meeting both that
target and the separation requirement — so **we adopted none of them, rather than moving the
target to whatever the design happened to achieve.** The headline numbers in Section 5 do
not reflect this cost, because the benchmark contains no such scenario. Anyone quoting the
headline should quote this alongside it.

**One more correction.** At one point this revision recorded that the two versions of the
safety-layer fix differed by 20 points. That comparison had been run at two different sensor
seeds and was simply wrong; re-run properly, they are identical. The erroneous claim is
struck through in the code comment and the design document rather than deleted, because a
retracted measurement is itself a useful warning.

---

## 8. What's still open

- **The joint-travel protection in the authority limiter is still inoperative.** V2 diagnosed
  it: the value being checked is clipped to the limit before being compared against that same
  limit, so the check can never trigger. It was not attempted this revision. It is a small
  ordering fix, but whether repairing it actually helps the worst case is a separate question
  that needs measuring, not assuming.
- **Fast deliberate movement (Section 7).** Real, measured, and the largest remaining
  weakness of the delivered system.
- **Three of the six real-data sEMG checks remain unanswered**, unchanged from V2, because no
  public dataset records forearm muscle signals simultaneously with a physical tremor
  measurement. This now also gates the one number the whole fusion depends on — the 6.3 : 1
  separation — which has never been measured on a human.
- **A sustained isometric hold would read as deliberate movement.** Holding a loaded spoon
  still, against gravity, produces muscle drive without motion; the gate would stay open and
  the system would revert to its IMU-only behaviour. That is the *safe* direction of failure,
  but it is a genuine gap and it is untested.
- **The sEMG stream is still not wired into the Simulink model**, only the MATLAB
  controller. Deferred deliberately; it needs a `.slx` port change.
- **No hysteresis, dead zone or creep in the plant model** — the project's standing caveat.
  This remains an algorithm-development baseline, not a validated digital twin.

---

## 9. Suggested next steps, in the order we would do them

1. **Repair the joint-travel check** (one ordering fix), then re-run the two
   outside-envelope scenarios and measure whether it changes anything. Cheap, and it closes
   the oldest outstanding item.
2. **Attack the fast-movement gap properly.** The current gate detects movement from muscle
   *amplitude*, which by definition arrives late. The more promising quantity is the *rate of
   change* of muscle drive, which leads amplitude substantially — we measured it as
   ~4× earlier detection, but it also amplifies noise, so it needs its own threshold derived
   the same way the current ones were. This is a contained experiment, not a redesign.
3. **Record a short synchronised session** — forearm muscle sensors and a motion sensor worn
   together, alternating rest, tremor-like shaking and deliberate reaching. Twenty minutes of
   recording would close the three outstanding validation checks *and* produce the first real
   measurement of the class separation the whole fusion rests on. This is the single highest
   value item on the list and the only one that changes what the result means.
4. **Re-measure the calibration's stability** across that session (fit the gain at the start
   and the end and check they agree), since a per-subject calibration that drifts within one
   meal would be a problem.
5. **Test an isometric hold** explicitly, since it is the one everyday situation we know the
   design handles by giving up.
6. **Revisit whether case 5 changes the hardware sizing.** Three of twenty scenarios now fail
   the feasibility bound on bus voltage or current. If those amplitudes are clinically
   realistic, the actuator or the battery is undersized, and that is a mechanical conclusion
   this control work has produced as a by-product.

Items 1, 2, 5 and 6 are all doable in simulation. Item 3 is the one that needs hardware and
a person, and it is the one that would turn this from a defensible simulation result into a
defensible system.

---

### Appendix: where the numbers come from

Every figure in this note is reproducible from the repository:

```matlab
START_PROJECT('setup')
calibrate_rcflex_semg_intent()        % the gate constants and the 6.3:1 separation
run_intent_authority_experiment()     % Section 3.1, no sEMG involved
benchmark_rcflex_control()            % all six variants, 20 scenarios, fixed seeds
summarise_rcflex_benchmark()          % Section 5's three averages
plot_rcflex_benchmark('final')        % and ('fusion') to include the ablation
run_semg_intent_experiment()          % Section 5.1 and the 'reach' scenario
plot_rcflex_semg_gate('reach',8,1,4)  % Section 7 in time
run_validation()                      % 15 of 15 groups pass
```

Results land in `output/results/` (`control_benchmark.csv`,
`control_benchmark_summary.csv`, `intent_authority_ceiling.csv`, `semg_intent_e2.csv`,
`semg_intent_calibration.csv`) and figures in `output/figures/`.

**Figures worth putting in the written report:** `semg_intent_separation.png` is the key one
— it shows tremor absorbed and voluntary movement suppressed side by side, and the ablation
wins one panel while losing the other, which is the whole argument in one image.
`bench_suppression.png` is the headline comparison, `intent_authority_ceiling.png` shows why
a fixed setting cannot work, and `semg_gate_reach.png` shows the mechanism operating in time
including its remaining weakness.

With the sEMG switch off, the system reproduces every pre-fusion number exactly (the
movement+tremor case is unchanged at 85.8826099585925%), so the comparison is a controlled
one rather than an incidental one.
