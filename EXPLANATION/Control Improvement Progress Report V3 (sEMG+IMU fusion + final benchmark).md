
# Control Improvement Progress Report V3
### sEMG + IMU fusion, and the final benchmark
Revision 1: 17 September 2026 — the fusion is built, benchmarked and shipped.
**Revision 2: 18 September 2026 — final validation and packaging (Sections 8–13 are new).**

This is an internal status note, not a polished deliverable. It summarises where the
"smarter tremor control" work stands: what's built, what's still open, and what tripped us
up. Kept light on the maths on purpose — the technical detail lives in
`docs/SEMG_INTENT_FUSION.md` (the main reference for the fusion itself),
`docs/CONTROL_IMPROVEMENT_PLAN.md`, `docs/SEMG_IMPLEMENTATION.md`, `docs/SEMG_DATASET.md`,
and — new in revision 2 — `docs/VALIDATION_STATUS.md` and
`docs/SEMG_PUBLIC_DATA_SURVEY.md`.

**What changed in revision 1.** V2 ended with sEMG validated as a signal source but not
connected to the control loop at all, and with the EKF not recommended as a replacement on
the evidence then available. Both of those changed. sEMG is fused into the estimator, the
combined system is the best arm on the benchmark by every measure we track, and the fusion
works through a mechanism V2 did not anticipate. Sections 3–6 cover that. Section 2 closes
out V2's pending list item by item, and Section 7 is an honest account of a failure we
introduced, found, partly fixed, and have **not** fully solved.

**What changed in revision 2.** No change to the shipped control algorithm — deliberately.
This revision closed the project's oldest outstanding defect, tested the one design idea
revision 1 recommended, took the sEMG assumptions to real human data as far as public
datasets allow, and made the whole evidence base reproducible from a single command. Three
of the four outcomes are negative, and they are the more useful ones:

- **Section 8 — the joint-travel check is repaired, and it changes nothing.** Re-running all
  five EKF arms over the full benchmark moved **zero of 960 metric cells**. The current
  branch of the same limiter was already binding first at every amplitude that matters.
- **Section 9 — the rate-of-change detector this report recommended does not work, and the
  measurement behind that recommendation is withdrawn.** The rate observable separates the
  two classes *the wrong way round*, because the rate of muscle drive is largest for the
  fastest thing in the signal and in a tremulous hand that is the tremor. No candidate met
  both criteria declared in advance; nothing was adopted and the 350 ms target was not
  moved.
- **Section 10 — real forearm sEMG says the synthetic 6.3 : 1 separation does not transfer
  in one of the two contraction directions.** Flexion separates 5–10 : 1 as designed;
  extension reads 0.5–2.5 : 1. Two mechanisms absent from the synthetic model explain it: a
  standing DC offset in the differential envelope and genuine agonist/antagonist
  co-activation. A systematic survey also confirms that no public dataset pairs forearm sEMG
  with a measured tremor, so four of the six validation stages cannot be closed without our
  own recording.
- **Section 11 — `START_PROJECT` is now the single entry point**, and Section 12 is the
  honest list of what remains.

**One-line summary.** The corrected IMU-only estimator averaged 92.1% cancellation on the
scenarios the hardware can actually serve, and met the 90% target on 11 of 16. Adding sEMG
takes that to **93.0% and 15 of 16**, while keeping voluntary-motion error at 0.207°
(against 0.458° for the original controller) — and we can show by direct ablation that the
sensor, not the mechanism, is what buys it. Those figures are **simulation results**. The
sentence the project is entitled to write is that *the control algorithm was validated in
simulation, and public real-data analysis was used to test the sEMG assumptions it rests on;
physical closed-loop human validation was outside the scope of this project.*

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
| Fix the inoperative joint-travel check in the authority limiter | **Open at revision 1; CLOSED in revision 2** — repaired, regression-tested, and measured to change nothing. Section 8 |
| Decide what to do about the single-frequency limitation | **Resolved.** The answer is not more oscillators; it is a second sensor. Section 3 |
| sEMG validated but not fused into the control loop | **Resolved.** Sections 3 and 5 |
| Three of six real-data sEMG checks unanswered | **Still open**, and revision 2 established that no public dataset can answer them. Section 10 |
| Final side-by-side comparison report | **Done.** `output/results/CONTROL_RESULTS.md`, figures, and this note |

Five of seven closed at revision 1. Of the two that remained, the code repair was done in
revision 2 and measured to change nothing (Section 8). The recording session is still
outstanding, and is now the *only* outstanding item that would change what the result
means (Section 10).

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

The real-data validation is deliberately kept as a separate stage with its own gate.
Revision 2 took that stage as far as public data allows, and the answer was not the
reassuring one: see Section 10.

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
achieves 404 ms at the seed this section used (407 ms averaged over four seeds). A 16-point
sweep over the filter settings found nothing meeting both that target and the separation
requirement — so **we adopted none of them, rather than moving the target to whatever the
design happened to achieve.** The headline numbers in Section 5 do not reflect this cost,
because the benchmark contains no such scenario. Anyone quoting the headline should quote
this alongside it.

**Revision 2 followed this up properly.** That informal sweep has been replaced by a
reproducible experiment over three detector families and four corner frequencies, and it
settles the question in the negative for a reason that is structural rather than a matter of
tuning. See Section 9.

**One more correction.** At one point this revision recorded that the two versions of the
safety-layer fix differed by 20 points. That comparison had been run at two different sensor
seeds and was simply wrong; re-run properly, they are identical. The erroneous claim is
struck through in the code comment and the design document rather than deleted, because a
retracted measurement is itself a useful warning.

---

## 8. The joint-travel check: repaired, and measurably inert

*Added in revision 2. This was the oldest item on the list — V2 diagnosed it, V3 left it
open, and it is now closed.*

**The defect.** The authority limiter derates control authority when the tremor correction
it predicts over the next 20 ms would exceed either the coil current limit or the
mechanism's joint travel. The travel half never worked. The predicted angle was clipped to
`C.joint_limit` *before* being compared against `C.joint_limit`, so the comparison could not
fail for any input. V2 probed it with 3,000 randomised inputs and it derated authority in
zero of them.

**The repair** is one line of ordering: the peak is now taken from the raw predicted
correction, while the clipped angle is still what the inverse-dynamics helper sees, because
current can only ever be spent on a reachable angle. The intended control law is unchanged.

**The regression test has teeth.** `tests/test_rcflex_authority_manager.m` asserts that the
branch fires on an over-travel demand, that the derate is *not* explainable by the current
branch alone (attributed using the predicted current the function itself returns, so the
test does not re-implement the check), that it stays open on an in-travel demand, and that
over 400 randomised probes it never derates an in-travel one. The defect was deliberately
reintroduced to confirm the test fails — it does, with
`Joint-travel branch did not trigger on a 40 deg demand (gamma=1.0000)`.

**And now the part that matters: it changes nothing.**

All five EKF arms were re-run over the full 20-case benchmark with the repair in place. The
result was compared cell by cell against the pre-repair CSV, which is preserved as
`output/results/control_benchmark_before_joint_fix.csv`:

> **Zero of 960 metric cells differ.** Suppression, attitude RMS, joint peak, current peak,
> stroke, power, side load and the within-limits flag are all bit-identical across 120 rows.

A dedicated sweep to 24° of tremor (`START_PROJECT('jointtravel')`) shows why:

| Tremor amplitude, 8 Hz | 1° | 3° | 5° | 8° | 11° | 14° | 18° | 24° |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| minimum authority γ, check **off** | 1.000 | 1.000 | 0.878 | 0.738 | 0.507 | 0.143 | 0.000022 | 0.00116 |
| minimum authority γ, check **on** | 1.000 | 1.000 | 0.878 | 0.738 | 0.507 | 0.143 | 0.000022 | 0.00093 |
| peak joint excursion | 1.1° | 3.3° | 18.4° | 20.5° | 20.9° | 21.6° | 22.2° | 23.9° |

The **current** branch is the binding constraint at every amplitude that matters, and it has
already collapsed authority to near zero by the time the travel branch would engage. On the
minimum-authority criterion the travel branch first binds at 24° of tremor — roughly five
times anything physiological — where it moves γ from 0.00116 to 0.00093 and changes
suppression not at all. (At 18° it moves the *mean* authority fractionally, 0.0919 to
0.0916, while leaving the minimum untouched; the effect is real but too small to register
in any performance metric.)

**The conclusion is worth stating plainly, because it is not the one we expected.** The
protection that was missing was already covered by a different protection. The repair is a
correctness fix with no performance consequence, and the only way to know that was to
measure it rather than assume the fix would help the worst case. Note also that the worst
case — benchmark case 14, which reaches 20.2° against a 15° travel limit — is driven there
by *voluntary* reference tracking, not by the tremor correction the branch inspects, so this
branch was structurally incapable of protecting it.

Sixteen of the twenty-two sweep rows breach the hardware envelope; their suppression
percentages describe the end-stop model rather than the controller, and the CSV carries
`within_limits` and `hardware_feasible` per row so that is auditable rather than implied.

---

## 9. Fast voluntary movement: the rate detector, tested properly

*Added in revision 2. This replaces the informal sweep referred to in Section 7 with a
reproducible experiment, and it overturns the recommendation V3 made.*

Section 7 left the fast-reach gap as the largest remaining weakness, and V3's next-steps
list recommended a **rate-of-change detector** on the grounds that muscle drive rises before
it is large. That recommendation was built on an informal look at a single trace, quoted as
"~4× earlier detection". It does not survive a proper test.

**The experiment (E3, `START_PROJECT('fastmove')`).** Three arms, selected by one switch,
`C.semg_gate_detector`:

| Arm | Early-detection path |
|---|---|
| **A `amplitude`** | the shipped two-speed gate. Reference. |
| **B `rate`** | the fast amplitude path replaced by the derivative of the same filtered drive |
| **C `hybrid`** | both early paths, combined with `max()` |

The slow amplitude path is common to all three and is the only path that may ever *close*
the gate, so the arms differ in early detection and nothing else. Every rate threshold is
derived by the **same geometric-mean rule** that produced the two amplitude floors, over the
same 20 benchmark cases, with the `reach` scenario **held out** so no threshold is fitted to
the case it is judged on. The corner frequency is swept and every point is reported.

The criteria were the project's existing ones, declared before the run: **latency < 350 ms**
*and* **confidence ≤ 0.10 on tremor-only** *and* **≥ 0.50 on intent-bearing**.

### 9.1 The rate observable separates the classes the wrong way round

| Rate corner | tremor-only max | intent-bearing min | **separation** |
|---|---:|---:|---:|
| 2 Hz | 6.50 °/s | 4.49 °/s | **0.69 : 1** |
| 4 Hz | 21.80 °/s | 5.95 °/s | **0.27 : 1** |
| 8 Hz | 45.96 °/s | 6.57 °/s | **0.14 : 1** |
| 16 Hz | 52.23 °/s | 6.88 °/s | **0.13 : 1** |

Against a requirement of ≥ 4 : 1, every one of these is not merely low but **inverted** —
the tremor-only class reads *higher* than the intent-bearing class. That is not a tuning
problem. No threshold can fix an observable that ranks the two classes backwards.

The reason is obvious once stated, and it is the reason the informal measurement misled us:
**the rate of muscle drive is largest for the fastest thing in the signal, and in a
tremulous hand that is the tremor.** The 20 benchmark cases drive voluntary motion at
0.2–0.25 Hz and tremor at 4–20 Hz, so the derivative belongs almost entirely to the tremor.

### 9.2 Could any threshold work on a fast movement?

That is the narrower question the inversion forces, so it was asked directly: on the fast
reach the detector exists for, does the rate rise above the worst tremor-only rate?

| Rate corner | worst tremor-only | reach peak (mean of 4 seeds) | ratio | separable on **every** seed |
|---|---:|---:|---:|:---:|
| 2 Hz | 6.50 °/s | 15.16 °/s | 2.33 | **yes** |
| 4 Hz | 21.80 °/s | 22.01 °/s | 1.01 | no |
| 8 Hz | 45.96 °/s | 42.96 °/s | 0.93 | no |
| 16 Hz | 52.23 °/s | 53.33 °/s | 1.02 | no |

Only at a 2 Hz corner does a separating threshold exist at all. At 4 Hz and above, none
exists at any value. This diagnostic deliberately chooses no threshold: picking one from the
reach would be fitting the detector to the case it is judged on.

### 9.3 What each arm actually did

| Arm | latency (mean, 4 seeds) | tremor conf | intent conf | open on tremor | separation | latency |
|---|---:|---:|---:|---:|:---:|:---:|
| **A amplitude (shipped)** | **407 ms** (404–418) | **0.093** | 0.811 | 0.0 % | **PASS** | FAIL |
| B rate 2 Hz | **349 ms** (280–384) | 0.343 | 0.811 | 0.0 % | FAIL | **PASS** |
| B rate 4 Hz | 387 ms (256–530) | 0.427 | 0.811 | 15.7 % | FAIL | FAIL |
| B rate 8 Hz | 1 ms | 0.666 | 0.811 | **100 %** | FAIL | PASS |
| B rate 16 Hz | 1 ms | 0.758 | 0.811 | **100 %** | FAIL | PASS |
| C hybrid 2 Hz | 349 ms (280–384) | 0.343 | 0.811 | 0.0 % | FAIL | **PASS** |
| C hybrid 4 Hz | 359 ms (256–418) | 0.427 | 0.811 | 15.7 % | FAIL | FAIL |
| C hybrid 8 / 16 Hz | 1 ms | 0.666 / 0.758 | 0.811 | **100 %** | FAIL | PASS |

The 8 and 16 Hz arms are degenerate: a 1 ms "latency" and a gate that is open the entire
time on a tremor-only record is not a detector, it is a gate that has stopped gating.

The interesting near-miss is **B rate at 2 Hz**. It genuinely reaches 349 ms — it meets the
latency target the shipped design misses — and it never actually crosses the open threshold
on a tremor-only record. But its *mean* confidence there is 0.343 against the ≤ 0.10 the
mechanism needs, which means it would reduce the intent process noise to about a third
rather than to a fiftieth on a tremor-only case. That is most of the `q_intent` swing the
whole gate exists to deliver, given away to buy 58 ms.

### 9.4 Verdict: nothing adopted, target not moved

**No candidate met both criteria.** Every arm that reached the latency target lost the
separation the fusion depends on; the only arm that kept the separation is the one already
shipped. The shipped amplitude gate is retained and the 350 ms target stands where it was
declared.

The cost therefore stands too, measured over four seeds in closed loop:

| | `reach` attitude error | tremor suppression, tremor-only cases |
|---|---:|---:|
| IMU-only, no gate | **0.289°** | 91.64 % |
| Shipped sEMG gate | **0.697°** | 92.26 % |

Roughly 2.4× worse on a fast deliberate movement, for about 0.6 points of tremor
suppression. **The headline numbers in Section 5 do not include this cost**, because the
20-case benchmark contains no fast voluntary movement. Anyone quoting 93.0 % should quote
this alongside it.

---

## 10. Real-data validation, within the limits of what is public

*Added in revision 2, in place of the human recording session, which was out of scope.*

### 10.1 The survey: the recording we need does not exist publicly

Eleven candidate sources were inspected against what each of the six validation stages
physically requires — not from citations, but from the record pages and, where the data was
in hand, from the signals themselves. The full table is in
`docs/SEMG_PUBLIC_DATA_SURVEY.md`. The conclusion:

> **No public dataset records forearm sEMG simultaneously with a measured tremor.** Every
> tremor dataset found (Oxford ET/DBS, Zenodo 7273759, Zenodo 10782573 ALAMEDA, Mendeley
> `fd6sdp6t7x`, PhysioNet PD rest-tremor velocity, IEEE DataPort) measures tremor with an
> inertial sensor and records **no muscle signal at all**. The nearest published match — 5
> patients, wrist flexor/extensor at 2 kHz with synchronised accelerometry — is private,
> with no repository or availability statement. The Oxford TMS recording that would be an
> exact match is an ongoing clinical trial, not a released dataset.

That is a gap in the field's data sharing, not a gap in the searching, and it is what keeps
stages 2, 5 and 6 open regardless of effort spent here.

**One candidate was identified and deliberately not run.** NinaPro DB7 (22 subjects,
12-channel sEMG at 2 kHz with a synchronised 9-axis IMU on the same Trigno sensors) could
answer a *modified* stage 4 — does sEMG lead *voluntary movement onset*, i.e. the 40 ms
electromechanical delay the whole fusion premise rests on. It was not run for three reasons,
all scope rather than science: it sits behind a registration wall that is a person's
decision to accept, its movements are unloaded finger and wrist gestures so the delay it
measures would need its own generalisation argument, and its no-derivatives licence means a
converted fixture could not be committed for reproducibility. It remains the recommended
next real-data step if a recording session stays out of reach.

### 10.2 What GRABMyo confirmed

Unchanged and still standing: acquisition health **passes** on both participants with
unmodified thresholds, and raw cross-talk **fails for one of two** (0.460 against a 0.30
provisional gate), which established that *placement, not physiology*, is the binding
constraint. Two instrumentation facts simulation could not have produced: real amplitudes
are 16.6–130 µV rms against the synthetic model's ~700 µV, and real crest factors are
11.5–23.2 against the 7–10 assumed.

### 10.3 The new analysis, and the problem it found

The 6.3 : 1 class separation cannot be measured on public data, because that needs a tremor
class. What *can* be measured is **voluntary contraction against quiescent rest**, which
bounds the intent/tremor separation **from above** — tremor is real muscle activity, rest is
its absence, so a tremor segment can only sit higher than a rest one. Run through the
shipped gate front end, by the same worst-case geometric-mean rule the synthetic floor uses
(`START_PROJECT('realdata')`):

| Condition, roll axis | P1 | P2 | Synthetic | Target |
|---|---:|---:|---:|---:|
| Wrist **flexion** vs rest | 10.1 : 1 | 5.1 : 1 | — | ≥ 4 : 1 |
| Wrist **extension** vs rest | 2.5 : 1 | **0.46 : 1** | — | ≥ 4 : 1 |
| **Pooled worst case** (the declared rule) | **0.8 : 1** | **0.1 : 1** | 6.3 : 1 | ≥ 4 : 1 |

**Flexion behaves exactly as designed. Extension does not, and the pooled rule therefore
fails.** Two mechanisms are visible in the data, and neither exists in the synthetic source:

1. **A standing DC offset in the differential envelope.** At rest, agonist minus antagonist
   should be ≈ 0. Measured, it is **7–20 % of a strong contraction's drive**,
   subject-specific, and the rectify-and-lowpass envelope passes it straight through the
   gate's 1 Hz filter. The synthetic source generates both channels with identical
   statistics, so its resting difference really is ≈ 0.
2. **Agonist/antagonist co-activation** of 0.63–0.78 on these same recordings: the two
   muscles rise *together*, so their difference cancels much of the drive whenever the
   antagonist leads. The synthetic source generates its channels independently, so its
   reciprocity is perfect by construction.

The obvious hypothesis — that the offset alone explains it — **was tested and rejected**.
Subtracting the measured resting baseline lifts the pooled ratio from 0.8 to 1.1 and from
0.1 to 1.9: real improvement, still far short of 4 : 1. The residual is the co-activation.

**What this does and does not establish.** It is not a refutation of the fusion: these are
hand gestures rather than wrist stabilisation, the channel pairs are selected empirically,
and the pitch pair is an acknowledged substitute. It *does* establish, on real muscle, that
the synthetic 6.3 : 1 is **not reproduced in one of the two contraction directions**, and it
names two specific mechanisms the model omits. Neither is fixed here: referring the gate
floor to a measured resting baseline and handling co-activation are architectural changes,
and the experiment that would justify them — a real subject, with tremor — has not been run.

### 10.4 The validation status is now written down in one place

`docs/VALIDATION_STATUS.md` separates the evidence into four tiers — algorithmic simulation,
public real-data, hardware (there is none), and remaining assumptions — and states what each
tier's numbers are allowed to claim. The sentence the project is entitled to write is:

> The control algorithm was validated in simulation, and public real-data analysis was used
> to test the sEMG assumptions it rests on. Physical closed-loop human validation was
> outside the scope of this project.

Stronger wording than that is not supported.

---

## 11. One command to reproduce any of it

*Added in revision 2.* Reproducing the evidence previously meant remembering which eight
functions to call in which order. `START_PROJECT` is now the single entry point; the
underlying functions stay individually callable for debugging.

```matlab
START_PROJECT('setup')        % paths, output folders, Simulink cache
START_PROJECT('test')         % every unit and regression test          ~25 s
START_PROJECT('realdata')     % public-dataset sEMG validation          ~30 s
START_PROJECT('figures')      % every report figure, from saved results ~1 min
START_PROJECT('jointtravel')  % Section 8                               ~8 min
START_PROJECT('fusion')       % calibration + the E2 experiment         ~27 min
START_PROJECT('fastmove')     % Section 9                               ~27 min
START_PROJECT('benchmark')    % the 6-arm sweep, summary, figures, report ~70 min
START_PROJECT('all')          % everything, in dependency order         ~2 h
START_PROJECT('all','DryRun',true)   % list the steps without running them
```

Each mode prints a progress line per step with its timing, stops on the first failure rather
than leaving a half-written CSV that looks finished, and ends by listing exactly which files
it wrote and where. `'figures'` rebuilds every report figure from the CSVs already on disk,
so a figure can never be regenerated from a different run than the table beside it.

---

## 12. What's still open

Revision 2 closed two of the six items V3 listed here. What remains:

- **Fast deliberate movement.** Still the largest remaining weakness of the delivered
  system, and Section 9 has now established that it is not fixable by swapping the detector:
  the alternative observable is anti-discriminative. 0.697° against 0.289° for IMU-only.
- **Stages 2, 4, 5 and 6 of the real-data gate remain unanswered**, and Section 10.1 has
  established that no public dataset can answer them. This still gates the one number the
  whole fusion depends on. Stage 4 has a named candidate (NinaPro DB7) that would need a
  registration a person must accept.
- **The differential envelope does not separate both contraction directions on real
  muscle** (Section 10.3). New this revision, measured rather than assumed, and unfixed. The
  two mechanisms — a standing DC offset and real co-activation — are both absent from the
  synthetic model, which means the synthetic figures are optimistic in a way that was not
  previously visible.
- **`C.semg_emd_s = 40 ms`, the electromechanical delay**, is the entire reason sEMG is
  worth adding and still rests on the literature rather than on a measurement made here.
- **A sustained isometric hold would read as deliberate movement.** Unchanged, untested, and
  still the safe direction of failure.
- **The sEMG stream is still not wired into the Simulink model**, only the MATLAB
  controller. Deferred deliberately; it needs a `.slx` port change.
- **No hysteresis, dead zone or creep in the plant model** — the project's standing caveat.
  This remains an algorithm-development baseline, not a validated digital twin.
- **No hardware of any kind.** No prototype was built, no bench measurement taken, no human
  recorded. Every actuator, coil and flexure parameter comes from a manufacturer data sheet
  applied to a structure that was never made.

---

## 13. Next steps, in the order we would do them

1. ~~**Repair the joint-travel check**, then measure whether it changes anything.~~
   **DONE (Section 8).** Repaired, regression-tested, and it changes nothing — zero of 960
   metric cells. The current branch was already binding first.
2. ~~**Attack the fast-movement gap** with a rate-of-change detector, which leads amplitude
   substantially — we measured it as ~4× earlier detection.~~
   **DONE, and the premise was wrong (Section 9).** The rate observable separates the two
   classes inverted. The "~4× earlier detection" came from an informal look at a single
   trace and does not survive the project's own threshold derivation. Nothing was adopted and
   the target was not moved. The struck-through text is kept because a retracted measurement
   is itself a warning.
3. **Record a short synchronised session** — forearm muscle sensors and a motion sensor worn
   together, alternating rest, tremor-like shaking and deliberate reaching. Twenty minutes
   would close stages 2, 4, 5 and 6 *and* produce the first real measurement of the class
   separation the whole fusion rests on. It is now also the only way to find out whether the
   extension/co-activation problem of Section 10.3 afflicts tremor as well. **This remains
   the single highest-value item and the only one that changes what the result means.**
   Section 10 has raised its priority rather than lowered it.
4. **Site the electrodes on opposite aspects of the forearm and verify cross-talk
   < 0.30 before recording the tremor trials.** Cheap, and it is a direct consequence of one
   of two GRABMyo participants failing that gate.
5. **Decide what to do about the resting DC offset** (Section 10.3). Referring the gate floor
   to a measured per-subject, per-session resting baseline is a small change with a clear
   rationale, but it should be justified by the recording in item 3 rather than fitted to
   GRABMyo, which has no tremor to check it against.
6. **Test an isometric hold** explicitly, since it is the one everyday situation we know the
   design handles by giving up.
7. **Revisit whether case 5 changes the hardware sizing.** Three of twenty scenarios fail the
   feasibility bound on bus voltage or current. If those amplitudes are clinically realistic,
   the actuator or the battery is undersized — a mechanical conclusion this control work has
   produced as a by-product.

Items 4, 6 and 7 are doable without new hardware. Item 3 is the one that needs a person, and
it is the one that would turn this from a defensible simulation result into a defensible
system.

---

### Appendix: where the numbers come from

Every figure in this note is reproducible from the repository with the entry point in
Section 11:

```matlab
START_PROJECT('setup')
START_PROJECT('benchmark')    % Sections 4, 5 and 8's re-run
START_PROJECT('fusion')       % Sections 3 and 5.1, and the 6.3:1 separation
START_PROJECT('jointtravel')  % Section 8
START_PROJECT('fastmove')     % Section 9
START_PROJECT('realdata')     % Section 10
START_PROJECT('test')         % 16 of 16 validation groups pass
START_PROJECT('figures')      % every figure below, from the saved CSVs
```

Results land in `output/results/` and figures in `output/figures/`:

| Result file | Section |
|---|---|
| `control_benchmark.csv`, `control_benchmark_summary.csv`, `CONTROL_RESULTS.md` | 4, 5 |
| `control_benchmark_before_joint_fix.csv` *(preserved pre-repair baseline)* | 8 |
| `joint_travel_authority.csv` | 8 |
| `fast_movement_thresholds.csv`, `fast_movement_separability.csv`, `fast_movement_detectors.csv`, `fast_movement_closed_loop.csv` | 9 |
| `semg_dataset_validation*.csv`, `semg_separation_real*.csv` | 10 |
| `semg_intent_calibration.csv`, `semg_intent_e2.csv`, `intent_authority_ceiling.csv` | 3, 5.1 |

**Figures worth putting in the written report.** `semg_intent_separation.png` is still the
key one — it shows tremor absorbed and voluntary movement suppressed side by side, and the
ablation wins one panel while losing the other, which is the whole argument in one image.
`bench_suppression.png` is the headline comparison and `intent_authority_ceiling.png` shows
why a fixed setting cannot work. Three are new in revision 2:
`fast_movement_detectors.png` plots latency against class separation with both declared
criteria drawn as lines, so the reader can see that the admissible corner of that plot is
*empty*; `joint_travel_authority.png` shows the two authority curves lying on top of each
other, which is the whole of Section 8 in one image; and `semg_separation_real.png` shows
the real flexion/extension/rest drive distributions that Section 10.3 reads.

With the sEMG switch off, the system reproduces every pre-fusion number exactly (the
movement+tremor case is unchanged at 85.8826099585925%), and the joint-travel repair
reproduces every pre-repair number exactly, so both comparisons are controlled ones rather
than incidental ones.
