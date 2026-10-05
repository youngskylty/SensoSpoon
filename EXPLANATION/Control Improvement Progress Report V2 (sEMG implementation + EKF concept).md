
# Control Improvement Progress Report V2
### sEMG implementation + EKF concept
Date: 16 September 2026 (revised — adds the full benchmark run and the failure diagnosis)

This is an internal status note, not a polished deliverable. It summarises where the
"smarter tremor control" work stands: what's built, what's still open, and what tripped us
up along the way. Kept light on the maths on purpose — the technical detail lives in
`docs/CONTROL_IMPROVEMENT_PLAN.md`, `docs/SEMG_IMPLEMENTATION.md` and `docs/SEMG_DATASET.md`
for anyone who wants it.

**What changed in this revision.** The full 20-scenario benchmark has now been run, which
was the outstanding milestone in the previous version. It produced a more complicated
answer than expected, and following that up turned into the main body of new work: two
of the worst-looking failures turned out not to be control problems at all, and the
estimator's remaining weakness was traced to a specific structural cause. Sections 4, 5
and 6 are new. Section 8 has been corrected — the previous version said two safety layers
were not yet wired in, and that was wrong.

---

## 1. The problem we're trying to fix

The spoon's current controller assumes it already knows the tremor frequency and filters
out a fixed band (roughly 4–12 Hz). That works well when the assumption holds — about 93%
tremor cancellation on average. But it breaks in two specific, and important, situations:

- When the person makes a deliberate hand movement (e.g. reaching for the plate) **at the
  same time** as tremor, the controller gets confused and cancellation drops to ~51%.
- When tremor sits outside the assumed 4–12 Hz band, or the hand moves suddenly, the
  controller actively makes things **worse than doing nothing** — in one test case, -197%
  (i.e. it amplifies the shake rather than cancelling it).

So while the headline number looks good, the honest average across all realistic test cases
is closer to 64%, and it can fail badly in exactly the moments that matter most in real use.

The full benchmark has now confirmed both of these numbers precisely (64.19% average,
-196.7% worst case), so this section is measured rather than estimated.

## 2. Why we're introducing an EKF (in plain terms)

An **EKF (Extended Kalman Filter)** is a mathematical technique for estimating something you
can't measure directly — here, "is this movement tremor or is it the person's own
intention?" — by continuously updating a best guess as new sensor data comes in, rather than
relying on a fixed, pre-set filter.

Two things made it look like a better fit than the current fixed-band filter:

1. **It adapts.** Instead of assuming tremor is always 4–12 Hz, it estimates the actual
   tremor frequency in real time, so it can handle a wider range of tremor speeds and adjust
   automatically if it changes.
2. **It knows when it's unsure.** An EKF naturally produces a "confidence" signal alongside
   its estimate. When that confidence drops, we can have the system fall back to "just
   follow the hand" rather than fight it.

We looked at simpler adaptive-filter alternatives used in tremor research (WFLC, BMFLC) but
picked the EKF specifically for that self-confidence signal.

**An important caveat has since emerged on point 2.** We have now measured that the
confidence signal cannot detect the estimator's actual failure mode (Section 5.3). That
doesn't invalidate the EKF choice, but it does remove one of the two original reasons for
making it, and that should be stated plainly.

## 3. What's done so far

**EKF core, built and working standalone.**
The estimator — a 13-variable model that separately tracks "intended movement" and "tremor"
from the handle's motion sensor — is implemented and passes its own correctness tests
(frequency tracking, stability, repeatability).

**A real bug found and fixed in that estimator.**
While investigating why the EKF struggled on the "voluntary movement + tremor together"
test case, we traced it to a genuine coding defect: the estimator's internal "best guess"
for hand position and its internal "uncertainty" about that guess were being updated on two
different, inconsistent assumptions. Once corrected, that specific failure case improved
from 30% cancellation to ~86% — now confirmed on the full benchmark, not just in isolated
testing.

**A second, independent sensor channel investigated: sEMG.**
sEMG (surface electromyography) measures the electrical signal produced by forearm muscles.
The idea: muscle activity happens *before* the hand physically moves, so it could give the
controller earlier warning. We built a simulated sEMG signal generator, a realistic sensor
front-end model, and a full processing pipeline extracting muscle effort level and rhythmic
tremor-frequency activity. On simulated data the muscle signal's tremor content matched the
true tremor frequency closely and led the physical shake by roughly 35 ms.

**Validated against a real, published sEMG dataset (not just simulation).**
We ran the pipeline against real human forearm recordings (GRABMyo, University of Waterloo,
via PhysioNet). Basic signal quality checks pass on real data. The question of whether two
nearby sensors "bleed into" each other (cross-talk) was tested directly: it depends heavily
on placement — on opposite sides of the forearm it's acceptable, close together it isn't.
That finding will directly shape the physical sensor layout.

**Tested whether sEMG improves the controller — result: not yet.**
Feeding the sEMG-derived tremor frequency into the EKF did not help. Even with a *perfect*
frequency estimate the hardest failure case improved by only ~8 points, and the sEMG-derived
estimate was biased enough to make several already-working cases worse. A genuine negative
result, not a dead end — see Section 9.

**The full 20-scenario benchmark has now been run** (Section 4), and the remaining failures
have been diagnosed (Section 5).

## 4. What the full benchmark showed

All controller variants were run over the same 20 scenarios with identical sensor seeds, so
the only difference between them is the algorithm. This is a controlled comparison, not a
collection of hand-picked cases.

| | current controller | corrected EKF |
|---|---|---|
| average cancellation | 64.2% | 71.0% |
| average, excluding invalid cases | 90.2% | 91.8% |
| cases meeting the 90% target | **15 of 20** | 11 of 20 |
| cases within hardware limits | 18 of 20 | 18 of 20 |
| worst case | -196.7% | -119.4% |

**The corrected EKF fixes what it was meant to fix.** On the "voluntary movement + tremor"
case, cancellation went from 51.5% (current controller) to 85.9%. On the tremor-free
voluntary-movement case, tracking error dropped from 0.46° to 0.20° — better than the
current controller. It also clearly beats the current controller whenever the tremor sits
*outside* the assumed 4–12 Hz band, which is the other original failure mode: at 5 Hz it
reaches 98.3% against 91.4%.

**But it does not yet earn the right to replace the current controller.** Two honest
qualifications:

- The average favours the EKF, but that average is misleading. It is dominated by one
  scenario where *both* controllers fail catastrophically and the EKF merely fails less
  badly. On the straightforward count of "how many scenarios actually met the 90% target",
  the current controller still wins, 15 to 11.
- Within the assumed tremor band — the conditions the current controller was designed for —
  the current controller is still slightly ahead (94.0% against 93.5%).

So the fair summary is: **the EKF is better where the old controller was weak, and slightly
worse where it was strong.** That is a real improvement in robustness, but it is a trade,
not a clean win, and it is not yet enough to justify switching the default.

There is also one genuine regression, on two scenarios with "messy" multi-frequency tremor,
where the EKF loses about 4–5 points against the current controller. Diagnosing that led to
the main finding of this revision (Section 6).

## 5. Two failures that turned out not to be control problems

Two scenarios produce alarming numbers (-117% and -197%). We investigated both, and neither
is an estimator or control-logic fault.

### 5.1 The "sudden movement" case is an actuator limit, not a control error

This scenario is not what its name suggests. It doubles the *tremor amplitude* from 5° to
10° partway through, rather than commanding a sudden voluntary movement. Cancelling a 10°
shake at 8 Hz is simply beyond what this hardware can do:

- the joint is driven **5.2° past its own physical end stop** (20.2° against a 15° limit)
- the actuator slide exceeds its travel (6.65 mm against 6.35 mm)
- average power runs 30% over the continuous rating (5.7 W against 4.4 W)
- the motor-current allocator is saturated **94% of the time**

Meanwhile the estimator's tremor-frequency estimate is *exactly* right (8.00 Hz). There is
nothing wrong with the estimate; the mechanism physically cannot deliver the correction.

### 5.2 The out-of-band case is a bandwidth limit, and only on one axis

At 20 Hz the two axes behave completely differently. One axis still cancels well (it removes
84% of the shake). The other *amplifies* it, because at that speed the mechanism responds
about 133° out of step, so the "correction" arrives nearly in phase with the shake and adds
to it. The only hardware limit exceeded here is average power.

### 5.3 Why the safety layers didn't catch either one

**Correction to the previous version of this report:** it stated that the "confidence
fallback" and "current/power budget" layers were designed but not yet wired in. They *are*
wired in and active. The problem is that they don't fire:

- The **confidence signal never drops** — it sits at its maximum value throughout. It is
  designed to detect "my model doesn't match the sensor data", and in these cases the model
  matches beautifully. The estimator is *right*; the actuator simply can't comply. It is the
  wrong kind of alarm for this kind of failure.
- The **authority limiter only partially engages**, reducing authority to about two-thirds
  when its own calculation implies it should go much lower.
- The part of the authority limiter meant to protect **joint travel is inoperative**. Because
  of an ordering mistake, the value it checks is clipped to the limit before being compared
  against that same limit, so the check can never trigger. We tested it against 3,000
  randomised inputs, including tremor amplitudes far beyond anything realistic: it derated
  authority in **zero** of them. The one limit the worst case violates most badly is the one
  with no working protection.

### 5.4 The scoring for these cases is not meaningful

Once the mechanism is driven past its end stops, the "percentage cancellation" figure stops
describing the controller and starts describing the end-stop model instead. Real hardware
would thermally derate or shut down long before this point. **These scenarios should be
recorded as "outside the hardware envelope" rather than given a percentage**, and excluded
from averages. Changing how they are scored is a pending decision, not something we have
applied.

## 6. The main new finding: why the EKF mis-reads messy tremor

This is the most useful result in this revision, and it is a structural limitation rather
than a bug.

**What we observed.** On the two scenarios with multi-frequency tremor, the estimator
reports large "voluntary movement" that is not there — the true voluntary movement in those
scenarios is exactly zero. The controller then faithfully follows that phantom movement
instead of cancelling it. About 90% of one tremor component was being mis-labelled this way.

**Why.** The estimator tracks **one** tremor frequency at a time. When tremor contains
several components at once, the strongest one captures that single frequency slot, and the
tremor model can then no longer represent anything else. Every remaining component has
nowhere to go except the "voluntary movement" part of the model, which is the only part left
with any freedom.

We confirmed this with a controlled experiment. A small 0.2° tremor component was presented
to the estimator twice: once on its own, and once alongside a strong 8 Hz component.

| small component at | mis-labelled as voluntary, **alone** | mis-labelled as voluntary, **with a strong 8 Hz component present** |
|---|---|---|
| 2 Hz | 62% | 128% |
| 3 Hz | 10% | 88% |
| 4 Hz | 2% | 55% |
| 5 Hz | 0.3% | 34% |
| 6 Hz | 0.0% | 19% |
| 7 Hz | 0.2% | 8% |

On its own, the estimator handles these components almost perfectly. With a stronger
component present, the same signal is misread by up to 30–1900 times more. The effect grows
smoothly the further the component sits from the captured frequency — so the worst errors
land at 2–4 Hz, which is exactly where genuine voluntary movement lives.

**The most important consequence: this failure is undetectable from the inside.** When the
estimator mis-labels 88% of a component, the mismatch between its prediction and the sensor
reading is about 0.00008° — roughly **260 times smaller than the sensor noise** the filter
already expects. It achieves this by splitting the component into two large, opposite pieces
that cancel out almost perfectly, so the overall fit looks excellent while the breakdown into
"tremor" and "intention" is badly wrong.

That explains, in one stroke, why the confidence signal never drops, and why a previously
designed remedy (Section 7) never activates. No self-check based on "does my model fit the
data" can see this problem, because the model *does* fit the data.

## 7. Experiment: adding a second tremor oscillator (negative result)

A second, independent tremor oscillator had already been designed and built for exactly this
situation, sitting behind an off-by-default switch. We enabled it and re-ran the full
20-scenario benchmark.

**It made no difference whatsoever.** Across every valid scenario the largest change was
0.02 percentage points, and the two problem scenarios did not improve at all. The extra
capacity was carried but never used.

The reason follows directly from Section 6. The second oscillator is guarded by a check that
only switches it on when the existing model *fails* to explain the incoming signal — and as
described above, the existing model explains it near-perfectly while getting the breakdown
wrong. The guard and the fault are mutually exclusive by construction.

When we manually forced the guard open as a diagnostic, the second oscillator did correctly
find the 3 Hz component — but recovered only about 17% of the mis-labelled signal. So extra
capacity is **necessary but not sufficient**: the "voluntary movement" part of the model
still wins most of the contested signal, because nothing penalises it for doing so.

## 8. What's still pending

- **Decide whether the EKF replaces the current controller.** The benchmark is now done, but
  the answer it gave is "not yet, on this evidence" (Section 4).
- **Decide how to score scenarios that exceed the hardware envelope** (Section 5.4). This
  needs an agreed definition before implementation.
- **Fix the inoperative joint-travel check** in the authority limiter (Section 5.3). Whether
  it then helps the worst case is a separate question that needs measuring.
- **Decide what to do about the single-frequency limitation** (Section 6). The evidence now
  points away from "add more oscillators" as a complete answer.
- **sEMG is validated as a signal source, but not fused into the control loop at all.**
- **Three of the six real-data validation checks for sEMG remain unanswered**, because no
  public dataset combines forearm muscle signals with a simultaneously recorded physical
  tremor signal.
- **The final side-by-side comparison report** is still to be written.

## 9. Problems / obstacles encountered

**Finding a suitable dataset was harder than expected, and remains only partly solved.**
We need real forearm muscle signals recorded *at the same time* as a real physical tremor
measurement. No public dataset provides all of that together: one promising Oxford dataset
is an unreleased clinical trial; a related published one has motion data but no muscle
signal; the closest published research used private data. We settled on GRABMyo as the best
available substitute, but it contains deliberate hand gestures, not tremor — which is why it
could only validate 3 of our 6 planned checks.

**Two real data-quality bugs found and fixed:** the simulated sensor front end was configured
in a way that would have clipped a real signal, and a filter initialisation bug caused a
spike at the start of every simulated recording that was masking the first problem.

**A "promising-looking" cross-talk result turned out to be measured wrong**, and we walked it
back once it was checked properly against real data.

**The sEMG fusion idea did not pan out on first attempt.** This is now better understood: the
tremor frequency estimate was never the binding constraint, so supplying a better one could
not have helped much.

**Two designed safety features turned out not to work as intended** — one because it measures
the wrong quantity, one because of an ordering mistake that makes it unable to trigger
(Section 5.3). Both were found only because the benchmark forced us to explain specific
numbers.

**The most useful lesson of this revision:** several of our worst-looking numbers were not
control failures at all, and one of our best-looking signals (a near-perfect model fit) was
actively hiding a serious error. Investigating the mechanism behind each number, rather than
tuning against it, is what surfaced both.

## 10. Planned next steps

1. **Decide the scoring definition** for scenarios outside the hardware envelope, then apply
   it and regenerate the comparison table, so the headline averages mean something.
2. **Repair the joint-travel check** in the authority limiter and measure whether it changes
   the worst case.
3. **Address the single-frequency limitation.** The measured evidence says the problem is
   that one dominant component monopolises the tremor model and everything else is pushed
   into "voluntary movement". Any candidate remedy should be validated standalone against
   the Section 6 experiment before being wired into the control loop.
4. **Then re-run the benchmark**, freeze the settings, and confirm reproducibility.
5. **Decide whether the EKF replaces the current controller**, and write the final before/after
   comparison.
6. **For sEMG:** record our own short synchronised session — muscle sensors plus a motion
   sensor worn together — once the physical placement is finalised, to close the three
   outstanding validation checks that no public dataset can answer.
7. **Revisit sEMG fusion last.** Section 6 suggests the useful question is not "what is the
   tremor frequency" but "how many tremor components are there, and which part of the motion
   is voluntary" — a different question from the one the first attempt tried to answer.

---

### Appendix: where the numbers come from

All figures in Sections 4–7 are reproducible from the repository:

- `benchmark_rcflex_control` — runs the 20 scenarios across all controller variants with
  fixed sensor seeds, and writes `output/results/control_benchmark.csv`.
- `plot_rcflex_benchmark` — draws the comparison figures. Defaults to the current controller
  versus the corrected EKF; `plot_rcflex_benchmark('exp1')` adds the second-oscillator arm
  and `('debug')` adds the uncorrected EKF kept for historical comparison.
- `run_omega_capture_experiment` — the Section 6 experiment, writing
  `output/results/omega_capture_experiment.csv` and its figure.

The benchmark was re-run end to end after all changes and reproduced the earlier figures
exactly, so the comparison is reproducible rather than incidental.
