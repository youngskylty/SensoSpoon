# SensoSpoon — sEMG + IMU Fusion: the Intent-Authority Gate

**Status:** implemented, tested and benchmarked. This is the shipped sEMG+IMU architecture.
Experiment tag **E2**. It supersedes E1 (`sEMG → ω`), which is retained in the tree as a
recorded negative result and is still selectable.

**Scope of the claim.** Everything here is an **algorithmic** result obtained in simulation
against a synthetic sEMG source. It establishes that the fusion architecture is sound and
that it improves the controlled benchmark. It is **not** evidence about human physiology,
and nothing in it should be read as a claim that a real recording will behave this way.
Real-data feasibility is a separate stage with its own gate (`docs/SEMG_DATASET.md` §5) and
is deliberately not attempted here. §8 states exactly what would have to be measured on a
real recording before any of this transfers.

---

## 1. The problem this fixes

The corrected `ekf_cv` estimator carries a tremor model with **one shared frequency state**
and a two-harmonic basis, so it can represent tremor content only at `{ω, 2ω}`. When a
dominant component captures ω, weaker components at other frequencies cannot be
represented — and the near-constant-velocity **intent** states have enough freedom to take
them instead.

Measured directly, open loop, in `run_omega_capture_experiment.m`
(`output/results/omega_capture_experiment.csv`):

| Input | Intent absorption |
|---|---|
| 3 Hz alone | 10.4 % |
| 8 Hz dominant + 7 Hz | 8 % |
| 8 Hz dominant + 6 Hz | 19 % |
| 8 Hz dominant + 5 Hz | 34 % |
| 8 Hz dominant + 4 Hz | 55 % |
| 8 Hz dominant + 3 Hz | 87.8 % |

Absorbed tremor is **passed through to the reference as though it were voluntary motion**,
so the controller faithfully reproduces it at the spoon instead of cancelling it.

**No IMU-side statistic detects this.** Innovation and NIS stay extremely small throughout,
because intent + tremor still reconstruct the measurement accurately. The decomposition is
wrong while the fit is excellent. That is the definition of a weakly identifiable model, and
it is why the repair has to come from outside the fit.

Two IMU-only repairs were tried first and are recorded as failures:

- **A second independent oscillator** (`C.ekf_osc2=true`). Its shipped presence gate
  effectively never activates; forced open it only partially recovers the missing component.
  The benchmark shows it tracking `ekf_cv` to within 0.2 points on every case.
- **An independent frequency observable** from sEMG (E1). Negative on two counts: a perfect
  ω oracle bought only +7.9 points on case 9, so ω was never the binding constraint; and the
  burst-band estimator is biased (9.15 Hz against a true 8 Hz), regressing 4 of 5 in-band
  sine cases.

---

## 2. The ceiling experiment — why a *gate*, and why it needs a *sensor*

Reproduce with `run_intent_authority_experiment()`
(`output/results/intent_authority_ceiling.csv`, figure `intent_authority_ceiling.png`).
**No sEMG is generated or read anywhere in that script.** It scales `C.ekf_q_intent`, the
intent process noise — the estimator's licence to attribute motion to voluntary intent — and
measures what happens.

| case | q ×1 | q ×1/2 | q ×1/8 | q ×1/64 | q ×1/512 |
|---|---|---|---|---|---|
| 6 `mixed` (off-harmonic tremor) | 87.99 % | 89.41 % | 91.46 % | **93.00 %** | 93.55 % |
| 8 `irregular` (off-harmonic tremor) | 87.81 % | 89.26 % | 91.32 % | **92.85 %** | 93.39 % |
| 3 `sine` 12 Hz | 89.77 % | 90.14 % | 90.64 % | 90.99 % | 91.17 % |
| 2 `sine` 8 Hz | 93.78 % | 93.84 % | 93.89 % | 93.96 % | 93.97 % |
| 9 `combined` (intent + tremor) | **85.88 %** | 75.86 % | 26.86 % | −81.41 % | −398.51 % |
| 10 `intent` (voluntary only) | **0.201° RMS** | 0.689° | 1.430° | 3.234° | 5.607° |

Read the last two rows against the first four. **The setting that best suppresses
off-harmonic tremor is the setting that destroys voluntary motion**, and the crossover is
brutal: at merely ×1/2 — where `mixed` has gained only 1.4 points — case 9 has already lost
10 and the voluntary-only case has more than trebled its error. By ×1/8 case 9 has lost 59
points. The tremor benefit saturates by ×1/64, so about 1.8 decades of swing is all the
mechanism can use, and it must be spent selectively.

This is the whole argument for the architecture:

> `q_intent` is the right knob, one fixed value of it cannot serve both classes of case, and
> therefore **something has to tell the estimator which case it is currently in**. No amount
> of tuning produces that information, because it is not in the IMU signal — the IMU sees
> only the sum, and the sum is identical either way. It is in the *muscle*.

---

## 3. The observable, and the one that was rejected

The synthetic source models activation as driven by the joint torque the muscle supplies
(`rcflex_semg_truth.m`): for the slow voluntary component that is dominated by the posture
term, so activation follows the intent **angle**; each axis is spanned by an
agonist/antagonist pair, so a signed drive loads one and unloads the other over a
co-contraction floor. The signed **difference** of the two envelopes therefore carries net
voluntary drive, while their sum is dominated by the co-contraction floor — which rises with
tremor as well as with voluntary effort and so does not discriminate.

### 3.1 Rejected: a calibrated intent *angle* pseudo-measurement

The obvious fusion is to treat the calibrated differential envelope as a direct measurement
of the intent state. It was built and measured first, and rejected on the numbers:

| Quantity | Measured |
|---|---|
| Correlation with true intent angle, intent-bearing scenarios | r = 0.945 – 0.984 |
| Full-band error, roll axis | **1.6 – 1.8° RMS** |
| Residual in the 1–5 Hz band (after high-passing out the slow error) | **0.6 – 1.4° RMS** |
| Intent absorption the fusion must remove | **0.02 – 0.18°** |

The roll error is dominated by the activation saturation the truth model imposes at full
MVC: the drive folds over above about 4.1° of wrist deviation, so a linear gain is not even
monotone there. An angle pseudo-measurement would therefore have injected an order of
magnitude more error than it removed, and it would have injected it straight into the
reference the controller tracks. High-passing the observable does not rescue it, because the
residual noise is broadband rectified-envelope noise rather than tremor leakage.

### 3.2 Adopted: voluntary-drive **magnitude**

The magnitude of the drive is a far weaker claim about the signal, and it is the claim the
estimator actually needs. Measured across all 20 benchmark cases
(`calibrate_rcflex_semg_intent`, `output/results/semg_intent_calibration.csv`), as an
equivalent wrist angle:

| Class | Cases | Drive magnitude (RMS) |
|---|---|---|
| Tremor-only | 18 | ≤ **0.331°** |
| Intent-bearing | 2 | ≥ **2.088°** |

A **6.3 : 1** separation, i.e. 1.7 decades in power — which is almost exactly the 1.8 decades
of `q_intent` swing the ceiling experiment says the mechanism can use. The observable and the
requirement are matched, and that match is what makes the gate the right shape of fusion.

The margin is real but it is **not generous**, and that should be said plainly: the
standalone acceptance test measures a worst tremor-only confidence of 0.093 against a
required 0.10, and a worst intent-bearing confidence of 0.811 against a required 0.50. The
closing side has about 7 % headroom. Any real-data degradation of the class separation will
show up there first, which is why §8 item 1 makes separation the primary real-data gate.

---

## 4. Architecture

```
          IMU (hand + spoon)                     sEMG (4 ch, 4 kHz)
                 |                                       |
                 v                                       v
        attitude / complementary            highpass -> mains notch -> rectify
                 |                                  -> envelope lowpass
                 |                                       |
                 |                          agonist - antagonist, per axis
                 |                                       |
                 |                      +----------------+----------------+
                 |                      |                                 |
                 |              SLOW 2-pole LP @ 1 Hz          FAST 2-pole LP @ 4 Hz
                 |              (clean, late; may CLOSE)      (noisy, early; may only OPEN)
                 |                      |                                 |
                 |              calibrated gain -> equivalent voluntary angle
                 |                      |                                 |
                 |              asymmetric envelope follower (attack 50 ms / release 1 s)
                 |                      |                                 |
                 |          (P/(P+Pfloor_slow))^2          (P/(P+Pfloor_fast))^2
                 |                      |                                 |
                 |                      +--------- max( . , . ) ----------+
                 |                                       |
                 |                             conf in [0,1], per axis
                 |                                       |
                 v                                       v
        +--------------------------------------------------------+
        |   13-state EKF   x = [intent(4) tremor(8) omega(1)]     |
        |   Q(theta_dot_x, theta_dot_x) *= conf_roll              |
        |   Q(theta_dot_y, theta_dot_y) *= conf_pitch             |
        +--------------------------------------------------------+
                 |
                 v
        lead-predicted voluntary reference -> authority manager -> PID -> allocator
```

**sEMG changes exactly one thing: how much process noise the intent states are given, per
axis, per sample.** It does not enter the measurement model, it adds no states, it never
touches the tremor states, and it cannot move the estimate directly. The IMU remains the
only thing that measures attitude.

### 4.1 Why process noise rather than a measurement

A pseudo-measurement asserts a value and can therefore be *wrong*, and its error goes
straight into the reference. Process noise asserts only a **bound on how fast a state may
change**, which is precisely the kind of statement a magnitude observable can support. If
the gate is wrong, the worst it can do is return the filter to its ungated behaviour (gate
wrongly open) or make the intent estimate slower (gate wrongly closed) — it cannot inject a
false angle. This is the standard adaptive-process-noise construction (Mehra 1972;
Bar-Shalom et al. 2001 §11), with the adaptation driven by an **exogenous sensor** rather
than by the filter's own residual, which is the necessary difference here: §1 showed the
residual carries no information about this failure at all.

### 4.2 Four design details that are load-bearing

**It overrides the NIS trust boost.** `Qeff` is boosted when the smoothed NIS is high. Gating
raises the residual *by design* — that is the mechanism working, because the unmodelled
component now lands in the residual instead of the intent states. A residual-driven boost
would therefore reopen exactly the freedom the gate has just withdrawn. The gate is applied
after the boost and wins. The physical claim is the stronger one: if the muscle is not
driving, the limb is not voluntarily accelerating, whatever the residual says.

**It defaults open.** A gate may only ever *remove* authority on positive evidence. The first
implementation started with a zero power accumulator, which asserts "no voluntary drive"
before anything has been measured; measured consequence, the confidence was pinned at 0.000
for the first 0.85 s of every run — exactly the window in which intent and tremor ramp
together and the joint fit is least identifiable — and case 9 fell from 85.9 % to 62.0 %.
The gate now publishes full confidence for `C.semg_gate_warmup_s` (1 s, the 2 % settling time
of its own 1 Hz lowpass) while its accumulators run underneath, so there is no release lag
when it takes over.

**It is two-speed, and the second path can only open it.** See §4.3 — this is the fix for
the measured fast-voluntary-movement failure and it is the largest single design detail.

**Its attack and release are asymmetric** (50 ms / 1 s). The two errors do not cost the same:
wrongly closing suppresses voluntary movement, which is the one failure this device must not
have; wrongly holding open merely reverts to the baseline. The asymmetry also removes a
measured fault — voluntary drive follows the intent *angle*, so a symmetric window collapses
the confidence at every zero crossing of a reciprocating movement, precisely when the intent
*velocity* is largest. Measured on case 9 before the fix: pitch confidence fell to 0.40 at
each crossing.

### 4.3 The two-speed gate — and the failure that forced it

The single-speed gate described above **degraded fast voluntary movement**, and the
degradation was large, reproducible and exactly the failure this device must not have.

The `'reach'` scenario was added to test it: a 0.8 s minimum-jerk voluntary movement
superimposed on 8 Hz tremor, starting inside the scoring window. Four sensor seeds,
single-speed gate:

| seed | IMU-only (`ekf_cv`) | single-speed sEMG gate |
|---|---|---|
| 9205 | 71.51 % / 0.285° | −14.8 % / 1.148° |
| 9206 | 71.50 % / 0.285° | 26.9 % / 0.731° |
| 9207 | 71.49 % / 0.285° | 5.2 % / 0.948° |
| 9208 | 71.49 % / 0.285° | −12.2 % / 1.122° |

About 3.6× worse than IMU-only, with a large seed spread — itself a symptom, since it means
the gate's opening instant was being decided against envelope noise.

**Cause.** Rejecting tremor-driven activation requires a low corner frequency, and a low
corner costs group delay: ~225 ms at 1 Hz, plus ~75 ms in the envelope stage ahead of it,
against only ~40 ms of electromechanical lead to spend. The gate needed 0.3–0.5 s to notice
a movement that lasts 0.8 s, and until it did, the intent states were starved.

**Two fixes were tried. The first did not work and is recorded as such.** The NIS trust
boost exists to restore intent authority during exactly this kind of transient, and the gate
was multiplying it away. Splitting the boost so that its transient part survives any gate
produced results identical to four significant figures at a matched seed — because
`gamma_trust` never leaves ~1 during the reach. The residual cannot see the problem, which
is the same blindness as §1. *(An earlier revision of this document reported that this change
made things worse by 20 points. That comparison was run at two different sensor seeds and was
wrong.)*

**What worked** is to split the observable instead of the boost, and to exploit the same
asymmetry the release time constant already rests on: a false open costs only a return to
baseline, while a false close suppresses voluntary movement. So the gate runs two paths and
takes the larger confidence — a slow, clean path that alone decides when the gate may close,
and a fast 4 Hz path that can only ever open it, with its own higher floor measured at its
own bandwidth.

| Path | Corner | Group delay | Class separation | Floor | Role |
|---|---|---|---|---|---|
| Slow | 1 Hz | ~225 ms | **6.3 : 1** | 0.83° | may close the gate |
| Fast | 4 Hz | ~56 ms | 3.0 : 1 | 1.17° | may only open it |

The standalone test measures the resulting gate response to a voluntary movement at
**404 ms** after onset (`tests/test_rcflex_semg_intent_gate.m`).

**Measured effect**, same four seeds, and the cost stated alongside:

| Benchmark case / scenario | IMU-only `ekf_cv` | single-speed gate | **two-speed gate (shipped)** |
|---|---|---|---|
| `reach`, mean attitude RMS over 4 seeds | 0.285° | 0.99° | **0.67°** |
| case 6 `mixed` | 87.99 % | 93.01 % | **92.62 %** |
| case 7 `chirp` | 88.57 % | 91.12 % | **90.59 %** |
| case 8 `irregular` | 87.81 % | 92.99 % | **92.82 %** |
| case 9 `combined` | 85.88 % | 85.31 % | **85.31 %** |
| case 10 `intent` | 0.2008° | 0.2074° | **0.2070°** |
| headline mean over feasible cases | 92.12 % | 93.13 % | **93.04 %** |

A 32 % reduction in the fast-voluntary error for about 0.3 points of tremor suppression. The
fast path's weaker 3.0 : 1 separation is paid for in tremor suppression rather than in
voluntary motion, by construction: on the worst tremor-only case the confidence floor rises
from 0.019 to 0.063, so the intent process noise closes to about 1/16 instead of 1/53.

**This does not fully close the gap.** At 0.67° against 0.285°, the shipped system is still
about 2.4× worse than the IMU-only estimator on a fast voluntary movement. §9 states that as
a limitation rather than burying it.

---

## 5. Calibration, and why it is realisable

`calibrate_rcflex_semg_intent.m` fits one scalar per axis: envelope-difference volts →
equivalent voluntary joint angle (radians). It is fitted on a **tremor-free voluntary trial**,
scored against the **handle attitude the IMU measures directly**. With no tremor present the
handle attitude *is* the voluntary angle, so no ground-truth decomposition is required and
nothing about the procedure depends on the source being synthetic. On a real subject this is
one slow guided wrist movement, or a rest-and-reach block.

Contrast the E1 ω oracle, which reads simulation truth through `C` and is explicitly **not**
realisable. Nothing in E2 does that.

The trial uses seed 4242, held out from the benchmark's sensor seeds (9100 + k), so the
constants are not fitted on the data they are scored against.

Current values (`C.semg_gate_cal_rad_per_V`): `[130.86; 85.98]`, fit r = 0.891 / 0.895.

**Robustness to calibration error.** The gate is a ratio test between `P` and a floor set at
the geometric mean of two classes 1.7 decades apart in power. A 20 % gain error moves `P` by
44 %, i.e. 0.16 decades — an order of magnitude less than the margin. This is a direct
consequence of choosing a magnitude gate over an angle measurement, and it is the property
that should make the architecture survive a real electrode set.

---

## 6. Parameters — all in `rcflex_control_parameters.m`, all justified before the run

| Field | Value | Set from |
|---|---|---|
| `semg_fusion` | `'intent_gate'` | **THE SINGLE SWITCH.** `'off'` / `'semg'` / `'oracle'` / `'intent_gate'` / `'intent_gate_fixed'` |
| `semg_gate_fc_Hz` | 1 | sweep 0.6–5 Hz: the corner below which measured tremor-only drive stops falling, with 0.2–0.25 Hz voluntary content still passed |
| `semg_gate_fast_fc_Hz` | 4 | §4.3. ~56 ms group delay instead of ~225 ms; the open-only path |
| `semg_gate_fast_floor_deg` | 1.17 | geometric mean of the two classes measured **at 4 Hz** (0.676 / 2.016), same rule as the slow floor |
| `semg_gate_attack_s` | 0.05 | kept inside the 35–40 ms electromechanical lead, which is the entire budget for opening before the limb moves |
| `semg_gate_release_s` | 1 | 20× the attack; asymmetric cost of the two errors, and the zero-crossing fault in §4.2 |
| `semg_gate_warmup_s` | 1 | 2 % settling time of the 1 Hz 2-pole gate lowpass, ≈ 3/(π·fc) |
| `semg_gate_floor_deg` | 0.83 | geometric mean of the two measured classes (0.331 / 2.088), the same rule `semg_fuse_floor` already uses |
| `semg_gate_exponent` | 2 | matches the gate's 0.85-decade first-order output range to the measured 1.7 decades of class separation |
| `semg_gate_q_min` | 0.01 | numerical safeguard; the gate's closed value is 0.0187, so it does not bind |
| `semg_gate_fixed_conf` | 0.0187 | the gate's measured closed confidence, so the ablation applies the same reduction unconditionally |

None of these was fitted to a closed-loop result. Every one was set from an open-loop
measurement or a filter property before the benchmark was run.

---

## 7. Files

| File | Role |
|---|---|
| `src/control/rcflex_semg_intent_gate.m` | **new** — the observable. Pure function, packed state. |
| `src/control/rcflex_semg_init.m` | gate coefficients and packed-state layout |
| `src/control/rcflex_intent_ekf_step.m` | 5th argument `intentConf`; scales `Qeff` on the intent velocity (or acceleration, under `'ca'`) states |
| `src/control/rcflex_attitude_step.m` | arm dispatch, gate call, diagnostic slots 6:7 |
| `src/control/rcflex_control_parameters.m` | the single switch and the constants above |
| `src/control/rcflex_control_case.m` | **new scenario** `'reach'` — fast voluntary movement on tremor |
| `src/control/rcflex_intent_split_metrics.m` | **new** — `tremor_leak_deg` / `intent_track_deg` |
| `src/control/summarise_rcflex_benchmark.m` | **new** — headline means with the feasibility exclusion stated in code |
| `src/analysis/rcflex_case_feasible.m` | **new** — per-case inverse-dynamics envelope bound, shared with `evaluate_rcflex_envelope` |
| `src/analysis/calibrate_rcflex_semg_intent.m` | **new** — the calibration procedure |
| `src/analysis/run_intent_authority_experiment.m` | **new** — the ceiling experiment (§2) |
| `src/analysis/run_semg_intent_experiment.m` | **new** — E2, the three-arm comparison |
| `tests/test_rcflex_semg_intent_gate.m` | **new** — standalone acceptance criteria |

State layout is appended, never reordered: the EKF state vector is unchanged at 13 elements,
the diagnostic vector is unchanged at 32, and the gate's packed state sits after the existing
sEMG blocks. `C.semg_fusion='off'` reproduces every pre-fusion number exactly.

---

## 7a. Results

### The 20-case benchmark (`output/results/control_benchmark.csv`, summary CSV alongside)

Six arms, identical cases, identical sensor seeds (9100 + k). `mean_feasible` excludes
cases 5, 14 and 20, which an inverse-dynamics bound independent of any controller says the
hardware cannot serve at all (§9).

| Arm | sEMG fusion | Mean, all scored | **Mean, feasible** | Worst feasible | Case 10 voluntary RMS | Meets 90 % |
|---|---|---:|---:|---:|---:|---:|
| `simple_pid` (baseline) | off | 64.19 % | 90.23 % | 51.51 % | 0.4583° | 15/16 |
| `ekf_cv_legacy` | off | 68.75 % | 88.96 % | 30.75 % | 1.2390° | 11/16 |
| **`ekf_cv`** (IMU-only EKF) | off | 71.03 % | 92.12 % | 85.88 % | 0.2008° | 11/16 |
| `ekf_cv_osc2` | off | 71.04 % | 92.13 % | 85.90 % | 0.2008° | 11/16 |
| `ekf_cv_qfix` (**IMU-only ablation**) | intent_gate_fixed | 65.62 % | 85.18 % | **−41.41 %** | **3.0418°** | 15/16 |
| **`ekf_cv_semg`** (sEMG+IMU) | intent_gate | **72.63 %** | **93.04 %** | 85.31 % | 0.2070° | **15/16** |

### What the sEMG stream is worth

The ablation is the measurement that matters, because it applies the **identical** Q-scaling
mechanism with no sensor. Against it, the sEMG stream buys:

| | IMU-only ablation | sEMG+IMU | Attributable to sEMG |
|---|---|---|---|
| Mean suppression, feasible cases | 85.18 % | 93.04 % | **+7.86 points** |
| Case 9 `combined` | −41.41 % | 85.31 % | **+126.7 points** |
| Case 10 voluntary-motion RMS | 3.0418° | 0.2070° | **14.7× lower** |

And against the IMU-only EKF it did not regress the thing the EKF was good at while fixing
what it was bad at: cases meeting the 90 % target rise from 11/16 to 15/16, and the
off-harmonic cases the EKF lost to the PID baseline on are recovered —
case 6 `mixed` 87.99 → 92.62 %, case 8 `irregular` 87.81 → 92.82 %, case 7 `chirp`
88.57 → 90.59 %, all now at or above the `simple_pid` baseline.

### The estimator's own decomposition (`output/results/semg_intent_e2.csv`)

`tremor_leak_deg` is tremor the estimator called voluntary and therefore reproduced at the
spoon; `intent_track_deg` is voluntary motion it failed to follow. Definitions and the 2 Hz
split were fixed before any fusion result existed (`rcflex_intent_split_metrics.m`).

| Scenario | Tremor leak: IMU-only → sEMG+IMU | Intent tracking: IMU-only → sEMG+IMU |
|---|---|---|
| `mixed` | 0.1707° → **0.0661°** | 0.0535° → 0.0237° |
| `irregular` | 0.1704° → **0.0553°** | 0.0911° → 0.0788° |
| `sine` 12 Hz | 0.0381° → **0.0143°** | 0.0126° → 0.0128° |
| `combined` | 0.0271° → 0.0268° | 0.1387° → 0.1465° |
| `intent` | 0.0402° → 0.0417° | 0.2110° → 0.2179° |
| `reach` | 0.2103° → 0.4233° | 0.2652° → **0.5337°** |

This is the mechanism visible directly: on the cases with off-harmonic tremor and no
voluntary motion, the tremor the estimator absorbs into intent falls by **2.6–3.1×**, and
voluntary tracking does not degrade. On the two cases with slow voluntary motion nothing
moves in either direction. On `reach` the cost of the gate's onset latency shows up, and is
the last row of §9.

### Actuator and power

No arm changes the actuator demand materially: peak current across the whole sweep is
1.4305 A (`ekf_cv`) against 1.4320 A (`ekf_cv_semg`) — both set by case 14, which is outside
the envelope anyway — and peak mean coil power is 5.7228 W against 5.7190 W. **The sEMG
result is not bought with actuator effort**, which matters because a suppression gain paid
for in current would not survive the thermal and battery limits.

---

## 8. What a real recording would have to show

This section is the honest boundary of §2–§6, and it is written so the next stage has a
falsifiable target rather than an aspiration.

The single quantity the whole architecture depends on is the **class separation of the
voluntary-drive magnitude** — measured at 6.3 : 1 here, entirely on a synthetic source that
was built from the same intent truth that drove the plant. On real data it must be measured
again, and the requirement is set by the ceiling experiment rather than by this result:

1. **Separation ≥ 4 : 1** between a tremor-only segment and a voluntary-movement segment, on
   the same subject, same session, same electrodes. Below that the gate cannot deliver the
   `q_intent` swing the mechanism needs, and the fusion should be abandoned rather than
   retuned. (The tests assert conf < 0.10 / conf > 0.50, which 4 : 1 satisfies.)
2. **Calibration stability** across a session: refit the gain at the start and end of a
   recording and require the two to agree within the 0.16-decade margin of §5.
3. **Gate latency** on a real voluntary movement < 350 ms, the criterion the `'reach'` test
   already applies synthetically.
4. The **acquisition-health and cross-talk gates** in `docs/SEMG_DATASET.md` §5 must pass
   first. They are prerequisites, not part of this experiment.

Things this experiment cannot speak to at all, and which are listed so they are not quietly
assumed: electrode shift, fatigue-driven spectral compression, real cross-talk between
agonist and antagonist sites (the synthetic source generates its channels independently and
therefore has essentially none by construction), co-contraction strategies that differ from
the modelled reciprocal pattern, and any subject whose voluntary drive does not follow joint
angle in the quasi-static way §3 assumes — which includes, importantly, a subject holding a
loaded spoon steady against gravity while making no movement at all.

The interface is kept clean for that stage: `C.semg_source='file'` routes a real synchronised
recording through the *identical* preprocessing, feature and gate code
(`rcflex_semg_source.m`, format contract in `rcflex_semg_dataset_spec.m`), and
`calibrate_rcflex_semg_intent.m` runs against it unchanged.

---

## 9. Known limitations of the result itself

- **The synthetic sEMG is generated from the tremor and intent truth that drive the plant.**
  Every figure in §2–§6 is an upper bound, and the fusion result is a conditional: *if* a
  real observable separates the classes this well, *then* the estimator improves this much.
- **Benchmark cases 5, 14 and 20 are outside the hardware envelope** by an inverse-dynamics
  bound that assumes perfect knowledge and exact tracking (`rcflex_case_feasible.m`). They
  are excluded from the headline mean and reported separately; no controller could have
  served them, so scoring one on them measures the actuator.
- **Fast voluntary movements are still tracked worse than by the IMU-only estimator.** On the
  `'reach'` scenario the shipped two-speed gate averages 0.67° attitude RMS against 0.285°
  for `ekf_cv`, over four sensor seeds — better than the 0.99° the single-speed gate gave,
  but a real remaining regression of about 2.4×. The cause is irreducible within this design:
  the gate's detection latency is set by filter corners that exist to reject tremor-driven
  activation, and trading them away costs the class separation the mechanism depends on.

  This was re-examined properly as **experiment E3**
  (`src/analysis/run_fast_movement_experiment.m`, `START_PROJECT('fastmove')`), comparing the
  shipped amplitude gate against a rate-of-change detector and against a hybrid, at four
  swept corner frequencies, on the criteria declared in advance. **No candidate met both
  criteria**, and the reason turned out to be structural rather than a matter of tuning: the
  rate of muscle drive is largest for the *fastest* thing in the signal, and in a tremulous
  hand that is the tremor, not the voluntary movement. Measured by the project's own
  geometric-mean rule, the rate observable separates the two classes **the wrong way round**
  — 0.69 : 1 at a 2 Hz corner falling to 0.13 : 1 at 16 Hz, against the ≥ 4 : 1 requirement.
  A separating threshold exists at all only at the 2 Hz corner, where a fast reach peaks
  2.3× above the worst tremor-only rate; at 4 Hz and above no threshold can separate them,
  at any value. The best rate arm (2 Hz) does reach 349 ms, but its mean confidence on a
  tremor-only record is 0.343 against the ≤ 0.10 the mechanism needs, so it would surrender
  most of the `q_intent` swing that is the entire point of the gate. The target was
  therefore not moved and no candidate was adopted. **The 20-case benchmark contains no fast voluntary movement, so the headline
  numbers in §7 do not reflect this cost** — which is precisely why `'reach'` exists and is
  reported separately. Anyone quoting the headline should quote this too.
- **The gate is binary in spirit.** It distinguishes "voluntary drive present" from "absent".
  It does not, and on this evidence cannot, estimate *how much* voluntary motion there is —
  §3.1 is why.
- **A sustained isometric hold reads as voluntary drive.** The gate would stay open with no
  movement occurring, returning the estimator to its ungated behaviour. That is the safe
  direction of failure, but it is a real gap and it is the first thing §8 item 1 should be
  tested against.
- The plant model still has no hysteresis, dead zone or creep, per the project's standing
  caveat.

---

## References

- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to
  Tracking and Navigation*. Wiley. (Adaptive and manoeuvre-dependent process noise.)
- Cavanagh, P. R., & Komi, P. V. (1979). Electromechanical delay in human skeletal muscle
  under concentric and eccentric contractions. *European Journal of Applied Physiology*,
  42(3), 159–163.
- De Luca, C. J. (1997). The use of surface electromyography in biomechanics. *Journal of
  Applied Biomechanics*, 13(2), 135–163.
- Fleischer, C., & Hommel, G. (2008). A human–exoskeleton interface utilizing electromyography.
  *IEEE Transactions on Robotics*, 24(4), 872–882.
- Gallego, J. A., Rocon, E., Roa, J. O., Moreno, J. C., & Pons, J. L. (2010). Real-time
  estimation of pathological tremor parameters from gyroscope data. *Sensors*, 10(3),
  2129–2149.
- Hermens, H. J., Freriks, B., Disselhorst-Klug, C., & Rau, G. (2000). Development of
  recommendations for SEMG sensors and sensor placement procedures. *Journal of
  Electromyography and Kinesiology*, 10(5), 361–374.
- Hogan, N., & Mann, R. W. (1980). Myoelectric signal processing: optimal estimation applied
  to electromyography. *IEEE Transactions on Biomedical Engineering*, 27(7), 382–395.
- Mehra, R. K. (1972). Approaches to adaptive filtering. *IEEE Transactions on Automatic
  Control*, 17(5), 693–698.
- Merletti, R., & Parker, P. A. (Eds.) (2004). *Electromyography: Physiology, Engineering and
  Non-Invasive Applications*. IEEE Press/Wiley.
- Riviere, C. N., Rader, R. S., & Thakor, N. V. (1998). Adaptive cancelling of physiological
  tremor for improved precision in microsurgery. *IEEE Transactions on Biomedical
  Engineering*, 45(7), 839–846.
- Rocon, E., Belda-Lois, J. M., Ruiz, A. F., Manto, M., Moreno, J. C., & Pons, J. L. (2007).
  Design and validation of a rehabilitation robotic exoskeleton for tremor assessment and
  suppression. *IEEE Transactions on Neural Systems and Rehabilitation Engineering*, 15(3),
  367–378. (WOTAS: the canonical tremor exoskeleton combining inertial and myoelectric
  sensing, and the closest prior art to this architecture.)
- Veluvolu, K. C., & Ang, W. T. (2011). Estimation of physiological tremor from accelerometers
  for real-time applications. *Sensors*, 11(3), 3020–3036.
