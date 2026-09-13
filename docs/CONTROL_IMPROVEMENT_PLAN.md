# SensoSpoon — Control Algorithm Upgrade: EKF-Based Predictive Tremor Cancellation

## Context

The SensoSpoon stabilises a spoon bowl on a 2-axis flexure gimbal driven by two voice-coil
actuators, cancelling hand tremor for users with essential tremor or Parkinson's. The
current controller is a cascaded PID: an 800 Hz attitude loop (ω_n = 35 Hz, ζ = 0.85) with
mass-matrix/gravity/friction feedforward and a 9-case active-set current allocator, over a
10 kHz current PI loop with back-calculation anti-windup. That inner machinery is sound and
is **not** what limits performance.

The limiting element is the **intent estimator** in `src/control/rcflex_prepare_control.m`:
a fixed causal IIR chain — five notches hard-wired at 4/6/8/10/12 Hz, a 0.14 s phase lead,
and a 10 Hz Butterworth rolloff. Every coefficient is a design-time constant. It assumes the
tremor frequency is known in advance and never changes.

`output/results/control_benchmark.csv` shows exactly what that assumption costs:

| Case | Condition | Suppression |
|---|---|---|
| 1–8, 11–13, 15–19 (16 cases) | tremor inside the 4–12 Hz design band | 89.3 – 98.5 % (mean **92.59 %**) |
| 9 | voluntary intent **and** tremor together | **51.5 %** |
| 14 | 5° step in intent | **−116.6 %**, limits violated |
| 20 | 20 Hz tremor, outside the notch band | **−196.7 %**, limits violated |
| 10 | intent only | NaN (no tremor baseline); 0.458° tracking error |

Negative suppression means the controller made the spoon **worse than if it were rigidly
bolted to the handle**. The headline "92–93 %" is the mean over the 16 in-band cases only.
The honest mean over all 19 scored cases is **≈ 64.2 %**.

Two further facts constrain any fix. First, the lag is in the *reference* path — the IIR
chain runs on handle attitude to produce the target, and the error is formed after it — so
raising loop bandwidth cannot recover it. Second, authority is already exhausted:
`src/analysis/evaluate_rcflex_envelope.m` (see `output/results/analysis_run.txt`) reports an
8 Hz / 5° / 5 g case demanding **1.415 A** against a 1.5 A limit and **7.50 V** against a
7.4 V bus. More aggression spends a budget that is already overdrawn.

### The intended outcome

Replace *filtering* with *modelling and prediction*. Hand tremor is quasi-periodic — coherent
over many cycles with a slowly drifting frequency — so an online model can estimate its
amplitude, phase and frequency and evaluate it one step ahead, yielding a tremor-free intent
reference with **zero phase lag** and **no fixed-frequency assumption**. An Extended Kalman
Filter is chosen over WFLC/BMFLC alternatives because it additionally produces an
**innovation and covariance**, which is the signal that detects "my oscillator model no
longer explains the data" — precisely the condition in cases 9 and 14. A predictive
authority manager then uses the same prediction to scale cancellation back *before*
saturating, converting catastrophic negative suppression into graceful degradation toward 0 %.

Target: **> 88 % mean across all 19 scored cases**, with no case below 0 %, while holding the
16 in-band cases at or above their current values.

---

## Architecture

```
IMU (14-ch, 800 Hz)
      │
      ▼
┌─────────────────────────────────────────────┐
│ NEW: EKF intent/tremor separator            │  θ̂_intent, θ̂̇_intent
│  • intent: near-constant-velocity states    │  ω̂_tremor
│  • tremor: adaptive-frequency oscillator    │  NIS (innovation)
│    (2 harmonics, shared ω, both axes)       │
└─────────────────────────────────────────────┘
      │                    │
      │ NIS, P_ωω          ▼
      ▼            ┌──────────────────────────┐
┌──────────────┐   │ NEW: trust gate γ_trust  │
│ NEW:         │   │  high NIS → fall back to │
│ predictive   │◄──┤  following the raw hand  │
│ authority    │   └──────────────────────────┘
│ manager      │
│  γ_auth      │  reuses rcflex_mass_bias + rcflex_linkage
└──────────────┘
      │  q_ref (rate-limited, feasible)
      ▼
╔═════════════════════════════════════════════╗
║ UNCHANGED: 800 Hz attitude PID + allocator  ║
║ UNCHANGED: 10 kHz current PI                ║
║ UNCHANGED: plant, sensors, Simulink wiring  ║
╚═════════════════════════════════════════════╝
```

---

## Algorithm specification

### 1. EKF intent/tremor separator

Runs at `C.outer_hz` = 800 Hz on the two handle-attitude channels
`hand = [atan2(Rh(3,2),Rh(2,2)); atan2(Rh(1,3),Rh(1,1))]` already computed at
`src/control/rcflex_attitude_step.m:19`. The tremor frequency is physically common to both
axes, so a **single joint EKF with a shared ω** is used rather than two independent filters.

**State vector (13 states):**

| Index | Symbol | Meaning |
|---|---|---|
| 1–2 | θ_i,x, θ̇_i,x | voluntary intent angle and rate, axis x |
| 3–4 | θ_i,y, θ̇_i,y | voluntary intent angle and rate, axis y |
| 5–6 | a_1,x, b_1,x | fundamental tremor quadrature pair, axis x |
| 7–8 | a_2,x, b_2,x | second harmonic, axis x |
| 9–10 | a_1,y, b_1,y | fundamental, axis y |
| 11–12 | a_2,y, b_2,y | second harmonic, axis y |
| 13 | ω | shared tremor angular frequency (rad/s) |

**Process model** (dt = 1/800 s):

```
θ_i(k+1)  = θ_i(k) + dt·θ̇_i(k)
θ̇_i(k+1)  = θ̇_i(k)                                    + w_intent
[a_h; b_h](k+1) = R(h·ω·dt) · [a_h; b_h]               + w_tremor,   h = 1,2
        where R(φ) = [cos φ, −sin φ; sin φ, cos φ]
ω(k+1)    = ω(k)                                       + w_ω
```

**Measurement model** (2 measurements, one per axis):

```
y_x = θ_i,x + a_1,x + a_2,x  + v
y_y = θ_i,y + a_1,y + a_2,y  + v
```

The only nonlinearity is `R(h·ω·dt)` depending on state 13. The Jacobian block for the ω
column is `h·dt · dR/dφ · [a_h; b_h]` with `dR/dφ = [−sin φ, −cos φ; cos φ, −sin φ]`.
Everything else is constant — so `F` is built once and only the ω column is recomputed per
step. Use the standard predict/update; enforce symmetry with `P = (P + P')/2` after update.

**ω constraint.** After each update clamp state 13 to `[2π·2, 2π·25]` rad/s (2–25 Hz) and
zero the corresponding covariance cross-terms if the clamp activates. This bounds the
adaptive loop and is what lets case 20 (20 Hz) succeed instead of failing at −196.7 %.

**Initialisation.** ω₀ = 2π·6 rad/s (mid-band), intent states from the first sample,
tremor quadratures zero, `P₀` diagonal with large variance on ω so it converges fast.

#### 1a. Independent tremor-frequency observable (identifiability repair)

*Added after the first integration, in response to a measured failure. Retained here
because it changes the packed state layout described in Step 4.*

The joint intent + two-harmonic + shared-ω model is **weakly identifiable** during a
transient in which the voluntary intent and the tremor amplitude ramp up *simultaneously*
(the `combined` scenario, case 9, and the amplitude ramp that opens every scenario in
`rcflex_control_case.m`). Over a short window the near-constant-velocity intent states can
absorb part of the oscillation, so the data is explained almost as well at a wrong ω as at
the right one. Measured consequence: ω was dragged to a false ≈ 2.0–2.3 Hz around
t ≈ 0.9–1.0 s and then recovered at only ≈ 1.5 Hz/s, never reaching the true 8 Hz inside the
4 s case. Critically `nis_bar` stayed below 0.15 (threshold 4) throughout — the filter is
*internally self-consistent while being wrong*, so a single-step innovation statistic cannot
detect this failure mode at all.

Four tuning-style remedies were tried and rejected, all for the same structural reason:
gating `Q_ω` by tremor amplitude, gating the covariance update at low amplitude, an
amplitude-confidence-weighted ω rate limit, and a fixed time-based ω freeze after reset.
Each slows or gates the *single* ω adaptation law, so each trades false-lock robustness
directly against the fast-convergence requirement (the 20 Hz case, and the unit tests) —
both requirements route through that one state. Median-filtering ω and feeding it back into
the process model is worse still: a lagged ω inside the oscillator's own phase-prediction
Jacobian is self-reinforcing and stalls convergence outright.

An **output-side decoupling** (a slow ω_output for the cancellation reference, separate from
a fast ω_internal for the process model) cannot work either, and the reason is worth
recording: the reference is the *intent* estimate, and the intent estimate is polluted
whenever ω_internal is wrong, because the oscillator then fails to absorb the tremor. The
harmful quantity depends on ω_internal, so only making ω_internal correct helps.

The structural remedy is therefore to **add information about ω that does not come from the
joint fit at all**. For a single sinusoid `h`,

```
E[ h(k−D)·(h(k) + h(k−2D)) ] = 2·cos(ω·D·dt)·E[h²]
```

so with `h = HP(hand)` (two-pole highpass at `C.freq_hp_Hz` = 3 Hz) and both axes summed,

```
cos(ω̂_ind·D·dt) = num_bar / den_bar,     ω̂_ind = acos(·)/(D·dt)
```

where `num_bar`, `den_bar` are EWMAs (`C.freq_tau_s` = 0.15 s) of the two products.
Properties that matter:

- **Intent is removed by the highpass, not by the ambiguous joint fit.** Voluntary reaching
  motion is below 2 Hz; the EKF's ω clamp already assumes tremor lies in 2–25 Hz. This
  estimate is therefore structurally immune to the intent/tremor trade-off above.
- **Noise averages out rather than biasing.** Measurement noise enters `num_bar` only as
  zero-mean cross terms; only `den_bar` carries a `+σ²` bias, and at 500:1 power SNR for a
  1° tremor that is negligible. A derivative-based (Rayleigh-quotient) alternative would
  instead carry a several-Hz noise bias at 800 Hz.
- **Lag `D` = 8 samples (`C.freq_lag_samples`)** puts `ω·D·dt` in 0.20–1.57 rad over 2–25 Hz —
  near the well-conditioned middle of `acos` (≈ 28× the sensitivity of `D` = 1) — and keeps
  the estimate unambiguous to 50 Hz.

It is injected as a **sequential scalar pseudo-measurement on state 13**, applied after the
two-axis attitude update so `nis` remains a pure attitude-fit statistic (the trust gate's
meaning, and the Step 9 NIS assertions, are untouched). Its noise is
`(2π·C.freq_meas_sd_Hz)² / confidence`, with

```
confidence = P_tremor / (P_tremor + P_floor),    P_floor = C.freq_power_floor_deg²
confidence = 0  if  ω̂_ind ∉ [ω_min, ω_max]
```

so it adds information when tremor is present and observable and contributes nothing when it
is not — the tremor-free `intent` scenario reads 1.3–1.9 Hz, below `ω_min`, and is rejected.
Because it is a *measurement* rather than a filtered copy of ω, it does not create the
self-reinforcing phase-prediction error that lagging ω inside the process model does.

```matlab
C.freq_hp_Hz=3; C.freq_lag_samples=8; C.freq_tau_s=.15;
C.freq_power_floor_deg=.3; C.freq_meas_sd_Hz=1;
```

**Suggested tuning (place in `rcflex_control_parameters.m`, tune via the grid in Step 6):**

```matlab
C.ekf_harmonics      = 2;
C.ekf_q_intent       = 1e-4;     % rad²/s³ — voluntary motion agility
C.ekf_q_tremor       = 1e-5;     % rad²    — tremor amplitude drift
C.ekf_q_omega        = 5e-3;     % (rad/s)²— frequency drift rate
C.ekf_r_meas         = (deg2rad(0.02))^2;  % attitude estimate noise
C.ekf_omega_min_Hz   = 2;
C.ekf_omega_max_Hz   = 25;
C.ekf_omega_init_Hz  = 6;
```

**Outputs.** Tremor-free intent reference, predicted one loop period ahead to cancel the
remaining computational/actuation delay:

```
ref = [θ̂_i,x + Δ_lead·θ̇̂_i,x ;  θ̂_i,y + Δ_lead·θ̇̂_i,y]
```

with `C.ekf_lead_s` (start at 1–2 sample periods, ≈ 2.5 ms — orders of magnitude smaller
than the 0.14 s lead the IIR chain needed, which is the whole point). This `ref` drops into
`rcflex_attitude_step.m:26` exactly where the IIR output currently lands, so the entire
downstream path is untouched.

### 2. Trust gate (fixes cases 9 and 14)

Normalised innovation squared, `NIS = νᵀ S⁻¹ ν` with `S = H P Hᵀ + R`, has expectation equal
to the measurement dimension (2) when the model fits. Maintain a smoothed value:

```
nis_bar(k) = (1 − α)·nis_bar(k−1) + α·NIS(k),    α = 1 − exp(−dt/C.ekf_nis_tau_s)
γ_trust    = clamp(1 − (nis_bar − n_lo)/(n_hi − n_lo), 0, 1)
```

with `C.ekf_nis_tau_s ≈ 0.05`, `n_lo ≈ 4`, `n_hi ≈ 25`. Two uses:

1. **Adaptive process noise.** When `γ_trust < 1`, inflate `Q_intent` by
   `(1 + C.ekf_q_boost·(1 − γ_trust))` so the intent states re-converge within a few samples
   after a step instead of being dragged by the oscillator model.
2. **Reference blending.** `ref ← γ_trust·ref_ekf + (1 − γ_trust)·hand_raw`. When the model
   has lost the plot, the spoon simply follows the hand — which is the rigid-spoon baseline,
   i.e. 0 % suppression, not −116 %.

### 3. Predictive authority manager (fixes the limit violations)

The EKF gives the tremor estimate one step ahead, so the required cancellation is known
*before* it is demanded. Over a short horizon `C.auth_horizon_s` (≈ 0.02 s, sampled at a
handful of points), predict the tremor trajectory by propagating the oscillator states
forward, then evaluate the required current using machinery that already exists:

- `rcflex_linkage(q, H)` → `L.J`, `L.s`  (`src/model/rcflex_linkage.m`)
- `rcflex_mass_bias(...)` → `M`, `gravity`  (`src/model/rcflex_mass_bias.m`)
- force constant `K = H.vca_force_constant_N_A * max(1 − 0.27*(L.s/(H.vca_stroke_mm/1000)).^2, 0.5)`
- `i_req = (L.J' * diag(K)) \ (M·q̈_pred + gravity + flexure terms)`

This is the same inverse-dynamics feasibility computation already implemented in
`src/analysis/evaluate_rcflex_envelope.m` — **factor that calculation out into a shared
helper rather than duplicating it**, so the offline envelope analysis and the online manager
cannot drift apart.

Then:

```
γ_auth_raw = min(1, C.auth_margin · I_limit / max(|i_req|)),   C.auth_margin ≈ 0.85
γ_auth     = rate-limited toward γ_auth_raw at C.auth_rate_per_s (≈ 5 /s)
```

Also cap on joint travel: if predicted `|q_ref| > C.joint_limit`, fold that into `γ_auth`
proportionally. Apply as `ref = hand_raw + γ_auth·(ref_intent − hand_raw)` — scaling *how much
tremor is cancelled*, never the intent itself, so voluntary tracking is unaffected by
derating. The rate limit is essential: instantaneous γ changes would inject a step into the
reference and defeat the purpose.

**Why scaling and not MPC.** Constrained MPC would reach a marginally higher ceiling, but
the metric is a mean over cases and the dominant term is the negative outliers: turning case
14 from −116.6 → ≈ 0 and case 20 from −196.7 → ≈ 0 is worth **+15.7 points of overall mean on
its own**, more than perfecting every already-working case. Graceful degradation dominates
peak performance under this scoring. State this trade-off explicitly in the report.

---

## Implementation steps

### Step 1 — The single switch  *(`src/control/rcflex_control_parameters.m`)*

Add **one field** that every downstream site branches on. Flipping algorithms must require
editing exactly this one line and nothing else.

```matlab
C.algorithm = 'ekf_predictive';   % 'simple_pid' | 'ekf_predictive'
```

Accept an optional argument `function C = rcflex_control_parameters(algorithm)` so scripts
and the benchmark can override programmatically without touching the file. Keep every
existing `simple_pid` field untouched, and add the `C.ekf_*` / `C.auth_*` fields listed
above. Set `C.name` from `C.algorithm` so existing result files self-label.

### Step 2 — New estimator files  *(`src/control/`)*

- **`rcflex_intent_ekf_init.m`** — given `C`, return the constant EKF matrices (`F` skeleton,
  `H`, `Q`, `R`, `P0`, `x0`, state-index map) packed into `C.ekf`. Called once from
  `rcflex_prepare_control.m`, mirroring how `C.intent_sos` is prepared today.
- **`rcflex_intent_ekf_step.m`** — signature
  `[s, ref, info] = rcflex_intent_ekf_step(s, hand, C)` where `s` is the packed state/covariance
  slice, `ref` is the 2×1 lead-predicted intent, and `info` returns
  `[omega_hat_Hz; nis_bar; gamma_trust; omega_indep_Hz; freq_confidence]` (the last two added
  with Sec 1a).
  Pure function, no persistent variables (determinism is asserted by
  `tests/test_rcflex_control.m`).
- **`rcflex_authority_manager.m`** — signature
  `[gamma_auth, i_pred] = rcflex_authority_manager(gamma_prev, q, qd, hand, tremor_states, H, B, C)`.
- **`rcflex_required_current.m`** — the shared inverse-dynamics helper extracted from
  `src/analysis/evaluate_rcflex_envelope.m`; refactor that file to call it.

### Step 3 — Branch the preparation  *(`src/control/rcflex_prepare_control.m`)*

Currently builds `C.intent_sos` unconditionally. Wrap:

```matlab
if strcmp(C.algorithm,'ekf_predictive')
    C = rcflex_intent_ekf_init(C);
    return
end
% ... existing lowpass/notch construction unchanged ...
```

### Step 4 — Branch the attitude step  *(`src/control/rcflex_attitude_step.m`)*

Replace **only** lines 20–26 (the IIR chain producing `ref`, then `Rd`). Everything from
line 27 onward — reference clamp, rate limit, error formation, PID, feedforward, allocator —
stays byte-identical. This is the whole point of the design: the intervention is confined to
the reference path.

```matlab
if strcmp(C.algorithm,'ekf_predictive')
    [z, refIntent, info] = ekf_slice_update(z, hand, C);       % writes z(101:160)
    gammaTrust = info(3);
    ref = gammaTrust*refIntent + (1-gammaTrust)*hand([2 1]);
    [gammaAuth, ~] = rcflex_authority_manager(z(161), q, qd_est, hand, ..., H, B, C);
    z(161) = gammaAuth;
    ref = hand([2 1]) + gammaAuth*(ref - hand([2 1]));
else
    % existing IIR chain, unchanged
end
```

Note the existing code uses `ref = hand([2 1])` — the axis swap must be preserved in the new
branch.

**State-width change.** The state vector is hard-coded as 100 in three places that must all
be updated together:
- `src/control/rcflex_attitude_step.m:7`
- `src/control/rcflex_controller_sfun.m:12` (`Dwork(1).Dimensions`)
- `src/control/rcflex_controller_sfun.m:16` (`init`)

Store `P` in full — memory is irrelevant here and full-matrix storage keeps the code
readable and the Jacobian bookkeeping obvious. **As built, the width is 400**, with the
packed EKF slice at 101–327 and γ_auth at 328:

| z indices | contents |
|---|---|
| 101–113 | `x` (13 states) |
| 114–282 | `P(:)` (13×13 = 169) |
| 283 | `nis_bar` |
| 284–291 | Sec 1a highpass states (2 stages × 2 axes × {y_prev, u_prev}) |
| 292–325 | Sec 1a delay lines (2·(2D+1) = 34 samples of the high-passed signal) |
| 326–327 | Sec 1a EWMA `num_bar`, `den_bar` |
| 328 | γ_auth |

The slice length is `C.ekf.s_len` and its initial value is `C.ekf.s0`, both built once in
`rcflex_intent_ekf_init`, so `rcflex_attitude_step` and `rcflex_controller_sfun` initialise
from a single definition and cannot drift apart. Both files' hard-coded `300` becomes `400`.

### Step 5 — Widen the diagnostic vector

`diagnostic` is 16 elements (`rcflex_attitude_step.m:70`) and that width is hard-coded in:
- `src/control/rcflex_controller_sfun.m:6` (`OutputPort(2).Dimensions=16`)
- `src/control/run_rcflex_control_fast.m:10` (`diagnostic=zeros(16,1)`)

Widen to **24** in all three, appending `[omega_hat_Hz; nis_bar; gamma_trust; gamma_auth;
i_pred_peak; omega_indep_Hz; freq_confidence; 0]` (the last two per Sec 1a — they occupy two
of the three spare slots reserved for this, so the width stays 24). For `simple_pid` these
trail as zeros so existing plots and
`rcflex_control_metrics.m` keep working unchanged. Define the width once as a named constant
if practical.

### Step 6 — Tuning  *(`src/control/tune_rcflex_control.m`)*

The existing script is stale — it tunes the legacy `'lowpass'` filter while the shipped
default is `'notch'`, so it is not the record of how current gains were chosen. Extend it to
branch on `C.algorithm` and sweep the EKF grid:

```
C.ekf_q_intent ∈ {1e-5, 1e-4, 1e-3}
C.ekf_q_omega  ∈ {1e-3, 5e-3, 2e-2}
C.auth_margin  ∈ {0.75, 0.85, 0.95}
```

scored on the existing training cases **plus** a 20 Hz case and a step case (the two failure
modes), written to `output/results/control_training.csv`. Gains are then frozen before the
held-out 20-case benchmark, exactly as now.

### Step 7 — Benchmark both algorithms  *(`src/control/benchmark_rcflex_control.m`)*

Wrap the case loop in an outer loop over `{'simple_pid','ekf_predictive'}`, using identical
seeds (`9100+k`) for both. Add an `algorithm` column to `control_benchmark.csv`. Keep the
existing `within_limits` / `meets_90_percent_target` gate untouched — it is the honest pass
criterion and the new design must clear it on its own merits.

### Step 8 — The comparison document  *(`docs/CONTROL_IMPROVEMENT.md`)*

Written as a standalone technical report — self-sufficient, with real citations, and detailed
enough that an engineer (or an LLM) could implement from it without this conversation.
Required sections:

1. **Problem statement** — the fixed-notch assumption, with the benchmark table above as
   evidence, and the honest 64.2 % all-case mean versus the 92.59 % in-band mean.
2. **Root-cause analysis** — why the lag is in the reference path (cite the line numbers),
   why raising bandwidth cannot fix it (Bode gain-phase relation), and why authority is
   already exhausted (`analysis_run.txt` figures).
3. **Algorithm comparison table** (the core deliverable):

   | Dimension | Current: fixed notch + PID | Proposed: EKF predictive |
   |---|---|---|
   | Tremor frequency | assumed known, fixed 4/6/8/10/12 Hz | estimated online, 2–25 Hz |
   | Phase lag on reference | notch lag partly offset by 0.14 s lead | ≈ 0 (model-based, predicted ahead) |
   | Intent/tremor separation | spectral, by filter band | temporal, by coherence model |
   | Out-of-band tremor | −196.7 % (case 20) | tracked by adaptive ω |
   | Transient / step | −116.6 % (case 14) | NIS gate → falls back to hand-following |
   | Saturation behaviour | hard clip, limits violated | predicted, scaled before violation |
   | Self-assessment signal | none | innovation + covariance |
   | Computation @ 800 Hz | 7 biquads | 13-state EKF (~2 k flops) |
   | Tuning parameters | 8 filter constants | 7 noise/gate constants |
   | Failure mode | degrades to negative suppression | degrades to ≈ 0 % (rigid baseline) |
4. **Theory and derivation** — full state/process/measurement equations and Jacobians as
   given above.
5. **Per-case before/after results table** — all 20 cases, both algorithms.
6. **Honest limitations** — EKF divergence risk and the ω clamp that bounds it; the
   authority manager's ceiling being below constrained MPC; the plant model has no
   hysteresis, dead zone or creep so real-hardware performance will be lower; the README's
   own caveat that this is an algorithm-development baseline, not a validated digital twin.
7. **References**, e.g.:
   - Riviere, C. N., Rader, R. S., & Thakor, N. V. (1998). Adaptive cancelling of
     physiological tremor for improved precision in microsurgery. *IEEE Trans. Biomedical
     Engineering*, 45(7), 839–846. — WFLC, the canonical adaptive tremor estimator.
   - Veluvolu, K. C., & Ang, W. T. (2011). Estimation of physiological tremor from
     accelerometers for real-time applications. *Sensors*, 11(3), 3020–3036. — BMFLC.
   - Riviere, C. N., & Thakor, N. V. (1996). Modeling and canceling tremor in human–machine
     interfaces. *IEEE Engineering in Medicine and Biology Magazine*, 15(3), 29–36.
   - Bar-Shalom, Y., Li, X.-R., & Kirubarajan, T. (2001). *Estimation with Applications to
     Tracking and Navigation*. Wiley. — EKF, NIS consistency testing (Ch. 5, 11).
   - Zhou, Y., Jenkins, M. E., Naish, M. D., & Trejos, A. L. (2018). Multi-sensor fusion for
     tremor estimation. — modern adaptive-oscillator variants.
   - Anderson, B. D. O., & Moore, J. B. (1979). *Optimal Filtering*. Prentice-Hall.

### Step 9 — Tests  *(`tests/`)*

New `tests/test_rcflex_intent_ekf.m`:
- ω converges to within 0.2 Hz of truth inside 2 s for pure 4, 8, 12 and **20** Hz sinusoids.
- ω tracks a 4 → 12 Hz chirp with bounded lag.
- Residual intent error < 0.05° on a pure-tremor input (the estimator must not leak tremor
  into intent).
- NIS spikes above `n_hi` within 50 ms of a 5° step and recovers within 300 ms.
- `P` stays symmetric positive-definite for 10 000 steps (`min(eig(P)) > 0`).
- Bit-identical output for repeated runs with the same seed (determinism).

Extend `tests/run_validation.m` to include it. Existing `tests/test_rcflex_control.m`
invariants (current-loop tracking, `|u| ≤ 0.98·I_limit`, pitch feedback sign, determinism)
must continue to pass unchanged for **both** algorithm settings.

---

## Verification

Run in this order — note the **matlab MCP server failed to connect this session
(`CONNECTION_CLOSED`)**, so these must be run from MATLAB directly (or the server restarted)
rather than driven from here.

1. `tests/test_rcflex_intent_ekf.m` — estimator correctness in isolation, before any
   integration.
2. `tests/run_validation.m` — all 12 checks pass, including physics energy balance and the
   existing control invariants.
3. `tests/verify_rcflex_control_numerics.m` — fast integrator versus native `ode23t` must
   still agree within `joint_difference_rms_deg < 0.02` and `suppression_difference < 2`
   points at mechanical steps 0.5 / 0.25 / 0.1 ms, for the EKF algorithm. If the EKF widens
   this gap, the state propagation is sensitive to step size and must be investigated before
   trusting any benchmark number.
4. `src/control/build_rcflex_control.m` then `src/control/run_rcflex_control.m` — native
   Simulink closed-loop run on the reference case (8 Hz, 1°/axis, 5 g, seed 1701). Current
   baseline is **96.61 % suppression, 0.0339° attitude RMS** (`CONTROL_RESULTS.md`); the new
   algorithm must match or beat it. This also confirms the widened Dwork and diagnostic port
   dimensions propagate correctly through Simulink.
5. `src/control/tune_rcflex_control.m` — sweep, inspect `control_training.csv`, freeze gains.
6. `src/control/benchmark_rcflex_control.m` — both algorithms, 20 cases, identical seeds.
   **Acceptance:** all-case mean > 88 %, no case below 0 %, all 16 in-band cases within
   1 point of their current values or better, and `within_limits` true for all 20.
7. `src/control/plot_rcflex_control.m` and `src/control/write_rcflex_control_report.m` —
   regenerate `control_response.png` and `CONTROL_RESULTS.md`.
8. Flip `C.algorithm` back to `'simple_pid'`, re-run step 6, and confirm the numbers
   reproduce the existing `control_benchmark.csv` exactly. This proves the refactor did not
   perturb the control group, which is what makes the comparison table trustworthy.
