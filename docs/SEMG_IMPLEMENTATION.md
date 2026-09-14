# SensoSpoon — sEMG Subsystem Implementation Reference

Permanent technical reference for the surface-EMG subsystem. Companion documents:
`SEMG_SUBSYSTEM.md` (full requirements, hardware design and component selection) and
`CONTROL_IMPROVEMENT_PLAN.md` (the controller work this stage sits inside).

---

## 1. Purpose

The SensoSpoon controller must separate **voluntary intent** from **tremor** in handle
attitude. The IMU-only estimator does this from a single measurement, and that is
structurally hard: during a transient where intent and tremor ramp together, the model is
weakly identifiable, and a single-step innovation statistic cannot even detect the failure
(`CONTROL_IMPROVEMENT_PLAN.md` §1a).

sEMG is being investigated because it offers two things the IMU cannot:

1. **An independent tremor-frequency observable** — information about ω that does not come
   from the same measurement being decomposed.
2. **Phase lead.** Muscle electrical activity precedes movement by the electromechanical
   delay (30–50 ms), so sEMG can in principle know about a tremor burst before the IMU can.

The second is the only reason to prefer fusion over simply improving the IMU-only
estimator, and is therefore the measurement the whole programme turns on.

**This stage is sensing and characterisation only.** Nothing here reads sEMG into the
control or estimation path.

---

## 2. Current state — exact

| Item | State |
|---|---|
| `C.semg_enabled` | **`false`** by default |
| `C.semg_source` | `'synthetic'` (`'file'` implemented, `'device'` not) |
| `C.ekf_osc2` | **`false`** |
| `C.algorithm` | `'simple_pid'` |
| EKF | **unmodified**; no sEMG in the measurement model or state vector |
| sEMG/IMU fusion | **none** |
| IMU-only baseline | **unchanged** — 94.9769 % suppression, 0.050225° RMS (`sine`, 8 Hz, 1°/axis, seed 1701), verified bit-identical to the pre-sEMG code |
| Frequency-estimation experiment (§1a lag-autocorrelation) | **reverted**; packed EKF slice back to 183 elements, `info(4:5)` read zero |
| sEMG subsystem | **standalone** |
| Validation | 14/14 groups pass (`tests/run_validation.m`) |

With `C.semg_enabled = false` the system is byte-identical to the pre-sEMG code: no sEMG is
generated, no extra state is carried, and diagnostic slots 25–32 read zero.

---

## 3. Architecture

```
  ┌─────────────── SOURCE (interchangeable) ───────────────┐
  │  rcflex_semg_source.m       switch: C.semg_source      │
  │    'synthetic'  truth + sample model      [implemented]│
  │    'file'       recorded dataset          [implemented]│
  │    'device'     physical front end        [not built]  │
  └───────────────────────┬────────────────────────────────┘
                          │  raw electrode-referred volts,
                          │  n x 4, uniform 4 kHz grid
                          ▼
  ┌────────────────────────────────────────────────────────┐
  │ rcflex_semg_preprocess.m   causal, pure, per-tick      │
  │   20 Hz HP → 50 Hz notch → rectify → {3 Hz env,        │
  │                                       2-20 Hz burst}   │
  └───────────────────────┬────────────────────────────────┘
                          │  8 features per 800 Hz tick
           ┌──────────────┴──────────────┐
           ▼                             ▼
  ┌─────────────────┐          ┌──────────────────────────┐
  │ rcflex_attitude │          │ rcflex_semg_features.m   │
  │ _step (arg 6)   │          │ (offline, whole stream)  │
  │ → diagnostics   │          │   ↓                      │
  │   25:32 ONLY    │          │ characterize_rcflex_semg │
  │                 │          │ validate_..._dataset     │
  │ NOT read by the │          │ rcflex_semg_quality      │
  │ reference path  │          └──────────────────────────┘
  └─────────────────┘
           ║
  ╔════════▼═══════════════════════════════════════════════╗
  ║ UNCHANGED: EKF, attitude PID, allocator, current loop, ║
  ║ plant, IMU path, Simulink wiring                       ║
  ╚════════════════════════════════════════════════════════╝
```

The stream reaches the controller as an **optional trailing argument** to
`rcflex_attitude_step(z, sensor, H, B, C, semgBlock)` rather than as an extension of the
14-wide `sensor` vector. That keeps every Simulink port width and every existing call site
unchanged. The consequence is that the Simulink closed-loop path has no sEMG port yet; this
is noted in `rcflex_controller_sfun.m`.

---

## 4. Files

| File | Role |
|---|---|
| `src/control/rcflex_semg_source.m` | **The only place a raw stream is obtained.** Dispatches on `C.semg_source`; defines and enforces the raw-stream contract |
| `src/control/rcflex_semg_dataset_spec.m` | **The only definition of the recorded-dataset format.** Returns the contract when called with no arguments; validates when called with `(S,C)` |
| `src/model/rcflex_semg_truth.m` | Synthetic muscle + electrode model on the analogue grid |
| `src/control/rcflex_semg_sample.m` | Instrumentation: corruptions → AC coupling → gain → anti-alias → decimate → ADC |
| `src/control/rcflex_semg_init.m` | Fixed preprocessing coefficients and packed-state layout, built once |
| `src/control/rcflex_semg_preprocess.m` | Causal per-tick chain. Pure function, packed state, no persistents |
| `src/control/rcflex_semg_features.m` | Offline feature extraction over a whole stream, using the same per-tick chain |
| `src/analysis/rcflex_semg_quality.m` | Source-agnostic acquisition health check |
| `src/analysis/characterize_rcflex_semg.m` | Intent-vs-tremor characterisation against ground truth |
| `src/analysis/validate_rcflex_semg_dataset.m` | The six-stage real-data gate |
| `tests/test_rcflex_semg.m` | Sensing-chain acceptance criteria |
| `tests/test_rcflex_semg_dataset.m` | Recorded-dataset path, proved against the synthetic stream |

Modified: `rcflex_control_parameters.m` (all `C.semg_*`), `rcflex_prepare_control.m`
(`rcflex_semg_init` hook), `rcflex_control_case.m` (routes through the source),
`rcflex_attitude_step.m` (optional 6th argument, z-slice, diagnostics 25–32),
`rcflex_controller_sfun.m` (diagnostic width 24 → 32, C-derived state width),
`run_rcflex_control_fast.m` (feeds the per-tick block).

---

## 5. Signal specification

| Parameter | Value | Set by |
|---|---|---|
| Channels | 4: `[roll agonist, roll antagonist, pitch agonist, pitch antagonist]` = `[FCR, ECRL/B, FCU, ECU]` | `C.semg_channels` |
| Sample rate | **4000 Hz** | `C.semg_hz` |
| Analogue-domain rate | 16 000 Hz (4× oversample), decimated after anti-aliasing | `C.semg_oversample` |
| Signal band | 20–450 Hz | `C.semg_band_lo_Hz` / `_hi_Hz` |
| Amplitude | 0.7 mV rms, ~6.8 mV peak, crest factor 7–10 | measured |
| Electrode DC offset | ±15 mV (≈10× the signal) | `C.semg_offset_V` |
| Analogue gain | **200** (bound 231) | `C.semg_gain` |
| ADC | 12-bit, ±1.65 V → ±8.25 mV electrode-referred, LSB 4.03 µV | `C.semg_adc_*` |

Gain is set from the **peak**, not the rms — the crest factor is 7–10, so an rms-derived
gain clips. Measured: zero clipping across 6 scenarios × 6 seeds at gain 200, with 3.4–10.0
dB headroom; clipping begins at 300.

The ADC LSB (4.03 µV) sits below the 8 µV noise floor, so the system is **noise-limited, not
quantisation-limited**. 10 bits is the formal floor; 12 is comfortable. A 24-bit
biopotential converter is explicitly *not* required — see `SEMG_SUBSYSTEM.md` §1.6.

---

## 6. Sampling rate and synchronisation

`C.semg_hz = 4000 = 5 × C.outer_hz`.

The integer ratio **is** the synchronisation strategy. Control tick *j* consumes sEMG
samples `(j−1)·5+1 … j·5`. There is no resampling, no timestamp matching and no accumulated
drift — alignment is structural. `rcflex_semg_init.m` asserts the ratio, so it cannot be
broken silently by changing either rate.

Admissible rates are integer multiples of 800 Hz that also satisfy Nyquist for the 450 Hz
band: 1600, 2400, 3200, 4000, 4800. 4000 Hz is chosen because it is 4.4× the band edge,
leaving a realistic transition band for a 2-pole analogue anti-alias filter.

For hardware this implies **one clock domain**: the sEMG ADC and the IMU must be sampled by
the same MCU from the same timebase, with jitter ≪ 1.25 ms. Two free-running USB devices
would drift and break the fixed-block assumption. Because the ~35 ms electromechanical lead
is the quantity being measured, any fixed pipeline delay between the streams biases it
directly — timestamp both at acquisition.

---

## 7. Preprocessing pipeline

Per sEMG sample, causal, in `rcflex_semg_preprocess.m`:

```
20 Hz highpass  →  50 Hz notch  →  full-wave rectify  →  ┬→ 3 Hz lowpass    = env
                                                          └→ 2-20 Hz bandpass = burst
```

- The **highpass leads** the chain: the half-cell offset would otherwise dominate every
  downstream stage including the rectifier.
- The **notch precedes rectification**: mains sits inside the EMG band, and rectifying first
  would fold it into the envelope irrecoverably.
- **Rectification is what moves the information to baseband.** The features live at 2–20 Hz
  but the signal must be acquired at 20–450 Hz — it cannot be acquired at 20 Hz directly.
  This is why a hardware module that outputs a pre-rectified envelope is unusable here.
- Features are evaluated at the **last sEMG sample of each tick**, so the output is causal
  and carries no lookahead.

State is packed into a plain vector; the step is a pure function with no persistents, under
the same determinism contract as the EKF step.

`env` is the slow activation envelope (voluntary effort level). `burst` is the tremor-band
modulation of that envelope — the discriminating feature, and the one that leads the IMU.

---

## 8. Synthetic source

`rcflex_semg_truth.m`. **Reproducible synthetic scenario, NOT a clinical recording** — the
same caveat `rcflex_tremor_input.m` carries.

Surface EMG is modelled as the interference pattern of many motor units: a band-limited
zero-mean Gaussian process amplitude-modulated by muscle activation (Hogan & Mann 1980;
De Luca 1997):

```
emg(t) = activation(t) · w(t),   w = 20-450 Hz band-limited, unit RMS
```

Activation follows the joint torque the muscle must supply. At ~0.2 Hz the voluntary
component is dominated by the posture term, so it follows the intent **angle**; at 4–20 Hz
the tremor component is dominated by the inertial term, so it follows tremor
**acceleration**. Each axis has an agonist/antagonist pair: a signed drive loads one and
unloads the other over a co-contraction floor that rises with voluntary effort.

The electromechanical delay is modelled **explicitly** as `C.semg_emd_s = 40 ms` of lead,
because it is the property the whole investigation rests on.

Inputs are the `intent` and `tremor` components that `rcflex_control_case.m` already keeps
separated, so the ground-truth decomposition the controller must infer is known exactly.

`rcflex_semg_sample.m` then applies, in physical order:

```
corruptions → AC coupling → gain → anti-alias → decimate → quantise/clip
```

Corruptions: electrode DC offset, 50 Hz mains, motion artefact **deliberately placed inside
the tremor band (2–9 Hz)** so characterisation cannot flatter the signal, and a broadband
noise floor.

Two orderings are requirements, not preferences:

- **AC coupling before gain.** The offset is ~10× the signal; at gain 200 it alone demands
  3.0 V of a ±1.65 V rail and rails the channel solid — per channel, since each electrode
  draws its own offset.
- **Anti-alias before sampling.** The analogue domain is modelled at 4× the final rate and
  decimated after filtering. Generating directly at 4 kHz would assume the band limiting
  this stage exists to enforce. Measured effect of enabling the filter: out-of-band residual
  8147 → 3126 nV/√Hz (−8.3 dB), in-band power fraction 0.756 → 0.908.

---

## 9. Real-data interface

`C.semg_source = 'file'`, contract in `rcflex_semg_dataset_spec.m` (call with no arguments
to obtain it programmatically).

**Required**

| Field | Type | Notes |
|---|---|---|
| `semg` | n × 4 | **Electrode-referred volts.** Not ADC counts, not gain-included |
| `fs` | scalar | Hz, ≥ 900. Regridded to 4000 Hz by the loader |

**Optional — each unlocks specific validation stages**

| Field | Unlocks |
|---|---|
| `imu_accel` (n × 2, tremor **acceleration**), `imu_fs` | stages 4, 5 |
| `labels` (`t_start`, `t_end`, `class` ∈ rest/tremor/intent/both) | stages 2, 3, 6 |
| `t0`, `channels`, `subject`, `session`, `device`, `notes` | provenance |

The spec actively rejects what real datasets actually get wrong: transposed matrices,
gain-included or ADC-count units, wrong channel count, sample rates too low for the band,
invalid label classes. Silent acceptance would produce plausible features over a partly
empty band — which presents as weak physiology rather than as a wrong recording.

A synthetic scenario exported to this format and read back produces **bit-identical
features**, so any later difference on a real recording is a property of the recording and
not of the plumbing (`tests/test_rcflex_semg_dataset.m`).

---

## 10. Physical sensor concept

Recommended build (`SEMG_SUBSYSTEM.md` §3, Option A):

```
Ag/AgCl gel electrodes (20 mm bipolar, SENIAM placement)
        ↓
4 × MyoWare 2.0 Muscle Sensor  — RAW output, gain toward the low end
        ↓
Teensy 4.1 — 12-bit ADC, 4 ch @ 4 kSPS, hardware-timer clocked,
             hosting the IMU on the SAME clock
        ↓  USB
Host → rcflex_semg_source.m ('device') → existing pipeline unchanged
```

≈ NZ$250–350 plus consumables. The design risk collapses to two things: gain setting and
synchronisation.

**Key hardware assumptions**

- AC coupling precedes gain (§8). Non-negotiable.
- An analogue anti-alias filter (≈480 Hz, ≥2-pole) exists before the ADC.
- The AC-coupling stage is allowed to **settle before recording** — a few hundred ms at a
  10 Hz corner — and the first second of any capture is discarded.
- Gain 150–200, set from the peak.
- CMRR ≥ 80 dB at 50 Hz; input impedance ≥ 10 MΩ with gel electrodes.
- MyoWare's default envelope output is rectified in hardware and is **unusable**; the raw
  output is required (§7).
- Body-connected instrument: battery power or a medically-rated USB isolator.

Electrode sites (SENIAM; Hermens et al. 2000): FCR, ECRL/B, FCU, ECU, reference on the ulnar
styloid. 20 mm spacing, over the muscle belly between motor point and tendon, skin prepared
so contact impedance is **matched** between the electrodes of a pair — CMRR is limited by
mismatch, not absolute impedance.

---

## 11. Characterisation results

Synthetic source, 4 s scenarios, 8 Hz tremor where present
(`output/results/semg_characterization.csv`):

| Quantity | Result |
|---|---|
| Burst-band SNR, tremor vs tremor-free | **+8.0 to +12.7 dB** |
| Burst dominant frequency vs true 8 Hz | **7.81–9.38 Hz**; tremor-free case reads noise |
| Burst lead over mechanical tremor | **+35 to +37.5 ms** (modelled EMD 40 ms; the shortfall is burst-bandpass group delay) |
| Envelope vs voluntary intent | `combined` axis 2 **r = −0.69** |
| Envelope vs tremor magnitude | **\|r\| ≤ 0.40** |
| Acquisition health | 0 % clipping, 4.6–6.3 dB headroom, band fraction 0.89–0.90, mains −12 to −13 dB |

Six-stage gate on a multi-trial synthetic protocol: SNR 8.5 dB, raw cross-talk 0.044, lead
35.0 ms, frequency agreement 0 Hz, discrimination 11.0 — all pass.

> **The stage 3 synthetic value is not a cross-talk reference.** The model generates its four
> channels independently, so it contains essentially zero cross-talk *by construction*; 0.044
> is a measurement of that assumption, not a prediction about hardware. The superseded value
> of **0.19** was produced by the old pooled *envelope* metric and must no longer be quoted as
> a cross-talk reference for any purpose — see §12.2.
>
> The same synthetic stream reports **coactivation_corr ≈ 0.78–0.85** (stage 3b) despite
> containing no cross-talk at all, which is precisely why envelope correlation cannot be used
> as the cross-talk gate.

**Conclusions supported (on the synthetic source as modelled):** the discriminating feature
is the **burst band, not the envelope**; sEMG carries an independent frequency observable;
and it carries phase information the IMU does not yet have.

The lead is measured against tremor **acceleration**, not angle. Muscle force drives
acceleration; for a sinusoid the angle is 180° away, so correlating against angle reports
the true lead minus half a tremor period (at 8 Hz, +40 ms reads as −22.5 ms).

---

## 12. Limitations and assumptions

Ordered by how much they could cost.

1. **The synthetic source is scored against the ground truth that generated it.** These
   results show the preprocessing chain *recovers structure that was put in*. They are not
   evidence about real physiology. Every number in §11 should be read as an upper bound.

2. **Zero cross-talk is assumed.** The model generates the four channels independently, so
   any cross-talk figure it produces (now |r| = 0.044 on the raw metric) is an **artefact of
   the model, not a prediction**. This remains the largest untested assumption in the design,
   and on real data it does **not** hold uniformly — see `SEMG_DATASET.md` §6.

   Two distinct quantities were conflated here until the GRABMyo run, and the distinction is
   now load-bearing:

   | | Measured on | Physical meaning | Role |
   |---|---|---|---|
   | **Stage 3, raw cross-talk** | band-limited **raw waveform**, zero lag | same motor-unit potentials volume-conducted into both electrodes, plus residual common-mode — an **acquisition defect** | **GATE**, \|r\| < 0.30 |
   | **Stage 3b, `coactivation_corr`** | **envelope** | both muscles genuinely driven together — a **physiological observation** | diagnostic, never gates |

   Volume conduction is quasi-static at EMG frequencies, so real cross-talk appears in the raw
   waveform at zero lag. Independent motor-unit pools mean co-contraction leaves the raw
   waveforms uncorrelated while their envelopes track each other closely. The synthetic source
   demonstrates the separation directly: zero cross-talk by construction, raw \|r\| = 0.044,
   envelope \|r\| = 0.78–0.85.

   The superseded metric correlated **envelopes** over the **whole pooled record**, and both
   choices were wrong. Envelope correlation measures shared drive rather than coupling; pooling
   flexion, extension and rest averages relationships of opposite sign and cancels to near zero
   regardless of the underlying cross-talk. Together they produced a confident PASS on a
   quantity that had never actually been measured.

3. **Constant electromechanical delay.** Real EMD varies with contraction level, fatigue and
   muscle. The +35 ms lead is therefore an optimistic bound on how reliable the phase
   advantage is.

4. **Not modelled at all:** electrode shift, fatigue-driven spectral compression, ECG
   contamination on proximal sites, mains harmonics at 100/150 Hz (only 50 Hz is notched),
   and CMRR / common-mode behaviour (the model is single-ended and has no common-mode
   concept).

5. **The plant model has no hysteresis, dead zone or creep**, per the project's standing
   caveat — this is an algorithm-development baseline, not a validated digital twin.

---

## 13. Deliberately not implemented

| Not done | Why |
|---|---|
| **IMU + sEMG fusion** | Requires the §14 evidence first |
| **Any EKF change** | Measurement model and state vector untouched until real data says what to fuse |
| **`ekf_osc2 = true`** | Infrastructure exists and is tested; stays off |
| **Controller / reference-path changes** | sEMG is not read by the reference path |
| **Simulink sEMG port** | Would need a `.slx` change; deferred until the fusion design is decided. Noted in `rcflex_controller_sfun.m` |
| **`C.semg_source = 'device'`** | No physical front end built |
| **Frequency-estimation observable (§1a)** | Reverted; superseded as a question by sEMG |

---

## 14. Next stage: standalone real-sEMG validation

No fusion work should begin until this gate passes. Run with

```matlab
C = rcflex_control_parameters; C.semg_enabled = true;
C = rcflex_prepare_control(C);
R = validate_rcflex_semg_dataset('recording.mat', C);
```

| # | Stage | Criterion | Synthetic reference |
|---|---|---|---|
| 1 | Acquisition health | clip < 0.1 %, headroom ≥ 0 dB, band fraction > 0.5, mains < −6 dB | 0 %, 4.6–6.3 dB, 0.89, −12.9 dB |
| 2 | Burst SNR | **> 6 dB** | 8.5 dB |
| 3 | Raw cross-talk | \|r\| < 0.30 **(provisional)** | 0.044 *(artefact — see §12.2)* |
| 4 | sEMG lead | 10–100 ms | +35 ms |
| 5 | Frequency agreement | < 1 Hz | 0 Hz |
| 6 | Intent/tremor discrimination | burst/envelope ratio > 2 | 11.0 |
| 3b | `coactivation_corr` | **no threshold — diagnostic** | 0.78–0.85 |

### Stage 3 — formal definition

For each axis (roll = channels 1–2, pitch = channels 3–4) and each labelled **isolated
contraction** segment — the cued isolated flexion / extension trials, logged with
`class = 'intent'` per `SEMG_SUBSYSTEM.md` §6.3:

1. Discard the first **1.5 s** of the segment, excluding the contraction ramp and any filter
   transient at a trial join. Segments leaving < 0.25 s are not scored.
2. Band-limit the raw electrode-referred channels to
   `[C.semg_band_lo_Hz, C.semg_band_hi_Hz]` (zero-phase, so the lag diagnostic is unbiased).
3. Compute the **zero-lag** Pearson correlation `r` between the two band-limited waveforms.

```
stage 3 = max over { all isolated segments } x { roll, pitch } of |r|
```

**Worst case gates**, because cross-talk is a property of electrode placement and one badly
sited pair breaks the agonist/antagonist feature layout that a median would hide. The
**median** is reported alongside as a diagnostic, and **every segment's value is written to
`output/results/semg_stage3_segments.csv`** so the threshold can be reassessed later against
our own recordings.

**Rest, `tremor` and `both` segments cannot score this stage.** Rest carries no signal to
correlate and the others are not isolated contractions. With no usable segment the stage
reports `SKIPPED`, never a pooled fallback.

**The 0.30 threshold is provisional.** It is an engineering gate derived from the
distance-decay structure of volume conduction and the residual common-mode floor of a
shared-reference montage (`SEMG_DATASET.md` §6.2) — **not** a clinically established
universal cross-talk limit. It must not be retuned to make a dataset pass.

**Cross-correlation peak lag** is recorded per segment as a diagnostic. The gate uses zero lag
regardless. A correlation peaking away from zero lag is *not* volume-conducted cross-talk and
must not be reinterpreted as such.

Stages report `PASS`, `FAIL` or **`SKIPPED`**. `SKIPPED` is deliberately distinct from
`PASS`: a stage that could not run has told you nothing, and reading an absent measurement
as a satisfied one is the most expensive mistake available here.

Stages 2 and 6 compare conditions, so a single continuous recording of one condition cannot
answer them. The recording protocol — cued rest / tremor / voluntary / combined trials plus
isolated flexion and extension, with boundaries logged at record time — is in
`SEMG_SUBSYSTEM.md` §6.3, and `protocol_fixture` in `tests/test_rcflex_semg_dataset.m` builds
exactly that shape from synthetic trials.

**Thresholds must not be relaxed to make a dataset pass.** A failing stage is the result.

For the dataset selected for this stage, its licence, contents and which stages it can
support, see `docs/SEMG_DATASET.md`.

---

## References

- De Luca, C. J. (1997). The use of surface electromyography in biomechanics. *Journal of
  Applied Biomechanics*, 13(2), 135–163.
- Hermens, H. J., Freriks, B., Disselhorst-Klug, C., & Rau, G. (2000). Development of
  recommendations for SEMG sensors and sensor placement procedures. *Journal of
  Electromyography and Kinesiology*, 10(5), 361–374.
- Hogan, N., & Mann, R. W. (1980). Myoelectric signal processing: optimal estimation applied
  to electromyography. *IEEE Trans. Biomedical Engineering*, 27(7), 382–395.
- Cavanagh, P. R., & Komi, P. V. (1979). Electromechanical delay in human skeletal muscle
  under concentric and eccentric contractions. *European Journal of Applied Physiology*,
  42(3), 159–163.
- Merletti, R., & Parker, P. A. (Eds.) (2004). *Electromyography: Physiology, Engineering and
  Non-Invasive Applications*. IEEE Press/Wiley.
- Zhou, Y., Jenkins, M. E., Naish, M. D., & Trejos, A. L. (2018). Multi-sensor fusion for
  tremor estimation.
