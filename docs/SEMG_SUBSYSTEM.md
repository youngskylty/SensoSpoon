# SensoSpoon — Standalone sEMG Subsystem: Requirements and Physical Design

## Scope and status

This document specifies the sEMG subsystem as a **standalone** stage: the signal
requirements, a physically buildable sensor, and the tests that must pass before sEMG is
allowed anywhere near the estimator.

It deliberately stops short of fusion. The EKF is unmodified, `C.ekf_osc2` remains `false`,
the reference path does not read sEMG, and the IMU-only controller remains the verified
baseline (`C.semg_enabled = false` reproduces the pre-sEMG system bit-identically). The
frequency-estimation experiment of `CONTROL_IMPROVEMENT_PLAN.md` §1a remains reverted.

Every number below is derived from what the software already assumes
(`rcflex_control_parameters.m`) or measured from the synthetic stream it produces, not
imported from general sEMG practice. Where a requirement is *not* driven by our pipeline it
is marked **[not required]** so it does not get built by accident.

---

## 1. Signal requirements

### 1.1 Summary table

| Parameter | Requirement | Driven by |
|---|---|---|
| Channels | **4** differential + 1 reference | `C.semg_channels`; two agonist/antagonist pairs, one per controlled axis |
| Sample rate | **4000 Hz**, must be an integer multiple of 800 Hz | `C.semg_hz`; `rcflex_semg_init.m` asserts the ratio |
| Minimum usable rate | 1600 Hz (= 2 × 800, ≥ 2 × 450 Hz) | Nyquist on the EMG band **and** the sync constraint |
| Signal bandwidth | **20–450 Hz** | `C.semg_band_lo_Hz` / `C.semg_band_hi_Hz` |
| Input amplitude, typical | **0.7 mV rms** differential | measured, `rcflex_semg_quality` |
| Input amplitude, peak | **≈ 6.8 mV** worst case over 20 scenario/seed combinations | measured |
| Crest factor | **≈ 7–10** | measured; this, not the rms, sets the gain |
| Electrode DC offset | up to **±15 mV**, ~10× the signal | `C.semg_offset_V` |
| Analogue gain | **200** (upper bound **231**) | 1.65 V rail ÷ 7.13 mV worst-case peak; measured zero-clipping |
| ADC | **≥ 10 bits**, 12 bits recommended, ±1.65 V (or 0–3.3 V) | `C.semg_adc_bits` / `C.semg_adc_range_V` |
| Synchronisation | common time base with the IMU, jitter ≪ 1.25 ms | the 5-samples-per-tick contract |
| CMRR | **≥ 80 dB** at 50 Hz | 1 V body common-mode vs the 0.2 mV modelled mains level |
| Input impedance | ≥ 10 MΩ (gel), ≥ 100 MΩ if dry electrodes are used | electrode source impedance and mismatch |

### 1.2 Sampling rate — why 4000 Hz specifically

Two constraints intersect, and only their intersection is admissible:

1. **Nyquist on the signal.** The EMG band ends at 450 Hz, so ≥ 900 Hz.
2. **The synchronisation contract.** `C.semg_hz` must be an integer multiple of
   `C.outer_hz = 800 Hz`, because the entire alignment scheme is that control tick *j*
   consumes sEMG samples `(j−1)·sub+1 … j·sub` with no resampling.

Admissible rates are therefore 1600, 2400, 3200, 4000, 4800 Hz. **4000 Hz** is chosen: it is
4.4× the band edge, which leaves a realistic transition band for a practical analogue
anti-alias filter, and it is a rate every candidate ADC reaches comfortably.

1600 Hz would satisfy both constraints but gives only a 450 → 800 Hz transition band, which
demands a steeper anti-alias filter than the rest of the design warrants. **[not required]:
rates above 4800 Hz.** Nothing in the pipeline uses content above 450 Hz.

### 1.3 Bandwidth

20–450 Hz. The lower edge is set by motion-artefact and half-cell-offset rejection rather
than by physiology; the upper by motor-unit action-potential content (De Luca 1997). Note
the *features* live far below this: the envelope is a 3 Hz lowpass and the burst feature a
2–20 Hz bandpass, both computed on the **rectified** signal. The 20–450 Hz band must be
acquired faithfully because rectification is what moves the information down to those
bands — it cannot be acquired at 20 Hz directly.

**[not required]:** content below 20 Hz. It is highpassed away twice (once in analogue at
10 Hz, once digitally at 20 Hz) and carries motion artefact, not EMG.

### 1.4 Amplitude, gain and the clipping constraint

This is the requirement most easily got wrong, and the model got it wrong twice before
the checks below existed.

Surface EMG is an interference pattern with a **high crest factor** — measured at 7–10 here.
Gain must be set from the peak, not the rms:

```
max gain = ADC half-range / worst-case peak
         = 1.65 V / (6.78 mV signal + 0.35 mV mains+artefact+noise)
         = 1.65 / 7.13e-3
         = 231
```

**Adopted analogue gain: 200** (`C.semg_gain`), giving ±8.25 mV of input range.

A gain sweep over 6 scenarios x 6 seeds, scored with `rcflex_semg_quality`, confirms the
analytical bound exactly:

| Gain | max clip % | headroom (dB) |
|---|---|---|
| 150 | 0.0000 | 4.10 – 9.94 |
| 175 | 0.0000 | 2.76 – 8.61 |
| **200** | **0.0000** | **1.60 – 7.45** |
| 231 | 0.0000 | 0.35 – 6.20 |
| 250 | 0.0062 | 0.00 – 5.51 |
| 500 | 0.9687 | 0.00 – 0.00 |

Clipping begins at 250, one step above the computed bound of 231. Gain 200 is adopted: it
uses the converter well while keeping every channel clear of the rail in every case tested.

> **Resolved defect.** The model previously ran `C.semg_gain = 500` and hard-clipped 0.4–1.5 %
> of samples. The characterisation survived it — the features are envelope-based and tolerant
> of rare clipping — which is exactly why it went unnoticed and why `rcflex_semg_quality` now
> exists.

> **A second defect surfaced while fixing the first, and it had been masking it.** The
> AC-coupling filter in `rcflex_semg_sample.m` was started from **zero state**, so the ±15 mV
> electrode offset arrived as a step at *t* = 0 and rang through the highpass at nearly full
> offset amplitude — ~14.6 mV against a 3.3 mV full scale. That railed every channel briefly
> *regardless of gain*, which is why halving the gain barely moved the clip rate and made the
> problem look unfixable. The filter is now initialised to the settled state for the initial
> DC level, which is also the physically correct condition: a real front end is powered up and
> settled long before acquisition begins.
>
> **Consequence for the hardware:** allow the AC-coupling stage to settle before recording —
> with a 10 Hz corner that is a few hundred milliseconds — and discard the first second of
> any capture.

### 1.5 The DC offset problem, and why it is an architecture constraint

The electrode half-cell potential (±15 mV) is roughly **ten times the EMG itself**. At any
usable gain it saturates the converter: at gain 200 it alone demands 3.0 V of a ±1.65 V rail.

This is not a filtering detail, it is an **ordering constraint on the architecture**: the
signal must be AC-coupled (or offset-corrected by a DC servo) **before** the main gain stage.
`C.semg_ac_couple_Hz = 10 Hz` models this. Getting the order wrong does not degrade the
signal gracefully — it rails the channel solid, and it does so *per channel*, because each
electrode draws its own offset. During development this killed two of four channels while
the other two looked healthy.

### 1.6 ADC requirements

With gain 200 and a ±1.65 V converter:

| | Value |
|---|---|
| Full scale, electrode-referred | ±8.25 mV |
| 12-bit LSB, electrode-referred | 4.03 µV |
| Modelled noise floor | 8 µV rms |

The noise floor sits at ~2 LSB, so the system is **noise-limited, not quantisation-limited**
— which is the correct place to be. Formally, putting 1 LSB at the noise floor needs
`log2(3.3 V / (8 µV × 200)) ≈ 11.0 bits`, so **10 bits is the floor and 12 bits is
comfortable**.

**[not required]: a 24-bit biopotential converter.** A 24-bit front end (ADS1299 and
similar) exists to acquire EEG-scale signals *without* an analogue gain stage and to absorb
the DC offset digitally. Our signal is millivolt-scale and we are already AC-coupling, so the
extra 12 bits buy nothing this pipeline can use. It is listed as an option in §3 only because
it simplifies the board, not because the signal requires it.

### 1.7 Filtering requirements

| Stage | Where | Requirement |
|---|---|---|
| AC coupling | analogue, **before gain** | 1-pole ≈ 10 Hz. Mandatory (§1.5) |
| Anti-alias | analogue, **before ADC** | ≥ 2-pole, −3 dB ≈ 450–500 Hz. Mandatory |
| Mains notch | digital | 50 Hz (NZ), ≈ 4 Hz wide |
| Band definition | digital | 20 Hz highpass |
| Envelope | digital | rectify → 3 Hz lowpass |
| Burst | digital | rectify → 2–20 Hz bandpass |

Only the first two must be analogue. Everything from the 20 Hz highpass onward is already
implemented in `rcflex_semg_preprocess.m` and **must not be duplicated in hardware** — a
sensor module that outputs a pre-rectified envelope (several hobby modules do by default)
destroys the burst feature, because rectification must happen after the mains notch and at
full bandwidth.

> **Gap:** the anti-alias filter is a hardware requirement with **no counterpart in the
> model**. `rcflex_semg_sample.m` models AC coupling and the ADC but no anti-alias stage,
> because the synthetic truth is band-limited to 450 Hz by construction and has nothing to
> alias. Real EMG has content above 450 Hz and the amplifier contributes wideband noise. See
> §4.2.

### 1.8 Synchronisation requirements

The pipeline assumes exactly `C.semg.sub = 5` sEMG samples per 800 Hz control tick, aligned
to the tick boundary. Practically:

- **One clock domain.** The sEMG ADC and the IMU must be sampled by the same MCU from the
  same timebase. Two free-running USB devices will drift and the fixed-block assumption
  silently breaks.
- **Jitter ≪ 1.25 ms** (one sEMG sample period). A hardware timer interrupt meets this
  easily; a polled `delay()` loop does not.
- **The ~35 ms electromechanical lead is the quantity being measured**, so any fixed pipeline
  delay between the two streams biases it directly. Timestamp both streams at acquisition,
  not at arrival.
- If a source cannot deliver a uniform 4 kHz grid, it must resample **inside**
  `rcflex_semg_source.m` before returning. That contract is documented in the file.

### 1.9 Electrode placement (forearm/wrist)

Four bipolar channels and one reference, targeting the muscles that produce wrist flexion/
extension (our roll axis) and radial/ulnar deviation (our pitch axis):

| Channel | Muscle | Site |
|---|---|---|
| 1 — roll agonist | Flexor carpi radialis | anterior forearm, ~⅓ from medial epicondyle toward the radial styloid |
| 2 — roll antagonist | Extensor carpi radialis longus/brevis | posterior forearm, ~⅓ from lateral epicondyle |
| 3 — pitch agonist | Flexor carpi ulnaris | anteromedial forearm, along the ulnar border |
| 4 — pitch antagonist | Extensor carpi ulnaris | posterior forearm, along the ulnar border |
| Reference | — | ulnar styloid or olecranon (electrically quiet bone) |

Following SENIAM recommendations (Hermens et al. 2000):

- Bipolar pairs at **20 mm** centre spacing, aligned with the muscle fibre direction.
- Placed over the **muscle belly, between the motor point and the tendon** — not over the
  motor point itself, where the signal reverses and amplitude is unstable.
- Skin prepared (shave if needed, abrade lightly, alcohol wipe) to bring contact impedance
  below ~10 kΩ and, more importantly, to make it *matched* between the two electrodes of a
  pair. CMRR is limited by impedance *mismatch*, not absolute impedance.
- **Cross-talk is the main placement risk here.** The forearm flexors and extensors are
  closely packed, so channels 1/3 and 2/4 will share signal. Our agonist/antagonist features
  assume reciprocal activation, so cross-talk directly attenuates the discrimination the
  pipeline relies on. Quantify it (§5.4) rather than assume it away.
- Cable strain relief matters more than usual: motion artefact is modelled at **2–9 Hz**,
  which is *inside* the tremor band, so a swinging cable is indistinguishable from tremor by
  band alone.

---

## 2. Physical sensor block diagram

```
        ┌──── per channel, ×4 ──────────────────────────────────────────┐
        │                                                               │
 ┌────────────┐   ┌──────────────┐   ┌──────────────┐   ┌────────────┐  │
 │ Ag/AgCl    │   │ Instrument-  │   │ AC coupling  │   │ Gain       │  │
 │ gel elec-  │──▶│ ation amp    │──▶│ 1-pole 10 Hz │──▶│ ×200 total │──┼─┐
 │ trode pair │   │ G≈10, CMRR   │   │ **before**   │   │ total      │  │ │
 │ 20 mm      │   │ ≥80 dB       │   │ the gain     │   │            │  │ │
 └────────────┘   └──────────────┘   └──────────────┘   └────────────┘  │ │
        │                                                               │ │
        └───────────────────────────────────────────────────────────────┘ │
                                                                          │
 ┌────────────┐                          ┌──────────────────────────┐     │
 │ Reference  │─────────────────────────▶│ Anti-alias LPF           │◀────┘
 │ electrode  │   (bias / driven ref)    │ ≥2-pole, fc ≈ 450–500 Hz │
 │ ulnar sty. │                          │ ANALOGUE, mandatory      │
 └────────────┘                          └───────────┬──────────────┘
                                                     │
                                         ┌───────────▼──────────────┐
                                         │ ADC, ≥10-bit (12 rec.)   │
                                         │ 4 ch @ 4000 Hz           │
                                         │ hardware-timer clocked   │
                                         └───────────┬──────────────┘
                                                     │
 ┌───────────────────┐                   ┌───────────▼──────────────┐
 │ IMU (existing)    │──── same clock ──▶│ MCU: single timebase     │
 │ 800 Hz            │                   │ 5 sEMG samples per       │
 └───────────────────┘                   │ 800 Hz tick, timestamped │
                                         └───────────┬──────────────┘
                                                     │ USB / serial
                                         ┌───────────▼──────────────┐
                                         │ HOST                     │
                                         │ rcflex_semg_source.m     │
                                         │   ('device')             │
                                         └───────────┬──────────────┘
                                                     │ raw electrode-referred volts
                                         ┌───────────▼──────────────┐
                                         │ UNCHANGED SOFTWARE:      │
                                         │ rcflex_semg_preprocess   │
                                         │ rcflex_semg_features     │
                                         │ rcflex_semg_quality      │
                                         │ characterize_rcflex_semg │
                                         └──────────────────────────┘
```

The boundary that matters is the last one: **the hardware's only job is to deliver raw
electrode-referred volts on a uniform 4 kHz grid.** Everything below that line already
exists and is source-agnostic.

---

## 3. Recommended components

Two builds are given. **Option A is the recommendation** for demonstrating the subsystem.

> Availability and pricing move; confirm both at purchase time. Prices are order-of-magnitude
> indications only.

### Option A — Integrated modules (recommended: simplest credible build)

| Block | Part | Qty | Notes |
|---|---|---|---|
| Electrodes | Ag/AgCl pre-gelled disposable, 24 mm (e.g. Covidien Kendall 530 / 3M Red Dot 2560) | ~100 | Consumable. Pre-gelled avoids mixing and gives repeatable impedance |
| Leads | Snap-to-2.54 mm lead wires, shielded | 5 | Shielding matters — see §1.9 on cable artefact |
| AFE | **MyoWare 2.0 Muscle Sensor** | 4 | **Use the RAW output, not the envelope output.** Integrates INA, AC coupling, gain and band limiting. Adjustable gain — set toward the low end per §1.4 |
| ADC + MCU | **Teensy 4.1** (or 4.0) | 1 | 12-bit ADC, comfortably 4 kSPS × 4 ch, hardware-timer clocked, and fast enough to host the IMU on the same timebase — which is the whole synchronisation requirement in one part |
| IMU | existing BMI270 | 1 | Already in the design; must share the Teensy's clock |
| Power | USB, or 2× 9 V / LiPo for isolation | — | See the safety note below |

Approximate cost: **NZ$250–350** plus consumables.

Why this is the right level of engineering: MyoWare 2.0 is exactly the AFE block of §2 in one
module, and the Teensy is exactly the "one clock domain" requirement in one module. The
design risk collapses to two things we actually care about — gain setting and
synchronisation.

**The one trap:** MyoWare's default/envelope output is rectified and smoothed in hardware.
That output is unusable here — it destroys the burst feature, because our chain must rectify
*after* the mains notch at full bandwidth (§1.7). Take the RAW output.

### Option B — Discrete analogue front end (better signal, more build effort)

| Block | Part | Notes |
|---|---|---|
| Instrumentation amp | **TI INA333** (or AD8237) | CMRR ≈ 100 dB+, low noise, single supply. G ≈ 10 in the first stage |
| AC coupling + gain | OPA2333 / OPA2376 dual op-amp | 10 Hz highpass then ×20, total 200 |
| Anti-alias | same op-amp, 2-pole Sallen–Key, fc 480 Hz | Explicit, unlike Option A where it is internal |
| Right-leg drive | third op-amp section | Actively drives the reference to improve mains CMRR |
| ADC | **TI ADS8688** (16-bit, 8-ch, SPI) or the Teensy's internal ADC | |
| MCU | Teensy 4.1 | as above |

Use Option B if mains rejection in the lab proves marginal, or if the report needs the
analogue design shown explicitly rather than delegated to a module.

### Option C — Integrated biopotential AFE **[not required]**

**TI ADS1299** (8-ch, 24-bit, built-in PGA and bias drive) or an OpenBCI Cyton board. This is
the medical-grade route. It is listed only to be explicitly rejected: §1.6 shows the extra
resolution is unusable by this pipeline, and it roughly triples the cost. Choose it only if
an ADS1299-class board is already available to borrow.

### Safety note

This is a body-connected instrument. Either **run the front end from batteries** and couple
to the host optically/wirelessly, or use a **medically-rated USB isolator** (e.g. an
ADuM3160-based module) between the MCU and the PC. Never connect a mains-powered,
non-isolated PC directly to body electrodes. Follow the course's ethics/safety approval for
recording from a person, including your own arm.

---

## 4. Correspondence between the simulation and the hardware

| MATLAB | Hardware block | Status |
|---|---|---|
| `rcflex_semg_truth.m` | the muscle (signal source) | Synthetic stand-in. Replaced by reality, never built |
| `C.semg_offset_V` | electrode half-cell potential | ✅ modelled |
| `C.semg_mains_Hz/_V` | 50 Hz pickup | ✅ modelled |
| `C.semg_artifact_V`, 2–9 Hz | cable/electrode motion artefact | ✅ modelled, and deliberately in-band |
| `C.semg_noise_V` | amplifier + electrode noise | ✅ modelled as broadband |
| `C.semg_ac_couple_Hz = 10` | AC coupling **before** gain | ✅ modelled |
| `C.semg_gain = 200` | analogue gain | ✅ fixed; zero clipping, 1.6–7.5 dB headroom |
| AC-coupling settled at start | front end powered up before capture | ✅ fixed; see §1.4 |
| — | **analogue anti-alias filter** | ❌ **§4.2 — absent from the model** |
| `C.semg_adc_bits/_range_V` | ADC | ✅ modelled, with saturation |
| `C.semg_hz`, `C.semg.sub` | sample clock and tick alignment | ✅ modelled and asserted |
| `rcflex_semg_source.m` | the acquisition boundary | ✅ seam exists; `'device'` not implemented |
| `rcflex_semg_preprocess.m` | **stays in software** | ✅ must NOT be moved into hardware |
| — | CMRR, input impedance, right-leg drive | ❌ not modelled; single-ended model has no common-mode concept |

### 4.1 Gain — resolved

`C.semg_gain` is now 200, within the derived bound of 231, with zero clipping measured across
36 scenario/seed combinations. The characterisation was regenerated and every conclusion held
(SNR 7.7–12.4 dB, lead 35–37.5 ms, burst frequency 7.81–8.20 Hz, separation ratio 10.0×).

That the conclusions barely moved is itself useful evidence: the features are robust to this
class of acquisition defect, so such defects have to be caught by an explicit check rather
than by noticing bad results. That is what `rcflex_semg_quality` is for, and it is why §5.1
runs first on any real recording.

### 4.2 Missing anti-alias stage

`rcflex_semg_sample.m` has no anti-alias filter because the synthetic truth is band-limited
to 450 Hz and nothing can alias. Real acquisition can and will. Consequence: the model
currently cannot show what aliasing would do to the features, so it cannot be used to
justify skipping the analogue filter. Adding a 2-pole 480 Hz stage before the ADC in
`rcflex_semg_sample.m` would close this and is cheap; it is not required until real data
arrives, but the hardware requirement stands regardless.

`rcflex_semg_quality`'s `oob_nV_rtHz` column is the field diagnostic for this: on real data,
a value far above the amplifier's datasheet noise density means the anti-alias filter is too
shallow.

### 4.3 What is already source-agnostic

No change is needed for real data in: `rcflex_semg_preprocess.m`, `rcflex_semg_features.m`,
`rcflex_semg_quality.m`, or the feature half of `characterize_rcflex_semg.m`. They consume
raw electrode volts and nothing else. Only `rcflex_semg_source.m` needs a new case.

The parts of `characterize_rcflex_semg.m` that **do** need ground truth —
`env_intent_r`, `env_tremor_r`, `burst_tremor_r`, `lead_ms` — cannot run on an
unlabelled recording. §5 accounts for this.

---

## 5. What to test with a real sEMG dataset, before fusion

In order. Each stage gates the next; the point is to fail early and cheaply.

### 5.1 Acquisition health — run first, on every recording

`rcflex_semg_quality(raw, C)`. No ground truth needed. Must show: `clip_percent < 0.1`,
`headroom_dB` in 3–10, `band_frac > 0.5`, `mains_dB < −6`, no channel below 1 µV rms.

If this fails, nothing downstream means anything. This is the stage that catches a bad gain
setting, a lifted electrode or a wrong sample rate — all of which otherwise present as
"sEMG turned out to be uninformative".

### 5.2 Does the preprocessing chain survive real noise?

Our mains notch and 20 Hz highpass are tuned against *modelled* interference. Real
recordings have wandering baselines, ECG contamination on proximal sites, and mains
harmonics at 100/150 Hz that we do not notch at all. Confirm the envelope and burst features
remain bounded and non-saturating, and check whether 100 Hz pickup is material.

### 5.3 Tremor detectability — the first real go/no-go

On a recording containing tremor versus one without (rest or a non-tremor subject):
**burst-band SNR must exceed ~6 dB.** Synthetic gives 7.7–12.4 dB; anything near 0 dB on
real data means the burst feature does not survive contact with reality and the fusion plan
needs rethinking, not tuning.

### 5.4 Cross-talk between agonist/antagonist channels

Not testable synthetically — the model generates channels independently, so it assumes zero
cross-talk. Measure the correlation between the paired channels during isolated flexion and
isolated extension. High correlation collapses the reciprocal-activation assumption behind
the agonist/antagonist feature layout, and would argue for different electrode sites or a
different feature.

**This is the largest untested assumption in the current design.**

### 5.5 Does sEMG really lead the IMU?

The single most important measurement, because it is the only thing sEMG offers that a
better IMU estimator could not. Record sEMG **and** IMU simultaneously from the same arm and
cross-correlate the burst feature against measured tremor acceleration.

Synthetic gives +35 to +37.5 ms from a modelled 40 ms constant EMD. Real EMD varies with
contraction level, fatigue and muscle. **Required: a consistent positive lead > 10 ms.** If
the measured lead is near zero or unstable across trials, fusion should not proceed on the
phase-advantage argument.

Note this needs no intent/tremor labelling — only simultaneous capture — so it is runnable
on our own hardware as soon as it exists.

### 5.6 Independent frequency observable

Compare the burst feature's dominant frequency against the IMU-derived tremor frequency.
Synthetic recovers 7.81–8.20 Hz against a true 8 Hz. On real data, agreement within ~1 Hz
would establish sEMG as the independent ω observable that `CONTROL_IMPROVEMENT_PLAN.md` §1a
concluded was structurally necessary — which is the specific thing that would justify
reopening the EKF work.

### 5.7 Intent/tremor discrimination — needs labelled data

`env_intent_r` and `burst_tremor_r` require knowing which motion was voluntary. Obtain by
protocol rather than by post-hoc labelling: cued voluntary reaching, cued rest, and both
together, with trial boundaries logged. Without this, §5.3 and §5.5 are still valid and are
enough to decide whether to continue.

### 5.8 Only then

Revisit the fusion design, and only then the extended EKF. The staging in
`CONTROL_IMPROVEMENT_PLAN.md` §1a stands: characterise first, decide what to fuse second,
change the estimator last.

### Suggested public datasets for §5.1–5.3 before hardware exists

- **PhysioNet** — several EMG databases, including tremor-relevant recordings.
- **UCI Machine Learning Repository** — sEMG for Basic Hand Movements; forearm placement,
  though not tremor.
- **Ninapro** — large multi-channel forearm sEMG; voluntary movement only, no tremor, but
  excellent for §5.1, §5.2 and §5.4 (cross-talk).

None of these carry synchronised IMU tremor data with intent labels, which is why §5.5 and
§5.7 ultimately need our own recording.

---

## References

- De Luca, C. J. (1997). The use of surface electromyography in biomechanics.
  *Journal of Applied Biomechanics*, 13(2), 135–163.
- Hermens, H. J., Freriks, B., Disselhorst-Klug, C., & Rau, G. (2000). Development of
  recommendations for SEMG sensors and sensor placement procedures.
  *Journal of Electromyography and Kinesiology*, 10(5), 361–374. — SENIAM; §1.9 follows it.
- Merletti, R., & Parker, P. A. (Eds.) (2004). *Electromyography: Physiology, Engineering
  and Non-Invasive Applications*. IEEE Press/Wiley. — front-end design, half-cell offset,
  CMRR and electrode impedance mismatch.
- Cavanagh, P. R., & Komi, P. V. (1979). Electromechanical delay in human skeletal muscle
  under concentric and eccentric contractions. *European Journal of Applied Physiology*,
  42(3), 159–163. — the 30–50 ms EMD underpinning §5.5.
- Hogan, N., & Mann, R. W. (1980). Myoelectric signal processing: optimal estimation applied
  to electromyography. *IEEE Trans. Biomedical Engineering*, 27(7), 382–395.
- Winter, D. A. (2009). *Biomechanics and Motor Control of Human Movement* (4th ed.). Wiley.
  — forearm muscle anatomy for §1.9.
- Zhou, Y., Jenkins, M. E., Naish, M. D., & Trejos, A. L. (2018). Multi-sensor fusion for
  tremor estimation. — prior art on sEMG + inertial tremor estimation.
