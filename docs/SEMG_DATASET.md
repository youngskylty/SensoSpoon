# Real sEMG Dataset for the Six-Stage Validation Gate

Selection, provenance and honest scope of the real-data test case for
`validate_rcflex_semg_dataset.m`. See `SEMG_IMPLEMENTATION.md` §14 for the gate itself.

---

## 1. Selected dataset

**GrabMyo — Large-Sample and Multiday Surface Electromyogram Recordings of Hand Gestures,
v1.1.0**

| | |
|---|---|
| Source | PhysioNet — <https://physionet.org/content/grabmyo/1.1.0/> |
| DOI | <https://doi.org/10.13026/rzbf-ts96> |
| Licence | **CC BY 4.0** — research and educational use permitted, redistribution permitted with attribution |
| Authors | Pradhan, Jiang et al., Waterloo Engineering Bionics Lab, University of Waterloo |
| Full size | 9.4 GB |
| **Downloaded** | **~12 MB** (see §4) |

> **Licence note.** `readme.txt` names "Open Data Commons Attribution License v1.0", but the
> shipped `LICENSE.txt` and the PhysioNet record both state **CC BY 4.0**. The licence file is
> authoritative. Both are attribution-only licences permitting our use either way.

### Contents

- 43 healthy participants, 3 sessions on different days.
- **16 forearm** electrodes (F1–F16) as two rings of eight equally spaced around the
  circumference, 2 cm apart longitudinally, at 1/3 forearm length from the elbow crease.
- 12 wrist electrodes (W1–W12) — unused here.
- **2048 Hz**, EMG-USB2+ (OT Bioelettronica), WFDB format-16.
- 16 gestures + rest (gesture 17), 7 trials each, 5 s per trial.
- Gestures **11 = wrist flexion** and **12 = wrist extension** — the isolated
  agonist/antagonist contractions our cross-talk stage needs.

> **Upstream labelling is self-contradictory.** `GestureList.JPG` gives 11 = wrist flexion,
> 12 = wrist extension; `MotionSequence.txt` gives the reverse, and also swaps 13/14
> (pronation/supination). We follow the JPG. This is an upstream defect, not a conversion
> error, and it is **harmless for stage 3**, which reports \|r\| and is invariant under
> swapping agonist with antagonist. It does mean the flexor/extensor *identity* of the two
> selected sites is not established, so the channels are named by their empirical
> flexion/extension selectivity rather than by muscle.

---

## 2. Why this one, and why not the others

### The Oxford candidate was investigated first and rejected

"EMG and accelerometer recordings during closed-loop TMS for tremor in Essential Tremor and
Parkinson's Disease" corresponds to an **ongoing clinical trial**
([NCT07574164](https://clinicaltrials.gov/study/NCT07574164)), not a released dataset. The
closest published Oxford tremor dataset —
[*Tremor data measured from essential tremor patients subjected to phase-locked deep brain
stimulation*](https://ora.ox.ac.uk/objects/uuid:4b3c7cea-ad85-4d90-a9fc-8a874dc94093)
(CC BY-NC-SA) — was inspected and contains **accelerometer data only, no EMG at all**
(7 patients, dominant tremor axis plus stimulation triggers). It cannot support any sEMG
stage.

### No public dataset has forearm sEMG + synchronised motion + tremor

This was searched for specifically and appears to be a genuine gap. The nearest match — 5
patients (4 PD, 1 ET), wrist flexor/extensor, 2 kHz Delsys Trigno, used in
[PMC7248755](https://pmc.ncbi.nlm.nih.gov/articles/PMC7248755/) — is **private**, obtained by
those authors by permission with no repository, DOI or availability statement.

### So GRABMyo is chosen for what it can actually answer

It is the best available match on the criteria that do not require tremor: multi-channel
**forearm** sEMG, **flexor and extensor** sites, isolated flexion/extension **and rest**
conditions, usable labels, clean provenance, and an unambiguous permissive licence.

It is explicitly **not** a tremor dataset, and is not treated as one.

---

## 3. Which stages it supports

| # | Stage | Supported | Why |
|---|---|---|---|
| 1 | Acquisition health | ✅ **Yes** | Needs only the signal |
| 2 | Burst SNR > 6 dB | ❌ No | No tremor condition — nothing to put in the numerator |
| 3 | Raw cross-talk | ✅ **Yes — and this is the point** | Isolated wrist flexion and extension with 16 forearm sites, labelled `intent` |
| 4 | sEMG lead | ❌ No | No accelerometer or IMU |
| 5 | Frequency agreement | ❌ No | No mechanical reference |
| 6 | Intent/tremor discrimination | ❌ No | Labels are voluntary/rest only; no tremor class |

**Stage 3 is exactly the one that most needed real data.** It is the largest untested
assumption in the design: the synthetic source generates its four channels independently, so
it *assumes* zero cross-talk, while the whole agonist/antagonist feature layout rests on
reciprocal activation of closely packed forearm muscles
(`SEMG_IMPLEMENTATION.md` §12.2).

Stages 2, 4, 5 and 6 report **SKIPPED**, not PASS. They remain unanswered and still gate
fusion.

---

## 4. Exact subset downloaded

`data/external/grabmyo/`, reproducible via `fetch_grabmyo_subset.sh`:

| Item | Value |
|---|---|
| Participants | 1 and 2 |
| Session | 1 |
| Gestures | 11 (wrist flexion), 12 (wrist extension), 17 (rest) |
| Trials | 1–3 each |
| Records | 18 (`.hea` + `.dat`) |
| Size | ~12 MB of 9.4 GB |
| Documentation | `LICENSE.txt`, `readme.txt`, `subject-info.csv`, `Electrodelocation.pdf`, `GestureList.JPG`, `MotionSequence.txt` |

```
data/external/grabmyo/
  fetch_grabmyo_subset.sh        re-download script       [committed]
  LICENSE.txt, readme.txt, ...   upstream documentation   [committed]
  grabmyo_p1s1_fixture.mat       participant 1, converted [committed, 1.4 MB]
  raw/session1_participantN/     WFDB records             [NOT committed - .gitignore]
```

Raw WFDB records are not committed: they are freely re-fetchable from PhysioNet and would
add bulk for no benefit. The converted fixture is committed so the next session can run the
gate immediately with no network access. Redistribution of both is permitted under CC BY 4.0;
attribution is carried in this file and in the fixture's `notes` field.

---

## 5. Conversion

`src/analysis/import_grabmyo.m` maps GRABMyo onto `rcflex_semg_dataset_spec`.

```matlab
C = rcflex_control_parameters; C.semg_enabled = true;
C = rcflex_prepare_control(C);
S = import_grabmyo('data/external/grabmyo', 1, 1, C);
R = validate_rcflex_semg_dataset(S, C);
% or straight from the committed fixture:
R = validate_rcflex_semg_dataset('data/external/grabmyo/grabmyo_p1s1_fixture.mat', C);
```

Two conversion decisions matter, and both were found by inspection rather than assumed.

### 5.1 Monopolar → bipolar

GRABMyo records **monopolar** against a common reference; our dataset contract specifies
**bipolar differential** channels. This is not cosmetic — every monopolar channel carries the
same reference potential, so any two are strongly correlated *by construction*.

Measured directly: feeding the monopolar channels in unchanged reports cross-talk of
**|r| = 0.95–0.98** and stage 3 **FAILS**. That number is an artefact of the montage, not a
property of the muscles, and taking it at face value would have produced a completely wrong
conclusion about the design's central assumption.

The two rings are 2 cm apart longitudinally, so `F(k) − F(k+8)` is a bipolar pair at 20 mm
along the fibre direction — exactly the montage `SEMG_SUBSYSTEM.md` §1.9 specifies. With that
derivation, raw cross-talk falls to **0.20–0.46** depending on electrode separation (§6.2),
against 0.95–0.98 for the monopolar montage. The derivation is therefore essential and
clearly correct; it is not, on its own, sufficient to pass the gate.

### 5.2 Channel selection is empirical

The eight bipolar sites are circumferential positions, not named muscles, and GRABMyo
publishes no site-to-muscle table (`Electrodelocation.pdf` is a figure showing ring geometry
only). Rather than guess, the mapping is derived from the data: the site most *selectively*
active during wrist flexion becomes the agonist, the most extension-selective the antagonist,
and the next-best pair serves the second axis.

The second pair is a **substitute, not a match** — our pitch axis should be FCU/ECU, and this
is another flexor/extensor pair on the same rings.

---

## 6. Results on this dataset

Run with **unmodified thresholds**. Participants 1 and 2, session 1:

| Stage | P1 | P2 | Threshold | Synthetic reference |
|---|---|---|---|---|
| 1 Acquisition health | **PASS** | **PASS** | clip < 0.1 % | 0 % |
| 3 Raw cross-talk \|r\| | **0.460 FAIL** | **0.262 PASS** | < 0.30 *(provisional)* | 0.044 *(artefact)* |
| 3b `coactivation_corr` | 0.63 / 0.65 | 0.78 / 0.76 | *diagnostic* | 0.78–0.85 |
| 2, 4, 5, 6 | SKIPPED | SKIPPED | — | — |

Acquisition detail: no clipping, headroom 12.0–27.3 dB, in-band power fraction 0.95–0.99,
mains −19 to −23 dB, **16.6–130 µV rms**, crest factor **11.5–23.2**.

> **Superseded result.** An earlier revision of this document reported stage 3 as
> **0.15 / 0.08 PASS** against a 0.6 threshold and concluded that the reciprocal-activation
> assumption "survives contact with real data." **That conclusion is withdrawn.** It was
> produced by the old metric, which correlated *envelopes* over the *whole pooled record* —
> two independent errors. Envelope correlation measures shared neural drive rather than
> electrical coupling, and pooling flexion, extension and rest averages relationships of
> opposite sign so they cancel. On these same recordings the pooled figure moves between
> −0.58 and +0.92 depending only on which conditions are included.

### 6.1 What the corrected metric shows

Per-segment raw zero-lag \|r\| (full detail in `output/results/semg_stage3_segments_p*.csv`):

- **P1 fails**: worst 0.460, median 0.331 across 12 segment-axes.
- **P2 passes**: worst 0.262, median 0.129.

The split is not subject variability — it tracks **electrode separation**. P1's roll pair sits
on opposite aspects of the forearm (4 positions apart on the ring) and reads 0.06–0.46; its
pitch pair is only 2 positions apart and reads 0.33–0.46 consistently.

### 6.2 Cross-talk decays with electrode separation

Raw band-limited \|r\| across all 8 bipolar sites, contraction segments only:

| Circumferential separation | P1 | P2 |
|---|---|---|
| 1 position | 0.703 | 0.395 |
| 2 positions | 0.441 | 0.198 |
| 3 positions | 0.314 | 0.191 |
| 4 positions (opposite) | **0.275** | **0.201** |

Monotonic decay is the signature of **volume conduction** — genuine electrical cross-talk.
The ~0.20 plateau that does not decay further is the **residual common-mode** of a shared
reference montage, and is irreducible with this electrode arrangement. Those two numbers are
what the provisional 0.30 gate is derived from: it sits above the irreducible common-mode
floor, so it does not demand better than the instrumentation can deliver, and below
adjacent-site placement, so it fails sloppy siting.

### 6.3 The finding that matters

**Placement, not physiology, is the binding constraint.** Agonist/antagonist channels sited on
opposite aspects of the forearm achieve acceptable cross-talk; closely spaced ones do not, and
one of two participants fails outright on a substitute pair that was only 2 positions apart.
`SEMG_SUBSYSTEM.md` §1.9 already calls cross-talk "the main placement risk" — this quantifies
it and promotes it to a hard requirement with a verification step.

Separately, **stage 3b shows co-activation of 0.63–0.78** on these healthy voluntary
contractions. That is a physiological observation, not a defect, and it does not gate — but it
does mean the agonist/antagonist envelopes are *not* reciprocal here, which is worth carrying
into the feature design.

### 6.4 Caveats

Measured on **healthy voluntary contractions**, not tremor; tremor may drive antagonist pairs
more synchronously — in essential tremor the classic pattern is synchronous rather than
alternating. Amplitudes (16.6–130 µV rms) are well below the synthetic model's 700 µV because
these are moderate voluntary contractions rather than near-MVC, so the real front end will sit
lower in its range than `SEMG_SUBSYSTEM.md` §1.4 assumes. Crest factors of **11.5–23.2**
exceed the 7–10 assumed in `SEMG_IMPLEMENTATION.md` §5, so the gain must be set from a real
captured peak rather than an rms estimate. Two participants is a small sample, and the 0.30
threshold is provisional on exactly that basis.

---

## 7. Still missing before the gate can complete

Stages 2, 4, 5 and 6 need what no public dataset provides together:

- **A tremor condition** (stages 2, 6).
- **A synchronised mechanical reference** — accelerometer or IMU on the same clock
  (stages 4, 5).

Both come from the same place: **our own recording**, on the hardware in
`SEMG_SUBSYSTEM.md` §3, following the protocol in §6.3. Stage 4 in particular — does sEMG
actually lead the IMU? — needs no labelling at all, only simultaneous capture, so it is
runnable as soon as the front end exists.

Until then the honest position is: **acquisition is validated on real data; cross-talk is
measured on real data and fails for one of two participants**; and the tremor-specific claims
rest on the synthetic model alone.

A third item now joins the list, and it is cheap: **the stage 3 failure is a placement
problem, so the recording protocol must include a placement-verification step.** Site the
agonist/antagonist pairs on opposite aspects of the forearm and confirm raw zero-lag \|r\| <
0.30 on an isolated contraction *before* recording the tremor trials, rather than discovering
it afterwards.

---

## Citation

> Pradhan, A., He, J., & Jiang, N. (2022). Multi-day dataset of forearm and wrist
> electromyogram for hand gesture recognition and biometrics (version 1.1.0). *PhysioNet*.
> <https://doi.org/10.13026/rzbf-ts96>

> Goldberger, A., et al. (2000). PhysioBank, PhysioToolkit, and PhysioNet. *Circulation*,
> 101(23), e215–e220.
