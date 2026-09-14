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
| 3 | Paired cross-talk | ✅ **Yes — and this is the point** | Isolated wrist flexion and extension with 16 forearm sites |
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
derivation, cross-talk is **0.02–0.15**.

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
| 3 Paired cross-talk \|r\| | **0.15 PASS** | **0.08 PASS** | < 0.6 | 0.19 *(artefact)* |
| 2, 4, 5, 6 | SKIPPED | SKIPPED | — | — |

Acquisition detail: no clipping, headroom 12–27 dB, in-band power fraction 0.95–0.98, mains
−19 to −21 dB, 82–143 µV rms.

**The finding that matters:** with a correct bipolar montage, real forearm sEMG shows
agonist/antagonist cross-talk of 0.02–0.15 — comfortably below the 0.6 threshold and in the
same range as the synthetic model's 0.19. The reciprocal-activation assumption behind the
feature layout **survives contact with real data**. That was the largest open risk in the
design.

Two caveats on that result. It is measured on **healthy voluntary contractions**, not tremor,
and tremor may drive antagonist pairs more synchronously — in essential tremor the classic
pattern is synchronous rather than alternating. And amplitudes here (82–143 µV rms) are well
below the synthetic model's 700 µV, because these are moderate voluntary contractions rather
than near-MVC; the real front end will therefore sit lower in its range than
`SEMG_SUBSYSTEM.md` §1.4 assumes, which argues for the upper end of the 150–200 gain range.

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

Until then the honest position is: acquisition and cross-talk are validated on real data;
the tremor-specific claims rest on the synthetic model alone.

---

## Citation

> Pradhan, A., He, J., & Jiang, N. (2022). Multi-day dataset of forearm and wrist
> electromyogram for hand gesture recognition and biometrics (version 1.1.0). *PhysioNet*.
> <https://doi.org/10.13026/rzbf-ts96>

> Goldberger, A., et al. (2000). PhysioBank, PhysioToolkit, and PhysioNet. *Circulation*,
> 101(23), e215–e220.
