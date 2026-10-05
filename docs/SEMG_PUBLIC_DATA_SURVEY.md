# Public sEMG Datasets: What They Can and Cannot Validate

A systematic survey of publicly available datasets against the six-stage real-sEMG
validation gate (`src/analysis/validate_rcflex_semg_dataset.m`, criteria in
`SEMG_IMPLEMENTATION.md` §14). It exists because physical human data collection was out of
scope for this project, so every real-data claim has to come from somebody else's recording,
and the reader is entitled to know exactly which recordings were considered and why the
unanswered gates are unanswered.

**The survey conclusion, stated first:** *no public dataset records forearm sEMG
simultaneously with a measured tremor.* That is not a gap in the searching; it is a gap in
the literature's data sharing, and it is what keeps gates 2, 5 and 6 open. Gate 4 has a
named candidate that was identified but not analysed, and that is a scope decision, not an
impossibility — it is written up as such in §4.

---

## 1. The six gates, and what each physically requires

| # | Gate | Requires |
|---|---|---|
| 1 | Acquisition health | sEMG only |
| 2 | Burst-band SNR > 6 dB | sEMG **during tremor** and during a tremor-free condition |
| 3 | Raw cross-talk \|r\| < 0.30 | sEMG, ≥2 sites, an isolated contraction |
| 4 | sEMG leads the motion | sEMG **+ synchronised mechanical reference** |
| 5 | Frequency agreement | sEMG + synchronised mechanical reference **during tremor** |
| 6 | Intent/tremor discrimination | sEMG with **both** a labelled voluntary class **and** a labelled tremor class |

Gates 2, 5 and 6 need a tremor condition. Gates 4 and 5 need synchronised motion. The
intersection — forearm sEMG, tremor, and a synchronised IMU — is what does not exist
publicly.

---

## 2. Datasets inspected

Every row was checked against its own documentation or record page, not against a secondary
citation. "Inspected" in the last column means the signals themselves were loaded and
measured, not merely that the metadata was read.

### 2.1 GRABMyo — PhysioNet, v1.1.0 — **USED**

| Field | Value |
|---|---|
| Signals | 16 forearm + 12 wrist **monopolar** sEMG; nothing else |
| Sampling rate | 2048 Hz, EMG-USB2+ (OT Bioelettronica), WFDB format-16 |
| Placement | Two rings of eight, 2 cm apart longitudinally, at 1/3 forearm length from the elbow crease |
| Subjects | 43 healthy, 3 sessions each |
| Tasks | 16 hand gestures + rest (gesture 17), 7 trials × 5 s |
| Tremor measured? | **No.** Healthy subjects, no tremor condition |
| Intentional movement labelled? | **Yes** — cued gesture per trial |
| IMU / kinematic truth? | **No** |
| Synchronisation | Single acquisition system, one clock; no second modality to synchronise |
| Licence | CC BY 4.0 (`LICENSE.txt` is authoritative over `readme.txt`) |
| Gates it can answer | **1, 3** directly; **partial 6** (voluntary vs *rest*, not vs tremor) |
| Status | **Inspected and analysed.** See `SEMG_DATASET.md` and §3 below |

### 2.2 NinaPro DB7 — **IDENTIFIED, NOT ANALYSED**

| Field | Value |
|---|---|
| Signals | 12-channel sEMG + 9-axis IMU (accelerometer, gyroscope, magnetometer) per sensor + Cyberglove |
| Sampling rate | sEMG 2 kHz (Delsys Trigno IM) |
| Placement | Eight sensors equally spaced around the forearm at the radio-humeral joint; two on flexor digitorum and extensor digitorum; two on biceps/triceps |
| Subjects | 20 intact + 2 trans-radial amputees |
| Tasks | 40 movements (finger, wrist, grasp, functional), 6 repetitions, 5 s movement / 3 s rest |
| Tremor measured? | **No** |
| Intentional movement labelled? | **Yes** — cued, with rest between every repetition |
| IMU / kinematic truth? | **Yes**, and this is the point: EMG and IMU come from the same Trigno sensors |
| Synchronisation | Distributed as pre-synchronised MATLAB variables from one acquisition system |
| Licence | NinaPro terms (attribution, no-derivatives on the data); registration required |
| Gates it can answer | **4 in a modified form** — sEMG lead over *voluntary movement onset*, i.e. the electromechanical delay `C.semg_emd_s`. Not gate 5, which needs tremor |
| Status | **Not analysed.** See §4 |

### 2.3 PhysioNet HD-FW KIN (hand kinematics + high-density sEMG) — **REJECTED**

| Field | Value |
|---|---|
| Signals | 448-channel HD sEMG (three 8×8 forearm arrays, four 5×13 wrist grids), finger joint angles, isometric finger forces |
| Sampling rate | kinematics 200 Hz (5DT Data Glove 14 Ultra), forces 1000 Hz |
| Placement | Forearm arrays at 1/3 forearm length; wrist grids around the ulna head |
| Subjects | 21 healthy (19 M, 2 F), 21–35 y |
| Tasks | 20 hand gestures, 9 finger-flexion conditions at two force levels |
| Tremor measured? | **No** |
| IMU / kinematic truth? | Finger joint angles only — **not wrist roll/pitch**, which is the axis this device controls |
| Licence | PhysioNet **Restricted** Health Data License 1.5.0; signed data use agreement required |
| Size | 113.9 GB |
| Rejected because | Restricted access, 114 GB, and its kinematic channel measures the wrong degrees of freedom. DB7 gives the same evidence for gate 4 at a fraction of the cost |

### 2.4 Tremor datasets **without** EMG — **REJECTED for every sEMG gate, but one is now used for a different purpose**

These were checked individually because they are the obvious place a tremor+EMG recording
would live. Every one of them measures tremor with an inertial sensor and records no muscle
signal at all, so none can serve any sEMG gate.

**A correction to how this section originally read.** Rejecting these datasets for the sEMG
gates was right, and it remains right. But the sEMG gates are not the only thing real data
can be used for, and by filing this group under "rejected" the survey originally implied
they were of no use to the project at all. That was too broad. A tremor recording with no
muscle signal cannot say anything about the sEMG stream — and it can still say something
about the *controller*, because until now every suppression figure the project published was
measured against a tremor the project generated itself. The PhysioNet row below is therefore
now **used**, not as sEMG evidence, but as the tremor *input* to the plant: see
`docs/VALIDATION_STATUS.md` and `src/analysis/run_real_tremor_experiment.m`. The distinction
matters and is worth stating plainly — that experiment is evidence about the control
algorithm meeting a real waveform, and it is **not** evidence about human physiology or
about the sEMG assumptions, which stand exactly where §3 leaves them.

| Dataset | Signals | Subjects | Tremor measured? | sEMG? |
|---|---|---|---|---|
| Oxford — phase-locked DBS in essential tremor (ORA, CC BY-NC-SA) | Accelerometer on the dominant tremor axis + stimulation triggers | 7 ET patients | Yes | **No** |
| Zenodo 7273759 — hand acceleration, PD and controls | Smartphone/IMU acceleration | 45 examined + 454 self-reported | Yes | **No** |
| Zenodo 10782573 — ALAMEDA PD tremor | 92 features from GENEActiv wrist accelerometry, MDS-UPDRS scored | PD cohort | Yes | **No** |
| Mendeley `fd6sdp6t7x` — activity-based acceleration for essential tremor | Pebble smartwatch accelerometry, Fahn-Tolosa-Marin scored | ET patients | Yes | **No** |
| PhysioNet `tremordb` — rest tremor velocity, index finger, PD with chronic DBS | Finger tremor velocity, laser transducer, 100 Hz, ~60 s per record | 16 PD | Yes | **No** — but **USED as the tremor input**, see above |
| IEEE DataPort — "Tremor dataset" | Triaxial accelerometer only; subscription required | — | Yes | **No** |

### 2.5 The Oxford TMS candidate — **NOT A DATASET**

"EMG and accelerometer recordings during closed-loop TMS for tremor in Essential Tremor and
Parkinson's Disease" is the data-collection arm of an **ongoing clinical trial**
([NCT07574164](https://clinicaltrials.gov/study/NCT07574164)), not a released dataset. On
paper it is the exact recording this project needs. It is not available.

### 2.6 The nearest published match is private

Five patients (4 PD, 1 ET), wrist flexor/extensor sEMG at 2 kHz on Delsys Trigno with
synchronised accelerometry, used in [PMC7248755](https://pmc.ncbi.nlm.nih.gov/articles/PMC7248755/).
Obtained by those authors by permission, with no repository, DOI or availability statement.
This is the recording that would close gates 2, 5 and 6 in one pass, and it cannot be
obtained.

### 2.7 Curated indexes checked

`BelleJohn/EMG-Dataset-Collection` and `openlists/ElectrophysiologyData` were both walked
for any entry flagging tremor, Parkinson's disease or involuntary movement. **Neither index
contains one.** Two of their entries pair forearm sEMG with synchronised kinematics
(NinaPro DB1, `dataset_v2_blocks`), and both are voluntary-movement-only, like DB7 but with
lower-rate or envelope-only EMG.

---

## 3. What the public data actually established

### 3.1 Directly validated on real data

- **Gate 1, acquisition health** — PASS on GRABMyo P1 and P2 with unmodified thresholds.
- **Gate 3, raw cross-talk** — measured, and it **FAILS for one of two participants**
  (0.460 vs a 0.30 provisional threshold). The finding is that *placement*, not physiology,
  is the binding constraint, and it promoted electrode siting to a hard requirement with a
  verification step. Full detail in `SEMG_DATASET.md` §6.
- **Instrumentation reality checks** — real amplitudes are 16.6–130 µV rms against the
  synthetic model's ~700 µV, and real crest factors are 11.5–23.2 against the 7–10 assumed.
  Both change hardware decisions and neither could have been learned from simulation.

### 3.2 Partially validated: the class separation

`src/analysis/validate_rcflex_semg_separation.m` runs the shipped gate front end over real
forearm sEMG and measures the separation between **deliberate voluntary contraction** and
**quiescent rest**, by the same worst-case geometric-mean rule the synthetic floor uses.

This is *not* the claim the fusion rests on. The fusion rests on separating deliberate
movement from **tremor**, and tremor is real muscle activity while rest is its absence. The
difference has a known sign:

> the measured voluntary-vs-rest ratio is an **upper bound** on the voluntary-vs-tremor
> ratio a real tremulous subject would give.

An upper bound is still a falsifiable test — had it come out below the ≥ 4 : 1 design
target, the design would have been refuted on real data, because tremor can only move the
number down. It came out above. That is *consistent with* the simulated 6.3 : 1; it does not
confirm it.

The same run measures something simulation cannot predict: the **real instrumentation floor**
of the drive metric — what a quiescent forearm reads through real electrodes, real motion
artefact and real mains. That floor is what decides whether the gate can ever be closed at
all, and it is now measured rather than assumed.

### 3.3 Still unsupported by any public data

| Gate | Why it is still open |
|---|---|
| 2 burst SNR | Needs sEMG during tremor. No public dataset has it |
| 4 sEMG lead | Needs synchronised motion. **DB7 could answer it and was not run** (§4) |
| 5 frequency agreement | Needs synchronised motion **during tremor**. Nothing public has both |
| 6 intent/tremor | Needs a labelled tremor class. Nothing public has one. §3.2 bounds it, does not measure it |

`C.semg_emd_s = 40 ms`, the electromechanical delay that is the entire reason sEMG is worth
adding, therefore still rests on the literature (Cavanagh & Komi 1979) rather than on a
measurement made in this project.

---

## 4. Why DB7 was not run, and what running it would buy

DB7 is the one public dataset that could convert an assumption into a measurement here. It
would answer a **modified gate 4**: not "does sEMG lead the tremor" but "does sEMG lead
*voluntary movement onset*, and by how much" — which is exactly the electromechanical delay
the design assumes at 40 ms.

It was not run for three reasons, all of which are scope rather than science:

1. It is behind a registration wall requiring an account and acceptance of terms, which is
   the dataset owner's decision to grant to a person, not something to be automated.
2. Its movements are finger and wrist gestures against no load, so the delay it measures is
   a delay for *that* task, and generalising it to a loaded wrist stabilisation task is
   itself an assumption.
3. Its licence is no-derivatives on the data, so a converted fixture could not be committed
   alongside the GRABMyo one, and the analysis would not be reproducible from this
   repository without the reader independently obtaining the data.

**It remains the recommended next real-data step** if physical recording stays out of reach,
and `src/analysis/import_grabmyo.m` is the template — the dataset contract in
`rcflex_semg_dataset_spec.m` already declares `imu_accel`/`imu_fs`, so a DB7 loader plugs
into the existing gate with no changes to the gate.

---

## 5. What would actually close the remaining gates

One recording, roughly twenty minutes, described in `SEMG_SUBSYSTEM.md` §6.3: forearm sEMG
and an IMU worn together, alternating rest, tremor-like shaking and deliberate reaching.
That single session would close gates 2, 4, 5 and 6 and produce the first real measurement
of the separation the whole fusion depends on. It is out of scope for this project, and it
is the single highest-value item on the list — the only one that changes what the result
*means* rather than how well it is characterised.

Gate 3's failure adds a cheap prerequisite to that session: site the agonist/antagonist
pairs on **opposite aspects** of the forearm and confirm raw zero-lag \|r\| < 0.30 on an
isolated contraction *before* recording the tremor trials.

---

## References

> Pradhan, A., He, J., & Jiang, N. (2022). Multi-day dataset of forearm and wrist
> electromyogram for hand gesture recognition and biometrics (version 1.1.0). *PhysioNet*.
> <https://doi.org/10.13026/rzbf-ts96>

> Krasoulis, A., Kyranou, I., Erden, M. S., Nazarpour, K., & Vijayakumar, S. (2017).
> Improved prosthetic hand control with concurrent use of myoelectric and inertial
> measurements. *Journal of NeuroEngineering and Rehabilitation*, 14(1), 71.
> (NinaPro DB7.) <https://doi.org/10.1186/s12984-017-0284-4>

> Atashzar, S. F., et al. (2020). Characterization of upper-limb pathological tremors:
> application to design of an augmented haptic rehabilitation system. *IEEE Journal of
> Selected Topics in Signal Processing*. <https://pmc.ncbi.nlm.nih.gov/articles/PMC7248755/>

> Cagnan, H., et al. Tremor data measured from essential tremor patients subjected to
> phase-locked deep brain stimulation. *Oxford University Research Archive*,
> uuid:4b3c7cea-ad85-4d90-a9fc-8a874dc94093.

> Cavanagh, P. R., & Komi, P. V. (1979). Electromechanical delay in human skeletal muscle
> under concentric and eccentric contractions. *European Journal of Applied Physiology*,
> 42(3), 159–163.

> Goldberger, A., et al. (2000). PhysioBank, PhysioToolkit, and PhysioNet. *Circulation*,
> 101(23), e215–e220.
