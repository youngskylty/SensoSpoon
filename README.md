# SensoSpoon — MATLAB / Simulink Physical Simulation Platform

The current design uses a two-axis flexure gimbal, staggered voice-coil actuators, and a spherical front cover. The spoon bowl measures 62 × 40 × 12 mm, the curved neck is 31 mm long, and the overall length is 271 mm. Motion is calculated from voltage, current, actuator force, and mechanical dynamics. The closed loop includes dual-IMU attitude PID and independent current PI control.

This is an algorithm-development baseline. Tests demonstrate attenuation within the intended frequency band, but voluntary-motion tracking still has errors. Large disturbances and out-of-band inputs can cause saturation and amplify motion. Complete stabilization under arbitrary tremor has not been achieved. See the [closed-loop test results](output/results/CONTROL_RESULTS.md).

## Getting started

`START_PROJECT` is the only function you need for normal use. Each mode runs the right
underlying functions in the right order, writes its CSVs and figures, prints a progress
line per step, and finishes by listing exactly where its outputs went. A step that fails
stops the mode and says which step failed, so a half-finished run never leaves a stale CSV
looking like a result.

```matlab
START_PROJECT('setup')        % paths, output folders, Simulink cache. Nothing else
START_PROJECT('test')         % every unit and regression test
START_PROJECT('benchmark')    % the 20-case x 6-arm controlled sweep + summary + 3 figures
START_PROJECT('fusion')       % sEMG intent-gate calibration + the E2 fusion experiment
START_PROJECT('fastmove')     % fast-voluntary-movement detector comparison (E3)
START_PROJECT('jointtravel')  % authority-limiter joint-travel experiment
START_PROJECT('realdata')     % public-dataset (GRABMyo) sEMG validation
START_PROJECT('validation')   % test + jointtravel + fastmove + realdata
START_PROJECT('figures')      % regenerate every report figure from the saved results
START_PROJECT('all')          % the complete final analysis, in dependency order
```

Design demonstrations are unchanged: `'preview'`, `'simulate'`, `'control'`, `'compare'`,
`'soup'`, `'analysis'`.

Options apply to any mode:

- `START_PROJECT('all','DryRun',true)` lists the steps a mode would run and the files it
  would write, and executes nothing. Use it to check the wiring before committing two hours.
- `START_PROJECT('benchmark','Quick',true)` re-runs only the sEMG arm and merges it into the
  existing CSV.
- `'Force',true` rebuilds the geometry assets even if they are current.

### How to reproduce each result

| You want | Command | Runtime | It writes |
|---|---|---|---|
| **Baseline benchmark** (`simple_pid`, pre-EKF) | `START_PROJECT('benchmark')` | ~70 min | `results/control_benchmark.csv`, rows with `arm == "simple_pid"` |
| **Final benchmark** (all six arms, one controlled experiment) | `START_PROJECT('benchmark')` | ~70 min | `results/control_benchmark.csv`, `control_benchmark_summary.csv`, `CONTROL_RESULTS.md`, `figures/bench_*.png` |
| **sEMG fusion analysis** (E2) | `START_PROJECT('fusion')` | ~27 min | `results/semg_intent_calibration.csv`, `semg_intent_e2.csv`, `figures/semg_intent_separation.png`, `semg_gate_reach.png` |
| **Fast-movement experiment** (E3) | `START_PROJECT('fastmove')` | ~27 min | `results/fast_movement_thresholds.csv`, `fast_movement_separability.csv`, `fast_movement_detectors.csv`, `fast_movement_closed_loop.csv`, `figures/fast_movement_detectors.png` |
| **Joint-travel repair evidence** | `START_PROJECT('jointtravel')` | ~8 min | `results/joint_travel_authority.csv`, `figures/joint_travel_authority.png` |
| **Real-data validation** | `START_PROJECT('realdata')` | ~30 s | `results/semg_dataset_validation*.csv`, `semg_separation_real*.csv`, `figures/semg_separation_real.png` |
| **All final figures**, from results already on disk | `START_PROJECT('figures')` | ~1 min | every `figures/*.png` the report uses |
| **Everything, from a clean state** | `START_PROJECT('all')` | ~2 h | all of the above |

The baseline and the final benchmark come from the **same run** on purpose. Every arm is
scored on identical cases and identical sensor seeds (`9100+k`), so the only difference
between the blocks of rows is the arm definition — that is what makes the comparison a
controlled experiment rather than two runs put side by side.

Real-data validation needs `data/external/grabmyo/`. The converted fixture
`grabmyo_p1s1_fixture.mat` is committed, so `'realdata'` runs with no network access; the
raw WFDB records are re-fetchable with `data/external/grabmyo/fetch_grabmyo_subset.sh`.

The individual functions (`benchmark_rcflex_control`, `summarise_rcflex_benchmark`,
`plot_rcflex_benchmark`, `run_semg_intent_experiment`, `validate_rcflex_semg_dataset`, ...)
all remain callable directly for debugging. `START_PROJECT` orchestrates them; it does not
replace them.

### What has and has not been validated

Read [`docs/VALIDATION_STATUS.md`](docs/VALIDATION_STATUS.md) before quoting any number from
this repository. It separates algorithmic simulation evidence, public real-data evidence,
hardware validation (none) and remaining assumptions, and it is the document that says what
each result is allowed to claim.

Verified with MATLAB R2026a and Simulink. The current model does not require the legacy
Simscape Multibody model. Geometry is generated on demand; animation export requires a
working graphics environment.

**Optional — Claude Code / MATLAB MCP.** `.mcp.json.example` shows how to let Claude Code drive
MATLAB. Copy it to `.mcp.json` and edit the server path for your machine (the real `.mcp.json`
is git-ignored because it holds a local absolute path). It sets a one-hour `timeout` because a
full benchmark arm runs for ten to fifteen minutes.

## Project structure

```text
START_PROJECT.m       Main entry point; configures paths and cache locations
src/
  model/              Parameters, linkage, dynamics, mass properties, sensor truth
  simulink/           Model builders, runners, plant and sensor S-functions
  geometry/           Parametric meshes and mesh mass properties
  visualization/      Exterior, cutaway, dimensions, animations, response and report plots
  analysis/           Disturbance inputs, metrics, experiments, dataset importers
  control/            Dual-IMU estimation, EKF, sEMG intent gate, attitude/current loops
  fluid/              Reduced-order slosh, volume conservation, rim overflow
  project_path.m      Absolute paths independent of the working directory
models/               Open-loop Analytic.slx and closed-loop Control.slx models
tests/                Validation functions and run_validation
data/external/        Public sEMG datasets (GRABMyo, TremorDB): converted fixtures + fetch scripts
docs/                 Design, model interfaces, validation status, hardware sources
  sources/            Original manufacturer PDFs
EXPLANATION/          Code walkthrough (00_START_HERE.md first) and progress reports V2/V3
reports/              Submitted IEEE report, Section V draft, and group presentation
assets/geometry/      Generated STL files, assembly.mat, and parts manifest
output/
  figures/            Previews, response plots, and videos
  results/            Result CSVs, validation reports, CONTROL_RESULTS.md
  cache/              Regenerable Simulink cache (git-ignored)
```

Large regenerable files (per-case benchmark traces `control_case_*.mat`, raw dataset downloads,
Simulink cache) are not committed; `START_PROJECT('benchmark')` recreates them.

**New here?** Read [`EXPLANATION/00_START_HERE.md`](EXPLANATION/00_START_HERE.md), then run
`START_PROJECT('setup')` and `START_PROJECT('test')`.

## Algorithm development

Run `START_PROJECT('setup')` to configure paths, then call the underlying interfaces:

```matlab
H = rcflex_parameters();
t = (0:1/H.fs:2)';
base = zeros(numel(t),15);
volts = [ones(size(t)), zeros(size(t))];
S = run_rcflex_analytic('voltage',2,H,volts,base);
plot_rcflex_plant(S);
```

Start parameter changes in `src/model/rcflex_parameters.m`. Algorithms should command two voltages and use sensor outputs with noise and delay. Ideal states are for debugging and scoring only. The `analysis` results estimate demands under ideal conditions; they are not achieved tremor suppression.

- [Liquid model and limitations](docs/LIQUID_MODEL.md)
- [Closed-loop control, calibration, and test definitions](docs/CONTROL.md)
- [Current mechanical design](docs/DESIGN.md)
- [Model equations, ports, and limitations](docs/RCFLEX_MODEL_GUIDE.md)
- [Real components and assumptions](docs/HARDWARE_BOM.md)
- [Cleanup and validation records](docs/CLEANUP.md)
- [**What has and has not been validated**](docs/VALIDATION_STATUS.md)
- [Public sEMG datasets: what they can and cannot validate](docs/SEMG_PUBLIC_DATA_SURVEY.md)
- [The GRABMyo real-data test case](docs/SEMG_DATASET.md)
- [sEMG subsystem, implementation and intent fusion](docs/SEMG_SUBSYSTEM.md)

The model combines actual component specifications with custom structures and uncalibrated parameters. It is not a validated manufacturing design or a validated digital twin of a physical prototype.

## Project language

Use English for project documentation, code comments, diagnostic messages, model annotations, plot labels, and video annotations.
