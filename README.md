# SensoSpoon — MATLAB / Simulink Physical Simulation Platform

The current design uses a two-axis flexure gimbal, staggered voice-coil actuators, and a spherical front cover. The spoon bowl measures 62 × 40 × 12 mm, the curved neck is 31 mm long, and the overall length is 271 mm. Motion is calculated from voltage, current, actuator force, and mechanical dynamics. The closed loop includes dual-IMU attitude PID and independent current PI control.

This is an algorithm-development baseline. Tests demonstrate attenuation within the intended frequency band, but voluntary-motion tracking still has errors. Large disturbances and out-of-band inputs can cause saturation and amplify motion. Complete stabilization under arbitrary tremor has not been achieved. See the [closed-loop test results](output/results/CONTROL_RESULTS.md).

## Getting started

Open `START_PROJECT.m` in the project root and click Run to preview the model, or use the MATLAB Command Window:

```matlab
START_PROJECT                  % Exterior, internal layout, and spoon previews
START_PROJECT('test')           % Physics, electrical, sensor, geometry, and controller checks
S = START_PROJECT('simulate')   % Six-second voltage simulation, plots, and MP4 animation
S = START_PROJECT('control')    % Native closed-loop simulation, plots, and animation
START_PROJECT('soup')           % Recompute with 5 mL of liquid and generate a comparison
START_PROJECT('compare')        % Side-by-side animation from saved native results
START_PROJECT('benchmark')      % Independent closed-loop tests and hardware-utilization report
%   -> then: summarise_rcflex_benchmark(); plot_rcflex_benchmark('final')
%   Arms: simple_pid (baseline) | ekf_cv (IMU-only EKF) | ekf_cv_semg (sEMG+IMU).
%   ekf_cv_qfix is the IMU-only ablation of the sEMG mechanism; see
%   docs/SEMG_INTENT_FUSION.md.
START_PROJECT('analysis')       % Inverse-dynamics demands under ideal tracking and adverse parameters
START_PROJECT('all')            % Rebuild, validate, preview, simulate, and animate
```

Verified with MATLAB R2026a and Simulink. The current model does not require the legacy Simscape Multibody model. Geometry is generated on demand; animation export requires a working graphics environment.

This project includes a project-scoped `.mcp.json` so Claude Code can connect to MATLAB. It points at absolute, machine-specific paths for the MCP server binary and MATLAB root — if you clone this repo on another machine, edit those two paths in `.mcp.json` to match your local install before using Claude Code here.

## Project structure

```text
START_PROJECT.m       Main entry point; configures paths and cache locations
src/
  model/              Parameters, linkage, dynamics, mass properties, sensor truth
  simulink/           Model builders, runners, plant and sensor S-functions
  geometry/           Parametric meshes and mesh mass properties
  visualization/      Exterior, cutaway, dimensions, animations, response plots
  analysis/           Disturbance inputs, metrics, inverse-dynamics feasibility
  control/            Dual-IMU estimation, attitude/current loops, validation
  fluid/              Reduced-order slosh, volume conservation, rim overflow
  project_path.m      Absolute paths independent of the working directory
models/               Open-loop Analytic.slx and closed-loop Control.slx models
tests/               Validation functions and run_validation
docs/                Design, model interfaces, hardware sources, cleanup records
  sources/            Original manufacturer PDFs
assets/geometry/      Generated STL files, assembly.mat, and parts manifest
output/
  figures/            Previews, response plots, and videos
  results/            Simulation data, validation reports, analysis tables
  cache/              Regenerable Simulink cache
backup/               Pre-cleanup recovery archive; excluded from the MATLAB path
```

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

The model combines actual component specifications with custom structures and uncalibrated parameters. It is not a validated manufacturing design or a validated digital twin of a physical prototype.

## Project language

Use English for project documentation, code comments, diagnostic messages, model annotations, plot labels, and video annotations.
