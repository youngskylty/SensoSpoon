# Liquid Slosh and Spillage: Preliminary Analytical Model

## Viewing and running

- `output/figures/soup_5ml_comparison.mp4`: 5 mL, 8 Hz, 1° amplitude per axis, with the original 5 g mechanical payload.
- `output/figures/soup_8ml_comparison.mp4`: near-full 8 mL under the same handle input, recomputed in native Simulink with an 8 g mechanical payload.
- The left side is an uncompensated baseline with the gimbal locked to the handle; the right side uses solved closed-loop motion.
- Both sides use identical liquid, initial fill, and model parameters. Neither side is forced to spill or remain spill-free.

```matlab
START_PROJECT('soup')           % Recompute the 5 mL case and export a comparison video
START_PROJECT('setup')
F = run_rcflex_soup_demo(8);     % Recompute the 8 mL case
```

Liquid simulations use separate result files and do not overwrite `control_native.mat`. The original 5 g control results and reports are preserved.

## Initial results

In the two-second test, neither side of the 5 mL case loses liquid over the rim. For 8 mL, the uncompensated side loses approximately 0.0016 mL (1.6 microlitres), while no loss is computed on the compensated side. This is a trace spill, not substantial spillage.

Halving the spatial grid and integration step gives approximately 0.00145 mL uncompensated loss and zero compensated loss; see `output/results/liquid_refinement.csv`. Near the overflow threshold, small parameter changes can substantially affect the result. These decimal values should not be interpreted as physical measurement precision.

## Model components

This is a **one-way coupled, two-mode free-surface model**, not full CFD. Electrical/mechanical closed-loop motion is solved first, then used to drive the liquid model. The mechanical plant includes the initial liquid mass as payload, but is not recomputed in response to instantaneous liquid distribution, fluid reaction forces, or mass lost through spillage.

- The inner surface uses the same analytical profile as the current spoon-mesh generator, giving approximately 9.1 mL capacity.
- Clear soup is modeled at 1000 kg/m³ density. The assumed damping ratio of 0.12 requires experimental calibration.
- Effective gravity in spoon coordinates includes actual translation, angular acceleration, and centripetal acceleration, and drives two free-surface slopes.
- Initial wet lengths and mean depth estimate modal frequency through `omega² = g k tanh(k h)`, where `k = pi / wet_length`. Equivalent rectangular lengths approximate the irregular bowl; the fluid modes have not been calibrated.
- Each slope follows a damped second-order equation, including the relative-rotation inertia term `[alpha_y; -alpha_x]`. The free surface is a dynamically tilted plane; higher-order surface waves are not fully modeled.
- At each time step, a volume constraint determines surface height by integrating positive liquid depth above the actual bowl floor to match remaining volume.
- When the surface exceeds the rim, a simplified weir law estimates perimeter-integrated outflow: `dQ = (2/3) Cd sqrt(2g) head^(3/2) dl`. The coefficient `Cd = 0.6` is an uncalibrated assumption.
- Visualization droplets are generated only after outflow is computed. They inherit outlet velocity and follow gravitational trajectories. They illustrate discharged volume rather than validated droplet sizes or breakup distributions.

Fluid volume is computed on a 1 mm grid with a 0.25 ms integration step. The animation uses a 0.25 mm surface grid for visual refinement, retaining the computed surface height and slopes without altering physical results. Amber color and highlights only represent the appearance of clear soup.

## Validation and limitations

`test_rcflex_liquid` checks volume reconstruction, no spontaneous slosh or loss in stationary water, identical liquid results under identical motion, natural overflow from a tilted full fixture, and conservation between accumulated droplet volume and liquid loss.

The model excludes surface tension, contact angle, wetting films, splash breakup, higher-order waves, and two-way fluid–structure coupling. It raises an error if effective normal gravity falls below 0.5 m/s² or the surface slope exceeds the reduced-model range. It therefore supports slosh visualization and preliminary spill-risk screening, but cannot establish that a physical spoon will never spill.

The modal modeling approach draws on [NASA: Equivalent mechanical models of liquid slosh and small-motion limitations](https://ntrs.nasa.gov/api/citations/20050243594/downloads/20050243594.pdf). This supports reduced-order modeling as a method; it does not validate this project's particular modes, damping, or discharge coefficients.

## Stronger-tremor comparison

Run `START_PROJECT('setup'); run_rcflex_strong_soup` for the separate four-second, 4 Hz, 5°-per-axis, initially 8 mL case. It exports `output/figures/soup_strong_tremor_comparison.mp4`. The input retains the existing startup ramp. The video first plays in real time, then at quarter speed without amplitude magnification.

Before selection, fixed controller and fluid parameters were screened at 3°, 4°, and 5° per axis using the fast integrator. Results are in `output/results/strong_soup_screen.csv`. The final video uses native Simulink results. The script checks current, stroke, joint angle, copper loss, and shaft side load, then repeats the liquid calculation with halved grid spacing and time step. Separate files preserve the earlier 1° videos.

Substantial liquid loss can increase modal errors because modal dimensions remain based on the initial wetted region. Grid refinement tests numerical sensitivity, not the validity of this physical approximation. Quantitative spill volumes still require fluid experiments or higher-fidelity simulation.

Native results: uncompensated loss is 2.728 mL (approximately 34.1%), with no computed loss on the compensated side. Halving grid spacing and time step gives 2.682 mL and 0 mL respectively, approximately 1.7% difference in uncompensated loss. Attitude RMS decreases from 4.999° to 0.371° (92.58% reduction). Spoon-position RMS remains 4.06 mm, so no overflow must not be described as no vibration.

Peak current is 0.865 A, stroke is 1.697 mm, and joint angle is 5.554°. The higher of the two coils' mean copper losses over the steady scoring window is 1.522 W; maximum computed shaft side load is 0.948 N. Passing these checks does not establish long-term thermal stability or physical reliability.
