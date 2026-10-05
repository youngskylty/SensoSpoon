# Tests (`tests/`)

`tests/run_validation.m` is the top-level entry point: it runs eleven check functions in a fixed order, times each, and writes `output/results/validation_summary.csv` (check name, pass/fail, elapsed seconds). Each check is a plain MATLAB function that uses `assert(...)` internally — if any assertion fails, the function throws an error and the whole run stops (there's no soft "collect all failures" mechanism; it's a straightforward fail-fast script, not a formal MATLAB unit-test class hierarchy). None of these tests concern themselves with whether the *control algorithm* achieves good tremor suppression — that's evaluated separately by the control benchmark suite (`03_CONTROL_SYSTEM.md`). These tests validate that the underlying model, geometry, and control mechanisms are *implemented correctly* and *internally consistent*.

Order run by `run_validation.m`:
```
test_rcflex_mesh_properties → test_spoon_reference → test_rcflex_packaging →
test_rcflex_shell_clearance → test_rcflex_front_cap → test_rcflex_spherical_clearance →
test_rcflex_physics → test_rcflex_hardware → test_rcflex_translation →
test_rcflex_control → test_rcflex_liquid
```

## `test_rcflex_mesh_properties.m`
Checks two independent things: (1) `rcflex_mesh_mass_properties.m`'s exact tetrahedral-integration formula gives the textbook-correct centroid and inertia for a simple box of known dimensions offset from the origin (a box has a known closed-form inertia formula, so this is a ground-truth check on the general mesh-integration code). (2) Every single part in the generated assembly (`assembly.mat`) is a genuinely *closed* mesh — every edge appears in exactly two triangles (`accumarray(g,1)==2`). **What would make it fail**: a bug in the volume/inertia integration math, or any part in `generate_rcflex_assets.m` that produces a mesh with a hole, a gap, or a non-manifold edge (which would make the assumed-solid mass-property calculation meaningless).

## `test_spoon_reference.m`
Checks that the generated spoon-bowl mesh's actual bounding-box extent matches the nominal `H.bowl_mm` dimensions, that the analytic outline function (`rcflex_spoon_outline.m`) reproduces the specific reference widths (28 mm at the inferred front station, the neck-connection width at the rear), that the widest point of the outline actually occurs at the intended 37.2 mm station, and that the bowl mesh contains vertex pairs at the correct rim/floor/wall-thickness z-offsets. **What would make it fail**: any change to `rcflex_parameters.m`'s bowl dimensions or `rcflex_spoon_outline.m`'s shape function that isn't kept mutually consistent, or a change to `bowlmesh(...)` in `generate_rcflex_assets.m` that no longer respects the documented wall thicknesses.

## `test_rcflex_packaging.m`
For every part rigidly fixed to the handle that's an actuator, battery, or electronics component, checks that all its mesh vertices fall *inside* the handle's outer shell profile (using the same superellipse formula the shell itself is built from, offset inward by the wall thickness). **What would make it fail**: adding or repositioning any handle-mounted component so it pokes outside the outer shell surface — i.e., a packaging/interference check, not a physics check.

## `test_rcflex_shell_clearance.m`
The dynamic version of the packaging check: sweeps the gimbal through a grid of pitch/roll angles (±17°, beyond the ±15° nominal limit, as a margin check) and confirms that *moving* parts (the pitch carrier and spoon carrier, which are rigid but rotate) never intersect the fixed handle shell profile at any sampled angle. There's also an optional "guard" check that's currently inactive because the corresponding `H.guard_x_mm` field and `Gimbal_guard` part don't exist in the current design (see note in `05_GEOMETRY_AND_VISUALIZATION.md`) — the check just skips itself via `isfield(...)`. **What would make it fail**: a moving part's geometry (spoon carrier or pitch carrier) swept through its full angular range colliding with the stationary shell.

## `test_rcflex_front_cap.m`
Checks that at every sampled gimbal angle, both actuator pushrods actually pass cleanly through their designated clearance windows in the front end cap (with margin for the rod's own finite diameter, computed conservatively for the oblique angle the rod makes as it passes through), and that no part of the moving gimbal body crosses in front of the front cap plane. **What would make it fail**: rod geometry, window placement, or gimbal travel range being inconsistent enough that a rod would physically hit the front cap instead of passing through its window.

## `test_rcflex_spherical_clearance.m`
Sweeps the gimbal angle range and checks the moving spherical carrier's surface never pokes outside the fixed spherical bezel's inner radius, reporting the minimum sampled clearance gap. **What would make it fail**: the moving ball being too large, or the travel range large enough, for the ball to hit the bezel.

## `test_rcflex_physics.m`
The most thorough physics check, covering:
- **Rod closure**: for a 7×7 grid of pitch/roll angles spanning ±15°, the computed rod length from the linkage solver matches the fixed nominal rod length to essentially machine precision (`<1e-12` m) — i.e., the "rigid rod, fixed length" kinematic constraint is actually being satisfied by the closed-form solution, not just approximately.
- **Stroke feasibility & transmission conditioning**: at every sampled angle, the actuator stroke stays within the actuator's physical travel, and the Jacobian's condition number stays below 20 (a proxy for staying away from a kinematic singularity where a small joint motion would require an enormous or undefined actuator motion).
- **Jacobian accuracy**: the analytical Jacobian matches a finite-difference approximation to high precision, and separately matches the complex-step derivative used inside `rcflex_mass_bias.m` — cross-checking that the "clever" complex-step trick used in the dynamics code agrees with an ordinary finite difference.
- **Positive-definite mass matrix**: at every sampled angle, the mass matrix's eigenvalues are positive — a basic physical sanity requirement (a mass matrix that isn't positive-definite would imply negative effective inertia, which is unphysical).
- **Energy balance**: at one specific state, numerically differentiates total mechanical+electrical energy and confirms it equals electrical power in minus all the modeled dissipation (resistive, flexure damping, friction) — a strong, model-agnostic check that the equations of motion don't secretly create or destroy energy.
- **Force-constant knockout**: with the actuator force constant set to zero, current has no effect on acceleration; with it restored, the same current produces a large acceleration difference — confirming the current-to-force-to-motion chain is actually wired correctly (not accidentally bypassed or always zero).
- **Passive decay & solver convergence**: integrates the mechanism from a slightly-displaced start with zero voltage ("off" mode) using two different solver tolerance settings and checks the two integrations agree closely (numerical convergence) and that total mechanical+electrical energy strictly decreases over time (a passive, unforced system should lose energy to damping/friction, never gain it).

**What would make it fail**: essentially any error in the linkage kinematics, the Jacobian derivation, the mass-matrix assembly, the force/current coupling, or an energy leak/gain bug in the equations of motion — this is the test most likely to catch a subtle sign error or missing term in the core dynamics.

## `test_rcflex_hardware.m`
Component-level acceptance tests independent of any control algorithm:
- **Locked-rotor RL step response**: with the mechanism artificially prevented from moving (so there's no back-EMF complication), a voltage step across the coil should produce the textbook exponential RL-circuit current rise `(1-exp(-tR/L))/R`, matched to high precision.
- **Reversed polarity**: applying the negative of the same voltage produces the exact negative of the same current trace (symmetry check).
- **Current chopping (limiting)**: forcing current up against the current limit with a large applied voltage, the derivative of current should be forced to (near) zero rather than continuing to rise — confirming the current-limiting logic in `rcflex_physics.m` actually engages.
- **Bus voltage clamp**: applied voltage never exceeds the modeled available bus voltage.
- **Copper heating**: applying current produces positive temperature-rise rate.
- **Payload sensitivity**: comparing free dynamics (same voltage input) with a light vs. heavy payload shows a physically meaningful difference in the resulting motion — confirming the payload mass actually affects the simulated dynamics rather than being silently ignored.
- **Accelerometer gravity reading**: with the mechanism at rest and level, both simulated IMUs' accelerometer axes read exactly 1 g (9.80665 m/s²) as expected for a stationary sensor under gravity.

**What would make it fail**: an incorrect RL model, a broken current-limiting implementation, a payload mass that isn't actually wired into the dynamics, or a sign/units error in the sensor model's gravity term.

## `test_rcflex_translation.m`
Confirms that base (handle) *translation* — not just rotation — actually excites the mechanism's dynamics (via the moving-base acceleration terms in `rcflex_mass_bias.m`), that both IMUs' accelerometers correctly report an applied world-frame linear acceleration added on top of gravity, and — using `rcflex_world_metrics.m` — that a scenario with the gimbal exactly canceling rotation still shows the bowl's *position* moving substantially (`position_rms_mm > 0.1`) even though *attitude* RMS is essentially zero (`<1e-5` degrees). **What would make it fail**: a bug that makes translation invisible to the dynamics or sensor model, or (more subtly) a change that collapses the important attitude-vs-position distinction the whole project relies on.

## `test_rcflex_control.m`
Checks the control-law implementation directly (not through Simulink):
- **Current tracking, reversal, and saturation recovery**: running the inner current-loop function against a hand-scripted reference-current profile (step to +0.7 A, reversal to −0.7 A, a deliberate 3 A over-limit request, then zero) and confirming the resulting simulated current tracks each phase correctly and voltage never exceeds the bus limit.
- **Intent-filter stability**: every second-order section's poles lie strictly inside the unit circle (a standard discrete-filter stability requirement — poles outside the unit circle mean the filter's output would grow unboundedly), and the cascade's overall DC gain is exactly 1 (so a constant/slow intentional hand tilt passes through unattenuated, as intended).
- **Controller determinism**: calling the attitude-control step function twice from the identical starting state and identical sensor input gives bit-identical results — confirming there's no hidden randomness or uninitialized-state dependence inside the deterministic control law itself (randomness only lives in the separate sensor-noise generation, which is seeded).
- **Current-limit respect**: the commanded current never exceeds 98% of the hardware current limit.
- **Correct feedback sign**: artificially tilting the estimated spoon orientation by a small positive pitch and confirming the resulting torque command changes in the direction that would *reduce* that pitch error (not amplify it) — this is the single most important sign-convention sanity check for a feedback controller; get this backwards and the "controller" would actively destabilize the system instead of stabilizing it.

**What would make it fail**: a sign error anywhere in the feedback path (which would flip stabilizing feedback into positive/destabilizing feedback), a broken anti-windup implementation, an unstable intent filter design, or any accidental source of nondeterminism in the control law.

## `test_rcflex_liquid.m`
Checks the fluid model's core invariants: volume reconstruction (the height field integrates back to exactly the requested volume), that perfectly still water in a level bowl produces zero surface tilt and zero spillage, that identical input motion produces bit-identical fluid results on both the "locked" and "controlled" simulation paths (when both are given the same q=0 test fixture), that a deliberately tilted, near-full static fixture *does* spill a meaningful amount (a positive-control check — confirming the model isn't silently reporting zero spill even when it obviously should spill), that remaining liquid volume never increases over time, that the accumulated mass-balance error stays under `1e-6` mL, and that the sum of individually-tracked spill "droplet" volumes matches the total computed outflow to high precision. **What would make it fail**: any leak in the volume-conservation logic, an asymmetric bug that makes "locked" and "controlled" fluid results differ even under identical forcing, or an inconsistency between the aggregate spill total and the per-droplet visualization bookkeeping.

## Two additional standalone scripts (not run by `run_validation.m`)

- **`verify_rcflex_control_numerics.m`** — cross-checks the *fast* multirate control integrator (used for benchmarking/tuning) against the *native* Simulink continuous closed-loop reference, at three different mechanical integration step sizes, reporting how much the two methods disagree on joint-angle RMS and suppression percentage. This is a numerical-method validation, not a physics or control-law correctness test — it answers "is the fast approximation trustworthy enough to use for the larger benchmark sweep?" It asserts the joint-angle RMS difference stays under 0.02° and writes `control_numerical_verification.csv`/`.mat`, which `write_rcflex_control_report.m` later reads to report this cross-check in `CONTROL_RESULTS.md`.
- **`test_rcflex_control.m` and the fluid tests are included in `run_validation.m`**, but there's no dedicated automated test for the geometry-mesh visual outputs or the analysis-envelope scripts — those are validated only by their own internal `assert`s when run directly (e.g. `evaluate_rcflex_envelope.m` doesn't get invoked by `run_validation.m` at all; it's a standalone analysis script, not a pass/fail test).

## Recorded results

The last full run recorded in `output/results/validation_summary.csv` shows all eleven checks passing, with `test_rcflex_liquid` (≈2.93 s) and `test_rcflex_physics` (≈2.31 s) the slowest, and the packaging/reference checks the fastest (well under 0.1 s each). `output/results/analytic_validation.txt` records the specific numeric tolerances achieved by `test_rcflex_physics` at that run (e.g. rod closure error ~7e-18 m, energy balance error ~3.7e-12 W, maximum transmission condition number ~16.8, maximum direct-rod side load ~1.95 N).
