# RC-Flex physical plant — algorithm-independent

This version replaces prescribed spoon motion with an 8-state continuous
electromechanical plant integrated by Simulink `ode15s`. MATLAB Level-2
S-functions expose the equations and sensor model. No estimator, stabilisation
controller or desired-angle trajectory drives the plant.

## Run

```matlab
START_PROJECT('all')
S=run_rcflex_analytic('voltage',6);
animate_rcflex_analytic(S);
plot_rcflex_plant(S);
```

`SensoSpoon_Analytic.slx` contains independent `Voltage_A_B` and `Base_motion`
sources, the physical plant, raw-state logging, and sampled sensor logging.
The default test applies zero, common negative/positive and differential
negative/positive voltage, followed by zero. This deliberately demonstrates
the unregulated plant. Movement/oscillation/end-stop contact are expected.
`off` means zero applied voltage in a shorted/braking bridge state, NOT an
open-circuit motor and NOT a locked spoon. Open-circuit switching is not modelled.

Example custom experiment:

```matlab
H=rcflex_parameters(); t=(0:1/H.fs:2)';
V=[ones(size(t)) zeros(size(t))]; % 1 V on A, 0 V on B
base=zeros(numel(t),15);
S=run_rcflex_analytic('voltage',2,H,V,base);
```

The 15 base columns are `[roll,pitch,roll_rate,pitch_rate,roll_acc,pitch_acc,
x,y,z,vx,vy,vz,ax,ay,az]`. Rotation uses rad, rad/s, rad/s^2; world translation
uses m, m/s, m/s^2. The run wrapper zero-pads legacy six-column inputs.
Supply mutually consistent trajectories; the base is
an external hand disturbance. A future algorithm replaces only the voltage
source and reads `Sensor_readings`; raw exact states are for debugging.

## Equations and coupling

States: `q_pitch,q_roll,qd_pitch,qd_roll,i_A,i_B,temperatureRise_A,B`.

* Coil: `L di/dt = V - (Rcoil(T)+Rdriver)i - K(s) sdot`.
* Force: `F=K(s)i`; using the same SI constant for force and back EMF preserves energy.
* Linkage: one spherical-ended direct rod per actuator with fixed length.
  The explicit assembly branch gives `s(q)` from rod closure.
  `J=ds/dq`, `sdot=J qdot`, `Q=J' F`. No bellcrank is present.
* Dynamics: `M(q) qdd = Q - bias(q,qd,base) - spring - damping - friction - stops`.
* `bias` is assembled from rigid-body Newton-Euler equations, including gravity,
  moving-base angular acceleration, centripetal terms and gyroscopic terms.
* Both 11 g motor movers contribute mass, gravity and inertia; the rear extension adds mass.
* Catalogue nominal flexure torque and compliant joint/slider end stops are solved,
  not implemented by clipping the position or velocity after integration.
* Temperature rise is driven by copper loss and a lumped thermal network.
  Thermal resistance/capacitance are assumptions; reported temperatures are not
  established manufacturer predictions.

The constant-length constraints are ideal rigid reductions of the proposed
linkage, not independently prescribed plunger trajectories. Animation uses the
same geometry and solved states, including current, moving coils
and both rod orientations. C-Flex sleeves follow the two bodies; internal leaf
elasticity is represented by catalogue torsional springs, not deformable FEA.

## Current mechanical scope

See `DESIGN.md` for current dimensions and `HARDWARE_BOM.md` for sourced
parts and assumptions. Two constrained flexure axes support the custom
spherical carrier. The centre is an equivalent local centre, not a validated
remote centre. Moving coils, both rods and the rear shaft extension contribute
mass/inertia. Custom moving bodies use closed-mesh volume moments.
Purchased internal mass placement remains approximate. Geometry is
parametric, not imported manufacturer STEP.

## Sensors

Two BMI270 devices are represented at the handle PCB and a small moving PCB.
`Sensor_readings` is sampled at 800 Hz, then has a one-sample output delay,
a 100 Hz single-pole filter approximation, quantization, range saturation,
repeatable noise and fixed test-unit bias. These are configurable operating
assumptions, not the exact internal Bosch filter transfer function.

Columns: handle gyro xyz (rad/s), handle specific acceleration xyz (m/s^2),
spoon gyro xyz, spoon specific acceleration xyz, currents A/B (A).
Accelerometers include gravity and rotational acceleration. They do not directly
report tilt. Current ADC is a 12-bit approximation; driver gain/offset errors
and PWM ripple need later identification. MCU firmware timing is not emulated.

## Output / validation

`S.Y`: 1:2 angles, 3:4 rates, 5:6 currents, 7:8 temperature rise,
9:10 reserved, 11:12 averaged bridge voltage, 13:14 electromagnetic force,
15:16 stroke, 17:18 reserved zeros (unused), 19:20 acceleration, 21:22 copper loss,
23 kinetic energy, 24 gravity + flexure potential, 25:26 slider speed.

`test_rcflex_physics` checks rod closure over the +/-15-degree joint square,
transmission Jacobian against finite differences, positive mass matrices,
electromechanical energy balance, force-constant knockout, passive decay and
solver-tolerance convergence. `test_rcflex_hardware` adds electrical step,
reversed-polarity, bus/current limit and payload-response checks.
Passing these checks verifies the equations/implementation, not a real prototype.

Remaining uncertainties are explicit: bearing friction, flexure end constraints
and stiffness/coupling, motor force-vs-stroke curve digitization, thermal network,
battery resistance/discharge curve, custom masses/inertias, clearances and
magnetic/structural parasitics. Do not optimize an algorithm against one exact
nominal parameter set and call it optimal for hardware. No algorithm is adapted
or optimized in this delivery.
