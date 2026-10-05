# Physics and the Core Model (`src/model/`)

This is the heart of the simulation: a physical model of the flexure gimbal, its two voice-coil actuators, and the rigid bodies that move. Everything else (Simulink wiring, control, fluid, visualization) is built on top of the functions in this folder.

## Files

- `rcflex_parameters.m` — all the numeric constants describing the current hardware design.
- `rcflex_linkage.m` — the geometric relationship between joint angles and actuator rod positions (kinematics).
- `rcflex_body_properties.m` — computes mass, center of mass, and inertia tensors of the moving parts from the 3D geometry.
- `rcflex_mass_bias.m` — assembles the rigid-body dynamics (the mass matrix and "everything else" bias term) for the whole mechanism, including moving-base effects.
- `rcflex_physics.m` — the actual state-derivative function: given the current state and inputs, returns the rates of change (this is what gets integrated over time) plus a big vector of diagnostic outputs.
- `rcflex_sensor_truth.m` — computes the exact (noise-free) values that the two IMUs would sense, given the true motion.

## The physical system being modeled

### The gimbal and flexures

Picture the spoon bowl mounted on a small carrier that can tilt in two independent directions:
- **Pitch** — nodding the bowl up/down.
- **Roll** — tilting the bowl side to side.

These two rotations are supported by flexure pivots — thin, precisely-shaped metal elements that flex elastically like a leaf spring, instead of using ball bearings with sliding/rolling friction. The docs mention real catalog parts: two **C-Flex B-10** pivots for pitch and one **C-Flex D-10** pivot for roll (see `docs/HARDWARE_BOM.md`). Because they're just elastic springs, an un-actuated flexure joint has a spring-like restoring torque pulling it back toward zero, plus some damping — you'll see this directly in the equations of motion.

The design uses a two-stage arrangement: a "pitch carrier" body pivots relative to the handle about the pitch axis, and a "spoon carrier" (holding the actual bowl) pivots relative to the pitch carrier about the roll axis. So pitch and roll are captured as a sequential 2-joint chain, not a true ball joint — this is why the code always talks about `q = [pitch; roll]`.

### The actuators (VCAs)

Two **Moticont GVCM-016-019-01M voice-coil actuators** (linear motors, like a loudspeaker's voice coil — current through a coil in a magnetic field produces force along a single axis) are mounted in the handle, "staggered" (offset from each other, one in front of the other) per `H.vca_centers_mm`. Each actuator has a rigid pushrod connecting it to the moving gimbal assembly. Because the actuators are linear (they push/pull in a straight line) but the gimbal rotates, there's a kinematic relationship — as the gimbal tilts, each pushrod's actuator-side end must slide along the actuator's axis by some amount `s`. That's exactly what `rcflex_linkage.m` computes.

### Coordinate frames and states

The full state vector integrated by the physics is 8-dimensional:

```
x = [q_pitch; q_roll; qd_pitch; qd_roll; i_A; i_B; tempRise_A; tempRise_B]
```

- `q_pitch, q_roll` — the two gimbal joint angles (radians), i.e. how far the spoon carrier has rotated relative to the handle.
- `qd_pitch, qd_roll` — their rates (angular velocities).
- `i_A, i_B` — the electrical current in each of the two VCA coils.
- `tempRise_A, tempRise_B` — how much each coil's temperature has risen above ambient, from resistive (copper) heating.

The **input** vector `u` to `rcflex_physics` is:

```
u = [hand(2); hand_velocity(2); hand_acceleration(2); voltage(2); world_position/velocity/accel(9)]
```

`hand` here means the handle's own roll/pitch orientation (the disturbance — the "tremor" being applied to the whole handle from outside), separate from `q` (the gimbal's angle relative to the handle). So there are really two rotations stacked: the handle's absolute orientation (driven externally, representing the shaking hand) and the gimbal's angle relative to the handle (controlled by the actuators). The last 9 input columns are the handle's translation in the world (position, velocity, acceleration) — used for scenarios where the whole hand also moves through space, not just rotates.

### Equations of motion, plain-language walkthrough (`rcflex_physics.m`)

At each instant, given the state `x` and input `u`:

1. **Kinematics**: `rcflex_linkage(q,H)` computes the actuator rod geometry for the current joint angles — where each rod attaches to the gimbal (`L.attach`), where its slider sits along the actuator axis (`L.s`), and the Jacobian `L.J` (the matrix relating small changes in joint angle to small changes in actuator-rod position — this is the standard "how much does the actuator move per unit of joint rotation" sensitivity used to convert between joint-space and actuator-space quantities).
2. **Rigid-body dynamics**: `rcflex_mass_bias.m` computes the 2x2 mass matrix `M` (how joint accelerations relate to net torque, accounting for all the moving mass and inertia) and a `bias` vector (everything that isn't due to `M * qdd` — gravity, centripetal and gyroscopic terms, and the effect of the handle itself accelerating, since the whole mechanism is riding on a moving/shaking base). This uses classic rigid-body Newton-Euler mechanics: each moving body's velocity and angular velocity are expressed via its own Jacobian (computed with a clever trick — a tiny imaginary-number perturbation, see below), then contributions are summed.
3. **Flexure spring and damping**: `spring = k .* q .* (1 + cubic_ratio*(q/limit)^2)` — currently the cubic term is disabled (`H.flexure_cubic_ratio = 0`), so this is just linear-spring torque `k*q`. `damping = flexure_damping * qd` opposes velocity.
4. **Compliant end stops**: if `|q|` exceeds `H.physical_travel_deg` (15°), a stiff spring+damper (`stop_stiffness_Nm_rad`, `stop_damping_Nms_rad`) kicks in to represent the mechanical hard stop — modeled as a soft penalty rather than a hard clip, so the simulation stays smooth (differentiable) even near the limit.
5. **Actuator force-vs-stroke curve**: `curve = 1 - 0.27*(s/stroke)^2`, a quadratic de-rating of force constant near the ends of stroke — described in the docs as "an approximate digitization of the motor drawing," i.e. an estimate, not a measured curve.
6. **Electrical circuit**: each coil is a simple RL circuit with back-EMF: `L di/dt = V - (R(T) + R_driver)*i - K*sdot`, where `R(T)` increases with temperature (`coil_alpha_K`), and `K*sdot` is the back-EMF voltage proportional to the actuator's linear speed `sdot = J*qd`.
7. **Battery sag and driver behavior**: available bus voltage drops with total current draw (`bus = bus_voltage - battery_R*sum(|i|)`); if bus voltage falls below the driver's minimum, output is forced to zero; and there's a current-chopping approximation (limiting how much voltage can be applied once current reaches `H.current_limit_A`, mimicking how a real motor driver would behave, rather than modeling PWM switching in detail).
8. **Friction**: `friction = guide_friction*tanh(sd/0.001) + guide_viscous*sd` — a smoothed Coulomb-friction model (the `tanh` avoids a hard discontinuous sign function, which is easier for the ODE solver) plus viscous damping.
9. **Net torque and acceleration**: force from each coil (`F = K*i`) is mapped to joint torque via the same Jacobian transpose (`Q = J' * F`, a completely standard trick for converting actuator-space forces into joint-space generalized torques). Then `acc = M \ (Q - bias - spring - damping - stop)` solves for the resulting joint acceleration.
10. **Thermal model**: `dT/dt = (R*i^2 - T/thermalResistance) / thermalCapacity` — a simple single-node RC thermal model (heating from resistive/copper loss, cooling proportional to the temperature rise, standard lumped thermal-capacitance approximation).

The function returns both `dx` (the 8 derivatives, for the ODE integrator) and a fixed 26-column `Y` output containing states plus lots of diagnostics (currents, voltage, force, stroke, acceleration, copper loss, kinetic/potential energy, slider speed) — this fixed layout is documented at the bottom of `rcflex_physics.m` and repeated in `docs/RCFLEX_MODEL_GUIDE.md`.

### The imaginary-number derivative trick

You'll notice `rcflex_mass_bias.m` computing `[~,~,jv,jw]=kin(z+1i*1e-20*zd,...)` and then `Jvd=imag(jv)/1e-20`. This is the **complex-step differentiation** trick: evaluating a real-valued function at a tiny imaginary offset and dividing the imaginary part of the result by that offset gives an extremely accurate numerical derivative (far more accurate than a normal finite-difference, since there's no subtraction of nearly-equal numbers). It's used here to get the time-derivative of the velocity Jacobian (`Jvd`, `Jwd`) without hand-deriving a more complex analytical expression.

### Body properties (`rcflex_body_properties.m`)

This function loads the generated 3D mesh assembly (`assets/geometry/assembly.mat`, produced by `generate_rcflex_assets.m` — see `05_GEOMETRY_AND_VISUALIZATION.md`) and computes physical mass/inertia for each rigid body group:
- `B(1)` — the "pitch carrier" (the intermediate body between handle and spoon).
- `B(2)` — the "spoon carrier" (which includes the spoon bowl, neck, and — importantly — the removable food/liquid payload mass `H.food_mass_kg`, added as a point mass at the bowl's measurement location).
- `B(3), B(4)` — the two actuator coil movers (their moving mass), with `B(4)` (the rear one) getting extra mass/inertia added for a coaxial shaft extension.
- `B(5), B(6)` — the two pushrods, modeled as slender cylinders with their own translational and transverse rotational inertia.

Masses for custom (non-purchased) parts come from the actual triangle-mesh geometry via `rcflex_mesh_mass_properties.m` (exact tetrahedral-decomposition volume integration — a standard technique to get a closed mesh's centroid and inertia tensor exactly, not approximately), assuming a uniform material density. Purchased components (actuators, C-Flex flexures) use catalog or estimated masses instead.

### Sensor truth model (`rcflex_sensor_truth.m`)

Given the plant state and base motion, this computes what two IMUs — one fixed to the handle, one fixed to the moving spoon carrier — would perfectly (noiselessly) measure: 3-axis angular velocity (gyroscope) and 3-axis specific force / accelerometer reading (which includes both gravity and the sensor's own linear acceleration — this is physically accurate: a real accelerometer cannot distinguish gravity from acceleration, which is exactly why the controller's attitude estimator has to use gyroscope-primary, accelerometer-correction logic — see `03_CONTROL_SYSTEM.md`). It numerically differentiates position twice (a small central-difference over `dt=1e-4` seconds) to get acceleration, rather than analytically deriving it — a pragmatic implementation shortcut.

This "truth" function is deliberately separate from the noisy/sampled/delayed sensor model used by the actual controller (`rcflex_imu_sample.m` in `src/control/`) — the truth model represents "what a perfect sensor would read," and the sampling model adds all the real-world imperfections on top.

## `rcflex_parameters.m` — where the numbers live

This single function returns a struct `H` with essentially every dimension, mass, electrical, and mechanical constant in the project: overall geometry in millimetres (bowl size, neck length, handle shape), actuator specs (from the Moticont datasheet), flexure stiffness (converted from the C-Flex datasheet's in·lbf/degree units into SI Nm/rad), motor/driver electrical parameters, battery model, IMU noise specs, thermal parameters, and the linkage geometry (`H.link.*` — attachment points, slider axis, and the derived `input_length` for each rod). If you ever want to change a physical assumption (a mass, a stiffness, a dimension), this is the one file to look at first — nearly everything downstream reads from this struct rather than hard-coding numbers itself.
