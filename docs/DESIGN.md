# Current mechanical design

This document consolidates the accepted V15–V17 design. Numerical geometry
is defined by `src/model/rcflex_parameters.m` and `src/geometry/generate_rcflex_assets.m`.

## Geometry and arrangement

| Feature | Current nominal value |
|---|---|
| Overall length | 271 mm; spoon tip x=-103, handle rear x=168 |
| Spoon bowl | 62 × 40 × 12 mm; rim at z=-4 mm |
| Bowl wall | 1.2 mm rim, 1.8 mm bottom; vertical thickness convention |
| Neck | 31 mm long, 10 mm at bowl junction, 6 mm minimum, 6.4 mm terminal width |
| Fixed spherical bezel | 42 mm maximum diameter; x=-16 to 0 |
| Moving spherical carrier | 34 mm diameter, 0.8 mm nominal wall |
| Grip sections | 33 × 28 mm at x=55; 32.6 × 27 at x=80; 34.4 × 30 at x=130 |
| Motor centres | [55.5,-5,6] and [87.5,5,6] mm |
| PCB target | 45 × 16 mm, centre [133,0,12] mm; not routed |
| Battery envelope | 41 × 20 × 13 mm, centre [133,0,4] mm |
| Handle IMU | [117,4,13] mm |

The shape uses a continuous silver upper shell, dark lower grip, rounded
control strip, spherical front cover and moving inner collar. Bowl shoulder
and neck use smooth width and height blends. The rear bowl depth blend has
zero longitudinal slope at the neck. The widest bowl section is 37.2 mm
from its tip; the 28 mm front-width station is inferred, not dimensioned by
the original reference. Perspective artwork does not uniquely specify CAD.

## Motion and physical representation

Two C-Flex B-10 pivots provide pitch support and one D-10 provides roll.
The spherical carrier is custom and follows those constrained axes; it is
not an unconstrained ball/socket bearing. Two guided Moticont motors are
fixed to the handle. Their direct rods terminate at a common neutral plane
x=30 mm. The rear actuator uses a 32 mm long, 3 mm diameter coaxial extension.
The extension and both rods contribute moving mass and inertia. Frame,
spoon and neck mass moments come from closed meshes with assumed densities.

The nominal joint range remains ±15°. The ±20° text in the product artwork
was not adopted. The centre is an equivalent local flexure centre, not a
synthesized or experimentally verified remote rotation centre.

## Limits requiring hardware validation

The sphere/bezel nominal radial gap is 2.8 mm; the smallest sampled bezel
gap is approximately 0.35 mm at the neck seal. The rear shaft extension
passes the front motor with approximately 0.55 mm nominal radial clearance.
These are nominal dimensions, not a tolerance stack-up or a complete
continuous collision certificate. Custom rod ends, quick release and mounts
still require detailed mechanical design. Extension bending, bearing moment,
fatigue and coupling stiffness require verification; the catalogue side-load
rating alone does not approve an extended shaft. PCB routing, wiring,
thermal performance and physical grip comfort remain unvalidated.

Current test output is under `output/results/`; old revision-specific figures
and suppression scores were removed from the active project during cleanup.
