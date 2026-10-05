# Thermal and Battery Estimates

Working note supporting material selection for the spoon body and the endurance figure quoted
in the report. Numbers are derived from the closed-loop simulation over the sixteen
hardware-feasible, in-band benchmark scenarios; assumptions are marked where they are
assumptions.

## A. Heat Generated

Essentially all electrical input becomes heat: the mechanical work done against a 5 g payload
at tremor amplitudes of 1–3° is negligible beside copper loss, so coil dissipation is the whole
thermal problem.

**TABLE I. TOTAL COPPER LOSS, BOTH COILS, IN-BAND FEASIBLE SCENARIOS**

| Case | Total coil dissipation | Combined coil current (RMS) |
|---|---:|---:|
| Median scenario | 2.38 W | 0.74 A |
| Mean of 16 scenarios | 2.57 W | — |
| Worst scenario (15 g payload) | 3.98 W | 0.93 A |

For reference, the manufacturer's continuous rating is 4.4 W **per coil**, so even the worst
in-band scenario runs both coils together at about 45 % of what one is rated to dissipate
continuously. Thermally the actuator is not the constraint; where it lands on the structure is.

## B. Component Temperatures

Two independent estimates were made, and they agree — which is the main reason to trust either.

**1) From the modelled thermal network.** Each coil has a lumped thermal resistance of
18 K/W to ambient and a capacity of 8 J/K, giving a time constant of *RC* = **144 s**. This
matters for reading the benchmark: the 4-second scenarios show only 0.4–1.0 K of rise, which is
the first 3 % of the transient and says nothing about steady state.

**2) From the handle's external surface.** At steady state all the heat must leave through the
outside of the handle. Treating the grip as an elliptical cylinder 33 × 28 mm over 168 mm gives
a surface area of 0.0161 m², and applying natural convection plus radiation, ΔT = *P* / (*hA*).

**TABLE II. STEADY-STATE TEMPERATURES, 25 °C AMBIENT**

| | Median (2.38 W) | Mean (2.57 W) | Worst (3.98 W) |
|---|---:|---:|---:|
| Coil, modelled network | 46 °C | 48 °C | 61 °C |
| Handle surface, *h* = 6 W/m²K | 50 °C | 52 °C | 66 °C |
| Handle surface, *h* = 10 W/m²K | 40 °C | 41 °C | 50 °C |
| Handle surface, *h* = 14 W/m²K | 36 °C | 36 °C | 43 °C |

**The consistency check.** The modelled network's total conductance is 2 / 18 = 0.111 W/K,
which over 0.0161 m² of handle is equivalent to *h* ≈ **6.9 W/m²K**. Natural convection for a
small object in still air is typically 5–10 W/m²K, so the simulation's thermal assumption sits
at the conservative end of that band and the two estimates describe the same device. Taking the
*h* = 6–10 W/m²K rows as the working range:

> **Design figure: handle surface 40–52 °C in normal use, up to 66 °C in the worst in-band
> scenario, at 25 °C ambient. Coil interior 46–61 °C.**

Two things push this the wrong way and neither is modelled. A hand wrapped around the grip
covers much of the convecting area and adds an insulating layer, so held temperatures will
exceed still-air figures. And ambient is not always 25 °C.

## C. Implications for Material Selection

The handle and the bowl have different binding constraints, and it is worth separating them
because the bowl is detachable.

**Handle shell — bounded by the electronics.** It must hold shape at a sustained 50 °C with a
worst case near 66 °C, and it is gripped for the length of a meal.

| Material | Heat-deflection / *T*_g | Verdict |
|---|---|---|
| PLA | HDT ≈ 50–55 °C, *T*_g ≈ 55–60 °C | **Unsuitable.** Softens inside the normal operating range |
| PETG | HDT ≈ 70 °C, *T*_g ≈ 80 °C | Marginal. Adequate typically, little margin at worst case |
| ABS | HDT ≈ 90 °C, *T*_g ≈ 105 °C | **Suitable**, with comfortable margin |
| PA12 (nylon) | HDT ≈ 95 °C | **Suitable**; also tough and fatigue-tolerant |
| Polycarbonate | *T*_g ≈ 147 °C | **Suitable**; stiffest of the four |

PLA is ruled out on thermal grounds alone, which is worth stating explicitly because it is the
default choice for a printed prototype and would fail in service rather than on the bench.

**Bowl — bounded by cleaning, not by electronics.** The bowl is detachable and carries no heat
source, so it never sees more than food temperature in use. Its real thermal requirement is
dishwashing: wash cycles run to roughly 70 °C and drying to about 85 °C, which is a more
demanding requirement than anything the electronics impose. That again excludes PLA and PETG,
and points to PP, ABS or a food-contact-grade PA — with food-contact compliance, not stiffness,
as the primary selection criterion.

**Touch temperature.** Grip temperature should be assessed against EN ISO 13732-1, which gives
burn thresholds by material and contact duration. The relevant regime here is continuous contact
for the length of a meal — roughly ten minutes — where the threshold for plastics is well below
the one-minute figure, and where comfort becomes binding before injury does. The 40–52 °C
working range is likely acceptable for a polymer grip but should be confirmed against the
standard rather than assumed, and the 66 °C worst case should be treated as a limit to design
away from. A textured or ribbed grip would help by increasing surface area, and siting the coils
away from the natural grip position would help more.

**Battery.** The pack sits inside the handle at roughly the surface temperature computed above.
LiPo cells are typically rated 0–60 °C for discharge but only 0–45 °C for charge, so the cell
itself is within specification in use, while **charging the device immediately after use, while
the handle is still warm, could exceed the charge-temperature limit**. Either allow a cool-down
before charging or include a temperature interlock.

## D. Battery Life

The pack is 2S 400 mAh at 7.4 V nominal, i.e. **2.96 Wh (10.7 kJ)**. Assumed on top of the
simulated coil dissipation: 90 % driver efficiency, and a 0.18 W allowance for the MCU, two
IMUs, the sEMG front end and driver quiescent current. **Neither is modelled**, so these are
engineering allowances rather than results.

**TABLE III. ENDURANCE**

| Condition | Coil dissipation | Total draw | Runtime |
|---|---:|---:|---:|
| Median scenario | 2.38 W | 2.82 W | **63 min** |
| Mean of 16 scenarios | 2.57 W | 3.04 W | **58 min** |
| Worst scenario | 3.98 W | 4.60 W | **39 min** |
| Idle, electronics only | 0 W | 0.18 W | 16.4 h |

**Roughly one hour of continuous tremor rejection.** The important qualifier is that this is
*continuous* worst-case duty — constant tremor at benchmark amplitude with the controller
working throughout. Real use is intermittent: the device is set down between mouthfuls, tremor
amplitude varies, and the authority limiter reduces effort when it can. Endurance across a
realistic meal will be considerably better than one hour, but the honest headline for the report
is the continuous figure, because it is the one that can be derived rather than assumed.

Three caveats. No battery derating or discharge curve is applied, so usable capacity below the
7.4 V nominal is optimistic. The 0.18 W electronics allowance is unverified and would need
measurement. And cell capacity falls at low temperature, which is not considered.

## E. What Would Change These Numbers

The thermal resistance (18 K/W) and capacity (8 J/K) are assumptions, not measured or
manufacturer values, and they set every temperature above. The convection coefficient is a
textbook range, not a measurement for this geometry. Neither the conduction path from coil to
shell nor the effect of a hand wrapped around the grip is modelled. A single thermocouple
measurement on a powered prototype, run to steady state over roughly ten minutes, would replace
the whole of Section B with data — and it is the cheapest useful hardware measurement available
to this project.

## References

- EN ISO 13732-1:2008, *Ergonomics of the thermal environment — Methods for the assessment of
  human responses to contact with surfaces — Part 1: Hot surfaces.*
- Moticont, GVCM-016-019-01M voice coil motor data sheet (continuous power and force ratings).
- Gens Ace GEA4002S60JST, 2S 400 mAh 7.4 V LiPo pack specification.
