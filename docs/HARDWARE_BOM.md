# SensoSpoon hardware basis — checked 2026-09-11

These are real catalog parts used as the model's hardware basis. Supplier stock
and NZ delivery are not confirmed and nothing has been ordered. Dimensions of
custom mechanics are proposed, not represented as a purchasable assembly.

| Item | Qty | Selected part / source | Parameters actually used |
|---|---:|---|---|
| Guided voice-coil actuator | 2 | [Moticont GVCM-016-019-01M](https://www.moticont.com/GVCM-016-019-01.htm), [drawing Rev 1](https://www.moticont.com/pdf/GVCM-016-019-01M.pdf) | R=4.6 ohm, L=0.8 mH @1 kHz, Kf=Ke=1.5 SI, total stroke12.7 mm, coil assembly11 g, body17 g, diameter15.9 mm, body19.1 mm plus4.8 mm cap. Continuous1.5 N, intermittent4.6 N @10% duty, continuous copper power4.4 W, side load2.2 N. |
| H-bridge | 2 | [TI DRV8876PWPR](https://www.ti.com/product/DRV8876), [datasheet](https://www.ti.com/lit/ds/symlink/drv8876.pdf) | 4.5 V minimum supply, 3.5 A peak capability, typical combined on-resistance0.7 ohm; averaged bridge and adjustable1.5 A design current limit. |
| Inertial sensor | 2 | [Bosch BMI270](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi270), [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf) | Gyro noise density0.007 deg/s/sqrt(Hz), accel160 ug/sqrt(Hz); 3x2.5x0.8 mm package. Selected800 Hz, +/-500 deg/s and +/-8 g ranges. Handle and moving-spoon sensors. |
| MCU | 1 | [ST STM32G431CBT6](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html), [datasheet](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf) | 170 MHz Cortex-M4/FPU, 128 KB flash, LQFP48; packaging allowance. Firmware execution is not emulated. |
| Battery | 1 | [Gens ace GEA4002S60JST](https://gensace.de/products/gens-ace-400mah-7-4v-60c-2s1p-lipo-battery-pack-with-jst-plug) | 2S7.4 V, 400 mAh, 41x20x13 mm, approx18 g. Nominal supply plus assumed internal resistance. |
| Pitch flexure pivots | 2 | [C-Flex B-10 specification](https://c-flex.com/wp-content/uploads/2025/02/B-10-SERIES-Specification-Sheet.pdf) | OD3.96748 x length6.35 mm; each 0.0004 in*lbf/degree; two parallel pivots give 0.005179 Nm/rad. |
| Roll flexure pivot | 1 | [C-Flex D-10 specification](https://c-flex.com/wp-content/uploads/2025/02/D-10-SERIES-Specification-Sheet.pdf) | OD6.35 x length10.16 mm; 0.0018 in*lbf/degree =0.011652 Nm/rad. Internal leaf pivots supersede custom shim leaves. |
| Spoon, frames, rods, extension, quick release, enclosure, PCB | custom | Source illustration and Hardware Feasibility PDF, with authorized dimensional redesign | Custom parts have specified nominal geometry; material, tolerances, joint seats, masses and stiffness still need mechanical validation. |


## Current assembly and explicit assumptions

The spherical carrier, mounts, rod-end seats and quick release are custom;
there is no sourced complete spherical gimbal. The rear motor uses a 32 mm
long, 3 mm diameter coaxial extension with mass and inertia included. Bending,
adapter stiffness and fatigue are unvalidated. Nominal front-motor clearance
is 0.55 mm. The motor's 2.2 N side-load rating does not certify the additional
extension bending moment. Vendor M2.2x0.45 threads require matching adapters.
Rendered miniature seats are not commercial igus KBRM-02 rod ends.

The 45 x 16 mm PCB is a packaging target, not a routed board. Power copper,
thermal layout, wiring, charger/protection and firmware timing are not fully
simulated. No stock, shipment or complete-assembly suitability is confirmed.

Motor force/stroke variation is an estimated quadratic curve. Thermal
capacity 8 J/K, thermal resistance 18 K/W, guide friction 0.03 N and 0.03 Ns/m,
battery resistance 0.15 ohm, flexure damping and compliant stops are assumptions.
Custom mesh moments assume uniform density; purchased internal mass placement
and C-Flex masses remain approximations. Catalogue torsional rates refer to
zero load. The +/-15-degree indefinite-life statement is conditional on zero
load; loaded fatigue and radial/axial compliance need hardware validation.

Local manufacturer PDFs are in `sources/`. See `DESIGN.md` for current
geometry and `RCFLEX_MODEL_GUIDE.md` for equations and ports. This is a
component-based analytic baseline, not a verified manufacturing BOM or a
bench-calibrated digital twin.
