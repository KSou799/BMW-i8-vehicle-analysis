# BMW i8 Vehicle Performance Analysis (GNU Octave)

Vehicle dynamics simulation of the BMW i8 Coupe using GNU Octave / MATLAB.  
Evaluates tractive force, resistive forces, and longitudinal acceleration performance across a 6-speed gearbox.

---

## Objective

- Compute per-gear tractive force envelopes and compare against total driving resistance
- Quantify aerodynamic drag, rolling resistance, and grade resistance contributions across operating conditions
- Simulate longitudinal acceleration per gear using a drivetrain inertia-corrected force model

---

## Vehicle Specifications

| Parameter | Value |
|-----------|-------|
| Vehicle | BMW i8 Coupe |
| Mass | 1530 kg |
| Max power | 275 kW |
| Transmission | 6-speed (ratios: 4.46, 2.51, 1.56, 1.14, 0.85, 0.67) |
| Drag coefficient Cd | 0.26 |
| Frontal area | 2.13 m² |

Parameters based on published BMW i8 Coupe specifications.

---

## Methods

- **Tractive force:** Computed per gear from engine torque, gear ratio, final drive ratio, and drivetrain efficiency (`Fx = η · Mm · i_f · i_g / Rd`)
- **Aerodynamic drag:** Modeled from Cd, frontal area, and air density as a function of vehicle speed
- **Rolling resistance:** Derived from per-axle load distribution with tire pressure and normal load corrections (`fR · (1.3 − 0.3·p/pT) · (1.3 − 0.3·N/G)`)
- **Grade resistance:** Evaluated across road inclinations from 0% to 100%; visualized as a 3D surface (gradient × speed × power)
- **Inertia correction:** Drivetrain rotational inertia included via equivalent mass factor `φ = 1.04 + 0.0025·(i_g · i_f)²`
- **CVT reference:** Theoretical CVT power hyperbola overlaid on tractive force plot as upper-bound comparison

---

## Outputs

| Figure | Description |
|--------|-------------|
| 1 | Aerodynamic drag force and power vs. speed |
| 2 | Rolling resistance force and power vs. speed |
| 3 | Grade resistance power demand — 3D surface (inclination vs. speed) |
| 4 | Tractive force vs. total resistance — all gears + CVT reference |
| 5 | Longitudinal acceleration vs. speed per gear |

---

## Sample Results

### Figure 1 – Aerodynamic Drag
![Aerodynamic Drag](aerodynamic_drag.png)

### Figure 2 – Rolling Resistance
![Rolling Resistance](rolling_resistance.png)

### Figure 3 – Grade Resistance
![Grade Resistance](grade_resistance.png)

### Figure 4 – Tractive Force
![Tractive Force](tractive_force.png)

### Figure 5 – Acceleration Performance
![Acceleration](acceleration.png)

---

## How to Run

1. Install [GNU Octave](https://octave.org/)
2. Run the main script:

```octave
MainScript
```

3. All figures generate automatically in separate windows.

