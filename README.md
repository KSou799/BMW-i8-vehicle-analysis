# BMW i8 Vehicle Performance Analysis (GNU Octave)

Vehicle dynamics simulation of the BMW i8 Coupe using GNU Octave / MATLAB.  
Models the hybrid powertrain as two independent axle systems and evaluates tractive force, resistive forces, and longitudinal acceleration performance.

---

## Objective

- Simulate longitudinal acceleration of a hybrid-electric vehicle with split powertrain architecture
- Evaluate tractive force envelope vs. resistive forces across all gears
- Quantify the contribution of aerodynamic drag, rolling resistance, and road gradient to total driving resistance

---

## Powertrain Architecture

The BMW i8 uses a parallel hybrid layout — modeled here as two independent subsystems:

| Subsystem | Unit | Value |
|-----------|------|-------|
| ICE (rear axle) | Peak power | 170 kW |
| ICE (rear axle) | Transmission | 6-speed gearbox |
| Electric motor (front axle) | Peak power | 105 kW |
| Electric motor (front axle) | Transmission | Single-speed fixed ratio |
| Combined system | Total power | 275 kW |
| Vehicle mass | — | 1530 kg |
| Drag coefficient Cd | — | 0.26 |
| Frontal area | — | 2.13 m² |

Parameters based on published BMW i8 Coupe specifications.

---

## Methods

- **Hybrid powertrain:** ICE and electric motor modeled as separate subsystems with independent gear ratios, torque curves, and per-axle traction limits
- **Tractive force:** Computed from torque × gear ratio × final drive ratio × drivetrain efficiency, per axle
- **Inertia correction:** Rotational inertia of drivetrain components included in longitudinal acceleration via equivalent mass factor
- **Aerodynamic drag:** Modeled from Cd, frontal area, and air density; drag power computed as a function of speed
- **Rolling resistance:** Derived from axle load distribution and tire rolling resistance coefficient
- **Grade resistance:** Evaluated across multiple road inclination angles
- **Acceleration simulation:** Euler integration of net longitudinal force; 0–100 km/h time compared against published specification (4.4 s)

---

## Outputs

| Figure | Description |
|--------|-------------|
| 1 | Aerodynamic drag force and power vs. speed |
| 2 | Rolling resistance force and power vs. speed |
| 3 | Grade resistance power demand at varying road inclinations |
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

---

## Author

**Sou Komiya**  
Mechanical Engineering Student – Metropolia UAS  
January 2026
