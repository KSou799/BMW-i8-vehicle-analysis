# BMW i8 Vehicle Performance Analysis (GNU Octave / MATLAB)

Comprehensive vehicle dynamics simulation of the **BMW i8 Coupe (2014–2020)** using engineering-based mathematical modeling.  

This project evaluates tractive capability, resistive forces, drivetrain behavior, and acceleration performance across the full operating range of the vehicle.  

The objective is not only to generate plots — but to understand *what limits vehicle performance and why.*

---

## Project Objective

- Model longitudinal vehicle dynamics using first-principles equations  
- Quantify the contribution of aerodynamic drag, rolling resistance, and grade resistance  
- Evaluate tractive force across all gears  
- Determine acceleration performance and performance limits  
- Compare gear-limited vs power-limited behavior  

---

## Engineering Model

### 1. Aerodynamic Drag
\[
F_d = \frac{1}{2} \rho C_d A v^2
\]

- Drag coefficient: 0.26  
- Frontal area: 2.13 m²  
- Air density assumed constant  

**Insight:** Aerodynamic drag dominates at higher speeds and becomes the primary limiting factor near top speed.

**Figure 1 – Aerodynamic Drag vs Velocity**  
![Aerodynamic Drag](images/aerodynamic_drag.png)

---

### 2. Rolling Resistance

Calculated from dynamic axle loads and includes load transfer effects, tire pressure influence, and front/rear axle separation.  

**Insight:** Rolling resistance is significant at low speeds but becomes secondary above ~100 km/h.

**Figure 2 – Rolling Resistance Force vs Velocity**  
![Rolling Resistance](images/rolling_resistance.png)

---

### 3. Grade Resistance

\[
F_g = mg \sin(\alpha)
\]

- Evaluated across multiple road gradients  
- 3D surface shows velocity vs incline vs required power  

**Insight:** Reveals climbing capability and power limitations under steep conditions.

**Figure 3 – Power Requirement vs Road Gradient**  
![Grade Resistance](images/grade_resistance.png)

---

### 4. Tractive Force Modeling

Calculated from engine torque, 6-speed gearbox ratios, final drive, drivetrain efficiency, and tire radius.  

**Insight:** Shows maximum speed in each gear, grade climbing capability, and transition from traction-limited to power-limited regimes.  

**Figure 4 – Tractive Force vs Resistance (All Gears + CVT)**  
![Tractive Force](images/tractive_force.png)

---

### 5. Acceleration Analysis

\[
a = \frac{F_x - F_R}{\phi m}
\]

Where:

- \(F_x\) = tractive force  
- \(F_R\) = total resistance  
- \(\phi\) = drivetrain inertia factor  

**Insight:**  
- High initial acceleration in lower gears  
- Acceleration decreases with speed due to aerodynamic drag  
- Clear transition to power-limited performance  

**Figure 5 – Acceleration vs Velocity for Each Gear**  
![Acceleration](images/acceleration.png)

---

## Key Engineering Insights

- Aerodynamic drag scales with \(v^2\), while required power scales with \(v^3\), making high-speed driving exponentially more energy intensive.  
- Top speed is power-limited rather than gear-limited.  
- Lower gears maximize tractive force but are constrained by RPM limits.  
- Rolling resistance has minimal influence at highway speeds compared to aerodynamic drag.  
- Vehicle climbing capability is strongly dependent on available power at mid-range RPM.

---

## Vehicle Parameters (Model Assumptions)

- Vehicle: BMW i8 Coupe  
- Mass: 1530 kg  
- Maximum power: 275 kW  
- Transmission: 6-speed  
- Drag coefficient (Cd): 0.26  
- Frontal area: 2.13 m²  

**Note:** The hybrid powertrain is modeled as an equivalent single driveline system using combined output for simplification.

---

## Generated Outputs

The script automatically generates:

1. Aerodynamic drag force & power vs velocity  
2. Rolling resistance force & power vs velocity  
3. Power requirement vs velocity & incline (3D surface)  
4. Tractive force vs resistance (all gears + CVT comparison)  
5. Acceleration vs velocity for each gear  

---
## Skills Demonstrated

- Vehicle dynamics  
- Engineering modeling and assumptions  
- Numerical computation in GNU Octave  
- Data visualization and interpretation  

---

## Author

**Sou Komiya**  
Mechanical Engineering Student – Metropolia UAS  
January 2026

---

## How to Run

1. Install GNU Octave or MATLAB  
2. Run:

```octave
MainScript
---
