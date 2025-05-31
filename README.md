# Aircraft Pitch Control System Analysis and Airfoil Visualization

This MATLAB project simulates and analyzes the pitch dynamics of an aircraft using various control techniques. It includes open-loop and closed-loop response analysis, lead compensator design, LQR controller implementation with actuator lag, and 2D airfoil visualization with rotation. The project also evaluates system performance and control input effort over time.

---

## 📂 Contents

- Open-loop Step Response
- Frequency Response and Lead Compensator Design
- LQR Control with Actuator Lag
- Airfoil Visualization and Rotation
- Control Input Over Time
- Simulation from Non-Zero Initial State

---

## 🛠️ Dependencies

- MATLAB (Tested with R2023a and above)
- Control System Toolbox

---

## 📈 Open-Loop System Response

The aircraft pitch transfer function is modeled as:

```matlab
P_pitch = (1.151*s + 0.1774)/(s^3 + 0.739*s^2 + 0.921*s);
```
A step response is generated for analysis, and system poles are examined to evaluate open-loop behavior.

---

## Frequency Response and Lead Compensator Design

To improve system stability and transient performance, a lead compensator is designed:

```matlab
C_lead = K * (T*s + 1) / (alpha*T*s + 1);
```
Bode plots and margin analysis are used to determine gain and phase margins. A closed-loop step response is plotted to observe improvements.

---

## LQR Controller with Actuator Lag

A more realistic control model is created by augmenting the system with actuator lag. State-space matrices are defined as:
```matlab
A_aug = [A, B; zeros(1,3), -1/tau];
B_aug = [zeros(3,1); 1/tau];
C_aug = [C, 0];
```
An LQR controller is designed with:
```matlab
K_aug = lqr(A_aug, B_aug, Q_aug, R);
```
The closed-loop system is simulated, and both the pitch angle response and control effort over time are visualized.

---

## Airfoil Shape and Rotation (NACA 2412)
The NACA 2412 airfoil is generated using standard thickness distribution equations:
```matlab
y = 0.12 * (0.2969 * sqrt(x) - 0.1260 * x - 0.3516 * x.^2 + 0.2843 * x.^3 - 0.1036 * x.^4);
```
The airfoil shape is centered and rotated using a 2D rotation matrix. Both original and rotated shapes are plotted to simulate pitch angle visually.

---

## Control Input Over Time
After applying the LQR controller, the control input u(t) is computed as:
```matlab
u = -K_aug * x' + Nbar * r;
```
This is plotted over time to evaluate the control effort required for tracking.

---

## Simulation from Non-Zero Initial State
The system is simulated from an initial state with a step reference input. The resulting pitch angle over time is plotted, showcasing system convergence and stability under the LQR controller with actuator lag.

---

## Output Visualizations
- Open-loop and closed-loop step responses

- Bode and gain/phase margin plots

- LQR-controlled system response

- Control input u(t) over time

- Airfoil shape (original and rotated)

- Pitch angle response from initial conditions

---

## How to Run
1) Clone this repository.

2) Open MATLAB and navigate to the project folder.

3) Run the script to execute all analyses and generate plots.

---

## References
- Ogata, K. (2010). Modern Control Engineering

- NACA Airfoil Profiles – NASA Archive

- MATLAB Control System Toolbox Documentation

---



