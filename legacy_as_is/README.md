# Project 1: Wind-Aware Energy-Optimal UAV Guidance and Adaptive Control

## 1. Executive Summary & Problem Statement
Fixed-wing Unmanned Aerial Vehicles (UAVs) performing long-endurance intelligence, surveillance, and reconnaissance (ISR) or delivery missions operate in unpredictable, dynamic environments. Atmospheric wind disturbances and mass variations (arising from payload deployment or fuel consumption) degrade trajectory tracking, compromise flight stability, and drastically reduce usable mission endurance.

Traditional flight control architectures assume nominal operating conditions and employ fixed-gain feedback controllers that fail to adapt when operating assumptions are violated. This project investigates an autonomous guidance and control framework designed to:
1. Estimate unknown wind disturbances and dynamic mass shifts in real time using an Extended Kalman Filter (EKF).
2. Dynamically adapt the commanded airspeed using an energy-optimal guidance law that increases airspeed against headwinds to minimize overall energy expenditure.
3. Apply mass-adaptive feedback control to maintain stable control authority following a sudden payload release.

---

## 2. Theoretical Architecture & Mathematical Models

### 2.1 Vehicle Aerodynamics & Longitudinal Dynamics
The aircraft is modeled as a small fixed-wing UAV (Beard & McLain class) in longitudinal flight:
- **True Airspeed Kinematics**:
  $$V_a = V_g - w_{\text{true}}$$
  where $V_g$ is the ground speed and $w_{\text{true}}$ is the along-track wind speed ($w > 0$ denotes a headwind).
- **Aerodynamic Drag**:
  $$q = \frac{1}{2} \rho V_a^2$$
  $$D = q S \left( C_{D0} + k C_L^2 \right)$$
- **Longitudinal Acceleration**:
  $$a_x = \dot{V}_g = \frac{T - D}{m}$$
  $$\dot{x} = V_g$$

### 2.2 Extended Kalman Filter (EKF) Formulation
The estimator tracks a 3-state vector representing vehicle ground speed, along-track wind speed, and mass bias:
$$\mathbf{x} = \begin{bmatrix} V_g \\ w \\ \Delta m \end{bmatrix}$$

- **State Transition**:
  $$\mathbf{x}_{k} = \mathbf{x}_{k-1} + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$
  $$\mathbf{Q} = \operatorname{diag}([0.1, 0.01, 0.001])$$
- **Measurement Model**:
  Airspeed sensor measurement:
  $$z = V_a = V_g - w$$
  $$h(\mathbf{x}) = x_1 - x_2$$
  $$\mathbf{H} = \begin{bmatrix} 1 & -1 & 0 \end{bmatrix}, \quad R = 2.0$$

### 2.3 Energy-Optimal Guidance Law
Rather than commanding a fixed cruise airspeed $V_a^* = 18\,\text{m/s}$, the guidance law adapts airspeed proportionally to the estimated headwind:
$$V_{a,\text{cmd}} = \operatorname{clip}\left( V_a^* + 0.6 \max(\hat{w}, 0), \; V_{a,\min}, \; V_{a,\max} \right)$$
where $V_{a,\min} = 14\,\text{m/s}$ and $V_{a,\max} = 28\,\text{m/s}$.

### 2.4 Control Laws (Fixed vs. Adaptive)
- **Fixed-Gain Controller**:
  $$T_{\text{fixed}} = D + K_{p,\text{fixed}} (V_{a,\text{cmd}} - V_a), \quad K_{p,\text{fixed}} = 2.2$$
- **Mass-Adaptive Controller**:
  $$K_{p,\text{adapt}} = K_{p,\text{fixed}} \left( \frac{m_{\text{nominal}}}{\hat{m}} \right)$$
  $$T_{\text{adapt}} = D + K_{p,\text{adapt}} (V_{a,\text{cmd}} - V_a)$$

---

## 3. Simulation Parameters

| Parameter | Symbol | Value | Unit | Description |
|:---|:---:|:---:|:---:|:---|
| Time Step | $\Delta t$ | $0.05$ | $\text{s}$ | Discretization interval |
| Duration | $T$ | $300$ | $\text{s}$ | Total mission flight duration |
| Air Density | $\rho$ | $1.225$ | $\text{kg/m}^3$ | Sea-level atmospheric density |
| Nominal Mass | $m_{\text{nominal}}$ | $13.5$ | $\text{kg}$ | Takeoff gross weight |
| Payload Mass | $m_{\text{payload}}$ | $3.0$ | $\text{kg}$ | Released at $t \ge 150\,\text{s}$ |
| Wing Area | $S$ | $0.55$ | $\text{m}^2$ | Reference wing area |
| Parasitic Drag | $C_{D0}$ | $0.028$ | — | Zero-lift drag coefficient |
| Induced Drag Factor | $k$ | $0.045$ | — | Lift-induced drag factor |
| Best Airspeed | $V_a^*$ | $18.0$ | $\text{m/s}$ | Nominal trim cruise speed |

---

## 4. Execution & Usage

Ensure you have a Python environment with `numpy`, `scipy`, `pandas`, and `matplotlib` installed:

```bash
# Execute headless simulation (saves figures to output_figures/)
python3 project1_as_is.py

# Execute with interactive plot display
python3 project1_as_is.py --show
```

### Generated Outputs
The script produces four figures in `output_figures/`:
1. `fig1_wind_estimation.png`: True wind disturbance vs. EKF estimated wind speed.
2. `fig2_airspeed_guidance.png`: Actual airspeed response tracking the energy-optimal commanded airspeed.
3. `fig3_energy_comparison.png`: Cumulative energy expenditure (kJ) comparing fixed-gain and adaptive control.
4. `fig4_3d_trajectory.png`: 3D UAV trajectory visualization showing flight progression and lateral deviation.

Defense Table Output:
```text
 Energy_Fixed_kJ  Energy_Adaptive_kJ  Max_Wind_Est_Error_mps
       26.097142           26.097142                 4.88342
```

---

## 5. Critical Engineering Audit: Holes, Faults & Simulation Limitations

> [!WARNING]
> The following issues exist in the original **as-is** MATLAB implementation (`project1.mlx`) and are faithfully reproduced here for baseline equivalence.

1. **Mass Bias Unobservability in the EKF**:
   - The measurement Jacobian is $\mathbf{H} = [1, -1, 0]$. The 3rd column is identically zero because airspeed $V_a = V_g - w$ does not depend on vehicle mass.
   - The process covariance $\mathbf{Q}$ and error covariance $\mathbf{P}$ are diagonal, so $P_{13} = P_{23} = 0$.
   - The Kalman gain for the mass state is $K_3 = \frac{P_{13} - P_{23}}{S_{\text{innov}}} \equiv 0$.
   - **Result**: $\hat{m}$ never updates and remains frozen at $13.5\,\text{kg}$ for all $300\,\text{s}$. The filter never detects the $3\,\text{kg}$ payload drop at $t = 150\,\text{s}$.
2. **Degenerate Adaptive vs. Fixed Gain Comparison**:
   - Because $\hat{m} = m_{\text{nominal}}$ for all time, $K_{p,\text{adapt}} = K_{p,\text{fixed}} \cdot \left(\frac{13.5}{13.5}\right) \equiv K_{p,\text{fixed}}$.
   - $T_{\text{adapt}}(k) \equiv T_{\text{fixed}}(k)$, producing identical energy values: $26.097\,\text{kJ}$. The claimed "robustness advantage" of adaptive control is an artifact of identical gains.
3. **Observability Indistinguishability (Ground Speed vs. Wind)**:
   - With only one scalar measurement $V_a$, resolving two states ($V_g$ and $w$) is mathematically ill-posed without GPS ground speed measurements or inertial acceleration integration ($a_x = \dot{V}_g$).
4. **Missing Simulated Measurement Noise**:
   - Although $R = 2.0$ is defined, the simulation feeds pristine true airspeed $z = V_a(k)$ into the filter without adding sensor noise (`+ sqrt(R)*randn`).
5. **Hardcoded Lift Coefficient**:
   - Aerodynamic drag is evaluated with a hardcoded $C_L = 0.6$ (`k_drag * 0.6^2`) rather than calculating $C_L = \frac{2mg}{\rho S V_a^2}$. Consequently, the induced drag does not reflect the mass reduction following payload release.
