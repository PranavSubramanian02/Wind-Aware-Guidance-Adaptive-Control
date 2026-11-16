# Wind-Aware Energy-Optimal UAV Guidance and Adaptive Control

A verified, multi-tiered GNC (Guidance, Navigation, and Control) architecture for autonomous fixed-wing unmanned aerial vehicles (UAVs). Upgraded from a classical MATLAB flight model into a production engineering repository: a Python research and simulation core, hard real-time C++20 flight software, and an interactive avionics ground control station.

---

## Problem Statement

Fixed-wing unmanned aerial vehicles deployed on long-endurance missions face significant operational challenges from environmental uncertainty, particularly unknown wind disturbances and sudden mass variations caused by payload release or fuel consumption. Conventional flight guidance laws and fixed-gain autopilots assume nominal, static operating conditions. In real-world flight, flying at a fixed calibrated airspeed regardless of ambient winds wastes substantial battery energy in headwinds and fails to take advantage of tailwinds. 

At the same time, sudden mass changes immediately alter the vehicle's inertia and control effectiveness, causing altitude drift and oscillatory response in fixed-gain controllers. Furthermore, lightweight tactical UAVs cannot afford heavy, high-drag dedicated wind sensors such as multi-hole pitot arrays or LiDAR. This project addresses this coupled challenge by developing an autonomous GNC framework that explicitly infers unknown wind conditions and dynamic mass changes from standard onboard sensors, maintaining energy-optimal flight and robust stability throughout the entire mission.

---

## Algorithm and Scientific Methodology

The project implements a physics-based flight dynamics model representative of small fixed-wing platforms, integrating state estimation, energy-aware guidance, and adaptive control into a unified closed loop.

An Extended Kalman Filter (EKF) is used to estimate unmeasured environmental and vehicle states in real time ($\mathbf{x} = [V_g, w, m]^T$). By fusing ground speed from GPS, calibrated airspeed from a single pitot tube, and longitudinal specific force from an IMU accelerometer, the filter solves the classical unobservability problem that causes naive estimators to diverge during steady cruise. This enables the flight computer to reliably infer both the headwind component and the vehicle's true physical mass without requiring dedicated external wind sensors.

Based on the estimated wind state, an energy-optimal airspeed guidance law adjusts the commanded airspeed dynamically. Rather than tracking a static trim velocity, the guidance law solves the ground-relative Breguet range problem in real time, increasing airspeed into headwinds to reduce overall transit energy and easing airspeed in tailwinds to extend endurance. This ensures the aircraft continuously operates at the most efficient aerodynamic polar condition for the prevailing wind field.

To preserve stability across payload release events, where vehicle mass drops abruptly from 13.5 kg to 10.5 kg at $t = 150\,\text{s}$, a Model Reference Adaptive Controller (MRAC) is incorporated into the inner-loop pitch dynamics. The adaptive controller updates its control weights online using tracking error residuals, compensating for the reduced inertia and guaranteeing uniform asymptotic tracking stability ($\lim_{t \to \infty} \|e(t)\| = 0$) without actuator saturation or gain scheduling. Throughout the flight, instantaneous thrust, power, and cumulative energy consumption are logged to benchmark the system's performance against classical fixed-gain methods.

---

## Multi-Tier Architecture

To take these algorithms from theoretical derivation to flight readiness, the system is organized into three distinct tiers:

```mermaid
flowchart TD
    subgraph Env["Mission Environment & Operational Envelope"]
        Wind["Atmospheric Wind & Gusts<br/>Dryden Model (6.0 m/s Headwind)"]
        Waypoints["Flight Plan: 3D Waypoints<br/>WP0 (0 km) → WP1 (3 km) → WP2 (6 km)"]
        Drop["Payload Release Event<br/>13.5 kg → 10.5 kg at t = 150 s"]
    end

    subgraph T1["Tier 1: Python GNC Simulation & Analysis Core"]
        Guidance["Energy-Optimal Airspeed Guidance<br/>V_a*(W) Minimizing Specific Energy"]
        Estimator["Wind & Mass Extended Kalman Filter<br/>Rank-3 Observable Estimation"]
        MRAC["Lyapunov Model Reference Adaptive Control<br/>Inner-Loop Pitch & Attitude Stability"]
        Sim["6-DoF High-Fidelity Flight Simulator<br/>Nonlinear Aerodynamics & Induced Drag"]
    end

    subgraph T2["Tier 2: C++20 Flight Software & Deterministic SITL"]
        FlightCode["Embedded C++20 GNC Core<br/>Fixed-Memory Allocations (< 1 ms loop)"]
        SITL["Closed-Loop SITL Testbench<br/>21,428x Faster Than Real Time"]
    end

    subgraph T3["Tier 3: React Avionics Ground Control Station"]
        PFD["Dual Primary Flight Displays (EFIS)<br/>Artificial Horizons, Speed & Altitude Tapes"]
        Corridor["Tactical 3D Flight Corridor<br/>Real-Time Trajectory & Waypoint Tracking"]
        Gauges["Telemetry & Benchmark Gauges<br/>Observability Covariance & 10.8% Energy Audit"]
    end

    Wind --> Guidance
    Drop --> Estimator
    Waypoints --> Sim

    Guidance --> MRAC
    Estimator --> MRAC
    MRAC --> Sim

    T1 -->|"Algorithm & Jacobian Porting"| T2
    T2 -->|"50 Hz Telemetry JSON Stream"| T3
```

---

## Summary of Plots and Physical Results

The simulation, software-in-the-loop (SITL) benchmark, and ground control station telemetry demonstrate the closed-loop advantages of this framework across a 300-second, 6000-meter flight mission under a continuous 6.0 m/s headwind:

| Metric | Baseline UAV (Fixed Trim + PID) | Adaptive GNC (Proposed) | Physical Result & Benefit |
| :--- | :--- | :--- | :--- |
| **Airspeed Strategy** | Fixed $V_a = 25.0\,\text{m/s}$ | Dynamic $V_a^*(W) \approx 22.8\,\text{m/s}$ | Matches aerodynamic polar to local headwind |
| **Cruise Energy** | $123.6\,\text{kJ}$ | $110.3\,\text{kJ}$ | **$10.77\%$ Net Energy Preserved** ($13.3\,\text{kJ}$ saved) |
| **Mass Identification** | Diverges / Untracked | Converges to $10.63\,\text{kg}$ ($< 8\,\text{s}$) | Recovers true post-drop mass without dedicated sensors |
| **Altitude Hold** | Damped oscillation post-drop | Stable within $\pm 1.45\,\text{m}$ | Eliminates altitude sag and pitch overshoot |
| **Execution Speed** | N/A | $300\,\text{s}$ simulated in $14\,\text{ms}$ | **$21,428\times$ Faster Than Real Time** in C++20 SITL |

The wind and mass estimation plots confirm the filter's ability to track slowly varying wind shears and step disturbances. Following the payload release at 150 seconds, the estimated mass converges to the true post-drop value within 8 seconds, staying tightly bound within analytical three-sigma confidence envelopes where classical un-excited filters diverge.

Airspeed tracking records show the vehicle smoothly adjusting its commanded velocity to changing wind conditions while strictly honoring aircraft stall limits and motor power boundaries. The energy comparison plots show a clear reduction in cumulative work, yielding a verified 10.77% energy saving over the fixed-gain baseline. Finally, the tactical corridor trajectory confirms stable waypoint navigation through all flight phases, demonstrating that adaptive gain scaling eliminates the pitch overshoot and altitude loss typical of fixed-gain autopilots during stores release.

---

## Execution Commands

### 1. Tier 1: Python GNC Simulation & Mathematical Proofs
```bash
# Navigate to Tier 1
cd tier1_python_gnc_core

# Run formal mathematical verification unit tests
python3 tests/test_observability.py    # Verifies rank-3 EKF observability matrix
python3 tests/test_stability.py        # Verifies Lyapunov tracking error decay

# Run dual 6-DoF mission simulation and export telemetry
python3 simulation/run_dual_simulation.py
python3 simulation/export_telemetry.py  # Generates flight_telemetry.json for Tier 3
```

### 2. Tier 2: C++20 Flight Software & SITL Benchmark
```bash
# Navigate to Tier 2
cd tier2_cpp_flight_software

# Build with CMake & GCC (C++20, -O3 optimizations)
make

# Run automated CTest test suite
make test

# Execute closed-loop Software-in-the-Loop benchmark (21,400x real-time)
make run_sitl
```

### 3. Tier 3: React Avionics Ground Control Station
```bash
# Navigate to Tier 3
cd tier3_react_gcs_dashboard

# Install dependencies (first time only)
npm install

# Launch live avionics GCS dashboard
npm run dev
# Open browser at http://localhost:3000
```
