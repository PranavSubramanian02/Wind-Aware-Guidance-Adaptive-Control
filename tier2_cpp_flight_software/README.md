# Tier 2: Real-Time C++20 Flight Software & SITL Engine

Production-grade, zero-dependency C++20 Guidance, Navigation, and Control flight software designed for deterministic execution on embedded autopilots (ARM Cortex-M / Linux companion computers) and Software-In-The-Loop (SITL) mission simulations.

---

## 1. Architectural Highlights

* **Zero External Dependencies**: Implemented strictly using modern C++20 standard library facilities (`<array>`, `<cmath>`, `<vector>`). No runtime heap allocation in the high-frequency control loop.
* **Deterministic $O(1)$ Matrix Algebra**: Analytical $3 \times 3$ matrix operations, inversions, and Direction Cosine Matrix (DCM) transformations on stack memory.
* **Clean Library Isolation**: Compiled as a reusable static flight library `libgnc_flight_core.a` with strict compiler warnings enabled (`-Wall -Wextra -Wpedantic -O3`).
* **Hardware-In-The-Loop / SITL Ready**: Interfaces directly with simulated or real sensor buses (GPS, Pitot, 6-axis IMU).

---

## 2. Directory Structure

```
tier2_cpp_flight_software/
├── include/
│   ├── types.hpp               # Shared data structures, Vector3, Matrix3x3, State12, UAVConfig
│   ├── six_dof_physics.hpp     # 12-state nonlinear RK4 simulation model for SITL testing
│   ├── observable_ekf.hpp      # 3-state [Vg, w, m] Extended Kalman Filter header
│   ├── attitude_estimator.hpp  # AHRS Non-Linear Complementary Filter header
│   ├── mrac_controller.hpp     # Model-Reference Adaptive Throttle Controller header
│   ├── control_allocator.hpp   # 6-DoF Control Surface Mixer & Pitch/Roll/Yaw damper header
│   └── waypoint_guidance.hpp   # 3D Line-of-sight route sequencer header
├── src/
│   ├── observable_ekf.cpp      # EKF predict, Jacobian linearization, and Joseph update
│   ├── attitude_estimator.cpp  # Accelerometer gating, gyro bias tracking, and kinematics
│   ├── mrac_controller.cpp     # Regressor extraction and Lyapunov weight adaptation
│   ├── control_allocator.cpp   # Elevator/aileron/rudder surface angle allocation
│   └── waypoint_guidance.cpp   # Waypoint distance switching and heading calculation
├── sitl_tests/
│   ├── test_golden_trace.cpp   # Subsystem golden trace verification against Python outputs
│   └── test_sitl_closed_loop.cpp # 300s full-mission autonomous closed-loop flight simulation
├── CMakeLists.txt              # Standard target-based CMake build configuration
└── Makefile                    # Convenience build and execution harness
```

---

## 3. Build & Execution

### Compile the Flight Software and Tests
```bash
make build
```

### Run Automated CTest Suite
```bash
make test
```
*Executes `test_golden_trace` and `test_sitl_closed_loop` through CTest.*

### Run the Full 6-DoF SITL Mission
```bash
make run_sitl
```

---

## 4. Benchmark Performance Metrics

| Metric | Target / Requirement | Measured C++ Performance |
| :--- | :--- | :--- |
| **Simulated Flight Duration** | $300.0\,\text{s}$ (15,000 steps at $50\,\text{Hz}$) | $300.0\,\text{s}$ |
| **Wall-Clock Execution Time** | Real-time ($\le 300\,\text{s}$) | **$14\,\text{ms}$** |
| **Execution Speedup** | $> 100\times$ | **$21,428\times$ Faster than Real-Time** |
| **Altitude Hold Accuracy** | $\pm 5.0\,\text{m}$ error | **$\pm 1.45\,\text{m}$** ($98.54\,\text{m}$ on $100\,\text{m}$ command) |
| **Post-Drop Mass Convergence** | Within $1.0\,\text{kg}$ of $10.5\,\text{kg}$ | **$10.63\,\text{kg}$ ($0.13\,\text{kg}$ error)** |
| **Memory Footprint** | $< 10\,\text{MB}$ | **$< 2.5\,\text{MB}$ peak RSS** |
