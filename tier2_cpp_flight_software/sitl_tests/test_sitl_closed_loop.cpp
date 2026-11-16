#include "attitude_estimator.hpp"
#include "control_allocator.hpp"
#include "mrac_controller.hpp"
#include "observable_ekf.hpp"
#include "six_dof_physics.hpp"
#include "waypoint_guidance.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace gnc;

int main() {
    std::cout << "--- Starting 6-DoF C++ SITL Closed-Loop Flight Simulation ---\n";

    constexpr double dt = 0.02;
    constexpr double t_end = 300.0;
    const auto total_steps = static_cast<std::size_t>(t_end / dt);

    std::vector<Waypoint> route{
        {0.0, 0.0, 100.0},
        {3000.0, 0.0, 100.0},
        {6000.0, 0.0, 100.0}
    };

    SixDoFPhysics physics{};
    ObservableEKF ekf(dt, 13.5, 25.0, 0.0);
    AttitudeEstimatorAHRS ahrs(dt);
    LongitudinalMRAC mrac(dt);
    ControlAllocator6DoF allocator{};
    WaypointGuidance3D guidance(route, 35.0);

    // Initial 12-state: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    State12 x{0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ActuatorCommands u_act{0.45, 0.0, 0.0, 0.0};
    Vector3 wind{6.0, 0.0, 0.0}; // 6 m/s North headwind

    double cumulative_energy_joules = 0.0;

    const auto start_wall_clock = std::chrono::high_resolution_clock::now();

    for (std::size_t k = 0; k < total_steps; ++k) {
        const double t = k * dt;

        // Payload release at t = 150 s
        if (std::abs(t - 150.0) < 0.5 * dt) {
            physics.params.drop_payload();
        }

        // Relative airspeed
        const double ur = x[3] - wind[0];
        const double vr = x[4] - wind[1];
        const double wr = x[5] - wind[2];
        const double Va = std::max(1e-4, std::sqrt(ur * ur + vr * vr + wr * wr));

        // Power consumption
        const double thrust = u_act.throttle * physics.params.T_max;
        cumulative_energy_joules += thrust * Va * dt;

        // Specific force & estimation
        const double q_inf_s = 0.5 * 1.225 * Va * Va;
        const double CL_s = (2.0 * physics.params.mass * physics.params.g) / (1.225 * physics.params.S * Va * Va);
        const double CD_s = physics.params.CD0 + physics.params.k_induced * CL_s * CL_s;
        const double Drag_s = q_inf_s * physics.params.S * CD_s;
        const double ax_specific = (thrust - Drag_s) / physics.params.mass;
        ekf.predict(thrust);
        ekf.update(Va, x[3], ax_specific, thrust);

        const auto [est_Vg, est_w, est_m] = ekf.get_state();

        // Analytical optimal airspeed guidance
        const double A = 0.5 * 1.225 * physics.params.S * physics.params.CD0;
        const double B = (2.0 * physics.params.k_induced * std::pow(est_m * physics.params.g, 2)) / (1.225 * physics.params.S);
        double Va_cmd = std::pow(B / A, 0.25) + 0.4 * est_w;
        Va_cmd = std::clamp(Va_cmd, 15.0, 32.0);

        // 3D Guidance & Control Allocation
        const auto guide = guidance.update(x[0], x[1], x[2]);
        const double throttle = mrac.update(Va, Va_cmd, 0.42);
        u_act = allocator.allocate(throttle, x, guide.h_cmd, guide.chi_cmd, dt);

        // Physics propagation
        x = physics.step_rk4(x, u_act, wind, dt);
    }

    const auto stop_wall_clock = std::chrono::high_resolution_clock::now();
    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop_wall_clock - start_wall_clock).count();

    const double final_alt = -x[2];
    const auto [final_Vg, final_w, final_m] = ekf.get_state();

    assert(std::abs(final_alt - 100.0) < 2.5); // Altitude hold within 2.5m
    assert(std::abs(final_m - 10.5) < 1.0);     // Mass identified accurately

    std::cout << "--------------------------------------------------------\n";
    std::cout << "SITL Mission Time Simulated: " << t_end << " s (15,000 steps)\n";
    std::cout << "C++ Wall-Clock Execution Time: " << duration_ms << " ms ("
              << (t_end * 1000.0 / duration_ms) << "x Real-Time Speed)\n";
    std::cout << "Final Altitude: " << final_alt << " m (Command: 100.0 m)\n";
    std::cout << "True Post-Drop Mass: " << physics.params.mass << " kg\n";
    std::cout << "EKF Identified Mass: " << final_m << " kg\n";
    std::cout << "Total Energy Consumed: " << (cumulative_energy_joules / 1000.0) << " kJ\n";
    std::cout << "[PASSED] 6-DoF SITL Closed-Loop C++ Flight Software Benchmark\n";
    std::cout << "--------------------------------------------------------\n";

    return 0;
}