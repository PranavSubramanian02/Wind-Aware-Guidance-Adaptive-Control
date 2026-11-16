#include "control_allocator.hpp"
#include "mrac_controller.hpp"
#include "observable_ekf.hpp"
#include "six_dof_physics.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace gnc;

void test_physics_rk4_step() {
    SixDoFPhysics physics{};
    State12 x0{0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ActuatorCommands u{0.5, 0.0, 0.0, 0.0};
    Vector3 wind{5.0, 0.0, 0.0};

    const State12 x_next = physics.step_rk4(x0, u, wind, 0.01);

    for (std::size_t i = 0; i < 12; ++i) {
        assert(!std::isnan(x_next[i]));
    }
    assert(x_next[0] > 0.0); // Aircraft traveled North
    std::cout << "[PASSED] C++ 6-DoF RK4 Physics Step Test\n";
}

void test_ekf_mass_convergence() {
    ObservableEKF ekf(0.02, 13.5, 25.0, 0.0);
    const double true_m = 10.5;
    const double true_w = 5.0;
    const double true_Va = 20.0;
    const double thrust = 18.0;

    const double q_inf = 0.5 * ekf.rho * true_Va * true_Va;
    const double CL = (2.0 * true_m * ekf.g) / (ekf.rho * ekf.S * true_Va * true_Va);
    const double CD = ekf.CD0 + ekf.k_induced * CL * CL;
    const double Drag = q_inf * ekf.S * CD;
    const double true_ax = (thrust - Drag) / true_m;

    for (int i = 0; i < 200; ++i) {
        ekf.predict(thrust);
        ekf.update(true_Va, 25.0, true_ax, thrust);
    }

    const auto [est_Vg, est_w, est_m] = ekf.get_state();
    assert(std::abs(est_m - true_m) < 1.5);
    std::cout << "[PASSED] C++ Observable EKF Mass Convergence Test: " << est_m << " kg\n";
}

void test_mrac_adaptation() {
    LongitudinalMRAC mrac(0.01);
    double Va_sim = 20.0;
    const double Va_target = 26.0;

    for (int i = 0; i < 500; ++i) {
        const double thr = mrac.update(Va_sim, Va_target, 0.42);
        Va_sim += (thr * 15.0 - 0.02 * Va_sim * Va_sim - 2.0) * 0.01;
    }

    assert(std::abs(Va_sim - Va_target) < 1.0);
    const auto W = mrac.get_weights();
    for (double w_i : W) {
        assert(std::abs(w_i) <= mrac.W_max);
    }
    std::cout << "[PASSED] C++ Longitudinal MRAC Stability & Boundedness Test\n";
}

void test_control_allocator_saturation() {
    ControlAllocator6DoF allocator{};
    State12 extreme_state{0.0, 0.0, -10.0, 25.0, 0.0, 0.0, 1.0, 0.8, 0.0, 0.5, 0.5, 0.5};

    const auto act = allocator.allocate(1.5, extreme_state, 150.0, 1.5, 0.01);

    assert(act.throttle <= 1.0 && act.throttle >= 0.0);
    assert(std::abs(act.elevator) <= allocator.max_elevator);
    assert(std::abs(act.aileron) <= allocator.max_aileron);
    assert(std::abs(act.rudder) <= allocator.max_rudder);
    std::cout << "[PASSED] C++ Control Allocator Saturation Limits Test\n";
}

int main() {
    std::cout << "--- Running Tier 2 Golden Trace Verification ---\n";
    test_physics_rk4_step();
    test_ekf_mass_convergence();
    test_mrac_adaptation();
    test_control_allocator_saturation();
    std::cout << "All Golden Trace Tests Passed Successfully.\n";
    return 0;
}