#include "control_allocator.hpp"
#include <algorithm>
#include <cmath>

namespace gnc {

ControlAllocator6DoF::ControlAllocator6DoF(
    double kp_p, double kd_p, double ki_a,
    double kp_r, double kd_r, double kp_y
) : kp_pitch(kp_p), kd_pitch(kd_p), ki_alt(ki_a),
    kp_roll(kp_r), kd_roll(kd_r), kp_yaw(kp_y) {}

void ControlAllocator6DoF::reset() {
    alt_int = 0.0;
}

ActuatorCommands ControlAllocator6DoF::allocate(
    double throttle_cmd,
    const State12& x_state,
    double h_cmd,
    double chi_cmd,
    double dt
) {
    const double pd = x_state[2];
    const double phi = x_state[6], theta = x_state[7], psi = x_state[8];
    const double p = x_state[9], q = x_state[10], r = x_state[11];
    const double h_current = -pd;

    // Pitch & Altitude Loop -> Elevator
    const double h_err = h_cmd - h_current;
    alt_int += h_err * dt;
    alt_int = std::clamp(alt_int, -20.0, 20.0);

    double theta_cmd = 0.03 * h_err + ki_alt * alt_int;
    theta_cmd = std::clamp(theta_cmd, -15.0 * (M_PI / 180.0), 20.0 * (M_PI / 180.0));

    double delta_e = -(kp_pitch * (theta_cmd - theta) - kd_pitch * q);
    delta_e = std::clamp(delta_e, -max_elevator, max_elevator);

    // Roll & Heading Loop -> Aileron
    double psi_err = chi_cmd - psi;
    psi_err = std::atan2(std::sin(psi_err), std::cos(psi_err));

    double phi_cmd = std::clamp(0.6 * psi_err, -30.0 * (M_PI / 180.0), 30.0 * (M_PI / 180.0));
    double delta_a = kp_roll * (phi_cmd - phi) - kd_roll * p;
    delta_a = std::clamp(delta_a, -max_aileron, max_aileron);

    // Yaw Damper -> Rudder
    double delta_r = -kp_yaw * r;
    delta_r = std::clamp(delta_r, -max_rudder, max_rudder);

    const double throttle = std::clamp(throttle_cmd, 0.0, 1.0);

    return ActuatorCommands{throttle, delta_e, delta_a, delta_r};
}

} // namespace gnc