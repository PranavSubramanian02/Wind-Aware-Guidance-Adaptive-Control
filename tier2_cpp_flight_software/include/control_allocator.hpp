#pragma once

#include "types.hpp"

namespace gnc {

class ControlAllocator6DoF {
public:
    double kp_pitch{1.2};
    double kd_pitch{0.25};
    double ki_alt{0.015};

    double kp_roll{0.8};
    double kd_roll{0.15};
    double kp_yaw{0.5};

    double alt_int{0.0};

    double max_elevator{25.0 * (M_PI / 180.0)};
    double max_aileron{20.0 * (M_PI / 180.0)};
    double max_rudder{20.0 * (M_PI / 180.0)};

    explicit ControlAllocator6DoF(
        double kp_p = 1.2, double kd_p = 0.25, double ki_a = 0.015,
        double kp_r = 0.8, double kd_r = 0.15, double kp_y = 0.5
    );

    void reset();
    ActuatorCommands allocate(double throttle_cmd, const State12& x_state, double h_cmd = 100.0, double chi_cmd = 0.0, double dt = 0.01);
};

} // namespace gnc