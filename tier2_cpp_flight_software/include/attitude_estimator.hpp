#pragma once

#include "types.hpp"

namespace gnc {

class AttitudeEstimatorAHRS {
public:
    double dt{0.01};
    double kp_accel{1.8};
    double ki_gyro{0.05};
    double kp_mag_gps{1.2};
    double g{9.81};

    double phi{0.0};
    double theta{0.0};
    double psi{0.0};

    double bias_p{0.0};
    double bias_q{0.0};
    double bias_r{0.0};

    explicit AttitudeEstimatorAHRS(double dt_step = 0.01, double kp_a = 1.8, double ki_g = 0.05, double kp_gps = 1.2);

    void reset(double phi0 = 0.0, double theta0 = 0.0, double psi0 = 0.0);
    Vector3 update(const Vector3& accel, const Vector3& gyro, double chi_gps, double Vg_mag);

    [[nodiscard]] Vector3 get_attitude() const { return Vector3{phi, theta, psi}; }
};

} // namespace gnc