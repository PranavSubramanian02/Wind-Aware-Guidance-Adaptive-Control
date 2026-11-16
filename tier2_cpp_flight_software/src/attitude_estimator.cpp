#include "attitude_estimator.hpp"
#include <cmath>

namespace gnc {

AttitudeEstimatorAHRS::AttitudeEstimatorAHRS(double dt_step, double kp_a, double ki_g, double kp_gps)
    : dt(dt_step), kp_accel(kp_a), ki_gyro(ki_g), kp_mag_gps(kp_gps) {}

void AttitudeEstimatorAHRS::reset(double phi0, double theta0, double psi0) {
    phi = phi0;
    theta = theta0;
    psi = psi0;
    bias_p = 0.0;
    bias_q = 0.0;
    bias_r = 0.0;
}

Vector3 AttitudeEstimatorAHRS::update(const Vector3& accel, const Vector3& gyro, double chi_gps, double Vg_mag) {
    const double ax = accel[0], ay = accel[1], az = accel[2];
    const double p  = gyro[0],  q  = gyro[1],  r  = gyro[2];

    const double p_corr = p - bias_p;
    const double q_corr = q - bias_q;
    const double r_corr = r - bias_r;

    const double a_norm = std::sqrt(ax * ax + ay * ay + az * az);
    double err_phi = 0.0;
    double err_theta = 0.0;

    if (std::abs(a_norm - g) < 2.5) {
        const double phi_acc = std::atan2(ay, az);
        const double theta_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        err_phi = phi_acc - phi;
        err_theta = theta_acc - theta;
    }

    double err_psi = 0.0;
    if (Vg_mag > 1.5) {
        const double psi_diff = chi_gps - psi;
        err_psi = std::atan2(std::sin(psi_diff), std::cos(psi_diff));
    }

    bias_p -= ki_gyro * err_phi * dt;
    bias_q -= ki_gyro * err_theta * dt;
    bias_r -= ki_gyro * err_psi * dt;

    double c_th = std::cos(theta);
    if (std::abs(c_th) < 1e-4) c_th = (c_th >= 0.0) ? 1e-4 : -1e-4;
    const double t_th = std::sin(theta) / c_th;

    const double phi_dot   = p_corr + (q_corr * std::sin(phi) + r_corr * std::cos(phi)) * t_th + kp_accel * err_phi;
    const double theta_dot = q_corr * std::cos(phi) - r_corr * std::sin(phi) + kp_accel * err_theta;
    const double psi_dot   = (q_corr * std::sin(phi) + r_corr * std::cos(phi)) / c_th + kp_mag_gps * err_psi;

    phi += phi_dot * dt;
    theta += theta_dot * dt;
    psi += psi_dot * dt;

    psi = std::atan2(std::sin(psi), std::cos(psi));

    return Vector3{phi, theta, psi};
}

} // namespace gnc