#include "mrac_controller.hpp"
#include <algorithm>

namespace gnc {

LongitudinalMRAC::LongitudinalMRAC(double dt_step, double tau_m, double gamma_rate, double sigma_leak, double kp_fb, double ki_fb)
    : dt(dt_step), am(1.0 / tau_m), bm(1.0 / tau_m), gamma(gamma_rate), sigma(sigma_leak), kp(kp_fb), ki(ki_fb) {}

void LongitudinalMRAC::reset(double xm_init) {
    xm = xm_init;
    e_int = 0.0;
    last_error = 0.0;
    W_hat.fill(0.0);
}

std::array<double, 4> LongitudinalMRAC::regressor(double Va) const {
    const double Va_safe = std::max(10.0, Va);
    return std::array<double, 4>{
        1.0,
        Va_safe,
        0.01 * Va_safe * Va_safe,
        100.0 / (Va_safe * Va_safe)
    };
}

double LongitudinalMRAC::update(double Va, double Va_cmd, double u_trim_base) {
    const double xm_dot = -am * xm + bm * Va_cmd;
    xm += xm_dot * dt;

    const double e = Va - xm;
    last_error = e;

    e_int += e * dt;
    e_int = std::clamp(e_int, -5.0, 5.0);

    const auto phi = regressor(Va);

    for (std::size_t i = 0; i < 4; ++i) {
        const double W_dot = gamma * (e * phi[i] - sigma * W_hat[i]);
        W_hat[i] += W_dot * dt;
        W_hat[i] = std::clamp(W_hat[i], -W_max, W_max);
    }

    double u_ad = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        u_ad += W_hat[i] * phi[i];
    }

    const double u_fb = -kp * e - ki * e_int;
    const double throttle = u_trim_base + u_fb - u_ad;

    return std::clamp(throttle, 0.0, 1.0);
}

} // namespace gnc