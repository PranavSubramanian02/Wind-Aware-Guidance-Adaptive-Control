#pragma once

#include "types.hpp"
#include <array>

namespace gnc {

class LongitudinalMRAC {
public:
    double dt{0.01};
    double am{1.0 / 1.5};
    double bm{1.0 / 1.5};

    double gamma{0.05};
    double sigma{0.001};
    double kp{0.12};
    double ki{0.02};

    double xm{25.0};
    double e_int{0.0};
    double last_error{0.0};

    std::array<double, 4> W_hat{0.0, 0.0, 0.0, 0.0};
    double W_max{10.0};

    explicit LongitudinalMRAC(double dt_step = 0.01, double tau_m = 1.5, double gamma_rate = 0.05, double sigma_leak = 0.001, double kp_fb = 0.12, double ki_fb = 0.02);

    void reset(double xm_init = 25.0);
    [[nodiscard]] std::array<double, 4> regressor(double Va) const;
    double update(double Va, double Va_cmd, double u_trim_base = 0.45);

    [[nodiscard]] double get_tracking_error() const { return last_error; }
    [[nodiscard]] double get_reference_state() const { return xm; }
    [[nodiscard]] std::array<double, 4> get_weights() const { return W_hat; }
};

} // namespace gnc