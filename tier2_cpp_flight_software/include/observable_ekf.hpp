#pragma once

#include "types.hpp"
#include <array>

namespace gnc {

class ObservableEKF {
public:
    double dt{0.02};

    // State: [Vg, w, m]^T
    Vector3 x{25.0, 0.0, 13.5};

    Matrix3x3 P{};
    Matrix3x3 Q{};
    Matrix3x3 R{};

    double S{0.55};
    double CD0{0.030};
    double k_induced{0.045};
    double g{9.81};
    double rho{1.225};
    double tau_w{10.0};

    explicit ObservableEKF(double dt_step = 0.02, double m_init = 13.5, double Vg_init = 25.0, double w_init = 0.0);

    void predict(double thrust);
    void update(double y_Va, double y_Vg, double y_ax, double thrust);

    [[nodiscard]] Vector3 get_state() const { return x; }
    [[nodiscard]] Vector3 get_variances() const { return Vector3{P[0][0], P[1][1], P[2][2]}; }

private:
    static Matrix3x3 matrix_multiply(const Matrix3x3& A, const Matrix3x3& B);
    static Matrix3x3 matrix_transpose(const Matrix3x3& A);
    static bool invert_matrix_3x3(const Matrix3x3& A, Matrix3x3& A_inv);
};

} // namespace gnc