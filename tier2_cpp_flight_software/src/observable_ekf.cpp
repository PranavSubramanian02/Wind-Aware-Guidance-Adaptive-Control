#include "observable_ekf.hpp"
#include <algorithm>
#include <cmath>

namespace gnc {

ObservableEKF::ObservableEKF(double dt_step, double m_init, double Vg_init, double w_init)
    : dt(dt_step), x{Vg_init, w_init, m_init} {
    P = Matrix3x3{{
        {1.0, 0.0, 0.0},
        {0.0, 4.0, 0.0},
        {0.0, 0.0, 4.0}
    }};

    Q = Matrix3x3{{
        {0.05 * dt, 0.0,       0.0},
        {0.0,       0.02 * dt, 0.0},
        {0.0,       0.0,       1e-5 * dt}
    }};

    R = Matrix3x3{{
        {0.25, 0.0,  0.0},
        {0.0,  0.04, 0.0},
        {0.0,  0.0,  0.0225}
    }};
}

Matrix3x3 ObservableEKF::matrix_multiply(const Matrix3x3& A, const Matrix3x3& B) {
    Matrix3x3 C{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
    return C;
}

Matrix3x3 ObservableEKF::matrix_transpose(const Matrix3x3& A) {
    return Matrix3x3{{
        {A[0][0], A[1][0], A[2][0]},
        {A[0][1], A[1][1], A[2][1]},
        {A[0][2], A[1][2], A[2][2]}
    }};
}

bool ObservableEKF::invert_matrix_3x3(const Matrix3x3& A, Matrix3x3& A_inv) {
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                     - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                     + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (std::abs(det) < 1e-12) return false;
    const double inv_det = 1.0 / det;

    A_inv[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    A_inv[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * inv_det;
    A_inv[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;

    A_inv[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * inv_det;
    A_inv[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    A_inv[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * inv_det;

    A_inv[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    A_inv[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * inv_det;
    A_inv[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

    return true;
}

void ObservableEKF::predict(double thrust) {
    const double Vg = x[0];
    const double w = x[1];
    const double m = std::max(1.0, x[2]);

    const double Va = std::max(1e-4, Vg - w);
    const double q_inf = 0.5 * rho * Va * Va;
    const double CL = (2.0 * m * g) / (rho * S * Va * Va);
    const double CD = CD0 + k_induced * CL * CL;
    const double Drag = q_inf * S * CD;

    const double f_Vg = (thrust - Drag) / m;
    const double f_w = -w / tau_w;

    x[0] += f_Vg * dt;
    x[1] += f_w * dt;

    const double dD_dVa = rho * Va * S * CD0 - (4.0 * k_induced * (m * g) * (m * g)) / (rho * S * Va * Va * Va);
    const double dD_dm  = (4.0 * k_induced * m * g * g) / (rho * S * Va * Va);

    const double df1_dVg = -dD_dVa / m;
    const double df1_dw  =  dD_dVa / m;
    const double df1_dm  = -(thrust - Drag) / (m * m) - (dD_dm / m);

    Matrix3x3 F{{
        {1.0 + df1_dVg * dt, df1_dw * dt,           df1_dm * dt},
        {0.0,                1.0 - (1.0 / tau_w) * dt, 0.0},
        {0.0,                0.0,                      1.0}
    }};

    const Matrix3x3 FP = matrix_multiply(F, P);
    const Matrix3x3 FPFT = matrix_multiply(FP, matrix_transpose(F));

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            P[i][j] = FPFT[i][j] + Q[i][j];
        }
    }
}

void ObservableEKF::update(double y_Va, double y_Vg, double y_ax, double thrust) {
    const double Vg = x[0];
    const double w = x[1];
    const double m = std::max(1.0, x[2]);

    const double Va = std::max(1e-4, Vg - w);
    const double q_inf = 0.5 * rho * Va * Va;
    const double CL = (2.0 * m * g) / (rho * S * Va * Va);
    const double CD = CD0 + k_induced * CL * CL;
    const double Drag = q_inf * S * CD;
    const double ax_pred = (thrust - Drag) / m;

    const Vector3 z{y_Va, y_Vg, y_ax};
    const Vector3 h{Va, Vg, ax_pred};
    const Vector3 y_res{z[0] - h[0], z[1] - h[1], z[2] - h[2]};

    const double dD_dVa = rho * Va * S * CD0 - (4.0 * k_induced * (m * g) * (m * g)) / (rho * S * Va * Va * Va);
    const double dD_dm  = (4.0 * k_induced * m * g * g) / (rho * S * Va * Va);

    const double dax_dVg = -dD_dVa / m;
    const double dax_dw  =  dD_dVa / m;
    const double dax_dm  = -(thrust - Drag) / (m * m) - (dD_dm / m);

    const Matrix3x3 H{{
        {1.0,     -1.0,    0.0},
        {1.0,      0.0,    0.0},
        {dax_dVg, dax_dw, dax_dm}
    }};

    const Matrix3x3 H_T = matrix_transpose(H);
    const Matrix3x3 HP = matrix_multiply(H, P);
    const Matrix3x3 S_cov_raw = matrix_multiply(HP, H_T);

    Matrix3x3 S_cov{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            S_cov[i][j] = S_cov_raw[i][j] + R[i][j];
        }
    }

    Matrix3x3 S_inv{};
    if (!invert_matrix_3x3(S_cov, S_inv)) return;

    const Matrix3x3 PHT = matrix_multiply(P, H_T);
    const Matrix3x3 K = matrix_multiply(PHT, S_inv);

    for (std::size_t i = 0; i < 3; ++i) {
        double corr = 0.0;
        for (std::size_t j = 0; j < 3; ++j) {
            corr += K[i][j] * y_res[j];
        }
        x[i] += corr;
    }
    x[2] = std::max(1.0, x[2]);

    const Matrix3x3 KH = matrix_multiply(K, H);
    Matrix3x3 I_KH{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }

    const Matrix3x3 I_KH_P = matrix_multiply(I_KH, P);
    const Matrix3x3 I_KH_P_IKHT = matrix_multiply(I_KH_P, matrix_transpose(I_KH));
    const Matrix3x3 KR = matrix_multiply(K, R);
    const Matrix3x3 KRKT = matrix_multiply(KR, matrix_transpose(K));

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            P[i][j] = I_KH_P_IKHT[i][j] + KRKT[i][j];
        }
    }
}

} // namespace gnc