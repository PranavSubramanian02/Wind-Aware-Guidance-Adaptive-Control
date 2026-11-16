#pragma once

#include "types.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace gnc {

class SixDoFPhysics {
public:
    AircraftParameters params;

    explicit SixDoFPhysics(const AircraftParameters& p = AircraftParameters{}) : params(p) {
        params.compute_inertias();
    }

    static Matrix3x3 rotation_body_to_inertial(double phi, double theta, double psi) {
        const double c_phi = std::cos(phi), s_phi = std::sin(phi);
        const double c_th  = std::cos(theta), s_th  = std::sin(theta);
        const double c_psi = std::cos(psi), s_psi = std::sin(psi);

        return Matrix3x3{{
            {c_th * c_psi, s_phi * s_th * c_psi - c_phi * s_psi, c_phi * s_th * c_psi + s_phi * s_psi},
            {c_th * s_psi, s_phi * s_th * s_psi + c_phi * c_psi, c_phi * s_th * s_psi - s_phi * c_psi},
            {-s_th,        s_phi * c_th,                        c_phi * c_th}
        }};
    }

    State12 derivatives(const State12& x, const ActuatorCommands& u_ctrl, const Vector3& w_n, double rho = 1.225) const {
        const double pn = x[0], pe = x[1], pd = x[2];
        const double u = x[3], v = x[4], w = x[5];
        const double phi = x[6], theta = x[7], psi = x[8];
        const double p = x[9], q = x[10], r = x[11];

        const Matrix3x3 R_b_to_i = rotation_body_to_inertial(phi, theta, psi);

        // Translational kinematics: p_dot = R_b_to_i * [u, v, w]^T
        const double pn_dot = R_b_to_i[0][0] * u + R_b_to_i[0][1] * v + R_b_to_i[0][2] * w;
        const double pe_dot = R_b_to_i[1][0] * u + R_b_to_i[1][1] * v + R_b_to_i[1][2] * w;
        const double pd_dot = R_b_to_i[2][0] * u + R_b_to_i[2][1] * v + R_b_to_i[2][2] * w;

        // Relative wind in body frame: wind_b = R_i_to_b * w_n = R_b_to_i^T * w_n
        const double w_bx = R_b_to_i[0][0] * w_n[0] + R_b_to_i[1][0] * w_n[1] + R_b_to_i[2][0] * w_n[2];
        const double w_by = R_b_to_i[0][1] * w_n[0] + R_b_to_i[1][1] * w_n[1] + R_b_to_i[2][1] * w_n[2];
        const double w_bz = R_b_to_i[0][2] * w_n[0] + R_b_to_i[1][2] * w_n[1] + R_b_to_i[2][2] * w_n[2];

        const double ur = u - w_bx;
        const double vr = v - w_by;
        const double wr = w - w_bz;

        double Va = std::sqrt(ur * ur + vr * vr + wr * wr);
        if (Va < 1e-4) Va = 1e-4;

        const double alpha = std::atan2(wr, ur);
        const double beta  = std::asin(std::clamp(vr / Va, -1.0, 1.0));

        // Aerodynamics
        const double q_inf = 0.5 * rho * Va * Va;
        const double CL = params.CL0 + params.CL_alpha * alpha + params.CL_q * (params.c / (2.0 * Va)) * q + params.CL_delta_e * u_ctrl.elevator;
        const double CD = params.CD0 + params.k_induced * CL * CL;

        const double Lift = q_inf * params.S * CL;
        const double Drag = q_inf * params.S * CD;

        const double Fx_aero = -Drag * std::cos(alpha) + Lift * std::sin(alpha);
        const double Fy_aero = q_inf * params.S * (params.CY0 + params.CY_beta * beta + params.CY_p * (params.b / (2.0 * Va)) * p + params.CY_r * (params.b / (2.0 * Va)) * r + params.CY_delta_a * u_ctrl.aileron + params.CY_delta_r * u_ctrl.rudder);
        const double Fz_aero = -Drag * std::sin(alpha) - Lift * std::cos(alpha);

        const double L_roll  = q_inf * params.S * params.b * (params.Cl0 + params.Cl_beta * beta + params.Cl_p * (params.b / (2.0 * Va)) * p + params.Cl_r * (params.b / (2.0 * Va)) * r + params.Cl_delta_a * u_ctrl.aileron + params.Cl_delta_r * u_ctrl.rudder);
        const double M_pitch = q_inf * params.S * params.c * (params.Cm0 + params.Cm_alpha * alpha + params.Cm_q * (params.c / (2.0 * Va)) * q + params.Cm_delta_e * u_ctrl.elevator);
        const double N_yaw   = q_inf * params.S * params.b * (params.Cn0 + params.Cn_beta * beta + params.Cn_p * (params.b / (2.0 * Va)) * p + params.Cn_r * (params.b / (2.0 * Va)) * r + params.Cn_delta_a * u_ctrl.aileron + params.Cn_delta_r * u_ctrl.rudder);

        // Forces
        const double thrust = std::clamp(u_ctrl.throttle, 0.0, 1.0) * params.T_max;
        const double m = params.mass;
        const double g = params.g;

        const double Fx_grav = -m * g * std::sin(theta);
        const double Fy_grav =  m * g * std::sin(phi) * std::cos(theta);
        const double Fz_grav =  m * g * std::cos(phi) * std::cos(theta);

        const double Fx = Fx_aero + thrust + Fx_grav;
        const double Fy = Fy_aero + Fy_grav;
        const double Fz = Fz_aero + Fz_grav;

        // Translational dynamics
        const double u_dot = r * v - q * w + Fx / m;
        const double v_dot = p * w - r * u + Fy / m;
        const double w_dot = q * u - p * v + Fz / m;

        // Rotational kinematics
        double c_th = std::cos(theta);
        if (std::abs(c_th) < 1e-4) c_th = (c_th >= 0.0) ? 1e-4 : -1e-4;
        const double t_th = std::sin(theta) / c_th;

        const double phi_dot   = p + (q * std::sin(phi) + r * std::cos(phi)) * t_th;
        const double theta_dot = q * std::cos(phi) - r * std::sin(phi);
        const double psi_dot   = (q * std::sin(phi) + r * std::cos(phi)) / c_th;

        // Rotational dynamics
        const double p_dot = params.Gamma1 * p * q - params.Gamma2 * q * r + params.Gamma3 * L_roll + params.Gamma4 * N_yaw;
        const double q_dot = params.Gamma5 * p * r - params.Gamma6 * (p * p - r * r) + (1.0 / params.Jy) * M_pitch;
        const double r_dot = params.Gamma7 * p * q - params.Gamma1 * q * r + params.Gamma4 * L_roll + params.Gamma8 * N_yaw;

        return State12{pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot};
    }

    State12 step_rk4(const State12& x, const ActuatorCommands& u, const Vector3& w_n, double dt, double rho = 1.225) const {
        const State12 k1 = derivatives(x, u, w_n, rho);

        State12 x_k2{};
        for (std::size_t i = 0; i < 12; ++i) x_k2[i] = x[i] + 0.5 * dt * k1[i];
        const State12 k2 = derivatives(x_k2, u, w_n, rho);

        State12 x_k3{};
        for (std::size_t i = 0; i < 12; ++i) x_k3[i] = x[i] + 0.5 * dt * k2[i];
        const State12 k3 = derivatives(x_k3, u, w_n, rho);

        State12 x_k4{};
        for (std::size_t i = 0; i < 12; ++i) x_k4[i] = x[i] + dt * k3[i];
        const State12 k4 = derivatives(x_k4, u, w_n, rho);

        State12 x_next{};
        for (std::size_t i = 0; i < 12; ++i) {
            x_next[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        return x_next;
    }
};

} // namespace gnc