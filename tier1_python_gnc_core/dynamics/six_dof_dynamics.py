import numpy as np

try:
    from tier1_python_gnc_core.dynamics.aerodynamics import (
        AerodynamicsModel,
        UAVParameters,
    )
except ImportError:
    try:
        from dynamics.aerodynamics import AerodynamicsModel, UAVParameters
    except ImportError:
        from aerodynamics import AerodynamicsModel, UAVParameters


class SixDoFDynamics:
    def __init__(self, params: UAVParameters) -> None:
        self.p: UAVParameters = params
        self.aero: AerodynamicsModel = AerodynamicsModel(self.p)

    def rotation_matrix_body_to_inertial(
        self, phi: float, theta: float, psi: float
    ) -> np.ndarray:
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)

        R = np.array(
            [
                [
                    c_th * c_psi,
                    s_phi * s_th * c_psi - c_phi * s_psi,
                    c_phi * s_th * c_psi + s_phi * s_psi,
                ],
                [
                    c_th * s_psi,
                    s_phi * s_th * s_psi + c_phi * c_psi,
                    c_phi * s_th * s_psi - s_phi * c_psi,
                ],
                [-s_th, s_phi * c_th, c_phi * c_th],
            ],
            dtype=float,
        )
        return R

    def rotation_matrix_inertial_to_body(
        self, phi: float, theta: float, psi: float
    ) -> np.ndarray:
        return self.rotation_matrix_body_to_inertial(phi, theta, psi).T

    def derivatives(
        self, x: np.ndarray, u_ctrl: np.ndarray, w_n: np.ndarray, rho: float = 1.225
    ) -> np.ndarray:
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = x
        throttle, delta_e, delta_a, delta_r = u_ctrl
        wn, we, wd = w_n

        R_b_to_i = self.rotation_matrix_body_to_inertial(phi, theta, psi)
        R_i_to_b = R_b_to_i.T

        pos_dot = R_b_to_i @ np.array([u, v, w], dtype=float)

        wind_b = R_i_to_b @ np.array([wn, we, wd], dtype=float)
        ur = u - wind_b[0]
        vr = v - wind_b[1]
        wr = w - wind_b[2]

        Va = float(np.sqrt(ur**2 + vr**2 + wr**2))
        Va = max(Va, 1.0)

        alpha = float(np.arctan2(wr, ur))
        beta = float(np.arcsin(np.clip(vr / Va, -1.0, 1.0)))

        aero_forces, aero_moments, _, _ = self.aero.compute_forces_and_moments(
            Va, alpha, beta, np.array([p, q, r], dtype=float), u_ctrl
        )

        Fx_thrust = float(np.clip(throttle, 0.0, 1.0) * self.p.T_max)
        Fy_thrust = 0.0
        Fz_thrust = 0.0

        m = self.p.mass
        g = self.p.g
        Fx_grav = -m * g * np.sin(theta)
        Fy_grav = m * g * np.sin(phi) * np.cos(theta)
        Fz_grav = m * g * np.cos(phi) * np.cos(theta)

        Fx = aero_forces[0] + Fx_thrust + Fx_grav
        Fy = aero_forces[1] + Fy_thrust + Fy_grav
        Fz = aero_forces[2] + Fz_thrust + Fz_grav

        u_dot = r * v - q * w + Fx / m
        v_dot = p * w - r * u + Fy / m
        w_dot = q * u - p * v + Fz / m

        c_th = np.cos(theta)
        if np.abs(c_th) < 1e-4:
            c_th = 1e-4 if c_th >= 0.0 else -1e-4
        t_th = np.sin(theta) / c_th

        phi_dot = p + (q * np.sin(phi) + r * np.cos(phi)) * t_th
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / c_th

        L_roll = aero_moments[0]
        M_pitch = aero_moments[1]
        N_yaw = aero_moments[2]

        p_dot = (
            self.p.Gamma1 * p * q
            - self.p.Gamma2 * q * r
            + self.p.Gamma3 * L_roll
            + self.p.Gamma4 * N_yaw
        )
        q_dot = (
            self.p.Gamma5 * p * r
            - self.p.Gamma6 * (p**2 - r**2)
            + (1.0 / self.p.Jy) * M_pitch
        )
        r_dot = (
            self.p.Gamma7 * p * q
            - self.p.Gamma1 * q * r
            + self.p.Gamma4 * L_roll
            + self.p.Gamma8 * N_yaw
        )

        return np.array(
            [
                pos_dot[0],
                pos_dot[1],
                pos_dot[2],
                u_dot,
                v_dot,
                w_dot,
                phi_dot,
                theta_dot,
                psi_dot,
                p_dot,
                q_dot,
                r_dot,
            ],
            dtype=float,
        )

    def step_rk4(
        self,
        x: np.ndarray,
        u_ctrl: np.ndarray,
        w_n: np.ndarray,
        dt: float,
        rho: float = 1.225,
    ) -> np.ndarray:
        k1 = self.derivatives(x, u_ctrl, w_n, rho)
        k2 = self.derivatives(x + 0.5 * dt * k1, u_ctrl, w_n, rho)
        k3 = self.derivatives(x + 0.5 * dt * k2, u_ctrl, w_n, rho)
        k4 = self.derivatives(x + dt * k3, u_ctrl, w_n, rho)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def drop_payload(self) -> None:
        self.p.drop_payload()


if __name__ == "__main__":
    params = UAVParameters()
    dynamics = SixDoFDynamics(params)

    # State: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    x0 = np.array(
        [0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float
    )
    u_ctrl = np.array([0.5, 0.0, 0.0, 0.0], dtype=float)
    w_n = np.array([5.0, 0.0, 0.0], dtype=float)

    dt = 0.01
    x_next = dynamics.step_rk4(x0, u_ctrl, w_n, dt)

    assert x_next.shape == (12,)
    assert not np.isnan(x_next).any()
    print("SixDoFDynamics RK4 Step Test Passed.")
