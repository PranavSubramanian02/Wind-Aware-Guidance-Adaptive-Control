import numpy as np


class UAVParameters:
    def __init__(self) -> None:
        self.m_nominal: float = 13.5
        self.m_payload: float = 3.0
        self.mass: float = self.m_nominal

        self.Jx: float = 0.8244
        self.Jy: float = 1.1350
        self.Jz: float = 1.7590
        self.Jxz: float = 0.1204
        self._update_inertia_constants()

        self.S: float = 0.55
        self.b: float = 2.89
        self.c: float = 0.1899
        self.AR: float = self.b**2 / self.S

        self.CL0: float = 0.28
        self.CD0: float = 0.030
        self.k_induced: float = 0.045
        self.CL_alpha: float = 3.45
        self.CL_q: float = 0.0
        self.CL_delta_e: float = -0.36

        self.Cm0: float = -0.020
        self.Cm_alpha: float = -0.380
        self.Cm_q: float = -3.600
        self.Cm_delta_e: float = -0.500

        self.CY0: float = 0.0
        self.CY_beta: float = -0.98
        self.CY_p: float = 0.0
        self.CY_r: float = 0.0
        self.CY_delta_a: float = 0.0
        self.CY_delta_r: float = -0.17

        self.Cl0: float = 0.0
        self.Cl_beta: float = -0.12
        self.Cl_p: float = -0.26
        self.Cl_r: float = 0.14
        self.Cl_delta_a: float = 0.08
        self.Cl_delta_r: float = 0.105

        self.Cn0: float = 0.0
        self.Cn_beta: float = 0.25
        self.Cn_p: float = 0.022
        self.Cn_r: float = -0.35
        self.Cn_delta_a: float = 0.06
        self.Cn_delta_r: float = -0.032

        self.T_max: float = 35.0
        self.rho: float = 1.225
        self.g: float = 9.81

    def _update_inertia_constants(self) -> None:
        self.Gamma: float = self.Jx * self.Jz - self.Jxz**2
        assert self.Gamma > 0.0
        self.Gamma1: float = (self.Jxz * (self.Jx - self.Jy + self.Jz)) / self.Gamma
        self.Gamma2: float = (self.Jz * (self.Jz - self.Jy) + self.Jxz**2) / self.Gamma
        self.Gamma3: float = self.Jz / self.Gamma
        self.Gamma4: float = self.Jxz / self.Gamma
        self.Gamma5: float = (self.Jz - self.Jx) / self.Jy
        self.Gamma6: float = self.Jxz / self.Jy
        self.Gamma7: float = ((self.Jx - self.Jy) * self.Jx + self.Jxz**2) / self.Gamma
        self.Gamma8: float = self.Jx / self.Gamma

    def release_payload(self) -> None:
        assert self.mass > self.m_payload
        self.mass = self.m_nominal - self.m_payload
        scale = self.mass / self.m_nominal
        self.Jx *= scale
        self.Jy *= scale
        self.Jz *= scale
        self.Jxz *= scale
        self._update_inertia_constants()


class AerodynamicsModel:
    def __init__(self, params: UAVParameters) -> None:
        self.p: UAVParameters = params

    def compute_forces_and_moments(
        self,
        Va: float,
        alpha: float,
        beta: float,
        angular_rates: np.ndarray,
        control_inputs: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        assert Va >= 1.0
        assert angular_rates.shape == (3,)
        assert control_inputs.shape == (4,)

        p, q, r = angular_rates
        delta_t, delta_e, delta_a, delta_r = control_inputs

        delta_e = np.clip(delta_e, np.deg2rad(-25.0), np.deg2rad(25.0))
        delta_a = np.clip(delta_a, np.deg2rad(-20.0), np.deg2rad(20.0))
        delta_r = np.clip(delta_r, np.deg2rad(-25.0), np.deg2rad(25.0))

        q_inf = 0.5 * self.p.rho * (Va**2)

        CL = (
            self.p.CL0
            + self.p.CL_alpha * alpha
            + self.p.CL_q * (self.p.c / (2.0 * Va)) * q
            + self.p.CL_delta_e * delta_e
        )
        CD = self.p.CD0 + self.p.k_induced * (CL**2)

        Lift = q_inf * self.p.S * CL
        Drag = q_inf * self.p.S * CD

        Cm = (
            self.p.Cm0
            + self.p.Cm_alpha * alpha
            + self.p.Cm_q * (self.p.c / (2.0 * Va)) * q
            + self.p.Cm_delta_e * delta_e
        )
        M_pitch = q_inf * self.p.S * self.p.c * Cm

        CY = (
            self.p.CY0
            + self.p.CY_beta * beta
            + self.p.CY_p * (self.p.b / (2.0 * Va)) * p
            + self.p.CY_r * (self.p.b / (2.0 * Va)) * r
            + self.p.CY_delta_a * delta_a
            + self.p.CY_delta_r * delta_r
        )
        Fy_aero = q_inf * self.p.S * CY

        Cl = (
            self.p.Cl0
            + self.p.Cl_beta * beta
            + self.p.Cl_p * (self.p.b / (2.0 * Va)) * p
            + self.p.Cl_r * (self.p.b / (2.0 * Va)) * r
            + self.p.Cl_delta_a * delta_a
            + self.p.Cl_delta_r * delta_r
        )
        L_roll = q_inf * self.p.S * self.p.b * Cl

        Cn = (
            self.p.Cn0
            + self.p.Cn_beta * beta
            + self.p.Cn_p * (self.p.b / (2.0 * Va)) * p
            + self.p.Cn_r * (self.p.b / (2.0 * Va)) * r
            + self.p.Cn_delta_a * delta_a
            + self.p.Cn_delta_r * delta_r
        )
        N_yaw = q_inf * self.p.S * self.p.b * Cn

        Fx_aero = -Drag * np.cos(alpha) + Lift * np.sin(alpha)
        Fz_aero = -Drag * np.sin(alpha) - Lift * np.cos(alpha)

        aero_forces = np.array([Fx_aero, Fy_aero, Fz_aero], dtype=float)
        aero_moments = np.array([L_roll, M_pitch, N_yaw], dtype=float)

        return aero_forces, aero_moments, Lift, Drag


if __name__ == "__main__":
    params = UAVParameters()
    aero = AerodynamicsModel(params)
    forces, moments, L, D = aero.compute_forces_and_moments(
        Va=18.0,
        alpha=np.deg2rad(2.0),
        beta=0.0,
        angular_rates=np.zeros(3),
        control_inputs=np.array([0.38, np.deg2rad(-3.0), 0.0, 0.0]),
    )
    assert abs(L - params.mass * params.g) < 25.0
    print("PASS: aerodynamics.py verified")
