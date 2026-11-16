import numpy as np


class ObservableEKF:
    def __init__(
        self,
        dt: float = 0.02,
        m_init: float = 13.5,
        Vg_init: float = 25.0,
        w_init: float = 0.0,
    ) -> None:
        self.dt: float = dt

        # State: x = [Vg, w, m]^T
        self.x: np.ndarray = np.array([Vg_init, w_init, m_init], dtype=float)

        self.P: np.ndarray = np.diag([1.0, 4.0, 4.0]).astype(float)
        self.Q: np.ndarray = np.diag([0.05, 0.02, 1e-5]).astype(float) * self.dt
        self.R: np.ndarray = np.diag([0.25, 0.04, 0.0225]).astype(float)

        self.S: float = 0.55
        self.CD0: float = 0.030
        self.k_induced: float = 0.045
        self.g: float = 9.81
        self.rho: float = 1.225
        self.tau_w: float = 10.0

    def predict(self, thrust: float) -> None:
        Vg = self.x[0]
        w = self.x[1]
        m = max(1.0, self.x[2])

        Va = max(1e-4, Vg - w)
        q_inf = 0.5 * self.rho * Va**2
        CL = (2.0 * m * self.g) / (self.rho * self.S * Va**2)
        CD = self.CD0 + self.k_induced * CL**2
        Drag = q_inf * self.S * CD

        f_Vg = (thrust - Drag) / m
        f_w = -w / self.tau_w
        f_m = 0.0

        self.x[0] += f_Vg * self.dt
        self.x[1] += f_w * self.dt
        self.x[2] += f_m * self.dt

        dD_dVa = self.rho * Va * self.S * self.CD0 - (
            4.0 * self.k_induced * (m * self.g) ** 2
        ) / (self.rho * self.S * Va**3)
        dD_dm = (4.0 * self.k_induced * m * self.g**2) / (self.rho * self.S * Va**2)

        df1_dVg = -dD_dVa / m
        df1_dw = dD_dVa / m
        df1_dm = -(thrust - Drag) / (m**2) - (dD_dm / m)

        F = np.eye(3, dtype=float)
        F[0, 0] += df1_dVg * self.dt
        F[0, 1] += df1_dw * self.dt
        F[0, 2] += df1_dm * self.dt
        F[1, 1] += (-1.0 / self.tau_w) * self.dt

        self.P = F @ self.P @ F.T + self.Q

    def update(self, y_Va: float, y_Vg: float, y_ax: float, thrust: float) -> None:
        Vg = self.x[0]
        w = self.x[1]
        m = max(1.0, self.x[2])

        Va = max(1e-4, Vg - w)
        q_inf = 0.5 * self.rho * Va**2
        CL = (2.0 * m * self.g) / (self.rho * self.S * Va**2)
        CD = self.CD0 + self.k_induced * CL**2
        Drag = q_inf * self.S * CD
        ax_pred = (thrust - Drag) / m

        z = np.array([y_Va, y_Vg, y_ax], dtype=float)
        h = np.array([Va, Vg, ax_pred], dtype=float)

        dD_dVa = self.rho * Va * self.S * self.CD0 - (
            4.0 * self.k_induced * (m * self.g) ** 2
        ) / (self.rho * self.S * Va**3)
        dD_dm = (4.0 * self.k_induced * m * self.g**2) / (self.rho * self.S * Va**2)

        dax_dVg = -dD_dVa / m
        dax_dw = dD_dVa / m
        dax_dm = -(thrust - Drag) / (m**2) - (dD_dm / m)

        H = np.array(
            [[1.0, -1.0, 0.0], [1.0, 0.0, 0.0], [dax_dVg, dax_dw, dax_dm]], dtype=float
        )

        y_res = z - h
        S_cov = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S_cov)

        self.x = self.x + K @ y_res
        self.x[2] = max(1.0, self.x[2])
        I_KH = np.eye(3, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    def get_state(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def get_variances(self) -> tuple[float, float, float]:
        return float(self.P[0, 0]), float(self.P[1, 1]), float(self.P[2, 2])


if __name__ == "__main__":
    ekf = ObservableEKF(dt=0.02, m_init=13.5, Vg_init=25.0, w_init=0.0)

    # True states
    true_m = 10.5
    true_w = 5.0
    true_Vg = 25.0
    true_Va = true_Vg - true_w

    thrust = 18.0

    # Simulate convergence over 200 steps (4 seconds)
    for _ in range(200):
        ekf.predict(thrust)

        q_inf = 0.5 * ekf.rho * true_Va**2
        CL = (2.0 * true_m * ekf.g) / (ekf.rho * ekf.S * true_Va**2)
        CD = ekf.CD0 + ekf.k_induced * CL**2
        Drag = q_inf * ekf.S * CD
        true_ax = (thrust - Drag) / true_m

        meas_Va = true_Va + np.random.normal(0.0, 0.5)
        meas_Vg = true_Vg + np.random.normal(0.0, 0.1)
        meas_ax = true_ax + np.random.normal(0.0, 0.15)

        ekf.update(meas_Va, meas_Vg, meas_ax, thrust)

    est_Vg, est_w, est_m = ekf.get_state()
    assert abs(est_m - true_m) < 1.0
    assert abs(est_w - true_w) < 1.0
    print("ObservableEKF Test Passed. Mass converged to:", round(est_m, 3), "kg")
