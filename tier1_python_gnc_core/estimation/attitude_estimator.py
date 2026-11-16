from typing import Tuple
import numpy as np


class AttitudeEstimatorAHRS:
    def __init__(
        self,
        dt: float = 0.01,
        kp_accel: float = 1.8,
        ki_gyro: float = 0.05,
        kp_mag_gps: float = 1.2
    ) -> None:
        assert dt > 0.0
        self.dt: float = dt
        self.kp_accel: float = kp_accel
        self.ki_gyro: float = ki_gyro
        self.kp_mag_gps: float = kp_mag_gps

        self.phi: float = 0.0
        self.theta: float = 0.0
        self.psi: float = 0.0

        self.bias_p: float = 0.0
        self.bias_q: float = 0.0
        self.bias_r: float = 0.0

        self.g: float = 9.81

    def reset(self, phi0: float = 0.0, theta0: float = 0.0, psi0: float = 0.0) -> None:
        self.phi = phi0
        self.theta = theta0
        self.psi = psi0
        self.bias_p = 0.0
        self.bias_q = 0.0
        self.bias_r = 0.0

    def update(
        self,
        accel: np.ndarray,
        gyro: np.ndarray,
        chi_gps: float,
        Vg_mag: float
    ) -> Tuple[float, float, float]:
        ax, ay, az = accel
        p, q, r = gyro

        p_corr = p - self.bias_p
        q_corr = q - self.bias_q
        r_corr = r - self.bias_r

        a_norm = float(np.sqrt(ax**2 + ay**2 + az**2))
        err_phi = 0.0
        err_theta = 0.0

        if abs(a_norm - self.g) < 2.5:
            phi_acc = float(np.arctan2(ay, az))
            theta_acc = float(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
            err_phi = phi_acc - self.phi
            err_theta = theta_acc - self.theta

        err_psi = 0.0
        if Vg_mag > 1.5:
            psi_diff = chi_gps - self.psi
            err_psi = float(np.arctan2(np.sin(psi_diff), np.cos(psi_diff)))

        self.bias_p -= self.ki_gyro * err_phi * self.dt
        self.bias_q -= self.ki_gyro * err_theta * self.dt
        self.bias_r -= self.ki_gyro * err_psi * self.dt

        c_th = np.cos(self.theta)
        if abs(c_th) < 1e-4:
            c_th = 1e-4 if c_th >= 0.0 else -1e-4
        t_th = np.sin(self.theta) / c_th

        phi_dot = p_corr + (q_corr * np.sin(self.phi) + r_corr * np.cos(self.phi)) * t_th + self.kp_accel * err_phi
        theta_dot = q_corr * np.cos(self.phi) - r_corr * np.sin(self.phi) + self.kp_accel * err_theta
        psi_dot = (q_corr * np.sin(self.phi) + r_corr * np.cos(self.phi)) / c_th + self.kp_mag_gps * err_psi

        self.phi += phi_dot * self.dt
        self.theta += theta_dot * self.dt
        self.psi += psi_dot * self.dt

        self.psi = float(np.arctan2(np.sin(self.psi), np.cos(self.psi)))

        return self.phi, self.theta, self.psi

    def get_attitude(self) -> Tuple[float, float, float]:
        return self.phi, self.theta, self.psi


if __name__ == "__main__":
    estimator = AttitudeEstimatorAHRS(dt=0.01)

    true_phi = 0.0
    true_theta = np.deg2rad(5.0)
    true_psi = np.deg2rad(45.0)

    g = 9.81
    meas_accel = np.array([-g * np.sin(true_theta), 0.0, g * np.cos(true_theta)], dtype=float)
    meas_gyro = np.array([0.005, -0.003, 0.002], dtype=float)

    for _ in range(500):
        phi_est, theta_est, psi_est = estimator.update(
            accel=meas_accel,
            gyro=meas_gyro,
            chi_gps=true_psi,
            Vg_mag=25.0
        )

    assert abs(np.rad2deg(theta_est) - 5.0) < 0.5
    assert abs(np.rad2deg(psi_est) - 45.0) < 1.0
    print("AttitudeEstimatorAHRS Test Passed. Converged Pitch:", round(np.rad2deg(theta_est), 2), "deg")
