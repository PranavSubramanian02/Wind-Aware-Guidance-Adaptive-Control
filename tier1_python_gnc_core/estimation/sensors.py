from typing import NamedTuple

import numpy as np


class SensorMeasurements(NamedTuple):
    Va: float
    gps_pos: np.ndarray
    gps_vel: np.ndarray
    accel: np.ndarray
    gyro: np.ndarray


class SensorModel:
    def __init__(self, seed: int = 42) -> None:
        self.rng: np.random.Generator = np.random.default_rng(seed)

        self.sigma_Va: float = 0.5
        self.sigma_gps_pos: float = 1.5
        self.sigma_gps_vel: float = 0.1
        self.sigma_accel: float = 0.15
        self.sigma_gyro: float = 0.005

        self.accel_bias: np.ndarray = np.array([0.05, -0.03, 0.04], dtype=float)
        self.gyro_bias: np.ndarray = np.array([0.001, -0.002, 0.001], dtype=float)

    def measure(
        self,
        x: np.ndarray,
        Va: float,
        aero_forces: np.ndarray,
        thrust: float,
        mass: float,
    ) -> SensorMeasurements:
        assert mass > 0.0
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = x

        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)

        R_b_to_i = np.array(
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

        pos_true = np.array([pn, pe, pd], dtype=float)
        vel_true = R_b_to_i @ np.array([u, v, w], dtype=float)

        noise_gps_p = self.rng.normal(0.0, self.sigma_gps_pos, 3)
        noise_gps_v = self.rng.normal(0.0, self.sigma_gps_vel, 3)
        noise_Va = float(self.rng.normal(0.0, self.sigma_Va))

        f_b_true = (aero_forces + np.array([thrust, 0.0, 0.0], dtype=float)) / mass
        noise_accel = self.rng.normal(0.0, self.sigma_accel, 3)
        noise_gyro = self.rng.normal(0.0, self.sigma_gyro, 3)

        meas_Va = max(0.0, Va + noise_Va)
        meas_pos = pos_true + noise_gps_p
        meas_vel = vel_true + noise_gps_v
        meas_accel = f_b_true + self.accel_bias + noise_accel
        meas_gyro = np.array([p, q, r], dtype=float) + self.gyro_bias + noise_gyro

        return SensorMeasurements(
            Va=meas_Va,
            gps_pos=meas_pos,
            gps_vel=meas_vel,
            accel=meas_accel,
            gyro=meas_gyro,
        )


if __name__ == "__main__":
    sensors = SensorModel(seed=42)
    x_test = np.array(
        [100.0, 50.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.05, 0.1, 0.0, 0.0, 0.0],
        dtype=float,
    )
    aero_forces = np.array([-15.0, 0.0, -130.0], dtype=float)
    thrust = 15.0
    mass = 13.5
    Va = 25.0

    meas = sensors.measure(x_test, Va, aero_forces, thrust, mass)
    assert meas.gps_pos.shape == (3,)
    assert meas.gps_vel.shape == (3,)
    assert meas.accel.shape == (3,)
    assert meas.gyro.shape == (3,)
    assert meas.Va > 0.0
    print("SensorModel Test Passed.")
