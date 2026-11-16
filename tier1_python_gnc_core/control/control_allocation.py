import numpy as np


class ControlAllocator6DoF:
    def __init__(
        self,
        kp_pitch: float = 1.2,
        kd_pitch: float = 0.25,
        ki_alt: float = 0.015,
        kp_roll: float = 0.8,
        kd_roll: float = 0.15,
        kp_yaw: float = 0.5,
    ) -> None:
        self.kp_pitch: float = kp_pitch
        self.kd_pitch: float = kd_pitch
        self.ki_alt: float = ki_alt

        self.kp_roll: float = kp_roll
        self.kd_roll: float = kd_roll
        self.kp_yaw: float = kp_yaw

        self.alt_int: float = 0.0

        # Actuator saturation limits
        self.max_elevator: float = np.deg2rad(25.0)
        self.max_aileron: float = np.deg2rad(20.0)
        self.max_rudder: float = np.deg2rad(20.0)

    def allocate(
        self,
        throttle_cmd: float,
        x_state: np.ndarray,
        h_cmd: float = 100.0,
        chi_cmd: float = 0.0,
        dt: float = 0.01,
    ) -> np.ndarray:
        assert dt > 0.0
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = x_state
        h_current = -pd

        # Longitudinal Pitch & Altitude Loop -> Elevator
        h_err = h_cmd - h_current
        self.alt_int += h_err * dt
        self.alt_int = float(np.clip(self.alt_int, -20.0, 20.0))

        theta_cmd = 0.03 * h_err + self.ki_alt * self.alt_int
        theta_cmd = float(np.clip(theta_cmd, np.deg2rad(-15.0), np.deg2rad(20.0)))

        delta_e = -(self.kp_pitch * (theta_cmd - theta) - self.kd_pitch * q)
        delta_e = float(np.clip(delta_e, -self.max_elevator, self.max_elevator))

        # Lateral Roll & Heading Loop -> Aileron
        psi_err = chi_cmd - psi
        psi_err = float(np.arctan2(np.sin(psi_err), np.cos(psi_err)))

        phi_cmd = float(np.clip(0.6 * psi_err, np.deg2rad(-30.0), np.deg2rad(30.0)))
        delta_a = self.kp_roll * (phi_cmd - phi) - self.kd_roll * p
        delta_a = float(np.clip(delta_a, -self.max_aileron, self.max_aileron))

        # Directional Damper (Yaw Rate / Sideslip suppression) -> Rudder
        delta_r = -self.kp_yaw * r
        delta_r = float(np.clip(delta_r, -self.max_rudder, self.max_rudder))

        throttle = float(np.clip(throttle_cmd, 0.0, 1.0))

        return np.array([throttle, delta_e, delta_a, delta_r], dtype=float)


if __name__ == "__main__":
    allocator = ControlAllocator6DoF()
    # State: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    state = np.array(
        [0.0, 0.0, -95.0, 25.0, 0.0, 0.0, 0.02, 0.01, 0.0, 0.0, 0.0, 0.0], dtype=float
    )

    u_actuators = allocator.allocate(
        throttle_cmd=0.55, x_state=state, h_cmd=100.0, chi_cmd=0.0, dt=0.01
    )

    assert u_actuators.shape == (4,)
    assert 0.0 <= u_actuators[0] <= 1.0
    print("ControlAllocator6DoF Test Passed.")
    print(
        "Actuator vector [throttle, delta_e, delta_a, delta_r]:",
        np.round(u_actuators, 4),
    )
