import numpy as np


class LongitudinalMRAC:
    def __init__(
        self,
        dt: float = 0.01,
        tau_m: float = 1.5,
        gamma: float = 0.05,
        sigma_leak: float = 0.001,
        kp_fb: float = 0.12,
        ki_fb: float = 0.02,
    ) -> None:
        assert dt > 0.0
        assert tau_m > 0.0

        self.dt: float = dt
        self.am: float = 1.0 / tau_m
        self.bm: float = self.am

        self.gamma: float = gamma
        self.sigma: float = sigma_leak
        self.kp: float = kp_fb
        self.ki: float = ki_fb

        self.xm: float = 25.0
        self.e_int: float = 0.0

        # Adaptive weights: [bias, Va, Va^2, 1/Va^2]
        self.W_hat: np.ndarray = np.zeros(4, dtype=float)
        self.W_max: float = 10.0

    def regressor(self, Va: float) -> np.ndarray:
        Va_safe = max(10.0, Va)
        return np.array(
            [1.0, Va_safe, 0.01 * Va_safe**2, 100.0 / (Va_safe**2)], dtype=float
        )

    def update(
        self, Va: float, Va_cmd: float, u_trim_base: float = 0.45
    ) -> tuple[float, float, float]:
        # Reference model propagation
        xm_dot = -self.am * self.xm + self.bm * Va_cmd
        self.xm += xm_dot * self.dt

        # Tracking error
        e = Va - self.xm
        self.e_int += e * self.dt
        self.e_int = float(np.clip(self.e_int, -5.0, 5.0))

        # Regressor vector
        phi = self.regressor(Va)

        # Lyapunov adaptation with sigma-modification leakage
        W_dot = self.gamma * (e * phi - self.sigma * self.W_hat)
        self.W_hat += W_dot * self.dt
        self.W_hat = np.clip(self.W_hat, -self.W_max, self.W_max)

        # Adaptive cancelling signal
        u_ad = float(np.dot(self.W_hat, phi))

        # Baseline linear feedback
        u_fb = -self.kp * e - self.ki * self.e_int

        # Combined throttle command
        throttle = u_trim_base + u_fb - u_ad
        throttle_sat = float(np.clip(throttle, 0.0, 1.0))

        return throttle_sat, e, self.xm


if __name__ == "__main__":
    controller = LongitudinalMRAC(dt=0.01)

    Va_sim = 20.0
    Va_target = 27.0
    u_trim = 0.40

    for _ in range(500):
        throttle, err, xm = controller.update(
            Va=Va_sim, Va_cmd=Va_target, u_trim_base=u_trim
        )
        # Simplified plant acceleration to verify closed-loop convergence
        Va_sim += (throttle * 15.0 - 0.02 * Va_sim**2 - 2.0) * 0.01

    assert abs(Va_sim - Va_target) < 1.0
    assert 0.0 <= throttle <= 1.0
    print("LongitudinalMRAC Test Passed. Converged Airspeed:", round(Va_sim, 2), "m/s")
