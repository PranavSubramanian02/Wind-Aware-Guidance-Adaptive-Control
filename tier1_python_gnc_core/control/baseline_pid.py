import numpy as np


class BaselinePIDController:
    def __init__(
        self,
        kp_va: float = 0.12,
        ki_va: float = 0.02,
        kd_va: float = 0.01,
        dt: float = 0.01,
    ) -> None:
        assert dt > 0.0
        self.kp: float = kp_va
        self.ki: float = ki_va
        self.kd: float = kd_va
        self.dt: float = dt

        self.integral_error: float = 0.0
        self.prev_error: float = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0
        self.prev_error = 0.0

    def update(
        self, Va_meas: float, Va_cmd: float, u_trim_base: float = 0.45
    ) -> tuple[float, float]:
        error = Va_cmd - Va_meas

        self.integral_error += error * self.dt
        self.integral_error = float(np.clip(self.integral_error, -5.0, 5.0))

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u_feedback = (
            self.kp * error + self.ki * self.integral_error + self.kd * derivative
        )
        throttle = u_trim_base + u_feedback
        throttle_sat = float(np.clip(throttle, 0.0, 1.0))

        return throttle_sat, error


if __name__ == "__main__":
    controller = BaselinePIDController(dt=0.01)

    Va_sim = 20.0
    Va_target = 25.0
    u_trim = 0.40

    for _ in range(500):
        throttle, err = controller.update(
            Va_meas=Va_sim, Va_cmd=Va_target, u_trim_base=u_trim
        )
        # Simplified plant response
        Va_sim += (throttle * 15.0 - 0.02 * Va_sim**2 - 2.0) * 0.01

    assert abs(Va_sim - Va_target) < 1.0
    assert 0.0 <= throttle <= 1.0
    print("BaselinePIDController Test Passed. Final Airspeed:", round(Va_sim, 2), "m/s")
