import numpy as np


class OptimalGuidance:
    def __init__(
        self,
        S: float = 0.55,
        CD0: float = 0.030,
        k_induced: float = 0.045,
        CL_max: float = 1.4,
        rho: float = 1.225,
        g: float = 9.81,
        Va_max: float = 35.0,
    ) -> None:
        self.S: float = S
        self.CD0: float = CD0
        self.k: float = k_induced
        self.CL_max: float = CL_max
        self.rho: float = rho
        self.g: float = g
        self.Va_max: float = Va_max

    def compute_stall_speed(self, mass: float) -> float:
        assert mass > 0.0
        return float(np.sqrt((2.0 * mass * self.g) / (self.rho * self.S * self.CL_max)))

    def solve_optimal_airspeed(self, mass: float, w_head: float) -> float:
        assert mass > 0.0
        V_stall = self.compute_stall_speed(mass)
        Va_min = max(V_stall * 1.2, 12.0)

        A = 0.5 * self.rho * self.S * self.CD0
        B = (2.0 * self.k * (mass * self.g) ** 2) / (self.rho * self.S)

        # Minimum drag airspeed (zero wind) as initial guess
        Va = (B / A) ** 0.25
        if w_head > 0.0:
            Va += 0.5 * w_head

        # Newton-Raphson iteration
        for _ in range(20):
            Va = np.clip(Va, Va_min, self.Va_max)
            f = (
                2.0 * A * Va**3
                - 3.0 * A * w_head * Va**2
                - (2.0 * B) / Va
                + (B * w_head) / (Va**2)
            )
            f_prime = (
                6.0 * A * Va**2
                - 6.0 * A * w_head * Va
                + (2.0 * B) / (Va**2)
                - (2.0 * B * w_head) / (Va**3)
            )

            if abs(f_prime) < 1e-6:
                break

            dVa = f / f_prime
            Va -= dVa
            if abs(dVa) < 1e-4:
                break

        return float(np.clip(Va, Va_min, self.Va_max))

    def compute_energy_rate(
        self, Va: float, mass: float, w_head: float
    ) -> tuple[float, float, float]:
        assert mass > 0.0
        Vg = max(0.1, Va - w_head)
        q_inf = 0.5 * self.rho * Va**2
        CL = (2.0 * mass * self.g) / (self.rho * self.S * Va**2)
        CD = self.CD0 + self.k * CL**2
        Drag = q_inf * self.S * CD
        Power = Drag * Va
        Energy_per_meter = Power / Vg
        return Drag, Power, Energy_per_meter


if __name__ == "__main__":
    guidance = OptimalGuidance()

    m_heavy = 13.5
    m_light = 10.5
    w_headwind = 6.0

    Va_opt_heavy = guidance.solve_optimal_airspeed(m_heavy, w_headwind)
    Va_opt_light = guidance.solve_optimal_airspeed(m_light, w_headwind)

    _, _, E_heavy = guidance.compute_energy_rate(Va_opt_heavy, m_heavy, w_headwind)
    _, _, E_light = guidance.compute_energy_rate(Va_opt_light, m_light, w_headwind)

    assert Va_opt_heavy > Va_opt_light
    assert E_heavy > E_light
    print("OptimalGuidance Test Passed.")
    print(
        f"Optimal Airspeed (Heavy {m_heavy}kg): {Va_opt_heavy:.2f} m/s | Energy: {E_heavy:.2f} J/m"
    )
    print(
        f"Optimal Airspeed (Light {m_light}kg): {Va_opt_light:.2f} m/s | Energy: {E_light:.2f} J/m"
    )
