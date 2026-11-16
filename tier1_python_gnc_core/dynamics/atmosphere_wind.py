import numpy as np


class AtmosphereWindModel:
    def __init__(
        self,
        rho0: float = 1.225,
        H_scale: float = 8500.0,
        steady_wind: np.ndarray = np.zeros(3),
        gust_magnitude: float = 2.0,
        gust_start_time: float = 120.0,
    ) -> None:
        assert steady_wind.shape == (3,)
        self.rho0: float = rho0
        self.H_scale: float = H_scale
        self.w_steady: np.ndarray = steady_wind.astype(float)
        self.gust_magnitude: float = gust_magnitude
        self.gust_start_time: float = gust_start_time

        # Dryden turbulence parameters (low-altitude moderate)
        self.sigma_u: float = 1.06
        self.Lu: float = 200.0
        self.w_turb: np.ndarray = np.zeros(3)

    def get_density(self, altitude_m: float) -> float:
        h = max(altitude_m, 0.0)
        return float(self.rho0 * np.exp(-h / self.H_scale))

    def update_dryden_turbulence(
        self, Va: float, dt: float, random_seed: int = None
    ) -> np.ndarray:
        if random_seed is not None:
            np.random.seed(random_seed)

        Va = max(Va, 1.0)
        # First-order Gauss-Markov approximation of Dryden gust
        alpha = np.exp(-Va * dt / self.Lu)
        noise = np.random.randn(3)
        std_dev = self.sigma_u * np.sqrt(1.0 - alpha**2)
        self.w_turb = alpha * self.w_turb + std_dev * noise
        return self.w_turb

    def get_total_wind(
        self, t: float, altitude_m: float = 0.0, Va: float = 18.0, dt: float = 0.05
    ) -> np.ndarray:
        # Base sinusoidal variation + step gust along North (inertial X)
        w_dynamic_n = 3.0 * np.sin(0.015 * t)
        if t >= self.gust_start_time:
            w_dynamic_n += self.gust_magnitude

        turb = self.update_dryden_turbulence(Va, dt)

        total_wind = np.array(
            [
                self.w_steady[0] + w_dynamic_n + turb[0],
                self.w_steady[1] + turb[1],
                self.w_steady[2] + turb[2],
            ],
            dtype=float,
        )
        return total_wind


if __name__ == "__main__":
    env = AtmosphereWindModel(steady_wind=np.array([2.0, 0.0, 0.0]))
    rho = env.get_density(100.0)
    assert 1.0 < rho < 1.25

    w_t0 = env.get_total_wind(t=0.0, altitude_m=100.0, Va=18.0)
    w_t150 = env.get_total_wind(t=150.0, altitude_m=100.0, Va=18.0)
    assert w_t0.shape == (3,)
    assert w_t150.shape == (3,)
    print("PASS: atmosphere_wind.py verified")
