import numpy as np


class WaypointGuidance3D:
    def __init__(
        self, waypoints: list[tuple[float, float, float]], switch_radius: float = 35.0
    ) -> None:
        assert len(waypoints) >= 2
        assert switch_radius > 0.0

        # Waypoints: [(pn, pe, altitude)]
        self.waypoints: list[tuple[float, float, float]] = waypoints
        self.switch_radius: float = switch_radius
        self.current_idx: int = 0
        self.mission_complete: bool = False

    def reset(self) -> None:
        self.current_idx = 0
        self.mission_complete = False

    def update(
        self, pn: float, pe: float, pd: float
    ) -> tuple[float, float, float, bool]:
        current_alt = -pd
        target_pn, target_pe, target_alt = self.waypoints[self.current_idx]

        delta_n = target_pn - pn
        delta_e = target_pe - pe
        dist_horizontal = float(np.sqrt(delta_n**2 + delta_e**2))

        if dist_horizontal < self.switch_radius:
            if self.current_idx < len(self.waypoints) - 1:
                self.current_idx += 1
                target_pn, target_pe, target_alt = self.waypoints[self.current_idx]
                delta_n = target_pn - pn
                delta_e = target_pe - pe
                dist_horizontal = float(np.sqrt(delta_n**2 + delta_e**2))
            else:
                self.mission_complete = True

        chi_cmd = float(np.arctan2(delta_e, delta_n))
        h_cmd = float(target_alt)

        return chi_cmd, h_cmd, dist_horizontal, self.mission_complete


if __name__ == "__main__":
    route = [(0.0, 0.0, 100.0), (500.0, 0.0, 120.0), (1000.0, 500.0, 100.0)]
    guidance = WaypointGuidance3D(route, switch_radius=25.0)

    # Initial position
    chi, h, dist, done = guidance.update(pn=10.0, pe=0.0, pd=-100.0)
    assert not done
    assert abs(chi - 0.0) < 1e-3
    assert abs(h - 100.0) < 1e-3

    # Position near second waypoint
    chi, h, dist, done = guidance.update(pn=495.0, pe=0.0, pd=-120.0)
    assert guidance.current_idx == 1
    print(
        "WaypointGuidance3D Test Passed. Active waypoint index:", guidance.current_idx
    )
