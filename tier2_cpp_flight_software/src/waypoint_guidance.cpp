#include "waypoint_guidance.hpp"
#include <cmath>
#include <utility>

namespace gnc {

WaypointGuidance3D::WaypointGuidance3D(std::vector<Waypoint> wps, double switch_r)
    : waypoints(std::move(wps)), switch_radius(switch_r) {}

void WaypointGuidance3D::reset() {
    current_idx = 0;
    mission_complete = false;
}

GuidanceOutput WaypointGuidance3D::update(double pn, double pe, double pd) {
    if (waypoints.empty()) {
        return GuidanceOutput{0.0, 100.0, 0.0, true};
    }

    auto target = waypoints[current_idx];
    double delta_n = target.pn - pn;
    double delta_e = target.pe - pe;
    double dist_horizontal = std::sqrt(delta_n * delta_n + delta_e * delta_e);

    if (dist_horizontal < switch_radius) {
        if (current_idx < waypoints.size() - 1) {
            ++current_idx;
            target = waypoints[current_idx];
            delta_n = target.pn - pn;
            delta_e = target.pe - pe;
            dist_horizontal = std::sqrt(delta_n * delta_n + delta_e * delta_e);
        } else {
            mission_complete = true;
        }
    }

    const double chi_cmd = std::atan2(delta_e, delta_n);
    const double h_cmd = target.alt;

    return GuidanceOutput{chi_cmd, h_cmd, dist_horizontal, mission_complete};
}

} // namespace gnc