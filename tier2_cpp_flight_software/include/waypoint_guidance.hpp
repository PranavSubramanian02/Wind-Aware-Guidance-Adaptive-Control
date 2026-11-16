#pragma once

#include "types.hpp"
#include <cstddef>
#include <vector>

namespace gnc {

struct Waypoint {
    double pn{0.0};
    double pe{0.0};
    double alt{100.0};
};

struct GuidanceOutput {
    double chi_cmd{0.0};
    double h_cmd{100.0};
    double dist_horizontal{0.0};
    bool mission_complete{false};
};

class WaypointGuidance3D {
public:
    std::vector<Waypoint> waypoints;
    double switch_radius{35.0};
    std::size_t current_idx{0};
    bool mission_complete{false};

    explicit WaypointGuidance3D(std::vector<Waypoint> wps, double switch_r = 35.0);

    void reset();
    GuidanceOutput update(double pn, double pe, double pd);
    [[nodiscard]] std::size_t get_active_index() const { return current_idx; }
};

} // namespace gnc