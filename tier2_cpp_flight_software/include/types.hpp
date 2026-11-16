#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace gnc {

using Vector3 = std::array<double, 3>;
using Matrix3x3 = std::array<std::array<double, 3>, 3>;
using State12 = std::array<double, 12>;

struct ActuatorCommands {
    double throttle{0.0};  // [0.0, 1.0]
    double elevator{0.0};  // rad
    double aileron{0.0};   // rad
    double rudder{0.0};    // rad
};

struct SensorReadings {
    double Va{0.0};
    Vector3 gps_pos{0.0, 0.0, 0.0};
    Vector3 gps_vel{0.0, 0.0, 0.0};
    Vector3 accel{0.0, 0.0, 0.0};
    Vector3 gyro{0.0, 0.0, 0.0};
};

struct AircraftParameters {
    double mass{13.5};
    double payload_mass{3.0};

    double Jx{0.8244};
    double Jy{1.1350};
    double Jz{1.7590};
    double Jxz{0.1204};

    double Gamma{0.0};
    double Gamma1{0.0};
    double Gamma2{0.0};
    double Gamma3{0.0};
    double Gamma4{0.0};
    double Gamma5{0.0};
    double Gamma6{0.0};
    double Gamma7{0.0};
    double Gamma8{0.0};

    double S{0.55};
    double b{2.89};
    double c{0.1899};

    double CL0{0.28};
    double CD0{0.030};
    double k_induced{0.045};
    double CL_alpha{3.45};
    double CL_q{0.0};
    double CL_delta_e{-0.36};

    double Cm0{-0.020};
    double Cm_alpha{-0.380};
    double Cm_q{-3.600};
    double Cm_delta_e{-0.500};

    double CY0{0.0};
    double CY_beta{-0.98};
    double CY_p{0.0};
    double CY_r{0.0};
    double CY_delta_a{0.0};
    double CY_delta_r{-0.17};

    double Cl0{0.0};
    double Cl_beta{-0.12};
    double Cl_p{-0.26};
    double Cl_r{0.14};
    double Cl_delta_a{0.08};
    double Cl_delta_r{0.105};

    double Cn0{0.0};
    double Cn_beta{0.25};
    double Cn_p{0.022};
    double Cn_r{-0.35};
    double Cn_delta_a{0.06};
    double Cn_delta_r{-0.032};

    double T_max{35.0};
    double rho{1.225};
    double g{9.81};

    void compute_inertias() {
        Gamma = Jx * Jz - Jxz * Jxz;
        Gamma1 = (Jxz * (Jx - Jy + Jz)) / Gamma;
        Gamma2 = (Jz * (Jz - Jy) + Jxz * Jxz) / Gamma;
        Gamma3 = Jz / Gamma;
        Gamma4 = Jxz / Gamma;
        Gamma5 = (Jz - Jx) / Jy;
        Gamma6 = Jxz / Jy;
        Gamma7 = ((Jx - Jy) * Jx + Jxz * Jxz) / Gamma;
        Gamma8 = Jx / Gamma;
    }

    void drop_payload() {
        mass -= payload_mass;
        Jx *= 0.88;
        Jy *= 0.92;
        Jz *= 0.90;
        Jxz *= 0.85;
        compute_inertias();
    }
};

} // namespace gnc