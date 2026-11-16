import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CORE_DIR = Path(__file__).resolve().parent.parent
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from control.baseline_pid import BaselinePIDController
from control.control_allocation import ControlAllocator6DoF
from control.mrac_controller import LongitudinalMRAC
from control.optimal_guidance import OptimalGuidance
from control.waypoint_guidance import WaypointGuidance3D
from dynamics.aerodynamics import UAVParameters
from dynamics.atmosphere_wind import AtmosphereWindModel
from dynamics.six_dof_dynamics import SixDoFDynamics
from estimation.attitude_estimator import AttitudeEstimatorAHRS
from estimation.observable_ekf import ObservableEKF
from estimation.sensors import SensorModel


def run_benchmark() -> dict[str, np.ndarray]:
    dt = 0.02
    t_end = 300.0
    time_steps = int(t_end / dt)
    time = np.linspace(0.0, t_end, time_steps)

    route = [(0.0, 0.0, 100.0), (3000.0, 0.0, 100.0), (6000.0, 0.0, 100.0)]

    wind_sim = AtmosphereWindModel(steady_wind=np.array([6.0, 0.0, 0.0]))
    guidance_opt = OptimalGuidance()

    # 1. Baseline Aircraft (Fixed Speed 25 m/s, Non-Adaptive PID)
    params_base = UAVParameters()
    dyn_base = SixDoFDynamics(params_base)
    sensors_base = SensorModel(seed=101)
    alloc_base = ControlAllocator6DoF()
    pid_base = BaselinePIDController(dt=dt)
    wp_base = WaypointGuidance3D(route)

    # 2. Proposed Aircraft (Wind-Aware Optimal Speed, Observable EKF, MRAC)
    params_prop = UAVParameters()
    dyn_prop = SixDoFDynamics(params_prop)
    sensors_prop = SensorModel(seed=101)
    alloc_prop = ControlAllocator6DoF()
    mrac_prop = LongitudinalMRAC(dt=dt)
    ekf_prop = ObservableEKF(dt=dt, m_init=13.5, Vg_init=25.0, w_init=0.0)
    ahrs_prop = AttitudeEstimatorAHRS(dt=dt)
    wp_prop = WaypointGuidance3D(route)

    # Initial 12-state vectors: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    x_base = np.array(
        [0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float
    )
    x_prop = np.array(
        [0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float
    )

    # Telemetry storage
    log = {
        "time": time,
        "base_Va": np.zeros(time_steps),
        "base_energy": np.zeros(time_steps),
        "base_pos_n": np.zeros(time_steps),
        "prop_Va": np.zeros(time_steps),
        "prop_Va_cmd": np.zeros(time_steps),
        "prop_energy": np.zeros(time_steps),
        "prop_pos_n": np.zeros(time_steps),
        "true_mass": np.zeros(time_steps),
        "ekf_mass": np.zeros(time_steps),
        "ekf_mass_sigma": np.zeros(time_steps),
        "legacy_mass": np.zeros(time_steps),
    }

    e_accum_base = 0.0
    e_accum_prop = 0.0
    u_act_base = np.array([0.45, 0.0, 0.0, 0.0], dtype=float)
    u_act_prop = np.array([0.45, 0.0, 0.0, 0.0], dtype=float)

    for k in range(time_steps):
        t = time[k]

        # Payload drop event at t = 150 s
        if abs(t - 150.0) < 0.5 * dt:
            dyn_base.drop_payload()
            dyn_prop.drop_payload()

        w_base = wind_sim.get_total_wind(t=t, altitude_m=-x_base[2], dt=dt)
        w_prop = wind_sim.get_total_wind(t=t, altitude_m=-x_prop[2], dt=dt)

        # Baseline execution
        Va_base = float(
            np.sqrt(
                (x_base[3] - w_base[0]) ** 2
                + (x_base[4] - w_base[1]) ** 2
                + (x_base[5] - w_base[2]) ** 2
            )
        )
        thrust_base = u_act_base[0] * params_base.T_max
        e_accum_base += thrust_base * Va_base * dt

        chi_cmd_base, h_cmd_base, _, _ = wp_base.update(x_base[0], x_base[1], x_base[2])
        thr_base, _ = pid_base.update(Va_base, 25.0, u_trim_base=0.45)
        u_act_base = alloc_base.allocate(thr_base, x_base, h_cmd_base, chi_cmd_base, dt)
        x_base = dyn_base.step_rk4(x_base, u_act_base, w_base, dt)

        # Proposed execution
        Va_prop = float(
            np.sqrt(
                (x_prop[3] - w_prop[0]) ** 2
                + (x_prop[4] - w_prop[1]) ** 2
                + (x_prop[5] - w_prop[2]) ** 2
            )
        )
        thrust_prop = u_act_prop[0] * params_prop.T_max
        e_accum_prop += thrust_prop * Va_prop * dt

        aero_f, _, _, _ = dyn_prop.aero.compute_forces_and_moments(
            max(1.0, Va_prop), 0.0, 0.0, x_prop[9:12], u_act_prop
        )
        meas = sensors_prop.measure(
            x_prop, Va_prop, aero_f, thrust_prop, params_prop.mass
        )

        # Observable EKF estimation
        ekf_prop.predict(thrust_prop)
        ekf_prop.update(
            meas.Va, float(np.linalg.norm(meas.gps_vel[:2])), meas.accel[0], thrust_prop
        )
        est_Vg, est_w, est_m = ekf_prop.get_state()
        _, _, var_m = ekf_prop.get_variances()

        # Wind-Aware Energy-Optimal Guidance
        Va_cmd_prop = guidance_opt.solve_optimal_airspeed(est_m, est_w)

        # MRAC + Control Allocation
        chi_cmd_prop, h_cmd_prop, _, _ = wp_prop.update(x_prop[0], x_prop[1], x_prop[2])
        thr_prop, _, _ = mrac_prop.update(Va_prop, Va_cmd_prop, u_trim_base=0.42)
        u_act_prop = alloc_prop.allocate(thr_prop, x_prop, h_cmd_prop, chi_cmd_prop, dt)
        x_prop = dyn_prop.step_rk4(x_prop, u_act_prop, w_prop, dt)

        # Logging
        log["base_Va"][k] = Va_base
        log["base_energy"][k] = e_accum_base / 1000.0
        log["base_pos_n"][k] = x_base[0]

        log["prop_Va"][k] = Va_prop
        log["prop_Va_cmd"][k] = Va_cmd_prop
        log["prop_energy"][k] = e_accum_prop / 1000.0
        log["prop_pos_n"][k] = x_prop[0]

        log["true_mass"][k] = params_prop.mass
        log["ekf_mass"][k] = est_m
        log["ekf_mass_sigma"][k] = np.sqrt(max(1e-6, var_m))
        log["legacy_mass"][k] = 13.5

    return log


def plot_results(log: dict[str, np.ndarray]) -> None:
    output_dir = CORE_DIR.parent / "output_figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    t = log["time"]

    # Figure 1: EKF Mass Convergence
    plt.figure(figsize=(9, 4.5))
    plt.plot(t, log["true_mass"], "k--", label="True Mass (kg)")
    plt.plot(t, log["legacy_mass"], "r-", label="Legacy EKF (Unobservable)")
    plt.plot(t, log["ekf_mass"], "b-", label="Observable EKF (Proposed)")
    plt.fill_between(
        t,
        log["ekf_mass"] - 3.0 * log["ekf_mass_sigma"],
        log["ekf_mass"] + 3.0 * log["ekf_mass_sigma"],
        color="b",
        alpha=0.15,
        label="±3σ Bounds",
    )
    plt.axvline(150.0, color="gray", linestyle=":", label="Payload Drop (t=150s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "fig1_mass_observability_benchmark.png", dpi=300)
    plt.close()

    # Figure 2: Airspeed Tracking
    plt.figure(figsize=(9, 4.5))
    plt.plot(t, np.full_like(t, 25.0), "k--", label="Baseline Commanded (25 m/s)")
    plt.plot(t, log["base_Va"], "r-", alpha=0.7, label="Baseline Airspeed (Fixed PID)")
    plt.plot(t, log["prop_Va_cmd"], "g--", label="Optimal Commanded Airspeed")
    plt.plot(t, log["prop_Va"], "b-", label="Proposed Airspeed (MRAC)")
    plt.xlabel("Time (s)")
    plt.ylabel("Airspeed (m/s)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "fig2_airspeed_guidance_benchmark.png", dpi=300)
    plt.close()

    # Figure 3: Energy Consumption
    plt.figure(figsize=(9, 4.5))
    plt.plot(t, log["base_energy"], "r-", label="Baseline (Fixed PID)")
    plt.plot(t, log["prop_energy"], "b-", label="Proposed (Optimal Guidance + MRAC)")
    plt.xlabel("Time (s)")
    plt.ylabel("Cumulative Energy (kJ)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "fig3_energy_consumption_benchmark.png", dpi=300)
    plt.close()


if __name__ == "__main__":
    print("Running 6-DoF Dual Comparative Benchmark...")
    results = run_benchmark()
    plot_results(results)

    e_base_final = results["base_energy"][-1]
    e_prop_final = results["prop_energy"][-1]
    saved_pct = ((e_base_final - e_prop_final) / e_base_final) * 100.0

    print("--------------------------------------------------")
    print(f"Baseline Energy:  {e_base_final:.2f} kJ")
    print(f"Proposed Energy:  {e_prop_final:.2f} kJ")
    print(f"Energy Reduction: {saved_pct:.2f} %")
    print("Figures successfully generated in output_figures/")
    print("--------------------------------------------------")
