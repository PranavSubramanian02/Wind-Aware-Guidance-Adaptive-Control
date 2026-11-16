"""
===========================================================================
UAV AUTONOMOUS FLIGHT UNDER ENVIRONMENTAL UNCERTAINTY
Wind-Aware | Energy-Optimal | Adaptive Control | EKF Estimation

As-Is Python Migration of MATLAB Live Script (project1.mlx)
Author of Original Model: Pranav Subramanian (M.Tech)
Target Venue: ICAAV-2026 (ADE-DRDO & AeSI)

References:
1. Beard & McLain – Small Unmanned Aircraft (Theory & Practice)
2. Langelaan – Wind Estimation for UAVs (AIAA JGCD)
3. Stevens & Lewis – Aircraft Control & Simulation
4. Bryson & Ho – Applied Optimal Control
5. Ioannou & Sun – Robust Adaptive Control
===========================================================================
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def run_simulation(show_plots=False, save_dir="output_figures"):
    # Reproducibility (equivalent to rng(1))
    np.random.seed(1)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # =========================================================================
    # SIMULATION SETUP
    # =========================================================================
    dt = 0.05  # Time step (s)
    T = 300.0  # Total simulation time (s)
    t = np.arange(0, T + dt, dt)  # Time vector
    N = len(t)  # Number of time steps

    rho = 1.225  # Air density (kg/m^3)

    # =========================================================================
    # UAV PARAMETERS (Fixed-Wing UAV – Beard & McLain Class)
    # =========================================================================
    m_nominal = 13.5  # Nominal mass (kg)
    payload = 3.0  # Payload mass (kg)

    S = 0.55  # Wing area (m^2)
    CD0 = 0.028  # Zero-lift drag coefficient
    k_drag = 0.045  # Induced drag factor (k)

    Va_star = 18.0  # Minimum power airspeed (m/s)
    Va_min = 14.0  # Minimum allowable airspeed (m/s)
    Va_max = 28.0  # Maximum allowable airspeed (m/s)

    # =========================================================================
    # ENVIRONMENT (UNKNOWN TO AUTONOMY SYSTEM)
    # =========================================================================
    # True wind profile (positive = headwind)
    wind_true = 3.0 * np.sin(0.015 * t) + 2.0 * (t > 120.0).astype(float)

    # Payload release event
    payload_released = t >= 150.0

    # =========================================================================
    # TRUE VEHICLE STATES (GROUND TRUTH)
    # =========================================================================
    x = np.zeros(N)  # Longitudinal position (m)
    y = np.zeros(N)  # Lateral deviation (m)
    Vg = np.zeros(N)  # Ground speed (m/s)
    Va = np.zeros(N)  # Airspeed (m/s)

    Vg[0] = Va_star  # Initial trim condition

    # =========================================================================
    # EXTENDED KALMAN FILTER (EKF) INITIALIZATION
    # States: [ Ground Speed ; Wind Speed ; Mass Bias ]
    # =========================================================================
    xhat = np.array([[Va_star], [0.0], [0.0]])  # Initial state estimate (3x1)
    P = np.diag([4.0, 4.0, 1.0])  # Initial covariance (3x3)

    Q = np.diag([0.1, 0.01, 0.001])  # Process noise covariance
    R = 2.0  # Airspeed measurement noise variance

    wind_est = np.zeros(N)  # Estimated wind
    mass_est = np.zeros(N)  # Estimated mass

    # =========================================================================
    # DATA LOGS FOR ANALYSIS
    # =========================================================================
    Va_cmd_log = np.zeros(N)

    Thrust_fixed = np.zeros(N)
    Thrust_adapt = np.zeros(N)

    Power_fixed = np.zeros(N)
    Power_adapt = np.zeros(N)

    Energy_fixed = np.zeros(N)
    Energy_adapt = np.zeros(N)

    # =========================================================================
    # MAIN AUTONOMOUS FLIGHT SIMULATION LOOP
    # =========================================================================
    for k in range(N - 1):
        # ---------------------------------------------------------------------
        # TRUE MASS (Payload Drop)
        # ---------------------------------------------------------------------
        if payload_released[k]:
            m_true = m_nominal - payload
        else:
            m_true = m_nominal

        # ---------------------------------------------------------------------
        # TRUE AIRSPEED (Wind-Relative)
        # ---------------------------------------------------------------------
        Va[k] = Vg[k] - wind_true[k]

        # ---------------------------------------------------------------------
        # EKF PREDICTION STEP
        # ---------------------------------------------------------------------
        xhat = xhat.copy()
        P = P + Q

        # ---------------------------------------------------------------------
        # EKF UPDATE STEP (Airspeed Measurement)
        # ---------------------------------------------------------------------
        z = Va[k]  # Measured airspeed
        h = float(xhat[0, 0] - xhat[1, 0])  # Measurement model
        H = np.array([[1.0, -1.0, 0.0]])  # Measurement Jacobian (1x3)

        S_innov = H @ P @ H.T + R  # Innovation covariance (1x1)
        K = P @ H.T / S_innov  # Kalman gain (3x1)
        xhat = xhat + K * (z - h)  # State update
        P = (np.eye(3) - K @ H) @ P  # Covariance update

        wind_hat = float(xhat[1, 0])
        m_hat = float(m_nominal + xhat[2, 0])

        wind_est[k] = wind_hat
        mass_est[k] = m_hat

        # ---------------------------------------------------------------------
        # ENERGY-OPTIMAL GUIDANCE LAW
        # ---------------------------------------------------------------------
        Va_cmd = Va_star + 0.6 * max(wind_hat, 0.0)
        Va_cmd = min(max(Va_cmd, Va_min), Va_max)
        Va_cmd_log[k] = Va_cmd

        # ---------------------------------------------------------------------
        # AERODYNAMIC DRAG MODEL
        # ---------------------------------------------------------------------
        q = 0.5 * rho * (Va[k] ** 2)  # Dynamic pressure
        Drag = q * S * (CD0 + k_drag * (0.6**2))

        # ---------------------------------------------------------------------
        # CONTROL LAWS (Fixed vs Adaptive)
        # ---------------------------------------------------------------------
        Kp_fixed = 2.2
        Kp_adapt = Kp_fixed * (m_nominal / max(m_hat, 1.0))

        Thrust_fixed[k] = Drag + Kp_fixed * (Va_cmd - Va[k])
        Thrust_adapt[k] = Drag + Kp_adapt * (Va_cmd - Va[k])

        # ---------------------------------------------------------------------
        # VEHICLE DYNAMICS UPDATE (Adaptive Control Applied)
        # ---------------------------------------------------------------------
        ax = (Thrust_adapt[k] - Drag) / m_true
        Vg[k + 1] = Vg[k] + ax * dt

        x[k + 1] = x[k] + Vg[k] * dt
        y[k + 1] = 30.0 * np.sin(0.01 * t[k])  # Lateral motion (for 3D view)

        # ---------------------------------------------------------------------
        # ENERGY ACCOUNTING
        # ---------------------------------------------------------------------
        Power_fixed[k] = Thrust_fixed[k] * Va[k]
        Power_adapt[k] = Thrust_adapt[k] * Va[k]

        Energy_fixed[k + 1] = Energy_fixed[k] + Power_fixed[k] * dt
        Energy_adapt[k + 1] = Energy_adapt[k] + Power_adapt[k] * dt

    # Final state values
    Va[-1] = Vg[-1] - wind_true[-1]
    wind_est[-1] = wind_est[-2]
    mass_est[-1] = mass_est[-2]
    Va_cmd_log[-1] = Va_cmd_log[-2]

    # =========================================================================
    # VISUALIZATION & RESULTS
    # =========================================================================

    # Figure 1: Wind Estimation Performance
    fig1, ax1 = plt.subplots(figsize=(8, 4.5))
    ax1.plot(t, wind_true, "k-", linewidth=2, label="True Wind")
    ax1.plot(t, wind_est, "r--", linewidth=2, label="EKF Estimate")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Wind Speed (m/s)")
    ax1.set_title("EKF Wind Estimation Performance")
    ax1.legend(loc="best")
    ax1.grid(True)
    fig1.tight_layout()
    if save_dir:
        fig1.savefig(os.path.join(save_dir, "fig1_wind_estimation.png"), dpi=300)

    # Figure 2: Airspeed Tracking
    fig2, ax2 = plt.subplots(figsize=(8, 4.5))
    ax2.plot(t, Va, "b-", linewidth=1.5, label="Actual")
    ax2.plot(t, Va_cmd_log, "r--", linewidth=1.5, label="Commanded")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Airspeed (m/s)")
    ax2.set_title("Energy-Optimal Airspeed Guidance")
    ax2.legend(loc="best")
    ax2.grid(True)
    fig2.tight_layout()
    if save_dir:
        fig2.savefig(os.path.join(save_dir, "fig2_airspeed_guidance.png"), dpi=300)

    # Figure 3: Energy Comparison
    fig3, ax3 = plt.subplots(figsize=(8, 4.5))
    ax3.plot(t, Energy_fixed / 1000.0, "r--", linewidth=2, label="Fixed-Gain")
    ax3.plot(t, Energy_adapt / 1000.0, "b-", linewidth=2, label="Adaptive")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Energy (kJ)")
    ax3.set_title("Robustness Comparison via Energy Consumption")
    ax3.legend(loc="best")
    ax3.grid(True)
    fig3.tight_layout()
    if save_dir:
        fig3.savefig(os.path.join(save_dir, "fig3_energy_comparison.png"), dpi=300)

    # Figure 4: 3D Trajectory Visualization
    fig4 = plt.figure(figsize=(9, 6))
    ax4 = fig4.add_subplot(111, projection="3d")
    ax4.plot(x / 1000.0, y, np.zeros_like(x), linewidth=2, color="navy")
    ax4.set_xlabel("X Distance (km)")
    ax4.set_ylabel("Y Deviation (m)")
    ax4.set_zlabel("Altitude (m)")
    ax4.set_title("3D UAV Mission Trajectory")
    ax4.view_init(elev=25, azim=35)
    ax4.grid(True)
    fig4.tight_layout()
    if save_dir:
        fig4.savefig(os.path.join(save_dir, "fig4_3d_trajectory.png"), dpi=300)

    # Summary Table (defense-ready)
    results_df = pd.DataFrame(
        {
            "Energy_Fixed_kJ": [Energy_fixed[-1] / 1000.0],
            "Energy_Adaptive_kJ": [Energy_adapt[-1] / 1000.0],
            "Max_Wind_Est_Error_mps": [np.max(np.abs(wind_true - wind_est))],
        }
    )

    print("\n" + "=" * 60)
    print("PROJECT 1: SIMULATION SUMMARY TABLE")
    print("=" * 60)
    print(results_df.to_string(index=False))
    print("=" * 60 + "\n")

    if show_plots:
        plt.show()

    return results_df


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Project 1 As-Is Simulation")
    parser.add_argument("--show", action="store_true", help="Display interactive plots")
    parser.add_argument(
        "--save-dir",
        type=str,
        default="output_figures",
        help="Directory to save figures",
    )
    args = parser.parse_args()

    run_simulation(show_plots=args.show, save_dir=args.save_dir)
