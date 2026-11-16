import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

CORE_DIR = Path(__file__).resolve().parent.parent
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from simulation.run_dual_simulation import run_benchmark


def export_flight_telemetry(decimate_factor: int = 5) -> Path:
    assert decimate_factor >= 1

    print("Running dual simulation for telemetry export...")
    log = run_benchmark()

    time = log["time"]
    total_steps = len(time)
    indices = np.arange(0, total_steps, decimate_factor)

    t_arr = time[indices]
    base_va = log["base_Va"][indices]
    base_e = log["base_energy"][indices]
    base_pn = log["base_pos_n"][indices]

    prop_va = log["prop_Va"][indices]
    prop_va_cmd = log["prop_Va_cmd"][indices]
    prop_e = log["prop_energy"][indices]
    prop_pn = log["prop_pos_n"][indices]

    true_m = log["true_mass"][indices]
    ekf_m = log["ekf_mass"][indices]
    ekf_sig = log["ekf_mass_sigma"][indices]
    legacy_m = log["legacy_mass"][indices]

    dt_export = float(np.round(t_arr[1] - t_arr[0], 4)) if len(t_arr) > 1 else 0.1
    e_base_final = float(base_e[-1])
    e_prop_final = float(prop_e[-1])
    saved_pct = float(
        np.round(((e_base_final - e_prop_final) / e_base_final) * 100.0, 2)
    )

    timeline: list[dict[str, Any]] = []
    for i in range(len(indices)):
        timeline.append(
            {
                "time": float(np.round(t_arr[i], 2)),
                "baseline": {
                    "pn": float(np.round(base_pn[i], 2)),
                    "pe": 0.0,
                    "alt": 100.0,
                    "airspeed": float(np.round(base_va[i], 2)),
                    "energy_kj": float(np.round(base_e[i], 2)),
                },
                "proposed": {
                    "pn": float(np.round(prop_pn[i], 2)),
                    "pe": 0.0,
                    "alt": 100.0,
                    "airspeed": float(np.round(prop_va[i], 2)),
                    "airspeed_cmd": float(np.round(prop_va_cmd[i], 2)),
                    "energy_kj": float(np.round(prop_e[i], 2)),
                    "true_mass": float(np.round(true_m[i], 2)),
                    "ekf_mass": float(np.round(ekf_m[i], 2)),
                    "ekf_sigma": float(np.round(ekf_sig[i], 3)),
                    "legacy_mass": float(np.round(legacy_m[i], 2)),
                },
            }
        )

    payload = {
        "metadata": {
            "mission": "6-DoF Wind-Aware Optimal Airspeed & Adaptive MRAC Benchmark",
            "dt": dt_export,
            "total_time": float(time[-1]),
            "samples": len(timeline),
            "payload_drop_time": 150.0,
            "m_nominal": 13.5,
            "m_payload": 3.0,
            "e_baseline_kj": float(np.round(e_base_final, 2)),
            "e_proposed_kj": float(np.round(e_prop_final, 2)),
            "energy_saved_percent": saved_pct,
        },
        "timeline": timeline,
    }

    target_path = (
        CORE_DIR.parent
        / "tier3_react_gcs_dashboard"
        / "src"
        / "data"
        / "flight_telemetry.json"
    )
    target_path.parent.mkdir(parents=True, exist_ok=True)

    with open(target_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    return target_path


if __name__ == "__main__":
    out_file = export_flight_telemetry(decimate_factor=5)
    assert out_file.exists()
    assert out_file.stat().st_size > 0
    print("Export Telemetry Test Passed.")
    print(
        f"JSON Telemetry exported to: {out_file} ({out_file.stat().st_size / 1024:.1f} KB)"
    )
