import sys
from pathlib import Path

import numpy as np

CORE_DIR = Path(__file__).resolve().parent.parent
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from estimation.observable_ekf import ObservableEKF


def compute_legacy_observability_matrix(tau_w: float = 10.0) -> np.ndarray:
    F = np.array(
        [[0.0, 0.0, 0.0], [0.0, -1.0 / tau_w, 0.0], [0.0, 0.0, 0.0]], dtype=float
    )

    H = np.array([[1.0, -1.0, 0.0]], dtype=float)

    O_matrix = np.vstack([H, H @ F, H @ (F @ F)])
    return O_matrix


def compute_proposed_observability_matrix(
    ekf: ObservableEKF, thrust: float = 18.0
) -> tuple[np.ndarray, np.ndarray]:
    Vg, w, m = ekf.get_state()
    Va = max(1e-4, Vg - w)
    q_inf = 0.5 * ekf.rho * Va**2
    CL = (2.0 * m * ekf.g) / (ekf.rho * ekf.S * Va**2)
    CD = ekf.CD0 + ekf.k_induced * CL**2
    Drag = q_inf * ekf.S * CD

    dD_dVa = ekf.rho * Va * ekf.S * ekf.CD0 - (
        4.0 * ekf.k_induced * (m * ekf.g) ** 2
    ) / (ekf.rho * ekf.S * Va**3)
    dD_dm = (4.0 * ekf.k_induced * m * ekf.g**2) / (ekf.rho * ekf.S * Va**2)

    df1_dVg = -dD_dVa / m
    df1_dw = dD_dVa / m
    df1_dm = -(thrust - Drag) / (m**2) - (dD_dm / m)

    F = np.array(
        [[df1_dVg, df1_dw, df1_dm], [0.0, -1.0 / ekf.tau_w, 0.0], [0.0, 0.0, 0.0]],
        dtype=float,
    )

    dax_dVg = -dD_dVa / m
    dax_dw = dD_dVa / m
    dax_dm = -(thrust - Drag) / (m**2) - (dD_dm / m)

    H = np.array(
        [[1.0, -1.0, 0.0], [1.0, 0.0, 0.0], [dax_dVg, dax_dw, dax_dm]], dtype=float
    )

    O_matrix = np.vstack([H, H @ F, H @ (F @ F)])
    return O_matrix, H


def test_observability_rank_proof() -> None:
    # 1. Evaluate Legacy MATLAB Observability
    O_legacy = compute_legacy_observability_matrix()
    rank_legacy = int(np.linalg.matrix_rank(O_legacy))

    # Column 3 (mass state) is strictly zero
    assert np.allclose(O_legacy[:, 2], 0.0)
    assert rank_legacy == 2
    print(
        f"[PASSED] Legacy Observability Rank: {rank_legacy}/3 (Mass is Strictly Unobservable)"
    )

    # 2. Evaluate Proposed Multi-Sensor Observability
    ekf = ObservableEKF(m_init=13.5, Vg_init=25.0, w_init=6.0)
    O_proposed, H_proposed = compute_proposed_observability_matrix(ekf, thrust=18.0)
    rank_proposed = int(np.linalg.matrix_rank(O_proposed))
    rank_H = int(np.linalg.matrix_rank(H_proposed))

    assert rank_H == 3
    assert rank_proposed == 3
    print(
        f"[PASSED] Proposed Observability Rank: {rank_proposed}/3 (Full Rank Algebraic Observability)"
    )

    # 3. Envelope Sweep (Va from 15 to 35 m/s, mass from 10 to 14 kg)
    for test_va in np.linspace(15.0, 35.0, 5):
        for test_m in [10.5, 12.0, 13.5]:
            ekf_sweep = ObservableEKF(m_init=test_m, Vg_init=test_va, w_init=0.0)
            O_sw, _ = compute_proposed_observability_matrix(ekf_sweep, thrust=20.0)
            assert np.linalg.matrix_rank(O_sw) == 3

    print("[PASSED] Observability Rank Verified Across Entire Flight Envelope.")


if __name__ == "__main__":
    test_observability_rank_proof()
