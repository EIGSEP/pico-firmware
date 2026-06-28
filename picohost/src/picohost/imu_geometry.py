"""Pure geometry for the two-IMU rotating-box conversion.

Ported from picohost/notebooks/imu_two_axis_simulation.ipynb. Shared by the
live PicoIMU handler and the calibrate_imu fit so lab and field run identical
math. All inverses are direction-only (accel must be preconditioned to a unit
gravity vector first), so a constant accel scale/bias does not affect output.
"""

import numpy as np

GRAVITY = 9.80665  # m/s^2


def R_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def R_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def R_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fit_accel_sphere(samples):
    """Algebraic least-squares sphere fit.

    Returns (bias, scale): center and radius of the sphere the accel
    samples lie on, i.e. the constant accel bias and effective |g|.
    """
    p = np.asarray(samples, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 4:
        raise ValueError("need >=4 (N,3) accel samples to fit a sphere")
    # |p|^2 = 2 c.p + (r^2 - |c|^2)  ->  linear in [cx,cy,cz,k]
    A = np.hstack([2.0 * p, np.ones((len(p), 1))])
    b = np.sum(p**2, axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    bias = sol[:3]
    scale = float(np.sqrt(sol[3] + bias @ bias))
    return bias, scale


def precondition(a, bias):
    """(a - bias) normalized to unit length. Accepts (3,) or (N,3)."""
    a = np.asarray(a, dtype=float) - np.asarray(bias, dtype=float)
    n = np.linalg.norm(a, axis=-1, keepdims=True)
    return a / n


_AXES = {
    "+x": np.array([1.0, 0.0, 0.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "+y": np.array([0.0, 1.0, 0.0]),
    "-y": np.array([0.0, -1.0, 0.0]),
    "+z": np.array([0.0, 0.0, 1.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
}


def kabsch(body, host):
    """Proper rotation M minimizing sum |M @ body_i - host_i|^2."""
    B = np.asarray(body, dtype=float)
    H = np.asarray(host, dtype=float)
    C = H.T @ B  # 3x3 = sum host_i body_i^T
    U, _, Vt = np.linalg.svd(C)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


def fit_plane_normal(unit_vectors):
    """Normal of the best-fit plane (PCA smallest-variance direction)."""
    X = np.asarray(unit_vectors, dtype=float)
    Xc = X - X.mean(axis=0)
    _, _, Vt = np.linalg.svd(Xc)
    return Vt[-1]


def nearest_signed_permutation(M):
    """Nearest signed-permutation mount + residual misalignment angle (deg).

    Column k of M is body-axis k expressed in the host frame, so the label
    for column k is the host axis it points most nearly along.
    """
    M = np.asarray(M, dtype=float)
    labels = [max(_AXES, key=lambda k: M[:, c] @ _AXES[k]) for c in range(3)]
    P = np.column_stack([_AXES[lbl] for lbl in labels])
    R = M @ P.T
    cos = (np.trace(R) - 1.0) / 2.0
    misalign = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    return labels, misalign
