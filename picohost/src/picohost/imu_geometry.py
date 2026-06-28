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


# ---------------------------------------------------------------------------
# Task 3: handler-facing estimators
# ---------------------------------------------------------------------------


def _wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def el_from_imu(a_unit, M):
    """Signed elevation (deg) for imu_el: arctan2(g_y, g_z) in box frame."""
    g = np.asarray(M, dtype=float) @ np.asarray(a_unit, dtype=float)
    return float(np.degrees(np.arctan2(g[1], g[2])))


def el_abs_from_imu_az(a_unit, M):
    """|theta| (deg) for imu_az; assumes theta >= 0 for the single-tick case."""
    g = np.asarray(M, dtype=float) @ np.asarray(a_unit, dtype=float)
    return float(np.degrees(np.arccos(np.clip(g[2], -1.0, 1.0))))


def az_from_accel(a_unit, M, az_sign=1.0, az_offset_deg=0.0):
    """Azimuth (deg) from the imu_az preconditioned accel, with sign/offset.

    theta >= 0 is assumed (single-tick imu_az), so sign(sin theta) = +1.
    """
    g = np.asarray(M, dtype=float) @ np.asarray(a_unit, dtype=float)
    phi = np.degrees(np.arctan2(g[0], g[1]))
    return float(az_sign * phi + az_offset_deg)


def az_from_yaw(yaw_deg, az_yaw_sign=1.0, az_yaw_offset_deg=0.0):
    """Azimuth (deg) from BNO08x yaw with sign/offset registration."""
    return float(az_yaw_sign * yaw_deg + az_yaw_offset_deg)


def blend_az(az_accel_deg, az_yaw_deg, el_deg, theta_cross_deg):
    """Tilt-weighted circular blend: yaw near level, accel when tilted.

    weight ramps 0 -> 1 as |el| goes 0 -> 2*theta_cross_deg.
    Interpolation takes the shortest circular path from yaw toward accel.
    When w reaches 1 the accel value is returned directly (avoids an
    off-by-360 at the wrap boundary). A non-positive theta_cross_deg is a
    misconfiguration and degrades to all-accel rather than dividing by zero.
    """
    if theta_cross_deg <= 0:
        return float(az_accel_deg), 1.0
    w = float(np.clip(abs(el_deg) / (2.0 * theta_cross_deg), 0.0, 1.0))
    if w >= 1.0:
        return float(az_accel_deg), w
    delta = _wrap180(az_accel_deg - az_yaw_deg)
    return float(az_yaw_deg + w * delta), w


def estimate_theta_phi_from_accel(a_el, a_az, M_el, M_az):
    """Closed-form (theta_deg, phi_deg) from two preconditioned accel vectors.

    Port of the notebook inverse. phi is degenerate where sin(theta) -> 0.
    """
    g_box = np.asarray(M_el, dtype=float) @ np.asarray(a_el, dtype=float)
    g_tt = np.asarray(M_az, dtype=float) @ np.asarray(a_az, dtype=float)
    theta = np.arctan2(g_box[1], g_box[2])
    s = 1.0 if np.sin(theta) >= 0.0 else -1.0
    phi = np.arctan2(s * g_tt[0], s * g_tt[1])
    return float(np.degrees(theta)), float(np.degrees(phi))
