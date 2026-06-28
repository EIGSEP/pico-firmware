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


# ---------------------------------------------------------------------------
# Task 8: sweep helpers + calibration fitter
# ---------------------------------------------------------------------------


def assign_sweep_theta(unit_vectors, level_index, direction):
    """Signed elevation angle (rad) per sample along a 1-axis sweep.

    Angle of each sample from the level sample, about the fitted plane
    normal; 0 at level_index; sign forced so it increases in `direction`.
    """
    X = np.asarray(unit_vectors, dtype=float)
    ref = X[level_index]
    n = fit_plane_normal(X)
    thetas = np.empty(len(X))
    for i, v in enumerate(X):
        ang = np.arccos(np.clip(v @ ref, -1.0, 1.0))
        s = np.sign(np.cross(ref, v) @ n)
        thetas[i] = ang * (s if s != 0 else 1.0)
    # orient so samples after level_index increase, scaled by commanded dir
    if level_index + 1 < len(thetas) and thetas[level_index + 1] < 0:
        thetas = -thetas
    return thetas * direction


def cone_angle(unit_vectors):
    """Polar angle (rad) of a fixed-tilt az sweep about its plane normal."""
    X = np.asarray(unit_vectors, dtype=float)
    n = fit_plane_normal(X)
    # samples sit at colatitude theta from the rotation axis; |X.n| = cos(theta)
    c = np.clip(np.abs(X @ n).mean(), -1.0, 1.0)
    return float(np.arccos(c))


def register_linear(pred_deg, truth_deg):
    """Find sign in {+1,-1} and offset so truth ~= sign*pred + offset."""
    pred = np.asarray(pred_deg, dtype=float)
    truth = np.asarray(truth_deg, dtype=float)
    best = None
    for sign in (1.0, -1.0):
        resid = truth - sign * pred
        # circular mean of the residual offset
        offset = np.degrees(
            np.arctan2(
                np.sin(np.radians(resid)).mean(),
                np.cos(np.radians(resid)).mean(),
            )
        )
        err = np.abs(_wrap180(truth - (sign * pred + offset)))
        score = err.mean()
        if best is None or score < best[0]:
            best = (score, sign, float(offset))
    return best[1], best[2]


def _host_el(theta_rad):
    return np.array([0.0, np.sin(theta_rad), np.cos(theta_rad)])


def fit_calibration_from_sweeps(
    el_sweep, az_level, az_tilt, *, theta_cross_deg=1.6
):
    """Fit per-IMU calibration sections from the three sweeps.

    Each present IMU gets a sphere fit (bias/scale), a Kabsch mount fit,
    and (imu_az) pot registration for accel-phi and yaw. Missing IMUs are
    omitted. See the test for the exact input dict shape.
    """
    out = {}

    # ---- imu_el: elevation sweep only ----
    # NOTE: a single-axis sweep is coplanar, so the sphere fit is rank
    # deficient along the el-axis. np.linalg.lstsq returns the min-norm
    # solution (the unobservable bias component -> ~0). That component lies
    # along the el rotation axis (host x), which does not enter
    # el = atan2(g_y, g_z), so el is unaffected. The in-plane bias (which
    # does affect el) is recovered.
    if el_sweep.get("imu_el") is not None:
        raw = np.asarray(el_sweep["imu_el"], float)
        bias, scale = fit_accel_sphere(raw)
        u = precondition(raw, bias)
        theta = assign_sweep_theta(
            u, el_sweep["level_index"], el_sweep["direction"]
        )
        host = np.array([_host_el(t) for t in theta])
        M = kabsch(u, host)
        perm, mis = nearest_signed_permutation(M)
        out["imu_el"] = {
            "accel_bias": bias.tolist(),
            "accel_scale": scale,
            "M": M.tolist(),
            "mount_perm": perm,
            "mount_misalign_deg": mis,
        }

    # ---- imu_az: elevation sweep + azimuth sweeps ----
    if az_tilt.get("imu_az") is not None:
        # Sphere fit over ALL imu_az accel we have (best sphere coverage).
        chunks = [np.asarray(az_tilt["imu_az"], float)]
        if az_level.get("imu_az") is not None:
            chunks.append(np.asarray(az_level["imu_az"], float))
        if el_sweep.get("imu_az") is not None:
            chunks.append(np.asarray(el_sweep["imu_az"], float))
        raw_all = np.vstack(chunks)
        bias, scale = fit_accel_sphere(raw_all)

        # theta for the tilt sweep: from imu_el if alive, else the cone angle.
        u_tilt = precondition(np.asarray(az_tilt["imu_az"], float), bias)
        if az_tilt.get("imu_el") is not None and "imu_el" in out:
            M_el = np.array(out["imu_el"]["M"])
            b_el = np.array(out["imu_el"]["accel_bias"])
            u_el = precondition(np.asarray(az_tilt["imu_el"], float), b_el)
            theta_tilt = np.array([el_from_imu(a, M_el) for a in u_el])
            theta_tilt = np.radians(theta_tilt)
        else:
            theta_tilt = np.full(len(u_tilt), cone_angle(u_tilt))

        phi_tilt = np.radians(np.asarray(az_tilt["pot_deg"], float))
        host_tilt = np.array(
            [R_z(p).T @ _host_el(t) for p, t in zip(phi_tilt, theta_tilt)]
        )
        M_az = kabsch(u_tilt, host_tilt)
        perm, mis = nearest_signed_permutation(M_az)

        # accel-phi -> pot registration (use tilt sweep, where phi is sharp)
        phi_pred = np.array([az_from_accel(a, M_az) for a in u_tilt])
        az_sign, az_off = register_linear(phi_pred, np.degrees(phi_tilt))

        section = {
            "accel_bias": bias.tolist(),
            "accel_scale": scale,
            "M": M_az.tolist(),
            "az_sign": az_sign,
            "az_accel_offset_deg": az_off,
            "theta_cross_deg": float(theta_cross_deg),
            "mount_perm": perm,
            "mount_misalign_deg": mis,
            "az_yaw_sign": 1.0,
            "az_yaw_offset_deg": 0.0,
        }
        # yaw -> pot registration from the near-level az sweep
        if (
            az_level.get("imu_az") is not None
            and az_level.get("yaw_deg") is not None
        ):
            yaw = np.asarray(az_level["yaw_deg"], float)
            pot = np.asarray(az_level["pot_deg"], float)
            ys, yo = register_linear(yaw, pot)
            section["az_yaw_sign"], section["az_yaw_offset_deg"] = ys, yo
        out["imu_az"] = section

    return out
