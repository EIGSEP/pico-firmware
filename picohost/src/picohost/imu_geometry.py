"""Pure geometry for the two-IMU rotating-box conversion.

Ported from picohost/notebooks/imu_two_axis_simulation.ipynb. Shared by the
live PicoIMU handler and the calibrate_imu fit so lab and field run identical
math. All inverses are direction-only (accel must be preconditioned to a unit
gravity vector first), so a constant accel scale/bias does not affect output.

El-only since the 2026-07-09 azimuth descope: both IMUs derive an elevation
angle (``el_from_imu`` for imu_el, signed; ``el_abs_from_imu_az`` for imu_az,
unsigned |theta|). Azimuth is owned by potmon; the accel/yaw blend estimator
and its sweep-based fitter have been retired.
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
    scale = float(np.sqrt(max(sol[3] + bias @ bias, 0.0)))
    if not np.isfinite(scale) or scale < 1e-6:
        raise ValueError(
            "fit_accel_sphere: degenerate accel sphere (scale "
            f"{scale:.3g} ~ 0; sensor likely faulted: all accel=[0,0,0])"
        )
    return bias, scale


def fit_accel_sphere_coplanar(samples, n):
    """Sphere fit for coplanar samples: bias constrained to the plane.

    A single-axis sweep's samples are coplanar (p . n ~ const), which
    makes fit_accel_sphere rank-deficient along n: the min-norm
    solution trades the large k = r^2 - |c|^2 term against a spurious
    bias component along n (~2 r^2 amplification per unit of plane
    offset), corrupting the fit for any realistic nonzero bias. The
    along-axis bias component is fundamentally unobservable from
    coplanar data, so pin it to zero and solve only for the in-plane
    (observable) bias and the radius. The rotation phase about n --
    what el_from_imu ultimately measures -- is invariant to the pinned
    component.
    """
    p = np.asarray(samples, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 4:
        raise ValueError("need >=4 (N,3) accel samples to fit a sphere")
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    seed = np.array([1.0, 0.0, 0.0])
    if abs(seed @ n) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    e1 = seed - (seed @ n) * n
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    A = np.hstack(
        [
            2.0 * (p @ e1)[:, None],
            2.0 * (p @ e2)[:, None],
            np.ones((len(p), 1)),
        ]
    )
    b = np.sum(p**2, axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    bias = sol[0] * e1 + sol[1] * e2
    scale = float(np.sqrt(max(sol[2] + bias @ bias, 0.0)))
    if not np.isfinite(scale) or scale < 1e-6:
        raise ValueError(
            "fit_accel_sphere_coplanar: degenerate accel sphere "
            f"(scale {scale:.3g} ~ 0; sensor likely faulted)"
        )
    return bias, scale


def precondition(a, bias):
    """(a - bias) normalized to unit length. Accepts (3,) or (N,3).

    Raises ValueError on a zero-norm vector (a faulted IMU streams
    accel=[0,0,0], which would otherwise normalize to NaN and surface
    several calls later as an opaque ``SVD did not converge``).
    """
    a = np.asarray(a, dtype=float) - np.asarray(bias, dtype=float)
    n = np.linalg.norm(a, axis=-1, keepdims=True)
    if np.any(n < 1e-9):
        raise ValueError(
            "precondition: zero-norm accel vector "
            "(sensor likely faulted: accel=[0,0,0])"
        )
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


# Where the preconditioned accel unit vector points at el=0, per mount
# convention: imu_el is mounted flat in the rotating box (a_unit -z at
# level); imu_az rides the az turntable, rotated 90 deg (a_unit +x).
# Validated against the 2026-07-08 field sweeps (a_unit at motor 0:
# imu_el [0.08, 0.02, -0.996]; imu_az [0.888, 0.058, 0.457]).
NOMINAL_LEVEL_AXIS = {
    "imu_el": np.array([0.0, 0.0, -1.0]),
    "imu_az": np.array([1.0, 0.0, 0.0]),
}
# Nominal axis must clear the sweep axis by at least this margin for
# the closest-point level derivation to be well-conditioned (~11 deg).
_LEVEL_AXIS_MIN_PERP = 0.2
# Derived home farther than this from motor 0 means the mount
# convention (NOMINAL_LEVEL_AXIS sign) is inverted — refuse to fit a
# calibration that is 180 deg off rather than warn.
_HOME_FLIP_GUARD_DEG = 90.0


def derive_level_theta(u, nominal_level_axis, motor_el_deg):
    """Sweep angles (rad) anchored at the derived mount-level pose.

    Replaces the operator-driven "drive to LEVEL first" anchor: level
    is the point on the fitted gravity circle closest to
    ``nominal_level_axis`` (where a_unit points at el=0 per the mount
    convention). ``motor_el_deg`` is used only to resolve the rotation
    SIGN (the fitted plane normal is sign-ambiguous) and to locate the
    derived home for the inverted-mount guard — never as fit truth.

    Returns ``(theta_rad, home_offset_deg)``: per-sample angle from
    derived level, and where the fit puts level in motor coordinates.

    Raises ValueError if the nominal axis is near-parallel to the sweep
    axis (wrong axis / IMU did not rotate) or if the derived home lies
    nearer motor +/-180 than motor 0 (inverted mount convention).
    """
    u = np.asarray(u, dtype=float)
    motor = np.asarray(motor_el_deg, dtype=float)
    n = fit_plane_normal(u)
    a = np.asarray(nominal_level_axis, dtype=float)
    a_perp = a - (a @ n) * n
    norm = np.linalg.norm(a_perp)
    if norm < _LEVEL_AXIS_MIN_PERP:
        raise ValueError(
            "nominal level axis is near-parallel to the sweep axis "
            f"(|perp| {norm:.2f}); wrong NOMINAL_LEVEL_AXIS or the "
            "sweep did not rotate this IMU"
        )
    ref = a_perp / norm
    v = np.cross(n, ref)
    theta = np.arctan2(u @ v, u @ ref)
    # resolve sign: theta must increase with motor el
    order = np.argsort(motor)
    th_u = np.unwrap(theta[order])
    slope = np.polyfit(motor[order], np.degrees(th_u), 1)[0]
    if slope < 0:
        theta = -theta
        th_u = np.unwrap(theta[order])
        slope = -slope
    coef = np.polyfit(motor[order], np.degrees(th_u), 1)
    home = -coef[1] / coef[0]
    home = (home + 180.0) % 360.0 - 180.0
    if abs(home) > _HOME_FLIP_GUARD_DEG:
        raise ValueError(
            f"derived level sits at motor {home:+.1f} deg — nearer "
            "motor 180 than motor 0. Inverted mount convention "
            "(NOMINAL_LEVEL_AXIS sign); refusing a 180-off calibration."
        )
    return theta, float(home)


def _host_el(theta_rad):
    return np.array([0.0, np.sin(theta_rad), np.cos(theta_rad)])


def fit_el_calibration(el_el, el_az, motor_el_deg):
    """Fit per-IMU el-only cal sections from one elevation sweep.

    Both IMUs are anchored to the SAME derived level: imu_el's when
    alive (it is az-invariant — the authority), imu_az's own nominal
    axis as the fallback. Requires az parked at home during the sweep
    for the imu_az section to be meaningful (enforced by the script).

    Returns (sections, report); sections match the ImuCalStore shape
    consumed by PicoIMU unchanged. report carries the derived home
    offset (motor deg), which IMU anchored it, and a per-stop
    cross-check table [(motor_deg, el_signed, el_abs), ...].
    """
    sections, u_el, u_az, theta = {}, None, None, None
    anchor = None
    home = None
    if el_el is not None:
        raw = np.asarray(el_el, dtype=float)
        n_plane = fit_plane_normal(raw)
        bias, scale = fit_accel_sphere_coplanar(raw, n_plane)
        u_el = precondition(raw, bias)
        theta, home = derive_level_theta(
            u_el, NOMINAL_LEVEL_AXIS["imu_el"], motor_el_deg
        )
        anchor = "imu_el"
        host = np.array([_host_el(t) for t in theta])
        M = kabsch(u_el, host)
        perm, mis = nearest_signed_permutation(M)
        sections["imu_el"] = {
            "accel_bias": bias.tolist(),
            "accel_scale": scale,
            "M": M.tolist(),
            "mount_perm": perm,
            "mount_misalign_deg": mis,
        }
    if el_az is not None:
        raw = np.asarray(el_az, dtype=float)
        n_plane = fit_plane_normal(raw)
        bias, scale = fit_accel_sphere_coplanar(raw, n_plane)
        u_az = precondition(raw, bias)
        if theta is None:
            theta, home = derive_level_theta(
                u_az, NOMINAL_LEVEL_AXIS["imu_az"], motor_el_deg
            )
            anchor = "imu_az"
        host = np.array([_host_el(t) for t in theta])
        M = kabsch(u_az, host)
        perm, mis = nearest_signed_permutation(M)
        sections["imu_az"] = {
            "accel_bias": bias.tolist(),
            "accel_scale": scale,
            "M": M.tolist(),
            "mount_perm": perm,
            "mount_misalign_deg": mis,
        }
    if not sections:
        return {}, {}
    cross = []
    for i, m in enumerate(np.asarray(motor_el_deg, dtype=float)):
        el_s = (
            el_from_imu(u_el[i], sections["imu_el"]["M"])
            if u_el is not None
            else None
        )
        el_a = (
            el_abs_from_imu_az(u_az[i], sections["imu_az"]["M"])
            if u_az is not None
            else None
        )
        cross.append((float(m), el_s, el_a))
    report = {
        "home_offset_motor_deg": home,
        "anchor": anchor,
        "cross_check": cross,
    }
    return sections, report


# ---------------------------------------------------------------------------
# Handler-facing estimators
# ---------------------------------------------------------------------------


def el_from_imu(a_unit, M):
    """Signed elevation (deg) for imu_el: arctan2(g_y, g_z) in box frame."""
    g = np.asarray(M, dtype=float) @ np.asarray(a_unit, dtype=float)
    return float(np.degrees(np.arctan2(g[1], g[2])))


def el_abs_from_imu_az(a_unit, M):
    """|theta| (deg) for imu_az; assumes theta >= 0 for the single-tick case."""
    g = np.asarray(M, dtype=float) @ np.asarray(a_unit, dtype=float)
    return float(np.degrees(np.arccos(np.clip(g[2], -1.0, 1.0))))
