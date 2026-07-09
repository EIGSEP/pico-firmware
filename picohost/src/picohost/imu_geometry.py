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
# Task 3: handler-facing estimators
# ---------------------------------------------------------------------------


def _wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    """Circular mean (deg) of angle samples; robust to the +/-180 wrap.

    A plain arithmetic mean of angles straddling the +/-180 seam (e.g. yaw
    near +/-179) collapses toward 0; averaging the unit vectors instead keeps
    the result on the circle.
    """
    a = np.radians(np.asarray(values, dtype=float))
    return float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())))


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


# Default blend shaping (deg). Centralized so the calibrate_imu CLI, the fit
# that stamps them into the cal section, and the live handler that reads them
# back all agree -- and so deployed cals written before these keys existed pick
# up the same values via .get(...) fallbacks.
DEFAULT_THETA_SAT_DEG = 45.0  # |sin el| >= sin(this) -> full accel
DEFAULT_THETA_DEAD_DEG = 8.0  # within this of either pole -> pure yaw


def blend_az(az_accel_deg, az_yaw_deg, el_deg, theta_sat_deg, theta_dead_deg):
    """Tilt-weighted circular blend: yaw near the poles, accel when tilted.

    ``el_deg`` is the az-IMU colatitude (angle between gravity and the azimuth
    spin axis). Accel-azimuth quality scales as ``|sin(el)|``, which is zero at
    *both* poles (colatitude 0 and 180 deg, i.e. level and inverted) and peaks
    at 90 deg. The weight on the accel estimate is a sin^2 ("signal power")
    ramp::

        w = clip((|sin el| / sin theta_sat)**2, 0, 1)

    which saturates to full accel across the well-tilted band
    ``[theta_sat, 180 - theta_sat]`` and is hard-zeroed inside a deadband
    within ``theta_dead`` of either pole, where accel-azimuth is degenerate.
    Keying on ``|sin el|`` makes the curve symmetric about 90 deg, so the
    inverted high pole is handled exactly like the low one.

    Interpolation takes the shortest circular path from yaw toward accel; when
    w reaches 1 the accel value is returned directly (avoids an off-by-360 at
    the wrap boundary). A non-positive ``theta_sat_deg`` is a misconfiguration
    and degrades to all-yaw -- accel is the untrustworthy estimator near the
    poles -- rather than dividing by zero.
    """
    if theta_sat_deg <= 0:
        return float(az_yaw_deg), 0.0
    s = abs(float(np.sin(np.radians(el_deg))))
    if s <= np.sin(np.radians(theta_dead_deg)):
        return float(az_yaw_deg), 0.0
    w = float(np.clip((s / np.sin(np.radians(theta_sat_deg))) ** 2, 0.0, 1.0))
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
    el_sweep,
    az_level,
    az_tilt,
    *,
    theta_sat_deg=DEFAULT_THETA_SAT_DEG,
    theta_dead_deg=DEFAULT_THETA_DEAD_DEG,
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
            "theta_sat_deg": float(theta_sat_deg),
            "theta_dead_deg": float(theta_dead_deg),
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
