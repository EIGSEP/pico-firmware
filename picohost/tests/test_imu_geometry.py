import numpy as np
import pytest

from picohost import imu_geometry as ig


def test_rotation_matrices_are_orthonormal():
    for R in (ig.R_x(0.3), ig.R_y(-1.1), ig.R_z(2.0)):
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(R), 1.0)


def test_rz_rotates_x_into_y():
    v = ig.R_z(np.pi / 2) @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(v, [0.0, 1.0, 0.0], atol=1e-12)


def test_fit_accel_sphere_recovers_bias_and_scale():
    rng = np.random.default_rng(0)
    bias_true = np.array([0.4, -0.2, 0.1])
    scale_true = 12.2  # mirrors the 0627 ~1.24x anomaly
    dirs = rng.normal(size=(200, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    samples = scale_true * dirs + bias_true
    bias, scale = ig.fit_accel_sphere(samples)
    assert np.allclose(bias, bias_true, atol=1e-6)
    assert scale == pytest.approx(scale_true, abs=1e-6)


def test_fit_accel_sphere_needs_four_points():
    with pytest.raises(ValueError):
        ig.fit_accel_sphere(np.zeros((3, 3)))


def test_precondition_subtracts_bias_and_normalizes():
    a = np.array([0.4 + 5.0, -0.2, 0.1])  # bias + 5*x_hat
    u = ig.precondition(a, bias=[0.4, -0.2, 0.1])
    assert np.allclose(u, [1.0, 0.0, 0.0], atol=1e-12)


def test_precondition_batched():
    bias = [0.0, 0.0, 0.0]
    a = np.array([[3.0, 0.0, 0.0], [0.0, 0.0, -2.0]])
    u = ig.precondition(a, bias)
    assert np.allclose(np.linalg.norm(u, axis=1), 1.0)


def test_kabsch_recovers_known_rotation():
    M_true = ig.R_z(0.7) @ ig.R_x(0.3)
    rng = np.random.default_rng(1)
    body = rng.normal(size=(50, 3))
    body /= np.linalg.norm(body, axis=1, keepdims=True)
    host = (M_true @ body.T).T
    M = ig.kabsch(body, host)
    assert np.allclose(M, M_true, atol=1e-9)
    assert np.isclose(np.linalg.det(M), 1.0)


def test_fit_plane_normal_of_circle():
    # points on a circle in the x-y plane -> normal is +/- z
    ang = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    pts = np.column_stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)])
    n = ig.fit_plane_normal(pts)
    assert np.allclose(np.abs(n), [0, 0, 1], atol=1e-9)


def test_nearest_signed_permutation_identity():
    labels, misalign = ig.nearest_signed_permutation(np.eye(3))
    assert labels == ["+x", "+y", "+z"]
    assert misalign == pytest.approx(0.0, abs=1e-9)


def test_nearest_signed_permutation_swapped_axes():
    # body x -> host +y, body y -> host -x, body z -> host +z
    M = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    labels, misalign = ig.nearest_signed_permutation(M)
    assert labels == ["+y", "-x", "+z"]
    assert misalign == pytest.approx(0.0, abs=1e-9)


def test_nearest_signed_permutation_reports_small_misalignment():
    M = ig.R_z(np.radians(4.0))  # 4 deg off identity
    labels, misalign = ig.nearest_signed_permutation(M)
    assert labels == ["+x", "+y", "+z"]
    assert misalign == pytest.approx(4.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Task 3: el/az estimators, tilt blend, accel inverse
# ---------------------------------------------------------------------------


def _accel_el(theta, M=np.eye(3)):
    """imu_el unit accel for elevation theta (rad): M^T @ [0,sin,cos]."""
    return M.T @ np.array([0.0, np.sin(theta), np.cos(theta)])


def _accel_az(theta, phi, M=np.eye(3)):
    """imu_az unit accel: M^T @ Rz(phi)^T @ [0,sin(theta),cos(theta)]."""
    g_box = np.array([0.0, np.sin(theta), np.cos(theta)])
    return M.T @ (ig.R_z(phi).T @ g_box)


def test_el_from_imu_recovers_theta_identity_and_mount():
    M = ig.R_y(0.5) @ ig.R_x(-0.2)
    for deg in (-40.0, 0.0, 25.0, 80.0):
        a = _accel_el(np.radians(deg), M)
        assert ig.el_from_imu(a, M) == pytest.approx(deg, abs=1e-6)


def test_el_abs_from_imu_az_is_unsigned():
    M = ig.R_z(0.3)
    a_pos = _accel_az(np.radians(30.0), np.radians(50.0), M)
    a_neg = _accel_az(np.radians(-30.0), np.radians(50.0), M)
    assert ig.el_abs_from_imu_az(a_pos, M) == pytest.approx(30.0, abs=1e-6)
    assert ig.el_abs_from_imu_az(a_neg, M) == pytest.approx(30.0, abs=1e-6)


def test_az_from_accel_recovers_phi_with_offset_and_sign():
    M = ig.R_x(0.1)
    a = _accel_az(np.radians(35.0), np.radians(70.0), M)
    # default sign/offset -> raw phi
    assert ig.az_from_accel(a, M) == pytest.approx(70.0, abs=1e-6)
    # registered to a pot frame: az = -phi + 200
    assert ig.az_from_accel(a, M, az_sign=-1.0, az_offset_deg=200.0) == (
        pytest.approx(130.0, abs=1e-6)
    )


def test_az_from_yaw_applies_sign_and_offset():
    assert ig.az_from_yaw(10.0, az_yaw_sign=-1.0, az_yaw_offset_deg=5.0) == (
        pytest.approx(-5.0)
    )


def test_blend_az_picks_yaw_near_level_and_accel_when_tilted():
    az_y, az_a = 100.0, 104.0
    near, w_near = ig.blend_az(
        az_a, az_y, el_deg=0.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    tilt, w_tilt = ig.blend_az(
        az_a, az_y, el_deg=90.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    assert near == pytest.approx(100.0)  # near a pole -> all yaw
    assert w_near == pytest.approx(0.0)
    assert tilt == pytest.approx(104.0)  # well tilted -> all accel
    assert w_tilt == pytest.approx(1.0)


def test_blend_az_full_accel_plateau_between_thresholds():
    # sin^2 weight saturates to 1 across the well-tilted band
    # [theta_sat, 180 - theta_sat]; both edges and the midpoint are pure accel.
    for el in (45.0, 90.0, 135.0):
        az, w = ig.blend_az(
            104.0, 100.0, el_deg=el, theta_sat_deg=45.0, theta_dead_deg=8.0
        )
        assert w == pytest.approx(1.0)
        assert az == pytest.approx(104.0)


def test_blend_az_deadband_zeros_weight_near_both_poles():
    # within theta_dead of colatitude 0 OR 180 deg the accel-azimuth is
    # degenerate, so the weight is hard-zeroed and the blend is pure yaw.
    for el in (5.0, 175.0):
        az, w = ig.blend_az(
            104.0, 100.0, el_deg=el, theta_sat_deg=45.0, theta_dead_deg=8.0
        )
        assert w == pytest.approx(0.0)
        assert az == pytest.approx(100.0)  # pure yaw


def test_blend_az_weight_is_pole_symmetric():
    # weight depends on |sin(el)|, so el and 180-el give identical weight:
    # the inverted high pole is handled exactly like the low one.
    _, w_lo = ig.blend_az(
        104.0, 100.0, el_deg=20.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    _, w_hi = ig.blend_az(
        104.0, 100.0, el_deg=160.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    assert w_lo == pytest.approx(w_hi)
    expected = (np.sin(np.radians(20.0)) / np.sin(np.radians(45.0))) ** 2
    assert w_lo == pytest.approx(expected)


def test_blend_az_takes_shortest_circular_path():
    # yaw 359, accel 1 -> blend should cross 0, not sweep backwards
    az, w = ig.blend_az(
        1.0, 359.0, el_deg=90.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    assert az == pytest.approx(1.0)  # w=1 -> accel; sanity on wrap helper
    # el=30 -> w = (sin30/sin45)^2 = 0.5; halfway across the wrap seam
    mid, w_mid = ig.blend_az(
        1.0, 359.0, el_deg=30.0, theta_sat_deg=45.0, theta_dead_deg=8.0
    )
    assert w_mid == pytest.approx(0.5)
    assert mid == pytest.approx(360.0, abs=1e-6)


def test_blend_az_misconfig_defaults_to_yaw():
    # non-positive theta_sat is a misconfiguration -> degrade to all-yaw
    # (accel-azimuth is the untrustworthy estimator near the poles), never
    # dividing by zero.
    az, w = ig.blend_az(
        104.0, 100.0, el_deg=90.0, theta_sat_deg=0.0, theta_dead_deg=8.0
    )
    assert az == pytest.approx(100.0)
    assert w == pytest.approx(0.0)


def test_estimate_theta_phi_round_trip():
    M_el, M_az = ig.R_x(0.2), ig.R_z(-0.4) @ ig.R_y(0.1)
    theta_t, phi_t = 35.0, -60.0
    a_el = _accel_el(np.radians(theta_t), M_el)
    a_az = _accel_az(np.radians(theta_t), np.radians(phi_t), M_az)
    theta, phi = ig.estimate_theta_phi_from_accel(a_el, a_az, M_el, M_az)
    assert theta == pytest.approx(theta_t, abs=1e-6)
    assert phi == pytest.approx(phi_t, abs=1e-6)


# ---------------------------------------------------------------------------
# Task 8: assign_sweep_theta, cone_angle, register_linear, fit_calibration
# ---------------------------------------------------------------------------


def _el_sweep_accel(thetas_deg, M, bias=(0, 0, 0), scale=1.0):
    out = []
    for d in thetas_deg:
        th = np.radians(d)
        g = M.T @ np.array([0.0, np.sin(th), np.cos(th)])
        out.append(scale * ig.GRAVITY * g + np.asarray(bias))
    return np.array(out)


def _az_sweep_accel(theta_deg, phis_deg, M, bias=(0, 0, 0), scale=1.0):
    out = []
    th = np.radians(theta_deg)
    for d in phis_deg:
        g = M.T @ (ig.R_z(np.radians(d)).T @ [0.0, np.sin(th), np.cos(th)])
        out.append(scale * ig.GRAVITY * g + np.asarray(bias))
    return np.array(out)


def test_assign_sweep_theta_signed_arc():
    M = np.eye(3)
    thetas = np.array([-30.0, -10.0, 0.0, 20.0, 50.0])
    u = ig.precondition(_el_sweep_accel(thetas, M), [0, 0, 0])
    level = 2  # index of the 0.0 entry
    rec = np.degrees(ig.assign_sweep_theta(u, level_index=level, direction=1))
    assert np.allclose(rec, thetas, atol=1e-6)


def test_cone_angle_of_fixed_tilt_az_sweep():
    phis = np.linspace(0, 360, 24, endpoint=False)
    u = ig.precondition(_az_sweep_accel(40.0, phis, np.eye(3)), [0, 0, 0])
    assert np.degrees(ig.cone_angle(u)) == pytest.approx(40.0, abs=1e-6)


def test_register_linear_recovers_sign_and_offset():
    truth = np.array([0.0, 50.0, 100.0, 150.0])
    pred = -truth + 30.0
    sign, offset = ig.register_linear(pred, truth)
    assert sign == pytest.approx(-1.0)
    # truth = sign*pred + offset  ->  offset = 30
    assert offset == pytest.approx(30.0, abs=1e-6)


def test_fit_calibration_full_reproduces_truth():
    M_el = ig.R_y(0.25) @ ig.R_x(-0.15)
    M_az = ig.R_z(-0.5) @ ig.R_x(0.1)
    bias_az, scale_az = (0.3, -0.2, 0.1), 12.2 / ig.GRAVITY
    el_degs = np.linspace(-80, 80, 33)
    phi_degs = np.linspace(0, 359, 36)
    # az sweeps reported in the POT frame: pot = -phi + 40 (sign -1, offset 40)
    pot_level = -phi_degs + 40.0
    pot_tilt = -phi_degs + 40.0

    # one IMU has one (bias, scale): apply bias_az/scale_az to EVERY imu_az
    # chunk. imu_el here is bias-free (bias=0) and shares its own scale=1.
    el_sweep = {
        "imu_el": _el_sweep_accel(el_degs, M_el),
        "imu_az": _el_sweep_accel(el_degs, M_az, bias_az, scale_az),
        "level_index": int(np.argmin(np.abs(el_degs))),
        "direction": 1,
    }
    az_level = {
        "imu_az": _az_sweep_accel(0.0, phi_degs, M_az, bias_az, scale_az),
        "yaw_deg": -phi_degs,  # yaw tracks -phi at level for this M_az
        "pot_deg": pot_level,
    }
    az_tilt = {
        "imu_az": _az_sweep_accel(40.0, phi_degs, M_az, bias_az, scale_az),
        "pot_deg": pot_tilt,
        "imu_el": _el_sweep_accel(np.full_like(phi_degs, 40.0), M_el),
    }
    cal = ig.fit_calibration_from_sweeps(el_sweep, az_level, az_tilt)

    # imu_el mount recovered (up to the fit) -> el reproduces truth
    a = ig.precondition(
        _el_sweep_accel([55.0], M_el)[0], cal["imu_el"]["accel_bias"]
    )
    assert ig.el_from_imu(a, np.array(cal["imu_el"]["M"])) == pytest.approx(
        55.0, abs=0.2
    )

    # imu_az: az reproduces the POT value (pot = -phi + 40) at a tilted pose
    M_az_fit = np.array(cal["imu_az"]["M"])
    a_az = ig.precondition(
        _az_sweep_accel(40.0, [70.0], M_az, bias_az, scale_az)[0],
        cal["imu_az"]["accel_bias"],
    )
    az = ig.az_from_accel(
        a_az,
        M_az_fit,
        cal["imu_az"]["az_sign"],
        cal["imu_az"]["az_accel_offset_deg"],
    )
    assert az == pytest.approx(-70.0 + 40.0, abs=0.5)
    # cross-check fields present
    assert (
        "mount_perm" in cal["imu_az"] and "mount_misalign_deg" in cal["imu_az"]
    )
    # blend shaping stamped for the live handler (base.py reads these back)
    assert cal["imu_az"]["theta_sat_deg"] == pytest.approx(45.0)
    assert cal["imu_az"]["theta_dead_deg"] == pytest.approx(8.0)
    # yaw registration: pot = -phi + 40, yaw = -phi => pot = yaw + 40
    # => sign = +1, offset = +40
    assert cal["imu_az"]["az_yaw_sign"] == pytest.approx(1.0)
    assert cal["imu_az"]["az_yaw_offset_deg"] == pytest.approx(40.0, abs=1.0)


def test_circular_mean_deg_handles_wrap():
    # ordinary samples: circular mean matches the arithmetic mean
    assert ig.circular_mean_deg([10.0, 20.0, 30.0]) == pytest.approx(20.0)
    # straddling the +/-180 seam: a linear mean would give ~0; circular = 180
    assert abs(ig.circular_mean_deg([179.0, -179.0])) == pytest.approx(180.0)
    assert abs(ig.circular_mean_deg([170.0, -170.0])) == pytest.approx(
        180.0, abs=1e-6
    )


# ---------------------------------------------------------------------------
# Task 1: Math backstops (zero-norm and degenerate accel guards)
# ---------------------------------------------------------------------------


def test_precondition_rejects_zero_norm_vector():
    """A faulted IMU streams accel=[0,0,0]; with a zero bias that normalizes
    to NaN. precondition must raise a descriptive ValueError instead."""
    with pytest.raises(ValueError, match="zero-norm"):
        ig.precondition(np.zeros((4, 3)), np.zeros(3))


def test_precondition_accepts_normal_vectors():
    """Guard must not disturb the normal path: real |g| accel preconditions
    to unit vectors."""
    a = np.array([[0.0, 0.0, 9.81], [9.81, 0.0, 0.0]])
    u = ig.precondition(a, np.zeros(3))
    assert np.allclose(np.linalg.norm(u, axis=-1), 1.0)


def test_fit_accel_sphere_rejects_degenerate_scale():
    """All-zero accel samples fit a sphere of radius 0; that degenerate scale
    must raise, not silently feed scale=0 into a divide."""
    with pytest.raises(ValueError, match="degenerate"):
        ig.fit_accel_sphere(np.zeros((6, 3)))
