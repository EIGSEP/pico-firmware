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
    near, w_near = ig.blend_az(az_a, az_y, el_deg=0.0, theta_cross_deg=1.6)
    tilt, w_tilt = ig.blend_az(az_a, az_y, el_deg=45.0, theta_cross_deg=1.6)
    assert near == pytest.approx(100.0)  # all yaw
    assert w_near == pytest.approx(0.0)
    assert tilt == pytest.approx(104.0)  # all accel
    assert w_tilt == pytest.approx(1.0)


def test_blend_az_takes_shortest_circular_path():
    # yaw 359, accel 1 -> blend should cross 0, not sweep backwards
    az, w = ig.blend_az(1.0, 359.0, el_deg=45.0, theta_cross_deg=1.6)
    assert az == pytest.approx(1.0)  # w=1 -> accel; sanity on wrap helper
    mid, _ = ig.blend_az(1.0, 359.0, el_deg=1.6, theta_cross_deg=1.6)
    assert mid == pytest.approx(360.0, abs=1e-6)  # halfway across the wrap


def test_estimate_theta_phi_round_trip():
    M_el, M_az = ig.R_x(0.2), ig.R_z(-0.4) @ ig.R_y(0.1)
    theta_t, phi_t = 35.0, -60.0
    a_el = _accel_el(np.radians(theta_t), M_el)
    a_az = _accel_az(np.radians(theta_t), np.radians(phi_t), M_az)
    theta, phi = ig.estimate_theta_phi_from_accel(a_el, a_az, M_el, M_az)
    assert theta == pytest.approx(theta_t, abs=1e-6)
    assert phi == pytest.approx(phi_t, abs=1e-6)
