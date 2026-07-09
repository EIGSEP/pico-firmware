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
# El estimators (imu_el signed, imu_az |theta|) — azimuth is owned by potmon
# since the 2026-07-09 descope; the accel/yaw blend estimator and its
# sweep-based fitter were retired along with their tests.
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


def _el_sweep_units(motor_deg, M_true, level_offset_deg=0.0):
    """Sensor-frame accel unit vectors for an el sweep.

    Physical el at each stop = motor + level_offset (the derived-level
    fit must find level at motor = -level_offset). M_true maps sensor
    a_unit -> host frame, host gravity at el t = [0, sin t, cos t].
    """
    ts = np.radians(np.asarray(motor_deg, float) + level_offset_deg)
    host = np.array([[0.0, np.sin(t), np.cos(t)] for t in ts])
    return host @ M_true  # == (M_true.T @ h) per row


MOTOR_STOPS = np.arange(-180.0, 181.0, 30.0)
# imu_el-like mount: a_unit at level = -z  (R_x(pi) maps -z -> +z),
# plus a ~3 deg misalignment like the field unit.
M_EL_TRUE = ig.R_z(np.radians(3.0)) @ ig.R_x(np.pi)
# imu_az-like mount: a_unit at level = +x  (R_y(-pi/2) maps +x -> +z).
M_AZ_TRUE = ig.R_y(-np.pi / 2) @ ig.R_x(np.radians(2.0))


def test_derive_level_theta_anchors_at_nominal_down():
    u = _el_sweep_units(MOTOR_STOPS, M_EL_TRUE, level_offset_deg=3.0)
    theta, home = ig.derive_level_theta(
        u, ig.NOMINAL_LEVEL_AXIS["imu_el"], MOTOR_STOPS
    )
    # level derived at motor ~ -3 (physical level is 3 deg past motor 0)
    assert abs(home - (-3.0)) < 1.0
    # theta increases with motor el (sign resolved via motor)
    order = np.argsort(MOTOR_STOPS)
    assert np.all(np.diff(np.unwrap(theta[order])) > 0)


def test_derive_level_theta_wraps_intercept_before_slope_division():
    # Slope != 1 (fitted, not nominal) exposed a wrap-before-divide bug:
    # home = -intercept/slope computed on the UNWRAPPED intercept and
    # only then wrapped, so a global +360 offset on the unwrapped theta
    # carried a 1/slope amplification through the wrap and flipped the
    # sign of small home offsets. physical el = 0.99*motor + 3.0, so
    # el=0 (home) at motor = -3.0/0.99 ~= -3.03 deg.
    scaled_motor = 0.99 * MOTOR_STOPS
    u = _el_sweep_units(scaled_motor, M_EL_TRUE, level_offset_deg=3.0)
    theta, home = ig.derive_level_theta(
        u, ig.NOMINAL_LEVEL_AXIS["imu_el"], MOTOR_STOPS
    )
    assert abs(home - (-3.0 / 0.99)) < 0.5


def test_derive_level_theta_flip_guard():
    # inverted mount: nominal -z actually points UP at level -> derived
    # level lands at motor ~180 -> hard error, not a silent 180 offset
    u = _el_sweep_units(MOTOR_STOPS, ig.R_z(np.radians(3.0)), 0.0)
    with pytest.raises(ValueError, match="[Ii]nverted|180"):
        ig.derive_level_theta(u, ig.NOMINAL_LEVEL_AXIS["imu_el"], MOTOR_STOPS)


def test_derive_level_theta_axis_parallel_degenerate():
    # nominal axis parallel to the sweep axis: no unique closest point
    u = _el_sweep_units(MOTOR_STOPS, M_EL_TRUE)
    with pytest.raises(ValueError, match="parallel"):
        ig.derive_level_theta(u, np.array([1.0, 0.0, 0.0]), MOTOR_STOPS)


def test_fit_el_calibration_recovers_both_mounts():
    scale, bias = 9.81, np.array([0.05, -0.02, 0.1])
    el_el = _el_sweep_units(MOTOR_STOPS, M_EL_TRUE) * scale + bias
    el_az = _el_sweep_units(MOTOR_STOPS, M_AZ_TRUE) * scale + bias
    sections, report = ig.fit_el_calibration(el_el, el_az, MOTOR_STOPS)
    assert set(sections) == {"imu_el", "imu_az"}
    for sec in sections.values():
        assert set(sec) == {
            "accel_bias",
            "accel_scale",
            "M",
            "mount_perm",
            "mount_misalign_deg",
        }
    # el round-trips through the same estimators the live handler uses
    u = ig.precondition(el_el, sections["imu_el"]["accel_bias"])
    for want, a in zip(MOTOR_STOPS, u):
        got = ig.el_from_imu(a, sections["imu_el"]["M"])
        assert abs((got - want + 180) % 360 - 180) < 0.5
    u_az = ig.precondition(el_az, sections["imu_az"]["accel_bias"])
    for want, a in zip(MOTOR_STOPS, u_az):
        got = ig.el_abs_from_imu_az(a, sections["imu_az"]["M"])
        assert abs(got - abs(want)) < 1.0
    assert report["anchor"] == "imu_el"
    assert abs(report["home_offset_motor_deg"]) < 1.0
    assert len(report["cross_check"]) == len(MOTOR_STOPS)


def test_fit_el_calibration_imu_az_only_falls_back():
    el_az = _el_sweep_units(MOTOR_STOPS, M_AZ_TRUE) * 9.81
    sections, report = ig.fit_el_calibration(None, el_az, MOTOR_STOPS)
    assert set(sections) == {"imu_az"}
    assert report["anchor"] == "imu_az"


def test_fit_accel_sphere_coplanar_pins_null_component():
    bias = np.array([0.05, -0.02, 0.1])
    u = _el_sweep_units(MOTOR_STOPS, M_EL_TRUE)
    p = u * 9.81 + bias
    n = ig.fit_plane_normal(p)
    fit_bias, scale = ig.fit_accel_sphere_coplanar(p, n)
    assert abs(fit_bias @ n) < 1e-9
    bias_perp = bias - (bias @ n) * n
    assert np.allclose(fit_bias, bias_perp, atol=1e-9)
    assert scale == pytest.approx(9.81, abs=0.01)
