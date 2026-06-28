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
