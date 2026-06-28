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
