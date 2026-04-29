"""
Numeric Inverse Kinematics Solver for 6-DOF Arm

Pure Python implementation (no sympy) for real-time GUI use.
DH parameters imported from kinematics/ik_symbolic.py (single source of truth).

DH Table:
  i | theta_i     | d_i      | a_i   | alpha_i
  1 | q1          | 70       | 0     | -90°
  2 | q2 - 90°    | 0        | 278   |   0°
  3 | q3          | 0        | 64.4  | -90°
  4 | q4 + 180°   | 165.05   | 0     | -90°
  5 | q5          | 0        | 0     |  90°
  6 | q6          | 78.65    | 0     |   0°

Orientation convention: RPY (Roll-Pitch-Yaw), ZYX extrinsic = XYZ intrinsic
  R = Rz(yaw) * Ry(pitch) * Rx(roll)
"""

import math
import sys
import os
import config
from angle_mapping import get_logical_limits

# Import DH parameters from ik_symbolic.py (single source of truth)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinematics'))
from ik_symbolic import DH_PARAMS, GEOM_PARAMS

D1 = DH_PARAMS["d1"]
A2 = DH_PARAMS["a2"]
A3 = DH_PARAMS["a3"]
D4 = DH_PARAMS["d4"]
D6 = DH_PARAMS["d6"]

L1 = GEOM_PARAMS["l1"]
L2 = GEOM_PARAMS["l2"]
L4 = GEOM_PARAMS["l4"]   # a3 in DH = link 3 physical length
L5 = GEOM_PARAMS["l5"]   # d4 in DH = wrist offset along joint 4 axis
L3 = math.sqrt(L4**2 + L5**2)  # effective elbow-to-wrist distance


class IKError(Exception):
    """Raised when IK cannot find a valid solution."""
    pass


# ---------------------------------------------------------------------------
# 3x3 matrix helpers (pure Python, no numpy needed)
# ---------------------------------------------------------------------------

def _rpy_to_matrix(rx, ry, rz):
    """RPY angles (radians) to 3x3 rotation matrix. R = Rz * Ry * Rx."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,            cy*cx],
    ]


def _mat_mul(A, B):
    """Multiply two 3x3 matrices."""
    return [
        [sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_T(A):
    """Transpose 3x3 matrix."""
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _dh_rot(theta, alpha):
    """3x3 rotation block of a standard DH transform."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st*ca,  st*sa],
        [st,  ct*ca, -ct*sa],
        [0,   sa,     ca],
    ]


def _R0_3(q1, q2, q3):
    """Compute R0_3 = R1 * R2 * R3 from DH table (angles in radians)."""
    R1 = _dh_rot(q1,          -math.pi/2)
    R2 = _dh_rot(q2 - math.pi/2,  0.0)
    R3 = _dh_rot(q3,          -math.pi/2)
    return _mat_mul(_mat_mul(R1, R2), R3)


def _dh_transform(theta, d, a, alpha):
    """Full 4x4 DH homogeneous transform for FK verification."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ],
    ]


def _mat4_mul(A, B):
    """Multiply two 4x4 matrices."""
    return [
        [sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _wrap_to_pi(angle):
    """Wrap an angle in radians to [-pi, +pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _fk_position(q1, q2, q3, q4, q5, q6):
    """Compute FK end-effector position (x, y, z) for solution verification."""
    T = _dh_transform(q1, D1, 0, -math.pi/2)
    for theta, d, a, alpha in [
        (q2 - math.pi/2, 0, A2, 0),
        (q3, 0, A3, -math.pi/2),
        (q4 + math.pi, D4, 0, -math.pi/2),
        (q5, 0, 0, math.pi/2),
        (q6, D6, 0, 0),
    ]:
        T = _mat4_mul(T, _dh_transform(theta, d, a, alpha))
    return T[0][3], T[1][3], T[2][3]


def _solve_position_ik(pw_x, pw_y, pw_z, elbow_up=True):
    """
    Solve position IK for q2, q3 using geometric 2R planar solver.
    Uses signed r = pw_x (q1=0 means arm operates in XZ plane).
    Returns (q2, q3) in radians, or raises IKError.
    """
    # Signed horizontal reach (preserves direction for backward-reaching poses)
    r = pw_x
    if abs(r) < 1e-6 and abs(pw_y) < 1e-6:
        raise IKError("Shoulder singularity\n(wrist center on base axis)")

    s = math.sqrt((pw_z - L1)**2 + r**2)

    cos_beta = (L2**2 + s**2 - L3**2) / (2.0 * L2 * s)
    if abs(cos_beta) > 1.0:
        raise IKError(
            f"Unreachable position\n"
            f"Wrist distance: {s:.0f} mm\n"
            f"Arm reach: [{abs(L2 - L3):.0f}, {L2 + L3:.0f}] mm"
        )

    cos_gamma = (L2**2 + L3**2 - s**2) / (2.0 * L2 * L3)
    if abs(cos_gamma) > 1.0:
        raise IKError("Unreachable (elbow constraint)")

    alpha = math.atan2(pw_z - L1, r)
    beta  = math.acos(max(-1.0, min(1.0, cos_beta)))
    gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
    phi   = math.atan2(L5, L4)

    if elbow_up:
        q2 = math.pi / 2.0 - beta - alpha
        q3 = math.pi - gamma - phi
    else:
        q2 = math.pi / 2.0 + beta - alpha
        q3 = gamma - math.pi - phi

    return q2, q3


def _solve_orientation_ik(q1, q2, q3, R_d):
    """
    Solve orientation IK for q4, q5, q6.
    R_d is the desired 3x3 rotation matrix.
    Returns a list of candidate (q4, q5, q6) tuples in radians.
    """
    R03 = _R0_3(q1, q2, q3)
    R36 = _mat_mul(_mat_T(R03), R_d)

    cos_q5 = max(-1.0, min(1.0, R36[2][2]))
    q5_mag = math.acos(cos_q5)
    solutions = []

    if abs(math.sin(q5_mag)) > 1e-6:
        # Wrist branch 1: positive q5 (current behavior)
        q4 = math.atan2(-R36[1][2], -R36[0][2])
        q6 = math.atan2(R36[2][1], R36[2][0])
        solutions.append((_wrap_to_pi(q4), _wrap_to_pi(q5_mag), _wrap_to_pi(q6)))

        # Wrist branch 2: negative q5. The sign flip in sin(q5) must be
        # compensated in q4/q6 so the same R36 is reconstructed.
        q4_alt = math.atan2(R36[1][2], R36[0][2])
        q6_alt = math.atan2(-R36[2][1], -R36[2][0])
        solutions.append((_wrap_to_pi(q4_alt), _wrap_to_pi(-q5_mag), _wrap_to_pi(q6_alt)))
    else:
        # Wrist singularity (q5 ≈ 0 or π): q4 and q6 are coupled.
        q6 = 0.0
        q4 = math.atan2(R36[0][1], R36[0][0])
        solutions.append((_wrap_to_pi(q4), _wrap_to_pi(q5_mag), _wrap_to_pi(q6)))

    # Deduplicate numerically-equivalent branches.
    unique = []
    for cand in solutions:
        if not any(all(abs(a - b) < 1e-6 for a, b in zip(cand, seen)) for seen in unique):
            unique.append(cand)
    return unique


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def solve_ik(x, y, z, rx_deg, ry_deg, rz_deg):
    """
    Solve inverse kinematics for the 6-DOF arm.

    Tries both elbow-up and elbow-down configurations and picks the
    solution with the smallest FK position error (handles backward-reaching
    poses where the wrist center is behind the base).

    Args:
        x, y, z:               End-effector position in mm
        rx_deg, ry_deg, rz_deg: End-effector orientation in degrees (RPY / ZYX)

    Returns:
        dict with:
            'angles_deg': [q1..q6] in degrees (logical angles matching GUI sliders)
            'reachable':  True

    Raises:
        IKError with descriptive message if unreachable or joint limits violated
    """
    R_d = _rpy_to_matrix(
        math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    )

    # Step 1: Wrist center  p_w = p - d6 * R[:,2]
    pw_x = x - D6 * R_d[0][2]
    pw_y = y - D6 * R_d[1][2]
    pw_z = z - D6 * R_d[2][2]

    # Step 2: q1 = 0 (base joint is fixed)
    q1 = 0.0

    if abs(pw_y) > 10.0:
        raise IKError(f"Y = {pw_y:.1f} mm (must be ~0, J1 fixed)")

    # Step 3: Try both elbow configurations and both wrist branches.
    # Prefer an in-range solution when multiple FK-equivalent branches exist.
    best_angles = None
    best_err = float('inf')
    best_violation = float('inf')
    last_exc = None

    for elbow_up in [True, False]:
        try:
            q2, q3 = _solve_position_ik(pw_x, pw_y, pw_z, elbow_up=elbow_up)
            for q4, q5, q6 in _solve_orientation_ik(q1, q2, q3, R_d):
                # FK verification: check if the solution actually reaches the target
                fx, fy, fz = _fk_position(q1, q2, q3, q4, q5, q6)
                err = math.sqrt((x - fx)**2 + (y - fy)**2 + (z - fz)**2)

                angles_deg = [
                    math.degrees(_wrap_to_pi(a)) for a in [q1, q2, q3, q4, q5, q6]
                ]

                violation = 0.0
                for i in range(6):
                    jcfg = config.JOINTS[i] if i < len(config.JOINTS) else {}
                    if not jcfg.get("enabled", 0):
                        continue
                    try:
                        lo, hi = get_logical_limits(i)
                        a = angles_deg[i]
                        if a < lo:
                            violation += lo - a
                        elif a > hi:
                            violation += a - hi
                    except Exception:
                        pass

                if (violation < best_violation - 1e-9 or
                        (abs(violation - best_violation) <= 1e-9 and err < best_err)):
                    best_violation = violation
                    best_err = err
                    best_angles = [q1, q2, q3, q4, q5, q6]
        except IKError as e:
            last_exc = e

    if best_angles is None:
        raise last_exc or IKError("No valid IK solution found")

    if best_err > 1.0:
        raise IKError(
            f"IK solution inaccurate\n"
            f"FK position error: {best_err:.1f} mm"
        )

    angles_deg = [math.degrees(_wrap_to_pi(a)) for a in best_angles]

    # Step 4: Joint limit check (enabled joints only)
    violations = []
    for i in range(6):
        jcfg = config.JOINTS[i] if i < len(config.JOINTS) else {}
        if not jcfg.get("enabled", 0):
            continue
        try:
            lo, hi = get_logical_limits(i)
            a = angles_deg[i]
            if a < lo - 0.5 or a > hi + 0.5:
                violations.append(f"J{i+1}: {a:.1f}° outside [{lo:.1f}°, {hi:.1f}°]")
        except Exception:
            pass

    if violations:
        raise IKError("Joint limits violated:\n" + "\n".join(violations))

    return {"angles_deg": angles_deg, "reachable": True}
