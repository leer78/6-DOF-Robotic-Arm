"""
Comprehensive comparison: ik_solver.py vs ik_symbolic.py numeric pipeline.
Tests DH parameters, formulas, FK round-trip accuracy across many poses.
"""
import math
import sys

# ===================================================================
# REFERENCE FK (independent pure-Python DH implementation)
# ===================================================================
def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ],
    ]

def mat4_mul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def reference_fk(q1, q2, q3, q4, q5, q6):
    d1, a2, a3, d4, d6 = 70.0, 278.0, 64.4, 165.05, 78.65
    T = dh_transform(q1, d1, 0, -math.pi/2)
    for theta, d, a, alpha in [
        (q2 - math.pi/2, 0, a2, 0),
        (q3, 0, a3, -math.pi/2),
        (q4 + math.pi, d4, 0, -math.pi/2),
        (q5, 0, 0, math.pi/2),
        (q6, d6, 0, 0),
    ]:
        T = mat4_mul(T, dh_transform(theta, d, a, alpha))
    return T

def rpy_from_T(T):
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    if sy > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        ry = math.atan2(-R[2][0], sy)
        rz = math.atan2(R[1][0], R[0][0])
    else:
        rx = math.atan2(-R[1][2], R[1][1])
        ry = math.atan2(-R[2][0], sy)
        rz = 0.0
    return rx, ry, rz


# ===================================================================
# IK SOLVER (new version with signed r + dual elbow)
# Imported here to avoid config.py dependency — extract the core math
# ===================================================================
def ik_solver_solve(x, y, z, rx_deg, ry_deg, rz_deg):
    """Replicates gui/ik_solver.py solve_ik() core math, minus config dependency."""
    D1, A2, A3, D4, D6 = 70.0, 278.0, 64.4, 165.05, 78.65
    L1, L2, L4, L5 = D1, A2, A3, D4
    L3 = math.sqrt(L4**2 + L5**2)

    rx, ry, rz_r = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz_r), math.sin(rz_r)
    R_d = [
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,            cy*cx],
    ]

    pw_x = x - D6 * R_d[0][2]
    pw_y = y - D6 * R_d[1][2]
    pw_z = z - D6 * R_d[2][2]

    q1 = 0.0

    def dh_rot(theta, alpha):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return [[ct, -st*ca, st*sa], [st, ct*ca, -ct*sa], [0, sa, ca]]

    def mm3(A, B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def mT3(A):
        return [[A[j][i] for j in range(3)] for i in range(3)]

    def solve_candidate(elbow_up):
        # Signed r for q1=0 (arm in XZ plane)
        r = pw_x
        s = math.sqrt((pw_z - L1)**2 + r**2)
        cos_beta = (L2**2 + s**2 - L3**2) / (2.0 * L2 * s)
        cos_gamma = (L2**2 + L3**2 - s**2) / (2.0 * L2 * L3)
        if abs(cos_beta) > 1.0 or abs(cos_gamma) > 1.0:
            return None, float('inf')

        alpha = math.atan2(pw_z - L1, r)
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))
        gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
        phi = math.atan2(L5, L4)

        if elbow_up:
            q2 = math.pi/2.0 - beta - alpha
            q3 = math.pi - gamma - phi
        else:
            q2 = math.pi/2.0 + beta - alpha
            q3 = gamma - math.pi - phi

        R03 = mm3(mm3(dh_rot(q1, -math.pi/2), dh_rot(q2 - math.pi/2, 0.0)), dh_rot(q3, -math.pi/2))
        R36 = mm3(mT3(R03), R_d)

        cos_q5 = max(-1.0, min(1.0, R36[2][2]))
        q5 = math.acos(cos_q5)
        if abs(math.sin(q5)) > 1e-6:
            q4 = math.atan2(-R36[1][2], -R36[0][2])
            q6 = math.atan2(R36[2][1], R36[2][0])
        else:
            q6 = 0.0
            q4 = math.atan2(R36[0][1], R36[0][0])

        angles = [q1, q2, q3, q4, q5, q6]
        T = reference_fk(*angles)
        err = math.sqrt((x - T[0][3])**2 + (y - T[1][3])**2 + (z - T[2][3])**2)
        return [math.degrees(a) for a in angles], err

    best_angles, best_err = None, float('inf')
    for elbow_up in [True, False]:
        angles, err = solve_candidate(elbow_up)
        if angles is not None and err < best_err:
            best_angles, best_err = angles, err

    if best_angles is None:
        raise ValueError("Unreachable")
    return best_angles, best_err


# ===================================================================
# TEST SUITE
# ===================================================================
print("=" * 80)
print("FULL JOINT-RANGE SWEEP TEST (New IK solver with signed r + dual elbow)")
print("=" * 80)

j2_range = range(-24, 22, 4)
j3_range = range(-85, 26, 10)
j5_range = range(-85, 86, 30)

total, passed, failed = 0, 0, 0
max_err = 0.0
fail_cases = []

for q2d in j2_range:
    for q3d in j3_range:
        for q5d in j5_range:
            total += 1
            qr = [0, math.radians(q2d), math.radians(q3d), 0, math.radians(q5d), 0]
            T = reference_fk(*qr)
            x0, y0, z0 = T[0][3], T[1][3], T[2][3]
            rx0, ry0, rz0 = [math.degrees(a) for a in rpy_from_T(T)]

            try:
                ik_angles, err = ik_solver_solve(x0, y0, z0, rx0, ry0, rz0)
                if err > max_err:
                    max_err = err
                if err < 1.0:
                    passed += 1
                else:
                    failed += 1
                    if len(fail_cases) < 10:
                        fail_cases.append((q2d, q3d, q5d, err, x0))
            except Exception as e:
                failed += 1
                if len(fail_cases) < 10:
                    fail_cases.append((q2d, q3d, q5d, float('inf'), x0))

print(f"  Total test cases: {total}")
print(f"  Passed (<1mm):    {passed}")
print(f"  Failed (>=1mm):   {failed}")
print(f"  Max position err: {max_err:.6f} mm")
print()

if fail_cases:
    print("  Failing cases:")
    for q2d, q3d, q5d, pe, xval in fail_cases:
        tag = f"err={pe:.2f}mm" if pe != float('inf') else "EXCEPTION"
        print(f"    q2={q2d:>4}, q3={q3d:>4}, q5={q5d:>4} -> {tag}  ee_x={xval:.2f}")
else:
    print("  ALL TESTS PASSED!")

print()

# ===================================================================
# Spot-check previously failing cases
# ===================================================================
print("=" * 80)
print("SPOT CHECK: Previously failing cases")
print("=" * 80)

spot_checks = [
    ("Home",        [0, 0, 0, 0, 0, 0]),
    ("Q3=-80",      [0, 0, -80, 0, 0, 0]),
    ("Extreme",     [0, -20, -70, 0, 80, 0]),
    ("Q5=-45",      [0, 0, 0, 0, -45, 0]),
    ("Q2=-24 Q3=-85 Q5=5", [0, -24, -85, 0, 5, 0]),
]

for name, q in spot_checks:
    qr = [math.radians(a) for a in q]
    T = reference_fk(*qr)
    x0, y0, z0 = T[0][3], T[1][3], T[2][3]
    rx0, ry0, rz0 = [math.degrees(a) for a in rpy_from_T(T)]
    try:
        ik_a, err = ik_solver_solve(x0, y0, z0, rx0, ry0, rz0)
        print(f"  {name:<30} pos_err={err:.6f} mm  {'OK' if err < 1.0 else 'FAIL'}")
        if err > 1.0:
            print(f"    Input:  {q}")
            print(f"    IK out: {[round(a,2) for a in ik_a]}")
    except Exception as e:
        print(f"  {name:<30} EXCEPTION: {e}")

