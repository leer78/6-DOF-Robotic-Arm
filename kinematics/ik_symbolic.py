"""
Symbolic IK for 6-DOF arm (spherical wrist).
Computes T1, T2, T3 from DH parameters symbolically,
chains them into T0_3, and extracts R0_3.

DH Table (rows 1-3):
  i | theta_i     | d_i     | a_i   | alpha_i
  1 | q1          | 70      | 0     | -90 deg
  2 | q2 - 90 deg | 0       | 278   |   0 deg
  3 | q3 - 90 deg | 0       | 64.4  | -90 deg
"""

from sympy import (
    symbols, cos, sin, tan, atan2, acos, atan, sqrt, pi,
    Matrix, simplify, trigsimp, pprint, Rational
)

# ---------------------------------------------------------------------------
# Symbolic joint variables
# ---------------------------------------------------------------------------
q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6', real=True)

# ---------------------------------------------------------------------------
# Symbolic link parameters (d_i = offsets, a_i = link lengths)
# Default numeric values shown in comments for reference
# ---------------------------------------------------------------------------
d1, d4, d6 = symbols('d1 d4 d6', positive=True)   # d1=70, d4=165.05, d6=78.65
a2, a3     = symbols('a2 a3',    positive=True)   # a2=278, a3=64.4

# Substitution dict — use subs(link_values) on any expression to evaluate numerically
link_values = {
    d1: 70,
    d4: Rational(165046, 1000),   # 165.05
    d6: Rational(7865, 100),      # 78.65
    a2: 278,
    a3: Rational(644, 10),        # 64.4
}

# ---------------------------------------------------------------------------
# Geometric IK link lengths (user's naming convention)
# These are physical segment lengths used in the position solver.
#   l1 = base height to shoulder joint
#   l2 = upper arm length (= a2)
#   l4 = vertical offset of wrist from elbow (= d4)
#   l5 = horizontal offset of wrist from elbow (= a3)
#   l3 = effective combined third link  (derived)
# ---------------------------------------------------------------------------
l1, l2, l4, l5 = symbols('l1 l2 l4 l5', positive=True)

geom_values = {
    l1: 70,            # must match d1=70 from DH table (base height to shoulder)
    l2: 278,
    l4: Rational(644, 10),          # 64.4   = a3 (link 3 length)
    l5: Rational(16505, 100),       # 165.05 = d4 exactly (joint 4 z-offset)
}

# ---------------------------------------------------------------------------
# Wrist center position (symbolic inputs to the position IK)
#   pw_z : wrist center z-coordinate (world frame)
#   r    : horizontal distance from base z-axis to wrist center
#          (when q1=0 this equals pw_x)
# ---------------------------------------------------------------------------
pw_z, r = symbols('pw_z r', real=True)

# ---------------------------------------------------------------------------
# Position IK — geometric solution for q2
# ---------------------------------------------------------------------------
s      = sqrt((pw_z - l1)**2 + r**2)                    # shoulder-to-wrist distance
alpha  = atan((pw_z - l1) / r)                           # elevation angle to wrist center
l3     = sqrt(l4**2 + l5**2)                             # effective third link length
beta    = acos((l2**2 + s**2 - l3**2) / (2 * l2 * s))   # law of cosines angle at shoulder
q2_expr = pi/2 - beta - alpha                            # q2 solution

gamma   = acos((l2**2 + l3**2 - s**2) / (2 * l2 * l3)) # angle between l2 and l3
phi     = atan(l5 / l4)                                  # internal angle of l3 triangle
q3_expr = pi - gamma - phi                               # q3 solution

# ---------------------------------------------------------------------------
# DH transformation matrix builder
# Standard form: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
# ---------------------------------------------------------------------------
def dh_matrix(theta, d, a, alpha):
    """Returns the 4x4 homogeneous DH transform T = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)."""
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d           ],
        [0,           0,                      0,                     1           ]
    ])

# ---------------------------------------------------------------------------
# Individual transforms (degrees converted to radians via pi)
# ---------------------------------------------------------------------------

# Row 1: theta=q1, d=d1, a=0, alpha=-90 deg
T1 = dh_matrix(
    theta = q1,
    d     = d1,
    a     = 0,
    alpha = -pi/2
)

# Row 2: theta=q2-90deg, d=0, a=a2, alpha=0
T2 = dh_matrix(
    theta = q2 - pi/2,
    d     = 0,
    a     = a2,
    alpha = 0
)

# Row 3: theta=q3, d=0, a=a3, alpha=-90deg
T3 = dh_matrix(
    theta = q3,
    d     = 0,
    a     = a3,
    alpha = -pi/2
)

# ---------------------------------------------------------------------------
# Chain: T0_3 = T1 * T2 * T3
# ---------------------------------------------------------------------------
T0_3 = trigsimp(T1 * T2 * T3)

# ---------------------------------------------------------------------------
# Extract rotation matrix R0_3 (top-left 3x3)
# ---------------------------------------------------------------------------
R0_3 = T0_3[:3, :3]

# ---------------------------------------------------------------------------
# Desired end-effector pose
#   p = [px, py, pz]  — end-effector position
#   R  = 3x3 orientation matrix (symbolic entries r11..r33)
# ---------------------------------------------------------------------------
px, py, pz = symbols('px py pz', real=True)
p = Matrix([px, py, pz])

r11, r12, r13 = symbols('r11 r12 r13', real=True)
r21, r22, r23 = symbols('r21 r22 r23', real=True)
r31, r32, r33 = symbols('r31 r32 r33', real=True)

R = Matrix([
    [r11, r12, r13],
    [r21, r22, r23],
    [r31, r32, r33],
])

# Wrist center: strip off d6 along the end-effector z-axis (third column of R)
p_w = p - d6 * R[:, 2]   # p_w = p - d6 * [r13, r23, r33]

# ---------------------------------------------------------------------------
# R3_6 = (R0_3).T * R   (evaluated once q1,q2,q3 are substituted)
# ---------------------------------------------------------------------------
R3_6 = R0_3.T * R

# ---------------------------------------------------------------------------
# Wrist transforms: rows 4, 5, 6 from DH table
#   4 | q4 + 180 deg | d4 | 0 | -90 deg
#   5 | q5           |  0 | 0 |  90 deg
#   6 | q6           | d6 | 0 |   0 deg
# ---------------------------------------------------------------------------

# Row 4: theta=q4+180deg, d=d4, a=0, alpha=-90deg
T4 = dh_matrix(
    theta = q4 + pi,
    d     = d4,
    a     = 0,
    alpha = -pi/2
)

# Row 5: theta=q5, d=0, a=0, alpha=90deg
T5 = dh_matrix(
    theta = q5,
    d     = 0,
    a     = 0,
    alpha = pi/2
)

# Row 6: theta=q6, d=d6, a=0, alpha=0
T6 = dh_matrix(
    theta = q6,
    d     = d6,
    a     = 0,
    alpha = 0
)

# Chain wrist: rotation block of T4*T5*T6
T3_6_sym  = trigsimp(T4 * T5 * T6)
R3_6_sym  = T3_6_sym[:3, :3]

# ---------------------------------------------------------------------------
# Wrist angle extraction from R3_6_sym
# Equate R3_6_sym = R3_6 (numeric) and read off:
#   q5  from element [2,2]  :  cos(q5)
#   q4  from elements [0,2] and [1,2]  (valid when sin(q5) != 0)
#   q6  from elements [2,0] and [2,1]
# ---------------------------------------------------------------------------
q5_expr = acos(R3_6_sym[2, 2])                          # q5 = acos(r33)
q4_expr = atan2(-R3_6_sym[1, 2], -R3_6_sym[0, 2])       # atan2(-r23, -r13)
q6_expr = atan2( R3_6_sym[2, 1],  R3_6_sym[2, 0])       # atan2(r32, r31)

# ---------------------------------------------------------------------------
# Print results
# ---------------------------------------------------------------------------
print("=" * 60)
print("T1  (frame 0 -> 1)")
print("=" * 60)
pprint(T1)

print()
print("=" * 60)
print("T2  (frame 1 -> 2)")
print("=" * 60)
pprint(T2)

print()
print("=" * 60)
print("T3  (frame 2 -> 3)")
print("=" * 60)
pprint(T3)

print()
print("=" * 60)
print("T0_3 = T1 * T2 * T3  (frame 0 -> 3, simplified)")
print("=" * 60)
pprint(T0_3)

print()
print("=" * 60)
print("R0_3  (rotation block of T0_3)")
print("=" * 60)
pprint(R0_3)

print()
print("=" * 60)
print("Position IK — intermediate geometric quantities")
print("=" * 60)
print("s      ="); pprint(s)
print("alpha  ="); pprint(alpha)
print("l3     ="); pprint(l3)
print("beta   ="); pprint(beta)

print()
print("=" * 60)
print("q2_expr  (symbolic solution for q2)")
print("=" * 60)
pprint(q2_expr)

print()
print("=" * 60)
print("q3_expr  (symbolic solution for q3)")
print("=" * 60)
print("gamma  ="); pprint(gamma)
print("phi    ="); pprint(phi)
print("q3     ="); pprint(q3_expr)

print()
print("=" * 60)
print("Wrist center p_w = p - d6 * R[:,2]")
print("=" * 60)
pprint(p_w)

print()
print("=" * 60)
print("R3_6 = R0_3.T * R  (substitute q1,q2,q3 before using)")
print("=" * 60)
pprint(R3_6)

print()
print("=" * 60)
print("T4  (frame 3 -> 4)")
print("=" * 60)
pprint(T4)

print()
print("=" * 60)
print("T5  (frame 4 -> 5)")
print("=" * 60)
pprint(T5)

print()
print("=" * 60)
print("T6  (frame 5 -> 6)")
print("=" * 60)
pprint(T6)

print()
print("=" * 60)
print("R3_6_sym = rot(T4*T5*T6)  — entries in q4,q5,q6")
print("=" * 60)
pprint(R3_6_sym)

print()
print("=" * 60)
print("Wrist angle extraction  (equate R3_6_sym to numeric R3_6)")
print("=" * 60)
print("q5 = acos(R3_6[2,2])          :"); pprint(q5_expr)
print("q4 = atan2(-R3_6[1,2],-R3_6[0,2]) :"); pprint(q4_expr)
print("q6 = atan2( R3_6[2,1], R3_6[2,0]) :"); pprint(q6_expr)

# ---------------------------------------------------------------------------
# Numeric evaluation pipeline
#
# INPUT: desired end-effector pose as x, y, z (mm) + Rx, Ry, Rz (radians)
#        Euler convention: RPY = ZYX  (R = Rz * Ry * Rx)
#        This is the most common robotics convention.
#
# OUTPUT: q1..q6 in radians (and degrees), plus FK verification back to
#         x, y, z, Rx, Ry, Rz so you can confirm against your visualizer.
# ---------------------------------------------------------------------------
import math

def rpy_to_matrix(rx, ry, rz):
    """Convert Roll(Rx)-Pitch(Ry)-Yaw(Rz) angles to a 3x3 rotation matrix.
    Convention: R = Rz(rz) * Ry(ry) * Rx(rx)  (ZYX / extrinsic XYZ)
    """
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [ cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx ],
        [ sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx ],
        [-sy,     cy*sx,             cy*cx            ],
    ]

def matrix_to_rpy(R):
    """Extract Roll-Pitch-Yaw from a 3x3 rotation matrix (ZYX convention)."""
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2( R[2][1], R[2][2])
        ry = math.atan2(-R[2][0], sy)
        rz = math.atan2( R[1][0], R[0][0])
    else:
        rx = math.atan2(-R[1][2], R[1][1])
        ry = math.atan2(-R[2][0], sy)
        rz = 0.0
    return rx, ry, rz

# --- EDIT THESE: desired end-effector pose ---
x_d  = 400.0   # mm
y_d  = 0.0     # mm  (must be ~0 since q1=0 locks arm to XZ plane)
z_d  = 200.0   # mm
Rx_d = 0.0     # rad  (roll)
Ry_d = 0.0     # rad  (pitch)
Rz_d = math.pi # rad  (yaw) — use pi to get Rz≈0° in visualizer (known DH frame offset)
# ---------------------------------------------

R_n = rpy_to_matrix(Rx_d, Ry_d, Rz_d)
px_n, py_n, pz_n = x_d, y_d, z_d

# Step 1: wrist center  p_w = p - d6 * R[:,2]
d6_n   = float(link_values[d6])
pw_x   = px_n - d6_n * R_n[0][2]
pw_y   = py_n - d6_n * R_n[1][2]
pw_z_n = pz_n - d6_n * R_n[2][2]

print()
print("=" * 60)
print("NUMERIC PIPELINE")
print("=" * 60)
print(f"Input pose:   x={x_d}  y={y_d}  z={z_d}  Rx={Rx_d:.4f}  Ry={Ry_d:.4f}  Rz={Rz_d:.4f}  (rad)")
print(f"Wrist center: pw_x={pw_x:.4f}  pw_y={pw_y:.4f}  pw_z={pw_z_n:.4f}")

# Step 2: r = horizontal reach to wrist center (pw_y must be ~0 for q1=0 arm)
r_n = math.sqrt(pw_x**2 + pw_y**2)
if abs(pw_y) > 1.0:
    print(f"WARNING: pw_y={pw_y:.4f} is non-zero. Pose may be unreachable with q1=0.")
print(f"r = {r_n:.4f} mm")

# Step 3: solve q2 and q3
gv = {k: float(v) for k, v in geom_values.items()}  # use geom_values (l1=70, l2=278, l4=64.4, l5=165.05)
q2_n = float(q2_expr.subs(gv).subs({pw_z: pw_z_n, r: r_n}))
q3_n = float(q3_expr.subs(gv).subs({pw_z: pw_z_n, r: r_n}))
print(f"q2 = {q2_n:.6f} rad  ({math.degrees(q2_n):.4f} deg)")
print(f"q3 = {q3_n:.6f} rad  ({math.degrees(q3_n):.4f} deg)")

# Step 4: numeric R0_3
R0_3_n = R0_3.subs({q1: 0, q2: q2_n, q3: q3_n})
R0_3_n = R0_3_n.applyfunc(lambda x: float(x.evalf()))

# Step 5: numeric R3_6 = R0_3.T * R_desired
R_desired_mat = Matrix(R_n)
R3_6_n = R0_3_n.T * R_desired_mat

# Step 6: extract q4, q5, q6
q5_n   = math.acos(max(-1.0, min(1.0, float(R3_6_n[2, 2]))))   # positive branch preferred
sin_q5 = math.sin(q5_n)
if abs(sin_q5) > 1e-6:
    q4_n = math.atan2(-float(R3_6_n[1, 2]), -float(R3_6_n[0, 2]))
    q6_n = math.atan2( float(R3_6_n[2, 1]),  float(R3_6_n[2, 0]))
else:
    print("WARNING: wrist singularity (q5 ~ 0 or pi). Setting q6=0.")
    q6_n = 0.0
    q4_n = math.atan2(float(R3_6_n[0, 1]), float(R3_6_n[0, 0]))

print(f"q4 = {q4_n:.6f} rad  ({math.degrees(q4_n):.4f} deg)")
print(f"q5 = {q5_n:.6f} rad  ({math.degrees(q5_n):.4f} deg)")
print(f"q6 = {q6_n:.6f} rad  ({math.degrees(q6_n):.4f} deg)")

# ---------------------------------------------------------------------------
# JOINT ANGLES SUMMARY  — plug these into your visualizer
# ---------------------------------------------------------------------------
print()
print("=" * 60)
print("JOINT ANGLES  (plug into visualizer)")
print("=" * 60)
print(f"  q1 = 0.000000 rad  (  0.0000 deg)  [fixed]")
print(f"  q2 = {q2_n:.6f} rad  ({math.degrees(q2_n):.4f} deg)")
print(f"  q3 = {q3_n:.6f} rad  ({math.degrees(q3_n):.4f} deg)")
print(f"  q4 = {q4_n:.6f} rad  ({math.degrees(q4_n):.4f} deg)")
print(f"  q5 = {q5_n:.6f} rad  ({math.degrees(q5_n):.4f} deg)")
print(f"  q6 = {q6_n:.6f} rad  ({math.degrees(q6_n):.4f} deg)")

# ---------------------------------------------------------------------------
# FK VERIFICATION — compute T0_6 from solved joints, recover x,y,z,Rx,Ry,Rz
# This should match your input pose. Compare with what the visualizer shows.
# ---------------------------------------------------------------------------
lv = link_values   # numeric link substitution dict
T0_6_n = (T1 * T2 * T3 * T4 * T5 * T6).subs(lv).subs({
    q1: 0, q2: q2_n, q3: q3_n, q4: q4_n, q5: q5_n, q6: q6_n
}).evalf()

fk_x  = float(T0_6_n[0, 3])
fk_y  = float(T0_6_n[1, 3])
fk_z  = float(T0_6_n[2, 3])
fk_R  = [[float(T0_6_n[i, j]) for j in range(3)] for i in range(3)]
fk_rx, fk_ry, fk_rz = matrix_to_rpy(fk_R)

print()
print("=" * 60)
print("FK VERIFICATION  (should match input pose)")
print("=" * 60)
print(f"  FK  x={fk_x:.4f}  y={fk_y:.4f}  z={fk_z:.4f}")
print(f"  IN  x={x_d:.4f}  y={y_d:.4f}  z={z_d:.4f}")
print(f"  FK  Rx={fk_rx:.6f}  Ry={fk_ry:.6f}  Rz={fk_rz:.6f}  (rad)")
print(f"  IN  Rx={Rx_d:.6f}  Ry={Ry_d:.6f}  Rz={Rz_d:.6f}  (rad)")
err_pos = math.sqrt((fk_x-x_d)**2 + (fk_y-y_d)**2 + (fk_z-z_d)**2)
print(f"  Position error: {err_pos:.6f} mm  ({'OK' if err_pos < 0.1 else 'CHECK'})")
