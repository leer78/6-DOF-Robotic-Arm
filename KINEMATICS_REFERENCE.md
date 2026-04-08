# Kinematics Reference — 6-DOF Robotic Arm

## Robot Classification

This arm is a **PUMA-type robot missing the base rotation joint (J1)**.

The PUMA 560 has the joint order: Base → Shoulder → Elbow → Forearm Roll → Wrist Pitch → Wrist Roll.
This arm is identical except J1 (base) is **fixed** — the arm cannot sweep left/right. All motion occurs in a single fixed vertical plane for the positioning chain, with full 3D orientation capability at the wrist.

Search term for math derivations: **"PUMA 560 inverse kinematics"** — then substitute θ₁ = 0 everywhere.

---

## Physical Joint Layout

```
[Fixed Base — J1 not built]
        │
      [J2]  Shoulder Pitch       — stepper, AS5600 encoder (mux ch 3)
        │
      [J3]  Elbow Pitch          — stepper, AS5600 encoder (mux ch 4)
        │
      [J4]  Forearm Roll         — stepper, NO encoder (pins not wired yet)
        │
      [J5]  Wrist Pitch          — stepper, AS5600 encoder (mux ch 6)
        │
      [J6]  Wrist Roll           — 25kg digital servo (pin 39), open-loop
        │
     [Gripper]                   — SG90 servo (pin 38), open-loop
```

### What each joint does

| Joint | Type | Axis of Rotation | Effect |
|-------|------|-----------------|--------|
| J2 | Shoulder pitch | Horizontal, perpendicular to arm plane | Raises/lowers the whole arm |
| J3 | Elbow pitch | Same as J2 | Folds/extends the forearm |
| J4 | Forearm roll | Along the forearm's long axis | Spins the wrist assembly like a screwdriver |
| J5 | Wrist pitch | Perpendicular to forearm | Tips the tool forward/backward |
| J6 | Wrist roll | Along the wrist axis (after J5) | Final spin / gripper alignment |

---

## Kinematic Groupings

### Positioning Chain: J2 + J3

J2 and J3 are both pitch joints rotating about the same axis direction (horizontal, in-plane). Together they form a **2R planar chain** that positions the **wrist center** anywhere in the reachable arc of the vertical plane. No left/right sweep is possible since J1 is absent.

### Wrist Orientation Chain: J4 + J5 + J6 (Roll-Pitch-Roll)

The wrist is a **spherical wrist** in a **Roll-Pitch-Roll (RPR)** configuration. This gives full 3D orientation of the tool, even though the arm itself is confined to a vertical plane.

- **J4** sets up *which direction* J5 will pitch into. On its own it just spins the forearm. Its value determines the plane in which J5 operates.
- **J5** tips the tool in whatever direction J4 set up.
- **J6** corrects the roll after J5 has moved. Fine-tunes final gripper alignment.

> **Note on redundancy:** J4 and J6 are only redundant when J5 = 0°. At J5 = 0°, both rotate about the same axis and are effectively combined. This is the wrist singularity. When J5 ≠ 0°, all three joints are doing distinct work.

---

## Physical Assumptions and Notes

### θ₁ = 0 (No base joint)
J1 is never built. Treat as a constant θ₁ = 0 throughout all IK math. Terms involving θ₁ in the PUMA derivation collapse to cos(0)=1, sin(0)=0.

### d₃ = 0 (No elbow offset)
The PUMA 560 has a perpendicular offset `d₃` between the shoulder and elbow axes (visible as the elbow "sticking out" to the side). **This arm does not have that offset** — J2 and J3 are inline in the same plane. Set `d₃ = 0` in the DH table. Terms with `d₃` in the PUMA derivation drop out entirely.

### J3 link offset (TO BE MEASURED)
Link 3 (the forearm, between J3 and J4) attaches to J3 **above** the J3 rotation axis by a physical displacement. This must be measured and captured as the appropriate DH parameter (`d₃` or `a₃` depending on direction). **Do not leave this as zero** — even a 1–2 cm offset causes noticeable tool-tip error.

---

## DH Parameter Table

Fill in `a` (link length) and `d` (offset) values by physically measuring the arm.
`α` (link twist) values are fixed by the joint axis directions.

| Joint | θ (variable) | a (link length) | d (offset) | α (twist) | Notes |
|-------|-------------|-----------------|------------|-----------|-------|
| J2 | θ₂ | L₂ = **?? mm** | 0 | 0° | Upper arm length |
| J3 | θ₃ | L₃ = **?? mm** | d₃_offset = **?? mm** | 90° | Forearm; also has perpendicular mount offset (see note above) |
| J4 | θ₄ | 0 | d₄ = **?? mm** | -90° | Wrist roll; d₄ is axial distance to wrist center |
| J5 | θ₅ | 0 | 0 | 90° | Wrist pitch |
| J6 | θ₆ | 0 | d₆ = **?? mm** | 0° | Tool offset from wrist center to gripper tip |

> **α convention:** α is the angle between consecutive Z axes, measured about the X axis. The 90°/-90° pattern here is standard for a PUMA-type wrist and causes the RPR Euler decomposition to work cleanly.

---

## Forward Kinematics (FK)

Each joint contributes one 4×4 homogeneous transform in the DH convention:

```
Tᵢ₋₁,ᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)

     = [ cos θᵢ   -sin θᵢ·cos αᵢ    sin θᵢ·sin αᵢ    aᵢ·cos θᵢ ]
       [ sin θᵢ    cos θᵢ·cos αᵢ   -cos θᵢ·sin αᵢ    aᵢ·sin θᵢ ]
       [    0          sin αᵢ            cos αᵢ            dᵢ    ]
       [    0             0                 0               1     ]
```

Full FK chain:
```
T₀₆ = T₀₂ · T₂₃ · T₃₄ · T₄₅ · T₅₆
```

The tool-tip position is the top-right 3×1 column of T₀₆.
The tool orientation is the top-left 3×3 rotation matrix of T₀₆.

---

## Inverse Kinematics (IK) — Algorithm Overview

### Input
```
target position:    (y, z)             — in the arm's vertical plane (no x since J1=0)
target orientation: R_desired (3×3)    — desired rotation matrix of the tool
```

### Step 1 — Compute Wrist Center
Back off from the tool tip by `d₆` along the tool's approach direction (last column of R_desired):

```
P_wc = P_tool - d₆ · R_desired[:, 2]
```

### Step 2 — Solve J3 (Elbow)
Use the law of cosines on the 2R chain (J2, J3):

```
cos(θ₃) = (y_wc² + z_wc² - L₂² - L₃²) / (2·L₂·L₃)

# Check: if |cos(θ₃)| > 1 → point is out of reach, abort

θ₃ = ± arccos(cos(θ₃))    # + = elbow down,  − = elbow up (two solutions)
```

### Step 3 — Solve J2 (Shoulder)
```
θ₂ = atan2(z_wc, y_wc) - atan2(L₃·sin(θ₃),  L₂ + L₃·cos(θ₃))
```

### Step 4 — Compute R₀₃
Compute the rotation achieved by J2 and J3 using FK with the angles just found:

```
R₀₃ = R₀₂(θ₂) · R₂₃(θ₃)
```

### Step 5 — Compute Remaining Rotation for Wrist
```
R₃₆ = R₀₃ᵀ · R_desired
```

### Step 6 — Extract Wrist Angles (RPR Euler Decomposition)
```
θ₅ = atan2( sqrt(R₃₆[0,2]² + R₃₆[1,2]²),  R₃₆[2,2] )   # wrist pitch
θ₄ = atan2( R₃₆[1,2],  R₃₆[0,2] )                        # forearm roll
θ₆ = atan2( R₃₆[2,1], -R₃₆[2,0] )                        # wrist roll
```

> **Note:** Exact signs and indices depend on the DH convention used. Verify against FK once implemented.

---

## Multiple Solutions

The IK has up to **4 valid solutions** for any reachable pose:
- Elbow up / Elbow down (sign of θ₃)
- Wrist flip (alternate RPR solution with θ₄ ± 180°, -θ₅, θ₆ ± 180°)

When multiple solutions exist, pick the one that:
1. Passes all joint limit checks
2. Avoids singularities
3. Requires the least total joint movement from current position

---

## Singularities

### Wrist Singularity (J5 ≈ 0°)
When J5 = 0°, J4 and J6 rotate about the same axis → infinite solutions exist for J4/J6, and small changes in target orientation cause large joint motions.

**Detection:** `if abs(θ₅) < threshold (e.g. 1°)`
**Handling:** Reject pose or clamp J5 away from 0°.

### Elbow Singularity (arm fully extended or folded)
When the wrist center lies exactly at maximum or minimum reach, `cos(θ₃) = ±1`.

**Detection:** `val = (y_wc² + z_wc² - L₂² - L₃²) / (2·L₂·L₃); if |val| > 1`
**Handling:** Reject pose as out of reach.

### Shoulder Singularity (wrist center at J2 axis)
When `y_wc ≈ 0` and `z_wc ≈ 0`, `atan2(0, 0)` is undefined.

**Detection:** `if sqrt(y_wc² + z_wc²) < threshold`
**Handling:** Reject pose.

---

## Joint Limits (from `gui/config.py`)

These are raw encoder ranges, converted to logical angles using the calibration mapping. Verify these against physical hard stops.

| Joint | Logical Min (approx) | Logical Max (approx) | Notes |
|-------|---------------------|---------------------|-------|
| J2 | ~−24.3° | ~+21.4° | ref=301.9° raw → 0° logical (direction=-1) |
| J3 | ~−25.9° | ~+85.4° | ref=203.4° raw → 0° logical (direction=-1) |
| J4 | n/a | n/a | Not wired yet |
| J5 | ~−85.4° | ~+86.3° | ref=83.8° raw → 0° logical (direction=-1) |
| J6 | 60° | 180° | Servo, open-loop |

---

## IK Validation Checklist (run after every IK solve)

```python
def validate_ik(angles):
    # 1. Joint limits
    for angle, (lo, hi) in zip(angles, JOINT_LIMITS):
        if not (lo <= angle <= hi):
            return False, "JOINT_LIMIT"

    # 2. Wrist singularity
    if abs(angles[J5]) < 1.0:  # degrees
        return False, "WRIST_SINGULARITY"

    # 3. Self-collision (basic — refine later)
    fk_positions = forward_kinematics(angles)
    if fk_positions['any_link_z'] < BASE_HEIGHT:
        return False, "SELF_COLLISION"

    return True, "OK"
```

---

## Implementation Plan

IK **must be implemented on the PC side (Python, in `gui/`)**, not on the Teensy.

**Rationale:**
- Teensy does joint-space PID control only (by design)
- PC has full floating point, numpy, easy debugging
- The existing `JOINTS_TO_ANGLE` serial command is the exact interface needed
- No firmware changes required to add IK

**Suggested file:** `gui/ik_solver.py`

**Data flow:**
```
User specifies target pose (y, z, roll, pitch, yaw)
        ↓
ik_solver.py  →  [θ₂, θ₃, θ₄, θ₅, θ₆]
        ↓
serial_protocol.py  →  JOINTS_TO_ANGLE packet
        ↓
Teensy PID controller tracks each joint angle
```

---

## Current Codebase State (as of implementation start)

- ✅ Joint-space PID control working (J2, J3, J5 with encoders)
- ✅ J4 stepper wired in firmware but encoder not connected
- ✅ J6 servo open-loop via `GRIP_CNTL` command
- ✅ Serial protocol (`JOINTS_TO_ANGLE`) ready to accept computed angles
- ❌ No FK implemented
- ❌ No IK implemented
- ❌ No DH parameters measured or defined
- ❌ J4 encoder not wired (IK will be open-loop for J4 until resolved)
