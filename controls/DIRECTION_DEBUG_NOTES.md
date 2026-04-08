# Stepper Motor Direction — Debug History & Reference

This document records all direction-related bugs found and fixed in the stepper
motor control system. Refer to it whenever a motor moves the wrong way or a new
joint is added.

---

## Architecture Overview

The direction of a stepper motor is determined by a chain of three layers:

```
PID error sign  →  cmdPulsesPerS sign  →  ISR DIR pin  →  physical rotation
```

1. **PID error** = `targetAngle − encoderAngle` (wrap-safe, ±180°)
2. **PID output** converts error to a signed pulse-rate command.
3. **ISR** maps the command sign to a DIR pin level, optionally flipped by
   `MOTOR_DIR_INVERT[j]`.
4. **Physical motor + driver wiring** determines which DIR level increases the
   raw encoder angle.

A bug in *any* layer causes the motor to move the wrong way. Below are the three
bugs we found and fixed, in the order they were discovered.

---

## Bug 1: Arduino PID Library Sign Inversion

**Symptom:** Positive error → negative PID output → motor moves away from target.

**Root cause:** We use an "error-based" PID formulation where `pidInput = error`
and `pidSetpoint = 0`. The Arduino PID_v1 library internally computes:

```
output = Kp × (setpoint − input) = Kp × (0 − error) = −Kp × error
```

This **negates** the error. A positive error (need to increase angle) produces a
negative output (motor decreases angle).

**Fix (controls.ino, modeMoveCode):**

```cpp
// BEFORE (broken):
pidInput[j] = (double)(error);

// AFTER (correct):
pidInput[j] = (double)(-error);
// Library computes: Kp × (0 − (−error)) = +Kp × error  ✓
```

**Side-effect:** Joint 2 previously had `MOTOR_DIR_INVERT = true` which
accidentally compensated for this sign bug. After the fix, J2 INVERT was
changed back to `false`.

**Date fixed:** 2026-04-07

---

## Bug 2: Per-Joint MOTOR_DIR_INVERT (Physical Wiring Mismatch)

**Symptom:** After Bug 1 was fixed, PID output sign was correct, but J3/J4/J5
still moved the wrong way. J2 worked fine.

**Root cause:** The TB6600 driver wiring for J3, J4, and J5 is physically
reversed compared to J2. When DIR = HIGH, these motors *decrease* raw encoder
angle (opposite of what the code assumes).

**Diagnosis method:** PID debug telemetry showed:
- J5: positive error → positive cmd → DIR = HIGH → raw angle **decreased**
- Confirmed with multiple commands in both directions

**Fix (controls_config.h):**

```cpp
// BEFORE:
static const bool MOTOR_DIR_INVERT[NUM_STEPPERS] = { false, false, false, false, false };

// AFTER:
static const bool MOTOR_DIR_INVERT[NUM_STEPPERS] = { false, false, true,  true,  true  };
//                                                     J1     J2     J3     J4     J5
```

**How MOTOR_DIR_INVERT works in the ISR:**

```cpp
bool desired_fwd = (cmd > 0);
if (MOTOR_DIR_INVERT[j]) desired_fwd = !desired_fwd;
// desired_fwd = true  → DIR pin HIGH
// desired_fwd = false → DIR pin LOW
```

Setting INVERT = true flips the mapping so positive cmd → DIR LOW → raw
increases (correct for these motors).

**How to determine INVERT for a new joint:** Send a known positive PID command
and observe encoder direction. If raw angle decreases, set INVERT = true.

**Date fixed:** 2026-04-07

---

## Bug 3: ISR `cur_dir_forward` Not Initialized (First-Move-Wrong-Direction)

**Symptom:** The very first JOINTS_TO_ANGLE command after entering Move mode
(power-up) moves the motor the wrong way. Subsequent commands work correctly.

**Root cause:** The ISR state struct is zero-initialized at static scope:

```cpp
static StepperISRState isrState[NUM_STEPPERS];  // zero-initialized
// → cur_dir_forward = false (0) for all joints
```

But `setup()` sets the physical DIR pin to HIGH:

```cpp
digitalWriteFast(DIR_PINS[j], HIGH);
```

This creates a **mismatch**: the ISR thinks direction is LOW (`cur_dir_forward
= false`) but the physical pin is HIGH.

When the first non-zero PID command arrives (e.g., positive cmd for J5 with
INVERT = true):

```
desired_fwd = true → INVERT → false
cur_dir_forward = false  (zero-init, stale)
false == false → NO direction change triggered!
DIR pin stays HIGH (from setup) → motor steps wrong way
```

On a subsequent command with the opposite sign, `desired_fwd` differs from
`cur_dir_forward`, a direction change fires, and the ISR syncs with the physical
pin. **From that point on, everything works.**

**Fix (controls.ino, setup):**

```cpp
digitalWriteFast(DIR_PINS[j], HIGH);
isrState[j].cur_dir_forward = true;   // Sync ISR state with physical DIR=HIGH pin
```

**Why it only affected the first command:** After any direction-change event,
`cur_dir_forward` and the DIR pin are always in sync. The bug only manifests
when the ISR has *never* performed a direction change since power-up.

**Date fixed:** 2026-04-08

---

## Quick Diagnostic Checklist

If a motor moves the wrong way, check these in order:

| # | Check | How to verify |
|---|-------|---------------|
| 1 | **PID sign** | PID debug telemetry: positive error should give positive `J*_OUT` |
| 2 | **MOTOR_DIR_INVERT** | PID debug: positive cmd + correct DIR level should increase raw angle |
| 3 | **ISR init state** | Only fails on first command after power-up; subsequent commands work |
| 4 | **Encoder wiring** | Raw angle should increase smoothly when motor steps in "forward" direction |
| 5 | **Angle wrapping** | Check if target−current crosses the 0°/360° boundary (shortest-path logic) |

## PID Debug Telemetry Reference

Added to aid direction debugging. Sent every 5th telemetry cycle in MOVE mode.

```
TYPE=DATA,CMD=PID_DEBUG,J2_ERR=...,J2_OUT=...,J2_CMD=...,J2_TGT=...,J2_DIR=L,...
```

| Field | Meaning |
|-------|---------|
| `ERR` | Signed error in degrees (target − current, wrap-safe) |
| `OUT` | Raw PID output (before deadband/clamp) |
| `CMD` | Actual `cmdPulsesPerS` sent to ISR (after deadband/clamp) |
| `TGT` | Current target angle (raw encoder degrees) |
| `DIR` | Current DIR pin state: `H` = HIGH, `L` = LOW |

**Timing note:** `sendPidDebug()` runs BEFORE `modeMoveCode()` in the main
loop, so values are from the *previous* PID cycle. This can cause a one-cycle
lag between reported values and actual motor behavior.

---

## Current Configuration (as of 2026-04-08)

```
Joint:          J1     J2     J3     J4     J5
INVERT:       false  false   true   true   true
Microsteps:      4      8      4      4      4
Kp:             10     10      5     15      5
Ki:              0      0      0      0      0
Kd:              0      0      0      0      0
Deadband(°):  0.75   0.75   0.75   0.75   0.75
Min speed:      10     10     10     10     10  (full-steps/s)
Max speed:     120    120    120    120    120  (full-steps/s)
```
