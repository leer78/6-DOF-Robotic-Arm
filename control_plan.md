# Control Plan (Stepper + AS5600 + Teensy 4.1)

This document captures the agreed control approach for moving each joint’s stepper motor to a desired **absolute joint angle** (single-turn, ≤ 360°) using **AS5600 encoder feedback** and a **TB6600 step/dir driver**.

## System summary
- **Controller:** Teensy 4.1
- **Sensing:** AS5600 absolute encoder(s) on a TCA9548A I2C mux (same I2C address, mux selects channel)
- **Actuation:** Stepper motor(s) driven by TB6600 modules using `STEP`, `DIR`, and per-joint `ENABLE`.
- **Additional actuators:** hobby servos (Joint 6 servo + SG90 gripper)
- **Commanding:** PC GUI sends **raw encoder-space target angles** (degrees in [0, 360)) to Teensy.

## Assumptions (matches current firmware)
- **Encoder read path:** reuse the same approach already present in `controls.ino`: select a TCA9548A channel, call `encoder.readAngle()` (0..4095), convert via `RAW_TO_DEG = 360/4096` to get degrees in [0, 360).
- **Single-turn joints:** joints are constrained to ≤ 360° mechanical travel, but we still must handle the 0/360 wrap correctly.

## Key conclusion: what we “control”
TB6600 does not accept a numeric “speed command” over a bus. It only accepts:
- `DIR` = direction (CW/CCW)
- `STEP` pulses = each rising edge advances one microstep

Therefore:
- **step rate (steps/s)** is the real control input,
- which maps to **joint velocity** via `steps_per_degree`.

## Recommended control structure (per joint)
### 1) Split into two time scales
1. **Pulse generation (fast, continuous, non-blocking):**
  - A timer/ISR continuously generates `STEP` edges at the currently commanded step frequency.
  - This must run even if the main loop is busy (I2C reads, parsing commands, telemetry).

2. **Control update (slower, fixed-period):**
   - Runs at a known sample time $T_s$ (typical starting point: **100–200 Hz**).
   - Reads encoder angle(s), computes error, updates step-rate commands.

### 2) Position error → velocity (step-rate) control
- Read actual encoder angle $\theta$ (deg)
- Setpoint is desired raw angle $\theta_{des}$ (deg)
- Compute shortest signed angular error (wrap-safe):
  - $e = \mathrm{wrap}_{[-180,180)}(\theta_{des} - \theta)$
- Apply deadband:
  - If $|e| \le e_{db}$: command speed = 0 (and typically keep driver enabled for holding torque)
- Controller output is a **signed velocity command** (deg/s), start with P-only:
  - $\omega_{cmd} = K_p \cdot e$
- Convert to step rate:
  - $f_{step} = \omega_{cmd} \cdot (\text{steps/deg})$
- Clamp to safe limits:
  - $|f_{step}| \le f_{max}$

### 2a) Wrap-safe shortest-path helper (required)
Always compute error using the minimal signed rotation so targets near 0/360 don’t command a long way around.

Reference implementation (degrees):
- While $e \ge 180$: $e -= 360$
- While $e < -180$: $e += 360$

Then use the sign of $e$ (or $\omega_{cmd}$) as direction.

### 2b) Deadband and integral windup
- Start with **P-only** for initial bring-up: set $K_i = 0, K_d = 0$.
- If you later add integral action, you must prevent the integrator from “winding up” while:
  - the motor is stopped by deadband, or
  - the output is clamped (speed saturation), or
  - the joint is disabled.

Practical anti-windup policies (choose one):
- **Freeze integration** when in deadband / disabled / saturated (preferred).
- **No-Compute-in-deadband:** skip PID compute and force output = 0 while in deadband.
- **Conditional integration:** only integrate when output is not saturated and error is outside deadband.

### 3) Holding heavy joints / fighting gravity
- “Hold” on a stepper system means: **driver enabled + zero step pulses**.
- For gravity-loaded joints, keep `ENABLE` asserted at rest to provide holding torque.
- Use the encoder loop as a supervisor: if the joint sags and error exceeds a threshold, re-apply corrective stepping.

## Why fixed-period control updates (not every `loop()`)
- `loop()` timing is non-deterministic (varies with I2C delays, serial traffic, logging).
- A controller should run with a predictable sample time $T_s$.
- Pulse generation must be continuous; control updates just adjust the pulse generator’s commanded frequency.

## Configuration values that matter
- **Microstepping** (TB6600 DIP settings) matters for conversion:
  - $\text{steps/deg} = \dfrac{200 \cdot \text{microsteps} \cdot \text{gear ratio}}{360}$
- Each joint should have:
  - `steps_per_degree`
  - `max_steps_per_s`
  - `Kp` (and optional `Ki`, `Kd` later)
  - `deadband_deg`
  - `enable_active_level` (depends on wiring)

## Implementation sketch (conceptual)
### Stepper pulse engine (Timer ISR) — chosen approach
We will use **one high-rate timer** (Teensy `IntervalTimer`) that services all stepper joints.

Key idea:
- The ISR runs at a constant tick rate `tick_hz` (typical: **20–60 kHz**).
- Each ISR call is a very short “scheduler tick”. It may emit **zero or one** step pulse per motor on that tick.
- The desired step rate is maintained by a per-motor accumulator.

Per-motor state (conceptual):
- `cmd_steps_per_s` (signed, updated by control loop)
- `dir_pin`, `step_pin`, `en_pin`
- `accum` (fixed-point phase accumulator)
- `step_high` (whether STEP is currently high and needs to be brought low)
- optional: `dir_guard_ticks` (ticks to wait after changing DIR)

ISR behavior on each tick:
1. For each motor:
   - If `step_high` is true: drive STEP low and clear `step_high` (this guarantees a minimum pulse width).
   - Otherwise, accumulate desired steps: `accum += abs(cmd_steps_per_s)` scaled by `tick_hz`.
   - If accumulator crosses 1 step (fixed-point threshold):
     - Ensure DIR pin matches sign of `cmd_steps_per_s`.
     - If DIR just changed, wait `dir_guard_ticks` before stepping.
     - Drive STEP high, set `step_high = true`, and subtract one-step threshold from `accum`.

Notes / requirements:
- Keep ISR extremely short: no I2C, no Serial, no floating-point, no prints.
- Use `digitalWriteFast()` (or direct GPIO) for STEP/DIR/EN.
- Choose `tick_hz` such that STEP pulse width meets TB6600 requirements. A simple pattern is:
  - STEP high for 1 tick, low for at least 1 tick.
  - At `tick_hz = 40 kHz`, one tick is 25 µs (comfortably above typical minimum pulse width).
- With the “high-then-low” scheme above, the theoretical maximum step rate per motor is approximately:
  - $f_{step,max} \approx \dfrac{tick\_hz}{2}$
  - (because a full pulse consumes at least 2 ticks: one to go high, one to return low)
- If you later need very high step rates, you can increase `tick_hz` or implement a different pulse scheme; start simple.

### Multi-axis simultaneity (5 steppers at once)
This plan is explicitly intended to run **all 5 stepper joints simultaneously**.

Important clarifications:
- “Simultaneous motion” means all joints can have independent nonzero `cmd_steps_per_s` at the same time, and the ISR will emit steps for each as needed.
- “Exactly the same time” at the nanosecond level is not required for a robotic arm. The goal is **reasonably simultaneous** multi-joint motion.
- The ISR loops motors sequentially, so two motors that step on the same tick may be separated by a few microseconds (pin-write time). This is acceptable for independent joint control.
- If you ever need tighter alignment for two or more STEP edges on the same tick, we can switch the ISR to a two-phase approach:
  - First compute which motors should step this tick.
  - Then set multiple STEP pins high using a single GPIO register write (and later clear them similarly).
  - This is an optimization, not required for initial success.

### Foreground-to-ISR data safety
- `cmd_steps_per_s[j]` is shared between foreground and ISR and must be `volatile`.
- Update shared commands atomically (brief interrupt disable around writes) to avoid torn reads.
- Avoid changing multiple related fields (DIR + rate + enable) without an atomic update rule.

### Control task @ 100–200 Hz (foreground)
Run a fixed-period control update that:
1. reads encoder angle(s) (through TCA9548A mux)
2. computes wrap-safe error
3. computes P/PID output (deg/s)
4. converts to steps/s and clamps (`max_steps_per_s`)
5. applies deadband (set steps/s = 0 inside band)
6. updates `cmd_steps_per_s[j]` for the ISR

Important timing separation:
- The control task may jitter slightly; the ISR keeps step timing stable.
- The control task should never block for long (avoid `delay()`, avoid heavy printing).

Recommended additions for stability (still “simple”):
- Add a per-joint **acceleration limit** (slew-rate limit on `cmd_steps_per_s`) so step rate does not jump instantly.
  - This dramatically reduces stalls and harshness and is simpler than full motion profiling.
- If encoder noise causes small oscillations near target, increase deadband slightly and/or low-pass filter the measured angle.

Configuration source of truth (firmware-side):
- Per-joint step rate limits will live in firmware config (e.g., `controls_config.h`):
  - `max_steps_per_s` (and optionally `min_steps_per_s` if you want a minimum effective speed)
  - These are tuning knobs and can be adjusted later without changing the control structure.

### Joint limits (soft stops)
Even though the encoder wraps 0–360, each joint may have a smaller mechanical travel.

Add per-joint soft limits:
- `min_deg`, `max_deg` (in the same absolute degree convention used for targets)
- If a target is outside limits, clamp it.
- If measured angle is outside limits (fault), stop that joint and optionally disable driver.

Near-term implementation decision:
- Mechanical limits currently exist in the PC GUI config (e.g., `gui/config.py`), but that data is not available to `controls.ino`.
- For initial bring-up, we will use **temporary hardcoded joint limits in firmware** (e.g., in `controls_config.h`).
- Later, we can unify sources of truth by either:
  - sending limits over serial at startup, or
  - generating a shared config artifact used by both GUI and firmware.

Note: the “shortest path” wrap logic is correct for a full circle, but if your allowed travel is less than 360°, shortest-path may command motion that violates limits unless you clamp target and/or constrain direction.

### Hobby servos (Joint 6 + gripper)
The two hobby servos are **not cut off** by the stepper ISR.

Plan:
- Drive the servos using the standard Teensyduino `Servo` library (or equivalent Teensy PWM-based approach).
- Update servo setpoints in the foreground at a reasonable rate (e.g., 50–100 Hz).
- Do not generate servo pulses manually in our stepper ISR.

Coexistence constraints (important):
- Servo signal generation also relies on precise timing (typically interrupt/timer driven under the hood).
- Our stepper ISR must remain short (microseconds). If an ISR runs too long or disables interrupts for too long, servo pulse timing can jitter.
- Using `IntervalTimer` for stepping and `Servo` for servos is a common, workable combination on Teensy as long as the stepper ISR stays lightweight.

Additional guidance:
- Keep the stepper ISR at a moderate priority (not the absolute highest) so the servo pulse timing isn’t starved.
- Avoid long critical sections (`noInterrupts()`) in foreground code.

## Safety & behavior rules
- If joint disabled (GUI toggle or ESTOP):
  - set steps/s = 0
  - deassert enable (or assert disable)
- In deadband:
  - steps/s = 0
  - keep enable asserted for holding torque unless you intentionally want free/backdrivable behavior.

Additional safety recommendations:
- If encoder read fails (I2C error / mux fault), command that joint to stop and optionally disable.
- Consider a watchdog: if control loop doesn’t run for > N ms, command all joints to stop.

## Test-first approach (`controls_test.ino`)
Before integrating with full serial GUI commands, implement a standalone test sketch that:
- controls **one stepper joint** with the above structure
- reads one AS5600 encoder (no mux initially, or fixed mux channel)
- flips between two target angles periodically (or uses a pot for target)
- demonstrates smooth motion, deadband settle, and non-blocking step generation

### Bring-up milestones (do one joint first)
Milestone 1 — Open-loop stepping (no encoder yet):
- Implement the stepper ISR engine for **one motor** only.
- Command a fixed `cmd_steps_per_s` and confirm smooth motion across a range of speeds.
- Verify STEP pulse width and DIR timing work with your TB6600 settings.

Milestone 2 — Closed-loop P control on one joint:
- Read one AS5600 (fixed mux channel or no mux).
- Implement wrap-safe error + deadband + P-only to generate `cmd_steps_per_s`.
- Verify it seeks and settles without hunting.

Milestone 3 — Scale to 5 steppers:
- Add remaining motors to the ISR (same single timer).
- Time-slice/optimize encoder reads as needed.

Milestone 4 — Integrate servos + GUI:
- Add Joint 6 servo and gripper updates in the foreground.
- Integrate serial targets and telemetry, keeping ISR unchanged.
