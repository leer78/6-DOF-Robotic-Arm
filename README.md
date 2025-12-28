# 6-DOF Robotic Arm — Global Plan & Control Flow

This README is the single-source-of-truth for the project scope, control algorithm choices, wiring and software stack, and the operational flow for the Teensy 4.1-based 6-DOF robotic arm (currently 4 joints assembled).

**Project Summary**
- **Goal:** Provide a robust closed-loop control system for a 6-DOF arm (4 joints currently connected) using NEMA17 steppers + TB6600 drivers and AS5600 magnetic encoders read through a TCA9548A I2C multiplexer. A PC-based UI will send commands to the Teensy 4.1 via serial (one-way UI->Teensy). The Teensy performs closed-loop joint control and safety.
- **Assumptions:** Joints rotate ≤360° (single rotation), AS5600 provides 12-bit angle data, TB6600 receives STEP/PUL (PUL) + DIR, multiplexer channel switching is used to address each encoder.

**Hardware Overview**
- **Controller:** `Teensy 4.1` — runs the main control loop, reads encoders, drives steppers.
- **Encoders:** `AS5600` magnetic encoders (I2C address `0x36`) connected behind a `TCA9548A` I2C multiplexer (channels used for each joint's encoder).
- **Motor Drivers:** `TB6600` drivers per joint. Control pins used: `DIR+/-`, `PUL+/-`, (and `ENA` if available for enabling/disabling drivers).
- **Steppers:** `NEMA 17` stepper motors (one per joint).
- **Local controls:** 1x potentiometer (jog / speed adjust) and 1x momentary push button (mode / teach / record zero).
- **UI:** PC-based GUI (Tkinter or similar) — sends serial-only commands to Teensy. No reliable two-way serial assumed.

**Wiring & I2C notes**
- Connect all `AS5600` SDA/SCL lines to corresponding `TCA9548A` channels. Control `TCA9548A` by writing the channel-select byte before reading an encoder.
- `AS5600` returns 12-bit raw angle (0..4095). Convert to degrees: `deg = raw * 360.0 / 4096.0`.
- Protect I2C with common pull-ups (3.3V), and ensure the Teensy I2C bus voltage matches encoder/multiplexer.

**Software Stack & Libraries (recommended)**
- On Teensy (C++ / Arduino-style):
  - `Wire` (I2C) — for AS5600 + TCA9548A
  - `AS5600` — AS5600 library by Robert Tillaart (install via Arduino Library Manager; used for reading AS5600 encoders)
  - `AccelStepper` — for STEP/DIR driver control (use `DRIVER` mode for TB6600)
  - `EEPROM` or a small nonvolatile store — to save per-joint calibration offsets
  - `PID_v1` (Brett Beauregard) or similar — per-joint PID controller if using PID
  - Optional: `IntervalTimer` (Teensy) for precise pulse timing if generating step pulses manually
- PC UI: `Tkinter` (Python) for prototype GUI; send serial ASCII packets over USB serial to Teensy.

**High-level Control Flow (Runtime)**
1. Power-up -> Teensy boots.
2. Teensy reads saved calibration offsets from EEPROM.
3. Teensy enters `IDLE` state (safe): motors disabled, awaiting serial command or local button input.
4. Operator uses UI to pick an operating mode (Calibration, Manual Joint, IK Move, Emergency Stop) and sends commands.
5. On motion commands: Teensy reads current encoder angles → computes error to setpoint → closed-loop controller runs → generates step/dir pulses to TB6600.
6. Monitor safety constraints (max speed, joint limits); if violated, abort movement and disable motors.

**Operating Modes (suggested)**
- `IDLE` — motors disabled. Wait for commands.
- `CALIBRATE` — per-joint calibration routine (record offsets and directions). Local button/pot can be used to jog.
- `MANUAL_JOINT` — set individual joint angles (absolute) from UI.
- `IK_MOVE` — UI sends desired end-effector pose (later). Teensy receives computed joint setpoints or UI requests Teensy to compute IK offline (TBD).
- `EMERGENCY_STOP` — immediately disable motor outputs.

**Closed-loop Control Strategy (recommended approach)**
We have two practical ways to implement closed-loop control with steppers + magnetic encoder feedback: a) Position-target-driven stepper with encoder correction, or b) PID velocity output controlling step frequency. Choose the one that fits your team skill and hardware timing.

Option A — Position-based with AccelStepper (recommended for simplicity):

Pros: reuses AccelStepper acceleration & ramping; simpler to implement.
Cons: AccelStepper is not inherently closed-loop — you must periodically reconcile encoder position with stepper position.

**Closed-loop Control Strategy (position → velocity PID using AccelStepper.runSpeed)**

This project will use a position-to-velocity PID control strategy: the encoder-derived position error is fed to a PID controller which outputs a signed velocity command. The velocity command is then applied to the stepper using `AccelStepper`'s `setSpeed()` + `runSpeed()` (or equivalent timed step generation). This matches your proposed flow and keeps the loop simple, robust and easy to tune.

Concise algorithm flow:
1. Receive `θ_des` (desired joint angle) from UI or IK module.
2. Read current joint angle `θ_actual` from the AS5600 via the TCA9548A.
3. Compute position error: `err = shortest_signed_angle(θ_des - θ_actual)`.
4. Feed `err` into a PID controller (treating error as the process variable input). The PID output maps to a desired angular velocity `ω_cmd` (deg/s).
5. Convert `ω_cmd` into stepper speed units (steps/s) using `steps_per_degree` for that joint; apply saturation limits to cap max speed.
6. Optionally apply a simple velocity ramp or slew-rate limiter to avoid step jitter (not a priority for initial testing).
7. Apply `AccelStepper.setSpeed( signed_steps_per_second )` and call `AccelStepper.runSpeed()` frequently in the main loop to produce motion.
8. Repeat the loop until `|err|` is below a deadband threshold (e.g., 5°) — then hold position (set speed to zero and keep calling `runSpeed()` if desired to hold motor idle), or optionally apply a small holding torque/current via driver enable pins if available.

Key details and implementation notes:
- Deadband / hold threshold: Implement a configurable angle deadband (your example: 5°). If `|err| <= deadband`, treat the joint as settled and set speed to zero to avoid hunting.
- Direction: Use the sign of `err` to set the sign of `ω_cmd`. Define 0° at the +X axis as your reference; ensure shortest path around wrap is used when computing `err`.
- Units and conversion: Maintain per-joint `steps_per_degree` (motor steps × microstep / gear ratio). Convert `ω_cmd (deg/s)` → `steps/s` by multiplying by `steps_per_degree`.
- Saturation: Clamp `steps/s` to a configured `max_steps_per_s` per joint to keep motor and driver within safe limits.
- runSpeed behavior: `runSpeed()` requires a continuously called loop to emit steps at the desired rate. Use a high-priority loop or timer-driven main loop to call `runSpeed()` frequently (ideally many times per step interval).
- PID tuning: Treat PID output range as mapped to a comfortable `ω_cmd` range. Start with P-only (Ki = Kd = 0), tune Kp so that large errors produce noticeable speed but not aggressive overshoot. Add small Ki if steady-state error persists and Kd to damp oscillation.
- Encoder wrap: Compute angle error using shortest signed difference (wrap_to_pi / modulo arithmetic) so motions near the 0/360 boundary choose the minimal rotation.
- Calibration & sync: On initialization, compute and store `zero_offset` and `steps_per_degree` in EEPROM. Use calibration routine to align encoder zero to mechanical zero. Use encoder readings to correct any step loss by momentarily setting `setCurrentPosition()` if you also track steps internally.
- Safety: Always enforce per-joint soft limits (min_angle, max_angle) before allowing motion. Implement an immediate `ESTOP` that sets speed to zero and disables motor outputs.

When to hold vs micro-correct:
- Use the deadband to avoid unnecessary micro-corrections. If precise final position is required (smaller than the deadband), reduce the deadband and retune PID to smaller, slower velocities near zero error.

Why this fits your constraints:
- Keeps UI→Teensy simple (UI sends desired angles). Teensy remains the single-loop executor.
- Uses `AccelStepper`'s `runSpeed()` for straightforward step emission and allows deterministic timed stepping without implementing a full custom pulse-timer system initially.
- Converts positional error directly into velocity — intuitive and robust for stepper systems with encoders.

Future improvements (later phases):
- Replace `runSpeed()` with a timer-driven pulse generator for even lower jitter if needed.
- Add smooth velocity profiling (S-curve) or slew-rate limiting on `ω_cmd` for heavy payloads or high-speed moves.
- Optionally add a supervisory PID on top of stepper position if you observe systematic steady-state drift that the encoder-velocity PID cannot correct.

- Per-joint PID reads encoder angle (process variable) and setpoint is desired angle.
- PID output is desired angular velocity (deg/s) or step frequency (steps/s).
- A pulse generator (timer-based) produces step pulses at rate determined by PID output and sets DIR accordingly.
- Implement trapezoidal profiling by limiting the rate-of-change of setpoint or by saturating the PID output.

Pros: tighter closed-loop response; avoids accumulation of step loss.
Cons: more complex; must implement safe, jitter-free pulse generation. Consider `IntervalTimer` on Teensy for precise pulses.

Recommendation: Start with Option A to get moving quickly and safe ramping; migrate to Option B for higher-performance, disturbance-tolerant control once system basics are stable.

**Encoder wrap and shortest-path logic**
- Even though joints won't spin >360°, compute angle error using the minimal signed difference to correctly handle wrap-around near 0/360:
  - error = wrap_to_pi(desired_deg - current_deg)
- Use that error to decide signed direction and the number of steps required.

**Serial Protocol (UI → Teensy)**
- Use a human-readable ASCII packet format (easy to debug). Each command ends with newline `\n`.
- Example commands:
  - `MODE:IDLE` — switch to IDLE
  - `MODE:CALIB` — start calibration mode
  - `SETJ:idx:angle:spd` — set joint `idx` (0..5) to `angle` (degrees) with optional `spd` (0..100)
    - Example: `SETJ:1:45.0:80`
  - `SETP:joint0, joint1, joint2,...` — set multiple joint angles in one packet: `SETP:0:10.0,1:20.0,2:5.5` or simplified `SETP:10,20,5.5,0` depending on parsing approach
  - `CAL:START` / `CAL:STORE` — start calibration / store calibration
  - `STOP` — stops motion immediately (soft stop)
  - `ESTOP` — emergency stop, disable motors
- Parsing notes: keep parsing simple and robust. On each packet, validate numbers and bounds. If packet malformed, ignore.

Serial UI constraints & feedback (one-way limitation):
- You said Teensy can only listen (UI→Teensy). That means: the UI cannot receive real-time telemetry from Teensy over the same serial link. Design the UI to be command-only and rely on local indicators for confirmation:
  - Use LED status codes on the Teensy (e.g., blink patterns) for simple status.
  - Use the potentiometer and button to manually confirm state during calibration.
  - Save logs or events to a removable medium (SD) on Teensy for post-run analysis.
  - If two-way comms become required later, consider: a second USB-serial channel, Wi-Fi/BLE telemetry, or hooking the Teensy as an HID device.

**Calibration & Initialization Procedure**
- Purpose: map raw AS5600 counts → joint absolute angle and determine direction/zero-offsets.
- Suggested flow:
  1. Enter `CALIBRATE` mode from UI.
  2. For each joint (index shown via LEDs or serial console during dev):
     - Allow operator to jog joint with potentiometer while Teensy reads encoder.
     - Press the push button to “record zero” when the joint is at the desired mechanical zero.
     - Teensy stores `zero_offset_raw` and `direction` (if encoder moves opposite expected direction).
  3. Compute `steps_per_degree` using motor steps and gearbox (if any) or calibrate empirically: move known degrees and count steps.
  4. Save offsets and steps-per-degree to EEPROM.

**PID & Motion Profiling Guidance**
- If using PID: use one PID instance per joint with reasonable defaults. Tune using Ziegler-Nichols or manual tuning:
  - Start with low Kp, Ki = 0, Kd = 0. Increase Kp until some oscillation or acceptable rise time.
  - Add Kd to reduce overshoot. Add Ki to remove steady-state error.
- Use PID output as a velocity command (Option B) or as a supervisory correction that nudges AccelStepper targets (Option A).
- Implement motion profiling (trapezoidal or S-curve) by limiting acceleration and jerk. `AccelStepper` handles basic acceleration (trapezoidal).

**Safety & Limits**
- Always provide: global soft limits per joint (min_angle, max_angle).
- Software E-Stop (`ESTOP`) that immediately disables step pulses and enables motor driver disable (if available).
- Thermal and current checks: if using driver current sense, monitor temperature or implement timeouts to avoid overheating.
- Start every run with small safe velocities until confidence and calibration verified.

**Testing & Tuning Plan**
- Phase 1: Build basic read loop
  - Read AS5600 angle from multiplexer channels and print raw values to serial (dev only).
  - Read pot and button locally to confirm I/O.
- Phase 2: Drive a single joint in open-loop using AccelStepper and confirm step→movement.
- Phase 3: Implement sync: map encoder angle ↔ step position, use `setCurrentPosition()` to sync.
- Phase 4: Implement position moves using `moveTo()` and reconcile encoder periodically.
- Phase 5: Add safety limits, calibration storage, and UI command support.
- Phase 6: Tune PID (if using velocity-PID) or refine supervisor correction, test with payloads and repeatability tests.

**Recommended Next Steps (immediate)**
- Implement a Teensy sketch skeleton that:
  - Initializes I2C and TCA9548A, reads AS5600 on each channel.
  - Initializes AccelStepper per joint.
  - Exposes a simple serial parser for `SETJ`, `MODE`, `STOP`, `CAL` commands.
  - Implements the `CALIBRATE` routine using the local button and pot.
- Build a minimal Tkinter UI that sends ASCII packets as described and a safe “arm enabled” toggle.

**Notes & Answers to Your Concerns**
- Two-way serial: if the Teensy cannot transmit back, you must design the UI as command-only. For development, use direct USB serial to inspect data. Later, add a second comms channel for telemetry if needed.
- PID vs ramping: prefer AccelStepper for ramping and start with supervisory PID. Move to a velocity-PID with a precise timer pulse generator for tighter closed-loop control.
- Libraries: `AccelStepper` + `PID_v1` are good starting points. Teensy `IntervalTimer` is useful if you need deterministic stepping.
- Calibration: store encoder offsets and steps-per-degree in EEPROM. Always implement a way to re-run calibration and to restore defaults.

---
If you want, I can now:
- produce a Teensy sketch skeleton (I2C read of AS5600, multiplexer switching, basic serial parser, AccelStepper init), or
- generate a minimal Tkinter UI that emits the command packets described for early testing.

File: `README.md` — this file is the project source of truth for control flow and next steps.
# 6-DOF-Robotic-Arm:w
