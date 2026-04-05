# Migration Plan: `controls_test.ino` → `controls.ino`

## Senior-Engineer Analysis of the Transition

**Objective:** Integrate the single-motor, single-PID, ISR-driven closed-loop control proven in `controls_test.ino` into the multi-joint `controls.ino` framework — scaling from 1 stepper to 5 steppers (joints 1–5), while preserving the existing serial protocol, servo control (Joint 6 + gripper), and mode-based state machine.

---

## 1. Inventory: What `controls_test.ino` Has That `controls.ino` Lacks

| Subsystem | `controls_test.ino` (single-axis) | `controls.ino` (multi-axis) | Action |
|---|---|---|---|
| **Stepper pin declarations** | `PIN_STEP`, `PIN_DIR`, `PIN_EN` (one set, hardcoded) | No stepper pin definitions at all | Add per-joint STEP/DIR/EN pin arrays to `controls_config.h` |
| **ISR pulse engine** | `IntervalTimer stepTimer` + `stepISR()` — fixed-point accumulator, DIR guard, pulse scheduling | Not present | Migrate ISR, generalize to N motors |
| **Fixed-point math constants** | `ONE_STEP`, `FIXED_INV_TICK` | Not present | Move to `controls_config.h` (shared) |
| **Shared volatile state** | `cmd_pulses_per_s`, `motor_enabled` (single scalars) | Not present | Generalize to per-joint arrays |
| **ISR-internal state** | `accum`, `step_high`, `dir_guard_remain`, `cur_dir_forward`, `step_pulse_count` | Not present | Generalize to per-joint struct |
| **PID controller** | `PID_v1` instance + `pidInput`, `pidOutput`, `pidSetpoint` | Not present | Create per-joint PID instances + state |
| **Tuning parameters** | `TICK_HZ`, `MICROSTEPS`, `MIN/MAX_FULL_STEPS_PER_S`, `MIN/MAX_PULSES_PER_S`, `DIR_GUARD_TICKS`, `PID_KP/KI/KD`, `DEADBAND_DEG`, `CONTROL_PERIOD_MS` | Not present | Move to `controls_config.h`, per-joint where needed |
| **Shortest-path error** | Wrap-safe error in `loop()` | Not present | Integrate into `modeMoveCode()` |
| **Deadband + clamp logic** | In `loop()` after PID compute | Not present | Integrate into `modeMoveCode()` |
| **Encoder read** | `selectMuxChannel()` + `encoder.readAngle()` (single channel) | ✅ Already present (`readEncoders()` scans all joints) | Reuse existing; no duplication needed |
| **Mux helpers** | `selectMuxChannel()`, `tcaPresent()` | ✅ Already present | Reuse existing |
| **Serial target input** | Manual serial input (type angle + Enter) | ✅ Replaced by `JOINTS_TO_ANGLE` command parsing | Use existing protocol; remove manual input |
| **Telemetry** | Raw debug `Serial.print` | ✅ Structured `sendTelemetry()` | Reuse existing |
| **Mode state machine** | Not present (always running) | ✅ `MODE_IDLE / CALIBRATION / MOVE` | Keep existing; PID loop only runs in `MODE_MOVE` |
| **Servo control (J6 + gripper)** | Not present | ✅ Already present | Keep as-is |
| **Button debounce** | `PIN_BTN` with `INPUT_PULLUP`, not debounced | ✅ Debounced `readButton()` | Keep existing |
| **ESTOP handling** | Not present | ✅ Disables all joints | Extend: ESTOP must also zero all ISR commands |
| **`EN_ACTIVE_LEVEL`** | Hardcoded `LOW` (active-low TB6600) | Only a global `ENABLE_pin` (pin 8, unused in move logic) | Per-joint enable via arrays |

---

## 2. Design Decisions

### 2.1 What Is Shared (Common to All Motors)

These values are identical for all stepper joints and belong as single `#define` / `static const` in `controls_config.h`:

| Parameter | Value (from test) | Notes |
|---|---|---|
| `TICK_HZ` | 40000 | ISR tick rate — determines pulse timing resolution |
| `DIR_GUARD_TICKS` | 3 | Ticks to wait after DIR pin change (~75 µs at 40 kHz) |
| `CONTROL_PERIOD_MS` | 50 | PID update rate (20 Hz) |
| `EN_ACTIVE_LEVEL` | `LOW` | TB6600 active-low enable (may override per-joint later) |
| `ONE_STEP` | `1UL << 16` | Fixed-point threshold (Q16.16) |
| `FIXED_INV_TICK` | computed in `setup()` | `(1ULL << 32) / TICK_HZ` |
| `NUM_STEPPERS` | 5 | Joints 1–5 (Joint 6 is servo) |

### 2.2 What Is Per-Joint (Unique to Each Motor)

These values differ per motor/joint and should be stored in arrays indexed `[0..NUM_STEPPERS-1]`:

| Parameter | Type | Example (from test, J2) | Notes |
|---|---|---|---|
| `STEP_PIN[j]` | `uint8_t` | 27 | TB6600 PUL+ |
| `DIR_PIN[j]` | `uint8_t` | 28 | TB6600 DIR+ |
| `EN_PIN[j]` | `uint8_t` | 29 | TB6600 ENA+ |
| `MICROSTEPS[j]` | `uint16_t` | 4 | TB6600 DIP setting |
| `MIN_FULL_STEPS_PER_S[j]` | `int32_t` | 10 | Min nonzero speed |
| `MAX_FULL_STEPS_PER_S[j]` | `int32_t` | 120 | Max speed |
| `MIN_PULSES_PER_S[j]` | `int32_t` | derived | `MICROSTEPS[j] * MIN_FULL_STEPS_PER_S[j]` |
| `MAX_PULSES_PER_S[j]` | `int32_t` | derived | `MICROSTEPS[j] * MAX_FULL_STEPS_PER_S[j]` |
| `PID_KP[j]` | `double` | 5.0 | Proportional gain |
| `PID_KI[j]` | `double` | 0.0 | Integral gain |
| `PID_KD[j]` | `double` | 0.0 | Derivative gain |
| `DEADBAND_DEG[j]` | `float` | 0.75 | Dead zone |
| `MUX_CH[j]` | `uint8_t` | 6 | Already in `controls_config.h` |

### 2.3 Pin Layout in `controls_config.h`

The pin array approach makes it trivial to see the wiring at a glance:

```
// ── Stepper Pins: {STEP, DIR, EN} per joint ──
//                       J1    J2    J3    J4    J5
STEP_PINS[5]      =  { ??,   27,   ??,   ??,   ??  };
DIR_PINS[5]       =  { ??,   28,   ??,   ??,   ??  };
EN_PINS[5]        =  { ??,   29,   ??,   ??,   ??  };
```

Placeholders (`??`) for un-wired joints will be filled in as hardware is connected. Disabled joints (`jointEnabled[j] == false`) skip ISR and PID entirely.

### 2.4 Button Pin

The button is standalone — not part of the per-joint arrays. It stays as `PUSH_SWITCH_pin` (pin 37), already in `controls_config.h`.

---

## 3. Structural Changes — File-by-File

### 3.1 `controls_config.h` — All New Sections

**Add the following new sections** (after existing content, before `#endif`):

#### A. ISR / Pulse Engine Constants (shared)

```
#define NUM_STEPPERS       5
#define TICK_HZ            40000
#define DIR_GUARD_TICKS    3
#define CONTROL_PERIOD_MS  50
#define EN_ACTIVE_LEVEL    LOW
```

#### B. Per-Joint Stepper Pin Arrays

Three `const uint8_t` arrays of length `NUM_STEPPERS`:
- `STEP_PINS[]`
- `DIR_PINS[]`
- `EN_PINS[]`

Each index `[0]` = Joint 1, `[1]` = Joint 2, … `[4]` = Joint 5.

#### C. Per-Joint Tuning Arrays

```
MICROSTEPS[NUM_STEPPERS]
MIN_FULL_STEPS_PER_S[NUM_STEPPERS]
MAX_FULL_STEPS_PER_S[NUM_STEPPERS]
```

`MIN_PULSES_PER_S` and `MAX_PULSES_PER_S` are **derived at startup** (`setup()`) rather than duplicated as defines:
```
minPulsesPerS[j] = MICROSTEPS[j] * MIN_FULL_STEPS_PER_S[j];
maxPulsesPerS[j] = MICROSTEPS[j] * MAX_FULL_STEPS_PER_S[j];
```

#### D. Per-Joint PID Tuning

```
PID_KP[NUM_STEPPERS]
PID_KI[NUM_STEPPERS]
PID_KD[NUM_STEPPERS]
DEADBAND_DEG[NUM_STEPPERS]
```

#### E. Fixed-Point Constants

```
#define ONE_STEP (1UL << 16)
```

`FIXED_INV_TICK` is computed in `setup()` (runtime, not preprocessor).

---

### 3.2 `controls.ino` — Section-by-Section Changes

#### A. New `#include` Directives

Add at the top:
```cpp
#include <PID_v1.h>
```
(`Wire.h`, `AS5600.h`, `Servo.h` are already included.)

#### B. New Global Data Structures

##### ISR-side per-motor struct

Create a struct to hold all per-motor ISR-internal state:

```cpp
struct StepperISRState {
    volatile uint32_t accum;
    volatile bool     step_high;
    volatile uint8_t  dir_guard_remain;
    volatile bool     cur_dir_forward;
    volatile uint32_t step_pulse_count;  // diagnostic
};
```

Instantiate as:
```cpp
static StepperISRState isrState[NUM_STEPPERS];  // zero-initialized
```

##### Shared state arrays (loop ↔ ISR)

```cpp
static volatile int32_t cmdPulsesPerS[NUM_STEPPERS]  = {0};
// motor_enabled already covered by jointEnabled[] — reuse it
```

##### PID arrays

```cpp
static double pidInput[NUM_STEPPERS]    = {0};
static double pidOutput[NUM_STEPPERS]   = {0};
static double pidSetpoint[NUM_STEPPERS] = {0};  // always 0 (error-based)

PID* posPID[NUM_STEPPERS];  // initialized in setup()
```

##### Derived tuning (computed in setup)

```cpp
static int32_t minPulsesPerS[NUM_STEPPERS];
static int32_t maxPulsesPerS[NUM_STEPPERS];
```

##### Fixed-point reciprocal

```cpp
static uint32_t FIXED_INV_TICK;  // computed in setup()
```

##### IntervalTimer

```cpp
IntervalTimer stepTimer;
```

#### C. ISR Function: `stepISR()` — Generalized to N Motors

The ISR loops over `NUM_STEPPERS` joints on every tick. For each joint `j`:

1. **If `step_high[j]`:** drive `STEP_PINS[j]` LOW, clear flag, `continue` to next joint.
2. **If `dir_guard_remain[j] > 0`:** decrement, `continue`.
3. **If joint disabled (`!jointEnabled[j]`) or `cmdPulsesPerS[j] == 0`:** zero accumulator, `continue`.
4. **Direction check:** if desired direction differs from `cur_dir_forward[j]`, set `DIR_PINS[j]`, start guard, zero accum, `continue`.
5. **Accumulate:** fixed-point increment, if ≥ `ONE_STEP` → drive `STEP_PINS[j]` HIGH, set `step_high`, subtract threshold.

**Critical constraints (unchanged from test):**
- No I2C, no Serial, no float, no `delay()`.
- Use `digitalWriteFast()` for all pin writes.
- Keep total ISR time well under 25 µs (at 40 kHz). With 5 motors and a few `digitalWriteFast` calls each, this is comfortable (~2–5 µs total).

#### D. `setup()` — New Initialization Steps

Insert **after** existing I2C / encoder / servo init, **before** `"System Ready"` message:

1. **Compute derived tuning:**
   ```
   for j in 0..NUM_STEPPERS-1:
       minPulsesPerS[j] = MICROSTEPS[j] * MIN_FULL_STEPS_PER_S[j]
       maxPulsesPerS[j] = MICROSTEPS[j] * MAX_FULL_STEPS_PER_S[j]
   ```

2. **Compute fixed-point reciprocal:**
   ```
   FIXED_INV_TICK = (uint32_t)((1ULL << 32) / TICK_HZ);
   ```

3. **Initialize stepper pins:**
   ```
   for j in 0..NUM_STEPPERS-1:
       pinMode(STEP_PINS[j], OUTPUT);  digitalWriteFast(STEP_PINS[j], LOW);
       pinMode(DIR_PINS[j],  OUTPUT);  digitalWriteFast(DIR_PINS[j], HIGH);
       pinMode(EN_PINS[j],   OUTPUT);  digitalWrite(EN_PINS[j], EN_ACTIVE_LEVEL);
   ```

4. **Initialize PID instances:**
   ```
   for j in 0..NUM_STEPPERS-1:
       posPID[j] = new PID(&pidInput[j], &pidOutput[j], &pidSetpoint[j],
                           PID_KP[j], PID_KI[j], PID_KD[j], DIRECT);
       posPID[j]->SetMode(AUTOMATIC);
       posPID[j]->SetOutputLimits(-(double)maxPulsesPerS[j], (double)maxPulsesPerS[j]);
       posPID[j]->SetSampleTime(1);  // we gate timing ourselves
   ```

5. **Start ISR timer (last, after all I2C init):**
   ```
   stepTimer.begin(stepISR, 1000000.0f / (float)TICK_HZ);
   ```

#### E. `modeMoveCode()` — The Heart of the Migration

Replace the current placeholder `modeMoveCode()` with a fixed-rate control loop that runs the PID + ISR pipeline for all enabled stepper joints.

**Flow (runs every `CONTROL_PERIOD_MS`):**

```
static uint32_t lastControlMs = 0;
uint32_t nowMs = millis();
if (nowMs - lastControlMs < CONTROL_PERIOD_MS) return;
lastControlMs = nowMs;

for (j = 0; j < NUM_STEPPERS; j++) {
    // Skip disabled joints — also zero their ISR command
    if (!jointEnabled[j]) {
        noInterrupts();
        cmdPulsesPerS[j] = 0;
        interrupts();
        continue;
    }

    // 1. Encoder angle already in encoderAngles[j] (from readEncoders())

    // 2. Shortest-path signed error (wrap-safe)
    float error = jointTargetAngles[j] - encoderAngles[j];
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 3. PID compute
    pidInput[j] = (double)error;
    posPID[j]->Compute();

    // 4. Deadband + clamp
    int32_t newCmd;
    if (fabsf(error) < DEADBAND_DEG[j]) {
        newCmd = 0;
    } else {
        newCmd = (int32_t)pidOutput[j];
        if (newCmd > 0 && newCmd < minPulsesPerS[j]) newCmd =  minPulsesPerS[j];
        if (newCmd < 0 && newCmd > -minPulsesPerS[j]) newCmd = -minPulsesPerS[j];
        if (newCmd >  maxPulsesPerS[j]) newCmd =  maxPulsesPerS[j];
        if (newCmd < -maxPulsesPerS[j]) newCmd = -maxPulsesPerS[j];
    }

    // 5. Atomic update
    noInterrupts();
    cmdPulsesPerS[j] = newCmd;
    interrupts();
}

// Joint 6: servo control (unchanged — already handled below this loop)
```

**Why `readEncoders()` is called before `modeMoveCode()`:**
The existing `loop()` already calls `readEncoders()` before mode-specific code (in the telemetry block). `modeMoveCode()` simply uses the freshly-read `encoderAngles[]`. No redundant I2C reads.

#### F. Mode Transitions — Safety Hooks

##### Entering `MODE_MOVE`
- Enable all `jointEnabled[j]` stepper drivers by asserting `EN_PINS[j]`.
- PID instances are already initialized; they will start computing on the next control tick.

##### Leaving `MODE_MOVE` (transition to IDLE/CALIBRATION)
- Zero all `cmdPulsesPerS[j]` (stop stepping).
- Optionally de-assert stepper enables to allow free movement (calibration use case).

##### ESTOP Handling
Extend the existing `CMD_ESTOP` handler:
```
// In addition to disabling jointEnabled[]:
noInterrupts();
for (j = 0; j < NUM_STEPPERS; j++) cmdPulsesPerS[j] = 0;
interrupts();
```
This immediately halts all pulse generation.

#### G. Remove / Do NOT Migrate

The following items from `controls_test.ino` are **NOT** migrated because `controls.ino` already has better equivalents:

| `controls_test.ino` feature | Why skip |
|---|---|
| Manual serial target input (`serialBuf`, `atof()` parsing) | Replaced by `JOINTS_TO_ANGLE` protocol command |
| `Serial.print` debug telemetry in control loop | Replaced by `sendTelemetry()` structured packets |
| I2C bus scan in `setup()` | Already present in `scanTCAChannels()` |
| Single `encoder` instance init | Already present |
| `PIN_BTN` usage | Already present as `PUSH_SWITCH_pin` with debounce |

---

## 4. Data Flow Diagram (Post-Migration)

```
┌─────────────────── GUI ───────────────────┐
│                                           │
│  JOINTS_TO_ANGLE packet (6 angles)  ──────┼──► Serial
│                                           │
└───────────────────────────────────────────┘
                    │
                    ▼
┌─────────────── controls.ino ──────────────┐
│                                           │
│  listenPackets()                          │
│    └─► parseAndExecutePacket()            │
│          └─► jointTargetAngles[0..5] ✓    │
│                                           │
│  loop() @ ~200 Hz                         │
│    ├─► readEncoders()                     │
│    │     └─► encoderAngles[0..5] ✓        │
│    ├─► sendTelemetry()                    │
│    └─► modeMoveCode() @ 20 Hz             │
│          │                                │
│          ├─► for j in [0..4] (steppers):  │
│          │     error = wrap(target - cur) │
│          │     PID[j].Compute(error)      │
│          │     deadband + clamp           │
│          │     cmdPulsesPerS[j] = result  │
│          │                                │
│          └─► j=5 (Joint 6 servo):         │
│                joint6Servo.write(target)  │
│                                           │
│  stepISR() @ 40 kHz (IntervalTimer)       │
│    └─► for j in [0..4]:                   │
│          accumulate cmdPulsesPerS[j]      │
│          emit STEP pulses on STEP_PIN[j]  │
│          manage DIR, guard, enable        │
│                                           │
└───────────────────────────────────────────┘
```

---

## 5. Serial Protocol Alignment: `JOINTS_TO_ANGLE`

### Current Protocol Key Names

From `config.py` and `controls_config.h`, the packet uses:
```
TYPE=CMD,CMD=JOINTS_TO_ANGLE,JOINT_1_ANG=<val>,...,JOINT_6_ANG=<val>
```

The Teensy parser already extracts `JOINT_n_ANG` and writes to `jointTargetAngles[n-1]`.

### What the GUI Sends

From `config.py` `PROTOCOL_SCHEMAS`:
- Required keys: `JOINT_1_ANG` through `JOINT_6_ANG`
- Allowed only in `MODE_MOVE` (mode 2)

### Important Note on Angle Convention

The GUI sends **raw encoder-space angles** (degrees, 0–360). The `controls_test.ino` PID loop operates in the same space. The wrap-safe shortest-path error handles the 0/360 boundary. **No conversion is needed on the Teensy side** — `jointTargetAngles[]` are used directly as PID setpoints.

Joint 6's angle is passed through to `Servo.write()` (which expects 0–180 for standard servos, or the constrained 60–180 range per config).

---

## 6. Potential Issues & Mitigations

### 6.1 ISR Duration with 5 Motors

**Risk:** ISR takes too long, causing jitter or missed ticks.

**Mitigation:** At 40 kHz, each tick budget is 25 µs. Each motor in the ISR loop does at most: 1 conditional branch, 1 `digitalWriteFast` (< 50 ns on Teensy 4.1), and 1 fixed-point multiply. With 5 motors, worst case is ~2–3 µs total. Well within budget.

### 6.2 `readEncoders()` I2C Latency vs. Control Period

**Risk:** Reading 5 encoders sequentially via I2C mux may take 5–10 ms, eating into the 50 ms control period.

**Mitigation:** Current `CONTROL_PERIOD_MS = 50` provides ample margin. The `readEncoders()` call in the main loop already happens before `modeMoveCode()`. If latency becomes an issue, we can stagger reads (1–2 encoders per loop iteration) or increase `CONTROL_PERIOD_MS`.

### 6.3 PID Library Heap Allocation

**Risk:** Creating 5 `PID` objects with `new` in `setup()` on an embedded system.

**Mitigation:** Teensy 4.1 has 1 MB RAM. 5 PID objects (~50 bytes each) are negligible. Alternatively, we can statically allocate them. Either approach is fine.

### 6.4 Atomic Updates of `cmdPulsesPerS[]`

**Risk:** Torn reads if ISR fires mid-write.

**Mitigation:** Already handled in `controls_test.ino` with `noInterrupts()` / `interrupts()` around each write. Apply the same pattern per-joint in the loop.

### 6.5 Mode Transition Glitches

**Risk:** If mode changes from `MOVE` to `IDLE` while PID is commanding motion, motors may continue stepping.

**Mitigation:** On any transition **out of** `MODE_MOVE`, immediately zero all `cmdPulsesPerS[]` in a critical section. Add this to the `SET_MODE` handler.

### 6.6 `JOINTS_TO_ANGLE` Key Mismatch

**Risk:** The GUI `config.py` `PROTOCOL_SCHEMAS` uses `JOINT_n_ANG` as key names, but the `SERIAL_PROTOCOL_DESIGN.md` examples use `JOINT_n_ANGLE`. The Teensy parser uses `JOINT_n_ANG`.

**Mitigation:** Verify consistency. The **Teensy parser** (`parseAndExecutePacket`) searches for `"JOINT_" + String(i) + "_ANG="`, and `config.py` schema defines `JOINT_1_ANG`. These match. The design doc examples may be out of date. No code change needed, but worth noting.

---

## 7. Implementation Order (Recommended Sequence)

### Phase 1: `controls_config.h` — Configuration Additions
1. Add `NUM_STEPPERS`, `TICK_HZ`, `DIR_GUARD_TICKS`, `CONTROL_PERIOD_MS`, `EN_ACTIVE_LEVEL`, `ONE_STEP`
2. Add per-joint pin arrays: `STEP_PINS[]`, `DIR_PINS[]`, `EN_PINS[]`
3. Add per-joint tuning arrays: `MICROSTEPS[]`, `MIN_FULL_STEPS_PER_S[]`, `MAX_FULL_STEPS_PER_S[]`
4. Add per-joint PID arrays: `PID_KP[]`, `PID_KI[]`, `PID_KD[]`, `DEADBAND_DEG[]`

### Phase 2: `controls.ino` — Data Structures
5. Add `#include <PID_v1.h>`
6. Add `StepperISRState` struct and `isrState[NUM_STEPPERS]` array
7. Add shared volatile arrays: `cmdPulsesPerS[]`
8. Add PID state arrays: `pidInput[]`, `pidOutput[]`, `pidSetpoint[]`, `posPID[]`
9. Add derived arrays: `minPulsesPerS[]`, `maxPulsesPerS[]`
10. Add `FIXED_INV_TICK` and `IntervalTimer stepTimer`

### Phase 3: `controls.ino` — ISR
11. Implement `stepISR()` — generalized N-motor version

### Phase 4: `controls.ino` — Setup
12. Add stepper pin initialization loop
13. Add derived tuning computation
14. Add `FIXED_INV_TICK` computation  
15. Add PID instance creation loop
16. Start `stepTimer` (after all I2C init)

### Phase 5: `controls.ino` — Control Loop
17. Rewrite `modeMoveCode()` with fixed-rate PID loop
18. Add mode-transition safety (zero commands on exit from MOVE)
19. Extend ESTOP to zero all ISR commands

### Phase 6: Validation
20. Compile and verify no errors
21. Test with one enabled joint (e.g., Joint 2, which has known working pins 27/28/29 and mux channel 3)
22. Gradually enable additional joints as hardware is connected

---

## 8. Files Changed Summary

| File | Change Type | Description |
|---|---|---|
| `controls/controls_config.h` | **Extend** | Add ISR constants, per-joint pin arrays, per-joint tuning arrays, PID tuning arrays, fixed-point constants |
| `controls/controls.ino` | **Extend** | Add `PID_v1.h` include, ISR struct/arrays, `stepISR()`, PID init in `setup()`, rewrite `modeMoveCode()`, add mode-transition safety, extend ESTOP |
| `controls_test/controls_test.ino` | **No change** | Preserved as reference; no modifications |
| `gui/config.py` | **No change** | Already compatible; protocol keys match |

---

## 9. What We Are NOT Doing Yet (Deferred)

These items are acknowledged but explicitly deferred to keep this migration focused:

1. **Acceleration/slew-rate limiting** on `cmdPulsesPerS` — mentioned in `control_plan.md` but adds complexity. Start without it.
2. **Soft joint limits in firmware** — currently only in GUI `config.py`. Add later.
3. **Integral/derivative PID tuning** — start P-only (`Ki=0, Kd=0`) as recommended.
4. **Anti-windup logic** — not needed until `Ki > 0`.
5. **Telemetry enrichment** — adding PID debug output (error, command) to telemetry packets.
6. **GUI-side `JOINTS_TO_ANGLE` sending logic** — the GUI may have incomplete sending logic; this migration is firmware-focused.
7. **Motion profiling / trajectory planning** — future work, not part of initial closed-loop control.

---

## 10. Quick Reference: Key Constants Mapping

| `controls_test.ino` name | `controls_config.h` name | Scope |
|---|---|---|
| `TICK_HZ` | `TICK_HZ` | Shared |
| `DIR_GUARD_TICKS` | `DIR_GUARD_TICKS` | Shared |
| `CONTROL_PERIOD_MS` | `CONTROL_PERIOD_MS` | Shared |
| `EN_ACTIVE_LEVEL` | `EN_ACTIVE_LEVEL` | Shared |
| `ONE_STEP` | `ONE_STEP` | Shared |
| `PIN_STEP / PIN_DIR / PIN_EN` | `STEP_PINS[j] / DIR_PINS[j] / EN_PINS[j]` | Per-joint |
| `MICROSTEPS` | `MICROSTEPS[j]` | Per-joint |
| `MIN_FULL_STEPS_PER_S` | `MIN_FULL_STEPS_PER_S[j]` | Per-joint |
| `MAX_FULL_STEPS_PER_S` | `MAX_FULL_STEPS_PER_S[j]` | Per-joint |
| `PID_KP / KI / KD` | `PID_KP[j] / PID_KI[j] / PID_KD[j]` | Per-joint |
| `DEADBAND_DEG` | `DEADBAND_DEG[j]` | Per-joint |
| `ENCODER_MUX_CH` | `JOINTn_MUX_CH` (already exists) | Per-joint |
