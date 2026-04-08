/*
 * joint4_motor_test.ino — Open-loop direction test for Joint 4 stepper
 *
 * Drives the Joint 4 TB6600 stepper VERY SLOWLY back and forth
 * (counter-clockwise → pause → clockwise → pause → repeat).
 * NO encoder feedback, NO PID — pure open-loop stepping to isolate
 * whether the motor+driver can physically reverse direction.
 *
 * SAFETY:
 *   - Minimum speed: 10 full-steps/s (same as controls_config.h minimum)
 *   - Travel per direction: 80 microsteps ≈ 12° joint movement (3:1 gear)
 *   - Well within the ±45° physical limit from default position
 *   - Emergency stop: push button (pin 41) OR send 's' via Serial Monitor
 *   - 3-second countdown before first motion
 *
 * Hardware (from controls_config.h):
 *   Joint 4: STEP=11, DIR=12, EN=24
 *   Microsteps: 4 (TB6600 DIP), 800 pulses/rev
 *   3:1 gear ratio (motor turns 3× per joint degree)
 *   TB6600 enable: active LOW
 */

#include <Arduino.h>

// ── Pin Definitions (Joint 4) ────────────────────────────────────────────────
static const uint8_t PIN_STEP = 11;   // TB6600 PUL+
static const uint8_t PIN_DIR  = 12;   // TB6600 DIR+
static const uint8_t PIN_EN   = 24;   // TB6600 ENA+
static const uint8_t PIN_BTN  = 41;   // push button (active LOW, internal pullup)

// ── TB6600 Enable Levels ─────────────────────────────────────────────────────
static const uint8_t EN_ACTIVE_LEVEL  = LOW;
static const uint8_t EN_DISABLE_LEVEL = HIGH;

// ── Motor / Stepper Configuration ────────────────────────────────────────────
static const uint16_t MICROSTEPS      = 4;      // must match TB6600 DIP switches
static const uint32_t TICK_HZ         = 40000;  // ISR rate (25 µs per tick)
static const uint8_t  DIR_GUARD_TICKS = 3;      // ~75 µs guard after DIR change

// Speed: 10 full-steps/s × 4 microsteps = 40 pulses/s  (VERY slow)
static const int32_t SPEED_PULSES_PER_S = 10 * (int32_t)MICROSTEPS;  // 40

// Travel per direction: 80 microsteps
//   Motor: 80/800 rev = 36° shaft rotation
//   Joint: 36° / 3 (gear ratio) = 12° joint movement
static const uint32_t STEPS_PER_DIRECTION = 80;

// Pause between direction changes (milliseconds)
static const uint32_t PAUSE_MS = 2000;

// ── Fixed-point (Q16.16) ─────────────────────────────────────────────────────
static const uint32_t ONE_STEP = 1UL << 16;
static uint32_t FIXED_INV_TICK;  // precomputed (1<<32)/TICK_HZ

// ── ISR State (only touched inside stepISR) ──────────────────────────────────
static volatile uint32_t accum            = 0;
static volatile bool     step_high        = false;
static volatile uint8_t  dir_guard_remain = 0;
static volatile bool     cur_dir_forward  = true;
static volatile uint32_t step_pulse_count = 0;

// ── Shared State (loop ↔ ISR, volatile) ──────────────────────────────────────
static volatile int32_t  cmd_pulses_per_s = 0;
static volatile bool     motor_enabled    = false;
// ISR-level hard safety limit: ISR stops stepping when budget hits 0.
// Reload before each movement phase. Prevents runaway if main loop hangs.
static volatile uint32_t isr_step_budget  = 0;

IntervalTimer stepTimer;

// ── State Machine ────────────────────────────────────────────────────────────
enum TestState { IDLE, MOVING_CCW, PAUSING_1, MOVING_CW, PAUSING_2, STOPPED };
TestState state        = IDLE;
uint32_t stateEnteredMs = 0;
uint32_t stepsAtStart   = 0;
uint32_t cycleCount     = 0;

// ═════════════════════════════════════════════════════════════════════════════
// ISR — 40 kHz pulse engine (proven in controls_test.ino & controls.ino)
// ═════════════════════════════════════════════════════════════════════════════
void stepISR() {
  // Phase 1: complete the HIGH pulse — bring STEP low.
  if (step_high) {
    digitalWriteFast(PIN_STEP, LOW);
    step_high = false;
    return;
  }

  // Phase 2: direction-change guard countdown.
  if (dir_guard_remain > 0) {
    dir_guard_remain--;
    return;
  }

  // Phase 3: accumulate and maybe emit a step pulse.
  int32_t cmd = cmd_pulses_per_s;
  if (!motor_enabled || cmd == 0) {
    accum = 0;
    return;
  }

  bool desired_fwd = (cmd > 0);
  uint32_t abs_cmd = desired_fwd ? (uint32_t)cmd : (uint32_t)(-cmd);

  // Direction change → update DIR pin, start guard, reset accumulator.
  if (desired_fwd != cur_dir_forward) {
    cur_dir_forward = desired_fwd;
    digitalWriteFast(PIN_DIR, desired_fwd ? HIGH : LOW);
    dir_guard_remain = DIR_GUARD_TICKS;
    accum = 0;
    return;
  }

  // Fixed-point accumulate.
  uint32_t inc = (uint32_t)(((uint64_t)abs_cmd * FIXED_INV_TICK) >> 16);
  accum += inc;

  if (accum >= ONE_STEP) {
    if (isr_step_budget == 0) {
      accum = 0;
      return;  // hard safety limit reached — refuse to step
    }
    digitalWriteFast(PIN_STEP, HIGH);
    step_high = true;
    accum -= ONE_STEP;
    step_pulse_count++;
    isr_step_budget--;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Helpers
// ═════════════════════════════════════════════════════════════════════════════
void stopMotor() {
  noInterrupts();
  cmd_pulses_per_s = 0;
  interrupts();
}

void emergencyStop(const char* reason) {
  stopMotor();
  motor_enabled = false;
  digitalWrite(PIN_EN, EN_DISABLE_LEVEL);
  state = STOPPED;
  Serial.println();
  Serial.print("!!! EMERGENCY STOP — ");
  Serial.print(reason);
  Serial.println(" !!!");
  Serial.println("Motor disabled. Reset Teensy to restart test.");
}

// ═════════════════════════════════════════════════════════════════════════════
// setup
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Configure pins
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  pinMode(PIN_BTN,  INPUT_PULLUP);

  digitalWriteFast(PIN_STEP, LOW);
  digitalWriteFast(PIN_DIR,  HIGH);  // match cur_dir_forward = true

  // Enable the TB6600 driver
  digitalWrite(PIN_EN, EN_ACTIVE_LEVEL);
  motor_enabled = true;

  // Precompute fixed-point reciprocal
  FIXED_INV_TICK = (uint32_t)((1ULL << 32) / TICK_HZ);

  // Start the 40 kHz pulse ISR
  bool timerOk = stepTimer.begin(stepISR, 1000000.0f / (float)TICK_HZ);

  while (!Serial) {}
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Joint 4 — Motor Direction Test");
  Serial.println("========================================");
  Serial.println("Mode:   Open-loop (no PID, no encoder)");
  Serial.print(  "Speed:  10 full-steps/s (");
  Serial.print(SPEED_PULSES_PER_S);
  Serial.println(" microsteps/s)");
  Serial.print(  "Travel: ");
  Serial.print(STEPS_PER_DIRECTION);
  Serial.println(" microsteps per direction (~12 deg joint)");
  Serial.println("Cycle:  CCW -> pause -> CW -> pause -> repeat");
  Serial.println("STOP:   Push button (pin 41) or send 's' in Serial");
  Serial.print(  "Timer:  ");
  Serial.println(timerOk ? "OK" : "FAIL");
  Serial.println("────────────────────────────────────────");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  // Begin first CCW movement
  state = MOVING_CCW;
  stateEnteredMs = millis();
  stepsAtStart = step_pulse_count;
  cycleCount = 1;

  noInterrupts();
  isr_step_budget  = STEPS_PER_DIRECTION + 2;  // hard ISR safety cap
  cmd_pulses_per_s = -SPEED_PULSES_PER_S;
  interrupts();

  Serial.println();
  Serial.println("[Cycle 1] >>> Moving COUNTER-CLOCKWISE...");
}

// ═════════════════════════════════════════════════════════════════════════════
// loop
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  // ── Emergency stop checks (non-blocking debounce) ──
  static uint32_t btnFirstLowMs = 0;
  static bool     btnPending    = false;
  if (digitalRead(PIN_BTN) == LOW && state != STOPPED) {
    if (!btnPending) {
      btnPending    = true;
      btnFirstLowMs = millis();
    } else if (millis() - btnFirstLowMs >= 50) {
      emergencyStop("Button pressed");
      return;
    }
  } else {
    btnPending = false;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if ((c == 's' || c == 'S') && state != STOPPED) {
      emergencyStop("Serial stop command");
      return;
    }
  }

  if (state == STOPPED || state == IDLE) return;

  uint32_t now = millis();
  uint32_t stepsDone = step_pulse_count - stepsAtStart;

  // ── Progress printout every 500 ms ──
  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;
    float jointDeg = (float)stepsDone / 800.0f * 360.0f / 3.0f;
    Serial.print("  steps: ");
    Serial.print(stepsDone);
    Serial.print("/");
    Serial.print(STEPS_PER_DIRECTION);
    Serial.print("  (~");
    Serial.print(jointDeg, 1);
    Serial.println(" deg joint)");
  }

  // ── State machine ──
  switch (state) {
    case MOVING_CCW:
      if (stepsDone >= STEPS_PER_DIRECTION) {
        stopMotor();
        state = PAUSING_1;
        stateEnteredMs = now;
        Serial.println(">>> CCW complete. Pausing 2s...");
      }
      break;

    case PAUSING_1:
      if (now - stateEnteredMs >= PAUSE_MS) {
        state = MOVING_CW;
        stepsAtStart = step_pulse_count;
        noInterrupts();
        isr_step_budget  = STEPS_PER_DIRECTION + 2;  // hard ISR safety cap
        cmd_pulses_per_s = SPEED_PULSES_PER_S;
        interrupts();
        Serial.println(">>> Moving CLOCKWISE...");
      }
      break;

    case MOVING_CW:
      if (stepsDone >= STEPS_PER_DIRECTION) {
        stopMotor();
        state = PAUSING_2;
        stateEnteredMs = now;
        Serial.println(">>> CW complete. Pausing 2s...");
      }
      break;

    case PAUSING_2:
      if (now - stateEnteredMs >= PAUSE_MS) {
        cycleCount++;
        state = MOVING_CCW;
        stepsAtStart = step_pulse_count;
        noInterrupts();
        isr_step_budget  = STEPS_PER_DIRECTION + 2;  // hard ISR safety cap
        cmd_pulses_per_s = -SPEED_PULSES_PER_S;
        interrupts();
        Serial.print("[Cycle ");
        Serial.print(cycleCount);
        Serial.println("] >>> Moving COUNTER-CLOCKWISE...");
      }
      break;

    default:
      break;
  }
}
