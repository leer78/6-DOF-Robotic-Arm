/*
 * controls_test.ino  —  Milestone 2: Closed-Loop PID Position Control
 *
 * Drives one TB6600 stepper via IntervalTimer ISR (pulse engine).
 * Reads AS5600 encoder via TCA9548A I2C mux for position feedback.
 * PID_v1 (Brett Beauregard) computes velocity commands from
 * wrap-safe shortest-path angular error.
 *
 * Serial Monitor: type a float (0.0 – 359.9) + Enter to set target angle.
 */

#include <Wire.h>
#include <AS5600.h>
#include <PID_v1.h>

// ── Pins ─────────────────────────────────────
static const uint8_t PIN_STEP = 27;   // TB6600 PUL+
static const uint8_t PIN_DIR  = 28;   // TB6600 DIR+
static const uint8_t PIN_EN   = 29;   // TB6600 ENA+
static const uint8_t PIN_BTN  = 41;   // push button (to GND with internal pullup)
// I2C: Teensy 4.1 default Wire uses pin 19 (SCL) and pin 18 (SDA)

// ── Encoder / Mux ────────────────────────────
static const uint8_t TCA9548A_ADDR   = 0x70; // I2C address of the TCA9548A mux
static const uint8_t ENCODER_MUX_CH  = 6;    // TCA9548A channel for AS5600 (sd6/sc6)
static const float   RAW_TO_DEG      = 360.0f / 4096.0f; // AS5600 12-bit → degrees

AS5600 encoder;  // AS5600 library instance (communicates over Wire)

// HIGH = driver enabled.  Flip to LOW if your wiring is active-low.
static const uint8_t EN_ACTIVE_LEVEL = LOW; // TB6600 enable is active-low

// ── Tuning ───────────────────────────────────
static const uint32_t TICK_HZ         = 40000; // ISR rate (25 µs per tick)
static const uint16_t MICROSTEPS          = 4;     // TB6600 DIP setting (4 => 800 pulses/rev)
static const int32_t  MIN_FULL_STEPS_PER_S = 10;     // minimum non-zero speed (full-steps/s)
static const int32_t  MAX_FULL_STEPS_PER_S = 120;    // maximum speed (full-steps/s)
static const int32_t  MIN_PULSES_PER_S     = (int32_t)MICROSTEPS * MIN_FULL_STEPS_PER_S; // 4
static const int32_t  MAX_PULSES_PER_S     = (int32_t)MICROSTEPS * MAX_FULL_STEPS_PER_S; // 80
static const uint8_t  DIR_GUARD_TICKS      = 3;     // ticks to wait after DIR flip (~75 µs)

// ── PID Tuning ───────────────────────────────
static const uint32_t CONTROL_PERIOD_MS = 50;     // control loop period (50 ms → 20 Hz)
static const double   PID_KP           = 5;     // proportional gain  (pulses/s per deg error)
static const double   PID_KI           = 0.0;     // integral gain      (start P-only)
static const double   PID_KD           = 0.0;     // derivative gain
static const float    DEADBAND_DEG     = 0.75f;    // error band where motor is commanded to 0

// ── Fixed-point (Q16.16) ─────────────────────
static const uint32_t ONE_STEP = 1UL << 16;
static uint32_t FIXED_INV_TICK;                 // (1<<32)/TICK_HZ, computed in setup()

// ── Shared state  (loop ↔ ISR, volatile) ─────
static volatile int32_t cmd_pulses_per_s = 0;   // signed PULSES/s (microsteps/s) sent to STEP pin
static volatile bool    motor_enabled   = false;

// ── ISR-only state ───────────────────────────
static volatile uint32_t accum           = 0;
static volatile bool     step_high       = false;
static volatile uint8_t  dir_guard_remain = 0;
static volatile bool     cur_dir_forward = true;
static volatile uint32_t step_pulse_count = 0; // counts STEP rising edges

IntervalTimer stepTimer;

// ── PID state ────────────────────────────────
static float  target_angle = 90.0f;   // desired angle (deg), updated via Serial
static double pidInput     = 0.0;    // PID input  = shortest-path error (deg)
static double pidOutput    = 0.0;    // PID output = velocity command (pulses/s)
static double pidSetpoint  = 0.0;    // always 0 (error-based formulation)

PID posPID(&pidInput, &pidOutput, &pidSetpoint,
           PID_KP, PID_KI, PID_KD, DIRECT);

// ── Serial input buffer (non-blocking) ───────
static char    serialBuf[32];
static uint8_t serialIdx = 0;

// ── ISR (40 kHz) — no Serial, no float, no delay ──
void stepISR() {
  // Phase 1: complete the pulse — bring STEP low after 1-tick HIGH.
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

  // Phase 3: accumulate and maybe emit a pulse.
  int32_t cmd = cmd_pulses_per_s;
  if (!motor_enabled || cmd == 0) {
    accum = 0;
    return;
  }

  bool desired_fwd = (cmd > 0);
  uint32_t abs_cmd;
  if (desired_fwd) {
    abs_cmd = (uint32_t)cmd;
  } else {
    abs_cmd = (uint32_t)(-cmd);
  }

  // Direction change → update pin, start guard, reset accumulator.
  if (desired_fwd != cur_dir_forward) {
    cur_dir_forward = desired_fwd;
    if (desired_fwd) {
      digitalWriteFast(PIN_DIR, HIGH);
    } else {
      digitalWriteFast(PIN_DIR, LOW);
    }
    dir_guard_remain = DIR_GUARD_TICKS;
    accum = 0;
    return;
  }

  // Fixed-point accumulate:
  // (abs_cmd * FIXED_INV_TICK) gives steps-per-tick in Q16.16, >>16 drops fractional bits.
  uint32_t inc = (uint32_t)(((uint64_t)abs_cmd * FIXED_INV_TICK) >> 16);
  accum += inc;

  if (accum >= ONE_STEP) {
    digitalWriteFast(PIN_STEP, HIGH);
    step_high = true;
    accum -= ONE_STEP;
    step_pulse_count++;
  }
}

// ── setup ────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  pinMode(PIN_BTN,  INPUT_PULLUP);
  digitalWriteFast(PIN_STEP, LOW);
  // Make initial DIR pin match initial software direction state.
  if (cur_dir_forward) {
    digitalWriteFast(PIN_DIR, HIGH);
  } else {
    digitalWriteFast(PIN_DIR, LOW);
  }

  // Enable driver immediately so the button can control motion right away.
  digitalWrite(PIN_EN, EN_ACTIVE_LEVEL);
  motor_enabled = true;

  // (1<<32)/TICK_HZ precomputes a reciprocal for fast fixed-point math.
  FIXED_INV_TICK = (uint32_t)((1ULL << 32) / TICK_HZ);

  // ── I2C + encoder init BEFORE starting the ISR timer ──
  // (I2C uses interrupts internally; start it before the 40 kHz ISR)
  Wire.begin();  // Teensy 4.1 default: SDA=18, SCL=19
  delay(1000);   // Let I2C bus and devices power up (matches controls.ino)

  encoder.begin();  // Initialize AS5600 library

  while (!Serial) {}
  Serial.println("\n=== Milestone 2 — ISR + Encoder ===");

  // Scan the raw I2C bus for any devices (before mux selection).
  Serial.println("I2C bus scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  // Verify TCA9548A is on the bus.
  if (tcaPresent()) {
    Serial.println("TCA9548A found at 0x70");
  } else {
    Serial.println("ERROR: TCA9548A not found at 0x70!");
    Serial.println("  Check: SDA->pin18, SCL->pin19, 3.3V pull-ups, TCA powered");
  }

  // Select encoder mux channel and check AS5600.
  selectMuxChannel(ENCODER_MUX_CH);
  if (encoder.isConnected()) {
    Serial.print("AS5600 found on mux ch ");
    Serial.println(ENCODER_MUX_CH);
  } else {
    Serial.print("ERROR: AS5600 not found on mux ch ");
    Serial.println(ENCODER_MUX_CH);
  }

  // NOW start the step-pulse ISR (after I2C is verified working).
  bool timerOk = stepTimer.begin(stepISR, 1000000.0f / (float)TICK_HZ);
  Serial.print("Step timer=");
  Serial.println(timerOk ? "OK" : "FAIL");

  // ── PID initialization ──
  posPID.SetMode(AUTOMATIC);
  posPID.SetOutputLimits(-(double)MAX_PULSES_PER_S, (double)MAX_PULSES_PER_S);
  posPID.SetSampleTime(1);  // always compute when called; we gate timing with millis()

  Serial.println("Type a target angle (0.0 - 359.9) + Enter.");
  Serial.println("──────────────────────────────────────────");
}

// ── Mux helpers (same as controls.ino) ───────
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // set the single channel bit
  Wire.endTransmission();
  delayMicroseconds(100);    // allow mux to settle
}

bool tcaPresent() {
  Wire.beginTransmission(TCA9548A_ADDR);
  return (Wire.endTransmission() == 0);
}

// ── loop — PID closed-loop position control ─
void loop() {
  uint32_t nowMs = millis();

  // ── Non-blocking Serial input for target angle ──
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        float val = atof(serialBuf);
        if (val >= 0.0f && val < 360.0f) {
          target_angle = val;
          Serial.print(">>> New target: ");
          Serial.println(target_angle, 1);
        } else {
          Serial.println(">>> Invalid angle (must be 0.0 - 359.9)");
        }
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }

  // ── Fixed-rate control loop (20 Hz) ──
  static uint32_t lastControlMs = 0;
  if (nowMs - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = nowMs;

    // 1. Read encoder via mux.
    selectMuxChannel(ENCODER_MUX_CH);
    uint16_t raw        = encoder.readAngle();          // 0..4095 (12-bit)
    float    currentAngle = (float)raw * RAW_TO_DEG;    // 0..~360°

    // 2. Compute shortest-path signed error (wrap-safe).
    //    Example: Current=5°, Target=290° → error = -75° (go backwards).
    float error = target_angle - currentAngle;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 3. Feed error to PID (setpoint is always 0).
    pidInput = (double)error;
    posPID.Compute();

    // 4. Apply deadband + clamp to [MIN_PULSES_PER_S .. MAX_PULSES_PER_S].
    int32_t newCmd;
    if (fabsf(error) < DEADBAND_DEG) {
      newCmd = 0;
    } else {
      newCmd = (int32_t)pidOutput;
      // Enforce minimum speed: if PID wants to move but below min, bump up.
      if (newCmd > 0 && newCmd < MIN_PULSES_PER_S)  newCmd =  MIN_PULSES_PER_S;
      if (newCmd < 0 && newCmd > -MIN_PULSES_PER_S)  newCmd = -MIN_PULSES_PER_S;
      // Enforce maximum speed (PID output limits handle this, but belt-and-suspenders).
      if (newCmd >  MAX_PULSES_PER_S)  newCmd =  MAX_PULSES_PER_S;
      if (newCmd < -MAX_PULSES_PER_S)  newCmd = -MAX_PULSES_PER_S;
    }

    // 5. Atomic update of the shared ISR variable.
    noInterrupts();
    cmd_pulses_per_s = newCmd;
    interrupts();

    // 6. Debug telemetry.
    Serial.print("T=");
    Serial.print(target_angle, 1);
    Serial.print("  C=");
    Serial.print(currentAngle, 1);
    Serial.print("  E=");
    Serial.print(error, 1);
    Serial.print("  PID=");
    Serial.print((int32_t)pidOutput);
      Serial.print("  speed=");
      Serial.println((float)newCmd / (float)MICROSTEPS, 2);
  }
}
