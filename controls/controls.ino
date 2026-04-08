#include <Wire.h>
#include <Servo.h>
#include "controls_config.h"
#include <AS5600.h>
#include <Arduino.h>
#include <PID_v1.h>

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Current operating mode
int currentMode = MODE_IDLE;

// Joint target angles (set by JOINTS_TO_ANGLE commands)
float jointTargetAngles[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Joint enable/disable status (set by JOINT_EN commands)
// Defaults are copied from DEFAULT_JOINT_EN[] in setup()
bool jointEnabled[NUM_JOINTS];

// Current encoder readings
float encoderAngles[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// SG90 gripper servo (end effector)
Servo sg90Gripper;
int gripperTargetAngle = SG90_DEFAULT_ANGLE;  // Current target angle for gripper

// Joint 6 servo (25kg digital servo, open-loop control)
Servo joint6Servo;
float joint6LastCommandedAngle = 0.0;  // Track last commanded angle for telemetry

// ============================================================================
// STEPPER PULSE ENGINE — ISR STATE & SHARED VARIABLES
// ============================================================================

// Per-motor ISR-internal state (only touched inside stepISR)
struct StepperISRState {
  volatile uint32_t accum;            // Fixed-point phase accumulator (Q16.16)
  volatile bool     step_high;        // true while STEP pin is HIGH (needs LOW next tick)
  volatile uint8_t  dir_guard_remain; // ticks remaining after DIR change
  volatile bool     cur_dir_forward;  // current DIR pin state (true = HIGH)
  volatile uint32_t step_pulse_count; // diagnostic: total STEP rising edges
};

static StepperISRState isrState[NUM_STEPPERS];  // zero-initialized

// Shared state (foreground loop ↔ ISR, must be volatile)
// Signed pulse rate in microsteps/s. Updated atomically by control loop.
static volatile int32_t cmdPulsesPerS[NUM_STEPPERS] = {0};

// Fixed-point reciprocal of TICK_HZ, computed once in setup()
// Used inside ISR: (abs_cmd * FIXED_INV_TICK) >> 16 gives steps-per-tick in Q16.16
static uint32_t FIXED_INV_TICK;

IntervalTimer stepTimer;

// ============================================================================
// PID CONTROLLER STATE (per stepper joint)
// ============================================================================

static double pidInput[NUM_STEPPERS]    = {0};  // PID input  = shortest-path error (deg)
static double pidOutput[NUM_STEPPERS]   = {0};  // PID output = velocity command (pulses/s)
static double pidSetpoint[NUM_STEPPERS] = {0};  // always 0 (error-based formulation)

// PID debug: snapshot of last-computed error and command for debug telemetry
static float  pidDebugError[NUM_STEPPERS]  = {0};  // last error (deg, signed)
static int32_t pidDebugCmd[NUM_STEPPERS]   = {0};  // last cmdPulsesPerS sent to ISR

// Telemetry counter for PID debug packets (sent every N telemetry cycles in MOVE mode)
static uint8_t pidDebugTelemCounter = 0;
#define PID_DEBUG_TELEM_INTERVAL 5  // send PID debug every 5th telemetry cycle

// PID instances (heap-allocated in setup, one per stepper joint)
PID* posPID[NUM_STEPPERS];

// Derived pulse-rate limits (computed in setup from config arrays)
static int32_t minPulsesPerS[NUM_STEPPERS];
static int32_t maxPulsesPerS[NUM_STEPPERS];

// ============================================================================
// HARDWARE / UTILITY
// ============================================================================

// Read button state with debouncing (active low: pressed = LOW)
// Latches state for 500ms after any change to prevent bouncing
int readButton() {
  static int latchedState = 0;           // Current latched button state
  static unsigned long lastChangeMs = 0;  // Last time state changed
  // Use debounce period from config
  const unsigned long DEBOUNCE_MS = BUTTON_DEBOUNCE_MS;
  // Read current physical button state
  int currentPhysicalState = (digitalRead(PUSH_SWITCH_pin) == LOW) ? 1 : 0;
  unsigned long now = millis();
  if ((now - lastChangeMs) >= DEBOUNCE_MS) {
    if (currentPhysicalState != latchedState) {
      latchedState = currentPhysicalState;
      lastChangeMs = now;
    }
  }
  return latchedState;
}



// Use the AS5600 library's API (readAngle()) and the TCA9548A mux
AS5600 encoder;

// Conversion: AS5600 raw (0..4095) to degrees
const float RAW_TO_DEG = 360.0 / 4096.0; // 0.087890625

// Select which channel on the TCA9548A to use (0-7)
// Includes settling delay for stable mux switching
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(100);
}

// Check if TCA9548A is present on the I2C bus
bool tcaPresent() {
  Wire.beginTransmission(TCA9548A_ADDR);
  return (Wire.endTransmission() == 0);
}

// Scan all TCA channels and report detected I2C devices
// Useful for debugging encoder connections
void scanTCAChannels() {
  Serial.println("\n=== Scanning TCA9548A Channels ===");
  for (uint8_t ch = 0; ch <= 7; ch++) {
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(": ");
    
    selectMuxChannel(ch);
    
    bool foundDevice = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        if (foundDevice) Serial.print(", ");
        Serial.print("0x");
        if (addr < 16) Serial.print("0");
        Serial.print(addr, HEX);
        foundDevice = true;
      }
    }
    
    if (!foundDevice) {
      Serial.print("No devices found");
    }
    Serial.println();
  }
  Serial.println("=== Scan Complete ===\n");
}

// Read all encoder positions and update encoderAngles[]
// Always reads encoders for wired joints regardless of enable state,
// so telemetry and PID always have the true physical position.
// NOTE: Joint 6 (index 5) uses a servo with no encoder - always skip it.
void readEncoders() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    // Joint 6 (index 5) is servo-controlled with no encoder - skip entirely
    // Do not attempt I2C communication on mux channel 7 for Joint 6
    if (i == 5) {
      encoderAngles[i] = joint6LastCommandedAngle;  // Use last commanded angle
      continue;
    }
    
    // Skip joints with no stepper wired (placeholder pin 0) —
    // these have no encoder connected, so I2C would hang.
    if (i < NUM_STEPPERS && STEP_PINS[i] == 0) {
      continue;  // leave encoderAngles[i] at its previous value
    }
    
    // Look up the TCA9548A channel for this joint from config
    uint8_t channel = MUX_CHANNELS[i];

    // Select the MUX channel for this encoder
    selectMuxChannel(channel);
    // Note: selectMuxChannel now includes settling delay

    // Read angle from AS5600 on the selected channel
    // The AS5600 library handles communication internally
    long raw = encoder.readAngle();
    
    // Check for valid reading (AS5600 returns 0-4095)
    // If encoder is disconnected, reading might be stuck at 0 or max
    encoderAngles[i] = raw * RAW_TO_DEG;
  }

  // Deselect all channels
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ============================================================================
// STEPPER PULSE ENGINE — ISR (runs at TICK_HZ, e.g. 40 kHz)
// ============================================================================
// This ISR services ALL stepper joints on every tick.
// Rules: NO I2C, NO Serial, NO float, NO delay().
// Use digitalWriteFast() for all pin writes.
// Total execution must stay well under 25 µs (at 40 kHz).

void stepISR() {
  for (uint8_t j = 0; j < NUM_STEPPERS; j++) {
    StepperISRState& s = isrState[j];

    // Phase 1: Complete the pulse — bring STEP low after 1-tick HIGH.
    if (s.step_high) {
      digitalWriteFast(STEP_PINS[j], LOW);
      s.step_high = false;
      continue;  // This motor is done for this tick
    }

    // Phase 2: Direction-change guard countdown.
    if (s.dir_guard_remain > 0) {
      s.dir_guard_remain--;
      continue;
    }

    // Phase 3: Accumulate and maybe emit a pulse.
    int32_t cmd = cmdPulsesPerS[j];
    if (!jointEnabled[j] || cmd == 0) {
      s.accum = 0;
      continue;
    }

    bool desired_fwd = (cmd > 0);
    if (MOTOR_DIR_INVERT[j]) desired_fwd = !desired_fwd;

    uint32_t abs_cmd;
    if (cmd > 0) {
      abs_cmd = (uint32_t)cmd;
    } else {
      abs_cmd = (uint32_t)(-cmd);
    }

    // Direction change → update pin, start guard, reset accumulator.
    if (desired_fwd != s.cur_dir_forward) {
      s.cur_dir_forward = desired_fwd;
      if (desired_fwd) {
        digitalWriteFast(DIR_PINS[j], HIGH);
      } else {
        digitalWriteFast(DIR_PINS[j], LOW);
      }
      s.dir_guard_remain = DIR_GUARD_TICKS;
      s.accum = 0;
      continue;
    }

    // Fixed-point accumulate:
    // (abs_cmd * FIXED_INV_TICK) gives steps-per-tick in Q16.16, >>16 drops fractional bits.
    uint32_t inc = (uint32_t)(((uint64_t)abs_cmd * FIXED_INV_TICK) >> 16);
    s.accum += inc;

    if (s.accum >= ONE_STEP) {
      digitalWriteFast(STEP_PINS[j], HIGH);
      s.step_high = true;
      s.accum -= ONE_STEP;
      s.step_pulse_count++;
    }
  }
}


// Send telemetry packet: TYPE=DATA,CMD=JOINT_ANGLES,ENCODER_1_ANGLE=X.X,...,BUTTON=N\n
// Only sends encoder angles for joints that are enabled
void sendTelemetry() {
  // Only send telemetry in CALIBRATION (1) or MOVE (2) modes
  if (currentMode != MODE_CALIBRATION && currentMode != MODE_MOVE) {
    return;
  }
  
  Serial.print("TYPE=DATA,CMD=JOINT_ANGLES");
  // Send encoder data for enabled joints
  // Joint 6 (index 5) always sends last commanded angle (no encoder)
  for (int i = 0; i < NUM_JOINTS; i++) {
    // Joint 6 is servo-controlled - always send its last commanded angle
    if (i == 5) {
      Serial.print(",ENCODER_6_ANGLE=");
      Serial.print(joint6LastCommandedAngle, 1);
      continue;
    }
    if (jointEnabled[i]) {
      Serial.print(",ENCODER_");
      Serial.print(i + 1);
      Serial.print("_ANGLE=");
      Serial.print(encoderAngles[i], 1);
    }
  }
  // Add button state
  Serial.print(",BUTTON=");
  Serial.print(readButton());
  Serial.println();
}


// Send PID debug telemetry: error, PID output, command, target, DIR pin state per enabled joint.
// Only called in MOVE mode, every PID_DEBUG_TELEM_INTERVAL telemetry cycles.
// Format: TYPE=DATA,CMD=PID_DEBUG,J2_ERR=X.X,J2_OUT=X,J2_CMD=X,J2_TGT=X.X,J2_DIR=H,...
void sendPidDebug() {
  Serial.print("TYPE=DATA,CMD=PID_DEBUG");
  for (int j = 0; j < NUM_STEPPERS; j++) {
    if (!jointEnabled[j]) continue;
    int jn = j + 1;  // 1-indexed joint number
    Serial.print(",J");  Serial.print(jn);
    Serial.print("_ERR="); Serial.print(pidDebugError[j], 2);
    Serial.print(",J");  Serial.print(jn);
    Serial.print("_OUT="); Serial.print((int32_t)pidOutput[j]);
    Serial.print(",J");  Serial.print(jn);
    Serial.print("_CMD="); Serial.print(pidDebugCmd[j]);
    Serial.print(",J");  Serial.print(jn);
    Serial.print("_TGT="); Serial.print(jointTargetAngles[j], 1);
    Serial.print(",J");  Serial.print(jn);
    Serial.print("_DIR="); Serial.print(isrState[j].cur_dir_forward ? "H" : "L");
  }
  Serial.println();
}

// Update physical EN pins on all stepper drivers.
// A driver is enabled only when we are in MOVE mode AND that joint is enabled.
// EN_ACTIVE_LEVEL = LOW means LOW = enabled, HIGH = disabled.
void updateStepperEnPins() {
  for (int j = 0; j < NUM_STEPPERS; j++) {
    if (STEP_PINS[j] == 0) continue;  // skip un-wired joints
    if (currentMode == MODE_MOVE && jointEnabled[j]) {
      digitalWrite(EN_PINS[j], EN_ACTIVE_LEVEL);       // LOW = motor powered
    } else {
      digitalWrite(EN_PINS[j], EN_DISABLE_LEVEL);      // HIGH = motor free
    }
  }
}

// Send ACK packet: TYPE=ACK,CMD=<original_cmd>,...\n
void sendAck(const String& originalPacket) {
  delay(10); // Small delay to ensure Serial buffer is ready
  String ack = originalPacket;
  ack.replace("TYPE=CMD", "TYPE=ACK");
  Serial.println(ack);
}

// ============================================================================
// PACKET LISTENING AND PARSING
// ============================================================================

String inLine = "";

// Listen for incoming packets on serial and parse them
void listenPackets() {
  while (Serial.available() > 0) {
    int b = Serial.read();
    if (b < 0) break;
    
    char c = (char)b;
    if (c == '\r') continue;
    if (c == '\n') {
      // Complete packet received - extract values and update state
      if (inLine.length() > 0) {
        parseAndExecutePacket(inLine);
      }
      inLine = "";
    } else {
      inLine += c;
      if (inLine.length() > PACKET_MAX_LEN) {
        inLine = "";
      }
    }
  }
}

// Extract the value for a "KEY=" field from a CSV packet.
// Returns the substring between "KEY=" and the next ',' or '\n' or end-of-string.
// Returns "" if the key is not found.
String extractField(const String& packet, const String& key) {
  int pos = packet.indexOf(key);
  if (pos == -1) return "";
  int valStart = pos + key.length();
  int valEnd = packet.indexOf(',', valStart);
  if (valEnd == -1) valEnd = packet.indexOf('\n', valStart);
  if (valEnd == -1) valEnd = packet.length();
  return packet.substring(valStart, valEnd);
}

// Parse packet and extract values into global state variables
void parseAndExecutePacket(const String& packet) {
  if (!packet.startsWith("TYPE=CMD")) return;
  
  // Extract CMD name
  String cmd = extractField(packet, "CMD=");
  if (cmd.length() == 0) return;
  
  // ========== Extract values based on command type ==========
  
  if (cmd == CMD_SET_MODE) {
    String modeVal = extractField(packet, "MODE=");
    if (modeVal.length() == 0) return;
    
    int newMode = modeVal.toInt();
    if (newMode >= 0 && newMode <= 3) {
      // Safety: if leaving MOVE mode, immediately halt all stepper motors
      if (currentMode == MODE_MOVE && newMode != MODE_MOVE) {
        noInterrupts();
        for (int j = 0; j < NUM_STEPPERS; j++) {
          cmdPulsesPerS[j] = 0;
        }
        interrupts();
      }
      // Capture current position when entering MOVE mode ("servo engage").
      // This sets target = current so error is zero and nothing moves
      // until a JOINTS_TO_ANGLE command changes the targets.
      if (newMode == MODE_MOVE && currentMode != MODE_MOVE) {
        readEncoders();  // Fresh readings (may be stale if coming from IDLE)
        for (int j = 0; j < NUM_STEPPERS; j++) {
          jointTargetAngles[j] = encoderAngles[j];
        }
        // Joint 6 (servo, no encoder) — hold its last commanded angle
        jointTargetAngles[5] = joint6LastCommandedAngle;
      }
      currentMode = newMode;
      updateStepperEnPins();  // Enable/disable drivers based on new mode
      sendAck(packet);
    }
  }
  
  else if (cmd == CMD_JOINTS_TO_ANGLE) {
    // Extract JOINT_n_ANG parameters
    for (int i = 1; i <= NUM_JOINTS; i++) {
      String val = extractField(packet, "JOINT_" + String(i) + "_ANG=");
      if (val.length() > 0) {
        jointTargetAngles[i - 1] = val.toFloat();
      }
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_JOINT_EN) {
    // Extract JOINT_n_EN parameters
    for (int i = 1; i <= NUM_JOINTS; i++) {
      String val = extractField(packet, "JOINT_" + String(i) + "_EN=");
      if (val.length() > 0) {
        bool newEn = (val.toInt() == 1);
        // Snap target to current position when enabling a joint so PID
        // doesn't jump to a stale target from before the joint was disabled.
        if (newEn && !jointEnabled[i - 1] && currentMode == MODE_MOVE) {
          jointTargetAngles[i - 1] = encoderAngles[i - 1];
        }
        jointEnabled[i - 1] = newEn;
      }
    }
    updateStepperEnPins();  // Toggle physical EN pins to match new flags
    sendAck(packet);
  }
  
  else if (cmd == CMD_ESTOP) {
    // Emergency stop - disable all joints AND halt all stepper motors immediately
    noInterrupts();
    for (int j = 0; j < NUM_STEPPERS; j++) {
      cmdPulsesPerS[j] = 0;
    }
    interrupts();
    for (int i = 0; i < NUM_JOINTS; i++) {
      jointEnabled[i] = false;
    }
    updateStepperEnPins();  // Physically disable all drivers
    sendAck(packet);
  }
  
  else if (cmd == CMD_CALIBRATE_JOINT) {
    String jointId = extractField(packet, "JOINT_ID=");
    if (jointId.length() > 0) {
      // TODO: Implement calibration logic for joint (1-6)
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_GRIP_CNTL) {
    String gripVal = extractField(packet, "GRIP_ANGLE=");
    if (gripVal.length() > 0) {
      int angle = gripVal.toInt();
      // Constrain to valid SG90 range
      angle = constrain(angle, SG90_MIN_ANGLE, SG90_MAX_ANGLE);
      gripperTargetAngle = angle;
      
      // Move gripper immediately in MOVE mode
      if (currentMode == MODE_MOVE) {
        sg90Gripper.write(gripperTargetAngle);
      }
    }
    sendAck(packet);
  }
}

// ============================================================================
// MODE-SPECIFIC CODE
// ========================lso i notice it so====================================================

// MODE 0: IDLE - System initialization, safe state
void modeIdleCode() {
  // In IDLE mode, we maintain current joint enable states
  // Joints can be enabled/disabled via JOINT_EN commands
  // Safety: Emergency stop (ESTOP) command will disable all joints
}

// MODE 1: CALIBRATION - Run calibration procedures
void modeCalibrationCode() {
  // Calibration procedures handled by GUI
  // Teensy sends telemetry, GUI captures button presses
}

// MODE 2: MOVE - Closed-loop PID position control for steppers + servo pass-through
// The PID loop runs at a fixed rate (CONTROL_PERIOD_MS) independent of loop() timing.
// Encoder readings are already in encoderAngles[] (updated by readEncoders() in main loop).
void modeMoveCode() {
  // ── Fixed-rate control loop gate ──
  static uint32_t lastControlMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastControlMs < CONTROL_PERIOD_MS) {
    // Not time yet — still handle servo pass-through every call
    goto servo_update;
  }
  lastControlMs = nowMs;

  // ── PID control for each stepper joint (Joints 1-5) ──
  for (int j = 0; j < NUM_STEPPERS; j++) {
    // Skip disabled joints — also zero their ISR command to stop stepping
    if (!jointEnabled[j]) {
      noInterrupts();
      cmdPulsesPerS[j] = 0;
      interrupts();
      continue;
    }

    // 1. Encoder angle already in encoderAngles[j] (from readEncoders())

    // 2. Compute shortest-path signed error (wrap-safe)
    //    Example: Current=5°, Target=290° → error = -75° (go backwards)
    float error = jointTargetAngles[j] - encoderAngles[j];
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 3. Feed error to PID (setpoint is always 0, error-based formulation)
    //    Arduino PID computes: output = Kp * (setpoint - input) = Kp * (0 - input)
    //    So we negate the error: input = -error → output = Kp * (0 - (-error)) = +Kp * error
    pidInput[j] = (double)(-error);
    posPID[j]->Compute();

    // 4. Apply deadband + clamp to [minPulsesPerS .. maxPulsesPerS]
    int32_t newCmd;
    if (fabsf(error) < DEADBAND_DEG[j]) {
      newCmd = 0;
    } else {
      newCmd = (int32_t)pidOutput[j];
      // Enforce minimum speed: if PID wants to move but below min, bump up
      if (newCmd > 0 && newCmd < minPulsesPerS[j])  newCmd =  minPulsesPerS[j];
      if (newCmd < 0 && newCmd > -minPulsesPerS[j]) newCmd = -minPulsesPerS[j];
      // Enforce maximum speed (PID output limits handle this, but belt-and-suspenders)
      if (newCmd >  maxPulsesPerS[j]) newCmd =  maxPulsesPerS[j];
      if (newCmd < -maxPulsesPerS[j]) newCmd = -maxPulsesPerS[j];
    }

    // 5. Snapshot for PID debug telemetry
    pidDebugError[j] = error;
    pidDebugCmd[j]   = newCmd;

    // 6. Atomic update of the shared ISR variable
    noInterrupts();
    cmdPulsesPerS[j] = newCmd;
    interrupts();
  }

servo_update:
  // ── Joint 6: servo pass-through (every loop call, not gated) ──
  if (jointEnabled[5]) {
    int servoAngle = constrain((int)jointTargetAngles[5],
                               JOINT6_SERVO_MIN_ANGLE,
                               JOINT6_SERVO_MAX_ANGLE);
    joint6Servo.write(servoAngle);
    joint6LastCommandedAngle = jointTargetAngles[5];
  }
}

// ============================================================================
// SETUP AND MAIN LOOP
// ============================================================================

// Telemetry timing
unsigned long lastTelemMs = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000); // Wait for serial to initialize
  
  Serial.println("\n\n=== 6-DOF Robotic Arm Control System ===");
  Serial.println("Initializing...");

  // Copy default joint-enable flags from config array
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointEnabled[i] = DEFAULT_JOINT_EN[i];
  }
  
  // Initialize button pin with internal pullup (active low)
  pinMode(PUSH_SWITCH_pin, INPUT_PULLUP);
  
  Wire.begin();
  Serial.println("I2C initialized");

  // Check if TCA9548A multiplexer is present
  if (tcaPresent()) {
    Serial.println("TCA9548A multiplexer detected");
    scanTCAChannels(); // Scan all channels for connected devices
  } else {
    Serial.println("WARNING: TCA9548A multiplexer NOT detected!");
  }

  // Initialize AS5600 library
  encoder.begin();
  Serial.println("AS5600 library initialized");

  // Initialize SG90 gripper servo (end effector open/close)
  sg90Gripper.attach(SG90_GRIPPER_pin);
  sg90Gripper.write(SG90_DEFAULT_ANGLE);
  Serial.println("SG90 gripper servo initialized");

  // Initialize Joint 6 servo (25kg digital servo, open-loop control)
  // Uses JOINT6_SERVO_pin (Pin 39) - range 60-180° within Servo.write() limits
  joint6Servo.attach(JOINT6_SERVO_pin);
  joint6Servo.write(JOINT6_SERVO_DEFAULT_ANGLE);  // Start at 120°
  joint6LastCommandedAngle = JOINT6_SERVO_DEFAULT_ANGLE;
  Serial.println("Joint 6 servo initialized at 120 deg default position");

  // ── Stepper motor initialization (Joints 1-5) ──
  // Compute derived pulse-rate limits from config arrays
  for (int j = 0; j < NUM_STEPPERS; j++) {
    minPulsesPerS[j] = (int32_t)MICROSTEPS[j] * MIN_FULL_STEPS_PER_S[j];
    maxPulsesPerS[j] = (int32_t)MICROSTEPS[j] * MAX_FULL_STEPS_PER_S[j];
  }

  // Precompute fixed-point reciprocal of TICK_HZ for ISR math
  FIXED_INV_TICK = (uint32_t)((1ULL << 32) / TICK_HZ);

  // Initialize stepper pins (STEP, DIR, EN) for each joint
  for (int j = 0; j < NUM_STEPPERS; j++) {
    // Skip un-wired joints (pin == 0 is placeholder)
    if (STEP_PINS[j] == 0) continue;

    pinMode(STEP_PINS[j], OUTPUT);
    digitalWriteFast(STEP_PINS[j], LOW);

    pinMode(DIR_PINS[j], OUTPUT);
    digitalWriteFast(DIR_PINS[j], HIGH);
    isrState[j].cur_dir_forward = true;   // Sync ISR state with physical DIR=HIGH pin

    pinMode(EN_PINS[j], OUTPUT);
    digitalWrite(EN_PINS[j], EN_DISABLE_LEVEL);  // Start DISABLED (enabled only in MOVE mode)
  }
  Serial.print("Stepper pins initialized for ");
  Serial.print(NUM_STEPPERS);
  Serial.println(" joints");

  // Initialize PID controllers (one per stepper joint)
  for (int j = 0; j < NUM_STEPPERS; j++) {
    posPID[j] = new PID(&pidInput[j], &pidOutput[j], &pidSetpoint[j],
                        PID_KP[j], PID_KI[j], PID_KD[j], DIRECT);
    posPID[j]->SetMode(AUTOMATIC);
    posPID[j]->SetOutputLimits(-(double)maxPulsesPerS[j], (double)maxPulsesPerS[j]);
    posPID[j]->SetSampleTime(1);  // We gate timing ourselves via CONTROL_PERIOD_MS
  }
  Serial.println("PID controllers initialized (P-only, error-based)");

  // Start the stepper pulse engine ISR (AFTER all I2C/encoder init)
  // This must be last so the ISR doesn't fire while I2C is still being set up
  bool timerOk = stepTimer.begin(stepISR, 1000000.0f / (float)TICK_HZ);
  Serial.print("Step timer (");
  Serial.print(TICK_HZ);
  Serial.print(" Hz) = ");
  Serial.println(timerOk ? "OK" : "FAIL");

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, LOW);
  
  currentMode = MODE_IDLE;
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Waiting for GUI commands...\n");
}

void loop() {
  // 1) ALWAYS process incoming commands FIRST (highest priority)
  // This ensures ACKs are sent immediately without blocking from telemetry
  listenPackets();
  
  // 2) Read encoder positions and send telemetry (only in modes 1 and 2)
  // CRITICAL: Only send telemetry if no incoming commands are pending
  // This prevents encoder read delays from blocking ACK responses
  unsigned long now = millis();
  if ((now - lastTelemMs) >= TELEM_INTERVAL_MS) {
    // Check if there are pending incoming bytes - if so, skip telemetry this cycle
    // to prioritize processing commands and sending ACKs
    if (!Serial.available()) {
      lastTelemMs = now;
      // Only read encoders and send telemetry in CALIBRATION or MOVE modes
      if (currentMode == MODE_CALIBRATION || currentMode == MODE_MOVE) {
        readEncoders();
        sendTelemetry();
        // In MOVE mode, also send PID debug info every Nth cycle
        if (currentMode == MODE_MOVE) {
          pidDebugTelemCounter++;
          if (pidDebugTelemCounter >= PID_DEBUG_TELEM_INTERVAL) {
            pidDebugTelemCounter = 0;
            sendPidDebug();
          }
        }
      }
      // Uncomment below for debug mode (direct Serial Monitor prints):
      // readEncoders();
      // printEncoderDebug();
    }
    // If Serial data is available, we'll skip telemetry this cycle and
    // process the command on the next loop iteration
  }
  
  // 3) Execute mode-specific behavior
  if (currentMode == MODE_IDLE) {
    modeIdleCode();
  }
  else if (currentMode == MODE_CALIBRATION) {
    modeCalibrationCode();
  }
  else if (currentMode == MODE_MOVE) {
    modeMoveCode();
  }
  else if (currentMode == MODE_RESERVED) {
    // Reserved for future use
  }
  delay(5);
}
