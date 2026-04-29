#ifndef CONTROLS_CONFIG_H
#define CONTROLS_CONFIG_H

// ============================================================================
// SERIAL PORT (mirrors gui/config.py)
// ============================================================================

#define BAUD_RATE 115200

// Sentinel for un-wired / unused pins.  Must be > max valid Teensy 4.1 pin (0-41).
// Use in place of literal 0 so Teensy pin 0 can be used as a real STEP/DIR/EN pin.
#define UNUSED_PIN 255

// ============================================================================
// SYSTEM DIMENSIONS
// ============================================================================

#define NUM_JOINTS   6  // Total joints (5 steppers + 1 servo)
#define NUM_STEPPERS 5  // Joints 1-5: stepper + encoder
                        // Joint 6:    servo (open-loop, no encoder)

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Misc
#define LED_pin           13
#define PUSH_SWITCH_pin   41

// Servos
#define JOINT6_SERVO_pin  39   // 25 kg digital servo (Joint 6)
#define SG90_GRIPPER_pin  38   // SG90 end-effector (grip open/close)

// Per-joint stepper pins: {STEP, DIR, EN}
// Index: [0]=J1  [1]=J2  [2]=J3  [3]=J4  [4]=J5
// Use UNUSED_PIN (255) as placeholder for un-wired joints (must be disabled via jointEnabled[]).
//
//  Joint 1 bring-up: STEP=0, DIR=1, EN=2 (Teensy 4.1 pins 0/1/2 → TB6600 PUL+/DIR+/ENA+)
//  Fill in remaining pins as hardware is connected.
//                                                J1   J2   J3   J4   J5
static const uint8_t STEP_PINS[NUM_STEPPERS] = {   0,   3,   8,  11,  27 };
static const uint8_t DIR_PINS[NUM_STEPPERS]  = {   1,   4,   9,  12,  28 };
static const uint8_t EN_PINS[NUM_STEPPERS]   = {   2,   5,  10,  24,  29 };

// ============================================================================
// I2C / TCA9548A MULTIPLEXER
// ============================================================================

#define TCA9548A_ADDR  0x70
#define AS5600_ADDR    0x36

// Map each joint index (0-5) to its TCA9548A mux channel.
// Joint 6 has no encoder — placeholder 0 (never read).
//                                                J1  J2  J3  J4  J5  J6
static const uint8_t MUX_CHANNELS[NUM_JOINTS] = {  2,  3,  4,  5,  6,  0 };

// ============================================================================
// JOINT ENABLE DEFAULTS
// ============================================================================
// false = disabled at boot, true = enabled at boot.
// Joint 2 is the only stepper enabled for bring-up testing.
// Joint 6 (servo) remains disabled unless you want it active too.
//                                                          J1     J2     J3     J4     J5    J6
static const bool DEFAULT_JOINT_EN[NUM_JOINTS] = {       true,  true, true, true, true, true };

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// SG90 gripper (end effector open/close)
#define SG90_MIN_ANGLE      68
#define SG90_MAX_ANGLE      112
#define SG90_DEFAULT_ANGLE  90

// Joint 6 servo (25 kg digital, range 60-180°)
#define JOINT6_SERVO_MIN_ANGLE      60
#define JOINT6_SERVO_MAX_ANGLE      180
#define JOINT6_SERVO_DEFAULT_ANGLE  120

// ============================================================================
// SERIAL PROTOCOL (mirrors gui/config.py)
// ============================================================================

// Mode IDs
#define MODE_IDLE        0
#define MODE_CALIBRATION 1
#define MODE_MOVE        2
#define MODE_RESERVED    3

// Command strings
#define CMD_SET_MODE         "SET_MODE"
#define CMD_JOINTS_TO_ANGLE  "JOINTS_TO_ANGLE"
#define CMD_JOINT_EN         "JOINT_EN"
#define CMD_ESTOP            "ESTOP"
#define CMD_CALIBRATE_JOINT  "CALIBRATE_JOINT"
#define CMD_GRIP_CNTL        "GRIP_CNTL"

// Telemetry / packets
#define TELEM_INTERVAL_MS  20
#define PACKET_MAX_LEN     256

// Button debounce
#define BUTTON_DEBOUNCE_MS 200

// ============================================================================
// STEPPER PULSE ENGINE / ISR
// ============================================================================

// ISR tick rate (Hz).  One tick = 25 µs at 40 kHz.
// Max step rate per motor ≈ TICK_HZ / 2 (pulse needs high + low tick).
#define TICK_HZ          40000

// Ticks to wait after DIR change before next STEP pulse.
// 3 ticks @ 40 kHz ≈ 75 µs — above TB6600 minimum.
#define DIR_GUARD_TICKS  3

// PID / control-loop update period.  50 ms → 20 Hz.
#define CONTROL_PERIOD_MS 50

// TB6600 enable pin levels.
// EN_ACTIVE_LEVEL  = pin state that ENABLES the driver (motor powered).
// EN_DISABLE_LEVEL = pin state that DISABLES the driver (motor free).
// Most TB6600 modules: LOW = enabled, HIGH = disabled.
#define EN_ACTIVE_LEVEL   LOW
#define EN_DISABLE_LEVEL  HIGH

// Fixed-point one-step threshold (Q16.16)
#define ONE_STEP (1UL << 16)

// ============================================================================
// PER-JOINT STEPPER TUNING
// ============================================================================

// Motor direction inversion (per joint).
// true  = invert DIR pin logic (positive PID output → DIR LOW instead of HIGH).
// false = normal (positive PID output → DIR HIGH).
// Set true for any joint whose motor+driver wiring causes it to move the wrong
// way relative to increasing encoder angle.
//                                                          J1     J2    J3     J4     J5
static const bool MOTOR_DIR_INVERT[NUM_STEPPERS]          = { false, false, true,  true,  true  };

// Microstep setting (TB6600 DIP).  Must match physical DIP switch setting.
// J2 TB6600 is wired for 8 microsteps (1600 pulses/rev).
//                                                          J1   J2   J3   J4   J5
static const uint16_t MICROSTEPS[NUM_STEPPERS]            = {  4,   8,   4,   4,   4 };

// Full-step speed limits (before microstep multiplier).
// Derived pulse limits computed in setup(): MIN_PULSES = MICROSTEPS × MIN_FULL_STEPS, etc.
static const int32_t  MIN_FULL_STEPS_PER_S[NUM_STEPPERS]  = { 10,  10,  10,  10,  10 };
static const int32_t  MAX_FULL_STEPS_PER_S[NUM_STEPPERS]  = {120, 240, 120, 120, 120 };

// ============================================================================
// PER-JOINT PID TUNING
// ============================================================================
// Start P-only (Ki=0, Kd=0) per control_plan.md.  Tune per joint.
// NOTE: J2 Kp is 2× others to compensate for 8-microstep (vs 4) — equalizes deg/s response.
// NOTE: J4 Kp is 3× others to compensate for 3:1 gear ratio — reduces gravity sag.
//                                                    J1    J2    J3    J4    J5
static const double PID_KP[NUM_STEPPERS]           = { 15.0, 120.0,  20.0, 15.0,  20.0 };
static const double PID_KI[NUM_STEPPERS]           = { 0.0,  0.0,  0.0,  0.0,  0.0 };
static const double PID_KD[NUM_STEPPERS]           = { 0.0,  0.0,  0.0,  0.0,  0.0 };

// Deadband (degrees).  Error inside this band → motor speed = 0.
static const float  DEADBAND_DEG[NUM_STEPPERS]     = { 0.75, 0.75, 0.75, 0.75, 0.75 };

#endif // CONTROLS_CONFIG_H
