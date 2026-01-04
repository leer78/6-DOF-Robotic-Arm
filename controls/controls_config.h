#ifndef CONTROLS_CONFIG_H
#define CONTROLS_CONFIG_H


// --- Serial port configs (mirrors gui/config.py) ---
#define SERIAL_PORT_NAME "COM3"
#define BAUD_RATE 115200
#define DRY_RUN false


// --- Pin definitions (all names end with _pin) ---
#define LED_pin 13
#define ENABLE_pin 8
#define BASE_SERVO_pin 2
#define SHOULDER_SERVO_pin 3
#define ELBOW_SERVO_pin 4
#define WRIST1_SERVO_pin 5
#define WRIST2_SERVO_pin 6
#define GRIPPER_SERVO_pin 7
#define PUSH_SWITCH_pin 37

// I2C / Multiplexer
#define TCA9548A_ADDR 0x70
#define AS5600_ADDR 0x36
// Number of encoders attached to the TCA9548A (use 6 for 6-DOF arm)
#define NUM_ENCODERS 6
#define MUX_DELAY 10


// --- Joint encoders (TCA9548A channel numbers) ---
// We use logical names scN/sdN as the user described (sc = SCL-line on that channel, sd = SDA-line)
#define sc0 0
#define sd0 0
#define sc1 1
#define sd1 1
#define sc2 2
#define sd2 2
#define sc3 3
#define sd3 3
#define sc4 4
#define sd4 4
#define sc5 5
#define sd5 5

// Map joints to mux channels
#define JOINT1_MUX_CH sc0
#define JOINT2_MUX_CH sc1
#define JOINT3_MUX_CH sc2
#define JOINT4_MUX_CH sc3
#define JOINT5_MUX_CH sc4
#define JOINT6_MUX_CH sc5

// ============================================================================
// SERIAL PROTOCOL CONFIGURATION (mirrors gui/config.py)
// ============================================================================

// Mode definitions
#define MODE_IDLE 0
#define MODE_CALIBRATION 1
#define MODE_MOVE 2
#define MODE_RESERVED 3

// Command definitions
#define CMD_SET_MODE "SET_MODE"
#define CMD_JOINTS_TO_ANGLE "JOINTS_TO_ANGLE"
#define CMD_JOINT_EN "JOINT_EN"
#define CMD_ESTOP "ESTOP"
#define CMD_CALIBRATE_JOINT "CALIBRATE_JOINT"

// Joint configuration
#define NUM_JOINTS 6
#define JOINT_MIN_ANGLE 0.0
#define JOINT_MAX_ANGLE 180.0


// Telemetry configuration
#define TELEM_INTERVAL_MS 20
#define PACKET_MAX_LEN 256

// Button debounce (ms)
#define BUTTON_DEBOUNCE_MS 200

#endif // CONTROLS_CONFIG_H
