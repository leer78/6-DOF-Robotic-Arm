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
#define MUX_DELAY 5

// Map joints to TCA9548A mux channels
// Joint 1 on sc2/sd2, Joint 6 on sc7/sd7 (both disabled/not connected yet)
// Joints 2-5 are on channels 3-6 and connected
#define JOINT1_MUX_CH 2  // Not connected (disabled)
#define JOINT2_MUX_CH 3  // Connected encoder
#define JOINT3_MUX_CH 4  // Connected encoder
#define JOINT4_MUX_CH 5  // Connected encoder
#define JOINT5_MUX_CH 6  // Connected encoder
#define JOINT6_MUX_CH 7  // Not connected (disabled)

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

// Joint enable defaults (0 = disabled, 1 = enabled)
// All joints disabled by default until JOINT_EN signal received from GUI
#define DEFAULT_JOINT1_EN 0
#define DEFAULT_JOINT2_EN 1
#define DEFAULT_JOINT3_EN 1
#define DEFAULT_JOINT4_EN 1
#define DEFAULT_JOINT5_EN 1
#define DEFAULT_JOINT6_EN 0


// Telemetry configuration
#define TELEM_INTERVAL_MS 20
#define PACKET_MAX_LEN 256

// Button debounce (ms)
#define BUTTON_DEBOUNCE_MS 200

#endif // CONTROLS_CONFIG_H
