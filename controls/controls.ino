#include <Wire.h>
#include <Servo.h>
#include "controls_config.h"
#include <AS5600.h>
#include <Arduino.h>

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Current operating mode
int currentMode = MODE_IDLE;

// Joint target angles (set by JOINTS_TO_ANGLE commands)
float jointTargetAngles[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Joint enable/disable status (set by JOINT_EN commands)
// Initialize with default values from config
bool jointEnabled[NUM_JOINTS] = {
  DEFAULT_JOINT1_EN,
  DEFAULT_JOINT2_EN,
  DEFAULT_JOINT3_EN,
  DEFAULT_JOINT4_EN,
  DEFAULT_JOINT5_EN,
  DEFAULT_JOINT6_EN
};

// Current encoder readings
float encoderAngles[NUM_ENCODERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// SG90 gripper servo (end effector)
Servo sg90Gripper;
int gripperTargetAngle = SG90_DEFAULT_ANGLE;  // Current target angle for gripper

// Joint 6 servo (25kg digital servo, open-loop control)
Servo joint6Servo;
float joint6LastCommandedAngle = 0.0;  // Track last commanded angle for telemetry

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

// Map each joint index (0-5) to its TCA9548A mux channel using config defines.
// This ensures all channel assignments are centralized in controls_config.h.
// NOTE: Joint 6 (index 5) is servo-controlled and has no encoder.
// Channel 7 is reserved but not used - set to 0 as placeholder.
const uint8_t jointMuxChannels[NUM_ENCODERS] = {
  JOINT1_MUX_CH,
  JOINT2_MUX_CH,
  JOINT3_MUX_CH,
  JOINT4_MUX_CH,
  JOINT5_MUX_CH,
  0
  //JOINT6_MUX_CH
};

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
// Only read encoders for joints that are enabled
// NOTE: Joint 6 (index 5) uses a servo with no encoder - always skip it
void readEncoders() {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    // Joint 6 (index 5) is servo-controlled with no encoder - skip entirely
    // Do not attempt I2C communication on mux channel 7 for Joint 6
    if (i == 5) {
      encoderAngles[i] = joint6LastCommandedAngle;  // Use last commanded angle
      continue;
    }
    
    // Skip reading disabled joints to avoid hanging on unconnected encoders
    if (!jointEnabled[i]) {
      encoderAngles[i] = 0.0;  // Set to 0 for disabled joints
      continue;
    }
    
    // Look up the TCA9548A channel for this joint from config
    uint8_t channel = jointMuxChannels[i];

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
  for (int i = 0; i < NUM_ENCODERS; i++) {
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

// Parse packet and extract values into global state variables
void parseAndExecutePacket(const String& packet) {
  if (!packet.startsWith("TYPE=CMD")) return;
  
  // Extract CMD name
  int cmdStart = packet.indexOf("CMD=");
  if (cmdStart == -1) return;
  int cmdEnd = packet.indexOf(',', cmdStart);
  if (cmdEnd == -1) cmdEnd = packet.indexOf('\n', cmdStart);
  if (cmdEnd == -1) cmdEnd = packet.length();
  
  String cmd = packet.substring(cmdStart + 4, cmdEnd);
  
  // ========== Extract values based on command type ==========
  
  if (cmd == CMD_SET_MODE) {
    // Extract MODE parameter
    int modePos = packet.indexOf("MODE=");
    if (modePos == -1) return;
    int modeEnd = packet.indexOf(',', modePos);
    if (modeEnd == -1) modeEnd = packet.indexOf('\n', modePos);
    if (modeEnd == -1) modeEnd = packet.length();
    
    int newMode = packet.substring(modePos + 5, modeEnd).toInt();
    if (newMode >= 0 && newMode <= 3) {
      currentMode = newMode;
      sendAck(packet);
    }
  }
  
  else if (cmd == CMD_JOINTS_TO_ANGLE) {
    // Extract JOINT_n_ANG parameters
    for (int i = 1; i <= NUM_JOINTS; i++) {
      String keyName = "JOINT_" + String(i) + "_ANG=";
      int pos = packet.indexOf(keyName);
      if (pos != -1) {
        int end = packet.indexOf(',', pos);
        if (end == -1) end = packet.indexOf('\n', pos);
        if (end == -1) end = packet.length();
        
        float angle = packet.substring(pos + keyName.length(), end).toFloat();
        jointTargetAngles[i - 1] = angle;
      }
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_JOINT_EN) {
    // Extract JOINT_n_EN parameters
    for (int i = 1; i <= NUM_JOINTS; i++) {
      String keyName = "JOINT_" + String(i) + "_EN=";
      int pos = packet.indexOf(keyName);
      if (pos != -1) {
        int end = packet.indexOf(',', pos);
        if (end == -1) end = packet.indexOf('\n', pos);
        if (end == -1) end = packet.length();
        
        int enabled = packet.substring(pos + keyName.length(), end).toInt();
        jointEnabled[i - 1] = (enabled == 1);
      }
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_ESTOP) {
    // Emergency stop - disable all joints
    for (int i = 0; i < NUM_JOINTS; i++) {
      jointEnabled[i] = false;
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_CALIBRATE_JOINT) {
    // Extract JOINT_ID parameter
    int idPos = packet.indexOf("JOINT_ID=");
    if (idPos != -1) {
      int idEnd = packet.indexOf(',', idPos);
      if (idEnd == -1) idEnd = packet.indexOf('\n', idPos);
      if (idEnd == -1) idEnd = packet.length();
      
      // TODO: Implement calibration logic for joint (1-6)
    }
    sendAck(packet);
  }
  
  else if (cmd == CMD_GRIP_CNTL) {
    // Extract GRIP_ANGLE parameter and move SG90 gripper servo
    int anglePos = packet.indexOf("GRIP_ANGLE=");
    if (anglePos != -1) {
      int angleEnd = packet.indexOf(',', anglePos);
      if (angleEnd == -1) angleEnd = packet.indexOf('\n', anglePos);
      if (angleEnd == -1) angleEnd = packet.length();
      
      int angle = packet.substring(anglePos + 11, angleEnd).toInt();
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
// ============================================================================

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

// MODE 2: MOVE - Execute movement commands
void modeMoveCode() {
  // Move each enabled joint to target angle
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (jointEnabled[i]) {
      // Joint 6 (index 5) is servo-controlled
      if (i == 5) {
        // Direct servo control - range 60-180° is within Servo.write() limits
        int servoAngle = constrain((int)jointTargetAngles[5], 
                                   JOINT6_SERVO_MIN_ANGLE, 
                                   JOINT6_SERVO_MAX_ANGLE);
        joint6Servo.write(servoAngle);
        joint6LastCommandedAngle = jointTargetAngles[5];
      } else {
        // TODO: Write target angle to stepper motor controller
        // stepper[i].moveTo(jointTargetAngles[i]);
      }
    }
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
  // Uses JOINT6_SERVO_pin (Pin 29) - range 60-180° within Servo.write() limits
  joint6Servo.attach(JOINT6_SERVO_pin);
  joint6Servo.write(JOINT6_SERVO_DEFAULT_ANGLE);  // Start at 120°
  joint6LastCommandedAngle = JOINT6_SERVO_DEFAULT_ANGLE;
  Serial.println("Joint 6 servo initialized at 120 deg default position");

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
