#include <Wire.h>
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
bool jointEnabled[NUM_JOINTS] = {true, true, true, true, true, true};

// Current encoder readings
float encoderAngles[NUM_ENCODERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Blink pattern timing
unsigned long lastBlinkMs = 0;
bool ledState = false;

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

// Blink function: patterns depend on current mode
void blinkPattern() {
  unsigned long now = millis();
  
  if (currentMode == MODE_IDLE) {
    // Mode 0: No blinking - keep LED off
    digitalWrite(LED_pin, LOW);
    ledState = false;
  }
  else if (currentMode == MODE_CALIBRATION) {
    // Mode 1: Slow blink (once every 3 sec)
    if ((now - lastBlinkMs) >= 3000) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_pin, ledState ? HIGH : LOW);
    }
  }
  else if (currentMode == MODE_MOVE) {
    // Mode 2: Very fast blink (100ms on/off)
    if ((now - lastBlinkMs) >= 100) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_pin, ledState ? HIGH : LOW);
    }
  }
}

// Use the AS5600 library's API (readAngle()) and the TCA9548A mux
AS5600 encoder;

// Conversion: AS5600 raw (0..4095) to degrees
const float RAW_TO_DEG = 360.0 / 4096.0; // 0.087890625

// Select which channel on the TCA9548A to use (0-7)
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read all encoder positions and update encoderAngles[]
void readEncoders() {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    selectMuxChannel(i);
    delay(MUX_DELAY);
    
    long raw = encoder.readAngle();
    encoderAngles[i] = raw * RAW_TO_DEG;
  }
  
  // Deselect all channels
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// Send telemetry packet: TYPE=DATA,CMD=JOINT_ANGLES,ENCODER_1_ANGLE=X.X,...,BUTTON=N\n
void sendTelemetry() {
  // Only send telemetry in CALIBRATION (1) or MOVE (2) modes
  if (currentMode != MODE_CALIBRATION && currentMode != MODE_MOVE) {
    return;
  }
  
  Serial.print("TYPE=DATA,CMD=JOINT_ANGLES");
  for (int i = 0; i < NUM_ENCODERS; i++) {
    Serial.print(",ENCODER_");
    Serial.print(i + 1);
    Serial.print("_ANGLE=");
    Serial.print(encoderAngles[i], 1);
  }
  // Add button state
  Serial.print(",BUTTON=");
  Serial.print(readButton());
  Serial.println();
}

// Send ACK packet: TYPE=ACK,CMD=<original_cmd>,...\n
void sendAck(const String& originalPacket) {
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
      digitalWrite(LED_pin, HIGH);
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
    digitalWrite(LED_pin, HIGH);
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
    digitalWrite(LED_pin, HIGH);
  }
  
  else if (cmd == CMD_ESTOP) {
    // Emergency stop - disable all joints
    for (int i = 0; i < NUM_JOINTS; i++) {
      jointEnabled[i] = false;
    }
    sendAck(packet);
    digitalWrite(LED_pin, HIGH);
  }
  
  else if (cmd == CMD_CALIBRATE_JOINT) {
    // Extract JOINT_ID parameter
    int idPos = packet.indexOf("JOINT_ID=");
    if (idPos != -1) {
      int idEnd = packet.indexOf(',', idPos);
      if (idEnd == -1) idEnd = packet.indexOf('\n', idPos);
      if (idEnd == -1) idEnd = packet.length();
      
      int jointId = packet.substring(idPos + 9, idEnd).toInt();
      // TODO: Implement calibration logic for joint (1-6)
    }
    sendAck(packet);
    digitalWrite(LED_pin, HIGH);
  }
}

// ============================================================================
// MODE-SPECIFIC CODE
// ============================================================================

// MODE 0: IDLE - System initialization, safe state
void modeIdleCode() {
  // Disable all joints in idle mode
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointEnabled[i] = false;
  }
  // Blink pattern for IDLE (none)
  blinkPattern();
  // TODO: Add initialization/safety code
}

// MODE 1: CALIBRATION - Run calibration procedures
void modeCalibrationCode() {
  // Blink pattern for CALIBRATION (slow: once every 3 sec)
  blinkPattern();
  // TODO: Implement calibration procedures
  // - Move individual joints to home position
  // - Store encoder offsets
  // - Verify encoder readings
}

// MODE 2: MOVE - Execute movement commands
void modeMoveCode() {
  // Blink pattern for MOVE (very fast: 100ms on/off)
  blinkPattern();
  // Move each enabled joint to target angle
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (jointEnabled[i]) {
      // TODO: Write target angle to servo/motor controller
      // servo[i].write(jointTargetAngles[i]);
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
  // Initialize button pin with internal pullup (active low)
  pinMode(PUSH_SWITCH_pin, INPUT_PULLUP);
  
  Wire.begin();

  // Initialize AS5600 library
  encoder.begin();

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, LOW);
  
  currentMode = MODE_IDLE;
}

void loop() {
  // 1) Listen for incoming packets and extract values into state variables
  listenPackets();
  
  // 2) Read encoder positions and send telemetry (only in modes 1 and 2)
  unsigned long now = millis();
  if ((now - lastTelemMs) >= TELEM_INTERVAL_MS) {
    lastTelemMs = now;
    // Only read encoders and send telemetry in CALIBRATION or MOVE modes
    if (currentMode == MODE_CALIBRATION || currentMode == MODE_MOVE) {
      readEncoders();
      sendTelemetry();
    }
  }
  
  // 4) Execute mode-specific behavior
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
}
