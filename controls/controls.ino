#include <Wire.h>
#include "controls_config.h"
#include <AS5600.h>

// Use the AS5600 library's API (readAngle()) and the TCA9548A mux
AS5600 encoder;

// Conversion: AS5600 raw (0..4095) to degrees
const float RAW_TO_DEG = 360.0 / 4096.0; // 0.087890625
const float NO_READ = -1000.0; // sentinel for no device present on mux channel

// Select which channel on the TCA9548A to use (0-7)
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read all encoder positions and return as array
float* get_encoder_positions() {
    /*
    How mux reading works:
    Begin transmission with the TCA9548A
    Send bitmask to mux (channel = 0, then 1 << 0 -> 00000001, channel = 1 -> 00000010, etc )
    End transmission
    */
  static float positions[NUM_ENCODERS];

  for (int i = 0; i < NUM_ENCODERS; i++) {
    selectMuxChannel(i);
    delay(MUX_DELAY); // Small delay for mux switching

    // AS5600::readAngle() typically returns a raw value (0..4095)
    long raw = encoder.readAngle();
    positions[i] = raw * RAW_TO_DEG;
  }

  // Deselect all channels
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  return positions;
}

// Print encoder positions to serial monitor
void print_encoder_positions() {
  float* positions = get_encoder_positions();

  Serial.print("Encoders: ");
  for (int i = 0; i < NUM_ENCODERS; i++) {
    Serial.print("E");
    Serial.print(i);
    Serial.print("=");
    Serial.print(positions[i], 2);
    Serial.print("° ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  // Initialize AS5600 library
  encoder.begin();

  delay(1000);
  Serial.println("Controls: Multi-Encoder Reader");
}

void loop() {
  print_encoder_positions();
  delay(100); // Read every 100ms
}
