#include "Wire.h"
#include <ESP32Servo.h>

#define I2C_ADDR 0x20
#define NUM_SERVOS 3

#define SERVO0_PIN 2
#define SERVO1_PIN 4
#define SERVO2_PIN 5

Servo servo0;
Servo servo1;
Servo servo2;

int current_angles[NUM_SERVOS] = {90, 90, 90};
int angle_targets[NUM_SERVOS] = {90, 90, 90};
bool update_flags[NUM_SERVOS] = {false, false, false};

void receiveEvent(int len) {
  while (Wire.available() >= 3) {
    uint8_t channel = Wire.read();
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();

    if (channel < NUM_SERVOS) {
      int16_t value = (high << 8) | low; // -1000 to +1000
      int angle = map(value, -1000, 1000, 0, 180);
      angle = constrain(angle, 0, 180);

      angle_targets[channel] = angle;
      update_flags[channel] = true;
    }
  }
}

void requestEvent() {
  for (int i = 0; i < NUM_SERVOS; ++i) {
    int16_t pos = current_angles[i];
    Wire.write((uint8_t)(pos >> 8));
    Wire.write((uint8_t)(pos & 0xFF));
  }
}

void moveServo(Servo& servo, int index) {
  if (update_flags[index]) {
    update_flags[index] = false;
    int angle = angle_targets[index];
    servo.write(angle);
    current_angles[index] = angle;
  }
}

void setup() {
  // Attach servos
  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Move to initial positions
  servo0.write(current_angles[0]);
  servo1.write(current_angles[1]);
  servo2.write(current_angles[2]);

  // Start I2C and register callbacks
  Wire.begin((uint8_t)I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  moveServo(servo0, 0);
  moveServo(servo1, 1);
  moveServo(servo2, 2);
}
