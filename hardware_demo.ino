#include <ESP32Servo.h>
Servo a0_a1; 
Servo a1_a2;
Servo a2_a3;

int zero1 = 90;
int zero2 = 90;
int zero3 = 30;

int p = 0;    // variable to store the servo position

void moveOne(Servo &servo, int pos0, int pos1, int zero) {
  if (pos0 <= pos1) {
    for (p = pos0; p <= pos1; p += 1) {
      servo.write(p + zero);
      delay(15);
    }
  }
  else {
    for (p = pos0; p >= pos1; p -= 1) {
      servo.write(p + zero);
      delay(15);
    }
  }
}

void moveThree(Servo &servo1, Servo &servo2, Servo &servo3, int pos0, int pos1) {
  if (pos0 <= pos1) {
    for (p = pos0; p <= pos1; p += 1) {
      servo1.write(p+zero1);
      servo2.write(p+zero2);
      servo3.write(p+zero3);
      delay(15);
    }
  }
  else {
    for (p = pos0; p >= pos1; p -= 1) {
      servo1.write(p+zero1);
      servo2.write(p+zero2);
      servo3.write(p+zero3);
      delay(15);
    }
  }
}

void setup() {
  a0_a1.attach(5);
  a1_a2.attach(4);
  a2_a3.attach(2);
}

void loop() {
  moveOne(a0_a1, 0, 120, zero1);
  delay(1000);
  moveOne(a1_a2, 0, 120, zero2);
  delay(1000);
  moveOne(a2_a3, 0, 120, zero3);
  delay(8000);
  moveThree(a0_a1, a1_a2, a2_a3, 120, 0);
  delay(5000);
}
