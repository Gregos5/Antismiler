#include <Servo.h>
#include <Arduino.h>
#include "Servo_arm.h"


// Setup
void Arm::setUp(void) {
  Arm::myservo1.attach(pinSer1);
  Arm::myservo2.attach(pinSer2);
  
  Arm::myservo1.write(0); 
  Arm::myservo2.write(0); 
}

void Arm::set_pos(int xpos, int ypos) {
  Arm::myservo1.write(xpos);
  Arm::myservo2.write(ypos);
}

// Get measurement of the servo motor (in degrees)
int Arm::get_pos_analog(void) {
  return Arm::myservo1.read();
}
