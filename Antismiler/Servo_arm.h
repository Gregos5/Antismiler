#ifndef Servo_arm_H
#define Servo_arm_H
#include <Servo.h>

class Arm{
  private:
    int pinSer1;  // pin number (should be 2 for the servo)
    int pinSer2;
    Servo myservo1;
    Servo myservo2;
  public:
// Constructor and initial settings
    Arm(int pinServo1, int pinServo2): pinSer1(pinServo1), pinSer2(pinServo2){}

// Setup
    void setUp();

// Set Extract position to int 0 to 180
    void set_pos(int xpos, int ypos);

    
// Get measurement of the servo motor (in degrees)
    int get_pos_analog(void);
};

#endif
