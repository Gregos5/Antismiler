#ifndef Valve_H
#define Valve_H
#include <Servo.h>

class Valve{
  private:
    int posOpen = 0;
    int posClose = 1; 
    int posMid = 2;
    int a = 0;
    int angleMid = 90;
    int angleOpen;
    int angleClose;  
    int pinSer;  // pin number (should be 2 for the servo)
    Servo myservo;


    
  public:
// Constructor and initial settings
    Valve(int pinServo, int posOpen, int posClose):pinSer(pinServo), angleOpen(posOpen), angleClose(posClose){}



// Setup
    void setUp();

// Set Valve position to 0 (open), 1 (closed), or 2 (middle)
    void set_pos(int pos);

// Set valve position to 1
    void set_pos1(void);


// Set valve position to 0
    void set_pos0(void);



// Toggle valve position
    void toggle_pos(void);
    


// Set the limit position of the servo motor (in degrees)
    void set_angle_lims(int angle0, int angle1);



// Get measurement of the servo motor (in degrees)
    int get_pos_analog(void);


// Get state of valve (0 or 1)
    int get_pos_digital(void);



// Get measurement of the limit set for the servo motor (in degrees)
    int get_lim(int pos);
};

#endif
