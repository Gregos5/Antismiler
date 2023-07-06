#include "Pump.h"
#include "Servo.h"

// Setup
void Pump::setUp(void) {
  if (pump_type == 0) {
    serPump.attach(pinNum);
    serPump.write(Stop);
  }
  if (pump_type == 1) {
    pinMode(dirStep, OUTPUT);
    pinMode(Step, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(EN, LOW);
  }
} // End function



// Set direction and volume of pump
void Pump::set_vol(float vol, bool direc) {
  volume = abs(vol);
  dir = direc;
  Serial.println(volume);
  if (pump_type == 0) { // SERVO
    float mlpersec = (0.0038 * volume) + 0.5967;
    uint32_t pumping_time = (volume / mlpersec) * 1000;
    Serial.println(mlpersec);
    Serial.println(pumping_time);
    flowrate = mlpersec;
    if (direc == 1) {
      serPump.write(550); //Clockwise maximum speed rotation (0 degrees)
      delay(pumping_time);
    }
    if (direc == 0) {
      serPump.write(2500); //Anti clockwise maximum speed rotation (0 degrees)
      delay(pumping_time);
    }
    serPump.write(Stop);
  }// End if servo

  if (pump_type == 1) { // STEPPER
    digitalWrite(dirStep, direc);
    for (uint32_t i = 0; i < vol * steps_per_ml; i++) {
      digitalWrite(Step, HIGH);
      delay(1);
      digitalWrite(Step, LOW);
      delay(1);
    }
  }// End if stepper

}// End function



// Stop pumping
void Pump::stop_pumping(void) {
  if (pump_type == 0) { // Servo
    serPump.write(Stop);
  }
  if (pump_type == 1) { // Stepper
    digitalWrite(Step, LOW);
  }
}// End function



// Set direction of pumping (no actual pumping); 1 = clockwise; 0 = anticlockwise
void Pump::set_dir(bool Direction) {
  dir = Direction;
}



// Get measurement for direction (0 for into reactor, 1 for reverse)
bool Pump::get_last_dir() {
  return Pump::dir;
}// End function



// Get measurement for volume pumped (in ml)
double Pump::get_last_vol() {
  return Pump::volume;
}// End function



// Get measurement for flowrate of SERVO (in ml per seconds)
double Pump::get_flow_rate() {
  return Pump::flowrate;
}// End function



// Set type of pump - SERVO = 0; STEPPER = 1
void Pump::set_pump_type(bool type_of_pump) {
  pump_type = type_of_pump;
}// End function



// Get pump type - SERVO = 0; STEPPER = 1
bool Pump::get_pump_type(void) {
  return Pump::pump_type;
}



// Set convertion ratio for servo pumps (for calibration)
void Pump::set_conv_ratio(double convertion) {
  if (pump_type == 0) {
    conv_ratio = convertion;
    //?????????????????????????????????????????????????????????????????????????????
    // Is it the constant 0.0052 in set_vol?
  }
}// End function



// Get convertion ratio (servo)
double Pump::get_conv_ratio(void) {
  if (pump_type == 0) {
    return conv_ratio;
  }
  else return 0;
}// End function



// Set the number of steps per ml in stepper pump (for calibration)
void Pump::set_steps_per_ml(double num_steps) {
  if (pump_type == 1) {
    steps_per_ml = num_steps;
  }
}// End function



// Get number of steps per ml (stepper)
double Pump::get_steps_per_nl(void) {
  if (pump_type == 0) {
    return steps_per_ml;
  }
  else return 0;
}// End function
