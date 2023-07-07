#include "Antismiler_Serial.h"
#include "Servo_arm.h"
#include "Pump.h"
#include "Valve.h"

String DeviceDesc = "Module Antismiler";
int Num_of_Pumps = 1;
int Num_of_Valves = 1;
int Num_of_Arms = 1;
int ResetPin = 3;
int PumpPin = 5;
int ArmPin1 = 6;
int ArmPin2 = 7;
int ValvePin = 8;
int ValveClose = 65;
int ValveOpen = 127;

ASerial Device(DeviceDesc, Num_of_Pumps, Num_of_Valves, Num_of_Arms, ResetPin);

Pump Pump1(PumpPin);
Arm Arm1(ArmPin1, ArmPin2);
Valve Valve1(ValvePin, ValveClose, ValveOpen);

void setup() {
  Serial.begin(115200);
  Device.Start();
  Valve1.setUp();
  Pump1.setUp();
  Arm1.setUp();
}

void loop() {
  if (Device.GotCommand()) {
    switch (Device.GetCommand()) {
      case PUMP: 
        Pump1.set_vol(Device.getPumpMls(),Device.getPumpDir());
        Serial.println("pump number: " + (String)Device.getPump());
        Serial.println("pump volume: " + (String)Device.getPumpMls());
        break;
      case VALVE:
        Valve1.set_pos(Device.getValveState());
        Serial.println("valve number: " + (String)Device.getValve());
        Serial.println("valve state:  " + (String)Device.getValveState());
        break;
      case ARM: //[A1 X12 Y33]
        Arm1.set_pos(Device.getArmxPos(), Device.getArmyPos());
        Serial.println("Arm number: " + (String)Device.getArm());
        Serial.println("X position:  " + (String)Device.getArmxPos());
        Serial.println("Y position:  " + (String)Device.getArmyPos());
        break;
      default:
        break;
    }
    Device.FinishedCommand();
    delay(20);
    
  }
}
