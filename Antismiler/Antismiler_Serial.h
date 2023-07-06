#ifndef Antismiler_Serial_H
#define Antismiler_Serial_H

#include <Arduino.h>

enum operation {
  PUMP,
  VALVE,
  ARM,
  DETAIL,
};



class ASerial {
  private:
    //Flushes the serial port of any unwanted data
    void serialFlush();

    //Checks to see if the receiver ID matches that of the device
    int CheckrID(String rID, int Device_ID);

    //Checks to see if the sender ID matches that of the listener
    int ChecksID(String sID, int Sender_ID);

    //Calculates the command packet size from the serial comms.
    int getPKSize(String rPK_Size);

    void ReturnDetails();

    int Device_ID;
    int Sender_ID;
    String sACK;
    String sBUSY;
    String DeviceDesc;

    int NumPump;
    int NumValve;
    int NumArm;
    int intPin;
    int ResPin;


    enum operation op;

    int pump;
    float pumpValue;
    int pumpDir;

    int valve;
    int valveState;

    int arm;
    int xPOS;
    int yPOS;

    void Pump();
    void Valve();
    void Arm();

    static ASerial * instance0_;

    String Command = "NaN";
    bool isWaiting;
    bool CMD = false;

    String rID = "NaN";
    String sID = "NaN";
    String rPK_Size = "NaN";

    String Data;

    static void serialInterrupt();
    static void reattach();
    void SerialLoop();

    int process();
    void analyse();

  public:
    ASerial(String DD, int P, int V, int A, int Res);

    //Error handler
    void Error(int code);

    void Start();

    void print2Serial();
    void FinishedCommand();

    void SetState(bool s);
    bool GetState();

    int GetCommand();
    String GetData() {
      return Data;
    }
    bool GotCommand();
    void SetCommand(bool v) {
      CMD = v;
    }

    int getPump();
    float getPumpMls();
    bool getPumpDir();

    int getValve();
    int getValveState();
    
    int getArm();
    int getArmxPos();
    int getArmyPos();
};
#endif
