#include "Antismiler_Serial.h"
#include "Arduino.h"
ASerial::ASerial(String DD, int P, int V, int A, int Res) {
  ResPin = Res;
  digitalWrite(ResPin, HIGH);
  pinMode(ResPin, OUTPUT);
  DeviceDesc = DD;
  sACK = "[ACK]";
  sBUSY = "[BUSY]";
  NumPump = P;
  NumValve = V;
  NumArm = A;
  isWaiting = true;
  instance0_ = this;
}

ASerial * ASerial::instance0_;

void ASerial::Start() {
}

void ASerial::serialFlush() {
  int i = 0;
  while (Serial.available() > 0 && i < 500) {
    char t = Serial.read();
    i++;
  }
}

void ASerial::ReturnDetails() {
  Serial.println(" P" + (String)NumPump +
                 " V" + (String)NumValve +
                 " A" + (String)NumArm +
                 " (" + DeviceDesc + ")]");

}



void ASerial::Error(int code) {
  Serial.println("[ERR" + (String)code + "]");
}

bool ASerial::GotCommand(){
  if(Serial.available()>0){
    Serial.println(instance0_->sACK);
    if (Serial.peek() == 'K') {
      Serial.println("KILL");
      digitalWrite(instance0_->ResPin, LOW);
    }
    return true;
  }
  else{
    return false;
  }
}

int ASerial::process() {
  if (Serial.available() > 0) {
    Command = Serial.readStringUntil("]");
    if (Command[0] == '['){
      Data=Command;
      Serial.flush();
      serialFlush();
      return 1;
    }
    else {
      Error(0);
      serialFlush();
      return -1;
    }
  }
  return -1;
}

void ASerial::analyse() {
  switch (Command[1]) {
    case 'P':
      op = PUMP;
      Pump();
      break;
    case 'V':
      op = VALVE;
      Valve();
      break;
    case 'A':
      op = ARM;
      Arm();
      break;
    case 'D':
      op = DETAIL;
      ReturnDetails();
      break;
    default:
      op = NONE;
      break;
  }
}

String readStringuntil(String s, char c) {
  int i = 0;
  String r = "";
  //Serial.println(sizeof(s));

  while (i < s.length()) {
    if (s[i - 1] == c) {
      //Serial.print(r);
      return r;
    }
    else {
      r = r + s[i];
      //Serial.println(r[i]);
    }
    i++;
  }
  return r;
}

void ASerial::Pump() {
  String rubbish;
  pump = Command[0] - '0';
  // Serial.println(pump);
  rubbish = readStringuntil(Command, 'm');
  Command.remove(0, rubbish.length());
  pumpValue = readStringuntil(Command, ' ').toFloat();
  // Serial.println(pumpValue);
}

void ASerial::Valve() {
  String rubbish;
  valve = Command[1] - '0';
  Serial.println(valve);
  rubbish = readStringuntil(Command, 'S');
  Command.remove(0, rubbish.length());
  valveState = Command[0] - '0';
  Serial.println(valveState);
}


void ASerial::Arm() {
  String rubbish;
  arm = Command[1] - '0';
  // Serial.println(arm);
  
  rubbish = readStringuntil(Command, 'X');
  Command.remove(0, rubbish.length());
  xPOS = readStringuntil(Command, ' ').toInt();
  // Serial.println(xPOS);
  
  rubbish = readStringuntil(Command, 'Y');
  Command.remove(0, rubbish.length());
  yPOS = readStringuntil(Command, ' ').toInt();
  // Serial.println(yPOS);
}


void ASerial::print2Serial() {
  Serial.print("The decoded Command is: ");
  Serial.println((String)GetData());
  Serial.println();
}
bool ASerial::GetState() {
  return isWaiting;
}
void ASerial::SetState(bool s) {
  isWaiting = s;
}
int ASerial::getPump() {
  return pump;
}
float ASerial::getPumpMls() {
  return pumpValue;
}
bool ASerial::getPumpDir() {
  return pumpDir;
}
int ASerial::getValve() {
  return (int)valve;
}
int ASerial::getValveState() {
  return valveState;
}
int ASerial::getArm() {
  return arm;
}
int ASerial::getArmxPos() {
  return xPOS;
}
int ASerial::getArmyPos() {
  return yPOS;
}
int ASerial::GetCommand() {
  int S = process();
  serialFlush();
  if (S == 1) {
    analyse();
    if (op != NONE){
      Serial.println("[VALID]");
    }
    else{
      Error(1);
    }
    return op;
  }
  return -1;
}

void ASerial::FinishedCommand() {
  isWaiting = true;
  CMD = false;
  Serial.println("[FREE]");
  serialFlush();
}
