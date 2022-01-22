#include "main.h"
#include "mech_lib.hpp"

// driver prgrammes
const double armHeights[] = {0,100,400,700};
double armTarg = armHeights[0], armKP = 1;
bool tiltstate = true;
bool armstate = true;
bool armDefault = true;
bool scoring = false;
void armControl(void*ignore) {
  Motor LA(LAPort);
  Motor RA(RAPort);
  ADIDigitalOut armClamp(armClampPort);
  LA.tare_position();
  while(true) {
    double armError = armTarg - LA.get_position();
    LA.move(armError*armKP);
    RA.move(armError*armKP);

    if(armstate){
      armClamp.set_value(LOW);
    } else {
      armClamp.set_value(HIGH);
    }

    if(armDefault){
      setArmPos(0);
    } else {
      setArmPos(3);
    }

    if (armDefault && !armstate){
      setArmPos(1);
    }

    if (scoring && !armDefault){
      setArmPos(2);
    }
  }
}

void tiltControl(void*ignore){
  ADIDigitalOut tilt(tiltPort);
	ADIDigitalOut tiltClamp(tiltClampPort);

  while(true){
    if(tiltstate){
      tilt.set_value(HIGH);
      delay (1000);
      tiltClamp.set_value(LOW);
    } else {
      tiltClamp.set_value(LOW);
      tilt.set_value(LOW);
      delay(1000);
      tiltClamp.set_value(HIGH);
    }
  }
}

void setArmPos(int pos) {armTarg = armHeights[pos];}
void armtiltSwitch(){armstate = !armstate;}
void tiltSwitch (){tiltstate = !tiltstate;}
void armState(){armDefault = !armDefault;}
void scorestate(){scoring = !scoring;}
