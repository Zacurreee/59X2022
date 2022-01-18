#include "main.h"
double encdL = 0, encdR = 0, bearing = 0, angle = halfPI;

void Sensors(void * ignore){
  Motor LGB(LGPort);
	Motor CL(CLPort);
	Motor BL(BLPort);
	Motor RGB(RGPort);
	Motor CR(CRPort);
	Motor BR(BRPort);
  Imu Inertial (ImuPort);
  while(true){
    if(!Inertial.is_calibrating()){
      encdL = CL.get_position();
      encdR = CR.get_position();
      bearing = Inertial.get_rotation();
      angle = halfPI - bearing * toRad;
    }
    delay(5);
  }
}
